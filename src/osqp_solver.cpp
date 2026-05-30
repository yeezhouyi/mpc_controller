#include "mpc_controller/osqp_solver.hpp"

#include <chrono>
#include <stdexcept>
#include <string>

namespace mpc_controller
{

namespace {

void throwIfOsqpError(osqp::OSQPError error, const char * operation)
{
  if (error != osqp::OSQPError::kNoError) {
    throw std::runtime_error(
      std::string(operation) + " failed with OSQP error " +
      std::to_string(static_cast<int>(error)));
  }
}

}  // anonymous namespace

void OSQPSolver::initialize(int state_dim, int input_dim, int horizon)
{
  state_dim_ = state_dim;
  input_dim_ = input_dim;
  horizon_ = horizon;
  n_vars_ = input_dim * horizon;
  n_constraints_ = 0;
  workspace_initialized_ = false;
}

void OSQPSolver::setSolverSettings(int max_iter, double abs_tol, double rel_tol)
{
  max_iterations_ = max_iter;
  abs_tol_ = abs_tol;
  rel_tol_ = rel_tol;
  acceptable_primal_res_ = 10.0 * abs_tol_;
  acceptable_dual_res_ = 10.0 * rel_tol_;
}

void OSQPSolver::setupProblem(
  const Eigen::SparseMatrix<double> & P,
  const Eigen::VectorXd & q,
  const Eigen::SparseMatrix<double> & A_lin,
  const Eigen::VectorXd & l,
  const Eigen::VectorXd & u)
{
  n_constraints_ = A_lin.rows();
  n_vars_ = P.rows();

  // Convert to compressed sparse column format for OSQP
  Eigen::SparseMatrix<double> P_upper;
  P_upper = P.triangularView<Eigen::Upper>();

  instance_.problem_mat = P_upper;
  instance_.gradient = q;
  instance_.constraint_mat = A_lin;
  instance_.lower_bounds = l;
  instance_.upper_bounds = u;

  osqp::OSQPSettings settings;
  settings.verbose = false;
  settings.eps_abs = abs_tol_;
  settings.eps_rel = rel_tol_;
  settings.max_iter = max_iterations_;
  settings.warm_start = true;
  settings.polish = true;
  settings.adaptive_rho = true;

  auto t_start = std::chrono::steady_clock::now();
  auto status = solver_.Init(instance_, settings);
  auto t_end = std::chrono::steady_clock::now();

  setup_time_us_ =
    std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
  last_diag_.setup_time_us = setup_time_us_;

  if (status != osqp::OSQPError::kNoError) {
    throw std::runtime_error("OSQP initialization failed with error code " +
                             std::to_string(static_cast<int>(status)));
  }
  workspace_initialized_ = true;

  // Store initial solution for warm starting
  warm_start_primal_ = Eigen::VectorXd::Zero(n_vars_);
}

bool OSQPSolver::solve()
{
  if (!workspace_initialized_) return false;

  // Apply warm start with shifted previous solution (if available)
  // Skip warm start if not yet initialized (first solve starts from zero anyway)
  if (warm_start_primal_.size() == n_vars_ && warm_start_primal_.squaredNorm() > 0.0) {
    throwIfOsqpError(
      solver_.SetPrimalDualWarmStart(warm_start_primal_,
                                      Eigen::VectorXd::Zero(n_constraints_)),
      "SetPrimalDualWarmStart");
  }

  auto t_start = std::chrono::steady_clock::now();
  auto status = solver_.Solve();
  auto t_end = std::chrono::steady_clock::now();

  last_diag_.solve_time_us =
    std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();

  last_diag_.status = static_cast<int>(solver_.GetStatus());
  last_diag_.iterations = solver_.GetIterations();
  last_diag_.objective = solver_.GetObjective();
  last_diag_.pri_res = solver_.GetPrimalResidual();
  last_diag_.dua_res = solver_.GetDualResidual();
  c_int osqp_status = solver_.GetStatus();
  bool solver_ok = (status == osqp::OSQPError::kNoError);

  // OSQP 0.6.2 status codes (C API):
  //  1 = OSQP_SOLVED (residuals within eps_abs/eps_rel)
  //  2 = OSQP_SOLVED_INACCURATE (residuals within 10x tolerance)
  // -2 = OSQP_MAX_ITER_REACHED
  // -3 = OSQP_PRIMAL_INFEASIBLE
  // -4 = OSQP_DUAL_INFEASIBLE

  // Strict: only OSQP_SOLVED (1) counts as fully solved
  last_diag_.solved = solver_ok &&
    (osqp_status == static_cast<c_int>(osqp::OSQPSolver::Status::kSolved));

  // SOLVED_INACCURATE (2) or kMaxIter (-2) with acceptable residuals count as approximate
  last_diag_.solved_approximate = false;
  if (solver_ok &&
      (osqp_status == 2 ||  // OSQP_SOLVED_INACCURATE
       osqp_status == static_cast<c_int>(osqp::OSQPSolver::Status::kMaxIter))) {
    last_diag_.solved_approximate =
      last_diag_.pri_res < acceptable_primal_res_ &&
      last_diag_.dua_res < acceptable_dual_res_;
  }

  // Cache shifted solution for next warm start (use best available solution)
  // NOTE: only update warm start if the solver actually found a solution,
  // to avoid feeding garbage back as the initial guess on the next cycle.
  // The shift accounts for the receding horizon: drop the first block of
  // decision variables (u_0 and ε_0) and pad with zeros.
  if (last_diag_.solved || last_diag_.solved_approximate) {
    Eigen::VectorXd sol = solver_.GetPrimalSolution();
    if (sol.size() == n_vars_ && sol.allFinite()) {
      const int n_u_vars = input_dim_ * horizon_;
      const int n_slack_vars = n_vel_ * horizon_;

      warm_start_primal_.setZero();

      // Shift U block: keep u_1..u_{N-1}, drop u_0
      if (n_u_vars > input_dim_) {
        warm_start_primal_.segment(0, n_u_vars - input_dim_) =
          sol.segment(input_dim_, n_u_vars - input_dim_);
      }

      // Shift slack block: keep ε_1..ε_{N-1}, drop ε_0
      if (n_slack_vars > n_vel_) {
        warm_start_primal_.segment(n_u_vars, n_slack_vars - n_vel_) =
          sol.segment(n_u_vars + n_vel_, n_slack_vars - n_vel_);
      }
    } else {
      warm_start_primal_.setZero();
    }
  } else {
    // Reset warm start to zero on failure to avoid OSQP's internal warm
    // starting from an unconverged ADMM state, which can trigger false
    // infeasibility detection on subsequent solves.
    warm_start_primal_.setZero();
    throwIfOsqpError(
      solver_.SetPrimalDualWarmStart(
        warm_start_primal_, Eigen::VectorXd::Zero(n_constraints_)),
      "ResetWarmStart");
  }

  return last_diag_.solved;
}

Eigen::VectorXd OSQPSolver::getSolution() const
{
  return solver_.GetPrimalSolution();
}

SolverDiagnostics OSQPSolver::getDiagnostics() const
{
  return last_diag_;
}

void OSQPSolver::updateBounds(
  const Eigen::VectorXd & l,
  const Eigen::VectorXd & u)
{
  if (!workspace_initialized_) return;
  instance_.lower_bounds = l;
  instance_.upper_bounds = u;
  throwIfOsqpError(solver_.UpdateBounds(l, u), "UpdateBounds");
}

void OSQPSolver::updateGradient(const Eigen::VectorXd & q)
{
  if (!workspace_initialized_) return;
  instance_.gradient = q;
  throwIfOsqpError(solver_.UpdateGradient(q), "UpdateGradient");
}

void OSQPSolver::updateCostMatrix(const Eigen::SparseMatrix<double> & P)
{
  if (!workspace_initialized_) return;
  Eigen::SparseMatrix<double> P_upper;
  P_upper = P.triangularView<Eigen::Upper>();
  instance_.problem_mat = P_upper;
  throwIfOsqpError(solver_.UpdateP(P_upper), "UpdateP");
}

void OSQPSolver::setWarmStart(
  const Eigen::VectorXd & primal,
  const Eigen::VectorXd & dual)
{
  if (!workspace_initialized_) return;
  warm_start_primal_ = primal;
  solver_.SetPrimalDualWarmStart(primal, dual);
}

}  // namespace mpc_controller
