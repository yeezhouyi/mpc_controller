#ifndef MPC_CONTROLLER__OSQP_SOLVER_HPP_
#define MPC_CONTROLLER__OSQP_SOLVER_HPP_

#include "3rdparty/osqp++.h"
#include <Eigen/Dense>
#include "mpc_controller/solver_base.hpp"

namespace mpc_controller
{

/// OSQP-based QP solver for MPC.
///
/// Solves the condensed MPC QP:
///   min  1/2 z^T P z + q^T z
///   s.t. l <= A_lin z <= u
///
/// where z = [u_0, u_1, ..., u_{N-1}] is the stacked input sequence.
///
/// Workspace is set up once (setupProblem) and then updated per cycle
/// via updateGradient / updateBounds.  Hot-reload of Q/R/S weights
/// triggers updateCostMatrix.  This avoids repeated osqp_setup calls.
class OSQPSolver : public SolverBase
{
public:
  OSQPSolver() = default;
  ~OSQPSolver() override = default;

  void initialize(int state_dim, int input_dim, int horizon) override;

  void setupProblem(
    const Eigen::SparseMatrix<double> & P,
    const Eigen::VectorXd & q,
    const Eigen::SparseMatrix<double> & A_lin,
    const Eigen::VectorXd & l,
    const Eigen::VectorXd & u) override;

  bool solve() override;

  Eigen::VectorXd getSolution() const override;

  SolverDiagnostics getDiagnostics() const override;

  void updateBounds(
    const Eigen::VectorXd & l,
    const Eigen::VectorXd & u) override;

  void updateGradient(const Eigen::VectorXd & q) override;

  void updateCostMatrix(const Eigen::SparseMatrix<double> & P) override;

  void setWarmStart(
    const Eigen::VectorXd & primal,
    const Eigen::VectorXd & dual) override;

  void setSolverSettings(int max_iter, double abs_tol, double rel_tol) override;

  /// Set the number of slack variables for warm-start shifting.
  /// Default is 0 (no slack).
  void setNumSlackVariables(int n_vel) override { n_vel_ = n_vel; }

private:
  int state_dim_{0};
  int input_dim_{0};
  int horizon_{0};
  int n_vars_{0};
  int n_constraints_{0};
  int n_vel_{0};  // number of slack velocity variables per timestep

  osqp::OSQPInstance instance_;
  osqp::OSQPSolver solver_;

  // Solver settings (from YAML config)
  int max_iterations_{4000};
  double abs_tol_{1e-4};
  double rel_tol_{1e-4};
  double acceptable_primal_res_{1e-3};  // 10x abs_tol for approximate acceptance
  double acceptable_dual_res_{1e-3};    // 10x rel_tol

  // Cached for diagnostics
  SolverDiagnostics last_diag_;

  // Setup-time tracking (only once)
  int64_t setup_time_us_{0};
  bool workspace_initialized_{false};

  // Cached warm-start vector (shifted previous solution)
  Eigen::VectorXd warm_start_primal_;
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__OSQP_SOLVER_HPP_
