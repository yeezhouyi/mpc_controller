#ifndef MPC_CONTROLLER__SOLVER_BASE_HPP_
#define MPC_CONTROLLER__SOLVER_BASE_HPP_

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "mpc_controller/types.hpp"

namespace mpc_controller
{

/// Abstract base class for MPC QP solvers.
class SolverBase
{
public:
  SolverBase() = default;
  virtual ~SolverBase() = default;

  /// Configure the solver with problem dimensions.
  virtual void initialize(
    int state_dim, int input_dim, int horizon) = 0;

  /// One-time setup of the QP with fixed sparsity structure.
  /// Called in on_configure().  Subsequent cycles only call the update methods.
  /// @param P  Hessian matrix (n_vars x n_vars, sparse, upper-triangular)
  /// @param q  Gradient vector (n_vars)
  /// @param A_lin  Linear constraint matrix (n_constraints x n_vars, sparse)
  /// @param l  Lower bound vector (n_constraints)
  /// @param u  Upper bound vector (n_constraints)
  virtual void setupProblem(
    const Eigen::SparseMatrix<double> & P,
    const Eigen::VectorXd & q,
    const Eigen::SparseMatrix<double> & A_lin,
    const Eigen::VectorXd & l,
    const Eigen::VectorXd & u) = 0;

  /// Solve the QP.
  /// @return true if the solver converged to an optimal solution.
  virtual bool solve() = 0;

  /// Get the optimal solution vector.
  virtual Eigen::VectorXd getSolution() const = 0;

  /// Get solver diagnostics from the last solve.
  virtual SolverDiagnostics getDiagnostics() const = 0;

  /// Update the linear constraint bounds.
  virtual void updateBounds(
    const Eigen::VectorXd & l,
    const Eigen::VectorXd & u) = 0;

  /// Update the cost gradient.
  virtual void updateGradient(const Eigen::VectorXd & q) = 0;

  /// Update the Hessian matrix values (sparsity must be unchanged).
  /// Used for hot-reload of Q/R/S weights.
  virtual void updateCostMatrix(const Eigen::SparseMatrix<double> & P) = 0;

  /// Set warm-start primal and dual variables.
  virtual void setWarmStart(
    const Eigen::VectorXd & primal,
    const Eigen::VectorXd & dual) = 0;

  /// Configure solver tolerances and max iterations.
  virtual void setSolverSettings(int max_iter, double abs_tol, double rel_tol) = 0;

  /// Set the number of slack variables for warm-start shifting.
  /// Default implementation does nothing (no slack).
  virtual void setNumSlackVariables(int /*n_vel*/) {}
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__SOLVER_BASE_HPP_
