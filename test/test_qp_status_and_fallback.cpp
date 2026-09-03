// C1: QP status -> deterministic fallback tests.
//
// Two layers:
//   1) pure policy tests (decideFallback/safeCommand) over synthetic
//      SolverDiagnostics covering solved / approximate / infeasible /
//      timeout / NaN-candidate;
//   2) one integration test building a truly infeasible QP through the
//      OSQPSolver wrapper and asserting the controller decision becomes a
//      zero command (never an unvalidated solution).
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <vector>

#include "mpc_controller/osqp_solver.hpp"
#include "mpc_controller/fallback_policy.hpp"

namespace mpc_controller
{

namespace
{

SolverDiagnostics diag(bool solved, bool approximate, int status)
{
  SolverDiagnostics d;
  d.solved = solved;
  d.solved_approximate = approximate;
  d.status = status;
  return d;
}

Eigen::VectorXd v(const std::vector<double> & x)
{
  Eigen::VectorXd out(static_cast<int>(x.size()));
  for (size_t i = 0; i < x.size(); ++i) {
    out(static_cast<int>(i)) = x[i];
  }
  return out;
}

}  // namespace

TEST(FallbackPolicy, SolvedAppliesSolution)
{
  const auto d = decideFallback(diag(true, false, 1), v({0.5, -0.3}));
  EXPECT_EQ(d.action, FallbackAction::kApplySolution);
  const auto cmd = safeCommand(d, v({0.5, -0.3}));
  ASSERT_EQ(cmd.size(), 2);
  EXPECT_DOUBLE_EQ(cmd(0), 0.5);
  EXPECT_DOUBLE_EQ(cmd(1), -0.3);
}

TEST(FallbackPolicy, ApproximateAppliesButFlags)
{
  const auto d = decideFallback(diag(false, true, -2), v({0.1, 0.2}));
  EXPECT_EQ(d.action, FallbackAction::kApplyApproximate);
  EXPECT_TRUE(d.approximate);
  const auto cmd = safeCommand(d, v({0.1, 0.2}));
  EXPECT_DOUBLE_EQ(cmd(0), 0.1);
}

TEST(FallbackPolicy, InfeasibleYieldsZeroNeverSolution)
{
  // OSQP_PRIMAL_INFEASIBLE (-3)
  const auto d = decideFallback(diag(false, false, -3), v({0.7, -0.9}));
  EXPECT_EQ(d.action, FallbackAction::kZeroCommand);
  const auto cmd = safeCommand(d, v({0.7, -0.9}));
  ASSERT_EQ(cmd.size(), 2);
  EXPECT_DOUBLE_EQ(cmd(0), 0.0);
  EXPECT_DOUBLE_EQ(cmd(1), 0.0);
}

TEST(FallbackPolicy, TimeoutWithBadResidualsYieldsZero)
{
  // kMaxIter reached but residuals NOT acceptable -> not approximate
  const auto d = decideFallback(diag(false, false, -2), v({1.0, 1.0}));
  EXPECT_EQ(d.action, FallbackAction::kZeroCommand);
  EXPECT_TRUE(safeCommand(d, v({1.0, 1.0})).isZero(0.0));
}

TEST(FallbackPolicy, NanCandidateNeverApplied)
{
  // A "solved" QP returning NaN must be treated as failed.
  const auto d = decideFallback(diag(true, false, 1), v({std::nan(""), 0.0}));
  EXPECT_EQ(d.action, FallbackAction::kZeroCommand);
  EXPECT_FALSE(d.solution_finite);
  const auto cmd = safeCommand(d, v({std::nan(""), 0.0}));
  EXPECT_TRUE(cmd.allFinite());
}

TEST(FallbackPolicy, ZeroCommandDimensionMatches)
{
  const auto d = decideFallback(diag(false, false, -3), v({1.0, 2.0, 3.0}));
  EXPECT_EQ(safeCommand(d, v({1.0, 2.0, 3.0})).size(), 3);
}

// --- integration: a truly infeasible QP through the real OSQP wrapper ---

TEST(QpStatusIntegration, InfeasibleQpNeverYieldsCommand)
{
  OSQPSolver solver;
  solver.initialize(1, 1, 1);  // nx=1, nu=1, N=1 -> one decision variable
  solver.setSolverSettings(4000, 1e-6, 1e-6);

  Eigen::SparseMatrix<double> P(1, 1);
  P.coeffRef(0, 0) = 2.0;
  P.makeCompressed();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(1);
  // Canonical infeasible pair on one variable:  x >= 1  AND  x <= 0.
  Eigen::SparseMatrix<double> A(2, 1);
  A.coeffRef(0, 0) = 1.0;  // row 0: x
  A.coeffRef(1, 0) = 1.0;  // row 1: x
  A.makeCompressed();
  Eigen::VectorXd l(2);
  l << 1.0, -1e30;         // row 0: x >= 1
  Eigen::VectorXd u(2);
  u << 1e30, 0.0;          // row 1: x <= 0

  solver.setupProblem(P, q, A, l, u);
  const bool solved = solver.solve();
  const auto diags = solver.getDiagnostics();

  EXPECT_FALSE(solved)
    << "status=" << diags.status << " iter=" << diags.iterations
    << " pri=" << diags.pri_res << " dua=" << diags.dua_res
    << " objective=" << diags.objective
    << " solved_approx=" << diags.solved_approximate;
  EXPECT_EQ(diags.status, -3)
    << "status=" << diags.status << " iter=" << diags.iterations;

  const auto decision = decideFallback(diags, solver.getSolution());
  EXPECT_EQ(decision.action, FallbackAction::kZeroCommand);
  EXPECT_TRUE(safeCommand(decision, solver.getSolution()).isZero(0.0));
}

}  // namespace mpc_controller
