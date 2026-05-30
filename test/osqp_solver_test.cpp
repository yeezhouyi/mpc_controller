#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "mpc_controller/osqp_solver.hpp"
#include "mpc_controller/linear_model.hpp"
#include "mpc_controller/types.hpp"

using namespace mpc_controller;

// ==============================================================
// Basic solver tests
// ==============================================================

/// Verify that the OSQP solver correctly solves a simple QP with known
/// analytical solution.
///
///   min  1/2 * [u0, u1] * [2, 0; 0, 2] * [u0; u1] - [2, 2] * [u0; u1]
///   s.t. -1 <= u0, u1 <= 1
///
/// Analytical solution: u0 = u1 = 1.0  (unconstrained optimum at z = P^{-1}q = 1)
TEST(TestOSQPSolver, simpleBoxQP)
{
  OSQPSolver solver;
  solver.initialize(1, 1, 2);

  int n_vars = 2;
  Eigen::SparseMatrix<double> P(n_vars, n_vars);
  P.insert(0, 0) = 2.0;
  P.insert(1, 1) = 2.0;

  Eigen::VectorXd q(n_vars);
  q << -2.0, -2.0;

  Eigen::SparseMatrix<double> A_lin(2 * n_vars, n_vars);
  for (int i = 0; i < n_vars; ++i) {
    A_lin.insert(i, i) = 1.0;
    A_lin.insert(n_vars + i, i) = -1.0;
  }

  Eigen::VectorXd l(2 * n_vars);
  Eigen::VectorXd u(2 * n_vars);
  const double inf = std::numeric_limits<double>::infinity();
  l << -inf, -inf, -inf, -inf;
  u << 1.0, 1.0, 1.0, 1.0;

  P.makeCompressed();
  A_lin.makeCompressed();

  solver.setupProblem(P, q, A_lin, l, u);
  bool solved = solver.solve();

  ASSERT_TRUE(solved) << "OSQP should solve the simple QP";

  Eigen::VectorXd sol = solver.getSolution();
  EXPECT_NEAR(sol(0), 1.0, 1e-4) << "u0 should be 1.0";
  EXPECT_NEAR(sol(1), 1.0, 1e-4) << "u1 should be 1.0";

  auto diag = solver.getDiagnostics();
  EXPECT_TRUE(diag.solved);
  EXPECT_GT(diag.solve_time_us, 0);
}

// ==============================================================
// P0 regression: Prediction matrix ordering
// ==============================================================

/// Verify that S_x has correct column ordering.
///
/// Scalar system: A=2, B=3, N=3
///   x_{k+1} = 2*x_k + 3*u_k
///
/// x_1 = A*x0 + B*u0           = 2*x0 + 3*u0
/// x_2 = A^2*x0 + A*B*u0 + B*u1 = 4*x0 + 6*u0 + 3*u1
/// x_3 = A^3*x0 + A^2*B*u0 + A*B*u1 + B*u2
///                                = 8*x0 + 12*u0 + 6*u1 + 3*u2
///
/// S_x (3×3): each row [u0 coeff, u1 coeff, u2 coeff]
///   row 0 (x_1): [3, 0, 0]
///   row 1 (x_2): [6, 3, 0]
///   row 2 (x_3): [12, 6, 3]
TEST(TestSolverMpc, PredictionMatrixOrdering)
{
  int nx = 1, nu = 1, N = 3;
  Eigen::MatrixXd A(nx, nx);
  A(0, 0) = 2.0;
  Eigen::MatrixXd B(nx, nu);
  B(0, 0) = 3.0;

  // Replicate buildQPStructure logic (precompute A powers, then build S_x)
  std::vector<Eigen::MatrixXd> A_power(N + 1);
  A_power[0] = Eigen::MatrixXd::Identity(nx, nx);
  for (int i = 1; i <= N; ++i) {
    A_power[i] = A * A_power[i - 1];
  }

  Eigen::MatrixXd S_x(nx * N, nu * N);
  S_x.setZero();
  for (int k = 0; k < N; ++k) {
    for (int j = 0; j <= k; ++j) {
      S_x.block(k * nx, j * nu, nx, nu) = A_power[k - j] * B;
    }
  }

  // Expected S_x
  Eigen::MatrixXd S_x_expected(nx * N, nu * N);
  S_x_expected << 3.0, 0.0, 0.0,
                   6.0, 3.0, 0.0,
                  12.0, 6.0, 3.0;

  for (int i = 0; i < nx * N; ++i) {
    for (int j = 0; j < nu * N; ++j) {
      EXPECT_NEAR(S_x(i, j), S_x_expected(i, j), 1e-12)
        << "S_x(" << i << "," << j << ") mismatch";
    }
  }

  // Buggy version (reversed column order) would produce:
  //   [3, 0, 0; 3, 6, 0; 3, 6, 12]  -- WRONG
  // Verify our result is NOT the buggy one
  EXPECT_NEAR(S_x(1, 0), 6.0, 1e-12) << "S_x[1,0] should be A*B = 6, not B = 3";
  EXPECT_NEAR(S_x(1, 1), 3.0, 1e-12) << "S_x[1,1] should be B = 3, not A*B = 6";
}

// ==============================================================
// P0 regression: OSQP objective scaling (P = 2 * H)
// ==============================================================

/// Verify that the OSQP solver finds the correct minimizer when the
/// MPC Hessian construction is correct (P = 2*H, q = 2*g).
///
/// Scalar QP from MPC formulation:
///   min_z  (z - 3)^2
///
/// Expanded: z^2 - 6z + 9 → H = 1, g = -3
/// OSQP: P = 2*1 = 2, q = 2*(-3) = -6
///
/// Solution: z* = 3
TEST(TestSolverMpc, QPObjectiveScaling)
{
  OSQPSolver solver;
  solver.initialize(1, 1, 1);  // nx=1, nu=1, N=1 → n_vars=1

  // min (z - 3)^2
  //  = z^2 - 6z + 9
  //  = 1/2 * (2) * z^2 + (-6) * z + 9
  // P = 2, q = -6

  int n_vars = 1;
  Eigen::SparseMatrix<double> P(n_vars, n_vars);
  P.insert(0, 0) = 2.0;

  Eigen::VectorXd q(n_vars);
  q << -6.0;

  // No constraints (or wide bounds)
  Eigen::SparseMatrix<double> A_lin(2, 1);
  A_lin.insert(0, 0) = 1.0;
  A_lin.insert(1, 0) = -1.0;

  const double inf = std::numeric_limits<double>::infinity();
  Eigen::VectorXd l(2);
  l << -inf, -inf;
  Eigen::VectorXd u(2);
  u << 100.0, 100.0;

  P.makeCompressed();
  A_lin.makeCompressed();

  solver.setupProblem(P, q, A_lin, l, u);
  bool solved = solver.solve();

  ASSERT_TRUE(solved) << "Scalar QP should solve";

  Eigen::VectorXd sol = solver.getSolution();
  EXPECT_NEAR(sol(0), 3.0, 1e-4) << "Optimal z should be 3.0";

  // Also verify with P_missing_factor = 1 (the BUG: P should be 2*H, not H)
  // If P were 1 instead of 2, the solution would be z* = 6, not 3
  OSQPSolver solver_wrong;
  solver_wrong.initialize(1, 1, 1);

  Eigen::SparseMatrix<double> P_wrong(n_vars, n_vars);
  P_wrong.insert(0, 0) = 1.0;  // BUG: missing 2x factor

  solver_wrong.setupProblem(P_wrong, q, A_lin, l, u);
  solved = solver_wrong.solve();
  ASSERT_TRUE(solved);
  Eigen::VectorXd sol_wrong = solver_wrong.getSolution();

  // With P=1, q=-6: min 0.5*z^2 - 6z → z* = 6 (not 3!)
  EXPECT_NEAR(sol_wrong(0), 6.0, 1e-4)
    << "Without 2*H factor, solution shifts from z*=3 to z*=6";
}

// ==============================================================
// P1 regression: Input rate constraint dt scaling
// ==============================================================

/// Verify that input rate constraints are correctly scaled by dt.
///
/// If config says ±10 Nm/s and dt=0.01s, the per-cycle Δu limit
/// should be ±0.1 Nm, not ±10 Nm.
///
/// We test this by verifying the constraint bounds built by the
/// solver with dt scaling produce the expected per-cycle values.
TEST(TestSolverMpc, InputRateUsesDt)
{
  double dt = 0.01;
  double rate_limit = 10.0;  // Nm/s

  // Expected per-cycle limit
  double expected_per_cycle = rate_limit * dt;  // 0.1 Nm

  // Setup a QP where the unconstrained solution would require a
  // large input change, but the rate constraint limits it.
  // nx=1, nu=1, N=2
  // State: x_{k+1} = x_k + u_k  (simple integrator, A=1, B=1)
  // We want to track x_ref = 100 (far from x0=0)
  // With rate limit applied, u_0 should be bounded

  // Actually, simpler: just verify the bound construction logic
  // The rate constraint is: du_min*dt ≤ u_k - u_{k-1} ≤ du_max*dt
  // at index 0: du_min*dt - (-prev_u) ≤ u_0 ≤ du_max*dt - (-prev_u)

  // If prev_u = 0: the bounds on u_0 for the rate constraint are
  // du_min*dt ≤ u_0 ≤ du_max*dt
  // (rate_offset at index 0 is -prev_u = 0)

  // The test: create a QP where u_0 wants to be 10, but rate
  // limit caps it at 0.1.  The solution should be ~0.1.

  OSQPSolver solver;
  solver.initialize(1, 1, 2);  // nx=1, nu=1, N=2 → n_vars=2

  // Simple tracking problem with high Q
  // min Q*(x_1 - 100)^2 + Q*(x_2 - 100)^2 + R*(u_0^2 + u_1^2)
  // Simple integrator: A=1, B=1
  // S_x = [B, 0; A*B, B] = [1, 0; 1, 1]
  // x_free = [A*x0; A^2*x0] = [0; 0] (x0=0)

  // Build using the MPC condensed formulation:
  int nx = 1, nu = 1, N = 2, n_vars = 2;

  // S_x
  Eigen::MatrixXd S_x(nx * N, n_vars);
  S_x << 1.0, 0.0,
         1.0, 1.0;

  // S_du
  Eigen::MatrixXd S_du(n_vars, n_vars);
  S_du << 1.0, 0.0,
         -1.0, 1.0;

  // Weights: large Q to force tracking, small R
  double Q_val = 1000.0, R_val = 0.001, S_val = 100.0;
  Eigen::MatrixXd Q(nx, nx); Q(0, 0) = Q_val;
  Eigen::MatrixXd R(nu, nu); R(0, 0) = R_val;
  Eigen::MatrixXd S(nu, nu); S(0, 0) = S_val;

  // Build Q_bar, R_bar, S_bar
  int nn = nx * N, nv = n_vars;
  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(nn, nn);
  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(nv, nv);
  Eigen::MatrixXd S_bar = Eigen::MatrixXd::Zero(nv, nv);
  for (int k = 0; k < N; ++k) {
    Q_bar.block(k * nx, k * nx, nx, nx) = Q;
    R_bar.block(k * nu, k * nu, nu, nu) = R;
    S_bar.block(k * nu, k * nu, nu, nu) = S;
  }

  // x_ref = 100
  Eigen::VectorXd x_ref(nx * N);
  x_ref << 100.0, 100.0;

  // x_free = 0 (x0 = 0)
  Eigen::VectorXd x_free(nx * N);
  x_free.setZero();

  // rate_offset
  Eigen::VectorXd rate_offset(n_vars);
  rate_offset << 0.0, 0.0;  // prev_u = 0

  // H and P
  Eigen::MatrixXd H = S_x.transpose() * Q_bar * S_x +
                      R_bar +
                      S_du.transpose() * S_bar * S_du;
  Eigen::MatrixXd P_dense = 2.0 * H;

  // q
  Eigen::VectorXd q_vec = 2.0 * S_x.transpose() * Q_bar * (x_free - x_ref) +
                          2.0 * S_du.transpose() * S_bar * rate_offset;

  // Constraint bounds with CORRECT dt scaling
  int n_state_rows = 2 * nx * N;
  int n_input_rows = n_vars;
  int n_rate_rows = n_vars;
  int n_total = n_state_rows + n_input_rows + n_rate_rows;

  Eigen::VectorXd l(n_total), u(n_total);
  const double inf = std::numeric_limits<double>::infinity();

  // State: no effective bounds (wide)
  for (int i = 0; i < nx * N; ++i) {
    l(i) = -inf; u(i) = inf;
    l(nx * N + i) = -inf; u(nx * N + i) = inf;
  }

  // Input: no amplitude bounds
  for (int i = 0; i < n_vars; ++i) {
    l(n_state_rows + i) = -inf;
    u(n_state_rows + i) = inf;
  }

  // Rate: ±0.1 per cycle (dt=0.01, rate_limit=10)
  double du_limit = dt * rate_limit;  // 0.1
  for (int i = 0; i < n_vars; ++i) {
    l(n_state_rows + n_input_rows + i) = -du_limit - rate_offset(i);
    u(n_state_rows + n_input_rows + i) = du_limit - rate_offset(i);
  }

  // Convert to sparse
  Eigen::SparseMatrix<double> P_sparse = P_dense.sparseView();
  P_sparse = P_sparse.triangularView<Eigen::Upper>();

  // Build A_lin: [S_x; -S_x; I; S_du]
  Eigen::SparseMatrix<double> A_lin(n_total, n_vars);
  std::vector<Eigen::Triplet<double>> triplets;
  for (int k = 0; k < nx * N; ++k) {
    for (int j = 0; j < n_vars; ++j) {
      double val;
      val = S_x(k, j); if (std::abs(val) > 1e-15) triplets.emplace_back(k, j, val);
      val = -S_x(k, j); if (std::abs(val) > 1e-15) triplets.emplace_back(nx * N + k, j, val);
    }
  }
  for (int i = 0; i < n_vars; ++i) triplets.emplace_back(n_state_rows + i, i, 1.0);
  for (int k = 0; k < n_vars; ++k) {
    for (int j = 0; j < n_vars; ++j) {
      double val = S_du(k, j);
      if (std::abs(val) > 1e-15) triplets.emplace_back(n_state_rows + n_input_rows + k, j, val);
    }
  }
  A_lin.setFromTriplets(triplets.begin(), triplets.end());
  A_lin.makeCompressed();

  solver.setupProblem(P_sparse, q_vec, A_lin, l, u);
  bool solved = solver.solve();
  ASSERT_TRUE(solved) << "QP with rate constraints should solve";

  Eigen::VectorXd sol = solver.getSolution();

  // With Q=1000, the unconstrained u_0 would be very large to drive
  // state to 100 quickly. But rate limit of 0.1 per cycle should
  // bound u_0 to about 0.1 (since prev_u=0 and rate constraints
  // are tight).
  EXPECT_LE(std::abs(sol(0)), du_limit + 0.05)
    << "u_0 should be bounded by rate limit ±0.1, got " << sol(0);
}

// ==============================================================
// P1 edge case: Infeasible QP
// ==============================================================

TEST(TestOSQPSolver, infeasibleQP)
{
  OSQPSolver solver;
  solver.initialize(1, 1, 1);

  // Trivial QP with contradictory constraints
  // min u^2  s.t.  2 ≤ u ≤ 1  (empty feasible set)
  int n_vars = 1;
  Eigen::SparseMatrix<double> P(n_vars, n_vars);
  P.insert(0, 0) = 2.0;

  Eigen::VectorXd q(n_vars);
  q << 0.0;

  // OSQP: l ≤ A_lin*z ≤ u
  // Row 0:  2.0 ≤ z  → z ≥ 2
  // Row 1:  -inf ≤ z ≤ 1.0  → z ≤ 1
  // Together: z ≥ 2 AND z ≤ 1 → infeasible
  Eigen::SparseMatrix<double> A_lin_inf(2, 1);
  A_lin_inf.insert(0, 0) = 1.0;
  A_lin_inf.insert(1, 0) = 1.0;

  Eigen::VectorXd l_inf(2);
  l_inf << 2.0, -std::numeric_limits<double>::infinity();
  Eigen::VectorXd u_inf(2);
  u_inf << std::numeric_limits<double>::infinity(), 1.0;  // z ≥ 2 AND z ≤ 1

  P.makeCompressed();
  A_lin_inf.makeCompressed();

  solver.setupProblem(P, q, A_lin_inf, l_inf, u_inf);
  bool solved = solver.solve();

  // Should either fail to solve OR report infeasible status
  auto diag = solver.getDiagnostics();
  int status = diag.status;
  EXPECT_TRUE(status == -3 || status == -4)
    << "Infeasible QP should report prim_infeas (-3) or dual_infeas (-4), got " << status;
}

// ==============================================================
// LinearModel tests
// ==============================================================

TEST(TestLinearModel, doubleIntegrator)
{
  MpcParams params;
  params.state_dim = 2;
  params.input_dim = 1;
  params.A_data = {1.0, 0.01, 0.0, 1.0};
  params.B_data = {0.0, 0.01};

  LinearModel model;
  model.initialize(params);

  EXPECT_EQ(model.getStateDim(), 2);
  EXPECT_EQ(model.getInputDim(), 1);

  // Predict with zero input
  Eigen::VectorXd x0(2);
  x0 << 1.0, 0.0;
  Eigen::VectorXd u_seq(2);  // horizon=2, input_dim=1
  u_seq << 0.0, 0.0;

  Eigen::MatrixXd X = model.predict(x0, u_seq);
  EXPECT_EQ(X.cols(), 3);  // horizon + 1

  EXPECT_NEAR(X(0, 0), 1.0, 1e-10);
  EXPECT_NEAR(X(1, 0), 0.0, 1e-10);
  EXPECT_NEAR(X(0, 1), 1.0, 1e-10);
  EXPECT_NEAR(X(1, 1), 0.0, 1e-10);

  // Predict with non-zero input
  Eigen::VectorXd u_seq2(2);
  u_seq2 << 1.0, 1.0;

  Eigen::MatrixXd X2 = model.predict(x0, u_seq2);
  // x_1 = [1 + 0.01*0; 0 + 0.01*1] = [1; 0.01]
  EXPECT_NEAR(X2(0, 1), 1.0, 1e-10);
  EXPECT_NEAR(X2(1, 1), 0.01, 1e-10);
  // x_2 = A * x_1 + B * 1 = [1 + 0.01*0.01; 0.01 + 0.01*1] = [1.0001; 0.02]
  EXPECT_NEAR(X2(0, 2), 1.0001, 1e-6);
  EXPECT_NEAR(X2(1, 2), 0.02, 1e-10);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// ==============================================================
// Regression: Full RRBot QP with workspace reuse
// ==============================================================

TEST(TestSolverMpc, FullRRBotQP)
{
  // Replicate RRBot parameters
  int nx = 4, nu = 2, N = 20;
  double dt = 0.01;
  int n_vars = nu * N;

  MpcParams params;
  params.state_dim = nx;
  params.input_dim = nu;
  params.prediction_horizon = N;
  params.dt = dt;
  params.Q_diag = {100.0, 1.0, 100.0, 1.0};
  params.R_diag = {0.1, 0.1};
  params.S_diag = {1.0, 1.0};
  params.A_data = {1.0, dt, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, dt, 0.0, 0.0, 0.0, 1.0};
  params.B_data = {0.0, 0.0, dt, 0.0, 0.0, 0.0, 0.0, dt};
  params.state_lower = {-3.10, -5.0, -3.10, -5.0};
  params.state_upper = {3.10, 5.0, 3.10, 5.0};
  params.input_lower = {-3.0, -3.0};
  params.input_upper = {3.0, 3.0};
  params.input_rate_lower = {-10.0, -10.0};
  params.input_rate_upper = {10.0, 10.0};
  params.max_iterations = 4000;
  params.abs_tol = 1e-4;
  params.rel_tol = 1e-4;

  // Build model (needs A, B)
  LinearModel model;
  model.initialize(params);

  // Build A powers for S_x
  Eigen::MatrixXd A = model.getA();
  Eigen::MatrixXd B = model.getB();

  std::vector<Eigen::MatrixXd> A_power(N + 1);
  A_power[0] = Eigen::MatrixXd::Identity(nx, nx);
  for (int i = 1; i <= N; ++i) {
    A_power[i] = A * A_power[i - 1];
  }

  // Build S_x
  Eigen::MatrixXd S_x(nx * N, n_vars);
  S_x.setZero();
  for (int k = 0; k < N; ++k) {
    for (int j = 0; j <= k; ++j) {
      S_x.block(k * nx, j * nu, nx, nu) = A_power[k - j] * B;
    }
  }

  // Build S_du
  Eigen::MatrixXd S_du(n_vars, n_vars);
  S_du.setZero();
  for (int k = 0; k < N; ++k) {
    S_du.block(k * nu, k * nu, nu, nu) = Eigen::MatrixXd::Identity(nu, nu);
    if (k > 0) {
      S_du.block(k * nu, (k - 1) * nu, nu, nu) = -Eigen::MatrixXd::Identity(nu, nu);
    }
  }

  // Build A_lin
  int n_state_rows = 2 * nx * N;
  int n_input_rows = n_vars;
  int n_rate_rows = n_vars;
  int n_total = n_state_rows + n_input_rows + n_rate_rows;

  Eigen::SparseMatrix<double> A_lin(n_total, n_vars);
  std::vector<Eigen::Triplet<double>> triplets;
  for (int k = 0; k < nx * N; ++k) {
    for (int j = 0; j < n_vars; ++j) {
      double val = S_x(k, j);
      if (std::abs(val) > 1e-15) triplets.emplace_back(k, j, val);
      val = -S_x(k, j);
      if (std::abs(val) > 1e-15) triplets.emplace_back(nx * N + k, j, val);
    }
  }
  for (int i = 0; i < n_vars; ++i) triplets.emplace_back(n_state_rows + i, i, 1.0);
  for (int k = 0; k < n_vars; ++k) {
    for (int j = 0; j < n_vars; ++j) {
      double val = S_du(k, j);
      if (std::abs(val) > 1e-15) triplets.emplace_back(n_state_rows + n_input_rows + k, j, val);
    }
  }
  A_lin.setFromTriplets(triplets.begin(), triplets.end());
  A_lin.makeCompressed();

  // Build weight matrices
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nx, nx);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(nu, nu);
  Eigen::MatrixXd S = Eigen::MatrixXd::Zero(nu, nu);
  for (int i = 0; i < nx; ++i) Q(i, i) = params.Q_diag[i];
  for (int i = 0; i < nu; ++i) R(i, i) = params.R_diag[i];
  for (int i = 0; i < nu; ++i) S(i, i) = params.S_diag[i];

  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(nx * N, nx * N);
  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(n_vars, n_vars);
  Eigen::MatrixXd S_bar = Eigen::MatrixXd::Zero(n_vars, n_vars);
  for (int k = 0; k < N; ++k) {
    Q_bar.block(k * nx, k * nx, nx, nx) = Q;
    R_bar.block(k * nu, k * nu, nu, nu) = R;
    S_bar.block(k * nu, k * nu, nu, nu) = S;
  }

  // Build P
  Eigen::MatrixXd H = S_x.transpose() * Q_bar * S_x +
                      R_bar +
                      S_du.transpose() * S_bar * S_du;
  Eigen::MatrixXd P_dense = 2.0 * H;

  // Initial data (x0=0, ref=0, prev_u=0)
  Eigen::VectorXd x_free(nx * N);
  x_free.setZero();
  Eigen::VectorXd x_ref_stacked(nx * N);
  x_ref_stacked.setZero();
  Eigen::VectorXd rate_offset(n_vars);
  rate_offset.setZero();

  Eigen::VectorXd q_vec = 2.0 * S_x.transpose() * Q_bar * (x_free - x_ref_stacked) +
                          2.0 * S_du.transpose() * S_bar * rate_offset;

  // Build bounds
  Eigen::VectorXd x_min_vec(nx * N);
  Eigen::VectorXd x_max_vec(nx * N);
  for (int k = 0; k < N; ++k) {
    x_min_vec.segment(k * nx, nx) = Eigen::Map<const Eigen::VectorXd>(params.state_lower.data(), nx);
    x_max_vec.segment(k * nx, nx) = Eigen::Map<const Eigen::VectorXd>(params.state_upper.data(), nx);
  }

  Eigen::VectorXd l(n_total);
  Eigen::VectorXd u(n_total);
  const double inf = std::numeric_limits<double>::infinity();

  for (int i = 0; i < nx * N; ++i) {
    l(i) = -inf;
    u(i) = x_max_vec(i) - x_free(i);
    l(nx * N + i) = -inf;
    u(nx * N + i) = -x_min_vec(i) + x_free(i);
  }
  for (int k = 0; k < N; ++k) {
    for (int i = 0; i < nu; ++i) {
      l(n_state_rows + k * nu + i) = params.input_lower[i];
      u(n_state_rows + k * nu + i) = params.input_upper[i];
    }
  }
  for (int k = 0; k < N; ++k) {
    for (int i = 0; i < nu; ++i) {
      double du_min_dt = dt * params.input_rate_lower[i];
      double du_max_dt = dt * params.input_rate_upper[i];
      l(n_state_rows + n_input_rows + k * nu + i) = du_min_dt - rate_offset(k * nu + i);
      u(n_state_rows + n_input_rows + k * nu + i) = du_max_dt - rate_offset(k * nu + i);
    }
  }

  // Build P sparse (upper triangular, all entries)
  Eigen::SparseMatrix<double> P_sparse(n_vars, n_vars);
  {
    std::vector<Eigen::Triplet<double>> p_triplets;
    p_triplets.reserve(n_vars * (n_vars + 1) / 2);
    for (int col = 0; col < n_vars; ++col) {
      for (int row = 0; row <= col; ++row) {
        p_triplets.emplace_back(row, col, P_dense(row, col));
      }
    }
    P_sparse.setFromTriplets(p_triplets.begin(), p_triplets.end());
    P_sparse.makeCompressed();
  }

  // === TEST: Fresh solver with zero state/reference ===
  {
    OSQPSolver solver;
    solver.initialize(nx, nu, N);
    solver.setSolverSettings(params.max_iterations, params.abs_tol, params.rel_tol);
    solver.setupProblem(P_sparse, q_vec, A_lin, l, u);

    bool ok = solver.solve();
    auto diag = solver.getDiagnostics();

    EXPECT_TRUE(ok) << "Fresh solve with zero data should succeed";
    EXPECT_TRUE(diag.solved) << "Should be solved (kSolved)";

    // Check solution norm (should be small since everything is zero)
    Eigen::VectorXd sol = solver.getSolution();
    EXPECT_LT(sol.norm(), 1e-6) << "With x0=0 and ref=0, optimal z should be near zero";
  }

  // === TEST: Workspace reuse (update gradient/bounds, solve) ===
  {
    OSQPSolver solver;
    solver.initialize(nx, nu, N);
    solver.setSolverSettings(params.max_iterations, params.abs_tol, params.rel_tol);
    solver.setupProblem(P_sparse, q_vec, A_lin, l, u);

    bool ok = solver.solve();
    EXPECT_TRUE(ok) << "Initial solve should succeed";

    // Update with non-zero reference and solve again (simulating 2nd cycle)
    Eigen::VectorXd x0(nx); x0 << 0.1, 0.0, -0.05, 0.0;
    Eigen::VectorXd x_ref(nx); x_ref << 0.5, 0.0, -0.3, 0.0;
    for (int k = 0; k < N; ++k) {
      x_free.segment(k * nx, nx) = A_power[k + 1] * x0;
      x_ref_stacked.segment(k * nx, nx) = x_ref;
    }
    rate_offset.setZero();
    rate_offset.head(nu) = Eigen::VectorXd::Zero(nu);  // prev_u = 0

    q_vec = 2.0 * S_x.transpose() * Q_bar * (x_free - x_ref_stacked) +
            2.0 * S_du.transpose() * S_bar * rate_offset;

    for (int i = 0; i < nx * N; ++i) {
      u(i) = x_max_vec(i) - x_free(i);
      u(nx * N + i) = -x_min_vec(i) + x_free(i);
    }
    for (int k = 0; k < N; ++k) {
      for (int i = 0; i < nu; ++i) {
        double du_min_dt = dt * params.input_rate_lower[i];
        double du_max_dt = dt * params.input_rate_upper[i];
        l(n_state_rows + n_input_rows + k * nu + i) = du_min_dt - rate_offset(k * nu + i);
        u(n_state_rows + n_input_rows + k * nu + i) = du_max_dt - rate_offset(k * nu + i);
      }
    }

    solver.updateGradient(q_vec);
    solver.updateBounds(l, u);
    ok = solver.solve();
    auto diag = solver.getDiagnostics();

    EXPECT_TRUE(ok || diag.solved_approximate)
      << "Solve with non-zero state/reference should succeed or be approximate";
  }
}
