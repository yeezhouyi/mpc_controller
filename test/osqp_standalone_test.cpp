// Standalone test replicating the exact RRBot QP to diagnose solver failure
#include <cstdio>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>

extern "C" {
#include "osqp.h"
}

// Include the c++ wrapper directly, but undef the logging first
#undef RCLCPP_INFO
#undef RCLCPP_ERROR
#undef RCLCPP_FATAL
#define RCLCPP_INFO(...) printf("INFO: "); printf(__VA_ARGS__); printf("\n")
#define RCLCPP_ERROR(...) printf("ERROR: "); printf(__VA_ARGS__); printf("\n")
#define RCLCPP_FATAL(...) printf("FATAL: "); printf(__VA_ARGS__); printf("\n")
#define RCLCPP_WARN_THROTTLE(...) printf("WARN: "); printf(__VA_ARGS__); printf("\n")

#include "mpc_controller/osqp_solver.hpp"
#include "mpc_controller/linear_model.hpp"

using namespace mpc_controller;

int main() {
  printf("=== RRBot QP Standalone Test ===\n");

  int nx = 4, nu = 2, N = 20;
  double dt = 0.01;
  int n_vars = nu * N;

  // Build model and params
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
  params.state_lower = {-2.5, -5.0, -2.5, -5.0};
  params.state_upper = {2.5, 5.0, 2.5, 5.0};
  params.input_lower = {-3.0, -3.0};
  params.input_upper = {3.0, 3.0};
  params.input_rate_lower = {-10.0, -10.0};
  params.input_rate_upper = {10.0, 10.0};

  // Build model
  LinearModel model;
  model.initialize(params);

  // Build A powers
  Eigen::MatrixXd A = model.getA();
  Eigen::MatrixXd B = model.getB();
  printf("A = \n"); std::cout << A << std::endl;
  printf("B = \n"); std::cout << B << std::endl;

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

  printf("A_lin: %ld x %ld, nnz=%ld\n", A_lin.rows(), A_lin.cols(), A_lin.nonZeros());

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

  // Build P (Hessian)
  Eigen::MatrixXd H = S_x.transpose() * Q_bar * S_x + R_bar + S_du.transpose() * S_bar * S_du;
  Eigen::MatrixXd P_dense = 2.0 * H;

  // Check P eigenvalues
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(P_dense);
  printf("P eigenvalues: min=%e, max=%e, cond=%e\n",
    eigensolver.eigenvalues()(0),
    eigensolver.eigenvalues()(n_vars - 1),
    eigensolver.eigenvalues()(n_vars - 1) / eigensolver.eigenvalues()(0));

  // Build initial q (with x0=0, x_ref=0, prev_u=0)
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nx);
  Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(nx);
  Eigen::VectorXd x_ref_stacked(nx * N);
  for (int k = 0; k < N; ++k) x_ref_stacked.segment(k * nx, nx) = x_ref;

  Eigen::VectorXd x_free(nx * N);
  for (int k = 0; k < N; ++k) x_free.segment(k * nx, nx) = A_power[k + 1] * x0;

  Eigen::VectorXd rate_offset = Eigen::VectorXd::Zero(n_vars);
  Eigen::VectorXd q_vec = 2.0 * S_x.transpose() * Q_bar * (x_free - x_ref_stacked) +
                          2.0 * S_du.transpose() * S_bar * rate_offset;

  printf("Initial q_vec norm: %e\n", q_vec.norm());

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

  // Build P sparse (upper triangular)
  Eigen::SparseMatrix<double> P_sparse(n_vars, n_vars);
  std::vector<Eigen::Triplet<double>> p_triplets;
  p_triplets.reserve(n_vars * (n_vars + 1) / 2);
  for (int col = 0; col < n_vars; ++col) {
    for (int row = 0; row <= col; ++row) {
      p_triplets.emplace_back(row, col, P_dense(row, col));
    }
  }
  P_sparse.setFromTriplets(p_triplets.begin(), p_triplets.end());
  P_sparse.makeCompressed();
  printf("P_sparse: %ld x %ld, nnz=%ld\n", P_sparse.rows(), P_sparse.cols(), P_sparse.nonZeros());

  // === TEST 1: fresh solver with x0=0, ref=0 ===
  printf("\n=== TEST 1: x0=0, ref=0, fresh solver ===\n");
  {
    OSQPSolver solver;
    solver.initialize(nx, nu, N);
    solver.setSolverSettings(4000, 1e-4, 1e-4);
    solver.setupProblem(P_sparse, q_vec, A_lin, l, u);
    bool ok = solver.solve();
    auto diag = solver.getDiagnostics();
    printf("solve ok=%d, status=%d, iter=%lld, pri_res=%e, dua_res=%e, obj=%e\n",
      ok, diag.status, (long long)diag.iterations, diag.pri_res, diag.dua_res, diag.objective);
    printf("solved=%d, solved_approx=%d\n", diag.solved, diag.solved_approximate);
  }

  // === TEST 2: fresh solver with x0≠0, ref≠0 ===
  printf("\n=== TEST 2: x0≠0, ref≠0, fresh solver ===\n");
  {
    x0 = Eigen::VectorXd::Zero(nx);
    x_ref = Eigen::VectorXd::Zero(nx);
    x_ref(0) = 1.0;  // track joint 1 at 1.0 rad

    // Recompute trajectory-dependent data
    for (int k = 0; k < N; ++k) x_free.segment(k * nx, nx) = A_power[k + 1] * x0;
    for (int k = 0; k < N; ++k) x_ref_stacked.segment(k * nx, nx) = x_ref;

    // With different rate offset
    rate_offset.setZero();
    q_vec = 2.0 * S_x.transpose() * Q_bar * (x_free - x_ref_stacked) +
            2.0 * S_du.transpose() * S_bar * rate_offset;

    // Rebuild bounds with x_free
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

    // Build P with new parameters (same weights, so same P)
    // Use existing P_sparse

    OSQPSolver solver;
    solver.initialize(nx, nu, N);
    solver.setSolverSettings(4000, 1e-4, 1e-4);
    solver.setupProblem(P_sparse, q_vec, A_lin, l, u);
    bool ok = solver.solve();
    auto diag = solver.getDiagnostics();
    printf("solve ok=%d, status=%d, iter=%lld, pri_res=%e, dua_res=%e, obj=%e\n",
      ok, diag.status, (long long)diag.iterations, diag.pri_res, diag.dua_res, diag.objective);
    printf("solved=%d, solved_approx=%d\n", diag.solved, diag.solved_approximate);

    // Check solution
    Eigen::VectorXd sol = solver.getSolution();
    printf("u0[0]=%e, u0[1]=%e, norm(z)=%e\n", sol(0), sol(1), sol.norm());
  }

  // === TEST 3: workspace reuse (setup once, update each cycle) ===
  printf("\n=== TEST 3: workspace reuse ===\n");
  {
    x0.setZero();
    x_ref.setZero();
    for (int k = 0; k < N; ++k) x_ref_stacked.segment(k * nx, nx) = x_ref;
    for (int k = 0; k < N; ++k) x_free.segment(k * nx, nx) = A_power[k + 1] * x0;
    rate_offset.setZero();
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

    OSQPSolver solver;
    solver.initialize(nx, nu, N);
    solver.setSolverSettings(4000, 1e-4, 1e-4);
    solver.setupProblem(P_sparse, q_vec, A_lin, l, u);

    // First solve
    bool ok = solver.solve();
    auto diag = solver.getDiagnostics();
    printf("Solve 1: ok=%d, status=%d, iter=%lld, pri_res=%e, dua_res=%e\n",
      ok, diag.status, (long long)diag.iterations, diag.pri_res, diag.dua_res);

    // Update and solve again (simulating next cycle)
    for (int cycle = 2; cycle <= 5; ++cycle) {
      x0 = Eigen::VectorXd::Random(nx);  // non-zero state
      x_ref = Eigen::VectorXd::Zero(nx);

      // Simulate trajectory at this state
      Eigen::VectorXd prev_u(2); prev_u << 0.1, -0.05;

      for (int k = 0; k < N; ++k) x_free.segment(k * nx, nx) = A_power[k + 1] * x0;
      for (int k = 0; k < N; ++k) x_ref_stacked.segment(k * nx, nx) = x_ref;

      rate_offset.setZero();
      rate_offset.head(nu) = -prev_u;
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
      diag = solver.getDiagnostics();
      printf("Solve %d: ok=%d, status=%d, iter=%lld, pri_res=%e, dua_res=%e\n",
        cycle, ok, diag.status, (long long)diag.iterations, diag.pri_res, diag.dua_res);
    }
  }

  printf("\n=== DONE ===\n");
  return 0;
}
