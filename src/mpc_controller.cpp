#include "mpc_controller/mpc_controller.hpp"
#include "mpc_controller/linear_model.hpp"
#include "mpc_controller/osqp_solver.hpp"

#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace mpc_controller
{

MPCController::MPCController()
: controller_interface::ControllerInterface()
{
}

// ---- Lifecycle hooks ----

controller_interface::CallbackReturn MPCController::on_init()
{
  try {
    auto & param_list = *this->get_node();
    param_list.declare_parameter<int>("prediction_horizon", 20);
    param_list.declare_parameter<double>("dt", 0.01);
    param_list.declare_parameter<std::vector<double>>("Q_diag", std::vector<double>());
    param_list.declare_parameter<std::vector<double>>("R_diag", std::vector<double>());
    param_list.declare_parameter<std::vector<double>>("S_diag", std::vector<double>());
    param_list.declare_parameter<std::vector<double>>("A_data", std::vector<double>());
    param_list.declare_parameter<std::vector<double>>("B_data", std::vector<double>());
    param_list.declare_parameter<std::vector<double>>("state_lower", std::vector<double>());
    param_list.declare_parameter<std::vector<double>>("state_upper", std::vector<double>());
    param_list.declare_parameter<std::vector<double>>("input_lower", std::vector<double>());
    param_list.declare_parameter<std::vector<double>>("input_upper", std::vector<double>());
    param_list.declare_parameter<std::vector<double>>("input_rate_lower", std::vector<double>());
    param_list.declare_parameter<std::vector<double>>("input_rate_upper", std::vector<double>());
    param_list.declare_parameter<std::vector<std::string>>(
      "state_interface_names", std::vector<std::string>());
    param_list.declare_parameter<std::vector<std::string>>(
      "command_interface_names", std::vector<std::string>());
    param_list.declare_parameter<std::vector<int64_t>>("velocity_indices", std::vector<int64_t>());
    param_list.declare_parameter<double>("slack_rho_1", 100.0);
    param_list.declare_parameter<double>("slack_rho_2", 10.0);
    param_list.declare_parameter<int>("max_iterations", 4000);
    param_list.declare_parameter<double>("abs_tol", 1e-4);
    param_list.declare_parameter<double>("rel_tol", 1e-4);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_node()->get_logger(), "Exception in on_init(): %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MPCController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto & node = *this->get_node();

  // Read parameters
  params_.prediction_horizon = node.get_parameter("prediction_horizon").as_int();
  params_.dt = node.get_parameter("dt").as_double();
  params_.Q_diag = node.get_parameter("Q_diag").as_double_array();
  params_.R_diag = node.get_parameter("R_diag").as_double_array();
  params_.S_diag = node.get_parameter("S_diag").as_double_array();
  params_.A_data = node.get_parameter("A_data").as_double_array();
  params_.B_data = node.get_parameter("B_data").as_double_array();

  params_.state_lower = node.get_parameter("state_lower").as_double_array();
  params_.state_upper = node.get_parameter("state_upper").as_double_array();
  params_.input_lower = node.get_parameter("input_lower").as_double_array();
  params_.input_upper = node.get_parameter("input_upper").as_double_array();
  params_.input_rate_lower = node.get_parameter("input_rate_lower").as_double_array();
  params_.input_rate_upper = node.get_parameter("input_rate_upper").as_double_array();

  params_.state_interface_names = node.get_parameter("state_interface_names").as_string_array();
  params_.command_interface_names = node.get_parameter("command_interface_names").as_string_array();

  params_.max_iterations = node.get_parameter("max_iterations").as_int();
  params_.abs_tol = node.get_parameter("abs_tol").as_double();
  params_.rel_tol = node.get_parameter("rel_tol").as_double();

  // Soft constraint parameters
  auto vel_idx_64 = node.get_parameter("velocity_indices").as_integer_array();
  params_.velocity_indices.resize(vel_idx_64.size());
  for (size_t i = 0; i < vel_idx_64.size(); ++i) {
    params_.velocity_indices[i] = static_cast<int>(vel_idx_64[i]);
  }
  params_.slack_rho_1 = node.get_parameter("slack_rho_1").as_double();
  params_.slack_rho_2 = node.get_parameter("slack_rho_2").as_double();

  // Infer dimensions from weight matrices
  int nx = params_.Q_diag.size();
  int nu = params_.R_diag.size();
  params_.state_dim = nx;
  params_.input_dim = nu;

  if (nx == 0 || nu == 0) {
    RCLCPP_ERROR(node.get_logger(), "Q_diag or R_diag is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Validate parameter sizes
  auto check_size = [&](const std::string & name, size_t got, size_t expected) {
    if (got != expected) {
      RCLCPP_ERROR(
        node.get_logger(), "%s size %zu != expected %zu",
        name.c_str(), got, expected);
      throw std::runtime_error("Parameter size mismatch");
    }
  };
  check_size("A_data", params_.A_data.size(), nx * nx);
  check_size("B_data", params_.B_data.size(), nx * nu);
  check_size("state_lower", params_.state_lower.size(), nx);
  check_size("state_upper", params_.state_upper.size(), nx);
  check_size("input_lower", params_.input_lower.size(), nu);
  check_size("input_upper", params_.input_upper.size(), nu);
  check_size("input_rate_lower", params_.input_rate_lower.size(), nu);
  check_size("input_rate_upper", params_.input_rate_upper.size(), nu);
  check_size(
    "state_interface_names", params_.state_interface_names.size(), nx);
  check_size(
    "command_interface_names", params_.command_interface_names.size(), nu);

  // Soft constraint initialization
  n_vel_ = params_.velocity_indices.size();
  n_slack_ = n_vel_ * params_.prediction_horizon;
  vel_indices_ = params_.velocity_indices;
  slack_rho_1_ = params_.slack_rho_1;
  slack_rho_2_ = params_.slack_rho_2;
  if (n_vel_ > 0) {
    // Validate velocity indices against state dimension
    for (int idx : vel_indices_) {
      if (idx < 0 || idx >= nx) {
        RCLCPP_ERROR(
          node.get_logger(),
          "velocity_indices entry %d out of range [0, %d)", idx, nx);
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    RCLCPP_INFO(
      node.get_logger(),
      "Soft velocity constraints enabled: %d velocity states, rho_1=%.0f, rho_2=%.0f",
      n_vel_, slack_rho_1_, slack_rho_2_);
  }

  // Initialize state storage
  x_ref_ = Eigen::VectorXd::Zero(nx);
  prev_u_ = Eigen::VectorXd::Zero(nu);

  // Initialize model
  model_ = std::make_unique<LinearModel>();
  model_->initialize(params_);

  // Initialize solver with settings from YAML
  solver_ = std::make_unique<OSQPSolver>();
  solver_->initialize(nx, nu, params_.prediction_horizon);
  solver_->setNumSlackVariables(n_vel_);
  solver_->setSolverSettings(
    params_.max_iterations, params_.abs_tol, params_.rel_tol);

  // Build fixed QP structure: S_x, S_du, A_lin (depends only on A, B, N)
  buildQPStructure();

  // Build initial weight matrices and setup the OSQP workspace once
  // (subsequent cycles only update gradient and bounds)
  const int N = params_.prediction_horizon;
  const int n_vars = nu * N;                     // U-only variables
  const int n_vars_total = nu * N + n_slack_;    // U + slack variables

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nx, nx);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(nu, nu);
  Eigen::MatrixXd S = Eigen::MatrixXd::Zero(nu, nu);
  for (int i = 0; i < nx; ++i) Q(i, i) = params_.Q_diag[i];
  for (int i = 0; i < nu; ++i) R(i, i) = params_.R_diag[i];
  for (int i = 0; i < nu; ++i) S(i, i) = params_.S_diag.empty() ? 0.0 : params_.S_diag[i];

  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(nx * N, nx * N);
  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(n_vars, n_vars);
  Eigen::MatrixXd S_bar = Eigen::MatrixXd::Zero(n_vars, n_vars);
  for (int k = 0; k < N; ++k) {
    Q_bar.block(k * nx, k * nx, nx, nx) = Q;
    R_bar.block(k * nu, k * nu, nu, nu) = R;
    S_bar.block(k * nu, k * nu, nu, nu) = S;
  }

  // Initial gradient (q) and Hessian (P) — these get updated per cycle
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nx);
  Eigen::VectorXd x_ref_stacked = Eigen::VectorXd::Zero(nx * N);  // no reference received yet
  Eigen::VectorXd x_free = Eigen::VectorXd::Zero(nx * N);
  // With x0=0, x_free = 0 since A_power[k+1] * 0 = 0

  Eigen::VectorXd rate_offset = Eigen::VectorXd::Zero(n_vars);  // prev_u_ = 0 at start

  // H = S_x^T * Q_bar * S_x + R_bar + S_du^T * S_bar * S_du
  Eigen::MatrixXd H = S_x_.transpose() * Q_bar * S_x_ +
                      R_bar +
                      S_du_.transpose() * S_bar * S_du_;

  // Extended Hessian with slack penalty (OSQP uses 1/2 * z^T * P * z)
  Eigen::MatrixXd P_dense = Eigen::MatrixXd::Zero(n_vars_total, n_vars_total);
  P_dense.topLeftCorner(n_vars, n_vars) = 2.0 * H;
  if (n_slack_ > 0) {
    for (int i = 0; i < n_slack_; ++i) {
      P_dense(n_vars + i, n_vars + i) = 2.0 * slack_rho_2_;
    }
  }

  // q = 2 * (S_x^T * Q_bar * (x_free - x_ref_stacked) + S_du^T * S_bar * rate_offset)
  Eigen::RowVectorXd q_row = 2.0 * (x_free - x_ref_stacked).transpose() * Q_bar * S_x_ +
                             2.0 * rate_offset.transpose() * S_bar * S_du_;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(n_vars_total);
  q.head(n_vars) = q_row.transpose();
  // Slack gradient: ρ₁ · 1 (L1 penalty, linear term)
  if (n_slack_ > 0) {
    q.tail(n_slack_).setConstant(slack_rho_1_);
  }

  // Build initial constraint bounds l, u
  // A_lin was built by buildQPStructure()
  int n_state_rows = 2 * nx * N;
  int n_input_rows = n_vars;
  int n_rate_rows = n_vars;
  int n_total = n_state_rows + n_input_rows + n_rate_rows + n_slack_;

  Eigen::VectorXd l_init(n_total);
  Eigen::VectorXd u_init(n_total);
  const double inf = std::numeric_limits<double>::infinity();

  // State bounds: -inf ≤ S_x*z ≤ x_max_vec - x_free (with x0=0, x_free=0)
  Eigen::VectorXd x_min_vec(nx * N);
  Eigen::VectorXd x_max_vec(nx * N);
  for (int k = 0; k < N; ++k) {
    x_min_vec.segment(k * nx, nx) = Eigen::Map<const Eigen::VectorXd>(
      params_.state_lower.data(), nx);
    x_max_vec.segment(k * nx, nx) = Eigen::Map<const Eigen::VectorXd>(
      params_.state_upper.data(), nx);
  }
  for (int i = 0; i < nx * N; ++i) {
    l_init(i) = -inf;
    u_init(i) = x_max_vec(i);
    l_init(nx * N + i) = -inf;
    u_init(nx * N + i) = -x_min_vec(i);
  }

  // Input bounds
  Eigen::VectorXd u_min_vec(n_vars);
  Eigen::VectorXd u_max_vec(n_vars);
  for (int k = 0; k < N; ++k) {
    u_min_vec.segment(k * nu, nu) = Eigen::Map<const Eigen::VectorXd>(
      params_.input_lower.data(), nu);
    u_max_vec.segment(k * nu, nu) = Eigen::Map<const Eigen::VectorXd>(
      params_.input_upper.data(), nu);
  }
  for (int i = 0; i < n_vars; ++i) {
    l_init(n_state_rows + i) = u_min_vec(i);
    u_init(n_state_rows + i) = u_max_vec(i);
  }

  // Rate bounds (initially with rate_offset = 0)
  double dt = params_.dt;
  Eigen::VectorXd du_min_vec(n_vars);
  Eigen::VectorXd du_max_vec(n_vars);
  for (int k = 0; k < N; ++k) {
    du_min_vec.segment(k * nu, nu) = dt * Eigen::Map<const Eigen::VectorXd>(
      params_.input_rate_lower.data(), nu);
    du_max_vec.segment(k * nu, nu) = dt * Eigen::Map<const Eigen::VectorXd>(
      params_.input_rate_upper.data(), nu);
  }
  for (int i = 0; i < n_vars; ++i) {
    l_init(n_state_rows + n_input_rows + i) = du_min_vec(i);
    u_init(n_state_rows + n_input_rows + i) = du_max_vec(i);
  }

  // Slack non-negativity constraints: -ε ≤ 0 → l=-inf, u=0
  for (int i = 0; i < n_slack_; ++i) {
    l_init(n_state_rows + n_input_rows + n_rate_rows + i) = -inf;
    u_init(n_state_rows + n_input_rows + n_rate_rows + i) = 0.0;
  }

  // One-time OSQP workspace setup
  // Build P_sparse with ALL upper-triangular entries to guarantee
  // identical sparsity structure across updates (required by OSQP).
  const int n_vars_total_p = n_vars + n_slack_;
  Eigen::SparseMatrix<double> P_sparse(n_vars_total_p, n_vars_total_p);
  {
    std::vector<Eigen::Triplet<double>> p_triplets;
    p_triplets.reserve(n_vars_total_p * (n_vars_total_p + 1) / 2);
    // U block (upper triangular)
    for (int col = 0; col < n_vars; ++col) {
      for (int row = 0; row <= col; ++row) {
        p_triplets.emplace_back(row, col, P_dense(row, col));
      }
    }
    // Slack block (diagonal only)
    if (n_slack_ > 0) {
      for (int i = 0; i < n_slack_; ++i) {
        int idx = n_vars + i;
        p_triplets.emplace_back(idx, idx, P_dense(idx, idx));
      }
    }
    P_sparse.setFromTriplets(p_triplets.begin(), p_triplets.end());
    P_sparse.makeCompressed();
  }

  try {
    solver_->setupProblem(P_sparse, q, A_lin_, l_init, u_init);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node.get_logger(), "QP setup failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reference subscriber
  rt_ref_.writeFromNonRT(Eigen::VectorXd::Zero(nx));
  ref_sub_ = node.create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/reference", rclcpp::SystemDefaultsQoS(),
    [this, nx](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if (msg->data.size() >= static_cast<size_t>(nx)) {
        Eigen::VectorXd ref(nx);
        for (int i = 0; i < nx; ++i) {
          ref(i) = msg->data[i];
        }
        rt_ref_.writeFromNonRT(ref);
      }
    });

  // Diagnostics publisher — pre-allocate to avoid allocation in realtime path
  diag_pub_ = node.create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/diagnostics", rclcpp::SystemDefaultsQoS());
  rt_diag_pub_ = std::make_shared<realtime_tools::RealtimePublisher<
      std_msgs::msg::Float64MultiArray>>(diag_pub_);
  rt_diag_pub_->lock();
  rt_diag_pub_->msg_.data.resize(5 + 3 * params_.state_dim + params_.input_dim + 13);
  rt_diag_pub_->unlock();

  // Dynamic parameter callback (hot-reload Q/R/S weights)
  rt_params_.writeFromNonRT(params_);
  param_cb_ = node.add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      return parameterUpdateCallback(params);
    });

  RCLCPP_INFO(
    node.get_logger(),
    "MPC configured: nx=%d, nu=%d, horizon=%d, dt=%.3f",
    nx, nu, params_.prediction_horizon, params_.dt);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MPCController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  prev_u_.setZero();
  iteration_count_ = 0;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MPCController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

// ---- Interface configuration ----

controller_interface::InterfaceConfiguration
MPCController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = params_.command_interface_names;
  return config;
}

controller_interface::InterfaceConfiguration
MPCController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = params_.state_interface_names;
  return config;
}

// ---- Build fixed QP structure (called once in on_configure) ----

void MPCController::buildQPStructure()
{
  const int N = params_.prediction_horizon;
  const int nx = params_.state_dim;
  const int nu = params_.input_dim;
  const int n_vars = nu * N;

  Eigen::MatrixXd A = model_->getA();
  Eigen::MatrixXd B = model_->getB();

  // Precompute A powers: A_power[i] = A^i
  A_power_.resize(N + 1);
  A_power_[0] = Eigen::MatrixXd::Identity(nx, nx);
  for (int i = 1; i <= N; ++i) {
    A_power_[i] = A * A_power_[i - 1];
  }

  // Build S_x (nx*N × nu*N)
  // x_{k+1} = A^{k+1} * x0 + sum_{j=0}^{k} A^{k-j} * B * u_j
  // S_x block (k, j) = A^{k-j} * B  for j ≤ k, else 0
  S_x_ = Eigen::MatrixXd::Zero(nx * N, n_vars);
  for (int k = 0; k < N; ++k) {
    for (int j = 0; j <= k; ++j) {
      S_x_.block(k * nx, j * nu, nx, nu) = A_power_[k - j] * B;
    }
  }

  // Build S_du (n_vars × n_vars)
  // du_0 = u_0 - u_prev, du_k = u_k - u_{k-1}
  // S_du * u_seq = [I, 0; -I, I, 0; 0, -I, I; ...] * u_seq
  S_du_ = Eigen::MatrixXd::Zero(n_vars, n_vars);
  for (int k = 0; k < N; ++k) {
    S_du_.block(k * nu, k * nu, nu, nu) = Eigen::MatrixXd::Identity(nu, nu);
    if (k > 0) {
      S_du_.block(k * nu, (k - 1) * nu, nu, nu) = -Eigen::MatrixXd::Identity(nu, nu);
    }
  }

  // Build A_lin sparse matrix (fixed structure + values)
  // Decision variables: z = [u_0, ..., u_{N-1}, ε_0, ..., ε_{N-1}]
  // Constraints:
  //   [ +S_x   -E_vel ]        [ state upper: x_seq ≤ x_max ]
  //   [ -S_x   -E_vel ]   * z  [ state lower: -x_seq ≤ -x_min ]
  //   [  +I      0    ]        [ input: u_min ≤ z ≤ u_max ]
  //   [ +S_du    0    ]        [ rate: du_min ≤ S_du*z + rate_offset ≤ du_max ]
  //   [   0     -I    ]        [ ε ≥ 0: -ε ≤ 0 ]
  //
  // E_vel maps velocity state rows to slack variables: -1 at the corresponding
  // slack column for each velocity state at each timestep.
  int n_state_rows = 2 * nx * N;
  int n_input_rows = n_vars;
  int n_rate_rows = n_vars;
  int n_slack_rows = n_slack_;
  int n_total = n_state_rows + n_input_rows + n_rate_rows + n_slack_rows;
  int n_vars_total = n_vars + n_slack_;

  A_lin_.resize(n_total, n_vars_total);
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(
    S_x_.nonZeros() * 2 + n_vars + S_du_.nonZeros() + 2 * n_slack_ + n_slack_);

  // Build state-index → slack-sub-index mapping (-1 = not a velocity state)
  std::vector<int> state_to_slack(nx, -1);
  for (int j = 0; j < n_vel_; ++j) {
    state_to_slack[vel_indices_[j]] = j;
  }

  // State upper: +S_x, plus -1 in slack column for velocity states
  for (int k = 0; k < N; ++k) {
    for (int i = 0; i < nx; ++i) {
      int row = k * nx + i;
      // U columns
      for (int j = 0; j < n_vars; ++j) {
        double val = S_x_(row, j);
        if (std::abs(val) > 1e-15) {
          triplets.emplace_back(row, j, val);
        }
      }
      // Slack column for velocity states
      if (state_to_slack[i] >= 0) {
        int slack_col = n_vars + k * n_vel_ + state_to_slack[i];
        triplets.emplace_back(row, slack_col, -1.0);
      }
    }
  }

  // State lower: -S_x, plus -1 in slack column for velocity states
  for (int k = 0; k < N; ++k) {
    for (int i = 0; i < nx; ++i) {
      int row = nx * N + k * nx + i;
      // U columns
      for (int j = 0; j < n_vars; ++j) {
        double val = -S_x_(k * nx + i, j);
        if (std::abs(val) > 1e-15) {
          triplets.emplace_back(row, j, val);
        }
      }
      // Slack column for velocity states
      if (state_to_slack[i] >= 0) {
        int slack_col = n_vars + k * n_vel_ + state_to_slack[i];
        triplets.emplace_back(row, slack_col, -1.0);
      }
    }
  }

  // Input: +I (no slack columns)
  for (int i = 0; i < n_vars; ++i) {
    triplets.emplace_back(n_state_rows + i, i, 1.0);
  }

  // Rate: +S_du (no slack columns)
  for (int k = 0; k < n_vars; ++k) {
    for (int j = 0; j < n_vars; ++j) {
      double val = S_du_(k, j);
      if (std::abs(val) > 1e-15) {
        triplets.emplace_back(n_state_rows + n_input_rows + k, j, val);
      }
    }
  }

  // ε ≥ 0: -ε ≤ 0 → -I on slack columns
  for (int i = 0; i < n_slack_; ++i) {
    triplets.emplace_back(n_state_rows + n_input_rows + n_rate_rows + i,
                          n_vars + i, -1.0);
  }

  A_lin_.setFromTriplets(triplets.begin(), triplets.end());
  A_lin_.makeCompressed();
}

// ---- Core update ----

controller_interface::return_type MPCController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  ++iteration_count_;

  auto cycle_start = std::chrono::steady_clock::now();

  // Apply dynamic parameter updates (hot-reload Q/R/S weights)
  if (params_version_.load() != applied_params_version_) {
    auto latest = rt_params_.readFromRT();
    params_ = *latest;
    applied_params_version_ = params_version_.load();
    cost_matrix_dirty_ = true;
  }

  // Read current state and latest reference (once per cycle)
  Eigen::VectorXd x0 = readState();
  x_ref_ = *rt_ref_.readFromRT();

  // Build and solve QP
  bool ok = buildAndSolveQP(x0);
  bool hold_applied = false;

  // Extract control input (or safe fallback on failure)
  Eigen::VectorXd u0;
  if (ok) {
    Eigen::VectorXd solution = solver_->getSolution();
    u0 = solution.head(params_.input_dim);
    prev_u_ = u0;
  } else {
    auto diag = solver_->getDiagnostics();
    if (diag.solved_approximate) {
      Eigen::VectorXd solution = solver_->getSolution();
      u0 = solution.head(params_.input_dim);
      prev_u_ = u0;
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "MPC solve approximate at iteration %d (pri_res=%.2e, dua_res=%.2e)",
        iteration_count_, diag.pri_res, diag.dua_res);
    } else {
      hold_applied = true;
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "MPC solve failed at iteration %d (status=%d, iter=%d)",
        iteration_count_, diag.status, diag.iterations);
      u0 = prev_u_;
      if (u0.size() != params_.input_dim) {
        u0 = Eigen::VectorXd::Zero(params_.input_dim);
      }
    }
  }
  writeCommand(u0);

  auto cycle_end = std::chrono::steady_clock::now();
  int64_t cycle_time_us =
    std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start).count();
  int64_t expected_period_us = period.nanoseconds() / 1000;
  bool deadline_missed = cycle_time_us > expected_period_us;

  // Publish diagnostics (always, even on failure)
  if (rt_diag_pub_->trylock()) {
    auto & msg = rt_diag_pub_->msg_;
    auto diag = solver_->getDiagnostics();
    const int nx = params_.state_dim;
    const int nu = params_.input_dim;
    diag.cycle_time_us = cycle_time_us;

    // Extract slack diagnostics from solution (only valid when solve succeeded)
    double slack_max_vel = std::numeric_limits<double>::quiet_NaN();
    double slack_l1 = std::numeric_limits<double>::quiet_NaN();
    double slack_active = 0.0;
    if (n_slack_ > 0 && (diag.solved || diag.solved_approximate)) {
      int n_u_vars = nu * params_.prediction_horizon;
      Eigen::VectorXd sol = solver_->getSolution();
      if (sol.size() >= n_u_vars + n_slack_ && sol.allFinite()) {
        Eigen::VectorXd eps = sol.segment(n_u_vars, n_slack_);
        slack_max_vel = eps.maxCoeff();
        slack_l1 = eps.sum();
        for (int i = 0; i < eps.size(); ++i) {
          if (eps(i) > 1e-6) slack_active += 1.0;
        }
      }
    }

    msg.data.clear();
    msg.data.reserve(5 + 3 * nx + nu + 13);
    // [0] nx, [1] nu  (self-describing header)
    msg.data.push_back(static_cast<double>(nx));
    msg.data.push_back(static_cast<double>(nu));
    // [2] solve time, [3] iterations, [4] solved flag
    msg.data.push_back(static_cast<double>(diag.solve_time_us));
    msg.data.push_back(static_cast<double>(diag.iterations));
    msg.data.push_back(diag.solved ? 1.0 : 0.0);
    // [5 .. 5+nx-1] current state
    for (int i = 0; i < nx; ++i) msg.data.push_back(x0(i));
    // [5+nx .. 5+2*nx-1] reference
    for (int i = 0; i < nx; ++i) msg.data.push_back(x_ref_(i));
    // [5+2*nx .. 5+3*nx-1] tracking error (ref - state)
    for (int i = 0; i < nx; ++i) msg.data.push_back(x_ref_(i) - x0(i));
    // [5+3*nx .. 5+3*nx+nu-1] control input
    for (int i = 0; i < nu; ++i) msg.data.push_back(u0(i));
    // [5+3*nx+nu] objective value
    msg.data.push_back(diag.objective);
    // [5+3*nx+nu+1] setup time
    msg.data.push_back(static_cast<double>(diag.setup_time_us));
    // [5+3*nx+nu+2] total cycle time
    msg.data.push_back(static_cast<double>(diag.cycle_time_us));
    // [5+3*nx+nu+3] solver status code
    msg.data.push_back(static_cast<double>(diag.status));
    // [5+3*nx+nu+4] primal residual
    msg.data.push_back(diag.pri_res);
    // [5+3*nx+nu+5] dual residual
    msg.data.push_back(diag.dua_res);
    // [5+3*nx+nu+6] approximate flag (1.0 if solved_approximate)
    msg.data.push_back(diag.solved_approximate ? 1.0 : 0.0);
    // [5+3*nx+nu+7] hold applied flag
    msg.data.push_back(hold_applied ? 1.0 : 0.0);
    // [5+3*nx+nu+8] cycle index
    msg.data.push_back(static_cast<double>(iteration_count_));
    // [5+3*nx+nu+9] deadline missed flag
    msg.data.push_back(deadline_missed ? 1.0 : 0.0);
    // [5+3*nx+nu+10] max velocity slack
    msg.data.push_back(slack_max_vel);
    // [5+3*nx+nu+11] slack L1 norm
    msg.data.push_back(slack_l1);
    // [5+3*nx+nu+12] slack active count
    msg.data.push_back(slack_active);
    rt_diag_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

// ---- QP Builder (per-cycle update; workspace already set up in configure) ----

bool MPCController::buildAndSolveQP(const Eigen::VectorXd & x0)
{
  const int N = params_.prediction_horizon;
  const int nx = params_.state_dim;
  const int nu = params_.input_dim;
  const int n_vars = nu * N;
  const int n_vars_total = n_vars + n_slack_;

  // Reference stacked over horizon
  Eigen::VectorXd x_ref_stacked(nx * N);
  for (int k = 0; k < N; ++k) {
    x_ref_stacked.segment(k * nx, nx) = x_ref_;
  }

  // Compute free response from cached A_power: x_free[k] = A^{k+1} * x0
  Eigen::VectorXd x_free(nx * N);
  for (int k = 0; k < N; ++k) {
    x_free.segment(k * nx, nx) = A_power_[k + 1] * x0;
  }

  // Rate offset: first step uses u_0 - u_prev
  Eigen::VectorXd rate_offset = Eigen::VectorXd::Zero(n_vars);
  rate_offset.head(nu) = -prev_u_;

  // Build weight matrices from current parameters
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nx, nx);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(nu, nu);
  Eigen::MatrixXd S = Eigen::MatrixXd::Zero(nu, nu);
  for (int i = 0; i < nx; ++i) Q(i, i) = params_.Q_diag[i];
  for (int i = 0; i < nu; ++i) R(i, i) = params_.R_diag[i];
  for (int i = 0; i < nu; ++i) S(i, i) = params_.S_diag.empty() ? 0.0 : params_.S_diag[i];

  // Block-diagonal weight matrices over horizon
  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(nx * N, nx * N);
  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(n_vars, n_vars);
  Eigen::MatrixXd S_bar = Eigen::MatrixXd::Zero(n_vars, n_vars);
  for (int k = 0; k < N; ++k) {
    Q_bar.block(k * nx, k * nx, nx, nx) = Q;
    R_bar.block(k * nu, k * nu, nu, nu) = R;
    S_bar.block(k * nu, k * nu, nu, nu) = S;
  }

  // ---- Build cost: P and q ----
  // OSQP: min 1/2 * z^T * P * z + q^T * z
  //
  // MPC cost:
  //   J = (S_x*z + x_free - x_ref)^T Q_bar (S_x*z + x_free - x_ref)
  //     + z^T R_bar z
  //     + (S_du*z + rate_offset)^T S_bar (S_du*z + rate_offset)
  //     + ρ₁·1^T·ε + ρ₂·ε^T·ε    (soft velocity constraint slack penalty)
  //
  // Expanded:
  //   J = z_U^T * H * z_U + 2 * g^T * z_U
  //     + ρ₁·1^T·ε + ρ₂·ε^T·ε  + const
  //   H = S_x^T Q_bar S_x + R_bar + S_du^T S_bar S_du
  //   g = S_x^T Q_bar (x_free - x_ref) + S_du^T S_bar rate_offset
  //
  // OSQP needs:
  //   P = diag(2*H, 2*ρ₂*I)
  //   q = [2*g; ρ₁·1]

  Eigen::MatrixXd H = S_x_.transpose() * Q_bar * S_x_ +
                      R_bar +
                      S_du_.transpose() * S_bar * S_du_;

  Eigen::MatrixXd P_dense = Eigen::MatrixXd::Zero(n_vars_total, n_vars_total);
  P_dense.topLeftCorner(n_vars, n_vars) = 2.0 * H;
  if (n_slack_ > 0) {
    for (int i = 0; i < n_slack_; ++i) {
      P_dense(n_vars + i, n_vars + i) = 2.0 * slack_rho_2_;
    }
  }

  // q for U part: 2 * (S_x^T * Q_bar * (x_free - x_ref) + S_du^T * S_bar * rate_offset)
  Eigen::VectorXd dx = x_free - x_ref_stacked;
  Eigen::VectorXd q_vec(n_vars_total);
  q_vec.head(n_vars) = 2.0 * S_x_.transpose() * Q_bar * dx +
                       2.0 * S_du_.transpose() * S_bar * rate_offset;
  // q for slack part: ρ₁ · 1 (L1 penalty)
  if (n_slack_ > 0) {
    q_vec.tail(n_slack_).setConstant(slack_rho_1_);
  }

  // ---- Build constraint bounds l, u ----
  // A_lin is fixed (built in buildQPStructure), only l and u change per cycle.
  // State bounds are the same as before (velocity rows get slack -1 in A_lin,
  // so bound values don't change — the slack absorbs violations).

  Eigen::VectorXd x_min_vec(nx * N);
  Eigen::VectorXd x_max_vec(nx * N);
  for (int k = 0; k < N; ++k) {
    x_min_vec.segment(k * nx, nx) = Eigen::Map<const Eigen::VectorXd>(
      params_.state_lower.data(), nx);
    x_max_vec.segment(k * nx, nx) = Eigen::Map<const Eigen::VectorXd>(
      params_.state_upper.data(), nx);
  }

  int n_state_rows = 2 * nx * N;
  int n_input_rows = n_vars;
  int n_rate_rows = n_vars;
  int n_total = n_state_rows + n_input_rows + n_rate_rows + n_slack_;

  Eigen::VectorXd l(n_total);
  Eigen::VectorXd u(n_total);
  const double inf = std::numeric_limits<double>::infinity();

  // State upper: -inf ≤ S_x * z ≤ x_max - x_free
  // State lower: -inf ≤ -S_x * z ≤ -x_min + x_free
  // (Velocity rows also have slack -1 in A_lin, but bound values unchanged)
  for (int i = 0; i < nx * N; ++i) {
    l(i) = -inf;
    u(i) = x_max_vec(i) - x_free(i);

    l(nx * N + i) = -inf;
    u(nx * N + i) = -x_min_vec(i) + x_free(i);
  }

  // Input bounds (fixed, same as in configure)
  for (int k = 0; k < N; ++k) {
    for (int i = 0; i < nu; ++i) {
      l(n_state_rows + k * nu + i) = params_.input_lower[i];
      u(n_state_rows + k * nu + i) = params_.input_upper[i];
    }
  }

  // Rate bounds with dt scaling: du = input_rate * dt
  // l = du_min_vec - rate_offset, u = du_max_vec - rate_offset
  double dt = params_.dt;
  for (int k = 0; k < N; ++k) {
    for (int i = 0; i < nu; ++i) {
      double du_min_dt = dt * params_.input_rate_lower[i];
      double du_max_dt = dt * params_.input_rate_upper[i];
      l(n_state_rows + n_input_rows + k * nu + i) = du_min_dt - rate_offset(k * nu + i);
      u(n_state_rows + n_input_rows + k * nu + i) = du_max_dt - rate_offset(k * nu + i);
    }
  }

  // Slack non-negativity: -ε ≤ 0 → l=-inf, u=0
  for (int i = 0; i < n_slack_; ++i) {
    l(n_state_rows + n_input_rows + n_rate_rows + i) = -inf;
    u(n_state_rows + n_input_rows + n_rate_rows + i) = 0.0;
  }

  // ---- Update solver and solve (no re-setup) ----
  try {
    solver_->updateGradient(q_vec);
    solver_->updateBounds(l, u);

    // Rebuild and update P only when weights have changed (cost_matrix_dirty_).
    // Build P_sparse with ALL upper-triangular entries to guarantee identical
    // sparsity structure (required by OSQP osqp_update_P).
    if (cost_matrix_dirty_) {
      Eigen::SparseMatrix<double> P_sparse(n_vars_total, n_vars_total);
      std::vector<Eigen::Triplet<double>> p_triplets;
      p_triplets.reserve(n_vars_total * (n_vars_total + 1) / 2);
      // U block (upper triangular)
      for (int col = 0; col < n_vars; ++col) {
        for (int row = 0; row <= col; ++row) {
          p_triplets.emplace_back(row, col, P_dense(row, col));
        }
      }
      // Slack block (diagonal only)
      for (int i = 0; i < n_slack_; ++i) {
        int idx = n_vars + i;
        p_triplets.emplace_back(idx, idx, P_dense(idx, idx));
      }
      P_sparse.setFromTriplets(p_triplets.begin(), p_triplets.end());
      P_sparse.makeCompressed();
      solver_->updateCostMatrix(P_sparse);
      cost_matrix_dirty_ = false;
    }

    return solver_->solve();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "QP update or solve failed: %s", e.what());
    return false;
  }
}

// ---- Helpers ----

Eigen::VectorXd MPCController::readState()
{
  Eigen::VectorXd x(params_.state_dim);
  for (int i = 0; i < params_.state_dim; ++i) {
    x(i) = state_interfaces_[i].get_value();
  }
  return x;
}

void MPCController::writeCommand(const Eigen::VectorXd & u0)
{
  for (int i = 0; i < params_.input_dim; ++i) {
    if (!command_interfaces_[i].set_value(u0(i))) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to write command interface %d", i);
    }
  }
}

rcl_interfaces::msg::SetParametersResult MPCController::parameterUpdateCallback(
  const std::vector<rclcpp::Parameter> & params)
{
  MpcParams new_params = params_;
  bool changed = false;

  for (const auto & p : params) {
    if (p.get_name() == "Q_diag" &&
        p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      auto v = p.as_double_array();
      if (static_cast<int>(v.size()) != params_.state_dim) {
        RCLCPP_WARN(get_node()->get_logger(),
          "Q_diag size %zu != state_dim %d, ignoring", v.size(), params_.state_dim);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        return result;
      }
      new_params.Q_diag = v;
      changed = true;
    } else if (p.get_name() == "R_diag" &&
               p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      auto v = p.as_double_array();
      if (static_cast<int>(v.size()) != params_.input_dim) {
        RCLCPP_WARN(get_node()->get_logger(),
          "R_diag size %zu != input_dim %d, ignoring", v.size(), params_.input_dim);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        return result;
      }
      new_params.R_diag = v;
      changed = true;
    } else if (p.get_name() == "S_diag" &&
               p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      auto v = p.as_double_array();
      if (static_cast<int>(v.size()) != params_.input_dim) {
        RCLCPP_WARN(get_node()->get_logger(),
          "S_diag size %zu != input_dim %d, ignoring", v.size(), params_.input_dim);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        return result;
      }
      new_params.S_diag = v;
      changed = true;
    }
  }

  if (changed) {
    rt_params_.writeFromNonRT(new_params);
    params_version_.fetch_add(1);
    RCLCPP_INFO(get_node()->get_logger(), "Updated MPC weights (version %ld)",
                params_version_.load());
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

}  // namespace mpc_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mpc_controller::MPCController, controller_interface::ControllerInterface)
