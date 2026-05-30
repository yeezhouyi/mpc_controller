#ifndef MPC_CONTROLLER__TYPES_HPP_
#define MPC_CONTROLLER__TYPES_HPP_

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace mpc_controller
{

using State = Eigen::VectorXd;
using Input = Eigen::VectorXd;
using Reference = Eigen::VectorXd;

struct Constraints
{
  State state_lower;
  State state_upper;
  Input input_lower;
  Input input_upper;
  Input input_rate_lower;
  Input input_rate_upper;
};

struct SolverDiagnostics
{
  int64_t solve_time_us{0};
  int64_t setup_time_us{0};
  int64_t cycle_time_us{0};
  int iterations{0};
  int status{-1};   // OSQP status code
  bool solved{false};               // true when OSQP reports kSolved
  bool solved_approximate{false};   // true when kMaxIter but residuals acceptable
  double objective{0.0};
  double pri_res{0.0};   // primal residual
  double dua_res{0.0};   // dual residual
};

struct MpcParams
{
  int prediction_horizon{20};
  int state_dim{0};
  int input_dim{0};
  double dt{0.01};

  // Weight matrices (diagonal)
  std::vector<double> Q_diag;
  std::vector<double> R_diag;
  std::vector<double> S_diag;  // input rate weight

  // Model matrices (row-major)
  std::vector<double> A_data;
  std::vector<double> B_data;

  // Constraint bounds
  std::vector<double> state_lower;
  std::vector<double> state_upper;
  std::vector<double> input_lower;
  std::vector<double> input_upper;
  std::vector<double> input_rate_lower;
  std::vector<double> input_rate_upper;

  // ROS2 interface mapping
  std::vector<std::string> state_interface_names;
  std::vector<std::string> command_interface_names;

  // Soft constraint parameters (for velocity bounds)
  std::vector<int> velocity_indices;  // state indices to soften (e.g. [1, 3])
  double slack_rho_1{100.0};         // L1 slack penalty coefficient
  double slack_rho_2{10.0};          // L2 slack penalty coefficient

  // Solver settings
  int max_iterations{4000};
  double abs_tol{1e-4};
  double rel_tol{1e-4};
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__TYPES_HPP_
