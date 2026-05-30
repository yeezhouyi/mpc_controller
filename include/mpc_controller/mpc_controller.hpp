#ifndef MPC_CONTROLLER__MPC_CONTROLLER_HPP_
#define MPC_CONTROLLER__MPC_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "mpc_controller/types.hpp"
#include "mpc_controller/solver_base.hpp"
#include "mpc_controller/model_base.hpp"

namespace mpc_controller
{

class MPCController : public controller_interface::ControllerInterface
{
public:
  MPCController();
  ~MPCController() override = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  /// Build fixed QP structure (S_x, S_du, A_lin) from model matrices.
  /// Called once during on_configure().
  void buildQPStructure();

  /// Build per-cycle cost and constraints, update solver, and solve.
  /// @param x0  Current state (read once in update() and passed down).
  bool buildAndSolveQP(const Eigen::VectorXd & x0);

  /// Read current state from hardware interfaces.
  Eigen::VectorXd readState();

  /// Write optimal control to command interfaces.
  void writeCommand(const Eigen::VectorXd & u0);

  /// Callback for dynamic parameter updates (Q/R/S weights).
  rcl_interfaces::msg::SetParametersResult parameterUpdateCallback(
    const std::vector<rclcpp::Parameter> & params);

  // --- Parameters ---
  MpcParams params_;

  // --- Sub-components ---
  std::unique_ptr<SolverBase> solver_;
  std::unique_ptr<ModelBase> model_;

  // --- State ---
  Eigen::VectorXd prev_u_;   // previous input (for rate constraints)
  Eigen::VectorXd x_ref_;    // current reference state
  int iteration_count_{0};

  // --- Dynamic parameters ---
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  realtime_tools::RealtimeBuffer<MpcParams> rt_params_;
  std::atomic<int64_t> params_version_{0};
  int64_t applied_params_version_{0};

  // --- Reference subscriber ---
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_sub_;
  realtime_tools::RealtimeBuffer<Eigen::VectorXd> rt_ref_;

  // --- Diagnostics publisher ---
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diag_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_diag_pub_;

  // --- Soft constraint tracking ---
  int n_vel_{0};                   // number of velocity states (inferred)
  int n_slack_{0};                 // n_vel * N (total slack variables)
  std::vector<int> vel_indices_;   // state indices that are velocities
  double slack_rho_1_{10000.0};
  double slack_rho_2_{1000.0};
  Eigen::VectorXd slack_grad_q_;   // cached linear term for slack (ρ₁·1)

  // --- Hot-update tracking ---
  bool cost_matrix_dirty_{false};  // true when Q/R/S changed and P needs rebuild

  // --- Cached QP structure (built once in configure) ---
  Eigen::MatrixXd S_x_;           // (nx*N × nu*N) forced-response matrix
  Eigen::MatrixXd S_du_;          // (nu*N × nu*N) input-difference matrix
  Eigen::SparseMatrix<double> A_lin_;  // sparse linear constraint matrix
  std::vector<Eigen::MatrixXd> A_power_;  // A^i for i = 0..N
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__MPC_CONTROLLER_HPP_
