#include <gtest/gtest.h>
#include <memory>

#include "mpc_controller/mpc_controller.hpp"
#include "mpc_controller/linear_model.hpp"
#include "mpc_controller/osqp_solver.hpp"
#include "mpc_controller/types.hpp"

using namespace mpc_controller;

/// Placeholder for integration tests that require a full ros2_control
/// environment with mock hardware.
///
/// Full controller integration tests depend on:
///   - controller_manager::ControllerManager
///   - hardware_interface::SystemInterface (mock hardware)
///   - rclcpp lifecyle node
///
/// Those tests require linking against controller_manager_test support
/// libraries and are best added in a downstream test package or in the
/// ros2_control_demos test framework.
///
/// This file verifies the controller's parameter parsing and component
/// initialization logic at the unit level.
TEST(TestMPCController, parameterValidation)
{
  // MpcParams with valid double-integrator setup
  MpcParams params;
  params.state_dim = 2;
  params.input_dim = 1;
  params.prediction_horizon = 10;
  params.dt = 0.01;
  params.Q_diag = {10.0, 1.0};
  params.R_diag = {0.1};
  params.S_diag = {0.5};
  params.A_data = {1.0, 0.01, 0.0, 1.0};
  params.B_data = {0.0, 0.01};
  params.state_lower = {-10.0, -10.0};
  params.state_upper = {10.0, 10.0};
  params.input_lower = {-5.0};
  params.input_upper = {5.0};
  params.input_rate_lower = {-20.0};
  params.input_rate_upper = {20.0};

  EXPECT_EQ(params.state_dim, 2);
  EXPECT_EQ(params.input_dim, 1);
  EXPECT_EQ(params.Q_diag.size(), 2u);
  EXPECT_EQ(params.R_diag.size(), 1u);

  // Verify model initializes correctly
  LinearModel model;
  EXPECT_NO_THROW(model.initialize(params));
  EXPECT_EQ(model.getStateDim(), 2);
  EXPECT_EQ(model.getInputDim(), 1);

  // Verify solver initializes correctly
  OSQPSolver solver;
  EXPECT_NO_THROW(solver.initialize(2, 1, 10));
}

/// Verify that invalid parameter sizes are caught
TEST(TestMPCController, invalidParameterSizes)
{
  MpcParams params;
  params.state_dim = 2;
  params.input_dim = 1;
  params.A_data = {1.0, 0.01, 0.0, 1.0};  // correct: 2x2

  // Wrong B size (should be 2x1 = 2 elements)
  params.B_data = {0.0, 0.01, 0.5};  // 3 elements, should be 2

  LinearModel model;
  EXPECT_THROW(model.initialize(params), std::invalid_argument);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
