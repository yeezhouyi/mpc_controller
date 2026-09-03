// Copyright 2026 zhouyi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// Unit tests for the configured discrete LTI model (plan C1:
/// test_model_discretization.cpp — the core class is LinearModel).
///
/// These tests exercise the model without any ROS: dimension validation,
/// row-major matrix loading and the equivalence of LinearModel::predict()
/// with a manual recursion of x[k+1] = A x[k] + B u[k].

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "mpc_controller/linear_model.hpp"
#include "mpc_controller/types.hpp"

namespace mpc_controller
{
namespace
{

/// RRBot double-integrator config: nx=4 (q1,q1dot,q2,q2dot), nu=2, dt=0.01.
MpcParams rrbotParams()
{
  MpcParams p;
  p.state_dim = 4;
  p.input_dim = 2;
  p.prediction_horizon = 20;
  p.dt = 0.01;
  p.Q_diag = {100.0, 1.0, 100.0, 1.0};
  p.R_diag = {0.1, 0.1};
  p.A_data = {
    1.0, 0.01, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.01,
    0.0, 0.0, 0.0, 1.0};
  p.B_data = {
    0.0, 0.0,
    0.01, 0.0,
    0.0, 0.0,
    0.0, 0.01};
  return p;
}

template<typename Fn>
bool throwsInvalidArgument(Fn && fn)
{
  // ROS2's gtest_vendor is built without exceptions, which silently breaks
  // EXPECT_THROW — use explicit try/catch (see the tunnel project notes).
  try {
    fn();
    return false;
  } catch (const std::invalid_argument &) {
    return true;
  } catch (...) {
    return false;
  }
}

TEST(TestLinearModel, LoadsRrbotDoubleIntegrator)
{
  const auto params = rrbotParams();
  LinearModel model;
  ASSERT_FALSE(throwsInvalidArgument([&]() {model.initialize(params);}));
  EXPECT_EQ(model.getStateDim(), 4);
  EXPECT_EQ(model.getInputDim(), 2);

  // Row-major A/B must be mapped so that A(q1 <- q1dot) == dt etc.
  const auto A = model.getA();
  const auto B = model.getB();
  EXPECT_DOUBLE_EQ(A(0, 1), 0.01);
  EXPECT_DOUBLE_EQ(A(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(A(2, 3), 0.01);
  EXPECT_DOUBLE_EQ(A(3, 3), 1.0);
  EXPECT_DOUBLE_EQ(B(1, 0), 0.01);
  EXPECT_DOUBLE_EQ(B(3, 1), 0.01);
  EXPECT_DOUBLE_EQ(B(0, 0), 0.0);
}

TEST(TestLinearModel, RejectsDimensionMismatch)
{
  auto params = rrbotParams();
  params.A_data.pop_back();  // wrong size
  LinearModel model;
  EXPECT_TRUE(throwsInvalidArgument([&]() {model.initialize(params);}));

  params = rrbotParams();
  params.B_data.push_back(0.0);  // wrong size
  EXPECT_TRUE(throwsInvalidArgument([&]() {model.initialize(params);}));
}

TEST(TestLinearModel, ZeroInputKeepsStateConstant)
{
  const auto params = rrbotParams();
  LinearModel model;
  model.initialize(params);

  Eigen::VectorXd x0(4);
  x0 << 0.3, 0.5, -0.2, -1.0;
  Eigen::VectorXd u_seq = Eigen::VectorXd::Zero(2 * 5);  // N = 5, u = 0
  const auto X = model.predict(x0, u_seq);

  EXPECT_EQ(X.rows(), 4);
  EXPECT_EQ(X.cols(), 6);
  for (int k = 0; k < X.cols(); ++k) {
    EXPECT_NEAR(X(0, k), x0(0), 1e-12);
    EXPECT_NEAR(X(1, k), x0(1), 1e-12);
    EXPECT_NEAR(X(2, k), x0(2), 1e-12);
    EXPECT_NEAR(X(3, k), x0(3), 1e-12);
  }
}

TEST(TestLinearModel, PredictMatchesManualRecursion)
{
  const auto params = rrbotParams();
  LinearModel model;
  model.initialize(params);
  const double dt = params.dt;

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
  const int N = 8;
  Eigen::VectorXd u_seq(2 * N);
  for (int k = 0; k < N; ++k) {
    u_seq(2 * k) = 1.0;      // effort on joint 1
    u_seq(2 * k + 1) = -0.5;  // effort on joint 2
  }

  const auto X = model.predict(x0, u_seq);

  // Manual recursion of the double integrator.
  Eigen::VectorXd x = x0;
  for (int k = 0; k < N; ++k) {
    const double u1 = u_seq(2 * k);
    const double u2 = u_seq(2 * k + 1);
    Eigen::VectorXd xnext(4);
    xnext(0) = x(0) + dt * x(1);
    xnext(1) = x(1) + dt * u1;
    xnext(2) = x(2) + dt * x(3);
    xnext(3) = x(3) + dt * u2;
    for (int i = 0; i < 4; ++i) {
      EXPECT_NEAR(X(i, k + 1), xnext(i), 1e-12);
    }
    x = xnext;
  }

  // Closed-form sanity: after 8 steps of constant u1=1 from rest,
  // v1 = 8*dt and q1 = dt^2 * 8*7/2.
  EXPECT_NEAR(X(1, N), 8.0 * dt, 1e-12);
  EXPECT_NEAR(X(0, N), dt * dt * 28.0, 1e-12);
  EXPECT_NEAR(X(3, N), -0.5 * 8.0 * dt, 1e-12);
}

TEST(TestLinearModel, MultiJointSynchronisedCommand)
{
  const auto params = rrbotParams();
  LinearModel model;
  model.initialize(params);

  // Same constant effort on both joints -> both axes behave identically.
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
  const int N = 4;
  Eigen::VectorXd u_seq(2 * N);
  u_seq.setConstant(0.2);
  const auto X = model.predict(x0, u_seq);
  for (int k = 0; k <= N; ++k) {
    EXPECT_NEAR(X(0, k), X(2, k), 1e-12);
    EXPECT_NEAR(X(1, k), X(3, k), 1e-12);
  }
}

}  // namespace
}  // namespace mpc_controller
