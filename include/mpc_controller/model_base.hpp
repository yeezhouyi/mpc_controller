#ifndef MPC_CONTROLLER__MODEL_BASE_HPP_
#define MPC_CONTROLLER__MODEL_BASE_HPP_

#include <Eigen/Dense>
#include "mpc_controller/types.hpp"

namespace mpc_controller
{

/// Abstract base class for MPC prediction models.
class ModelBase
{
public:
  ModelBase() = default;
  virtual ~ModelBase() = default;

  /// Initialize the model with parameters.
  virtual void initialize(const MpcParams & params) = 0;

  /// Get the state dimension.
  virtual int getStateDim() const = 0;

  /// Get the input dimension.
  virtual int getInputDim() const = 0;

  /// Predict the state trajectory given initial state and input sequence.
  /// @param x0  Initial state (state_dim)
  /// @param u_seq  Input sequence over horizon (input_dim * horizon)
  /// @return Predicted state sequence (state_dim * (horizon+1))
  virtual Eigen::MatrixXd predict(
    const Eigen::VectorXd & x0,
    const Eigen::VectorXd & u_seq) const = 0;

  /// Get the state-transition matrix A (state_dim x state_dim).
  virtual Eigen::MatrixXd getA() const = 0;

  /// Get the input matrix B (state_dim x input_dim).
  virtual Eigen::MatrixXd getB() const = 0;
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__MODEL_BASE_HPP_
