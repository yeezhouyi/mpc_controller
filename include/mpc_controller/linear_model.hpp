#ifndef MPC_CONTROLLER__LINEAR_MODEL_HPP_
#define MPC_CONTROLLER__LINEAR_MODEL_HPP_

#include <Eigen/Dense>
#include "mpc_controller/model_base.hpp"

namespace mpc_controller
{

/// Linear time-invariant (LTI) state-space model.
///
///   x_{k+1} = A x_k + B u_k
///
/// with A (state_dim x state_dim) and B (state_dim x input_dim)
/// loaded from YAML configuration.
class LinearModel : public ModelBase
{
public:
  LinearModel() = default;
  ~LinearModel() override = default;

  void initialize(const MpcParams & params) override;

  int getStateDim() const override { return A_.rows(); }
  int getInputDim() const override { return B_.cols(); }

  Eigen::MatrixXd predict(
    const Eigen::VectorXd & x0,
    const Eigen::VectorXd & u_seq) const override;

  Eigen::MatrixXd getA() const override { return A_; }
  Eigen::MatrixXd getB() const override { return B_; }

private:
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__LINEAR_MODEL_HPP_
