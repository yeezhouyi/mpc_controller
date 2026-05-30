#include "mpc_controller/linear_model.hpp"
#include <stdexcept>

namespace mpc_controller
{

void LinearModel::initialize(const MpcParams & params)
{
  int nx = params.state_dim;
  int nu = params.input_dim;

  if (params.A_data.size() != static_cast<size_t>(nx * nx)) {
    throw std::invalid_argument(
      "LinearModel: A_data size " + std::to_string(params.A_data.size()) +
      " does not match state_dim^2 = " + std::to_string(nx * nx));
  }
  if (params.B_data.size() != static_cast<size_t>(nx * nu)) {
    throw std::invalid_argument(
      "LinearModel: B_data size " + std::to_string(params.B_data.size()) +
      " does not match state_dim * input_dim = " + std::to_string(nx * nu));
  }

  A_ = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
    params.A_data.data(), nx, nx);
  B_ = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
    params.B_data.data(), nx, nu);
}

Eigen::MatrixXd LinearModel::predict(
  const Eigen::VectorXd & x0,
  const Eigen::VectorXd & u_seq) const
{
  const int nx = A_.rows();
  const int nu = B_.cols();
  const int N = u_seq.size() / nu;

  Eigen::MatrixXd X(nx, N + 1);
  X.col(0) = x0;

  for (int k = 0; k < N; ++k) {
    Eigen::VectorXd uk = u_seq.segment(k * nu, nu);
    X.col(k + 1) = A_ * X.col(k) + B_ * uk;
  }

  return X;
}

}  // namespace mpc_controller
