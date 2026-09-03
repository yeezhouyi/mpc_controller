// Reference validation seams (C1): dimension / NaN / freshness checks.
//
// The RRBot controller receives references on `~/reference` as
// std_msgs/Float64MultiArray (nx doubles, no stamp).  These pure helpers
// give the update path deterministic guards:
//   * dimension must equal the state dimension (nx),
//   * every element must be finite (no NaN/Inf),
//   * freshness is expressed as an age in seconds (seam; wiring a real
//     stamp into the message/contract is a C2 interface decision).
//
// Testable without ROS; wiring into MPCController::update() is C2.
#ifndef MPC_CONTROLLER__REFERENCE_VALIDATION_HPP_
#define MPC_CONTROLLER__REFERENCE_VALIDATION_HPP_

#include <cmath>
#include <cstddef>
#include <vector>

namespace mpc_controller
{

inline bool validReferenceDimension(const std::vector<double> & ref, std::size_t nx)
{
  return ref.size() == nx;
}

inline bool referenceAllFinite(const std::vector<double> & ref)
{
  for (double v : ref) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

/// A reference is fresh when its age (now - stamp) is within [0, max_age].
/// Negative ages (future stamps) are treated as stale to avoid trusting
/// messages that claim to come from the future.
inline bool isReferenceFresh(double age_seconds, double max_age_seconds)
{
  return age_seconds >= 0.0 && age_seconds <= max_age_seconds;
}

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__REFERENCE_VALIDATION_HPP_
