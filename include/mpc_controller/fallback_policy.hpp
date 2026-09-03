// Deterministic fallback decision seam (C1, RRBot plan weeks 1-2).
//
// Pure decision logic: given the last solver diagnostics + candidate
// solution, decide whether the candidate may be written to the command
// interface.  Rules (plan: "QP timeout/infeasible 时进入确定性 fallback，
// 绝不让未验证 command 上接口"):
//   * OSQP solved            -> apply solution;
//   * solved_approximate     -> apply (controller keeps the approximate
//                                solve, as v0.2.1 did) but flag it;
//   * failed / non-finite    -> ZERO command (deterministic), never reuse
//                                an unvalidated previous command.
//
// NOTE: this is a testable seam; wiring it into MPCController::update() is
// the C2 realtime step (controller behaviour is intentionally unchanged in
// C1 per the plan).
#ifndef MPC_CONTROLLER__FALLBACK_POLICY_HPP_
#define MPC_CONTROLLER__FALLBACK_POLICY_HPP_

#include <cmath>

#include <Eigen/Dense>

#include "mpc_controller/types.hpp"

namespace mpc_controller
{

enum class FallbackAction
{
  kApplySolution,    // write the (validated) QP solution
  kApplyApproximate, // write the approximate solution; mark degraded
  kZeroCommand,      // deterministic zero command (safe fallback)
};

struct FallbackDecision
{
  FallbackAction action{FallbackAction::kZeroCommand};
  bool solution_finite{false};
  bool approximate{false};
  int solver_status{-1};
};

/// Decide what the controller may write this cycle.  Pure, deterministic.
inline FallbackDecision decideFallback(
  const SolverDiagnostics & diag,
  const Eigen::VectorXd & solution)
{
  FallbackDecision out;
  out.solver_status = diag.status;
  out.solution_finite = solution.allFinite();

  if (diag.solved && out.solution_finite) {
    out.action = FallbackAction::kApplySolution;
  } else if (diag.solved_approximate && out.solution_finite) {
    out.action = FallbackAction::kApplyApproximate;
    out.approximate = true;
  } else {
    // Failed solve or non-finite candidate: deterministic zero command.
    // Never reuse an unvalidated previous command.
    out.action = FallbackAction::kZeroCommand;
  }
  return out;
}

/// The only command vector allowed to reach the hardware interface.
/// Returns the vector to write (solution or zeros depending on decision).
inline Eigen::VectorXd safeCommand(const FallbackDecision & d, const Eigen::VectorXd & solution)
{
  if (d.action == FallbackAction::kApplySolution ||
      d.action == FallbackAction::kApplyApproximate) {
    return solution;
  }
  return Eigen::VectorXd::Zero(solution.size());
}

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__FALLBACK_POLICY_HPP_
