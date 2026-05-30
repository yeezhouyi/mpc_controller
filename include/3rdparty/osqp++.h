#ifndef OSQP_CXX_WRAPPER_H
#define OSQP_CXX_WRAPPER_H

/// Minimal C++ wrapper around the OSQP C API (v0.6.x / ROS Jazzy vendor).
///
/// Provides RAII lifetime management for the OSQP C workspace and
/// type-safe Eigen-based access to problem data and solution.

#include <Eigen/Sparse>
#include <Eigen/Dense>

#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" {
#include "osqp.h"
}

namespace osqp {

/// Error codes returned by OSQP C API calls.
enum class OSQPError : c_int {
  kNoError                = 0,
  kSetupFailed            = 1,
  kSolveFailed            = 2,
  kUpdateFailed           = 3,
  kWarmStartFailed        = 4,
  kWorkspaceNotInitialized = 5,
  kInvalidInput           = 6,
};

/// Problem data stored as Eigen objects, convertible to OSQP CSC format.
struct OSQPInstance {
  Eigen::SparseMatrix<double> problem_mat;    // P (upper triangular)
  Eigen::VectorXd             gradient;       // q
  Eigen::SparseMatrix<double> constraint_mat; // A
  Eigen::VectorXd             lower_bounds;   // l
  Eigen::VectorXd             upper_bounds;   // u
};

/// QP solver settings (thin wrapper around OSQPSettings).
struct OSQPSettings {
  c_int   adaptive_rho               = 1;
  c_int   polish                     = 0;
  c_int   verbose                    = 0;
  c_float eps_abs                    = 1.0e-3;
  c_float eps_rel                    = 1.0e-3;
  c_int   max_iter                   = 4000;
  c_int   warm_start                 = 1;
  // optional extras
  c_float rho                        = 0.1;
  c_float sigma                      = 1.0e-6;
  c_int   scaling                    = 10;
  c_int   adaptive_rho_interval      = 0;
  c_float adaptive_rho_tolerance     = 5.0;
  c_float alpha                      = 1.6;
  c_int   polish_refine_iter         = 3;
  c_int   scaled_termination         = 0;
  c_int   check_termination          = 25;
  c_float delta                      = 1.0e-6;
  c_float eps_prim_inf               = 1.0e-4;
  c_float eps_dual_inf               = 1.0e-4;
  c_float time_limit                 = 0.0;
  enum linsys_solver_type linsys_solver = QDLDL_SOLVER;

  void apply(::OSQPSettings *out) const {
    osqp_set_default_settings(out);
    out->adaptive_rho               = adaptive_rho;
    out->polish                     = polish;
    out->verbose                    = verbose;
    out->eps_abs                    = eps_abs;
    out->eps_rel                    = eps_rel;
    out->max_iter                   = max_iter;
    out->warm_start                 = warm_start;
    out->rho                        = rho;
    out->sigma                      = sigma;
    out->scaling                    = scaling;
    out->adaptive_rho_interval      = adaptive_rho_interval;
    out->adaptive_rho_tolerance     = adaptive_rho_tolerance;
    out->alpha                      = alpha;
    out->polish_refine_iter         = polish_refine_iter;
    out->scaled_termination         = scaled_termination;
    out->check_termination          = check_termination;
    out->delta                      = delta;
    out->eps_prim_inf               = eps_prim_inf;
    out->eps_dual_inf               = eps_dual_inf;
    out->linsys_solver              = linsys_solver;
  }
};

/// Convert an Eigen sparse matrix (col-major) to OSQP CSC.
static csc * eigen_to_csc(const Eigen::SparseMatrix<double> &mat) {
  const auto inner = mat.innerIndexPtr();
  const auto outer = mat.outerIndexPtr();
  const auto vals  = mat.valuePtr();

  c_int nzmax = static_cast<c_int>(mat.nonZeros());
  c_int m     = static_cast<c_int>(mat.rows());
  c_int n     = static_cast<c_int>(mat.cols());

  auto *M = static_cast<csc *>(std::malloc(sizeof(csc)));
  if (!M) throw std::bad_alloc();

  M->nzmax = nzmax;
  M->m     = m;
  M->n     = n;
  M->nz    = -1; // compressed column

  // NOTE: Eigen index pointers are int (4 bytes), c_int is 8 bytes on 64-bit.
  // Use element-wise copy, NOT memcpy with sizeof(c_int).
  M->p = static_cast<c_int *>(std::malloc(sizeof(c_int) * (static_cast<size_t>(n) + 1)));
  M->i = static_cast<c_int *>(std::malloc(sizeof(c_int) * static_cast<size_t>(nzmax)));
  M->x = static_cast<c_float *>(std::malloc(sizeof(c_float) * static_cast<size_t>(nzmax)));
  if (!M->p || !M->i || !M->x) throw std::bad_alloc();

  for (c_int i = 0; i <= n; ++i)    M->p[i] = static_cast<c_int>(outer[i]);
  for (c_int i = 0; i < nzmax; ++i) M->i[i] = static_cast<c_int>(inner[i]);
  for (c_int i = 0; i < nzmax; ++i) M->x[i] = static_cast<c_float>(vals[i]);
  return M;
}

/// RAII solver wrapping the OSQP C workspace.
///
/// Manages the full lifecycle: setup → (update gradient/bounds/P | solve)* → cleanup.
class OSQPSolver {
public:
  OSQPSolver() noexcept = default;

  // Non-copyable, movable
  OSQPSolver(const OSQPSolver &) = delete;
  OSQPSolver & operator=(const OSQPSolver &) = delete;
  OSQPSolver(OSQPSolver &&other) noexcept
    : work_(other.work_), owns_(other.owns_)
  {
    other.work_ = nullptr;
    other.owns_ = false;
  }
  OSQPSolver & operator=(OSQPSolver &&other) noexcept {
    if (this != &other) { cleanup(); work_ = other.work_; owns_ = other.owns_; other.work_ = nullptr; other.owns_ = false; }
    return *this;
  }

  ~OSQPSolver() { cleanup(); }

  enum class Status : c_int {
    kSolved    = 1,
    kMaxIter   = -2,
    kPrimInfeas= -3,
    kDualInfeas= -4,
    kNotRun    = 0,
  };

  /// Initialise solver from instance and settings.
  /// On success the workspace is ready; on failure the workspace is cleaned up.
  OSQPError Init(const OSQPInstance &inst, const OSQPSettings &settings) {
    cleanup();

    // Convert Eigen → CSC
    csc *P_csc = eigen_to_csc(inst.problem_mat);
    csc *A_csc = eigen_to_csc(inst.constraint_mat);

    // Build OSQPData
    c_int n = static_cast<c_int>(inst.problem_mat.cols());
    c_int m = static_cast<c_int>(inst.constraint_mat.rows());

    data_.n = n;
    data_.m = m;
    data_.P = P_csc;
    data_.A = A_csc;
    data_.q = static_cast<c_float *>(std::malloc(sizeof(c_float) * static_cast<size_t>(n)));
    data_.l = static_cast<c_float *>(std::malloc(sizeof(c_float) * static_cast<size_t>(m)));
    data_.u = static_cast<c_float *>(std::malloc(sizeof(c_float) * static_cast<size_t>(m)));
    if (!data_.q || !data_.l || !data_.u) throw std::bad_alloc();

    Eigen::Map<Eigen::VectorXd>(data_.q, n) = inst.gradient;
    Eigen::Map<Eigen::VectorXd>(data_.l, m) = inst.lower_bounds;
    Eigen::Map<Eigen::VectorXd>(data_.u, m) = inst.upper_bounds;

    // Sanitize bounds: OSQP cannot handle ±Inf, replace with large finite values
    for (c_int i = 0; i < m; ++i) {
      if (!std::isfinite(data_.l[i])) data_.l[i] = data_.l[i] < 0 ? -OSQP_INFTY : OSQP_INFTY;
      if (!std::isfinite(data_.u[i])) data_.u[i] = data_.u[i] < 0 ? -OSQP_INFTY : OSQP_INFTY;
    }

    // Apply settings
    ::OSQPSettings c_settings;
    settings.apply(&c_settings);

    // Setup
    c_int exitflag = osqp_setup(&work_, &data_, &c_settings);
    if (exitflag == 0) {
      owns_ = true;
      return OSQPError::kNoError;
    }
    // Setup failed — clean up and return error
    owns_ = false;
    cleanup();
    return OSQPError::kSetupFailed;
  }

  /// Solve the QP.
  OSQPError Solve() {
    if (!work_) return OSQPError::kWorkspaceNotInitialized;
    c_int exitflag = osqp_solve(work_);
    if (exitflag == 0) return OSQPError::kNoError;
    return OSQPError::kSolveFailed;
  }

  /// Get solver status (as integer code).
  c_int GetStatus() const {
    if (!work_ || !work_->info) return static_cast<c_int>(Status::kNotRun);
    return work_->info->status_val;
  }

  /// Get iteration count.
  c_int GetIterations() const {
    if (!work_ || !work_->info) return 0;
    return work_->info->iter;
  }

  /// Get primal objective value at the solution.
  double GetObjective() const {
    if (!work_ || !work_->info) return 0.0;
    return static_cast<double>(work_->info->obj_val);
  }

  /// Get norm of primal residual (constraint satisfaction measure).
  double GetPrimalResidual() const {
    if (!work_ || !work_->info) return 0.0;
    return static_cast<double>(work_->info->pri_res);
  }

  /// Get norm of dual residual (optimality measure).
  double GetDualResidual() const {
    if (!work_ || !work_->info) return 0.0;
    return static_cast<double>(work_->info->dua_res);
  }

  /// Get primal solution vector.
  Eigen::VectorXd GetPrimalSolution() const {
    if (!work_ || !work_->solution) return {};
    c_int n = work_->data->n;
    return Eigen::Map<Eigen::VectorXd>(work_->solution->x, n);
  }

  /// Update both lower and upper bounds.
  /// NOTE: OSQP cannot handle ±Inf in bounds — replace with large finite values.
  OSQPError UpdateBounds(const Eigen::VectorXd &l, const Eigen::VectorXd &u) {
    if (!work_) return OSQPError::kWorkspaceNotInitialized;
    c_int m = static_cast<c_int>(l.size());
    std::vector<c_float> lv(m), uv(m);
    Eigen::Map<Eigen::VectorXd>(lv.data(), m) = l;
    Eigen::Map<Eigen::VectorXd>(uv.data(), m) = u;
    for (c_int i = 0; i < m; ++i) {
      if (!std::isfinite(lv[i])) lv[i] = lv[i] < 0 ? -OSQP_INFTY : OSQP_INFTY;
      if (!std::isfinite(uv[i])) uv[i] = uv[i] < 0 ? -OSQP_INFTY : OSQP_INFTY;
    }
    c_int exitflag = osqp_update_bounds(work_, lv.data(), uv.data());
    if (exitflag == 0) return OSQPError::kNoError;
    return OSQPError::kUpdateFailed;
  }

  /// Update linear cost (gradient).
  OSQPError UpdateGradient(const Eigen::VectorXd &q) {
    if (!work_) return OSQPError::kWorkspaceNotInitialized;
    c_int n = static_cast<c_int>(q.size());
    std::vector<c_float> qv(n);
    Eigen::Map<Eigen::VectorXd>(qv.data(), n) = q;
    c_int exitflag = osqp_update_lin_cost(work_, qv.data());
    if (exitflag == 0) return OSQPError::kNoError;
    return OSQPError::kUpdateFailed;
  }

  /// Update all values of P (sparsity must be identical).
  /// @param P_new  Sparse matrix with IDENTICAL outer/inner index structure
  ///               as the original P passed to Init().  Only the value array
  ///               may differ.  Caller must ensure identical sparsity.
  OSQPError UpdateP(const Eigen::SparseMatrix<double> &P_new) {
    if (!work_) return OSQPError::kWorkspaceNotInitialized;
    c_int n = static_cast<c_int>(P_new.nonZeros());
    std::vector<c_float> Px(n);
    for (c_int i = 0; i < n; ++i) {
      Px[i] = static_cast<c_float>(P_new.valuePtr()[i]);
    }
    // OSQP_NULL, 0 means "update all values in storage order"
    c_int exitflag = osqp_update_P(work_, Px.data(), OSQP_NULL, 0);
    if (exitflag == 0) return OSQPError::kNoError;
    return OSQPError::kUpdateFailed;
  }

  /// Return the outer (column pointer) index array of the problem matrix P.
  /// Used to verify sparsity structure before UpdateP.
  const c_int * GetPOuterIndices() const {
    if (!work_ || !work_->data || !work_->data->P) return nullptr;
    return work_->data->P->p;
  }

  /// Return the inner (row) index array of the problem matrix P.
  const c_int * GetPInnerIndices() const {
    if (!work_ || !work_->data || !work_->data->P) return nullptr;
    return work_->data->P->i;
  }

  /// Return the number of non-zeros in the problem matrix P.
  c_int GetPNnz() const {
    if (!work_ || !work_->data || !work_->data->P) return 0;
    return work_->data->P->nzmax;
  }

  /// Set primal / dual warm-start values.
  OSQPError SetPrimalDualWarmStart(const Eigen::VectorXd &primal,
                                    const Eigen::VectorXd &dual) {
    if (!work_) return OSQPError::kWorkspaceNotInitialized;
    c_int n = static_cast<c_int>(primal.size());
    c_int m = static_cast<c_int>(dual.size());
    std::vector<c_float> px(n), dy(m);
    Eigen::Map<Eigen::VectorXd>(px.data(), n) = primal;
    Eigen::Map<Eigen::VectorXd>(dy.data(), m) = dual;
    c_int exitflag = osqp_warm_start(work_, px.data(), dy.data());
    if (exitflag == 0) return OSQPError::kNoError;
    return OSQPError::kWarmStartFailed;
  }

  /// Check if the workspace is initialized and ready for updates/solve.
  bool isInitialized() const { return work_ != nullptr && owns_; }

private:
  ::OSQPWorkspace *work_ = nullptr;
  bool owns_ = false;

  // We keep data_ alive for the lifetime of the solver so that
  // osqp_setup / osqp_cleanup have valid input.
  ::OSQPData data_{};

  void cleanup() {
    if (work_ && owns_) {
      osqp_cleanup(work_);
      work_ = nullptr;
      owns_ = false;
    }
    // Free the CSC matrices and arrays we allocated in Init()
    free_csc(data_.P);
    free_csc(data_.A);
    std::free(data_.q); data_.q = nullptr;
    std::free(data_.l); data_.l = nullptr;
    std::free(data_.u); data_.u = nullptr;
    data_.n = 0;
    data_.m = 0;
  }

  static void free_csc(csc *M) {
    if (M) {
      std::free(M->p);
      std::free(M->i);
      std::free(M->x);
      std::free(M);
    }
  }
};

}  // namespace osqp

#endif  // OSQP_CXX_WRAPPER_H
