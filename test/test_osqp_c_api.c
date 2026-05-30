// Test OSQP C API with a dense constraint matrix like our MPC formulation
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "osqp.h"

#define CI_FMT "%lld"

// Build P like our MPC: 40x40 upper triangular with realistic values
static csc* make_P(c_int n) {
  c_int nnz = n * (n + 1) / 2;
  csc *M = (csc*)malloc(sizeof(csc));
  M->nzmax = nnz; M->m = n; M->n = n; M->nz = -1;
  M->p = (c_int*)malloc((n + 1) * sizeof(c_int));
  M->i = (c_int*)malloc(nnz * sizeof(c_int));
  M->x = (c_float*)malloc(nnz * sizeof(c_float));
  c_int idx = 0;
  for (c_int col = 0; col < n; col++) {
    M->p[col] = idx;
    for (c_int row = 0; row <= col; row++) {
      M->i[idx] = row;
      // MPC-like P: diag ~2-4, off-diag ~0.5-1
      M->x[idx] = (row == col) ? (2.0 + (double)row / n) : (0.5 / (1.0 + (double)(col-row)));
      idx++;
    }
  }
  M->p[n] = idx;
  return M;
}

// Build A like our MPC: 240x40 dense matrix (4 blocks of 160+40+40)
// [S_x; -S_x; I; S_du] where each block is mostly non-zero
static csc* make_A_dense(c_int m, c_int n) {
  // We'll build a dense 240x40 matrix (all ~6600 entries)
  c_int nnz = m * n;  // fully dense
  csc *M = (csc*)malloc(sizeof(csc));
  M->nzmax = nnz; M->m = m; M->n = n; M->nz = -1;
  M->p = (c_int*)malloc((n + 1) * sizeof(c_int));
  M->i = (c_int*)malloc(nnz * sizeof(c_int));
  M->x = (c_float*)malloc(nnz * sizeof(c_float));

  c_int idx = 0;
  for (c_int col = 0; col < n; col++) {
    M->p[col] = idx;
    for (c_int row = 0; row < m; row++) {
      // Simple pattern: vary values by row and column
      double val = 0.0;
      if (row < 80) {  // S_x block
        val = sin((double)(row * n + col) / (m * n) * 6.28) * 0.5 + 0.5;
      } else if (row < 160) {  // -S_x block
        val = -(sin((double)((row-80) * n + col) / (80 * n) * 6.28) * 0.5 + 0.5);
      } else if (row < 200) {  // I block
        val = (row - 160 == col) ? 1.0 : 0.0;
      } else {  // S_du-like block
        int r = row - 200;
        if (r == col) val = 1.0;
        else if (r - 2 == col) val = -1.0;
        else val = 0.001;  // near-zero but not zero
      }
      if (fabs(val) > 1e-10) {
        M->i[idx] = row;
        M->x[idx] = val;
        idx++;
      }
    }
  }
  M->nzmax = idx;
  M->p[n] = idx;
  // Realloc to shrink
  M->i = (c_int*)realloc(M->i, idx * sizeof(c_int));
  M->x = (c_float*)realloc(M->x, idx * sizeof(c_float));
  return M;
}

int main(int argc, char **argv) {
  c_int n = 40, m = 240;
  int use_dense = (argc > 1) ? atoi(argv[1]) : 0;

  c_float *q = (c_float*)calloc(n, sizeof(c_float));
  c_float *l = (c_float*)malloc(m * sizeof(c_float));
  c_float *u = (c_float*)malloc(m * sizeof(c_float));
  const c_float OSQP_INFTY_VAL = 1e20;

  // Simple bounds
  for (c_int i = 0; i < m; i++) {
    if (i < n) { l[i] = -3.0; u[i] = 3.0; }
    else { l[i] = -OSQP_INFTY_VAL; u[i] = OSQP_INFTY_VAL; }
  }

  csc *P = make_P(n);
  csc *A = use_dense ? make_A_dense(m, n) : make_A_dense(m, n);  // always dense

  printf("A: %lld x %lld, nnz=%lld\n", (c_int)A->m, (c_int)A->n, (c_int)A->nzmax);

  OSQPSettings *settings = (OSQPSettings*)malloc(sizeof(OSQPSettings));
  OSQPData *data = (OSQPData*)malloc(sizeof(OSQPData));
  osqp_set_default_settings(settings);
  settings->verbose = 0;
  settings->eps_abs = 1e-4;
  settings->eps_rel = 1e-4;
  settings->max_iter = 4000;
  settings->warm_start = 1;
  settings->polish = 0;

  data->n = n; data->m = m;
  data->P = P; data->A = A;
  data->q = q; data->l = l; data->u = u;

  OSQPWorkspace *work = NULL;
  c_int exitflag = osqp_setup(&work, data, settings);
  printf("osqp_setup exitflag: " CI_FMT "\n", exitflag);
  if (exitflag != 0) { printf("Setup failed!\n"); return 1; }

  // Solve
  exitflag = osqp_solve(work);
  printf("Status: %s (val=" CI_FMT ")\n", work->info->status, work->info->status_val);
  printf("Iter: " CI_FMT ", pri_res: %e, dua_res: %e\n",
         (c_int)work->info->iter, work->info->pri_res, work->info->dua_res);

  // Update gradient and bounds, re-solve (simulating MPC cycle)
  for (int cycle = 0; cycle < 3; cycle++) {
    // Slightly different random gradient
    for (c_int i = 0; i < n; i++)
      q[i] = ((double)rand() / RAND_MAX - 0.5) * 10.0;
    osqp_update_lin_cost(work, q);

    // Change bounds slightly
    for (c_int i = 0; i < m; i++) {
      if (i < n) { l[i] = -3.0 - 0.1 * cycle; u[i] = 3.0 + 0.1 * cycle; }
    }
    osqp_update_bounds(work, l, u);

    exitflag = osqp_solve(work);
    printf("Cycle %d: Status=%s (val=" CI_FMT "), iter=" CI_FMT ", pri_res=%e, dua_res=%e\n",
           cycle, work->info->status, work->info->status_val,
           (c_int)work->info->iter, work->info->pri_res, work->info->dua_res);
  }

  osqp_cleanup(work);
  free(settings); free(data);
  printf("\n=== DONE ===\n");
  return 0;
}
