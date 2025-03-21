 /**
 ******************************************************************************
 * @file    pkf.c
 * @author  GPM Application Team
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "pkf.h"

#include <math.h>
#include <string.h>

/* result can overlap */
/* result = m * v */
static void kf_mat_vector_dot_product(double *result, double *m, double *v, int row_nb, int col_nb)
{
  double res[row_nb];
  int r, c;

  for (r = 0; r < row_nb; r++) {
    res[r] = 0;
    for (c = 0; c < col_nb; c++) {
      res[r] += m[r * col_nb + c] * v[c];
    }
  }

  memcpy(result, res, sizeof(res));
}

/* matrix transposition
 * result = transpose(m)
 * result is a matrix of size row_nb   * col_nb
 * m      is a matrix of size col_nb   * row_nb
 * both matrix must not overlap
*/
static void kf_mat_transpose(double *result, double *m, int row_nb, int col_nb)
{
  int r, c;

  for (r = 0; r < row_nb; r++) {
    for (c = 0; c < col_nb; c++) {
      result[r * col_nb + c] = m[c * row_nb + r];
    }
  }
}

/* matrix dot product
 * result = m1 * m2
 * result is a matrix of size row_nb   * col_nb
 * m1     is a matrix of size row_nb   * inter_nb
 * m2     is a matrix of size inter_nb * col_nb
 * result can overlap with either m1 or m2
*/
static void kf_mat_dot_product(double *result, double *m1, double *m2, int row_nb, int col_nb, int inter_nb)
{
  double res[row_nb][col_nb];
  int r, c, i;

  for (r = 0; r < row_nb; r++) {
    for (c = 0; c < col_nb; c++) {
      res[r][c] = 0;
      for (i = 0; i < inter_nb; i++) {
        res[r][c] += m1[r * inter_nb + i] * m2[i * col_nb + c];
      }
    }
  }

  memcpy(result, res, sizeof(res));
}

/* matric addition
 * result = m1 + m2
 */
static void kf_mat_add(double *result, double *m1, double *m2, int row_nb, int col_nb)
{
  int r, c;

  for (r = 0; r < row_nb; r++) {
    for (c = 0; c < col_nb; c++) {
      result[r * col_nb + c] = m1[r * col_nb + c] + m2[r * col_nb + c];
    }
  }
}

/* matric addition
 * result = m1 - m2
 */
static void kf_mat_sub(double *result, double *m1, double *m2, int row_nb, int col_nb)
{
  int r, c;

  for (r = 0; r < row_nb; r++) {
    for (c = 0; c < col_nb; c++) {
      result[r * col_nb + c] = m1[r * col_nb + c] - m2[r * col_nb + c];
    }
  }
}

static void kf_cho_decomposition(double *lower, double *A, int row_col_nb)
{
  const int n = row_col_nb;
  double sum;
  int r, c;
  int i;

  memset(lower, 0, row_col_nb * row_col_nb * sizeof(double));

  for (r = 0; r < row_col_nb; r++) {
    for (c = 0; c <= r; c++) {
      sum = 0;
      if (r == c) {
        for (i = 0; i < c; i++)
          sum += lower[c * n + i] * lower[c * n + i];
        lower[c * n + c] = sqrt(A[c * n + c] - sum);
      } else {
        for (i = 0; i < c; i++)
          sum += lower[r * n + i] * lower[c * n + i];
        lower[r * n + c] = (A[r * n + c] - sum) / lower[c * n + c];
      }
    }
  }
}

static void kf_triangular_lower_invert(double *m_inv, double *m, int row_col_nb)
{
  const int n = row_col_nb;
  double sum;
  int r, c;
  int k;

  memset(m_inv, 0, row_col_nb * row_col_nb * sizeof(double));

  for (r = 0; r < row_col_nb; r++) {
    for (c = 0; c <= r; c++) {
      if (r == c) {
        m_inv[r * n + c] = 1 / m[r * n + c];
      } else {
        sum = 0;
        for (k = c; k < r; k++)
          sum += m[r * n + k] * m_inv[k * n + c];
        m_inv[r * n + c] = -sum / m[r * n + r];
      }
    }
  }
}

/* solve A * x = B with A being symetric */
static void kf_cho_solve(double *x, double *A, double *B, double *cho, double *cho_inv, double * cho_inv_t, int row_nb,
                         int col_nb)
{
  kf_cho_decomposition(cho, A, row_nb);
  kf_triangular_lower_invert(cho_inv, cho, row_nb);
  kf_mat_transpose(cho_inv_t, cho_inv, row_nb, row_nb);

  /* use cho for intermediate multiplication */
  kf_mat_dot_product(cho, cho_inv_t, cho_inv, row_nb, row_nb, row_nb);
  kf_mat_dot_product(x, cho, B, row_nb, col_nb, row_nb);
}

static void pkf_compute_kalman_gain(struct pkf_state *st, double K[4][2])
{
  struct pkf_context *ctx = st->ctx;
  double cho_inv_t[2][2];
  double cho_inv[2][2];
  double cho[2][2];
  double Kt[2][4];
  double Pt[4][4];
  double HP[2][4]; /* H * P */
  double A[2][2];
  double B[2][4];

  /* compute A = H * P * Ht + R which is a symmetric matrix */
   /* first H * P */
  kf_mat_dot_product((double *)HP, (double *)ctx->H, (double *)st->P, 2, 4, 4);
   /* then H * P * Ht */
  kf_mat_dot_product((double *)A, (double *)HP, (double *)ctx->Ht, 2, 2, 4);
   /* and to finish H * P * Ht + R */
  kf_mat_add((double *)A, (double *)A, (double *)ctx->R, 2, 2);

  /* compute B = (P * Ht)t = H * Pt */
  kf_mat_transpose((double *)Pt, (double *)st->P, 4, 4);
  kf_mat_dot_product((double *)B, (double *)ctx->H, (double *)Pt, 2, 4, 4);

  /* solve Kt = A^-1 * B using cho */
  kf_cho_solve((double *)Kt, (double *)A, (double *)B, (double *)cho, (double *)cho_inv, (double *)cho_inv_t, 2, 4);

  /* compute K from Kt */
  kf_mat_transpose((double *)K, (double *)Kt, 4, 2);
}

/* public API */
void pkf_init_context(struct pkf_context *ctx, double dt, double speed_std, double measure_noise_std)
{
  double a, b, c;
  int i;

  /* init state matrix */
  memset(ctx->F, 0, sizeof(ctx->F));
  for (i = 0; i < 4; i++)
    ctx->F[i][i] = 1;
  ctx->F[0][1] = dt;
  ctx->F[2][3] = dt;

  /* init measurement matrix */
  memset(ctx->H, 0, sizeof(ctx->H));
  ctx->H[0][0] = 1;
  ctx->H[1][2] = 1;

  /* process noise matrix */
  memset(ctx->Q, 0, sizeof(ctx->Q));
  a = (speed_std * speed_std * pow(dt, 4)) / 4;
  b = (speed_std * speed_std * pow(dt, 3)) / 2;
  c = (speed_std * speed_std * pow(dt, 2));
  ctx->Q[0][0] = ctx->Q[2][2] = a;
  ctx->Q[1][1] = ctx->Q[3][3] = c;
  ctx->Q[0][1] = ctx->Q[2][3] = b;
  ctx->Q[1][0] = ctx->Q[3][2] = b;

  /* measure noise matrix */
  memset(ctx->R, 0, sizeof(ctx->R));
  ctx->R[0][0] = ctx->R[1][1] = measure_noise_std * measure_noise_std;

  /* transpose of F */
  kf_mat_transpose((double *)ctx->Ft, (double *)ctx->F, 4, 4);

  /* transpose of H */
  kf_mat_transpose((double *)ctx->Ht, (double *)ctx->H, 4, 2);
}

void pkf_init(struct pkf_state *st, struct pkf_context *ctx, struct pkf_point *init)
{
  int i;

  memset(st->X, 0, sizeof(st->X));
  st->X[0] = init->x;
  st->X[2] = init->y;

  memset(st->P, 0, sizeof(st->P));
  for (i = 0; i < 4; i++)
    st->P[i][i] = 1;

  st->ctx = ctx;
}

void pkf_predict(struct pkf_state *st, struct pkf_point *predicted)
{
  /* compute a priori state : X = F * X */
  kf_mat_vector_dot_product(st->X, (double *)st->ctx->F, st->X, 4, 4);

  /* compute a priori covariance : P = F * P * Ft + Q */
  /* tmp = F * P */
  kf_mat_dot_product((double *)st->P, (double *)st->ctx->F, (double *)st->P, 4, 4, 4);
  /* tmp = F * P *Ft */
  kf_mat_dot_product((double *)st->P, (double *)st->P, (double *)st->ctx->Ft, 4, 4, 4);
  /* P = F * P * Ft + Q */
  kf_mat_add((double *)st->P, (double *)st->P, (double *)st->ctx->Q, 4, 4);

  predicted->x = st->X[0];
  predicted->y = st->X[2];
}

void pkf_update(struct pkf_state *st, struct pkf_point *measure, struct pkf_point *updated)
{
  double residual[2];
  double KHP[4][4];
  double K[4][2];
  double KR[4];
  double HP[2][4];
  int i;

  pkf_compute_kalman_gain(st, K);

  /* compute residual = M - H * X */
  kf_mat_vector_dot_product(residual, (double *)st->ctx->H, st->X, 2, 4);
  residual[0] = measure->x - residual[0];
  residual[1] = measure->y - residual[1];

  /* compute a posteriori state : X = X + K * residual */
  kf_mat_vector_dot_product(KR, (double *)K, residual, 4, 2);
  for (i = 0; i < 4; i++)
    st->X[i] += KR[i];

  /* compute a posteriori covariance : P = P - K * H * P */
  kf_mat_dot_product((double *)HP, (double *)st->ctx->H, (double *)st->P, 2, 4, 4);
  kf_mat_dot_product((double *)KHP, (double *)K, (double *)HP, 4, 4, 2);
  kf_mat_sub((double *)st->P, (double *)st->P, (double *)KHP, 4, 4);

  updated->x = st->X[0];
  updated->y = st->X[2];
}
