 /**
 ******************************************************************************
 * @file    pkf.h
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

#ifndef __PKF__
#define __PKF__ 1

/* Generic kalman filter point tracking with constant velocity */

struct pkf_context {
  /* state matrix for constant velocity model */
  double F[4][4];
  /* measurement matrix */
  double H[2][4];
  /* process noise matrix */
  double Q[4][4];
  /* measure noise matrix */
  double R[2][2];
  /* transpose of F */
  double Ft[4][4];
  /* transpose of H */
  double Ht[4][2];
};

struct pkf_state {
  /* state vector is [x, vx, y, vy]*/
  double X[4];
  /* covariance matrix */
  double P[4][4];
  /* link to context */
  struct pkf_context *ctx;
};

struct pkf_point {
  double x;
  double y;
};

void pkf_init_context(struct pkf_context *ctx, double dt, double speed_std, double measure_noise_std);
void pkf_init(struct pkf_state *st, struct pkf_context *ctx, struct pkf_point *init);
void pkf_predict(struct pkf_state *st, struct pkf_point *predicted);
void pkf_update(struct pkf_state *st, struct pkf_point *measure, struct pkf_point *updated);

#endif
