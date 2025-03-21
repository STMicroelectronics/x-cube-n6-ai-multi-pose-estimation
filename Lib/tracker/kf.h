 /**
 ******************************************************************************
 * @file    kf.h
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

#ifndef __KF__
#define __KF__ 1

#define KF_DIM 4

struct kf_state {
  double mean[2 * KF_DIM];
  double covariance[2 * KF_DIM][2 * KF_DIM];
};

struct kf_box {
  double cx;
  double cy;
  double a;
  double h;
};

void kf_init(struct kf_state *state, struct kf_box *measure);
void kf_update(struct kf_state *state, struct kf_box *measure);
void kf_predict(struct kf_state *state, struct kf_box *predicted);

#endif
