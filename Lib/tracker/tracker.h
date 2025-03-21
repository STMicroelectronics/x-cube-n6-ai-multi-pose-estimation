 /**
 ******************************************************************************
 * @file    tracker.h
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

#ifndef _TRACKER_
#define _TRACKER_ 1

#include <stdint.h>

#include "kf.h"
#include "ulist.h"

typedef struct {
  double track_thresh;
  double det_thresh;
  double sim1_thresh;
  double sim2_thresh;
  int tlost_cnt;
} trk_conf_t;

typedef struct {
  /* public read only field */
  int is_tracking;
  uint32_t id;
  uint32_t tlost_cnt;
  double cx;
  double cy;
  double w;
  double h;
  void *userdata;
  void *dbox_userdata;
  /* private data */
  struct ulist list;
  struct kf_state kf_state;
} trk_tbox_t;

typedef struct {
  /* public field */
  double cx;
  double cy;
  double w;
  double h;
  double conf;
  void *userdata;
  /* private data */
  struct ulist list;
} trk_dbox_t;

typedef struct {
  trk_conf_t cfg;
  uint32_t next_id;
  struct ulist tfree;
  struct ulist ttracking;
  struct ulist tlost;
  struct ulist tremain;
  struct ulist dhigh;
  struct ulist dlow;
} trk_ctx_t;

int trk_init(trk_ctx_t *ctx, trk_conf_t *cfg, int trk_tbox_nb, trk_tbox_t *tboxes);
int trk_update(trk_ctx_t *ctx, int trk_dbox_nb, trk_dbox_t *dboxes);

#endif
