/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#ifndef __MTK_HRT_H__
#define __MTK_HRT_H__

#include "disp_session.h"
#include "disp_lcm.h"
#include "disp_drv_log.h"
#include "primary_display.h"
#include "disp_drv_platform.h"
#include "display_recorder.h"

#define EMI_EXTREME_LOWER_BOUND 2
#define EMI_LOWER_BOUND 4
#define EMI_UPPER_BOUND 6
#define LARB_LOWER_BOUND 3
#define LARB_UPPER_BOUND 4
#define OD_EMI_LOWER_BOUND 3
#define OD_EMI_UPPER_BOUND 4
#define OD_LARB_LOWER_BOUND 2
#define OD_LARB_UPPER_BOUND 3

/**
 * OVL HW capabilities
 * May be different with different platform
 */
#define DISP_HW_OVL_EXT_LAYER_NUM			(3)
#define PRIMARY_HW_OVL_LAYER_NUM			(4)
#define PRIMARY_HW_OVL_2L_LAYER_NUM			(2)
#define EXTERNAL_HW_OVL_LAYER_NUM			(2)
#define EXTERNAL_HW_OVL_2L_LAYER_NUM		(2)

/**
 *  All bandwidth limitations are simulated with 60fps, buffer format ARGB(bpp=4)
 *  with full-screen size layer
 *  So, total bandwidth limiter = max layer number * (4*60)
 */
#define BYTES_PER_SECOND_FULLSCREEN (4*60)

#ifdef CONFIG_MTK_DISPLAY_120HZ_SUPPORT
#define HRT_LEVEL(id) ((id)&0xff)
#define HRT_FPS(id) (((id)>>8)&0xff)
#define MAKE_HRT_NUM(fps, level) (unsigned int)((fps)<<8 | (level))
#endif

/* #define HRT_DEBUG */

int dispsys_hrt_calc(disp_layer_info *disp_info);

typedef struct hrt_sort_entry_t {
	struct hrt_sort_entry_t *head, *tail;
	layer_config *layer_info;
	int key;
	int overlap_w;
} hrt_sort_entry;

typedef enum {
	HRT_LEVEL_EXTREME_LOW = 0,
	HRT_LEVEL_LOW,
	HRT_LEVEL_HIGH,
	HRT_OVER_LIMIT,
} HRT_LEVEL;

typedef enum {
	HRT_PRIMARY = 0,
	HRT_SECONDARY,
} HRT_DISP_TYPE;

extern int hdmi_get_dev_info(int is_sf, void *info);

#endif
