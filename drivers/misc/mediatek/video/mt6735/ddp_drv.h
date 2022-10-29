/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */


#ifndef __DDP_DRV_H__
#define __DDP_DRV_H__
#include <linux/ioctl.h>
#include "ddp_hal.h"
#include "ddp_aal.h"
#include "ddp_gamma.h"
#include "disp_event.h"
#include "DpDataType.h"
#ifndef CONFIG_MTK_CLKMGR
#include <linux/clk.h>
#endif
#include "ddp_pq.h"

typedef struct {
	unsigned int reg;
	unsigned int val;
	unsigned int mask;
} DISP_WRITE_REG;

typedef struct {
	unsigned int reg;
	unsigned int val;
	unsigned int mask;
} DISP_READ_REG;

#if 0
typedef struct {
	DISP_MODULE_ENUM module;
	unsigned int timeout_ms;	/* timeout, unit is ms */
} disp_wait_irq_struct;
#endif

typedef struct DISP_EXEC_COMMAND {
	int taskID;
	uint32_t scenario;
	uint32_t priority;
	uint32_t engineFlag;
	uint32_t *pFrameBaseSW;
	uint32_t *pTileBaseSW;
	uint32_t blockSize;
} DISP_EXEC_COMMAND;

typedef struct {
	int layer;

	unsigned long addr;
	DpColorFormat fmt;

	int x;
	int y;
	int w;
	int h;			/* clip region */
	int pitch;
} DISP_OVL_INFO;

typedef enum {
	DISP_INTERLACE_FORMAT_NONE,
	DISP_INTERLACE_FORMAT_TOP_FIELD,
	DISP_INTERLACE_FORMAT_BOTTOM_FIELD
} DISP_INTERLACE_FORMAT;

struct device *disp_get_device(void);

#define DISP_IOCTL_MAGIC        'x'

#define DISP_IOCTL_WRITE_REG       _IOW(DISP_IOCTL_MAGIC, 1, DISP_WRITE_REG)	/* also defined in atci_pq_cmd.h */
#define DISP_IOCTL_READ_REG        _IOWR(DISP_IOCTL_MAGIC, 2, DISP_READ_REG)	/* also defined in atci_pq_cmd.h */
/* #define DISP_IOCTL_WAIT_IRQ        _IOR     (DISP_IOCTL_MAGIC, 3, disp_wait_irq_struct) */
#define DISP_IOCTL_DUMP_REG        _IOR(DISP_IOCTL_MAGIC, 4, int)
#define DISP_IOCTL_LOCK_THREAD     _IOR(DISP_IOCTL_MAGIC, 5, int)
#define DISP_IOCTL_UNLOCK_THREAD   _IOR(DISP_IOCTL_MAGIC, 6, int)
#define DISP_IOCTL_MARK_CMQ        _IOR(DISP_IOCTL_MAGIC, 7, int)
#define DISP_IOCTL_WAIT_CMQ        _IOR(DISP_IOCTL_MAGIC, 8, int)
#define DISP_IOCTL_SYNC_REG        _IOR(DISP_IOCTL_MAGIC, 9, int)

#define DISP_IOCTL_LOCK_MUTEX      _IOW(DISP_IOCTL_MAGIC, 20, int)
#define DISP_IOCTL_UNLOCK_MUTEX    _IOR(DISP_IOCTL_MAGIC, 21, int)

#define DISP_IOCTL_LOCK_RESOURCE   _IOW(DISP_IOCTL_MAGIC, 25, int)
#define DISP_IOCTL_UNLOCK_RESOURCE _IOR(DISP_IOCTL_MAGIC, 26, int)

#define DISP_IOCTL_SET_INTR        _IOR(DISP_IOCTL_MAGIC, 10, int)
#define DISP_IOCTL_TEST_PATH       _IOR(DISP_IOCTL_MAGIC, 11, int)

#define DISP_IOCTL_CLOCK_ON        _IOR(DISP_IOCTL_MAGIC, 12, int)
#define DISP_IOCTL_CLOCK_OFF       _IOR(DISP_IOCTL_MAGIC, 13, int)

#define DISP_IOCTL_RUN_DPF         _IOW(DISP_IOCTL_MAGIC, 30, int)
#define DISP_IOCTL_CHECK_OVL       _IOR(DISP_IOCTL_MAGIC, 31, int)
#define DISP_IOCTL_GET_OVL         _IOWR(DISP_IOCTL_MAGIC, 32, DISP_OVL_INFO)

#define DISP_IOCTL_EXEC_COMMAND    _IOW(DISP_IOCTL_MAGIC, 33, DISP_EXEC_COMMAND)
#define DISP_IOCTL_RESOURCE_REQUIRE   _IOR(DISP_IOCTL_MAGIC, 34, int)

/* Add for AAL control - S */
/* 0 : disable AAL event, 1 : enable AAL event */
#define DISP_IOCTL_AAL_EVENTCTL    _IOW(DISP_IOCTL_MAGIC, 15 , int)
/* Get AAL statistics data. */
#define DISP_IOCTL_AAL_GET_HIST    _IOR(DISP_IOCTL_MAGIC, 16 , DISP_AAL_HIST)
/* Update AAL setting */
#define DISP_IOCTL_AAL_SET_PARAM   _IOW(DISP_IOCTL_MAGIC, 17 , DISP_AAL_PARAM)
#define DISP_IOCTL_AAL_INIT_REG    _IOW(DISP_IOCTL_MAGIC, 18 , DISP_AAL_INITREG)
#define DISP_IOCTL_SET_GAMMALUT    _IOW(DISP_IOCTL_MAGIC, 23 , DISP_GAMMA_LUT_T)
#define DISP_IOCTL_SET_CCORR       _IOW(DISP_IOCTL_MAGIC, 24 , DISP_CCORR_COEF_T)

/* Add for PQ transition control */
/* 0 : disable CCORR event, 1 : enable CCORR event */
#define DISP_IOCTL_CCORR_EVENTCTL    _IOW(DISP_IOCTL_MAGIC, 110, int)
/* Get CCORR interrupt */
#define DISP_IOCTL_CCORR_GET_IRQ    _IOR(DISP_IOCTL_MAGIC, 111, int)
/* Get color transform support */
#define DISP_IOCTL_SUPPORT_COLOR_TRANSFORM \
	_IOW(DISP_IOCTL_MAGIC, 112, struct DISP_COLOR_TRANSFORM)

/* Add for AAL control - E */
/*-----------------------------------------------------------------------------
    DDP Kernel Mode API  (for Kernel Trap)
  -----------------------------------------------------------------------------*/
/* DDPK Bitblit */
/* #define DISP_IOCTL_G_WAIT_REQUEST  _IOR     (DISP_IOCTL_MAGIC , 40 , DDPIOCTL_DdpkBitbltConfig) */
/* #define DISP_IOCTL_T_INFORM_DONE   _IOW     (DISP_IOCTL_MAGIC , 41 , DDPIOCTL_DdpkBitbltInformDone) */

#define DISP_IOCTL_SET_CLKON        _IOW(DISP_IOCTL_MAGIC, 50 , DISP_MODULE_ENUM)
#define DISP_IOCTL_SET_CLKOFF       _IOW(DISP_IOCTL_MAGIC, 51 , DISP_MODULE_ENUM)

#define DISP_IOCTL_MUTEX_CONTROL    _IOW(DISP_IOCTL_MAGIC, 55 , int)	/* also defined in atci_pq_cmd.h */
#define DISP_IOCTL_GET_LCMINDEX     _IOR(DISP_IOCTL_MAGIC, 56 , int)

/* PQ setting */
#define DISP_IOCTL_SET_PQPARAM      _IOW(DISP_IOCTL_MAGIC, 60 , DISP_PQ_PARAM)
#define DISP_IOCTL_GET_PQPARAM      _IOR(DISP_IOCTL_MAGIC, 61 , DISP_PQ_PARAM)
#define DISP_IOCTL_GET_PQINDEX      _IOR(DISP_IOCTL_MAGIC, 63,  DISPLAY_PQ_T)
#define DISP_IOCTL_SET_PQINDEX      _IOW(DISP_IOCTL_MAGIC, 64 , DISPLAY_PQ_T)
#define DISP_IOCTL_SET_TDSHPINDEX   _IOW(DISP_IOCTL_MAGIC, 65 , DISPLAY_TDSHP_T)
#define DISP_IOCTL_GET_TDSHPINDEX   _IOR(DISP_IOCTL_MAGIC, 66 , DISPLAY_TDSHP_T)
#define DISP_IOCTL_SET_PQ_CAM_PARAM _IOW(DISP_IOCTL_MAGIC, 67 , DISP_PQ_PARAM)
#define DISP_IOCTL_GET_PQ_CAM_PARAM _IOR(DISP_IOCTL_MAGIC, 68 , DISP_PQ_PARAM)
#define DISP_IOCTL_SET_PQ_GAL_PARAM _IOW(DISP_IOCTL_MAGIC, 69 , DISP_PQ_PARAM)
#define DISP_IOCTL_GET_PQ_GAL_PARAM _IOR(DISP_IOCTL_MAGIC, 70 , DISP_PQ_PARAM)

#define DISP_IOCTL_PQ_SET_BYPASS_COLOR  _IOW(DISP_IOCTL_MAGIC, 71 , int)
#define DISP_IOCTL_PQ_SET_WINDOW    _IOW(DISP_IOCTL_MAGIC, 72 , DISP_PQ_WIN_PARAM)
#define DISP_IOCTL_PQ_GET_TDSHP_FLAG    _IOR(DISP_IOCTL_MAGIC, 73 , int)
#define DISP_IOCTL_PQ_SET_TDSHP_FLAG    _IOW(DISP_IOCTL_MAGIC, 74 , int)
#define DISP_IOCTL_PQ_GET_DC_PARAM  _IOR(DISP_IOCTL_MAGIC, 75, DISP_PQ_DC_PARAM)
#define DISP_IOCTL_PQ_SET_DC_PARAM  _IOW(DISP_IOCTL_MAGIC, 76, DISP_PQ_DC_PARAM)
#define DISP_IOCTL_WRITE_SW_REG     _IOW(DISP_IOCTL_MAGIC, 77, DISP_WRITE_REG)	/* also defined in atci_pq_cmd.h */
#define DISP_IOCTL_READ_SW_REG      _IOWR(DISP_IOCTL_MAGIC, 78, DISP_READ_REG)	/* also defined in atci_pq_cmd.h */
#define DISP_IOCTL_SET_COLOR_REG    _IOWR(DISP_IOCTL_MAGIC, 79, DISPLAY_COLOR_REG_T)

/* OD */
#define DISP_IOCTL_OD_CTL           _IOWR(DISP_IOCTL_MAGIC, 80 , DISP_OD_CMD)

/* OVL */
#define DISP_IOCTL_OVL_ENABLE_CASCADE  _IOW(DISP_IOCTL_MAGIC, 90 , int)
#define DISP_IOCTL_OVL_DISABLE_CASCADE  _IOW(DISP_IOCTL_MAGIC, 91 , int)

/*PQ setting*/
#define DISP_IOCTL_PQ_GET_DS_PARAM      _IOR(DISP_IOCTL_MAGIC, 100, DISP_PQ_DS_PARAM)
#define DISP_IOCTL_PQ_GET_MDP_COLOR_CAP _IOR(DISP_IOCTL_MAGIC, 101, MDP_COLOR_CAP)
#define DISP_IOCTL_PQ_GET_MDP_TDSHP_REG _IOR(DISP_IOCTL_MAGIC, 102, MDP_TDSHP_REG)

/* secure video path implementation: the handle value */
#define DISP_IOCTL_SET_TPLAY_HANDLE    _IOW(DISP_IOCTL_MAGIC, 200, unsigned int)

#ifndef CONFIG_MTK_CLKMGR
typedef enum disp_clk_id {
	DISP0_SMI_COMMON = 0,
	DISP0_SMI_LARB0,
	DISP0_DISP_OVL0,
	DISP0_DISP_RDMA0,
	DISP0_DISP_RDMA1,
	DISP0_DISP_WDMA0,
	DISP0_DISP_COLOR,
	DISP0_DISP_CCORR,
	DISP0_DISP_AAL,
	DISP0_DISP_GAMMA,
	DISP0_DISP_DITHER,
	DISP1_DSI_ENGINE,
	DISP1_DSI_DIGITAL,
	DISP1_DPI_ENGINE,
	DISP1_DPI_PIXEL,
	DISP_PWM,
	MUX_DPI0,
	TVDPLL,
	TVDPLL_CK,
	TVDPLL_D2,
	TVDPLL_D4,
	DPI_CK,
	MUX_PWM,
	UNIVPLL2_D4,
	SYSPLL4_D2_D8,
	SYS_26M_CK,
	DISP_MTCMOS_CLK,
	MAX_DISP_CLK_CNT
} eDDP_CLK_ID;

int ddp_clk_prepare(eDDP_CLK_ID id);
int ddp_clk_unprepare(eDDP_CLK_ID id);
int ddp_clk_enable(eDDP_CLK_ID id);
int ddp_clk_disable(eDDP_CLK_ID id);
int ddp_clk_prepare_enable(eDDP_CLK_ID id);
int ddp_clk_disable_unprepare(eDDP_CLK_ID id);
int ddp_clk_set_parent(eDDP_CLK_ID id, eDDP_CLK_ID parent);
int ddp_clk_set_rate(eDDP_CLK_ID id, unsigned long rate);
#endif

extern unsigned int dispsys_irq[DISP_REG_NUM];
extern unsigned long dispsys_reg[DISP_REG_NUM];

extern void disp_m4u_tf_disable(void);

/* TODO: FIXME */
#include <linux/types.h>

#include "disp_drv_platform.h"
#ifndef DISP_NO_DPI
#include "ddp_dpi_reg.h"
extern struct DPI_REGS *DPI_REG;
extern unsigned long DPI_TVDPLL_CON0;
extern unsigned long DPI_TVDPLL_CON1;
#endif

extern int m4u_enable_tf(int port, bool fgenable);

#endif