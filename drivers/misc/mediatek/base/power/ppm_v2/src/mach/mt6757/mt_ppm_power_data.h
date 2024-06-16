/*
 * Copyright (C) 2016 MediaTek Inc.
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


static int cpu_perf_idx_ref_tbl_FY[NR_PPM_CLUSTERS][DVFS_OPP_NUM] = {
	{501, 455, 410, 345, 299, 247, 169, 78},
	{904, 826, 748, 598, 514, 436, 332, 143},
};

static int cpu_perf_idx_ref_tbl_SB[NR_PPM_CLUSTERS][DVFS_OPP_NUM] = {
	{572, 507, 436, 345, 299, 247, 169, 78},
	{975, 878, 787, 598, 514, 436, 332, 143},
};

static struct ppm_pwr_idx_ref_tbl cpu_pwr_idx_ref_tbl_FY[] = {
	[0] = {
		{145, 123, 103, 74, 61, 47, 31, 16},
		{145, 123, 103, 74, 61, 47, 31, 16},
		{145, 123, 103, 74, 61, 47, 31, 16},
		{145, 123, 103, 74, 61, 47, 31, 16},
		{145, 123, 103, 74, 61, 47, 31, 16},
		{145, 123, 103, 74, 61, 47, 31, 16},
		{33, 28, 23, 17, 14, 11, 7, 4},
		{33, 28, 23, 17, 14, 11, 7, 4},
		{33, 28, 23, 17, 14, 11, 7, 4},
	},
	[1] = {
		{409, 348, 294, 202, 164, 132, 95, 46},
		{409, 348, 294, 202, 164, 132, 95, 46},
		{409, 348, 294, 202, 164, 132, 95, 46},
		{409, 348, 294, 202, 164, 132, 95, 46},
		{409, 348, 294, 202, 164, 132, 95, 46},
		{409, 348, 294, 202, 164, 132, 95, 46},
		{124, 106, 89, 62, 52, 43, 33, 21},
		{124, 106, 89, 62, 52, 43, 33, 21},
		{124, 106, 89, 62, 52, 43, 33, 21},
	},
};

static struct ppm_pwr_idx_ref_tbl cpu_pwr_idx_ref_tbl_SB[] = {
	[0] = {
		{186, 155, 125, 87, 71, 56, 37, 16},
		{186, 155, 125, 87, 71, 56, 37, 16},
		{186, 155, 125, 87, 71, 56, 37, 16},
		{186, 155, 125, 87, 71, 56, 37, 16},
		{186, 155, 125, 87, 71, 56, 37, 16},
		{186, 155, 125, 87, 71, 56, 37, 16},
		{42, 35, 29, 20, 16, 13, 9, 4},
		{42, 35, 29, 20, 16, 13, 9, 4},
		{42, 35, 29, 20, 16, 13, 9, 4},
	},
	[1] = {
		{499, 422, 354, 237, 194, 157, 114, 46},
		{499, 422, 354, 237, 194, 157, 114, 46},
		{499, 422, 354, 237, 194, 157, 114, 46},
		{499, 422, 354, 237, 194, 157, 114, 46},
		{499, 422, 354, 237, 194, 157, 114, 46},
		{499, 422, 354, 237, 194, 157, 114, 46},
		{155, 132, 112, 77, 65, 54, 42, 21},
		{155, 132, 112, 77, 65, 54, 42, 21},
		{155, 132, 112, 77, 65, 54, 42, 21},
	},
};

