/*
 * arch/arm/mach-msm/include/mach/smd.h
 *
 * This file was created as a part of XDANDROID project
 * Copyright 2011 Alexander Tarasikov <alexander_tarasikov@gmail.com>
 * Copyright 2008-2010 Markinus
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MACH_SMD_H__
#define __MACH_SMD_H__

#include <linux/platform_device.h>

enum amss_id {
	AMSS_AUDMGR_PROG_VERS,
	AMSS_AUDMGR_VERS,
	AMSS_RPC_SND_VERS,
	AMSS_SND_SET_DEVICE_PROC,
	AMSS_SND_SET_VOLUME_PROC,
	AMSS_AUDMGR_CB_PROG_VERS,
	AMSS_AUDMGR_CB_VERS,
	AMSS_TIME_REMOTE_MTOA_VERS,
	AMSS_RPC_TIME_REMOTE_MTOA_NULL,
	AMSS_RPC_TIME_TOD_SET_APPS_BASES,
	AMSS_PM_LIBVERS,
	AMSS_RPC_ADSP_RTOS_ATOM_VERS,
	AMSS_RPC_ADSP_RTOS_MTOA_VERS,
	AMSS_RPC_DOG_KEEPALIVE_NULL,  
	AMSS_RPC_DOG_KEEPALIVE_BEACON,
	AMSS_DOG_KEEPALIVE_VERS,
};

enum amss_value_type {
	AMSS_VAL_UINT,
	AMSS_VAL_STRING,
};

struct amss_value {
	enum amss_id id;
	enum amss_value_type type;
	union {
		uint32_t value;
		const char* string;
	};
};

struct smd_overrides {
		struct amss_value *amss_values;
		int n_amss_values;
};

extern bool amss_get_num_value(enum amss_id, uint32_t* out);
extern const char* amss_get_str_value(enum amss_id);
extern void amss_set_overrides(struct smd_overrides*);

#endif
