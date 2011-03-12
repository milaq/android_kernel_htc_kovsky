/*
 * Copyright (C) 2011 htc-linux.org
 * Copyright (C) 2008 HTC, Inc.
 * Copyright (C) 2008 Google, Inc.
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

#ifndef __ASM_ARCH_HTC_HEADSET_35MM_H__
#define __ASM_ARCH_HTC_HEADSET_35MM_H__

struct htc_headset_35mm_pdata {
	int gpio_detect;
	int gpio_headset_mic;
	bool jack_inverted;
};

#define BIT_HEADSET		(1 << 0)
#define BIT_HEADSET_NO_MIC	(1 << 1)
#define BIT_TTY			(1 << 2)
#define BIT_FM_HEADSET 		(1 << 3)
#define BIT_FM_SPEAKER		(1 << 4)
#define BIT_TTY_VCO             (1 << 5)
#define BIT_TTY_HCO             (1 << 6)
#define BIT_35MM_HEADSET        (1 << 7)

enum {
	HDS_NO_DEVICE	= 0,
	HDS_HTC_HEADSET	= 1,
	NORMAL_HEARPHONE= 2,
	HDS_DEVICE	= 3,
	HDS_USB_CRADLE	= 4,
	HDS_UART_DEBUG	= 5,
};


#endif
