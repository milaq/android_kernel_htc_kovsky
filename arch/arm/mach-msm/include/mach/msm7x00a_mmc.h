/* arch/arm/mach-msm/board-msm7x00A-mmc.h
 *
 * Copyright (c) 2010 htc-linux.org
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

#ifndef __MSM7X00A_MMC_H__
#define __MSM7X00A_MMC_H__

#include <linux/device.h>

struct msm7x00a_mmc_platform_data {
	int (*set_voltage)(unsigned int vdd);
	int (*get_status)(void);
	int sdcc_id;
	int sd_irq;
	int (*init)(struct device *dev);
	void (*exit)(struct device *dev);	
};

#endif /* __MSM7X00A_MMC_H__ */
