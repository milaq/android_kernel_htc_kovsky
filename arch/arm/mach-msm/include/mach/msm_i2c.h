/* arch/arm/mach-msm/include/mach/msm_i2c.h
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

#ifndef __ASM_ARCH_MSM_I2C_H
#define __ASM_ARCH_MSM_I2C_H

struct msm_i2c_platform_data {
	int clk_freq;
	int pri_clk;
	int pri_dat;
	int aux_clk;
	int aux_dat;
	void (*msm_i2c_config_gpio)(int iface, int config_type);
};

#endif
