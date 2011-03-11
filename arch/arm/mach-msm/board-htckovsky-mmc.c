/* linux/arch/arm/mach-msm/board-htckovsky-mmc.c
*
* Copyright (C) 2011 Alexander Tarasikov <alexander.tarasikov@gmail.com>
* based on previous work which is
* Copyright (C) 2008-2009 Octavian Voicu, Martijn Stolk
* Copyright (C) 2007-2008 Brian Swetland <swetland@google.com>
* Copyright (C) 2007 Google, Inc.
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/spi/wl12xx.h>

#include <asm/gpio.h>
#include <asm/io.h>

#include <mach/board.h>
#include <mach/mmc.h>
#include <mach/vreg.h>

#include "board-htckovsky.h"
#include "devices.h"

#define DEBUG_SDSLOT_VDD 1

static struct msm_gpio sdc1_on_gpio_table[] = {
	{.gpio_cfg = GPIO_CFG(51, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC1_DAT3"},
	{.gpio_cfg = GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC1_DAT2"},
	{.gpio_cfg = GPIO_CFG(53, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC1_DAT1"},
	{.gpio_cfg = GPIO_CFG(54, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),.label = "MMC1_DAT0"},
	{.gpio_cfg = GPIO_CFG(55, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA),.label = "MMC1_CMD"},
	{.gpio_cfg = GPIO_CFG(56, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),.label = "MMC1_CLK"},
};

static struct msm_gpio sdc1_off_gpio_table[] = {
	{.gpio_cfg = GPIO_CFG(51, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC1_DAT3"},
	{.gpio_cfg = GPIO_CFG(52, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC1_DAT2"},
	{.gpio_cfg = GPIO_CFG(53, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC1_DAT1"},
	{.gpio_cfg = GPIO_CFG(54, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC1_DAT0"},
	{.gpio_cfg = GPIO_CFG(55, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC1_CMD"},
	{.gpio_cfg = GPIO_CFG(56, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC1_CLK"},
};

static struct msm_gpio sdc3_on_gpio_table[] = {
	{.gpio_cfg = GPIO_CFG(88, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),.label = "MMC3_CLK"},
	{.gpio_cfg = GPIO_CFG(89, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),.label = "MMC3_CMD"},
	{.gpio_cfg = GPIO_CFG(90, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),.label = "MMC3_DAT3"},
	{.gpio_cfg = GPIO_CFG(91, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),.label = "MMC3_DAT2"},
	{.gpio_cfg = GPIO_CFG(92, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),.label = "MMC3_DAT1"},
	{.gpio_cfg = GPIO_CFG(93, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),.label = "MMC3_DAT0"},
};

static struct msm_gpio sdc3_off_gpio_table[] = {
	{.gpio_cfg = GPIO_CFG(88, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC3_CLK"},
	{.gpio_cfg = GPIO_CFG(89, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC3_CMD"},
	{.gpio_cfg = GPIO_CFG(90, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC3_DAT3"},
	{.gpio_cfg = GPIO_CFG(91, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC3_DAT2"},
	{.gpio_cfg = GPIO_CFG(92, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC3_DAT1"},
	{.gpio_cfg = GPIO_CFG(93, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "MMC3_DAT0"},
};

#define MSM_MMC_VDD	MMC_VDD_165_195 | MMC_VDD_20_21 | MMC_VDD_21_22 \
			| MMC_VDD_22_23 | MMC_VDD_23_24 | MMC_VDD_24_25 \
			| MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 \
			| MMC_VDD_28_29 | MMC_VDD_29_30

static struct mmc_vdd_xlat {
	int mask;
	int level;
} mmc_vdd_table[] = {
	{MMC_VDD_165_195, 1800},
	{MMC_VDD_20_21, 2050},
	{MMC_VDD_21_22, 2150},
	{MMC_VDD_22_23, 2250},
	{MMC_VDD_23_24, 2350},
	{MMC_VDD_24_25, 2450},
	{MMC_VDD_25_26, 2550},
	{MMC_VDD_26_27, 2650},
	{MMC_VDD_27_28, 2750},
	{MMC_VDD_28_29, 2850},
	{MMC_VDD_29_30, 2950},
};

/******************************************************************************
 * SD Slot
 ******************************************************************************/
static struct vreg *vreg_sdslot = NULL;
static uint32_t sdslot_switchvdd(struct device *dev, unsigned int vdd)
{
	int rc, i;
	if (vdd) {
		msm_gpios_disable(sdc3_on_gpio_table,
				 ARRAY_SIZE(sdc3_on_gpio_table));
		rc = vreg_enable(vreg_sdslot);
		if (rc)
			printk(KERN_ERR
			       "%s: Error enabling sd slot error code %d\n",
			       __func__, rc);
		return 0;
	} else {
		rc = vreg_disable(vreg_sdslot);
		if (rc) {
			printk(KERN_ERR
			       "%s: Error disabling sd slot error code %d\n",
			       __func__, rc);
			return rc;
		}
		msm_gpios_disable(sdc3_off_gpio_table,
				  ARRAY_SIZE(sdc3_off_gpio_table));
	}

	for (i = 0; i < ARRAY_SIZE(mmc_vdd_table); i++) {
		if (mmc_vdd_table[i].mask == (1 << vdd)) {
			rc = vreg_set_level(vreg_sdslot,
					    mmc_vdd_table[i].level);
			if (rc) {
				printk(KERN_ERR
				       "%s: Error setting vreg level (%d)\n",
				       __func__, rc);
			}
			return rc;
		}
	}
	printk(KERN_ERR "%s: Invalid VDD %d specified\n", __func__, vdd);

	return 0;
}

static unsigned int htckovsky_sdslot_get_status(struct device *dev)
{
	return !gpio_get_value(KOVS100_N_SD_STATUS);
}

static struct mmc_platform_data htckovsky_sdslot_data = {
	.ocr_mask = MSM_MMC_VDD,
	.status = htckovsky_sdslot_get_status,
	.translate_vdd = sdslot_switchvdd,
};

/******************************************************************************
 * WiFi SDIO
 ******************************************************************************/
static struct vreg *vreg_wifi = NULL;

static void wifi_set_power(bool enable)
{
	gpio_set_value(KOVS100_WIFI_PWR, enable);
}

//Uh, that's kind of ugly, but we need to set power
//here to actually make the transceiver work
static struct wl12xx_platform_data wl1251_pdata = {
	.set_power = wifi_set_power,
};

static struct sdio_embedded_func wifi_func = {
	.f_class = SDIO_CLASS_WLAN,
	.f_maxblksize = 512,
};

static struct platform_device wl1251_device = {
	.id = -1,
	.name = "wl1251_data",
	.dev = {
		.platform_data = &wl1251_pdata,
	}
};

static struct embedded_sdio_data ti_wifi_emb_data = {
	.cis = {
		.vendor = 0x104c,
		.device = 0x9066,
		.blksize = 512,
		.max_dtr = 20000000,
		},
	.cccr = {
		 .multi_block = 0,
		 .low_speed = 0,
		 .wide_bus = 1,
		 .high_power = 0,
		 .high_speed = 0,
		 },
	.funcs = &wifi_func,
	.num_funcs = 1,
};

static uint32_t wifi_switchvdd(struct device *dev, unsigned int vdd)
{
	int rc;

	if (vdd) {
		msm_gpios_enable(sdc1_on_gpio_table,
				 ARRAY_SIZE(sdc1_on_gpio_table));

		mdelay(50);
		rc = vreg_enable(vreg_wifi);
		if (rc) {
			printk(KERN_ERR "%s: failed to enable vreg %d\n",
			       __func__, rc);
			goto pwroff;
		}

		//We need to set power here to make the card identify
		mdelay(100);
		wifi_set_power(true);
		mdelay(100);
		return 0;
	}

pwroff:
	vreg_disable(vreg_wifi);
	//We need to disable power here to save power
	wifi_set_power(false);
	mdelay(200);
	msm_gpios_disable(sdc1_off_gpio_table, ARRAY_SIZE(sdc1_off_gpio_table));
	return 0;
}

static struct mmc_platform_data htckovsky_wifi_data = {
	.built_in = 1,
	.ocr_mask = MMC_VDD_28_29 | MMC_VDD_29_30,
	.embedded_sdio = &ti_wifi_emb_data,
	.translate_vdd = wifi_switchvdd,
};

static int htckovsky_mmc_probe(struct platform_device *pdev)
{
	int ret = -EINVAL;

	ret = gpio_request(KOVS100_N_SD_STATUS, "HTC Kovsky SD Status");
	if (ret)
		goto fail_sd_status;

	ret = gpio_request(KOVS100_WIFI_PWR, "HTC Kovsky wifi power");
	if (ret)
		goto fail_wifi_pwr;

	ret = gpio_request(KOVS100_WIFI_IRQ, "wl1251 irq");
	if (ret)
		goto fail_wifi_irq;

	gpio_direction_input(KOVS100_N_SD_STATUS);
	gpio_direction_input(KOVS100_WIFI_IRQ);

	vreg_sdslot = vreg_get_by_id(0, 23);
	if (IS_ERR(vreg_sdslot)) {
		ret = PTR_ERR(vreg_sdslot);
		goto fail_vreg_sdslot;
	}
	vreg_wifi = vreg_get_by_id(0, 5);
	if (IS_ERR(vreg_wifi)) {
		ret = PTR_ERR(vreg_wifi);
		goto fail_vreg_wifi;
	}

	ret =
	    msm_gpios_request(sdc3_on_gpio_table,
			      ARRAY_SIZE(sdc3_on_gpio_table));
	if (ret)
		goto fail_gpios3;
	ret =
	    msm_gpios_request(sdc1_on_gpio_table,
			      ARRAY_SIZE(sdc1_on_gpio_table));
	if (ret)
		goto fail_gpios1;

	set_irq_wake(MSM_GPIO_TO_INT(KOVS100_N_SD_STATUS), 1);
	ret =
	    msm_add_sdcc(3, &htckovsky_sdslot_data,
			 MSM_GPIO_TO_INT(KOVS100_N_SD_STATUS),
			 IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING);
	if (ret)
		goto fail_sdcc3;

	ret = msm_add_sdcc(1, &htckovsky_wifi_data, 0, 0);
	if (ret) {
		printk(KERN_ERR
		       "%s: failed to register sdcc 1 for wifi, ret=%d\n",
		       __func__, ret);
		goto fail_sdcc1;
	}

	//Register the platform data, but do not care if it succeeds
	//if not, we can just shut all the power off via rfkill
	platform_device_register(&wl1251_device);
	return 0;

 fail_sdcc1:
	platform_device_unregister(&msm_device_sdc3);
 fail_sdcc3:
	msm_gpios_free(sdc1_on_gpio_table, ARRAY_SIZE(sdc1_on_gpio_table));
 fail_gpios1:
	msm_gpios_free(sdc3_on_gpio_table, ARRAY_SIZE(sdc3_on_gpio_table));
 fail_gpios3:
	vreg_put(vreg_wifi);
	vreg_wifi = NULL;
 fail_vreg_wifi:
	vreg_put(vreg_sdslot);
	vreg_sdslot = NULL;
 fail_vreg_sdslot:
	gpio_free(KOVS100_WIFI_IRQ);
 fail_wifi_irq:
	gpio_free(KOVS100_WIFI_PWR);
 fail_wifi_pwr:
	gpio_free(KOVS100_N_SD_STATUS);
 fail_sd_status:
	return ret;
}

static struct platform_driver htckovsky_mmc_driver = {
	.probe = htckovsky_mmc_probe,
	.driver = {
		   .name = "htckovsky-mmc",
		   .owner = THIS_MODULE,
		   },
};

static int __init htckovsky_mmc_init(void)
{
	return platform_driver_register(&htckovsky_mmc_driver);
}

module_init(htckovsky_mmc_init);
