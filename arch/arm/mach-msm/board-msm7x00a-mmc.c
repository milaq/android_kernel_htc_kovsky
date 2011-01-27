/* linux/arch/arm/mach-msm/board-msm7x00A-mmc.c
** Author: Brian Swetland <swetland@google.com>
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>
#include <linux/debugfs.h>

#include <asm/mach-types.h>
#include <asm/mach/mmc.h>
#include <asm/gpio.h>
#include <asm/io.h>

#include <mach/board.h>
#include <mach/htc_pwrsink.h>
#include <mach/msm7x00a_mmc.h>
#include <mach/vreg.h>

#include "devices.h"

#define DEBUG_SDSLOT_VDD 1

#define MSM72K_SDCC_COUNT 4

/* ---- COMMON ---- */
static void config_gpio_table(unsigned *table, int len, enum msm_gpio_state enable)
{
	int n;
	for (n = 0; n < len; n++) {
		gpio_tlmm_config(table[n], enable);
	}
}

// On and Off table sizes and gpios MUST match
static unsigned sdc1_on_gpio_table[] = {
	GPIO_CFG(51, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),	/* DAT3 */	
	GPIO_CFG(52, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),	/* DAT2 */
	GPIO_CFG(53, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),	/* DAT1 */
	GPIO_CFG(54, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),	/* DAT0 */
	GPIO_CFG(55, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),	/* CMD */
	GPIO_CFG(56, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CLK */
};

static unsigned sdc1_off_gpio_table[] = {
	GPIO_CFG(51, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT3 */
	GPIO_CFG(52, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT2 */
	GPIO_CFG(53, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT1 */
	GPIO_CFG(54, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT0 */
	GPIO_CFG(55, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CMD */
	GPIO_CFG(56, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CLK */
};

static unsigned sdc2_on_gpio_table[] = {
	GPIO_CFG(62, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CLK */
	GPIO_CFG(63, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CMD */
	GPIO_CFG(64, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT3 */
	GPIO_CFG(65, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT2 */
	GPIO_CFG(66, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT1 */
	GPIO_CFG(67, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT0 */
};

static unsigned sdc2_off_gpio_table[] = {
	GPIO_CFG(62, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CLK */
	GPIO_CFG(63, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CMD */
	GPIO_CFG(64, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT3 */
	GPIO_CFG(65, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT2 */
	GPIO_CFG(66, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT1 */
	GPIO_CFG(67, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT0 */
};

static unsigned sdc3_on_gpio_table[] = {
	GPIO_CFG(88, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	/* CLK */
	GPIO_CFG(89, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	/* CMD */
	GPIO_CFG(90, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	/* DAT3 */
	GPIO_CFG(91, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	/* DAT2 */
	GPIO_CFG(92, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	/* DAT1 */
	GPIO_CFG(93, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	/* DAT0 */
};

static unsigned sdc3_off_gpio_table[] = {
	GPIO_CFG(88, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CLK */
	GPIO_CFG(89, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CMD */
	GPIO_CFG(90, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT3 */
	GPIO_CFG(91, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT2 */
	GPIO_CFG(92, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT1 */
	GPIO_CFG(93, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT0 */
};

static unsigned sdc4_on_gpio_table[] = {
	GPIO_CFG(142, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	/* CLK */
	GPIO_CFG(143, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	/* CMD */
	GPIO_CFG(144, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	/* DAT3 */
	GPIO_CFG(145, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	/* DAT2 */
	GPIO_CFG(146, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	/* DAT1 */
	GPIO_CFG(147, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),	/* DAT0 */
};

static unsigned sdc4_off_gpio_table[] = {
	GPIO_CFG(142, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CLK */
	GPIO_CFG(143, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CMD */
	GPIO_CFG(144, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT3 */
	GPIO_CFG(145, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT2 */
	GPIO_CFG(146, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT1 */
	GPIO_CFG(147, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* DAT0 */
};

//just kill me...
static struct msm7x00a_mmc_platform_data *sdcc_data[MSM72K_SDCC_COUNT] = {
	NULL,
	NULL,
	NULL,
	NULL
};

static unsigned int sdslot_status(struct device *dev)
{
	unsigned int status = 0;
	int sdcc_id = -1;
	if (dev == &msm_device_sdc1.dev)
		sdcc_id = 0;
	if (dev == &msm_device_sdc2.dev)
		sdcc_id = 1;
	if (dev == &msm_device_sdc3.dev)
		sdcc_id = 2;
	if (dev == &msm_device_sdc4.dev)
		sdcc_id = 3;
	if (sdcc_id < 0)
		return 0;

	if (sdcc_data[sdcc_id] && sdcc_data[sdcc_id]->get_status)
		return sdcc_data[sdcc_id]->get_status();

	return status;
}

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

static uint32_t sdslot_switchvdd(struct device *dev, unsigned int vdd)
{
	int rc = 0, i, sdcc_id = -1;
	if (dev == &msm_device_sdc1.dev)
		sdcc_id = 0;
	if (dev == &msm_device_sdc2.dev)
		sdcc_id = 1;
	if (dev == &msm_device_sdc3.dev)
		sdcc_id = 2;
	if (dev == &msm_device_sdc4.dev)
		sdcc_id = 3;
	if (sdcc_id < 0)
		return 0;

	if (vdd == 0) {
		switch(sdcc_id) {
		case 0:	
			config_gpio_table(sdc1_off_gpio_table, ARRAY_SIZE(sdc1_on_gpio_table), GPIO_CFG_DISABLE);
			break;
		case 1:
			config_gpio_table(sdc2_off_gpio_table, ARRAY_SIZE(sdc2_on_gpio_table), GPIO_CFG_DISABLE);
			break;
		case 2:
			config_gpio_table(sdc3_off_gpio_table, ARRAY_SIZE(sdc3_off_gpio_table), GPIO_CFG_DISABLE);
			break;
		case 3:
			config_gpio_table(sdc4_off_gpio_table, ARRAY_SIZE(sdc4_off_gpio_table), GPIO_CFG_DISABLE);
			break;
		}
		if (sdcc_data[sdcc_id]->set_voltage)
			rc = sdcc_data[sdcc_id]->set_voltage(0);
		if (rc)
			printk(KERN_ERR "%s: Error disabling sd slot %d error code %d\n", __func__, sdcc_id + 1, rc);
		return 0;
	}
	else {
		if (sdcc_data[sdcc_id]->set_voltage)
			rc = sdcc_data[sdcc_id]->set_voltage(1);
		switch(sdcc_id) {
		case 0:	
			config_gpio_table(sdc1_on_gpio_table, ARRAY_SIZE(sdc1_on_gpio_table), GPIO_CFG_ENABLE);
			break;
		case 1:
			config_gpio_table(sdc2_on_gpio_table, ARRAY_SIZE(sdc2_on_gpio_table), GPIO_CFG_ENABLE);
			break;
		case 2:
			config_gpio_table(sdc3_on_gpio_table, ARRAY_SIZE(sdc3_on_gpio_table), GPIO_CFG_ENABLE);
			break;
		case 3:
			config_gpio_table(sdc4_on_gpio_table, ARRAY_SIZE(sdc4_on_gpio_table), GPIO_CFG_ENABLE);
			break;
		}
	}

	if (rc) {
		printk(KERN_ERR "%s: Error setting voltage %d\n", __func__, rc);
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(mmc_vdd_table); i++) {
		if (mmc_vdd_table[i].mask == (1 << vdd)) {
			printk("%s: Setting level to %u\n",
			__func__, mmc_vdd_table[i].level);
			if (sdcc_data[sdcc_id] && sdcc_data[sdcc_id]->set_voltage)
				rc = sdcc_data[sdcc_id]->set_voltage(!!vdd);
			if (rc) {
				printk(KERN_ERR
				"%s: Error setting vreg level (%d)\n",
				__func__, rc);
			}
			return 0;
		}
	}
	printk(KERN_ERR "%s: Invalid VDD %d specified\n", __func__, vdd);

	return 0;
}

#define MSM_MMC_VDD	MMC_VDD_165_195 | MMC_VDD_20_21 | MMC_VDD_21_22 \
			| MMC_VDD_22_23 | MMC_VDD_23_24 | MMC_VDD_24_25 \
			| MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 \
			| MMC_VDD_28_29 | MMC_VDD_29_30

static struct mmc_platform_data sdslot_data[MSM72K_SDCC_COUNT] = {
	{
		.ocr_mask = MSM_MMC_VDD,
		.status = sdslot_status,
		.translate_vdd = sdslot_switchvdd,
	},
	{
		.ocr_mask = MSM_MMC_VDD,
		.status = sdslot_status,
		.translate_vdd = sdslot_switchvdd,
	},
	{
		.ocr_mask = MSM_MMC_VDD,
		.status = sdslot_status,
		.translate_vdd = sdslot_switchvdd,
	},
	{
		.ocr_mask = MSM_MMC_VDD,
		.status = sdslot_status,
		.translate_vdd = sdslot_switchvdd,
	}
};

static int mmc_request_gpios(unsigned sdcc_id) {
	int n, ret = 0, len;
	unsigned *gpios;
	switch (sdcc_id) {
	case 1:
		gpios = sdc1_on_gpio_table;
		len = ARRAY_SIZE(sdc1_on_gpio_table);
		break;
	case 2:
		gpios = sdc2_on_gpio_table;
		len = ARRAY_SIZE(sdc2_on_gpio_table);
		break;
	case 3:
		gpios = sdc3_on_gpio_table;
		len = ARRAY_SIZE(sdc3_on_gpio_table);
		break;
	case 4:
		gpios = sdc4_on_gpio_table;
		len = ARRAY_SIZE(sdc4_on_gpio_table);
		break;
	default:
		return -EINVAL;
	}

	for (n = 0; n < len; n++) {
		ret = gpio_request(GPIO_PIN(gpios[n]), "MSM SDCC");
		if (ret)
			goto free_sd;
	}

	return ret;
	
free_sd:
	for (--n; n >= 0; n--) {
		gpio_free(GPIO_PIN(gpios[n]));
	}
	return ret;
}

static void mmc_free_gpios(unsigned sdcc_id) {
	int n, len;
	unsigned *gpios;
	switch (sdcc_id) {
	case 1:
		gpios = sdc1_on_gpio_table;
		len = ARRAY_SIZE(sdc1_on_gpio_table);
		break;
	case 2:
		gpios = sdc2_on_gpio_table;
		len = ARRAY_SIZE(sdc2_on_gpio_table);
		break;
	case 3:
		gpios = sdc3_on_gpio_table;
		len = ARRAY_SIZE(sdc3_on_gpio_table);
		break;
	case 4:
		gpios = sdc4_on_gpio_table;
		len = ARRAY_SIZE(sdc4_on_gpio_table);
		break;
	default:
		return;
	}

	for (n = 0; n < len; n++)
		gpio_free(GPIO_PIN(gpios[n]));
}


static int msm7x00a_mmc_probe(struct platform_device *pdev)
{
	int ret = -EINVAL;
	struct msm7x00a_mmc_platform_data *pdata = pdev->dev.platform_data;

	if (pdata->sdcc_id < 1 || pdata->sdcc_id > MSM72K_SDCC_COUNT)
		goto fail_pdata;
	if (!pdata->sd_irq)
		goto fail_pdata;

	if (pdata->init)
		ret = pdata->init(&pdev->dev);

	if (ret)
		goto fail_pdata;

	ret = mmc_request_gpios(pdata->sdcc_id);
	if (ret)
		goto fail_gpios;

	set_irq_wake(pdata->sd_irq, 1);
	ret = msm_add_sdcc(pdata->sdcc_id, &sdslot_data[pdata->sdcc_id - 1],
		pdata->sd_irq,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING);

	if (ret)
		goto fail_sdcc;

	sdcc_data[pdata->sdcc_id - 1] = pdata;
	return ret;

fail_sdcc:
	mmc_free_gpios(pdata->sdcc_id);
fail_gpios:
	if (pdata->exit)
		pdata->exit(&pdev->dev);
fail_pdata:
	return ret;
}

static int msm7x00a_mmc_remove(struct platform_device *pdev)
{
	struct msm7x00a_mmc_platform_data *pdata = pdev->dev.platform_data;
	sdcc_data[pdata->sdcc_id - 1] = NULL;

	set_irq_wake(pdata->sd_irq, 0);
	mmc_free_gpios(pdata->sdcc_id);

	if (pdata->exit)
		pdata->exit(&pdev->dev);

	return 0;
}

static struct platform_driver msm7x00a_mmc_driver = {
	.probe		= msm7x00a_mmc_probe,
	.remove		= msm7x00a_mmc_remove,
	.driver		= {
		.name		= "msm7x00a-mmc",
		.owner		= THIS_MODULE,
	},
};

static int __init msm7x00a_mmc_init(void)
{
	return platform_driver_register(&msm7x00a_mmc_driver);
}

static void __exit msm7x00a_mmc_exit(void)
{
	platform_driver_unregister(&msm7x00a_mmc_driver);
}

module_init(msm7x00a_mmc_init);
module_exit(msm7x00a_mmc_exit)
