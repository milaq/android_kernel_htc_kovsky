/* linux/arch/arm/mach-msm/amss_5225.c
 *
 * Copyright (C) 2011 htc-linux.org
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

#include <linux/platform_device.h>
#include <mach/msm_smd.h>
#include <mach/amss/amss_5225.h>

static struct platform_device adsp_device = {
	.name = "msm_adsp_5225",
	.id = -1,
};

static struct msm_early_server smd_5225_early_servers[] = {
	{
		.pid = 1,
		.cid = 0xfadefade,
		.prog = 0x3000fffe,
		.vers = 1,
	}
};

static struct msm_smd_platform_data smd_pdata_5225 = {
	.amss_values = amss_5225_para,
	.n_amss_values = ARRAY_SIZE(amss_5225_para),
	.early_servers = smd_5225_early_servers,
	.n_early_servers = ARRAY_SIZE(smd_5225_early_servers),
};

static const struct smd_tty_channel_desc smd_5225_tty_channels[] = {
	{.id = 0,.name = "SMD_DS"},
	{.id = 1,.name = "SMD_DIAG"},
	{.id = 7,.name = "SMD_DATA1"},
};

static struct platform_device msm_device_smd = {
	.name	= "msm_smd",
	.dev = {
		.platform_data = &smd_pdata_5225,
	},
	.id	= -1,
};

static int amss_5225_probe(struct platform_device *pdev)
{
	int ret;
	//do it before anything rpc kicks in
	smd_set_channel_list(smd_5225_tty_channels, ARRAY_SIZE(smd_5225_tty_channels));

	//We're in a serious trouble
	ret = platform_device_register(&msm_device_smd);
	if (ret) {
		printk(KERN_CRIT "%s: failed to register SMD driver\n", __func__);
		return ret;
	}

	//We can live without adsp, though
	ret = platform_device_register(&adsp_device);
	if (ret) {
		printk(KERN_ERR "%s: failed to register ADSP driver\n", __func__);
	}

	return 0;
}

static struct platform_driver amss_5225_driver = {
	.probe		= amss_5225_probe,
	.driver		= {
		.name		= "amss_5225",
		.owner		= THIS_MODULE,
	},
};

static int __init amss_5225_init(void)
{
	return platform_driver_register(&amss_5225_driver);
}

module_init(amss_5225_init);
