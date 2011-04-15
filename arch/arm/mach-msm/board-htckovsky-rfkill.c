/* linux/arch/arm/mach-msm/board-htckovsky-rfkill.c
 *
 * Copyright (C) 2010-2011 Alexander Tarasikov <alexander.tarasikov@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
*/

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include "board-htckovsky.h"

static struct rfkill *bt_rfk;
static const char bt_name[] = "brf6300";
static struct vreg *vreg_bt;

static struct msm_gpio htckovsky_bt_on_table[] = {
    { .gpio_cfg = GPIO_CFG(KOVS100_UART2DM_RTS, 4,
						GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
						.label = "BT RTS"},
	{ .gpio_cfg = GPIO_CFG(KOVS100_UART2DM_CTS, 4,
							GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							.label = "BT CTS"},
	{ .gpio_cfg = GPIO_CFG(KOVS100_UART2DM_RX,  4,
							GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							.label = "BT RXD"},
	{ .gpio_cfg = GPIO_CFG(KOVS100_UART2DM_TX,  2,
							GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							.label = "BT TXD"},
};
static struct msm_gpio htckovsky_bt_off_table[] = {
	{ .gpio_cfg = GPIO_CFG(KOVS100_UART2DM_RTS, 0,
							GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							.label = "BT RTS"},
    { .gpio_cfg = GPIO_CFG(KOVS100_UART2DM_CTS, 0,
						GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
						.label = "BT CTS"},
	{ .gpio_cfg = GPIO_CFG(KOVS100_UART2DM_RX,  0,
							GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
							.label = "BT RXD"},
	{ .gpio_cfg = GPIO_CFG(KOVS100_UART2DM_TX,  0,
							GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							.label = "BT TXD"},
};

static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked) {
			msm_gpios_enable(htckovsky_bt_on_table,
								ARRAY_SIZE(htckovsky_bt_on_table));
			vreg_enable(vreg_bt);
			gpio_direction_output(KOVS100_BT_ROUTER, 1);
			gpio_direction_output(KOVS100_BT_POWER, 0);
			mdelay(20);
			gpio_direction_output(KOVS100_BT_POWER, 1);
			mdelay(20);
	} else {
			gpio_direction_output(KOVS100_BT_POWER, 0);
			vreg_disable(vreg_bt);
			msm_gpios_disable(htckovsky_bt_off_table,
								ARRAY_SIZE(htckovsky_bt_off_table));
			gpio_set_value(KOVS100_BT_ROUTER, 0);
			mdelay(20);
	}
	return 0;
}

static struct rfkill_ops htckovsky_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int htckovsky_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;

	vreg_bt = vreg_get_by_id(0, 5);
	if (IS_ERR(vreg_bt)) {
		rc = PTR_ERR(vreg_bt);
		goto fail_vreg_bt;
	}
	rc = gpio_request(KOVS100_BT_POWER, "BT Power");
	if (rc)
		goto fail_power_gpio;

	rc = gpio_request(KOVS100_BT_ROUTER, "BT Router");
	if (rc)
		goto fail_router_gpio;

	rc = msm_gpios_request(htckovsky_bt_off_table, ARRAY_SIZE(htckovsky_bt_off_table));
	if (rc)
		goto fail_gpios;

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
			   &htckovsky_rfkill_ops, NULL);
	if (!bt_rfk) {
		rc = -ENOMEM;
		goto fail_rfk_alloc;
	}

	rfkill_set_states(bt_rfk, true, false);
	
	rc = rfkill_register(bt_rfk);
	if (rc)
		goto fail_rfk_reg;


	return 0;

fail_rfk_reg:
	rfkill_destroy(bt_rfk);
fail_rfk_alloc:
	msm_gpios_disable_free(htckovsky_bt_off_table, ARRAY_SIZE(htckovsky_bt_off_table));
fail_gpios:
	gpio_free(KOVS100_BT_ROUTER);
fail_router_gpio:
	gpio_free(KOVS100_BT_POWER);
fail_power_gpio:
	vreg_put(vreg_bt);
fail_vreg_bt:
	return rc;
}

static int htckovsky_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);
	msm_gpios_disable_free(htckovsky_bt_on_table, ARRAY_SIZE(htckovsky_bt_on_table));
	gpio_free(KOVS100_BT_ROUTER);
	gpio_free(KOVS100_BT_POWER);
	vreg_put(vreg_bt);
	return 0;
}

static struct platform_driver htckovsky_rfkill_driver = {
	.probe = htckovsky_rfkill_probe,
	.remove = htckovsky_rfkill_remove,
	.driver = {
		.name = "htckovsky_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init htckovsky_rfkill_init(void)
{
	return platform_driver_register(&htckovsky_rfkill_driver);
}

static void __exit htckovsky_rfkill_exit(void)
{
	platform_driver_unregister(&htckovsky_rfkill_driver);
}

module_init(htckovsky_rfkill_init);
module_exit(htckovsky_rfkill_exit);
MODULE_DESCRIPTION("HTC Kovsky rfkill");
MODULE_AUTHOR("Alexander Tarasikov <alexander.tarasikov@gmail.com>");
MODULE_LICENSE("GPL");
