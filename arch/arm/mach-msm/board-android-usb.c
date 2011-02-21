/* linux/arch/arm/mach-msm/board-android-usb.c
 *
 * Copyright (C) 2010 htc-linux.org
 * based on devices.c by HTC Corporation
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

#include "board-android-usb.h"

/***************************************************************
 * Android stuff
 ***************************************************************/
#ifdef CONFIG_USB_ANDROID
static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

#ifdef CONFIG_USB_ANDROID_DIAG
static char *usb_functions_adb_diag[] = {
	"usb_mass_storage",
	"adb",
	"diag",
};
#endif

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x0ffe,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x0c01,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x0c02,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
	{
		.product_id	= 0x0ffc,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
#ifdef CONFIG_USB_ANDROID_DIAG
	{
		.product_id	= 0x0fff,
		.num_functions	= ARRAY_SIZE(usb_functions_adb_diag),
		.functions	= usb_functions_adb_diag,
	},
#endif
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "HTC",
	.product	= "XDA",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x18d1,
	.vendorDescr	= "HTC",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};
#endif


static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0bb4,
	.product_id	= 0x0c01,
	.version	= 0x0100,
	.serial_number		= "000000000000",
	.product_name		= "XDA",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

/***************************************************************
 * End of android stuff
 ***************************************************************/

static int android_usb_probe(struct platform_device *pdev)
{
	int ret = 0;

#ifdef CONFIG_USB_ANDROID
	ret = platform_device_register(&android_usb_device);
	if (ret)
        goto exit_or_fail;
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	ret = platform_device_register(&rndis_device);
    if (ret)
        goto exit_or_fail;
#endif
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	ret = platform_device_register(&usb_mass_storage_device);
    if (ret)
        goto exit_or_fail;
#endif

exit_or_fail:
	return ret;
}

static int android_usb_remove(struct platform_device *pdev)
{
#ifdef CONFIG_USB_ANDROID_MASS_STORAGE
	platform_device_unregister(&usb_mass_storage_device);
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	platform_device_unregister(&rndis_device);
#endif
#ifdef CONFIG_USB_ANDROID
	platform_device_unregister(&android_usb_device);
#endif
	return 0;
}


static struct platform_driver android_usb_driver = {
	.probe		= android_usb_probe,
	.remove		= android_usb_remove,
	.driver		= {
		.name		= "android_usb_devices",
		.owner		= THIS_MODULE,
	},
};

static int __init android_usb_init(void)
{
	return platform_driver_register(&android_usb_driver);
}

static void __exit android_usb_exit(void)
{
	platform_driver_unregister(&android_usb_driver);
}

module_init(android_usb_init);
module_exit(android_usb_exit)
