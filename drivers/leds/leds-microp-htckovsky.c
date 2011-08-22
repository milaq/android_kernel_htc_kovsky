/*
htckovsky-microp - the driver for the multifunction controller
on HTC Kovsky (Sony Ericsson Xperia X1). The hardware includes
programmable color leds, two PWMs for LCD and keypad backlight,
light sensor and optical joystick

Copyright (2010) Alexander Tarasikov <alexander.tarasikov@gmail.com>
Some code was written by ultrashot at xda-developers

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <linux/mfd/microp-ng.h>
#include <linux/microp-htckovsky.h>

static void htckovsky_update_color_leds(struct work_struct* work);
static void htckovsky_update_backlight(struct work_struct* work);
static void htckovsky_update_button_light(struct work_struct* work);

static void htckovsky_set_backlight(struct led_classdev*, enum led_brightness);
static void htckovsky_set_button_light(struct led_classdev*, enum led_brightness);
static void htckovsky_set_brightness_color(struct led_classdev*, enum led_brightness);

static DECLARE_WORK(colorled_wq, htckovsky_update_color_leds);
static DECLARE_WORK(backlight_wq, htckovsky_update_backlight);
static DECLARE_WORK(buttonlight_wq, htckovsky_update_button_light);
static struct i2c_client *client = NULL;

enum kovsky_led {RED, GREEN, BLUE, LCD, BUTTONS};

static struct led_classdev kovsky_leds[] = {
	[RED] = {
	 .name = "red",
	 .brightness_set = htckovsky_set_brightness_color,
	 },
	[GREEN] = {
	 .name = "green",
	 .brightness_set = htckovsky_set_brightness_color,
	 .default_trigger = "usb-online",
	 },
	[BLUE] = {
	 .name = "blue",
	 .brightness_set = htckovsky_set_brightness_color,
	 },
	[LCD] = {
	 .name = "lcd-backlight",
	 .brightness_set = htckovsky_set_backlight,
	 },
	[BUTTONS] = {
	 .name = "button-backlight",
	 .brightness_set = htckovsky_set_button_light,
	 }
};

static void htckovsky_update_color_leds(struct work_struct* work) {
	char buf[2] = {0x20, (!!kovsky_leds[RED].brightness)
	| ((!!kovsky_leds[GREEN].brightness) << 1) | ((!!kovsky_leds[BLUE].brightness) << 2) | 0x80};
	microp_ng_write(client, buf, 2);
}

static void htckovsky_update_backlight(struct work_struct* work) {
	char buffer[3] = {};
	enum led_brightness brightness = kovsky_leds[LCD].brightness;
	if (brightness) {
		buffer[0] = MICROP_LCD_BRIGHTNESS_KOVS;
		buffer[1] = LED_FULL;
		buffer[2] = brightness;
		microp_ng_write(client, buffer, 3);
	}
	else {
		buffer[1] = buffer[2] = 0;

		buffer[0] = MICROP_LCD_BRIGHTNESS_KOVS;
		microp_ng_write(client, buffer, 3);

		buffer[0] = 0x11;
		microp_ng_write(client, buffer, 3);

		buffer[0] = 0x13;
		microp_ng_write(client, buffer, 3);
	}
}

static void htckovsky_update_button_light(struct work_struct* work) {
	char buffer[3] = {MICROP_KEYPAD_BRIGHTNESS_KOVS, 0, 0};
	char brightness = kovsky_leds[BUTTONS].brightness;
	if (brightness) {
		buffer[1] = 0x94;
		buffer[2] = brightness >> 2;
	}
	microp_ng_write(client, buffer, 3);
}

static void htckovsky_set_brightness_color(struct led_classdev *led_cdev,
					 enum led_brightness brightness)
{
	schedule_work(&colorled_wq);
}

static void htckovsky_set_button_light(struct led_classdev *led_cdev,
					       enum led_brightness brightness)
{
	schedule_work(&buttonlight_wq);
}

static void htckovsky_set_backlight(struct led_classdev *led_cdev,
					 enum led_brightness brightness)
{
	schedule_work(&backlight_wq);
}

static int htckovsky_leds_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;

	client = dev_get_drvdata(&pdev->dev);

	for (i = 0; i < ARRAY_SIZE(kovsky_leds); i++) {
		ret = led_classdev_register(&pdev->dev, &kovsky_leds[i]);
		if (ret < 0) {
			goto led_fail;
		}
	}

	return 0;

led_fail:
	for (i--; i >= 0; i--) {
		led_classdev_unregister(&kovsky_leds[i]);
	}
	return ret;
}

static int htckovsky_leds_remove(struct platform_device *pdev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(kovsky_leds); i++) {
		led_classdev_unregister(&kovsky_leds[i]);
	}
	client = NULL;
	return 0;
}

#if CONFIG_PM
static int htckovsky_leds_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	pr_debug("+%s\n", __func__);
	cancel_work_sync(&colorled_wq);
	cancel_work_sync(&backlight_wq);
	cancel_work_sync(&buttonlight_wq);
	pr_debug("-%s\n", __func__);
	return 0;
}

static int htckovsky_leds_resume(struct platform_device *pdev)
{
	pr_debug("%s\n", __func__);
	return 0;
}
#else
#define htckovsky_leds_suspend NULL
#define htckovsky_leds_resume NULL
#endif

static struct platform_driver htckovsky_leds_driver = {
	.probe		= htckovsky_leds_probe,
	.remove		= htckovsky_leds_remove,
	.suspend	= htckovsky_leds_suspend,
	.resume		= htckovsky_leds_resume,
	.driver		= {
		.name		= "htckovsky-microp-leds",
		.owner		= THIS_MODULE,
	},
};

static int __init htckovsky_leds_init(void)
{
	return platform_driver_register(&htckovsky_leds_driver);
}

static void __exit htckovsky_leds_exit(void)
{
	platform_driver_unregister(&htckovsky_leds_driver);
}

module_init(htckovsky_leds_init);
module_exit(htckovsky_leds_exit)
