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

#include <linux/microp-ng.h>
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


static unsigned char xperia_green_pattern[] = {
	0x11,
	0x00, 0x00, 0x00, 0xff, 0x08, 0xff, 0x10, 0xff, 0x18, 0xff,
	0x20, 0xff, 0x28, 0xff, 0x30, 0xff, 0x38, 0xff, 0xfc, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x10, 0x00, 0x18, 0x00,
	0x20, 0x00, 0x28, 0x00, 0x30, 0x00, 0x38, 0x00, 0xfc, 0x00,
	0x00, 0x00, 0x00, 0xff, 0x08, 0xff, 0x10, 0xff, 0x18, 0xff,
	0x20, 0xff, 0x28, 0xff, 0x30, 0xff, 0x38, 0xff, 0xfc, 0x00,
	0x08, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x00,
};

static unsigned char xperia_purple_pattern[] = {
	0x11,
	0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x10, 0x00, 0x18, 0x00,
	0x20, 0x00, 0x28, 0x00, 0x30, 0x00, 0x38, 0x00, 0xfc, 0x00,
	0x00, 0x00, 0x00, 0xff, 0x08, 0xff, 0x10, 0xff, 0x18, 0xff,
	0x20, 0xff, 0x28, 0xff, 0x30, 0xff, 0x38, 0xff, 0xfc, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x10, 0x00, 0x18, 0x00,
	0x20, 0x00, 0x28, 0x00, 0x30, 0x00, 0x38, 0x00, 0xfc, 0x00,
	0x08, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x00,
};

static void htckovsky_set_pattern(char *cmd, size_t count)
{
	static unsigned char off_cmd[] = {
		0x11, 0xfc, 1,
	};

	static unsigned char on_cmd[] = {
		0x11, 0, 0,
	};

	if (count != 0x47)
		return;

	microp_ng_write(client, off_cmd, ARRAY_SIZE(off_cmd));
	microp_ng_write(client, cmd, count);
	microp_ng_write(client, on_cmd, ARRAY_SIZE(on_cmd));
}

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

static int htckovsky_microp_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;

	printk(KERN_INFO "+%s\n", __func__);
	client = dev_get_drvdata(&pdev->dev);

	for (i = 0; i < ARRAY_SIZE(kovsky_leds); i++) {
		ret = led_classdev_register(&pdev->dev, &kovsky_leds[i]);
		if (ret < 0) {
			goto led_fail;
		}
	}
	printk(KERN_INFO "-%s\n", __func__);

	return 0;

led_fail:
	for (i--; i >= 0; i--) {
		led_classdev_unregister(&kovsky_leds[i]);
	}
	return ret;
}

static int htckovsky_microp_remove(struct platform_device *pdev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(kovsky_leds); i++) {
		led_classdev_unregister(&kovsky_leds[i]);
	}
	client = NULL;
	return 0;
}

#if CONFIG_PM
static int htckovsky_microp_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	htckovsky_set_pattern(xperia_green_pattern,
	ARRAY_SIZE(xperia_green_pattern));
	return 0;
}

static int htckovsky_microp_resume(struct platform_device *pdev)
{
	htckovsky_set_pattern(xperia_purple_pattern,
        ARRAY_SIZE(xperia_purple_pattern));
	return 0;
}
#else
#define htckovsky_microp_suspend NULL
#define htckovsky_microp_resume NULL
#endif

static struct platform_driver htckovsky_microp_driver = {
	.probe		= htckovsky_microp_probe,
	.remove		= htckovsky_microp_remove,
	.suspend	= htckovsky_microp_suspend,
	.resume		= htckovsky_microp_resume,
	.driver		= {
		.name		= "htckovsky-microp-leds",
		.owner		= THIS_MODULE,
	},
};

static int __init htckovsky_microp_init(void)
{
	return platform_driver_register(&htckovsky_microp_driver);
}

static void __exit htckovsky_microp_exit(void)
{
	platform_driver_unregister(&htckovsky_microp_driver);
}

module_init(htckovsky_microp_init);
module_exit(htckovsky_microp_exit)
