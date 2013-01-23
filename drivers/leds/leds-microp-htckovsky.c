/*
htckovsky-microp - the driver for the multifunction controller
on HTC Kovsky (Sony Ericsson Xperia X1). The hardware includes
programmable color leds, two PWMs for LCD and keypad backlight,
light sensor and optical joystick

Copyright (2012) Tor Berg <tob@samfundet.no>
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
#include <linux/delay.h>
#include <linux/mfd/microp-ng.h>
#include <linux/microp-htckovsky.h>

static int leds_init_done = 0;

static void htckovsky_update_color_leds(struct work_struct* work);
static void htckovsky_update_backlight(struct work_struct* work);
static void htckovsky_update_button_light(struct work_struct* work);

static void htckovsky_set_backlight(struct led_classdev*, enum led_brightness);
static void htckovsky_set_button_light(struct led_classdev*, enum led_brightness);
static void htckovsky_set_brightness_color(struct led_classdev*, enum led_brightness);

struct timer_list sensor_timer;

static long debuglevel = 0;
module_param(debuglevel, long, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(debuglevel, "Debuglevel");

static long auto_brightness = 0;
module_param(auto_brightness, long, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(auto_brightness, "Enable auto brightness");


static long low_level = 40;
module_param(low_level, long, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(low_level, "Lowest_level");

static long high_level = 300;
module_param(high_level, long, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(high_level, "Lowest_level");

static long div = 3;
module_param(div, long, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(div, "divisor");


static long mult = 2;
module_param(mult, long, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(mult, "Multiplier");

static long div2 = 1;
module_param(div2, long, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(div2, "2nd order divisor");


static long mult2 = 0;
module_param(mult2, long, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(mult2, "2nd order Multiplier");


static DECLARE_WORK(colorled_wq, htckovsky_update_color_leds);
//static DECLARE_WORK(leds_init_wq, leds_init);
static DECLARE_WORK(backlight_wq, htckovsky_update_backlight);
static DECLARE_WORK(buttonlight_wq, htckovsky_update_button_light);
static struct i2c_client *client = NULL;
static int read_light=0;
static int enable_backlight=1;
static int unblank = 0;

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
    int brightness = (kovsky_leds[LCD].brightness)&0xff;
    int calc_brightness;
    int light=0;
    if(debuglevel) printk("%s: 0x%.8X\n", __func__, kovsky_leds[LCD].brightness);
    
    if(brightness== 0xff){
      auto_brightness = 1;
      if(debuglevel) printk("%s: Turning ON auto brightness\n", __func__);
    } else {
      auto_brightness = 0;
      if(debuglevel) printk("%s: Turning OFF auto brightness\n", __func__);
    }
    
    if(unblank){
 	buffer[0] = MICROP_LCD_BRIGHTNESS_KOVS;
        buffer[1] = LED_FULL;
        buffer[2] = 0x7f;
        microp_ng_write(client, buffer, 3);
	microp_ng_write(client, buffer, 3);
	msleep(100);
        unblank=0;
    }
    
    
    if(brightness && read_light && auto_brightness && enable_backlight) {
       // printk("Auto screen brightness enabled %x\n", kovsky_leds[LCD].brightness);
        microp_ng_read(client, 0x30, buffer, 2);
	light=(buffer[1] + (buffer[0] << 8));
       // printk("Lightsensor returned %.2x %.2x\n", buffer[0], buffer[1]);
	if(div != 0 && div2 != 0)
	  calc_brightness = (low_level + light*mult/div + light*light*mult2/div2) ;
	else
	  calc_brightness = low_level;
		      
	if(debuglevel) printk("%s: read %x got %d used %d\n", __func__, light , brightness, calc_brightness);
	
	if(calc_brightness> high_level) calc_brightness = high_level;
	
	buffer[0] = MICROP_LCD_BRIGHTNESS_KOVS;
        buffer[1] = LED_FULL;
        buffer[2] = ((calc_brightness) >> 1)&0xff;
        microp_ng_write(client, buffer, 3);

        read_light=0;

    } else if (!brightness || !enable_backlight) {
        buffer[1] = buffer[2] = 0;

        buffer[0] = MICROP_LCD_BRIGHTNESS_KOVS;
        microp_ng_write(client, buffer, 3);

        buffer[0] = 0x11;
        microp_ng_write(client, buffer, 3);

        buffer[0] = 0x13;
        if(debuglevel) printk("%s: Turning off\n", __func__);
	microp_ng_write(client, buffer, 3);
    }   else if(!auto_brightness) {
	buffer[0] = MICROP_LCD_BRIGHTNESS_KOVS;
        buffer[1] = LED_FULL;
        buffer[2] = (brightness & 0xff) >> 1;
	if(debuglevel) printk("%s: used %d\n", __func__, brightness);
	microp_ng_write(client, buffer, 3);
    }
      
}


void sensor_timer_routine(unsigned long data)
{
  read_light=1;
  schedule_work(&backlight_wq);
   mod_timer(&sensor_timer, jiffies + HZ);
    return;

}

static void htckovsky_update_button_light(struct work_struct* work) {
	char buffer[3] = {MICROP_KEYPAD_BRIGHTNESS_KOVS, 0, 0};
	char brightness = kovsky_leds[BUTTONS].brightness;
	if (brightness) {
		buffer[1] = 0x94 >> 2;
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
	
	 	char buffer[10];
	char buffer41[72] = {0x41, 0x00, 0x00, 0x03, 
			   0x00, 0x0A, 0xDA, 0x11,
			   0xA1, 0x18, 0xB6, 0x1F,
			   0xE6, 0x26, 0x6C, 0x2D,
			   0x00, 0x2E, 0x00, 0x41,
			   0x00, 0x00, 0x00, 0x03,
			   0xF0, 0x0A, 0x35, 0x11,
			   0xA1, 0x18, 0x1F, 0x1F,
			   0x00, 0x26, 0x00, 0x2D,
			   0x00, 0x2E, 0x00, 0x41,
			   0x00, 0x00, 0x00, 0x03,
			   0x00, 0x0A, 0x00, 0x11,
			   0x00, 0x18, 0x13, 0x1F,
			   0xC0, 0x26, 0xF0, 0x2D,
			   0x00, 0x2E, 0x00, 0x41,
			   0x00, 0x0A, 0x0F, 0x0F,
			   0x0F, 0x0F, 0x0F, 0x0F,
			   0x0F, 0x0F, 0x0F, 0x00};
	
	char buffer51[22] = { 51, 00 , 00 , 0x19 ,
                                00 , 0x64 , 00, 0x65 ,
                                00 , 0x66 , 00 , 0x67 ,
                                00 , 0x68 , 00 , 0x69 ,
                                00 , 0x6A , 00 , 0x7D ,
                                00, 00};

				
	char buffer42[71] = {     0x42, 00, 0x00, 0x19,
                                00, 0x32, 0xF0, 0x4B,
                                00, 0x50, 0x00, 0x51,
                                00, 0x52, 0x00, 0x53,
                                00, 0x54, 0x00, 0x64,
                                00, 0x00, 0x00, 0x19,
                                00, 0x32, 0xF0, 0x4B,
                                00, 0x50, 0x00, 0x51,
                                00, 0x52, 0x00, 0x53,
                                00, 0x54, 0x00, 0x64,
                                00, 0x00, 0x00, 0x19,
                                00, 0x32, 0xF0, 0x4B,
                                00, 0x50, 0x00, 0x51, 
                                00, 0x52, 0x00, 0x53,
                                00, 0x54, 0x00, 0x64,
                                00, 0x0A, 0x05, 0x05,
                                0x05, 0x05, 0x05, 0x05,
                                0x05, 0x05, 0x05};
	char buffer44[71] = { 44, 0x00, 0x00, 0x0C,
				0xF0, 0x19, 0x00, 0x20,
				0x00, 0x21, 0x00, 0x22,
				0x00, 0x23, 0x00, 0x24,
				0x00, 0x25, 0x00, 0x32,
				0x00, 0x00, 0x00, 0x0C,
				0xF0, 0x19, 0x00, 0x20,
				0x00, 0x21, 0x00, 0x22,
				0x00, 0x23, 0x00, 0x24,
				0x00, 0x25, 0x00, 0x32,
				0x00, 0x00, 0x00, 0x0C,
				0xF0, 0x19, 0x00, 0x20,
				0x00, 0x21, 0x00, 0x22,
				0x00, 0x23, 0x00, 0x24,
				0x00, 0x25, 0x00, 0x32,
				0x00, 0x0A, 0x0F, 0x0F,
				0x0F, 0x0F, 0x0F, 0x0F,
				0x0F, 0x0F, 0x0F};
	char buffer53[]= {0x53, 0x00, 0x00, 0x19,
			  0xF0, 0x32, 0x00, 0x33,
			  0x00, 0x34, 0x00, 0x35,
			  0x00, 0x36, 0x00, 0x37,
			  0x00, 0x38, 0x00, 0x39,
			  0x00};
	int len=2;
	char addr=0x07;
	client = dev_get_drvdata(&pdev->dev);
	
	microp_ng_read(client, addr, buffer, len);
	printk("micropLEDS: R %d %.2x: %.2x %.2x %.2x\n", len, addr, buffer[0], buffer[1], buffer[2]);
	buffer[0] = 0x02; buffer[1]= 0x01;
	microp_ng_write(client, buffer, 2);
	microp_ng_write(client, buffer, 2);
	microp_ng_read(client, addr, buffer, len);
	printk("micropLEDS: R %d %.2x: %.2x %.2x %.2x\n", len, addr, buffer[0], buffer[1], buffer[2]);
	
	printk("micropLEDS: Sending 41\n");
	microp_ng_write(client, buffer41, 72);
	printk("micropLEDS: Sending 51\n");
	microp_ng_write(client, buffer51, 22);
	buffer[0] = 0x11; buffer[1]= 0x00;
	microp_ng_write(client, buffer, 2);
	buffer[0] = 0x11; buffer[1]= 0x01;
	microp_ng_write(client, buffer, 2);


	printk("micropLEDS: Sending 42\n");
	microp_ng_write(client, buffer42, 71);
	printk("micropLEDS: Sending 44\n");
	microp_ng_write(client, buffer44, 71);
	printk("micropLEDS: Sending 51\n");
	microp_ng_write(client, buffer51, 21);
	printk("micropLEDS: Sending 53\n");
	microp_ng_write(client, buffer53, 21);
	buffer[0] = 0x03; buffer[1]= 0x00;
	microp_ng_write(client, buffer, 2);
	buffer[0] = 0xBA; buffer[1]= 0x5A;
	microp_ng_write(client, buffer, 2);
	buffer[0] = 0xF0; buffer[1]= 0x02;
	microp_ng_write(client, buffer, 2);
	addr=0xF1; len=2;
	microp_ng_read(client, addr, buffer, len);
	printk("micropLEDS: R %d %.2x: %.2x %.2x %.2x\n", len, addr, buffer[0], buffer[1], buffer[2]);
	buffer[0] = 0xF0; buffer[1]= 0x03;
	microp_ng_write(client, buffer, 2);
	addr=0xF1; len=2;
	microp_ng_read(client, addr, buffer, len);
	printk("micropLEDS: R %d %.2x: %.2x %.2x %.2x\n", len, addr, buffer[0], buffer[1], buffer[2]);
	buffer[0] = 0xF0; buffer[1]= 0x04;
	microp_ng_write(client, buffer, 2);
	addr=0xF1; len=2;
	microp_ng_read(client, addr, buffer, len);
	printk("micropLEDS: R %d %.2x: %.2x %.2x %.2x\n", len, addr, buffer[0], buffer[1], buffer[2]);
	buffer[0] = 0xF7; buffer[1]= 0x01;
	microp_ng_write(client, buffer, 2);
	buffer[0] = 0x80; buffer[1]= 0x00;
	microp_ng_write(client, buffer, 2);
	buffer[0] = 0x80; buffer[1]= 0x01;
	microp_ng_write(client, buffer, 2);
	buffer[0] = 0xF7; buffer[1]= 0x01;
	microp_ng_write(client, buffer, 2);
	buffer[0] = 0x80; buffer[1]= 0x00;
	microp_ng_write(client, buffer, 2);
	buffer[0] = 0x80; buffer[1]= 0x01;
	microp_ng_write(client, buffer, 2);
	buffer[0] = 0x80; buffer[1]= 0x00;
	microp_ng_write(client, buffer, 2);
	addr=0x30; len=2;
	microp_ng_read(client, addr, buffer, len);
	printk("micropLEDS: R %d %.2x: %.2x %.2x %.2x\n", len, addr, buffer[0], buffer[1], buffer[2]);
	
	 leds_init_done=1;
	for (i = 0; i < ARRAY_SIZE(kovsky_leds); i++) {
		ret = led_classdev_register(&pdev->dev, &kovsky_leds[i]);
		if (ret < 0) {
			goto led_fail;
		}
	}
      kovsky_leds[LCD].brightness=0xff;
#if 1
      init_timer(&sensor_timer);
      sensor_timer.function = sensor_timer_routine;
      sensor_timer.data = (unsigned long)client;
      sensor_timer.expires = jiffies + HZ;
      add_timer(&sensor_timer); /* Starting the timer */

#endif

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
	del_timer_sync(&sensor_timer);
	client = NULL;
	
	return 0;
}

void htckovsky_leds_enable_backlight(int enabled)
{
  read_light=1;
  enable_backlight=enabled;
  unblank = enabled;
  schedule_work(&backlight_wq);
  return;
}

#if CONFIG_PM
static int htckovsky_leds_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	pr_debug("+%s\n", __func__);
	del_timer_sync(&sensor_timer);
	cancel_work_sync(&colorled_wq);
	cancel_work_sync(&backlight_wq);
	cancel_work_sync(&buttonlight_wq);
	pr_debug("-%s\n", __func__);
	return 0;
}

static int htckovsky_leds_resume(struct platform_device *pdev)
{
	pr_debug("%s\n", __func__);
	mod_timer(&sensor_timer, jiffies + (HZ/3));
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

MODULE_AUTHOR("Alexander Tarasikov <alexander.tarasikov@gmail.com>");
MODULE_DESCRIPTION("MicroP manager driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.3");
module_init(htckovsky_leds_init);
module_exit(htckovsky_leds_exit)
