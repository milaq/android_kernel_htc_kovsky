/*
 *  The driver for the 3.5mm headset found on some HTC phones
 *  (namely, HTC Blackstone and HTC Kovsky (aka Sony Ericsson Xperia X1))
 *	these devices do not implement hds protocol, they just use gpios
 *	to signal jack and button state. Nevertheless, use hds-compatible interface
 *	for userspace
 *
 *	Copyright (C) 2010 htc-linux.org
 *  Copyright (C) 2008 Google, Inc.
 *  Copyright (C) 2008 HTC, Inc.
 *
 *  Authors:
 *		Alexander Tarasikov <alexander.tarasikov@gmail.com>
 *		based on HDS headset driver by:
 *      Laurence Chen <Laurence_Chen@htc.com>
 *      Nick Pelly <npelly@google.com>
 *      Thomas Tsai <thomas_tsai@htc.com>
 *      Farmer Tseng <farmer_tseng@htc.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>

#include <asm/atomic.h>
#include <mach/htc_headset_35mm.h>

#define HDSI(fmt, arg...) \
	printk(KERN_INFO "[HDS] %s " fmt, __func__, ## arg)
#define HDSE(fmt, arg...) \
	printk(KERN_ERR "[HDS] %s " fmt, __func__, ## arg)

#define CONFIG_DEBUG_HDS 1

#ifdef CONFIG_DEBUG_HDS
#define HDS_DBG(fmt, arg...) printk(KERN_INFO "[HDS] %s " fmt, __func__, ## arg)
#else
#define HDS_DBG(fmt, arg...) do {} while (0)
#endif

static struct workqueue_struct *g_detection_work_queue;
static void detection_work(struct work_struct *work);
static DECLARE_WORK(g_detection_work, detection_work);

struct hds_info {
	struct switch_dev sdev;
	struct input_dev *input;
	struct mutex mutex_lock;

	atomic_t btn_state;

	unsigned int irq;
	unsigned int irq_btn_35mm;

	int gpio_detect;
	int gpio_headset_mic;

	bool jack_inverted;
	int btn_11pin_35mm_flag;

	struct hrtimer timer;
	ktime_t debounce_time;

	struct hrtimer btn35mm_timer;
	ktime_t btn35mm_debounce_time;
};
static struct hds_info *hi = 0;

static void button_pressed(void)
{
	HDS_DBG("\n");
	atomic_set(&hi->btn_state, 1);
	input_report_key(hi->input, KEY_MEDIA, 1);
	input_sync(hi->input);
}

static void button_released(void)
{
	HDS_DBG("\n");
	atomic_set(&hi->btn_state, 0);
	input_report_key(hi->input, KEY_MEDIA, 0);
	input_sync(hi->input);
}

static void remove_headset(void)
{
	HDS_DBG("");

	mutex_lock(&hi->mutex_lock);
	switch_set_state(&hi->sdev, switch_get_state(&hi->sdev) &
			~(BIT_HEADSET | BIT_HEADSET_NO_MIC));
	mutex_unlock(&hi->mutex_lock);

	if (hi->btn_11pin_35mm_flag) {
		disable_irq(hi->irq_btn_35mm);
		//turn_mic_bias_on(0);
		hi->btn_11pin_35mm_flag = 0;
		if (atomic_read(&hi->btn_state))
			button_released();
	}
	HDS_DBG("removed 3.5mm headset\n");

	hi->debounce_time = ktime_set(0, 100000000);  /* 100 ms */
}

static void insert_headset(void)
{
	int state;

	HDS_DBG("");

	state = switch_get_state(&hi->sdev);
	state &= ~(BIT_HEADSET_NO_MIC | BIT_HEADSET);

	if (hi->gpio_headset_mic) {
		/* support 3.5mm earphone with mic */
		printk(KERN_INFO "3.5mm_headset plug in %d\n",
			gpio_get_value(hi->gpio_headset_mic));

		state |= BIT_HEADSET;
		if (!hi->btn_11pin_35mm_flag) {
			hi->btn35mm_debounce_time = ktime_set(0, 200 * 1000 * 1000);
			printk(KERN_INFO "enabling mic irq\n");
			set_irq_type(hi->irq_btn_35mm,
				     IRQF_TRIGGER_HIGH);
			enable_irq(hi->irq_btn_35mm);
			hi->btn_11pin_35mm_flag = 1;
		}
	}
	else
		state |= BIT_HEADSET_NO_MIC;

	hi->debounce_time = ktime_set(0, 500000000);  /* 500 ms */

	mutex_lock(&hi->mutex_lock);
	switch_set_state(&hi->sdev, state);
	mutex_unlock(&hi->mutex_lock);
}

static void detection_work(struct work_struct *work)
{
	unsigned long irq_flags;

	HDS_DBG();
	if (gpio_get_value(hi->gpio_detect) != hi->jack_inverted) {
		/* Headset not plugged in */
		if (switch_get_state(&hi->sdev) != HDS_NO_DEVICE)
			remove_headset();
		return;
	}

	/* Disable headset interrupt while detecting.*/
	local_irq_save(irq_flags);
	disable_irq(hi->irq);
	local_irq_restore(irq_flags);

	/* Restore IRQs */
	local_irq_save(irq_flags);
	enable_irq(hi->irq);
	local_irq_restore(irq_flags);

	insert_headset();
}

static enum hrtimer_restart button_35mm_event_timer_func(struct hrtimer *data)
{
	printk(KERN_INFO "%s\n", __func__);
	if (gpio_get_value(hi->gpio_headset_mic)) {
		button_pressed();
		hi->btn35mm_debounce_time = ktime_set(0, 200 * 1000 * 1000);
	} else {
		button_released();
		hi->btn35mm_debounce_time = ktime_set(0, 500 * 1000 * 1000);
	}

	return HRTIMER_NORESTART;
}

static enum hrtimer_restart detect_event_timer_func(struct hrtimer *data)
{
	HDS_DBG("");

	queue_work(g_detection_work_queue, &g_detection_work);
	return HRTIMER_NORESTART;
}

static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

	HDS_DBG("");
	do {
		value1 = gpio_get_value(hi->gpio_detect);
		set_irq_type(hi->irq, value1 ?
				IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(hi->gpio_detect);
	} while (value1 != value2 && retry_limit-- > 0);

	HDS_DBG("value2 = %d (%d retries), device=%d",
		value2, (10-retry_limit), switch_get_state(&hi->sdev));

	if ((switch_get_state(&hi->sdev) == HDS_NO_DEVICE) ^ value2 ^ hi->jack_inverted)
		hrtimer_start(&hi->timer, hi->debounce_time, HRTIMER_MODE_REL);

	return IRQ_HANDLED;
}

static irqreturn_t button_35mm_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

	HDS_DBG("");
	do {
		value1 = gpio_get_value(hi->gpio_headset_mic);
		set_irq_type(hi->irq_btn_35mm, value1 ?
				IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(hi->gpio_headset_mic);
	} while (value1 != value2 && retry_limit-- > 0);

	HDS_DBG("value2 = %d (%d retries)", value2, (10-retry_limit));

	hrtimer_start(&hi->btn35mm_timer, hi->btn35mm_debounce_time, HRTIMER_MODE_REL);

	return IRQ_HANDLED;

}

static int hds_probe(struct platform_device *pdev)
{
	int ret;
	struct htc_headset_35mm_pdata *pdata = pdev->dev.platform_data;

	HDSI("Registering 3.5mm headset driver\n");
	hi = kzalloc(sizeof(struct hds_info), GFP_KERNEL);
	if (!hi)
		return -ENOMEM;

	atomic_set(&hi->btn_state, 0);
	hi->debounce_time = ktime_set(0, 100000000);  /* 100 ms */
	hi->btn35mm_debounce_time = ktime_set(0, 500000000);  /* 50 ms */
	hi->btn_11pin_35mm_flag = 0;
	hi->gpio_detect = pdata->gpio_detect;
	hi->gpio_headset_mic = pdata->gpio_headset_mic;
	hi->jack_inverted = pdata->jack_inverted;
	hi->sdev.name = "h2w";

	mutex_init(&hi->mutex_lock);

	ret = switch_dev_register(&hi->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	g_detection_work_queue = create_workqueue("detection");
	if (g_detection_work_queue == NULL) {
		ret = -ENOMEM;
		goto err_create_work_queue;
	}

	if (hi->gpio_headset_mic) {
		ret = gpio_request(hi->gpio_headset_mic, "3.5mm mic detect");
		if (ret < 0)
			goto err_request_35mm_mic_detect_gpio;

		ret = gpio_direction_input(hi->gpio_headset_mic);
		if (ret < 0)
			goto err_set_35mm_mic_detect_gpio;

		hi->irq_btn_35mm = gpio_to_irq(hi->gpio_headset_mic);
		if (hi->irq_btn_35mm < 0) {
			ret = hi->irq_btn_35mm;
			goto err_request_btn_35mm_irq;
		}
		set_irq_flags(hi->irq_btn_35mm, IRQF_VALID | IRQF_NOAUTOEN);
		ret = request_irq(hi->irq_btn_35mm,
				  button_35mm_irq_handler,
				  IRQF_TRIGGER_HIGH, "35mm_button", NULL);
		if (ret < 0)
			goto err_request_btn_35mm_irq;
	}

	ret = gpio_request(hi->gpio_detect, "3.5mm jack");
	if (ret < 0)
		goto err_request_detect_gpio;

	ret = gpio_direction_input(hi->gpio_detect);
	if (ret < 0)
		goto err_set_detect_gpio;


	hi->irq = gpio_to_irq(hi->gpio_detect);
	if (hi->irq < 0) {
		ret = hi->irq;
		goto err_get_hds_detect_irq_num_failed;
	}


	hrtimer_init(&hi->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hi->timer.function = detect_event_timer_func;
	hrtimer_init(&hi->btn35mm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hi->btn35mm_timer.function = button_35mm_event_timer_func;

	ret = request_irq(hi->irq, detect_irq_handler,
			  IRQF_TRIGGER_LOW, "3.5mm headset detection", NULL);
	if (ret < 0)
		goto err_request_detect_irq;

	ret = set_irq_wake(hi->irq, 1);
	if (ret < 0)
		goto err_request_input_dev;

	hi->input = input_allocate_device();
	if (!hi->input) {
		ret = -ENOMEM;
		goto err_request_input_dev;
	}

	hi->input->name = "3.5mm headset";
	set_bit(EV_SYN, hi->input->evbit);
	set_bit(EV_KEY, hi->input->evbit);
	set_bit(KEY_MEDIA, hi->input->keybit);

	ret = input_register_device(hi->input);
	if (ret < 0)
		goto err_register_input_dev;

	return 0;

err_register_input_dev:
	input_free_device(hi->input);
err_request_input_dev:
	free_irq(hi->irq, 0);
err_request_detect_irq:
err_get_hds_detect_irq_num_failed:
err_set_detect_gpio:
	gpio_free(hi->gpio_detect);
err_request_detect_gpio:
	if (hi->gpio_headset_mic)
		free_irq(hi->irq_btn_35mm, 0);
err_request_btn_35mm_irq:
err_set_35mm_mic_detect_gpio:
	if (hi->gpio_headset_mic)
		gpio_free(hi->gpio_headset_mic);
err_request_35mm_mic_detect_gpio:
	destroy_workqueue(g_detection_work_queue);
err_create_work_queue:
	switch_dev_unregister(&hi->sdev);
err_switch_dev_register:
	HDSE("Failed to register driver\n");

	return ret;
}

static int hds_remove(struct platform_device *pdev)
{
	HDSI("Removing 3.5mm headset driver\n");
	if (switch_get_state(&hi->sdev))
		remove_headset();
	input_unregister_device(hi->input);
	if (hi->gpio_detect)
		gpio_free(hi->gpio_detect);
	if (hi->gpio_headset_mic) {
		gpio_free(hi->gpio_headset_mic);
		free_irq(hi->irq_btn_35mm, 0);
	}
	destroy_workqueue(g_detection_work_queue);
	switch_dev_unregister(&hi->sdev);

	return 0;
}


static struct platform_driver hds_driver = {
	.probe		= hds_probe,
	.remove		= hds_remove,
	.driver		= {
		.name		= "htc_headset_35mm",
		.owner		= THIS_MODULE,
	},
};

static int __init hds_init(void)
{
	return platform_driver_register(&hds_driver);
}

static void __exit hds_exit(void)
{
	platform_driver_unregister(&hds_driver);
}

module_init(hds_init);
module_exit(hds_exit);

MODULE_AUTHOR("Alexander Tarasikov <alexander.tarasikov@gmail.com>");
MODULE_DESCRIPTION("35mm simple headset driver for HTC phones");
MODULE_LICENSE("GPL");
