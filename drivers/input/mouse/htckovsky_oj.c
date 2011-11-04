/*
 * htckovsky_oj - the driver for the optical joystick found
 * in HTC Kovsky (Sony Ericsson Xperia X1) mobile phones
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/input.h>

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <linux/mfd/microp-ng.h>
#include <linux/microp-htckovsky.h>

#include <mach/../../board-htckovsky.h>
#include <mach/vreg.h>

#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>

#define MODULE_NAME "htckovsky-microp-oj"
#define OJ_POLL_DELAY 100

#define OJ_THRESHOLD 40

static struct htckovsky_oj_t {
	struct mutex lock;
	struct i2c_client *client;
	struct input_dev *input;
	struct work_struct oj_work;
	struct platform_device *pdev;
	struct vreg *vreg_pwr;
	struct vreg *vreg_28v;

	struct task_struct *task;
} htckovsky_oj;

static void htckovsky_oj_reset(void) {
	char buffer[2];
	buffer[0] = MICROP_OJ_RESET_KOVS;
	buffer[1] = MICROP_CMD_OJ_RESET_KOVS;
	microp_ng_write(htckovsky_oj.client, buffer, ARRAY_SIZE(buffer));
}

static void htckovsky_oj_overlow(void) {
	char buffer[2];
	buffer[0] = MICROP_OJ_OVERFLOW_KOVS;
	buffer[1] = 0x0;
	microp_ng_write(htckovsky_oj.client, buffer, ARRAY_SIZE(buffer));
}

static void htckovsky_oj_power(bool enable) {
	char buffer[2];
	
	if (enable) {
		vreg_set_level(htckovsky_oj.vreg_pwr, 2800);
		vreg_enable(htckovsky_oj.vreg_pwr);
		vreg_enable(htckovsky_oj.vreg_28v);
		
		buffer[0] = MICROP_OJ_POWER_KOVS;
		buffer[1] = MICROP_CMD_OJ_PWR_ON;
		microp_ng_write(htckovsky_oj.client, buffer, ARRAY_SIZE(buffer));
		htckovsky_oj_reset();	
	}
	else {
		buffer[0] = MICROP_OJ_POWER_KOVS;
		buffer[1] = MICROP_CMD_OJ_PWR_OFF;
		microp_ng_write(htckovsky_oj.client, buffer, ARRAY_SIZE(buffer));
		
		vreg_disable(htckovsky_oj.vreg_28v);
		vreg_disable(htckovsky_oj.vreg_pwr);
	}
}

static irqreturn_t htckovsky_oj_interrupt(int irq, void *handle)
{
	schedule_work(&htckovsky_oj.oj_work);
	return IRQ_HANDLED;
}

static void htckovsky_oj_key_emu(int dx, int dy) {
	input_report_key(htckovsky_oj.input, KEY_UP, 0);
	input_report_key(htckovsky_oj.input, KEY_DOWN, 0);
	input_report_key(htckovsky_oj.input, KEY_LEFT, 0);
	input_report_key(htckovsky_oj.input, KEY_RIGHT, 0);

	if (abs(dx) > abs(dy)) {
		if (dx > OJ_THRESHOLD) {
			input_report_key(htckovsky_oj.input, KEY_LEFT, 1);
		}
		else if (dx < -OJ_THRESHOLD) {
			input_report_key(htckovsky_oj.input, KEY_RIGHT, 1);
		}
	}
	else {
		if (dy > OJ_THRESHOLD) {
			input_report_key(htckovsky_oj.input, KEY_DOWN, 1);
		}
		else if (dy < -OJ_THRESHOLD) {
			input_report_key(htckovsky_oj.input, KEY_UP, 1);
		}
	}
}

static void htckovsky_oj_work(struct work_struct *work)
{
	signed char buffer[3] = {};
	mutex_lock(&htckovsky_oj.lock);

	microp_ng_read(htckovsky_oj.client, 0xf5, buffer, 3);
	pr_debug("%s: [%02x %02x %02x]\n", __func__,
		buffer[0] & 0xff, buffer[1] & 0xff, buffer[2] & 0xff);

	if ((buffer[0] & 0xff) == 0xe0) {
		input_report_rel(htckovsky_oj.input, REL_X, -buffer[1]);
		input_report_rel(htckovsky_oj.input, REL_Y, -buffer[2]);
	
		htckovsky_oj_key_emu(buffer[1], buffer[2]);
		input_sync(htckovsky_oj.input);
		goto done;
	}
	
	if ((buffer[0] & 0x90) == 0x10) {
		pr_debug("%s: incorrect data\n", __func__);
		htckovsky_oj_reset();
	}
	else if (buffer[1] && buffer[2]) {
		pr_debug("%s: data overflow\n", __func__);
		htckovsky_oj_overlow();
	}
done:
	mutex_unlock(&htckovsky_oj.lock);
}

static int htckovsky_oj_thread(void *v) {
	struct completion exit;
	siginfo_t info;

	daemonize("oj_thread");
	allow_signal(SIGKILL);
	init_completion(&exit);

	while (true) {
		if (signal_pending(current)) {
			if (dequeue_signal_lock(current, &current->blocked, &info)
				== SIGKILL) {
				goto done;
			}
		}
		htckovsky_oj_work(NULL);
		msleep(OJ_POLL_DELAY);
	}

done:
	complete_and_exit(&exit, 0);
}

static void htckovsky_oj_start_polling(void) {
	struct task_struct *nthread;
	
	mutex_lock(&htckovsky_oj.lock);
	if (htckovsky_oj.task) {
		goto done;
	}

	nthread = kthread_create(htckovsky_oj_thread, NULL, "oj_thread");
	if (!nthread) {
		pr_err(MODULE_NAME": failed to start OJ refresh thread\n");
		goto done;
	}
	wake_up_process(nthread);
	htckovsky_oj.task = nthread;
done:
	mutex_unlock(&htckovsky_oj.lock);
}

static void htckovsky_oj_stop_polling(void) {
	mutex_lock(&htckovsky_oj.lock);
	if (!htckovsky_oj.task) {
		goto exit;
	}
	send_sig(SIGKILL, htckovsky_oj.task, 0);
	htckovsky_oj.task = NULL;
exit:
	mutex_unlock(&htckovsky_oj.lock);
}

static int htckovsky_oj_probe(struct platform_device *pdev)
{
	struct input_dev *input = NULL;
	struct i2c_client *client;
	int ret = 0;

	pr_info(MODULE_NAME ": Initializing MicroP Optical Joystick driver\n");

	client = dev_get_drvdata(&pdev->dev);

	if (!client) {
		pr_err(MODULE_NAME ": failed to get i2c_client via drvdata\n");
		ret = -EINVAL;
		goto fail_invalid_data;
	}
	htckovsky_oj.client = client;

	if (pdev->id != -1) {
		pr_err(MODULE_NAME ": device id != -1, aborting\n");
		ret = -EINVAL;
		goto fail_invalid_data;
	}

	mutex_init(&htckovsky_oj.lock);
	INIT_WORK(&htckovsky_oj.oj_work, htckovsky_oj_work);
	// Initialize input device
	input = input_allocate_device();
	if (!input) {
		ret = -ENODEV;
		goto fail_input_alloc;
	}

	input->name = MODULE_NAME;
	input->phys = client->adapter->name;
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	__set_bit(EV_REL, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	input_set_capability(input, EV_REL, REL_X);
	input_set_capability(input, EV_REL, REL_Y);
	
	input_set_capability(input, EV_KEY, KEY_UP);
	input_set_capability(input, EV_KEY, KEY_DOWN);
	input_set_capability(input, EV_KEY, KEY_LEFT);
	input_set_capability(input, EV_KEY, KEY_RIGHT);

	ret = input_register_device(input);
	if (ret) {
		goto fail_input_register;
	}

	htckovsky_oj.pdev = pdev;
	htckovsky_oj.input = input;
	platform_set_drvdata(pdev, &htckovsky_oj);

	ret = gpio_request(KOVS100_JOYSTICK_IRQ, "Joystick IRQ");
	if (ret) {
		goto fail_gpio_request;
	}
	gpio_direction_input(KOVS100_JOYSTICK_IRQ);

	htckovsky_oj.vreg_pwr = vreg_get_by_id(&pdev->dev, 11);
	if (!htckovsky_oj.vreg_pwr) {
		goto fail_vreg_pwr;
	}

	htckovsky_oj.vreg_28v = vreg_get_by_id(&pdev->dev, 17);
	if (!htckovsky_oj.vreg_28v) {
		goto fail_vreg_28v;
	}

	ret = request_irq(gpio_to_irq(KOVS100_JOYSTICK_IRQ),
		htckovsky_oj_interrupt, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		"Joystick IRQ", NULL);
	if (ret) {
		pr_err("Couldn't request joystick IRQ error: %d\n", ret);
		goto fail_irq;
	}

	device_init_wakeup(&pdev->dev, 1);
	htckovsky_oj_start_polling();

	return 0;

fail_irq:
	vreg_put(htckovsky_oj.vreg_28v);
fail_vreg_28v:
	vreg_put(htckovsky_oj.vreg_pwr);
fail_vreg_pwr:
	gpio_free(KOVS100_JOYSTICK_IRQ);
fail_gpio_request:
	input_unregister_device(input);
fail_input_register:
	input_free_device(input);
fail_input_alloc:
fail_invalid_data:
	return ret;
}

static int htckovsky_oj_remove(struct platform_device *pdev)
{
	htckovsky_oj_stop_polling();

	free_irq(gpio_to_irq(KOVS100_JOYSTICK_IRQ), pdev);
	gpio_free(KOVS100_JOYSTICK_IRQ);
	
	cancel_work_sync(&htckovsky_oj.oj_work);
	
	htckovsky_oj_power(false);
	vreg_put(htckovsky_oj.vreg_28v);
	vreg_put(htckovsky_oj.vreg_pwr);

	input_unregister_device(htckovsky_oj.input);
	input_free_device(htckovsky_oj.input);

	htckovsky_oj.client = NULL;
	return 0;
}

static int htckovsky_oj_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	htckovsky_oj_stop_polling();
	cancel_work_sync(&htckovsky_oj.oj_work);
	htckovsky_oj_power(false);
	return 0;
}

static int htckovsky_oj_resume(struct platform_device *pdev)
{
	htckovsky_oj_power(true);
	htckovsky_oj_start_polling();
	return 0;
}

static struct platform_driver htckovsky_oj_driver = {
	.driver = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
	},
	.probe = htckovsky_oj_probe,
	.remove = htckovsky_oj_remove,
	.suspend = htckovsky_oj_suspend,
	.resume = htckovsky_oj_resume,

};

static int __init htckovsky_oj_init(void)
{
	pr_info("Registering MicroP keypad driver\n");
	return platform_driver_register(&htckovsky_oj_driver);
}

static void __exit htckovsky_oj_exit(void)
{
	pr_info("Unregistered MicroP keypad driver\n");
	platform_driver_unregister(&htckovsky_oj_driver);
}

MODULE_AUTHOR("Alexander Tarasikov <alexander.tarasikov@gmail.com>");
MODULE_DESCRIPTION("MicroP Optical Joystick driver");
MODULE_LICENSE("GPL");

module_init(htckovsky_oj_init);
module_exit(htckovsky_oj_exit);
