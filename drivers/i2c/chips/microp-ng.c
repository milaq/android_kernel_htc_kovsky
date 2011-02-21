/*
    microp-ng.c - i2c chip driver for microp-led

    Copyright (C) 2011 Alexander Tarasikov <alexander.tarasikov@gmail.com>

    Original driver is (C) 2007-2011
    Joe Hansche <madcoder@gmail.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; version 2 of the License.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/microp-ng.h>

#define I2C_READ_RETRY_TIMES 10
#define I2C_WRITE_RETRY_TIMES 10

static struct i2c_client *client_ptr = NULL;

int microp_ng_write(uint8_t *sendbuf, int len)
{
	int rc;
	int retry;

	struct i2c_msg msg[] = {
		{
		 .addr = client_ptr->addr,
		 .flags = 0,
		 .len = len,
		 .buf = sendbuf,
		 },
	};

	if (!client_ptr) {
		printk(KERN_ERR "%s: i2c_client is NULL\n", __func__);
		return -EIO;
	}

	msg[0].addr = client_ptr->addr;

	for (retry = 0; retry <= I2C_WRITE_RETRY_TIMES; retry++) {
		rc = i2c_transfer(client_ptr->adapter, msg, 1);
		if (rc == 1) {
			rc = 0;
			goto exit;
		}
		msleep(10);
		printk(KERN_WARNING "microp_ng, i2c write retry\n");
	}

	if (retry >= I2C_WRITE_RETRY_TIMES)
		printk(KERN_ERR
		       "microp_ng_write, i2c_write_block retry over %d\n",
		       I2C_WRITE_RETRY_TIMES);

exit:
	return rc;
}

EXPORT_SYMBOL(microp_ng_write);

int microp_ng_read(uint8_t id, uint8_t *buf, int len)
{
	int retry;
	int rc;

	struct i2c_msg msgs[] = {
		{
		 .addr = 0,
		 .flags = 0,
		 .len = 1,
		 .buf = &id,
		 },
		{
		 .addr = 0,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 }
	};

	if (!client_ptr) {
		printk(KERN_ERR "%s: i2c_client is NULL\n", __func__);
		return -EIO;
	}

	msgs[0].addr = msgs[1].addr = client_ptr->addr;

	for (retry = 0; retry <= I2C_READ_RETRY_TIMES; retry++) {
		rc = i2c_transfer(client_ptr->adapter, msgs, 2);
		if (rc == 2) {
			rc = 0;
			goto exit;
		}
		msleep(10);
		printk(KERN_WARNING "microp_ng, i2c read retry\n");
	}

	if (retry >= I2C_WRITE_RETRY_TIMES) {
		dev_err(&client_ptr->dev, "i2c_read_block retry over %d\n",
			I2C_READ_RETRY_TIMES);
		rc = -EIO;
	}

exit:
	return rc;
}
EXPORT_SYMBOL(microp_ng_read);

static int microp_ng_remove(struct i2c_client *client)
{
	client_ptr = NULL;
	return 0;
}

static int microp_ng_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct microp_platform_data *data = NULL;
	int r, i = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_ERR "%s: i2c bus not supported\n", __func__);
		r = -EINVAL;
		goto fail;
	}

	if (!client->dev.platform_data) {
		printk(KERN_ERR "%s: platform data is null\n", __func__);
		r = -EINVAL;
		goto fail;
	}

	data = (struct microp_platform_data *)(client->dev.platform_data);
	client_ptr = client;
	
	if (!data->nclients) {
	  printk(KERN_ERR "%s: platform did not specify any clients\n", __func__);
	  r = -EINVAL;
	  goto fail;
	}

	if (!data->is_supported()) {
		printk(KERN_WARNING
		       "%s: This hardware is not yet supported\n",
		       __func__);
		r = -ENOTSUPP;
		goto fail;
	}
	
	for (i = 0; i < data->nclients; i++) {
	  r = platform_device_register(data->clients[i]);
	  if (r)
	    goto fail;
	}

	return 0;
fail:
	for (i--; i >= 0; i--) {
	  platform_device_unregister(data->clients[i]);
	}
	client_ptr = NULL;
	return r;
}

static const struct i2c_device_id microp_ng_ids[] = {
	{"microp-ng", 0},
	{}
};

static struct i2c_driver microp_ng_driver = {
	.driver = {
		.name = "microp-ng",
		.owner = THIS_MODULE,
	},
	.id_table = microp_ng_ids,
	.probe = microp_ng_probe,
	.remove = microp_ng_remove,
};

static int __init microp_ng_init(void)
{
	printk(KERN_INFO "microp-ng: Registering MicroP-LED driver\n");
	return i2c_add_driver(&microp_ng_driver);
}

static void __exit microp_ng_exit(void)
{
	printk(KERN_INFO "microp-ng: Unregistered MicroP-LED driver\n");
	i2c_del_driver(&microp_ng_driver);
}

MODULE_AUTHOR("Alexander Tarasikov <alexander.tarasikov@gmail.com>");
MODULE_DESCRIPTION("MicroP manager driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.2");
module_init(microp_ng_init);
module_exit(microp_ng_exit);
