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
#include <linux/slab.h>
#include <linux/mfd/microp-ng.h>

#define I2C_READ_RETRY_TIMES 10
#define I2C_WRITE_RETRY_TIMES 10

int microp_ng_write(struct i2c_client *client_ptr, uint8_t *sendbuf, int len)
{
	int rc;
	int retry;

	struct i2c_msg msgs[] = {
		{
		 .addr = 0,
		 .flags = 0,
		 .len = len,
		 .buf = sendbuf,
		 },
	};

	if (!client_ptr) {
		pr_err("%s: client_ptr is null\n", __func__);
		return -EIO;
	}

	msgs[0].addr = client_ptr->addr;

	for (retry = 0; retry <= I2C_WRITE_RETRY_TIMES; retry++) {
		rc = i2c_transfer(client_ptr->adapter, msgs, 1);
		if (rc == 1) {
			rc = 0;
			goto exit;
		}
		msleep(10);
		pr_err("microp_ng, i2c write retry\n");
	}

	if (retry >= I2C_WRITE_RETRY_TIMES) {
		pr_err("microp_ng_write, i2c_write_block retry over %d\n",
		       I2C_WRITE_RETRY_TIMES);
		rc = -ETIMEDOUT;
	}

exit:
	return rc;
}

EXPORT_SYMBOL(microp_ng_write);

int microp_ng_read(struct i2c_client *client_ptr, uint8_t id, uint8_t *buf, int len)
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
		pr_err("%s: client_ptr is null\n", __func__);
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
		pr_err("microp_ng, i2c read retry\n");
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
	struct microp_platform_data *data = NULL;
	int i;
	data = (struct microp_platform_data*)client->dev.platform_data;
	for (i = data->nclients - 1; i >= 0; i--) {
		dev_set_drvdata(&data->clients[i]->dev, NULL);
		platform_device_unregister(data->clients[i]);
	}
	return 0;
}

static uint16_t microp_ng_get_version(struct i2c_client *client, unsigned char reg) {
	uint8_t version[2];

	int ret = microp_ng_read(client, reg, version, 2);
	if (ret < 0) {
	  pr_err("%s: error reading microp version %d\n", __func__, ret);
	  return false;
	}
	pr_info("%s: version %x%x\n", __func__, version[0], version[1]);
	return ((version[0] << 8) | version[1]);
}

static int microp_ng_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct microp_platform_data *data = NULL;
	int r, i = 0;
	uint16_t version;
	bool found = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("%s: i2c bus not supported\n", __func__);
		r = -EINVAL;
		goto fail;
	}

	if (!client->dev.platform_data) {
		pr_err("%s: platform data is null\n", __func__);
		r = -EINVAL;
		goto fail;
	}

	data = (struct microp_platform_data *)(client->dev.platform_data);

	if (!data->nclients) {
	  pr_err("%s: platform did not specify any clients\n", __func__);
	  r = -EINVAL;
	  goto fail;
	}

	if (!data->comp_versions || !data->n_comp_versions) {
		pr_err("%s: platform did not specify compatible versions\n",
				 __func__);
		r = -EINVAL;
		goto fail;
	}

	version = microp_ng_get_version(client, data->version_reg);
	for (i = 0; i < data->n_comp_versions; i++) {
		if (data->comp_versions[i] == version) {
			found = true;
			break;
		}
	}

	if (!found) {
		pr_err("%s: no compatible version was found\n",
				 __func__);
		r = -EINVAL;
		goto fail;
	}

	for (i = 0; i < data->nclients; i++) {
		dev_set_drvdata(&data->clients[i]->dev, client);
		r = platform_device_register(data->clients[i]);
		if (r)
			goto fail_pdev;
	}

	return 0;
fail_pdev:
	for (i--; i >= 0; i--) {
		dev_set_drvdata(&data->clients[i]->dev, NULL);
		platform_device_unregister(data->clients[i]);
	}
fail:
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
	pr_info("microp-ng: Registering MicroP-LED driver\n");
	return i2c_add_driver(&microp_ng_driver);
}

static void __exit microp_ng_exit(void)
{
	pr_info("microp-ng: Unregistered MicroP-LED driver\n");
	i2c_del_driver(&microp_ng_driver);
}

MODULE_AUTHOR("Alexander Tarasikov <alexander.tarasikov@gmail.com>");
MODULE_DESCRIPTION("MicroP manager driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.3");
module_init(microp_ng_init);
module_exit(microp_ng_exit);
