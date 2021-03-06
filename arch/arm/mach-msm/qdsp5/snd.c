/* arch/arm/mach-msm/qdsp5/snd.c
 *
 * interface to "snd" service on the baseband cpu
 *
 * Copyright (C) 2008 HTC Corporation
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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/msm_audio.h>

#include <asm/atomic.h>
#include <asm/ioctls.h>
#include <mach/board.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_smd.h>
#include "adsp.h"

struct snd_ctxt {
	struct mutex lock;
	int opened;
	struct msm_rpc_endpoint *ept;
	struct msm_snd_platform_data *snd_pdata;
};

static struct snd_ctxt the_snd;


static int currentDevice = 0;
static int IdleDeviceID = 0;

static inline int get_idle_deviceid(void)

{

	int i;

	struct snd_ctxt *snd = &the_snd;

	for (i = 0; i < snd->snd_pdata->num_endpoints; i++) {

		if (!strcmp("IDLE", snd->snd_pdata->endpoints[i].name)) {

			printk("Found SND_DEVICE_IDLE id %d\n", snd->snd_pdata->endpoints[i].id);

			return snd->snd_pdata->endpoints[i].id;

		}

	}

	printk("SND_DEVICE_IDLE not found\n");

	/* default to earcuple */

	return 0;

}

struct rpc_snd_set_device_args {
	uint32_t device;
	uint32_t ear_mute;
	uint32_t mic_mute;

	uint32_t cb_func;
	uint32_t client_data;
};

struct rpc_snd_set_volume_args {
	uint32_t device;
	uint32_t method;
	uint32_t volume;

	uint32_t cb_func;
	uint32_t client_data;
};

struct snd_set_device_msg {
	struct rpc_request_hdr hdr;
	struct rpc_snd_set_device_args args;
};

struct snd_set_volume_msg {
	struct rpc_request_hdr hdr;
	struct rpc_snd_set_volume_args args;
};

struct msm_snd_endpoint *get_msm_snd_endpoints(int *size);

static inline int check_mute(int mute)
{
	return (mute == SND_MUTE_MUTED ||
		mute == SND_MUTE_UNMUTED) ? 0 : -EINVAL;
}

static int get_endpoint(struct snd_ctxt *snd, unsigned long arg)
{
	int rc = 0, index;
	struct msm_snd_endpoint ept;

	if (copy_from_user(&ept, (void __user *)arg, sizeof(ept))) {
		pr_err("snd_ioctl get endpoint: invalid read pointer.\n");
		return -EFAULT;
	}

	index = ept.id;
	if (index < 0 || index >= snd->snd_pdata->num_endpoints) {
		pr_err("snd_ioctl get endpoint: invalid index!\n");
		return -EINVAL;
	}

	ept.id = snd->snd_pdata->endpoints[index].id;
	strncpy(ept.name,
		snd->snd_pdata->endpoints[index].name,
		sizeof(ept.name));

	if (copy_to_user((void __user *)arg, &ept, sizeof(ept))) {
		pr_err("snd_ioctl get endpoint: invalid write pointer.\n");
		rc = -EFAULT;
	}

	return rc;
}

static inline void check_device(int *device)
{

	/* Is device ID out of range ? */

	if (*device > IdleDeviceID || *device < 0) {

		pr_err("%s: snd device %d out of range: use last device %d\n", __func__,

			*device, currentDevice);

		*device = currentDevice;

	} else {

		currentDevice = *device;

	}

}
static long snd_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct snd_set_device_msg dmsg;
	struct snd_set_volume_msg vmsg;
	struct msm_snd_device_config dev;
	struct msm_snd_volume_config vol;
	struct snd_ctxt *snd = file->private_data;
	int rc = 0;

	mutex_lock(&snd->lock);
	switch (cmd) {
	case SND_SET_DEVICE:
		if (copy_from_user(&dev, (void __user *) arg, sizeof(dev))) {
			pr_err("snd_ioctl set device: invalid pointer.\n");
			rc = -EFAULT;
			break;
		}
	
	/* Prevent wrong device to make the snd processor crashing */

        check_device(&dev.device);
	
		dmsg.args.device = cpu_to_be32(dev.device);
		dmsg.args.ear_mute = cpu_to_be32(dev.ear_mute);
		dmsg.args.mic_mute = cpu_to_be32(dev.mic_mute);
		if (check_mute(dev.ear_mute) < 0 ||
				check_mute(dev.mic_mute) < 0) {
			pr_err("snd_ioctl set device: invalid mute status.\n");
			rc = -EINVAL;
			break;
		}
		dmsg.args.cb_func = -1;
		dmsg.args.client_data = 0;

		pr_debug("snd_set_device %d %d %d\n", dev.device,
						 dev.ear_mute, dev.mic_mute);

		rc = msm_rpc_call(snd->ept,
			adsp_info->snd_device_proc,
			&dmsg, sizeof(dmsg), 5 * HZ);
		break;

	case SND_SET_VOLUME:
		if (copy_from_user(&vol, (void __user *) arg, sizeof(vol))) {
			pr_err("snd_ioctl set volume: invalid pointer.\n");
			rc = -EFAULT;
			break;
		}

		vmsg.args.device = cpu_to_be32(vol.device);
		vmsg.args.method = cpu_to_be32(vol.method);
		if (vol.method != SND_METHOD_VOICE && vol.method != SND_METHOD_AUDIO) {
			pr_err("snd_ioctl set volume: invalid method.\n");
			rc = -EINVAL;
			break;
		}

		vmsg.args.volume = cpu_to_be32(vol.volume);
		vmsg.args.cb_func = -1;
		vmsg.args.client_data = 0;

		pr_debug("snd_set_volume %d %d %d\n", vol.device,
						vol.method, vol.volume);

		rc = msm_rpc_call(snd->ept,
			adsp_info->snd_volume_proc,
			&vmsg, sizeof(vmsg), 5 * HZ);
		break;

	case SND_GET_NUM_ENDPOINTS:
		if (copy_to_user((void __user *)arg,
				&snd->snd_pdata->num_endpoints, sizeof(unsigned))) {
			pr_err("snd_ioctl get endpoint: invalid pointer.\n");
			rc = -EFAULT;
		}
		break;

	case SND_GET_ENDPOINT:
		rc = get_endpoint(snd, arg);
		break;

	default:
		pr_err("snd_ioctl unknown command.\n");
		rc = -EINVAL;
		break;
	}
	mutex_unlock(&snd->lock);

	return rc;
}

static int snd_release(struct inode *inode, struct file *file)
{
	struct snd_ctxt *snd = file->private_data;

	mutex_lock(&snd->lock);
	snd->opened = 0;
	mutex_unlock(&snd->lock);
	return 0;
}

static int snd_open(struct inode *inode, struct file *file)
{
	struct snd_ctxt *snd = &the_snd;
	int rc = 0;

	mutex_lock(&snd->lock);
	if (snd->opened == 0) {
		if (snd->ept == NULL) {
			snd->ept = msm_rpc_connect(adsp_info->snd_prog, adsp_info->snd_vers,
					MSM_RPC_UNINTERRUPTIBLE);
			if (IS_ERR(snd->ept)) {
				rc = PTR_ERR(snd->ept);
				snd->ept = NULL;
				pr_err("snd: failed to connect snd svc\n");
				goto err;
			}
		}
		file->private_data = snd;
		snd->opened = 1;
	} else {
		pr_err("snd already opened.\n");
		rc = -EBUSY;
	}

err:
	mutex_unlock(&snd->lock);
	return rc;
}

static struct file_operations snd_fops = {
	.owner		= THIS_MODULE,
	.open		= snd_open,
	.release	= snd_release,
	.unlocked_ioctl	= snd_ioctl,
};

struct miscdevice snd_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_snd",
	.fops	= &snd_fops,
};

static int snd_probe(struct platform_device *pdev)
{
	int rc;
	struct snd_ctxt *snd = &the_snd;
	pr_debug("Loading MSM sound driver\n");
	if (!adsp_info) {
		printk(KERN_ERR "%s: ADSP is not registered, aborting\n", __func__);
		rc = -ENODEV;
		goto ret;
	}
	mutex_init(&snd->lock);
	snd->snd_pdata = (struct msm_snd_platform_data *)pdev->dev.platform_data;
	/* Find the idle sound device */
	
      IdleDeviceID = get_idle_deviceid();

      rc = misc_register(&snd_misc);
	


ret:
	pr_debug("Loaded MSM sound driver rc=%d\n", rc);
	return rc;
}

static struct platform_driver snd_plat_driver = {
	.probe = snd_probe,
	.driver = {
		.name = "msm_snd",
		.owner = THIS_MODULE,
	},
};

static int __init snd_init(void)
{
	return platform_driver_register(&snd_plat_driver);
}

module_init(snd_init);
