/* kionix-kxsd9.c
 *
 * G-Sensor found in HTC Raphael (Touch Pro) and HTC Diamond mobile phones
 *
 * also acts as pedometer and free-fall detector
 * reports g-force as x,y,z
 * reports step count

    ^
 +y |
    ___
-x | -'| +x
<--|   |-->
   |___| / -z
   |_O_|/
 -y |  /
    v / +z

 * TODO: calibration, report free fall, ..
 *
 * Job Bolle <jb@b4m.com>
 */

#include <asm/mach-types.h>
#include <asm/io.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/delay.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

#define MODULE_NAME "kionix-kxsd9"

#define I2C_READ_RETRY_TIMES 10
#define I2C_WRITE_RETRY_TIMES 10

#define KXSD9_DEBUG  0
#define KXSD9_DUMP   0

#define KXSD9_FREE_FALL 0x800
#define KXSD9_WROP_BUF	30

#if KXSD9_DEBUG
 #define DLOG(fmt, arg...) printk(KERN_DEBUG MODULE_NAME ", %s: " fmt "\n", __FUNCTION__, ## arg);
#else
 #define DLOG(fmt, arg...) do {} while(0)
#endif

struct kxsd9 {
	struct i2c_client *client;
	struct input_dev *inputdev;
	struct hrtimer timer;
	struct delayed_work work;
	struct mutex lock;
#ifdef CONFIG_ANDROID_POWER
	android_suspend_lock_t suspend_lock;
#endif
	int on,scale,rate,susp;
	unsigned short pedo_count;
	int pedo_up,pedo_lim;

	unsigned short wrop[KXSD9_WROP_BUF];
	int head,tail;
};

#define KXSD9_REG_RST	0x0A
#define KXSD9_RST_KEY	0xCA
#define KXSD9_REG_A	0x0E	// --- --- --- --- --- --- Mhi ---
#define KXSD9_REG_B	0x0D	// ClH ENA ST  -0- -0- Men -0- -0-
#define KXSD9_REG_C	0x0C	// LP2 LP1 LP0 Mle Mla -0- FS1 FS0

#define KXSD9_B_CLKHLD	(1<<7)
#define KXSD9_B_ENABLE	(1<<6)
#define KXSD9_B_SLFTST	(1<<5)
#define KXSD9_C_MI_LEV	(1<<4)	// moti extra sensitive
#define KXSD9_C_MI_LAT	(1<<3)	// moti latched vs. non-latched
#define KXSD9_B_MI_ON	(1<<2)	// moti enabled
#define KXSD9_A_MI_INT	(1<<1)	// moti has happened (read clears)
#define KXSD9_BWIDTH	0xE0	// 111- =50Hz

enum {	/* operation     	   param */
	KXSD9_CTL_RESET,	// ignored
	KXSD9_CTL_ENABLE,	// 0 = disabled
	KXSD9_CTL_SCALE,	// 1 (2G) .. 4 (8G)
	KXSD9_CTL_RATE		// samples per 10 seconds
};

static int kxsd9_wrop_enq(struct kxsd9 *kxsd9,unsigned char reg,unsigned char val)
{
	int nt;

	nt = (kxsd9->tail + 1) % KXSD9_WROP_BUF;
	if (nt == kxsd9->head) {
		// buffer full
		return -1;
	}
	kxsd9->wrop[kxsd9->tail] = (reg << 8) | val;
	kxsd9->tail = nt;
	return 0;
}

static int kxsd9_wrop_deq(struct kxsd9 *kxsd9,char *buf)
{
	if (kxsd9->head == kxsd9->tail) {
		// buffer empty
		return -1;
	}
	buf[0] = kxsd9->wrop[kxsd9->head] >> 8;
	buf[1] = kxsd9->wrop[kxsd9->head] & 0xFF;
	kxsd9->head = (kxsd9->head + 1) % KXSD9_WROP_BUF;
	return 0;
}

int kxsd9_control(struct kxsd9 *kxsd9,int oper,int param)
{
	int restart;

	DLOG(": %s(%d, %d)\n", __func__, oper, param);
	mutex_lock(&kxsd9->lock);
	restart = (kxsd9->head == kxsd9->tail);
	switch (oper)
	{
		case KXSD9_CTL_RESET:
			kxsd9_wrop_enq(kxsd9,KXSD9_REG_RST,KXSD9_RST_KEY);
			kxsd9->pedo_up = kxsd9->pedo_lim = kxsd9->pedo_count = 0;
			break;

		case KXSD9_CTL_ENABLE:
			kxsd9->on = !!param;
			kxsd9_wrop_enq(kxsd9,KXSD9_REG_B,param ?
					KXSD9_B_CLKHLD | KXSD9_B_ENABLE : 0x00);
			break;

		case KXSD9_CTL_SCALE:
			if (param < 1)
				param = 1;
			else if (param > 4)
				param = 4;
			kxsd9->scale = param;
			param = 4 - param;
			kxsd9_wrop_enq(kxsd9,KXSD9_REG_C,KXSD9_BWIDTH | param);
			break;

		case KXSD9_CTL_RATE:
			param &= 0x1FFF;
			restart = (param > kxsd9->rate);
			kxsd9->rate = param;
			break;
	}
	if (restart) {
		hrtimer_start(&kxsd9->timer, ktime_set(0,16 * NSEC_PER_MSEC),
				HRTIMER_MODE_REL);
	}
	mutex_unlock(&kxsd9->lock);
	return 0;
}

static int kxsd9_i2c_write(struct i2c_client *client, const uint8_t *sendbuf, int len)
{
	int rc;
	int retry;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = len,
			.buf = sendbuf,
		},
	};

	for (retry = 0; retry <= I2C_WRITE_RETRY_TIMES; retry++) {
		rc = i2c_transfer(client->adapter, msg, 1);
		if (rc == 1)
			return 0;
		msleep(10);
		printk(KERN_WARNING "kxsd9, i2c write retry\n");
	}
	dev_err(&client->dev, "i2c_write_block retry over %d\n",
			I2C_WRITE_RETRY_TIMES);
	return rc;
}

static int kxsd9_i2c_read(struct i2c_client *client, uint8_t id,
						uint8_t *recvbuf, int len)
{
	int retry;
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &id,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = recvbuf,
		}
	};
	for (retry = 0; retry <= I2C_READ_RETRY_TIMES; retry++) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			return 0;
		msleep(10);
		printk(KERN_WARNING "kxsd9, i2c read retry\n");
	}
	dev_err(&client->dev, "i2c_read_block retry over %d\n",
			I2C_READ_RETRY_TIMES);
	return -EIO;
}

static enum hrtimer_restart kxsd9_poll_timer(struct hrtimer *timer)
{
	struct kxsd9 *kxsd9;

	kxsd9 = container_of(timer, struct kxsd9, timer);
#ifdef CONFIG_ANDROID_POWER
	android_lock_suspend(&kxsd9->suspend_lock);
#endif
	schedule_work(&kxsd9->work.work);
	return HRTIMER_NORESTART;
}

static void kxsd9_work(struct work_struct *work)
{
	struct kxsd9 *kxsd9;
	int err;
	uint8_t buf[6];
	int x,y,z;
	unsigned long long gabs;
	ktime_t restart_time = {0};

	kxsd9 = container_of(work, struct kxsd9, work.work);
	mutex_lock(&kxsd9->lock);
	if (kxsd9_wrop_deq(kxsd9,buf) == 0) {
		DLOG(": write %02x to %02x\n", buf[1], buf[0]);

		kxsd9_i2c_write(kxsd9->client, buf, 2);

		restart_time.tv.nsec = 4 * NSEC_PER_MSEC;
		hrtimer_start(&kxsd9->timer, restart_time, HRTIMER_MODE_REL);
	} else {
		kxsd9_i2c_read(kxsd9->client, 0, buf, 6);
		
		x = 0x800 - (buf[2] << 4) - (buf[3] >> 4);
		y = 0x800 - (buf[0] << 4) - (buf[1] >> 4);
		z = (buf[4] << 4) + (buf[5] >> 4) - 0x800 - 64; // calib?

		// detect step
		gabs = x * x + y * y + z * z;
		if (kxsd9->pedo_up) {
			if (gabs > kxsd9->pedo_lim) {
				kxsd9->pedo_up = 0;
				kxsd9->pedo_lim = gabs / 2;
				kxsd9->pedo_count++;
				input_report_abs(kxsd9->inputdev, ABS_GAS,
						kxsd9->pedo_count);
			} else if (kxsd9->pedo_lim > gabs * 2) {
				kxsd9->pedo_lim = gabs * 2;
			}
		} else {
			if (gabs < kxsd9->pedo_lim) {
				kxsd9->pedo_up = 1;
				kxsd9->pedo_lim = gabs * 2;
			} else if (kxsd9->pedo_lim < gabs / 2) {
				kxsd9->pedo_lim = gabs / 2;
			}
		}
#if KXSD9_DUMP
#if 1
		printk(KERN_INFO "G=(%6d, %6d, %6d) P=%d %s\n",
				x, y, z, kxsd9->pedo_count,
				gabs < KXSD9_FREE_FALL ? "FF" : ""); // free-fall
#else
		printk(KERN_INFO "G=( %02X %02X  %02X %02X  %02X %02X )\n",
				buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
#endif
#endif
		input_report_abs(kxsd9->inputdev, ABS_X, x);
		input_report_abs(kxsd9->inputdev, ABS_Y, y);
		input_report_abs(kxsd9->inputdev, ABS_Z, z);
		input_sync(kxsd9->inputdev);

		if (kxsd9->on && kxsd9->rate && !kxsd9->susp)
		{
			restart_time.tv.nsec = (10000 / kxsd9->rate)
					* NSEC_PER_MSEC;
			hrtimer_start(&kxsd9->timer, restart_time,
					HRTIMER_MODE_REL);
		}

	}
#ifdef CONFIG_ANDROID_POWER
	android_unlock_suspend(&kxsd9->suspend_lock);
#endif
	mutex_unlock(&kxsd9->lock);
}

static ssize_t kxsd9_ctl_rate_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct kxsd9 *kxsd9 = i2c_get_clientdata(to_i2c_client(dev));

	DLOG(" %s\n", __func__);
	return sprintf(buf, "%u\n", kxsd9 ? kxsd9->rate : 0);
}

static ssize_t kxsd9_ctl_rate_store(struct device *dev, struct device_attribute *attr,
				const char *buf,size_t count)
{
	struct kxsd9 *kxsd9 = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long val = simple_strtoul(buf, NULL, 10);

	DLOG(" %s\n", __func__);
	kxsd9_control(kxsd9,KXSD9_CTL_RATE,val);
        return count;
}

static ssize_t kxsd9_ctl_scale_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct kxsd9 *kxsd9 = i2c_get_clientdata(to_i2c_client(dev));

	DLOG(" %s\n", __func__);
	return sprintf(buf, "%u\n", kxsd9 ? kxsd9->scale : 0);
}

static ssize_t kxsd9_ctl_scale_store(struct device *dev, struct device_attribute *attr,
				const char *buf,size_t count)
{
	struct kxsd9 *kxsd9 = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long val = simple_strtoul(buf, NULL, 10);

	DLOG(" %s\n", __func__);
	kxsd9_control(kxsd9,KXSD9_CTL_SCALE,val);
        return count;
}

static ssize_t kxsd9_ctl_enable_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct kxsd9 *kxsd9 = i2c_get_clientdata(to_i2c_client(dev));

	DLOG(" %s\n", __func__);
	return sprintf(buf, "%u\n", kxsd9 && kxsd9->on ? 1 : 0);
}

static ssize_t kxsd9_ctl_enable_store(struct device *dev, struct device_attribute *attr,
				const char *buf,size_t count)
{
	struct kxsd9 *kxsd9 = i2c_get_clientdata(to_i2c_client(dev));
	unsigned long val = simple_strtoul(buf, NULL, 10);

	DLOG(" %s\n", __func__);
	kxsd9_control(kxsd9,KXSD9_CTL_ENABLE,!!val);
        return count;
}

struct device_attribute kxsd9_sysfs_ctl_rate =
{
	.attr = {	.name = "rate",
			.mode = S_IWUSR | S_IRUGO },
	.show	= kxsd9_ctl_rate_show,
	.store	= kxsd9_ctl_rate_store,
};

struct device_attribute kxsd9_sysfs_ctl_scale =
{
	.attr = {	.name = "scale",
			.mode = S_IWUSR | S_IRUGO },
	.show	= kxsd9_ctl_scale_show,
	.store	= kxsd9_ctl_scale_store,
};

struct device_attribute kxsd9_sysfs_ctl_enable =
{
	.attr = {	.name = "enable",
			.mode = S_IWUSR | S_IRUGO },
	.show	= kxsd9_ctl_enable_show,
	.store	= kxsd9_ctl_enable_store,
};

static int kxsd9_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct kxsd9 *kxsd9;
	struct input_dev *idev;

	printk(KERN_INFO MODULE_NAME ": Initializing Kionix KXSD9 driver "
					"at addr: 0x%02x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_ERR MODULE_NAME ": i2c bus not supported\n");
		return -EINVAL;
	}

	kxsd9 = kzalloc(sizeof *kxsd9, GFP_KERNEL);
	if (kxsd9 < 0) {
		printk(KERN_ERR MODULE_NAME ": Not enough memory\n");
		return -ENOMEM;
	}
	mutex_init(&kxsd9->lock);
	kxsd9->client = client;
	i2c_set_clientdata(client, kxsd9);

	idev = input_allocate_device();
	if (idev) {
		idev->name = MODULE_NAME;
		idev->phys=kzalloc(12, GFP_KERNEL);
		snprintf(idev->phys, 11, "i2c/0-%04x", client->addr);
		set_bit(EV_ABS, idev->evbit);
		input_set_abs_params(idev, ABS_X, -2048, 2047, 0, 0);
		input_set_abs_params(idev, ABS_Y, -2048, 2047, 0, 0);
		input_set_abs_params(idev, ABS_Z, -2048, 2047, 0, 0);
		input_set_abs_params(idev, ABS_GAS, 0, 65535, 0, 0);
		if (!input_register_device(idev)) {
			kxsd9->inputdev = idev;
		} else {
			kxsd9->inputdev = 0;
			printk(KERN_ERR MODULE_NAME
					": Failed to register input device\n");
		}
	}
	if (device_create_file(&client->dev, &kxsd9_sysfs_ctl_enable) != 0)
		printk(KERN_ERR MODULE_NAME ": Failed to create 'enable' file\n");
	if (device_create_file(&client->dev, &kxsd9_sysfs_ctl_scale) != 0)
		printk(KERN_ERR MODULE_NAME ": Failed to create 'scale' file\n");
	if (device_create_file(&client->dev, &kxsd9_sysfs_ctl_rate) != 0)
		printk(KERN_ERR MODULE_NAME ": Failed to create 'rate' file\n");
#ifdef CONFIG_ANDROID_POWER
	kxsd9->suspend_lock.name = MODULE_NAME;
	android_init_suspend_lock(&kxsd9->suspend_lock);
	android_lock_suspend(&kxsd9->suspend_lock);
#endif
	INIT_DELAYED_WORK(&kxsd9->work, kxsd9_work);
	hrtimer_init(&kxsd9->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	kxsd9->timer.function = kxsd9_poll_timer;
	kxsd9_control(kxsd9,KXSD9_CTL_RESET,0);
	kxsd9_control(kxsd9,KXSD9_CTL_SCALE,1);
	kxsd9_control(kxsd9,KXSD9_CTL_RATE,100);
#if KXSD9_DEBUG
	kxsd9_control(kxsd9,KXSD9_CTL_ENABLE,1);
#else
	kxsd9_control(kxsd9,KXSD9_CTL_ENABLE,0);
#endif
	return 0;
}

static int kxsd9_remove(struct i2c_client * client)
{
	struct kxsd9 *kxsd9 = i2c_get_clientdata(client);

	input_unregister_device(kxsd9->inputdev);
	input_free_device(kxsd9->inputdev);
#ifdef CONFIG_ANDROID_POWER
	android_uninit_suspend_lock(&kxsd9->suspend_lock);
#endif
	kfree(kxsd9);
	return 0;
}

#if CONFIG_PM
static int kxsd9_suspend(struct i2c_client * client, pm_message_t mesg)
{
	struct kxsd9 *kxsd9 = i2c_get_clientdata(client);

	DLOG(": suspending device...\n");
	kxsd9->susp = 1;
	if (kxsd9->on) {
		kxsd9_control(kxsd9,KXSD9_CTL_ENABLE,0);
		kxsd9->on = 1;
	}
	return 0;
}

static int kxsd9_resume(struct i2c_client * client)
{
	struct kxsd9 *kxsd9 = i2c_get_clientdata(client);

	DLOG(": resuming device...\n");
	kxsd9->susp = 0;
	if (kxsd9->on)
		kxsd9_control(kxsd9,KXSD9_CTL_ENABLE,1);
	return 0;
}
#else
#define kxsd9_suspend NULL
#define kxsd9_resume NULL
#endif

static const struct i2c_device_id kxsd9_ids[] = {
        { MODULE_NAME, 0 },
        { }
};

static struct i2c_driver kxsd9_driver = {
	.driver = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
	},
	.id_table = kxsd9_ids,
	.probe = kxsd9_probe,
	.remove = kxsd9_remove,
#if CONFIG_PM
	.suspend = kxsd9_suspend,
	.resume = kxsd9_resume,
#endif
};

static int __init kxsd9_init(void)
{
	printk(KERN_INFO MODULE_NAME ": Registering Kionix KXSD9 driver\n");
	return i2c_add_driver(&kxsd9_driver);
}

static void __exit kxsd9_exit(void)
{
	printk(KERN_INFO MODULE_NAME ": Unregistered Kionix KXSD9 driver\n");
	i2c_del_driver(&kxsd9_driver);
}

MODULE_AUTHOR("Job Bolle");
MODULE_DESCRIPTION("Kionix KXSD9 driver");
MODULE_LICENSE("GPL");

module_init(kxsd9_init);
module_exit(kxsd9_exit);
