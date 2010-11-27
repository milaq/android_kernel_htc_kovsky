/* bma150.c
 *
 * G-Sensor found in HTC Topaz (Touch Pro) and HTC Rhodium mobile phones
 *
*/
#include <asm/mach-types.h>
#include <asm/io.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif
#include <linux/bma150.h>
#include <linux/microp.h>
#include <linux/microp-klt.h>

#define MODULE_NAME "bma150"

#define BMA150_DEBUG  0
#define BMA150_DUMP   0

#define BMA150_FREE_FALL 0x800
#define EVENT_TYPE_TEMPERATURE      ABS_THROTTLE

#define I2C_READ_RETRY_TIMES  10
#define I2C_WRITE_RETRY_TIMES 10

static int gsensor_read_reg(uint8_t reg, uint8_t *data);
static int gsensor_write_reg(uint8_t reg, uint8_t data);
static int microp_spi_enable(uint8_t on);
static int bma150_i2c_read(struct i2c_client *client, uint8_t addr, uint8_t *data, int len);
static int bma150_i2c_write(struct i2c_client *client, uint8_t addr, uint8_t *data, int len);

enum {	/* operation     	   param */
	BMA150_CTL_RESET,	// ignored	
	BMA150_CTL_ENABLE,	// 0 = disabled
	BMA150_CTL_SCALE,	// 1 (2G) .. 4 (8G)
	BMA150_CTL_RATE		// samples per 10 seconds
};

static struct bma_t {
	struct i2c_client *client;
	struct microp_klt *micropklt_t;
	//	For BMA150
	struct input_dev *inputdev;
	struct hrtimer timer;
	struct delayed_work work;
#ifdef CONFIG_ANDROID_POWER
	android_suspend_lock_t suspend_lock;
#endif
	int on,scale,rate,susp;
	unsigned short pedo_count;
	int pedo_up,pedo_lim;

	unsigned short wrop[BMA150_WROP_BUF];
	int head,tail;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
	uint8_t enable_early_suspend;
#endif
} *_bma = 0;

/*
 * Comes from mahimahi's microp driver.
 * This is OMFG ugly.
 */
static char *hex2string(uint8_t *data, int len)
{
	static char buf[101];
	int i;

	i = (sizeof(buf) - 1) / 4;
	if (len > i)
		len = i;

	for (i = 0; i < len; i++)
		sprintf(buf + i * 4, "[%02X]", data[i]);

	return buf;
}

static int bma150_wrop_enq(unsigned char reg,unsigned char val)
{
	int nt;

	nt = (_bma->tail + 1) % BMA150_WROP_BUF;
	if (nt == _bma->head) {
		// buffer full
		return -1;
	}
	_bma->wrop[_bma->tail] = (reg << 8) | val;
	_bma->tail = nt;
	return 0;
}

static int bma150_wrop_deq(struct bma_t *bma150_w, char *buf)
{
	if (bma150_w->head == bma150_w->tail) {
		// buffer empty
		return -1;
	}
	buf[0] = bma150_w->wrop[bma150_w->head] >> 8;
	buf[1] = bma150_w->wrop[bma150_w->head] & 0xFF;
	bma150_w->head = (bma150_w->head + 1) % BMA150_WROP_BUF;
	return 0;
}

static int gsensor_init_hw(void)
{
	uint8_t reg;
	int ret;

	pr_debug("%s\n", __func__);

	microp_spi_enable(1);

	ret = gsensor_read_reg(RANGE_BWIDTH_REG, &reg);
	if (ret < 0 )
		return -EIO;
	reg &= 0xe0;	// 25 Hz ,  2G
	ret = gsensor_write_reg(RANGE_BWIDTH_REG, reg);
	if (ret < 0 )
		return -EIO;

	ret = gsensor_read_reg(SMB150_CONF2_REG, &reg);
	if (ret < 0 )
		return -EIO;
	reg |= (1 << 3);
	ret = gsensor_write_reg(SMB150_CONF2_REG, reg);

	return ret;
}

static int bma150_set_mode(char mode)
{
	uint8_t reg;
	int ret;

	if (mode == BMA_MODE_NORMAL)
		microp_spi_enable(1);


	ret = gsensor_read_reg(SMB150_CTRL_REG, &reg);
	if (ret < 0 )
		return -EIO;
	reg = (reg & 0xfe) | mode;
	ret = gsensor_write_reg(SMB150_CTRL_REG, reg);

	if (mode == BMA_MODE_SLEEP)
		microp_spi_enable(0);

	return ret;
}

static int bma150_set_bandwidth(char bw)
{
	uint8_t reg;
	int ret;

	if(bw<0 || bw > BMA_BW_1500HZ)
		return -1;

	ret = gsensor_read_reg(RANGE_BWIDTH_REG, &reg);
	if (ret < 0 )
		return -EIO;
	reg = (reg & 0xf8) | bw;
	ret = gsensor_write_reg(RANGE_BWIDTH_REG, reg);

	return ret;
}

static int bma150_set_scale(char scale)
{
	uint8_t reg;
	int ret;

	if(scale<0 || scale > BMA_RANGE_8G)
		return -1;

	ret = gsensor_read_reg(RANGE_BWIDTH_REG, &reg);
	if (ret < 0 )
		return -EIO;
	reg = (reg & 0xe7) | (scale << 3);
	ret = gsensor_write_reg(RANGE_BWIDTH_REG, reg);

	return ret;
}

static int bma150_read_temp()
{
	uint8_t reg;
	int ret;

	ret = gsensor_read_reg(TEMP_RD_REG, &reg);
	if (ret < 0 )
		return -EIO;

	return reg;
}

static int gsensor_read_acceleration(short *buf)
{
	struct i2c_client *client;
	int ret;
	uint8_t tmp[6]={0,0,0,0,0,0};

	//BMA on microp
	if(!_bma->client) {
		client = _bma->micropklt_t->client;

		tmp[0] = 1;
		ret = bma150_i2c_write(client, MICROP_I2C_WCMD_GSENSOR_DATA_REQ,
				      tmp, 1);
		if (ret < 0) {
			dev_err(&client->dev,"%s: i2c_write_block fail\n", __func__);
			return ret;
		}

		msleep(10);

		if (_bma->micropklt_t->version <= 0x615 || _bma->micropklt_t->version == 0x0a0e || _bma->micropklt_t->version == 0x0b0e) {
			/*
			 * Note the data is a 10bit signed value from the chip.
			*/
			ret = bma150_i2c_read(client, MICROP_I2C_RCMD_GSENSOR_X_DATA,
					     tmp, 2);
			if (ret < 0) {
				dev_err(&client->dev, "%s: i2c_read_block fail\n",
					__func__);
				return ret;
			}
			buf[0] = (short)(tmp[0] << 8 | tmp[1]);
			buf[0] >>= 6;

			ret = bma150_i2c_read(client, MICROP_I2C_RCMD_GSENSOR_Y_DATA,
					     tmp, 2);
			if (ret < 0) {
				dev_err(&client->dev, "%s: i2c_read_block fail\n",
					__func__);
				return ret;
			}
			buf[1] = (short)(tmp[0] << 8 | tmp[1]);
			buf[1] >>= 6;

			ret = bma150_i2c_read(client, MICROP_I2C_RCMD_GSENSOR_Z_DATA,
					     tmp, 2);
			if (ret < 0) {
				dev_err(&client->dev, "%s: i2c_read_block fail\n",
					__func__);
				return ret;
			}
			buf[2] = (short)(tmp[0] << 8 | tmp[1]);
			buf[2] >>= 6;
		} else {
			ret = bma150_i2c_read(client, MICROP_I2C_RCMD_GSENSOR_DATA,
					     tmp, 6);
			if (ret < 0) {
				dev_err(&client->dev, "%s: i2c_read_block fail\n",
					__func__);
				return ret;
			}
			buf[0] = (short)(tmp[0] << 8 | tmp[1]);
			buf[0] >>= 6;
			buf[1] = (short)(tmp[2] << 8 | tmp[3]);
			buf[1] >>= 6;
			buf[2] = (short)(tmp[4] << 8 | tmp[5]);
			buf[2] >>= 6;
		}
	} else {
		//bma150 on I2C
		client = _bma->client;
		ret = bma150_i2c_read(client, X_AXIS_LSB_REG, tmp, 6);
		if (ret < 0) {
			dev_err(&client->dev, "%s: i2c_read_block fail\n",
				__func__);
			return ret;
		}
		/*
		buf[0] = (short)(tmp[0] << 8 | tmp[1]);
		buf[0] >>= 6;
		buf[1] = (short)(tmp[2] << 8 | tmp[3]);
		buf[1] >>= 6;
		buf[2] = (short)(tmp[4] << 8 | tmp[5]);
		buf[2] >>= 6;*/
		buf[0] = tmp[1]<<2|tmp[0]>>6;
		if (buf[0]&0x200)
			buf[0] -= 1<<10;
		buf[1] = tmp[3]<<2|tmp[2]>>6;
		if (buf[1]&0x200)
			buf[1] -= 1<<10;
		buf[2] = tmp[5]<<2|tmp[4]>>6;
		if (buf[2]&0x200)
			buf[2] -= 1<<10;

	}

#ifdef DEBUG_BMA150
	/* Log this to debugfs */
	gsensor_log_status(ktime_get(), buf[0], buf[1], buf[2]);
#endif
	return 1;
}

int bma150_control(int oper,int param)
{
	int restart;
	
#if BMA150_DEBUG
	printk(KERN_INFO MODULE_NAME ": %s(%d, %d)\n", __func__, oper, param);
#endif
	if(_bma->micropklt_t)
		mutex_lock(&_bma->micropklt_t->lock);
	//TODO: lock for i2c ?
	restart = (_bma->head == _bma->tail);
	switch (oper)
	{
		case BMA150_CTL_ENABLE:
			_bma->on = !!param;
			bma150_wrop_enq(BMA150_ACT_ENABLE, param ? BMA_MODE_NORMAL : BMA_MODE_SLEEP);
			break;
			
		case BMA150_CTL_SCALE:
			if (param < 1)
				param = 1;
			else if (param > 3)
				param = 3;
			_bma->scale = param;
			param = param -1;
			bma150_wrop_enq(BMA150_ACT_SCALE, param);
			bma150_wrop_enq(BMA150_ACT_BW, BMA_BW_50HZ);
			break;
		
		case BMA150_CTL_RATE:
			param &= 0x1FFF;
			restart = (param > _bma->rate);
			_bma->rate = param;
			break;
	}
	if (restart) {
		hrtimer_start(&_bma->timer, ktime_set(0,16 * NSEC_PER_MSEC), 
				HRTIMER_MODE_REL);
	}
	if(_bma->micropklt_t)
		mutex_unlock(&_bma->micropklt_t->lock);
	return 0;
}

static int bma150_i2c_read(struct i2c_client *client, uint8_t addr,
						uint8_t *data, int len)
{
	int retry;
	int rc;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		}
	};

	for (retry = 0; retry <= I2C_READ_RETRY_TIMES; retry++) {
		rc = i2c_transfer(client->adapter, msgs, 2);
		if (rc == 2) {
#if BMA150_DEBUG
			dev_dbg(&client->dev, "R [%02X] = %s\n", addr,
					hex2string(data, len));
#endif
			return 0;
		}
		msleep(10);
		printk(KERN_WARNING "bma150, i2c read retry\n");
	}

	dev_err(&client->dev, "i2c_read_block retry over %d\n",
			I2C_READ_RETRY_TIMES);
	return -EIO;
}

#define MICROP_I2C_WRITE_BLOCK_SIZE 21
static int bma150_i2c_write(struct i2c_client *client, uint8_t addr,
						uint8_t *data, int len)
{
	int retry;
	int rc;
	uint8_t buf[MICROP_I2C_WRITE_BLOCK_SIZE];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = len + 1,
			.buf = buf,
		}
	};

	dev_dbg(&client->dev, "W [%02X] = %s\n", addr,
			hex2string(data, len));

	if (len + 1 > MICROP_I2C_WRITE_BLOCK_SIZE) {
		dev_err(&client->dev, "i2c_write_block length too long\n");
		return -E2BIG;
	}

	buf[0] = addr;
	memcpy((void *)&buf[1], (void *)data, len);

	for (retry = 0; retry <= I2C_WRITE_RETRY_TIMES; retry++) {
		rc = i2c_transfer(client->adapter, msg, 1);
		if (rc == 1)
			return 0;
		msleep(10);
		printk(KERN_WARNING "bma150, i2c write retry\n");
	}
	dev_err(&client->dev, "i2c_write_block retry over %d\n",
			I2C_WRITE_RETRY_TIMES);
	return -EIO;
}

static int microp_spi_enable(uint8_t on)
{
	struct i2c_client *client;
	int ret;

	if(!_bma->micropklt_t)
		return 0;
	client = _bma->micropklt_t->client;;
	ret = bma150_i2c_write(client, MICROP_I2C_WCMD_SPI_EN, &on, 1);
	if (ret < 0) {
		dev_err(&client->dev,"%s: i2c_write_block fail\n", __func__);
		return ret;
	}
	msleep(10);
	return ret;
}

static int gsensor_read_reg(uint8_t reg, uint8_t *data)
{
	struct i2c_client *client;
	int ret;
	uint8_t tmp[2];

	if(_bma->micropklt_t) {
		client = _bma->micropklt_t->client;
		ret = bma150_i2c_write(client, MICROP_I2C_WCMD_GSENSOR_REG_DATA_REQ,
				      &reg, 1);
		if (ret < 0) {
			dev_err(&client->dev,"%s: i2c_write_block fail\n", __func__);
			return ret;
		}
		msleep(10);

		ret = bma150_i2c_read(client, MICROP_I2C_WCMD_GSENSOR_REG_DATA_REQ, tmp, 2);
		if (ret < 0) {
			dev_err(&client->dev,"%s: i2c_read_block fail\n", __func__);
			return ret;
		}
		*data = tmp[1];
	} else {
		client=_bma->client;
		ret = bma150_i2c_read(client, reg, tmp, 1);
		if (ret < 0) {
			dev_err(&client->dev,"%s: i2c_read_block fail\n", __func__);
			return ret;
		}
		*data = tmp[0];
	}
	return ret;
}

static int gsensor_write_reg(uint8_t reg, uint8_t data)
{
	struct i2c_client *client;
	int ret;
	uint8_t tmp[2];

	if(_bma->micropklt_t) {
		client = _bma->micropklt_t->client;

		tmp[0] = reg;
		tmp[1] = data;
		ret = bma150_i2c_write(client, MICROP_I2C_WCMD_GSENSOR_REG, tmp, 2);
		if (ret < 0) {
			dev_err(&client->dev,"%s: i2c_write_block fail\n", __func__);
			return ret;
		}
	} else {
		client = _bma->client;

		tmp[0] = data;
		ret = bma150_i2c_write(client, reg, tmp, 1);
		if (ret < 0) {
			dev_err(&client->dev,"%s: i2c_write_block fail\n", __func__);
			return ret;
		}
	}

	return ret;
}

static enum hrtimer_restart bma150_poll_timer(struct hrtimer *timer)
{

	struct bma_t *bma150_w;
	bma150_w = container_of(timer, struct bma_t, timer);
#ifdef CONFIG_ANDROID_POWER
	android_lock_suspend(&bma150_w->suspend_lock);
#endif
	schedule_work(&bma150_w->work.work);
	return HRTIMER_NORESTART;
}

static void bma150_work(struct work_struct *work)
{
	int err;
	char buf[3];
	short vals[3];
	int x,y,z, temp;
	unsigned short action;
	unsigned long long gabs;
	ktime_t restart_time = {0};
	struct bma_t *bma150_w;

	bma150_w = container_of(work, struct bma_t, work.work);
	if(bma150_w->micropklt_t)
		mutex_lock(&bma150_w->micropklt_t->lock);
	if (bma150_wrop_deq(bma150_w, buf) == 0) {
#if BMA150_DEBUG
		printk(KERN_INFO MODULE_NAME ": write %02x ACTION %02x\n",
				buf[1], buf[0]);
#endif
		action = buf[0];
		switch(action) {
			  case BMA150_ACT_ENABLE:
			  err = bma150_set_mode(buf[1]);
			  break;
			  case BMA150_ACT_SCALE:
			  err = bma150_set_scale(buf[1]);
			  break;
			  case BMA150_ACT_BW:
			  err = bma150_set_bandwidth(buf[1]);
			  break;
			  default:
			  err = gsensor_write_reg(buf[0], buf[1]);
			  break;
		}
	      	if (err < 0) {
			printk(KERN_WARNING MODULE_NAME 
					": %s: error %d\n", __func__, err);
		}
		restart_time.tv.nsec = 4 * NSEC_PER_MSEC;
		hrtimer_start(&bma150_w->timer, restart_time, HRTIMER_MODE_REL);
	} else {
		vals[0] = vals[1] = vals[2] = 0;
		err = gsensor_read_acceleration(vals);
		x = vals[0];
		y = vals[1];
		z = vals[2];
		temp = bma150_read_temp();
		// detect step
		gabs = x * x + y * y + z * z;
		if (bma150_w->pedo_up) {
			if (gabs > bma150_w->pedo_lim) {
				bma150_w->pedo_up = 0;
				bma150_w->pedo_lim = gabs / 2;
				bma150_w->pedo_count++;
				input_report_abs(bma150_w->inputdev, ABS_GAS, 
						bma150_w->pedo_count);
			} else if (bma150_w->pedo_lim > gabs * 2) {
				bma150_w->pedo_lim = gabs * 2;
			}
		} else {
			if (gabs < bma150_w->pedo_lim) {
				bma150_w->pedo_up = 1;
				bma150_w->pedo_lim = gabs * 2;
			} else if (bma150_w->pedo_lim < gabs / 2) {
				bma150_w->pedo_lim = gabs / 2;
			}
		}
#if BMA150_DUMP
#if 1
		printk(KERN_INFO "G=(%6d, %6d, %6d) P=%d %s\n",
				x, y, z, bma150_w->pedo_count,
				gabs < BMA150_FREE_FALL ? "FF" : ""); // free-fall
#else
		printk(KERN_INFO "G=( %02X %02X %02X)\n", 
				vals[0],vals[1],vals[2]);
#endif
#endif
		input_report_abs(bma150_w->inputdev, ABS_X, x);
		input_report_abs(bma150_w->inputdev, ABS_Y, y);
		input_report_abs(bma150_w->inputdev, ABS_Z, z);
		input_report_abs(bma150_w->inputdev, EVENT_TYPE_TEMPERATURE, temp);
		input_sync(bma150_w->inputdev);
		
		if (bma150_w->on && bma150_w->rate && !bma150_w->susp)
		{
			restart_time.tv.nsec = (10000 / bma150_w->rate)
					* NSEC_PER_MSEC;
			hrtimer_start(&bma150_w->timer, restart_time,
					HRTIMER_MODE_REL);
		}

	}
#ifdef CONFIG_ANDROID_POWER
	android_unlock_suspend(&bma150_w->suspend_lock);
#endif
	if(bma150_w->micropklt_t)
		mutex_unlock(&bma150_w->micropklt_t->lock);
}

static ssize_t bma150_ctl_rate_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
#if BMA150_DEBUG
	printk(KERN_INFO MODULE_NAME " %s\n", __func__);
#endif
	return sprintf(buf, "%u\n", _bma ? _bma->rate : 0);
}

static ssize_t bma150_ctl_rate_store(struct device *dev, struct device_attribute *attr,
				const char *buf,size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 10);

#if BMA150_DEBUG
	printk(KERN_INFO MODULE_NAME " %s\n", __func__);
#endif
	bma150_control(BMA150_CTL_RATE,val);
        return count;
}

static ssize_t bma150_ctl_scale_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
#if BMA150_DEBUG
	printk(KERN_INFO MODULE_NAME " %s\n", __func__);
#endif
	return sprintf(buf, "%u\n", _bma ? _bma->scale : 0);
}

static ssize_t bma150_ctl_scale_store(struct device *dev, struct device_attribute *attr,
				const char *buf,size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 10);

#if BMA150_DEBUG
	printk(KERN_INFO MODULE_NAME " %s\n", __func__);
#endif
	bma150_control(BMA150_CTL_SCALE,val);
        return count;
}

static ssize_t bma150_ctl_enable_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{

#if BMA150_DEBUG
	printk(KERN_INFO MODULE_NAME " %s\n", __func__);
#endif
	return sprintf(buf, "%u\n", _bma && _bma->on ? 1 : 0);
}

static ssize_t bma150_ctl_enable_store(struct device *dev, struct device_attribute *attr,
				const char *buf,size_t count)
{
	unsigned long val = simple_strtoul(buf, NULL, 10);

#if BMA150_DEBUG
	printk(KERN_INFO MODULE_NAME " %s\n", __func__);
#endif
	bma150_control(BMA150_CTL_ENABLE,!!val);
        return count;
}

struct device_attribute bma150_sysfs_ctl_rate = 
{
	.attr = {	.name = "rate",
			.mode = S_IWUGO | S_IRUGO },
	.show	= bma150_ctl_rate_show,
	.store	= bma150_ctl_rate_store,
};

struct device_attribute bma150_sysfs_ctl_scale = 
{
	.attr = {	.name = "scale",
			.mode = S_IWUGO | S_IRUGO },
	.show	= bma150_ctl_scale_show,
	.store	= bma150_ctl_scale_store,
};

struct device_attribute bma150_sysfs_ctl_enable = 
{
	.attr = {	.name = "enable",
			.mode = S_IWUGO | S_IRUGO },
	.show	= bma150_ctl_enable_show,
	.store	= bma150_ctl_enable_store,
};

int bma150_probe(struct microp_klt* data)
{
	struct input_dev *idev;
	_bma=kzalloc(sizeof(*_bma), GFP_KERNEL);
	_bma->micropklt_t = data;
	if(_bma->micropklt_t==0) {
		pr_err("Microp KLT driver has to be initialized first!\n");
		return -1;
	}

	printk(KERN_ERR MODULE_NAME ": Initializing BMA150 over microp driver "
					"at addr: 0x%02x\n", _bma->micropklt_t->client->addr);

	idev = input_allocate_device();
	if (idev) {
		idev->name = MODULE_NAME;
		idev->phys=kzalloc(12, GFP_KERNEL);
		snprintf((char*)idev->phys, 11, "i2c/0-%04x", _bma->micropklt_t->client->addr);
		set_bit(EV_ABS, idev->evbit);
		input_set_abs_params(idev, ABS_X, -2048, 2047, 0, 0);
		input_set_abs_params(idev, ABS_Y, -2048, 2047, 0, 0);
		input_set_abs_params(idev, ABS_Z, -2048, 2047, 0, 0);
		input_set_abs_params(idev, EVENT_TYPE_TEMPERATURE, 0, 256, 0, 0);
		input_set_abs_params(idev, ABS_GAS, 0, 65535, 0, 0);
		if (!input_register_device(idev)) {
			_bma->inputdev = idev;			
		} else {
			_bma->inputdev = 0;
			printk(KERN_ERR MODULE_NAME 
					": Failed to register input device\n");
		}
	}
	if (device_create_file(&_bma->micropklt_t->client->dev, &bma150_sysfs_ctl_enable) != 0)
		printk(KERN_ERR MODULE_NAME ": Failed to create 'enable' file\n");
	if (device_create_file(&_bma->micropklt_t->client->dev, &bma150_sysfs_ctl_scale) != 0)
		printk(KERN_ERR MODULE_NAME ": Failed to create 'scale' file\n");
	if (device_create_file(&_bma->micropklt_t->client->dev, &bma150_sysfs_ctl_rate) != 0)
		printk(KERN_ERR MODULE_NAME ": Failed to create 'rate' file\n");
#ifdef CONFIG_ANDROID_POWER
	_bma->suspend_lock.name = MODULE_NAME;
	android_init_suspend_lock(&_bma->suspend_lock);
	android_lock_suspend(&_bma->suspend_lock);
#endif
	gsensor_init_hw();

	INIT_DELAYED_WORK(&_bma->work, bma150_work);
	hrtimer_init(&_bma->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	_bma->timer.function = bma150_poll_timer;
	
	return 0;
}
EXPORT_SYMBOL(bma150_probe);

int bma150_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	struct input_dev *idev;
	_bma=kzalloc(sizeof(*_bma), GFP_KERNEL);
	_bma->micropklt_t = 0;
	_bma->client=client;
	
	printk(KERN_ERR MODULE_NAME ": Initializing BMA150 over i2c driver "
					"at addr: 0x%02x\n", client->addr);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_ERR MODULE_NAME ": i2c bus not supported\n");
		return -EINVAL;
	}

	idev = input_allocate_device();
	if (idev) {
		idev->name = MODULE_NAME;
		idev->phys=kzalloc(12, GFP_KERNEL);
		snprintf((char*)idev->phys, 11, "i2c/0-%04x", _bma->client->addr);
		set_bit(EV_ABS, idev->evbit);
		input_set_abs_params(idev, ABS_X, -2048, 2047, 0, 0);
		input_set_abs_params(idev, ABS_Y, -2048, 2047, 0, 0);
		input_set_abs_params(idev, ABS_Z, -2048, 2047, 0, 0);
		input_set_abs_params(idev, EVENT_TYPE_TEMPERATURE, 0, 256, 0, 0);
		input_set_abs_params(idev, ABS_GAS, 0, 65535, 0, 0);
		if (!input_register_device(idev)) {
			_bma->inputdev = idev;			
		} else {
			_bma->inputdev = 0;
			printk(KERN_ERR MODULE_NAME 
					": Failed to register input device\n");
		}
	}
	if (device_create_file(&client->dev, &bma150_sysfs_ctl_enable) != 0)
		printk(KERN_ERR MODULE_NAME ": Failed to create 'enable' file\n");
	if (device_create_file(&client->dev, &bma150_sysfs_ctl_scale) != 0)
		printk(KERN_ERR MODULE_NAME ": Failed to create 'scale' file\n");
	if (device_create_file(&client->dev, &bma150_sysfs_ctl_rate) != 0)
		printk(KERN_ERR MODULE_NAME ": Failed to create 'rate' file\n");
#ifdef CONFIG_ANDROID_POWER
	_bma->suspend_lock.name = MODULE_NAME;
	android_init_suspend_lock(&_bma->suspend_lock);
	android_lock_suspend(&_bma->suspend_lock);
#endif
	gsensor_init_hw();

	INIT_DELAYED_WORK(&_bma->work, bma150_work);
	hrtimer_init(&_bma->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	_bma->timer.function = bma150_poll_timer;
	
	return 0;
}

static int bma150_remove(struct i2c_client * client)
{
	input_unregister_device(_bma->inputdev);
	input_free_device(_bma->inputdev);
#ifdef CONFIG_ANDROID_POWER
	android_uninit_suspend_lock(&_bma->suspend_lock);
#endif
	kfree(_bma);
	return 0;
}

#if CONFIG_PM
static int bma150_suspend(struct i2c_client * client, pm_message_t mesg)
{
#if BMA150_DEBUG
	printk(KERN_INFO MODULE_NAME ": suspending device...\n");
#endif
	_bma->susp = 1;
	if (_bma->on) {
		bma150_control(BMA150_CTL_ENABLE,0);
		_bma->on = 1;
	}
	return 0;
}

static int bma150_resume(struct i2c_client * client)
{
#if BMA150_DEBUG
	printk(KERN_INFO MODULE_NAME ": resuming device...\n");
#endif
	_bma->susp = 0;
	if (_bma->on)
		bma150_control(BMA150_CTL_ENABLE,1);
	return 0;
}
#else
#define bma150_suspend NULL
#define bma150_resume NULL
#endif

static const struct i2c_device_id bma150_ids[] = {
        { MODULE_NAME, 0 },
        { }
};

static struct i2c_driver bma150_driver = {
	.driver = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
	},
	.id_table = bma150_ids,
	.probe = bma150_i2c_probe,
	.remove = bma150_remove,
#if CONFIG_PM
	.suspend = bma150_suspend,
	.resume = bma150_resume,
#endif
};

int __init bma150_init(void)
{
	int ret;
	printk(KERN_INFO MODULE_NAME ": Registering Bosch BMA150 driver\n");
	ret = i2c_add_driver(&bma150_driver);
	return ret;
}

void __exit bma150_exit(void)
{
	printk(KERN_INFO MODULE_NAME ": Unregistered Bosch BMA150 driver\n");
	i2c_del_driver(&bma150_driver);
}

MODULE_AUTHOR("Markinus");
MODULE_DESCRIPTION("Bosch BMA150 driver over Microp");
MODULE_LICENSE("GPL");

module_init(bma150_init);
module_exit(bma150_exit);
