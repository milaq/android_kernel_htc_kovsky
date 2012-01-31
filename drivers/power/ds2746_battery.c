/*
 * Driver for batteries with DS2746 chips inside.
 *
 * Copyright © 2009 Matthew Kern
 * 	       2007 Anton Vorontsov
 *	       2004-2007 Matt Reimer
 *	       2004 Szabolcs Gyurko
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 * Author:  Matthew Kern <pyrophobicman@gmail.com
 * 	    January 2009
 *
 * 	    Anton Vorontsov <cbou@mail.ru>
 *	    February 2007
 *
 *	    Matt Reimer <mreimer@vpop.net>
 *	    April 2004, 2005, 2007
 *
 *	    Szabolcs Gyurko <szabolcs.gyurko@tlt.hu>
 *	    September 2004
 *
 *      Martin Johnson <m.j.johnson@massey.ac.nz>
 *      February 2009  - Simplified for HTC Kaiser
 *
 *	Stefan Seidel <kaiser@stefanseidel.info>
 *	March 2009 - Correct battery charge pctg. calculation
 *
 *      Carsten Wilhelm <carsten@wilhelm-net.de>
 *      May 2010 - Changed implementation for DEFAULT (aka Xperia X1)
 *
 *	Mathew McBride <matt@mcbridematt.dhs.org>
 *	August 2010 - Made standalone
 *
 * 	Alexander Tarasikov <alexander.tarasikov@gmail.com>
 * 	October 2010 - made generic
*/

/* References:
	ds2760 battery driver
	ds2782 battery driver
	ds2784 battery driver in Android kernel tree
 */
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/ds2746_battery.h>
#include <asm/gpio.h>

#include <linux/android_alarm.h>
#include <linux/wakelock.h>

/* definitions for registers we care about. */
#define DS2746_DATA_SIZE		0x12

#define DS2746_STATUS_REG		0x01
#define DS2746_AUX0_MSB			0x08
#define DS2746_AUX0_LSB			0x09
#define DS2746_AUX1_MSB			0x0a
#define DS2746_AUX1_LSB			0x0b
#define DS2746_VOLTAGE_MSB		0x0c
#define DS2746_VOLTAGE_LSB		0x0d
#define DS2746_CURRENT_MSB		0x0e
#define DS2746_CURRENT_LSB		0x0f
#define DS2746_CURRENT_ACCUM_MSB	0x10
#define DS2746_CURRENT_ACCUM_LSB	0x11
#define DS2746_OFFSET_BIAS              0x61
#define DS2746_ACCUM_BIAS               0x62

#define DEFAULT_RSNS			1500	// in mOHM. From the maths, this should be correct
#define DEFAULT_BATTERY_RATING		1500	// capacity of standard DEFAULT battery 1500mAh
#define DEFAULT_HIGH_VOLTAGE		4200
#define DEFAULT_LOW_VOLTAGE		3300
#define DEFAULT_MAX_OVER_CAPACITY	 105 // over capacity percent allowed

#define DS2746_CURRENT_ACCUM_RES	2440	// resolution of ACCUM-register in uVh * 100 per bit
#define DS2746_VOLTAGE_RES		2440	// resolution of voltage register multiplied by 1000

#define FAST_POLL (30 * 1000)
#define SLOW_POLL (2 * 60 * 1000)

struct i2c_client *pclient = 0;

static unsigned int battery_capacity = DEFAULT_BATTERY_RATING;
static unsigned int current_accum_capacity = 500;

module_param(battery_capacity, int, 0644);
MODULE_PARM_DESC(battery_capacity, "Estimated battery capacity in mAh");

#define DBG(fmt, x...) pr_debug(fmt, ##x)

struct ds2746_info {
	u32 batt_id;		/* Battery ID from ADC */
	u32 batt_vol;		/* Battery voltage from ADC */
	u32 batt_temp;		/* Battery Temperature (C) from formula and ADC */
	int batt_current;	/* Battery current from ADC */
	u32 level;		/* formula */
	u32 charging_source;	/* 0: no cable, 1:usb, 2:AC */
	bool charging_enabled;	/* 0: Disable, 1: Enable */
	u32 full_bat;		/* Full capacity of battery (mAh) */
	struct ds2746_platform_data bat_pdata;
	struct power_supply *bat;	/* Hold the supply struct so it can be passed along */
};

static struct ds2746_info *bi;

/** TODO: Should all of this live in the battery_info struct instead? */
/* Mutexes */
static DEFINE_MUTEX(bat_lock);
static struct delayed_work bat_work;
struct mutex work_lock;

/* Polling  */
struct workqueue_struct *monitor_wqueue;	/* Work queue used to ping the battery */

/* Polling periods - fast if on wall, slow if on battery */
/* Power management API stuff */
static enum power_supply_property ds2746_bat_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY
};

static int
ds2746_bat_get_property(struct power_supply *bat_ps,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (bi->charging_enabled) {
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		} else {
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bi->level;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = (bi->batt_temp * 10);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = (bi->batt_vol * 1000);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bi->batt_current;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void ds2746_ext_power_changed(struct power_supply *psy)
{
	cancel_delayed_work(&bat_work);
	queue_delayed_work(monitor_wqueue, &bat_work, 0);
}

static struct power_supply bat_ps = {
	.name = DS2746_NAME,
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = ds2746_bat_properties,
	.num_properties = ARRAY_SIZE(ds2746_bat_properties),
	.get_property = ds2746_bat_get_property,
	.external_power_changed = ds2746_ext_power_changed,
};

/* -----------------------------------------------------------
 * i2c utilitiy functions
 */
static int i2c_read(int r)
{
	unsigned char i2c_msg[1];
	unsigned char i2c_data[2];
	i2c_msg[0] = r;
	i2c_master_send(pclient, i2c_msg, 1);
	i2c_master_recv(pclient, i2c_data, 2);
	return i2c_data[0];
}

static void i2c_write(int r, int v)
{
	unsigned char i2c_msg[3];
	i2c_msg[0] = r;
	i2c_msg[1] = v >> 8;
	i2c_msg[2] = v & 0xFF;
	i2c_master_send(pclient, i2c_msg, 3);
}

static int reading2capacity(int r)
{
	return (DS2746_CURRENT_ACCUM_RES * r) / (bi->bat_pdata.resistance);
}

static int ds2746_battery_read_status(struct ds2746_info *b)
{
	signed short s;
	int aux0, aux1;
	int aux0r, aux1r;


	if (!pclient) {
		pr_err("client is null\n");
		return -ENODEV;
	}

	s = i2c_read(DS2746_VOLTAGE_LSB);
	s |= i2c_read(DS2746_VOLTAGE_MSB) << 8;
	b->batt_vol = ((s >> 4) * DS2746_VOLTAGE_RES) / 1000;

	s = i2c_read(DS2746_CURRENT_LSB);
	s |= i2c_read(DS2746_CURRENT_MSB) << 8;
	s >>= 2;
	b->batt_current = (s * DS2746_CURRENT_ACCUM_RES) / (bi->bat_pdata.resistance);

	/* if battery voltage is < 3.3V and depleting, we assume it's almost empty! */
	if (b->batt_vol < bi->bat_pdata.low_voltage && b->batt_current < 0) {
		aux0 = ((bi->bat_pdata.low_voltage - b->batt_vol) * current_accum_capacity * 5) / 3;
		printk(KERN_INFO "ds2746: correcting ACR to %d\n", aux0);
		/* use approximate formula: 3.3V=5%, 3.0V=0% */
		/* correction-factor is (capacity * 0.05) / (3300 - 3000) */
		/*  or (capacity*5/3) */
		i2c_write(DS2746_CURRENT_ACCUM_MSB, 0);
		i2c_write(DS2746_CURRENT_ACCUM_LSB, aux0);
	}
	s = i2c_read(DS2746_CURRENT_ACCUM_LSB);
	s |= i2c_read(DS2746_CURRENT_ACCUM_MSB) << 8;

	if (s > 0 && s <
			(DEFAULT_BATTERY_RATING * DEFAULT_RSNS * DEFAULT_MAX_OVER_CAPACITY) /
			(100 * DS2746_CURRENT_ACCUM_RES)) {
		if (s > current_accum_capacity) {
			/* if the battery is "fuller" than expected,
			 * update our expectations */
			printk(KERN_INFO "ds2746: old capacity == %u\n", current_accum_capacity);
			current_accum_capacity = s;
			battery_capacity = reading2capacity(s);
			printk(KERN_INFO
			       "ds2746: correcting max capacity to %d mAh (current_capacity=%u)\n",
			       battery_capacity, current_accum_capacity);
		} else {
			/* check if we're >4.2V and <5mA */
			if ((b->batt_vol >= bi->bat_pdata.high_voltage) && (b->batt_current < 5)) {
				/* that's almost full, so let's assume it is */
				current_accum_capacity = s + 20;
				battery_capacity = reading2capacity(s);
				printk(KERN_INFO
				       "ds2746: correcting capacity to %d (current capacity=%u)\n",
				       battery_capacity,
				       current_accum_capacity);
			}
		}
	}

	b->level = (s * 100) / current_accum_capacity;
	/* if we read 0%, */
	if (b->level < 1) {
		/* only report 0% if we're really down, <3.1V */
		b->level = ((b->batt_vol <= bi->bat_pdata.low_voltage - 100) ? 0 : 1);
	}

	if (b->level > 100)
		b->level = 100;

	aux0 = i2c_read(DS2746_AUX0_LSB);
	aux0 |= i2c_read(DS2746_AUX0_MSB) << 8;
	aux0 >>= 4;

	aux1 = i2c_read(DS2746_AUX1_LSB);
	aux1 |= i2c_read(DS2746_AUX1_MSB) << 8;
	aux1 >>= 4;

	// ratio
	aux0r = (1000L * aux0) / 2047;
	aux1r = (1000L * aux1) / 2047;
	// resistance in Ohm (R_AUX=10kOhm)
	//aux0r = (10000L*aux0r)/(1000-aux0r);
	//aux1r = (10000L*aux1r)/(1000-aux1r);

	b->batt_temp = (aux1r - 32L);
	b->batt_temp = ((b->batt_temp * 5) / 9) + 5;
	DBG("ds2746: %dmV %dmA charge: %d/100 (%d units) aux0: %d (%d) aux1: %d (%d) / %d\n",
	       b->batt_vol, b->batt_current, b->level, s, aux0, aux0r, aux1,
	       aux1r, b->batt_temp);

	return 0;
}

/**
 * Called by the work queue. Obtains a battery reading then
 * schedules the next read.
 */
static void ds2746_battery_work(struct work_struct *work)
{
	unsigned long next_update;

	if (bi->bat_pdata.block_charge) {
		bi->bat_pdata.block_charge(true);
		msleep(150);
	}

	ds2746_battery_read_status(bi);

	if (bi->bat_pdata.block_charge) {
		bi->bat_pdata.block_charge(false);
	}

	bi->charging_enabled = false;
	if (power_supply_am_i_supplied(bi->bat)) {
		next_update = msecs_to_jiffies(FAST_POLL);
		if (bi->level < 100 && bi->batt_vol < bi->bat_pdata.high_voltage + 60)
			bi->charging_enabled = true;
	}
	else {
		next_update = msecs_to_jiffies(SLOW_POLL);
	}

	power_supply_changed(bi->bat);
	queue_delayed_work(monitor_wqueue, &bat_work, next_update);
}

static int
ds2746_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct ds2746_platform_data pdata = {
		.resistance = DEFAULT_RSNS,
		.capacity = DEFAULT_BATTERY_RATING,
		.high_voltage = DEFAULT_HIGH_VOLTAGE,
		.low_voltage = DEFAULT_LOW_VOLTAGE,
	};

	// Try and check if we actually have the ds2746 on this device
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("ds2746: DS-2746 chip not present");
		return -ENODEV;
	}
	bi = kzalloc(sizeof(*bi), GFP_KERNEL);
	if (!bi) {
		return -ENOMEM;
	}
	pr_info("ds2746: Initializing DS-2746 chip driver at addr: 0x%02x\n",
	       client->addr);
	pclient = client;

	if (client->dev.platform_data)
		pdata = *(struct ds2746_platform_data*)client->dev.platform_data;

	current_accum_capacity = pdata.capacity * pdata.resistance;
	current_accum_capacity /= DS2746_CURRENT_ACCUM_RES;

	bi->bat_pdata = pdata;
	bi->full_bat = 100;

	DBG("ds2746: resistance = %d, capacity = %d, "
		"high_voltage = %d, low_voltage = %d, softACR = %d\n",
		pdata.resistance, pdata.capacity,
		pdata.high_voltage, pdata.low_voltage,
		current_accum_capacity);

	INIT_DELAYED_WORK(&bat_work, ds2746_battery_work);

	// Create a worker queue just for us
	monitor_wqueue = create_singlethread_workqueue("ds2746-monitor");
	// Schedule a reading
	queue_delayed_work(monitor_wqueue, &bat_work, 1);

	ret = power_supply_register(&client->dev, &bat_ps);
	bi->bat = &bat_ps;

	return ret;
}

static int ds2746_detach_client(struct i2c_client *client)
{
//	i2c_detach_client(client);
	return 0;
}

static struct i2c_device_id ds2746_idtable[] = {
	{DS2746_NAME, 0},
};

#ifdef CONFIG_PM
static int ds2746_battery_suspend(struct i2c_client *client, pm_message_t pmesg)
{
	DBG("ds2746: Cancelling scheduled reads\n");
	cancel_delayed_work(&bat_work);
	flush_workqueue(monitor_wqueue);
	return 0;
}

static int ds2746_battery_resume(struct i2c_client *client)
{
	DBG("ds2746: Resuming scheduled reads\n");
	queue_delayed_work(monitor_wqueue, &bat_work, 1);
	return 0;
}

#else

#define ds2746_battery_suspend NULL
#define ds2746_battery_resume NULL

#endif

static struct i2c_driver ds2746_battery_driver = {
	.probe = ds2746_probe,
	.remove = ds2746_detach_client,
	.suspend = ds2746_battery_suspend,
	.resume = ds2746_battery_resume,
	.driver = {
		   .name = DS2746_NAME,
		   },
	.id_table = ds2746_idtable,
};

static int __init ds2746_battery_init(void)
{
	return i2c_add_driver(&ds2746_battery_driver);
}

static void __exit ds2746_battery_exit(void)
{
	i2c_del_driver(&ds2746_battery_driver);
}

module_init(ds2746_battery_init);
module_exit(ds2746_battery_exit);

MODULE_DESCRIPTION("Battery driver for Xperia X1");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Matthew Kern <pyrophobicman@gmail.com>, "
	      "Szabolcs Gyurko <szabolcs.gyurko@tlt.hu>, "
	      "Matt Reimer <mreimer@vpop.net>, "
	      "Anton Vorontsov <cbou@mail.ru>, "
	      "Carsten Wilhelm <carsten@wilhelm-net.de>");
