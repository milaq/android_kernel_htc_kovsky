/* linux/arch/arm/mach-msm/board-htckovsky.c
*
* Copyright (C) 2010-2011 Alexander Tarasikov <alexander.tarasikov@gmail.com>
* Copyright (C) 2008-2009 Octavian Voicu, Martijn Stolk
* Copyright (C) 2007-2008 Brian Swetland <swetland@google.com>
* Copyright (C) 2007 Google, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/input.h>
#include <linux/delay.h>

#include <linux/android_pmem.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/gpio_keys.h>
#include <linux/microp-keypad.h>
#include <linux/microp-htckovsky.h>
#include <linux/microp-ng.h>
#include <linux/ds2746_battery.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/mach/mmc.h>
#include <asm/setup.h>

#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/gpio.h>
#include <mach/io.h>
#include <mach/msm_iomap.h>
#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/msm7x00a_mmc.h>
#include <mach/msm_fb.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_ts.h>
#include <mach/vreg.h>
#include <mach/amss/amss_5225.h>

#ifdef CONFIG_HTC_HEADSET
#include <mach/htc_headset.h>
#endif

#include "dex_comm.h"
#include "devices.h"
#include "board-htckovsky.h"
#include "clock-msm-a11.h"
#include "gpio_chip.h"

/******************************************************************************
 * MicroP Keypad
 ******************************************************************************/
static int htckovsky_microp_keymap[] = {
	KEY_RESERVED, // invalid
	KEY_ENTER,
	KEY_LEFT,
	KEY_F1, // NAVI_SILVER_LEFT; Menu-Key for Android
	KEY_SEND, // NAVI_GREEN
	KEY_A,
	KEY_F,
	KEY_S,
	KEY_D,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_POWER, // NAVI_SILVER_RIGHT; Power-Key for Android
	KEY_DOWN,
	KEY_RIGHT,
	KEY_HOME, // NAVI_XPANEL; Home-Key for Android
	KEY_K,
	KEY_J,
	KEY_H,
	KEY_G,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_END, // NAVI_RED
	KEY_BACK, // NAVI_OK; Back-Key for Android
	KEY_UP,
	KEY_RESERVED,
	KEY_L,
	KEY_I,
	KEY_P,
	KEY_O,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_B,
	KEY_APOSTROPHE,
	KEY_SEMICOLON,
	KEY_N,
	KEY_ENTER,
	KEY_M,
	KEY_C,
	KEY_V,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_ESC,
	KEY_U,
	KEY_E,
	KEY_R,
	KEY_Q,
	KEY_T,
	KEY_Y,
	KEY_W,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_SPACE,
	KEY_SPACE,
	KEY_SPACE,
	KEY_BACKSPACE, //CLOSE
	KEY_DOT,
	KEY_BACK, //OK ; same as front OK
	KEY_SLASH,
	KEY_COMMA,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_2,
	KEY_TAB,
	KEY_RIGHTALT,
	KEY_LEFTSHIFT,
	KEY_Z,
	KEY_X,
	KEY_LEFTCTRL,
	KEY_LEFTALT,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_RESERVED,
	KEY_VOLUMEUP,
	KEY_VOLUMEDOWN,
};

static int htckovsky_init_microp_keypad(struct device *dev) {
	int ret;
	ret = gpio_request(KOVS100_SLIDER_IRQ_GPIO, "HTC Kovsky Keyboard slider");
	if (ret)
		return ret;
	ret = gpio_direction_input(KOVS100_SLIDER_IRQ_GPIO);
	if (ret)
		gpio_free(KOVS100_SLIDER_IRQ_GPIO);

	return ret;
}

static void htckovsky_exit_microp_keypad(struct device *dev) {
	gpio_free(KOVS100_SLIDER_IRQ_GPIO);
}

static struct microp_keypad_platform_data htckovsky_keypad_data = {
	.init = htckovsky_init_microp_keypad,
	.exit = htckovsky_exit_microp_keypad,
	.irq_keypress = MSM_GPIO_TO_INT(KOVS100_SLIDER_IRQ_GPIO),
	.keypad_scancodes = htckovsky_microp_keymap,
	.keypad_scancodes_size = ARRAY_SIZE(htckovsky_microp_keymap),
};

static struct platform_device htckovsky_keypad = {
	.name = "microp-keypad",
	.id = -1,
	.dev = {
		.platform_data = &htckovsky_keypad_data,
	},
};

/******************************************************************************
 * Power Supply
 ******************************************************************************/
static char *supplicants[] = {
	"ds2746_battery",	//hardcoded in ds2746_battery driver for now
};

static struct resource htckovsky_power_resources[] = {
	[0] = {
	.name = "ac",
	.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE |
	IORESOURCE_IRQ_LOWEDGE,
	},
	[1] = {
	.name = "usb",
	.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE |
	IORESOURCE_IRQ_LOWEDGE,
	},
};

static int htckovsky_is_usb_online(void)
{
	int vbus_state = readl(MSM_SHARED_RAM_BASE + 0xfc00c);
	printk(KERN_DEBUG "[KOVSKY]: is usb online == %d\n", vbus_state);
	msm_hsusb_set_vbus_state(vbus_state);
	return vbus_state;
}

static int htckovsky_is_ac_online(void)
{
	int ret = !gpio_get_value(KOVS100_AC_DETECT);
	printk(KERN_DEBUG "[KOVSKY]: is ac online == %d\n", ret);
	return ret;
}

static void htckovsky_set_charge(int flags)
{
	switch (flags) {
	case PDA_POWER_CHARGE_USB:
		printk(KERN_DEBUG "[KOVSKY]: set USB charging\n");
		break;
	case PDA_POWER_CHARGE_AC:
		printk(KERN_DEBUG "[KOVSKY]: set AC charging\n");
		break;
	default:
		gpio_set_value(KOVS100_N_CHG_ENABLE, 1);
		return;
	}
	gpio_set_value(KOVS100_N_CHG_ENABLE, 0);
}

static int htckovsky_power_init(struct device *dev)
{
	int rc = 0;
	printk(KERN_DEBUG "[KOVSKY]: POWER INIT\n");

	rc = gpio_request(KOVS100_N_CHG_ENABLE, "HTC Kovsky Power Supply");
	if (rc)
		goto err;

	rc = gpio_request(KOVS100_AC_DETECT, "HTC Kovsky Power Supply");
	if (rc)
		goto err;

	htckovsky_power_resources[0].start = gpio_to_irq(KOVS100_AC_DETECT);
	htckovsky_power_resources[0].end = htckovsky_power_resources[0].start;
	htckovsky_power_resources[1].start = gpio_to_irq(KOVS100_AC_DETECT);
	htckovsky_power_resources[1].end = htckovsky_power_resources[1].start;
err:
	return rc;
}

static void htckovsky_power_exit(struct device *dev)
{
	gpio_free(KOVS100_AC_DETECT);
	gpio_free(KOVS100_N_CHG_ENABLE);
}

static struct pda_power_pdata htckovsky_power_data = {
	.init = htckovsky_power_init,
	.is_ac_online = htckovsky_is_ac_online,
	.is_usb_online = htckovsky_is_usb_online,
	.set_charge = htckovsky_set_charge,
	.exit = htckovsky_power_exit,
	.supplied_to = supplicants,
	.num_supplicants = ARRAY_SIZE(supplicants),
};

static struct platform_device htckovsky_powerdev = {
	.name = "pda-power",
	.id = -1,
	.resource = htckovsky_power_resources,
	.num_resources = ARRAY_SIZE(htckovsky_power_resources),
	.dev = {
		.platform_data = &htckovsky_power_data,
		},
};//End of power supply

static void ds2746_set_charge(int enable)
{
	if (!enable) {
		htckovsky_set_charge(0);
		return;
	}

	if (htckovsky_is_ac_online())
		htckovsky_set_charge(PDA_POWER_CHARGE_AC);
	else if (htckovsky_is_usb_online())
		htckovsky_set_charge(PDA_POWER_CHARGE_USB);
	else
		htckovsky_set_charge(0);

};

static struct ds2746_platform_data kovsky_battery_data = {
	.resistance = 1500,
	.capacity = 1660,
	.high_voltage = 4200,
	.low_voltage = 3600,
	.set_charge = ds2746_set_charge,
};

/******************************************************************************
 * MicroP
 ******************************************************************************/
static bool htckovsky_is_microp_supported(void) {
	uint8_t version[2];
	
	int ret = microp_ng_read(MICROP_VERSION_REG_KOVS, version, 2);
	if (ret < 0) {
	  printk(KERN_ERR "%s: error reading microp version %d\n", __func__, ret);
	  return false;
	}
	printk("%s: version %x%x\n", __func__, version[0], version[1]);
	return ((version[0] << 8) | version[1]) == 0x787;
}

static struct platform_device htckovsky_microp_leds = {
  .id = -1,
  .name = "htckovsky-microp-leds",
};

static struct platform_device* htckovsky_microp_clients[] = {
    &htckovsky_microp_leds,
};

static struct microp_platform_data htckovsky_microp_pdata = {
	.is_supported = htckovsky_is_microp_supported,
	.clients = htckovsky_microp_clients,
	.nclients = ARRAY_SIZE(htckovsky_microp_clients),
};

/******************************************************************************
 * I2C
 ******************************************************************************/
static struct i2c_board_info i2c_devices[] = {
	{
	I2C_BOARD_INFO("mt9t012vc", 0x20),
	},
	{
	// Battery driver
	.type = "ds2746-battery",
	.addr = 0x36,
	.platform_data = &kovsky_battery_data,
	},
	{
	 // LED & Backlight controller
	 .type = "microp-ng",
	 .addr = 0x66,
	 .platform_data = &htckovsky_microp_pdata,
	 },
//	{
//	// Keyboard controller
//	I2C_BOARD_INFO("microp-ksc", 0x67),
//	},
};

#define SND(num, desc) { .name = desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(0, "HANDSET"),
	SND(1, "SPEAKER"),
	SND(2, "HEADSET"),
	SND(3, "BT"),
	SND(44, "BT_EC_OFF"),
	SND(10, "HEADSET_AND_SPEAKER"),
	SND(256, "CURRENT"),

	/* Bluetooth accessories. */

	SND(12, "HTC BH S100"),
	SND(13, "HTC BH M100"),
	SND(14, "Motorola H500"),
	SND(15, "Nokia HS-36W"),
	SND(16, "PLT 510v.D"),
	SND(17, "M2500 by Plantronics"),
	SND(18, "Nokia HDW-3"),
	SND(19, "HBH-608"),
	SND(20, "HBH-DS970"),
	SND(21, "i.Tech BlueBAND"),
	SND(22, "Nokia BH-800"),
	SND(23, "Motorola H700"),
	SND(24, "HTC BH M200"),
	SND(25, "Jabra JX10"),
	SND(26, "320Plantronics"),
	SND(27, "640Plantronics"),
	SND(28, "Jabra BT500"),
	SND(29, "Motorola HT820"),
	SND(30, "HBH-IV840"),
	SND(31, "6XXPlantronics"),
	SND(32, "3XXPlantronics"),
	SND(33, "HBH-PV710"),
	SND(34, "Motorola H670"),
	SND(35, "HBM-300"),
	SND(36, "Nokia BH-208"),
	SND(37, "Samsung WEP410"),
	SND(38, "Jabra BT8010"),
	SND(39, "Motorola S9"),
	SND(40, "Jabra BT620s"),
	SND(41, "Nokia BH-902"),
	SND(42, "HBH-DS220"),
	SND(43, "HBH-DS980"),
};

#undef SND

static struct msm_snd_endpoints htckovsky_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = ARRAY_SIZE(snd_endpoints_list),
};

static struct platform_device htckovsky_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev = {
		.platform_data = &htckovsky_snd_endpoints,
		},
};

#ifdef CONFIG_HTC_HEADSET
static struct h2w_platform_data kovsky_headset_data = {
	.cable_in1 = KOVS100_HEADSET_IN,
	.headset_mic_35mm = KOVS100_EXT_MIC,
	.jack_inverted = 1,
};

static struct platform_device kovsky_headset = {
	.name = "htc_headset_35mm",
	.id = -1,
	.dev = {
		.platform_data = &kovsky_headset_data,
		},
};
#endif

/******************************************************************************
 * USB Client
 ******************************************************************************/
static void msm72k_usb_ulpi_config(int enable)
{
	int n;
	/* Configure ULPI DATA pins */
	for (n = 0x6f; n <= 0x76; n++) {
		gpio_tlmm_config(GPIO_CFG(n, 1,
					enable ? GPIO_CFG_INPUT :
					GPIO_CFG_OUTPUT,
					enable ? GPIO_CFG_NO_PULL :
					GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 0);
	}
	gpio_tlmm_config(GPIO_CFG(0x77, 1, GPIO_CFG_INPUT,
				enable ? GPIO_CFG_NO_PULL :
				GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 0);
	gpio_tlmm_config(GPIO_CFG
			(0x78, 1, GPIO_CFG_INPUT,
			enable ? GPIO_CFG_NO_PULL : GPIO_CFG_PULL_DOWN,
			GPIO_CFG_2MA), 0);
	gpio_tlmm_config(GPIO_CFG
			(0x79, 1, GPIO_CFG_OUTPUT,
			enable ? GPIO_CFG_NO_PULL : GPIO_CFG_PULL_UP,
			GPIO_CFG_2MA), 0);

}

static int usb_phy_init_seq_msm72k[] = {
	0x40, 0x31,		/* High Speed TX Boost */
	0x1D, 0x0D,		/* Rising edge interrupts control register */
	0x1D, 0x10,		/* Falling edge interrupts control register */
	0x5, 0xA,		/* OTG Control register */
	-1
};

//TODO: implement the corresponding functions
//in the usb driver via pdata
static int htckovsky_request_ulpi_gpios(void)
{
	static bool done = false;
	int n, ret;

	if (done)
		return 0;

	for (n = 0x6f; n <= 0x79; n++) {
		ret = gpio_request(n, "MSM ULPI");
		if (ret)
			goto free_ulpi;
	}
	ret = gpio_request(0x54, "MSM USB");
	if (ret)
		goto free_x54;
	ret = gpio_request(0x64, "MSM USB");
	if (ret)
		goto free_x64;
	ret = gpio_request(0x69, "MSM USB");
	if (ret)
		goto free_x69;

	done = true;
	return 0;

free_x69:
	gpio_free(0x69);
free_x64:
	gpio_free(0x64);
free_x54:
	gpio_free(0x54);

free_ulpi:
	for (--n; n >= 0x6f; n--)
		gpio_free(n);
	return ret;
}

static inline void htckovsky_usb_disable(void)
{
	printk(KERN_DEBUG "[KOVSKY]: Disable USB\n");
	htckovsky_request_ulpi_gpios();
	gpio_set_value(0x64, 0);
	gpio_set_value(0x69, 0);
}

static void htckovsky_usb_enable(void)
{
	htckovsky_request_ulpi_gpios();
	htckovsky_usb_disable();

	printk(KERN_DEBUG "[KOVSKY]: Enable USB\n");
	gpio_set_value(0x54, 1);

	gpio_set_value(KOVS100_BT_ROUTER, 0);

	gpio_set_value(0x69, 1);
	gpio_set_value(0x64, 0);
	mdelay(3);
	gpio_set_value(0x64, 1);
	mdelay(3);

	msm72k_usb_ulpi_config(1);
}

static void htckovsky_usb_connected(int state)
{
	printk(KERN_DEBUG "[KOVSKY]: %s(%d)\n", __func__, state);
	htckovsky_request_ulpi_gpios();
	if (!state) {
		msm72k_usb_ulpi_config(0);
		htckovsky_usb_disable();
	}
}

static struct msm_hsusb_platform_data htckovsky_hsusb_pdata = {
	.phy_init_seq = usb_phy_init_seq_msm72k,
	.phy_reset = htckovsky_usb_enable,
	.usb_connected = htckovsky_usb_connected,
};

static struct platform_device android_usb = {
	.name = "android_usb_devices",
	.id = -1,
};

/******************************************************************************
 * Camera
 ******************************************************************************/
#ifdef CONFIG_MSM_CAMERA
static unsigned camera_off_gpio_table[] = {
	/* CAMERA */
	GPIO_CFG(2, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),	/* DAT2 */
	GPIO_CFG(3, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),	/* DAT3 */
	GPIO_CFG(4, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),	/* DAT4 */
	GPIO_CFG(5, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),	/* DAT5 */
	GPIO_CFG(6, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),	/* DAT6 */
	GPIO_CFG(7, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),	/* DAT7 */
	GPIO_CFG(8, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),	/* DAT8 */
	GPIO_CFG(9, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),	/* DAT9 */
	GPIO_CFG(10, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),	/* DAT10 */
	GPIO_CFG(11, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),	/* DAT11 */
	GPIO_CFG(12, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),	/* PCLK */
	GPIO_CFG(13, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),	/* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),	/* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),	/* MCLK */
};

static unsigned camera_on_gpio_table[] = {
	/* CAMERA */
	GPIO_CFG(2, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT2 */
	GPIO_CFG(3, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT3 */
	GPIO_CFG(4, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT4 */
	GPIO_CFG(5, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT5 */
	GPIO_CFG(6, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT6 */
	GPIO_CFG(7, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT7 */
	GPIO_CFG(8, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT8 */
	GPIO_CFG(9, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT9 */
	GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT10 */
	GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* DAT11 */
	GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* PCLK */
	GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* VSYNC_IN */
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),	/* MCLK */
};

static void config_gpio_table(unsigned *table, int len)
{
	int n;
	unsigned id;
	for (n = 0; n < len; n++) {
		id = table[n];
		gpio_tlmm_config(id, 0);
	}
}

static void config_camera_on_gpios(void)
{
	printk(KERN_DEBUG "+%s\n", __func__);
	config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));
	printk(KERN_DEBUG "-%s\n", __func__);
}

static void config_camera_off_gpios(void)
{
	printk(KERN_DEBUG "-%s\n", __func__);
	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
	printk(KERN_DEBUG "-%s\n", __func__);
}

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz = MSM_CLK_CTL_SIZE,
};

#ifdef CONFIG_MT9T012VC
static void kovsky_af_vdd(int on)
{
	struct msm_dex_command dex = {
		.cmd = on ? DEX_PMIC_REG_ON : DEX_PMIC_REG_OFF,
		.has_data = 1,
		.data = 0x100,
	};

	msm_dex_comm(&dex, 0);
}

/* This is needed to control the lens position*/
static void kovsky_pull_vcm_d(int on)
{
	kovsky_af_vdd(on);
	gpio_direction_output(0x6b, on);
	gpio_direction_output(0x1c, on);
}

static int camera_set_state(int on)
{
	gpio_direction_output(1, on);
	gpio_direction_output(0x17, on);
	gpio_direction_output(0x1f, on);
	gpio_direction_output(0, on);
	gpio_direction_output(0x63, !on);

	if (on) {
		gpio_direction_output(0, 1);
		mdelay(10);
		gpio_direction_output(0, 0);
		mdelay(10);
		gpio_direction_output(0, 1);
	}
	return 0;
}

static struct msm_camera_sensor_info msm_camera_sensor_mt9t012vc_data = {
	//fake sensor name for the stupid proprietary driver
	.sensor_name = "mt9t013",
	.pdata = &msm_camera_device_data,
	.set_actuator = kovsky_pull_vcm_d,
	.set_state = camera_set_state,
};

static struct platform_device msm_camera_sensor_mt9t012vc = {
	.name = "msm_camera_mt9t012vc",
	.dev = {
		.platform_data = &msm_camera_sensor_mt9t012vc_data,
	},
};
#endif				//CONFIG_MT9T012VC
#endif				//CONFIG_MSM_CAMERA


/******************************************************************************
 * GPIO Keys
 ******************************************************************************/
static struct gpio_keys_button htckovsky_button_table[] = {
	/*KEY   GPIO    ACTIVE_LOW      DESCRIPTION     type    wakeup  debounce */
	{KEY_UP, 39, 1, "Volume Up", EV_KEY, 0, 0},
	{KEY_DOWN, 40, 1, "Volume Down", EV_KEY, 0, 0},
	{KEY_ENTER, 83, 1, "Power button", EV_KEY, 1, 0},
	{KEY_MENU, 42, 1, "Camera half press", EV_KEY, 0, 10},
	{KEY_HOME, 41, 1, "Camera full press", EV_KEY, 0, 0},
};

static struct gpio_keys_platform_data htckovsky_gpio_keys_data = {
	.buttons = htckovsky_button_table,
	.nbuttons = ARRAY_SIZE(htckovsky_button_table),
};

static struct platform_device htckovsky_gpio_keys = {
	.name = "gpio-keys",
	.dev = {
		.platform_data = &htckovsky_gpio_keys_data,
	},
	.id = -1,
};

/******************************************************************************
 * RTC
 ******************************************************************************/
struct platform_device htckovsky_rtc = {
	.name = "msm_rtc_dex",
	.id = -1,
};

/******************************************************************************
 * Touchscreen
 ******************************************************************************/
static struct msm_ts_platform_data htckovsky_ts_pdata = {
	.min_x		= 715,
	.max_x		= 3500,
	.min_y		= 290,
	.max_y		= 3680,
	.min_press	= 0,
	.max_press	= 256,
	.inv_x		= 0,
	.inv_y		= 0,
};

/******************************************************************************
 * SD Card slot
 ******************************************************************************/
static struct vreg *vreg_sdslot = NULL;

static int htckovsky_sdslot_init(struct device *dev) {
	int ret;

	ret = gpio_request(KOVS100_SD_STATUS, "HTC Kovsky SD Status");
	if (ret)
		goto fail_sd_status;

	ret = gpio_direction_input(KOVS100_SD_STATUS);
	if (ret)
		goto fail_gpio_in;

	vreg_sdslot = vreg_get(0, "gp6");
	if (IS_ERR(vreg_sdslot)) {
		ret = PTR_ERR(vreg_sdslot);
		goto fail_vreg;
	}
	return 0;

fail_vreg:
	vreg_sdslot = NULL;
fail_gpio_in:
	gpio_free(KOVS100_SD_STATUS);
fail_sd_status:
	return ret;
}

static void htckovsky_sdslot_exit(struct device *dev) {
	gpio_free(KOVS100_SD_STATUS);
	vreg_put(vreg_sdslot);
	vreg_sdslot = NULL;
}

static int htckovsky_sdslot_set_power(unsigned int vdd) {
	int rc = 0;
	switch (vdd) {
	case 0:
		rc = vreg_disable(vreg_sdslot);
		break;
	case 1:
		rc = vreg_enable(vreg_sdslot);
		break;
	default:
		rc = vreg_set_level(vreg_sdslot, vdd);
		break;
	}
	return rc;
}

static int htckovsky_sdslot_get_status(void) {
	int ret = !gpio_get_value(KOVS100_SD_STATUS);
	return ret;
}

static struct msm7x00a_mmc_platform_data htckovsky_sdslot_data = {
	.sdcc_id = 3,
	.sd_irq = MSM_GPIO_TO_INT(KOVS100_SD_STATUS),
	.init = htckovsky_sdslot_init,
	.exit = htckovsky_sdslot_exit,
	.set_voltage = htckovsky_sdslot_set_power,
	.get_status = htckovsky_sdslot_get_status,
};

static struct platform_device htckovsky_sd_slot = {
	.name = "msm7x00a-mmc",
	.dev = {
		.platform_data = &htckovsky_sdslot_data,
	},
	.id = 0,
};

/******************************************************************************
 * Reserved memory areas
 ******************************************************************************/
static struct msm_pmem_setting htckovsky_pmem_settings = {
	.pmem_start = KOVS110_PMEM_START,
	.pmem_size = KOVS110_PMEM_SIZE,
	.pmem_adsp_start = KOVS110_PMEM_ADSP_START,
	.pmem_adsp_size = KOVS110_PMEM_ADSP_SIZE,
	.pmem_gpu0_start = KOVS110_PMEM_GPU0_START,
	.pmem_gpu0_size = KOVS110_PMEM_GPU0_SIZE,
	.pmem_gpu1_start = KOVS110_PMEM_GPU1_START,
	.pmem_gpu1_size = KOVS110_PMEM_GPU1_SIZE,
	.pmem_camera_start = KOVS110_PMEM_CAMERA_START,
	.pmem_camera_size = KOVS110_PMEM_CAMERA_SIZE,
	.ram_console_start = KOVS110_RAMCONSOLE_START,
	.ram_console_size = KOVS110_RAMCONSOLE_SIZE,
};

static struct platform_device amss_device = {
	.name = "msm_adsp_5225",
	.id = -1,
};

struct smd_overrides smd_overrides_5225 = {
	.amss_values = amss_5225_para,
	.n_amss_values = ARRAY_SIZE(amss_5225_para),
};

static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&amss_device,
	&htckovsky_snd,
	&msm_device_i2c,
	&htckovsky_rtc,
	&htckovsky_sd_slot,
	&htckovsky_gpio_keys,
#ifdef CONFIG_SERIAL_MSM_HS
//      &msm_device_uart_dm2,
#endif
	&msm_device_hsusb,
#ifdef CONFIG_USB_ANDROID
	&android_usb,
#endif
	&htckovsky_powerdev,
#ifdef CONFIG_MT9T012VC
//      &msm_camera_sensor_mt9t012vc,
#endif
	&msm_device_touchscreen,
	&htckovsky_keypad,
};

extern struct sys_timer msm_timer;

static struct msm_acpu_clock_platform_data htckovsky_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200,
	.wait_for_irq_khz = 19200,
};

void msm_serial_debug_init(unsigned int base, int irq,
			const char *clkname, int signal_irq);

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm2_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(21),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
};
#endif

static void htcraphael_reset(void)
{
	struct msm_dex_command dex = {.cmd = DEX_NOTIFY_ARM9_REBOOT };
	msm_dex_comm(&dex, 0);
	mdelay(0x15e);
	gpio_request(25, "MSM Reset");
	msm_gpio_set_flags(25, GPIOF_OWNER_ARM11);
	gpio_direction_output(25, 0);
	printk(KERN_INFO "%s: Soft reset done.\n", __func__);
}

static void htcraphael_set_vibrate(uint32_t val)
{
	struct msm_dex_command vibra;

	if (val == 0) {
		vibra.cmd = DEX_VIBRA_OFF;
		msm_dex_comm(&vibra, 0);
	} else if (val > 0) {
		if (val == 1 || val > 0xb22)
			val = 0xb22;
		writel(val, MSM_SHARED_RAM_BASE + 0xfc130);
		vibra.cmd = DEX_VIBRA_ON;
		msm_dex_comm(&vibra, 0);
	}
}

static void __init htckovsky_init(void)
{
	int i;

	msm_acpu_clock_init(&htckovsky_clock_data);
	msm_dex_comm_init();
	msm_add_mem_devices(&htckovsky_pmem_settings);

	// Register hardware reset hook
	msm_hw_reset_hook = htcraphael_reset;

	msm_device_hsusb.dev.platform_data = &htckovsky_hsusb_pdata;

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm2.dev.platform_data = &msm_uart_dm2_pdata;
#endif
	msm_device_touchscreen.dev.platform_data = &htckovsky_ts_pdata;

	//do it before anything rpc kicks in
	amss_set_overrides(&smd_overrides_5225);
	// Register devices
	platform_add_devices(devices, ARRAY_SIZE(devices));

	// Register I2C devices
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	/* A little vibrating welcome */
	for (i = 0; i < 2; i++) {
		htcraphael_set_vibrate(1);
		mdelay(150);
		htcraphael_set_vibrate(0);
		mdelay(75);
	}
}

static void __init htckovsky_map_io(void)
{
	msm_map_common_io();
	msm_clock_a11_fixup();
	msm_clock_init(msm_clocks_7x01a, msm_num_clocks_7x01a);
}

static void __init htckovsky_fixup(struct machine_desc *desc, struct tag *tags,
				char **cmdline, struct meminfo *mi)
{
	int i;
	mi->nr_banks = 2;
	mi->bank[0].start = PAGE_ALIGN(PHYS_OFFSET);
	mi->bank[0].node = PHYS_TO_NID(mi->bank[0].start);
	mi->bank[0].size = 107 * 1024 * 1024;

	mi->bank[1].start = PAGE_ALIGN(PHYS_OFFSET + 0x10000000);
	mi->bank[1].node = PHYS_TO_NID(mi->bank[1].start);
	mi->bank[1].size = (128 - 34) * 1024 * 1024;
	printk(KERN_INFO "fixup: nr_banks = %d\n", mi->nr_banks);

	for (i = 0; i < mi->nr_banks; i++) {
		printk(KERN_INFO "fixup: bank%d start=%08lx, node=%08x, size=%08lx\n",
		i, mi->bank[i].start, mi->bank[i].node, mi->bank[i].size);
	}
}

MACHINE_START(HTCKOVSKY, "HTC Kovsky GSM phone (aka Xperia X1)")
	.fixup = htckovsky_fixup,
	.boot_params = 0x10000100,
	.map_io = htckovsky_map_io,
	.init_irq = msm_init_irq,
	.init_machine = htckovsky_init,
	.timer = &msm_timer,
MACHINE_END
