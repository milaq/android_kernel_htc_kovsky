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
#include <linux/input/msm_ts.h>
#include <linux/pda_power.h>
#include <linux/gpio_keys.h>
#include <linux/microp-keypad.h>
#include <linux/microp-htckovsky.h>
#include <linux/mfd/microp-ng.h>
#include <linux/ds2746_battery.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/mach/mmc.h>
#include <asm/setup.h>

#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/dma.h>
#include <mach/gpio.h>
#include <mach/htc_acoustic_wince.h>
#include <mach/htc_headset_35mm.h>
#include <mach/io.h>
#include <mach/msm_iomap.h>
#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/msm_fb.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_serial_hs.h>
#include <mach/vreg.h>

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
	ret = gpio_request(KOVS100_SLIDER_IRQ_GPIO, "Keyboard slider");
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
	.read_modifiers = true,
	.gpio_clamshell = -1,
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

static struct platform_device* htckovsky_microp_keypad_clients[] = {
	&htckovsky_keypad,
};

static uint16_t micropksc_compatible_versions[] = {
	0x707,
};

static struct microp_platform_data htckovsky_microp_keypad_pdata = {
	.version_reg = 0x12,
	.clients = htckovsky_microp_keypad_clients,
	.nclients = ARRAY_SIZE(htckovsky_microp_keypad_clients),
	.comp_versions = micropksc_compatible_versions,
	.n_comp_versions = ARRAY_SIZE(micropksc_compatible_versions),
};


/******************************************************************************
 * Power Supply
 ******************************************************************************/
static char *supplicants[] = {
	DS2746_NAME,
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
	msm_hsusb_set_vbus_state(vbus_state);
	return vbus_state;
}

static int htckovsky_is_ac_online(void)
{
	int vbus_state = readl(MSM_SHARED_RAM_BASE + 0xfc00c);
	int cable_in = !gpio_get_value(KOVS100_AC_DETECT);
	return cable_in & !vbus_state;
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
		gpio_direction_output(KOVS100_N_CHG_ENABLE, 1);
		return;
	}
	gpio_direction_output(KOVS100_N_CHG_ENABLE, 0);
}

static int htckovsky_power_init(struct device *dev)
{
	int rc = 0;

	rc = gpio_request(KOVS100_N_CHG_ENABLE, "Charger Disable");
	if (rc)
		goto err;

	rc = gpio_request(KOVS100_AC_DETECT, "Charger Detection");
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

static struct ds2746_platform_data kovsky_battery_data = {
	.resistance = 1500,
	.capacity = 1660,
	.high_voltage = 4200,
	.low_voltage = 3600,
};

/******************************************************************************
 * MicroP
 ******************************************************************************/
static struct platform_device htckovsky_microp_leds = {
  .id = -1,
  .name = "htckovsky-microp-leds",
};

static struct platform_device htckovsky_optical_joystick = {
  .id = -1,
  .name = "htckovsky-oj",
};

static struct platform_device* htckovsky_microp_clients[] = {
	&htckovsky_microp_leds,
//	&htckovsky_optical_joystick,
};

static uint16_t micropklt_compatible_versions[] = {
	0x787,
};

static struct microp_platform_data htckovsky_microp_pdata = {
	.version_reg = MICROP_VERSION_REG_KOVS,
	.clients = htckovsky_microp_clients,
	.nclients = ARRAY_SIZE(htckovsky_microp_clients),
	.comp_versions = micropklt_compatible_versions,
	.n_comp_versions = ARRAY_SIZE(micropklt_compatible_versions),
};

/******************************************************************************
 * I2C
 ******************************************************************************/
static struct i2c_board_info i2c_devices[] = {
	{
	// Battery driver
	.type = DS2746_NAME,
	.addr = 0x36,
	.platform_data = &kovsky_battery_data,
	},
	{
	 // LED & Backlight controller
	 .type = "microp-ng",
	 .addr = 0x66,
	 .platform_data = &htckovsky_microp_pdata,
	 },
	 {
	 // Keyboard controller
	 .type = "microp-ng",
	 .addr = 0x67,
	 .platform_data = &htckovsky_microp_keypad_pdata,
	 },
	{
	I2C_BOARD_INFO("mt9t012vc", 0x20),
	},
};

/******************************************************************************
 * USB Client
 ******************************************************************************/
static int htckovsky_request_ulpi_gpios(void)
{
	static bool done = false;
	int ret;

	if (done)
		return 0;

	ret = gpio_request(0x54, "Kovsky USB Unknown");
	if (ret)
		goto free_x54;
	ret = gpio_request(KOVS100_USB_RESET_PHY, "USB PHY Reset");
	if (ret)
		goto free_x64;
	ret = gpio_request(KOVS100_USB_POWER_PHY, "USB PHY Power");
	if (ret)
		goto free_x69;

	done = true;
	return 0;

free_x69:
	gpio_free(KOVS100_USB_POWER_PHY);
free_x64:
	gpio_free(KOVS100_USB_RESET_PHY);
free_x54:
	gpio_free(0x54);

	return ret;
}

static inline void htckovsky_usb_disable(void)
{
	printk(KERN_DEBUG "[KOVSKY]: Disable USB\n");
	gpio_direction_output(KOVS100_USB_RESET_PHY, 0);
	gpio_direction_output(KOVS100_USB_POWER_PHY, 0);
}

static void htckovsky_usb_enable(void)
{
	printk(KERN_DEBUG "[KOVSKY]: Enable USB\n");
	gpio_direction_output(0x54, 1);

	gpio_set_value(KOVS100_BT_ROUTER, 0);

	gpio_direction_output(KOVS100_USB_POWER_PHY, 1);
	gpio_direction_output(KOVS100_USB_RESET_PHY, 0);
	mdelay(3);
	gpio_direction_output(KOVS100_USB_RESET_PHY, 1);
	mdelay(3);
}

static void htckovsky_phy_reset(void) {
	printk(KERN_DEBUG "[KOVSKY]: %s\n", __func__);
	htckovsky_usb_disable();
	htckovsky_usb_enable();
}

static void htckovsky_usb_hw_reset(bool state)
{
	printk(KERN_DEBUG "[KOVSKY]: %s(%d)\n", __func__, state);
	if (state) {
		htckovsky_usb_disable();
	}
	else {
		htckovsky_usb_enable();
	}
}

static struct msm_hsusb_platform_data htckovsky_hsusb_pdata = {
	.hw_reset = htckovsky_usb_hw_reset,
	.phy_reset = htckovsky_phy_reset,
};

static struct platform_device android_usb = {
	.name = "android_usb_devices",
	.id = -1,
};

/******************************************************************************
 * Camera
 ******************************************************************************/
#ifdef CONFIG_MSM_CAMERA

static struct msm_gpio htckovsky_camera_gpios_on[] = {
	{.gpio_cfg = GPIO_CFG(2, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),.label = "DAT2 "},
	{.gpio_cfg = GPIO_CFG(3, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),.label = "DAT3 "},
	{.gpio_cfg = GPIO_CFG(4, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),.label = "DAT4 "},
	{.gpio_cfg = GPIO_CFG(5, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),.label = "DAT5 "},
	{.gpio_cfg = GPIO_CFG(6, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),.label = "DAT6 "},
	{.gpio_cfg = GPIO_CFG(7, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),.label = "DAT7 "},
	{.gpio_cfg = GPIO_CFG(8, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),.label = "DAT8 "},
	{.gpio_cfg = GPIO_CFG(9, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),.label = "DAT9 "},
	{.gpio_cfg = GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),.label = "DAT10 "},
	{.gpio_cfg = GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),.label = "DAT11 "},
	{.gpio_cfg = GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),.label = "PCLK "},
	{.gpio_cfg = GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),.label = "HSYNC_IN "},
	{.gpio_cfg = GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),.label = "VSYNC_IN "},
	{.gpio_cfg = GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),.label = "MCLK "},
};

static struct msm_gpio htckovsky_camera_gpios_off[] = {
	{.gpio_cfg = GPIO_CFG(2, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),.label = "DAT2 "},
	{.gpio_cfg = GPIO_CFG(3, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),.label = "DAT3 "},
	{.gpio_cfg = GPIO_CFG(4, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),.label = "DAT4 "},
	{.gpio_cfg = GPIO_CFG(5, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),.label = "DAT5 "},
	{.gpio_cfg = GPIO_CFG(6, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),.label = "DAT6 "},
	{.gpio_cfg = GPIO_CFG(7, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),.label = "DAT7 "},
	{.gpio_cfg = GPIO_CFG(8, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),.label = "DAT8 "},
	{.gpio_cfg = GPIO_CFG(9, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),.label = "DAT9 "},
	{.gpio_cfg = GPIO_CFG(10, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),.label = "DAT10 "},
	{.gpio_cfg = GPIO_CFG(11, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),.label = "DAT11 "},
	{.gpio_cfg = GPIO_CFG(12, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),.label = "PCLK "},
	{.gpio_cfg = GPIO_CFG(13, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),.label = "HSYNC_IN "},
	{.gpio_cfg = GPIO_CFG(14, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),.label = "VSYNC_IN "},
	{.gpio_cfg = GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA),.label = "MCLK "},
};

static void config_camera_on_gpios(void)
{
	printk(KERN_DEBUG "+%s\n", __func__);
	msm_gpios_request(htckovsky_camera_gpios_on,
							ARRAY_SIZE(htckovsky_camera_gpios_on));
	msm_gpios_disable(htckovsky_camera_gpios_on,
							ARRAY_SIZE(htckovsky_camera_gpios_on));
	printk(KERN_DEBUG "-%s\n", __func__);
}

static void config_camera_off_gpios(void)
{
	printk(KERN_DEBUG "-%s\n", __func__);
	msm_gpios_disable_free(htckovsky_camera_gpios_off,
							ARRAY_SIZE(htckovsky_camera_gpios_off));
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
int kovsky_af_vdd(int on)
{
	printk("%s(%d)\n", __func__, on);
	struct msm_dex_command dex = {
		.cmd = on ? DEX_PMIC_REG_ON : DEX_PMIC_REG_OFF,
		.has_data = 1,
		.data = 0x100,
	};

	return msm_dex_comm(&dex, 0);
}

int kovsky_pull_vcm_d(int on)
{
	printk("%s(%d)\n", __func__, on);
	kovsky_af_vdd(on);
	volatile char* focus;
	focus = ioremap(0xa9d00000, 0x1000);

	if (on) {
		writel(2, focus + 0x4c);
		writel(0x1e21, focus + 0x50);
		writel(0x1de, focus + 0x54);
		gpio_direction_output(0x6c, 1);
		gpio_direction_output(0x1c, 0);
	}
	else {
		gpio_direction_output(0x6c, 0);
		gpio_direction_output(0x1c, 0);
		writel(0, focus + 0x4c);
		writel(0, focus + 0x50);
		writel(0, focus + 0x54);
	}
	iounmap(focus);
	return 0;
}

#if 0
torch 0x57 - bright
0x55 - power

static void ov6680_power(int on) {
	0x17, 1
	0x1f, 1
	0x63, 1

	0x1, 0
	0x0, 1
}

static void set_sensor_vdd(void) {
	0, 0
	0x17, 1
	0x1f, 1
	pull vcm
}

#endif

int kovsky_camera_set_state(int on)
{
	static bool requested = false;

	if (!requested) {
		gpio_request(0, "Camera Reset");
		gpio_request(1, "Camera Power");
		gpio_request(0x17 /* 23 */, "Camera");
		gpio_request(0x1f /* 31 */, "Camera");
		gpio_request(0x63 /* 99 */, "VFE Mux");
		gpio_request(0x6c /* 107 */, "Camera VCM PWD");
		gpio_request(0x1c, "Camera VCM 2");

		requested = true;
	}

	printk("%s(%d)\n", __func__, on);
	gpio_direction_output(1, on); //sensor pwd
	//gpio_direction_output(0, 0); //sensor reset
	gpio_direction_output(0x17 /*23*/, on);
	gpio_direction_output(0x1f /*31*/, on);
	gpio_direction_output(0x63 /*99*/, !on); //MUX: 0 -> rear, 1 -> front

	if (on) {
		mdelay(2);
		gpio_direction_output(0, 1);
		mdelay(10);
		gpio_direction_output(0, 0);
		mdelay(10);
		gpio_direction_output(0, 1);
		mdelay(2);
	}
	return 0;
}

static struct msm_camera_sensor_info msm_camera_sensor_mt9t012vc_data = {
	//fake sensor name for the stupid proprietary driver
	.sensor_name = "mt9t013",
	.sensor_reset   = -1,
	.sensor_pwd     = -1,
	.vcm_pwd        = -1,
	.pdata = &msm_camera_device_data,
};

static struct platform_device msm_camera_sensor_mt9t012vc = {
	.name = "msm_camera_mt9t012vc",
	.dev = {
		.platform_data = &msm_camera_sensor_mt9t012vc_data,
	},
};
#endif	//CONFIG_MT9T012VC
#endif	//CONFIG_MSM_CAMERA


/******************************************************************************
 * GPIO Keys
 ******************************************************************************/
static struct gpio_keys_button htckovsky_button_table[] = {
	/*KEY   GPIO    ACTIVE_LOW      DESCRIPTION     type    wakeup  debounce */
	{KEY_POWER, 83, 1, "Power button", EV_KEY, 1, 0},
	{KEY_CAMERA - 1, 42, 1, "Camera half press", EV_KEY, 0, 10},
	{KEY_CAMERA, 41, 1, "Camera full press", EV_KEY, 0, 0},
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
static struct platform_device htckovsky_sdcc = {
	.name = "htckovsky-mmc",
	.id = -1,
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

/******************************************************************************
 * AMSS-specific stuff
 ******************************************************************************/
static struct platform_device amss_device = {
	.name = "amss_5225",
	.id = -1,
};

/******************************************************************************
 * Bluetooth
 ******************************************************************************/
struct platform_device htckovsky_rfkill = {
	.name = "htckovsky_rfkill",
	.id = -1,
};

static struct msm_serial_hs_platform_data msm_uart_dm2_pdata = {
	.rx_wakeup_irq = MSM_GPIO_TO_INT(KOVS100_UART2DM_RX),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
};

static void __init htckovsky_patch_uart_dma(void) {
	struct resource* res = msm_device_uart_dm2.resource;
	int n = msm_device_uart_dm2.num_resources;
	int i;

	for (i = 0; i < n; i++) {
		if (res[i].flags == IORESOURCE_DMA &&
			res[i].end == DMOV_HSUART2_RX_CHAN) {
			res[i].end = 9;
			break;
		}
	}

}
/******************************************************************************
 * Headset
 ******************************************************************************/
static struct htc_headset_35mm_pdata htckovsky_headset_data = {
	.gpio_detect = KOVS100_HEADSET_IN,
	.gpio_headset_mic = KOVS100_EXT_MIC,
	.jack_inverted = 1,
};

static struct platform_device htckovsky_headset = {
	.name = "htc_headset_35mm",
	.id = -1,
	.dev = {
		.platform_data = &htckovsky_headset_data,
		},
};

/******************************************************************************
 * Acoustic
 ******************************************************************************/
static void htckovsky_set_speaker_amp(bool enable) {
	gpio_direction_output(KOVS100_SPK_AMP, enable);
}

static void htckovsky_set_headset_amp(bool enable) {
	gpio_direction_output(KOVS100_HP_AMP, enable);
}

static struct htc_acoustic_wce_board_data htckovsky_acoustic_data = {
	.set_speaker_amp = htckovsky_set_speaker_amp,
	.set_headset_amp = htckovsky_set_headset_amp,
};

static int __init htckovsky_init_acoustic(void) {
	int ret = gpio_request(KOVS100_SPK_AMP, "Speaker Amplifier");
	if (ret)
		return ret;

	ret = gpio_request(KOVS100_HP_AMP, "Headset amplifier");
	if (ret) {
		gpio_free(KOVS100_SPK_AMP);
		return ret;
	}

	return 0;
}

/******************************************************************************
 * GPIOs
 * Only input gpios and those not handled by other drivers are listed here
 ******************************************************************************/
static struct msm_gpio kovsky_gpios_init_off[] = {
	{.gpio_cfg = GPIO_CFG(KOVS100_EXT_MIC, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Ext Mic"},
	{.gpio_cfg = GPIO_CFG(0x1b, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
	{.gpio_cfg = GPIO_CFG(0x1c, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
	{.gpio_cfg = GPIO_CFG(KOVS100_HEADSET_IN, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Headset Jack"},
	{.gpio_cfg = GPIO_CFG(0x1f, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
	{.gpio_cfg = GPIO_CFG(0x21, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
	{.gpio_cfg = GPIO_CFG(0x22, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
	{.gpio_cfg = GPIO_CFG(0x23, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
	{.gpio_cfg = GPIO_CFG(KOVS100_SLIDER_IRQ_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),.label = "Slider"},
	{.gpio_cfg = GPIO_CFG(KOVS100_CAM_FULL_KEY, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),.label = "Camera Key2"},
	{.gpio_cfg = GPIO_CFG(KOVS100_CAM_HALF_KEY, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),.label = "Camera Key1"},
	{.gpio_cfg = GPIO_CFG(KOVS100_JOYSTICK_IRQ, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),.label = "MicroP2"},
	{.gpio_cfg = GPIO_CFG(0x3e, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
	{.gpio_cfg = GPIO_CFG(0x44, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
	{.gpio_cfg = GPIO_CFG(0x45, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
	{.gpio_cfg = GPIO_CFG(0x46, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
	{.gpio_cfg = GPIO_CFG(0x47, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
	{.gpio_cfg = GPIO_CFG(0x55, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
	{.gpio_cfg = GPIO_CFG(0x56, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
	{.gpio_cfg = GPIO_CFG(0x57, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
	{.gpio_cfg = GPIO_CFG(0x65, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
	{.gpio_cfg = GPIO_CFG(0x67, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
	{.gpio_cfg = GPIO_CFG(0x6d, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
};

static struct msm_gpio kovsky_gpios_init_on[] = {
	{.gpio_cfg = GPIO_CFG(KOVS100_N_CHG_ENABLE, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Charger"},
	{.gpio_cfg = GPIO_CFG(KOVS100_SPK_UNK_0, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Speaker"},
	{.gpio_cfg = GPIO_CFG(KOVS100_SPK_UNK_1, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Speaker"},
	{.gpio_cfg = GPIO_CFG(0x54, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),.label = "Unknown"},
};

static void htckovsky_gpios_init(void) {
	int rc;
	rc = msm_gpios_request(kovsky_gpios_init_off, ARRAY_SIZE(kovsky_gpios_init_off));
	if (rc) {
		printk(KERN_ERR "%s: unable to request off gpios\n", __func__);
		return;
	}
	rc = msm_gpios_request(kovsky_gpios_init_on, ARRAY_SIZE(kovsky_gpios_init_on));
	if (rc) {
		printk(KERN_ERR "%s: unable to request on gpios\n", __func__);
		goto free_off;
	}

	msm_gpios_disable(kovsky_gpios_init_off, ARRAY_SIZE(kovsky_gpios_init_off));
	msm_gpios_enable(kovsky_gpios_init_on, ARRAY_SIZE(kovsky_gpios_init_on));
	msm_gpios_free(kovsky_gpios_init_on, ARRAY_SIZE(kovsky_gpios_init_on));

free_off:
	msm_gpios_free(kovsky_gpios_init_off, ARRAY_SIZE(kovsky_gpios_init_off));
}

static struct platform_device *devices[] __initdata = {
	&amss_device,
	&msm_device_i2c,
	&htckovsky_rtc,
	&htckovsky_sdcc,
	&htckovsky_gpio_keys,
	&htckovsky_rfkill,
	&msm_device_uart_dm2,
	&msm_device_hsusb,
	&android_usb,
	&htckovsky_powerdev,
	&msm_camera_sensor_mt9t012vc,
	&msm_device_touchscreen,
	&htckovsky_headset,
	&msm_device_nand,
};

extern struct sys_timer msm_timer;

static struct msm_acpu_clock_platform_data htckovsky_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200,
	.wait_for_irq_khz = 128000,
};

static void __init htckovsky_init(void)
{
	int i;

	msm_acpu_clock_init(&htckovsky_clock_data);
	msm_dex_comm_init();
	msm_add_mem_devices(&htckovsky_pmem_settings);
	htckovsky_gpios_init();

	htckovsky_request_ulpi_gpios();
	msm_hsusb_board_pdata = &htckovsky_hsusb_pdata;

	htckovsky_patch_uart_dma();
	msm_device_uart_dm2.dev.platform_data = &msm_uart_dm2_pdata;

	msm_device_touchscreen.dev.platform_data = &htckovsky_ts_pdata;

	if (!htckovsky_init_acoustic())
		htc_acoustic_wce_board_data = &htckovsky_acoustic_data;

	// Register devices
	platform_add_devices(devices, ARRAY_SIZE(devices));

	// Register I2C devices
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	//trigger usb detection
	htckovsky_is_usb_online();

	/* A little vibrating welcome */
	for (i = 0; i < 2; i++) {
		dex_vibrate(1);
		mdelay(150);
		dex_vibrate(0);
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
	mi->nr_banks = 1;
	mi->bank[0].start = PAGE_ALIGN(PHYS_OFFSET);
	mi->bank[0].node = PHYS_TO_NID(mi->bank[0].start);
#ifdef CONFIG_HOLES_IN_ZONE
	mi->bank[0].size = 107 * 1024 * 1024;
#else
	mi->bank[0].size = 104 << 20;
#endif

	mi->nr_banks++;
	mi->bank[1].start = PAGE_ALIGN(PHYS_OFFSET + 0x10000000);
	mi->bank[1].node = PHYS_TO_NID(mi->bank[1].start);
#ifdef CONFIG_HOLES_IN_ZONE
	mi->bank[1].size = (128 - 50) * 1024 * 1024;
#else
	mi->bank[1].size = 76 << 20;
#endif
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
