/*
 * MT9T012VC sensor driver for MSM VFE (C) 2010-2011 Alexander Tarasikov
 * <alexander.tarasikov@gmail.com>
 *
 * Original driver for MT9T013:
 * Copyright (C) 2008-2009 QUALCOMM Incorporated.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

//TODO:
// 1.Snapshot/Video mode frame size fix
// 2.AutoFocus
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "mt9t012vc.h"

extern int kovsky_camera_set_state(int on);
extern int kovsky_af_vdd(int on);
extern int kovsky_pull_vcm_d(int on);

enum mt9t012vc_resolution {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};

enum mt9t012vc_reg_update {
	REG_INIT,		/* registers that need to be updated during initialization */
	UPDATE_PERIODIC,	/* registers that needs periodic I2C writes */
	UPDATE_ALL,		/* all registers will be updated */
	UPDATE_INVALID
};

enum mt9t012vc_setting {
	RES_PREVIEW,
	RES_CAPTURE
};

/*
* AF Total steps parameters
*/
#define MT9T012VC_TOTAL_STEPS_NEAR_TO_FAR    45

/*
 * Time in milisecs for waiting for the sensor to reset.
 */
#define MT9T012VC_RESET_DELAY_MSECS   66

/* for 30 fps preview */
#define MT9T012VC_DEFAULT_CLOCK_RATE  24000000
#define MT9T012VC_DEFAULT_MAX_FPS     26

/* FIXME: Changes from here */
struct mt9t012vc_work {
	struct work_struct work;
};

static struct mt9t012vc_work *mt9t012vc_sensorw;
static struct i2c_client *mt9t012vc_client;

struct mt9t012vc_ctrl {
	const struct msm_camera_sensor_info *sensordata;

	int sensormode;
	uint32_t fps_divider;	/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;	/* init to 1 * 0x00000400 */

	uint16_t curr_lens_pos;
	uint16_t init_curr_lens_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;

	enum mt9t012vc_resolution prev_res;
	enum mt9t012vc_resolution pict_res;
	enum mt9t012vc_resolution curr_res;

	unsigned short imgaddr;
};

#define DLINE CDBG("[MT9T012VC] %s: at line %d\n", __func__, __LINE__);

static struct mt9t012vc_ctrl *mt9t012vc_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(mt9t012vc_wait_queue);
DECLARE_MUTEX(mt9t012vc_sem);

extern struct mt9t012vc_reg mt9t012vc_regs;	/* from mt9t012vc_reg.c */

static int mt9t012vc_i2c_rxdata(unsigned short saddr,
				unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
		.addr = saddr,
		.flags = 0,
		.len = 2,
		.buf = rxdata,
		},
		{
		.addr = saddr,
		.flags = I2C_M_RD,
		.len = length,
		.buf = rxdata,
		},
	};

	if (i2c_transfer(mt9t012vc_client->adapter, msgs, 2) < 0) {
		pr_err("mt9t012vc_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t mt9t012vc_i2c_read_w(unsigned short saddr,
				unsigned short raddr, unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);

	rc = mt9t012vc_i2c_rxdata(saddr, buf, 2);
	if (rc < 0)
		return rc;

	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)
		pr_err("mt9t012vc_i2c_read failed!\n");

	return rc;
}

static int32_t mt9t012vc_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
		.addr = saddr,
		.flags = 0,
		.len = length,
		.buf = txdata,
		},
	};

	if (i2c_transfer(mt9t012vc_client->adapter, msg, 1) < 0) {
		pr_err("mt9t012vc_i2c_txdata failed\n");
		return -EIO;
	}

	return 0;
}

static int32_t mt9t012vc_i2c_write_w(unsigned short saddr,
				unsigned short waddr, unsigned short wdata)
{
	int32_t rc = -EIO;
	unsigned char buf[4];
	pr_debug("[MT9T012VC]: %s(%02x, %02x)\n", __func__, waddr, wdata);

	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00) >> 8;
	buf[3] = (wdata & 0x00FF);

	rc = mt9t012vc_i2c_txdata(saddr, buf, 4);

	if (rc < 0)
		pr_err("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
		waddr, wdata);

	return rc;
}

static int32_t mt9t012vc_i2c_write_w_table(const struct mt9t012vc_i2c_reg_conf
					const *reg_conf_tbl,
					int num_of_items_in_table)
{
	int i;
	int32_t rc = -EIO;

	for (i = 0; i < num_of_items_in_table; i++) {
		rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr,
					reg_conf_tbl->waddr,
					reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}

	return rc;
}

static int32_t mt9t012vc_set_lc(void)
{
	int32_t rc;

	rc = mt9t012vc_i2c_write_w_table(mt9t012vc_regs.lctbl,
					mt9t012vc_regs.lctbl_size);
	if (rc < 0)
		return rc;

	return rc;
}

static int32_t mt9t012vc_move_focus(int direction, int32_t num_steps)
{
	//static const int max_focus = 0x1de;
	//static const int min_focus = 0x9a;
	pr_debug("[MT9T012VC]: %s(dir=%d, num_steps=%d)\n", __func__, direction, num_steps);
	return 0;
}

static int32_t mt9t012vc_set_default_focus(uint8_t af_step)
{
	pr_debug("[MT9T012VC]: %s(%d)\n", __func__, af_step);
	return 0;
}

static void mt9t012vc_get_pict_fps(uint16_t fps, uint16_t * pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider;	/*Q10 */
	uint32_t pclk_mult;	/*Q10 */

	if (mt9t012vc_ctrl->prev_res == QTR_SIZE) {
		divider =
		(uint32_t) (((mt9t012vc_regs.
				preview_reg_pat->frame_length_lines *
				mt9t012vc_regs.
				preview_reg_pat->line_length_pck) *
				0x00000400) /
				(mt9t012vc_regs.
				capture_reg_pat->frame_length_lines *
				mt9t012vc_regs.
				capture_reg_pat->line_length_pck));

		pclk_mult =
		(uint32_t) ((mt9t012vc_regs.
				capture_reg_pat->pll_multiplier *
				0x00000400) /
				(mt9t012vc_regs.
				preview_reg_pat->pll_multiplier));

	} else {
		/* full size resolution used for preview. */
		divider = 0x00000400;	/*1.0 */
		pclk_mult = 0x00000400;	/*1.0 */
	}

	/* Verify PCLK settings and frame sizes. */
	*pfps =
	(uint16_t) (fps * divider * pclk_mult / 0x00000400 / 0x00000400);
}

static uint16_t mt9t012vc_get_prev_lines_pf(void)
{
	if (mt9t012vc_ctrl->prev_res == QTR_SIZE)
		return mt9t012vc_regs.preview_reg_pat->frame_length_lines;
	else
		return mt9t012vc_regs.capture_reg_pat->frame_length_lines;
}

static uint16_t mt9t012vc_get_prev_pixels_pl(void)
{
	if (mt9t012vc_ctrl->prev_res == QTR_SIZE)
		return mt9t012vc_regs.preview_reg_pat->line_length_pck;
	else
		return mt9t012vc_regs.capture_reg_pat->line_length_pck;
}

static uint16_t mt9t012vc_get_pict_lines_pf(void)
{
	return mt9t012vc_regs.capture_reg_pat->frame_length_lines;
}

static uint16_t mt9t012vc_get_pict_pixels_pl(void)
{
	return mt9t012vc_regs.capture_reg_pat->line_length_pck;
}

static uint32_t mt9t012vc_get_pict_max_exp_lc(void)
{
	uint16_t snapshot_lines_per_frame;

	if (mt9t012vc_ctrl->pict_res == QTR_SIZE) {
		snapshot_lines_per_frame =
		mt9t012vc_regs.preview_reg_pat->frame_length_lines - 1;
	} else {
		snapshot_lines_per_frame =
		mt9t012vc_regs.capture_reg_pat->frame_length_lines - 1;
	}

	return snapshot_lines_per_frame * 24;
}

static int32_t mt9t012vc_set_fps(struct fps_cfg *fps)
{
	/* input is new fps in Q8 format */
	int32_t rc = 0;

	mt9t012vc_ctrl->fps_divider = fps->fps_div;
	mt9t012vc_ctrl->pict_fps_divider = fps->pict_fps_div;

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr,
				REG_GROUPED_PARAMETER_HOLD,
				GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return -EBUSY;

	pr_debug("mt9t012vc_set_fps: fps_div is %d, frame_rate is %d\n",
	fps->fps_div,
	(uint16_t) (mt9t012vc_regs.preview_reg_pat->
			frame_length_lines * fps->fps_div / 0x00000400));

	pr_debug("mt9t012vc_set_fps: fps_mult is %d, frame_rate is %d\n",
	fps->f_mult,
	(uint16_t) (mt9t012vc_regs.preview_reg_pat->line_length_pck *
			fps->f_mult / 0x00000400));

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr,
				REG_LINE_LENGTH_PCK,
				(uint16_t) (mt9t012vc_regs.preview_reg_pat->line_length_pck *
					fps->f_mult / 0x00000400));
	if (rc < 0)
		return rc;

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr,
				REG_GROUPED_PARAMETER_HOLD,
				GROUPED_PARAMETER_UPDATE);
	if (rc < 0)
		return rc;

	return rc;
}

static int32_t mt9t012vc_write_exp_gain(uint16_t gain, uint32_t line)
{
	const uint16_t max_legal_gain = 0x6F;
	uint32_t current_frame_length_lines;
	uint32_t line_length_ratio = 0x00000400;
	enum mt9t012vc_setting setting;
	int32_t rc = 0;

	if (mt9t012vc_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
		mt9t012vc_ctrl->my_reg_gain = gain;
		mt9t012vc_ctrl->my_reg_line_count = (uint16_t) line;
	}

	if (gain > max_legal_gain)
		gain = max_legal_gain;

	/* Verify no overflow */
	if (mt9t012vc_ctrl->sensormode != SENSOR_SNAPSHOT_MODE) {
		line = (uint32_t) (line * mt9t012vc_ctrl->fps_divider /
				0x00000400);

		setting = RES_PREVIEW;

	} else {
		line = (uint32_t) (line * mt9t012vc_ctrl->pict_fps_divider /
				0x00000400);

		setting = RES_CAPTURE;
	}

	/*Set digital gain to 1 */
	gain |= 0x0100;

	if (RES_PREVIEW == setting) {
		current_frame_length_lines = mt9t012vc_regs.preview_reg_pat->frame_length_lines;
	}
	else {
		current_frame_length_lines = mt9t012vc_regs.capture_reg_pat->frame_length_lines;
	}

	if ((current_frame_length_lines - 1) < line) {
		line_length_ratio =
		(uint32_t) (line * 0x00000400) /
		(current_frame_length_lines - 1);
	} else
		line_length_ratio = 0x00000400;

	pr_debug("[MT9T012VC] %s(gain = 0x%x, line = %d)\n", __func__, gain, line);

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, REG_GLOBAL_GAIN,
				gain);
	if (rc < 0)
		return rc;

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr,
				REG_COARSE_INT_TIME,
				(uint16_t)line);
	if (rc < 0)
		return rc;

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_UPDATE);
	if (rc < 0)
		return rc;

	return rc;
}

static int32_t mt9t012vc_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;

	rc = mt9t012vc_write_exp_gain(gain, line);
	if (rc < 0)
		return rc;

/*	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr,
				MT9T012VC_REG_RESET_REGISTER,
				MT9T012VC_RESET_REGISTER_PWON | 0x0002);*/

	mdelay(5);

	/* camera_timed_wait(snapshot_wait*exposure_ratio); */
	return rc;
}

static int32_t mt9t012vc_setting(enum mt9t012vc_reg_update rupdate,
				enum mt9t012vc_setting rt)
{
	const struct reg_struct* mode_regs;
	int32_t rc = 0;

	if (RES_PREVIEW == rt) {
		mode_regs = mt9t012vc_regs.preview_reg_pat;
	}
	else {
		mode_regs = mt9t012vc_regs.capture_reg_pat;
	}

	switch (rupdate) {
	case UPDATE_PERIODIC:
	case REG_INIT:
		if (rt != RES_PREVIEW && rt != RES_CAPTURE)
			break;

		rc = mt9t012vc_set_lc();
		if (rc < 0)
			return -EBUSY;

		rc = mt9t012vc_i2c_write_w
		(mt9t012vc_client->addr, 0x318a, 0x8000);
		if (rc < 0)
			return rc;

		rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, REG_GROUPED_PARAMETER_HOLD, 0x0);
		if (rc < 0)
			return rc;

		rc = mt9t012vc_i2c_write_w
		(mt9t012vc_client->addr,
			REG_MODE_SELECT,
			0x100);
		if (rc < 0)
			return rc;

		rc = mt9t012vc_i2c_write_w
		(mt9t012vc_client->addr, MT9T012VC_REG_RESET_REGISTER, MT9T012VC_RESET_REGISTER_PWON);
		if (rc < 0)
			return rc;

		rc = mt9t012vc_i2c_write_w
		(mt9t012vc_client->addr, REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);
		if (rc < 0)
			return rc;

		rc = mt9t012vc_i2c_write_w
		(mt9t012vc_client->addr, REG_READ_MODE, mode_regs->read_mode);
		if (rc < 0)
			return rc;

		rc = mt9t012vc_i2c_write_w
		(mt9t012vc_client->addr, REG_LINE_LENGTH_PCK, mode_regs->line_length_pck);
		if (rc < 0)
			return rc;

		rc = mt9t012vc_i2c_write_w
		(mt9t012vc_client->addr, REG_FRAME_LENGTH_LINES, mode_regs->frame_length_lines);
		if (rc < 0)
			return rc;

		rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, REG_X_ADDR_START, mode_regs->x_addr_start);
		if (rc < 0)
			return rc;

		rc = mt9t012vc_i2c_write_w
		(mt9t012vc_client->addr, REG_X_ADDR_END, mode_regs->x_addr_end);
		if (rc < 0)
			return rc;

		rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, REG_Y_ADDR_START, mode_regs->y_addr_start);
		if (rc < 0)
			return rc;

		rc = mt9t012vc_i2c_write_w
		(mt9t012vc_client->addr, REG_Y_ADDR_END, mode_regs->y_addr_end);
		if (rc < 0)
			return rc;

		rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, REG_PIXEL_ORDER, 0x0);
		if (rc < 0)
			return rc;

		rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, REG_X_OUTPUT_SIZE, mode_regs->x_output_size);
		if (rc < 0)
			return rc;

		rc = mt9t012vc_i2c_write_w
		(mt9t012vc_client->addr, REG_Y_OUTPUT_SIZE, mode_regs->y_output_size);
		if (rc < 0)
			return rc;

		rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_UPDATE);
		if (rc < 0)
			return rc;

		if (rupdate == REG_INIT)
			CDBG("[MT9T012VC]: PowerOn is done\n");
		mdelay(5);
		return rc;
		break;
	default:
		DLINE;
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int32_t mt9t012vc_video_config(int mode, int res)
{
	int32_t rc;

	switch (res) {
	case QTR_SIZE:
		rc = mt9t012vc_setting(UPDATE_PERIODIC, RES_PREVIEW);
		if (rc < 0)
			return rc;
		CDBG("sensor configuration done!\n");
		break;

	case FULL_SIZE:
		rc = mt9t012vc_setting(UPDATE_PERIODIC, RES_CAPTURE);
		if (rc < 0)
			return rc;
		break;

	default:
		return -EINVAL;
	}

	mt9t012vc_ctrl->prev_res = res;
	mt9t012vc_ctrl->curr_res = res;
	mt9t012vc_ctrl->sensormode = mode;

	return mt9t012vc_write_exp_gain(mt9t012vc_ctrl->my_reg_gain,
					mt9t012vc_ctrl->my_reg_line_count);
}

static int32_t mt9t012vc_snapshot_config(int mode)
{
	int32_t rc = 0;

	rc = mt9t012vc_setting(UPDATE_PERIODIC, RES_CAPTURE);
	if (rc < 0)
		return rc;

	mt9t012vc_ctrl->curr_res = mt9t012vc_ctrl->pict_res;
	mt9t012vc_ctrl->sensormode = mode;
	return rc;
}

static int32_t mt9t012vc_raw_snapshot_config(int mode)
{
	int32_t rc = 0;

	rc = mt9t012vc_setting(UPDATE_PERIODIC, RES_CAPTURE);
	if (rc < 0)
		return rc;

	mt9t012vc_ctrl->curr_res = mt9t012vc_ctrl->pict_res;
	mt9t012vc_ctrl->sensormode = mode;
	return rc;
}

static int32_t mt9t012vc_power_down(void)
{
	int32_t rc = 0;

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr,
				MT9T012VC_REG_RESET_REGISTER,
				MT9T012VC_RESET_REGISTER_PWOFF);
	if (rc >= 0)
		mdelay(5);
	return rc;
}

static int mt9t012vc_sensor_init_done(const struct msm_camera_sensor_info
				*data)
{
	kovsky_camera_set_state(0);
	return 0;
}

static int mt9t012vc_probe_init_sensor(const struct msm_camera_sensor_info
				*data)
{
	int rc;
	int i = 0;
	uint16_t chipid;

//	data->set_sensor(0);
	kovsky_camera_set_state(0);
	mdelay(20);
//	data->set_sensor(1);
	kovsky_camera_set_state(1);
	mdelay(60);

	DLINE;

	do {
		++i;
		rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, REG_SOFTWARE_RESET, 0x100);
		if (rc >= 0)
			break;
		mdelay(20);
	} while (i < 10);
	if (rc < 0) {
		goto init_probe_fail;
	}

	mdelay(0x64);

	/* RESET the sensor image part via I2C command */
	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr,
				MT9T012VC_REG_RESET_REGISTER, MT9T012VC_RESET_REGISTER_PWOFF);
	if (rc < 0)
		goto init_probe_fail;

	/* 3. Read sensor Model ID: */
	rc = mt9t012vc_i2c_read_w(mt9t012vc_client->addr,
				MT9T012VC_REG_MODEL_ID, &chipid);

	if (rc < 0)
		goto init_probe_fail;

	CDBG("mt9t012vc model_id = 0x%x\n", chipid);

	/* 4. Compare sensor ID to MT9T012VC ID: */
	if (chipid != MT9T012VC_MODEL_ID) {
		rc = -ENODEV;
		DLINE;
		goto init_probe_fail;
	}

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, REG_PLL_MULTIPLIER, 0x50);
	if (rc < 0) {
		goto init_probe_fail;
	}

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, REG_PRE_PLL_CLK_DIV, 0x3);
	if (rc < 0) {
		goto init_probe_fail;
	}

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, REG_VT_SYS_CLK_DIV, 0x2);
	if (rc < 0) {
		goto init_probe_fail;
	}

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, REG_VT_PIX_CLK_DIV, 0x5);
	if (rc < 0) {
		goto init_probe_fail;
	}

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, 0x3064, 0x5);
	if (rc < 0) {
		goto init_probe_fail;
	}

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, MT9T012VC_REG_RESET_REGISTER, MT9T012VC_RESET_REGISTER_PWOFF | 0x9000);
	if (rc < 0) {
		goto init_probe_fail;
	}

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, 0x600, 0x0);
	if (rc < 0) {
		goto init_probe_fail;
	}

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, 0x30c0, 0x81);
	if (rc < 0) {
		goto init_probe_fail;
	}

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, 0x30c2, 0x32);
	if (rc < 0) {
		goto init_probe_fail;
	}

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, 0x30c4, 0x32);
	if (rc < 0) {
		goto init_probe_fail;
	}

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, 0x30c6, 0x32);
	if (rc < 0) {
		goto init_probe_fail;
	}

	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, 0x30c8, 0x32);
	if (rc < 0) {
		goto init_probe_fail;
	}

	mdelay(1);
	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr, REG_DATAPATH_SELECT, 0xFC00);
	if (rc < 0) {
		goto init_probe_fail;
	}

	goto init_probe_done;

 init_probe_fail:
	CDBG("[MT9T012VC]: %s failed with rc=%d\n", __func__, rc);
	//data->set_sensor(0);
	kovsky_camera_set_state(0);
 init_probe_done:
	CDBG("[MT9T012VC]: %s done successfully\n", __func__);
	return rc;
}

static int32_t mt9t012vc_poweron_af(void)
{
	int32_t rc = 0;
	kovsky_pull_vcm_d(1);
	mt9t012vc_set_default_focus(0);

	return rc;
}

static void mt9t012vc_poweroff_af(void)
{
	kovsky_pull_vcm_d(0);
}

int mt9t012vc_sensor_open_init(const struct msm_camera_sensor_info
			*data)
{
	int32_t rc;

	mt9t012vc_ctrl = kzalloc(sizeof(struct mt9t012vc_ctrl), GFP_KERNEL);
	if (!mt9t012vc_ctrl) {
		pr_err("mt9t012vc_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	mt9t012vc_ctrl->fps_divider = 1 * 0x00000400;
	mt9t012vc_ctrl->pict_fps_divider = 1 * 0x00000400;
	//mt9t012vc_ctrl->set_test = TEST_OFF;
	mt9t012vc_ctrl->prev_res = QTR_SIZE;
	mt9t012vc_ctrl->pict_res = FULL_SIZE;

	if (data)
		mt9t012vc_ctrl->sensordata = data;

	/* enable mclk first */
	msm_camio_clk_rate_set(MT9T012VC_DEFAULT_CLOCK_RATE);
	mdelay(20);

	msm_camio_camif_pad_reg_reset();
	mdelay(20);

	DLINE;
	rc = mt9t012vc_probe_init_sensor(data);
	if (rc < 0)
		goto init_fail;
	DLINE;

	if (mt9t012vc_ctrl->prev_res == QTR_SIZE)
		rc = mt9t012vc_setting(REG_INIT, RES_PREVIEW);
	else
		rc = mt9t012vc_setting(REG_INIT, RES_CAPTURE);

	if (rc >= 0)
		rc = mt9t012vc_poweron_af();

	DLINE;
	if (rc < 0)
		goto init_fail;
	else
		goto init_done;
	DLINE;
 init_fail:
	kfree(mt9t012vc_ctrl);
	DLINE;
 init_done:
	DLINE;
	return rc;
}

static int mt9t012vc_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9t012vc_wait_queue);
	return 0;
}

static int32_t mt9t012vc_set_sensor_mode(int mode, int res)
{
	int32_t rc = 0;
	rc = mt9t012vc_i2c_write_w(mt9t012vc_client->addr,
				REG_GROUPED_PARAMETER_HOLD,
				GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = mt9t012vc_video_config(mode, res);
		break;

	case SENSOR_SNAPSHOT_MODE:
		rc = mt9t012vc_snapshot_config(mode);
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
		rc = mt9t012vc_raw_snapshot_config(mode);
		break;

	default:
		return -EINVAL;
	}

	/* FIXME: what should we do if rc < 0? */
	if (rc >= 0)
		return mt9t012vc_i2c_write_w(mt9t012vc_client->addr,
					REG_GROUPED_PARAMETER_HOLD,
					GROUPED_PARAMETER_UPDATE);
	return rc;
}

int mt9t012vc_sensor_config(void __user * argp)
{
	struct sensor_cfg_data cdata;
	long rc = 0;

	if (copy_from_user(&cdata, (void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	down(&mt9t012vc_sem);

	pr_debug("mt9t012vc_sensor_config: cfgtype = %d\n", cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_GET_PICT_FPS:
		mt9t012vc_get_pict_fps(cdata.cfg.gfps.prevfps,
				&(cdata.cfg.gfps.pictfps));
		if (copy_to_user((void *)argp,
				&cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_L_PF:
		cdata.cfg.prevl_pf = mt9t012vc_get_prev_lines_pf();
		if (copy_to_user((void *)argp,
				&cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_P_PL:
		cdata.cfg.prevp_pl = mt9t012vc_get_prev_pixels_pl();
		if (copy_to_user((void *)argp,
				&cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_L_PF:
		cdata.cfg.pictl_pf = mt9t012vc_get_pict_lines_pf();
		if (copy_to_user((void *)argp,
				&cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_P_PL:
		cdata.cfg.pictp_pl = mt9t012vc_get_pict_pixels_pl();

		if (copy_to_user((void *)argp,
				&cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_MAX_EXP_LC:
		cdata.cfg.pict_max_exp_lc = mt9t012vc_get_pict_max_exp_lc();

		if (copy_to_user((void *)argp,
				&cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_FPS:
	case CFG_SET_PICT_FPS:
		rc = mt9t012vc_set_fps(&(cdata.cfg.fps));
		break;

	case CFG_SET_EXP_GAIN:
		rc = mt9t012vc_write_exp_gain(cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
		break;

	case CFG_SET_PICT_EXP_GAIN:
		rc = mt9t012vc_set_pict_exp_gain(cdata.cfg.exp_gain.gain,
						cdata.cfg.exp_gain.line);
		break;

	case CFG_SET_MODE:
		rc = mt9t012vc_set_sensor_mode(cdata.mode, cdata.rs);
		break;

	case CFG_PWR_DOWN:
		rc = mt9t012vc_power_down();
		break;

	case CFG_MOVE_FOCUS:
		rc = mt9t012vc_move_focus(cdata.cfg.focus.dir,
					cdata.cfg.focus.steps);
		break;

	case CFG_SET_DEFAULT_FOCUS:
		rc = mt9t012vc_set_default_focus(cdata.cfg.focus.steps);
		break;

	case CFG_GET_AF_MAX_STEPS:
		cdata.max_steps = MT9T012VC_TOTAL_STEPS_NEAR_TO_FAR;
		if (copy_to_user((void *)argp,
				&cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_EFFECT:
	default:
		rc = -EINVAL;
		break;
	}

	up(&mt9t012vc_sem);
	return rc;
}

static int mt9t012vc_sensor_release(void)
{
	int rc = -EBADF;

	down(&mt9t012vc_sem);

	mt9t012vc_poweroff_af();
	mt9t012vc_power_down();

	kovsky_camera_set_state(0);

//	gpio_direction_output(mt9t012vc_ctrl->sensordata->sensor_reset,
//			0);
//	gpio_free(mt9t012vc_ctrl->sensordata->sensor_reset);

//	mt9t012vc_ctrl->sensordata->set_sensor(0);

	kfree(mt9t012vc_ctrl);

	up(&mt9t012vc_sem);
	CDBG("mt9t012vc_release completed!\n");
	return rc;
}

static int mt9t012vc_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc = 0;
	DLINE;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DLINE;
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	mt9t012vc_sensorw = kzalloc(sizeof(struct mt9t012vc_work), GFP_KERNEL);

	if (!mt9t012vc_sensorw) {
		DLINE;
		rc = -ENOMEM;
		goto probe_failure;
	}

	DLINE;
	i2c_set_clientdata(client, mt9t012vc_sensorw);
	mt9t012vc_init_client(client);
	mt9t012vc_client = client;
	mt9t012vc_client->addr = mt9t012vc_client->addr >> 1;
	mdelay(50);
	DLINE;

	CDBG("i2c probe ok\n");
	return 0;

 probe_failure:
	kfree(mt9t012vc_sensorw);
	mt9t012vc_sensorw = NULL;
	pr_err("i2c probe failure %d\n", rc);
	return rc;
}

static const struct i2c_device_id mt9t012vc_i2c_id[] = {
	{"mt9t012vc", 0},
	{}
};

static struct i2c_driver mt9t012vc_i2c_driver = {
	.id_table = mt9t012vc_i2c_id,
	.probe = mt9t012vc_i2c_probe,
	.remove = __exit_p(mt9t012vc_i2c_remove),
	.driver = {
		.name = "mt9t012vc",
		},
};

static int mt9t012vc_sensor_probe(const struct msm_camera_sensor_info
				*info, struct msm_sensor_ctrl *s)
{
	/* We expect this driver to match with the i2c device registered
	* in the board file immediately. */
	int rc = i2c_add_driver(&mt9t012vc_i2c_driver);
	if (rc < 0 || mt9t012vc_client == NULL) {
		DLINE;
		rc = -ENOTSUPP;
		goto probe_done;
	}

	/* enable mclk first */
	msm_camio_clk_rate_set(MT9T012VC_DEFAULT_CLOCK_RATE);
	mdelay(20);

	rc = mt9t012vc_probe_init_sensor(info);
	if (rc < 0) {
		DLINE;
		i2c_del_driver(&mt9t012vc_i2c_driver);
		goto probe_done;
	}

	s->s_init = mt9t012vc_sensor_open_init;
	s->s_release = mt9t012vc_sensor_release;
	s->s_config = mt9t012vc_sensor_config;
	mt9t012vc_sensor_init_done(info);

 probe_done:
	return rc;
}

static int __mt9t012vc_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, mt9t012vc_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __mt9t012vc_probe,
	.driver = {
		.name = "msm_camera_mt9t012vc",
		.owner = THIS_MODULE,
		},
};

static int __init mt9t012vc_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(mt9t012vc_init);


/******************************************************************************
* DebugFS								      *
******************************************************************************/
#if defined(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#include <asm/io.h>
static struct reg_struct* dbg_capture_pat = &mt9t012vc_preview_reg_pat;

static int set_x_end(void *data, u64 val)
{
	dbg_capture_pat->x_addr_end = val;
	return 0;
}

static int set_x_start(void *data, u64 val)
{
	dbg_capture_pat->x_addr_start = val;
	return 0;
}

static int set_y_end(void *data, u64 val)
{
	dbg_capture_pat->y_addr_end = val;
	return 0;
}

static int set_y_start(void *data, u64 val)
{
	dbg_capture_pat->y_addr_start = val;
	return 0;
}

static int set_len_pck(void *data, u64 val)
{
	dbg_capture_pat->line_length_pck = val;
	return 0;
}

static int set_len_line(void *data, u64 val)
{
	dbg_capture_pat->frame_length_lines = val;
	return 0;
}

static int set_coarse(void *data, u64 val)
{
	dbg_capture_pat->coarse_int_time = val;
	return 0;
}

static int get_x_end(void *data, u64 * val)
{
	*val = dbg_capture_pat->x_addr_end;
	return 0;
}

static int get_y_end(void *data, u64 * val)
{
	*val = dbg_capture_pat->y_addr_end;
	return 0;
}

static int get_x_start(void *data, u64 * val)
{
	*val = dbg_capture_pat->x_addr_start;
	return 0;
}

static int get_y_start(void *data, u64 * val)
{
	*val = dbg_capture_pat->y_addr_start;
	return 0;
}

static int get_len_pck(void *data, u64 * val)
{
	*val = dbg_capture_pat->line_length_pck;
	return 0;
}

static int get_len_line(void *data, u64 * val)
{
	*val = dbg_capture_pat->frame_length_lines;
	return 0;
}

static int get_coarse(void *data, u64 * val)
{
	*val = dbg_capture_pat->coarse_int_time;
	return 0;
}

static int reg_set(void *data, u64 val)
{
	if (val) {
		dbg_capture_pat = &mt9t012vc_capture_reg_pat;
	}
	else {
		dbg_capture_pat = &mt9t012vc_preview_reg_pat;
	}
	return 0;
}

static int reg_get(void *data, u64 * val)
{
	*val = dbg_capture_pat == &mt9t012vc_capture_reg_pat;
	return 0;
}

static bool vdd = false;

static int vdd_set(void *data, u64 val)
{
	vdd = !!val;
	return 0;
}

static int vdd_get(void *data, u64 * val)
{
	*val = !!vdd;
	return 0;
}

static int focus_set(void *data, u64 val)
{
	volatile char* focus;
	mt9t012vc_ctrl->curr_lens_pos = val;
	focus = ioremap(0xa9d00000, 0x1000);

	if (vdd)
		kovsky_af_vdd(0);
	gpio_direction_output(0x6b, 0);
	gpio_direction_output(0x1c, 0);
	if (vdd)
		kovsky_af_vdd(1);
	writel(mt9t012vc_ctrl->curr_lens_pos, focus + 0x54);
	iounmap(focus);
	gpio_direction_output(0x6b, 1);
	gpio_direction_output(0x1c, 1);

	return 0;
}

static int focus_get(void *data, u64 * val)
{
	*val = mt9t012vc_ctrl->curr_lens_pos;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(x_start_ops, get_x_start, set_x_start, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(x_end_ops, get_x_end, set_x_end, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(y_start_ops, get_y_start, set_y_start, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(y_end_ops, get_y_end, set_y_end, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(len_pck_ops, get_len_pck, set_len_pck, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(len_line_ops, get_len_line, set_len_line, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(coarse_ops, get_coarse, set_coarse, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(reg_ops, reg_get, reg_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(focus_ops, focus_get, focus_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(vdd_ops, vdd_get, vdd_set, "%llu\n");



static int __init mt9t012vc_dbg_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("mt9t012vc", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("x_addr_start", 0644, dent, NULL, &x_start_ops);
	debugfs_create_file("x_addr_end", 0644, dent, NULL, &x_end_ops);
	debugfs_create_file("y_addr_start", 0644, dent, NULL, &y_start_ops);
	debugfs_create_file("y_addr_end", 0644, dent, NULL, &y_end_ops);
	debugfs_create_file("frame_length_pck", 0644, dent, NULL, &len_pck_ops);
	debugfs_create_file("frame_length_lines", 0644, dent, NULL, &len_line_ops);
	debugfs_create_file("coarse_int_time", 0644, dent, NULL, &coarse_ops);
	debugfs_create_file("capture_if_one", 0644, dent, NULL, &reg_ops);
	debugfs_create_file("vdd", 0644, dent, NULL, &vdd_ops);
	debugfs_create_file("focus", 0644, dent, NULL, &focus_ops);

	return 0;
}

device_initcall(mt9t012vc_dbg_init);
#endif
