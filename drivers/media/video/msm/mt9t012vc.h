/*
 * Copyright (C) 2008-2009 QUALCOMM Incorporated.
 */

#ifndef MT9T012VC_H
#define MT9T012VC_H

#include <linux/types.h>

struct reg_struct {
	uint16_t vt_pix_clk_div;
	uint16_t vt_sys_clk_div;
	uint16_t pre_pll_clk_div;
	uint16_t pll_multiplier;
	uint16_t op_pix_clk_div;
	uint16_t op_sys_clk_div;
	uint16_t x_addr_start;
	uint16_t x_addr_end;
	uint16_t y_addr_start;
	uint16_t y_addr_end;
	uint16_t read_mode;
	uint16_t x_output_size;
	uint16_t y_output_size;
	uint16_t line_length_pck;
	uint16_t frame_length_lines;
	uint16_t coarse_int_time;
};

struct mt9t012vc_i2c_reg_conf {
	unsigned short waddr;
	unsigned short wdata;
};

struct mt9t012vc_reg {
	struct reg_struct * preview_reg_pat;
	struct reg_struct *capture_reg_pat;
	const struct mt9t012vc_i2c_reg_conf *const ttbl;
	uint16_t ttbl_size;
	const struct mt9t012vc_i2c_reg_conf *const lctbl;
	uint16_t lctbl_size;
	const struct mt9t012vc_i2c_reg_conf *const rftbl;
	uint16_t rftbl_size;
};

#define MT9T012VC_REG_MODEL_ID		0x0000
#define MT9T012VC_MODEL_ID		0x1600

#define REG_MODE_SELECT			0x0100
#define REG_IMAGE_ORIENTATION		0x0101
#define REG_SOFTWARE_RESET		0x0103
#define REG_GROUPED_PARAMETER_HOLD	0x0104
#define GROUPED_PARAMETER_HOLD		0x0100
#define GROUPED_PARAMETER_UPDATE	0x0000

#define REG_MASK_CORRUPTED_FRAMES 0x105
#define REG_GAIN_MODE 0x120

#define REG_FINE_INT_TIME		0x0200
#define REG_COARSE_INT_TIME 0x0202

#define REG_VT_PIX_CLK_DIV		0x0300
#define REG_VT_SYS_CLK_DIV		0x0302
#define REG_PRE_PLL_CLK_DIV		0x0304
#define REG_PLL_MULTIPLIER		0x0306
#define REG_OP_PIX_CLK_DIV		0x0308
#define REG_OP_SYS_CLK_DIV		0x030A
#define REG_SCALE_M				0x0404
#define REG_FRAME_LENGTH_LINES	0x0340
#define REG_LINE_LENGTH_PCK		0x0342

#define REG_X_ADDR_START		0x0344
#define REG_Y_ADDR_START		0x0346
#define REG_X_ADDR_END			0x0348
#define REG_Y_ADDR_END			0x034A
#define REG_X_OUTPUT_SIZE		0x034C
#define REG_Y_OUTPUT_SIZE		0x034E
#define REG_X_ODD_INC			0x0382
#define REG_Y_ODD_INC			0x0386

#define REG_SCALING_MODE		0x0400
#define REG_SPATIAL_SAMPLING  0x0402
#define REG_SCALE_M			0x0404

#define REG_ROW_SPEED			0x3016
#define MT9T012VC_REG_RESET_REGISTER	0x301A
#define MT9T012VC_RESET_REGISTER_PWON	0x18CC
#define MT9T012VC_RESET_REGISTER_PWOFF	0x18C0

#define REG_PIXEL_ORDER			0x3024
#define REG_READ_MODE			0x3040
#define REG_GLOBAL_GAIN			0x305E

#define REG_DATAPATH_STATUS		0x306A
#define REG_DATAPATH_SELECT		0x306E

#define REG_TEST_PATTERN_MODE		0x3070

/*
commented are original values left in case someone reimplements the camera
library to work properly with arbitary capture sizes
frame_length_lines and coarse_int_time were chosen so as to get rid of
flicker and dark images when filming in direct light or on the border of
light and darkness. in some cases that may fail and one will need to set
software exposure via the userspace app
*/
struct reg_struct const mt9t012vc_preview_reg_pat = {
	.vt_pix_clk_div = 5,
	.vt_sys_clk_div = 2,
	.pre_pll_clk_div = 3,
	.pll_multiplier = 80,
	.op_pix_clk_div = 10,
	.op_sys_clk_div = 1,
	.x_addr_start = 4,
	.x_addr_end = 2049, //2089
	.y_addr_start = 4,
	.y_addr_end = 1537, //1553
	.read_mode = 0x6C,
	.x_output_size = 1024, //1044
	.y_output_size = 768, //774
	.line_length_pck = 1708,
	.frame_length_lines = 4126, //1249
	.coarse_int_time = 4125, //1213
};

/*
higher values are used for frame_length_lines and coarse_int_time
during capture to increase exposure time
this allows to use camera in direct light and removes artifacts
like overdarkened image or purple tint
*/
struct reg_struct mt9t012vc_capture_reg_pat = {
	.vt_pix_clk_div = 5,
	.vt_sys_clk_div = 2,
	.pre_pll_clk_div = 3,
	.pll_multiplier = 80,
	.op_pix_clk_div = 10,
	.op_sys_clk_div = 1,
	.x_addr_start = 0,
	.x_addr_end = 2063,
	.y_addr_start = 0,
	.y_addr_end = 1543, //1541 in winmo.. but that fubars htc library
	.read_mode = 0x24,
	.x_output_size = 2064,
	.y_output_size = 1544, //1542. but you know, htc library is very picky
	.line_length_pck = 2748,
	.frame_length_lines = 11445, //1557
	.coarse_int_time = 11444, //4379
};

struct mt9t012vc_i2c_reg_conf const mt9t012vc_test_tbl[] = {
	{0x3044, 0x0544 & 0xFBFF},
	{0x30CA, 0x0004 | 0x0001},
	{0x30D4, 0x9020 & 0x7FFF},
	{0x31E0, 0x0003 & 0xFFFE},
	{0x3180, 0x91FF & 0x7FFF},
	{0x301A, (0x10CC | 0x8000) & 0xFFF7},
	{0x301E, 0x0000},
	{0x3780, 0x0000},
};

/* [Lens shading 85 Percent TL84] */
struct mt9t012vc_i2c_reg_conf const mt9t012vc_lc_tbl[] = {
	{0x3600, 0x2969},
	{0x3602, 0x2030},
	{0x3604, 0x5010},
	{0x3606, 0x7060},
	{0x3608, 0x1824},
	{0x360a, 0x3c0c},
	{0x360c, 0x5448},
	{0x360e, 0x3040},
	{0x3610, 0x179},
	{0x3618, 0x112},
	{0x3620, 0xad6},
	{0x3628, 0xbaa},
	{0x3632, 0x515},
	{0x363a, 0x2f29},
	{0x3642, 0x3d35},
	{0x364a, 0x3e41},
	{0x3652, 0x3d40},
	{0x365a, 0x4237},
	{0x3662, 0x343e},
	{0x366a, 0x27d9},
	{0x3686, 0x458},
	{0x368e, 0x440},
	{0x3676, 0x438},
	{0x367e, 0x440},
	{0x3612, 0x1fb},
	{0x361a, 0x120},
	{0x3622, 0xa93},
	{0x362a, 0xb64},
	{0x3634, 0x5e6},
	{0x363c, 0x323c},
	{0x3644, 0x4440},
	{0x364c, 0x4547},
	{0x3654, 0x3d44},
	{0x365c, 0x423e},
	{0x3664, 0x3541},
	{0x366c, 0x28d7},
	{0x3688, 0x43f},
	{0x3690, 0x47d},
	{0x3678, 0x498},
	{0x3680, 0x28},
	{0x3616, 0x178},
	{0x361e, 0xcd},
	{0x3626, 0xb0d},
	{0x362e, 0xd8c},
	{0x3638, 0xd30b},
	{0x3640, 0x3134},
	{0x3648, 0x2d2b},
	{0x3650, 0x3436},
	{0x3658, 0x2e33},
	{0x3660, 0x302b},
	{0x3668, 0x2734},
	{0x3670, 0x1ef1},
	{0x368c, 0x4d8},
	{0x3694, 0x460},
	{0x367c, 0x450},
	{0x3684, 0x488},
	{0x3614, 0x1d0},
	{0x361c, 0x111},
	{0x3624, 0xa36},
	{0x362c, 0xb3a},
	{0x3636, 0x1501},
	{0x363e, 0x373d},
	{0x3646, 0x3b3d},
	{0x364e, 0x4440},
	{0x3656, 0x4548},
	{0x365e, 0x4143},
	{0x3666, 0x3937},
	{0x366e, 0x1ec3},
	{0x368a, 0x43f},
	{0x3692, 0x47d},
	{0x367a, 0x480},
	{0x3682, 0x410},
	{0x3672, 0x0},
	{0x3674, 0x20},
	{0x3696, 0x28}
};

struct mt9t012vc_reg mt9t012vc_regs = {
	.preview_reg_pat = &mt9t012vc_preview_reg_pat,
	.capture_reg_pat = &mt9t012vc_capture_reg_pat,
	.ttbl = &mt9t012vc_test_tbl[0],
	.ttbl_size = ARRAY_SIZE(mt9t012vc_test_tbl),
	.lctbl = &mt9t012vc_lc_tbl[0],
	.lctbl_size = ARRAY_SIZE(mt9t012vc_lc_tbl),
	.rftbl = &mt9t012vc_lc_tbl[0],
	.rftbl_size = ARRAY_SIZE(mt9t012vc_lc_tbl)
};

#endif				//MT9T012VC_H
