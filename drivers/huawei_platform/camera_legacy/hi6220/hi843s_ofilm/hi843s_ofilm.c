
/*
 *  hi843s camera driver source file
 *
 *  CopyRight (C) Hisilicon Co., Ltd.
 *	Author :
 *  Version:  1.2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/videodev2.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <asm/div64.h>
#include "sensor_common.h"

#include "carmel/hi843s_ofilm_carmel.h"
#include <asm/bug.h>
#include <linux/device.h>

#define LOG_TAG "HI843S_OFILM"
/* #define DEBUG_DEBUG 1 */
#include "cam_log.h"
#include "video_config.h"
//#include "../isp/effect.h"
#define HI843S_OFILM_AP_WRITEAE_MODE
#include "effect.h"
#include "cam_util.h"

#define HI843S_OFILM_SLAVE_ADDRESS    0xc0

#define HI843S_OFILM_CHIP_ID_REG_H        0x0F17
#define HI843S_OFILM_CHIP_ID_REG_L        0x0F16
#define HI843S_OFILM_CHIP_ID                    0x0843

#define HI843S_OFILM_FLIP_MIRROR_REG         0x0000 //bit[0] mirror;bit[1] flip

#define HI843S_DGAIN_GR_REG_H     0x0508
#define HI843S_DGAIN_GR_REG_L     0x0509
#define HI843S_DGAIN_GB_REG_H     0x050A
#define HI843S_DGAIN_GB_REG_L     0x050B
#define HI843S_DGAIN_R_REG_H      0x050C
#define HI843S_DGAIN_R_REG_L      0x050D
#define HI843S_DGAIN_B_REG_H      0x050E
#define HI843S_DGAIN_B_REG_L      0x050F

#define HI843S_OFILM_ANALOG_GAIN_REG      0x003b
#define HI843S_OFILM_MAX_ANALOG_GAIN      256//(240/16+1)*16

#define HI843S_OFILM_EXPOSURE_REG_L       0x0005
#define HI843S_OFILM_EXPOSURE_REG_H       0x0004
#define HI843S_OFILM_EXPOSURE_REG_HW      0x0057

#define HI843S_OFILM_VTS_REG_L        0x0007
#define HI843S_OFILM_VTS_REG_H        0x0006

#define HI843S_MODULE_HUAWEI_ID 0xC8 //23060200

#define HI843S_OFILM_EQUIVALENT_FOCUS            22 // 22mm

/* camera sensor override parameters, define in binning preview mode */
#define HI843S_OFILM_MAX_ISO            800
#define HI843S_OFILM_MIN_ISO            100

#define HI843S_OFILM_APERTURE_FACTOR              240 //F2.4

#define HI843S_OFILM_AUTOFPS_GAIN_LOW2MID            0x16
#define HI843S_OFILM_AUTOFPS_GAIN_MID2HIGH            0x16
#define HI843S_OFILM_AUTOFPS_GAIN_HIGH2MID            0x40
#define HI843S_OFILM_AUTOFPS_GAIN_MID2LOW            0x40

#define HI843S_OFILM_MAX_FRAMERATE            30
#define HI843S_OFILM_MIDDLE_FRAMERATE       15
#define HI843S_OFILM_MIN_FRAMERATE            15

#define HI843S_OFILM_MIN_CAP_FRAMERATE      8

#define HI843S_OFILM_FLASH_TRIGGER_GAIN      0x80

#define HI843S_OFILM_SHARPNESS_PREVIEW      0x10
#define HI843S_OFILM_SHARPNESS_CAPTURE      0x10

#define HI843S_OFILM_ZSL    (0x00)//(1 << CAMERA_ZSL_OFF)

#define HI843S_OFILM_OTP_FEATURE 0
#define HI843S_ID_INFO_NUM     5
#define HI843S_LSC_INFO_NUM    858
#define HI843S_AWB_INFO_NUM    6
#define HI843S_OFILM_I2C_RETRY_TIMES      5

static effect_params effect_hi843s_ofilm= {
    #include "carmel/effect_hi843s_ofilm_carmel.h"
};

static camera_capability hi843s_ofilm_cap[] = {
    {V4L2_CID_FOCAL_LENGTH, 238},//2.38mm
    {V4L2_CID_ZSL,HI843S_OFILM_ZSL},
    {V4L2_CID_HORIZONTAL_VIEW_ANGLE, 9800},//add FOV angel
    {V4L2_CID_VERTICAL_VIEW_ANGLE, 9800},

};

/*should be calibrated, three lights, from 0x1c264*/
/*here is long exposure*/
char hi843s_ofilm_lensc_param[86 * 3] = {
};

/*should be calibrated, 6 groups 3x3, from 0x1c1d8*/
short hi843s_ofilm_ccm_param[54] = {
};

char hi843s_ofilm_awb_param[] = {
};

/*y36721 todo*/
char hi843s_ofilm_awb_param_short[] = {
};

static const struct _sensor_reg_t hi843s_ofilm_stream_off_regs[] = {
      {0x0a00, 0x00},
};
static const struct _sensor_reg_t hi843s_ofilm_stream_on_regs[] = {
//    {0x0a00, 0x01},
};

static sensor_setting_t hi843s_ofilm_carmel_init_array[] = {
      {hi843s_ofilm_init_regs, ARRAY_SIZE(hi843s_ofilm_init_regs)},
};
static framesize_s hi843s_ofilm_framesizes[] = {
        //1600x1200 29.7fps
        {0, 0, 1600, 1200, 3800, 1276, 30, 30, 0x17b, 0x13c, 0x100, VIEW_FULL, RESOLUTION_4_3, false, false, ECGC_TYPE_MAX, {hi843s_ofilm_framesize_quarter, ARRAY_SIZE(hi843s_ofilm_framesize_quarter)}, CLK_350M},
        //3264x2448 15.1fps
        {0, 0, 3264, 2448, 3800, 2524, 15, 15, 0x17d, 0x13e, 0x101, VIEW_FULL, RESOLUTION_4_3, false, false, ECGC_TYPE_BSHUTTER_LONG, {hi843s_ofilm_framesize_full, ARRAY_SIZE(hi843s_ofilm_framesize_full)}, CLK_750M },
};

static const sensor_config_s hi843s_ofilm_config_settings[]= {
    {
        "carmel",
        {hi843s_ofilm_carmel_init_array,  ARRAY_SIZE(hi843s_ofilm_carmel_init_array)},
        {hi843s_ofilm_framesizes, ARRAY_SIZE(hi843s_ofilm_framesizes)},
        &effect_hi843s_ofilm,
        NULL,
    },
};
static const sensor_config_s* hi843s_ofilm_config = hi843s_ofilm_config_settings;
static camera_sensor hi843s_ofilm_sensor;

static void hi843s_ofilm_set_default(void);
void hi843s_ofilm_set_vts(u16 vts);
static void hi843s_ofilm_config_dphy_clk(camera_state state);
static void hi843s_ofilm_reset_dphy(void);

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_read_reg;
 * Description : read hi843s reg by i2c;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi843s_ofilm_read_reg(u16 reg, u8 *val)
{
	return k3_ispio_read_reg(hi843s_ofilm_sensor.i2c_config.index,
				 hi843s_ofilm_sensor.i2c_config.addr, reg, (u16*)val, hi843s_ofilm_sensor.i2c_config.val_bits,hi843s_ofilm_sensor.i2c_config.addr_bits);
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_write_reg;
 * Description : write hi843s reg by i2c;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi843s_ofilm_write_reg(u16 reg, u8 val, u8 mask)
{
	return k3_ispio_write_reg(hi843s_ofilm_sensor.i2c_config.index,
			hi843s_ofilm_sensor.i2c_config.addr, reg, val, hi843s_ofilm_sensor.i2c_config.val_bits, mask,hi843s_ofilm_sensor.i2c_config.addr_bits);
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_write_seq;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int hi843s_ofilm_write_seq(const struct _sensor_reg_t *seq, u32 size, u8 mask)
{
       print_debug("Enter %s, seq[%#x], size=%d", __func__, (int)seq, size);

       return k3_ispio_write_seq(hi843s_ofilm_sensor.i2c_config.index,
       hi843s_ofilm_sensor.i2c_config.addr, seq, size, hi843s_ofilm_sensor.i2c_config.val_bits, mask,hi843s_ofilm_sensor.i2c_config.addr_bits);

}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_write_isp_seq;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void hi843s_ofilm_write_isp_seq(const struct isp_reg_t *seq, u32 size)
{
       print_debug("Enter %s, seq[%#x], size=%d", __func__, (int)seq, size);
       k3_ispio_write_isp_seq(seq, size);
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_enum_frame_intervals;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi843s_ofilm_enum_frame_intervals(struct v4l2_frmivalenum *fi)
{
	assert(fi);

	if(NULL == fi) {
		return -EINVAL;
	}
	print_debug("enter %s", __func__);
	if (fi->index >= CAMERA_MAX_FRAMERATE) {
		return -EINVAL;
	}

	fi->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fi->discrete.numerator = 1;
	fi->discrete.denominator = (fi->index + 1);
	return 0;
}

static int hi843s_ofilm_get_capability(u32 id, u32 *value)
{
	int i;
	for (i = 0; i < sizeof(hi843s_ofilm_cap) / sizeof(hi843s_ofilm_cap[0]); ++i) {
		if (id == hi843s_ofilm_cap[i].id) {
			*value = hi843s_ofilm_cap[i].value;
			break;
		}
	}
	return 0;
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_get_format;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi843s_ofilm_get_format(struct v4l2_fmtdesc *fmt)
{
	if (fmt->type == V4L2_BUF_TYPE_VIDEO_OVERLAY)
		fmt->pixelformat = V4L2_PIX_FMT_RAW10;
	else
		fmt->pixelformat = V4L2_PIX_FMT_RAW10;
	return 0;
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_enum_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi843s_ofilm_enum_framesizes(struct v4l2_frmsizeenum *framesizes)
{
	u32 max_index = ARRAY_SIZE(camera_framesizes) - 1;
	u32 this_max_index = ARRAY_SIZE(hi843s_ofilm_framesizes) - 1;

	assert(framesizes);
	if(NULL == framesizes) {
		return -EINVAL;
	}

	print_debug("enter %s;", __func__);

	if (framesizes->index > max_index) {
		print_error("framesizes->index = %d error", framesizes->index);
		return -EINVAL;
	}

	if ((camera_framesizes[framesizes->index].width > hi843s_ofilm_framesizes[this_max_index].width)
		|| (camera_framesizes[framesizes->index].height > hi843s_ofilm_framesizes[this_max_index].height)) {
		print_error("framesizes->index = %d error", framesizes->index);
		return -EINVAL;
	}

	framesizes->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	framesizes->discrete.width = hi843s_ofilm_framesizes[this_max_index].width;
	framesizes->discrete.height = hi843s_ofilm_framesizes[this_max_index].height;

	return 0;
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_try_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi843s_ofilm_try_framesizes(struct v4l2_frmsizeenum *framesizes)
{
	int max_index = ARRAY_SIZE(hi843s_ofilm_framesizes) - 1;

	assert(framesizes);

	if(NULL == framesizes) {
		return -EINVAL;
	}

	print_debug("Enter Function:%s ", __func__);

	if (framesizes->discrete.width <= hi843s_ofilm_framesizes[max_index].width
		&& framesizes->discrete.height <= hi843s_ofilm_framesizes[max_index].height) {
		print_debug("===========width = %d", framesizes->discrete.width);
		print_debug("===========height = %d", framesizes->discrete.height);
		return 0;
	}

	print_error("frame size too large, [%d,%d]",
			framesizes->discrete.width, framesizes->discrete.height);
	return -EINVAL;
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_set_framesizes;
 * Description : NA;
 * Input       : flag: if 1, set framesize to sensor,
 *					   if 0, only store framesize to camera_interface;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi843s_ofilm_set_framesizes(camera_state state,
				 struct v4l2_frmsize_discrete *fs, int flag, camera_setting_view_type view_type, bool zsl_preview, camera_b_shutter_mode b_shutter_mode,ecgc_support_type_s ecgc_type)
{
	int i = 0;
	bool match = false;
	assert(fs);

	if(NULL == fs) {
		return -EINVAL;
	}
     (void)b_shutter_mode;//front sensor does not support b_shutter_mode
     (void)ecgc_type;

	print_debug("Enter Function:%s State(%d), flag=%d, width=%d, height=%d, b_shutter_mode=0x%x, ecgc_type=0x%x",
		    __func__, state, flag, fs->width, fs->height,b_shutter_mode,ecgc_type);

	if (VIEW_FULL == view_type) {
		for (i = 0; i < ARRAY_SIZE(hi843s_ofilm_framesizes); i++) {
			if ((hi843s_ofilm_framesizes[i].width >= fs->width)
				&& (hi843s_ofilm_framesizes[i].height >= fs->height)
				&& (VIEW_FULL == hi843s_ofilm_framesizes[i].view_type)
				&& (camera_get_resolution_type(fs->width, fs->height)
				<= hi843s_ofilm_framesizes[i].resolution_type)) {
				fs->width = hi843s_ofilm_framesizes[i].width;
				fs->height = hi843s_ofilm_framesizes[i].height;
				match = true;
				break;
			}
		}
	}

	if (false == match) {
		for (i = 0; i < ARRAY_SIZE(hi843s_ofilm_framesizes); i++) {
			if ((hi843s_ofilm_framesizes[i].width >= fs->width)
				&& (hi843s_ofilm_framesizes[i].height >= fs->height)
				&& (camera_get_resolution_type(fs->width, fs->height)
				<= hi843s_ofilm_framesizes[i].resolution_type)) {
				fs->width = hi843s_ofilm_framesizes[i].width;
				fs->height = hi843s_ofilm_framesizes[i].height;
				break;
			}
		}
	}

	if (i >= ARRAY_SIZE(hi843s_ofilm_framesizes)) {
		print_error("request resolution larger than sensor's max resolution");
		return -EINVAL;
	}

	if (state == STATE_PREVIEW)
		hi843s_ofilm_sensor.preview_frmsize_index = i;
	else
		hi843s_ofilm_sensor.capture_frmsize_index = i;

	return 0;
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_get_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi843s_ofilm_get_framesizes(camera_state state,
				 struct v4l2_frmsize_discrete *fs)
{
	int frmsize_index;
	assert(fs);

	if(NULL == fs) {
		return -EINVAL;
	}

	if (state == STATE_PREVIEW)
		frmsize_index = hi843s_ofilm_sensor.preview_frmsize_index;
	else if (state == STATE_CAPTURE)
		frmsize_index = hi843s_ofilm_sensor.capture_frmsize_index;
	else
		return -EINVAL;
	fs->width = hi843s_ofilm_framesizes[frmsize_index].width;
	fs->height = hi843s_ofilm_framesizes[frmsize_index].height;

	return 0;
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_init_reg;
 * Description : download initial seq for sensor init;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi843s_ofilm_init_reg(void)
{
	int size = 0;

	print_info("Enter Function:%s  , initsize=%d", __func__, sizeof(hi843s_ofilm_init_regs));

	if (0 != k3_ispio_init_csi(hi843s_ofilm_sensor.mipi_index,
		 hi843s_ofilm_sensor.mipi_lane_count, hi843s_ofilm_sensor.lane_clk)) {
		return -EFAULT;
	}

	mdelay(3);

	hi843s_ofilm_sensor.i2c_config.val_bits = I2C_16BIT;

	size = ARRAY_SIZE(hi843s_ofilm_init_regs);
	if (0 != hi843s_ofilm_write_seq(hi843s_ofilm_init_regs, size, 0x00)) {
		print_error("fail to init hi843s sensor");
			hi843s_ofilm_sensor.i2c_config.val_bits = I2C_8BIT;
		return -EFAULT;
	}
	hi843s_ofilm_sensor.i2c_config.val_bits = I2C_8BIT;

	return 0;
}

static int hi843s_ofilm_get_sensor_aperture(void)
{
	return HI843S_OFILM_APERTURE_FACTOR;
}

static int hi843s_ofilm_set_hflip(int flip)
{
	print_debug("enter %s flip=%d", __func__, flip);
	hi843s_ofilm_sensor.hflip = flip;
	return 0;
}

static int hi843s_ofilm_set_vflip(int flip)
{
	print_debug("enter %s flip=%d", __func__, flip);
	hi843s_ofilm_sensor.vflip = flip;
	return 0;
}

static int hi843s_ofilm_get_hflip(void)
{
	print_debug("enter %s flip=%d", __func__, hi843s_ofilm_sensor.hflip);
	return hi843s_ofilm_sensor.hflip;
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofolm_dump_reg_debug;
 * Description : dump standby, frame count, cap relate reg for debug
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/

static void hi843s_ofilm_dump_reg_debug(void)
{
     u16 reg = 0;
     u8 val = 0;

     reg=0x0004;hi843s_ofilm_read_reg(reg,&val);print_info("0x%0x=0x%0x", reg, val);
     reg=0x0005;hi843s_ofilm_read_reg(reg,&val);print_info("0x%0x=0x%0x", reg, val);
     reg=0x0006;hi843s_ofilm_read_reg(reg,&val);print_info("0x%0x=0x%0x", reg, val);
     reg=0x0007;hi843s_ofilm_read_reg(reg,&val);print_info("0x%0x=0x%0x", reg, val);
     reg=0x0008;hi843s_ofilm_read_reg(reg,&val);print_info("0x%0x=0x%0x", reg, val);
     reg=0x0009;hi843s_ofilm_read_reg(reg,&val);print_info("0x%0x=0x%0x", reg, val);
     reg=0x003b;hi843s_ofilm_read_reg(reg,&val);print_info("0x%0x=0x%0x", reg, val);

}

static int hi843s_ofilm_get_vflip(void)
{
	print_debug("enter %s flip=%d", __func__, hi843s_ofilm_sensor.vflip);
	return hi843s_ofilm_sensor.vflip;
}

static int hi843s_ofilm_update_flip(u16 width, u16 height)
{
	u8 new_flip = ((hi843s_ofilm_sensor.vflip << 1) | hi843s_ofilm_sensor.hflip);
	print_info("Enter %s", __func__);

	if(hi843s_ofilm_sensor.old_flip != new_flip) {
	k3_ispio_update_flip((hi843s_ofilm_sensor.old_flip ^ new_flip) & 0x03, width, height, PIXEL_ORDER_CHANGED);

	hi843s_ofilm_sensor.old_flip = new_flip;

	hi843s_ofilm_write_reg(HI843S_OFILM_FLIP_MIRROR_REG, hi843s_ofilm_sensor.vflip ? 0x00 : 0x02, ~0x02);
	hi843s_ofilm_write_reg(HI843S_OFILM_FLIP_MIRROR_REG, hi843s_ofilm_sensor.hflip ? 0x01 : 0x00, ~0x01);

	}
	return 0;
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_framesize_switch;
 * Description : switch frame size, used by preview and capture
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi843s_ofilm_framesize_switch(camera_state state)
{
	u8 next_frmsize_index;

	print_info("Enter Function: %s, state =%d", __func__, state);
	if (state == STATE_PREVIEW)
		next_frmsize_index = hi843s_ofilm_sensor.preview_frmsize_index;
	else
		next_frmsize_index = hi843s_ofilm_sensor.capture_frmsize_index;

	print_info("Enter Function:%s frm index=%d", __func__, next_frmsize_index);

	if (next_frmsize_index >= ARRAY_SIZE( hi843s_ofilm_framesizes )){
		print_error("Unsupport sensor setting index: %d",next_frmsize_index);
		return -ETIME;
	}

	if (0 != hi843s_ofilm_write_seq( hi843s_ofilm_sensor.frmsize_list[next_frmsize_index].sensor_setting.setting
		, hi843s_ofilm_sensor.frmsize_list[next_frmsize_index].sensor_setting.seq_size, 0x00)) {
		print_error("fail to init hi843s sensor");
		return -ETIME;
	}

	if(NULL != hi843s_ofilm_sensor.stream_on_setting)
	{
		if (0 != hi843s_ofilm_write_seq(hi843s_ofilm_sensor.stream_on_setting, hi843s_ofilm_sensor.stream_on_setting_size, 0x00)) {
			print_error("%s fail to stream on hi843s sensor", __func__);
			return -ETIME;
		}
	}

    return 0;
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_start_preview;
 * Description : download preview seq for sensor preview;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi843s_ofilm_stream_on(camera_state state)
{
    int ret = 0;

    print_info("Enter Function:%s  , preview frmsize index=%d",
            __func__, hi843s_ofilm_sensor.preview_frmsize_index);

    hi843s_ofilm_config_dphy_clk(state);

    ret = hi843s_ofilm_framesize_switch(state);
    print_info("Enter Function:%s  , state=%d",__func__, state);
    if (ret != 0)
    {
        print_warn("%s,hi843s_ofilm_framesize_switch fail.ret=%d.",__func__,ret);
        return ret;
    }

    return 0;
}

/*  **************************************************************************
* FunctionName: hi843s_ofilm_check_sensor;
* Description : NA;
* Input       : NA;
* Output      : NA;
* ReturnValue : NA;
* Other       : NA;
***************************************************************************/
static int hi843s_ofilm_check_sensor(void)
{
	u16 chip_id = 0;
	u8 idh = 0;
	u8 idl = 0;
	int retry = 0;

    for(retry = 0; retry < HI843S_OFILM_I2C_RETRY_TIMES; retry++) {
		hi843s_ofilm_read_reg(HI843S_OFILM_CHIP_ID_REG_L, &idl);
		hi843s_ofilm_read_reg(HI843S_OFILM_CHIP_ID_REG_H, &idh);
		chip_id = (idh<<8)|idl;

		print_info("hi843s product id:0x%x ,retrytimes:%d", chip_id, retry);
		if(HI843S_OFILM_CHIP_ID == chip_id) {
                  break;
             }
        udelay(100);
    }

	if (HI843S_OFILM_CHIP_ID != chip_id) {
		print_error("Invalid product id ,Could not load sensor hi843s");
		return -1;
	}

	return 0;
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_init_isp_reg;
 * Description : load initial seq for sensor init;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi843s_ofilm_init_isp_reg(void)
{
    int size = 0;

    camera_sensor *sensor = &hi843s_ofilm_sensor;

    size = CAMERA_MAX_SETTING_SIZE;

    hi843s_ofilm_write_isp_seq(sensor->effect->isp_settings, size);

    return 0;
}

/*  **************************************************************************
* FunctionName: hi843s_ofilm_check_sensor;
* Description : NA;
* Input       : NA;
* Output      : NA;
* ReturnValue : NA;
* Other       : NA;
***************************************************************************/
int hi843s_ofilm_power(camera_power_state power)
{
	if (power == POWER_ON) {

		k3_ispldo_power_sensor(power, S_CAMERA_IO_VDD);
		k3_socgpio_power_sensor(1, S_CAMERA_ANALOG_VDD);//for external AVDD LDO
		k3_ispldo_power_sensor(power, S_CAMERA_ANALOG_VDD);
		k3_ispldo_power_sensor(power, M_CAMERA_CORE_VDD);//power on main sensor's DVDD
		udelay(100);
		k3_ispio_ioconfig(&hi843s_ofilm_sensor, power);

		k3_ispgpio_power_sensor(&hi843s_ofilm_sensor, power);

		k3_isp_io_enable_mclk(MCLK_ENABLE, hi843s_ofilm_sensor.sensor_index);
		msleep(3);
		k3_ispgpio_reset_sensor(hi843s_ofilm_sensor.sensor_index, power, hi843s_ofilm_sensor.power_conf.reset_valid);
		msleep(5);

	} else {
		k3_ispio_deinit_csi(hi843s_ofilm_sensor.mipi_index);
		k3_ispgpio_reset_sensor(hi843s_ofilm_sensor.sensor_index, power, hi843s_ofilm_sensor.power_conf.reset_valid);
		k3_ispio_ioconfig(&hi843s_ofilm_sensor, power);
		k3_ispgpio_power_sensor(&hi843s_ofilm_sensor, power);
		udelay(100);
		k3_isp_io_enable_mclk(MCLK_DISABLE, hi843s_ofilm_sensor.sensor_index);

		k3_ispldo_power_sensor(power, M_CAMERA_CORE_VDD);//power off main sensor' DVDD
		k3_ispldo_power_sensor(power, S_CAMERA_ANALOG_VDD);
		k3_socgpio_power_sensor(0, S_CAMERA_ANALOG_VDD);//for external AVDD  LDO
		k3_ispldo_power_sensor(power, S_CAMERA_IO_VDD);
		msleep(5);
	}

	return 0;
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_shut_down;
 * Description : hi843s shut down function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void hi843s_ofilm_shut_down(void)
{
	print_debug("enter %s", __func__);
	k3_ispgpio_power_sensor(&hi843s_ofilm_sensor, POWER_OFF);
}

/*
 * Here gain is in unit 1/16 of sensor gain,
 * y36721 todo, temporarily if sensor gain=0x10, ISO is 100
 * in fact we need calibrate an ISO-ET-gain table.
 */
u32 hi843s_ofilm_gain_to_iso(int gain)
{
	return (gain * 100) / 0x10;
}

u32 hi843s_ofilm_iso_to_gain(int iso)
{
	return (iso * 0x10) / 100;
}

void hi843s_ofilm_set_gain(u32 gain)
{
	u32   tmp_digital_gain = 0;
	u8    digital_ratio = 0;
	u8    digital_h = 0;
	u8    digital_l = 0;
	u32   analog_gain = 0;

	if (gain <= 0x10)//min gain = 16
		gain = 0x10;

    //isp2.2 gain = register_value/0x10
	if(gain > HI843S_OFILM_MAX_ANALOG_GAIN)
	{
        /*tmp_digital_gain*256 is keep the decimal part*/
		tmp_digital_gain = (gain*256) >> 4;
		tmp_digital_gain = (tmp_digital_gain*16)/HI843S_OFILM_MAX_ANALOG_GAIN;

		digital_ratio = gain/HI843S_OFILM_MAX_ANALOG_GAIN;
		digital_h = digital_ratio;
		digital_l = tmp_digital_gain - digital_ratio*256;

		analog_gain = HI843S_OFILM_MAX_ANALOG_GAIN;
	}
	else
	{
		analog_gain = gain;
		digital_h = 1;
		digital_l = 0;
	}

	analog_gain = analog_gain - 16;

	hi843s_ofilm_write_reg(HI843S_OFILM_ANALOG_GAIN_REG, analog_gain & 0xff, 0x00);

	hi843s_ofilm_write_reg(HI843S_DGAIN_GR_REG_H, digital_h & 0x1f, 0x00);
	hi843s_ofilm_write_reg(HI843S_DGAIN_GR_REG_L, digital_l & 0xff, 0x00);
	hi843s_ofilm_write_reg(HI843S_DGAIN_GB_REG_H, digital_h & 0x1f, 0x00);
	hi843s_ofilm_write_reg(HI843S_DGAIN_GB_REG_L, digital_l & 0xff, 0x00);
	hi843s_ofilm_write_reg(HI843S_DGAIN_R_REG_H, digital_h & 0x1f, 0x00);
	hi843s_ofilm_write_reg(HI843S_DGAIN_R_REG_L, digital_l & 0xff, 0x00);
	hi843s_ofilm_write_reg(HI843S_DGAIN_B_REG_H, digital_h & 0x1f, 0x00);
	hi843s_ofilm_write_reg(HI843S_DGAIN_B_REG_L, digital_l & 0xff, 0x00);

	return;
}

void hi843s_ofilm_set_exposure(u32 exposure)
{
       if(0 == exposure){
              return;
       }

       exposure >>= 4;
       print_info("%s, set exposure = 0x%x\n",__func__, exposure);

       //hi843s_ofilm_write_reg(HI843S_OFILM_EXPOSURE_REG_HW, (exposure >> 16) & 0x0f, 0x00);
       hi843s_ofilm_write_reg(HI843S_OFILM_EXPOSURE_REG_H, (exposure >> 8) & 0xff, 0x00);
       hi843s_ofilm_write_reg(HI843S_OFILM_EXPOSURE_REG_L, exposure & 0xff, 0x00);	/*fraction part not used */
}

#if 0
static void hi843s_ofilm_set_exposure_gain(u32 exposure, u32 gain)
{
    return;
}
#endif

u32 hi843s_ofilm_get_vts_reg_addr(void)
{
	return HI843S_OFILM_VTS_REG_H;
}

void hi843s_ofilm_set_vts(u16 vts)
{
	print_debug("Enter %s  ", __func__);
	hi843s_ofilm_write_reg(HI843S_OFILM_VTS_REG_H, (vts >> 8) & 0xff, 0x00);
	hi843s_ofilm_write_reg(HI843S_OFILM_VTS_REG_L, vts & 0xff, 0x00);
}

static u32 hi843s_ofilm_get_override_param(camera_override_type_t type)
{
	u32 ret_val = sensor_override_params[type];

	switch (type) {
	case OVERRIDE_ISO_HIGH:
		ret_val = HI843S_OFILM_MAX_ISO;
		break;

	case OVERRIDE_ISO_LOW:
		ret_val = HI843S_OFILM_MIN_ISO;
		break;

	/* increase frame rate gain threshold */
	case OVERRIDE_AUTOFPS_GAIN_LOW2MID:
		ret_val = HI843S_OFILM_AUTOFPS_GAIN_LOW2MID;
		break;
	case OVERRIDE_AUTOFPS_GAIN_MID2HIGH:
		ret_val = HI843S_OFILM_AUTOFPS_GAIN_MID2HIGH;
		break;

	/* reduce frame rate gain threshold */
	case OVERRIDE_AUTOFPS_GAIN_MID2LOW:
		ret_val = HI843S_OFILM_AUTOFPS_GAIN_MID2LOW;
		break;
	case OVERRIDE_AUTOFPS_GAIN_HIGH2MID:
		ret_val = HI843S_OFILM_AUTOFPS_GAIN_HIGH2MID;
		break;

	case OVERRIDE_FPS_MAX:
		ret_val = HI843S_OFILM_MAX_FRAMERATE;
		break;

	case OVERRIDE_FPS_MIN:
		ret_val = HI843S_OFILM_MIN_FRAMERATE;
		break;

       case OVERRIDE_FPS_MID:
             ret_val = HI843S_OFILM_MIDDLE_FRAMERATE;
             break;

	case OVERRIDE_CAP_FPS_MIN:
		ret_val = HI843S_OFILM_MIN_CAP_FRAMERATE;
		break;

	case OVERRIDE_FLASH_TRIGGER_GAIN:
		ret_val = HI843S_OFILM_FLASH_TRIGGER_GAIN;
		break;

	case OVERRIDE_SHARPNESS_PREVIEW:
		ret_val = HI843S_OFILM_SHARPNESS_PREVIEW;
		break;

	case OVERRIDE_SHARPNESS_CAPTURE:
		ret_val = HI843S_OFILM_SHARPNESS_CAPTURE;
		break;

	default:
		print_error("%s:not override or invalid type %d, use default",__func__, type);
		break;
	}

	return ret_val;
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_reset;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int hi843s_ofilm_reset(camera_power_state power_state)
{
	print_debug("%s", __func__);

	hi843s_ofilm_sensor.old_flip = 0;

	return 0;
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_init;
 * Description : hi843s init function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : Error code indicating success or failure;
 * Other       : NA;
 **************************************************************************
*/
static int hi843s_ofilm_init(void)
{
    static bool hi843s_ofilm_check = false;
    int ret = 0;
    print_info("%s", __func__);

    if (false == hi843s_ofilm_check) {
          if (video_check_suspensory_camera("hi843sofilm_sensor") != 1) {
            print_error("%s: product not have this sensor", __func__);
             return -ENODEV;
          }
             print_info("%s:hi843s_ofilm_check ok",__func__);
        hi843s_ofilm_check = true;
        ret = camera_get_matching_sensor_config(hi843s_ofilm_config_settings, ARRAY_SIZE(hi843s_ofilm_config_settings),&hi843s_ofilm_config);
        if( ret < 0 ){
            print_error("%s: fail to match sensor config.", __func__);
            return -ENODEV;
        } else {
            print_info("%s: choose the %s's setting.",__func__,hi843s_ofilm_config->product);
        }
      }

    //S_CAMERA_CORE_VDD and S_CAMERA_IO_VDD use same vdd source, so only init once
    k3_ispio_power_init(S_CAMERA_IO_VDD, LDO_VOLTAGE_18V, LDO_VOLTAGE_18V);    /*IO 1.8V - sec camera*/
    k3_ispio_power_init(S_CAMERA_ANALOG_VDD, LDO_VOLTAGE_28V, LDO_VOLTAGE_28V); /*analog 2.8V - sec camera*/
    k3_ispio_power_init(M_CAMERA_CORE_VDD, LDO_VOLTAGE_12V, LDO_VOLTAGE_12V); /*core 1.2V - sec camera*/

    return 0;
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_exit;
 * Description : hi843s exit function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void hi843s_ofilm_exit(void)
{
    print_debug("enter %s", __func__);

    k3_ispio_power_deinit();

    if (hi843s_ofilm_sensor.owner) {
        module_put(hi843s_ofilm_sensor.owner);
    }
    print_debug("exit %s", __func__);
}

/*
 **************************************************************************
 * FunctionName: sonyimx134_config_dphy_clk;
 * Description : set MIPI clock to dphy;
 * Input       : camera state
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void hi843s_ofilm_config_dphy_clk(camera_state state)
{
    u8 lane_clk;

    if (state == STATE_PREVIEW) {
        lane_clk =hi843s_ofilm_sensor.frmsize_list[hi843s_ofilm_sensor.preview_frmsize_index].lane_clk;
    } else {
        lane_clk =hi843s_ofilm_sensor.frmsize_list[hi843s_ofilm_sensor.capture_frmsize_index].lane_clk;
    }

    print_info("%s lane_clk = 0x%x state = %d",__func__, lane_clk, state);
    k3_ispio_config_lane_clk(hi843s_ofilm_sensor.mipi_index, hi843s_ofilm_sensor.mipi_lane_count, lane_clk);
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_reset_dphy;
 * Description : reset dphy;
 * Input       : index:sensor index; mipi_lane_count: mipi land count;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void hi843s_ofilm_reset_dphy(void)
{
       k3_ispio_reset_phy(hi843s_ofilm_sensor.mipi_index, hi843s_ofilm_sensor.mipi_lane_count);
}

static int hi843s_ofilm_get_equivalent_focus(void)
{
    print_debug("enter %s", __func__);
    return HI843S_OFILM_EQUIVALENT_FOCUS;
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_set_default;
 * Description : init hi843s_ofilm_sensor;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void hi843s_ofilm_set_default(void)
{
	hi843s_ofilm_sensor.init = hi843s_ofilm_init;
	hi843s_ofilm_sensor.exit = hi843s_ofilm_exit;
	hi843s_ofilm_sensor.reset = hi843s_ofilm_reset;
	hi843s_ofilm_sensor.shut_down = hi843s_ofilm_shut_down;
	hi843s_ofilm_sensor.check_sensor = hi843s_ofilm_check_sensor;
	hi843s_ofilm_sensor.power = hi843s_ofilm_power;
	hi843s_ofilm_sensor.init_reg = hi843s_ofilm_init_reg;
	hi843s_ofilm_sensor.stream_on = hi843s_ofilm_stream_on;

	hi843s_ofilm_sensor.get_format = hi843s_ofilm_get_format;
	hi843s_ofilm_sensor.set_flash = NULL;
	hi843s_ofilm_sensor.get_flash = NULL;
	hi843s_ofilm_sensor.set_scene = NULL;
	hi843s_ofilm_sensor.get_scene = NULL;

	hi843s_ofilm_sensor.enum_framesizes = hi843s_ofilm_enum_framesizes;
	hi843s_ofilm_sensor.try_framesizes = hi843s_ofilm_try_framesizes;
	hi843s_ofilm_sensor.set_framesizes = hi843s_ofilm_set_framesizes;
	hi843s_ofilm_sensor.get_framesizes = hi843s_ofilm_get_framesizes;

	hi843s_ofilm_sensor.enum_frame_intervals = hi843s_ofilm_enum_frame_intervals;
	hi843s_ofilm_sensor.try_frame_intervals = NULL;
	hi843s_ofilm_sensor.set_frame_intervals = NULL;
	hi843s_ofilm_sensor.get_frame_intervals = NULL;
	hi843s_ofilm_sensor.get_capability = hi843s_ofilm_get_capability;

    //hi843s_ofilm_sensor.sensor_hw_3a.support_awb_otp = hi843s_ofilm_support_awb_otp;

	strncpy(hi843s_ofilm_sensor.info.name, "hi843s_ofilm", sizeof(hi843s_ofilm_sensor.info.name));
	hi843s_ofilm_sensor.interface_type = MIPI2;
	hi843s_ofilm_sensor.mipi_lane_count = CSI_LINES_2;
	hi843s_ofilm_sensor.mipi_index = CSI_INDEX_1;
	hi843s_ofilm_sensor.sensor_index = CAMERA_SENSOR_SECONDARY;
	hi843s_ofilm_sensor.skip_frames = 3;

	hi843s_ofilm_sensor.capture_skip_frames = CAPTURE_SKIP_2;


	hi843s_ofilm_sensor.power_conf.pd_valid = LOW_VALID;
	hi843s_ofilm_sensor.power_conf.reset_valid = LOW_VALID;
	hi843s_ofilm_sensor.i2c_config.index = I2C_SECONDARY;
	hi843s_ofilm_sensor.i2c_config.speed = I2C_SPEED_400;
	hi843s_ofilm_sensor.i2c_config.addr = HI843S_OFILM_SLAVE_ADDRESS;
	hi843s_ofilm_sensor.i2c_config.addr_bits = I2C_16BIT;
	hi843s_ofilm_sensor.i2c_config.val_bits = I2C_8BIT;

	hi843s_ofilm_sensor.preview_frmsize_index = 0;
	hi843s_ofilm_sensor.capture_frmsize_index = 0;
	hi843s_ofilm_sensor.frmsize_list = hi843s_ofilm_framesizes;
	hi843s_ofilm_sensor.fmt[STATE_PREVIEW] = V4L2_PIX_FMT_RAW10;
	hi843s_ofilm_sensor.fmt[STATE_CAPTURE] = V4L2_PIX_FMT_RAW10;
#ifdef HI843S_OFILM_AP_WRITEAE_MODE
	hi843s_ofilm_sensor.aec_addr[0] = 0;
	hi843s_ofilm_sensor.aec_addr[1] = 0;
	hi843s_ofilm_sensor.aec_addr[2] = 0;
	hi843s_ofilm_sensor.agc_addr[0] = 0;
	hi843s_ofilm_sensor.agc_addr[1] = 0;
	hi843s_ofilm_sensor.ap_writeAE_delay = 0;
#else
	hi843s_ofilm_sensor.aec_addr[0] = 0x3500;
	hi843s_ofilm_sensor.aec_addr[1] = 0x3501;
	hi843s_ofilm_sensor.aec_addr[2] = 0x3502;
	hi843s_ofilm_sensor.agc_addr[0] = 0x350a;
	hi843s_ofilm_sensor.agc_addr[1] = 0x350b;
#endif

	hi843s_ofilm_sensor.sensor_type = SENSOR_OV;

	hi843s_ofilm_sensor.get_vts_reg_addr = hi843s_ofilm_get_vts_reg_addr;
	hi843s_ofilm_sensor.set_vts = hi843s_ofilm_set_vts;

	hi843s_ofilm_sensor.set_gain = hi843s_ofilm_set_gain;
	hi843s_ofilm_sensor.set_exposure = hi843s_ofilm_set_exposure;
	hi843s_ofilm_sensor.set_exposure_gain = NULL;

       hi843s_ofilm_sensor.sensor_dump_reg = hi843s_ofilm_dump_reg_debug;

	hi843s_ofilm_sensor.get_override_param = hi843s_ofilm_get_override_param;

	hi843s_ofilm_sensor.sensor_gain_to_iso = hi843s_ofilm_gain_to_iso;
	hi843s_ofilm_sensor.sensor_iso_to_gain = hi843s_ofilm_iso_to_gain;


	hi843s_ofilm_sensor.get_sensor_aperture = hi843s_ofilm_get_sensor_aperture;

	hi843s_ofilm_sensor.get_equivalent_focus = hi843s_ofilm_get_equivalent_focus;

	hi843s_ofilm_sensor.isp_location = CAMERA_USE_K3ISP;
	hi843s_ofilm_sensor.sensor_tune_ops = NULL;

	hi843s_ofilm_sensor.af_enable = 0;

	hi843s_ofilm_sensor.sensor_gain_to_iso = hi843s_ofilm_gain_to_iso;
	hi843s_ofilm_sensor.sensor_iso_to_gain = hi843s_ofilm_iso_to_gain;
	hi843s_ofilm_sensor.image_setting.lensc_param = hi843s_ofilm_lensc_param;
	hi843s_ofilm_sensor.image_setting.ccm_param = hi843s_ofilm_ccm_param;
	hi843s_ofilm_sensor.image_setting.awb_param = hi843s_ofilm_awb_param;

	/*default is preview size */
	hi843s_ofilm_sensor.fps_max = 30;
	hi843s_ofilm_sensor.fps_min = 16;
	hi843s_ofilm_sensor.fps = 25;

	/*defalt flip*/
	hi843s_ofilm_sensor.vflip			= 0;
	hi843s_ofilm_sensor.hflip			= 0;
	hi843s_ofilm_sensor.old_flip		= 0;
	hi843s_ofilm_sensor.set_vflip		= hi843s_ofilm_set_vflip;
	hi843s_ofilm_sensor.set_hflip		= hi843s_ofilm_set_hflip;
	hi843s_ofilm_sensor.get_vflip		= hi843s_ofilm_get_vflip;
	hi843s_ofilm_sensor.get_hflip		= hi843s_ofilm_get_hflip;
	hi843s_ofilm_sensor.update_flip     = hi843s_ofilm_update_flip;
	hi843s_ofilm_sensor.sensor_rgb_type = SENSOR_GRBG;

	hi843s_ofilm_sensor.owner = THIS_MODULE;


	hi843s_ofilm_sensor.info.facing = CAMERA_FACING_FRONT;
	hi843s_ofilm_sensor.info.orientation = 270;
	hi843s_ofilm_sensor.info.focal_length = 238; /* 2.38mm*/
	hi843s_ofilm_sensor.info.h_view_angle = 75;
	hi843s_ofilm_sensor.info.v_view_angle = 60;

	hi843s_ofilm_sensor.lane_clk = CLK_350M;
	hi843s_ofilm_sensor.effect = hi843s_ofilm_config_settings[0].effect;;
	hi843s_ofilm_sensor.support_summary = false;
	hi843s_ofilm_sensor.init_isp_reg = hi843s_ofilm_init_isp_reg;
	hi843s_ofilm_sensor.isp_idi_skip = false;

	hi843s_ofilm_sensor.stream_off_setting = hi843s_ofilm_stream_off_regs;
	hi843s_ofilm_sensor.stream_off_setting_size = ARRAY_SIZE(hi843s_ofilm_stream_off_regs);

	//hi843s_ofilm_sensor.stream_on_setting = hi843s_ofilm_stream_on_regs;
	//hi843s_ofilm_sensor.stream_on_setting_size = ARRAY_SIZE(hi843s_ofilm_stream_on_regs);


	/*if there have different data rate of sensor resolution we need this config_dphy_clk
       otherwise if all resolution is same rate config_dphy_clk must to be null*/
	hi843s_ofilm_sensor.config_dphy_clk = hi843s_ofilm_config_dphy_clk;
	hi843s_ofilm_sensor.reset_dphy = hi843s_ofilm_reset_dphy;

}


/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_module_init;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static __init int hi843s_ofilm_module_init(void)
{
    hi843s_ofilm_set_default();
    return register_camera_sensor(hi843s_ofilm_sensor.sensor_index, &hi843s_ofilm_sensor);
}

/*
 **************************************************************************
 * FunctionName: hi843s_ofilm_module_exit;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void __exit hi843s_ofilm_module_exit(void)
{
    unregister_camera_sensor(hi843s_ofilm_sensor.sensor_index, &hi843s_ofilm_sensor);
}

MODULE_AUTHOR("Hisilicon");
module_init(hi843s_ofilm_module_init);
module_exit(hi843s_ofilm_module_exit);
MODULE_LICENSE("GPL");

/********************************** END **********************************************/
