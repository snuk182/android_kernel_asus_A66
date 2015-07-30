/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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
 */

#include "msm_sensor.h"
#include "ov2720.h"
#define SENSOR_NAME "ov2720"
#define PLATFORM_DRIVER_NAME "msm_camera_ov2720"
#define ov2720_obj ov2720_##obj

//ASUS_BSP +++ Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
#include <linux/a60k_gpio_pinname.h>
#include <linux/regulator/consumer.h>
#include "msm_ispif.h"	//ASUS_BSP LiJen "[A60K][8M][NA][Fix] add NOTIFY_ISPIF_STREAM v4l2 notify in kernel
#include "msm.h"    //ASUS_BSP Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
#include "fjm6mo.h"
#include <linux/asusdebug.h> //ASUS_BSP LiJen "[A66][8M][NA][Others]add camera power event logs"

#include <linux/gpio.h>

int g_rt_for_monitor = 0;	//ASUS_BSP Stimber "For checking if higher resolution while monitor"
bool g_if_ISP_change_res = 0; //ASUS_BSP Stimber "For checking if ISP change resolution while monitor mode"
static unsigned char g_ov2720_power = false; //ASUS_BSP LiJen [A60K][8M][NA][Others]modify camera power error handling

DEFINE_MUTEX(ov2720_mut);
struct msm_sensor_ctrl_t ov2720_s_ctrl;
#if 0	//LiJen: ISP dosen't  need
static struct msm_camera_i2c_reg_conf ov2720_start_settings[] = {
	{0x0100, 0x01},
};

static struct msm_camera_i2c_reg_conf ov2720_stop_settings[] = {
	{0x0100, 0x00},
};

static struct msm_camera_i2c_reg_conf ov2720_groupon_settings[] = {
	{0x3208, 0x00},
};

static struct msm_camera_i2c_reg_conf ov2720_groupoff_settings[] = {
	{0x3208, 0x10},
	{0x3208, 0xA0},
};

static struct msm_camera_i2c_reg_conf ov2720_prev_settings[] = {
	{0x3800, 0x00},
	{0x3801, 0x02},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x07},
	{0x3805, 0xA1},
	{0x3806, 0x04},
	{0x3807, 0x47},
	{0x3810, 0x00},
	{0x3811, 0x09},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x3820, 0x80},
	{0x3821, 0x06},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3612, 0x0b},
	{0x3618, 0x04},
	{0x3a08, 0x01},
	{0x3a09, 0x50},
	{0x3a0a, 0x01},
	{0x3a0b, 0x18},
	{0x3a0d, 0x03},
	{0x3a0e, 0x03},
	{0x4520, 0x00},
	{0x4837, 0x1b},
	{0x3000, 0xff},
	{0x3001, 0xff},
	{0x3002, 0xf0},
	{0x3600, 0x08},
	{0x3621, 0xc0},
	{0x3632, 0xd2},
	{0x3633, 0x23},
	{0x3634, 0x54},
	{0x3f01, 0x0c},
	{0x5001, 0xc1},
	{0x3614, 0xf0},
	{0x3630, 0x2d},
	{0x370b, 0x62},
	{0x3706, 0x61},
	{0x4000, 0x02},
	{0x4002, 0xc5},
	{0x4005, 0x08},
	{0x404f, 0x84},
	{0x4051, 0x00},
	{0x5000, 0xcf},
	{0x3a18, 0x00},
	{0x3a19, 0x80},
	{0x3503, 0x03},
	{0x4521, 0x00},
	{0x5183, 0xb0},
	{0x5184, 0xb0},
	{0x5185, 0xb0},
	{0x370c, 0x0c},
	{0x3035, 0x10},
	{0x3036, 0x1e},
	{0x3037, 0x21},
	{0x303e, 0x19},
	{0x3038, 0x06},
	{0x3018, 0x04},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},
	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},
	{0x4800, 0x24},
};

static struct msm_camera_i2c_reg_conf ov2720_720_settings[] = {
	{0x3800, 0x01},
	{0x3801, 0x4a},
	{0x3802, 0x00},
	{0x3803, 0xba},
	{0x3804, 0x06},
	{0x3805, 0x51+32},
	{0x3806, 0x03},
	{0x3807, 0x8d+24},
	{0x3810, 0x00},
	{0x3811, 0x05},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x3820, 0x80},
	{0x3821, 0x06},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3612, 0x0b},
	{0x3618, 0x04},
	{0x3a08, 0x01},
	{0x3a09, 0x50},
	{0x3a0a, 0x01},
	{0x3a0b, 0x18},
	{0x3a0d, 0x03},
	{0x3a0e, 0x03},
	{0x4520, 0x00},
	{0x4837, 0x1b},
	{0x3000, 0xff},
	{0x3001, 0xff},
	{0x3002, 0xf0},
	{0x3600, 0x08},
	{0x3621, 0xc0},
	{0x3632, 0xd2},
	{0x3633, 0x23},
	{0x3634, 0x54},
	{0x3f01, 0x0c},
	{0x5001, 0xc1},
	{0x3614, 0xf0},
	{0x3630, 0x2d},
	{0x370b, 0x62},
	{0x3706, 0x61},
	{0x4000, 0x02},
	{0x4002, 0xc5},
	{0x4005, 0x08},
	{0x404f, 0x84},
	{0x4051, 0x00},
	{0x5000, 0xff},
	{0x3a18, 0x00},
	{0x3a19, 0x80},
	{0x3503, 0x13},
	{0x4521, 0x00},
	{0x5183, 0xb0},
	{0x5184, 0xb0},
	{0x5185, 0xb0},
	{0x370c, 0x0c},
	{0x3035, 0x10},
	{0x3036, 0x04},
	{0x3037, 0x61},
	{0x303e, 0x19},
	{0x3038, 0x06},
	{0x3018, 0x04},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},
	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},
	{0x4800, 0x24},
};

static struct msm_camera_i2c_reg_conf ov2720_vga_settings[] = {
	{0x3800, 0x00},
	{0x3801, 0x0c},
	{0x3802, 0x00},
	{0x3803, 0x02},
	{0x3804, 0x07},
	{0x3805, 0x97+32},
	{0x3806, 0x04},
	{0x3807, 0x45+24},
	{0x3810, 0x00},
	{0x3811, 0x03},
	{0x3812, 0x00},
	{0x3813, 0x03},
	{0x3820, 0x80},
	{0x3821, 0x06},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3612, 0x0b},
	{0x3618, 0x04},
	{0x3a08, 0x01},
	{0x3a09, 0x50},
	{0x3a0a, 0x01},
	{0x3a0b, 0x18},
	{0x3a0d, 0x03},
	{0x3a0e, 0x03},
	{0x4520, 0x00},
	{0x4837, 0x1b},
	{0x3000, 0xff},
	{0x3001, 0xff},
	{0x3002, 0xf0},
	{0x3600, 0x08},
	{0x3621, 0xc0},
	{0x3632, 0xd2},
	{0x3633, 0x23},
	{0x3634, 0x54},
	{0x3f01, 0x0c},
	{0x5001, 0xc1},
	{0x3614, 0xf0},
	{0x3630, 0x2d},
	{0x370b, 0x62},
	{0x3706, 0x61},
	{0x4000, 0x02},
	{0x4002, 0xc5},
	{0x4005, 0x08},
	{0x404f, 0x84},
	{0x4051, 0x00},
	{0x5000, 0xff},
	{0x3a18, 0x00},
	{0x3a19, 0x80},
	{0x3503, 0x13},
	{0x4521, 0x00},
	{0x5183, 0xb0},
	{0x5184, 0xb0},
	{0x5185, 0xb0},
	{0x370c, 0x0c},
	{0x3035, 0x10},
	{0x3036, 0x04},
	{0x3037, 0x61},
	{0x303e, 0x19},
	{0x3038, 0x06},
	{0x3018, 0x04},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},
	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},
	{0x4800, 0x24},
	{0x3500, 0x00},
	{0x3501, 0x17},
	{0x3502, 0xf0},
	{0x3508, 0x00},
	{0x3509, 0x20},
};

static struct msm_camera_i2c_reg_conf ov2720_60fps_settings[] = {
	{0x3718, 0x10},
	{0x3702, 0x18},
	{0x373a, 0x3c},
	{0x3715, 0x01},
	{0x3703, 0x1d},
	{0x3705, 0x0b},
	{0x3730, 0x1f},
	{0x3704, 0x3f},
	{0x3f06, 0x1d},
	{0x371c, 0x00},
	{0x371d, 0x83},
	{0x371e, 0x00},
	{0x371f, 0xb6},
	{0x3708, 0x63},
	{0x3709, 0x52},
	{0x3800, 0x01},
	{0x3801, 0x42},
	{0x3802, 0x00},
	{0x3803, 0x40},
	{0x3804, 0x06},
	{0x3805, 0x61},
	{0x3806, 0x04},
	{0x3807, 0x08},
	{0x3808, 0x02},
	{0x3809, 0x80},
	{0x380a, 0x01},
	{0x380b, 0xe0},
	{0x380c, 0x03},
	{0x380d, 0x0c},
	{0x380e, 0x02},
	{0x380f, 0x00},
	{0x3810, 0x00},
	{0x3811, 0x0f},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x3820, 0x80},
	{0x3821, 0x06},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3612, 0x0b},
	{0x3618, 0x04},
	{0x3a08, 0x02},
	{0x3a09, 0x67},
	{0x3a0a, 0x02},
	{0x3a0b, 0x00},
	{0x3a0d, 0x00},
	{0x3a0e, 0x00},
	{0x4520, 0x0a},
	{0x4837, 0x29},
	{0x3000, 0xff},
	{0x3001, 0xff},
	{0x3002, 0xf0},
	{0x3600, 0x08},
	{0x3621, 0xc0},
	{0x3632, 0xd2},
	{0x3633, 0x23},
	{0x3634, 0x54},
	{0x3f01, 0x0c},
	{0x5001, 0xc1},
	{0x3614, 0xf0},
	{0x3630, 0x2d},
	{0x370b, 0x62},
	{0x3706, 0x61},
	{0x4000, 0x02},
	{0x4002, 0xc5},
	{0x4005, 0x08},
	{0x404f, 0x84},
	{0x4051, 0x00},
	{0x5000, 0xcf},
	{0x3a18, 0x00},
	{0x3a19, 0x80},
	{0x3503, 0x07},
	{0x4521, 0x00},
	{0x5183, 0xb0},
	{0x5184, 0xb0},
	{0x5185, 0xb0},
	{0x370c, 0x0c},
	{0x3035, 0x30},
	{0x3036, 0x14},
	{0x3037, 0x21},
	{0x303e, 0x19},
	{0x3038, 0x06},
	{0x3018, 0x04},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},
	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},
	{0x3011, 0x22},
	{0x3a00, 0x58},
};

static struct msm_camera_i2c_reg_conf ov2720_90fps_settings[] = {
	{0x3718, 0x10},
	{0x3702, 0x18},
	{0x373a, 0x3c},
	{0x3715, 0x01},
	{0x3703, 0x1d},
	{0x3705, 0x0b},
	{0x3730, 0x1f},
	{0x3704, 0x3f},
	{0x3f06, 0x1d},
	{0x371c, 0x00},
	{0x371d, 0x83},
	{0x371e, 0x00},
	{0x371f, 0xb6},
	{0x3708, 0x63},
	{0x3709, 0x52},
	{0x3800, 0x01},
	{0x3801, 0x42},
	{0x3802, 0x00},
	{0x3803, 0x40},
	{0x3804, 0x06},
	{0x3805, 0x61},
	{0x3806, 0x04},
	{0x3807, 0x08},
	{0x3808, 0x02},
	{0x3809, 0x80},
	{0x380a, 0x01},
	{0x380b, 0xe0},
	{0x380c, 0x03},
	{0x380d, 0x0c},
	{0x380e, 0x02},
	{0x380f, 0x00},
	{0x3810, 0x00},
	{0x3811, 0x0f},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x3820, 0x80},
	{0x3821, 0x06},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3612, 0x0b},
	{0x3618, 0x04},
	{0x3a08, 0x02},
	{0x3a09, 0x67},
	{0x3a0a, 0x02},
	{0x3a0b, 0x00},
	{0x3a0d, 0x00},
	{0x3a0e, 0x00},
	{0x4520, 0x0a},
	{0x4837, 0x29},
	{0x3000, 0xff},
	{0x3001, 0xff},
	{0x3002, 0xf0},
	{0x3600, 0x08},
	{0x3621, 0xc0},
	{0x3632, 0xd2},
	{0x3633, 0x23},
	{0x3634, 0x54},
	{0x3f01, 0x0c},
	{0x5001, 0xc1},
	{0x3614, 0xf0},
	{0x3630, 0x2d},
	{0x370b, 0x62},
	{0x3706, 0x61},
	{0x4000, 0x02},
	{0x4002, 0xc5},
	{0x4005, 0x08},
	{0x404f, 0x84},
	{0x4051, 0x00},
	{0x5000, 0xcf},
	{0x3a18, 0x00},
	{0x3a19, 0x80},
	{0x3503, 0x07},
	{0x4521, 0x00},
	{0x5183, 0xb0},
	{0x5184, 0xb0},
	{0x5185, 0xb0},
	{0x370c, 0x0c},
	{0x3035, 0x30},
	{0x3036, 0x1e},
	{0x3037, 0x21},
	{0x303e, 0x19},
	{0x3038, 0x06},
	{0x3018, 0x04},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},
	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},
	{0x3011, 0x22},
	{0x3a00, 0x58},
};

static struct msm_camera_i2c_reg_conf ov2720_120fps_settings[] = {
	{0x3718, 0x10},
	{0x3702, 0x18},
	{0x373a, 0x3c},
	{0x3715, 0x01},
	{0x3703, 0x1d},
	{0x3705, 0x0b},
	{0x3730, 0x1f},
	{0x3704, 0x3f},
	{0x3f06, 0x1d},
	{0x371c, 0x00},
	{0x371d, 0x83},
	{0x371e, 0x00},
	{0x371f, 0xb6},
	{0x3708, 0x63},
	{0x3709, 0x52},
	{0x3800, 0x01},
	{0x3801, 0x42},
	{0x3802, 0x00},
	{0x3803, 0x40},
	{0x3804, 0x06},
	{0x3805, 0x61},
	{0x3806, 0x04},
	{0x3807, 0x08},
	{0x3808, 0x02},
	{0x3809, 0x80},
	{0x380a, 0x01},
	{0x380b, 0xe0},
	{0x380c, 0x03},
	{0x380d, 0x0c},
	{0x380e, 0x02},
	{0x380f, 0x00},
	{0x3810, 0x00},
	{0x3811, 0x0f},
	{0x3812, 0x00},
	{0x3813, 0x02},
	{0x3820, 0x80},
	{0x3821, 0x06},
	{0x3814, 0x31},
	{0x3815, 0x31},
	{0x3612, 0x0b},
	{0x3618, 0x04},
	{0x3a08, 0x02},
	{0x3a09, 0x67},
	{0x3a0a, 0x02},
	{0x3a0b, 0x00},
	{0x3a0d, 0x00},
	{0x3a0e, 0x00},
	{0x4520, 0x0a},
	{0x4837, 0x29},
	{0x3000, 0xff},
	{0x3001, 0xff},
	{0x3002, 0xf0},
	{0x3600, 0x08},
	{0x3621, 0xc0},
	{0x3632, 0xd2},
	{0x3633, 0x23},
	{0x3634, 0x54},
	{0x3f01, 0x0c},
	{0x5001, 0xc1},
	{0x3614, 0xf0},
	{0x3630, 0x2d},
	{0x370b, 0x62},
	{0x3706, 0x61},
	{0x4000, 0x02},
	{0x4002, 0xc5},
	{0x4005, 0x08},
	{0x404f, 0x84},
	{0x4051, 0x00},
	{0x5000, 0xcf},
	{0x3a18, 0x00},
	{0x3a19, 0x80},
	{0x3503, 0x07},
	{0x4521, 0x00},
	{0x5183, 0xb0},
	{0x5184, 0xb0},
	{0x5185, 0xb0},
	{0x370c, 0x0c},
	{0x3035, 0x10},
	{0x3036, 0x14},
	{0x3037, 0x21},
	{0x303e, 0x19},
	{0x3038, 0x06},
	{0x3018, 0x04},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x3a0f, 0x40},
	{0x3a10, 0x38},
	{0x3a1b, 0x48},
	{0x3a1e, 0x30},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},
	{0x3011, 0x22},
	{0x3a00, 0x58},
};

static struct msm_camera_i2c_reg_conf ov2720_recommend_settings[] = {
	{0x0103, 0x01},
	{0x3718, 0x10},
	{0x3702, 0x24},
	{0x373a, 0x60},
	{0x3715, 0x01},
	{0x3703, 0x2e},
	{0x3705, 0x10},
	{0x3730, 0x30},
	{0x3704, 0x62},
	{0x3f06, 0x3a},
	{0x371c, 0x00},
	{0x371d, 0xc4},
	{0x371e, 0x01},
	{0x371f, 0x0d},
	{0x3708, 0x61},
	{0x3709, 0x12},
};
#endif

static struct v4l2_subdev_info ov2720_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_YUYV8_2X8,  //V4L2_MBUS_FMT_SBGGR10_1X10 // format ???
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

#if 0	//LiJen: ISP dosen't  need
static struct msm_camera_i2c_conf_array ov2720_init_conf[] = {
	{&ov2720_recommend_settings[0],
	ARRAY_SIZE(ov2720_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA}
};

static struct msm_camera_i2c_conf_array ov2720_confs[] = {
	{&ov2720_prev_settings[0],
	ARRAY_SIZE(ov2720_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ov2720_vga_settings[0],
	ARRAY_SIZE(ov2720_vga_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ov2720_720_settings[0],
	ARRAY_SIZE(ov2720_720_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
#if 0
	{&ov2720_60fps_settings[0],
	ARRAY_SIZE(ov2720_60fps_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ov2720_90fps_settings[0],
	ARRAY_SIZE(ov2720_90fps_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&ov2720_120fps_settings[0],
	ARRAY_SIZE(ov2720_120fps_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
#endif
};
#endif
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"

//ASUS_BSP +++ LiJen "[ov2720] porting Qcamera server for 8M camera"
static struct msm_sensor_output_info_t ov2720_dimensions[] = {
	{	//Capture
		.x_output = 3264,
		.y_output = 2448,
		.line_length_pclk = 0x85c,
		.frame_length_lines = 0x460,
		.vt_pixel_clk = 216000000,	//ASUS_BSP Stimber "Need to comfirm"
		.op_pixel_clk = 216000000,	//ASUS_BSP Stimber "Need to comfirm"
		.binning_factor = 1,
	},
	{	//Preview
		.x_output = 1280,
		.y_output = 960,
		.line_length_pclk = 0x85c,
		.frame_length_lines = 0x460,
		.vt_pixel_clk = 216000000,	//ASUS_BSP Stimber "Need to comfirm"
		.op_pixel_clk = 216000000,	//ASUS_BSP Stimber "Need to comfirm"
		.binning_factor = 1,
	},
//ASUS_BSP +++ Stimber "Add Full HD resolution for recording"
	{	//Video
		.x_output = 1920,
		.y_output = 1080,
		.line_length_pclk = 0x85c,
		.frame_length_lines = 0x460,
		.vt_pixel_clk = 216000000,	//ASUS_BSP Stimber "Need to comfirm"
		.op_pixel_clk = 216000000,	//ASUS_BSP Stimber "Need to comfirm"
		.binning_factor = 1,
	},
//ASUS_BSP --- Stimber "Add Full HD resolution for recording"
#if 0
	{
		.x_output = 640,//0x280, /* 640 */
		.y_output = 480,//0x1E0, /* 480 */
		.line_length_pclk = 0x30C, /* 780 */
		.frame_length_lines = 0x200, /* 512 */
		.vt_pixel_clk = 216000000,//24000000,
		.op_pixel_clk = 216000000,//24000000,
		.binning_factor = 1,
	},
	{
		.x_output = 640,//0x280, /* 640 */
		.y_output = 480,//0x1E0, /* 480 */
		.line_length_pclk = 0x30C, /* 780 */
		.frame_length_lines = 0x200, /* 512 */
		.vt_pixel_clk = 216000000,//36000000,
		.op_pixel_clk = 216000000,//36000000,
		.binning_factor = 1,
	},
	{
		.x_output = 640,//0x280, /* 640 */
		.y_output = 480,//0x1E0, /* 480 */
		.line_length_pclk = 0x30C, /* 780 */
		.frame_length_lines = 0x200, /* 512 */
		.vt_pixel_clk = 216000000,//48000000,
		.op_pixel_clk = 216000000,//48000000,
		.binning_factor = 1,
	},
#endif
};

static struct msm_camera_csid_vc_cfg ov2720_cid_cfg[] = {
	{0, 0x1E, CSI_DECODE_8BIT},
	{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
};

static struct msm_camera_csi2_params ov2720_csi_params = {
	.csid_params = {
		.lane_assign = 0xe4,
		.lane_cnt = 2,
		.lut_params = {
			.num_cid = 2,
			.vc_cfg = ov2720_cid_cfg,
		},
	},
	.csiphy_params = {
		.lane_cnt = 2,
		.settle_cnt = 0x11, //0x1B
	},
};

static struct msm_camera_csi2_params *ov2720_csi_params_array[] = {
	&ov2720_csi_params,
	&ov2720_csi_params,
	&ov2720_csi_params,
#if 0
	&ov2720_csi_params,
	&ov2720_csi_params,
	&ov2720_csi_params,
#endif
};
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"

static struct msm_sensor_output_reg_addr_t ov2720_reg_addr = {
	.x_output = 0x3808,
	.y_output = 0x380a,
	.line_length_pclk = 0x380c,
	.frame_length_lines = 0x380e,
};

static struct msm_sensor_id_info_t ov2720_id_info = {
	.sensor_id_reg_addr = 0x300A,
	.sensor_id = 0x2720,
};

static struct msm_sensor_exp_gain_info_t ov2720_exp_gain_info = {
	.coarse_int_time_addr = 0x3501,
	.global_gain_addr = 0x3508,
	.vert_offset = 6,
};

//ASUS_BSP +++ Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
static int ov2720_regulator_init(bool on)
{
	static struct regulator *reg_8921_l8, *reg_8921_l16, *reg_8921_lvs6;
	static int prev_on = false;
	int rc;

	pr_info("%s +++\n",__func__);

	if (on) {
		if (on == prev_on) {
			pr_info("on(%d) == prev_on(%d)\n",on,prev_on);
			pr_info("%s ---\n",__func__);
			return 0;
		}
		prev_on = true;

		if (!reg_8921_l8) {
			reg_8921_l8 = regulator_get(&ov2720_s_ctrl.sensor_i2c_client->client->dev, "8921_l8"); //ASUS_BSP Patrick "[A60K][8M][NA][Others] Modify GPIO parameters"
			if (IS_ERR(reg_8921_l8)) {
				pr_info("PTR_ERR(reg_8921_l8)=%ld\n", PTR_ERR(reg_8921_l8));
				return -ENODEV;
			}
		}

		if (!reg_8921_l16) {
			reg_8921_l16 = regulator_get(&ov2720_s_ctrl.sensor_i2c_client->client->dev, "8921_l16"); //ASUS_BSP Patrick "[A60K][8M][NA][Others] Modify GPIO parameters"
			if (IS_ERR(reg_8921_l16)) {
				pr_info("PTR_ERR(reg_8921_l16)=%ld\n", PTR_ERR(reg_8921_l16));
				return -ENODEV;
			}
		}

		if(g_A60K_hwID==A60K_EVB) {
			if (!reg_8921_lvs6) {
				reg_8921_lvs6 = regulator_get(&ov2720_s_ctrl.sensor_i2c_client->client->dev, "8921_lvs6"); //ASUS_BSP Patrick "[A60K][8M][NA][Others] Modify GPIO parameters"
				if (IS_ERR(reg_8921_lvs6)) {
					pr_info("PTR_ERR(reg_8921_lvs6)=%ld\n", PTR_ERR(reg_8921_lvs6));
					return -ENODEV;
				}
			}
		}

		pr_info("Turn on the regulators\n");
		rc = regulator_set_voltage(reg_8921_l8, 2800000, 2800000);
		if (!rc) {
			pr_info("reg_8921_l8 regulator_set_voltage, !rc is true, rc=%d--\n", rc);
			rc = regulator_enable(reg_8921_l8);
		}

		if (rc) {
			pr_info("8921_l8 regulator enable failed, rc=%d--\n", rc);
			return rc;
		}
		pr_info("Turn on reg_8921_l8 success\n");

		rc = regulator_set_voltage(reg_8921_l16, 2800000, 2800000);
		if (!rc) {
			pr_info("8921_l16 regulator_set_voltage, !rc is true, rc=%d--\n", rc);
			rc = regulator_enable(reg_8921_l16);
		}
		if (rc) {
			pr_info("8921_l16 regulator enable failed, rc=%d--\n", rc);
			return rc;
		}

		if(g_A60K_hwID==A60K_EVB) {
			pr_info("Turn on reg_8921_l16 success\n");
			rc = regulator_enable(reg_8921_lvs6);
			if (rc) {
				pr_info("8921_lvs6 regulator enable failed, rc=%d--\n", rc);
				return rc;
			}

			pr_info("Turn on reg_8921_lvs6 regulator success\n");
			pr_info("reg_8921_lvs6(%d)",regulator_get_voltage(reg_8921_lvs6));
			pr_info("reg_8921_lvs6 enable(%d)",regulator_is_enabled(reg_8921_lvs6));
		}

		pr_info("reg_8921_l8(%d)",regulator_get_voltage(reg_8921_l8));
		pr_info("reg_8921_l16(%d)",regulator_get_voltage(reg_8921_l16));

		pr_info("reg_8921_l8 enable(%d)",regulator_is_enabled(reg_8921_l8));
		pr_info("reg_8921_l16 enable(%d)",regulator_is_enabled(reg_8921_l16));
	} else {           //(on == false) /* Turn off the regulators */

		if (on == prev_on) {
			pr_info("on(%d) == prev_on(%d)\n",on,prev_on);
			pr_info("%s ---\n",__func__);
			return 0;
		}

		pr_info("Turn off the regulators\n");
		prev_on = false;

		regulator_disable(reg_8921_l8);
		regulator_disable(reg_8921_l16);
		if(g_A60K_hwID==A60K_EVB) {
			regulator_disable(reg_8921_lvs6);
		}
	}
	pr_info("%s ---\n",__func__);
	return 0;
}

void ov2720_sensor_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	pr_info("%s +++ \n",__func__);

	pr_info("%s --- \n",__func__);
}

void ov2720_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	pr_info("%s +++ \n",__func__);

	pr_info("%s --- \n",__func__);
}

late_initcall(fjm6mo_i2c_debuginit);

static int32_t ov2720_sensor_setting(struct msm_sensor_ctrl_t *s_ctrl,
				int update_type, int rt)
{
	int32_t rc = 0;
	static int prvMonitorState = MSM_SENSOR_INVALID_RES;	//ASUS_BSP Stimber "Keep the previous state"

	pr_info("%s +++\n",__func__);
	//v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
		//NOTIFY_ISPIF_STREAM, (void *)ISPIF_STREAM(
		//PIX_0, ISPIF_OFF_IMMEDIATELY));
	
#if 0	//LiJen: ISP dosen't  need 	
	s_ctrl->func_tbl.sensor_stop_stream(s_ctrl);
	msleep(30);
#endif
	if (update_type == MSM_SENSOR_REG_INIT) {
		pr_info("%s MSM_SENSOR_REG_INIT\n",__func__);	

		//CAM_START		
		rc = fjm6mo_sensor_open();
              if(rc < 0){
                pr_err("fjm6mo_sensor_open failed\n");
                return rc;
              }

		//set Parameter mode
// ASUS_BSP +++ LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 		
#ifdef WITH_WQ
		sensor_change_status(E_M6MO_Status_Init);
#else
              sensor_change_status(E_M6MO_Status_Parameter);
#endif
// ASUS_BSP --- LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 

		s_ctrl->config_csi_flag = 1;
		s_ctrl->curr_csi_params = NULL;
#if 0	//LiJen: ISP dosen't  need 
		msm_sensor_enable_debugfs(s_ctrl);
		msm_sensor_write_init_settings(s_ctrl);
#endif	
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
		pr_info("%s MSM_SENSOR_UPDATE_PERIODIC : prvMonitorState = %d, rt = %d\n", __func__, prvMonitorState, rt);

#if 0	//LiJen: ISP dosen't  need 
		msm_sensor_write_res_settings(s_ctrl, rt);
#endif

            if (s_ctrl->config_csi_flag) {
		if (s_ctrl->curr_csi_params != s_ctrl->csi_params[0]) {
			pr_info("config to csi +++\n");
			s_ctrl->curr_csi_params = s_ctrl->csi_params[0];
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_CSID_CFG,
				&s_ctrl->curr_csi_params->csid_params);
			//v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
						//NOTIFY_CID_CHANGE, NULL);
			mb();
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_CSIPHY_CFG,
				&s_ctrl->curr_csi_params->csiphy_params);
			mb();
			msleep(20);
			s_ctrl->config_csi_flag = 0;
			pr_info("config to csi ---\n");
		}
}

		v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
			NOTIFY_PCLK_CHANGE, &s_ctrl->msm_sensor_reg->
			output_settings[0].op_pixel_clk);

		//v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
			//NOTIFY_ISPIF_STREAM, (void *)ISPIF_STREAM(
			//PIX_0, ISPIF_ON_FRAME_BOUNDARY));
		
#if 0	//LiJen: ISP dosen't  need 	
		s_ctrl->func_tbl.sensor_start_stream(s_ctrl);
		msleep(30);
#endif
		if(rt == MSM_SENSOR_RES_FULL){	//Enter to capture mode
					//set Capture mode
					sensor_change_status(E_M6MO_Status_Capture);
		} else if (rt ==MSM_SENSOR_RES_QTR || rt== MSM_SENSOR_RES_FULL_HD) { // enter Monitor mode (param->monitor, capture->monitor && monitot->monitor)
				pr_info("[ISP] Enter Monitor mode. resolution(%d->%d)\n", prvMonitorState, rt);	
			
				if (prvMonitorState != rt && prvMonitorState!=MSM_SENSOR_INVALID_RES) { //preview resolution change
					sensor_change_status(E_M6MO_Status_Parameter);	//ASUS_BSP Stimber "New add if changing ISP resolution"
				}
				g_rt_for_monitor = rt; //Change to new resolution in rt			
				sensor_change_status(E_M6MO_Status_Monitor);
				prvMonitorState = rt; // Save current Monitor resolution.
		}else {
			pr_info("[ISP]ERROR invalid rt(%d)\n", rt);
			rc = -1;
		}

	}
	pr_info("%s ---\n",__func__);
	return rc;
}

int32_t ov2720_sensor_set_sensor_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int mode, int res)
{
	int32_t rc = 0;

	pr_info("mode(%d), res(%d), curr_res(%d)",mode,res,s_ctrl->curr_res);
	if (s_ctrl->curr_res != res) {
		switch (mode) {
		case SENSOR_MODE_PREVIEW:
		case SENSOR_MODE_VIDEO:
		case SENSOR_MODE_VIDEO_FULL_HD:	//ASUS_BSP Stimber "Add Full HD resolution for recording"
			s_ctrl->prev_res = res;
			break;
		case SENSOR_MODE_SNAPSHOT:
		case SENSOR_MODE_RAW_SNAPSHOT:
			s_ctrl->pict_res = res;
			break;
		default:
			pr_err("%s --- mode(%d) is not support \n",__func__,mode);
			rc = -EINVAL;
			return rc;
		}

#if 0	//LiJen: ISP dosen't  need 
		//Todo LiJen: get from fjm6mo	
		s_ctrl->curr_frame_length_lines =
			s_ctrl->msm_sensor_reg->
			output_settings[res].frame_length_lines;

		s_ctrl->curr_line_length_pclk =
			s_ctrl->msm_sensor_reg->
			output_settings[res].line_length_pclk;
#endif

		if (s_ctrl->func_tbl->sensor_setting
			(s_ctrl, MSM_SENSOR_UPDATE_PERIODIC, res) < 0)
			return rc;
		s_ctrl->curr_res = res;
	}
	pr_info("%s --- \n",__func__);
	return rc;
}

int32_t ov2720_sensor_mode_init(struct msm_sensor_ctrl_t *s_ctrl,
			int mode, struct sensor_init_cfg *init_info)
{
	int32_t rc = 0;
	pr_info("%s +++ return\n",__func__);
	s_ctrl->fps_divider = Q10;
	s_ctrl->cam_mode = MSM_SENSOR_MODE_INVALID;

	CDBG("%s: %d\n", __func__, __LINE__);
	if (mode != s_ctrl->cam_mode) {

#if 0	//LiJen: ISP dosen't  need			
		if (init_info->prev_res >=
			s_ctrl->msm_sensor_reg->num_conf ||
			init_info->pict_res >=
			s_ctrl->msm_sensor_reg->num_conf) {
			CDBG("Resolution does not exist");
			return -EINVAL;
		}
#endif		

		s_ctrl->prev_res = init_info->prev_res;
		s_ctrl->pict_res = init_info->pict_res;
		s_ctrl->curr_res = MSM_SENSOR_INVALID_RES;
		s_ctrl->cam_mode = mode;

		rc = s_ctrl->func_tbl->sensor_setting(s_ctrl,
			MSM_SENSOR_REG_INIT, 0);
	}
	pr_info("%s --- \n",__func__);
	return rc;
}

static int ov2720_sensor_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	pr_info("%s +++ \n",__func__);
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("msm_sensor_config: cfgtype = %d\n",
	cdata.cfgtype);
		switch (cdata.cfgtype) {
		case CFG_SET_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_sensor_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->func_tbl->
				sensor_set_sensor_mode(
					s_ctrl,
					cdata.mode,
					cdata.rs);
			break;

		case CFG_SET_EFFECT:
			break;

		case CFG_SENSOR_INIT:
			if (s_ctrl->func_tbl->
			sensor_mode_init == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->func_tbl->
				sensor_mode_init(
				s_ctrl,
				cdata.mode,
				&(cdata.cfg.init_info));
			break;

		case CFG_GET_OUTPUT_INFO:
			if (s_ctrl->func_tbl->
			sensor_get_output_info == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->func_tbl->
				sensor_get_output_info(
				s_ctrl,
				&cdata.cfg.output_info);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;
#if 0
		case CFG_GET_EEPROM_DATA:
			if (s_ctrl->sensor_eeprom_client == NULL ||
				s_ctrl->sensor_eeprom_client->
				func_tbl.eeprom_get_data == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->sensor_eeprom_client->
				func_tbl.eeprom_get_data(
				s_ctrl->sensor_eeprom_client,
				&cdata.cfg.eeprom_data);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_eeprom_data_t)))
				rc = -EFAULT;
			break;
#endif

//ASUS_BSP +++ 
		case CFG_ISP_AF_START:
			if (s_ctrl->func_tbl->
			sensor_isp_af_start == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_ISP_AF_START afParam(%d,%d,%d,%d,%d,%d)\n",cdata.cfg.focus.af_enable,cdata.cfg.focus.mode,cdata.cfg.focus.coordinate_x,cdata.cfg.focus.coordinate_y,cdata.cfg.focus.rectangle_h,cdata.cfg.focus.rectangle_w);
			s_ctrl->func_tbl->
				sensor_isp_af_start(
				cdata.cfg.focus.af_enable,
				cdata.cfg.focus.mode,
				cdata.cfg.focus.coordinate_x,
				cdata.cfg.focus.coordinate_y,
				cdata.cfg.focus.rectangle_h,
				cdata.cfg.focus.rectangle_w);
			break;

		case CFG_GET_ISP_AF_RESULT:
			if (s_ctrl->func_tbl->
			sensor_get_isp_af_result == NULL) {
				rc = -EFAULT;
				break;
			}
			cdata.cfg.focus.result =
				s_ctrl->func_tbl->
				sensor_get_isp_af_result
				(s_ctrl);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			
			pr_info("CFG_GET_ISP_AF_RESULT: %d\n",cdata.cfg.focus.result);
			break;		
            
		case CFG_SET_ISP_LED_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_led_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_LED_MODE mode(%d)\n",cdata.cfg.is_autoflash);
			s_ctrl->func_tbl->
			sensor_set_isp_led_mode(cdata.cfg.is_autoflash);					
			break;

		case CFG_SET_ISP_EFFECT_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_effect_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_EFFECT_MODE mode(%d)\n",cdata.cfg.effect);
			s_ctrl->func_tbl->
			sensor_set_isp_effect_mode(cdata.cfg.effect);						
			break;

		case CFG_SET_ISP_WB_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_wb_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_WB_MODE mode(%d)\n",cdata.cfg.wb);
			s_ctrl->func_tbl->
			sensor_set_isp_wb_mode(cdata.cfg.wb);						
			break;

		case CFG_SET_ISP_EV_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_ev_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_EV_MODE mode(%d)\n",cdata.cfg.ev);
			s_ctrl->func_tbl->
			sensor_set_isp_ev_mode(cdata.cfg.ev);						
			break;

		case CFG_SET_ISP_SCENE_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_scene_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_SCENE_MODE mode(%d)\n",cdata.cfg.scene);
			s_ctrl->func_tbl->
			sensor_set_isp_scene_mode(cdata.cfg.scene);						
			break;

		case CFG_SET_ISP_CAF_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_caf_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_CAF_MODE mode(%d)\n",cdata.cfg.focus.af_continue);
			s_ctrl->func_tbl->
			sensor_set_isp_caf_mode(cdata.cfg.focus.af_continue);						
			break;

		case CFG_SET_ISP_AECLOCK_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_aeclock_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_AECLOCK_MODE mode(%d)\n",cdata.cfg.aeclock);
			s_ctrl->func_tbl->
			sensor_set_isp_aeclock_mode(cdata.cfg.aeclock);						
			break;

		case CFG_SET_ISP_AWBLOCK_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_awblock_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_AWBLOCK_MODE mode(%d)\n",cdata.cfg.awblock);
			s_ctrl->func_tbl->
			sensor_set_isp_awblock_mode(cdata.cfg.awblock);						
			break;
		case CFG_SET_ISP_ISO_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_iso_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_ISO_MODE mode(%d)\n",cdata.cfg.iso);
			s_ctrl->func_tbl->
			sensor_set_isp_iso_mode(cdata.cfg.iso);						
			break;      
		case CFG_SET_ISP_FLICKER_MODE:
			if (s_ctrl->func_tbl->
			sensor_set_isp_flicker_mode == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_SET_ISP_FLICKER_MODE mode(%d)\n",cdata.cfg.flicker);
			s_ctrl->func_tbl->
			sensor_set_isp_flicker_mode(cdata.cfg.flicker);						
			break;   

//ASUS_BSP +++ Stimber "[A60K][8M][NA][Other] Implement EXIF info for 8M camera with ISP"
		case CFG_GET_ISP_EXIF:
			if (s_ctrl->func_tbl->sensor_get_isp_exif == NULL) {
				rc = -EFAULT;
				break;
			}
			pr_info("CFG_GET_ISP_EXIF\n");
			s_ctrl->func_tbl->sensor_get_isp_exif(&cdata.cfg.exif);
			if (copy_to_user((void *)argp, &cdata, sizeof(struct sensor_cfg_data))){
				rc = -EFAULT;
			}
			break;
//ASUS_BSP --- Stimber "[A60K][8M][NA][Other] Implement EXIF info for 8M camera with ISP"

//add by sam +++
#if 0
		case CFG_SET_SATURATION:
		case CFG_SET_SHARPNESS:             
		case CFG_SET_TOUCHAEC:              
		case CFG_SET_AUTO_FOCUS:            
		case CFG_SET_AUTOFLASH:            
		case CFG_SET_EXPOSURE_COMPENSATION: 
		case CFG_SET_ISO:          
			printk("Work_around!\n");
			break;
		case CFG_START_STREAM:  
			if (s_ctrl->func_tbl->sensor_start_stream == NULL) {
				rc = -EFAULT;
				break;
			}
			s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
			break;
		case CFG_STOP_STREAM:
			if (s_ctrl->func_tbl->sensor_stop_stream == NULL) {
				rc = -EFAULT;
				break;
			}
			s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
			break;
#endif
//add by sam ---

		case CFG_START_STREAM:  
			if (s_ctrl->func_tbl->sensor_start_stream == NULL) {
				rc = -EFAULT;
				break;
			}
			s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
			break;
		case CFG_STOP_STREAM:
			if (s_ctrl->func_tbl->sensor_stop_stream == NULL) {
				rc = -EFAULT;
				break;
			}
			s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
			break;

		case CFG_GET_CSI_PARAMS:
			if (s_ctrl->func_tbl->sensor_get_csi_params == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->func_tbl->sensor_get_csi_params(
				s_ctrl,
				&cdata.cfg.csi_lane_params);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

// ASUS BSP ---
		default:
			pr_info("%s: cfgtype = %d is not supported!!\n",__func__,cdata.cfgtype);
			rc = -EFAULT;
			break;
		}

	mutex_unlock(s_ctrl->msm_sensor_mutex);
	pr_info("%s --- \n",__func__);
	return rc;
}

static int ov2720_power_down(const struct msm_camera_sensor_info *data)
{
	pr_info("%s +++\n",__func__);

	if(!data)
	{
		pr_info("data is NULL, return\n");
		pr_info("%s ---\n",__func__);
		return -1;
	}

	if(!ov2720_s_ctrl.sensordata)
	{
		pr_info("ov2720_s_ctrl.sensordata is NULL, return\n");
		pr_info("%s ---\n",__func__);
		return -1;
	}

	fjm6mo_sensor_release();
       ov2720_regulator_init(false);

	//mutex_lock(ov2720_s_ctrl.msm_sensor_mutex);    //ASUS_BSP Stimber "Fix the issue which fail to re-open camera"
	gpio_set_value(ov2720_s_ctrl.sensordata->sensor_platform_info->vga_cmos_mclk_en, 0);
       gpio_set_value(ov2720_s_ctrl.sensordata->sensor_platform_info->sensor_reset, 0);
	gpio_set_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_1p8_en, 0);

	// T4, T2
	msleep(1);
	gpio_set_value(ov2720_s_ctrl.sensordata->sensor_platform_info->isp_pwr_en, 0);

	gpio_set_value_cansleep(ov2720_s_ctrl.sensordata->sensor_reset, 0);

	if(g_A60K_hwID!=A60K_EVB) {
		gpio_set_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect, 1);
		gpio_set_value(g_GPIO_CAM_8M_RST_N, 0);
	}

	msleep(20);
	gpio_free(ov2720_s_ctrl.sensordata->sensor_platform_info->sensor_reset);
	gpio_free(ov2720_s_ctrl.sensordata->sensor_platform_info->isp_pwr_en);
	gpio_free(ov2720_s_ctrl.sensordata->sensor_platform_info->vga_cmos_mclk_en);
	gpio_free(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_1p8_en);	

	if(g_A60K_hwID!=A60K_EVB) {
		gpio_free(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect);
		pr_info("g_GPIO_8M_WP=%d--\n", ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect);
		gpio_free(g_GPIO_CAM_8M_RST_N);
		pr_info("g_GPIO_CAM_8M_RST_N=%d--\n",  g_GPIO_CAM_8M_RST_N);
	}

//Turn off 24M CLK 	
       //ov2720_sensor_probe_off(&ov2720_s_ctrl.sensor_i2c_client->client->dev);

	//mutex_unlock(ov2720_s_ctrl.msm_sensor_mutex);    //ASUS_BSP Stimber "Fix the issue which fail to re-open camera"
	//msleep(1000);

	pr_info("%s ---\n",__func__);
	return 0;
}

static int ov2720_gpio_request(void)
{
	int32_t rc = 0;
    
        pr_info("%s +++\n",__func__);

        // Power on ISP module:
        rc = gpio_request(ov2720_s_ctrl.sensordata->sensor_platform_info->isp_pwr_en, "ov2720");
        if (rc) {
        	pr_err("%s: gpio isp_pwr_en %d, rc(%d)fail\n",__func__, ov2720_s_ctrl.sensordata->sensor_platform_info->isp_pwr_en, rc);
        	goto init_probe_fail0;
        }

        // Power on 8M camera OV8830 module:
        rc = gpio_request(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_1p8_en, "ov2720");
        if (rc) {
        	pr_err("%s: gpio cam_1p8_en %d, rc(%d)fail\n",__func__, ov2720_s_ctrl.sensordata->sensor_platform_info->cam_1p8_en, rc);
        	goto init_probe_fail1;
        }

        rc = gpio_request(ov2720_s_ctrl.sensordata->sensor_platform_info->vga_cmos_mclk_en, "ov2720");
        if (rc) {
        	pr_err("%s: gpio vga_cmos_mclk_en %d, rc(%d)fail\n",__func__, ov2720_s_ctrl.sensordata->sensor_platform_info->vga_cmos_mclk_en, rc);
        	goto init_probe_fail2;
        }

        rc = gpio_request(ov2720_s_ctrl.sensordata->sensor_platform_info->sensor_reset, "ov2720");
        if (rc) {
        	pr_err("%s: gpio sensor_reset %d, rc(%d)fail\n",__func__, ov2720_s_ctrl.sensordata->sensor_platform_info->sensor_reset, rc);
        	goto init_probe_fail3;
        }

        if(g_A60K_hwID!=A60K_EVB) {
        	rc = gpio_request(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect, "ov2720");
        	if (rc) {
        		pr_err("%s: gpio 8M_WP %d, rc(%d)fail\n",__func__, ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect, rc);
        		goto init_probe_fail4;
        	}
        	rc = gpio_request(g_GPIO_CAM_8M_RST_N, "ov2720");
        	if (rc) {
        		pr_err("%s: gpio 8M_RST_N %d, rc(%d)fail\n",__func__, g_GPIO_CAM_8M_RST_N, rc);
        		goto init_probe_fail5;
        	}
        }
        
	pr_info("%s ---\n",__func__);
	return rc;

if (g_A60K_hwID!=A60K_EVB) {

init_probe_fail5:
	gpio_free(g_GPIO_CAM_8M_RST_N);

init_probe_fail4:
	gpio_free(ov2720_s_ctrl.sensordata->sensor_platform_info->sensor_reset);
}

init_probe_fail3:
	gpio_free(ov2720_s_ctrl.sensordata->sensor_platform_info->vga_cmos_mclk_en);

init_probe_fail2:
	gpio_free(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_1p8_en);

init_probe_fail1:
	gpio_free(ov2720_s_ctrl.sensordata->sensor_platform_info->isp_pwr_en);

init_probe_fail0:    
	pr_info("%s ---\n",__func__);

	return rc;
}

static int ov2720_power_up(const struct msm_camera_sensor_info *data)
{
	int ret = -1;

	pr_info("%s +++\n",__func__);

	if(!data)
	{
		pr_err("data is NULL, return\n");
		pr_err("%s ---\n",__func__);
		return -1;
	}

	if(!ov2720_s_ctrl.sensordata)
	{
		pr_err("ov2720_s_ctrl.sensordata is NULL, return\n");
		pr_info("%s ---\n",__func__);
		return -1;
	}

	//Todo: ENABLE LVS6
	//ov2720_regulator_init(true);
	//msleep(50);

	ret = ov2720_gpio_request();
	if(ret < 0)
	{
		pr_err("8M Camera GPIO request fail!!\n");
		pr_info("%s ---\n",__func__);
		return -1;
	}

//ISP_RST reset
	gpio_direction_output(ov2720_s_ctrl.sensordata->sensor_platform_info->sensor_reset, 0);

// vga_cmos_mclk_en switch to VGA, ensure initial state
	gpio_direction_output(ov2720_s_ctrl.sensordata->sensor_platform_info->vga_cmos_mclk_en, 0);
 

// ISP 1.2V ON
	gpio_direction_output(ov2720_s_ctrl.sensordata->sensor_platform_info->isp_pwr_en, 1);
	msleep(10);	//To do : perfomance

// ISP 1.8V on
	gpio_direction_output(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_1p8_en, 1);

// delay(t2)
	msleep(1);

// Switch CLK to 8M
	gpio_set_value(ov2720_s_ctrl.sensordata->sensor_platform_info->vga_cmos_mclk_en, 1);

	if(g_A60K_hwID!=A60K_EVB) {
		gpio_direction_output(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect, 1);
		pr_info("gpio g_GPIO_8M_WP(%d)\n",gpio_get_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect));
	}

// delay(t4)
	msleep(1);

// ISP_RST release
	gpio_set_value(ov2720_s_ctrl.sensordata->sensor_platform_info->sensor_reset, 1);


	ov2720_regulator_init(true);

	// delay before 1st I2C command
	msleep(30);	//To do : perfomance

	pr_info("gpio sensor_reset(%d)\n",gpio_get_value(ov2720_s_ctrl.sensordata->sensor_platform_info->sensor_reset));
	pr_info("gpio isp_pwr_en(%d)\n",gpio_get_value(ov2720_s_ctrl.sensordata->sensor_platform_info->isp_pwr_en));
	pr_info("gpio vga_cmos_mclk_en(%d)\n",gpio_get_value(ov2720_s_ctrl.sensordata->sensor_platform_info->vga_cmos_mclk_en));
	pr_info("gpio cam_1p8_en(%d)\n",gpio_get_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_1p8_en));

	if(g_A60K_hwID!=A60K_EVB) {
		pr_info("gpio g_GPIO_8M_WP(%d)\n",gpio_get_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect));
		pr_info("gpio g_GPIO_CAM_8M_RST_N(%d)\n",gpio_get_value(g_GPIO_CAM_8M_RST_N));
	}

	pr_info("gpio GPIO5(%d)\n",gpio_get_value(5));
	pr_info("%s ---\n",__func__);
	return 0;	
}

//ASUS_BSP +++ Stimber "Implement i2c stress test method"
#ifdef CONFIG_I2C_STRESS_TEST
#include <linux/i2c_testcase.h>
#define I2C_TEST_Fjm6mo_FAIL (-1)

static int i2c_test_fjm6mo_open_probe(void)
{
	int rc = 0;

	msm_sensor_power_up(&ov2720_s_ctrl);
	pr_info("turn on MCLK");
	
	rc = ov2720_s_ctrl.func_tbl->sensor_power_up(&ov2720_s_ctrl);
	if (rc < 0)
		goto probe_fail;
	
	//Read Fjm6mo ID
	fw_version();
	
	rc = ov2720_s_ctrl.func_tbl->sensor_power_down(&ov2720_s_ctrl);
	if (rc < 0)
		goto probe_fail;

	msm_sensor_power_down(&ov2720_s_ctrl);
	pr_info("turn off MCLK");

	return rc;

probe_fail:
	pr_err("%s fails\n", __func__);

	return rc;

}
static int i2c_test_fjm6mo_camera(struct i2c_client *apClient)
{

	int lnResult = I2C_TEST_PASS;
    
	i2c_log_in_test_case("i2c_test_Fjm6mo_camera++\n");
	if (i2c_test_fjm6mo_open_probe() < 0) {
        	i2c_log_in_test_case("i2c_test_Fjm6mo_camera failed\n");        
		lnResult = I2C_TEST_Fjm6mo_FAIL;
	}
    
	i2c_log_in_test_case("i2c_test_Fjm6mo_camera--\n");
	return lnResult;
};

static struct i2c_test_case_info gFjm6moTestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(i2c_test_fjm6mo_camera),
};

#endif
//ASUS_BSP --- Stimber "Implement i2c stress test method"
//ASUS_BSP +++ LiJen [A60K][8M][NA][Others]modify camera power error handling
int32_t ov2720_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc =0;
    if(g_ov2720_power == false){
        // Config common GPIO
        rc = msm_sensor_power_up(s_ctrl);
        if(rc < 0){
            pr_err("%s: config common gpio failed\n", __func__);
            ASUSEvtlog("[BAT][8M]Report Capacity: EVTLOG_CAMERA_InitFail\n"); //ASUS_BSP LiJen "[A66][8M][NA][Others]add camera power event logs" 
            goto fail;
        }

        // Condif ov2720 GPIO
        rc = ov2720_power_up(s_ctrl->sensordata);
        if(rc < 0){
            pr_err("%s: config ov2720 gpio failed\n", __func__);
            ASUSEvtlog("[BAT][8M]Report Capacity: EVTLOG_CAMERA_InitFail\n"); //ASUS_BSP LiJen "[A66][8M][NA][Others]add camera power event logs" 
            goto fail;
        }
        g_ov2720_power = true;
        ASUSEvtlog("[BAT][8M]Report Capacity: EVTLOG_CAMERA_ON\n"); //ASUS_BSP LiJen "[A66][8M][NA][Others]add camera power event logs"
    }else{
        pr_info("%s: power has enabled\n", __func__);
         rc = -2; // already power on, do nothing
    }
    
fail:    
    return rc;
}

int32_t ov2720_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc =0;
    
    if(g_ov2720_power == true){

// ASUS_BSP +++ LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 
#ifdef WITH_WQ
      	fjm6mo_flush_workqueue();
#endif
// ASUS_BSP --- LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode"   

        // Disable common GPIO
        msm_sensor_power_down(s_ctrl);
        // Disable ov2720 GPIO
        ov2720_power_down(s_ctrl->sensordata);      
        g_ov2720_power = false;
        ASUSEvtlog("[BAT][8M]Report Capacity: EVTLOG_CAMERA_OFF\n"); //ASUS_BSP LiJen "[A66][8M][NA][Others]add camera power event logs"        
    }else{
        pr_info("%s: power has disabled\n", __func__);
    }
    
    return rc;
}
//ASUS_BSP --- LiJen [A60K][8M][NA][Others]modify camera power error handling

int32_t ov2720_sensor_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl;
	pr_info("%s +++ \n",__func__);
	CDBG("%s_i2c_probe called\n", client->name);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		rc = -EFAULT;
                pr_info("%s --- \n",__func__);
		return rc;
	}

	s_ctrl = (struct msm_sensor_ctrl_t *)(id->driver_data);
	if (s_ctrl->sensor_i2c_client != NULL) {
		s_ctrl->sensor_i2c_client->client = client;
		if (s_ctrl->sensor_i2c_addr != 0)
			s_ctrl->sensor_i2c_client->client->addr =
				s_ctrl->sensor_i2c_addr;
	} else {
		rc = -EFAULT;
                pr_info("%s --- \n",__func__);
		return rc;
	}

	s_ctrl->sensordata = client->dev.platform_data;
       ov2720_s_ctrl.sensordata = client->dev.platform_data;

	   switch (g_A60K_hwID)
	   {
			case A60K_EVB:
				ov2720_s_ctrl.sensordata->sensor_platform_info->vga_cmos_mclk_en = 1;
				ov2720_s_ctrl.sensordata->sensor_platform_info->sensor_reset = 6;
				ov2720_s_ctrl.sensordata->sensor_platform_info->isp_pwr_en = 8;
				ov2720_s_ctrl.sensordata->sensor_platform_info->cam_1p8_en = 9;
				pr_info("In %s, g_A60K_hwID==A60K_EVB\n", __func__);
				break;
			case A60K_SR1_1:
			case A60K_SR1_2_ES1:
			case A60K_SR1_2_ES2:
			case A60K_ER1:
			case A60K_ER2:
			case A60K_PR:
				ov2720_s_ctrl.sensordata->sensor_platform_info->vga_cmos_mclk_en = 1;
				ov2720_s_ctrl.sensordata->sensor_platform_info->sensor_reset = 6;
				ov2720_s_ctrl.sensordata->sensor_platform_info->isp_pwr_en = 8;
				ov2720_s_ctrl.sensordata->sensor_platform_info->cam_1p8_en = 9;
				ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect= 2;
				pr_info("In %s, g_A60K_hwID==A60K\n", __func__);
				break;
			case A66_HW_ID_SR1_1:
			case A66_HW_ID_SR2:
			case A66_HW_ID_ER1:
			case A66_HW_ID_ER2:
			case A66_HW_ID_ER3:
			case A66_HW_ID_PR:
			default:
				ov2720_s_ctrl.sensordata->sensor_platform_info->vga_cmos_mclk_en = 1;
				ov2720_s_ctrl.sensordata->sensor_platform_info->isp_pwr_en = 8;
				ov2720_s_ctrl.sensordata->sensor_platform_info->sensor_reset = 24;
				ov2720_s_ctrl.sensordata->sensor_platform_info->cam_1p8_en = 56;
				ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect= 48;
				pr_info("In %s, g_A60K_hwID==A66\n", __func__);
				break;
		}

		rc = s_ctrl->func_tbl->sensor_power_up(&ov2720_s_ctrl);
		if (rc < 0)
			goto probe_fail;
	
		//Read Fjm6mo ID
		rc = fw_version();
		if (rc < 0)
			goto probe_fail;

//ASUS_BSP +++ Stimber "Implement i2c stress test method"
#ifdef CONFIG_I2C_STRESS_TEST
		printk("[8M Camera] Fjm6mo_Camera add test case+\n");
				
		i2c_add_test_case(ov2720_s_ctrl.sensor_i2c_client->client , "Fjm6mo", ARRAY_AND_SIZE(gFjm6moTestCaseInfo));
				
		printk("[8M Camera] Fjm6mo_Camera add test case-\n");
#endif
//ASUS_BSP --- Stimber "Implement i2c stress test method"
        
	snprintf(s_ctrl->sensor_v4l2_subdev.name,
		sizeof(s_ctrl->sensor_v4l2_subdev.name), "%s", id->name);
	v4l2_i2c_subdev_init(&s_ctrl->sensor_v4l2_subdev, client,
		s_ctrl->sensor_v4l2_subdev_ops);

	msm_sensor_register(&s_ctrl->sensor_v4l2_subdev);
	goto power_down;
    
probe_fail:
	CDBG("%s_i2c_probe failed\n", client->name);
power_down:
        s_ctrl->func_tbl->sensor_power_down(&ov2720_s_ctrl);
        pr_info("%s --- \n",__func__);
	return rc;
}
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"

static int32_t ov2720_write_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
		uint16_t gain, uint32_t line)
{
	uint32_t fl_lines, offset;
	uint8_t int_time[3];
	fl_lines =
		(s_ctrl->curr_frame_length_lines * s_ctrl->fps_divider) / Q10;
	offset = s_ctrl->sensor_exp_gain_info->vert_offset;
	if (line > (fl_lines - offset))
		fl_lines = line + offset;

	s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_output_reg_addr->frame_length_lines, fl_lines,
		MSM_CAMERA_I2C_WORD_DATA);
	int_time[0] = line >> 12;
	int_time[1] = line >> 4;
	int_time[2] = line << 4;
	msm_camera_i2c_write_seq(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->coarse_int_time_addr-1,
		&int_time[0], 3);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
		s_ctrl->sensor_exp_gain_info->global_gain_addr, gain,
		MSM_CAMERA_I2C_WORD_DATA);
	s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
	return 0;
}

static const struct i2c_device_id ov2720_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&ov2720_s_ctrl},
	{ }
};

static struct i2c_driver ov2720_i2c_driver = {
	.id_table = ov2720_i2c_id,
	.probe  = ov2720_sensor_i2c_probe,  //ASUS_BSP Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov2720_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int __init msm_sensor_init_module(void)
{
       create_fjm6mo_proc_file();   //ASUS_BSP  LiJen "[A66][8M][NA][Others]add proc file fo AP ISP update"
// ASUS_BSP +++ LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 
#ifdef WITH_WQ
      	fjm6mo_create_workqueue();
#endif
// ASUS_BSP --- LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 
	return i2c_add_driver(&ov2720_i2c_driver);
}

static struct v4l2_subdev_core_ops ov2720_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops ov2720_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops ov2720_subdev_ops = {
	.core = &ov2720_subdev_core_ops,
	.video  = &ov2720_subdev_video_ops,
};

static struct msm_sensor_fn_t ov2720_func_tbl = {
//ASUS_BSP +++ Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
	.sensor_start_stream = ov2720_sensor_start_stream,
	.sensor_stop_stream = ov2720_sensor_stop_stream,
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = ov2720_write_exp_gain,
	.sensor_write_snapshot_exp_gain = ov2720_write_exp_gain,
//ASUS_BSP +++ Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
	.sensor_isp_af_start = fjm6mo_start_AF,
	.sensor_get_isp_af_result = fjm6mo_get_AF_result,
	.sensor_set_isp_led_mode = fjm6mo_set_led_mode,	//ASUS_BSP LiJen "[A60K][8M][NA][Others]implement LED/Flash mode in 8M camera with ISP"	
	.sensor_set_isp_effect_mode = fjm6mo_set_effect_mode, //ASUS_BSP LiJen "[A60K][8M][NA][Others]implement Effect mode in 8M camera with ISP"	
	.sensor_set_isp_wb_mode = fjm6mo_set_wb_mode, //ASUS_BSP LiJen "[A60K][8M][NA][Others]implement WB mode in 8M camera with ISP"	
	.sensor_set_isp_ev_mode =  fjm6mo_set_ev_mode, //ASUS_BSP LiJen "[A60K][8M][NA][Others]implement EV mode in 8M camera with ISP"
	.sensor_set_isp_scene_mode =  fjm6mo_set_scene_mode, //ASUS_BSP LiJen "[A60K][8M][NA][Others]implement Sence mode in 8M camera with ISP"
	.sensor_set_isp_caf_mode =  fjm6mo_set_caf_mode, //ASUS_BSP LiJen "[A60K][8M][NA][Others]implement CAF mode in 8M camera with ISP"
	.sensor_set_isp_aeclock_mode =  fjm6mo_set_acelock_mode, //ASUS_BSP LiJen "[A60K][8M][NA][Others]implement AEC Lock mode in 8M camera with ISP"
	.sensor_set_isp_awblock_mode =  fjm6mo_set_awblock_mode, //ASUS_BSP LiJen "[A60K][8M][NA][Others]implement AWB Lock mode in 8M camera with ISP"
	.sensor_set_isp_iso_mode =  fjm6mo_set_iso_mode, //ASUS_BSP LiJen "[A60K][8M][NA][Others]implement ISO mode in 8M camera with ISP"
	.sensor_set_isp_flicker_mode =  fjm6mo_set_flicker_mode, //ASUS_BSP LiJen "[A60K][8M][NA][Others]implement Flicker mode in 8M camera with ISP"
       .sensor_setting = ov2720_sensor_setting,
	.sensor_set_sensor_mode = ov2720_sensor_set_sensor_mode,
	.sensor_mode_init = ov2720_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = ov2720_sensor_config,
	.sensor_power_up = ov2720_sensor_power_up,
	.sensor_power_down = ov2720_sensor_power_down,
	.sensor_adjust_frame_lines = msm_sensor_adjust_frame_lines,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
	.sensor_get_isp_exif = fjm6mo_get_exif,	//ASUS_BSP Stimber "[A60K][8M][NA][Other] Implement EXIF info for 8M camera with ISP"
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
};

//ASUS_BSP +++ Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
#if 0	//LiJen: ISP dosen't  need
static struct msm_sensor_reg_t ov2720_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = ov2720_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(ov2720_start_settings),
	.stop_stream_conf = ov2720_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(ov2720_stop_settings),
	.group_hold_on_conf = ov2720_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(ov2720_groupon_settings),
	.group_hold_off_conf = ov2720_groupoff_settings,
	.group_hold_off_conf_size =
		ARRAY_SIZE(ov2720_groupoff_settings),
	.init_settings = &ov2720_init_conf[0],
	.init_size = ARRAY_SIZE(ov2720_init_conf),
	.mode_settings = &ov2720_confs[0],
	.output_settings = &ov2720_dimensions[0],
	.num_conf = ARRAY_SIZE(ov2720_confs),
};
#endif
static struct msm_sensor_reg_t ov2720_regs = {
	.output_settings = &ov2720_dimensions[0],
	.num_conf = 3,
};
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"

struct msm_sensor_ctrl_t ov2720_s_ctrl = { //ASUS_BSP Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
	.msm_sensor_reg = &ov2720_regs,
	.sensor_i2c_client = &ov2720_sensor_i2c_client,
	.sensor_i2c_addr = (0x3E >> 1),		//ASUS_BSP Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
	.sensor_output_reg_addr = &ov2720_reg_addr,
	.sensor_id_info = &ov2720_id_info,
	.sensor_exp_gain_info = &ov2720_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.csi_params = &ov2720_csi_params_array[0],
	.msm_sensor_mutex = &ov2720_mut,
	.sensor_i2c_driver = &ov2720_i2c_driver,
	.sensor_v4l2_subdev_info = ov2720_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov2720_subdev_info),
	.sensor_v4l2_subdev_ops = &ov2720_subdev_ops,
	.func_tbl = &ov2720_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

module_init(msm_sensor_init_module);
MODULE_DESCRIPTION("Omnivision 2MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");


