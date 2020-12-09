/*
********************* (C) COPYRIGHT 2013 STMicroelectronics ********************
*
* File Name		: lsm330.h
* Authors		: MEMS Motion Sensors Products Div- Application Team
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Denis Ciocca (denis.ciocca@st.com)
* Version		: V.1.2.6.1
* Date			: 2013/Oct/03
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************

********************************************************************************
Version History.
	V 1.0.0		First Release
	V 1.0.2		I2C address bugfix
	V 1.2.0		Registers names compliant to correct datasheet
	V.1.2.1		Removed enable_interrupt_output sysfs file, manage int1
			and int2, implements int1 isr.
	V.1.2.2		Added HR_Timer and custom sysfs path
	V.1.2.3		Ch state program codes and state prog parameters define,
			gyro: corrects remove and poll_function_work
	V.1.2.5		Changes create_sysfs_interfaces
	V.1.2.6		Changes resume and suspend functions
	V.1.2.6.1	Introduce SignMotion feat implementation and solves
			acc suspend/resume issue;

********************************************************************************
SYSFS interface
- range: set full scale
	-> accelerometer: 	2,4,6,8,16 				[g]
	-> gyroscope:		250,500,2000				[dps]
- poll_period_ms: set 1/ODR
	-> accelerometer:	LSM330_ACC_MIN_POLL_PERIOD_MS < t	[ms]
	-> gyroscope:		LSM330_GYR_MIN_POLL_PERIOD_MS < t	[ms]
- enable_device: enable/disable sensor					[1/0]
- enable_polling: enable data polling/enable interrupt driven data acq	[1/0]

INPUT subsystem: NOTE-> output data INCLUDE the sensitivity in accelerometer,
			but NOT INCLUDE the sensitivity in gyroscope.
- accelerometer:	abs_x, abs_y, abs_z		[ug]
- gyroscope:		abs_x, abs_y, abs_z		[raw data]
*******************************************************************************/

#ifndef __LSM330_H__
#define __LSM330_H__


#define LSM330_ACC_DEV_NAME		"lsm330_acc"
#define LSM330_GYR_DEV_NAME		"lsm330_gyr"
#ifdef CONFIG_ENABLE_ACC_BUFFERING
#define LSM330_ACCBUF_DEV_NAME		"lsm330_accbuf"
#endif

//#define CUSTOM_SYSFS_PATH
#define CUSTOM_SYSFS_CLASS_NAME_GYR	"ST_gyr"
#define CUSTOM_SYSFS_CLASS_NAME_ACC	"ST_acc"

#define LSM330_GYR_SAD0L		(0x00)
#define LSM330_ACC_SAD0L		(0x02)
#define LSM330_SAD0H			(0x01)
#define LSM330_ACC_I2C_SADROOT		(0x07)
#define LSM330_ACC_I2C_SAD_L		((LSM330_ACC_I2C_SADROOT<<2) | \
							LSM330_ACC_SAD0L)
#define LSM330_ACC_I2C_SAD_H		((LSM330_ACC_I2C_SADROOT<<2) | \
							LSM330_SAD0H)

#define LSM330_GYR_I2C_SADROOT		(0x35)
#define LSM330_GYR_I2C_SAD_L		((LSM330_GYR_I2C_SADROOT<<1)| \
							LSM330_GYR_SAD0L)
#define LSM330_GYR_I2C_SAD_H		((LSM330_GYR_I2C_SADROOT<<1)| \
							LSM330_SAD0H)
#ifdef CONFIG_ENABLE_ACC_BUFFERING
#define LSM330_MAXSAMPLE       1000
#endif

/* Poll Interval */
#define LSM330_ACC_MIN_POLL_PERIOD_MS		1

#define LSM330_GYR_MIN_POLL_PERIOD_MS		2


#ifdef __KERNEL__
/* enable significan motion program options
 * and configurations on INT2
 * */
#define ENABLE_SIGNIFICANT_MOTION	1

/* Interrupt */
#define LSM330_ACC_DEFAULT_INT1_GPIO		(-EINVAL)
#define LSM330_ACC_DEFAULT_INT2_GPIO		(-EINVAL)

/* replace previous defines with something like following
 * if you like to have default platform_data to
 * have gpios for interrupt pins defined.
#define LSM330_ACC_DEFAULT_INT1_GPIO		134
#define LSM330_ACC_DEFAULT_INT2_GPIO		39
*/

#define LSM330_GYR_DEFAULT_INT1_GPIO		(-EINVAL)
#define LSM330_GYR_DEFAULT_INT2_GPIO		(-EINVAL)


/* Accelerometer Sensor Full Scale */
#define LSM330_ACC_G_2G				(0x00)
#define LSM330_ACC_G_4G				(0x08)
#define LSM330_ACC_G_6G				(0x10)
#define LSM330_ACC_G_8G				(0x18)
#define LSM330_ACC_G_16G			(0x20)

/* Gyroscope Sensor Full Scale */
#define LSM330_GYR_FS_250DPS			(0x00)
#define LSM330_GYR_FS_500DPS			(0x10)
#define LSM330_GYR_FS_2000DPS			(0x30)



struct lsm330_acc_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;
#ifdef CONFIG_ENABLE_ACC_BUFFERING
	unsigned int max_buffer_time;
	unsigned int report_evt_cnt;
#endif

	u8 fs_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(struct i2c_client *client);
	void (*exit)(struct i2c_client *client);
	int (*power_on)(struct i2c_client *client);
	int (*power_off)(struct i2c_client *client);

	/* set gpio_int either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	int gpio_int1;
	int gpio_int2;
};

struct lsm330_gyr_platform_data {
	int (*init)(struct i2c_client *client);
	void (*exit)(struct i2c_client *client);
	int (*power_on)(struct i2c_client *client);
	int (*power_off)(struct i2c_client *client);

	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	/* fifo related */
	u8 watermark;
	u8 fifomode;

	/* gpio ports for interrupt pads */
	int gpio_int1;
	int gpio_int2;		/* int for fifo */

	/* axis mapping */
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
};
#endif	/* __KERNEL__ */

#endif	/* __LSM330_H__ */
