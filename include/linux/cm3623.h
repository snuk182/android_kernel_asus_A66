/*
 * Definitions for CM3623 proximity sensor with ALS.
 */
#ifndef CM3623_H
#define CM3623_H

#include <linux/ioctl.h>
//#include <asm-arm/arch/regs-gpio.h>

#define CM3623_I2C_ADDRESS 		0x48 // --> (0x90 >> 1) , for CM3623A3OP-AD
//#define CM3623_I2C_ADDRESS 		0x10 // --> (0x20 >> 1)

/* CM3623 Register  (Please refer to CM3623 Specifications) */
#if 1
#define CM3623_ALS_CTL_REG			0x90 //write control cmd for ambient light sensor.
#define CM3623_ALS_DATA_REG1			0x91 //read cmd, read MSB 8 bits of ambient light data.
#define CM3623_ALS_DATA_REG2			0x93 //read cmd, read LSB 8 bits of ambient light data.
#define CM3623_PS_CTL_REG			0xF0 //write control cmd for ps sensor.
#define CM3623_PS_THD_REG			0xF2 //write threshold cmd for ps sensor.
#define CM3623_PS_DATA_REG			0xF1 //read cmd, read 8 bits of proximity data.
#define CM3623_INIT_CMD				0x92 //initialization.
#else
#define CM3623_ALS_CTL_REG			0x20 //write control cmd for ambient light sensor.
#define CM3623_ALS_DATA_REG1			0x21 //read cmd, read MSB 8 bits of ambient light data.
#define CM3623_ALS_DATA_REG2			0x23 //read cmd, read LSB 8 bits of ambient light data.
#define CM3623_PS_CTL_REG			0xB0 //write control cmd for ps sensor.
#define CM3623_PS_THD_REG			0xB2 //write threshold cmd for ps sensor.
#define CM3623_PS_DATA_REG			0xB1 //read cmd, read 8 bits of proximity data.
#define CM3623_INIT_CMD				0x22 //initialization.
#endif

#define CM3623_ARA_REG				0x18 //write cmd, Alert Response Address (used for INT_MODE)

/* Bit Description for each cmd */
#define SD_ALS	0	//bit 0 in 0x20 or 0x90
#define SD_PS	0	//bit 0 in 0xB0 or 0xF0
#define INT_MODE	0x10	// cm3623 INT pin is triggered by interrupt-based mechanism.
#define NORMAL_MODE	0x20	// cm3623 INT pin is controlled by a simple proximity logic high/low.
#define INIT_CM3623 	NORMAL_MODE
#define INIT_ALS	0x0
#define INIT_PS		0x0
#define DEFAULT_PS_THRESHOLD	9

#define IT_ALS_BIT	2
#define IT_ALS_100MS	(0<<IT_ALS_BIT)
#define IT_ALS_200MS	(1<<IT_ALS_BIT)
#define IT_ALS_400MS	(2<<IT_ALS_BIT)
#define IT_ALS_800MS	(3<<IT_ALS_BIT)

#define GAIN_ALS_BIT	6
#define GAIN_ALS_DIV1	(0<<GAIN_ALS_BIT)
#define GAIN_ALS_DIV2	(1<<GAIN_ALS_BIT)
#define GAIN_ALS_DIV4	(2<<GAIN_ALS_BIT)
#define GAIN_ALS_DIV8	(3<<GAIN_ALS_BIT)

#define ALS_RESOLUTION_180	(0x00 | IT_ALS_100MS | GAIN_ALS_DIV1)
#define ALS_RESOLUTION_90	(0x00 | IT_ALS_100MS | GAIN_ALS_DIV2)
#define ALS_RESOLUTION_45	(0x00 | IT_ALS_100MS | GAIN_ALS_DIV4)
#define ALS_RESOLUTION_22	(0x00 | IT_ALS_100MS | GAIN_ALS_DIV8)
#define ALS_RESOLUTION_11	(0x00 | IT_ALS_200MS | GAIN_ALS_DIV8)
#define ALS_RESOLUTION_5	(0x00 | IT_ALS_400MS | GAIN_ALS_DIV8)
#define ALS_RESOLUTION_2	(0x00 | IT_ALS_800MS | GAIN_ALS_DIV8)

enum cm3623_sensors {
	PS_SENSOR=0,
	ALS_SENSOR,
};

struct cm3623_platform_data {
    int   (*init_platform_hw)(struct i2c_client *client);
    int   (*exit_platform_hw)(struct i2c_client *client);
    u8    (*read_int_pin_state)(void);
};

/* IOCTLs for KXTF9 misc. device library */
/*
#define KXTF9IO						   	0x86
#define KXTF9_IOCTL_INIT                  		_IO(KXTF9IO, 0x01)
#define KXTF9_IOCTL_READ_CHIPINFO         	_IOR(KXTF9IO, 0x02, int)
#define KXTF9_IOCTL_READ_SENSORDATA     	_IOR(KXTF9IO, 0x03, int)
#define KXTF9_IOCTL_READ_POSTUREDATA    	_IOR(KXTF9IO, 0x04, int)
#define KXTF9_IOCTL_READ_CALIDATA         	_IOR(KXTF9IO, 0x05, int)
#define KXTF9_IOCTL_READ_CONTROL          	_IOR(KXTF9IO, 0x06, int)
#define KXTF9_IOCTL_SET_CONTROL           	_IOW(KXTF9IO, 0x07, int)
#define KXTF9_IOCTL_SET_MODE              		_IOW(KXTF9IO, 0x08, int)
*/

/* IOCTLs for TEST tools */
#define LS_IOC_MAGIC					0xf4
#define READ_LIGHT_DATA_ON_OFF				_IOW(LS_IOC_MAGIC, 1, int)
#define ATD_ASK_LIGHTDATA 				_IOW(LS_IOC_MAGIC, 6, int*) 
#define IOCTL_ENABLE_CTRL 				_IOW(LS_IOC_MAGIC, 7, int)
#define CM3623_ANDROID_OR_OMS				_IOW(LS_IOC_MAGIC, 8, int)

#define PS_IOC_MAGIC					0xf5
#define PS_POLL_READ_GPIO_ON_OFF			_IOW(PS_IOC_MAGIC, 1, int)
#define ATD_ASK_PR_STATUS 				_IOW(PS_IOC_MAGIC, 6, int*)


#ifndef DBGMSK_PRO_G0
#define DBGMSK_PRO_G0	KERN_INFO
#define DBGMSK_PRO_G1	KERN_INFO
#define DBGMSK_PRO_G2	KERN_INFO
#define DBGMSK_PRO_G3	KERN_INFO
#define DBGMSK_PRO_G4	KERN_INFO
#define DBGMSK_PRO_G5	KERN_INFO
#define DBGMSK_PRO_G6	KERN_INFO
#define DBGMSK_PRO_G7	KERN_INFO
#endif

#endif

