
//                     ASUSTek Computer Inc.
//         Copyright (c) 2010 ASUSTek Computer inc, Taipei.

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <mach/gpio.h>

#include <linux/cdev.h>
#include "linux/p02_scaler.h"

static struct i2c_client *p02_scaler_update_client;

extern bool i2c_error;
extern void reportPadStationI2CFail(char *devname);

extern int isMicroPConnected(void);
extern int AX_MicroP_readBattCapacity(int target);
extern int set_als_power_state_of_P01(int state);
static struct class *driver_class;
static dev_t scaler_device_no;
static struct cdev scaler_cdev;

#define SCALER_DEV "scaler_update"

#define  _P_FLASH_BLOCK_ERASE_OPCODE  0xD8
#define  _P_FLASH_WREN_OPCODE         0x06
#define  _P_FLASH_WRSR_OPCODE         0x01

#define  _SCALAR_PAGE_SEL             0x9F
#define  _SCALAR_PAGE_0x10            0x10
#define  _SCALAR_PINSHARE_CTRL0E      0xAE
#define  _SCALAR_PORT81_PIN_REG       0xD7

#define _ENTER_ISP_COUNT 100

unsigned char m_byCodeData[256][256];
int  m_nProgramLength = 65536;

extern u8 g_scaler_ver;

static int scaler_i2c_read(u8 addr, int len, void *data)
{
	int i=0;
	int retries=6;
	int status=0;

	struct i2c_msg msg[] = {
		{
			.addr = p02_scaler_update_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = p02_scaler_update_client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	do{    
		status = i2c_transfer(p02_scaler_update_client->adapter, msg, ARRAY_SIZE(msg));
		if ((status < 0) && (i < retries)){
			msleep(5);
			printk("[p02_scaler_update] %s retry %d\r\n", __FUNCTION__, i);                                
			i++;
		}
	} while ((status < 0) && (i < retries));
        
	if(status < 0){
		printk("[p02_scaler_update] i2c read error %d\n", status);
	}
	
	return status;
}

static int scaler_i2c_write(u8 addr, int len, void *data)
{
	int i=0;
	int status=0;
	u8 buf[len + 1];
	struct i2c_msg msg[] = {
		{
		.addr = p02_scaler_update_client->addr,
		.flags = 0,
		.len = len + 1,
		.buf = buf,
		},
	};
	int retries = 6;

	buf[0] = addr;
	memcpy(buf + 1, data, len);

	do {
		status = i2c_transfer(p02_scaler_update_client->adapter, msg, ARRAY_SIZE(msg));
		if ((status < 0) && (i < retries)){
			msleep(5);                    
			printk("[p02_scaler_update] %s retry %d\r\n", __FUNCTION__, i);
			i++;
		}
	} while ((status < 0) && (i < retries));

	if (status < 0) {
		printk("[p02_scaler_update] i2c write error %d \n", status);
	}

	return status;
}

int RTDToolWrite(unsigned char bySlaveAddr, unsigned char bySubAddr, unsigned char* pBuf, int wTotalLen, int wLenOnce, bool bIncrease)
{
	scaler_i2c_write(bySubAddr, wTotalLen, pBuf);
	return 0; 
}

int RTDToolRead(unsigned char bySlaveAddr, unsigned char bySubAddr, unsigned char* pBuf, int wTotalLen, int wLenOnce, bool bIncrease)
{
	scaler_i2c_read(bySubAddr, wTotalLen, pBuf);
	return 0;
}

bool WaitForBitChange(unsigned char bySlave, unsigned char byRegister, unsigned char byAnd, int nWaitCount)
{
    unsigned char byBuf[16] = {0};
    int nCount = 0;
	
    // wait for read operation allowed
	RTDToolRead(bySlave,byRegister,byBuf,1,1,false);
    nCount = 0;
    while (byBuf[0] & byAnd)
    {
		msleep(5);
		RTDToolRead(bySlave,byRegister,byBuf,1,1,false);
        if (nCount > nWaitCount)
        {
            return false;
        }
        nCount++;
    }
	
    return true;
}

bool SerialFlashIntoISPMode(void)
{
	unsigned char byBuf[16] = {0};
	int bReturn = 0;

	//ISP enable,Set 0x6F[7] = 1
	byBuf[0] = 0x94;
	byBuf[1] = 0x6F;
	byBuf[2] = 0x80;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	//Disable Watchdog Timer
	byBuf[0] = 0x94;
	byBuf[1] = 0xea;
	byBuf[2] = 0x00;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	//add by william 20090911
	byBuf[0] = 0x00;
	RTDToolRead(0x94,0x6F,byBuf,1,1,false);
	msleep(100);
	if ((byBuf[0] & 0x80) == 0x80)
	{
		// Set Interanl OSC divider register to default to speed up MCU start
		byBuf[0] = 0x94;
		byBuf[1] = 0xf4;
		byBuf[2] = 0x9f;
		RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

		byBuf[0] = 0x94;
		byBuf[1] = 0xf5;
		byBuf[2] = 0x06;
		RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

		byBuf[0] = 0x94;
		byBuf[1] = 0xf4;
		byBuf[2] = 0xa0;
		RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

		byBuf[0] = 0x94;
		byBuf[1] = 0xf5;
		byBuf[2] = 0x74;
		RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
		// Set Interanl OSC divider register to default to speed up MCU end
		bReturn = true;
	}
	else
	{
		bReturn = false;
	}

	return bReturn;
}

int SerialFlashBankReadCRC(unsigned char ucBankIndex)
{
	unsigned char byBuf[16] = {0};
    int nTimeOut = 0;
	unsigned char ucI2CBuf[1]= {0};
	
	//Set start address
    byBuf[0] = 0x94;
    byBuf[1] = 0x64;
    byBuf[2] = ucBankIndex;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	byBuf[0] = 0x94;
    byBuf[1] = 0x65;
    byBuf[2] = 0x00;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	byBuf[0] = 0x94;
    byBuf[1] = 0x66;
    byBuf[2] = 0x00;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
    
	//Set end address
    byBuf[0] = 0x94;
    byBuf[1] = 0x72;
    byBuf[2] = ucBankIndex;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	byBuf[0] = 0x94;
    byBuf[1] = 0x73;
    byBuf[2] = 0xFF;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	byBuf[0] = 0x94;
    byBuf[1] = 0x74;
    byBuf[2] = 0xFF;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

    
	RTDToolRead(0x94,0x6F,ucI2CBuf,1,1,false);
    byBuf[0] = 0x94;
    byBuf[1] = 0x6F;
    byBuf[2] = (unsigned char)(0x84|ucI2CBuf[0]);  // CRC start
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	
    msleep(5);
    
	RTDToolRead(0x94,0x6F,byBuf,1,1,false);
	
    nTimeOut = 0;
    while(byBuf[0] & 0x04)
    {
		msleep(1);
		RTDToolRead(0x94,0x6F,byBuf,1,1,false);
		
        if (byBuf[0] & 0x02)
        {
            break;
        }
        if (nTimeOut > 200) 
        {
            return -1;
        }
        nTimeOut++;
    }
	
	RTDToolRead(0x94,0x75,byBuf,1,1,false); //Read CRC
	
    return byBuf[0];
}

void DisableFlashWriteProtection(void)
{
	unsigned char byBuf[16]={0};

	//Disable Flash softeware write protection
	//%%%%%%%%% WREN instruction %%%%%%%%%%
	byBuf[0] = 0x94;
	byBuf[1] = 0x60;
	byBuf[2] = 0x20;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	
	byBuf[0] = 0x94;
	byBuf[1] = 0x61;
	byBuf[2] = _P_FLASH_WREN_OPCODE;//0x06;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	
	byBuf[0] = 0x94;
	byBuf[1] = 0x60;
	byBuf[2] = 0x21;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	
	// wait for wren instruction finish        
	if (!WaitForBitChange(0x94, 0x60, 0x01, 100))
	{
		return ;
	}
	//%%%%%%%%% WREN instruction %%%%%%%%%%
	
	//%%%%%%%%%%%%% WRSR instruction %%%%%%%%%%
	//set WRSR
	byBuf[0] = 0x94;
	byBuf[1] = 0x60;
	byBuf[2] = 0x68;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	
	//WRSR OP        
	byBuf[0] = 0x94;
	byBuf[1] = 0x61;
	byBuf[2] = _P_FLASH_WRSR_OPCODE;//0x01;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	
	//disable software WR protect        
	byBuf[0] = 0x94;
	byBuf[1] = 0x64;
	byBuf[2] = 0x02;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	
	byBuf[0] = 0x94;
	byBuf[1] = 0x60;
	byBuf[2] = 0x69;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	
	// wait for Write Status finish        
	if (!WaitForBitChange(0x94, 0x60, 0x01, 100))
	{
		return ;
	}
	//%%%%%%%%%%%%% WRSR instruction %%%%%%%%%%

	/*//Debug
	//RDSR
	byBuf[0] = 0x94;
	byBuf[1] = 0x60;
	byBuf[2] = 0x42;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,FALSE);

	byBuf[0] = 0x94;
	byBuf[1] = 0x61;
	byBuf[2] = 0x05;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,FALSE);

	byBuf[0] = 0x94;
	byBuf[1] = 0x60;
	byBuf[2] = 0x43;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,FALSE);

	byBuf[0] = 0x00;
	RTDToolRead(0x94,0x67,byBuf,1,1,FALSE);
	//Debug
	*/
	
	return ;
}

void EnableFlashWriteProtection(void)
{
	unsigned char byBuf[16]={0};

	//Enable Flash softeware write protection
	//%%%%%%%%% WREN instruction %%%%%%%%%%
	byBuf[0] = 0x94;
	byBuf[1] = 0x60;
	byBuf[2] = 0x20;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	
	byBuf[0] = 0x94;
	byBuf[1] = 0x61;
	byBuf[2] = _P_FLASH_WREN_OPCODE;//0x06;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	
	byBuf[0] = 0x94;
	byBuf[1] = 0x60;
	byBuf[2] = 0x21;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	
	// wait for wren instruction finish        
	if (!WaitForBitChange(0x94, 0x60, 0x01, 100))
	{
		return ;
	}
	//%%%%%%%%% WREN instruction %%%%%%%%%%
	//%%%%%%%%%%%%% WRSR instruction %%%%%%%%%%
	//set WRSR
	byBuf[0] = 0x94;
	byBuf[1] = 0x60;
	byBuf[2] = 0x68;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	
	//WRSR OP  code      
	byBuf[0] = 0x94;
	byBuf[1] = 0x61;
	byBuf[2] = _P_FLASH_WRSR_OPCODE;//0x01;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	
	//Set flash status reg     
	byBuf[0] = 0x94;
	byBuf[1] = 0x64;
	byBuf[2] = 0x7C;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	
	byBuf[0] = 0x94;
	byBuf[1] = 0x60;
	byBuf[2] = 0x69;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	
	// wait for Write Status finish        
	if (!WaitForBitChange(0x94, 0x60, 0x01, 100))
	{
		return ;
	}
	//%%%%%%%%%%%%% WRSR instruction %%%%%%%%%%

	/*//Debug
	//RDSR
	byBuf[0] = 0x94;
	byBuf[1] = 0x60;
	byBuf[2] = 0x42;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,FALSE);
	
	byBuf[0] = 0x94;
	byBuf[1] = 0x61;
	byBuf[2] = 0x05;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,FALSE);
	
	byBuf[0] = 0x94;
	byBuf[1] = 0x60;
	byBuf[2] = 0x43;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,FALSE);
	
	byBuf[0] = 0x00;
	RTDToolRead(0x94,0x67,byBuf,1,1,FALSE);
	//Debug
	*/
	
	return ;
}

bool BlockErase(unsigned char ucBankIndex)
{
    unsigned char byBuf[16] = {0};

    //ISP enable 
    byBuf[0] = 0x94;
    byBuf[1] = 0x6F;
    byBuf[2] = 0x80;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	DisableFlashWriteProtection();

	//Block Erase  
	//set start address
	byBuf[0] = 0x94;
	byBuf[1] = 0x64;
	byBuf[2] = ucBankIndex;                
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	byBuf[0] = 0x94;
	byBuf[1] = 0x65;
	byBuf[2] = 0x00;                
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	byBuf[0] = 0x94;
	byBuf[1] = 0x66;
	byBuf[2] = 0x00;                
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	//Command Erase
	byBuf[0] = 0x94;
	byBuf[1] = 0x60;
	byBuf[2] = 0xB8;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	//Block Erase OP code,it depends on flash type
	byBuf[0] = 0x94;
	byBuf[1] = 0x61;
	byBuf[2] = _P_FLASH_BLOCK_ERASE_OPCODE;//0xD8;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	//Common instruction enable    
	byBuf[0] = 0x94;
	byBuf[1] = 0x60;
	byBuf[2] = 0xB9;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	// wait for Erase finish 
	if (!WaitForBitChange(0x94, 0x60, 0x01, 200))
	{
		return false;
	}	

	EnableFlashWriteProtection();
	
    return true;
}

void DisableFlashHardwareWriteProtect(void)
{
	//Take RD series as an example
	//Assume Pin108 of 2482RD scalar IC is connected to WP# Pin of SerialFlash
	//We should pull pin108 high to close serialflash hardware writeprotection  
	unsigned char byBuf[16] = {0};
	
	byBuf[0] = 0x94;
	byBuf[1] = 0xF4;
	byBuf[2] = 0x8B;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	byBuf[0] = 0x94;
	byBuf[1] = 0xF5;
	byBuf[2] = 0xC3;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	byBuf[0] = 0x94;
	byBuf[1] = 0xF4;
	byBuf[2] = 0x8C;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	byBuf[0] = 0x94;
	byBuf[1] = 0xF5;
	byBuf[2] = 0x02;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
    

	//page 10
	byBuf[0] = 0x94;
	byBuf[1] = 0xF4;
	byBuf[2] = _SCALAR_PAGE_SEL;//0x9F;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	byBuf[0] = 0x94;
	byBuf[1] = 0xF5;
	byBuf[2] = _SCALAR_PAGE_0x10;//0x10;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	//Config Pin
	byBuf[0] = 0x94;
	byBuf[1] = 0xF4;
	byBuf[2] = _SCALAR_PINSHARE_CTRL0E;//0xAE;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	byBuf[0] = 0x94;
	byBuf[1] = 0xF5;
	byBuf[2] = 0x01;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	//Set Pin Value
	byBuf[0] = 0x94;
	byBuf[1] = _SCALAR_PORT81_PIN_REG;//0xD7;
	byBuf[2] = 0x01;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
}

bool SetDefaultValueToReg(void)
{
	unsigned char byBuf[16] = {0};
	unsigned char ucI2CBuf[3]={0};
	int nTimeCount=0;
	
    //DVI DDC some registers
    byBuf[0] = 0x94;
    byBuf[1] = 0x1E;
    byBuf[2] = 0x02;
    RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);  
    
    byBuf[0] = 0x94;
    byBuf[1] = 0x1F;
    byBuf[2] = 0x00;
    RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

    byBuf[0] = 0x94;
    byBuf[1] = 0x20;
    byBuf[2] = 0x1C;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

    //HDMI DDC some registers
    byBuf[0] = 0x94;
    byBuf[1] = 0x2C;
    byBuf[2] = 0x02;
    RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

    byBuf[0] = 0x94;
    byBuf[1] = 0x2D;
    byBuf[2] = 0x00;
    RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

    byBuf[0] = 0x94;
    byBuf[1] = 0x2E;
    byBuf[2] = 0x1C;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	//ADC DDC some registers
	byBuf[0] = 0x94;
	byBuf[1] = 0x1B;
	byBuf[2] = 0x02;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	byBuf[0] = 0x94;
	byBuf[1] = 0x1C;
	byBuf[2] = 0x00;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	byBuf[0] = 0x94;
	byBuf[1] = 0x1D;
    byBuf[2] = 0x1C;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	byBuf[0] = 0x94;
	byBuf[1] = 0x6A;
	byBuf[2] = 0x03;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	byBuf[0] = 0x94;
	byBuf[1] = 0x6B;
	byBuf[2] = 0x0b;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	byBuf[0] = 0x94;
	byBuf[1] = 0x6C;
	byBuf[2] = 0x00;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	byBuf[0] = 0x94;
    byBuf[1] = 0x62;
    byBuf[2] = 0x06;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	byBuf[0] = 0x94;
    byBuf[1] = 0x63;
    byBuf[2] = 0x50;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	//Default 0xFFED register
	byBuf[0] = 0x94;
	byBuf[1] = 0xED;
	byBuf[2] = 0x88;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	//Default 0xFFEE register
	byBuf[0] = 0x94;
	byBuf[1] = 0xEE;
	byBuf[2] = 0x04;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	// check register 0xED and 0xEE
	nTimeCount=0;  
	ucI2CBuf[0]=ucI2CBuf[1]=0x00;
	while(nTimeCount < 10 && !((ucI2CBuf[0]==0x88)&&(ucI2CBuf[1]==0x04)))
	{
		nTimeCount++;
		
		RTDToolRead(0x94,0xED,byBuf,1,1,false);
		msleep(100);
		ucI2CBuf[0]=byBuf[0];

		RTDToolRead(0x94,0xEE,byBuf,1,1,false);
		msleep(100);
		ucI2CBuf[1]=byBuf[0];

		msleep(200);
	}
	
	return (nTimeCount < 10);  	
}

bool SerialFlashWriteCode(unsigned char ucBankIndex)
{
	//SetRightValueToReg();

	unsigned char byBuf[16] = {0}, byLow = 0, byHigh = 0;
	int i = 0, nCount = 0, nHigh = 0;

	printk("[p02_scaler_update] SerialFlashWriteCode ucBankIndex = %d\n", ucBankIndex);
	//ISP_EN
	byBuf[0] = 0x94;
	byBuf[1] = 0x6F;
	byBuf[2] = 0x80;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	DisableFlashWriteProtection();

	//Prog_OP
	byBuf[0] = 0x94;
	byBuf[1] = 0x6D;
	byBuf[2] = 0x02;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	byLow = (unsigned char)((m_nProgramLength % 256) - 1);
	if (byLow == 0xFF)
	{
		byHigh = (unsigned char)(m_nProgramLength / 256 - 1);
	}
	else
	{
		byHigh = (unsigned char)(m_nProgramLength / 256);
	}
	nHigh = byHigh + 1;

	// set program length = 256   
	byBuf[0] = 0x94;
	byBuf[1] = 0x71;
	byBuf[2] = 0xFF;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	// set start address, BANK number
	byBuf[0] = 0x94;
	byBuf[1] = 0x64;
	byBuf[2] = ucBankIndex;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	byBuf[0] = 0x94;
	byBuf[1] = 0x65;
	byBuf[2] = 0x00;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	byBuf[0] = 0x94;
	byBuf[1] = 0x66;
	byBuf[2] = 0x00;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

	for (i=0; i<nHigh; i++)
	{
		//////////////////////////////////////////////////////////////////////////
		// wait for buf empty
		RTDToolRead(0x94,0x6F,byBuf,1,1,false);
		nCount=0;
		while ((byBuf[0] & 0x10) != 0x10)    
		{
			RTDToolRead(0x94,0x6F,byBuf,1,1,false);
			if (nCount > 100)
			{
				return false;
			}
			nCount++;
		}
		//////////////////////////////////////////////////////////////////////////

		if (nCount <= 100)
		{
			RTDToolWrite(0x94, 0x70, m_byCodeData[i], 256, 256, false);
		}
		else
		{
			return false;
		}

		//start program 
		byBuf[0] = 0x94;
		byBuf[1] = 0x6F;
		byBuf[2] = 0xA0;
		RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);

		//Polling waiting for page program finished
		if (!WaitForBitChange(0x94, 0x6F, 0x20, 150))
		{
			return false;
		}
	}

	EnableFlashWriteProtection();
	//SetWrongValueToReg();

    return true;
}

int SerialFlashCalculateCRC(unsigned char ucBankIndex)
{
    unsigned char byLow = 0, byHigh = 0;
    int i = 0, j = 0, k = 0, nHigh = 0;
    bool bCrcArr[8] = {false}, bDinArr[8] = {false}, bTempArr[8] = {false};
    
    byLow = (unsigned char)((m_nProgramLength % 256) - 1);
    
    if (byLow == 0xFF)
    {
        byHigh = (unsigned char)(m_nProgramLength / 256 - 1);
    }
    else
    {
        byHigh = (unsigned char)(m_nProgramLength / 256);
    }
    nHigh = byHigh + 1;

    for (i=0; i<nHigh; i++)
    {
        for (j=0; j<256; j++)
        {
            bDinArr[0] = (bool)(m_byCodeData[i][j] & 0x01);
            bDinArr[1] = (bool)(m_byCodeData[i][j] & 0x02);
            bDinArr[2] = (bool)(m_byCodeData[i][j] & 0x04);
            bDinArr[3] = (bool)(m_byCodeData[i][j] & 0x08);
            bDinArr[4] = (bool)(m_byCodeData[i][j] & 0x10);
            bDinArr[5] = (bool)(m_byCodeData[i][j] & 0x20);
            bDinArr[6] = (bool)(m_byCodeData[i][j] & 0x40);
            bDinArr[7] = (bool)(m_byCodeData[i][j] & 0x80);
			
            for (k=0; k<8; k++)
            {
                if (bDinArr[k] )
                {
                    bDinArr[k] = true;
                }
                else
                {
                    bDinArr[k] = false;
                }
            }
			
            bTempArr[0]= bDinArr[7]^bDinArr[6]^bDinArr[0]^bCrcArr[0]^bCrcArr[6]^bCrcArr[7];
            bTempArr[1]= bDinArr[6]^bDinArr[1]^bDinArr[0]^bCrcArr[0]^bCrcArr[1]^bCrcArr[6];
            bTempArr[2]= bDinArr[6]^bDinArr[2]^bDinArr[1]^bDinArr[0]^bCrcArr[0]^bCrcArr[1]^bCrcArr[2]^bCrcArr[6];
            bTempArr[3]= bDinArr[7]^bDinArr[3]^bDinArr[2]^bDinArr[1]^bCrcArr[1]^bCrcArr[2]^bCrcArr[3]^bCrcArr[7];
            bTempArr[4]= bDinArr[4]^bDinArr[3]^bDinArr[2]^bCrcArr[2]^bCrcArr[3]^bCrcArr[4];
            bTempArr[5]= bDinArr[5]^bDinArr[4]^bDinArr[3]^bCrcArr[3]^bCrcArr[4]^bCrcArr[5];
            bTempArr[6]= bDinArr[6]^bDinArr[5]^bDinArr[4]^bCrcArr[4]^bCrcArr[5]^bCrcArr[6];
            bTempArr[7]= bDinArr[7]^bDinArr[6]^bDinArr[5]^bCrcArr[5]^bCrcArr[6]^bCrcArr[7];
            
            for (k=0; k<8; k++)
            {
                bCrcArr[k] = bTempArr[k];
            }
            
            if (i == (nHigh-1))
            {
                if (j == byLow)
                {
                    break;
                }
            }
        }
    }
	
    i = 0;
    for (k=0; k<8; k++)
    {
        if (bCrcArr[k])
        {
            i = i + (1<<k);
        }
    }
	
    return i;
}

int SerialFlashReadCRC(unsigned char ucBankIndex)
{
    unsigned char byLow = 0, byHigh = 0, byBuf[16] = {0};
    int nTimeOut = 0;
	unsigned char ucI2CBuf[1]= {0};
	
    byLow = (unsigned char)((m_nProgramLength % 256) - 1);
    if (byLow == 0xFF)
    {
        byHigh = (unsigned char)((m_nProgramLength / 256) - 1);
    }
    else
    {
        byHigh = (unsigned char)(m_nProgramLength / 256);
    }

	//Set start address
    byBuf[0] = 0x94;
    byBuf[1] = 0x64;
    byBuf[2] = ucBankIndex;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	byBuf[0] = 0x94;
    byBuf[1] = 0x65;
    byBuf[2] = 0x00;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	byBuf[0] = 0x94;
    byBuf[1] = 0x66;
    byBuf[2] = 0x00;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
    
    byBuf[0] = 0x94;
    byBuf[1] = 0x72;
    byBuf[2] = ucBankIndex;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	byBuf[0] = 0x94;
    byBuf[1] = 0x73;
    byBuf[2] = byHigh;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	byBuf[0] = 0x94;
    byBuf[1] = 0x74;
    byBuf[2] = byLow;
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
    
	RTDToolRead(0x94,0x6F,ucI2CBuf,1,1,false);
    byBuf[0] = 0x94;
    byBuf[1] = 0x6F;
    byBuf[2] = (unsigned char)(0x84|ucI2CBuf[0]);  // CRC start
	RTDToolWrite(byBuf[0],byBuf[1],byBuf+2,1,1,false);
	
    msleep(5);
    
	RTDToolRead(0x94,0x6F,byBuf,1,1,false);
	
    nTimeOut = 0;
    while(byBuf[0] & 0x04)
    {
		msleep(1);
		RTDToolRead(0x94,0x6F,byBuf,1,1,false);
		
        if (byBuf[0] & 0x02)
        {
            break;
        }
        if (nTimeOut > 200) 
        {
            return -1;
        }
        nTimeOut++;
    }
	
	RTDToolRead(0x94,0x75,byBuf,1,1,false); //Read CRC
	
    return byBuf[0];
}

bool SerialFlash_ISP_Flow(void)
{
	unsigned char ucCount = 0;
	struct file *fp = NULL;
	loff_t pos = 0;
	mm_segment_t old_fs;
	char firmware_file_name_bin[80];
	unsigned char *buf = NULL;
	unsigned long file_len = 0;
	int nReadBankCRC;
	int nCalculateCRC = 0;
	int nReadCRC = 0;
	
	//First Into ISP Mode
	while(ucCount < _ENTER_ISP_COUNT)
	{
		ucCount++;
		if(SerialFlashIntoISPMode()){
			printk("[p02_scaler_update] SerialFlashIntoISPMode success\n");
			break;
		}
	}

    if(ucCount >= _ENTER_ISP_COUNT){
        printk("[p02_scaler_update] SerialFlashIntoISPMode fail\n");
        return false;
    }
	

	//Set Default Value to some MCU Regs which may be modified by FW
	printk("[p02_scaler_update] SetDefaultValueToReg\n");
	if(!SetDefaultValueToReg())
	{
		printk("[p02_scaler_update] SetDefaultValueToReg fail\n");
		return false;
	}

    //Disable SerialFlash Hardware WriteProtection
    printk("[p02_scaler_update] DisableFlashHardwareWriteProtect\n");
	DisableFlashHardwareWriteProtect();

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	strcpy(firmware_file_name_bin, "/data/media/scaler_fw/A60K-PAD-scaler.bin");
	
	fp = filp_open(firmware_file_name_bin, O_RDONLY, 0);
	if(IS_ERR_OR_NULL(fp)){
		printk("[p02_scaler_update] SerialFlash_ISP_Flow open (%s) fail\n", firmware_file_name_bin);
		return false;
	}
		
	buf = kmalloc(128*1024, GFP_KERNEL);
	if(buf==NULL) {
		printk("[p02_scaler_update] allocate mem for file fail\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		return false;
	}
	memset(buf, 0xFF, 128*1024);

	file_len = fp->f_op->read(fp, buf, 128*1024, &pos);
	if(file_len < 0) {
		printk("[p02_scaler_update]read file error\n");
		set_fs(old_fs);
		filp_close(fp, NULL);
		kfree(buf);
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);
		
	//Block Erase
	if(!BlockErase(1)){
		printk("[p02_scaler_update] BlockErase 1 fail\n");
		return false;
	}
		
	//Check Block Erase successfull
	nReadBankCRC = SerialFlashBankReadCRC(1);
	if (nReadBankCRC != 0xDE){
		printk("[p02_scaler_update] Check Block Erase 1 fail, nReadBankCRC = 0x%x\n", nReadBankCRC);
		return false;
	}
		
	//Block Erase
	if(!BlockErase(0)){
		printk("[p02_scaler_update] BlockErase 0 fail\n");
		return false;
	}
		
	//Check Block Erase successfull
	nReadBankCRC = SerialFlashBankReadCRC(0);
	if (nReadBankCRC != 0xDE){
		printk("[p02_scaler_update] Check Block Erase 0 fail, nReadBankCRC = 0x%x\n", nReadBankCRC);
		return false;
	}
		
	memset(m_byCodeData, 0xFF, 65536);
	memcpy(m_byCodeData, buf+65536, 65536);
	//Program Bank
	SerialFlashWriteCode(1);

	//Check CRC
	nCalculateCRC = SerialFlashCalculateCRC(1);
	nReadCRC = SerialFlashReadCRC(1);
	if (nCalculateCRC != nReadCRC)
	{
		printk("[p02_scaler_update] nCalculateCRC 1 = 0x%x\n", nCalculateCRC);
		printk("[p02_scaler_update] nReadCRC 1 = 0x%x\n", nReadCRC);
		return false;
	}
		
	memset(m_byCodeData, 0xFF, 65536);
	memcpy(m_byCodeData, buf, 65536);
	//Program Bank
	SerialFlashWriteCode(0);

	//Check CRC
	nCalculateCRC = SerialFlashCalculateCRC(0);
	nReadCRC = SerialFlashReadCRC(0);
	if (nCalculateCRC != nReadCRC)
	{
		printk("[p02_scaler_update] nCalculateCRC 0 = 0x%x\n", nCalculateCRC);
		printk("[p02_scaler_update] nReadCRC 0 = 0x%x\n", nReadCRC);
		return false;
	}
			
	return true;
}

int HexString2Value(unsigned char* pData, unsigned char byCount)
{
	int i = 0, nReturn = 0;
	unsigned char byTemp = 0;

	printk("[p02_scaler_update] HexString2Value: pData = %s\n", pData);
	printk("[p02_scaler_update] HexString2Value: byCount = %d\n", byCount);
	
    for (i=0; i<byCount; i++)
    {
        byTemp = *(pData+i);
        if ((byTemp >= 'A') && (byTemp <= 'F'))
        {
            nReturn = nReturn * 16 + (byTemp - 'A') + 10;
        }
        else if ((byTemp >= 'a') && (byTemp <= 'f'))
        {
            nReturn = nReturn * 16 + (byTemp - 'a') + 10;
        }
        else if ((byTemp >= '0') && (byTemp <= '9'))
        {
            nReturn = nReturn * 16 + (byTemp - '0');
        }
        else
        {
            return -1;
        }
    }
	
    return nReturn;
}

static long scaler_ioctl(struct file *file, unsigned cmd, unsigned long arg)
{
	int ret = 0;
	int pad_cap = 0;
	
	printk("[p02_scaler_update] scaler_ioctl +++\r\n");
           
	if(_IOC_TYPE(cmd) != ASUS_SCALER_IOC_TYPE)
		return -ENOTTY;
	if(_IOC_NR(cmd) > ASUS_SCALER_MAX_NR)
		return -ENOTTY;
       
	switch (cmd) {
		case ASUS_SCALER_FW_UPDATE:
			printk("[p02_scaler_update] ioctl: ASUS_SCALER_FW_UPDATE ++\r\n");
			set_als_power_state_of_P01(0);
			pad_cap = AX_MicroP_readBattCapacity(0);
			if(pad_cap > 10) {
				if(SerialFlash_ISP_Flow() == false){
					ret = -EAGAIN;
					printk("[p02_scaler_update] ioctl fail, please try again!\n");
				}
			}
			else{
				printk("[p02_scaler_update] ioctl: ASUS_SCALER_FW_UPDATE pad capacity less than 15\r\n");
				ret= -EINVAL;
			}
			set_als_power_state_of_P01(1);
			pr_info("[p02_scaler_update] ioctl: ASUS_SCALER_FW_UPDATE --\r\n");
			break;
		case ASUS_SCALER_GET_FW_VERSION:
			ret=__put_user(g_scaler_ver, (int __user *)arg);
			break;
		default:
			pr_err("Invalid ioctl code\r\n");
			ret= -EINVAL;
			break;
	}
	
	return ret;
}

static int scaler_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	
	printk("[p02_scaler_update] scaler_open+++\n");
	return ret;
}

static int scaler_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	
	printk("[p02_scaler_update] scaler_release+++\n");
	return ret;
}

static int scaler_ver_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int scaler_ver = (int)g_scaler_ver;
	sprintf(page, "%d\n", scaler_ver);
	return strlen(page);
}

static int p02_scaler_update_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
	struct proc_dir_entry *entry;
    
	printk("[p02_scaler_update] p02_scaler_update_probe\n");
	p02_scaler_update_client = client;

	entry = create_proc_entry("scaler_ver", 0666, NULL);
	if (entry == NULL) {
		printk("[p02_scaler_update] create_proc_entry fail\r\n");
	}
	else {
		entry->read_proc = scaler_ver_read_proc;
	}

    return 0;
}

static const struct i2c_device_id p02_scaler_update_id[] = {
    { "p02_scaler_update", 0 },
    { }
};

static struct i2c_driver p02_scaler_update_i2c_driver = {
    .driver = {
        .name   = "p02_scaler_update",
    },
    .id_table   = p02_scaler_update_id,
    .probe      = p02_scaler_update_probe,

};

static const struct file_operations scaler_fops = {
		.owner = THIS_MODULE,
		.unlocked_ioctl = scaler_ioctl,
		.open = scaler_open,
		.release = scaler_release
};

static int __init p02_scaler_update_init(void)
{
	int rc;
    struct device *class_dev;
    
    printk("[p02_scaler_update] p02_scaler_update_init\n");
    
    rc = alloc_chrdev_region(&scaler_device_no, 0, 1, SCALER_DEV);
	if (rc < 0) {
		pr_err("alloc_chrdev_region failed %d\n", rc);
		return rc;
	}

	driver_class = class_create(THIS_MODULE, SCALER_DEV);
	if (IS_ERR(driver_class)) {
		rc = -ENOMEM;
		pr_err("class_create failed %d\n", rc);
		goto unregister_chrdev_region;
	}

	class_dev = device_create(driver_class, NULL, scaler_device_no, NULL, SCALER_DEV);
	if (!class_dev) {
		pr_err("class_device_create failed %d\n", rc);
		rc = -ENOMEM;
		goto class_destroy;
	}

	cdev_init(&scaler_cdev, &scaler_fops);
	scaler_cdev.owner = THIS_MODULE;

	rc = cdev_add(&scaler_cdev, MKDEV(MAJOR(scaler_device_no), 0), 1);
	if (rc < 0) {
		pr_err("cdev_add failed %d\n", rc);
		goto err;
	}
    
    return i2c_add_driver(&p02_scaler_update_i2c_driver);

err:
	device_destroy(driver_class, scaler_device_no);
class_destroy:
	class_destroy(driver_class);
unregister_chrdev_region:
	unregister_chrdev_region(scaler_device_no, 1);

    return rc;
}

static void __exit p02_scaler_update_exit(void)
{
    printk("[p02_scaler_update] p02_scaler_update_exit\n");
    i2c_del_driver(&p02_scaler_update_i2c_driver);
    
}

module_init(p02_scaler_update_init);
module_exit(p02_scaler_update_exit);

MODULE_DESCRIPTION("p02 scaler update driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lenter_Chang");


