/*
 * eeprom_nuvoton.c - driver for EEPROM of Nuvoton
 *
 * Copyright (C) 2011 Sina Chou <sina_chou@asus.com>
 *
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/microp.h>
#include <linux/microp_notify.h>
#include <linux/microp_pin_def.h>
#include <linux/uaccess.h>
#include <linux/microp_api.h>
#include <linux/switch.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/delay.h>
//#include <mach/msm_iomap.h>
//#include <mach/gpio.h>
//#include <mach/gpiomux.h>
//#include <linux/a60k_gpio_pinname.h>
#ifdef CONFIG_CHARGER_MODE
#include <linux/reboot.h>
#endif

#include <linux/fs.h>

#define TEGRA_MICROP_NAME "microp"

#define GPIO_CONFIG(n)    (MSM_TLMM_BASE + 0x1000 + (0x10 * n))
#define GPIO_IN_OUT(n) (MSM_TLMM_BASE + 0x1004 + (0x10 * n))

static unsigned long jiffies_hp_plugIn=0 ; 


struct microP_info {
       int carkit_inited;
       struct i2c_client *i2c_client;
       struct microP_platform_data *pdata;
       struct delayed_work work;
       struct delayed_work initP01;
       struct delayed_work poweroffP01;
       struct delayed_work notifyDockInit;
       struct delayed_work initCarkit;
};


struct nuvoton_microp_command{
    char *name;;
    u8 addr;
    u8 len;
    enum readwrite{
		E_READ=0,
		E_WRITE=1,
		E_READWRITE=2,
		E_NOUSE=3,

    }rw;

};


/*
*       addr: length
*       
*/
struct nuvoton_microp_command uP_CMD_Table[]={
        {"hw_id",                   0x00,  4,   E_READ},         //MICROP_HW_ID
        {"fw_ver",                 0x01,  2,   E_READ},         //MICROP_FW_VER
        {"input_lvl",               0x20,  4,   E_READ},         //MICROP_GPIO_INPUT_LEVEL
        {"out_lvl",                  0x21,  4,   E_READWRITE},         //MICROP_GPIO_OUTPUT_LEVEL
        {"charging_status",     0x30,  1,   E_READ},         //MICROP_CHARGING_STATUS
        {"gauge_id",               0x31,  4,   E_READ},         //MICROP_GAUGE_ID
        {"gauge_cap",            0x32,  1,   E_READ},         //MICROP_GAUGE_CAP
        {"gauge_tmp",           0x33,  1,   E_READ},         //MICROP_GAUGE_TMP
        {"usb_det",                0x34,  1,   E_READ},         //MICROP_USB_DET
        {"pwm",                     0x40,  1,   E_NOUSE},         //MICROP_PWM
        {"intr_status",             0x41,  4,   E_READ},         //MICROP_INTR_STATUS
        {"intr_en",                 0x42,  4,   E_READWRITE},         //MICROP_INTR_EN
        {"boot_sel",                0x50,  1,   E_READ},         //MICROP_BOOT_SELECTION
        {"boot_ld",                 0x51,  1,   E_WRITE},         //MICROP_SET_BOOT_LDROM
        {"boot_ap",                0x52,  1,   E_WRITE},         //MICROP_SET_BOOT_APROM
        {"prog_ap",                0x53,  32,  E_WRITE},         //MICROP_PROGRAM_APROM
        {"ap_chksum",           0x54,  4,  E_READ},          //MICROP_APROM_CHECKSUM
        {"software_off",           0x55,  4,  E_WRITE},          //MICROP_SOFTWARE_OFF
        {"ldrom_id",               0x03,  2,  E_READ},          //MICROP_LDROM_ID_CODE
        {"out_lvr_bit_set",      0x22,  4,  E_WRITE},          //MICROP_GPIO_OUTPUT_BIT_SET
        {"out_lvr_bit_clr",      0x23,  4,  E_WRITE},          //MICROP_GPIO_OUTPUT_BIT_CLR        
        {"intr_en_bit_set",      0x43,  4,  E_WRITE},          //MICROP_INTR_EN_BIT_SET
        {"intr_en_bit_clr",       0x44,  4,  E_WRITE},          //MICROP_INTR_EN_BIT_CLR        
};

 
//const unsigned int microP_hw_ID=0x00005200;
unsigned int g_microp_ver=0;
unsigned int g_ldrom_ver=0;


static struct microP_info *g_uP_info=NULL;
uint8_t *img_buf=NULL;
static u32 g_slave_addr=0;
static unsigned int g_fw_update_progress=0;
unsigned int g_b_isP01Connected=0;
unsigned int g_b_isP01USBConnected=0;
struct switch_dev p01_switch_dev;
struct switch_dev p01_switch_usb_cable;
struct switch_dev pad_switch_proximity;
struct switch_dev pad_err_notify;
struct mutex microp_mutex_lock;
struct mutex vote_mutex_lock;
extern void msleep(unsigned int msecs);
#ifdef CONFIG_CHARGER_MODE
extern char g_CHG_mode;
extern int g_chgmode_extra;
#endif

static uint8_t g_uPadStationUnderPoweroff=0;
struct workqueue_struct *microp_slow_job_wq = NULL;
struct workqueue_struct *microp_insert_wq = NULL;

extern unsigned g_GPIO_HDMI_5V_ENABLE;//Mickey+++
extern unsigned g_GPIO_P01_I2C_LS_OE;
unsigned int g_GPIO_DDC_I2C_LS_OE=7;
int g_up_suspend_flag=0;

enum _microp_state{
        st_DISCONNECTED=0,
        st_CONNECTED=1,
        st_INITIALIZING=2,
};

/*
    votesScalar[0]: Display vote for hdmi sleep, default 'no'
    votesScalar[1]: Audio vote for hdmi sleep, default 'yes'
*/
uint8_t votesScalar[2]={1,1};
uint8_t g_uPadErrStatus=0;
extern bool hdmi_exist(void);
extern bool hdmi_exist_realtime(void);

void reportPadStationI2CFail(char *devname);
int isFirmwareUpdating(void);

static int uP_i2c_read(u8 addr, int len, void *data)
{
        int i=0;
        int retries=6;
        int status=0;

	struct i2c_msg msg[] = {
		{
			.addr = g_slave_addr,
			.flags = 0,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = g_slave_addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};
    
        if(g_uP_info){
            do{    
        		status = i2c_transfer(g_uP_info->i2c_client->adapter,
        			msg, ARRAY_SIZE(msg));
        		// if (retries > 1) 20110804
        		if ((status < 0) && (i < retries)){
						if (g_up_suspend_flag) 
							msleep(10);
						else
							mdelay(10);
/*                        
                                if(!hdmi_exist() 
                                    || !gpio_get_value(g_GPIO_HDMI_5V_ENABLE) 
                                    || !gpio_get_value(g_GPIO_P01_I2C_LS_OE)){ //giveup retry
                                                    printk("up: i2c read retry GIVEUP !<run %d><%d, %d, %d>\r\n", i, hdmi_exist(), 
                                                                                        gpio_get_value(g_GPIO_HDMI_5V_ENABLE), 
                                                                                        gpio_get_value(g_GPIO_P01_I2C_LS_OE));
                                                    break;
                                }
*/                                
                                printk("%s retry %d I2C status=0x%x\r\n", __FUNCTION__, i, status);                                
						if  ((status & 0xf00) == 0xE00)
						{
							printk("%s I2C error happened: I2C status = 0x%x , HDMI_5V_ENABLE=%d , I2C_LS_OE=%d , DDC_I2C_LS_OE=%d\r\n", __FUNCTION__, status, gpio_get_value(g_GPIO_HDMI_5V_ENABLE), gpio_get_value(g_GPIO_P01_I2C_LS_OE), gpio_get_value(7) );
							msleep(15);
						}
                                i++;
                     }
        	    } while ((status < 0) && (i < retries));
        }
        if(status < 0){
            printk("MicroP: i2c read error %d , g_b_isP01Connected=%d, hdmi_exist_realtime()=%d\n", status, g_b_isP01Connected, hdmi_exist_realtime());
            if(st_CONNECTED==g_b_isP01Connected && hdmi_exist_realtime())
                    reportPadStationI2CFail("MicroP");
        }
    

        return status;
        
}



static int uP_i2c_write(u8 addr, int len, void *data)
{
        int i=0;
	int status=0;
	u8 buf[len + 1];
	struct i2c_msg msg[] = {
		{
			.addr = g_slave_addr,
			.flags = 0,
			.len = len + 1,
			.buf = buf,
		},
	};
	int retries = 6;

	buf[0] = addr;
	memcpy(buf + 1, data, len);

	do {
		status = i2c_transfer(g_uP_info->i2c_client->adapter,
			msg, ARRAY_SIZE(msg));
        	if ((status < 0) && (i < retries)){
						if (g_up_suspend_flag) 
							msleep(10);
						else
							mdelay(10);
/*                    
                    if(!hdmi_exist() 
                        || !gpio_get_value(g_GPIO_HDMI_5V_ENABLE) 
                        || !gpio_get_value(g_GPIO_P01_I2C_LS_OE)){ //giveup retry
                                        printk("up: i2c write retry GIVEUP !<run %d><%d, %d, %d>\r\n", i, hdmi_exist(), 
                                                                            gpio_get_value(g_GPIO_HDMI_5V_ENABLE), 
                                                                            gpio_get_value(g_GPIO_P01_I2C_LS_OE));
                                        break;
                    }
*/                    
                    printk("%s retry %d, I2C status=0x%x\r\n", __FUNCTION__, i, status);
					if  ((status & 0xf00) == 0xE00)
					{
						printk("%s I2C error happened: I2C status = 0x%x , HDMI_5V_ENABLE=%d , I2C_LS_OE=%d , DDC_I2C_LS_OE=%d\r\n", __FUNCTION__, status, gpio_get_value(g_GPIO_HDMI_5V_ENABLE), gpio_get_value(g_GPIO_P01_I2C_LS_OE), gpio_get_value(7) );
						msleep(15);
					}
                    i++;
			}
       } while ((status < 0) && (i < retries));

	if (status < 0) {
                    printk("MicroP: i2c write error %d , g_b_isP01Connected=%d, hdmi_exist_realtime()=%d\n", status, g_b_isP01Connected, hdmi_exist_realtime());
                    if(st_CONNECTED==g_b_isP01Connected && hdmi_exist_realtime())
                            reportPadStationI2CFail("MicroP");
	}

	return status;
}

/*
* return read data length of the reg
*/

int uP_nuvoton_read_reg(int cmd, void *data){
    int status=0;
    if(cmd>=0 && cmd < ARRAY_SIZE(uP_CMD_Table)){
        if(E_WRITE==uP_CMD_Table[cmd].rw || E_NOUSE==uP_CMD_Table[cmd].rw){ // skip read for these command
               
        }
        else   
            status=uP_i2c_read(uP_CMD_Table[cmd].addr, uP_CMD_Table[cmd].len, data);
    }
    else
        printk("MicroP: unknown cmd\r\n");
            
    return status;
}

EXPORT_SYMBOL_GPL(uP_nuvoton_read_reg);

/*
* return read data length of the reg
*/

int uP_nuvoton_write_reg(int cmd, void *data){
    int status=0;
    if(cmd>=0 && cmd < ARRAY_SIZE(uP_CMD_Table)){
        if(E_READ==uP_CMD_Table[cmd].rw  || E_NOUSE==uP_CMD_Table[cmd].rw){ // skip read for these command               
        }
        else
            status=uP_i2c_write(uP_CMD_Table[cmd].addr, uP_CMD_Table[cmd].len, data);
    }
    else
        printk("MicroP: unknown cmd\r\n");
            
    return status;
}

EXPORT_SYMBOL_GPL(uP_nuvoton_write_reg);

/*
* return 1 if microp is connected
*/


int isMicroPConnected(void){    
       int status;
       int ret=0;
       int reg_id=0;
       printk("%s \r\n", __FUNCTION__);
       status=uP_nuvoton_read_reg(MICROP_FW_VER,&reg_id);
       if(status > 0 && reg_id > 0){
            printk("MicroP found! fw ver=0x%x\r\n",reg_id);
            g_microp_ver=reg_id;
            uP_nuvoton_read_reg(MICROP_LDROM_ID_CODE,&g_ldrom_ver);
            printk("LDROM ver=0x%x\r\n",g_ldrom_ver);            
            ret=1;             
       }else{
            printk("%s : not connected\r\n", __FUNCTION__);
       }
       return ret;
}

EXPORT_SYMBOL_GPL(isMicroPConnected);



/*
*
*       for debug
*
*
*/
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int dbg_microp_regs_dump(struct seq_file *s, void *unused)
{
        int i=0;
        struct nuvoton_microp_command cmd;
        unsigned int val;
        seq_printf(s, "\nMicroP CMD       ADDR        VAL\r\n");
        seq_printf(s, "============================== %d,\r\n",ARRAY_SIZE(uP_CMD_Table));
        for(i=0;i<ARRAY_SIZE(uP_CMD_Table);i++){                
            cmd=uP_CMD_Table[i];
            val=0;
            if(st_CONNECTED == g_b_isP01Connected){
				uP_nuvoton_read_reg(i,&val);
			}
//            uP_i2c_read(cmd.addr,cmd.len,&val);
            seq_printf(s, "%16s%4x\t%8x(h)\r\n",cmd.name, cmd.addr, val);
        }
	return 0;
}

static int dbg_microp_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_microp_regs_dump, &inode->i_private);
}
static const struct file_operations debug_fops = {
	.open		= dbg_microp_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


#endif


static int updateMicroPFirmware(unsigned long arg);

/************************************
*
* macro definition
*
*************************************/
// check reg_intr_sta val

// Dock-Related
#define IsDockHavingEvt()                   ( ((reg_intr_sta>>INTR_STA_DOCK_INT) & 0x1)?1:0 )
#define IsDockInOut()                          ( ((reg_intr_sta>>INTR_STA_DOCK_IN_OUT) & 0x1)?1:0 )
#define IsDockBattPBChg()                   ( ((reg_intr_sta>>INTR_STA_DOCK_PB) & 0x1)?1:0 )
#define IsDockExtPowerInOut()           ( ((reg_intr_sta>>INTR_STA_DOCK_ACOK) & 0x1)?1:0 )
#define IsDockLidEvt()					( ((reg_intr_sta>>INTR_STA_DOCK_LID) & 0x1)?1:0 )
#define IsHeadSetHookEvt()			( ((reg_intr_sta>>INTR_STA_HEADSET_KEY) & 0x1)?1:0 )

// P01-Related
#define IsP01ACUSBInOut()                  ( ((reg_intr_sta>>INTR_STA_AC_USB_IN_OUT) & 0x1)?1:0 )
#define IsP01VolDnEvt()               ( ((reg_intr_sta>>INTR_STA_VOL_DOWN) & 0x1)?1:0 )
#define IsP01VolUpEvt()               ( ((reg_intr_sta>>INTR_STA_VOL_UP) & 0x1)?1:0 )
#define IsP01PwrBtnEvt()             ( ((reg_intr_sta>>INTR_STA_POWER_KEY) & 0x1)?1:0 )
#define IsP01HeadPhoneInOut()          ( ((reg_intr_sta>>INTR_STA_HP_IN) & 0x1)?1:0 )
#define IsP01HavingLSEvt()                 ( ((reg_intr_sta>>INTR_STA_ALS_INT) & 0x1)?1:0 )
#define IsP01HavingBatStaChg()                 ( ((reg_intr_sta>>INTR_STA_BAT_GOOD) & 0x1)?1:0 )
#define IsP01HavingBatChgErr()                 ( ((reg_intr_sta>>INTR_STA_BAT_ERR) & 0x1)?1:0 )
#define IsP01HavingProxAct()                 ( ((reg_intr_sta>>INTR_STA_PROX_ACT) & 0x1)?1:0 )
#define IsP01PwrBtnLongPressEvt()             ( ((reg_intr_sta>>INTR_STA_LONG_PRESS_PWRKEY) & 0x1)?1:0 )
#define IsP01NoIntrEvt()                            ((reg_intr_sta==0)?1:0)
/*
* check reg_input val
*     IN_DOCK_ACOK => high: inserted, low: removed
*	IN_DOCK_INT => low active
*	IN_ALS_INT => low active
*	IN_HP_IN => low active
*	IN_VOL_DOWN => low: pressed
*	IN_VOL_UP => low: pressed
*	IN_SUS_SW_R => low: pwrbtn pressed
*	IN_AC_USB_IN_R => high: cable in, low: cable out
*	IN_PWR_ON_OFF_R =>
*	IN_PB_DIS_uP => high: battery power bad
*	IN_DOCK_IN_R => low active. low: dock inerted
*	IN_O_LID_R => low active
*	IN_LOUT_DET_R => low active
*	IN_BATT_GOOD => high: good, low: bad
*   IN_CAP_RPOX_INT => high: close to human, low: far away from human
*   IN_O_LID_R => low active, low: keyboard lid has been closed
*   IN_LOUT_DET_R => low active, audio accessory has been pluged-in
*/

// Dock
#define IsDockPlugged()                ( ((reg_input>>IN_DOCK_IN_R) & 0x1)?0:1 )
#define IsDockExtPowerIn()                ( ((reg_input>>IN_DOCK_ACOK) & 0x1)?1:0 )
#define IsDockBattBad()                      ( ((reg_input>>IN_PB_DIS_uP) & 0x1)?1:0 )
#define IsDockLidClose()					( ((reg_input>>IN_O_LID_R) & 0x1)?0:1 )


// P01
#define IsP01ACUSBIn()                      ( ((reg_input>>IN_AC_USB_IN_R) & 0x1)?1:0 )
#define IsP01HeadPhoneIn()                ( ((reg_input>>IN_HP_IN) & 0x1)?0:1 )
#define IsP01VolUpPressed()                ( ((reg_input>>IN_VOL_UP) & 0x1)?0:1 )
#define IsP01VolDnPressed()                ( ((reg_input>>IN_VOL_DOWN) & 0x1)?0:1 )
#define IsP01PwrBtnPressed()                ( ((reg_input>>IN_SUS_SW_R) & 0x1)?0:1 )
#define IsHookKeyPressed()			( ((reg_input>>IN_LOUT_DET_R) & 0x1)?0:1 )
#define IsP01BATGood()                  ( ((reg_input>>IN_BATT_GOOD) & 0x1)?1:0 )
#define IsP01ProxMoveNear()                  ( ((reg_input>>IN_CAP_RPOX_INT) & 0x1)?0:1 )
#define IsP01_15V_In()                  ( ((reg_input>>IN_15V_IN_R) & 0x1)?1:0 )


extern int micropSendNotify(unsigned long val);
extern int is_p01_hdmi_inserted(void);
extern int asusec_dock_cable_status(void);
extern void bl_pad_add_func(struct work_struct *work);






int isAlwaysPowerOnMicroP(void){    
       int status;
       unsigned reg_input=0;

       status=uP_nuvoton_read_reg(MICROP_GPIO_INPUT_LEVEL,&reg_input);
       printk("%s input reg=%d\r\n", __FUNCTION__, reg_input);
       if(status){
                printk("%s <15v, dockin, acok, dockpb>=<%d,%d,%d,%d>\r\n", __FUNCTION__, IsP01_15V_In(),
                                                                                                                                        IsDockPlugged(), 
                                                                                                                                        IsDockExtPowerIn(),
                                                                                                                                        IsDockBattBad());
                if(IsP01_15V_In() 
                        ||(IsDockPlugged() && !IsDockBattBad())){
                        return 1;
                }
       }
       return 0;
}






/*
*
* Export to Display driver to inform HDMI status change
*
*/
void notify_microp_hdmi_insert(void)
{
	printk("MicroP: HDMI -> INSERT \r\n");
       if(g_uP_info){        
                g_b_isP01Connected=st_INITIALIZING; // 2: hdmi cable inserted but not intialized ready
                switch_set_state(&p01_switch_dev,st_INITIALIZING);
                //schedule_delayed_work(&g_uP_info->initP01,0);
                queue_delayed_work(microp_insert_wq, &g_uP_info->initP01, 0);
        }
       else{
            printk("MicroP: not yet init \r\n");
       }
       
}
EXPORT_SYMBOL_GPL(notify_microp_hdmi_insert);

void notify_microp_hdmi_remove(int virtual)
{
       if(virtual){
	        printk("MicroP: HDMI -> VIRTUAL REMOVE \r\n");
       }
       else{
               printk("MicroP: HDMI -> REMOVE \r\n");
       }
       
       if(g_uP_info){
                if(st_CONNECTED==g_b_isP01Connected){
                        g_b_isP01Connected=st_DISCONNECTED;
                        switch_set_state(&p01_switch_dev,st_DISCONNECTED);


                        printk("MicroP: cancel work...\r\n");
                        cancel_delayed_work_sync(&g_uP_info->initP01);
                        printk("MicroP: Done\r\n");
                        // cancel all works of init wq
                        
                        // set votes for defaultreg_input
                        votesScalar[0]=0;
                        votesScalar[1]=1;
                        micropSendNotify(P01_REMOVE);

                        if(virtual){
                            switch_set_state(&pad_err_notify, g_uPadErrStatus);
                        }

       
                }
                else
                        printk("MicroP: P01 already removed, skip \r\n");
                    
                
        }
       else{
            printk("MicroP: not yet init \r\n");
       }

       
}
EXPORT_SYMBOL_GPL(notify_microp_hdmi_remove);



void TriggerPadStationPowerOff(void){
       if(g_uP_info && microp_slow_job_wq){
                if(!g_uPadStationUnderPoweroff){
                        printk("power off P02 : 6 secs later");
                        queue_delayed_work(microp_slow_job_wq, &g_uP_info->poweroffP01,0);
                }else{
                        printk("power off work is onGoing now\r\n");
                }
        }
       else{
                printk("MicroP: not yet init \r\n");
       }

}

void reportPadStationI2CFail(char *devname){

        printk("%s: dev[%s] <5v_en, ls_oe,hdmi_exist_realtime, p01>=<%d,%d,%d,%d>\r\n",  __FUNCTION__, 
                                                                                            devname, 
                                                                                            gpio_get_value(g_GPIO_HDMI_5V_ENABLE), 
                                                                                            gpio_get_value(g_GPIO_P01_I2C_LS_OE),
                                                                                            hdmi_exist_realtime(),
                                                                                            g_b_isP01Connected                                                                                                                                                                                  
                                                                                            );
                                                                                                 
        if(st_CONNECTED==g_b_isP01Connected && 
            hdmi_exist_realtime() && !(g_microp_ver & 0x8000)){
                    g_uPadErrStatus=3; //i2c error
                    micropSendNotify(P01_DEAD);  
        }
                
}

void reportPadStationDockI2CFail(void){
        printk("[%s] ++\r\n", __FUNCTION__);
        micropSendNotify(DOCK_EXT_POWER_PLUG_OUT);        
        micropSendNotify(DOCK_PLUG_OUT);        
        printk("[%s] --\r\n", __FUNCTION__);
}


EXPORT_SYMBOL_GPL(TriggerPadStationPowerOff);
EXPORT_SYMBOL_GPL(reportPadStationI2CFail);
EXPORT_SYMBOL_GPL(reportPadStationDockI2CFail);
// for test +++




#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>
#define I2C_TEST_FAIL_MICROP_READ_I2C (-1)
#define I2C_TEST_FAIL_MICROP_WROND_CHIP_ID (-2)

static int TestMicroPChipID(struct i2c_client *apClient)
{

	int lnResult = I2C_TEST_PASS;
       unsigned int l_micropID=0;

	i2c_log_in_test_case("TestMicroPChipID++\n");
	lnResult = uP_i2c_read(0x00, 4,&l_micropID);
	if(lnResult <= 0){

		i2c_log_in_test_case("Fail to get microp id\n");

		lnResult = I2C_TEST_FAIL_MICROP_READ_I2C;
	}
/*		
	}else if(l_micropID != microP_hw_ID){

		i2c_log_in_test_case("Get wrong chip id=%d",l_micropID);

		lnResult = I2C_TEST_FAIL_MICROP_WROND_CHIP_ID;
	}
*/	
        else
	{
	    i2c_log_in_test_case("Get wrong chip id=%d",l_micropID);	  
            lnResult=I2C_TEST_PASS;
	}
	i2c_log_in_test_case("TestMicroPChipID--\n");

	return lnResult;
};

static struct i2c_test_case_info gMicroPTestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(TestMicroPChipID),
};


#endif
// for test ---


static void initP01(struct work_struct *work){
        int reg_intr_sta=0;
        int reg_input=0;        
        uint8_t i2cdata[32]={0};
        printk("%s +\r\n",__FUNCTION__);
        

        
        if(isMicroPConnected()==1){
                if(st_CONNECTED!=g_b_isP01Connected){
                        g_b_isP01Connected=st_CONNECTED;
                        // default: not enable  HP Hookkey Intr bit 12
                        i2cdata[0]=0xfd;
                        i2cdata[1]=0xef;
                        i2cdata[2]=0x00;
                        uP_nuvoton_write_reg(MICROP_INTR_EN,i2cdata);

#ifdef CONFIG_CHARGER_MODE
                        if(g_CHG_mode && !(g_chgmode_extra&0x1)) {
                            //if(!gpio_get_value(35)) { // bat_low:0  not_bat_low:1
                                printk(KERN_INFO "Battery low in P02. Reboot to Android now ...\r\n");
                                kernel_restart("oem-03"); //For new requirement, no charging mode in P02
                            //}
                        }
#endif

                        micropSendNotify(P01_ADD);
                        switch_set_state(&p01_switch_dev,st_CONNECTED);
                        g_uPadErrStatus=0;      // reset error status to 0
                        switch_set_state(&pad_err_notify, 0);

                }
                else
                        printk("MicroP: P01 already added, skip \r\n");
        }
        else{
                printk("uP_Init: failed to access microp\r\n");
        }


        
        if(st_CONNECTED==g_b_isP01Connected){
                    uP_nuvoton_read_reg(MICROP_INTR_STATUS,&reg_intr_sta);
                    uP_nuvoton_read_reg(MICROP_GPIO_INPUT_LEVEL,&reg_input);
                    printk("skip first intr status = %x, input = %x\r\nCheck Devices Status on P01\r\n",reg_intr_sta,reg_input);
#if 0
                // check Dock if available
                    if(IsDockPlugged()){
                                    printk(KERN_INFO "uP_Init: EC_Dock_in !!\n");
                                    micropSendNotify(DOCK_PLUG_IN);
                                    if(IsDockExtPowerIn()){
                                                    printk("uP_Init: EC Ext Power In!!\r\n");
                                                    micropSendNotify(DOCK_EXT_POWER_PLUG_IN);
                                                    if(AX_MicroP_get_USBDetectStatus(Batt_Dock)==P01_CABLE_USB){
                                                            printk("uP_Init: EC USB In!!\r\n");
                                                            g_b_isP01USBConnected=1;
                                                    }
                                                    else{
                                                           printk("uP_Init: EC Charger/AC  In!!\r\n");
                                                            g_b_isP01USBConnected=0;
                                                    }
                                                    switch_set_state(&p01_switch_usb_cable,g_b_isP01USBConnected);
                                     }
                                    
                                     if(IsDockBattBad()){
                                                    printk("uP_Init: EC Batt PowerBad!!\r\n");
                                                    micropSendNotify(DOCK_BATTERY_POWER_BAD);
                                     }                                 
                    }                
#endif
                    if(IsDockPlugged()){
                                       printk(KERN_INFO "uP_Init: EC_Dock_in, init 2 secs later!!\n");
                                       queue_delayed_work(microp_slow_job_wq, &g_uP_info->notifyDockInit, 2*HZ);
                    }
                    if(IsP01ACUSBIn()){
                                        printk(KERN_INFO "uP_Init: AC/USB in !!\n");
                                        micropSendNotify(P01_AC_USB_IN);
                    }

                    if(IsP01HeadPhoneIn()){
                                        printk(KERN_INFO "uP_Init: Headphone in !!\n");
                                        micropSendNotify(P01_HEADPHONE_IN);
                                        jiffies_hp_plugIn=jiffies;
                                        printk(KERN_INFO "uP_Init: Enable HookKey Intr !!\n");
                                        AX_MicroP_enablePinInterrupt(INTR_EN_HEADSET_KEY, 1);
                                        
                    }

#ifdef CONFIG_CHARGER_MODE
                if(!g_CHG_mode)
#endif
                {
                        // enable Pad Virator for 100 ms to let use know P01 inited successfully
                        AX_MicroP_setGPIOOutputPin(OUT_uP_VIB_EN, 1);
                        msleep(100);
                        AX_MicroP_setGPIOOutputPin(OUT_uP_VIB_EN, 0);
                }
        }
        printk("%s -\r\n",__FUNCTION__);
}


static void microp_poweroff(struct work_struct *work){
        unsigned short off=0xAA;
        g_uPadStationUnderPoweroff=1;
        printk("[%s] ++\r\n", __FUNCTION__);
        uP_nuvoton_write_reg(MICROP_SOFTWARE_OFF,  &off);
        msleep(5500); //wait P01 truly power off its power. microp fw need 5 seconds to power off
        //g_uPadErrStatus=4; // pad battery low, forcely remove        
        micropSendNotify(P01_BATTERY_POWER_BAD);           
        printk("[%s] --\r\n", __FUNCTION__);
        g_uPadStationUnderPoweroff=0;

}
static void notifyDockInit(struct work_struct *work){
#ifdef CONFIG_ASUSEC

        int reg_input=0;
        if(isMicroPConnected()==1){
                    uP_nuvoton_read_reg(MICROP_GPIO_INPUT_LEVEL,&reg_input);
                    if(IsDockPlugged()){
                                    printk(KERN_INFO "%s: EC_Dock_in !!\n", __FUNCTION__);
                                    micropSendNotify(DOCK_PLUG_IN);
                                    if(IsDockExtPowerIn()){
                                                    printk("%s: EC Ext Power In!!\r\n", __FUNCTION__);
                                                    micropSendNotify(DOCK_EXT_POWER_PLUG_IN);
                                                    if(AX_MicroP_get_USBDetectStatus(Batt_Dock)==P01_CABLE_USB){
                                                            printk("%s: EC USB In!!\r\n", __FUNCTION__);
                                                            g_b_isP01USBConnected=1;
                                                    }
                                                    else{
                                                           printk("%s: EC Charger/AC  In!!\r\n", __FUNCTION__);
                                                            g_b_isP01USBConnected=0;
                                                    }
                                                    switch_set_state(&p01_switch_usb_cable,g_b_isP01USBConnected);
                                     }
                                    
                                     if(IsDockBattBad()){
                                                    printk("%s: EC Batt PowerBad!!\r\n", __FUNCTION__);
                                                    micropSendNotify(DOCK_BATTERY_POWER_BAD);
                                     }                                 
                    }                
        }
#endif
}
static void initCarkit(struct work_struct *work){
       printk("%s ++\r\n", __FUNCTION__);
       if(g_uP_info && !hdmi_exist_realtime() && 0==gpio_get_value(g_uP_info->pdata->intr_gpio)){
                printk("[gpio %d] is low, checking if carkit\r\n", g_uP_info->pdata->intr_gpio);

                // make sure i2c is ready
                printk("<15V, LS_EN, DDC_EN>=<%d,%d,%d>\r\n", gpio_get_value(g_GPIO_HDMI_5V_ENABLE),
                                                                                                gpio_get_value(g_GPIO_P01_I2C_LS_OE),
                                                                                                gpio_get_value(g_GPIO_DDC_I2C_LS_OE));

                gpio_set_value(g_GPIO_P01_I2C_LS_OE, 1);
                gpio_set_value(g_GPIO_DDC_I2C_LS_OE, 0);
                msleep(15);

                if(0==isMicroPConnected() && !g_b_isP01Connected){ //carkit
                        printk("carkit detect!\r\n");    
                        g_b_isP01USBConnected=2;
                       switch_set_state(&p01_switch_usb_cable, 2);                        
                }


                gpio_set_value(g_GPIO_P01_I2C_LS_OE, 0);
                gpio_set_value(g_GPIO_DDC_I2C_LS_OE, 0);

       }

       g_uP_info->carkit_inited=1;
       printk("%s --\r\n", __FUNCTION__);
}


static void microP_work(struct work_struct *work)
{
       int reg_intr_sta=0;
       int reg_input=0;   
       int pad_cap=0;
       pr_debug("MicroP wq <%d,%d>+++\r\n", g_b_isP01Connected, gpio_get_value(g_uP_info->pdata->intr_gpio));
       if(st_CONNECTED==g_b_isP01Connected){
                        while(0==gpio_get_value(g_uP_info->pdata->intr_gpio)){
                                       uP_nuvoton_read_reg(MICROP_INTR_STATUS,&reg_intr_sta);
                                       uP_nuvoton_read_reg(MICROP_GPIO_INPUT_LEVEL,&reg_input);
                                       pr_debug("MicroP intr: status=%8x, input=%8x\r\n",reg_intr_sta,reg_input);
#ifdef CONFIG_ASUSEC
                                        if(IsDockInOut()){
                                            if(IsDockPlugged()){
                                                       printk("MicroP EC_Dock_in !!\n");
                                                       micropSendNotify(DOCK_PLUG_IN);
                                            }
                                             else {
                                                       printk("MicroP EC_Dock_out !!\n");
                                                       micropSendNotify(DOCK_PLUG_OUT);
                                            }
                                        }

                                         if(IsDockHavingEvt()){
                                                if(IsDockPlugged()){
                                                        pr_debug("MicroP EC Evt Occurs !!\n");
                                                        micropSendNotify(DOCK_KEY_TOUCH_EVENT);                
                                                }
                                        }

                                        if(IsDockBattPBChg() && IsDockBattBad()){
                                                       printk("MicroP EC Battery Low!!\r\n");
                                                       micropSendNotify(DOCK_BATTERY_POWER_BAD);
                                        }

                                        if(IsDockExtPowerInOut()){
                                                       printk("MicroP EC Ext Power In/Out!!\r\n");
                                                       if(IsDockExtPowerIn()){
                                                                   printk("MicroP EC Ext Power In!!\r\n");
                                                                   micropSendNotify(DOCK_EXT_POWER_PLUG_IN);
                                                                   if(AX_MicroP_get_USBDetectStatus(Batt_Dock)==P01_CABLE_USB){
                                                                                            printk("uP_Init: EC USB In!!\r\n");
                                                                                            g_b_isP01USBConnected=1;
                                                                    }
                                                                    else{
                                                                                            printk("uP_Init: EC Charger/AC  In!!\r\n");
                                                                                            g_b_isP01USBConnected=0;
#ifdef CONFIG_CHARGER_MODE
                                                                                            if (g_CHG_mode)
                                                                                                     kernel_restart(NULL);
#endif
                                                                    }
                                                                    
                                                        }
                                                       else{
                                                           printk("MicroP EC Ext Power Out!!\r\n");
                                                           g_b_isP01USBConnected=0;
                                                           micropSendNotify(DOCK_EXT_POWER_PLUG_OUT);
                                                       }

                                                       switch_set_state(&p01_switch_usb_cable,g_b_isP01USBConnected);
                                        }

                                        if(IsDockLidEvt())
                                       {
                                                   printk("MicroP EC keyboard lid status has been changed\n");
                                                   micropSendNotify(DOCK_LID_CHANGE_EVENT);
                                        }
#endif       

                                        if(IsP01HavingBatStaChg()){
                                                           printk("MicroP HavingBatStaChg!!\r\n");
                                                           if(IsP01BATGood()){
                                                                    printk("MicroP Batt Good !!\r\n");
                                                           }
                                                           else{
                                                                    uP_nuvoton_read_reg(MICROP_GAUGE_CAP, &pad_cap);
                                                                    uP_nuvoton_read_reg(MICROP_GAUGE_CAP, &pad_cap);                                                    
                                                                    printk("MicroP Batt Bad  , pad_cap=%d !!\r\n", pad_cap);
                                                                    if(pad_cap <=3 && !isAlwaysPowerOnMicroP())
                                                                            TriggerPadStationPowerOff();
                                                           }
                                        }

                                        if(IsP01HavingBatChgErr()){
                                                           printk("MicroP HavingBatChgErr!!\r\n");
                                        }

                                        if(IsP01HavingProxAct()){
                                                           printk("MicroP HavingProxAct!!\r\n");
                                                           if(IsP01ProxMoveNear()){
                                                                    pr_debug("MicroP Prox: Moving Near !!\r\n");
                                                                    switch_set_state(&pad_switch_proximity, 1);
                                                            }
                                                           else{
                                                                    pr_debug("MicroP Prox: Moving Away !!\r\n");
                                                                    switch_set_state(&pad_switch_proximity, 0);
                                                           }
                                        }

                                        if(IsP01ACUSBInOut()){
                                                       if(IsP01ACUSBIn()){
                                                           printk("MicroP P01 AC/USB Cable In!!\r\n");
                                                           micropSendNotify(P01_AC_USB_IN);
                                                           if(AX_MicroP_get_USBDetectStatus(Batt_P01)==P01_CABLE_USB){
                                                                    printk("P01 USB IN\r\n");
                                                                    g_b_isP01USBConnected=1;
                                                            }
                                                           else{
                                                                    printk("P01 AC IN\r\n");
                                                                    g_b_isP01USBConnected=0;
#ifdef CONFIG_CHARGER_MODE
                                                                    if (g_CHG_mode)
                                                                             kernel_restart(NULL);
#endif
                                                           }
                                                            
                                                       }
                                                       else{
                                                           printk("MicroP P01 AC/USB Cable Out!!\r\n");
                                                           micropSendNotify(P01_AC_USB_OUT);
                                                           g_b_isP01USBConnected=0;
                                                       }
                                                       switch_set_state(&p01_switch_usb_cable,g_b_isP01USBConnected);
                                                       
                                        }
                                        
                                        if(IsP01VolDnEvt()){
                                                       printk("MicroP VolDnKey !!\r\n");
                                                       if(IsP01VolDnPressed()){                                    
                                                              micropSendNotify(P01_VOLDN_KEY_PRESSED);
                                                        }
                                                       else
                                                           micropSendNotify(P01_VOLDN_KEY_RELEASED);                                            
                                        }
                                        
                                        if(IsP01VolUpEvt()){
                                                       printk("MicroP VolUpKey !!\r\n");
                                                       if(IsP01VolUpPressed()){
                                                                micropSendNotify(P01_VOLUP_KEY_PRESSED);
                                                            }
                                                       else
                                                            micropSendNotify(P01_VOLUP_KEY_RELEASED);
                                        }
                                        
                                        if(IsP01PwrBtnEvt()){
                                                       printk("MicroP PwrKey !!\r\n");
                                                       if(IsP01PwrBtnPressed())
                                                            micropSendNotify(P01_PWR_KEY_PRESSED);
                                                       else
                                                            micropSendNotify(P01_PWR_KEY_RELEASED);
                                        }

                                        if(IsP01HeadPhoneInOut()){
                                                       printk("MicroP Headphone In/Out !!\r\n");
                                                       if(IsP01HeadPhoneIn()){
                                                           printk("MicroP P01 Headphone In!!\r\n");
                                                           jiffies_hp_plugIn=jiffies;
                                                           micropSendNotify(P01_HEADPHONE_IN);
                                                           AX_MicroP_enablePinInterrupt(INTR_EN_HEADSET_KEY, 1);
                                                       }
                                                       else{
                                                           printk("MicroP P01 Headphone Out!!\r\n");
                                                           jiffies_hp_plugIn=0;
                                                           micropSendNotify(P01_HEADPHONE_OUT);
                                                           AX_MicroP_enablePinInterrupt(INTR_EN_HEADSET_KEY, 0);
                                                       }
                                        }
                                        if(IsHeadSetHookEvt())
                                        {
                                                 if(jiffies_hp_plugIn && time_after(jiffies, jiffies_hp_plugIn + 1* HZ))
                                                          if(IsP01HeadPhoneIn ()){
                                                                 printk("MicroP Hookey Evt !!\n");
                                                                    if(IsHookKeyPressed()) {
                                                                            micropSendNotify(P01_HEADSET_HOOKKEY_PRESSED);
                                                           }else{
                                                                            micropSendNotify(P01_HEADSET_HOOKKEY_RELEASED);
                                                           }
                                                           }else{
                                                                            printk("HP not in, skip evt\r\n");
                                                           }
                                                  else{
                                                             printk("microp: hookey debounce failed in 1 sec\r\n");
                                                  }
                                                          
                                        }
                                        if(IsP01HavingLSEvt()){
                                                           printk("MicroP Light Sensor Having Event !!\r\n");
                                                           micropSendNotify(P01_LIGHT_SENSOR);
                                        }
                                        if(IsP01NoIntrEvt()){
                                                            pr_debug("MicroP No Event !! wait..\r\n");
                                                            msleep(5); //wait gpio 107 from low -> high again
                                        }
                                        
                            }
        }
        else{
                if(st_DISCONNECTED==g_b_isP01Connected){
                            msleep(200);
                            if(1==g_uP_info->carkit_inited && !hdmi_exist_realtime() && 0==gpio_get_value(g_uP_info->pdata->intr_gpio)){
                                           printk("MicroP:  carkit in ~\r\n");
                                           g_b_isP01USBConnected=2;
                                           switch_set_state(&p01_switch_usb_cable, 2);

                            }
                            if(1==g_uP_info->carkit_inited  && 1==gpio_get_value(g_uP_info->pdata->intr_gpio)){
                                           printk("MicroP:  carkit out ~\r\n");
                                           g_b_isP01USBConnected=0;
                                           switch_set_state(&p01_switch_usb_cable, 0);
                            }
                }
                
        }
        pr_debug("MicroP wq <%d, %d>---\r\n", g_b_isP01Connected,gpio_get_value(g_uP_info->pdata->intr_gpio));
}



static irqreturn_t microP_irq_handler(int irq, void *dev_id)
{
        struct microP_info *info =
            (struct microP_info *)dev_id;
        schedule_delayed_work(&info->work,0);
        
        return IRQ_HANDLED;

}


const unsigned int gc_firmwareSize=8*1024;

int getCmdID(int addr){
    int i=0;
    for(i=0;i < ARRAY_SIZE(uP_CMD_Table);i++){
        if(addr==uP_CMD_Table[i].addr)
            return i;
    }
    return -1;
}


int isFirmwareUpdating(void){        
    if((g_fw_update_progress>0 && g_fw_update_progress<100 ) || g_microp_ver & 0x8000){
            printk("isFirmwareUpdating: 1\r\n");
            return 1;
     }
    return 0;
}

/*
 *       Firmware update procedure via I2C
 *      - Change to LDROM using CMD 0x51
 *      - 
 *
 *
*/

static int updateMicroPFirmware(unsigned long arg){
        uint8_t i2cdata[32]={0};

        int i=0;
        int j=0;
        int val=0;
        int ret=0;
        int num_updated_bytes=0;

        uint32_t backup_output=0;
        uint32_t backup_intr_en=0;
        uint32_t reg_input=0;

        if(img_buf==NULL)
                img_buf = kzalloc(sizeof(uint8_t)*8192, GFP_KERNEL);
        
        if(!img_buf){
                pr_err("%s: Unable to allocate memory!\n",__FUNCTION__);
                return -ENOMEM;
        }

        memset(img_buf,0,sizeof(uint8_t)*8192);

       /* first element of arg must be u32 with id of module to talk to */
        if(copy_from_user(img_buf, (const void __user *)arg, sizeof(uint8_t)*gc_firmwareSize)) {
                pr_err("%s: Failed to copy arg from user", __FUNCTION__);
                return -EFAULT;
        }

        uP_nuvoton_read_reg(MICROP_GPIO_INPUT_LEVEL,&reg_input);
                if(IsDockPlugged()) {
                    micropSendNotify(DOCK_PLUG_OUT);
        }

        printk("MicroP disable irq first\r\n");
        disable_irq(g_uP_info->i2c_client->irq);
   
        uP_nuvoton_read_reg(MICROP_GPIO_OUTPUT_LEVEL,&backup_output);
        uP_nuvoton_read_reg(MICROP_INTR_EN,&backup_intr_en);

        printk("Microp backup out=%x, intren=%x\r\n", backup_output, backup_intr_en);            

        if(uP_nuvoton_read_reg(MICROP_BOOT_SELECTION,&val))
            printk("Microp %s\r\n",val?"@LDROM":"@APROM");



       i2cdata[0] = 0xA5;
       uP_nuvoton_write_reg(MICROP_SET_BOOT_APROM, i2cdata);

    	i2cdata[0] = 0x5A;
    	uP_nuvoton_write_reg(MICROP_SET_BOOT_LDROM, i2cdata);

       if(uP_nuvoton_read_reg(MICROP_BOOT_SELECTION,&val)){
            printk("Boot selection => %s\r\n",val?"@LDROM":"@APROM");
            if(val!=1){
                    printk("Cannot Change To LDROM: Fatal Error\r\n");
                    return -EAGAIN;
            }            
        }
        for (i = 0;i < (gc_firmwareSize);i += 32){
            for (j = 0;j < 32;j ++){
                i2cdata[j] = *(img_buf+i+j);
            }
            ret=uP_nuvoton_write_reg(MICROP_PROGRAM_APROM, i2cdata);
            if(ret < 0){
                printk("MicroP update failed: I2C write failed\r\n");
                goto update_fw_failed;
            }
            msleep(50);
            num_updated_bytes+=32;
            g_fw_update_progress=((num_updated_bytes*100) - 1)/gc_firmwareSize;
            printk(".");
            if(num_updated_bytes % 1024 ==0){
                    printk("==> [ %d %%]\r\n", g_fw_update_progress);
            }
        }

        printk("MicroP done\r\n");
        msleep(10);
        printk("MicroP Update APROM finish..rebooting\r\n");

        i2cdata[0] = 0xA5;
        uP_nuvoton_write_reg(MICROP_SET_BOOT_APROM, i2cdata);
        msleep(50);
        
        if(uP_nuvoton_read_reg(MICROP_BOOT_SELECTION,&val))
            printk("MicroP After Update: Microp %s\r\n",val?"@LDROM":"@APROM");

       if(val==1){
            printk("MicroP update failed: cannot enter into APROM mode\r\n");
            ret=-EAGAIN;
            goto update_fw_failed;
       }




       printk("switch backlight on\r\n");
       bl_pad_add_func(NULL);

       uP_nuvoton_read_reg(MICROP_INTR_STATUS,&val);
       uP_nuvoton_read_reg(MICROP_INTR_STATUS,&val);
       printk("MicroP flush discard intr status=%8x\r\n",val);

       uP_nuvoton_write_reg(MICROP_GPIO_OUTPUT_LEVEL, &backup_output);
       uP_nuvoton_write_reg(MICROP_INTR_EN, &backup_intr_en);
       
       uP_nuvoton_read_reg(MICROP_GPIO_OUTPUT_LEVEL,&backup_output);
       uP_nuvoton_read_reg(MICROP_INTR_EN,&backup_intr_en);
       printk("after restore out, intr_en = <%x, %x>\r\n", backup_output, backup_intr_en);


       g_fw_update_progress=100;     


       
update_fw_failed:
       enable_irq(g_uP_info->i2c_client->irq);
       printk("MicroP F/W done. enable irq again\r\n");
       if(img_buf)
            kfree(img_buf);
       img_buf=NULL;

       uP_nuvoton_read_reg(MICROP_GPIO_INPUT_LEVEL,&reg_input);
        if(IsDockPlugged()) {
                    micropSendNotify(DOCK_PLUG_IN);
        }

       return ret;
    
}
/*
* file operation implementation
*/

static int asus_microp_open(struct inode *inode, struct file *flip){
        pr_debug("%s\r\n",__FUNCTION__);
        return 0;
}
static int asus_microp_release(struct inode *inode, struct file *flip){
        pr_debug("%s\r\n",__FUNCTION__);
        return 0;

}
static long asus_microp_ioctl(struct file *flip, unsigned int cmd, unsigned long arg){

       int ret=0;
       int val=0;
       pr_info("%s\r\n",__FUNCTION__);
           
       if(_IOC_TYPE(cmd) != ASUS_MICROP_IOC_TYPE)
            return -ENOTTY;
       if(_IOC_NR(cmd) > ASUS_MICROP_MAX_NR)
            return -ENOTTY;

       if(g_uP_info==NULL)
            return -EFAULT;
       
	switch (cmd) {
	    case ASUS_MICROP_FW_UPDATE:
                    pr_info("ioctl: ASUSEC_FW_UPDATE ++\r\n");
                    ret=updateMicroPFirmware(arg);
                    pr_info("ioctl: ASUSEC_FW_UPDATE --\r\n");
                    break;
           case ASUS_MICROP_CHECK_CONNECTED:                    
                    ret=__put_user((g_b_isP01Connected==st_CONNECTED),(int __user *)arg);
                    break;
            case ASUS_MICROP_GET_FW_VERSION:
                    if(isMicroPConnected())
                            ret=__put_user(g_microp_ver,(int __user *)arg);
                    else
                            ret=-EINVAL;
                    break;
            case ASUS_MICROP_ON_OFF_GPS_ON_PAD:
                    pr_info("ioctl: ASUS_MICROP_ON_OFF_GPS_ON_PAD ++\\r\n");
                    if(isMicroPConnected()){
                            ret=__get_user(val,(int __user *)arg);                        
                    }
                    else
                            ret=-EINVAL;
                    if(ret >=0){
                            pr_info("ioctl: ASUS_MICROP_ON_OFF_GPS_ON_PAD val=%dr\n", val);
                            AX_MicroP_setGPIOOutputPin(OUT_uP_GPS_LNA_EN,val);                            
                    }
                        
                    pr_info("ioctl: ASUS_MICROP_ON_OFF_GPS_ON_PAD --\r\n");                    
                    break;
            default:
                pr_err("Invalid ioctl code\r\n");
                ret= -EINVAL;
                break;
        }
    

        return ret;
}


static struct file_operations asus_microp_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = asus_microp_ioctl,
	.open = asus_microp_open,
	.release = asus_microp_release,
};

static struct miscdevice asus_microp_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = TEGRA_MICROP_NAME,
	.fops = &asus_microp_fops,
	.mode= 0666,
};


/*
* show/restory interface
*/
static int g_cmd_idx=0;
static ssize_t reg_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
       int val=0;
       int count =0;
       int status=0;

	if(st_CONNECTED!=g_b_isP01Connected){
            	count+=sprintf(buf,"microp: failed, not connected\r\n");
		return count;
	}
       
       status=uP_nuvoton_read_reg(g_cmd_idx,&val);
       if(status > 0){                    
            count+=sprintf(buf, "\"%16s\": addr[0x%2x] = %8x(h)\r\n",uP_CMD_Table[g_cmd_idx].name, 
                                                                        uP_CMD_Table[g_cmd_idx].addr, 
                                                                        val);
       }
       else 
            count+=sprintf(buf,"reg read failed %d\r\n",status);
        
	return count;
}
static ssize_t reg_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
    	int addr=0;
       int val=0;
	if(st_CONNECTED!=g_b_isP01Connected){
		pr_info("microp: failed, not connected\r\n");
		return 0;
	}
       if(buf[0]=='-'){
            addr = simple_strtol(buf+1, NULL, 16);       
            g_cmd_idx=getCmdID(addr);
            pr_info("set addr=0x%2x, ID=%d\r\n",addr,g_cmd_idx);
            if(g_cmd_idx==-1)
                    g_cmd_idx=0;
       }
       else{    //write value to idx
                if(g_cmd_idx<0 && g_cmd_idx > ARRAY_SIZE(uP_CMD_Table))
                        pr_info("set addr first\r\n");
                else{
                        val=simple_strtol(buf, NULL, 16);
                        pr_info("write val %d to addr [0x%x]\r\n", val, uP_CMD_Table[g_cmd_idx].addr);
                        uP_nuvoton_write_reg(g_cmd_idx, &val);
                }
       }

	return count;
}


/*
* show/restory interface
*/
static ssize_t pad_update_progress_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
	return sprintf(buf, "%d\r\n",g_fw_update_progress);
}



/*
* show/restory interface
*/
static ssize_t dock_hallsensor_status_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
	int count=0;
       if(st_CONNECTED!=g_b_isP01Connected){
                count+= sprintf(buf,"0\r\n");   // return 0 if p01 not connected
       }
       else{
                if(AX_MicroP_getGPIOPinLevel(IN_O_LID_R) == 0)
                        count+= sprintf(buf,"1\r\n");        // return 1 if p01 connected & pin is low
                else
                        count+= sprintf(buf,"0\r\n");        // return 0 if p01 connected & pin is high or i2c err
       }
       return count;
}


/*
* show/restory interface
*/
static ssize_t pad_proximity_sensor_status_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
	int count=0;
       if(st_CONNECTED!=g_b_isP01Connected){
                count+= sprintf(buf,"0\r\n");   // return 0 if p01 not connected
       }
       else{
                if(AX_MicroP_getGPIOPinLevel(IN_CAP_RPOX_INT) == 0)
                        count+= sprintf(buf,"1\r\n");        // return 1 if p01 connected & pin is low
                else
                        count+= sprintf(buf,"0\r\n");        // return 0 if p01 connected & pin is high or i2c err
       }
       return count;
}


static DEVICE_ATTR(reg, 0664, reg_show, reg_store);
static DEVICE_ATTR(pad_update_progress, 0644, pad_update_progress_show, NULL);
static DEVICE_ATTR(dock_hallsensor_status, 0644, dock_hallsensor_status_show, NULL);
static DEVICE_ATTR(pad_proximity_sensor_status, 0644, pad_proximity_sensor_status_show, NULL);






//gpio switch class related-functions

static ssize_t p01_switch_name(struct switch_dev *sdev, char *buf)
{
       char model_name[30]={0};
       if(st_CONNECTED==g_b_isP01Connected)
                sprintf(model_name,"p%02d",g_microp_ver);
       else
                sprintf(model_name,"unknown");
       
	return sprintf(buf, "%s\n", model_name);
}


static ssize_t p01_switch_state(struct switch_dev *sdev, char *buf)
{        
	return sprintf(buf, "%s\n", (g_b_isP01Connected)?"1":"0");
}



static ssize_t p01_usb_switch_name(struct switch_dev *sdev, char *buf)
{
       
	return sprintf(buf, "Asus-P01\n");
}


static ssize_t p01_usb_switch_state(struct switch_dev *sdev, char *buf)
{
//	return sprintf(buf, "%s\n", (g_b_isP01USBConnected ? "1" : "0"));
       return sprintf(buf, "%d\n", g_b_isP01USBConnected);
}


static ssize_t pad_proximity_switch_state(struct switch_dev *sdev, char *buf)
{
	int count=0;
       if(st_CONNECTED!=g_b_isP01Connected){
                count+= sprintf(buf,"0\r\n");   // return 0 if p01 not connected
       }
       else{
                if(AX_MicroP_getGPIOPinLevel(IN_CAP_RPOX_INT) == 0)
                        count+= sprintf(buf,"1\r\n");        // return 1 if p01 connected & pin is low
                else
                        count+= sprintf(buf,"0\r\n");        // return 0 if p01 connected & pin is high or i2c err
       }
       return count;

}

static ssize_t pad_notify_switch_state(struct switch_dev *sdev, char *buf)
{
	int count=0;
       count+= sprintf(buf,"%d\n", g_uPadErrStatus);
       return count;
}



static int microp_suspend(struct i2c_client *client,
	pm_message_t mesg)
{
       printk("%s ++\r\n",__FUNCTION__);

       AX_MicroP_enablePinInterrupt(INTR_EN_ALS_INT | INTR_EN_VOL_DOWN | INTR_EN_VOL_UP|INTR_EN_CAP_RPOX_INT, 0);
       enable_irq_wake(g_uP_info->i2c_client->irq);
       g_up_suspend_flag = 1;
	return 0;
}

static int microp_resume(struct i2c_client *client)
{
       printk("%s ++\r\n",__FUNCTION__);
       disable_irq_wake(g_uP_info->i2c_client->irq);
       AX_MicroP_enablePinInterrupt(INTR_EN_ALS_INT | INTR_EN_VOL_DOWN | INTR_EN_VOL_UP|INTR_EN_CAP_RPOX_INT, 1);
       printk("%s --\r\n",__FUNCTION__);
       g_up_suspend_flag = 0;
	return 0;
}





static int microP_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
       struct microP_info *info;
       struct microP_platform_data *microp_pdata;
       int ret=0;

       
       pr_info("microP: %s +++\n", __FUNCTION__);
	g_uP_info=info = kzalloc(sizeof(struct microP_info), GFP_KERNEL);
	if (!info) {
		pr_err("microP: Unable to allocate memory!\n");
		return -ENOMEM;
	}

       info->carkit_inited=0;
	microp_pdata=info->pdata = client->dev.platform_data;
        g_slave_addr=client->addr;
	info->i2c_client = client;
	i2c_set_clientdata(client, info);
    

        if(microp_pdata==NULL)
    		return -ENOMEM;


        microp_slow_job_wq = create_singlethread_workqueue("microp_slow_job");
        microp_insert_wq = create_singlethread_workqueue("microp_insert_job");

        // initialize work struct
        INIT_DELAYED_WORK(&info->work, microP_work);
        INIT_DELAYED_WORK(&info->initP01, initP01);
        INIT_DELAYED_WORK(&info->poweroffP01, microp_poweroff);
        INIT_DELAYED_WORK(&info->notifyDockInit, notifyDockInit);
        INIT_DELAYED_WORK(&info->initCarkit, initCarkit);
        // init mutex
        mutex_init(&microp_mutex_lock);
        mutex_init(&vote_mutex_lock);        
        // setup gpio for hall-sensor
        ret = gpio_request(microp_pdata->intr_gpio, client->name);
        if (ret < 0){
            goto err_request_gpio;
        }

        ret = gpio_direction_input(microp_pdata->intr_gpio);

        if (ret < 0)
            goto err_set_gpio_input;




        ret = request_irq(client->irq, microP_irq_handler,
                    IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING , client->name, info);

        
        if (ret < 0)
            goto err_request_irq;

 
        

	ret = misc_register(&asus_microp_device);
	if (ret < 0) {
		pr_err("%s: Unable to register misc device!\n",
		       __FUNCTION__);
		goto misc_register_err;
	}


       ret = device_create_file(&client->dev, &dev_attr_reg);
	if (ret < 0) {
		pr_err("%s: Unable to create file! %d\n",
		       __FUNCTION__,ret);
	}

       ret = device_create_file(&client->dev, &dev_attr_pad_update_progress);
	if (ret < 0) {
		pr_err("%s: Unable to create file! %d\n",
		       __FUNCTION__,ret);
	}
    
       ret = device_create_file(&client->dev, &dev_attr_dock_hallsensor_status);
	if (ret < 0) {
		pr_err("%s: Unable to create file! %d\n",
		       __FUNCTION__,ret);
	}

       ret = device_create_file(&client->dev, &dev_attr_pad_proximity_sensor_status);
	if (ret < 0) {
		pr_err("%s: Unable to create file! %d\n",
		       __FUNCTION__,ret);
	}
	

        // registered as gpio switch device
        p01_switch_dev.name="p01";
        p01_switch_dev.print_state=p01_switch_state;
        p01_switch_dev.print_name=p01_switch_name;
	ret=switch_dev_register(&p01_switch_dev);
	
	if (ret < 0){
		pr_err("%s: Unable to register switch dev! %d\n",
		       __FUNCTION__,ret);
	}

        p01_switch_usb_cable.name="usbcable";
        p01_switch_usb_cable.print_state=p01_usb_switch_state;
        p01_switch_usb_cable.print_name=p01_usb_switch_name;
	ret=switch_dev_register(&p01_switch_usb_cable);
	if (ret < 0){
		pr_err("%s: Unable to register switch dev! %d\n",
		       __FUNCTION__,ret);
	}
        pad_switch_proximity.name="pad_proximity";
        pad_switch_proximity.print_state=pad_proximity_switch_state;
        
	ret=switch_dev_register(&pad_switch_proximity);        
	if (ret < 0){
		pr_err("%s: Unable to register switch dev! %d\n",
		       __FUNCTION__,ret);
	}


        pad_err_notify.name="pad_notify";
        pad_err_notify.print_state=pad_notify_switch_state;
        
	ret=switch_dev_register(&pad_err_notify);        
	if (ret < 0){
		pr_err("%s: Unable to register switch dev! %d\n",
		       __FUNCTION__,ret);
	}
        
#ifdef CONFIG_DEBUG_FS

	(void) debugfs_create_file("microp", S_IRUGO,
					NULL, NULL, &debug_fops);
#endif    

#ifdef CONFIG_I2C_STRESS_TEST
       i2c_add_test_case(info->i2c_client, "MicroP",ARRAY_AND_SIZE(gMicroPTestCaseInfo));
#endif


       queue_delayed_work(microp_slow_job_wq, &g_uP_info->initCarkit,12*HZ);
       pr_info("microP: addr= %x, pin=%d\r\n", g_slave_addr,  microp_pdata->intr_gpio);
       pr_info("microP: %s ---\n", __FUNCTION__);
	return 0;
    
    err_request_gpio:
    err_set_gpio_input:
    misc_register_err:
    err_request_irq:    
        return ret;
}

static int microP_remove(struct i2c_client *client)
{
	struct microP_info *info = i2c_get_clientdata(client);
       g_uP_info=NULL;
       switch_dev_unregister(&p01_switch_dev);
       misc_deregister(&asus_microp_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id microP_id[] = {
	{ TEGRA_MICROP_NAME, 0 },
	{ }
};







static struct i2c_driver microp_nuvoton_driver = {
	.driver = {
		.name   = TEGRA_MICROP_NAME,
		.owner  = THIS_MODULE,
	},
	.probe      = microP_probe,
	.remove     = microP_remove,
	.suspend= microp_suspend,
	.resume= microp_resume,
	
	.id_table   = microP_id,
};





static int __init microP_init(void)
{
	return i2c_add_driver(&microp_nuvoton_driver);
}

static void __exit microP_exit(void)
{
	i2c_del_driver(&microp_nuvoton_driver);
}


MODULE_AUTHOR("Sina Chou <sina_chou@asus.com>");
MODULE_DESCRIPTION("EEPROM_Nuvoton driver");
MODULE_LICENSE("GPL");

fs_initcall_sync(microP_init);
module_exit(microP_exit);
