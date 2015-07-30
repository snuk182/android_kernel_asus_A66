/* cm3623.c - CM3623 proximity/light sensor driver
 *
 * Copyright (C) 2011 ASUSTek Inc.
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

#include <linux/miscdevice.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/gpio.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/cdev.h>
#include <linux/cpu.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/percpu.h>
#include <linux/notifier.h>
#include <linux/syscalls.h>
#include <linux/kallsyms.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/unistd.h> 
#include <asm/uaccess.h>
#include <asm/unistd.h>
#include <linux/linkage.h>
#include <linux/stringify.h>
#include <linux/errno.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/mman.h>
#include <linux/shm.h>
#include <linux/wait.h>
#include <linux/slab.h>

#include <linux/kthread.h>
#include <linux/poll.h>

#include <linux/wait.h>
#include <linux/wakelock.h>

#include "linux/cm3623.h"
#include "linux/proximity_class.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h> 
#endif

#include "linux/al3010.h"
#include <linux/microp_notify.h>

#ifdef DBGMSK_PRX_G2
#undef DBGMSK_PRX_G2
#define DBGMSK_PRX_G2 KERN_INFO
#endif

#ifdef DBGMSK_PRX_G0
#undef DBGMSK_PRX_G0
#define DBGMSK_PRX_G0 KERN_INFO
#endif

#define CM3623_DRV_NAME     "cm3623"


#define ALS_WORD_MODE   1

#define ALS_CALIBRATION 1

struct i2c_client *cm3623_client = NULL;
struct cm3623_data {
    struct i2c_client       client ;
    struct input_dev        *input_dev;     /* Pointer to input device */
    struct delayed_work i2c_poll_work;
    unsigned int            poll_interval_ms;   /* I2C polling period */
    unsigned int            event_threshold;    /* Change reqd to gen event */
    unsigned int            open_count;     /* Reference count */
    //unsigned short        x;              /* Current X position */
    //unsigned short        y;              /* Current Y position */
    //unsigned short        z;              /* Current Z position */
    char                polling;            /* Polling flag */
};

struct _cm3623_device {
    int (*init_hw)(struct i2c_client *client);
    u8    (*read_int_pin_state)(void);
    int irq;
} g_cm3623_device;

struct cm3623_data  *g_cm3623_data_as;
struct cm3623_data  *g_cm3623_data_ps;

struct input_dev *this_input_dev_as = NULL;
struct input_dev *this_input_dev_ps = NULL;

static struct workqueue_struct *cm3623light_workqueue;
//[CR] Queue a work to write ATD self-detect file
struct delayed_work g_light_work;

static struct work_struct cm3623_attached_P02_work;
static struct work_struct cm3623_ISR_work;
static struct work_struct cm3623_light_interrupt_work;
static struct work_struct cm3623_proximity_interrupt_work;

static int g_switch[2] = {0,0}; //switch on/off for proximity and light.

static u8 g_cm3623_int_state =1;
static u8 g_last_cm3623_int_state =1;

static int g_proxm_dbg = 0; /* Add for debug only */
static int g_ambient_dbg = 0; /* Add for debug only */
static int g_interval = 100;
static int g_proxm_switch_on = 0;
static int g_cm3623_switch_earlysuspend = 0;
static int g_ambient_suspended = 0;

#ifdef ALS_WORD_MODE
static u16 g_cm3623_light=0;
static u16 g_last_cm3623_light=0;
#else
static u8 g_cm3623_light=0;
static u8 g_last_cm3623_light=0;
#endif
static u16 g_cm3623_light_adc = 0;
static u16 g_cm3623_light_k_adc = 0;
static u16 g_cm3623_light_calibration_fval_x1000 = 10240;    // it is actually 10 after shift right 10 bits

static int g_nProxm_Faraway = 1; //1 means no object detected!
//static int g_last_nProxm_Faraway = 1; //1 means no object detected!

//INFOMATION:
//The level of autobrightness is defined in frameworks/base/core/res/res/values/config.xml
//Setting config_autoBrightnessLevels and config_autoBrightnessLcdBacklightValues.
//static int g_light_level[11] = {0, 10, 40, 90, 160, 225, 320, 640, 1280, 8000, 12000};
//A60K EVB
static int g_light_level[14] = {0, 50, 100, 200, 300, 400, 500, 600, 700, 900, 1100, 1400, 1700, 2100};

atomic_t ambient_update, proxm_update, ps_status, als_status, INT_status;

static int g_ps_threshold = DEFAULT_PS_THRESHOLD;
static int g_ioctl_enable=1; //allow ioctl function.
static unsigned char g_data[2];

static int g_als_threshold = 0;
static u16 g_psData = 0;
static int g_proxim_state = 0;

#ifdef ALS_WORD_MODE
static u16 g_prxVer=0; //proximity data.
#else
static u8 g_prxpVer=0; //proximity data.
#endif
static u8 als_IT = 3; //0=100ms, 1=200ms, 2=400ms, 3=800ms.
static u8 als_GAIN = 3; //0~3.
static u8 ps_IT=3; //0=1T, 3=1.875T.
static u8 als_ps_SD = 0x1;
static int g_android_or_oms = 0; // 0 for android, 1 for oms.
static int g_light_map[3][14] = { // start from level9 to level16.
                    {1,4,20,40,60,80,150,300,9200,13700}, //for SR1
                    {5,25,50,100,200,400,850,1650,3400,13700}, //for SR2
                    {118,571,1143,2286,3429,4571,5714,6857,8000,10286,12571,16000,19429,24000}  //A60K EVB
                };

//wait_queue_head_t cm3623_wq_head; /* Wait queue for the misc device */
DECLARE_WAIT_QUEUE_HEAD(proxm_wq_head);
DECLARE_WAIT_QUEUE_HEAD(ambient_wq_head);
DECLARE_WAIT_QUEUE_HEAD(als_polling_wq_head);
//wait_queue_head_t proxm_isr_wq;

static int cm3623_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int cm3623_remove(struct i2c_client *client);
static int cm3623_suspend(struct i2c_client *client, pm_message_t mesg);
static int cm3623_resume(struct i2c_client *client);
static int cm3623_reset(void);
#ifdef ALS_WORD_MODE
static int cm3623_read_reg(struct i2c_client* client, u8 reg, u16 *data);
#else
static int cm3623_read_reg(struct i2c_client* client, u8 reg, u8 *data);
#endif
//static int cm3623_write_reg(struct i2c_client* client, u8 reg, u8 val);

static int proxmdev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int proxmdev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int proxmdev_open(struct inode *inode, struct file *file);
static ssize_t proxmdev_read(struct file *file, char __user * buffer, size_t size, loff_t * f_pos);
static unsigned int proxmdev_poll(struct file *filep, poll_table * wait);
static long proxmdev_ioctl(struct file *file, unsigned int cmd,unsigned long arg);
static int proxmdev_release(struct inode *inode, struct file *filp);

static int proxm_set_threshold(int value);
int proximity_als_turn_on(int bOn);
static int cm3623_turn_onoff_proxm(bool bOn);

static int ambientDev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int ambientDev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val);
static int ambientDev_open(struct inode *inode, struct file *file);
static ssize_t ambientDev_read(struct file *file, char __user * buffer, size_t size, loff_t * f_pos);
static unsigned int ambientDev_poll(struct file *filep, poll_table * wait);
static int ambientDev_release(struct inode *inode, struct file *filp);
static int cm3623_turn_onoff_als(bool);
int get_adc_calibrated_lux_from_cm3623(void);

bool read_lightsensor_calibrationvalue(void);
static void write_lightsensor_calibrationvalue_work(struct work_struct *work);

//#ifdef CONFIG_I2C_STRESS_TEST
static int g_i2ctest_proxm_switch = 0;
//#endif

static int atd_write_status_and_adc_2(int *als_sts, int *als_adc, int *ps_sts);
static void cm3623_lightsensor_attached_pad_P01(struct work_struct *work);
void als_lux_report_event(int);
int g_HAL_als_switch_on = 0;     //this var. means if HAL is turning on als or not

static struct wake_lock proximity_wake_lock;
extern struct device *proximity_dev;
extern int qup_i2c_resume(struct device *device);
static int write_lightsensor_thd_als(u8 thd_val);

static void light_interrupt_work(struct work_struct *work);
static void proximity_interrupt_work(struct work_struct *work);
static void light_proxim_error_reset(int handle, bool enable);

// for error handle ++
static unsigned long jtimes0, jtimes1;
enum sensors {
        LIGHT = 0,
        PROXIMITY,
} sensor_mode;
// for error handle --

/////////////////////////////////////////////////////////////////////////////////////////
//P01 reltaed
bool g_cm3623_als_switch_on = false;    //this var. means if cm3623 als hw is turn on or not
int g_polling_count = 0;

extern bool g_bIsP01Attached;
extern bool g_al3010_switch_on;


/////////////////////////////////////////////////////////////////////////////////////////
//

// BSP Louis ++
#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>

#define I2C_TEST_Lsensor_FAIL (-1)

//DECLARE_WAIT_QUEUE_HEAD(i2ctest_wq_head);

static int TestLightSensorI2C (struct i2c_client *apClient)
{
    int lnResult = I2C_TEST_PASS;
    int ret = 0, ps_adc = 0;
    struct i2c_msg msg[1];
    unsigned char data[2] = {0,0};
    u16 psData = 0;
        printk(KERN_INFO"------------------ TestLightSensorI2C\n");

    i2c_log_in_test_case("TestLSensorI2C ++\n");

        //enable proximity sensor
    msg->addr = (CM3623_PS_CTL_REG >> 1);
    msg->flags = 0;
    msg->len = 1;
    msg->buf = data;
    data[0] = (INIT_PS & 0xfe);
    data[0] |= (ps_IT<<4);

    if (g_proxm_switch_on == 0) {       //avoid phone mode keeping on/off
    g_i2ctest_proxm_switch = 1;

    ret = i2c_transfer(apClient->adapter, msg, 1);
    if(ret!=1)
        printk("proximity open fail\n");
    }

        //enable light sensor
    msg->addr = cm3623_client->addr;
    data[0] = (u8) g_switch[1];
    data[0] = (~data[0]) & INIT_ALS ;
    data[0] = ( (als_IT<<2) | data[0] );
#ifndef ALS_WORD_MODE
    data[0] |= (data[0] | (als_GAIN<<6));
#endif
    data[0] |= (0x2);  

    ret = i2c_transfer(apClient->adapter, msg, 1);
    if(ret!=1)
        printk("light sensor open fail\n");    

    set_current_state(TASK_INTERRUPTIBLE);
    schedule_timeout(msecs_to_jiffies(2000));

    cm3623_read_reg(apClient, CM3623_PS_DATA_REG, &psData);
    ps_adc = psData;
    printk("[Proximity] adc property = %d\n", ps_adc);

    ret = get_adc_calibrated_lux_from_cm3623();
    printk("[LightSensor] cal_adc = %d, lux = %d\n", g_cm3623_light_k_adc, g_cm3623_light);

    g_last_cm3623_light = g_cm3623_light;


    data[0] |= als_ps_SD;       //SD data command
    g_i2ctest_proxm_switch = 0;

    if(0 == g_cm3623_als_switch_on) {      //SD lightsensor
         msg->addr = cm3623_client->addr;
         i2c_transfer(cm3623_client->adapter, msg, 1); 
         printk("[LightSensor] power off\n");
    }   

    if( (0==g_i2ctest_proxm_switch)&&(0 == g_proxm_switch_on) ){    //SD Promximity Sensor
         waitqueue_active(&proxm_wq_head);
         msg->addr = (CM3623_PS_CTL_REG >> 1);
         i2c_transfer(apClient->adapter, msg, 1);    
         printk("[Proximity] power off\n");
    }        


    i2c_log_in_test_case("TestLSensorI2C --\n");


    return lnResult;
}
// BSP Louis --

static struct i2c_test_case_info gLSensorTestCaseInfo[] =
{
     __I2C_STRESS_TEST_CASE_ATTR(TestLightSensorI2C),
};


#endif

static enum proximity_property proxmdev_properties[] = {
    /* int */
    SENSORS_PROP_INTERVAL,
    SENSORS_PROP_THRESHOLD,
    SENSORS_PROP_MAXRANGE,      /* read only */
    SENSORS_PROP_RESOLUTION,    /* read only */
    SENSORS_PROP_VERSION,       /* read only */
    SENSORS_PROP_CURRENT,       /* read only */
    SENSORS_PROP_DBG, /* Add for debug only */
    /* char */
    SENSORS_PROP_SWITCH,
    SENSORS_PROP_VENDOR,        /* read only */
    SENSORS_PROP_ADC,           /* adc raw data */
    SENSORS_PROP_ATD_STATUS     /* for atd mode only */
};

static struct file_operations proxmdev_fops = {
    .owner = THIS_MODULE,
    .open = proxmdev_open,
    .read = proxmdev_read,
    .poll = proxmdev_poll,
    .unlocked_ioctl = proxmdev_ioctl,
    .release = proxmdev_release
};

struct proximity_class_dev g_proxmDev = {
    .id = SENSORS_PROXIMITY,
    .name = "psensor",
    .num_properties = ARRAY_SIZE(proxmdev_properties),
    .properties = proxmdev_properties,
    .get_property = proxmdev_get_property,
    .put_property = proxmdev_put_property,
    .fops = &proxmdev_fops
};


static int proxmdev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
    //printk(DBGMSK_PRO_G0 "[proxmdl]: proxmdl_get_property +.\n");

    switch( property ) 
    {
        case SENSORS_PROP_THRESHOLD:
            printk(DBGMSK_PRX_G4"[cm3623][ps] SENSORS_PROP_THRESHOLD. (%d)\n", g_ps_threshold);
            val->intval = g_ps_threshold;
            break;

        case SENSORS_PROP_INTERVAL:
            //printk("[proxmdl]: config IT_PS by using \"echo XXX > /sys/class/sensors/sensor3/interval\" XXX is only 0~3. \n");
            printk(DBGMSK_PRX_G4"[cm3623][ps] SENSORS_PROP_INTERVAL.\n");
            val->intval = g_interval;
            break;

        case SENSORS_PROP_MAXRANGE:
            printk(DBGMSK_PRX_G4"[cm3623][ps] SENSORS_PROP_MAXRANGE.\n");
            val->intval = 1; //keep it 1.0 for OMS.
            break;

        case SENSORS_PROP_RESOLUTION:
            printk(DBGMSK_PRX_G4"[cm3623][ps] SENSORS_PROP_RESOLUTION.\n");
            val->intval = 1;
            break;

        case SENSORS_PROP_VERSION:
            printk(DBGMSK_PRX_G4"[cm3623][ps] SENSORS_PROP_VERSION.\n");
            cm3623_read_reg(cm3623_client, CM3623_PS_DATA_REG, &g_prxVer);
            printk(DBGMSK_PRX_G4"[cm3623][ps] pData=%d\n", g_prxVer);
            val->intval = g_prxVer; //1;
            break;

        case SENSORS_PROP_CURRENT:
            printk(DBGMSK_PRX_G4"[cm3623][ps] SENSORS_PROP_CURRENT.\n");
            val->intval = 30;   
            break;

        case SENSORS_PROP_SWITCH:
            //printk(DBGMSK_PRO_G0 "[proxmdl]: SENSORS_PROP_SWITCH (%d).\n", g_kxtf9_switch_on);
            printk(DBGMSK_PRX_G4"[cm3623][ps] get switch = %d.\n", g_proxm_switch_on);
            val->intval = g_proxm_switch_on;
            break;

        case SENSORS_PROP_VENDOR:
            printk(DBGMSK_PRX_G4"[cm3623][ps] SENSORS_PROP_VENDOR.\n");
            sprintf(val->strval, "CAPELLA");
            break;

        /* Add for debug only */
        case SENSORS_PROP_DBG:
            val->intval = g_proxm_dbg;
            printk(DBGMSK_PRX_G4"[cm3623][ps] dbg = %d.\n", g_proxm_dbg);
            printk(DBGMSK_PRX_G4"[cm3623][ps] ps_Threshold(%d),ps_IT(%d)\n", g_ps_threshold, ps_IT);
            break;
        case SENSORS_PROP_ADC:
           // cm3623_read_reg(cm3623_client, CM3623_PS_DATA_REG, &psData);
            val->intval = g_psData;
            printk(DBGMSK_PRX_G4"[cm3623][ps] get adc property: %d\n", val->intval);
            break;

        case SENSORS_PROP_ATD_STATUS:
        {
            int als_sts =0, als_adc =0, ps_sts =0;

            atd_write_status_and_adc_2(&als_sts, &als_adc, &ps_sts);
            val->intval = ps_sts;
            printk(DBGMSK_PRX_G4"[cm3623][ps] get atd status: %d\n", val->intval);
            break;
        }

        default:
            printk(DBGMSK_PRX_G4"[cm3623][ps] default.\n");
            return -EINVAL;
    }

    //printk(DBGMSK_PRO_G0 "[proxmdl]: proxmdl_get_property -.\n");
    return 0;
}

// Louis ++
static int cm3623_turn_onoff_proxm(bool bOn)
{
    struct i2c_msg msg[1];
    unsigned char data[2]={0,0};
    int err = 0;

        if(1 == bOn) {
            printk(DBGMSK_PRX_G4"[cm3623][ps] sensor switch, turn on proximity sensor\n");
            //initialization
            msg->addr = CM3623_INIT_CMD >> 1;         //0x92
            msg->flags = 0; //0 - write.
            msg->len = 1;
            msg->buf = data;
            data[0] = 0x10;

            err = i2c_transfer(cm3623_client->adapter, msg, 1);
            if(err!=1) {
                printk(DBGMSK_PRX_G4"[cm3623][ps] addr=0x%x val=0x%x err=%d\n", (CM3623_INIT_CMD >> 1), data[0], err);
                goto proxim_reset;
            }
            else {
                printk(DBGMSK_PRX_G4"[cm3623][ps] set initialization, addr=0x%x val=0x%x \n",(CM3623_INIT_CMD >> 1), data[0]);        
            }
/*
                    // +/- 256 steps
                    msg->addr = CM3623_ALS_CTL_REG >> 1;       //ALS Control register: 0x90
                    data[0] = (INIT_ALS & 0xfe); 
                    data[0] |= (0x2); //WDM=1.

                    err = i2c_transfer(cm3623_client->adapter, msg, 1);

                    if(err!=1) {
                        printk(DBGMSK_PRX_G4"[cm3623][ps] addr=0x%x val=0x%x err=%d\n", (CM3623_ALS_CTL_REG >> 1), data[0], err);
                    }
                    else {
                        printk(DBGMSK_PRX_G4"[cm3623][ps] set steps, addr=0x%x val=0x%x \n",(CM3623_ALS_CTL_REG >> 1), data[0]);
                    }
*/

                    //set ps threshold 3~255
            msg->addr = CM3623_PS_THD_REG >> 1;       //PS threshold command: 0xF2
            data[0] = (INIT_PS | g_ps_threshold);

            if(err!=1) {
                printk(DBGMSK_PRX_G4"[cm3623][ps] addr=0x%x val=0x%x err=%d\n", (CM3623_PS_THD_REG >> 1), data[0], err);
                goto proxim_reset;
            }
            else {
                printk(DBGMSK_PRX_G4"[cm3623][ps] set threshold, addr=0x%x val=0x%x \n",(CM3623_PS_THD_REG >> 1), data[0]);
            }

            // enable ps interrupt
            msg->addr = CM3623_PS_CTL_REG >> 1;       //PS Control register: 0xF0

            if (g_cm3623_als_switch_on == 0)
                data[0] = (INIT_PS | (ps_IT<<4) | (0x1<<2));  //enable ps_INT
            else
                data[0] = (INIT_PS | (ps_IT<<4) | (0x3<<2)); //enable als_INT, ps_INT

            err = i2c_transfer(cm3623_client->adapter, msg, 1);

            if(err!=1) {
                printk(DBGMSK_PRX_G4"[cm3623][ps] addr=0x%x val=0x%x err=%d\n", (CM3623_PS_CTL_REG >> 1), data[0], err);
                goto proxim_reset;
            }
            else {
                printk(DBGMSK_PRX_G4"[cm3623][ps] enable interrpt,addr=0x%x val=0x%x \n",(CM3623_PS_CTL_REG >> 1), data[0]);
            }
        }

        else {
            printk(DBGMSK_PRX_G4"[cm3623][ps] sensor switch, turn off proximity sensor\n");
            msg->addr = (CM3623_PS_CTL_REG >> 1);
            msg->flags = 0;
            msg->len = 1;
            msg->buf = data;
            data[0] |= als_ps_SD;
            err = i2c_transfer(cm3623_client->adapter, msg, 1);
        }
        return err;

proxim_reset:

    sensor_mode = PROXIMITY;
    light_proxim_error_reset(sensor_mode, bOn);
    return err;

}
// Louis --

static int proxmdev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
    int ret = 0;
    int err=1;
    struct i2c_msg msg[1];
    unsigned char data[2] = {0,0};

    switch (property) 
    {
        case SENSORS_PROP_SWITCH:
            printk(DBGMSK_PRX_G4"[cm3623][ps] put SENSORS_PROP_SWITCH (%d,%d).\n", (val->intval), g_proxm_switch_on );
            if((g_proxm_switch_on != val->intval) && (!g_bIsP01Attached)) {
                if(val->intval==1) { //turn on PS

                    cm3623_turn_onoff_proxm(1);

                    //send an init value
                    input_report_abs(g_cm3623_data_ps->input_dev, ABS_DISTANCE, 1);
                    input_event(g_cm3623_data_ps->input_dev, EV_SYN, SYN_REPORT, 1);
                    input_sync(g_cm3623_data_ps->input_dev);

                    g_proxm_switch_on = 1;
                    printk(DBGMSK_PRX_G4"[cm3623][ps] proximity on.\n");
                }
                else {
                    err = waitqueue_active(&proxm_wq_head);
                    printk(DBGMSK_PRX_G4"[cm3623][ps] proxm_wq_head(%d)\n", err);

                    g_proxm_switch_on = 0;

                    if(g_HAL_als_switch_on == 0) {
                    // disable PS or directly Power off cm3623
                        msg->addr = CM3623_PS_CTL_REG >> 1;
                        msg->flags = 0; //0 - write.
                        msg->len = 1;
                        msg->buf = data;
                        data[0] = (INIT_PS | 0x1); //bit 0 is SD_PS bit.

                        err = i2c_transfer(cm3623_client->adapter, msg, 1);

                        if(err!=1) {
                                printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl addr=0x%x val=0x%x err=%d\n", (CM3623_PS_CTL_REG >> 1), data[0], err);
                                goto proxim_reset;
                        }
                        else {
                                printk(DBGMSK_PRX_G4"[cm3623][ps] power off directly, proxmdl addr=0x%x val=0x%x \n",(CM3623_PS_CTL_REG >> 1), data[0]);
                        }
                    }

                    else {
                        msg->addr = CM3623_PS_CTL_REG >> 1;
                        msg->flags = 0; //0 - write.
                        msg->len = 1;
                        msg->buf = data;
                        data[0] = (INIT_PS | (ps_IT<<4) | (0x1<<3)); // only disable INT_PS

                        err = i2c_transfer(cm3623_client->adapter, msg, 1);

                        if(err!=1) {
                                printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl addr=0x%x val=0x%x err=%d\n", (CM3623_PS_CTL_REG >> 1), data[0], err);
                                goto proxim_reset;
                        }
                        else {
                                printk(DBGMSK_PRX_G4"[cm3623][ps] turn off ps, als still on, proxmdl addr=0x%x val=0x%x \n",(CM3623_PS_CTL_REG >> 1), data[0]);
                        }
                    }
                    //reset the state or store the last state??
                    g_nProxm_Faraway = 1;
                    g_last_cm3623_int_state = 1;    //[SCR] reset state

                    printk(DBGMSK_PRX_G4"[cm3623][ps] off\n");
                }
            }
//          printk(DBGMSK_PRO_G0 "[proxmdl]: GPIO126, mfpr=0x%x , PLR3=0x%x , PDR3=0x%x \n", __raw_readl(0xFE01E06C), __raw_readl(0xFE019100+0x0000), __raw_readl(0xFE019100+0x000C));
//          printk(DBGMSK_PRO_G0 "[proxmdl]: GPIO127, mfpr=0x%x , PLR3=0x%x , PDR3=0x%x \n", __raw_readl(0xFE01E070), __raw_readl(0xFE019100+0x0000), __raw_readl(0xFE019100+0x000C));
            break;

        case SENSORS_PROP_THRESHOLD:
            printk(DBGMSK_PRX_G4"[cm3623][ps] config THRESHOLD (%d).\n", val->intval);
            if(val->intval>=3 && val->intval<=255) {
                //if(g_proxm_switch_on==1) {
                    if(g_ps_threshold != val->intval) {
                        g_ps_threshold = val->intval;
                        ret = proxm_set_threshold(g_ps_threshold);
                    }
                //}
                //else {
                //  printk(DBGMSK_PRO_G0 "[proxmdl]: ERROR!!! Sensor is not power on yet!!! \n");
                //}
            }
            else {
                printk(DBGMSK_PRX_G4"[cm3623][ps] ERROR!!! OUT OF THRESHOLD (3~255)!!! \n");
            }
            break;

        case SENSORS_PROP_INTERVAL:
            if(1) {
                printk(DBGMSK_PRX_G4"[cm3623][ps] set interval (0x%x)\n", val->intval);
                g_interval = val->intval;
            }
            else {
                printk(DBGMSK_PRX_G4"[cm3623][ps] config IT_PS (0x%x)\n", val->intval);
                if( (val->intval>=0) && (val->intval<=3) ) {
                    #if 0
                    gpio_set_value(g_proxm_pwr_pin, 0);
                    msleep(1);
                    gpio_set_value(g_proxm_pwr_pin, 1);
                    gpio_free(g_proxm_pwr_pin);
                    #endif
                    ps_IT = val->intval;
                    ret = cm3623_reset();
                }
            }
            break;

        /* Add for debug only */
        case SENSORS_PROP_DBG:
            g_proxm_dbg = val->intval;
            printk(DBGMSK_PRX_G4"[cm3623][ps] dbg = %d.\n", g_proxm_dbg);
            break;
        default:
            printk(DBGMSK_PRX_G4"[cm3623][ps] put default.\n");
            return -EINVAL;
    }

    return 0;

proxim_reset:

    sensor_mode = PROXIMITY;
    light_proxim_error_reset(sensor_mode, val->intval);
    return 0;

}


static int proxmdev_open(struct inode *inode, struct file *file)
{
    int ret = 0;

    printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdev_dev_open.\n");

    if (file->f_flags & O_NONBLOCK) {
        printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_open (O_NONBLOCK)\n");
    }
    atomic_set(&proxm_update, 0); //initialize atomic.
    atomic_set(&ps_status, 0); //initialize atomic.
    atomic_set(&als_status,0);
    atomic_set(&INT_status,0);
    ret = nonseekable_open(inode, file);

    return ret;
}


static ssize_t proxmdev_read(struct file *file, char __user * buffer, size_t size, loff_t * f_pos)
{
    struct input_event event;
    int ret = 0;
    DEFINE_WAIT(pwait);

    if(g_proxm_dbg==1)
    printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_read++\n");

    if (size < 1) {
        printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_read (size error!)\n");
        return -EINVAL;
    }

    if(g_android_or_oms==1) {

        while(1) {

            // I shall return EAGAIN when the request is O_NONBLOCK and the data is not ready yet.
            if (file->f_flags & O_NONBLOCK) {
                ret = -EAGAIN;
                printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_read (-EAGAIN)\n");
                return ret;
            }

            if(g_proxm_dbg==1)
                printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_read wait++\n");

            prepare_to_wait(&proxm_wq_head, &pwait, TASK_INTERRUPTIBLE);

            if(g_proxm_dbg==1)
                printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_read wait--\n");

            if(atomic_read(&proxm_update)) {
                // If there is update, don't sleep and just update.
                atomic_dec(&proxm_update);
                finish_wait(&proxm_wq_head, &pwait);
                if(g_proxm_dbg==1)
                printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_read break!\n");
                break;
            }
            else {
                // If there is no avaialbe, just sleep and wait for wakeup or signal.
                if(g_proxm_dbg==1)
                	printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_read schedule++\n");
                schedule();
                if(g_proxm_dbg==1)
                	printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_read schedule--\n");
            }

            if(g_proxm_dbg==1)
            	printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_read finish++\n");
            finish_wait(&proxm_wq_head, &pwait);
            if(g_proxm_dbg==1)
            	printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_read finish--\n");

            // If the signal wakes me up, I shall return to user-space.
            if (signal_pending(current)) {
                ret = -ERESTARTSYS;
                printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_read (-ERESTARTSYS)\n");
                return ret;
            }
        } //end of while
    }

    do_gettimeofday(&event.time);

    event.type = EV_ABS;
    event.code = ABS_DISTANCE;
    //jonathan: change the way to get intr. pin state
    //g_cm3623_proxm = gpio_get_value(g_proxm_int_pin);
    

    event.value = g_nProxm_Faraway ? 1: 0;

    /* make sure we are not going into copy_to_user() with
     * TASK_INTERRUPTIBLE state */
    set_current_state(TASK_RUNNING);
    if (copy_to_user(buffer, &event, sizeof(event))) {
        printk(DBGMSK_PRX_G4"[cm3623][ps] Copy proxm data Fail!\n");
        ret = -EFAULT;
    } 
    else  {
        ret = sizeof(event);
    }

    if(g_proxm_dbg) {
        cm3623_read_reg(cm3623_client, CM3623_PS_DATA_REG, &g_prxVer);
        printk(DBGMSK_PRX_G4"[cm3623][ps] proxm value 0x%x (%d)\n", event.value, g_prxVer);
    }

    //msleep(100); //100ms

    if(g_proxm_dbg==1)
    	printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_read--\n");
    return ret;
}

static unsigned int proxmdev_poll(struct file *filep, poll_table * wait)
{
    if(g_proxm_dbg==1)
    	printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_poll++\n");
    poll_wait(filep, &proxm_wq_head, wait);

    if(atomic_read(&proxm_update)) {
        if(g_proxm_dbg==1)
        	printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_poll-- (POLLIN %d)\n",atomic_read(&proxm_update));
        return (POLLIN | POLLRDNORM);
    }

    if(g_proxm_dbg==1)
    	printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_poll--\n");
    return 0;
}


static long proxmdev_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
    void __user *data;
    int value=0;
    int ret=1;
    struct i2c_msg msg[1];
    unsigned char idata[2] = {0,0};
    int err=1;
//    u8 tmp = g_cm3623_light;
//#ifdef ALS_WORD_MODE
//    u8 lobyte = 0;
//#endif

    switch (cmd) 
    {
        case PS_POLL_READ_GPIO_ON_OFF: //sensor on/off
            printk(DBGMSK_PRX_G4"[cm3623][ps] PS_POLL_READ_GPIO_ON_OFF %d\n", (int)arg);
            value = (int)arg;
            if(value==1) {
                printk(DBGMSK_PRX_G4"[cm3623][ps] promxdl_dev_ioctl, SD_PS = 0.\n");
                msg->addr = CM3623_PS_CTL_REG >> 1;
                msg->flags = 0; //0 - write.
                msg->len = 1;
                msg->buf = idata;
                idata[0] = (INIT_PS & 0xfe); //bit 0 is SD_PS bit.
                err = i2c_transfer(cm3623_client->adapter, msg, 1);
                if(err!=1) {
                    printk(DBGMSK_PRX_G4"[cm3623][ps] addr=0x%x val=0x%x err=%d\n", (CM3623_PS_CTL_REG >> 1), idata[0], err);
                }
                else {
                    printk(DBGMSK_PRX_G4"[cm3623][ps] addr=0x%x val=0x%x \n",(CM3623_PS_CTL_REG >> 1), idata[0]);
                }
                g_proxm_switch_on = 1;
            }
            else {
                printk(DBGMSK_PRX_G4"[cm3623][ps] promxdl_dev_ioctl, SD_PS = 1.\n");
                g_proxm_switch_on = 0;
                msg->addr = CM3623_PS_CTL_REG >> 1;
                msg->flags = 0; //0 - write.
                msg->len = 1;
                msg->buf = idata;
                idata[0] = (INIT_PS | 0x1); //bit 0 is SD_PS bit.
                err = i2c_transfer(cm3623_client->adapter, msg, 1);
                if(err!=1) {
                    printk(DBGMSK_PRX_G4"[cm3623][ps] addr=0x%x val=0x%x err=%d\n", (CM3623_PS_CTL_REG >> 1), idata[0], err);
                }
                else {
                    printk(DBGMSK_PRX_G4"[cm3623][ps] addr=0x%x val=0x%x \n",(CM3623_PS_CTL_REG >> 1), idata[0]);
                }
            }

            break;
        case ATD_ASK_PR_STATUS: //get data
            printk(DBGMSK_PRX_G4"[cm3623][ps] ATD_ASK_PR_STATUS \n");
            data = (void __user *) arg;
            if (data == NULL) {
                printk(DBGMSK_PRX_G4"[cm3623][ps] null data!!! \n");
                ret = -EFAULT;
                break;
            }

            if (copy_from_user(&value, data, sizeof(int))) {
                printk(DBGMSK_PRX_G4"[cm3623][ps] copy_from_user fail (%d).\n", (-EFAULT));
                return -EFAULT;
            }
            else {
                printk(DBGMSK_PRX_G4"[cm3623][ps] value=%d \n", value);
            }

            if(value>=0 && value<=255) {
                if(g_ps_threshold!=value) {
                    printk(DBGMSK_PRX_G4"[cm3623][ps] promxdl_dev_ioctl, changing PS setting (%d)\n", value);
                    msg->addr = CM3623_PS_THD_REG >> 1;
                    msg->flags = 0; //0 - write.
                    msg->len = 1;
                    msg->buf = idata;
                    idata[0] = value; //PS threshould range is 3~255.
                    ret = i2c_transfer(cm3623_client->adapter, msg, 1);
                    if(ret!=1) {
                        printk(DBGMSK_PRX_G4"[cm3623][ps] promxdl_dev_ioctl, i2c failed!\n");
                        ret = -EFAULT;
                    }
                    else {
                        g_ps_threshold = value;
                    }
                }
                if(ret!=(-EFAULT)) {
                    g_cm3623_int_state = g_cm3623_device.read_int_pin_state();
                    g_cm3623_int_state = g_cm3623_int_state ? 1 : 0;
                    if (copy_to_user( (void *)data, &g_cm3623_int_state, sizeof(g_cm3623_int_state))) {
                        printk(DBGMSK_PRX_G4"[cm3623][ps] promxdl_dev_ioctl, copy_to_user failed!\n");
                        ret = -EFAULT;
                    }
                }
            }
            else {
                printk(DBGMSK_PRX_G4"[cm3623][ps] promxdl_dev_ioctl, wrong PS setting!\n");
                ret = -EFAULT;
            }

            break;
        ///////////////////////////////////////////////////////////////
        case IOCTL_ENABLE_CTRL: //turn on/off sensor power
            printk(DBGMSK_PRX_G4"[cm3623][ps] IOCTL_ENABLE_CTRL %d\n", (int)arg);

            value = (int)arg;
            printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_ioctl, IOCTL_ENABLE_CTRL(%d)\n",value);
            g_ioctl_enable = value;

            break;
        case CM3623_ANDROID_OR_OMS:
            value = (int)arg;
            g_android_or_oms = value;
            printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_ioctl, g_android_or_oms=%d\n",g_android_or_oms);
            break;
        default:
            printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_ioctl, unknown cmd(0x%x)\n", cmd);
            break;
    }
    return ret;
}

static int proxmdev_release(struct inode *inode, struct file *filp)
{
    printk(DBGMSK_PRX_G4"[cm3623][ps] proxmdl_dev_release.\n");

    return 0;
}

/////////////////////////////////////////////////////////////////////////////////
//---ambient light sensor part---
//
static enum proximity_property ambientDev_properties[] = {
    /* int */
    SENSORS_PROP_INTERVAL,
    SENSORS_PROP_THRESHOLD,
    SENSORS_PROP_MAXRANGE,      /* read only */
    SENSORS_PROP_RESOLUTION,    /* read only */
    SENSORS_PROP_VERSION,       /* read only */
    SENSORS_PROP_CURRENT,       /* read only */
    SENSORS_PROP_DBG,           /* Add for debug only */
    /* char */
    SENSORS_PROP_SWITCH,
    SENSORS_PROP_VENDOR,        /* read only */
    SENSORS_PROP_CALIBRATION,   /* calibration value */
    SENSORS_PROP_ADC,           /* adc raw data */
    SENSORS_PROP_K_ADC,         /* adc raw data w/ calibrated */
    SENSORS_PROP_LUX,            /* lux data (calibrated) */
    SENSORS_PROP_ATD_STATUS,    /* for atd mode only */
    SENSORS_PROP_ATD_ADC,        /* for atd mode only */
    SENSORS_PROP_THD_THRESHOLD /* INT threshold   */
};

static struct file_operations ambientDev_fops = {
    .owner = THIS_MODULE,
    .open = ambientDev_open,
    .read = ambientDev_read,
    .poll = ambientDev_poll,
    //.ioctl = ambientdl_dev_ioctl,
    .release = ambientDev_release,
};

struct proximity_class_dev g_ambientDev = {
    .id = SENSORS_LIGHT,
    .name = "lsensor",
    .num_properties = ARRAY_SIZE(ambientDev_properties),
    .properties = ambientDev_properties,
    .get_property = ambientDev_get_property,
    .put_property = ambientDev_put_property,
    .fops = &ambientDev_fops
};


static int ambientDev_get_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
    printk(DBGMSK_PRX_G2"[cm3623][als] ambientdl_get_property +.\n");

    switch( property ) 
    {
        case SENSORS_PROP_THRESHOLD:
            printk(DBGMSK_PRX_G2"[cm3623][als] config IT_ALS by using \"echo XXX > /sys/class/sensors/sensor4/threshold\" XXX is only 0~3. \n");
            printk(DBGMSK_PRX_G2"[cm3623][als] SENSORS_PROP_THRESHOLD.\n");
            val->intval = 0;
            break;

        case SENSORS_PROP_INTERVAL:
            printk(DBGMSK_PRX_G2"[cm3623][als] config GAIN_ALS by using \"echo XXX > /sys/class/sensors/sensor4/interval\" XXX is only 0~3. \n");
            printk(DBGMSK_PRX_G2"[cm3623][als] SENSORS_PROP_INTERVAL.\n");
            val->intval = g_interval;
            break;

        case SENSORS_PROP_MAXRANGE:
            printk(DBGMSK_PRX_G2"[cm3623][als] SENSORS_PROP_MAXRANGE.\n");
            val->intval = 128;  
            break;

        case SENSORS_PROP_RESOLUTION:
            printk(DBGMSK_PRX_G2"[cm3623][als] SENSORS_PROP_RESOLUTION.\n");
            val->intval = 1;
            break;

        case SENSORS_PROP_VERSION:
            printk(DBGMSK_PRX_G2"[cm3623][als] SENSORS_PROP_VERSION.\n");
            val->intval = 1;    
            break;

        case SENSORS_PROP_CURRENT:
            printk(DBGMSK_PRX_G2"[cm3623][als] SENSORS_PROP_CURRENT.\n");
            val->intval = 30;   
            break;

        case SENSORS_PROP_SWITCH:
            //printk(DBGMSK_PRO_G0 "[proxmdl]: SENSORS_PROP_SWITCH (%d).\n", g_kxtf9_switch_on);
            printk(DBGMSK_PRX_G2"[cm3623][als] get switch = %d.\n", g_HAL_als_switch_on);
            val->intval = g_HAL_als_switch_on;
            break;

        case SENSORS_PROP_VENDOR:
            printk(DBGMSK_PRX_G2"[cm3623][als] SENSORS_PROP_VENDOR.\n");
            sprintf(val->strval, "CAPELLA");
            break;

        /* Add for debug only */
        case SENSORS_PROP_DBG:
            val->intval = g_ambient_dbg;
            printk(DBGMSK_PRX_G2"[cm3623][als] dbg = %d.\n", g_ambient_dbg);
            printk(DBGMSK_PRX_G2"[cm3623][als] als_IT(%d),als_GAIN(%d)\n", als_IT, als_GAIN);
            break;
        case SENSORS_PROP_ADC:
            val->intval = g_cm3623_light_adc;
            printk(DBGMSK_PRX_G2"[cm3623][als] get adc property: %d\n", val->intval);
            break;
        case SENSORS_PROP_K_ADC:
            val->intval = g_cm3623_light_k_adc;
            printk(DBGMSK_PRX_G2"[cm3623][als] get k_adc property: %d\n", val->intval);
            break;
        case SENSORS_PROP_LUX:
            val->intval = g_cm3623_light;
            printk(DBGMSK_PRX_G2"[cm3623][als] get lux property: %d\n", val->intval);
            break;

       case SENSORS_PROP_ATD_STATUS:
        {
            int als_sts =0, als_adc =0, ps_sts =0;

            atd_write_status_and_adc_2(&als_sts, &als_adc, &ps_sts);
            val->intval = als_sts;
            printk(DBGMSK_PRX_G4"[cm3623][als] get atd status: %d\n", val->intval);
            break;
        }
        case SENSORS_PROP_ATD_ADC:
        {
            int als_sts =0, als_adc =0, ps_sts =0;

            atd_write_status_and_adc_2(&als_sts, &als_adc, &ps_sts);
            val->intval = als_adc;
            printk(DBGMSK_PRX_G4"[cm3623][als] get atd adc: %d\n", val->intval);
            break;
        }
// ++Louis
        case SENSORS_PROP_THD_THRESHOLD:
        {
            int i,j,temp,iDigit[4];
            int indx = 0;
            static int lux_range[4]= {180, 90, 45, 22};
            static int gain_diff[4]= {1, 2, 4, 8};

            val->intval = g_als_threshold;
            temp = g_als_threshold;

            printk(DBGMSK_PRX_G2"\nGAIN_ALS\t\tTHD_ALS\t\tTHD_RANGE\n");
            for(i=0; i<=3; i++) 
            {
                printk(DBGMSK_PRX_G2"0x%x",i);
                for(j=0; j<=3; j++)
                    printk(DBGMSK_PRX_G2"\t\t0x%x\t\t+- %d\n\t",j,lux_range[i]*gain_diff[j]);
                printk("\n");
            }    
            
            printk(DBGMSK_PRX_G2"detect lux =(%d)\t INT_threshold = ",g_cm3623_light);

            do
            {
                 iDigit[indx] = temp % 2;
                 temp = temp / 2;
                 indx++;
            } while(temp > 0); 
            
            for(i=0; i<indx; i++)
                printk(DBGMSK_PRX_G2"%d",iDigit[indx-i-1]);
            printk(DBGMSK_PRX_G2"\n");

            break;
       }
// --Louis
       default:
            printk(DBGMSK_PRX_G2"[cm3623][als] default\n");
            return -EINVAL;
    }

    //printk(DBGMSK_PRO_G0 "[cm3623]: ambientdl_get_property -.\n");
    return 0;
}

static int cm3623_turn_onoff_als(bool bOn)
{
    struct i2c_msg msg[1];
    unsigned char data[2] = {0,0};
    int err = 1;
    DEFINE_WAIT(lwait);

    printk(DBGMSK_PRX_G2"[cm3623][als]++Turn on ambient sensor\n");

    if(g_cm3623_als_switch_on != bOn) {
        if(1 == bOn) { //turn on ambient sensor.
            printk(DBGMSK_PRX_G2"[cm3623][als] Turn on ambient sensor\n");

            if(g_cm3623_switch_earlysuspend==1) {
                printk(DBGMSK_PRX_G2"[cm3623][als]senosr_switch_on: switch on later when late_resume.\n");
                g_ambient_suspended = 1;   //set 1 that late_resume func will turn on als
            }
            else {
                g_cm3623_als_switch_on = 1;

                //polling initial amibent light ++
                msg->addr = cm3623_client->addr;
                msg->flags = 0; //0 - write.
                msg->len = 1;
                msg->buf = data;
                #ifndef ALS_WORD_MODE
                data[0] = ((als_IT<<2) | (als_GAIN<<6) | data[0]);
                #else
                data[0] = ((als_IT<<2) | data[0]);
                data[0] |= (0x2); //WDM=1.
                #endif
                err = i2c_transfer(cm3623_client->adapter, msg, 1);

                if (err != 1) {
                    printk(DBGMSK_PRX_G2"[cm3623[als] sending initial amibent light fail\n");
                    goto light_reset;
                }

                prepare_to_wait(&als_polling_wq_head, &lwait, TASK_INTERRUPTIBLE);
                
                if(g_HAL_als_switch_on == 1) {

                    queue_delayed_work(cm3623light_workqueue, &g_light_work, (HZ*10/1000));
                //    printk(DBGMSK_PRX_G2"[cm3623[als] (%s): schedule ++\n", __FUNCTION__);

                //    schedule_timeout(jiffies_to_msecs(2000));

               //     printk(DBGMSK_PRX_G2"[cm3623[als] (%s): schedule --\n", __FUNCTION__);
                }
                finish_wait(&als_polling_wq_head, &lwait);

                if(g_cm3623_als_switch_on == 0) {
                    g_polling_count = 0;

                    if(delayed_work_pending(&g_light_work)) {
                        cancel_delayed_work_sync(&g_light_work);
                    }
                    return 0;
                }
                // --

                //initialization
                msg->addr = CM3623_INIT_CMD>>1;    // 0x92
                msg->flags = 0; 
                msg->len = 1;
                msg->buf = data;
                data[0] = INT_MODE;
                err = i2c_transfer(cm3623_client->adapter, msg, 1); 

                if (err != 1) {
                    printk(DBGMSK_PRX_G2"[cm3623[als] setting interrupt initialization fail\n");
                    goto light_reset;
                }

                //set initial THD
                msg->addr = CM3623_ALS_CTL_REG>>1;    // address 0x90
                #ifndef ALS_WORD_MODE
                    //  data[0] = (ALS_RESOLUTION_11 | data[0]);
                    data[0] = ( (als_IT<<2) | (als_GAIN<<6) | data[0]);
                #else
                    data[0] = INIT_ALS;
                    data[0] = ( GAIN_ALS_DIV8 | (0x00<<4) | (als_IT<<2) | data[0]);  //set +/-23 lux
                    data[0] |= (0x2); //WDM=1.
                #endif
                err = i2c_transfer(cm3623_client->adapter, msg, 1);

                if (err != 1) {
                    printk(DBGMSK_PRX_G2"[cm3623][als] setting initial threshold fail\n");
                    goto light_reset;
                }

                else {
                    printk(DBGMSK_PRX_G2"[cm3623][als] setting interrupt initial threshold +/- 23 lux\n");
                }

                //enable ALS INT
                msg->addr = CM3623_PS_CTL_REG>>1;       //0xF0

                if(g_proxm_switch_on == 0)
                    data[0] = ( INIT_ALS | (ps_IT<<4) | (0x1<<3) );
                else
                    data[0] = ( INIT_ALS | (ps_IT<<4) | (0x3<<2));

                err = i2c_transfer(cm3623_client->adapter, msg, 1);

                if(err!=1) {
                    printk(DBGMSK_PRX_G2"[cm3623][als] addr=0x%x val=0x%x err=%d\n", msg->addr, data[0], err);
                    goto light_reset;
                }

                else {
                    printk(DBGMSK_PRX_G2"[cm3623][als] addr=0x%x val=0x%x \n", msg->addr, data[0]);
                }

             //   printk(DBGMSK_PRX_G2"[cm3623][als] on/off als, launch polling thread\n");
               // queue_delayed_work(cm3623light_workqueue, &g_light_work, (HZ*10/1000));
                printk(DBGMSK_PRX_G2"[cm3623][als] turned on\n");
            }
        } // if(bOn==1)
        else { //turn off
                printk(DBGMSK_PRX_G2"[cm3623][als] Turn OFF ambient sensor\n");
                err = waitqueue_active(&ambient_wq_head);
                printk(DBGMSK_PRX_G2"[cm3tu623][als] ambient_wq_head(%d)\n", err);

                if(g_ambient_suspended==1) {
                    printk(DBGMSK_PRX_G2"[cm3623][als] switch off was done in early_suspend.\n");
                    g_cm3623_als_switch_on = 0;
                }
                else {
                    g_cm3623_als_switch_on = 0;
                    wake_up_interruptible(&als_polling_wq_head);

                    // address 0x90
                    msg->addr = cm3623_client->addr;
                    msg->flags = 0; //0 - write.
                    msg->len = 1;
                    msg->buf = data;
                    data[0] = (INIT_ALS | 0x1); //shutdown ALS.

                    err = i2c_transfer(cm3623_client->adapter, msg, 1);

                    if(err != 1) {
                        printk(DBGMSK_PRX_G2"[cm3623][als] addr=0x%x val=0x%x err=%d\n", msg->addr, data[0], err);
                        goto light_reset;
                    }

                    else {
                        printk(DBGMSK_PRX_G2"[cm3623][als] addr=0x%x val=0x%x \n", msg->addr, data[0]);
                    }

                    printk(DBGMSK_PRX_G2"[cm3623][als] turned off\n");
                }
        }
    }

    printk(DBGMSK_PRX_G2"[cm3623][als] --Turn on ambient sensor\n");
    return err;

light_reset:
        sensor_mode = LIGHT;
        light_proxim_error_reset(sensor_mode, bOn);
        return 0;

}

static struct write_calvalue {
    struct work_struct write_calvalue_work;
    int calvalue;
} *cm3623_write_calvalue;

static int ambientDev_put_property(struct proximity_class_dev *sdev, enum proximity_property property, union proximity_propval *val)
{
    static bool bFirst = true;
    //int ret = 0;

    switch (property) 
    {
        case SENSORS_PROP_SWITCH:
            printk(DBGMSK_PRX_G2"[cm3623][als] put SENSORS_PROP_SWITCH (%d,%d).\n", 
                    (val->intval), g_HAL_als_switch_on);

            //read calibration value
            if(bFirst) {
                printk(DBGMSK_PRX_G2"[cm3623][als] put switch 1st read calvalue\n");
                read_lightsensor_calibrationvalue();
                bFirst = false;
            }

            if(val->intval > 0) {
                g_HAL_als_switch_on = 1;
            }
            else {
                g_HAL_als_switch_on = 0;
            }

            if(!g_bIsP01Attached) {
                printk(DBGMSK_PRX_G2"[cm3623][als] sensor switch, turn on/off cm3623: %d\n",
                         g_HAL_als_switch_on);

                cm3623_turn_onoff_als(g_HAL_als_switch_on);
            }
            else {
                printk(DBGMSK_PRX_G2"[cm3623][als] sensor switch, turn on/off al3010: %d\n",
                         g_HAL_als_switch_on);

                set_als_power_state_of_P01(g_HAL_als_switch_on);
           //     cm3623_turn_onoff_proxm(0);

                if(1 == g_HAL_als_switch_on && 0 == g_cm3623_als_switch_on) {
                    printk(DBGMSK_PRX_G2"[cm3623][als] sensor switch, launch polling thread\n");
                    queue_delayed_work(cm3623light_workqueue, &g_light_work, msecs_to_jiffies(0));
                }
            }

            break;

        case SENSORS_PROP_THRESHOLD:
            /*
            printk(DBGMSK_PRO_G1 "[ambientdl]: config IT_ALS (0x%x)\n", val->intval);
            if(val->intval>=0 && val->intval<=3) {
                gpio_set_value(g_proxm_pwr_pin, 0);
                msleep(1);
                gpio_set_value(g_proxm_pwr_pin, 1);
                gpio_free(g_proxm_pwr_pin);
                als_IT = val->intval;
                ret = cm3623_reset();
            }
            */
            break;

        case SENSORS_PROP_INTERVAL:
            printk(DBGMSK_PRX_G2"[cm3623][als] put SENSORS_PROP_INTERVAL. %d\n", val->intval);
            if(val->intval < 100) {
                g_interval = 100;
            }
            else {
                g_interval = val->intval;
            }
            /*
            printk(DBGMSK_PRO_G1 "[ambientdl]: config GAIN_ALS (0x%x)\n", val->intval);
            //g_interval = val->intval;
            if(val->intval>=0 && val->intval<=3) {
                gpio_set_value(g_proxm_pwr_pin, 0);
                msleep(1);
                gpio_set_value(g_proxm_pwr_pin, 1);
                gpio_free(g_proxm_pwr_pin);
                als_GAIN = val->intval;
                ret = cm3623_reset();
            }
            */
            break;

        /* Add for debug only */
        case SENSORS_PROP_DBG:
            g_ambient_dbg = val->intval;
            printk(DBGMSK_PRX_G2"[cm3623][als] dbg = %d.\n", g_ambient_dbg);
            break;
        case SENSORS_PROP_CALIBRATION:
            if(val->intval > 30720)
                g_cm3623_light_calibration_fval_x1000 = 30720;
            else if(val->intval < 1024)
                g_cm3623_light_calibration_fval_x1000 = 1024;
            else
                g_cm3623_light_calibration_fval_x1000 = val->intval;
            printk(DBGMSK_PRX_G2"[cm3623][als] calibration val x1000= %d\n", g_cm3623_light_calibration_fval_x1000);

            cm3623_write_calvalue = kmalloc(sizeof(struct write_calvalue), GFP_KERNEL);
            INIT_WORK(&cm3623_write_calvalue->write_calvalue_work, write_lightsensor_calibrationvalue_work);

            cm3623_write_calvalue -> calvalue = g_cm3623_light_calibration_fval_x1000;

            queue_work(cm3623light_workqueue, &cm3623_write_calvalue->write_calvalue_work);
            break;
        case SENSORS_PROP_THD_THRESHOLD:
            printk(DBGMSK_PRX_G2"[cm3623][als] config THRESHOLD (0x%x).\n", val->intval);
            if(val->intval >= 0 && val->intval <= 15) 
            {
                g_als_threshold = val->intval;
                write_lightsensor_thd_als(g_als_threshold);
            }
            else {
                printk(DBGMSK_PRX_G2"[cm3623][als] ERROR!!! OUT OF THRESHOLD (0~15)!!! \n");
            }
            break;

        default:
            printk(DBGMSK_PRX_G2"[cm3623][als] put default.\n");
            return -EINVAL;
    }

    return 0;
}

static int ambientDev_open(struct inode *inode, struct file *file)
{
    int ret = 0;

    printk(DBGMSK_PRX_G2"[cm3623][als] ambientdl_dev_open \n");

    if (file->f_flags & O_NONBLOCK) {
        printk(DBGMSK_PRX_G2"[cm3623][als] ambientdl_dev_open (O_NONBLOCK)\n");
    }
    atomic_set(&ambient_update, 0); //initialize atomic.

    ret = nonseekable_open(inode, file);

    return ret;
}

static ssize_t ambientDev_read(struct file *file, char __user * buffer, size_t size, loff_t * f_pos)
{
    printk("[cm3623]: ambientDev_read\n");
    return 0;
}


static unsigned int ambientDev_poll(struct file *filep, poll_table * wait)
{
//  if(g_ambient_dbg==1)
//  printk("[cm3623]: ambientdl_dev_poll++\n");

    if(g_cm3623_switch_earlysuspend==1 || g_cm3623_als_switch_on==0) {
//      if(g_ambient_dbg==1)
        printk(DBGMSK_PRX_G2"[cm3623][als] ambientdl_dev_poll-- not ready\n");
        return 0;
    }

    poll_wait(filep, &ambient_wq_head, wait);
    
    if(atomic_read(&ambient_update)) {
//      if(g_ambient_dbg==1)
//      printk("[cm3623]: ambientdl_dev_poll-- (POLLIN %d)\n", atomic_read(&ambient_update));
        return (POLLIN | POLLRDNORM);
    }

//  if(g_ambient_dbg==1)
//  printk("[cm3623]: ambientdl_dev_poll--\n");
    return 0;
}

static int ambientDev_release(struct inode *inode, struct file *filp)
{
    printk(DBGMSK_PRX_G2"[cm3623][als] ambientdl_dev_release.\n");

    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//  Low layer driver part
//


//
// Support ATD light sensor test
//

#define LSENSOR_ASUS_NV_FILE  "/data/asusdata/lsensor.nv"

int atd_read_P_L_sensor_adc(int *adc)
{
    int status = 0;
    int ret = 0;
    u16 alsdata = 0;
    //    u16 pData=0; //proximity data.
    int lux = 0;
    u8 lobyte = 0;
    int idx = 0;

    printk(DBGMSK_PRX_G5"[cm3623][atd]readadc: trying to turn on lsensor\n");

    status = proximity_als_turn_on(1);

    if(0 == status) {
        //if(g_ambient_dbg==1)
        printk(DBGMSK_PRX_G5"[cm3623][atd]readadc: lsensor is not on\n");
        return status;
    }


    *adc = 0;

    for(idx = 0; idx < 5; idx++)
    {
        //0x23 or 0x93 CM3623_ALS_DATA_REG2 (LSB)
        ret = cm3623_read_reg(cm3623_client, CM3623_ALS_DATA_REG2, &alsdata); //get ALS data.
        if(ret!=1) {
            printk(DBGMSK_PRX_G5"[cm3623][atd]readadc: reg2 error! (cmd=0x%x)\n",CM3623_ALS_DATA_REG2);
            alsdata = 0;
            ret = -EIO;
        }
        else {
            lobyte = (u8)alsdata;
            printk(DBGMSK_PRX_G5"[cm3623][atd]readadc: read-LoByte: %d\n",(int)lobyte);
        }

        //0x21 or 0x91 (MSB)
        ret = cm3623_read_reg(cm3623_client, CM3623_ALS_DATA_REG1, &alsdata); //get ALS data.
        if(ret!=1) {
            printk(DBGMSK_PRX_G5"[cm3623][atd]readadc: reg1 error! (cmd=0x%x)\n",CM3623_ALS_DATA_REG1);
            alsdata = 0;
            ret = -EIO;
        }
        else {
            printk(DBGMSK_PRX_G5"[cm3623][atd]readadc: read-HiByte: %d\n", (int)alsdata);
            alsdata = ((alsdata<<8) | lobyte);
        }

        msleep(100);
    }

    *adc = alsdata;

    lux = (alsdata * 875) / 10000; // multiply (0.0875*10000) to avoid float value...

    printk(DBGMSK_PRX_G5"[cm3623][atd]readadc steps=%d lux=%d\n", *adc, lux);

/*
    if(g_proxm_switch_on==1) {
        cm3623_read_reg(cm3623_client, CM3623_PS_DATA_REG, &pData);
        input_report_abs(g_cm3623_data_ps->input_dev, ABS_DISTANCE, pData);
        input_event(g_cm3623_data_ps->input_dev, EV_SYN, SYN_REPORT, 1);
        input_sync(g_cm3623_data_ps->input_dev);
    }
    if(g_proxm_dbg) {
        printk("[cm3623]: pData=%d\n", pData);
    }
*/

    proximity_als_turn_on(0);

    return status;
}

static int atd_write_status_and_adc_2(int *als_sts, int *als_adc, int *ps_sts)
{
    int adc = 0;
    int status = 0; // 0x1: light sensor is ok, 0x2: proximity is ok, 0x3: both are ok, 0x0: both failed

    printk(DBGMSK_PRX_G5"[cm3623][atd]writests_2: started\n");

    status = atd_read_P_L_sensor_adc(&adc);

    if(status & 0x01) {
        *als_sts = 1;
        *als_adc = adc;
    }
    else {
        *als_sts = 0;
        *als_adc = 0;
    }

    if(status & 0x02) {
        *ps_sts = 1;
    }
    else {
        *ps_sts = 0;
    }

    printk(DBGMSK_PRX_G5"[cm3623][atd]writests_2: get adc, als_sts:%d, als_adc:%d, ps_sts:%d\n", 
            *als_sts, *als_adc, *ps_sts);

    return status;
}


bool read_lightsensor_calibrationvalue()
{
    struct file *fp = NULL; 
    loff_t pos_lsts = 0;
    char readstr[8];
    int ori_val = 0, readlen =0;
    mm_segment_t old_fs;

    printk(DBGMSK_PRX_G5"[cm3623] ++read_lsensor_calvalue open\n");

    fp = filp_open(LSENSOR_ASUS_NV_FILE, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
    //fp = filp_open(pFile, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
    if(IS_ERR_OR_NULL(fp)) {
        printk(DBGMSK_PRX_G5"[cm3623] read_lsensor_calvalue open (%s) fail\n", LSENSOR_ASUS_NV_FILE);
        return false;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    if(fp->f_op != NULL && fp->f_op->read != NULL) {
        pos_lsts = 0;

       // printk(DBGMSK_PRX_G5"[cm3623] strlen:%d\n", strlen(readstr));
        readlen = fp->f_op->read(fp, readstr, 6, &pos_lsts);
      //  printk(DBGMSK_PRX_G5"[cm3623] strlen_2:%s(%d), readlen=%d\n", readstr, strlen(readstr), readlen);
        readstr[readlen] = '\0';
        printk(DBGMSK_PRX_G5"[cm3623] strlen_3:%s(%d)\n", readstr, strlen(readstr));
    }
    else {
        printk(DBGMSK_PRX_G5"[cm3623] read_lsensor_calvalue, f_op=NULL or op->read=NULL\n");
    }

    set_fs(old_fs);
    filp_close(fp, NULL);

    sscanf(readstr, "%d", &ori_val);

    //limit the calibration value range
    if(ori_val < 1024)
        g_cm3623_light_calibration_fval_x1000 = 1024;
    else if(ori_val > 30720)
        g_cm3623_light_calibration_fval_x1000 = 30720;
    else
        g_cm3623_light_calibration_fval_x1000 = ori_val;

    printk(DBGMSK_PRX_G5"[cm3623] read_lsensor_calvalue: Ori: %d, Cal: %d\n", 
            ori_val, g_cm3623_light_calibration_fval_x1000);

    printk(DBGMSK_PRX_G5"[cm3623] --read_lsensor_calvalue open\n");
    return true;
}


static void write_lightsensor_calibrationvalue_work(struct work_struct *work)
{
    struct file *fp = NULL; 
    struct write_calvalue *this = NULL;
    loff_t pos_lsts = 0;
    char writestr[8];
    mm_segment_t old_fs;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    this = container_of(work, struct write_calvalue, write_calvalue_work);

    fp = filp_open(LSENSOR_ASUS_NV_FILE, O_RDWR|O_CREAT|O_TRUNC, S_IRWXU|S_IRWXG|S_IRWXO);
    if(IS_ERR_OR_NULL(fp)) {
        printk(DBGMSK_PRX_G5"[cm3623] write_lsensor_calvalue open (%s) fail\n", LSENSOR_ASUS_NV_FILE);
        return;
    }

    sprintf(writestr, "%d", this->calvalue);

    printk(DBGMSK_PRX_G5"[cm3623] write_lsensor_calvalue = %d[%s(%d)]\n", 
            this->calvalue, writestr, strlen(writestr));

    if(fp->f_op != NULL && fp->f_op->write != NULL){
        pos_lsts = 0;

        fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);

    }

    else
        printk(DBGMSK_PRX_G5"[cm3623] write_lsensor_calvalue fail\n");
        
    set_fs(old_fs);
    filp_close(fp, NULL);

    return;
}

// ++Louis
static int write_lightsensor_thd_als(u8 thd_val)
{
    struct i2c_msg msg[1];
    unsigned char data[2] = {0,0};
    int ret = 0;
    //set new threshold    
    msg->addr = CM3623_ALS_CTL_REG >> 1;    // address 0x90
    msg->flags = 0;                   
    msg->len = 1;
    msg->buf = data;
#ifndef ALS_WORD_MODE
    data[0] = (ALS_RESOLUTION_11 | data[0]);
    data[0] = ( (als_IT<<2) | (als_GAIN<<6) | data[0]);
#else
    data[0] = (0xff && 0);
    data[0] = ( (thd_val<<4) | (als_IT<<2) | data[0]);  //set +/- step
    data[0] |= (0x2); //WDM=1.
#endif
    ret = i2c_transfer(cm3623_client->adapter, msg, 1);
    if(ret != 1) {
        printk(DBGMSK_PRX_G3"[cm3623][als] setting als threshold error\n");
    }
    
    return ret;

}
// --Louis

static int proxm_set_threshold(int value)
{
    int err=1;
    struct i2c_msg msg[1];
    unsigned char data[2] = {0,0};

    printk(DBGMSK_PRX_G4"[cm3623][ps] set threshold: %d\n", value);

    // cmd 0xB2 or 0xF2
    msg->addr = CM3623_PS_THD_REG >> 1;
    msg->flags = 0; //0 - write.
    msg->len = 1;
    msg->buf = data;
    data[0] = value; //PS threshould range is 3~255.

    err = i2c_transfer(cm3623_client->adapter, msg, 1);

    if(err!=1) {
        printk(DBGMSK_PRX_G0"[cm3623]: proxm_set_threshold addr=0x%x val=0x%x err=%d\n", msg->addr, data[0], err);
        printk(DBGMSK_PRX_G0"[cm3623]: WARNING! Please check sensor is power on!\n");
    }

    return err;
}

//static irq_handler_t proxm_interrupt_handler(int irq, void *dev_id)
static irqreturn_t als_proxm_interrupt_handler(int irq, void *dev_id)
{       
#if 0
    //u8 value = 1;
  /*   
    if (g_cm3623_als_switch_on && g_proxm_switch_on != 1) {
        printk(DBGMSK_PRX_G4"[cm3623][als] als_interrupt_handler, irq:%d ++\n", irq);
        schedule_work(&cm3623_als_ISR_work);
        return IRQ_HANDLED;
    }

    printk(DBGMSK_PRX_G4"[cm3623][ps] proxm_interrupt_handler, irq:%d ++\n", irq);
    if( (g_proxm_switch_on==0)&&(g_i2ctest_proxm_switch==0) ) {
        printk(DBGMSK_PRX_G4"[cm3623][ps] proxm_interrupt_handler: not yet switch on\n");
        return IRQ_HANDLED;
    }

    value = g_cm3623_device.read_int_pin_state();
    printk(DBGMSK_PRX_G4"[cm3623][ps] proxm_interrupt_handler int_state:%d\n", value);

    g_cm3623_int_state = (value?1:0);

    printk(DBGMSK_PRX_G4"[cm3623][ps] proxm_interrupt_handler (%d -> %d) \n", g_last_cm3623_int_state, g_cm3623_int_state);

    if(g_last_cm3623_int_state!=g_cm3623_int_state) {
        g_last_cm3623_int_state = g_cm3623_int_state;
        printk(DBGMSK_PRX_G4"[cm3623][ps] proxm_interrupt_handler, state_change\n");
        //complete(&proxm_comp);

        input_report_abs(g_cm3623_data_ps->input_dev, ABS_DISTANCE, g_cm3623_int_state);
        input_event(g_cm3623_data_ps->input_dev, EV_SYN, SYN_REPORT, 1);
        input_sync(g_cm3623_data_ps->input_dev);
#ifndef INPUT_EVENT_MODE
        atomic_inc(&proxm_update);
        printk(DBGMSK_PRX_G4"[cm3623][ps] proxm_interrupt_handler fire(%d)\n",atomic_read(&proxm_update));
        wake_up_interruptible(&proxm_wq_head);
#endif
    }

    printk(DBGMSK_PRX_G4"[cm3623][ps] proxm_interrupt_handler--\n");
*/
#else
         queue_work(cm3623light_workqueue, &cm3623_ISR_work);

         if (g_proxim_state == 1) {
                wake_lock_timeout(&proximity_wake_lock, 1 * HZ);
         }

         return IRQ_HANDLED;
#endif

}

/*
static void myProxm_poll_work(struct work_struct *work)
{
    int ret = 0;
    u16 pVal=0; //proximity data.
    u8 lobyte = 0;

    printk("[cm3623]myProxm_poll_work\n");

    if(g_ambient_switch_on==0) {
        //if(g_ambient_dbg==1)
        printk("[cm3623]: light_poll_work leaves!\n");
        return;
    }
    
    g_cm3623_light = 0;

#ifndef ALS_WORD_MODE
    //0x21 or 0x91
    ret = cm3623_read_reg(cm3623_client, CM3623_ALS_DATA_REG1, &g_cm3623_light); //get ALS data.
    if(ret!=1) {
        printk("[cm3623]: ambientdl_dev_read. error!\n");
        ret = -EIO;
    }
#else //WMD=1
    //0x23 or 0x93 CM3623_ALS_DATA_REG2 (LSB)
    ret = cm3623_read_reg(cm3623_client, CM3623_ALS_DATA_REG2, &g_cm3623_light); //get ALS data.
    if(ret!=1) {
        printk("[cm3623]: ambientdl_dev_read. error! (cmd=0x%x)\n",CM3623_ALS_DATA_REG2);
        ret = -EIO;
    }
    else {
        lobyte = g_cm3623_light;
        //printk("[cm3623]: read-LoByte (%d)\n",lobyte);
    }
    //0x21 or 0x91 (MSB)
    ret = cm3623_read_reg(cm3623_client, CM3623_ALS_DATA_REG1, &g_cm3623_light); //get ALS data.
    if(ret!=1) {
        printk("[cm3623]: ambientdl_dev_read. error! (cmd=0x%x)\n",CM3623_ALS_DATA_REG1);
        ret = -EIO;
    }
    else {
        //printk("[cm3623]: read-HiByte (%d)\n",g_cm3623_light);
        g_cm3623_light = ((g_cm3623_light<<8) | lobyte);
    }
#endif

    printk("[cm3623]Als value:%d\n", g_cm3623_light);

    if(g_proxm_switch_on==1) {
        cm3623_read_reg(cm3623_client, CM3623_PS_DATA_REG, &pVal);
        printk("[cm3623]PS val:%d\n", pVal);
    }

    ret = queue_delayed_work(cm3623light_workqueue, &g_light_work, (HZ*2000/1000)) ; //2000ms.
    if(ret==0) {
        printk("[cm3623]: queue_delayed_work failed!\n");
    }
}
*/

int get_adc_calibrated_lux_from_cm3623(void)
{
    int ret = 0;
    u8 tmp = g_cm3623_light;
    u8 lobyte = 0;
    int steps = 0;
    int i;

#ifdef INTERNAL_TEST
    int lux = 0;
#endif

#ifndef ALS_WORD_MODE
        //0x21 or 0x91
        ret = cm3623_read_reg(cm3623_client, CM3623_ALS_DATA_REG1, &g_cm3623_light); //get ALS data.
        if(ret!=1) {
            printk(DBGMSK_PRX_G0"[cm3623][als] ambientdl_dev_read. error!\n");
            g_cm3623_light = tmp;
            ret = -EIO;
        }
#else //WMD=1
        //0x23 or 0x93 CM3623_ALS_DATA_REG2 (LSB)
        ret = cm3623_read_reg(cm3623_client, CM3623_ALS_DATA_REG2, &g_cm3623_light); //get ALS data.
        if(ret!=1) {
            printk(DBGMSK_PRX_G0"[cm3623][als] ambientdl_dev_read. error! (cmd=0x%x)\n",CM3623_ALS_DATA_REG2);
            g_cm3623_light = tmp;
            ret = -EIO;
        }
        else {
            lobyte = g_cm3623_light;
            //printk("[cm3623]: read-LoByte (%d)\n",lobyte);
        }
        //0x21 or 0x91 (MSB)
        ret = cm3623_read_reg(cm3623_client, CM3623_ALS_DATA_REG1, &g_cm3623_light); //get ALS data.
        if(ret!=1) {
            printk(DBGMSK_PRX_G0"[cm3623][als] ambientdl_dev_read. error! (cmd=0x%x)\n",CM3623_ALS_DATA_REG1);
            g_cm3623_light = tmp;
            ret = -EIO;
        }
        else {
            //printk("[cm3623]: read-HiByte (%d)\n",g_cm3623_light);
            g_cm3623_light = ((g_cm3623_light<<8) | lobyte);
        }
#endif

    //printk(DBGMSK_PRO_G0 "[cm3623]: GP126=%d data[0]=%d mfpr=0x%x PLR3=0x%x PDR3=0x%x \n", ret, g_data[0], __raw_readl(0xFE01E06C), __raw_readl(0xFE019100+0x0000), __raw_readl(0xFE019100+0x000C));
//  __raw_writel(0x9080, 0xFE01E0DC);
//  printk(DBGMSK_PRO_G0 "[cm3623]: GPIO0=%d , mfpr=0x%x , GPIOPLR0=0x%x , GPIOPDR0=0x%x \n",ret, __raw_readl(0xFE01E0DC), __raw_readl(0xFE019000+0x0000), __raw_readl(0xFE019000+0x000C));

    steps = g_cm3623_light;

    g_cm3623_light_adc = steps;     // adc is the value that is returned from HW directly

#ifdef ALS_CALIBRATION
    steps = (u32)(steps * g_cm3623_light_calibration_fval_x1000) >> 10;     //apply calibration value
#endif

    g_cm3623_light_k_adc = steps;

    printk(DBGMSK_PRX_G3"[cm3623][als] read adc: %d, cal adc: %d, file adc: %d\n", g_cm3623_light, g_cm3623_light_k_adc, g_cm3623_light_adc);

    if (g_cm3623_light <= 32)    // INT minimum detection: +-32 adc
    {
        g_cm3623_light = g_light_level[0];
        return 0;
    }

#ifndef INTERNAL_TEST
    for(i=0;i<14;i++) {
        if( steps < g_light_map[2][i] ) {
            g_cm3623_light = g_light_level[i];
            break;
        }
        else if (steps > g_light_map[2][13])
            g_cm3623_light = g_light_level[13];
    }
    
    if(i==10) {
        g_cm3623_light = g_light_level[10]; //12000 lux.
    }
#else
    lux = steps * 875 / 10000; // multiply (0.0875*10000) to avoid float value...
#endif
    //if(g_ambient_dbg) {
        printk(DBGMSK_PRX_G3"[cm3623][als] last=%d light=%d steps=%d i=%d\n", g_last_cm3623_light, g_cm3623_light, steps, i);
    //}

#ifndef INTERNAL_TEST
    return i;
#else
    return lux;
#endif
}

//Louis 20120323 ++
#define  CHK_INT_ADD    0x18
static int g_GAIN_ALS[15] ={3,   3,  3,  3,  3,  3,  3,  3,  3,  3,   3,   3,   3,   0,    0 };
static int g_THD_ALS[15]  ={0,   0,  1,  1,  1,  1,  1,  1,  2,  2,   3,   3,   3,   1,    2 }; 
static int g_THD_LUX[15] = {23, 23, 45, 45, 45, 45, 45, 45, 90, 90, 180, 180, 180, 360,  720};

static void cm3623_interrupt_handler(struct work_struct *work)
{
    int ret;
    u16 buff;
    u8 value = 1;

    printk(DBGMSK_PRX_G2"[cm3623] cm3623_interrupt_handler int_state:%d\n", value);

    if ((g_proxim_state == 1) || (g_cm3623_switch_earlysuspend == 1)) {  //consider autosuspend in phone mode
        qup_i2c_resume(proximity_dev);  //wake up qup_i2c
    //   wait_event_timeout(proxm_isr_wq, 0, msecs_to_jiffies(80));        
    }

    ret = cm3623_read_reg(cm3623_client, CHK_INT_ADD, &buff); 
    if (ret == 1)
    {
        printk(DBGMSK_PRX_G2"[cm3623] +++++++++ interrput_pin_address = 0x%x\n", buff);

        if (buff == CM3623_ALS_CTL_REG)
        {
             queue_work(cm3623light_workqueue, &cm3623_light_interrupt_work);

        }
        else
             queue_work(cm3623light_workqueue, &cm3623_proximity_interrupt_work);
    }

    value = g_cm3623_device.read_int_pin_state();

    if (value == 0)
    {
        ret = cm3623_read_reg(cm3623_client, CHK_INT_ADD, &buff); 
        printk(DBGMSK_PRX_G2"[cm3623] --------- another interrput_pin_address = 0x%x\n", buff);

        if (buff == CM3623_ALS_CTL_REG)
        {
             queue_work(cm3623light_workqueue, &cm3623_light_interrupt_work);

        }
        else
             queue_work(cm3623light_workqueue, &cm3623_proximity_interrupt_work);
    }

}

static void light_interrupt_work(struct work_struct *work)
{
    struct i2c_msg msg[1];
    unsigned char data[2] = {0,0};
    int level, ret, indx;

    printk(DBGMSK_PRX_G3"/****************    ALS INT start  *******************/\n");
             
             indx = get_adc_calibrated_lux_from_cm3623();
             level = indx - 1;

             if(level <= 0)
                level = 0;
             else if (level >=14)
                level = 14;

             printk(DBGMSK_PRX_G3"[cm3623][als] level = %d, +/-threshold = %d lux\n",level, g_THD_LUX[level]); 

           //set new threshold    
             msg->addr = CM3623_ALS_CTL_REG >> 1;    // address 0x90
             msg->flags = 0;                   
             msg->len = 1;
             msg->buf = data;
#ifndef ALS_WORD_MODE
             data[0] = (ALS_RESOLUTION_11 | data[0]);
             data[0] = ( (als_IT<<2) | (als_GAIN<<6) | data[0]);
#else
             data[0] = (0xff && 0);
             data[0] = ( (g_GAIN_ALS[level]<<6) | (g_THD_ALS[level]<<4) | (als_IT<<2) | data[0]);  //set +/- step
             data[0] |= (0x2); //WDM=1.
#endif
             ret = i2c_transfer(cm3623_client->adapter, msg, 1);
             if(ret != 1) {
                  printk(DBGMSK_PRX_G3"[cm3623][als] setting als threshold error\n");
                  sensor_mode = LIGHT;
                  light_proxim_error_reset(sensor_mode, true);
                  return;
             }

             g_als_threshold = ((g_GAIN_ALS[level]<<2) | g_THD_ALS[level]);

             if(g_cm3623_light != g_last_cm3623_light) {
                  g_last_cm3623_light = g_cm3623_light;
                  als_lux_report_event(g_cm3623_light);
             }

             if(g_proxim_state == 1) 
                  wake_unlock(&proximity_wake_lock);

   printk(DBGMSK_PRX_G3"/****************    ALS INT end   *******************/\n");
}

static void proximity_interrupt_work(struct work_struct *work)
{
     int ret;
     u16 buff;
             if( (g_proxm_switch_on==0) && (g_i2ctest_proxm_switch==0) ) {
                  printk(DBGMSK_PRX_G4"[cm3623][ps] proxm_interrupt_handler: not yet switch on\n");
                  return ;
             }

             ret = cm3623_read_reg(cm3623_client, CM3623_PS_DATA_REG, &buff);
   printk(DBGMSK_PRX_G4"/----------------------------------------------------\n");

             printk(DBGMSK_PRX_G4"[cm3623][ps] PS_data = 0x%x\n", buff);
             g_psData = buff;

             if(buff >= g_ps_threshold) 
             {
                  input_report_abs(g_cm3623_data_ps->input_dev, ABS_DISTANCE, 0);
                  input_sync(g_cm3623_data_ps->input_dev);
                  g_proxim_state = 1;
                  printk(DBGMSK_PRX_G4"[cm3623][ps] panel off(%d)\n",g_proxim_state);
             }

             else 
             {
                //  enable_irq(MSM_GPIO_TO_INT(g_cm3623_device.irq));
                  input_report_abs(g_cm3623_data_ps->input_dev, ABS_DISTANCE, 1);
                  input_sync(g_cm3623_data_ps->input_dev);
                  //wake_unlock(&proximity_wake_lock);
                  g_proxim_state = 0; 
                  printk(DBGMSK_PRX_G4"[cm3623][ps] panel on(%d)\n",g_proxim_state);
             }
#ifndef INPUT_EVENT_MODE
                  atomic_inc(&proxm_update);
                  printk(DBGMSK_PRX_G4"[cm3623][ps] proxm_interrupt_handler, state_change\n");  
                  printk(DBGMSK_PRX_G4"[cm3623][ps] proxm_interrupt_handler fire(%d)\n",atomic_read(&proxm_update));

                  wake_up_interruptible(&proxm_wq_head);

                  printk(DBGMSK_PRX_G4"[cm3623][ps] proxm_interrupt_handler--\n");
   printk(DBGMSK_PRX_G4"/----------------------------------------------------\n");
                  return;
#endif            
}

//Louis 20120323 --
static int err_count = 0;
static void light_poll_work(struct work_struct *work)
{
    int ret = 0;
#ifdef INTERNAL_TEST
    u16 pData=0; //proximity data.
#endif
	printk(DBGMSK_PRX_G3"[cm3623][als] ---------------------  light_poll_work !\n");
    if(0 == g_HAL_als_switch_on) {
        //if(g_ambient_dbg==1)
        printk(DBGMSK_PRX_G3"[cm3623][als] light_poll_work leaves!\n");
        g_polling_count = 0;
        return;
    }

    if(g_polling_count == 10) 
    {
        g_polling_count = 0;

        if(delayed_work_pending(&g_light_work)) {
            cancel_delayed_work_sync(&g_light_work);
        }
        printk(DBGMSK_PRX_G3"[cm3623][als] finish light polling thread\n");
        wake_up_interruptible(&als_polling_wq_head);
        return;
    }

    //read lux value from cm3623 or P01 (al3010)
    if(g_cm3623_als_switch_on) {
        printk(DBGMSK_PRX_G3"[cm3623][als] light_poll_work, read from cm3623\n");
        ret = get_adc_calibrated_lux_from_cm3623();
    }
    else {
        printk(DBGMSK_PRX_G3"[cm3623][als] light_poll_work, read from al3010\n");
        if(g_bIsP01Attached && g_al3010_switch_on) {
            g_cm3623_light = get_calibrated_lux_value_from_P01();
                if (g_cm3623_light == 0 || g_cm3623_light > 10000) {
                    err_count ++;
                    printk("[P02][ALS] light sensor not connected I2C, count=%d\n", err_count);
                    if (err_count >= 10) 
                        return;
                }
                if (err_count != 0 && g_cm3623_light > 0 && g_cm3623_light < 10000) 
                    err_count = 0; 

            printk(DBGMSK_PRX_G3"[cm3623][als] light_poll_work, al3010 lux=%d\n", g_cm3623_light);
        }
        else {
            printk(DBGMSK_PRX_G0"[cm3623][als] light_poll_work: P01 status mismatch\n");
            //[SCR]Fix bug in Dev project that system can't go into suspend
            return;
        }
    }

#ifndef INTERNAL_TEST

    if(g_polling_count == 7) {
        g_last_cm3623_light = g_cm3623_light;
        als_lux_report_event(g_cm3623_light);
    }

    // only report light data when data changed
    if(g_cm3623_light != g_last_cm3623_light) {
#else
    // report light data periodically
    if(1) {
#endif
        g_last_cm3623_light = g_cm3623_light;

        if(g_polling_count > 7) {
            als_lux_report_event(g_cm3623_light);
        }
    }

#ifdef INTERNAL_TEST
    if(g_proxm_switch_on==1) {
        cm3623_read_reg(cm3623_client, CM3623_PS_DATA_REG, &pData);
        input_report_abs(g_cm3623_data_ps->input_dev, ABS_DISTANCE, pData);
        input_event(g_cm3623_data_ps->input_dev, EV_SYN, SYN_REPORT, 1);
        input_sync(g_cm3623_data_ps->input_dev);
    }
    if(g_proxm_dbg) {
        printk(DBGMSK_PRX_G3"[cm3623][als] pData=%d\n", pData);
    }
#endif

    ret = queue_delayed_work(cm3623light_workqueue, &g_light_work, (HZ*100/1000)) ; //100ms.
    if(ret==0) {
        printk(DBGMSK_PRX_G0"[cm3623][als] queue_delayed_work failed!\n");
    }
    g_polling_count++;
}

void als_lux_report_event(int lux)
{       
        printk(DBGMSK_PRX_G2"[cm3623][als] ** report lux = %d\n",lux);

#ifndef INTERNAL_TEST
        input_report_abs(g_cm3623_data_as->input_dev, ABS_MISC, lux);
#else
      //  lux = ret;  //ret value is not 0 only under internal test 
        input_report_abs(g_cm3623_data_as->input_dev, ABS_MISC, get_adc_calibrated_lux_from_cm3623);
#endif
        input_event(g_cm3623_data_as->input_dev, EV_SYN, SYN_REPORT, 1);
        input_sync(g_cm3623_data_as->input_dev);

#ifndef INPUT_EVENT_MODE
        atomic_inc(&ambient_update);
        if(g_ambient_dbg==1)
        printk(DBGMSK_PRX_G3"[cm3623][als] ambient_poll_work fire(%d)\n",atomic_read(&ambient_update));
        wake_up_interruptible(&ambient_wq_head);
#endif
}    
EXPORT_SYMBOL(als_lux_report_event);

int proximity_als_turn_on(int bOn)
{ 
    struct i2c_msg msg[1];
    unsigned char idata[2] = {0,0};
    int err=1;
    unsigned char data[2] = {0,0};
    int status = 0; // 0x1: light sensor is ok, 0x2: proximity is ok, 0x3: both are ok, 0x0: both failed

    printk(DBGMSK_PRX_G2"[cm3623]proximity_light_turn_on:%d\n", bOn);

    // address 0x90
    msg->addr = cm3623_client->addr;
    msg->flags = 0; //0 - write.
    msg->len = 1;
    msg->buf = data;

    data[0] = ( (als_IT<<2) | (als_GAIN<<6) | data[0]);
    if(!bOn) {
        data[0] |= als_ps_SD;
    }

    err = i2c_transfer(cm3623_client->adapter, msg, 1);

    if(err!=1) {
        printk(DBGMSK_PRX_G0"[cm3623]turn on/off: addr=0x%x val=0x%x err=%d\n", msg->addr, data[0], err);
    }
    else {
        printk(DBGMSK_PRX_G2"[cm3623]turn on/off: addr=0x%x val=0x%x \n", msg->addr, data[0]);
        status |= 0x1;
    }

    //this function is used for atd or internal test, do not access the global var.
    //g_HAL_als_switch_on = 1;

    msg->addr = CM3623_PS_CTL_REG >> 1;
    msg->flags = 0; //0 - write.
    msg->len = 1;
    msg->buf = idata;
    idata[0] = (INIT_PS & 0xfe); //bit 0 is SD_PS bit.

    if(!bOn) {
        idata[0] |= als_ps_SD;
    }

    err = i2c_transfer(cm3623_client->adapter, msg, 1);
    if(err!=1) {
        printk(DBGMSK_PRX_G0"[cm3623]turn on/off: addr=0x%x val=0x%x err=%d\n", (CM3623_PS_CTL_REG >> 1), idata[0], err);
    }
    else {
        printk(DBGMSK_PRX_G2"[cm3623]turn on/off: addr=0x%x val=0x%x \n",(CM3623_PS_CTL_REG >> 1), idata[0]);
        status |= 0x2;
    }

    //this function is used for atd or internal test, do not access the global var.
    //g_proxm_switch_on = 1;

    printk(DBGMSK_PRX_G2"[cm3623]turn on/off, status:0x%x (bitwise)\n", status);

    return status;
}

// error handle ++
static void light_proxim_error_reset(int handle, bool enable)   //0:light sensor; 1:proximity sensor
{
    int indx, ret;
    char *sensor[2] = {"light", "proximity"};

    jtimes1 = jiffies;
    printk("[cm3623] jtimes1= (%lu), jtimes0= (%lu)\n",jtimes1, jtimes0);

    if ( time_after(jtimes1, jtimes0 + 3*HZ) )
    {
        for (indx = 0; indx < 3; indx++) {
           ret = cm3623_reset();

           if (ret == 1) {            //reset successful
                printk("[cm3623] cm3623 reset successful, error handle(%d)\n",handle);
                switch (handle) 
                {
                     case LIGHT:
                                cm3623_turn_onoff_als(enable);
                                break;
        
                     case PROXIMITY:
                                cm3623_turn_onoff_proxm(enable);  
                                break;
                     default:
                                printk("[cm3623] put default.\n");
                                break;
                }

                jtimes0 = jiffies; 
                return;
           }
           else
                printk("[cm3623] (%s) retry(%d)\n",sensor[handle],indx);
        }
        if (indx == 3)
                printk("[cm3623] (%s) sensor reset fail\n",sensor[handle]);
    }

    else {
        printk("[cm3623]+++++ (%d)ms\n", jiffies_to_msecs(jtimes1 - jtimes0));
    }

    jtimes0 = jiffies;
    return;
}
// error handle --

/**
 * cm3623_read_reg - read data from cm3623 (g_data[] stores the data)
 * @i2c_client: context of i2c client of cm3623
 * @reg: the target register
 *
 * Returns negative errno, else the number of messages executed.
 */
#ifdef ALS_WORD_MODE
static int cm3623_read_reg(struct i2c_client* client, u8 reg, u16 *data)
#else
static int cm3623_read_reg(struct i2c_client* client, u8 reg, u8 *data)
#endif
{
    int err=1;
    struct i2c_msg msg[1];

    if (!client->adapter) {
        return -ENODEV;
    }

    memset(&g_data,0,sizeof(g_data));

    msg->addr = reg >> 1;
    msg->flags = I2C_M_RD;
    msg->len = 1;
    msg->buf = g_data;

    err = i2c_transfer(client->adapter, msg, 1);
    if (err==1) {
        data[0] = g_data[0];
        //g_data[1] = g_data[0];
        //printk(DBGMSK_PRO_G0 "[cm3623]: cm3623_read_reg data[0-1] 0x%x 0x%x\n", g_data[0], g_data[1]);
    }
    else {
        printk(DBGMSK_PRX_G0"[cm3623] cm3623_read_reg err %d\n", err);
    }

    return err; // return 1 is expected.
}
/**
 * cm3623_write_reg - write an I2C message to cm3623
 * @i2c_client: context of i2c client of cm3623
 * @reg: the target register
 * @val: the value will be wrote
 *
 * Returns negative errno, else the number of messages executed.
 */
/*
static int cm3623_write_reg(struct i2c_client* client, u8 reg, u8 val)
{
    int err=1;

    struct i2c_msg msg[1];
    unsigned char data[2] = {0,0};


    if (!client->adapter) {
        return -ENODEV;
    }

    msg->addr = client->addr + reg;
    msg->flags = 0; //0 - write.
    msg->len = 1;
    msg->buf = data;
    
    data[0] = val;
    //data[0] = reg;
    //data[1] = val;

    err = i2c_transfer(client->adapter, msg, 1);

    if(err!=1) {
        printk("[cm3623]: cm3623_write_reg addr=0x%x reg=0x%x val=0x%x err=%d\n", msg->addr, reg, data[0], err);
    }
    
    return err; // return 1 is expected.
}
*/


/**
 * cm3623_reset - reset cm3623
 *
 * Returns negative errno, else the number of messages executed.
 */
static int cm3623_reset(void)
{
    int err=1;
    struct i2c_msg msg[1];
    unsigned char data[2] = {0,0};
    u16 buff;
    u8 value = 1;

    printk(DBGMSK_PRX_G2"[cm3623] cm3623_reset ++\n");
    //printk("[cm3623]: proxm_int_pin=%d \n",gpio_get_value(g_proxm_int_pin));

    if (!cm3623_client->adapter) {
        return -ENODEV;
    }

//VENDOR's NOTE:
// This is used to clear internal interrupt logic and interrupt registers for cm3623.
// If the outstanding interrupt exists inside cm3623, cm3623 will not process the further I2C transcations.
// To avoid the unknown status, it is better to send special cm3623 ARA register cmd.
// However, if INT pin is already in high (when power on), we could skip this step.
//
// If there is no outstanding interrupt, cm3623 will not ACK our I2C controller inside Marvell pxa910.
// This will make Marvell I2C controller to retry 6 times (because of losing ACK from I2C client).
// To prevent this behavior, we DON'T send cm3623 ARA register cmd during initialization.
#if 0
    // cmd 0x18
    msg->addr = CM3623_ARA_REG >> 1;
    msg->flags = I2C_M_RD; //0 - write.
    msg->len = 1;
    msg->buf = data;
    data[0] = 0;
    err = i2c_transfer(cm3623_client->adapter, msg, 1);

    if(err!=1) {
        printk(DBGMSK_PRX_G2"[cm3623]: cm3623_reset addr=0x%x data[0]=0x%x err=%d\n", msg->addr, data[0], err);
    }

    data[0] = 0;
    err = i2c_transfer(cm3623_client->adapter, msg, 1);

    if(err!=1) {
        printk(DBGMSK_PRX_G2"[cm3623]: cm3623_reset addr=0x%x val=0x%x err=%d\n", msg->addr, data[0], err);
    }
#endif

    // check INT pin status
    value = g_cm3623_device.read_int_pin_state();

    if (value == 0)
    {
        err = cm3623_read_reg(cm3623_client, CHK_INT_ADD, &buff); 
    }

    // addr 0x92
    msg->addr = cm3623_client->addr + 1;    
    msg->flags = 0; //0 - write.
    msg->len = 1;
    msg->buf = data;
    data[0] = NORMAL_MODE; //0x20, PS logic High/Low mode.
    err = i2c_transfer(cm3623_client->adapter, msg, 1);

    if(err!=1) {
        printk(DBGMSK_PRX_G0"[cm3623] cm3623_reset Addr=0x%x val=0x%x err=%d\n", msg->addr, data[0], err);
    }
    else {
        printk(DBGMSK_PRX_G2"[cm3623] cm3623_reset Addr=0x%x val=0x%x \n", msg->addr, data[0]);
    }

    // addr 0x90
    msg->addr = cm3623_client->addr;
    msg->flags = 0; //0 - write.
    data[0] = (u8) g_switch[1];
    data[0] = ~(data[0]);
    data[0] = (data[0] & 0x1);
    data[0] = (INIT_ALS | data[0]);   //data[0]==1
#ifndef ALS_WORD_MODE
//  data[0] = (ALS_RESOLUTION_11 | data[0]);
    data[0] = ( (als_IT<<2) | (als_GAIN<<6) | data[0]);
#else
    data[0] = ( (als_IT<<2) | data[0]);
    data[0] |= (0x2); //WDM=1   //data[0]==0x0f
#endif

    err = i2c_transfer(cm3623_client->adapter, msg, 1);

    if(err!=1) {
        printk(DBGMSK_PRX_G0"[cm3623] cm3623_reset addr=0x%x val=0x%x err=%d\n", msg->addr, data[0], err);
    }
    else {
        printk(DBGMSK_PRX_G2"[cm3623] cm3623_reset addr=0x%x val=0x%x \n", msg->addr, data[0]);
    }

    // addr 0xF2
    msg->addr = CM3623_PS_THD_REG >> 1;
    msg->flags = 0; //0 - write.
    data[0] = g_ps_threshold; //0x05, PS threshould range is 3~255.

    err = i2c_transfer(cm3623_client->adapter, msg, 1);

    if(err!=1) {
        printk(DBGMSK_PRX_G0"[cm3623] cm3623_reset addr=0x%x val=0x%x err=%d\n", msg->addr, data[0], err);
    }
    else {
        printk(DBGMSK_PRX_G2"[cm3623] cm3623_reset addr=0x%x val=0x%x \n",(CM3623_PS_THD_REG >> 1), data[0]);
    }

    // addr 0xF0
    msg->addr = CM3623_PS_CTL_REG >> 1;
    msg->flags = 0; //0 - write.
    data[0] = (u8) g_switch[0];
    data[0] = ~(data[0]);
    data[0] = (data[0] & 0x1);
    data[0] = (INIT_PS | data[0]);
    data[0] |= (ps_IT<<4);      //data[0] = 0x31

    err = i2c_transfer(cm3623_client->adapter, msg, 1);

    if(err!=1) {
        printk(DBGMSK_PRX_G0"[cm3623] cm3623_reset addr=0x%x val=0x%x err=%d\n", msg->addr, data[0], err);
    }
    else {
        printk(DBGMSK_PRX_G2"[cm3623] cm3623_reset addr=0x%x val=0x%x \n",(CM3623_PS_CTL_REG >> 1), data[0]);
    }

//jonathan    printk("[cm3623]: proxm_int_pin=%d \n",gpio_get_value(g_proxm_int_pin));
    printk(DBGMSK_PRX_G2"[cm3623] cm3623_reset --\n");
    return err; // return 1 is expected.
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void cm3623_early_suspend(struct early_suspend *handler)
{
    //int ret = 0;
    int err=1;
    struct i2c_msg msg[1];
    unsigned char data[2] = {0,0};

    printk(DBGMSK_PRX_G2"[cm3623] ++cm3623_early_suspend, psensor:%d, als:%d\n", g_proxm_switch_on, g_cm3623_als_switch_on);

    g_cm3623_switch_earlysuspend = 1;

    enable_irq_wake(g_cm3623_device.irq);

    if(g_proxm_switch_on==0) {
        if(g_cm3623_als_switch_on==1) {
            //In case upper layer doesn't switch off ambient before early_suspend.
            g_cm3623_als_switch_on = 0;
            g_ambient_suspended = 1;
            g_polling_count = 0;

            printk(DBGMSK_PRX_G2"[cm3623] cm3623_early_suspend, turn off ambient\n");

            // cmd 0x20 or 0x90
            msg->addr = cm3623_client->addr;
            msg->flags = 0; //0 - write.
            msg->len = 1;
            msg->buf = data;
            data[0] = (INIT_ALS | 0x1); //shutdown ALS.

            err = i2c_transfer(cm3623_client->adapter, msg, 1);

            if(err!=1) {
                printk(DBGMSK_PRX_G0"[cm3623]: addr=0x%x val=0x%x err=%d\n", msg->addr, data[0], err);
            }       
            else {
                printk(DBGMSK_PRX_G2"[cm3623]: addr=0x%x val=0x%x \n", msg->addr, data[0]);
            }
     
            //turn off als interrupt
            msg->addr = CM3623_PS_CTL_REG >> 1;
            data[0] |= als_ps_SD;
            err = i2c_transfer(cm3623_client->adapter, msg, 1);
                
            if(err!=1) {
                printk(DBGMSK_PRX_G0"[cm3623]: addr=0x%x val=0x%x err=%d\n", msg->addr, data[0], err);
            }       
            else {
                printk(DBGMSK_PRX_G2"[cm3623]: addr=0x%x val=0x%x \n", CM3623_PS_CTL_REG >> 1, data[0]);
            }
        }
    }

    printk(DBGMSK_PRX_G2"[cm3623] --cm3623_early_suspend\n");
}


static void cm3623_late_resume(struct early_suspend *handler)
{
    //int ret = 0;
    int err=1;
    struct i2c_msg msg[1];
    unsigned char data[2] = {0,0};

    //printk(DBGMSK_PRO_G3 "[cm3623]: cm3623_late_resume + (GPIO%d=%d)\n", g_proxm_int_pin, gpio_get_value(g_proxm_int_pin));
    //printk("[cm3623]: GPIO0 , mfpr=0x%x , GPIOPLR0=0x%x , GPIOPDR0=0x%x \n", __raw_readl(0xFE01E0DC), __raw_readl(0xFE019000+0x0000), __raw_readl(0xFE019000+0x000C));

    printk(DBGMSK_PRX_G2"[cm3623] ++cm3623_late_resume, psensor:%d, als:%d\n", g_proxm_switch_on, g_cm3623_als_switch_on);

    if(g_ambient_suspended==1) {
        // cmd 0x20 or 0x90
     /*   msg->addr = cm3623_client->addr;
        msg->flags = 0; //0 - write.
        msg->len = 1;
        msg->buf = data;
        #ifndef ALS_WORD_MODE
        //  data[0] = (ALS_RESOLUTION_11 | data[0]);
            data[0] = ( (als_IT<<2) | (als_GAIN<<6) | data[0]);
        #else
            data[0] = ( (als_IT<<2) | data[0]);
            data[0] |= (0x2); //WDM=1.
        #endif

        err = i2c_transfer(cm3623_client->adapter, msg, 1);

        if(err!=1) {
            printk(DBGMSK_PRX_G0"[cm3623]: addr=0x%x val=0x%x err=%d\n", msg->addr, data[0], err);
        }
        else {
            printk(DBGMSK_PRX_G2"[cm3623]: addr=0x%x val=0x%x \n", msg->addr, data[0]);
        }
*/
        //initialization
        msg->addr = CM3623_INIT_CMD>>1;    // 0x92
        msg->flags = 0;                     
        msg->len = 1;
        msg->buf = data;
        data[0] = INT_MODE;
        err = i2c_transfer(cm3623_client->adapter, msg, 1); 
        if (err == 1)
              printk(DBGMSK_PRX_G2"[cm3623[als] setting interrupt initialization\n");

        //set initial THD
        msg->addr = CM3623_ALS_CTL_REG>>1;    // address 0x90
#ifndef ALS_WORD_MODE
        //  data[0] = (ALS_RESOLUTION_11 | data[0]);
        data[0] = ( (als_IT<<2) | (als_GAIN<<6) | data[0]);
#else
        data[0] = (0xff & 0);
        data[0] = ( GAIN_ALS_DIV8 | (0x00<<4) | (als_IT<<2) | data[0]);  //set +/-23 lux
        data[0] |= (0x2); //WDM=1.
#endif
        err = i2c_transfer(cm3623_client->adapter, msg, 1);
        if (err == 1)
              printk(DBGMSK_PRX_G2"[cm3623[als] setting interrupt initial threshold +/- 23 lux\n");

        //enable ALS INT
        msg->addr = CM3623_PS_CTL_REG>>1;       //0xF0

        if (g_proxm_switch_on == 1)
                data[0] = ( INIT_PS | (ps_IT<<4) | (0x3<<2) ) ;
        else
                data[0] = ( INIT_PS | (ps_IT<<4) | (0x1<<3) );
        err = i2c_transfer(cm3623_client->adapter, msg, 1);

        if(err!=1) {
              printk(DBGMSK_PRX_G2"[cm3623][als] addr=0x%x val=0x%x err=%d\n", msg->addr, data[0], err);
        }
        else 
              printk(DBGMSK_PRX_G2"[cm3623][als] addr=0x%x val=0x%x \n", msg->addr, data[0]);

        g_ambient_suspended = 0;
        g_cm3623_als_switch_on = 1; //this flag is usually changed in put_property.

        printk(DBGMSK_PRX_G2"[cm3623][als] late_resume: apply ALS interrupt mode\n");

        schedule_delayed_work(&g_light_work, 10);
    }
    else if(g_cm3623_switch_earlysuspend==1) {
        ;
    }

    disable_irq_wake(g_cm3623_device.irq);
    g_cm3623_switch_earlysuspend=0;

    printk(DBGMSK_PRX_G2"[cm3623]--cm3623_late_resume\n");
}


static struct early_suspend cm3623_early_suspend_desc = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = cm3623_early_suspend,
    .resume = cm3623_late_resume,
};
#endif


const struct i2c_device_id cm3623_id[] = {
    {"cm3623", 0},
    {}
};

MODULE_DEVICE_TABLE(i2c, cm3623_id);

static struct i2c_driver cm3623_driver = {
    .driver = {
        .name = "cm3623",
        .owner  = THIS_MODULE,
     },
    .probe = cm3623_probe,
    .remove = cm3623_remove,
    .suspend = cm3623_suspend,
    .resume = cm3623_resume,
    .id_table = cm3623_id,
};




static int init_cm3623(void)
{
    int ret;
    //u8 regaddr, value;
    //const char* label_proxm_int = "cm3623_int";
    //const char* label_proxm_en = "cm3623_en";

    printk(DBGMSK_PRX_G2"[cm3623]: init_cm3623 +.\n");

    jtimes0 = jiffies;

    g_proxm_switch_on = 0;
    g_HAL_als_switch_on = 0;
    g_cm3623_als_switch_on = 0;

    //WARNING!
    //If I2C bus is turnned on and CM3623 is off,
    //I2C 1.8v will sink current to CM3623 and cause current leakage!
    //I shall keep CM3623 power on and control SD_PS and SD_ALS bit.
    //gpio_set_value(g_proxm_pwr_pin, 1);
    //gpio_free(g_proxm_pwr_pin);
    g_switch[0] = 0; //disable PS as default!
    g_switch[1] = 0; //disable ALS as default!
    ret = cm3623_reset(); // turn on both sensor (test only)
//jonathan    printk("[cm3623]: proximity=%d \n",gpio_get_value(g_proxm_int_pin));

    //for test purpose only
    //proximity_als_turn_on(1);

    // polling thread for light sensor
    cm3623light_workqueue = create_singlethread_workqueue("cm3623lightwq");

    INIT_DELAYED_WORK(&g_light_work, light_poll_work);

    INIT_WORK(&cm3623_attached_P02_work, cm3623_lightsensor_attached_pad_P01);
    INIT_WORK(&cm3623_ISR_work, cm3623_interrupt_handler);
    INIT_WORK(&cm3623_light_interrupt_work, light_interrupt_work);
    INIT_WORK(&cm3623_proximity_interrupt_work, proximity_interrupt_work);

    wake_lock_init(&proximity_wake_lock, WAKE_LOCK_SUSPEND, "proxm_wake_lock");

#ifndef INTERNAL_TEST // register irq handler after sensor power on in order to avoid unexpected irq error!

    if( g_cm3623_device.irq <= 0 ) {
        printk(DBGMSK_PRX_G0"[cm3623] gpio_to_irq fail (g_cm3623_device.irq).\n");
    }
    else {
        printk(DBGMSK_PRX_G2"[cm3623] (g_cm3623_device.irq) irq=%d.\n", g_cm3623_device.irq);

        ret = request_irq(  g_cm3623_device.irq,
                            als_proxm_interrupt_handler,
                            IRQF_TRIGGER_FALLING,
                            "cm3623_INT",
                            &cm3623_client->dev );
        if (ret < 0) {
            printk(DBGMSK_PRX_G0"[cm3623] (g_cm3623_device.irq) request_irq() error %d.\n",ret);
        }
        else {
            printk(DBGMSK_PRX_G2"[cm3623] (g_cm3623_device.irq) request_irq ok.\n");
        }
    }
#endif

    //init_waitqueue_head(&proxm_isr_wq);

    g_cm3623_data_as->polling = 0;
    g_cm3623_data_as->poll_interval_ms = 100;
    g_cm3623_data_as->event_threshold = 1000;

    g_cm3623_data_ps->polling = 0;
    g_cm3623_data_ps->poll_interval_ms = 100;
    g_cm3623_data_ps->event_threshold = 1000;

    printk(DBGMSK_PRX_G2"[cm3623] init_cm3623 -.\n");
    return 1;
}


static int cm3623_input_init(void)
{
    int ret = 0;
    struct input_dev *input_dev_as = NULL;
    struct input_dev *input_dev_ps = NULL;

    input_dev_as = input_allocate_device();
    input_dev_ps = input_allocate_device();
    if (!input_dev_as || !input_dev_ps) {
        ret = -ENOMEM;
        printk(DBGMSK_PRX_G0"[cm3623]: Failed to allocate input_data device\n");
        goto error_1;
    }

    input_dev_as->name = "cm3623_als";
    input_dev_as->id.bustype = BUS_I2C;
    input_set_capability(input_dev_as, EV_ABS, ABS_MISC);
    __set_bit(EV_ABS, input_dev_as->evbit);
    __set_bit(ABS_MISC, input_dev_as->absbit);
    input_set_abs_params(input_dev_as, ABS_MISC, 0, 1048576, 0, 0);
    input_set_drvdata(input_dev_as, g_cm3623_data_as);

    input_dev_ps->name = "cm3623_ps";
    input_dev_ps->id.bustype = BUS_I2C;
    input_set_capability(input_dev_ps, EV_ABS, ABS_DISTANCE);
    __set_bit(EV_ABS, input_dev_ps->evbit);
    __set_bit(ABS_DISTANCE, input_dev_ps->absbit);
    input_set_abs_params(input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);
    input_set_drvdata(input_dev_ps, g_cm3623_data_ps);

    ret = input_register_device(input_dev_as);
    if (ret < 0) {
        input_free_device(input_dev_as);
        goto error_1;
    }
    g_cm3623_data_as->input_dev = input_dev_as;

    ret = input_register_device(input_dev_ps);
    if (ret < 0) {
        input_free_device(input_dev_ps);
        goto error_1;
    }
    g_cm3623_data_ps->input_dev = input_dev_ps;

    this_input_dev_as = input_dev_as;
    this_input_dev_ps = input_dev_ps;

error_1:

    return ret;
}


static int cm3623_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    //const char* label = "cm3623";
    int ret = 0;
    int error = 0;
    struct cm3623_platform_data *pdata;

    printk(DBGMSK_PRX_G2"[cm3623] cm3623_probe +.\n");

    if (client == NULL) {
        printk(DBGMSK_PRX_G0"[cm3623] Client is NUll.\n");
        ret =  -EFAULT;
        goto cm3623_probe_err;
    }

    if (!(g_cm3623_data_as = kmalloc(sizeof(struct cm3623_data), GFP_KERNEL))) {
        ret = -ENOMEM;
        goto cm3623_probe_err;
    }
    memset(g_cm3623_data_as, 0, sizeof(struct cm3623_data));

    if (!(g_cm3623_data_ps = kmalloc(sizeof(struct cm3623_data), GFP_KERNEL))) {
        ret = -ENOMEM;
        goto cm3623_probe_err;
    }
    memset(g_cm3623_data_ps, 0, sizeof(struct cm3623_data));

    cm3623_client = client;
    i2c_set_clientdata(cm3623_client, g_cm3623_data_as);
    i2c_set_clientdata(cm3623_client, g_cm3623_data_ps);
    cm3623_client->driver = &cm3623_driver;
    cm3623_client->flags = 1;
    strlcpy(cm3623_client->name, CM3623_DRV_NAME, I2C_NAME_SIZE);

    pdata = client->dev.platform_data;
    if (pdata == NULL) {
        dev_err(&client->dev, "platform data is required!\n");
        error = -EINVAL;
        goto cm3623_probe_err;
    }


    /* Get data that is defined in board specific code. */
    g_cm3623_device.irq = client->irq;
    g_cm3623_device.read_int_pin_state = pdata->read_int_pin_state;

    g_cm3623_device.init_hw = pdata->init_platform_hw;
    if(g_cm3623_device.init_hw) {
        printk(DBGMSK_PRX_G2"[cm3623]calling init_platform_hw\n");
        error = g_cm3623_device.init_hw(client);
        if (error) {
            dev_err(&client->dev, "hw init failed");
            goto cm3623_probe_err;
        }
    }

    printk(DBGMSK_PRX_G2"[cm3623] Register input device...\n");
    if( cm3623_input_init() != 0 ) {
        goto cm3623_probe_err;
    }

    ret = init_cm3623();
    if( ret <= 0 )  {
        printk(DBGMSK_PRX_G0"[cm3623] init_cm3623() error.\n");
        goto cm3623_probe_err;
    }

    ret = proximity_dev_register(&g_proxmDev);
    if (ret) {
        printk(DBGMSK_PRX_G0"[cm3623] proxmdl create sysfile fail.\n");
    }

    ret = proximity_dev_register(&g_ambientDev);
    if (ret) {
        printk(DBGMSK_PRX_G0"[cm3623] ambientdl create sysfile fail.\n");
    }


#ifdef CONFIG_HAS_EARLYSUSPEND
    register_early_suspend( &cm3623_early_suspend_desc );
#endif

    schedule_delayed_work(&g_light_work, 10);

    printk(DBGMSK_PRX_G2"[cm3623] cm3623_probe -.\n");

#ifdef CONFIG_I2C_STRESS_TEST

    printk("LSenor add test case+\n");

    i2c_add_test_case(client, "LightSensorTest",ARRAY_AND_SIZE(gLSensorTestCaseInfo));

    printk("LSensor add test case-\n");

#endif

    return 0;


cm3623_probe_err:
    printk(DBGMSK_PRX_G0"[cm3623] cm3623_probe - (error).\n");
    //if (g_cm3623_data_ps != NULL) {
    //  kfree(g_cm3623_data_ps);
    //}

    return ret;

}




static int cm3623_remove(struct i2c_client *client)
{
    printk(DBGMSK_PRX_G2"[cm3623] cm3623_remove +.\n");

    /* free_irq(client->irq, NULL); */

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend( &cm3623_early_suspend_desc );
#endif

    printk(DBGMSK_PRX_G2"[cm3623] cm3623_remove -.\n");
    return 0;
}


static int cm3623_suspend(struct i2c_client *client, pm_message_t mesg)
{
    //printk("[cm3623]: cm3623_suspend. (GPIO0=%d)\n", gpio_get_value(g_proxm_int_pin));
    //printk("[cm3623]: GPIO0 , mfpr=0x%x , GPIOPLR0=0x%x , GPIOPDR0=0x%x \n", __raw_readl(0xFE01E0DC), __raw_readl(0xFE019000+0x0000), __raw_readl(0xFE019000+0x000C));
    return 0 ;
}


static int cm3623_resume(struct i2c_client *client)
{
//  printk("[cm3623]: cm3623_resume. (GPIO0=%d)\n", gpio_get_value(g_proxm_int_pin));
//  printk("[cm3623]: GPIO0 , mfpr=0x%x , GPIOPLR0=0x%x , GPIOPDR0=0x%x \n", __raw_readl(0xFE01E0DC), __raw_readl(0xFE019000+0x0000), __raw_readl(0xFE019000+0x000C));

    return 0 ;
}

//Disable P01 attached temporarily for 1st ICS check-in
static int cm3623_lightsensor_p01_mp_event(struct notifier_block *this, unsigned long event, void *ptr);


static struct notifier_block cm3623_lightsensor_p01_mp_notifier = {
        .notifier_call = cm3623_lightsensor_p01_mp_event,
        .priority = CM3623_LIGHTSENSOR_MP_NOTIFY,
};

static int __init cm3623_init(void)
{
        int ret = 0;

    printk(DBGMSK_PRX_G2"[cm3623] cm3623_init +.\n");

//    g_proxm_int_pin = TEGRA_GPIO_PV0;

    ret = i2c_add_driver(&cm3623_driver);
    if (ret) {
        printk(DBGMSK_PRX_G0"[cm3623] i2c_add_driver fail.\n");
    }
    else {
        printk(DBGMSK_PRX_G2"[cm3623] i2c_add_driver.\n");
    }

	//Disable P01 attached temporarily for 1st ICS check-in
    register_microp_notifier(&cm3623_lightsensor_p01_mp_notifier);

    printk(DBGMSK_PRX_G2"[cm3623] cm3623_init -.\n");
    return ret;
}


static void __exit cm3623_exit(void)
{
    printk(DBGMSK_PRX_G2"[cm3623] cm3623_exit +.\n");

    i2c_del_driver(&cm3623_driver);
    proximity_dev_unregister(&g_proxmDev);
    proximity_dev_unregister(&g_ambientDev);
    destroy_workqueue(cm3623light_workqueue);

//     //tasklet_kill(&cm3623_ISR_work);

    printk(DBGMSK_PRX_G2"[cm3623] cm3623_exit -.\n");
}


module_init(cm3623_init);
module_exit(cm3623_exit);

// Disable P01 attached temporarily for 1st ICS check-in
static void cm3623_lightsensor_attached_pad_P01(struct work_struct *work)
{
    printk("[cm3623_als] lightsensor_attached_pad_P01()++\n");

     //if HAL already turned on als, we switch to P01 al3010
    if(g_proxm_switch_on)
        cm3623_turn_onoff_proxm(0);

    if(g_HAL_als_switch_on) 
    {
        //try to turn on al3010
        msleep(600);
        printk("[cm3623_als] lightsensor_attached_pad_P01, checking if P01 is attached\n");

        if(g_bIsP01Attached) {
            printk("[cm3623_als] lightsensor_attached_pad_P01, attached! switch to al3010\n");
            //shut down cm3623_als
            cm3623_turn_onoff_als(0);

            //make sure g_cm3623_als_switch_on is turn off
            queue_delayed_work(cm3623light_workqueue, &g_light_work, msecs_to_jiffies(10));
        }
        else
            printk("[cm3623_als] al3010_attached_P02 fail\n");
    }
    printk("[cm3623_als] lightsensor_attached_pad_P01()--\n");

    return;
}

int cm3623_lightsensor_detached_pad_P01(void)
{
    printk("[cm3623_als] lightsensor_detached_pad_P01()++\n");

    //if HAL still turned on the als, we switch back to cm3623
    if(g_HAL_als_switch_on) {
        printk("[cm3623_als] lightsensor_detached_pad_P01(), switch back to cm3623\n");
        //turn on cm3623_als
        cm3623_turn_onoff_als(1);
        als_lux_report_event(g_cm3623_light);
    }
    else {
        printk("[cm3623_als] lightsensor_detached_pad_P01(), als is turned off\n");
    }

    printk("[cm3623_als] lightsensor_detached_pad_P01()--\n");

    if (g_proxm_switch_on)  {
        printk("[cm3623_proxm] detached_P01, phone call still on\n");
        cm3623_turn_onoff_proxm(1);
    }

    return 0;
}


static int cm3623_lightsensor_p01_mp_event(struct notifier_block *this, unsigned long event, void *ptr)
{

    switch (event) {
        case P01_ADD:
            printk("[cm3623_als][MicroP] P01_ADD \r\n");
            //cm3623_lightsensor_attached_pad_P01();
            queue_work(cm3623light_workqueue, &cm3623_attached_P02_work);
            return NOTIFY_DONE;

        case P01_REMOVE:
            printk("[cm3623_als][MicroP] P01_REMOVE \r\n");
            cm3623_lightsensor_detached_pad_P01();
            return NOTIFY_DONE;

        default:
            return NOTIFY_DONE;
    }
}


MODULE_AUTHOR("ASUS");
MODULE_DESCRIPTION("CAPELLA CM3623 proximity with ALS");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

