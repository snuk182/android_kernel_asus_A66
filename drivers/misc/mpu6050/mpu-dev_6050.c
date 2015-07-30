/*
	$License:
	Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	$
 */
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/signal.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <linux/poll.h>

#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#include "mpuirq.h"
#include "slaveirq.h"
#include "mlsl.h"
#include "mldl_cfg.h"
#include <linux/mpu_6050.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#define AMI_INTER_OFFSET_PAD "7 -24 -17 -34 -59 12\n"     //AICHI-JP Add for ASUS PadFone
#define AMI_INTER_OFFSET_PHONE "0 0 0 0 0 0\n"

#define AMI_DEFAULT_CALI_FILE "921368\n  1024   1024   1024\n -2048  -2048  -2048\n   600    600    600\n     0      0      0\n     0      0      0\n     0      0      0\n   100    100    100\n415\n0 0 0 0 0 0"
#define AMI_PAD_DEFAULT_CALI_FILE "921368\n  1024   1024   1024\n -2048  -2048  -2048\n   600    600    600\n     0      0      0\n     0      0      0\n     0      0      0\n   100    100    100\n415\n7 -24 -17 -34 -59 12"
#define AMI_CALIBRATION_FILE "/data/amit/AMI304_Config.ini"
#define ASUS_PAD_CALIBRATION_FILE "/data/amit/AMI304_Config_PAD.ini"
#define ASUS_PHONE_CALIBRATION_FILE "/data/amit/AMI304_Config_PHONE.ini"
#define ASUS_COMPASS_CALIBRATION_FILE "/data/asusdata/AMI304_Config.ini"
//ASUS_BSP +++ Jason Chang "dynamic change orientation"
struct mldl_cfg *asus_g_mldl_cfg_6050;
static int g_orientation_flag = 0; //0-smartphone; 1-pad
static signed char *Pad_Acc_Orient;
static signed char *Pad_Gyro_Orient;
static signed char *Pad_Mag_Orient;
static signed char *Phone_Acc_Orient;
static signed char *Phone_Gyro_Orient;
static signed char *Phone_Mag_Orient;

//EVB
static signed char Pad_accel_orientation [9] = { 0, -1, 0, 1, 0, 0, 0, 0, 1 };
static signed char Phone_accel_orientation [9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

static signed char Pad_gyro_orientation [9] = { 0, -1, 0, 1, 0, 0, 0, 0, 1 };
static signed char Phone_gyro_orientation [9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

static signed char Pad_mag_orientation [9] = { -1, 0, 0, 0, -1, 0, 0, 0, 1 };
static signed char Phone_mag_orientation [9] = { 0, -1, 0, 1, 0, 0, 0, 0, 1 };

//SR1
static signed char Pad_accel_orientation_sr1 [9] = { 0, 1, 0, -1, 0, 0, 0, 0, 1 };
static signed char Phone_accel_orientation_sr1 [9] = { -1, 0, 0, 0, -1, 0, 0, 0, 1 };

static signed char Pad_gyro_orientation_sr1 [9] = { 0, -1, 0, -1, 0, 0, 0, 0, -1 };
static signed char Phone_gyro_orientation_sr1 [9] = { -1, 0, 0, 0, 1, 0, 0, 0, -1 };

static signed char Pad_mag_orientation_sr1 [9] = { 0, 1, 0, -1, 0, 0, 0, 0, 1 };
static signed char Phone_mag_orientation_sr1 [9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

//A66_SR1
//A66_ER1
static signed char Pad_accel_orientation_A66sr1 [9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1};
static signed char Phone_accel_orientation_A66sr1 [9] = { 0, 1, 0, -1, 0, 0, 0, 0, 1 };

static signed char Pad_gyro_orientation_A66sr1 [9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
static signed char Phone_gyro_orientation_A66sr1 [9] = { 0, 1, 0, -1, 0, 0, 0, 0, 1 };

static signed char Pad_mag_orientation_A66sr1 [9] = { -1, 0, 0, 0, 1, 0, 0, 0, -1 };
static signed char Phone_mag_orientation_A66sr1 [9] = { 0, 1, 0, 1, 0, 0, 0, 0, -1 };
//ASUS_BSP+++ Jason Chang "change oritation for pad"
#include <linux/microp_notify.h>
//ASUS_BSP--- Jason Chang "change oritation for pad"
//ASUS_BSP --- Jason Chang "dynamic change orientation"
#include "accel/mpu6050.h"

//ASUS_BSP +++ Jason Chang "replace compass calibration by workqueue"
struct delayed_work pad_plugin_work_6050;
struct delayed_work pad_plugout_work_6050;
struct workqueue_struct *pad_plug_wq_6050 = NULL;
int g_calibration_file_flag_6050 = 0;
//ASUS_BSP --- Jason Chang "replace compass calibration by workqueue"

/* Platform data for the MPU */
struct mpu_private_data_6050 {
	struct miscdevice dev;
	struct i2c_client *client;

	/* mldl_cfg data */
	struct mldl_cfg mldl_cfg;
	struct mpu_ram		mpu_ram;
	struct mpu_gyro_cfg	mpu_gyro_cfg;
	struct mpu_offsets	mpu_offsets;
	struct mpu_chip_info	mpu_chip_info;
	struct inv_mpu_cfg	inv_mpu_cfg;
	struct inv_mpu_state	inv_mpu_state;

	struct mutex mutex;
	wait_queue_head_t mpu_event_wait;
	struct completion completion;
	struct timer_list timeout;
	struct notifier_block nb;
	struct mpuirq_data mpu_pm_event;
	int response_timeout;	/* In seconds */
	unsigned long event;
	int pid;
	struct module *slave_modules[EXT_SLAVE_NUM_TYPES];
//ASUS_BSP +++ Jason Chang "ATD Interface of 9-axis sensor"
    struct attribute_group attrs;
    int gyro_status;
    int gsensor_status;
    int accel_raw[6];
//ASUS_BSP --- Jason Chang "ATD Interface of 9-axis sensor"
};
extern int asus_get_gsensor_status(void);
//ASUS_BSP +++ Jason Chang "ATD Interface of 9-axis sensor"
static ssize_t read_gyro_status(struct device *dev, struct device_attribute *devattr, char *buf)
{	struct i2c_client *client = to_i2c_client(dev);
	struct mpu_private_data_6050 *data = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", data->gyro_status);
}

static ssize_t read_accel_status(struct device *dev, struct device_attribute *devattr, char *buf)
{	struct i2c_client *client = to_i2c_client(dev);
	struct mpu_private_data_6050 *data = i2c_get_clientdata(client);
        data->gsensor_status = asus_get_gsensor_status();
	return sprintf(buf, "%d\n", data->gsensor_status);
}

static ssize_t read_accel_raw(struct device *dev, struct device_attribute *devattr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct mpu_private_data_6050 *mpu_pdata = i2c_get_clientdata(client);
    struct mldl_cfg *mldl_cfg = &mpu_pdata->mldl_cfg;
    struct i2c_adapter *slave_adapter[EXT_SLAVE_NUM_TYPES];
    struct ext_slave_descr **slave = mldl_cfg->slave;
    struct ext_slave_platform_data **pdata_slave = mldl_cfg->pdata_slave;
    unsigned char data[6];
    short X = 0, Y = 0, Z = 0;
    int res = 0, ii;

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++) {
		if (!pdata_slave[ii])
			slave_adapter[ii] = NULL;
		else
			slave_adapter[ii] =
				i2c_get_adapter(pdata_slave[ii]->adapt_num);
	}
	slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE] = client->adapter;
    
    //res = inv_serial_read_6050(client->adapter, 0x68,0x3B, 6, data);
    res = inv_mpu_slave_read_6050(mldl_cfg, 
                                                             slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
                                                             slave_adapter[EXT_SLAVE_TYPE_ACCEL],
                                                             slave[EXT_SLAVE_TYPE_ACCEL],
                                                             pdata_slave[EXT_SLAVE_TYPE_ACCEL],
                                                             data);
    printk("MPU6050 raw data 1~6: %d, %d, %d, %d, %d, %d\n",data[0],data[1],data[2],data[3],data[4],data[5]);

    if(res)
            printk("%s: Read accel data register fail\n", __FUNCTION__);
    else{
     /*       X = ((data[1] << 4) | (data[0] >> 4));
            Y = ((data[3] << 4) | (data[2] >> 4));
            Z = ((data[5] << 4) | (data[4] >> 4));*/
    X = ((data[0]<<8) | data[1]);
    Y = ((data[2]<<8) | data[3]);
    Z = ((data[4]<<8) | data[5]);
    printk("gsensor raw: X=%d Y=%d Z=%d\n",X,Y,Z);
 /*           if (X & 0x800)
                X |= 0xFFFFF000;
            if (Y & 0x800)
                Y |= 0xFFFFF000;
            if (Z & 0x800)
                Z |= 0xFFFFF000;*/
        }            
//printk("jason: X=%u Y=%u Z=%u\n",X,Y,Z);
    return sprintf(buf, "%d %d %d\n", X, Y, Z);
}

DEVICE_ATTR(gyro_6050_status, S_IRUGO, read_gyro_status, NULL);
DEVICE_ATTR(accel_6050_raw, S_IRUGO, read_accel_raw, NULL);
DEVICE_ATTR(accel_6050_status, S_IRUGO, read_accel_status, NULL);

static struct attribute *mpu_6050_attr[] = {
	&dev_attr_gyro_6050_status.attr,
        &dev_attr_accel_6050_raw.attr,
        &dev_attr_accel_6050_status.attr,
	NULL
};
//ASUS_BSP --- Jason Chang "ATD Interface of 9-axis sensor"
struct mpu_private_data_6050 *mpu_private_data_6050;

//ASUS_BSP +++ Jason Chang "switch compass calibration data when plug in/out PAD"
int mpu_file_read_6050(char * file_name, unsigned char *buf, int read_len)
{
    int ret=0;
    struct file *fp;
    mm_segment_t oldfs;
    unsigned char local_buf[256];

    memset(local_buf,0,sizeof(local_buf));
    
    oldfs = get_fs(); 
    set_fs(KERNEL_DS);
    
    fp = filp_open(file_name, O_RDONLY, S_IROTH | S_IWOTH | S_IWGRP | S_IRGRP | S_IWUSR | S_IRUSR);
    if(!IS_ERR(fp))
    {
        fp->f_op->read(fp, local_buf, read_len, &fp->f_pos);
        printk("[mpu] read file %s to buffer\n",file_name);
        if(filp_close(fp,NULL))
            printk("[mpu] filp_close failed\n");
    }
    else
    {
        printk("[mpu] open file %s failed\n",file_name);
        ret = -1;
    }

    memcpy(buf,local_buf,sizeof(local_buf));

    set_fs(oldfs); 

    return ret;
}
int mpu_file_write_6050(char * file_name, unsigned char *buf, int len)
{
    int ret=0;
    struct file *fp;
    mm_segment_t oldfs;
    
    oldfs = get_fs(); 
    set_fs(KERNEL_DS);

    fp = filp_open(file_name, O_CREAT | O_RDWR | O_SYNC, S_IROTH | S_IWOTH | S_IWGRP | S_IRGRP | S_IWUSR | S_IRUSR);
    if(!IS_ERR(fp))
    {
        fp->f_op->write(fp, buf, len, &fp->f_pos);
        printk("[mpu] write %d byte to file %s\n", strlen(buf), file_name);
        if(filp_close(fp,NULL))
            printk("[mpu] filp_close failed\n");
    }
    else
    {
        printk("[mpu] open file %s failed\n",file_name);
        ret =  -1;
    }
    
    set_fs(oldfs); 

    return ret;
}
//ASUS_BSP +++ Jason Chang "Recover compass calibration file when file crashed"
int Create_compass_calibration_file_6050(void)
{
    int ret = 0;
    unsigned char *cali_buf;
    unsigned char *temp_buf;

    if(!(cali_buf = kmalloc(256, GFP_KERNEL)) || !(temp_buf = kmalloc(256, GFP_KERNEL)))
    {
        ret = 7;
        printk("[mpu] Allocate calibration data buffer failed\n");
    }

    if(!mpu_file_read_6050(ASUS_COMPASS_CALIBRATION_FILE,cali_buf,256) && !ret)
    {
            printk("[mpu] copy calibration from asusdata\n");

            memset(temp_buf, 0, 256);
            memcpy(temp_buf, cali_buf, 7);
            memcpy(temp_buf+7,cali_buf+8, 151);
            strcat(temp_buf, AMI_INTER_OFFSET_PHONE);
            if(0 != mpu_file_write_6050(AMI_CALIBRATION_FILE,temp_buf,strlen(temp_buf)))
            {
                ret = 2;
                printk("[mpu] Error: create %s failed\n",AMI_CALIBRATION_FILE);
            }
                        
            memset(temp_buf, 0, 256);
            memcpy(temp_buf, cali_buf, 7);
            memcpy(temp_buf+7,cali_buf+8, 151);
            strcat(temp_buf, AMI_INTER_OFFSET_PAD);
            if(0 != mpu_file_write_6050(ASUS_PAD_CALIBRATION_FILE,temp_buf,strlen(temp_buf)))
            {
                ret = 3;
                printk("[mpu] Error: create %s failed\n",ASUS_PAD_CALIBRATION_FILE);
            }
                        
            memset(temp_buf, 0, 256);
            memcpy(temp_buf, cali_buf, 7);
            memcpy(temp_buf+7,cali_buf+8, 151);
            strcat(temp_buf, AMI_INTER_OFFSET_PHONE);
            if(0 != mpu_file_write_6050(ASUS_PHONE_CALIBRATION_FILE,temp_buf,strlen(temp_buf)))
            {
                ret = 4;
                printk("[mpu] Error: create %s failed\n",ASUS_PHONE_CALIBRATION_FILE);
            }
    }
    else
    {
        //ASUS_BSP +++ Jason Chang "create default calibration file when lost calibration in asusdata"
        printk("[mpu] asusdata has no AMI304_Config.ini, create default calibration file\n");
        
        memset(temp_buf, 0, 256);
        memcpy(temp_buf, AMI_DEFAULT_CALI_FILE, strlen(AMI_DEFAULT_CALI_FILE));
        if(0 != mpu_file_write_6050(AMI_CALIBRATION_FILE,temp_buf,strlen(temp_buf)))
        {
            ret = 2;
            printk("[mpu] create default %s failed\n",AMI_CALIBRATION_FILE);
        }
        if(0 != mpu_file_write_6050(ASUS_PHONE_CALIBRATION_FILE,temp_buf,strlen(temp_buf)))
        {
            ret = 3;
            printk("[mpu] create default %s failed\n",ASUS_PHONE_CALIBRATION_FILE);
        }

        memset(temp_buf, 0, 256);
        memcpy(temp_buf, AMI_PAD_DEFAULT_CALI_FILE, strlen(AMI_PAD_DEFAULT_CALI_FILE));
        if(0 != mpu_file_write_6050(ASUS_PAD_CALIBRATION_FILE,temp_buf,strlen(temp_buf)))
        {
            ret = 4;
            printk("[mpu] create default %s failed\n",ASUS_PHONE_CALIBRATION_FILE);
        }
        //ASUS_BSP --- Jason Chang "create default calibration file when lost calibration in asusdata"
    }

    if(cali_buf)
        kfree(cali_buf);
    if(temp_buf)
        kfree(temp_buf);

    return ret;
}
//ASUS_BSP --- Jason Chang "Recover compass calibration file when file crashed"
//ASUS_BSP +++ Jason Chang "replace compass calibration by workqueue"
int mpu_copy_calibration_file_6050(char *target_name, char *source_name)
{
    int ret=0;
    int debug = 0;
    unsigned char buffer[256];
    struct file *target_fp;
    struct file *source_fp;
    mm_segment_t oldfs;

    oldfs = get_fs(); 
    set_fs(KERNEL_DS);

    source_fp = filp_open(source_name, O_CREAT | O_RDWR | O_SYNC, S_IROTH | S_IWOTH | S_IWGRP | S_IRGRP | S_IWUSR | S_IRUSR);
    if(!IS_ERR(source_fp))
    {
        target_fp = filp_open(target_name, O_CREAT | O_RDWR | O_SYNC, S_IROTH | S_IWOTH | S_IWGRP | S_IRGRP | S_IWUSR | S_IRUSR);
        if(!IS_ERR(target_fp))
        {
            memset(buffer, 0, 256);
            target_fp->f_op->write(target_fp, buffer, 256, &target_fp->f_pos);
            source_fp->f_op->read(source_fp, buffer, 256, &source_fp->f_pos);
            if(debug)
                printk("\njason debug:%s = \n %s\n",source_name,buffer);
            target_fp->f_pos = 0;
            target_fp->f_op->write(target_fp, buffer, 256, &target_fp->f_pos);
            if(debug)
            {
                memset(buffer, 0, 256);
                target_fp->f_pos = 0;
                target_fp->f_op->read(target_fp, buffer, 256, &target_fp->f_pos);
                printk("\njason debug:write to %s = \n %s\n",target_name,buffer);
            }
            if(filp_close(target_fp, NULL))
                printk("[mpu] Err: close %s failed\n",target_name);
        }
        else
        {
            
            printk("[mpu] Err: open %s failed\n",target_name);
            ret = -1;
        }
        if(filp_close(source_fp, NULL))
            printk("[mpu] Err: close %s failed\n",source_name);
    }
    else
    {
        printk("[mpu] Err: open %s failed\n",source_name);
        ret = -1;
    }
    
    set_fs(oldfs); 

    if(ret == 0)
        printk("[mpu] copy %s to %s\n",source_name,target_name);
    return ret;
}

int asus_switch_compass_calibration_file_6050(int current_status)
{
    int ret = 0;
    
    if(current_status)
    {
        ret += mpu_copy_calibration_file_6050(ASUS_PHONE_CALIBRATION_FILE, AMI_CALIBRATION_FILE);
        ret += mpu_copy_calibration_file_6050(AMI_CALIBRATION_FILE, ASUS_PAD_CALIBRATION_FILE);
    }
    else
    {
        ret += mpu_copy_calibration_file_6050(ASUS_PAD_CALIBRATION_FILE, AMI_CALIBRATION_FILE);
        ret += mpu_copy_calibration_file_6050(AMI_CALIBRATION_FILE, ASUS_PHONE_CALIBRATION_FILE);
    }

    return ret;
}
//ASUS_BSP --- Jason Chang "replace compass calibration by workqueue"
//ASUS_BSP --- Jason Chang "switch compass calibration data when plug in/out PAD"

//ASUS_BSP +++ Jason Chang "dynamic change orientation"
void mpu_change_rotation_axes_6050(int value) {
    
    if(value == MPU_EVENT_NOTIFY_PAD)
    {
        printk("[mpu] change ori to PAD\n");
        memcpy(asus_g_mldl_cfg_6050->pdata->orientation, Pad_Gyro_Orient, 9);
        memcpy(asus_g_mldl_cfg_6050->pdata_slave[EXT_SLAVE_TYPE_ACCEL]->orientation, Pad_Acc_Orient, 9);
        memcpy(asus_g_mldl_cfg_6050->pdata_slave[EXT_SLAVE_TYPE_COMPASS]->orientation, Pad_Mag_Orient, 9);
    }
    else if(value == MPU_EVENT_NOTIFY_PHONE)
    {
        printk("[mpu] change ori to PHONE\n");
        memcpy(asus_g_mldl_cfg_6050->pdata->orientation, Phone_Gyro_Orient, 9);
        memcpy(asus_g_mldl_cfg_6050->pdata_slave[EXT_SLAVE_TYPE_ACCEL]->orientation, Phone_Acc_Orient, 9);
        memcpy(asus_g_mldl_cfg_6050->pdata_slave[EXT_SLAVE_TYPE_COMPASS]->orientation, Phone_Mag_Orient, 9);
    }
    g_orientation_flag = value;

}

void mpu_get_orientation_6050(void)
{
    switch (g_A60K_hwID)
    {
        case A60K_EVB:
            printk("[mpu] get evb orientation\n");
            Pad_Acc_Orient = Pad_accel_orientation;
            Pad_Gyro_Orient = Pad_gyro_orientation;
            Pad_Mag_Orient = Pad_mag_orientation;
            Phone_Acc_Orient = Phone_accel_orientation;
            Phone_Gyro_Orient = Phone_gyro_orientation;
            Phone_Mag_Orient = Phone_mag_orientation;
            break;
        
        case A60K_SR1_1:
        case A60K_SR1_2_ES1:
        case A60K_SR1_2_ES2:
        case A60K_ER1:
        case A60K_ER2:
        case A60K_PR:
            printk("[mpu] get sr1 orientation\n");
            Pad_Acc_Orient = Pad_accel_orientation_sr1;
            Pad_Gyro_Orient = Pad_gyro_orientation_sr1;
            Pad_Mag_Orient = Pad_mag_orientation_sr1;
            Phone_Acc_Orient = Phone_accel_orientation_sr1;
            Phone_Gyro_Orient = Phone_gyro_orientation_sr1;
            Phone_Mag_Orient = Phone_mag_orientation_sr1;
            break;
			
        case A66_HW_ID_SR1_1:
       	case A66_HW_ID_SR2:
	case A66_HW_ID_ER1:
	case A66_HW_ID_ER2:
	case A66_HW_ID_ER3:
	case A66_HW_ID_PR:
            printk("[mpu] get A66_SR1 orientation\n");
            Pad_Acc_Orient = Pad_accel_orientation_A66sr1;
            Pad_Gyro_Orient = Pad_gyro_orientation_A66sr1;
            Pad_Mag_Orient = Pad_mag_orientation_A66sr1;
            Phone_Acc_Orient = Phone_accel_orientation_A66sr1;
            Phone_Gyro_Orient = Phone_gyro_orientation_A66sr1;
            Phone_Mag_Orient = Phone_mag_orientation_A66sr1;
            break;

        default:
            printk(KERN_ERR "[mpu] Error: There is NO valid hardware ID\n");
            break;
    }
    return;
}
//ASUS_BSP+++ Jason Chang "change oritation for pad"
static int mp_sensor_event(struct notifier_block *this, unsigned long event, void *ptr)
{
        switch (event) {
        case P01_ADD:
                printk("[mpu] PAD ADD \r\n");
                mpu_change_rotation_axes_6050(MPU_EVENT_NOTIFY_PAD);
                //ASUS_BSP +++ Jason Chang "replace compass calibration by workqueue"
                cancel_delayed_work_sync(&pad_plugout_work_6050);
                queue_delayed_work(pad_plug_wq_6050, &pad_plugin_work_6050, 0);
                //ASUS_BSP --- Jason Chang "replace compass calibration by workqueue"
                return NOTIFY_DONE;
                
        case P01_REMOVE:
                printk("[mpu] PAD REMOVE \r\n");
                mpu_change_rotation_axes_6050(MPU_EVENT_NOTIFY_PHONE);
                //ASUS_BSP +++ Jason Chang "replace compass calibration by workqueue"
                cancel_delayed_work_sync(&pad_plugin_work_6050);
                queue_delayed_work(pad_plug_wq_6050, &pad_plugout_work_6050, 0);
                //ASUS_BSP --- Jason Chang "replace compass calibration by workqueue"
                return NOTIFY_DONE;

        default:
                return NOTIFY_DONE;
        }
}

static struct notifier_block mpu_hs_notifier = {
        .notifier_call = mp_sensor_event,
        .priority = NINE_AXIS_SENSOR_MP_NOTIFY,
};
//ASUS_BSP--- Jason Chang "change oritation for pad"
//ASUS_BSP --- Jason Chang "dynamic change orientation"

//ASUS_BSP +++ Jason Chang "replace compass calibration by workqueue"
static void mpu_plugin_pad_work(struct work_struct *work)
{
    g_calibration_file_flag_6050 = 1;
    if(asus_switch_compass_calibration_file_6050(MPU_EVENT_NOTIFY_PAD))
        printk("[mpu] switch calibration file to PAD failed\n");
    g_calibration_file_flag_6050 = 0;
    return;
}

static void mpu_plugout_pad_work(struct work_struct *work)
{
    g_calibration_file_flag_6050 = 1;
    if(asus_switch_compass_calibration_file_6050(MPU_EVENT_NOTIFY_PHONE))
        printk("[mpu] switch calibration file to PHONE failed\n");
    g_calibration_file_flag_6050 = 0;
    return;
}
//ASUS_BSP --- Jason Chang "replace compass calibration by workqueue"

static void mpu_pm_timeout(u_long data)
{
	struct mpu_private_data_6050 *mpu = (struct mpu_private_data_6050 *)data;
	struct i2c_client *client = mpu->client;
	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	complete(&mpu->completion);
}

static int mpu_pm_notifier_callback(struct notifier_block *nb,
				    unsigned long event, void *unused)
{
	struct mpu_private_data_6050 *mpu =
	    container_of(nb, struct mpu_private_data_6050, nb);
	struct i2c_client *client = mpu->client;
	struct timeval event_time;
	dev_dbg(&client->adapter->dev, "%s: %ld\n", __func__, event);

	/* Prevent the file handle from being closed before we initialize
	   the completion event */
	mutex_lock(&mpu->mutex);
	if (!(mpu->pid) ||
	    (event != PM_SUSPEND_PREPARE && event != PM_POST_SUSPEND)) {
		mutex_unlock(&mpu->mutex);
		return NOTIFY_OK;
	}

	if (event == PM_SUSPEND_PREPARE)
		mpu->event = MPU_PM_EVENT_SUSPEND_PREPARE;
	if (event == PM_POST_SUSPEND)
		mpu->event = MPU_PM_EVENT_POST_SUSPEND;

	do_gettimeofday(&event_time);
	mpu->mpu_pm_event.interruptcount++;
	mpu->mpu_pm_event.irqtime =
	    (((long long)event_time.tv_sec) << 32) + event_time.tv_usec;
	mpu->mpu_pm_event.data_type = MPUIRQ_DATA_TYPE_PM_EVENT;
	mpu->mpu_pm_event.data = mpu->event;

	if (mpu->response_timeout > 0) {
		mpu->timeout.expires = jiffies + mpu->response_timeout * HZ;
		add_timer(&mpu->timeout);
	}
	INIT_COMPLETION(mpu->completion);
	mutex_unlock(&mpu->mutex);

	wake_up_interruptible(&mpu->mpu_event_wait);
	wait_for_completion(&mpu->completion);
	del_timer_sync(&mpu->timeout);
	dev_dbg(&client->adapter->dev, "%s: %ld DONE\n", __func__, event);
	return NOTIFY_OK;
}

static int mpu_dev_open(struct inode *inode, struct file *file)
{
	struct mpu_private_data_6050 *mpu =
	    container_of(file->private_data, struct mpu_private_data_6050, dev);
	struct i2c_client *client = mpu->client;
	int result;
	int ii;
	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	dev_dbg(&client->adapter->dev, "current->pid %d\n", current->pid);

	result = mutex_lock_interruptible(&mpu->mutex);
/*	if (mpu->pid) {
		mutex_unlock(&mpu->mutex);
		return -EBUSY;
	}*/
	mpu->pid = current->pid;

	/* Reset the sensors to the default */
	if (result) {
		dev_err(&client->adapter->dev,
			"%s: mutex_lock_interruptible returned %d\n",
			__func__, result);
		return result;
	}

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++)
		__module_get(mpu->slave_modules[ii]);

	mutex_unlock(&mpu->mutex);
	return 0;
}

/* close function - called when the "file" /dev/mpu is closed in userspace   */
static int mpu_release(struct inode *inode, struct file *file)
{
	struct mpu_private_data_6050 *mpu =
	    container_of(file->private_data, struct mpu_private_data_6050, dev);
	struct i2c_client *client = mpu->client;
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	int result = 0;
	int ii;
	struct i2c_adapter *slave_adapter[EXT_SLAVE_NUM_TYPES];
	struct ext_slave_platform_data **pdata_slave = mldl_cfg->pdata_slave;

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++) {
		if (!pdata_slave[ii])
			slave_adapter[ii] = NULL;
		else
			slave_adapter[ii] =
				i2c_get_adapter(pdata_slave[ii]->adapt_num);
	}
	slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE] = client->adapter;

	mutex_lock(&mpu->mutex);
	mldl_cfg->inv_mpu_cfg->requested_sensors = 0;
	result = inv_mpu_suspend_6050(mldl_cfg,
				slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
				slave_adapter[EXT_SLAVE_TYPE_ACCEL],
				slave_adapter[EXT_SLAVE_TYPE_COMPASS],
				slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
				INV_ALL_SENSORS);
	mpu->pid = 0;
	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++)
		module_put(mpu->slave_modules[ii]);

	mutex_unlock(&mpu->mutex);
	complete(&mpu->completion);
	dev_dbg(&client->adapter->dev, "mpu_release\n");

	return result;
}

/* read function called when from /dev/mpu is read.  Read from the FIFO */
static ssize_t mpu_read(struct file *file,
			char __user *buf, size_t count, loff_t *offset)
{
	struct mpu_private_data_6050 *mpu =
	    container_of(file->private_data, struct mpu_private_data_6050, dev);
	struct i2c_client *client = mpu->client;
	size_t len = sizeof(mpu->mpu_pm_event) + sizeof(unsigned long);
	int err;

	if (!mpu->event && (!(file->f_flags & O_NONBLOCK)))
		wait_event_interruptible(mpu->mpu_event_wait, mpu->event);

	if (!mpu->event || !buf
	    || count < sizeof(mpu->mpu_pm_event))
		return 0;

	err = copy_to_user(buf, &mpu->mpu_pm_event, sizeof(mpu->mpu_pm_event));
	if (err) {
		dev_err(&client->adapter->dev,
			"Copy to user returned %d\n", err);
		return -EFAULT;
	}
	mpu->event = 0;
	return len;
}

static unsigned int mpu_poll(struct file *file, struct poll_table_struct *poll)
{
	struct mpu_private_data_6050 *mpu =
	    container_of(file->private_data, struct mpu_private_data_6050, dev);
	int mask = 0;

	poll_wait(file, &mpu->mpu_event_wait, poll);
	if (mpu->event)
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

static int mpu_dev_ioctl_get_ext_slave_platform_data(
	struct i2c_client *client,
	struct ext_slave_platform_data __user *arg)
{
	struct mpu_private_data_6050 *mpu =
	    (struct mpu_private_data_6050 *)i2c_get_clientdata(client);
	struct ext_slave_platform_data *pdata_slave;
	struct ext_slave_platform_data local_pdata_slave;

	if (copy_from_user(&local_pdata_slave, arg, sizeof(local_pdata_slave)))
		return -EFAULT;

	if (local_pdata_slave.type >= EXT_SLAVE_NUM_TYPES)
		return -EINVAL;

	pdata_slave = mpu->mldl_cfg.pdata_slave[local_pdata_slave.type];
	/* All but private data and irq_data */
	if (!pdata_slave)
		return -ENODEV;
	if (copy_to_user(arg, pdata_slave, sizeof(*pdata_slave)))
		return -EFAULT;
	return 0;
}

static int mpu_dev_ioctl_get_mpu_platform_data(
	struct i2c_client *client,
	struct mpu_platform_data __user *arg)
{
	struct mpu_private_data_6050 *mpu =
	    (struct mpu_private_data_6050 *)i2c_get_clientdata(client);
	struct mpu_platform_data *pdata = mpu->mldl_cfg.pdata;

	if (copy_to_user(arg, pdata, sizeof(*pdata)))
		return -EFAULT;
	return 0;
}

static int mpu_dev_ioctl_get_ext_slave_descr(
	struct i2c_client *client,
	struct ext_slave_descr __user *arg)
{
	struct mpu_private_data_6050 *mpu =
	    (struct mpu_private_data_6050 *)i2c_get_clientdata(client);
	struct ext_slave_descr *slave;
	struct ext_slave_descr local_slave;

	if (copy_from_user(&local_slave, arg, sizeof(local_slave)))
		return -EFAULT;

	if (local_slave.type >= EXT_SLAVE_NUM_TYPES)
		return -EINVAL;

	slave = mpu->mldl_cfg.slave[local_slave.type];
	/* All but private data and irq_data */
	if (!slave)
		return -ENODEV;
	if (copy_to_user(arg, slave, sizeof(*slave)))
		return -EFAULT;
	return 0;
}


/**
 * slave_config() - Pass a requested slave configuration to the slave sensor
 *
 * @adapter the adaptor to use to communicate with the slave
 * @mldl_cfg the mldl configuration structuer
 * @slave pointer to the slave descriptor
 * @usr_config The configuration to pass to the slave sensor
 *
 * returns 0 or non-zero error code
 */
static int inv_mpu_config(struct mldl_cfg *mldl_cfg,
			void *gyro_adapter,
			struct ext_slave_config __user *usr_config)
{
	int retval = 0;
	struct ext_slave_config config;

	retval = copy_from_user(&config, usr_config, sizeof(config));
	if (retval)
		return -EFAULT;

	if (config.len && config.data) {
		void *data;
		data = kmalloc(config.len, GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		retval = copy_from_user(data,
					(void __user *)config.data, config.len);
		if (retval) {
			retval = -EFAULT;
			kfree(data);
			return retval;
		}
		config.data = data;
	}
	retval = gyro_config_6050(gyro_adapter, mldl_cfg, &config);
	kfree(config.data);
	return retval;
}

static int inv_mpu_get_config(struct mldl_cfg *mldl_cfg,
			    void *gyro_adapter,
			    struct ext_slave_config __user *usr_config)
{
	int retval = 0;
	struct ext_slave_config config;
	void *user_data;

	retval = copy_from_user(&config, usr_config, sizeof(config));
	if (retval)
		return -EFAULT;

	user_data = config.data;
	if (config.len && config.data) {
		void *data;
		data = kmalloc(config.len, GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		retval = copy_from_user(data,
					(void __user *)config.data, config.len);
		if (retval) {
			retval = -EFAULT;
			kfree(data);
			return retval;
		}
		config.data = data;
	}
	retval = gyro_get_config_6050(gyro_adapter, mldl_cfg, &config);
	if (!retval)
		retval = copy_to_user((unsigned char __user *)user_data,
				config.data, config.len);
	kfree(config.data);
	return retval;
}

static int slave_config(struct mldl_cfg *mldl_cfg,
			void *gyro_adapter,
			void *slave_adapter,
			struct ext_slave_descr *slave,
			struct ext_slave_platform_data *pdata,
			struct ext_slave_config __user *usr_config)
{
	int retval = 0;
	struct ext_slave_config config;
	if ((!slave) || (!slave->config))
		return -ENODEV;

	retval = copy_from_user(&config, usr_config, sizeof(config));
	if (retval)
		return -EFAULT;

	if (config.len && config.data) {
		void *data;
		data = kmalloc(config.len, GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		retval = copy_from_user(data,
					(void __user *)config.data, config.len);
		if (retval) {
			retval = -EFAULT;
			kfree(data);
			return retval;
		}
		config.data = data;
	}
	retval = inv_mpu_slave_config_6050(mldl_cfg, gyro_adapter, slave_adapter,
				      &config, slave, pdata);
	kfree(config.data);
	return retval;
}

static int slave_get_config(struct mldl_cfg *mldl_cfg,
			    void *gyro_adapter,
			    void *slave_adapter,
			    struct ext_slave_descr *slave,
			    struct ext_slave_platform_data *pdata,
			    struct ext_slave_config __user *usr_config)
{
	int retval = 0;
	struct ext_slave_config config;
	void *user_data;
	if (!(slave) || !(slave->get_config))
		return -ENODEV;

	retval = copy_from_user(&config, usr_config, sizeof(config));
	if (retval)
		return -EFAULT;

	user_data = config.data;
	if (config.len && config.data) {
		void *data;
		data = kmalloc(config.len, GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		retval = copy_from_user(data,
					(void __user *)config.data, config.len);
		if (retval) {
			retval = -EFAULT;
			kfree(data);
			return retval;
		}
		config.data = data;
	}
	retval = inv_mpu_get_slave_config_6050(mldl_cfg, gyro_adapter,
					  slave_adapter, &config, slave, pdata);
	if (retval) {
		kfree(config.data);
		return retval;
	}
	retval = copy_to_user((unsigned char __user *)user_data,
			      config.data, config.len);
	kfree(config.data);
	return retval;
}

static int inv_slave_read(struct mldl_cfg *mldl_cfg,
			  void *gyro_adapter,
			  void *slave_adapter,
			  struct ext_slave_descr *slave,
			  struct ext_slave_platform_data *pdata,
			  void __user *usr_data)
{
	int retval;
	unsigned char *data;
	data = kzalloc(slave->read_len, GFP_KERNEL);
	if (!data)
		return -EFAULT;

	retval = inv_mpu_slave_read_6050(mldl_cfg, gyro_adapter, slave_adapter,
				    slave, pdata, data);

	if ((!retval) &&
	    (copy_to_user((unsigned char __user *)usr_data,
			  data, slave->read_len)))
		retval = -EFAULT;

	kfree(data);
	return retval;
}
//ASUS_BSP +++ Jason Chang "dynamic change orientation"
int asus_orient_status_read_6050(void)
{
    return g_orientation_flag;
}
//ASUS_BSP --- Jason Chang "dynamic change orientation"
static int mpu_handle_mlsl(void *sl_handle,
			   unsigned char addr,
			   unsigned int cmd,
			   struct mpu_read_write __user *usr_msg)
{
	int retval = 0;
	struct mpu_read_write msg;
	unsigned char *user_data;
	retval = copy_from_user(&msg, usr_msg, sizeof(msg));
	if (retval)
		return -EFAULT;

	user_data = msg.data;
	if (msg.length && msg.data) {
		unsigned char *data;
		data = kmalloc(msg.length, GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		retval = copy_from_user(data,
					(void __user *)msg.data, msg.length);
		if (retval) {
			retval = -EFAULT;
			kfree(data);
			return retval;
		}
		msg.data = data;
	} else {
		return -EPERM;
	}

	switch (cmd) {
	case MPU_READ:
		retval = inv_serial_read_6050(sl_handle, addr,
					 msg.address, msg.length, msg.data);
		break;
	case MPU_WRITE:
		retval = inv_serial_write_6050(sl_handle, addr,
					  msg.length, msg.data);
		break;
	case MPU_READ_MEM:
		retval = inv_serial_read_mem_6050(sl_handle, addr,
					     msg.address, msg.length, msg.data);
		break;
	case MPU_WRITE_MEM:
		retval = inv_serial_write_mem_6050(sl_handle, addr,
					      msg.address, msg.length,
					      msg.data);
		break;
	case MPU_READ_FIFO:
		retval = inv_serial_read_fifo_6050(sl_handle, addr,
					      msg.length, msg.data);
		break;
	case MPU_WRITE_FIFO:
		retval = inv_serial_write_fifo_6050(sl_handle, addr,
					       msg.length, msg.data);
		break;

	};
	if (retval) {
		dev_err(&((struct i2c_adapter *)sl_handle)->dev,
			"%s: i2c %d error %d\n",
			__func__, cmd, retval);
		kfree(msg.data);
		return retval;
	}
	retval = copy_to_user((unsigned char __user *)user_data,
			      msg.data, msg.length);
	kfree(msg.data);
	return retval;
}

/* ioctl - I/O control */
static long mpu_dev_ioctl(struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	struct mpu_private_data_6050 *mpu =
	    container_of(file->private_data, struct mpu_private_data_6050, dev);
	struct i2c_client *client = mpu->client;
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	int retval = 0;
	struct i2c_adapter *slave_adapter[EXT_SLAVE_NUM_TYPES];
	struct ext_slave_descr **slave = mldl_cfg->slave;
	struct ext_slave_platform_data **pdata_slave = mldl_cfg->pdata_slave;
	int ii;
	unsigned char temp[32];
//ASUS_BSP +++ Jason Chang "switch compass calibration data when plug in/out PAD"
	unsigned char *cali_buf;
        unsigned char *temp_buf;
	static int one_time = 1;
	static int retry = 0;
	struct file *fp;
	mm_segment_t oldfs;
//ASUS_BSP --- Jason Chang "switch compass calibration data when plug in/out PAD"
//ASUS_BSP +++ Jason Chang "Recover compass calibration file when file crashed"
        int mem_index;
        int count = 0;
        char temp_string[64];
//ASUS_BSP --- Jason Chang "Recover compass calibration file when file crashed"

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++) {
		if (!pdata_slave[ii])
			slave_adapter[ii] = NULL;
		else
			slave_adapter[ii] =
				i2c_get_adapter(pdata_slave[ii]->adapt_num);
	}
	slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE] = client->adapter;

	retval = mutex_lock_interruptible(&mpu->mutex);
	if (retval) {
		dev_err(&client->adapter->dev,
			"%s: mutex_lock_interruptible returned %d\n",
			__func__, retval);
		return retval;
	}

	switch (cmd) {
	case MPU_GET_EXT_SLAVE_PLATFORM_DATA:
		retval = mpu_dev_ioctl_get_ext_slave_platform_data(
			client,
			(struct ext_slave_platform_data __user *)arg);
		break;
	case MPU_GET_MPU_PLATFORM_DATA:
        //ASUS_BSP +++ Jason Chang "switch compass calibration data when plug in/out PAD"
            if(one_time && retry < 5)
            {
                if(!(cali_buf = kmalloc(256, GFP_KERNEL)) || !(temp_buf = kmalloc(256, GFP_KERNEL)))
                    printk("[mpu] Allocate calibration data buffer failed\n");
                else
                {
                    oldfs = get_fs(); 
                    set_fs(KERNEL_DS);
                    
                    fp = filp_open(AMI_CALIBRATION_FILE, O_RDONLY, 0);
                    if(IS_ERR(fp))
                    {
                        if(0 != mpu_file_read_6050(ASUS_COMPASS_CALIBRATION_FILE,cali_buf,256))
                        {
                            printk("[mpu] get %s failed, Create default calibration data\n",ASUS_COMPASS_CALIBRATION_FILE);

                            memset(temp_buf, 0, 256);
                            memcpy(temp_buf, AMI_DEFAULT_CALI_FILE, strlen(AMI_DEFAULT_CALI_FILE));
                            if(0 != mpu_file_write_6050(AMI_CALIBRATION_FILE,temp_buf,strlen(temp_buf)))
                                printk("[mpu] create default %s failed\n",AMI_CALIBRATION_FILE);
                            if(0 != mpu_file_write_6050(ASUS_PHONE_CALIBRATION_FILE,temp_buf,strlen(temp_buf)))
                                printk("[mpu] create default %s failed\n",ASUS_PHONE_CALIBRATION_FILE);

                            memset(temp_buf, 0, 256);
                            memcpy(temp_buf, AMI_PAD_DEFAULT_CALI_FILE, strlen(AMI_PAD_DEFAULT_CALI_FILE));
                            if(0 != mpu_file_write_6050(ASUS_PAD_CALIBRATION_FILE,temp_buf,strlen(temp_buf)))
                                printk("[mpu] create default %s failed\n",ASUS_PHONE_CALIBRATION_FILE);
                        }
                        else
                        {
                            Create_compass_calibration_file_6050();
                            one_time = 0;
                        }
                    }
                    else
                    {
                        filp_close(fp,NULL);
                        printk("[mpu] compass calibration exist, check calibration file...\n");

                        //ASUS_BSP +++ Jason Chang "Recover compass calibration file when file crashed"
                        memset(temp_buf, 0, 256);
                        count = 0;
                        if(0 == mpu_file_read_6050(AMI_CALIBRATION_FILE,temp_buf,256))
                        {
                            //parser inter offset
                            for(mem_index = 0; mem_index<256; mem_index++)
                            {
                                if(temp_buf[mem_index] == 0x0a)
                                {
                                    count++;
                                    if(count == 9)  // line 9
                                    {
                                        memset(temp_string, 0xff, sizeof(temp));
                                        memcpy(temp_string, (temp_buf+mem_index+1), sizeof(temp_string));
                                        printk("[mpu] inter offset = [%s]\n",temp_string);

                                        if(!memcmp(temp_string,AMI_INTER_OFFSET_PHONE,strlen(temp_string)) || !memcmp(temp_string,AMI_INTER_OFFSET_PAD,strlen(temp_string)))
                                        {
                                            printk("[mpu] calibraiton file is valid\n");
                                        }
                                        else
                                        {
                                            printk("[mpu] %s invalid, recover calibration file\n",AMI_CALIBRATION_FILE);
                                            Create_compass_calibration_file_6050();
                                        }
                                    }
                                }
                            }
                        }
                        //ASUS_BSP --- Jason Chang "Recover compass calibration file when file crashed"
                        one_time = 0;
                    }
                
                    set_fs(oldfs); 
                    retry++ ;
                    kfree(cali_buf);
                    kfree(temp_buf);
                }
            }
        //ASUS_BSP --- Jason Chang "switch compass calibration data when plug in/out PAD"
		retval = mpu_dev_ioctl_get_mpu_platform_data(
			client,
			(struct mpu_platform_data __user *)arg);
		break;
	case MPU_GET_EXT_SLAVE_DESCR:
		retval = mpu_dev_ioctl_get_ext_slave_descr(
			client,
			(struct ext_slave_descr __user *)arg);
		break;
	case MPU_READ:
	case MPU_WRITE:
	case MPU_READ_MEM:
	case MPU_WRITE_MEM:
	case MPU_READ_FIFO:
	case MPU_WRITE_FIFO:
		retval = mpu_handle_mlsl(
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			mldl_cfg->mpu_chip_info->addr, cmd,
			(struct mpu_read_write __user *)arg);
		break;
	case MPU_CONFIG_GYRO:
		retval = inv_mpu_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_CONFIG_ACCEL:
		retval = slave_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave[EXT_SLAVE_TYPE_ACCEL],
			pdata_slave[EXT_SLAVE_TYPE_ACCEL],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_CONFIG_COMPASS:
		retval = slave_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave[EXT_SLAVE_TYPE_COMPASS],
			pdata_slave[EXT_SLAVE_TYPE_COMPASS],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_CONFIG_PRESSURE:
		retval = slave_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			slave[EXT_SLAVE_TYPE_PRESSURE],
			pdata_slave[EXT_SLAVE_TYPE_PRESSURE],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_GET_CONFIG_GYRO:
		retval = inv_mpu_get_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_GET_CONFIG_ACCEL:
		retval = slave_get_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave[EXT_SLAVE_TYPE_ACCEL],
			pdata_slave[EXT_SLAVE_TYPE_ACCEL],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_GET_CONFIG_COMPASS:
		retval = slave_get_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave[EXT_SLAVE_TYPE_COMPASS],
			pdata_slave[EXT_SLAVE_TYPE_COMPASS],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_GET_CONFIG_PRESSURE:
		retval = slave_get_config(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			slave[EXT_SLAVE_TYPE_PRESSURE],
			pdata_slave[EXT_SLAVE_TYPE_PRESSURE],
			(struct ext_slave_config __user *)arg);
		break;
	case MPU_SUSPEND:
		retval = inv_mpu_suspend_6050(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			arg);
		break;
	case MPU_RESUME:
		retval = inv_mpu_resume_6050(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			arg);
		break;
	case MPU_PM_EVENT_HANDLED:
		dev_dbg(&client->adapter->dev, "%s: %d\n", __func__, cmd);
		complete(&mpu->completion);
		break;
	case MPU_READ_ACCEL:
		retval = inv_slave_read(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave[EXT_SLAVE_TYPE_ACCEL],
			pdata_slave[EXT_SLAVE_TYPE_ACCEL],
			(unsigned char __user *)arg);
		break;
	case MPU_READ_COMPASS:
		retval = inv_slave_read(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave[EXT_SLAVE_TYPE_COMPASS],
			pdata_slave[EXT_SLAVE_TYPE_COMPASS],
			(unsigned char __user *)arg);
		break;
	case MPU_READ_PRESSURE:
		retval = inv_slave_read(
			mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			slave[EXT_SLAVE_TYPE_PRESSURE],
			pdata_slave[EXT_SLAVE_TYPE_PRESSURE],
			(unsigned char __user *)arg);
		break;
	case MPU_GET_REQUESTED_SENSORS:
		if (copy_to_user(
			   (__u32 __user *)arg,
			   &mldl_cfg->inv_mpu_cfg->requested_sensors,
			   sizeof(mldl_cfg->inv_mpu_cfg->requested_sensors)))
			retval = -EFAULT;
		break;
	case MPU_SET_REQUESTED_SENSORS:
		mldl_cfg->inv_mpu_cfg->requested_sensors = arg;
		break;
	case MPU_GET_IGNORE_SYSTEM_SUSPEND:
		if (copy_to_user(
			(unsigned char __user *)arg,
			&mldl_cfg->inv_mpu_cfg->ignore_system_suspend,
			sizeof(mldl_cfg->inv_mpu_cfg->ignore_system_suspend)))
			retval = -EFAULT;
		break;
	case MPU_SET_IGNORE_SYSTEM_SUSPEND:
		mldl_cfg->inv_mpu_cfg->ignore_system_suspend = arg;
		break;
	case MPU_GET_MLDL_STATUS:
		if (copy_to_user(
			(unsigned char __user *)arg,
			&mldl_cfg->inv_mpu_state->status,
			sizeof(mldl_cfg->inv_mpu_state->status)))
			retval = -EFAULT;
		break;
	case MPU_GET_I2C_SLAVES_ENABLED:
		if (copy_to_user(
			(unsigned char __user *)arg,
			&mldl_cfg->inv_mpu_state->i2c_slaves_enabled,
			sizeof(mldl_cfg->inv_mpu_state->i2c_slaves_enabled)))
			retval = -EFAULT;
		break;
//ASUS_BSP +++ Jason Chang "dynamic change orientation"
        case ASUS_MPU_READ_ORIENT_STASUS:
                {
                        retval = asus_orient_status_read_6050();
                        //printk("mpu: orientation = %d\n",retval);
                }
                break;
//ASUS_BSP --- Jason Chang "dynamic change orientation"
//ASUS_BSP +++ Jason Chang "replace compass calibration by workqueue"
        case ASUS_CHECK_COMPASS_CALIBRATION_STATUS:
                {
                        retval = g_calibration_file_flag_6050;
                }
                break;
//ASUS_BSP --- Jason Chang "replace compass calibration by workqueue"
	default:
		dev_err(&client->adapter->dev,
			"%s6050: Unknown cmd %x, arg %lu\n",
			__func__, cmd, arg);
		retval = -EINVAL;
	};

	mutex_unlock(&mpu->mutex);
	dev_dbg(&client->adapter->dev, "%s: %08x, %08lx, %d\n",
		__func__, cmd, arg, retval);

	if ((retval > 0)&&(cmd!=ASUS_MPU_READ_ORIENT_STASUS))
		retval = -retval;

	return retval;
}

void mpu_shutdown_6050(struct i2c_client *client)
{
	struct mpu_private_data_6050 *mpu =
	    (struct mpu_private_data_6050 *)i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct i2c_adapter *slave_adapter[EXT_SLAVE_NUM_TYPES];
	struct ext_slave_platform_data **pdata_slave = mldl_cfg->pdata_slave;
	int ii;

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++) {
		if (!pdata_slave[ii])
			slave_adapter[ii] = NULL;
		else
			slave_adapter[ii] =
				i2c_get_adapter(pdata_slave[ii]->adapt_num);
	}
	slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE] = client->adapter;

	mutex_lock(&mpu->mutex);
	(void)inv_mpu_suspend_6050(mldl_cfg,
			slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
			slave_adapter[EXT_SLAVE_TYPE_ACCEL],
			slave_adapter[EXT_SLAVE_TYPE_COMPASS],
			slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
			INV_ALL_SENSORS);
	mutex_unlock(&mpu->mutex);
	dev_dbg(&client->adapter->dev, "%s\n", __func__);
}

int mpu_dev_suspend_6050(struct i2c_client *client, pm_message_t mesg)
{
	struct mpu_private_data_6050 *mpu =
	    (struct mpu_private_data_6050 *)i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct i2c_adapter *slave_adapter[EXT_SLAVE_NUM_TYPES];
	struct ext_slave_platform_data **pdata_slave = mldl_cfg->pdata_slave;
	int ii;

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++) {
		if (!pdata_slave[ii])
			slave_adapter[ii] = NULL;
		else
			slave_adapter[ii] =
				i2c_get_adapter(pdata_slave[ii]->adapt_num);
	}
	slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE] = client->adapter;

	mutex_lock(&mpu->mutex);
	if (!mldl_cfg->inv_mpu_cfg->ignore_system_suspend) {
		dev_dbg(&client->adapter->dev,
			"%s: suspending on event %d\n", __func__, mesg.event);
		(void)inv_mpu_suspend_6050(mldl_cfg,
				slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
				slave_adapter[EXT_SLAVE_TYPE_ACCEL],
				slave_adapter[EXT_SLAVE_TYPE_COMPASS],
				slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
				INV_ALL_SENSORS);
	} else {
		dev_dbg(&client->adapter->dev,
			"%s: Already suspended %d\n", __func__, mesg.event);
	}
	mutex_unlock(&mpu->mutex);
	return 0;
}

int mpu_dev_resume_6050(struct i2c_client *client)
{
	struct mpu_private_data_6050 *mpu =
	    (struct mpu_private_data_6050 *)i2c_get_clientdata(client);
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct i2c_adapter *slave_adapter[EXT_SLAVE_NUM_TYPES];
	struct ext_slave_platform_data **pdata_slave = mldl_cfg->pdata_slave;
	int ii;

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++) {
		if (!pdata_slave[ii])
			slave_adapter[ii] = NULL;
		else
			slave_adapter[ii] =
				i2c_get_adapter(pdata_slave[ii]->adapt_num);
	}
	slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE] = client->adapter;

	mutex_lock(&mpu->mutex);
	if (mpu->pid && !mldl_cfg->inv_mpu_cfg->ignore_system_suspend) {
		(void)inv_mpu_resume_6050(mldl_cfg,
				slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
				slave_adapter[EXT_SLAVE_TYPE_ACCEL],
				slave_adapter[EXT_SLAVE_TYPE_COMPASS],
				slave_adapter[EXT_SLAVE_TYPE_PRESSURE],
				mldl_cfg->inv_mpu_cfg->requested_sensors);
		dev_dbg(&client->adapter->dev,
			"%s for pid %d\n", __func__, mpu->pid);
	}
	mutex_unlock(&mpu->mutex);
	return 0;
}

/* define which file operations are supported */
static const struct file_operations mpu_fops = {
	.owner = THIS_MODULE,
	.read = mpu_read,
	.poll = mpu_poll,
	.unlocked_ioctl = mpu_dev_ioctl,
	.open = mpu_dev_open,
	.release = mpu_release,
};

int inv_mpu_register_slave_6050(struct module *slave_module,
			struct i2c_client *slave_client,
			struct ext_slave_platform_data *slave_pdata,
			struct ext_slave_descr *(*get_slave_descr)(void))
{
	struct mpu_private_data_6050 *mpu = mpu_private_data_6050;
	struct mldl_cfg *mldl_cfg;
	struct ext_slave_descr *slave_descr;
	struct ext_slave_platform_data **pdata_slave;
	char *irq_name = NULL;
	int result = 0;

	if (!slave_client || !slave_pdata || !get_slave_descr)
		return -EINVAL;

	if (!mpu) {
		dev_err(&slave_client->adapter->dev,
			"%s: Null mpu_private_data_6050\n", __func__);
		return -EINVAL;
	}
	mldl_cfg    = &mpu->mldl_cfg;
	pdata_slave = mldl_cfg->pdata_slave;
	slave_descr = get_slave_descr();

	if (!slave_descr) {
		dev_err(&slave_client->adapter->dev,
			"%s: Null ext_slave_descr\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&mpu->mutex);
	if (mpu->pid) {
		mutex_unlock(&mpu->mutex);
		return -EBUSY;
	}

	if (pdata_slave[slave_descr->type]) {
		result = -EBUSY;
		goto out_unlock_mutex;
	}

	slave_pdata->address	= slave_client->addr;
	slave_pdata->irq	= slave_client->irq;
	slave_pdata->adapt_num	= i2c_adapter_id(slave_client->adapter);

	dev_info(&slave_client->adapter->dev,
		"%s: +%s Type %d: Addr: %2x IRQ: %2d, Adapt: %2d\n",
		__func__,
		slave_descr->name,
		slave_descr->type,
		slave_pdata->address,
		slave_pdata->irq,
		slave_pdata->adapt_num);

	switch (slave_descr->type) {
	case EXT_SLAVE_TYPE_ACCEL:
		irq_name = "accelirq_6050";
		break;
	case EXT_SLAVE_TYPE_COMPASS:
		irq_name = "compassirq";
		break;
	case EXT_SLAVE_TYPE_PRESSURE:
		irq_name = "pressureirq";
		break;
	default:
		irq_name = "none";
	};
	if (slave_descr->init) {
		result = slave_descr->init(slave_client->adapter,
					slave_descr,
					slave_pdata);
		if (result) {
			dev_err(&slave_client->adapter->dev,
				"%s init failed %d\n",
				slave_descr->name, result);
			goto out_unlock_mutex;
		}
	}

	if (slave_descr->type == EXT_SLAVE_TYPE_ACCEL &&
	    slave_descr->id == ACCEL_ID_MPU6050 &&
	    slave_descr->config) {
		/* pass a reference to the mldl_cfg data
		   structure to the mpu6050 accel "class" */
		struct ext_slave_config config;
		config.key = MPU_SLAVE_CONFIG_INTERNAL_REFERENCE;
		config.len = sizeof(struct mldl_cfg *);
		config.apply = true;
		config.data = mldl_cfg;
		result = slave_descr->config(
			slave_client->adapter, slave_descr,
			slave_pdata, &config);
		if (result) {
			LOG_RESULT_LOCATION(result);
			goto out_slavedescr_exit;
		}
	}
	pdata_slave[slave_descr->type] = slave_pdata;
	mpu->slave_modules[slave_descr->type] = slave_module;
	mldl_cfg->slave[slave_descr->type] = slave_descr;

	goto out_unlock_mutex;

out_slavedescr_exit:
	if (slave_descr->exit)
		slave_descr->exit(slave_client->adapter,
				  slave_descr, slave_pdata);
out_unlock_mutex:
	mutex_unlock(&mpu->mutex);

	if (!result && irq_name && (slave_pdata->irq > 0)) {
		int warn_result;
		dev_info(&slave_client->adapter->dev,
			"Installing %s irq using %d\n",
			irq_name,
			slave_pdata->irq);
		warn_result = slaveirq_init_6050(slave_client->adapter,
					slave_pdata, irq_name);
		if (result)
			dev_WARN(&slave_client->adapter->dev,
				"%s irq assigned error: %d\n",
				slave_descr->name, warn_result);
	} else {
//ASUS_BSP +++ Jason Chang "disable warning"
/*
		dev_WARN(&slave_client->adapter->dev,
			"%s irq not assigned: %d %d %d\n",
			slave_descr->name,
			result, (int)irq_name, slave_pdata->irq);
*/
		printk("%s irq not assigned: %d %d %d\n",
			slave_descr->name,
			result, (int)irq_name, slave_pdata->irq);
//ASUS_BSP --- Jason Chang "disable warning"
	}

	return result;
}
EXPORT_SYMBOL(inv_mpu_register_slave_6050);

void inv_mpu_unregister_slave_6050(struct i2c_client *slave_client,
			struct ext_slave_platform_data *slave_pdata,
			struct ext_slave_descr *(*get_slave_descr)(void))
{
	struct mpu_private_data_6050 *mpu = mpu_private_data_6050;
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct ext_slave_descr *slave_descr;
	int result;

	dev_info(&slave_client->adapter->dev, "%s\n", __func__);

	if (!slave_client || !slave_pdata || !get_slave_descr)
		return;

	if (slave_pdata->irq)
		slaveirq_exit_6050(slave_pdata);

	slave_descr = get_slave_descr();
	if (!slave_descr)
		return;

	mutex_lock(&mpu->mutex);

	if (slave_descr->exit) {
		result = slave_descr->exit(slave_client->adapter,
					slave_descr,
					slave_pdata);
		if (result)
			dev_err(&slave_client->adapter->dev,
				"Accel exit failed %d\n", result);
	}
	mldl_cfg->slave[slave_descr->type] = NULL;
	mldl_cfg->pdata_slave[slave_descr->type] = NULL;
	mpu->slave_modules[slave_descr->type] = NULL;

	mutex_unlock(&mpu->mutex);

}
EXPORT_SYMBOL(inv_mpu_unregister_slave_6050);

static unsigned short normal_i2c[] = { I2C_CLIENT_END };

static const struct i2c_device_id mpu_id[] = {
	//{"mpu3050", 0},
	{"mpu6050", 0},
	{"mpu6050_no_accel", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, mpu_id);

int mpu_probe_6050(struct i2c_client *client, const struct i2c_device_id *devid)
{
	struct mpu_platform_data *pdata;
	struct mpu_private_data_6050 *mpu;
	struct mldl_cfg *mldl_cfg;
	int res = 0;
	int ii = 0;
printk("mpu_probe_6050+\n");
	dev_info(&client->adapter->dev, "%s: %d\n", __func__, ii++);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		res = -ENODEV;
		goto out_check_functionality_failed;
	}

	mpu = kzalloc(sizeof(struct mpu_private_data_6050), GFP_KERNEL);
	if (!mpu) {
		res = -ENOMEM;
		goto out_alloc_data_failed;
	}
	mldl_cfg = &mpu->mldl_cfg;
	mldl_cfg->mpu_ram	= &mpu->mpu_ram;
	mldl_cfg->mpu_gyro_cfg	= &mpu->mpu_gyro_cfg;
	mldl_cfg->mpu_offsets	= &mpu->mpu_offsets;
	mldl_cfg->mpu_chip_info	= &mpu->mpu_chip_info;
	mldl_cfg->inv_mpu_cfg	= &mpu->inv_mpu_cfg;
	mldl_cfg->inv_mpu_state	= &mpu->inv_mpu_state;

	mldl_cfg->mpu_ram->length = MPU_MEM_NUM_RAM_BANKS * MPU_MEM_BANK_SIZE;
	mldl_cfg->mpu_ram->ram = kzalloc(mldl_cfg->mpu_ram->length, GFP_KERNEL);
	if (!mldl_cfg->mpu_ram->ram) {
		res = -ENOMEM;
		goto out_alloc_ram_failed;
	}
	mpu_private_data_6050 = mpu;
	i2c_set_clientdata(client, mpu);
	mpu->client = client;

	init_waitqueue_head(&mpu->mpu_event_wait);
	mutex_init(&mpu->mutex);
	init_completion(&mpu->completion);

	mpu->response_timeout = 60;	/* Seconds */
	mpu->timeout.function = mpu_pm_timeout;
	mpu->timeout.data = (u_long) mpu;
	init_timer(&mpu->timeout);

	mpu->nb.notifier_call = mpu_pm_notifier_callback;
	mpu->nb.priority = 0;
	res = register_pm_notifier(&mpu->nb);
	if (res) {
		dev_err(&client->adapter->dev,
			"Unable to register pm_notifier %d\n", res);
		goto out_register_pm_notifier_failed;
	}

	pdata = (struct mpu_platform_data *)client->dev.platform_data;
	if (!pdata) {
		dev_WARN(&client->adapter->dev,
			 "Missing platform data for mpu\n");
	}
	mldl_cfg->pdata = pdata;
//ASUS_BSP +++ Jason Chang "dynamic change orientation"
mpu_get_orientation_6050();
asus_g_mldl_cfg_6050 = mldl_cfg;
//ASUS_BSP --- Jason Chang "dynamic change orientation"

//ASUS_BSP +++ Jason Chang "replace compass calibration by workqueue"
        INIT_DELAYED_WORK(&pad_plugin_work_6050, mpu_plugin_pad_work);
        INIT_DELAYED_WORK(&pad_plugout_work_6050, mpu_plugout_pad_work);
        if(!(pad_plug_wq_6050 = create_singlethread_workqueue("mpu_pad_plug_wq")))
            printk("[mpu] error: create pad plug workqueue failed\n");
//ASUS_BSP --- Jason Chang "replace compass calibration by workqueue"
	mldl_cfg->mpu_chip_info->addr = client->addr;
	res = inv_mpu_open_6050(&mpu->mldl_cfg, client->adapter, NULL, NULL, NULL);

	if (res) {
		dev_err(&client->adapter->dev,
			"Unable to open %s %d\n", MPU_NAME, res);
		res = -ENODEV;
		goto out_whoami_failed;
	}
//ASUS_BSP +++ Jason
/*if(!asus_check_mpu6050())
{
    printk("[mpu] gyro chip is not mpu6050, unload %s driver",MPU_NAME);
    goto out_misc_register_failed;
}*/
//ASUS_BSP --- Jason
	mpu->dev.minor = MISC_DYNAMIC_MINOR;
	mpu->dev.name = "mpu6050";
	mpu->dev.fops = &mpu_fops;
	res = misc_register(&mpu->dev);
	if (res < 0) {
		dev_err(&client->adapter->dev,
			"ERROR: misc_register returned %d\n", res);
		goto out_misc_register_failed;
	}

	if (client->irq) {
		dev_info(&client->adapter->dev,
			 "Installing irq using %d\n", client->irq);
		res = mpuirq_init_6050(client, mldl_cfg);
		if (res)
			goto out_mpuirq_failed;
	} else {
		dev_WARN(&client->adapter->dev,
			 "Missing %s IRQ\n", MPU_NAME);
	}
	if (!strcmp(mpu_id[0].name, devid->name)) {
		/* Special case to re-use the inv_mpu_register_slave */
		struct ext_slave_platform_data *slave_pdata;
		slave_pdata = kzalloc(sizeof(*slave_pdata), GFP_KERNEL);
		if (!slave_pdata) {
			res = -ENOMEM;
			goto out_slave_pdata_kzalloc_failed;
		}
		slave_pdata->bus = EXT_SLAVE_BUS_PRIMARY;
		for (ii = 0; ii < 9; ii++)
			slave_pdata->orientation[ii] = pdata->orientation[ii];
		res = inv_mpu_register_slave_6050(
			NULL, client,
			slave_pdata,
			mpu6050_get_slave_descr);
		if (res) {
			/* if inv_mpu_register_slave fails there are no pointer
			   references to the memory allocated to slave_pdata */
			kfree(slave_pdata);
			goto out_slave_pdata_kzalloc_failed;
		}
	}
//ASUS_BSP+++ Jason Chang "change oritation for pad"
        register_microp_notifier(&mpu_hs_notifier);
//ASUS_BSP--- Jason Chang "change oritation for pad"
//ASUS_BSP +++ Jason Chang "ATD Interface of 9-axis sensor"
	mpu_private_data_6050->attrs.attrs = mpu_6050_attr;
	res = sysfs_create_group(&client->dev.kobj, &mpu_private_data_6050->attrs);
	if (res) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
		printk("%s:Not able to create the sysfs\n", __FUNCTION__);
	}
	mpu_private_data_6050->gyro_status = 1;
//ASUS_BSP --- Jason Chang "ATD Interface of 9-axis sensor"
printk("mpu_probe_6050-\n");
	return res;

out_slave_pdata_kzalloc_failed:
	if (client->irq)
		mpuirq_exit_6050();
out_mpuirq_failed:
	misc_deregister(&mpu->dev);
out_misc_register_failed:
	inv_mpu_close_6050(&mpu->mldl_cfg, client->adapter, NULL, NULL, NULL);
out_whoami_failed:
	unregister_pm_notifier(&mpu->nb);
out_register_pm_notifier_failed:
	kfree(mldl_cfg->mpu_ram->ram);
	mpu_private_data_6050 = NULL;
out_alloc_ram_failed:
	kfree(mpu);
out_alloc_data_failed:
out_check_functionality_failed:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, res);
	return res;

}

static int mpu_remove(struct i2c_client *client)
{
	struct mpu_private_data_6050 *mpu = i2c_get_clientdata(client);
	struct i2c_adapter *slave_adapter[EXT_SLAVE_NUM_TYPES];
	struct mldl_cfg *mldl_cfg = &mpu->mldl_cfg;
	struct ext_slave_platform_data **pdata_slave = mldl_cfg->pdata_slave;
	int ii;

	for (ii = 0; ii < EXT_SLAVE_NUM_TYPES; ii++) {
		if (!pdata_slave[ii])
			slave_adapter[ii] = NULL;
		else
			slave_adapter[ii] =
				i2c_get_adapter(pdata_slave[ii]->adapt_num);
	}

	slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE] = client->adapter;
	dev_dbg(&client->adapter->dev, "%s\n", __func__);

	inv_mpu_close_6050(mldl_cfg,
		slave_adapter[EXT_SLAVE_TYPE_GYROSCOPE],
		slave_adapter[EXT_SLAVE_TYPE_ACCEL],
		slave_adapter[EXT_SLAVE_TYPE_COMPASS],
		slave_adapter[EXT_SLAVE_TYPE_PRESSURE]);

	if (mldl_cfg->slave[EXT_SLAVE_TYPE_ACCEL] &&
		(mldl_cfg->slave[EXT_SLAVE_TYPE_ACCEL]->id ==
			ACCEL_ID_MPU6050)) {
		struct ext_slave_platform_data *slave_pdata =
			mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_ACCEL];
		inv_mpu_unregister_slave_6050(
			client,
			mldl_cfg->pdata_slave[EXT_SLAVE_TYPE_ACCEL],
			mpu6050_get_slave_descr);
		kfree(slave_pdata);
	}

	if (client->irq)
		mpuirq_exit_6050();

	misc_deregister(&mpu->dev);

	unregister_pm_notifier(&mpu->nb);

	kfree(mpu->mldl_cfg.mpu_ram->ram);
	kfree(mpu);

//ASUS_BSP +++ Jason Chang "change oritation for pad"
        unregister_microp_notifier(&mpu_hs_notifier);
//ASUS_BSP --- Jason Chang "change oritation for pad"
//ASUS_BSP +++ Jason Chang "replace compass calibration by workqueue"
        destroy_workqueue(pad_plug_wq_6050);
//ASUS_BSP --- Jason Chang "replace compass calibration by workqueue"
	return 0;
}

static struct i2c_driver mpu_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = mpu_probe_6050,
	.remove = mpu_remove,
	.id_table = mpu_id,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "mpu6050",
		   },
	.address_list = normal_i2c,
	.shutdown = mpu_shutdown_6050,	/* optional */
	.suspend = mpu_dev_suspend_6050,	/* optional */
	.resume = mpu_dev_resume_6050,	/* optional */

};

static int __init mpu_init(void)
{
	int res = i2c_add_driver(&mpu_driver);
	pr_info("%s: Probe name %s\n", __func__, MPU_NAME);
	if (res)
		pr_err("%s failed\n", __func__);
	return res;
}

static void __exit mpu_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&mpu_driver);
}

module_init(mpu_init);
module_exit(mpu_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("User space character device interface for MPU");
MODULE_LICENSE("GPL");
MODULE_ALIAS(MPU_NAME);
