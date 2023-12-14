//--------------------------------------------------------------------
//                     ASUSTek Computer Inc.
//         Copyright (c) 2012 ASUSTek Computer inc, Taipei.
//
//			Fjm6mo ISP Device
//--------------------------------------------------------------------
//File: fjm6mo.c
//Revision History:
//[2012.01.09]	LiJen_Chang created.

#include "fjm6mo.h"
#include <linux/debugfs.h>
#include <linux/proc_fs.h>  //ASUS_BSP  LiJen "[A66][8M][NA][Others]add proc file fo AP ISP update

#include <linux/gpio.h>

// ASUS_BSP +++ LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode"
#ifdef WITH_WQ
#include <linux/workqueue.h>
static struct workqueue_struct *g_fjm6mo_wq;
static struct fjm6mo_work g_fjm6mo_work_mon;
static struct fjm6mo_work g_fjm6mo_work_init;
static unsigned char g_fjm6mo_wq_start;
#endif // end of WITH_WQ
// ASUS_BSP --- LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode"

extern struct msm_sensor_ctrl_t ov2720_s_ctrl;
extern int g_rt_for_monitor;

#define FLASH_GPIO 68 

//ASUS_BSP +++ LiJen "[A66][8M][NA][Others]add proc file fo AP ISP update
// create proc file
#ifdef	CONFIG_PROC_FS
#define	FJM6MO_PROC_FILE	"driver/fjm6mo"
static struct proc_dir_entry *fjm6mo_proc_file;

//ASUS_BSP+++ Patrick "[A66][8M][NA][Others]add proc file for firmware version"
#define	FJM6MO_FIRMWARE_VERSION_PROC_FILE	"driver/isp_fw_version"
#define	FJM6MO_GOLDEN_VALUE_PROC_FILE		"driver/fjm6mo_GV"
static struct proc_dir_entry *fjm6mo_fw_version_proc_file;
static struct proc_dir_entry *fjm6mo_golden_value_proc_file;

static ssize_t fjm6mo_golden_value_proc_read(char *page, char **start, off_t off, int count,
            	int *eof, void *data)
{
	int len=0;
	if(*eof == 0){
		if(count > 22) {	// golden value string length 22
			char calibrationGoldenValue[4] = {0xFF, 0xFF, 0xFF, 0xFF};
			u32 status;

			calibrationGoldenValue[0] = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x02, 0x3A, 0x01, &status);
			if(status == 0xFF) {
				*eof = 1;
				len=-1;
				pr_err("%s:X read golden string 3A fail", __func__);
				return len;
			}
			calibrationGoldenValue[1] = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x02, 0x3B, 0x01, &status);
			calibrationGoldenValue[2] = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x02, 0x3C, 0x01, &status);
			if(status == 0xFF) {
				*eof = 1;
				len=-1;
				pr_err("%s:X read golden string 3C fail", __func__);
				return len;
			}
			calibrationGoldenValue[3] = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x02, 0x3D, 0x01, &status);
			len+=sprintf(page+len, "0x%X 0x%X 0x%X 0x%X\n",
				calibrationGoldenValue[0], calibrationGoldenValue[1], calibrationGoldenValue[2], calibrationGoldenValue[3]);
			pr_info("%s:X string=%s", __func__, (char *)page);
		} else {
			len=-1;
			pr_err("%s:X string length less than golden value length", __func__);
		}
		*eof = 1;
	}
	  return len;
}

static ssize_t fjm6mo_fw_version_proc_read(char *page, char **start, off_t off, int count,
            	int *eof, void *data)
{
	int len=0;
	if(*eof == 0){
		if(count>8) {	// version number string length 8
			len+=sprintf(page+len, "%x\n", VersionNum);
			pr_info("%s:X string=%s", __func__, (char *)page);
		} else {
			len=-1;
		}
		*eof = 1;
	}
	  return len;
}

static ssize_t fjm6mo_fw_version_proc_write(struct file *filp, const char __user *buff,
	            unsigned long len, void *data)
{
	pr_info("fjm6mo write proc file\n");
	return len;
}
//ASUS_BSP--- Patrick "[A66][8M][NA][Others]add proc file for firmware version"
static ssize_t fjm6mo_proc_read(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	int len=0;

	if(*eof == 0){
		u32 BinVersionNum = 0;
		int ret = 0;
		ret = bin_fw_version(&BinVersionNum);

		if(ret <= 0 ) {
			len+=sprintf(page+len, "%x %x\nE:%d\n", UPDATE_UNNECESSARY, VersionNum, ret);
		} else {
			pr_info("VersionNum=%x, BinVersionNum=%x--\n", VersionNum, BinVersionNum);
			if((VersionNum==0xffffff)||(VersionNum<BinVersionNum)) {
				len+=sprintf(page+len, "%x %x\n%x\n", NEED_UPDATE, VersionNum, BinVersionNum);
			}else{
				len+=sprintf(page+len, "%x %x\n%x\n", UPDATE_UNNECESSARY, VersionNum, BinVersionNum);
			}
		}
		*eof = 1;
		pr_info("%s:X string=%s", __func__, (char *)page);
	}
	return len;
}

static ssize_t fjm6mo_proc_write(struct file *filp, const char __user *buff, 
	            unsigned long len, void *data)
{
	static char messages[256]="";

	if (len > 256)
		len = 256;

	memset(messages, 0, 256);
	if (copy_from_user(messages, buff, len))
		return -EFAULT;
        
	pr_info("fjm6mo_proc_write %s\n", messages);

	if (strlen(messages)<=0) {
	     pr_info("command not support\n");
	} else {
		struct file *fp = NULL;
		int str_len = strlen(messages);
		messages[str_len-1] = 0;

		mutex_lock(ov2720_s_ctrl.msm_sensor_mutex);

		pr_info("test filp_open %s--\n", messages);
		fp = filp_open(messages, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
		if ( !IS_ERR_OR_NULL(fp) ){
			UPDATE_FILE_WITH_PATH = messages;
			filp_close(fp, NULL);
		} else {
			pr_info("choose system A60K_RS_M6Mo.bin\n");
			UPDATE_FILE_WITH_PATH = BIN_FILE_WITH_PATH;
		}

		/* Update ISP firmware*/
            pr_info("fjm6mo firmware update start\n");
            fw_update();

		mutex_unlock(ov2720_s_ctrl.msm_sensor_mutex);
	}

	return len;
}

void create_fjm6mo_proc_file(void)
{
    fjm6mo_proc_file = create_proc_entry(FJM6MO_PROC_FILE, 0666, NULL);
    if (fjm6mo_proc_file) {
		fjm6mo_proc_file->read_proc = fjm6mo_proc_read;
		fjm6mo_proc_file->write_proc = fjm6mo_proc_write;
    } else{
        pr_err("proc file create failed!\n");
    }
//ASUS_BSP+++ Patrick "[A66][8M][NA][Others]add proc file for firmware version"
    fjm6mo_fw_version_proc_file = create_proc_entry(FJM6MO_FIRMWARE_VERSION_PROC_FILE, 0666, NULL);
    if (fjm6mo_fw_version_proc_file) {
		fjm6mo_fw_version_proc_file->read_proc = fjm6mo_fw_version_proc_read;
		fjm6mo_fw_version_proc_file->write_proc = fjm6mo_fw_version_proc_write;
    } else{
        pr_err("proc file fjm6mo fw version create failed!\n");
    }
    fjm6mo_golden_value_proc_file = create_proc_entry(FJM6MO_GOLDEN_VALUE_PROC_FILE, 0666, NULL);
    if (fjm6mo_golden_value_proc_file) {
		fjm6mo_golden_value_proc_file->read_proc = fjm6mo_golden_value_proc_read;
		fjm6mo_golden_value_proc_file->write_proc = fjm6mo_fw_version_proc_write;
    } else{
        pr_err("proc file fjm6mo fw version create failed!\n");
    }
//ASUS_BSP--- Patrick "[A66][8M][NA][Others]add proc file for firmware version"
}

void remove_fjm6mo_proc_file(void)
{
    extern struct proc_dir_entry proc_root;
    pr_info("fjm6mo_proc_file\n");	
    remove_proc_entry(FJM6MO_PROC_FILE, &proc_root);
}
#endif // end of CONFIG_PROC_FS
//ASUS_BSP --- LiJen "[A66][8M][NA][Others]add proc file fo AP ISP update"
