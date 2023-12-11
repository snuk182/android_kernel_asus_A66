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

// ASUS_BSP+++ Patrick "[A60K][8M][NA][Others] ISP firmware update"
#define NEED_UPDATE				0x0
#define UPDATE_UNNECESSARY	0x1

static char  BIN_FILE_WITH_PATH[] = "/system/etc/firmware/camera/A60K-RS-M6Mo.bin";
#define FILE_ACCESS_SUCCESS	1
#define FILE_NOT_FOUND		-1
#define FILE_OPEN_FAIL		-2
#define FILE_NOT_CORRECT		-3

#define VERSION_STATUS_LENGTH	32
#define ISP_FW_UPDATE_SUCCESS	"0"
#define ISP_CHECKSUM_FAIL		"1"
#define ISP_FW_UPDATE_INTERRUPT	"2"

static u32 VersionNum = 0xffffffff;
static int fjm6mo_update_status = 0;

static char* UPDATE_FILE_WITH_PATH;
// ASUS_BSP--- Patrick "[A60K][8M][NA][Others] ISP firmware update"

// ASUS_BSP+++ Patrick "[ov2720] add calibration functionalities"
#ifdef ASUS_FACTORY_BUILD
#define CALIBRATION_ISP_POWERON_FAIL	1
#define CALIBRATION_ISP_INIT_FAIL		2
#define CALIBRATION_ISP_MONITOR_FAIL	3
#define CALIBRATION_LIGHT_SOURCE_FAIL	4
#define CALIBRATION_LIGHT_SOURCE_OK	5
#define CALIBRATION_ISP_CAPTURE_FAIL	6
#define CALIBRATION_ISP_CHECKSUM_FAIL	7
#define CALIBRATION_ISP_PGAIN_FAIL		8
#define CALIBRATION_ISP_GOLDEN_FAIL	9
#define CALIBRATION_ISP_OK				10

#define PGAIN_VARIANT 3	//3%

static unsigned char golden_value[4] = {0xFF, 0xFF, 0xFF, 0xFF};
static int fjm6mo_calibration_status = 0;
static u32 Golden_R_Gain=0x00000000, Golden_B_Gain=0x00000000;
#endif //ASUS_FACTORY_BUILD

static unsigned int shading_table = 0;
// ASUS_BSP--- Patrick "[ov2720] add calibration functionalities"

#define MAX_LOOPS_RETRIES 50
#define MAX_ERASE_ROM_RETRIES 100
#define FLASH_ROM_ADDRESS         0x10000000
#define FLASH_ROM_FACT_ADDRESS    0x101F8000
#define FJM6MO_RAM_ADDRESS        0x68000000
#define FJM6MO_SECTOR_SIZE_TEMP   0x10000
#define FJM6MO_SECTOR_SIZE        0x0000
#define FJM6MO_SECTOR_SIZE_8      0x2000
#define FJM6MO_SECTOR_SIZE_1k     0x0200
#define FJM6MO_CHECKSUM_SIZE      0x8000

#define FJM6MO_PARA_STATUS_ERR    0
#define FJM6MO_PARA_STATUS_PAR    0x01
#define FJM6MO_PARA_STATUS_MON    0x02
#define FJM6MO_PARA_STATUS_CAP    0x03
// ASUS_BSP +++ LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode"
#ifdef WITH_WQ
#define FJM6MO_PARA_STATUS_INT    0x04
#endif // end of WITH_WQ
// ASUS_BSP --- LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode"

#define FJM6MO_FLAG_USE_CAPTURE (0x1U << 0)
#define FJM6MO_FLAG_DEBUG       (0x1U << 31)
#define SENSOR_MAX_RETRIES      3 /* max counter for retry I2C access */
#define DBG_TXT_BUF_SIZE 256

#define WAIT_FOR_CHANGE_CAPTURE_DONE	20

static unsigned int s_flag = FJM6MO_FLAG_USE_CAPTURE;
static int fjm6mo_status = 0;
static char debugTxtBuf[DBG_TXT_BUF_SIZE];

static u8 fw_chip_erase_pin1[16] = {
    0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x1f,
    0x76, 0x00, 0x18, 0x00, 0x00, 0x00, 0xff, 0xff
};
static u8 fw_chip_erase_pin2[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static u8 fw_chip_erase_pin3[16] = {
    0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x3f,
    0xff, 0x00, 0x20, 0x11, 0x00, 0x00, 0xff, 0xff
};

//static bool capture_mode = false;
static bool factory_mode = true;
static bool caf_mode = false;

enum {
    INT_STATUS_MODE = 0x01,
    INT_STATUS_AF = 0x02,
    INT_STATUS_ZOOM = 0x04,
    INT_STATUS_CAPTURE = 0x08,
    INT_STATUS_FRAMESYNC = 0x10,
    INT_STATUS_FD = 0x20,
    INT_STATELENS_INIT = 0x40,
    INT_STATUS_SOUND = 0x80,
};

static int fjm6mo_write_memory(struct i2c_client *client, u8* send_buf, u32 byte_size, u32 addr, u32 write_size)
{
    int err, retry = 0;
    struct i2c_msg msg;
    unsigned char data[600];
    memset(data, 66, 600);

    if (!client->adapter)
        return -ENODEV;

    data[0] = 0x0;
    data[1] = 4;
    data[2] = (u8)((addr & 0xFF000000) >> 24);
    data[3] = (u8)((addr & 0x00FF0000) >> 16);
    data[4] = (u8)((addr & 0x0000FF00) >>  8);
    data[5] = (u8)( addr & 0x000000FF);
    data[6] = (u8)((write_size & 0xFF00) >> 8);
    data[7] = (u8)(write_size & 0x00FF);

//    pr_info("addr:%x %x %x %x\n", data[2], data[3], data[4], data[5]);

    memcpy(data+8, send_buf, write_size);

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = 8 + write_size;
    msg.buf = data;

    do {
        err = i2c_transfer(client->adapter, &msg, 1);
        if (err == 1)
            return 0;
        retry += 1;
        pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n", addr, write_size);
        pr_err("yuv_sensor : i2c transfer failed, count %x \n", msg.addr);
    } while (retry <= SENSOR_MAX_RETRIES);

    return 0;
}
static int fjm6mo_read_memory(struct i2c_client *client, u32 addr, u32 read_size, u32* val)
{
    int err;
    struct i2c_msg msg[2];
    unsigned char data[600];
    memset(data, 66, 600);

    if (!client->adapter)
        return -ENODEV;

    data[0] = 0x0;
    data[1] = 3;
    data[2] = (u8)((addr & 0xFF000000) >> 24);
    data[3] = (u8)((addr & 0x00FF0000) >> 16);
    data[4] = (u8)((addr & 0x0000FF00) >>  8);
    data[5] = (u8)( addr & 0x000000FF);
    data[6] = (u8)((read_size & 0xFF00) >> 8);
    data[7] = (u8)(read_size & 0x00FF);

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 8;
    msg[0].buf = data;

    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = (u16)(read_size & 0xFFFF) + 3;
    msg[1].buf = data + 8;

    err = i2c_transfer(client->adapter, msg, 2);
    if (err != 2)
        return -EINVAL;

    memcpy(val, data+11, read_size);

    return 0;
}

static int fjm6mo_write_register(struct i2c_client *client, u8 byte_num, u8 cat, u8 byte, u32 val)
{
    int err;
    struct i2c_msg msg;
    unsigned char data[32];
    int retry = 0;
    memset(data, 66, 32);

	pr_info("ISP w-reg [Byte_num=0x%X,CAT=0x%X,Byte=0x%X,val=0x%X]", byte_num, cat, byte, val);

    if (!client->adapter)
        return -ENODEV;

    data[1] = 2;
    data[2] = cat;
    data[3] = byte;

    switch(byte_num)
    {
        case 1:
        {
            data[0] = 5;
            data[4] = (u8)(val & 0xff);
            break;
        }
        case 2:
        {
            data[0] = 6;
            data[4] = (u8)((val & 0xff00) >> 8);
            data[5] = (u8)(val & 0x00ff);
            break;
        }
        case 4:
        {
            data[0] = 8;
            data[4] = (u8)((val & 0xff000000) >> 24);
            data[5] = (u8)((val & 0x00ff0000) >> 16);
            data[6] = (u8)((val & 0x0000ff00) >> 8);
            data[7] = (u8)(val & 0x000000ff);
            break;
        }
        default:
            break;
    }

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = data[0];
    msg.buf = data;

//    pr_info("data[0]:0x%x, data[4]:0x%x\n", data[0], data[4]);

    do {
        err = i2c_transfer(client->adapter, &msg, 1);
        if (err == 1)
            return 0;
        retry++;
        pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n", cat, byte);
        pr_err("yuv_sensor : i2c transfer failed, count %x \n", msg.addr);
    } while (retry <= SENSOR_MAX_RETRIES);

    return err;
}

static int fjm6mo_read_register(struct i2c_client *client, u8 cat, u8 byte, u8 byte_num, u32* val)
{
    u32 err=0, ret=0;
    struct i2c_msg msg[2];
    unsigned char data[37];

    memset(data, 66, 37);

    if (!client->adapter)
        return -ENODEV;

    do{
        data[0] = 5;
        data[1] = 1;
        data[2] = cat;
        data[3] = byte;
        data[4] = byte_num;
        data[5] = 0;

        msg[0].addr = client->addr;
        msg[0].flags = 0;
        msg[0].len = data[0];
        msg[0].buf = data;

        msg[1].addr = client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = byte_num + 1;
        msg[1].buf = data + 5;

        err = i2c_transfer(client->adapter, msg, 2);
//        pr_info("!!!!!!read register: data[5]:0x%x data[6]:0x%x!!!!!!\n", data[5], data[6]);
        if (err != 2)
            return -EINVAL;
    }while(data[5] == 0xfa || data[5] == 0xf2);

    switch(byte_num)
    {
        case 1:
        {
            ret = data[6];
            memcpy(val, data+6, byte_num);
            *val = *val & 0xff;
            pr_info("read register: cat:0x%x byte:0x%x data[5]:0x%x data[6]:0x%x\n", data[2], data[3], data[5], data[6]);
            break;
        }
        case 2:
        {
            ret  = data[6] << 8;
            ret += data[7];
            swap(*(data+6),*(data+7));
            memcpy(val, data+6, byte_num);
            *val = *val & 0xffff;
            pr_info("read register: cat:0x%x byte:0x%x data[5]:0x%x data[6]:0x%x data[7]:0x%x val:0x%x ret:0x%x\n ", data[2], data[3], data[5], data[6], data[7], *val, ret);
            break;
        }
        case 4:
        {
            ret  = data[6] << 24;
            ret += data[7] << 16;
            ret += data[8] << 8;
            ret += data[9];
            swap(*(data+6),*(data+9));
            swap(*(data+7),*(data+8));
            memcpy(val, data+6, byte_num);
            *val = *val & 0xffffffff;
            break;
        }
        default:
            break;
    }

    return ret;
}

static int fw_initial_purpose(void)
{
    int err = 0;

    pr_info("fjm6mo fw_initial_purpose\n");

    err = fjm6mo_write_memory(ov2720_s_ctrl.sensor_i2c_client->client, fw_chip_erase_pin1, 1, 0x50000300,  0x0010);
    if(err)
        return err;
    err = fjm6mo_write_memory(ov2720_s_ctrl.sensor_i2c_client->client, fw_chip_erase_pin2, 1, 0x50000100,  0x0010);
    if(err)
        return err;
    err = fjm6mo_write_memory(ov2720_s_ctrl.sensor_i2c_client->client, fw_chip_erase_pin3, 1, 0x50000200,  0x0010);
    if(err)
        return err;
    return 0;
}

static int fw_rom_erase(u32 start_rom_address)
{
    u32 status;
    u8 result = 0x01;
    int err, count = 0;

    pr_info("fjm6mo fw_rom_erase set address: 0x%x\n", start_rom_address);
    err = fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 4, 0x0f, 0x00, start_rom_address);
    if(err){
        pr_err("fw_rom_erase write_register 0x0f, 0x00 error ");
        return -ENOMEM;
    }
    err = fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0f, 0x06, 0x02);
    if(err){
        pr_err("fw_rom_erase 0x0f, 0x06 error");
        return -ENOMEM;
    }
    do{
        count++;
        msleep(500);
        result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0f, 0x06, 0x01, &status);
    } while ((result != 0x00)&&(count<MAX_ERASE_ROM_RETRIES));

	if(count>=MAX_ERASE_ROM_RETRIES)
		return true;
	else
		return false;
}

static int checksum_read_register(struct i2c_client *client, u8 cat, u8 byte, u8 byte_num, u32* val)
{
    u32 err=0, ret=0;
    struct i2c_msg msg[2];
    unsigned char data[37];

    memset(data, 66, 37);

    if (!client->adapter)
        return -ENODEV;

    do{
        data[0] = 5;
        data[1] = 1;
        data[2] = cat;
        data[3] = byte;
        data[4] = byte_num;
        data[5] = 0;

        msg[0].addr = client->addr;
        msg[0].flags = 0;
        msg[0].len = data[0];
        msg[0].buf = data;

        msg[1].addr = client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = byte_num + 1;
        msg[1].buf = data + 5;

        err = i2c_transfer(client->adapter, msg, 2);
//        pr_info("!!!!!!read register: data[5]:0x%x data[6]:0x%x!!!!!!\n", data[5], data[6]);
        if (err != 2)
            return -EINVAL;
    }while(data[5] == 0xfa || data[5] == 0xf2);

    switch(byte_num)
    {
        case 1:
        {
            ret = data[6];
            memcpy(val, data+6, byte_num);
            *val = *val & 0xff;
//            pr_info("read register: cat:0x%x byte:0x%x data[5]:0x%x data[6]:0x%x\n", data[2], data[3], data[5], data[6]);
            break;
        }
        case 2:
        {
            ret  = data[6] << 8;
            ret += data[7];
            swap(*(data+6),*(data+7));
            memcpy(val, data+6, byte_num);
            *val = *val & 0xffff;
//            pr_info("read register: cat:0x%x byte:0x%x data[5]:0x%x data[6]:0x%x data[7]:0x%x val:0x%x ret:0x%x\n ",
//							data[2], data[3], data[5], data[6], data[7], *val, ret);
            break;
        }
        case 4:
        {
            ret  = data[6] << 24;
            ret += data[7] << 16;
            ret += data[8] << 8;
            ret += data[9];
            swap(*(data+6),*(data+9));
            swap(*(data+7),*(data+8));
            memcpy(val, data+6, byte_num);
            *val = *val & 0xffffffff;
            break;
        }
        default:
            break;
    }

    return ret;
}
static int fw_checksum(u32* sum)
{
    u32 start_rom_address = FLASH_ROM_ADDRESS;
    u32 count = 0x001f8000 / FJM6MO_CHECKSUM_SIZE;
    u32 i = 0, j =0;
    u32 status, result;
    u16 ret_sum = 0;

    for(i = 0; i < count ; i += 1){
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 4, 0x0f, 0x00, start_rom_address);
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0f, 0x04, FJM6MO_CHECKSUM_SIZE);
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0f, 0x09, 0x02);
        for(j = 0; j < MAX_LOOPS_RETRIES ; j +=1){
            msleep(5);
            result = checksum_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0f, 0x09, 0x01, &status);
            if(status == 0x00){
                checksum_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0f, 0x0A, 0x02, &result);
                ret_sum += result;
                break;
            }
        }
        start_rom_address += FJM6MO_CHECKSUM_SIZE;
    }

    *sum = ret_sum;

    pr_info("fw_checksum:0x%x\n", ret_sum);

    return 0;
}

static int bin_checksum(u32* sum)
{
    u32 start_rom_address = FLASH_ROM_ADDRESS;
//    u32 count = 0x001f8000 / FJM6MO_CHECKSUM_SIZE;
    u16 result = 0;
    u32 write_size = FJM6MO_SECTOR_SIZE_1k;
    u16 temp_result = 0;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    char *pFile = BIN_FILE_WITH_PATH;
    u8 buf[FJM6MO_SECTOR_SIZE_1k];
    int line_count = 0, times = 0;

    pr_info("filp_open RS_M6Mo.bin\n");
    fp = filp_open(pFile, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
    memset(buf, 66, write_size);
    if ( !IS_ERR_OR_NULL(fp) ){
        pr_info("filp_open success fp:%p\n", fp);
        old_fs = get_fs();
        set_fs(KERNEL_DS);
        if(fp->f_op != NULL && fp->f_op->read != NULL){
            pr_info("file RS_M6Mo.bin start to read!\n");
            while (fp->f_op && fp->f_op->read) {
                fp->f_op->read(fp, buf, write_size, &fp->f_pos);
                for(times = 0; times < write_size ; times += 2){
                    temp_result = (buf[times] << 8) + buf[times+1];
                    result += temp_result;
                }
                line_count += 1;
                if(line_count % 64 == 0)
                    pr_info("start_rom_address:0x%x result=0x%x\n ", start_rom_address, result);
                start_rom_address += write_size;
                if(start_rom_address >= 0x101f8000)
                    break;
            }
        }
        set_fs(old_fs);
        filp_close(fp, NULL);
    }
    else if(PTR_ERR(fp) == -ENOENT){
        pr_err("file RS_M6Mo.bin not found error\n");
        return -ENOMEM;
    }
    else{
        pr_err("file RS_M6Mo.bin open error\n");
        return -ENOMEM;
    }

    *sum = result;

    pr_info("bin_checksum:0x%x\n", result);

    return 0;

}

int fw_version(void)
{
	int err=0;
	u8 val[4] = {0x00, 0x00, 0x00, 0x00};

	err = fjm6mo_read_memory(ov2720_s_ctrl.sensor_i2c_client->client, 0x1016FFFC, 2, (u32*)val);
	if(err == 0) {
		VersionNum = val[0];
		VersionNum = VersionNum << 8;
		VersionNum += val[1];
		VersionNum = VersionNum << 8;
//		pr_info("Camera ISP firmware version number1 0x%x, val=%x-%x-%x-%x\n", VersionNum, val[0], val[1], val[2], val[3]);
		err = fjm6mo_read_memory(ov2720_s_ctrl.sensor_i2c_client->client, 0x1016FFE4, 1, (u32*)val);
		if(err == 0) {
			VersionNum += val[0];

			pr_info("Camera ISP firmware version number 0x%x\n", VersionNum);
//			pr_info("Camera ISP firmware version number2 0x%x, val=%x-%x-%x-%x\n", VersionNum, val[0], val[1], val[2], val[3]);
		} else {
			pr_info("Camera ISP memory read sub-version fail!!\n");
                     err = -1;
		}
	} else {
		pr_info("Camera ISP memory read version fail!!\n");
              err = -1;
	}
	return err;
}

int bin_fw_version(u32* BinVersionNum)
{
	u32 start_rom_address = FLASH_ROM_ADDRESS;
	u32 write_size = FJM6MO_SECTOR_SIZE_1k;
	u8 buf[FJM6MO_SECTOR_SIZE_1k];

	struct file *fp = NULL;
	char *pFile = BIN_FILE_WITH_PATH;
	mm_segment_t old_fs;

//	pr_info("filp_open %s\n",pFile);
	fp = filp_open(pFile, O_RDONLY, 0);
	if( !IS_ERR_OR_NULL(fp) ) {
		memset(buf, 0, write_size);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(fp->f_op != NULL && fp->f_op->read != NULL) {
//			pr_info("file RS_M6Mo.bin start to read!\n");
			while (fp->f_op && fp->f_op->read) {
				if(fp->f_op && fp->f_op->read) {
					fp->f_op->read(fp, buf, write_size, &fp->f_pos);
				} else {
					pr_err("Read RS_M6Mo.bin fail\n");
					break;
				}
				start_rom_address += write_size;

				if(start_rom_address >= 0x10170000)
					break;
			}
		}
		set_fs(old_fs);
		filp_close(fp, NULL);

		if(start_rom_address >= 0x10170000) {
//			pr_info("Version:%x%x, %s--\n", buf[0x1FC], buf[0x1FD], &buf[0x1FC]);
			*BinVersionNum = buf[0x1FC];
			*BinVersionNum = *BinVersionNum << 8;
			*BinVersionNum += buf[0x1FD];
			*BinVersionNum = *BinVersionNum << 8;
//			pr_info("Version:%x, %s--\n", buf[0x1E4], &buf[0x1E4]);
			*BinVersionNum += buf[0x1E4];
		} else {
			pr_err("Not RS_M6Mo.bin?\n");
			return FILE_NOT_CORRECT;
		}
	} else if(PTR_ERR(fp) == -ENOENT) {
		pr_err("file RS_M6Mo.bin not found error\n");
		return FILE_NOT_FOUND;
	} else {
		pr_err("file RS_M6Mo.bin open error\n");
		return FILE_OPEN_FAIL;
	}
	return FILE_ACCESS_SUCCESS;
}

static int check_after_update(u32 start_rom_address, char* Progress_file_path)
{
	char update_result_file_path[] = "/data/fw_update_result";
	struct file *filePtr = NULL;
	int len = 0;
	mm_segment_t old_fs;
	loff_t pos = 0;
	u32 RomCheckSum = 0;

	//Show new version
	fw_version();

	//Verify checksum
	fw_checksum(&RomCheckSum);

	if(RomCheckSum!=0) {
		u32 tmp_rom_address = 0x10160000;
		int err=0;

		pr_info("ISP firmware update checksum NOT correct, erase 0x%x firmware\n", tmp_rom_address);
		err = fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 4, 0x0f, 0x00, tmp_rom_address);
		if(err) {
			pr_err("ISP firmware update checksum NOT correct, write_register 0x0f, 0x00 error");
		} else {
			err = fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0f, 0x06, 0x01);
			if(err) {
				pr_err("ISP firmware update checksum NOT correct, write_register 0x0f, 0x06 error");
			} else {
				int retry_count = 0;
				u8 result = 0x01;
				u32 status;

				do {
					retry_count++;
					msleep(500);
					result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0f, 0x06, 0x01, &status);
				} while ((result != 0x00)&&(retry_count<MAX_LOOPS_RETRIES));
				if(result != 0x00)
					pr_err("goto update_read_retry_fail;\n");
			}
		}

		filePtr = filp_open(update_result_file_path, O_RDWR, S_IRWXU|S_IRWXG|S_IRWXO);

		if(!IS_ERR_OR_NULL(filePtr)) {
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			pos = 0;

			len = filePtr->f_op->write(filePtr, ISP_CHECKSUM_FAIL, sizeof(char), &pos);

			set_fs(old_fs);
			filp_close(filePtr, NULL);
			pr_info("Write ISP_CHECKSUM_FAIL in %s length=%d\n", update_result_file_path, len);
		} else {
			pr_err("Write update result in %s Fail!!\n", update_result_file_path);
		}
	} else if(start_rom_address >= 0x101f8000) {
		struct file *ProgressFilePtr = NULL;
		pr_info("ISP firmware checksum is correct\n");

		ProgressFilePtr = filp_open(Progress_file_path, O_RDWR, S_IRWXU|S_IRWXG|S_IRWXO);

		if(!IS_ERR_OR_NULL(ProgressFilePtr)) {
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			pos = 0;

			len = ProgressFilePtr->f_op->write(ProgressFilePtr, "100", 3*sizeof(char), &pos);
			set_fs(old_fs);
			filp_close(ProgressFilePtr, NULL);
			pr_err("Writing update progress result correct\n");
		} else {
			pr_err("Writing update progress result in %s Fail!!\n", Progress_file_path);
		}

		filePtr = filp_open(update_result_file_path, O_RDWR, S_IRWXU|S_IRWXG|S_IRWXO);
		if(!IS_ERR_OR_NULL(filePtr)) {
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			pos = 0;

			len = filePtr->f_op->write(filePtr, ISP_FW_UPDATE_SUCCESS, sizeof(char), &pos);
			set_fs(old_fs);
			filp_close(filePtr, NULL);
			pr_info("Write ISP_FW_UPDATE_SUCCESS in %s length=%d\n", update_result_file_path, len);
		} else {
			pr_err("Write update result in %s Fail!!\n", update_result_file_path);
		}
	} else {
		pr_info("ISP firmware NOT write to end?? start_rom_address=0x%x--\n", start_rom_address);
		filePtr = filp_open(update_result_file_path, O_RDWR, S_IRWXU|S_IRWXG|S_IRWXO);

		if(!IS_ERR_OR_NULL(filePtr)) {
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			pos = 0;

			len = filePtr->f_op->write(filePtr, ISP_FW_UPDATE_INTERRUPT, sizeof(char), &pos);

			set_fs(old_fs);
			filp_close(filePtr, NULL);
			pr_info("Write ISP_FW_UPDATE_INTERRUPT in %s length=%d\n", update_result_file_path, len);
		} else {
			pr_err("Write update result in %s Fail!!\n", update_result_file_path);
		}
	}
	return 0;
}
static int fw_update(void)
{
    u32 status;
    u32 start_rom_address = FLASH_ROM_ADDRESS;
    u32 start_ram_address = FJM6MO_RAM_ADDRESS;
    u32 write_size = FJM6MO_SECTOR_SIZE_1k;
    u32 sector_size = FJM6MO_SECTOR_SIZE;
    u16 i = 0;
    u8 result = 0x01;

    struct file *fp = NULL;
    mm_segment_t old_fs;
    u8 buf[FJM6MO_SECTOR_SIZE_1k];
    int line_count = 0;
	int err;
	int retry_count = 0;
	loff_t temp_pos = 0;
	u32 temp_rom_address = 0;

	int update_progress=1, write_file_ret=0, update_progress_file=0;
	char Progress_file_path[] = "/data/isp_fw_update_progress";
	struct file *filePtr = NULL;
	loff_t pos = 0;
	char temp_progress[4];

    pr_info("%s +++\n",__func__);

    //power on
    err = ov2720_s_ctrl.func_tbl->sensor_power_up(&ov2720_s_ctrl);
    if(err){
		pr_err("isp power on error \n");
		goto update_fail;
    }

	pr_info("Camera ISP firmware version number 0x%x %d\n", VersionNum, fjm6mo_status);

//    wake_lock_init(&fjm6mo_wake_lock, WAKE_LOCK_SUSPEND, "fjm6mo_wake");
 //   wake_lock(&fjm6mo_wake_lock);

    //Set M6MO pin(1) pin(2) pin(3)
    err = fw_initial_purpose();
    if(err){
        pr_err("fw_initial_purpose error");
        goto update_fail;
    }

    pr_info("filp_open %s\n", UPDATE_FILE_WITH_PATH);
    fp = filp_open(UPDATE_FILE_WITH_PATH, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
    memset(buf, 66, write_size);
    if ( !IS_ERR_OR_NULL(fp) ){
//        pr_info("filp_open success fp:%p\n", fp);
        old_fs = get_fs();
        set_fs(KERNEL_DS);
        if(fp->f_op != NULL && fp->f_op->read != NULL){
            pr_info("file RS_M6Mo.bin start to read!\n");
            while (fp->f_op && fp->f_op->read) {
                fp->f_op->read(fp, buf, write_size, &fp->f_pos);
                line_count += 1;
                start_rom_address += write_size;
                if(start_rom_address >= 0x101f8000)
                    break;
            }
        }
        set_fs(old_fs);
        filp_close(fp, NULL);
    }
    else if(PTR_ERR(fp) == -ENOENT){
        pr_err("file %s not found error\n", UPDATE_FILE_WITH_PATH);
        goto update_fail;
    }
    else{
        pr_err("file %s open error\n", UPDATE_FILE_WITH_PATH);
        goto update_fail;
    }

// ASUS_BSP+++ Patrick "[A60K][8M][NA][Spec] add ISP firmware progress file checking"
	filePtr = filp_open(Progress_file_path, O_RDWR, S_IRWXU|S_IRWXG|S_IRWXO);

	if(!IS_ERR_OR_NULL(filePtr)) {
		// Set update progress 1 percent
		sprintf(temp_progress, "%d", update_progress++);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		write_file_ret = filePtr->f_op->write(filePtr, &temp_progress[0], sizeof(char), &pos);
		set_fs(old_fs);
		filp_close(filePtr, NULL);

		if(write_file_ret>0) {
			update_progress_file=FILE_ACCESS_SUCCESS;
			pr_info("%s ok to write progress\n", Progress_file_path);
		} else {
			update_progress_file=FILE_NOT_CORRECT;
			pr_info("%s write progress fail\n", Progress_file_path);
		}
	} else if(PTR_ERR(filePtr) == -ENOENT) {
		update_progress_file=FILE_NOT_FOUND;
		pr_info("%s not found\n", Progress_file_path);
	} else {
		update_progress_file=FILE_OPEN_FAIL;
		pr_err("%s open error\n", Progress_file_path);
	}
// ASUS_BSP--- Patrick "[A60K][8M][NA][Spec] add ISP firmware progress file checking"

    //Set Falsh Rom address for chip erase
	err = fw_rom_erase(start_rom_address);
	if(err){
		pr_err("fw_rom_erase error");
		goto update_fail;
// ASUS_BSP+++ Patrick "[A60K][8M][NA][Spec] add ISP firmware progress while erase rom"
	} else if(update_progress_file==FILE_ACCESS_SUCCESS) {
		// Set update progress 2 percent
		sprintf(temp_progress, "%d", update_progress);
		filePtr = filp_open(Progress_file_path, O_RDWR, S_IRWXU|S_IRWXG|S_IRWXO);
		if(!IS_ERR_OR_NULL(filePtr)) {
			pos = 0;
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			write_file_ret = filePtr->f_op->write(filePtr, &temp_progress[0], sizeof(char), &pos);
			set_fs(old_fs);
			filp_close(filePtr, NULL);
			if(write_file_ret<=0) {
				update_progress_file=FILE_NOT_CORRECT;
				pr_info("%s write progress fail\n", Progress_file_path);
			}
		}
		pr_info("update progress:%d, after fw_rom_erase\n", update_progress);
// ASUS_BSP--- Patrick "[A60K][8M][NA][Spec] add ISP firmware progress while erase rom"
	}

    fp = filp_open(UPDATE_FILE_WITH_PATH, O_RDONLY, 0);
    if( !IS_ERR_OR_NULL(fp) ){
        start_rom_address = FLASH_ROM_ADDRESS;
        write_size = FJM6MO_SECTOR_SIZE_1k;
        old_fs = get_fs();
        set_fs(KERNEL_DS);
        pr_info("file RS_M6Mo.bin start to update!\n");

		for(i = 0; i < line_count; i++) {
			if((start_rom_address&0xFFFF0000)!=0x10160000) {	//ASUS_BSP+++ Patrick "Save version number block"
				if(i % 128 == 0 || (i % 16 == 0 && sector_size == FJM6MO_SECTOR_SIZE_8)) {

// ASUS_BSP+++ Patrick "[A60K][8M][NA][Spec] add ISP firmware update progress"
					if((i % 128) == 0)
						update_progress = (i/128)*3;
					else
						update_progress += 1;

					pr_info("update progress:%d--\n", update_progress);
					if(update_progress>=100) {
						pr_info("update progress %d more than 100??\n", update_progress);
						sprintf(temp_progress, "%d", 99);
					} else {
						sprintf(temp_progress, "%d", update_progress);
					}

					if((update_progress_file==FILE_ACCESS_SUCCESS)&&(update_progress!=0)) {
						filePtr = filp_open(Progress_file_path, O_RDWR, S_IRWXU|S_IRWXG|S_IRWXO);

						if(!IS_ERR_OR_NULL(filePtr)) {
							pos = 0;
							if(update_progress>=10) {
								write_file_ret = filePtr->f_op->write(filePtr, &temp_progress[0], sizeof(char)*2, &pos);
								if(write_file_ret<=0) {
									update_progress_file=FILE_NOT_CORRECT;
									pr_info("%s write progress fail\n", Progress_file_path);
								}
							} else {
								write_file_ret = filePtr->f_op->write(filePtr, &temp_progress[0], sizeof(char), &pos);
								if(write_file_ret<=0) {
									update_progress_file=FILE_NOT_CORRECT;
									pr_info("%s write progress fail\n", Progress_file_path);
								}
							}
							filp_close(filePtr, NULL);
						}
					}
// ASUS_BSP--- Patrick "[A60K][8M][NA][Spec] add ISP firmware update progress"

					pr_info("start rom address:0x%x\n", start_rom_address);
					//Set flash rom address
					fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 4, 0x0f, 0x00, start_rom_address);
					//Set programming byte
					fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0f, 0x04, sector_size);
					//Clear m6mo internal ram
					fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0f, 0x08, 0x01);
						retry_count = 0;
					do {
							retry_count++;
						msleep(10);
						result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0f, 0x08, 0x01, &status);
					} while ((result != 0x00)&&(retry_count<MAX_LOOPS_RETRIES));
					if(result != 0x00)
						goto update_read_retry_fail;
				}
				//Send programmed firmware
				memset(buf, 0, write_size);
				if(fp->f_op && fp->f_op->read) {
					fp->f_op->read(fp, buf, write_size, &fp->f_pos);
				} else {
					pr_err("Read RS_M6Mo.bin fail\n");
					break;
				}
				fjm6mo_write_memory(ov2720_s_ctrl.sensor_i2c_client->client, buf, 1, start_ram_address,  write_size);
				start_ram_address += write_size;

				if( i % 128 == 127 || (i % 16 == 15 && sector_size == FJM6MO_SECTOR_SIZE_8)) {
					start_ram_address = FJM6MO_RAM_ADDRESS;

					//Program
					fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0f, 0x07, 0x01);
					retry_count = 0;
					do {
						retry_count++;
						msleep(50);
						result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0f, 0x07, 0x01, &status);
					} while ((result != 0x00)&&(retry_count<MAX_LOOPS_RETRIES));
					if(result != 0x00)
						goto update_read_retry_fail;

					if(sector_size == 0) {
						start_rom_address += 0x10000;
					} else {
						start_rom_address += sector_size;
					}

					if(start_rom_address >= 0x101f0000)
						sector_size = FJM6MO_SECTOR_SIZE_8;
				}
// ASUS_BSP+++ Patrick "Save version number block"
			} else { //if((start_rom_address&0xFFFF0000)==0x10160000)
				temp_pos = fp->f_pos;

				temp_rom_address = start_rom_address;
				i += 0x10000/write_size -1;
				fp->f_pos += 0x10000;
				start_rom_address += 0x10000;
			}
// ASUS_BSP--- Patrick "Save version number block"

			if(start_rom_address > FLASH_ROM_FACT_ADDRESS) {
				pr_info("start_rom_address > 0x101f8000!!!, i=%d, line_count=%d--", i, line_count);
				break;
			}
		} //for(i = 0; i < line_count; i += 1)

// ASUS_BSP+++ Patrick "Save version number block"

		if(start_rom_address >= FLASH_ROM_FACT_ADDRESS) {
			// Write the version number block.
			fp->f_pos = temp_pos;
			sector_size = 0;
			pr_info("restart rom address:0x%x\n", temp_rom_address);

			//Set flash rom address
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 4, 0x0f, 0x00, temp_rom_address);
			//Set programming byte
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0f, 0x04, sector_size);
			//Clear m6mo internal ram
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0f, 0x08, 0x01);
			retry_count = 0;
			do{
				retry_count++;
				msleep(10);
				result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0f, 0x08, 0x01, &status);
			} while ((result != 0x00)&&(retry_count<MAX_LOOPS_RETRIES));
			if(result != 0x00)
				goto update_read_retry_fail;

			for(i=0; i <= 127; i++) {
				//Send programmed firmware
				memset(buf, 0, write_size);
				if(fp->f_op && fp->f_op->read) {
					fp->f_op->read(fp, buf, write_size, &fp->f_pos);
				} else {
					pr_err("Read RS_M6Mo.bin fail\n");
				}

				fjm6mo_write_memory(ov2720_s_ctrl.sensor_i2c_client->client, buf, 1, start_ram_address,  write_size);
				start_ram_address += write_size;
			}

			//Program
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0f, 0x07, 0x01);
			retry_count = 1;
			do {
				retry_count++;
				msleep(50);
				result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0f, 0x07, 0x01, &status);
			} while ((result != 0x00)&&(retry_count<MAX_LOOPS_RETRIES));
			if(result != 0x00)
				goto update_read_retry_fail;
		} //if(start_rom_address >= FLASH_ROM_FACT_ADDRESS)

		set_fs(old_fs);
		filp_close(fp, NULL);
	} else { //if( !IS_ERR_OR_NULL(fp) )
		pr_err("file RS_M6Mo.bin open error");
		goto update_fail;
	} //if( !IS_ERR_OR_NULL(fp) )

	pr_info("start_rom_address:0x%x, start_ram_address:0x%x, i=%d--\n",
				start_rom_address, start_ram_address, i);

// ASUS_BSP+++ Patrick "[A60K][8M][NA][Other] verify checksum after firmware update"
	check_after_update(start_rom_address, Progress_file_path);
// ASUS_BSP--- Patrick "[A60K][8M][NA][Other] verify checksum after firmware update"
	err = ov2720_s_ctrl.func_tbl->sensor_power_down(&ov2720_s_ctrl);

	if(err){
		pr_err("isp power on error \n");
		goto update_fail;
	}

//    wake_unlock(&fjm6mo_wake_lock);
//    wake_lock_destroy(&fjm6mo_wake_lock);

	goto update_done;

update_read_retry_fail:
	set_fs(old_fs);
	filp_close(fp, NULL);
	pr_info("ISP read retry more than MAX_LOOPS_RETRIES");

update_fail:	
	pr_info("%s Fail!!\n",__func__);
	return -ENOMEM;
	
update_done:    
	pr_info("%s ---\n",__func__);
	return 0;
}
// ASUS_BSP--- Patrick "[A60K][8M][NA][Others] add for ISP firmware update"

int fjm6mo_sensor_open(void){
// ASUS_BSP +++ LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 
	u32 result = 0;
#ifndef WITH_WQ
        u32 temp;
        int retry =0;
#endif
	
	pr_info("%s +++\n",__func__);
	
	//CAM_START
	result = fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0F, 0x12, 0x01);
  
#ifndef WITH_WQ
	msleep(110);
	do{
		msleep(10);
		result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x00, 0x1C, 0x01, &temp);
		pr_info("stop when (%d)==1?",(temp & 0x01));    
              retry ++;
              if(retry >= MAX_LOOPS_RETRIES){
                pr_err("%s CAM_START fail",__func__);
                return -1;
              }        
	} while((temp & 0x01) != 0x01);		
#endif // end of WITH_WQ
// ASUS_BSP --- LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 
    
	pr_info("%s ---\n",__func__);
	return result;
}

static int sensor_get_status(void)
{
    int result = 0x00;
    u32 buffer;
    result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x00, 0x0C, 0x01, &buffer);
    return buffer;
}

static int isp_interrupt(u8 interrupt_flag)
{
    int retry = 0;
    u32 temp;
    //Enable all type of interrupt factor
    //msleep(10);  // LiJen: no need
    pr_info("isp_interrupt\n");
    fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x00, 0x10, 0xff);

    do{
		//mutex_unlock(ov2720_s_ctrl.msm_sensor_mutex);//add lock
		if(retry == 0)
        msleep(120);//wait for ISP AF process
		else
	 	msleep(15);
		//mutex_lock(ov2720_s_ctrl.msm_sensor_mutex);//add lock
        fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x00, 0x1C, 0x01, &temp);
        retry += 1;
    } while((temp != interrupt_flag) && retry < 135);
    if(temp != interrupt_flag)
        return -ENOMEM;
    else
        return 0;
}
// ASUS_BSP +++ LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode"   
#ifdef WITH_WQ

void fjm6mo_sensor_change_init_wq(struct work_struct *work){
    	u32 result = 0, temp;
	int retry =0;

       pr_info("%s E",__func__);	
       mutex_lock(ov2720_s_ctrl.msm_sensor_mutex);  
       g_fjm6mo_wq_start = true;
       caf_mode = false;

       //Wait CAM_START
	msleep(110);
	do{
		msleep(10);
		result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x00, 0x1C, 0x01, &temp);
		pr_info("stop when (%d)==1?",(temp & 0x01));    
              retry ++;
              if(retry >= MAX_LOOPS_RETRIES){
                pr_err("%s CAM_START fail",__func__);
                goto end;
              }        
	} while((temp & 0x01) != 0x01);		

    	//set Parameters
       fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x00, 0x10, 0xff);
       fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x00, 0x0B, FJM6MO_PARA_STATUS_PAR);    	       
        retry = 0;
        do{
	     msleep(10);
            result = sensor_get_status();
            retry += 1;
        }while(result != 0x1 && retry < MAX_LOOPS_RETRIES);
        if(result != 0x1){
            pr_info("Change to parameter mode fail\n");
            if(factory_mode == true)
                goto end;
        }

// ASUS_BSP+++ Patrick "[A60K][8M][NA][Others] Set shading value in ISP camera"
		//Set Shading on
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x07, 0x01);
	if(shading_table != 0){
		//Set shading table for calibration
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x2C, 0x01);
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x07, 0x02);
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0D, 0x30, shading_table);
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0D, 0x31, shading_table);
	}
// ASUS_BSP--- Patrick "[A60K][8M][NA][Others] Set shading value in ISP camera"

	 //Setting Capture parameter 
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0C, 0x06, 0x01); //Cap sel frame = 1
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0B, 0x00, 0x00); //YUV422
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0B, 0x1, 0x25); // SET Capture resolution 3264x2448 8MP

         // set default focus coodinate and retangle
         fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0A, 0x22, 0x33);	//w     
         fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0A, 0x24, 0x33);      //h 
         fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0A, 0x2A, 0x80);	//x
         fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0A, 0x2C, 0x80);	//y          
end:
    g_fjm6mo_wq_start = false;    
    mutex_unlock(ov2720_s_ctrl.msm_sensor_mutex);  
    pr_info("%s X",__func__);
}

void fjm6mo_sensor_change_mon_wq(struct work_struct *work)
{	     
    //struct fjm6mo_work *fjm6mo_work = container_of(work, struct fjm6mo_work, work);
    //E_M6MO_Status state = fjm6mo_work->state;
    int result = 0x00, err=0;
    u32 buffer;
    
    pr_info("%s E",__func__);
    mutex_lock(ov2720_s_ctrl.msm_sensor_mutex);  
    g_fjm6mo_wq_start = true;

    err = isp_interrupt(INT_STATUS_MODE);
    if(err){
        pr_info("Change to monitor mode fail\n");
        if(factory_mode == true)
                goto end;
    }

    //AF_START: is AF done?
    result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0A, 0x02, 0x01, &buffer);
    if(buffer == 0x2)
    {
        pr_info("Release autofocus\n");
        if(!caf_mode){
            //LiJen: do not release AF before entering preview mode to keep AF status"
            //AF_START: AF release
            //fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x02, 0x03);
        }
        else {
             //AF_SCAN_MODE: set continuous focus
            fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x41, 0x04);
            //AF_START: AF start
            fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x02, 0x01);
        }		
    }		

end:
    g_fjm6mo_wq_start = false;  
    mutex_unlock(ov2720_s_ctrl.msm_sensor_mutex);  
    pr_info("%s X",__func__);
}

void fjm6mo_create_workqueue(void){
    g_fjm6mo_wq = create_singlethread_workqueue("fjm6mo_wq");
    INIT_WORK(&(g_fjm6mo_work_mon.work), fjm6mo_sensor_change_mon_wq);
    INIT_WORK(&(g_fjm6mo_work_init.work), fjm6mo_sensor_change_init_wq);
}

void  fjm6mo_destroy_workqueue(void){
    destroy_workqueue(g_fjm6mo_wq);
}

void  fjm6mo_flush_workqueue(void){
    flush_workqueue(g_fjm6mo_wq);
}


void fjm6mo_change_status(u8 mode){
    //struct fjm6mo_work *p_fjm6mo_work = NULL;

    pr_info("%s E",__func__);
    //p_fjm6mo_work = kzalloc(sizeof(struct fjm6mo_work), GFP_ATOMIC);

    //if (p_fjm6mo_work == NULL) {
	//pr_err("%s: mem failure X\n", __func__);
	//return;
    //}
    
    //p_fjm6mo_work->state= state;
    if(FJM6MO_PARA_STATUS_MON == mode){
        //INIT_WORK(&p_fjm6mo_work->work, fjm6mo_sensor_change_mon_wq);
        queue_work(g_fjm6mo_wq, &(g_fjm6mo_work_mon.work));  
    }else if(FJM6MO_PARA_STATUS_INT== mode){
        //INIT_WORK(&p_fjm6mo_work->work, fjm6mo_sensor_change_init_wq);
        queue_work(g_fjm6mo_wq, &(g_fjm6mo_work_init.work));  
    }

    pr_info("%s X",__func__);
}
#endif //end of WITH_WQ
// ASUS_BSP --- LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode"   

int sensor_change_status(E_M6MO_Status status)
{
// ASUS_BSP +++ LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 
    u8 parameter;
    int result = 0x00, retry = 0;
    u32 buffer;
#ifndef WITH_WQ
    int err=0;
#endif // end of WITH_WQ
// ASUS_BSP --- LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 
    switch(status)
    {
// ASUS_BSP +++ LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode"     
#ifdef WITH_WQ    
        case E_M6MO_Status_Init:
        {
            parameter = FJM6MO_PARA_STATUS_INT;
            break;
        }
#endif        
// ASUS_BSP --- LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 
        case E_M6MO_Status_Parameter:
        {
            parameter = FJM6MO_PARA_STATUS_PAR;
            break;
        }
        case E_M6MO_Status_Monitor:
        {
            parameter = FJM6MO_PARA_STATUS_MON;
            break;
        }
        case E_M6MO_Status_Capture:
        {
            parameter = FJM6MO_PARA_STATUS_CAP;
            break;
        }
        default:
        {
            parameter = FJM6MO_PARA_STATUS_ERR;
            break;
        }
    }

    pr_info("fjm6mo sensor_change_status: %d\n", parameter);

// ASUS_BSP +++ LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode"
#ifdef WITH_WQ
    while(true == g_fjm6mo_wq_start && retry < (MAX_LOOPS_RETRIES * 2)){
        msleep(10);
        retry += 1;
        pr_info("%s waiting for work queue stop",__func__);
    }
#endif // end of WITH_WQ
// ASUS_BSP --- LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode"

    if(parameter == FJM6MO_PARA_STATUS_MON)
    {
//ASUS_BSP +++ Stimber "Set the resolution before Monitor mode, because this could only be set in parameter mode"
		if(g_rt_for_monitor == MSM_SENSOR_RES_FULL_HD){
	 		pr_info("[ISP] SET Preview resolution 1920x1080\n");
	 		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x01, 0x28); // SET Preview resolution 1920x1080
	 	}
	 	else{
	 		pr_info("[ISP] SET Preview resolution 1280x960\n");
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x01, 0x24); // SET Preview resolution 1280x960
	 	}
//ASUS_BSP --- Stimber "Set the resolution before Monitor mode, because this could only be set in parameter mode"
	
#if 0    //don't need 
        //Stop continuous ouput capture mode     
        if(capture_mode){
            pr_info("Stop continuous ouput frame from capture mode\n");
            capture_mode = false;
            fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0C, 0x06, 0x01); //Cap sel frame = 1
            err = isp_interrupt(INT_STATUS_CAPTURE);
            if(err)
                pr_info("Select image frame fail\n");
        }
#endif
        //Go to the monitor mode
        //SYS_MODE: set Monitor mode
// ASUS_BSP +++ LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 
#ifdef WITH_WQ
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x00, 0x10, 0xff);
#endif // end of WITH_WQ
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x00, 0x0B, parameter);
#ifdef WITH_WQ
        fjm6mo_change_status(parameter);
#else        // end of WITH_WQ
        err = isp_interrupt(INT_STATUS_MODE);
        if(err){
            pr_info("Change to monitor mode fail\n");
            if(factory_mode == true)
                return -ENOMEM;
        }
	 //AF_START: is AF done?
        result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0A, 0x02, 0x01, &buffer);
        if(buffer == 0x2)
        {
            pr_info("Release autofocus\n");
	     //ASUS_BSP LiJen +++ "[A60K][8M][NA][Spec]do not release AF before entering preview mode to keep AF status"
            if(!caf_mode){
		  //AF_START: AF release
                //fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x02, 0x03);
            }
	     //ASUS_BSP LiJen --- "[A60K][8M][NA][Spec]do not release AF before entering preview mode to keep AF status"
            else {
		  //AF_SCAN_MODE: set continuous focus
                fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x41, 0x04);
		  //AF_START: AF start
                fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x02, 0x01);
            }
#if 0  // don't need	face detect 		
            //Set af_window back to center or face_window
            fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x09, 0x00, 0x01, &buffer);
            if(buffer == 0x1)
                fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x40, 0x03);
            else
                fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x40, 0x01);
#endif			
        }		
#endif //end of WITH_WQ
// ASUS_BSP --- LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode"       
    }
    else if(parameter == FJM6MO_PARA_STATUS_PAR){
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x00, 0x0B, parameter);
        retry = 0;
        do{
	     msleep(10);
            result = sensor_get_status();
            retry += 1;
        }while(result != 0x1 && retry < MAX_LOOPS_RETRIES);
        if(result != 0x1){
            pr_info("Change to parameter mode fail\n");
            if(factory_mode == true)
                return -ENOMEM;
        }

	 //Setting Monitor parameter 
//ASUS_BSP +++ Stimber "Move to the state before entering to monitor mode"
	 //fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x01, 0x24); // SET Preview resolution 1280x960
	 //fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x01, 0x28); // SET Preview resolution 2112x1188
//ASUS_BSP --- Stimber "Move to the state before entering to monitor mode"	

// ASUS_BSP+++ Patrick "[A60K][8M][NA][Others] Set shading value in ISP camera"
		//Set Shading on
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x07, 0x01);
	if(shading_table != 0){
		//Set shading table for calibration
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x2C, 0x01);
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x07, 0x02);
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0D, 0x30, shading_table);
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0D, 0x31, shading_table);
	}
// ASUS_BSP--- Patrick "[A60K][8M][NA][Others] Set shading value in ISP camera"

	 //Setting Capture parameter 
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0C, 0x06, 0x01); //Cap sel frame = 1
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0B, 0x00, 0x00); //YUV422
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0B, 0x1, 0x25); // SET Capture resolution 3264x2448 8MP

         // set default focus coodinate and retangle
         fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0A, 0x22, 0x33);	//w     
         fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0A, 0x24, 0x33);      //h 
         fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0A, 0x2A, 0x80);	//x
         fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0A, 0x2C, 0x80);	//y       
    }
    else if(parameter == FJM6MO_PARA_STATUS_CAP){
        int int_sticky = 0x88;
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x00, 0x10, 0xFF);
 	 fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x00, 0x12, 0x01);
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x00, 0x0B, parameter);
        retry = 0;
        do{
            msleep(WAIT_FOR_CHANGE_CAPTURE_DONE);	//To do: performance
            result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x00, 0x1C, 0x01, &buffer);
            if ((buffer & 0x80) && (buffer != 0xfa))
            {
                int_sticky &= ~0x80;
        	}
            if ((buffer & 0x08) && (buffer != 0xfa))
            {
                int_sticky &= ~0x08;
        	}
            retry += 1;

			//ASUS_BSP +++ Stimber "Add debug log when change capture mode slow"
			if((retry * WAIT_FOR_CHANGE_CAPTURE_DONE >= 1000) && ((retry * WAIT_FOR_CHANGE_CAPTURE_DONE) % 400) == 0){
				pr_info("[ISP] Change to capture mode slow(%d ms)\n", (retry * WAIT_FOR_CHANGE_CAPTURE_DONE));
			}
			//ASUS_BSP --- Stimber "Add debug log when change capture mode slow"
        } while(int_sticky != 0x00 && retry < MAX_LOOPS_RETRIES*5);
        if(int_sticky != 0x00){
            pr_info("Change to capture mode fail\n");
            return -ENOMEM;
	 }        
	 //Setting Capture parameter 
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0C, 0x06, 0x01); //Cap sel frame = 1
	 //CAP_TRANSFER_START
        pr_info("start transfer...\n");
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0C, 0x9, 0x01); // start transfer		
    }
// ASUS_BSP +++ LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode"       
#ifdef WITH_WQ
    else if(parameter == FJM6MO_PARA_STATUS_INT){
        fjm6mo_change_status(parameter);       
    }
#endif //end of WITH_WQ
// ASUS_BSP --- LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode"     

    return 0;
}

// ASUS_BSP+++ Patrick "[A66][8M][NA][Others] change ISP mode for calibration"
//20120621 copy from sensor_change_status()
int sensor_change_status_without_WQ(E_M6MO_Status status)
{
// ASUS_BSP +++ LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 
    u8 parameter;
    int result = 0x00, retry = 0;
    u32 buffer;
    int err=0;
// ASUS_BSP --- LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 
    switch(status)
    {
// ASUS_BSP +++ LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode"     

// ASUS_BSP --- LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 
        case E_M6MO_Status_Parameter:
        {
            parameter = FJM6MO_PARA_STATUS_PAR;
            break;
        }
        case E_M6MO_Status_Monitor:
        {
            parameter = FJM6MO_PARA_STATUS_MON;
            break;
        }
        case E_M6MO_Status_Capture:
        {
            parameter = FJM6MO_PARA_STATUS_CAP;
            break;
        }
        default:
        {
            parameter = FJM6MO_PARA_STATUS_ERR;
            break;
        }
    }

    pr_info("fjm6mo sensor_change_status: %d\n", parameter);

    if(parameter == FJM6MO_PARA_STATUS_MON)
    {
//ASUS_BSP +++ Stimber "Set the resolution before Monitor mode, because this could only be set in parameter mode"
		if(g_rt_for_monitor == MSM_SENSOR_RES_FULL_HD){
	 		pr_info("[ISP] SET Preview resolution 1920x1080\n");
	 		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x01, 0x28); // SET Preview resolution 1920x1080
	 	}
	 	else{
	 		pr_info("[ISP] SET Preview resolution 1280x960\n");
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x01, 0x24); // SET Preview resolution 1280x960
	 	}
//ASUS_BSP --- Stimber "Set the resolution before Monitor mode, because this could only be set in parameter mode"
	
#if 0    //don't need 
        //Stop continuous ouput capture mode     

        if(capture_mode){
            pr_info("Stop continuous ouput frame from capture mode\n");
            capture_mode = false;
            fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0C, 0x06, 0x01); //Cap sel frame = 1
            err = isp_interrupt(INT_STATUS_CAPTURE);
            if(err)
                pr_info("Select image frame fail\n");
        }
#endif
        //Go to the monitor mode
        //SYS_MODE: set Monitor mode
// ASUS_BSP +++ LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x00, 0x0B, parameter);

        err = isp_interrupt(INT_STATUS_MODE);
        if(err){
            pr_info("Change to monitor mode fail\n");
            if(factory_mode == true)
                return -ENOMEM;
        }
	 //AF_START: is AF done?
        result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0A, 0x02, 0x01, &buffer);
        if(buffer == 0x2)
        {
            pr_info("Release autofocus\n");
	     //ASUS_BSP LiJen +++ "[A60K][8M][NA][Spec]do not release AF before entering preview mode to keep AF status"
            if(!caf_mode){
		  //AF_START: AF release
                //fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x02, 0x03);
            }
	     //ASUS_BSP LiJen --- "[A60K][8M][NA][Spec]do not release AF before entering preview mode to keep AF status"
            else {
		  //AF_SCAN_MODE: set continuous focus
                fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x41, 0x04);
		  //AF_START: AF start
                fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x02, 0x01);
            }
#if 0  // don't need	face detect 		
            //Set af_window back to center or face_window
            fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x09, 0x00, 0x01, &buffer);
            if(buffer == 0x1)
                fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x40, 0x03);
            else
                fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x40, 0x01);
#endif			
        }		

// ASUS_BSP --- LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode"       
    }
    else if(parameter == FJM6MO_PARA_STATUS_PAR){
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x00, 0x0B, parameter);
        retry = 0;
        do{
	     msleep(10);
            result = sensor_get_status();
            retry += 1;
        }while(result != 0x1 && retry < MAX_LOOPS_RETRIES);
        if(result != 0x1){
            pr_info("Change to parameter mode fail\n");
            if(factory_mode == true)
                return -ENOMEM;
        }

	 //Setting Monitor parameter 
//ASUS_BSP +++ Stimber "Move to the state before entering to monitor mode"
	 //fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x01, 0x24); // SET Preview resolution 1280x960
	 //fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x01, 0x28); // SET Preview resolution 2112x1188
//ASUS_BSP --- Stimber "Move to the state before entering to monitor mode"	

// ASUS_BSP+++ Patrick "[A60K][8M][NA][Others] Set shading value in ISP camera"
		//Set Shading on
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x07, 0x01);
	if(shading_table != 0){
		//Set shading table for calibration
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x2C, 0x01);
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x07, 0x02);
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0D, 0x30, shading_table);
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0D, 0x31, shading_table);
	}
// ASUS_BSP--- Patrick "[A60K][8M][NA][Others] Set shading value in ISP camera"

	 //Setting Capture parameter 
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0C, 0x06, 0x01); //Cap sel frame = 1
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0B, 0x00, 0x00); //YUV422
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0B, 0x1, 0x25); // SET Capture resolution 3264x2448 8MP

         // set default focus coodinate and retangle
         fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0A, 0x22, 0x33);	//w     
         fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0A, 0x24, 0x33);      //h 
         fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0A, 0x2A, 0x80);	//x
         fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0A, 0x2C, 0x80);	//y       
    }
    else if(parameter == FJM6MO_PARA_STATUS_CAP){
        int int_sticky = 0x88;
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x00, 0x10, 0xFF);
 	 fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x00, 0x12, 0x01);
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x00, 0x0B, parameter);
        retry = 0;
        do{
            msleep(WAIT_FOR_CHANGE_CAPTURE_DONE);	//To do: performance
            result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x00, 0x1C, 0x01, &buffer);
            if ((buffer & 0x80) && (buffer != 0xfa))
            {
                int_sticky &= ~0x80;
        	}
            if ((buffer & 0x08) && (buffer != 0xfa))
            {
                int_sticky &= ~0x08;
        	}
            retry += 1;

			//ASUS_BSP +++ Stimber "Add debug log when change capture mode slow"
			if((retry * WAIT_FOR_CHANGE_CAPTURE_DONE >= 1000) && ((retry * WAIT_FOR_CHANGE_CAPTURE_DONE) % 400) == 0){
				pr_info("[ISP] Change to capture mode slow(%d ms)\n", (retry * WAIT_FOR_CHANGE_CAPTURE_DONE));
			}
			//ASUS_BSP --- Stimber "Add debug log when change capture mode slow"
        } while(int_sticky != 0x00 && retry < MAX_LOOPS_RETRIES*5);
        if(int_sticky != 0x00){
            pr_info("Change to capture mode fail\n");
            return -ENOMEM;
	 }        
	 //Setting Capture parameter 
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0C, 0x06, 0x01); //Cap sel frame = 1
	 //CAP_TRANSFER_START
        pr_info("start transfer...\n");
        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0C, 0x9, 0x01); // start transfer		
    }

    return 0;
}
// ASUS_BSP--- Patrick "[A66][8M][NA][Others] change ISP mode for calibration"

//ASUS_BSP +++ LiJen "[A60K][8M][NA][Spec]Implemet continuous autofocus for 8M camera with ISP"
// ASUS_BSP+++ LiJen "[ov2720] add autofocus function"
void fjm6mo_set_CAF(bool continuous){
	int result =0x00,err=0;
	u32 buffer;
	
	pr_info("%s +++ as %d\n",__func__,continuous);

	if(continuous){
	    caf_mode = true;
	    //AF_SCAN_MODE: set continuous focus
	    fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x41, 0x4);
	    //AF_START: AF start
	    fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x02, 0x1);
	}
	else{
	    caf_mode = false;
	    //read AF_SCAN_MODE
	    result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0A, 0x41, 0x01, &buffer);
	    if(buffer == 0x4){ //AF_SCAN_MODE = continuous focus
	        //INT_ENABLE: enable interrupt
	        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x00, 0x10, 0xff);
		 //AF_START: AF stop by force
	        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x02, 0x0);
	        err = isp_interrupt(INT_STATUS_AF);
	        if(err)
	            pr_err("CAF stop error: no interrupt");
		 //AF_START: AF release
	        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x02, 0x3);
		 //AF_SCAN_MODE: set fast focus
	        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x41, 0x3);
	    }
	}

	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A60K][8M][NA][Spec]Implemet continuous autofocus for 8M camera with ISP"
void fjm6mo_set_touch_AF(int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w){
	u32 af_w=0x55, af_h=0x55, af_x=0x55, af_y=0x55;
	pr_info("%s +++\n",__func__);
	pr_info("%s: coordinate_x:0x%x coordinate_y:0x%x rectangle_h:0x%x rectangle_w:0x%x\n", __func__, coordinate_x, coordinate_y, rectangle_h, rectangle_w);

//ASUS_BSP +++ LiJen "[A60K][8M][NA][Others]coordinate remapping of anypoint focus for ICS"
	// get preview resolution from ISP
	if(coordinate_x == -1){
		af_x = 0x55;  // ISP default
	}else if(coordinate_x > 0x0100){
		af_x = 0x0100;
	}else if(coordinate_x < 0){
		af_x = 0x0;
	}else{
		af_x = coordinate_x;
	}
	
	if(coordinate_y == -1){
		af_y = 0x55;  // ISP default
	}else if(coordinate_y > 0x0100){
		af_y = 0x0100;
	}else if(coordinate_y < 0){
		af_y = 0x0;
	}else{
		af_y = coordinate_y;
	}

	if(rectangle_w == -1){
		af_w = 0x55;  // ISP default
	}else if(rectangle_w > 0x0100){
		af_w = 0x0100;
	}else if(rectangle_w < 0){
		af_w = 0x0;
	}else{
		af_w = rectangle_w;
	}

	if(rectangle_h == -1){
		af_h = 0x55;  // ISP default
	}else if(rectangle_h > 0x0100){
		af_h = 0x0100;
	}else if(rectangle_h < 0){
		af_h = 0x0;
	}else{
		af_h = rectangle_h;
	}	
//ASUS_BSP --- LiJen "[A60K][8M][NA][Others]coordinate remapping of anypoint focus for ICS"
	pr_info("%s: af_x:0x%x af_y:0x%x af_w:0x%x af_h:0x%x\n", __func__, af_x, af_y, af_w, af_h);

//ASUS_BSP +++ LiJen "[A60K][8M][NA][Fix]fix center auto focus is inaccurate"
       if(coordinate_x == 0x55 && coordinate_y == 0x55 && rectangle_w == 0x55 && rectangle_h == 0x55){
            // set Centre large window
            fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x40, 0x01);
       }else{
            // set specified position by user (For touch AF)
            fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x40, 0x04);

            // set focus coodinate and retangle
            fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0A, 0x22, af_w);	 
            fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0A, 0x24, af_h);		 
            fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0A, 0x2A, af_x);	 
            fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0A, 0x2C, af_y);	
       }
//ASUS_BSP --- LiJen "[A60K][8M][NA][Fix]fix center auto focus is inaccurate"       
	pr_info("%s ---\n",__func__);
}

//ASUS_BSP +++ LiJen "[A60K][8M][NA][Spec]implement set focus mode"
//ASUS_BSP +++ LiJen "[A60K][8M][NA][Spec]Implemet continuous autofocus for 8M camera with ISP"
void fjm6mo_start_AF(bool on, isp3a_af_mode_t mode, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w)
{
// ASUS_BSP +++ LiJen "[ov2720] add ISP Continuous AF start function"
	int err=0;
	
	pr_info("%s +++ Param(%d,%d,%d,%d,%d,%d)\n",__func__,on,mode,coordinate_x,coordinate_y,rectangle_h,rectangle_w);
			
	//ASUS_BSP +++ LiJen "[A60K][8M][NA][Others]implement cancel autofocus in 8M camera with ISP"
	if(on){
            if(caf_mode){
              pr_info("CAF starting ... Cancel autofocus ---\n");
		return;
            }else{
                switch(mode) {
                    case AF_MODE_MACRO:
                        //set AF_RANGE_MODE = Macro range  (50cm to macro)
                        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x42, 0x1);
                        break;
                    case AF_MODE_NORMAL:
                        //set AF_RANGE_MODE = Normal range (Infinity to 50cm)
                        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x42, 0x0);
                        break; 
                    case AF_MODE_AUTO:                     
                        //set AF_RANGE_MODE = Full range (Infinity to macro)
                        fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x42, 0x2);
                        break;                          
                    case AF_MODE_UNCHANGED:                        
                    case AF_MODE_CAF:
                    case AF_MODE_MAX:  
                    default:
                        pr_info("%s ---\n",__func__);
                        return;          
               }
        
		//Any point focus setting
		fjm6mo_set_touch_AF(coordinate_x,coordinate_y,rectangle_h,rectangle_w);
		//AF_START: AF start
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x02, 0x1);
		pr_info("Start autofocus\n");		
			
		//Wait AF interrupt
		err = isp_interrupt(INT_STATUS_AF);
		if(err){
			pr_err("waitting AF interrupt");
		}
          }
	}else{
		//Cancel AutoFocus
              //AF_START: AF release
              fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0A, 0x02, 0x03);
		pr_info("Cancel autofocus\n");
	}
	//ASUS_BSP --- LiJen "[A60K][8M][NA][Others]implement cancel autofocus in 8M camera with ISP"
// ASUS_BSP --- LiJen "[ov2720] add ISP Continuous AF start function"
	pr_info("%s ---\n",__func__);
}
// ASUS_BSP--- LiJen "[ov2720] add autofocus function"	
//ASUS_BSP --- LiJen "[A60K][8M][NA][Spec]implement set focus mode"

uint16_t fjm6mo_get_AF_result(struct msm_sensor_ctrl_t *s_ctrl)
{
	u32 status;
	bool result=false;
	
	pr_info("%s +++\n",__func__);
	if(!caf_mode){
		//Read AF result
	
		fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0A, 0x03, 0x01, &status);
		switch(status)
		{
			case 0:
			{
				pr_info("AF operating\n");
				result = false;                
				break;
			}
			case 1:
			{
				pr_info("AF success\n");
				result = true;
				break;
			}
			case 2:
			{
				pr_info("AF fail\n");
				result = false;
				break;
			}
			case 3:
			{
				pr_info("AF stopped at edge\n");
				result = false;
				break;
			}
		}
	}
	else{
		result = false;
		pr_err("CAF is starting\n");
	}
	
	pr_info("%s result(%d)---\n",__func__,result);
	return result;
}
//ASUS_BSP --- LiJen "[A60K][8M][NA][Spec]Implemet continuous autofocus for 8M camera with ISP"

//ASUS_BSP +++ LiJen "[A60K][8M][NA][Others]implement LED/Flash mode in 8M camera with ISP"
void fjm6mo_set_led_mode(led_mode_t mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);
	switch(mode)
	{
		case LED_MODE_ON:
                     fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x39, 0x03);
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0B, 0x1F, 0x03);
			break;
		case LED_MODE_OFF:
                     fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x39, 0x03);
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0B, 0x1F, 0x00);
			break;
		case LED_MODE_AUTO:
                     fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x39, 0x03);
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0B, 0x1F, 0x02);
			break;
		case LED_MODE_TORCH:
                   fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x39, 0x00);
			break;
		default:
			pr_info("%s mode(%d) is not support \n",__func__,mode);
			break;
	}		
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A60K][8M][NA][Others]implement LED/Flash mode in 8M camera with ISP"

//ASUS_BSP +++ LiJen "[A60K][8M][NA][Others]implement Effect mode in 8M camera with ISP"	
void fjm6mo_set_effect_mode(int16_t mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);
	switch(mode)
	{
		case CAMERA_EFFECT_OFF:	// close effect
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0B, 0x0);
			break;
		case CAMERA_EFFECT_SEPIA:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0B, 0x01);
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x09, 0xDB);
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0A, 0x18);
                    break;
		case CAMERA_EFFECT_MONO:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0B, 0x01);
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x09, 0x00);
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0A, 0x00);
			break;
		case CAMERA_EFFECT_NEGATIVE:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0B, 0x02);
			break;
		default:
			pr_info("%s mode(%d) is not support \n",__func__,mode);
			break;
            }
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A60K][8M][NA][Others]implement Effect mode in 8M camera with ISP"	

//ASUS_BSP +++ LiJen "[A60K][8M][NA][Others]implement WB mode in 8M camera with ISP"	
void fjm6mo_set_wb_mode(config3a_wb_t mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);
	if(mode == CAMERA_WB_AUTO){
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x06, 0x02, 0x01);	
	}
	else{
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x06, 0x02, 0x02);
		switch(mode)
		{
			case CAMERA_WB_INCANDESCENT:
				fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x06, 0x03, 0x01);
				break;
			case CAMERA_WB_DAYLIGHT:
				fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x06, 0x03, 0x04);
				break;
			case CAMERA_WB_FLUORESCENT:
				fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x06, 0x03, 0x02);
	                    break;
			case CAMERA_WB_CLOUDY_DAYLIGHT:
				fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x06, 0x03, 0x05);
				break;
			case CAMERA_WB_TWILIGHT:
				fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x06, 0x03, 0x03);
				break;
			case CAMERA_WB_SHADE:
				fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x06, 0x03, 0x06);
				break;
			default:
				pr_info("%s mode(%d) is not support \n",__func__,mode);
				break;
		}		
	}
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A60K][8M][NA][Others]implement WB mode in 8M camera with ISP"	

//ASUS_BSP +++ LiJen "[A60K][8M][NA][Others]implement EV mode in 8M camera with ISP"	
void fjm6mo_set_ev_mode(int16_t mode)
{
	uint16_t ev_val =0;
	pr_info("%s +++ mode(%d)\n",__func__,mode);

	if(mode>=-18 && mode <=18){	// AP: -3EV=-18, 0EV=0, +3EV=18
		ev_val = (mode+18)*5/3;		//ISP: -3EV=0, 0EV=30, +3EV=60
	}else{
		pr_info("%s -- mode(%d) is not support \n",__func__,mode);
		return;
	}
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x09, ev_val);
			
	pr_info("%s --- ev_val(0x%x)\n",__func__,ev_val);
}
//ASUS_BSP --- LiJen "[A60K][8M][NA][Others]implement EV mode in 8M camera with ISP"

//ASUS_BSP +++ LiJen "[A60K][8M][NA][Others]implement Sence mode in 8M camera with ISP"	
void fjm6mo_set_scene_mode(camera_bestshot_mode_type mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);

	switch(mode)
	{
		case CAMERA_BESTSHOT_OFF:
                     fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0F, 0x10);
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x37, 0x00);
                     break;
		case CAMERA_BESTSHOT_AUTO:  //Normal mode
                     fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0F, 0x10);            
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x37, 0x01);            
			break;	
		case CAMERA_BESTSHOT_LANDSCAPE:
                     fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0F, 0x10);            
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x37, 0x03);
			break;
		case CAMERA_BESTSHOT_SNOW:
                     fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0F, 0x10);            
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x37, 0x0A);
			break;
		case CAMERA_BESTSHOT_SUNSET:
                     fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0F, 0x10);            
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x37, 0x06);
			break;
		case CAMERA_BESTSHOT_NIGHT:
                     fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0F, 0x10);            
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x37, 0x05);
			break;			
		case CAMERA_BESTSHOT_PORTRAIT:
                     fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0F, 0x10);            
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x37, 0x02);
			break;	
		case CAMERA_BESTSHOT_BACKLIGHT:
                     fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0F, 0x10);            
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x37, 0x08);
			break;				
		case CAMERA_BESTSHOT_SPORTS:
                     fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0F, 0x10);            
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x37, 0x04);
			break;	
		case CAMERA_BESTSHOT_FLOWERS:
			//fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x37, 0x09);
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x37, 0x01);
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0F, 0x16);
			break;			
		case CAMERA_BESTSHOT_PARTY:
                     fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x0F, 0x10);            
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x37, 0x0E);
			break;	
		case CAMERA_BESTSHOT_BEACH:				
		case CAMERA_BESTSHOT_ANTISHAKE:
		case CAMERA_BESTSHOT_CANDLELIGHT:
		case CAMERA_BESTSHOT_FIREWORKS:			
		case CAMERA_BESTSHOT_NIGHT_PORTRAIT:		
		case CAMERA_BESTSHOT_THEATRE:
		case CAMERA_BESTSHOT_ACTION:
		case CAMERA_BESTSHOT_AR:		
		default:
			pr_info("%s mode(%d) is not support \n",__func__,mode);
			break;
            }	
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A60K][8M][NA][Others]implement Sence mode in 8M camera with ISP"

//ASUS_BSP +++ LiJen "[A60K][8M][NA][Others]implement CAF mode in 8M camera with ISP"
void fjm6mo_set_caf_mode(int16_t mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);	
	fjm6mo_set_CAF(mode);			
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A60K][8M][NA][Others]implement CAF mode in 8M camera with ISP"	

//ASUS_BSP +++ LiJen "[A60K][8M][NA][Others]implement AEC Lock mode in 8M camera with ISP"
void fjm6mo_set_acelock_mode(int16_t mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);	
	switch(mode)
	{
		case 0:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x00, 0x00);
			break;		
		case 1:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x00, 0x01);
			break;	            
		default:
			pr_info("%s mode(%d) is not support \n",__func__,mode);
			break;            
       }		
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A60K][8M][NA][Others]implement AEC Lock mode in 8M camera with ISP"

//ASUS_BSP +++ LiJen "[A60K][8M][NA][Others]implement AWB Lock mode in 8M camera with ISP"
void fjm6mo_set_awblock_mode(int16_t mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);	
	switch(mode)
	{
		case 0:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x06, 0x02, 0x00);
			break;		
		case 1:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x06, 0x02, 0x01);
			break;	            
		default:
			pr_info("%s mode(%d) is not support \n",__func__,mode);
			break;            
       }		
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A60K][8M][NA][Others]implement AWB Lock mode in 8M camera with ISP"

//ASUS_BSP +++ LiJen "[A60K][8M][NA][Others]implement ISO mode in 8M camera with ISP"
void fjm6mo_set_iso_mode(int16_t mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);	
	switch(mode)
	{
		case MSM_V4L2_ISO_AUTO:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x05, 0x00);
			break;		
		case MSM_V4L2_ISO_50:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x05, 0x01);
                     break;
		case MSM_V4L2_ISO_100:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x05, 0x02);
			break;	
		case MSM_V4L2_ISO_200:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x05, 0x03);
			break;	
		case MSM_V4L2_ISO_400:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x05, 0x04);
			break;	
		case MSM_V4L2_ISO_800:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x05, 0x05);
			break;	        
		default:
			pr_info("%s mode(%d) is not support \n",__func__,mode);
			break;            
       }		
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A60K][8M][NA][Others]implement ISO mode in 8M camera with ISP"

//ASUS_BSP +++ LiJen "[A60K][8M][NA][Others]implement Flicker mode in 8M camera with ISP"

void fjm6mo_set_flicker_mode(int16_t mode)
{
	pr_info("%s +++ mode(%d)\n",__func__,mode);	
	switch(mode)
	{
		case CAMERA_ANTIBANDING_OFF:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x06, 0x04);
			break;		
		case CAMERA_ANTIBANDING_60HZ:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x06, 0x02);
			break;	
		case CAMERA_ANTIBANDING_50HZ:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x06, 0x01);
			break;	
		case CAMERA_ANTIBANDING_AUTO:
			fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x06, 0x00);
			break;	
		default:
			pr_info("%s mode(%d) is not support \n",__func__,mode);
			break;            
       }		
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A60K][8M][NA][Others]implement Flicker mode in 8M camera with ISP"

//ASUS_BSP +++ Stimber "[A60K][8M][NA][Other] Implement EXIF info for 8M camera with ISP"
void fjm6mo_get_exif(struct exif_cfg *exif)
{
	uint32_t buffer = 0;
	uint16_t iso = 0;
	uint32_t expTime_num = 0;
	uint32_t expTime_deno = 0;

	pr_info("%s +++\n", __func__);
	
    /*if(capture_mode){
        pr_info("Select Image number of frame\n");
        //_T("Stop continuous ouput frame from capture mode\n");
        if(!(s_flag & 0x2))
            fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0C, 0x06, 0x01); //Cap sel frame = 1
        err = isp_interrupt(INT_STATUS_CAPTURE);
        if(err)
            pr_err("Select image frame fail\n");
        else
            capture_mode = false;
    }*/
    //CHECK_WRITE(err);

    fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x07, 0x28, 0x02, &buffer);
	pr_err("[ISP] EXIF before map ISO=%d\n", iso);
	
	if(buffer < 75)
		iso = 50;
	else if(buffer < 140)
		iso = 100;
	else if(buffer < 280)
		iso = 200;
	else if(buffer < 560)
		iso = 400;
	else
		iso = 800;
	
	fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x07, 0x00, 0x04, &expTime_num);
	fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x07, 0x04, 0x04, &expTime_deno);

	exif->iso = iso;
	exif->exp_time_num = expTime_num;
	exif->exp_time_denom = expTime_deno;

	pr_info("%s --- ISO(%d), ET(%d/%d)\n", __func__, iso, expTime_num, expTime_deno);
}
//ASUS_BSP --- Stimber "[A60K][8M][NA][Other] Implement EXIF info for 8M camera with ISP"

// ASUS_BSP+++ Patrick "[ov2720] add calibration functionalities"
static int isp_cam_start(void)
{
	int err;
	char pFile[] = "/data/data/com.asus.atd.camera/files/factory_flicker_60";
	struct file *fp = NULL;

	pr_info("isp_cam_start\n");
	fp = filp_open(pFile, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	if ( !IS_ERR_OR_NULL(fp) ) {
		factory_mode = false;
		pr_info("Calibration flickless mode 60 Hz\n");
		filp_close(fp, NULL);
	} else {
		pr_info("Calibration flickless mode 50 Hz\n");
	}
	msleep(10);
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0F, 0x12, 0x01);
	err = isp_interrupt(INT_STATUS_MODE);
	if(err){
		pr_err("isp_cam_start error");
		return -ENOMEM;
	}
	else{
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x01, 0x24);
		return 0;
	}
}

#ifdef ASUS_FACTORY_BUILD
static int isp_calibration_init(u32 table)
{
	u32 temp;
	int err;

	err = ov2720_s_ctrl.func_tbl->sensor_power_up(&ov2720_s_ctrl);

	if(err<0){
		pr_err("isp power on error \n");
		return -ENOMEM;
	} else {
		if(g_A60K_hwID!=A60K_EVB) {
			gpio_direction_output(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect, 1);
			gpio_set_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect, 0);
		}
//		pr_info("gpio g_GPIO_8M_WP(%d)\n",gpio_get_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect));
	}

	err = isp_cam_start();
	if(err){
		pr_err("isp camera start error \n");
		return -ENOMEM;
	}
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x07, 0x00);
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0B, 0x00, 0x06);
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0E, 0x31, table-1);

	if(factory_mode == true)
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x06, 0x01);
	else
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x06, 0x02);

	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0E, 0x30, table);

	err = sensor_change_status_without_WQ(E_M6MO_Status_Monitor);
	if(err){
		fjm6mo_calibration_status = CALIBRATION_ISP_MONITOR_FAIL;
		pr_err("isp CALIBRATION_ISP_MONITOR_FAIL \n");
		return -ENOMEM;
	}

	msleep(3000);
	fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x3, 0xE, 0x02, &temp);
	if(temp > 0x0078){
		pr_err("Sensor gain:0x%x\n", temp);
		pr_err("isp CALIBRATION_LIGHT_SOURCE_FAIL \n");
		fjm6mo_calibration_status = CALIBRATION_LIGHT_SOURCE_FAIL;
	} else {
		fjm6mo_calibration_status = CALIBRATION_LIGHT_SOURCE_OK;
		pr_err("isp CALIBRATION_LIGHT_SOURCE_OK \n");
	}

	return 0;
}

static int isp_calibration_capture(u32 table)
{
	u32 temp=0;
	int err;

	err = sensor_change_status(E_M6MO_Status_Capture);
	if(err){
		pr_err("isp CALIBRATION_ISP_CAPTURE_FAIL \n");
		fjm6mo_calibration_status = CALIBRATION_ISP_CAPTURE_FAIL;
		return -ENOMEM;
	}
	err = sensor_change_status_without_WQ(E_M6MO_Status_Parameter);
	msleep(100);
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0E, 0x3C, table);
	msleep(500);
	fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0xE, 0x3D, 0x02, &temp);
	if(temp == 0xABCD) {
		pr_err("isp CALIBRATION_ISP_OK \n");
		fjm6mo_calibration_status = CALIBRATION_ISP_OK;
	} else {
		pr_err("Calibration checksum:0x%x\n", temp);
		fjm6mo_calibration_status = CALIBRATION_ISP_CHECKSUM_FAIL;
		pr_err("isp CALIBRATION_ISP_CHECKSUM_FAIL\n");
	}

	return 0;
}

static int isp_calibration_aging(u32 table)
{
	u32 temp=0;
	int err;

	err = ov2720_s_ctrl.func_tbl->sensor_power_up(&ov2720_s_ctrl);
	if(err){
		pr_err("isp CALIBRATION_ISP_POWERON_FAIL \n");
		fjm6mo_calibration_status = CALIBRATION_ISP_POWERON_FAIL;
		return -ENOMEM;
	} else {
		if(g_A60K_hwID!=A60K_EVB) {
			gpio_direction_output(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect, 1);
			gpio_set_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect, 0);
		}
//		pr_info("gpio g_GPIO_8M_WP(%d)\n",gpio_get_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect));
	}
	err = isp_cam_start();
	if(err){
		pr_err("isp CALIBRATION_ISP_INIT_FAIL \n");
		fjm6mo_calibration_status = CALIBRATION_ISP_INIT_FAIL;
		return -ENOMEM;
	}
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0E, 0x3C, table);
	msleep(500);
	fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0xE, 0x3D, 0x02, &temp);
	if(temp == 0xABCD) {
		pr_err("isp CALIBRATION_ISP_OK \n");
		fjm6mo_calibration_status = CALIBRATION_ISP_OK;
	} else {
		pr_err("Calibration checksum:0x%x\n", temp);
		fjm6mo_calibration_status = CALIBRATION_ISP_CHECKSUM_FAIL;
		pr_err("isp CALIBRATION_ISP_CHECKSUM_FAIL\n");
	}
	err = ov2720_s_ctrl.func_tbl->sensor_power_down(&ov2720_s_ctrl);

	return 0;
}

static int isp_calibration_golden_init(void)
{
	u32 temp;
	int err;

	err = ov2720_s_ctrl.func_tbl->sensor_power_up(&ov2720_s_ctrl);;
	if(err){
		pr_err("isp CALIBRATION_ISP_POWERON_FAIL \n");
		fjm6mo_calibration_status = CALIBRATION_ISP_POWERON_FAIL;
		return -ENOMEM;
	} else {
		if(g_A60K_hwID!=A60K_EVB) {
			gpio_direction_output(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect, 1);
			gpio_set_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect, 0);
		}
//		pr_info("gpio g_GPIO_8M_WP(%d)\n",gpio_get_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect));
	}
	err = isp_cam_start();
	if(err){
		pr_err("isp CALIBRATION_ISP_INIT_FAIL \n");
		fjm6mo_calibration_status = CALIBRATION_ISP_INIT_FAIL;
		return -ENOMEM;
	}
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0E, 0x27, 0xFFFF);
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0E, 0x2B, 0xFFFF);
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0E, 0x29, 0xFFFF);
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0E, 0x2D, 0xFFFF);
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x01, 0x24);

	if(factory_mode == true)
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x06, 0x01);
	else
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x06, 0x02);

	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x07, 0x00);

	err = sensor_change_status_without_WQ(E_M6MO_Status_Monitor);
    if(err){
		pr_err("isp CALIBRATION_ISP_MONITOR_FAIL \n");
		fjm6mo_calibration_status = CALIBRATION_ISP_MONITOR_FAIL;
		return -ENOMEM;
	}

	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0D, 0x47, 0x02);
	msleep(3000);
	fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x3, 0xE, 0x02, &temp);
	if(temp > 0x0078){
		pr_err("Sensor gain:0x%x\n", temp);
		pr_err("isp CALIBRATION_LIGHT_SOURCE_FAIL \n");
		fjm6mo_calibration_status = CALIBRATION_LIGHT_SOURCE_FAIL;
	} else {
		pr_err("isp CALIBRATION_LIGHT_SOURCE_OK \n");
		fjm6mo_calibration_status = CALIBRATION_LIGHT_SOURCE_OK;
	}

	return 0;
}

static int isp_calibration_golden_result(void)
{
	u32 temp;

	msleep(5000);
	fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0E, 0x1B, 0x01, &temp);
	if(temp != 0x01) {
		pr_err("Calibration golden read:0x%x\n", temp);
		pr_err("isp CALIBRATION_ISP_GOLDEN_FAIL \n");
		fjm6mo_calibration_status = CALIBRATION_ISP_GOLDEN_FAIL;
	} else {
		golden_value[0] = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0E, 0x3F, 0x01, &temp);
		golden_value[1] = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0E, 0x40, 0x01, &temp);
		golden_value[2] = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0E, 0x41, 0x01, &temp);
		golden_value[3] = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0E, 0x42, 0x01, &temp);
		fjm6mo_calibration_status = CALIBRATION_ISP_OK;
		pr_err("isp CALIBRATION_ISP_OK \n");
	}

	return 0;
}

static int isp_calibration_pgain_init(u32 val0, u32 val1, u32 val2, u32 val3)
{
	u32 temp;
	int err;

	err = ov2720_s_ctrl.func_tbl->sensor_power_up(&ov2720_s_ctrl);
	if(err){
		pr_err("isp CALIBRATION_ISP_POWERON_FAIL \n");
		fjm6mo_calibration_status = CALIBRATION_ISP_POWERON_FAIL;
		return -ENOMEM;
	} else {
		if(g_A60K_hwID!=A60K_EVB) {
			gpio_direction_output(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect, 1);
			gpio_set_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect, 0);
		}
//		pr_info("gpio g_GPIO_8M_WP(%d)\n",gpio_get_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect));
	}

	err = isp_cam_start();
	if(err){
		pr_err("isp CALIBRATION_ISP_INIT_FAIL \n");
		fjm6mo_calibration_status = CALIBRATION_ISP_INIT_FAIL;
		return -ENOMEM;
	}

	Golden_R_Gain = ((val0 & 0x00FF) << 8) | (val1 & 0x00FF);
	Golden_B_Gain = ((val2 & 0x00FF) << 8) | (val3 & 0x00FF);
	pr_info("Golden_R_Gain=%x, Golden_R_Gain=%x--", Golden_R_Gain, Golden_B_Gain);

	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0E, 0x27, Golden_R_Gain);
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0E, 0x2B, Golden_R_Gain);
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0E, 0x29, Golden_B_Gain);
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0E, 0x2D, Golden_B_Gain);
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x01, 0x24);

	if(factory_mode == true)
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x06, 0x01);
	else
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x06, 0x02);

	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x07, 0x00);

	err = sensor_change_status_without_WQ(E_M6MO_Status_Monitor);
	if(err){
		fjm6mo_calibration_status = CALIBRATION_ISP_MONITOR_FAIL;
		pr_err("isp CALIBRATION_ISP_MONITOR_FAIL \n");
		return -ENOMEM;
	}

	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0D, 0x47, 0x01);
	msleep(3000);
	fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x3, 0xE, 0x02, &temp);
	if(temp > 0x0078){
		pr_err("Sensor gain:0x%x\n", temp);
		pr_err("isp CALIBRATION_LIGHT_SOURCE_FAIL; \n");
		fjm6mo_calibration_status = CALIBRATION_LIGHT_SOURCE_FAIL;
	} else {
		pr_err("isp CALIBRATION_LIGHT_SOURCE_OK; \n");
		fjm6mo_calibration_status = CALIBRATION_LIGHT_SOURCE_OK;
	}

	return 0;
}

static int isp_calibration_pgain_retest(void)
{
	u32 temp;
	int err, l_RetryCount=0;
	u32 temp_R_gain=0x00000000, temp_B_gain=0x00000000;

	ov2720_s_ctrl.func_tbl->sensor_power_down(&ov2720_s_ctrl);
    
   msleep(100);
	err = ov2720_s_ctrl.func_tbl->sensor_power_up(&ov2720_s_ctrl);
	if(err){
		pr_err("isp CALIBRATION_ISP_POWERON_FAIL \n");
		return CALIBRATION_ISP_POWERON_FAIL;
	} else {
		if(g_A60K_hwID!=A60K_EVB) {
			gpio_direction_output(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect, 1);
			gpio_set_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect, 0);
		}
		pr_info("gpio g_GPIO_8M_WP(%d)\n",gpio_get_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect));
	}

	err = isp_cam_start();
	if(err){
		pr_err("isp CALIBRATION_ISP_INIT_FAIL \n");
		return CALIBRATION_ISP_INIT_FAIL;
	}
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0E, 0x27, Golden_R_Gain);
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0E, 0x2B, Golden_R_Gain);
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0E, 0x29, Golden_B_Gain);
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 2, 0x0E, 0x2D, Golden_B_Gain);
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x01, 0x24);

	if(factory_mode == true)
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x06, 0x01);
	else
		fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x03, 0x06, 0x02);

	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x01, 0x07, 0x00);

	err = sensor_change_status_without_WQ(E_M6MO_Status_Monitor);
	if(err){
		pr_err("isp CALIBRATION_ISP_MONITOR_FAIL \n");
        return CALIBRATION_ISP_MONITOR_FAIL;
	}

    // CMD Cat:0xD, 47, 5
	fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0D, 0x47, 0x05);

    // Sleep > 5sec
	msleep(7000);
    do {
        // Read Cat:0xE, 1B
	    fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0E, 0x1B, 0x01, &temp);
	    pr_info("Read register 0x0E, 0x1B, 0x01 result=%x", temp);
	    if(temp != 0x01) {
		    pr_err("Pgain check CAT:0x0E BYTE:0x1B, cnt: %d\n", l_RetryCount); 
            msleep(500);
	    } else {
           break;
         }
    } while (++l_RetryCount<5);

	if(temp != 0x01) {
		    pr_err("Pgain check CAT:0x0E BYTE:0x1B, FAIL result:0x%x != 0x01\n", temp);
		    pr_err("isp CALIBRATION_ISP_PGAIN_FAIL; \n");
		    return CALIBRATION_ISP_PGAIN_FAIL;
    }

    // Read pGain to check 3% variant
	fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0E, 0x3F, 0x02, &temp_R_gain);

	fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0E, 0x41, 0x02, &temp_B_gain);

	if((temp_R_gain*100>(Golden_R_Gain*(100+PGAIN_VARIANT)))||
		(temp_R_gain*100<(Golden_R_Gain*(100-PGAIN_VARIANT)))) {
		pr_err("Rgain=0x%x out of range\n", temp_R_gain);
		return CALIBRATION_ISP_PGAIN_FAIL;
	}

	if((temp_B_gain*100>(Golden_B_Gain*(100+PGAIN_VARIANT)))||
		(temp_B_gain*100<(Golden_B_Gain*(100-PGAIN_VARIANT)))) {
		pr_err("Bgain=0x%x out of range\n", temp_B_gain);
		return CALIBRATION_ISP_PGAIN_FAIL;
	}

	return 0;
}

static int isp_calibration_pgain_result(int wait_time)
{
	u32 temp;
	int err;

	msleep(wait_time * 1000);
	fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x0E, 0x1B, 0x01, &temp);
	if(temp == 0x05) { //Calibration OK
		pr_info("isp_calibration_pgain_retest\n");
		err = isp_calibration_pgain_retest();
		if(err==0) {
			pr_err("isp_calibration_pgain_retest OK; \n");
			fjm6mo_calibration_status = CALIBRATION_ISP_OK;
		} else {
           pr_err("isp_calibration_pgain_retest FAIL(%d)\n", err);
           fjm6mo_calibration_status = err;
        }
	} else {
		pr_err("isp_calibration_pgain_result fail:0x%x\n", temp);
		//pr_err("isp CALIBRATION_ISP_PGAIN_FAIL; \n");
		fjm6mo_calibration_status = CALIBRATION_ISP_PGAIN_FAIL;
	}

	return 0;
}
#endif //ASUS_FACTORY_BUILD
// ASUS_BSP--- Patrick "[ov2720] add calibration functionalities"

// Fjm6mo firmware -
static int i2c_set_open(struct inode *inode, struct file *file)
{
    file->private_data = inode->i_private;
    return 0;
}

static ssize_t i2c_camera(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int len;
    int arg[2];
    u32 result = 0, temp;

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end
//ASUS_BSP +++ LiJen "[A66][8M][NA][Others]add flash control by ISP debug file"
    sscanf(debugTxtBuf, "%d", &arg[0]);
    pr_info("0 is open_camera 1 is close_camera 2 is firmware update\n");
    pr_info("command is arg1=%d \n", arg[0]);
//ASUS_BSP --- LiJen "[A66][8M][NA][Others]add flash control by ISP debug file"	
    *ppos = len;

    switch(arg[0])
    {
        case 0:
        {
			pr_info("fjm6mo power_on\n");
			ov2720_s_ctrl.func_tbl->sensor_power_up(&ov2720_s_ctrl);
			break;
        }
        case 1:
        {
                     pr_info("fjm6mo power_off\n");
			ov2720_s_ctrl.func_tbl->sensor_power_down(&ov2720_s_ctrl);
			break;
        }
        case 2:
        {
			if(fjm6mo_update_status==0) {
				fjm6mo_update_status=1;
				pr_info("fjm6mo firmware update start\n");
				fw_update();
				fjm6mo_update_status=0;
			}
            break;
        }
        case 3:
        {
            pr_info("fjm6mo start and enter monitor mode\n");
            result = fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x0F, 0x12, 0x01);
            msleep(150);
            do{
                msleep(10);
                result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, 0x00, 0x1C, 0x01, &temp);
            } while((temp & 0x01) != 0x01);
            result = sensor_change_status(E_M6MO_Status_Monitor);
            break;
        }
        case 4:
        {
            pr_info("m6mo checksum\n");
            fw_checksum(&temp);
            break;
        }
        case 5:
        {
            pr_info("m6mo compare\n");
            //fw_compare();
            break;
        }	
        case 6:
        {
            pr_info("m6mo binary checksum\n");
            bin_checksum(&temp);
            break;
        }
        case 7:
        {
			pr_info("m6mo checkversion\n");
			fw_version();
			break;
        }
        case 8:
// ASUS_BSP+++ Patrick "for ISP firmware update functionatliy"
        {
	        	u32 BinVersionNum=0x11111111;
        		bin_fw_version(&BinVersionNum);
			pr_info("BinVersionNum=%x--\n", BinVersionNum);
			if(VersionNum!=BinVersionNum) {
				pr_info("VersionNum!=ISP_FIRMWARE_VERSION, len=%d--\n", len);
				len = NEED_UPDATE;
			} else {
				len = UPDATE_UNNECESSARY;
				pr_info("VersionNum==BinVersionNum=%x, len=%d--\n", BinVersionNum, len);
			}
			break;
		}
// ASUS_BSP--- Patrick "for ISP firmware update functionatliy"
//ASUS_BSP +++ LiJen "[A66][8M][NA][Others]add flash control by ISP debug file"
        case 10:
        {
            pr_info("fjm6mo power_on\n");
	     ov2720_s_ctrl.func_tbl->sensor_power_up(&ov2720_s_ctrl);
		 
            pr_info("fjm6mo start\n");
            isp_cam_start();

            pr_info("Led on\n");
            fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x39, 0x00);
            break;
        }
        case 11:
        {
            pr_info("Led off\n");
            fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x39, 0x03);
			
            pr_info("fjm6mo power_off\n");
	     ov2720_s_ctrl.func_tbl->sensor_power_down(&ov2720_s_ctrl);
            break;
        }
        case 12:
        {
            pr_info("fjm6mo power_on\n");
	     ov2720_s_ctrl.func_tbl->sensor_power_up(&ov2720_s_ctrl);
		 
            pr_info("fjm6mo start\n");
            isp_cam_start();
			
            pr_info("Flash on\n");
            fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, 1, 0x02, 0x39, 0x02);
            msleep(1000);
			
            pr_info("fjm6mo power_off\n");
	     ov2720_s_ctrl.func_tbl->sensor_power_down(&ov2720_s_ctrl);
            break;
        }
//ASUS_BSP --- LiJen "[A66][8M][NA][Others]add flash control by ISP debug file"	
        default:
            break;
    }

    return len;    /* the end */
}

static ssize_t i2c_camera_ack(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char *bp = debugTxtBuf;

	if (*ppos)
		return 0;	/* the end */
	len = snprintf(bp, DBG_TXT_BUF_SIZE, "the value is %d\n", fjm6mo_status);

	if (copy_to_user((void __user *)buf, (const void *)debugTxtBuf, (unsigned long)len))
		return -EFAULT;

	*ppos += len;
	return len;
}

// ASUS_BSP+++ Patrick "[ov2720] add calibration functionalities"
#ifdef ASUS_FACTORY_BUILD
static ssize_t i2c_calibration_shading(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	int len, wait_time=0;
	char light[10], cmd[10];
	int err;

	if (*ppos)
		return 0;    /* the end */


//+ parsing......
	len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
	if ( copy_from_user( debugTxtBuf, buf, len ) )
		return -EFAULT;

	debugTxtBuf[len] = 0; //add string end

	sscanf(debugTxtBuf, "%s %s", light, cmd);

	pr_info("command is light=%s cmd=%s\n", light, cmd);

	*ppos = len;

	if(strncmp(light, "2800", 4) == 0){
		if(strncmp(cmd, "start", 5) == 0)
			err = isp_calibration_init(1);
		else if(strncmp(cmd, "capture", 7) == 0)
			err = isp_calibration_capture(1);
		else if(strncmp(cmd, "aging", 7) == 0)
			err = isp_calibration_aging(1);
	}
	else if(strncmp(light, "3500", 4) == 0){
		if(strncmp(cmd, "start", 5) == 0)
			err = isp_calibration_init(2);
		else if(strncmp(cmd, "capture", 7) == 0)
			err = isp_calibration_capture(2);
		else if(strncmp(cmd, "aging", 7) == 0)
			err = isp_calibration_aging(2);
	}
	else if(strncmp(light, "5000", 4) == 0){
		if(strncmp(cmd, "start", 5) == 0)
			err = isp_calibration_init(3);
		else if(strncmp(cmd, "capture", 7) == 0)
			err = isp_calibration_capture(3);
		else if(strncmp(cmd, "aging", 7) == 0)
			err = isp_calibration_aging(3);
	}
	else if(strncmp(light, "golden", 6) == 0){
		if(strncmp(cmd, "start", 5) == 0)
			err = isp_calibration_golden_init();
		else if(strncmp(cmd, "capture", 7) == 0)
			err = isp_calibration_golden_result();
	}
	else if(strncmp(light, "pgain", 5) == 0){
		sscanf(debugTxtBuf, "%s %d", light, &wait_time);
		pr_info("wait_time=%d--\n", wait_time);
		err = isp_calibration_pgain_result(wait_time);
	}
	else if(strncmp(light, "shading", 7) == 0)
	{
		if(strncmp(cmd, "2800", 4) == 0)
			shading_table = 1;
		else if(strncmp(cmd, "3500", 4) == 0)
			shading_table = 2;
		else if(strncmp(cmd, "5000", 4) == 0)
			shading_table = 3;
	}

	return len;    /* the end */
}

static ssize_t i2c_calibration_pgain(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	int len;
	char light[10], cmd[10];
	int err;
	u32 val0 = 0xff;
	u32 val1 = 0xff;
	u32 val2 = 0xff;
	u32 val3 = 0xff;

	if (*ppos)
		return 0;    /* the end */

//+ parsing......
	len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
	if ( copy_from_user( debugTxtBuf, buf, len ) )
		return -EFAULT;

	debugTxtBuf[len] = 0; //add string end

	sscanf(debugTxtBuf, "%s %s 0x%X 0x%X 0x%X 0x%X", light, cmd, &val0, &val1, &val2, &val3);

	pr_info("command is light=%s cmd=%s\n", light, cmd);
	pr_info("golden value input is 0x%X 0x%X 0x%X 0x%X\n", val0, val1, val2, val3);

	*ppos = len;

	if(strncmp(light, "pgain", 4) == 0) {
		if(strncmp(cmd, "start", 5) == 0)
			err = isp_calibration_pgain_init(val0, val1, val2, val3);
	}

	return len;    /* the end */
}

static int i2c_calibration_ack(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int len = 0;
	char *bp = debugTxtBuf;

	if (*ppos)
	return 0;    /* the end */

    // 0:Fail 1:Pass
    // 10:power 11:init 12:monitor 13:light 14:capture
    // 15:checksum 16:pgain 17:golden

	switch(fjm6mo_calibration_status)
	{
        case CALIBRATION_ISP_POWERON_FAIL:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 10\n");
            break;
        case CALIBRATION_ISP_INIT_FAIL:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 11\n");
            break;
        case CALIBRATION_ISP_MONITOR_FAIL:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 12\n");
            break;
        case CALIBRATION_LIGHT_SOURCE_FAIL:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 13\n");
            break;
        case CALIBRATION_LIGHT_SOURCE_OK:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "1 0\n");
            break;
        case CALIBRATION_ISP_CAPTURE_FAIL:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 14\n");
            break;
        case CALIBRATION_ISP_CHECKSUM_FAIL:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 15\n");
            break;
        case CALIBRATION_ISP_PGAIN_FAIL:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 16\n");
            break;
        case CALIBRATION_ISP_GOLDEN_FAIL:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 17\n");
            break;
        case CALIBRATION_ISP_OK:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "1 1\n");
            break;
        default:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 0\n");
            break;
	}

	if (copy_to_user(buf, debugTxtBuf, len))
		return -EFAULT;

	*ppos += len;
	return len;
}

static int i2c_calibration_golden_value(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int len = 0;
	char *bp = debugTxtBuf;

	if (*ppos)
		return 0;    /* the end */

	len = snprintf(bp, DBG_TXT_BUF_SIZE, "0x%X 0x%X 0x%X 0x%X\n", golden_value[0], golden_value[1], golden_value[2], golden_value[3]);

	pr_info("golden value=%s--\n", bp);
	if (copy_to_user(buf, debugTxtBuf, len))
		return -EFAULT;

	*ppos += len;
	return len;
}
#endif //ASUS_FACTORY_BUILD
// ASUS_BSP--- Patrick "[ov2720] add calibration functionalities"

static ssize_t i2c_camera_read(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    ssize_t len;
    u32 arg[3];
    //int ret=-EINVAL;
    u32 result = 0, temp;

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end

    sscanf(debugTxtBuf, "%x %x %x", &arg[0], &arg[1], &arg[2]);
    pr_info("command is cat=%x byte=%x number=%x \n", arg[0], arg[1], arg[2]);

    *ppos = len;

    result = fjm6mo_read_register(ov2720_s_ctrl.sensor_i2c_client->client, arg[0], arg[1], arg[2], &temp);
    pr_info("register value: 0x%x\n", temp);

    return len;    /* the end */
}

static ssize_t i2c_camera_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    ssize_t len;
    u32 arg[4];

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end

    sscanf(debugTxtBuf, "%x %x %x %x", &arg[0], &arg[1], &arg[2], &arg[3]);
    pr_info("command is number=%x cat=%x byte=%x value=%x\n", arg[0], arg[1], arg[2], arg[3]);

    *ppos = len;

    fjm6mo_write_register(ov2720_s_ctrl.sensor_i2c_client->client, arg[0], arg[1], arg[2], arg[3]);

    return len;    /* the end */
}

static ssize_t i2c_camera_read_memory(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    ssize_t len;
    u32 arg[4];
    //int ret=-EINVAL;
    u32 result = 0, temp = 0;

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end

    sscanf(debugTxtBuf, "%x %x", &arg[0], &arg[1]);
    pr_info("command is address=%x number=%x \n", arg[0], arg[1]);

    *ppos = len;

    result = fjm6mo_read_memory(ov2720_s_ctrl.sensor_i2c_client->client, arg[0], arg[1], &temp);
    if(result == 0){
    	pr_info("memory read value at 0x%x: 0x%x\n", arg[0], temp);
    }else{
	pr_info("memory read fail\n");
    }
			
    return len;    /* the end */
}

static ssize_t i2c_camera_write_memory(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    ssize_t len;
    u32 arg[5];
    //int ret=-EINVAL;
    u32 result = 0;

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end

    sscanf(debugTxtBuf, "%x %x %x %x", &arg[0], &arg[1], &arg[2], &arg[3]);
    pr_info("command is value=%x byte_size=%x address=%x write_size:%x\n", arg[0], arg[1], arg[2], arg[3]);

    *ppos = len;

    result= fjm6mo_write_memory(ov2720_s_ctrl.sensor_i2c_client->client, (u8*)arg, arg[1], arg[2], arg[3]);
    pr_info("memory write value at 0x%x: 0x%x\n", arg[2], arg[0]);

    return len;    /* the end */
}

static int i2c_firmware_status(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	static int ret = -1;
// ASUS_BSP+++ Patrick "[A60K][8M][NA][Other] add ISP firmware version"
	int len = 0;
	char *update_version_file_path = "/data/fw_update_version";
	struct file *filePtr = NULL;
	loff_t pos = 0;
	char version_num[VERSION_STATUS_LENGTH];
	mm_segment_t old_fs;
	u32 BinVersionNum=0;

	bin_fw_version(&BinVersionNum);
// ASUS_BSP--- Patrick "[A60K][8M][NA][Other] add ISP firmware version"

	if((VersionNum==0xffffff)||(VersionNum<BinVersionNum)) {
		pr_info("Firmware need NOT update VersionNum=ISP_FIRMWARE_VERSION=%x, return=%x--\n",
			VersionNum, UPDATE_UNNECESSARY);
// ASUS_BSP+++ Patrick "[A60K][8M][NA][Other] add ISP firmware version"
		len = sprintf(version_num, "%x %x\n%x\n", UPDATE_UNNECESSARY, VersionNum, BinVersionNum);

		filePtr = filp_open(update_version_file_path, O_RDWR, S_IRWXU|S_IRWXG|S_IRWXO);

//		filePtr = filp_open(update_version_file_path, O_CREAT, S_IWUSR|S_IRUSR);
		if(!IS_ERR_OR_NULL(filePtr)) {
			old_fs = get_fs();
			set_fs(KERNEL_DS);

			ret = filePtr->f_op->write(filePtr, version_num, len*sizeof(char), &pos);

			set_fs(old_fs);
			filp_close(filePtr, NULL);
//			pr_info("In %s, version_num=%s, ppos=%d, len=%d, ret=%d--", __func__, version_num, (int)pos, len, ret);
		} else {
			pr_info("File open fail!!, progress filePtr=%d--\n", (int)filePtr);
		}
// ASUS_BSP--- Patrick "[A60K][8M][NA][Other] add ISP firmware version"

		ret = UPDATE_UNNECESSARY;
		*buf = UPDATE_UNNECESSARY;
	} else {
		pr_info("Firmware need update VersionNum=%x, ISP_FW_V=%x, return=%x--\n",
			VersionNum, BinVersionNum, NEED_UPDATE);
// ASUS_BSP+++ Patrick "[A60K][8M][NA][Other] add ISP firmware version"
		len = sprintf(version_num, "%x %x\n%x\n", NEED_UPDATE, VersionNum, BinVersionNum);
		pr_info("In %s, version_num=%s, ppos=%d--", __func__, version_num, (int)pos);

		filePtr = filp_open(update_version_file_path, O_RDWR, S_IRWXU|S_IRWXG|S_IRWXO);
//		filePtr = filp_open(update_version_file_path, O_CREAT, S_IWUSR|S_IRUSR);
		if(!IS_ERR_OR_NULL(filePtr)) {
			old_fs = get_fs();
			set_fs(KERNEL_DS);

			ret = filePtr->f_op->write(filePtr, version_num, len*sizeof(char), &pos);

			set_fs(old_fs);
			filp_close(filePtr, NULL);
//			pr_info("In %s, version_num=%s, ppos=%d, len=%d, ret=%d--", __func__, version_num, (int)pos, len, ret);
		} else {
			pr_info("File open fail!!, progress filePtr=%d--\n", (int)filePtr);
		}
// ASUS_BSP--- Patrick "[A60K][8M][NA][Other] add ISP firmware version"
		ret = NEED_UPDATE;
		*buf = NEED_UPDATE;
	}

	return ret;
}

//ASUS_BSP +++ LiJen "[A66][8M][NA][Others]add flash control debug file"
static ssize_t gpio_flash(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int len;
    int arg[2];
    int ret=-EINVAL;

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end

    sscanf(debugTxtBuf, "%x", &arg[0]);
    //pr_info("0 is enable flash, 1 is disable flash 2\n");
    //pr_info("command is arg1=%x \n", arg[0]);	
	
    *ppos = len;

    switch(arg[0])
    {
        case 0:
        {
		pr_info("Flash off\n");
		gpio_set_value(FLASH_GPIO, 0);	
		gpio_free(FLASH_GPIO);
		break;
        }
        case 1:
        {
		pr_info("Flash on\n");
    		ret=gpio_request(FLASH_GPIO, "ov2720");
		if (ret) {
			pr_err("%s: request Flash GPIO failed rc(%d)\n",__func__, ret);
    		}	
		msleep(10);
		gpio_set_value(FLASH_GPIO, 1);		
		break;
        }	
        default:
            break;
    }

    return len;    /* the end */
}
//ASUS_BSP --- LiJen "[A66][8M][NA][Others]add flash control debug file"

// ASUS_BSP+++ Patrick "[A66][8M][NA][Other]add debug file to set or read ISP write protect status"
static ssize_t isp_set_camera_write_protect(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int len;
    int arg[2];

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end
    sscanf(debugTxtBuf, "%x", &arg[0]);
    //pr_info("0 is disable write protect, 1 is enable write protect\n");
    //pr_info("command is arg=%x \n", arg[0]);

    *ppos = len;

    switch(arg[0])
    {
        case 0:
        {
		gpio_set_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect, 0);
		pr_info("disable write protect\n");
		break;
        }
        case 1:
        {
		gpio_set_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect, 1);
		pr_info("enable write protect\n");
		break;
        }
        default:
		pr_info("Illegal argument ?!\n0 is disable write protect, 1 is enable write protect\n");
            break;
    }

    return len;    /* the end */
}

static ssize_t isp_show_camera_write_protect(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	char *bp = debugTxtBuf;

	if (*ppos)
		return 0;	/* the end */
	len = snprintf(bp, DBG_TXT_BUF_SIZE, "Write Protect value is %d\n",
			gpio_get_value(ov2720_s_ctrl.sensordata->sensor_platform_info->cam_write_protect));

	if (copy_to_user((void __user *)buf, (const void *)debugTxtBuf, (unsigned long)len))
		return -EFAULT;

	*ppos += len;
	return len;
}
// ASUS_BSP--- Patrick "[A66][8M][NA][Other]add debug file to set or read ISP write protect status"

//Fjm6mo late init to update firmware -

// Fjm6mo debugfs +
static const struct file_operations i2c_open_camera = {
    .open = i2c_set_open,
    .write = i2c_camera,
};

// ASUS_BSP+++ Patrick "[ov2720] add calibration functionalities"
#ifdef ASUS_FACTORY_BUILD
static const struct file_operations isp_calibration_shading = {
    .open = i2c_set_open,
    .write = i2c_calibration_shading,
};
static const struct file_operations isp_calibration_pgain = {
    .open = i2c_set_open,
    .write = i2c_calibration_pgain,
};
static const struct file_operations calibration_status = {
    .open = i2c_set_open,
    .read = i2c_calibration_ack,
};
static const struct file_operations calibration_golden_value = {
    .open = i2c_set_open,
    .read = i2c_calibration_golden_value,
};
#endif //ASUS_FACTORY_BUILD
// ASUS_BSP--- Patrick "[ov2720] add calibration functionalities"

static const struct file_operations camera_status = {
    .open = i2c_set_open,
    .read = i2c_camera_ack,
};
static const struct file_operations i2c_read_register = {
    .open = i2c_set_open,
    .write = i2c_camera_read,
};
static const struct file_operations i2c_write_register = {
    .open = i2c_set_open,
    .write = i2c_camera_write,
};
static const struct file_operations i2c_read_memory = {
    .open = i2c_set_open,
    .write = i2c_camera_read_memory,
};
static const struct file_operations i2c_write_memory = {
    .open = i2c_set_open,
    .write = i2c_camera_write_memory,
};
static const struct file_operations isp_firmware_update_status = {
    .open = i2c_set_open,
    .read = i2c_firmware_status,
};
//ASUS_BSP +++ LiJen "[A66][8M][NA][Others]add flash control debug file"
static const struct file_operations gpio_enable_flash = {
    .open = i2c_set_open,
    .write = gpio_flash,
};
//ASUS_BSP --- LiJen "[A66][8M][NA][Others]add flash control debug file"

// ASUS_BSP+++ Patrick "[A66][8M][NA][Other]add debug file to set or read ISP write protect status"
static const struct file_operations isp_write_protect = {
    .open = i2c_set_open,
    .write = isp_set_camera_write_protect ,
    .read = isp_show_camera_write_protect ,
};
// ASUS_BSP--- Patrick "[A66][8M][NA][Other]add debug file to set or read ISP write protect status"

int fjm6mo_i2c_debuginit(void)
{
    struct dentry *dent = debugfs_create_dir("fjm6mo", NULL);
//ASUS_BSP +++ LiJen "[A66][8M][NA][Others]add flash control debug file"
    if(g_A60K_hwID >= A66_HW_ID_SR1_1) {
    	struct dentry *dent_flash = debugfs_create_dir("flash", NULL);
    	(void) debugfs_create_file("gpio_enable_flash", 0700,
            dent_flash, NULL, &gpio_enable_flash);
    }	
//ASUS_BSP --- LiJen "[A66][8M][NA][Others]add flash control debug file"

// ASUS_BSP+++ Patrick "[ov2720] add calibration functionalities"
#ifdef ASUS_FACTORY_BUILD
    (void) debugfs_create_file("isp_calibration_shading", 0775,
            dent, NULL, &isp_calibration_shading);
    (void) debugfs_create_file("isp_calibration_pgain", 0775,
            dent, NULL, &isp_calibration_pgain);
    (void) debugfs_create_file("calibration_status", 0775,
            dent, NULL, &calibration_status);
    (void) debugfs_create_file("calibration_golden_value", 0775,
            dent, NULL, &calibration_golden_value);
#endif //ASUS_FACTORY_BUILD
// ASUS_BSP--- Patrick "[ov2720] add calibration functionalities"

    (void) debugfs_create_file("i2c_open_camera", 0775,
            dent, NULL, &i2c_open_camera);
    (void) debugfs_create_file("camera_status", 0700,
            dent, NULL, &camera_status);
    (void) debugfs_create_file("isp_firmware_status", 0775,
            dent, NULL, &isp_firmware_update_status);
    (void) debugfs_create_file("i2c_read_register", 0700,
            dent, NULL, &i2c_read_register);
    (void) debugfs_create_file("i2c_write_register", 0700,
            dent, NULL, &i2c_write_register);
    (void) debugfs_create_file("i2c_read_memory", 0700,
            dent, NULL, &i2c_read_memory);
    (void) debugfs_create_file("i2c_write_memory", 0700,
            dent, NULL, &i2c_write_memory);

// ASUS_BSP+++ Patrick "[A66][8M][NA][Other]add debug file to set or read ISP write protect status"
    (void) debugfs_create_file("isp_wp", 0700,
            dent, NULL, &isp_write_protect);
// ASUS_BSP--- Patrick "[A66][8M][NA][Other]add debug file to set or read ISP write protect status"

    debugfs_create_x32("flag",0755, dent, &s_flag);

    return 0;
}

int fjm6mo_sensor_release(void)
{
	shading_table = 0;
	return 0;
}

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
