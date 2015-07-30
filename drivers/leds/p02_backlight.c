
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
static struct i2c_client *p02_backlight_client;
bool read_scaler_version(u8 *ver);
static bool write_scaler_version(u8);
extern bool i2c_error;
extern void reportPadStationI2CFail(char *devname);
int p02_backlight_write(u16 val)   
{
    unsigned char buf[7];
    int ret;
    int address = 0x6E;
    int i;
    char p02_backlight[] = "p02_backlight";

    buf[0] = 0X51;//Source address
    buf[1] = 0x84;//lenght
    buf[2] = 0x03;//set vcp feature
    buf[3] = 0x10;//vcp opcode
    buf[4] = (val >> 8) & 0xFF;//high byte
    buf[5] = val & 0xFF;//low byte
    buf[6] = address ^ buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5] ;
    
    msleep(50);
    i2c_error = false;
    //printk("p02_backlight_write checksum = 0x%x\n",buf[6]);
    ret = i2c_master_send(p02_backlight_client, buf, 7);
    if (i2c_error == false){
        for (i=1;i<=5;i++){
            if (ret < 0){
                msleep(100);
                printk("p02_backlight_write I2C fail retry %d\n",i);
                ret = i2c_master_send(p02_backlight_client, buf, 7);
            }
            else
                return ret;
        }
    }
    i2c_error = true;
    reportPadStationI2CFail(p02_backlight);

    return 0;

}
EXPORT_SYMBOL_GPL(p02_backlight_write);

int read_backlight_checksum(void)   
{
    int ret;
    int i;
    int address =0x6E;
    unsigned char checksum = 0x50;
    unsigned char value[12];
    unsigned char buf[5];

    buf[0] = 0x51;//source Address
    buf[1] = 0x82;//length
    buf[2] = 0x01;//Get VCP 
    buf[3] = 0xE3;//LCD on/off control
    buf[4] = address ^ buf[0] ^ buf[1] ^ buf[2] ^ buf[3];//checksum
    //msleep(100);   
    ret = i2c_master_send(p02_backlight_client, buf, 5);
    msleep(100);
    ret =  i2c_master_recv(p02_backlight_client, value ,12);

    for (i=0;i<=9;i++)
        checksum = checksum ^ value[i];

    if (checksum == value[10])
    {
        ret = 1;//setting scalar is successful
        //printk("setting scalar is successful\n");
    }
    else
    {
        ret = 0;//setting scalar is fail
        //printk("setting scalar is fail\n");
    }

        return ret;
}
EXPORT_SYMBOL_GPL(read_backlight_checksum);

int switch_backlight(int on)
{
    unsigned char buf[7];
    int ret;
    int address = 0x6E;
    int i;
    buf[0] = 0X51;//Source address
    buf[1] = 0x84;//lenght
    buf[2] = 0x03;//set vcp feature
    buf[3] = 0xE6;//vcp opcode
    buf[4] = 0x00;//high byte
    buf[5] = on;//low byte
    buf[6] = address ^ buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5] ;
    //msleep(100);
    i2c_error = false;
    ret = i2c_master_send(p02_backlight_client, buf, 7);
    if (i2c_error == false){
        for (i=1;i<=5;i++){
            if (ret < 0){
                msleep(100);
                printk("switch_backlight I2C fail retry %d\n",i);
                ret = i2c_master_send(p02_backlight_client, buf, 7);
            }
            else
                return ret;
        }
    }
    i2c_error = true;
    reportPadStationI2CFail("p02_backlight");
    if (ret < 0)
        return ret;

    return 0;

}
EXPORT_SYMBOL_GPL(switch_backlight);

int switch_backlight_and_panel(int on)
{
    unsigned char buf[7];
    int ret;
    int address = 0x6E;
    int i;
    buf[0] = 0X51;//Source address
    buf[1] = 0x84;//lenght
    buf[2] = 0x03;//set vcp feature
    buf[3] = 0xE3;//vcp opcode
    buf[4] = 0x00;//high byte
    buf[5] = on;//low byte
    buf[6] = address ^ buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5] ;
    //msleep(100);
    i2c_error = false;
    ret = i2c_master_send(p02_backlight_client, buf, 7);
    if (i2c_error == false){
        for (i=1;i<=5;i++){
            if (ret < 0){
                msleep(100);
                printk("switch_backlight I2C fail retry %d\n",i);
                ret = i2c_master_send(p02_backlight_client, buf, 7);
            }
            else
                return ret;
        }
    }
    i2c_error = true;
    reportPadStationI2CFail("p02_backlight");
    if (ret < 0)
        return ret;

    return 0;

}
EXPORT_SYMBOL_GPL(switch_backlight_and_panel);

int get_backlight(void)
{
    unsigned char buf[7];
    int ret;
    int address = 0x6E;
    unsigned char value[11];

    buf[0] = 0X51;//Source address
    buf[1] = 0x82;//lenght
    buf[2] = 0x01;//set vcp feature
    buf[3] = 0xE3;//vcp opcode
    buf[4] = address ^ buf[0] ^ buf[1] ^ buf[2] ^ buf[3];

    ret =  i2c_master_send(p02_backlight_client, buf, 5);
    if (ret < 0)
    {
        printk("[Scaler] get_backlight I2C fail\n");
        return ret;
    }

    msleep(100);    
    ret =  i2c_master_recv(p02_backlight_client, value ,11);
    if (ret < 0)
    {
        printk("[Scaler] get_backlight I2C fail\n");
        return ret;
    }

    if (value[4] == 0xE3)
        printk("[Scaler] get_backlight status is %s (%d)\n", value[9] ? "ON" : "OFF", value[9]);
    else
        printk("[Scaler] get_backlight status return wrong vcp code\n");

    return value[9];
}
EXPORT_SYMBOL_GPL(get_backlight);

int scaler_set_to_suspend(int enter_suspend)
{
    unsigned char buf[7];
    int ret;
    int address = 0x6E;
    int i;
    buf[0] = 0X51;//Source address
    buf[1] = 0x84;//lenght
    buf[2] = 0x03;//set vcp feature
    buf[3] = 0xE7;//vcp opcode
    buf[4] = 0x00;//high byte
    buf[5] = enter_suspend;//low byte
    buf[6] = address ^ buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5] ;

    for (i=0;i<5;i++)
    {
        ret = i2c_master_send(p02_backlight_client, buf, 7);
        if (ret < 0)
        {
            msleep(100);
            printk("scaler_set_to_suspend I2C fail retry %d\n",i);
        }
        else
        {
			printk("scaler_set_to_suspend success\n");
            break;
		}
    }
    //if (ret < 0)
        //reportPadStationI2CFail("p02_backlight");

    return ret;
}
EXPORT_SYMBOL_GPL(scaler_set_to_suspend);


int scaler_set_waiting_icon(int set)
{
    unsigned char buf[7];
    int ret;
    int address = 0x6E;
    int i;
    buf[0] = 0X51;//Source address
    buf[1] = 0x84;//lenght
    buf[2] = 0x03;//set vcp feature
    buf[3] = 0xE8;//vcp opcode
    buf[4] = 0x00;//high byte
    buf[5] = set;//low byte
    buf[6] = address ^ buf[0] ^ buf[1] ^ buf[2] ^ buf[3] ^ buf[4] ^ buf[5] ;

    for (i=0;i<5;i++)
    {
        ret = i2c_master_send(p02_backlight_client, buf, 7);
        if (ret < 0)
        {
            msleep(100);
            printk("scaler_set_waiting_icon I2C fail retry %d\n",i);
        }
        else
        {
			printk("scaler_set_waiting_icon success\n");
            break;
		}
    }
    //if (ret < 0)
        //reportPadStationI2CFail("p02_backlight");

    return ret;
}
EXPORT_SYMBOL_GPL(scaler_set_waiting_icon);


//Louis ++
#define SCALER_ASUS_VERSION_FILE        "/data/asusdata/scalar_FW.nv"
static int scaler_err1 = 0, scaler_err2 = 0;
bool read_scaler_version(u8 *ver)
{
    unsigned char buf[5];
    int address = 0x6E;
    unsigned char value[11];
//    int i;

    buf[0] = 0x51;
    buf[1] = 0x82;  
    buf[2] = 0x01;
    buf[3] = 0xE1;      //VCP code:FW version
    buf[4] = address ^ buf[0] ^ buf[1] ^ buf[2] ^ buf[3];

    scaler_err1 =  i2c_master_send(p02_backlight_client, buf, 5);
    msleep(100);    

    scaler_err2 =  i2c_master_recv(p02_backlight_client, value ,11);
    //msleep(50);

    //printk("[Scaler]error code1=%d ;error code2=%d\n",scaler_err1, scaler_err2);

    //for(i=0;i<=11;i++)
    //    printk("[P02] value[%d]= 0x%x\n",i, value[i]); 

    if( (scaler_err1 < 0) || (scaler_err2 < 0)  || (value[4] != 0xE1)) 
    {
		*ver = 0;
	}
	else
	{
		*ver = value[9];
	}

    if( !write_scaler_version(value[9]) ) {
        printk("write scaler version fail \n");   
        return false;
    }
    return true;
    
}
EXPORT_SYMBOL_GPL(read_scaler_version);
  
static bool write_scaler_version(u8 FW)
{
    struct file *fp = NULL; 
    loff_t pos_lsts = 0;
    char writestr[1];
    mm_segment_t old_fs;
    char msg[8]="unknown";

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    fp = filp_open(SCALER_ASUS_VERSION_FILE, O_RDWR|O_CREAT|O_TRUNC, 0644);
    if(IS_ERR_OR_NULL(fp)) {
        printk("[P02 FW] write_scaler_version open (%s) fail\n", SCALER_ASUS_VERSION_FILE);
        return false;
    }

    sprintf(writestr, "0x%2x", FW);

    printk("[P02 FW] write_P02_FW = %d, %s, %d]\n", FW, writestr, strlen(writestr));

    if( (scaler_err1 < 0) || (scaler_err2 < 0) ) {
         fp->f_op->write(fp, msg, strlen(msg), &pos_lsts);
         return false;
    }

    if (fp->f_op != NULL && fp->f_op->write != NULL){
        pos_lsts = 0;

        fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);

    }

    else {
        printk("[P02 FW] write_P02_FW fail\n");
        return false;
    }    
    
    set_fs(old_fs);
    filp_close(fp, NULL);

    return true;
}
//Louis--

#ifdef CONFIG_I2C_STRESS_TEST
#include <linux/i2c_testcase.h>
static int Test_p02_scalar_I2C (struct i2c_client *Client)
{
    unsigned char buf[5];
    int ret;
    unsigned char value[11];
    int address =0x6F;
    buf[0] = 0x51;//source Address
    buf[1] = 0x82;//length
    buf[2] = 0x01;//Get VCP
    buf[3] = 0xDF;//VCP Version
    buf[4] = address ^ buf[0] ^ buf[1] ^ buf[2] ^ buf[3];//checksum
    ret = i2c_master_send(p02_backlight_client, buf, 5);
    msleep(50);
    ret =  i2c_master_recv(p02_backlight_client, value ,11);
    msleep(50);

    if (value[8]==0x2 && value[9]==0x1)
        printk("p02 scalar test OK!!!\n");

    return 0;
}

static struct i2c_test_case_info p02_scalar_TestCaseInfo[] =
{
     __I2C_STRESS_TEST_CASE_ATTR(Test_p02_scalar_I2C),
};

#endif

static int p02_backlight_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
#ifdef CONFIG_I2C_STRESS_TEST

    printk("\n");

    i2c_add_test_case(client, "P02ScalarTest",ARRAY_AND_SIZE(p02_scalar_TestCaseInfo));

    printk("\n");

#endif
    printk("[Backlight] p02_backlight_probe\n");
    p02_backlight_client = client;
    
    return 0;
}

static const struct i2c_device_id p02_backlight_id[] = {
    { "p02_backlight", 0 },
    { }
};

static struct i2c_driver a60k_P01_backlight = {
    .driver = {
        .name   = "p02_backlight",
    },
    .id_table   = p02_backlight_id,
    .probe      = p02_backlight_probe,

};

static int __init p01_backlight_init(void)
{
    printk(  "[Audio] p01_backlight_init\n");
    
    return i2c_add_driver(&a60k_P01_backlight);
}

static void __exit p01_backlight_exit(void)
{
    printk(  "[Audio] p01_backlight_exit\n");
    i2c_del_driver(&a60k_P01_backlight);
    
}

module_init(p01_backlight_init);
module_exit(p01_backlight_exit);

MODULE_DESCRIPTION("p01 backlight Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tim_su");


