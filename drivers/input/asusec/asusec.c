/* 
 * ASUS EC driver.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/gpio_event.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <asm/gpio.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/power_supply.h>
#include <linux/microp_notify.h>
#include <linux/microp_pin_def.h>
#include <linux/asus_ec_info.h>

#include "asusec.h"
#include "elan_i2c_asus.h"
#include <linux/microp_api.h>
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

// ASUS_BSP Jiunhau_Wang proc control file 20110909+++
#include <linux/proc_fs.h>

#define ASUS_EC_PROC_FILE  "driver/asus_ec"
#define ASUS_EC_PROC_FILE_PERMISSION  0644
#define ASUS_EC_PROC_MAX_BUFF_SIZE  256
static struct proc_dir_entry *ec_proc_file;
// ASUS_BSP Jiunhau_Wang proc control file 20110909---

extern void pet_watchdog(void);

// ASUS_BSP Eason_Chang +++
#ifdef CONFIG_BATTERY_ASUS
extern void setDockInitNotReady(void);
#endif
// ASUS_BSP Eason_Chang ---

#ifdef CONFIG_P02_BACKLIGHT
extern int query_pad_backlight_state(void);
#endif

extern void reportPadStationDockI2CFail(void);
extern int micropSendNotify(unsigned long val);

/*
 * functions declaration
 */
static int asusec_i2c_write_data(struct i2c_client *client, u16 data);
static int asusec_i2c_read_data(struct i2c_client *client);
static void asusec_reset_dock(void);
static int asusec_chip_init(struct i2c_client *client);
static void asusec_work_function(struct work_struct *dat);
static void asusec_dock_init_work_function(struct work_struct *dat);
static void asusec_fw_update_work_function(struct work_struct *dat);
static void asusec_i2c_err_check_work_function(struct work_struct *dat);
static void asusec_resume_work_function(struct work_struct *dat);
static void asusec_bat_work_function(struct work_struct *dat);
static int __devinit asusec_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
static int __devexit asusec_remove(struct i2c_client *client);
static int asusec_kp_key_mapping(int x);
static ssize_t asusec_show_dock(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asusec_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asusec_info_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asusec_lid_status(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asusec_store_led(struct device *class,
		struct device_attribute *attr,const char *buf, size_t count);
static ssize_t asusec_store_ec_wakeup(struct device *class,
		struct device_attribute *attr,const char *buf, size_t count);
static ssize_t asusec_show_drain(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asusec_show_dock_battery(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asusec_show_dock_temperature(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asusec_show_dock_battery_all(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t asusec_show_dock_control_flag(struct device *class,
		struct device_attribute *attr,char *buf);

static int asusec_keypad_get_response(struct i2c_client *client, int res);
static int asusec_keypad_enable(struct i2c_client *client);
static int asusec_keypad_reset(struct i2c_client *client);
static int asusec_touchpad_get_response(struct i2c_client *client, int res);
static int asusec_touchpad_enable(struct i2c_client *client);
static int asusec_touchpad_disable(struct i2c_client *client);
static int asusec_touchpad_reset(struct i2c_client *client);
static int asusec_suspend(struct i2c_client *client, pm_message_t mesg);
static int asusec_resume(struct i2c_client *client);
static int asusec_open(struct inode *inode, struct file *flip);
static int asusec_release(struct inode *inode, struct file *flip);
static long asusec_ioctl(struct file *flip, unsigned int cmd, unsigned long arg);
static ssize_t ec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);
static ssize_t ec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos);
static void BuffPush(char data);
static int BuffDataSize(void);
static int asusec_input_device_create(struct i2c_client *client);
static int asusec_lid_input_device_create(struct i2c_client *client);
static ssize_t asusec_switch_name(struct switch_dev *sdev, char *buf);
static ssize_t asusec_switch_state(struct switch_dev *sdev, char *buf);
static ssize_t asusec_lid_switch_name(struct switch_dev *sdev, char *buf);
static ssize_t asusec_lid_switch_state(struct switch_dev *sdev, char *buf);
static int asusec_event(struct input_dev *dev, unsigned int type, unsigned int code, int value);
static void asusec_sw_plug_in_out_dock(void);
void EC_Dock_out_callback(void);
void EC_Dock_in_callback(void);
int isHWDockConnected(void);

int asusec_release_input_event(void);
int asusec_close_keyboard(void);
int asusec_dock_battery_callback(void);
int asusec_suspend_pre_process_callback(bool);
int asusec_suspend_hub(void);
int asusec_sus_res_callback(bool);
int isKeyboardWakeup(void);
void P02VirtualPWRKey(void);

// ASUS_BSP Jiunhau_Wang proc control file 20110909+++
void asus_ec_keyboard_control_cmd(const char *msg);
void asus_ec_touchpad_control_cmd(const char *msg);
void asus_ec_bat_control_cmd(const char *msg);
int asus_ec_version_get_cmd(void);
static void asus_ec_diag_handle_all_cmd(const char *msg);
static ssize_t asus_ec_read_proc(char *page, char **start, off_t off,
int count, int *eof, void *data);
static ssize_t asus_ec_write_proc(struct file *filp, const char __user *buff,
     unsigned long len, void *data);
static void asus_ec_create_ec_proc_file(void);
// ASUS_BSP Jiunhau_Wang proc control file 20110909---

/*
 * global variable
 */
static char host_to_ec_buffer[EC_BUFF_LEN];
static char ec_to_host_buffer[EC_BUFF_LEN];
static int h2ec_count;
static int buff_in_ptr;	  // point to the next free place
static int buff_out_ptr;	  // points to the first data

struct i2c_client dockram_client;
static struct class *asusec_class;
static struct device *asusec_device ;
struct asusec_chip *ec_chip;

struct cdev *asusec_cdev ;
static dev_t asusec_dev ;
static int asusec_major = 0 ;
static int asusec_minor = 0 ;

static struct workqueue_struct *asusec_wq;
struct delayed_work asusec_stress_work;

static const struct i2c_device_id asusec_id[] = {
	{"asusec", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, asusec_id);

struct file_operations asusec_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = asusec_ioctl,
	.open = asusec_open,
	.write = ec_write,
	.read = ec_read,
	.release = asusec_release,
};

static struct i2c_driver asusec_driver = {
	.class	= I2C_CLASS_HWMON,
	.driver	 = {
		.name = "asusec",
		.owner = THIS_MODULE,
	},
	.probe	 = asusec_probe,
	.remove	 = __devexit_p(asusec_remove),
	.suspend = asusec_suspend,
	.resume = asusec_resume,
	.id_table = asusec_id,
};

static DEVICE_ATTR(ec_status, S_IWUSR | S_IRUGO, asusec_show,NULL);
static DEVICE_ATTR(ec_info, S_IWUSR | S_IRUGO, asusec_info_show,NULL);
static DEVICE_ATTR(ec_dock, S_IWUSR | S_IRUGO, asusec_show_dock,NULL);
static DEVICE_ATTR(ec_lid_status, S_IWUSR | S_IRUGO, asusec_lid_status,NULL);
static DEVICE_ATTR(ec_dock_led, S_IWUSR | S_IRUGO, NULL,asusec_store_led);
static DEVICE_ATTR(ec_wakeup, S_IWUSR | S_IRUGO, NULL,asusec_store_ec_wakeup);
static DEVICE_ATTR(ec_dock_discharge, S_IWUSR | S_IRUGO, asusec_show_drain,NULL);
static DEVICE_ATTR(ec_dock_temperature, S_IWUSR | S_IRUGO, asusec_show_dock_temperature,NULL);
static DEVICE_ATTR(ec_dock_battery, S_IWUSR | S_IRUGO, asusec_show_dock_battery,NULL);
static DEVICE_ATTR(ec_dock_battery_all, S_IWUSR | S_IRUGO, asusec_show_dock_battery_all,NULL);
static DEVICE_ATTR(ec_dock_control_flag, S_IWUSR | S_IRUGO, asusec_show_dock_control_flag,NULL);

static struct attribute *asusec_smbus_attributes[] = {
	&dev_attr_ec_status.attr,
	&dev_attr_ec_info.attr,
	&dev_attr_ec_dock.attr,
	&dev_attr_ec_lid_status.attr,
	&dev_attr_ec_dock_led.attr,
	&dev_attr_ec_wakeup.attr,
	&dev_attr_ec_dock_discharge.attr,
	&dev_attr_ec_dock_temperature.attr,
	&dev_attr_ec_dock_battery.attr,
	&dev_attr_ec_dock_battery_all.attr,
	&dev_attr_ec_dock_control_flag.attr,
NULL
};

static const struct attribute_group asusec_smbus_group = {
	.attrs = asusec_smbus_attributes,
};

static int asusec_kp_sci_table[]={0, KEY_SLEEP, KEY_WLAN, KEY_BLUETOOTH, 
		ASUSEC_KEY_TOUCHPAD, KEY_BRIGHTNESSDOWN, KEY_BRIGHTNESSUP, ASUSEC_KEY_AUTOBRIGHT, 
		KEY_CAMERA, -9, -10, -11, 
		-12, -13, -14, -15, 
		KEY_WWW, ASUSEC_KEY_SETTING, KEY_PREVIOUSSONG, KEY_PLAYPAUSE, 
		KEY_NEXTSONG, KEY_MUTE, KEY_VOLUMEDOWN, KEY_VOLUMEUP};



#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>
#define I2C_TEST_FAIL_ASUSEC_READ_I2C (-1)

static int TestAsusECID(struct i2c_client *apClient)
{

	int lnResult = I2C_TEST_PASS;

	i2c_log_in_test_case("TestAsusECID++\n");

        if (asus_ec_version_get_cmd() < 0 )
           lnResult=I2C_TEST_FAIL_ASUSEC_READ_I2C;
		
	i2c_log_in_test_case("TestAsusECID--\n");

	return lnResult;
};

static struct i2c_test_case_info gAsusECTestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(TestAsusECID),
};


#endif
//ASUS_MERGE_END





/*
 * functions definition
 */
static void asusec_dockram_init(struct i2c_client *client){
	dockram_client.adapter = client->adapter;
	dockram_client.addr = 0x1b;
	dockram_client.detected = client->detected;
	dockram_client.dev = client->dev;
	dockram_client.driver = client->driver;
	dockram_client.flags = client->flags;
	strcpy(dockram_client.name,client->name);
}

int asusec_check_pad_HW_statsus(void)
{
	int backlight_status = 0;
	int hub_sleep_status = 0;

	backlight_status = query_pad_backlight_state();
	hub_sleep_status = get_MicroP_HUB_SLEEP_STATUS();

	if((backlight_status <= 0) || (hub_sleep_status <= 0)) {
		printk(KERN_INFO "[EC] BL(%d) | HUB(%d)\n", backlight_status, hub_sleep_status);
		return -1;
	}
	return 0;
}

static int asusec_dockram_write_data(int cmd, int length)
{
	int ret = 0;
	int retry_count = 3;
	bool check_pad = false;

	if ((ec_chip->dock_in == 0) || (ec_chip->isDockHWConnected <= 0)){
		printk(KERN_INFO "[EC] Dock not ready\n");
		return -1;
	}

	do{
		ret = i2c_smbus_write_i2c_block_data(&dockram_client, cmd, length, ec_chip->i2c_dm_data);

		if(ret < 0) {
			if(check_pad == false) {
				check_pad = true;
				if(asusec_check_pad_HW_statsus() == -1) {
					ec_chip->pad_err_count++;
					return -1;
				}
			}
			ec_chip->i2c_err_count++;
			msleep(10);
		}
	}while ((ret < 0) && (retry_count-- > 0));

	if (ret < 0) {
		printk(KERN_INFO "[EC] Fail to write dockram data, status %d cmd %d\n", ret, cmd);
	}
	return ret;
}

static int asusec_dockram_read_data(int cmd)
{
	int ret = 0;
	int retry_count = 3;
	bool check_pad = false;

	if ((ec_chip->dock_in == 0) || (ec_chip->isDockHWConnected <= 0)){
		printk(KERN_INFO "[EC] Dock not ready\n");
		return -1;
	}

	do{
		ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_data);

		if(ret < 0) {
			if(check_pad == false) {
				check_pad = true;
				if(asusec_check_pad_HW_statsus() == -1) {
					ec_chip->pad_err_count++;
					return -1;
				}
			}
			ec_chip->i2c_err_count++;
			msleep(10);
		}
	}while ((ret < 0) && (retry_count-- > 0));

	if (ret < 0) {
		printk(KERN_INFO "[EC] Fail to read dockram data, status %d cmd %d\n", ret, cmd);
	}
	return ret;
}

static int asusec_i2c_write_data(struct i2c_client *client, u16 data)
{
	int ret = 0;
	int retry_count = 3;
	bool check_pad = false;

	if ((ec_chip->dock_in == 0) || (ec_chip->isDockHWConnected <= 0)){
		printk(KERN_INFO "[EC] Dock not ready\n");
		return -1;
	}

	do {
		ret = i2c_smbus_write_word_data(client, 0x64, data);

		if(ret < 0) {
			if(check_pad == false) {
				check_pad = true;
				if(asusec_check_pad_HW_statsus() == -1) {
					ec_chip->pad_err_count++;
					return -1;
				}
			}
			ec_chip->i2c_err_count++;
			msleep(10);
		}
	}while ((ret < 0) && (retry_count-- > 0));

	if (ret < 0) {
		printk(KERN_INFO "[EC] Fail to write data, status %d\n", ret);
	}
	return ret;
}

static int asusec_i2c_read_data(struct i2c_client *client)
{
	int ret = 0;
	int retry_count = 3;
	bool check_pad = false;

	if ((ec_chip->dock_in == 0) || (ec_chip->isDockHWConnected <= 0)){
		printk(KERN_INFO "[EC] Dock not ready\n");
		return -1;
	}

	do{
		ret = i2c_smbus_read_i2c_block_data(client, 0x6A, 8, ec_chip->i2c_data);

		if(ret < 0) {
			if(check_pad == false) {
				check_pad = true;
				if(asusec_check_pad_HW_statsus() == -1) {
					ec_chip->pad_err_count++;
					return -1;
				}
			}
			ec_chip->i2c_err_count++;
			msleep(10);
		}
	}while ((ret < 0) && (retry_count-- > 0));

	if (ret < 0) {;
		printk(KERN_INFO "[EC] Fail to read data, status %d\n", ret);
	}
	return ret;
}

static int asusec_keypad_get_response(struct i2c_client *client, int res)
{
	int retry = ASUSEC_RETRY_COUNT;

	while(retry-- > 0){
		if(asusec_i2c_read_data(client) < 0)
			return -1;
		ASUSEC_I2C_DATA(ec_chip->i2c_data, ec_chip->index);
		pr_debug("[EC][%s] : %d %d \n",__FUNCTION__, ec_chip->i2c_data[1], ec_chip->i2c_data[2]);
		if ((ec_chip->i2c_data[1] & ASUSEC_OBF_MASK) && 
			(!(ec_chip->i2c_data[1] & ASUSEC_AUX_MASK))){ 
			if (ec_chip->i2c_data[2]  == res){
				goto get_asusec_keypad_i2c;
			}
		}		
		msleep(CONVERSION_TIME_MS/5);
	}
	printk(KERN_INFO "[EC] Keypad no response!! \n");
	return -1;

get_asusec_keypad_i2c:
	return 0;	

}

static int asusec_keypad_enable(struct i2c_client *client)
{
	int retry = ASUSEC_RETRY_COUNT;

	while(retry-- > 0){
		if(asusec_i2c_write_data(client, 0xF400) < 0)
			return -1;		
		if(!asusec_keypad_get_response(client, ASUSEC_PS2_ACK)){
			goto keypad_enable_ok;
		}
	}
	printk(KERN_INFO "[EC] fail to enable keypad!!\n");
	asusec_check_pad_HW_statsus();
	return -1;

keypad_enable_ok:
	printk(KERN_INFO "[EC] enable keypad ok!!\n");
	return 0;
}

static int asusec_keypad_disable(struct i2c_client *client)
{	
	int retry = ASUSEC_RETRY_COUNT;	
	
	while(retry-- > 0){
		if(asusec_i2c_write_data(client, 0xF500) < 0)
			return -1;
		if(!asusec_keypad_get_response(client, ASUSEC_PS2_ACK)){
			goto keypad_disable_ok;
		}
	}

	printk(KERN_INFO "[EC] fail to disable keypad!!\n");
	asusec_check_pad_HW_statsus();
	return -1;

keypad_disable_ok:
	printk(KERN_INFO "[EC] disable keypad ok!!\n");
	return 0;
}

static int asusec_keypad_reset(struct i2c_client *client)
{
	// ToDo : To check I2C cmd 
#if 0
	int retry = ASUSEC_RETRY_COUNT;

	while(retry-- > 0){
		asusec_i2c_write_data(client, 0xFF00);		
		if(!asusec_keypad_get_response(client, ASUSEC_PS2_ACK)){
			continue;
		}
		if((ec_chip->i2c_data[3] == 0xAA) &&
			(ec_chip->i2c_data[4] == 0x00)){
			goto keypad_reset_ok;
		}
	}
	ASUSEC_ERR("fail to reset keypad");
	printk(KERN_INFO "[EC] fail to reset keypad!!\n");
	return -1;

keypad_reset_ok:
#endif
	return 0;
}

// keypad led on/off --> LCD_BACKGHT_ENABLE 
static void asusec_keypad_led_on(struct work_struct *dat)
{	
	ec_chip->kbc_value = 1;
	if((ec_chip->isNeedResume == 0) && (ec_chip->init_success == 1)) {
		pr_debug("[EC] send led cmd 1\n");
		asusec_i2c_write_data(ec_chip->client, 0xED00);
	}
}


static void asusec_keypad_led_off(struct work_struct *dat)
{	
	ec_chip->kbc_value = 0;
	if((ec_chip->isNeedResume == 0) && (ec_chip->init_success == 1)) {
		pr_debug("[EC] send led cmd 1\n");
		asusec_i2c_write_data(ec_chip->client, 0xED00);
	}
}

static int asusec_touchpad_get_response(struct i2c_client *client, int res)
{
	int retry = ASUSEC_RETRY_COUNT;

	msleep(CONVERSION_TIME_MS);
	while(retry-- > 0){
		if(asusec_i2c_read_data(client) < 0)
			return -1;
		ASUSEC_I2C_DATA(ec_chip->i2c_data, ec_chip->index);
		if ((ec_chip->i2c_data[1] & ASUSEC_OBF_MASK) && 
			(ec_chip->i2c_data[1] & ASUSEC_AUX_MASK)){ 
			if (ec_chip->i2c_data[2] == res){
				goto get_asusec_touchpad_i2c;
			}
		}		
		msleep(CONVERSION_TIME_MS/5);
	}

	pr_debug("[EC] fail to get touchpad response\n");
	return -1;

get_asusec_touchpad_i2c:
	return 0;	

}

static int asusec_touchpad_enable(struct i2c_client *client)
{
	int retry = 5;

	while(retry-- > 0){
		ec_chip->tp_wait_ack = 1;		
		if(asusec_i2c_write_data(client, 0xF4D4) < 0)
			return -1;
		if(!asusec_touchpad_get_response(client, ASUSEC_PS2_ACK)){
			goto touchpad_enable_ok;
		}
	}

	printk(KERN_INFO "[EC] fail to enable touchpad\n");
	asusec_check_pad_HW_statsus();
	return -1;

touchpad_enable_ok:
	printk(KERN_INFO "[EC] success to enable touchpad\n");
	return 0;
}

static int asusec_touchpad_disable(struct i2c_client *client)
{	
	int retry = 5;	

	while(retry-- > 0){
		if(asusec_i2c_write_data(client, 0xF5D4) < 0)
			return -1;
		if(!asusec_touchpad_get_response(client, ASUSEC_PS2_ACK)){
			goto touchpad_disable_ok;
		}
	}

	printk(KERN_INFO "[EC] fail to disable touchpad\n");
	asusec_check_pad_HW_statsus();
	return -1;

touchpad_disable_ok:
	printk(KERN_INFO "[EC] success to disable touchpad\n");
	return 0;
}

static int asusec_touchpad_reset(struct i2c_client *client)
{

	int retry = ASUSEC_RETRY_COUNT;

	while(retry-- > 0){
		if(asusec_i2c_write_data(client, 0xFFD4) < 0)
			return -1;
		msleep(CONVERSION_TIME_MS*5);
		if(asusec_touchpad_get_response(client, ASUSEC_PS2_ACK)){
			continue;
		}	
		if((ec_chip->i2c_data[3] == 0xAA) &&
			(ec_chip->i2c_data[4] == 0x00)){
			goto touchpad_reset_ok;
		}
	}

	printk(KERN_INFO "[EC] fail to reset touchpad\n");
	asusec_check_pad_HW_statsus();
	return -1;

touchpad_reset_ok:
	printk(KERN_INFO "[EC] success to reset touchpad\n");
	return 0;
}


static void asusec_fw_clear_buf(void){
	int i;

	for (i = 0; i < 64; i++){
		i2c_smbus_read_byte_data(&dockram_client, 0);
	}
}

static void asusec_fw_reset_ec_op(void){
	char i2c_data[32];
	int i;

	asusec_fw_clear_buf();
	
	i2c_data[0] = 0x01;
	i2c_data[1] = 0x21;
	for (i = 0; i < i2c_data[0]+1 ; i++){
		i2c_smbus_write_byte_data(&dockram_client, i2c_data[i],0);
	}
	msleep(CONVERSION_TIME_MS*4);
}

static void asusec_fw_address_set_op(void){
	char i2c_data[32];
	int i;

	asusec_fw_clear_buf();

	i2c_data[0] = 0x05;
	i2c_data[1] = 0xa0;
	i2c_data[2] = 0x00;
	i2c_data[3] = 0x00;
	i2c_data[4] = 0x02;
	i2c_data[5] = 0x00;
	for (i = 0; i < i2c_data[0]+1 ; i++){
		i2c_smbus_write_byte_data(&dockram_client, i2c_data[i],0);
	}
	msleep(CONVERSION_TIME_MS*4);
}

static void asusec_fw_enter_op(void){
	char i2c_data[32];
	int i;

	asusec_fw_clear_buf();

	i2c_data[0] = 0x05;
	i2c_data[1] = 0x10;
	i2c_data[2] = 0x55;
	i2c_data[3] = 0xaa;
	i2c_data[4] = 0xcd;
	i2c_data[5] = 0xbe;
	for (i = 0; i < i2c_data[0]+1 ; i++){
		i2c_smbus_write_byte_data(&dockram_client, i2c_data[i],0);
	}
	msleep(CONVERSION_TIME_MS*4);
}

static int asusec_fw_cmp_id(void){
	char i2c_data[32];
	int i;
	int r_data[32];
	int ret_val = 0;

	asusec_fw_clear_buf();

	i2c_data[0] = 0x01;
	i2c_data[1] = 0xC0;
	for (i = 0; i < i2c_data[0]+1 ; i++){
		i2c_smbus_write_byte_data(&dockram_client, i2c_data[i],0);
	}
	msleep(CONVERSION_TIME_MS*10);

	for (i = 0; i < 5; i++){
		r_data[i] = i2c_smbus_read_byte_data(&dockram_client, 0);
	}

	for (i = 0; i < 5; i++){
		pr_debug("[EC] r_data[%d] = 0x%x\n", i, r_data[i]);
	}

	if (r_data[0] == 0xfa &&
		r_data[1] == 0xf0 &&
		r_data[2] == 0x12 &&
		r_data[3] == 0xef &&
		r_data[4] == 0x12){
		ret_val = 0;
	} else {
		ret_val = 1;
	}

	return ret_val;
}

static void asusec_fw_reset(void){

	if (asusec_fw_cmp_id() == 0){
		asusec_fw_enter_op();
		asusec_fw_address_set_op();
		asusec_fw_reset_ec_op();
		asusec_fw_clear_buf();
		if (ec_chip->re_init == 0){
			queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_dock_init_work, HZ/2);
			ec_chip->re_init = 1;
		}
	}
}
static int asusec_i2c_test(struct i2c_client *client){
	return asusec_i2c_write_data(client, 0x0000);
}

static void asusec_reset_dock(void){
	ec_chip->dock_init = 0;
	pr_debug("[EC] send EC_Request\n");	

	set_EC_REQUEST_VALUE(0);
	msleep(15);
	set_EC_REQUEST_VALUE(1);
	pr_debug("[EC] asusec_reset_dock\n");
}

static void asusec_sw_plug_in_out_dock(void){
	if((ec_chip->dock_in == 1) && (isHWDockConnected())){
		printk(KERN_INFO "[EC] software plug-out dock!\n");
		EC_Dock_out_callback();
		printk(KERN_INFO "[EC] software plug-in dock!\n");
		EC_Dock_in_callback(); 
	}
	else{
		printk(KERN_INFO "[EC] No dock detect\n");
		return;
	}
}

static void asusec_clear_i2c_buffer(struct i2c_client *client){
	int i = 0;

	for ( i=0; i<100; i++){
		if(asusec_i2c_read_data(client) < 0)
			return;
		if(get_EC_AP_WAKE_STATUS())
		{
			pr_debug("[EC] AP_WAKE_STATUS has be cleand ! (%d)\n", i+1);
			break;
		}
	}
}
static int asusec_chip_init(struct i2c_client *client)
{
	int ret_val = 0;
	int i;

	wake_lock(&ec_chip->wake_lock);

	strcpy(ec_chip->ec_model_name, " \n");
	strcpy(ec_chip->ec_version, " \n");

	ec_chip->op_mode = 0;

	pr_debug("[EC][chip_init] i2c test start\n");
	for ( i = 0; i < 10; i++){
		ret_val = asusec_i2c_test(client);
		if (ret_val < 0)
			msleep(50);
		else
			break;
	}
	pr_debug("[EC][chip_init]%d waste time = %d", i, i*50);

	if(ret_val < 0){
		goto fail_to_access_ec;
	}	

	ret_val = asus_ec_version_get_cmd();
	if(ret_val < 0){
		goto fail_to_access_ec;
	}
	pr_debug("[EC][chip_init] asus_ec_version_get_cmd done\n");

	if(asusec_input_device_create(client)) {
		goto fail_to_access_ec;
	}

	pr_debug("[EC][chip_init] asusec_input_device_create done\n");

	asusec_clear_i2c_buffer(client);

	if(asusec_keypad_disable(client) < 0){
		goto fail_to_access_ec;
	}
	pr_debug("[EC][chip_init] asusec_keypad_disable done\n");

	if(asusec_keypad_enable(client) < 0){
		goto fail_to_access_ec;
	}
	pr_debug("[EC][chip_init] asusec_keypad_enable done\n");

	AX_MicroP_enablePinInterrupt(INTR_EN_DOCK_INT, 1);
	pr_debug("[EC][chip_init] enable microp interrupt\n");

	asusec_clear_i2c_buffer(client);

	if(asusec_touchpad_disable(client) < 0){
		goto fail_to_access_ec;
	}
	pr_debug("[EC][chip_init] asusec_touchpad_disable done\n");

#if TOUCHPAD_ELAN
#if TOUCHPAD_MODE
	asusec_clear_i2c_buffer(client);
	if ((!elantech_detect(ec_chip)) && (!elantech_init(ec_chip))){
	    ec_chip->touchpad_member = ELANTOUCHPAD;
	} else {
		ec_chip->touchpad_member = -1;
	}
#endif
#endif
	pr_debug("[EC][chip_init] touchpad and keyboard init\n");
	ec_chip->d_index = 0;
	
	if (ec_chip->tp_enable){
		if(asusec_touchpad_enable(client) < 0){
			goto fail_to_access_ec;
		}
	}
	pr_debug("[EC][chip_init] asusec_touchpad_enable done\n");

	asusec_clear_i2c_buffer(client);

	ec_chip->init_success = 1;
	ec_chip->dock_init = 1;
	ec_chip->status = 1;
	wake_unlock(&ec_chip->wake_lock);
	return 0;

fail_to_access_ec:
	if (asusec_dockram_read_data(0x00) < 0){
		pr_debug("[EC] No EC detected\n");
		if(ec_chip->dock_in == 1) {
			ec_chip->dock_in = 0;
			switch_set_state(&ec_chip->dock_sdev, ec_chip->dock_in ? 10 : 0);
			printk(KERN_INFO "[EC] asusec_init_fail, remove dock\n");
		}
	} else {
		pr_debug("[EC] Need EC FW update\n");
		asusec_fw_reset();
	}
	wake_unlock(&ec_chip->wake_lock);
	return -1;

}
// ASUS_BSP Jiunhau_Wang EC porting +++
void P02VirtualPWRKey(void)
{
	printk(KERN_INFO "[EC] send power key notify to wake up P02\n");
	micropSendNotify(P01_PWR_KEY_PRESSED);
	msleep(10);
	micropSendNotify(P01_PWR_KEY_RELEASED);
}

int isKeyboardWakeup(void)
{
	return ec_chip->ec_wakeup;
}
EXPORT_SYMBOL(isKeyboardWakeup);

int EC_Dock_exist_callback(void)
{
	return ((ec_chip->dock_in) & isHWDockConnected());
}
EXPORT_SYMBOL(EC_Dock_exist_callback);

void EC_Dock_in_callback(void )
{
	printk(KERN_INFO "[EC] Dock in\n");
	ec_chip->isDockHWConnected = 1;
	if(ec_chip->dock_init == 0)
	{
		ec_chip->init_retry_count = 0;
		ec_chip->isNeedResume = 0;
		ec_chip->dock_in = 0;
		ec_chip->init_success = 0;
		ec_chip->bat_info = 0;
		ec_chip->i2c_err_count = 0;
		ec_chip->pad_err_count = 0;
		ec_chip->UsedPowerKey = false;
		cancel_delayed_work(&ec_chip->asusec_dock_init_work);
		queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_i2c_err_check_work, 0);
		queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_dock_init_work, 0);
	}
}

// release key and touchpad event before dock out
int asusec_release_input_event(void){

	flush_workqueue(asusec_wq);
	// release keyboard
	if((ec_chip->keypad_data.input_keycode > 0) && (ec_chip->keypad_data.value == 1)){
		printk(KERN_INFO "[EC] keycode = 0x%x, status = 0x%x\n", 
			   ec_chip->keypad_data.input_keycode, ec_chip->keypad_data.value);
		ec_chip->keypad_data.value = 0;
		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, ec_chip->keypad_data.value);
		input_sync(ec_chip->indev);
	}

	// release touchpad
	elantech_release_input_event(ec_chip);

	return 0;
}

void EC_Dock_out_callback(void )
{
	printk(KERN_INFO "[EC] Dock out\n");

	if(ec_chip->dock_init == 1) {
		asusec_release_input_event();
	}

	if((ec_chip->dock_in == 1) || (ec_chip->isDockHWConnected == 1)) {
		if (ec_chip->indev){
			input_unregister_device(ec_chip->indev);
			ec_chip->indev = NULL;
		}
		if (ec_chip->private->abs_dev){
			input_unregister_device(ec_chip->private->abs_dev);
			ec_chip->private->abs_dev = NULL;
		}
		ec_chip->dock_in = 0;
		switch_set_state(&ec_chip->dock_sdev, ec_chip->dock_in ? 10 : 0);
		printk(KERN_INFO "[EC] unregister input device and remove dock uevent\n");
	}

	if(ec_chip->isExtPower == true) {
		ec_chip->isExtPower = false;
	}

	if(ec_chip->isUseWakeLock == true) {
		wake_unlock(&ec_chip->wake_lock_keywakeup);
		ec_chip->isUseWakeLock = false;
	}

	ec_chip->dock_init = 0;
	ec_chip->op_mode = 0;
	ec_chip->status = 0;
	ec_chip->last_bat = -1;
	ec_chip->bat_info = 0;
	ec_chip->isNeedResume = 0;
	ec_chip->init_retry_count = 0;
	ec_chip->i2c_err_count = 0;
	ec_chip->pad_err_count = 0;
	ec_chip->isDockHWConnected = 0;
	ec_chip->isHubSleepOn = 0;
	ec_chip->UsedPowerKey = false;

	if (AX_MicroP_IsP01Connected() == 1)
		AX_MicroP_enablePinInterrupt(INTR_EN_DOCK_INT, 0);
}

void EC_Ap_wake_callback(void )
{
// ASUS_BSP Jiunhau +++ 20120105  
	// A66 + (P02 + DOCK) will generate "MicroP EC Evt Occurs"
	// asusec init will become two-thread initialization
	if((ec_chip->dock_init == 0) && (ec_chip->op_mode == 0)) {
		pr_debug("[EC][%s] EC not initial\n", __FUNCTION__);
		return;
	}
// ASUS_BSP Jiunhau --- 20120105

	pr_debug("[EC][%s] EC AP wake up ++++ \n", __FUNCTION__);
	if (ec_chip->op_mode){
		pr_debug("[EC][%s] op mode\n", __FUNCTION__);
		queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_fw_update_work, 0);
	}
	else{
		if (ec_chip->suspend_state){
			ec_chip->wakeup_lcd = 1;
			ec_chip->ap_wake_wakeup = 1;
		}
		queue_work_on(0, asusec_wq, &ec_chip->asusec_work);
	}
	pr_debug("[EC][%s] EC AP wake up ---- \n", __FUNCTION__);
}

int pin_debounce(int (*input_get_fun)(void))
{
	int i = 0,  gpio_state = 0, d_counter = 0;

	gpio_state = input_get_fun();
	for(i = 0; i < 40; i++){
		msleep(50);
		if (gpio_state == input_get_fun()){
			d_counter++;
		} else {
			gpio_state = input_get_fun();
			d_counter = 0;
		}
		if (d_counter > DEBOUNCE_CONTINUOUS){
			pr_debug("[EC][PinDebounce] pass (%d) ", i+1);
			break;
		}
	}

	if(d_counter < DEBOUNCE_CONTINUOUS) {
		pr_debug("[EC][PinDebounce] fail \n");
		return DEBOUNCE_ERR;
	}
	return gpio_state;
}

void EC_Lid_change_callback(void)
{
	// get_EC_HALL_SENSOR_STATUS 
	// ret = 1 -> opened  | ret = 0 -> closed

	int value = 0;

	value = pin_debounce(get_EC_HALL_SENSOR_STATUS);
	if (value == DEBOUNCE_ERR) {
		return;
	}
	else{
		ec_chip->lid_status = value;
		queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_lid_work, 0);
	}
}

// ASUS_BSP Jiunhau_Wang EC porting ---

static int asusec_kp_key_mapping(int x)
{
	switch (x){
		case ASUSEC_KEYPAD_ESC:
			return KEY_BACK; 

		case ASUSEC_KEYPAD_KEY_WAVE:
			return KEY_GRAVE;

		case ASUSEC_KEYPAD_KEY_1:
			return KEY_1;

		case ASUSEC_KEYPAD_KEY_2:
			return KEY_2;

		case ASUSEC_KEYPAD_KEY_3:
			return KEY_3;

		case ASUSEC_KEYPAD_KEY_4:
			return KEY_4;

		case ASUSEC_KEYPAD_KEY_5:
			return KEY_5;

		case ASUSEC_KEYPAD_KEY_6:
			return KEY_6;

		case ASUSEC_KEYPAD_KEY_7:
			return KEY_7;

		case ASUSEC_KEYPAD_KEY_8:
			return KEY_8;

		case ASUSEC_KEYPAD_KEY_9:
			return KEY_9;

		case ASUSEC_KEYPAD_KEY_0:
			return KEY_0;

		case ASUSEC_KEYPAD_KEY_MINUS:
			return KEY_MINUS;

		case ASUSEC_KEYPAD_KEY_EQUAL:
			return KEY_EQUAL;

		case ASUSEC_KEYPAD_KEY_BACKSPACE:
			return KEY_BACKSPACE;

		case ASUSEC_KEYPAD_KEY_TAB:
			return KEY_TAB;

		case ASUSEC_KEYPAD_KEY_Q:
			return KEY_Q;

		case ASUSEC_KEYPAD_KEY_W:
			return KEY_W;

		case ASUSEC_KEYPAD_KEY_E:
			return KEY_E;

		case ASUSEC_KEYPAD_KEY_R:
			return KEY_R;

		case ASUSEC_KEYPAD_KEY_T:
			return KEY_T;

		case ASUSEC_KEYPAD_KEY_Y:
			return KEY_Y;

		case ASUSEC_KEYPAD_KEY_U:
			return KEY_U;

		case ASUSEC_KEYPAD_KEY_I:
			return KEY_I;

		case ASUSEC_KEYPAD_KEY_O:
			return KEY_O;

		case ASUSEC_KEYPAD_KEY_P:
			return KEY_P;

		case ASUSEC_KEYPAD_KEY_LEFTBRACE:
			return KEY_LEFTBRACE;

		case ASUSEC_KEYPAD_KEY_RIGHTBRACE:
			return KEY_RIGHTBRACE;

		case ASUSEC_KEYPAD_KEY_BACKSLASH:
			return KEY_BACKSLASH;

		case ASUSEC_KEYPAD_KEY_CAPSLOCK:
			return KEY_CAPSLOCK;

		case ASUSEC_KEYPAD_KEY_A:
			return KEY_A;

		case ASUSEC_KEYPAD_KEY_S:
			return KEY_S;

		case ASUSEC_KEYPAD_KEY_D:
			return KEY_D;

		case ASUSEC_KEYPAD_KEY_F:
			return KEY_F;

		case ASUSEC_KEYPAD_KEY_G:
			return KEY_G;

		case ASUSEC_KEYPAD_KEY_H:
			return KEY_H;

		case ASUSEC_KEYPAD_KEY_J:
			return KEY_J;

		case ASUSEC_KEYPAD_KEY_K:
			return KEY_K;

		case ASUSEC_KEYPAD_KEY_L:
			return KEY_L;

		case ASUSEC_KEYPAD_KEY_SEMICOLON:
			return KEY_SEMICOLON;

		case ASUSEC_KEYPAD_KEY_APOSTROPHE:
			return KEY_APOSTROPHE;

		case ASUSEC_KEYPAD_KEY_ENTER:
			return KEY_ENTER;

		case ASUSEC_KEYPAD_KEY_LEFTSHIFT:
			return KEY_LEFTSHIFT;

		case ASUSEC_KEYPAD_KEY_Z:
			return KEY_Z;

		case ASUSEC_KEYPAD_KEY_X:
			return KEY_X;

		case ASUSEC_KEYPAD_KEY_C:
			return KEY_C;

		case ASUSEC_KEYPAD_KEY_V:
			return KEY_V;

		case ASUSEC_KEYPAD_KEY_B:
			return KEY_B;

		case ASUSEC_KEYPAD_KEY_N:
			return KEY_N;

		case ASUSEC_KEYPAD_KEY_M:
			return KEY_M;

		case ASUSEC_KEYPAD_KEY_COMMA:
			return KEY_COMMA;

		case ASUSEC_KEYPAD_KEY_DOT:
			return KEY_DOT;

		case ASUSEC_KEYPAD_KEY_SLASH:
			return KEY_SLASH;

		case ASUSEC_KEYPAD_KEY_RIGHTSHIFT:
			return KEY_RIGHTSHIFT;

		case ASUSEC_KEYPAD_KEY_LEFT:
			return KEY_LEFT;

		case ASUSEC_KEYPAD_KEY_RIGHT:
			return KEY_RIGHT;

		case ASUSEC_KEYPAD_KEY_UP:
			return KEY_UP;

		case ASUSEC_KEYPAD_KEY_DOWN:
			return KEY_DOWN;

		case ASUSEC_KEYPAD_RIGHTWIN:
			return KEY_SEARCH;

		case ASUSEC_KEYPAD_LEFTCTRL:
			return KEY_LEFTCTRL;

		case ASUSEC_KEYPAD_LEFTWIN:
			return KEY_HOMEPAGE;

		case ASUSEC_KEYPAD_LEFTALT:
			return KEY_LEFTALT;

		case ASUSEC_KEYPAD_KEY_SPACE:
			return KEY_SPACE;

		case ASUSEC_KEYPAD_RIGHTALT:
			return KEY_RIGHTALT;

		case ASUSEC_KEYPAD_WINAPP:
			return KEY_MENU;

		case ASUSEC_KEYPAD_RIGHTCTRL:
			return KEY_RIGHTCTRL;

		case ASUSEC_KEYPAD_HOME:	
			return KEY_HOME;

		case ASUSEC_KEYPAD_PAGEUP:
			return KEY_PAGEUP;

		case ASUSEC_KEYPAD_PAGEDOWN:
			return KEY_PAGEDOWN;

		case ASUSEC_KEYPAD_END:
			return KEY_END;

		//--- JP keys
		case ASUSEC_YEN:
			return KEY_YEN;
			
		case ASUSEC_RO:
			return KEY_RO;
			
		case ASUSEC_MUHENKAN:
			return KEY_MUHENKAN;
			
		case ASUSEC_HENKAN:
			return KEY_HENKAN;
			
		case ASUSEC_HIRAGANA_KATAKANA:
			return KEY_KATAKANAHIRAGANA;			
			
		//--- UK keys
		case ASUSEC_EUROPE_2:
			return KEY_102ND;
			
		default:
			return -1;
	}
}

static void asusec_reset_counter(unsigned long data){
	ec_chip->d_index = 0;
}

static int asusec_tp_control(int arg){

	int ret_val = 0;	
	
	if(arg == ASUSEC_TP_ON){
		if (ec_chip->tp_enable == 0){
			ec_chip->tp_wait_ack = 1;
			ec_chip->tp_enable = 1;
			asusec_i2c_write_data(ec_chip->client, 0xF4D4);
			ec_chip->d_index = 0;
		}
		ret_val = 0;
	} else if (arg == ASUSEC_TP_OFF){
		ec_chip->tp_wait_ack = 1;
		ec_chip->tp_enable = 0;
		asusec_i2c_write_data(ec_chip->client, 0xF5D4);
		ec_chip->d_index = 0;
		ret_val = 0;
	} else
		ret_val = -ENOTTY;
	
	return ret_val;

}
#if (!TOUCHPAD_MODE)
static void asusec_tp_rel(void){

	ec_chip->touchpad_data.x_sign = (ec_chip->ec_data[0] & X_SIGN_MASK) ? 1:0;
	ec_chip->touchpad_data.y_sign = (ec_chip->ec_data[0] & Y_SIGN_MASK) ? 1:0;
	ec_chip->touchpad_data.left_btn = (ec_chip->ec_data[0] & LEFT_BTN_MASK) ? 1:0;
	ec_chip->touchpad_data.right_btn = (ec_chip->ec_data[0] & RIGHT_BTN_MASK) ? 1:0;
	ec_chip->touchpad_data.delta_x = 
		(ec_chip->touchpad_data.x_sign) ? (ec_chip->ec_data[1] - 0xff):ec_chip->ec_data[1];
	ec_chip->touchpad_data.delta_y = 
		(ec_chip->touchpad_data.y_sign) ? (ec_chip->ec_data[2] - 0xff):ec_chip->ec_data[2];

	input_report_rel(ec_chip->indev, REL_X, ec_chip->touchpad_data.delta_x);
	input_report_rel(ec_chip->indev, REL_Y, (-1) * ec_chip->touchpad_data.delta_y);
	input_report_key(ec_chip->indev, BTN_LEFT, ec_chip->touchpad_data.left_btn);
	input_report_key(ec_chip->indev, KEY_BACK, ec_chip->touchpad_data.right_btn);				
	input_sync(ec_chip->indev);
	
}
#endif

#if TOUCHPAD_MODE
static void asusec_tp_abs(void){
	unsigned char SA1,A1,B1,SB1,C1,D1;
	static unsigned char SA1_O=0,A1_O=0,B1_O=0,SB1_O=0,C1_O=0,D1_O=0;
	static int Null_data_times = 0;
	
	if ((ec_chip->tp_enable) && (ec_chip->touchpad_member == ELANTOUCHPAD)){
		SA1= ec_chip->ec_data[0];
		A1 = ec_chip->ec_data[1];
		B1 = ec_chip->ec_data[2];
		SB1= ec_chip->ec_data[3];
		C1 = ec_chip->ec_data[4];
		D1 = ec_chip->ec_data[5];
		pr_debug("[EC] SA1=0x%x A1=0x%x B1=0x%x SB1=0x%x C1=0x%x D1=0x%x \n",SA1,A1,B1,SB1,C1,D1);
		if ( (SA1 == 0xC4) && (A1 == 0xFF) && (B1 == 0xFF) && 
		     (SB1 == 0x02) && (C1 == 0xFF) && (D1 == 0xFF)){
			Null_data_times ++;
			goto asusec_tp_abs_end;
		}

		if(!(SA1 == SA1_O && A1 == A1_O && B1 == B1_O && 
		   SB1 == SB1_O && C1 == C1_O && D1 == D1_O)) {
			elantech_report_absolute_to_related(ec_chip, &Null_data_times);
		}
		
asusec_tp_abs_end:
		SA1_O = SA1;
		A1_O = A1;
		B1_O = B1;
		SB1_O = SB1;
		C1_O = C1;
		D1_O = D1;
	}

}
#endif

static void asusec_touchpad_processing(void){
	int i;
	int length = 0;
	int tp_start = 0;
	ASUSEC_I2C_DATA(ec_chip->i2c_data,ec_chip->index);

#if TOUCHPAD_MODE
	length = ec_chip->i2c_data[0];
	if (ec_chip->tp_wait_ack){
		ec_chip->tp_wait_ack = 0;
		// TF101 : tp_start = 1 data = 0xFA; PadFone : tp_start = 0
		if(ec_chip->i2c_data[2] == 0xFA) {
			tp_start = 1;
		}
		else{
			tp_start = 0;
		}
		ec_chip->d_index = 0;
	} else {
		tp_start = 0;
	}
		
	for( i = tp_start; i < length - 1 ; i++){
		ec_chip->ec_data[ec_chip->d_index] = ec_chip->i2c_data[i+2];
		ec_chip->d_index++;
		if (ec_chip->d_index == 6){	
			asusec_tp_abs();
			ec_chip->d_index = 0;
		}
	}
	
	
	if (ec_chip->d_index)
		mod_timer(&ec_chip->asusec_timer,jiffies+(HZ * 1/20));
#else
	length = ec_chip->i2c_data[0];
	for( i = 0; i < length -1 ; i++){
		ec_chip->ec_data[ec_chip->d_index] = ec_chip->i2c_data[i+2];
		ec_chip->d_index++;
		if (ec_chip->d_index == 3){
			asusec_tp_rel();
			ec_chip->d_index = 0;
		}
	}	
#endif
}

#if 0
static void asusec_kp_wake(void){
	pr_debug("[EC] ASUSEC WAKE\n");
	if (asusec_input_device_create(ec_chip->client)){
		return ;
	}
	input_report_key(ec_chip->indev, KEY_MENU, 1);
	input_sync(ec_chip->indev);
	input_report_key(ec_chip->indev, KEY_MENU, 0);
	input_sync(ec_chip->indev);
}

static void asusec_kp_smi(void){
	if (ec_chip->i2c_data[2] == ASUSEC_SMI_HANDSHAKING){
		pr_debug("[EC] ASUSEC_SMI_HANDSHAKING\n");
		asusec_chip_init(ec_chip->client);
	} else if (ec_chip->i2c_data[2] == ASUSEC_SMI_RESET){
		pr_debug("[EC] ASUSEC_SMI_RESET\n");
		ec_chip->init_success = 0;
		asusec_dock_init_work_function(NULL);
	} else if (ec_chip->i2c_data[2] == ASUSEC_SMI_WAKE){
		pr_debug("[EC] ASUSEC_SMI_WAKE\n");
		asusec_kp_wake();
	}
}
#endif

static void asusec_kp_kbc(void){
	if (ec_chip->i2c_data[2] == ASUSEC_PS2_ACK){
		if (ec_chip->kbc_value == 0){
			pr_debug("[EC] send led cmd 2\n");
			asusec_i2c_write_data(ec_chip->client, 0x0000);
		} else {
			pr_debug("[EC] send led cmd 2\n");
			asusec_i2c_write_data(ec_chip->client, 0x0400);
		}
	}
}
static void asusec_kp_sci(void){
	int ec_signal = ec_chip->i2c_data[2];
	
	ec_chip->keypad_data.input_keycode = asusec_kp_sci_table[ec_signal];
	if(ec_chip->keypad_data.input_keycode > 0){
		pr_debug("[EC] input_keycode = 0x%x\n", ec_chip->keypad_data.input_keycode);
		
		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 1);
		input_sync(ec_chip->indev); 
		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 0);
		input_sync(ec_chip->indev); 
		
	}else{				
		pr_debug("[EC] Unknown ec_signal = 0x%x\n", ec_signal);
	}
}
static void asusec_kp_key(void){
	int scancode = 0;
	
	pr_debug("[EC][%s] input event process ! ++++ \n", __FUNCTION__);
	if (ec_chip->i2c_data[2] == ASUSEC_KEYPAD_KEY_EXTEND){		// data is an extended data
		ec_chip->keypad_data.extend = 1;
		ec_chip->bc = 3;
	}else{
		ec_chip->keypad_data.extend = 0;
		ec_chip->bc = 2;
	}
	if(ec_chip->i2c_data[ec_chip->bc] == ASUSEC_KEYPAD_KEY_BREAK){ // the data is a break signal
		ec_chip->keypad_data.value = 0;	
		ec_chip->bc++;
	}else{
		ec_chip->keypad_data.value = 1;
	}
	
	if (ec_chip->keypad_data.extend == 1){
		scancode = ((ASUSEC_KEYPAD_KEY_EXTEND << 8) | ec_chip->i2c_data[ec_chip->bc]);
	} else {
		scancode = ec_chip->i2c_data[ec_chip->bc];
	}
	if (ec_chip->i2c_data[0] == 6){								// left shift DOWN + note2 keys
		if ((ec_chip->i2c_data[2] == 0xE0) &&
			(ec_chip->i2c_data[3] == 0xF0) &&
			(ec_chip->i2c_data[4] == 0x12)){
			scancode = ec_chip->i2c_data[5] << 8 | ec_chip->i2c_data[6];
			ec_chip->keypad_data.value = 1;
		}
		else if ((ec_chip->i2c_data[2] == 0xE0) &&				// right shift DOWN + note2 keys
			(ec_chip->i2c_data[3] == 0xF0) &&
			(ec_chip->i2c_data[4] == 0x59)){
			scancode = ec_chip->i2c_data[5] << 8 | ec_chip->i2c_data[6];
			ec_chip->keypad_data.value = 1;
		}
	}
	pr_debug("[EC] scancode = 0x%x\n", scancode);
	ec_chip->keypad_data.input_keycode = asusec_kp_key_mapping(scancode);
	if(ec_chip->keypad_data.input_keycode > 0){
		printk(KERN_INFO "input_keycode = 0x%x, input_value = %d\n", 
				ec_chip->keypad_data.input_keycode, ec_chip->keypad_data.value);
		
		input_report_key(ec_chip->indev, 
			ec_chip->keypad_data.input_keycode, ec_chip->keypad_data.value);
		input_sync(ec_chip->indev); 
		pr_debug("[EC][%s] input event process ! ---- \n", __FUNCTION__);
	}else{				
		pr_debug("[EC] Unknown scancode = 0x%x\n", scancode);
	}
}

static void asusec_keypad_processing(void){
		
	ASUSEC_I2C_DATA(ec_chip->i2c_data,ec_chip->index);	
	if (ec_chip->i2c_data[1] & ASUSEC_KBC_MASK)
	{
		pr_debug("[EC][%s] asusec_kp_kbc\n", __FUNCTION__);
		asusec_kp_kbc();
	}
	else if (ec_chip->i2c_data[1] & ASUSEC_SCI_MASK)	// ec data is a signal
	{
		pr_debug("[EC][%s] asusec_kp_sci\n", __FUNCTION__);
		asusec_kp_sci();
	}
	else
	{												// ec data is a scan code
		pr_debug("[EC][%s] asusec_kp_key\n", __FUNCTION__);
		asusec_kp_key();
	}
}

static void asusec_lid_report_function(struct work_struct *dat)
{
	if(ec_chip->lid_indev == NULL) {
		pr_debug("[EC][LID] input device doesn`t exist\n");
		return;
	}

	input_report_switch(ec_chip->lid_indev, SW_LID, ec_chip->lid_status); 
	switch_set_state(&ec_chip->lid_sdev, ec_chip->lid_status ? ASUSEC_LID_OPEN_UEVENT : ASUSEC_LID_CLOSE_UEVENT);
	printk(KERN_INFO "[EC][LID] SW_LID report value = %d (1 : open ; 0 : close)\n", ec_chip->lid_status);
}

static void asusec_stresstest_work_function(struct work_struct *dat)
{
	if(asusec_i2c_read_data(ec_chip->client) < 0)
		return;
	if (ec_chip->i2c_data[1] & ASUSEC_OBF_MASK){		// ec data is valid
		if (ec_chip->i2c_data[1] & ASUSEC_AUX_MASK){	// ec data is from touchpad
			pr_debug("[EC][%s] asusec_touchpad\n", __FUNCTION__);
			asusec_touchpad_processing();
		}else{		// ec data is from keyboard
			pr_debug("[EC][%s] asusec_keypad\n", __FUNCTION__);
			asusec_keypad_processing();
		}
	}
	queue_delayed_work_on(0, asusec_wq, &asusec_stress_work, HZ/ec_chip->polling_rate);
}

int isHWDockConnected(void)
{
	return get_EC_DOCK_IN_STATUS() ? (0) : (1);
}


static void asusec_dock_init_work_function(struct work_struct *dat)
{
	static int bl_retry_count = 0;

	// check Hub-sleep pin level
	pr_debug("[EC][init] check hub_sleep status start\n"); 

	ec_chip->isHubSleepOn = get_MicroP_HUB_SLEEP_STATUS();
	 

	if( ec_chip->isHubSleepOn == 0) {
		if(ec_chip->init_retry_count < ASUSEC_HUB_SLEEP_RETRY) {
			cancel_delayed_work(&ec_chip->asusec_dock_init_work);
			queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_dock_init_work, 0.3 * HZ);
			ec_chip->init_retry_count++;
			printk(KERN_INFO "[EC][init] HUB_SLEEP not ready (retry = %d)\n", ec_chip->init_retry_count);
		}
		else
			printk(KERN_INFO "[EC][init] HUB_SLEEP not ready!\n");
		return;
	}
	else if(ec_chip->isHubSleepOn < 0) {
		if(ec_chip->init_retry_count < ASUSEC_MICROP_RETRY) {
			cancel_delayed_work(&ec_chip->asusec_dock_init_work);
			queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_dock_init_work, 2 * HZ);
			ec_chip->init_retry_count++;
			printk(KERN_INFO "[EC][init] MicroP status error (retry = %d)\n", ec_chip->init_retry_count);
		}
		else
			printk(KERN_INFO "[EC][init] MicroP status error!\n");
		return;
	}
	ec_chip->init_retry_count = 0;
	pr_debug("[EC][init] check hub_sleep status end\n"); 

#ifdef CONFIG_P02_BACKLIGHT
	pr_debug("[EC][init] check back light status start\n");
	do {
		if(query_pad_backlight_state() == 1) {
			break;
		}
		else{
			msleep(50);
			bl_retry_count++;
		}
	} while (bl_retry_count <= ASUSEC_BL_RETRY);
	printk(KERN_INFO "[EC][init] check back light status end (retry = %d)\n", bl_retry_count);
	bl_retry_count = 0;
#endif

	pr_debug("[EC][%s] A60K Dock-init function\n", __FUNCTION__);

	wake_lock(&ec_chip->wake_lock_init);
		
	ec_chip->re_init = 0;

	mutex_lock(&ec_chip->input_lock);
	if(isHWDockConnected()){
		if(ec_chip->isNeedResume == 1) {
			ec_chip->isNeedResume = 2;
		}
		pr_debug("[EC][%s] Dock detected\n", __FUNCTION__);
		ec_chip->dock_in = 1;
		pr_debug("[EC][init] reset dock start \n"); 
		asusec_reset_dock();
		printk(KERN_INFO "[EC][init] EC_REQ \n"); 

		if(ec_chip->ec_wakeup == 0) {
			msleep(500);
		}else{
			msleep(50);
		}

		if(asusec_chip_init(ec_chip->client) < 0){
			goto ec_init_fail;
		}
		pr_debug("[EC][init] complete \n"); 

		// protect re-resume
		if(ec_chip->isNeedResume == 2) {
			ec_chip->isNeedResume = 0;
		}

		EC_Init_Complete();
		EC_Lid_change_callback();
	}else{
		pr_debug("[EC][%s] No dock detected\n", __FUNCTION__);
		ec_chip->dock_in = 0;
		ec_chip->init_success = 0;
		ec_chip->tp_enable = 1;
		if (ec_chip->indev){
			input_unregister_device(ec_chip->indev);
			ec_chip->indev = NULL;
		}
		if (ec_chip->private->abs_dev){
			input_unregister_device(ec_chip->private->abs_dev);
			ec_chip->private->abs_dev = NULL;
		}
	}
	ec_chip->UsedPowerKey = false;
	switch_set_state(&ec_chip->dock_sdev, ec_chip->dock_in ? 10 : 0);
	mutex_unlock(&ec_chip->input_lock);
	wake_unlock(&ec_chip->wake_lock_init);

	return;

ec_init_fail:
	mutex_unlock(&ec_chip->input_lock);
	wake_unlock(&ec_chip->wake_lock_init);

	if(ec_chip->isHubSleepOn == 0) {

		cancel_delayed_work(&ec_chip->asusec_dock_init_work);
		return;
	}
#if 0
	// if resume fail, remove this work
	if(ec_chip->isNeedResume != 0)
	{
		ec_chip->isNeedResume = 1;
		cancel_delayed_work(&ec_chip->asusec_dock_init_work);
		return;
	}
#endif

	printk(KERN_INFO "[EC] ec chip initial retry !\n");
	if (isHWDockConnected()){
		cancel_delayed_work(&ec_chip->asusec_dock_init_work);
		queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_dock_init_work, 0.5 * HZ);
	}

	return;
}

static void asusec_fw_update_work_function(struct work_struct *dat)
{
	int smbus_data;

	mutex_lock(&ec_chip->lock);

	do{
		smbus_data = i2c_smbus_read_byte_data(&dockram_client, 0);
		if(smbus_data < 0) {
			pr_debug("[EC][%s] I2C read fail!! \n", __FUNCTION__);
		}
		else{
			if(smbus_data != 0) {
				//printk(KERN_INFO"[EC][%s] I2C read byte = 0x%x\n", __FUNCTION__, smbus_data);
				pet_watchdog(); // avoid fw_update fail
				BuffPush(smbus_data);
			}
		}
	}while (!get_EC_AP_WAKE_STATUS());

	mutex_unlock(&ec_chip->lock);
}

static void asusec_i2c_err_check_work_function(struct work_struct *dat)
{
	if(ec_chip->i2c_err_count != 0)
		printk(KERN_INFO "[EC][I2C] i2c_err_count = %d\n", ec_chip->i2c_err_count);

#if 0
	// Pad hw error : 1. dock dis-connected 2. Backlight disable 3. USB_HUP disable
	if(ec_chip->pad_err_count != 0){
		printk(KERN_INFO "[EC] pad status error, reset dock out status! \n");
		EC_Dock_out_callback();
	}
#endif

	// dock I2C error
	if(ec_chip->i2c_err_count > I2C_ERR_COUNT) {
		ec_chip->i2c_err_count = 0;
		ec_chip->dock_in = -1;
		printk(KERN_INFO "[EC] dock I2C error, dock remove! \n");
		// send DOCK_EXT_POWER_PLUG_OUT and DOCK_PLUG_OUT 
#if 0
		reportPadStationDockI2CFail();
		// ToDo : report dock err uevent
#endif
	}
	else{
		queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_i2c_err_check_work, 60 * HZ);
	}
}

static void asusec_work_function(struct work_struct *dat)
{
	int ret_val = 0;
	pr_debug("[EC][%s] asusec_work_function! ++++ \n", __FUNCTION__);

	if((ec_chip->isNeedResume == 1) && (ec_chip->ec_wakeup == 1)) {
		if(ec_chip->UsedPowerKey == false){
			P02VirtualPWRKey();
			ec_chip->UsedPowerKey = true;
		}
		return;
	}
#if 0
	if(ec_chip->isNeedResume) {
		if (get_EC_HALL_SENSOR_STATUS()){
			ec_chip->dock_in = isHWDockConnected();
			wake_lock_timeout(&ec_chip->wake_lock, 3*HZ);
			msleep(500);
		}
	}
#endif

#if 0
	if (ec_chip->wakeup_lcd){
		if (get_EC_HALL_SENSOR_STATUS()){
			ec_chip->wakeup_lcd = 0;
			ec_chip->dock_in = isHWDockConnected();
			wake_lock_timeout(&ec_chip->wake_lock, 3*HZ);
			msleep(500);
		}
	}
#endif
	do{
		ret_val = asusec_i2c_read_data(ec_chip->client);

		if (ret_val < 0){
			pr_debug("[EC][%s] ret_val=%d \n",__FUNCTION__,ret_val);
			return ;
		}
		else
		{
			pr_debug("[EC][%s] i2c_data: %x,%x,%x,%x,%x,%x,%x,%x\n",__FUNCTION__,
														ec_chip->i2c_data[0],
														ec_chip->i2c_data[1],
														ec_chip->i2c_data[2],
														ec_chip->i2c_data[3],
														ec_chip->i2c_data[4],
														ec_chip->i2c_data[5],
														ec_chip->i2c_data[6],
														ec_chip->i2c_data[7]);
		}
#if 0
		if (ec_chip->i2c_data[1] & ASUSEC_OBF_MASK){		// ec data is valid
			if (ec_chip->i2c_data[1] & ASUSEC_SMI_MASK){	// ec data is from touchpad
				asusec_kp_smi();
				return ;
			}
		}
#endif

		if(ec_chip->lid_status == 0) {
			return;
		}

		mutex_lock(&ec_chip->input_lock);
		if (ec_chip->indev == NULL || ec_chip->init_success == 0){
			mutex_unlock(&ec_chip->input_lock);
			return;
		}
		if (ec_chip->i2c_data[1] & ASUSEC_OBF_MASK){		// ec data is valid
			if (ec_chip->i2c_data[1] & ASUSEC_AUX_MASK){	// ec data is from touchpad
				if (ec_chip->private->abs_dev)
					asusec_touchpad_processing();
			}else{		// ec data is from keyboard
				asusec_keypad_processing();
			}
		}
		mutex_unlock(&ec_chip->input_lock);
		pr_debug("[EC][%s] asusec_work_function! ---- \n", __FUNCTION__);
	}while (!get_EC_AP_WAKE_STATUS());
}

static void asusec_keypad_set_input_params(struct input_dev *dev)
{
	int i = 0;
	set_bit(EV_KEY, dev->evbit);	
	for ( i = 0; i < 246; i++)
		set_bit(i,dev->keybit);

	input_set_capability(dev, EV_LED, LED_CAPSL);
}

static void asusec_lid_set_input_params(struct input_dev *dev)
{
	set_bit(EV_SW, dev->evbit);
	set_bit(SW_LID, dev->swbit);
}

/*
static void asusec_touchpad_set_input_params(struct input_dev *dev)
{
	set_bit(EV_KEY, dev->evbit);
	set_bit(BTN_LEFT, dev->keybit);
	set_bit(BTN_RIGHT, dev->keybit);

	set_bit(EV_REL, dev->evbit);
	set_bit(REL_X, dev->relbit);
	set_bit(REL_Y, dev->relbit);

	set_bit(EV_SYN, dev->evbit);
	set_bit(EV_ABS, dev->evbit);
	set_bit(BTN_TOUCH, dev->keybit);
	set_bit(BTN_2, dev->keybit);
	set_bit(ABS_TOOL_WIDTH, dev->absbit);

	input_set_abs_params(dev, ABS_X, XMIN_NOMINAL, XMAX_NOMINAL, 0, 0);
	input_set_abs_params(dev, ABS_Y, YMIN_NOMINAL, YMAX_NOMINAL, 0, 0);
	input_set_abs_params(dev, ABS_HAT0X, XMIN_NOMINAL, XMAX_NOMINAL, 0, 0);
	input_set_abs_params(dev, ABS_HAT0Y, YMIN_NOMINAL, YMAX_NOMINAL, 0, 0);
	input_set_abs_params(dev, ABS_HAT1X, XMIN_NOMINAL, XMAX_NOMINAL, 0, 0);
	input_set_abs_params(dev, ABS_HAT1Y, YMIN_NOMINAL, YMAX_NOMINAL, 0, 0);

	input_set_abs_params(dev, ABS_MT_POSITION_X, XMIN_NOMINAL, XMAX_NOMINAL, 0, 0);	
	input_set_abs_params(dev, ABS_MT_POSITION_Y, YMIN_NOMINAL, YMAX_NOMINAL, 0, 0);
	input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	input_set_abs_params(dev, ABS_PRESSURE, 0, 255, 0, 0);
}
*/
static int asusec_input_device_create(struct i2c_client *client){
	int err = 0;
	
	if (ec_chip->indev){
		return 0;
	}
	ec_chip->indev = input_allocate_device();
	if (!ec_chip->indev) {
		pr_debug("[EC] input_dev allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	ec_chip->indev->name = "asusec";
	ec_chip->indev->phys = "/dev/input/asusec";
	ec_chip->indev->dev.parent = &client->dev;
	ec_chip->indev->event = asusec_event;
	
	asusec_keypad_set_input_params(ec_chip->indev);
	//asusec_touchpad_set_input_params(ec_chip->indev);
	err = input_register_device(ec_chip->indev);
	if (err) {
		pr_debug("[EC] input registration fails\n");
		goto exit_input_free;
	}
	return 0;

exit_input_free:
	input_free_device(ec_chip->indev);
	ec_chip->indev = NULL;
exit:
	return err;

}

static int asusec_lid_input_device_create(struct i2c_client *client)
{
	int err = 0;

	ec_chip->lid_indev = input_allocate_device();
	if(!ec_chip->lid_indev) {
		pr_debug("[EC][LID] lid_indev allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	ec_chip->lid_indev->name = "lid_input";
	ec_chip->lid_indev->phys = "/dev/input/lid_indev";
	ec_chip->lid_indev->dev.parent = &client->dev;

	asusec_lid_set_input_params(ec_chip->lid_indev);
	err = input_register_device(ec_chip->lid_indev);
	if(err) {
		pr_debug("[EC][LID] lid_indev registeration fail\n");
		goto exit_input_free;
	}
	return 0;

exit_input_free:
	input_free_device(ec_chip->lid_indev);
	ec_chip->lid_indev = NULL;
exit:
	return err;
}

static int asus_ec_dock_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	switch (event) {
	case P01_REMOVE:
		printk(KERN_INFO "[EC] P01 remove !\n");
		if(ec_chip->dock_in || ec_chip->isDockHWConnected == 1) {
			EC_Dock_out_callback();
		}
		return NOTIFY_DONE;
	case DOCK_PLUG_IN:
		EC_Dock_in_callback();
		return NOTIFY_DONE;
	case DOCK_PLUG_OUT:
		EC_Dock_out_callback();
		return NOTIFY_DONE;
	case DOCK_KEY_TOUCH_EVENT:
		EC_Ap_wake_callback();
		return NOTIFY_DONE;
	case DOCK_EXT_POWER_PLUG_IN:
		ec_chip->bat_info |= EC_EXT_POWER_PLUG_IN;
		ec_chip->isExtPower = true;
		if((ec_chip->isDockHWConnected == 1) && (ec_chip->ec_wakeup == 1) && (ec_chip->isUseWakeLock == false)) {
			wake_lock(&ec_chip->wake_lock_keywakeup);
			ec_chip->isUseWakeLock = true;
			printk(KERN_INFO "[EC] Wake lock\n");
		}
		queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_bat_work, 0.1*HZ);
		return NOTIFY_DONE;
	case DOCK_EXT_POWER_PLUG_OUT:
		ec_chip->bat_info |= EC_EXT_POWER_PLUG_OUT;
		ec_chip->isExtPower = false;
		if((ec_chip->isDockHWConnected == 1) && (ec_chip->ec_wakeup == 1) && (ec_chip->isUseWakeLock == true)) {
			wake_unlock(&ec_chip->wake_lock_keywakeup);
			ec_chip->isUseWakeLock = false;
			printk(KERN_INFO "[EC] Wake unlock\n");
		}
		queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_bat_work, 0.1*HZ);
		return NOTIFY_DONE;
	case DOCK_BATTERY_POWER_BAD:
		ec_chip->bat_info |= EC_POWER_BAD;
		queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_bat_work, 0.1*HZ);
		pr_debug("[EC] Low-power\n");
		return NOTIFY_DONE;
	case DOCK_LID_CHANGE_EVENT:
		EC_Lid_change_callback();
		return NOTIFY_DONE;
	default:
		return NOTIFY_DONE;
	}
}

static struct notifier_block asus_ec_dock_notifier = {
		.notifier_call = asus_ec_dock_event,
		.priority = DOCK_MP_NOTIFY,
};

// ASUS_BSP Jiunhau_Wang proc control file 20110909+++
void asus_ec_keyboard_control_cmd(const char *msg)
{
	pr_debug("[EC] %s\n", __FUNCTION__);
	switch (msg[2]) {
	case '0':
		asusec_keypad_disable(ec_chip->client);
		break;
	case '1':
		asusec_keypad_enable(ec_chip->client);
		break;
	case '2':
		asusec_keypad_reset(ec_chip->client);
		break;
	default:
		pr_debug("[EC] Not support function! \n");
		break;
	}
	return;
}

void asus_ec_touchpad_control_cmd(const char *msg)
{
	pr_debug("[EC] %s\n", __FUNCTION__);
	switch (msg[2]) {
	case '0':
		asusec_touchpad_disable(ec_chip->client);
		break;
	case '1':
		asusec_touchpad_enable(ec_chip->client);
		break;
	case '2':
		asusec_touchpad_reset(ec_chip->client);
		break;
	default:
		pr_debug("[EC] Not support function! \n");
		break;
	}
	return;

}

void asus_ec_bat_control_cmd(const char *msg)
{
	int ret = 0;
	pr_debug("[EC] %s\n", __FUNCTION__);
	switch (msg[2]) {
	case '0':
		ret = asusec_dock_battery_charging_status();
		switch(ret) {
		case EC_CHARGING_FULL:
			printk(KERN_INFO "[EC] charging full\n");
			break;
		case EC_CHARGING_ONGOING:
			printk(KERN_INFO "[EC] charging ongoing\n");
			break;
		case EC_CHARGING_NO:
			printk(KERN_INFO "[EC] no charging\n");
			break;
		case EC_CHARGING_ERR:
			printk(KERN_INFO "[EC] charging error\n");
			break;
		}
		break;
	case '1':
		ret = asusec_dock_cable_status();
		switch(ret) {
		case EC_CABLE_AC:
			printk(KERN_INFO "[EC] cable AC\n");
			break;
		case EC_CABLE_USB:
			printk(KERN_INFO "[EC] cable USB\n");
			break;
		case EC_CABLE_NO:
			printk(KERN_INFO "[EC] No cable\n");
			break;
		case EC_CABLE_ERR:
			printk(KERN_INFO "[EC] cable error\n");
			break;
		}
		break;
	default:
		pr_debug("[EC] Not support function! \n");
		break;
	}
	return;
}

int asus_ec_version_get_cmd(void)
{
	strcpy(ec_chip->ec_model_name, " \n");
	strcpy(ec_chip->ec_version, " \n");

	asusec_clear_i2c_buffer(ec_chip->client);

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		if (asusec_dockram_read_data(0x01) < 0){
			goto fail_to_access_ec;
		}
		strcpy(ec_chip->ec_model_name, &ec_chip->i2c_dm_data[1]);
		printk(KERN_INFO "[EC] Model Name: %s\n", ec_chip->ec_model_name);
		
		if (asusec_dockram_read_data(0x02) < 0){
			goto fail_to_access_ec;
		}
		strcpy(ec_chip->ec_version, &ec_chip->i2c_dm_data[1]);
		printk(KERN_INFO "[EC] FW Version: %s\n", ec_chip->ec_version);
		
		if (asusec_dockram_read_data(0x03) < 0){
			goto fail_to_access_ec;
		}
		pr_debug("[EC] Coding Format: %s\n", ec_chip->i2c_dm_data);
		
		if (asusec_dockram_read_data(0x04) < 0){
			goto fail_to_access_ec;
		}
		strcpy(ec_chip->dock_pid, &ec_chip->i2c_dm_data[1]);
		pr_debug("[EC] PID Version: %s\n", ec_chip->dock_pid);
	}
	else {
		pr_debug("[EC] No dock! \n");
	}
	return 0;

fail_to_access_ec:
	pr_debug("[EC] : DockRam read fail! \n");
	return -1;
}

static void asus_ec_diag_handle_all_cmd(const char *msg)
{
     switch (msg[0]) {
	 case 'b':  // i2c buffer clear and interrupt status clear
		 asusec_clear_i2c_buffer(ec_chip->client);
		 break;
	 case 'c':  // charging status
		 asus_ec_bat_control_cmd(msg);
		 break;
	 case 'k':  // keyboard enable or disable
         asus_ec_keyboard_control_cmd(msg);
         break;
     case 't':  // touchpad enable or disable
         asus_ec_touchpad_control_cmd(msg);
         break;
	 case 'v':  // get info of EC version
		 asus_ec_version_get_cmd();
		 break;
	 case 'r':
		 asusec_reset_dock();
		 break;
	 case 's':
		 asusec_sw_plug_in_out_dock();
		 break;
	 case 'h':
		 printk(KERN_INFO "[EC] Command description \n");
		 printk(KERN_INFO "===============================================================\n");
		 printk(KERN_INFO " b - i2c buffer clear and interrupt status clear\n");
		 printk(KERN_INFO " c - charging status (0 : charging - 1 : cable)\n");
		 printk(KERN_INFO " k - keyboard (0 : disable - 1 : enable)\n");
		 printk(KERN_INFO " r - reset dock\n");
		 printk(KERN_INFO " s - software reset\n");
		 printk(KERN_INFO " t - touchpad (0 : disable - 1 : enable - 2 : reset)\n");
		 printk(KERN_INFO " v - EC version\n");
		 printk(KERN_INFO "===============================================================\n");
		 break;
     default:
		 break;
     }
     return;
}

static ssize_t asus_ec_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	return 0;
}

static ssize_t asus_ec_write_proc(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
	char msg[ASUS_EC_PROC_MAX_BUFF_SIZE];
	if (len > ASUS_EC_PROC_MAX_BUFF_SIZE)
		len = ASUS_EC_PROC_MAX_BUFF_SIZE;

	if (copy_from_user(msg, buff, len))
		return -EFAULT;

	asus_ec_diag_handle_all_cmd(msg);

	return len;
}

static void asus_ec_create_ec_proc_file(void)
{
	pr_debug("[EC] %s\n", __FUNCTION__);
	ec_proc_file = create_proc_entry(ASUS_EC_PROC_FILE, ASUS_EC_PROC_FILE_PERMISSION, NULL);

	if (NULL == ec_proc_file) {
		pr_debug("[EC] ec proc file created failed!\n");
		return;
	}

	ec_proc_file->read_proc = asus_ec_read_proc;
	ec_proc_file->write_proc = asus_ec_write_proc;

	return;
}

// ASUS_BSP Jiunhau_Wang proc control file 20110909---

static int __devinit asusec_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;

	err = sysfs_create_group(&client->dev.kobj, &asusec_smbus_group);
	if (err) {
		pr_debug("[EC] Unable to create the sysfs\n");
		goto exit;
	}

	ec_chip = kzalloc(sizeof (struct asusec_chip), GFP_KERNEL);
	if (!ec_chip) {
		pr_debug("[EC] Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}
	ec_chip->private = kzalloc(sizeof(struct elantech_data), GFP_KERNEL);
	if (!ec_chip->private) {
		pr_debug("[EC] Memory allocation (elantech_data) fails\n");
		err = -ENOMEM;
		goto exit;
	}
		
	i2c_set_clientdata(client, ec_chip);
	ec_chip->client = client;
	ec_chip->client->driver = &asusec_driver;				
	ec_chip->client->flags = 1;

	mutex_init(&ec_chip->lock);
	mutex_init(&ec_chip->kbc_lock);
	mutex_init(&ec_chip->input_lock);
	mutex_init(&ec_chip->dock_init_lock);

	init_timer(&ec_chip->asusec_timer);
	ec_chip->asusec_timer.function = asusec_reset_counter;

	wake_lock_init(&ec_chip->wake_lock, WAKE_LOCK_SUSPEND, "asusec_wake");
	wake_lock_init(&ec_chip->wake_lock_init, WAKE_LOCK_SUSPEND, "asusec_wake_init");
	wake_lock_init(&ec_chip->wake_lock_keywakeup, WAKE_LOCK_SUSPEND, "asusec_wake_keywakeup");

	ec_chip->status = 0;
	ec_chip->dock_in = 0;
	ec_chip->dock_init = 0;
	ec_chip->lid_status = 0;
	ec_chip->d_index = 0;
	ec_chip->suspend_state = 0;
	ec_chip->init_success = 0;
	ec_chip->wakeup_lcd = 0;
	ec_chip->tp_wait_ack = 0;
	ec_chip->tp_enable = 1;
	ec_chip->re_init = 0;
	ec_chip->ec_wakeup = 0;
	ec_chip->isNeedResume = 0;
	ec_chip->bat_info = 0;
	ec_chip->i2c_err_count = 0;
	ec_chip->pad_err_count = 0;
	ec_chip->isDockHWConnected = 0;
	ec_chip->isHubSleepOn = 0;
	ec_chip->UsedPowerKey = false;
	ec_chip->isUseWakeLock = false;
	ec_chip->isExtPower = false;
	ec_chip->indev = NULL;
	ec_chip->lid_indev = NULL;
	ec_chip->private->abs_dev = NULL;
	
	asusec_dockram_init(client);
	
	cdev_add(asusec_cdev,asusec_dev,1) ;

	ec_chip->dock_sdev.name = DOCK_SDEV_NAME;
	ec_chip->dock_sdev.print_name = asusec_switch_name;
	ec_chip->dock_sdev.print_state = asusec_switch_state;
	if(switch_dev_register(&ec_chip->dock_sdev) < 0){
		pr_debug("[EC][%s] switch_dev_register for dock failed!\n", __FUNCTION__);
		goto exit;
	}
	switch_set_state(&ec_chip->dock_sdev, 0);

	// lid_sensor
	ec_chip->lid_sdev.name = LID_SDEV_NAME;
	ec_chip->lid_sdev.print_name = asusec_lid_switch_name;
	ec_chip->lid_sdev.print_state = asusec_lid_switch_state;
	if(switch_dev_register(&ec_chip->lid_sdev) < 0){
		pr_debug("[EC][%s] switch_dev_register for lid failed!\n", __FUNCTION__);
		goto exit;
	}
	switch_set_state(&ec_chip->lid_sdev, 0);
	
	asusec_lid_input_device_create(ec_chip->client);

	asusec_wq = create_singlethread_workqueue("asusec_wq");
	if (!asusec_wq) {
		err= -ESRCH;
		goto free_asus_ec_wq;
	}

	INIT_WORK(&ec_chip->asusec_work, asusec_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusec_dock_init_work, asusec_dock_init_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusec_fw_update_work, asusec_fw_update_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusec_i2c_err_check_work, asusec_i2c_err_check_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusec_led_on_work, asusec_keypad_led_on);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusec_led_off_work, asusec_keypad_led_off);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusec_lid_work, asusec_lid_report_function);
	INIT_DELAYED_WORK_DEFERRABLE(&asusec_stress_work, asusec_stresstest_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusec_resume_work, asusec_resume_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->asusec_bat_work, asusec_bat_work_function);
	
	// ASUS_BSP Jiunhau_Wang Add proc control cmd
	asus_ec_create_ec_proc_file();

	printk(KERN_INFO "[EC][%s] asusec probe complete !\n", __FUNCTION__);
#ifdef CONFIG_I2C_STRESS_TEST
       i2c_add_test_case(client, "AsusEC",ARRAY_AND_SIZE(gAsusECTestCaseInfo));
#endif

	return 0;

free_asus_ec_wq:
	destroy_workqueue(asusec_wq);

exit:
	return err;
}

static int __devexit asusec_remove(struct i2c_client *client)
{
	struct asusec_chip *chip = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s()\n", __func__);
	input_unregister_device(chip->indev);
	kfree(chip);
	return 0;
}

static ssize_t asusec_info_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "Model Name: %s, EC-FW Version: %s\n", 
			ec_chip->ec_model_name, ec_chip->ec_version);
}

static ssize_t asusec_lid_status(struct device *class, struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", ec_chip->lid_status);
}

static ssize_t asusec_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", ec_chip->status);
}

static ssize_t asusec_show_dock(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "dock detect = %d\n", ec_chip->dock_in);
}

static ssize_t asusec_store_led(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		if (buf[0] == '0')
			queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_led_off_work, 0);
		else 
			queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_led_on_work, 0);
	}
	
	return 0 ;
}

static ssize_t asusec_store_ec_wakeup(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	if (buf[0] == '0'){
		ec_chip->ec_wakeup = 0;
		pr_debug("[EC] Set EC shutdown when PAD in LP0\n");
	}
	else{
		ec_chip->ec_wakeup = 1;
		pr_debug("[EC] Keep EC active when PAD in LP0\n");
	}
		
	return 0 ;
}

static ssize_t asusec_show_drain(struct device *class,struct device_attribute *attr,char *buf)
{
	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		asusec_dockram_read_data(0x0A);

		ec_chip->i2c_dm_data[0] = 8;
		ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x8;
		asusec_dockram_write_data(0x0A,9);
		pr_debug("[EC] discharging 15 seconds\n");
		return sprintf(buf, "discharging 15 seconds\n");
	}	

	return 0;
}
// ASUS_BSP Jiunhau_Wang add temperature callback function +++

static ssize_t asusec_show_dock_temperature(struct device *class,struct device_attribute *attr,char *buf)
{
	int temperature = 0;
	int ret_val = 0;
	int decimal_value = 0;
	int integer_value = 0;

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		ret_val = asusec_dockram_read_data(0x14);

		if (ret_val < 0)
			return sprintf(buf, "-1\n");
		else{
			temperature = (ec_chip->i2c_dm_data[8] << 8 )| ec_chip->i2c_dm_data[7];
			integer_value = (temperature - 2731) / 10;
			decimal_value = (temperature - 2731) % 10;
			return sprintf(buf, "%d.%d\n", integer_value, decimal_value);
		}
	}

	return sprintf(buf, "-1\n");
}

// ASUS_BSP Jiunhau_Wang add temperature callback function ---

static ssize_t asusec_show_dock_battery(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret_val = 0;

	ret_val = asusec_dock_battery_callback();

	return sprintf(buf, "%d\n", (ret_val < 0)?(-1) : ret_val);
}

static ssize_t asusec_show_dock_battery_all(struct device *class,struct device_attribute *attr,char *buf)
{
	int i = 0;
	char temp_buf[64];
	int ret_val = 0;
	int battery_value = 0;

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		ret_val = asusec_dockram_read_data(0x14);

		if (ret_val < 0)
			return sprintf(buf, "fail to get dock-battery info\n");
		else{
			sprintf(temp_buf, "byte[0] = 0x%x\n", ec_chip->i2c_dm_data[i]);
			strcpy(buf, temp_buf);
			battery_value = (ec_chip->i2c_dm_data[2] << 8 )| ec_chip->i2c_dm_data[1];
			sprintf(temp_buf, "Battery Status : %d\n", battery_value);
			strcat(buf, temp_buf);
			battery_value = (ec_chip->i2c_dm_data[4] << 8 )| ec_chip->i2c_dm_data[3];
			sprintf(temp_buf, "Charge Voltage : %d\n", battery_value);
			strcat(buf, temp_buf);
			battery_value = (ec_chip->i2c_dm_data[6] << 8 )| ec_chip->i2c_dm_data[5];
			sprintf(temp_buf, "Charge Current : %d\n", battery_value);
			strcat(buf, temp_buf);
			battery_value = (ec_chip->i2c_dm_data[8] << 8 )| ec_chip->i2c_dm_data[7];
			sprintf(temp_buf, "Temperature : %d\n", battery_value);
			strcat(buf, temp_buf);
			battery_value = (ec_chip->i2c_dm_data[10] << 8 )| ec_chip->i2c_dm_data[9];
			sprintf(temp_buf, "Voltage : %d\n", battery_value);
			strcat(buf, temp_buf);
			battery_value = (ec_chip->i2c_dm_data[12] << 8 )| ec_chip->i2c_dm_data[11];
			sprintf(temp_buf, "Current : %d\n", battery_value);
			strcat(buf, temp_buf);
			battery_value = (ec_chip->i2c_dm_data[14] << 8 )| ec_chip->i2c_dm_data[13];
			sprintf(temp_buf, "RSOC : %d\n", battery_value);
			strcat(buf, temp_buf);
			battery_value = (ec_chip->i2c_dm_data[16] << 8 )| ec_chip->i2c_dm_data[15];
			sprintf(temp_buf, "Remaining Capacity : %d\n", battery_value);
			strcat(buf, temp_buf);
			return strlen(buf);
		}
	}

	return sprintf(buf, "fail to get dock-battery info\n");
}

static ssize_t asusec_show_dock_control_flag(struct device *class,struct device_attribute *attr,char *buf)
{
	int i = 0;
	char temp_buf[64];
	int ret_val = 0;

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		ret_val = asusec_dockram_read_data(0x0A);

		if (ret_val < 0)
			return sprintf(buf, "fail to get control-flag info\n");
		else{
			sprintf(temp_buf, "byte[0] = 0x%x\n", ec_chip->i2c_dm_data[i]);
			strcpy(buf, temp_buf);
			for (i = 1; i < 9; i++){
				sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_dm_data[i]);
				strcat(buf, temp_buf);
			}
			return strlen(buf);
		}
	}

	return sprintf(buf, "fail to get control-flag info\n");
}

static void asusec_dock_info_update(void){
	int ret_val = 0;
	char *envp[3];
	char name_buf[64];
	char state_buf[64];
	int env_offset = 0;
	int length = 0;

	if (ec_chip->ap_wake_wakeup && isHWDockConnected()){
		ec_chip->ap_wake_wakeup = 0;
		ret_val = asusec_i2c_test(ec_chip->client);
		if((ret_val >= 0) && (asusec_dockram_read_data(0x04) >= 0)){
			strcpy(ec_chip->dock_pid, &ec_chip->i2c_dm_data[1]);
			pr_debug("[EC] PID Version: %s\n", ec_chip->dock_pid);
		}
		if ((ret_val >= 0) && (asusec_dockram_read_data(0x02) >= 0)){
			if (ec_chip->dock_in &&
				strncmp(ec_chip->ec_version, &ec_chip->i2c_dm_data[1], 11)){

				strcpy(ec_chip->ec_version, &ec_chip->i2c_dm_data[1]);
				pr_debug("[EC] EC-FW Version: %s\n", ec_chip->ec_version);

				length = strlen(ec_chip->ec_version);
				ec_chip->ec_version[length] = '\0';
				snprintf(name_buf, sizeof(name_buf), "SWITCH_NAME=%s", ec_chip->ec_version);
				envp[env_offset++] = name_buf;

				snprintf(state_buf, sizeof(state_buf), "SWITCH_STATE=%s", "10");
				envp[env_offset++] = state_buf;

				envp[env_offset] = '\0';
				kobject_uevent_env(&ec_chip->dock_sdev.dev->kobj, KOBJ_CHANGE, envp);
			}
		}
	}
}

static void asusec_dock_status_check(void){
	if ((ec_chip->ec_version[6] <= '0') &&
	    (ec_chip->ec_version[7] <= '2') &&
	    (ec_chip->ec_version[8] <= '0') &&
	    (ec_chip->ec_version[9] <= '9') &&
	    (ec_chip->ec_version[10] == '\0')){
		ec_chip->init_success = 0;
		//wake_lock(&ec_chip->wake_lock_init);
		cancel_delayed_work(&ec_chip->asusec_dock_init_work);
		queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_dock_init_work, 0);
	} else if ((ec_chip->ec_version[6] <= '0') &&
	    (ec_chip->ec_version[7] <= '2') &&
	    (ec_chip->ec_version[8] <= '1') &&
	    (ec_chip->ec_version[9] <= '2') &&
	    (ec_chip->ec_version[10] == '\0')){
		ec_chip->init_success = 0;
		//wake_lock(&ec_chip->wake_lock_init);
		cancel_delayed_work(&ec_chip->asusec_dock_init_work);
		queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_dock_init_work, 0);
	} else if (isHWDockConnected() && ec_chip->indev){
		ec_chip->init_success = 0;
		//wake_lock(&ec_chip->wake_lock_init);
		cancel_delayed_work(&ec_chip->asusec_dock_init_work);
		queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_dock_init_work, 0);
	} else if (strcmp(ec_chip->dock_pid, "PCBA-EP101") == 0){
		ec_chip->init_success = 0;
		//wake_lock(&ec_chip->wake_lock_init);
		cancel_delayed_work(&ec_chip->asusec_dock_init_work);
		queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_dock_init_work, 0);
	}
}

static int asusec_suspend(struct i2c_client *client, pm_message_t mesg){
	pr_debug("[EC] suspend+\n");
#if 0
	if (ec_chip->isDockHWConnected) {
		if(ec_chip->pre_suspend_enable == 0){
		//ec_chip->init_success = 0;
			ec_chip->isNeedResume = 1;
			ec_chip->last_bat = -1;
			ec_chip->UsedPowerKey = false;
		}
		else{
			printk(KERN_INFO "[EC] suspend before HUB_SLEEP!\n");
		}
	} 
	else{
		printk(KERN_INFO "[EC] No dock!\n");
	}
#endif
	pr_debug("[EC] suspend-\n");
	return 0;
}

extern int AX_MicroP_IsP01Connected(void);

static void asusec_resume_work_function(struct work_struct *dat){

	if (AX_MicroP_IsP01Connected()==1) {
		printk(KERN_INFO "[EC][%s]\n", __FUNCTION__);
		if(isHWDockConnected()) {
			asusec_check_pad_HW_statsus();
			if((get_MicroP_HUB_SLEEP_STATUS()==1)) {
				printk(KERN_INFO "[EC][%s] resume+\n", __FUNCTION__);
				ec_chip->suspend_state = 0;
				printk(KERN_INFO "[EC] asusec_dock_info_update\n"); 
				asusec_dock_info_update();
				printk(KERN_INFO "[EC] asusec_dock_status_check\n");
				asusec_dock_status_check();
				ec_chip->pre_suspend_enable = 0;
				printk(KERN_INFO "[EC][%s] resume-\n", __FUNCTION__);
			}
			else{
				cancel_delayed_work(&ec_chip->asusec_resume_work);
				queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_resume_work, HZ*1);
				printk(KERN_INFO "[EC] MicroP resume not ready\n");
			}
		}
		else
		{
			printk(KERN_INFO "[EC] DOCK not connected!\n");
		}
	}
	else
	{
		printk(KERN_INFO "[EC] P01 not connected!\n");
	}
}

static void asusec_bat_work_function(struct work_struct *dat)
{
	if((ec_chip->dock_in) && (ec_chip->bat_info != 0)) {
		pr_debug("[EC] bat status : %d\n", ec_chip->bat_info);
		if(ec_chip->dock_init) {
			if(ec_chip->bat_info & EC_EXT_POWER_PLUG_IN) {
				ec_chip->bat_info &= ~EC_EXT_POWER_PLUG_IN;
				pr_debug("[EC] EC_EXT_POWER_PLUG_IN\n");
				EC_Get_EXT_POWER_PLUG_IN_Ready();
			}
			if(ec_chip->bat_info & EC_EXT_POWER_PLUG_OUT) {
				ec_chip->bat_info &= ~EC_EXT_POWER_PLUG_OUT;
				pr_debug("[EC] EC_EXT_POWER_PLUG_OUT\n");
				EC_Get_EXT_POWER_PLUG_OUT_Ready();
			}
			if(ec_chip->bat_info & EC_POWER_BAD) {
				ec_chip->bat_info &= ~EC_POWER_BAD;
				pr_debug("[EC] EC_POWER_BAD\n");
				EC_Get_DOCK_BATTERY_POWER_BAD_READY();
			}
		}
		else {
			queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_bat_work, HZ*1);
		}
	}
}

int asusec_sus_res_callback(bool status){
	// status = 1 -> hub_sleep : on -> EC resume

	ec_chip->isHubSleepOn = 1;

	if(status == 1) {
		if(ec_chip->pre_suspend_enable == 1) {
			if(ec_chip->isNeedResume == 1) {
				pr_debug("[EC][resume] HUB_SLEEP call me !\n"); 
				queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_resume_work, 0);
			}
			else {
				printk(KERN_INFO "[EC] DOCK doesn`t need resume!\n");
			}
		}
	}
	// status = 0 -> hub_sleep : off -> EC suspend (HW control)
	else{
		ec_chip->isNeedResume = 1;
		ec_chip->last_bat = -1;
	}
	return 0;
}
EXPORT_SYMBOL(asusec_sus_res_callback);

static int asusec_resume(struct i2c_client *client){
	pr_debug("[EC] resume+\n");
#if 0
	if(ec_chip->pre_suspend_enable == 0){
		printk(KERN_INFO "[EC] resume!\n");
		queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_resume_work, HZ*1);
	}
	else{
		printk(KERN_INFO "[EC] late resume!\n");
	}
#endif
	pr_debug("[EC] resume-\n");
	return 0;
}

static int asusec_set_wakeup_cmd(void){
	int ret_val = 0;

	if (ec_chip->dock_in){
		ret_val = asusec_i2c_test(ec_chip->client);
		if(ret_val >= 0){
			asusec_dockram_read_data(0x0A);
			ec_chip->i2c_dm_data[0] = 8;
			if (ec_chip->ec_wakeup){
				ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x80;
			} else {
				ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0x7F;
			}
			asusec_dockram_write_data(0x0A,9);
		}
	}
	return 0;
}
static ssize_t asusec_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t asusec_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", (ec_chip->dock_in ? "10" : "0"));
}

static ssize_t asusec_lid_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", LID_SDEV_NAME);
}

static ssize_t asusec_lid_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", (ec_chip->lid_status ? "0" : "1"));
}

static int asusec_open(struct inode *inode, struct file *flip){
	pr_debug(" ");
	return 0;
}
static int asusec_release(struct inode *inode, struct file *flip){
	pr_debug(" ");
	return 0;
}
static long asusec_ioctl(struct file *flip,
					unsigned int cmd, unsigned long arg){
	int err = 1;
	char *envp[3];
	char name_buf[64];
	int env_offset = 0;
	int length = 0;

	if (_IOC_TYPE(cmd) != ASUSEC_IOC_MAGIC)
	 return -ENOTTY;
	if (_IOC_NR(cmd) > ASUSEC_IOC_MAXNR)
	return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;

	 switch (cmd) {
        case ASUSEC_POLLING_DATA:
			if (arg == ASUSEC_IOCTL_HEAVY){
				pr_debug("[EC] heavy polling\n");
				ec_chip->polling_rate = 80;
				queue_delayed_work_on(0, asusec_wq, &asusec_stress_work, HZ/ec_chip->polling_rate);
			}
			else if (arg == ASUSEC_IOCTL_NORMAL){
				pr_debug("[EC] normal polling\n");
				ec_chip->polling_rate = 10;
				queue_delayed_work_on(0, asusec_wq, &asusec_stress_work, HZ/ec_chip->polling_rate);
			}
			else if  (arg == ASUSEC_IOCTL_END){
				pr_debug("[EC] polling end\n");
		    	cancel_delayed_work_sync(&asusec_stress_work) ;
			}
			else
				return -ENOTTY;
			break;
		case ASUSEC_FW_UPDATE:
			if (ec_chip->dock_in){
				pr_debug("[EC] ASUSEC_FW_UPDATE\n");
				buff_in_ptr = 0;
				buff_out_ptr = 0;
				h2ec_count = 0;
				ec_chip->suspend_state = 0;
				ec_chip->status = 0;
				asusec_reset_dock();
				wake_lock_timeout(&ec_chip->wake_lock, 3*60*HZ);
				msleep(3000);
				asusec_clear_i2c_buffer(ec_chip->client);
				ec_chip->op_mode = 1;
				ec_chip->i2c_dm_data[0] = 0x02;
				ec_chip->i2c_dm_data[1] = 0x55;
				ec_chip->i2c_dm_data[2] = 0xAA;
				asusec_dockram_write_data(0x40,3);
				ec_chip->init_success = 0;
				msleep(1000);
				printk(KERN_INFO "[EC] Firmware update initial complete !!\n");
			} else {
				pr_debug("[EC] No dock detected\n");
				return -1;
			}
			break;
		case ASUSEC_INIT:
			msleep(500);
			ec_chip->status = 0;
			queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_dock_init_work, 0);
			msleep(2500);
			pr_debug("[EC] ASUSEC_INIT - EC version: %s\n", ec_chip->ec_version);
			length = strlen(ec_chip->ec_version);
			ec_chip->ec_version[length] = '\0';
			snprintf(name_buf, sizeof(name_buf), "SWITCH_NAME=%s", ec_chip->ec_version);
			envp[env_offset++] = name_buf;
			envp[env_offset] = '\0';
			kobject_uevent_env(&ec_chip->dock_sdev.dev->kobj, KOBJ_CHANGE, envp);
			break;
		case ASUSEC_TP_CONTROL:
			pr_debug("[EC] ASUSEC_TP_CONTROL\n");
			if ((ec_chip->op_mode == 0) && ec_chip->dock_in && ec_chip->init_success){				
				err = asusec_tp_control(arg);
				return err;
			}
			else
				return -ENOTTY;
		case ASUSEC_EC_WAKEUP:
			pr_debug("[EC] ASUSEC_EC_WAKEUP, arg = %lu\n", arg);
			if (arg == ASUSEC_EC_OFF){
				ec_chip->ec_wakeup = 0;
				if((ec_chip->isDockHWConnected == 1) && (ec_chip->isExtPower == true) && (ec_chip->isUseWakeLock == true))  {
					wake_unlock(&ec_chip->wake_lock_keywakeup);
					ec_chip->isUseWakeLock = false;
					printk(KERN_INFO "[EC] EC_WAKEUP wake unlock\n");
				}
				pr_debug("[EC] Set EC shutdown when PAD in LP0\n");
				return asusec_set_wakeup_cmd();
			}
			else if (arg == ASUSEC_EC_ON){
				ec_chip->ec_wakeup = 1;
				if((ec_chip->isDockHWConnected == 1) && (ec_chip->isExtPower == true) && (ec_chip->isUseWakeLock == false))  {
					wake_lock(&ec_chip->wake_lock_keywakeup);
					ec_chip->isUseWakeLock = true;
					printk(KERN_INFO "[EC] EC_WAKEUP wake lock\n");
				}
				pr_debug("[EC] Keep EC active when PAD in LP0\n");
				return asusec_set_wakeup_cmd();
			}
			else {
				pr_debug("[EC] Unknown argument");
				return -ENOTTY;
			}
		case ASUSEC_FW_DUMMY:
			ec_chip->i2c_dm_data[0] = 0x02;
			ec_chip->i2c_dm_data[1] = 0x55;
			ec_chip->i2c_dm_data[2] = 0xAA;
			asusec_dockram_write_data(0x40,3);
			pr_debug(KERN_INFO "[EC][FW] reset! \n");
			return 0;
	 case ASUSEC_DISABLE_KEY:
			pr_debug(KERN_INFO "[EC][KEYBOARD] Change keyboard setting!\n");
			if (arg == ASUSEC_KEYBOARD_DISABLE){
				pr_debug("[EC] Disable keyboard\n");
				return asusec_keypad_disable(ec_chip->client);
			}
			else if (arg == ASUSEC_KEYBOARD_ENABLE) {
				pr_debug("[EC] Enable keyboard\n");
				return asusec_keypad_enable(ec_chip->client);
			}
			else{
				pr_debug("[EC] Unknown argument");
				return -ENOTTY;
			}
        default: /* redundant, as cmd was checked against MAXNR */
            return -ENOTTY;
	}
    return 0;
}

static int BuffDataSize(void)
{   
    int in = buff_in_ptr;
    int out = buff_out_ptr;

    if (in >= out)
    {
        return (in - out);
    }
    else
    {
        return ((EC_BUFF_LEN - out) + in);
    }
}
static void BuffPush(char data)
{

    if (BuffDataSize() >= (EC_BUFF_LEN -1)) 
    {
		pr_debug("[EC] EC work-buf overflow \n");
        return;
    }

	pr_debug("[EC] ec_to_host_buffer[%d] = 0x%x\n", buff_in_ptr, data);
    ec_to_host_buffer[buff_in_ptr] = data;
    buff_in_ptr++;
    if (buff_in_ptr >= EC_BUFF_LEN) 
    {
        buff_in_ptr = 0;
    }    
}

static char BuffGet(void)
{
    char c = (char)0;

    if (BuffDataSize() != 0) 
    {
        c = (char) ec_to_host_buffer[buff_out_ptr];        
        buff_out_ptr++;
         if (buff_out_ptr >= EC_BUFF_LEN) 
         {
             buff_out_ptr = 0;
         }
    }
    return c;
}

static ssize_t ec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int i = 0;
    int ret;
    char tmp_buf[EC_BUFF_LEN];
	static int total_buf = 0;

	mutex_lock(&ec_chip->lock);
	mutex_unlock(&ec_chip->lock);
    
    while ((BuffDataSize() > 0) && count)
    {
        tmp_buf[i] = BuffGet();
        count--;
        i++;
		total_buf++;
    }	

    ret = copy_to_user(buf, tmp_buf, i);
    if (ret == 0)
    {
        ret = i; // No error. Return the number of byte read.
    }

    return ret;
}

static ssize_t ec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int err;    
    int i;

    if (h2ec_count > 0) 
    {                   /* There is still data in the buffer that */
        return -EBUSY;  /* was not sent to the EC */
    }
    if (count > EC_BUFF_LEN) 
    {
        return -EINVAL; /* data size is too big */
    }
    
    err = copy_from_user(host_to_ec_buffer, buf, count);
    if (err)
    {
        pr_debug("[EC] ec_write copy error\n");
        return err;
    }
   
    h2ec_count = count;
    for (i = 0; i < count ; i++) 
    {
		pr_debug("[EC] smbus write, i = %d, data = 0x%x\n", i, host_to_ec_buffer[i]);
		i2c_smbus_write_byte_data(&dockram_client, host_to_ec_buffer[i],0);
    }
    h2ec_count = 0;
    return count;

}

static int asusec_event(struct input_dev *dev, unsigned int type, unsigned int code, int value){
	pr_debug("[EC] type = 0x%x, code = 0x%x, value = 0x%x\n", type, code, value);
	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		if ((type == EV_LED) && (code == LED_CAPSL)){
			if(value == 0){
				queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_led_off_work, 0);
				return 0;
			} else {
				queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_led_on_work, 0);
				return 0;
			}
		}
	}
	return -ENOTTY;		
}

int asusec_dock_resume(void){
	pr_debug("[EC] keyboard opened, op_mode = %d\n", ec_chip->op_mode);
	if (ec_chip->op_mode == 0){
		pr_debug("[EC] keyboard opened\n");
		if (ec_chip->suspend_state == 0){
			cancel_delayed_work(&ec_chip->asusec_dock_init_work);
			queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_dock_init_work, 0);
		}
	}
	return 0;

}
EXPORT_SYMBOL(asusec_dock_resume);

int asusec_open_keyboard(void){
	pr_debug("[EC] keyboard opened, op_mode = %d\n", ec_chip->op_mode);
	if ((ec_chip->suspend_state == 0) && (ec_chip->op_mode == 0)){
		pr_debug("[EC] keyboard opened\n");
		ec_chip->init_success = 0;
//		wake_lock(&ec_chip->wake_lock_init);
		cancel_delayed_work(&ec_chip->asusec_dock_init_work);
		queue_delayed_work_on(0, asusec_wq, &ec_chip->asusec_dock_init_work, 0);
	}
	return 0;
	
}
EXPORT_SYMBOL(asusec_open_keyboard);


int asusec_close_keyboard(void){
	int ret_val;

	if (ec_chip->status == 0){
		return 0;
	} else if ((ec_chip->suspend_state == 0) && (ec_chip->op_mode == 0)){
		pr_debug("[EC] keyboard closed\n");
		if (ec_chip->dock_in){
			ret_val = asusec_i2c_test(ec_chip->client);
			if(ret_val < 0){
				goto fail_to_access_ec;
			}
		
			asusec_dockram_read_data(0x0A);

			ec_chip->i2c_dm_data[0] = 8;
			ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0xDF;
			ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x22;
			asusec_dockram_write_data(0x0A,9);
		}
	
fail_to_access_ec:	
		ec_chip->init_success = 0;
	}
	return 0;
	
}
EXPORT_SYMBOL(asusec_close_keyboard);

int asusec_suspend_pre_process_callback(bool power_status){

	//int i = 0;
	printk(KERN_INFO "[EC][%s] +++\n", __FUNCTION__);

	ec_chip->isHubSleepOn = 0;

// ASUS_BSP Eason_Chang +++
#ifdef CONFIG_BATTERY_ASUS    
    setDockInitNotReady();
#endif
// ASUS_BSP Eason_Chang ---
	if (ec_chip->isDockHWConnected){

#if 0 // max wait time = 4 sec (BL : 2 + dock init :2)
		//ASUS_BSP wait EC initial complete +++
		while(ec_chip->isNeedResume != 0){
			i++;
			msleep(100);
		}
		printk(KERN_INFO "[EC] wait EC initial complete (%d)\n", i * 100);
		//ASUS_BSP wait EC initial complete ---
#endif
		//ASUS_BSP clear EC buf brfore suspend +++
		asusec_clear_i2c_buffer(ec_chip->client);
		//ASUS_BSP clear EC buf brfore suspend ---
#if P02_ER2
		// turn off the MicroP hub power
		if(power_status == 0) {
			//ec_chip->init_success = 0;	
			//ASUS_BSP setting PS_HOLD = 1 +++
			//asusec_suspend_hub();
			//ASUS_BSP setting PS_HOLD = 1 ---
			ec_chip->init_retry_count = 0;
			ec_chip->isNeedResume = 1;
			ec_chip->last_bat = -1;
			ec_chip->pre_suspend_enable = 1;
			ec_chip->i2c_err_count = 0;
			ec_chip->pad_err_count = 0;
			ec_chip->UsedPowerKey = false;
		}
#endif
	}
	else{
		printk(KERN_INFO "[EC] No dock!\n");
	}
	printk(KERN_INFO "[EC][%s] ---\n", __FUNCTION__);
	return 0;
}
EXPORT_SYMBOL(asusec_suspend_pre_process_callback);

int asusec_suspend_hub(void){
	int ret_val;

	pr_debug("[EC][%s] +++\n", __FUNCTION__);
	if (ec_chip->dock_in){
		ret_val = asusec_i2c_test(ec_chip->client);
		if(ret_val < 0){
			goto fail_to_access_ec;
		}
		
		asusec_dockram_read_data(0x0A);

		ec_chip->i2c_dm_data[0] = 8;
		ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0xDF;
		ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x22;
		if (ec_chip->ec_wakeup){
			ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x80;
		} else {
			ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0x7F;
		}
		asusec_dockram_write_data(0x0A,9);	
	}
	
fail_to_access_ec:		
	flush_workqueue(asusec_wq);
	ec_chip->suspend_state = 1;
	pr_debug("[EC][%s] ---\n", __FUNCTION__);
	return 0;
	
}

int asusec_is_ac_over_10v_callback(void){

	int ret_val;

	pr_debug("[EC] access dockram\n");
	if (ec_chip->dock_in){
		ret_val = asusec_i2c_test(ec_chip->client);
		if(ret_val < 0){
			goto fail_to_access_ec;
		}	
		asusec_dockram_read_data(0x0A);
		pr_debug("[EC] byte[1] = 0x%x\n", ec_chip->i2c_dm_data[1]);

		return ec_chip->i2c_dm_data[1] & 0x20;
	}
		
fail_to_access_ec:	
	pr_debug("[EC] dock doesn't exist or fail to access ec\n");
	return -1;
}
EXPORT_SYMBOL(asusec_is_ac_over_10v_callback);

int asusec_is_battery_full_callback(int full){

	int ret_val;

	pr_debug("[EC] access dockram\n");
	if (ec_chip->dock_in){
		msleep(500);
		ret_val = asusec_i2c_test(ec_chip->client);
		if(ret_val < 0){
			goto fail_to_access_ec;
		}
		asusec_dockram_read_data(0x0A);

		ec_chip->i2c_dm_data[0] = 8;
		if (full){
			ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x10;
		} else{
			ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0xEF;
		}
		ret_val = asusec_dockram_write_data(0x0A,9);
		return ret_val;
	}
	
fail_to_access_ec:	
	pr_debug("[EC] dock doesn't exist or fail to access ec\n");
	return -1;
}
EXPORT_SYMBOL(asusec_is_battery_full_callback);

int asusec_dock_battery_callback(void){

	int bat_percentage = 0;
	int ret_val = 0;
	int bat_retry_count = 3;
	static int zero_count = 0;

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		// get value from I2C 
		ret_val = asusec_dockram_read_data(0x14);

		if (ret_val < 0){
			printk(KERN_INFO "[EC][%s] read I2c error!\n", __FUNCTION__);
			return 0;
		} 

		// check the bat range info
		do{
			bat_percentage = (ec_chip->i2c_dm_data[14] << 8 )| ec_chip->i2c_dm_data[13];
			if((bat_percentage >= 0) && (bat_percentage <= 100))
				break;
			else{
				ret_val = asusec_dockram_read_data(0x14);
				if (ret_val < 0){
					printk(KERN_INFO "[EC][%s] read I2c error!\n", __FUNCTION__);
				} 
			}
		}while (bat_retry_count-- > 0);

		if (ret_val < 0){
			printk(KERN_INFO "[EC][%s] read I2c error!\n", __FUNCTION__);
			return 0;
		} 

		// process bat value
		if((bat_percentage >= 0) && (bat_percentage <= 100)) {
			printk(KERN_INFO "[EC][%s] last_bat = %d | bat_percentage = %d\n", 
				   __FUNCTION__, ec_chip->last_bat, bat_percentage);
			// Dock-in first detect bat percentage
			if(ec_chip->last_bat == -1) {
				ec_chip->last_bat = bat_percentage;
				zero_count = 0;			
			}
			// second 
			else { 
				if(bat_percentage == 0) {
					if(ec_chip->last_bat != 0) {
						zero_count++;
						if(zero_count > 3) {
							ec_chip->last_bat = bat_percentage;	
						}
					}
				}
				else{
					ec_chip->last_bat = bat_percentage;
					zero_count = 0;
				}
			}
			return ec_chip->last_bat;
		}
		else{
			printk(KERN_INFO "[EC][%s] error bat range!(%d)\n", 
				   __FUNCTION__, bat_percentage);
			return 0;
		}

	}

	if(ec_chip->op_mode == 1) {
		printk(KERN_INFO "[EC][BAT] firmware update mode, stop updating bat status!!\n");
	}

	if(ec_chip->dock_in == 0) {
		printk(KERN_INFO "[EC][BAT] No dock !!\n");
	}
	return 0;
}
EXPORT_SYMBOL(asusec_dock_battery_callback);

enum asusec_Charging_Status asusec_dock_battery_charging_status(void){

	int ret_val = 0;

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		ret_val = asusec_dockram_read_data(0x0A);

		if(ret_val < 0){
			goto fail_to_access_ec;
		}
		else
		{
			if (ec_chip->i2c_dm_data[1] & EC_BAT_F_C_MASK){
				pr_debug("[EC][BAT] Full charged !\n");
				return EC_CHARGING_FULL;
			}
			else if (ec_chip->i2c_dm_data[1] & EC_BAT_C_MASK){
				pr_debug("[EC][BAT] Charging ongoing !\n");
				return EC_CHARGING_ONGOING;
			}
			else{
				pr_debug("[EC][BAT] No charging!\n");
				return EC_CHARGING_NO;
			}
		}
	}
	return EC_CHARGING_ERR;
	
fail_to_access_ec:	
	pr_debug("[EC] dock doesn't exist or fail to access ec\n");
	return EC_CHARGING_ERR;
}
EXPORT_SYMBOL(asusec_dock_battery_charging_status);

// EC firmware : 0213 read 0x0A 
// 1000010  - No Cable
// 0100111  - AC Cable
// 1000011  - USB Cable
enum asusec_Cable_Status asusec_dock_cable_status(void){

	int ret_val = 0;
	u8 val = 0;
	int i = 0;

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		// Note : Need to wait EC bat status
		msleep(500);
        for (i = 0; i < ASUSEC_RETRY_BAT_COUNT; i++) 
		{
			ret_val = asusec_dockram_read_data(0x0A);
			if(ret_val < 0){
				goto fail_to_access_ec;
			}
            if(ret_val == 32){
				val = ec_chip->i2c_dm_data[1];
				// No cable : pass ; AC/USB cable : retry 
				if((val & 0x1) != 0x1)
					break;
			}
			msleep(50);
		}

		val = ec_chip->i2c_dm_data[1];
		if ((val & 0x1) == 0x1){
			if(((val >> 5) & 0x1) == 0x1) {
				pr_debug("[EC][BAT] AC Cable !\n");
				return EC_CABLE_AC;
			}
			else{
				pr_debug("[EC][BAT] USB Cable !\n");
				return EC_CABLE_USB;
			}
		}
		else{
			pr_debug("[EC][BAT] No cable!\n");
			return EC_CABLE_NO;
		}
	}
	return EC_CABLE_ERR;
	
fail_to_access_ec:	
	pr_debug("[EC][BAT] dock doesn't exist or fail to access ec\n");
	return EC_CABLE_ERR;

}
EXPORT_SYMBOL(asusec_dock_cable_status);

int asusec_Dock_Ready_status(void)
{
	int ret = 0;

	ret = ec_chip->dock_init & (~ec_chip->isNeedResume);
	if(ret == 0) {
		printk(KERN_INFO "[EC] get dock bat ready ? (%d)\n", ret);
	}
	
	return ret;
}
EXPORT_SYMBOL(asusec_Dock_Ready_status);

static int __init asusec_init(void)
{
	int err_code = 0;	

	if (asusec_major) {
		asusec_dev = MKDEV(asusec_major, asusec_minor);
		err_code = register_chrdev_region(asusec_dev, 1, "asusec");
	} else {
		err_code = alloc_chrdev_region(&asusec_dev, asusec_minor, 1,"asusec");
		asusec_major = MAJOR(asusec_dev);
	}
	pr_debug("[EC][%s] cdev_alloc\n", __FUNCTION__);
	asusec_cdev = cdev_alloc() ;
	asusec_cdev->owner = THIS_MODULE ;
	asusec_cdev->ops = &asusec_fops ;
		
	err_code=i2c_add_driver(&asusec_driver);
	if(err_code){
		pr_debug("[EC] i2c_add_driver fail\n") ;
		goto i2c_add_driver_fail ;
	}
	asusec_class = class_create(THIS_MODULE, "asusec");
	if(asusec_class <= 0){
		pr_debug("[EC] asusec_class create fail\n");
		err_code = -1;
		goto class_create_fail ;
	}
	asusec_device = device_create(asusec_class, NULL, MKDEV(asusec_major, asusec_minor), NULL, "asusec" );
	if(asusec_device <= 0){
		pr_debug("[EC] asusec_device create fail\n");
		err_code = -1;
		goto device_create_fail ;
	}
	register_microp_notifier(&asus_ec_dock_notifier);

	pr_debug("[EC] return value %d\n", err_code) ;

	return 0;

device_create_fail :
	class_destroy(asusec_class) ;	
class_create_fail :
	i2c_del_driver(&asusec_driver);	
i2c_add_driver_fail :

	return err_code;

}

static void __exit asusec_exit(void)
{
	device_destroy(asusec_class,MKDEV(asusec_major, asusec_minor)) ;
	class_destroy(asusec_class) ;
	i2c_del_driver(&asusec_driver);
	unregister_chrdev_region(asusec_dev, 1);
	unregister_microp_notifier(&asus_ec_dock_notifier);
	switch_dev_unregister(&ec_chip->dock_sdev);
	switch_dev_unregister(&ec_chip->lid_sdev);
}

module_init(asusec_init);
module_exit(asusec_exit);
