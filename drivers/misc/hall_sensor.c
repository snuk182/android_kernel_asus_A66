/*
*  drivers/misc/hall-sensor.c
*
* Copyright (C) 2011 AsusTek, Inc.
* Author: Sina Chou
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


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/hs_notify.h>
#include <linux/hall_sensor.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/microp_notify.h>

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif


static int g_driver_initialized=0;
static int g_p01_status=0;
static int g_p01_pre_status=0;
static char rf_switch_status[6] = "phone";
struct switch_dev hs_switch_dev;
static int g_is_suspend=0;

extern enum A60K_HWID g_A60K_hwID;

extern int AX_MicroP_IsP01Connected(void);
extern int micropSendNotify(unsigned long val);

/*
* define p01 status macro used here
*/

#define IS_P01_BACKCOVER_ON(status)		(status & P01_STATUS_BACKCOVER_BIT)



/*
* get p01_status
*/
int is_p01_backcover_on(void)
{
	if(g_driver_initialized){
		return IS_P01_BACKCOVER_ON(g_p01_status)?1:0;
	}
	else{
		return -1;
	}
}
EXPORT_SYMBOL_GPL(is_p01_backcover_on);

int is_p01_hdmi_inserted(void)
{
    return -1;
}
EXPORT_SYMBOL_GPL(is_p01_hdmi_inserted);

void set_antenna_to_phone(void)
{
	if(g_A60K_hwID >= A66_HW_ID_SR1_1) {
		gpio_set_value(89, 0);
		gpio_set_value(90, 1);
	}
	else if(g_A60K_hwID > A60K_EVB){
		gpio_set_value(128, 1);
		gpio_set_value(129, 0);
	}
}

void set_antenna_to_pad(void)
{
	if(g_A60K_hwID >= A66_HW_ID_SR1_1) {
		gpio_set_value(89, 1);
		gpio_set_value(90, 0);
	}
	else if(g_A60K_hwID > A60K_EVB){
		gpio_set_value(128, 0);
		gpio_set_value(129, 1);
	}
}


/*
* notification part
* each driver have to use register_hs_notifier() to register a callback-function
* if the driver want to be notified while hall sensor status is changing
*
* register_hs_notifier(): drivers can use the func to register a callback
* unregister_hs_notifier(): drivers can use the func to unregister a callback
*/

static BLOCKING_NOTIFIER_HEAD(hall_sensor_chain_head);

int hs_notifier_call_chain(unsigned long val)
{
	return (blocking_notifier_call_chain(&hall_sensor_chain_head, val, &g_p01_status) == NOTIFY_BAD) ? -EINVAL : 0;
}


int register_hs_notifier(struct notifier_block *nb)
{
	if(!g_driver_initialized)
		printk("###Warning: hall-sensor driver not initialized\r\n" );
                
	blocking_notifier_chain_register(&hall_sensor_chain_head, nb);        
	return g_p01_status;
}
EXPORT_SYMBOL_GPL(register_hs_notifier);

int unregister_hs_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&hall_sensor_chain_head, nb);
}
EXPORT_SYMBOL_GPL(unregister_hs_notifier);



static void p01_state_change_notify(struct work_struct *work)
{
	printk("[hallsensor]  pre_s=%2x, now_s=%2x\r\n",g_p01_pre_status,g_p01_status);
	if(g_p01_status==g_p01_pre_status)
	{
		printk("[hallsensor] p01: no changed\r\n");
		return;
	}

	if((g_p01_status^g_p01_pre_status) & P01_STATUS_BACKCOVER_BIT) // p01 back status changed
	{
		if(IS_P01_BACKCOVER_ON(g_p01_status))
		{
			hs_notifier_call_chain(P01_EVENT_NOTIFY_BACKCOVER_ADD);
			// set switch dev state to send an uevent
			switch_set_state(&hs_switch_dev,P01_EVENT_NOTIFY_BACKCOVER_ADD);
			g_p01_pre_status=g_p01_pre_status | P01_STATUS_BACKCOVER_BIT;
		}
		else
		{			
			hs_notifier_call_chain(P01_EVENT_NOTIFY_BACKCOVER_REMOVE);
			// set switch dev state to send an uevent
			switch_set_state(&hs_switch_dev,P01_EVENT_NOTIFY_BACKCOVER_REMOVE);                            
			g_p01_pre_status=g_p01_pre_status & ~P01_STATUS_BACKCOVER_BIT;

#ifndef ASUS_FACTORY_BUILD
			msleep(200);
			if(g_is_suspend)
			{
				printk("[hallsensor] send power key notify to wake up P02\r\n");
				micropSendNotify(P01_PWR_KEY_PRESSED);
				msleep(10);
				micropSendNotify(P01_PWR_KEY_RELEASED);
			}
#endif			
		}
	}
}


static DECLARE_WORK(p01_notify_work, p01_state_change_notify);


static int hs_mp_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	switch (event) {
	case P01_ADD:
		printk("[hall_sensor] pad add \r\n");
#ifndef ASUS_FACTORY_BUILD
		if( !(gpio_get_value(39)) )
		{
			irq_set_irq_wake(39, 1);
			set_antenna_to_pad();
			strcpy(rf_switch_status, "pad");
			g_p01_status=g_p01_status | P01_STATUS_BACKCOVER_BIT;
			schedule_work(&p01_notify_work);
		}
#endif			
		return NOTIFY_DONE;
                
	case P01_REMOVE:
		printk("[hall_sensor] pad remove \r\n");
		return NOTIFY_DONE;

	default:
		return NOTIFY_DONE;
	}
}

static struct notifier_block hs_mp_notifier = {
        .notifier_call = hs_mp_event,
        .priority = LCD_MP_NOTIFY,
};

/*
*
* Export to Display driver to inform HDMI status change
*
*/
void notify_hs_hdmi_insert(void)
{
	printk("\r\n[hallsensor] notify_hs_hdmi_insert\r\n");
}
EXPORT_SYMBOL_GPL(notify_hs_hdmi_insert);

void notify_hs_hdmi_remove(void)
{
	printk("\r\n[hallsensor] notify_hs_hdmi_remove\r\n");
}
EXPORT_SYMBOL_GPL(notify_hs_hdmi_remove);




struct hall_sensor_data 
{
	unsigned int gpio;
	int irq;
};



static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hall_sensor_data *hs_data = (struct hall_sensor_data *)dev->platform_data;

	return sprintf(buf, "%d\n", gpio_get_value(hs_data->gpio)?0:1);
}


static DEVICE_ATTR(state, S_IRUGO, state_show, NULL);


static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct hall_sensor_data *hs_data = (struct hall_sensor_data *)dev_id;

	if(gpio_get_value(hs_data->gpio))
	{
#ifndef ASUS_FACTORY_BUILD
		if(IS_P01_BACKCOVER_ON(g_p01_status))
		{
			irq_set_irq_wake(irq, 0);
#endif

			set_antenna_to_phone();
			strcpy(rf_switch_status, "phone");
			g_p01_status=g_p01_status & ~P01_STATUS_BACKCOVER_BIT;
			
#ifndef ASUS_FACTORY_BUILD
		}
		else
		{
			return IRQ_HANDLED;
		}
#endif
	}
	else
	{
#ifndef ASUS_FACTORY_BUILD
		if(AX_MicroP_IsP01Connected())
		{
			irq_set_irq_wake(irq, 1);
#endif

			set_antenna_to_pad();
			strcpy(rf_switch_status, "pad");
			g_p01_status=g_p01_status | P01_STATUS_BACKCOVER_BIT;
			
#ifndef ASUS_FACTORY_BUILD
		}
		else
		{
			return IRQ_HANDLED;
		}
#endif
	}
	
	schedule_work(&p01_notify_work);
	return IRQ_HANDLED;
}

static int rf_switch_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len = strlen(rf_switch_status);
	
	sprintf(page, "%s\n", rf_switch_status);
	return len ;
}

static int rf_switch_write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char buf[16];
	unsigned long len = count;
	
	if (len > 16){
		len = 16;
	}

	if (copy_from_user(buf, buffer, len)){
		return -EFAULT;
	}
	
	if (strncmp(buf, "phone", 5) == 0) 
	{
		strcpy(rf_switch_status, "phone");
		set_antenna_to_phone();
	} 
	else if (strncmp(buf, "pad", 3) == 0) 
	{
		strcpy(rf_switch_status, "pad");
		set_antenna_to_pad();
	}
	else 
	{
		printk("usage: echo {phone/pad} > /proc/rf_switch_status\n");
	}
	
	return len;
}

static int hall_sensor_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	printk("[hallsensor] suspend ++\r\n");
	g_is_suspend = 1;
	return 0;
}

static int hall_sensor_resume(struct platform_device *pdev)
{
	printk("[hallsensor] resume ++\r\n");
	g_is_suspend = 0;
	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
struct early_suspend early_suspend;

static void hs_early_suspend(struct early_suspend *h)
{
	printk("[hallsensor] early suspend ++\r\n");
	g_is_suspend = 1;
}

static void hs_late_resume(struct early_suspend *h)
{
	printk("[hallsensor] late resume ++\r\n");
	g_is_suspend = 0;
}
#endif

static int __devinit hall_sensor_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct hall_sensor_data *hs_data;
	int ret=0;
	int rf_switch_gpio_1 = -1;
	int rf_switch_gpio_2 = -1;
	struct proc_dir_entry *entry;
        
	printk("[hallsensor] hall_sensor_probe ++\r\n");

	hs_data = kzalloc(sizeof(struct hall_sensor_data), GFP_KERNEL);
	if (!hs_data){
		return -ENOMEM;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IO, "hall_sensor_gpio");

	if (res) {
		hs_data->gpio=res->start;
	}
	else {
		pr_warn("%s : can't find hall_sensor gpio.\n", __func__);
		goto err_get_resource;
	}

	// registered as gpio switch device
	hs_switch_dev.name="hall_sensor";
	ret=switch_dev_register(&hs_switch_dev);

	if (ret < 0){
		goto err_register_switch;
	}

	// fill into platform data
	pdev->dev.platform_data=hs_data;


	// setup gpio for hall-sensor
	ret = gpio_request(hs_data->gpio, pdev->name);
	if (ret < 0){
		goto err_request_gpio;
	}

	ret = gpio_direction_input(hs_data->gpio);
	if (ret < 0){
		goto err_set_gpio_input;
	}

	// request a gpio irq
	hs_data->irq = gpio_to_irq(hs_data->gpio);
	if (hs_data->irq < 0) {
		ret = hs_data->irq;
		goto err_detect_irq_num_failed;
	}

	ret = request_irq(hs_data->irq, gpio_irq_handler, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, pdev->name, hs_data);

	if (ret < 0){
		goto err_request_irq;
	}
	
	//suport ATD test
	if (pdev->name) {
		ret = device_create_file(&pdev->dev, &dev_attr_state);
		if (unlikely(ret)) {
			dev_err(&pdev->dev, "Failed creating device attrs\n");
			ret = -EINVAL;
			goto err_attr_create;
		}
	}

	// initialize the status of hall-sensor
	if(gpio_get_value(hs_data->gpio)){
		g_p01_status=g_p01_status & ~P01_STATUS_BACKCOVER_BIT;
	}
	else{
		g_p01_status=g_p01_status | P01_STATUS_BACKCOVER_BIT;
	}


	g_p01_pre_status=g_p01_status;
	
	g_driver_initialized=1;
	
	//setup gpio for rf switch
	if(g_A60K_hwID != A60K_EVB){
		if(g_A60K_hwID >= A66_HW_ID_SR1_1) {
			rf_switch_gpio_1 = 90;
			rf_switch_gpio_2 = 89;
		}
		else if(g_A60K_hwID > A60K_EVB){
			rf_switch_gpio_1 = 128;
			rf_switch_gpio_2 = 129;
		}
		
		ret = gpio_request(rf_switch_gpio_1, "rf_switch_128");
		if (ret < 0){
			goto err_request_gpio;
		}
	
		ret = gpio_request(rf_switch_gpio_2, "rf_switch_129");
		if (ret < 0){
			goto err_request_gpio;
		}

		if(is_p01_backcover_on()){
			strcpy(rf_switch_status, "pad");
			
			ret = gpio_direction_output(rf_switch_gpio_1, 0);
			if (ret < 0){
				goto err_set_gpio_input;
			}
			
			ret = gpio_direction_output(rf_switch_gpio_2, 1);
			if (ret < 0){
				goto err_set_gpio_input;
			}
		}
		else{
			strcpy(rf_switch_status, "phone");
			
			ret = gpio_direction_output(rf_switch_gpio_1, 1);
			if (ret < 0){
				goto err_set_gpio_input;
			}
			
			ret = gpio_direction_output(rf_switch_gpio_2, 0);
			if (ret < 0){
				goto err_set_gpio_input;
			}
		}
	}
	
	entry = create_proc_entry("rf_switch_status", 0666, NULL);
	if (entry == NULL) {
		printk("[hallsensor] create_proc_entry fail\r\n");
	}
	else {
		entry->read_proc = rf_switch_read_proc;
		entry->write_proc = rf_switch_write_proc;
	}
	
	early_suspend.level = 1;
	early_suspend.suspend = hs_early_suspend;
	early_suspend.resume = hs_late_resume;
	register_early_suspend(&early_suspend);
	
	register_microp_notifier(&hs_mp_notifier);
	
	printk("[hallsensor] hall_sensor_probe  pin<%d>=%d, pre=%d, cur=%d\r\n",hs_data->gpio,gpio_get_value(hs_data->gpio),g_p01_pre_status,g_p01_status);
	return 0;
    
err_get_resource:
err_register_switch:
err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
err_request_gpio:
err_attr_create:
	return ret;
}

static int __devexit hall_sensor_remove(struct platform_device *pdev)
{
	struct hall_sensor_data *hs_data = pdev->dev.platform_data;
	free_irq(gpio_to_irq(hs_data->gpio), 0);
	switch_dev_unregister(&hs_switch_dev);
	gpio_free(hs_data->gpio);
	g_driver_initialized=0;
	
	return 0;
}

static struct platform_driver hall_sensor_driver = 
{
	.probe = hall_sensor_probe,
	.remove = __devexit_p(hall_sensor_remove),
	.driver = {
		.name	= "hall_sensor",
		.owner	= THIS_MODULE,
	},
	.suspend = hall_sensor_suspend,
	.resume = hall_sensor_resume,
};

static int __init hall_sensor_init(void)
{
	
	printk("[hallsensor] hall_sensor_init ++\r\n");
	return platform_driver_register(&hall_sensor_driver);
}

static void __exit hall_sensor_exit(void)
{
	printk("[hallsensor] hall_sensor_exit ++\r\n");
	platform_driver_unregister(&hall_sensor_driver);
}

fs_initcall(hall_sensor_init);
module_exit(hall_sensor_exit);

MODULE_AUTHOR("Sina Chou <sina_chou@asus.com>");
MODULE_DESCRIPTION("Hall Sensor Driver");
MODULE_LICENSE("GPL");
