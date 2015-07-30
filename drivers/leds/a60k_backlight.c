/*
 * leds-msm-pmic.c - MSM PMIC LEDs driver.
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/platform_device.h>


#include <mach/pmic.h>
#include <linux/pwm.h>
#include <linux/mfd/pm8xxx/pm8921.h>

#include <linux/time.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <mach/board.h>
#include <linux/leds.h>
#include <linux/wakelock.h>
	
#define A60K_PWM_FREQ_HZ 510
#define A60K_PWM_PERIOD_USEC (USEC_PER_SEC / A60K_PWM_FREQ_HZ)
#define A60K_PWM_LEVEL 255
#define A60K_PWM_DUTY_LEVEL \
    (A60K_PWM_PERIOD_USEC / A60K_PWM_LEVEL)

#define MAX_BACKLIGHT_BRIGHTNESS 255
#define MIN_BACKLIGHT_BRIGHTNESS 0
#include <linux/microp_notify.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#include <linux/workqueue.h>

// ASUS_BSP +++ Tingyi "[A66][Bootup] Turne on P0x backlight when bootup"    
#include <linux/gpio.h>
// ASUS_BSP --- Tingyi "[A66][Bootup] Turne on P0x backlight when bootup"    

//debug mask G0 add/remove p02
//debug mask G1 scalar info
//debug mask G2 brightness
//debug mask G3 ignore value
extern int p02_backlight_write(u16 val);
extern int p02_backlight_read(void);
extern int switch_backlight(int on);
extern int switch_backlight_and_panel(int on);
extern int get_backlight(void);
extern bool read_scaler_version(u8 *ver);
extern int scaler_set_waiting_icon(int set);
extern bool hdmi_exist(void);

static int a60k_backlight_registered;
//static struct pwm_device *bl_lpm;
struct a60k_backlight_data *backlight_pdata;
static int backlight_mode_state= 0;
#if 0
static int a60k_backlight_value;
#endif
static int p01_brightness_value;
static int p01_backlight_state = 0;
static int p01_panel_state = 0;
#if 0
static int p02_auto_brightness_set_backlight(int);
#endif

static unsigned long P01_last_blctrl_time=0;
static unsigned long P01_last_brctrl_time=0;
static bool P01_set_BL_cmd=0;

static struct workqueue_struct *backlight_workqueue;
static struct workqueue_struct *brightness_workqueue;
static struct workqueue_struct *P01_Add_Remove_workqueue;
//static struct work_struct switch_backlight_on_work;
//static struct work_struct switch_backlight_off_work;
static struct delayed_work switch_backlight_work;
static struct delayed_work turn_off_panel_work;
static struct delayed_work set_brightness_work;
static struct delayed_work disable_pad_wait_icon_work;
static struct work_struct add_func_work;
static struct work_struct remove_func_work;
static struct workqueue_struct *scaler_version_workqueue;
static struct delayed_work g_write_scaler_work;
static struct wake_lock turn_off_pad_panel_wake_lock;
static struct mutex scaler_mutex_lock;
#if 0
static bool g_bFirst = true;
#endif
u8 g_scaler_ver;
static bool g_new_pad_bl_control_method = true;
static int g_more_delay_after_resume_for_old_scaler;
bool i2c_error = false;




enum backlight_mode{
    a60k = 0,
    p01 = 1,
};
/*
struct adjust_brightness{
    struct work_struct adjust_brightness_work;         
    int brightness;
};

static void adjust_brightness_func(struct work_struct *work)
{
    struct adjust_brightness* my_BL_work = container_of(work, struct adjust_brightness, adjust_brightness_work);
    p02_backlight_write(my_BL_work->brightness);
    printk(DBGMSK_BL_G2"----------p02_backlight_write=%d----------\n",my_BL_work->brightness);
    kfree(my_BL_work);

}

static int add_brightness_work(int *brightness)
{
    if (hdmi_exist() == true){
    struct adjust_brightness* adjust_bl;
    adjust_bl = kmalloc(sizeof(struct adjust_brightness), GFP_KERNEL);
    INIT_DELAYED_WORK(&adjust_bl->adjust_brightness_work, adjust_brightness_func);
    adjust_bl->brightness = *brightness;
    queue_delayed_work(brightness_workqueue, &adjust_bl->adjust_brightness_work);
    }
    return 0;
}

struct ignore_bitghtness{
    struct delayed_work ignore_brightness_work;
    int ignore_brightness;
};

static void ignore_brightness_func(struct work_struct *work)
{
    struct ignore_bitghtness* my_ignore_work = container_of(work, struct ignore_bitghtness, ignore_brightness_work.work);
    if (my_ignore_work->ignore_brightness == p01_brightness_value){
        printk(DBGMSK_BL_G3"----------set ignored value=%d----------\n",p01_brightness_value);
        p02_backlight_write(my_ignore_work->ignore_brightness);
    }
    kfree(my_ignore_work);
}

static int add_ignore_brightness_work(int *brightness)
{
    if (hdmi_exist() == true){
        struct ignore_bitghtness* ignore_bl;
        ignore_bl = kmalloc(sizeof(struct ignore_bitghtness), GFP_KERNEL);
        INIT_DELAYED_WORK(&ignore_bl->ignore_brightness_work, ignore_brightness_func);
        ignore_bl->ignore_brightness = *brightness;
        schedule_delayed_work(&ignore_bl->ignore_brightness_work,50);
    }
    return 0;
}
*/

static int a60k_to_p01_value(int value)
{
	if (value == 0 || value == 1000 || value == 2000) {
        printk("[BL] value=%d, turn off P02 backlight\n", value);
        return 0;
    }
    
	if (value > 2000 && value <= 2255){  //auto mode: 0~100
		//printk("auto mode: value=%d\n", value);
		value = (value -2020) * 100 / 235;
	}else if (value > 1000 && value <= 1255) { //autodoor mode: 40~100
		//printk("outdoor mode: value=%d\n", value);
		value = ((value - 1020) * ((60*10000)/ 235)) / 10000 + 40 ;
	}else if (value <= 255) {
		//printk("normal mode: value=%d\n", value); //normal mode: 0~60
		value = ((value - 20) * ((60*10000)/ 255)) / 10000;
		if (value > 60)
			value = 60;
	}
	
	if (value <=2)
		value =2 ;
	if (value >= 100)
		value =100;
	
	return value;
}

#if 0
static int p01_to_a60k_value(int value)
{
    value = value * 255 / 100 ;
    return value;
}
#endif

extern int amoled_set_brightness(int index);//Mickey+++
#if 0
static void A66_set_autobrightness_backlight(int value)
{
    int index = 0;
    printk(DBGMSK_BL_G2"----------a60K_set_backlight=%d----------\n",value); 

    if (value >= 1000)
    {
            if (value == 1000) {
                index = -1;             // display off
                value = 0;
            }
            else if (value == 1030) {    //40 
                index = 0;
                value = 23;     
            }   
            else if (value == 1031) {    //50
                index = 1;      
                value = 23;
            }
            else if (value == 1032) {    //60
                index = 2;      
                value = 23;
            }
            else if (value == 1033) {    //70
                index = 3;      
                value = 23;
            }
            else if (value == 1034) {    //80
                index = 4;      
                value = 27;
            }    
            else if (value == 1035) {    //100
                index = 6;      
                value = 34;
            }
            else if (value == 1036) {    //120
                index = 8;      
                value = 40;
            }
            else if (value == 1037) {    //150
                index = 11;     
                value = 50;
            }
            else if (value == 1038) {    //180
                index = 14;      
                value = 61;
            }
            else if (value == 1039) {    //200
                index = 16;     
                value = 70;
            }
            else if (value == 1040) {    //230
                index = 19;     
                value = 94;
            }
            else if (value == 1041) {    //280
                index = 24;     
                value = 100;
            }
            else if (value >= 1042) {    //300
                index = 26;     
                value = 100;
            }
   }
   if(index == -1)
       printk(DBGMSK_BL_G2"[A66_BL] turn off display\n");
   //amoled_set_brightness(index);
}
#endif
int a60K_set_backlight(int value)
{
    int index = 0;
    int ret = 0;
    printk(DBGMSK_BL_G2"----------a60K_set_backlight=%d----------\n",value);    

            //Mickey+++
    if (g_A60K_hwID>=A66_HW_ID_SR1_1)
    {

        if (value == MIN_BACKLIGHT_BRIGHTNESS) {        //turn off display
            printk(DBGMSK_BL_G2"[A66_BL] turn off display\n");
            //amoled_set_brightness(-1);//Mickey+++, no need to turn off A66 backlight now
            return 0;
        }

        index = ((value-20)*((27*10000)/235))/10000;
        if (index <= 0)
        	index=0;
        else if (index >= 26)
            index=26;
        //ret = amoled_set_brightness(index);
        if (ret) {
			pr_err("amoled_set_brightness failed\n");
			return ret;
		}
            //Mickey---
    }
/*    else 
    {
        if (bl_lpm) 
        {
            ret = pwm_config(bl_lpm, A60K_PWM_DUTY_LEVEL *
                 value, A60K_PWM_PERIOD_USEC);

            if (ret) {
                 pr_err("pwm_config on lpm failed %d\n", ret);
                 return ret;
            }
            if (value) {
                 ret = pwm_enable(bl_lpm);
                 if (ret)
                        pr_err("pwm enable/disable on lpm failed"
                            "for bl %d\n",  value);
            } else {
                 pwm_disable(bl_lpm);
            }
        }
    }
*/
    return 0;
}

#if 0
static int p02_auto_brightness_set_backlight(int value)
{
        if (value > 1045)
            value = 100;
        else if (value >= 1030)
        {
            if (value == 1030)
                value = 2;              //nits:32.5
            else if (value == 1031)
                value = 2;             //32.5
            else if (value == 1032)
                value = 5;             //40
            else if (value == 1033)
                value = 11;            //60
            else if (value == 1034)
                value = 19;            //80
            else if (value == 1035)
                value = 28;            //100
            else if (value == 1036)
                value = 34;            //120
            else if (value == 1037)
                value = 47;            //150
            else if (value == 1038)
                value = 57;            //180
            else if (value == 1039)
                value = 65;            //200
            else if (value == 1040)
                value = 77;            //230
            else if (value == 1041)
                value = 97;            //280
            else if (value >= 1042)
                value = 100;           //290
        }
        else  //value = 1000;
            value = 0;

        return value;
}
#endif

/*
void p01_set_backlight(int value)
{
    static unsigned long time[2];
    printk(DBGMSK_BL_G2"----------p01_set_backlight=%d----------\n",value);

        time[1] = jiffies;
        if (jiffies_to_msecs(time[1] - time[0]) < 50){
                printk(DBGMSK_BL_G3"----------ignore_value=%d----------\n",value);
                add_ignore_brightness_work(&value);
            return;
        }
        else
                time[0] = time[1];

    if (value == 0 ) {
        if ( backlight_state ==1)
        {
            queue_work(backlight_workqueue, &switch_backlight_off_work);
         } 
        else
        {
        }
    } 
    else 
    {
        if ( backlight_state ==1)
        {
            add_brightness_work(&value);
        } 
        else
        {   
            queue_work(backlight_workqueue, &switch_backlight_on_work);
            //add_brightness_work(&value);
        }
    }
    return;
}
*/

#define P02_ON2OFF_DEBOUNCE_TIME      300  //ms
#define P02_OFF2ON_DEBOUNCE_TIME     1300  //ms
extern unsigned long g_resume_time_stamp;
#define P02_SCALER_INIT_NEED_TIME     500  //ms
void p01_set_backlight(void)
{
    printk(DBGMSK_BL_G2"++++++++++[BACKLIGHT_SCALAR] p01_set_backlight enter, p01_backlight_state=%d ,p01_brightness_value=%d  ++++++++++++\n",p01_backlight_state,p01_brightness_value  );

    if (p01_brightness_value == 0)
    {
        if ( p01_backlight_state ==1) // we should turn off BL
        {
            unsigned long jtime = jiffies;
			// check if last command time > P02_ON2OFF_DEBOUNCE_TIME
			if ( jiffies_to_msecs(jtime - P01_last_blctrl_time) > P02_ON2OFF_DEBOUNCE_TIME )
            {
				P01_set_BL_cmd = 0;
                printk(DBGMSK_BL_G3"++++++++++[BACKLIGHT_SCALAR] p01_set_backlight QUEUE backlight_workqueue TURN OFF NOW  [%d]ms++++++++++++\n", jiffies_to_msecs(jtime - P01_last_blctrl_time));
				queue_delayed_work(backlight_workqueue, &switch_backlight_work, msecs_to_jiffies(0));
            }
            else
            {
                // check if there is still pending work
                if (delayed_work_pending(&switch_backlight_work))
                {
                    P01_set_BL_cmd = 0;  //just update global variable
                    printk(DBGMSK_BL_G3"++++++++++[BACKLIGHT_SCALAR] p01_set_backlight QUEUE backlight_workqueue TURN OFF PENDING++++++++++++\n");
				}
				else
                {
                    P01_set_BL_cmd = 0;
                    queue_delayed_work(backlight_workqueue, &switch_backlight_work, msecs_to_jiffies(P02_ON2OFF_DEBOUNCE_TIME));
                    printk(DBGMSK_BL_G3"++++++++++[BACKLIGHT_SCALAR] p01_set_backlight QUEUE backlight_workqueue TURN OFF DELAY++++++++++++\n");
                }
			}
        }
        else  // backlight is off already
        {
			// set command to BL_OFF, in case there is still pending work
			P01_set_BL_cmd = 0;
            // check if there is still pending work
            if (delayed_work_pending(&switch_backlight_work))
            {
                cancel_delayed_work_sync(&switch_backlight_work);
                printk(DBGMSK_BL_G3"++++++++++[BACKLIGHT_SCALAR] p01_set_backlight CANCEL backlight_workqueue ++++++++++++\n");
			}
		}
    } 
    else 
    {
        if ( p01_backlight_state ==1) // backlight is ON already
        {
#if 0
            unsigned long jtime = jiffies;

			if ( jiffies_to_msecs(jtime - P01_last_brctrl_time) > 50 )
            {
                printk(DBGMSK_BL_G3"++++++++++[BACKLIGHT_SCALAR] p01_set_backlight QUEUE brightness_workqueue NOW  %d+++++++++++\n",p01_brightness_value);
				queue_delayed_work(brightness_workqueue, &set_brightness_work, msecs_to_jiffies(0));
            }
            else
            {

                // check if there is still pending work
                if (delayed_work_pending(&set_brightness_work))
                {
                    // do nothing
                    printk(DBGMSK_BL_G3"++++++++++[BACKLIGHT_SCALAR] p01_set_backlight QUEUE brightness_workqueue PENDING %d++++++++++++\n",p01_brightness_value);
				}
				else

                {
                    queue_delayed_work(brightness_workqueue, &set_brightness_work, 50);
                    printk(DBGMSK_BL_G3"++++++++++[BACKLIGHT_SCALAR] p01_set_backlight QUEUE brightness_workqueue DELAY %d+++++++++++\n",p01_brightness_value);
                }
			}
#else
			if (delayed_work_pending(&set_brightness_work))
			{
				 printk("++++++++++[BACKLIGHT_SCALAR] p01_set_backlight QUEUE brightness_workqueue PENDING %d++++++++++++\n",p01_brightness_value);
			}
			else 
			{
				queue_delayed_work(brightness_workqueue, &set_brightness_work, msecs_to_jiffies(0));
				printk("++++++++++[BACKLIGHT_SCALAR] p01_set_backlight QUEUE brightness_workqueue NOW  %d+++++++++++\n",p01_brightness_value);
			}
#endif
        } 
        else // backlight is off, so we should turn it on first
        {   
            unsigned long jtime = jiffies;
            unsigned long queue_delay_time;
			// check if last command time > P02_OFF2ON_DEBOUNCE_TIME
			if ( jiffies_to_msecs(jtime - P01_last_blctrl_time) > P02_OFF2ON_DEBOUNCE_TIME )
            {
				P01_set_BL_cmd = 1;
				if ((jtime - g_resume_time_stamp) > msecs_to_jiffies(P02_SCALER_INIT_NEED_TIME+g_more_delay_after_resume_for_old_scaler))
				    queue_delay_time = 0;
				else
				    queue_delay_time = msecs_to_jiffies(P02_SCALER_INIT_NEED_TIME+g_more_delay_after_resume_for_old_scaler) - (jtime - g_resume_time_stamp);
                queue_delayed_work(backlight_workqueue, &switch_backlight_work, (queue_delay_time));
                printk(DBGMSK_BL_G3 "++++++++++[BACKLIGHT_SCALAR] p01_set_backlight QUEUE backlight_workqueue TURN ON NOW [%d]ms ++++++++++++\n",jiffies_to_msecs(queue_delay_time));
                //queue_delayed_work(brightness_workqueue, &set_brightness_work, (queue_delay_time+200));
            }
            else
            {
                // check if there is still pending work
                if (delayed_work_pending(&switch_backlight_work))
                {
                    P01_set_BL_cmd = 1;  //just update global variable
                    printk(DBGMSK_BL_G3 "++++++++++[BACKLIGHT_SCALAR] p01_set_backlight QUEUE backlight_workqueue TURN ON PENDING++++++++++++\n");
				}
				else
                {
                    P01_set_BL_cmd = 1;
                    queue_delayed_work(backlight_workqueue, &switch_backlight_work, msecs_to_jiffies(P02_OFF2ON_DEBOUNCE_TIME));
                    printk(DBGMSK_BL_G3 "++++++++++[BACKLIGHT_SCALAR] p01_set_backlight QUEUE backlight_workqueue TURN ON DELAY++++++++++++\n");
                    //queue_delayed_work(brightness_workqueue, &set_brightness_work, msecs_to_jiffies(P02_OFF2ON_DEBOUNCE_TIME + 200));
                }
			}
        }
    }
    return;
}

static void P02_write_scaler_work(struct work_struct *work)
{       
        int ret = 0;
        ret = read_scaler_version(&g_scaler_ver);
        return ;
}

// ASUS_BSP +++ Tingyi "[A66][Bootup] Turne on P0x backlight when bootup"    
extern unsigned g_GPIO_HDMI_5V_ENABLE;
extern unsigned g_GPIO_P01_I2C_LS_OE;
// ASUS_BSP --- Tingyi "[A66][Bootup] Turne on P0x backlight when bootup"
#if 0
static void set_bl_brightness(struct led_classdev *led_cdev,
                    enum led_brightness value)
{
/*
    if (value == 20)
    {
        printk(DBGMSK_BL_G2"++++++++++set_bl_brightness=%d , RETURN+++++++++\n",value);
        return;
	}
*/

// ASUS_BSP +++ Tingyi "[A66][Bootup] Turne on P0x backlight when bootup"    
	if (value == 999){
		gpio_set_value(g_GPIO_P01_I2C_LS_OE, 1);
		gpio_set_value(7,0);
		msleep(15);
		printk("++++++++++[BACKLIGHT_SCALAR] set_bl_brightness=%d at booup +++++++++\n",100);
#if 1
		switch_backlight_and_panel(1);
		msleep(P02_OFF2ON_DEBOUNCE_TIME);
		p02_backlight_write(100);
#else
       p01_brightness_value = a60k_to_p01_value(200);
       p01_set_backlight();
#endif		
		gpio_set_value(g_GPIO_P01_I2C_LS_OE, 0);
		gpio_set_value(7,0);
		return;
	}
// ASUS_BSP --- Tingyi "[A66][Bootup] Turne on P0x backlight when bootup"    

    if (g_bFirst) {
        if (value == 0 || value == 1000) 
            g_bFirst = false;
       return;
    }

    if (value >= 1000) // auto brightness
    {
        if (backlight_mode_state == a60k)
            A66_set_autobrightness_backlight(value);
        else //if (backlight_mode_state == p01)
        {
            p01_brightness_value = p02_auto_brightness_set_backlight(value);
            printk(DBGMSK_BL_G2"++++++++++[BACKLIGHT_SCALAR] set_bl_brightness=%d +++++++++\n",p01_brightness_value);
			//queue_delayed_work(brightness_workqueue, &set_brightness_work, msecs_to_jiffies(0));
			p01_set_backlight();
		}
    }
    else 
    {
        a60k_backlight_value = value;
        p01_brightness_value = a60k_to_p01_value(value);

        if (backlight_mode_state == a60k)
            a60K_set_backlight(a60k_backlight_value);
        else //if (backlight_mode_state == p01)
            p01_set_backlight();
    }        
}
#endif

void set_pad_bl_brightness(struct led_classdev *led_cdev,
                    enum led_brightness value) {
	
	p01_brightness_value = a60k_to_p01_value(value);
	
	if(backlight_mode_state == p01) {
			p01_set_backlight();
	}					
}

EXPORT_SYMBOL(set_pad_bl_brightness);

static void set_p01_brightness_work(struct work_struct *work)
{
    if (hdmi_exist() == true)
    {
        mutex_lock(&scaler_mutex_lock);
        printk(DBGMSK_BL_G2"++++++++++[BACKLIGHT_SCALAR] lock_enter: set brightness %d ++++++++++\n", p01_brightness_value);
        p02_backlight_write(p01_brightness_value);
        P01_last_brctrl_time = jiffies;
        mutex_unlock(&scaler_mutex_lock);
    }
}

//Bruno++
// TomChu miniport +++
#ifdef CONFIG_FM34
#include "../../sound/fm34.h"
#endif
// TomChu miniport ---
#define P02_TURN_OFF_PANEL_DELAY_TIME  3000
static void set_p01_backlight_func(struct work_struct *work)
{
    if (hdmi_exist() == true){

        if (delayed_work_pending(&turn_off_panel_work))
        {
             cancel_delayed_work_sync(&turn_off_panel_work);
        }
//        if (g_new_pad_bl_control_method)
        {
            if (P01_set_BL_cmd == 1)
		    {
                if (p01_panel_state == 0)
                {
                    printk("++++++++++[BACKLIGHT_SCALAR] turn on BL and PANEL ++++++++++\n");
                    mutex_lock(&scaler_mutex_lock);
                    printk("++++++++++[BACKLIGHT_SCALAR] lock_enter: turn on BL++++++++++\n");
                    switch_backlight_and_panel(1);
                    P01_last_blctrl_time = jiffies;
                    p01_backlight_state = 1;
                    p01_panel_state = 1;
                    msleep(400);
                    mutex_unlock(&scaler_mutex_lock);
#ifdef CONFIG_FM34               
                    init_fm34_when_bl_on();     //Bruno++
#endif
					mutex_lock(&scaler_mutex_lock);
                    printk("++++++++++[BACKLIGHT_SCALAR] lock_enter: set brightness %d ++++++++++\n", p01_brightness_value);
                    p02_backlight_write(p01_brightness_value);
                    mutex_unlock(&scaler_mutex_lock);
			    }
                else
                {
                    printk("++++++++++[BACKLIGHT_SCALAR] turn on BL only ++++++++++\n");
                    mutex_lock(&scaler_mutex_lock);
                    printk("++++++++++[BACKLIGHT_SCALAR] lock_enter: turn on BL++++++++++\n");
                    switch_backlight(1);
                    p01_backlight_state = 1;
                    mutex_unlock(&scaler_mutex_lock);
                    mutex_lock(&scaler_mutex_lock);
                    printk("++++++++++[BACKLIGHT_SCALAR] lock_enter: set brightness %d ++++++++++\n", p01_brightness_value);
                    p02_backlight_write(p01_brightness_value);
                    mutex_unlock(&scaler_mutex_lock);
			    }
            }
            else
            {
                mutex_lock(&scaler_mutex_lock);
                printk(DBGMSK_BL_G2"++++++++++[BACKLIGHT_SCALAR] lock_enter: turn off BL++++++++++\n");
                switch_backlight(0);
                p01_backlight_state = 0;
                mutex_unlock(&scaler_mutex_lock);
                queue_delayed_work(backlight_workqueue, &turn_off_panel_work, msecs_to_jiffies(P02_TURN_OFF_PANEL_DELAY_TIME));
                wake_lock_timeout(&turn_off_pad_panel_wake_lock, msecs_to_jiffies(P02_TURN_OFF_PANEL_DELAY_TIME + P02_OFF2ON_DEBOUNCE_TIME ));
		    }
        }
/*
        else
        {
            if (P01_set_BL_cmd == 1)
		    {
                printk(DBGMSK_BL_G2"++++++++++[BACKLIGHT_SCALAR] turn on BL and PANEL ++++++++++\n");
                mutex_lock(&scaler_mutex_lock);
                printk(DBGMSK_BL_G2"++++++++++[BACKLIGHT_SCALAR] lock_enter: turn on BL++++++++++\n");
                switch_backlight_and_panel(1);
                P01_last_blctrl_time = jiffies;
                p01_backlight_state = 1;
                p01_panel_state = 1;
                msleep(300);
                mutex_unlock(&scaler_mutex_lock);
                init_fm34_when_bl_on();     //Bruno++
                mutex_lock(&scaler_mutex_lock);
                printk(DBGMSK_BL_G2"++++++++++[BACKLIGHT_SCALAR] lock_enter: set brightness %d ++++++++++\n", p01_brightness_value);
                p02_backlight_write(p01_brightness_value);
                mutex_unlock(&scaler_mutex_lock);
            }
            else
            {
                mutex_lock(&scaler_mutex_lock);
                printk(DBGMSK_BL_G2"++++++++++[BACKLIGHT_SCALAR] lock_enter: turn off BL++++++++++\n");
                switch_backlight_and_panel(0);
                P01_last_blctrl_time = jiffies;
                p01_backlight_state = 0;
                p01_panel_state = 0;
                mutex_unlock(&scaler_mutex_lock);
		    }
		}
*/
        //P01_last_blctrl_time = jiffies;
    }
    else
        if (P01_set_BL_cmd == 1)
            printk("--------- [BACKLIGHT_SCALAR] try to turn on BL, but not in Pad ----------\n");
        else
            printk("--------- [BACKLIGHT_SCALAR] try to turn off BL, but not in Pad ----------\n");
}

static void set_p01_panel_off_func(struct work_struct *work)
{
    if (hdmi_exist() == true){
        printk("++++++++++[BACKLIGHT_SCALAR] turn off BL and PANEL ++++++++++\n");
        mutex_lock(&scaler_mutex_lock);
        printk(DBGMSK_BL_G2"++++++++++[BACKLIGHT_SCALAR] lock_enter: turn off BL++++++++++\n");
        switch_backlight_and_panel(0);
        p01_panel_state = 0;
        P01_last_blctrl_time = jiffies;
        mutex_unlock(&scaler_mutex_lock);
    }
    else
        printk("----------[BACKLIGHT_SCALAR] try to turn off BL and PANEL, but not in Pad ----------\n");
}

static void disable_pad_wait_icon_func(struct work_struct *work)
{
    if (hdmi_exist() == true){
        if (p01_backlight_state == 0)
        {
            mutex_lock(&scaler_mutex_lock);
            printk("++++++++++[SCALAR] disable waiting icon ++++++++++\n");
            scaler_set_waiting_icon(0);
            msleep(100);
            mutex_unlock(&scaler_mutex_lock);
        }
        else
        {
            printk("++++++++++[SCALAR] BL is on, so skip disable waiting icon ++++++++++\n");
        }
    }
    else 
        printk("---------[SCALAR] try to disable waiting icon, but not in Pad ----------\n");
}

void disable_pad_wait_icon(void)
{
    if (delayed_work_pending(&disable_pad_wait_icon_work))
    {
        cancel_delayed_work_sync(&disable_pad_wait_icon_work);
    }
    queue_delayed_work(backlight_workqueue, &disable_pad_wait_icon_work, msecs_to_jiffies(P02_SCALER_INIT_NEED_TIME));
}

/*
static void set_backlight_on(struct work_struct *work)
{
    int bl_status;
    if (hdmi_exist() == true){
        //blVoteScalarSleepEntry(0);
        switch_backlight(1);
        printk(DBGMSK_BL_G1"++++++++++[BACKLIGHT_SCALAR] vote to wakeup scalar++++++++++\n");
        backlight_state = 1;

        msleep(400);
        bl_status = get_backlight();
        if (bl_status < 0)
            printk("++++++++++++++++++++++++++++++++++[BACKLIGHT_SCALAR] get BL I2C error\n");
        else
            printk("++++++++++++++++++++++++++++++++++[BACKLIGHT_SCALAR] BL is %s (%d)\n", bl_status ? "ON" : "OFF", bl_status);

    }
}

static void set_backlight_off(struct work_struct *work)
{
    if (hdmi_exist() == true){
        switch_backlight(0);
        //blVoteScalarSleepEntry(1);
        printk(DBGMSK_BL_G1"----------[BACKLIGHT_SCALAR] vote to suspend scalar----------\n");
        backlight_state = 0;
    }
}
*/
static int bl_pad_add_count = 0;
static int bl_pad_remove_count = 0;
void bl_pad_add_func(struct work_struct *work)
{
    printk("----------Add p02 #%d----------\n",bl_pad_add_count++);

    msleep(1000); //wait for scaler AC_ON initial

    read_scaler_version(&g_scaler_ver);
    if (g_scaler_ver == 0) //try again
    {
		msleep(300);
        read_scaler_version(&g_scaler_ver);
	}
    printk("++++++++++[SCALER VER] 0x%02x ++++++++++\n", g_scaler_ver);

    if ( (( g_scaler_ver >= 0x20) && (g_scaler_ver < 0xe0)) || (g_scaler_ver >= 0xe2))
        g_new_pad_bl_control_method = 1;
    else
        g_new_pad_bl_control_method = 0;

    if ( g_scaler_ver < 0x20 )
        g_more_delay_after_resume_for_old_scaler = 300;  //delay more 300ms after resume for scaler ver 0x1d
    else
        g_more_delay_after_resume_for_old_scaler = 0;

    p01_backlight_state = 0; //treat P01 as default backlight off
    p01_panel_state = 0;
    P01_last_blctrl_time = 0;
    P01_last_brctrl_time = 0;
    if (p01_brightness_value == 0)
        p01_brightness_value = 50;

    backlight_mode_state = p01;
    //a60K_set_backlight(0);
    p01_set_backlight();
    //queue_delayed_work(brightness_workqueue, &set_brightness_work, msecs_to_jiffies(500));

//    msleep(1000);
//    p01_backlight_state = 0; //reset state and turn on BL twice in case first time fail
//    p01_set_backlight();

    //queue_delayed_work(scaler_version_workqueue, &g_write_scaler_work, (HZ*5000/1000));
}

#define A66_SET_BRIGHTNESS_RETRY_COUNT  10
static void bl_pad_remove_func(struct work_struct *work)
{
	int ret;
	int i;
    printk("----------Remove p02 #%d----------\n",bl_pad_remove_count++);
    backlight_mode_state = a60k;
    for (i=0; i<A66_SET_BRIGHTNESS_RETRY_COUNT; i++)
    {
        ret=0; //a60K_set_backlight(a60k_backlight_value);
        if (ret ==0)
	        break;
        //retry again
	    msleep(200);
	}
    i2c_error = false;
}

int query_pad_backlight_state(void)
{
	return p01_backlight_state;
}

#if 0
static struct led_classdev a60k_backlight_led = {
    .name       = "lcd-backlight",
    .brightness = MAX_BACKLIGHT_BRIGHTNESS,
    .brightness_set = set_bl_brightness,
};
#endif



static int a60K_backlight_probe(struct platform_device *pdev)
{
    scaler_version_workqueue = create_singlethread_workqueue("scaler_version_wq");
    INIT_DELAYED_WORK(&g_write_scaler_work, P02_write_scaler_work);
    backlight_pdata = pdev->dev.platform_data;
    
    //bl_lpm = pwm_request(backlight_pdata->gpio[0],
    //   "backlight");

    P01_Add_Remove_workqueue  = create_singlethread_workqueue("P02ADDWORKQUEUE");
    INIT_WORK(&add_func_work, bl_pad_add_func);
    INIT_WORK(&remove_func_work, bl_pad_remove_func);

    backlight_workqueue  = create_singlethread_workqueue("P02BACKLIGHTWORKQUEUE");
    //INIT_WORK(&switch_backlight_on_work, set_backlight_on);
    //INIT_WORK(&switch_backlight_off_work, set_backlight_off);
    INIT_DELAYED_WORK(&switch_backlight_work, set_p01_backlight_func);
    INIT_DELAYED_WORK(&turn_off_panel_work, set_p01_panel_off_func);
    INIT_DELAYED_WORK(&disable_pad_wait_icon_work, disable_pad_wait_icon_func);

    brightness_workqueue  = create_singlethread_workqueue("P02BRIGHTNESSWORKQUEUE");
    INIT_DELAYED_WORK(&set_brightness_work, set_p01_brightness_work);

    // initialize wake_lock
    wake_lock_init(&turn_off_pad_panel_wake_lock, WAKE_LOCK_SUSPEND, "turn_off_pad_panel");

    // init mutex
    mutex_init(&scaler_mutex_lock);
#if 0
    if (led_classdev_register(&pdev->dev, &a60k_backlight_led))
    {
        printk(KERN_ERR "led_classdev_register failed\n");            
    }
    else
#endif
    {
        a60k_backlight_registered = 1;
        a60K_set_backlight(MAX_BACKLIGHT_BRIGHTNESS);
    }
    return 0;
}

static int a60K_backlight_remove(struct platform_device *pdev)
{
    if (a60k_backlight_registered) {
        a60k_backlight_registered = 0;
#if 0
        led_classdev_unregister(&a60k_backlight_led);
#endif
    }
    return 0;
}

static int change_backlight_mode(struct notifier_block *this, unsigned long event, void *ptr)
{
        switch (event) {
        case P01_ADD:
                queue_work(P01_Add_Remove_workqueue, &add_func_work);
                return NOTIFY_DONE;
        case P01_REMOVE:
                queue_work(P01_Add_Remove_workqueue, &remove_func_work);
                return NOTIFY_DONE;
        default:
                return NOTIFY_DONE;
        }
}
 
static struct notifier_block my_hs_notifier = {
        .notifier_call = change_backlight_mode,
        .priority = VIBRATOR_MP_NOTIFY,
};

static struct platform_driver a60K_backlight_driver = {
	.probe		= a60K_backlight_probe,
	.remove     = a60K_backlight_remove,
	.driver		= {
		.name	= "a60k_backlight",
		.owner	= THIS_MODULE,
        	},
};

static int __init msm_pmic_led_init(void)
{
    register_microp_notifier(&my_hs_notifier);

    return platform_driver_register(&a60K_backlight_driver);
}
module_init(msm_pmic_led_init);

static void __exit msm_pmic_led_exit(void)
{
	platform_driver_unregister(&a60K_backlight_driver);
    destroy_workqueue(scaler_version_workqueue);
}
module_exit(msm_pmic_led_exit);

MODULE_DESCRIPTION("A60K BACKLIGHT DRIVER");

