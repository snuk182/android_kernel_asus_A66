#ifndef __A60K_EVB_GPIO_PINMUX_H
#define __A60K_EVB_GPIO_PINMUX_H

#include "a60k_gpio_pinmux_setting.h"

static struct msm_gpiomux_config a60k_evb_msm8960_gpio_configs[] = {
    {
	    .gpio = 0,
	    .settings = {
	        [GPIOMUX_ACTIVE] = &mic2_bias_en,
	    },
    },
    {
		.gpio = 1,	//A60K: VGA_CMOS_MCLK_EN
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 2,	//A60K: FLASH_ENT, No need on A60K
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
       {
		.gpio = 3,
		.settings = {
		    [GPIOMUX_ACTIVE]    = &hdmi_active_3_cfg,
		    [GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
    	},	
    	{
		.gpio = 4,
		.settings = {
		    [GPIOMUX_ACTIVE]    = &hdmi_active_3_cfg,
		    [GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},	
	{
		.gpio = 5,	//A60K: FUNC1: MCLK
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[1],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 6,	//A60K: CAM_RST_ISP
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
    	{
		.gpio = 7,
		.settings = {
		    [GPIOMUX_ACTIVE]    = &hdmi_active_3_cfg,
		    [GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},	
	{
		.gpio = 8,	//A60K: ISP_PWR_EN: 1.2v on/off
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
	{
		.gpio = 9,	//A60K: CAM 1.8V EN (for OV8830 and VGA_CMOS)
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
//joe1_++
	{
		.gpio = 11, //TS INTERRUPT: A60K_EVB=CHG#_IRQ
		.settings = {
			[GPIOMUX_ACTIVE]    = &ts_int_cfg,
			[GPIOMUX_SUSPENDED] = &ts_int_cfg,
		},
	},
//joe1_--
      {
	        .gpio = 12,
	        .settings = {
	            [GPIOMUX_ACTIVE] = &hs_button_detect,
	        },
      },
	
//ASUS BSP TIM-2011.08.12++
      {
	        .gpio = 13,
	        .settings = {
	            [GPIOMUX_ACTIVE]    = &gpio_volup_keys_cfg,	            
	        },
      },
      {
	        .gpio = 14,
	        .settings = {
	            [GPIOMUX_ACTIVE]    = &gpio_voldown_keys_cfg,	            
	        },
      },
      {
	        .gpio = 15,
	        .settings = {
	            [GPIOMUX_ACTIVE]    = &gpio_power_keys_cfg,
	        },
      },
//ASUS BSP TIM-2011.08.12--		
	{
		.gpio      = 16,	/* GSBI3 I2C QUP SDA */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi3,
		},
	},
	{
		.gpio      = 17,	/* GSBI3 I2C QUP SCL */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi3,
		},
	},
    {
	    .gpio = 18,
	    .settings = {
	        [GPIOMUX_ACTIVE] = &hs_path_en,
	    },
    },
	{
		.gpio      = 20,	/* GSBI4 I2C QUP SDA */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[8],
		},
	},
	{
		.gpio      = 21,	/* GSBI4 I2C QUP SCL */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[8],
		},
	},
	{
		.gpio      = 22,	/* GSBI5 UART2 */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi5,
		},
	},
	{
		.gpio      = 23,	/* GSBI5 UART2 */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi5,
		},
	},
//ASUS_BSP +++ Josh_Liao "add asus battery driver"
	{
		.gpio      = 24,	/* GSBI5 UART2 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &a60k_bat_low_cfg,
			[GPIOMUX_SUSPENDED] = &a60k_bat_low_cfg,
		},
	},
//ASUS_BSP --- Josh_Liao "add asus battery driver"
//ASUS_BSP+++ CR_0000 Randy_Change@asus.com.tw [2011/9/14] Modify Begin
	{
		.gpio = 25,	//A60K: VGA Camera standby
		.settings = {
			[GPIOMUX_ACTIVE]		= &cam_settings[2],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
//ASUS_BSP--- CR_0000 Randy_Change@asus.com.tw [2011/9/14] Modify End
    //[psensor]Add cm3623 support
    {
        .gpio = 38,
        .settings = {
            [GPIOMUX_ACTIVE] = &cm3623_pwr_en,
            [GPIOMUX_SUSPENDED] = &cm3623_pwr_en,
        },
    },
	{
		.gpio = 39,
		.settings = {
			[GPIOMUX_ACTIVE]    = &a60k_hall_sensor_cfg,
			[GPIOMUX_SUSPENDED] = &a60k_hall_sensor_cfg,
		},
	},
    	
	{
		.gpio      = 44,	/* GSBI12 I2C QUP SDA */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi12,
		},
	},
	{
		.gpio      = 45,	/* GSBI12 I2C QUP SCL */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi12,
		},
	},
      {
	        .gpio = 46,
	        .settings = {
                [GPIOMUX_ACTIVE] = &spk_amp_en,
                [GPIOMUX_SUSPENDED] = &spk_amp_dis,
	        },
      },	
    //[psensor]Add cm3623 support
    {
        .gpio = 49,
        .settings = {
            [GPIOMUX_ACTIVE] = &cm3623_interrupt,
            [GPIOMUX_SUSPENDED] = &cm3623_interrupt,
        },
    },
//joe1_++
	{
		.gpio = 50, //TS RESET : A60K_EVB=TP_RESET#
		.settings = {
			[GPIOMUX_ACTIVE]    = &ts_reset_cfg,
			[GPIOMUX_SUSPENDED] = &ts_reset_cfg,
		},
	},
//joe1_--
      {
		.gpio = 59,
		.settings = {
			[GPIOMUX_SUSPENDED] = &cdc_mclk,
		},
      },      	
	{
		.gpio	= 60,		/* slimbus data */
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
	{
		.gpio	= 61,		/* slimbus clk */
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
	{
		.gpio      = 73,	/* GSBI10 I2C QUP SDA */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi10,
		},
	},
	{
		.gpio      = 74,	/* GSBI10 I2C QUP SCL */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi10,
		},
	},
	{
		.gpio = 84,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 85,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 86,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 87,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 88,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},	
//joe1_++
	{
		.gpio = 99, //TS PAD INTERRUPT: A60K_EVB=HDMI_UTILITY_INT
		.settings = {
			[GPIOMUX_ACTIVE]    = &ts_pad_int_cfg,
			[GPIOMUX_SUSPENDED] = &ts_pad_int_cfg,
		},
	},
//joe1_--
	{
		.gpio = 100,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 101,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 102,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_2_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 107,
		.settings = {
			[GPIOMUX_ACTIVE]    = &microp_intr_cfg,
			[GPIOMUX_SUSPENDED] = &microp_intr_cfg,
        	},
	},

};

#endif  /* __A60K_EVB_GPIO_PINMUX_H  */
