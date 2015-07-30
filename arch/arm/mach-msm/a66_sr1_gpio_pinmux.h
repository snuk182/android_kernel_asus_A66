#ifndef __A66_SR1_1_GPIO_PINMUX_H
#define __A66_SR1_1_GPIO_PINMUX_H

#include "a60k_gpio_pinmux_setting.h"

static struct msm_gpiomux_config a66_sr1_msm8960_gpio_configs[] = {
        {
                .gpio = 0,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &mdp_vsync_active_cfg,//Mickey
                        [GPIOMUX_SUSPENDED] = &mdp_vsync_suspend_cfg,//Mickey
                },
        },
//ASUS_BSP +++ Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
        {
		.gpio = 1,	//A60K: VGA_CMOS_MCLK_EN -> VGA_MCLK_EN_N
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[9],
			[GPIOMUX_SUSPENDED] = &cam_settings[9],
		},
	},
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
	{
		.gpio = 2,
		.settings = {
                        [GPIOMUX_ACTIVE] = &spk_amp_en,
                        [GPIOMUX_SUSPENDED] = &spk_amp_dis,
		},
	},
        {       //N.C. Ledger 11.9.27
                .gpio = 3,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
    	{
		.gpio = 4,
		.settings = {
		    [GPIOMUX_ACTIVE]    = &I_NP_cfg,
		    [GPIOMUX_SUSPENDED] = &I_NP_cfg,
		},
	},
//ASUS_BSP +++ Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
	{
		.gpio = 5,	//A60K: FUNC1: CAM_MCLK0
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[0],
		},
	},
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
	{
		.gpio = 6,
		.settings = {
			[GPIOMUX_SUSPENDED] = &NC_cfg,
		},
	},
    	{
		.gpio = 7,
		.settings = {
		    [GPIOMUX_ACTIVE]    = &hdmi_active_3_cfg,
		    [GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
	{
		.gpio = 8,	//A60K: ISP_PWR_EN: 1.2v on/off -> ISP_1P2_EN
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[9],
			[GPIOMUX_SUSPENDED] = &cam_settings[9],
		},
	},
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
	{
		.gpio = 9,
		.settings = {
			[GPIOMUX_SUSPENDED] = &NC_cfg,
		},
	},
	{
		.gpio = 10,	//A60K: CAM_8M_RST_N
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[9],
			[GPIOMUX_SUSPENDED] = &cam_settings[9],
		},
	},
//joe1_++
	{
		.gpio = 11, //TS INTERRUPT: A60K_SR1=TP_IRQ_N
		.settings = {
			[GPIOMUX_ACTIVE]    = &ts_int_cfg,
			[GPIOMUX_SUSPENDED] = &ts_int_cfg,
		},
	},
//joe1_--
        {
	        .gpio = 12,
	        .settings = {
                        [GPIOMUX_SUSPENDED] = &NC_cfg,
	        },
        },

        {
	        .gpio = 13,
	        .settings = {
                        [GPIOMUX_ACTIVE]    = &cam_settings[2],
                        [GPIOMUX_SUSPENDED] = &cam_settings[0],
	        },
        },
        {
	        .gpio = 14,
	        .settings = {
                    [GPIOMUX_ACTIVE]    = &hdmi_active_3_cfg,
                    [GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
	        },
        },
        {
	        .gpio = 15,
	        .settings = {
	            [GPIOMUX_ACTIVE]    = &gpio_power_keys_cfg,
	        },
        },
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
                        [GPIOMUX_ACTIVE] = &gpio_out,
                        [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {
                .gpio = 19,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
//ASUS_BSP +++ Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
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
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
	{
		.gpio      = 22,	/* GSBI5 UART2 */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gsbi5,
		},
	},
	{
		.gpio      = 23,	/* GSBI5 UART2 */
		.settings = {
                        [GPIOMUX_ACTIVE]    = &gsbi5,   //Ledger
			[GPIOMUX_SUSPENDED] = &gsbi5,
		},
	},
//ASUS_BSP +++ Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
	{
		.gpio = 24,	//A60K: CAM_RST_ISP -> ISP_RST_N
		.settings = {
                        [GPIOMUX_ACTIVE]    = &cam_settings[9],
                        [GPIOMUX_SUSPENDED] = &cam_settings[9],
		},
	},
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
        {
                .gpio = 25,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &I_NP_cfg,
                        [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 32,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {
                .gpio = 33,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {
                .gpio = 34,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
//ASUS_BSP +++ Josh_Liao "add asus battery driver"        
        {
                .gpio = 35,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &a60k_bat_low_cfg,
                        [GPIOMUX_SUSPENDED] = &a60k_bat_low_cfg,
                },
        },
//ASUS_BSP --- Josh_Liao "add asus battery driver"
        {       //N.C. Ledger 11.9.27
                .gpio = 36,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 37,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
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
        {       //N.C. Ledger 11.9.27
                .gpio = 40,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 41,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 42,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 43,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
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
                        [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {
                .gpio = 47,
                .settings = {
                        [GPIOMUX_ACTIVE] = &hs_path_en,
                        [GPIOMUX_SUSPENDED] = &hs_path_en,
                },
        },
//ASUS_BSP +++ Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
        {
                .gpio = 48,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &cam_settings[10],
                        [GPIOMUX_SUSPENDED] = &cam_settings[10],
                },
        },
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
    //[psensor]Add cm3623 support
        {
                .gpio = 49,
                .settings = {
                        [GPIOMUX_ACTIVE] = &cm3623_interrupt,
                        [GPIOMUX_SUSPENDED] = &cm3623_interrupt,
                },
        },
        {
                .gpio = 50,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {
                .gpio = 51,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &O_L_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 52,
                .settings = {
                    [GPIOMUX_ACTIVE] = &hs_button_detect,
                    [GPIOMUX_SUSPENDED] = &hs_button_detect,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 53,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
//joe1_--
        {
                .gpio = 54,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &hs_detect,
                },
        },
        {
                .gpio = 55,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
//ASUS_BSP +++ Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
        {
			.gpio = 56,	//A60K: CAM 1.8V EN (for OV8830 and VGA_CMOS) -> ISP_1P8_EN
                .settings = {
                        [GPIOMUX_ACTIVE]    = &cam_settings[9],
                        [GPIOMUX_SUSPENDED] = &cam_settings[9],
                },
        },
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
        {
                .gpio = 57,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {
                .gpio = 58,
                .settings = {
                    [GPIOMUX_ACTIVE]    = &gpio_voldown_keys_cfg,
                    [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
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
                .gpio = 62,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {
                .gpio = 63,
                .settings = {
                    [GPIOMUX_ACTIVE]    = &hdmi_active_3_cfg,
                    [GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 64,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 65,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 66,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {
                .gpio = 67,
                .settings = {
                    [GPIOMUX_ACTIVE]    = &gpio_volup_keys_cfg,             
                    [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {
                .gpio = 68,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &O_L_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 69,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 70,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {
                .gpio = 71,
                .settings = {
                        [GPIOMUX_ACTIVE] = &mic2_bias_en,
                },
        },
//ASUS_BSP +++ Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
        {
						.gpio = 72, //A60K: VGA Camera standby
                .settings = {
                        [GPIOMUX_ACTIVE]    = &cam_settings[10],
                        [GPIOMUX_SUSPENDED] = &cam_settings[10],
                },
        },
//ASUS_BSP +++ Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
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
        {       //N.C. Ledger 11.9.27
                .gpio = 75,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {
                .gpio = 76,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {
                .gpio = 77,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 78,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 79,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 80,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 81,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 82,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
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
        {
                .gpio = 89,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &rf_switch_cfg,
                        [GPIOMUX_SUSPENDED] = &rf_switch_cfg,
                },
        },
        {
                .gpio = 90,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &rf_switch_cfg,
                        [GPIOMUX_SUSPENDED] = &rf_switch_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 91,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {
                .gpio = 92,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 93,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {
                .gpio = 94,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {
                .gpio = 95,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {
                .gpio = 96,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &I_NP_cfg,
                },
        },
        {
                .gpio = 97,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {
                .gpio = 98,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
//joe1_++
	{
		.gpio = 99, //TS PAD INTERRUPT: A60K_SR1=HDMI_UTILITY_INT
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
        {       //N.C. Ledger 11.9.27
                .gpio = 106,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
 // sina: microp intr pin
     	{
		.gpio = 107,
		.settings = {
			[GPIOMUX_ACTIVE]    = &microp_intr_cfg,
			[GPIOMUX_SUSPENDED] = &microp_intr_cfg,
		},
	},
        {       //N.C. Ledger 11.9.27
                .gpio = 109,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 110,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 112,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 113,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 114,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 115,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 120,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 121,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 124,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 127,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },		
        {
                .gpio = 128,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {
                .gpio = 129,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 133,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 134,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 138,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 139,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 140,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 141,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {       //N.C. Ledger 11.9.27
                .gpio = 149,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {
                .gpio = 150,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },
        {
                .gpio = 151,
                .settings = {
                    [GPIOMUX_SUSPENDED] = &NC_cfg,
                },
        },

};

#endif  /* __A60K_SR1_1_GPIO_PINMUX_H  */
