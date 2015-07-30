/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/bootmem.h>
#include <linux/ion.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include <mach/msm_bus_board.h>
#include <mach/msm_memtypes.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <mach/ion.h>
#include <mach/socinfo.h>

#include "devices.h"
#include "board-8960.h"
#include <linux/fb.h> //Mickey
#include <linux/a60k_gpio_pinname.h>    //Mickey

#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_PRIM_BUF_SIZE (1280 * 960 * 4 * 3 + 544*960*4*2) /* 4 bpp x 3 pages */
#else
#define MSM_FB_PRIM_BUF_SIZE (1280 * 960 * 4 * 2) /* 4 bpp x 2 pages */
#endif

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
#define MSM_FB_EXT_BUF_SIZE \
		(roundup((1920 * 1088 * 2), 4096) * 1) /* 2 bpp x 1 page */
#elif defined(CONFIG_FB_MSM_TVOUT)
#define MSM_FB_EXT_BUF_SIZE \
		(roundup((720 * 576 * 2), 4096) * 2) /* 2 bpp x 2 pages */
#else
#define MSM_FB_EXT_BUF_SIZE	0
#endif

/* Note: must be multiple of 4096 */
#define MSM_FB_SIZE roundup(MSM_FB_PRIM_BUF_SIZE + MSM_FB_EXT_BUF_SIZE, 4096)

#ifdef CONFIG_FB_MSM_OVERLAY0_WRITEBACK
#define MSM_FB_OVERLAY0_WRITEBACK_SIZE \
		roundup((roundup(1920, 32) * roundup(1200, 32) * 3 * 2), 4096)
#else
#define MSM_FB_OVERLAY0_WRITEBACK_SIZE (0)
#endif  /* CONFIG_FB_MSM_OVERLAY0_WRITEBACK */

#ifdef CONFIG_FB_MSM_OVERLAY1_WRITEBACK
#define MSM_FB_OVERLAY1_WRITEBACK_SIZE \
		roundup((roundup(1920, 32) * roundup(1080, 32) * 3 * 2), 4096)
#else
#define MSM_FB_OVERLAY1_WRITEBACK_SIZE (0)
#endif  /* CONFIG_FB_MSM_OVERLAY1_WRITEBACK */

#define MDP_VSYNC_GPIO 0

#define MIPI_CMD_NOVATEK_QHD_PANEL_NAME	"mipi_cmd_novatek_qhd"
#define MIPI_VIDEO_NOVATEK_QHD_PANEL_NAME	"mipi_video_novatek_qhd"
#define MIPI_VIDEO_TOSHIBA_WSVGA_PANEL_NAME	"mipi_video_toshiba_wsvga"
#define MIPI_VIDEO_TOSHIBA_WUXGA_PANEL_NAME	"mipi_video_toshiba_wuxga"
#define MIPI_VIDEO_CHIMEI_WXGA_PANEL_NAME	"mipi_video_chimei_wxga"
#define MIPI_VIDEO_CHIMEI_WUXGA_PANEL_NAME	"mipi_video_chimei_wuxga"
#define MIPI_VIDEO_SIMULATOR_VGA_PANEL_NAME	"mipi_video_simulator_vga"
#define MIPI_CMD_RENESAS_FWVGA_PANEL_NAME	"mipi_cmd_renesas_fwvga"
#define MIPI_VIDEO_ORISE_720P_PANEL_NAME	"mipi_video_orise_720p"
#define MIPI_CMD_ORISE_720P_PANEL_NAME		"mipi_cmd_orise_720p"
#define HDMI_PANEL_NAME	"hdmi_msm"
#define TVOUT_PANEL_NAME	"tvout_msm"

#ifdef CONFIG_FB_MSM_HDMI_AS_PRIMARY
static unsigned char hdmi_is_primary = 1;
#else
static unsigned char hdmi_is_primary;
#endif

unsigned char msm8960_hdmi_as_primary_selected(void)
{
	return hdmi_is_primary;
}

static struct resource msm_fb_resources[] = {
	{
		.flags = IORESOURCE_DMA,
	}
};

#ifndef CONFIG_FB_MSM_MIPI_PANEL_DETECT
static void set_mdp_clocks_for_wuxga(void);
#endif

static int msm_fb_detect_panel(const char *name)
{
	if (machine_is_msm8960_liquid()) {
		u32 ver = socinfo_get_platform_version();
		if (SOCINFO_VERSION_MAJOR(ver) == 3) {
			if (!strncmp(name, MIPI_VIDEO_CHIMEI_WUXGA_PANEL_NAME,
				     strnlen(MIPI_VIDEO_CHIMEI_WUXGA_PANEL_NAME,
						PANEL_NAME_MAX_LEN))) {
				set_mdp_clocks_for_wuxga();
				return 0;
			}
		} else {
			if (!strncmp(name, MIPI_VIDEO_CHIMEI_WXGA_PANEL_NAME,
				     strnlen(MIPI_VIDEO_CHIMEI_WXGA_PANEL_NAME,
						PANEL_NAME_MAX_LEN)))
				return 0;
		}
	} else {
		if (!strncmp(name, MIPI_VIDEO_TOSHIBA_WSVGA_PANEL_NAME,
				strnlen(MIPI_VIDEO_TOSHIBA_WSVGA_PANEL_NAME,
					PANEL_NAME_MAX_LEN)))
			return 0;

#if !defined(CONFIG_FB_MSM_LVDS_MIPI_PANEL_DETECT) && \
	!defined(CONFIG_FB_MSM_MIPI_PANEL_DETECT)
		if (!strncmp(name, MIPI_VIDEO_NOVATEK_QHD_PANEL_NAME,
				strnlen(MIPI_VIDEO_NOVATEK_QHD_PANEL_NAME,
					PANEL_NAME_MAX_LEN)))
			return 0;

		if (!strncmp(name, MIPI_CMD_NOVATEK_QHD_PANEL_NAME,
				strnlen(MIPI_CMD_NOVATEK_QHD_PANEL_NAME,
					PANEL_NAME_MAX_LEN)))
			return 0;

		if (!strncmp(name, MIPI_VIDEO_SIMULATOR_VGA_PANEL_NAME,
				strnlen(MIPI_VIDEO_SIMULATOR_VGA_PANEL_NAME,
					PANEL_NAME_MAX_LEN)))
			return 0;

		if (!strncmp(name, MIPI_CMD_RENESAS_FWVGA_PANEL_NAME,
				strnlen(MIPI_CMD_RENESAS_FWVGA_PANEL_NAME,
					PANEL_NAME_MAX_LEN)))
			return 0;

		if (!strncmp(name, MIPI_VIDEO_TOSHIBA_WUXGA_PANEL_NAME,
				strnlen(MIPI_VIDEO_TOSHIBA_WUXGA_PANEL_NAME,
					PANEL_NAME_MAX_LEN))) {
			set_mdp_clocks_for_wuxga();
			return 0;
		}

		if (!strncmp(name, MIPI_VIDEO_ORISE_720P_PANEL_NAME,
				strnlen(MIPI_VIDEO_ORISE_720P_PANEL_NAME,
					PANEL_NAME_MAX_LEN)))
			return 0;

		if (!strncmp(name, MIPI_CMD_ORISE_720P_PANEL_NAME,
				strnlen(MIPI_CMD_ORISE_720P_PANEL_NAME,
					PANEL_NAME_MAX_LEN)))
			return 0;
#endif
	}

	if (!strncmp(name, HDMI_PANEL_NAME,
			strnlen(HDMI_PANEL_NAME,
				PANEL_NAME_MAX_LEN))) {
		if (hdmi_is_primary)
			set_mdp_clocks_for_wuxga();
		return 0;
	}

	if (!strncmp(name, TVOUT_PANEL_NAME,
			strnlen(TVOUT_PANEL_NAME,
				PANEL_NAME_MAX_LEN)))
		return 0;

	pr_warning("%s: not supported '%s'", __func__, name);
	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources     = ARRAY_SIZE(msm_fb_resources),
	.resource          = msm_fb_resources,
	.dev.platform_data = &msm_fb_pdata,
};
static bool dsi_power_on;
#if 0   //QCT default// +++ ASUS_BSP : miniporting

static void mipi_dsi_panel_pwm_cfg(void)
{
	int rc;
	static int mipi_dsi_panel_gpio_configured;
	static struct pm_gpio pwm_enable = {
		.direction        = PM_GPIO_DIR_OUT,
		.output_buffer    = PM_GPIO_OUT_BUF_CMOS,
		.output_value     = 1,
		.pull             = PM_GPIO_PULL_NO,
		.vin_sel          = PM_GPIO_VIN_VPH,
		.out_strength     = PM_GPIO_STRENGTH_HIGH,
		.function         = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol      = 0,
		.disable_pin      = 0,
	};
	static struct pm_gpio pwm_mode = {
		.direction        = PM_GPIO_DIR_OUT,
		.output_buffer    = PM_GPIO_OUT_BUF_CMOS,
		.output_value     = 0,
		.pull             = PM_GPIO_PULL_NO,
		.vin_sel          = PM_GPIO_VIN_S4,
		.out_strength     = PM_GPIO_STRENGTH_HIGH,
		.function         = PM_GPIO_FUNC_2,
		.inv_int_pol      = 0,
		.disable_pin      = 0,
	};

	if (mipi_dsi_panel_gpio_configured == 0) {
		/* pm8xxx: gpio-21, Backlight Enable */
		rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(21),
					&pwm_enable);
		if (rc != 0)
			pr_err("%s: pwm_enabled failed\n", __func__);

		/* pm8xxx: gpio-24, Bl: Off, PWM mode */
		rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(24),
					&pwm_mode);
		if (rc != 0)
			pr_err("%s: pwm_mode failed\n", __func__);

		mipi_dsi_panel_gpio_configured++;
	}
}

/**
 * LiQUID panel on/off
 *
 * @param on
 *
 * @return int
 */
static int mipi_dsi_liquid_panel_power(int on)
{
	static struct regulator *reg_l2, *reg_ext_3p3v;
	static int gpio21, gpio24, gpio43;
	int rc;

	mipi_dsi_panel_pwm_cfg();
	pr_debug("%s: on=%d\n", __func__, on);

	gpio21 = PM8921_GPIO_PM_TO_SYS(21); /* disp power enable_n */
	gpio43 = PM8921_GPIO_PM_TO_SYS(43); /* Displays Enable (rst_n)*/
	gpio24 = PM8921_GPIO_PM_TO_SYS(24); /* Backlight PWM */

	if (!dsi_power_on) {

		reg_l2 = regulator_get(&msm_mipi_dsi1_device.dev,
				"dsi_vdda");
		if (IS_ERR(reg_l2)) {
			pr_err("could not get 8921_l2, rc = %ld\n",
				PTR_ERR(reg_l2));
			return -ENODEV;
		}

		rc = regulator_set_voltage(reg_l2, 1200000, 1200000);
		if (rc) {
			pr_err("set_voltage l2 failed, rc=%d\n", rc);
			return -EINVAL;
		}

		reg_ext_3p3v = regulator_get(&msm_mipi_dsi1_device.dev,
			"vdd_lvds_3p3v");
		if (IS_ERR(reg_ext_3p3v)) {
			pr_err("could not get reg_ext_3p3v, rc = %ld\n",
			       PTR_ERR(reg_ext_3p3v));
		    return -ENODEV;
		}

		rc = gpio_request(gpio21, "disp_pwr_en_n");
		if (rc) {
			pr_err("request gpio 21 failed, rc=%d\n", rc);
			return -ENODEV;
		}

		rc = gpio_request(gpio43, "disp_rst_n");
		if (rc) {
			pr_err("request gpio 43 failed, rc=%d\n", rc);
			return -ENODEV;
		}

		rc = gpio_request(gpio24, "disp_backlight_pwm");
		if (rc) {
			pr_err("request gpio 24 failed, rc=%d\n", rc);
			return -ENODEV;
		}

		dsi_power_on = true;
	}

	if (on) {
		rc = regulator_set_optimum_mode(reg_l2, 100000);
		if (rc < 0) {
			pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		rc = regulator_enable(reg_l2);
		if (rc) {
			pr_err("enable l2 failed, rc=%d\n", rc);
			return -ENODEV;
		}

		rc = regulator_enable(reg_ext_3p3v);
		if (rc) {
			pr_err("enable reg_ext_3p3v failed, rc=%d\n", rc);
			return -ENODEV;
		}

		/* set reset pin before power enable */
		gpio_set_value_cansleep(gpio43, 0); /* disp disable (resx=0) */

		gpio_set_value_cansleep(gpio21, 0); /* disp power enable_n */
		msleep(20);
		gpio_set_value_cansleep(gpio43, 1); /* disp enable */
		msleep(20);
		gpio_set_value_cansleep(gpio43, 0); /* disp enable */
		msleep(20);
		gpio_set_value_cansleep(gpio43, 1); /* disp enable */
		msleep(20);
	} else {
		gpio_set_value_cansleep(gpio43, 0);
		gpio_set_value_cansleep(gpio21, 1);

		rc = regulator_disable(reg_l2);
		if (rc) {
			pr_err("disable reg_l2 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_disable(reg_ext_3p3v);
		if (rc) {
			pr_err("disable reg_ext_3p3v failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_set_optimum_mode(reg_l2, 100);
		if (rc < 0) {
			pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
			return -EINVAL;
		}
	}

	return 0;
}

static int mipi_dsi_cdp_panel_power(int on)
{
	static struct regulator *reg_l8, *reg_l23, *reg_l2;
	static int gpio43;
	int rc;

	pr_debug("%s: state : %d\n", __func__, on);

	if (!dsi_power_on) {

		reg_l8 = regulator_get(&msm_mipi_dsi1_device.dev,
				"dsi_vdc");
		if (IS_ERR(reg_l8)) {
			pr_err("could not get 8921_l8, rc = %ld\n",
				PTR_ERR(reg_l8));
			return -ENODEV;
		}
		reg_l23 = regulator_get(&msm_mipi_dsi1_device.dev,
				"dsi_vddio");
		if (IS_ERR(reg_l23)) {
			pr_err("could not get 8921_l23, rc = %ld\n",
				PTR_ERR(reg_l23));
			return -ENODEV;
		}
		reg_l2 = regulator_get(&msm_mipi_dsi1_device.dev,
				"dsi_vdda");
		if (IS_ERR(reg_l2)) {
			pr_err("could not get 8921_l2, rc = %ld\n",
				PTR_ERR(reg_l2));
			return -ENODEV;
		}
		rc = regulator_set_voltage(reg_l8, 2800000, 3000000);
		if (rc) {
			pr_err("set_voltage l8 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		rc = regulator_set_voltage(reg_l23, 1800000, 1800000);
		if (rc) {
			pr_err("set_voltage l23 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		rc = regulator_set_voltage(reg_l2, 1200000, 1200000);
		if (rc) {
			pr_err("set_voltage l2 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		gpio43 = PM8921_GPIO_PM_TO_SYS(43);
		rc = gpio_request(gpio43, "disp_rst_n");
		if (rc) {
			pr_err("request gpio 43 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		dsi_power_on = true;
	}
	if (on) {
		rc = regulator_set_optimum_mode(reg_l8, 100000);
		if (rc < 0) {
			pr_err("set_optimum_mode l8 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		rc = regulator_set_optimum_mode(reg_l23, 100000);
		if (rc < 0) {
			pr_err("set_optimum_mode l23 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		rc = regulator_set_optimum_mode(reg_l2, 100000);
		if (rc < 0) {
			pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		rc = regulator_enable(reg_l8);
		if (rc) {
			pr_err("enable l8 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_enable(reg_l23);
		if (rc) {
			pr_err("enable l8 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_enable(reg_l2);
		if (rc) {
			pr_err("enable l2 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		gpio_set_value_cansleep(gpio43, 1);
	} else {
		rc = regulator_disable(reg_l2);
		if (rc) {
			pr_err("disable reg_l2 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_disable(reg_l8);
		if (rc) {
			pr_err("disable reg_l8 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_disable(reg_l23);
		if (rc) {
			pr_err("disable reg_l23 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_set_optimum_mode(reg_l8, 100);
		if (rc < 0) {
			pr_err("set_optimum_mode l8 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		rc = regulator_set_optimum_mode(reg_l23, 100);
		if (rc < 0) {
			pr_err("set_optimum_mode l23 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		rc = regulator_set_optimum_mode(reg_l2, 100);
		if (rc < 0) {
			pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		gpio_set_value_cansleep(gpio43, 0);
	}
	return 0;
}
#else
static int a60k_mipi_dsi_panel_power(int on)
{
    static struct regulator *reg_l11, *reg_l23, *reg_l2;
    static int gpio43;

    int rc;

    struct pm_gpio gpio43_param = {
        .direction = PM_GPIO_DIR_OUT,
        .output_buffer = PM_GPIO_OUT_BUF_CMOS,
        .output_value = 0,
        .pull = PM_GPIO_PULL_NO,
        .vin_sel = 2,
        .out_strength = PM_GPIO_STRENGTH_HIGH,
        .function = PM_GPIO_FUNC_PAIRED,
        .inv_int_pol = 0,
        .disable_pin = 0,
    };

    if (!dsi_power_on) {
        reg_l11 = regulator_get(&msm_mipi_dsi1_device.dev,"dsi_vdc");
        if (IS_ERR(reg_l11)) {
            pr_err("could not get 8921_l11, rc = %ld\n",PTR_ERR(reg_l11));
            return -ENODEV;
        }
        if (g_A60K_hwID<A66_HW_ID_ER1)
        {
            reg_l23 = regulator_get(&msm_mipi_dsi1_device.dev,"dsi_vddio");
            if (IS_ERR(reg_l23)) {
                pr_err("could not get 8921_l23, rc = %ld\n",PTR_ERR(reg_l23));
                return -ENODEV;
            }
        }
        else
        {
            if (gpio_request(18,"lcd_1.8_en") < 0) {
                pr_err("%s not able to get gpio %d\n", __func__,18);
                return -ENODEV;
            }
            gpio_direction_output(18,0);
        }

        reg_l2 = regulator_get(&msm_mipi_dsi1_device.dev,"dsi_vdda");
        if (IS_ERR(reg_l2)) {
            pr_err("could not get 8921_l2, rc = %ld\n",
                PTR_ERR(reg_l2));
            return -ENODEV;
        }
        rc = regulator_set_voltage(reg_l11, 3100000, 3100000);
        if (rc) {
            pr_err("set_voltage l11 failed, rc=%d\n", rc);
            return -EINVAL;
        }
        if (g_A60K_hwID<A66_HW_ID_ER1)
        {
            rc = regulator_set_voltage(reg_l23, 1800000, 1800000);
            if (rc) {
                pr_err("set_voltage l23 failed, rc=%d\n", rc);
                return -EINVAL;
            }
        }
        rc = regulator_set_voltage(reg_l2, 1200000, 1200000);
        if (rc) {
            pr_err("set_voltage l2 failed, rc=%d\n", rc);
            return -EINVAL;
        }

        gpio43 = PM8921_GPIO_PM_TO_SYS(43);
        rc = gpio_request(gpio43, "disp_rst_n");
        if (rc) {
            pr_err("request gpio 43 failed, rc=%d\n", rc);
            return -ENODEV;
        }

        dsi_power_on = true;
    }

    if (on) {
        rc = regulator_set_optimum_mode(reg_l11, 100000);
        if (rc < 0) {
            pr_err("set_optimum_mode l11 failed, rc=%d\n", rc); 
            return -EINVAL;
        }
        if (g_A60K_hwID<A66_HW_ID_ER1)
        {
            rc = regulator_set_optimum_mode(reg_l23, 100000);
            if (rc < 0) {
                pr_err("set_optimum_mode l23 failed, rc=%d\n", rc);
                return -EINVAL;
            }
        }
        rc = regulator_set_optimum_mode(reg_l2, 100000);
        if (rc < 0) {
            pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
            return -EINVAL;
        }

        rc = regulator_enable(reg_l11);
        if (rc) {
            pr_err("enable l11 failed, rc=%d\n", rc);
            return -ENODEV;
        }
        udelay(5);
        if (g_A60K_hwID<A66_HW_ID_ER1)
        {
            rc = regulator_enable(reg_l23);
            if (rc) {
                pr_err("enable l23 failed, rc=%d\n", rc);
                return -ENODEV;
            }
        }
        else
            gpio_set_value(18,1);

        rc = regulator_enable(reg_l2);
        if (rc) {
            pr_err("enable l2 failed, rc=%d\n", rc);
            return -ENODEV;
        }

        gpio43_param.pull = PM_GPIO_PULL_UP_30;
        gpio43_param.output_value = 1;  
        rc = pm8xxx_gpio_config(gpio43, &gpio43_param);
        if (rc) {
            pr_err("gpio_config 43 failed (2), rc=%d\n", rc);
            return -EINVAL;
        }

        gpio43_param.pull = PM_GPIO_PULL_NO;
        gpio43_param.output_value = 0;  
        rc = pm8xxx_gpio_config(gpio43, &gpio43_param);
        if (rc) {
            pr_err("gpio_config 43 failed (2), rc=%d\n", rc);
            return -EINVAL;
        }
        udelay(15);
        gpio43_param.pull = PM_GPIO_PULL_UP_30;
        gpio43_param.output_value = 1;  
        rc = pm8xxx_gpio_config(gpio43, &gpio43_param);
        if (rc) {
            pr_err("gpio_config 43 failed (2), rc=%d\n", rc);
            return -EINVAL;
        }

        gpio_set_value_cansleep(gpio43, 1);

    } else {
        rc = regulator_disable(reg_l2);
        if (rc) {
            pr_err("disable reg_l2 failed, rc=%d\n", rc);
            return -ENODEV;
        }
        rc = regulator_disable(reg_l11);
        if (rc) {
            pr_err("disable reg_l8 failed, rc=%d\n", rc);
            return -ENODEV;
        }
        if (g_A60K_hwID<A66_HW_ID_ER1)
        {
            rc = regulator_disable(reg_l23);
            if (rc) {
                pr_err("disable reg_l23 failed, rc=%d\n", rc);
                return -ENODEV;
            }
        }
        else
            gpio_set_value(18,0);
        rc = regulator_set_optimum_mode(reg_l11, 100);
        if (rc < 0) {
            pr_err("set_optimum_mode l11 failed, rc=%d\n", rc);
            return -EINVAL;
        }
        if (g_A60K_hwID<A66_HW_ID_ER1)
        {
            rc = regulator_set_optimum_mode(reg_l23, 100);
            if (rc < 0) {
                pr_err("set_optimum_mode l23 failed, rc=%d\n", rc);
                return -EINVAL;
            }
        }
        rc = regulator_set_optimum_mode(reg_l2, 100);
        if (rc < 0) {
            pr_err("set_optimum_mode l2 failed, rc=%d\n", rc);
            return -EINVAL;
        }

        gpio43_param.pull = PM_GPIO_PULL_NO;
        gpio43_param.output_value = 0;  
        rc = pm8xxx_gpio_config(gpio43, &gpio43_param);
        if (rc) {
            pr_err("gpio_config 43 failed (2), rc=%d\n", rc);
            return -EINVAL;
        }
        gpio_set_value_cansleep(gpio43, 0);
    }
    return 0;
}
#endif// --- ASUS_BSP : miniporting

static char mipi_dsi_splash_is_enabled(void);
static int mipi_dsi_panel_power(int on)
{
	int ret;

	pr_info("%s: on=%d\n", __func__, on);
#if 0   //QCT default// +++ ASUS_BSP : miniporting
	if (machine_is_msm8960_liquid())
		ret = mipi_dsi_liquid_panel_power(on);
	else
		ret = mipi_dsi_cdp_panel_power(on);
#else
    return a60k_mipi_dsi_panel_power(on);
#endif// --- ASUS_BSP : miniporting
	return ret;
}

static struct mipi_dsi_platform_data mipi_dsi_pdata = {
	.vsync_gpio = MDP_VSYNC_GPIO,
	.dsi_power_save = mipi_dsi_panel_power,
	.splash_is_enabled = mipi_dsi_splash_is_enabled,
};

#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors mdp_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors mdp_ui_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 2000000000,//216000000 * 2,
		.ib = 2000000000,//270000000 * 2,
	},
};

static struct msm_bus_vectors mdp_vga_vectors[] = {
	/* VGA and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 2000000000,//216000000 * 2,
		.ib = 2000000000,//270000000 * 2,
	},
};

static struct msm_bus_vectors mdp_720p_vectors[] = {
	/* 720p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 2000000000,//230400000 * 2,
		.ib = 2000000000,//288000000 * 2,
	},
};

static struct msm_bus_vectors mdp_1080p_vectors[] = {
	/* 1080p and less video */
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 2000000000,//334080000 * 2,
		.ib = 2000000000,//417600000 * 2,
	},
};

static struct msm_bus_paths mdp_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(mdp_init_vectors),
		mdp_init_vectors,
	},
	{
		ARRAY_SIZE(mdp_ui_vectors),
		mdp_ui_vectors,
	},
	{
		ARRAY_SIZE(mdp_ui_vectors),
		mdp_ui_vectors,
	},
	{
		ARRAY_SIZE(mdp_vga_vectors),
		mdp_vga_vectors,
	},
	{
		ARRAY_SIZE(mdp_720p_vectors),
		mdp_720p_vectors,
	},
	{
		ARRAY_SIZE(mdp_1080p_vectors),
		mdp_1080p_vectors,
	},
};

static struct msm_bus_scale_pdata mdp_bus_scale_pdata = {
	mdp_bus_scale_usecases,
	ARRAY_SIZE(mdp_bus_scale_usecases),
	.name = "mdp",
};

#endif

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = MDP_VSYNC_GPIO,
	.mdp_max_clk = 200000000,
#ifdef CONFIG_MSM_BUS_SCALING
	.mdp_bus_scale_table = &mdp_bus_scale_pdata,
#endif
	.mdp_rev = MDP_REV_42,
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	.mem_hid = BIT(ION_CP_MM_HEAP_ID),
#else
	.mem_hid = MEMTYPE_EBI1,
#endif
	.cont_splash_enabled = 0,   //ASUS BSP: disable splash feature
	.mdp_iommu_split_domain = 0,
};

void __init msm8960_mdp_writeback(struct memtype_reserve* reserve_table)
{
	mdp_pdata.ov0_wb_size = MSM_FB_OVERLAY0_WRITEBACK_SIZE;
	mdp_pdata.ov1_wb_size = MSM_FB_OVERLAY1_WRITEBACK_SIZE;
#if defined(CONFIG_ANDROID_PMEM) && !defined(CONFIG_MSM_MULTIMEDIA_USE_ION)
	reserve_table[mdp_pdata.mem_hid].size +=
		mdp_pdata.ov0_wb_size;
	reserve_table[mdp_pdata.mem_hid].size +=
		mdp_pdata.ov1_wb_size;
#endif
}

static char mipi_dsi_splash_is_enabled(void)
{
	return mdp_pdata.cont_splash_enabled;
}

#if 0   //QCT default// +++ ASUS_BSP : miniporting
static struct platform_device mipi_dsi_renesas_panel_device = {
	.name = "mipi_renesas",
	.id = 0,
};

static struct platform_device mipi_dsi_simulator_panel_device = {
	.name = "mipi_simulator",
	.id = 0,
};

#define LPM_CHANNEL0 0
static int toshiba_gpio[] = {LPM_CHANNEL0};

static struct mipi_dsi_panel_platform_data toshiba_pdata = {
	.gpio = toshiba_gpio,
	.dsi_pwm_cfg = mipi_dsi_panel_pwm_cfg,
};

static struct platform_device mipi_dsi_toshiba_panel_device = {
	.name = "mipi_toshiba",
	.id = 0,
	.dev = {
		.platform_data = &toshiba_pdata,
	}
};

#define FPGA_3D_GPIO_CONFIG_ADDR	0xB5
static int dsi2lvds_gpio[4] = {
	0,/* Backlight PWM-ID=0 for PMIC-GPIO#24 */
	0x1F08, /* DSI2LVDS Bridge GPIO Output, mask=0x1f, out=0x08 */
	GPIO_LIQUID_EXPANDER_BASE+6,	/* TN Enable */
	GPIO_LIQUID_EXPANDER_BASE+7,	/* TN Mode */
	};

static struct msm_panel_common_pdata mipi_dsi2lvds_pdata = {
	.gpio_num = dsi2lvds_gpio,
};
#endif// --- ASUS_BSP : miniporting
static struct mipi_dsi_phy_ctrl dsi_novatek_cmd_mode_phy_db = {

/* DSI_BIT_CLK at 500MHz, 2 lane, RGB888 */
	{0x0F, 0x0a, 0x04, 0x00, 0x20},	/* regulator */
	/* timing   */
	{0xab, 0x8a, 0x18, 0x00, 0x92, 0x97, 0x1b, 0x8c,
	0x0c, 0x03, 0x04, 0xa0},
	{0x5f, 0x00, 0x00, 0x10},	/* phy ctrl */
	{0xff, 0x00, 0x06, 0x00},	/* strength */
	/* pll control */
	{0x40, 0xf9, 0x30, 0xda, 0x00, 0x40, 0x03, 0x62,
	0x40, 0x07, 0x03,
	0x00, 0x1a, 0x00, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01},
};

static struct mipi_dsi_panel_platform_data novatek_pdata = {
	//.fpga_3d_config_addr  = FPGA_3D_GPIO_CONFIG_ADDR, //QCT default// +++ ASUS_BSP : miniporting
	//.fpga_ctrl_mode = FPGA_SPI_INTF, //QCT default// +++ ASUS_BSP : miniporting
	.phy_ctrl_settings = &dsi_novatek_cmd_mode_phy_db,
};

static struct platform_device mipi_dsi_novatek_panel_device = {
	.name = "mipi_novatek",
	.id = 0,
	.dev = {
		.platform_data = &novatek_pdata,
	}
};
#if 0   //QCT default// +++ ASUS_BSP : miniporting
static struct platform_device mipi_dsi2lvds_bridge_device = {
	.name = "mipi_tc358764",
	.id = 0,
	.dev.platform_data = &mipi_dsi2lvds_pdata,
};
#endif// --- ASUS_BSP : miniporting
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
static struct resource hdmi_msm_resources[] = {
	{
		.name  = "hdmi_msm_qfprom_addr",
		.start = 0x00700000,
		.end   = 0x007060FF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "hdmi_msm_hdmi_addr",
		.start = 0x04A00000,
		.end   = 0x04A00FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "hdmi_msm_irq",
		.start = HDMI_IRQ,
		.end   = HDMI_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static int hdmi_enable_5v(int on);
static int hdmi_core_power(int on, int show);
static void hdmi_ddc_switch(bool ddc, bool on); //Mickey, add for hdmi ddc switch
//static int hdmi_cec_power(int on); //ASUS_BSP joe1_++
static int hdmi_gpio_config(int on);
//static int hdmi_panel_power(int on);

static struct msm_hdmi_platform_data hdmi_msm_data = {
	.irq = HDMI_IRQ,
	.enable_5v = hdmi_enable_5v,
	.core_power = hdmi_core_power,
    .ddc_switch = hdmi_ddc_switch, //Mickey, add for hdmi ddc switch
//  .cec_power = hdmi_cec_power, //ASUS_BSP joe1_++
//	.panel_power = hdmi_panel_power,
	.gpio_config = hdmi_gpio_config,
};

static struct platform_device hdmi_msm_device = {
	.name = "hdmi_msm",
	.id = 0,
	.num_resources = ARRAY_SIZE(hdmi_msm_resources),
	.resource = hdmi_msm_resources,
	.dev.platform_data = &hdmi_msm_data,
};
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */

#ifdef CONFIG_FB_MSM_WRITEBACK_MSM_PANEL
static struct platform_device wfd_panel_device = {
	.name = "wfd_panel",
	.id = 0,
	.dev.platform_data = NULL,
};

static struct platform_device wfd_device = {
	.name          = "msm_wfd",
	.id            = -1,
};
#endif

#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors dtv_bus_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors dtv_bus_def_vectors[] = {
	{
		.src = MSM_BUS_MASTER_MDP_PORT0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 2000000000,//566092800 * 2,
		.ib = 2000000000,//707616000 * 2,
	},
};

static struct msm_bus_paths dtv_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(dtv_bus_init_vectors),
		dtv_bus_init_vectors,
	},
	{
		ARRAY_SIZE(dtv_bus_def_vectors),
		dtv_bus_def_vectors,
	},
};
static struct msm_bus_scale_pdata dtv_bus_scale_pdata = {
	dtv_bus_scale_usecases,
	ARRAY_SIZE(dtv_bus_scale_usecases),
	.name = "dtv",
};

static struct lcdc_platform_data dtv_pdata = {
	.bus_scale_table = &dtv_bus_scale_pdata,
//	.lcdc_power_save = hdmi_panel_power,
};
#if 0 // Tingyi
static int hdmi_panel_power(int on)
{
	int rc;

	pr_debug("%s: HDMI Core: %s\n", __func__, (on ? "ON" : "OFF"));
	rc = hdmi_core_power(on, 1);
	//if (rc)
	//	rc = hdmi_cec_power(on);

	pr_debug("%s: HDMI Core: %s Success\n", __func__, (on ? "ON" : "OFF"));
	return rc;
}
#endif
#endif

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
//Mickey+++, turn on 5v boost and gsbi level shift
//#define HDMI_5V_BOOST_EN 4
//#define GSBI_I2C_LS_OE g_GPIO_P01_I2C_LS_OE
#define DDC_I2C_LS_OE 7
#define HDMI_GPIO_50 50 //Mickey add for ER3
bool g_ddcEnable = false; //Mickey, add for hdmi ddc switch
//Mickey---
static int hdmi_enable_5v(int on)
{
#if 0 //Mickey+++, Qualcomm default
	/* TBD: PM8921 regulator instead of 8901 */
	static struct regulator *reg_8921_hdmi_mvs;	/* HDMI_5V */
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (!reg_8921_hdmi_mvs) {
		reg_8921_hdmi_mvs = regulator_get(&hdmi_msm_device.dev,
					"hdmi_mvs");
		if (IS_ERR(reg_8921_hdmi_mvs)) {
			pr_err("'%s' regulator not found, rc=%ld\n",
				"hdmi_mvs", IS_ERR(reg_8921_hdmi_mvs));
			reg_8921_hdmi_mvs = NULL;
			return -ENODEV;
		}
	}

	if (on) {
		rc = regulator_enable(reg_8921_hdmi_mvs);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"8921_hdmi_mvs", rc);
			return rc;
		}
		pr_debug("%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_8921_hdmi_mvs);
		if (rc)
			pr_warning("'%s' regulator disable failed, rc=%d\n",
				"8921_hdmi_mvs", rc);
		pr_debug("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;
#else
    static bool firstOn = true;
    if (firstOn)
    {   
        if (gpio_request(g_GPIO_HDMI_5V_ENABLE,"hdmi_ddc_boost5v") < 0) {
            pr_err("%s not able to get gpio %d\n", __func__,g_GPIO_HDMI_5V_ENABLE);
            //return -1;
        }
        printk("==== pull up GPIO_P01_I2C_LS_OE =====\n");
        if (gpio_request(g_GPIO_P01_I2C_LS_OE,"hdmi_gsbi_5v_LS") < 0) {
            pr_err("%s not able to get gpio %d\n", __func__,g_GPIO_P01_I2C_LS_OE);
            return -1;
        }
        if (gpio_request(DDC_I2C_LS_OE,"hdmi_ddc_5v_LS") < 0) {
            pr_err("%s not able to get gpio %d\n", __func__,DDC_I2C_LS_OE);
            return -1;
        }
        gpio_direction_output(g_GPIO_P01_I2C_LS_OE,0);
        gpio_direction_output(g_GPIO_HDMI_5V_ENABLE,0);
        gpio_direction_output(DDC_I2C_LS_OE,0);
        if (g_A60K_hwID>=A66_HW_ID_ER3)
        {
            if (gpio_request(HDMI_GPIO_50,"hdmi_gpio_50") < 0) {
                pr_err("%s not able to get gpio %d\n", __func__,HDMI_GPIO_50);
                return -1;
            }
            gpio_direction_output(HDMI_GPIO_50,1);
            printk("Mickey:: ER HDMI GPIO 50\n");
        }
        firstOn = false;
    }

    if (on)
        gpio_set_value(g_GPIO_HDMI_5V_ENABLE,1);
    else
        gpio_set_value(g_GPIO_HDMI_5V_ENABLE,0);
    return 0;
#endif//Mickey---
}

//Mickey++++, add for hdmi ddc switch
static void hdmi_ddc_switch(bool ddc, bool on)
{
    if (on)
    {
        if (ddc)
        {
            g_ddcEnable = true;
            gpio_set_value(DDC_I2C_LS_OE,1);
            gpio_set_value(g_GPIO_P01_I2C_LS_OE,0);
            msleep(15);
            if (g_A60K_hwID>=A66_HW_ID_ER3)
            {
                gpio_set_value(HDMI_GPIO_50,1);
                printk("Mickey:: ER HDMI GPIO 50 high\n");
            }
        }
        else
        {
            g_ddcEnable = false;
            gpio_set_value(DDC_I2C_LS_OE,0);
            gpio_set_value(g_GPIO_P01_I2C_LS_OE,1);
            msleep(15);
            if (g_A60K_hwID>=A66_HW_ID_ER3)
            {
                gpio_set_value(HDMI_GPIO_50,0);
                printk("Mickey:: ER HDMI GPIO 50 low\n");
            }
        }
    }
    else
    {
        g_ddcEnable = false;
        gpio_set_value(DDC_I2C_LS_OE,0);
        gpio_set_value(g_GPIO_P01_I2C_LS_OE,0);
        if (g_A60K_hwID>=A66_HW_ID_ER3)
        {
            gpio_set_value(HDMI_GPIO_50,1);
            printk("Mickey:: ER HDMI GPIO 50 high\n");
        }
    }
}

bool g_hdmi_exist = false;

bool hdmi_exist_realtime(void)
{
    g_hdmi_exist = gpio_get_value_cansleep(102);
    printk("HDMI: %s = %d\n",__func__,g_hdmi_exist);
    return g_hdmi_exist;
}
EXPORT_SYMBOL(hdmi_exist_realtime);

bool hdmi_exist(void)
{
    return g_hdmi_exist;
}
EXPORT_SYMBOL(hdmi_exist);
//Mickey---

static int hdmi_core_power(int on, int show)
{
	static struct regulator *reg_8921_l23, *reg_8921_s4;
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	/* TBD: PM8921 regulator instead of 8901 */
	if (!reg_8921_l23) {
		reg_8921_l23 = regulator_get(&hdmi_msm_device.dev, "hdmi_avdd");
		if (IS_ERR(reg_8921_l23)) {
			pr_err("could not get reg_8921_l23, rc = %ld\n",
				PTR_ERR(reg_8921_l23));
			return -ENODEV;
		}
		rc = regulator_set_voltage(reg_8921_l23, 1800000, 1800000);
		if (rc) {
			pr_err("set_voltage failed for 8921_l23, rc=%d\n", rc);
			return -EINVAL;
		}
	}
	if (!reg_8921_s4) {
		reg_8921_s4 = regulator_get(&hdmi_msm_device.dev, "hdmi_vcc");
		if (IS_ERR(reg_8921_s4)) {
			pr_err("could not get reg_8921_s4, rc = %ld\n",
				PTR_ERR(reg_8921_s4));
			return -ENODEV;
		}
		rc = regulator_set_voltage(reg_8921_s4, 1800000, 1800000);
		if (rc) {
			pr_err("set_voltage failed for 8921_s4, rc=%d\n", rc);
			return -EINVAL;
		}
	}

	if (on) {
		rc = regulator_set_optimum_mode(reg_8921_l23, 100000);
		if (rc < 0) {
			pr_err("set_optimum_mode l23 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		rc = regulator_enable(reg_8921_l23);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"hdmi_avdd", rc);
			return rc;
		}
		rc = regulator_enable(reg_8921_s4);
		if (rc) {
			pr_err("'%s' regulator enable failed, rc=%d\n",
				"hdmi_vcc", rc);
			return rc;
		}
		pr_debug("%s(on): success\n", __func__);
	} else {
		rc = regulator_disable(reg_8921_l23);
		if (rc) {
			pr_err("disable reg_8921_l23 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_disable(reg_8921_s4);
		if (rc) {
			pr_err("disable reg_8921_s4 failed, rc=%d\n", rc);
			return -ENODEV;
		}
		rc = regulator_set_optimum_mode(reg_8921_l23, 100);
		if (rc < 0) {
			pr_err("set_optimum_mode l23 failed, rc=%d\n", rc);
			return -EINVAL;
		}
		pr_debug("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;
}

static int hdmi_gpio_config(int on)
{
	int rc = 0;
	static int prev_on;

	if (on == prev_on)
		return 0;

	if (on) {
		rc = gpio_request(100, "HDMI_DDC_CLK");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_DDC_CLK", 100, rc);
			return rc;
		}
		rc = gpio_request(101, "HDMI_DDC_DATA");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_DDC_DATA", 101, rc);
			goto error1;
		}
		rc = gpio_request(102, "HDMI_HPD");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_HPD", 102, rc);
			goto error2;
		}
		pr_debug("%s(on): success\n", __func__);
	} else {
		gpio_free(100);
		gpio_free(101);
		gpio_free(102);
		pr_debug("%s(off): success\n", __func__);
	}

	prev_on = on;
	return 0;

error2:
	gpio_free(101);
error1:
	gpio_free(100);
	return rc;
}

// +++ Louis
#include <linux/fb.h> 

struct fb_info * asus_get_fb_info(int fb_num);
void asus_update_screen(struct fb_var_screeninfo *var, struct fb_info *info);
bool update_screen_flag = false;
extern int mdp4_overlay_vsync_ctrl(struct fb_info *info, int enable);

void asus_draw_last_screen(u32 img_color)
{
    unsigned char *src_addr, *dest_addr;
    char img_col[4];
    unsigned int pixels_num = 0;
    struct fb_info * fbi = asus_get_fb_info(0);
    u32 tmp_yoffset;
    u32 tmp_ysride;

    update_screen_flag = true;

    img_col[0] = (img_color & 0xFF000000) >> 24;
    img_col[1] = (img_color & 0x00FF0000) >> 16;
    img_col[2] = (img_color & 0x0000FF00) >> 8;
    img_col[3] = (img_color & 0x000000FF) ;

    tmp_yoffset = fbi->var.yoffset;
    tmp_ysride = fbi->fix.line_length;

    fbi->fix.line_length = 544; //ystride
    fbi->var.yoffset = 960 * 3;

    src_addr = (unsigned char *) __va((fbi->fix.smem_start)) +  fbi->var.yoffset * fbi->fix.line_length;

    dest_addr = src_addr;

    memcpy(dest_addr, img_col, sizeof(img_col)); //copy first 4 byte
    pixels_num += 1;    //handle first pixel

    while (pixels_num < (544 * 960 / 2))
    {
        memcpy(dest_addr + 4 * pixels_num, src_addr, 4 * pixels_num);
        pixels_num = pixels_num * 2;
    }

    if (pixels_num < 544 * 960) {
        memcpy(dest_addr + 4 * pixels_num, src_addr, 4 * (544 * 960 - pixels_num));
    }

    mdp4_overlay_vsync_ctrl(fbi, 1);
    asus_update_screen(&(fbi->var),fbi);
    mdp4_overlay_vsync_ctrl(fbi, 0);

    fbi->var.yoffset = tmp_yoffset;
    fbi->fix.line_length = tmp_ysride;

    update_screen_flag = false;
    return;
}
EXPORT_SYMBOL(asus_draw_last_screen);
// --- Louis



//ASUS_BSP joe1_++
#if 0
static int hdmi_cec_power(int on)
{
	static int prev_on;
	int rc;

	if (on == prev_on)
		return 0;

	if (on) {
		rc = gpio_request(99, "HDMI_CEC_VAR");
		if (rc) {
			pr_err("'%s'(%d) gpio_request failed, rc=%d\n",
				"HDMI_CEC_VAR", 99, rc);
			goto error;
		}
		pr_debug("%s(on): success\n", __func__);
	} else {
		gpio_free(99);
		pr_debug("%s(off): success\n", __func__);
	}

	prev_on = on;

	return 0;
error:
	return rc;
}
#endif
//ASUS_BSP joe1_--
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL */

#ifndef ASUS_SHIP_BUILD
//Mickey+++, for display show string function in board initial stage
#define MDP_PHY_BASE                          (0x05100000)
#define REG_MDP(off)                          (mdp_base + (off))

#define MY_MDP_INTR_ENABLE                    REG_MDP(0x00050)
#define MDP_OVERLAYPROC0_CFG                  REG_MDP(0x10004)
#define MDP_DMA_P_CONFIG                      REG_MDP(0x90000)
#define MDP_DMA_P_OUT_XY                      REG_MDP(0x90010)
#define MDP_DMA_P_SIZE                        REG_MDP(0x90004)
#define MDP_DMA_P_BUF_ADDR                    REG_MDP(0x90008)
#define MDP_DMA_P_BUF_Y_STRIDE                REG_MDP(0x9000C)
#define MDP_DMA_P_OP_MODE                     REG_MDP(0x90070)
#define MDP_DSI_CMD_MODE_ID_MAP               REG_MDP(0x000A0)
#define MDP_DSI_CMD_MODE_TRIGGER_EN           REG_MDP(0x000A4)
#define MDP_AXI_RDMASTER_CONFIG               REG_MDP(0x00028)
#define MDP_AXI_WRMASTER_CONFIG               REG_MDP(0x00030)
#define MDP_MAX_RD_PENDING_CMD_CONFIG         REG_MDP(0x0004C)
#define MDP_DISP_INTF_SEL                     REG_MDP(0x00038)
#define MDP_DMA_P_START                       REG_MDP(0x0000C)
#define MDP_DSI_VIDEO_EN                      REG_MDP(0xE0000)
#define MDP_OVERLAYPROC0_CFG                  REG_MDP(0x10004)


#define MIPI_DSI_PHY_BASE                     (0x04700000)
#define REG_DSI(off)                          (dsi_base + (off))
#define DSI_CTRL                              REG_DSI(0x000)
#define DSI_CLK_CTRL                          REG_DSI(0x118)
#define DSI_ERR_INT_MASK0                     REG_DSI(0x108)
#define DSI_COMMAND_MODE_MDP_CTRL             REG_DSI(0x03C)
#define DSI_COMMAND_MODE_MDP_STREAM0_CTRL     REG_DSI(0x054)
#define DSI_COMMAND_MODE_MDP_STREAM1_CTRL     REG_DSI(0x05C)
#define DSI_COMMAND_MODE_MDP_STREAM0_TOTAL    REG_DSI(0x058)
#define DSI_COMMAND_MODE_MDP_STREAM1_TOTAL    REG_DSI(0x060)
#define DSI_CAL_STRENGTH_CTRL                 REG_DSI(0x100)
#define DSI_CAL_CTRL                          REG_DSI(0x0F4)
#define DSI_TRIG_CTRL                         REG_DSI(0x080)
#define DSI_COMMAND_MODE_MDP_DCS_CMD_CTRL     REG_DSI(0x040)
#define DSI_COMMAND_MODE_DMA_CTRL             REG_DSI(0x038)
#define DSI_MISR_CMD_CTRL                     REG_DSI(0x09C)
#define DSI_EOT_PACKET_CTRL                   REG_DSI(0x0C8)
#define DSI_CMD_MODE_MDP_SW_TRIGGER           REG_DSI(0x090)
#define DSI_INT_CTRL                          REG_DSI(0x10C)

char *mdp_base;
char *dsi_base;

int mipi_dsi_cmd_config(unsigned short image_wd, unsigned short image_ht)
{

    int status = 0;
    unsigned long input_img_addr = msm_fb_resources[0].start;
    unsigned short pack_pattern = 0x12;
    unsigned char ystride = 3;

    writel(0x03ffffff, MY_MDP_INTR_ENABLE);

    writel(0x0000000b, MDP_OVERLAYPROC0_CFG);

    // ------------- programming MDP_DMA_P_CONFIG ---------------------
    writel(pack_pattern << 8 | 0x3f | (0 << 25), MDP_DMA_P_CONFIG); // rgb888

    writel(0x00000000, MDP_DMA_P_OUT_XY);
    writel(image_ht << 16 | image_wd, MDP_DMA_P_SIZE);
    writel(input_img_addr, MDP_DMA_P_BUF_ADDR);

    writel(image_wd * ystride, MDP_DMA_P_BUF_Y_STRIDE);

    writel(0x00000000, MDP_DMA_P_OP_MODE);

    writel(0x10, MDP_DSI_CMD_MODE_ID_MAP);
    writel(0x01, MDP_DSI_CMD_MODE_TRIGGER_EN);

    writel(0x0001a000, MDP_AXI_RDMASTER_CONFIG);
    writel(0x00000004, MDP_AXI_WRMASTER_CONFIG);
    writel(0x00007777, MDP_MAX_RD_PENDING_CMD_CONFIG);
    writel(0x8a, MDP_DISP_INTF_SEL);
    return status;
}

int config_dsi_cmd_mode(unsigned short img_width, unsigned short img_height)
{
    unsigned char DST_FORMAT;
    unsigned char TRAFIC_MODE;
    unsigned char DLNx_EN;
    // video mode data ctrl
    int status = 0;
    unsigned char interleav = 0;
    unsigned char ystride = 0x03;
    // disable mdp first

    writel(0x00000000, DSI_CLK_CTRL);
    writel(0x00000000, DSI_CLK_CTRL);
    writel(0x00000000, DSI_CLK_CTRL);
    writel(0x00000000, DSI_CLK_CTRL);
    writel(0x00000002, DSI_CLK_CTRL);
    writel(0x00000006, DSI_CLK_CTRL);
    writel(0x0000000e, DSI_CLK_CTRL);
    writel(0x0000001e, DSI_CLK_CTRL);
    writel(0x0000003e, DSI_CLK_CTRL);

    writel(0x10000000, DSI_ERR_INT_MASK0);

    // writel(0, DSI_CTRL);

    // writel(0, DSI_ERR_INT_MASK0);

    DST_FORMAT = 8;             // RGB888

    DLNx_EN = 3;                // 2 lane with clk programming

    TRAFIC_MODE = 0;            // non burst mode with sync pulses

    writel(0x02020202, DSI_INT_CTRL);

    writel(0x00100000 | DST_FORMAT, DSI_COMMAND_MODE_MDP_CTRL);
    writel((img_width * ystride + 1) << 16 | 0x0039,
           DSI_COMMAND_MODE_MDP_STREAM0_CTRL);
    writel((img_width * ystride + 1) << 16 | 0x0039,
           DSI_COMMAND_MODE_MDP_STREAM1_CTRL);
    writel(img_height << 16 | img_width, DSI_COMMAND_MODE_MDP_STREAM0_TOTAL);
    writel(img_height << 16 | img_width, DSI_COMMAND_MODE_MDP_STREAM1_TOTAL);
    writel(0xEE, DSI_CAL_STRENGTH_CTRL);
    writel(0x80000000, DSI_CAL_CTRL);
    writel(0x40, DSI_TRIG_CTRL);
    writel(0x13c2c, DSI_COMMAND_MODE_MDP_DCS_CMD_CTRL);
    writel(interleav << 30 | 0 << 24 | 0 << 20 | DLNx_EN << 4 | 0x105,
           DSI_CTRL);
    mdelay(10);
    writel(0x10000000, DSI_COMMAND_MODE_DMA_CTRL);
    writel(0x10000000, DSI_MISR_CMD_CTRL);
    writel(0x00000040, DSI_ERR_INT_MASK0);
    writel(0x1, DSI_EOT_PACKET_CTRL);
    // writel(0x0, MDP_OVERLAYPROC0_START);
    writel(0x00000001, MDP_DMA_P_START);
    mdelay(10);
    writel(0x1, DSI_CMD_MODE_MDP_SW_TRIGGER);

    status = 1;
    return status;
}

void asus_pan_display(void)
{
    unsigned short screen_wd;
    unsigned short screen_ht;


    screen_wd = 540;
    screen_ht = 960;

    mdp_base = ioremap(MDP_PHY_BASE, 0x000F0000);
    dsi_base = ioremap(MIPI_DSI_PHY_BASE, 0x000F0000);
//if the dsi control register is in init state, then we don't pan(not init in aboot)
    if (readl(DSI_CTRL)==0) 
        return;
    mipi_dsi_cmd_config(screen_wd, screen_ht);
    mdelay(50);
    config_dsi_cmd_mode(screen_wd, screen_ht);
}
#define FONT_WIDTH      5
#define FONT_HEIGHT     12
unsigned font5x12[] = {
    0x00000000, 0x00000000,
    0x08421080, 0x00020084,
    0x00052940, 0x00000000,
    0x15f52800, 0x0000295f,
    0x1c52f880, 0x00023e94,
    0x08855640, 0x0004d542,
    0x04528800, 0x000b2725,
    0x00021080, 0x00000000,
    0x04211088, 0x00821042,
    0x10841082, 0x00221108,
    0x09575480, 0x00000000,
    0x3e420000, 0x00000084,
    0x00000000, 0x00223000,
    0x3e000000, 0x00000000,
    0x00000000, 0x00471000,
    0x08844200, 0x00008442,
    0x2318a880, 0x00022a31,
    0x08429880, 0x000f9084,
    0x1108c5c0, 0x000f8444,
    0x1c4443e0, 0x00074610,
    0x14a62100, 0x000423e9,
    0x26d087e0, 0x00074610,
    0x1e10c5c0, 0x00074631,
    0x088443e0, 0x00010844,
    0x1d18c5c0, 0x00074631,
    0x3d18c5c0, 0x00074610,
    0x08e20000, 0x00471000,
    0x08e20000, 0x00223000,
    0x02222200, 0x00082082,
    0x01f00000, 0x000003e0,
    0x20820820, 0x00008888,
    0x1108c5c0, 0x00020084,
    0x2b98c5c0, 0x000f05b5,
    0x2318a880, 0x0008c63f,
    0x1d2949e0, 0x0007ca52,
    0x0210c5c0, 0x00074421,
    0x252949e0, 0x0007ca52,
    0x1e1087e0, 0x000f8421,
    0x1e1087e0, 0x00008421,
    0x0210c5c0, 0x00074639,
    0x3f18c620, 0x0008c631,
    0x084211c0, 0x00071084,
    0x10842380, 0x00032508,
    0x0654c620, 0x0008c525,
    0x02108420, 0x000f8421,
    0x2b5dc620, 0x0008c631,
    0x2b59ce20, 0x0008c739,
    0x2318c5c0, 0x00074631,
    0x1f18c5e0, 0x00008421,
    0x2318c5c0, 0x01075631,
    0x1f18c5e0, 0x0008c525,
    0x1c10c5c0, 0x00074610,
    0x084213e0, 0x00021084,
    0x2318c620, 0x00074631,
    0x1518c620, 0x0002114a,
    0x2b18c620, 0x000556b5,
    0x08a54620, 0x0008c54a,
    0x08a54620, 0x00021084,
    0x088443e0, 0x000f8442,
    0x0421084e, 0x00e10842,
    0x08210420, 0x00084108,
    0x1084210e, 0x00e42108,
    0x0008a880, 0x00000000,
    0x00000000, 0x01f00000,
    0x00000104, 0x00000000,
    0x20e00000, 0x000b663e,
    0x22f08420, 0x0007c631,
    0x22e00000, 0x00074421,
    0x23e84200, 0x000f4631,
    0x22e00000, 0x0007443f,
    0x1e214980, 0x00010842,
    0x22e00000, 0x1d187a31,
    0x26d08420, 0x0008c631,
    0x08601000, 0x00071084,
    0x10c02000, 0x0c94a108,
    0x0a908420, 0x0008a4a3,
    0x084210c0, 0x00071084,
    0x2ab00000, 0x0008d6b5,
    0x26d00000, 0x0008c631,
    0x22e00000, 0x00074631,
    0x22f00000, 0x0210be31,
    0x23e00000, 0x21087a31,
    0x26d00000, 0x00008421,
    0x22e00000, 0x00074506,
    0x04f10800, 0x00064842,
    0x23100000, 0x000b6631,
    0x23100000, 0x00022951,
    0x23100000, 0x000556b5,
    0x15100000, 0x0008a884,
    0x23100000, 0x1d185b31,
    0x11f00000, 0x000f8444,
    0x06421098, 0x01821084,
    0x08421080, 0x00021084,
    0x30421083, 0x00321084,
    0x0004d640, 0x00000000,
    0x00000000, 0x00000000,
};
/////////////////////////////////////////////////////////////////////////////////////
#define FB_WIDTH 1280
#define FB_HEIGHT 960
static int printk_lcd_line = 0;
static char message[128];
#endif
int get_fb_phys_info(unsigned long *start, unsigned long *len, int fb_num);
struct fb_info * asus_get_fb_info(int fb_num);
void asus_update_screen(struct fb_var_screeninfo *var,struct fb_info *info);
void printk_lcd_string(int xx, int yy, char* message,  unsigned int color)
{
#ifdef ASUS_SHIP_BUILD
    return;
#else
    unsigned char *pixels, *pos_orig;
    unsigned * glyph;
    int  m;
    char *s;
    unsigned x, xxx, y, tmpData, data;
    int frame_buffer_id, j;
    unsigned short screen_wd;
    unsigned short screen_ht;
    unsigned short stride;
    struct fb_info * fbi = asus_get_fb_info(0);
   
    screen_wd = FB_WIDTH;
    screen_ht = FB_HEIGHT;
    
    stride = screen_wd * 4;
    stride -= ((FONT_WIDTH)*2*4);
    xxx = xx;
    
    pos_orig = (unsigned char *)__va((char *)msm_fb_resources[0].start)  ;//+ fbi->var.buffer_offset;;
    //printk("pos_orig = %p, fbi->var.buffer_offset = %d, xx=%d\n", pos_orig, fbi->var.buffer_offset, xx);
    for(frame_buffer_id = 0; frame_buffer_id < 2; frame_buffer_id++) // two frame buffers
    {
        pos_orig = pos_orig + frame_buffer_id * FB_WIDTH * FB_HEIGHT * 4; //every frame buffer is 1280*960*4 bytes
        xx = xxx;
        //printk("printing pos_orig=%p\n",pos_orig);
        s = &message[0];
        while (*s) 
        {
            pixels = pos_orig + yy * screen_wd * 4 + xx * 4; //start position = line start
            glyph = &font5x12[((unsigned char)*s - 32) * 2] ; //find the word's array address
            //printk("printing char %c, xx=%d, yy=%d\n", *s, xx, yy);
            //printk(" statrting pixels addr = %p\n", pixels);
            for(j = 0; j < 2; j++) //upper side and lower side of character
            {
                data = glyph[j]; //get upper or lower
                
                for (y = 0; y < (FONT_HEIGHT / 2); y++)  // upper of character total 12 / 2 = 6 pixels
                {
                    for(m = 0; m <2 ; m++) // 2X2 for each pixel
                    {
                        tmpData = data;
                        for (x = 0; x < FONT_WIDTH; x++) 
                        {
                            if (data & 1)
                            {
                                *((unsigned int*)pixels) = color;
                                pixels+=4;
                                *((unsigned int*)pixels) = color;
                                pixels+=4;
                            }
                            else
                                pixels+=8;
                            data >>= 1;
                        }
                        pixels += stride;
                        if(m == 0) //go back to first pixel for next line
                            data = tmpData;
                        //printk("pixels addr = %p\n", pixels);
                    }
                }
            }
    
            xx += FONT_WIDTH * 2; // goto next character
            s++;  //goto next character
        }
    }
    
    asus_update_screen(&(fbi->var),fbi);
#endif
}
void printk_bar(int xx, int yy, int width, int height, unsigned int color)
{
#ifdef ASUS_SHIP_BUILD
    return;
#else
    unsigned char *pixels, *pos_orig;
    unsigned x, y;
    int frame_buffer_id;
    unsigned short screen_wd;
    unsigned short screen_ht;
    unsigned short stride;
    struct fb_info * fbi = asus_get_fb_info(0);
    screen_wd = FB_WIDTH;
    screen_ht = FB_HEIGHT;
    
    stride = screen_wd * 4 - width * 4;
    //pos_orig = ioremap_nocache(msm_fb_resources[0].start, 1280 * 960 * 4 * 2)+ fbi->var.buffer_offset;;
    pos_orig = (unsigned char *)__va((char *)msm_fb_resources[0].start); //+ fbi->var.buffer_offset;
    for(frame_buffer_id = 0; frame_buffer_id < 2; frame_buffer_id++) // two frame buffers
    {
        pos_orig = pos_orig + frame_buffer_id * FB_WIDTH * FB_HEIGHT * 4; //every frame buffer is 1280*960*4 bytes
        
        pixels = pos_orig + yy * screen_wd * 4 + xx * 4; //start position = line start
                
        for (y = 0; y < height; y++)  // height 
        {
            for (x = 0; x < width; x++) //width
            {
                *((unsigned int*)pixels) = color;
                pixels += 4;
            }
            pixels += stride;
        }

    }
    asus_update_screen(&(fbi->var),fbi);
#endif
}
void printk_lcd_xy(int xx, int yy, unsigned int color, const char *fmt, ...)
{
#ifdef ASUS_SHIP_BUILD
    return;
#else
    va_list args;
    va_start(args, fmt);
    vscnprintf(message, sizeof(message), fmt, args);
    va_end(args);
   
    printk_bar(xx, yy, strlen(message) * FONT_WIDTH * 2, (FONT_HEIGHT * 2 + 2), 0x00000000);
    printk_lcd_string(xx, yy, message, color);
#endif
}
void printk_lcd(const char *fmt, ...)
{
#ifdef ASUS_SHIP_BUILD
    return;
#else
    unsigned int yy;
    va_list args;
    va_start(args, fmt);
    vscnprintf(message, sizeof(message), fmt, args);
    va_end(args);
    
    printk(message);
    
    yy = printk_lcd_line  * (FONT_HEIGHT * 2 + 2); //(line start_y) = printk_lcd_line * (each word's height(FONT_HEIGHT * 2) + 2) 
    printk_lcd_xy(0, yy, 0x00ffffff, message);
    
    printk_lcd_line ++;
    printk_lcd_line %= ((854 / ((FONT_HEIGHT*2 + 2))) - 1);
#endif
}
//Mickey---
#ifndef ASUS_SHIP_BUILD
void asus_drawstring(int xx, int yy, unsigned char *s, uint32_t color)
{
    unsigned char *pixels, *pos;
    unsigned char c;
    unsigned *glyph;
    unsigned x, y, data, i, tmpData;
    int count = strlen (s);
    unsigned short screen_wd;
    unsigned short screen_ht;
    unsigned short stride;

    screen_wd = 540;
    screen_ht = 960;

    stride = screen_wd;

    if ( (xx+count*(FONT_WIDTH+1)*2)>screen_wd || (yy+FONT_HEIGHT*2)>screen_ht )
        return; //error position, return

    stride -= (FONT_WIDTH+1)*2;

    pos = __va((char *)msm_fb_resources[0].start);

    pos = pos + yy * screen_wd * 3 + xx * 3;

    while (count--) {
        c = *s;
        glyph = font5x12 + (c - 32) * 2 ;
        pixels = pos;
        data = glyph[0];
        i = 0;
        for (y = 0; y < (FONT_HEIGHT); ++y) {
            pixels+=3;
            tmpData = data;
            for (x = 0; x < FONT_WIDTH; ++x) {
                if (data & 1)
                {
                    *pixels = (unsigned char)((color & 0xFF0000)>>16);
                    pixels++;
                    *pixels = (unsigned char)((color & 0xFF00)>>8);
                    pixels++;
                    *pixels = (unsigned char)(color & 0xFF);
                    pixels++;
                    *pixels = (unsigned char)((color & 0xFF0000)>>16);
                    pixels++;
                    *pixels = (unsigned char)((color & 0xFF00)>>8);
                    pixels++;
                    *pixels = (unsigned char)(color & 0xFF);
                    pixels++;
                }
                else
                    pixels+=6;
                data >>= 1;
            }
            pixels+=3;
            if ((i%2) == 0)
                data = tmpData;
            pixels += stride*3;
            i++;
        }

        i = 0;
        data = glyph[1];
        for (y = 0; y < (FONT_HEIGHT); y++) {
            tmpData = data;
            pixels+=3;
            for (x = 0; x < FONT_WIDTH; x++) {
                if (data & 1)
                {
                    *pixels = (unsigned char)((color & 0xFF0000)>>16);
                    pixels++;
                    *pixels = (unsigned char)((color & 0xFF00)>>8);
                    pixels++;
                    *pixels = (unsigned char)(color & 0xFF);
                    pixels++;
                    *pixels = (unsigned char)((color & 0xFF0000)>>16);
                    pixels++;
                    *pixels = (unsigned char)((color & 0xFF00)>>8);
                    pixels++;
                    *pixels = (unsigned char)(color & 0xFF);
                    pixels++;
                }
                else
                    pixels+=6;
                data >>= 1;
            }
            pixels+=3;
            if ((i%2) == 0)
                data = tmpData;
            pixels += stride*3;
            i++;
        }

        pos = pos + (FONT_WIDTH+1)*2 * 3;
        s++;
    }
}

void asus_clear_screen(void)
{
    unsigned char *pos;
    unsigned short screen_wd;
    unsigned short screen_ht;

    screen_wd = 540;
    screen_ht = 960;
    pos = __va((char *)msm_fb_resources[0].start);
    memset(pos,(char)0x00,screen_wd*screen_ht*3);
}
#endif
//Mickey---
void __init msm8960_init_fb(void)
{
	platform_device_register(&msm_fb_device);

#ifdef CONFIG_FB_MSM_WRITEBACK_MSM_PANEL
	platform_device_register(&wfd_panel_device);
	platform_device_register(&wfd_device);
#endif
#if 0   //QCT default// +++ ASUS_BSP : miniporting
	if (machine_is_msm8960_sim())
		platform_device_register(&mipi_dsi_simulator_panel_device);

	if (machine_is_msm8960_rumi3())
		platform_device_register(&mipi_dsi_renesas_panel_device);

	if (!machine_is_msm8960_sim() && !machine_is_msm8960_rumi3()) {
		platform_device_register(&mipi_dsi_novatek_panel_device);
#endif// --- ASUS_BSP : miniporting
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL
		platform_device_register(&hdmi_msm_device);
#endif
#if 0   //QCT default// +++ ASUS_BSP : miniporting
	}

	if (machine_is_msm8960_liquid())
		platform_device_register(&mipi_dsi2lvds_bridge_device);
	else
		platform_device_register(&mipi_dsi_toshiba_panel_device);
#else
    platform_device_register(&mipi_dsi_novatek_panel_device);
#endif// --- ASUS_BSP : miniporting
	if (machine_is_msm8x60_rumi3()) {
		msm_fb_register_device("mdp", NULL);
		mipi_dsi_pdata.target_type = 1;
	} else
		msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("mipi_dsi", &mipi_dsi_pdata);
#ifdef CONFIG_MSM_BUS_SCALING
	msm_fb_register_device("dtv", &dtv_pdata);
#endif
}

void __init msm8960_allocate_fb_region(void)
{
	void *addr;
	unsigned long size;

	size = MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
			size, addr, __pa(addr));
}

#ifndef CONFIG_FB_MSM_MIPI_PANEL_DETECT
/**
 * Set MDP clocks to high frequency to avoid DSI underflow
 * when using high resolution 1200x1920 WUXGA panels
 */
static void set_mdp_clocks_for_wuxga(void)
{
	mdp_ui_vectors[0].ab = 2000000000;
	mdp_ui_vectors[0].ib = 2000000000;
	mdp_vga_vectors[0].ab = 2000000000;
	mdp_vga_vectors[0].ib = 2000000000;
	mdp_720p_vectors[0].ab = 2000000000;
	mdp_720p_vectors[0].ib = 2000000000;
	mdp_1080p_vectors[0].ab = 2000000000;
	mdp_1080p_vectors[0].ib = 2000000000;

	if (hdmi_is_primary) {
		dtv_bus_def_vectors[0].ab = 2000000000;
		dtv_bus_def_vectors[0].ib = 2000000000;
	}
}
#endif

void __init msm8960_set_display_params(char *prim_panel, char *ext_panel)
{
	int disable_splash = 0;
	if (strnlen(prim_panel, PANEL_NAME_MAX_LEN)) {
		strlcpy(msm_fb_pdata.prim_panel_name, prim_panel,
			PANEL_NAME_MAX_LEN);
		pr_debug("msm_fb_pdata.prim_panel_name %s\n",
			msm_fb_pdata.prim_panel_name);

		if (strncmp((char *)msm_fb_pdata.prim_panel_name,
			MIPI_VIDEO_TOSHIBA_WSVGA_PANEL_NAME,
			strnlen(MIPI_VIDEO_TOSHIBA_WSVGA_PANEL_NAME,
				PANEL_NAME_MAX_LEN))) {
			/* Disable splash for panels other than Toshiba WSVGA */
			disable_splash = 1;
		}

		if (!strncmp((char *)msm_fb_pdata.prim_panel_name,
			HDMI_PANEL_NAME, strnlen(HDMI_PANEL_NAME,
				PANEL_NAME_MAX_LEN))) {
			pr_debug("HDMI is the primary display by"
				" boot parameter\n");
			hdmi_is_primary = 1;
			set_mdp_clocks_for_wuxga();
		}
		if (!strncmp((char *)msm_fb_pdata.prim_panel_name,
				MIPI_VIDEO_TOSHIBA_WUXGA_PANEL_NAME,
				strnlen(MIPI_VIDEO_TOSHIBA_WUXGA_PANEL_NAME,
					PANEL_NAME_MAX_LEN))) {
			set_mdp_clocks_for_wuxga();
		}
	}
	if (strnlen(ext_panel, PANEL_NAME_MAX_LEN)) {
		strlcpy(msm_fb_pdata.ext_panel_name, ext_panel,
			PANEL_NAME_MAX_LEN);
		pr_debug("msm_fb_pdata.ext_panel_name %s\n",
			msm_fb_pdata.ext_panel_name);
	}

	if (disable_splash)
		mdp_pdata.cont_splash_enabled = 0;
}
