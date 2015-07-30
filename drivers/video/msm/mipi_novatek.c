/* Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
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

#ifdef CONFIG_SPI_QUP
#include <linux/spi/spi.h>
#endif
#include <linux/leds.h>
#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_novatek.h"
#include "mdp4.h"


static struct mipi_dsi_panel_platform_data *mipi_novatek_pdata;

static struct dsi_buf novatek_tx_buf;
static struct dsi_buf novatek_rx_buf;
static int mipi_novatek_lcd_init(void);

static int wled_trigger_initialized;

#define MIPI_DSI_NOVATEK_SPI_DEVICE_NAME	"dsi_novatek_3d_panel_spi"
#define HPCI_FPGA_READ_CMD	0x84
#define HPCI_FPGA_WRITE_CMD	0x04

#ifdef CONFIG_SPI_QUP
static struct spi_device *panel_3d_spi_client;

static void novatek_fpga_write(uint8 addr, uint16 value)
{
	char tx_buf[32];
	int  rc;
	struct spi_message  m;
	struct spi_transfer t;
	u8 data[4] = {0x0, 0x0, 0x0, 0x0};

	if (!panel_3d_spi_client) {
		pr_err("%s panel_3d_spi_client is NULL\n", __func__);
		return;
	}
	data[0] = HPCI_FPGA_WRITE_CMD;
	data[1] = addr;
	data[2] = ((value >> 8) & 0xFF);
	data[3] = (value & 0xFF);

	memset(&t, 0, sizeof t);
	memset(tx_buf, 0, sizeof tx_buf);
	t.tx_buf = data;
	t.len = 4;
	spi_setup(panel_3d_spi_client);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	rc = spi_sync(panel_3d_spi_client, &m);
	if (rc)
		pr_err("%s: SPI transfer failed\n", __func__);

	return;
}

static void novatek_fpga_read(uint8 addr)
{
	char tx_buf[32];
	int  rc;
	struct spi_message  m;
	struct spi_transfer t;
	struct spi_transfer rx;
	char rx_value[2];
	u8 data[4] = {0x0, 0x0};

	if (!panel_3d_spi_client) {
		pr_err("%s panel_3d_spi_client is NULL\n", __func__);
		return;
	}

	data[0] = HPCI_FPGA_READ_CMD;
	data[1] = addr;

	memset(&t, 0, sizeof t);
	memset(tx_buf, 0, sizeof tx_buf);
	memset(&rx, 0, sizeof rx);
	memset(rx_value, 0, sizeof rx_value);
	t.tx_buf = data;
	t.len = 2;
	rx.rx_buf = rx_value;
	rx.len = 2;
	spi_setup(panel_3d_spi_client);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	spi_message_add_tail(&rx, &m);

	rc = spi_sync(panel_3d_spi_client, &m);
	if (rc)
		pr_err("%s: SPI transfer failed\n", __func__);
	else
		pr_info("%s: rx_value = 0x%x, 0x%x\n", __func__,
						rx_value[0], rx_value[1]);

	return;
}

static int __devinit panel_3d_spi_probe(struct spi_device *spi)
{
	panel_3d_spi_client = spi;
	return 0;
}
static int __devexit panel_3d_spi_remove(struct spi_device *spi)
{
	panel_3d_spi_client = NULL;
	return 0;
}
static struct spi_driver panel_3d_spi_driver = {
	.probe         = panel_3d_spi_probe,
	.remove        = __devexit_p(panel_3d_spi_remove),
	.driver		   = {
		.name = "dsi_novatek_3d_panel_spi",
		.owner  = THIS_MODULE,
	}
};

#else

static void novatek_fpga_write(uint8 addr, uint16 value)
{
	return;
}

static void novatek_fpga_read(uint8 addr)
{
	return;
}

#endif


/* novatek blue panel */

#ifdef NOVETAK_COMMANDS_UNUSED
static char display_config_cmd_mode1[] = {
	/* TYPE_DCS_LWRITE */
	0x2A, 0x00, 0x00, 0x01,
	0x3F, 0xFF, 0xFF, 0xFF
};

static char display_config_cmd_mode2[] = {
	/* DTYPE_DCS_LWRITE */
	0x2B, 0x00, 0x00, 0x01,
	0xDF, 0xFF, 0xFF, 0xFF
};

static char display_config_cmd_mode3_666[] = {
	/* DTYPE_DCS_WRITE1 */
	0x3A, 0x66, 0x15, 0x80 /* 666 Packed (18-bits) */
};

static char display_config_cmd_mode3_565[] = {
	/* DTYPE_DCS_WRITE1 */
	0x3A, 0x55, 0x15, 0x80 /* 565 mode */
};

static char display_config_321[] = {
	/* DTYPE_DCS_WRITE1 */
	0x66, 0x2e, 0x15, 0x00 /* Reg 0x66 : 2E */
};

static char display_config_323[] = {
	/* DTYPE_DCS_WRITE */
	0x13, 0x00, 0x05, 0x00 /* Reg 0x13 < Set for Normal Mode> */
};

static char display_config_2lan[] = {
	/* DTYPE_DCS_WRITE */
	0x61, 0x01, 0x02, 0xff /* Reg 0x61 : 01,02 < Set for 2 Data Lane > */
};

static char display_config_exit_sleep[] = {
	/* DTYPE_DCS_WRITE */
	0x11, 0x00, 0x05, 0x80 /* Reg 0x11 < exit sleep mode> */
};

static char display_config_TE_ON[] = {
	/* DTYPE_DCS_WRITE1 */
	0x35, 0x00, 0x15, 0x80
};

static char display_config_39H[] = {
	/* DTYPE_DCS_WRITE */
	0x39, 0x00, 0x05, 0x80
};

static char display_config_set_tear_scanline[] = {
	/* DTYPE_DCS_LWRITE */
	0x44, 0x00, 0x00, 0xff
};

static char display_config_set_twolane[] = {
	/* DTYPE_DCS_WRITE1 */
	0xae, 0x03, 0x15, 0x80
};

static char display_config_set_threelane[] = {
	/* DTYPE_DCS_WRITE1 */
	0xae, 0x05, 0x15, 0x80
};

#else

static char sw_reset[2] = {0x01, 0x00}; /* DTYPE_DCS_WRITE */
static char enter_sleep[2] = {0x10, 0x00}; /* DTYPE_DCS_WRITE */
static char exit_sleep[2] = {0x11, 0x00}; /* DTYPE_DCS_WRITE */
static char display_off[2] = {0x28, 0x00}; /* DTYPE_DCS_WRITE */
static char display_on[2] = {0x29, 0x00}; /* DTYPE_DCS_WRITE */



static char rgb_888[2] = {0x3A, 0x77}; /* DTYPE_DCS_WRITE1 */

#if defined(NOVATEK_TWO_LANE)
static char set_num_of_lanes[2] = {0xae, 0x03}; /* DTYPE_DCS_WRITE1 */
#else  /* 1 lane */
static char set_num_of_lanes[2] = {0xae, 0x01}; /* DTYPE_DCS_WRITE1 */
#endif
/* commands by Novatke */
static char novatek_f4[2] = {0xf4, 0x55}; /* DTYPE_DCS_WRITE1 */
static char novatek_8c[16] = { /* DTYPE_DCS_LWRITE */
	0x8C, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x08, 0x08, 0x00, 0x30, 0xC0, 0xB7, 0x37};
static char novatek_ff[2] = {0xff, 0x55 }; /* DTYPE_DCS_WRITE1 */

static char set_width[5] = { /* DTYPE_DCS_LWRITE */
	0x2A, 0x00, 0x00, 0x02, 0x1B}; /* 540 - 1 */
static char set_height[5] = { /* DTYPE_DCS_LWRITE */
	0x2B, 0x00, 0x00, 0x03, 0xBF}; /* 960 - 1 */
#endif

static char led_pwm2[2] = {0x53, 0x24}; /* DTYPE_DCS_WRITE1 */
static char led_pwm3[2] = {0x55, 0x00}; /* DTYPE_DCS_WRITE1 */

static struct dsi_cmd_desc novatek_video_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 50,
		sizeof(sw_reset), sw_reset},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(display_on), display_on},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(set_num_of_lanes), set_num_of_lanes},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(rgb_888), rgb_888},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(led_pwm2), led_pwm2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(led_pwm3), led_pwm3},
};

static struct dsi_cmd_desc novatek_cmd_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 50,
		sizeof(sw_reset), sw_reset},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(exit_sleep), exit_sleep},
	//{DTYPE_DCS_WRITE, 1, 0, 0, 10,
	//	sizeof(display_on), display_on},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 50,
		sizeof(novatek_f4), novatek_f4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 50,
		sizeof(novatek_8c), novatek_8c},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 50,
		sizeof(novatek_ff), novatek_ff},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(set_num_of_lanes), set_num_of_lanes},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 50,
		sizeof(set_width), set_width},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 50,
		sizeof(set_height), set_height},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(rgb_888), rgb_888},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(led_pwm2), led_pwm2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(led_pwm3), led_pwm3},
};

static struct dsi_cmd_desc novatek_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(enter_sleep), enter_sleep}
};

// +++ ASUS_BSP : miniporting : Add for A66 Proj -Samsung_AMOLED_qHD
//Etc condition set 1
static char ss_etc_set1_f0[3] = {0xF0,0x5A,0x5A};
static char ss_etc_set1_f1[3] = {0xF1,0x5A,0x5A};
static char ss_etc_set1_fc[3] = {0xFC,0x5A,0x5A};
//Gamma condition set
//Louis
static char (*ss_brightness_set)[26];

static char ss_gamma_setting[26] ={
    0xFA,0x02,0x4A,0x1D,0x57,
    0xB5,0xD4,0x98,0xB6,0xD7,
    0xBB,0xC8,0xD9,0xC6,0xA0,
    0xB5,0x96,0xB9,0xC7,0xB2,
    0x00,0xAC,0x00,0xA3,0x00,
    0xC8};
static char ss_gamma_update[2] ={0xFA,0x03};
//Panel condition set
static char ss_panel_set[14] ={
    0xF8,0x27,0x27,0x08,0x08,
    0x4E,0xAA,0x5E,0x8A,0x10,
    0x3F,0x10,0x10,0x00};
//Etc condition set 2
static char ss_etc_set2_f6_1[4] = {0xF6,0x00,0x84,0x09};
static char ss_etc_set2_b0_1[2] = {0xB0,0x09};
static char ss_etc_set2_d5_1[2] = {0xD5,0x64};
static char ss_etc_set2_b0_2[2] = {0xB0,0x0B};
static char ss_etc_set2_d5_2[4] = {0xD5,0xA4,0x7E,0x20};
static char ss_etc_set2_f7_1[2] = {0xF7,0x03};
static char ss_etc_set2_b0_3[2] = {0xB0,0x02};
static char ss_etc_set2_b3_1[2] = {0xB3,0xC3};
static char ss_etc_set2_b0_4[2] = {0xB0,0x08};
static char ss_etc_set2_fd_1[2] = {0xFD,0xF8};
static char ss_etc_set2_b0_5[2] = {0xB0,0x04};
static char ss_etc_set2_f2_1[2] = {0xF2,0x4D};
static char ss_etc_set2_b0_6[2] = {0xB0,0x05};
static char ss_etc_set2_fd_2[2] = {0xFD,0x1F};
static char ss_etc_set2_b1_1[4] = {0xB1,0x01,0x00,0x16};
static char ss_etc_set2_b2_1[5] = {0xB2,0x08,0x08,0x08,0x08};
//Memory window set
//static char ss_memory_window_set_35[2] = {0x35,0x00};
//static char ss_memory_window_set_36[2] = {0x36,0xC0};//Mickey add for upside down
static char ss_memory_window_set_2a[5] = {0x2A,0x00,0x1E,0x02,0x39};
static char ss_memory_window_set_2b[5] = {0x2B,0x00,0x00,0x03,0xBF};
static char ss_memory_window_set_d1[2] = {0xD1,0x8A};
//Mickey+++ add for ACL setting
static char ss_acl_set_60[29] ={
    0xC1,0x47,0x53,0x13,0x53,
    0x00,0x00,0x01,0xDF,0x00,
    0x00,0x03,0x1F,0x00,0x00,
    0x00,0x00,0x00,0x01,0x04,
    0x09,0x0F,0x16,0x1F,0x28,
    0x32,0x3D,0x3D,0x00};

//static char ss_acl_on[2] = {0xC0,0x01};
//static char ss_acl_off[2] = {0xC0,0x00};
//Mickey---
static struct dsi_cmd_desc samsung_qhd_on_cmds[] = {
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ss_etc_set1_f0), ss_etc_set1_f0},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ss_etc_set1_f1), ss_etc_set1_f1},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ss_etc_set1_fc), ss_etc_set1_fc},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ss_gamma_setting), ss_gamma_setting},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_gamma_update), ss_gamma_update},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ss_panel_set), ss_panel_set},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ss_etc_set2_f6_1), ss_etc_set2_f6_1},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_etc_set2_b0_1), ss_etc_set2_b0_1},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_etc_set2_d5_1), ss_etc_set2_d5_1},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_etc_set2_b0_2), ss_etc_set2_b0_2},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ss_etc_set2_d5_2), ss_etc_set2_d5_2},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_etc_set2_f7_1), ss_etc_set2_f7_1},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_etc_set2_b0_3), ss_etc_set2_b0_3},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_etc_set2_b3_1), ss_etc_set2_b3_1},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_etc_set2_b0_4), ss_etc_set2_b0_4},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_etc_set2_fd_1), ss_etc_set2_fd_1},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_etc_set2_b0_5), ss_etc_set2_b0_5},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_etc_set2_f2_1), ss_etc_set2_f2_1},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_etc_set2_b0_6), ss_etc_set2_b0_6},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_etc_set2_fd_2), ss_etc_set2_fd_2},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ss_etc_set2_b1_1), ss_etc_set2_b1_1},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ss_etc_set2_b2_1), ss_etc_set2_b2_1},
    {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(exit_sleep), exit_sleep},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ss_memory_window_set_2a), ss_memory_window_set_2a},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ss_memory_window_set_2b), ss_memory_window_set_2b},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(ss_memory_window_set_d1), ss_memory_window_set_d1},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ss_acl_set_60), ss_acl_set_60},
//    {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(display_on), display_on},
};

static struct dsi_cmd_desc samsung_display_on[] = {
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(ss_gamma_setting), ss_gamma_setting},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_gamma_update), ss_gamma_update},
    {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(display_on), display_on},
};

//static struct dsi_cmd_desc samsung_acl_on_cmds[] = {
//    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_acl_on), ss_acl_on},
//    };
//static struct dsi_cmd_desc samsung_acl_off_cmds[] = {
//    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_acl_off), ss_acl_off},
//    };
//Mickey+++
static char ss_brightness_set_id3[25][26] = {   //60~300 nits
//ASUS_BSP joe1_++: add the 60~40 nit of brightness setting
#if 0
{0xFA,0x02,0x1F,0x00,0x51,0xCF,0xDC,0x7C,0xC6,0xD8,0x94,0xD3,0xE8,0xC5,0xB0,0xCA,0x9E,0xCA,0xD8,0xBC,0x00,0x70,0x00,0x6B,0x00,0x80},
{0xFA,0x02,0x27,0x00,0x53,0xD1,0xE0,0x80,0xC4,0xDA,0x97,0xD4,0xE7,0xCB,0xAF,0xC9,0x9C,0xC9,0xD6,0xBA,0x00,0x77,0x00,0x72,0x00,0x89},
#endif
{0xFA,0x02,0x2F,0x00,0x55,0xD0,0xE1,0x7F,0xC2,0xD9,0xAC,0xD3,0xE6,0xCB,0xAF,0xC7,0x9C,0xC5,0xD5,0xB6,0x00,0x80,0x00,0x79,0x00,0x93},
//ASUS_BSP joe1_--
{0xFA,0x02,0x38,0x00,0x56,0xC9,0xDD,0x94,0xC2,0xD9,0xAC,0xD3,0xE6,0xCE,0xAE,0xC5,0x9D,0xC6,0xD4,0xB9,0x00,0x8A,0x00,0x82,0x00,0x9F},
{0xFA,0x02,0x3C,0x10,0x57,0xC6,0xD3,0x90,0xC0,0xD7,0xB1,0xD0,0xE2,0xCA,0xAB,0xBF,0x9B,0xC4,0xCF,0xB8,0x00,0x90,0x00,0x88,0x00,0xA6},
{0xFA,0x02,0x43,0x00,0x57,0xBD,0xE1,0x91,0xBB,0xDC,0xB3,0xCD,0xE3,0xCA,0xA7,0xC4,0x9B,0xC0,0xD1,0xB5,0x00,0x95,0x00,0x8D,0x00,0xAD},
{0xFA,0x02,0x43,0x00,0x58,0xBD,0xE2,0x90,0xBC,0xDE,0xB6,0xCC,0xE2,0xC8,0xA6,0xC2,0x99,0xC0,0xD1,0xB4,0x00,0x9A,0x00,0x92,0x00,0xB3},
{0xFA,0x02,0x47,0x00,0x57,0xB9,0xE3,0x93,0xBA,0xDE,0xB9,0xC9,0xE2,0xC9,0xA4,0xC1,0x98,0xBC,0xD0,0xB3,0x00,0x9F,0x00,0x96,0x00,0xB9},
{0xFA,0x02,0x48,0x00,0x57,0xB7,0xE5,0x95,0xB9,0xDE,0xBA,0xCA,0xE2,0xC8,0xA2,0xC0,0x97,0xBE,0xCE,0xB5,0x00,0xA1,0x00,0x9B,0x00,0xBC},
{0xFA,0x02,0x4A,0x19,0x58,0xB6,0xD7,0x95,0xB7,0xD8,0xBA,0xC8,0xDB,0xC6,0x9F,0xB7,0x96,0xBA,0xC9,0xB4,0x00,0xAA,0x00,0xA0,0x00,0xC3},
{0xFA,0x02,0x4A,0x1D,0x57,0xB5,0xD4,0x98,0xB6,0xD7,0xBB,0xC8,0xD9,0xC6,0xA0,0xB5,0x96,0xB9,0xC7,0xB2,0x00,0xAC,0x00,0xA3,0x00,0xC8},
{0xFA,0x02,0x4C,0x1F,0x57,0xB3,0xD5,0x9C,0xB4,0xD6,0xBB,0xC7,0xD8,0xC6,0x9E,0xB4,0x96,0xB8,0xC5,0xAF,0x00,0xB0,0x00,0xA7,0x00,0xCE},
{0xFA,0x02,0x4C,0x26,0x58,0xB5,0xD0,0x99,0xB5,0xD3,0xBB,0xC7,0xD6,0xC5,0x9C,0xB0,0x94,0xB9,0xC3,0xB0,0x00,0xB4,0x00,0xAB,0x00,0xD2},
{0xFA,0x02,0x4D,0x2A,0x58,0xB3,0xCE,0x9C,0xB4,0xD2,0xBB,0xC6,0xD4,0xC4,0x9B,0xAD,0x93,0xB9,0xC1,0xB1,0x00,0xB7,0x00,0xAF,0x00,0xD6},
{0xFA,0x02,0x4F,0x28,0x57,0xB1,0xD0,0x9D,0xB2,0xD3,0xBB,0xC5,0xD5,0xC5,0x9B,0xAD,0x94,0xB5,0xC1,0xAD,0x00,0xBB,0x00,0xB2,0x00,0xDC},
{0xFA,0x02,0x50,0x2E,0x58,0xB0,0xCD,0x9D,0xB2,0xD0,0xBB,0xC4,0xD2,0xC3,0x9A,0xA9,0x92,0xB4,0xC0,0xAE,0x00,0xBF,0x00,0xB6,0x00,0xE0},
{0xFA,0x02,0x4F,0x31,0x57,0xB2,0xCB,0xA0,0xB3,0xCE,0xBB,0xC4,0xD1,0xC4,0x9B,0xA9,0x92,0xB3,0xBC,0xAD,0x00,0xC3,0x00,0xBA,0x00,0xE5},
{0xFA,0x02,0x4F,0x34,0x58,0xB2,0xC9,0x9F,0xB3,0xCC,0xBA,0xC4,0xCF,0xC3,0x9A,0xA7,0x91,0xB3,0xBB,0xAC,0x00,0xC6,0x00,0xBD,0x00,0xE9},
{0xFA,0x02,0x4F,0x38,0x57,0xB3,0xC6,0xA3,0xB4,0xCA,0xBB,0xC4,0xCE,0xC3,0x99,0xA4,0x92,0xB3,0xB9,0xAB,0x00,0xC9,0x00,0xC0,0x00,0xED},
{0xFA,0x02,0x51,0x38,0x58,0xB0,0xC8,0xA3,0xB1,0xC9,0xB9,0xC3,0xCE,0xC3,0x98,0xA4,0x90,0xB1,0xB8,0xAB,0x00,0xCD,0x00,0xC4,0x00,0xF1},
{0xFA,0x02,0x52,0x39,0x58,0xAF,0xC7,0xA3,0xB1,0xC8,0xBA,0xC3,0xCD,0xC3,0x97,0xA3,0x8F,0xB1,0xB7,0xAB,0x00,0xCF,0x00,0xC7,0x00,0xF4},
{0xFA,0x02,0x53,0x3A,0x57,0xAE,0xC7,0xA7,0xB1,0xC8,0xBA,0xC2,0xCD,0xC2,0x95,0xA1,0x90,0xB0,0xB7,0xA9,0x00,0xD3,0x00,0xCA,0x00,0xFA},
{0xFA,0x02,0x52,0x3C,0x58,0xB0,0xC6,0xA6,0xB1,0xC7,0xB9,0xC2,0xCB,0xC1,0x96,0xA0,0x8F,0xB0,0xB6,0xA9,0x00,0xD6,0x00,0xCD,0x00,0xFD},
{0xFA,0x02,0x53,0x3D,0x58,0xAE,0xC4,0xA6,0xB1,0xC5,0xB9,0xC2,0xCB,0xC1,0x95,0xA0,0x8E,0xAE,0xB4,0xA8,0x00,0xD9,0x00,0xD0,0x01,0x01},
{0xFA,0x02,0x55,0x3E,0x58,0xAC,0xC4,0xA7,0xAF,0xC5,0xB8,0xC1,0xCB,0xC1,0x94,0x9F,0x8E,0xAC,0xB2,0xA7,0x00,0xDD,0x00,0xD4,0x01,0x06},
{0xFA,0x02,0x53,0x41,0x5A,0xB0,0xC4,0xA4,0xB0,0xC2,0xB6,0xC2,0xCA,0xC1,0x94,0x9C,0x8C,0xAE,0xB2,0xA5,0x00,0xDF,0x00,0xD6,0x01,0x0A},
{0xFA,0x02,0x51,0x39,0x55,0xB0,0xC7,0xA0,0xB0,0xC5,0xB8,0xC2,0xCB,0xC1,0x94,0xA0,0x8F,0xAD,0xB3,0xA6,0x00,0xE0,0x00,0xD7,0x01,0x08}
};

static char ss_brightness_set_id5[25][26] = {   // 60~300 nits
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xD1, 0x34, 0xD0, 0xD1, 0xCA, 0xD1, 0xDB, 0xDA, 0xC8, 0xB5,
 0xB8, 0xB0, 0xC5, 0xC8, 0xBF, 0x00, 0x6E, 0x00, 0x54, 0x00, 0x7C},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xD1, 0x34, 0xD0, 0xD6, 0xBA, 0xDC, 0xE0, 0xD9, 0xE2, 0xC2,
 0xC0, 0xBF, 0xD4, 0xD5, 0xD0, 0x00, 0x73, 0x00, 0x59, 0x00, 0x82},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xD7, 0x39, 0xD6, 0xD6, 0xBF, 0xDD, 0xE1, 0xDA, 0xE2, 0xC0,
 0xBF, 0xBD, 0xD3, 0xD5, 0xCF, 0x00, 0x78, 0x00, 0x5D, 0x00, 0x88},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xD7, 0x39, 0xD5, 0xD5, 0xBF, 0xDC, 0xDF, 0xDA, 0xE0, 0xC1,
 0xC0, 0xBD, 0xD2, 0xD4, 0xCF, 0x00, 0x7C, 0x00, 0x60, 0x00, 0x8C},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xDD, 0x3A, 0xE3, 0xD7, 0xC5, 0xDD, 0xDF, 0xDA, 0xDF, 0xC0,
 0xBF, 0xBC, 0xD0, 0xD3, 0xCD, 0x00, 0x81, 0x00, 0x64, 0x00, 0x92},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xE1, 0x43, 0xE2, 0xD6, 0xC5, 0xDC, 0xDE, 0xDA, 0xDF, 0xBF,
 0xBF, 0xBB, 0xD0, 0xD3, 0xCD, 0x00, 0x85, 0x00, 0x67, 0x00, 0x96},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xE5, 0x48, 0xE4, 0xD5, 0xC5, 0xDB, 0xDE, 0xDA, 0xDD, 0xBE,
 0xBF, 0xBB, 0xD0, 0xD2, 0xCC, 0x00, 0x88, 0x00, 0x6A, 0x00, 0x9A},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xE6, 0x4D, 0xE3, 0xD5, 0xC5, 0xDA, 0xDD, 0xDA, 0xDD, 0xBE,
 0xBE, 0xBA, 0xCE, 0xD1, 0xCA, 0x00, 0x8C, 0x00, 0x6D, 0x00, 0x9F},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xE8, 0x51, 0xE4, 0xD6, 0xC9, 0xDB, 0xDC, 0xD9, 0xDC, 0xBE,
 0xBF, 0xBA, 0xCD, 0xD0, 0xC9, 0x00, 0x90, 0x00, 0x70, 0x00, 0xA3},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEA, 0x57, 0xE9, 0xD6, 0xC9, 0xDA, 0xDD, 0xDA, 0xDC, 0xBC,
 0xBE, 0xB8, 0xCE, 0xD0, 0xCA, 0x00, 0x92, 0x00, 0x72, 0x00, 0xA6},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xF0, 0x61, 0xEE, 0xD5, 0xC9, 0xD9, 0xDD, 0xDA, 0xDB, 0xBC,
 0xBE, 0xB9, 0xCC, 0xCF, 0xC8, 0x00, 0x96, 0x00, 0x75, 0x00, 0xAA},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xF1, 0x81, 0xEE, 0xD4, 0xC9, 0xD9, 0xDD, 0xDB, 0xDB, 0xBB,
 0xBD, 0xB7, 0xCC, 0xCE, 0xC8, 0x00, 0x99, 0x00, 0x78, 0x00, 0xAE},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xF0, 0x8C, 0xED, 0xD5, 0xCA, 0xD8, 0xDC, 0xDB, 0xDC, 0xBB,
 0xBD, 0xB7, 0xCB, 0xCD, 0xC5, 0x00, 0x9C, 0x00, 0x7A, 0x00, 0xB2},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xED, 0x90, 0xEC, 0xD4, 0xC9, 0xD7, 0xDD, 0xDB, 0xDB, 0xBA,
 0xBD, 0xB7, 0xCA, 0xCD, 0xC5, 0x00, 0x9F, 0x00, 0x7C, 0x00, 0xB5},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xED, 0x8F, 0xED, 0xD2, 0xC7, 0xD5, 0xDC, 0xDA, 0xDA, 0xBA,
 0xBB, 0xB5, 0xCA, 0xCC, 0xC4, 0x00, 0xA0, 0x00, 0x7F, 0x00, 0xBB},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEF, 0x96, 0xEF, 0xD1, 0xC7, 0xD3, 0xDB, 0xD9, 0xD9, 0xB9,
 0xBB, 0xB4, 0xCA, 0xCC, 0xC6, 0x00, 0xA3, 0x00, 0x81, 0x00, 0xBD},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xED, 0x99, 0xEE, 0xD3, 0xC8, 0xD3, 0xDB, 0xD9, 0xD9, 0xB8,
 0xBB, 0xB4, 0xC9, 0xCC, 0xC4, 0x00, 0xA6, 0x00, 0x83, 0x00, 0xC1},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEE, 0xA3, 0xEF, 0xD2, 0xC9, 0xD3, 0xDB, 0xD9, 0xD9, 0xB9,
 0xBA, 0xB3, 0xC8, 0xCC, 0xC4, 0x00, 0xA8, 0x00, 0x85, 0x00, 0xC4},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xED, 0xA4, 0xEE, 0xD2, 0xC9, 0xD3, 0xDC, 0xD9, 0xD9, 0xB7,
 0xBA, 0xB3, 0xC8, 0xCB, 0xC2, 0x00, 0xAB, 0x00, 0x87, 0x00, 0xC8},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xED, 0xA8, 0xEF, 0xD2, 0xC8, 0xD2, 0xDA, 0xD9, 0xD8, 0xB7,
 0xB9, 0xB2, 0xC8, 0xCB, 0xC2, 0x00, 0xAD, 0x00, 0x89, 0x00, 0xCB},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEB, 0xB2, 0xEF, 0xD2, 0xC9, 0xD3, 0xDB, 0xDA, 0xD9, 0xB6,
 0xB9, 0xBA, 0xC7, 0xCA, 0xC1, 0x00, 0xB0, 0x00, 0x8B, 0x00, 0xCE},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB0, 0xEF, 0xD1, 0xC9, 0xD2, 0xDB, 0xDA, 0xD9, 0xB7,
 0xB9, 0xB1, 0xC6, 0xC9, 0xC0, 0x00, 0xB2, 0x00, 0x8D, 0x00, 0xD1},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xED, 0xB6, 0xF0, 0xD1, 0xC9, 0xD2, 0xDB, 0xDA, 0xD9, 0xB6,
 0xB8, 0xAF, 0xC7, 0xCA, 0xC2, 0x00, 0xB4, 0x00, 0x8F, 0x00, 0xD3},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB7, 0xEE, 0xD0, 0xC8, 0xD1, 0xDB, 0xDA, 0xD9, 0xB6,
 0xB8, 0xB0, 0xC5, 0xC9, 0xC1, 0x00, 0xB7, 0x00, 0x91, 0x00, 0xD5},
{0xFA, 0x02, 0x10, 0x10, 0x10, 0xEC, 0xB7, 0xEF, 0xD1, 0xCA, 0xD1, 0xDB, 0xDA, 0xD8, 0xB5,
 0xB8, 0xB0, 0xC5, 0xC8, 0xBF, 0x00, 0xB9, 0x00, 0x93, 0x00, 0xD9},
};

struct msm_fb_data_type *g_mfd=NULL;

static char manufacture_id[2] = {0x04, 0x00}; /* DTYPE_DCS_READ */

static struct dsi_cmd_desc novatek_manufacture_id_cmd = {
	DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id), manufacture_id};

struct dcs_cmd_req cmdreq;
static u32 manu_id;

static void mipi_novatek_manufature_cb(u32 data)
{
	manu_id = data & 0xff; // 0xff for samsung, 0xffffff for novatek
	pr_info("%s: manufature_id=%x\n", __func__, manu_id);
}

static uint32 mipi_novatek_manufacture_id(struct msm_fb_data_type *mfd)
{
	cmdreq.cmds = &novatek_manufacture_id_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = 3;
	cmdreq.cb = mipi_novatek_manufature_cb;
	mipi_dsi_cmdlist_put(&cmdreq);

	return manu_id;
}

static uint32 mipi_samsung_panel_id(struct msm_fb_data_type *mfd)
{
	char amoled_id[] = {0xDB};
	struct dsi_cmd_desc samsung_amoled_id_cmd = {
   		DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(amoled_id), amoled_id};

	cmdreq.cmds = &samsung_amoled_id_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = 1;
	cmdreq.cb = mipi_novatek_manufature_cb;
	mipi_dsi_cmdlist_put(&cmdreq);

	return manu_id;
}

static int fpga_addr;
static int fpga_access_mode;
static bool support_3d;

static void mipi_novatek_3d_init(int addr, int mode)
{
	fpga_addr = addr;
	fpga_access_mode = mode;
}

static void mipi_dsi_enable_3d_barrier(int mode)
{
	void __iomem *fpga_ptr;
	uint32_t ptr_value = 0;

	if (!fpga_addr && support_3d) {
		pr_err("%s: fpga_addr not set. Failed to enable 3D barrier\n",
					__func__);
		return;
	}

	if (fpga_access_mode == FPGA_SPI_INTF) {
		if (mode == LANDSCAPE)
			novatek_fpga_write(fpga_addr, 1);
		else if (mode == PORTRAIT)
			novatek_fpga_write(fpga_addr, 3);
		else
			novatek_fpga_write(fpga_addr, 0);

		mb();
		novatek_fpga_read(fpga_addr);
	} else if (fpga_access_mode == FPGA_EBI2_INTF) {
		fpga_ptr = ioremap_nocache(fpga_addr, sizeof(uint32_t));
		if (!fpga_ptr) {
			pr_err("%s: FPGA ioremap failed."
				"Failed to enable 3D barrier\n",
						__func__);
			return;
		}

		ptr_value = readl_relaxed(fpga_ptr);
		if (mode == LANDSCAPE)
			writel_relaxed(((0xFFFF0000 & ptr_value) | 1),
								fpga_ptr);
		else if (mode == PORTRAIT)
			writel_relaxed(((0xFFFF0000 & ptr_value) | 3),
								fpga_ptr);
		else
			writel_relaxed((0xFFFF0000 & ptr_value),
								fpga_ptr);

		mb();
		iounmap(fpga_ptr);
	} else
		pr_err("%s: 3D barrier not configured correctly\n",
					__func__);
}

static int g_brightness_index = 0;//Mickey
bool g_displayOn = false;//Mickey+++
extern int g_fb0_dsi_block;
extern bool g_fb0_on;

static int mipi_novatek_lcd_on(struct platform_device *pdev)
{
	static bool bFirst = TRUE;
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	struct msm_panel_info *pinfo;

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	pinfo = &mfd->panel_info;
	if (pinfo->is_3d_panel)
		support_3d = TRUE;

	mipi  = &mfd->panel_info.mipi;
	
	mipi_set_tx_power_mode(1);
	// +++ ASUS_BSP : miniporting
    if (g_A60K_hwID<=A60K_SR1_2_ES2)
    {
		if (mipi->mode == DSI_VIDEO_MODE) {
			cmdreq.cmds = novatek_video_on_cmds;
			cmdreq.cmds_cnt = ARRAY_SIZE(novatek_video_on_cmds);
			cmdreq.flags = CMD_REQ_COMMIT;
			cmdreq.rlen = 0;
			cmdreq.cb = NULL;
			mipi_dsi_cmdlist_put(&cmdreq);
	    } else {
			cmdreq.cmds = novatek_cmd_on_cmds;
			cmdreq.cmds_cnt = ARRAY_SIZE(novatek_cmd_on_cmds);
			cmdreq.flags = CMD_REQ_COMMIT;
			cmdreq.rlen = 0;
			cmdreq.cb = NULL;
			mipi_dsi_cmdlist_put(&cmdreq);

			/* clean up ack_err_status */
			mipi_dsi_cmd_bta_sw_trigger();
			mipi_novatek_manufacture_id(mfd);
		}
    }
    else
    {
		if(bFirst){
			if (mipi_samsung_panel_id(mfd) == 5) {
				ss_brightness_set = ss_brightness_set_id5;
			}
			else {
				ss_brightness_set = ss_brightness_set_id3;
			}
			samsung_qhd_on_cmds[3].dlen = sizeof(ss_brightness_set[0]);
			samsung_qhd_on_cmds[3].payload = ss_brightness_set[0];
			bFirst = false;
		}
			cmdreq.cmds = samsung_qhd_on_cmds;
			cmdreq.cmds_cnt = ARRAY_SIZE(samsung_qhd_on_cmds);
			cmdreq.flags = CMD_REQ_COMMIT;
			cmdreq.rlen = 0;
			cmdreq.cb = NULL;
			mipi_dsi_cmdlist_put(&cmdreq);
    }
    mipi_set_tx_power_mode(0);
    printk("[Display] panel power on\n");
	// --- ASUS_BSP : miniporting
	return 0;
}

static int mipi_novatek_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;
	
	mipi_set_tx_power_mode(1);
	cmdreq.cmds = novatek_display_off_cmds;
	cmdreq.cmds_cnt = ARRAY_SIZE(novatek_display_off_cmds);
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mipi_dsi_cmdlist_put(&cmdreq);
	mipi_set_tx_power_mode(0);
	
	g_displayOn = false;
	
	printk("[Display] panel off\n");

	return 0;
}

DEFINE_LED_TRIGGER(bkl_led_trigger);

static char led_pwm1[2] = {0x51, 0x0};	/* DTYPE_DCS_WRITE1 */
static struct dsi_cmd_desc backlight_cmd = {
	DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(led_pwm1), led_pwm1};


static void mipi_novatek_set_backlight(struct msm_fb_data_type *mfd)
{
	int value = mfd->bl_level;
	int index = -1;
	
	if (value == 0 || value == 1000 || value == 2000){
		// TODO: close screen
	}
	
	if (value < 0 || value > 2255 ||
		(value > 255 && value < 1020) ||
		(value > 1255 && value < 2020) ) {
		return ;
	}
	else if (value >= 2000) {
		// auto 0~24 >< 20~255
		value -= 2000;
		index = ((value-20) * 24 * 8192) / (235 * 8192);
	}
	else if (value >= 1000) {
		// outdoor 4~24 >< 20~255
		value -= 1000;
		index = ((value-20) * 20 * 8192) / (235 * 8192) + 4;
	}
	else {
		// normal 0~13 >< 20~255
		index = ((value-20) * 13 * 8192) / (235 * 8192);
	}
	
	if (index <= 0)
		index=0;
	else if (index >= 24)
		index=24;
   
	if(g_brightness_index != index){
		
		if(mfd->bl_level != 0)
			g_brightness_index = index;
		
		if(!g_fb0_on || mfd->asus_panel_disable) {
			printk("%s : asus_panel_disable=%d, g_fb0_on=%d\n",__func__,mfd->asus_panel_disable,g_fb0_on);
			return;
		}
		
		if (mdp4_overlay_dsi_state_get() == ST_DSI_SUSPEND || g_fb0_dsi_block) {
			printk("[Display] %s: unable to send DCS command due to ST_DSI_SUSPEND, g_fb0_dsi_block(%d)\n", __func__, g_fb0_dsi_block);
			return ;
		}
		
		/*
		struct dsi_cmd_desc amoled_brightness_cmds[2] = {
			{DTYPE_DCS_LWRITE, 1, 0, 0, 10, sizeof(ss_brightness_set[index]), ss_brightness_set[index]},
			{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(ss_gamma_update), ss_gamma_update}
		};
		*/
		mipi_set_tx_power_mode(1);
		samsung_qhd_on_cmds[3].dlen = sizeof(ss_brightness_set[index]);
		samsung_qhd_on_cmds[3].payload = ss_brightness_set[index];
		cmdreq.cmds = &samsung_qhd_on_cmds[3]; //amoled_brightness_cmds
		cmdreq.cmds_cnt = 2;
		cmdreq.flags = CMD_REQ_COMMIT;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;
		
		mipi_dsi_cmdlist_put(&cmdreq);
		mipi_set_tx_power_mode(0);
		printk("[backlight] input=%d, value=%d, index=%d\n", mfd->bl_level, value, index);
	}
	return;	// +++ ASUS_BSP : miniporting
	if ((mipi_novatek_pdata->enable_wled_bl_ctrl)
	    && (wled_trigger_initialized)) {
		led_trigger_event(bkl_led_trigger, mfd->bl_level);
		return;
	}

	led_pwm1[1] = (unsigned char)mfd->bl_level;

	cmdreq.cmds = &backlight_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);
}

void amoled_display_on(void)
{
   struct mipi_panel_info *mipi;
    mipi  = &g_mfd->panel_info.mipi;

    if (mdp4_overlay_dsi_state_get() == ST_DSI_SUSPEND) {
        printk("[Display] %s: unable to send DCS command due to ST_DSI_SUSPEND\n", __func__);
        return;
    }
    
    mipi_set_tx_power_mode(1);
    samsung_display_on[0].dlen = sizeof(ss_brightness_set[g_brightness_index]);
    samsung_display_on[0].payload = ss_brightness_set[g_brightness_index];

    cmdreq.cmds = samsung_display_on;
    cmdreq.cmds_cnt = ARRAY_SIZE(samsung_display_on);
    cmdreq.flags = CMD_REQ_COMMIT;
    cmdreq.rlen = 0;
    cmdreq.cb = NULL;
    mipi_dsi_cmdlist_put(&cmdreq);
    mipi_set_tx_power_mode(0);
    g_displayOn = true;
    printk("[Display] display on due to screen update, brightness=%d\n", g_brightness_index);
}
EXPORT_SYMBOL(amoled_display_on);

static int mipi_dsi_3d_barrier_sysfs_register(struct device *dev);
static int barrier_mode;

static int __devinit mipi_novatek_lcd_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	struct platform_device *current_pdev;
	static struct mipi_dsi_phy_ctrl *phy_settings;
	static char dlane_swap;

	if (pdev->id == 0) {
		mipi_novatek_pdata = pdev->dev.platform_data;

		if (mipi_novatek_pdata
			&& mipi_novatek_pdata->phy_ctrl_settings) {
			phy_settings = (mipi_novatek_pdata->phy_ctrl_settings);
		}

		if (mipi_novatek_pdata
			&& mipi_novatek_pdata->dlane_swap) {
			dlane_swap = (mipi_novatek_pdata->dlane_swap);
		}

		if (mipi_novatek_pdata
			 && mipi_novatek_pdata->fpga_3d_config_addr)
			mipi_novatek_3d_init(mipi_novatek_pdata
	->fpga_3d_config_addr, mipi_novatek_pdata->fpga_ctrl_mode);

		/* create sysfs to control 3D barrier for the Sharp panel */
		if (mipi_dsi_3d_barrier_sysfs_register(&pdev->dev)) {
			pr_err("%s: Failed to register 3d Barrier sysfs\n",
						__func__);
			return -ENODEV;
		}
		barrier_mode = 0;

		return 0;
	}

	current_pdev = msm_fb_add_device(pdev);

	if (current_pdev) {
		mfd = platform_get_drvdata(current_pdev);
		if (!mfd)
			return -ENODEV;
		if (mfd->key != MFD_KEY)
			return -EINVAL;

		mipi  = &mfd->panel_info.mipi;

		if (phy_settings != NULL)
			mipi->dsi_phy_db = phy_settings;

		if (dlane_swap)
			mipi->dlane_swap = dlane_swap;
			
		g_mfd = mfd;// +++ ASUS_BSP : miniporting
	}
	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_novatek_lcd_probe,
	.driver = {
		.name   = "mipi_novatek",
	},
};

static struct msm_fb_panel_data novatek_panel_data = {
	.on		= mipi_novatek_lcd_on,
	.off		= mipi_novatek_lcd_off,
	.set_backlight = mipi_novatek_set_backlight,
};

static ssize_t mipi_dsi_3d_barrier_read(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf((char *)buf, sizeof(buf), "%u\n", barrier_mode);
}

static ssize_t mipi_dsi_3d_barrier_write(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	int ret = -1;
	u32 data = 0;

	if (sscanf((char *)buf, "%u", &data) != 1) {
		dev_err(dev, "%s\n", __func__);
		ret = -EINVAL;
	} else {
		barrier_mode = data;
		if (data == 1)
			mipi_dsi_enable_3d_barrier(LANDSCAPE);
		else if (data == 2)
			mipi_dsi_enable_3d_barrier(PORTRAIT);
		else
			mipi_dsi_enable_3d_barrier(0);
	}

	return count;
}

static struct device_attribute mipi_dsi_3d_barrier_attributes[] = {
	__ATTR(enable_3d_barrier, 0664, mipi_dsi_3d_barrier_read,
					 mipi_dsi_3d_barrier_write),
};

static int mipi_dsi_3d_barrier_sysfs_register(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mipi_dsi_3d_barrier_attributes); i++)
		if (device_create_file(dev, mipi_dsi_3d_barrier_attributes + i))
			goto error;

	return 0;

error:
	for (; i >= 0 ; i--)
		device_remove_file(dev, mipi_dsi_3d_barrier_attributes + i);
	pr_err("%s: Unable to create interface\n", __func__);

	return -ENODEV;
}

static int ch_used[3];

int mipi_novatek_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	ret = mipi_novatek_lcd_init();
	if (ret) {
		pr_err("mipi_novatek_lcd_init() failed with ret %u\n", ret);
		return ret;
	}

	pdev = platform_device_alloc("mipi_novatek", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	novatek_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &novatek_panel_data,
		sizeof(novatek_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int mipi_novatek_lcd_init(void)
{
#ifdef CONFIG_SPI_QUP
	int ret;
	ret = spi_register_driver(&panel_3d_spi_driver);

	if (ret) {
		pr_err("%s: spi register failed: rc=%d\n", __func__, ret);
		platform_driver_unregister(&this_driver);
	} else
		pr_info("%s: SUCCESS (SPI)\n", __func__);
#endif

	led_trigger_register_simple("bkl_trigger", &bkl_led_trigger);
	pr_info("%s: SUCCESS (WLED TRIGGER)\n", __func__);
	wled_trigger_initialized = 1;

	mipi_dsi_buf_alloc(&novatek_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&novatek_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}
