/* Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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

#ifndef __LINUX_REVERSE_H__
#define __LINUX_REVERSE_H__

#include <linux/switch.h>

#define REVERSE_MAX_GPIO 2

struct reverse_switch_platform_data {
	const char	*name;
	unsigned	gpio[REVERSE_MAX_GPIO];
	unsigned int	key_code;
	unsigned	debounce_time;	/* ms */
	int		active_low[REVERSE_MAX_GPIO];

	/* if NULL, switch_dev.name will be printed */
	const char *name_on;
	const char *name_off;
	/* if NULL, "0" or "1" will be printed */
	const char *state_on;
	const char *state_off;
};

extern int switch_dev_register(struct switch_dev *sdev);

enum reverse_level {
	REVERSE_CAMERA,
	REVERSE_DISPLAY_1,
	REVERSE_DISPLAY_2,
};

struct reverse_struct {
	int level;
	int (*enter_handler) (struct reverse_struct *h);
	int (*exit_handler) (struct reverse_struct *h);
};

extern int camera_preview_ready;
extern int camera_preview_exit;

#ifndef CONFIG_FB_MSM_MDP_ARB
int disable_camera_preview(void);
int enable_camera_preview(void);
#endif
void show_pic(void);
void shutdown_pic(void);
void pic_update(unsigned char *pic, unsigned int pos_x, unsigned int pos_y,
		unsigned int image_w, unsigned int image_h);
int recovery_splash_logo(void);
void pic_pan_display(void);

#endif /* __LINUX_REVERSE_H__ */
