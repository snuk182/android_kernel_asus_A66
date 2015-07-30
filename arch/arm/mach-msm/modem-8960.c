/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/stringify.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/reboot.h>

#include <mach/irqs.h>
#include <mach/scm.h>
#include <mach/peripheral-loader.h>
#include <mach/subsystem_restart.h>
#include <mach/subsystem_notif.h>
#include <mach/socinfo.h>
#include <mach/msm_smsm.h>

#include "smd_private.h"
#include "modem_notifier.h"
#include "ramdump.h"

static int crash_shutdown;

#define MAX_SSR_REASON_LEN 81U
#define Q6_FW_WDOG_ENABLE		0x08882024
#define Q6_SW_WDOG_ENABLE		0x08982024

static void log_modem_sfr(void)
{
	u32 size;
	char *smem_reason, reason[MAX_SSR_REASON_LEN];

	smem_reason = smem_get_entry(SMEM_SSR_REASON_MSS0, &size);
	if (!smem_reason || !size) {
		pr_err("modem subsystem failure reason: (unknown, smem_get_entry failed).\n");
		return;
	}
	if (!smem_reason[0]) {
		pr_err("modem subsystem failure reason: (unknown, init string found).\n");
		return;
	}

	size = min(size, MAX_SSR_REASON_LEN-1);
	memcpy(reason, smem_reason, size);
	reason[size] = '\0';
	pr_err("modem subsystem failure reason: %s.\n", reason);

	smem_reason[0] = '\0';
	wmb();
}

static void restart_modem(void)
{
	log_modem_sfr();
	subsystem_restart("modem");
}

static void modem_wdog_check(struct work_struct *work)
{
	void __iomem *q6_sw_wdog_addr;
	u32 regval;

	q6_sw_wdog_addr = ioremap_nocache(Q6_SW_WDOG_ENABLE, 4);
	if (!q6_sw_wdog_addr)
		panic("Unable to check modem watchdog status.\n");

	regval = readl_relaxed(q6_sw_wdog_addr);
	if (!regval) {
	#if 0	
		pr_err("modem-8960: Modem watchdog wasn't activated!. Restarting the modem now.\n");
		restart_modem();
	#else
		pr_err("modem-8960: Modem watchdog wasn't activated!. Reboot now !!! \n");
		kernel_restart(NULL);
	#endif
	}

	iounmap(q6_sw_wdog_addr);
}

static DECLARE_DELAYED_WORK(modem_wdog_check_work, modem_wdog_check);

static void smsm_state_cb(void *data, uint32_t old_state, uint32_t new_state)
{
	/* Ignore if we're the one that set SMSM_RESET */
	if (crash_shutdown)
		return;

	if (new_state & SMSM_RESET) {
		pr_err("Probable fatal error on the modem.\n");
		restart_modem();
	}
}

static int modem_shutdown(const struct subsys_data *subsys)
{
	void __iomem *q6_fw_wdog_addr;
	void __iomem *q6_sw_wdog_addr;

	/*
	 * Cancel any pending wdog_check work items, since we're shutting
	 * down anyway.
	 */
	cancel_delayed_work(&modem_wdog_check_work);

	/*
	 * Disable the modem watchdog since it keeps running even after the
	 * modem is shutdown.
	 */
	q6_fw_wdog_addr = ioremap_nocache(Q6_FW_WDOG_ENABLE, 4);
	if (!q6_fw_wdog_addr)
		return -ENOMEM;

	q6_sw_wdog_addr = ioremap_nocache(Q6_SW_WDOG_ENABLE, 4);
	if (!q6_sw_wdog_addr) {
		iounmap(q6_fw_wdog_addr);
		return -ENOMEM;
	}

	writel_relaxed(0x0, q6_fw_wdog_addr);
	writel_relaxed(0x0, q6_sw_wdog_addr);
	mb();
	iounmap(q6_sw_wdog_addr);
	iounmap(q6_fw_wdog_addr);

	pil_force_shutdown("modem");
	pil_force_shutdown("modem_fw");
	disable_irq_nosync(Q6FW_WDOG_EXPIRED_IRQ);
	disable_irq_nosync(Q6SW_WDOG_EXPIRED_IRQ);

	return 0;
}

#define MODEM_WDOG_CHECK_TIMEOUT_MS 10000

static int modem_powerup(const struct subsys_data *subsys)
{
	pil_force_boot("modem_fw");
	pil_force_boot("modem");
	enable_irq(Q6FW_WDOG_EXPIRED_IRQ);
	enable_irq(Q6SW_WDOG_EXPIRED_IRQ);
	schedule_delayed_work(&modem_wdog_check_work,
				msecs_to_jiffies(MODEM_WDOG_CHECK_TIMEOUT_MS));
	return 0;
}

void modem_crash_shutdown(const struct subsys_data *subsys)
{
	crash_shutdown = 1;
	smsm_reset_modem(SMSM_RESET);
}
#ifndef ASUS_SHIP_BUILD 
/* FIXME: Get address, size from PIL */
static struct ramdump_segment modemsw_segments[] = {
	{0x89000000, 0x8D400000 - 0x89000000},
};

static struct ramdump_segment modemfw_segments[] = {
	{0x8D400000, 0x8DA00000 - 0x8D400000},
};

static struct ramdump_segment smem_segments[] = {
	{0x80000000, 0x00200000},
};
#endif
static void *modemfw_ramdump_dev;
static void *modemsw_ramdump_dev;
static void *smem_ramdump_dev;
#ifndef ASUS_SHIP_BUILD
// ASUS_BSP+++ Wenli "Modify for modem restart"
#include <linux/rtc.h>
#include <linux/syscalls.h>

extern void pet_watchdog(void);
extern int asus_rtc_read_time(struct rtc_time *tm);
#define ASUSQ6SW_OFFSET 0x01BA01B0
#define ASUSQ6SW_BYTES 48
// ASUS_BSP--- Wenli "Modify for modem restart"
#endif
static int modem_ramdump(int enable,
				const struct subsys_data *crashed_subsys)
{
	int ret = 0;
// ASUS_BSP+++ Wenli "Modify for modem restart"
#ifndef ASUS_SHIP_BUILD
	if (enable) {
		int file_handle;
		int ret;
		struct rtc_time tm;
		unsigned char *ptr;
		mm_segment_t oldfs;
		char dump_dir[256];
		char messages[256];
		char dump_log[64];
		unsigned long offset = 0;
		unsigned long copy_size = 0;

		memset(dump_log, 0, sizeof(dump_log));

		asus_rtc_read_time(&tm);

		pet_watchdog();
		//ptr = ioremap_nocache(modemsw_segments[0].address,  modemsw_segments[0].size);
		//memcpy(dump_log, ptr + ASUSQ6SW_OFFSET, ASUSQ6SW_BYTES);
		ptr = ioremap_nocache(modemsw_segments[0].address + ASUSQ6SW_OFFSET,  ASUSQ6SW_BYTES);
		memcpy(dump_log, ptr, ASUSQ6SW_BYTES);
		iounmap(ptr);
		printk("%s\n", dump_log);
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		sprintf(dump_dir, "/data/log/RAMDump_%04d%02d%02d-%02d%02d%02d",
				tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		file_handle = sys_mkdir(dump_dir, 0755);
		if(IS_ERR((const void *)file_handle))
		{
			printk("mkdir %s fail\n", dump_dir);
		}
		sprintf(messages, "%s/modem_sw.bin", dump_dir);
		file_handle = sys_open(messages, O_CREAT|O_WRONLY|O_SYNC, 0644);

		if(!IS_ERR((const void *)file_handle))
		{
			printk("save_ram_dump Modem_sw+++\n");
			while(modemsw_segments[0].size > offset){
				if( modemsw_segments[0].size - offset > 1024 * 1024)
					copy_size = 1024 * 1024;
				else
					copy_size = modemsw_segments[0].size - offset;
				ptr = ioremap_nocache(modemsw_segments[0].address + offset,  copy_size);
				ret = sys_write(file_handle, (unsigned char*)ptr, copy_size);
				iounmap(ptr);
				offset += copy_size;
			}
			sys_close(file_handle);
			printk("save_ram_dump Modem_sw---\n");
		}
		set_fs(oldfs);
		

		pet_watchdog();
		ptr = ioremap_nocache(modemfw_segments[0].address, modemfw_segments[0].size);
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		sprintf(messages, "%s/modem_fw.bin", dump_dir);
		file_handle = sys_open(messages, O_CREAT|O_WRONLY|O_SYNC, 0644);

		if(!IS_ERR((const void *)file_handle))
		{
			printk("save_ram_dump Modem_fw+++\n");
			ret = sys_write(file_handle, (unsigned char*)ptr, modemfw_segments[0].size);
			sys_close(file_handle);
			printk("save_ram_dump Modem_fw---\n");
		}
		set_fs(oldfs);
		iounmap(ptr);

		pet_watchdog();
		ptr = ioremap_nocache(smem_segments[0].address, smem_segments[0].size);
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		sprintf(messages, "%s/modem_smem.bin", dump_dir);
		file_handle = sys_open(messages, O_CREAT|O_WRONLY|O_SYNC, 0644);

		if(!IS_ERR((const void *)file_handle))
		{
			printk("save_ram_dump SMEM+++\n");
			ret = sys_write(file_handle, (unsigned char*)ptr, smem_segments[0].size);
			sys_close(file_handle);
			printk("save_ram_dump SMEM---\n");
		}
		set_fs(oldfs);
		iounmap(ptr);

		pet_watchdog();
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		sprintf(messages, "/data/log/modem_crash.log");
		file_handle = sys_open(messages, O_CREAT|O_WRONLY|O_SYNC | O_TRUNC, 0644);
		if(!IS_ERR((const void *)file_handle))
		{
			sprintf(messages, "%s\n%s\n", dump_dir, dump_log);
			ret = sys_write(file_handle, (unsigned char*)messages, strlen(messages));
			sys_close(file_handle);
		}
		sprintf(messages, "%s/load.cmm", dump_dir);
		file_handle = sys_open(messages, O_CREAT|O_WRONLY|O_SYNC, 0644);

		if(!IS_ERR((const void *)file_handle))
		{
			/* LOADMEM:
			 * ENTRY &1 &2 &3 &4
			 * LOCAL &START_ADDR
			 * LOCAL &SIZE
			 * LOCAL &LOG
			 * LOCAL &OFFSET

			 * &START_ADDR="&1"
			 * &SIZE="&2"
			 * &LOG="&RAMDUMPDIR/"+"&3"
			 * &OFFSET="&4"

			 * DATA.LOAD.BINARY &LOG &START_ADDR++&SIZE /LONG /SKIP &4
			 * PRINT %CONTINUE " &LOG LOADED."
			 * RETURN
			 */
			printk("save_ram_dump load.cmm+++\n");
			sprintf(messages, "%s\n", dump_log);
			ret = sys_write(file_handle, (unsigned char*)messages, strlen(messages));
			sprintf(messages, "GOSUB LOADMEM 0x%08lX 0x%08lX RAMDump_SMEM_%04d%02d%02d-%02d%02d%02d.bin 0\r\n",
					smem_segments[0].address, smem_segments[0].size,
					tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
			ret = sys_write(file_handle, (unsigned char*)messages, strlen(messages));
			sprintf(messages, "GOSUB LOADMEM 0x%08lX 0x%08lX RAMDump_Modem_sw_%04d%02d%02d-%02d%02d%02d.bin 0\r\n",
					modemsw_segments[0].address, modemsw_segments[0].size,
					tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
			ret = sys_write(file_handle, (unsigned char*)messages, strlen(messages));
			sprintf(messages, "GOSUB LOADMEM 0x%08lX 0x%08lX RAMDump_Modem_fw_%04d%02d%02d-%02d%02d%02d.bin 0\r\n",
					modemfw_segments[0].address, modemfw_segments[0].size,
					tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
			ret = sys_write(file_handle, (unsigned char*)messages, strlen(messages));
			sprintf(messages, "ENDDO\r\n");
			ret = sys_write(file_handle, (unsigned char*)messages, strlen(messages));

			sprintf(messages, "LOADMEM:\r\nENTRY &1 &2 &3 &4\r\nLOCAL &START_ADDR\r\nLOCAL &SIZE\r\nLOCAL &LOG\r\nLOCAL &OFFSET\r\n");
			ret = sys_write(file_handle, (unsigned char*)messages, strlen(messages));
			sprintf(messages, "&START_ADDR=\"&1\"\r\n&SIZE=\"&2\"\r\n&LOG=\"&RAMDUMPDIR/\"+\"&3\"\r\n");
			ret = sys_write(file_handle, (unsigned char*)messages, strlen(messages));
			sprintf(messages, "&OFFSET=\"&4\"\r\nDATA.LOAD.BINARY &LOG &START_ADDR++&SIZE /LONG /SKIP &4\r\nPRINT %%CONTINUE \" &LOG LOADED.\"\r\nRETURN\r\n");
			ret = sys_write(file_handle, (unsigned char*)messages, strlen(messages));        

			sys_close(file_handle);
			printk("save_ram_dump load.cmm---\n");
		}
		set_fs(oldfs);
	} else {
		unsigned char *ptr;
		char dump_log[64];
		int file_handle;
		int ret;
		mm_segment_t oldfs;
		char messages[256];

		memset(dump_log, 0, sizeof(dump_log));
		//ptr = ioremap_nocache(modemsw_segments[0].address,  modemsw_segments[0].size);
		ptr = ioremap_nocache(modemsw_segments[0].address + ASUSQ6SW_OFFSET,  ASUSQ6SW_BYTES);
		memcpy(dump_log, ptr, ASUSQ6SW_BYTES);
		pet_watchdog();
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		sprintf(messages, "/data/log/modem_crash.log");
		file_handle = sys_open(messages, O_CREAT|O_WRONLY|O_SYNC | O_TRUNC, 0644);
		if(!IS_ERR((const void *)file_handle))
		{
			sprintf(messages, "%s\n", dump_log);
			ret = sys_write(file_handle, (unsigned char*)messages, strlen(messages));
			sys_close(file_handle);
		}
		set_fs(oldfs);
		printk("%s\n", dump_log);
		iounmap(ptr);
	}
#endif
// ASUS_BSP--- Wenli "Modify for modem restart"
	return ret;
}

static irqreturn_t modem_wdog_bite_irq(int irq, void *dev_id)
{
	switch (irq) {

	case Q6SW_WDOG_EXPIRED_IRQ:
		pr_err("Watchdog bite received from modem software!\n");
		restart_modem();
		break;
	case Q6FW_WDOG_EXPIRED_IRQ:
		pr_err("Watchdog bite received from modem firmware!\n");
		restart_modem();
		break;
	break;

	default:
		pr_err("%s: Unknown IRQ!\n", __func__);
	}

	return IRQ_HANDLED;
}

static struct subsys_data modem_8960 = {
	.name = "modem",
	.shutdown = modem_shutdown,
	.powerup = modem_powerup,
	.ramdump = modem_ramdump,
	.crash_shutdown = modem_crash_shutdown
};

static int modem_subsystem_restart_init(void)
{
	return ssr_register_subsystem(&modem_8960);
}

static int modem_debug_set(void *data, u64 val)
{
	if (val == 1)
		subsystem_restart("modem");

	return 0;
}

static int modem_debug_get(void *data, u64 *val)
{
	*val = 0;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(modem_debug_fops, modem_debug_get, modem_debug_set,
				"%llu\n");

static int modem_debugfs_init(void)
{
	struct dentry *dent;
	dent = debugfs_create_dir("modem_debug", 0);

	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("reset_modem", 0644, dent, NULL,
		&modem_debug_fops);
	return 0;
}

static int __init modem_8960_init(void)
{
	int ret;

	if (!cpu_is_msm8960() && !cpu_is_msm8930() && !cpu_is_msm8930aa() &&
	    !cpu_is_msm9615() && !cpu_is_msm8627())
		return -ENODEV;

	ret = smsm_state_cb_register(SMSM_MODEM_STATE, SMSM_RESET,
		smsm_state_cb, 0);

	if (ret < 0)
		pr_err("%s: Unable to register SMSM callback! (%d)\n",
				__func__, ret);

	ret = request_irq(Q6FW_WDOG_EXPIRED_IRQ, modem_wdog_bite_irq,
			IRQF_TRIGGER_RISING, "modem_wdog_fw", NULL);

	if (ret < 0) {
		pr_err("%s: Unable to request q6fw watchdog IRQ. (%d)\n",
				__func__, ret);
		goto out;
	}

	ret = request_irq(Q6SW_WDOG_EXPIRED_IRQ, modem_wdog_bite_irq,
			IRQF_TRIGGER_RISING, "modem_wdog_sw", NULL);

	if (ret < 0) {
		pr_err("%s: Unable to request q6sw watchdog IRQ. (%d)\n",
				__func__, ret);
		disable_irq_nosync(Q6FW_WDOG_EXPIRED_IRQ);
		goto out;
	}

	ret = modem_subsystem_restart_init();

	if (ret < 0) {
		pr_err("%s: Unable to reg with subsystem restart. (%d)\n",
				__func__, ret);
		goto out;
	}

	modemfw_ramdump_dev = create_ramdump_device("modem_fw");

	if (!modemfw_ramdump_dev) {
		pr_err("%s: Unable to create modem fw ramdump device. (%d)\n",
				__func__, -ENOMEM);
		ret = -ENOMEM;
		goto out;
	}

	modemsw_ramdump_dev = create_ramdump_device("modem_sw");

	if (!modemsw_ramdump_dev) {
		pr_err("%s: Unable to create modem sw ramdump device. (%d)\n",
				__func__, -ENOMEM);
		ret = -ENOMEM;
		goto out;
	}

	smem_ramdump_dev = create_ramdump_device("smem-modem");

	if (!smem_ramdump_dev) {
		pr_err("%s: Unable to create smem ramdump device. (%d)\n",
				__func__, -ENOMEM);
		ret = -ENOMEM;
		goto out;
	}

	ret = modem_debugfs_init();

	pr_info("%s: modem fatal driver init'ed.\n", __func__);
out:
	return ret;
}

module_init(modem_8960_init);
