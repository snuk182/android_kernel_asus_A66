/* kernel/power/earlysuspend.c
 *
 * Copyright (C) 2005-2008 Google, Inc.
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

#include <linux/earlysuspend.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#include "power.h"
// [ASUS][PM][+++] CY add for power debug
#include <linux/delay.h>
static bool bIsASUSMSKSet=0;
static int intTotalTime;
// [ASUS][PM][---] CY add for power debug

int pm_new_state;//Ledger

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 2,
	DEBUG_VERBOSE = 1U << 3,
};
static int debug_mask = DEBUG_USER_STATE | DEBUG_SUSPEND;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static DEFINE_MUTEX(early_suspend_lock);
static LIST_HEAD(early_suspend_handlers);
static void early_suspend(struct work_struct *work);
static void late_resume(struct work_struct *work);
static DECLARE_WORK(early_suspend_work, early_suspend);
static DECLARE_WORK(late_resume_work, late_resume);
static DEFINE_SPINLOCK(state_lock);
enum {
	SUSPEND_REQUESTED = 0x1,
	SUSPENDED = 0x2,
	SUSPEND_REQUESTED_AND_SUSPENDED = SUSPEND_REQUESTED | SUSPENDED,
};
static int state;

void register_early_suspend(struct early_suspend *handler)
{
	struct list_head *pos;

	mutex_lock(&early_suspend_lock);
	list_for_each(pos, &early_suspend_handlers) {
		struct early_suspend *e;
		e = list_entry(pos, struct early_suspend, link);
		if (e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	if ((state & SUSPENDED) && handler->suspend)
		handler->suspend(handler);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(register_early_suspend);

void unregister_early_suspend(struct early_suspend *handler)
{
	mutex_lock(&early_suspend_lock);
	list_del(&handler->link);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(unregister_early_suspend);

// [ASUS][PM][+++] CY add for power debug
static void drv_show_time(ktime_t starttime)
{
	ktime_t calltime;
	u64 usecs64;
	int usecs;

	calltime = ktime_get();
	usecs64 = ktime_to_ns(ktime_sub(calltime, starttime));
	do_div(usecs64, NSEC_PER_USEC);
	usecs = usecs64;
	if (usecs == 0)
		usecs = 1;
	if (bIsASUSMSKSet)
	{
		printk(DBGMSK_PWR_G3 "  [PM]cost:%ld.%03ld msecs\n", usecs / USEC_PER_MSEC, usecs % USEC_PER_MSEC);
		intTotalTime = intTotalTime+usecs;
	}
}
// [ASUS][PM][---] CY add for power debug
static void early_suspend(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED)
		state |= SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("early_suspend: abort, state %d\n", state);
		mutex_unlock(&early_suspend_lock);
		goto abort;
	}

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: call handlers\n");

	// [ASUS][PM][+++] CY add for power debug
	printk(DBGMSK_PWR_G0 "[PM][+++] early_suspend callback loop\n");
	bIsASUSMSKSet = isASUS_MSK_set(DBGMSK_PWR_G3);
	printk(DBGMSK_PWR_G0 "[PM] PWR_G3_MSK:%d\n", bIsASUSMSKSet);

	intTotalTime =0;
	list_for_each_entry(pos, &early_suspend_handlers, link) {
		if (pos->suspend != NULL) {
			if (bIsASUSMSKSet)
			{
				ktime_t starttime;

				printk(DBGMSK_PWR_G3 " [PM]:%pf ++\n", pos->suspend);
				starttime = ktime_get();
				pos->suspend(pos);
				drv_show_time(starttime);
				printk(DBGMSK_PWR_G3 " [PM]%pf --\n", pos->suspend);
				msleep(0);
			}
			else
			{
				if (debug_mask & DEBUG_VERBOSE)
					pr_info("early_suspend: calling %pf\n", pos->suspend);
				pos->suspend(pos);
			}
		}
	}
	mutex_unlock(&early_suspend_lock);

	if (bIsASUSMSKSet)
		printk(DBGMSK_PWR_G3 " [PM]Total cost by drivers:%ld.%03ld msecs\n", intTotalTime / USEC_PER_MSEC, intTotalTime % USEC_PER_MSEC);
	printk(DBGMSK_PWR_G0 "[PM][---] early_suspend callback loop\n");
	// [ASUS][PM][---] CY add for power debug
	suspend_sys_sync_queue();
abort:
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED_AND_SUSPENDED) {
        printk(DBGMSK_PWR_G2 "[PM]unlock main_wakelock\n");
		wake_unlock(&main_wake_lock);
    }
	spin_unlock_irqrestore(&state_lock, irqflags);
}

static void late_resume(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPENDED)
		state &= ~SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("late_resume: abort, state %d\n", state);
		goto abort;
	}
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: call handlers\n");
	// [ASUS][PM][+++] CY add for power debug
	printk(DBGMSK_PWR_G0 "[PM][+++] late_resume callback loop\n");
	intTotalTime =0;
	
	list_for_each_entry_reverse(pos, &early_suspend_handlers, link) {
		if (pos->resume != NULL) {
			if (bIsASUSMSKSet==1)
			{
				ktime_t starttime;
				
				printk(DBGMSK_PWR_G3 " [PM]%pf ++\n", pos->resume);
				starttime = ktime_get();
				pos->resume(pos);
				drv_show_time(starttime);
				printk(DBGMSK_PWR_G3 " [PM]%pf --\n", pos->resume);
				msleep(0);
			}
			else
			{
				if (debug_mask & DEBUG_VERBOSE)
					pr_info("late_resume: calling %pf\n", pos->resume);
				pos->resume(pos);
			}
		}
	}

	if (bIsASUSMSKSet)
		printk(DBGMSK_PWR_G3 " [PM]Total cost by drivers:%ld.%03ld msecs\n", intTotalTime / USEC_PER_MSEC, intTotalTime % USEC_PER_MSEC);
	printk(DBGMSK_PWR_G0 "[PM][---] late_resume callback loop\n");
	// [ASUS][PM][---] CY add for power debug
	
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: done\n");
abort:
	mutex_unlock(&early_suspend_lock);
}

extern struct timer_list unattended_timer;

void request_suspend_state(suspend_state_t new_state)
{
	unsigned long irqflags;
	int old_sleep;

	spin_lock_irqsave(&state_lock, irqflags);
	old_sleep = state & SUSPEND_REQUESTED;

//++Ledger
        pm_new_state=new_state;
        ASUSEvtlog("[PM] request_suspend_state: %s (%d->%d)\n", new_state != PM_SUSPEND_ON ? "sleep" : "wakeup",
                        requested_suspend_state, new_state);
//--Ledger
	if (debug_mask & DEBUG_USER_STATE) {
		struct timespec ts;
		struct rtc_time tm;
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
		pr_info("request_suspend_state: %s (%d->%d) at %lld "
			"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
			new_state != PM_SUSPEND_ON ? "sleep" : "wakeup",
			requested_suspend_state, new_state,
			ktime_to_ns(ktime_get()),
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	}
	if (!old_sleep && new_state != PM_SUSPEND_ON) {
        printk(DBGMSK_PWR_G2 "[PM]req_suspend_state: early_suspend_work\n");
		state |= SUSPEND_REQUESTED;
		queue_work(suspend_work_queue, &early_suspend_work);
//Add a timer to debug unattended mode wakelock
	printk("[PM]unattended timer: mod_timer (early_suspend)\n");
	mod_timer(&unattended_timer, jiffies + msecs_to_jiffies(PM_UNATTENDED_TIMEOUT));
	} else if (old_sleep && new_state == PM_SUSPEND_ON) {
        printk(DBGMSK_PWR_G2 "[PM]req_suspend_state: late_resume_work\n");
		state &= ~SUSPEND_REQUESTED;
		wake_lock(&main_wake_lock);
		queue_work(suspend_work_queue, &late_resume_work);
//delete timer wakelock debug
	printk("[PM]unattended_timer: del_timer (late_resume)\n");
	del_timer(&unattended_timer);
	}
    else {
        printk(DBGMSK_PWR_G2 "[PM]req_suspend_state: wrong state, old_sleep:%d, new_state:%d\n",
                old_sleep, new_state);
	}
	requested_suspend_state = new_state;
	spin_unlock_irqrestore(&state_lock, irqflags);
}

suspend_state_t get_suspend_state(void)
{
	return requested_suspend_state;
}
