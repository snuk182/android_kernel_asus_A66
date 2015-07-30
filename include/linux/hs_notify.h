#ifndef _LINUX_HS_NOTIFY_H
#define _LINUX_HS_NOTIFY_H
#include <linux/notifier.h>

/*
*   Notification Msg definition
*/

#define P01_EVENT_NOTIFY_BACKCOVER_REMOVE         0
#define P01_EVENT_NOTIFY_BACKCOVER_ADD               1

//#define P01_EVENT_NOTIFY_HDMI_ADD               2
//#define P01_EVENT_NOTIFY_HDMI_REMOVE             3


/*
*   Priority here is for hall sensor call function
*   The functions registered with higher prority are called/notified earlier
*
*/


enum hs_notify_priority {
	CAMERA_NOTIFY=0,
	BT_PEN_NOTIFY,
	USB_NOTIFY,
	BATTERY_NOTIFY,
	AUDIO_NOTIFY,
	TOUCH_NOTIFY,
	LCD_NOTIFY,
	NINE_AXIS_SENSOR_NOTIFY,
	VIBRATOR_NOTIFY,
	MICROP_NOTIFY,
};


extern int register_hs_notifier(struct notifier_block *nb);
extern int unregister_hs_notifier(struct notifier_block *nb);

/*
*   Macro function used to declare notification callback & priority of hall sensor
*/
#define hs_notifier(fn, pri) {                          \
	static struct notifier_block fn##_nb =                  \
		{ .notifier_call = fn, .priority = pri };       \
	register_hs_notifier(&fn##_nb);                 \
}

#endif /* _LINUX_HS_NOTIFY_H */

