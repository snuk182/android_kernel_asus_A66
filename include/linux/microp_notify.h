#ifndef MICROP_NOTIFY_H
#define MICROP_NOTIFY_H


/*
*   Notification Msg definition
*/

enum microp_msg{
        P01_ADD=0,
        P01_REMOVE,
        P01_BATTERY_POWER_BAD,
        P01_BATTERY_TO_CHARGING,    
        P01_BATTERY_TO_NON_CHARGING,    
        P01_HEADPHONE_IN,
        P01_HEADPHONE_OUT,
        P01_VOLUP_KEY_PRESSED,
        P01_VOLUP_KEY_RELEASED,
        P01_VOLDN_KEY_PRESSED,
        P01_VOLDN_KEY_RELEASED,
        P01_PWR_KEY_PRESSED,
        P01_PWR_KEY_RELEASED,
        P01_LIGHT_SENSOR,
        P01_AC_USB_IN,
        P01_AC_USB_OUT,
        DOCK_PLUG_IN,
        DOCK_PLUG_OUT,
        DOCK_KEY_TOUCH_EVENT,
        DOCK_EXT_POWER_PLUG_IN,
        DOCK_EXT_POWER_PLUG_OUT,
        DOCK_BATTERY_POWER_BAD,
        DOCK_LID_CHANGE_EVENT,
        P01_DEAD,
        DOCK_INIT_READY,
        DOCK_EXT_POWER_PLUG_IN_READY,
        DOCK_EXT_POWER_PLUG_OUT_READY,
        DOCK_BATTERY_POWER_BAD_READY,
        P01_HEADSET_HOOKKEY_PRESSED,
        P01_HEADSET_HOOKKEY_RELEASED,
};

/*
*   Priority here is for hall sensor call function
*   The functions registered with higher prority are called/notified earlier
*
*/



enum microp_notify_id {
        CAMERA_MP_NOTIFY=0,
        BT_PEN_MP_NOTIFY,
        USB_MP_NOTIFY,
        BATTERY_MP_NOTIFY,
        AUDIO_MP_NOTIFY,
        DOCK_MP_NOTIFY,
        TOUCH_MP_NOTIFY,
        LCD_MP_NOTIFY,
        NINE_AXIS_SENSOR_MP_NOTIFY,
        VIBRATOR_MP_NOTIFY,
        MICROP_MP_NOTIFY,
        CM3623_LIGHTSENSOR_MP_NOTIFY,
        AL3010_LIGHTSENSOR_MP_NOTIFY,
};



extern int register_microp_notifier(struct notifier_block *nb);
extern int unregister_microp_notifier(struct notifier_block *nb);


#endif
