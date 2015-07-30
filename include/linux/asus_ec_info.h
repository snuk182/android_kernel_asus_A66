#ifndef _ASUS_EC_INFO_H
#define _ASUS_EC_INFO_H


enum asusec_Charging_Status{
	EC_CHARGING_ERR = -1,
	EC_CHARGING_NO = 0,
	EC_CHARGING_ONGOING = 1,
	EC_CHARGING_FULL = 2,
};

enum asusec_Cable_Status{
	EC_CABLE_ERR = -1,
	EC_CABLE_NO = 0,
	EC_CABLE_AC = 1,
	EC_CABLE_USB = 2,
};

enum asusec_Charging_Status asusec_dock_battery_charging_status(void);
enum asusec_Cable_Status asusec_dock_cable_status(void);

// asus_bat.c
int asusec_Dock_Ready_status(void);

// msm-otg.c
int asusec_sus_res_callback(bool);
int asusec_suspend_pre_process_callback(bool);

#endif
