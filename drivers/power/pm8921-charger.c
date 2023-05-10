/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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
//snuk182 !!!!
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/mfd/pm8xxx/pm8921-charger.h>
#include <linux/mfd/pm8xxx/pm8921-bms.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>
#include <linux/mfd/pm8xxx/ccadc.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/regulator/consumer.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/mfd/pm8xxx/batt-alarm.h>
#include <linux/ratelimit.h>

#include <mach/msm_xo.h>
#include <mach/msm_hsusb.h>
//#include <mach/pm.h>          //ASUS_BSP eason QCPatch:fix statemachine lookup
//#include <mach/cpuidle.h>	  //ASUS_BSP eason QCPatch:fix statemachine lookup

//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp_api.h>
#endif /* CONFIG_EEPROM_NUVOTON */
#include <linux/asus_bat.h>
#include <linux/asus_chg.h>
//#include <linux/mutex.h>
#define CHK_STATUS_CHECK_PERIOD_S	100 //100 S
#define DELAY_FOR_CHK_STATUS	5// 5S, because the temp state of pmic, will be stable in 2 sec
#define DELAY_FOR_CHK_STATUS_IN_RESUME	1//DELAY_FOR_CHECK_STATE_IN_RESUME
//ASUS_BSP --- Josh_Liao "add asus battery driver"

#define CHG_BUCK_CLOCK_CTRL	0x14
#define CHG_BUCK_CLOCK_CTRL_8038	0xD

#define PBL_ACCESS1		0x04
#define PBL_ACCESS2		0x05
#define SYS_CONFIG_1		0x06
#define SYS_CONFIG_2		0x07
#define CHG_CNTRL		0x204
#define CHG_IBAT_MAX		0x205
#define CHG_TEST		0x206
#define CHG_BUCK_CTRL_TEST1	0x207
#define CHG_BUCK_CTRL_TEST2	0x208
#define CHG_BUCK_CTRL_TEST3	0x209
#define COMPARATOR_OVERRIDE	0x20A
#define PSI_TXRX_SAMPLE_DATA_0	0x20B
#define PSI_TXRX_SAMPLE_DATA_1	0x20C
#define PSI_TXRX_SAMPLE_DATA_2	0x20D
#define PSI_TXRX_SAMPLE_DATA_3	0x20E
#define PSI_CONFIG_STATUS	0x20F
#define CHG_IBAT_SAFE		0x210
#define CHG_ITRICKLE		0x211
#define CHG_CNTRL_2		0x212
#define CHG_VBAT_DET		0x213
#define CHG_VTRICKLE		0x214
#define CHG_ITERM		0x215
#define CHG_CNTRL_3		0x216
#define CHG_VIN_MIN		0x217
#define CHG_TWDOG		0x218
#define CHG_TTRKL_MAX		0x219
#define CHG_TEMP_THRESH		0x21A
#define CHG_TCHG_MAX		0x21B
#define USB_OVP_CONTROL		0x21C
#define DC_OVP_CONTROL		0x21D
#define USB_OVP_TEST		0x21E
#define DC_OVP_TEST		0x21F
#define CHG_VDD_MAX		0x220
#define CHG_VDD_SAFE		0x221
#define CHG_VBAT_BOOT_THRESH	0x222
#define USB_OVP_TRIM		0x355
#define BUCK_CONTROL_TRIM1	0x356
#define BUCK_CONTROL_TRIM2	0x357
#define BUCK_CONTROL_TRIM3	0x358
#define BUCK_CONTROL_TRIM4	0x359
#define CHG_DEFAULTS_TRIM	0x35A
#define CHG_ITRIM		0x35B
#define CHG_TTRIM		0x35C
#define CHG_COMP_OVR		0x20A
#define IUSB_FINE_RES		0x2B6
#define OVP_USB_UVD		0x2B7

/* check EOC every 10 seconds */
#define EOC_CHECK_PERIOD_MS	10000
/* check for USB unplug every 200 msecs */
#define UNPLUG_CHECK_WAIT_PERIOD_MS 200
#define UNPLUG_CHECK_RAMP_MS 25
#define USB_TRIM_ENTRIES 16

//ASUS BSP +++ Eason Chang add BAT global variable
#define UPDATE_BATGLOBAL_PERIOD      120       //min
#define UPDATE_BATGLOBAL_FIRSTTIME   30000     //msecs
//ASUS BSP --- Eason Chang add BAT global variable
//Eason if doesn't get correct ADC Vol&Curr at first update Cap show unknow status +++ 
extern bool g_adc_get_correct_VolCurr;
//Eason if doesn't get correct ADC Vol&Curr at first update Cap show unknow status ---
//Eason : when thermal too hot, limit charging current +++ 
extern void notifyThermalLimit(int thermalnotify);
//Eason : when thermal too hot, limit charging current ---

enum chg_fsm_state {
	FSM_STATE_OFF_0 = 0,
	FSM_STATE_BATFETDET_START_12 = 12,
	FSM_STATE_BATFETDET_END_16 = 16,
	FSM_STATE_ON_CHG_HIGHI_1 = 1,
	FSM_STATE_ATC_2A = 2,
	FSM_STATE_ATC_2B = 18,
	FSM_STATE_ON_BAT_3 = 3,
	FSM_STATE_ATC_FAIL_4 = 4 ,
	FSM_STATE_DELAY_5 = 5,
	FSM_STATE_ON_CHG_AND_BAT_6 = 6,
	FSM_STATE_FAST_CHG_7 = 7,
	FSM_STATE_TRKL_CHG_8 = 8,
	FSM_STATE_CHG_FAIL_9 = 9,
	FSM_STATE_EOC_10 = 10,
	FSM_STATE_ON_CHG_VREGOK_11 = 11,
	FSM_STATE_ATC_PAUSE_13 = 13,
	FSM_STATE_FAST_CHG_PAUSE_14 = 14,
	FSM_STATE_TRKL_CHG_PAUSE_15 = 15,
	FSM_STATE_START_BOOT = 20,
	FSM_STATE_FLCB_VREGOK = 21,
	FSM_STATE_FLCB = 22,
};

struct fsm_state_to_batt_status {
	enum chg_fsm_state	fsm_state;
	int			batt_state;
};

static struct fsm_state_to_batt_status map[] = {
	{FSM_STATE_OFF_0, POWER_SUPPLY_STATUS_UNKNOWN},
	{FSM_STATE_BATFETDET_START_12, POWER_SUPPLY_STATUS_UNKNOWN},
	{FSM_STATE_BATFETDET_END_16, POWER_SUPPLY_STATUS_UNKNOWN},
	/*
	 * for CHG_HIGHI_1 report NOT_CHARGING if battery missing,
	 * too hot/cold, charger too hot
	 */
	{FSM_STATE_ON_CHG_HIGHI_1, POWER_SUPPLY_STATUS_FULL},
	{FSM_STATE_ATC_2A, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_ATC_2B, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_ON_BAT_3, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_ATC_FAIL_4, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_DELAY_5, POWER_SUPPLY_STATUS_UNKNOWN },
	{FSM_STATE_ON_CHG_AND_BAT_6, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_FAST_CHG_7, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_TRKL_CHG_8, POWER_SUPPLY_STATUS_CHARGING},
	{FSM_STATE_CHG_FAIL_9, POWER_SUPPLY_STATUS_DISCHARGING},
	{FSM_STATE_EOC_10, POWER_SUPPLY_STATUS_FULL},
	{FSM_STATE_ON_CHG_VREGOK_11, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_ATC_PAUSE_13, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FAST_CHG_PAUSE_14, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_TRKL_CHG_PAUSE_15, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_START_BOOT, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FLCB_VREGOK, POWER_SUPPLY_STATUS_NOT_CHARGING},
	{FSM_STATE_FLCB, POWER_SUPPLY_STATUS_NOT_CHARGING},
};

enum chg_regulation_loop {
	VDD_LOOP = BIT(3),
	BAT_CURRENT_LOOP = BIT(2),
	INPUT_CURRENT_LOOP = BIT(1),
	INPUT_VOLTAGE_LOOP = BIT(0),
	CHG_ALL_LOOPS = VDD_LOOP | BAT_CURRENT_LOOP
			| INPUT_CURRENT_LOOP | INPUT_VOLTAGE_LOOP,
};

enum pmic_chg_interrupts {
	USBIN_VALID_IRQ = 0,
	USBIN_OV_IRQ,
	BATT_INSERTED_IRQ,
	VBATDET_LOW_IRQ,
	USBIN_UV_IRQ,
	VBAT_OV_IRQ,
	CHGWDOG_IRQ,
	VCP_IRQ,
	ATCDONE_IRQ,
	ATCFAIL_IRQ,
	CHGDONE_IRQ,
	CHGFAIL_IRQ,
	CHGSTATE_IRQ,
	LOOP_CHANGE_IRQ,
	FASTCHG_IRQ,
	TRKLCHG_IRQ,
	BATT_REMOVED_IRQ,
	BATTTEMP_HOT_IRQ,
	CHGHOT_IRQ,
	BATTTEMP_COLD_IRQ,
	CHG_GONE_IRQ,
	BAT_TEMP_OK_IRQ,
	COARSE_DET_LOW_IRQ,
	VDD_LOOP_IRQ,
	VREG_OV_IRQ,
	VBATDET_IRQ,
	BATFET_IRQ,
	PSI_IRQ,
	DCIN_VALID_IRQ,
	DCIN_OV_IRQ,
	DCIN_UV_IRQ,
	PM_CHG_MAX_INTS,
};

struct bms_notify {
	int			is_battery_full;
	int			is_charging;
	struct	work_struct	work;
};

/**
 * struct pm8921_chg_chip -device information
 * @dev:			device pointer to access the parent
 * @usb_present:		present status of usb
 * @dc_present:			present status of dc
 * @usb_charger_current:	usb current to charge the battery with used when
 *				the usb path is enabled or charging is resumed
 * @update_time:		how frequently the userland needs to be updated
 * @max_voltage_mv:		the max volts the batt should be charged up to
 * @min_voltage_mv:		the min battery voltage before turning the FETon
 * @uvd_voltage_mv:		(PM8917 only) the falling UVD threshold voltage
 * @alarm_low_mv:		the battery alarm voltage low
 * @alarm_high_mv:		the battery alarm voltage high
 * @cool_temp_dc:		the cool temp threshold in deciCelcius
 * @warm_temp_dc:		the warm temp threshold in deciCelcius
 * @resume_voltage_delta:	the voltage delta from vdd max at which the
 *				battery should resume charging
 * @term_current:		The charging based term current
 *
 */
struct pm8921_chg_chip {
	struct device			*dev;
//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
	unsigned int			chg_present;
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"
	unsigned int			usb_present;
	unsigned int			dc_present;
	unsigned int			usb_charger_current;
	unsigned int			max_bat_chg_current;
	unsigned int			pmic_chg_irq[PM_CHG_MAX_INTS];
	unsigned int			ttrkl_time;
	unsigned int			update_time;
	unsigned int			max_voltage_mv;
	unsigned int			min_voltage_mv;
	unsigned int			uvd_voltage_mv;
	unsigned int			safe_current_ma;
	unsigned int			alarm_low_mv;
	unsigned int			alarm_high_mv;
	int				cool_temp_dc;
	int				warm_temp_dc;
	unsigned int			temp_check_period;
	unsigned int			cool_bat_chg_current;
	unsigned int			warm_bat_chg_current;
	unsigned int			cool_bat_voltage;
	unsigned int			warm_bat_voltage;
	unsigned int			is_bat_cool;
	unsigned int			is_bat_warm;
	unsigned int			resume_voltage_delta;
	int				resume_charge_percent;
	unsigned int			term_current;
	unsigned int			vbat_channel;
	unsigned int			batt_temp_channel;
	unsigned int			batt_id_channel;
	struct power_supply		usb_psy;
	struct power_supply		dc_psy;
	struct power_supply		*ext_psy;
	struct power_supply		batt_psy;
	struct dentry			*dent;
	struct bms_notify		bms_notify;
	struct ext_chg_pm8921		*ext;//ASUS BSP Eason_Chang 1030 porting
	int				*usb_trim_table;
	struct regulator		*vreg_xoadc;
	bool				ext_charging;
	bool				ext_charge_done;
	bool				iusb_fine_res;
	bool				final_kickstart;
	bool				lockup_lpm_wrkarnd;
	DECLARE_BITMAP(enabled_irqs, PM_CHG_MAX_INTS);
	struct work_struct		battery_id_valid_work;
	int64_t				batt_id_min;
	int64_t				batt_id_max;
	int				trkl_voltage;
	int				weak_voltage;
	int				trkl_current;
	int				weak_current;
	int				vin_min;
	unsigned int			*thermal_mitigation;
	int				thermal_levels;
	struct delayed_work		update_heartbeat_work;
	struct delayed_work		eoc_work;
	struct delayed_work		unplug_wrkarnd_restore_work;     //ASUS_BSP eason QCPatch:Workaround for USB unplug issue
	struct delayed_work		unplug_check_work;
	struct wake_lock		unplug_wrkarnd_restore_wake_lock;//ASUS_BSP eason QCPatch:Workaround for USB unplug issue
	struct delayed_work		vin_collapse_check_work;
	struct delayed_work		btc_override_work;
	struct wake_lock		eoc_wake_lock;
	enum pm8921_chg_cold_thr	cold_thr;
	enum pm8921_chg_hot_thr		hot_thr;
	struct notifier_block		notifier;    //ASUS_BSP eason QCPatch:fix statemachine lookup  
	//ASUS BSP+++ for chg statue checking  
	struct workqueue_struct *chg_status_check_wq ;
	struct delayed_work chg_status_check_work;
	int chg_state_polling_interval;
	bool chgdone_reported;
	bool fastchg_done;
	//ASUS BSP--- for chg status checking 
	//ASUS BSP +++ Eason Chang add BAT global variable
	struct delayed_work update_BATinfo_worker;
	//ASUS BSP --- Eason Chang add BAT global variable
	int				rconn_mohm;
	enum pm8921_chg_led_src_config	led_src_config;
	bool				host_mode;
	bool				has_dc_supply;
	u8				active_path;
	int				recent_reported_soc;
	int				ext_batt_health;
	int				ext_batt_temp_monitor;
	int				battery_less_hardware;
	int				ibatmax_max_adj_ma;
	int				btc_override;
	int				btc_override_cold_decidegc;
	int				btc_override_hot_decidegc;
	int				btc_delay_ms;
	bool				btc_panic_if_cant_stop_chg;
	int				stop_chg_upon_expiry;
	bool				disable_aicl;
};

///+++ASUS_BSP Eason_Chang  ASUS BAT Update using Global variable
//int  gBatteryStatus=0;
static int  gBatteryVol=0;
static int  gChargeStatus=0;
static int  gBatteryHealth=0;
static int  gBatteryPresent=0;
//int  gBatteryCapacity=50;
static int  gBatteryCurrent=0;
static int  gBatteryCurrentMax=0;//Eason: A80 12130 porting
static int  gBatteryFcc=1520;
static int  gBatteryTemp=25;
///---ASUS_BSP Eason_Chang  ASUS BAT Update using Global variable

/* user space parameter to limit usb current */
static unsigned int usb_max_current;
/*
 * usb_target_ma is used for wall charger
 * adaptive input current limiting only. Use
 * pm_iusbmax_get() to get current maximum usb current setting.
 */
static int usb_target_ma;
static int charging_disabled;
static int thermal_mitigation;

//ASUS BSP+++ notify BatteryService   
static int gSaved8921FsmState = FSM_STATE_ON_BAT_3;
//ASUS BSP--- notify BatteryService 

static struct pm8921_chg_chip *the_chip;

//+++ASUS_BSP+++ Eason_Chang  ASUS SW gauge
static struct pm8921_chg_chip *the_chip = NULL;
static bool asus_charge_done = false;
//Victor@20120112:for chgdone checking....

///ASUS_BSP +++ Eason_Chang  ASUS BAT Update using Global variable
void UpdateGlobalValue(void);
///ASUS_BSP --- Eason_Chang  ASUS BAT Update using Global variable

bool asus_get_charge_done(void)
{
	return asus_charge_done;
}
static void asus_set_charge_done(void)
{
	asus_charge_done = true;
}
static void asus_set_not_charge_done(void)
{
	asus_charge_done = false;
}
// ASUS_BSP +++ SinaChou
extern int AX_MicroP_isFWSupportP02(void);
// ASUS_BSP ---
static DEFINE_SPINLOCK(lpm_lock);
#define LPM_ENABLE_BIT	BIT(2)
static int pm8921_chg_set_lpm(struct pm8921_chg_chip *chip, int enable)
{
	int rc;
	u8 reg;

	rc = pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &reg);
	if (rc) {
		pr_err("pm8xxx_readb failed: addr=%03X, rc=%d\n",
				CHG_CNTRL, rc);
		return rc;
	}
	reg &= ~LPM_ENABLE_BIT;
	reg |= (enable ? LPM_ENABLE_BIT : 0);

	rc = pm8xxx_writeb(chip->dev->parent, CHG_CNTRL, reg);
	if (rc) {
		pr_err("pm_chg_write failed: addr=%03X, rc=%d\n",
				CHG_CNTRL, rc);
		return rc;
	}

	return rc;
}

static int pm_chg_write(struct pm8921_chg_chip *chip, u16 addr, u8 reg)
{
	int rc;
	unsigned long flags = 0;
	u8 temp;

	/* Disable LPM */
	if (chip->lockup_lpm_wrkarnd) {
		spin_lock_irqsave(&lpm_lock, flags);

		/*
		 * This delay is to prevent exit out of 32khz mode within
		 * 200uS. It could be that chg was removed just few uS before
		 * this gets called.
		 */
		udelay(200);
		/* no clks */
		temp = 0xD1;
		rc = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
		if (rc) {
			pr_err("Error %d writing %d to CHG_TEST\n", rc, temp);
			goto release_lpm_lock;
		}

		/* force 19.2Mhz before reading */
		temp = 0xD3;
		rc = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
		if (rc) {
			pr_err("Error %d writing %d to CHG_TEST\n", rc, temp);
			goto release_lpm_lock;
		}

		rc = pm8xxx_writeb(chip->dev->parent, addr, reg);
		if (rc) {
			pr_err("failed: addr=%03X, rc=%d\n", addr, rc);
			goto release_lpm_lock;
		}

		/* no clks */
		temp = 0xD1;
		rc = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
		if (rc) {
			pr_err("Error %d writing %d to CHG_TEST\n", rc, temp);
			goto release_lpm_lock;
		}

		/* switch to hw clk selection */
		temp = 0xD0;
		rc = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
		if (rc) {
			pr_err("Error %d writing %d to CHG_TEST\n", rc, temp);
			goto release_lpm_lock;
		}

		udelay(200);

release_lpm_lock:
		spin_unlock_irqrestore(&lpm_lock, flags);
	} else {
		rc = pm8xxx_writeb(chip->dev->parent, addr, reg);
		if (rc)
			pr_err("failed: addr=%03X, rc=%d\n", addr, rc);
	}
	return rc;
}

//ASUS_BSP +++ Josh_Liao "add asus battery driver"
static enum asus_chg_src local_chg_src = ASUS_CHG_SRC_UNKNOWN;

static struct pm8921_chg_chip *local_chip = NULL;

#define AC_CHG_CURR 	1500
#define USB_CHG_CURR 	500
#define NONE_CHG_CURR 	0

static int is_chg_plugged_in(struct pm8921_chg_chip *chip);
//ASUS_BSP --- Josh_Liao "add asus battery driver"

///+++ASUS_BSP--- Eason_Chang  ASUS SW gauge

// Asus BSP Eason_Chang +++ function for AXI_BatteryServiceFacade
void asus_bat_status_change(void)
{
	power_supply_changed(&the_chip->batt_psy);
}

void asus_chg_set_chg_mode_forBatteryservice(enum asus_chg_src Batteryservice_src)
{
    //not really do the set charging....because the work has already done by pm8921
    //just schedule the delay work to start status checking....
    //need to prevent 
    if(g_A60K_hwID >=A66_HW_ID_ER2){

        return;
    }
    if(is_chg_plugged_in(the_chip)){

        if (!delayed_work_pending(&the_chip->chg_status_check_work)){

            //for debunce the state of fsm of pmic
            schedule_delayed_work(&the_chip->chg_status_check_work,
                          DELAY_FOR_CHK_STATUS*HZ);           
        }

    }else{

        the_chip->chgdone_reported = false;

        the_chip->fastchg_done = false;

    }
}

extern void asus_onCableInOut(int src);
extern void asus_onChargingStop(int chg_stop_reason);
extern void asus_onChargingStart(void);
extern void asus_onBatteryFullAlarm(bool isFull);
// Asus BSP Eason_Chang --- function for AXI_BatteryServiceFacade



//+++ ASUS_BSP Allen1_Huang: enable ICS charger mode
#ifdef CONFIG_CHARGER_MODE
extern char g_CHG_mode;
int g_chg_present = 0; //set to no charger as default
#endif
//--- ASUS_BSP Allen1_Huang: enable ICS charger mode

static int pm_chg_masked_write(struct pm8921_chg_chip *chip, u16 addr,
							u8 mask, u8 val)
{
	int rc;
	u8 reg;

	rc = pm8xxx_readb(chip->dev->parent, addr, &reg);
	if (rc) {
		pr_err("pm8xxx_readb failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}
	reg &= ~mask;
	reg |= val & mask;
	rc = pm_chg_write(chip, addr, reg);
	if (rc) {
		pr_err("pm_chg_write failed: addr=%03X, rc=%d\n", addr, rc);
		return rc;
	}
	return 0;
}

//ASUS_BSP eason QCPatch:fix statemachine lookup +++
static void pm8921_fix_charger_lockup(struct pm8921_chg_chip *chip)
{
	int err;
	u8 temp;

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD3;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}

	temp  = 0xD0;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return;
	}
}
//ASUS_BSP eason QCPatch:fix statemachine lookup---

static int pm_chg_get_rt_status(struct pm8921_chg_chip *chip, int irq_id)
{
	return pm8xxx_read_irq_stat(chip->dev->parent,
					chip->pmic_chg_irq[irq_id]);
}

static int is_chg_on_bat(struct pm8921_chg_chip *chip)
{
	return !(pm_chg_get_rt_status(chip, DCIN_VALID_IRQ)
			|| pm_chg_get_rt_status(chip, USBIN_VALID_IRQ));
}

static void pm8921_chg_bypass_bat_gone_debounce(struct pm8921_chg_chip *chip,
		int bypass)
{
	int rc;

	rc = pm_chg_write(chip, COMPARATOR_OVERRIDE, bypass ? 0x89 : 0x88);
	if (rc) {
		pr_err("Failed to set bypass bit to %d rc=%d\n", bypass, rc);
	}
}

/* Treat OverVoltage/UnderVoltage as source missing */
static int is_usb_chg_plugged_in(struct pm8921_chg_chip *chip)
{
//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
	if (is_chg_plugged_in(chip)&&(chip->usb_present))
		return 1;
	else
		return 0;
#else
	return pm_chg_get_rt_status(chip, USBIN_VALID_IRQ);
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"
}

/* Treat OverVoltage/UnderVoltage as source missing */
static int is_dc_chg_plugged_in(struct pm8921_chg_chip *chip)
{
//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
	if (is_chg_plugged_in(chip)&&(chip->dc_present))
		return 1;
	else
		return 0;
#else
	return pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"
}

static int is_batfet_closed(struct pm8921_chg_chip *chip)
{
	return pm_chg_get_rt_status(chip, BATFET_IRQ);
}
#define CAPTURE_FSM_STATE_CMD	0xC2
#define READ_BANK_7		0x70
#define READ_BANK_4		0x40
static int pm_chg_get_fsm_state(struct pm8921_chg_chip *chip)
{
	u8 temp;
	unsigned long flags = 0;
	int err = 0, ret = 0;

	if (chip->lockup_lpm_wrkarnd) {
		spin_lock_irqsave(&lpm_lock, flags);

		/*
		 * This delay is to prevent exit out of 32khz mode within
		 * 200uS. It could be that chg was removed just few uS before
		 * this gets called.
		 */
		udelay(200);
		/* no clks */
		temp = 0xD1;
		err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
		if (err) {
			pr_err("Error %d writing %d to CHG_TEST\n", err, temp);
			goto err_out;
		}

		/* force 19.2Mhz before reading */
		temp = 0xD3;
		err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
		if (err) {
			pr_err("Error %d writing %d to CHG_TEST\n", err, temp);
			goto err_out;
		}
	}

	temp = CAPTURE_FSM_STATE_CMD;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		goto err_out;
	}

	temp = READ_BANK_7;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		goto err_out;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &temp);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		goto err_out;
	}
	/* get the lower 4 bits */
	ret = temp & 0xF;

	temp = READ_BANK_4;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		goto err_out;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &temp);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		goto err_out;
	}
	/* get the upper 1 bit */
	ret |= (temp & 0x1) << 4;

	if (chip->lockup_lpm_wrkarnd) {
		/* no clks */
		temp = 0xD1;
		err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
		if (err) {
			pr_err("Error %d writing %d to CHG_TEST\n", err, temp);
			goto err_out;
		}

		/* switch to hw clk selection */
		temp = 0xD0;
		err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
		if (err) {
			pr_err("Error %d writing %d to CHG_TEST\n", err, temp);
			goto err_out;
		}

		udelay(200);
	}

err_out:
	if (chip->lockup_lpm_wrkarnd)
		spin_unlock_irqrestore(&lpm_lock, flags);
	if (err)
		return err;

	return  ret;
}

#define READ_BANK_6		0x60
static int pm_chg_get_regulation_loop(struct pm8921_chg_chip *chip)
{
	u8 temp, data;
	unsigned long flags = 0;
	int err = 0;

	if (chip->lockup_lpm_wrkarnd) {
		spin_lock_irqsave(&lpm_lock, flags);

		/*
		 * This delay is to prevent exit out of 32khz mode within
		 * 200uS. It could be that chg was removed just few uS before
		 * this gets called.
		 */
		udelay(200);
		/* no clks */
		temp = 0xD1;
		err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
		if (err) {
			pr_err("Error %d writing %d to CHG_TEST\n", err, temp);
			goto err_out;
		}

		/* force 19.2Mhz before reading */
		temp = 0xD3;
		err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
		if (err) {
			pr_err("Error %d writing %d to CHG_TEST\n", err, temp);
			goto err_out;
		}
	}

	temp = READ_BANK_6;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		goto err_out;
	}

	err = pm8xxx_readb(chip->dev->parent, CHG_TEST, &data);
	if (err) {
		pr_err("pm8xxx_readb fail: addr=%03X, rc=%d\n", CHG_TEST, err);
		goto err_out;
	}

	if (chip->lockup_lpm_wrkarnd) {
		/* no clks */
		temp = 0xD1;
		err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
		if (err) {
			pr_err("Error %d writing %d to CHG_TEST\n", err, temp);
			goto err_out;
		}

		/* switch to hw clk selection */
		temp = 0xD0;
		err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
		if (err) {
			pr_err("Error %d writing %d to CHG_TEST\n", err, temp);
			goto err_out;
		}

		udelay(200);
	}

err_out:
	if (chip->lockup_lpm_wrkarnd)
		spin_unlock_irqrestore(&lpm_lock, flags);
	if (err)
		return err;

	/* return the lower 4 bits */
	return data & CHG_ALL_LOOPS;
}

#define CHG_USB_SUSPEND_BIT  BIT(2)
static int pm_chg_usb_suspend_enable(struct pm8921_chg_chip *chip, int enable)
{
	return pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_USB_SUSPEND_BIT,
			enable ? CHG_USB_SUSPEND_BIT : 0);
}

#define CHG_EN_BIT	BIT(7)
static int pm_chg_auto_enable(struct pm8921_chg_chip *chip, int enable)
{
	return pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_EN_BIT,
				enable ? CHG_EN_BIT : 0);
}

#define CHG_FAILED_CLEAR	BIT(0)
#define ATC_FAILED_CLEAR	BIT(1)
static int pm_chg_failed_clear(struct pm8921_chg_chip *chip, int clear)
{
	int rc;

	rc = pm_chg_masked_write(chip, CHG_CNTRL_3, ATC_FAILED_CLEAR,
				clear ? ATC_FAILED_CLEAR : 0);
	rc |= pm_chg_masked_write(chip, CHG_CNTRL_3, CHG_FAILED_CLEAR,
				clear ? CHG_FAILED_CLEAR : 0);
	return rc;
}

#define CHG_CHARGE_DIS_BIT	BIT(1)
static int pm_chg_charge_dis(struct pm8921_chg_chip *chip, int disable)
{
	return pm_chg_masked_write(chip, CHG_CNTRL, CHG_CHARGE_DIS_BIT,
				disable ? CHG_CHARGE_DIS_BIT : 0);
}

static int pm_is_chg_charge_dis(struct pm8921_chg_chip *chip)
{
	u8 temp;

	pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &temp);
	return  temp & CHG_CHARGE_DIS_BIT;
}
#define PM8921_CHG_V_MIN_MV	3240
#define PM8921_CHG_V_STEP_MV	20
#define PM8921_CHG_V_STEP_10MV_OFFSET_BIT	BIT(7)
#define PM8921_CHG_VDDMAX_MAX	4500
#define PM8921_CHG_VDDMAX_MIN	3400
#define PM8921_CHG_V_MASK	0x7F
static int __pm_chg_vddmax_set(struct pm8921_chg_chip *chip, int voltage)
{
	int remainder;
	u8 temp = 0;

	if (voltage < PM8921_CHG_VDDMAX_MIN
			|| voltage > PM8921_CHG_VDDMAX_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;

	remainder = voltage % 20;
	if (remainder >= 10) {
		temp |= PM8921_CHG_V_STEP_10MV_OFFSET_BIT;
	}

	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_write(chip, CHG_VDD_MAX, temp);
}

static int pm_chg_vddmax_get(struct pm8921_chg_chip *chip, int *voltage)
{
	u8 temp;
	int rc;

	rc = pm8xxx_readb(chip->dev->parent, CHG_VDD_MAX, &temp);
	if (rc) {
		pr_err("rc = %d while reading vdd max\n", rc);
		*voltage = 0;
		return rc;
	}
	*voltage = (int)(temp & PM8921_CHG_V_MASK) * PM8921_CHG_V_STEP_MV
							+ PM8921_CHG_V_MIN_MV;
	if (temp & PM8921_CHG_V_STEP_10MV_OFFSET_BIT)
		*voltage =  *voltage + 10;
	return 0;
}

static int pm_chg_vddmax_set(struct pm8921_chg_chip *chip, int voltage)
{
	int current_mv, ret, steps, i;
	bool increase;

	ret = 0;

	if (voltage < PM8921_CHG_VDDMAX_MIN
		|| voltage > PM8921_CHG_VDDMAX_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	ret = pm_chg_vddmax_get(chip, &current_mv);
	if (ret) {
		pr_err("Failed to read vddmax rc=%d\n", ret);
		return -EINVAL;
	}
	if (current_mv == voltage)
		return 0;

	/* Only change in increments when USB is present */
	if (is_usb_chg_plugged_in(chip)) {
		if (current_mv < voltage) {
			steps = (voltage - current_mv) / PM8921_CHG_V_STEP_MV;
			increase = true;
		} else {
			steps = (current_mv - voltage) / PM8921_CHG_V_STEP_MV;
			increase = false;
		}
		for (i = 0; i < steps; i++) {
			if (increase)
				current_mv += PM8921_CHG_V_STEP_MV;
			else
				current_mv -= PM8921_CHG_V_STEP_MV;
			ret |= __pm_chg_vddmax_set(chip, current_mv);
		}
	}
	ret |= __pm_chg_vddmax_set(chip, voltage);
	return ret;
}

#define PM8921_CHG_VDDSAFE_MIN	3400
#define PM8921_CHG_VDDSAFE_MAX	4500
static int pm_chg_vddsafe_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VDDSAFE_MIN
			|| voltage > PM8921_CHG_VDDSAFE_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VDD_SAFE, PM8921_CHG_V_MASK, temp);
}

#define PM8921_CHG_VBATDET_MIN	3240
#define PM8921_CHG_VBATDET_MAX	5780
static int pm_chg_vbatdet_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VBATDET_MIN
			|| voltage > PM8921_CHG_VBATDET_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_V_MIN_MV) / PM8921_CHG_V_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VBAT_DET, PM8921_CHG_V_MASK, temp);
}

#define PM8921_CHG_VINMIN_MIN_MV	3800
#define PM8921_CHG_VINMIN_STEP_MV	100
#define PM8921_CHG_VINMIN_USABLE_MAX	6500
#define PM8921_CHG_VINMIN_USABLE_MIN	4300
#define PM8921_CHG_VINMIN_MASK		0x1F
static int pm_chg_vinmin_set(struct pm8921_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < PM8921_CHG_VINMIN_USABLE_MIN
			|| voltage > PM8921_CHG_VINMIN_USABLE_MAX) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - PM8921_CHG_VINMIN_MIN_MV) / PM8921_CHG_VINMIN_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return pm_chg_masked_write(chip, CHG_VIN_MIN, PM8921_CHG_VINMIN_MASK,
									temp);
}

static int pm_chg_vinmin_get(struct pm8921_chg_chip *chip)
{
	u8 temp;
	int rc, voltage_mv;

	rc = pm8xxx_readb(chip->dev->parent, CHG_VIN_MIN, &temp);
	temp &= PM8921_CHG_VINMIN_MASK;

	voltage_mv = PM8921_CHG_VINMIN_MIN_MV +
			(int)temp * PM8921_CHG_VINMIN_STEP_MV;

	return voltage_mv;
}

#define PM8917_USB_UVD_MIN_MV	3850
#define PM8917_USB_UVD_MAX_MV	4350
#define PM8917_USB_UVD_STEP_MV	100
#define PM8917_USB_UVD_MASK	0x7
static int pm_chg_uvd_threshold_set(struct pm8921_chg_chip *chip, int thresh_mv)
{
	u8 temp;

	if (thresh_mv < PM8917_USB_UVD_MIN_MV
			|| thresh_mv > PM8917_USB_UVD_MAX_MV) {
		pr_err("bad mV=%d asked to set\n", thresh_mv);
		return -EINVAL;
	}
	temp = (thresh_mv - PM8917_USB_UVD_MIN_MV) / PM8917_USB_UVD_STEP_MV;
	return pm_chg_masked_write(chip, OVP_USB_UVD,
				PM8917_USB_UVD_MASK, temp);
}

#define PM8921_CHG_IBATMAX_MIN	325
#define PM8921_CHG_IBATMAX_MAX	3025
#define PM8921_CHG_I_MIN_MA	225
#define PM8921_CHG_I_STEP_MA	50
#define PM8921_CHG_I_MASK	0x3F
static int pm_chg_ibatmax_get(struct pm8921_chg_chip *chip, int *ibat_ma)
{
	u8 temp;
	int rc;

	rc = pm8xxx_readb(chip->dev->parent, CHG_IBAT_MAX, &temp);
	if (rc) {
		pr_err("rc = %d while reading ibat max\n", rc);
		*ibat_ma = 0;
		return rc;
	}
	*ibat_ma = (int)(temp & PM8921_CHG_I_MASK) * PM8921_CHG_I_STEP_MA
							+ PM8921_CHG_I_MIN_MA;
	return 0;
}

static int pm_chg_ibatmax_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	if (chg_current < PM8921_CHG_IBATMAX_MIN
			|| chg_current > PM8921_CHG_IBATMAX_MAX) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}
	temp = (chg_current - PM8921_CHG_I_MIN_MA) / PM8921_CHG_I_STEP_MA;
	return pm_chg_masked_write(chip, CHG_IBAT_MAX, PM8921_CHG_I_MASK, temp);
}

#define PM8921_CHG_IBATSAFE_MIN	225
#define PM8921_CHG_IBATSAFE_MAX	3375
static int pm_chg_ibatsafe_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	if (chg_current < PM8921_CHG_IBATSAFE_MIN
			|| chg_current > PM8921_CHG_IBATSAFE_MAX) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}
	temp = (chg_current - PM8921_CHG_I_MIN_MA) / PM8921_CHG_I_STEP_MA;
	return pm_chg_masked_write(chip, CHG_IBAT_SAFE,
						PM8921_CHG_I_MASK, temp);
}

#define PM8921_CHG_ITERM_MIN_MA		50
#define PM8921_CHG_ITERM_MAX_MA		200
#define PM8921_CHG_ITERM_STEP_MA	10
#define PM8921_CHG_ITERM_MASK		0xF
static int pm_chg_iterm_set(struct pm8921_chg_chip *chip, int chg_current)
{
	u8 temp;

	if (chg_current < PM8921_CHG_ITERM_MIN_MA
			|| chg_current > PM8921_CHG_ITERM_MAX_MA) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}

	temp = (chg_current - PM8921_CHG_ITERM_MIN_MA)
				/ PM8921_CHG_ITERM_STEP_MA;
	return pm_chg_masked_write(chip, CHG_ITERM, PM8921_CHG_ITERM_MASK,
					 temp);
}

static int pm_chg_iterm_get(struct pm8921_chg_chip *chip, int *chg_current)
{
	u8 temp;
	int rc;

	rc = pm8xxx_readb(chip->dev->parent, CHG_ITERM, &temp);
	if (rc) {
		pr_err("err=%d reading CHG_ITEM\n", rc);
		*chg_current = 0;
		return rc;
	}
	temp &= PM8921_CHG_ITERM_MASK;
	*chg_current = (int)temp * PM8921_CHG_ITERM_STEP_MA
					+ PM8921_CHG_ITERM_MIN_MA;
	return 0;
}

struct usb_ma_limit_entry {
	int	usb_ma;
	u8	value;
};

/* USB Trim tables */
static int usb_trim_8038_table[USB_TRIM_ENTRIES] = {
	0x0,
	0x0,
	-0x9,
	0x0,
	-0xD,
	0x0,
	-0x10,
	-0x11,
	0x0,
	0x0,
	-0x25,
	0x0,
	-0x28,
	0x0,
	-0x32,
	0x0
};

static int usb_trim_8917_table[USB_TRIM_ENTRIES] = {
	0x0,
	0x0,
	0xA,
	0xC,
	0x10,
	0x10,
	0x13,
	0x14,
	0x13,
	0x3,
	0x1A,
	0x1D,
	0x1D,
	0x21,
	0x24,
	0x26
};

/* Maximum USB  setting table */
static struct usb_ma_limit_entry usb_ma_table[] = {
	{100, 0x0},
	{200, 0x1},
	{500, 0x2},
	{600, 0x3},
	{700, 0x4},
	{800, 0x5},
	{850, 0x6},
	{900, 0x8},
	{950, 0x7},
	{1000, 0x9},
	{1100, 0xA},
	{1200, 0xB},
	{1300, 0xC},
	{1400, 0xD},
	{1500, 0xE},
	{1600, 0xF},
};

#define REG_SBI_CONFIG			0x04F
#define PAGE3_ENABLE_MASK		0x6
#define USB_OVP_TRIM_MASK		0x3F
#define USB_OVP_TRIM_PM8917_MASK	0x7F
#define USB_OVP_TRIM_MIN		0x00
#define REG_USB_OVP_TRIM_ORIG_LSB	0x10A
#define REG_USB_OVP_TRIM_ORIG_MSB	0x09C
#define REG_USB_OVP_TRIM_PM8917		0x2B5
#define REG_USB_OVP_TRIM_PM8917_BIT	BIT(0)
static int pm_chg_usb_trim(struct pm8921_chg_chip *chip, int index)
{
	u8 temp, sbi_config, msb, lsb, mask;
	s8 trim;
	int rc = 0;
	static u8 usb_trim_reg_orig = 0xFF;

	/* No trim data for PM8921 */
	if (!chip->usb_trim_table)
		return 0;

	if (usb_trim_reg_orig == 0xFF) {
		rc = pm8xxx_readb(chip->dev->parent,
				REG_USB_OVP_TRIM_ORIG_MSB, &msb);
		if (rc) {
			pr_err("error = %d reading sbi config reg\n", rc);
			return rc;
		}

		rc = pm8xxx_readb(chip->dev->parent,
				REG_USB_OVP_TRIM_ORIG_LSB, &lsb);
		if (rc) {
			pr_err("error = %d reading sbi config reg\n", rc);
			return rc;
		}

		msb = msb >> 5;
		lsb = lsb >> 5;
		usb_trim_reg_orig = msb << 3 | lsb;

		if (pm8xxx_get_version(chip->dev->parent)
				== PM8XXX_VERSION_8917) {
			rc = pm8xxx_readb(chip->dev->parent,
					REG_USB_OVP_TRIM_PM8917, &msb);
			if (rc) {
				pr_err("error = %d reading config reg\n", rc);
				return rc;
			}

			msb = msb & REG_USB_OVP_TRIM_PM8917_BIT;
			usb_trim_reg_orig |= msb << 6;
		}
	}

	/* use the original trim value */
	trim = usb_trim_reg_orig;

	trim += chip->usb_trim_table[index];
	if (trim < 0)
		trim = 0;

	pr_debug("trim_orig %d write 0x%x index=%d value 0x%x to USB_OVP_TRIM\n",
		usb_trim_reg_orig, trim, index, chip->usb_trim_table[index]);

	rc = pm8xxx_readb(chip->dev->parent, REG_SBI_CONFIG, &sbi_config);
	if (rc) {
		pr_err("error = %d reading sbi config reg\n", rc);
		return rc;
	}

	temp = sbi_config | PAGE3_ENABLE_MASK;
	rc = pm_chg_write(chip, REG_SBI_CONFIG, temp);
	if (rc) {
		pr_err("error = %d writing sbi config reg\n", rc);
		return rc;
	}

	mask = USB_OVP_TRIM_MASK;

	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8917)
		mask = USB_OVP_TRIM_PM8917_MASK;

	rc = pm_chg_masked_write(chip, USB_OVP_TRIM, mask, trim);
	if (rc) {
		pr_err("error = %d writing USB_OVP_TRIM\n", rc);
		return rc;
	}

	rc = pm_chg_write(chip, REG_SBI_CONFIG, sbi_config);
	if (rc) {
		pr_err("error = %d writing sbi config reg\n", rc);
		return rc;
	}
	return rc;
}

#define PM8921_CHG_IUSB_MASK 0x1C
#define PM8921_CHG_IUSB_SHIFT 2
#define PM8921_CHG_IUSB_MAX  7
#define PM8921_CHG_IUSB_MIN  0
#define PM8917_IUSB_FINE_RES BIT(0)
static int pm_chg_iusbmax_set(struct pm8921_chg_chip *chip, int index)
{
	u8 temp, fineres, reg_val;
	int rc;

	reg_val = usb_ma_table[index].value >> 1;
	fineres = PM8917_IUSB_FINE_RES & usb_ma_table[index].value;

	if (reg_val < PM8921_CHG_IUSB_MIN || reg_val > PM8921_CHG_IUSB_MAX) {
		pr_err("bad mA=%d asked to set\n", reg_val);
		return -EINVAL;
	}
	temp = reg_val << PM8921_CHG_IUSB_SHIFT;

	/* IUSB_FINE_RES */
	if (chip->iusb_fine_res) {
		/* Clear IUSB_FINE_RES bit to avoid overshoot */
		rc = pm_chg_masked_write(chip, IUSB_FINE_RES,
			PM8917_IUSB_FINE_RES, 0);

		rc |= pm_chg_masked_write(chip, PBL_ACCESS2,
			PM8921_CHG_IUSB_MASK, temp);

		if (rc) {
			pr_err("Failed to write PBL_ACCESS2 rc=%d\n", rc);
			return rc;
		}

		if (fineres) {
			rc = pm_chg_masked_write(chip, IUSB_FINE_RES,
				PM8917_IUSB_FINE_RES, fineres);
			if (rc) {
				pr_err("Failed to write ISUB_FINE_RES rc=%d\n",
					rc);
				return rc;
			}
		}
	} else {
		rc = pm_chg_masked_write(chip, PBL_ACCESS2,
			PM8921_CHG_IUSB_MASK, temp);
		if (rc) {
			pr_err("Failed to write PBL_ACCESS2 rc=%d\n", rc);
			return rc;
		}
	}

	rc = pm_chg_usb_trim(chip, index);
	if (rc)
			pr_err("unable to set usb trim rc = %d\n", rc);

	return rc;
}

static int pm_chg_iusbmax_get(struct pm8921_chg_chip *chip, int *mA)
{
	u8 temp, fineres;
	int rc, i;

	fineres = 0;
	*mA = 0;
	rc = pm8xxx_readb(chip->dev->parent, PBL_ACCESS2, &temp);
	if (rc) {
		pr_err("err=%d reading PBL_ACCESS2\n", rc);
		return rc;
	}

	if (chip->iusb_fine_res) {
		rc = pm8xxx_readb(chip->dev->parent, IUSB_FINE_RES, &fineres);
		if (rc) {
			pr_err("err=%d reading IUSB_FINE_RES\n", rc);
			return rc;
		}
	}
	temp &= PM8921_CHG_IUSB_MASK;
	temp = temp >> PM8921_CHG_IUSB_SHIFT;

	temp = (temp << 1) | (fineres & PM8917_IUSB_FINE_RES);
	for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
		if (usb_ma_table[i].value == temp)
			break;
	}

	if (i < 0) {
		pr_err("can't find %d in usb_ma_table. Use min.\n", temp);
		i = 0;
	}

	*mA = usb_ma_table[i].usb_ma;

	return rc;
}

#define PM8921_CHG_WD_MASK 0x1F
static int pm_chg_disable_wd(struct pm8921_chg_chip *chip)
{
	/* writing 0 to the wd timer disables it */
	return pm_chg_masked_write(chip, CHG_TWDOG, PM8921_CHG_WD_MASK, 0);
}

#define PM8921_CHG_TCHG_MASK	0x7F
#define PM8921_CHG_TCHG_MIN	4
#define PM8921_CHG_TCHG_MAX	512
#define PM8921_CHG_TCHG_STEP	4
static int pm_chg_tchg_max_set(struct pm8921_chg_chip *chip, int minutes)
{
	u8 temp;

	if (minutes < PM8921_CHG_TCHG_MIN || minutes > PM8921_CHG_TCHG_MAX) {
		pr_err("bad max minutes =%d asked to set\n", minutes);
		return -EINVAL;
	}

	temp = (minutes - 1)/PM8921_CHG_TCHG_STEP;
	return pm_chg_masked_write(chip, CHG_TCHG_MAX, PM8921_CHG_TCHG_MASK,
					 temp);
}

#define PM8921_CHG_TTRKL_MASK	0x3F
#define PM8921_CHG_TTRKL_MIN	1
#define PM8921_CHG_TTRKL_MAX	64
static int pm_chg_ttrkl_max_set(struct pm8921_chg_chip *chip, int minutes)
{
	u8 temp;

	if (minutes < PM8921_CHG_TTRKL_MIN || minutes > PM8921_CHG_TTRKL_MAX) {
		pr_err("bad max minutes =%d asked to set\n", minutes);
		return -EINVAL;
	}

	temp = minutes - 1;
	return pm_chg_masked_write(chip, CHG_TTRKL_MAX, PM8921_CHG_TTRKL_MASK,
					 temp);
}

#define PM8921_CHG_VTRKL_MIN_MV		2050
#define PM8921_CHG_VTRKL_MAX_MV		2800
#define PM8921_CHG_VTRKL_STEP_MV	50
#define PM8921_CHG_VTRKL_SHIFT		4
#define PM8921_CHG_VTRKL_MASK		0xF0
static int pm_chg_vtrkl_low_set(struct pm8921_chg_chip *chip, int millivolts)
{
	u8 temp;

	if (millivolts < PM8921_CHG_VTRKL_MIN_MV
			|| millivolts > PM8921_CHG_VTRKL_MAX_MV) {
		pr_err("bad voltage = %dmV asked to set\n", millivolts);
		return -EINVAL;
	}

	temp = (millivolts - PM8921_CHG_VTRKL_MIN_MV)/PM8921_CHG_VTRKL_STEP_MV;
	temp = temp << PM8921_CHG_VTRKL_SHIFT;
	return pm_chg_masked_write(chip, CHG_VTRICKLE, PM8921_CHG_VTRKL_MASK,
					 temp);
}

#define PM8921_CHG_VWEAK_MIN_MV		2100
#define PM8921_CHG_VWEAK_MAX_MV		3600
#define PM8921_CHG_VWEAK_STEP_MV	100
#define PM8921_CHG_VWEAK_MASK		0x0F
static int pm_chg_vweak_set(struct pm8921_chg_chip *chip, int millivolts)
{
	u8 temp;

	if (millivolts < PM8921_CHG_VWEAK_MIN_MV
			|| millivolts > PM8921_CHG_VWEAK_MAX_MV) {
		pr_err("bad voltage = %dmV asked to set\n", millivolts);
		return -EINVAL;
	}

	temp = (millivolts - PM8921_CHG_VWEAK_MIN_MV)/PM8921_CHG_VWEAK_STEP_MV;
	return pm_chg_masked_write(chip, CHG_VTRICKLE, PM8921_CHG_VWEAK_MASK,
					 temp);
}

#define PM8921_CHG_ITRKL_MIN_MA		50
#define PM8921_CHG_ITRKL_MAX_MA		200
#define PM8921_CHG_ITRKL_MASK		0x0F
#define PM8921_CHG_ITRKL_STEP_MA	10
static int pm_chg_itrkl_set(struct pm8921_chg_chip *chip, int milliamps)
{
	u8 temp;

	if (milliamps < PM8921_CHG_ITRKL_MIN_MA
		|| milliamps > PM8921_CHG_ITRKL_MAX_MA) {
		pr_err("bad current = %dmA asked to set\n", milliamps);
		return -EINVAL;
	}

	temp = (milliamps - PM8921_CHG_ITRKL_MIN_MA)/PM8921_CHG_ITRKL_STEP_MA;

	return pm_chg_masked_write(chip, CHG_ITRICKLE, PM8921_CHG_ITRKL_MASK,
					 temp);
}

#define PM8921_CHG_IWEAK_MIN_MA		325
#define PM8921_CHG_IWEAK_MAX_MA		525
#define PM8921_CHG_IWEAK_SHIFT		7
#define PM8921_CHG_IWEAK_MASK		0x80
static int pm_chg_iweak_set(struct pm8921_chg_chip *chip, int milliamps)
{
	u8 temp;

	if (milliamps < PM8921_CHG_IWEAK_MIN_MA
		|| milliamps > PM8921_CHG_IWEAK_MAX_MA) {
		pr_err("bad current = %dmA asked to set\n", milliamps);
		return -EINVAL;
	}

	if (milliamps < PM8921_CHG_IWEAK_MAX_MA)
		temp = 0;
	else
		temp = 1;

	temp = temp << PM8921_CHG_IWEAK_SHIFT;
	return pm_chg_masked_write(chip, CHG_ITRICKLE, PM8921_CHG_IWEAK_MASK,
					 temp);
}

#define PM8921_CHG_BATT_TEMP_THR_COLD	BIT(1)
#define PM8921_CHG_BATT_TEMP_THR_COLD_SHIFT	1
static int pm_chg_batt_cold_temp_config(struct pm8921_chg_chip *chip,
					enum pm8921_chg_cold_thr cold_thr)
{
	u8 temp;

	temp = cold_thr << PM8921_CHG_BATT_TEMP_THR_COLD_SHIFT;
	temp = temp & PM8921_CHG_BATT_TEMP_THR_COLD;
	return pm_chg_masked_write(chip, CHG_CNTRL_2,
					PM8921_CHG_BATT_TEMP_THR_COLD,
					 temp);
}

#define PM8921_CHG_BATT_TEMP_THR_HOT		BIT(0)
#define PM8921_CHG_BATT_TEMP_THR_HOT_SHIFT	0
static int pm_chg_batt_hot_temp_config(struct pm8921_chg_chip *chip,
					enum pm8921_chg_hot_thr hot_thr)
{
	u8 temp;

	temp = hot_thr << PM8921_CHG_BATT_TEMP_THR_HOT_SHIFT;
	temp = temp & PM8921_CHG_BATT_TEMP_THR_HOT;
	return pm_chg_masked_write(chip, CHG_CNTRL_2,
					PM8921_CHG_BATT_TEMP_THR_HOT,
					 temp);
}

#define PM8921_CHG_LED_SRC_CONFIG_SHIFT	4
#define PM8921_CHG_LED_SRC_CONFIG_MASK	0x30
static int pm_chg_led_src_config(struct pm8921_chg_chip *chip,
				enum pm8921_chg_led_src_config led_src_config)
{
	u8 temp;

	if (led_src_config < LED_SRC_GND ||
			led_src_config > LED_SRC_BYPASS)
		return -EINVAL;

	if (led_src_config == LED_SRC_BYPASS)
		return 0;

	temp = led_src_config << PM8921_CHG_LED_SRC_CONFIG_SHIFT;

	return pm_chg_masked_write(chip, CHG_CNTRL_3,
					PM8921_CHG_LED_SRC_CONFIG_MASK, temp);
}


static int64_t read_battery_id(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	rc = pm8xxx_adc_read(chip->batt_id_channel, &result);
	if (rc) {
		pr_err("error reading batt id channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
	pr_debug("batt_id phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	return result.physical;
}

static int is_battery_valid(struct pm8921_chg_chip *chip)
{
	int64_t rc;

	if (chip->batt_id_min == 0 && chip->batt_id_max == 0)
		return 1;

	rc = read_battery_id(chip);
	if (rc < 0) {
		pr_err("error reading batt id channel = %d, rc = %lld\n",
					chip->vbat_channel, rc);
		/* assume battery id is valid when adc error happens */
		return 1;
	}

	if (rc < chip->batt_id_min || rc > chip->batt_id_max) {
		pr_err("batt_id phy =%lld is not valid\n", rc);
		return 0;
	}
	return 1;
}

static void check_battery_valid(struct pm8921_chg_chip *chip)
{
	if (is_battery_valid(chip) == 0) {
		pr_err("batt_id not valid, disbling charging\n");
		pm_chg_auto_enable(chip, 0);
	} else {
		pm_chg_auto_enable(chip, !charging_disabled);
	}
}

static void battery_id_valid(struct work_struct *work)
{
	struct pm8921_chg_chip *chip = container_of(work,
				struct pm8921_chg_chip, battery_id_valid_work);

	check_battery_valid(chip);
}

static void pm8921_chg_enable_irq(struct pm8921_chg_chip *chip, int interrupt)
{
	if (!__test_and_set_bit(interrupt, chip->enabled_irqs)) {
		dev_dbg(chip->dev, "%d\n", chip->pmic_chg_irq[interrupt]);
		enable_irq(chip->pmic_chg_irq[interrupt]);
	}
}

static void pm8921_chg_disable_irq(struct pm8921_chg_chip *chip, int interrupt)
{
	if (__test_and_clear_bit(interrupt, chip->enabled_irqs)) {
		dev_dbg(chip->dev, "%d\n", chip->pmic_chg_irq[interrupt]);
		disable_irq_nosync(chip->pmic_chg_irq[interrupt]);
	}
}

static int pm8921_chg_is_enabled(struct pm8921_chg_chip *chip, int interrupt)
{
	return test_bit(interrupt, chip->enabled_irqs);
}

//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
static int is_chg_plugged_in(struct pm8921_chg_chip *chip)
{
/*	int pres, ov, uv;
	
//TODO: check if just read USBIN_VALID_IRQ is work
	pres = pm_chg_get_rt_status(chip, USBIN_VALID_IRQ);
	ov = pm_chg_get_rt_status(chip, USBIN_OV_IRQ);
	uv = pm_chg_get_rt_status(chip, USBIN_UV_IRQ);

	printk("[BAT]pres:%d,ov:%d,uv:%d\n",pres,ov,uv);

	return pres && !ov && !uv;
*/
    return pm_chg_get_rt_status(chip, USBIN_VALID_IRQ);
}
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"

static bool is_ext_charging(struct pm8921_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (!chip->ext_psy)
		return false;
	if (chip->ext_psy->get_property(chip->ext_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &ret))
		return false;
	if (ret.intval > POWER_SUPPLY_CHARGE_TYPE_NONE)
		return ret.intval;

	return false;
}

static bool is_ext_trickle_charging(struct pm8921_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (!chip->ext_psy)
		return false;
	if (chip->ext_psy->get_property(chip->ext_psy,
					POWER_SUPPLY_PROP_CHARGE_TYPE, &ret))
		return false;
	if (ret.intval == POWER_SUPPLY_CHARGE_TYPE_TRICKLE)
		return true;

	return false;
}

static int is_battery_charging(int fsm_state)
{
	if (is_ext_charging(the_chip))
		return 1;

	switch (fsm_state) {
	case FSM_STATE_ATC_2A:
	case FSM_STATE_ATC_2B:
	case FSM_STATE_ON_CHG_AND_BAT_6:
	case FSM_STATE_FAST_CHG_7:
	case FSM_STATE_TRKL_CHG_8:
		return 1;
	}
	return 0;
}

static void bms_notify(struct work_struct *work)
{
	struct bms_notify *n = container_of(work, struct bms_notify, work);

	if (n->is_charging) {
		pm8921_bms_charging_began();
	} else {
		pm8921_bms_charging_end(n->is_battery_full);
		n->is_battery_full = 0;
	}
}

static void bms_notify_check(struct pm8921_chg_chip *chip)
{
	int fsm_state, new_is_charging;

	fsm_state = pm_chg_get_fsm_state(chip);
	new_is_charging = is_battery_charging(fsm_state);

	if (chip->bms_notify.is_charging ^ new_is_charging) {
		chip->bms_notify.is_charging = new_is_charging;
		schedule_work(&(chip->bms_notify.work));
	}
}

static enum power_supply_property pm_power_props_usb[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_HEALTH,
};

static enum power_supply_property pm_power_props_mains[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static char *pm_power_supplied_to[] = {
	"battery",
#ifdef CONFIG_TOUCHSCREEN_CHARGER_NOTIFY
	"touch",
#endif
};

#define USB_WALL_THRESHOLD_MA	500
#ifdef CONFIG_MAXIM_8934_CHARGER
static int pm_power_get_property_mains(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	int type;

	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;

		if (the_chip->has_dc_supply) {
			val->intval = 1;
			return 0;
		}

		if (the_chip->dc_present) {
			val->intval = 1;
			return 0;
		}

		type = the_chip->usb_psy.type;
		if (type == POWER_SUPPLY_TYPE_USB_DCP ||
			type == POWER_SUPPLY_TYPE_USB_ACA ||
			type == POWER_SUPPLY_TYPE_USB_CDP)
			val->intval = is_usb_chg_plugged_in(the_chip);

		break;
	default:
		return -EINVAL;
	}
	return 0;
}
#endif

static int disable_aicl(int disable)
{
	if (disable != POWER_SUPPLY_HEALTH_UNKNOWN
		&& disable != POWER_SUPPLY_HEALTH_GOOD) {
		pr_err("called with invalid param :%d\n", disable);
		return -EINVAL;
	}

	if (!the_chip) {
		pr_err("%s called before init\n", __func__);
		return -EINVAL;
	}

	pr_debug("Disable AICL = %d\n", disable);
	the_chip->disable_aicl = disable;
	return 0;
}

static int switch_usb_to_charge_mode(struct pm8921_chg_chip *chip)
{
	int rc;

	if (!chip->host_mode)
		return 0;

	/* enable usbin valid comparator and remove force usb ovp fet off */
	rc = pm_chg_write(chip, USB_OVP_TEST, 0xB2);
	if (rc < 0) {
		pr_err("Failed to write 0xB2 to USB_OVP_TEST rc = %d\n", rc);
		return rc;
	}

	chip->host_mode = 0;

	return 0;
}

static int switch_usb_to_host_mode(struct pm8921_chg_chip *chip)
{
	int rc;

	if (chip->host_mode)
		return 0;

	/* disable usbin valid comparator and force usb ovp fet off */
	rc = pm_chg_write(chip, USB_OVP_TEST, 0xB3);
	if (rc < 0) {
		pr_err("Failed to write 0xB3 to USB_OVP_TEST rc = %d\n", rc);
		return rc;
	}

	chip->host_mode = 1;

	return 0;
}

static int pm_power_set_property_usb(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_SCOPE:
		if (val->intval == POWER_SUPPLY_SCOPE_SYSTEM)
			return switch_usb_to_host_mode(the_chip);
		if (val->intval == POWER_SUPPLY_SCOPE_DEVICE)
			return switch_usb_to_charge_mode(the_chip);
		else
			return -EINVAL;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		return pm8921_set_usb_power_supply_type(val->intval);
	case POWER_SUPPLY_PROP_HEALTH:
		/* UNKNOWN(0) means enable aicl, GOOD(1) means disable aicl */
		return disable_aicl(val->intval);
	default:
		return -EINVAL;
	}
	return 0;
}

static int usb_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_HEALTH:
		return 1;
	default:
		break;
	}

	return 0;
}

static int pm_power_get_property_usb(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	int current_max;

	/* Check if called before init */
	if (!the_chip)
		return -EINVAL;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (pm_is_chg_charge_dis(the_chip)) {
			val->intval = 0;
		} else {
			pm_chg_iusbmax_get(the_chip, &current_max);
			val->intval = current_max;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;

		if (the_chip->usb_psy.type == POWER_SUPPLY_TYPE_USB)
			val->intval = is_usb_chg_plugged_in(the_chip);

		break;

	case POWER_SUPPLY_PROP_SCOPE:
		if (the_chip->host_mode)
			val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		else
			val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		/* UNKNOWN(0) means enable aicl, GOOD(1) means disable aicl */
		val->intval = the_chip->disable_aicl;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
};

static int get_prop_battery_uvolts(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

    pr_debug("%s()+++",__FUNCTION__);
	rc = pm8xxx_adc_read(chip->vbat_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
	pr_debug("mvolts phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	//ASUS BSP +++ Eason Chang add BAT global variable					
	gBatteryVol = (int)result.physical;			
	//ASUS BSP --- Eason Chang add BAT global variable		
	return (int)result.physical;
}

static unsigned int voltage_based_capacity(struct pm8921_chg_chip *chip)
{
	unsigned int current_voltage_uv = get_prop_battery_uvolts(chip);
	unsigned int current_voltage_mv = current_voltage_uv / 1000;
	unsigned int low_voltage = chip->min_voltage_mv;
	unsigned int high_voltage = chip->max_voltage_mv;

	if (current_voltage_mv <= low_voltage)
		return 0;
	else if (current_voltage_mv >= high_voltage)
		return 100;
	else
		return (current_voltage_mv - low_voltage) * 100
		    / (high_voltage - low_voltage);
}

static int get_prop_batt_present(struct pm8921_chg_chip *chip)
{
   //ASUS BSP +++ Eason Chang add BAT global variable
   gBatteryPresent = pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
   //ASUS BSP --- Eason Chang add BAT global variable
	return pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
}

static int get_prop_batt_status(struct pm8921_chg_chip *chip)
{
	int batt_state = POWER_SUPPLY_STATUS_DISCHARGING;
	int fsm_state = pm_chg_get_fsm_state(chip);
	int i;

	if (chip->ext_psy) {
		if (chip->ext_charge_done)
			return POWER_SUPPLY_STATUS_FULL;
		if (chip->ext_charging)
			return POWER_SUPPLY_STATUS_CHARGING;
	}

	for (i = 0; i < ARRAY_SIZE(map); i++)
		if (map[i].fsm_state == fsm_state)
			batt_state = map[i].batt_state;

	if (fsm_state == FSM_STATE_ON_CHG_HIGHI_1) {
		if (!pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ)
			|| !pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ)
			|| pm_chg_get_rt_status(chip, CHGHOT_IRQ)
			|| pm_chg_get_rt_status(chip, VBATDET_LOW_IRQ))

			batt_state = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	return batt_state;
}

static int get_prop_batt_capacity(struct pm8921_chg_chip *chip)
{
	int percent_soc;

	if (chip->battery_less_hardware)
		return 100;

	if (!get_prop_batt_present(chip))
    {//ASUS_BSP Eason   
		percent_soc = voltage_based_capacity(chip);
        printk("[BAT][Bms]:battery not present\n");//ASUS_BSP Eason   
    }//ASUS_BSP Eason           
	else
		percent_soc = pm8921_bms_get_percent_charge();

	if (percent_soc == -ENXIO)
    {//ASUS_BSP Eason      
		percent_soc = voltage_based_capacity(chip);
        printk("[BAT][Bms]: No such device or address\n");//ASUS_BSP Eason   
    }//ASUS_BSP Eason  

	if (percent_soc <= 10)
		pr_warn_ratelimited("low battery charge = %d%%\n",
						percent_soc);

	if (percent_soc <= chip->resume_charge_percent
		&& get_prop_batt_status(chip) == POWER_SUPPLY_STATUS_FULL) {
		pr_debug("soc fell below %d. charging enabled.\n",
						chip->resume_charge_percent);
		if (chip->is_bat_warm)
			pr_warn_ratelimited("battery is warm = %d, do not resume charging at %d%%.\n",
					chip->is_bat_warm,
					chip->resume_charge_percent);
		else if (chip->is_bat_cool)
			pr_warn_ratelimited("battery is cool = %d, do not resume charging at %d%%.\n",
					chip->is_bat_cool,
					chip->resume_charge_percent);
		else
			pm_chg_vbatdet_set(the_chip, PM8921_CHG_VBATDET_MAX);
	}

	chip->recent_reported_soc = percent_soc;
	printk("[BAT][Bms]:%d\n", percent_soc);//ASUS_BSP Eason 
	return percent_soc;
}

//Eason: choose Capacity type SWGauge/BMS +++ 
int get_BMS_capacity(void)
{
	int bms_soc;
	int adjust_bms_soc;
	bms_soc = get_prop_batt_capacity(the_chip);

      if(bms_soc>=80)
  	{
		adjust_bms_soc = bms_soc+2;
      	}else if( (bms_soc<80)&&(bms_soc>=60) )
      	{
      		adjust_bms_soc = bms_soc+1;
	}else
	{
		adjust_bms_soc = bms_soc;
	}
	
	if(adjust_bms_soc>100)
	{
		adjust_bms_soc = 100;
	}else if(adjust_bms_soc<0){
		adjust_bms_soc = 0;
	}	

	printk("[BAT][Bms][adjust]:%d\n", adjust_bms_soc);

	return  adjust_bms_soc;
}
//Eason: choose Capacity type SWGauge/BMS ---
static int get_prop_batt_current_max(struct pm8921_chg_chip *chip)
{
	int curr = pm8921_bms_get_current_max();
	if (curr == -EINVAL)
		return -EINVAL;

	//ASUS BSP +++ Eason Chang add BAT global variable
	gBatteryCurrentMax = (curr)*1000;
	//ASUS BSP --- Eason Chang add BAT global variable

	return curr;
}

static int get_prop_batt_current(struct pm8921_chg_chip *chip)
{
	int result_ua, rc;

	rc = pm8921_bms_get_battery_current(&result_ua);
	if (rc == -ENXIO) {
		rc = pm8xxx_ccadc_get_battery_current(&result_ua);
	}

	if (rc) {
		pr_err("unable to get batt current rc = %d\n", rc);
		return rc;
	} else {
		//ASUS BSP +++ Eason Chang add BAT global variable
		gBatteryCurrent = result_ua;
		//ASUS BSP --- Eason Chang add BAT global variable
		return result_ua;
	}
}

static int get_prop_batt_fcc(struct pm8921_chg_chip *chip)
{
	int rc;

	rc = pm8921_bms_get_fcc();
	if (rc < 0)
		pr_err("unable to get batt fcc rc = %d\n", rc);
		//ASUS BSP +++ Eason Chang add BAT global variable
       gBatteryFcc = rc;
		//ASUS BSP --- Eason Chang add BAT global variable
	return rc;
}

static int get_prop_batt_charge_now(struct pm8921_chg_chip *chip)
{
	int rc;
	int cc_uah;

	rc = pm8921_bms_cc_uah(&cc_uah);

	if (rc == 0)
		return cc_uah;

	pr_err("unable to get batt fcc rc = %d\n", rc);
	return rc;
}

static int get_prop_batt_health(struct pm8921_chg_chip *chip)
{
	int temp;

	temp = pm_chg_get_rt_status(chip, BATTTEMP_HOT_IRQ);
	if (temp) {
		gBatteryHealth = POWER_SUPPLY_HEALTH_OVERHEAT;//ASUS BSP +++ Eason Chang add BAT global variable
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	}

	temp = pm_chg_get_rt_status(chip, BATTTEMP_COLD_IRQ);
	if (temp) {
		gBatteryHealth = POWER_SUPPLY_HEALTH_COLD;//ASUS BSP +++ Eason Chang add BAT global variable
		return POWER_SUPPLY_HEALTH_COLD;
	}

	gBatteryHealth = POWER_SUPPLY_HEALTH_GOOD;//ASUS BSP +++ Eason Chang add BAT global variable
	return POWER_SUPPLY_HEALTH_GOOD;
}

static int get_prop_charge_type(struct pm8921_chg_chip *chip)
{
	int temp;

	if (!get_prop_batt_present(chip))
	{
		gChargeStatus = POWER_SUPPLY_CHARGE_TYPE_NONE;//ASUS BSP +++ Eason Chang add BAT global variable
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}
	if (is_ext_trickle_charging(chip))
	{
		gChargeStatus = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;//ASUS BSP +++ Eason Chang add BAT global variable
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	}
	if (is_ext_charging(chip))
	{
		gChargeStatus = POWER_SUPPLY_CHARGE_TYPE_FAST;//ASUS BSP +++ Eason Chang add BAT global variable
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	}

	temp = pm_chg_get_rt_status(chip, TRKLCHG_IRQ);
	if (temp)
	{
		gChargeStatus = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;//ASUS BSP +++ Eason Chang add BAT global variable
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	}

	temp = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
	if (temp)
	{
		gChargeStatus = POWER_SUPPLY_CHARGE_TYPE_FAST;//ASUS BSP +++ Eason Chang add BAT global variable
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	}
	gChargeStatus = POWER_SUPPLY_CHARGE_TYPE_NONE;//ASUS BSP +++ Eason Chang add BAT global variable
	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

#define MAX_TOLERABLE_BATT_TEMP_DDC	680
static int get_prop_batt_temp(struct pm8921_chg_chip *chip)
{
	int rc;
	struct pm8xxx_adc_chan_result result;

	if (chip->battery_less_hardware)
		return 300;

	rc = pm8xxx_adc_read(chip->batt_temp_channel, &result);
	if (rc) {
		pr_err("error reading adc channel = %d, rc = %d\n",
					chip->vbat_channel, rc);
		return rc;
	}
	pr_debug("batt_temp phy = %lld meas = 0x%llx\n", result.physical,
						result.measurement);
	if (result.physical > MAX_TOLERABLE_BATT_TEMP_DDC)
		pr_err("BATT_TEMP= %d > 68degC, device will be shutdown\n",
							(int) result.physical);

 
	//carol+, workaround, to avoid auto shutdown for PMIC V1 chip, V2/V3 need to test 
       if( result.physical > 500 ) 
       { 
		pr_err("warning, batt_temp phy = %lld meas = 0x%llx\n", result.physical,
							result.measurement);
		result.physical = 300; 
		pr_err("is_bat_warm=%d, set temp to 30\n", chip->is_bat_warm ); 
	} 
	//carol-
   	gBatteryTemp = (int)result.physical;
	return (int)result.physical;
}

//ASUS BSP +++ Eason Chang add BAT global variable
void UpdateGlobalValue(void)
{
    pr_debug("[BAT][8921]%s()+++\n",__FUNCTION__);
	 //asus_bat_report_phone_status(get_prop_batt_status(the_chip));
	get_prop_battery_uvolts(the_chip);
	get_prop_batt_health(the_chip);
	get_prop_charge_type(the_chip);
	get_prop_batt_present(the_chip);
    //asus_bat_report_phone_capacity(get_prop_batt_capacity(the_chip));
	get_prop_batt_current(the_chip);
	get_prop_batt_current_max(the_chip);
   get_prop_batt_temp(the_chip);
   get_prop_batt_fcc(the_chip);
   pr_debug("[BAT][8921]%s()---\n",__FUNCTION__);
}
//ASUS BSP --- Eason Chang add BAT global variable

///+++ASUS_BSP+++ Eason_Chang  ASUS SW gauge
int get_temp_for_ASUSswgauge(void)
{ 
  if (the_chip == NULL){
  		printk("[BAT][SWgauge]%s():the_chip == NULL",__FUNCTION__);
        return 25;
  }else{	
  		return get_prop_batt_temp(the_chip)/10;
  }
}

int get_voltage_for_ASUSswgauge(void)
{ 
  if (the_chip == NULL){
 	 	printk("[BAT][SWgauge]%s():the_chip == NULL",__FUNCTION__);
		return 3700;
  }else{
		return get_prop_battery_uvolts(the_chip)/1000;
  }		
}

int get_current_for_ASUSswgauge(void)
{ 
  if (the_chip == NULL){
  		printk("[BAT][SWgauge]%s():the_chip == NULL",__FUNCTION__);
		return 500;
  }else{
        return get_prop_batt_current(the_chip)/1000;
  }		
}
///---ASUS_BSP--- Eason_Chang  ASUS SW gauge

static int pm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct pm8921_chg_chip *chip = container_of(psy, struct pm8921_chg_chip,
								batt_psy);

//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
	enum asus_bat_charger_cable charger;
	int bat_status;
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
		bat_status = get_prop_batt_status(chip);
		// set charger here, not in ac/usb irq context
		if (chip->usb_present) {
			charger = ASUS_BAT_CHARGER_USB;
		} else if (chip->dc_present) {
			charger = ASUS_BAT_CHARGER_AC;
		} else if (POWER_SUPPLY_STATUS_CHARGING == bat_status) {
			// special case: inserted in p01
			charger = ASUS_BAT_CHARGER_OTHER_BAT;
		} else {
			charger = ASUS_BAT_CHARGER_NONE;
		}
		asus_bat_report_phone_charger(charger);

		val->intval = asus_bat_report_phone_status(bat_status);

#else
		val->intval = get_prop_batt_status(chip);
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
        #if 0	
		val->intval = get_prop_charge_type(chip);
        #else
		val->intval = gChargeStatus;
		pr_debug("POWER_SUPPLY_PROP_CHARGE_TYPE = %d\r\n",val->intval);
        #endif		
		break;
	case POWER_SUPPLY_PROP_HEALTH:
        #if 0	
		val->intval = get_prop_batt_health(chip);
        #else
		val->intval = gBatteryHealth;
		pr_debug("POWER_SUPPLY_PROP_HEALTH = %d\r\n",val->intval);
        #endif		
		break;
	case POWER_SUPPLY_PROP_PRESENT:
//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
        #if 0
		val->intval = asus_bat_report_phone_present(get_prop_batt_present(chip));
		pr_debug("[BAT]%s(), phone bat present val:%d \r\n", __FUNCTION__, val->intval);
        #else
		val->intval = gBatteryPresent;
		pr_debug("POWER_SUPPLY_PROP_PRESENT = %d\r\n",val->intval);
        #endif		
#else
		val->intval = get_prop_batt_present(chip);
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->max_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->min_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        #if 0	
		val->intval = get_prop_battery_uvolts(chip);
        #else
		val->intval = gBatteryVol;
		pr_debug("POWER_SUPPLY_PROP_VOLTAGE_NOW = %d\r\n",val->intval);
        #endif		
//ASUS_BSP +++ Josh_Liao "add asus battery driver"
		pr_debug( "[BAT]%s(), POWER_SUPPLY_PROP_VOLTAGE_NOW:%d \r\n", __FUNCTION__, val->intval);
//ASUS_BSP --- Josh_Liao "add asus battery driver"
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS

		val->intval = asus_bat_report_phone_capacity(get_prop_batt_capacity(chip));
		//asus_bat_write_phone_bat_capacity_tofile();

#else
		val->intval = get_prop_batt_capacity(chip);
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
        #if 0	
		val->intval = get_prop_batt_current(chip);
        #else
		val->intval = gBatteryCurrent;
		pr_debug("POWER_SUPPLY_PROP_CURRENT_NOW = %d\r\n",val->intval);
        #endif		
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	#if 0
		val->intval = get_prop_batt_current_max(chip);
	#else
		val->intval = gBatteryCurrentMax;
		pr_debug("POWER_SUPPLY_PROP_CURRENT_NOW_MAX = %d\r\n",val->intval);
	#endif
		break;
	case POWER_SUPPLY_PROP_TEMP:
        #if 0	
		val->intval = get_prop_batt_temp(chip);
        #else
		val->intval = gBatteryTemp;
		pr_debug("POWER_SUPPLY_PROP_TEMP = %d\r\n",val->intval);
        #endif		
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
        #if 0	
		val->intval = get_prop_batt_fcc(chip);
        #else
		val->intval = gBatteryFcc;
		pr_debug("POWER_SUPPLY_PROP_ENERGY_FULL val=%d\r\n",val->intval);
        #endif	
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = get_prop_batt_charge_now(chip);
		break;
	default:
		return -EINVAL;
	}

    //Eason if doesn't get correct ADC Vol&Curr at first update Cap show unknow status +++ 
    if(false == g_adc_get_correct_VolCurr)
    {
        return -EINVAL;
    }else{    
	    return 0;
    }
    //Eason if doesn't get correct ADC Vol&Curr at first update Cap show unknow status ---
}

//ASUS_BSP +++ Josh_Liao "add asus battery driver"
static void api_phone_bat_power_supply_changed(void)
{
	power_supply_changed(&local_chip->batt_psy);
}

static int api_get_prop_batt_present(void)
{
	return get_prop_batt_present(local_chip);
}

static int api_get_prop_batt_capacity(void)
{
	return get_prop_batt_capacity(local_chip);
}

static int api_get_prop_batt_status(void)
{
	return get_prop_batt_status(local_chip);
}

//ASUS_BSP +++ Eason_Chang "to check if chgin"
static int api_get_prop_batt_chgin(void)
{
	return is_chg_plugged_in(local_chip);
}
//ASUS_BSP --- Eason_Chang "to check if chgin"

static int api_get_prop_charge_type(void)
{
	return get_prop_charge_type(local_chip);

}

static int api_get_prop_batt_health(void)
{
	return get_prop_batt_health(local_chip);
}

static int api_get_prop_batt_volt(void)
{
	return get_prop_battery_uvolts(local_chip);
}

static int api_get_prop_batt_curr(void)
{
	return get_prop_batt_current(local_chip);
}

extern enum asus_chg_src asus_chg_get_chg_mode(void)
{
	return local_chg_src;
}

#ifdef CONFIG_CHARGER_ASUS
static DEFINE_SPINLOCK(asus_chg_mode_lock);
#endif /* CONFIG_CHARGER_ASUS */
//static DEFINE_MUTEX(asus_chg_curr_lock);

/* boot_check_for_set_dc_curr - Even detecting DC plugged in, we should not draw 1500mA during 30s 
  * from booting. Because it may be just connected to p01 and p01_add not be notified. We should
  * wait for all state ready to determine if need to set DC charging current.
  */    
static bool boot_check_for_set_dc_curr = false;

static void asus_chg_set_dc_curr_work(struct work_struct *work)
{
//	mutex_lock(&asus_chg_curr_lock);
#ifdef CONFIG_EEPROM_NUVOTON
	int p01_cont = AX_MicroP_IsP01Connected();
#else
	int p01_cont = 0;
#endif /* CONFIG_EEPROM_NUVOTON */

	printk(DBGMSK_BAT_DEBUG "[BAT]%s(), P01 connected:%d \r\n", __FUNCTION__, p01_cont);
			
	if ((ASUS_CHG_SRC_DC == local_chg_src) && 
		(!p01_cont))
	{
		pr_debug(DBGMSK_BAT_INFO "[BAT][8921],charging current:%d \r\n", AC_CHG_CURR);	
		pm8921_charger_vbus_draw(AC_CHG_CURR);		
	}

	if (!boot_check_for_set_dc_curr) {
		printk(DBGMSK_BAT_INFO "[BAT]%s(), first time to check if need to set charging dc current after boot. \r\n", __FUNCTION__);
		boot_check_for_set_dc_curr = true;
	}
	return;
//	mutex_unlock(&asus_chg_curr_lock);

}

//ASUS BSP+++ Eason notify BatteryService   
#if 0
static void checkIf8921FsmStateChange(void)
{
        int new8921State;
        bool new8921StateChargig = 0;
        bool Saved8921StateCharging = 0;

        new8921State = pm_chg_get_fsm_state(the_chip);

        switch(new8921State){
		case FSM_STATE_ATC_2A:
		case FSM_STATE_ATC_2B:
		case FSM_STATE_ON_CHG_AND_BAT_6:
		case FSM_STATE_FAST_CHG_7:
		case FSM_STATE_TRKL_CHG_8:
            	 	new8921StateChargig = 1;
        	default:
            	 	new8921StateChargig = 0;
       }       

        switch(gSaved8921FsmState){
		case FSM_STATE_ATC_2A:
		case FSM_STATE_ATC_2B:
		case FSM_STATE_ON_CHG_AND_BAT_6:
		case FSM_STATE_FAST_CHG_7:
		case FSM_STATE_TRKL_CHG_8:
           	  	 Saved8921StateCharging = 1;
       		default:
             	 	Saved8921StateCharging = 0;
       }       

       if( 1==Saved8921StateCharging  &&  0==new8921StateChargig )
        		asus_onChargingStop(ex8921FSM_change);
       if( 0==Saved8921StateCharging  &&  1==new8921StateChargig )
                        asus_onChargingStart();

       gSaved8921FsmState= printk("state_changed_to=%d\n", pm_chg_get_fsm_state(the_chip));
}
#endif
//ASUS BSP--- Eason notify BatteryService  

static void pm8921_power_supply_changed(struct work_struct *work)
{
	printk(DBGMSK_BAT_DEBUG "[BAT] power_supply_changed(all) in %s() \r\n", __FUNCTION__);

    //ASUS BSP+++ Eason notify BatteryService        
    //checkIf8921FsmStateChange();
    //ASUS BSP--- Eason notify BatteryService

	//power_supply_changed(&the_chip->batt_psy);//ASUS BSP+++ Eason notify BatteryService 
//+++ ASUS_BSP Allen1_Huang: enable ICS charger mode
#ifdef CONFIG_CHARGER_MODE
	if (g_CHG_mode) {
		power_supply_changed(&the_chip->usb_psy);
		power_supply_changed(&the_chip->dc_psy);
	}
#endif
//--- ASUS_BSP Allen1_Huang: enable ICS charger mode
}

DECLARE_DELAYED_WORK(asus_chg_set_dc_curr_worker, asus_chg_set_dc_curr_work);

DECLARE_DELAYED_WORK(pm_ps_changed_work, pm8921_power_supply_changed);

#ifdef CONFIG_CHARGER_ASUS
//BY default.... it will check the during all the time when charger in
//Check chg done firse...
//todo : maybe we can add more 
void change_chg_statue_check_interval(int interval)
{
        the_chip->chg_state_polling_interval = interval;
        
	if (delayed_work_pending(&the_chip->chg_status_check_work)) {
		cancel_delayed_work_sync(&the_chip->chg_status_check_work);
		schedule_delayed_work(&the_chip->chg_status_check_work, the_chip->chg_state_polling_interval *HZ);
	} else {
		schedule_delayed_work(&the_chip->chg_status_check_work, the_chip->chg_state_polling_interval *HZ);
	}        
}
static void chg_status_check_work(struct work_struct *work)
{
    unsigned long flags;
    struct delayed_work *dwork = to_delayed_work(work);
    struct pm8921_chg_chip *chip = container_of(dwork,
    			struct pm8921_chg_chip, chg_status_check_work);

        if(g_A60K_hwID >=A66_HW_ID_ER2){


            return ;
        }

        spin_lock_irqsave(&asus_chg_mode_lock, flags);

        if(is_chg_plugged_in(chip)){

            int iterm_programmed;

            int fsm_state = pm_chg_get_fsm_state(chip);

            pm_chg_iterm_get(chip, &iterm_programmed);

            printk("[pm8921]fsm:%d,INSERTED:%d,TEMPOK:%d,CHGHOT:%d,VBATLOW:%d\n"
                ,fsm_state
                ,pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ)
                ,pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ)
                ,pm_chg_get_rt_status(chip, CHGHOT_IRQ)
                ,pm_chg_get_rt_status(chip, VBATDET_LOW_IRQ));
            printk("[pm8921]current:%d,eoc threshold:%d\n"
                ,(get_prop_batt_current(chip)) / 1000
                ,iterm_programmed);

            if(chip->chgdone_reported == false){

                if ((fsm_state == FSM_STATE_ON_CHG_HIGHI_1 && pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ))
                    || chip->fastchg_done == true) {

                    printk("[PM8921]charging done!\n");

                    asus_onChargingStop(exCHGDOWN);                    

                    chip->chgdone_reported = true;
                }

                
                if (!delayed_work_pending(&chip->chg_status_check_work)){
                
                    //for debunce the state of fsm of pmic
                    schedule_delayed_work(&chip->chg_status_check_work,
                                  chip->chg_state_polling_interval*HZ);           
                }

                if (!delayed_work_pending(&chip->eoc_work)) {
                 	schedule_delayed_work(&chip->eoc_work,
                			      round_jiffies_relative(msecs_to_jiffies
                					     (EOC_CHECK_PERIOD_MS)));
                    
                    printk("[PM8921]Rech-eoc-worker\n");
                }                
  
            }
            
        }else{

            printk("[PM8921]End of watching status\n");

            chip->chgdone_reported = false;

            chip->fastchg_done = false;
        }        
        spin_unlock_irqrestore(&asus_chg_mode_lock, flags);  
}
extern void pm8921_chg_enable_charging(bool enabled)
{
        if(g_A60K_hwID >=A66_HW_ID_ER2){    
            return;
        }

    if(true == enabled){

        switch (local_chg_src) {
            case ASUS_CHG_SRC_USB:
                pm8921_charger_vbus_draw(USB_CHG_CURR);
                break;
            case ASUS_CHG_SRC_DC:
                pm8921_charger_vbus_draw(AC_CHG_CURR);
                break;
            case ASUS_CHG_SRC_PAD_BAT:
                pm8921_charger_vbus_draw(USB_CHG_CURR);
                break;
            default:
                break;
        }
    }else{
        pm8921_charger_vbus_draw(NONE_CHG_CURR);
    }
}
EXPORT_SYMBOL_GPL(pm8921_chg_enable_charging);
extern void pm8921_chg_set_chg_mode(enum asus_chg_src chg_src)
{

	unsigned long flags;

    if(g_A60K_hwID >=A66_HW_ID_ER2){    
        return;
    }

	spin_lock_irqsave(&asus_chg_mode_lock, flags);	
	local_chg_src = chg_src;

	printk(DBGMSK_BAT_INFO "[BAT][8921] chg_src:%d \r\n", chg_src);

	if (NULL == the_chip) {
		printk(DBGMSK_BAT_INFO "[BAT]%s(), NULL == the_chip \r\n", __FUNCTION__);
		spin_unlock_irqrestore(&asus_chg_mode_lock, flags);
		return;
	}

    asus_onCableInOut(chg_src);

	switch (chg_src) {
	case ASUS_CHG_SRC_USB:
		pm8921_charger_vbus_draw(USB_CHG_CURR);
		the_chip->usb_present = 1;
		the_chip->dc_present = 0;
		break;
	case ASUS_CHG_SRC_DC:
		/* draw 500 mA first, then draw 1500mA when making sure p01 disconnected */
		pm8921_charger_vbus_draw(USB_CHG_CURR);
		the_chip->dc_present = 1;
		the_chip->usb_present = 0;
		if (boot_check_for_set_dc_curr) 
			schedule_delayed_work(&asus_chg_set_dc_curr_worker, 1.5*HZ);
		break;
	case ASUS_CHG_SRC_NONE:
///+++ASUS_BSP+++ Eason_Chang  ASUS SW gauge
		asus_set_not_charge_done();
///+++ASUS_BSP--- Eason_Chang  ASUS SW gauge		
		pm8921_charger_vbus_draw(NONE_CHG_CURR);
		the_chip->dc_present = 0;
		the_chip->usb_present = 0;		
		break;
	case ASUS_CHG_SRC_PAD_BAT:
        // ASUS_BSP +++ Sina: Only P02 support VBUS draw 1500 mA. Otherwise, to avoid crash, we allow to draw 500 mA.
              if(AX_MicroP_isFWSupportP02()){
                        //pm8921_charger_vbus_draw(AC_CHG_CURR); //sina
                        //Victor@20120112: avoid some hang or some error of audio, asked by Tom
                        pm8921_charger_vbus_draw(USB_CHG_CURR); 
        	}else{
                        pm8921_charger_vbus_draw(USB_CHG_CURR); //sina
             }
        // ASUS_BSP ---                        
		the_chip->dc_present = 1;
		the_chip->usb_present = 0;		
		break;
	default:
		printk(DBGMSK_BAT_ERR "[BAT]%s(), unknown charger mode. \r\n", __FUNCTION__);
	}

	spin_unlock_irqrestore(&asus_chg_mode_lock, flags);

//ASUS_BSP +++ Josh_Liao "add asus battery driver"
	if (delayed_work_pending(&pm_ps_changed_work)) {
		printk(DBGMSK_BAT_INFO "[BAT] re-queue pm_ps_changed_work \r\n");
		cancel_delayed_work_sync(&pm_ps_changed_work);
		schedule_delayed_work(&pm_ps_changed_work, 1*HZ);
	} else {
		schedule_delayed_work(&pm_ps_changed_work, 1*HZ);
	}
//ASUS_BSP --- Josh_Liao "add asus battery driver"
	//power_supply_changed(&the_chip->usb_psy);
	//power_supply_changed(&the_chip->dc_psy);	
		
	return;
}
EXPORT_SYMBOL_GPL(pm8921_chg_set_chg_mode);
#endif /* CONFIG_CHARGER_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"

//ASUS BSP +++ Eason Chang add BAT global variable
static void  UpdateGlobalValue_work(struct work_struct *work)
{
    struct delayed_work *dwork = to_delayed_work(work);
    struct pm8921_chg_chip *chip = container_of(dwork,
    			struct pm8921_chg_chip, update_BATinfo_worker);

    UpdateGlobalValue();

    schedule_delayed_work(&chip->update_BATinfo_worker,UPDATE_BATGLOBAL_PERIOD*HZ);

}
//ASUS BSP --- Eason Chang add BAT global variable

static void (*notify_vbus_state_func_ptr)(int);
static int usb_chg_current;

int pm8921_charger_register_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	notify_vbus_state_func_ptr = callback;
	return 0;
}
EXPORT_SYMBOL_GPL(pm8921_charger_register_vbus_sn);

/* this is passed to the hsusb via platform_data msm_otg_pdata */
void pm8921_charger_unregister_vbus_sn(void (*callback)(int))
{
	pr_debug("%p\n", callback);
	notify_vbus_state_func_ptr = NULL;
}
EXPORT_SYMBOL_GPL(pm8921_charger_unregister_vbus_sn);

static void notify_usb_of_the_plugin_event(int plugin)
{
	plugin = !!plugin;
	if (notify_vbus_state_func_ptr) {
		pr_debug("notifying plugin\n");
		(*notify_vbus_state_func_ptr) (plugin);
	} else {
		pr_debug("unable to notify plugin\n");
	}
}

static void __pm8921_charger_vbus_draw(unsigned int mA)
{
	int i, rc;
	if (!the_chip) {
		pr_err("called before init\n");
		return;
	}

	if (usb_max_current && mA > usb_max_current) {
		pr_debug("restricting usb current to %d instead of %d\n",
					usb_max_current, mA);
		mA = usb_max_current;
	}

	if (mA <= 2) {
		usb_chg_current = 0;
		rc = pm_chg_iusbmax_set(the_chip, 0);
		if (rc) {
			pr_err("unable to set iusb to %d rc = %d\n", 0, rc);
		}
		rc = pm_chg_usb_suspend_enable(the_chip, 1);
		if (rc)
			pr_err("fail to set suspend bit rc=%d\n", rc);
	} else {
		rc = pm_chg_usb_suspend_enable(the_chip, 0);
		if (rc)
			pr_err("fail to reset suspend bit rc=%d\n", rc);
		for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
			if (usb_ma_table[i].usb_ma <= mA)
				break;
		}

		if (i < 0) {
			pr_err("can't find %dmA in usb_ma_table. Use min.\n",
			       mA);
			i = 0;
		}

		/* Check if IUSB_FINE_RES is available */
		while ((usb_ma_table[i].value & PM8917_IUSB_FINE_RES)
				&& !the_chip->iusb_fine_res)
			i--;
		if (i < 0)
			i = 0;
		rc = pm_chg_iusbmax_set(the_chip, i);
		if (rc)
			pr_err("unable to set iusb to %d rc = %d\n", i, rc);
	}
}

/* USB calls these to tell us how much max usb current the system can draw */
void pm8921_charger_vbus_draw(unsigned int mA)
{
	int set_usb_now_ma;

///ASUS BSP Eason_Chang +++ print charger current 
    printk("[BAT]:vbus_draw:%d\n",mA);
///ASUS BSP Eason_Chang --- print charger current 
	pr_debug("Enter charge=%d\n", mA);

	/*
	 * Reject VBUS requests if USB connection is the only available
	 * power source. This makes sure that if booting without
	 * battery the iusb_max value is not decreased avoiding potential
	 * brown_outs.
	 *
	 * This would also apply when the battery has been
	 * removed from the running system.
	 */
	if (the_chip && !get_prop_batt_present(the_chip)
		&& !is_dc_chg_plugged_in(the_chip)) {
		if (!the_chip->has_dc_supply) {
			pr_err("rejected: no other power source connected\n");
			return;
		}
	}

	if (usb_max_current && mA > usb_max_current) {
		pr_warn("restricting usb current to %d instead of %d\n",
					usb_max_current, mA);
		mA = usb_max_current;
	}
	if (usb_target_ma == 0 && mA > USB_WALL_THRESHOLD_MA)
		usb_target_ma = mA;

	if (usb_target_ma)
		usb_target_ma = mA;


	if (mA > USB_WALL_THRESHOLD_MA)
		set_usb_now_ma = USB_WALL_THRESHOLD_MA;
	else
		set_usb_now_ma = mA;

	if (the_chip && the_chip->disable_aicl)
		set_usb_now_ma = mA;

	if (the_chip)
		__pm8921_charger_vbus_draw(set_usb_now_ma);
	else
		/*
		 * called before pmic initialized,
		 * save this value and use it at probe
		 */
		usb_chg_current = set_usb_now_ma;
}
EXPORT_SYMBOL_GPL(pm8921_charger_vbus_draw);

//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
int pm8921_is_chg_plugged_in(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_chg_plugged_in(the_chip);
}
EXPORT_SYMBOL(pm8921_is_chg_plugged_in);
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"

int pm8921_is_chg_auto_enable(void)
{
	u8 temp = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	pm8xxx_readb(the_chip->dev->parent, CHG_CNTRL_3, &temp);
	return  temp & CHG_EN_BIT;
}
EXPORT_SYMBOL(pm8921_is_chg_auto_enable);

int pm8921_is_usb_chg_plugged_in(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_usb_chg_plugged_in(the_chip);
}
EXPORT_SYMBOL(pm8921_is_usb_chg_plugged_in);

int pm8921_is_dc_chg_plugged_in(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_dc_chg_plugged_in(the_chip);
}
EXPORT_SYMBOL(pm8921_is_dc_chg_plugged_in);

int pm8921_is_battery_present(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return get_prop_batt_present(the_chip);
}
EXPORT_SYMBOL(pm8921_is_battery_present);

int pm8921_is_batfet_closed(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return is_batfet_closed(the_chip);
}
EXPORT_SYMBOL(pm8921_is_batfet_closed);
/*
 * Disabling the charge current limit causes current
 * current limits to have no monitoring. An adequate charger
 * capable of supplying high current while sustaining VIN_MIN
 * is required if the limiting is disabled.
 */
int pm8921_disable_input_current_limit(bool disable)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	if (disable) {
		pr_warn("Disabling input current limit!\n");

		return pm_chg_write(the_chip, CHG_BUCK_CTRL_TEST3, 0xF2);
	}
	return 0;
}
EXPORT_SYMBOL(pm8921_disable_input_current_limit);

int pm8917_set_under_voltage_detection_threshold(int mv)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return pm_chg_uvd_threshold_set(the_chip, mv);
}
EXPORT_SYMBOL(pm8917_set_under_voltage_detection_threshold);

int pm8921_set_max_battery_charge_current(int ma)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return pm_chg_ibatmax_set(the_chip, ma);
}
EXPORT_SYMBOL(pm8921_set_max_battery_charge_current);

int pm8921_disable_source_current(bool disable)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	if (disable)
		pr_warn("current drawn from chg=0, battery provides current\n");

	pm_chg_usb_suspend_enable(the_chip, disable);

	return pm_chg_charge_dis(the_chip, disable);
}
EXPORT_SYMBOL(pm8921_disable_source_current);

int pm8921_regulate_input_voltage(int voltage)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	rc = pm_chg_vinmin_set(the_chip, voltage);

	if (rc == 0)
		the_chip->vin_min = voltage;

	return rc;
}

#define USB_OV_THRESHOLD_MASK  0x60
#define USB_OV_THRESHOLD_SHIFT  5
int pm8921_usb_ovp_set_threshold(enum pm8921_usb_ov_threshold ov)
{
	u8 temp;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (ov > PM_USB_OV_7V) {
		pr_err("limiting to over voltage threshold to 7volts\n");
		ov = PM_USB_OV_7V;
	}

	temp = USB_OV_THRESHOLD_MASK & (ov << USB_OV_THRESHOLD_SHIFT);

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_OV_THRESHOLD_MASK, temp);
}
EXPORT_SYMBOL(pm8921_usb_ovp_set_threshold);

#define USB_DEBOUNCE_TIME_MASK	0x06
#define USB_DEBOUNCE_TIME_SHIFT 1
int pm8921_usb_ovp_set_hystersis(enum pm8921_usb_debounce_time ms)
{
	u8 temp;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (ms > PM_USB_DEBOUNCE_80P5MS) {
		pr_err("limiting debounce to 80.5ms\n");
		ms = PM_USB_DEBOUNCE_80P5MS;
	}

	temp = USB_DEBOUNCE_TIME_MASK & (ms << USB_DEBOUNCE_TIME_SHIFT);

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_DEBOUNCE_TIME_MASK, temp);
}
EXPORT_SYMBOL(pm8921_usb_ovp_set_hystersis);

#define USB_OVP_DISABLE_MASK	0x80
int pm8921_usb_ovp_disable(int disable)
{
	u8 temp = 0;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (disable)
		temp = USB_OVP_DISABLE_MASK;

	return pm_chg_masked_write(the_chip, USB_OVP_CONTROL,
				USB_OVP_DISABLE_MASK, temp);
}

bool pm8921_is_battery_charging(int *source)
{
	int fsm_state, is_charging, dc_present, usb_present;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	fsm_state = pm_chg_get_fsm_state(the_chip);
	is_charging = is_battery_charging(fsm_state);
	if (is_charging == 0) {
		*source = PM8921_CHG_SRC_NONE;
		return is_charging;
	}

	if (source == NULL)
		return is_charging;

	/* the battery is charging, the source is requested, find it */
	dc_present = is_dc_chg_plugged_in(the_chip);
	usb_present = is_usb_chg_plugged_in(the_chip);

	if (dc_present && !usb_present)
		*source = PM8921_CHG_SRC_DC;

	if (usb_present && !dc_present)
		*source = PM8921_CHG_SRC_USB;

	if (usb_present && dc_present)
		/*
		 * The system always chooses dc for charging since it has
		 * higher priority.
		 */
		*source = PM8921_CHG_SRC_DC;

	return is_charging;
}
EXPORT_SYMBOL(pm8921_is_battery_charging);

int pm8921_set_usb_power_supply_type(enum power_supply_type type)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (type < POWER_SUPPLY_TYPE_USB && type > POWER_SUPPLY_TYPE_BATTERY)
		return -EINVAL;

	the_chip->usb_psy.type = type;
	power_supply_changed(&the_chip->usb_psy);
	power_supply_changed(&the_chip->dc_psy);
	return 0;
}
EXPORT_SYMBOL_GPL(pm8921_set_usb_power_supply_type);

int pm8921_batt_temperature(void)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return get_prop_batt_temp(the_chip);
}

static int __pm8921_apply_19p2mhz_kickstart(struct pm8921_chg_chip *chip)
{
	int err;
	u8 temp;


	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	temp  = 0xD3;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	temp  = 0xD5;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	/* Wait a few clock cycles before re-enabling hw clock switching */
	udelay(183);

	temp  = 0xD1;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	temp  = 0xD0;
	err = pm8xxx_writeb(chip->dev->parent, CHG_TEST, temp);
	if (err) {
		pr_err("Error %d writing %d to addr %d\n", err, temp, CHG_TEST);
		return err;
	}

	/* Wait for few clock cycles before re-enabling LPM */
	udelay(32);

	return 0;
}

static int pm8921_apply_19p2mhz_kickstart(struct pm8921_chg_chip *chip)
{
	int err;
	unsigned long flags = 0;

	spin_lock_irqsave(&lpm_lock, flags);
	err = pm8921_chg_set_lpm(chip, 0);
	if (err) {
		pr_err("Error settig LPM rc=%d\n", err);
		goto kick_err;
	}

	__pm8921_apply_19p2mhz_kickstart(chip);

kick_err:
	err = pm8921_chg_set_lpm(chip, 1);
	if (err)
		pr_err("Error settig LPM rc=%d\n", err);

	spin_unlock_irqrestore(&lpm_lock, flags);

	return err;
}

//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
static void handle_chg_insertion_removal(struct pm8921_chg_chip *chip)
{
	int chg_present;
    printk("[BAT]chg_I/O+++\n");
	chg_present = is_chg_plugged_in(chip);
	printk("[BAT]chg_I/O chg_present:%d\n",chg_present);
	printk("[BAT]chg_I/O---\n");
//+++ ASUS_BSP Allen1_Huang: enable ICS charger mode
#ifdef CONFIG_CHARGER_MODE
	g_chg_present = chg_present;
#endif
//--- ASUS_BSP Allen1_Huang: enable ICS charger mode
	if (chip->chg_present ^ chg_present) {
		notify_usb_of_the_plugin_event(chg_present)	;
		chip->chg_present = chg_present;
	}
     //ASUS_BSP eason QCPatch:Workaround for USB unplug issue+++
	//if (usb_present) {
     if(g_A60K_hwID <A66_HW_ID_ER2){    

	if (chg_present) {
		schedule_delayed_work(&chip->unplug_check_work,
			round_jiffies_relative(msecs_to_jiffies
				(UNPLUG_CHECK_WAIT_PERIOD_MS)));
	}
        }
     //ASUS_BSP eason QCPatch:Workaround for USB unplug issue---

	bms_notify_check(chip);
}
#else
static void handle_usb_insertion_removal(struct pm8921_chg_chip *chip)
{
	int usb_present, rc = 0;

	if (chip->lockup_lpm_wrkarnd) {
		rc = pm8921_apply_19p2mhz_kickstart(chip);
		if (rc)
			pr_err("Failed to apply kickstart rc=%d\n", rc);
	}

	pm_chg_failed_clear(chip, 1);
	usb_present = is_usb_chg_plugged_in(chip);
	if (chip->usb_present ^ usb_present) {
		notify_usb_of_the_plugin_event(usb_present);
		chip->usb_present = usb_present;
		power_supply_changed(&chip->usb_psy);
		power_supply_changed(&chip->batt_psy);
		pm8921_bms_calibrate_hkadc();

		/* Enable/disable bypass if charger is on battery */
		if (chip->lockup_lpm_wrkarnd)
			pm8921_chg_bypass_bat_gone_debounce(chip,
				is_chg_on_bat(chip));
	}
	if (usb_present) {
		schedule_delayed_work(&chip->unplug_check_work,
			msecs_to_jiffies(UNPLUG_CHECK_RAMP_MS));
		pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);
	} else {
		/* USB unplugged reset target current */
		usb_target_ma = 0;
		pm8921_chg_disable_irq(chip, CHG_GONE_IRQ);
	}
	bms_notify_check(chip);
}
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"

static void handle_stop_ext_chg(struct pm8921_chg_chip *chip)
{
	if (chip->lockup_lpm_wrkarnd)
		/* Enable bypass if charger is on battery */
		pm8921_chg_bypass_bat_gone_debounce(chip, is_chg_on_bat(chip));

	if (!chip->ext_psy) {
		pr_debug("external charger not registered.\n");
		return;
	}

	if (!chip->ext_charging) {
		pr_debug("already not charging.\n");
		return;
	}

	power_supply_set_charge_type(chip->ext_psy,
					POWER_SUPPLY_CHARGE_TYPE_NONE);
	pm8921_disable_source_current(false); /* release BATFET */
	power_supply_changed(&chip->dc_psy);
	chip->ext_charging = false;
	chip->ext_charge_done = false;
	bms_notify_check(chip);
	/* Update battery charging LEDs and user space battery info */
	power_supply_changed(&chip->batt_psy);
}

static void handle_start_ext_chg(struct pm8921_chg_chip *chip)
{
	int dc_present;
	int batt_present;
	int batt_temp_ok;
	unsigned long delay =
		round_jiffies_relative(msecs_to_jiffies(EOC_CHECK_PERIOD_MS));

	/* Disable bypass if charger connected and not running on bat */
	if (chip->lockup_lpm_wrkarnd)
		pm8921_chg_bypass_bat_gone_debounce(chip, is_chg_on_bat(chip));

	if (!chip->ext_psy) {
		pr_debug("external charger not registered.\n");
		return;
	}

	if (chip->ext_charging) {
		pr_debug("already charging.\n");
		return;
	}

	dc_present = is_dc_chg_plugged_in(chip);
	batt_present = pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
	batt_temp_ok = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);

	if (!dc_present) {
		pr_warn("%s. dc not present.\n", __func__);
		return;
	}
	if (!batt_present) {
		pr_warn("%s. battery not present.\n", __func__);
		return;
	}
	if (!batt_temp_ok) {
		pr_warn("%s. battery temperature not ok.\n", __func__);
		return;
	}

	/* Force BATFET=ON */
	pm8921_disable_source_current(true);

	schedule_delayed_work(&chip->unplug_check_work,
			msecs_to_jiffies(UNPLUG_CHECK_RAMP_MS));

	power_supply_set_online(chip->ext_psy, dc_present);
	power_supply_set_charge_type(chip->ext_psy,
					POWER_SUPPLY_CHARGE_TYPE_FAST);
	chip->ext_charging = true;
	chip->ext_charge_done = false;
	bms_notify_check(chip);
	/*
	 * since we wont get a fastchg irq from external charger
	 * use eoc worker to detect end of charging
	 */
	schedule_delayed_work(&chip->eoc_work, delay);

	//ASUS_BSP --- Josh_Liao "remove eoc wakelock to enable suspend when ac charging"
	//wake_lock(&chip->eoc_wake_lock);
	//ASUS_BSP --- Josh_Liao "remove eoc wakelock to enable suspend when ac charging"
	if (chip->btc_override)
		schedule_delayed_work(&chip->btc_override_work,
				round_jiffies_relative(msecs_to_jiffies
					(chip->btc_delay_ms)));
	/* Update battery charging LEDs and user space battery info */
	power_supply_changed(&chip->batt_psy);
}

//ASUS BSP Eason_Chang 1030 porting+++
static void handle_dc_removal_insertion(struct pm8921_chg_chip *chip)
{
	int dc_present;

	dc_present = is_dc_chg_plugged_in(chip);
	if (chip->dc_present ^ dc_present) {
		chip->dc_present = dc_present;
		power_supply_changed(&chip->dc_psy);
		power_supply_changed(&chip->batt_psy);
	}
	bms_notify_check(chip);
}
//ASUS BSP Eason_Chang 1030 porting---

static void turn_off_ovp_fet(struct pm8921_chg_chip *chip, u16 ovptestreg)
{
	u8 temp;
	int rc;

	rc = pm_chg_write(chip, ovptestreg, 0x30);
	if (rc) {
		pr_err("Failed to write 0x30 to ovptestreg rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, ovptestreg, &temp);
	if (rc) {
		pr_err("Failed to read from ovptestreg rc = %d\n", rc);
		return;
	}
	/* set ovp fet disable bit and the write bit */
	temp |= 0x81;
	rc = pm_chg_write(chip, ovptestreg, temp);
	if (rc) {
		pr_err("Failed to write 0x%x ovptestreg rc=%d\n", temp, rc);
		return;
	}
}

static void turn_on_ovp_fet(struct pm8921_chg_chip *chip, u16 ovptestreg)
{
	u8 temp;
	int rc;

	rc = pm_chg_write(chip, ovptestreg, 0x30);
	if (rc) {
		pr_err("Failed to write 0x30 to OVP_TEST rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, ovptestreg, &temp);
	if (rc) {
		pr_err("Failed to read from OVP_TEST rc = %d\n", rc);
		return;
	}
	/* unset ovp fet disable bit and set the write bit */
	temp &= 0xFE;
	temp |= 0x80;
	rc = pm_chg_write(chip, ovptestreg, temp);
	if (rc) {
		pr_err("Failed to write 0x%x to OVP_TEST rc = %d\n",
								temp, rc);
		return;
	}
}

static int param_open_ovp_counter = 10;
module_param(param_open_ovp_counter, int, 0644);

#define USB_ACTIVE_BIT BIT(5)
#define DC_ACTIVE_BIT BIT(6)
static int is_active_chg_plugged_in(struct pm8921_chg_chip *chip,
						u8 active_chg_mask)
{
	if (active_chg_mask & USB_ACTIVE_BIT)
		return pm_chg_get_rt_status(chip, USBIN_VALID_IRQ);
	else if (active_chg_mask & DC_ACTIVE_BIT)
		return pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);
	else
		return 0;
}

#define WRITE_BANK_4		0xC0
#define OVP_DEBOUNCE_TIME 0x06

//ASUS_BSP eason QCPatch:Workaround for USB unplug issue+++
static void unplug_wrkarnd_restore_worker(struct work_struct *work)
{
	u8 temp;
	int rc;
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip,
				unplug_wrkarnd_restore_work);

	pr_debug("restoring vin_min to %d mV\n", chip->vin_min);
	rc = pm_chg_vinmin_set(the_chip, chip->vin_min);

	temp = WRITE_BANK_4 | 0xA;
	rc = pm8xxx_writeb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, temp);
	if (rc) {
		pr_err("Error %d writing %d to addr %d\n", rc,
					temp, CHG_BUCK_CTRL_TEST3);
	}
	wake_unlock(&chip->unplug_wrkarnd_restore_wake_lock);
}
//ASUS_BSP eason QCPatch:Workaround for USB unplug issue---

static void unplug_ovp_fet_open(struct pm8921_chg_chip *chip)
{
	int chg_gone = 0, active_chg_plugged_in = 0;
	int count = 0;
	u8 active_mask = 0;
	u16 ovpreg, ovptestreg;

	if (is_usb_chg_plugged_in(chip) &&
		(chip->active_path & USB_ACTIVE_BIT)) {
		ovpreg = USB_OVP_CONTROL;
		ovptestreg = USB_OVP_TEST;
		active_mask = USB_ACTIVE_BIT;
	} else if (is_dc_chg_plugged_in(chip) &&
		(chip->active_path & DC_ACTIVE_BIT)) {
		ovpreg = DC_OVP_CONTROL;
		ovptestreg = DC_OVP_TEST;
		active_mask = DC_ACTIVE_BIT;
	} else {
		return;
	}

	while (count++ < param_open_ovp_counter) {
		pm_chg_masked_write(chip, ovpreg, OVP_DEBOUNCE_TIME, 0x0);
		usleep(10);
		active_chg_plugged_in
			= is_active_chg_plugged_in(chip, active_mask);
		chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);
		pr_debug("OVP FET count = %d chg_gone=%d, active_valid = %d\n",
					count, chg_gone, active_chg_plugged_in);

		/* note usb_chg_plugged_in=0 => chg_gone=1 */
		if (chg_gone == 1 && active_chg_plugged_in == 1) {
			pr_debug("since chg_gone = 1 dis ovp_fet for 20msec\n");
			turn_off_ovp_fet(chip, ovptestreg);

			msleep(20);

			turn_on_ovp_fet(chip, ovptestreg);
		} else {
			break;
		}
	}
	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8917)
		pm_chg_masked_write(chip, ovpreg, OVP_DEBOUNCE_TIME, 0x6);
	else
		pm_chg_masked_write(chip, ovpreg, OVP_DEBOUNCE_TIME, 0x2);

	pr_debug("Exit count=%d chg_gone=%d, active_valid=%d\n",
		count, chg_gone, active_chg_plugged_in);
	return;
}

static int find_usb_ma_value(int value)
{
	int i;

	for (i = ARRAY_SIZE(usb_ma_table) - 1; i >= 0; i--) {
		if (value >= usb_ma_table[i].usb_ma)
			break;
	}

	return i;
}

static void decrease_usb_ma_value(int *value)
{
	int i;

	if (value) {
		i = find_usb_ma_value(*value);
		if (i > 0)
			i--;
		while (!the_chip->iusb_fine_res && i > 0
			&& (usb_ma_table[i].value & PM8917_IUSB_FINE_RES))
			i--;

		if (i < 0) {
			pr_err("can't find %dmA in usb_ma_table. Use min.\n",
			       *value);
			i = 0;
		}

		*value = usb_ma_table[i].usb_ma;
	}
}

static void increase_usb_ma_value(int *value)
{
	int i;

	if (value) {
		i = find_usb_ma_value(*value);

		if (i < (ARRAY_SIZE(usb_ma_table) - 1))
			i++;
		/* Get next correct entry if IUSB_FINE_RES is not available */
		while (!the_chip->iusb_fine_res
			&& (usb_ma_table[i].value & PM8917_IUSB_FINE_RES)
			&& i < (ARRAY_SIZE(usb_ma_table) - 1))
			i++;

		*value = usb_ma_table[i].usb_ma;
	}
}

static void vin_collapse_check_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
			struct pm8921_chg_chip, vin_collapse_check_work);

	/*
	 * AICL only for wall-chargers. If the charger appears to be plugged
	 * back in now, the corresponding unplug must have been because of we
	 * were trying to draw more current than the charger can support. In
	 * such a case reset usb current to 500mA and decrease the target.
	 * The AICL algorithm will step up the current from 500mA to target
	 */
	if (is_usb_chg_plugged_in(chip)
		&& usb_target_ma > USB_WALL_THRESHOLD_MA
		&& !chip->disable_aicl) {
		/* decrease usb_target_ma */
		decrease_usb_ma_value(&usb_target_ma);
		/* reset here, increase in unplug_check_worker */
		__pm8921_charger_vbus_draw(USB_WALL_THRESHOLD_MA);
		pr_debug("usb_now=%d, usb_target = %d\n",
				USB_WALL_THRESHOLD_MA, usb_target_ma);
		if (!delayed_work_pending(&chip->unplug_check_work))
			schedule_delayed_work(&chip->unplug_check_work,
				      msecs_to_jiffies
						(UNPLUG_CHECK_WAIT_PERIOD_MS));
	} else {
//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
		handle_chg_insertion_removal(chip);
#else
		handle_usb_insertion_removal(chip);
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"
	}
}

#define VIN_MIN_COLLAPSE_CHECK_MS	50
static irqreturn_t usbin_valid_irq_handler(int irq, void *data)
{
	if (usb_target_ma)
		schedule_delayed_work(&the_chip->vin_collapse_check_work,
				      round_jiffies_relative(msecs_to_jiffies
						(VIN_MIN_COLLAPSE_CHECK_MS)));
	else
//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
	handle_chg_insertion_removal(data);
#else
	    handle_usb_insertion_removal(data);
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"

	return IRQ_HANDLED;
}

static irqreturn_t batt_inserted_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int status;

	status = pm_chg_get_rt_status(chip, BATT_INSERTED_IRQ);
	schedule_work(&chip->battery_id_valid_work);
	handle_start_ext_chg(chip);
//ASUS BSP+++ notify BatteryService    
	//asus_onChargingStart();
    printk("[BAT][8921]%s():\n",__FUNCTION__);
//ASUS BSP--- notify BatteryService 
	pr_debug("battery present=%d", status);
	//power_supply_changed(&chip->batt_psy);//ASUS BSP+++ notify BatteryService 
	return IRQ_HANDLED;
}

/*
 * this interrupt used to restart charging a battery.
 *
 * Note: When DC-inserted the VBAT can't go low.
 * VPH_PWR is provided by the ext-charger.
 * After End-Of-Charging from DC, charging can be resumed only
 * if DC is removed and then inserted after the battery was in use.
 * Therefore the handle_start_ext_chg() is not called.
 */
static irqreturn_t vbatdet_low_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int high_transition;

	high_transition = pm_chg_get_rt_status(chip, VBATDET_LOW_IRQ);

	if (high_transition) {
		/* enable auto charging */
		pm_chg_auto_enable(chip, !charging_disabled);
		pr_info("batt fell below resume voltage %s\n",
			charging_disabled ? "" : "charger enabled");
	}
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));

//ASUS_BSP +++ Josh_Liao "add asus battery driver"
if (asus_bat_has_done_first_periodic_update()) {
//ASUS_BSP --- Josh_Liao "add asus battery driver"

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);//ASUS BSP Eason_Chang 1030 porting
//ASUS_BSP +++ Josh_Liao "add asus battery driver"
}
//ASUS_BSP --- Josh_Liao "add asus battery driver"
	return IRQ_HANDLED;
}

static irqreturn_t chgwdog_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vcp_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t atcdone_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t atcfail_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t chgdone_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("state_changed_to=%d\n", pm_chg_get_fsm_state(data));

	handle_stop_ext_chg(chip);
//ASUS BSP+++ notify BatteryService    
        chip->fastchg_done = true;
	//asus_onChargingStop(exCHGDOWN);
      //printk("[ASUS][BAT]%s():\n",__FUNCTION__);
	//power_supply_changed(&chip->batt_psy); 
//ASUS BSP--- notify BatteryService   	
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);//ASUS BSP Eason_Chang 1030 porting

	bms_notify_check(chip);

///+++ASUS_BSP+++ Eason_Chang  ASUS SW gauge
        if(g_A60K_hwID <A66_HW_ID_ER2){    

		asus_set_charge_done();
        }
///+++ASUS_BSP--- Eason_Chang  ASUS SW gauge
	return IRQ_HANDLED;
}

static irqreturn_t chgfail_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int ret;

	if (!chip->stop_chg_upon_expiry) {
		ret = pm_chg_failed_clear(chip, 1);
		if (ret)
			pr_err("Failed to write CHG_FAILED_CLEAR bit\n");
	}

	pr_err("batt_present = %d, batt_temp_ok = %d, state_changed_to=%d\n",
			get_prop_batt_present(chip),
			pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ),
			pm_chg_get_fsm_state(data));

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);
	return IRQ_HANDLED;
}

static irqreturn_t chgstate_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("state_changed_to=%d\n", pm_chg_get_fsm_state(data));
//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
    if(g_A60K_hwID <A66_HW_ID_ER2){    

	if (delayed_work_pending(&pm_ps_changed_work)) {
		printk(DBGMSK_BAT_INFO "[BAT] re-queue pm_ps_changed_work \r\n");
		cancel_delayed_work_sync(&pm_ps_changed_work);
		schedule_delayed_work(&pm_ps_changed_work, 2*HZ);
	} else {
		schedule_delayed_work(&pm_ps_changed_work, 2*HZ);
	}
        }
#else
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);//ASUS BSP Eason_Chang 1030 porting
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"

	bms_notify_check(chip);

	return IRQ_HANDLED;
}

enum {
	PON_TIME_25NS	= 0x04,
	PON_TIME_50NS	= 0x08,
	PON_TIME_100NS	= 0x0C,
};

static void set_min_pon_time(struct pm8921_chg_chip *chip, int pon_time_ns)
{
	u8 temp;
	int rc;

	rc = pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, 0x40);
	if (rc) {
		pr_err("Failed to write 0x70 to CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, CHG_BUCK_CTRL_TEST3, &temp);
	if (rc) {
		pr_err("Failed to read CTRL_TEST3 rc = %d\n", rc);
		return;
	}
	/* clear the min pon time select bit */
	temp &= 0xF3;
	/* set the pon time */
	temp |= (u8)pon_time_ns;
	/* write enable bank 4 */
	temp |= 0x80;
	rc = pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, temp);
	if (rc) {
		pr_err("Failed to write 0x%x to CTRL_TEST3 rc=%d\n", temp, rc);
		return;
	}
}

static void attempt_reverse_boost_fix(struct pm8921_chg_chip *chip)
{
	pr_debug("Start\n");
	set_min_pon_time(chip, PON_TIME_100NS);
	pm_chg_vinmin_set(chip, chip->vin_min + 200);
	msleep(250);
	pm_chg_vinmin_set(chip, chip->vin_min);
	set_min_pon_time(chip, PON_TIME_25NS);
	pr_debug("End\n");
}


 //ASUS_BSP eason QCPatch:Workaround for USB unplug issue+++
#define VIN_ACTIVE_BIT BIT(0)
#define UNPLUG_WRKARND_RESTORE_WAIT_PERIOD_US	200
#define VIN_MIN_INCREASE_MV	100
static void unplug_check_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, unplug_check_work);
	u8 reg_loop = 0, active_path;
	int rc, ibat, active_chg_plugged_in, usb_ma;
	int chg_gone = 0;
	bool ramp = false;

	rc = pm8xxx_readb(chip->dev->parent, PBL_ACCESS1, &active_path);
	if (rc) {
		pr_err("Failed to read PBL_ACCESS1 rc=%d\n", rc);
		return;
	}

	chip->active_path = active_path;
	active_chg_plugged_in = is_active_chg_plugged_in(chip, active_path);
	pr_debug("active_path = 0x%x, active_chg_plugged_in = %d\n",
			active_path, active_chg_plugged_in);
	if (active_path & USB_ACTIVE_BIT) {
		pr_debug("USB charger active\n");

		pm_chg_iusbmax_get(chip, &usb_ma);

		if (usb_ma <= 100) {
			pr_debug(
				"Unenumerated or suspended usb_ma = %d skip\n",
				usb_ma);
			goto check_again_later;
		}
	} else if (active_path & DC_ACTIVE_BIT) {
		pr_debug("DC charger active\n");
	} else {
		/* No charger active */
		if (!(is_usb_chg_plugged_in(chip)
				&& !(is_dc_chg_plugged_in(chip)))) {
			pr_debug(
			"Stop: chg removed reg_loop = %d, fsm = %d ibat = %d\n",
				pm_chg_get_regulation_loop(chip),
				pm_chg_get_fsm_state(chip),
				get_prop_batt_current(chip)
				);
			if (chip->lockup_lpm_wrkarnd) {
				rc = pm8921_apply_19p2mhz_kickstart(chip);
				if (rc)
					pr_err("Failed kickstart rc=%d\n", rc);

				/*
				 * Make sure kickstart happens at least 200 ms
				 * after charger has been removed.
				 */
				if (chip->final_kickstart) {
					chip->final_kickstart = false;
					goto check_again_later;
				}
			}
			return;
		} else {
			goto check_again_later;
		}
	}

	chip->final_kickstart = true;

	/* AICL only for usb wall charger */
	if ((active_path & USB_ACTIVE_BIT) && usb_target_ma > 0 &&
		!chip->disable_aicl) {
		reg_loop = pm_chg_get_regulation_loop(chip);
		pr_debug("reg_loop=0x%x usb_ma = %d\n", reg_loop, usb_ma);
		if ((reg_loop & VIN_ACTIVE_BIT) &&
			(usb_ma > USB_WALL_THRESHOLD_MA)
			&& !charging_disabled) {
			decrease_usb_ma_value(&usb_ma);
			usb_target_ma = usb_ma;
			/* end AICL here */
			__pm8921_charger_vbus_draw(usb_ma);
			pr_debug("usb_now=%d, usb_target = %d\n",
				usb_ma, usb_target_ma);
		}
	}

	reg_loop = pm_chg_get_regulation_loop(chip);
	pr_debug("reg_loop=0x%x usb_ma = %d\n", reg_loop, usb_ma);

	ibat = get_prop_batt_current(chip);
	if (reg_loop & VIN_ACTIVE_BIT) {
		if (ibat > 0) {
			pr_debug("revboost ibat = %d fsm = %d loop = 0x%x\n",
				ibat, pm_chg_get_fsm_state(chip), reg_loop);
			attempt_reverse_boost_fix(chip);
			/* after reverse boost fix check if the active
			 * charger was detected as removed */
			active_chg_plugged_in
				= is_active_chg_plugged_in(chip,
					active_path);
			pr_debug("revboost post: active_chg_plugged_in = %d\n",
					active_chg_plugged_in);
		}
	}

	active_chg_plugged_in = is_active_chg_plugged_in(chip, active_path);
	pr_debug("active_path = 0x%x, active_chg = %d\n",
			active_path, active_chg_plugged_in);
	chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);

	if (chg_gone == 1  && active_chg_plugged_in == 1) {
		pr_debug("chg_gone=%d, active_chg_plugged_in = %d\n",
					chg_gone, active_chg_plugged_in);
		unplug_ovp_fet_open(chip);
	}

	/* AICL only for usb wall charger */
	if (!(reg_loop & VIN_ACTIVE_BIT) && (active_path & USB_ACTIVE_BIT)
		&& usb_target_ma > 0
		&& !charging_disabled
		&& !chip->disable_aicl) {
		/* only increase iusb_max if vin loop not active */
		if (usb_ma < usb_target_ma) {
			increase_usb_ma_value(&usb_ma);
			if (usb_ma > usb_target_ma)
				usb_ma = usb_target_ma;
			__pm8921_charger_vbus_draw(usb_ma);
			pr_debug("usb_now=%d, usb_target = %d\n",
					usb_ma, usb_target_ma);
			ramp = true;
		} else {
			usb_target_ma = usb_ma;
		}
	}
check_again_later:
	pr_debug("ramp: %d\n", ramp);
	/* schedule to check again later */
	if (ramp)
		schedule_delayed_work(&chip->unplug_check_work,
			msecs_to_jiffies(UNPLUG_CHECK_RAMP_MS));
	else
		schedule_delayed_work(&chip->unplug_check_work,
			msecs_to_jiffies(UNPLUG_CHECK_WAIT_PERIOD_MS));
}
 //ASUS_BSP eason QCPatch:Workaround for USB unplug issue---

static irqreturn_t loop_change_irq_handler(int irq, void *data)
{
//ASUS_BSP eason QCPatch:Workaround for USB unplug issue+++
	struct pm8921_chg_chip *chip = data;

	pr_debug("fsm_state=%d reg_loop=0x%x\n",
		pm_chg_get_fsm_state(data),
		pm_chg_get_regulation_loop(data));
	//unplug_check_worker(&(chip->unplug_check_work.work));
	schedule_delayed_work(&chip->unplug_check_work,0); //carol+
 //ASUS_BSP eason QCPatch:Workaround for USB unplug issue---
	return IRQ_HANDLED;
}

struct ibatmax_max_adj_entry {
	int ibat_max_ma;
	int max_adj_ma;
};

static struct ibatmax_max_adj_entry ibatmax_adj_table[] = {
	{975, 300},
	{1475, 150},
	{1975, 200},
	{2475, 250},
};

static int find_ibat_max_adj_ma(int ibat_target_ma)
{
	int i = 0;

	for (i = ARRAY_SIZE(ibatmax_adj_table); i > 0; i--) {
		if (ibat_target_ma >= ibatmax_adj_table[i - 1].ibat_max_ma)
			break;
	}

	return ibatmax_adj_table[i].max_adj_ma;
}

static irqreturn_t fastchg_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int high_transition;

	high_transition = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
	if (high_transition && !delayed_work_pending(&chip->eoc_work)) {
//ASUS_BSP +++ Josh_Liao "remove eoc wakelock to enable suspend when ac charging"
//	wake_lock(&chip->eoc_wake_lock);
//ASUS_BSP --- Josh_Liao "remove eoc wakelock to enable suspend when ac charging"
		schedule_delayed_work(&chip->eoc_work,
				      round_jiffies_relative(msecs_to_jiffies
						     (EOC_CHECK_PERIOD_MS)));
	}
	if (high_transition
		&& chip->btc_override
		&& !delayed_work_pending(&chip->btc_override_work)) {
		schedule_delayed_work(&chip->btc_override_work,
					round_jiffies_relative(msecs_to_jiffies
						(chip->btc_delay_ms)));
	}
	power_supply_changed(&chip->batt_psy);
	bms_notify_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t trklchg_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t batt_removed_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int status;

	status = pm_chg_get_rt_status(chip, BATT_REMOVED_IRQ);
	pr_debug("battery present=%d state=%d", !status,
					 pm_chg_get_fsm_state(data));
	handle_stop_ext_chg(chip);
//ASUS BSP+++ notify BatteryService    
	//asus_onChargingStop(exBATT_REMOVED_ERROR);
    //printk("[ASUS][BAT]%s():\n",__FUNCTION__);
	//power_supply_changed(&chip->batt_psy);
//ASUS BSP--- notify BatteryService 	
	return IRQ_HANDLED;
}

static irqreturn_t batttemp_hot_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	handle_stop_ext_chg(chip);
//ASUS BSP+++ notify BatteryService        
	//asus_onChargingStop(exBATTEMP_HOT_ERROR);//Eason for ASUS BatteryService
    //printk("[ASUS][BAT]%s():\n",__FUNCTION__);
	//power_supply_changed(&chip->batt_psy);//Eason for ASUS BatteryService
//ASUS BSP--- notify BatteryService    	
	return IRQ_HANDLED;
}

static irqreturn_t chghot_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
    //asus_onChargingStop(exOT_CHARGER_ERROR);

	pr_debug("Chg hot fsm_state=%d\n", pm_chg_get_fsm_state(data));
	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);//ASUS BSP Eason_Chang 1030 porting
	handle_stop_ext_chg(chip);
	return IRQ_HANDLED;
}

static irqreturn_t batttemp_cold_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("Batt cold fsm_state=%d\n", pm_chg_get_fsm_state(data));
	handle_stop_ext_chg(chip);
//ASUS BSP+++ notify BatteryService        
    //asus_onChargingStop(exBATTEMP_COLD_ERROR);
    //printk("[ASUS][BAT]%s():\n",__FUNCTION__);
	//power_supply_changed(&chip->batt_psy);
//ASUS BSP--- notify BatteryService    	
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);//ASUS BSP Eason_Chang 1030 porting
	return IRQ_HANDLED;
}

static irqreturn_t chg_gone_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int chg_gone, usb_chg_plugged_in;

	usb_chg_plugged_in = is_usb_chg_plugged_in(chip);
	chg_gone = pm_chg_get_rt_status(chip, CHG_GONE_IRQ);

	pr_debug("chg_gone=%d, usb_valid = %d\n", chg_gone, usb_chg_plugged_in);
	pr_debug("Chg gone fsm_state=%d\n", pm_chg_get_fsm_state(data));

	power_supply_changed(&chip->batt_psy);
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);//ASUS BSP Eason_Chang 1030 porting
	return IRQ_HANDLED;
}
/*
 *
 * bat_temp_ok_irq_handler - is edge triggered, hence it will
 * fire for two cases:
 *
 * If the interrupt line switches to high temperature is okay
 * and thus charging begins.
 * If bat_temp_ok is low this means the temperature is now
 * too hot or cold, so charging is stopped.
 *
 */
static irqreturn_t bat_temp_ok_irq_handler(int irq, void *data)
{
	int bat_temp_ok;
	struct pm8921_chg_chip *chip = data;

	bat_temp_ok = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);

	pr_debug("batt_temp_ok = %d fsm_state%d\n",
			 bat_temp_ok, pm_chg_get_fsm_state(data));

	if (bat_temp_ok){
		handle_start_ext_chg(chip);
        //ASUS BSP+++ notify BatteryService 
        if(g_A60K_hwID <A66_HW_ID_ER2){    

        asus_onChargingStart();
        }
        printk("[BAT][8921]%s():true\n",__FUNCTION__);
        //ASUS BSP--- notify BatteryService 
        }
	else{
		handle_stop_ext_chg(chip);
        //ASUS BSP+++ notify BatteryService    
		//asus_onChargingStop(exTEMP_OK_ERROR);
        //printk("[ASUS][BAT]%s():false\n",__FUNCTION__);
		//ASUS BSP--- notify BatteryService    
	}
	//power_supply_changed(&chip->batt_psy);//ASUS BSP notify BatteryService 
	power_supply_changed(&chip->usb_psy);
	power_supply_changed(&chip->dc_psy);//ASUS BSP Eason_Chang 1030 porting
	bms_notify_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t coarse_det_low_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vdd_loop_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vreg_ov_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t vbatdet_irq_handler(int irq, void *data)
{
	pr_debug("fsm_state=%d\n", pm_chg_get_fsm_state(data));
	return IRQ_HANDLED;
}

static irqreturn_t batfet_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	pr_debug("vreg ov\n");
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t dcin_valid_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;
	int dc_present;

   	handle_dc_removal_insertion(chip);//ASUS BSP Eason_Chang 1030 porting

	pm_chg_failed_clear(chip, 1);
	dc_present = pm_chg_get_rt_status(chip, DCIN_VALID_IRQ);

	if (chip->dc_present ^ dc_present)
		pm8921_bms_calibrate_hkadc();
   {
		handle_start_ext_chg(chip);
        //ASUS BSP+++ notify BatteryService    
        if(g_A60K_hwID <A66_HW_ID_ER2){    

		 asus_onChargingStart();
        }
    	 printk("[BAT][8921]%s():\n",__FUNCTION__); 
    	 //ASUS BSP--- notify BatteryService    
   }  
	if (dc_present)
		pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);
	else
		pm8921_chg_disable_irq(chip, CHG_GONE_IRQ);

	chip->dc_present = dc_present;

	if (chip->ext_psy) {
		if (dc_present)
			handle_start_ext_chg(chip);
		else
			handle_stop_ext_chg(chip);
	} else {
		if (chip->lockup_lpm_wrkarnd)
			/* if no external supply call bypass debounce here */
			pm8921_chg_bypass_bat_gone_debounce(chip,
				is_chg_on_bat(chip));

		if (dc_present)
			schedule_delayed_work(&chip->unplug_check_work,
				msecs_to_jiffies(UNPLUG_CHECK_WAIT_PERIOD_MS));
		power_supply_changed(&chip->dc_psy);
	}

	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}

static irqreturn_t dcin_ov_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

   	handle_dc_removal_insertion(chip);//ASUS BSP Eason_Chang 1030 porting
	handle_stop_ext_chg(chip);
	//ASUS BSP+++ notify BatteryService    
	//asus_onChargingStop(exDCIN_OV);
    //printk("[ASUS][BAT]%s():\n",__FUNCTION__);
    //ASUS BSP--- notify BatteryService   
	return IRQ_HANDLED;
}

static irqreturn_t dcin_uv_irq_handler(int irq, void *data)
{
	struct pm8921_chg_chip *chip = data;

	handle_stop_ext_chg(chip);

	return IRQ_HANDLED;
}

static int __pm_batt_external_power_changed_work(struct device *dev, void *data)
{
	struct power_supply *psy = &the_chip->batt_psy;
	struct power_supply *epsy = dev_get_drvdata(dev);
	int i, dcin_irq;

	/* Only search for external supply if none is registered */
	if (!the_chip->ext_psy) {
		dcin_irq = the_chip->pmic_chg_irq[DCIN_VALID_IRQ];
		for (i = 0; i < epsy->num_supplicants; i++) {
			if (!strncmp(epsy->supplied_to[i], psy->name, 7)) {
				if (!strncmp(epsy->name, "dc", 2)) {
					the_chip->ext_psy = epsy;
					dcin_valid_irq_handler(dcin_irq,
							the_chip);
				}
			}
		}
	}
	return 0;
}

static void pm_batt_external_power_changed(struct power_supply *psy)
{
	/* Only look for an external supply if it hasn't been registered */
	if (!the_chip->ext_psy)
		class_for_each_device(power_supply_class, NULL, psy,
					 __pm_batt_external_power_changed_work);
}

/**
 * update_heartbeat - internal function to update userspace
 *		per update_time minutes
 *
 */
#define LOW_SOC_HEARTBEAT_MS	20000
static void update_heartbeat(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, update_heartbeat_work);

	power_supply_changed(&chip->batt_psy);
	if (chip->recent_reported_soc <= 20)
		schedule_delayed_work(&chip->update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (LOW_SOC_HEARTBEAT_MS)));
	else
		schedule_delayed_work(&chip->update_heartbeat_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (chip->update_time)));
}
#define VDD_LOOP_ACTIVE_BIT	BIT(3)
#define VDD_MAX_INCREASE_MV	400
static int vdd_max_increase_mv = VDD_MAX_INCREASE_MV;
module_param(vdd_max_increase_mv, int, 0644);

static int ichg_threshold_ua = -400000;
module_param(ichg_threshold_ua, int, 0644);

#define PM8921_CHG_VDDMAX_RES_MV	10
static void adjust_vdd_max_for_fastchg(struct pm8921_chg_chip *chip,
						int vbat_batt_terminal_uv)
{
	int adj_vdd_max_mv, programmed_vdd_max;
	int vbat_batt_terminal_mv;
	int reg_loop;
	int delta_mv = 0;

	if (chip->rconn_mohm == 0) {
		pr_debug("Exiting as rconn_mohm is 0\n");
		return;
	}
	/* adjust vdd_max only in normal temperature zone */
	if (chip->is_bat_cool || chip->is_bat_warm) {
		pr_debug("Exiting is_bat_cool = %d is_batt_warm = %d\n",
				chip->is_bat_cool, chip->is_bat_warm);
		return;
	}

	reg_loop = pm_chg_get_regulation_loop(chip);
	if (!(reg_loop & VDD_LOOP_ACTIVE_BIT)) {
		pr_debug("Exiting Vdd loop is not active reg loop=0x%x\n",
			reg_loop);
		return;
	}
	vbat_batt_terminal_mv = vbat_batt_terminal_uv/1000;
	pm_chg_vddmax_get(the_chip, &programmed_vdd_max);

	delta_mv =  chip->max_voltage_mv - vbat_batt_terminal_mv;

	adj_vdd_max_mv = programmed_vdd_max + delta_mv;
	pr_debug("vdd_max needs to be changed by %d mv from %d to %d\n",
			delta_mv,
			programmed_vdd_max,
			adj_vdd_max_mv);

	if (adj_vdd_max_mv < chip->max_voltage_mv) {
		pr_debug("adj vdd_max lower than default max voltage\n");
		return;
	}

	adj_vdd_max_mv = DIV_ROUND_UP(adj_vdd_max_mv, PM8921_CHG_VDDMAX_RES_MV)
					* PM8921_CHG_VDDMAX_RES_MV;

	if (adj_vdd_max_mv > (chip->max_voltage_mv + vdd_max_increase_mv))
		adj_vdd_max_mv = chip->max_voltage_mv + vdd_max_increase_mv;
	pr_debug("adjusting vdd_max_mv to %d to make "
		"vbat_batt_termial_uv = %d to %d\n",
		adj_vdd_max_mv, vbat_batt_terminal_uv, chip->max_voltage_mv);
	pm_chg_vddmax_set(chip, adj_vdd_max_mv);
}

static void set_appropriate_vbatdet(struct pm8921_chg_chip *chip)
{
	if (chip->is_bat_cool)
		pm_chg_vbatdet_set(the_chip,
			the_chip->cool_bat_voltage
			- the_chip->resume_voltage_delta);
	else if (chip->is_bat_warm)
		pm_chg_vbatdet_set(the_chip,
			the_chip->warm_bat_voltage
			- the_chip->resume_voltage_delta);
	else
		pm_chg_vbatdet_set(the_chip,
			the_chip->max_voltage_mv
			- the_chip->resume_voltage_delta);
}

static void set_appropriate_battery_current(struct pm8921_chg_chip *chip)
{
	unsigned int chg_current = chip->max_bat_chg_current;

	if (chip->is_bat_cool)
		chg_current = min(chg_current, chip->cool_bat_chg_current);

	if (chip->is_bat_warm)
		chg_current = min(chg_current, chip->warm_bat_chg_current);

	if (thermal_mitigation != 0 && chip->thermal_mitigation)
		chg_current = min(chg_current,
				chip->thermal_mitigation[thermal_mitigation]);

	pm_chg_ibatmax_set(the_chip, chg_current);
}

#define TEMP_HYSTERISIS_DECIDEGC 20
static void battery_cool(bool enter)
{
	pr_debug("enter = %d\n", enter);
	if (enter == the_chip->is_bat_cool)
		return;
	the_chip->is_bat_cool = enter;
	if (enter)
		pm_chg_vddmax_set(the_chip, the_chip->cool_bat_voltage);
	else
		pm_chg_vddmax_set(the_chip, the_chip->max_voltage_mv);
	set_appropriate_battery_current(the_chip);
	set_appropriate_vbatdet(the_chip);
}

static void battery_warm(bool enter)
{
	pr_debug("enter = %d\n", enter);
	if (enter == the_chip->is_bat_warm)
		return;
	the_chip->is_bat_warm = enter;
	if (enter)
		pm_chg_vddmax_set(the_chip, the_chip->warm_bat_voltage);
	else
		pm_chg_vddmax_set(the_chip, the_chip->max_voltage_mv);

	set_appropriate_battery_current(the_chip);
	set_appropriate_vbatdet(the_chip);
}

static void check_temp_thresholds(struct pm8921_chg_chip *chip)
{
	int temp = 0;

	temp = get_prop_batt_temp(chip);
	pr_debug("temp = %d, warm_thr_temp = %d, cool_thr_temp = %d\n",
			temp, chip->warm_temp_dc,
			chip->cool_temp_dc);

	if (chip->warm_temp_dc != INT_MIN) {
		if (chip->is_bat_warm
			&& temp < chip->warm_temp_dc - TEMP_HYSTERISIS_DECIDEGC)
			battery_warm(false);
		else if (!chip->is_bat_warm && temp >= chip->warm_temp_dc)
			battery_warm(true);
	}

	if (chip->cool_temp_dc != INT_MIN) {
		if (chip->is_bat_cool
			&& temp > chip->cool_temp_dc + TEMP_HYSTERISIS_DECIDEGC)
			battery_cool(false);
		else if (!chip->is_bat_cool && temp <= chip->cool_temp_dc)
			battery_cool(true);
	}
}

enum {
	CHG_IN_PROGRESS,
	CHG_NOT_IN_PROGRESS,
	CHG_FINISHED,
};

#define VBAT_TOLERANCE_MV	70
#define CHG_DISABLE_MSLEEP	100
static int is_charging_finished(struct pm8921_chg_chip *chip,
			int vbat_batt_terminal_uv, int ichg_meas_ma)
{
	int vbat_programmed, iterm_programmed, vbat_intended;
	int regulation_loop, fast_chg, vcp;
	int rc;
	static int last_vbat_programmed = -EINVAL;

	if (!is_ext_charging(chip)) {
		/* return if the battery is not being fastcharged */
		fast_chg = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
		pr_debug("fast_chg = %d\n", fast_chg);
		if (fast_chg == 0)
			return CHG_NOT_IN_PROGRESS;

		vcp = pm_chg_get_rt_status(chip, VCP_IRQ);
		pr_debug("vcp = %d\n", vcp);
		if (vcp == 1)
			return CHG_IN_PROGRESS;

		/* reset count if battery is hot/cold */
		rc = pm_chg_get_rt_status(chip, BAT_TEMP_OK_IRQ);
		pr_debug("batt_temp_ok = %d\n", rc);
		if (rc == 0)
			return CHG_IN_PROGRESS;

		rc = pm_chg_vddmax_get(chip, &vbat_programmed);
		if (rc) {
			pr_err("couldnt read vddmax rc = %d\n", rc);
			return CHG_IN_PROGRESS;
		}
		pr_debug("vddmax = %d vbat_batt_terminal_uv=%d\n",
			 vbat_programmed, vbat_batt_terminal_uv);

		if (last_vbat_programmed == -EINVAL)
			last_vbat_programmed = vbat_programmed;
		if (last_vbat_programmed !=  vbat_programmed) {
			/* vddmax changed, reset and check again */
			pr_debug("vddmax = %d last_vdd_max=%d\n",
				 vbat_programmed, last_vbat_programmed);
			last_vbat_programmed = vbat_programmed;
			return CHG_IN_PROGRESS;
		}

		if (chip->is_bat_cool)
			vbat_intended = chip->cool_bat_voltage;
		else if (chip->is_bat_warm)
			vbat_intended = chip->warm_bat_voltage;
		else
			vbat_intended = chip->max_voltage_mv;

		if (vbat_batt_terminal_uv / 1000 < vbat_intended) {
			pr_debug("terminal_uv:%d < vbat_intended:%d.\n",
							vbat_batt_terminal_uv,
							vbat_intended);
			return CHG_IN_PROGRESS;
		}

		regulation_loop = pm_chg_get_regulation_loop(chip);
		if (regulation_loop < 0) {
			pr_err("couldnt read the regulation loop err=%d\n",
				regulation_loop);
			return CHG_IN_PROGRESS;
		}
		pr_debug("regulation_loop=%d\n", regulation_loop);

		if (regulation_loop != 0 && regulation_loop != VDD_LOOP)
			return CHG_IN_PROGRESS;
	} /* !is_ext_charging */

	/* reset count if battery chg current is more than iterm */
	rc = pm_chg_iterm_get(chip, &iterm_programmed);
	if (rc) {
		pr_err("couldnt read iterm rc = %d\n", rc);
		return CHG_IN_PROGRESS;
	}

	pr_debug("iterm_programmed = %d ichg_meas_ma=%d\n",
				iterm_programmed, ichg_meas_ma);
	/*
	 * ichg_meas_ma < 0 means battery is drawing current
	 * ichg_meas_ma > 0 means battery is providing current
	 */
	if (ichg_meas_ma > 0)
		return CHG_IN_PROGRESS;

	if (ichg_meas_ma * -1 > iterm_programmed)
		return CHG_IN_PROGRESS;

	return CHG_FINISHED;
}

#define COMP_OVERRIDE_HOT_BANK	6
#define COMP_OVERRIDE_COLD_BANK	7
#define COMP_OVERRIDE_BIT  BIT(1)
static int pm_chg_override_cold(struct pm8921_chg_chip *chip, int flag)
{
	u8 val;
	int rc = 0;

	val = 0x80 | COMP_OVERRIDE_COLD_BANK << 2 | COMP_OVERRIDE_BIT;

	if (flag)
		val |= 0x01;

	rc = pm_chg_write(chip, COMPARATOR_OVERRIDE, val);
	if (rc < 0)
		pr_err("Could not write 0x%x to override rc = %d\n", val, rc);

	pr_debug("btc cold = %d val = 0x%x\n", flag, val);
	return rc;
}

static int pm_chg_override_hot(struct pm8921_chg_chip *chip, int flag)
{
	u8 val;
	int rc = 0;

	val = 0x80 | COMP_OVERRIDE_HOT_BANK << 2 | COMP_OVERRIDE_BIT;

	if (flag)
		val |= 0x01;

	rc = pm_chg_write(chip, COMPARATOR_OVERRIDE, val);
	if (rc < 0)
		pr_err("Could not write 0x%x to override rc = %d\n", val, rc);

	pr_debug("btc hot = %d val = 0x%x\n", flag, val);
	return rc;
}

static void __devinit pm8921_chg_btc_override_init(struct pm8921_chg_chip *chip)
{
	int rc = 0;
	u8 reg;
	u8 val;

	val = COMP_OVERRIDE_HOT_BANK << 2;
	rc = pm_chg_write(chip, COMPARATOR_OVERRIDE, val);
	if (rc < 0) {
		pr_err("Could not write 0x%x to override rc = %d\n", val, rc);
		goto cold_init;
	}
	rc = pm8xxx_readb(chip->dev->parent, COMPARATOR_OVERRIDE, &reg);
	if (rc < 0) {
		pr_err("Could not read bank %d of override rc = %d\n",
				COMP_OVERRIDE_HOT_BANK, rc);
		goto cold_init;
	}
	if ((reg & COMP_OVERRIDE_BIT) != COMP_OVERRIDE_BIT) {
		/* for now override it as not hot */
		rc = pm_chg_override_hot(chip, 0);
		if (rc < 0)
			pr_err("Could not override hot rc = %d\n", rc);
	}

cold_init:
	val = COMP_OVERRIDE_COLD_BANK << 2;
	rc = pm_chg_write(chip, COMPARATOR_OVERRIDE, val);
	if (rc < 0) {
		pr_err("Could not write 0x%x to override rc = %d\n", val, rc);
		return;
	}
	rc = pm8xxx_readb(chip->dev->parent, COMPARATOR_OVERRIDE, &reg);
	if (rc < 0) {
		pr_err("Could not read bank %d of override rc = %d\n",
				COMP_OVERRIDE_COLD_BANK, rc);
		return;
	}
	if ((reg & COMP_OVERRIDE_BIT) != COMP_OVERRIDE_BIT) {
		/* for now override it as not cold */
		rc = pm_chg_override_cold(chip, 0);
		if (rc < 0)
			pr_err("Could not override cold rc = %d\n", rc);
	}
}

static void btc_override_worker(struct work_struct *work)
{
	int decidegc;
	int temp;
	int rc = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, btc_override_work);

	if (!chip->btc_override) {
		pr_err("called when not enabled\n");
		return;
	}

	decidegc = get_prop_batt_temp(chip);

	pr_debug("temp=%d\n", decidegc);

	temp = pm_chg_get_rt_status(chip, BATTTEMP_HOT_IRQ);
	if (temp) {
		if (decidegc < chip->btc_override_hot_decidegc)
			/* stop forcing batt hot */
			rc = pm_chg_override_hot(chip, 0);
			if (rc)
				pr_err("Couldnt write 0 to hot comp\n");
	} else {
		if (decidegc >= chip->btc_override_hot_decidegc)
			/* start forcing batt hot */
			rc = pm_chg_override_hot(chip, 1);
			if (rc && chip->btc_panic_if_cant_stop_chg)
				panic("Couldnt override comps to stop chg\n");
	}

	temp = pm_chg_get_rt_status(chip, BATTTEMP_COLD_IRQ);
	if (temp) {
		if (decidegc > chip->btc_override_cold_decidegc)
			/* stop forcing batt cold */
			rc = pm_chg_override_cold(chip, 0);
			if (rc)
				pr_err("Couldnt write 0 to cold comp\n");
	} else {
		if (decidegc <= chip->btc_override_cold_decidegc)
			/* start forcing batt cold */
			rc = pm_chg_override_cold(chip, 1);
			if (rc && chip->btc_panic_if_cant_stop_chg)
				panic("Couldnt override comps to stop chg\n");
	}

	if ((is_dc_chg_plugged_in(the_chip) || is_usb_chg_plugged_in(the_chip))
		&& get_prop_batt_status(chip) != POWER_SUPPLY_STATUS_FULL) {
		schedule_delayed_work(&chip->btc_override_work,
					round_jiffies_relative(msecs_to_jiffies
						(chip->btc_delay_ms)));
		return;
	}

	rc = pm_chg_override_hot(chip, 0);
	if (rc)
		pr_err("Couldnt write 0 to hot comp\n");
	rc = pm_chg_override_cold(chip, 0);
	if (rc)
		pr_err("Couldnt write 0 to cold comp\n");
}

/**
 * eoc_worker - internal function to check if battery EOC
 *		has happened
 *
 * If all conditions favouring, if the charge current is
 * less than the term current for three consecutive times
 * an EOC has happened.
 * The wakelock is released if there is no need to reshedule
 * - this happens when the battery is removed or EOC has
 * happened
 */
#define CONSECUTIVE_COUNT	3
static void eoc_worker(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct pm8921_chg_chip *chip = container_of(dwork,
				struct pm8921_chg_chip, eoc_work);
	static int count;
	int end;
	int vbat_meas_uv, vbat_meas_mv;
	int ichg_meas_ua, ichg_meas_ma;
	int vbat_batt_terminal_uv;

	pm8921_bms_get_simultaneous_battery_voltage_and_current(
					&ichg_meas_ua,	&vbat_meas_uv);
	vbat_meas_mv = vbat_meas_uv / 1000;
	/* rconn_mohm is in milliOhms */
	ichg_meas_ma = ichg_meas_ua / 1000;
	vbat_batt_terminal_uv = vbat_meas_uv
					+ ichg_meas_ma
					* the_chip->rconn_mohm;

	end = is_charging_finished(chip, vbat_batt_terminal_uv, ichg_meas_ma);

	if (end == CHG_NOT_IN_PROGRESS) {
		count = 0;
		goto eoc_worker_stop;
	}

	if (end == CHG_FINISHED) {
		count++;
	} else {
		count = 0;
	}

	if (count == CONSECUTIVE_COUNT) {
		count = 0;
		pr_info("End of Charging\n");

		pm_chg_auto_enable(chip, 0);

		if (is_ext_charging(chip))
			chip->ext_charge_done = true;

		if (chip->is_bat_warm || chip->is_bat_cool)
			chip->bms_notify.is_battery_full = 0;
		else
			chip->bms_notify.is_battery_full = 1;
		/* declare end of charging by invoking chgdone interrupt */
		chgdone_irq_handler(chip->pmic_chg_irq[CHGDONE_IRQ], chip);
	} else {
		check_temp_thresholds(chip);
		adjust_vdd_max_for_fastchg(chip, vbat_batt_terminal_uv);
		pr_debug("EOC count = %d\n", count);
		schedule_delayed_work(&chip->eoc_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (EOC_CHECK_PERIOD_MS)));
		return;
	}

eoc_worker_stop:
	wake_unlock(&chip->eoc_wake_lock);
	/* set the vbatdet back, in case it was changed to trigger charging */
	set_appropriate_vbatdet(chip);
}

//ASUS BSP Eason_Chang 1030 porting+++
int register_external_dc_charger(struct ext_chg_pm8921 *ext)
{
	if (the_chip == NULL) {
		pr_err("called too early\n");
		return -EINVAL;
	}
	/* TODO check function pointers */
	the_chip->ext = ext;
	the_chip->ext_charging = false;

	if (is_dc_chg_plugged_in(the_chip))
		pm8921_disable_source_current(true); /* Force BATFET=ON */

	handle_start_ext_chg(the_chip);
    //ASUS BSP+++ notify BatteryService 
    if(g_A60K_hwID <A66_HW_ID_ER2){    
	asus_onChargingStart();
    }
    printk("[BAT][8921]%s():\n",__FUNCTION__);
    //ASUS BSP--- notify BatteryService  

	return 0;
}
EXPORT_SYMBOL(register_external_dc_charger);

void unregister_external_dc_charger(struct ext_chg_pm8921 *ext)
{
	if (the_chip == NULL) {
		pr_err("called too early\n");
		return;
	}
	handle_stop_ext_chg(the_chip);
	//ASUS BSP+++ notify BatteryService    
	//asus_onChargingStop(exUNREG_extDC);
    //printk("[ASUS][BAT]%s():\n",__FUNCTION__);
    //ASUS BSP--- notify BatteryService   
	the_chip->ext = NULL;
}
EXPORT_SYMBOL(unregister_external_dc_charger);
//ASUS BSP Eason_Chang 1030 porting-----

/**
 * set_disable_status_param -
 *
 * Internal function to disable battery charging and also disable drawing
 * any current from the source. The device is forced to run on a battery
 * after this.
 */
static int set_disable_status_param(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	pr_info("factory set disable param to %d\n", charging_disabled);
	if (chip) {
		pm_chg_auto_enable(chip, !charging_disabled);
		pm_chg_charge_dis(chip, charging_disabled);
	}
	return 0;
}
module_param_call(disabled, set_disable_status_param, param_get_uint,
					&charging_disabled, 0644);

static int rconn_mohm;
static int set_rconn_mohm(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	if (chip)
		chip->rconn_mohm = rconn_mohm;
	return 0;
}
module_param_call(rconn_mohm, set_rconn_mohm, param_get_uint,
					&rconn_mohm, 0644);
/**
 * set_thermal_mitigation_level -
 *
 * Internal function to control battery charging current to reduce
 * temperature
 */
static int set_therm_mitigation_level(const char *val, struct kernel_param *kp)
{
	int ret;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if (!chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	if (!chip->thermal_mitigation) {
		pr_err("no thermal mitigation\n");
		return -EINVAL;
	}

	if (thermal_mitigation < 0
		|| thermal_mitigation >= chip->thermal_levels) {
		pr_err("out of bound level selected\n");
		return -EINVAL;
	}
//Eason : when thermal too hot, limit charging current +++
    if(g_A60K_hwID >=A66_HW_ID_ER2)
    {
        notifyThermalLimit(thermal_mitigation);
    }
//Eason : when thermal too hot, limit charging current ---   
	set_appropriate_battery_current(chip);
	return ret;
}
module_param_call(thermal_mitigation, set_therm_mitigation_level,
					param_get_uint,
					&thermal_mitigation, 0644);

static int set_usb_max_current(const char *val, struct kernel_param *kp)
{
	int ret, mA;
	struct pm8921_chg_chip *chip = the_chip;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	if (chip) {
		pr_warn("setting current max to %d\n", usb_max_current);
		pm_chg_iusbmax_get(chip, &mA);
		if (mA > usb_max_current)
			pm8921_charger_vbus_draw(usb_max_current);
		return 0;
	}
	return -EINVAL;
}
module_param_call(usb_max_current, set_usb_max_current,
	param_get_uint, &usb_max_current, 0644);

static void free_irqs(struct pm8921_chg_chip *chip)
{
	int i;

	for (i = 0; i < PM_CHG_MAX_INTS; i++)
		if (chip->pmic_chg_irq[i]) {
			free_irq(chip->pmic_chg_irq[i], chip);
			chip->pmic_chg_irq[i] = 0;
		}
}

/* determines the initial present states */
static void __devinit determine_initial_state(struct pm8921_chg_chip *chip)
{
	int fsm_state;
	int is_fast_chg;
//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
	chip->chg_present = !!is_chg_plugged_in(chip);
	notify_usb_of_the_plugin_event(chip->chg_present);
     //ASUS_BSP eason QCPatch:Workaround for USB unplug issue+++
 if(g_A60K_hwID <A66_HW_ID_ER2){
	if (chip->chg_present) {
		schedule_delayed_work(&chip->unplug_check_work,
			round_jiffies_relative(msecs_to_jiffies
				(UNPLUG_CHECK_WAIT_PERIOD_MS)));
	}
}
     //ASUS_BSP eason QCPatch:Workaround for USB unplug issue---
#else
	chip->dc_present = !!is_dc_chg_plugged_in(chip);
	chip->usb_present = !!is_usb_chg_plugged_in(chip);

	notify_usb_of_the_plugin_event(chip->usb_present);
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"

	if (chip->usb_present || chip->dc_present) {
		schedule_delayed_work(&chip->unplug_check_work,
			msecs_to_jiffies(UNPLUG_CHECK_WAIT_PERIOD_MS));
		pm8921_chg_enable_irq(chip, CHG_GONE_IRQ);
	}

	pm8921_chg_enable_irq(chip, DCIN_VALID_IRQ);
	pm8921_chg_enable_irq(chip, USBIN_VALID_IRQ);
	pm8921_chg_enable_irq(chip, BATT_REMOVED_IRQ);
	pm8921_chg_enable_irq(chip, BATT_INSERTED_IRQ);
	pm8921_chg_enable_irq(chip, DCIN_OV_IRQ);
	pm8921_chg_enable_irq(chip, DCIN_UV_IRQ);
	pm8921_chg_enable_irq(chip, CHGFAIL_IRQ);
	pm8921_chg_enable_irq(chip, FASTCHG_IRQ);
	pm8921_chg_enable_irq(chip, VBATDET_LOW_IRQ);
	pm8921_chg_enable_irq(chip, BAT_TEMP_OK_IRQ);

	if (get_prop_batt_present(the_chip) || is_dc_chg_plugged_in(the_chip))
		if (usb_chg_current)
			/*
			 * Reissue a vbus draw call only if a battery
			 * or DC is present. We don't want to brown out the
			 * device if usb is its only source
			 */
			__pm8921_charger_vbus_draw(usb_chg_current);
	usb_chg_current = 0;

	/*
	 * The bootloader could have started charging, a fastchg interrupt
	 * might not happen. Check the real time status and if it is fast
	 * charging invoke the handler so that the eoc worker could be
	 * started
	 */
	is_fast_chg = pm_chg_get_rt_status(chip, FASTCHG_IRQ);
	if (is_fast_chg)
		fastchg_irq_handler(chip->pmic_chg_irq[FASTCHG_IRQ], chip);

	fsm_state = pm_chg_get_fsm_state(chip);
	if (is_battery_charging(fsm_state)) {
		chip->bms_notify.is_charging = 1;
		pm8921_bms_charging_began();
	}

	check_battery_valid(chip);

	pr_debug("usb = %d, dc = %d  batt = %d state=%d\n",
			chip->usb_present,
			chip->dc_present,
			get_prop_batt_present(chip),
			fsm_state);

//+++ ASUS_BSP Allen1_Huang: enable ICS charger mode
#ifdef CONFIG_CHARGER_MODE
	g_chg_present = chip->chg_present;
#endif
//--- ASUS_BSP Allen1_Huang: enable ICS charger mode

//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
    if(g_A60K_hwID <A66_HW_ID_ER2){

	schedule_delayed_work(&asus_chg_set_dc_curr_worker, 25*HZ);
	asus_chg_set_chg_mode(local_chg_src);
    }
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"

	/* Determine which USB trim column to use */
	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8917)
		chip->usb_trim_table = usb_trim_8917_table;
	else if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8038)
		chip->usb_trim_table = usb_trim_8038_table;
}

struct pm_chg_irq_init_data {
	unsigned int	irq_id;
	char		*name;
	unsigned long	flags;
	irqreturn_t	(*handler)(int, void *);
};

#define CHG_IRQ(_id, _flags, _handler) \
{ \
	.irq_id		= _id, \
	.name		= #_id, \
	.flags		= _flags, \
	.handler	= _handler, \
}
struct pm_chg_irq_init_data chg_irq_data[] = {
	CHG_IRQ(USBIN_VALID_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						usbin_valid_irq_handler),
	CHG_IRQ(BATT_INSERTED_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						batt_inserted_irq_handler),
	CHG_IRQ(VBATDET_LOW_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						vbatdet_low_irq_handler),
	CHG_IRQ(CHGWDOG_IRQ, IRQF_TRIGGER_RISING, chgwdog_irq_handler),
	CHG_IRQ(VCP_IRQ, IRQF_TRIGGER_RISING, vcp_irq_handler),
	CHG_IRQ(ATCDONE_IRQ, IRQF_TRIGGER_RISING, atcdone_irq_handler),
	CHG_IRQ(ATCFAIL_IRQ, IRQF_TRIGGER_RISING, atcfail_irq_handler),
	CHG_IRQ(CHGDONE_IRQ, IRQF_TRIGGER_RISING, chgdone_irq_handler),
	CHG_IRQ(CHGFAIL_IRQ, IRQF_TRIGGER_RISING, chgfail_irq_handler),
	CHG_IRQ(CHGSTATE_IRQ, IRQF_TRIGGER_RISING, chgstate_irq_handler),
	CHG_IRQ(LOOP_CHANGE_IRQ, IRQF_TRIGGER_RISING, loop_change_irq_handler),
	CHG_IRQ(FASTCHG_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						fastchg_irq_handler),
	CHG_IRQ(TRKLCHG_IRQ, IRQF_TRIGGER_RISING, trklchg_irq_handler),
	CHG_IRQ(BATT_REMOVED_IRQ, IRQF_TRIGGER_RISING,
						batt_removed_irq_handler),
	CHG_IRQ(BATTTEMP_HOT_IRQ, IRQF_TRIGGER_RISING,
						batttemp_hot_irq_handler),
	CHG_IRQ(CHGHOT_IRQ, IRQF_TRIGGER_RISING, chghot_irq_handler),
	CHG_IRQ(BATTTEMP_COLD_IRQ, IRQF_TRIGGER_RISING,
						batttemp_cold_irq_handler),
	CHG_IRQ(CHG_GONE_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						chg_gone_irq_handler),
	CHG_IRQ(BAT_TEMP_OK_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						bat_temp_ok_irq_handler),
	CHG_IRQ(COARSE_DET_LOW_IRQ, IRQF_TRIGGER_RISING,
						coarse_det_low_irq_handler),
	CHG_IRQ(VDD_LOOP_IRQ, IRQF_TRIGGER_RISING, vdd_loop_irq_handler),
	CHG_IRQ(VREG_OV_IRQ, IRQF_TRIGGER_RISING, vreg_ov_irq_handler),
	CHG_IRQ(VBATDET_IRQ, IRQF_TRIGGER_RISING, vbatdet_irq_handler),
	CHG_IRQ(BATFET_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						batfet_irq_handler),
	CHG_IRQ(DCIN_VALID_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						dcin_valid_irq_handler),
	CHG_IRQ(DCIN_OV_IRQ, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
						dcin_ov_irq_handler),
	CHG_IRQ(DCIN_UV_IRQ, IRQF_TRIGGER_RISING, dcin_uv_irq_handler),
};

static int __devinit request_irqs(struct pm8921_chg_chip *chip,
					struct platform_device *pdev)
{
	struct resource *res;
	int ret, i;

	ret = 0;
	bitmap_fill(chip->enabled_irqs, PM_CHG_MAX_INTS);

	for (i = 0; i < ARRAY_SIZE(chg_irq_data); i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
				chg_irq_data[i].name);
		if (res == NULL) {
			pr_err("couldn't find %s\n", chg_irq_data[i].name);
			goto err_out;
		}
		chip->pmic_chg_irq[chg_irq_data[i].irq_id] = res->start;
		ret = request_irq(res->start, chg_irq_data[i].handler,
			chg_irq_data[i].flags,
			chg_irq_data[i].name, chip);
		if (ret < 0) {
			pr_err("couldn't request %d (%s) %d\n", res->start,
					chg_irq_data[i].name, ret);
			chip->pmic_chg_irq[chg_irq_data[i].irq_id] = 0;
			goto err_out;
		}
		pm8921_chg_disable_irq(chip, chg_irq_data[i].irq_id);
	}
	return 0;

err_out:
	free_irqs(chip);
	return -EINVAL;
}

#define VREF_BATT_THERM_FORCE_ON	BIT(7)
static void detect_battery_removal(struct pm8921_chg_chip *chip)
{
	u8 temp;

	pm8xxx_readb(chip->dev->parent, CHG_CNTRL, &temp);
	pr_debug("upon restart CHG_CNTRL = 0x%x\n",  temp);

	if (!(temp & VREF_BATT_THERM_FORCE_ON))
		/*
		 * batt therm force on bit is battery backed and is default 0
		 * The charger sets this bit at init time. If this bit is found
		 * 0 that means the battery was removed. Tell the bms about it
		 */
		pm8921_bms_invalidate_shutdown_soc();
}

#define ENUM_TIMER_STOP_BIT	BIT(1)
#define BOOT_DONE_BIT		BIT(6)
#define CHG_BATFET_ON_BIT	BIT(3)
#define CHG_VCP_EN		BIT(0)
#define CHG_BAT_TEMP_DIS_BIT	BIT(2)
#define SAFE_CURRENT_MA		1500
#define PM_SUB_REV		0x001
#define MIN_CHARGE_CURRENT_MA	350
#define DEFAULT_SAFETY_MINUTES	500
static int __devinit pm8921_chg_hw_init(struct pm8921_chg_chip *chip)
{
	u8 subrev;
	int rc, vdd_safe, fcc_uah, safety_time = DEFAULT_SAFETY_MINUTES;

	spin_lock_init(&lpm_lock);

	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8921) {
		rc = __pm8921_apply_19p2mhz_kickstart(chip);
		if (rc) {
			pr_err("Failed to apply kickstart rc=%d\n", rc);
			return rc;
		}
	}

	detect_battery_removal(chip);

	rc = pm_chg_masked_write(chip, SYS_CONFIG_2,
					BOOT_DONE_BIT, BOOT_DONE_BIT);
	if (rc) {
		pr_err("Failed to set BOOT_DONE_BIT rc=%d\n", rc);
		return rc;
	}

	vdd_safe = chip->max_voltage_mv + VDD_MAX_INCREASE_MV;

	if (vdd_safe > PM8921_CHG_VDDSAFE_MAX)
		vdd_safe = PM8921_CHG_VDDSAFE_MAX;

	rc = pm_chg_vddsafe_set(chip, vdd_safe);

	if (rc) {
		pr_err("Failed to set safe voltage to %d rc=%d\n",
						chip->max_voltage_mv, rc);
		return rc;
	}
	rc = pm_chg_vbatdet_set(chip,
				chip->max_voltage_mv
				- chip->resume_voltage_delta);
	if (rc) {
		pr_err("Failed to set vbatdet comprator voltage to %d rc=%d\n",
			chip->max_voltage_mv - chip->resume_voltage_delta, rc);
		return rc;
	}

	rc = pm_chg_vddmax_set(chip, chip->max_voltage_mv);
	if (rc) {
		pr_err("Failed to set max voltage to %d rc=%d\n",
						chip->max_voltage_mv, rc);
		return rc;
	}

	if (chip->safe_current_ma == 0)
		chip->safe_current_ma = SAFE_CURRENT_MA;

	rc = pm_chg_ibatsafe_set(chip, chip->safe_current_ma);
	if (rc) {
		pr_err("Failed to set max voltage to %d rc=%d\n",
						SAFE_CURRENT_MA, rc);
		return rc;
	}

	rc = pm_chg_ibatmax_set(chip, chip->max_bat_chg_current);
	if (rc) {
		pr_err("Failed to set max current to 400 rc=%d\n", rc);
		return rc;
	}

	rc = pm_chg_iterm_set(chip, chip->term_current);
	if (rc) {
		pr_err("Failed to set term current to %d rc=%d\n",
						chip->term_current, rc);
		return rc;
	}

	/* Disable the ENUM TIMER */
	rc = pm_chg_masked_write(chip, PBL_ACCESS2, ENUM_TIMER_STOP_BIT,
			ENUM_TIMER_STOP_BIT);
	if (rc) {
		pr_err("Failed to set enum timer stop rc=%d\n", rc);
		return rc;
	}

	/* init with the lowest USB current */
	rc = pm_chg_iusbmax_set(chip, 1);//ASUS BSP Eason_Chang
	if (rc) {
		pr_err("Failed to set usb max to %d rc=%d\n", 1, rc);//ASUS BSP Eason_Chang
		return rc;
	}

	fcc_uah = pm8921_bms_get_fcc();
	if (fcc_uah > 0) {
		safety_time = div_s64((s64)fcc_uah * 60,
						1000 * MIN_CHARGE_CURRENT_MA);
		/* add 20 minutes of buffer time */
		safety_time += 20;

		/* make sure we do not exceed the maximum programmable time */
		if (safety_time > PM8921_CHG_TCHG_MAX)
			safety_time = PM8921_CHG_TCHG_MAX;
	}

	rc = pm_chg_tchg_max_set(chip, safety_time);
	if (rc) {
		pr_err("Failed to set max time to %d minutes rc=%d\n",
						safety_time, rc);
		return rc;
	}

	if (chip->ttrkl_time != 0) {
		rc = pm_chg_ttrkl_max_set(chip, chip->ttrkl_time);
		if (rc) {
			pr_err("Failed to set trkl time to %d minutes rc=%d\n",
							chip->ttrkl_time, rc);
			return rc;
		}
	}

	if (chip->vin_min != 0) {
		rc = pm_chg_vinmin_set(chip, chip->vin_min);
		if (rc) {
			pr_err("Failed to set vin min to %d mV rc=%d\n",
							chip->vin_min, rc);
			return rc;
		}
	} else {
		chip->vin_min = pm_chg_vinmin_get(chip);
	}

	rc = pm_chg_disable_wd(chip);
	if (rc) {
		pr_err("Failed to disable wd rc=%d\n", rc);
		return rc;
	}

	rc = pm_chg_masked_write(chip, CHG_CNTRL_2,
				CHG_BAT_TEMP_DIS_BIT, 0);
	if (rc) {
		pr_err("Failed to enable temp control chg rc=%d\n", rc);
		return rc;
	}
	/* switch to a 3.2Mhz for the buck */
	if (pm8xxx_get_revision(chip->dev->parent) >= PM8XXX_REVISION_8038_1p0)
		rc = pm_chg_write(chip,
			CHG_BUCK_CLOCK_CTRL_8038, 0x15);
	else
		rc = pm_chg_write(chip,
			CHG_BUCK_CLOCK_CTRL, 0x15);

	if (rc) {
		pr_err("Failed to switch buck clk rc=%d\n", rc);
		return rc;
	}

	if (chip->trkl_voltage != 0) {
		rc = pm_chg_vtrkl_low_set(chip, chip->trkl_voltage);
		if (rc) {
			pr_err("Failed to set trkl voltage to %dmv  rc=%d\n",
							chip->trkl_voltage, rc);
			return rc;
		}
	}

	if (chip->weak_voltage != 0) {
		rc = pm_chg_vweak_set(chip, chip->weak_voltage);
		if (rc) {
			pr_err("Failed to set weak voltage to %dmv  rc=%d\n",
							chip->weak_voltage, rc);
			return rc;
		}
	}

	if (chip->trkl_current != 0) {
		rc = pm_chg_itrkl_set(chip, chip->trkl_current);
		if (rc) {
			pr_err("Failed to set trkl current to %dmA  rc=%d\n",
							chip->trkl_voltage, rc);
			return rc;
		}
	}

	if (chip->weak_current != 0) {
		rc = pm_chg_iweak_set(chip, chip->weak_current);
		if (rc) {
			pr_err("Failed to set weak current to %dmA  rc=%d\n",
							chip->weak_current, rc);
			return rc;
		}
	}

	rc = pm_chg_batt_cold_temp_config(chip, chip->cold_thr);
	if (rc) {
		pr_err("Failed to set cold config %d  rc=%d\n",
						chip->cold_thr, rc);
	}

	rc = pm_chg_batt_hot_temp_config(chip, chip->hot_thr);
	if (rc) {
		pr_err("Failed to set hot config %d  rc=%d\n",
						chip->hot_thr, rc);
	}

	rc = pm_chg_led_src_config(chip, chip->led_src_config);
	if (rc) {
		pr_err("Failed to set charger LED src config %d  rc=%d\n",
						chip->led_src_config, rc);
	}

	/* Workarounds for die 3.0 */
	if (pm8xxx_get_revision(chip->dev->parent) == PM8XXX_REVISION_8921_3p0
	&& pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8921) {
		rc = pm8xxx_readb(chip->dev->parent, PM_SUB_REV, &subrev);
		if (rc) {
			pr_err("read failed: addr=%03X, rc=%d\n",
				PM_SUB_REV, rc);
			return rc;
		}
		/* Check if die 3.0.1 is present */
		if (subrev & 0x1)
			pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, 0xA4);
		else
			pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, 0xAC);
	}

	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8917) {
		/* Set PM8917 USB_OVP debounce time to 15 ms */
		rc = pm_chg_masked_write(chip, USB_OVP_CONTROL,
			OVP_DEBOUNCE_TIME, 0x6);
		if (rc) {
			pr_err("Failed to set USB OVP db rc=%d\n", rc);
			return rc;
		}

		/* Enable isub_fine resolution AICL for PM8917 */
		chip->iusb_fine_res = true;
		if (chip->uvd_voltage_mv) {
			rc = pm_chg_uvd_threshold_set(chip,
					chip->uvd_voltage_mv);
			if (rc) {
				pr_err("Failed to set UVD threshold %drc=%d\n",
						chip->uvd_voltage_mv, rc);
				return rc;
			}
		}
	}

	pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, 0xD9);

	/* Disable EOC FSM processing */
	pm_chg_write(chip, CHG_BUCK_CTRL_TEST3, 0x91);

	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON,
						VREF_BATT_THERM_FORCE_ON);
	if (rc)
		pr_err("Failed to Force Vref therm rc=%d\n", rc);

	rc = pm_chg_charge_dis(chip, charging_disabled);
	if (rc) {
		pr_err("Failed to disable CHG_CHARGE_DIS bit rc=%d\n", rc);
		return rc;
	}

	rc = pm_chg_auto_enable(chip, !charging_disabled);
	if (rc) {
		pr_err("Failed to enable charging rc=%d\n", rc);
		return rc;
	}

	if (pm8xxx_get_version(chip->dev->parent) == PM8XXX_VERSION_8921) {
		/* Clear kickstart */
		rc = pm8xxx_writeb(chip->dev->parent, CHG_TEST, 0xD0);
		if (rc) {
			pr_err("Failed to clear kickstart rc=%d\n", rc);
			return rc;
		}

		/* From here the lpm_workaround will be active */
		chip->lockup_lpm_wrkarnd = true;

		/* Enable LPM */
		pm8921_chg_set_lpm(chip, 1);
	}

	if (chip->lockup_lpm_wrkarnd) {
		chip->vreg_xoadc = regulator_get(chip->dev, "vreg_xoadc");
		if (IS_ERR(chip->vreg_xoadc))
			return -ENODEV;

		rc = regulator_set_optimum_mode(chip->vreg_xoadc, 10000);
		if (rc < 0) {
			pr_err("Failed to set configure HPM rc=%d\n", rc);
			return rc;
		}

		rc = regulator_set_voltage(chip->vreg_xoadc, 1800000, 1800000);
		if (rc) {
			pr_err("Failed to set L14 voltage rc=%d\n", rc);
			return rc;
		}

		rc = regulator_enable(chip->vreg_xoadc);
		if (rc) {
			pr_err("Failed to enable L14 rc=%d\n", rc);
			return rc;
		}
	}

	return 0;
}

static int get_rt_status(void *data, u64 * val)
{
	int i = (int)data;
	int ret;

	/* global irq number is passed in via data */
	ret = pm_chg_get_rt_status(the_chip, i);
	*val = ret;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(rt_fops, get_rt_status, NULL, "%llu\n");

static int get_fsm_status(void *data, u64 * val)
{
	u8 temp;

	temp = pm_chg_get_fsm_state(the_chip);
	*val = temp;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fsm_fops, get_fsm_status, NULL, "%llu\n");

static int get_reg_loop(void *data, u64 * val)
{
	u8 temp;

	if (!the_chip) {
		pr_err("%s called before init\n", __func__);
		return -EINVAL;
	}
	temp = pm_chg_get_regulation_loop(the_chip);
	*val = temp;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_loop_fops, get_reg_loop, NULL, "0x%02llx\n");

static int get_reg(void *data, u64 * val)
{
	int addr = (int)data;
	int ret;
	u8 temp;

	ret = pm8xxx_readb(the_chip->dev->parent, addr, &temp);
	if (ret) {
		pr_err("pm8xxx_readb to %x value =%d errored = %d\n",
			addr, temp, ret);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	int addr = (int)data;
	int ret;
	u8 temp;

	temp = (u8) val;
	ret = pm_chg_write(the_chip, addr, temp);
	if (ret) {
		pr_err("pm_chg_write to %x value =%d errored = %d\n",
			addr, temp, ret);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_fops, get_reg, set_reg, "0x%02llx\n");

static int reg_loop;
#define MAX_REG_LOOP_CHAR	10
static int get_reg_loop_param(char *buf, struct kernel_param *kp)
{
	u8 temp;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	temp = pm_chg_get_regulation_loop(the_chip);
	return snprintf(buf, MAX_REG_LOOP_CHAR, "%d", temp);
}
module_param_call(reg_loop, NULL, get_reg_loop_param,
					&reg_loop, 0644);

static int max_chg_ma;
#define MAX_MA_CHAR	10
static int get_max_chg_ma_param(char *buf, struct kernel_param *kp)
{
	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}
	return snprintf(buf, MAX_MA_CHAR, "%d", the_chip->max_bat_chg_current);
}
module_param_call(max_chg_ma, NULL, get_max_chg_ma_param,
					&max_chg_ma, 0644);
static int ibatmax_ma;
static int set_ibat_max(const char *val, struct kernel_param *kp)
{
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = param_set_int(val, kp);
	if (rc) {
		pr_err("error setting value %d\n", rc);
		return rc;
	}

	if (abs(ibatmax_ma - the_chip->max_bat_chg_current)
				<= the_chip->ibatmax_max_adj_ma) {
		rc = pm_chg_ibatmax_set(the_chip, ibatmax_ma);
		if (rc) {
			pr_err("Failed to set ibatmax rc = %d\n", rc);
			return rc;
		}
	}

	return 0;
}
static int get_ibat_max(char *buf, struct kernel_param *kp)
{
	int ibat_ma;
	int rc;

	if (!the_chip) {
		pr_err("called before init\n");
		return -EINVAL;
	}

	rc = pm_chg_ibatmax_get(the_chip, &ibat_ma);
	if (rc) {
		pr_err("ibatmax_get error = %d\n", rc);
		return rc;
	}

	return snprintf(buf, MAX_MA_CHAR, "%d", ibat_ma);
}
module_param_call(ibatmax_ma, set_ibat_max, get_ibat_max,
					&ibatmax_ma, 0644);
enum {
	BAT_WARM_ZONE,
	BAT_COOL_ZONE,
};
static int get_warm_cool(void *data, u64 * val)
{
	if (!the_chip) {
		pr_err("%s called before init\n", __func__);
		return -EINVAL;
	}
	if ((int)data == BAT_WARM_ZONE)
		*val = the_chip->is_bat_warm;
	if ((int)data == BAT_COOL_ZONE)
		*val = the_chip->is_bat_cool;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(warm_cool_fops, get_warm_cool, NULL, "0x%lld\n");

static void create_debugfs_entries(struct pm8921_chg_chip *chip)
{
	int i;

	chip->dent = debugfs_create_dir("pm8921_chg", NULL);

	if (IS_ERR(chip->dent)) {
		pr_err("pmic charger couldnt create debugfs dir\n");
		return;
	}

	debugfs_create_file("CHG_CNTRL", 0644, chip->dent,
			    (void *)CHG_CNTRL, &reg_fops);
	debugfs_create_file("CHG_CNTRL_2", 0644, chip->dent,
			    (void *)CHG_CNTRL_2, &reg_fops);
	debugfs_create_file("CHG_CNTRL_3", 0644, chip->dent,
			    (void *)CHG_CNTRL_3, &reg_fops);
	debugfs_create_file("PBL_ACCESS1", 0644, chip->dent,
			    (void *)PBL_ACCESS1, &reg_fops);
	debugfs_create_file("PBL_ACCESS2", 0644, chip->dent,
			    (void *)PBL_ACCESS2, &reg_fops);
	debugfs_create_file("SYS_CONFIG_1", 0644, chip->dent,
			    (void *)SYS_CONFIG_1, &reg_fops);
	debugfs_create_file("SYS_CONFIG_2", 0644, chip->dent,
			    (void *)SYS_CONFIG_2, &reg_fops);
	debugfs_create_file("CHG_VDD_MAX", 0644, chip->dent,
			    (void *)CHG_VDD_MAX, &reg_fops);
	debugfs_create_file("CHG_VDD_SAFE", 0644, chip->dent,
			    (void *)CHG_VDD_SAFE, &reg_fops);
	debugfs_create_file("CHG_VBAT_DET", 0644, chip->dent,
			    (void *)CHG_VBAT_DET, &reg_fops);
	debugfs_create_file("CHG_IBAT_MAX", 0644, chip->dent,
			    (void *)CHG_IBAT_MAX, &reg_fops);
	debugfs_create_file("CHG_IBAT_SAFE", 0644, chip->dent,
			    (void *)CHG_IBAT_SAFE, &reg_fops);
	debugfs_create_file("CHG_VIN_MIN", 0644, chip->dent,
			    (void *)CHG_VIN_MIN, &reg_fops);
	debugfs_create_file("CHG_VTRICKLE", 0644, chip->dent,
			    (void *)CHG_VTRICKLE, &reg_fops);
	debugfs_create_file("CHG_ITRICKLE", 0644, chip->dent,
			    (void *)CHG_ITRICKLE, &reg_fops);
	debugfs_create_file("CHG_ITERM", 0644, chip->dent,
			    (void *)CHG_ITERM, &reg_fops);
	debugfs_create_file("CHG_TCHG_MAX", 0644, chip->dent,
			    (void *)CHG_TCHG_MAX, &reg_fops);
	debugfs_create_file("CHG_TWDOG", 0644, chip->dent,
			    (void *)CHG_TWDOG, &reg_fops);
	debugfs_create_file("CHG_TEMP_THRESH", 0644, chip->dent,
			    (void *)CHG_TEMP_THRESH, &reg_fops);
	debugfs_create_file("CHG_COMP_OVR", 0644, chip->dent,
			    (void *)CHG_COMP_OVR, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST1", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST1, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST2", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST2, &reg_fops);
	debugfs_create_file("CHG_BUCK_CTRL_TEST3", 0644, chip->dent,
			    (void *)CHG_BUCK_CTRL_TEST3, &reg_fops);
	debugfs_create_file("CHG_TEST", 0644, chip->dent,
			    (void *)CHG_TEST, &reg_fops);

	debugfs_create_file("FSM_STATE", 0644, chip->dent, NULL,
			    &fsm_fops);

	debugfs_create_file("REGULATION_LOOP_CONTROL", 0644, chip->dent, NULL,
			    &reg_loop_fops);

	debugfs_create_file("BAT_WARM_ZONE", 0644, chip->dent,
				(void *)BAT_WARM_ZONE, &warm_cool_fops);
	debugfs_create_file("BAT_COOL_ZONE", 0644, chip->dent,
				(void *)BAT_COOL_ZONE, &warm_cool_fops);

	for (i = 0; i < ARRAY_SIZE(chg_irq_data); i++) {
		if (chip->pmic_chg_irq[chg_irq_data[i].irq_id])
			debugfs_create_file(chg_irq_data[i].name, 0444,
				chip->dent,
				(void *)chg_irq_data[i].irq_id,
				&rt_fops);
	}
}

static int pm8921_charger_suspend_noirq(struct device *dev)
{
	int rc;
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	if (chip->lockup_lpm_wrkarnd) {
		rc = regulator_disable(chip->vreg_xoadc);
		if (rc)
			pr_err("Failed to disable L14 rc=%d\n", rc);

		rc = pm8921_apply_19p2mhz_kickstart(chip);
		if (rc)
			pr_err("Failed to apply kickstart rc=%d\n", rc);
	}

	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON, 0);
	if (rc)
		pr_err("Failed to Force Vref therm off rc=%d\n", rc);
	return 0;
}

static int pm8921_charger_resume_noirq(struct device *dev)
{
	int rc;
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	if (chip->lockup_lpm_wrkarnd) {
		rc = regulator_enable(chip->vreg_xoadc);
		if (rc)
			pr_err("Failed to enable L14 rc=%d\n", rc);

		rc = pm8921_apply_19p2mhz_kickstart(chip);
		if (rc)
			pr_err("Failed to apply kickstart rc=%d\n", rc);
	}

	rc = pm_chg_masked_write(chip, CHG_CNTRL, VREF_BATT_THERM_FORCE_ON,
						VREF_BATT_THERM_FORCE_ON);
	if (rc)
		pr_err("Failed to Force Vref therm on rc=%d\n", rc);
	return 0;
}

static int pm8921_charger_resume(struct device *dev)
{
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	pm8921_fix_charger_lockup(chip); //ASUS_BSP eason QCPatch:fix statemachine lookup
	if (pm8921_chg_is_enabled(chip, LOOP_CHANGE_IRQ)) {
		disable_irq_wake(chip->pmic_chg_irq[LOOP_CHANGE_IRQ]);
		pm8921_chg_disable_irq(chip, LOOP_CHANGE_IRQ);
	}

	if (chip->btc_override && (is_dc_chg_plugged_in(the_chip) ||
					is_usb_chg_plugged_in(the_chip)))
		schedule_delayed_work(&chip->btc_override_work, 0);

	return 0;
}

static int pm8921_charger_suspend(struct device *dev)
{
	struct pm8921_chg_chip *chip = dev_get_drvdata(dev);

	if (chip->btc_override)
		cancel_delayed_work_sync(&chip->btc_override_work);

	if (is_usb_chg_plugged_in(chip)) {
		pm8921_chg_enable_irq(chip, LOOP_CHANGE_IRQ);
		enable_irq_wake(chip->pmic_chg_irq[LOOP_CHANGE_IRQ]);
	}

	return 0;
}

/*
//ASUS_BSP eason QCPatch:fix statemachine lookup+++
static int idle_enter_exit_notifier(struct notifier_block *nb,
	unsigned long val, void *v)
{
	struct pm8921_chg_chip *chip = container_of(nb,
					struct pm8921_chg_chip, notifier);
	enum msm_pm_sleep_mode sleep_mode = (enum msm_pm_sleep_mode)v;

	if (val == MSM_CPUIDLE_STATE_EXIT
		&& sleep_mode == MSM_PM_SLEEP_MODE_POWER_COLLAPSE) {
		pm8921_fix_charger_lockup(chip);
	}

	return 0;
}
//ASUS_BSP eason QCPatch:fix statemachine lookup---
*/

//ASUS_BSP+++ 
#ifdef CONFIG_EARLYSUSPEND
static void pm8921_early_suspend(struct early_suspend *h)
{
    printk("%s+++\r\n",__FUNCTION__);
    if(g_A60K_hwID >=A66_HW_ID_ER2){

            return;
    }
    
    if(is_chg_plugged_in(the_chip)){
        if (delayed_work_pending(&the_chip->chg_status_check_work)){
            
            cancel_delayed_work_sync(&the_chip->chg_status_check_work);
        }
    }
    printk("%s---\r\n",__FUNCTION__);
}

static void pm8921_late_resume(struct early_suspend *h)
{
    printk("%s+++\r\n",__FUNCTION__);

    if(g_A60K_hwID >=A66_HW_ID_ER2){

            return;
    }

    
    if(is_chg_plugged_in(the_chip)){
        if (!delayed_work_pending(&the_chip->chg_status_check_work)){


            printk("Re-sche checking state worker...\r\n");
            schedule_delayed_work(&the_chip->chg_status_check_work,
                          DELAY_FOR_CHK_STATUS_IN_RESUME*HZ);

        }
    }
    printk("%s---\r\n",__FUNCTION__);

}

struct early_suspend pm8921_early_suspend_handler = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = pm8921_early_suspend,
    .resume = pm8921_late_resume,
};
#endif
//ASUS BSP---  


static int __devinit pm8921_charger_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct pm8921_chg_chip *chip;
	const struct pm8921_charger_platform_data *pdata
				= pdev->dev.platform_data;

//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
	struct asus_bat_phone_bat_struct *phone_bat;
	printk(DBGMSK_BAT_DEBUG "[BAT] %s() +++\n", __FUNCTION__);
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"

	if (!pdata) {
		pr_err("missing platform data\n");
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct pm8921_chg_chip),
					GFP_KERNEL);
	if (!chip) {
		pr_err("Cannot allocate pm_chg_chip\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	chip->ttrkl_time = pdata->ttrkl_time;
	chip->update_time = pdata->update_time;
	chip->max_voltage_mv = pdata->max_voltage;
	chip->alarm_low_mv = pdata->alarm_low_mv;
	chip->alarm_high_mv = pdata->alarm_high_mv;
	chip->min_voltage_mv = pdata->min_voltage;
	chip->safe_current_ma = pdata->safe_current_ma;
	chip->uvd_voltage_mv = pdata->uvd_thresh_voltage;
	chip->resume_voltage_delta = pdata->resume_voltage_delta;
	chip->resume_charge_percent = pdata->resume_charge_percent;
	chip->term_current = pdata->term_current;
	chip->vbat_channel = pdata->charger_cdata.vbat_channel;
	chip->batt_temp_channel = pdata->charger_cdata.batt_temp_channel;
	chip->batt_id_channel = pdata->charger_cdata.batt_id_channel;
	chip->batt_id_min = pdata->batt_id_min;
	chip->batt_id_max = pdata->batt_id_max;
	if (pdata->cool_temp != INT_MIN)
		chip->cool_temp_dc = pdata->cool_temp * 10;
	else
		chip->cool_temp_dc = INT_MIN;

	if (pdata->warm_temp != INT_MIN)
		chip->warm_temp_dc = pdata->warm_temp * 10;
	else
		chip->warm_temp_dc = INT_MIN;

	chip->temp_check_period = pdata->temp_check_period;
	chip->max_bat_chg_current = pdata->max_bat_chg_current;
	/* Assign to corresponding module parameter */
	usb_max_current = pdata->usb_max_current;
	chip->cool_bat_chg_current = pdata->cool_bat_chg_current;
	chip->warm_bat_chg_current = pdata->warm_bat_chg_current;
	chip->cool_bat_voltage = pdata->cool_bat_voltage;
	chip->warm_bat_voltage = pdata->warm_bat_voltage;
	chip->trkl_voltage = pdata->trkl_voltage;
	chip->weak_voltage = pdata->weak_voltage;
	chip->trkl_current = pdata->trkl_current;
	chip->weak_current = pdata->weak_current;
	chip->vin_min = pdata->vin_min;
	chip->thermal_mitigation = pdata->thermal_mitigation;
	chip->thermal_levels = pdata->thermal_levels;

	chip->cold_thr = pdata->cold_thr;
	chip->hot_thr = pdata->hot_thr;
	chip->rconn_mohm = pdata->rconn_mohm;
	chip->led_src_config = pdata->led_src_config;
	chip->has_dc_supply = pdata->has_dc_supply;
	chip->battery_less_hardware = pdata->battery_less_hardware;
	chip->btc_override = pdata->btc_override;
	if (chip->btc_override) {
		chip->btc_delay_ms = pdata->btc_delay_ms;
		chip->btc_override_cold_decidegc
			= pdata->btc_override_cold_degc * 10;
		chip->btc_override_hot_decidegc
			= pdata->btc_override_hot_degc * 10;
		chip->btc_panic_if_cant_stop_chg
			= pdata->btc_panic_if_cant_stop_chg;
	}

	if (chip->battery_less_hardware)
		charging_disabled = 1;

	chip->ibatmax_max_adj_ma = find_ibat_max_adj_ma(
					chip->max_bat_chg_current);

	rc = pm8921_chg_hw_init(chip);
	if (rc) {
		pr_err("couldn't init hardware rc=%d\n", rc);
		goto free_chip;
	}

//ASUS_BSP +++
	if (chip->btc_override)
		pm8921_chg_btc_override_init(chip);

	chip->stop_chg_upon_expiry = pdata->stop_chg_upon_expiry;

#ifdef CONFIG_MAXIM_8934_CHARGER
    if(g_A60K_hwID < A66_HW_ID_ER2){
	chip->usb_psy.name = "usb";
	chip->usb_psy.type = POWER_SUPPLY_TYPE_USB;
	chip->usb_psy.supplied_to = pm_power_supplied_to;
	chip->usb_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to);
	chip->usb_psy.properties = pm_power_props_usb;
	chip->usb_psy.num_properties = ARRAY_SIZE(pm_power_props_usb);
	chip->usb_psy.get_property = pm_power_get_property_usb;
	chip->usb_psy.set_property = pm_power_set_property_usb;
	chip->usb_psy.property_is_writeable = usb_property_is_writeable;

	chip->dc_psy.name = "pm8921-dc";
	chip->dc_psy.type = POWER_SUPPLY_TYPE_MAINS;
	chip->dc_psy.supplied_to = pm_power_supplied_to;
	chip->dc_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to);
	chip->dc_psy.properties = pm_power_props_mains;
	chip->dc_psy.num_properties = ARRAY_SIZE(pm_power_props_mains);
	chip->dc_psy.get_property = pm_power_get_property_mains;
}
#endif
//ASUS_BSP --- 
	chip->batt_psy.name = "battery";
	chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.properties = msm_batt_power_props;
	chip->batt_psy.num_properties = ARRAY_SIZE(msm_batt_power_props);
	chip->batt_psy.get_property = pm_batt_power_get_property;
	chip->batt_psy.external_power_changed = pm_batt_external_power_changed;

//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
	local_chip = chip;
	phone_bat = kzalloc(sizeof(struct asus_bat_phone_bat_struct), GFP_KERNEL);
	if (!phone_bat) {
		pr_err("[BAT] cannot allocate asus_bat_phone_bat_struct\n");
		return -ENOMEM;
//TODO:  goto and free 
	}

	phone_bat->phone_bat_power_supply_changed = api_phone_bat_power_supply_changed;
	phone_bat->get_prop_bat_present_byhw = api_get_prop_batt_present;
	phone_bat->get_prop_bat_capacity_byhw = api_get_prop_batt_capacity;
	phone_bat->get_prop_bat_status_byhw = api_get_prop_batt_status;
    //ASUS_BSP +++ Eason_Chang "to check if chgin"
    phone_bat->get_prop_bat_chgin_byhw = api_get_prop_batt_chgin;
    //ASUS_BSP --- Eason_Chang "to check if chgin"
	phone_bat->get_prop_charge_type_byhw = api_get_prop_charge_type;
	phone_bat->get_prop_batt_health_byhw = api_get_prop_batt_health;
	phone_bat->get_prop_batt_volt_byhw = api_get_prop_batt_volt;
	phone_bat->get_prop_batt_curr_byhw = api_get_prop_batt_curr;

	asus_bat_set_phone_bat(phone_bat);
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"

//ASUS_BSP +++
#ifdef CONFIG_MAXIM_8934_CHARGER
    
        if(g_A60K_hwID < A66_HW_ID_ER2){

	rc = power_supply_register(chip->dev, &chip->usb_psy);
	if (rc < 0) {
		pr_err("power_supply_register usb failed rc = %d\n", rc);
		goto free_chip;
	}
//ASUS BSP Eason_Chang 1030 porting
	rc = power_supply_register(chip->dev, &chip->dc_psy);
	if (rc < 0) {
		pr_err("power_supply_register dc failed rc = %d\n", rc);
		goto unregister_usb;
	}
}
#endif
//ASUS_BSP ---
	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		pr_err("power_supply_register batt failed rc = %d\n", rc);
		goto unregister_dc;
	}
//ASUS BSP Eason_Chang 1030 porting

	platform_set_drvdata(pdev, chip);
	the_chip = chip;

	/* set initial state of the USB charger type to UNKNOWN */
	power_supply_set_supply_type(&chip->usb_psy, POWER_SUPPLY_TYPE_UNKNOWN);

	wake_lock_init(&chip->eoc_wake_lock, WAKE_LOCK_SUSPEND, "pm8921_eoc");
    //ASUS_BSP eason QCPatch:Workaround for USB unplug issue+++
	wake_lock_init(&chip->unplug_wrkarnd_restore_wake_lock,
			WAKE_LOCK_SUSPEND, "pm8921_unplug_wrkarnd");
    //ASUS_BSP eason QCPatch:Workaround for USB unplug issue---

	INIT_DELAYED_WORK(&chip->eoc_work, eoc_worker);
	INIT_DELAYED_WORK(&chip->vin_collapse_check_work,
						vin_collapse_check_worker);
    //ASUS_BSP eason QCPatch:Workaround for USB unplug issue+++
	INIT_DELAYED_WORK(&chip->unplug_wrkarnd_restore_work,
					unplug_wrkarnd_restore_worker);
	INIT_DELAYED_WORK(&chip->unplug_check_work, unplug_check_worker);
    //ASUS_BSP eason QCPatch:Workaround for USB unplug issue----

	INIT_WORK(&chip->bms_notify.work, bms_notify);
	INIT_WORK(&chip->battery_id_valid_work, battery_id_valid);

	INIT_DELAYED_WORK(&chip->update_heartbeat_work, update_heartbeat);
	INIT_DELAYED_WORK(&chip->btc_override_work, btc_override_worker);

	rc = request_irqs(chip, pdev);
	if (rc) {
		pr_err("couldn't register interrupts rc=%d\n", rc);
		goto unregister_batt;
	}

	enable_irq_wake(chip->pmic_chg_irq[USBIN_VALID_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[DCIN_VALID_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[VBATDET_LOW_IRQ]);
	enable_irq_wake(chip->pmic_chg_irq[FASTCHG_IRQ]);

	create_debugfs_entries(chip);

	/* determine what state the charger is in */
	determine_initial_state(chip);

	if (chip->update_time) {
//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
		pr_err("[BAT] error!! should not use qualcomm battery update work \r\n");
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"
		INIT_DELAYED_WORK(&chip->update_heartbeat_work,
							update_heartbeat);
		schedule_delayed_work(&chip->update_heartbeat_work,
				      round_jiffies_relative(msecs_to_jiffies
							(chip->update_time)));
	}

//ASUS_BSP +++ Josh_Liao "add asus battery driver"
#ifdef CONFIG_BATTERY_ASUS
if(g_A60K_hwID <A66_HW_ID_ER2){

if (is_chg_plugged_in(chip)==1 && asus_chg_get_chg_mode()!=2){
	printk("[BAT]:AC/USB set srcUSB\n");
	//pm8921_charger_vbus_draw(USB_CHG_CURR);
	asus_chg_set_chg_mode(ASUS_CHG_SRC_USB);
}
}
	printk(DBGMSK_BAT_DEBUG "[BAT] %s() ---\n", __FUNCTION__);
#endif /* CONFIG_BATTERY_ASUS */
//ASUS_BSP --- Josh_Liao "add asus battery driver"	
//ASUS BSP+++ Eason notify BatteryService   
    gSaved8921FsmState = pm_chg_get_fsm_state(chip);

    chip->chg_status_check_wq = create_singlethread_workqueue("PM8921ChgStatusCheckQueue");

    INIT_DELAYED_WORK(&chip->chg_status_check_work,
                        chg_status_check_work);

    chip->chg_state_polling_interval = CHK_STATUS_CHECK_PERIOD_S;

    chip->chgdone_reported  = false;

    chip->fastchg_done = false;

//ASUS BSP--- Eason notify BatteryService 
//ASUS BSP +++ Eason Chang add BAT global variable
 INIT_DELAYED_WORK(&chip->update_BATinfo_worker,\
                            UpdateGlobalValue_work);
 schedule_delayed_work(&chip->update_BATinfo_worker,\
                         round_jiffies_relative(msecs_to_jiffies(UPDATE_BATGLOBAL_FIRSTTIME)));
//ASUS BSP --- Eason Chang add BAT global variable

	return 0;

unregister_batt:
	wake_lock_destroy(&chip->eoc_wake_lock);
	power_supply_unregister(&chip->batt_psy);
unregister_dc:
	power_supply_unregister(&chip->dc_psy);
unregister_usb:
	power_supply_unregister(&chip->usb_psy);
free_chip:
	kfree(chip);
	return rc;
}

static int __devexit pm8921_charger_remove(struct platform_device *pdev)
{
	struct pm8921_chg_chip *chip = platform_get_drvdata(pdev);

	regulator_put(chip->vreg_xoadc);
	free_irqs(chip);
	platform_set_drvdata(pdev, NULL);
	the_chip = NULL;
	kfree(chip);
	return 0;
}
static const struct dev_pm_ops pm8921_pm_ops = {
	.suspend	= pm8921_charger_suspend,
	.suspend_noirq  = pm8921_charger_suspend_noirq,
	.resume_noirq   = pm8921_charger_resume_noirq,
	.resume		= pm8921_charger_resume,
};
static struct platform_driver pm8921_charger_driver = {
	.probe		= pm8921_charger_probe,
	.remove		= __devexit_p(pm8921_charger_remove),
	.driver		= {
			.name	= PM8921_CHARGER_DEV_NAME,
			.owner	= THIS_MODULE,
			.pm	= &pm8921_pm_ops,
	},
};

static int __init pm8921_charger_init(void)
{
	return platform_driver_register(&pm8921_charger_driver);
}

static void __exit pm8921_charger_exit(void)
{
	platform_driver_unregister(&pm8921_charger_driver);
}

late_initcall(pm8921_charger_init);
module_exit(pm8921_charger_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PMIC8921 charger/battery driver");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:" PM8921_CHARGER_DEV_NAME);
