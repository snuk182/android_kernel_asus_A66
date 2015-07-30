#ifdef CONFIG_BATTERY_ASUS_SERVICE
#include <AXC_BatteryService.h>
#include <linux/kernel.h>
#include <linux/rtc.h>
//ASUS BSP Eason_Chang +++ batteryservice to fsm
#include "../fsm/AXC_Charging_FSM.h"
AXC_Charging_FSM *lpFSM;
//ASUS BSP Eason_Chang --- batteryservice to fsm
//ASUS BSP Eason_Chang +++ batteryservice to gauge
#include "../gauge/axc_gaugefactory.h"
#include "../gauge/axi_gauge.h"
#include "../gauge/AXI_CapacityFilter.h"
#include "../capfilter/axc_cap_filter_factory.h"
#include "../capfilter/axi_cap_filter.h"
#include "../capfilter/axc_cap_filter_a66.h"
#include "../capfilter/axc_cap_filter_p02.h"
#include <linux/time.h>
//ASUS BSP Eason_Chang --- batteryservice to gauge
//ASUS_BSP +++ Eason_Chang BalanceMode
#include <linux/asus_chg.h>
#include <linux/notifier.h>

#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#include <linux/microp_notify.h>

#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include "../charger/axi_charger.h" 
static int IsBalanceMode = 1;//default 1.  0:PowerbankMode, 1:balanceMode, 2:ForcePowerBankMode
//Eason: do ForcePowerBankMode+++
#include <linux/microp.h>
extern int uP_nuvoton_write_reg(int cmd, void *data);
//Eason: do ForcePowerBankMode---
extern int IsBalanceTest(void);
extern int GetBalanceModeStartRatio(void);
extern int GetBalanceModeStopRatio(void);
extern int GetBalanceModeA66CAP(void);
extern int BatteryServiceGetPADCAP(void);
extern int BatteryServiceReportPADCAP(void);
static int IsBalanceCharge = 1;
static int IsPowerBankCharge = 1;
static int LastTimeIsBalMode = 1;
static struct AXC_BatteryService *balance_this=NULL;
//Eason: A68 new balance mode +++

static bool IsSystemdraw = false;
static bool IsBalanceSuspendStartcharge = false;
static bool IsKeepChgFrom15pTo19p = false;//Eason:balance mode keep charge from Cap 15 to 19

//Eason: A68 new balance mode ---
//ASUS_BSP --- Eason_Chang BalanceMode
//ASUS_BSP +++ Eason_Chang add event log +++
#include <linux/asusdebug.h>
//ASUS_BSP +++ Eason_Chang add event log ---
#include <linux/wakelock.h>
#include <linux/gpio.h> //Eason:get cable In/Out at first time ask Cap
#define maxim8934DCIN 106 //Eason: boot up in BatLow situation, take off cable can shutdown
//ASUS_BSP +++ Eason_Chang charger
extern AXI_Charger * getAsusCharger(void);
static struct AXI_Charger *gpCharger = NULL;
//ASUS_BSP --- Eason_Chang charger
extern int AX_MicroP_IsECDockIn(void);
extern void BatteryService_P02update(void);
extern bool reportRtcReady(void);
extern void asus_bat_update_DockAcOnline(void);
extern void asus_bat_update_PadAcOnline(void);
#define SUSPEND_DISCHG_CURRENT 10
#define DOCK_SUSPEND_DISCHG_CURRENT 18
#define MAX_DISCHG_CURRENT    700
#define USB_CHG_CURRENT       500
#define PAD_CHG_CURRENT       500
#define DOCK_DISCHG_CURRENT   500
#define AC_CHG_CURRENT        900

#define BAT_CAP_REPLY_ERR	-1
#define RESUME_UPDATE_TIME   600      //10 min
#define RESUME_UPDATE_TIMEwhenCapLess20  600  //10min
#define RESUME_UPDATE_TIMEwhenBATlow  300  //10min
#define FORCERESUME_UPDATE_TIME   300  //5 min
#define DOCKRESUME_UPDATE_TIME   300  //5 min
#define RTC_READY_DELAY_TIME   20
#define KEEP_CAPACITY_TIME 300

//Eason set alarm +++
#include <linux/android_alarm.h>
static struct alarm bat_alarm;
static struct alarm batLow_alarm;
static struct alarm cableIn_alarm;
static DEFINE_SPINLOCK(bat_alarm_slock);
static DEFINE_SPINLOCK(batLow_alarm_slock);
static DEFINE_SPINLOCK(cableIn_alarm_slock);
struct wake_lock bat_alarm_wake_lock;
struct wake_lock batLow_alarm_wake_lock;
struct wake_lock cableIn_alarm_wake_lock;
static DECLARE_WAIT_QUEUE_HEAD(bat_alarm_wait_queue);
static DECLARE_WAIT_QUEUE_HEAD(batLow_alarm_wait_queue);
static DECLARE_WAIT_QUEUE_HEAD(cableIn_alarm_wait_queue);
static uint32_t alarm_enabled;
static uint32_t batLowAlarm_enabled;
static uint32_t cableInAlarm_enabled;
extern void alarm_start_range(struct alarm *alarm, ktime_t start, ktime_t end);
#define RTCSetInterval 610
//Eason: dynamic set Pad alarm +++
//#define RTCSetIntervalwhenCapLess20  610

#define RTCSetIntervalwhenBalSuspendStopChg 3610
#define RTCSetIntervalwhenAlarmIntervalLess3min 190
static int RTCSetIntervalwhenBalanceMode = RTCSetInterval;
static bool InSuspendNeedDoPadAlarmHandler=false;//in suspend set true, in late resume set false,
												//Pad alarm handler need to do only when display off												

//Eason: dynamic set Pad alarm ---
#define RTCSetIntervalwhenBATlow  310
#define RTCSetIntervalwhenCABLEIn  3610
#define CapChangeRTCInterval 20
//Eason set alarm ---
// when A66 Cap = 0% shutdown device no matter if has cable+++ 
extern bool g_AcUsbOnline_Change0;
extern void AcUsbPowerSupplyChange(void);
extern void PadDock_AC_PowerSupplyChange(void);
// when A66 Cap = 0% shutdown device no matter if has cable---
//Eason boot up in BatLow situation, take off cable can shutdown+++
extern bool g_BootUp_IsBatLow;
//Eason boot up in BatLow situation, take off cable can shutdown---
//Eason : In suspend have same cap don't update savedTime +++
bool SameCapDontUpdateSavedTime = false;
extern bool g_RTC_update;
//Eason : In suspend have same cap don't update savedTime ---
//Eason : prevent thermal too hot, limit charging current in phone call+++
extern bool g_audio_limit;
static bool IsPhoneOn = false;
//Eason : prevent thermal too hot, limit charging current in phone call---
//Eason : when thermal too hot, limit charging current +++ 
extern bool g_padMic_On; 
extern bool g_thermal_limit;
static bool IsThermalHot = false;
//Eason : when thermal too hot, limit charging current ---
//Eason : if last time is 10mA +++
static bool IsLastTimeMah10mA = false;
static bool IfUpdateSavedTime = false;	
//Eason : if last time is 10mA ---

//Eason : prevent thermal too hot, limit charging current in phone call+++
extern void setChgDrawACTypeCurrent(void);
static void judgePhoneOnCurLimit(void)
{
	
	if( (true==IsPhoneOn)&&(balance_this->A66_capacity>20) )
   	{ 
		g_audio_limit = true;
		printk("[BAT][Ser]:judge g_audio_limit true\n");
		   
		if( (NORMAL_CURRENT_CHARGER_TYPE==balance_this->chargerType)
		    || (HIGH_CURRENT_CHARGER_TYPE==balance_this->chargerType) ) 
   		{
		setChgDrawACTypeCurrent();
		}
	}else{
		g_audio_limit = false;
		printk("[BAT][Ser]:judge g_audio_limit false\n");

		if( (NORMAL_CURRENT_CHARGER_TYPE==balance_this->chargerType)
		    || (HIGH_CURRENT_CHARGER_TYPE==balance_this->chargerType) ) 
   		{	
		setChgDrawACTypeCurrent();
		}
	}
}

void SetLimitCurrentInPhoneCall(bool phoneOn)
{  
   if(phoneOn)
   {
        IsPhoneOn = true;
        printk("[BAT][Ser]:Phone call on\n");
   }else{
    	IsPhoneOn = false;
    	g_audio_limit = false;
    	printk("[BAT][Ser]:Phone call off\n");
   }

   if(g_A60K_hwID >=A66_HW_ID_ER2)
   {
       	judgePhoneOnCurLimit();
   }
}
//Eason : prevent thermal too hot, limit charging current in phone call---
//Eason : when thermal too hot, limit charging current +++
static void judgeThermalCurrentLimit(void)
{
    if( (true==IsThermalHot)&&(balance_this->A66_capacity>20) )
    {
            g_thermal_limit = true;
            printk("[BAT][Ser]:judge g_thermal_limit true\n");
            
            if( (NORMAL_CURRENT_CHARGER_TYPE==balance_this->chargerType)
                ||(HIGH_CURRENT_CHARGER_TYPE==balance_this->chargerType) )
        	{
    			setChgDrawACTypeCurrent();
            }    
    }else{
            g_thermal_limit = false;
             printk("[BAT][Ser]:judge g_thermal_limit false\n");

            if( (NORMAL_CURRENT_CHARGER_TYPE==balance_this->chargerType)
                ||(HIGH_CURRENT_CHARGER_TYPE==balance_this->chargerType) )
        	{
    			setChgDrawACTypeCurrent();
            }    
    }
}

void notifyThermalLimit(int thermalnotify)
{
    if(1==thermalnotify)
	{
			IsThermalHot = true;
            printk("[BAT][Ser]:Thermal hot \n");
	}else{
        	IsThermalHot = false;
			g_thermal_limit = false;
            printk("[BAT][Ser]:Thermal normal \n");
    }    

    judgeThermalCurrentLimit();
    
}
//Eason : when thermal too hot, limit charging current ---

//ASUS_BSP Eason when audio on, draw 1A from Pad ++++
extern void setChgDrawPadCurrent(bool audioOn);
void SetPadCurrentDependOnAudio(bool audioOn)
{  
   if( (1==AX_MicroP_IsP01Connected())&&(g_A60K_hwID >=A66_HW_ID_ER2) )
   {
   	setChgDrawPadCurrent(audioOn);
   }
}
//ASUS_BSP Eason when audio on, draw 1A from Pad ---
//ASUS BSP Eason_Chang +++ batteryservice to fsm
static void AXC_BatteryService_reportPropertyCapacity(struct AXC_BatteryService *_this, int refcapacity);
int ReportBatteryServiceDockCap(void)
{
    return balance_this->Dock_capacity;
}  
int ReportBatteryServiceP02Cap(void)
{
    return balance_this->Pad_capacity;
}    
static void BatteryService_enable_ChargingFsm(AXC_BatteryService *_this)
{
    if(NULL == _this->fsm){

        _this->fsm = getChargingFSM(E_ASUS_A66_FSM_CHARGING_TYPE,&_this->fsmCallback);

        _this->fsmState = _this->fsm->getState(_this->fsm);
    }
}  
//ASUS BSP Eason_Chang --- batteryservice to fsm
//ASUS BSP Eason_Chang +++ batteryservice to gauge
static void BatteryService_enable_Gauge(AXC_BatteryService *_this)
{
    if(NULL == _this->gauge){

        AXC_GaugeFactory_GetGaugeV2(E_SW_GAUGE_V2_TYPE , &_this->gauge, &_this->gaugeCallback);
    }
    if(NULL == _this->P02gauge){

        AXC_GaugeFactory_GetGaugeV2(E_HW_GAUGE_PAD_TYPE , &_this->P02gauge, &_this->P02gaugeCallback);
    }
    if(NULL == _this->Dockgauge){

        AXC_GaugeFactory_GetGaugeV2(E_HW_GAUGE_DOCK_TYPE, &_this->Dockgauge, &_this->DockgaugeCallback);
    }
    
}
//ASUS BSP Eason_Chang --- batteryservice to gauge

static void BatteryService_enable_Filter(AXC_BatteryService *_this)
{
    if(NULL == _this->gpCapFilterA66){

       AXC_Cap_Filter_Get(E_CAP_FILTER_PHONE_A66, &_this->gpCapFilterA66, 1500);
    }
    if(NULL == _this->gpCapFilterP02){

       AXC_Cap_Filter_Get(E_CAP_FILTER_PAD_P02, &_this->gpCapFilterP02, 3300);
    }
    if(NULL == _this->gpCapFilterDock){

       AXC_Cap_Filter_Get(E_CAP_FILTER_DOCK, &_this->gpCapFilterDock, 3300);
    }
    

}
//ASUS_BSP  +++ Eason_Chang charger
static void NotifyForChargerStateChanged(struct AXI_Charger *apCharger, AXE_Charger_Type aeCharger_Mode)
{
#ifdef CONFIG_BATTERY_ASUS_SERVICE

    if(NULL == balance_this){

        return;
    }

    balance_this->miParent.onCableInOut(&balance_this->miParent,aeCharger_Mode);

    balance_this->isMainBatteryChargingDone = false;    
    
#endif
}
static void onChargingStart(struct AXI_Charger *apCharger, bool startCharging)
{


}
//ASUS_BSP  --- Eason_Chang charger


//ASUS_BSP +++ Eason_Chang BalanceMode
static void set_microp_vbus(int level)
{
    int rt;
 	   rt = AX_MicroP_setGPIOOutputPin(OUT_uP_VBUS_EN, level);
    if (rt<0){
           printk("[BAT][Bal]microp set error\n");
    }else if(rt == 0){
           printk("[BAT][Bal]microp set success\n");
    }     
}

static int get_microp_vbus(void)
{
    return AX_MicroP_getGPIOOutputPinLevel(OUT_uP_VBUS_EN);
}

void Init_Microp_Vbus__Chg(void)
{
         gpCharger->EnableCharging(gpCharger,true);         
         set_microp_vbus(1);
         IsBalanceCharge = 1;
         IsPowerBankCharge = 1;
         balance_this->fsm->onChargingStart(balance_this->fsm);
         printk("[BAT][Bal]InitVbus:%d,InitChg:%d\n",get_microp_vbus(),gpCharger->IsCharging(gpCharger));
}    

void  openMicropVbusBeforeShutDown(void){  
         set_microp_vbus(1);
}    
//ASUS_BSP --- Eason_Chang BalanceMode
//ASUS_BSP +++ Eason_Chang BalanceMode
static void Do_PowerBankMode(void)
{
   
   printk("[BAT][Bal]:DoPowerBank+++\n");
   //set_microp_vbus(1);
   
   if(balance_this->A66_capacity >= 90){
            
         //set_microp_vbus(0);
         gpCharger->EnableCharging(gpCharger,false);
         balance_this->fsm->onChargingStop(balance_this->fsm,POWERBANK_STOP);

         IsPowerBankCharge = 0;
         printk("[BAT][Bal]mode:%d,StopChg,Vbus:%d\n"
                                        ,IsBalanceMode,get_microp_vbus());
   }else if(balance_this->A66_capacity <= 70){   
         
         //set_microp_vbus(1);
         gpCharger->EnableCharging(gpCharger,true);
         balance_this->fsm->onChargingStart(balance_this->fsm);

         IsPowerBankCharge = 1;
         printk("[BAT][Bal]mode:%d,StartChg,Vbus:%d\n"
                                        ,IsBalanceMode,get_microp_vbus());
   }else{
         printk("[BAT][Bal]mode:%d,sameChg,Vbus:%d\n"
                                        ,IsBalanceMode,get_microp_vbus());
   }  
   printk("[BAT][Bal]:DoPowerBank---\n");
   
}

//Eason: A68 new balance mode +++	
static bool DecideIfPadDockHaveExtChgAC(void);
//Eason: A68 new balance mode ---

//Eason: dynamic set Pad alarm +++

static void judgeIfneedDoBalanceModeWhenSuspend(void)
{

	if( true==IsKeepChgFrom15pTo19p )
	{
		 IsBalanceSuspendStartcharge = true;	
	}else if( (balance_this->A66_capacity>=85)||(balance_this->A66_capacity*10-balance_this->Pad_capacity*12 >=0) )
	{
		 IsBalanceSuspendStartcharge = false;

	}else if((balance_this->A66_capacity<=70)&&(balance_this->A66_capacity*10-balance_this->Pad_capacity*9 <=0) ){

 		 IsBalanceSuspendStartcharge = true;		 
	}
}

static void SetRTCAlarm(void);
//Eason: dynamic set Pad alarm ---

static void BatteryServiceDoBalance(struct AXC_BatteryService *_this)
{
   int StartRatio;
   int StopRatio;

   printk("[BAT][Bal]:DoBalance +++\n");
   //gpCharger->EnableCharging(gpCharger,true);
   StartRatio = GetBalanceModeStartRatio();
   StopRatio = GetBalanceModeStopRatio();

   printk("[BAT][Bal]%d,%d,%d,%d,%d\n",
                      IsBalanceMode,StartRatio,StopRatio,
                      _this->A66_capacity,_this->Pad_capacity);

   if(1 == IsBalanceMode){

         LastTimeIsBalMode = 1;

	//Eason: A68 new balance mode +++	


		//Eason:balance mode keep charge from Cap 15 to 19+++
		if( (_this->A66_capacity>=20)||(false==IsKeepChgFrom15pTo19p) )//16~19 first dobalance will default set_microp_vbus(0) by false==IsKeepChgFrom15pTo19p
		{
			//when forceresume default turn off vbus +++
			if ((false==IsSystemdraw)&&(false == DecideIfPadDockHaveExtChgAC()))//can't take off this, cause if plug extChg and interval calculate Cap can't turn off vbus  
			{
					set_microp_vbus(0);//do this cause in suspend will turn on vbus to charge
					printk("[BAT][Bal]turn off vbus default\n");
			}			
			//when forceresume default turn off vbus ---	
			//judge if draw current to system but does not charge battery +++
			if((_this->A66_capacity>=90)||(_this->A66_capacity-_this->Pad_capacity*StopRatio>=0))
			{
					set_microp_vbus(0);
					gpCharger->EnableCharging(gpCharger,false);
					_this->fsm->onChargingStop(_this->fsm,BALANCE_STOP);   
	             
					IsBalanceCharge = 0;
					IsSystemdraw = false;
					printk("[BAT][Bal]mode:%d,N_Vbus N_Chg,Vbus:%d,SysD:%d\n"
									,IsBalanceMode,get_microp_vbus(),IsSystemdraw);
					ASUSEvtlog("[BAT][Bal]draw system[stop]\n");
	          
			}else if((_this->A66_capacity*10 - _this->Pad_capacity*StartRatio <= 0)
					&&(_this->A66_capacity <= 70 ))
			{
					set_microp_vbus(1);
					gpCharger->EnableCharging(gpCharger,false);
					_this->fsm->onChargingStop(_this->fsm,BALANCE_STOP);
             
					IsBalanceCharge = 0;
					IsSystemdraw = true;
					printk("[BAT][Bal]mode:%d,Y_Vbus N_Chg,Vbus:%d,SysD:%d\n"
									,IsBalanceMode,get_microp_vbus(),IsSystemdraw);
					ASUSEvtlog("[BAT][Bal]draw system[Start]\n");
			}
			//judge if draw current to system, but does not charge battery ---
		}
		//Eason:balance mode keep charge from Cap 15 to 19---
		
		//judge if charge to battery +++
		if(_this->A66_capacity>=20)
		{
				gpCharger->EnableCharging(gpCharger,false);
				_this->fsm->onChargingStop(_this->fsm,BALANCE_STOP);

				IsBalanceCharge = 0;
				IsKeepChgFrom15pTo19p = false;//Eason:balance mode keep charge from Cap 15 to 19
				
				printk("[BAT][Bal]mode:%d,F_Vbus N_Chg,Vbus:%d\n"
								,IsBalanceMode,get_microp_vbus());
				ASUSEvtlog("[BAT][Bal]active charge[stop]\n");
				
		}else if(_this->A66_capacity<=15)
		{
				set_microp_vbus(1);
				gpCharger->EnableCharging(gpCharger,true);
				_this->fsm->onChargingStart(_this->fsm);

				IsBalanceCharge = 1;
				IsKeepChgFrom15pTo19p = true;//Eason:balance mode keep charge from Cap 15 to 19
				
				printk("[BAT][Bal]mode:%d,Y_Vbus Y_Chg,Vbus:%d\n"
								,IsBalanceMode,get_microp_vbus());
				ASUSEvtlog("[BAT][Bal]active charge[Start]\n");
		}
		//judge if charge to battery ---
		//Eason: dynamic set Pad alarm +++
		judgeIfneedDoBalanceModeWhenSuspend();
		//Eason: dynamic set Pad alarm ---
	//Eason: A68 new balance mode ---

         
   //}else if(0==IsBalanceMode && 1==LastTimeIsBalMode && 0==IsBalanceCharge){
   }else if(0==IsBalanceMode){
         
         LastTimeIsBalMode = 0;

         Do_PowerBankMode();
         
         
   }
   //Eason: do ForcePowerBankMode+++
   else if(2==IsBalanceMode){
   	
         LastTimeIsBalMode = 0;

         Do_PowerBankMode();
   }	
   //Eason: do ForcePowerBankMode---
	
   pr_debug("[BAT][Bal]LastBal:%d,IsBalChg:%d,IsBankChg:%d\n"
                            ,LastTimeIsBalMode,IsBalanceCharge,IsPowerBankCharge);
   
   printk("[BAT][Bal]:DoBalance ---\n");
   
}

static bool DecideIfPadDockHaveExtChgAC(void)
{
    bool IsPadDockExtChgAC = false;
    int PadChgCable = 0;
    bool DockChgCable = false;
    int IsDockIn = 0;

    PadChgCable = AX_MicroP_get_USBDetectStatus(Batt_P01);
    
#ifdef CONFIG_ASUSEC    
    IsDockIn = AX_MicroP_IsECDockIn();
#endif    
    if(1==IsDockIn)
    {       
            DockChgCable = balance_this->IsDockExtChgIn;
        	if(true==DockChgCable){
        		IsPadDockExtChgAC = true; 
        	}   
    }else{
            if(1==PadChgCable){
        		IsPadDockExtChgAC = true; 
        	}   		
    }
    printk("[BAT][Ser]:DockI:%d,PadAC:%d,DockAC:%d,ExtChg:%d\n"
                                ,IsDockIn,PadChgCable,DockChgCable,IsPadDockExtChgAC);
 
    return IsPadDockExtChgAC;
}  

//Eason: do ForcePowerBankMode+++
void DoForcePowerBankMode(void)
{
	unsigned short off=0xAA;

	uP_nuvoton_write_reg(MICROP_SOFTWARE_OFF,  &off);
	printk("[BAT][Bal]:ForcePowerBankMode\n");
}
//Eason: do ForcePowerBankMode---

static ssize_t balanceChg_read_proc(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	return sprintf(page, "%d\n", IsBalanceMode);
}
static ssize_t balanceChg_write_proc(struct file *filp, const char __user *buff, 
	            unsigned long len, void *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	
	val = (int)simple_strtol(messages, NULL, 10);


	IsBalanceMode = val;

//when takeoff extChg default turn off vbus +++

	  IsSystemdraw=false;
	  IsBalanceSuspendStartcharge = false;
	  IsKeepChgFrom15pTo19p = false;//Eason:balance mode keep charge from Cap 15 to 19s

//when takeoff extChg default turn off vbus ---
  
   
    if(1==AX_MicroP_IsP01Connected()){

		if( false == DecideIfPadDockHaveExtChgAC()){ 
				Init_Microp_Vbus__Chg();
				BatteryServiceDoBalance(balance_this);
		}else{
				Init_Microp_Vbus__Chg();
		}
		//Eason: do ForcePowerBankMode+++
		if(2==IsBalanceMode){
				DoForcePowerBankMode();
		}
		//Eason: do ForcePowerBankMode---
    }
    
    printk("[BAT][Bal]mode:%d\n",val);
	
	return len;
}

void static create_balanceChg_proc_file(void)
{
	struct proc_dir_entry *balanceChg_proc_file = create_proc_entry("driver/balanceChg", 0644, NULL);

	if (balanceChg_proc_file) {
		balanceChg_proc_file->read_proc = balanceChg_read_proc;
		balanceChg_proc_file->write_proc = balanceChg_write_proc;
	}
    else {
		printk("[BAT][Bal]proc file create failed!\n");
    }

	return;
}

static inline time_t  updateNowTime(struct AXC_BatteryService *_this)
{
    struct timespec mtNow;
    
    mtNow = current_kernel_time();    

    return mtNow.tv_sec;
}

//ASUS_BSP  +++ Eason_Chang "add BAT info time"
static void ReportTime(void)
{
	struct timespec ts;
	struct rtc_time tm;
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);

	pr_info ("[BAT][Ser] %d-%02d-%02d %02d:%02d:%02d\n"
		,tm.tm_year + 1900
		,tm.tm_mon + 1
		,tm.tm_mday
		,tm.tm_hour
		,tm.tm_min
		,tm.tm_sec);
}
//ASUS_BSP  --- Eason_Chang "add BAT info time"

void Init_BalanceMode_Flag(void)
{

         Init_Microp_Vbus__Chg();
		 
//Eason: A68 new balance mode +++			 

	 IsBalanceSuspendStartcharge = false;//when plugIn Pad default false, or in doInBalanceModeWhenSuspend will keep last plugIn time status 
	 IsSystemdraw = false;//when plugIn Pad default false,
	 IsKeepChgFrom15pTo19p = false;//Eason:balance mode keep charge from Cap 15 to 19

//Eason: A68 new balance mode ---

         LastTimeIsBalMode = 1;
         IsBalanceCharge = 1;
         IsPowerBankCharge =1;
         balance_this->P02_savedTime = updateNowTime(balance_this);
}
//ASUS_BSP --- Eason_Chang BalanceMode
bool reportDockInitReady(void)
{
    printk("[BAT][Bal]DockReady:%d\n",balance_this->IsDockInitReady);
    return balance_this->IsDockInitReady;
}

void setDockInitNotReady(void)
{
    balance_this->IsDockInitReady = false;
    printk("[BAT][Bal]setDockNotReady:%d\n",balance_this->IsDockInitReady);
}

bool reportDockExtPowerPlug(void)
{
    return balance_this->IsDockExtCableIn;
}

bool DockCapNeedUpdate(void)
{
    time_t nowDockResumeTime;
    time_t nowDockResumeInterval;
    bool needDoDockResume=false;

    nowDockResumeTime = updateNowTime(balance_this);
    nowDockResumeInterval = nowDockResumeTime - balance_this->Dock_savedTime;
    printk("[BAT][Ser]:DockResume()===:%ld,%ld,%ld\n"
            ,nowDockResumeTime,balance_this->Dock_savedTime,nowDockResumeInterval);

    if( true==balance_this->Dock_IsFirstAskCap ){
            needDoDockResume = true;
    }else if( nowDockResumeInterval >= DOCKRESUME_UPDATE_TIME
                            &&false==balance_this->IsCalculateCapOngoing){
                                            
            needDoDockResume = true;
    }

    return needDoDockResume;
}
//ASUS_BSP +++ Eason_Chang BalanceMode
#ifdef CONFIG_EEPROM_NUVOTON
static int batSer_microp_event_handler(
	struct notifier_block *this,
	unsigned long event,
	void *ptr)
{
    unsigned long flags;
	pr_debug( "[BAT][Bal] %s() +++, evt:%lu \n", __FUNCTION__, event);

	switch (event) {
	case P01_ADD:
		printk( "[BAT][Bal]P01_ADD \r\n");
        asus_chg_set_chg_mode(ASUS_CHG_SRC_PAD_BAT);
        balance_this->P02_IsFirstAskCap = true;
        Init_BalanceMode_Flag();
        
        cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker);
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);

	//Eason: dynamic set Pad alarm +++
#if 0	
        schedule_delayed_work(&balance_this->SetRTCWorker, 1*HZ);
#endif
	//Eason: dynamic set Pad alarm ---
	 
        //Init_BalanceMode_Flag();
        //BatteryServiceDoBalance(balance_this);
		break;	
	case P01_REMOVE: // means P01 removed
	    balance_this->IsDockInitReady = false;
        balance_this->IsDockExtCableIn = false;
        g_padMic_On = false;//Eason:thermal limit charging current,cause setChgDrawPadCurrent only do inPad
		printk( "[BAT][Bal]P01_REMOVE \r\n");
        
        spin_lock_irqsave(&bat_alarm_slock, flags);
        alarm_try_to_cancel(&bat_alarm);
        spin_unlock_irqrestore(&bat_alarm_slock, flags);

        asus_chg_set_chg_mode(ASUS_CHG_SRC_PAD_NONE);
		break;
    case P01_AC_USB_IN:
        msleep(800);//Eason ,need time delay to get PAD AC/USB
        asus_bat_update_PadAcOnline();    
        printk( "[BAT][Bal]P01_AC_USB_IN\r\n");

        if(true==DecideIfPadDockHaveExtChgAC()){
                Init_Microp_Vbus__Chg();
        }
		break;
    case P01_AC_USB_OUT:
        asus_bat_update_PadAcOnline();    
        printk( "[BAT][Bal]P01_AC_USB_OUT \r\n");
        schedule_delayed_work(&balance_this->CableOffWorker,1*HZ);//keep 100% 5 min
  //when takeoff extChg default turn off vbus +++

	  IsSystemdraw=false;
	 IsBalanceSuspendStartcharge = false;
	 IsKeepChgFrom15pTo19p = false;//Eason:balance mode keep charge from Cap 15 to 19

  //when takeoff extChg default turn off vbus ---
        BatteryServiceDoBalance(balance_this);
		break;
    case DOCK_EXT_POWER_PLUG_IN:
        balance_this->IsDockExtCableIn = true;
        asus_bat_update_DockAcOnline();
        printk( "[BAT][Bal]DOCK_EXT_PLUG_IN:%d\r\n",balance_this->IsDockExtCableIn);
        break;
    case DOCK_EXT_POWER_PLUG_OUT:
        balance_this->IsDockExtCableIn = false;
        asus_bat_update_DockAcOnline();
        printk( "[BAT][Bal]DOCK_EXT_PLUG_OUT:%d\r\n",balance_this->IsDockExtCableIn);
        break;
    case DOCK_EXT_POWER_PLUG_IN_READY: // means dock charging
        balance_this->IsDockExtChgIn = true;
		printk( "[BAT][Bal]DOCK_EXT_POWER_PLUG_IN:%d\r\n",balance_this->IsDockExtChgIn);
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue,
                               &balance_this->BatEcAcWorker,
                               1 * HZ);
		break;
    case DOCK_EXT_POWER_PLUG_OUT_READY:	// means dock discharging
        balance_this->IsDockExtChgIn = false;
        printk( "[BAT][Bal]DOCK_EXT_POWER_PLUG_OUT_READY:%d \r\n",balance_this->IsDockExtChgIn);
        schedule_delayed_work(&balance_this->CableOffWorker,1*HZ);//keep 100% 5 min
  //when takeoff extChg default turn off vbus +++

	  IsSystemdraw=false;
	 IsBalanceSuspendStartcharge = false;
	 IsKeepChgFrom15pTo19p = false;//Eason:balance mode keep charge from Cap 15 to 19

  //when takeoff extChg default turn off vbus ---
        BatteryServiceDoBalance(balance_this);
		break;	

	case DOCK_PLUG_IN:  
		asus_bat_update_PadAcOnline();
        balance_this->Dock_IsFirstAskCap = true;
        balance_this->Dock_savedTime = updateNowTime(balance_this);
        break;

    case DOCK_INIT_READY:
        printk( "[BAT][Bal]DOCK_INIT_READY+++\n");
        balance_this->IsDockInitReady = true;
        
        if(1==AX_MicroP_get_USBDetectStatus(Batt_Dock))
        {
                balance_this->IsDockExtChgIn = true;
                Init_Microp_Vbus__Chg();
        }
#ifdef CONFIG_ASUSEC
        if(AX_MicroP_IsDockReady() && DockCapNeedUpdate())
        {
                
                cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker);
                queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);

        }    
#endif    
        printk( "[BAT][Bal]DOCK_INIT_READY---:%d,%d,%d,%d \r\n"
            ,balance_this->IsDockInitReady,AX_MicroP_get_USBDetectStatus(Batt_Dock)
            ,balance_this->IsDockExtChgIn,balance_this->Dock_IsFirstAskCap);
        break;

    case DOCK_PLUG_OUT:
        balance_this->IsDockInitReady = false;
        balance_this->IsDockExtChgIn = false;
  //when takeoff extChg default turn off vbus +++

	  IsSystemdraw=false;
	 IsBalanceSuspendStartcharge = false;
	 IsKeepChgFrom15pTo19p = false;//Eason:balance mode keep charge from Cap 15 to 19

  //when takeoff extChg default turn off vbus ---
        BatteryServiceDoBalance(balance_this);
        printk( "[BAT][Bal]DOCK_PLUG_OUT:%d,%d\r\n"
            ,balance_this->IsDockInitReady,balance_this->IsDockExtChgIn);
        break;

	default:
		pr_debug("[BAT][Bal] %s(), not listened evt: %lu \n", __FUNCTION__, event);
		return NOTIFY_DONE;
	}


	pr_debug("[BAT][Bal] %s() ---\n", __FUNCTION__);
	return NOTIFY_DONE;
}
#endif /* CONFIG_EEPROM_NUVOTON */
//ASUS_BSP --- Eason_Chang BalanceMode
//ASUS_BSP +++ Eason_Chang BalanceMode
#ifdef CONFIG_EEPROM_NUVOTON
static struct notifier_block batSer_microp_notifier = {
        .notifier_call = batSer_microp_event_handler,
};
#endif /* CONFIG_EEPROM_NUVOTON */
//ASUS_BSP --- Eason_Chang BalanceMode



static void CheckBatEcAc(struct work_struct *dat)
{
        if(true == DecideIfPadDockHaveExtChgAC()){
                Init_Microp_Vbus__Chg();
        }
}

//Eason: dynamic set Pad alarm +++

static int CalBalanceInterval(void)
{
	int BalanceInterval;
	int StopInterval_Ratio_1p3;
	int StopInterval_90p;
	int StopInterval_20p;

	//f2=f1+(900*100/2100)*x1/3600 
	//P2=p1-25*(x1/3600)    , (900*5V)/(19*0.95)~=25
	//f2/p2<=1.3
	StopInterval_Ratio_1p3 = (balance_this->Pad_capacity*13104 - balance_this->A66_capacity*10080)/211;

	//f2=f1+(900*100/2100)*x1/3600 
	//f2<=90
	StopInterval_90p =  (7560 - (balance_this->A66_capacity*84));

	BalanceInterval=min(StopInterval_Ratio_1p3,StopInterval_90p);

	
	if(balance_this->A66_capacity<20)
	{
			//f2=f1+(900*100/2100)*x1/3600
			StopInterval_20p = (1680 - (balance_this->A66_capacity*84));
			
			BalanceInterval = max(StopInterval_20p,RTCSetIntervalwhenAlarmIntervalLess3min);
			printk("[BAT][Bal]:Phone less 20p:%d\n",BalanceInterval);
	}else if(BalanceInterval<=180)
	{
			BalanceInterval = RTCSetIntervalwhenAlarmIntervalLess3min;
			printk("[BAT][Bal]:interval less 180sec:%d\n",BalanceInterval);
	}else if(BalanceInterval>=3600)
	{
			BalanceInterval = RTCSetIntervalwhenBalSuspendStopChg;
			printk("[BAT][Bal]:interval >1hr :%d\n",BalanceInterval);
	}else{
			printk("[BAT][Bal]:interval :%d\n",BalanceInterval);
	}

	return BalanceInterval;
	
}

static int CalPowerBankInterval(void)
{
	int PowerBankInterval;
	int StopPowerBankInterval_90p;

	StopPowerBankInterval_90p =  (7560 - (balance_this->A66_capacity*84));

	PowerBankInterval = StopPowerBankInterval_90p;

	if(0==IsPowerBankCharge)//PowerBank Mode Stop condition
	{
			PowerBankInterval = RTCSetIntervalwhenBalSuspendStopChg;
			printk("[BAT][Bal][PwrB]:stop chg interval 1hr:%d\n",PowerBankInterval);
	}
	else if(StopPowerBankInterval_90p <= 180)
	{	
			PowerBankInterval = RTCSetIntervalwhenAlarmIntervalLess3min;	
			printk("[BAT][Bal][PwrB]:interval less 180sec:%d\n",PowerBankInterval);
	}else if(PowerBankInterval>=3600)
	{
			PowerBankInterval = RTCSetIntervalwhenBalSuspendStopChg;
			printk("[BAT][Bal][PwrB]:interval >1hr :%d\n",PowerBankInterval);
	}else{
			printk("[BAT][Bal][PwrB]:interval :%d\n",PowerBankInterval);
	}

	return  PowerBankInterval;
}

static void decideBalanceModeInterval(void)
{
	if(1==IsBalanceMode)
	{
		RTCSetIntervalwhenBalanceMode= CalBalanceInterval();
	}else if((0==IsBalanceMode)||(2==IsBalanceMode)){//Eason: do ForcePowerBankMode
		RTCSetIntervalwhenBalanceMode= CalPowerBankInterval();
	}
}

static void DoWhenPadAlarmResume(void)
{	
    printk("[BAT][Ser]:PadAlarmResume()+++\n");

        balance_this->IsResumeUpdate = true;
        balance_this->IsResumeMahUpdate = true;
        balance_this->P02_IsResumeUpdate = true;

        if(delayed_work_pending(&balance_this->BatteryServiceUpdateWorker))
        {
            cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker);
        }    
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);

        if( false == reportRtcReady()){
            queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue,
                                   &balance_this->BatRtcReadyWorker,
                                   RTC_READY_DELAY_TIME * HZ);
        }

        printk("[BAT][Ser]:PadAlarmResume()---\n");
}

//Eason: dynamic set Pad alarm ---

//Eason set alarm +++
static void SetRTCAlarm(void)
{
    int alarm_type = 0;
    uint32_t alarm_type_mask = 1U << alarm_type;
    unsigned long flags;
    struct timespec new_alarm_time;
    struct timespec mtNow;

    mtNow = current_kernel_time(); 
    new_alarm_time.tv_sec = 0;
    new_alarm_time.tv_nsec = 0;

    printk("[BAT][alarm]:%ld.%ld\n",mtNow.tv_sec,mtNow.tv_nsec);

//Eason: dynamic set Pad alarm +++

		if((1==AX_MicroP_IsP01Connected())&&( true == DecideIfPadDockHaveExtChgAC()))
		{
			new_alarm_time.tv_sec = mtNow.tv_sec+RTCSetInterval;
		}else if(( 0==IsBalanceMode)||( 2==IsBalanceMode))//PowerBankMode//Eason: do ForcePowerBankMode
		{			
			decideBalanceModeInterval();
			new_alarm_time.tv_sec = mtNow.tv_sec+RTCSetIntervalwhenBalanceMode;
		}else if( (true==IsBalanceSuspendStartcharge) && ( 1==IsBalanceMode))//BalanceMode need do suspend charge
		{
			decideBalanceModeInterval();
			new_alarm_time.tv_sec = mtNow.tv_sec+RTCSetIntervalwhenBalanceMode;
		}else{//BalanceMode dont need do suspend charge
			new_alarm_time.tv_sec = mtNow.tv_sec+RTCSetIntervalwhenBalSuspendStopChg;
		}

//Eason: dynamic set Pad alarm ---
    
    printk("[BAT][alarm]:%ld,A66:%d\n",new_alarm_time.tv_sec,balance_this->A66_capacity);
    ReportTime();
    spin_lock_irqsave(&bat_alarm_slock, flags);
    alarm_enabled |= alarm_type_mask;
    alarm_start_range(&bat_alarm,
    timespec_to_ktime(new_alarm_time),
    timespec_to_ktime(new_alarm_time));
    spin_unlock_irqrestore(&bat_alarm_slock, flags);

}

static void alarm_handler(struct alarm *alarm)
{
	unsigned long flags;

	printk("[BAT]battery alarm triggered\n");
	spin_lock_irqsave(&bat_alarm_slock, flags);

	wake_lock_timeout(&bat_alarm_wake_lock, 3 * HZ);
	wake_up(&bat_alarm_wait_queue);

	spin_unlock_irqrestore(&bat_alarm_slock, flags);
//Eason: dynamic set Pad alarm +++

	if(true==InSuspendNeedDoPadAlarmHandler)//Pad alarm handler need to do only when display off
	{
		queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
		                               &balance_this->PadAlarmResumeWorker,\
		                               0 * HZ);
	}

//Eason: dynamic set Pad alarm ---
}


static void SetBatLowRTCAlarm(void)
{
    int batLowAlarm_type = 0;
    uint32_t batLowAlarm_type_mask = 1U << batLowAlarm_type;
    unsigned long batlowflags;
    struct timespec new_batLowAlarm_time;
    struct timespec mtNow;

    mtNow = current_kernel_time(); 
    new_batLowAlarm_time.tv_sec = 0;
    new_batLowAlarm_time.tv_nsec = 0;

    printk("[BAT][alarm][BatLow]:%ld.%ld\n",mtNow.tv_sec,mtNow.tv_nsec);


    new_batLowAlarm_time.tv_sec = mtNow.tv_sec+RTCSetIntervalwhenBATlow;

    
    printk("[BAT][alarm][BatLow]:%ld,A66:%d\n",new_batLowAlarm_time.tv_sec
                                ,balance_this->BatteryService_IsBatLow);
    ReportTime();
    spin_lock_irqsave(&batLow_alarm_slock, batlowflags);
    batLowAlarm_enabled |= batLowAlarm_type_mask;
    alarm_start_range(&batLow_alarm,
    timespec_to_ktime(new_batLowAlarm_time),
    timespec_to_ktime(new_batLowAlarm_time));
    spin_unlock_irqrestore(&batLow_alarm_slock, batlowflags);

} 

static void batLowAlarm_handler(struct alarm *alarm)
{
	unsigned long batlowflags;

	printk("[BAT][alarm]batLow alarm triggered\n");
	spin_lock_irqsave(&batLow_alarm_slock, batlowflags);

	wake_lock_timeout(&batLow_alarm_wake_lock, 3 * HZ);
	wake_up(&batLow_alarm_wait_queue);

	spin_unlock_irqrestore(&batLow_alarm_slock, batlowflags);
    SetBatLowRTCAlarm();
}

static void SetCableInRTCAlarm(void)
{
    int cableInAlarm_type = 0;
    uint32_t cableInAlarm_type_mask = 1U << cableInAlarm_type;
    unsigned long cableInflags;
    struct timespec new_cableInAlarm_time;
    struct timespec mtNow;

    mtNow = current_kernel_time(); 
    new_cableInAlarm_time.tv_sec = 0;
    new_cableInAlarm_time.tv_nsec = 0;

    printk("[BAT][alarm][cableIn]:%ld.%ld\n",mtNow.tv_sec,mtNow.tv_nsec);


    new_cableInAlarm_time.tv_sec = mtNow.tv_sec+RTCSetIntervalwhenCABLEIn;

    
    printk("[BAT][alarm][cableIn]:%ld,A66:%d\n",new_cableInAlarm_time.tv_sec
                                ,balance_this->BatteryService_IsCable);
    ReportTime();
    spin_lock_irqsave(&cableIn_alarm_slock, cableInflags);
    cableInAlarm_enabled |= cableInAlarm_type_mask;
    alarm_start_range(&cableIn_alarm,
    timespec_to_ktime(new_cableInAlarm_time),
    timespec_to_ktime(new_cableInAlarm_time));
    spin_unlock_irqrestore(&cableIn_alarm_slock, cableInflags);

} 

static void cableInAlarm_handler(struct alarm *alarm)
{
	unsigned long cableInflags;

	printk("[BAT][alarm]cableIn alarm triggered\n");
	spin_lock_irqsave(&cableIn_alarm_slock, cableInflags);

	wake_lock_timeout(&cableIn_alarm_wake_lock, 3 * HZ);
	wake_up(&cableIn_alarm_wait_queue);

	spin_unlock_irqrestore(&cableIn_alarm_slock, cableInflags);
    SetCableInRTCAlarm();
}
//Eason set alarm ---

static void CheckBatRtcReady(struct work_struct *dat)
{
       AXC_BatteryService *_this = container_of(dat,AXC_BatteryService,\
                                                BatRtcReadyWorker.work);
       
       if( true == reportRtcReady())
       {
            _this->savedTime=updateNowTime(_this);
		//Eason: when change MaxMah clear interval+++
		_this->ForceSavedTime = updateNowTime(_this);
		//Eason: when change MaxMah clear interval---
            
            if(1==AX_MicroP_IsP01Connected())
            {
                    _this->P02_savedTime = updateNowTime(_this);
            } 
            if (true==reportDockInitReady()){
#ifdef CONFIG_ASUSEC
                	if (AX_MicroP_IsDockReady()){
            		_this->Dock_savedTime = updateNowTime(_this);
            	}
#endif
            }
            printk("[BAT][Ser]sys time ready\n");
       }else{
            queue_delayed_work(_this->BatteryServiceCapUpdateQueue,
                                   &_this->BatRtcReadyWorker,
                                   5 * HZ);
       }       
} 

static void BatteryServiceCapSample(struct work_struct *dat)
{
       AXC_BatteryService *_this = container_of(dat,AXC_BatteryService,\
                                                BatteryServiceUpdateWorker.work);
       wake_lock(&_this->cap_wake_lock);

       _this->IsCalculateCapOngoing = true;
       
        if(true==reportDockInitReady()){  
#ifdef CONFIG_ASUSEC
               if (AX_MicroP_IsDockReady())
               {
                   _this->Dockgauge->askCapacity(_this->Dockgauge);
               }else if(1==AX_MicroP_IsP01Connected()){
                    printk("[BAT][Ser]: Dock bat error,Cap can't update,report P01\n");
                    _this->P02gauge->askCapacity(_this->P02gauge);
               }else{
                    printk("[BAT][Ser]: Dock bat error,Cap can't update,report A66\n");
                    _this->gauge->askCapacity(_this->gauge);
               }
#endif
        }else if(1==AX_MicroP_IsP01Connected())
        {
               _this->P02gauge->askCapacity(_this->P02gauge);
        }else{
               _this->gauge->askCapacity(_this->gauge);
        }        
      
}

static time_t Dock_BatteryService_getIntervalSinceLastUpdate(AXC_BatteryService  *_this)
{
    struct timespec mtNow;
    
    time_t Dock_intervalSinceLastUpdate;
    
    mtNow = current_kernel_time();

    if(_this->test.ifFixedFilterLastUpdateInterval(&_this->test)){
        
        Dock_intervalSinceLastUpdate = _this->test.filterLastUpdateInterval;
        
    }else if( true == _this->Dock_IsFirstAskCap){
    
        Dock_intervalSinceLastUpdate = 0;
          
    }else{

        if(mtNow.tv_sec >= _this->Dock_savedTime){

            pr_debug("[BAT][Ser][Dock]%s:%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,_this->Dock_savedTime);
            
            Dock_intervalSinceLastUpdate = mtNow.tv_sec - _this->Dock_savedTime;

            //cause system time didn't work at first time update capacity (8secs) 
            //filter intervalSinceLastUpdate more than one month
            if(Dock_intervalSinceLastUpdate > 2592000){
                printk("[BAT][Ser][Dock]wrongInt %ld \n",Dock_intervalSinceLastUpdate);
                Dock_intervalSinceLastUpdate = 180;
            }    
         
        }else{
        
            printk("[BAT][Ser]%s:OVERFLOW....%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,_this->Dock_savedTime);              
            //todo: to do the correct calculation here....
            Dock_intervalSinceLastUpdate = mtNow.tv_sec;
        }
    }

    return Dock_intervalSinceLastUpdate ; 
}


static time_t P02_BatteryService_getIntervalSinceLastUpdate(AXC_BatteryService  *_this)
{
    struct timespec mtNow;
    
    time_t P02_intervalSinceLastUpdate;
    
    mtNow = current_kernel_time();

    if(_this->test.ifFixedFilterLastUpdateInterval(&_this->test)){
        
        P02_intervalSinceLastUpdate = _this->test.filterLastUpdateInterval;
        
    }else if( true == _this->P02_IsFirstAskCap){
    
        P02_intervalSinceLastUpdate = 0;
          
    }else{

        if(mtNow.tv_sec >= _this->P02_savedTime){

            pr_debug("[BAT][Ser][P02]%s:%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,_this->P02_savedTime);
            
            P02_intervalSinceLastUpdate = mtNow.tv_sec - _this->P02_savedTime;

            //cause system time didn't work at first time update capacity (8secs) 
            //filter intervalSinceLastUpdate more than one month
            if(P02_intervalSinceLastUpdate > 2592000){
                printk("[BAT][Ser]wrongInt %ld \n",P02_intervalSinceLastUpdate);
                P02_intervalSinceLastUpdate = 180;
            }    
         
        }else{
        
            printk("[BAT][Ser]%s:OVERFLOW....%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,_this->P02_savedTime);              
            //todo: to do the correct calculation here....
            P02_intervalSinceLastUpdate = mtNow.tv_sec;
        }
    }

    return P02_intervalSinceLastUpdate ; 
}

static time_t BatteryService_getIntervalSinceLastUpdate(AXC_BatteryService  *_this)
{
    struct timespec mtNow;
    
    time_t intervalSinceLastUpdate;
    
    mtNow = current_kernel_time();

    if(_this->test.ifFixedFilterLastUpdateInterval(&_this->test)){
        
        intervalSinceLastUpdate = _this->test.filterLastUpdateInterval;
        
    }else if( true == _this->IsFirstAskCap){
    
        intervalSinceLastUpdate = 0;
          
    }else{

        if(mtNow.tv_sec >= _this->savedTime){

            pr_debug("[BAT][Ser]%s:%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,_this->savedTime);
            
            intervalSinceLastUpdate = mtNow.tv_sec - _this->savedTime;

            //cause system time didn't work at first time update capacity (8secs) 
            //filter intervalSinceLastUpdate more than one month
            if(intervalSinceLastUpdate > 2592000){
                printk("[BAT][Ser]wrongInt %ld \n",intervalSinceLastUpdate);
                intervalSinceLastUpdate = 180;
            }    
         
        }else{
        
            printk("[BAT][Ser]%s:OVERFLOW....%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,_this->savedTime);              
            //todo: to do the correct calculation here....
            intervalSinceLastUpdate = 180;
        }
    }

    return intervalSinceLastUpdate ; 
}

static int BatteryService_ChooseMaxMah(AXC_BatteryService  *_this, bool MahDrop)
{   
    //Eason : In suspend have same cap don't update savedTime +++
    SameCapDontUpdateSavedTime = false;
    //Eason : In suspend have same cap don't update savedTime ---

    //Eason : if last time is 10mA +++
    if ( (NO_CHARGER_TYPE==_this->chargerType)||((NOTDEFINE_TYPE==_this->chargerType)) )
    {
    		//printk("dont change IsLastTimeMah10mA\n");
    }else{
	    	//Eason: when change MaxMah clear interval+++
	    	if(true == IsLastTimeMah10mA)
    		{
    			IfUpdateSavedTime = true;
    		}
	      IsLastTimeMah10mA = false;
		//Eason: when change MaxMah clear interval---
    }
    //Eason : if last time is 10mA ---

    switch(_this->chargerType){
        case NO_CHARGER_TYPE:
             if(false == _this->HasCableBeforeSuspend && true==_this->IsResumeMahUpdate){
                _this->IsResumeMahUpdate = false;
                //Eason : In suspend have same cap don't update savedTime +++
                SameCapDontUpdateSavedTime = true;
                //Eason : In suspend have same cap don't update savedTime ---
				//Eason : if last time is 10mA +++
				IsLastTimeMah10mA = true;
				//Eason : if last time is 10mA ---
                return SUSPEND_DISCHG_CURRENT;
             }else{
				//Eason : if last time is 10mA +++
				if(true == IsLastTimeMah10mA)
				{
					IfUpdateSavedTime = true;
				}
				IsLastTimeMah10mA = false;
				//Eason : if last time is 10mA ---
                return MAX_DISCHG_CURRENT;
             }   
         case ILLEGAL_CHARGER_TYPE:
             if(false == MahDrop){
                return USB_CHG_CURRENT;
             }else{
                return MAX_DISCHG_CURRENT-USB_CHG_CURRENT;
             }
        case LOW_CURRENT_CHARGER_TYPE:
             if(false == MahDrop){
                return USB_CHG_CURRENT;
             }else{
                return MAX_DISCHG_CURRENT-USB_CHG_CURRENT;
             }
        case NORMAL_CURRENT_CHARGER_TYPE:
             if(false == MahDrop){
                return PAD_CHG_CURRENT;
             }else{
                return MAX_DISCHG_CURRENT-PAD_CHG_CURRENT;
             }   
        case HIGH_CURRENT_CHARGER_TYPE:
             return AC_CHG_CURRENT;
        default:
             printk("[BAT][Ser]:%s():NO mapping\n",__FUNCTION__);
             if(true==_this->IsResumeMahUpdate){
                _this->IsResumeMahUpdate = false;
                //Eason : In suspend have same cap don't update savedTime +++
                SameCapDontUpdateSavedTime = true;
                //Eason : In suspend have same cap don't update savedTime ---
				//Eason : if last time is 10mA +++
				IsLastTimeMah10mA = true;
				//Eason : if last time is 10mA ---
                return SUSPEND_DISCHG_CURRENT;
             }else{
				//Eason : if last time is 10mA +++
				if(true==IsLastTimeMah10mA)
				{
					IfUpdateSavedTime = true;
				}
				IsLastTimeMah10mA = false;
				//Eason : if last time is 10mA ---
                return MAX_DISCHG_CURRENT;
             }   
        }     
}

static void CheckEoc(struct work_struct *dat)
{
    AXC_BatteryService *_this = container_of(dat,AXC_BatteryService,\
                                             BatEocWorker.work);

    static int count = 0;
    
    if(NO_CHARGER_TYPE >= gpCharger->GetChargerStatus(gpCharger) ||
        !gpCharger->IsCharging(gpCharger)){//if no charger && not being charging

        count = 0;

        return;

    }

    if(count < 3){

        int nCurrent =  _this->callback->getIBAT(_this->callback);

        if(0 >= nCurrent &&
            -90 < nCurrent &&
            _this->A66_capacity > 95){

            count ++;

            if(!delayed_work_pending(&_this->BatEocWorker)){

                    queue_delayed_work(_this->BatteryServiceCapUpdateQueue, \
                                           &_this->BatEocWorker,\
                                           10 * HZ);
            }

        }else{

            count = 0;

            return;

        }
    }  

    printk("[BAT][Ser]%s:chg done\n",__FUNCTION__);

    _this->isMainBatteryChargingDone = true;

    return;


}

static void ResumeCalCap(struct work_struct *dat)
{
    time_t nowResumeTime;
    time_t nowResumeInterval;
    bool needDoResume=false;

    nowResumeTime = updateNowTime(balance_this);
    nowResumeInterval = nowResumeTime - balance_this->savedTime;

    if(true == balance_this->BatteryService_IsBatLow 
        && nowResumeInterval > RESUME_UPDATE_TIMEwhenBATlow)
    {
           needDoResume = true; 
    }    
    else if(balance_this->A66_capacity <= CapChangeRTCInterval 
        && nowResumeInterval > RESUME_UPDATE_TIMEwhenCapLess20)
    {
           needDoResume = true;                
    }else if(nowResumeInterval > RESUME_UPDATE_TIME){
           needDoResume = true;
    }
    printk("[BAT][Ser]:ResumeCalCap()===:%ld,%ld,%ld,A66:%d\n"
            ,nowResumeTime,balance_this->savedTime,nowResumeInterval,balance_this->A66_capacity);

    ReportTime();

//Eason resume always calculate capacity no matter if in   Pad or CableIn or BatLow+++
   if(true==needDoResume)
   {
   	  //Eason set these flag when true==needDoResume+++
         balance_this->IsResumeUpdate = true;
         balance_this->IsResumeMahUpdate = true;
         balance_this->P02_IsResumeUpdate = true;
	  //Eason set these flag when true==needDoResume---	
	
        if(delayed_work_pending(&balance_this->BatteryServiceUpdateWorker))
        {
            cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker); 
        }    
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);
        printk("[BAT][Ser]:resume queue\n");
    }
//Eason resume always calculate capacity no matter if in   Pad or CableIn or BatLow---		
//Eason resume always calculate capacity no matter if in   Pad or CableIn or BatLow+++
#if 0		

    if(1==AX_MicroP_IsP01Connected()&&true==needDoResume)
    {
        if(delayed_work_pending(&balance_this->BatteryServiceUpdateWorker))
        {
            cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker); 
        }    
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);
        printk("[BAT][Ser]:resume queue\n");
    }
    else 
      
        if(true==balance_this->BatteryService_IsBatLow && true==needDoResume)
    {
        if(delayed_work_pending(&balance_this->BatteryServiceUpdateWorker))
        {
            cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker); 
        }    
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);
        printk("[BAT][Ser]:bat Low resume queue\n");
    }
    else if(true==balance_this->BatteryService_IsCable && true==needDoResume)
    {
        if(delayed_work_pending(&balance_this->BatteryServiceUpdateWorker))
        {
            cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker); 
        }    
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);
        printk("[BAT][Ser]:cable in resume queue\n");        
    }
#endif
//Eason resume always calculate capacity no matter if in   Pad or CableIn or BatLow---	
}

static void CableOffKeep5Min(struct work_struct *dat)
{
    time_t nowCableOffTime;
    time_t nowCableOffInterval;
    bool needDoCableOffKeep5Min=false;

    nowCableOffTime = updateNowTime(balance_this);
    nowCableOffInterval = nowCableOffTime - balance_this->savedTime;
    if(nowCableOffInterval <  KEEP_CAPACITY_TIME){
           needDoCableOffKeep5Min = true;
    }
    printk("[BAT][Ser]:CableOffKeep5()===:%ld,%ld,%ld\n"
            ,nowCableOffTime,balance_this->savedTime,nowCableOffInterval);
    ReportTime();

    if(true==needDoCableOffKeep5Min)
    {
        if(delayed_work_pending(&balance_this->BatteryServiceUpdateWorker))
        {
            cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker); 
        }    
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               KEEP_CAPACITY_TIME * HZ);
        
        balance_this->savedTime = updateNowTime(balance_this);    
        if(1==AX_MicroP_IsP01Connected())
        {
                balance_this->P02_savedTime = updateNowTime(balance_this);
        } 
        if (true==reportDockInitReady()){
#ifdef CONFIG_ASUSEC
            	if (AX_MicroP_IsDockReady()){
        		balance_this->Dock_savedTime = updateNowTime(balance_this);
        	}
#endif
        }
        printk("[BAT][Ser]:CableOffKeep5():savedtime:%ld,%ld,%ld\n"
            , balance_this->savedTime,balance_this->P02_savedTime,balance_this->Dock_savedTime);
    }    
}

//Eason: dynamic set Pad alarm +++		
#if 0
static void PlugIntoP02SetRTC(struct work_struct *dat)
{
        SetRTCAlarm();
}
#endif
static void PadRTCAlarmResume(struct work_struct *dat)
{
        DoWhenPadAlarmResume();
}

//Eason: dynamic set Pad alarm ---

static void BatLowTriggeredSetRTC(struct work_struct *dat)
{
        SetBatLowRTCAlarm();
}
//Eason cable in set alarm +++
static void CableInTriggeredSetRTC(struct work_struct *dat)
{
        SetCableInRTCAlarm();
}
//Eason cable in set alarm ---

static void BatteryService_InitWoker(AXC_BatteryService *_this)
{
        _this->BatteryServiceCapUpdateQueue \
            = create_singlethread_workqueue("BatteryServiceCapUpdateQueue");
        INIT_DELAYED_WORK(&_this->BatteryServiceUpdateWorker,\
                            BatteryServiceCapSample);

        INIT_DELAYED_WORK(&_this->BatRtcReadyWorker,\
                            CheckBatRtcReady);

        INIT_DELAYED_WORK(&_this->BatEcAcWorker,
                            CheckBatEcAc);

        INIT_DELAYED_WORK(&_this->BatEocWorker,
                            CheckEoc);

        INIT_DELAYED_WORK(&_this->ResumeWorker,
                            ResumeCalCap);

        INIT_DELAYED_WORK(&_this->CableOffWorker,
                            CableOffKeep5Min);
//Eason: dynamic set Pad alarm +++		
#if 0	
        INIT_DELAYED_WORK(&_this->SetRTCWorker,
                            PlugIntoP02SetRTC);
#endif  
         INIT_DELAYED_WORK(&_this->PadAlarmResumeWorker,
                            PadRTCAlarmResume);

//Eason: dynamic set Pad alarm ---

        INIT_DELAYED_WORK(&_this->SetBatLowRTCWorker,
                            BatLowTriggeredSetRTC);

        INIT_DELAYED_WORK(&_this->SetCableInRTCWorker,
                            CableInTriggeredSetRTC);        

}

static AXE_BAT_CHARGING_STATUS  AXC_BatteryService_getChargingStatus(struct AXI_BatteryServiceFacade * bat)
{
    AXE_BAT_CHARGING_STATUS status = BAT_DISCHARGING_STATUS; 

    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

    switch(_this->fsmState){ 
    case DISCHARGING_STATE:
         status = BAT_DISCHARGING_STATUS;
         break;
    case CHARGING_STATE:
         status = BAT_CHARGING_STATUS;
         break;
    case CHARGING_STOP_STATE:
         //ASUS_BSP +++ Eason_Chang BalanceMode
         if(1==AX_MicroP_IsP01Connected()){       
                status = BAT_NOT_CHARGING_STATUS;  
         }else{              
         //ASUS_BSP --- Eason_Chang BalanceMode
         status = BAT_CHARGING_STATUS;
         }
         break;
    case CHARGING_FULL_STATE:
         status = BAT_CHARGING_FULL_STATUS;
         break;
    case CHARGING_FULL_KEEP_STATE:
         status = BAT_CHARGING_FULL_STATUS;
         break;
    case CHARGING_FULL_KEEP_STOP_STATE:
         status = BAT_CHARGING_FULL_STATUS;
         break;
    default:
         printk("[BAT][Ser]%s():status error\n",__FUNCTION__);
    }
    return status;
}

static int  AXC_BatteryService_getCapacity(struct AXI_BatteryServiceFacade * bat)
{
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

    return _this-> A66_capacity;
}
static void AXC_BatteryService_onCableInOut(struct AXI_BatteryServiceFacade *bat, AXE_Charger_Type type)
{
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

    //Eason cable in set alarm +++
    unsigned long cableInFlags;
    //Eason cable in set alarm ---

printk("[BAT][Ser]:onCableInOut()+++\n");
    _this->chargerType = type ;
    _this->fsm->onCableInOut(_this->fsm,type);
    if ( 100 == _this->A66_capacity){
          _this->fsm->onChargingStop(_this->fsm,CHARGING_DONE);   
    }    

    switch(type){
        case NO_CHARGER_TYPE:
             _this->gauge->notifyCableInOut(_this->gauge,false);
             _this->BatteryService_IsCable = false ;
		//Eason :when  low bat Cap draw large current  +++	 
		if(10 <= _this->A66_capacity )
		{
             schedule_delayed_work(&_this->CableOffWorker,1*HZ);//keep 100% 5 min
		}
		 //Eason :when  low bat Cap draw large current  ---
             //Eason cable in set alarm +++
             spin_lock_irqsave(&cableIn_alarm_slock, cableInFlags);
             alarm_try_to_cancel(&cableIn_alarm);
             spin_unlock_irqrestore(&cableIn_alarm_slock, cableInFlags);
             //Eason cable in set alarm --- 
             //_this->BatteryService_IsFULL = false;
             //_this->gauge->notifyBatFullChanged(_this->gauge,false);
             break;
        case ILLEGAL_CHARGER_TYPE:
        case LOW_CURRENT_CHARGER_TYPE:
        case NORMAL_CURRENT_CHARGER_TYPE:
        case HIGH_CURRENT_CHARGER_TYPE:
             _this->gauge->notifyCableInOut(_this->gauge,true);
             _this->BatteryService_IsCable = true ;
	       //Eason: dynamic set Pad alarm, Pad has its own alrarm+++
		 if( NORMAL_CURRENT_CHARGER_TYPE!=_this->chargerType)
	 	{
	             //Eason cable in set alarm +++
	             schedule_delayed_work(&_this->SetCableInRTCWorker, 0*HZ);
	             //Eason cable in set alarm ---
	 	}
		//Eason: dynamic set Pad alarm, Pad has its own alrarm---

             break;
        default:
             printk("[BAT][Ser]:%s():NO mapping\n",__FUNCTION__);
             break;
    } 
printk("[BAT][Ser]:onCableInOut():%d---\n",type);
}
static void AXC_BatteryService_onChargingStop(struct AXI_BatteryServiceFacade *bat,AXE_Charging_Error_Reason reason)
{/*
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

    _this->fsm->onChargingStop(_this->fsm,reason);
    _this->BatteryService_IsCharging = false ;

    if (CHARGING_DONE == reason){
        
        _this->BatteryService_IsFULL = true;
        
        _this->gauge->notifyBatFullChanged(_this->gauge,true);

        wake_lock(&_this->cap_wake_lock);
        _this->gauge->askCapacity(_this->gauge);
    }
 */   
}
static void AXC_BatteryService_onChargingStart(struct AXI_BatteryServiceFacade *bat)
{/*
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

    _this->fsm->onChargingStart(_this->fsm);
    _this->BatteryService_IsCharging = true ;

 */
}
static void AXC_BatteryService_onBatteryLowAlarm(struct AXI_BatteryServiceFacade *bat, bool isCurrentBattlow)
{
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);
    
    unsigned long batLowFlags;

    if(false==_this->BatteryService_IsBatLow && true==isCurrentBattlow)
    {
            schedule_delayed_work(&balance_this->SetBatLowRTCWorker, 0*HZ);
    }
    else if(true==_this->BatteryService_IsBatLow && false==isCurrentBattlow)
    {
            spin_lock_irqsave(&batLow_alarm_slock, batLowFlags);
            alarm_try_to_cancel(&batLow_alarm);
            spin_unlock_irqrestore(&batLow_alarm_slock, batLowFlags);
    }
    _this->BatteryService_IsBatLow = isCurrentBattlow ;


}   
static void AXC_BatteryService_onBatteryRemoved(struct AXI_BatteryServiceFacade * bat, bool isRemoved)
{
   // AXC_BatteryService  *_this=
    //    container_of(bat, AXC_BatteryService, miParent);

}
static void Record_P02BeforeSuspend(void)
{
	if(1==AX_MicroP_get_USBDetectStatus(Batt_P01))
    {
		balance_this->P02_IsCable = true;
	}else{
		balance_this->P02_IsCable = false;
	}

    //balance_this->P02_ChgStatusBeforeSuspend = AX_MicroP_get_ChargingStatus(Batt_P01);
}
static int P02_ChooseMaxMahBeforeSuspend(void)
{
    if(true == DecideIfPadDockHaveExtChgAC())
    {
          return AC_CHG_CURRENT;
#ifdef CONFIG_ASUSEC
    }else if(1==AX_MicroP_IsECDockIn()){
          return USB_CHG_CURRENT;
#endif          
    }else{
	  return SUSPEND_DISCHG_CURRENT;
    }			
}

static int P02_ChooseMaxMah(void)
{
   if(true == DecideIfPadDockHaveExtChgAC()){
          return AC_CHG_CURRENT;
#ifdef CONFIG_ASUSEC          
   }else if(1==AX_MicroP_IsECDockIn()){
          return USB_CHG_CURRENT;
#endif          
   }else{
	  return PAD_CHG_CURRENT;
   }			
}

static void Record_DockBeforeSuspend(void)
{
	if(1==AX_MicroP_get_USBDetectStatus(Batt_Dock))
    {
		balance_this->Dock_IsCable = true;
	}else{
		balance_this->Dock_IsCable = false;
	}

    //balance_this->Dock_ChgStatusBeforeSuspend = AX_MicroP_get_ChargingStatus(Batt_Dock);
}
static int Dock_ChooseMaxMahBeforeSuspend(void)
{
    if(true == DecideIfPadDockHaveExtChgAC())
    {
          return AC_CHG_CURRENT;     
    }else{
	  return DOCK_SUSPEND_DISCHG_CURRENT;
    }			
}
static int Dock_ChooseMaxMah(void)
{
   if(true == DecideIfPadDockHaveExtChgAC()){
          return AC_CHG_CURRENT;
#ifdef CONFIG_ASUSEC          
   }else if(1==AX_MicroP_IsECDockIn()){
          return USB_CHG_CURRENT;
#endif          
   }else{
	  return DOCK_DISCHG_CURRENT;
   }			
}

/*void cancelBatCapQueueBeforeSuspend(void){
    printk("[BAT] cancel Bat Cap Queue\n");
    cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker);
}*/
static void AXC_BatteryService_dockSuspend(struct AXI_BatteryServiceFacade *bat)
{
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

    if (true==reportDockInitReady()){
        	//if (AX_MicroP_IsDockReady()){
            		Record_DockBeforeSuspend();
                    _this->Dock_HasCableBeforeSuspend = _this->Dock_IsCable;
                    _this->Dock_MaxMahBeforeSuspend = Dock_ChooseMaxMahBeforeSuspend();
                    printk("[BAT][Ser][Dock]:suspend:%d,%d\n"
                            ,_this->Dock_IsCable
                            //,_this->Dock_ChgStatusBeforeSuspend
                            ,_this->Dock_MaxMahBeforeSuspend);
        	//}
    }
    
}

//Eason: A68 new balance mode +++

void doInBalanceModeWhenSuspend(void)
{
	//when resume default turn off vbus +++
	IsSystemdraw = false;
	//when resume default turn off vbus ---

	if( false==IsBalanceSuspendStartcharge )
	{

		set_microp_vbus(0);
		//set_5VPWR_EN(0);
		printk("[BAT][Bal]stop Vbus:%d\n"
                                        ,get_microp_vbus());
		 gpCharger->EnableCharging(gpCharger,false);
              balance_this->fsm->onChargingStop(balance_this->fsm,BALANCE_STOP);               
             IsBalanceCharge = 0;
	
	}else if(true==IsBalanceSuspendStartcharge){//remember this flag to do suspendCharge before suspendStopChg condition match
	
		//set_5VPWR_EN(1);
		set_microp_vbus(1);
		printk("[BAT][Bal]start Vbus:%d\n"
                                       ,get_microp_vbus());
		 gpCharger->EnableCharging(gpCharger,true);
              balance_this->fsm->onChargingStart(balance_this->fsm);                 
             IsBalanceCharge = 1;
			 
	}
}

//Eason: A68 new balance mode ---
	
static void AXC_BatteryService_suspend(struct AXI_BatteryServiceFacade *bat)
{
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

    printk("[BAT][Ser]:suspend()+++\n");
//Eason: dynamic set Pad alarm +++
		
	InSuspendNeedDoPadAlarmHandler=true;

//Eason: dynamic set Pad alarm ---	
    
    //if(false == _this->IsSuspend){

        //_this->IsSuspend = true;
        _this->HasCableBeforeSuspend = _this->BatteryService_IsCable;
        Record_P02BeforeSuspend();
        _this->P02_HasCableBeforeSuspend = _this->P02_IsCable;
        _this->P02_MaxMahBeforeSuspend = P02_ChooseMaxMahBeforeSuspend();
        
        //cancel_delayed_work_sync(&_this->BatteryServiceUpdateWorker); 

    //}

//Eason: A68 new balance mode +++

	if ((1==AX_MicroP_IsP01Connected())&&(1 == IsBalanceMode)&&(false == DecideIfPadDockHaveExtChgAC()))
	{
		printk("[BAT][Bal]Phone:%d,Pad:%d\n",_this->A66_capacity,_this->Pad_capacity);
		doInBalanceModeWhenSuspend();
	}

//Eason: A68 new balance mode ---
    printk("[BAT][Ser]:suspend()---\n");

}
//Eason resume always calculate capacity no matter if in   Pad or CableIn or BatLow+++
#if 0
static void AXC_BatteryService_resume(struct AXI_BatteryServiceFacade *bat,int delayStartInSeconds)
{
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

    
    if( (1==AX_MicroP_IsP01Connected())||(true==_this->BatteryService_IsCable) )
    {

        printk("[BAT][Ser]:resume()+++\n");
		
	  ASUSEvtlog("[BAT][Bal]resume:%d\n",IsBalanceSuspendStartcharge);	
	
    //if(true == _this->IsSuspend){

        //_this->IsSuspend = false;

        _this->IsResumeUpdate = true;
        _this->IsResumeMahUpdate = true;
        _this->P02_IsResumeUpdate = true;

        /*if(delayed_work_pending(&_this->BatteryServiceUpdateWorker)){  
        cancel_delayed_work(&_this->BatteryServiceUpdateWorker);
        printk("[BAT][Ser]:resume pending\n");
        }*/
        schedule_delayed_work(&_this->ResumeWorker,1*HZ);
        wake_lock_timeout(&_this->resume_wake_lock,2* HZ);
        
        

        if( false == reportRtcReady()){
            queue_delayed_work(_this->BatteryServiceCapUpdateQueue,
                                   &_this->BatRtcReadyWorker,
                                   RTC_READY_DELAY_TIME * HZ);
        }
    //}

        printk("[BAT][Ser]:resume()---\n");
    }else if(true==balance_this->BatteryService_IsBatLow)
    {
        printk("[BAT][Ser][BatLow]:resume()+++\n");

        _this->IsResumeUpdate = true;
        _this->IsResumeMahUpdate = true;
        _this->P02_IsResumeUpdate = true;

        schedule_delayed_work(&_this->ResumeWorker,1*HZ);
        wake_lock_timeout(&_this->resume_wake_lock,2* HZ);
        
        

        if( false == reportRtcReady()){
            queue_delayed_work(_this->BatteryServiceCapUpdateQueue,
                                   &_this->BatRtcReadyWorker,
                                   RTC_READY_DELAY_TIME * HZ);
        }

        printk("[BAT][Ser][BatLow]:resume()---\n");
    }
}
#endif//end #if 0 
//Eason resume always calculate capacity no matter if in   Pad or CableIn or BatLow---
//Eason resume always calculate capacity no matter if in   Pad or CableIn or BatLow+++
static void AXC_BatteryService_resume(struct AXI_BatteryServiceFacade *bat,int delayStartInSeconds)
{
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

        printk("[BAT][Ser]:resume()+++\n");

	if(1==AX_MicroP_IsP01Connected())
	{
#ifndef ASUS_FACTORY_BUILD		
	  ASUSEvtlog("[BAT][Bal]resume:%d\n",IsBalanceSuspendStartcharge);	
#endif//ASUS_FACTORY_BUILD	
	}
	

        schedule_delayed_work(&_this->ResumeWorker,1*HZ);
        wake_lock_timeout(&_this->resume_wake_lock,2* HZ);
        
        

        if( false == reportRtcReady()){
            queue_delayed_work(_this->BatteryServiceCapUpdateQueue,
                                   &_this->BatRtcReadyWorker,
                                   RTC_READY_DELAY_TIME * HZ);
        }

        printk("[BAT][Ser]:resume()---\n");

}
//Eason resume always calculate capacity no matter if in   Pad or CableIn or BatLow---

static void AXC_BatteryService_forceResume(struct AXI_BatteryServiceFacade *bat,int delayStartInSeconds)
{
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

    time_t nowForceResumeTime;
    time_t nowForceResumeInterval;
    bool needDoForceResume=false;


//when forceresume default turn off vbus +++
		
	IsSystemdraw= false;
	IsBalanceSuspendStartcharge = false;
	//Eason: dynamic set Pad alarm +++
	InSuspendNeedDoPadAlarmHandler=false;
	//Eason: dynamic set Pad alarm ---	

//when forceresume default turn off vbus ---

    nowForceResumeTime = updateNowTime(_this);
    nowForceResumeInterval = nowForceResumeTime - _this->savedTime;
    printk("[BAT][Ser]:forceResume()===:%ld,%ld,%ld\n"
            ,nowForceResumeTime,_this->savedTime,nowForceResumeInterval);

    if( true==_this->IsFirstForceResume ){
            needDoForceResume = true;
            _this->IsFirstForceResume = false;
    }else if( nowForceResumeInterval >= FORCERESUME_UPDATE_TIME
                            &&false==_this->IsCalculateCapOngoing){
                                            
            needDoForceResume = true;
    }/*else{
            printk("[BAT][Ser]:forceResume queue 5min\n");
            queue_delayed_work(_this->BatteryServiceCapUpdateQueue, 
                               &_this->BatteryServiceUpdateWorker,
                               FORCERESUME_UPDATE_TIME * HZ);    
    }*/
    



    if( true==needDoForceResume )
    {
        printk("[BAT][Ser]:forceResume()+++\n");

    //if(true == _this->IsSuspend){

        //_this->IsSuspend = false;

        _this->IsResumeUpdate = true;
        _this->IsResumeMahUpdate = true;
        _this->P02_IsResumeUpdate = true;
        _this->Dock_IsResumeUpdate = true;

        if(delayed_work_pending(&_this->BatteryServiceUpdateWorker))
        {
            cancel_delayed_work_sync(&_this->BatteryServiceUpdateWorker);
        }    
        queue_delayed_work(_this->BatteryServiceCapUpdateQueue, \
                               &_this->BatteryServiceUpdateWorker,\
                               delayStartInSeconds * HZ);
        ReportTime();

        if( false == reportRtcReady()){
            queue_delayed_work(_this->BatteryServiceCapUpdateQueue,
                                   &_this->BatRtcReadyWorker,
                                   RTC_READY_DELAY_TIME * HZ);
        }
    //}

        printk("[BAT][Ser]:forceResume()---\n");
    }
//Eason: A68 new balance mode +++	

    else if((1==AX_MicroP_IsP01Connected())&&(1 == IsBalanceMode)&&(false == DecideIfPadDockHaveExtChgAC()))
    {
    		printk("[BAT][Ser]:less 5 min forceResume()+++\n");
    		BatteryServiceDoBalance(balance_this);
		printk("[BAT][Ser]:less 5 min forceResume()---\n");	
    }	

//Eason: A68 new balance mode ---

}
static void  AXC_BatteryService_constructor(struct AXC_BatteryService *_this,AXI_BatteryServiceFacadeCallback *callback)
{
    _this->callback = callback;

    if(false == _this->mbInit){

        //todo....add internal module creation & init here...
        BatteryService_enable_ChargingFsm(_this);// batteryservice to fsm
        BatteryService_enable_Gauge(_this);// batteryservice to fsm
        BatteryService_enable_Filter(_this);
        BatteryService_InitWoker(_this);
        //ASUS_BSP +++ Eason_Chang BalanceMode
        create_balanceChg_proc_file();
        balance_this = _this;
        #ifdef CONFIG_EEPROM_NUVOTON
	        register_microp_notifier(&batSer_microp_notifier);
        #endif /* CONFIG_EEPROM_NUVOTON */
        //ASUS_BSP --- Eason_Chang BalanceMode
        mutex_init(&_this->main_lock);
        mutex_init(&_this->filter_lock);
        wake_lock_init(&_this->cap_wake_lock, WAKE_LOCK_SUSPEND, "bat_cap");
        wake_lock_init(&_this->resume_wake_lock, WAKE_LOCK_SUSPEND, "resume_wake"); 
        //Eason set alarm +++
        alarm_init(&bat_alarm, 0, alarm_handler);
        alarm_init(&batLow_alarm, 0, batLowAlarm_handler);
        alarm_init(&cableIn_alarm, 0, cableInAlarm_handler);
        wake_lock_init(&bat_alarm_wake_lock, WAKE_LOCK_SUSPEND, "bat_alarm_wake");
        wake_lock_init(&batLow_alarm_wake_lock, WAKE_LOCK_SUSPEND, "batLow_alarm_wake");
        wake_lock_init(&cableIn_alarm_wake_lock, WAKE_LOCK_SUSPEND, "cableIn_alarm_wake");
        //Eason set alarm ---
        //ASUS_BSP  +++ Eason_Chang charger
    	gpCharger = getAsusCharger();
    	gpCharger->Init(gpCharger);
    	gpCharger->RegisterChargerStateChanged(gpCharger, &balance_this->gChargerStateChangeNotifier, _this->chargerType);
        //ASUS_BSP  --- Eason_Chang charger
        _this->mbInit = true;
    }
    
}

//ASUS BSP Eason_Chang +++ batteryservice to fsm
static void BatteryServiceFsm_OnChangeChargingCurrent(struct AXI_Charging_FSM_Callback *callback,AXE_Charger_Type chargertype)
{
    AXC_BatteryService  *_this=
        container_of(callback, AXC_BatteryService, fsmCallback);

        pr_debug("[BAT][Ser]%s()\n",__FUNCTION__);
    
    _this->callback->changeChargingCurrent(_this->callback,chargertype);
}

static void BatteryServiceFsm_OnStateChanged(struct AXI_Charging_FSM_Callback *callback)
{   
    AXE_Charging_State GetStateFromFsm;
    bool NeedUpdate = 0;
    AXC_BatteryService  *_this=
        container_of(callback, AXC_BatteryService, fsmCallback);
     
    GetStateFromFsm = _this->fsm->getState(_this->fsm);
    

    switch(_this->fsmState){
        case DISCHARGING_STATE:
             if(GetStateFromFsm == CHARGING_STATE){
                NeedUpdate = 1;
             }
            break;
        case CHARGING_STATE:
             //ASUS_BSP +++ Eason_Chang BalanceMode
             if(1==AX_MicroP_IsP01Connected()){                   
                  if(    GetStateFromFsm == DISCHARGING_STATE
                       ||GetStateFromFsm == CHARGING_FULL_STATE
                       ||GetStateFromFsm == CHARGING_STOP_STATE){
                       NeedUpdate = 1;  
                  }                
             }else
             //ASUS_BSP --- Eason_Chang BalanceMode    
             if(GetStateFromFsm == DISCHARGING_STATE || GetStateFromFsm == CHARGING_FULL_STATE ){
                NeedUpdate = 1;
             }
            break;
        case CHARGING_STOP_STATE:
            //ASUS_BSP +++ Eason_Chang BalanceMode
            if(1==AX_MicroP_IsP01Connected()){
                 if(     GetStateFromFsm == DISCHARGING_STATE
                       ||GetStateFromFsm == CHARGING_STATE){
                       NeedUpdate = 1;
                }
            }else 
            //ASUS_BSP --- Eason_Chang BalanceMode  
            if(GetStateFromFsm == DISCHARGING_STATE ){
                NeedUpdate = 1;
            }   
            break;
        case CHARGING_FULL_STATE:
            if(GetStateFromFsm == DISCHARGING_STATE ){
                NeedUpdate = 1;
            } 
            break;
        case CHARGING_FULL_KEEP_STATE:
            if(GetStateFromFsm == DISCHARGING_STATE ){
                NeedUpdate = 1;
            }    
            break;
        case CHARGING_FULL_KEEP_STOP_STATE:
            if(GetStateFromFsm == DISCHARGING_STATE ){
                NeedUpdate = 1;
            }
            break;
        default:
            printk("[BAT][ser]%s():NOT mapping\n",__FUNCTION__);
            break;
            
    }

    _this->fsmState = GetStateFromFsm;

    if( 1 == NeedUpdate){
    _this->callback->onServiceStatusUpdated(_this->callback);
    }
    
}

//ASUS BSP Eason_Chang --- batteryservice to fsm
//ASUS BSP Eason_Chang +++ batteryservice to gauge
static inline int AXC_BatteryService_getNextPollingInterval(struct AXC_BatteryService *_this)
{

    if(_this->test.ifFixedPollingInterval(&_this->test)){

        return _this->test.pollingInterval;

    }else{

        return _this->gauge->getNextPollingInterval(_this->gauge);
    }

}

static inline void  AXC_BatteryService_scheduleNextPolling(struct AXC_BatteryService *_this)
{

    queue_delayed_work(_this->BatteryServiceCapUpdateQueue,
                            &_this->BatteryServiceUpdateWorker,
                            AXC_BatteryService_getNextPollingInterval(_this)* HZ);
}

static bool Get_CapRiseWhenCableIn(int nowCap, int lastCap)
{
   //Eason: Cap<10 with cable, but Cap decrease. Let Cap drop, ignore rule of lastCap - nowCap >= 5 +++
   if( (lastCap - nowCap > 0)&&(balance_this->A66_capacity<10) ){
	  printk("[BAT][Ser]:CapDropWhenCableInCapLessThan10\n");
        return false; 
   }else	
   //Eason: Cap<10 with cable, but Cap decrease. Let Cap drop, ignore rule of lastCap - nowCap >= 5 ---
   if(lastCap - nowCap >= 5){
        printk("[BAT][Ser]:CapDropWhenCableIn\n");
        return false; 
   }else{
        return true;
   }
}

static void DoAfterDecideFull(struct AXC_BatteryService *_this)
{
    if(false == _this->BatteryService_IsFULL){
        
        _this->BatteryService_IsFULL = true;
        _this->fsm->onChargingStop(_this->fsm,CHARGING_DONE);
        _this->gauge->notifyBatFullChanged(_this->gauge,true);

    }

}

static void DoAfterDecideNotFull(struct AXC_BatteryService *_this)
{
    if (true==_this->BatteryService_IsFULL){
            _this->gauge->notifyBatFullChanged(_this->gauge,false);
            _this->fsm->onFullDrop(_this->fsm);
    }

     _this->BatteryService_IsFULL = false;
}
static void DecideIsFull(struct AXC_BatteryService *_this,int nowGaugeCap,bool hasCableInPastTime)
{
    bool chgStatus;
    bool IsPadDock_ExtChg = false;
    int nCurrent = _this->callback->getIBAT(_this->callback);

    chgStatus = gpCharger->IsCharging(gpCharger);

    printk("[BAT][Ser]:chgStatus:%d,cur:%d\n",chgStatus,nCurrent);

    if(chgStatus ){

        if(!_this->isMainBatteryChargingDone){// if still charging....

            

            if(nowGaugeCap > 95 &&
                0 >= nCurrent &&
                -90 < nCurrent &&
                !delayed_work_pending(&_this->BatEocWorker)){

                printk("[BAT][Ser]start eoc worker\n");
                
                queue_delayed_work(_this->BatteryServiceCapUpdateQueue, \
                                       &_this->BatEocWorker,\
                                       10 * HZ);
            }
            
        }else{
        
            chgStatus = false;
            
        }
    }else{
            if( 0 >= nCurrent && -90 < nCurrent){
                    chgStatus = false;
            }else if( 100 == balance_this->A66_capacity){
                    chgStatus = false;
                    printk("[BAT][Ser]:when cap100 don't judge current\n");
                    //Eason : when resume by take off cable can be judge full 
            }else{
                    chgStatus = true;
                    printk("[BAT][Ser]:chg current not in 0~-90, can't judge Full\n");
            }
    }

    //Eason for resume by EXTchg off can be full+++
    if(100 == balance_this->A66_capacity){
        IsPadDock_ExtChg = true;
    }else{    
        IsPadDock_ExtChg = DecideIfPadDockHaveExtChgAC();
    }
    //Eason for resume by EXTchg off can be full---

    if(1==AX_MicroP_IsP01Connected())
    {
               if(CHARGING_FULL_STATE==balance_this->fsmState
                	|| CHARGING_FULL_KEEP_STATE==balance_this->fsmState
                    || CHARGING_FULL_KEEP_STOP_STATE==balance_this->fsmState)
               {
                     DoAfterDecideFull(_this);    
               }else if(true==hasCableInPastTime && nowGaugeCap > 95 
                    && false==chgStatus && true==IsPadDock_ExtChg)
               {
                     DoAfterDecideFull(_this);
               }else if((100 == balance_this->A66_capacity)&&(100==nowGaugeCap))
               {
                     DoAfterDecideFull(_this);
               }else{
                     DoAfterDecideNotFull(_this);
               } 
    }else{    
               if(CHARGING_FULL_STATE==balance_this->fsmState
                	|| CHARGING_FULL_KEEP_STATE==balance_this->fsmState
                    || CHARGING_FULL_KEEP_STOP_STATE==balance_this->fsmState)
               {
                     DoAfterDecideFull(_this);
               }else if(true==hasCableInPastTime && nowGaugeCap > 95 
                    && false==chgStatus )
               {
                     DoAfterDecideFull(_this);
               }else if((100 == balance_this->A66_capacity)&&(100==nowGaugeCap))
               {
                     DoAfterDecideFull(_this);                     
               }else{
                     DoAfterDecideNotFull(_this);
               } 
    }           

} 

bool report_BatteryService_If_HasCable(void)
{
    bool hasCable = false;

    if (true == balance_this->IsFirstAskCable)
    {
    	if(1==!gpio_get_value(maxim8934DCIN))
            {
                printk("[BAT][Ser]FirstAskCable report true\n");
                hasCable = true;
            }
    
    }else if (true == balance_this->IsResumeUpdate){
        hasCable = balance_this->HasCableBeforeSuspend;

    }else{
        hasCable = balance_this->BatteryService_IsCable; 
    }

    return hasCable;
}

//Eason: when change MaxMah clear interval+++
static time_t BatteryService_getForceIntervalSinceLastUpdate(AXC_BatteryService  *_this)
{
    struct timespec mtNow;
    
    time_t intervalSinceLastUpdate;
    
    mtNow = current_kernel_time();



    if(mtNow.tv_sec >= _this->ForceSavedTime){

        pr_debug("[BAT][Ser]%s:%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,_this->ForceSavedTime);
            
        intervalSinceLastUpdate = mtNow.tv_sec - _this->ForceSavedTime;

        if(intervalSinceLastUpdate > 2592000){
            printk("[BAT][Ser]wrongInt %ld \n",intervalSinceLastUpdate);
            intervalSinceLastUpdate = 180;
        }    
     
    }else{
        
        printk("[BAT][Ser]%s:OVERFLOW....%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,_this->ForceSavedTime);              
        //todo: to do the correct calculation here....
        intervalSinceLastUpdate = 180;
    }


    return intervalSinceLastUpdate ; 
}
//Eason: when change MaxMah clear interval---

static void AXC_BatteryService_reportPropertyCapacity(struct AXC_BatteryService *_this, int refcapacity)
{
    int A66_capacity;
    
    //int EC_capacity;
    int IsBalTest;//ASUS_BSP Eason_Chang BalanceMode

    int lastCapacity;

    int maxMah;

    bool hasCable = false;
    bool EnableBATLifeRise;
    bool maxMahDrop = false;
    //Eason boot up in BatLow situation, take off cable can shutdown+++
    bool IsBatLowtoFilter ;
    //Eason boot up in BatLow situation, take off cable can shutdown---

    time_t intervalSinceLastUpdate;

    mutex_lock(&_this->filter_lock);

    intervalSinceLastUpdate  = BatteryService_getIntervalSinceLastUpdate(_this);
    

    //We need do ask capcaity to filter at first time, in case there is FULL orBATLow 
    if(true == _this->IsFirstAskCap){

        lastCapacity = refcapacity;

        _this->IsFirstAskCap = false;
        
    }else{

        lastCapacity = _this->A66_capacity;

    }


    if (true == _this->IsFirstAskCable)
    {
        if(1==!gpio_get_value(maxim8934DCIN))
            {
                printk("[BAT][Ser]FirstAskCable true\n");
                hasCable = true;
            }
        _this->IsResumeUpdate = false;
        _this->IsFirstAskCable = false;
        
    }else if (true == _this->IsResumeUpdate){
        hasCable = _this->HasCableBeforeSuspend;
        _this->IsResumeUpdate = false;
 
    }else{
        hasCable = _this->BatteryService_IsCable; 
    }

    _this->BatteryService_IsCharging = gpCharger->IsCharging(gpCharger);

    DecideIsFull(_this,refcapacity,hasCable);
//Eason: BAT Cap can drop when cable in +++ 
    if( true == hasCable )//A66 has cable  
    { 
            if(1==AX_MicroP_IsP01Connected()){//cable of A66 is Pad
                    if( true == _this->BatteryService_IsCharging){//A66 is charging 
                            EnableBATLifeRise = Get_CapRiseWhenCableIn(refcapacity, lastCapacity);
                            if( false == EnableBATLifeRise){
                                    maxMahDrop = true;
                            }
                    }else if(true == _this->BatteryService_IsFULL){//A66 is not charging but full
                            //Eason for resume by EXTchg off can be full, after full can drop
                            EnableBATLifeRise = Get_CapRiseWhenCableIn(refcapacity, lastCapacity);
                            if( false == EnableBATLifeRise){
                                    maxMahDrop = true;
                            }
                    }else if(true == DecideIfPadDockHaveExtChgAC()){//A66 have ext chg 
                            EnableBATLifeRise = Get_CapRiseWhenCableIn(refcapacity, lastCapacity);
                            if( false == EnableBATLifeRise){
                                    maxMahDrop = true;
                            }                    
                    }else{//A66 is neither charging  nor full
                            EnableBATLifeRise = false;
                    }
            }else{//cable of A66 is not Pad

                    EnableBATLifeRise = Get_CapRiseWhenCableIn(refcapacity, lastCapacity);
                    if( false == EnableBATLifeRise){
                            maxMahDrop = true;
                    }
            }      
    }else{//A66 doesn't has cable 
      EnableBATLifeRise = hasCable;
    }
    maxMah = BatteryService_ChooseMaxMah(_this,maxMahDrop);
      //Eason : if last time is 10mA +++
	if(true==IfUpdateSavedTime)//only do when last time is 10mA
	{
		intervalSinceLastUpdate = BatteryService_getForceIntervalSinceLastUpdate(_this);//Eason: when change MaxMah clear interval
		IfUpdateSavedTime = false;
	}
	//Eason : if last time is 10mA ---	
//Eason: BAT Cap can drop when cable in --- 

//Eason boot up in BatLow situation, take off cable can shutdown+++
if(true==g_BootUp_IsBatLow )
{
    if(1==!gpio_get_value(maxim8934DCIN))
    {
        IsBatLowtoFilter = false;	
        printk("[BAT][BootUp]BootUp IsBatLow, Cable on, BatLow false\n");
    }else{
        IsBatLowtoFilter = g_BootUp_IsBatLow;
        printk("[BAT][BootUp]BootUp IsBatLow, Cable off, BatLow true\n");
    }
}else{
	IsBatLowtoFilter = _this->BatteryService_IsBatLow;
}
//Eason boot up in BatLow situation, take off cable can shutdown---    
    
    A66_capacity = _this->gpCapFilterA66->filterCapacity
                                    (_this->gpCapFilterA66,
                                      refcapacity, 
                                      lastCapacity,
                                      EnableBATLifeRise,
                                      _this->BatteryService_IsCharging,
                                      _this->BatteryService_IsFULL,
                                      IsBatLowtoFilter,
                                      maxMah,
                                      intervalSinceLastUpdate);
//Eason add to check full & 100%+++
    if(true==_this->BatteryService_IsFULL && A66_capacity != 100)
    {
        printk("[BAT][Ser]Full but not 100 ,restart charging\n");
        DoAfterDecideNotFull(_this);
    }
//Eason add to check full & 100%---

    printk("[BAT][Ser]report Capacity:%d,%d,%d,%d,%d,%d,%d,%d,%ld==>%d\n",
                                    refcapacity,
                                    lastCapacity,
                                      hasCable,
                                      EnableBATLifeRise,
                                      _this->BatteryService_IsCharging,
                                      _this->BatteryService_IsFULL,
                                      IsBatLowtoFilter,
                                      maxMah,
                                      intervalSinceLastUpdate,
                                      A66_capacity);
//ASUS_BSP +++ Eason_Chang add event log +++
     ASUSEvtlog("[BAT][Ser]report Capacity:%d,%d,%d,%d,%d,%d,%d,%d,%ld==>%d\n",
                                    refcapacity,
                                    lastCapacity,
                                      hasCable,
                                      EnableBATLifeRise,
                                      _this->BatteryService_IsCharging,
                                      _this->BatteryService_IsFULL,
                                      IsBatLowtoFilter,
                                      maxMah,
                                      intervalSinceLastUpdate,
                                      A66_capacity);
//ASUS_BSP --- Eason_Chang add event log ---     
//ASUS_BSP +++ Eason_Chang BalanceMode : set A66_cap for cmd test 
    IsBalTest = IsBalanceTest();
         if( 1 == IsBalTest){
                 A66_capacity = GetBalanceModeA66CAP();
                 printk("[BAT][Bal][test]A66 cap: %d\n",A66_capacity );
         }
//ASUS_BSP --- Eason_Chang BalanceMode : set A66_cap for cmd test 

    if(A66_capacity < 0 || A66_capacity >100)
    {
        printk("[BAT][Ser]Filter return value fail!!!\n");
    }else /*if(_this->A66_capacity == A66_capacity){    
       printk("[BAT][Ser]A66  have same cap:%d\n",A66_capacity);
    }else if(_this->A66_capacity != A66_capacity)*/{
       _this->A66_capacity = A66_capacity;

       if(1==AX_MicroP_IsP01Connected()){
              if( false == DecideIfPadDockHaveExtChgAC()){ 
                    BatteryServiceDoBalance(_this);
              }else{
                    Init_Microp_Vbus__Chg();
              }

	  	//Eason: dynamic set Pad alarm +++
 	
		 SetRTCAlarm();

		//Eason: dynamic set Pad alarm ---
       }

       // when A66 Cap = 0% shutdown device no matter if has cable+++ 
       if( 0==_this->A66_capacity )
       {
          g_AcUsbOnline_Change0 = true;
          AcUsbPowerSupplyChange();
          PadDock_AC_PowerSupplyChange();
       }
       // when A66 Cap = 0% shutdown device no matter if has cable---
       //Eason : prevent thermal too hot, limit charging current in phone call+++
       if(g_A60K_hwID >=A66_HW_ID_ER2)
       {
        	judgePhoneOnCurLimit();
            judgeThermalCurrentLimit(); //when thermal too hot limit charging current
       }
       //Eason : prevent thermal too hot, limit charging current in phone call---
       _this->callback->onServiceStatusUpdated(_this->callback);
    }   
    
    _this->IsCalculateCapOngoing = false;
    

    mutex_unlock(&_this->filter_lock);

}
static int BatteryServiceGauge_OnCapacityReply(struct AXI_Gauge *gauge, struct AXI_Gauge_Callback *gaugeCb, int batCap, int result)
{   

    AXC_BatteryService  *_this=
        container_of(gaugeCb, AXC_BatteryService, gaugeCallback);

    //Eason : In suspend have same cap don't update savedTime +++
    int A66_LastTime_capacity;
    A66_LastTime_capacity = _this->A66_capacity;
    //Eason : In suspend have same cap don't update savedTime ---

    mutex_lock(&_this->main_lock);

    if(BAT_CAP_REPLY_ERR==result){
            _this->IsResumeUpdate = false;
            _this->IsResumeMahUpdate = false;
            pr_err("[A66][BAT][Ser]:Error askCapacity\n");
    }else{
        AXC_BatteryService_reportPropertyCapacity(
            _this,
            batCap);

        //Eason : In suspend have same cap don't update savedTime +++
        if( (A66_LastTime_capacity == _this->A66_capacity)&&(true==SameCapDontUpdateSavedTime)&&
            (false==g_RTC_update) )
        {
            printk("[BAT][Ser]:In suspend have same Cap dont update savedTime\n");
        }else{
            g_RTC_update = false;
            _this->savedTime=updateNowTime(_this);    
        }
        //Eason : In suspend have same cap don't update savedTime ---
        //Eason: when change MaxMah clear interval+++
	  _this->ForceSavedTime = updateNowTime(_this);//for A66 will always update no matter if change MaxMAh
	  //Eason: when change MaxMah clear interval---
    }
    AXC_BatteryService_scheduleNextPolling(_this);
    
    wake_unlock(&_this->cap_wake_lock);
    mutex_unlock(&_this->main_lock);

    return 0;
}
int BatteryServiceGauge_AskSuspendCharging(struct AXI_Gauge_Callback *gaugeCb)
{
    AXC_BatteryService  *_this=
         container_of(gaugeCb, AXC_BatteryService, gaugeCallback);
    _this->callback->changeChargingCurrent(_this->callback,NO_CHARGER_TYPE);
    //gpCharger->EnableCharging(gpCharger,false);//stop curr may need delay
    return 0;
}
int BatteryServiceGauge_AskResumeCharging(struct AXI_Gauge_Callback *gaugeCb)
{
    AXC_BatteryService  *_this=
         container_of(gaugeCb, AXC_BatteryService, gaugeCallback);
    _this->callback->changeChargingCurrent(_this->callback,_this->chargerType);
    //gpCharger->EnableCharging(gpCharger,true);//stop curr may need delay
    return 0;
}

static int Report_P02Cable(void)
{
	if(1==AX_MicroP_get_USBDetectStatus(Batt_P01)){
		balance_this->P02_IsCable = true;
	}else{
		balance_this->P02_IsCable = false;
	}

    return balance_this->P02_IsCable;
}
static void Report_P02ChgStatus(int P02Chg) 
{
    balance_this->P02_IsCharging = false;
    balance_this->P02_IsFULL = false; 

    if(1==P02Chg){
            balance_this->P02_IsCharging = true;
    }else if(2==P02Chg){
            balance_this->P02_IsFULL = true; 
    }       
}
static void P02_reportPropertyCapacity(struct AXC_BatteryService *_this, int P02_refcapacity)
{

    int Pad_capacity;
    int IsBalTest;//ASUS_BSP Eason_Chang BalanceMode

    int lastCapacity;

    int P02_maxMah;

    bool P02_hasCable;
    int  P02_chgStatus;

    time_t P02_intervalSinceLastUpdate;

    mutex_lock(&_this->filter_lock);

    P02_intervalSinceLastUpdate  = P02_BatteryService_getIntervalSinceLastUpdate(_this);
    

    //We need do ask capcaity to filter at first time, in case there is FULL orBATLow 
    if(true == _this->P02_IsFirstAskCap){

        lastCapacity = P02_refcapacity;

        _this->P02_IsFirstAskCap = false;
        
    }else{

        lastCapacity = _this->Pad_capacity;

    }

    if (true == _this->P02_IsResumeUpdate){
        P02_hasCable = _this->P02_HasCableBeforeSuspend;
        //P02_chgStatus = _this->P02_ChgStatusBeforeSuspend;
        P02_maxMah = _this->P02_MaxMahBeforeSuspend;
        _this->P02_IsResumeUpdate = false;
 
    }else{
        P02_hasCable = Report_P02Cable();
        //P02_chgStatus = AX_MicroP_get_ChargingStatus(Batt_P01);
        P02_maxMah = P02_ChooseMaxMah();
    }

    P02_chgStatus = AX_MicroP_get_ChargingStatus(Batt_P01);
    Report_P02ChgStatus(P02_chgStatus);

    Pad_capacity = _this->gpCapFilterP02->filterCapacity
                                     (_this->gpCapFilterP02,
                                      P02_refcapacity, lastCapacity,
                                      P02_hasCable,
                                      _this->P02_IsCharging,
                                      _this->P02_IsFULL,
                                      _this->P02_IsBatLow,
                                      P02_maxMah,
                                      P02_intervalSinceLastUpdate);

    printk("[BAT][Ser][P02]report Capacity:%d,%d,%d,%d,%d,%d,%d,%ld==>%d\n",
                                    P02_refcapacity,
                                    lastCapacity,
                                      P02_hasCable,
                                      _this->P02_IsCharging,
                                      _this->P02_IsFULL,
                                      _this->P02_IsBatLow,
                                      P02_maxMah,
                                      P02_intervalSinceLastUpdate,
                                      Pad_capacity);
    //ASUS_BSP Eason_Chang add event log +++
    ASUSEvtlog("[BAT][Ser][P02]report Capacity:%d,%d,%d,%d,%d,%d,%d,%ld==>%d\n",
                                    P02_refcapacity,
                                    lastCapacity,
                                      P02_hasCable,
                                      _this->P02_IsCharging,
                                      _this->P02_IsFULL,
                                      _this->P02_IsBatLow,
                                      P02_maxMah,
                                      P02_intervalSinceLastUpdate,
                                      Pad_capacity);
    //ASUS_BSP Eason_Chang add event log ---

//ASUS_BSP +++ Eason_Chang  : set Pad_cap for cmd test
    IsBalTest = IsBalanceTest();
    if( 1 == IsBalTest){
            Pad_capacity = BatteryServiceGetPADCAP();
    }
//ASUS_BSP --- Eason_Chang  : set Pad_cap for cmd test 


    if(Pad_capacity < 0 || Pad_capacity >100){

        printk("[BAT][Ser]Filter return value fail!!!\n");
    }else if(_this->Pad_capacity == Pad_capacity){    
       printk("[BAT][Ser]Pad have same cap:%d\n",Pad_capacity);
    }else if(_this->Pad_capacity != Pad_capacity){
       _this->Pad_capacity = Pad_capacity;
       
       BatteryService_P02update();
    }   
    
    //wake_unlock(&_this->cap_wake_lock);

    mutex_unlock(&_this->filter_lock);

}
static int P02Gauge_OnCapacityReply(struct AXI_Gauge *gauge, struct AXI_Gauge_Callback *gaugeCb, int batCap, int result)
{   

    AXC_BatteryService  *_this=
        container_of(gaugeCb, AXC_BatteryService, P02gaugeCallback);

    mutex_lock(&_this->main_lock);

    P02_reportPropertyCapacity(
        _this,
        batCap);

    _this->P02_savedTime=updateNowTime(_this);
    pr_debug("[BAT][Ser]:P02Gauge_OnCapacityReply\n");
    //ReportTime();

    mutex_unlock(&_this->main_lock);

    _this->gauge->askCapacity(_this->gauge);
    pr_debug("[BAT][Ser]:P02Gauge_askCapacity\n");
    //ReportTime();
    return 0;
}
int P02Gauge_AskSuspendCharging(struct AXI_Gauge_Callback *gaugeCb)
{ 
    return 0;
}
int P02Gauge_AskResumeCharging(struct AXI_Gauge_Callback *gaugeCb)
{
    return 0;
}


static int Report_DockCable(void)
{
	if(1==AX_MicroP_get_USBDetectStatus(Batt_Dock)){
		balance_this->Dock_IsCable = true;
	}else{
		balance_this->Dock_IsCable = false;
	}

    return balance_this->Dock_IsCable;
}

static void Report_DockChgStatus(int DockChg) 
{
    balance_this->Dock_IsCharging = false;
    balance_this->Dock_IsFULL = false; 

    if(1==DockChg){
            balance_this->Dock_IsCharging = true;
    }else if(2==DockChg){
            balance_this->Dock_IsFULL = true; 
    }       
}

static void Dock_reportPropertyCapacity(struct AXC_BatteryService *_this, int Dock_refcapacity)
{

    int Dock_capacity;

    int lastCapacity;

    int Dock_maxMah;

    bool Dock_hasCable;
    int  Dock_chgStatus;

    time_t Dock_intervalSinceLastUpdate;

    mutex_lock(&_this->filter_lock);

    Dock_intervalSinceLastUpdate  = Dock_BatteryService_getIntervalSinceLastUpdate(_this);
    

    //We need do ask capcaity to filter at first time, in case there is FULL orBATLow 
    if(true == _this->Dock_IsFirstAskCap){

        lastCapacity = Dock_refcapacity;

        _this->Dock_IsFirstAskCap = false;
        
    }else{

        lastCapacity = _this->Dock_capacity;

    }

    if (true == _this->Dock_IsResumeUpdate){
        Dock_hasCable = _this->Dock_HasCableBeforeSuspend;
        //Dock_chgStatus = _this->Dock_ChgStatusBeforeSuspend;
        Dock_maxMah = _this->Dock_MaxMahBeforeSuspend;
        printk("[BAT][Ser][Dock]ResumeUpdate\n");
        _this->Dock_IsResumeUpdate = false;
 
    }else{
        Dock_hasCable = Report_DockCable();
        //Dock_chgStatus = AX_MicroP_get_ChargingStatus(Batt_Dock);
        Dock_maxMah = Dock_ChooseMaxMah();
    }

    Dock_chgStatus = AX_MicroP_get_ChargingStatus(Batt_Dock);
    Report_DockChgStatus(Dock_chgStatus);

    Dock_capacity = _this->gpCapFilterDock->filterCapacity
                                     (_this->gpCapFilterDock,
                                      Dock_refcapacity, lastCapacity,
                                      Dock_hasCable,
                                      _this->Dock_IsCharging,
                                      _this->Dock_IsFULL,
                                      _this->Dock_IsBatLow,
                                      Dock_maxMah,
                                      Dock_intervalSinceLastUpdate);

    printk("[BAT][Ser][Dock]report Capacity:%d,%d,%d,%d,%d,%d,%d,%ld==>%d\n",
                                    Dock_refcapacity,
                                    lastCapacity,
                                      Dock_hasCable,
                                      _this->Dock_IsCharging,
                                      _this->Dock_IsFULL,
                                      _this->Dock_IsBatLow,
                                      Dock_maxMah,
                                      Dock_intervalSinceLastUpdate,
                                      Dock_capacity);

//ASUS_BSP +++ Eason_Chang  : set Pad_cap for cmd test
/*
    IsBalTest = IsBalanceTest();
    if( 1 == IsBalTest){
            Pad_capacity = BatteryServiceGetDockCAP();
    }
*/
//ASUS_BSP --- Eason_Chang  : set Pad_cap for cmd test 


    if(Dock_capacity < 0 || Dock_capacity >100){

        printk("[BAT][Ser][Dock]Filter return value fail!!!\n");
    }else if(_this->Dock_capacity == Dock_capacity){    
       printk("[BAT][Ser]Dock have same cap:%d\n",Dock_capacity);
    }else if(_this->Dock_capacity != Dock_capacity){
       _this->Dock_capacity = Dock_capacity;
       
       //BatteryService_Dockupdate();//Pad will do this together
    }   
    
    //wake_unlock(&_this->cap_wake_lock);

    mutex_unlock(&_this->filter_lock);

}
static int DockGauge_OnCapacityReply(struct AXI_Gauge *gauge, struct AXI_Gauge_Callback *gaugeCb, int batCap, int result)
{   

    AXC_BatteryService  *_this=
        container_of(gaugeCb, AXC_BatteryService, DockgaugeCallback);

    mutex_lock(&_this->main_lock);

    Dock_reportPropertyCapacity(
        _this,
        batCap);

    _this->Dock_savedTime=updateNowTime(_this);
    pr_debug("[BAT][Ser]:P02Gauge_OnCapacityReply\n");
    //ReportTime();

    mutex_unlock(&_this->main_lock);

    _this->P02gauge->askCapacity(_this->P02gauge);
    pr_debug("[BAT][Ser]:P02Gauge_askCapacity\n");
    //ReportTime();
    return 0;
}
int DockGauge_AskSuspendCharging(struct AXI_Gauge_Callback *gaugeCb)
{ 
    return 0;
}
int DockGauge_AskResumeCharging(struct AXI_Gauge_Callback *gaugeCb)
{
    return 0;
}

//ASUS BSP Eason_Chang --- batteryservice to gauge

//static int BatteryService_CalculateBATCAP(AXC_BatteryService *_this)
//{             
//    return _this->gauge->GetBatteryLife(_this->gauge);
//}
static bool BatteryService_ifFixedPollingInterval(struct AXC_BatteryServiceTest *test)
{
    return (-1 != test->pollingInterval);
}
static bool BatteryService_ifFixedFilterLastUpdateInterval(struct AXC_BatteryServiceTest *test)
{
    return (-1 != test->filterLastUpdateInterval);

}
static void BatteryService_changePollingInterval(struct AXC_BatteryServiceTest *test,bool fixed,int interval)
{
    AXC_BatteryService  *_this=
        container_of(test, AXC_BatteryService, test);

    if(fixed){
        printk("%s:fix interval to %d\n",__FUNCTION__,interval);

        test->pollingInterval = interval;

        _this->miParent.suspend(&_this->miParent);

        _this->miParent.resume(&_this->miParent, interval);

    }else{
        printk("%s:don't fix interval\n",__FUNCTION__);

        test->pollingInterval = -1;

    }
}
static void BatteryService_changeFilterLastUpdateInterval(struct AXC_BatteryServiceTest *test,bool fixed,int interval)
{
    if(fixed){

        printk("%s:fix interval to %d\n",__FUNCTION__,interval);

        test->filterLastUpdateInterval = interval;

    }else{
        printk("%s:don't fix interval\n",__FUNCTION__);

        test->filterLastUpdateInterval = -1;
    }
}
static AXC_BatteryService g_AXC_BatteryService={
    .miParent = {
        .getChargingStatus = AXC_BatteryService_getChargingStatus,
        .getCapacity = AXC_BatteryService_getCapacity,
        .onCableInOut =AXC_BatteryService_onCableInOut,
        .onChargingStop =AXC_BatteryService_onChargingStop,
        .onChargingStart = AXC_BatteryService_onChargingStart,
        .onBatteryLowAlarm= AXC_BatteryService_onBatteryLowAlarm,
        .onBatteryRemoved = AXC_BatteryService_onBatteryRemoved,
        .suspend = AXC_BatteryService_suspend,
        .resume =AXC_BatteryService_resume,
        .forceResume = AXC_BatteryService_forceResume,
        .dockSuspend = AXC_BatteryService_dockSuspend,
    },
    .mbInit = false,
    .IsFirstForceResume = true,
    .callback = NULL,
    .A66_capacity = 100,//saved capacity
    .Pad_capacity = 100,
    .Dock_capacity = 100,
    .ForceSavedTime = 0,//for A66 will always update no matter if change MaxMAh
    .savedTime = 0,//for A66 may dont update if change 10==MaxMAh
    .P02_savedTime = 0,
    .Dock_savedTime = 0,
    .BatteryService_IsCable = false,
    .BatteryService_IsCharging = false,
    .BatteryService_IsFULL = false,
    .BatteryService_IsBatLow = false,
    .isMainBatteryChargingDone= false,
    .IsFirstAskCap = true,
    .IsFirstAskCable = true,
    .HasCableBeforeSuspend = false,
    .IsResumeUpdate = false,
    .IsResumeMahUpdate = false,
    .IsCalculateCapOngoing = false,
    .P02_IsCable = false,
    .P02_IsCharging = false,
    .P02_IsFULL = false,
    .P02_IsBatLow = false,
    .P02_IsFirstAskCap = true,
    .P02_HasCableBeforeSuspend = false,
    .P02_IsResumeUpdate = false,
    //.P02_ChgStatusBeforeSuspend = 0,
    .P02_MaxMahBeforeSuspend = 0,
    .Dock_IsCable = false,
    .Dock_IsCharging = false,
    .Dock_IsFULL = false,
    .Dock_IsBatLow = false,
    .Dock_IsFirstAskCap = true,
    .Dock_HasCableBeforeSuspend = false,
    .Dock_IsResumeUpdate = false,
    //.Dock_ChgStatusBeforeSuspend = 0,
    .Dock_MaxMahBeforeSuspend = 0,
    .IsSuspend = true,
    .IsDockExtChgIn = false,
    .IsDockInitReady = false,
    .gaugeCallback ={
        .onCapacityReply = BatteryServiceGauge_OnCapacityReply,
        .askSuspendCharging = BatteryServiceGauge_AskSuspendCharging,   
        .askResumeCharging = BatteryServiceGauge_AskResumeCharging,
        },// batteryservice to gauge
    .P02gaugeCallback ={
        .onCapacityReply = P02Gauge_OnCapacityReply,
        .askSuspendCharging = P02Gauge_AskSuspendCharging,   
        .askResumeCharging = P02Gauge_AskResumeCharging,
        },// batteryservice to gauge 
    .DockgaugeCallback ={
        .onCapacityReply = DockGauge_OnCapacityReply,
        .askSuspendCharging = DockGauge_AskSuspendCharging,   
        .askResumeCharging = DockGauge_AskResumeCharging,
        },// batteryservice to gauge      
    .chargerType =  NOTDEFINE_TYPE ,  // batteryservice to gauge
    .gauge = NULL,  // batteryservice to gauge
    .P02gauge = NULL,
    .gpCapFilterA66 = NULL,
    .gpCapFilterP02 = NULL,
    .defaultPollingInterval = DEFAULT_ASUSBAT_POLLING_INTERVAL , // batteryservice to gauge
    .fsmCallback ={
        .onChangeChargingCurrent = BatteryServiceFsm_OnChangeChargingCurrent,
        .onStateChanged = BatteryServiceFsm_OnStateChanged,
        },// batteryservice to fsm
    .fsm = NULL,                 // batteryservice to fsm
    .fsmState = NOTDEFINE_STATE ,// batteryservice to fsm
    .test = {
        .pollingInterval = -1,
        .filterLastUpdateInterval = -1,
        .ifFixedPollingInterval = BatteryService_ifFixedPollingInterval,
        .ifFixedFilterLastUpdateInterval =BatteryService_ifFixedFilterLastUpdateInterval,
        .changePollingInterval=BatteryService_changePollingInterval,
        .changeFilterLastUpdateInterval= BatteryService_changeFilterLastUpdateInterval,
    },
    .gChargerStateChangeNotifier={
        .Notify = NotifyForChargerStateChanged,
        .onChargingStart = onChargingStart,
        },
};

AXI_BatteryServiceFacade *getBatteryService(AXI_BatteryServiceFacadeCallback *callback)
{
    static AXI_BatteryServiceFacade *lpBatteryService = NULL;

    if(NULL == lpBatteryService){

        lpBatteryService = &g_AXC_BatteryService.miParent;

        AXC_BatteryService_constructor(&g_AXC_BatteryService, callback);
    }

    return lpBatteryService;
}

AXC_BatteryServiceTest *getBatteryServiceTest(void)
{
    return &g_AXC_BatteryService.test;
}

int getPowerBankCharge(void)
{
    return IsPowerBankCharge;
}

int getBalanceCharge(void)
{
    return IsBalanceCharge;
}
#endif //#ifdef CONFIG_BATTERY_ASUS_SERVICE








