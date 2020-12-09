/* kernel/drivers/power/gauge/axc_gauge_a66.c
 *
 * Copyright (C) 2012 ASUS Corporation.
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
 
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/rtc.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/jiffies.h>
#include <linux/delay.h>

#include "axc_gaugefactory.h"
#include "axc_gauge_a66.h"
//ASUS_BSP Eason_Chang add event log +++
#include <linux/asusdebug.h>
//ASUS_BSP Eason_Chang add event log ---


/* -------------------------------------------------------------------------- */

#define BATTERY_RESISTOR_CALIBRATION_FILE_PATH    "/data/data/BattResicali.nvm"

#define OCV_PER_SPEEDUP_UPDATE		12

#define TIMES_FOR_GENERAL_READ_ADC		15

#define DEFAULT_DEVICE_RESISTOR_VALUE	150

#define RESISTOR_MAX_VAL	200
#define RESISTOR_MIN_VAL		150

/* UPDATE_INERVAL */
#define UPDATE_INERVAL_DEFAULT			180
#define UPDATE_INERVAL_BAT_LOW_IRQ		60

/* TBL_SIZE - different table size */
#define OCV_TBL_SIZE		101

/* -------------------------------------------------------------------------- */

int MainSrcOCVtoPerTbl[OCV_TBL_SIZE]={
	3401, 3439, 3474, 3508, 3539, 3565, 3588, 3609, 3626, 3642, 
	3657, 3673, 3679, 3682, 3684, 3685, 3686, 3687, 3688, 3690,
	3698, 3707, 3714, 3721, 3729, 3735, 3740, 3743, 3746, 3750,
	3755, 3758, 3762, 3764, 3767, 3770, 3772, 3775, 3777, 3779,
	3781, 3784, 3786, 3788, 3791, 3793, 3796, 3799, 3801, 3804,

	3807, 3811, 3814, 3817, 3821, 3825, 3829, 3833, 3837, 3842,
	3848, 3854, 3862, 3872, 3883, 3893, 3902, 3909, 3916, 3922,
	3928, 3934, 3940, 3946, 3952, 3958, 3963, 3968, 3973, 3978,
	3983, 3989, 3996, 4005, 4015, 4026, 4038, 4050, 4060, 4067,
	4072, 4077, 4085, 4093, 4101, 4110, 4119, 4128, 4137, 4146,
	4156
};

/* CurrOCVtoPerTbl - to determine which ocv to percent table used */
const int *CurrOCVtoPerTbl = NULL;

typedef union mytemp mytemp;

union mytemp{
	s16 mnValue;
	char mnChar[2];
};


/* -------------------------------------------------------------------------- */

extern int get_temp_for_ASUSswgauge(void);
extern int get_voltage_for_ASUSswgauge(void);
extern int get_current_for_ASUSswgauge(void);

/* -------------------------------------------------------------------------- */
//ASUS BSP Eason filter adc read current <= 15mA when doesn't has cable +++
extern bool report_BatteryService_If_HasCable(void);
//ASUS BSP Eason filter adc read current <= 15mA when doesn't has cable ---
//Eason: Do VF with Cable when boot up+++
static bool WhenBootUpDoVf = true;
extern int getIfonline(void);
//Eason: Do VF with Cable when boot up---
//Eason if doesn't get correct ADC Vol&Curr at first update Cap show unknow status +++ 
bool g_adc_get_correct_VolCurr = false;
//Eason if doesn't get correct ADC Vol&Curr at first update Cap show unknow status ---

static void AXC_Gauge_A66_ReadVoltageCurrentFromADC(AXI_Gauge *apGauge, int *volt, int *curr)
{
	*curr = get_current_for_ASUSswgauge();
 	*volt = get_voltage_for_ASUSswgauge();
 	return;
}

static int find_tbl_index(int target,const int *array, int array_size)
{
	int i = 0;
	int target_index = -1;

	BUG_ON(NULL == array);

	if (target <= array[0]) { 
		target_index = 0;
	} else if(target >= array[array_size-1]) {
		target_index = array_size-1;
	} else {
		for (i = 1; i < array_size; i++)
		{
			if(target < array[i])
				break;
		}
		target_index = i-1;
    }
	return target_index;
}

static int init_resistor_cali(AXC_Gauge_A66 *this)
{
	struct file *fd;
	mm_segment_t mmseg_fs;
	int FileLength = 0;
	mytemp lnResistorCalibrationData;

	mmseg_fs = get_fs();
	set_fs(KERNEL_DS);

	fd = filp_open( BATTERY_RESISTOR_CALIBRATION_FILE_PATH, O_RDONLY,0);
	if (IS_ERR(fd))
	{
		pr_err( "[BAT][Gau][A66]error! can not open battery Resistor cali file, use default value.\n");
		this->resistorCali = DEFAULT_DEVICE_RESISTOR_VALUE;
		set_fs(mmseg_fs);
		return -1;
	}

	FileLength = fd->f_op->read(fd, lnResistorCalibrationData.mnChar, 2, &fd->f_pos);
	if (2 <= FileLength) {
		this->resistorCali = lnResistorCalibrationData.mnValue;
		pr_info( "[BAT][Gau][A66] init R Cali:%d\n", this->resistorCali);
	} else {
		pr_err( "[BAT][Gau][A66] cannot read R Cali\n");
	}

	if (filp_close(fd, NULL))
       		pr_err("[BAT][Gau][A66] fail to close resistor cali file in %s\n", __func__);
	
	set_fs(mmseg_fs);
  	return 0;
}

//ASUS_BSP  +++ Eason_Chang "add BAT info time"
static void ReportTime(void)
{
	struct timespec ts;
	struct rtc_time tm;
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);

	pr_info ("[BAT][Gau][A66] %d-%02d-%02d %02d:%02d:%02d,T=%d\n"
		,tm.tm_year + 1900
		,tm.tm_mon + 1
		,tm.tm_mday
		,tm.tm_hour
		,tm.tm_min
		,tm.tm_sec
		,get_temp_for_ASUSswgauge());
}
//ASUS_BSP  --- Eason_Chang "add BAT info time"


static int map_per_with_ocv(int anVocv)
{
    int target_index = 0;

    target_index = find_tbl_index(anVocv, CurrOCVtoPerTbl, OCV_TBL_SIZE);
    if (target_index == -1) {
    	pr_err("[BAT][Gau]There are something wrong in OCV(%d) to Percentage\n",anVocv);
        target_index = 0;
    } else {
		pr_debug( "[BAT][Gau] ocv per index:%d\n", target_index);
	}

    return target_index;	
}

/* cal_ocv_percent_when_discharging - Calculate battery life when discharging.
 * Use formula, OCV = V + I*R, to get ocv persent. 
 */
static int cal_ocv_percent_when_discharging(
	AXC_Gauge_A66 *this,
	int volt,
	int curr)
{
	int ocv = 0;
	int percent = 0;

	int resistor;

	resistor = this->resistorCali;
    resistor = DEFAULT_DEVICE_RESISTOR_VALUE;//Eason use default R:150

	ocv = volt + curr*resistor/1000 ;

	percent = map_per_with_ocv(ocv);
	/*printk( "[BAT][Gau]ocv per=%d, ocv=%d, v=%d, c=%d, r=%d\n",
		 percent, ocv, volt, curr, resistor);*/
    printk( "[BAT][Gau]ocv per=%d, ocv=%d, r=%d\n",
		 percent, ocv, resistor);
    
    //ASUS_BSP Eason_Chang add event log +++
    ASUSEvtlog( "[BAT][Gau]ocv per=%d, ocv=%d, r=%d\n",
		 percent, ocv, resistor);
    //ASUS_BSP Eason_Chang add event log ---

	return percent;
}

/* cal_ocv_percent_by_volt_directly - Calculate ocv percent by voltage directly.
 * Usually used by the case of discharging to read volt.
 */
//Eason always use V+IR  ++++++++// 
/*
static int cal_ocv_percent_by_volt_directly(
	int volt)
{
	int percent = 0;

	percent = map_per_with_ocv(volt);
	// Essential information for OCV table
	printk( "[BAT][Gau]ocv per=%d, ocv=v=%d\n",
		percent,
		volt);
	return percent;
}
*/
//Eason always use V+IR  --------//
static int decideVoltSec(int volt)
{
    int i=0;
    int j;
    int voltTableSec[20];

    for(j=0;j<20;j++)
    { 
        voltTableSec[j]=3440+40*j;
    }

    while(voltTableSec[i]<=volt && i<20)
    {
       i++;
    }
    
    return i;
}

enum currentSec{
    currSectionI = 0,
    currSectionII,    
    currSectionIII,
};    

static int decideCurrSec(int curr)
{
    if(curr <= 300){
        return currSectionI;
    }else if(curr >= 900){
        return currSectionIII;
    }else{
        return currSectionII;
    }    
}

static int decideVf300(int voltSec)
{
    int Vf300TableSec[20] = {29,27,27,29,29,35,38,43,41,28,28,28,29,25,24,25,25,24,15,15};
                          //3440 to 4200  interval:40 mV , total:20
    if(voltSec > 19){                      
        return Vf300TableSec[19];
    }else{
        return Vf300TableSec[voltSec];                      
    }    
}

static int decideVf900(int voltSec)
{
    int Vf900TableSec[20] = {15,15,15,9,19,22,29,31,42,45,48,52,64,65,67,65,60,61,33,33};
                          //3440 to 4200  interval:40 mV , total:20
    if(voltSec > 19){                      
        return Vf900TableSec[19];
    }else{
        return Vf900TableSec[voltSec];                      
    }    
}

static int decideVolSecSpec(int voltSec)
{
    if(0==voltSec){
        return 3440;
    }else if(20==voltSec){
        return 4200;
    }else{
        return (3440+40*(voltSec-1));
    }
}

static void doAdcVfModify(int* volt,int* curr)
{   
    int voltSec;
    int currSec;
    long Vfcurr_seci;
    long Vf300_seci;
    long Vf900_seci;
    long Vfcurr_sec_iplusOne;
    long Vf300_sec_iplusOne;
    long Vf900_sec_iplusOne;
    long Vminus;
    long voltSpecSeci;
    

    *curr = (*curr)*(-1);//for calculate convinient
    
    voltSec = decideVoltSec(*volt);
    currSec = decideCurrSec(*curr);

    printk("[BAT][vf]Vsec:%d,Csec:%d\n",voltSec,currSec);

   
    if(0==voltSec){//voltSec:0~20
        Vf300_seci = decideVf300(voltSec);
        Vf300_sec_iplusOne = decideVf300(voltSec);
    }else if(20==voltSec){
        Vf300_seci = decideVf300(voltSec-1);
        Vf300_sec_iplusOne = decideVf300(voltSec-1);
    }else{
        Vf300_seci = decideVf300(voltSec-1);
        Vf300_sec_iplusOne = decideVf300(voltSec);
    }

    if(0==voltSec){//voltSec:0~20
        Vf900_seci = decideVf900(voltSec);
        Vf900_sec_iplusOne = decideVf900(voltSec);
    }else if(20==voltSec){
        Vf900_seci = decideVf900(voltSec-1);
        Vf900_sec_iplusOne = decideVf900(voltSec-1);
    }else{
        Vf900_seci = decideVf900(voltSec-1);
        Vf900_sec_iplusOne = decideVf900(voltSec);
    }
    

    if(currSectionII==currSec){
        Vfcurr_seci = Vf300_seci + (*curr-300)*(Vf900_seci-Vf300_seci)/(900-300);
        Vfcurr_sec_iplusOne = Vf300_sec_iplusOne 
                            + (*curr-300)*(Vf900_sec_iplusOne-Vf300_sec_iplusOne)/(900-300);
    }else if(currSectionI==currSec){
        Vfcurr_seci = Vf300_seci;
        Vfcurr_sec_iplusOne = Vf300_sec_iplusOne;
    }else{//currSectionIII==currSec
        Vfcurr_seci = Vf900_seci;
        Vfcurr_sec_iplusOne = Vf900_sec_iplusOne;
    }


    voltSpecSeci =  decideVolSecSpec(voltSec);

    Vminus = Vfcurr_seci + (*volt-voltSpecSeci)*(Vfcurr_sec_iplusOne-Vfcurr_seci)/40;

    printk("[BAT][vf]Vf300i:%ld,Vf300i+1:%ld,Vf900i:%ld,Vf900i+1:%ld,Vfcurri:%ld,Vfcurri+1:%ld,Vminus:%ld\n"
                    ,Vf300_seci,Vf300_sec_iplusOne,Vf900_seci,Vf900_sec_iplusOne
                    ,Vfcurr_seci,Vfcurr_sec_iplusOne,Vminus);

    *volt = (*volt)-(int)Vminus;
    *curr = (*curr)*(-1);//for calculate convinient
    
}

/* cal_bat_capacity_work -
 */
static void cal_bat_capacity_work(struct work_struct *work)
{
	//int one_time_volt;
	//int one_time_curr;
	int volt;
	int curr;
	int ocv_percent=0;
    int online; ////Eason: Do VF with Cable when boot up

	AXC_Gauge_A66 *this = container_of(work, AXC_Gauge_A66, calBatCapacityWorker.work);

	// Read resistor when first time to cal battery capacity 
	if (!this->firstCalBatCapacity) {
		if (init_resistor_cali(this) < 0) {
			pr_info("[BAT][Gau][A66] cannot read bat resistor from file, maybe file system not ready, try again!\n");
			queue_delayed_work(this->gaugeQueue, &this->readBatResistorWorker , 8*HZ);
		}
			
		this->firstCalBatCapacity = true;
	}

	wake_lock(&this->calBatCapWlock);

    /*
	this->msParentGauge.ReadVoltCurr(
		&this->msParentGauge,
		1,
		&one_time_volt,
		&one_time_curr);

	printk("[BAT][Gau][A66]judge curr:%d\n",one_time_curr);
	*/

    //Eason always use V+IR  +++++++++//
	//if (one_time_curr > 0) {  // on discharging
		this->msParentGauge.ReadVoltCurr(
			&this->msParentGauge,
			TIMES_FOR_GENERAL_READ_ADC,
			&volt,
			&curr);


        if( (curr < 0) &&(g_A60K_hwID >=A66_HW_ID_ER2)&& (1==getIfonline()) )
        {
            printk("[BAT][vf]beforeVF: V:%d,C:%d\n",volt,curr);
            doAdcVfModify(&volt,&curr);
            printk("[BAT][vf]afterVF: V:%d,C:%d\n",volt,curr);
            //ASUS_BSP Eason_Chang add event log +++
            ASUSEvtlog("[BAT][vf]afterVF: V:%d,C:%d\n",volt,curr);
            //ASUS_BSP Eason_Chang add event log ---
        }
        else if((true == WhenBootUpDoVf)&&(g_A60K_hwID >=A66_HW_ID_ER2))//Eason: Do VF with Cable when boot up+++
        {
        	online = getIfonline();

        	if( 1==online )
            {
                printk("[BAT][vf]:Do VF with Cable when boot up\n");
                printk("[BAT][vf]beforeVF: V:%d,C:%d\n",volt,curr);
                doAdcVfModify(&volt,&curr);
                printk("[BAT][vf]afterVF: V:%d,C:%d\n",volt,curr);
            } 
        }//Eason: Do VF with Cable when boot up---

        WhenBootUpDoVf = false;//Eason: Do VF with Cable when boot up
    
        if (volt > 0){
    		ocv_percent = cal_ocv_percent_when_discharging(this, volt, curr);
    		pr_debug("[BAT][Gau][A66] %s(), discharging, ocv = %d\n", __func__, ocv_percent);
        }else{
            pr_err("[BAT][Gau][A66]:Error: volt not >0\n");
        }
	/*} else { // on charging
		this->gaugeCb->askSuspendCharging(this->gaugeCb);
		this->msParentGauge.ReadVoltCurr(
			&this->msParentGauge,
			TIMES_FOR_GENERAL_READ_ADC,
			&volt,
			&curr);
		ocv_percent = cal_ocv_percent_by_volt_directly(volt);
		this->gaugeCb->askResumeCharging(this->gaugeCb);
		pr_debug("[BAT][Gau][A66] %s(), charging, ocv = %d\n", __func__, ocv_percent);		
	}
    */
    //Eason always use V+IR  --------//
	wake_unlock(&this->calBatCapWlock);

       //ASUS_BSP  +++ Eason_Chang "add BAT info time"
        ReportTime();
       //ASUS_BSP  --- Eason_Chang "add BAT info time"
	
    if (volt >0 ){
        this->ocvPercent = ocv_percent;
    	this->gaugeCb->onCapacityReply(&this->msParentGauge, this->gaugeCb, ocv_percent, BAT_CAP_REPLY_OK);
    }else{
        this->gaugeCb->onCapacityReply(&this->msParentGauge, this->gaugeCb, ocv_percent, BAT_CAP_REPLY_ERR);
    }
// TODO: need to add BAT_CAP_REPLY_ERR case

	return;
}

/* cal_bat_100_per_ocv_work -
 */
static void cal_bat_100_per_ocv_work(struct work_struct *work)
{
	int volt;
	int curr;

	AXC_Gauge_A66 *this = container_of(work, AXC_Gauge_A66, calBat100PerOCVWorker.work);

	wake_lock(&this->calBat100PerOCVWlock);

	this->msParentGauge.ReadVoltCurr(
		&this->msParentGauge,
		TIMES_FOR_GENERAL_READ_ADC,
		&volt,
		&curr);

	wake_unlock(&this->calBat100PerOCVWlock);

	if ( 0 == this->Per100OCV) {
		this->Per100OCV = volt;
		pr_info("[BAT][Gau][A66] first time update 100 per OCV: %d\n", this->Per100OCV);
	} else {
//TODO: average 100 percent OCV
		this->Per100OCV = (this->Per100OCV + volt)/2;
		pr_info("[BAT][Gau][A66] cycle update 100 per OCV: %d\n", this->Per100OCV);
	}

	return;
}

/* write_resistor_cali -
 */
static void write_resistor_cali(AXC_Gauge_A66 *this ,int lnResisterValue)
{
	struct file *cfile;
	mm_segment_t orgfs;
	char delta_t_str[4];

	mytemp lnResisterCalibrationData;
	lnResisterCalibrationData.mnValue = (s16)lnResisterValue;
	pr_info( "[BAT]Save R-Value:%d\n", lnResisterValue);

	orgfs = get_fs();
	set_fs(KERNEL_DS);

	cfile = filp_open(BATTERY_RESISTOR_CALIBRATION_FILE_PATH, O_CREAT | O_RDWR | O_SYNC, 0666);
	if (!IS_ERR(cfile))
	{	
		delta_t_str[0]=lnResisterCalibrationData.mnChar[0];
		delta_t_str[1]=lnResisterCalibrationData.mnChar[1];
		cfile->f_op->write(cfile, delta_t_str, 2, &cfile->f_pos);
		if(filp_close(cfile, NULL))
       			pr_err("[BAT][Gau] fail to close resistor cali file in %s()\n", __func__);	
	}

	set_fs (orgfs);
	return;
}

/* cal_bat_resistor_work -
 */
static void cal_bat_resistor_work(struct work_struct *work)
{
	int volt, curr;
	int cnt = 0;

	AXC_Gauge_A66 *this = container_of(work, AXC_Gauge_A66, calBatResistorWorker.work);

	wake_lock(&this->calBatResistorWlock);
	// sometimes read current as 0 at the time of charger plug-out, so retry more times.
	do {
		this->msParentGauge.ReadVoltCurr(
			&this->msParentGauge, 
			TIMES_FOR_GENERAL_READ_ADC,
			&volt,
			&curr);
		cnt++;
		pr_info( "[BAT][Gau][A66]cal bat resistor, V=%d, C=%d, CNT=%d\n",volt, curr, cnt);
		msleep(1);
	} while ((curr <= 10) && (cnt < 15));

	wake_unlock(&this->calBatResistorWlock);

	if (this->Per100OCV < volt) {
		pr_err("[BAT][Gau][A66] error!! 100 per OCV less than volt \n");
		return;
	}

	if (curr <= 0) {
		pr_err("[BAT][Gau][A66] error!! unknown current \n");
		return;
	}

	this->resistorCali = (this->Per100OCV - volt)*1000/curr;
	pr_info( "[BAT][Gau][A66] cal resistor=%d\n", this->resistorCali);

	if (this->resistorCali > RESISTOR_MAX_VAL) {
		this->resistorCali = RESISTOR_MAX_VAL;
	} else if (this->resistorCali < RESISTOR_MIN_VAL) {
		this->resistorCali = RESISTOR_MIN_VAL;
	}

	pr_info("[BAT][Gau][A66] reset resistor cali:%d \n", this->resistorCali);

	write_resistor_cali(this, this->resistorCali);
	//TODO: Redesign calculate cali way, use average
	return;
}

/* read_bat_resistor_work - the work to read battery resistor from file */
static void read_bat_resistor_work(struct work_struct *work)
{
	AXC_Gauge_A66 *this = container_of(work, AXC_Gauge_A66, readBatResistorWorker.work);

	init_resistor_cali(this);

	return;		
}

/* AXC_Gauge_A66_AskCapacity -
 */
int AXC_Gauge_A66_AskCapacity(struct AXI_Gauge *gauge)
{
	AXC_Gauge_A66 *this = container_of(gauge, AXC_Gauge_A66, msParentGauge);
	BUG_ON(NULL == gauge);

	queue_delayed_work(this->calBatCapacityQueue, &this->calBatCapacityWorker , 0*HZ);

	return 0;
}

/* AXC_Gauge_A66_GetNextPollingInterval -
 */
int AXC_Gauge_A66_GetNextPollingInterval(struct AXI_Gauge *gauge)
{
	int interval;
	AXC_Gauge_A66 *this = container_of(gauge, AXC_Gauge_A66, msParentGauge);

	BUG_ON(NULL == gauge);

	if (this->ocvPercent < OCV_PER_SPEEDUP_UPDATE)
		interval = UPDATE_INERVAL_BAT_LOW_IRQ;
	else
		interval = UPDATE_INERVAL_DEFAULT;

	pr_debug("[BAT][Gau][A66] next polling interval:%d\n", interval);

	return interval;
}

/* AXC_Gauge_A66_NotifyCableInOut -
 */
int AXC_Gauge_A66_NotifyCableInOut(struct AXI_Gauge *gauge, bool hasCable)
{

	AXC_Gauge_A66 *this = container_of(gauge, AXC_Gauge_A66, msParentGauge);
	this->hasCable = hasCable;

	BUG_ON(NULL == gauge);

	pr_debug("[BAT][Gau][A66] notify calbe: %s \n", hasCable?"in":"out");
	pr_debug("[BAT][Gau][A66] isBatFull: %s \n", this->isBatFull?"yes":"no");

	if ((!this->hasCable) && (this->isBatFull)) {	// cable out
		pr_debug("[BAT][Gau][A66] calBatResistorWorker \n");
		queue_delayed_work(this->gaugeQueue, &this->calBatResistorWorker , 1.5*HZ);
	}

	return 0;
}

/* AXC_Gauge_A66_NotifyBatFullChanged -
 */
int AXC_Gauge_A66_NotifyBatFullChanged(struct AXI_Gauge *gauge, bool isFull)
{
	AXC_Gauge_A66 *this = container_of(gauge, AXC_Gauge_A66, msParentGauge);

	BUG_ON(NULL == gauge);

	if (isFull == this->isBatFull) {
		pr_info("[BAT][Gau][A66] %s(), same, need not to be changed \n", __FUNCTION__);
		return -1;
	}

	this->isBatFull = isFull;

	if (this->isBatFull) {
		queue_delayed_work(this->gaugeQueue, &this->calBat100PerOCVWorker , 0*HZ);		
		pr_debug("[BAT][Gau][A66] notify bat full\n");
	} else {
		pr_debug("[BAT][Gau][A66] notify bat changed to not full \n");
	}

	return 0;
}

/* AXC_Gauge_A66_IsBatExisted -
 */
bool AXC_Gauge_A66_IsBatExisted(struct AXI_Gauge *gauge)
{
	return true;
}

//Eason takeoff ADC read max & min +++
int findMax(int* Array)
{
   int Max;
   int i;
   int MaxIndex = 0;
  
   Max = Array[0];

   for(i=1;i<TIMES_FOR_GENERAL_READ_ADC;i++)
   {
	if(Array[i] > Max)
	{
		Max = Array[i];
                MaxIndex = i;
	}
   }
   
   return MaxIndex;
}

int findMin(int* Array)
{
   int Min;
   int i;
   int MinIndex = TIMES_FOR_GENERAL_READ_ADC-1;
  
   Min = Array[TIMES_FOR_GENERAL_READ_ADC-1];

   for(i=TIMES_FOR_GENERAL_READ_ADC-2; i>=0 ;i--)
   {
	if(Array[i] < Min)
	{
		Min = Array[i];
                MinIndex = i;
	}
   }
   
   return MinIndex;
}
//Eason takeoff ADC read max & min ---

extern void pet_watchdog(void);
/* AXC_Gauge_A66_ReadVoltCurrWithoutCali - To read voltage and current without
 * calibration for several times.
 */
static void AXC_Gauge_A66_ReadVoltCurrWithoutCali(
	AXI_Gauge *apGauge,
	unsigned int anTimes,
	int *volt,
	int *curr)
{
	int i = 0;
    int j = 0;
	int tmp_volt;
	int tmp_curr;
	int volt_sum = 0;
	int curr_sum = 0;
	bool curr_nega = false;
	int volArray[anTimes]; //Eason print VC
	int currArray[anTimes]; //Eason print VC
	//Eason takeoff ADC read max & min +++
	int volMax = 0;
    int volMin = TIMES_FOR_GENERAL_READ_ADC-1;
    int currMax = 0;
    int currMin = TIMES_FOR_GENERAL_READ_ADC-1;
	//Eason takeoff ADC read max & min ---

	BUG_ON(anTimes <= 0);
	BUG_ON(NULL == volt);
	BUG_ON(NULL == curr);

	while (i < anTimes)
	{
		AXC_Gauge_A66_ReadVoltageCurrentFromADC(apGauge, &tmp_volt, &tmp_curr);
		if ((tmp_volt < 2700)&&(j < 3)) {
			pr_err("[BAT][Gau][A66] read error volt: %d, try again \n", tmp_volt);
            j++;
            pet_watchdog();
			msleep(2);            
			continue;
		}

        if (j >= 3) {
            *volt = -1;
            *curr = -1;
            pr_err("[BAT][Gau][A66] read error over 3, return v,c as -1\n");
            return;
        }

		volt_sum += tmp_volt;
		curr_sum += tmp_curr;
		volArray[i] = tmp_volt; //Eason print VC
		currArray[i] = tmp_curr; //Eason print VC
		++i;
		msleep(2);
	}

    //Eason takeoff ADC read max & min +++
    volMax = findMax(volArray);
    volMin = findMin(volArray);
    currMax = findMax(currArray);
    currMin = findMin(currArray);
    
    volt_sum = volt_sum - volArray[volMax] - volArray[volMin];   
    curr_sum = curr_sum - currArray[currMax] - currArray[currMin]; 
    //Eason takeoff ADC read max & min ---

	if(curr_sum < 0){
		curr_nega = true ;
		curr_sum = curr_sum * (-1);
	}else{
		curr_nega = false;
	}

	*volt = volt_sum/(anTimes-2);//Eason takeoff ADC read max & min
	*curr = curr_sum/(anTimes-2);//Eason takeoff ADC read max & min

	if(true == curr_nega){
		*curr = (*curr)*(-1);
	}    

    //ASUS BSP Eason filter adc read current <= 15mA when doesn't has cable +++
    if( (*curr >= 0)&&(*curr <= 15)&&(false == report_BatteryService_If_HasCable()) )
    {
            printk("[BAT][Gau][A66] read error current<=15mA without cable:%d\n",*curr);
    	    *volt = -1;
            *curr = -1;
            pr_err("[BAT][Gau][A66] read error current<=15mA without cable, return v,c as -1\n");
            return;
    }
    //ASUS BSP Eason filter adc read current <= 15mA when doesn't has cable ---
    

	//Eason print VC +++
	/*for(i = 0; i < anTimes; i++){ 
		printk("%d,%d",volArray[i],currArray[i]);
	}*/
    //Eason takeoff ADC read max & min+++    
	if(TIMES_FOR_GENERAL_READ_ADC==anTimes){    
		printk("[BAT][Gau][A66]:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",volArray[0],volArray[1],volArray[2],volArray[3],volArray[anTimes-1],
			currArray[0],currArray[1],currArray[2],currArray[3],currArray[anTimes-1]);
		printk("[BAT][Gau][A66]:Vmax:%d,Vmin:%d,avgV:%d,Cmax:%d,Cmin:%d,avgC:%d\n",volArray[volMax],volArray[volMin],*volt,
            currArray[currMax],currArray[currMin],*curr);
        //ASUS_BSP Eason_Chang add event log +++
        ASUSEvtlog("[BAT][Gau][A66]:Vmax:%d,Vmin:%d,avgV:%d,Cmax:%d,Cmin:%d,avgC:%d\n",volArray[volMax],volArray[volMin],*volt,
            currArray[currMax],currArray[currMin],*curr);
        //ASUS_BSP Eason_Chang add event log ---
	}
    //Eason takeoff ADC read max & min---
	//Eason print VC ---

   //Eason if doesn't get correct ADC Vol&Curr at first update Cap show unknow status +++ 
   g_adc_get_correct_VolCurr = true;
   //Eason if doesn't get correct ADC Vol&Curr at first update Cap show unknow status ---    

	return;
}


/* AXC_Gauge_A66_ReadVoltCurr - To read voltage and current with
 * calibration for several times.
 */
static void AXC_Gauge_A66_ReadVoltCurr(
	AXI_Gauge *apGauge,
	unsigned int anTimes,
	int *volt,
	int *curr)
{
	/*AXC_Gauge_A66 *this = container_of(
		apGauge,
		AXC_Gauge_A66,
		msParentGauge);
    */
	AXC_Gauge_A66_ReadVoltCurrWithoutCali(apGauge, anTimes, volt, curr);

	//*volt += this->mnVoltageCalibration;
	//*curr += this->mnCurrentCalibration;

	return;
}


int AXC_Gauge_A66_GetType(AXI_Gauge *apGauge)
{
    AXC_Gauge_A66 *this = container_of(apGauge, AXC_Gauge_A66, msParentGauge);
    return this->mnType;
}

void AXC_Gauge_A66_SetType(AXI_Gauge *apGauge ,int anType)
{
    AXC_Gauge_A66 *this = container_of(apGauge, AXC_Gauge_A66, msParentGauge);
    this->mnType = anType;
}

void AXC_Gauge_A66_Constructor(AXI_Gauge *apGauge, int anType, AXI_Gauge_Callback *gaugeCb)
{
	AXC_Gauge_A66 *this = container_of(apGauge, AXC_Gauge_A66, msParentGauge);

	BUG_ON((NULL == gaugeCb) || (NULL == apGauge));

	CurrOCVtoPerTbl = MainSrcOCVtoPerTbl;

	this->gaugeCb = gaugeCb;

	wake_lock_init(&this->calBatCapWlock, WAKE_LOCK_SUSPEND, "calBatCapWlock");
	wake_lock_init(&this->calBat100PerOCVWlock, WAKE_LOCK_SUSPEND, "calBat100PerOCVWlock");
	wake_lock_init(&this->calBatResistorWlock, WAKE_LOCK_SUSPEND, "calBatResistorWlock");

	INIT_DELAYED_WORK(&this->calBatCapacityWorker, cal_bat_capacity_work) ;
	INIT_DELAYED_WORK(&this->calBat100PerOCVWorker, cal_bat_100_per_ocv_work) ;
	INIT_DELAYED_WORK(&this->calBatResistorWorker, cal_bat_resistor_work) ;
	INIT_DELAYED_WORK(&this->readBatResistorWorker, read_bat_resistor_work) ;

	this->calBatCapacityQueue = create_singlethread_workqueue("CalBatCapacityWorker");
	if (!this->calBatCapacityQueue)
		pr_err("[BAT][Gau][A66]error!! when create calBatCapacityQueue\n");

	this->gaugeQueue = create_singlethread_workqueue("GaugeWorker");	
	if (!this->gaugeQueue)
		pr_err("[BAT][Gau][A66]error!! when create gaugeQueue\n");

	this->msParentGauge.askCapacity = AXC_Gauge_A66_AskCapacity;
	this->msParentGauge.getNextPollingInterval = AXC_Gauge_A66_GetNextPollingInterval;
	this->msParentGauge.notifyCableInOut = AXC_Gauge_A66_NotifyCableInOut;
	this->msParentGauge.notifyBatFullChanged = AXC_Gauge_A66_NotifyBatFullChanged;
	this->msParentGauge.isBatExisted = AXC_Gauge_A66_IsBatExisted;
	this->msParentGauge.SetType = AXC_Gauge_A66_SetType;
	this->msParentGauge.GetType = AXC_Gauge_A66_GetType;
	this->msParentGauge.ReadVoltCurr = AXC_Gauge_A66_ReadVoltCurr;
	this->msParentGauge.ReadVoltCurrWithoutCali = AXC_Gauge_A66_ReadVoltCurrWithoutCali;

	this->mbInited = true;
}


