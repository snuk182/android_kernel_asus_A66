#include <linux/kernel.h>
#include <linux/mutex.h>
#include "axi_cap_filter.h"
#include "axc_cap_filter_a66.h"

/* BAT_LIFE - different battery life definition */
#define BAT_LIFE_TO_SHUTDOWN 0
#define BAT_LIFE_ONE 1
#define BAT_LIFE_FIVE 5
#define BAT_LIFE_TWO  2
#define BAT_LIFE_FULL 100
#define BAT_LIFE_BEFORE_FULL 99
#define BAT_LIFE_BEFORE_FULL_ONE_UNIT 83

#define SECOND_OF_HOUR	3600
int ignore = 0;
/* g_bat_life_after_dot - remain battery life after dot, used for more accuracy */
static int g_bat_life_after_dot = 0;

//Eason add fasterLeverage judge+++ 
static int g_do_fasterLeverage_count = 0; 
//Eason add fasterLeverage judge---  
//Eason:In  phone call suspend, use 200mA do fasterLeverage+++
extern int g_flag_csvoice_fe_connected;
//Eason:In  phone call suspend, use 200mA do fasterLeverage---

/* eval_bat_life_when_discharging - Evaluate conservative battery life when discharging.
 * Use (maximum consuming current * update interval) as one factor.
 */
static int eval_bat_life_when_discharging(
	int nowCap, int lastCap, int maxMah, int interval, int batCapMah)
{
	int bat_life = 0;
	int var_drop_base=0;
	int drop_val = lastCap - nowCap;

	pr_info( "[BAT][Fil]%s(), drop_val:%d\n",
		__func__,
		drop_val);

	if (drop_val > 0) {
		int max_predict_drop_val = 0;
		int finetune_max_predict_drop_val = 0;
        int fasterLeverage_drop_val = 0;

		if (interval > 0) {
			/* if interval is more than 108sec, max_predict_drop_val will be more than 1 */
			//Eason :when  low bat Cap draw large current  +++
			//max_predict_drop_val = (maxMah*100/batCapMah)*interval/SECOND_OF_HOUR;
			max_predict_drop_val = (maxMah*100*100/batCapMah)*interval/SECOND_OF_HOUR/100;//fix  suspend  (10*100/2100)will be 0
			if(nowCap<10)
			{
            			//fasterLeverage_drop_val = (1400*100/batCapMah)*interval/SECOND_OF_HOUR;
            			fasterLeverage_drop_val = (1400*100*100/batCapMah)*interval/SECOND_OF_HOUR/100;
			}else{
				//fasterLeverage_drop_val = (900*100/batCapMah)*interval/SECOND_OF_HOUR;
				fasterLeverage_drop_val = (900*100*100/batCapMah)*interval/SECOND_OF_HOUR/100;
			}
			//Eason :when  low bat Cap draw large current  ---
			//Eason:prevent in unattend mode mass drop+++
			if(10==maxMah)
			{
				//Eason:In  phone call suspend, use 200mA do fasterLeverage+++
				/*
				*	extern int g_flag_csvoice_fe_connected, 
				*     - 0: not in phone call
				*     - 1: in phone call
				*/
				if(0 == g_flag_csvoice_fe_connected)
				{
					fasterLeverage_drop_val = (30*100*100/batCapMah)*interval/SECOND_OF_HOUR/100;
				}else{
					fasterLeverage_drop_val = (200*100*100/batCapMah)*interval/SECOND_OF_HOUR/100;
				}
				//Eason:In  phone call suspend, use 200mA do fasterLeverage---
			}
			//Eason:prevent in unattend mode mass drop---
			/* variable drop base for faster leverage true batter life */
			if ((drop_val/2) > 0) {
				var_drop_base = drop_val/2;
			} else {
				var_drop_base = 1;
			}

            //Eason add fasterLeverage judge+++  
            if((drop_val > max_predict_drop_val) && (g_do_fasterLeverage_count < 3)){
                g_do_fasterLeverage_count++; 
            }else if((drop_val <= max_predict_drop_val) && (g_do_fasterLeverage_count > 0)){    
                g_do_fasterLeverage_count--;
            }

		if( (2<=g_do_fasterLeverage_count)&&(nowCap<10) ){
			finetune_max_predict_drop_val = fasterLeverage_drop_val;
            }else if(3 == g_do_fasterLeverage_count){
			    //finetune_max_predict_drop_val = max(var_drop_base, max_predict_drop_val);
                finetune_max_predict_drop_val = fasterLeverage_drop_val;
            }else{
                finetune_max_predict_drop_val = max_predict_drop_val;
            }
            //Eason add fasterLeverage judge---
            
			bat_life = lastCap - min(drop_val, finetune_max_predict_drop_val);
		} else {
			//bat_life = lastCap - drop_val;
			bat_life = lastCap;
			pr_err( "[BAT][Fil]Error!!! %s(), interval < 0\n",
					__func__);
		}

		pr_info( "[BAT][Fil] interval:%d, var_drop_base:%d, max_predict_drop_val:%d, fasterLeverage_drop_val:%d, finetune_max_predict_drop_val:%d, count:%d \n",
			interval,
			var_drop_base,
			max_predict_drop_val,
			fasterLeverage_drop_val,
			finetune_max_predict_drop_val,
			g_do_fasterLeverage_count);
	} else {
		bat_life = lastCap;

        if(g_do_fasterLeverage_count > 0)
        {
            g_do_fasterLeverage_count--;
        }
                
		if (drop_val < 0) {
			pr_info( "[BAT][Fil] Error!!! %s(), drop val less than 0. count:%d\n", __func__,g_do_fasterLeverage_count);
		}
	}

	return bat_life;
}



/* eval_bat_life_when_charging - Evaluate conservative battery life when charging.
 * Use (maximum charging current * update interval) as one factor.
 */
static int eval_bat_life_when_charging(
	int nowCap, int lastCap, int maxMah, int interval, int batCapMah)
{
	int bat_life = 0;
	int rise_val = nowCap - lastCap; 
	int tmp_val_after_dot = 0;

	if (rise_val > 0) {
		long max_bat_life_rise_val = 0;

		if (interval > 0) {
			/* if interval is more than 108sec, max_predict_drop_val will be more than 1 */
			max_bat_life_rise_val = (maxMah*100/batCapMah)*interval/SECOND_OF_HOUR;
			/* to calculate the first number after the decimal dot for more accuracy.*/
			tmp_val_after_dot = ((10*maxMah*100/batCapMah)*interval/SECOND_OF_HOUR)%10;
			pr_debug( "[BAT][Fil]%s(), tmp_val_after_dot:%d\n",
				__func__,
				tmp_val_after_dot);	
			g_bat_life_after_dot += tmp_val_after_dot;
			if ((g_bat_life_after_dot/10) >= 1) {
				pr_debug( "[BAT][Fil]%s(), g_bat_life_after_dot:%d\n",
					__func__,
					g_bat_life_after_dot);
				max_bat_life_rise_val += (g_bat_life_after_dot/10);
				g_bat_life_after_dot = g_bat_life_after_dot%10;
			}
			bat_life = lastCap + min(rise_val, (int)max_bat_life_rise_val);
            //TO DO ...if interval is too big...will get a negative value for capacity returned...
		} else {
			bat_life = lastCap + rise_val;
			pr_err("[BAT][Fil]Error!!! %s(), interval < 0\n",
					__func__);
		}

		pr_debug( "[BAT][Fil]%s(), rise_val:%d, interval:%d, max_bat_life_rise_val:%d, bat_life:%d\n",
			__func__,
			rise_val,
			(int)interval,
			(int)max_bat_life_rise_val,
			bat_life);
	} else {
		bat_life = lastCap;
		pr_debug( "[BAT][Fil]%s(), keep the same bat_life:%d as before\n",
			__func__,
			bat_life);

		if (rise_val < 0) {
			pr_err("[BAT][Fil] Error!!! %s(), rise val less than 0.\n", __func__);
		}
	}
	return bat_life;
}


/* update_bat_info_for_speical_case - Update battery info for some special cases.
 * Such as when to update battery life to 100.
 */
static int update_bat_info_for_speical_case(
	int bat_life,
	bool hasCable,
	bool isCharing,	
	bool isBatFull,
	bool isBatLow)
{
	int final_bat_life;

	if ((!isBatLow) && (bat_life <= 0)) {
		pr_info( "[BAT][Fil]%s(), bat not low, but get cap as 0, return 1\n", __func__);
		final_bat_life = BAT_LIFE_ONE;
		return final_bat_life;
	}

	if (bat_life >= BAT_LIFE_FULL) {
		if(false == isBatFull)
		//if(this->mpCharger->IsCharging(this->mpCharger))
		{
			pr_debug( "[BAT][Fil]%s(), Still in charging status, so update bat life to 99\n", __func__);
			final_bat_life = BAT_LIFE_BEFORE_FULL;
			return final_bat_life;
		} else {
			final_bat_life = BAT_LIFE_FULL;
			return final_bat_life;
		}
	} 


	if (bat_life >= BAT_LIFE_BEFORE_FULL_ONE_UNIT) {
		//Need to know if being charging or not	
		if (hasCable) {
			if(true == isBatFull){
				//if(this->mpCharger->IsCharging(this->mpCharger)) {
				pr_debug( "[BAT][Fil]%s(), keep bat life to 100\n", __func__);
				final_bat_life = BAT_LIFE_FULL;
				return final_bat_life;
			} else {
				pr_debug( "[BAT][Fil]%s(), still in charging\n", __func__);
				final_bat_life = bat_life;
				return final_bat_life;
			}
		}
	}

	final_bat_life = bat_life;

	return final_bat_life;
}

//#define BAT_LOW_LIFE_MAP_START 	10

//static int bat_low_life_map_tbl[BAT_LOW_LIFE_MAP_START + 1] = {1, 1, 1, 1, 2, 3, 4, 6, 7, 8, 10};

int AXC_Cap_Filter_A66_GetType(struct AXI_Cap_Filter *apCapFilter)
{
	AXC_Cap_Filter_A66 *this = container_of(apCapFilter, AXC_Cap_Filter_A66, parentCapFilter);
	return this->filterType;
}


int AXC_Cap_Filter_A66_FilterCapacity(struct AXI_Cap_Filter *apCapFilter, int nowCap, int lastCap, bool hasCable, bool isCharing, bool isBatFull, bool isBatLow, int maxMah, int interval)
{
	int bat_life;
	AXC_Cap_Filter_A66 *this = container_of(apCapFilter, AXC_Cap_Filter_A66, parentCapFilter);
	
	/* the criteria to set bat life as 0 to shutdown */	
	if (isBatLow && (nowCap <= 0) && (lastCap <= 3)) {
		pr_info("[BAT][Fil]%s(), bat low and cap <= 3, shutdown!! \n", __func__);
		return BAT_LIFE_TO_SHUTDOWN;
	}

	/* battery low life remap - avoid user not to see battery low warning notification */
/*	if (nowCap <= BAT_LOW_LIFE_MAP_START) {
		bat_life = bat_low_life_map_tbl[nowCap];
		pr_info("[BAT][Fil]%s(), remap cap from %d to %d \n", __func__, nowCap, bat_life);
		return bat_life;
	}
*/
	if (hasCable) {	// has cable
		bat_life = eval_bat_life_when_charging(nowCap, lastCap, maxMah, interval, this->capMah);
	} else {	// no cable
		bat_life = eval_bat_life_when_discharging(nowCap, lastCap, maxMah, interval, this->capMah);
	}
	//++++workaround when charging ,capacity decrease
	if(bat_life < lastCap && isCharing==1 && ignore==0)
	{
		printk("++++workaround usb plug in ,capacity decrease. true filter capacity = %d",bat_life);
		bat_life=lastCap;
		ignore++;
		printk("++++workaround usb plug in ,capacity decrease. now filter capacity = %d",bat_life);
	}
	if(bat_life >lastCap && ignore !=0)
	{
		ignore = 0;
	}
	//----workaround when charging ,capacity decrease
	bat_life = update_bat_info_for_speical_case(bat_life, hasCable, isCharing, isBatFull, isBatLow);

	pr_debug("[BAT][Fil]%s(), filter cap:%d \n", __func__, bat_life);
	return bat_life;

}

extern void AXC_Cap_Filter_A66_Constructor(AXI_Cap_Filter *apCapFilter, int filterType, int capMah)
{

	AXC_Cap_Filter_A66 *this = container_of(apCapFilter, AXC_Cap_Filter_A66, parentCapFilter);
	this->capMah = capMah;
	this->filterType = filterType;
	this->parentCapFilter.getType = AXC_Cap_Filter_A66_GetType;
	this->parentCapFilter.filterCapacity = AXC_Cap_Filter_A66_FilterCapacity;


}


