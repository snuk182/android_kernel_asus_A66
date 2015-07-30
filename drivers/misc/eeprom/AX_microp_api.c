#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/microp.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#include <linux/microp_notify.h>
#include <linux/mutex.h>
#ifdef CONFIG_ASUSEC
#include <linux/asus_ec_info.h>
#endif
extern int uP_nuvoton_read_reg(int cmd, void *data);
extern int uP_nuvoton_write_reg(int cmd, void *data);
extern void TriggerPadStationPowerOff(void);
extern int isFirmwareUpdating(void);

extern unsigned int g_b_isP01Connected;
extern unsigned int g_microp_ver;
extern int micropSendNotify(unsigned long val);
extern struct mutex microp_mutex_lock;
extern struct mutex vote_mutex_lock;
extern uint8_t votesScalar[2];
extern unsigned int g_microp_ver;
extern unsigned int g_ldrom_ver;




/*
*       Check the status of P01 connectness
*       return value: 1: P01 connected
*/

int AX_MicroP_IsP01Connected(void){
        return (g_b_isP01Connected==1)?1:0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_IsP01Connected);



int AX_MicroP_isFWSupportP02(void){
        if(AX_MicroP_IsP01Connected() && g_microp_ver >=7)
            return 1;
        return 0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_isFWSupportP02);


int AX_MicroP_HW_isER3(void){
        if(AX_MicroP_IsP01Connected() && g_ldrom_ver >=0x8006)
            return 1;
        return 0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_HW_isER3);



/*
*     Input Pin definition on microp
*     IN_DOCK_ACOK => high: inserted, low: removed
*	IN_DOCK_INT => low active
*	IN_ALS_INT => low active
*	IN_HP_IN => low active
*	IN_VOL_DOWN => low: pressed
*	IN_VOL_UP => low: pressed
*	IN_SUS_SW_R => low: pwrbtn pressed
*	IN_AC_USB_IN_R => high: cable in, low: cable out
*	IN_PWR_ON_OFF_R =>
*	IN_PB_DIS_uP => high: battery power bad
*	IN_DOCK_IN_R => low active. low: dock inerted
*	IN_O_LID_R => low active
*	IN_LOUT_DET_R => low active
*/




/*
*       Check the status of Headphone if it is inserted
*       return value: 0: plugged out, 1: plugged in, <0: err
*/

int AX_MicroP_IsHeadPhoneIn(void){
        int pin=-1;
        pin=AX_MicroP_getGPIOPinLevel(IN_HP_IN);
        if(pin<0)
            return pin;
        return (pin==0)?1:0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_IsHeadPhoneIn);



/*
*       Check the status of AC/USB if it is inserted
*       return value: 0: plugged out, 1: plugged in, <0: err
*/

int AX_MicroP_IsACUSBIn(void){
        int pin=-1;
        pin=AX_MicroP_getGPIOPinLevel(IN_AC_USB_IN_R);
        if(pin<0)
            return pin;
        return (pin==1)?1:0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_IsACUSBIn);


#ifdef CONFIG_ASUSEC


/*
*       Check the status of Dock if it is inserted
*       return value: 0: plugged out, 1: plugged in, <0: err
*/

int AX_MicroP_IsECDockIn(void){
        int pin=-1;
        pin=AX_MicroP_getGPIOPinLevel(IN_DOCK_IN_R);
        if(pin<0)
            return pin;
        return (pin==0)?1:0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_IsECDockIn);



/*
*       Check the status of Dock battery if it is power-bad
*       return value: 1: PowerBad, <0: err
*/

int AX_MicroP_Is_ECBattPowerBad(void){
        int pin=-1;
        pin=AX_MicroP_getGPIOPinLevel(IN_PB_DIS_uP);
        if(pin<0)
            return pin;
        return (pin==1)?1:0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_Is_ECBattPowerBad);



/*
*       Check the status of Dock Ext. Power if ext power is in
*       return value: 1: PowerBad, <0: err
*/
int AX_MicroP_Is_ECExtPowerCableIn(void){
        int pin=-1;
        pin=AX_MicroP_getGPIOPinLevel(IN_DOCK_ACOK);
        if(pin<0)
            return pin;
        return (pin==1)?1:0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_Is_ECExtPowerCableIn);


#endif


/*
*   @AX_MicroP_get_ChargingStatus
*  input: target
*           0: p01 battery
*           1: dock battery
*    return: 0 for 'no charging', 1 for 'charging', 2 for 'charged full', <0 value means something error
*/

int AX_MicroP_get_ChargingStatus(int target){
	int regval=0;
       int ret=0;
       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(target==Batt_P01){
                // redundant read to wakeup microp from power-down mode and trigger microp to read h/w gauge again
                ret=uP_nuvoton_read_reg(MICROP_CHARGING_STATUS, &regval); 
        
                ret=uP_nuvoton_read_reg(MICROP_CHARGING_STATUS,&regval);
                if(ret <= 0 || regval==255)
                            regval=P01_CHARGING_ERR;
                else if(regval==0)
                            regval=P01_CHARGING_NO;
                else if(regval==1){
                            if(AX_MicroP_get_USBDetectStatus(Batt_P01)==P01_CABLE_CHARGER)
                                    regval=P01_CHARGING_ONGOING;
                            else
                                    regval=P01_CHARGING_NO;
                }
                else if(regval==2)
                            regval=P01_CHARGING_FULL;
                    
        }
#ifdef CONFIG_ASUSEC        
       else if(target==Batt_Dock){
                regval=asusec_dock_battery_charging_status();
                if(regval==EC_CHARGING_ONGOING)
                            regval=P01_CHARGING_ONGOING;
                else if(regval==EC_CHARGING_FULL)
                            regval=P01_CHARGING_FULL;
                else if(regval==EC_CHARGING_NO)
                            regval=P01_CHARGING_NO;
                else if(regval==EC_CHARGING_ERR)
                            regval=P01_CHARGING_ERR;
        }
#endif
       else{
                printk(KERN_ERR "%s: known target %d\r\n", __FUNCTION__, target);
       }
       
	return regval;
}

EXPORT_SYMBOL_GPL(AX_MicroP_get_ChargingStatus);

/*
*   @AX_MicroP_get_USBDetectStatus
*  input: target
*           0: p01 battery
*           1: dock battery
*    return: 0 for 'no charger/usb', 1 for 'charger', 2 for 'USB', <0 value means something error
*
*/ 

int AX_MicroP_get_USBDetectStatus(int target){
    	int regval=0;
       int ret=0;
       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(target==Batt_P01){
                ret=uP_nuvoton_read_reg(MICROP_USB_DET,&regval);
                if(ret <= 0 || regval==255)
                        regval=P01_CABLE_UNKNOWN;
                else if(regval==0){
                        regval=P01_CABLE_NO;
#ifdef CONFIG_ASUSEC        
                        // it might insert keyboard dock. if yes, we regard it as charger
                        if(AX_MicroP_IsECDockIn())
                                regval=P01_CABLE_CHARGER;    
#endif
                }
                else if(regval==1)
                        regval=P01_CABLE_CHARGER;
                else if(regval==2)
                        regval=P01_CABLE_USB;
                    
        }
#ifdef CONFIG_ASUSEC        
       else if(target==Batt_Dock){
                regval=asusec_dock_cable_status();
                if(regval==EC_CABLE_NO)
                            regval=P01_CABLE_NO;
                else if(regval==EC_CABLE_AC)
                            regval=P01_CABLE_CHARGER;
                else if(regval==EC_CABLE_USB)
                            regval=P01_CABLE_USB;
                else 
                            regval=P01_CABLE_UNKNOWN;
        }
#endif
       else{
                printk(KERN_ERR "%s: known target %d\r\n", __FUNCTION__, target);
       }

       return regval;
}
EXPORT_SYMBOL_GPL(AX_MicroP_get_USBDetectStatus);



/*
*  GPIO direct control
*  @ AX_MicroP_getGPIOPinLevel
*  input: 
            - pinID
*  return: 0 for low, 1 for high, <0 value means something error
*
*/


int AX_MicroP_getGPIOPinLevel(int pinID){
	int regval=0;
       int ret=0;
       int gpiolevel=0;
       unsigned int sel_offset=1<<pinID;
       pr_debug("[MicroP] try to get GPIO pin=%d\r\n",pinID);
       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }
	ret=uP_nuvoton_read_reg(MICROP_GPIO_INPUT_LEVEL,&regval);
       if(ret > 0){
                gpiolevel=(regval & sel_offset)?1:0;
       }

	return ((ret < 0)?ret:gpiolevel);
}

EXPORT_SYMBOL_GPL(AX_MicroP_getGPIOPinLevel);
/*
*  @ AX_MicroP_setGPIOOutputPin
*  input: 
*           - pinID
*           - level: 0 for low, 1 for high
*  return: the status of operation. 0 for success, <0 value means something error
*/



int AX_MicroP_setGPIOOutputPin(int pinID, int level){
	int regval=0;
       unsigned int sel_offset=0;
       int ret=0;

       printk("[MicroP] set GPIO pin=%d, level=%d\r\n",pinID, level);

       if(g_microp_ver >= 0x20){
                    if(AX_MicroP_IsP01Connected()==0){
                        printk("%s: P01 removed\r\n",__FUNCTION__);
                        return -1;
                   }

                    if(pinID >= OUT_uP_SIZE){
                            printk("%s: index error =%d\r\n",__FUNCTION__, pinID);                   
                            return -1;
                    }

                    sel_offset=1<<pinID;
                    if(level)
                            ret=uP_nuvoton_write_reg(MICROP_GPIO_OUTPUT_BIT_SET,&sel_offset);
                    else
                            ret=uP_nuvoton_write_reg(MICROP_GPIO_OUTPUT_BIT_CLR,&sel_offset);
       }
       else{
                   mutex_lock(&microp_mutex_lock);
                   if(AX_MicroP_IsP01Connected()==0){
                        printk("%s: P01 removed\r\n",__FUNCTION__);
                        mutex_unlock(&microp_mutex_lock);
                        return -1;
                   }

                    if(pinID >= OUT_uP_SIZE){
                            printk("%s: index error =%d\r\n",__FUNCTION__, pinID);
                            mutex_unlock(&microp_mutex_lock);
                            return -1;
                    }

                    sel_offset=1<<pinID;



                   
            	    ret=uP_nuvoton_read_reg(MICROP_GPIO_OUTPUT_LEVEL,&regval);
                   if(ret > 0){
                    	if(level)
                    		regval=regval | sel_offset;
                    	else
                    		regval=regval & ~sel_offset;

                    	ret=uP_nuvoton_write_reg(MICROP_GPIO_OUTPUT_LEVEL,&regval);
                   }

                   mutex_unlock(&microp_mutex_lock);
        }
	return ((ret < 0)?ret:0);

}

EXPORT_SYMBOL_GPL(AX_MicroP_setGPIOOutputPin);


/*
*  @ AX_MicroP_getGPIOOutputPinLevel
*  input:
*           - pinID

*  return: 0 for low, 1 for high, <0 value means something error
*/

int AX_MicroP_getGPIOOutputPinLevel(int pinID){
	int regval=0;
	int ret=0;
	int gpiolevel=0;
	unsigned int sel_offset=1<<pinID;
       pr_debug("[MicroP] try to get GPIO_OutPut pin=%d\r\n",pinID);

       if(g_microp_ver < 0x20)
               mutex_lock(&microp_mutex_lock);

        if(AX_MicroP_IsP01Connected()==0){
		printk("%s: P01 removed\r\n",__FUNCTION__);

               if(g_microp_ver < 0x20)
                      mutex_unlock(&microp_mutex_lock);
               
		return -1;
	}

	ret=uP_nuvoton_read_reg(MICROP_GPIO_OUTPUT_LEVEL,&regval);
	if(ret > 0){
		gpiolevel=(regval & sel_offset)?1:0;
	}

       if(g_microp_ver < 0x20)
               mutex_unlock(&microp_mutex_lock);
       
	return ((ret < 0)?ret:gpiolevel);
}

EXPORT_SYMBOL_GPL(AX_MicroP_getGPIOOutputPinLevel);


/*
*  @AX_MicroP_enableInterrupt
*  input: 
*            - intrpin: input pin id
*            -  enable: 0 for 'disable', 1 for 'enable'
*  return: 0 for success, <0 value means something error
*/

int AX_MicroP_enablePinInterrupt(unsigned int pinID, int enable){

	unsigned int regval=0;
       int ret=0;
       printk("[MicroP] enable Pin Intr pin=0x%x, enable=%d\r\n",pinID, enable);


       if(g_microp_ver >= 0x20){
                       if(AX_MicroP_IsP01Connected()==0){
                            printk("%s: P01 removed\r\n",__FUNCTION__);
                            return -1;
                       }

                        if(enable){
                                ret=uP_nuvoton_write_reg(MICROP_INTR_EN_BIT_SET,&pinID);
                        }
                        else{
                                ret=uP_nuvoton_write_reg(MICROP_INTR_EN_BIT_CLR,&pinID);                                                
                        }
       }
       else{
                       if(AX_MicroP_IsP01Connected()==0){
                            printk("%s: P01 removed\r\n",__FUNCTION__);
                            return -1;
                       }
                       mutex_lock(&microp_mutex_lock);

                       ret=uP_nuvoton_read_reg(MICROP_INTR_EN,&regval);
                       if(ret > 0){
                                if(enable)
                                        regval=regval | pinID;
                                else
                                        regval=regval & ~pinID;

                                ret=uP_nuvoton_write_reg(MICROP_INTR_EN,&regval);
                       }
                       mutex_unlock(&microp_mutex_lock);
        }
	return ((ret < 0)?ret:0);

}

EXPORT_SYMBOL_GPL(AX_MicroP_enablePinInterrupt);





/*
*  @AX_MicroP_readBattCapacity
*  input: target
*           0: p01 battery
*           1: dock battery
*  return: value >=0: success, value < 0: error
*/



extern int asusec_dock_battery_callback(void);
extern int isAlwaysPowerOnMicroP(void);
int AX_MicroP_readBattCapacity(int target){
        static int dock_cap=0;
        static int pad_cap=0;
        
        int flag=0;

        if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            goto failed;
       }


        if(target==1){
#ifdef CONFIG_ASUSEC
                    dock_cap=asusec_dock_battery_callback();
#endif
            
         }
        else{

                    if(isFirmwareUpdating()){
                            printk("%s: P01 updating, not support read cap now\r\n",__FUNCTION__);
                            goto failed;
                    }

                    // redundant read to wakeup microp from power-down mode and trigger microp to read h/w gauge again
                    flag=uP_nuvoton_read_reg(MICROP_GAUGE_CAP, &pad_cap); 

                    flag=uP_nuvoton_read_reg(MICROP_GAUGE_CAP,&pad_cap);
                    if(flag < 0){
                            printk("%s: read cap fail due to i2c flag=%d\r\n",__FUNCTION__, flag);
                            goto failed;
                    }
                    if(pad_cap <=3 && !isAlwaysPowerOnMicroP()){
                            TriggerPadStationPowerOff();
                    }
        }
        printk("%s: target[%s]=%d\r\n", __FUNCTION__, (target==1)?"dock":"p01",(target==1)?dock_cap:pad_cap);

failed:        
        return (target==1)?dock_cap:pad_cap;
}

/*
*  @get_MicroP_HUB_SLEEP_STATUS
*  return: value = 1: turn on, value = 0: turn off
*/

int get_MicroP_HUB_SLEEP_STATUS(void){
	return AX_MicroP_getGPIOOutputPinLevel(OUT_uP_HUB_SLEEP);
}

EXPORT_SYMBOL_GPL(get_MicroP_HUB_SLEEP_STATUS);


#ifdef CONFIG_ASUSEC

int get_EC_DOCK_IN_STATUS(void){
    return AX_MicroP_getGPIOPinLevel(IN_DOCK_IN_R);
};

EXPORT_SYMBOL_GPL(get_EC_DOCK_IN_STATUS);

int get_EC_AP_WAKE_STATUS(void){
    return AX_MicroP_getGPIOPinLevel(IN_DOCK_INT);
};

EXPORT_SYMBOL_GPL(get_EC_AP_WAKE_STATUS);

int set_EC_REQUEST_VALUE(int value){
	return AX_MicroP_setGPIOOutputPin(OUT_uP_EC_REQUEST,value);
};

EXPORT_SYMBOL_GPL(set_EC_REQUEST_VALUE);

int get_EC_HALL_SENSOR_STATUS(void){
	return AX_MicroP_getGPIOPinLevel(IN_O_LID_R);
};

EXPORT_SYMBOL_GPL(get_EC_HALL_SENSOR_STATUS);


int EC_Init_Complete(void){
        printk("[MicroP] EC -> MicroP: Init Ready +++\r\n");
        micropSendNotify(DOCK_INIT_READY);
        printk("[MicroP] EC -> MicroP: Init Ready ---\r\n");
        return 0;
}
EXPORT_SYMBOL_GPL(EC_Init_Complete);

int EC_Get_EXT_POWER_PLUG_IN_Ready(void){
        printk("[MicroP][%s] +++\r\n", __FUNCTION__);
        micropSendNotify(DOCK_EXT_POWER_PLUG_IN_READY);
        printk("[MicroP][%s] ---\r\n", __FUNCTION__);
        return 0;
}
EXPORT_SYMBOL_GPL(EC_Get_EXT_POWER_PLUG_IN_Ready);

int EC_Get_EXT_POWER_PLUG_OUT_Ready(void){
        printk("[MicroP][%s] +++\r\n", __FUNCTION__);
        micropSendNotify(DOCK_EXT_POWER_PLUG_OUT_READY);
        printk("[MicroP][%s] ---\r\n", __FUNCTION__);
        return 0;
}
EXPORT_SYMBOL_GPL(EC_Get_EXT_POWER_PLUG_OUT_Ready);

int EC_Get_DOCK_BATTERY_POWER_BAD_READY(void){
        printk("[MicroP][%s] +++\r\n", __FUNCTION__);
        micropSendNotify(DOCK_BATTERY_POWER_BAD_READY);
        printk("[MicroP][%s] ---\r\n", __FUNCTION__);
        return 0;
}
EXPORT_SYMBOL_GPL(EC_Get_DOCK_BATTERY_POWER_BAD_READY);

/*
*       Check the status of Dock connectness
*       return value: 1: Dock connected and ready
*/
int AX_MicroP_IsDockReady(void){
	return asusec_Dock_Ready_status()?1:0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_IsDockReady);

#endif




int getScalarVote(void){
/*

    printk("[GetVote] BL: %d, Audio: %d\r\n",votesScalar[0],votesScalar[1]);

    return ((votesScalar[0] && votesScalar[1])?1:0);
*/
    return 0;
}

int checkScalarVote(void){

/*

       printk("[CheckVote] BL: %d, Audio: %d\r\n",votesScalar[0],votesScalar[1]);
       if(votesScalar[0]==1 && votesScalar[1]==1){ // set bat_pic 1 to make scalar sleep
                if(AX_MicroP_getGPIOOutputPinLevel(OUT_uP_BAT_L_PIC)==0){
                        printk("[Microp] suspend scalar\r\n");
                        AX_MicroP_setGPIOOutputPin(OUT_uP_BAT_L_PIC, 1);
                }
        }
        else{
                if(AX_MicroP_getGPIOOutputPinLevel(OUT_uP_BAT_L_PIC)==1){
                        printk("[Microp] wakeup scalar\r\n");
                        AX_MicroP_setGPIOOutputPin(OUT_uP_BAT_L_PIC, 0);
                           // migth need a delay for scalar wakeup
                }
                
        }
*/
    return 0;
    
}
int blVoteScalarSleepEntry(int vsleep){
    /*
       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       printk("[BL] vote: %d\r\n", vsleep);
       if(votesScalar[0]==vsleep)   //vote for same result, skip 
            return 0;

       mutex_lock(&vote_mutex_lock);       
       votesScalar[0]=vsleep;
       checkScalarVote();
        mutex_unlock(&vote_mutex_lock);
*/        
        return 0;

}


int audioVoteScalarSleepEntry(int vsleep){
    /*
       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }
       printk("[Audio] vote: %d\r\n", vsleep);       
       
       if(votesScalar[1]==vsleep)   //vote for same result, skip 
            return 0;

       mutex_lock(&vote_mutex_lock);
       
        votesScalar[1]=vsleep;
       checkScalarVote();

        mutex_unlock(&vote_mutex_lock);
*/        
        return 0;
}
