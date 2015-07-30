/*
        Maxim 8934 Charger IC Implementation

*/
/*
	Definition on TD
	GPIO57	GPIO58	N/A
	PEN1	PEN2	USUS
	H	x	x	3000/Rpset
	L	H	L	475mA
	L	L	L	95mA
	L	x	H	0mA(UsbSuspend)
*/
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include "axc_Maxim8934Charger.h"
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/mfd/pm8xxx/pm8921-charger.h>
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp_notify.h>
#include <linux/microp_api.h>
#include <linux/platform_device.h>
#endif /*CONFIG_EEPROM_NUVOTON */


#ifdef CONFIG_CHARGER_MODE
extern int g_chg_present;
extern char g_CHG_mode;
#endif

//Eason takeoff Battery shutdown +++
extern bool g_AcUsbOnline_Change0;
//Eason takeoff Battery shutdown ---
//Eason : when thermal too hot, limit charging current +++ 
bool g_thermal_limit = false;
bool g_audio_limit = false;
bool g_padMic_On = false;
//Eason : when thermal too hot, limit charging current ---

//+++ASUS_BSP Peter_lu : for set charging current to 1A..........................................
extern unsigned int ac_in;
int GPIO_Maxim8934_PIN1 = 97;
int ENABLE = 1;
//---ASUS_BSP Peter_lu : for set charging current to 1A..........................................

#ifdef CONFIG_MAXIM_8934_CHARGER
AXC_Maxim8934Charger *gpCharger = NULL;
// For checking initializing or not
static bool AXC_Maxim8934_Charger_GetGPIO(int anPin)
{
	return gpio_get_value(anPin);
}

static void AXC_Maxim8934_Charger_SetGPIO(int anPin, bool abHigh)
{
	gpio_set_value(anPin,abHigh);
	pr_debug( "[BAT][CHG]SetGPIO pin%d,%d:%d\n",anPin,abHigh,AXC_Maxim8934_Charger_GetGPIO(anPin));
}
static void AXC_Maxim8934_Charger_SetPIN1(AXC_Maxim8934Charger *this,bool abHigh)
{
	AXC_Maxim8934_Charger_SetGPIO(this->mpGpio_pin[Maxim8934_PIN1].gpio,abHigh);
}
#ifdef CONFIG_ENABLE_PIN2
static void AXC_Maxim8934_Charger_SetPIN2(AXC_Maxim8934Charger *this,bool abHigh)
{
	AXC_Maxim8934_Charger_SetGPIO(this->mpGpio_pin[Maxim8934_PIN2].gpio,abHigh);
}
#endif

static void AXC_Maxim8934_Charger_SetChargrDisbalePin(AXC_Maxim8934Charger *this,bool abDisabled)
{
	
	AXC_Maxim8934_Charger_SetGPIO(this->mpGpio_pin[Maxim8934_CHARGING_DISABLE].gpio,abDisabled);

}
static void EnableCharging(struct AXI_Charger *apCharger, bool enabled)
{
    AXC_Maxim8934Charger *this = container_of(apCharger,
                                            AXC_Maxim8934Charger,
                                            msParentCharger);

    AXC_Maxim8934_Charger_SetChargrDisbalePin(this,!enabled);
}

//Eason: Do VF with Cable when boot up+++
int getIfonline(void)
{
    return !gpio_get_value(gpCharger->mpGpio_pin[Maxim8934_DC_IN].gpio);
}
//Eason: Do VF with Cable when boot up---

static int AXC_Maxim8934_Charger_InitGPIO(AXC_Maxim8934Charger *this)
{
    Maxim8934_PIN_DEF i;

    int err = 0;

    for(i = 0; i<Maxim8934_PIN_COUNT;i++){
        //request
        err  = gpio_request(this->mpGpio_pin[i].gpio, this->mpGpio_pin[i].name);
        if (err < 0) {
            printk( "[BAT][CHG]gpio_request %s failed, err = %d\n",this->mpGpio_pin[i].name, err);
            goto err_exit;
        }

        //input
        if(this->mpGpio_pin[i].in_out_flag == 0){

            err = gpio_direction_input(this->mpGpio_pin[i].gpio);
            
            if (err  < 0) {
                printk( "[BAT][CHG]gpio_direction_input %s failed, err = %d\n", this->mpGpio_pin[i].name,err);
                goto err_exit;
            }

            if(this->mpGpio_pin[i].handler != NULL){

                this->mpGpio_pin[i].irq = gpio_to_irq(this->mpGpio_pin[i].gpio);

                if(true == this->mpGpio_pin[i].irq_enabled){
                    
                    enable_irq_wake(this->mpGpio_pin[i].irq);

                }

                err = request_irq(this->mpGpio_pin[i].irq , 
                    this->mpGpio_pin[i].handler, 
                    this->mpGpio_pin[i].trigger_flag,
                    this->mpGpio_pin[i].name, 
                    this);

                if (err  < 0) {
                    printk( "[BAT][CHG]request_irq %s failed, err = %d\n", this->mpGpio_pin[i].name,err);
                    goto err_exit;
                }

            }

        }else{//output

//+++ASUS_BSP Peter_lu : for set charging current to 1A.........................................
		if(ac_in && this->mpGpio_pin[i].gpio == GPIO_Maxim8934_PIN1 )	{
			this->mpGpio_pin[i].init_value = ENABLE;
			gpio_direction_output(this->mpGpio_pin[i].gpio,this->mpGpio_pin[i].init_value);
		}	else
			gpio_direction_output(this->mpGpio_pin[i].gpio,this->mpGpio_pin[i].init_value);
//+++ASUS_BSP Peter_lu : for set charging current to 1A.........................................
		
        }
    }

    return 0;
    
err_exit:

    for(i = 0; i<Maxim8934_PIN_COUNT;i++){
        
        gpio_free(this->mpGpio_pin[i].gpio);
        
    }
    
    return err;
}

static void AXC_Maxim8934_Charger_DeInitGPIO(AXC_Maxim8934Charger *this)
{
    Maxim8934_PIN_DEF i;
    
    for(i = 0; i<Maxim8934_PIN_COUNT;i++){
        
        gpio_free(this->mpGpio_pin[i].gpio);
        
    }
}

/*
static void AXC_Maxim8934_Charger_NotifyClientForStablePlugIn(struct work_struct *dat)
{
	AXC_Maxim8934Charger *this = container_of(dat,
                                                AXC_Maxim8934Charger,
                                                msNotifierWorker.work);

	this->msParentCharger.SetCharger(&this->msParentCharger, STABLE_CHARGER);
}
*/
//ASUS_BSP +++ Victor "suspend for fastboot mode"
#ifdef CONFIG_FASTBOOT
#include <linux/fastboot.h>
#endif //#ifdef CONFIG_FASTBOOT
//ASUS_BSP --- Victor "suspend for fastboot mode"      
static void (*notify_charger_in_out_func_ptr)(int) = NULL;
static DEFINE_SPINLOCK(charger_in_out_debounce_lock);
static void charger_in_out_debounce_time_expired(unsigned long _data)
{
    unsigned long flags;

    int online;

    struct AXC_Maxim8934Charger *this = (struct AXC_Maxim8934Charger *)_data;
    
    spin_lock_irqsave(&charger_in_out_debounce_lock, flags);

    online = !gpio_get_value(this->mpGpio_pin[Maxim8934_DC_IN].gpio);

    printk("[BAT][CHG]%s,%d\n",__FUNCTION__,online);

    wake_lock_timeout(&this->cable_in_out_wakelock, 3 * HZ);

    //ASUS_BSP +++ Victor "suspend for fastboot mode"
#ifdef CONFIG_FASTBOOT
      if(is_fastboot_enable()){

        if(online){
            ready_to_wake_up_and_send_power_key_press_event_in_fastboot();
        }
      }
#endif //#ifdef CONFIG_FASTBOOT
    //ASUS_BSP --- Victor "suspend for fastboot mode"        

#ifdef CONFIG_CHARGER_MODE
    g_chg_present = online;
#endif

    if(NULL != notify_charger_in_out_func_ptr){
    
         (*notify_charger_in_out_func_ptr) (online);
    
    }else{
    
         printk("Nobody registed..\n");
    }
    
    enable_irq_wake(this->mpGpio_pin[Maxim8934_DC_IN].irq);

    spin_unlock_irqrestore(&charger_in_out_debounce_lock, flags);

}

static irqreturn_t charger_in_out_handler(int irq, void *dev_id)
{
    unsigned long flags;
    
    AXC_Maxim8934Charger *this = (AXC_Maxim8934Charger *)dev_id;

    if(!timer_pending(&this->charger_in_out_timer)){

        spin_lock_irqsave(&charger_in_out_debounce_lock, flags);

        disable_irq_wake(irq);

        mod_timer(&this->charger_in_out_timer, jiffies + msecs_to_jiffies(20));

        spin_unlock_irqrestore(&charger_in_out_debounce_lock, flags);

    }
        
    return IRQ_HANDLED;

}
#ifdef ENABLE_WATCHING_STATUS_PIN_IN_IRQ
static void status_handle_work(struct work_struct *work)
{

    if(NULL == gpCharger){
        return;
    }

    printk("[BAT][CHG]%s,%d\n",__FUNCTION__,gpCharger->msParentCharger.IsCharging(&gpCharger->msParentCharger));

   if(true == gpCharger->mpGpio_pin[Maxim8934_CHARGING_STATUS].irq_enabled){
        disable_irq_wake(gpCharger->mpGpio_pin[Maxim8934_CHARGING_STATUS].irq);
        gpCharger->mpGpio_pin[Maxim8934_CHARGING_STATUS].irq_enabled = false;
    }    

    if(NULL != gpCharger->mpNotifier){

        gpCharger->mpNotifier->onChargingStart(&gpCharger->msParentCharger, gpCharger->msParentCharger.IsCharging(&gpCharger->msParentCharger));
    }    

}

static irqreturn_t charging_status_changed_handler(int irq, void *dev_id)
{
    
    AXC_Maxim8934Charger *this = (AXC_Maxim8934Charger *)dev_id;

    schedule_delayed_work(&this->asus_chg_work, (2000 * HZ/1000));
    
    return IRQ_HANDLED;

}
#endif

static void create_charger_proc_file(void);
static Maxim8934_PIN gGpio_pin[]=
{
    {//8934_DC_IN
        .gpio = 106,
        .name = "8934_DCIN",
        .in_out_flag = 0,
        .handler = charger_in_out_handler,
        .trigger_flag= IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, 
        .irq_enabled = false,        
    },
    {//8934_PIN1
        .gpio = 97,
        .name = "8934_Pin1",
        .in_out_flag = 1,
        .init_value = 0,
    },
    {//8934_CHARGING_DISABLE
        .gpio = 4,
        .name = "8934_Disable",
        .in_out_flag = 1,
        .init_value = 0,
    },
    {//8934_CHARGING_STATUS
        .gpio = 95,
        .name = "8934_ChargingStatus",
        .in_out_flag = 0,
#ifdef ENABLE_WATCHING_STATUS_PIN_IN_IRQ
        .handler = charging_status_changed_handler,    
        .trigger_flag= IRQF_TRIGGER_RISING,
        .irq_enabled = true,
#else
        .handler = NULL,    
#endif
    },
};

//Function implementation for AXC_Maxim8934_Charger
static void AXC_Maxim8934_Charger_Init(AXI_Charger *apCharger)
{
    AXC_Maxim8934Charger *this = container_of(apCharger,
                                            AXC_Maxim8934Charger,
                                            msParentCharger);

    if(false == this->mbInited)
    {
		printk( "[BAT][CHG]Init++\n");
		this->type = NO_CHARGER_TYPE;
		this->mpNotifier = NULL;
		//INIT_DELAYED_WORK(&this->msNotifierWorker, AXC_Maxim8934_Charger_NotifyClientForStablePlugIn) ;

#ifdef ENABLE_WATCHING_STATUS_PIN_IN_IRQ

        INIT_DELAYED_WORK(&this->asus_chg_work, status_handle_work);
#endif
             this->mpGpio_pin = gGpio_pin;



        wake_lock_init(&this->cable_in_out_wakelock, WAKE_LOCK_SUSPEND, "cable in out");

        setup_timer(&this->charger_in_out_timer, charger_in_out_debounce_time_expired,(unsigned long)this);

		if (0 == AXC_Maxim8934_Charger_InitGPIO(this)) {
			this->mbInited = true;
		} else {
			printk( "[BAT][CHG]Charger can't init\n");
        }
        
        charger_in_out_debounce_time_expired((unsigned long)this);
        
        //charger_in_out_handler(this->mpGpio_pin[Maxim8934_DC_IN].irq, this);

	create_charger_proc_file();

             gpCharger = this;
		printk( "[BAT][CHG]Init--\n");
    }
}

static void AXC_Maxim8934_Charger_DeInit(AXI_Charger *apCharger)
{
    AXC_Maxim8934Charger *this = container_of(apCharger,
                                            AXC_Maxim8934Charger,
                                            msParentCharger);

    if(true == this->mbInited) {
	AXC_Maxim8934_Charger_DeInitGPIO(this);
        this->mbInited = false;
    }
}

int AXC_Maxim8934_Charger_GetType(AXI_Charger *apCharger)
{
    AXC_Maxim8934Charger *this = container_of(apCharger,
                                            AXC_Maxim8934Charger,
                                            msParentCharger);

    return this->mnType;
}

void AXC_Maxim8934_Charger_SetType(AXI_Charger *apCharger ,int anType)
{
    AXC_Maxim8934Charger *this = container_of(apCharger,
                                            AXC_Maxim8934Charger,
                                            msParentCharger);

    this->mnType = anType;
}

static AXE_Charger_Type AXC_Maxim8934_Charger_GetChargerStatus(AXI_Charger *apCharger)
{
	AXC_Maxim8934Charger *this = container_of(apCharger,
                                                AXC_Maxim8934Charger,
                                                msParentCharger);

	return this->type;
}

// joshtest
/*

static struct timespec g_charger_update_time = {0};

static void chg_set_charger_update_time(void)
{
	g_charger_update_time = current_kernel_time();
	printk( "[BAT][Chg] %s(), tv_sec:%ld\n",
		__func__,
		g_charger_update_time.tv_sec);
	return;
}


void chg_get_charger_update_time(struct timespec *charger_update_time)
{
	BUG_ON(NULL != charger_update_time);
	*charger_update_time = g_charger_update_time;
	printk( "[BAT][Chg] %s(), tv_sec:%ld\n",
		__func__,
		charger_update_time->tv_sec);
	return;
}
*/

// joshtest
static char *pm_power_supplied_to[] = {
	"battery",
};

static enum power_supply_property pm_power_props[] = {
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
};

static int pm_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
        AXE_Charger_Type type;


       if(NULL == gpCharger){

            val->intval = 0;
            return 0;
       }

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
             val->intval =  0;

            type =gpCharger->msParentCharger.GetChargerStatus(&gpCharger->msParentCharger);

        
             if(NO_CHARGER_TYPE < type){

                if(psy->type ==POWER_SUPPLY_TYPE_MAINS && 
                    (type == HIGH_CURRENT_CHARGER_TYPE || 
                    type == ILLEGAL_CHARGER_TYPE|| 
                    type == NORMAL_CURRENT_CHARGER_TYPE)){            

                    val->intval =  1;

                }

                 if(psy->type ==POWER_SUPPLY_TYPE_USB && type == LOW_CURRENT_CHARGER_TYPE){            
                 
                     val->intval =  1;
                 
                 }

             }

             if(true == g_AcUsbOnline_Change0)
             {
                     val->intval = 0;
                     printk("[BAT][Chg]: set online 0 to shutdown device\n");   
             }
		break;
	default:
		return -EINVAL;
	}
	return 0;
}


static struct power_supply usb_psy = {
	.name		= "usb",
	.type		= POWER_SUPPLY_TYPE_USB,
	.supplied_to = pm_power_supplied_to,
       .num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	.properties	= pm_power_props,
	.num_properties	= ARRAY_SIZE(pm_power_props),
	.get_property	= pm_power_get_property,
};

static struct power_supply main_psy = {
	.name		= "ac",
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = pm_power_supplied_to,
       .num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	.properties	= pm_power_props,
	.num_properties	= ARRAY_SIZE(pm_power_props),
	.get_property	= pm_power_get_property,
};

//Eason : when thermal too hot, limit charging current +++ 
void setChgDrawACTypeCurrent(void)
{
	if( (true==g_thermal_limit)||(true==g_audio_limit) )
	{
    		AXC_Maxim8934_Charger_SetPIN1(gpCharger,0);
    	#ifdef CONFIG_ENABLE_PIN2
    		AXC_Maxim8934_Charger_SetPIN2(gpCharger,1);
    	#endif
    		printk("[BAT][CHG]:limit charging current 500mA\n");
    }else if(true==g_padMic_On){
    		AXC_Maxim8934_Charger_SetPIN1(gpCharger,0);
    	#ifdef CONFIG_ENABLE_PIN2
    		AXC_Maxim8934_Charger_SetPIN2(gpCharger,1);
    	#endif
            printk("[BAT][CHG]:InPad onCall limit cur500mA\n");
	}else{
		AXC_Maxim8934_Charger_SetPIN1(gpCharger,1);
		printk("[BAT][CHG]:dont limit charging current 1000mA\n");
	}
}
//Eason : when thermal too hot, limit charging current ---

//ASUS_BSP Eason when audio on, draw 1A from Pad ++++
void setChgDrawPadCurrent(bool audioOn)
{
   g_padMic_On = audioOn;
   printk("[BAT][CHG]g_padMic_On:%d\n",g_padMic_On);
   
   if( (false == audioOn)&&(false == g_thermal_limit)&&(false == g_audio_limit) )
   {
	 AXC_Maxim8934_Charger_SetPIN1(gpCharger,1);	
     printk("[BAT][CHG]:audio off : draw P02 1000mA\n");
   }else{
	 AXC_Maxim8934_Charger_SetPIN1(gpCharger,0);
#ifdef CONFIG_ENABLE_PIN2
	 AXC_Maxim8934_Charger_SetPIN2(gpCharger,1);
#endif
     printk("[BAT][CHG]:audio on : draw P02 500mA\n");
   }	
}
//ASUS_BSP Eason when audio on, draw 1A from Pad ---



static void AXC_Maxim8934_Charger_SetCharger(AXI_Charger *apCharger , AXE_Charger_Type aeChargerType)
{
	static bool first_call = true;

    AXC_Maxim8934Charger *this = container_of(apCharger,
                                            AXC_Maxim8934Charger,
                                            msParentCharger);

	if(false == this->mbInited)
		return;
/*
	if (!first_call && !this->m_is_bat_valid) {
		printk(KERN_INFO "[BAT][Chg] %s(), battery is invalid and cannot charging\n", __func__);
		aeChargerType = NO_CHARGER_TYPE;
	}
*/

//+++ASUS_BSP Peter_lu : for set charging current to 1A.........................................
	if(ac_in)	{
		if (aeChargerType == ILLEGAL_CHARGER_TYPE)
			aeChargerType =  HIGH_CURRENT_CHARGER_TYPE;
		printk( "[BAT][CHG]Bypass Charging Mode in P02:%d\n",aeChargerType);
	}	else	
//---ASUS_BSP Peter_lu : for set charging current to 1A..........................................
		printk( "[BAT][CHG]CharegeModeSet:%d\n",aeChargerType);

//+++ASUS_BSP Peter_lu : for set charging current to 1A..........................................
	ac_in = 0;
//---ASUS_BSP Peter_lu : for set charging current to 1A..........................................

//ASUS BSP Eason_Chang prevent P02 be set as illegal charger +++ 
    if(NORMAL_CURRENT_CHARGER_TYPE==this->type && ILLEGAL_CHARGER_TYPE==aeChargerType)
    {
        printk("[BAT][CHG]prevent P02 be set as illegal charger\n");
        return;
    }
//ASUS BSP Eason_Chang prevent P02 be set as illegal charger ---

	switch(aeChargerType)
    {
		case NO_CHARGER_TYPE:
			AXC_Maxim8934_Charger_SetPIN1(this,0);
#ifdef CONFIG_ENABLE_PIN2
			AXC_Maxim8934_Charger_SetPIN2(this,0);
#endif
			AXC_Maxim8934_Charger_SetChargrDisbalePin(this,1);
			this->type = aeChargerType;
 #ifdef ENABLE_WATCHING_STATUS_PIN_IN_IRQ          
            if(true == this->mpGpio_pin[Maxim8934_CHARGING_STATUS].irq_enabled){
                disable_irq_wake(this->mpGpio_pin[Maxim8934_CHARGING_STATUS].irq);
                this->mpGpio_pin[Maxim8934_CHARGING_STATUS].irq_enabled = false;
            }    
#endif

			//cancel_delayed_work(&this->msNotifierWorker);
			if(NULL != this->mpNotifier)
				this->mpNotifier->Notify(&this->msParentCharger,this->type);
			break;

        	case ILLEGAL_CHARGER_TYPE:
		case LOW_CURRENT_CHARGER_TYPE:

			AXC_Maxim8934_Charger_SetPIN1(this,0);
#ifdef CONFIG_ENABLE_PIN2
			AXC_Maxim8934_Charger_SetPIN2(this,1);
#endif
			AXC_Maxim8934_Charger_SetChargrDisbalePin(this,0);
			this->type = aeChargerType;
#ifdef ENABLE_WATCHING_STATUS_PIN_IN_IRQ
            if(false== this->mpGpio_pin[Maxim8934_CHARGING_STATUS].irq_enabled){
                enable_irq_wake(this->mpGpio_pin[Maxim8934_CHARGING_STATUS].irq);
                this->mpGpio_pin[Maxim8934_CHARGING_STATUS].irq_enabled = true;
            }    
#endif
			if(NULL != this->mpNotifier)
				this->mpNotifier->Notify(&this->msParentCharger,this->type);
			//cancel_delayed_work(&this->msNotifierWorker);
			//schedule_delayed_work(&this->msNotifierWorker , round_jiffies_relative(TIME_FOR_NOTIFY_AFTER_PLUGIN_CABLE));
			break;
        case NORMAL_CURRENT_CHARGER_TYPE:    
		case HIGH_CURRENT_CHARGER_TYPE:
		    //For SR1, we should not support ac mode to prevent device broken.
		    //Reenable for SR2@100325
		    //Eason : when thermal too hot, limit charging current +++ 
            //AXC_Maxim8934_Charger_SetPIN1(this,1);
            setChgDrawACTypeCurrent();
            //Eason : when thermal too hot, limit charging current ---            
		    //AXC_Maxim8934_Charger_SetPIN1(this,LOW);
		    //AXC_Maxim8934_Charger_SetPIN2(this,HIGH);
            AXC_Maxim8934_Charger_SetChargrDisbalePin(this,0);
			this->type = aeChargerType;
#ifdef ENABLE_WATCHING_STATUS_PIN_IN_IRQ   
            if(false== this->mpGpio_pin[Maxim8934_CHARGING_STATUS].irq_enabled){
                enable_irq_wake(this->mpGpio_pin[Maxim8934_CHARGING_STATUS].irq);
                this->mpGpio_pin[Maxim8934_CHARGING_STATUS].irq_enabled = true;
            }    
#endif

			if(NULL != this->mpNotifier)
				this->mpNotifier->Notify(&this->msParentCharger,this->type);
			//cancel_delayed_work(&this->msNotifierWorker);
			//schedule_delayed_work(&this->msNotifierWorker , round_jiffies_relative(TIME_FOR_NOTIFY_AFTER_PLUGIN_CABLE));
			break;
/*
		case TEMP_NO_CHARGER_TYPE:
				AXC_Maxim8934_Charger_SetPIN1(this,0);
#ifdef CONFIG_ENABLE_PIN2
				AXC_Maxim8934_Charger_SetPIN2(this,0);
#endif
				AXC_Maxim8934_Charger_SetChargrDisbalePin(this,1);
			break;
*/
		/*
		case STABLE_CHARGER:
			this->type = aeChargerType;
			if(true == this->msParentCharger.IsCharegrPlugin(&this->msParentCharger))
            {
				if(NULL != this->mpNotifier)
					this->mpNotifier->Notify(&this->msParentCharger,this->type);
			}
            else
				this->msParentCharger.SetCharger(&this->msParentCharger, NO_CHARGER_TYPE);
		    break;
		*/

		default:
			printk( "[BAT][CHG]Wrong ChargerMode:%d\n",aeChargerType);
			break;
	}

	if (first_call) {
		first_call = false;
	}

    	power_supply_changed(&usb_psy);
    	power_supply_changed(&main_psy);

}

void AcUsbPowerSupplyChange(void)
{
        power_supply_changed(&usb_psy);
    	power_supply_changed(&main_psy);
        printk("[BAT][Chg]:Ac Usb PowerSupply Change\n");
}

static void AXC_Maxim8934_Charger_SetBatConfig(AXI_Charger *apCharger , bool is_bat_valid)
{
//	AXC_Maxim8934Charger *this = container_of(apCharger,
//                                                AXC_Maxim8934Charger,
//                                                msParentCharger);
	if (is_bat_valid) {
		printk(KERN_INFO "[BAT][CHG]%s, bat is valid\n", __func__);
	}

	//this->m_is_bat_valid = is_bat_valid;

	return;
}

static bool AXC_Maxim8934_Charger_IsCharegrPlugin(AXI_Charger *apCharger)
{
	AXC_Maxim8934Charger *this = container_of(apCharger,
                                                AXC_Maxim8934Charger,
                                                msParentCharger);

#ifdef STAND_ALONE_WITHOUT_USB_DRIVER
	//Should be configured by usb driver...
	return (!AXC_Maxim8934_Charger_GetGPIO(this->mnChargePlugInPin));
#else
	return (this->type != NO_CHARGER_TYPE);
#endif
}


static bool AXC_Maxim8934_Charger_IsCharging(AXI_Charger *apCharger)
{
	AXC_Maxim8934Charger *this = container_of(apCharger,
                                                AXC_Maxim8934Charger,
                                                msParentCharger);

	if(0 != gpio_get_value(this->mpGpio_pin[Maxim8934_CHARGING_STATUS].gpio))
		return false;
	
	return true;
}
/*
static bool AXC_Maxim8934_Charger_Test(void *apTestObject)
{
	return true;
}
*/
#ifdef CONFIG_CHARGER_MODE
static void ChgModeIfCableInSetAsUsbDefault(struct AXI_Charger *apCharger, AXI_ChargerStateChangeNotifier *apNotifier
                                                ,AXE_Charger_Type chargerType)
{
        AXC_Maxim8934Charger *this = container_of(apCharger,
                                            AXC_Maxim8934Charger,
                                            msParentCharger);        
        int online;

        if( 1==g_CHG_mode ){

            online = !gpio_get_value(this->mpGpio_pin[Maxim8934_DC_IN].gpio);
            printk("[BAT][CHG]If cableIn:%d,%d\n",chargerType,online);
 
            if( (1==online) && (NOTDEFINE_TYPE== chargerType) )
            {
                this->type = NORMAL_CURRENT_CHARGER_TYPE;
                printk("[BAT][CHG]If cableIn set Pad default\n");
            
                if(NULL != notify_charger_in_out_func_ptr){
    
                     (*notify_charger_in_out_func_ptr) (online);
                     printk("[BAT][CHG]If cableIn notify online\n");
    
                }else{
    
                     printk("[BAT][CHG]Nobody registed..\n");
                }
                
            }
        } 
}
#endif

static void AXC_Maxim8934Charger_RegisterChargerStateChanged(struct AXI_Charger *apCharger, AXI_ChargerStateChangeNotifier *apNotifier
                                                            ,AXE_Charger_Type chargerType)
{
    AXC_Maxim8934Charger *this = container_of(apCharger,
                                            AXC_Maxim8934Charger,
                                            msParentCharger);   
#ifdef CONFIG_CHARGER_MODE     
    if( 1==g_CHG_mode ){
            ChgModeIfCableInSetAsUsbDefault(apCharger, apNotifier, chargerType);
    }
#endif
    this->mpNotifier = apNotifier;    
}

static void AXC_Maxim8934Charger_DeregisterChargerStateChanged(struct AXI_Charger *apCharger,AXI_ChargerStateChangeNotifier *apNotifier)
{
    AXC_Maxim8934Charger *this = container_of(apCharger,
                                            AXC_Maxim8934Charger,
                                            msParentCharger);

	if(this->mpNotifier == apNotifier)
		this->mpNotifier = NULL;
}

void AXC_Maxim8934Charger_Binding(AXI_Charger *apCharger,int anType)
{
    AXC_Maxim8934Charger *this = container_of(apCharger,
                                            AXC_Maxim8934Charger,
                                            msParentCharger);

    this->msParentCharger.Init = AXC_Maxim8934_Charger_Init;
    this->msParentCharger.DeInit = AXC_Maxim8934_Charger_DeInit;
    this->msParentCharger.GetType = AXC_Maxim8934_Charger_GetType;
    this->msParentCharger.SetType = AXC_Maxim8934_Charger_SetType;
    this->msParentCharger.GetChargerStatus = AXC_Maxim8934_Charger_GetChargerStatus;
    this->msParentCharger.SetCharger = AXC_Maxim8934_Charger_SetCharger;
    this->msParentCharger.EnableCharging = EnableCharging;
    this->msParentCharger.SetBatConfig = AXC_Maxim8934_Charger_SetBatConfig;
    this->msParentCharger.IsCharegrPlugin = AXC_Maxim8934_Charger_IsCharegrPlugin;
    this->msParentCharger.IsCharging = AXC_Maxim8934_Charger_IsCharging;
    this->msParentCharger.RegisterChargerStateChanged= AXC_Maxim8934Charger_RegisterChargerStateChanged;	
    this->msParentCharger.DeregisterChargerStateChanged= AXC_Maxim8934Charger_DeregisterChargerStateChanged;	
    this->msParentCharger.SetType(apCharger,anType);

    //this->mbInited = false;
#ifdef CHARGER_SELF_TEST_ENABLE
    this->msParentSelfTest.Test = AXC_Maxim8934_Charger_Test;
#endif
}

/*
Implement Interface for USB Driver
*/
#include "axc_chargerfactory.h"
/*
static unsigned GetChargerStatus(void)
{
	static AXI_Charger *lpCharger = NULL;

	if(NULL == lpCharger)
    {
		AXC_ChargerFactory_GetCharger(E_MAXIM8934_CHARGER_TYPE ,&lpCharger);
		lpCharger->Init(lpCharger);
	}

	return (unsigned) lpCharger->GetChargerStatus(lpCharger);
}

static void SetCharegerUSBMode(void)
{
	static AXI_Charger *lpCharger = NULL;

	if(NULL == lpCharger)
    {
		AXC_ChargerFactory_GetCharger(E_MAXIM8934_CHARGER_TYPE ,&lpCharger);
		lpCharger->Init(lpCharger);
	}

	printk( "[BAT][Chg]SetCharegerUSBMode\n");
	lpCharger->SetCharger(lpCharger,LOW_CURRENT_CHARGER_TYPE);
}

static void SetCharegerACMode(void)
{
	static AXI_Charger *lpCharger = NULL;

	if(NULL == lpCharger)
    {
		AXC_ChargerFactory_GetCharger(E_MAXIM8934_CHARGER_TYPE ,&lpCharger);
		lpCharger->Init(lpCharger);
	}

	printk( "[BAT][Chg]SetCharegerACMode\n");
	lpCharger->SetCharger(lpCharger,HIGH_CURRENT_CHARGER_TYPE);
}

static void SetCharegerNoPluginMode(void)
{
	static AXI_Charger *lpCharger = NULL;

	if(NULL == lpCharger)
    {
		AXC_ChargerFactory_GetCharger(E_MAXIM8934_CHARGER_TYPE ,&lpCharger);
		lpCharger->Init(lpCharger);
	}

    printk( "[BAT][Chg]SetCharegerNoPluginMode\n");
	lpCharger->SetCharger(lpCharger,NO_CHARGER_TYPE);
}
*/
static ssize_t charger_read_proc(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	static AXI_Charger *lpCharger = NULL;

	if(NULL == lpCharger)
    {
		AXC_ChargerFactory_GetCharger(E_MAXIM8934_CHARGER_TYPE ,&lpCharger);
		lpCharger->Init(lpCharger);
	}

	return sprintf(page, "%d\n", lpCharger->GetChargerStatus(lpCharger));
}

static ssize_t charger_write_proc(struct file *filp, const char __user *buff, 
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

    {
	    static AXI_Charger *lpCharger = NULL;

	    if(NULL == lpCharger)
        {
		    AXC_ChargerFactory_GetCharger(E_MAXIM8934_CHARGER_TYPE ,&lpCharger);
		    lpCharger->Init(lpCharger);
	    }

	    lpCharger->SetCharger(lpCharger,val);
    }
	//UpdateBatteryLifePercentage(gpGauge, val);
	
	return len;
}

static void create_charger_proc_file(void)
{
	struct proc_dir_entry *charger_proc_file = create_proc_entry("driver/asus_chg", 0644, NULL);

	if (charger_proc_file) {
		charger_proc_file->read_proc = charger_read_proc;
		charger_proc_file->write_proc = charger_write_proc;
	}
    else {
		printk( "[BAT][CHG]proc file create failed!\n");
    }

	return;
}
int registerChargerInOutNotificaition(void (*callback)(int))
{
    printk("[BAT][CHG]%s\n",__FUNCTION__);

    if(g_A60K_hwID >=A66_HW_ID_ER2){

        notify_charger_in_out_func_ptr = callback;   
        
    }else{
    
       pm8921_charger_register_vbus_sn(callback); 
       
    }

    return 0;
}
#ifdef CONFIG_EEPROM_NUVOTON
static int maxim8934_microp_event_handler(
	struct notifier_block *this,
	unsigned long event,
	void *ptr)
{
    if(gpCharger == NULL){

        return NOTIFY_DONE;
    }

	switch (event) {
	case P01_ADD:
            gpCharger->msParentCharger.SetCharger(&gpCharger->msParentCharger,NORMAL_CURRENT_CHARGER_TYPE);           
		break;	
	case P01_REMOVE: // means P01 removed
        gpCharger->msParentCharger.SetCharger(&gpCharger->msParentCharger, NO_CHARGER_TYPE);
		break;
	case P01_BATTERY_POWER_BAD: // P01 battery low
		break;
	case P01_AC_USB_IN:
		break;
	case P01_AC_USB_OUT:
		break;
	case DOCK_INIT_READY:
		break;
	case DOCK_PLUG_OUT:
		break;
	case DOCK_EXT_POWER_PLUG_IN: // means dock charging
		break;
	case DOCK_EXT_POWER_PLUG_OUT:	// means dock discharging
		break;		
	case DOCK_BATTERY_POWER_BAD:
		break;
	default:
             break;
	}

	return NOTIFY_DONE;
}
#endif /* CONFIG_EEPROM_NUVOTON */


#ifdef CONFIG_EEPROM_NUVOTON
static struct notifier_block maxim8934_microp_notifier = {
        .notifier_call = maxim8934_microp_event_handler,
};
#endif /* CONFIG_EEPROM_NUVOTON */



static int maxim8934_probe(struct platform_device *pdev)
{
	int err;
	//struct resource *res;

        printk("%s+++\n",__FUNCTION__);

#ifdef CONFIG_EEPROM_NUVOTON
        err = register_microp_notifier(&maxim8934_microp_notifier);
#endif /* CONFIG_EEPROM_NUVOTON */

    err = power_supply_register(&pdev->dev, &usb_psy);
    if (err < 0) {
        pr_err("power_supply_register usb failed rc = %d\n", err);
    }
    
    err= power_supply_register(&pdev->dev, &main_psy);
    if (err < 0) {
        pr_err("power_supply_register ac failed rc = %d\n", err);
    }


    printk("%s---\n",__FUNCTION__);
    return err;
}

static int maxim8934_remove(struct platform_device *pdev)
{
	power_supply_unregister(&usb_psy);
	power_supply_unregister(&main_psy);
	return 0;
}

static struct platform_driver maxim8934_driver = {
	.probe	= maxim8934_probe,
	.remove	= maxim8934_remove,
	.driver	= {
		.name	= "asus_chg",
		.owner	= THIS_MODULE,
	},
};

static int __init maxim8934_init(void)
{
	return platform_driver_register(&maxim8934_driver);
}

static void __exit maxim8934_exit(void)
{
	platform_driver_unregister(&maxim8934_driver);
}
// be be later after usb...
late_initcall(maxim8934_init);
module_exit(maxim8934_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ASUS battery virtual driver");
MODULE_VERSION("1.0");
MODULE_AUTHOR("Josh Liao <josh_liao@asus.com>");

//ASUS_BSP --- Josh_Liao "add asus battery driver"



#endif //#ifdef CONFIG_MAXIM_8934_CHARGER

