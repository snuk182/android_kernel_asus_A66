/*                                                                                                                                                       
        Maxim8934 Charge IC include file

*/
#ifndef __AXC_MAXIM8934CHARGER_H__
#define __AXC_MAXIM8934CHARGER_H__
#include <linux/types.h>
#include "axi_charger.h"         

//#define STAND_ALONE_WITHOUT_USB_DRIVER

// #define CHARGER_SELF_TEST_ENABLE

#ifdef CHARGER_SELF_TEST_ENABLE

#include "../selftest/axi_selftest.h"

#endif //CHARGER_SELF_TEST_ENABLE

//#define CONFIG_ENABLE_PIN2
//#define CONFIG_ENABLE_CHG_DONE_PIN
//#define CONFIG_ENABLE_CHG_FAULT_PIN

#include <linux/workqueue.h>
#include <linux/param.h>
#include <linux/irq.h>
#include <linux/wakelock.h>

//#define ENABLE_WATCHING_STATUS_PIN_IN_IRQ
typedef struct Maxim8934_PIN {
    int gpio;
    const char * name;
    int irq;
    int in_out_flag;
    irqreturn_t (*handler)(int, void *);
    int init_value;
    unsigned long trigger_flag;
    bool irq_enabled;
}Maxim8934_PIN;

typedef enum
{
    Maxim8934_DC_IN,
    Maxim8934_PIN1,
#ifdef CONFIG_ENABLE_PIN2
    Maxim8934_PIN2,
#endif
    Maxim8934_CHARGING_DISABLE,
    Maxim8934_CHARGING_STATUS,
#ifdef CONFIG_ENABLE_CHG_DONE_PIN
    Maxim8934_CHARGING_DONE,
#endif
#ifdef CONFIG_ENABLE_CHG_FAULT_PIN
    Maxim8934_CHARGING_FAULT,
#endif
    Maxim8934_PIN_COUNT
}Maxim8934_PIN_DEF;

typedef struct AXC_Maxim8934Charger {

	bool mbInited;
	int mnType;
	AXE_Charger_Type type;
    
	Maxim8934_PIN *mpGpio_pin;
	//bool m_is_bat_valid;

	AXI_ChargerStateChangeNotifier *mpNotifier;

        AXI_Charger msParentCharger;

    struct delayed_work asus_chg_work;    

    struct wake_lock cable_in_out_wakelock;

    struct timer_list charger_in_out_timer;

	//struct delayed_work msNotifierWorker;

#ifdef CHARGER_SELF_TEST_ENABLE

        AXI_SelfTest msParentSelfTest; 

#endif //CHARGER_SELF_TEST_ENABLE
}AXC_Maxim8934Charger;

extern void AXC_Maxim8934Charger_Binding(AXI_Charger *apCharger,int anType);
#endif //__AXC_MAXIM8934CHARGER_H__

