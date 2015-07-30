#include <linux/slab.h>
#include "axc_chargerfactory.h"
#include "axc_Maxim8934Charger.h"
#include "axc_PM8921Charger.h"
#include <linux/asus_bat.h>
#include "axc_DummyCharger.h"
#include "../axi_powerglobal.h"


//#include "../asus_bat_dbg.h"
void AXC_ChargerFactory_GetCharger(AXE_CHARGER_TYPE aeChargerType , AXI_Charger **appCharger)
{
	if (NULL != *appCharger)  {
		printk(DBGMSK_BAT_ERR "[BAT][ChagerFactory]Memory leak...\n");
	}

	*appCharger =  NULL;
	switch(aeChargerType) {
#ifdef CONFIG_MAXIM_8934_CHARGER
	case E_MAXIM8934_CHARGER_TYPE:
{
		static AXC_Maxim8934Charger *lpCharger = NULL;
		if(NULL == lpCharger)
		{
			lpCharger = kzalloc(sizeof(AXC_Maxim8934Charger), GFP_KERNEL);
			BUG_ON(NULL == lpCharger);
		}		

		*appCharger = &lpCharger->msParentCharger ;
		AXC_Maxim8934Charger_Binding(*appCharger, aeChargerType);
}
		break;
#endif
	case E_PM8921_CHARGER_TYPE:
	{
		static AXC_PM8921Charger *lpCharger = NULL;
		if(NULL == lpCharger)
		{
			lpCharger = kzalloc(sizeof(AXC_PM8921Charger), GFP_KERNEL);
//			BUG_ON(NULL == lpCharger);
		}		

		*appCharger = &lpCharger->msParentCharger ;
		AXC_PM8921Charger_Binding(*appCharger, aeChargerType);
		break;
	}
//#ifdef CONFIG_DUMMY_CHARGER		
    case  E_DUMMY_CHARGER_TYPE:
    {
            static AXC_DummyCharger *lpCharger = NULL;
			if(NULL == lpCharger)
            {
                lpCharger = kzalloc(sizeof(AXC_DummyCharger), GFP_KERNEL);
                assert(NULL != lpCharger);
            }		

            *appCharger = &lpCharger->msParentCharger ;
            AXC_DummyCharger_Binding(*appCharger, aeChargerType);
            break;
     }
//#endif
	default:
		printk(DBGMSK_BAT_ERR "[BAT][ChagerFactory]Not defined type...\n");
		break;
	}
	return;
}

void AXC_ChargerFactory_FreeCharger(AXI_Charger *apCharger)
{
	if (NULL == apCharger)
		return;

	switch(apCharger->GetType(apCharger)) {
 #ifdef CONFIG_MAXIM_8934_CHARGER
	case E_MAXIM8934_CHARGER_TYPE:
{
		AXC_Maxim8934Charger *lpCharger = container_of(apCharger, AXC_Maxim8934Charger, msParentCharger);
		kfree(lpCharger);
}
		break;
#endif
	case E_PM8921_CHARGER_TYPE:
	{
		AXC_PM8921Charger *lpCharger = container_of(apCharger, AXC_PM8921Charger, msParentCharger);
		kfree(lpCharger);
		break;
	}
#ifdef CONFIG_DUMMY_CHARGER			
	case E_DUMMY_CHARGER_TYPE:
   {
                AXC_DummyCharger *lpCharger = container_of(apCharger, AXC_DummyCharger, msParentCharger);
                kfree(lpCharger);
				break;
   }
#endif
	default:
		printk(DBGMSK_BAT_ERR "[BAT][FreeChager]Not defined type...\n");
		break;
	}
}
