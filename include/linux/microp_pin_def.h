#ifndef MICROP_PIN_DEF_H
#define MICROP_PIN_DEF_H

enum MICROP_INPUT{
        IN_DOCK_ACOK=0,
        IN_DOCK_INT,
        IN_ALS_INT,
        IN_HP_IN,
        IN_VOL_DOWN,
        IN_VOL_UP,
        IN_SUS_SW_R,
        IN_AC_USB_IN_R,
        IN_PWR_ON_OFF_R,
        IN_PB_DIS_uP,	// DOCK_PB
        IN_DOCK_IN_R,	
        IN_O_LID_R,		//  LID_CLOSE
        IN_LOUT_DET_R,	// HOOKKEY
        IN_BATT_GOOD,
        IN_15V_IN_R,
        IN_CAP_RPOX_INT,
};



enum MICROP_INTR_MASK{
        INTR_EN_DOCK_ACOK     =   0x1<<0,
        INTR_EN_DOCK_INT        =   0x1<<1,
        INTR_EN_ALS_INT          =   0x1<<2,
        INTR_EN_HP_IN              =   0x1<<3,
        INTR_EN_VOL_DOWN      =   0x1<<4,
        INTR_EN_VOL_UP            =  0x1<<5,
        INTR_EN_SUS_SW_R       =  0x1<<6,
        INTR_EN_AC_USB_IN_R  =  0x1<<7,
        INTR_EN_PWR_ON_OFF_R = 0x1<<8,
        INTR_EN_PB_DIS_uP   =   0x1<<9,       // DOCK_PB
        INTR_EN_DOCK_IN_R   =   0x1<<10,
        INTR_EN_O_LID_R         =   0x1<<11,          //  LID_CLOSE
        INTR_EN_HEADSET_KEY =   0x1<<12,     // AUD_ACC_IN	
        INTR_EN_BATT_GOOD     =    0x1<<13,
        INTR_EN_15V_IN_R        =    0x1<<14,
        INTR_EN_CAP_RPOX_INT    =   0x1<<15,
};


enum MICROP_INTR_STATUS{
        INTR_STA_DOCK_ACOK=0,
        INTR_STA_DOCK_INT,
        INTR_STA_ALS_INT,
        INTR_STA_HP_IN,
        INTR_STA_VOL_DOWN,
        INTR_STA_VOL_UP,
        INTR_STA_POWER_KEY,
        INTR_STA_AC_USB_IN_OUT,
        INTR_STA_PWR_EN,
        INTR_STA_DOCK_PB,	
        INTR_STA_DOCK_IN_OUT,	
        INTR_STA_DOCK_LID,	
        INTR_STA_HEADSET_KEY,
        INTR_STA_BAT_GOOD,
        INTR_STA_PROX_ACT,
        INTR_STA_BAT_ERR,
        INTR_STA_LONG_PRESS_PWRKEY,
};



/*
 *      g_microp_ver >=5 has the following defintion
*/
enum MICROP_OUTPUT{
        OUT_uP_EC_REQUEST=0,
        OUT_uP_TS_RST,
        OUT_uP_DIS_CHG,
        OUT_uP_FM34_RST,
        OUT_uP_FM34_PDN,
        OUT_uP_SPK_EN,
        OUT_uP_AUD_EN,
        OUT_uP_PWR_EN,
        OUT_uP_CAM_PWR_EN,
        OUT_uP_VIB_EN,
        OUT_uP_HP_MIC_SEL,
        OUT_uP_SYS_FET,		//dont care
        OUT_uP_BAT_FET,		//dont care
        OUT_uP_BAT_L_MOD,
        OUT_uP_EN_3V3_1V2,
        OUT_uP_HUB_SLEEP,
        OUT_uP_GPS_LNA_EN,
        OUT_uP_VBUS_EN,
        OUT_uP_SIZE,
};


#endif
