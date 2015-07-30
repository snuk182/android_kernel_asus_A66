//ASUS_BSP +++ Josh_Liao "add asus battery driver"
/* asus charger header file */
                
#ifndef _ASUS_CHG_H
#define _ASUS_CHG_H                                                                                   

/* asus charger source */
enum asus_chg_src {
	ASUS_CHG_SRC_NONE = 0,
	ASUS_CHG_SRC_USB,
	ASUS_CHG_SRC_DC,
	ASUS_CHG_SRC_PAD_BAT,
	ASUS_CHG_SRC_UNKNOWN,
	ASUS_CHG_SRC_PAD_NONE,
};


#ifdef CONFIG_CHARGER_ASUS
/**
 * asus_chg_set_chg_mode - set charger source to determine how much max usb current the 
 * system can draw
 * @chg_src: charger source 
 */
extern void asus_chg_set_chg_mode(enum asus_chg_src chg_src);

extern enum asus_chg_src asus_chg_get_chg_mode(void);

extern int registerChargerInOutNotificaition(void (*callback)(int));

#else

static inline void asus_chg_set_chg_mode(enum asus_chg_src chg_src) {}

static inline enum asus_chg_src asus_chg_get_chg_mode(void) { return ASUS_CHG_SRC_UNKNOWN;}

static inline int registerChargerInOutNotificaition(void (*callback)(int)){return 0;}


#endif /* CONFIG_CHARGER_ASUS */

#endif /* _ASUS_CHG_H */

//ASUS_BSP --- Josh_Liao "add asus battery driver"
