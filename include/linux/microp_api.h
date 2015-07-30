#ifndef __API_MICROP_H
#define __API_MICROP_H




enum Batt_ID{
	Batt_P01=0,
	Batt_Dock=1,
};


enum p01_Cable_Status{
       P01_CABLE_UNKNOWN=-1,
       P01_CABLE_NO=0,
       P01_CABLE_CHARGER=1,
       P01_CABLE_USB=2,
};


enum p01_Charging_Status{
	P01_CHARGING_ERR = -1,
	P01_CHARGING_NO = 0,
	P01_CHARGING_ONGOING=1,
	P01_CHARGING_FULL=2,
};




int AX_MicroP_HW_isER3(void);

/*
*       Check the status of P01 connectness
*       return value: 1: P01 connected
*/

int AX_MicroP_IsP01Connected(void);


/*
*       Check the status of Dock connectness
*       return value: 1: Dock connected and ready
*/
int AX_MicroP_IsDockReady(void);


/*
*       Check the status of Headphone if it is inserted
*       return value: 0: plugged out, 1: plugged in, <0: err
*/

int AX_MicroP_IsHeadPhoneIn(void);



/*
*       Check the status of AC/USB if it is inserted
*       return value: 0: plugged out, 1: plugged in, <0: err
*/

int AX_MicroP_IsACUSBIn(void);




/*
*       Check the status of Dock if it is inserted
*       return value: 0: plugged out, 1: plugged in, <0: err
*/

int AX_MicroP_IsECDockIn(void);



/*
*       Check the status of Dock battery if it is power-bad
*       return value: 1: PowerBad, <0: err
*/

int AX_MicroP_Is_ECBattPowerBad(void);


/*
*       Check the status of Dock Ext. Power if ext power is in
*       return value: 1: PowerBad, <0: err
*/
int AX_MicroP_Is_ECExtPowerCableIn(void);


/*
*   @AX_MicroP_get_ChargingStatus
*  input: target
*           0: p01 battery
*           1: dock battery
*
* return: -1: charge error, 0: no charge, 1: charging normal, 2: charging full, < 0: other error
*/

int AX_MicroP_get_ChargingStatus(int target);




/*
*   @AX_MicroP_get_USBDetectStatus
*  input: target
*           0: p01 battery
*           1: dock battery
*
*  return: 0 for 'no charger/usb', 1 for 'charger', 2 for 'USB', '255' for 'unknown', <0 value means something error
*/ 

int AX_MicroP_get_USBDetectStatus(int target);




/*
*  GPIO direct control
*  @ AX_MicroP_getGPIOPinLevel
*  input: 
            - pinID
*  return: 0 for low, 1 for high, <0 value means something error
*

*  @ AX_MicroP_setGPIOOutputPin
*  input: 
*           - pinID
*           - level: 0 for low, 1 for high
*  return: the status of operation. 0 for success, <0 value means something error

*  @ AX_MicroP_getGPIOOutputPinLevel
*  input:
            - pinID
*  return: 0 for low, 1 for high, <0 value means something error
*/

int AX_MicroP_getGPIOPinLevel(int pinID);
int AX_MicroP_setGPIOOutputPin(int pinID, int level);
int AX_MicroP_getGPIOOutputPinLevel(int pinID);




/*
*  @AX_MicroP_enableInterrupt
*  input: 
*            - intrpin: input pin id
*            -  enable: 0 for 'disable', 1 for 'enable'
*  return: 0 for success, <0 value means something error
*/

int AX_MicroP_enablePinInterrupt(unsigned int pinID, int enable);



/*
*  @AX_MicroP_readBattCapacity
*  input: target
*           0: p01 battery
*           1: dock battery
*  return: 0 for success, <0 value means something error
*/


int AX_MicroP_readBattCapacity(int target);




/*
*      Export function for Dock Use
*/

int get_EC_DOCK_IN_STATUS(void);
int get_EC_AP_WAKE_STATUS(void);
int set_EC_REQUEST_VALUE(int value);
int get_EC_HALL_SENSOR_STATUS(void);
int get_MicroP_HUB_SLEEP_STATUS(void);
int EC_Init_Complete(void);
int EC_Get_EXT_POWER_PLUG_IN_Ready(void);
int EC_Get_EXT_POWER_PLUG_OUT_Ready(void);
int EC_Get_DOCK_BATTERY_POWER_BAD_READY(void);

/*
Export Function for HDMI sleep control

*  input: 
*            -  vsleep: 
                           1: release hdmi for sleep, 
                           0: do not allow hdmi for sleep
*  return: 0 for success, <0 value means something error

*/

int checkScalarVote(void);
int audioVoteScalarSleepEntry(int vsleep);
int blVoteScalarSleepEntry(int vsleep);

#endif
