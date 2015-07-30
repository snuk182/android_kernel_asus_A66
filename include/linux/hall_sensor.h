#ifndef _LINUX_HALL_SENSOR_H
#define _LINUX_HALL_SENSOR_H


/*
*   hall sensor status
*/

enum P01_STATUS {
	P01_STATUS_BACKCOVER=0x1,
	P01_STATUS_HDMI=0x2,
	P01_STATUS_MICROP=0x4,
	P01_STATUS_UNKNOWN=0xFF,
};

/*
*   hall sensor status bit
*/
#define P01_STATUS_BACKCOVER_BIT              0x0001
#define P01_STATUS_HDMI_BIT              0x0002

/*
*   get hall sensor status
*/

extern int is_p01_backcover_on(void);
extern int is_p01_hdmi_inserted(void);
#endif
