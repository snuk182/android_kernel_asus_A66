/*
 *  Atmel maXTouch Touchscreen Controller Driver
 *
 *  
 *  Copyright (C) 2010 Atmel Corporation
 *  Copyright (C) 2010 Ulf Samuelsson (ulf@atmel.com)
 *  Copyright (C) 2009 Raphael Derosso Pereira <raphaelpereira@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * 
 * Driver for Atmel maXTouch family of touch controllers.
 *
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <asm/uaccess.h>
#include <linux/atmel_maxtouch.h>
//ASUS_BSP joe1_++
#include <linux/delay.h>
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp_notify.h>
#endif //CONFIG_EEPROM_NUVOTON
//ASUS_BSP joe1_--

#include <linux/module.h>

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>

/* Early-suspend level */
#define MXT_SUSPEND_LEVEL 1
#endif

#define DRIVER_VERSION "0.94p"

//ASUS_BSP joe1_++
//#define CAL_REPORT_RATE
//#define SUPPORT_PRESSURE

#define MAX_FINGER_NUM	10
#define MXT_BACKUP_TIME		25	//ms
#define MXT1386_RESET_TIME		255	//~250ms
#define MXT768_RESET_TIME		285	//~280ms
#define I2C_WAKE_TIME	25 //ms, for mXT1386 wake line in deep sleep mode
#define DISABLE_AUTO_CALIBRATION_TIME		15000 //ms
#define DISABLE_AUTO_CALIBRATION_DELAY		2000 //ms
#define DO_CALIBRATION_DELAY					0 //ms
#define LOCK_POINT_DELAY						4000 //ms

#define MOVE_DISTANCE	170
#define MOVE_DISTANCE_X	130
#define MOVE_DISTANCE_Y	150
#define LOCK_DISTANCE_X	100
#define LOCK_DISTANCE_Y	100

#define T9_CTRL_VAL		143

static struct mxt_data *g_mxt;

static bool g_bIsMxtInit = false;
static bool g_bIsPadAttach = false;
static bool g_bIsMxtIrqDisable = false;
static int g_iMxtProbeError = 0xff;
static u8 g_lastFirmwareMajor = 0;
static u8 g_lastFirmwareMinor = 0;
static bool g_bIsCmdCalibration = false;
static bool g_bIsCmdCalibrationOk = false;
static bool g_bIsCheckLockPoint = false;
static bool g_bIsCheckLockPointAlready = false;
#if 0
static bool g_bIsMakingChglineHigh = false;
#endif

static struct workqueue_struct *g_atmel_wq_dis_auto_cal;
static struct workqueue_struct *g_atmel_wq_attach_detach;
#ifdef CONFIG_EEPROM_NUVOTON
static struct work_struct g_mp_attach_work;
static struct work_struct g_mp_detach_work;
#endif //CONFIG_EEPROM_NUVOTON

static int debug = NO_DEBUG;
//ASUS_BSP joe1_--

module_param(debug, int, 0644);

MODULE_PARM_DESC(debug, "Activate debugging output");

#define T7_DATA_SIZE 3

/* Device Info descriptor */
/* Parsed from maXTouch "Id information" inside device */
struct mxt_device_info {
	u8   family_id;
	u8   variant_id;
	u8   major;
	u8   minor;
	u8   build;
	u8   num_objs;
	u8   x_size;
	u8   y_size;
	char family_name[16];	 /* Family name */
	char variant_name[16];    /* Variant name */
	u16  num_nodes;           /* Number of sensor nodes */
};

/* object descriptor table, parsed from maXTouch "object table" */
struct mxt_object {
	u16 chip_addr;
	u8  type;
	u8  size;
	u8  instances;
	u8  num_report_ids;
};

/* Mapping from report id to object type and instance */
struct report_id_map {
	u8  object;
	u8  instance;
/*
 * This is the first report ID belonging to object. It enables us to
 * find out easily the touch number: each touch has different report
 * ID (which are assigned to touches in increasing order). By
 * subtracting the first report ID from current, we get the touch
 * number.
 */
	u8  first_rid;
};

//ASUS_BSP joe1_++
struct finger_info {
	int status;
	int x;
	int y;
	int area;
	int amplitude;
	int vector;
};

struct first_finger_info {
	int status;
	int start_x;
	int start_y;
	int dx;
	int dy;
	int dsum;
	struct timeval timeval;
};

enum calibration_type
{
	CAL_TYPE_CMD,
	CAL_TYPE_LOCK_POINT
};
//ASUS_BSP joe1_--

/* Driver datastructure */
struct mxt_data {
	struct i2c_client    *client;
	struct input_dev     *input;
	int                  irq;

	u16                  last_read_addr;
	int                  read_fail_counter;

	struct mxt_device_info	device_info;

	u32	                info_block_crc;
	u32                  configuration_crc;
	u16                  report_id_count;
	struct report_id_map *rid_map;
	struct mxt_object    *object_table;

	u16                  msg_proc_addr;
	u8                   message_size;

	u16                  min_x_val;
	u16                  min_y_val;
	u16                  max_x_val;
	u16                  max_y_val;

	int                  (*init_hw)(struct i2c_client *client);
	int	               (*exit_hw)(struct i2c_client *client);
	int	               (*power_on)(bool on);
	u8                   (*valid_interrupt)(void);
	u8                   (*read_chg)(void);

	/* debugfs variables */
	struct dentry        *debug_dir;
	int                  current_debug_datap;
	struct mutex         debug_mutex;
	u16                  *debug_data;

#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend		early_suspend;
#endif
	u16 t7_addr; //ASUS_BSP joe1_++
	u8 t7_data[T7_DATA_SIZE];
	bool is_suspended;
	//ASUS_BSP joe1_++
	struct attribute_group attrs;
	struct finger_info finger[MAX_FINGER_NUM];
	struct delayed_work d_work_disable_auto_calibration;
	struct delayed_work d_work_disable_irq;
	struct delayed_work d_work_do_calibration;
	struct delayed_work d_work_check_lock_point;
	struct first_finger_info first_finger[MAX_FINGER_NUM];
	//ASUS_BSP joe1_--
};

/*default value, enough to read versioning*/
#define CONFIG_DATA_SIZE	6
static u16 t38_size = CONFIG_DATA_SIZE;

static int mxt_read_block(struct i2c_client *client, u16 addr, u16 length, u8 *value);

static int mxt_write_byte(struct i2c_client *client, u16 addr, u8 value);

static int mxt_write_block(struct i2c_client *client, u16 addr, u16 length, u8 *value);

static u8 mxt_valid_interrupt_dummy(void)
{
	return 1;
}

#define I2C_RETRY_COUNT 3

/* Returns the start address of object in mXT memory. */
#define	MXT_BASE_ADDR(object_type, mxt)					\
	get_object_address(object_type, 0, mxt->object_table,           \
			   mxt->device_info.num_objs)

/* Maps a report ID to an object type (object type number). */
#define	REPORT_ID_TO_OBJECT(rid, mxt)			\
	(((rid) == 0xff) ? 0 : mxt->rid_map[rid].object)

/* Maps a report ID to an object type (string). */
#define	REPORT_ID_TO_OBJECT_NAME(rid, mxt)			\
	object_type_name[REPORT_ID_TO_OBJECT(rid, mxt)]

/* Returns non-zero if given object is a touch object */
#define IS_TOUCH_OBJECT(object) \
	((object == MXT_TOUCH_MULTITOUCHSCREEN_T9) || \
	 (object == MXT_TOUCH_KEYARRAY_T15) ||	\
	 (object == MXT_TOUCH_PROXIMITY_T23) || \
	 (object == MXT_TOUCH_SINGLETOUCHSCREEN_T10) || \
	 (object == MXT_TOUCH_XSLIDER_T11) || \
	 (object == MXT_TOUCH_YSLIDER_T12) || \
	 (object == MXT_TOUCH_XWHEEL_T13) || \
	 (object == MXT_TOUCH_YWHEEL_T14) || \
	 (object == MXT_TOUCH_KEYSET_T31) || \
	 (object == MXT_TOUCH_XSLIDERSET_T32) ? 1 : 0)

//ASUS_BSP joe1_++
#define mxt_debug(level, ...) \
		if (debug >= (level)) \
			pr_info(__VA_ARGS__);
//ASUS_BSP joe1_--


/* 
 * Check whether we have multi-touch enabled kernel; if not, report just the
 * first touch (on mXT224, the maximum is 10 simultaneous touches).
 * Because just the 1st one is reported, it might seem that the screen is not
 * responding to touch if the first touch is removed while the screen is being
 * touched by another finger, so beware. 
 *
 */
#ifdef ABS_MT_TRACKING_ID
static inline void report_mt(int touch_number, int size, int x, int y, struct
			mxt_data *mxt) {
	input_report_abs(mxt->input, ABS_MT_TRACKING_ID, touch_number);
	input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR, size);
	input_report_abs(mxt->input, ABS_MT_POSITION_X, x);
	input_report_abs(mxt->input, ABS_MT_POSITION_Y, y);
	input_mt_sync(mxt->input);
}
#else
static inline void report_mt(int touch_number, int size, int x, int y, struct
			mxt_data *mxt) {
	if (touch_number == 0) {
		input_report_abs(mxt->input, ABS_TOOL_WIDTH, size);
		input_report_abs(mxt->input, ABS_X, x);
		input_report_abs(mxt->input, ABS_Y, y);
	}
}
#endif

static inline void report_gesture(int data, struct mxt_data *mxt)
{
	input_event(mxt->input, EV_MSC, MSC_GESTURE, data); 
}


static const u8	*object_type_name[] = {
	[0]  = "Reserved",
	[5]  = "GEN_MESSAGEPROCESSOR_T5",
	[6]  = "GEN_COMMANDPROCESSOR_T6",
	[7]  = "GEN_POWERCONFIG_T7",
	[8]  = "GEN_ACQUIRECONFIG_T8",
	[9]  = "TOUCH_MULTITOUCHSCREEN_T9",
	[15] = "TOUCH_KEYARRAY_T15",
	[18] = "SPT_COMMSCONFIG_T18",
	[19] = "SPT_GPIOPWM_T19",
	[20] = "PROCI_GRIPFACESUPPRESSION_T20",
	[22] = "PROCG_NOISESUPPRESSION_T22",
	[23] = "TOUCH_PROXIMITY_T23",
	[24] = "PROCI_ONETOUCHGESTUREPROCESSOR_T24",
	[25] = "SPT_SELFTEST_T25",
	[27] = "PROCI_TWOTOUCHGESTUREPROCESSOR_T27",
	[28] = "SPT_CTECONFIG_T28",
	[37] = "DEBUG_DIAGNOSTICS_T37",
	[38] = "SPT_USER_DATA_T38",
	[40] = "PROCI_GRIPSUPPRESSION_T40",
	[41] = "PROCI_PALMSUPPRESSION_T41",
	[42] = "PROCI_TOUCHSUPPRESSION_T42", //ASUS_BSP joe1_++
	[43] = "SPT_DIGITIZER_T43",
	[44] = "SPT_MESSAGECOUNT_T44",
	[46] = "SPT_CTECONFIG_T46", //ASUS_BSP joe1_++
	[47] = "PROCI_STYLUS_T47", //ASUS_BSP joe1_++
	[48] = "PROCG_NOISESUPPRESSION_T48", //ASUS_BSP joe1_++
	[52] = "TOUCH_PROXKEY_T52", //ASUS_BSP joe1_++
	[53] = "GEN_DATASOURCE_T53", //ASUS_BSP joe1_++
};

static u16 get_object_address(uint8_t object_type,
			      uint8_t instance,
			      struct mxt_object *object_table,
			      int max_objs);

//ASUS_BSP joe1_++
static bool cfg_flag = true;
static irqreturn_t mxt_irq_handler(int irq, void *_mxt);

#ifdef CONFIG_EEPROM_NUVOTON
extern void reportPadStationI2CFail(char *devname);
static void __devinit attach_padstation_work(struct work_struct *work);
static void __devinit detach_padstation_work(struct work_struct *work);
#endif //CONFIG_EEPROM_NUVOTON

static void force_release_fingers(void)
{
	struct mxt_data *mxt = g_mxt;
	struct finger_info *finger = g_mxt->finger;
	int i;

	mxt_debug(DEBUG_INFO, "[touch_pad] force_release_fingers()++\n");

	for (i = 0; i < MAX_FINGER_NUM; i++)
	{
		if (finger[i].area == 0)
			continue;
		
		finger[i].area = 0;
		finger[i].amplitude = 0;
		finger[i].vector = 0;

		input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR, finger[i].area);
		input_report_abs(mxt->input, ABS_MT_POSITION_X, finger[i].x);
		input_report_abs(mxt->input, ABS_MT_POSITION_Y, finger[i].y);
#ifdef SUPPORT_PRESSURE
		input_report_abs(mxt->input, ABS_MT_PRESSURE, finger[i].amplitude);
		input_report_abs(mxt->input, ABS_MT_ORIENTATION, finger[i].vector);
#endif

		input_mt_sync(mxt->input);

		mxt_debug(DEBUG_INFO, "[touch_pad] force_release_fingers: finger[%d].x=%d\n", i, finger[i].x);
		mxt_debug(DEBUG_INFO, "[touch_pad] force_release_fingers: finger[%d].y=%d\n", i, finger[i].y);
	}

	input_report_key(mxt->input, BTN_TOUCH, 0);

	input_mt_sync(mxt->input);

	input_sync(mxt->input);

	mxt_debug(DEBUG_INFO, "[touch_pad] force_release_fingers()--\n");
}

static inline void init_T8_auto_cal_config(struct mxt_data *mxt, int iTCHAUTOCAL, int iATCHFRCCALTHR)
{
	if ( iTCHAUTOCAL )
	{
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+4, 20);
	}
	else
	{
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+4, 0);
	}

	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+6, 5);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+7, 40);

	if ( iATCHFRCCALTHR )
	{
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+8, 50);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+9, 25);
	}
	else
	{
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+8, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+9, 192);
	}
}

static void do_cal_work(struct work_struct *work)
{
	struct mxt_data *mxt = container_of(work, struct mxt_data, d_work_do_calibration.work);
	struct first_finger_info *first_finger = mxt->first_finger;
	int i;

	printk("[touch_pad] do_cal_work()++\n");

//	printk("[touch_pad] g_mxt=0x%x ; mxt=0x%x\n", (unsigned int)g_mxt, (unsigned int)mxt);
//	printk("[touch_pad] max_x_val=%d, max_y_val=%d\n", mxt->max_x_val, mxt->max_y_val);

	for (i = 0; i < MAX_FINGER_NUM; i++)
	{
		first_finger[i].status = -1;
		first_finger[i].start_x = 0;
		first_finger[i].start_y = 0;
		first_finger[i].dx = 0;
		first_finger[i].dy = 0;
		first_finger[i].dsum = 0;
	}

	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt), 1);

	force_release_fingers();

	init_T8_auto_cal_config(mxt, 0, 0);

	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PALMSUPPRESSION_T41, mxt), 0);

	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt) + MXT_ADR_T6_CALIBRATE, 1);

	g_bIsCmdCalibration = true;

	printk("[touch_pad] do_cal_work()--\n");
}

static void disable_auto_cal_delayed_work(struct work_struct *work)
{
	static bool bIsFingersDetect = false;
	struct mxt_data *mxt = container_of(work, struct mxt_data, d_work_disable_auto_calibration.work);
	struct finger_info *finger = mxt->finger;
	int i;

	printk("[touch_pad] disable_auto_cal_delayed_work()++\n");

//	printk("[touch_pad] g_mxt=0x%x ; mxt=0x%x\n", (unsigned int)g_mxt, (unsigned int)mxt);
//	printk("[touch_pad] max_x_val=%d, max_y_val=%d\n", mxt->max_x_val, mxt->max_y_val);

	bIsFingersDetect = false;

	for (i = 0; i < MAX_FINGER_NUM; i++)
	{
		if (finger[i].area != 0)
		{
			bIsFingersDetect = true;
			break;
		}
	}

	if ( !bIsFingersDetect )
	{
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+4, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+6, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+7, 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+8, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+9, 0);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PALMSUPPRESSION_T41, mxt), 1);

		printk("[touch_pad] disable_auto_cal_delayed_work(): no finger, disable auto calibration success!\n");
	}
	else
	{
		queue_delayed_work(g_atmel_wq_dis_auto_cal, &mxt->d_work_disable_auto_calibration, msecs_to_jiffies(DISABLE_AUTO_CALIBRATION_DELAY));

		printk("[touch_pad] disable_auto_cal_delayed_work(): fingers detected, delay %d ms\n", DISABLE_AUTO_CALIBRATION_DELAY);
	}

	printk("[touch_pad] disable_auto_cal_delayed_work()--bIsFingersDetect=%d\n", bIsFingersDetect);
}

static void check_lock_point_work(struct work_struct *work)
{
	struct mxt_data *mxt = container_of(work, struct mxt_data, d_work_check_lock_point.work);
	struct first_finger_info *first_finger = mxt->first_finger;
	struct finger_info *finger = mxt->finger;
	int i;
	bool bDoCalibration = false;

	printk("[touch_pad] check_lock_point_work()++\n");

	for (i = 0; i < MAX_FINGER_NUM; i++)
	{
		if ( first_finger[i].status == 1 )
		{
			first_finger[i].dx = finger[i].x - first_finger[i].start_x;
			first_finger[i].dy = finger[i].y - first_finger[i].start_y;
//			first_finger[i].dsum = abs(first_finger[i].dx) + abs(first_finger[i].dy);

			printk("[touch_pad] check_lock_point_work: touch_number=%d; dsum=%d\n", i, first_finger[i].dsum);

			if ( first_finger[i].dx < LOCK_DISTANCE_X && first_finger[i].dy < LOCK_DISTANCE_Y )
			{
				bDoCalibration = true;

				printk("[touch_pad] check_lock_point_work: touch_number=%d; trigger Calibraion!!!\n", i);

				break;
			}
		}
	}

	if ( bDoCalibration )
	{
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt), 1);

		force_release_fingers();

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt) + MXT_ADR_T6_CALIBRATE, 1);
	}

	g_bIsCheckLockPoint = false;
	g_bIsCheckLockPointAlready = true;

	printk("[touch_pad] check_lock_point_work()--\n");
}

static void disable_irq_delayed_work(struct work_struct *work)
{
	struct mxt_data *mxt = container_of(work, struct mxt_data, d_work_disable_irq.work);

	printk("[touch_pad] disable_irq_delayed_work()++\n");

	cancel_delayed_work_sync(&mxt->d_work_do_calibration);
	cancel_delayed_work_sync(&mxt->d_work_disable_auto_calibration);
	cancel_delayed_work_sync(&mxt->d_work_check_lock_point);
	flush_workqueue(g_atmel_wq_dis_auto_cal);

	disable_irq(mxt->irq);

#ifdef CONFIG_EEPROM_NUVOTON
	reportPadStationI2CFail("maXTouchPad");
#endif

	printk("[touch_pad] disable_irq_delayed_work()--\n");
}

static ssize_t dump_T7(struct device *dev, struct device_attribute *devattr, char *buf)
{
	int i;
	int err;
	u8 tmp[100];
	char tmpstr[800];
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);

//	sprintf(buf,"");

	err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, data), 3, (u8 *) tmp);
	if (err < 0){
		sprintf(tmpstr, "read T7 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}
	else{
		for (i=0; i < 3; i++){
			sprintf(tmpstr,"T7 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}

	err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, data), 10, (u8 *) tmp);
	if (err < 0){
		sprintf(tmpstr, "read T8 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}
	else{
		for (i=0; i < 10; i++){
			sprintf(tmpstr,"T8 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}

	if ( data->device_info.family_id == MXT1386_FAMILYID ) //mXT1386
	{
		err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, data), 34, (u8 *) tmp);
		if (err < 0){
			sprintf(tmpstr, "read T9 cfg error, ret %d\n",err);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
		else{
			for (i=0; i < 34; i++){
				sprintf(tmpstr,"T9 byte[%d] = %d\n",i,tmp[i]);
				strncat (buf,tmpstr,strlen(tmpstr));
			}
		}
	}
	else if ( data->device_info.family_id == MXT768_FAMILYID ) //mXT768
	{
		err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, data), 35, (u8 *) tmp);
		if (err < 0){
			sprintf(tmpstr, "read T9 cfg error, ret %d\n",err);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
		else{
			for (i=0; i < 35; i++){
				sprintf(tmpstr,"T9 byte[%d] = %d\n",i,tmp[i]);
				strncat (buf,tmpstr,strlen(tmpstr));
			}
		}
	}

	err = mxt_read_block(data->client, MXT_ADDR_INFO_BLOCK, MXT_ID_BLOCK_SIZE,(u8 *) tmp);
	if (err < 0){
		sprintf(tmpstr, "read Family ID error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}
	else{
		sprintf(tmpstr, "Family ID is %d\n",tmp[0]);
		strncat (buf,tmpstr,strlen(tmpstr));
	}

	return strlen(buf);
}

static ssize_t dump_T15(struct device *dev, struct device_attribute *devattr, char *buf)
{	
	int i;
	u8 tmp[100];
	char tmpstr[800];
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);

//	sprintf(buf,"");

	mxt_read_block(data->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, data), 11, (u8 *) tmp);
	for (i=0; i < 11; i++){
		sprintf(tmpstr,"T15 byte[%d] = %d\n",i,tmp[i]);
		strncat (buf,tmpstr,strlen(tmpstr));
	}

	mxt_read_block(data->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, data), 2, (u8 *) tmp);
	for (i=0; i < 2; i++){
		sprintf(tmpstr,"T18 byte[%d] = %d\n",i,tmp[i]);
		strncat (buf,tmpstr,strlen(tmpstr));
	}
	
	if ( data->device_info.family_id == MXT768_FAMILYID ) //mXT768
	{
		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_SPT_GPIOPWM_T19, data), 16, (u8 *) tmp);
		for (i=0; i < 16; i++){
			sprintf(tmpstr,"T19 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}

	return strlen(buf);
}

static ssize_t dump_T20(struct device *dev, struct device_attribute *devattr, char *buf)
{	
	int i;
	u8 tmp[100];
	char tmpstr[800];
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);

//	sprintf(buf,"");

	if ( data->device_info.family_id == MXT1386_FAMILYID ) //mXT1386
	{
		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, data), 17, (u8 *) tmp);
		for (i=0; i < 17; i++){
			sprintf(tmpstr,"T22 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}

		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, data), 19, (u8 *) tmp);
		for (i=0; i < 19; i++){
			sprintf(tmpstr,"T24 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}

		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, data), 14, (u8 *) tmp);
		for (i=0; i < 14; i++){
			sprintf(tmpstr,"T25 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}

		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, data), 7, (u8 *) tmp);
		for (i=0; i < 7; i++){
			sprintf(tmpstr,"T27 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}

		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, data), 6, (u8 *) tmp);
		for (i=0; i < 6; i++){
			sprintf(tmpstr,"T28 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}
	else if ( data->device_info.family_id == MXT768_FAMILYID ) //mXT768
	{
		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, data), 19, (u8 *) tmp);
		for (i=0; i < 19; i++){
			sprintf(tmpstr,"T24 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}

		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, data), 26, (u8 *) tmp);
		for (i=0; i < 26; i++){
			sprintf(tmpstr,"T25 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}

		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, data), 7, (u8 *) tmp);
		for (i=0; i < 7; i++){
			sprintf(tmpstr,"T27 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}

	return strlen(buf);
}

static ssize_t dump_T38(struct device *dev, struct device_attribute *devattr, char *buf)
{	
	int i;
	u8 tmp[100];
	char tmpstr[800];
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);

//	sprintf(buf,"");

	if ( data->device_info.family_id == MXT1386_FAMILYID ) //mXT1386
	{
		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_USER_INFO_T38, data), 64, (u8 *) tmp);
		for (i=0; i < 64; i++){
			sprintf(tmpstr,"T38 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}
	else if ( data->device_info.family_id == MXT768_FAMILYID ) //mXT768
	{
		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_USER_INFO_T38, data), 8, (u8 *) tmp);
		for (i=0; i < 8; i++){
			sprintf(tmpstr,"T38 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}

	return strlen(buf);
}

static ssize_t dump_T40(struct device *dev, struct device_attribute *devattr, char *buf)
{	
	int i;
	u8 tmp[100];
	char tmpstr[800];
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);

//	sprintf(buf,"");

	mxt_read_block(data->client, MXT_BASE_ADDR(MXT_GRIPSUPPRESSION_T40, data), 5, (u8 *) tmp);
	for (i=0; i < 5; i++){
		sprintf(tmpstr,"T40 byte[%d] = %d\n",i,tmp[i]);
		strncat (buf,tmpstr,strlen(tmpstr));
	}

	if ( data->device_info.family_id == MXT1386_FAMILYID ) //mXT1386
	{
		if ( (data->device_info.major == 1) && (data->device_info.minor == 1 ) )
		{
			mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PALMSUPPRESSION_T41, data), 7, (u8 *) tmp);
			for (i=0; i < 7; i++){
				sprintf(tmpstr,"T41 byte[%d] = %d\n",i,tmp[i]);
				strncat (buf,tmpstr,strlen(tmpstr));
			}

			mxt_read_block(data->client, MXT_BASE_ADDR(MXT_DIGITIZER_T43, data), 7, (u8 *) tmp);
			for (i=0; i < 7; i++){
				sprintf(tmpstr,"T43 byte[%d] = %d\n",i,tmp[i]);
				strncat (buf,tmpstr,strlen(tmpstr));
			}
		}
		else
		{
			mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PALMSUPPRESSION_T41, data), 6, (u8 *) tmp);
			for (i=0; i < 6; i++){
				sprintf(tmpstr,"T41 byte[%d] = %d\n",i,tmp[i]);
				strncat (buf,tmpstr,strlen(tmpstr));
			}

			mxt_read_block(data->client, MXT_BASE_ADDR(MXT_DIGITIZER_T43, data), 6, (u8 *) tmp);
			for (i=0; i < 6; i++){
				sprintf(tmpstr,"T43 byte[%d] = %d\n",i,tmp[i]);
				strncat (buf,tmpstr,strlen(tmpstr));
			}
		}
	}
	else if ( data->device_info.family_id == MXT768_FAMILYID ) //mXT768
	{
		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, data), 8, (u8 *) tmp);
		for (i=0; i < 8; i++){
			sprintf(tmpstr,"T42 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}

		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_DIGITIZER_T43, data), 7, (u8 *) tmp);
		for (i=0; i < 7; i++){
			sprintf(tmpstr,"T43 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}

		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, data), 9, (u8 *) tmp);
		for (i=0; i < 9; i++){
			sprintf(tmpstr,"T46 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}

		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, data), 5, (u8 *) tmp);
		for (i=0; i < 5; i++){
			sprintf(tmpstr,"T53 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}

	return strlen(buf);
}

static ssize_t dump_T48(struct device *dev, struct device_attribute *devattr, char *buf)
{	
	int i;
	u8 tmp[100];
	char tmpstr[800];
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);

//	sprintf(buf,"");

	if ( data->device_info.family_id == MXT1386_FAMILYID ) //mXT1386
	{
		sprintf(buf,"no objects!!!\n");
	}
	else if ( data->device_info.family_id == MXT768_FAMILYID ) //mXT768
	{
		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, data), 74, (u8 *) tmp);
		for (i=0; i < 74; i++){
			sprintf(tmpstr,"T48 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}

	return strlen(buf);
}

static ssize_t store_mode2(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	int cfg[3];

	disable_irq(data->irq);

	sscanf(buf, "%d%d%d\n",&cfg[0], &cfg[1], &cfg[2]);
		
	mxt_write_byte(data->client, MXT_BASE_ADDR(cfg[0], data)+cfg[1],cfg[2]);

	printk("[touch_pad] cfg[0]=%d, cfg[1]=%d, cfg[2]=%d\n",cfg[0],cfg[1],cfg[2]);
	
//	mxt_write_byte(data->client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, data) + MXT_ADR_T6_BACKUPNV,MXT_CMD_T6_BACKUP);
//	msleep(MXT_BACKUP_TIME); //wait to backup
	msleep(100); //wait to backup

#if 0
	//sw reset
//	mxt_write_byte(data->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6,data) + MXT_ADR_T6_RESET, 0x01);
	if ( data->device_info.family_id == MXT1386_FAMILYID ) //mXT1386
	{
		msleep(MXT1386_RESET_TIME);
	}
	else
	{
		msleep(MXT768_RESET_TIME);
	}
#endif

	enable_irq(data->irq);

	printk("[touch_pad] config finish!\n");

	return count;
}

static void mxt_backup_nvm(struct mxt_data *mxt)
{
	mxt_write_byte(mxt->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6,mxt) + MXT_ADR_T6_BACKUPNV,MXT_CMD_T6_BACKUP);
//	msleep(MXT_BACKUP_TIME); //wait to backup
	msleep(100); //wait to backup

#if 0
	//sw reset
	mxt_write_byte(mxt->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6,mxt) + MXT_ADR_T6_RESET, 0x01);
	if ( mxt->device_info.family_id == MXT1386_FAMILYID ) //mXT1386
	{
		msleep(MXT1386_RESET_TIME);
	}
	else
	{
		msleep(MXT768_RESET_TIME);
	}
#endif
}

static void disable_auto_calibration(struct mxt_data *mxt)
{
	cancel_delayed_work_sync(&mxt->d_work_do_calibration);
	cancel_delayed_work_sync(&mxt->d_work_disable_auto_calibration);
	cancel_delayed_work_sync(&mxt->d_work_check_lock_point);
	flush_workqueue(g_atmel_wq_dis_auto_cal);

	queue_delayed_work(g_atmel_wq_dis_auto_cal, &mxt->d_work_disable_auto_calibration, msecs_to_jiffies(DISABLE_AUTO_CALIBRATION_TIME));
}

static int touch_config_zero(struct mxt_data *mxt)
{
	int i;

	printk("[touch_pad] touch_config_zero()++\n");

	if ( mxt->device_info.family_id == MXT1386_FAMILYID ) //mXT1386
	{
		printk("[touch_pad] touch_config_zero(): mXT1386: family_id = 0x%x\n", mxt->device_info.family_id);

		for ( i=0; i < 64; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_USER_INFO_T38, mxt)+i, 0);
		}

		for ( i=0; i < 3; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt)+i, 0);
		}

		for ( i=0; i < 10; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+i, 0);
		}

		for ( i=0; i < 34; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+i, 0);
		}

		for ( i=0; i < 11; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt)+i, 0);
		}

		for ( i=0; i < 2; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, mxt)+i, 0);
		}

		for ( i=0; i < 17; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+i, 0);
		}

		for ( i=0; i < 19; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+i, 0);
		}

		for ( i=0; i < 14; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+i, 0);
		}

		for ( i=0; i < 7; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, mxt)+i, 0);
		}

		for ( i=0; i < 6; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, mxt)+i, 0);
		}

		for ( i=0; i < 5; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GRIPSUPPRESSION_T40, mxt)+i, 0);
		}

		if ( (mxt->device_info.major == 1) && (mxt->device_info.minor == 1 ) )
		{
			for ( i=0; i < 7; i++ )
			{
				mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PALMSUPPRESSION_T41, mxt)+i, 0);
			}

			for ( i=0; i < 7; i++ )
			{
				mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_DIGITIZER_T43, mxt)+i, 0);
			}
		}
		else
		{
			for ( i=0; i < 6; i++ )
			{
				mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PALMSUPPRESSION_T41, mxt)+i, 0);
			}

			for ( i=0; i < 6; i++ )
			{
				mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_DIGITIZER_T43, mxt)+i, 0);
			}
		}
	}
	else if ( mxt->device_info.family_id == MXT768_FAMILYID ) //mXT768
	{
		printk("[touch_pad] touch_config_zero(): mXT768: family_id = 0x%x\n", mxt->device_info.family_id);

		for ( i=0; i < 5; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, mxt)+i, 0);
		}

		for ( i=0; i < 64; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_USER_INFO_T38, mxt)+i, 0);
		}

		for ( i=0; i < 3; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt)+i, 0);
		}

		for ( i=0; i < 10; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+i, 0);
		}

		for ( i=0; i < 35; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+i, 0);
		}

		for ( i=0; i < 11; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt)+i, 0);
		}

		for ( i=0; i < 2; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, mxt)+i, 0);
		}

		for ( i=0; i < 16; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_GPIOPWM_T19, mxt)+i, 0);
		}

		for ( i=0; i < 19; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+i, 0);
		}

		for ( i=0; i < 26; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+i, 0);
		}

		for ( i=0; i < 7; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, mxt)+i, 0);
		}

		for ( i=0; i < 5; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GRIPSUPPRESSION_T40, mxt)+i, 0);
		}

		for ( i=0; i < 8; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+i, 0);
		}

		for ( i=0; i < 7; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_DIGITIZER_T43, mxt)+i, 0);
		}

		for ( i=0; i < 9; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+i, 0);
		}

		for ( i=0; i < 10; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_STYLUS_T47, mxt)+i, 0);
		}

		for ( i=0; i < 74; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+i, 0);
		}

		for ( i=0; i < 15; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_PROXKEY_T52, mxt)+i, 0);
		}
	}
	
	mxt_backup_nvm(mxt);

	printk("[touch_pad] touch_config_zero()--\n");

	return 0;
}

static int init_touch_config(struct mxt_data *mxt)
{
	int i;

	printk("[touch_pad] init_touch_config()++\n");

	if ( mxt->device_info.family_id == MXT1386_FAMILYID ) //mXT1386
	{
		printk("[touch_pad] init_touch_config(): mXT1386: family_id = 0x%x\n", mxt->device_info.family_id);

//		mxt_write_byte(mxt->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt) + MXT_ADR_T6_CALIBRATE, 1);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt), 65);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt)+1, 14);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt)+2, 10);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt), 9);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+1, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+2, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+3, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+5, 0);
		init_T8_auto_cal_config(mxt, 0, 0);

#ifdef SUPPORT_PRESSURE
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt), 131);
#else
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt), T9_CTRL_VAL);
#endif
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+1, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+2, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+3, 28);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+4, 42);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+5, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+6, 16);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+7, 55);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+8, 3);
//ASUS_BSP joe1_++
//#ifdef COORDINATE_INVERT
//		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+9, 5); //ASUS_BSP joe1_++:invert
//#else
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+9, 3); //ASUS_BSP joe1_++:normal
//#endif
//ASUS_BSP joe1_--
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+10, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+11, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+12, 3);//3
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+13, 14);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+14, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+15, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+16, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+17, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+18, 31);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+19, 3);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+20, 255);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+21, 4);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+22, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+23, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+24, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+25, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+26, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+27, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+28, 64);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+29, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+30, 15);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+31, 15);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+32, 49);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+33, 52);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt), 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt)+1, 7);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt)+2, 41);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt)+3, 14);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt)+4, 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt)+5, 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt)+6, 16);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt)+7, 50);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt)+8, 2);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt)+9, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt)+10,0);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, mxt),0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, mxt)+1,0);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt), 5);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+1, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+2, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+3, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+4, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+5, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+6, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+7, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+8, 32);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+9, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+10, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+11, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+12, 15);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+13, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+14, 25);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+15, 30);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+16, 0);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt), 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+1, 4);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+2, 255);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+3, 3);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+4, 63);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+5, 100);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+6, 100);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+7, 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+8, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+9, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+10, 40);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+11, 75);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+12, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+13, 2);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+14, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+15, 100);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+16, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+17, 25);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+18, 0);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt), 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+1, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+2, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+3, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+4, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+5, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+6, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+7, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+8, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+9, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+10, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+11, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+12, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+13, 0);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, mxt), 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, mxt)+1, 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, mxt)+2, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, mxt)+3, 224);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, mxt)+4, 3);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, mxt)+5, 35);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, mxt)+6, 0);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, mxt), 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, mxt)+1, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, mxt)+2, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, mxt)+3, 8);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, mxt)+4, 28);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, mxt)+5, 60);

		for (i=0; i < 64; i++)
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_USER_INFO_T38, mxt)+i, 0);
		}

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GRIPSUPPRESSION_T40, mxt), 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GRIPSUPPRESSION_T40, mxt)+1, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GRIPSUPPRESSION_T40, mxt)+2, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GRIPSUPPRESSION_T40, mxt)+3, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GRIPSUPPRESSION_T40, mxt)+4, 20);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PALMSUPPRESSION_T41, mxt), 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PALMSUPPRESSION_T41, mxt)+1, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PALMSUPPRESSION_T41, mxt)+2, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PALMSUPPRESSION_T41, mxt)+3, 35);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PALMSUPPRESSION_T41, mxt)+4, 5);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PALMSUPPRESSION_T41, mxt)+5, 20);
		if ( (mxt->device_info.major == 1) && (mxt->device_info.minor == 1 ) )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PALMSUPPRESSION_T41, mxt)+6, 0);
//			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PALMSUPPRESSION_T41, mxt)+6, 0xAA);
		}

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_DIGITIZER_T43, mxt), 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_DIGITIZER_T43, mxt)+1, 125);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_DIGITIZER_T43, mxt)+2, 92);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_DIGITIZER_T43, mxt)+3, 5);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_DIGITIZER_T43, mxt)+4, 137);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_DIGITIZER_T43, mxt)+5, 8);
	}
	else if ( mxt->device_info.family_id == MXT768_FAMILYID ) //mXT768
	{
		printk("[touch_pad] init_touch_config(): mXT768: family_id = 0x%x\n", mxt->device_info.family_id);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, mxt), 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, mxt)+1, 32);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, mxt)+2, 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, mxt)+3, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_DATASOURCE_T53, mxt)+4, 2);

		for ( i=0; i < 64; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_USER_INFO_T38, mxt)+i, 0);
		}

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt), 255);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt)+1, 255);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt)+2, 10);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt), 60);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+1, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+2, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+3, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+4, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+5, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+6, 5);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+7, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+8, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+9, 0);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt), 143);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+1, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+2, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+3, 24);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+4, 32);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+5, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+6, 144);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+7, 50);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+8, 2);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+9, 7);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+10, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+11, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+12, 5);//3
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+13, 31);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+14, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+15, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+16, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+17, 5);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+18, 31);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+19, 3);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+20, 255);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+21, 4);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+22, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+23, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+24, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+25, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+26, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+27, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+28, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+29, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+30, 100);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+31, 12);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+32, 57);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+33, 69);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+34, 0);

		for ( i=0; i < 11; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt)+i, 0);
		}

		for ( i=0; i < 2; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, mxt)+i, 0);
		}

		for ( i=0; i < 16; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_GPIOPWM_T19, mxt)+i, 0);
		}

		for ( i=0; i < 19; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+i, 0);
		}

		for ( i=0; i < 26; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+i, 0);
		}

		for ( i=0; i < 7; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, mxt)+i, 0);
		}

		for ( i=0; i < 5; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GRIPSUPPRESSION_T40, mxt)+i, 0);
		}

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt), 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+1, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+2, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+3, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+4, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+5, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+6, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+7, 0);

		for ( i=0; i < 7; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_DIGITIZER_T43, mxt)+i, 0);
		}

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt), 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+1, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+2, 16);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+3, 16);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+4, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+5, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+6, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+7, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+8, 0);

		for ( i=0; i < 10; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_STYLUS_T47, mxt)+i, 0);
		}

#if 0
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt), 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+1, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+2, 64);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+3, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+4, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+5, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+6, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+7, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+8, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+9, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+10, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+11, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+12, 0);//3
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+13, 6);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+14, 6);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+15, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+16, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+17, 100);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+18, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+19, 63);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+20, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+21, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+22, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+23, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+24, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+25, 46);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+26, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+27, 0);
		for ( i=28; i < 74; i++ )
#endif
		for ( i=0; i < 74; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+i, 0);
		}

		for ( i=0; i < 15; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_PROXKEY_T52, mxt)+i, 0);
		}

		//#if 0
		//	mxt_write_byte(mxt->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt) + MXT_ADR_T6_CALIBRATE, 1);
		//#endif
	}

	mxt_backup_nvm(mxt);
	disable_auto_calibration(mxt);

	printk("[touch_pad] init_touch_config()--\n");

	return 0;
}

static ssize_t chip_config(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *mxt = i2c_get_clientdata(client);
	int cfg;

	disable_irq(mxt->irq);

	sscanf(buf, "%d\n", &cfg);

	printk("[touch_pad] chip_config: cfg=%d\n", cfg);

	if ( cfg == 0 )
	{
		touch_config_zero( mxt );
	}
	else
	{
		init_touch_config( mxt );

		// save power state values for suspend/resume
		mxt_read_block(mxt->client, mxt->t7_addr, ARRAY_SIZE(mxt->t7_data), mxt->t7_data);
	}

	enable_irq(mxt->irq);

	printk("[touch_pad] chip_config: config finish!\n");

	return count;
}

static ssize_t force_release(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *mxt = i2c_get_clientdata(client);
	struct finger_info *finger = mxt->finger;
	int cfg;
	int i;

	sscanf(buf, "%d\n", &cfg);

	printk("[touch_pad] force_release()++ cfg=%d\n", cfg);

	if ( cfg == 0 )
	{
		force_release_fingers();
	}
	else
	{
		for (i = 0; i < MAX_FINGER_NUM; i++)
		{
			printk("[touch_pad] force_release: finger[%d].status=0x%x\n", i, finger[i].status);
			printk("[touch_pad] force_release: finger[%d].area=%d\n", i, finger[i].area);
			printk("[touch_pad] force_release: finger[%d].x=%d\n", i, finger[i].x);
			printk("[touch_pad] force_release: finger[%d].y=%d\n", i, finger[i].y);
			printk("[touch_pad] force_release: finger[%d].amplitude=%d\n", i, finger[i].amplitude);
			printk("[touch_pad] force_release: finger[%d].vector=%d\n", i, finger[i].vector);
		}
	}
	
	printk("[touch_pad] force_release()--\n");

	return count;
}

static ssize_t reset_irq(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *mxt = i2c_get_clientdata(client);
	int cfg;
	int error;

	sscanf(buf, "%d\n", &cfg);

	printk("[touch_pad] reset_irq()++ cfg=%d\n", cfg);

	free_irq(mxt->irq, mxt);

	msleep(50);

	if ( cfg == 0 )
	{
		printk("[touch_pad] reset_irq: IRQF_TRIGGER_FALLING\n");

		error = request_threaded_irq(mxt->irq,
									NULL,
									mxt_irq_handler,
									IRQF_TRIGGER_FALLING,
									client->dev.driver->name,
									mxt);
		if (error < 0)
		{
			dev_err(&client->dev, "[touch_pad] failed to allocate irq %d\n", mxt->irq);
		}
	
	}
	else
	{
		printk("[touch_pad] reset_irq: IRQF_TRIGGER_LOW | IRQF_ONESHOT\n");

		error = request_threaded_irq(mxt->irq,
									NULL,
									mxt_irq_handler,
									IRQF_TRIGGER_LOW | IRQF_ONESHOT,
									client->dev.driver->name,
									mxt);
		if (error < 0)
		{
			dev_err(&client->dev, "[touch_pad] failed to allocate irq %d\n", mxt->irq);
		}
	}
	
	printk("[touch_pad] reset_irq()--\n");

	return count;
}

//ASUS_BSP simpson: add for ATD reading raw data +++
#define CHANNEL_NUM 1176 //X*Y=28*42
static ssize_t chk_raw_data(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *mxt = i2c_get_clientdata(client);
	int i;
	u16 *data;
	u16 diagnostics_reg;
	int offset = 0;
	int size;
	int read_size;
	int error;
	char *buf_start;
	u16 debug_data_addr;
	u16 page_address;
	u8 page;
	u8 debug_command_reg;
	u8 debug_command = MXT_CMD_T6_REFERENCES_MODE;
	u16 raw_max = 0, raw_min = 20000;

	printk("[touch_pad] dump_raw_data()++\n");

//ASUS_BSP simpson: porting from debug_data_open()+++
if (mxt->debug_data == NULL){
	mxt->current_debug_datap = 0;

	mxt->debug_data = kmalloc(mxt->device_info.num_nodes * sizeof(u16), GFP_KERNEL);
	if (mxt->debug_data == NULL)
		return -ENOMEM;
	printk("[touch_pad] debug_data_open()!!\n");
}
//ASUS_BSP simpson: porting from debug_data_open()---

	data = mxt->debug_data;
	if (data == NULL)
		return -EIO;

	/* If first read after open, read all data to buffer. */
	if (mxt->current_debug_datap == 0)
	{
		diagnostics_reg = MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt) + MXT_ADR_T6_DIAGNOSTIC;
		debug_data_addr = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, mxt) + MXT_ADR_T37_DATA;
		page_address = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, mxt) + MXT_ADR_T37_PAGE;

		error = mxt_read_block(mxt->client, page_address, 1, &page);
		if (error < 0) {
			printk("[touch_pad] read page error!!");
			return error;
		}

		//printk("[touch_pad] debug data page = %d\n", page);

		while (page != 0)
		{
			error = mxt_write_byte(mxt->client, diagnostics_reg, MXT_CMD_T6_PAGE_DOWN);
			if (error < 0)
				return error;

			/* Wait for command to be handled; when it has, the
			   register will be cleared. */
			debug_command_reg = 1;
			while (debug_command_reg != 0)
			{
				error = mxt_read_block(mxt->client, diagnostics_reg, 1, &debug_command_reg);
				if (error < 0)
					return error;

				//printk("[touch_pad] Waiting for debug diag command to propagate...\n");
			}

			error = mxt_read_block(mxt->client, page_address, 1, &page);
			if (error < 0)
				return error;

			//printk("[touch_pad] debug data page = %d\n", page);
		}

		/*
		 * Lock mutex to prevent writing some unwanted data to debug
		 * command register. User can still write through the char
		 * device interface though. TODO: fix?
		 */
		mutex_lock(&mxt->debug_mutex);
		printk("[touch_pad] mutex_lock!!\n");

		/* Configure Debug Diagnostics object to show deltas/refs */
		error = mxt_write_byte(mxt->client, diagnostics_reg, debug_command);

		if (error < 0)
		{
			printk("[touch_pad] Error writing to maXTouch device!\n");
			return error;
		}

		/* Wait for command to be handled; when it has, the
		 * register will be cleared. */
		debug_command_reg = 1;
		while (debug_command_reg != 0)
		{
			error = mxt_read_block(mxt->client, diagnostics_reg, 1, &debug_command_reg);
			if (error < 0)
				return error;

			//printk("[touch_pad] Waiting for debug diag command to propagate...\n");
		}

		size = CHANNEL_NUM * sizeof(u16);

		while (size > 0)
		{
			if ( page==7 || page==15 || page==23 )
			{
				goto page_skip;
			}
			else if ( page==6 || page==14 || page==22 )
			{
				read_size = 8 * sizeof(u16);
			}
			else
			{
				read_size = 64 * sizeof(u16);
			}

			//printk("[touch_pad] Debug data read loop, reading %d bytes...\n", read_size);

			error = mxt_read_block(mxt->client, debug_data_addr, read_size, (u8 *) &data[offset]);
			if (error < 0)
			{
				printk("[touch_pad] Error reading debug data\n");
				goto error;
			}

			offset += read_size/2;
			size -= read_size;
			//printk("[touch_pad] Debug data: left %d bytes to read...\n", size);

page_skip:
			/* Select next page */
			//printk("[touch_pad] try Page Up!\n");
			error = mxt_write_byte(mxt->client, diagnostics_reg, MXT_CMD_T6_PAGE_UP);
			if (error < 0)
			{
				printk("[touch_pad] Error writing to maXTouch device!\n");
				goto error;
			}

			//printk("[touch_pad] Page Up!\n");

			/* Wait for command to be handled; when it has, the
			   register will be cleared. */
			debug_command_reg = 1;
			while (debug_command_reg != 0)
			{
				error = mxt_read_block(mxt->client, diagnostics_reg, 1, &debug_command_reg);
				if (error < 0)
					return error;

				//printk("[touch_pad] Waiting for debug diag command to propagate...\n");
			}

			error = mxt_read_block(mxt->client, page_address, 1, &page);
			if (error < 0)
				return error;

			//printk("[touch_pad] debug data page = %d\n", page);
		}

		mutex_unlock(&mxt->debug_mutex);
		printk("[touch_pad] mutex_unlock!! -> P02_raw_data dump finished.\n");
	}

	buf_start = buf;

	for (i=0; i < CHANNEL_NUM; i++){
		if ( data[i] > raw_max )
			raw_max = data[i];
		if ( data[i] < raw_min )
			raw_min = data[i];
	}

	//buf += sprintf(buf, "%5d\n",raw_max);
	//buf += sprintf(buf, "%5d\n",raw_min);
	if (( raw_max < 11000 ) && ( raw_min > 6000 )){
		buf += sprintf(buf, "%d\n",1);
	} else {
		buf += sprintf(buf, "%d\n",0);
	}


//ASUS_BSP simpson: porting from debug_data_release()+++
if ( i == CHANNEL_NUM ){
	kfree(mxt->debug_data);
	mxt->debug_data = NULL;
	printk("[touch_pad] debug_data_release()!!\n");
}
//ASUS_BSP simpson: porting from debug_data_release()---

	printk("[touch_pad] dump_raw_data()--\n");
	return (buf - buf_start);

error:
	mutex_unlock(&mxt->debug_mutex);
	printk("[touch_pad] dump_raw_data() error!!\n");
	return error;

}
//ASUS_BSP simpson: add for ATD reading raw data ---

DEVICE_ATTR(load_cfg_padstation, S_IRUGO | S_IWUSR, NULL, chip_config);
DEVICE_ATTR(cfg_padstation, S_IRUGO | S_IWUSR, NULL, store_mode2);
DEVICE_ATTR(dump_T7_pad, 0755, dump_T7, NULL);
DEVICE_ATTR(dump_T15_pad, 0755, dump_T15, NULL);
DEVICE_ATTR(dump_T20_pad, 0755, dump_T20, NULL);
DEVICE_ATTR(dump_T38_pad, 0755, dump_T38, NULL);
DEVICE_ATTR(dump_T40_pad, 0755, dump_T40, NULL);
DEVICE_ATTR(dump_T48_pad, 0755, dump_T48, NULL);
DEVICE_ATTR(chk_raw_data_pad, 0755, chk_raw_data, NULL);
DEVICE_ATTR(release_fingers_pad, S_IRUGO | S_IWUSR, NULL, force_release);
DEVICE_ATTR(reset_irq_pad, S_IRUGO | S_IWUSR, NULL, reset_irq);

static struct attribute *mxt_attr[] = {
	&dev_attr_dump_T7_pad.attr,
	&dev_attr_dump_T15_pad.attr,
	&dev_attr_dump_T20_pad.attr,
	&dev_attr_dump_T38_pad.attr,
	&dev_attr_dump_T40_pad.attr,
	&dev_attr_dump_T48_pad.attr,
	&dev_attr_chk_raw_data_pad.attr,
	&dev_attr_cfg_padstation.attr,
	&dev_attr_load_cfg_padstation.attr,
	&dev_attr_release_fingers_pad.attr,
	&dev_attr_reset_irq_pad.attr,
	NULL
};
//ASUS_BSP joe1_--

static ssize_t debug_data_read(struct mxt_data *mxt, char *buf, size_t count, 
			loff_t *ppos, u8 debug_command){
	int i;
	u16 *data;
	u16 diagnostics_reg;
	int offset = 0;
	int size;
	int read_size;
	int error;
	char *buf_start;
	u16 debug_data_addr;
	u16 page_address;
	u8 page;
	u8 debug_command_reg;

	data = mxt->debug_data;
	if (data == NULL)
		return -EIO;

	/* If first read after open, read all data to buffer. */
	if (mxt->current_debug_datap == 0){

		diagnostics_reg = MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, 
						mxt) + 
			          MXT_ADR_T6_DIAGNOSTIC;
		if (count > (mxt->device_info.num_nodes * 2))
			count = mxt->device_info.num_nodes;
	
		debug_data_addr = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, mxt)+ 
			          MXT_ADR_T37_DATA;
		page_address = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, mxt) +
			       MXT_ADR_T37_PAGE;
		error = mxt_read_block(mxt->client, page_address, 1, &page);
		if (error < 0)
			return error;
		mxt_debug(DEBUG_TRACE, "[touch_pad] debug data page = %d\n", page);		
		while (page != 0) {
			error = mxt_write_byte(mxt->client, 
					diagnostics_reg, 
					MXT_CMD_T6_PAGE_DOWN);
			if (error < 0)
				return error;
			/* Wait for command to be handled; when it has, the
			   register will be cleared. */
			debug_command_reg = 1;
			while (debug_command_reg != 0) {
				error = mxt_read_block(mxt->client, 
						diagnostics_reg, 1,
						&debug_command_reg);
				if (error < 0)
					return error;
				mxt_debug(DEBUG_TRACE, 
					"[touch_pad] Waiting for debug diag command "
					"to propagate...\n");

			}
		        error = mxt_read_block(mxt->client, page_address, 1, 
					&page);
			if (error < 0)
				return error;
			mxt_debug(DEBUG_TRACE, "[touch_pad] debug data page = %d\n", page);	
		}

		/*
		 * Lock mutex to prevent writing some unwanted data to debug
		 * command register. User can still write through the char 
		 * device interface though. TODO: fix?
		 */

		mutex_lock(&mxt->debug_mutex);
		/* Configure Debug Diagnostics object to show deltas/refs */
		error = mxt_write_byte(mxt->client, diagnostics_reg,
				debug_command);

                /* Wait for command to be handled; when it has, the
		 * register will be cleared. */
		debug_command_reg = 1;
		while (debug_command_reg != 0) {
			error = mxt_read_block(mxt->client, 
					diagnostics_reg, 1,
					&debug_command_reg);
			if (error < 0)
				return error;
			mxt_debug(DEBUG_TRACE, "[touch_pad] Waiting for debug diag command "
				"to propagate...\n");

		}	

		if (error < 0) {
			printk (KERN_WARNING 
				"[touch_pad] Error writing to maXTouch device!\n");
			return error;
		}
	
		size = mxt->device_info.num_nodes * sizeof(u16);

		while (size > 0) {
			read_size = size > 128 ? 128 : size;
			mxt_debug(DEBUG_TRACE, 
				"[touch_pad] Debug data read loop, reading %d bytes...\n",
				read_size);
			error = mxt_read_block(mxt->client, 
					       debug_data_addr, 
					       read_size, 
					       (u8 *) &data[offset]);
			if (error < 0) {
				printk(KERN_WARNING 
				       "[touch_pad] Error reading debug data\n");
				goto error;
			}
			offset += read_size/2;
			size -= read_size;

			/* Select next page */
			error = mxt_write_byte(mxt->client, diagnostics_reg, 
					MXT_CMD_T6_PAGE_UP);
			if (error < 0) {
				printk(KERN_WARNING
					"[touch_pad] Error writing to maXTouch device!\n");
				goto error;
			}
		}
		mutex_unlock(&mxt->debug_mutex);
	}

	buf_start = buf;
	i = mxt->current_debug_datap;

	while (((buf- buf_start) < (count - 6)) && 
		(i < mxt->device_info.num_nodes)){

		mxt->current_debug_datap++;
		if (debug_command == MXT_CMD_T6_REFERENCES_MODE)
			buf += sprintf(buf, "%d: %5d\n", i,
				       (u16) le16_to_cpu(data[i]));
		else if (debug_command == MXT_CMD_T6_DELTAS_MODE)
			buf += sprintf(buf, "%d: %5d\n", i,
				       (s16) le16_to_cpu(data[i]));
		i++;
	}

	return (buf - buf_start);
error:
	mutex_unlock(&mxt->debug_mutex);
	return error;
}

//ASUS_BSP simpson: add for ATD reading raw data +++
#define CHANNEL_NUM 1176 //X*Y=28*42

static ssize_t debug_raw_data_read(struct mxt_data *mxt, char *buf, size_t count, 
			loff_t *ppos, u8 debug_command){
	int i;
	u16 *data;
	u16 diagnostics_reg;
	int offset = 0;
	int size;
	int read_size;
	int error;
	char *buf_start;
	u16 debug_data_addr;
	u16 page_address;
	u8 page;
	u8 debug_command_reg;

	data = mxt->debug_data;
	if (data == NULL)
		return -EIO;

	/* If first read after open, read all data to buffer. */
	if (mxt->current_debug_datap == 0)
	{
		diagnostics_reg = MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt) + MXT_ADR_T6_DIAGNOSTIC;

		if (count > (CHANNEL_NUM * 2))
			count = CHANNEL_NUM;
	
		debug_data_addr = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, mxt)+ MXT_ADR_T37_DATA;
		page_address = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, mxt) + MXT_ADR_T37_PAGE;
		error = mxt_read_block(mxt->client, page_address, 1, &page);
		if (error < 0)
			return error;

		mxt_debug(DEBUG_TRACE, "[touch_pad] debug data page = %d\n", page);

		while (page != 0)
		{
			error = mxt_write_byte(mxt->client, diagnostics_reg, MXT_CMD_T6_PAGE_DOWN);
			if (error < 0)
				return error;

			/* Wait for command to be handled; when it has, the
			   register will be cleared. */
			debug_command_reg = 1;
			while (debug_command_reg != 0)
			{
				error = mxt_read_block(mxt->client, diagnostics_reg, 1, &debug_command_reg);
				if (error < 0)
					return error;

				mxt_debug(DEBUG_TRACE, "[touch_pad] Waiting for debug diag command to propagate...\n");
			}
			
			error = mxt_read_block(mxt->client, page_address, 1, &page);
			if (error < 0)
				return error;

			mxt_debug(DEBUG_TRACE, "[touch_pad] debug data page = %d\n", page);
		}

		/*
		 * Lock mutex to prevent writing some unwanted data to debug
		 * command register. User can still write through the char 
		 * device interface though. TODO: fix?
		 */
		mutex_lock(&mxt->debug_mutex);

		/* Configure Debug Diagnostics object to show deltas/refs */
		error = mxt_write_byte(mxt->client, diagnostics_reg, debug_command);

		if (error < 0)
		{
			printk (KERN_WARNING "[touch_pad] Error writing to maXTouch device!\n");
			return error;
		}

		/* Wait for command to be handled; when it has, the
		 * register will be cleared. */
		debug_command_reg = 1;
		while (debug_command_reg != 0)
		{
			error = mxt_read_block(mxt->client, diagnostics_reg, 1, &debug_command_reg);
			if (error < 0)
				return error;

			mxt_debug(DEBUG_TRACE, "[touch_pad] Waiting for debug diag command to propagate...\n");
		}

		size = CHANNEL_NUM * sizeof(u16);

		while (size > 0)
		{
			if ( page==7 || page==15 || page==23 )
			{
				goto page_skip;
			}
			else if ( page==6 || page==14 || page==22 )
			{
				read_size = 8 * sizeof(u16);
			}
			else
			{
				read_size = 64 * sizeof(u16);
			}

			mxt_debug(DEBUG_TRACE, "[touch_pad] Debug data read loop, reading %d bytes...\n", read_size);

			error = mxt_read_block(mxt->client, debug_data_addr, read_size, (u8 *) &data[offset]);
			if (error < 0)
			{
				printk(KERN_WARNING "[touch_pad] Error reading debug data\n");
				goto error;
			}

			offset += read_size/2;
			size -= read_size;

page_skip:
			/* Select next page */
			error = mxt_write_byte(mxt->client, diagnostics_reg, MXT_CMD_T6_PAGE_UP);
			if (error < 0)
			{
				printk(KERN_WARNING "[touch_pad] Error writing to maXTouch device!\n");
				goto error;
			}

			/* Wait for command to be handled; when it has, the
			   register will be cleared. */
			debug_command_reg = 1;
			while (debug_command_reg != 0)
			{
				error = mxt_read_block(mxt->client, diagnostics_reg, 1, &debug_command_reg);
				if (error < 0)
					return error;

				mxt_debug(DEBUG_TRACE, "[touch_pad] Waiting for debug diag command to propagate...\n");
			}

			error = mxt_read_block(mxt->client, page_address, 1, &page);
			if (error < 0)
				return error;

			mxt_debug(DEBUG_TRACE, "[touch_pad] debug data page = %d\n", page);
		}

		mutex_unlock(&mxt->debug_mutex);
	}

	buf_start = buf;
	i = mxt->current_debug_datap;

	while (((buf- buf_start) < (count - 6)) && (i < CHANNEL_NUM))
	{
		mxt->current_debug_datap++;

		if (debug_command == MXT_CMD_T6_REFERENCES_MODE)
			buf += sprintf(buf, "%5d\n",(u16) le16_to_cpu(data[i]));
		else if (debug_command == MXT_CMD_T6_DELTAS_MODE)
			buf += sprintf(buf, "%5d\n",(s16) le16_to_cpu(data[i]));
		i++;
	}

	return (buf - buf_start);

error:
	mutex_unlock(&mxt->debug_mutex);
	return error;
}
//ASUS_BSP simpson: add for ATD reading raw data ---

static ssize_t deltas_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return debug_data_read(file->private_data, buf, count, ppos, 
			       MXT_CMD_T6_DELTAS_MODE);
}

static ssize_t refs_read(struct file *file, char *buf, size_t count, 
			loff_t *ppos)
{
	return debug_data_read(file->private_data, buf, count, ppos, 
			       MXT_CMD_T6_REFERENCES_MODE);
}

//ASUS_BSP simpson: add for ATD reading raw data +++
static ssize_t raw_data_read(struct file *file, char *buf, size_t count, 
			loff_t *ppos)
{
	return debug_raw_data_read(file->private_data, buf, count, ppos, 
			       MXT_CMD_T6_REFERENCES_MODE);
}
//ASUS_BSP simpson: add for ATD reading raw data ---

static int debug_data_open(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	int i;
	mxt = inode->i_private;
	if (mxt == NULL)
		return -EIO;
	mxt->current_debug_datap = 0;
	mxt->debug_data = kmalloc(mxt->device_info.num_nodes * sizeof(u16),
				  GFP_KERNEL);
	if (mxt->debug_data == NULL)
		return -ENOMEM;

	
	for (i = 0; i < mxt->device_info.num_nodes; i++)
		mxt->debug_data[i] = 7777;
	

	file->private_data = mxt;
	return 0;
}

static int debug_data_release(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	mxt = file->private_data;
	kfree(mxt->debug_data);
	return 0;
}

static struct file_operations delta_fops = {
	.owner = THIS_MODULE,
	.open = debug_data_open,
	.release = debug_data_release,
	.read = deltas_read,
};

static struct file_operations refs_fops = {
	.owner = THIS_MODULE,
	.open = debug_data_open,
	.release = debug_data_release,
	.read = refs_read,
};

//ASUS_BSP simpson: add for ATD reading raw data +++
static struct file_operations raw_data_fops = {
	.owner = THIS_MODULE,
	.open = debug_data_open,
	.release = debug_data_release,
	.read = raw_data_read,
};
//ASUS_BSP simpson: add for ATD reading raw data ---


/* Calculates the 24-bit CRC sum. */
static u32 CRC_24(u32 crc, u8 byte1, u8 byte2)
{
	static const u32 crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = ((((u16) byte2) << 8u) | byte1);
	result = ((crc << 1u) ^ data_word);
	if (result & 0x1000000)
		result ^= crcpoly;
	return result;
}

/* Returns object address in mXT chip, or zero if object is not found */
static u16 get_object_address(uint8_t object_type,
			      uint8_t instance,
			      struct mxt_object *object_table,
			      int max_objs)
{
	uint8_t object_table_index = 0;
	uint8_t address_found = 0;
	uint16_t address = 0;
	struct mxt_object *obj;

	while ((object_table_index < max_objs) && !address_found) {
		obj = &object_table[object_table_index];
		if (obj->type == object_type) {
			address_found = 1;
			/* Are there enough instances defined in the FW? */
			if (obj->instances >= instance) {
				address = obj->chip_addr +
					  (obj->size + 1) * instance;
			} else {
				return 0;
			}
		}
		object_table_index++;
	}
	return address;
}


/*
 * Reads a block of bytes from given address from mXT chip. If we are
 * reading from message window, and previous read was from message window,
 * there's no need to write the address pointer: the mXT chip will
 * automatically set the address pointer back to message window start.
 */

static int mxt_read_block(struct i2c_client *client,
		   u16 addr,
		   u16 length,
		   u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	__le16	le_addr;
	struct mxt_data *mxt;
	int iRetries = 0;

	mxt = i2c_get_clientdata(client);

	if (mxt != NULL)
	{
		if ((mxt->last_read_addr == addr) && (addr == mxt->msg_proc_addr))
		{
			do
			{
				if  (i2c_master_recv(client, value, length) == length)
					return length;

//				dev_err(&client->dev, "[touch_pad] %s: i2c_master_recv failed: iRetries=%d\n", __func__, iRetries);

				msleep(I2C_WAKE_TIME);
			} while(++iRetries < I2C_RETRY_COUNT);

			if (iRetries >= I2C_RETRY_COUNT)
				goto i2c_error;
		}
		else
		{
			mxt->last_read_addr = addr;
		}
	}

	mxt_debug(DEBUG_TRACE, "[touch_pad] Writing address pointer & reading %d bytes "
		"in on i2c transaction...\n", length); 

	le_addr = cpu_to_le16(addr);
	msg[0].addr  = client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *) &le_addr;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = length;
	msg[1].buf   = (u8 *) value;

	do
	{
		if  (i2c_transfer(adapter, msg, 2) == 2)
			return length;

//		dev_err(&client->dev, "[touch_pad] %s: i2c_transfer failed: iRetries=%d\n", __func__, iRetries);

		msleep(I2C_WAKE_TIME);
	} while(++iRetries < I2C_RETRY_COUNT);

i2c_error:
	dev_err(&client->dev, "[touch_pad] %s: i2c transfer failed\n", __func__);
	
	return -EIO;
}


/* Writes one byte to given address in mXT chip. */
static int mxt_write_byte(struct i2c_client *client, u16 addr, u8 value)
{
	struct {
		__le16 le_addr;
		u8 data;

	} i2c_byte_transfer;

	struct mxt_data *mxt;
	int iRetries = 0;

	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;

	i2c_byte_transfer.le_addr = cpu_to_le16(addr);
	i2c_byte_transfer.data = value;

	do
	{
		if  (i2c_master_send(client, (u8 *) &i2c_byte_transfer, 3) == 3)
			return 0;

//		dev_err(&client->dev, "[touch_pad] %s: i2c_master_send failed: iRetries=%d\n", __func__, iRetries);

		msleep(I2C_WAKE_TIME);
	} while(++iRetries < I2C_RETRY_COUNT);

	dev_err(&client->dev, "[touch_pad] %s: i2c transfer failed\n", __func__);

	return -EIO;
}


/* Writes a block of bytes (max 256) to given address in mXT chip. */
static int mxt_write_block(struct i2c_client *client,
		    u16 addr,
		    u16 length,
		    u8 *value)
{
	int i;
	struct {
		__le16	le_addr;
		u8	data[256];

	} i2c_block_transfer;

	struct mxt_data *mxt;
	int iRetries = 0;

	mxt_debug(DEBUG_TRACE, "[touch_pad] Writing %d bytes to %d...", length, addr);

	if (length > 256)
		return -EINVAL;

	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;

	for (i = 0; i < length; i++)
		i2c_block_transfer.data[i] = *value++;

	i2c_block_transfer.le_addr = cpu_to_le16(addr);

	do
	{
		i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);
		if (i == (length + 2))
			return length;

//		dev_err(&client->dev, "[touch_pad] %s: i2c_master_send failed: iRetries=%d\n", __func__, iRetries);

		msleep(I2C_WAKE_TIME);
	} while(++iRetries < I2C_RETRY_COUNT);

	dev_err(&client->dev, "[touch_pad] %s: i2c transfer failed\n", __func__);

	return -EIO;
}

/* Calculates the CRC value for mXT infoblock. */
static int calculate_infoblock_crc(u32 *crc_result, u8 *data, int crc_area_size)
{
	u32 crc = 0;
	int i;

	for (i = 0; i < (crc_area_size - 1); i = i + 2)
		crc = CRC_24(crc, *(data + i), *(data + i + 1));
	/* If uneven size, pad with zero */
	if (crc_area_size & 0x0001)
		crc = CRC_24(crc, *(data + i), 0);
	/* Return only 24 bits of CRC. */
	*crc_result = (crc & 0x00FFFFFF);

	return 0;
}

//ASUS_BSP joe1_++
/* Processes a touchscreen message. */
static void process_T9_message(u8 *message, struct mxt_data *mxt, int last_touch)
{
	u8  status;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8  touch_size = 255;
	u8  touch_number;
//	u8  amplitude = 0xFF;
	u8  report_id;
	int i;
	int active_touches = 0;
	struct finger_info *finger = mxt->finger;
	struct first_finger_info *first_finger = mxt->first_finger;
	bool bCheckFingerMove = false;
//	bool bDisableCheckLockPoint = false;

#ifdef CAL_REPORT_RATE
	static u32 u32TouchCount = 0;
	static u64 u64timestamp_start = 0;
	u64 u64timestamp_end = 0;
#endif

	/*
	 * If the 'last_touch' flag is set, we have received all the touch
	 * messages
	 * there are available in this cycle, so send the events for touches 
	 * that are 
  	 * active.
 	 */ 
	if (last_touch)
	{
	        /* TODO: For compatibility with single-touch systems, send ABS_X & 
		 * ABS_Y */
	        /*
	        if (finger[0].area){
	            input_report_abs(mxt->input, ABS_X, finger[0].x);
	            input_report_abs(mxt->input, ABS_Y, finger[0].y);
	        }*/
    
		for (i = 0; i < MAX_FINGER_NUM; i++)
		{
			if (finger[i].area)
			{
				active_touches++;

				input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR, finger[i].area);
				input_report_abs(mxt->input, ABS_MT_POSITION_X, finger[i].x);
				input_report_abs(mxt->input, ABS_MT_POSITION_Y, finger[i].y);
#ifdef SUPPORT_PRESSURE
				input_report_abs(mxt->input, ABS_MT_PRESSURE, finger[i].amplitude);
				input_report_abs(mxt->input, ABS_MT_ORIENTATION, finger[i].vector);
#endif

				input_mt_sync(mxt->input);
			}
		}

		input_report_key(mxt->input, BTN_TOUCH, !!active_touches);

		if (active_touches == 0)
			input_mt_sync(mxt->input);

		input_sync(mxt->input);

//		mxt_debug(DEBUG_TRACE, "[touch_pad] process_T9_message: input_sync; active_touches=%d\n", active_touches);
	}
	else
	{
		status = message[MXT_MSG_T9_STATUS];
		report_id = message[0];
		touch_number = message[MXT_MSG_REPORTID] - mxt->rid_map[report_id].first_rid;

		finger[touch_number].status = status;

		/* Put together the 10-/12-bit coordinate values. */
		xpos = message[MXT_MSG_T9_XPOSMSB] * 16 +
			((message[MXT_MSG_T9_XYPOSLSB] >> 4) & 0xF);
		ypos = message[MXT_MSG_T9_YPOSMSB] * 16 +
			((message[MXT_MSG_T9_XYPOSLSB] >> 0) & 0xF);

		if (mxt->max_x_val < 1024)
			xpos >>= 2;
		if (mxt->max_y_val < 1024)
			ypos >>= 2;

		finger[touch_number].x = xpos;
		finger[touch_number].y = ypos;

		if (status & MXT_MSGB_T9_DETECT)
		{
			/*
			 * mXT224 reports the number of touched nodes,
			 * so the exact value for touch ellipse major
			 * axis length in nodes would be 2*sqrt(touch_size/pi)
			 * (assuming round touch shape), which would then need
			 * to be scaled using information about how many sensor
			 * lines we do have. So this is very much simplified,
			 * but sufficient for most if not all apps?
			 */
			touch_size = message[MXT_MSG_T9_TCHAREA];
//			touch_size = touch_size >> 2; //ASUS_BSP joe1_++
			if (!touch_size)
				touch_size = 1;

			finger[touch_number].area = touch_size;

			finger[touch_number].amplitude = message[MXT_MSG_T9_TCHAMPLITUDE];

			finger[touch_number].vector = message[MXT_MSG_T9_TCHVECTOR];

#if 0
			if (status & MXT_MSGB_T9_AMP)
			{
				/* Amplitude of touch has changed */
				amplitude = message[MXT_MSG_T9_TCHAMPLITUDE];

//				finger[touch_number].amplitude = message[MXT_MSG_T9_TCHAMPLITUDE]; //ASUS_BSP joe1_++
			}
#endif

			if ( g_bIsCmdCalibrationOk )
			{
				if ( status & MXT_MSGB_T9_PRESS )
				{
					first_finger[touch_number].status = 1;

					first_finger[touch_number].start_x = finger[touch_number].x;
					first_finger[touch_number].start_y = finger[touch_number].y;

					if ( !g_bIsCheckLockPointAlready && !g_bIsCheckLockPoint )
					{
						queue_delayed_work(g_atmel_wq_dis_auto_cal, &mxt->d_work_check_lock_point, msecs_to_jiffies(LOCK_POINT_DELAY));

						g_bIsCheckLockPoint = true;

						//printk("[touch_pad] g_bIsCmdCalibrationOk:press: g_bIsCheckLockPoint =%d\n", g_bIsCheckLockPoint);
					}

					//printk("[touch_pad] g_bIsCmdCalibrationOk:press: touch_number=%d; start_x=%d; start_y=%d\n", touch_number, first_finger[touch_number].start_x, first_finger[touch_number].start_y);
				}
			}
		}
		else if (status & (MXT_MSGB_T9_RELEASE|MXT_MSGB_T9_SUPPRESS))
		{
			//The touch has been removed or suppressed
			finger[touch_number].area = 0;

			finger[touch_number].amplitude = 0;

			finger[touch_number].vector = 0;

			if ( g_bIsCmdCalibrationOk )
			{
				first_finger[touch_number].status = 0;

				for ( i = 0; i < MAX_FINGER_NUM; i++ )
				{
					if ( first_finger[i].status == 1 )
					{
						bCheckFingerMove = false;
//						bDisableCheckLockPoint = false;

						//printk("[touch_pad] g_bIsCmdCalibrationOk:release:i=%d; status=%d: NOT CheckFingerMove\n", i, first_finger[i].status);
							
						break;
					}
					else
					{
						bCheckFingerMove = true;
//						bDisableCheckLockPoint = true;

						//printk("[touch_pad] g_bIsCmdCalibrationOk:release:i=%d; status=%d: CheckFingerMove!!!\n", i, first_finger[i].status);
					}
				}

#if 0
				if ( bDisableCheckLockPoint )
				{
					cancel_delayed_work_sync(&mxt->d_work_check_lock_point);

					g_bIsCheckLockPoint = false;

					printk("[touch_pad] disable check lock!!!\n");
				}
				else
				{
					printk("[touch_pad] NOT disable check lock\n");
				}
#endif

				if ( bCheckFingerMove )
				{
					if ( !g_bIsCheckLockPointAlready )
					{
						cancel_delayed_work_sync(&mxt->d_work_check_lock_point);
						g_bIsCheckLockPoint = false;
						printk("[touch_pad] disable check lock!!!\n");
					}

					first_finger[touch_number].dx = abs(finger[touch_number].x - first_finger[touch_number].start_x);
					first_finger[touch_number].dy = abs(finger[touch_number].y - first_finger[touch_number].start_y);
//					first_finger[touch_number].dsum = abs(first_finger[touch_number].dx) + abs(first_finger[touch_number].dy);

					printk("[touch_pad] bCheckFingerMove: touch_number=%d; dx=%d;dy=%d\n", touch_number, first_finger[touch_number].dx, first_finger[touch_number].dy);

//					if ( first_finger[touch_number].dsum > MOVE_DISTANCE)
					if ( first_finger[touch_number].dx > MOVE_DISTANCE_X || first_finger[touch_number].dy > MOVE_DISTANCE_Y )
					{
						queue_delayed_work(g_atmel_wq_dis_auto_cal, &mxt->d_work_disable_auto_calibration, msecs_to_jiffies(DISABLE_AUTO_CALIBRATION_DELAY));

						g_bIsCmdCalibrationOk = false;

						g_bIsCheckLockPointAlready = false;

						printk("[touch_pad] bCheckFingerMove: Disable auto-cal!!!\n");
					}

					printk("[touch_pad] CheckFingerMove!!!\n");
				}
				else
				{
					printk("[touch_pad] NOT disable check lock\n");
					printk("[touch_pad] NOT CheckFingerMove\n");
				}
			}
		}

#ifdef CAL_REPORT_RATE
		if ( u64timestamp_start == 0 )
		{
			u64timestamp_start = get_jiffies_64();
		}
		else
		{
			u64timestamp_end = get_jiffies_64();

			u32TouchCount++;

			if ( ((u64timestamp_end -u64timestamp_start)/HZ) >= 5 ) //5 sec.
			{
				printk("[touch_pad] report rate= %d\n", u32TouchCount/5);

				u64timestamp_start = 0;
				u32TouchCount = 0;
			}
		}

		if (status & (MXT_MSGB_T9_RELEASE|MXT_MSGB_T9_SUPPRESS))
		{
			u64timestamp_start = 0;
			u32TouchCount = 0;
		}
#endif
	
		if ( debug >= DEBUG_TRACE )
		{
			if (status & MXT_MSGB_T9_SUPPRESS) 
			{
				printk("[touch_pad] SUPRESS\n");
			}
			else
			{
				if (status & MXT_MSGB_T9_DETECT)
				{
					printk("[touch_pad] DETECT:%s%s%s%s\n", 
						((status & MXT_MSGB_T9_PRESS) ? " PRESS" : ""), 
						((status & MXT_MSGB_T9_MOVE) ? " MOVE" : ""), 
						((status & MXT_MSGB_T9_AMP) ? " AMP" : ""), 
						((status & MXT_MSGB_T9_VECTOR) ? " VECT" : ""));
				}
				else if (status & MXT_MSGB_T9_RELEASE)
				{
					printk("[touch_pad] RELEASE\n");
				}
			}

			printk("[touch_pad] X=%d, Y=%d, TOUCHSIZE=%d, Amplitude=%d, VECTOR=0x%x\n", 
				finger[touch_number].x,
				finger[touch_number].y,
				finger[touch_number].area,
				finger[touch_number].amplitude,
				finger[touch_number].vector);
		}
	}

	return;
}


static int process_message(u8 *message, u8 object, struct mxt_data *mxt)
{
	struct i2c_client *client = mxt->client;
	u8  status;
	u8  buf[3];
	u32 cfg_crc = 0;
	u32 cfg_crc_onchip;
	static int cal_type = -1;

	switch (object)
	{
		case MXT_GEN_COMMANDPROCESSOR_T6:
			//ASUS_BSP joe1_+++
			buf[0]=message[2];
			buf[1]=message[3];
			buf[2]=message[4];
			cfg_crc_onchip = buf[2] << 16 | buf[1] <<8 | buf[0];
			mxt->configuration_crc = cfg_crc_onchip;
			printk("[touch_pad] configuration checksum is 0x%lx\n",(long unsigned int)cfg_crc_onchip);
			mxt_debug(DEBUG_INFO, "[touch_pad] mxt->configuration_crc = 0x%lx\n",(long unsigned int)mxt->configuration_crc);

			if (cfg_flag)
			{
				if ( mxt->device_info.family_id == MXT1386_FAMILYID ) //mXT1386
				{
					if ( (mxt->device_info.major == 1) && (mxt->device_info.minor == 0 ) )
					{
						cfg_crc = 0xf420c3;
					}
					else
					{
						cfg_crc = 0x31479e;
					}

					printk("[touch_pad] MXT1386_FAMILYID [%d.%d]: cfg_crc = 0x%lx\n",mxt->device_info.major,mxt->device_info.minor,(long unsigned int)cfg_crc);
				}
				else if ( mxt->device_info.family_id == MXT768_FAMILYID ) //mXT768
				{
					cfg_crc = 0x701758;

					printk("[touch_pad] MXT768_FAMILYID: cfg_crc = 0x%lx\n",(long unsigned int)cfg_crc);
				}
				else
				{
					printk("[touch_pad] UNKNOWN_FAMILYID: cfg_crc = 0x%lx\n",(long unsigned int)cfg_crc);
				}

				if (cfg_crc_onchip != cfg_crc)
				{
					printk("[touch_pad] start BACKUP\n");

					init_touch_config(mxt);

					// save power state values for suspend/resume
					mxt_read_block(mxt->client, mxt->t7_addr, ARRAY_SIZE(mxt->t7_data), mxt->t7_data);

					printk("[touch_pad] config BACKUP finished\n");
				}
				else
				{
					printk("[touch_pad] config dont need BACKUP!!!\n");
				}

				cfg_flag = false;
			}
			//ASUS_BSP joe1_---

			status = message[1];
			if (status & MXT_MSGB_T6_COMSERR)
				dev_err(&client->dev, "[touch_pad] maXTouch checksum error\n");
			if (status & MXT_MSGB_T6_CFGERR) {
				/* 
				 * Configuration error. A proper configuration
				 * needs to be written to chip and backed up.
				 */
				dev_err(&client->dev, "[touch_pad] maXTouch configuration error\n");
				//ASUS_BSP simpson: turn on for fail-safe+++
				init_touch_config(mxt);

				// save power state values for suspend/resume
				mxt_read_block(mxt->client, mxt->t7_addr, ARRAY_SIZE(mxt->t7_data), mxt->t7_data);
				//ASUS_BSP simpson: turn on for fail-safe---
			}
			if (status & MXT_MSGB_T6_CAL) {
				if ( g_bIsCmdCalibration )
				{
					cal_type = CAL_TYPE_CMD;
				}
				else if ( g_bIsCheckLockPointAlready)
				{
					cal_type = CAL_TYPE_LOCK_POINT;
				}
				
				/* Calibration in action, no need to react */
				dev_info(&client->dev, "[touch_pad] maXTouch calibration in progress\n");
			}
			if (status & MXT_MSGB_T6_SIGERR) {
				/* 
				 * Signal acquisition error, something is seriously
				 * wrong, not much we can in the driver to correct
				 * this
				 */
				dev_err(&client->dev, "[touch_pad] maXTouch acquisition error\n");
			}
			if (status & MXT_MSGB_T6_OFL) {
				/*
				 * Cycle overflow, the acquisition is too short.
				 * Can happen temporarily when there's a complex
				 * touch shape on the screen requiring lots of
				 * processing.
				 */
				dev_err(&client->dev, "[touch_pad] maXTouch cycle overflow\n");
			}
			if (status & MXT_MSGB_T6_RESET) {
				/* Chip has reseted, no need to react. */
				dev_info(&client->dev, "[touch_pad] maXTouch chip reset\n");
			}
			if (status == 0) {
				switch ( cal_type )
				{
					case CAL_TYPE_CMD:
						force_release_fingers();

						mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt), T9_CTRL_VAL);

						g_bIsCmdCalibrationOk = true;

						g_bIsCmdCalibration = false;

						cal_type = -1;

						printk("[touch_pad] calibration!!!CAL_TYPE_CMD!\n");

						break;

					case CAL_TYPE_LOCK_POINT:
						force_release_fingers();

						mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt), T9_CTRL_VAL);

						cal_type = -1;

						printk("[touch_pad] calibration!!!CAL_TYPE_LOCK_POINT!\n");

						break;

					default:
						break;
				}
				
				/* Chip status back to normal. */
				dev_info(&client->dev, "[touch_pad] maXTouch status normal\n");
			}
			break;

		case MXT_SPT_SELFTEST_T25:
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev, "[touch_pad] Receiving Self-Test msg\n");

			if (message[MXT_MSG_T25_STATUS] == MXT_MSGR_T25_OK)
			{
				if (debug >= DEBUG_TRACE)
					dev_info(&client->dev, "[touch_pad] maXTouch: Self-Test OK\n");
			}
			else
			{
				dev_err(&client->dev,
					"[touch_pad] maXTouch: Self-Test Failed [%02x]:"
					"{%02x,%02x,%02x,%02x,%02x}\n",
					message[MXT_MSG_T25_STATUS],
					message[MXT_MSG_T25_STATUS + 0],
					message[MXT_MSG_T25_STATUS + 1],
					message[MXT_MSG_T25_STATUS + 2],
					message[MXT_MSG_T25_STATUS + 3],
					message[MXT_MSG_T25_STATUS + 4]
					);
			}
			break;

		case MXT_SPT_CTECONFIG_T28:
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev, "[touch_pad] Receiving CTE message...\n");

			status = message[MXT_MSG_T28_STATUS];

			if (status & MXT_MSGB_T28_CHKERR)
				dev_err(&client->dev, "[touch_pad] maXTouch: Power-Up CRC failure\n");

			break;

		default:
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev, "[touch_pad] maXTouch: Unknown message!\n");

			break;
	}

	return 0;
}

/*
 * The maXTouch device will signal the host about a new message by asserting
 * the CHG line.
 * Processes messages when the interrupt line (CHG) is asserted. Keeps
 * reading messages until a message with report ID 0xFF is received,
 * which indicates that there is no more new messages.
 */
#define MESSAGE_SIZE 8 //total size=9, it doesn't include CRC
#if 0
static int make_chgline_high(struct mxt_data *mxt)
{
	struct i2c_client *client = mxt->client;
	u8 message[MESSAGE_SIZE] = {0};
	u16	message_addr = mxt->msg_proc_addr;
	u16	message_length = MESSAGE_SIZE;//mxt->message_size;
	u8	report_id;
	u8	object = 0;
	int	error;
	int	count = 10;
	int	ret;

	g_bIsMakingChglineHigh = true;

	printk("[touch_pad] make_chgline_high++\n");

	do
	{
		error = mxt_read_block(client,
				       message_addr,
				       message_length,// - 1,
				       message);
		if (error < 0)
		{
			dev_err(&client->dev, "[touch_pad] Failure reading maxTouch device. read_fail_counter=%d\n", mxt->read_fail_counter);
			ret = -EIO;

			goto end;
		}
		
		report_id = message[0];
		
		if ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0))
		{
			object = mxt->rid_map[report_id].object;

			if ( object != MXT_TOUCH_MULTITOUCHSCREEN_T9 )
			{
				process_message(message, object, mxt);
			}
		}

//		mxt_debug(DEBUG_TRACE, "[touch_pad] chgline: %d\n", mxt->read_chg());
	} while ( (report_id != MXT_END_OF_MESSAGES) && (report_id != 0) && (--count) );

	if (!count)
	{
		dev_err(&client->dev, "[touch_pad] chgline isn't cleared\n");
		ret = -EBUSY;

		goto end;
	}

	ret = 1;

end:
	printk("[touch_pad] make_chgline_high--\n");

	g_bIsMakingChglineHigh = false;

	return ret;
}
#endif
//ASUS_BSP joe1_--

static irqreturn_t mxt_irq_handler(int irq, void *_mxt)
{
	struct mxt_data *mxt = _mxt;
	struct i2c_client *client = mxt->client;
	u8 message[MESSAGE_SIZE] = {0};
	u16	message_addr = mxt->msg_proc_addr;
	u16	message_length = MESSAGE_SIZE;//mxt->message_size;
	u8	report_id;
	u8	object = 0;
	int	error;
	int	i;
	char    *message_string;
	char    *message_start;

//	mxt_debug(DEBUG_TRACE, "[touch_pad] mxt_irq_handler:\n");

//ASUS_BSP joe1_++
#if 0
	if ( g_bIsMakingChglineHigh )
	{
//		mxt_debug(DEBUG_POWER, "[touch_pad] g_bIsMakingChglineHigh!\n");

		goto end_irq;
	}
#endif
//ASUS_BSP joe1_--

	do
	{
		/* TODO: message length, CRC included? */
		error = mxt_read_block(client,
				       message_addr,
				       message_length,// - 1,
				       message);
		if (error < 0)
		{
			mxt->read_fail_counter++;
			dev_err(&client->dev, "[touch_pad] Failure reading maxTouch device. read_fail_counter=%d\n", mxt->read_fail_counter);

			force_release_fingers();

			if ( mxt->read_fail_counter == 6 )
			{
				schedule_delayed_work(&mxt->d_work_disable_irq, 0);

				printk("[touch_pad] read_fail_counter=%d > 5; disable_irq(mxt->irq)!Disable touch!!!\n", mxt->read_fail_counter);
			}

			goto end_irq;
		}
		
		report_id = message[0];

		if (debug >= DEBUG_RAW)
		{
			printk("[touch_pad] %s message :\n", REPORT_ID_TO_OBJECT_NAME(report_id, mxt));

			/* 5 characters per one byte */
			message_string = kmalloc(message_length * 5, GFP_KERNEL);

			if (message_string == NULL)
			{
				dev_err(&client->dev, "[touch_pad] Error allocating memory\n");

				kfree(message);

				goto end_irq;
			}

			message_start = message_string;

			for (i = 0; i < message_length; i++)
			{
				message_string += sprintf(message_string, "0x%02X ", message[i]);
			}

			printk("[touch_pad] %s\n", message_start);

			kfree(message_start);
		}
		
		if ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0))
		{
			/* Get type of object and process the message */
			object = mxt->rid_map[report_id].object;

			if ( object == MXT_TOUCH_MULTITOUCHSCREEN_T9 )
			{
				process_T9_message(message, mxt, 0);
			}
			else
			{
				process_message(message, object, mxt);
			}
		}

//		mxt_debug(DEBUG_TRACE, "[touch_pad] chgline: %d\n", mxt->read_chg());
	} while ( (report_id != MXT_END_OF_MESSAGES) && (report_id != 0) );

	/* All messages processed, send the events) */
	process_T9_message(NULL, mxt, 1);

end_irq:
	return IRQ_HANDLED;
}
//ASUS_BSP joe1_--

/******************************************************************************/
/* Initialization of driver                                                   */
/******************************************************************************/

static int mxt_identify(struct i2c_client *client,
				  struct mxt_data *mxt,
				  u8 *id_block_data)
{
	u8 buf[MXT_ID_BLOCK_SIZE];
	int error;
	int identified;

	identified = 0;

	/* Read Device info to check if chip is valid */
	error = mxt_read_block(client, MXT_ADDR_INFO_BLOCK, MXT_ID_BLOCK_SIZE,
			       (u8 *) buf);

	if (error < 0) {
		mxt->read_fail_counter++;
		dev_err(&client->dev, "[touch_pad] Failure accessing maXTouch device\n");
		return -EIO;
	}

	memcpy(id_block_data, buf, MXT_ID_BLOCK_SIZE);

	mxt->device_info.family_id  = buf[0];
	mxt->device_info.variant_id = buf[1];
	mxt->device_info.major	    = ((buf[2] >> 4) & 0x0F);
	mxt->device_info.minor      = (buf[2] & 0x0F);
	mxt->device_info.build	    = buf[3];
	mxt->device_info.x_size	    = buf[4];
	mxt->device_info.y_size	    = buf[5];
	mxt->device_info.num_objs   = buf[6];
	mxt->device_info.num_nodes  = mxt->device_info.x_size *
				      mxt->device_info.y_size;

	/*
         * Check Family & Variant Info; warn if not recognized but
         * still continue.
         */

	/* MXT224 */
	if (mxt->device_info.family_id == MXT224_FAMILYID) {
		strcpy(mxt->device_info.family_name, "mXT224");

		if (mxt->device_info.variant_id == MXT224_CAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Calibrated");
		} else if (mxt->device_info.variant_id == 
			MXT224_UNCAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Uncalibrated");
		} else {
			dev_err(&client->dev,
				"[touch_pad] Warning: maXTouch Variant ID [%d] not "
				"supported\n",
				mxt->device_info.variant_id);
			strcpy(mxt->device_info.variant_name, "UNKNOWN");
			identified = -ENXIO; //ASUS_BSP joe1_++
		}

	/* MXT768 */
	} else if (mxt->device_info.family_id == MXT768_FAMILYID) {
		strcpy(mxt->device_info.family_name, "mXT768");

		if (mxt->device_info.variant_id == MXT768_CAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Calibrated");
		} else {
			dev_err(&client->dev,
				"[touch_pad] Warning: maXTouch Variant ID [%d] not "
				"supported\n",
				mxt->device_info.variant_id);
			strcpy(mxt->device_info.variant_name, "UNKNOWN");
			/* identified = -ENXIO; */
		}
	/* MXT1386 */
	} else if (mxt->device_info.family_id == MXT1386_FAMILYID) {
		strcpy(mxt->device_info.family_name, "mXT1386");

		if (mxt->device_info.variant_id == MXT1386_CAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Calibrated");
		} else {
			dev_err(&client->dev,
				"[touch_pad] Warning: maXTouch Variant ID [%d] not "
				"supported\n",
				mxt->device_info.variant_id);
			strcpy(mxt->device_info.variant_name, "UNKNOWN");
			/* identified = -ENXIO; */
		}
	/* Unknown family ID! */
	} else {
		dev_err(&client->dev,
			"[touch_pad] Warning: maXTouch Family ID [%d] not supported\n",
			mxt->device_info.family_id);
		strcpy(mxt->device_info.family_name, "UNKNOWN");
		strcpy(mxt->device_info.variant_name, "UNKNOWN");
		identified = -ENXIO; //ASUS_BSP joe1_++
	}

	dev_info(
		&client->dev,
		"[touch_pad] Atmel maXTouch (Family %s (%X), Variant %s (%X)) Firmware "
		"version [%d.%d] Build %d\n",
		mxt->device_info.family_name,
		mxt->device_info.family_id,
		mxt->device_info.variant_name,
		mxt->device_info.variant_id,
		mxt->device_info.major,
		mxt->device_info.minor,
		mxt->device_info.build
	);
	dev_dbg(
		&client->dev,
		"[touch_pad] Atmel maXTouch Configuration "
		"[X: %d] x [Y: %d]\n",
		mxt->device_info.x_size,
		mxt->device_info.y_size
	);
	return identified;
}

/*
 * Reads the object table from maXTouch chip to get object data like
 * address, size, report id. For Info Block CRC calculation, already read
 * id data is passed to this function too (Info Block consists of the ID
 * block and object table).
 *
 */
static int __devinit mxt_read_object_table(struct i2c_client *client,
					   struct mxt_data *mxt,
					   u8 *raw_id_data)
{
	u16	report_id_count;
	u8	buf[MXT_OBJECT_TABLE_ELEMENT_SIZE];
	u8      *raw_ib_data;
	u8	object_type;
	u16	object_address;
	u16	object_size;
	u8	object_instances;
	u8	object_report_ids;
	u16	object_info_address;
	u32	crc;
	u32     calculated_crc;
	int	i;
	int	error;

	u8	object_instance;
	u8	object_report_id;
	u8	report_id;
	int     first_report_id;
	int     ib_pointer;
	struct mxt_object *object_table;

	mxt_debug(DEBUG_TRACE, "[touch_pad] maXTouch driver reading configuration\n");

	object_table = kzalloc(sizeof(struct mxt_object) *
			       mxt->device_info.num_objs,
			       GFP_KERNEL);
	if (object_table == NULL) {
		printk(KERN_WARNING "[touch_pad] maXTouch: Memory allocation failed!\n");
		error = -ENOMEM;
		goto err_object_table_alloc;
	}

	raw_ib_data = kmalloc(MXT_OBJECT_TABLE_ELEMENT_SIZE *
			mxt->device_info.num_objs + MXT_ID_BLOCK_SIZE,
			GFP_KERNEL);
	if (raw_ib_data == NULL) {
		printk(KERN_WARNING "[touch_pad] maXTouch: Memory allocation failed!\n");
		error = -ENOMEM;
		goto err_ib_alloc;
	}

	/* Copy the ID data for CRC calculation. */
	memcpy(raw_ib_data, raw_id_data, MXT_ID_BLOCK_SIZE);
	ib_pointer = MXT_ID_BLOCK_SIZE;

	mxt->object_table = object_table;

	mxt_debug(DEBUG_TRACE, "[touch_pad] maXTouch driver Memory allocated\n");

	object_info_address = MXT_ADDR_OBJECT_TABLE;

	report_id_count = 0;
	for (i = 0; i < mxt->device_info.num_objs; i++) {
		mxt_debug(DEBUG_TRACE, "[touch_pad] Reading maXTouch at [0x%04x]: ",
			  object_info_address);

		error = mxt_read_block(client, object_info_address,
				       MXT_OBJECT_TABLE_ELEMENT_SIZE, buf);

		if (error < 0) {
			mxt->read_fail_counter++;
			dev_err(&client->dev,
				"[touch_pad] maXTouch Object %d could not be read\n", i);
			error = -EIO;
			goto err_object_read;
		}

		memcpy(raw_ib_data + ib_pointer, buf, 
		       MXT_OBJECT_TABLE_ELEMENT_SIZE);
		ib_pointer += MXT_OBJECT_TABLE_ELEMENT_SIZE;

		object_type       =  buf[0];
		object_address    = (buf[2] << 8) + buf[1];
		object_size       =  buf[3] + 1;
		object_instances  =  buf[4] + 1;
		object_report_ids =  buf[5];
		mxt_debug(DEBUG_TRACE, "[touch_pad] Type=%03d, Address=0x%04x, "
			  "Size=0x%02x, %d instances, %d report id's\n",
			  object_type,
			  object_address,
			  object_size,
			  object_instances,
			  object_report_ids
		);

		if (object_type == 38)
			t38_size = object_size;
		/* TODO: check whether object is known and supported? */
		
		/* Save frequently needed info. */
		if (object_type == MXT_GEN_MESSAGEPROCESSOR_T5) {
			mxt->msg_proc_addr = object_address;
			mxt->message_size = object_size;
		}

		object_table[i].type            = object_type;
		object_table[i].chip_addr       = object_address;
		object_table[i].size            = object_size;
		object_table[i].instances       = object_instances;
		object_table[i].num_report_ids  = object_report_ids;
		report_id_count += object_instances * object_report_ids;

		object_info_address += MXT_OBJECT_TABLE_ELEMENT_SIZE;
	}

	mxt->rid_map =
		kzalloc(sizeof(struct report_id_map) * (report_id_count + 1),
			/* allocate for report_id 0, even if not used */
			GFP_KERNEL);
	if (mxt->rid_map == NULL) {
		printk(KERN_WARNING "[touch_pad] maXTouch: Can't allocate memory!\n");
		error = -ENOMEM;
		goto err_rid_map_alloc;
	}

	mxt->report_id_count = report_id_count;
	if (report_id_count > 254) {	/* 0 & 255 are reserved */
			dev_err(&client->dev,
				"[touch_pad] Too many maXTouch report id's [%d]\n",
				report_id_count);
			error = -ENXIO;
			goto err_max_rid;
	}

	/* Create a mapping from report id to object type */
	report_id = 1; /* Start from 1, 0 is reserved. */

	/* Create table associating report id's with objects & instances */
	for (i = 0; i < mxt->device_info.num_objs; i++) {
		for (object_instance = 0;
		     object_instance < object_table[i].instances;
		     object_instance++){
			first_report_id = report_id;
			for (object_report_id = 0;
			     object_report_id < object_table[i].num_report_ids;
			     object_report_id++) {
				mxt->rid_map[report_id].object =
					object_table[i].type;
				mxt->rid_map[report_id].instance =
					object_instance;
				mxt->rid_map[report_id].first_rid =
					first_report_id;
				report_id++;
			}
		}
	}

	/* Read 3 byte CRC */
	error = mxt_read_block(client, object_info_address, 3, buf);
	if (error < 0) {
		mxt->read_fail_counter++;
		dev_err(&client->dev, "[touch_pad] Error reading CRC\n");
	}

	crc = (buf[2] << 16) | (buf[1] << 8) | buf[0];

	if (calculate_infoblock_crc(&calculated_crc, raw_ib_data,
				    ib_pointer)) {
		printk(KERN_WARNING "[touch_pad] Error while calculating CRC!\n");
		calculated_crc = 0;
	}
	kfree(raw_ib_data);

	mxt_debug(DEBUG_TRACE, "\n[touch_pad] Reported info block CRC = 0x%6X\n", crc);
	mxt_debug(DEBUG_TRACE, "[touch_pad] Calculated info block CRC = 0x%6X\n\n",
		       calculated_crc);
	
	if (crc == calculated_crc) {
		mxt->info_block_crc = crc;
	} else {
		mxt->info_block_crc = 0;
		printk(KERN_ALERT "[touch_pad] maXTouch: Info block CRC invalid!\n");
	}

	if (debug >= DEBUG_VERBOSE) {

		dev_info(&client->dev, "[touch_pad] maXTouch: %d Objects\n",
				mxt->device_info.num_objs);

		for (i = 0; i < mxt->device_info.num_objs; i++) {
			dev_info(&client->dev, "[touch_pad] Type:\t\t\t[%d]: %s\n",
				 object_table[i].type,
				 object_type_name[object_table[i].type]);
			dev_info(&client->dev, "[touch_pad] \tAddress:\t0x%04X\n",
				object_table[i].chip_addr);
			dev_info(&client->dev, "[touch_pad] \tSize:\t\t%d Bytes\n",
				 object_table[i].size);
			dev_info(&client->dev, "[touch_pad] \tInstances:\t%d\n",
				 object_table[i].instances);
			dev_info(&client->dev, "[touch_pad] \tReport Id's:\t%d\n",
				 object_table[i].num_report_ids);
		}
	}

	return 0;


err_max_rid:
	kfree(mxt->rid_map);
err_rid_map_alloc:
err_object_read:
	kfree(raw_ib_data);
err_ib_alloc:
	kfree(object_table);
err_object_table_alloc:
	return error;
}

#if defined(CONFIG_PM)
//ASUS_BSP joe1_++
static int mxt_suspend(struct device *dev)
{
	struct mxt_data *mxt = dev_get_drvdata(dev);
	int error;
	u8 t7_deepsl_data[T7_DATA_SIZE] = {0};

	mxt_debug(DEBUG_POWER, "[touch_pad] mxt_suspend()++\n");

	if (mxt->is_suspended)
		return 0;

	cancel_delayed_work_sync(&mxt->d_work_do_calibration);
	cancel_delayed_work_sync(&mxt->d_work_disable_auto_calibration);
	cancel_delayed_work_sync(&mxt->d_work_check_lock_point);
	flush_workqueue(g_atmel_wq_dis_auto_cal);

	g_bIsCmdCalibrationOk = false;
	g_bIsCmdCalibration = false;
	g_bIsCheckLockPoint = false;
	g_bIsCheckLockPointAlready = false;

	disable_irq(mxt->irq);

	/* configure deep sleep mode */
	error = mxt_write_block(mxt->client, mxt->t7_addr, ARRAY_SIZE(t7_deepsl_data), t7_deepsl_data);
	if (error < 0)
		goto err_enable_irq;

	/* power off the device */
	if (mxt->power_on)
	{
		error = mxt->power_on(false);
		if (error)
		{
			dev_err(dev, "[touch_pad] power off failed");
			goto err_write_block;
		}
	}

	force_release_fingers();

	mxt->is_suspended = true;

	mxt_debug(DEBUG_POWER, "[touch_pad] mxt_suspend()--\n");

	return 0;

err_write_block:
	mxt_write_block(mxt->client, mxt->t7_addr, ARRAY_SIZE(mxt->t7_data), mxt->t7_data);
err_enable_irq:
	enable_irq(mxt->irq);
	return error;
}

static int mxt_resume(struct device *dev)
{
	struct mxt_data *mxt = dev_get_drvdata(dev);
	int error;

	mxt_debug(DEBUG_POWER, "[touch_pad] mxt_resume()++\n");

	if (!mxt->is_suspended)
		return 0;

	/* power on the device */
	if (mxt->power_on)
	{
		error = mxt->power_on(true);
		if (error)
		{
			dev_err(dev, "[touch_pad] power on failed");
			return error;
		}
	}

	force_release_fingers();

	/* restore the old power state values */
	error = mxt_write_block(mxt->client, mxt->t7_addr, ARRAY_SIZE(mxt->t7_data), mxt->t7_data);
	if (error < 0)
		goto err_write_block;

	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt), T9_CTRL_VAL);

//	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt) + MXT_ADR_T6_CALIBRATE, 1);

	enable_irq(mxt->irq);

	queue_delayed_work(g_atmel_wq_dis_auto_cal, &mxt->d_work_do_calibration, msecs_to_jiffies(DO_CALIBRATION_DELAY));
//	queue_delayed_work(g_atmel_wq_dis_auto_cal, &mxt->d_work_disable_auto_calibration, msecs_to_jiffies(DISABLE_AUTO_CALIBRATION_TIME));

	mxt->is_suspended = false;

	mxt_debug(DEBUG_POWER, "[touch_pad] mxt_resume()--\n");

	return 0;

err_write_block:
	if (mxt->power_on)
		mxt->power_on(false);

	return error;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void mxt_early_suspend(struct early_suspend *h)
{
	struct mxt_data *mxt = container_of(h, struct mxt_data, early_suspend);

	mxt_debug(DEBUG_POWER, "[touch_pad] mxt_early_suspend()++\n");

	if ( g_bIsPadAttach )
	{
		mxt_suspend(&mxt->client->dev);
	}

	mxt_debug(DEBUG_POWER, "[touch_pad] mxt_early_suspend()--\n");
}

static void mxt_late_resume(struct early_suspend *h)
{
	struct mxt_data *mxt = container_of(h, struct mxt_data, early_suspend);

	mxt_debug(DEBUG_POWER, "[touch_pad] mxt_late_resume()++\n");

	if ( g_bIsPadAttach )
	{
		mxt_resume(&mxt->client->dev);
	}

	mxt_debug(DEBUG_POWER, "[touch_pad] mxt_late_resume()--\n");
}
#endif
//ASUS_BSP joe1_--

static const struct dev_pm_ops mxt_pm_ops = {
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= mxt_suspend,
	.resume		= mxt_resume,
#endif
};
#endif


#ifdef CONFIG_I2C_STRESS_TEST


#include <linux/i2c_testcase.h>

#define I2C_TEST_TOUCH_FAIL (-1)
static int TestP01Touch(struct i2c_client *apClient)
{

	int lnResult = I2C_TEST_PASS;
	u8 id_data[7];
    
	i2c_log_in_test_case("TestP01Touch++\n");
	if (mxt_identify(apClient, g_mxt, id_data) < 0) {
        	i2c_log_in_test_case("TestP01Touch failed\n");        
		lnResult = I2C_TEST_TOUCH_FAIL;
	}
    
	i2c_log_in_test_case("TestP01Touch--\n");
	return lnResult;
};

static struct i2c_test_case_info gP01TouchTestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(TestP01Touch),
};


#endif


static int __devinit mxt_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct mxt_data          *mxt;
	struct maxtouch_platform_data *pdata;
	struct input_dev         *input;
	u8 *id_data;
//	u8 *t38_data;
//	u16 t38_addr;
	int error;

	
	mxt_debug(DEBUG_INFO, "[touch_pad] mxt_probe\n");

	mxt_debug(DEBUG_INFO, "[touch_pad] maXTouch driver v. %s\n", DRIVER_VERSION);
	mxt_debug(DEBUG_INFO, "[touch_pad] \t \"%s\"\n", client->name);
	mxt_debug(DEBUG_INFO, "[touch_pad] \taddr:\t0x%04x\n", client->addr);
	mxt_debug(DEBUG_INFO, "[touch_pad] \tirq:\t%d\n", client->irq);
	mxt_debug(DEBUG_INFO, "[touch_pad] \tflags:\t0x%04x\n", client->flags);
	mxt_debug(DEBUG_INFO, "[touch_pad] \tadapter:\"%s\"\n", client->adapter->name);
	mxt_debug(DEBUG_INFO, "[touch_pad] \tdevice:\t\"%s\"\n", client->dev.init_name);

	/* Allocate structure - we need it to identify device */
	mxt = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (mxt == NULL) {
		dev_err(&client->dev, "[touch_pad] insufficient memory\n");
		error = -ENOMEM;
		goto err_mxt_alloc;
	}

	g_mxt = mxt; //ASUS_BSP joe1_++

	id_data = kmalloc(MXT_ID_BLOCK_SIZE, GFP_KERNEL);
	if (id_data == NULL) {
		dev_err(&client->dev, "[touch_pad] insufficient memory\n");
		error = -ENOMEM;
		goto err_id_alloc;
	}

	input = input_allocate_device();
	if (!input) {
		dev_err(&client->dev, "[touch_pad] error allocating input device\n");
		error = -ENOMEM;
		goto err_input_dev_alloc;
	}

	/* Initialize Platform data */

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "[touch_pad] platform data is required!\n");
		error = -EINVAL;
		goto err_pdata;
	}

	mxt_debug(DEBUG_TRACE, "[touch_pad] Platform OK: pdata = 0x%08x\n",  (unsigned int) pdata);

	mxt->is_suspended = false;
	mxt->read_fail_counter = 0;

	if (pdata->min_x)
		mxt->min_x_val = pdata->min_x;
	else
		mxt->min_x_val = 0;

	if (pdata->min_y)
		mxt->min_y_val = pdata->min_y;
	else
		mxt->min_y_val = 0;

	mxt->max_x_val         = pdata->max_x;
	mxt->max_y_val         = pdata->max_y;

	/* Get data that is defined in board specific code. */
	mxt->init_hw = pdata->init_platform_hw;
	mxt->exit_hw = pdata->exit_platform_hw;
	mxt->power_on = pdata->power_on;
	mxt->read_chg = pdata->read_chg;

	if (pdata->valid_interrupt != NULL)
		mxt->valid_interrupt = pdata->valid_interrupt;
	else
		mxt->valid_interrupt = mxt_valid_interrupt_dummy;

	if (mxt->init_hw) {
		error = mxt->init_hw(client);
		if (error) {
			dev_err(&client->dev, "[touch_pad] hw init failed");
			goto err_init_hw;
		}
	}

	/* power on the device */
	if (mxt->power_on) {
		error = mxt->power_on(true);
		if (error) {
			dev_err(&client->dev, "[touch_pad] power on failed");
			goto err_pwr_on;
		}
	}

	mxt_debug(DEBUG_TRACE, "[touch_pad] maXTouch driver identifying chip\n");

	if (mxt_identify(client, mxt, id_data) < 0) {
		dev_err(&client->dev, "[touch_pad] Chip could not be identified\n");
		error = -ENODEV;
		goto err_identify;
	}

	/* Chip is valid and active. */
	mxt_debug(DEBUG_TRACE, "[touch_pad] maXTouch driver allocating input device\n");

	mxt->client = client;
	mxt->input  = input;
	
	mutex_init(&mxt->debug_mutex);

	mxt_debug(DEBUG_TRACE, "[touch_pad] maXTouch driver creating device name\n");

	input->name = "atmel-maxtouch_pad"; //ASUS_BSP joe1_++
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	mxt_debug(DEBUG_INFO, "[touch_pad] maXTouch name: \"%s\"\n", input->name);

	mxt_debug(DEBUG_INFO, "[touch_pad] maXTouch driver setting abs parameters\n");

	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
//	__set_bit(EV_MSC, input->evbit); //ASUS_BSP joe1_++
//	input->mscbit[0] = BIT_MASK(MSC_GESTURE); //ASUS_BSP joe1_++

	set_bit(BTN_TOUCH, input->keybit);
	set_bit(INPUT_PROP_DIRECT, input->propbit);

	/* Single touch */
	input_set_abs_params(input, ABS_X, mxt->min_x_val,
				mxt->max_x_val, 0, 0);
	input_set_abs_params(input, ABS_Y, mxt->min_y_val,
				mxt->max_y_val, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, MXT_MAX_REPORTED_PRESSURE,
			     0, 0);
	input_set_abs_params(input, ABS_TOOL_WIDTH, 0, MXT_MAX_REPORTED_WIDTH,
			     0, 0);

	/* Multitouch */
	input_set_abs_params(input, ABS_MT_POSITION_X, mxt->min_x_val,
				mxt->max_x_val, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, mxt->min_y_val,
				mxt->max_y_val, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, MXT_MAX_TOUCH_SIZE,
			     0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, MXT_MAX_NUM_TOUCHES,
			     0, 0);
//ASUS_BSP joe1_++
#ifdef SUPPORT_PRESSURE
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, MXT_MAX_REPORTED_PRESSURE,
			     0, 0);
	input_set_abs_params(input, ABS_MT_ORIENTATION, 0, 255,
			     0, 0);
#endif
//ASUS_BSP joe1_--
	
	mxt_debug(DEBUG_TRACE, "[touch_pad] maXTouch driver setting client data\n");
	i2c_set_clientdata(client, mxt);

	mxt_debug(DEBUG_TRACE, "[touch_pad] maXTouch driver setting drv data\n");
	input_set_drvdata(input, mxt);

	mxt_debug(DEBUG_TRACE, "[touch_pad] maXTouch driver input register device\n");
	error = input_register_device(mxt->input);
	if (error < 0) {
		dev_err(&client->dev,
			"[touch_pad] Failed to register input device\n");
		goto err_register_device;
	}

	error = mxt_read_object_table(client, mxt, id_data);
	if (error < 0)
		goto err_read_ot;

//ASUS_BSP joe1_++
	//sw reset
	/*
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt) + MXT_ADR_T6_RESET, 0x01);

	if ( mxt->device_info.family_id == MXT1386_FAMILYID ) //mXT1386
	{
		msleep(MXT1386_RESET_TIME);
	}
	else
	{
		msleep(MXT768_RESET_TIME);
	}
	*/
	mxt->t7_addr = MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt);

	// save power state values for suspend/resume
	mxt_read_block(mxt->client, mxt->t7_addr, ARRAY_SIZE(mxt->t7_data), mxt->t7_data);

	g_lastFirmwareMajor = mxt->device_info.major;
	g_lastFirmwareMinor = mxt->device_info.minor;
//ASUS_BSP joe1_--

	/* Create debugfs entries. */
	mxt->debug_dir = debugfs_create_dir("maXTouch_pad", NULL);
	if (mxt->debug_dir == ERR_PTR(-ENODEV)) {
		/* debugfs is not enabled. */
		printk(KERN_WARNING "[touch_pad] debugfs not enabled in kernel\n");
	} else if (mxt->debug_dir == NULL) {
		printk(KERN_WARNING "[touch_pad] error creating debugfs dir\n");
	} else {
		mxt_debug(DEBUG_TRACE, "[touch_pad] created \"maXTouch_pad\" debugfs dir\n");
		
		debugfs_create_file("deltas", S_IRUSR, mxt->debug_dir, mxt, 
				    &delta_fops);
		debugfs_create_file("refs", S_IRUSR, mxt->debug_dir, mxt,
				    &refs_fops);
//ASUS_BSP simpson: add for ATD reading raw data +++
		debugfs_create_file("raw_data", S_IRUGO, mxt->debug_dir, mxt,
				    &raw_data_fops);
//ASUS_BSP simpson: add for ATD reading raw data ---
	}

	/* Allocate the interrupt */
	mxt_debug(DEBUG_TRACE, "[touch_pad] maXTouch driver allocating interrupt...\n");
	
	INIT_DELAYED_WORK(&mxt->d_work_disable_auto_calibration, disable_auto_cal_delayed_work);
	INIT_DELAYED_WORK(&mxt->d_work_disable_irq, disable_irq_delayed_work);
	INIT_DELAYED_WORK(&mxt->d_work_do_calibration, do_cal_work);
	INIT_DELAYED_WORK(&mxt->d_work_check_lock_point, check_lock_point_work);

	mxt->irq = client->irq;

	if (mxt->irq) {
		/* Try to request IRQ with falling edge first. This is
		 * not always supported. If it fails, try with any edge. */
		error = request_threaded_irq(mxt->irq,
									NULL,
									mxt_irq_handler,
									IRQF_TRIGGER_FALLING,
									client->dev.driver->name,
									mxt);
		if (error < 0) {
			dev_err(&client->dev,
				"[touch_pad] failed to allocate irq %d\n", mxt->irq);
			goto err_irq;
		}

		//make_chgline_high(mxt); //ASUS_BSP joe1_++
	}

	mxt_debug(DEBUG_INFO, "[touch_pad] touchscreen, irq %d\n", mxt->irq);

//ASUS_BSP joe1_++
	g_bIsMxtIrqDisable = false;
//ASUS_BSP joe1_--
		
#if 0
	t38_data = kmalloc(t38_size*sizeof(u8), GFP_KERNEL);

	if (t38_data == NULL) {
		dev_err(&client->dev, "[touch_pad] insufficient memory\n");
		error = -ENOMEM;
		goto err_t38;
	}

	t38_addr = MXT_BASE_ADDR(MXT_USER_INFO_T38, mxt);
	mxt_read_block(client, t38_addr, t38_size, t38_data);
	dev_info(&client->dev, "[touch_pad] VERSION:%02x.%02x.%02x, DATE: %d/%d/%d\n",
		t38_data[0], t38_data[1], t38_data[2],
		t38_data[3], t38_data[4], t38_data[5]);

	kfree(t38_data);
#endif

	kfree(id_data);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	mxt->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						MXT_SUSPEND_LEVEL;
	mxt->early_suspend.suspend = mxt_early_suspend;
	mxt->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&mxt->early_suspend);
#endif

//ASUS_BSP joe1_++
	mxt->attrs.attrs = mxt_attr;
	error = sysfs_create_group(&client->dev.kobj, &mxt->attrs);
	if (error) {
		dev_err(&client->dev, "[touch_pad] Not able to create the sysfs\n");
		goto exit_remove;
	}

	queue_delayed_work(g_atmel_wq_dis_auto_cal, &mxt->d_work_disable_auto_calibration, msecs_to_jiffies(DISABLE_AUTO_CALIBRATION_TIME));
//	queue_delayed_work(g_atmel_wq_dis_auto_cal, &mxt->d_work_do_calibration, msecs_to_jiffies(DO_CALIBRATION_DELAY));

	g_iMxtProbeError = 0;
//ASUS_BSP joe1_--


#ifdef CONFIG_I2C_STRESS_TEST
       i2c_add_test_case(client, "P01_Touch",ARRAY_AND_SIZE(gP01TouchTestCaseInfo));
#endif

	//mxt_write_byte(mxt->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6,mxt) + MXT_ADR_T6_RESET, 0x01);
	
	return 0;

//ASUS_BSP joe1_++
exit_remove:
//	sysfs_remove_group(&client->dev.kobj, &mxt->attrs);

//err_t38:
//ASUS_BSP joe1_--
	free_irq(mxt->irq, mxt);
err_irq:
	if (mxt->debug_dir)
		debugfs_remove(mxt->debug_dir);
	kfree(mxt->rid_map);
	kfree(mxt->object_table);
err_read_ot:
	input_unregister_device(mxt->input);
	mxt->input = NULL;
err_register_device:
	mutex_destroy(&mxt->debug_mutex);
err_identify:
	if (mxt->power_on)
		mxt->power_on(false);
err_pwr_on:
	if (mxt->exit_hw != NULL)
		mxt->exit_hw(client);
err_init_hw:
err_pdata:
	input_free_device(input);
err_input_dev_alloc:
	kfree(id_data);
err_id_alloc:
	kfree(mxt);
	mxt = NULL;
	g_mxt = NULL; //ASUS_BSP joe1_++
err_mxt_alloc:
	g_iMxtProbeError = error;

	printk("[touch_pad] probe: g_iMxtProbeError= %d\n", g_iMxtProbeError);
	
	return error;
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);

	/* Remove debug dir entries */
	debugfs_remove_recursive(mxt->debug_dir);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&mxt->early_suspend);
#endif

	if (mxt != NULL) {
		if (mxt->power_on)
			mxt->power_on(false);

		if (mxt->exit_hw != NULL)
			mxt->exit_hw(client);

		if (mxt->irq) {
			free_irq(mxt->irq, mxt);
		}

		input_unregister_device(mxt->input);
		debugfs_remove(mxt->debug_dir);

		kfree(mxt->rid_map);
		kfree(mxt->object_table);

		sysfs_remove_group(&client->dev.kobj, &mxt->attrs); //ASUS_BSP joe1_++
	}
	kfree(mxt);
	g_mxt = NULL; //ASUS_BSP joe1_++

	i2c_set_clientdata(client, NULL);
	if (debug >= DEBUG_TRACE)
		dev_info(&client->dev, "[touch_pad] Touchscreen unregistered\n");

	return 0;
}

static const struct i2c_device_id mxt_idtable[] = {
	{"maXTouchPad", 0,},
	{ }
};

MODULE_DEVICE_TABLE(i2c, mxt_idtable);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "maXTouchPad",
		.owner  = THIS_MODULE,
#if defined(CONFIG_PM)
		.pm = &mxt_pm_ops,
#endif
	},

	.id_table	= mxt_idtable,
	.probe		= mxt_probe,
	.remove		= __devexit_p(mxt_remove),
};

//ASUS_BSP joe1_++
#ifdef CONFIG_EEPROM_NUVOTON
static int touch_mp_event(struct notifier_block *this, unsigned long event, void *ptr);

static struct notifier_block touch_mp_notifier = {
        .notifier_call = touch_mp_event,
        .priority = TOUCH_MP_NOTIFY,
};
#endif //CONFIG_EEPROM_NUVOTON
//ASUS_BSP joe1_--

static int __init mxt_init(void)
{
//ASUS_BSP joe1_++
	int err = 0;

	mxt_debug(DEBUG_INFO, "[touch_pad] mxt_init()++\n");

	g_atmel_wq_attach_detach = create_singlethread_workqueue("g_atmel_wq_attach_detach_pad");
	if (!g_atmel_wq_attach_detach) {
		printk("[touch_pad] %s: create workqueue failed: g_atmel_wq_attach_detach\n", __func__);
	}

	g_atmel_wq_dis_auto_cal = create_singlethread_workqueue("g_atmel_wq_dis_auto_cal_pad");
	if (!g_atmel_wq_dis_auto_cal) {
		printk("[touch_pad] %s: create workqueue failed: g_atmel_wq_dis_auto_cal\n", __func__);
	}

	if ( g_bIsPadAttach )
	{
		err = i2c_add_driver(&mxt_driver);
		if (err) {
			printk(KERN_WARNING "[touch_pad] Adding maXTouch driver failed "
			       "(errno = %d)\n", err);
		} else {
			mxt_debug(DEBUG_TRACE, "[touch_pad] Successfully added driver %s\n",
				  mxt_driver.driver.name);
		}
	}

	g_bIsMxtInit = true;

#ifdef CONFIG_EEPROM_NUVOTON
	INIT_WORK(&g_mp_attach_work, attach_padstation_work);
	INIT_WORK(&g_mp_detach_work, detach_padstation_work);

	register_microp_notifier(&touch_mp_notifier);
#endif //CONFIG_EEPROM_NUVOTON

	mxt_debug(DEBUG_INFO, "[touch_pad] mxt_init()--\n");
//ASUS_BSP joe1_--
	return err;
}

static void __exit mxt_cleanup(void)
{
	i2c_del_driver(&mxt_driver);

//ASUS_BSP joe1_++
	destroy_workqueue(g_atmel_wq_attach_detach);
	destroy_workqueue(g_atmel_wq_dis_auto_cal);

#ifdef CONFIG_EEPROM_NUVOTON
	unregister_microp_notifier(&touch_mp_notifier);
#endif //CONFIG_EEPROM_NUVOTON
//ASUS_BSP joe1_--
}


module_init(mxt_init);
module_exit(mxt_cleanup);

//ASUS_BSP joe1_++
#ifdef CONFIG_EEPROM_NUVOTON
#define P02_TOUCH_FW_VERSION_FILE        "/data/asusdata/p02_touch_FW.nv"

static bool write_touch_fw_version(void)
{
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char writestr[25];
	mm_segment_t old_fs;
	bool bRet = true;

	mxt_debug(DEBUG_INFO, "[touch_pad] write_touch_fw_version()++\n");

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(P02_TOUCH_FW_VERSION_FILE, O_RDWR|O_CREAT|O_TRUNC, 0644);
	if(IS_ERR_OR_NULL(fp))
	{
		printk("[touch_pad] write_touch_fw_version open (%s) failed\n", P02_TOUCH_FW_VERSION_FILE);
		return false;
	}

	sprintf(writestr, "v[%d.%d], Build:%d\n", g_mxt->device_info.major, g_mxt->device_info.minor, g_mxt->device_info.build);

	printk("[touch_pad] %s; length=%d\n", writestr, strlen(writestr));

	if (fp->f_op != NULL && fp->f_op->write != NULL)
	{
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
	{
		printk("[touch_pad] write_touch_fw_version failed\n");

		bRet = false;
	}

	filp_close(fp, NULL);

	set_fs(old_fs);

	mxt_debug(DEBUG_INFO, "[touch_pad] write_touch_fw_version()--\n");

	return bRet;
}

static int touch_controller_hw_init(void)
{
	mxt_debug(DEBUG_INFO, "[touch_pad] touch_controller_hw_init()++\n");

	//sw reset
	/*
	mxt_write_byte(g_mxt->client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, g_mxt) + MXT_ADR_T6_RESET, 0x01);

	if ( g_mxt->device_info.family_id == MXT1386_FAMILYID ) //mXT1386
	{
		msleep(MXT1386_RESET_TIME);
	}
	else
	{
		msleep(MXT768_RESET_TIME);
	}
	*/
	
	// save power state values for suspend/resume
	mxt_read_block(g_mxt->client, g_mxt->t7_addr, ARRAY_SIZE(g_mxt->t7_data), g_mxt->t7_data);

//	mxt_write_byte(g_mxt->client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, g_mxt) + MXT_ADR_T6_CALIBRATE, 1);

//	disable_auto_calibration(g_mxt);
	queue_delayed_work(g_atmel_wq_dis_auto_cal, &g_mxt->d_work_disable_auto_calibration, msecs_to_jiffies(DISABLE_AUTO_CALIBRATION_TIME));

	mxt_debug(DEBUG_INFO, "[touch_pad] touch_controller_hw_init()--\n");

	return 0;
}

static void __devinit attach_padstation_work(struct work_struct *work)
{
	u8 *id_data;
	int error;
	static int attach_counter=0;//ASUS_BSP simpson: add for P02 insert/remove stress test++

	printk("[touch_pad] attach_padstation_work()++\n");

	if ( (g_bIsMxtInit == true) && (g_iMxtProbeError != 0) )
	{
		i2c_add_driver(&mxt_driver);
		mxt_debug(DEBUG_INFO, "[touch_pad] Successfully added driver %s\n", mxt_driver.driver.name);

		if ( g_iMxtProbeError )
		{
			printk("[touch_pad] i2c_del_driver++g_iMxtProbeError=%d\n", g_iMxtProbeError);

			i2c_del_driver(&mxt_driver);

			printk("[touch_pad] i2c_del_driver--\n");
		}
	}

	if ( g_mxt )
	{
		force_release_fingers();

		if ( g_bIsMxtIrqDisable )
		{
			id_data = kmalloc(MXT_ID_BLOCK_SIZE, GFP_KERNEL);
			if (id_data == NULL) {
				dev_err(&g_mxt->client->dev, "[touch_pad] insufficient memory\n");
			}

			if (mxt_identify(g_mxt->client, g_mxt, id_data) < 0) {
				dev_err(&g_mxt->client->dev, "[touch_pad] Chip could not be identified\n");
			}

			if ( (g_mxt->device_info.major != g_lastFirmwareMajor) || (g_mxt->device_info.minor != g_lastFirmwareMinor) )
			{
				printk("[touch_pad] firmware version: current=[%d.%d]; last=[%d.%d]\n", g_mxt->device_info.major, g_mxt->device_info.minor, g_lastFirmwareMajor, g_lastFirmwareMinor);

				kfree(g_mxt->object_table);

				kfree(g_mxt->rid_map);

				error = mxt_read_object_table(g_mxt->client, g_mxt, id_data);
				if (error < 0)
				{
					dev_err(&g_mxt->client->dev, "[touch_pad] failed to create object table!\n");

					kfree(id_data);
					
					goto end;				
				}
			}

			kfree(id_data);

			g_lastFirmwareMajor = g_mxt->device_info.major;
			g_lastFirmwareMinor = g_mxt->device_info.minor;
		
			touch_controller_hw_init();

			error = request_threaded_irq(g_mxt->irq,
									NULL,
									mxt_irq_handler,
									IRQF_TRIGGER_FALLING,
									g_mxt->client->dev.driver->name,
									g_mxt);
			if (error < 0)
			{
				dev_err(&g_mxt->client->dev, "[touch_pad] failed to allocate irq %d\n", g_mxt->irq);
			}

			//make_chgline_high( g_mxt ); //ASUS_BSP joe1_++

			g_mxt->is_suspended = false;

//			enable_irq(g_mxt->irq);

			g_bIsMxtIrqDisable = false;

			mxt_debug(DEBUG_INFO, "[touch_pad] attach_padstation_work: enable_irq\n");
		}

		g_mxt->read_fail_counter = 0;

		write_touch_fw_version();
	}

	g_bIsPadAttach = true;

end:
//ASUS_BSP simpson: add for P02 insert/remove stress test+++
	if(g_mxt)
		mxt_write_byte(g_mxt->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6,g_mxt) + MXT_ADR_T6_RESET, 0x01);
	attach_counter ++;
	printk("[touch_pad] attach_counter= %d\n" , attach_counter );
//ASUS_BSP simpson: add for P02 insert/remove stress test---

	printk("[touch_pad] attach_padstation_work()--\n");
}

static void __devinit detach_padstation_work(struct work_struct *work)
{
	static int detach_counter=0;//ASUS_BSP simpson: add for P02 insert/remove stress test++

	printk("[touch_pad] detach_padstation_work()++\n");

	if ( g_mxt )
	{
		cancel_delayed_work_sync(&g_mxt->d_work_do_calibration);
		cancel_delayed_work_sync(&g_mxt->d_work_disable_auto_calibration);
		cancel_delayed_work_sync(&g_mxt->d_work_check_lock_point);
		flush_workqueue(g_atmel_wq_dis_auto_cal);

		if ( !g_bIsMxtIrqDisable )
		{
			free_irq(g_mxt->irq, g_mxt);

//			disable_irq(g_mxt->irq);

			g_bIsMxtIrqDisable = true;

			mxt_debug(DEBUG_INFO, "[touch_pad] detach_padstation_work: disable_irq\n");
		}

		force_release_fingers();
	}

	cfg_flag = true;

	g_bIsPadAttach = false;

//ASUS_BSP simpson: add for P02 insert/remove stress test+++
	detach_counter ++;
	printk("[touch_pad] detach_counter= %d\n" , detach_counter );
//ASUS_BSP simpson: add for P02 insert/remove stress test---

	printk("[touch_pad] detach_padstation_work()--\n");
}

int touch_attach_padstation_pad(void)
{
//	mxt_debug(DEBUG_INFO, "[touch_pad] touch_attach_padstation_pad()++\n");
	printk("[touch_pad] touch_attach_padstation_pad()++\n");

	queue_work(g_atmel_wq_attach_detach, &g_mp_attach_work);

//	mxt_debug(DEBUG_INFO, "[touch_pad] touch_attach_padstation_pad()--\n");
	printk("[touch_pad] touch_attach_padstation_pad()--\n");

	return 0;
}
EXPORT_SYMBOL(touch_attach_padstation_pad);

int touch_detach_padstation_pad(void)
{
//	mxt_debug(DEBUG_INFO, "[touch_pad] touch_detach_padstation_pad()++\n");
	printk("[touch_pad] touch_detach_padstation_pad()++\n");

	queue_work(g_atmel_wq_attach_detach, &g_mp_detach_work);

//	mxt_debug(DEBUG_INFO, "[touch_pad] touch_detach_padstation_pad()--\n");
	printk("[touch_pad] touch_detach_padstation_pad()--\n");

	return 0;
}
EXPORT_SYMBOL(touch_detach_padstation_pad);

static int touch_mp_event(struct notifier_block *this, unsigned long event, void *ptr)
{

        switch (event) {

        case P01_ADD:
                printk("[touch_pad][MicroP] P01_ADD++\r\n");

                touch_attach_padstation_pad();

//                mxt_debug(DEBUG_INFO, "[touch_pad][MicroP] P01_ADD \r\n");
                printk("[touch_pad][MicroP] P01_ADD--\r\n");

                return NOTIFY_DONE;

        case P01_REMOVE:
                printk("[touch_pad][MicroP] P01_REMOVE++\r\n");

                touch_detach_padstation_pad();
					
//                mxt_debug(DEBUG_INFO, "[touch_pad][MicroP] P01_REMOVE \r\n");
                printk("[touch_pad][MicroP] P01_REMOVE--\r\n");

                return NOTIFY_DONE;

        default:

                return NOTIFY_DONE;

        }

}
#endif //CONFIG_EEPROM_NUVOTON
//ASUS_BSP joe1_--

MODULE_AUTHOR("Iiro Valkonen");
MODULE_DESCRIPTION("Driver for Atmel maXTouch Touchscreen Controller");
MODULE_LICENSE("GPL");
