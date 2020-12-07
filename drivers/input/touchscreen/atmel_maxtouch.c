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
#include <linux/module.h>

#include <asm/uaccess.h>
#include <linux/atmel_maxtouch.h>
//ASUS_BSP joe1_++
#include <linux/delay.h>
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp_notify.h>
#endif //CONFIG_EEPROM_NUVOTON
//ASUS_BSP joe1_--

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>

/* Early-suspend level */
#define MXT_SUSPEND_LEVEL 1
#endif

#define DRIVER_VERSION "0.94"

//ASUS_BSP joe1_++
//#define CAL_REPORT_RATE

#define MAX_FINGER_NUM	5
#define MXT_BACKUP_TIME		25	//ms
#define MXT224_RESET_TIME		70	//~64ms
#define MXT224E_RESET_TIME		25 	//~22ms

#define DISABLE_AUTO_CALIBRATION_TIME_FIRST	30000 //ms
#define DISABLE_AUTO_CALIBRATION_TIME		15000 //ms
#define DISABLE_AUTO_CALIBRATION_DELAY		2000 //ms
#define DO_CALIBRATION_DELAY					6000 //ms

#define MOVE_DISTANCE	170

#define T9_CTRL_VAL		139
static struct mxt_data *g_mxt;

static bool g_bIsPadAttach = false;
static bool g_bIsSysSuspended = false;
static bool g_bIsCmdCalibration = false;
static bool g_bIsCmdCalibrationOk = false;

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
	struct delayed_work d_work_do_calibration;
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
enum virtual_buttons{
	VBTN_MENU=0,
	VBTN_HOME,
	VBTN_BACK,
	VBTN_SEARCH,
	NUM_OF_VBTN
};

static u8 pre_button[NUM_OF_VBTN];

static bool cfg_flag = true;

#ifdef CONFIG_EEPROM_NUVOTON
static void __devinit attach_padstation_work(struct work_struct *work);
static void __devinit detach_padstation_work(struct work_struct *work);
#endif //CONFIG_EEPROM_NUVOTON

static void force_release_fingers(void)
{
	struct mxt_data *mxt = g_mxt;
	struct finger_info *finger = g_mxt->finger;
	int i;

	mxt_debug(DEBUG_INFO, "[touch] force_release_fingers()++\n");

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
		input_report_abs(mxt->input, ABS_MT_PRESSURE, finger[i].amplitude);
//		input_report_abs(mxt->input, ABS_MT_ORIENTATION, finger[i].vector);

		input_mt_sync(mxt->input);

		mxt_debug(DEBUG_INFO, "[touch] force_release_fingers: finger[%d].x=%d\n", i, finger[i].x);
		mxt_debug(DEBUG_INFO, "[touch] force_release_fingers: finger[%d].y=%d\n", i, finger[i].y);
	}

	input_report_key(mxt->input, BTN_TOUCH, 0);

	input_mt_sync(mxt->input);

	input_sync(mxt->input);

	mxt_debug(DEBUG_INFO, "[touch] force_release_fingers()--\n");
}

#define RETRY_COUNT 500
#define DELTA_VALUE 40
int check_T37_delta(void)
{
	int i;
	u16 *data;
	u16 diagnostics_reg;
	int offset = 0;
	int offset_ori;
	int size;
	int read_size;
	int error;
	u16 debug_data_addr;
	u16 page_address;
	u8 page;
	u8 debug_command_reg;
//	char temp[24];
	u8 debug_command = MXT_CMD_T6_DELTAS_MODE;
	s16 delta_value_raw;
	s16 delta_value;
	int iCount_1 = 0;
	int iCount_2 = 0;
	int iRet = -EIO;
	int iTouchCount = 0;
	int iAntiTouchCount = 0;

	printk("[touch] check_T37_delta()++\n");

	disable_irq(g_mxt->irq);

	if ( !g_mxt )
		return -ENXIO;

	data = kzalloc(g_mxt->device_info.num_nodes * sizeof(u16), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;
	
//	for (i = 0; i < g_mxt->device_info.num_nodes; i++)
//		data[i] = 7777;

	diagnostics_reg = MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, g_mxt) + MXT_ADR_T6_DIAGNOSTIC;
	debug_data_addr = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, g_mxt)+ MXT_ADR_T37_DATA;
	page_address = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, g_mxt) + MXT_ADR_T37_PAGE;

	error = mxt_read_block(g_mxt->client, page_address, 1, &page);
	if (error < 0)
	{
		goto free_data;
	}

	mxt_debug(DEBUG_TRACE, "[touch] debug data page = %d\n", page);

	while (page != 0 && iCount_1 < RETRY_COUNT)
	{
		error = mxt_write_byte(g_mxt->client, diagnostics_reg, MXT_CMD_T6_PAGE_DOWN);
		if (error < 0)
		{
			goto free_data;
		}

		/* Wait for command to be handled; when it has, the register will be cleared. */
		debug_command_reg = 1;
		while (debug_command_reg != 0 && iCount_2 < RETRY_COUNT)
		{
			error = mxt_read_block(g_mxt->client, diagnostics_reg, 1, &debug_command_reg);
			if (error < 0)
			{
				goto free_data;
			}

			mxt_debug(DEBUG_TRACE, "[touch] Waiting for debug diag command to propagate...; iCount_2=%d\n", iCount_2);

			iCount_2++;
		}

		if ( iCount_2 >= RETRY_COUNT )
		{
			printk("[touch] Waiting command failed!iCount_2=%d\n", iCount_2);

			goto free_data;
		}

		error = mxt_read_block(g_mxt->client, page_address, 1, &page);
		if (error < 0)
		{
			goto free_data;
		}

		mxt_debug(DEBUG_TRACE, "[touch] debug data page = %d; iCount_1=%d\n", page, iCount_1);

		iCount_1++;
	}

	if ( iCount_1 >= RETRY_COUNT )
	{
		printk("[touch] reset page failed!iCount_1=%d\n", iCount_1);

		goto free_data;
	}

#if 0
	/*
	 * Lock mutex to prevent writing some unwanted data to debug
	 * command register. User can still write through the char 
	 * device interface though. TODO: fix?
	 */
	mutex_lock(&g_mxt->debug_mutex);
#endif

	/* Configure Debug Diagnostics object to show deltas/refs */
	error = mxt_write_byte(g_mxt->client, diagnostics_reg, debug_command);
	if (error < 0)
	{
		goto free_mutex;
	}

	/* Wait for command to be handled; when it has, the register will be cleared. */
	iCount_1 = 0;
	debug_command_reg = 1;
	while (debug_command_reg != 0 && iCount_1 < RETRY_COUNT)
	{
		error = mxt_read_block(g_mxt->client, diagnostics_reg, 1, &debug_command_reg);
		if (error < 0)
		{
			goto free_mutex;
		}

		mxt_debug(DEBUG_TRACE, "[touch] Waiting for debug diag command to propagate...; iCount_1=%d\n", iCount_1);

		iCount_1++;
	}

	if ( iCount_1 >= RETRY_COUNT )
	{
		printk (KERN_WARNING "[touch] Error writing to maXTouch device! iCount_1=%d\n", iCount_1);

		goto free_mutex;
	}

	size = g_mxt->device_info.num_nodes * sizeof(u16);

	while (size > 0)
	{
		read_size = size > 128 ? 128 : size;

		mxt_debug(DEBUG_TRACE, "[touch] Debug data read loop, reading %d bytes...\n", read_size);

		error = mxt_read_block(g_mxt->client, debug_data_addr, read_size, (u8 *) &data[offset]);
		if (error < 0)
		{
			printk(KERN_WARNING "[touch] Error reading debug data\n");

			goto free_mutex;
		}

		offset_ori = offset;

		offset += read_size/2;
		size -= read_size;

#if 0
		for (i = offset_ori; i < offset; i++)
		{
			delta_value_raw = (s16) le16_to_cpu(data[i]);

			delta_value = delta_value_raw/8;

			if ( delta_value < -40 )
			{
				sprintf(temp, "!--%4d: %5d\n", i, delta_value);

				printk("%s\n", temp);

				iRet = 0;

				goto free_mutex;
			}
#if 0
			else if ( delta_value > 60 )
			{
				sprintf(temp, "!++%4d: %5d\n", i, delta_value);

				printk("%s\n", temp);

				iRet = 0;

				goto free_mutex;
			}
#endif
		}
#endif

		/* Select next page */
		error = mxt_write_byte(g_mxt->client, diagnostics_reg, MXT_CMD_T6_PAGE_UP);
		if (error < 0)
		{
			printk(KERN_WARNING "[touch] Error writing to maXTouch device!\n");

			goto free_mutex;
		}

		/* Wait for command to be handled; when it has, the register will be cleared. */
		iCount_2 = 0;
		debug_command_reg = 1;
		while (debug_command_reg != 0 && iCount_2 < RETRY_COUNT)
		{
			error = mxt_read_block(g_mxt->client, diagnostics_reg, 1, &debug_command_reg);
			if (error < 0)
			{
				goto free_mutex;
			}

			mxt_debug(DEBUG_TRACE, "[touch] Waiting for debug diag command to propagate...; iCount_2=%d\n", iCount_2);

			iCount_2++;
		}

		if ( iCount_2 >= RETRY_COUNT )
		{
			printk("[touch] Waiting command failed!iCount_2=%d\n", iCount_2);

			goto free_mutex;
		}

#if 0
		error = mxt_read_block(g_mxt->client, page_address, 1, &page);
		if (error < 0)
		{
			goto free_mutex;
		}

		printk("[touch] debug data page = %d\n", page);
#endif
	}

//	mutex_unlock(&g_mxt->debug_mutex);

//	iRet = 1;

	for (i = 0; i < g_mxt->device_info.num_nodes; i++)
	{
		delta_value_raw = (s16) le16_to_cpu(data[i]);

		delta_value = delta_value_raw/8;

		if ( delta_value > DELTA_VALUE )
		{
			iTouchCount++;

//			sprintf(temp, "++%4d: %5d\n", i, delta_value);

//			printk("%s\n", temp);

//			iRet = 1;

//			goto free_data;
		}
		else if ( delta_value < -DELTA_VALUE )
		{
			iAntiTouchCount++;

//			sprintf(temp, "--%4d: %5d\n", i, delta_value);

//			printk("%s\n", temp);

//			iRet = 0;
		}

#if 0
		if ( delta_value >= 0 )
		{
			sprintf(temp, "++%4d: %5d\n", i, delta_value);
		}
		else
		{
			sprintf(temp, "--%4d: %5d\n", i, delta_value);
		}
#endif
	}

	printk("[touch] iTouchCount=%d\n", iTouchCount);
	printk("[touch] iAntiTouchCount=%d\n", iAntiTouchCount);

	if ( iAntiTouchCount > 0 )
	{
		iRet = 0;

		if ( iTouchCount > 0 )
		{
			iRet = 1;
		}
	}
	else
	{
		iRet = 2;
	}

free_mutex:
//	mutex_unlock(&g_mxt->debug_mutex);
free_data:
	kfree(data);

	enable_irq(g_mxt->irq);

	printk("[touch] check_T37_delta()-- iRet=%d\n", iRet);

	return iRet;
}
EXPORT_SYMBOL(check_T37_delta);

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

	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+6, 1);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+7, 40);

	if ( iATCHFRCCALTHR )
	{
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+8, 50);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+9, 25);
	}
	else
	{
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+8, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+9, 0);
	}
}

static void do_cal_work(struct work_struct *work)
{
	struct mxt_data *mxt = container_of(work, struct mxt_data, d_work_do_calibration.work);
	struct first_finger_info *first_finger = mxt->first_finger;
	int i;

	printk("[touch] do_cal_work()++\n");

//	printk("[touch] g_mxt=0x%x ; mxt=0x%x\n", (unsigned int)g_mxt, (unsigned int)mxt);
//	printk("[touch] max_x_val=%d, max_y_val=%d\n", mxt->max_x_val, mxt->max_y_val);

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

	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt), 0);

	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt) + MXT_ADR_T6_CALIBRATE, 1);

	g_bIsCmdCalibration = true;

	printk("[touch] do_cal_work()--\n");
}

static void disable_auto_cal_delayed_work(struct work_struct *work)
{
	static bool bIsFingersDetect = false;
	struct mxt_data *mxt = container_of(work, struct mxt_data, d_work_disable_auto_calibration.work);
	struct finger_info *finger = mxt->finger;
	int i;

	printk("[touch] disable_auto_cal_delayed_work()++\n");

//	printk("[touch] g_mxt=0x%x ; mxt=0x%x\n", (unsigned int)g_mxt, (unsigned int)mxt);
//	printk("[touch] max_x_val=%d, max_y_val=%d\n", mxt->max_x_val, mxt->max_y_val);

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

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt), 1);

		printk("[touch] disable_auto_cal_delayed_work(): no finger, disable auto calibration success!\n");
	}
	else
	{
		queue_delayed_work(g_atmel_wq_dis_auto_cal, &mxt->d_work_disable_auto_calibration, msecs_to_jiffies(DISABLE_AUTO_CALIBRATION_DELAY));

		printk("[touch] disable_auto_cal_delayed_work(): fingers detected, delay %d ms\n", DISABLE_AUTO_CALIBRATION_DELAY);
	}

	printk("[touch] disable_auto_cal_delayed_work()--bIsFingersDetect=%d\n", bIsFingersDetect);
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

	if ( data->device_info.family_id == MXT224E_FAMILYID ) //mXT224E
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
	else if ( data->device_info.family_id == MXT224_FAMILYID ) //mXT224
	{
		err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, data), 32, (u8 *) tmp);
		if (err < 0){
			sprintf(tmpstr, "read T9 cfg error, ret %d\n",err);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
		else{
			for (i=0; i < 32; i++){
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
	
	mxt_read_block(data->client, MXT_BASE_ADDR(MXT_SPT_GPIOPWM_T19, data), 16, (u8 *) tmp);
	for (i=0; i < 16; i++){
		sprintf(tmpstr,"T19 byte[%d] = %d\n",i,tmp[i]);
		strncat (buf,tmpstr,strlen(tmpstr));
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

	if ( data->device_info.family_id == MXT224E_FAMILYID ) //mXT224E
	{
		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_TOUCH_PROXIMITY_T23, data), 15, (u8 *) tmp);
		for (i=0; i < 15; i++){
			sprintf(tmpstr,"T23 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}

		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, data), 14, (u8 *) tmp);
		for (i=0; i < 14; i++){
			sprintf(tmpstr,"T25 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}
	else if ( data->device_info.family_id == MXT224_FAMILYID ) //mXT224
	{
		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCI_GRIPFACESUPPRESSION_T20, data), 12, (u8 *) tmp);
		for (i=0; i < 12; i++){
			sprintf(tmpstr,"T20 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}

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

		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, data), 6, (u8 *) tmp);
		for (i=0; i < 6; i++){
			sprintf(tmpstr,"T28 byte[%d] = %d\n",i,tmp[i]);
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

	mxt_read_block(data->client, MXT_BASE_ADDR(MXT_USER_INFO_T38, data), 8, (u8 *) tmp);
	for (i=0; i < 8; i++){
		sprintf(tmpstr,"T38 byte[%d] = %d\n",i,tmp[i]);
		strncat (buf,tmpstr,strlen(tmpstr));
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
	if ( data->device_info.family_id == MXT224E_FAMILYID ) //mXT224E
	{
		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_GRIPSUPPRESSION_T40, data), 5, (u8 *) tmp);
		for (i=0; i < 5; i++){
			sprintf(tmpstr,"T40 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}

		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, data), 8, (u8 *) tmp);
		for (i=0; i < 8; i++){
			sprintf(tmpstr,"T42 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
		
		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, data), 9, (u8 *) tmp);
		for (i=0; i < 9; i++){
			sprintf(tmpstr,"T46 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}

		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCI_STYLUS_T47, data), 10, (u8 *) tmp);
		for (i=0; i < 10; i++){
			sprintf(tmpstr,"T47 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}
	else if ( data->device_info.family_id == MXT224_FAMILYID ) //mXT224
	{
		sprintf(buf,"no objects!!!\n");
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

	if ( data->device_info.family_id == MXT224E_FAMILYID ) //mXT224E
	{
		mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, data), 54, (u8 *) tmp);
		for (i=0; i < 54; i++){
			sprintf(tmpstr,"T48 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}
	else if ( data->device_info.family_id == MXT224_FAMILYID ) //mXT224
	{
		sprintf(buf,"no objects!!!\n");
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

	printk("[touch] cfg[0]=%d, cfg[1]=%d, cfg[2]=%d\n",cfg[0],cfg[1],cfg[2]);
	
//	mxt_write_byte(data->client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, data) + MXT_ADR_T6_BACKUPNV,MXT_CMD_T6_BACKUP);
	msleep(MXT_BACKUP_TIME); //wait to backup

	//sw reset
//	mxt_write_byte(data->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6,data) + MXT_ADR_T6_RESET, 0x01);
	if ( data->device_info.family_id == MXT224E_FAMILYID ) //mXT224E
	{
		msleep(MXT224E_RESET_TIME);
	}
	else
	{
		msleep(MXT224_RESET_TIME);
	}	

	enable_irq(data->irq);

	printk("[touch] config finish!\n");

	return count;
}

static void mxt_backup_nvm(struct mxt_data *mxt)
{
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt) + MXT_ADR_T6_BACKUPNV,MXT_CMD_T6_BACKUP);
	msleep(MXT_BACKUP_TIME); //wait to backup

	//sw reset
	mxt_write_byte(mxt->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6,mxt) + MXT_ADR_T6_RESET, 0x01);
	if ( mxt->device_info.family_id == MXT224E_FAMILYID ) //mXT224E
	{
		msleep(MXT224E_RESET_TIME);
	}
	else
	{
		msleep(MXT224_RESET_TIME);
	}
}

static void disable_auto_calibration(struct mxt_data *mxt)
{
	cancel_delayed_work_sync(&mxt->d_work_disable_auto_calibration);
	flush_workqueue(g_atmel_wq_dis_auto_cal);

	queue_delayed_work(g_atmel_wq_dis_auto_cal, &mxt->d_work_disable_auto_calibration, msecs_to_jiffies(DISABLE_AUTO_CALIBRATION_TIME));
}

static int touch_config_zero(struct mxt_data *mxt)
{
	int i;

	printk("[touch] touch_config_zero()++\n");

	if ( mxt->device_info.family_id == MXT224E_FAMILYID ) //mXT224E
	{
		printk("[touch] touch_config_zero(): mXT224E: family_id = 0x%x\n", mxt->device_info.family_id);

		for ( i=0; i < 8; i++ )
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

		for ( i=0; i < 15; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_PROXIMITY_T23, mxt)+i, 0);
		}

		for ( i=0; i < 14; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+i, 0);
		}

		for ( i=0; i < 5; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GRIPSUPPRESSION_T40, mxt)+i, 0);
		}

		for ( i=0; i < 8; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+i, 0);
		}

		for ( i=0; i < 9; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+i, 0);
		}

		for ( i=0; i < 10; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_STYLUS_T47, mxt)+i, 0);
		}

		for ( i=0; i < 54; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+i, 0);
		}
	}
	else if ( mxt->device_info.family_id == MXT224_FAMILYID ) //mXT224
	{
		printk("[touch] touch_config_zero(): mXT224: family_id = 0x%x\n", mxt->device_info.family_id);

		for ( i=0; i < 8; i++ )
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

		for ( i=0; i < 32; i++ )
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

		for ( i=0; i < 12; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_GRIPFACESUPPRESSION_T20, mxt)+i, 0);
		}

		for ( i=0; i < 17; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+i, 0);
		}

		for ( i=0; i < 15; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_PROXIMITY_T23, mxt)+i, 0);
		}

		for ( i=0; i < 19; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, mxt)+i, 0);
		}

		for ( i=0; i < 14; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+i, 0);
		}

		for ( i=0; i < 6; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, mxt)+i, 0);
		}
	}
	
	mxt_backup_nvm(mxt);

	printk("[touch] touch_config_zero()--\n");

	return 0;
}

static int init_touch_config(struct mxt_data *mxt)
{
	int i;

	printk("[touch] init_touch_config()++\n");

	if ( mxt->device_info.family_id == MXT224E_FAMILYID ) //mXT224E
	{
		printk("[touch] init_touch_config(): mXT224E: family_id = 0x%x\n", mxt->device_info.family_id);

		for ( i=0; i < 8; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_USER_INFO_T38, mxt)+i, 0);
		}

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt), 64);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt)+1, 15);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt)+2, 10);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt), 25);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+1, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+2, 5);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+3, 5);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+5, 0);
		init_T8_auto_cal_config(mxt, 1, 1);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt), T9_CTRL_VAL);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+1, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+2, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+3, 19);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+4, 11);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+5, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+6, 32);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+7, 60);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+8, 2);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+9, 1); //ori=7
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+10, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+11, 3);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+12, 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+13, 30);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+14, 5);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+15, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+16, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+17, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+18, 191);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+19, 3);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+20, 27);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+21, 2);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+22, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+23, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+24, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+25, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+26, 138);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+27, 70);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+28, 148);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+29, 87);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+30, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+31, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+32, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+33, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+34, 0);

		for ( i=0; i < 11; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt)+i, 0);
		}

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, mxt), 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, mxt)+1, 0);

		for ( i=0; i < 16; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_GPIOPWM_T19, mxt)+i, 0);
		}

		for ( i=0; i < 15; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_PROXIMITY_T23, mxt)+i, 0);
		}

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

		for ( i=0; i < 5; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GRIPSUPPRESSION_T40, mxt)+i, 0);
		}

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt), 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+1, 50);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+2, 40);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+3, 35);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+4, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+5, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+6, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt)+7, 0);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt), 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+1, 3);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+2, 16);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+3, 32);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+4, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+5, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+6, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+7, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, mxt)+8, 0);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_STYLUS_T47, mxt), 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_STYLUS_T47, mxt)+1, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_STYLUS_T47, mxt)+2, 50);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_STYLUS_T47, mxt)+3, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_STYLUS_T47, mxt)+4, 3);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_STYLUS_T47, mxt)+5, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_STYLUS_T47, mxt)+6, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_STYLUS_T47, mxt)+7, 100);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_STYLUS_T47, mxt)+8, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_STYLUS_T47, mxt)+9, 10);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt), 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+1, 4);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+2, 66);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+3, 50);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+4, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+5, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+6, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+7, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+8, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+9, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+10, 16);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+11, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+12, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+13, 6);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+14, 6);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+15, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+16, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+17, 100);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+18, 4);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+19, 64);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+20, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+21, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+22, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+23, 5);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+24, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+25, 38);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+26, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+27, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+28, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+29, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+30, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+31, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+32, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+33, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+34, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+35, 50);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+36, 2);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+37, 3);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+38, 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+39, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+40, 5);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+41, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+42, 40);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+43, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+44, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+45, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+46, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+47, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+48, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+49, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+50, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+51, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+52, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_NOISESUPPRESSION_T48, mxt)+53, 3);
	}
	else if ( mxt->device_info.family_id == MXT224_FAMILYID ) //mXT224
	{
		printk("[touch] init_touch_config(): mXT224: family_id = 0x%x\n", mxt->device_info.family_id);
		
		for ( i=0; i < 8; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_USER_INFO_T38, mxt)+i, 0);
		}

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt), 65);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt)+1, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt)+2, 20);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt), 9);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+1, 5);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+2, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+3, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+4, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+5, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+6, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+7, 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+8, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+9, 0);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt), T9_CTRL_VAL);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+1, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+2, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+3, 19);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+4, 11);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+5, 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+6, 16);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+7, 35);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+8, 2);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+9, 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+10, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+11, 2);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+12, 2);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+13, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+14, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+15, 5);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+16, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+17, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+18, 234);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+19, 3);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+20, 27);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+21, 2);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+22, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+23, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+24, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+25, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+26, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+27, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+28, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+29, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+30, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt)+31, 12);

		for ( i=0; i < 11; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, mxt)+i, 0);
		}

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, mxt), 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, mxt)+1, 0);

		for ( i=0; i < 16; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_GPIOPWM_T19, mxt)+i, 0);
		}

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_GRIPFACESUPPRESSION_T20, mxt), 5);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_GRIPFACESUPPRESSION_T20, mxt)+1, 100);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_GRIPFACESUPPRESSION_T20, mxt)+2, 100);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_GRIPFACESUPPRESSION_T20, mxt)+3, 100);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_GRIPFACESUPPRESSION_T20, mxt)+4, 100);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_GRIPFACESUPPRESSION_T20, mxt)+5, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_GRIPFACESUPPRESSION_T20, mxt)+6, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_GRIPFACESUPPRESSION_T20, mxt)+7, 30);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_GRIPFACESUPPRESSION_T20, mxt)+8, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_GRIPFACESUPPRESSION_T20, mxt)+9, 4);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_GRIPFACESUPPRESSION_T20, mxt)+10, 15);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_GRIPFACESUPPRESSION_T20, mxt)+11, 5);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt), 5);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+1, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+2, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+3, 25);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+4, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+5, 231);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+6, 255);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+7, 4);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+8, 35);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+9, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+10, 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+11, 10);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+12, 15);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+13, 20);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+14, 25);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+15, 30);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22, mxt)+16, 4);

		for ( i=0; i < 15; i++ )
		{
			mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_PROXIMITY_T23, mxt)+i, 0);
		}

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
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+2, 224);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+3, 46);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+4, 88);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+5, 27);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+6, 176);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+7, 54);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+8, 244);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+9, 1);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+10, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+11, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+12, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, mxt)+13, 0);

		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, mxt), 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, mxt)+1, 0);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, mxt)+2, 3);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, mxt)+3, 16);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, mxt)+4, 16);
		mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, mxt)+5, 0);

	//	mxt_write_byte(mxt->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt) + MXT_ADR_T6_CALIBRATE, 1);
	}

	mxt_backup_nvm(mxt);
	disable_auto_calibration(mxt);

	printk("[touch] init_touch_config()--\n");

	return 0;
}

static ssize_t chip_config(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *mxt = i2c_get_clientdata(client);
	int cfg;

	disable_irq(mxt->irq);

	sscanf(buf, "%d\n", &cfg);

	printk("[touch] chip_config: cfg=%d\n", cfg);

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

	printk("[touch] chip_config: config finish!\n");

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

	printk("[touch] force_release()++ cfg=%d\n", cfg);

	if ( cfg == 0 )
	{
		force_release_fingers();
	}
	else
	{
		for (i = 0; i < MAX_FINGER_NUM; i++)
		{
			printk("[touch] force_release: finger[%d].status=0x%x\n", i, finger[i].status);
			printk("[touch] force_release: finger[%d].area=%d\n", i, finger[i].area);
			printk("[touch] force_release: finger[%d].x=%d\n", i, finger[i].x);
			printk("[touch] force_release: finger[%d].y=%d\n", i, finger[i].y);
			printk("[touch] force_release: finger[%d].amplitude=%d\n", i, finger[i].amplitude);
			printk("[touch] force_release: finger[%d].vector=%d\n", i, finger[i].vector);
		}
	}
	
	printk("[touch] force_release()--\n");

	return count;
}

static ssize_t show_release_fingers(struct device *dev, struct device_attribute *devattr, char *buf)
{
	sscanf(buf, "breeze: skip cts check\n");
	return strlen(buf);
}

DEVICE_ATTR(load_cfg_padfone, S_IRUGO | S_IWUSR, NULL, chip_config);
DEVICE_ATTR(cfg_padfone, S_IRUGO | S_IWUSR, NULL, store_mode2);
DEVICE_ATTR(dump_T7, 0755, dump_T7, NULL);
DEVICE_ATTR(dump_T15, 0755, dump_T15, NULL);
DEVICE_ATTR(dump_T20, 0755, dump_T20, NULL);
DEVICE_ATTR(dump_T38, 0755, dump_T38, NULL);
DEVICE_ATTR(dump_T40, 0755, dump_T40, NULL);
DEVICE_ATTR(dump_T48, 0755, dump_T48, NULL);
DEVICE_ATTR(release_fingers, S_IRUGO | S_IWUSR, show_release_fingers, force_release);

static struct attribute *mxt_attr[] = {
	&dev_attr_dump_T7.attr,
	&dev_attr_dump_T15.attr,
	&dev_attr_dump_T20.attr,
	&dev_attr_dump_T38.attr,
	&dev_attr_dump_T40.attr,
	&dev_attr_dump_T48.attr,
	&dev_attr_cfg_padfone.attr,
	&dev_attr_load_cfg_padfone.attr,
	&dev_attr_release_fingers.attr,
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
		mxt_debug(DEBUG_TRACE, "[touch] debug data page = %d\n", page);		
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
					"[touch] Waiting for debug diag command "
					"to propagate...\n");

			}
		        error = mxt_read_block(mxt->client, page_address, 1, 
					&page);
			if (error < 0)
				return error;
			mxt_debug(DEBUG_TRACE, "[touch] debug data page = %d\n", page);	
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
			mxt_debug(DEBUG_TRACE, "[touch] Waiting for debug diag command "
				"to propagate...\n");

		}	

		if (error < 0) {
			printk (KERN_WARNING 
				"[touch] Error writing to maXTouch device!\n");
			return error;
		}
	
		size = mxt->device_info.num_nodes * sizeof(u16);

		while (size > 0) {
			read_size = size > 128 ? 128 : size;
			mxt_debug(DEBUG_TRACE, 
				"[touch] Debug data read loop, reading %d bytes...\n",
				read_size);
			error = mxt_read_block(mxt->client, 
					       debug_data_addr, 
					       read_size, 
					       (u8 *) &data[offset]);
			if (error < 0) {
				printk(KERN_WARNING 
				       "[touch] Error reading debug data\n");
				goto error;
			}
			offset += read_size/2;
			size -= read_size;

			/* Select next page */
			error = mxt_write_byte(mxt->client, diagnostics_reg, 
					MXT_CMD_T6_PAGE_UP);
			if (error < 0) {
				printk(KERN_WARNING
					"[touch] Error writing to maXTouch device!\n");
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

	mxt = i2c_get_clientdata(client);

	if (mxt != NULL) {
		if ((mxt->last_read_addr == addr) &&
			(addr == mxt->msg_proc_addr)) {
			if  (i2c_master_recv(client, value, length) == length)
				return length;
			else
				return -EIO;
		} else {
			mxt->last_read_addr = addr;
		}
	}

	mxt_debug(DEBUG_TRACE, "[touch] Writing address pointer & reading %d bytes "
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
	if  (i2c_transfer(adapter, msg, 2) == 2)
		return length;
	else
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

	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;
	i2c_byte_transfer.le_addr = cpu_to_le16(addr);
	i2c_byte_transfer.data = value;
	if  (i2c_master_send(client, (u8 *) &i2c_byte_transfer, 3) == 3)
		return 0;
	else
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

	mxt_debug(DEBUG_TRACE, "[touch] Writing %d bytes to %d...", length, addr);
	if (length > 256)
		return -EINVAL;
	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;
	for (i = 0; i < length; i++)
		i2c_block_transfer.data[i] = *value++;
	i2c_block_transfer.le_addr = cpu_to_le16(addr);
	i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);
	if (i == (length + 2))
		return length;
	else
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
//extern void vibrator_enable(struct timed_output_dev *dev, int value);

static void process_virtual_key(u8 *message, struct mxt_data *mxt, int xpos)
{
	u8 status;
	u8 report_id;
	u8 pressed;

	status = message[MXT_MSG_T9_STATUS];
	report_id = message[0];


	if (status & MXT_MSGB_T9_RELEASE)
		pressed=0;
	else
		pressed=1;

	if (xpos <135){
		if(pressed && pre_button[VBTN_MENU]==0){
			input_report_key(mxt->input, KEY_MENU, 1);
			pre_button[VBTN_MENU]=1;

//			vibrator_enable(NULL, 35);
		}
		else if(!pressed && pre_button[VBTN_MENU]==1){
			input_report_key(mxt->input, KEY_MENU, 0);
			pre_button[VBTN_MENU]=0;

//			vibrator_enable(NULL, 35);
		}
	}

	else if(xpos > 135 && xpos <270){
		if(pressed && pre_button[VBTN_HOME]==0){
			input_report_key(mxt->input, KEY_HOME, 1);
			pre_button[VBTN_HOME]=1;

//			vibrator_enable(NULL, 35);
		}
		else if(!pressed && pre_button[VBTN_HOME]==1){
			input_report_key(mxt->input, KEY_HOME, 0);
			pre_button[VBTN_HOME]=0;

//			vibrator_enable(NULL, 35);
		}
	}

	else if (xpos > 270 && xpos <405){
		if(pressed && pre_button[VBTN_BACK]==0){
			input_report_key(mxt->input, KEY_BACK, 1);
			pre_button[VBTN_BACK]=1;

//			vibrator_enable(NULL, 35);
		}
		else if(!pressed && pre_button[VBTN_BACK]==1){
			input_report_key(mxt->input, KEY_BACK, 0);
			pre_button[VBTN_BACK]=0;

//			vibrator_enable(NULL, 35);
		}
	}

	else if (xpos > 405){
		if(pressed && pre_button[VBTN_SEARCH]==0){
			input_report_key(mxt->input, KEY_SEARCH, 1);
			pre_button[VBTN_SEARCH]=1;

//			vibrator_enable(NULL, 35);
		}
		else if(!pressed && pre_button[VBTN_SEARCH]==1){
			input_report_key(mxt->input, KEY_SEARCH, 0);
			pre_button[VBTN_SEARCH]=0;

//			vibrator_enable(NULL, 35);
		}
	}
	input_sync(mxt->input);
}
//ASUS_BSP joe1_--

/* Processes a touchscreen message. */
static void process_T9_message(u8 *message, struct mxt_data *mxt)
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

#ifdef CAL_REPORT_RATE
	static u32 u32TouchCount = 0;
	static u64 u64timestamp_start = 0;
	u64 u64timestamp_end = 0;
#endif

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

	if (finger[touch_number].y > 975)
	{
		process_virtual_key(message, mxt, xpos);
	}
	else
	{
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

//				finger[touch_number].amplitude = message[MXT_MSG_T9_TCHAMPLITUDE];
			}
#endif

			if ( g_bIsCmdCalibrationOk )
			{
				if ( status & MXT_MSGB_T9_PRESS )
				{
					first_finger[touch_number].status = 1;

					first_finger[touch_number].start_x = finger[touch_number].x;
					first_finger[touch_number].start_y = finger[touch_number].y;

					//printk("[joe1] g_bIsCmdCalibrationOk:press: touch_number=%d; start_x=%d; start_y=%d\n", touch_number, first_finger[touch_number].start_x, first_finger[touch_number].start_y);
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

						//printk("[touch] g_bIsCmdCalibrationOk:release:i=%d; status=%d: bCheckFingerMove=false\n", i, first_finger[i].status);
							
						break;
					}
					else
					{
						bCheckFingerMove = true;

						//printk("[touch] g_bIsCmdCalibrationOk:release:i=%d; status=%d: bCheckFingerMove=true\n", i, first_finger[i].status);
					}
				}

				if ( bCheckFingerMove )
				{
					first_finger[touch_number].dx = finger[touch_number].x - first_finger[touch_number].start_x;
					first_finger[touch_number].dy = finger[touch_number].y - first_finger[touch_number].start_y;
					first_finger[touch_number].dsum = abs(first_finger[touch_number].dx) + abs(first_finger[touch_number].dy);

					printk("[touch] bCheckFingerMove: touch_number=%d; dsum=%d\n", touch_number, first_finger[touch_number].dsum);

					if ( first_finger[touch_number].dsum > MOVE_DISTANCE)
					{
						queue_delayed_work(g_atmel_wq_dis_auto_cal, &mxt->d_work_disable_auto_calibration, msecs_to_jiffies(DISABLE_AUTO_CALIBRATION_DELAY));

						g_bIsCmdCalibrationOk = false;

						printk("[touch] bCheckFingerMove: Disable auto-cal!!!\n");
					}

					printk("[touch] CheckFingerMove!!!\n");
				}
				else
				{
					printk("[touch] NOT CheckFingerMove\n");
				}
			}
		}

		for (i = 0; i < MAX_FINGER_NUM; i++)
		{
			if (finger[i].area)
			{
				active_touches++;

				input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR, finger[i].area);
				input_report_abs(mxt->input, ABS_MT_POSITION_X, finger[i].x);
				input_report_abs(mxt->input, ABS_MT_POSITION_Y, finger[i].y);
				input_report_abs(mxt->input, ABS_MT_PRESSURE, finger[i].amplitude);
//				input_report_abs(mxt->input, ABS_MT_ORIENTATION, finger[i].vector);

				input_mt_sync(mxt->input);
			}
		}

		input_report_key(mxt->input, BTN_TOUCH, !!active_touches);

		if (active_touches == 0)
			input_mt_sync(mxt->input);

		input_sync(mxt->input);

//		mxt_debug(DEBUG_TRACE, "[touch] process_T9_message: input_sync; active_touches=%d\n", active_touches);

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
				printk("[touch] report rate= %d\n", u32TouchCount/5);

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
	}

	if ( debug >= DEBUG_TRACE )
	{
		if (status & MXT_MSGB_T9_SUPPRESS) 
		{
			printk("[touch] SUPRESS\n");
		}
		else
		{
			if (status & MXT_MSGB_T9_DETECT)
			{
				printk("[touch] DETECT:%s%s%s%s\n", 
					((status & MXT_MSGB_T9_PRESS) ? " PRESS" : ""), 
					((status & MXT_MSGB_T9_MOVE) ? " MOVE" : ""), 
					((status & MXT_MSGB_T9_AMP) ? " AMP" : ""), 
					((status & MXT_MSGB_T9_VECTOR) ? " VECT" : ""));
			}
			else if (status & MXT_MSGB_T9_RELEASE)
			{
				printk("[touch] RELEASE\n");
			}
		}

		printk("[touch] X=%d, Y=%d, TOUCHSIZE=%d, Amplitude=%d, VECTOR=0x%x\n", 
			finger[touch_number].x,
			finger[touch_number].y,
			finger[touch_number].area,
			finger[touch_number].amplitude,
			finger[touch_number].vector);
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
			printk("[touch] configuration checksum is 0x%lx\n",(long unsigned int)cfg_crc_onchip);
			mxt_debug(DEBUG_INFO, "[touch] mxt->configuration_crc = 0x%lx\n",(long unsigned int)mxt->configuration_crc);

			if (cfg_flag)
			{
				if ( mxt->device_info.family_id == MXT224E_FAMILYID ) //mXT224E
				{
					cfg_crc = 0xc5af45;

					printk("[touch] MXT224E_FAMILYID: cfg_crc = 0x%lx\n",(long unsigned int)cfg_crc);
				}
				else if ( mxt->device_info.family_id == MXT224_FAMILYID ) //mXT224
				{
					cfg_crc = 0xc15972;

					printk("[touch] MXT224_FAMILYID: cfg_crc = 0x%lx\n",(long unsigned int)cfg_crc);
				}
				else
				{
					printk("[touch] UNKNOWN_FAMILYID: cfg_crc = 0x%lx\n",(long unsigned int)cfg_crc);
				}

				if (cfg_crc_onchip != cfg_crc)
				{
					printk("[touch] start BACKUP\n");

					init_touch_config(mxt);

					// save power state values for suspend/resume
					mxt_read_block(mxt->client, mxt->t7_addr, ARRAY_SIZE(mxt->t7_data), mxt->t7_data);

					mxt_read_block(mxt->client, 4, 2, (u8 *) buf);

					mxt->device_info.x_size = buf[0];
					mxt->device_info.y_size = buf[1];

					printk("[touch] after BACKUP: x=%d, y=%d\n", mxt->device_info.x_size, mxt->device_info.y_size);

					printk("[touch] config BACKUP finished\n");
				}
				else
				{
					printk("[touch] config dont need BACKUP!!!\n");
				}

				cfg_flag = false;
			}
			//ASUS_BSP joe1_---

			status = message[1];
			if (status & MXT_MSGB_T6_COMSERR)
				dev_err(&client->dev, "[touch] maXTouch checksum error\n");
			if (status & MXT_MSGB_T6_CFGERR) {
				/* 
				 * Configuration error. A proper configuration
				 * needs to be written to chip and backed up.
				 */
				dev_err(&client->dev, "[touch] maXTouch configuration error\n");
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
				
				/* Calibration in action, no need to react */
				dev_info(&client->dev, "[touch] maXTouch calibration in progress\n");
			}
			if (status & MXT_MSGB_T6_SIGERR) {
				/* 
				 * Signal acquisition error, something is seriously
				 * wrong, not much we can in the driver to correct
				 * this
				 */
				dev_err(&client->dev, "[touch] maXTouch acquisition error\n");
			}
			if (status & MXT_MSGB_T6_OFL) {
				/*
				 * Cycle overflow, the acquisition is too short.
				 * Can happen temporarily when there's a complex
				 * touch shape on the screen requiring lots of
				 * processing.
				 */
				dev_err(&client->dev, "[touch] maXTouch cycle overflow\n");
			}
			if (status & MXT_MSGB_T6_RESET) {
				/* Chip has reseted, no need to react. */
				dev_info(&client->dev, "[touch] maXTouch chip reset\n");
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

						printk("[touch] calibration!!!CAL_TYPE_CMD!\n");

						break;
					default:
						break;
				}
				
				/* Chip status back to normal. */
				dev_info(&client->dev, "[touch] maXTouch status normal\n");
			}
			break;

		case MXT_SPT_SELFTEST_T25:
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev, "[touch] Receiving Self-Test msg\n");

			if (message[MXT_MSG_T25_STATUS] == MXT_MSGR_T25_OK)
			{
				if (debug >= DEBUG_TRACE)
					dev_info(&client->dev, "[touch] maXTouch: Self-Test OK\n");
			}
			else
			{
				dev_err(&client->dev,
					"[touch] maXTouch: Self-Test Failed [%02x]:"
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

		case MXT_SPT_CTECONFIG_T46:
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev, "[touch] Receiving CTE message...\n");

			status = message[MXT_MSG_T46_STATUS];

			if (status & MXT_MSGB_T46_CHKERR)
				dev_err(&client->dev, "[touch] maXTouch: Power-Up CRC failure\n");

			break;

		default:
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev, "[touch] maXTouch: Unknown message!\n");

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

//	mxt_debug(DEBUG_TRACE, "[touch] mxt_irq_handler:\n");
	
	do
	{
		/* Read next message, reread on failure. */
		/* TODO: message length, CRC included? */
		for (i = 1; i < I2C_RETRY_COUNT; i++)
		{
			error = mxt_read_block(client,
					       message_addr,
					       message_length,// - 1,
					       message);
			if (error >= 0)
				break;

			mxt->read_fail_counter++;
			dev_err(&client->dev, "[touch] Failure reading maxTouch device. read_fail_counter=%d\n", mxt->read_fail_counter);
		}

		if (error < 0)
		{
			force_release_fingers();

			goto end_irq;
		}
		
		report_id = message[0];

		if (debug >= DEBUG_RAW)
		{
			printk("[touch] %s message :\n", REPORT_ID_TO_OBJECT_NAME(report_id, mxt));

			/* 5 characters per one byte */
			message_string = kmalloc(message_length * 5, GFP_KERNEL);

			if (message_string == NULL)
			{
				dev_err(&client->dev, "[touch] Error allocating memory\n");

				kfree(message);

				goto end_irq;
			}

			message_start = message_string;

			for (i = 0; i < message_length; i++)
			{
				message_string += sprintf(message_string, "0x%02X ", message[i]);
			}

			printk("[touch] %s\n", message_start);

			kfree(message_start);
		}
		
		if ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0))
		{
			/* Get type of object and process the message */
			object = mxt->rid_map[report_id].object;

			if ( object == MXT_TOUCH_MULTITOUCHSCREEN_T9 )
			{
				process_T9_message(message, mxt);
			}
			else
			{
				process_message(message, object, mxt);
			}
		}

//		mxt_debug(DEBUG_TRACE, "[touch] chgline: %d\n", mxt->read_chg());
	} while ( (report_id != MXT_END_OF_MESSAGES) && (report_id != 0) );

end_irq:
	return IRQ_HANDLED;
}


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
		dev_err(&client->dev, "[touch] Failure accessing maXTouch device\n");
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
				"[touch] Warning: maXTouch Variant ID [%d] not "
				"supported\n",
				mxt->device_info.variant_id);
			strcpy(mxt->device_info.variant_name, "UNKNOWN");
			/* identified = -ENXIO; */
		}
	/* MXT224E */
	} else if (mxt->device_info.family_id == MXT224E_FAMILYID) {
		strcpy(mxt->device_info.family_name, "mXT224E");

		if (mxt->device_info.variant_id == MXT224E_CAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Calibrated");
		} else {
			dev_err(&client->dev,
				"[touch] Warning: maXTouch Variant ID [%d] not "
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
				"[touch] Warning: maXTouch Variant ID [%d] not "
				"supported\n",
				mxt->device_info.variant_id);
			strcpy(mxt->device_info.variant_name, "UNKNOWN");
			/* identified = -ENXIO; */
		}
	/* Unknown family ID! */
	} else {
		dev_err(&client->dev,
			"[touch] Warning: maXTouch Family ID [%d] not supported\n",
			mxt->device_info.family_id);
		strcpy(mxt->device_info.family_name, "UNKNOWN");
		strcpy(mxt->device_info.variant_name, "UNKNOWN");
		/* identified = -ENXIO; */
	}

	dev_info(
		&client->dev,
		"[touch] Atmel maXTouch (Family %s (%X), Variant %s (%X)) Firmware "
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
		"[touch] Atmel maXTouch Configuration "
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

	mxt_debug(DEBUG_TRACE, "[touch] maXTouch driver reading configuration\n");

	object_table = kzalloc(sizeof(struct mxt_object) *
			       mxt->device_info.num_objs,
			       GFP_KERNEL);
	if (object_table == NULL) {
		printk(KERN_WARNING "[touch] maXTouch: Memory allocation failed!\n");
		error = -ENOMEM;
		goto err_object_table_alloc;
	}

	raw_ib_data = kmalloc(MXT_OBJECT_TABLE_ELEMENT_SIZE *
			mxt->device_info.num_objs + MXT_ID_BLOCK_SIZE,
			GFP_KERNEL);
	if (raw_ib_data == NULL) {
		printk(KERN_WARNING "[touch] maXTouch: Memory allocation failed!\n");
		error = -ENOMEM;
		goto err_ib_alloc;
	}

	/* Copy the ID data for CRC calculation. */
	memcpy(raw_ib_data, raw_id_data, MXT_ID_BLOCK_SIZE);
	ib_pointer = MXT_ID_BLOCK_SIZE;

	mxt->object_table = object_table;

	mxt_debug(DEBUG_TRACE, "[touch] maXTouch driver Memory allocated\n");

	object_info_address = MXT_ADDR_OBJECT_TABLE;

	report_id_count = 0;
	for (i = 0; i < mxt->device_info.num_objs; i++) {
		mxt_debug(DEBUG_TRACE, "[touch] Reading maXTouch at [0x%04x]: ",
			  object_info_address);

		error = mxt_read_block(client, object_info_address,
				       MXT_OBJECT_TABLE_ELEMENT_SIZE, buf);

		if (error < 0) {
			mxt->read_fail_counter++;
			dev_err(&client->dev,
				"[touch] maXTouch Object %d could not be read\n", i);
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
		mxt_debug(DEBUG_TRACE, "[touch] Type=%03d, Address=0x%04x, "
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
		printk(KERN_WARNING "[touch] maXTouch: Can't allocate memory!\n");
		error = -ENOMEM;
		goto err_rid_map_alloc;
	}

	mxt->report_id_count = report_id_count;
	if (report_id_count > 254) {	/* 0 & 255 are reserved */
			dev_err(&client->dev,
				"[touch] Too many maXTouch report id's [%d]\n",
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
		dev_err(&client->dev, "[touch] Error reading CRC\n");
	}

	crc = (buf[2] << 16) | (buf[1] << 8) | buf[0];

	if (calculate_infoblock_crc(&calculated_crc, raw_ib_data,
				    ib_pointer)) {
		printk(KERN_WARNING "[touch] Error while calculating CRC!\n");
		calculated_crc = 0;
	}
	kfree(raw_ib_data);

	mxt_debug(DEBUG_TRACE, "\n[touch] Reported info block CRC = 0x%6X\n", crc);
	mxt_debug(DEBUG_TRACE, "[touch] Calculated info block CRC = 0x%6X\n\n",
		       calculated_crc);
	
	if (crc == calculated_crc) {
		mxt->info_block_crc = crc;
	} else {
		mxt->info_block_crc = 0;
		printk(KERN_ALERT "[touch] maXTouch: Info block CRC invalid!\n");
	}

	if (debug >= DEBUG_VERBOSE) {

		dev_info(&client->dev, "[touch] maXTouch: %d Objects\n",
				mxt->device_info.num_objs);

		for (i = 0; i < mxt->device_info.num_objs; i++) {
			dev_info(&client->dev, "[touch] Type:\t\t\t[%d]: %s\n",
				 object_table[i].type,
				 object_type_name[object_table[i].type]);
			dev_info(&client->dev, "[touch] \tAddress:\t0x%04X\n",
				object_table[i].chip_addr);
			dev_info(&client->dev, "[touch] \tSize:\t\t%d Bytes\n",
				 object_table[i].size);
			dev_info(&client->dev, "[touch] \tInstances:\t%d\n",
				 object_table[i].instances);
			dev_info(&client->dev, "[touch] \tReport Id's:\t%d\n",
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

	mxt_debug(DEBUG_POWER, "[touch] mxt_suspend()++\n");

	if (mxt->is_suspended)
		return 0;

	cancel_delayed_work_sync(&mxt->d_work_do_calibration);
	cancel_delayed_work_sync(&mxt->d_work_disable_auto_calibration);
	flush_workqueue(g_atmel_wq_dis_auto_cal);

	g_bIsCmdCalibrationOk = false;
	g_bIsCmdCalibration = false;

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
			dev_err(dev, "[touch] power off failed");
			goto err_write_block;
		}
	}

	force_release_fingers();

	mxt->is_suspended = true;

	mxt_debug(DEBUG_POWER, "[touch] mxt_suspend()--\n");

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

	mxt_debug(DEBUG_POWER, "[touch] mxt_resume()++\n");

	if (!mxt->is_suspended)
		return 0;

	/* power on the device */
	if (mxt->power_on)
	{
		error = mxt->power_on(true);
		if (error)
		{
			dev_err(dev, "[touch] power on failed");
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

	mxt_debug(DEBUG_POWER, "[touch] mxt_resume()--\n");

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

	mxt_debug(DEBUG_POWER, "[touch] mxt_early_suspend()++\n");

	if ( g_bIsPadAttach == false )
	{
		mxt_suspend(&mxt->client->dev);
	}

	g_bIsSysSuspended = true;

	mxt_debug(DEBUG_POWER, "[touch] mxt_early_suspend()--\n");
}

static void mxt_late_resume(struct early_suspend *h)
{
	struct mxt_data *mxt = container_of(h, struct mxt_data, early_suspend);

	mxt_debug(DEBUG_POWER, "[touch] mxt_late_resume()++\n");

	if ( g_bIsPadAttach == false )
	{
		mxt_resume(&mxt->client->dev);
	}

	g_bIsSysSuspended = false;

	mxt_debug(DEBUG_POWER, "[touch] mxt_late_resume()--\n");
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
static int TestATMELTouch(struct i2c_client *apClient)
{

	int lnResult = I2C_TEST_PASS;
	u8 id_data[7];
    
	i2c_log_in_test_case("TestATMELTouch++\n");
	if (mxt_identify(apClient, g_mxt, id_data) < 0) {
        	i2c_log_in_test_case("TestATMELTouch failed\n");        
		lnResult = I2C_TEST_TOUCH_FAIL;
	}
    
	i2c_log_in_test_case("TestATMELTouch--\n");
	return lnResult;
};

static struct i2c_test_case_info gTouchTestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(TestATMELTouch),
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
	int i; //ASUS_BSP joe1_++
	
	mxt_debug(DEBUG_INFO, "[touch] mXT224: mxt_probe\n");

	mxt_debug(DEBUG_INFO, "[touch] maXTouch driver v. %s\n", DRIVER_VERSION);
	mxt_debug(DEBUG_INFO, "[touch] \t \"%s\"\n", client->name);
	mxt_debug(DEBUG_INFO, "[touch] \taddr:\t0x%04x\n", client->addr);
	mxt_debug(DEBUG_INFO, "[touch] \tirq:\t%d\n", client->irq);
	mxt_debug(DEBUG_INFO, "[touch] \tflags:\t0x%04x\n", client->flags);
	mxt_debug(DEBUG_INFO, "[touch] \tadapter:\"%s\"\n", client->adapter->name);
	mxt_debug(DEBUG_INFO, "[touch] \tdevice:\t\"%s\"\n", client->dev.init_name);

	/* Allocate structure - we need it to identify device */
	mxt = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (mxt == NULL) {
		dev_err(&client->dev, "[touch] insufficient memory\n");
		error = -ENOMEM;
		goto err_mxt_alloc;
	}

	g_mxt = mxt; //ASUS_BSP joe1_++

	id_data = kmalloc(MXT_ID_BLOCK_SIZE, GFP_KERNEL);
	if (id_data == NULL) {
		dev_err(&client->dev, "[touch] insufficient memory\n");
		error = -ENOMEM;
		goto err_id_alloc;
	}

	input = input_allocate_device();
	if (!input) {
		dev_err(&client->dev, "[touch] error allocating input device\n");
		error = -ENOMEM;
		goto err_input_dev_alloc;
	}

	/* Initialize Platform data */

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "[touch] platform data is required!\n");
		error = -EINVAL;
		goto err_pdata;
	}

	mxt_debug(DEBUG_TRACE, "[touch] Platform OK: pdata = 0x%08x\n",  (unsigned int) pdata);

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
			dev_err(&client->dev, "[touch] hw init failed");
			goto err_init_hw;
		}
	}

	/* power on the device */
	if (mxt->power_on) {
		error = mxt->power_on(true);
		if (error) {
			dev_err(&client->dev, "[touch] power on failed");
			goto err_pwr_on;
		}
	}

	mxt_debug(DEBUG_TRACE, "[touch] maXTouch driver identifying chip\n");

	if (mxt_identify(client, mxt, id_data) < 0) {
		dev_err(&client->dev, "[touch] Chip could not be identified\n");
		error = -ENODEV;
		goto err_identify;
	}

	/* Chip is valid and active. */
	mxt_debug(DEBUG_TRACE, "[touch] maXTouch driver allocating input device\n");

	mxt->client = client;
	mxt->input  = input;
	
	mutex_init(&mxt->debug_mutex);

	mxt_debug(DEBUG_TRACE, "[touch] maXTouch driver creating device name\n");

	input->name = "atmel-maxtouch"; //ASUS_BSP joe1_++
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	mxt_debug(DEBUG_INFO, "[touch] maXTouch name: \"%s\"\n", input->name);

	mxt_debug(DEBUG_INFO, "[touch] maXTouch driver setting abs parameters\n");

	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
//	__set_bit(EV_MSC, input->evbit); //ASUS_BSP joe1_++
//	input->mscbit[0] = BIT_MASK(MSC_GESTURE); //ASUS_BSP joe1_++

	set_bit(BTN_TOUCH, input->keybit);
	set_bit(INPUT_PROP_DIRECT, input->propbit);
//ASUS_BSP joe1_++
	set_bit(KEY_BACK,input->keybit);
	set_bit(KEY_MENU,input->keybit);
	set_bit(KEY_HOME,input->keybit);
	set_bit(KEY_SEARCH,input->keybit);

	for (i=0; i<NUM_OF_VBTN; i++){
		pre_button[i]=0;
	}
//ASUS_BSP joe1_--

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
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, MXT_MAX_REPORTED_PRESSURE,
			     0, 0);
//	input_set_abs_params(input, ABS_MT_ORIENTATION, 0, 255, 0, 0);
//ASUS_BSP joe1_--
	
	mxt_debug(DEBUG_TRACE, "[touch] maXTouch driver setting client data\n");
	i2c_set_clientdata(client, mxt);

	mxt_debug(DEBUG_TRACE, "[touch] maXTouch driver setting drv data\n");
	input_set_drvdata(input, mxt);

	mxt_debug(DEBUG_TRACE, "[touch] maXTouch driver input register device\n");
	error = input_register_device(mxt->input);
	if (error < 0) {
		dev_err(&client->dev,
			"[touch] Failed to register input device\n");
		goto err_register_device;
	}

	error = mxt_read_object_table(client, mxt, id_data);
	if (error < 0)
		goto err_read_ot;

//ASUS_BSP joe1_++
	mxt->t7_addr = MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt);

	// save power state values for suspend/resume
	mxt_read_block(mxt->client, mxt->t7_addr, ARRAY_SIZE(mxt->t7_data), mxt->t7_data);
//ASUS_BSP joe1_--

	/* Create debugfs entries. */
	mxt->debug_dir = debugfs_create_dir("maXTouch", NULL);
	if (mxt->debug_dir == ERR_PTR(-ENODEV)) {
		/* debugfs is not enabled. */
		printk(KERN_WARNING "[touch] debugfs not enabled in kernel\n");
	} else if (mxt->debug_dir == NULL) {
		printk(KERN_WARNING "[touch] error creating debugfs dir\n");
	} else {
		mxt_debug(DEBUG_TRACE, "[touch] created \"maXTouch\" debugfs dir\n");
		
		debugfs_create_file("deltas", S_IRUSR, mxt->debug_dir, mxt, 
				    &delta_fops);
		debugfs_create_file("refs", S_IRUSR, mxt->debug_dir, mxt,
				    &refs_fops);
	}

	/* Allocate the interrupt */
	mxt_debug(DEBUG_TRACE, "[touch] maXTouch driver allocating interrupt...\n");

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
				"[touch] failed to allocate irq %d\n", mxt->irq);
			goto err_irq;
		}
	}

	mxt_debug(DEBUG_INFO, "[touch] touchscreen, irq %d\n", mxt->irq);
		
#if 0
	t38_data = kmalloc(t38_size*sizeof(u8), GFP_KERNEL);

	if (t38_data == NULL) {
		dev_err(&client->dev, "[touch] insufficient memory\n");
		error = -ENOMEM;
		goto err_t38;
	}

	t38_addr = MXT_BASE_ADDR(MXT_USER_INFO_T38, mxt);
	mxt_read_block(client, t38_addr, t38_size, t38_data);
	dev_info(&client->dev, "[touch] VERSION:%02x.%02x.%02x, DATE: %d/%d/%d\n",
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
	INIT_DELAYED_WORK(&mxt->d_work_disable_auto_calibration, disable_auto_cal_delayed_work);
	INIT_DELAYED_WORK(&mxt->d_work_do_calibration, do_cal_work);

	mxt->attrs.attrs = mxt_attr;
	error = sysfs_create_group(&client->dev.kobj, &mxt->attrs);
	if (error) {
		dev_err(&client->dev, "[touch] Not able to create the sysfs\n");
		goto exit_remove;
	}

	queue_delayed_work(g_atmel_wq_dis_auto_cal, &mxt->d_work_disable_auto_calibration, msecs_to_jiffies(DISABLE_AUTO_CALIBRATION_TIME_FIRST));
//ASUS_BSP joe1_--


#ifdef CONFIG_I2C_STRESS_TEST
	printk("[touch] ATMEL_Touch add test case+\n");

       i2c_add_test_case(client, "ATMELTouch",ARRAY_AND_SIZE(gTouchTestCaseInfo));

	printk("[touch] ATMEL_Touch add test case-\n");
#endif

	mxt_write_byte(mxt->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6,mxt) + MXT_ADR_T6_RESET, 0x01); // Simpson

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
err_mxt_alloc:
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

	i2c_set_clientdata(client, NULL);
	if (debug >= DEBUG_TRACE)
		dev_info(&client->dev, "[touch] Touchscreen unregistered\n");

	return 0;
}

static const struct i2c_device_id mxt_idtable[] = {
	{"maXTouch", 0,},
	{ }
};

MODULE_DEVICE_TABLE(i2c, mxt_idtable);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "maXTouch",
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
	int err;

//ASUS_BSP joe1_++
	g_atmel_wq_attach_detach = create_singlethread_workqueue("g_atmel_wq_attach_detach");
	if (!g_atmel_wq_attach_detach) {
		printk("[touch_pad] %s: create workqueue failed: g_atmel_wq_attach_detach\n", __func__);
	}

	g_atmel_wq_dis_auto_cal = create_singlethread_workqueue("g_atmel_wq_dis_auto_cal");
	if (!g_atmel_wq_dis_auto_cal) {
		printk("[touch_pad] %s: create workqueue failed: g_atmel_wq_dis_auto_cal\n", __func__);
	}
//ASUS_BSP joe1_--

	err = i2c_add_driver(&mxt_driver);
	if (err) {
		printk(KERN_WARNING "[touch] Adding maXTouch driver failed "
		       "(errno = %d)\n", err);
	} else {
		mxt_debug(DEBUG_TRACE, "[touch] Successfully added driver %s\n",
		          mxt_driver.driver.name);
	}

//ASUS_BSP joe1_++
#ifdef CONFIG_EEPROM_NUVOTON
	INIT_WORK(&g_mp_attach_work, attach_padstation_work);
	INIT_WORK(&g_mp_detach_work, detach_padstation_work);

	register_microp_notifier(&touch_mp_notifier);
#endif //CONFIG_EEPROM_NUVOTON
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
static void __devinit attach_padstation_work(struct work_struct *work)
{
	printk("[touch] attach_padstation_work()++\n");

	if ( g_mxt )
	{
		if ( !g_mxt->is_suspended )
		{
			mxt_suspend(&g_mxt->client->dev);
		}

		force_release_fingers();
	}

	g_bIsPadAttach = true;

	printk("[touch] attach_padstation_work()--\n");
}

static void __devinit detach_padstation_work(struct work_struct *work)
{
	printk("[touch] detach_padstation_work()++\n");

	if ( g_mxt )
	{
		force_release_fingers();

		if ( (!g_bIsSysSuspended) && (g_mxt->is_suspended) )
		{
			mxt_resume(&g_mxt->client->dev);

			mxt_debug(DEBUG_INFO, "[touch] detach_padstation_work: enable_irq\n");
		}
	}

	g_bIsPadAttach = false;

	printk("[touch] detach_padstation_work()--\n");
}

int touch_attach_padstation(void)
{
//	mxt_debug(DEBUG_INFO, "[touch] touch_attach_padstation()++\n");
	printk("[touch] touch_attach_padstation()++\n");

	queue_work(g_atmel_wq_attach_detach, &g_mp_attach_work);

//	mxt_debug(DEBUG_INFO, "[touch] touch_attach_padstation()--\n");
	printk("[touch] touch_attach_padstation()--\n");

	return 0;
}
EXPORT_SYMBOL(touch_attach_padstation);

int touch_detach_padstation(void)
{
//	mxt_debug(DEBUG_INFO, "[touch] touch_detach_padstation()++\n");
	printk("[touch] touch_detach_padstation()++\n");

	queue_work(g_atmel_wq_attach_detach, &g_mp_detach_work);

//	mxt_debug(DEBUG_INFO, "[touch] touch_detach_padstation()--\n");
	printk("[touch] touch_detach_padstation()--\n");

	return 0;
}
EXPORT_SYMBOL(touch_detach_padstation);

static int touch_mp_event(struct notifier_block *this, unsigned long event, void *ptr)
{

        switch (event) {

        case P01_ADD:
                printk("[touch][MicroP] P01_ADD++\r\n");

                touch_attach_padstation();

//                mxt_debug(DEBUG_INFO, "[touch][MicroP] P01_ADD \r\n");
                printk("[touch][MicroP] P01_ADD--\r\n");

                return NOTIFY_DONE;

        case P01_REMOVE:
                printk("[touch][MicroP] P01_REMOVE++\r\n");

                touch_detach_padstation();
					
//                mxt_debug(DEBUG_INFO, "[touch][MicroP] P01_REMOVE \r\n");
                printk("[touch][MicroP] P01_REMOVE--\r\n");

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
