#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/microp_notify.h>
#include <linux/atomic.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/switch.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h> 
#endif

#define AL3010_DRV_NAME	"al3010_light_sensor"
#define DRIVER_VERSION		"1.0"

#define P01_EVENT_NOTIFY_LIGHTSENSOR_NO_ERROR (0)
#define P01_EVENT_NOTIFY_LIGHTSENSOR_ERROR (-1)

#define AL3010_NUM_CACHABLE_REGS	9

#define	AL3010_ALS_COMMAND	0x04
#define	AL3010_RAN_MASK	0x70
#define	AL3010_RAN_SHIFT	(4)

#define AL3010_MODE_COMMAND	0x00
#define AL3010_MODE_SHIFT	(0)
#define AL3010_MODE_MASK	0x07

#define AL3010_POW_MASK		0x01
#define AL3010_POW_UP		0x01
#define AL3010_POW_DOWN		0x00
#define AL3010_POW_RESET        0x04
#define AL3010_POW_SHIFT	(0)

#define	AL3010_ADC_LSB	0x0c
#define	AL3010_ADC_MSB	0x0d

static u8 al3010_reg[AL3010_NUM_CACHABLE_REGS] = 
	{0x00,0x01,0x0c,0x0d,0x10,0x1a,0x1b,0x1c,0x1d};

static int al3010_range[4] = {77806,19452,4863,1216};

struct al3010_data {
	struct i2c_client *client;
	struct mutex lock;
        struct input_dev   *input_dev;
	u8 reg_cache[AL3010_NUM_CACHABLE_REGS];
	u8 power_state_before_suspend;
};

struct _al3010_device {
    int (*init_hw)(struct i2c_client *client);
    u8    (*read_int_pin_state)(void);
    int irq;
} g_al3010_device;

struct al3010_data *g_al3010_data_as;
struct input_dev *this_input_dev_p02_als = NULL;

static struct workqueue_struct *al3010light_workqueue;
static struct delayed_work g_light_work;
static struct work_struct al3010_attached_P02_work;
static struct work_struct al3010_ISR_work;

struct i2c_client *al3010_client = NULL;

#define TOTALMAPS  14
static int g_al3010_light_level[TOTALMAPS] = 
    {1, 50, 100, 200, 300, 400, 500, 600, 700, 900, 1100, 1400, 1700, 2100};   //#14 levels

static int g_al3010_light_map[TOTALMAPS] = 
    {42, 84, 168, 253, 337, 421, 505, 590, 758, 927, 1179, 1432, 1769, 5000};    //#14 maps

bool g_bIsP01Attached = false;
bool g_al3010_switch_on = false;
static int g_AlsP01ProbeError = 0xff;

static int g_last_report_lux = 0;
static int g_al3010_switch_earlysuspend = 0;
static int g_ambient_suspended = 0;
static int g_al3010_light = 0;
static int g_last_al3010_light = 0;
static u16 g_al3010_light_calibration_fval_x1000 = 5120;

static int al3010_get_adc_value(struct i2c_client *client);
static void lightsensor_attached_pad_P01(struct work_struct *work);
static int al3010_put_property(struct i2c_client *client);
static void mp_als_interrupt_handler(struct work_struct *work);

void reportPadStationI2CFail(char *devname);;
extern void als_lux_report_event(int);
extern bool hdmi_exist(void);
extern int g_HAL_als_switch_on;
extern int g_polling_count;

atomic_t p02_ambient_update;


static struct switch_dev ls_switch_dev ={ 
        .name = AL3010_DRV_NAME,
        .index = 0,
};

#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>

#define I2C_TEST_FAIL_SENSOR (-1)
int get_calibrated_lux_value_from_P01(void);
int set_als_power_state_of_P01(int);

static int TestSensorI2C(struct i2c_client *apClient)
{

	int lnResult = I2C_TEST_PASS;


	i2c_log_in_test_case("TestSensorI2C++\n");

        set_als_power_state_of_P01(1);

        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(msecs_to_jiffies(2000));

        printk("reported lux=%d\r\n",get_calibrated_lux_value_from_P01());
	
        set_als_power_state_of_P01(0);

	i2c_log_in_test_case("TestSensorI2C--\n");

	return lnResult;
};

static struct i2c_test_case_info gSensorTestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(TestSensorI2C),
};


#endif
//ASUS_MERGE_END



/*
 *  Polling light sensor data
 */
void al3010_poll_work(struct work_struct *work)
{
    int lux = 0;

    printk(DBGMSK_PRX_G6"[als_P01]++al3010_poll_work\n");

    if(false == g_al3010_switch_on) {
        printk(DBGMSK_PRX_G6"[als_P01]al3010_poll_work: al3010 is off, leaving thread\n");
        return;
    }

    lux = al3010_get_adc_value(al3010_client);

    if(lux < 0) {
        printk(DBGMSK_PRX_G6"[als_P01] al3010_poll_work: failed err=%d\n", lux);
    }
    else {
        printk(DBGMSK_PRX_G6"[als_P01] al3010_poll_work: lux=%d\n", lux);
    }

    queue_delayed_work(al3010light_workqueue, &g_light_work, (HZ*2000/1000)) ; //200ms.

    printk(DBGMSK_PRX_G6"[als_P01]--al3010_poll_work\n");
}

/*
 * register access helpers
 */

static int __al3010_read_reg(struct i2c_client *client,
			       u32 reg, u8 mask, u8 shift)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	return (data->reg_cache[reg] & mask) >> shift;
}

static int __al3010_write_reg(struct i2c_client *client,
				u32 reg, u8 mask, u8 shift, u8 val)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	int ret = 0;
	u8 tmp;

	if (reg >= AL3010_NUM_CACHABLE_REGS)
		return -EINVAL;

	mutex_lock(&data->lock);

	tmp = data->reg_cache[reg];
	tmp &= ~mask;
	tmp |= val << shift;

	ret = i2c_smbus_write_byte_data(client, reg, tmp);
	if (!ret)
		data->reg_cache[reg] = tmp;

	mutex_unlock(&data->lock);
	return ret;
}

/*
 * internally used functions
 */

/* range */
static int al3010_get_range(struct i2c_client *client)
{
	int tmp;
	tmp = __al3010_read_reg(client, AL3010_ALS_COMMAND,
											AL3010_RAN_MASK, AL3010_RAN_SHIFT);;
	return al3010_range[tmp];
}

static int al3010_set_range(struct i2c_client *client, int range)
{
	return __al3010_write_reg(client, AL3010_ALS_COMMAND, 
											AL3010_RAN_MASK, AL3010_RAN_SHIFT, range);
}

/* resolution */
static int al3010_get_resolution(struct i2c_client *client)
{
	return 0;
}

static int al3010_set_resolution(struct i2c_client *client, int res)
{
	return 0;
}

/* mode */
static int al3010_get_mode(struct i2c_client *client)
{
	return __al3010_read_reg(client, AL3010_MODE_COMMAND,
		AL3010_MODE_MASK, AL3010_MODE_SHIFT);
}

static int al3010_set_mode(struct i2c_client *client, int mode)
{
    if(AL3010_POW_UP == (mode & AL3010_POW_MASK)) {
        g_al3010_switch_on = true;
    }
    else if(AL3010_POW_DOWN == (mode & AL3010_POW_MASK)) {
        g_al3010_switch_on = false;
    }

	return __al3010_write_reg(client, AL3010_MODE_COMMAND,
		AL3010_MODE_MASK, AL3010_MODE_SHIFT, mode);
}

/* power_state */
static int al3010_set_power_state(struct i2c_client *client, int state)
{
    if(AL3010_POW_UP == state) {
        g_al3010_switch_on = true;
    }
    else if(AL3010_POW_DOWN == state){
        g_al3010_switch_on = false;
    }

    printk(DBGMSK_PRX_G2"[als_P01] al3010_set_pwr_state: state:%d\n", g_al3010_switch_on);

        return i2c_smbus_write_byte_data(al3010_client, AL3010_MODE_COMMAND, g_al3010_switch_on);
/*	return __al3010_write_reg(client, AL3010_MODE_COMMAND,
				AL3010_POW_MASK, AL3010_POW_SHIFT, 
				state ? AL3010_POW_UP : AL3010_POW_DOWN);
*/
}

static int al3010_get_power_state(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	u8 cmdreg = data->reg_cache[AL3010_MODE_COMMAND];
	return (cmdreg & AL3010_POW_MASK) >> AL3010_POW_SHIFT;
}

int set_als_power_state_of_P01(int state)
{   
    int ret, indx;

    if(hdmi_exist() == 0) {
        printk("[als_P02] al3010 not access P02\n"); 
        return -1;
    }
    printk(DBGMSK_PRX_G2"[als_P01]set_als_pwr_state: state:%d\n", state);

    for(indx = 0; indx<5; indx++) {
        ret = al3010_set_power_state(al3010_client, state? AL3010_POW_UP:AL3010_POW_DOWN);
        if(!ret) {
              printk(DBGMSK_PRX_G2"[al3010][als] switch on al3010 success\n");
              break;
        }
        else
              printk(DBGMSK_PRX_G2"[al3010][als] i2c error retry = %d\n",indx);
    }  

    if (indx == 5) {
        reportPadStationI2CFail("al3010");
        return ret;   
    }

    if (state == 1) {
        ret = al3010_put_property(al3010_client);
        printk(DBGMSK_PRX_G2"[al3010][als] P02 light sensor dev_open\n");
    }
    else
        printk(DBGMSK_PRX_G2"[al3010][als] P02 light sensor dev_close\n");
    return ret;

}
EXPORT_SYMBOL(set_als_power_state_of_P01);

int get_calibrated_lux_value_from_P01(void)
{
    int lux;
    lux = al3010_get_adc_value(al3010_client);
    printk(DBGMSK_PRX_G6"[als_P01] al3010_light = %d \n",lux); 

    return lux;

}
EXPORT_SYMBOL(get_calibrated_lux_value_from_P01);

static int al3010_get_adc_value(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	int lsb, msb, range;
    u16 adc, k_adc;
    int i = 0;
    int lux =0;
 //   u16 adc_data = 0;

    printk(DBGMSK_PRX_G6"[als_P01]++al3010_get_adc_value \n");

	mutex_lock(&data->lock);
    msb = i2c_smbus_read_byte_data(client, 0x00);
    printk(DBGMSK_PRX_G6"[als_P01]al3010_get_adc_value: reg (0x00) = 0x%x\n", msb);
    msb = i2c_smbus_read_byte_data(client, 0x10);
    printk(DBGMSK_PRX_G6"[als_P01]al3010_get_adc_value: reg (0x10) = 0x%x\n", msb);

	lsb = i2c_smbus_read_byte_data(client, AL3010_ADC_LSB);

	if (lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, AL3010_ADC_MSB);
	mutex_unlock(&data->lock);


    printk(DBGMSK_PRX_G6"[als_P01]****al3010_get_adc_value: msb=%d, lsb=%d\n", msb, lsb);

	if (msb < 0)
		return msb;

	range = al3010_get_range(client);

  //  printk(DBGMSK_PRX_G6"[als_P01]al3010_get_adc_value: get range=%d\n", range);

	adc = (u32)((msb << 8) | lsb);
    k_adc = (u32)(adc * g_al3010_light_calibration_fval_x1000) >> 10;   //apply calibration value 

    for(i=0 ; i<TOTALMAPS ; i++) {
        if( k_adc < g_al3010_light_map[i] ) {
            lux = g_al3010_light_level[i];
            break;
        }
    }

    if(i==TOTALMAPS) {
        lux = g_al3010_light_level[13]; //12000 lux.
    }

    g_last_report_lux = lux;

    printk(DBGMSK_PRX_G6"[als_P01]--al3010_get_adc_value \n");

    return lux;

}

// ++Louis 20120215

#define AL3010_INT_STATUS          0x01
#define AL3010_INT_COMMAND         0x10
#define AL3010_INT_SHIFT           0x00       
#define AL3010_INT_IF              0x03
#define AL3010_INT_MASK            0xff

#define AL3010_LOW_THD_LSB         0x1A
#define AL3010_LOW_THD_MSB         0x1B
#define AL3010_HIGH_THD_LSB        0x1C
#define AL3010_HIGH_THD_MSB        0x1D

static int g_lsb_thd[16] = {0,8,17,33,50,67,84,101,118,152,185,236,30,98 ,232, 0xff};  
static int g_msb_thd[16] = {0,0,0 ,0 ,0 ,0 ,0 ,0  ,0  ,0  ,0  ,0  ,1  ,1 ,3  , 0xff};
//static int g_al3010_light_map[TOTALMAPS] / 5 = 
//    {8, 17, 33, 50, 67, 84, 101, 118, 152, 185, 236, 286, 354, 1000};   
//#14 maps / ( g_al3010_light_calibration_fval_x1000 >> 10 )

static int ALS_IF = 3;
//static u16 g_al3010_als_data = 0;
//static u16 g_last_al3010_als_data = 0;

static int al3010_put_property(struct i2c_client *client)
{
      //  struct al3010_data *data = i2c_get_clientdata(client);
        int lsb, msb;
       // int lux;
        int ret = 0;
        int status = 0;

      //  __al3010_write_reg(al3010_client, AL3010_ALS_COMMAND, AL3010_INT_IF, AL3010_INT_SHIFT, ALS_IF);

        ret = i2c_smbus_write_byte_data(client, AL3010_INT_COMMAND,  ALS_IF);
        if (ret < 0)  {
            printk(DBGMSK_PRX_G2"[als_P02] addr=0x%x, val=0x%x, ret=%d\n",AL3010_INT_COMMAND, ALS_IF, ret);
            switch_set_state(&ls_switch_dev, P01_EVENT_NOTIFY_LIGHTSENSOR_ERROR);
            return ret;
        }
        else
            printk(DBGMSK_PRX_G2"[als_P02] addr=0x%x, val=0x%x\n",AL3010_INT_COMMAND, ALS_IF);

        i2c_smbus_write_byte_data(client, AL3010_LOW_THD_LSB, g_lsb_thd[0]);
        lsb = i2c_smbus_read_byte_data(client, AL3010_LOW_THD_LSB);
        
        i2c_smbus_write_byte_data(client, AL3010_LOW_THD_MSB, g_lsb_thd[0]);
        msb = i2c_smbus_read_byte_data(client, AL3010_LOW_THD_MSB);

        printk(DBGMSK_PRX_G2"[als_P02]++ al3010_get_inital_low_threshold_value: msb=%d, lsb=%d\n", msb, lsb);

        i2c_smbus_write_byte_data(client, AL3010_HIGH_THD_LSB, g_lsb_thd[1]);
        lsb = i2c_smbus_read_byte_data(client, AL3010_HIGH_THD_LSB);
        

        i2c_smbus_write_byte_data(client, AL3010_HIGH_THD_MSB, g_msb_thd[1]);
        msb = i2c_smbus_read_byte_data(client, AL3010_HIGH_THD_MSB);

        printk(DBGMSK_PRX_G2"[als_P02]-- al3010_get_inital_high_threshold_value: msb=%d, lsb=%d\n", msb, lsb);

        status = i2c_smbus_read_byte_data(client, AL3010_INT_STATUS);

        if (status == 0) {
            printk(DBGMSK_PRX_G2"[al3010][als] P02 light sensor interrupt is cleared\n");
        }
        else
            printk(DBGMSK_PRX_G2"[al3010][als] P02 light sensor interrupt is triggered\n");
      
        return 0;
}

static void mp_als_interrupt_handler(struct work_struct *work)
{
     //   struct al3010_data *data = i2c_get_clientdata(al3010_client);
        int lsb, msb, range, i;
        int level = 0;
        u16 adc = 0;
        u16 k_adc = 0;

        lsb = i2c_smbus_read_byte_data(al3010_client, AL3010_ADC_LSB);
        if (lsb < 0) {
                switch_set_state(&ls_switch_dev,P01_EVENT_NOTIFY_LIGHTSENSOR_ERROR);
                return;
        }

        msb = i2c_smbus_read_byte_data(al3010_client, AL3010_ADC_MSB);
        if (msb < 0) {
                switch_set_state(&ls_switch_dev,P01_EVENT_NOTIFY_LIGHTSENSOR_ERROR);
                return;
        }

    printk(DBGMSK_PRX_G2"/********************************************************/\n");
    printk(DBGMSK_PRX_G2"[als_P02] al3010_get_raw_adc_value: msb=%d, lsb=%d\n", msb, lsb);


        range = al3010_get_range(al3010_client);

        adc = (u32)((msb << 8) | lsb) ;

        k_adc = (u32)(adc * g_al3010_light_calibration_fval_x1000) >> 10;   //apply calibration value 

        for(i=0 ; i<TOTALMAPS ; i++) {
                if( k_adc < g_al3010_light_map[i] ) {
                        g_al3010_light = g_al3010_light_level[i];
                        level = i;
                        break;
                }
        }
        if(i==TOTALMAPS) {
           level = 14;
           g_al3010_light = g_al3010_light_level[13];
        }

        printk(DBGMSK_PRX_G2"[als_P02] level= %d, raw adc= %d, cal_adc= %d, lux = %d\n",level, adc, k_adc, g_al3010_light);

        i2c_smbus_write_byte_data(al3010_client, AL3010_LOW_THD_LSB, g_lsb_thd[level]);
        lsb = i2c_smbus_read_byte_data(al3010_client, AL3010_LOW_THD_LSB);
        
        i2c_smbus_write_byte_data(al3010_client, AL3010_LOW_THD_MSB, g_msb_thd[level]);
        msb = i2c_smbus_read_byte_data(al3010_client, AL3010_LOW_THD_MSB);

        printk(DBGMSK_PRX_G2"[als_P02]++ al3010_get_low_threshold_value: msb=%d, lsb=%d\n", msb, lsb);

        i2c_smbus_write_byte_data(al3010_client, AL3010_HIGH_THD_LSB, g_lsb_thd[level+1]);
        lsb = i2c_smbus_read_byte_data(al3010_client, AL3010_HIGH_THD_LSB);
        

        i2c_smbus_write_byte_data(al3010_client, AL3010_HIGH_THD_MSB, g_msb_thd[level+1]);
        msb = i2c_smbus_read_byte_data(al3010_client, AL3010_HIGH_THD_MSB);

        printk(DBGMSK_PRX_G2"[als_P02]-- al3010_get_high_threshold_value: msb=%d, lsb=%d\n", msb, lsb);

        if(g_al3010_light != g_last_al3010_light) {
            g_last_al3010_light = g_al3010_light;
            als_lux_report_event(g_al3010_light);
        }
}

/*
static irqreturn_t als_interrupt_handler(int irq, void *dev_id)
{
        u16 adc_data = 0;
        u16 als_thd = 0;
        int range, lsb, msb;
        int i = 0;
        struct al3010_data *data = i2c_get_clientdata(al3010_client);
    //    struct i2c_client *client = al3010_client;
        printk("[al3010][als] als_interrupt_handler, irq:%d ++\n", irq);

        if (g_al3010_switch_on == 0) {
            printk("[al3010][als] als_interrupt_handler: not yet switch on\n");
            return IRQ_HANDLED;
        }

        mutex_lock(&data->lock);
        lsb = i2c_smbus_read_byte_data(al3010_client, AL3010_HIGH_THD_LSB);
        if (lsb < 0) {
                mutex_unlock(&data->lock);
                return IRQ_HANDLED;
        }

        msb = i2c_smbus_read_byte_data(al3010_client, AL3010_HIGH_THD_MSB);
        mutex_unlock(&data->lock);
        printk("[als_P01]al3010_get_threshold_value: msb=%d, lsb=%d\n", msb, lsb);
        if (msb < 0)
                return IRQ_HANDLED;

        als_thd = ((msb<<8) | lsb );

        mutex_lock(&data->lock);
        lsb = i2c_smbus_read_byte_data(al3010_client, AL3010_ADC_LSB);
        msb = i2c_smbus_read_byte_data(al3010_client, AL3010_ADC_MSB);
        mutex_unlock(&data->lock);

        adc_data = ( (msb << 8) | lsb );
        printk("[al3010][als] als_interrupt_handler adc:%d\n", adc_data);

        g_al3010_als_data = (adc_data > als_thd) ? adc_data : als_thd;

        range = al3010_get_range(al3010_client);
        g_al3010_als_data = (u32)(((msb << 8) | lsb) * range) >> 16;

        for(i=0 ; i<TOTALMAPS ; i++) {
                if( g_al3010_als_data < g_al3010_light_map[i] ) {
                        g_al3010_als_data = g_al3010_light_level[i];
                        break;
                }
        }

        if(i==TOTALMAPS) {
                g_al3010_als_data = g_al3010_light_level[13]; //12000 lux.
        }

        if(g_last_al3010_als_data != g_al3010_als_data) {
           g_last_al3010_als_data = g_al3010_als_data;
           printk("[al3010][als] als_interrupt_handler, state_change\n");
           printk("[al3010][als] al3010 lux = %d\n", g_al3010_als_data);
           input_report_abs(g_al3010_data_as->input_dev, ABS_MISC, g_al3010_als_data);
           input_event(g_al3010_data_as->input_dev, EV_SYN, SYN_REPORT, 1);
           input_sync(g_al3010_data_as->input_dev);
#ifndef INPUT_EVENT_MODE
           atomic_inc(&p02_ambient_update);
           printk("[al3010][als] als_interrupt_handler fire(%d)\n",atomic_read(&p02_ambient_update));
#endif
        }

        printk("[al3010][als] als_interrupt_handler --\n");
        return IRQ_HANDLED;
}

static int p02_als_report_event_init(void)
{
        int ret = 0;
        struct input_dev *input_dev_as = NULL;
        input_dev_as = input_allocate_device();

        if (!input_dev_as) {
            ret = -ENOMEM;
            printk("[al3010]: Failed to allocate input_data device\n");
            return ret;
        }

        input_dev_as->name = "al3010_als_report";
        input_dev_as->id.bustype = BUS_I2C;
        input_set_capability(input_dev_as, EV_ABS, ABS_MISC);
        __set_bit(EV_ABS, input_dev_as->evbit);
        __set_bit(ABS_MISC, input_dev_as->absbit);
        input_set_abs_params(input_dev_as, ABS_MISC, 0, 1048576, 0, 0);
        input_set_drvdata(input_dev_as, g_al3010_data_as);

        ret = input_register_device(input_dev_as);
        if (ret < 0) {
           input_free_device(input_dev_as);
           return ret;
        }  

        g_al3010_data_as->input_dev = input_dev_as;
        this_input_dev_p02_als = input_dev_as;

        return 0;
}
*/
// --Louis 20120215

/*
 * sysfs layer
 */

/* range */
static ssize_t al3010_show_range(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

    printk(DBGMSK_PRX_G2"[als_P01] al3010_show_range: %d\n", al3010_get_range(client));

	return sprintf(buf, "%i\n", al3010_get_range(client));
}

static ssize_t al3010_store_range(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

    printk(DBGMSK_PRX_G2"[als_P01] al3010_store_range\n");

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 3))
		return -EINVAL;

    printk(DBGMSK_PRX_G2"[als_P01] al3010_store_range: %lu\n", val);
	ret = al3010_set_range(client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(range, S_IWUSR | S_IRUGO,
		   al3010_show_range, al3010_store_range);


/* resolution */
static ssize_t al3010_show_resolution(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
    printk(DBGMSK_PRX_G2"[als_P01] al3010_show_resolution: %d\n", al3010_get_resolution(client));

	return sprintf(buf, "%d\n", al3010_get_resolution(client));
}

static ssize_t al3010_store_resolution(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

    printk(DBGMSK_PRX_G2"[als_P01] al3010_store_resolution\n");

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 3))
		return -EINVAL;

    printk(DBGMSK_PRX_G2"[als_P01] al3010_store_resolution: %lu\n", val);
	ret = al3010_set_resolution(client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(resolution, S_IWUSR | S_IRUGO,
		   al3010_show_resolution, al3010_store_resolution);

/* mode */
static ssize_t al3010_show_mode(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
    printk(DBGMSK_PRX_G2"[als_P01] al3010_show_mode: %d\n", al3010_get_mode(client));

	return sprintf(buf, "%d\n", al3010_get_mode(client));
}

static ssize_t al3010_store_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

    printk(DBGMSK_PRX_G2"[als_P01] al3010_store_mode\n");

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 2))
		return -EINVAL;

    printk(DBGMSK_PRX_G2"[als_P01] al3010_store_mode: %lu\n", val);
	ret = al3010_set_mode(client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(mode, S_IWUSR | S_IRUGO,
		   al3010_show_mode, al3010_store_mode);


/* power state */
static ssize_t al3010_show_power_state(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
    printk(DBGMSK_PRX_G2"[als_P01] al3010_show_power_state: %d\n", al3010_get_power_state(client));
	return sprintf(buf, "%d\n", al3010_get_power_state(client));
}

static ssize_t al3010_store_power_state(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

    printk(DBGMSK_PRX_G2"[als_P01] al3010_store_power_state\n");

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 1))
		return -EINVAL;

    printk(DBGMSK_PRX_G2"[als_P01] al3010_store_power_state: %lu\n", val);

	ret = al3010_set_power_state(client, val);
	return ret ? ret : count;
}

static DEVICE_ATTR(power_state, S_IWUSR | S_IRUGO,
		   al3010_show_power_state, al3010_store_power_state);


/* lux */
static ssize_t al3010_show_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

    printk(DBGMSK_PRX_G2"[als_P01] al3010_show_lux\n");

	/* No LUX data if not operational */
	if (al3010_get_power_state(client) != 0x01)
		return -EBUSY;

    printk(DBGMSK_PRX_G2"[als_P01] al3010_show_lux: %d\n", al3010_get_adc_value(client));
	return sprintf(buf, "%d\n", al3010_get_adc_value(client));
}

static DEVICE_ATTR(lux, S_IRUGO, al3010_show_lux, NULL);

static struct attribute *al3010_attributes[] = {
	&dev_attr_range.attr,
	&dev_attr_resolution.attr,
	&dev_attr_mode.attr,
	&dev_attr_power_state.attr,
	&dev_attr_lux.attr,
	NULL
};

static const struct attribute_group al3010_attr_group = {
    .name = "al3010",
	.attrs = al3010_attributes,
};

static int al3010_init_client(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	int i;

    printk(DBGMSK_PRX_G2"[als_P01]++al3010_init_client\n");

	/* read all the registers once to fill the cache.
	 * if one of the reads fails, we consider the init failed */
	for (i = 0; i < ARRAY_SIZE(data->reg_cache); i++) {
		int v = i2c_smbus_read_byte_data(client, al3010_reg[i]);
		if (v < 0)
			return -ENODEV;

		data->reg_cache[i] = v;
	}

	/* set defaults */
	al3010_set_range(client, 0);
	al3010_set_resolution(client, 0);
	al3010_set_mode(client, 0);
	al3010_set_power_state(client, 0);

/*
    //for test onley
    if(0 != al3010_set_range(client, 0)) {
        printk("[als_P01] al3010_init_client: set range failed\n");
    }

    if(0 != al3010_set_mode(client, 1)) {
        printk("[als_P01] al3010_init_client: set mode failed\n");
    }

    if(0 != al3010_set_power_state(client, 1)) {
        printk("[als_P01] al3010_init_client: set power state failed\n");
    }
*/
    printk(DBGMSK_PRX_G2"[als_P01]--al3010_init_client\n");

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void al3010_early_suspend(struct early_suspend *handler)
{
    printk(DBGMSK_PRX_G2"[als_P01] ++al3010_early_suspend, als:%d\n", g_al3010_switch_on);

    g_al3010_switch_earlysuspend = 1;
    g_polling_count = 0;

    if(1 == g_al3010_switch_on) {
        //In case upper layer doesn't switch off ambient before early_suspend.
        g_ambient_suspended = 1;

        printk(DBGMSK_PRX_G2"[als_P01] al3010_early_suspend, turn off ambient\n");

        set_als_power_state_of_P01(0);
    }

    printk(DBGMSK_PRX_G2"[als_P01] --al3010_early_suspend\n");
}


static void al3010_late_resume(struct early_suspend *handler)
{
    printk(DBGMSK_PRX_G2"[als_P01] ++al3010_late_resume, als:%d\n", g_al3010_switch_on);

    if(1 == g_ambient_suspended) {
        printk(DBGMSK_PRX_G2"[als_P01] al3010_late_resume, P01 attached: %d\n", g_bIsP01Attached);

        if(g_bIsP01Attached) {
           printk(DBGMSK_PRX_G2"[als_P01] al3010_late_resume, turn on ALS\n");
           set_als_power_state_of_P01(1);

            //[SCR] Fix bug that causes exception
            //printk(DBGMSK_PRX_G2"[als_P01] al3010_late_resume: start light_poll_work\n");
        }

        printk(DBGMSK_PRX_G2"[als_P01] al3010_late_resume, P01 not attached\n");
        g_ambient_suspended = 0;
    }

    g_al3010_switch_earlysuspend=0;

    printk(DBGMSK_PRX_G2"[als_P01]--al3010_late_resume\n");
}


static struct early_suspend al3010_early_suspend_desc = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = al3010_early_suspend,
    .resume = al3010_late_resume,
};
#endif

/*
 * I2C layer
 */

static int __devinit al3010_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct al3010_data *data;
	int err = 0;

    printk(DBGMSK_PRX_G2"[als_P01]++al3010_probe\n");

    data = kzalloc(sizeof(struct al3010_data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    // registered as switch device
    err = switch_dev_register(&ls_switch_dev);
    if (err < 0) {
        goto exit_kfree;
    }
   
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);

    //store i2c client structure
    al3010_client = client;

    printk(DBGMSK_PRX_G2"[als_P01]++al3010_probe: init_client\n");

	/* initialize the AL3010 chip */
	err = al3010_init_client(client);
	if (err)
		goto exit_kfree;

    printk(DBGMSK_PRX_G2"[als_P01]++al3010_probe: create_group\n");

	/* register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &al3010_attr_group);
	if (err)
		goto exit_kfree;

	dev_info(&client->dev, "driver version %s enabled\n", DRIVER_VERSION);

    /*
    // test code: polling thread for light sensor
    al3010light_workqueue = create_singlethread_workqueue("al3010lightwq");
    INIT_DELAYED_WORK(&g_light_work, al3010_poll_work);

    printk("[als_P01]Probe: start light poll work\n");
    schedule_delayed_work(&g_light_work, 10);
    */
/*
//++Louis
        if (!(g_al3010_data_as = kmalloc(sizeof(struct al3010_data), GFP_KERNEL))) {
           err = -ENOMEM;
           goto exit_kfree;
        }
        memset(g_al3010_data_as, 0, sizeof(struct al3010_data));

        i2c_set_clientdata(client, g_al3010_data_as);

        err = p02_als_report_event_init();
        if (!err) {
           printk("Unable to register input device\n");
           goto exit_kfree;
        }
//--Louis
*/
#ifdef CONFIG_HAS_EARLYSUSPEND
    register_early_suspend( &al3010_early_suspend_desc );
#endif

    g_AlsP01ProbeError = 0;


#ifdef CONFIG_I2C_STRESS_TEST
       i2c_add_test_case(client, "Sensor_Al3010",ARRAY_AND_SIZE(gSensorTestCaseInfo));
#endif


    printk(DBGMSK_PRX_G2"[als_P01]--al3010_probe\n");
	return 0;

exit_kfree:
    g_AlsP01ProbeError = err;
	kfree(data);
	return err;
}

static int __devexit al3010_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &al3010_attr_group);
	al3010_set_power_state(client, 0);
    switch_dev_unregister(&ls_switch_dev);
	kfree(i2c_get_clientdata(client));

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend( &al3010_early_suspend_desc );
#endif
	return 0;
}

#ifdef CONFIG_PM
static int al3010_suspend(struct i2c_client *client, pm_message_t mesg)
{
    return 0;
}

static int al3010_resume(struct i2c_client *client)
{
    return 0;
}

#else
#define al3010_suspend	NULL
#define al3010_resume		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id al3010_id[] = {
	{ "al3010", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, al3010_id);

static struct i2c_driver al3010_driver = {
	.driver = {
		.name	= AL3010_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = al3010_suspend,
	.resume	= al3010_resume,
	.probe	= al3010_probe,
	.remove	= __devexit_p(al3010_remove),
	.id_table = al3010_id,
};

//Disable P01 attached temporarily for 1st ICS check-in
static int lightsensor_p01_mp_event(struct notifier_block *this, unsigned long event, void *ptr);

static struct notifier_block lightsensor_p01_mp_notifier = {
       .notifier_call = lightsensor_p01_mp_event,
        .priority = AL3010_LIGHTSENSOR_MP_NOTIFY,
};

static int __init al3010_init(void)
{
    int err = 0;

    printk(DBGMSK_PRX_G2"[als_P01] al3010_init\n");
    al3010light_workqueue = create_singlethread_workqueue("al3010light_wq");
    INIT_WORK(&al3010_attached_P02_work, lightsensor_attached_pad_P01);
    INIT_WORK(&al3010_ISR_work, mp_als_interrupt_handler);

    if ( g_bIsP01Attached == true ) {
        err = i2c_add_driver(&al3010_driver);
        if(err) {
            printk(DBGMSK_PRX_G0"[als_P01] load al3010 driver failed\n");
        }
        else {
            printk(DBGMSK_PRX_G2"[als_P01] load al3010 driver failed\n");
        }
    }
/*
    if( g_al3010_device.irq <= 0 ) {
        printk("[al3010] gpio_to_irq fail (g_al3010_device.irq).\n");
    }
    else {
        printk("[al3010_attributes] (g_al3010_device.irq) irq=%d.\n", g_al3010_device.irq);

        err = request_irq(  g_al3010_device.irq,
                            als_interrupt_handler,
                            IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                            "al3010_INT",
                            &al3010_client->dev );
        if (err < 0) {
            printk("[al3010] (g_al3010_device.irq) request_irq() error %d.\n",err);
        }
        else {
            printk("[al3010] (g_al3010_device.irq) request_irq ok.\n");
        }
    }
*/
    //Disable P01 attached temporarily for 1st ICS check-in
	register_microp_notifier(&lightsensor_p01_mp_notifier);

	return err; 
}

static void __exit al3010_exit(void)
{
    destroy_workqueue(al3010light_workqueue);
	i2c_del_driver(&al3010_driver);
}

// Disable P01 attached temporarily for 1st ICS check-in
static void lightsensor_attached_pad_P01(struct work_struct *work)
{
//    u8 *id_data;
    int error;

    printk(DBGMSK_PRX_G2"[als_P01] lightsensor_attached_pad_P01()++\n");

    if ( (g_AlsP01ProbeError != 0) )
    {
        i2c_add_driver(&al3010_driver);
        printk(DBGMSK_PRX_G2"[als_P01] P01 Successfully added driver %s\n", al3010_driver.driver.name);

        if ( g_AlsP01ProbeError ) {
            printk(DBGMSK_PRX_G0"[als_P01] i2c_del_driver++g_AlsP01ProbeError=%d\n", g_AlsP01ProbeError);
            i2c_del_driver(&al3010_driver);
            printk(DBGMSK_PRX_G2"[als_P01]i2c_del_driver--\n");

            //[SCR] Add error handling code if i2c attaching failed
            printk("[als_P01] al3010 attach fails, i2c_del_driver\n");

            //report uevent if prob error
            printk("[als_P01] al3010 prob error, report uevent to framework\n");
            switch_set_state(&ls_switch_dev, P01_EVENT_NOTIFY_LIGHTSENSOR_ERROR);
            return;
        }
        printk(DBGMSK_PRX_G2"[als_P01] lightsensor_attached_pad_P01(), driver already added\n");
    }
    g_bIsP01Attached = true;
    g_polling_count = 0;

    error = set_als_power_state_of_P01(0);

    if (error != 0) {
        switch_set_state(&ls_switch_dev, P01_EVENT_NOTIFY_LIGHTSENSOR_ERROR);
        return;
    }
    else
        switch_set_state(&ls_switch_dev, P01_EVENT_NOTIFY_LIGHTSENSOR_NO_ERROR);

    if (g_HAL_als_switch_on) {
        //reset
        i2c_smbus_write_byte_data(al3010_client, AL3010_MODE_COMMAND, AL3010_POW_RESET);
        msleep(15);
        set_als_power_state_of_P01(1);
    }

    printk(DBGMSK_PRX_G2"[als_P01] lightsensor_attached_pad_P01()--\n");

    return;
}
EXPORT_SYMBOL(lightsensor_attached_pad_P01);

int lightsensor_detached_pad_P01(void)
{
    printk(DBGMSK_PRX_G2"[als_P01] lightsensor_detached_pad_P01()++\n");

    //turn al3010 off
    if(g_al3010_switch_on) {
        printk(DBGMSK_PRX_G2"[als_P01] lightsensor_detached_pad_P01(), turn off al3010\n");
        set_als_power_state_of_P01(0);
    }

    g_bIsP01Attached = false;

    printk(DBGMSK_PRX_G2"[als_P01] lightsensor_detached_pad_P01()--\n");

    return 0;
}
EXPORT_SYMBOL(lightsensor_detached_pad_P01);

static int lightsensor_p01_mp_event(struct notifier_block *this, unsigned long event, void *ptr)
{

    switch (event) {
        case P01_ADD:
                printk(DBGMSK_PRX_G2"[als_P01][MicroP] P01_ADD \r\n");
                //lightsensor_attached_pad_P01();
                queue_work(al3010light_workqueue, &al3010_attached_P02_work);
                return NOTIFY_DONE;

        case P01_REMOVE:
                printk(DBGMSK_PRX_G2"[als_P01][MicroP] P01_REMOVE \r\n");
                lightsensor_detached_pad_P01();
                return NOTIFY_DONE;

        case P01_LIGHT_SENSOR:
                printk(DBGMSK_PRX_G2"[als_P01][MicroP] P01_ISR \r\n");
                queue_work(al3010light_workqueue ,&al3010_ISR_work);
                //mp_als_interrupt_handler(al3010_client);
                return NOTIFY_DONE;
        default:
                return NOTIFY_DONE;
    }
}



MODULE_AUTHOR("ASUS");
MODULE_DESCRIPTION("test version v1.0");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(al3010_init);
module_exit(al3010_exit);

