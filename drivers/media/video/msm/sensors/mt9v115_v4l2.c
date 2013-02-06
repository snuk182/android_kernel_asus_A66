/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#include "msm.h"    //ASUS_BSP Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
#include "mt9v115_v4l2.h"
#include <linux/asusdebug.h> //ASUS_BSP LiJen "[A66][8M][NA][Others]add camera power event logs"

#define SENSOR_NAME "mt9v115"
#include <linux/regulator/consumer.h>
#include <linux/a60k_gpio_pinname.h>
#include "msm_ispif.h"	//ASUS_BSP LiJen "[A60K][8M][NA][Fix] add NOTIFY_ISPIF_STREAM v4l2 notify in kernel
struct mt9v115_regs {
	const unsigned short addr;
	unsigned short value;
	const unsigned char lens;
};
 
DEFINE_MUTEX(mt9v115_mut);
static struct msm_sensor_ctrl_t mt9v115_s_ctrl;

static struct v4l2_subdev_info mt9v115_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_YUYV8_2X8,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
};

static struct msm_camera_csid_vc_cfg mt9v115_cid_cfg[] = {
	{0, 0x1E, CSI_DECODE_8BIT},
	{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
};

static struct msm_camera_csi2_params mt9v115_csi_params = {
	.csid_params = {
		.lane_assign = 0xe4,
		.lane_cnt = 1,
		.lut_params = {
			.num_cid = 2,
			.vc_cfg = mt9v115_cid_cfg,
		},
	},
	.csiphy_params = {
		.lane_cnt = 1,
		.settle_cnt = 0x14, //0x1B
	},
};

static struct msm_camera_csi2_params *mt9v115_csi_params_array[] = {
	&mt9v115_csi_params,
	&mt9v115_csi_params,
	&mt9v115_csi_params,
};

static int32_t mt9v115_i2c_read(const unsigned short raddr,	unsigned short *rdata,const unsigned short rlen, const unsigned short  iBurstCount)
{
return msm_camera_i2c_read(mt9v115_s_ctrl.sensor_i2c_client, raddr, rdata, rlen);
}

static int32_t mt9v115_i2c_write(const unsigned short waddr,unsigned short* pdata,const unsigned short sValueWidth, const unsigned short  iBurstCount)
{
return msm_camera_i2c_write(mt9v115_s_ctrl.sensor_i2c_client, waddr, *pdata, sValueWidth);
}

//#define DEBUG_I2C_RESULT

static int32_t mt9v115_i2c_loop_array( 
    int32_t (*op)(const unsigned short,unsigned short*,const unsigned short, const unsigned short),
	struct mt9v115_regs* regs, uint16_t iSize) {
	uint16_t i=0;
#ifdef DEBUG_I2C_RESULT
			pr_err("MT9V115 [0x%X]=0x%X, len=%d, Sizes=%d", regs[i].addr, regs[i].value, regs[i].lens, iSize);
#endif			
	for (i=0;i<iSize;i++) {
		int32_t rc = 0;
		if (regs[i].addr == 0xffff) {
			msleep(regs[i].value);
			continue;
		}
		rc = (*op)(regs[i].addr, &regs[i].value, regs[i].lens, 1);
#ifdef DEBUG_I2C_RESULT
     if (iSize<10) {
		pr_err("MT9V115 [0x%X]=0x%X, len=%d", regs[i].addr, regs[i].value, regs[i].lens);
     		}
#endif			
		if (rc < 0) return rc;
	}
	return 0;
}
static int32_t mt9v115_i2c_write_array(
	struct mt9v115_regs* regs, uint16_t iSize) {
#ifdef DEBUG_I2C_RESULT
pr_err("MT9V115 write register begin:\n");
#endif			
	return mt9v115_i2c_loop_array(&mt9v115_i2c_write, regs, iSize);
}

static int32_t mt9v115_i2c_read_array(
	struct mt9v115_regs* regs, uint16_t iSize) {
#ifdef DEBUG_I2C_RESULT
	pr_err("MT9V115 read register begin:\n");
#endif			
	return mt9v115_i2c_loop_array(&mt9v115_i2c_read, regs, iSize);

}

int mt9v115_sensor_write_setting(
const void* sensorSetting,const uint16_t iSettingCount) {
	int   rc = 0;
	struct mt9v115_regs* mt9v115_init_settings=(struct mt9v115_regs*)sensorSetting;
	rc = mt9v115_i2c_write_array(mt9v115_init_settings, iSettingCount); //dummy
	if (rc <0) {
		pr_err("mt9v115_sensor_write_setting fail");
		return rc;
	}
		pr_info("mt9v115_sensor_write_setting ok\n");
		return rc;
}


static int32_t mt9v115_set_status_to_reset(void) {
	int32_t rc = 0;
	unsigned short DefaultValue=0x0106;
			pr_err("MT9V115 writing mt9v115_init_settings to 0x001A...");
			msleep(10);
		rc = mt9v115_i2c_write( 0x001A, &DefaultValue, 2, 1);
			if (rc <0) {
				pr_err("(Ignore this I2C NACK, since VGA sensor is reseting)\n");
				msleep(10);
	//			goto LOOP_001A;
	//			return rc;
			}
	//		pr_err("OK\n");
	return 0;
}

static bool mt9v115_wait_to_leave_reset(void) {
	int32_t rc = 0;
	unsigned short DefaultValue=0xFFFF;
	unsigned char i=0;
  pr_err("Polling for 0x001A (sensor to normal state)...");
	 for (i=0;i<10;i++) {
		 rc = mt9v115_i2c_read(0x001A, &DefaultValue, 2, 0);
		 if (rc >=0) {
		 //  const char sOutStandbyMsg[] = {"MT9V115 OK to %s standby mode, 0x0018=0x%X"};
			 if ((DefaultValue & 0x2)==0 ) {
				 //enter standby, check until 0x0018[14] == 1
				 pr_err("MT9V115 OK to leave reset mode, 0x001A=0x%X", DefaultValue); 	 
				 break;
			 } else {
				 pr_err("MT9V115 FAIL to leave reset mode, 0x001A=0x%X... Timer=%d", DefaultValue, i); 	
				 rc = -EFAULT;
			 }
		 } else {
			 pr_err("MT9V115 Read 0x001A fail"); 	 
		 }
		 msleep(5);
	 }	 
	 return rc;
}

static int32_t mt9v115_init_setting(
	const void* sensorSetting,const uint16_t iSettingCount) {
	int32_t rc = 0;
	unsigned short DefaultValue=0xFFFF;
	struct mt9v115_regs* mt9v115_init_settings=(struct mt9v115_regs*)sensorSetting;

//LOOP_001A:	
	rc = mt9v115_set_status_to_reset();
	if (rc<0) return rc;
	
	rc = mt9v115_wait_to_leave_reset();
	if (rc<0) return rc;
 	pr_info("mt9v115_regs[0].reg=0x%X, count=%d", mt9v115_init_settings[0].addr, iSettingCount);
	rc = mt9v115_i2c_write_array(mt9v115_init_settings, iSettingCount); //dummy
	if (rc <0) {
		pr_err("MT9V115 write mt9v115_init_settings array fail");
		return rc;
	}

DefaultValue = 0x003C;
if ( mt9v115_i2c_write(0x301A, &DefaultValue, 2, 1) >=0) {
	//stream on
	return 0;
}

	return 0;
}


//ASUS_BSP+++ CR_0000 Randy_Change@asus.com.tw [2011/8/9] Modify Begin
static int MT9V115_enable_regulator(struct device *dev, const char* regulator_Name, bool on, int regulator_level) {
	int rc;
	struct regulator *reg_8921=NULL;
	reg_8921 = regulator_get(dev, regulator_Name);
//	regulator_get(&mt9v115_s_ctrl.sensor_i2c_client->client->dev, "8921_l8"); //ASUS_BSP Patrick "[A60K][8M][NA][Others] Modify GPIO parameters"
	if (IS_ERR(reg_8921)) {
		pr_info("PTR_ERR(%s)=%ld\n", regulator_Name, PTR_ERR(reg_8921)); 
		return -ENODEV;
	}
	
	if (on) {
		pr_info("Turn on the regulators:%s\n", regulator_Name);
		if (regulator_level) {
			rc = regulator_set_voltage(reg_8921, 2800000, 2800000);
			if (!rc) {
//				pr_info("%s regulator_set_voltage, !rc is true, rc=%d--\n",regulator_Name, rc);
			} else {
				return -ENODEV;
			}
		}

		rc = regulator_enable(reg_8921);
		if (rc) {
			pr_info("%s regulator enable failed, rc=%d--\n", regulator_Name, rc);
			return rc;
		}
		pr_err("Turn on %s(%d) success\n", regulator_Name, regulator_get_voltage(reg_8921));
	} else {					 //(on == false) /* Turn off the regulators */
		regulator_disable(reg_8921);
		pr_err("Turn off the regulators:%s\n", regulator_Name);	
	}
	regulator_put(reg_8921);
	return 0;	
}

static int mt9v115_regulator_init(struct i2c_client *client, bool on)
{ 
	static bool prev_on=false;
	pr_info("%s +++, on=%d\n",__func__, on);
	if (on == prev_on) {
		pr_info("MT9V115 regulator already same, skip switch regulator on=%d, pre_on=%d\n",on,prev_on); 
		return 0;
	}
	prev_on = on;

	MT9V115_enable_regulator(&client->dev, "8921_l8", on, 2800000); //close after power off
//	MT9V115_enable_regulator(&client->dev, "8921_l16", on, 2800000);
	if (A60K_EVB==g_A60K_hwID) {
		MT9V115_enable_regulator(&client->dev, "8921_lvs6", on, 0);  
	}
	msleep(100);
	return 0;
}

void readingRegs(const unsigned char* sRegs, unsigned short reg) {
	unsigned short regreturned = 0;
		 if (mt9v115_i2c_read(reg, &regreturned, 2, 0) <0) {
			pr_err("Read %s fail\n", sRegs); 
			} else
	 pr_err("[%s]=0x%X\n", sRegs, regreturned);

}

void mt9v115_sensor_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	pr_info("%s +++ \n",__func__);

	pr_info("%s --- \n",__func__);
}

void mt9v115_sensor_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	pr_info("%s +++ \n",__func__);

	pr_info("%s --- \n",__func__);
}

static int32_t mt9v115_sensor_setting(struct msm_sensor_ctrl_t *s_ctrl,
	const void* sensorSetting,const uint16_t iSettingCount,
				int update_type)
{
	int32_t rc = 0;
	CDBG("%s +++\n",__func__);
	
	if (update_type == MSM_SENSOR_REG_INIT) {
		CDBG("%s MSM_SENSOR_REG_INIT\n",__func__);	
		s_ctrl->config_csi_flag = 1;
		s_ctrl->curr_csi_params = NULL;
	} else if (update_type == MSM_SENSOR_UPDATE_PERIODIC) {
	pr_info("%s MSM_SENSOR_UPDATE_PERIODIC\n",__func__);

       if (s_ctrl->config_csi_flag) {
		if (s_ctrl->curr_csi_params != s_ctrl->csi_params[0]) {
			pr_info("config to csi +++\n");

			//v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				//NOTIFY_ISPIF_STREAM, (void *)ISPIF_STREAM(PIX_0, ISPIF_OFF_IMMEDIATELY));

			mt9v115_set_status_to_reset();
			
			s_ctrl->curr_csi_params = s_ctrl->csi_params[0];
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,
				NOTIFY_CSID_CFG,	&s_ctrl->curr_csi_params->csid_params);
			//v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,NOTIFY_CID_CHANGE, NULL);
			mb();
			v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,NOTIFY_CSIPHY_CFG,&s_ctrl->curr_csi_params->csiphy_params);
			mb();
			msleep(20);
			s_ctrl->config_csi_flag = 0;
			pr_info("config to csi ---\n");
		}
{
	uint32_t op_pixel_clk = 72000000;				
		v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev,NOTIFY_PCLK_CHANGE, &op_pixel_clk);
}
		//v4l2_subdev_notify(&s_ctrl->sensor_v4l2_subdev, NOTIFY_ISPIF_STREAM, (void *)ISPIF_STREAM(	PIX_0, ISPIF_ON_FRAME_BOUNDARY));

				rc = mt9v115_init_setting(sensorSetting, iSettingCount);
					if (rc < 0) {
						pr_err("MT9V115 leave standby mode fail\n");
						return rc;
					}
					CDBG("mt9v115_sensor_setting UPDATE_PERIODIC ok\n");
	
					readingRegs("0x0042", 0x0042);
					readingRegs("0x3C00", 0x3C00);
					readingRegs("0x001A", 0x001A);
					readingRegs("0x0018", 0x0018);
					readingRegs("0x3C40", 0x3C40);
					readingRegs("0x301A", 0x301A);
					readingRegs("0x3C42", 0x3C42);
	{
		unsigned short regreturned = 0;
		 if (mt9v115_i2c_read(0x0018, &regreturned, 2, 0) <0 || regreturned != 2) {
			pr_err("Sensor status is incorrect!! 0x0018 Read fail, Reg[0x0018]=0x%X\n", regreturned); 
			rc = -EINVAL; 		
		 }		
	}

       	}
	}
	CDBG("%s ---\n",__func__);
	return rc;
}

static int mt9v115_sensor_config(struct msm_sensor_ctrl_t *s_ctrl, void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,(void *)argp, sizeof(struct sensor_cfg_data))) return -EFAULT;
	mutex_lock(&mt9v115_mut);
	CDBG("mt9v115_sensor_config: cfgtype = %d\n",cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_SET_MODE:
		rc = mt9v115_sensor_setting(s_ctrl, cdata.sensorSetting, cdata.iSettingCount, MSM_SENSOR_UPDATE_PERIODIC);
		break;
	case CFG_SENSOR_INIT:
		rc = mt9v115_sensor_setting(s_ctrl, cdata.sensorSetting, cdata.iSettingCount, MSM_SENSOR_REG_INIT);
		break;
	case CFG_SET_ISP_EFFECT_MODE:
		rc = mt9v115_sensor_write_setting(cdata.sensorSetting, cdata.iSettingCount);
		break;
	case CFG_REGISTER_TO_REAL_GAIN:
		 if (mt9v115_i2c_read_array((struct mt9v115_regs* )cdata.sensorSetting, cdata.iSettingCount) <0) {
			rc = -EINVAL; 		
			break;
		 }		
		break;
	case CFG_START_STREAM:  
		if (s_ctrl->func_tbl->sensor_start_stream == NULL) {
			rc = -EFAULT;
			break;
		}
		s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
		break;
	case CFG_STOP_STREAM:
		if (s_ctrl->func_tbl->sensor_stop_stream == NULL) {
			rc = -EFAULT;
			break;
		}
		s_ctrl->func_tbl->sensor_stop_stream(s_ctrl);
		break;
	case CFG_GET_CSI_PARAMS:
			if (s_ctrl->func_tbl->sensor_get_csi_params == NULL) {
				rc = -EFAULT;
				break;
			}
			rc = s_ctrl->func_tbl->sensor_get_csi_params(
				s_ctrl,
				&cdata.cfg.csi_lane_params);

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;
	default:
		pr_info("mt9v115_sensor_config: cfgtype = %d is not supported!!\n",cdata.cfgtype);
		rc = -EFAULT;
		break;
	}
	mutex_unlock(&mt9v115_mut);

	return rc;
}

static int mt9v115_probe_read_sensor_id(void)
{
	int32_t rc = 0;
	unsigned short chipidl;
	unsigned short chipidh;

	CDBG("%s: %d\n", __func__, __LINE__);
	CDBG("mt9v115_probe_init_sensor is called\n");
//	SetupUpCameraSensorPin(true);
	/* 3. Read sensor Model ID: */
	rc = mt9v115_i2c_read(0x0000, &chipidh, 2, 0);
	if (rc < 0) {
		pr_err("chep ID read failed!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
//		msleep(100000);
		goto init_probe_fail;
	}
	rc = mt9v115_i2c_read(0x001A, &chipidl, 2, 0);
	if (rc < 0) {
		printk("Model read failed\n");
		goto init_probe_fail;
	}
	pr_info("mt9v115 model_id = 0x%x  0x%x ^^^^^^^^^^^^^^^^^\n", chipidh, chipidl);
	/* 4. Compare sensor ID to mt9v115 ID: */
	if (chipidh != 0x2284) {
		rc = -ENODEV;
		pr_err("mt9v115_probe_init_sensor fail chip id doesnot match!!!!!!!!!!!!!!!!\n");
		goto init_probe_fail;
	}
	
	goto init_probe_done;
init_probe_fail:
	pr_err("mt9v115_probe_init_sensor fails\n");
init_probe_done:
	CDBG(" mt9v115_probe_init_sensor finishes\n");
	return rc;
	}
//ASUS_BSP +++ Stimber "Implement i2c stress test method"
#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>

static int i2c_test_VGA_Camera_camera(struct i2c_client *apClient)
{

	int lnResult = I2C_TEST_PASS;
    
	i2c_log_in_test_case("i2c_test_VGA_Camera_camera++\n");
	if (mt9v115_probe_read_sensor_id() < 0) {
        	i2c_log_in_test_case("i2c_test_VGA_Camera_camera failed\n");        
		lnResult = -1;
	}
    
	i2c_log_in_test_case("i2c_test_VGA_Camera_camera--\n");
	return lnResult;
};

static struct i2c_test_case_info gVGACameraTestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(i2c_test_VGA_Camera_camera),
};

#endif
//ASUS_BSP --- Stimber "Implement i2c stress test method"

static int SetupUpCameraSensorPin( bool bOn) { 
	int rc = -EBADF, i=0;
  struct tGPIOStyle {const unsigned char num;const bool lvl;} pinDef[][3] = {
			{{ 9, 1}, {1, 0}, {25, 0}},
			{{56,1}, {1, 0}, {72, 0}},
  };
  unsigned char HWID=1;
switch (g_A60K_hwID) {
	case A60K_EVB:
	case A60K_SR1_1:   		//  Hann Start Panel & Sharp Panel
	case A60K_SR1_2_ES1:   	// M8960 ES1
	case A60K_SR1_2_ES2:	// Non - Used
	case A60K_ER1:			// Non - Used
	case A60K_ER2:			// Non - Used	
	case A60K_PR:			// Non - Used
		HWID=0;
		break;
	default:
		break;
}

  if (bOn) {
		mt9v115_regulator_init(mt9v115_s_ctrl.sensor_i2c_client->client, true);
		for (i=0;i<ARRAY_SIZE(pinDef[HWID]);i++) {
			rc = gpio_request(pinDef[HWID][i].num, SENSOR_NAME);
			if (!rc) {
				CDBG("MT9V115 gpio[%d]=%d, status = %d\n", pinDef[HWID][i].num, pinDef[HWID][i].lvl, rc);
				gpio_direction_output(pinDef[HWID][i].num, pinDef[HWID][i].lvl);
			}else {
				pr_err("gpio %d power fail\n", pinDef[HWID][i].num);
				goto init_probe_fail;
			}			
			msleep(1);
		}
  	} else {
		for (i=ARRAY_SIZE(pinDef[HWID])-1;i>=0;i--) {
			gpio_direction_output(pinDef[HWID][i].num, !pinDef[HWID][i].lvl);
			gpio_free(pinDef[HWID][i].num);
			CDBG("MT9V115 free gpio[%d]\n", pinDef[HWID][i].num);
			msleep(1);
		}
		mt9v115_regulator_init(mt9v115_s_ctrl.sensor_i2c_client->client, false);
  }

	return 1;
init_probe_fail:
	return -EBADF;
}
//ASUS_BSP--- CR_0000 Randy_Change@asus.com.tw [2011/8/9] Modify End

int32_t mt9v115_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl) {
    int32_t rc =0;
    
//    rc = mt9v115_power_up(s_ctrl->sensordata);
		CDBG("%s +++, i2c id = 0x%X\n",__func__, s_ctrl->sensor_i2c_client->client->addr);		
		if(!s_ctrl->sensordata) {
			CDBG("data is NULL, return\n");
			pr_info("%s ---\n",__func__);
			return -1;
		}
		
		//Turn on 24M CLK 	
//		msm_sensor_probe_on(&s_ctrl->sensor_i2c_client->client->dev);
		rc = msm_sensor_power_up(s_ctrl);
		if (rc < 0) {
			pr_err("%s: msm_sensor_power_up failed\n", __func__);
			ASUSEvtlog("[BAT][VGA]Report Capacity: EVTLOG_CAMERA_InitFail\n"); //ASUS_BSP LiJen "[A66][8M][NA][Others]add camera power event logs" 
			return rc;
		}

	//	msm_camio_clk_rate_set(MSM_SENSOR_MCLK_24HZ);
		SetupUpCameraSensorPin(true);

//		mt9v115_regulator_init(mt9v115_s_ctrl.sensor_i2c_client->client, true);
		CDBG("%s ---, i2c id = 0x%X\n",__func__, s_ctrl->sensor_i2c_client->client->addr);
		ASUSEvtlog("[BAT][VGA]Report Capacity: EVTLOG_CAMERA_ON\n"); //ASUS_BSP LiJen "[A66][8M][NA][Others]add camera power event logs"

    return rc;
}

int32_t mt9v115_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl) {
    int32_t rc =0;
	CDBG("%s +++\n",__func__);

	if(!s_ctrl->sensordata)
	{
		CDBG("data is NULL, return\n");
		pr_err("%s ---\n",__func__);
		return -1;
	}
	SetupUpCameraSensorPin(false);
//Turn off 24M CLK	
//	msm_sensor_probe_off(&mt9v115_s_ctrl.sensor_i2c_client->client->dev);
msm_sensor_power_down(s_ctrl);
ASUSEvtlog("[BAT][VGA]Report Capacity: EVTLOG_CAMERA_OFF\n"); //ASUS_BSP LiJen "[A66][8M][NA][Others]add camera power event logs"				
CDBG("%s ---\n",__func__);
    return rc;
}

int32_t mt9v115_sensor_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl;
	CDBG("%s randy +++ \n",__func__);
	CDBG("%s_i2c_probe called\n", client->name);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		rc = -EFAULT;
       pr_err("%s error--- \n",__func__);
		return rc;
	}

	s_ctrl = (struct msm_sensor_ctrl_t *)(id->driver_data);
	if (s_ctrl->sensor_i2c_client != NULL) {
		s_ctrl->sensor_i2c_client->client = client;
    } else {
		rc = -EFAULT;
       pr_err("%s error --- \n",__func__);
		return rc;
	}

	s_ctrl->sensordata = client->dev.platform_data;
//       mt9v115_s_ctrl.sensordata = client->dev.platform_data;
//  s_ctrl->sensor_i2c_client->client->dev->platform_data
	rc = mt9v115_sensor_power_up(s_ctrl);
	if (rc < 0)  goto probe_fail;
	
	rc = mt9v115_probe_read_sensor_id();
	if (rc < 0) goto probe_fail;

//ASUS_BSP +++ Stimber "Implement i2c stress test method"
#ifdef CONFIG_I2C_STRESS_TEST
		printk("[VGA Camera] VGA_Camera add test case+\n");			
		i2c_add_test_case(mt9v115_s_ctrl.sensor_i2c_client->client , SENSOR_NAME, ARRAY_AND_SIZE(gVGACameraTestCaseInfo));		
		printk("VGA Camera] VGA_Camera add test case-\n");
#endif
//ASUS_BSP --- Stimber "Implement i2c stress test method"
	
		rc = s_ctrl->func_tbl->sensor_power_down(&mt9v115_s_ctrl);
		if (rc < 0)
			goto probe_fail;
        
	snprintf(s_ctrl->sensor_v4l2_subdev.name,
		sizeof(s_ctrl->sensor_v4l2_subdev.name), "%s", id->name);
	v4l2_i2c_subdev_init(&s_ctrl->sensor_v4l2_subdev, client,
		s_ctrl->sensor_v4l2_subdev_ops);

	msm_sensor_register(&s_ctrl->sensor_v4l2_subdev);
	goto power_down;
probe_fail:
	pr_err("%s_i2c_probe failed\n", client->name);
power_down:
   CDBG("%s --- \n",__func__);
	return rc;
}

static const struct i2c_device_id mt9v115_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t)&mt9v115_s_ctrl},
	{ }
};

static struct i2c_driver mt9v115_i2c_driver = {
	.id_table = mt9v115_i2c_id,
	.probe  = mt9v115_sensor_i2c_probe,
	.driver = {
		.name = SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client mt9v115_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int __init msm_sensor_init_module(void)
{
	return i2c_add_driver(&mt9v115_i2c_driver);
}

static struct v4l2_subdev_core_ops mt9v115_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops mt9v115_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops mt9v115_subdev_ops = {
	.core = &mt9v115_subdev_core_ops,
	.video  = &mt9v115_subdev_video_ops,
};

int32_t mt9v115_sensor_setting_from_msm_sensor(struct msm_sensor_ctrl_t * s_sctrl,
     int update_type, int rt){
     int32_t rc = 0;
     if (update_type == MSM_SENSOR_REG_INIT) {
		 	rc = mt9v115_sensor_setting(s_sctrl, NULL, 0, update_type);
     } else {
			//call update, monitor
     }     
	 return rc;
}

void NULL_Implement(struct msm_sensor_ctrl_t *s_ctrl)
{
	pr_info("%s +++ \n",__func__);

	pr_info("%s --- \n",__func__);
}


static struct msm_sensor_fn_t mt9v115_func_tbl = {
#if 0
	.sensor_group_hold_on = NULL,
	.sensor_group_hold_off = NULL,
	.sensor_set_fps = NULL,
	.sensor_write_exp_gain = NULL,
	.sensor_write_snapshot_exp_gain = NULL,
	.sensor_set_isp_awblock_mode =  NULL, //ASUS_BSP LiJen "[A60K][8M][NA][Others]implement AWB Lock mode in 8M camera with ISP"
	.sensor_get_output_info = NULL,
#endif
	.sensor_get_csi_params = msm_sensor_get_csi_params,//add by sam for qc patch
	.sensor_setting = mt9v115_sensor_setting_from_msm_sensor,
	.sensor_config = mt9v115_sensor_config,
	.sensor_power_up = mt9v115_sensor_power_up,
	.sensor_power_down = mt9v115_sensor_power_down,
	.sensor_start_stream = mt9v115_sensor_start_stream,//NULL_Implement
	.sensor_stop_stream = mt9v115_sensor_stop_stream,//NULL_Implement
};

static struct msm_sensor_ctrl_t mt9v115_s_ctrl = {
	.sensor_i2c_client = &mt9v115_sensor_i2c_client,
#if 0
	.sensor_output_reg_addr = NULL,
	.sensor_exp_gain_info = NULL,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
#endif	
	.csi_params = &mt9v115_csi_params_array[0],
	.msm_sensor_mutex = &mt9v115_mut,
	.sensor_i2c_driver = &mt9v115_i2c_driver,
	.sensor_v4l2_subdev_info = mt9v115_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(mt9v115_subdev_info),
	.sensor_v4l2_subdev_ops = &mt9v115_subdev_ops,
	.func_tbl = &mt9v115_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

module_init(msm_sensor_init_module);
MODULE_DESCRIPTION("Omnivision 2MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");
