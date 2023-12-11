//--------------------------------------------------------------------
//                     ASUSTek Computer Inc.
//         Copyright (c) 2012 ASUSTek Computer inc, Taipei.
//
//			Fjm6mo ISP Device
//--------------------------------------------------------------------
//File: fjm6mo.h
//Revision History:
//[2012.01.09]	LiJen_Chang created.

#ifndef FJM6MO_H
#define FJM6MO_H

#include "msm_sensor.h"

// ASUS_BSP +++ LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 
#define WITH_WQ 1

typedef enum {
#ifdef WITH_WQ		
    E_M6MO_Status_Init,
#endif    
    E_M6MO_Status_Parameter,            /**< Parameter mode */
    E_M6MO_Status_Monitor,              /**< Monitor mode */
    E_M6MO_Status_Capture,              /**< Capture mode */
} E_M6MO_Status;

#ifdef WITH_WQ
struct fjm6mo_work {
	struct work_struct work;
	E_M6MO_Status state;
};

void fjm6mo_create_workqueue(void);
void fjm6mo_destroy_workqueue(void);
void fjm6mo_flush_workqueue(void);
#endif
// ASUS_BSP --- LiJen "[A66][8M][NA][Others] add workqueue to change ISP mode" 

void fjm6mo_start_AF(bool on, isp3a_af_mode_t mode, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w);
uint16_t fjm6mo_get_AF_result(struct msm_sensor_ctrl_t *s_ctrl);
void fjm6mo_set_led_mode(led_mode_t mode);
void fjm6mo_set_effect_mode(int16_t mode);
void fjm6mo_set_wb_mode(config3a_wb_t mode);
void fjm6mo_set_ev_mode(int16_t mode);
void fjm6mo_set_scene_mode(camera_bestshot_mode_type mode);
void fjm6mo_set_caf_mode(int16_t mode);
void fjm6mo_set_acelock_mode(int16_t mode);
void fjm6mo_set_awblock_mode(int16_t mode);
void fjm6mo_set_iso_mode(int16_t mode);
void fjm6mo_set_flicker_mode(int16_t mode);
void fjm6mo_get_exif(struct exif_cfg *exif);		//ASUS_BSP Stimber "[A60K][8M][NA][Other] Implement EXIF info for 8M camera with ISP"

int fjm6mo_sensor_open(void);
int fjm6mo_sensor_release(void);
int fjm6mo_i2c_debuginit(void);   
int sensor_change_status(E_M6MO_Status status);
int fw_version(void);
//ASUS_BSP +++ LiJen "[A66][8M][NA][Others]add proc file fo AP ISP update"
void create_fjm6mo_proc_file(void);
void remove_fjm6mo_proc_file(void);
//ASUS_BSP --- LiJen "[A66][8M][NA][Others]add proc file fo AP ISP update"

#endif  // end of FJM6MO_H
