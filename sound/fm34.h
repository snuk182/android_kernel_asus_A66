//--------------------------------------------------------------------
//                     ASUSTek Computer Inc.
//         Copyright (c) 2010 ASUSTek Computer inc, Taipei.
//
//			FM34 Voice processor
//--------------------------------------------------------------------
//File: fm34.h
//Revision History:


#ifndef __MACH_FM34_H
#define __MACH_FM34_H

#define Reg_Parser_Sync_Flag 	0x22FB
#define Reg_SW_Bypass_Ctrl 	0x22F5

#define CMD_MEM_WRITE		0x3B	// follow by 2 bytes address, 2 bytes data, total 5 bytes
#define CMD_MEM_READ		0x37	// follow by 2 bytes address, 0 bytes data, total 3 bytes
#define CMD_REG_WRITE		0x6A	// follow by 1 bytes address, 2 bytes data, total 4 bytes
#define CMD_REG_READ		0x60	// follow by 1 bytes address, 0 bytes data, total 2 bytes
#define CMD_PORT_WRITE      0x68    // 
#define CMD_PATCH_WRITE     0x0D    //  

#define LEN_SYNC_BYTE		2
#define SYNC_BYTE1		0xFC
#define SYNC_BYTE2		0xF3

#define LEN_MEM_WRITE		5
#define LEN_MEM_READ		3
#define LEN_REG_WRITE		4
#define LEN_REG_READ		2

#define LEN_PORT_WRITE      3
#define LEN_PATCH_WRITE      6

#define FM34_MIN_MEM_ADDR	0x2200	// check the spec, all reg add are larger than 0x2200
#define FM34_MAX_MEM_ADDR	0x2400	// check the spec, all reg add are smaller than 0x2400

#define MIC_PGAGAIN_0      0x00
#define MIC_PGAGAIN_M12    0x10
#define MIC_PGAGAIN_M24    0x20
#define MIC_PGAGAIN_M36    0x30
#define MIC_PGAGAIN_M48    0x40
#define MIC_PGAGAIN_M60    0x50
#define MIC_PGAGAIN_M72    0x60
#define MIC_PGAGAIN_M84    0x70
#define MIC_PGAGAIN_M96    0x80
#define MIC_PGAGAIN_M108   0x90
#define MIC_PGAGAIN_M120   0xa0
#define MIC_PGAGAIN_M132   0xb0
#define MIC_PGAGAIN_24     0xe0
#define MIC_PGAGAIN_12     0xf0

#define MIC_PGAGAIN_M0p75  0x01
#define MIC_PGAGAIN_M1p50  0x02
#define MIC_PGAGAIN_M2p25  0x03
#define MIC_PGAGAIN_M3p00  0x04
#define MIC_PGAGAIN_M3p75  0x05

#define HS_AMP_SHDN_N   1 //  confirm with HW, didn't mount this amp on device
#define FM34_PWDN_N     2 
#define FM34_RST_N      3
#define FM34_BP_N       4

#define FM34_PARAM_FILE_PATH "/data/.NVM/fm34_tuning.nvm"

struct fm34_platform_data {
	int		(*platform_init)(unsigned char);
	spinlock_t	lock;
};


enum enFM34_PATH
{
    enFM34_PATH_ONE_MIC = 0,
    enFM34_PATH_DUAL_MIC,
    enFM34_PATH_BY_PASS,
    enFM34_PATH_RECORDING_dual,
    enFM34_PATH_RECORDING_main,
    enFM34_PATH_RECORDING_ref,
    enFM34_PATH_RECORDING_voice,
    enFM34_PATH_RECORDING_VR,
    
    enFM34_TOTAL_PATH
} ;

enum enFM34_ACTION
{
    enFM34_END_ACTION,
    enFM34_SLEEP,
    enFM34_PWRDN_PIN = 2,
    enFM34_RST_PIN   = 3,
    enFM34_BYPASS_PIN= 4,
    enFM34_SET_REG,
    enFM34_RESET_FM34,
    
    enFM34_TOTAL_ACTION,
} ;

enum after_Mute_Action
{
	DO_NOTHING,
	ONE_MIC,
	BY_PASS,
	DUAL_MIC
};

#define FM34_TABLE_MAX_ACTION 100

typedef struct _tag_FM34_PATH_TABLE
{
    enum enFM34_ACTION fm34_action;
    unsigned int reg;
    unsigned int value;        

}FM34_PATH_TABLE, *pFM34_PATH_TABLE;



/* General */

#endif

void do_FM34_Table(enum enFM34_PATH fm34_Path);
void init_fm34_when_bl_on(void);    //Bruno++
#define FM34_ON2ON_DEBOUNCE_TIME     150  //ms
