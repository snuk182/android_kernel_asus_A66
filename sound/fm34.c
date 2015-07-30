//--------------------------------------------------------------------
//                     ASUSTek Computer Inc.
//         Copyright (c) 2010 ASUSTek Computer inc, Taipei.
//
//			FM34 Voice processor
//--------------------------------------------------------------------
//File: fm34.c
//Revision History:

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include "fm34.h"
#include <linux/i2c.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#include <linux/microp_notify.h>

int gbFM34attached = 0;
int gPIN_FM34_ON = 0;
static struct work_struct fm34_attach_work;
extern bool hdmi_exist(void);   //Bruno++
extern void reportPadStationI2CFail(char *devname);
extern bool g_p01State;
#ifdef CONFIG_CHARGER_MODE
extern char g_CHG_mode;
#endif

DEFINE_MUTEX(fm34_mutex);

#if 1
struct s_dsp_patch{
    unsigned short addr;
    unsigned char data1;
    unsigned char data2;
    unsigned char data3;
    unsigned char dummy;
} ;

static struct s_dsp_patch dsp_patch[] = 
{
    {0x1000, 0x90, 0x00, 0x1A},
    {0x1001, 0x82, 0x30, 0x50},
    {0x1002, 0x27, 0x90, 0x9F},
    {0x1003, 0x19, 0x00, 0xA0},
    {0x1004, 0x1C, 0x5A, 0xAF},
    {0x1005, 0x90, 0x00, 0x1A},
    {0x1006, 0x80, 0x00, 0x0A},
    {0x1007, 0x1C, 0x5A, 0xAF},
    {0x1008, 0x90, 0x00, 0x0A},
    {0x1009, 0x19, 0x01, 0x1F},
    {0x100A, 0x27, 0x90, 0x5F},
    {0x100B, 0x19, 0x01, 0x10},
    {0x100C, 0x1C, 0x58, 0xEF},
    {0x100D, 0x90, 0x00, 0x1A},
    {0x100E, 0x80, 0x00, 0x0A},
    {0x100F, 0x1C, 0x58, 0xEF},
    {0x1010, 0x90, 0x00, 0x0A},
    {0x1011, 0x82, 0x2D, 0x20},
    {0x1012, 0x27, 0x98, 0x5F},
    {0x1013, 0x19, 0x01, 0x80},
    {0x1014, 0x80, 0x50, 0x5A},
    {0x1015, 0x90, 0x50, 0x3A},
    {0x1016, 0x80, 0x50, 0x6F},
    {0x1017, 0x90, 0x50, 0x4F},
    {0x1018, 0x34, 0x00, 0x0E},
    {0x1019, 0x19, 0x00, 0x6F},
    {0x101A, 0x80, 0x00, 0xAC},
    {0x101B, 0x26, 0x7C, 0x0F},
    {0x101C, 0x34, 0x00, 0x0E},
    {0x101D, 0x19, 0x1D, 0x82},
    {0x101E, 0x19, 0x1C, 0xEF},
    {0x101F, 0x80, 0x53, 0x9A},
    {0x1020, 0x93, 0xE2, 0xAA},
    {0x1021, 0x80, 0x4F, 0xBA},
    {0x1022, 0x22, 0x7A, 0x0F},
    {0x1023, 0x34, 0x00, 0x0E},
    {0x1024, 0x18, 0x2F, 0xD0},
    {0x1025, 0x94, 0x4F, 0xB6},
    {0x1026, 0x80, 0x4F, 0x6A},
    {0x1027, 0x26, 0x7A, 0x0F},
    {0x1028, 0x18, 0x2F, 0x80},
    {0x1029, 0x18, 0x2B, 0xCF},
    {0x102A, 0x95, 0x62, 0x06},
    {0x102B, 0x95, 0x61, 0x46},
    {0x102C, 0x40, 0xFA, 0x0A},
    {0x102D, 0x40, 0xE5, 0xB0},
    {0x102E, 0x82, 0x30, 0x14},
    {0x102F, 0x27, 0x00, 0x0F},
    {0x1030, 0x22, 0x78, 0x00},
    {0x1031, 0x83, 0xFD, 0x44},
    {0x1032, 0x26, 0xE2, 0x0F},
    {0x1033, 0x19, 0x03, 0x90},
    {0x1034, 0x93, 0xFD, 0x4A},
    {0x1035, 0x83, 0xFD, 0x5A},
    {0x1036, 0x23, 0xA2, 0x1F},
    {0x1037, 0x93, 0xFD, 0x5A},
    {0x1038, 0x00, 0x00, 0x00},
    {0x1039, 0x34, 0x00, 0x0E},
    {0x103A, 0x18, 0x3F, 0x7F},
    {0x103B, 0x34, 0x00, 0x0E},
    {0x103C, 0x19, 0x6C, 0xA4},
    {0x103D, 0x19, 0x66, 0x7F},
    {0x103E, 0x80, 0x79, 0xCA},
    {0x103F, 0x82, 0x2D, 0x2F},
    {0x1040, 0x27, 0x97, 0x9F},
    {0x1041, 0x19, 0x04, 0x40},
    {0x1042, 0x94, 0x7B, 0xD6},
    {0x1043, 0x80, 0x00, 0x1A},
    {0x1044, 0x34, 0x00, 0x0E},
    {0x1045, 0x18, 0x48, 0xDF},
    {0x1046, 0x38, 0x00, 0x87},
    {0x1047, 0x09, 0x00, 0x1B},
    {0x1048, 0x34, 0x00, 0x0E},
    {0x1049, 0x18, 0x2D, 0x1F},
    {0x104A, 0x18, 0x21, 0x5F},
    {0x104B, 0x88, 0x4F, 0x47},
    {0x104C, 0x82, 0x2A, 0x0A},
    {0x104D, 0x22, 0x62, 0x1F},
    {0x104E, 0x26, 0x62, 0x7F},
    {0x104F, 0x19, 0x05, 0x25},
    {0x1050, 0x3B, 0xFF, 0xC7},
    {0x1051, 0x19, 0x05, 0x3F},
    {0x1052, 0x0D, 0x01, 0xA6},
    {0x1053, 0x92, 0x2A, 0x0A},
    {0x1054, 0x34, 0x00, 0x0E},
    {0x1055, 0x18, 0x23, 0x2F},
    {0x1056, 0x96, 0x2A, 0xA1},
    {0x1057, 0x96, 0x2A, 0xB0},
    {0x1058, 0x9A, 0x2A, 0xCA},
    {0x1059, 0x9A, 0x2A, 0xD7},
    {0x105A, 0x96, 0x2A, 0xE8},
    {0x105B, 0x38, 0x00, 0x0A},
    {0x105C, 0x83, 0xFF, 0xAA},
    {0x105D, 0x3B, 0x20, 0x02},
    {0x105E, 0x36, 0x2B, 0x01},
    {0x105F, 0x36, 0x2B, 0x60},
    {0x1060, 0x1C, 0x72, 0x2F},
    {0x1061, 0x93, 0xFF, 0x9A},
    {0x1062, 0x93, 0xFF, 0xAA},
    {0x1063, 0x86, 0x2A, 0xA1},
    {0x1064, 0x86, 0x2A, 0xB0},
    {0x1065, 0x8A, 0x2A, 0xCA},
    {0x1066, 0x8A, 0x2A, 0xD7},
    {0x1067, 0x86, 0x2A, 0xE8},
    {0x1068, 0x0D, 0x08, 0x28},
    {0x1069, 0x34, 0x00, 0x0E},
    {0x106A, 0x18, 0x24, 0x0F},
    {0x106B, 0x83, 0xFC, 0xFA},
    {0x106C, 0x23, 0x8A, 0xBF},
    {0x106D, 0x82, 0x2D, 0x2F},
    {0x106E, 0x27, 0x97, 0xDF},
    {0x106F, 0x23, 0xAA, 0x91},
    {0x1070, 0x93, 0xFC, 0xFA},
    {0x1071, 0x83, 0x80, 0x30},
    {0x1072, 0x34, 0x00, 0x0E},
    {0x1073, 0x18, 0x3E, 0x9F},
    {0x1074, 0x80, 0x7A, 0x4A},
    {0x1075, 0x19, 0x07, 0xE2},
    {0x1076, 0x23, 0x3E, 0x0F},
    {0x1077, 0x0D, 0x00, 0xEA},
    {0x1078, 0x80, 0x7B, 0xCA},
    {0x1079, 0x38, 0x7E, 0x02},
    {0x107A, 0x1C, 0x76, 0x9F},
    {0x107B, 0x90, 0x7B, 0xCA},
    {0x107C, 0x34, 0x00, 0x0E},
    {0x107D, 0x19, 0x75, 0xCF},
    {0x107E, 0x34, 0x00, 0x0E},
    {0x107F, 0x19, 0x75, 0x9F},
};
#endif
struct s_fm34_init {
    unsigned short addr;
    unsigned short data;
} ;
struct s_fm34_init fm34_init_Data[] =
{

    {0x3FA0, 0x91CB},
    {0x3FB0, 0x501A},
    {0x3FA1, 0x82F3},
    {0x3FB1, 0x02F5},
    {0x3FA2, 0x82B5},
    {0x3FB2, 0x501F},
    {0x3FA3, 0x83F4},
    {0x3FB3, 0x502A},
    {0x3FA4, 0x9666},
    {0x3FB4, 0x503B},
    {0x3FA5, 0x82CC},
    {0x3FB5, 0x5046},
    {0x3FA6, 0xC2CC},
    {0x3FB6, 0x5046},
    {0x3FA7, 0xC210},
    {0x3FB7, 0x504A},
    {0x3FA8, 0x8210},
    {0x3FB8, 0x504A},
    {0x3FA9, 0x8231},
    {0x3FB9, 0x504B},
    {0x3FAA, 0xC231},
    {0x3FBA, 0x504B},
    {0x3FAB, 0x823F},
    {0x3FBB, 0x5056},
    {0x3FAC, 0xC23F},
    {0x3FBC, 0x5056},
    {0x3FAD, 0x83E8},
    {0x3FBD, 0x506B},
    {0x3FAE, 0x9758},
    {0x3FBE, 0x5074},
};

FM34_PATH_TABLE g_FM34_Table[enFM34_TOTAL_PATH][FM34_TABLE_MAX_ACTION] = 
{
    {   //enFM34_ONE_MIC // hands free
        {enFM34_RESET_FM34,0x8005,0xf8f8},
        {enFM34_SET_REG,0x23b9,0x000c},
        {enFM34_SET_REG,0x22d2,0x0294},
        {enFM34_SET_REG,0x2339,0x11},
        {enFM34_SET_REG,0x2301,0x12},
        {enFM34_SET_REG,0x23a4,0x0008},
        {enFM34_SET_REG,0x22f2,0x0040},
        {enFM34_SET_REG,0x23d7,0x2a},
        {enFM34_SET_REG,0x22ba,0x0},
        {enFM34_SET_REG,0x230d,0x300},
        {enFM34_SET_REG,0x230c,0x300},
        {enFM34_SET_REG,0x2303,0x49e1},
        {enFM34_SET_REG,0x23e0,0x4190},
        {enFM34_SET_REG,0x23e1,0x4000},
        {enFM34_SET_REG,0x23E2,0x4000},
        {enFM34_SET_REG,0x23E3,0x4000},
        {enFM34_SET_REG,0x23E4,0x4000},
        {enFM34_SET_REG,0x2309,0x0800}, 
        {enFM34_SET_REG,0x22FB,0x0},
    },
    {   //enFM34_DUAL_MIC // handset
        {enFM34_RESET_FM34,0x8002,0xf3f3},
        {enFM34_SET_REG,0x2301,0x0012},
        {enFM34_SET_REG,0x230C,0x0230},
        {enFM34_SET_REG,0x230D,0x0220},
        {enFM34_SET_REG,0x232F,0x0005},
        {enFM34_SET_REG,0x2337,0xffff},
        {enFM34_SET_REG,0x2339,0x0012},
        {enFM34_SET_REG,0x236E,0x1200},
        {enFM34_SET_REG,0x236F,0x0C00},
        {enFM34_SET_REG,0x2370,0x1000},
        {enFM34_SET_REG,0x2305,0x002C},
        {enFM34_SET_REG,0x22F2,0x0040},
        {enFM34_SET_REG,0x2351,0x2000},
        {enFM34_SET_REG,0x2303,0x6DE1},
        {enFM34_SET_REG,0x23E2,0x4000},
        {enFM34_SET_REG,0x23E3,0x4000},
        {enFM34_SET_REG,0x23E4,0x4000},
        {enFM34_SET_REG,0x22FB,0x0},
    }, 
    {   //enFM34_BY_PASS
        {enFM34_RESET_FM34,0x8002,0xf0},
        {enFM34_SET_REG,0x22FB,0x0},
        {enFM34_SLEEP,200,200},
        {enFM34_SET_REG, 0x22f6, 0x02}, 
        {enFM34_SET_REG, 0x22f5, 0x03}, 
        {enFM34_BYPASS_PIN, 0, 0},
    }, 
    {   //enFM34_PATH_RECORDING_dual
        {enFM34_RESET_FM34, 0x8005, 0x1010},
        {enFM34_SET_REG, 0x22C6, 0x007D},
        {enFM34_SET_REG, 0x22C7, 0x0000},
        {enFM34_SET_REG, 0x22C8, 0x000C},
        {enFM34_SET_REG, 0x22FA, 0x2489},
        {enFM34_SET_REG, 0x22F9, 0x001F},
        {enFM34_SET_REG, 0x23ce, 0x800},
        {enFM34_SET_REG, 0x23b9, 0xb},
        {enFM34_SET_REG, 0x22d2, 0x0a94},
        {enFM34_SET_REG, 0x22f6, 0x3},
        {enFM34_SET_REG, 0x2339, 0x8},
        {enFM34_SET_REG, 0x2301, 0x2},
        {enFM34_SET_REG, 0x23a4, 0x8},
        {enFM34_SET_REG, 0x22ba, 0x0},
        {enFM34_SET_REG, 0x230d, 0x100},
        {enFM34_SET_REG, 0x230c, 0x100},
        {enFM34_SET_REG, 0x2303, 0x0001},
        {enFM34_SET_REG, 0x2304, 0x2310},
        {enFM34_SET_REG, 0x236e, 0x1600},
        {enFM34_SET_REG, 0x2370, 0x1800},
        {enFM34_SET_REG, 0x2390, 0x3a06},
        {enFM34_SET_REG, 0x2391, 0x4000},
        {enFM34_SET_REG, 0x2392, 0x4000},
        {enFM34_SET_REG, 0x2393, 0x4000},
        {enFM34_SET_REG, 0x2394, 0x4000},
        {enFM34_SET_REG, 0x2395, 0x4000},
        {enFM34_SET_REG, 0x2357, 0x100},
        {enFM34_SET_REG, 0x22ee, 0},
        {enFM34_SET_REG, 0x22F2, 0x0044},
        {enFM34_SET_REG, 0x2305, 0x0001},
        {enFM34_SET_REG, 0x232F, 0x0100},
        {enFM34_SET_REG, 0x2333, 0x0008},
        {enFM34_SET_REG, 0x2384, 0x0008},
        {enFM34_SET_REG, 0x23B3, 0x0010},
        {enFM34_SET_REG, 0x23B4, 0x005},
        {enFM34_SET_REG, 0x23CF, 0x1000},
        {enFM34_SET_REG, 0x23D0, 0x0620},
        {enFM34_SET_REG, 0x23D5, 0x6000},
        {enFM34_SET_REG, 0x3FD2, 0x0032},
        {enFM34_SET_REG, 0x22FB, 0x0000},
        {enFM34_END_ACTION, 0, 0},
    }, 
    {   //enFM34_PATH_RECORDING_main
        {enFM34_RESET_FM34, 0x8005, 0xf8f8}, 
        {enFM34_SET_REG, 0x23ce, 0x800}, 
        {enFM34_SET_REG, 0x230c, 0x800}, 
        {enFM34_SET_REG, 0x2357, 0x600}, 
        {enFM34_SET_REG, 0x2302, 0x101}, 
        {enFM34_SET_REG, 0x22FB, 0}, 
        {enFM34_SET_REG, 0x04f6, 0x1},
    }, 
    {   //enFM34_PATH_RECORDING_ref
		{enFM34_RESET_FM34,0x8005,0x0000 },
		{enFM34_SET_REG,0x22f8,0x8005},
		{enFM34_SET_REG,0x22C6,0x007D},
		{enFM34_SET_REG,0x22C7,0x0000},
		{enFM34_SET_REG,0x22C8,0x000C},
		{enFM34_SET_REG,0x22FA,0x2289},
		{enFM34_SET_REG,0x22F9,0x001F},
		{enFM34_SET_REG,0x2307,0x0000},
		{enFM34_SET_REG,0x23ce,0x800 },
		{enFM34_SET_REG,0x23b9,0xB   },
		{enFM34_SET_REG,0x22d2,0xa94 },
		{enFM34_SET_REG,0x22F6,0x3   },
		{enFM34_SET_REG,0x2339,0x20  },
		{enFM34_SET_REG,0x2301,0x2   },
		{enFM34_SET_REG,0x23a4,0x8   },
		{enFM34_SET_REG,0x230d,0x100 },
		{enFM34_SET_REG,0x230c,0x100 },
		{enFM34_SET_REG,0x2303,0x0001},
		{enFM34_SET_REG,0x2304,0x2310},
		{enFM34_SET_REG,0x236e,0x7FFF},
		{enFM34_SET_REG,0x2370,0x1800},
		{enFM34_SET_REG,0x2390,0x3a06},
		{enFM34_SET_REG,0x2391,0x4000},
		{enFM34_SET_REG,0x2392,0x4000},
		{enFM34_SET_REG,0x2393,0x4000},
		{enFM34_SET_REG,0x2394,0x4000},
		{enFM34_SET_REG,0x2395,0x4000},
		{enFM34_SET_REG,0x2357,0x100 },
		{enFM34_SET_REG,0x22ee,0x0   },
		{enFM34_SET_REG,0x22f2,0x44  },
		{enFM34_SET_REG,0x2305,0x1   },
		{enFM34_SET_REG,0x232f,0xA0  },
		{enFM34_SET_REG,0x2333,0x8   },
		{enFM34_SET_REG,0x2384,0x8   },
		{enFM34_SET_REG,0x23b3,0x18  },
		{enFM34_SET_REG,0x23b4,0x10  },
		{enFM34_SET_REG,0x23BD,0x1000},
		{enFM34_SET_REG,0x23BC,0x7FFF},
		{enFM34_SET_REG,0x233c,0x0600},
		{enFM34_SET_REG,0x2309,0x0800},
		{enFM34_SET_REG,0x23cf,0x0780},
		{enFM34_SET_REG,0x23d0,0x620 },
		{enFM34_SET_REG,0x23d5,0x3C00},
		{enFM34_SET_REG,0x23B7,0x14  },
		{enFM34_SET_REG,0x3fd2,0x32  },
		{enFM34_SET_REG,0x2348,0x0800},
		{enFM34_SET_REG,0x2349,0x0800},
		{enFM34_SET_REG,0x2332,0x3000},
		{enFM34_SET_REG,0x3FA0,0x91CB},
		{enFM34_SET_REG,0x3FB0,0x501A},
		{enFM34_SET_REG,0x3FA1,0x82F3},
		{enFM34_SET_REG,0x3FB1,0x02F5},
		{enFM34_SET_REG,0x3FA2,0x82B5},
		{enFM34_SET_REG,0x3FB2,0x501F},
		{enFM34_SET_REG,0x3FA3,0x83F4},
		{enFM34_SET_REG,0x3FB3,0x502A},
		{enFM34_SET_REG,0x3FA4,0x9666},
		{enFM34_SET_REG,0x3FB4,0x503B},
		{enFM34_SET_REG,0x3FA5,0x82CC},
		{enFM34_SET_REG,0x3FB5,0x5046},
		{enFM34_SET_REG,0x3FA6,0xC2CC},
		{enFM34_SET_REG,0x3FB6,0x5046},
		{enFM34_SET_REG,0x3FA7,0xC210},
		{enFM34_SET_REG,0x3FB7,0x504A},
		{enFM34_SET_REG,0x3FA8,0x8210},
		{enFM34_SET_REG,0x3FB8,0x504A},
		{enFM34_SET_REG,0x3FA9,0x8231},
		{enFM34_SET_REG,0x3FB9,0x504B},
		{enFM34_SET_REG,0x3FAA,0xC231},
		{enFM34_SET_REG,0x3FBA,0x504B},
		{enFM34_SET_REG,0x3FAB,0x823F},
		{enFM34_SET_REG,0x3FBB,0x5056},
		{enFM34_SET_REG,0x3FAC,0xC23F},
		{enFM34_SET_REG,0x3FBC,0x5056},
		{enFM34_SET_REG,0x3FAD,0x83E8},
		{enFM34_SET_REG,0x3FBD,0x506B},
		{enFM34_SET_REG,0x3FAE,0x9758},
		{enFM34_SET_REG,0x3FBE,0x5074},
		{enFM34_SET_REG,0x22fb,0x0   },
        {enFM34_END_ACTION, 0, 0},
    }, 
	{   //enFM34_PATH_voice
        {enFM34_SET_REG, 0x2303, 0x0001}, 
        {enFM34_SET_REG, 0x2384, 0x0008}, 
        {enFM34_SET_REG, 0x2304, 0x2310}, 
        {enFM34_SET_REG, 0x230C, 0x0100}, 
        {enFM34_SET_REG, 0x23CF, 0x0780}, 
        {enFM34_SET_REG, 0x23D0, 0x0620},
		{enFM34_SET_REG, 0x23D5, 0x3C00},
		{enFM34_END_ACTION, 0, 0},
    }, 
	{   //enFM34_PATH_VR
        {enFM34_SET_REG, 0x2303, 0x0000}, 
        {enFM34_SET_REG, 0x2384, 0x0003}, 
        {enFM34_SET_REG, 0x2304, 0x0010}, 
        {enFM34_SET_REG, 0x230C, 0x0200}, 
        {enFM34_SET_REG, 0x23CF, 0x0200}, 
        {enFM34_SET_REG, 0x23D0, 0x0200},
		{enFM34_SET_REG, 0x23D5, 0x0200},
		{enFM34_END_ACTION, 0, 0},
    }, 
};
//..................................................


static struct i2c_client *g_client;
static u32 g_Mem_Addr = 0;	//keep last proc_write reg addr, for next proc read

int fm34_mem_read(u16 addr, u32 *pval)
{
	int ret;
	int status;
	u8 val = 0;
	u8 MEMData[LEN_SYNC_BYTE+LEN_MEM_READ];
	u8 REGData[LEN_SYNC_BYTE+LEN_REG_READ];
	int i;

	printk(DBGMSK_SND_G2  "[Audio] fm34_mem_read addr 0x%x\n",addr);
	if (g_client == NULL)	/* i2c client pointer is NULL? */
		return -ENODEV;

	// Pack mem addr
	// the seq should be 
	// start 0xC0 0xFC 0xF3 0x37 addr_high addr_low 0xC0 stop
	MEMData[0] = SYNC_BYTE1;
	MEMData[1] = SYNC_BYTE2;
	MEMData[2] = CMD_MEM_READ;
	MEMData[3] = (addr >> 8) & 0xFF;
	MEMData[4] = addr & 0xFF ;
	
	printk(DBGMSK_SND_G2  "[Audio] fm34_mem_read (write addr) 0x%x 0x%x 0x%x 0x%x 0x%x  \n",MEMData[0],MEMData[1],MEMData[2],MEMData[3],MEMData[4]);
	for(i=0; i<5; i++)
	{
        if (hdmi_exist() && gPIN_FM34_ON) {
		    ret =  i2c_master_send(g_client, MEMData, LEN_SYNC_BYTE+LEN_MEM_READ);
        } else {
            printk("[Audio] hdmi_exist:%d gPIN_FM34_ON:%d\n", hdmi_exist(), gPIN_FM34_ON);
            return -ENODEV;
        }
        
		if ((LEN_SYNC_BYTE+LEN_MEM_READ) == ret) {
			status = 0;
			break;
		} else {
			printk(KERN_ERR "[Audio] fm34_mem_read, write addr fail  ret=%d\n", ret);
			status = -EIO;	
		}
		
		msleep(25);
	}

    if (status == -EIO && !(isASUS_MSK_set(DBGMSK_SND_G7)))
    {
        reportPadStationI2CFail("fm34");
        return status;
    }

	// Pack reg addr to read 0x25
	// the seq should be 
	// start 0xC0 0xFC 0xF3 0x60 0x25 start 0xC1 get_low_byte stop
	// but there is no existed i2c comment could complete this process.
	// so we combin it as
	// 1. start 0xC0 0xFC 0xF3 0x60 0x25 stop (the stop is not expected by FM34)
	// 2. start 0xC1 get_low_byte stop
	REGData[0] = SYNC_BYTE1;
	REGData[1] = SYNC_BYTE2;
	REGData[2] = CMD_REG_READ;
	REGData[3] = 0x25;

	printk(DBGMSK_SND_G2  "[Audio] fm34_mem_read (write reg) 0x%x 0x%x 0x%x 0x%x \n",REGData[0],REGData[1],REGData[2],REGData[3]);
	for(i=0; i<5; i++)
	{
        if (hdmi_exist() && gPIN_FM34_ON) {
		    ret =  i2c_master_send(g_client, REGData, LEN_SYNC_BYTE+LEN_REG_READ);
        } else {
            printk("[Audio] hdmi_exist:%d gPIN_FM34_ON:%d\n", hdmi_exist(), gPIN_FM34_ON);
            return -ENODEV;
        }

		if ((LEN_SYNC_BYTE+LEN_REG_READ) == ret) {
			status = 0;
			break;
		} else {
			printk(KERN_ERR "[Audio] fm34_mem_read, write reg 0x25 fail ret=%d\n", ret);
			status = -EIO;	
		}
		
		msleep(25);
	}

    if (status == -EIO && !(isASUS_MSK_set(DBGMSK_SND_G7)))
    {
        reportPadStationI2CFail("fm34");
        return status;
    }
	
	for(i=0; i<5; i++)
	{
        if (hdmi_exist() && gPIN_FM34_ON) {
		    ret =  i2c_master_recv(g_client, &val ,1);
        } else {
            printk("[Audio] hdmi_exist:%d gPIN_FM34_ON:%d\n", hdmi_exist(), gPIN_FM34_ON);
            return -ENODEV;
        }

		if(1 == ret) {
			break;
		} else {
			printk(KERN_ERR "[Audio] fm34_mem_read fail ret=%d\n", ret);
            status = -EIO;
        }
        
        msleep(25);
	}

    if (status == -EIO && !(isASUS_MSK_set(DBGMSK_SND_G7)))
    {
        reportPadStationI2CFail("fm34");
        return status;
    }

	*pval = (val&0xFF);
	printk(DBGMSK_SND_G2  "[Audio] fm34_mem_read, 0x25 = 0x%x\n",val);

	// Pack reg addr to read 0x26
	// the seq should be 
	// start 0xC0 0xFC 0xF3 0x60 0x26 start 0xC1 get_high_byte stop
	// but there is no existed i2c comment could complete this process.
	// so we combin it as
	// 1. start 0xC0 0xFC 0xF3 0x60 0x26 stop (the stop is not expected by FM34)
	// 2. start 0xC1 get_low_byte stop
	REGData[0] = SYNC_BYTE1;
	REGData[1] = SYNC_BYTE2;
	REGData[2] = CMD_REG_READ;
	REGData[3] = 0x26;

	printk(DBGMSK_SND_G2  "[Audio] fm34_mem_read (write reg) 0x%x 0x%x 0x%x 0x%x \n",REGData[0],REGData[1],REGData[2],REGData[3]);
	for(i=0; i<5; i++)
	{
        if (hdmi_exist() && gPIN_FM34_ON) {           
		    ret =  i2c_master_send(g_client, REGData, LEN_SYNC_BYTE+LEN_REG_READ);
        } else {
            printk("[Audio] hdmi_exist:%d gPIN_FM34_ON:%d\n", hdmi_exist(), gPIN_FM34_ON);
            return -ENODEV;
        }
	
		if ((LEN_SYNC_BYTE+LEN_REG_READ) == ret) {
			status = 0;            
			break;
		} else {
			printk(KERN_ERR "[Audio] fm34_mem_read, write reg 0x26 fail ret=%d\n", ret);
			status = -EIO;	
		}
		
		msleep(25);
	}

    if (status == -EIO && !(isASUS_MSK_set(DBGMSK_SND_G7)))
    {
        reportPadStationI2CFail("fm34");
        return status;
    }

	for(i=0; i<5; i++)
	{
        if (hdmi_exist() && gPIN_FM34_ON) { 
		    ret =  i2c_master_recv(g_client, &val ,1);
        } else {
            printk("[Audio] hdmi_exist:%d gPIN_FM34_ON:%d\n", hdmi_exist(), gPIN_FM34_ON);
            return -ENODEV;
        }

		if(1 == ret) {
			break;
		} else {
			printk(KERN_ERR "[Audio] fm34_mem_read fail ret=%d\n", ret);
            status = -EIO;
		}
		
		msleep(25);		
	}

    if (status == -EIO && !(isASUS_MSK_set(DBGMSK_SND_G7)))
    {
        reportPadStationI2CFail("fm34");
        return status;
    }
    
    printk(DBGMSK_SND_G2  "[Audio] fm34_mem_read, 0x26 = 0x%x\n",val);

	*pval |= ((val<<8)&0xFF00);
	printk(DBGMSK_SND_G2  "[Audio] fm34_mem_read [0x%x]=[0x%x]\n",addr,*pval);
	return status;
}

int fm34_mem_write(u16 addr, u16 val)
{
	int ret;
	int status;
	u8 Data[LEN_SYNC_BYTE+LEN_MEM_WRITE];
	int i;

	printk( DBGMSK_SND_G2 "[Audio] fm34_mem_write [0x%x]=[0x%x]\n",addr,val);
	if (g_client == NULL)	/* i2c client pointer is NULL? */
		return -ENODEV;

	
	Data[0] = SYNC_BYTE1;
	Data[1] = SYNC_BYTE2;
	Data[2] = CMD_MEM_WRITE;
	Data[3] = (addr >> 8) & 0xFF;
	Data[4] = addr & 0xFF ;
	Data[5] = (val >> 8) & 0xFF;
	Data[6] = val & 0xFF ;
	
	for(i=0; i<5; i++)
	{
        if (hdmi_exist() && gPIN_FM34_ON) {
		    ret =  i2c_master_send(g_client,Data ,LEN_SYNC_BYTE+LEN_MEM_WRITE);
        } else {
            printk("[Audio] hdmi_exist:%d gPIN_FM34_ON:%d\n", hdmi_exist(), gPIN_FM34_ON);
            return -ENODEV;
        }
	
		if ((LEN_SYNC_BYTE+LEN_MEM_WRITE) == ret) {
			status = 0;
			break;
		} else {
			printk( "[Audio] fm34_mem_write fail, retry %d ret=%d\n",i, ret);
			status = -EIO;	
		}
		
		msleep(25);
	}

    if (status == -EIO && !(isASUS_MSK_set(DBGMSK_SND_G7)))
    {
        printk("[Audio][FM34] gPIN_FM34_ON:%d RST:%d PDN:%d\n",
                gPIN_FM34_ON, AX_MicroP_getGPIOOutputPinLevel(OUT_uP_FM34_RST), AX_MicroP_getGPIOOutputPinLevel(OUT_uP_FM34_PDN));
        reportPadStationI2CFail("fm34");
        return status;
    }

	return status;
}

int fm34_port_write(u8 addr, u8 val)
{
    int ret;
    int status;
    u8 Data[LEN_SYNC_BYTE+LEN_PORT_WRITE];
    int i;

    printk( DBGMSK_SND_G2 "[Audio] fm34_port_write [0x%x]=[0x%x]\n",addr,val);
    if (g_client == NULL)   /* i2c client pointer is NULL? */
        return -ENODEV;

    
    Data[0] = SYNC_BYTE1;
    Data[1] = SYNC_BYTE2;
    Data[2] = CMD_PORT_WRITE;
    Data[3] = (addr) & 0xFF;
    Data[4] = (val ) & 0xFF;
        
    for(i=0; i<5; i++)
    {
        if (hdmi_exist() && gPIN_FM34_ON) {
            ret =  i2c_master_send(g_client,Data ,LEN_SYNC_BYTE+LEN_PORT_WRITE);
        } else {
            printk("[Audio] hdmi_exist:%d gPIN_FM34_ON:%d\n", hdmi_exist(), gPIN_FM34_ON);
            return -ENODEV;
        }
        
        if ((LEN_SYNC_BYTE+LEN_PORT_WRITE) == ret) {
            status = 0;
            break;
        } else {
            printk( "[Audio] fm34_port_write fail, retry %d ret=%d\n",i, ret);
            status = -EIO;  
        }
        
        msleep(25);
    }

    if (status == -EIO && !(isASUS_MSK_set(DBGMSK_SND_G7)))
    {
        printk("[Audio][FM34] gPIN_FM34_ON:%d RST:%d PDN:%d\n",
                gPIN_FM34_ON, AX_MicroP_getGPIOOutputPinLevel(OUT_uP_FM34_RST), AX_MicroP_getGPIOOutputPinLevel(OUT_uP_FM34_PDN));
        reportPadStationI2CFail("fm34");
        return status;
    }

    return status;
}


int fm34_patch_write(u16 addr, u8* val)
{
    int ret;
    int status;
    u8 Data[LEN_SYNC_BYTE+LEN_PATCH_WRITE];
    int i;

    printk( DBGMSK_SND_G2 "[Audio] fm34_patch_write [0x%x]=[0x%x,0x%x,0x%x]\n",addr,val[0],val[1],val[2]);
    if (g_client == NULL)   /* i2c client pointer is NULL? */
        return -ENODEV;

    Data[0] = SYNC_BYTE1;
    Data[1] = SYNC_BYTE2;
    Data[2] = CMD_PATCH_WRITE;
    Data[3] = (addr >> 8) & 0xFF;
    Data[4] = (addr) & 0xFF;
    Data[5] = (val[0] ) & 0xFF;
    Data[6] = (val[1] ) & 0xFF;
    Data[7] = (val[2] ) & 0xFF;
        
    for(i=0; i<5; i++)
    {
        if (hdmi_exist() && gPIN_FM34_ON) {
            ret =  i2c_master_send(g_client,Data ,LEN_SYNC_BYTE+LEN_PATCH_WRITE);
        } else {
            printk("[Audio] hdmi_exist:%d gPIN_FM34_ON:%d\n", hdmi_exist(), gPIN_FM34_ON);
            return -ENODEV;
        }
    
        if ((LEN_SYNC_BYTE+LEN_PATCH_WRITE) == ret) {
            status = 0;
            break;
        } else {
            printk( "[Audio] fm34_patch_write fail, retry %d ret=%d\n",i, ret);
            status = -EIO;  
        }
       
        msleep(25);
		
    }

    if (status == -EIO && !(isASUS_MSK_set(DBGMSK_SND_G7)))
    {
        printk("[Audio][FM34] gPIN_FM34_ON:%d RST:%d PDN:%d\n",
                gPIN_FM34_ON, AX_MicroP_getGPIOOutputPinLevel(OUT_uP_FM34_RST), AX_MicroP_getGPIOOutputPinLevel(OUT_uP_FM34_PDN));
        reportPadStationI2CFail("fm34");
        return status;
    }

    return status;
}


int fm34_patch_write_short(u16 addr, u8 val[], u16 len)
{
    int ret;
    int status = 0;
    u8 *Data;
    int i, len_sent;

    printk( DBGMSK_SND_G2 "[Audio] fm34_patch_write_short len=%d, [0x%x]=[0x%x]\n", len,addr,val[0]);
    if (g_client == NULL)   /* i2c client pointer is NULL? */
        return -ENODEV;

    len_sent = LEN_SYNC_BYTE + 3 + len;
    Data = kmalloc(len_sent, GFP_KERNEL);
    if(Data)
    {       
        Data[0] = SYNC_BYTE1;
        Data[1] = SYNC_BYTE2;
        Data[2] = CMD_PATCH_WRITE;
        Data[3] = (addr >> 8) & 0xFF;
        Data[4] = (addr) & 0xFF;
        
        for(i=0; i<len; i++)
        {
            Data[5 + i] = val[i];
        }
        
        for(i=0; i<5; i++)
        {
            if (hdmi_exist() && gPIN_FM34_ON) {
                ret =  i2c_master_send(g_client,Data , len_sent);
            } else {
                printk("[Audio] hdmi_exist:%d gPIN_FM34_ON:%d\n", hdmi_exist(), gPIN_FM34_ON);
                return -ENODEV;
            }
        
            if (len_sent == ret) {
                status = 0;
                break;
            } else {
                printk( "[Audio] fm34_patch_write_short fail, retry %d ret=%d\n",i, ret);
                status = -EIO;  
            }
            
            msleep(25);
        }

        if (status == -EIO && !(isASUS_MSK_set(DBGMSK_SND_G7)))
        {
            printk("[Audio][FM34] gPIN_FM34_ON:%d RST:%d PDN:%d\n",
                    gPIN_FM34_ON, AX_MicroP_getGPIOOutputPinLevel(OUT_uP_FM34_RST), AX_MicroP_getGPIOOutputPinLevel(OUT_uP_FM34_PDN));
            reportPadStationI2CFail("fm34");
            return status;
        }
            
        kfree(Data);    
    }

    return status;
}

int fm34_patch_DSP(void)
{
    int i;
    int ret = 0;

    ret = fm34_port_write(0x64, 0x04);
    if (ret != 0) {
        printk("[Audio][FM34] fm34_patch_DSP fail.\n");
        return ret;
    }
    
    msleep(30);

    printk("[fm34] fm34_patch_write start!!!\n");
    for(i = 0; i < sizeof(dsp_patch) / sizeof(struct s_dsp_patch); i++)
    {
        ret = fm34_patch_write(dsp_patch[i].addr, &dsp_patch[i].data1);
        if (ret != 0) {            
            printk("[Audio][FM34] fm34_patch_DSP fail.\n");
            return ret;
        }
    }

    ret = fm34_port_write(0x64, 0x00);
    if (ret != 0) {            
        printk("[Audio][FM34] fm34_patch_DSP fail.\n");
        return ret;
    }

    for(i = 0; i < sizeof(fm34_init_Data) / sizeof(struct s_fm34_init); i++)
    {
        ret = fm34_mem_write(fm34_init_Data[i].addr, fm34_init_Data[i].data);
        if (ret != 0) {            
            printk("[Audio][FM34] fm34_patch_DSP fail.\n");
            return ret;
        }
    }
    msleep(30);
    //gbFM34attached = 1;     //set to 1 if no error
    return ret;
}

#ifdef	CONFIG_PROC_FS
#define	FM34_PROC_FILE	"driver/fm34"
static struct proc_dir_entry *fm34_proc_file;


//TODO, sometimes read is not stable.
// but proc_write is fine.
static ssize_t fm34_proc_read(struct file *filp,
		char *buffer, size_t length, loff_t *offset)
{
	u32 reg_val;
    int ret = 0;

    ret = fm34_mem_read(g_Mem_Addr, &reg_val);
    if (ret != 0) {            
        printk("[Audio][FM34] fm34_proc_read fail.\n");
        return 0;
    }
	
	printk(DBGMSK_SND_G2  "[Audio]fm34_proc_read 0x%x: 0x%x\n", g_Mem_Addr, reg_val);
	return 0;
}

int do_reset_FM34(u32 paramProfile, u32 paramPGAGain)
{
    int ret = 0;

    printk(DBGMSK_SND_G2"[Audio]do_reset_FM34 +++ \n");
#if 0
    AX_MicroP_setGPIOOutputPin(OUT_uP_FM34_PDN, 1);
    gPIN_FM34_ON = 1;
    AX_MicroP_setGPIOOutputPin(OUT_uP_FM34_RST, 0);
    msleep(60);
    AX_MicroP_setGPIOOutputPin(OUT_uP_FM34_RST, 1);
    msleep(30);
#endif
    if(gbFM34attached)
    {
        ret = fm34_mem_write(0x22F8, paramProfile);
        if (ret != 0) {            
            printk("[Audio][FM34] do_reset_FM34 fail.\n");
            return 0;
        }
    
        msleep(20); //sleep for profile loading

        ret = fm34_mem_write(0x2307, paramPGAGain);
        if (ret != 0) {            
            printk("[Audio][FM34] do_reset_FM34 fail.\n");
            return 0;
        }
            
        msleep(10);
        printk(DBGMSK_SND_G2"[Audio]do_reset_FM34 --- \n");
    }   

    return 1;
}

void do_FM34_Table(enum enFM34_PATH fm34_Path)
{
    int i;
    int ret = 0;

    printk(DBGMSK_SND_G2"[audio] do_FM34_Table %d\n", fm34_Path);
    for(i = 0; i < FM34_TABLE_MAX_ACTION; i++)
    {
        switch(g_FM34_Table[fm34_Path][i].fm34_action)
        {
            case enFM34_RESET_FM34:
                printk(DBGMSK_SND_G2"[audio] do_FM34_Table enFM34_RESET_FM34\n"); 
                do_reset_FM34(g_FM34_Table[fm34_Path][i].reg, g_FM34_Table[fm34_Path][i].value);
                break;
            case enFM34_END_ACTION:
                printk(DBGMSK_SND_G2"[audio] do_FM34_Table enFM34_END_ACTION\n"); 
                return;
            case enFM34_SLEEP:
                printk(DBGMSK_SND_G2"[audio] do_FM34_Table enFM34_SLEEP %d\n", g_FM34_Table[fm34_Path][i].value); 
                msleep(g_FM34_Table[fm34_Path][i].value);
                break;            
            case enFM34_PWRDN_PIN:
                printk(DBGMSK_SND_G2"[audio] do_FM34_Table enFM34_PWRDN_PIN %x\n",g_FM34_Table[fm34_Path][i].value); 
                AX_MicroP_setGPIOOutputPin(OUT_uP_FM34_PDN, g_FM34_Table[fm34_Path][i].value);
                if (g_FM34_Table[fm34_Path][i].value) {
                    gPIN_FM34_ON = 1;
                } else {
                    gPIN_FM34_ON = 0;
                }
                break;
            case enFM34_RST_PIN: 
                printk(DBGMSK_SND_G2"[audio] do_FM34_Table enFM34_RST_PIN %x\n",g_FM34_Table[fm34_Path][i].value);  
                AX_MicroP_setGPIOOutputPin(OUT_uP_FM34_RST, g_FM34_Table[fm34_Path][i].value);
                break;
            case enFM34_SET_REG:
                printk(DBGMSK_SND_G2"[audio] do_FM34_Table enFM34_SET_REG %x, %x\n", g_FM34_Table[fm34_Path][i].reg, g_FM34_Table[fm34_Path][i].value); 
                ret = fm34_mem_write(g_FM34_Table[fm34_Path][i].reg, g_FM34_Table[fm34_Path][i].value);
                if (ret != 0) {            
                    printk("[Audio][FM34] do_FM34_Table fail.\n");
                    return;
                }
                break;
            default:
                printk(DBGMSK_SND_G2"[audio] ERROR! wrong action %d\n", g_FM34_Table[fm34_Path][i].fm34_action);
                break;
        }
        msleep(10);
    }

    return;
}

#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/file.h>

static mm_segment_t oldfs;
static void initKernelEnv(void)
{
    oldfs = get_fs();
    set_fs(KERNEL_DS);
}

static void deinitKernelEnv(void)
{
    set_fs(oldfs);
}
//g_FM34_Table

void load_fm34_param(void)
{
    int file_handle;
    char* buffer, *ptr;
    enum enFM34_PATH fm34_path = enFM34_PATH_ONE_MIC;
    int iorder = 0;
    int match;
    
    printk(DBGMSK_SND_G2"Doing load_fm34_param\n");
    initKernelEnv();
    file_handle = sys_open(FM34_PARAM_FILE_PATH, O_RDONLY, 0);
    
    if(!IS_ERR((const void *)file_handle))
    {
        memset(g_FM34_Table, 0, sizeof(g_FM34_Table));
        buffer = kmalloc(8192, GFP_KERNEL);
        if(buffer)
        {
            memset(buffer, 0, 8192);
            
            sys_read(file_handle, (unsigned char*)buffer, 8192);
            sys_close(file_handle);
            
            ptr = buffer;
            do
            {
                match = 0;
///// PATHs                
                if(!strncmp(ptr, "enFM34_PATH_ONE_MIC", strlen("enFM34_PATH_ONE_MIC")))
                {
                    printk(DBGMSK_SND_G2"enFM34_PATH_ONE_MIC\n");
                    fm34_path =  enFM34_PATH_ONE_MIC;   
                    ptr += strlen("enFM34_PATH_ONE_MIC");
                    iorder = 0;
                }
                else if(!strncmp(ptr, "enFM34_PATH_DUAL_MIC", strlen("enFM34_PATH_DUAL_MIC")))
                {
                    printk(DBGMSK_SND_G2"enFM34_PATH_DUAL_MIC\n");
                    fm34_path =  enFM34_PATH_DUAL_MIC;   
                    ptr += strlen("enFM34_PATH_DUAL_MIC");
                    iorder = 0;
                }
                else if(!strncmp(ptr, "enFM34_PATH_RECORDING_dual", strlen("enFM34_PATH_RECORDING_dual")))
                {
                    printk(DBGMSK_SND_G2"enFM34_PATH_RECORDING_dual\n");
                    fm34_path =  enFM34_PATH_RECORDING_dual;   
                    ptr += strlen("enFM34_PATH_RECORDING_dual");
                    iorder = 0;
                }
                else if(!strncmp(ptr, "enFM34_PATH_RECORDING_main", strlen("enFM34_PATH_RECORDING_main")))
                {
                    printk(DBGMSK_SND_G2"enFM34_PATH_RECORDING_main\n");
                    fm34_path =  enFM34_PATH_RECORDING_main;   
                    ptr += strlen("enFM34_PATH_RECORDING_main");
                    iorder = 0;
                }
                else if(!strncmp(ptr, "enFM34_PATH_RECORDING_ref", strlen("enFM34_PATH_RECORDING_ref")))
                {
                    printk(DBGMSK_SND_G2"enFM34_PATH_RECORDING_ref\n");
                    fm34_path =  enFM34_PATH_RECORDING_ref;   
                    ptr += strlen("enFM34_PATH_RECORDING_ref");
                    iorder = 0;
                }                
                else if(!strncmp(ptr, "enFM34_PATH_BY_PASS", strlen("enFM34_PATH_BY_PASS")))
                {
                    printk(DBGMSK_SND_G2"enFM34_PATH_BY_PASS\n");
                    fm34_path =  enFM34_PATH_BY_PASS;   
                    ptr += strlen("enFM34_PATH_BY_PASS");
                    iorder = 0;
                }
				else if(!strncmp(ptr, "enFM34_PATH_voice", strlen("enFM34_PATH_voice")))
                {
                    printk(DBGMSK_SND_G2"enFM34_PATH_voice\n");
                    fm34_path =  enFM34_PATH_RECORDING_voice;   
                    ptr += strlen("enFM34_PATH_voice");
                    iorder = 0;
                }
				else if(!strncmp(ptr, "enFM34_PATH_VR", strlen("enFM34_PATH_VR")))
                {
                    printk(DBGMSK_SND_G2"enFM34_PATH_VR\n");
                    fm34_path =  enFM34_PATH_RECORDING_VR;   
                    ptr += strlen("enFM34_PATH_VR");
                    iorder = 0;
                }
                
/////  actions                
                
                else if(!strncmp(ptr, "enFM34_SET_REG", strlen("enFM34_SET_REG")))
                {
                    printk(DBGMSK_SND_G2"enFM34_SET_REG\n");
                    match = 1;
                    g_FM34_Table[fm34_path][iorder].fm34_action = enFM34_SET_REG; 
                    ptr += strlen("enFM34_SET_REG");
                }
                else if(!strncmp(ptr, "enFM34_SLEEP", strlen("enFM34_SLEEP")))
                {
                    printk(DBGMSK_SND_G2"enFM34_SLEEP\n");
                    match = 1;
                    g_FM34_Table[fm34_path][iorder].fm34_action = enFM34_SLEEP;
                    ptr += strlen("enFM34_SLEEP");
                }
                else if(!strncmp(ptr, "enFM34_PWRDN_PIN", strlen("enFM34_PWRDN_PIN")))
                {
                    printk(DBGMSK_SND_G2"enFM34_PWRDN_PIN\n");
                    match = 1;
                    g_FM34_Table[fm34_path][iorder].fm34_action = enFM34_PWRDN_PIN;
                    ptr += strlen("enFM34_PWRDN_PIN");
                }
                else if(!strncmp(ptr, "enFM34_RST_PIN", strlen("enFM34_RST_PIN")))
                {
                    printk(DBGMSK_SND_G2"enFM34_RST_PIN\n");
                    match = 1;
                    g_FM34_Table[fm34_path][iorder].fm34_action = enFM34_RST_PIN;
                    ptr += strlen("enFM34_RST_PIN");
                }
                else if(!strncmp(ptr, "enFM34_BYPASS_PIN", strlen("enFM34_BYPASS_PIN")))
                {
                    printk(DBGMSK_SND_G2"enFM34_BYPASS_PIN\n");
                    match = 1;
                    g_FM34_Table[fm34_path][iorder].fm34_action = enFM34_BYPASS_PIN;
                    ptr += strlen("enFM34_BYPASS_PIN");
                }
                else if(!strncmp(ptr, "enFM34_RESET_FM34", strlen("enFM34_RESET_FM34")))
                {
                    printk(DBGMSK_SND_G2"enFM34_RESET_FM34\n");
                    match = 1;
                    g_FM34_Table[fm34_path][iorder].fm34_action = enFM34_RESET_FM34;
                    ptr += strlen("enFM34_RESET_FM34");
                }                
                
                if(match)
                {
                    sscanf(ptr, ",%x,%x", &g_FM34_Table[fm34_path][iorder].reg, &g_FM34_Table[fm34_path][iorder].value);
                    printk(DBGMSK_SND_G2"Table path %d, order %d, Action=%d, Reg=%X, value=%X\n", fm34_path, iorder, g_FM34_Table[fm34_path][iorder].fm34_action, g_FM34_Table[fm34_path][iorder].reg, g_FM34_Table[fm34_path][iorder].value);
                    iorder ++;
                    if(iorder == FM34_TABLE_MAX_ACTION)
                    {
                        printk("ERROR! more than %d actions\n", FM34_TABLE_MAX_ACTION);
                        break;
                    }
                }
            
            }while(*ptr++);
            
            kfree(buffer);
        }
        else
            printk("ERROR! buffer not allocated\n");

    }
    else
        printk("ERROR! param file not found\n");
    
    deinitKernelEnv();
}

/*
	system("/busybox/bin/echo a0 > /proc/driver/fm34");
	-> pull low BP

	system("/busybox/bin/echo a1 > /proc/driver/fm34");
	-> pull high BP

	system("/busybox/bin/echo 0xXXXX 0xXXXX > /proc/driver/fm34");
	-> set write reg addr and val

	so if you want to set 0x23FB=0x3, you should
	system("/busybox/bin/echo 0x23FB 0x3 > /proc/driver/fm34");
*/

int g_Mute = 0;
enum after_Mute_Action after_mute = DO_NOTHING;
	
static ssize_t fm34_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	u32 reg_val;
	char messages[256];
	int ret = 0;

    memset(messages, 0, sizeof(messages));
    printk(DBGMSK_SND_G2 "[Audio_FM34] fm34_proc_write\n");
	if (len > 256)
	{
		len = 256;
	}
	if (copy_from_user(messages, buff, len))
	{
		return -EFAULT;
	}
    
    if(strncmp(messages, "readall", strlen("readall")) == 0) //read register
    {
        u32 Reg[30] = { 0x22F8, 0x2307, 0x22F9, 0x22FA, 0x22ee,
                        0x22c6, 0x22c7, 0x22c8, 0x22cc, 0x22cd,
                        0x2301, 0x230C, 0x230D, 0x232F, 0x2337,
                        0x2339, 0x236E, 0x236F, 0x2370, 0x2305,
                        0x22F2, 0x2351, 0x2303, 0x23E2, 0x23E3,
                        0x23E4, 0x2352, 0x22FB};
        u32 val;
        int i;
        for(i = 0; i < 28; i++)
        {            
            ret = fm34_mem_read(Reg[i], &val);
            if (ret == 0) {
                printk("[Audio_FM34] read register reg[%x]=[%x]\n", Reg[i], val);
            } else {
                printk("[Audio][FM34] fm34_proc_write fail.\n");
                return 0;
            }
        }
    }
    else if(strncmp(messages, "read", strlen("read")) == 0) //read register
    {
        u32 val;
        sscanf(messages + 5, "%x", &reg_val);

        ret = fm34_mem_read(reg_val, &val);
        if (ret == 0) {
            printk("[Audio_FM34] read register reg[%x]=[%x]\n", reg_val, val);        
        } else {
            printk("[Audio][FM34] fm34_proc_write fail.\n");
            return 0;
        }
    }
    else if(strncmp(messages, "reset", strlen("reset")) == 0) //read register
    {
        printk(  "[Audio_FM34] reset fm34\n");       
        do_reset_FM34(0x8005, MIC_PGAGAIN_12);
        ret = fm34_mem_write(0x22FB, 0x00);
        if (ret != 0) {
            printk("[Audio][FM34] fm34_proc_write fail.\n");
            return 0;
        }
    }
    else if(strncmp(messages, "load_param", strlen("load_param")) == 0) // enable one MIC
    {
        printk(  "[Audio_FM34] LOAD_PARAM\n");
        load_fm34_param();
    } 
	else if(strncmp(messages, "ONBOARD_ONE_MIC", strlen("ONBOARD_ONE_MIC")) == 0) // enable one MIC
	{
		if (g_Mute)
		{
			printk(  "[Audio_FM34] after_mute = ONE_MIC\n");
			after_mute = ONE_MIC;
		}
		else
		{
	    	printk(  "[Audio_FM34] ONBOARD_ONE_MIC\n");
    		do_FM34_Table(enFM34_PATH_ONE_MIC);
		}
	} 
    else if(strncmp(messages, "RECORDING", strlen("RECORDING")) == 0) // set to bypass mode
	{
    	printk( "[Audio_FM34] PATH_RECORDING_MODE\n");
        do_FM34_Table(enFM34_PATH_RECORDING_dual);
	}
    else if(strncmp(messages, "RECORDING_main", strlen("RECORDING_main")) == 0) // set to bypass mode
	{
    	printk( "[Audio_FM34] PATH_RECORDING_main_MODE\n");
        do_FM34_Table(enFM34_PATH_RECORDING_main);
	}
    else if(strncmp(messages, "RECORDING_ref", strlen("RECORDING_ref")) == 0) // set to bypass mode
	{
    	printk( "[Audio_FM34] PATH_RECORDING_ref_MODE\n");
        do_FM34_Table(enFM34_PATH_RECORDING_ref);
	}
 	else if(strncmp(messages, "voice", strlen("voice")) == 0) // set to bypass mode
	{
    	printk( "[Audio_FM34] PATH_RECORDING_voice_MODE\n");
        do_FM34_Table(enFM34_PATH_RECORDING_voice);
	}
	else if(strncmp(messages, "VR", strlen("VR")) == 0) // set to bypass mode
	{
    	printk( "[Audio_FM34] PATH_RECORDING_VR_MODE\n");
        do_FM34_Table(enFM34_PATH_RECORDING_VR);
	}
	else
	{
		sscanf(messages, "%x %x", &g_Mem_Addr, &reg_val);
		printk(DBGMSK_SND_G2 "fm34_proc_write [0x%x]=[0x%x]\n", g_Mem_Addr, reg_val);

		if (g_Mem_Addr == 0x230F)
		{
			if (reg_val)
			{
				g_Mute = 0;
				switch(after_mute)
				{
					case ONE_MIC:	do_FM34_Table(enFM34_PATH_ONE_MIC);		break;
					case BY_PASS:	do_FM34_Table(enFM34_PATH_BY_PASS);		break;
					case DUAL_MIC:	do_FM34_Table(enFM34_PATH_DUAL_MIC);	break;
					default:	break;
				}
			}
			else
			{
				printk( "[Audio_FM34] MUTE!!!\n");
				g_Mute = 1;
				after_mute = DO_NOTHING;
			}
		}

		if(g_Mem_Addr < FM34_MIN_MEM_ADDR || g_Mem_Addr > FM34_MAX_MEM_ADDR)
		{
			printk(DBGMSK_SND_G2  "[FM34] Invalid MEM addr \n");
			return -EFAULT;
		}
		//TODO: how to verify valid reg_val value?
		/* set the register value */

        ret = fm34_mem_write(g_Mem_Addr, reg_val);
        if (ret != 0) {
            printk("[Audio][FM34] fm34_proc_write fail.\n");
            return 0;
        }
	}
	return len;
}

static struct file_operations fm34_proc_ops = {
	.read = fm34_proc_read,
	.write = fm34_proc_write,
};

static void create_fm34_proc_file(void)
{
	printk("[Audio] create_fm34_proc_file\n");
	// ASUS_BSP 20101003 SamChen AudioPolicy can't write this driver
	// so we extend the permission from 0644 to 0666
	fm34_proc_file = create_proc_entry(FM34_PROC_FILE, 0666, NULL);
	if (fm34_proc_file) {
		fm34_proc_file->proc_fops = &fm34_proc_ops;
	} 
}

static void remove_fm34_proc_file(void)
{
	extern struct proc_dir_entry proc_root;
    printk(DBGMSK_SND_G2  "[Audio] remove_fm34_proc_file\n");	
    remove_proc_entry(FM34_PROC_FILE, &proc_root);
}
#endif //#ifdef	CONFIG_PROC_FS

unsigned long g_fm34on2on_jtime = 0;
void fm34_attach(struct work_struct *work)
{
    int ret;
    mutex_lock(&fm34_mutex);
    printk("[fm34] fm34_attach()++\n");    
    AX_MicroP_setGPIOOutputPin(OUT_uP_FM34_PDN, 1);
    gPIN_FM34_ON = 1;
    AX_MicroP_setGPIOOutputPin(OUT_uP_FM34_RST, 0);
    msleep(60);
    AX_MicroP_setGPIOOutputPin(OUT_uP_FM34_RST, 1);
    msleep(30);
        
    printk("[fm34] calling fm34_patch_DSP+++\n");
    ret = fm34_patch_DSP(); //patch DSP
    printk("[fm34] calling fm34_patch_DSP---\n");

    if (ret == 0 )
    {
	    do_FM34_Table(enFM34_PATH_RECORDING_ref);
	    do_FM34_Table(enFM34_PATH_RECORDING_VR);
	    
	    msleep(200);
        gPIN_FM34_ON = 0;
	    AX_MicroP_setGPIOOutputPin(OUT_uP_FM34_PDN, 0);	
	    //AX_MicroP_setGPIOOutputPin(OUT_uP_FM34_RST, 0);        
        g_fm34on2on_jtime = jiffies;
		    
	    gbFM34attached = 1;   //put it in fm34_patch_DSP();
    }
    printk("[fm34] fm34_attach()--\n");
    mutex_unlock(&fm34_mutex);
    return;
}

#ifdef CONFIG_I2C_STRESS_TEST
#include <linux/i2c_testcase.h>
static int TestFM34(struct i2c_client *apClient)
{
	int lnResult = I2C_TEST_PASS;
    
	i2c_log_in_test_case("TestFM34++\n");
    if(do_reset_FM34(0x8005, MIC_PGAGAIN_12) == 0) {
	    i2c_log_in_test_case("do_test_FM34 fail\n");      	
        lnResult = -1;
    }
    gPIN_FM34_ON = 0;
    AX_MicroP_setGPIOOutputPin(OUT_uP_FM34_PDN, 0);    
    AX_MicroP_setGPIOOutputPin(OUT_uP_FM34_RST, 0);    
    msleep(100);
	i2c_log_in_test_case("TestFM34--\n");
	return lnResult;
};

static struct i2c_test_case_info gFM34TestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(TestFM34),
};
#endif


void init_fm34_when_bl_on(void)
{
    printk("[fm34]init_fm34_when_bl_on()++, gbFM34attached = %d, mutex_is_locked %d\r\n", gbFM34attached, mutex_is_locked(&fm34_mutex));
#ifdef CONFIG_CHARGER_MODE
    if((!g_CHG_mode) && (!gbFM34attached) && (!mutex_is_locked(&fm34_mutex))) {
#else
    if ((!gbFM34attached) && (!mutex_is_locked(&fm34_mutex))) {
#endif
        schedule_work(&fm34_attach_work);
    }
    printk("[fm34]init_fm34_when_bl_on()--\r\n");
}
EXPORT_SYMBOL(init_fm34_when_bl_on);


static int fm34_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
 
    printk("[Audio] fm34_probe\n");
	g_client = client;

    INIT_WORK(&fm34_attach_work, fm34_attach);

#ifdef	CONFIG_PROC_FS
	create_fm34_proc_file();
#endif

#ifdef CONFIG_I2C_STRESS_TEST
	printk("FM34 add test case+\n");
    i2c_add_test_case(client, "FM34Test",ARRAY_AND_SIZE(gFM34TestCaseInfo));
	printk("FM34 add test case-\n");
#endif

	return 0;
}

static int fm34_remove(struct i2c_client *client)
{
	printk(DBGMSK_SND_G2  "[Audio] fm34_remove\n");
#ifdef	CONFIG_PROC_FS
	remove_fm34_proc_file();
#endif

	return 0;
}

#if defined(CONFIG_PM)
static int fm34_suspend(struct device *dev)
{
    if (g_p01State)
    {
        printk("[fm34]fm34_suspend()++\r\n");
        gbFM34attached = 0;
        gPIN_FM34_ON = 0;
        printk("[fm34]fm34_suspend()--\r\n");
    }
    return 0;
}

static int fm34_resume(struct device *dev)
{
    if (g_p01State)
    {
        printk("[fm34]fm34_resume()++\r\n");
        //schedule_work(&fm34_attach_work);
        printk("[fm34]fm34_resume()--\r\n");
    }
        return 0;
}
static const struct dev_pm_ops fm34_pm_ops = {
    .suspend    = fm34_suspend,
    .resume     = fm34_resume,
};
#endif

static const struct i2c_device_id fm34_id[] = {
	{ "fm34", 0 },
	{ }
};

static struct i2c_driver fm34_driver = {
	.driver = {
		.name	= "fm34",
#if defined(CONFIG_PM)
		.pm = &fm34_pm_ops,
#endif
	},
	.id_table 	= fm34_id,
	.probe		= fm34_probe,
	.remove		= fm34_remove,
};

static int fm34_p01_event(struct notifier_block *this, unsigned long event, void *ptr);

static struct notifier_block fm34_p01_notifier = {
        .notifier_call = fm34_p01_event,
        .priority = AUDIO_MP_NOTIFY,
};

extern bool g_voice_call_param;
static int fm34_p01_event(struct notifier_block *this, unsigned long event, void *ptr)
{
    switch(event)
    {
//Bruno++ avoid if no MCLK
/*
        case P01_ADD:
            printk("[Audio][FM34]P01 ADD, gbFM34attached = %d, mutex_is_locked %d\r\n", gbFM34attached, mutex_is_locked(&fm34_mutex));
#ifdef CONFIG_CHARGER_MODE
            if((!g_CHG_mode) && (!gbFM34attached) && (!mutex_is_locked(&fm34_mutex))) {
#else
            if ((!gbFM34attached) && (!mutex_is_locked(&fm34_mutex))) {
#endif
                schedule_work(&fm34_attach_work);
            }
			return NOTIFY_DONE;
*/
//Bruno++ avoid if no MCLK
        case P01_REMOVE:    //<-- handled by microP, cause I2C cmd can't use when HDMI unplugged.
            printk("[Audio][FM34]P01 REMOVE\n");
            gbFM34attached = 0;
            gPIN_FM34_ON = 0;
            g_voice_call_param = 0;
            return NOTIFY_DONE;
        default:
            return NOTIFY_DONE;
    }
};

static int __init fm34_init(void)
{
	printk(  "[Audio] fm34_init\n");
    register_microp_notifier(&fm34_p01_notifier);
	return i2c_add_driver(&fm34_driver);
}

static void __exit fm34_exit(void)
{
	printk(  "[Audio] fm34_exit\n");
	i2c_del_driver(&fm34_driver);
    unregister_microp_notifier(&fm34_p01_notifier);
}

module_init(fm34_init);
module_exit(fm34_exit);

MODULE_DESCRIPTION("FM34 Driver");
MODULE_LICENSE("GPL");

