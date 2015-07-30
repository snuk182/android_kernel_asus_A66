#ifndef __ASUS_GLOBAL_H
#define __ASUS_GLOBAL_H

#define ASUS_GLOBAL_TAG		"ADGB"

#define ASUS_GLOBAL_MAJOR_VER		1
#define ASUS_GLOBAL_MINOR_VER		0

#define ASUS_GLOBAL_RUMDUMP_MAGIC	0x28943447
#define ASUS_GLOBAL_MAGIC			0x23638777
struct _asus_global
{
	unsigned int asus_global_magic;
	unsigned int ramdump_enable_magic;
	char* kernel_log_addr;
	unsigned int kernel_log_size;
	char* log_main_addr;
	unsigned int sizeof_log_main;
	char* log_system_addr;
	unsigned int sizeof_log_system;
	char* log_events_addr;
	unsigned int sizeof_log_events;
	char* log_radio_addr;
	unsigned int sizeof_log_radio;
	void* pprev_cpu0;                   /* save cpu0 prev task pointer */
	void* pnext_cpu0;                   /* save cpu0 next task pointer */
    void* pprev_cpu1;                   /* save cpu1 prev task pointer */
    void* pnext_cpu1;                   /* save cpu1 next task pointer */
	unsigned int phycpucontextadd;
	char	reboot_reason[16];
	unsigned long reboot_reason_crc;	/* The CRC for reboot_reason */
	unsigned char tag[4];				/* Tag to detect existed of driver global */
	unsigned short global_major_ver;	/* version of driver global */
	unsigned short global_minor_ver;
	char kernel_version[64];			/* used to store kernel version */
	unsigned int modem_restart_status;
};

#endif // __ASUS_GLOBAL_H

