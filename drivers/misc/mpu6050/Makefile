
# Kernel makefile for motions sensors
#
#

# MPU
#ifdef CONFIG_MPU_SENSORS_MPU3050
#INV_MODULE_NAME := mpu3050
#endif

ifdef CONFIG_MPU_SENSORS_MPU6050A2
INV_MODULE_NAME := mpu6050
endif

ifdef CONFIG_MPU_SENSORS_MPU6050B1
INV_MODULE_NAME := mpu6050
endif

obj-$(CONFIG_INV_SENSORS)	+= $(INV_MODULE_NAME).o

$(INV_MODULE_NAME)-objs += mpuirq_6050.o
$(INV_MODULE_NAME)-objs += slaveirq_6050.o
$(INV_MODULE_NAME)-objs += mpu-dev_6050.o
$(INV_MODULE_NAME)-objs += mlsl-kernel_6050.o
$(INV_MODULE_NAME)-objs += mldl_cfg_6050.o
$(INV_MODULE_NAME)-objs += mldl_print_cfg_6050.o

ifdef CONFIG_MPU_SENSORS_MPU6050A2
$(INV_MODULE_NAME)-objs += accel/mpu6050.o
endif

ifdef CONFIG_MPU_SENSORS_MPU6050B1
$(INV_MODULE_NAME)-objs += accel/mpu6050.o
endif

EXTRA_CFLAGS += -Idrivers/misc/mpu6050
EXTRA_CFLAGS += -D__C99_DESIGNATED_INITIALIZER
EXTRA_CFLAGS += -DINV_CACHE_DMP=1

obj-$(CONFIG_MPU_SENSORS_TIMERIRQ)+= timerirq_6050.o

obj-y			+= accel/
obj-y			+= compass/
obj-y			+= pressure/

