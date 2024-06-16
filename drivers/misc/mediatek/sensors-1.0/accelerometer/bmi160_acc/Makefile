ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/accelerometer/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include

ifneq (, $(filter %000F_FTM, $(TARGET_PRODUCT)))
    ccflags-y += -D_00F_BUILD_
endif

obj-y	:=  bmi160_acc.o
