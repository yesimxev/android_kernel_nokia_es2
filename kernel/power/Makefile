
ccflags-$(CONFIG_PM_DEBUG)	:= -DDEBUG

ifeq ($(CONFIG_FIH_DUMP_GPIO2FILE), y)
#GPIO dump to /data/PINdumpxxx.dump
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/${MTK_PLATFORM}/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/gpio/mt6755/
endif

obj-y				+= qos.o
obj-$(CONFIG_PM)		+= main.o
obj-$(CONFIG_VT_CONSOLE_SLEEP)	+= console.o
obj-$(CONFIG_FREEZER)		+= process.o
obj-$(CONFIG_SUSPEND)		+= suspend.o
obj-$(CONFIG_PM_TEST_SUSPEND)	+= suspend_test.o
obj-$(CONFIG_HIBERNATION)	+= hibernate.o snapshot.o swap.o user.o \
				   block_io.o
obj-$(CONFIG_PM_AUTOSLEEP)	+= autosleep.o
obj-$(CONFIG_PM_WAKELOCKS)	+= wakelock.o

obj-$(CONFIG_MAGIC_SYSRQ)	+= poweroff.o

obj-$(CONFIG_SUSPEND)	+= wakeup_reason.o
