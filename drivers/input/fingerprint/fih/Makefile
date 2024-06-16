#
# Makefile for the fingerprint device.
#

# Each configuration option enables a list of files.
#include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include

#ccflags-y += -I$(srctree)/drivers/misc/mediatek/gud/302d/gud/MobiCoreKernelApi/public/
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/gud/302d/gud/MobiCoreKernelApi/include/
#ccflags-y += -I$(srctree)/drivers/spi/trustzone/spi/Tlspi/inc/

ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
subdir-ccflags-y += -I$(srctree)/drivers/spi/trustzone/spi/Tlspi/inc
	include $(srctree)/drivers/misc/mediatek/gud/Makefile.include
endif


obj-$(CONFIG_FIH_FINGERPRINT)	+= fih_finger.o elan_fp_mtk_tee.o
obj-$(CONFIG_FIH_FINGERPRINT)	+= gf_spi_factory.o gf_spi_tee.o
