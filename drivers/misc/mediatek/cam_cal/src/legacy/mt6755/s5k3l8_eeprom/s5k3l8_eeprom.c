/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*
 * Driver for 3L8_CAM_CAL
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "s5k3l8_eeprom.h"
/*#include <asm/system.h>  // for SM*/
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include <linux/proc_fs.h>


/* #define 3L8_CAM_CALGETDLT_DEBUG */
#define S5K3L8_CAM_CAL_DEBUG
#ifdef S5K3L8_CAM_CAL_DEBUG

#define PFX "s5k3l8_eeprom"

#define CAM_CALINF(fmt, arg...)    pr_debug("[%s] " fmt, __func__, ##arg)
#define CAM_CALDB(fmt, arg...)    pr_debug("[%s] " fmt, __func__, ##arg)
#define CAM_CALERR(fmt, arg...)    pr_err("[%s] " fmt, __func__, ##arg)
#else
#define CAM_CALDB(x, ...)
#endif

static DEFINE_SPINLOCK(g_CAM_CALLock); /* for SMP */
#define CAM_CAL_I2C_BUSNUM 2

/*******************************************************************************
 *
 ********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 /* seanlin111208 */
/*******************************************************************************
 *
 ********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_DRV_3L8"
#define CAM_CAL_I2C_GROUP_ID     0
#define CAM_CAL_DEV_MAJOR_NUMBER 226

/* CAM_CAL ID */
#define S5K3L8_DEVICE_ID         0xA0

#define S5K3L8_AWB_SIZE		28
#define S5K3L8_AF_SIZE 		4
#define S5K3L8_LSC_SIZE		1868
#define TOTAL_SIZE 		3343
/*******************************************************************************
 *
 ********************************************************************************/
static struct i2c_board_info kd_cam_cal_dev __initdata = { I2C_BOARD_INFO(CAM_CAL_DRVNAME, S5K3L8_DEVICE_ID >> 1)};

static struct i2c_client *g_pstI2Cclient;

/*81 is used for V4L driver*/
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER, 0);
static struct cdev *g_pCAM_CAL_CharDrv;
/*static spinlock_t g_CAM_CALLock;*/
/*spin_lock(&g_CAM_CALLock);*/
/*spin_unlock(&g_CAM_CALLock);*/

static struct class *CAM_CAL_class;
static atomic_t g_CAM_CALatomic;
/*static DEFINE_SPINLOCK(kdcam_cal_drv_lock);*/
/*spin_lock(&kdcam_cal_drv_lock);*/
/*spin_unlock(&kdcam_cal_drv_lock);*/

static int iReadCAM_CAL_16(u16 a_u2Addr, u8 * a_puBuff, u16 i2c_id)
{
	int  i4RetValue = 0;
	char puReadCmd[2] = {(char)(a_u2Addr >> 8), (char)(a_u2Addr & 0xFF) };

	spin_lock(&g_CAM_CALLock); //for SMP
	g_pstI2Cclient->addr = i2c_id>>1;
	g_pstI2Cclient->timing=100;
	g_pstI2Cclient->addr = g_pstI2Cclient->addr & (I2C_MASK_FLAG );
	spin_unlock(&g_CAM_CALLock); // for SMP

	i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 2);

	CAM_CALINF("%s, addr:%x, i2c_id:%x \n", "iReadCAM_CAL_16",  a_u2Addr, i2c_id) ;

	if (i4RetValue != 2)
	{
		CAM_CALDB("[S5K3L8_CAL] I2C send read address failed!! \n");
		CAM_CALDB("[CAMERA SENSOR] I2C send failed, addr = 0x%x, data = 0x%x !! \n", a_u2Addr,  *a_puBuff );
	}

	udelay(50);

	i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, 1);

	CAM_CALDB("[S5K3L8_CAL][iReadCAM_CAL] Read 0x%x=0x%x \n", a_u2Addr, a_puBuff[0]);
	if (i4RetValue != 1)
	{
		CAM_CALDB("[S5K3L8_CAL] I2C read data failed!! \n");
	}

	udelay(50);

	return 0;
}

u8 S5K3L8_ReadOtp(kal_uint16 address)
{
	u8 readbuff;
	int ret ;

	CAM_CALDB("[S5K3L8_CAL]ENTER address:0x%x \n ",address);
	ret= iReadCAM_CAL_16(address,&readbuff,S5K3L8_DEVICE_ID);
	return readbuff;
}

kal_uint8 S5K3L8_Read_Flag(void)
{
	kal_uint8 otp_flag;

	otp_flag = S5K3L8_ReadOtp(0x0000);

	if(otp_flag == 0x01) {
		CAM_CALDB("[S5K3L8_CAL] EEPROM Flag is one.\n");
		return KAL_TRUE;
	} else {
		CAM_CALERR("[S5K3L8_CAL] EEPROM is null\n");
		return KAL_FALSE;
	}
}

kal_uint8 S5K3L8_Read_module_id(void)
{
	kal_uint8 module_id;
	kal_uint8 otp_flag;

	otp_flag = S5K3L8_Read_Flag();

	if(otp_flag == 0x01) {
		module_id = S5K3L8_ReadOtp(0x0001);
		return module_id;
	} else {
		CAM_CALERR("[S5K3L8_CAL] Module information is null\n");
		return KAL_FALSE;
	}
}

kal_uint8 S5K3L8_Read_time(void)
{
	kal_uint8 year, month, day;
	kal_uint8 otp_flag;

	otp_flag = S5K3L8_Read_Flag();

	if(otp_flag == 0x01) {
		year = S5K3L8_ReadOtp(0x000A);
		month = S5K3L8_ReadOtp(0x000B);
		day = S5K3L8_ReadOtp(0x000C);
		CAM_CALDB("[S5K3L8_CAL] Module year = 0x%x, month = 0x%x, day = 0x%x\n",year,month,day);
		return KAL_TRUE;
	} else {
		CAM_CALERR("[S5K3L8_CAL] OTP is null\n");
		return KAL_FALSE;
	}
}

#if 0
static kal_uint8 S5K3L8_get_module_info(void)
{
	int i = 0;
	int buffersize = 18;
	u8 data[18];
	u8 checksum =0;
	u8 datasum = 0;
	u8 temph, templ;

	kal_uint16 mod_info_add = 0x0001;

	kal_uint8 otp_flag;

	otp_flag = S5K3L8_Read_Flag();
	if(otp_flag == 0x01) {

		for(i = 0;i < buffersize;i++)
		{
			data[i] = S5K3L8_ReadOtp(i + mod_info_add);
			datasum += data[i];
		}
		temph = S5K3L8_ReadOtp(0x0013);
		templ = S5K3L8_ReadOtp(0x0014);
		checksum = ((templ&0xFF)<<8) + (templ&0xFF);
		if(checksum == datasum){
			CAM_CALDB("[S5K3L8_CAL] moudle infomation checksum OK!\n");
			return KAL_TRUE;
		} else {
			CAM_CALERR("[S5K3L8_CAL] moudle infomation checksum Failed!\n");
			return KAL_FALSE;
		}
	} else {
		CAM_CALERR("[S5K3L8_CAL] OTP is null\n");
		return KAL_FALSE;
	}
}
#endif

static kal_uint8 S5K3L8_Read_AWB_Otp(kal_uint8 address,unsigned char *iBuffer,unsigned int buffersize)
{
	int i = 0;
	u8 checksum =0;
	u8 datasum = 0;
	u8 temph, templ;
	kal_uint16 AWBaddr = 0x0015;
	kal_uint8 otp_flag;

	otp_flag = S5K3L8_Read_Flag();

	if(buffersize != S5K3L8_AWB_SIZE)
	{
		CAM_CALDB("[S5K3L8_CAL]wrong AWBSIZE:0x%x \n ",buffersize);
		return KAL_FALSE;
	}

	if(otp_flag == 0x01) {
		for(i = 0;i < buffersize;i++)
		{
			iBuffer[i] = S5K3L8_ReadOtp(i + AWBaddr);
			datasum += iBuffer[i];
		}
		temph = S5K3L8_ReadOtp(0x0031);
		templ = S5K3L8_ReadOtp(0x0032);

		checksum = ((templ&0xFF)<<8) + (templ&0xFF);

		if(checksum == datasum){
			CAM_CALDB("[S5K3L8_CAL] AWB checksum OK!\n");
			return KAL_TRUE;
		} else {
			CAM_CALERR("[S5K3L8_CAL] AWB checksum Failed!\n");
			return KAL_FALSE;
		}
	} else {
		CAM_CALERR("[S5K3L8_CAL] OTP is null\n");
		return KAL_FALSE;
	}
}

static kal_uint8 S5K3L8_Read_AF_Otp(kal_uint8 address,unsigned char *iBuffer,unsigned int buffersize)
{
	int i = 0;
	int j = 0;
	kal_uint16 AFaddr = 0x0782;
	u8 af_direction;
	u8 checksum =0;
	u8 datasum = 0;
	u8 temph, templ;
	u8 reserved_data[8];
	kal_uint8 otp_flag;

	otp_flag = S5K3L8_Read_Flag();
	if(buffersize != S5K3L8_AF_SIZE)
	{
		CAM_CALDB("[S5K3L8_CAL]wrong AFSIZE:0x%x\n ",buffersize);
		return KAL_FALSE;
	}

	af_direction = S5K3L8_ReadOtp(AFaddr -1);
	CAM_CALDB("[S5K3L8_CAL]read af calibration direction:0x%x\n",af_direction);

	if(otp_flag == 0x01) {
		for(i = 0;i < buffersize;i++)
		{
			iBuffer[i] = S5K3L8_ReadOtp(i + AFaddr);
			datasum += iBuffer[i];
		}
		for(j = 0; j < 8;j++){
			reserved_data[j] = S5K3L8_ReadOtp(j + 0x0786);
			datasum += reserved_data[j];
		}

		datasum += af_direction;
		temph = S5K3L8_ReadOtp(0x078E);
		templ = S5K3L8_ReadOtp(0x078F);

		checksum = ((temph&0xFF)<<8) + (templ&0xFF);

		if(checksum == datasum){
			CAM_CALDB("[S5K3L8_CAL] AF checksum OK!\n");
			return KAL_TRUE;
		} else {
			CAM_CALERR("[S5K3L8_CAL] AF checksum Failed!\n");
			return KAL_FALSE;
		}
	} else {
		CAM_CALERR("[S5K3L8_CAL] OTP is null\n");
		return KAL_FALSE;
	}
}

static kal_uint8 S5K3L8_Read_LSC_Otp(kal_uint8 address,unsigned char *iBuffer,unsigned int buffersize)
{
	int i = 0;
	kal_uint16 LSCaddr = 0x0033;

	u8 checksum =0;
	u8 datasum = 0;
	u8 temph, templ;
	kal_uint8 otp_flag;

	otp_flag = S5K3L8_Read_Flag();
	if(buffersize != S5K3L8_LSC_SIZE)
	{
		CAM_CALDB("[S5K3L8_CAL]wrong LSCSIZE:0x%x\n ",buffersize);
		return KAL_FALSE;
	}

	if(otp_flag == 0x01) {
		for(i = 0;i < buffersize;i++)
		{
			iBuffer[i] = S5K3L8_ReadOtp(i + LSCaddr);
			datasum += iBuffer[i];
		}

		temph = S5K3L8_ReadOtp(0x077F);
		templ = S5K3L8_ReadOtp(0x0780);

		checksum = ((templ&0xFF)<<8) + (templ&0xFF);

		if(checksum == datasum){
			CAM_CALDB("[S5K3L8_CAL] LSC checksum OK!\n");
			return KAL_TRUE;
		} else {
			CAM_CALERR("[S5K3L8_CAL] LSC checksum Failed!\n");
			return KAL_FALSE;
		}
	} else {
		CAM_CALERR("[S5K3L8_CAL] OTP is null\n");
		return KAL_FALSE;
	}
}
#if 0
static kal_uint8 S5K3L8_total_checksum(void)
{
	int i = 0;
	u8 data[TOTAL_SIZE];
	u8 checksum =0;
	u8 datasum = 0;
	u8 temph, templ;
	kal_uint16 first_add = 0x0001;
	kal_uint8 otp_flag;

	otp_flag = S5K3L8_Read_Flag();
	if(otp_flag == 0x01) {
		for(i = 0;i < TOTAL_SIZE;i++)
		{
			data[i] = S5K3L8_ReadOtp(i + first_add);
			datasum += data[i];
		}
		datasum += otp_flag;
		temph = S5K3L8_ReadOtp(0x0D10);
		templ = S5K3L8_ReadOtp(0x0D11);
		checksum = ((temph&0xFF)<<8) + (templ&0xFF);
		if(checksum == datasum){
			CAM_CALDB("[S5K3L8_CAL] total size checksum OK!\n");
			return KAL_TRUE;
		} else {
			CAM_CALERR("[S5K3L8_CAL] total size Failed!\n");
			return KAL_FALSE;
		}
	} else {
		CAM_CALERR("[S5K3L8_CAL] OTP is null\n");
		return KAL_FALSE;
	}
}
#endif
//Burst Write Data
static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
	return KAL_FALSE;
}

int iReadData_3L8(kal_uint16 ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
	if(ui4_length ==1)
	{
		S5K3L8_ReadOtp(ui4_offset);
	}
	else if(ui4_length == S5K3L8_AWB_SIZE)
	{
		S5K3L8_Read_AWB_Otp(ui4_offset, pinputdata, ui4_length);
	}
	else if(ui4_length == S5K3L8_AF_SIZE)
	{
		S5K3L8_Read_AF_Otp(ui4_offset, pinputdata, ui4_length);
	}
	else if(ui4_length == S5K3L8_LSC_SIZE)
	{
		S5K3L8_Read_LSC_Otp(ui4_offset, pinputdata, ui4_length);
	}

	CAM_CALDB(" [S5K3L8_CAL]ui4_length = %d,ui4_offset =%d\n ",ui4_length,ui4_offset);
	return 0;
}

#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
		COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
		stCAM_CAL_INFO_STRUCT __user *data)
{
	compat_uptr_t p;
	compat_uint_t i;
	int err;

	err = get_user(i, &data->u4Offset);
	err |= put_user(i, &data32->u4Offset);
	err |= get_user(i, &data->u4Length);
	err |= put_user(i, &data32->u4Length);
	/* Assume pointer is not change */
#if 1
	err |= get_user(p, (compat_uptr_t *)&data->pu1Params);
	err |= put_user(p, &data32->pu1Params);
#endif
	return err;
}
static int compat_get_cal_info_struct(
		COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
		stCAM_CAL_INFO_STRUCT __user *data)
{
	compat_uptr_t p;
	compat_uint_t i;
	int err;

	err = get_user(i, &data32->u4Offset);
	err |= put_user(i, &data->u4Offset);
	err |= get_user(i, &data32->u4Length);
	err |= put_user(i, &data->u4Length);
	err |= get_user(p, &data32->pu1Params);
	err |= put_user(compat_ptr(p), &data->pu1Params);

	return err;
}

static long s5k3l8_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;
	COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
	stCAM_CAL_INFO_STRUCT __user *data;
	int err;

	CAM_CALDB("[CAMERA SENSOR] s5k3l8_Ioctl_Compat,%p %p %x ioc size %d\n", filp->f_op ,
			filp->f_op->unlocked_ioctl, cmd, _IOC_SIZE(cmd));

	if (!filp->f_op || !filp->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
		case COMPAT_CAM_CALIOC_G_READ: {
			data32 = compat_ptr(arg);
			data = compat_alloc_user_space(sizeof(*data));
			if (data == NULL)
				return -EFAULT;
			err = compat_get_cal_info_struct(data32, data);
			if (err)
				return err;
			ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ, (unsigned long)data);
			err = compat_put_cal_info_struct(data32, data);
			if (err != 0)
			       CAM_CALERR("[3L8_CAM_CAL] compat_put_acdk_sensor_getinfo_struct failed\n");
		       return ret;
		       }
		default:
			return -ENOIOCTLCMD;
	}
}
#endif


/*******************************************************************************
 *
 ********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode *a_pstInode,
		struct file *a_pstFile,
		unsigned int a_u4Command,
		unsigned long a_u4Param)
#else
static long CAM_CAL_Ioctl(
		struct file *file,
		unsigned int a_u4Command,
		unsigned long a_u4Param
		)
#endif
{
	int i4RetValue = 0;
	u8 *pBuff = NULL;
	u8 *pu1Params = NULL;
	stCAM_CAL_INFO_STRUCT *ptempbuf = NULL;/*LukeHu++160201=Fix Code Defect.*/

	CAM_CALDB("[3L8_CAM_CAL] ioctl\n");

#ifdef CAM_CALGETDLT_DEBUG
	struct timeval ktv1, ktv2;
	unsigned long TimeIntervalUS;
#endif

	/*if (_IOC_NONE == _IOC_DIR(a_u4Command)) {
	  } else {*/
	if (_IOC_NONE != _IOC_DIR(a_u4Command)) {
		pBuff = kmalloc(sizeof(stCAM_CAL_INFO_STRUCT), GFP_KERNEL);

		if (NULL == pBuff) {
			CAM_CALDB(" ioctl allocate mem failed\n");
			return -ENOMEM;
		}

		if (_IOC_WRITE & _IOC_DIR(a_u4Command)) {
			if (copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT))) {
				/*get input structure address*/
				kfree(pBuff);
				CAM_CALDB("[3L8_CAM_CAL] ioctl copy from user failed\n");
				return -EFAULT;
			}
		}
	}

	ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
	if(NULL == pBuff)
		return -ENOMEM;
	else
		pu1Params = kmalloc(ptempbuf->u4Length, GFP_KERNEL);
	if (NULL == pu1Params) {
		kfree(pBuff);
		CAM_CALDB("[S5K3L8_CAM_CAL] ioctl allocate mem failed\n");
		return -ENOMEM;
	}
	CAM_CALDB(" init Working buffer address 0x%p  command is 0x%x\n", pu1Params, a_u4Command);

	if (ptempbuf->u4Length > 65535) {
		kfree(pBuff);
		kfree(pu1Params);
		CAM_CALDB("[3L8_CAM_CAL] ptempbuf->u4Length=%d is so large!\n", ptempbuf->u4Length);
		return -EFAULT;
	}

	if (copy_from_user((u8 *)pu1Params, (u8 *)ptempbuf->pu1Params, ptempbuf->u4Length)) {
		kfree(pBuff);
		kfree(pu1Params);
		CAM_CALDB("[3L8_CAM_CAL] ioctl copy from user failed\n");
		return -EFAULT;
	}

	switch (a_u4Command) {
		case CAM_CALIOC_S_WRITE:
			CAM_CALDB("[3L8_CAM_CAL] Write CMD\n");
#ifdef CAM_CALGETDLT_DEBUG
			do_gettimeofday(&ktv1);
#endif
			i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
#ifdef CAM_CALGETDLT_DEBUG
			do_gettimeofday(&ktv2);
			if (ktv2.tv_sec > ktv1.tv_sec)
				TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
			else
				TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;

			CAM_CALDB("Write data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
#endif
			break;
		case CAM_CALIOC_G_READ:
			CAM_CALDB("[S5K3L8_CAM_CAL] Read CMD\n");

#ifdef CAM_CALGETDLT_DEBUG
			do_gettimeofday(&ktv1);
#endif
			CAM_CALDB("[3L8_CAM_CAL] offset %d\n", ptempbuf->u4Offset);
			CAM_CALDB("[3L8_CAM_CAL] length %d\n", ptempbuf->u4Length);
			i4RetValue = iReadData_3L8((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
#ifdef CAM_CALGETDLT_DEBUG
			do_gettimeofday(&ktv2);
			if (ktv2.tv_sec > ktv1.tv_sec)
				TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
			else
				TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;

			CAM_CALDB("Read data %d bytes take %lu us\n", ptempbuf->u4Length, TimeIntervalUS);
#endif

			break;
		default:
			CAM_CALDB("[3L8_CAM_CAL] No CMD\n");
			i4RetValue = -EPERM;
			break;
	}

	if (_IOC_READ & _IOC_DIR(a_u4Command)) {
		/*copy data to user space buffer, keep other input paremeter unchange.*/
		CAM_CALDB("[3L8_CAM_CAL] to user length %d\n", ptempbuf->u4Length);
		CAM_CALDB("[3L8_CAM_CAL] to user  Working buffer address 0x%p\n", pu1Params);

		if (ptempbuf->u4Length > 65535) {
			kfree(pBuff);
			kfree(pu1Params);
			CAM_CALDB("[3L8_CAM_CAL] ptempbuf->u4Length is so large\n");
			return -EFAULT;
		}

		if (copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pu1Params , ptempbuf->u4Length)) {
			kfree(pBuff);
			kfree(pu1Params);
			CAM_CALDB("[3L8_CAM_CAL] ioctl copy to user failed\n");
			return -EFAULT;
		}
	}

	kfree(pBuff);
	kfree(pu1Params);
	return i4RetValue;
}


static u32 g_u4Opened;
/*#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.*/
static int CAM_CAL_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
	CAM_CALDB("[3L8_CAM_CAL] CAM_CAL_Open\n");
	spin_lock(&g_CAM_CALLock);
	if (g_u4Opened) {
		spin_unlock(&g_CAM_CALLock);
		CAM_CALDB("[3L8_CAM_CAL] Opened, return -EBUSY\n");
		return -EBUSY;
	} else {
		g_u4Opened = 1;
		atomic_set(&g_CAM_CALatomic, 0);
	}
	spin_unlock(&g_CAM_CALLock);
	mdelay(2);
	return 0;
}

/*Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.*/
static int CAM_CAL_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	spin_lock(&g_CAM_CALLock);

	g_u4Opened = 0;

	atomic_set(&g_CAM_CALatomic, 0);

	spin_unlock(&g_CAM_CALLock);

	return 0;
}

static const struct file_operations g_stCAM_CAL_fops = {
	.owner = THIS_MODULE,
	.open = CAM_CAL_Open,
	.release = CAM_CAL_Release,
	/*.ioctl = CAM_CAL_Ioctl*/
#ifdef CONFIG_COMPAT
	.compat_ioctl = s5k3l8_Ioctl_Compat,
#endif
	.unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
static inline int RegisterCAM_CALCharDrv(void)
{
	struct device *CAM_CAL_device = NULL;

	CAM_CALDB("[3L8_CAM_CAL] RegisterCAM_CALCharDrv Start\n");

#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
	if (alloc_chrdev_region(&g_CAM_CALdevno, 0, 1, CAM_CAL_DRVNAME)) {
		CAM_CALDB("[3L8_CAM_CAL] Allocate device no failed\n");

		return -EAGAIN;
	}
#else
	if (register_chrdev_region(g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME)) {
		CAM_CALDB("[3L8_CAM_CAL] Register device no failed\n");

		return -EAGAIN;
	}
#endif

	/*Allocate driver*/
	g_pCAM_CAL_CharDrv = cdev_alloc();

	if (NULL == g_pCAM_CAL_CharDrv) {
		unregister_chrdev_region(g_CAM_CALdevno, 1);

		CAM_CALDB("[3L8_CAM_CAL] Allocate mem for kobject failed\n");

		return -ENOMEM;
	}

	/*Attatch file operation.*/
	cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

	g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

	/*Add to system*/
	if (cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1)) {
		CAM_CALDB("[3L8_CAM_CAL] Attatch file operation failed\n");

		unregister_chrdev_region(g_CAM_CALdevno, 1);

		return -EAGAIN;
	}

	CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv_3L8");

	if (IS_ERR(CAM_CAL_class)) {
		int ret = PTR_ERR(CAM_CAL_class);

		CAM_CALDB("[3L8_CAM_CAL] Unable to create class, err = %d\n", ret);
		return ret;
	}
	CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);
	CAM_CALDB("[3L8_CAM_CAL] RegisterCAM_CALCharDrv End\n");

	return 0;
}

static inline void UnregisterCAM_CALCharDrv(void)
{
	/*Release char driver*/
	cdev_del(g_pCAM_CAL_CharDrv);

	unregister_chrdev_region(g_CAM_CALdevno, 1);

	device_destroy(CAM_CAL_class, g_CAM_CALdevno);
	class_destroy(CAM_CAL_class);
}


/* //////////////////////////////////////////////////////////////////// */
#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int CAM_CAL_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int CAM_CAL_i2c_remove(struct i2c_client *);

static const struct i2c_device_id CAM_CAL_i2c_id[] = {{CAM_CAL_DRVNAME, 0}, {} };



static struct i2c_driver CAM_CAL_i2c_driver = {
	.probe = CAM_CAL_i2c_probe,
	.remove = CAM_CAL_i2c_remove,
	/*   .detect = CAM_CAL_i2c_detect,*/
	.driver.name = CAM_CAL_DRVNAME,
	.id_table = CAM_CAL_i2c_id,
};

#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strcpy(info->type, CAM_CAL_DRVNAME);
	return 0;
}
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i4RetValue = 0;

	CAM_CALDB("[3L8_CAM_CAL] CAM_CAL_i2c_probe Start!\n");
	/*    spin_lock_init(&g_CAM_CALLock);*/

	/*get sensor i2c client*/
	spin_lock(&g_CAM_CALLock); /*for SMP*/
	g_pstI2Cclient = client;
	g_pstI2Cclient->addr = S5K3L8_DEVICE_ID >> 1;
	spin_unlock(&g_CAM_CALLock); /* for SMP*/

	CAM_CALDB("[3L8_CAM_CAL] g_pstI2Cclient->addr = 0x%x\n", g_pstI2Cclient->addr);
	/*Register char driver*/
	i4RetValue = RegisterCAM_CALCharDrv();

	if (i4RetValue) {
		CAM_CALDB("[3L8_CAM_CAL] register char device failed!\n");
		return i4RetValue;
	}

	CAM_CALDB("[3L8_CAM_CAL] CAM_CAL_i2c_probe End!\n");
	return 0;
}

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static int CAM_CAL_probe(struct platform_device *pdev)
{
	CAM_CALDB("[3L8_CAM_CAL] CAM_CAL_probe start!\n");
	return i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
	i2c_del_driver(&CAM_CAL_i2c_driver);
	return 0;
}
static ssize_t  eeprom_proc_write(struct file *file, const char *buffer, size_t count,
		loff_t *data)
{
	return count;
}
static ssize_t  eeprom_proc_read(struct file *file, char __user *data, size_t len, loff_t *ppos)
{
	unsigned int inf = 0;
	unsigned int macro = 0;
	ssize_t length = 0;
	char tmpbuf[30];
	unsigned long pos = *ppos;
	int otp_flag = 0;
	int i;
	unsigned char iBuffer[4];

	kal_uint16 AFaddr = 0x0782;
	otp_flag = S5K3L8_Read_Flag();

	if(otp_flag == 0x01) {
		for(i = 0;i < S5K3L8_AF_SIZE;i++){
			iBuffer[i] = S5K3L8_ReadOtp(i + AFaddr);
		}

		inf = ((iBuffer[0]&0xFF)<<8) + (iBuffer[1]&0xFF);
		macro = ((iBuffer[2]&0xFF)<<8) + (iBuffer[3]&0xFF);
	}

	if (pos != 0)
		return  0;

	length = snprintf(tmpbuf,sizeof(tmpbuf),"%d,%d",macro,inf);

	if (copy_to_user(data, tmpbuf, length))
		return  0;
	pos += length;
	*ppos = pos;
	return length;

}

static  struct file_operations eeprom_proc_fops = {
	.read = eeprom_proc_read,
	.write = eeprom_proc_write,
};
/*platform structure*/
static struct platform_driver g_stCAM_CAL_Driver = {
	.probe              = CAM_CAL_probe,
	.remove     = CAM_CAL_remove,
	.driver             = {
		.name   = CAM_CAL_DRVNAME,
		.owner  = THIS_MODULE,
	}
};


static struct platform_device g_stCAM_CAL_Device = {
	.name = CAM_CAL_DRVNAME,
	.id = 0,
	.dev = {
	}
};

static int __init CAM_CAL_i2C_init(void)
{
	CAM_CALDB("[3L8_CAM_CAL] CAM_CAL_i2C_init Start!\n");
	i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
	if (platform_driver_register(&g_stCAM_CAL_Driver)) {
		CAM_CALDB("[3L8_CAM_CAL] failed to register CAM_CAL driver\n");
		return -ENODEV;
	}

	if (platform_device_register(&g_stCAM_CAL_Device)) {
		CAM_CALDB("[3L8_CAM_CAL] failed to register CAM_CAL driver, 2nd time\n");
		return -ENODEV;
	}
	CAM_CALDB("[3L8_CAM_CAL] CAM_CAL_i2C_init End!\n");
	proc_create("driver/eeprom_3l8", 0666, NULL, &eeprom_proc_fops);
	return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);

MODULE_DESCRIPTION("S5K3L8 CAM_CAL driver");
MODULE_AUTHOR("DreamYeh <Dream.Yeh@Mediatek.com>");
