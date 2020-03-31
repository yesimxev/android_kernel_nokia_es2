/*
 * SAMSUNG NFC Controller
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 * Author: Woonki Lee <woonki84.lee@samsung.com>
 *         Heejae Kim <heejae12.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Last update: 2016-01-14
 *
 */
#ifdef CONFIG_SEC_NFC_IF_I2C_GPIO
#define CONFIG_SEC_NFC_IF_I2C
#endif

#include <linux/wait.h>
#include <linux/delay.h>

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/nfc/sec_nfc.h>
#include <../../misc/mediatek/base/power/include/mt_clkbuf_ctl.h>	/*  for clock buffer */
#include <linux/regulator/consumer.h>

#ifdef CONFIG_SEC_NFC_CLK_REQ
#include <linux/interrupt.h>
#endif
#include <linux/wakelock.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <../../misc/mediatek/include/mt-plat/mt_boot_common.h>
#include <../../misc/mediatek/include/mt-plat/mt_gpio.h>
#include <linux/kthread.h>

#ifndef CONFIG_SEC_NFC_IF_I2C
struct sec_nfc_i2c_info {};
#define sec_nfc_read			NULL
#define sec_nfc_write			NULL
#define sec_nfc_poll			NULL
#define sec_nfc_i2c_irq_clear(x)

#define SEC_NFC_GET_INFO(dev) platform_get_drvdata(to_platform_device(dev))

#else /* CONFIG_SEC_NFC_IF_I2C */
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/i2c.h>

#define SEC_NFC_DTS_NAME "mediatek,nfc"
#define SEC_NFC_PLATFORM_DTS_NAME "samsung,nfc-sec"
#define NFC_CLIENT_TIMING 400	/* I2C speed */
#define MAX_BUFFER_SIZE 255

#define SEC_NFC_GET_INFO(dev) i2c_get_clientdata(to_i2c_client(dev))
enum sec_nfc_irq {
	SEC_NFC_NONE,
	SEC_NFC_INT,
	SEC_NFC_SKIP,
};

struct sec_nfc_i2c_info {
	struct i2c_client *i2c_dev;
	struct mutex read_mutex;
	enum sec_nfc_irq read_irq;
	wait_queue_head_t read_wait;
	size_t buflen;
	u8 *buf;
};

#endif

struct sec_nfc_info {
	struct miscdevice miscdev;
	struct mutex mutex;
	enum sec_nfc_mode mode;
	struct device *dev;
	struct sec_nfc_platform_data *pdata;
	struct sec_nfc_i2c_info i2c_info;
	struct wake_lock nfc_wake_lock;
#ifdef	CONFIG_SEC_NFC_CLK_REQ
	bool clk_ctl;
	bool clk_state;
#endif
};

/* For DMA */
static char *I2CDMABuf;
static dma_addr_t I2CDMABuf_pa;
static struct regulator *nfc_1v8_ldo; /* For regulator */

/* defined in proc_info.c */
extern char nfc_hw_support[16];
extern struct mutex proc_read_mutex;

static struct pinctrl *gpctrl = NULL;
static struct pinctrl_state *st_ven_inpd = NULL;
static struct pinctrl_state *st_firm_inpd = NULL;
static struct pinctrl_state *st_irq_inpd = NULL;
static struct pinctrl_state *st_clk_req_inpd = NULL;

static int sec_nfc_set_gpio_inpd(void)
{
	pr_info("sec_nfc_set_gpio_inpd enter\n");

	if(gpctrl) {
		/*ven_gpio*/
		if(st_ven_inpd) {
			pinctrl_select_state(gpctrl, st_ven_inpd);
		}

		/*irq_gpio*/
		if(st_irq_inpd) {
			pinctrl_select_state(gpctrl, st_irq_inpd);
		}

		/*firm_gpio*/
		if(st_firm_inpd) {
			pinctrl_select_state(gpctrl, st_firm_inpd);
		}

		/*clk_req_gpio*/
		if(st_clk_req_inpd) {
			pinctrl_select_state(gpctrl, st_clk_req_inpd);
		}
	}
	return 0;
}
int nfc_ldo_enable(void)
{
              int ret = 0;
              int retval = 0;

              /* set voltage with min & max*/
              ret = regulator_set_voltage(nfc_1v8_ldo, 1800000, 1800000);
              if (ret < 0)
                             pr_err("set voltage nfc_1v8_ldo fail, ret = %d\n", ret);
              retval |= ret;

              /* enable regulator */
              ret = regulator_enable(nfc_1v8_ldo);
              if (ret < 0)
                             pr_err("enable regulator nfc_1v8_ldo fail, ret = %d\n", ret);
              retval |= ret;

              return retval;
}

int nfc_ldo_disable(void)
{
              int ret = 0;
              int retval = 0;

              ret = regulator_disable(nfc_1v8_ldo);
              if (ret < 0)
                             pr_err("disable regulator disp_bias_neg fail, ret = %d\n", ret);
              retval |= ret;

              return retval;
}

#ifdef CONFIG_SEC_NFC_IF_I2C
static irqreturn_t sec_nfc_irq_thread_fn(int irq, void *dev_id)
{
	struct sec_nfc_info *info = dev_id;
	struct sec_nfc_platform_data *pdata = info->pdata;

	dev_dbg(info->dev, "[NFC] Read Interrupt is occurred!\n");
        /*
	if(gpio_get_value(pdata->irq) == 0) {
		dev_err(info->dev, "[NFC] Warning,irq-gpio state is low!\n");
		return IRQ_HANDLED;
	}
        */
	mutex_lock(&info->i2c_info.read_mutex);
	/* Skip interrupt during power switching
	 * It is released after first write */
	if (info->i2c_info.read_irq == SEC_NFC_SKIP) {
		dev_dbg(info->dev, "%s: Now power swiching. Skip this IRQ\n", __func__);
		mutex_unlock(&info->i2c_info.read_mutex);
		return IRQ_HANDLED;
	}

	info->i2c_info.read_irq = SEC_NFC_INT;
	mutex_unlock(&info->i2c_info.read_mutex);

	wake_up_interruptible(&info->i2c_info.read_wait);
	wake_lock_timeout(&info->nfc_wake_lock, 2*HZ);

	return IRQ_HANDLED;
}

static ssize_t sec_nfc_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
        struct sec_nfc_i2c_info i2c_info = info->i2c_info;
        struct i2c_client *client = i2c_info.i2c_dev;
	enum sec_nfc_irq irq;
	int ret = 0;

	dev_dbg(info->dev, "%s: info: %p, count: %zu\n", __func__,
		info, count);

	mutex_lock(&info->mutex);

	if (info->mode == SEC_NFC_MODE_OFF) {
		dev_err(info->dev, "sec_nfc_read sec_nfc is not enabled\n");
		ret = -ENODEV;
		goto out;
	}

	mutex_lock(&info->i2c_info.read_mutex);
	irq = info->i2c_info.read_irq;
	mutex_unlock(&info->i2c_info.read_mutex);
	if (irq == SEC_NFC_NONE) {
		if (file->f_flags & O_NONBLOCK) {
			dev_err(info->dev, "%s it is nonblock\n", __func__);
			ret = -EAGAIN;
			goto out;
		}
	}

	/* i2c recv */
	if (count > info->i2c_info.buflen)
		count = info->i2c_info.buflen;

	if (count > SEC_NFC_MSG_MAX_SIZE) {
		dev_err(info->dev, "user required wrong size :%d\n", (int)count);
		ret = -EINVAL;
		goto out;
	}

	mutex_lock(&info->i2c_info.read_mutex);
	memset(info->i2c_info.buf, 0, count);
        client->addr = client->addr & I2C_MASK_FLAG;
        client->ext_flag |= I2C_DMA_FLAG;
	ret =
	    i2c_master_recv(client,
			    (unsigned char *)(uintptr_t) I2CDMABuf_pa,
			    count);
        /*
	ret = i2c_master_recv(info->i2c_info.i2c_dev, info->i2c_info.buf, count);
        */
	dev_dbg(info->dev, "[NFC] recv size : %d\n", ret);

	if (ret == -EREMOTEIO) {
		ret = -ERESTART;
		goto read_error;
	} else if (ret != count) {
		dev_err(info->dev, "[NFC] read failed: return: %d count: %d\n",
			ret, (int)count);
		//ret = -EREMOTEIO;
		goto read_error;
	}

	info->i2c_info.read_irq = SEC_NFC_NONE;
	mutex_unlock(&info->i2c_info.read_mutex);

	//if (copy_to_user(buf, info->i2c_info.buf, ret)) {
	if (copy_to_user(buf, I2CDMABuf, ret)) {
		dev_err(info->dev, "copy failed to user\n");
		ret = -EFAULT;
	}

	goto out;

read_error:
	info->i2c_info.read_irq = SEC_NFC_NONE;
	mutex_unlock(&info->i2c_info.read_mutex);
out:
	mutex_unlock(&info->mutex);

	return ret;
}

static ssize_t sec_nfc_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
        struct sec_nfc_i2c_info i2c_info = info->i2c_info;
        struct i2c_client *client = i2c_info.i2c_dev;
	int ret = 0;
        int i, idx = 0, len_left, to_write;

	dev_dbg(info->dev, "%s: info: %p, count %zu\n", __func__,
		info, count);

	mutex_lock(&info->mutex);

	if (info->mode == SEC_NFC_MODE_OFF) {
		dev_err(info->dev, "sec_nfc_write sec_nfc is not enabled\n");
		ret = -ENODEV;
		goto out;
	}

        if (count > SEC_NFC_MSG_MAX_SIZE)
		count = SEC_NFC_MSG_MAX_SIZE;
	
	mutex_lock(&info->i2c_info.read_mutex);
        len_left = count;
        while (len_left > 0) {
            if (len_left > MAX_BUFFER_SIZE)
                to_write = MAX_BUFFER_SIZE;
            else
                to_write = len_left;

	    //if (copy_from_user(info->i2c_info.buf, buf, count)) {
	    if (copy_from_user(I2CDMABuf, buf + (count - len_left), to_write)) {
	        dev_err(info->dev, "copy failed from user\n");
		ret = -EFAULT;
	        mutex_unlock(&info->i2c_info.read_mutex);
		goto out;
	    }
            client->addr = client->addr & I2C_MASK_FLAG;
            client->ext_flag |= I2C_DMA_FLAG;
	    ret = i2c_master_send(client,
			    (unsigned char *)(uintptr_t) I2CDMABuf_pa,
			    to_write);
            if (ret != to_write) {
                dev_err(info->dev, "sec_nfc %s : i2c_master_send %d returned %d\n",
                        __func__, to_write, ret);
                ret = -EIO;
	        mutex_unlock(&info->i2c_info.read_mutex);
		goto out;
            }
            len_left -= ret;
        }

	if (info->i2c_info.read_irq == SEC_NFC_SKIP)
	    info->i2c_info.read_irq = SEC_NFC_NONE;
	mutex_unlock(&info->i2c_info.read_mutex);

        ret = count - len_left;
        
        dev_dbg(info->dev, "[NFC] write done [%d]\n", ret);
out:
	mutex_unlock(&info->mutex);

	return ret;
}

static unsigned int sec_nfc_poll(struct file *file, poll_table *wait)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
	enum sec_nfc_irq irq;

	int ret = 0;

	dev_dbg(info->dev, "%s: info: %p\n", __func__, info);

	mutex_lock(&info->mutex);

	if (info->mode == SEC_NFC_MODE_OFF) {
		dev_err(info->dev, "sec_nfc_poll sec_nfc is not enabled\n");
		ret = -ENODEV;
		goto out;
	}

	poll_wait(file, &info->i2c_info.read_wait, wait);

	mutex_lock(&info->i2c_info.read_mutex);
	irq = info->i2c_info.read_irq;
	if (irq == SEC_NFC_INT)
		ret = (POLLIN | POLLRDNORM);
	mutex_unlock(&info->i2c_info.read_mutex);

out:
	mutex_unlock(&info->mutex);

	return ret;
}

void sec_nfc_i2c_irq_clear(struct sec_nfc_info *info)
{
	/* clear interrupt. Interrupt will be occured at power off */
	mutex_lock(&info->i2c_info.read_mutex);
	info->i2c_info.read_irq = SEC_NFC_NONE;
	mutex_unlock(&info->i2c_info.read_mutex);
}

static int I2C_DMA_alloc(struct i2c_client *client)
{
#ifdef CONFIG_64BIT
    I2CDMABuf =
        (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE,
				       &I2CDMABuf_pa, GFP_KERNEL);
#else
    I2CDMABuf =
        (char *)dma_alloc_coherent(NULL, MAX_BUFFER_SIZE,
				       &I2CDMABuf_pa, GFP_KERNEL);
#endif
    if (I2CDMABuf == NULL) {
        return -1;
    }
    return 0;
}

int sec_nfc_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sec_nfc_info *info = dev_get_drvdata(dev);
	struct sec_nfc_platform_data *pdata = info->pdata;
	int ret;

        dev_dbg(info->dev, "%s: start: %p\n", __func__, info);

        ret = I2C_DMA_alloc(client);
        if (ret == -1) {
		dev_err(dev,
			"failed to allocate DMA\n");
		return -ENOMEM;
        }

	info->i2c_info.buflen = SEC_NFC_MAX_BUFFER_SIZE;
	info->i2c_info.buf = kzalloc(SEC_NFC_MAX_BUFFER_SIZE, GFP_KERNEL);
	if (!info->i2c_info.buf) {
		dev_err(dev,
			"failed to allocate memory for sec_nfc_info->buf\n");
		return -ENOMEM;
	}
	info->i2c_info.i2c_dev = client;
	info->i2c_info.read_irq = SEC_NFC_NONE;
	mutex_init(&info->i2c_info.read_mutex);
	init_waitqueue_head(&info->i2c_info.read_wait);
	i2c_set_clientdata(client, info);

	ret = gpio_request(pdata->irq, "nfc_int");
	if (ret) {
		dev_err(dev, "GPIO request is failed to register IRQ\n");
                goto err_irq_req;
	}
        client->timing = NFC_CLIENT_TIMING;
	client->irq = gpio_to_irq(pdata->irq);
	pr_info("%s: push interrupt no = %d\n", __func__, client->irq);

	ret = request_threaded_irq(client->irq, NULL, sec_nfc_irq_thread_fn,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, SEC_NFC_DRIVER_NAME,
			info);
	if (ret < 0) {
		dev_err(dev, "failed to register IRQ handler\n");
		kfree(info->i2c_info.buf);
		return ret;
	}

	dev_dbg(info->dev, "%s: success: %p\n", __func__, info);
	return 0;

err_irq_req:
        return ret;
}

void sec_nfc_i2c_remove(struct device *dev)
{
	struct sec_nfc_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->i2c_info.i2c_dev;
	struct sec_nfc_platform_data *pdata = info->pdata;
	free_irq(client->irq, info);
	gpio_free(pdata->irq);
}
#endif /* CONFIG_SEC_NFC_IF_I2C */

#ifdef	CONFIG_SEC_NFC_CLK_REQ
static irqreturn_t sec_nfc_clk_irq_thread(int irq, void *dev_id)
{
	struct sec_nfc_info *info = dev_id;
	struct sec_nfc_platform_data *pdata = info->pdata;
	bool value;

    dev_dbg(info->dev, "[NFC]Clock Interrupt is occurred!\n");
    value = gpio_get_value(pdata->clk_req) > 0 ? true : false;

	gpio_set_value(pdata->clk, value);

	info->clk_state = value;

	return IRQ_HANDLED;
}

void sec_nfc_clk_ctl_enable(struct sec_nfc_info *info)
{
	struct sec_nfc_platform_data *pdata = info->pdata;
	unsigned int irq = gpio_to_irq(pdata->clk_req);
	int ret;

	if (info->clk_ctl)
		return;

	info->clk_state = false;
	ret = request_threaded_irq(irq, NULL, sec_nfc_clk_irq_thread,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			SEC_NFC_DRIVER_NAME, info);
	if (ret < 0) {
		dev_err(info->dev, "failed to register CLK REQ IRQ handler\n");
	}
	info->clk_ctl = true;
}
void sec_nfc_clk_ctl_disable(struct sec_nfc_info *info)
{
	struct sec_nfc_platform_data *pdata = info->pdata;
	unsigned int irq = gpio_to_irq(pdata->clk_req);
	bool value;

        if (!info->clk_ctl)
		return;

	free_irq(irq, info);
	if (info->clk_state)
	{
        value = gpio_get_value(pdata->clk_req) > 0 ? true : false;
        gpio_set_value(pdata->clk, value);
	}
	info->clk_state = false;
	info->clk_ctl = false;
}
#else
#define sec_nfc_clk_ctl_enable(x)
#define sec_nfc_clk_ctl_disable(x)
#endif /* CONFIG_SEC_NFC_CLK_REQ */

static void sec_nfc_set_mode(struct sec_nfc_info *info,
					enum sec_nfc_mode mode)
{
	struct sec_nfc_platform_data *pdata = info->pdata;

	/* intfo lock is aleady gotten before calling this function */
	if (info->mode == mode) {
		dev_dbg(info->dev, "Power mode is already %d", mode);
		return;
	}
	info->mode = mode;

#ifdef CONFIG_SEC_NFC_IF_I2C
	/* Skip interrupt during power switching
	 * It is released after first write */
	mutex_lock(&info->i2c_info.read_mutex);
	info->i2c_info.read_irq = SEC_NFC_SKIP;
	mutex_unlock(&info->i2c_info.read_mutex);
#endif

	gpio_set_value(pdata->ven, SEC_NFC_PW_OFF);
	if (pdata->firm) gpio_set_value(pdata->firm, SEC_NFC_FW_OFF);

	if (mode == SEC_NFC_MODE_BOOTLOADER)
		if (pdata->firm) gpio_set_value(pdata->firm, SEC_NFC_FW_ON);

	if (mode != SEC_NFC_MODE_OFF)
	{
		msleep(SEC_NFC_VEN_WAIT_TIME);
		gpio_set_value(pdata->ven, SEC_NFC_PW_ON);
		sec_nfc_clk_ctl_enable(info);
#ifdef CONFIG_SEC_NFC_IF_I2C
		enable_irq_wake(info->i2c_info.i2c_dev->irq);
#endif
		msleep(SEC_NFC_VEN_WAIT_TIME/2);
	} else {
		sec_nfc_clk_ctl_disable(info);
#ifdef CONFIG_SEC_NFC_IF_I2C
		disable_irq_wake(info->i2c_info.i2c_dev->irq);
#endif
	}

	if(wake_lock_active(&info->nfc_wake_lock))
		wake_unlock(&info->nfc_wake_lock);

	dev_dbg(info->dev, "Power mode is : %d\n", mode);
}

static long sec_nfc_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
	struct sec_nfc_platform_data *pdata = info->pdata;
	unsigned int new = (unsigned int)arg;
	int ret = 0;

	dev_dbg(info->dev, "%s: info: %p, cmd: 0x%x\n",
			__func__, info, cmd);

	mutex_lock(&info->mutex);

	switch (cmd) {
	case SEC_NFC_SET_MODE:
		dev_dbg(info->dev, "%s: SEC_NFC_SET_MODE\n", __func__);

		if (info->mode == new)
			break;

		if (new >= SEC_NFC_MODE_COUNT) {
			dev_err(info->dev, "wrong mode (%d)\n", new);
			ret = -EFAULT;
			break;
		}
		sec_nfc_set_mode(info, new);

		break;

#if defined(CONFIG_SEC_NFC_PRODUCT_N3)
	case SEC_NFC_SLEEP:
	case SEC_NFC_WAKEUP:
		break;

#elif defined(CONFIG_SEC_NFC_PRODUCT_N5) || defined(CONFIG_SEC_NFC_PRODUCT_N7)
	case SEC_NFC_SLEEP:
		if (info->mode != SEC_NFC_MODE_BOOTLOADER) {
			if(wake_lock_active(&info->nfc_wake_lock))
				wake_unlock(&info->nfc_wake_lock);
			gpio_set_value(pdata->wake, SEC_NFC_WAKE_SLEEP);
		}
		break;

	case SEC_NFC_WAKEUP:
		if (info->mode != SEC_NFC_MODE_BOOTLOADER) {
			gpio_set_value(pdata->wake, SEC_NFC_WAKE_UP);
			if(!wake_lock_active(&info->nfc_wake_lock))
				wake_lock(&info->nfc_wake_lock);
		}
		break;
#endif

	case SEC_NFC_POWER_DOWN:
		if(get_boot_mode() != KERNEL_POWER_OFF_CHARGING_BOOT) {
			nfc_ldo_disable();
		}
		break;
	default:
		dev_err(info->dev, "Unknow ioctl 0x%x\n", cmd);
		ret = -ENOIOCTLCMD;
		break;
	}

	mutex_unlock(&info->mutex);

	return ret;
}

static int sec_nfc_open(struct inode *inode, struct file *file)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
	int ret = 0;

	dev_dbg(info->dev, "%s: info : %p" , __func__, info);

        //nfc_ldo_enable();
	mutex_lock(&info->mutex);
	if (info->mode != SEC_NFC_MODE_OFF) {
		dev_err(info->dev, "sec_nfc is busy\n");
		ret = -EBUSY;
		goto out;
	}

	sec_nfc_set_mode(info, SEC_NFC_MODE_OFF);

out:
	mutex_unlock(&info->mutex);
	return ret;
}

static int sec_nfc_close(struct inode *inode, struct file *file)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);

	dev_dbg(info->dev, "%s: info : %p" , __func__, info);

	mutex_lock(&info->mutex);
	sec_nfc_set_mode(info, SEC_NFC_MODE_OFF);
	mutex_unlock(&info->mutex);
        
        //nfc_ldo_disable();
	return 0;
}

static const struct file_operations sec_nfc_fops = {
	.owner		= THIS_MODULE,
	.read		= sec_nfc_read,
	.write		= sec_nfc_write,
	.poll		= sec_nfc_poll,
	.open		= sec_nfc_open,
	.release	= sec_nfc_close,
	.unlocked_ioctl	= sec_nfc_ioctl,
};

#ifdef CONFIG_PM
static int sec_nfc_suspend(struct device *dev)
{
	struct sec_nfc_info *info = SEC_NFC_GET_INFO(dev);
	int ret = 0;

	mutex_lock(&info->mutex);

	if (info->mode == SEC_NFC_MODE_BOOTLOADER)
		ret = -EPERM;

	mutex_unlock(&info->mutex);

	return ret;
}

static int sec_nfc_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(sec_nfc_pm_ops, sec_nfc_suspend, sec_nfc_resume);
#endif

#ifdef CONFIG_OF
/*device tree parsing*/
static int sec_nfc_parse_dt(struct device *dev,
	struct sec_nfc_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	struct device_node *node;
        /*
	pdata->ven = of_get_named_gpio(np, "ven-gpio", 0);
	pdata->firm = of_get_named_gpio(np, "firm-gpio", 0);
	pdata->wake = pdata->firm;
#ifdef CONFIG_SEC_NFC_IF_I2C
	pdata->irq = of_get_named_gpio(np, "irq-gpio", 0);
#endif
#ifdef CONFIG_SEC_NFC_CLK_REQ
	pdata->clk_req = of_get_named_gpio(np, "clk_req-gpio", 0);
#endif
       */
	node = of_find_compatible_node(NULL, NULL, SEC_NFC_PLATFORM_DTS_NAME);
        if (node) {
		of_property_read_u32_array(node, "ven-gpio",
					   &(pdata->ven), 1);
		of_property_read_u32_array(node, "firm-gpio",
					   &(pdata->firm), 1);
#ifdef CONFIG_SEC_NFC_IF_I2C
		of_property_read_u32_array(node, "irq-gpio",
					   &(pdata->irq), 1);
#endif
#ifdef CONFIG_SEC_NFC_CLK_REQ
		of_property_read_u32_array(node, "clk_req-gpio",
					   &(pdata->clk_req), 1);
#endif
	}
	pdata->wake = pdata->firm;
	pr_info("%s: irq : %d, ven : %d, firm : %d\n",
			__func__, pdata->irq, pdata->ven, pdata->firm);
	return 0;
}

#else
static int sec_nfc_parse_dt(struct device *dev,
	struct sec_nfc_platform_data *pdata)
{
	return -ENODEV;
}
#endif
static int __sec_nfc_probe(struct device *dev)
{
	struct sec_nfc_info *info;
	struct sec_nfc_platform_data *pdata = NULL;
	int ret = 0;

	dev_dbg(dev, "[NFC]sec-nfc probe start \n");
	if (dev->of_node) {
		pdata = devm_kzalloc(dev,
			sizeof(struct sec_nfc_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		ret = sec_nfc_parse_dt(dev, pdata);
		if (ret)
			return ret;
	} else {
		pdata = dev->platform_data;
	}

	if (!pdata) {
		dev_err(dev, "No platform data\n");
		ret = -ENOMEM;
		goto err_pdata;
	}

	info = kzalloc(sizeof(struct sec_nfc_info), GFP_KERNEL);
	if (!info) {
		dev_err(dev, "failed to allocate memory for sec_nfc_info\n");
		ret = -ENOMEM;
		goto err_info_alloc;
	}
	info->dev = dev;
	info->pdata = pdata;
	info->mode = SEC_NFC_MODE_OFF;

	mutex_init(&info->mutex);
	dev_set_drvdata(dev, info);

	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	info->miscdev.name = SEC_NFC_DRIVER_NAME;
	info->miscdev.fops = &sec_nfc_fops;
	info->miscdev.parent = dev;
	ret = misc_register(&info->miscdev);
	if (ret < 0) {
		dev_err(dev, "failed to register Device\n");
		goto err_dev_reg;
	}

	ret = gpio_request(pdata->ven, "nfc_ven");
	if (ret) {
		dev_err(dev, "failed to get gpio ven\n");
		goto err_gpio_ven;
	}
	gpio_direction_output(pdata->ven, SEC_NFC_PW_OFF);

	if (pdata->firm)
	{
		ret = gpio_request(pdata->firm, "nfc_firm");
		if (ret) {
			dev_err(dev, "failed to get gpio firm\n");
			goto err_gpio_firm;
		}
		gpio_direction_output(pdata->firm, SEC_NFC_FW_OFF);
	}

	wake_lock_init(&info->nfc_wake_lock, WAKE_LOCK_SUSPEND, "nfc_wake_lock");

	dev_dbg(dev, "%s: success info: %p, pdata %p\n", __func__, info, pdata);
        /* TODO */
        clk_buf_ctrl(CLK_BUF_NFC, 1);
	return 0;

err_gpio_firm:
	gpio_free(pdata->ven);
err_gpio_ven:
err_dev_reg:
	kfree(info);
err_info_alloc:
err_pdata:
	return ret;
}

static int __sec_nfc_remove(struct device *dev)
{
	struct sec_nfc_info *info = dev_get_drvdata(dev);
	struct sec_nfc_platform_data *pdata = info->pdata;
        struct sec_nfc_i2c_info i2c_info = info->i2c_info;
        struct i2c_client *client = i2c_info.i2c_dev;

	dev_dbg(info->dev, "%s\n", __func__);

	if (I2CDMABuf) {
#ifdef CONFIG_64BIT
		dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMABuf,
				  I2CDMABuf_pa);
#else
		dma_free_coherent(NULL, MAX_BUFFER_SIZE, I2CDMABuf,
				  I2CDMABuf_pa);
#endif
		I2CDMABuf = NULL;
		I2CDMABuf_pa = 0;
	}

	misc_deregister(&info->miscdev);
	sec_nfc_set_mode(info, SEC_NFC_MODE_OFF);
	gpio_set_value(pdata->firm, 0);
	gpio_free(pdata->ven);
	if (pdata->firm) gpio_free(pdata->firm);
	wake_lock_destroy(&info->nfc_wake_lock);

	kfree(info);
	sec_nfc_set_gpio_inpd();

	return 0;
}

#ifdef CONFIG_SEC_NFC_IF_I2C
MODULE_DEVICE_TABLE(i2c, sec_nfc_id_table);
typedef struct i2c_driver sec_nfc_driver_type;
#define SEC_NFC_INIT(driver)	i2c_add_driver(driver);
#define SEC_NFC_EXIT(driver)	i2c_del_driver(driver);

static int sec_nfc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;

        if(get_boot_mode() != KERNEL_POWER_OFF_CHARGING_BOOT) {
            nfc_1v8_ldo = regulator_get(NULL, "rt5081a_ldo");
            if (IS_ERR(nfc_1v8_ldo)) {
                ret = PTR_ERR(nfc_1v8_ldo);
                pr_err("[NFC] get rt5081a_ldo fail, error: %d\n", ret);
                return ret;
            }

            ret = nfc_ldo_enable();
            if (ret !=0) {
                pr_err("[NFC] failed to enable nfc ldo, error: %d\n", ret);
                return ret;
            }
        }

	ret = __sec_nfc_probe(&client->dev);
	if (ret)
		return ret;

	if (sec_nfc_i2c_probe(client))
		__sec_nfc_remove(&client->dev);

	return ret;
}

static int sec_nfc_remove(struct i2c_client *client)
{
	sec_nfc_i2c_remove(&client->dev);
	return __sec_nfc_remove(&client->dev);
}

static struct i2c_device_id sec_nfc_id_table[] = {
	{ SEC_NFC_DRIVER_NAME, 0 },
	{ }
};

#else	/* CONFIG_SEC_NFC_IF_I2C */
MODULE_DEVICE_TABLE(platform, sec_nfc_id_table);
typedef struct platform_driver sec_nfc_driver_type;
#define SEC_NFC_INIT(driver)	platform_driver_register(driver);
#define SEC_NFC_EXIT(driver)	platform_driver_unregister(driver);

static int sec_nfc_probe(struct platform_device *pdev)
{
	return __sec_nfc_probe(&pdev->dev);
}

static int sec_nfc_remove(struct platform_device *pdev)
{
	return __sec_nfc_remove(&pdev->dev);
}

static struct platform_device_id sec_nfc_id_table[] = {
	{ SEC_NFC_DRIVER_NAME, 0 },
	{ }
};

#endif /* CONFIG_SEC_NFC_IF_I2C */

#ifdef CONFIG_OF
static struct of_device_id nfc_match_table[] = {
	{ .compatible = SEC_NFC_DTS_NAME,},
	{},
};
#else
#define nfc_match_table NULL
#endif

static sec_nfc_driver_type sec_nfc_driver = {
	.probe = sec_nfc_probe,
	.id_table = sec_nfc_id_table,
	.remove = sec_nfc_remove,
	.driver = {
		.name = SEC_NFC_DRIVER_NAME,
#ifdef CONFIG_PM
		.pm = &sec_nfc_pm_ops,
#endif
		.of_match_table = nfc_match_table,
	},
};

static int sec_nfc_platform_probe(struct platform_device *pdev)
{
	pr_info("sec_nfc_platform_probe enter\n");

	gpctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(gpctrl)) {
		pr_err("%s : Cannot find pinctrl!\n", __func__);
		gpctrl = NULL;
		return 0;
	}

	/*ven_gpio*/
	st_ven_inpd = pinctrl_lookup_state(gpctrl, "ven_input_pulldown");
	if (IS_ERR(st_ven_inpd)) {
		pr_err("%s : pinctrl err, ven_input_pulldown\n", __func__);
		st_ven_inpd = NULL;
		return 0;
	}

	/*irq_gpio*/
	st_irq_inpd = pinctrl_lookup_state(gpctrl, "irq_input_pulldown");
	if (IS_ERR(st_irq_inpd)) {
		pr_err("%s : pinctrl err, irq_input_pulldown\n", __func__);
		st_irq_inpd = NULL;
		return 0;
	}
	/*firm_gpio*/
	st_firm_inpd = pinctrl_lookup_state(gpctrl, "firm_input_pulldown");
	if (IS_ERR(st_firm_inpd)) {
		pr_err("%s : pinctrl err, firm_input_pulldown\n", __func__);
		st_firm_inpd = NULL;
		return 0;
	}

	/*clk_req_gpio*/
	st_clk_req_inpd = pinctrl_lookup_state(gpctrl, "clk_req_input_pulldown");
	if (IS_ERR(st_clk_req_inpd)) {
		pr_err("%s : pinctrl err, clk_req_input_pulldown\n", __func__);
		st_clk_req_inpd = NULL;
		return 0;
	}
	return 0;
}
static int sec_nfc_platform_remove(struct platform_device *pdev)
{
	pr_info("sec_nfc_platform_remove enter\n");

	return 0;
}

/*  platform driver */
static const struct of_device_id nfc_platform_match_table[] = {
	{.compatible = SEC_NFC_PLATFORM_DTS_NAME,},
	{},
};

static struct platform_driver sec_nfc_platform_driver = {
	.probe = sec_nfc_platform_probe,
	.remove = sec_nfc_platform_remove,
	.driver = {
		   .name = SEC_NFC_DRIVER_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = nfc_platform_match_table,
#endif
		   },
};

int check_nfc_hw_support(void *x)
{
	unsigned int check_num = 0;
	char temp_nfc_hw_support[sizeof(nfc_hw_support)] = {'\0'};

	while(1) {
		mutex_lock(&proc_read_mutex);
		memcpy(temp_nfc_hw_support, nfc_hw_support, sizeof(nfc_hw_support));
		mutex_unlock(&proc_read_mutex);

		if('\0' == temp_nfc_hw_support[0] || 0 == strncmp(temp_nfc_hw_support,"unknow",6)) {
			msleep(500);
			if(check_num >= 20) {
				pr_err("get nfc_hw_support config timeout, treat as nfc supported");
				return 0;
			}
			check_num ++;
		}
		else {
			if(0 == strncmp(temp_nfc_hw_support,"Support",7)) {
				return 0;
			}
			else {
				/* remove driver if nfc is not supported */
				SEC_NFC_EXIT(&sec_nfc_driver);
				return 0;
			}
		}
	}
}

static int __init sec_nfc_init(void)
{
	int ret;

	platform_driver_register(&sec_nfc_platform_driver);
	ret = SEC_NFC_INIT(&sec_nfc_driver);
	if(0 == ret) {
		kthread_run(check_nfc_hw_support, NULL, "check_nfc_hw_support");
	}
	return ret;
}

static void __exit sec_nfc_exit(void)
{
	SEC_NFC_EXIT(&sec_nfc_driver);
	/*platform_driver_unregister(&sec_nfc_platform_driver);*/
}

module_init(sec_nfc_init);
module_exit(sec_nfc_exit);

MODULE_DESCRIPTION("Samsung sec_nfc driver");
MODULE_LICENSE("GPL");
