/*
 * Copyright (C) 2017 FIH Mobile Limited. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/nsc_gpio.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <../../../misc/mediatek/include/mt-plat/mt_gpio.h>

extern int elan_init(void);
extern void elan_exist(void);

extern int gf_init(void);
extern void gf_exit(void);
static int ic_flag = 0;

static int __init fih_finger_init(void)
{
	printk("Damon add for gpio test\n");
	ic_flag = gpio_get_value(25);
	mt_set_gpio_dir(25,GPIO_DIR_OUT);
	mt_set_gpio_out(25,GPIO_OUT_ONE);
	printk("Damon add for gpio test, ret = %d\n", ic_flag);
//	if(!ic_flag)
	if(0)
	{
		printk("This is damon for add elan driver\n");
		elan_init();
	}
	else
	{
		printk("This is damon for add goodix driver\n");
		gf_init();
	}


	return 0;
}

static void __exit fih_finger_exit(void)
{
	if(ic_flag)
		elan_exist();
	else
		gf_exit();
}

module_init(fih_finger_init);
module_exit(fih_finger_exit);

MODULE_AUTHOR("FIH");
MODULE_DESCRIPTION("FIH finger chose driver");
MODULE_LICENSE("GPL");

