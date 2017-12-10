/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/i2c.h>

#include "keys.h"

static struct gpio_keys_button M6x_gpio_keys_tables[] = {
	{
		.code			= KEY_VOLUMEDOWN,
		.desc			= "gpio-keys: KEY_VOLUMEDOWN",
		.type			= EV_KEY,
		.active_low		= 1,
		.debounce_interval	= 50,
		.gpio           = GPIO_KEY_VOLDOWN_EINT_PIN,
		.irq            = CUST_EINT_KEYDOWN_EINT_NUM,
	}, {
		.code			= KEY_VOLUMEUP,
		.desc			= "gpio-keys: KEY_VOLUMEUP",
		.type			= EV_KEY,
		.active_low		= 1,
		.debounce_interval	= 50,
		.gpio           = GPIO_KEY_VOLUP_EINT_PIN,
		.irq            = CUST_EINT_KEYUP_EINT_NUM,
	},{
		.code			= SW_LID,// 0x15, //KEY_HALL_CLOSED,//KEY_HALL_REMOVED, //KEY_POWER,
		.desc			= "gpio-keys: KEY_HALL",
		.type			= EV_SW,
		.active_low		= 1,
		.wakeup			= 1,
		.debounce_interval	= 100,
		.gpio           = GPIO_MHALL_EINT_PIN,
		.irq            = CUST_EINT_MHALL_NUM,
	}
};

static struct gpio_keys_platform_data M6x_gpio_keys_data = {
	.buttons		= M6x_gpio_keys_tables,
	.nbuttons		= ARRAY_SIZE(M6x_gpio_keys_tables),
};

static struct platform_device m6x_gpio_keys = {
	.name			= "gpio-keys",
	.dev			= {
		.platform_data	= &M6x_gpio_keys_data,
	},
};

static struct platform_device *m6x_input_devices[] __initdata = {

	&m6x_gpio_keys,
};

static int __init mtk_external_input_init(void)
{
	return platform_add_devices(m6x_input_devices,
			ARRAY_SIZE(m6x_input_devices));
}

module_init(mtk_external_input_init);

MODULE_LICENSE("GPL2");

MODULE_DESCRIPTION("GPIO KEYS");
MODULE_ALIAS("platform:gpio");
