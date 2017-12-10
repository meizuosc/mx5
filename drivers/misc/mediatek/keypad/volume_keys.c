/*
 * volum_up.c
 *
 *  Created on: 2013年12月3日
 *      Author: xiaopeng
 *
 *  Created for volume up key
 *  This file include in mediatek/platform/mt6592/kernel/drivers/keypad/kpd.c
 *  GPIO75 -- external interrupt source
 *  Trigger by edge, both rising and falling
 */

#include "keys.h"

void  external_kpd_eint_set_polarity(int eint, int pol)
{
	mt_eint_set_polarity(eint, pol);
}

int  external_kpd_eint_to_irq(int eint)
{
	return mt_gpio_to_irq(eint);
}
/*
 * Init de-bounce timer
 * Set GPIO direction...
 * Register interrupt function
 * Enable interrupt
 */
void  external_kpd_eint_init(struct gpio_button_data *bdata)
{
	const struct gpio_keys_button *button = bdata->button;

	mt_set_gpio_mode(button->gpio, GPIO_MODE_00);
	mt_set_gpio_dir(button->gpio, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(button->gpio, GPIO_PULL_ENABLE);
	if(button->active_low){
		mt_set_gpio_pull_select(button->gpio, GPIO_PULL_UP);
	}else{
		mt_set_gpio_pull_select(button->gpio, GPIO_PULL_DOWN);
	}
	mt_eint_set_sens(button->irq, MT_LEVEL_SENSITIVE);
	mt_eint_set_hw_debounce(button->irq, button->debounce_interval);
	mt_eint_unmask(button->irq);
}
