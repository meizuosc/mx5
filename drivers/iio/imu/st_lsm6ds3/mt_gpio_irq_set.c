#include <mach/eint.h>
#include <mach/mt_gpio.h>
#include "cust_gpio_usage.h"
#include "cust_eint.h"


// extern int mt_gpio_to_irq(unsigned gpio);
int sensor_gpio_to_irq(void)
{
//	return mt_gpio_to_irq(GPIO_GYRO_EINT_PIN);
	return CUST_EINT_GS_INT_NUM;
}
EXPORT_SYMBOL(sensor_gpio_to_irq);

void mt_register_irq(void) 
{
	// set INT mode
	mt_set_gpio_mode(GPIO_GYRO_EINT_PIN, GPIO_GYRO_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_GYRO_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_GYRO_EINT_PIN, GPIO_PULL_DISABLE);

	extern void st_lsm6ds3_save_timestamp(void);
	mt_eint_set_sens(CUST_EINT_GS_INT_NUM,MT_EDGE_SENSITIVE);
	mt_eint_registration(CUST_EINT_GS_INT_NUM, EINTF_TRIGGER_HIGH, st_lsm6ds3_save_timestamp, 0);
	mt_eint_unmask(CUST_EINT_GS_INT_NUM);
}
EXPORT_SYMBOL(mt_register_irq);

/*
 * 1: enable; 0:disable
 */
void mt_enable_irq(bool enable)
{
	if( enable ) {
		mt_eint_unmask(CUST_EINT_GS_INT_NUM);
	} else {
		mt_eint_mask(CUST_EINT_GS_INT_NUM);
	}
}
EXPORT_SYMBOL(mt_enable_irq);

