#include <mach/eint.h>
#include <mach/mt_gpio.h>
#include "cust_gpio_usage.h"
#include "cust_eint.h"
#include "inv_mpu_iio.h"

void inv_mpu_init_irq(void) 
{
	// set INT mode
	mt_set_gpio_mode(GPIO_SENSORHUB_EINT_PIN, GPIO_SENSORHUB_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_SENSORHUB_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_SENSORHUB_EINT_PIN, GPIO_PULL_DISABLE);
}
EXPORT_SYMBOL(inv_mpu_init_irq);

int inv_mpu_get_irq(void)
{
	return mt_gpio_to_irq(CUST_EINT_GYRO_NUM);
}
EXPORT_SYMBOL(inv_mpu_get_irq);

void inv_mpu_enable_irq(bool enable)
{
	if( enable ) {
		mt_eint_unmask(CUST_EINT_GYRO_NUM);
	} else {
		mt_eint_mask(CUST_EINT_GYRO_NUM);
	}
}
EXPORT_SYMBOL(inv_mpu_enable_irq);

