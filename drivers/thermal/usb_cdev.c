#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/kernel.h>

#include "usb_thermal.h"

extern int pwork_state;
extern int usb_thermal_gpio_get(void);
static unsigned long cur_state = 0;
static struct thermal_cooling_device *cool_dev;

// Return: 0 on success, an error code otherwise.
static int usb_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	INFOR("\n");
	*state = 1;
	return 0;
}

static int usb_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	*state = cur_state;
	INFOR("cur_state:%lu\n", cur_state);
	return 0;
}

extern int usb_thermal_get_temp(void);
extern void usb_thermal_gpio_set(unsigned char value);

static int status = 0;
static int usb_set_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long state)
{
	int temp = 0;
	cur_state = state;


	INFOR("work_state:%d\n", pwork_state);

		temp = usb_thermal_get_temp();
		if( temp>=TRIP_TEMP ) {//60°C
			if( status!=MOS_OFF ) {
				usb_thermal_gpio_set(OFF);
				INFOR("MOS off, temp:%d °C\n", temp);
			}
			status = MOS_OFF;

		} else if( temp<=NORMAL_TEMP ) {//45°C
#if 1
			if( status!=MOS_ON ) {
				usb_thermal_gpio_set(ON);
				INFOR("MOS on, temp:%d °C\n", temp);
			}
#endif
			status = MOS_ON;

		} else {
			status = MOS_KEEP;
		}

	INFOR("cur_state:%lu, status:0x%x, gpio:%d\n", cur_state, status, usb_thermal_gpio_get());
	return 0;
}
int usb_mos_state(void)
{
	/*
	 * #define MOS_OFF  (0xff), disable charge
	 * #define MOS_ON   (0x80), enable charge
	 * #define MOS_KEEP (0)   , do nothing
	 */
	return status;
}
EXPORT_SYMBOL(usb_mos_state);


static struct thermal_cooling_device_ops const usb_cooling_ops = {
	.get_max_state = usb_get_max_state,
	.get_cur_state = usb_get_cur_state,
	.set_cur_state = usb_set_cur_state,
};

static int usb_cdev_register(void)
{
	cool_dev = thermal_cooling_device_register("usb_cooling_dev", NULL,
						   &usb_cooling_ops);
	if (!cool_dev) {
		INFOR("register failed\n");
		return ERR_PTR(-EINVAL);
	}

	INFOR("register ok\n");
	return 0;
}

static void usb_cdev_unregister(void)
{
	if( cool_dev )
		thermal_cooling_device_unregister(cool_dev);
}
fs_initcall(usb_cdev_register);
module_exit(usb_cdev_unregister);

MODULE_DESCRIPTION("MA01 usb cooling dev driver");
MODULE_LICENSE("GPL");


