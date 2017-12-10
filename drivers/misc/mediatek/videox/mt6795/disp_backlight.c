#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/param.h>
#include <linux/uaccess.h>
#include <linux/backlight.h>
#include <linux/err.h>

#include <linux/delay.h>
#include <linux/module.h>
#include "primary_display.h"

static int disp_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static int disp_set_brightness(struct backlight_device *bd)
{
	int brightness = bd->props.brightness;

	primary_display_setbacklight(brightness);

	return 0;
}

static const struct backlight_ops disp_backlight_ops = {
	.get_brightness = disp_get_brightness,
	.update_status = disp_set_brightness,
};
static int disp_backlight_probe(struct platform_device *pdev)
{
	struct backlight_device *bl;
	struct backlight_properties props;

	props.type = BACKLIGHT_PLATFORM;
	props.max_brightness = 0xff;
	props.brightness = 68;

	bl = backlight_device_register("amoled", &pdev->dev, NULL, &disp_backlight_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	return 0;
}
static int disp_backlight_remove(struct platform_device *pdev)
{
	return 0;
}
static int disp_backlight_suspend(struct platform_device *pdev)
{
	return 0;
}
static int disp_backlight_resume(struct platform_device *pdev)
{
	return 0;
}
static struct platform_device disp_backlight_device = {
	.name	= "backlight",
	.id      	= 0,
};

static struct platform_driver disp_backlight_driver = {
	.probe = disp_backlight_probe,
	.remove = disp_backlight_remove,
	.suspend = disp_backlight_suspend,
	.resume = disp_backlight_resume,
	.driver = {
		.name = "backlight"	
	},
};

static int __init disp_backlight_init(void)
{
	if (platform_device_register(&disp_backlight_device)) {
		return -ENODEV;
	}

	if (platform_driver_register(&disp_backlight_driver)) {
		platform_device_unregister(&disp_backlight_device);
		return -ENODEV;
	}
	return 0;
}

static void __exit disp_backlight_exit(void)
{
	platform_driver_unregister(&disp_backlight_driver);
	platform_device_unregister(&disp_backlight_device);
}

module_init(disp_backlight_init);
module_exit(disp_backlight_exit);


MODULE_DESCRIPTION("Backlight driver for Samsung Amoled");
MODULE_LICENSE("GPL");
