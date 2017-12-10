#include <linux/cpu_cooling.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include "usb_thermal.h"

/*
 * log debug
 */
#define DISABLE (0)
#define ENABLE  (1)
unsigned char log_enable = DISABLE;

static struct thermal_zone_device *thermal_dev;
int pwork_state = OFF;
struct mutex usb_temp_mutex;

#define COOLING_DEV_MAX  (8)
char * cooling_dev_name = "usb_cooling_dev";
#define USB_THERMAL_ZONE_NAME "usb_thermal_zone"
void temperature_poll_control(bool en);
extern int usb_mos_state(void);

struct usb_trip_option {
	unsigned long temp;
	enum thermal_trip_type type;
	char cdev_name[COOLING_DEV_MAX][THERMAL_NAME_LENGTH];
};

struct usb_platform_data {
	struct usb_trip_option trip_option[THERMAL_MAX_TRIPS];
	int num_trips;
};

struct usb_trip_table {
	unsigned long temp;
	enum thermal_trip_type type;
	char cdev_name[20];
};

struct usb_thermal_zone {
	struct thermal_zone_device *therm_dev;
	struct mutex th_lock;
	struct work_struct therm_work;
//	struct usb_platform_data *trip_tab;
	int trip_num;
	struct usb_trip_table trip_tab;
	enum thermal_device_mode mode;
	enum thermal_trend trend;
	unsigned long cur_temp_pseudo;
	unsigned int cur_index;
	int pdelay;

};


extern void usb_thermal_gpio_init(void);
extern void usb_thermal_gpio_set(unsigned char value);
extern int usb_thermal_gpio_get(void);
extern int usb_thermal_get_temp(void);
extern void usb_mt6331_cali_prepare(void);

static ssize_t gpio_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "gpio state:%d\n", usb_thermal_gpio_get());
}
static ssize_t gpio_state_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int result, data;

	result = kstrtoint(buf, 10, &data);
	if (result)
		return result;

	usb_thermal_gpio_set(data);

	return count;
}

static ssize_t poll_temp_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int mos_state = usb_mos_state();
	int temp = usb_thermal_get_temp();
	return sprintf(buf, "gpio state:%d, poll:%s, mos:%s, temp:%d\n", usb_thermal_gpio_get(), pwork_state==ON?"ON":"OFF",
															(mos_state==MOS_OFF)?"MOS_OFF":((mos_state==MOS_ON)?"MOS_ON":"KEEP_ON"),temp );
}

static ssize_t poll_temp_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int result, data;

	result = kstrtoint(buf, 10, &data);
	if (result)
		return result;

	temperature_poll_control(data);

	return count;
}

static ssize_t enable_log_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "log_en:%s\n", log_enable==ENABLE?"enable":"disable");
}

static ssize_t enable_log_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int result, data;

	result = kstrtoint(buf, 10, &data);
	if (result)
		return result;

	if( data ) {
		log_enable = ENABLE;
	} else {
		log_enable = DISABLE;
	}

	return count;
}

/*
 * /sys/bus/platform/devices/usb-thermal
 *
 */

static DEVICE_ATTR(enable_log, S_IRUGO | S_IWUGO, enable_log_show, enable_log_store);
static DEVICE_ATTR(poll_temp,  S_IRUGO | S_IWUGO, poll_temp_show, poll_temp_store);
static DEVICE_ATTR(gpio_state, S_IRUGO | S_IWUGO, gpio_state_show, gpio_state_store);

static struct attribute *usb_attributes[] = {
			&dev_attr_enable_log.attr,
			&dev_attr_gpio_state.attr,
			&dev_attr_poll_temp.attr,
			NULL
};

static struct attribute_group usb_attribute_group = {
	.attrs = usb_attributes
};
/*
 ********************************************************************************************************************
 ********************************************************************************************************************
 */

static int usb_cdev_match(struct usb_thermal_zone *pzone, struct thermal_cooling_device *cdev, int trip)
{
	if( !strcmp(pzone->trip_tab.cdev_name, cdev->type) ) {
		INFOR("matched tz:%s--cl:%s\n", pzone->therm_dev->type, cdev->type);
		return 0;
	}

	return -ENODEV;
}
/* Callback to bind cooling device to thermal zone */
static int usb_cdev_bind(struct thermal_zone_device *thermal,
		struct thermal_cooling_device *cdev)
{
	int i, ret= -EINVAL;
	unsigned long max_state, upper, lower;
	struct usb_thermal_zone *pzone = thermal->devdata;

	if( !strncmp(thermal->type,  USB_THERMAL_ZONE_NAME, THERMAL_NAME_LENGTH)
		&& !strncmp(cdev->type,  cooling_dev_name, THERMAL_NAME_LENGTH)	) {
		for( i=0; i<pzone->trip_num;i++ ) {
			if( usb_cdev_match(pzone, cdev, i) )
				continue;
		}

		i = 0;
		cdev->ops->get_max_state(cdev, &max_state);
		upper = lower = i > max_state ? max_state : i;
		ret = thermal_zone_bind_cooling_device(thermal, i, cdev,
			upper, lower);

		INFOR("bind tz:%s--cl:%s ret:%d\n", thermal->type, cdev->type, ret);
	}

	return ret;
}

/* Callback to unbind cooling device from thermal zone */
static int usb_cdev_unbind(struct thermal_zone_device *thermal,
		struct thermal_cooling_device *cdev)
{
	int i = 0,ret = -EINVAL;

	if( !strncmp(thermal->type,  USB_THERMAL_ZONE_NAME, THERMAL_NAME_LENGTH)
		&& !strncmp(cdev->type,  cooling_dev_name, THERMAL_NAME_LENGTH)) {

		ret = thermal_zone_unbind_cooling_device(thermal, i, cdev);
		INFOR("unbind tz:%s--cl:%s ret:%d\n", thermal->type, cdev->type, ret);
	}


	return ret;
}

/* Callback to get current temperature */
static int usb_sys_get_temp(struct thermal_zone_device *thermal,
		unsigned long *temp)
{
	*temp = usb_thermal_get_temp();
	INFOR("temp:%lu\n", *temp);
	return 0;
}

/* Callback to get temperature changing trend */
static int usb_sys_get_trend(struct thermal_zone_device *thermal,
		int trip, enum thermal_trend *trend)
{
	return 0;
}

/* Callback to get thermal zone mode */
static int usb_sys_get_mode(struct thermal_zone_device *thermal,
		enum thermal_device_mode *mode)
{
	struct usb_thermal_zone *pzone = thermal->devdata;
	*mode = pzone->mode;
	INFOR("mode:%d\n", *mode);
	return 0;
}

/* Callback to set thermal zone mode */
static int usb_sys_set_mode(struct thermal_zone_device *thermal,
		enum thermal_device_mode mode)
{
	struct usb_thermal_zone *pzone = thermal->devdata;
	pzone->mode = mode;
	INFOR("mode:%d\n", mode);
	return 0;
}

/* Callback to get trip point type */
static int usb_sys_get_trip_type(struct thermal_zone_device *thermal,
		int trip, enum thermal_trip_type *type)
{
	*type = THERMAL_TRIP_ACTIVE;
	return 0;
}

/* Callback to get trip point temperature */
static int usb_sys_get_trip_temp(struct thermal_zone_device *thermal,
		int trip, unsigned long *temp)
{
	struct usb_thermal_zone *pzone = thermal->devdata;
	*temp = pzone->trip_tab.temp;
	INFOR("trip temp:%lu\n", pzone->trip_tab.temp);
	return 0;
}

/* Callback to get critical trip point temperature */
static int usb_sys_get_crit_temp(struct thermal_zone_device *thermal,
		unsigned long *temp)
{
	struct usb_thermal_zone *pzone = thermal->devdata;
	*temp = CRITIC_TEMP;
	return 0;
}
static struct thermal_zone_device_ops usb_thdev_ops = {
	.bind = usb_cdev_bind,
	.unbind = usb_cdev_unbind,
	.get_temp = usb_sys_get_temp,
//	.get_trend = usb_sys_get_trend,
	.get_mode = usb_sys_get_mode,
	.set_mode = usb_sys_set_mode,
	.get_trip_type = usb_sys_get_trip_type,
	.get_trip_temp = usb_sys_get_trip_temp,
	.get_crit_temp = usb_sys_get_crit_temp,
};


/*
 ********************************************************************************************************************
 ********************************************************************************************************************
 */
static int usb_thermal_probe(struct platform_device *pdev)
{
	int ret;
	struct usb_thermal_zone *pzone = NULL;
	struct usb_trip_table trip_tab;
	INFOR("%s probe\n", pdev->name);
	usb_thermal_gpio_init();

	pzone = kzalloc(sizeof(struct usb_thermal_zone), GFP_KERNEL);
	if (!pzone) {
		INFOR("alloc mem failed\n");
		return -ENOMEM;
	}

	pzone->trip_num = TRIP_NUM;
	trip_tab.temp = TRIP_TEMP;// °C
	memcpy(trip_tab.cdev_name, "usb_cooling_dev", THERMAL_NAME_LENGTH);
	trip_tab.type = THERMAL_TRIP_ACTIVE;
	pzone->trip_tab = trip_tab;
	pzone->pdelay = POLL_PEROID;//ms
	pwork_state = OFF;
	mutex_init(&usb_temp_mutex);

	pzone->therm_dev = thermal_zone_device_register(USB_THERMAL_ZONE_NAME,
							pzone->trip_num, 0, pzone, &usb_thdev_ops, NULL, 0, POLL_PEROID); // poll delay:2000ms
	thermal_dev = pzone->therm_dev;

	if (IS_ERR(pzone->therm_dev)) {
		INFOR("Register thermal zone device failed.\n");
		ret = PTR_ERR(pzone->therm_dev);
		goto err;
	}
	INFOR("Thermal zone device registered.\n");

	platform_set_drvdata(pdev, pzone);
	pzone->mode = THERMAL_DEVICE_ENABLED;



	ret = sysfs_create_group(&pdev->dev.kobj, &usb_attribute_group);          
	if(ret) {                                                                   
		INFOR(" Failed to create sysfs\n");                    
		goto err;                                                           
	} 

err:
	return 0;
}
static int usb_thermal_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	INFOR("\n");

	return 0;
}

static int usb_thermal_resume(struct platform_device *pdev)
{
	INFOR("\n");
	return 0;
}

static int usb_thermal_remove(struct platform_device *pdev)
{
	INFOR("\n");
	return 0;
}
struct platform_device usb_platform_dev = {
	.name = "usb-thermal",
	.id   = -1,
};
static struct platform_device *usb_thermal_device[] = {
	&usb_platform_dev,
};

static struct platform_driver usb_thermal_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "usb-thermal",
	},
	.probe = usb_thermal_probe,
	.suspend = usb_thermal_suspend,
	.resume = usb_thermal_resume,
	.remove = usb_thermal_remove,
};

static int usb_add_platform_dev(void)
{
	INFOR("\n");
	platform_add_devices(usb_thermal_device, ARRAY_SIZE(usb_thermal_device));
	return 0;
}
fs_initcall(usb_add_platform_dev);
module_platform_driver(usb_thermal_driver);


/*
 * en/disable temperature polling
 */

void temperature_poll_control(bool en)
{
	/*
	 * en!=0, poll temperature
	 * en==0, disable poll
	 */
	// end
	/*
	 * useless,2015.11.06
	 */
	struct thermal_zone_device *thermal = thermal_dev;
	struct usb_thermal_zone *pzone = thermal->devdata;
	if( !!en ) {
		mutex_lock(&usb_temp_mutex);
		pwork_state = ON;
		thermal->polling_delay = pzone->pdelay;
		mutex_unlock(&usb_temp_mutex);

		usb_thermal_gpio_set(ON);
		INFOR("start usb temp pollling:%d ms, work_state:%d\n", pzone->pdelay, pwork_state);
		mod_delayed_work(system_freezable_wq, &thermal->poll_queue, msecs_to_jiffies(0));


	} else {

		mutex_lock(&usb_temp_mutex);
		pwork_state = OFF;
		thermal->polling_delay = 0;
		mutex_unlock(&usb_temp_mutex);

//		cancel_delayed_work(&thermal->poll_queue);
		cancel_delayed_work_sync(&thermal->poll_queue);

		usb_thermal_gpio_set(OFF);
		INFOR("exit usb temp pollling, MOS off, work_state:%d\n", pwork_state);

	}
}
EXPORT_SYMBOL(temperature_poll_control);



MODULE_DESCRIPTION("MA01 usb thermal driver");
MODULE_LICENSE("GPL");
