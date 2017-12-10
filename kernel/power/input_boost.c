#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/pm_qos.h>
#include <linux/input.h>
#include <linux/cpu.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define DEFAULT_BOOST_FREQ  	(1183000) 		/* 1183M Hz*/
#define DEFUALT_BOOST_TIME	(200 * 1000)		/* 200ms */

#define TOUCH_START_CODE	57
#define INFO(fmt,args...)  	printk("mzBoost:%s " fmt,__func__,##args)
#define MAX_DEVICES		3

#ifdef CONFIG_HAS_EARLYSUSPEND
static void boost_early_suspend(struct early_suspend *h);
static void boost_late_resume(struct early_suspend *h);
static struct early_suspend boost_handler = {
        .level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
        .suspend = boost_early_suspend,
        .resume  = boost_late_resume,
};
#endif

static struct pm_qos_request qos_req;
static struct workqueue_struct *up_wq;
static struct delayed_work boost_wk;
static unsigned int boost_device_count = 0;
static unsigned int screen_on;

static void boost_func(struct work_struct *w)
{
	struct cpufreq_policy *policy;
	unsigned int cur, target;
	unsigned int boost_time = DEFUALT_BOOST_TIME;
	target = DEFAULT_BOOST_FREQ;

	policy = cpufreq_cpu_get(0);
	if (policy) {
		cur = policy->cur;
		cpufreq_cpu_put(policy);
		if (cur < target) {
			INFO("pm_qos_update_request_timeout %d cpu freq for %d msecs\n", target, boost_time/1000);
			pm_qos_update_request_timeout(&qos_req, target, boost_time);
		}
	} else {
		INFO("policy null,boost failed\n");
	}
}



static void input_events(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{
	if (pm_qos_request_active(&qos_req) && screen_on)
	{
		//INFO("==handle:%s, type:%d, code:%d, value:%d\n",handle->name, type, code, value);
		if ((type == EV_KEY && code == BTN_TOUCH && value > 0) || (type == EV_KEY && code == KEY_HOME) || (type == EV_KEY && code == KEY_POWER))
			queue_delayed_work(up_wq, &boost_wk, 0);
	}

}

static bool input_match(struct input_handler *handler, struct input_dev *dev)
{
	if ((test_bit(EV_KEY, dev->evbit) && test_bit(BTN_TOUCH, dev->keybit)) ||
		 (test_bit(KEY_POWER, dev->keybit)) || (test_bit(KEY_POWER, dev->keybit)) ) {
		INFO("success\n");
		return true;
	}

	INFO("input_match failed\n");
	return false;
}

static int input_connect(struct input_handler *handler, struct input_dev *dev, const struct input_device_id *id)
{
	int err = -1;
	struct input_handle *handle;

	if (boost_device_count  >= MAX_DEVICES)
		return err;

	handle = kzalloc(sizeof(struct input_handle),GFP_KERNEL);
	handle->dev = dev;
	handle->handler = handler;
	handle->name = "input_boost_handle";

	err = input_register_handle(handle);
	if(err) {
		INFO("register handle fail\n");
		goto error1;
	}

	err = input_open_device(handle);
	if(err) {
		INFO("open input device failed\n");
		goto error;
	}

	screen_on = 1;
	boost_device_count++;
	INFO("input_connect success: %s\n", dev->name);
	return err;
error:
	input_unregister_handle(handle);
error1:
	kfree(handle);
	return err;
}

static void input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
	boost_device_count--;
}

static const struct input_device_id id_table[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = {BIT_MASK(EV_KEY)},
		.keybit = {[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH)},
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = {BIT_MASK(EV_KEY)},
		.keybit = {[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER)},
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = {BIT_MASK(EV_KEY)},
		.keybit = {[BIT_WORD(KEY_HOME)] = BIT_MASK(KEY_HOME)},
	},
	{}
};

static struct input_handler input_boost_handler = {
		.event = input_events,
		.match = input_match,
		.connect = input_connect,
		.disconnect = input_disconnect,
		.name = "input_boost_mz",
		.id_table = id_table,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void boost_early_suspend(struct early_suspend *h)
{
	if (pm_qos_request_active(&qos_req)) {
		screen_on = 0;
	}
}

static void boost_late_resume(struct early_suspend *h)
{
	if (pm_qos_request_active(&qos_req)) {
		screen_on = 1;
	}
}
#endif

static int __init input_boost_init(void)
{
	int ret;

	INIT_DELAYED_WORK(&boost_wk, boost_func);

	ret = input_register_handler(&input_boost_handler);
	if(!ret) {
		INFO("success\n");
	} else {
		INFO("failed\n");
	}

	up_wq = create_singlethread_workqueue("input_boost");
	if( !up_wq ) {
		INFO("create workqueue input_boost failed\n");
		input_unregister_handler(&input_boost_handler);
		return -1;
	}

	//pm_qos_add_request(&qos_req, PM_QOS_CPU_FREQ_MIN, 0);

#ifdef CONFIG_HAS_EARLYSUSPEND
    	register_early_suspend(&boost_handler);
#endif
	return 0;
}

static void __exit input_boost_exit(void)
{
	input_unregister_handler(&input_boost_handler);
	destroy_workqueue(up_wq);
	pm_qos_remove_request(&qos_req);
}

module_init(input_boost_init);
module_exit(input_boost_exit);
MODULE_AUTHOR("bsp input");
MODULE_DESCRIPTION("input boost driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("input_boost");
