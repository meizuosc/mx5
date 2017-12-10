/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 * Copyright 2010, 2011 David Jander <david@protonic.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
#include "keys.h"

#include <linux/sysrq.h>
#include <linux/oom.h>

#define SYSRQ_CONVERT_ADJ(x) ((x * OOM_SCORE_ADJ_MAX) / -OOM_DISABLE)
#define SYSRQ_REVERT_ADJ(x)  (x * (-OOM_DISABLE + 1) / OOM_SCORE_ADJ_MAX)

/*
 * SYSFS interface for enabling/disabling keys and switches:
 *
 * There are 4 attributes under /sys/devices/platform/gpio-keys/
 *	keys [ro]              - bitmap of keys (EV_KEY) which can be
 *	                         disabled
 *	switches [ro]          - bitmap of switches (EV_SW) which can be
 *	                         disabled
 *	disabled_keys [rw]     - bitmap of keys currently disabled
 *	disabled_switches [rw] - bitmap of switches currently disabled
 *
 * Userland can change these values and hence disable event generation
 * for each key (or switch). Disabling a key means its interrupt line
 * is disabled.
 *
 * For example, if we have following switches set up as gpio-keys:
 *	SW_DOCK = 5
 *	SW_CAMERA_LENS_COVER = 9
 *	SW_KEYPAD_SLIDE = 10
 *	SW_FRONT_PROXIMITY = 11
 * This is read from switches:
 *	11-9,5
 * Next we want to disable proximity (11) and dock (5), we write:
 *	11,5
 * to file disabled_switches. Now proximity and dock IRQs are disabled.
 * This can be verified by reading the file disabled_switches:
 *	11,5
 * If we now want to enable proximity (11) switch we write:
 *	5
 * to disabled_switches.
 *
 * We can disable only those keys which don't allow sharing the irq.
 */

/**
 * get_n_events_by_type() - returns maximum number of events per @type
 * @type: type of button (%EV_KEY, %EV_SW)
 *
 * Return value of this function can be used to allocate bitmap
 * large enough to hold all bits for given type.
 */
static inline int get_n_events_by_type(int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? KEY_CNT : SW_CNT;
}

/**
 * gpio_keys_disable_button() - disables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Disables button pointed by @bdata. This is done by masking
 * IRQ line. After this function is called, button won't generate
 * input events anymore. Note that one can only disable buttons
 * that don't share IRQs.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races when concurrent threads are
 * disabling buttons at the same time.
 */
static void gpio_keys_disable_button(struct gpio_button_data *bdata)
{
	if (!bdata->disabled) {
		/*
		 * Disable IRQ and possible debouncing timer.
		 */
		disable_irq(bdata->irq);

		bdata->disabled = true;
	}
}

/**
 * gpio_keys_enable_button() - enables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Enables given button pointed by @bdata.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races with concurrent threads trying
 * to enable the same button at the same time.
 */
static void gpio_keys_enable_button(struct gpio_button_data *bdata)
{
	if (bdata->disabled) {
		enable_irq(bdata->irq);
		bdata->disabled = false;
	}
}

/**
 * gpio_keys_attr_show_helper() - fill in stringified bitmap of buttons
 * @ddata: pointer to drvdata
 * @buf: buffer where stringified bitmap is written
 * @type: button type (%EV_KEY, %EV_SW)
 * @only_disabled: does caller want only those buttons that are
 *                 currently disabled or all buttons that can be
 *                 disabled
 *
 * This function writes buttons that can be disabled to @buf. If
 * @only_disabled is true, then @buf contains only those buttons
 * that are currently disabled. Returns 0 on success or negative
 * errno on failure.
 */
static ssize_t gpio_keys_attr_show_helper(struct gpio_keys_drvdata *ddata,
					  char *buf, unsigned int type,
					  bool only_disabled)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t ret;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (only_disabled && !bdata->disabled)
			continue;

		__set_bit(bdata->button->code, bits);
	}

	ret = bitmap_scnlistprintf(buf, PAGE_SIZE - 2, bits, n_events);
	buf[ret++] = '\n';
	buf[ret] = '\0';

	kfree(bits);

	return ret;
}

/**
 * gpio_keys_attr_store_helper() - enable/disable buttons based on given bitmap
 * @ddata: pointer to drvdata
 * @buf: buffer from userspace that contains stringified bitmap
 * @type: button type (%EV_KEY, %EV_SW)
 *
 * This function parses stringified bitmap from @buf and disables/enables
 * GPIO buttons accordingly. Returns 0 on success and negative error
 * on failure.
 */
static ssize_t gpio_keys_attr_store_helper(struct gpio_keys_drvdata *ddata,
					   const char *buf, unsigned int type)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t error;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	error = bitmap_parselist(buf, bits, n_events);
	if (error)
		goto out;

	/* First validate */
	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits) &&
		    !bdata->button->can_disable) {
			error = -EINVAL;
			goto out;
		}
	}

	mutex_lock(&ddata->disable_lock);

	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits))
			gpio_keys_disable_button(bdata);
		else
			gpio_keys_enable_button(bdata);
	}

	mutex_unlock(&ddata->disable_lock);

out:
	kfree(bits);
	return error;
}

#define ATTR_SHOW_FN(name, type, only_disabled)				\
static ssize_t gpio_keys_show_##name(struct device *dev,		\
				     struct device_attribute *attr,	\
				     char *buf)				\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
									\
	return gpio_keys_attr_show_helper(ddata, buf,			\
					  type, only_disabled);		\
}

ATTR_SHOW_FN(keys, EV_KEY, false);
ATTR_SHOW_FN(switches, EV_SW, false);
ATTR_SHOW_FN(disabled_keys, EV_KEY, true);
ATTR_SHOW_FN(disabled_switches, EV_SW, true);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/keys [ro]
 * /sys/devices/platform/gpio-keys/switches [ro]
 */
static DEVICE_ATTR(keys, S_IRUGO, gpio_keys_show_keys, NULL);
static DEVICE_ATTR(switches, S_IRUGO, gpio_keys_show_switches, NULL);

#define ATTR_STORE_FN(name, type)					\
static ssize_t gpio_keys_store_##name(struct device *dev,		\
				      struct device_attribute *attr,	\
				      const char *buf,			\
				      size_t count)			\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
	ssize_t error;							\
									\
	error = gpio_keys_attr_store_helper(ddata, buf, type);		\
	if (error)							\
		return error;						\
									\
	return count;							\
}

ATTR_STORE_FN(disabled_keys, EV_KEY);
ATTR_STORE_FN(disabled_switches, EV_SW);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/disabled_keys [rw]
 * /sys/devices/platform/gpio-keys/disables_switches [rw]
 */
static ssize_t key_hall_press_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	int i, value = REMOVED;
	
	/* First validate */
	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type == EV_SW){
			value = (atomic_read(&bdata->key_pressed) == PRESSED)?CLOSED:REMOVED;
			break;
		}
	}
	return sprintf(buf,"%d\n",value);
}

static DEVICE_ATTR(key_hall_state, 0444, key_hall_press_show, NULL);

static DEVICE_ATTR(disabled_keys, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_keys,
		   gpio_keys_store_disabled_keys);
static DEVICE_ATTR(disabled_switches, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_switches,
		   gpio_keys_store_disabled_switches);

static struct attribute *gpio_keys_attrs[] = {
	&dev_attr_keys.attr,
	&dev_attr_switches.attr,
	&dev_attr_disabled_keys.attr,
	&dev_attr_disabled_switches.attr,
	&dev_attr_key_hall_state.attr,
	NULL,
};

static struct attribute_group gpio_keys_attr_group = {
	.attrs = gpio_keys_attrs,
};

extern bool printk_disable_uart;

static int dump_processes(void)
{
	struct task_struct *tsk;
	short oom_score_adj;

	pr_emerg("======   sysrq combine key show processes   =====\n");
#ifdef CONFIG_ZRAM
	pr_emerg("<sysrq>   pid  adj    score_adj   rss    rswap      name\n");
#else
	pr_emerg("<sysrq>   pid  adj    score_adj   rss      name\n");
#endif

	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *p;

		if (tsk->flags & PF_KTHREAD)
			continue;

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

		oom_score_adj = p->signal->oom_score_adj;

		pr_emerg(

#ifdef CONFIG_ZRAM
				"<sysrq> %5d%5d%11d%8lu%8lu		%s\n", p->pid,
				SYSRQ_REVERT_ADJ(oom_score_adj), oom_score_adj,
				get_mm_rss(p->mm),
				get_mm_counter(p->mm, MM_SWAPENTS), p->comm);
#else /* CONFIG_ZRAM */
				"<sysrq> %5d%5d%11d%8lu		%s\n", p->pid,
				SYSRQ_REVERT_ADJ(oom_score_adj), oom_score_adj,
				get_mm_rss(p->mm), p->comm);
#endif
		task_unlock(p);
	}
	rcu_read_unlock();

	return 0;
}

#ifdef CONFIG_MTK_ION
extern void ion_mm_heap_memory_detail(void);
#endif

static void gpio_combin_sysrq(u16 vol_key_up_flag, u16 vol_key_down_flag)
{
	static unsigned int sysrq_combin_keys = 0;

	/* step1: press volume-up 4 times */
	if (sysrq_combin_keys < 4) {
		if ((vol_key_up_flag == 1) && (vol_key_down_flag == 0)) {
			sysrq_combin_keys++;
		}
		else {
			sysrq_combin_keys = 0;
		}
	/* step2: press volume-down 4 times */
	} else if (sysrq_combin_keys >= 4 && sysrq_combin_keys < 8) {
		if ((vol_key_down_flag == 1) && (vol_key_up_flag == 0)) {
			sysrq_combin_keys++;
		}
		else {
			sysrq_combin_keys = 0;
		}
	}

	if (8 == sysrq_combin_keys) {
		sysrq_combin_keys = 0;
		printk_disable_uart = 0;
		pr_emerg("\n\n--- --- show memory info --- ---\n");
		handle_sysrq('m');
#ifdef CONFIG_MTK_ION
		pr_emerg("\n\n--- --- show ion buffer info --- ---\n");
		ion_mm_heap_memory_detail();
#endif
		pr_emerg("\n\n--- --- show D processes info --- ---\n");
		handle_sysrq('w'); //sysrq_showstate_blocked_op
		pr_emerg("\n\n--- --- show processes status --- ---\n");
		dump_processes();
		pr_emerg("\n\n--- --- show processes backtrace --- ---\n");
		handle_sysrq('l'); //sysrq_showallcpus_op
	}
}

static void gpio_keys_gpio_report_event(struct gpio_button_data *bdata)
{
	u16 vol_up_flag = 0;
	u16 vol_down_flag = 0;
	unsigned char pressed = RELEASED, input_value = 0xff;

	mutex_lock(&bdata->key_mutex);
	input_value = mt_get_gpio_in(bdata->button->gpio);
	pressed = (input_value?1:0) ^ bdata->button->active_low;

	if (bdata->button->code == KEY_VOLUMEUP) {
		if (pressed)
			vol_up_flag = 1;
	} else 	if (bdata->button->code == KEY_VOLUMEDOWN) {
		if (pressed)
			vol_down_flag = 1;
	}

	if (vol_up_flag | vol_down_flag)
		gpio_combin_sysrq(vol_up_flag, vol_down_flag);

	atomic_set(&bdata->key_pressed, (pressed == PRESSED)?PRESSED:RELEASED);
	input_event(bdata->input, bdata->button->type, bdata->button->code, !!pressed);
	input_sync(bdata->input);

	mutex_unlock(&bdata->key_mutex);
}

static irqreturn_t gpio_keys_gpio_thread_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;

	gpio_keys_gpio_report_event(bdata);
	return IRQ_HANDLED;
}

static irqreturn_t gpio_keys_gpio_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;

	external_kpd_eint_set_polarity(bdata->button->irq, mt_get_gpio_in(bdata->button->gpio)?LOW:HIGH);
	return IRQ_WAKE_THREAD;
}

static int gpio_keys_setup_key(struct platform_device *pdev,
					 struct input_dev *input,
					 struct gpio_button_data *bdata,
					 const struct gpio_keys_button *button)
{
	const char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	unsigned long irqflags;
	int error;

	bdata->input = input;
	bdata->button = button;
	bdata->irq = external_kpd_eint_to_irq(button->irq);

	atomic_set(&bdata->key_pressed, RELEASED);
	mutex_init(&bdata->key_mutex);
	spin_lock_init(&bdata->lock);

	if (button->debounce_interval)
		bdata->timer_debounce = button->debounce_interval;

	input_set_capability(input, button->type ?: EV_KEY, button->code);

	external_kpd_eint_init(bdata);

	if(bdata->button->active_low)
		irqflags = IRQF_TRIGGER_LOW | IRQF_ONESHOT;
	else
		irqflags = IRQF_TRIGGER_HIGH | IRQF_ONESHOT;

	error = request_threaded_irq(bdata->irq, gpio_keys_gpio_isr, gpio_keys_gpio_thread_isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			bdata->irq, error);
		return -1;
	}


	return 0;
}

static int gpio_keys_open(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);

	return ddata->enable ? ddata->enable(input->dev.parent) : 0;
}

static void gpio_keys_close(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);

	if (ddata->disable)
		ddata->disable(input->dev.parent);
}

static void gpio_remove_key(struct gpio_button_data *bdata)
{
	free_irq(bdata->irq, bdata);
}

static int  gpio_keys_probe(struct platform_device *pdev)
{
	const struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata;
	struct device *dev = &pdev->dev;
	struct input_dev *input;
	int i, error;
	int wakeup = 0;

	if (!pdata) {
		printk("[%s] pdata is NULL\n",__func__);
		return -ENOMEM;
	}

	ddata = kzalloc(sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data),
			GFP_KERNEL);
	if (!ddata) {
		printk("[%s] allocate is fail\n",__func__);
		return -ENOMEM;
	}

	input = input_allocate_device();
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		kfree(ddata);
		return -ENOMEM;
	}

	ddata->input = input;
	ddata->n_buttons = pdata->nbuttons;
	ddata->enable = pdata->enable;
	ddata->disable = pdata->disable;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdata->name ? : pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;
	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		const struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];
		error = gpio_keys_setup_key(pdev, input, bdata, button);
		if (error)
			goto fail2;

		printk("[%s] key[%d]-%s is %s\n",__func__,i,bdata->button->desc,((bdata==NULL)?"NULL":"FULL"));

		if (button->wakeup)
			wakeup = 1;
	}

	error = sysfs_create_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	if (error) {
		dev_err(dev, "Unable to export keys/switches, error: %d\n",
			error);
		goto fail2;
	}


#ifdef INTERFACE
	meizu_sysfslink_register_n(&pdev->dev, "hall");
#endif

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto fail3;
	}

	device_init_wakeup(&pdev->dev, wakeup);

	printk("[gpio_keys_probe] ok!\n");
	return 0;

 fail3:
	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);
 fail2:
	while (--i >= 0)
		gpio_remove_key(&ddata->data[i]);

	platform_set_drvdata(pdev, NULL);
	input_free_device(input);
	kfree(ddata);

	return error;
}

static int gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < ddata->n_buttons; i++)
		gpio_remove_key(&ddata->data[i]);

	input_unregister_device(input);

	/*
	 * If we had no platform_data, we allocated buttons dynamically, and
	 * must free them here. ddata->data[0].button is the pointer to the
	 * beginning of the allocated array.
	 */
	if (!pdev->dev.platform_data)
		kfree(ddata->data[0].button);

	kfree(ddata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gpio_keys_suspend(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	int i;

	printk("%s\n",__func__);

	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->n_buttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];
			if (bdata->button->wakeup)
				enable_irq_wake(bdata->button->gpio);
		}
	}

	return 0;
}

#ifdef CONFIG_AUTOTEST_SUSPEND
static int emulate_power_key(struct gpio_keys_drvdata *ddata)
{
	if (!sysctl_suspend_test)
		return 0;

	/* Force to send a HOME key to wake the whole system */
	input_report_key(ddata->input, KEY_POWER, 1);
	udelay(5);
	input_report_key(ddata->input, KEY_POWER, 0);

	return 1;
}
#else
#define emulate_power_key(ddata)	(0)
#endif


static int gpio_keys_resume(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	int i;

	printk("%s\n",__func__);

	/* Added for autotest framework */
	if (emulate_power_key(ddata))
		return 0;

	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];
		if (bdata->button->wakeup && device_may_wakeup(dev))
			disable_irq_wake(bdata->button->gpio);

		if (bdata->button->gpio) {
			//gpio_keys_gpio_report_event(bdata);
#if 0
			input_report_key(bdata->input, bdata->button->code, RELEASED);
			input_sync(bdata->input);
#else
			// do nothing
#endif
			printk("@@@@@@@@@@@@@@@@@@@@@@ %s--do nothing check_status:%s\n",
					bdata->button->desc,(atomic_read(&bdata->key_pressed) == PRESSED)?"Pressed":"Released");
		}
	}
	input_sync(ddata->input);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(gpio_keys_pm_ops, gpio_keys_suspend, gpio_keys_resume);

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= gpio_keys_remove,
	.driver		= {
		.name	= "gpio-keys",
		.owner	= THIS_MODULE,
		.pm	= &gpio_keys_pm_ops,
	}
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

late_initcall(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for GPIOs");
MODULE_ALIAS("platform:gpio-keys");
