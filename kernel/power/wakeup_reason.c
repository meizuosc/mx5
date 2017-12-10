/*
 * kernel/power/wakeup_reason.c
 *
 * Logs the reasons which caused the kernel to resume from
 * the suspend mode.
 *
 * Copyright (C) 2014 Google, Inc.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/wakeup_reason.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <mach/eint.h>

#define MAX_WAKEUP_REASON_IRQS 32
static int irq_list[MAX_WAKEUP_REASON_IRQS];
static int irqcount;
static bool suspend_abort;
static char abort_reason[MAX_SUSPEND_ABORT_LEN];
static struct kobject *wakeup_reason;
static DEFINE_SPINLOCK(resume_reason_lock);
extern unsigned int mt_eint_get_wakeup_count(unsigned int eint_num);
extern unsigned int modem_cnt;

static ssize_t last_resume_reason_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int irq_no, buf_offset = 0;
	struct irq_desc *desc;
	spin_lock(&resume_reason_lock);
	if (suspend_abort) {
		buf_offset = sprintf(buf, "Abort: %s", abort_reason);
	} else {
		for (irq_no = 0; irq_no < irqcount; irq_no++) {
			desc = irq_to_desc(irq_list[irq_no]);
			if (desc && desc->action && desc->action->name)
				buf_offset += sprintf(buf + buf_offset, "%d %s\n",
						irq_list[irq_no], desc->action->name);
			else
				buf_offset += sprintf(buf + buf_offset, "%d\n",
						irq_list[irq_no]);
		}
	}
	spin_unlock(&resume_reason_lock);
	return buf_offset;
}

static struct kobj_attribute resume_reason = __ATTR_RO(last_resume_reason);

static struct attribute *attrs[] = {
	&resume_reason.attr,
	NULL,
};
static struct attribute_group attr_group = {
	.attrs = attrs,
};

/*
 * logs all the wake up reasons to the kernel
 * stores the irqs to expose them to the userspace via sysfs
 */
void log_wakeup_reason(int irq)
{
	struct irq_desc *desc;
	desc = irq_to_desc(irq);
	if (desc && desc->action && desc->action->name)
		printk(KERN_INFO "Resume caused by IRQ %d, %s\n", irq,
				desc->action->name);
	else
		printk(KERN_INFO "Resume caused by IRQ %d\n", irq);

	spin_lock(&resume_reason_lock);
	if (irqcount == MAX_WAKEUP_REASON_IRQS) {
		spin_unlock(&resume_reason_lock);
		printk(KERN_WARNING "Resume caused by more than %d IRQs\n",
				MAX_WAKEUP_REASON_IRQS);
		return;
	}

	irq_list[irqcount++] = irq;
	spin_unlock(&resume_reason_lock);
}

int check_wakeup_reason(int irq)
{
	int irq_no;
	int ret = false;

	spin_lock(&resume_reason_lock);
	for (irq_no = 0; irq_no < irqcount; irq_no++)
		if (irq_list[irq_no] == irq) {
			ret = true;
			break;
	}
	spin_unlock(&resume_reason_lock);
	return ret;
}

void log_suspend_abort_reason(const char *fmt, ...)
{
	va_list args;

	spin_lock(&resume_reason_lock);

	//Suspend abort reason has already been logged.
	if (suspend_abort) {
		spin_unlock(&resume_reason_lock);
		return;
	}

	suspend_abort = true;
	va_start(args, fmt);
	snprintf(abort_reason, MAX_SUSPEND_ABORT_LEN, fmt, args);
	va_end(args);
	spin_unlock(&resume_reason_lock);
}

/* Detects a suspend and clears all the previous wake up reasons*/
static int wakeup_reason_pm_event(struct notifier_block *notifier,
		unsigned long pm_event, void *unused)
{
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		spin_lock(&resume_reason_lock);
		irqcount = 0;
		suspend_abort = false;
		spin_unlock(&resume_reason_lock);
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block wakeup_reason_pm_notifier_block = {
	.notifier_call = wakeup_reason_pm_event,
};

#ifdef CONFIG_MZ_M85
int wakeup_reason_stats_show(struct seq_file *m, void *unused) {
	unsigned int channel, eint_count;
	char name[16];
	if (modem_cnt > 0)
		seq_printf(m, "Modem wakeup: %u\n", modem_cnt);
	seq_printf(m, "Name\t\tEINT\t\tWakeup_count\n");
	for( channel = 0 ; channel < EINT_MAX_CHANNEL; channel++){
		eint_count = mt_eint_get_wakeup_count(channel);
		if (eint_count > 0) {
			if (channel == 0 ){
				strcpy(name,"ACCDET");
			} else if (channel == 1 ) {
				strcpy(name,"MHALL");
			} else if (channel == 2 ) {
				strcpy(name,"TP");
			} else if (channel == 3 ) {
				strcpy(name,"INT29125");
			} else if (channel == 4 ) {
				strcpy(name,"WIFI");
			} else if (channel == 6 ) {
				strcpy(name,"LOWBATDET");
			} else if (channel == 7 ) {
				strcpy(name,"EXT_BUCK");
			} else if (channel == 8 ) {
				strcpy(name,"INTI");
			} else if (channel == 9 ) {
				strcpy(name,"GYRO");
			} else if (channel == 10 ) {
				strcpy(name,"FP");
			} else if (channel == 11 ) {
				strcpy(name,"MUIC");
			} else if (channel == 12 ) {
				strcpy(name,"CHARGER1");
			} else if (channel == 13 ) {
				strcpy(name,"MUIC_ISET");
			} else if (channel == 15 ) {
				strcpy(name,"CHARGER2");
			} else if (channel == 17 ) {
				strcpy(name,"USB20_P1");
			} else if (channel == 18 || channel == 18 ) {
				strcpy(name,"USB20_P0");
			} else if (channel == 19 ) {
				strcpy(name,"SSUSB_SLEEP");
			} else if (channel == 21 ) {
				strcpy(name,"PMIC_INT0");
			} else if (channel == 22 ) {
				strcpy(name,"PMIC_INT1");
			} else if (channel == 107 ) {
				strcpy(name,"DSI_TE");
			} else if (channel == 117 ) {
				strcpy(name,"SMARTPA");
			} else if (channel == 122 ) {
				strcpy(name,"KEYDOWN");
			} else if (channel == 123 ) {
				strcpy(name,"KEYUP");
			} else if (channel == 132 ) {
				strcpy(name,"LASER");
			} else {
				strcpy(name,"UNKNOWN");
			}
			seq_printf(m, "%-16s %u\t\t%u\n", name, channel, eint_count);
		}
		eint_count = 0;
	}
	return 0;
}
#else
int wakeup_reason_stats_show(struct seq_file *m, void *unused) {
	unsigned int channel = 0, eint_count = 0;
	char name[16];
	if (modem_cnt > 0)
		seq_printf(m, "Modem wakeup: %u\n", modem_cnt);
	seq_printf(m, "Name\t\tEINT\t\tWakeup_count\n");
	for( channel = 0 ; channel < EINT_MAX_CHANNEL; channel++){
		eint_count = mt_eint_get_wakeup_count(channel);
		if (eint_count > 0) {
			if (channel == 2 ){
				strcpy(name,"ALS");
			} else if (channel == 4 ) {
				strcpy(name,"WIFI");
			} else if (channel == 7 ) {
				strcpy(name,"EXT_BUCK_OC");
			} else if (channel == 10 ) {
				strcpy(name,"FP");
			} else if (channel == 11 ) {
				strcpy(name,"INT_MUIC");
			} else if (channel == 13 ) {
				strcpy(name,"MUIC_ISET");
			} else if (channel == 15 ) {
				strcpy(name,"DRDY_G");
			} else if (channel == 54 ) {
				strcpy(name,"ACCDET");
			} else if (channel == 55 ) {
				strcpy(name,"GS_INT");
			} else if (channel == 82 ) {
				strcpy(name,"SD_INT");
			} else if (channel == 86 ) {
				strcpy(name,"SMARTPA");
			} else if (channel == 107 ) {
				strcpy(name,"DSI_TE");
			} else if (channel == 122 ) {
				strcpy(name,"KEYDOWN");
			} else if (channel == 123 ) {
				strcpy(name,"KEYUP");
			} else if (channel == 129 ) {
				strcpy(name,"TP");
			} else if (channel == 130 ) {
				strcpy(name,"INTI");
			} else if (channel == 131 ) {
				strcpy(name,"MHALL");
			} else {
				strcpy(name,"UNKNOWN");
			}
			seq_printf(m, "%-16s%u\t\t%u\n", name, channel, eint_count);
		}
	}
	return 0;
}
#endif
static int proc_wakeup_reason_show(struct seq_file *p, void *unused)
{
	return wakeup_reason_stats_show(p, NULL);
}

static int proc_wakeup_reason_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_wakeup_reason_show, NULL);
}

static const struct file_operations proc_wakeup_reason_operations = {
	.open		= proc_wakeup_reason_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

/* Initializes the sysfs parameter
 * registers the pm_event notifier
 */
int __init wakeup_reason_init(void)
{
	int retval;

	retval = register_pm_notifier(&wakeup_reason_pm_notifier_block);
	if (retval)
		printk(KERN_WARNING "[%s] failed to register PM notifier %d\n",
				__func__, retval);

	wakeup_reason = kobject_create_and_add("wakeup_reasons", kernel_kobj);
	if (!wakeup_reason) {
		printk(KERN_WARNING "[%s] failed to create a sysfs kobject\n",
				__func__);
		return 1;
	}
	retval = sysfs_create_group(wakeup_reason, &attr_group);
	if (retval) {
		kobject_put(wakeup_reason);
		printk(KERN_WARNING "[%s] failed to create a sysfs group %d\n",
				__func__, retval);
	}
	retval = proc_create("wakeup_reason", S_IFREG | S_IRUGO, NULL, &proc_wakeup_reason_operations);
	if (retval) {
		printk(KERN_WARNING "[%s] failed to create /proc/wakeup_reason %d\n",
				__func__, retval);
	}
	return 0;
}

late_initcall(wakeup_reason_init);
