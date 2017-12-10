#include <linux/init.h>
#include <linux/module.h>
#include <mach/eint.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_gpio.h>
#include <mach/mt_gpio_core.h>
#include "cust_gpio_usage.h"
#include "cust_eint.h"

static int sim_card_status = 3;
static bool sim_card_detect;

module_param(sim_card_status,int, S_IRUGO);

static int enable_sim_gpio(const char *val, const struct kernel_param *kp)
{
	int rt = param_set_bool(val, kp);
	int temp = 3;

	if (rt)
		return rt;

	if (sim_card_detect) {
		mt_set_gpio_mode(GPIO127 | 0x80000000, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO127 | 0x80000000, GPIO_DIR_IN);
		temp = mt_get_gpio_in(GPIO127 | 0x80000000);
		printk(KERN_ERR"product test: meizu sim_card_detect enable\n");
	} else {
		mt_set_gpio_mode(GPIO127 | 0x80000000, GPIO_MODE_01);
		mt_set_gpio_dir(GPIO127 | 0x80000000, GPIO_DIR_IN);
		printk(KERN_ERR"product test: meizu sim_card_detect disable\n");
	}
    
	if (temp == 0)
		sim_card_status = 0; //remove card
	if (temp == 1)
		sim_card_status = 1; //insert card

	printk(KERN_ERR"product test: INT_SIM GPIO status=%d\n", sim_card_status);

	return 0;
}

static struct kernel_param_ops md_enabled_gpio_ops = {
	.set = enable_sim_gpio,
	.get = param_get_bool,
};

static int sim_int_to_gpio_init(void)
{
	printk(KERN_ERR"%s:Set sim to gpio module init.\n",__FUNCTION__);
	return 0;
}

static void sim_int_to_gpio_exit(void)
{
	printk("Goodbye sim to gpio module exit.\n");
}

module_init(sim_int_to_gpio_init);
module_exit(sim_int_to_gpio_exit);
module_param_cb(sim_card_detect, &md_enabled_gpio_ops, &sim_card_detect, S_IRUGO | S_IWUSR);

MODULE_DESCRIPTION("Read sim card status");
MODULE_ALIAS("a product test program");
MODULE_LICENSE("Dual BSD/GPL");
