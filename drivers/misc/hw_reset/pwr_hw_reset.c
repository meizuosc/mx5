#include <mach/hal_pub_kpd.h>
#include <mach/hal_priv_kpd.h>
#include <mach/mt_clkmgr.h>
#include <linux/kpd.h>
#include <linux/hw_reset.h>

static int pwrkey_hw_reset_notifier(struct notifier_block *notifier,
					unsigned long event, void *unused)
{
	printk("[reset] event: %ld\n", event);
	if (event) {//code = 1, disable the pmic hw reset function
		mt6331_upmu_set_rg_pwrkey_rst_en(0x00);
		mt6331_upmu_set_rg_homekey_rst_en(0x00);
	} else {
		/*enable the pmic hw reset function*/
		mt6331_upmu_set_rg_pwrkey_rst_en(0x01);
		mt6331_upmu_set_rg_homekey_rst_en(0x00);
	}
	return NOTIFY_OK;
}

static struct notifier_block pwrkey_hw_reset_nb = {
	.notifier_call = pwrkey_hw_reset_notifier,
};

static int pwrkey_reset_init(void)
{
	int ret = 0;
	ret = register_hw_reset_notifier(&pwrkey_hw_reset_nb);
	if (ret) {
		printk("Failed to register hardware reset notifier\n");
	}
	return ret;
}

static void pwrkey_reset_exit(void)
{
	unregister_hw_reset_notifier(&pwrkey_hw_reset_nb);
}

module_init(pwrkey_reset_init);
module_exit(pwrkey_reset_exit);
