#include <mach/mt_clkmgr.h>
#include <linux/hw_reset.h>

#define BQ2589X_BATFET_RST_EN_MASK  0x04
#define BQ2589X_REG_09              0x09
#define BQ2589X_BATFET_RST_EN_SHIFT 2

extern int bq2589x_update_bits(u8 reg, u8 mask, u8 data);
static int charger_hw_reset_notifier(struct notifier_block *notifier,
					unsigned long event, void *unused)
{
	printk("[reset] event: %ld\n", event);
	if (event) {//code = 1, disable the charger batfet reset function
		bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_BATFET_RST_EN_MASK,
				0 << BQ2589X_BATFET_RST_EN_SHIFT);
	} else {
		/*enable the charger batfet reset function*/
		bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_BATFET_RST_EN_MASK,
				1 << BQ2589X_BATFET_RST_EN_SHIFT);
	}

	return NOTIFY_OK;
}

static struct notifier_block charger_hw_reset_nb = {
	.notifier_call = charger_hw_reset_notifier,
};

static int charger_reset_init(void)
{
	int ret = 0;
	ret = register_hw_reset_notifier(&charger_hw_reset_nb);
	if (ret) {
		printk("Failed to register hardware reset notifier\n");
	}
	return ret;
}

static void pwrkey_reset_exit(void)
{
	unregister_hw_reset_notifier(&charger_hw_reset_nb);
}

module_init(charger_reset_init);
module_exit(pwrkey_reset_exit);

