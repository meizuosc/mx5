#include <cust_leds.h>
#include <cust_leds_def.h>
#include <mach/mt_pwm.h>

#include <linux/kernel.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
//extern int mtkfb_set_backlight_level(unsigned int level);
//extern int mtkfb_set_backlight_pwm(int div);
extern int mtkfb_set_backlight_level(unsigned int level);
extern int disp_bls_set_backlight(unsigned int level);

// Only support 64 levels of backlight (when lcd-backlight = MT65XX_LED_MODE_PWM)
#define BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT 64 
// Support 256 levels of backlight (when lcd-backlight = MT65XX_LED_MODE_PWM)
#define BACKLIGHT_LEVEL_PWM_256_SUPPORT 256 

// Configure the support type "BACKLIGHT_LEVEL_PWM_256_SUPPORT" or "BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT" !!
#define BACKLIGHT_LEVEL_PWM_MODE_CONFIG BACKLIGHT_LEVEL_PWM_256_SUPPORT

unsigned int Cust_GetBacklightLevelSupport_byPWM(void)
{
	return BACKLIGHT_LEVEL_PWM_MODE_CONFIG;
}

unsigned int brightness_mapping(unsigned int level)
{
    unsigned int mapped_level;
    
    mapped_level = level;
       
	return mapped_level;
}
/*

 * To explain How to set these para for cust_led_list[] of led/backlight
 * "name" para: led or backlight
 * "mode" para:which mode for led/backlight
 *	such as:
 *			MT65XX_LED_MODE_NONE,	
 *			MT65XX_LED_MODE_PWM,	
 *			MT65XX_LED_MODE_GPIO,	
 *			MT65XX_LED_MODE_PMIC,	
 *			MT65XX_LED_MODE_CUST_LCM,	
 *			MT65XX_LED_MODE_CUST_BLS_PWM
 *
 *"data" para: control methord for led/backlight
 *   such as:
 *			MT65XX_LED_PMIC_LCD_ISINK=0,	
 *			MT65XX_LED_PMIC_NLED_ISINK0,
 *			MT65XX_LED_PMIC_NLED_ISINK1,
 *			MT65XX_LED_PMIC_NLED_ISINK2,
 *			MT65XX_LED_PMIC_NLED_ISINK3
 * 
 *"PWM_config" para:PWM(AP side Or BLS module), by default setting{0,0,0,0,0} Or {0}
 *struct PWM_config {	 
 *  int clock_source;
 *  int div; 
 *  int low_duration;
 *  int High_duration;
 *  BOOL pmic_pad;//AP side PWM pin in PMIC chip (only 89 needs confirm); 1:yes 0:no(default)
 *};
 *-------------------------------------------------------------------------------------------
 *   for AP PWM setting as follow:
 *1.	 PWM config data
 *  clock_source: clock source frequency, can be 0/1
 *  div: clock division, can be any value within 0~7 (i.e. 1/2^(div) = /1, /2, /4, /8, /16, /32, /64, /128)
 *  low_duration: only for BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT
 *  High_duration: only for BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT
 *
 *2.	 PWM freq.
 * If BACKLIGHT_LEVEL_PWM_MODE_CONFIG = BACKLIGHT_LEVEL_PWM_256_SUPPORT,
 *	 PWM freq. = clock source / 2^(div) / 256  
 *
 * If BACKLIGHT_LEVEL_PWM_MODE_CONFIG = BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT,
 *	 PWM freq. = clock source / 2^(div) / [(High_duration+1)(Level')+(low_duration+1)(64 - Level')]
 *	           = clock source / 2^(div) / [(High_duration+1)*64]     (when low_duration = High_duration)
 *Clock source: 
 *	 0: block clock/1625 = 26M/1625 = 16K (MT6571)
 *	 1: block clock = 26M (MT6571)
 *Div: 0~7
 *
 *For example, in MT6571, PWM_config = {1,1,0,0,0} 
 *	 ==> PWM freq. = 26M/2^1/256 	 =	50.78 KHz ( when BACKLIGHT_LEVEL_PWM_256_SUPPORT )
 *	 ==> PWM freq. = 26M/2^1/(0+1)*64 = 203.13 KHz ( when BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT )
 *-------------------------------------------------------------------------------------------
 *   for BLS PWM setting as follow:
 *1.	 PWM config data
 *	 clock_source: clock source frequency, can be 0/1/2/3
 *	 div: clock division, can be any value within 0~1023
 *	 low_duration: non-use
 *	 High_duration: non-use
 *	 pmic_pad: non-use
 *
 *2.	 PWM freq.= clock source / (div + 1) /1024
 *Clock source: 
 *	 0: 26 MHz
 *	 1: 104 MHz
 *	 2: 124.8 MHz
 *	 3: 156 MHz
 *Div: 0~1023
 *
 *By default, clock_source = 0 and div = 0 => PWM freq. = 26 KHz 
 *-------------------------------------------------------------------------------------------
 */
struct lm3695_data {
	struct i2c_client *client;
	struct device *dev;
	unsigned int default_lv;
};

struct lm3695_data *lm3695_chip_data = NULL;

static int lm3695_write_byte(unsigned char addr, unsigned char value)
{
	int ret = 0;
	
	char wdata[2] = {0,};

	if (!lm3695_chip_data) {
		pr_err("Backlight driver has not been initialised yet!\n");
		return -ENODEV;
	}

	wdata[0] = addr;
	wdata[1] = value;
	ret = i2c_master_send(lm3695_chip_data->client, wdata, 2);
	if (ret < 0)
		dev_err(lm3695_chip_data->dev, "fail to send value to LM3695\n");

	return ret;
}
static int lm3695_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lm3695_data *pdata = NULL;

	if (!i2c_check_functionality(client->adapter,I2C_FUNC_I2C)) {
		dev_err(&client->dev, "fail: i2c functionality check ...\n");
		return -EOPNOTSUPP;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(struct lm3695_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(&client->dev, "alloc mm fail\n");
		return -ENOMEM;
	}

	lm3695_chip_data = pdata;
	pdata->client = client;
	pdata->dev = &client->dev;

	return 0;
}

static int lm3695_remove(struct i2c_client *client)
{
	devm_kfree(&client->dev, lm3695_chip_data);
	lm3695_chip_data = NULL;
	i2c_unregister_device(client);
}
static const struct i2c_device_id lm3695_id[] = {
	{"lm3695", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, lm3695_id);

static struct i2c_driver lm3695_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lm3695",
	},
	.probe = lm3695_probe,
	.remove = lm3695_remove,
	.id_table = lm3695_id,
};

static struct i2c_board_info __initdata lm3695_board_info = {
	I2C_BOARD_INFO("lm3695", 0x63)
};
static int __init lm3695_init(void)
{
	int ret = 0;
	ret = i2c_register_board_info(2/*I2C2*/, &lm3695_board_info, 1);

	if (ret) {
		pr_err("!!!backlight driver i2c register fail!\n");
		return -ENODEV;
	}

	ret = i2c_add_driver(&lm3695_driver);

	if (ret) {
		pr_err("!!!backlight driver add fail!\n");
		return -ENODEV;
	}

	return ret;
}

static int __exit lm3695_exit(void)
{
	i2c_del_driver(&lm3695_driver);
}

module_init(lm3695_init);
module_exit(lm3695_exit);

long lm3695_set_backlight(unsigned int level)
{
	unsigned char lsb = 0;
	unsigned char msb = 0;
	int real_level = level;
	static int last_level = 0;
	int ret = 0;

	real_level = (level<<3);
	if (real_level > 1960)
		real_level = 1960;

	if (real_level != last_level) {
		last_level = real_level;
	} else {
		pr_err("set same backlight as before!\n");
		return 0;
	}
	lsb = (real_level) & 0x7;
	msb = (real_level>>3) & 0xff;
	ret = lm3695_write_byte(0x10, 0x37);
	ret = lm3695_write_byte(0x13, lsb);
	ret = lm3695_write_byte(0x14, msb);

	if (ret < 0) {
		pr_err("set backlight fail errcode %d\n", ret);
		return ret;
	}
	return 0;
}	
static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"mx-led",               MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK0,{0}},
	//{"red",               MT65XX_LED_MODE_NONE, -1,{0}},
	{"green",             MT65XX_LED_MODE_NONE, -1,{0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1,{0}},
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1,{0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1,{0}},
	{"button-backlight",  MT65XX_LED_MODE_NONE, -1,{0}},
	//{"lcd-backlight",     MT65XX_LED_MODE_CUST_BLS_PWM, (long)disp_bls_set_backlight,{0,5}},
	//{"lcd-backlight",     MT65XX_LED_MODE_CUST_LCM, (int)mtkfb_set_backlight_level,{0}},
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_LCM, lm3695_set_backlight,{0}},
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}

