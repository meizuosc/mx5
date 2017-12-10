;/*
 * BQ2589x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 * Copyright (C) 2014  Meizu Technology Co.Ltd 
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <mach/mt_gpio.h>
#include <mach/irqs.h>
#include <mach/eint.h>
#include "cust_eint.h"
#include <mach/mt_clkmgr.h>
#include <linux/kthread.h>
#include <linux/earlysuspend.h>
#include <mach/battery_common.h>
#include <linux/meizu-sys.h>
#include <linux/wakelock.h>
#include <mach/bq2589x_reg.h>

#define BQ2589x_REG_NUM	21 
#define BQ2589x_BUSNUM0 0
#define BQ2589x_BUSNUM2 2
#define BQ2589x_SLAVE_ADDR  0x6B //BQ25892
#define BQ2589x_SLAVE_ADDR0  0x6A //BQ25890

#define BQ25892_I2C0 0
#define BQ25892_I2C2 1

kal_bool fast_charge_current_1C = KAL_TRUE;
static kal_bool power_good = KAL_FALSE;
static kal_uint32 charger_fault = KAL_FALSE;

static DEFINE_MUTEX(i2c_mutex);
static DEFINE_MUTEX(i2c_update_mutex);
static DEFINE_MUTEX(handler_mutex);

static struct i2c_board_info __initdata i2c_bq2589x = 
{ 
	I2C_BOARD_INFO("bq25892", BQ2589x_SLAVE_ADDR)
};

static struct i2c_board_info __initdata i2c_bq2589x_sed = 
{ 
	I2C_BOARD_INFO("bq25892_sed", BQ2589x_SLAVE_ADDR)
};

enum bq2589x_vbus_type {
	BQ2589X_VBUS_NONE,
	BQ2589X_VBUS_USB_SDP,
	BQ2589X_VBUS_USB_CDP,
	BQ2589X_VBUS_USB_DCP,
	BQ2589X_VBUS_MAXC,
	BQ2589X_VBUS_UNKNOWN,
	BQ2589X_VBUS_NONSTAND,
	BQ2589X_VBUS_OTG,
    BQ2589X_VBUS_TYPE_NUM,
};

struct bq2589x {
	struct device *dev;
	struct i2c_client *client;
	struct early_suspend fast_handler;
	struct delayed_work irq_dwork;
	struct wake_lock charger_wake_lock;
};

static struct bq2589x *g_bq = NULL;
static struct bq2589x *g_bq2 = NULL;

kal_bool chargin_hw_init_done = KAL_TRUE;
static DEFINE_MUTEX(chgreset_mutex);
static DEFINE_MUTEX(chgreset1_mutex);

static int bq2589x_read_byte(u8 *data, u8 reg)
{
	int ret;

	mutex_lock(&i2c_mutex);
	ret = i2c_smbus_read_byte_data(g_bq->client, reg);
	if (ret < 0) {
		dev_err(g_bq->dev, "failed to read 0x%.2x\n", reg);
		mutex_unlock(&i2c_mutex);
		return ret;
	}

	*data = (u8)ret;
	mutex_unlock(&i2c_mutex);

	return 0;
}

static int bq2589x_write_byte(u8 reg, u8 data)
{
	int ret;

	mutex_lock(&i2c_mutex);
	ret = i2c_smbus_write_byte_data(g_bq->client, reg, data);
	if (ret < 0 ) {
		dev_err(g_bq->dev, "failed to read 0x%.2x\n", reg);
		mutex_unlock(&i2c_mutex);
		return ret;
	}

	mutex_unlock(&i2c_mutex);
	return ret;
}

int bq2589x_update_bits(u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&i2c_update_mutex);
	ret = bq2589x_read_byte(&tmp, reg);
	if (ret) {
		mutex_unlock(&i2c_update_mutex);
		return ret;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = bq2589x_write_byte(reg, tmp);

	mutex_unlock(&i2c_update_mutex);
	return ret;
}

int bq2589x_sed_read_byte(u8 *data, u8 reg)
{
	int ret;

	mutex_lock(&i2c_mutex);
	ret = i2c_smbus_read_byte_data(g_bq2->client, reg);
	if (ret < 0) {
		dev_err(g_bq2->dev, "failed to read 0x%.2x\n", reg);
		mutex_unlock(&i2c_mutex);
		return ret;
	}

	*data = (u8)ret;

	mutex_unlock(&i2c_mutex);
	return 0;
}
 int bq2589x_sed_write_byte(u8 reg, u8 data)
{
	int ret;

	mutex_lock(&i2c_mutex);
	ret = i2c_smbus_write_byte_data(g_bq2->client, reg, data);
	if (ret < 0 ) {
		dev_err(g_bq2->dev, "failed to read 0x%.2x\n", reg);
		mutex_unlock(&i2c_mutex);
		return ret;
	}

	mutex_unlock(&i2c_mutex);
	return ret;
}

int bq2589x_sed_update_bits(u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	mutex_lock(&i2c_update_mutex);
	ret = bq2589x_sed_read_byte(&tmp, reg);
	if (ret) {
		mutex_unlock(&i2c_update_mutex);
		return ret;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = bq2589x_sed_write_byte(reg, tmp);

	mutex_unlock(&i2c_update_mutex);
	return ret;
}

//CON0--------------------------------------
void bq2589x_set_en_hiz(int val)
{
	int ret;

	ret = bq2589x_update_bits(BQ2589X_REG_00, BQ2589X_ENHIZ_MASK,
			val << BQ2589X_ENHIZ_SHIFT);

	ret = bq2589x_sed_update_bits(BQ2589X_REG_00, BQ2589X_ENHIZ_MASK,
			val << BQ2589X_ENHIZ_SHIFT);
}

void bq2589x_set_en_hiz_minor(int val)
{
	int ret;

	ret = bq2589x_update_bits(BQ2589X_REG_00, BQ2589X_ENHIZ_MASK,
			val << BQ2589X_ENHIZ_SHIFT);
}

void bq2589x_set_en_ilim(int val)
{
	int ret;

	ret = bq2589x_update_bits(BQ2589X_REG_00, BQ2589X_ENILIM_MASK,
			val << BQ2589X_ENILIM_SHIFT);

	ret = bq2589x_sed_update_bits(BQ2589X_REG_00, BQ2589X_ENILIM_MASK,
			val << BQ2589X_ENILIM_SHIFT);
}

int bq2589x_set_input_current_limit1(int curr)
{
	u8 val;

	val = (curr/100 - BQ2589X_IINLIM_BASE)/BQ2589X_IINLIM_LSB;
	return bq2589x_update_bits(BQ2589X_REG_00,BQ2589X_IINLIM_MASK,
			val << BQ2589X_IINLIM_SHIFT);
}

int bq2589x_set_input_current_limit(int curr)
{
	u8 val = 0;
	int val_curr;

	val = (curr/100 - BQ2589X_IINLIM_BASE)/BQ2589X_IINLIM_LSB;
	return bq2589x_sed_update_bits(BQ2589X_REG_00,BQ2589X_IINLIM_MASK,
			val << BQ2589X_IINLIM_SHIFT);
}

//CON2------------------------------------
//I2C2为主IC, VBUS直接与其相连
int bq2589x_adc_start(bool oneshot)
{
    u8 val, val1;
    int ret;

    ret = bq2589x_sed_read_byte(&val1,BQ2589X_REG_02);
    if(ret < 0){
        dev_err(g_bq2->dev,"%s failed to read register 0x02:%d\n",__func__,ret);
        return ret;
    }

    if(oneshot) {
        ret = bq2589x_sed_update_bits(BQ2589X_REG_02,BQ2589X_CONV_START_MASK, BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
        ret = bq2589x_sed_update_bits(BQ2589X_REG_02,BQ2589X_CONV_START_MASK, BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
    } else {
        ret = bq2589x_sed_update_bits(BQ2589X_REG_02,BQ2589X_CONV_RATE_MASK, BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);
    }

    return ret;
}

int bq2589x_adc_stop(void)//stop continue scan 
{
	int ret;

    	ret = bq2589x_sed_update_bits(BQ2589X_REG_02,BQ2589X_CONV_RATE_MASK,
		    BQ2589X_ADC_CONTINUE_DISABLE << BQ2589X_CONV_RATE_SHIFT);

	return ret;
}

int bq2589x_disable_ico(int disable)
{
	int ret;
	u8 val;

	if (disable) {
		val = BQ2589X_ICO_DISABLE << BQ2589X_ICOEN_SHIFT;
	} else {
		val = BQ2589X_ICO_ENABLE << BQ2589X_ICOEN_SHIFT;
	}

	ret = bq2589x_update_bits(BQ2589X_REG_02, BQ2589X_ICOEN_MASK, val);
	ret = bq2589x_sed_update_bits(BQ2589X_REG_02, BQ2589X_ICOEN_MASK, val);

	return ret;
}

//CON3--------------------------------------
void bq2589x_set_wdt_rst(void)
{	
	int ret;

	u8 val = BQ2589X_WDT_RESET << BQ2589X_WDT_RESET_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_03, BQ2589X_WDT_RESET_MASK, val);
	ret = bq2589x_sed_update_bits(BQ2589X_REG_03, BQ2589X_WDT_RESET_MASK, val);
}

int bq2589x_enable_otg(void)
{
	int ret;

   	 u8 val = BQ2589X_OTG_ENABLE << BQ2589X_OTG_CONFIG_SHIFT;

	ret = bq2589x_sed_update_bits(BQ2589X_REG_03, BQ2589X_OTG_CONFIG_MASK, val);

	return ret;
}

int bq2589x_disable_otg(void)
{
	int ret;
	u8 val = BQ2589X_OTG_DISABLE << BQ2589X_OTG_CONFIG_SHIFT;

	ret = bq2589x_sed_update_bits(BQ2589X_REG_03, BQ2589X_OTG_CONFIG_MASK, val);
	return ret;
}

static int bq2589x_enable_charger1(void)
{
	int ret;
    	u8 val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;

    	ret = bq2589x_update_bits(BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);

	return ret;
}

int bq2589x_enable_charger(void)
{
	int ret;
    	u8 val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;

    	ret = bq2589x_sed_update_bits(BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
	return ret;
}

static int bq2589x_disable_charger1(void)
{
	int ret;
    	u8 val = BQ2589X_CHG_DISABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);

	return ret;
}

static int bq2589x_disable_charger(void)
{
	int ret;
    	u8 val = BQ2589X_CHG_DISABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_sed_update_bits(BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);

	return ret;
}

void bq2589x_set_sys_min(int sysv_min)
{
	int ret;
	// system min voltage
	ret = bq2589x_update_bits(BQ2589X_REG_03,BQ2589X_SYS_MINV_MASK,
			sysv_min << BQ2589X_SYS_MINV_SHIFT);

	ret = bq2589x_sed_update_bits(BQ2589X_REG_03,BQ2589X_SYS_MINV_MASK,
			sysv_min << BQ2589X_SYS_MINV_SHIFT);
}

//CON4---------------------------------------
/*
 * Enable Mtk Pump Express Pulse
 */
void bq2589x_enable_pumpx(int enable)
{
	int ret;

	ret = bq2589x_sed_update_bits(BQ2589X_REG_04, BQ2589X_EN_PUMPX_MASK,
			enable << BQ2589X_EN_PUMPX_SHIFT);
}

int bq2589x_set_chargecurrent1(int curr)	
{
	u8 ichg;
	int ret;

	ichg = (curr/100 - BQ2589X_ICHG_BASE)/BQ2589X_ICHG_LSB;
    	ret = bq2589x_update_bits(BQ2589X_REG_04,BQ2589X_ICHG_MASK,
			ichg << BQ2589X_ICHG_SHIFT);

	return ret;
}

//I2C2
int bq2589x_set_chargecurrent(int curr)	
{
	u8 ichg;
	int ret;

	ichg = (curr/100 - BQ2589X_ICHG_BASE)/BQ2589X_ICHG_LSB;
    	ret = bq2589x_sed_update_bits(BQ2589X_REG_04,BQ2589X_ICHG_MASK,
			ichg << BQ2589X_ICHG_SHIFT);

	return ret;
}

//CON5-------------------------------------------
void bq2589x_set_iprechg(int pre_curr)
{
	int ret;

	ret = bq2589x_update_bits(BQ2589X_REG_05, BQ2589X_IPRECHG_MASK,
			pre_curr << BQ2589X_IPRECHG_SHIFT);

	ret = bq2589x_sed_update_bits(BQ2589X_REG_05, BQ2589X_IPRECHG_MASK,
			pre_curr << BQ2589X_IPRECHG_SHIFT);
}

void bq2589x_set_iterm(int curr)
{
	int ret;

	// termination current
	ret = bq2589x_update_bits(BQ2589X_REG_05,BQ2589X_ITERM_MASK,
			curr << BQ2589X_ITERM_SHIFT);

	ret = bq2589x_sed_update_bits(BQ2589X_REG_05,BQ2589X_ITERM_MASK,
			curr << BQ2589X_ITERM_SHIFT);
}

//CON6------------------------------------------------
void bq2589x_set_chargevoltage(int volt)
{
	int ret;

	ret = bq2589x_update_bits(BQ2589X_REG_06,BQ2589X_VREG_MASK,
			volt << BQ2589X_VREG_SHIFT);

	ret = bq2589x_sed_update_bits(BQ2589X_REG_06,BQ2589X_VREG_MASK,
			volt << BQ2589X_VREG_SHIFT);
}

void bq2589x_set_batlowv(int low_volt)
{
	int ret;
	// precharge voltage

        ret = bq2589x_update_bits(BQ2589X_REG_06,BQ2589X_BATLOWV_MASK,
			low_volt << BQ2589X_BATLOWV_SHIFT);

        ret = bq2589x_sed_update_bits(BQ2589X_REG_06,BQ2589X_BATLOWV_MASK,
			low_volt << BQ2589X_BATLOWV_SHIFT);
}

void bq2589x_set_vrechg(int val)
{
	int ret;
        ret = bq2589x_update_bits(BQ2589X_REG_06,BQ2589X_VRECHG_MASK,
			val << BQ2589X_VRECHG_SHIFT);
	
        ret = bq2589x_sed_update_bits(BQ2589X_REG_06,BQ2589X_VRECHG_MASK,
			val << BQ2589X_VRECHG_SHIFT);
}

//CON7-------------------------------------
void bq2589x_set_en_term(int val)
{
	int ret;
        ret = bq2589x_update_bits(BQ2589X_REG_07,BQ2589X_EN_TERM_MASK,
			val << BQ2589X_EN_TERM_SHIFT);

        ret = bq2589x_sed_update_bits(BQ2589X_REG_07,BQ2589X_EN_TERM_MASK,
			val << BQ2589X_EN_TERM_SHIFT);
}
  
void bq2589x_set_en_timer(int val)
{
	int ret;
    // charger timer
        ret = bq2589x_update_bits(BQ2589X_REG_07,BQ2589X_EN_TIMER_MASK,
			val << BQ2589X_EN_TIMER_SHIFT);

        ret = bq2589x_sed_update_bits(BQ2589X_REG_07,BQ2589X_EN_TIMER_MASK,
			val << BQ2589X_EN_TIMER_SHIFT);
}

int bq2589x_set_watchdog(int val)
{
	int ret;

	ret = bq2589x_update_bits(BQ2589X_REG_07, BQ2589X_WDT_MASK,
			val << BQ2589X_WDT_SHIFT);

	ret = bq2589x_sed_update_bits(BQ2589X_REG_07, BQ2589X_WDT_MASK,
			val << BQ2589X_WDT_SHIFT);

	return ret;
}

int bq2589x_disable_watchdog_timer(void)
{
	int ret;
	u8 val = BQ2589X_WDT_DISABLE << BQ2589X_WDT_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_07, BQ2589X_WDT_MASK, val);
	ret = bq2589x_sed_update_bits(BQ2589X_REG_07, BQ2589X_WDT_MASK, val);

	return ret;
}

//CON8----------------------------------------------
void bq2589x_set_ir_comp_resistance(int val)
{
	int ret;

	ret = bq2589x_sed_update_bits(BQ2589X_REG_08, BQ2589X_BAT_COMP_MASK,
			val << BQ2589X_BAT_COMP_SHIFT);

	ret = bq2589x_update_bits(BQ2589X_REG_08, BQ2589X_BAT_COMP_MASK,
			val << BQ2589X_BAT_COMP_SHIFT);
}

void bq2589x_set_ir_comp_voltage_clamp(int val)
{
	int ret;

	ret = bq2589x_sed_update_bits(BQ2589X_REG_08, BQ2589X_VCLAMP_MASK,
			val << BQ2589X_VCLAMP_SHIFT);

	ret = bq2589x_update_bits(BQ2589X_REG_08, BQ2589X_VCLAMP_MASK,
			val << BQ2589X_VCLAMP_SHIFT);


}

//CON9-----------------------------------------------
void bq2589x_enter_ship_mode(void)
{
	int ret;

	u8 val = (BQ2589X_BATFET_OFF << BQ2589X_BATFET_DIS_SHIFT);
	u8 val1 = BQ2589X_BATFET_DLY << BQ2589X_BATFET_DLY_SHIFT;

	ret = bq2589x_sed_update_bits(BQ2589X_REG_09, BQ2589X_BATFET_DLY_MASK, val1);
	ret = bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_BATFET_DLY_MASK, val1);

	ret = bq2589x_sed_update_bits(BQ2589X_REG_09, BQ2589X_BATFET_DIS_MASK, val);
	ret = bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_BATFET_DIS_MASK, val);
}

void bq2589x_force_ico(void)
{
	int ret;

	ret = bq2589x_sed_update_bits(BQ2589X_REG_09, BQ2589X_FORCE_ICO_MASK,
			1 << BQ2589X_FORCE_ICO_SHIFT);
}

/*
 * BQ25892 MTK-PE+ Mediatek Pump Express+ Interface
 */
void bq2589x_pumpx_up(void)
{
	int ret;

	ret = bq2589x_sed_update_bits(BQ2589X_REG_09, BQ2589X_PUMPX_UP_MASK,
			1 << BQ2589X_PUMPX_UP_SHIFT);
}

int bq2589x_pumpx_up_done(void)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_09);

	if(val & BQ2589X_PUMPX_UP_MASK)
		return 1;   // not finished
	else
		return 0;   // pumpx up finished
}

void bq2589x_pumpx_down(void)
{
	int ret ;

	ret = bq2589x_sed_update_bits(BQ2589X_REG_09, BQ2589X_PUMPX_DN_MASK,
			1 << BQ2589X_PUMPX_DN_SHIFT);
}

int bq2589x_pumpx_down_done(void)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_09);

	if(val & BQ2589X_PUMPX_DN_MASK) 
		return 1;   // not finished
	else
		return 0;   // pumpx down finished
}

//CONA----------------------------------------------
static int bq2589x_set_otg_volt(int volt)
{
    u8 val = 0;
    int ret;

    if (volt < BQ2589X_BOOSTV_BASE)
        volt = BQ2589X_BOOSTV_BASE;
    if (volt > BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB)
        volt = BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB;
    

	val = ((volt - BQ2589X_BOOSTV_BASE)/BQ2589X_BOOSTV_LSB) << BQ2589X_BOOSTV_SHIFT;
	
	ret = bq2589x_update_bits(BQ2589X_REG_0A,BQ2589X_BOOSTV_MASK,val);

	return ret;
}

static int bq2589x_set_otg_current(int curr)
{
	u8 temp;

	if(curr  == 500)
	temp = BQ2589X_BOOST_LIM_500MA;
	else if(curr == 700)
	temp = BQ2589X_BOOST_LIM_700MA;
	else if(curr == 1100)
	temp = BQ2589X_BOOST_LIM_1100MA;
	else if(curr == 1600)
	temp = BQ2589X_BOOST_LIM_1600MA;
	else if(curr == 1800)
	temp = BQ2589X_BOOST_LIM_1800MA;
	else if(curr == 2100)
	temp = BQ2589X_BOOST_LIM_2100MA;
	else if(curr == 2400)
	temp = BQ2589X_BOOST_LIM_2400MA;
	else
	temp = BQ2589X_BOOST_LIM_1300MA;

    return bq2589x_update_bits(BQ2589X_REG_0A,BQ2589X_BOOST_LIM_MASK,
		    temp << BQ2589X_BOOST_LIM_SHIFT);
}

//CON0B------------------------------------
int bq2589x_get_vbus_type(void)
{
	u8 val = 0;
	int ret;

	ret = bq2589x_sed_read_byte(&val, BQ2589X_REG_0B);
	if (ret < 0)
		return 0;

	val &= BQ2589X_VBUS_STAT_MASK;
	val >>= BQ2589X_VBUS_STAT_SHIFT;

	return val;
}

int bq2589x_get_charging_status(void)
{
    u8 val = 0;
    int ret;

    ret = bq2589x_sed_read_byte(&val, BQ2589X_REG_0B);
    if(ret < 0){
        dev_err(g_bq->dev,"%s Failed to read register 0x0b:%d\n",__func__,ret);
        return ret;
    }
    val &= BQ2589X_CHRG_STAT_MASK;
    val >>= BQ2589X_CHRG_STAT_SHIFT;
    return val;
}

static int bq2589x_charge_status(void)
{
	u8 val = 0;

	bq2589x_read_byte(&val, BQ2589X_REG_0B);
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
	switch(val){
	case BQ2589X_CHRG_STAT_FASTCHG:
	    return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case BQ2589X_CHRG_STAT_PRECHG:
	    return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case BQ2589X_CHRG_STAT_CHGDONE:
	case BQ2589X_CHRG_STAT_IDLE:
	    return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
	    return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

int bq2589x_get_pg_stat(void)
{
	int ret;	
	u8 val;

	ret = bq2589x_sed_read_byte(&val, BQ2589X_REG_0B);
	if(ret < 0){
		dev_err(g_bq->dev,"%s Failed to read register 0x0b:%d\n",__func__,ret);
		return ret;
	}

	val &= BQ2589X_PG_STAT_MASK;
	val >>= BQ2589X_PG_STAT_SHIFT;
	return val;
}

//CON0D-----------------------------
void bq2589x_force_vindpm(int val)
{
	int ret;

	ret = bq2589x_update_bits(BQ2589X_REG_0D, BQ2589X_FORCE_VINDPM_MASK, 
			val << BQ2589X_FORCE_VINDPM_SHIFT);
	ret = bq2589x_sed_update_bits(BQ2589X_REG_0D, BQ2589X_FORCE_VINDPM_MASK, 
			val << BQ2589X_FORCE_VINDPM_SHIFT);
}

void bq2589x_set_input_volt_limit(int volt)
{
	int ret;

        ret = bq2589x_update_bits(BQ2589X_REG_0D,BQ2589X_VINDPM_MASK,
			volt << BQ2589X_VINDPM_SHIFT);
        ret = bq2589x_sed_update_bits(BQ2589X_REG_0D,BQ2589X_VINDPM_MASK,
			volt << BQ2589X_VINDPM_SHIFT);
}

//CON0E-----------------------------
int bq2589x_adc_read_battery_volt(void)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_0E);
	if(ret < 0){
		dev_err(g_bq->dev,"read battery voltage failed :%d\n",ret);
		return ret;
	} else{
		volt = BQ2589X_BATV_BASE +
			((val & BQ2589X_BATV_MASK) >> BQ2589X_BATV_SHIFT) * BQ2589X_BATV_LSB ;
		return volt;
	}
}

int bq2589x_adc_read_battery_volt1(void)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_sed_read_byte(&val, BQ2589X_REG_0E);
	if(ret < 0){
		dev_err(g_bq->dev,"read battery voltage failed :%d\n",ret);
		return ret;
	} else{
		volt = BQ2589X_BATV_BASE +
			((val & BQ2589X_BATV_MASK) >> BQ2589X_BATV_SHIFT) * BQ2589X_BATV_LSB ;
		return volt;
	}
}

//CON0F---------------------------
int bq2589x_adc_read_sys_volt(void)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_sed_read_byte(&val, BQ2589X_REG_0F);
	if(ret < 0){
		dev_err(g_bq->dev,"read system voltage failed :%d\n",ret);
		return ret;
	}
	else{
		volt = BQ2589X_SYSV_BASE +
			((val & BQ2589X_SYSV_MASK) >> BQ2589X_SYSV_SHIFT) * BQ2589X_SYSV_LSB ;
		return volt;
	}
}

//CON10----------------------------
int bq2589x_adc_read_temperature(void)
{
	uint8_t val;
	int temp;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_10);
	if(ret < 0){
		dev_err(g_bq->dev,"read temperature failed :%d\n",ret);
		return ret;
	}
	else{
		temp = BQ2589X_TSPCT_BASE +
			((val & BQ2589X_TSPCT_MASK) >> BQ2589X_TSPCT_SHIFT) * BQ2589X_TSPCT_LSB ;
		return temp;
    }
}

int bq2589x_adc_read_temperature1(void)
{
	uint8_t val;
	int temp;
	int ret;

	ret = bq2589x_sed_read_byte(&val, BQ2589X_REG_10);
	if(ret < 0){
		dev_err(g_bq->dev,"read temperature failed :%d\n",ret);
		return ret;
	}
	else{
		temp = BQ2589X_TSPCT_BASE +
			((val & BQ2589X_TSPCT_MASK) >> BQ2589X_TSPCT_SHIFT) * BQ2589X_TSPCT_LSB ;
		return temp;
    }
}

//CON11---------------------------
int bq2589x_adc_read_charger_volt(void)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_sed_read_byte(&val, BQ2589X_REG_11);
	if(ret < 0) {
		dev_err(g_bq->dev,"read vbus voltage failed :%d\n",ret);
		return ret;
	} else{
		volt = BQ2589X_VBUSV_BASE +
			((val & BQ2589X_VBUSV_MASK) >> BQ2589X_VBUSV_SHIFT) * BQ2589X_VBUSV_LSB ;
		return volt;
	}
}

//CON12--------------------------------
int bq2589x_adc_read_charge_current(void)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_read_byte(&val, BQ2589X_REG_12);
	if(ret < 0){
		dev_err(g_bq->dev,"read charge current failed :%d\n",ret);
		return ret;
	}
	else{
		volt = (int)(BQ2589X_ICHGR_BASE +
				((val & BQ2589X_ICHGR_MASK) >> BQ2589X_ICHGR_SHIFT) * BQ2589X_ICHGR_LSB) ;
		return volt;
	}
}

int bq2589x_adc_read_charge_current1(void)
{
	uint8_t val;
	int volt;
	int ret;

	ret = bq2589x_sed_read_byte(&val, BQ2589X_REG_12);
	if(ret < 0){
		dev_err(g_bq->dev,"read charge current failed :%d\n",ret);
		return ret;
	}
	else{
		volt = (int)(BQ2589X_ICHGR_BASE +
				((val & BQ2589X_ICHGR_MASK) >> BQ2589X_ICHGR_SHIFT) * BQ2589X_ICHGR_LSB) ;
		return volt;
	}
}

//CON13------------VINDPM,IINDPM STATUS---------------------------
void bq2589x_get_vindpm_status(int *data)
{
	int ret;
	u8 val;

	ret = bq2589x_sed_read_byte(&val, BQ2589X_REG_13);

	*data = (val & BQ2589X_VDPM_STAT_MASK) >> BQ2589X_VDPM_STAT_SHIFT;
}

void bq2589x_get_iindpm_status(int *data)
{
	int ret;
	u8 val;

	ret = bq2589x_sed_read_byte(&val, BQ2589X_REG_13);
	*data = (val & BQ2589X_IDPM_STAT_MASK) >> BQ2589X_IDPM_STAT_SHIFT;
}

int bq2589x_get_idpm_limit(void)
{
	int ret;
	u8 val;
	int idpm_limit = 0;

	ret = bq2589x_sed_read_byte(&val, BQ2589X_REG_13);
	idpm_limit = ((val & BQ2589X_IDPM_LIM_MASK) >> BQ2589X_IDPM_LIM_SHIFT) * BQ2589X_IDPM_LIM_LSB + BQ2589X_IDPM_LIM_BASE;
	return idpm_limit;
}

int bq2589x_get_ico_status(void)
{
	int ret;
	u8 val;
	int ico_status;

	ret = bq2589x_sed_read_byte(&val, BQ2589X_REG_14);
	ico_status = (val & BQ2589X_ICO_OPTIMIZED_MASK) >> BQ2589X_ICO_OPTIMIZED_SHIFT; 

	return ico_status;
}

int bq2589x_reset_chip(void)
{
	int ret;
	u8 val = BQ2589X_RESET << BQ2589X_RESET_SHIFT;

	ret = bq2589x_update_bits(BQ2589X_REG_14, BQ2589X_RESET_MASK, val);
	if(ret < 0) return ret;

	ret = bq2589x_sed_update_bits(BQ2589X_REG_14, BQ2589X_RESET_MASK, val);
	if(ret < 0) return ret;

	mdelay(100);

	return 0;
}

void bq2589x_dump_register(void)
{
	int i=0;
	u8 regval;

	for (i=0;i<BQ2589x_REG_NUM;i++)
	{
		bq2589x_read_byte(&regval, i);
		printk("[bq2589x_dump_register] Reg[0x%X]=0x%X\n", i, regval);        
	}

	for (i=0;i<BQ2589x_REG_NUM;i++)
	{
		bq2589x_sed_read_byte(&regval, i);
		printk("[bq2589x_dump_register1] Reg[0x%X]=0x%X\n", i, regval);        
	}

}

static ssize_t bq2589x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bq2589x *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	int count = 0;

	for (addr = 0x0; addr <= 0x14; addr++) {
		bq2589x_read_byte(&val, addr);
		count+=sprintf(buf+count,"[Minor:0x%x] = (0x%0x)\n", 0x00+addr,val);
	}
	for (addr = 0x0; addr <= 0x14; addr++) {
		bq2589x_sed_read_byte(&val, addr);
		count+=sprintf(buf+count,"[Major:0x%x] = (0x%0x)\n",0x00+addr,val);
	}

	return count;
}
static DEVICE_ATTR(regs_dump, S_IRUGO, bq2589x_show_registers, NULL);

static struct attribute *bq2589x_attributes[] = {
	&dev_attr_regs_dump.attr,
	NULL,
};

static const struct attribute_group bq2589x_attr_group = {
	.attrs = bq2589x_attributes,
};

static int bq2589x_detect_device(struct bq2589x* bq)
{
    int ret;
    u8 data;
    int part_no, revision;

    ret = bq2589x_read_byte(&data,BQ2589X_REG_14);
    if(ret == 0){
        part_no = (data & BQ2589X_PN_MASK) >> BQ2589X_PN_SHIFT;
        revision = (data & BQ2589X_DEV_REV_MASK) >> BQ2589X_DEV_REV_SHIFT;
    }

    return ret;
}

//I2C2
void bq2589x_start_charging(void)
{
	/*打开charger之前，需要禁止调ILIM的硬件限流功能*/
	bq2589x_set_en_ilim(0);
	mt_set_gpio_out(GPIO_CHARGER0_EN_PIN, 0);
	bq2589x_enable_charger();// in case of charger enable bit is cleared due to fault
}

//I2C0
void bq2589x_start_charging1(void)
{
	mt_set_gpio_out(GPIO_CHARGER1_EN_PIN, 0);
	bq2589x_enable_charger1();// in case of charger enable bit is cleared due to fault
}

//I2C2 major charger IC
void bq2589x_stop_charging(void)
{
	mt_set_gpio_out(GPIO_CHARGER0_EN_PIN, 1);	
	bq2589x_disable_charger();// in case of charger enable bit is cleared due to fault
}

//I2C0 minor charger IC
void bq2589x_stop_charging1(void)
{
	mt_set_gpio_out(GPIO_CHARGER1_EN_PIN, 1);	
	bq2589x_disable_charger1();// in case of charger enable bit is cleared due to fault
}

void bq2589x_set_otg(int enable)
{
    int ret;

    if(enable){
        ret = bq2589x_enable_otg();
        if(ret < 0){
            dev_err(g_bq->dev,"%s:Failed to enable otg-%d\n",__func__,ret);
            return;
        }
	mt_set_gpio_out(GPIO_CHARGER0_OTG_PIN, 1);
    }
    else{
        ret = bq2589x_disable_otg();
        if(ret < 0){
            dev_err(g_bq->dev,"%s:Failed to disable otg-%d\n",__func__,ret);
        }
    }
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void fast_early_suspend(struct early_suspend *h)
{
#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	fast_charge_current_1C = KAL_FALSE;
	wake_up_bat();
#endif
}

static void fast_late_resume(struct early_suspend *h)
{
#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	if (fast_charge_current_1C == KAL_FALSE) {
		fast_charge_current_1C = KAL_TRUE;
		wake_up_bat();
	}
#endif
}
#endif

void bq2589x_charger_fault_reset(kal_uint32 *data)
{
	charger_fault = *data;
}

kal_uint32 bq2589x_charger_fault_status(void)
{
	return charger_fault; 
}

static void bq2589x_irq_handle(struct work_struct *work)
{
	u8 valb, valc;
	int pg_stat = 0;
	bq2589x_chrg_stat chrg_stat;
	int watchdog_fault, chrg_fault,bat_fault, ntc_fault;
	int vbus_stat = 0;

	mutex_lock(&handler_mutex);
	wake_lock(&g_bq2->charger_wake_lock);

	bq2589x_sed_read_byte(&valb, BQ2589X_REG_0B);
	bq2589x_sed_read_byte(&valc, BQ2589X_REG_0C);

	chrg_stat = (valb & BQ2589X_CHRG_STAT_MASK) >> BQ2589X_CHRG_STAT_SHIFT;
	pg_stat = (valb & BQ2589X_PG_STAT_MASK) >> BQ2589X_PG_STAT_SHIFT;
	vbus_stat = (valb & BQ2589X_VBUS_STAT_MASK) >> BQ2589X_VBUS_STAT_SHIFT;
	
	if (pg_stat != 0)
		wake_up_bat();

	watchdog_fault = (valc & BQ2589X_FAULT_WDT_MASK) >> BQ2589X_FAULT_WDT_SHIFT;
	chrg_fault = (valc & BQ2589X_FAULT_CHRG_MASK) >> BQ2589X_FAULT_CHRG_SHIFT;
	bat_fault = (valc & BQ2589X_FAULT_BAT_MASK) >> BQ2589X_FAULT_BAT_SHIFT;
	ntc_fault = (valc & BQ2589X_FAULT_NTC_MASK) >> BQ2589X_FAULT_NTC_SHIFT;

	if (watchdog_fault == 1 || chrg_fault != BQ2589X_FAULT_CHRG_NORMAL 
			|| bat_fault == 1 || ntc_fault != 0) {
		printk("%s:charger fault occurs\n", __func__);
		charger_fault = KAL_TRUE;
	}
	mt_eint_unmask(CUST_EINT_CHARGER1_EINT_NUM);
	wake_unlock(&g_bq2->charger_wake_lock);
	mutex_unlock(&handler_mutex);
	return;	
}

static void bq2589x_isr(void)
{
        schedule_delayed_work_on(0, &g_bq2->irq_dwork, 0);
}

static void bq2589x_irq_init(void)
{
	mt_set_gpio_dir(GPIO_CHARGER0_INT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CHARGER0_INT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CHARGER0_INT_PIN, GPIO_PULL_UP);
	mt_eint_set_sens(CUST_EINT_CHARGER1_EINT_NUM, MT_EDGE_SENSITIVE);
        mt_eint_set_hw_debounce(CUST_EINT_CHARGER1_EINT_NUM, 0);
        
        mt_eint_registration(CUST_EINT_CHARGER1_EINT_NUM, EINTF_TRIGGER_FALLING, bq2589x_isr, 0);

        mt_eint_unmask(CUST_EINT_CHARGER1_EINT_NUM);

   }

static int bq2589x_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	static struct bq2589x *bq;
	int ret;
	char *name;
	int num;
	int chip_id;
	u8 device_conf = 0;
	u8 batfet_on = 0 << BQ2589X_BATFET_DIS_SHIFT;

	bq = kzalloc(sizeof(struct bq2589x), GFP_KERNEL);
	if (!bq){
		dev_err(&client->dev,"%s:out of memory\n",__func__);
		return -ENOMEM;
	}
	chip_id = id->driver_data;
	
	if (chip_id == BQ25892_I2C0) {

		bq->dev = &client->dev;
		bq->client = client;
		i2c_set_clientdata(client, bq);
		g_bq = bq;

		ret = bq2589x_read_byte(&device_conf, BQ2589X_REG_14);
		if (ret < 0) {
			printk("%s:[BQ25892_I2C0]the slaver addr is not 0x6B\n", __func__);
			g_bq->client->addr = BQ2589x_SLAVE_ADDR0;
		}
		ret = bq2589x_update_bits(BQ2589X_REG_07, BQ2589X_WDT_MASK, BQ2589X_WDT_DISABLE << BQ2589X_WDT_SHIFT);
		ret = bq2589x_update_bits(BQ2589X_REG_09, BQ2589X_BATFET_DIS_MASK, batfet_on);
	} else if (chip_id == BQ25892_I2C2) {
		bq->dev = &client->dev;
		bq->client = client;
		i2c_set_clientdata(client, bq);
		g_bq2 = bq;

		ret = sysfs_create_group(&bq->dev->kobj, &bq2589x_attr_group);
		if (ret) {
			dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);
			goto err_0;
		}
		meizu_sysfslink_register_n(bq->dev, "charger");

		ret = bq2589x_sed_read_byte(&device_conf, BQ2589X_REG_14);
		if (ret < 0) {
			printk("%s:[BQ25892_I2C2]the slaver addr is not 0x6B\n", __func__);
			g_bq2->client->addr = BQ2589x_SLAVE_ADDR0;
		}

#ifdef CONFIG_HAS_EARLYSUSPEND
		bq->fast_handler.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
		bq->fast_handler.suspend = fast_early_suspend;
		bq->fast_handler.resume = fast_late_resume;
		register_early_suspend(&bq->fast_handler);
#endif
		wake_lock_init(&g_bq2->charger_wake_lock, WAKE_LOCK_SUSPEND, "charger wakelock");
		INIT_DELAYED_WORK(&g_bq2->irq_dwork, bq2589x_irq_handle);
		bq2589x_irq_init();
		ret = bq2589x_sed_update_bits(BQ2589X_REG_07, BQ2589X_WDT_MASK, BQ2589X_WDT_DISABLE << BQ2589X_WDT_SHIFT);
	}

/*default disbale charger */
	mt_set_gpio_out(GPIO_CHARGER0_EN_PIN, 1);		
	mt_set_gpio_out(GPIO_CHARGER1_EN_PIN, 1);

	printk("%s:chip_id %d,client addr 0x%02x\n", __func__, chip_id, client->addr);
	return 0;
err_0:
	kfree(bq);
	return ret;
}

static int bq2589x_charger_remove(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);

	sysfs_remove_group(&bq->dev->kobj, &bq2589x_attr_group);
	meizu_sysfslink_unregister(bq->dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&bq->fast_handler);
#endif
	kfree(bq);
	g_bq2 = NULL;
	g_bq = NULL;

	return 0;
}

static void bq2589x_shutdown(struct i2c_client *client)
{	
	if (is_ta_connect == KAL_TRUE)
		mtk_ta_reset_vchr();

	bq2589x_reset_chip();
}

static const struct i2c_device_id bq2589x_id[] = {
    	{ "bq25892", BQ25892_I2C0},
    	{ "bq25892_sed", BQ25892_I2C2},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2589x_id);


static struct i2c_driver bq2589x_charger = {
	.probe		= bq2589x_charger_probe,
	.remove		= bq2589x_charger_remove,
	.shutdown	= bq2589x_shutdown,
	.id_table	= bq2589x_id,
	.driver		= {
		.name	= "bq25892",
	},
};

static int __init bq2589x_init(void)
{    
    int ret=0;
    
    i2c_register_board_info(BQ2589x_BUSNUM0, &i2c_bq2589x, 1);
    i2c_register_board_info(BQ2589x_BUSNUM2, &i2c_bq2589x_sed, 1);

    if(i2c_add_driver(&bq2589x_charger) != 0)
    {
        printk("[bq2589x_init] failed to register bq2589x i2c driver.\n");
    } else {
        printk("[bq2589x_init] Success to register bq2589x i2c driver.\n");
    }
    
    return 0;        
}

static void __exit bq2589x_exit(void)
{
    i2c_del_driver(&bq2589x_charger);
}

module_init(bq2589x_init);
module_exit(bq2589x_exit);

MODULE_DESCRIPTION("TI BQ2589x Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments, Meizu");
