/*
 * bq27532 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * Datasheets:
 * http://focus.ti.com/docs/prod/folders/print/bq27000.html
 * http://focus.ti.com/docs/prod/folders/print/bq27500.html
 * http://www.ti.com/product/bq27411-g1
 * http://www.ti.com/product/bq27421-g1
 * http://www.ti.com/product/bq27425-g1
 * http://www.ti.com/product/bq27441-g1
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/irqs.h>
#include <mach/eint.h>
#include "cust_eint.h"
#include <linux/meizu-sys.h>
#include <mach/battery_common.h>
#include <linux/fs.h>
#include <linux/sched/rt.h>
#include <asm/uaccess.h>
#include <linux/rtc.h>
#include <linux/wakelock.h>
#include <mach/mtk_rtc.h>
#include <mach/bq27532_battery.h>

#define REGS_DUMP_TIMES	(5*HZ)

#define BQ27532_BUSNUM 0
#define bq27532_SLAVE_ADDR  0x55
static struct i2c_board_info __initdata i2c_bq27532 = 
{ 
	I2C_BOARD_INFO("bq27532", bq27532_SLAVE_ADDR)
};

struct bq27532_device_info *di_info = NULL;

static unsigned int poll_interval = 360;
static int battery_exist = 1;
static int dump_regs = 1;
static int init_temp = 15;

module_param(poll_interval, uint, 0664);
MODULE_PARM_DESC(poll_interval, "battery poll interval in seconds - " \
				"0 disables polling");

/* i2c specific code */

/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

static int BQ27532_read_i2c(struct bq27532_device_info *di, u8 reg)
{
    char     cmd_buf[2]={0x00};
    int     readData = 0;
    int      ret=0;
    struct i2c_client *new_client = di->client;

    mutex_lock(&di->bat_i2c_access);
    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;
    new_client->timing = 50;

    cmd_buf[0] = reg;
    ret = i2c_master_send(new_client, &cmd_buf[0], (2<<8 | 1));
    if (ret < 0) 
    {   
        new_client->ext_flag=0;
        mutex_unlock(&di->bat_i2c_access);
        return 0;
    }
    
    readData = cmd_buf[1] << 8 | cmd_buf[0];
    new_client->ext_flag=0;
    
    mutex_unlock(&di->bat_i2c_access);    
    return readData;
}

static void BQ27532_write_i2c(struct bq27532_device_info *di, u8 reg, int value)
{
    char    write_data[3] = {0};
    int     ret=0;
    struct i2c_client *new_client = di->client;
    
    mutex_lock(&di->bat_i2c_access);
    
    write_data[0] = reg;
    write_data[1] = (value & 0xff00) >> 8;
    write_data[2] = value & 0x00ff;
    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
    new_client->timing = 50;
    
    ret = i2c_master_send(new_client, write_data, 3);
    if (ret < 0) 
    {
        new_client->ext_flag=0;
        mutex_unlock(&di->bat_i2c_access);
        return;
    }
    
    new_client->ext_flag=0;
    mutex_unlock(&di->bat_i2c_access);
    return;
}

u8 checksum(u8 *data)
{
    u16 sum = 0;
    int i;

    for (i = 0; i < 32; i++)
        sum += data[i];

    sum &= 0xFF;

    return 0xFF - sum;
}


static int read_dm_block(struct bq27532_device_info *di, u8 subclass,
        u8 offset, u8 *data)
{
    u8 cksum_calc, cksum;
    u8 blk_offset = offset >> 5;

    i2c_smbus_write_byte_data(di->client, BLOCK_DATA_CONTROL, 0);
    msleep(5);

    i2c_smbus_write_byte_data(di->client, BLOCK_DATA_CLASS, subclass);
    msleep(5);

    i2c_smbus_write_byte_data(di->client, DATA_BLOCK, blk_offset);
    msleep(5);

    data = i2c_smbus_read_byte_data(di->client, BLOCK_DATA);

    cksum_calc = checksum(data);
    cksum = i2c_smbus_read_byte_data(di->client, BLOCK_DATA_CHECKSUM);
    if (cksum != cksum_calc) {
        dev_err(di->dev, "%s: error reading subclass %d offset %d\n",
                __func__, subclass, offset);
        return 0;
    }

    return 1;
}


static int bq27532_seal_enabled(struct bq27532_device_info *di)
{
    u8 buf[32];
    u8 op_cfg_b;

    if (!read_dm_block(di, BQ276XX_OP_CFG_B_SUBCLASS,
                BQ276XX_OP_CFG_B_OFFSET, buf)) {
        return 1; /* Err on the side of caution and try to seal */
    }

    op_cfg_b = buf[BQ276XX_OP_CFG_B_OFFSET & 0x1F];

    if (op_cfg_b & BQ276XX_OP_CFG_B_DEF_SEAL_BIT)
        return 1;

    return 0;
}

static inline int sealed(struct bq27532_device_info *di)
{
	kal_int32 value = 0;
	int seal_flag = 0;

	i2c_smbus_write_word_data(di->client, BQ27532_REG_CTRL, 0x0020);
	msleep(5);

	i2c_smbus_write_word_data(di->client, BQ27532_REG_CTRL, 0x0000);
	value = BQ27532_read_i2c(di, BQ27532_REG_CTRL);
	
	seal_flag = (value & 0x2000) >> 13;

	return seal_flag;
}

#define SEAL_UNSEAL_POLLING_RETRY_LIMIT    200
static int seal(struct bq27532_device_info *di)
{
    int i = 0;
    int is_sealed;

    dev_dbg(di->dev, "%s:\n", __func__);

    is_sealed = sealed(di);
    if (is_sealed)
        return is_sealed;

    if (!bq27532_seal_enabled(di)) {
        dev_dbg(di->dev, "%s: sealing is not enabled\n", __func__);
        return is_sealed;
    }

    i2c_smbus_write_word_data(di->client, BQ27532_REG_CTRL, SEAL_SUBCMD);

    while (i < SEAL_UNSEAL_POLLING_RETRY_LIMIT) {
        i++;
        is_sealed = sealed(di);
        if (is_sealed)
            break;
        msleep(10);
    }

    if (!is_sealed)
        dev_err(di->dev, "%s: failed\n", __func__);

    return is_sealed;
}

static int bq27532_battery_reset(struct bq27532_device_info *di)
{
	int val = 0;
	dev_info(di->dev, "Gas Gauge Reset\n");

	BQ27532_write_i2c(di, BQ27532_REG_CTRL, RESET_SUBCMD);

	msleep(10);

	i2c_smbus_write_word_data(di->client, BQ27532_REG_CTRL, 0x0000);
	val = BQ27532_read_i2c(di, BQ27532_REG_CTRL);

	return val;
}

static int exit_cfg_update_mode(struct bq27532_device_info *di)
{
    dev_info(di->dev, "%s:\n", __func__);
    bq27532_battery_reset(di);

    if (seal(di))
        return 1;
    else
        return 0;
}


static int unseal(struct bq27532_device_info *di, u32 key)
{
    i2c_smbus_write_word_data(di->client, BQ27532_REG_CTRL, 0x0414);
    msleep(50);
    i2c_smbus_write_word_data(di->client, BQ27532_REG_CTRL, 0x3672);
    msleep(50);
    i2c_smbus_write_word_data(di->client, BQ27532_REG_CTRL, 0xFFFF);
    msleep(50);
    i2c_smbus_write_word_data(di->client, BQ27532_REG_CTRL, 0xFFFF);
    msleep(50);
    return 1;
}

static int enter_cfg_update_mode(struct bq27532_device_info *di)
{
	int val;
    if (!unseal(di, BQ274XX_UNSEAL_KEY))
        return 0;
    msleep(5);
    i2c_smbus_write_byte_data(di->client, BLOCK_DATA_CONTROL, 0);


	i2c_smbus_write_word_data(di->client, BQ27532_REG_CTRL, 0x0000);
	val = BQ27532_read_i2c(di, BQ27532_REG_CTRL);
	
 //   printk("%s:***********val 0x%04x\n", __func__, val);

    return 1;
}

static int update_dm_block(struct bq27532_device_info *di, u8 subclass,
        u8 offset, u8 data)
{
    u8 old_cksum,new_cksum,temp_sum,old_data;
    u8 blk_offset = offset >> 5;
    int retry_count = 0;

//    printk("%s:*subclass %d, offset %d, data %d\n", __func__, subclass, offset, data);

retry:
    i2c_smbus_write_byte_data(di->client, BLOCK_DATA_CONTROL, 0);
	msleep(5);
    i2c_smbus_write_byte_data(di->client, BLOCK_DATA_CLASS, subclass);
    msleep(5);

    i2c_smbus_write_byte_data(di->client, DATA_BLOCK, blk_offset);
    msleep(5);

    old_data = i2c_smbus_read_byte_data(di->client, BLOCK_DATA+offset%32);
  //  printk("%s:old_data %d\n", __func__, old_data);

    old_cksum = i2c_smbus_read_byte_data(di->client, BLOCK_DATA_CHECKSUM);
   // printk("%s:old_cksum %d\n", __func__, old_cksum);
    temp_sum = (255-old_cksum-old_data)%256;
    new_cksum = 255 - (temp_sum + data)%256;
   // printk("%s:temp_sum %d, new_cksum %d\n", __func__, temp_sum, new_cksum);

    i2c_smbus_write_byte_data(di->client, BLOCK_DATA+offset%32, data);
    msleep(5);

    i2c_smbus_write_byte_data(di->client, BLOCK_DATA_CHECKSUM, new_cksum);
    msleep(200);

    /* Read back and compare to make sure write is successful */
    i2c_smbus_write_byte_data(di->client, DATA_BLOCK, blk_offset);
    msleep(5);
    old_data = i2c_smbus_read_byte_data(di->client, BLOCK_DATA+offset%32);
   // printk("%s:old_data %d\n", __func__, old_data);
    if (old_data != data) {
        dev_err(di->dev, "%s: error updating subclass %d offset %d\n",
                __func__, subclass, offset);
	if (retry_count < 3) {
		retry_count++;
		goto retry;
	}
        return 0;
    } else {
        return 1;
    }
}

static int bq27532_battery_read_charge(struct bq27532_device_info *di, u8 reg)
{
	int charge;

	charge = BQ27532_read_i2c(di, reg);
	if (charge < 0) {
		dev_dbg(di->dev, "error reading charge register %02x: %d\n",
			reg, charge);
		return charge;
	}

	charge *= 1000;

	return charge;
}

static inline int bq27532_battery_read_nac(struct bq27532_device_info *di)
{
	int flags;

	return bq27532_battery_read_charge(di, BQ27532_REG_NAC);
}

static inline int bq27532_battery_read_fcc(struct bq27532_device_info *di)
{
	return bq27532_battery_read_charge(di, BQ27532_REG_FCC);
}

static int bq27532_battery_read_cyct(struct bq27532_device_info *di)
{
	int cyct;

	cyct = BQ27532_read_i2c(di, BQ27532_REG_CYCT);
	if (cyct < 0)
		dev_err(di->dev, "error reading cycle count total\n");

	return cyct;
}

static int overtemperature(struct bq27532_device_info *di, u16 flags)
{
	return flags & BQ27XXX_FLAG_OTC;
}

/*
 * Read flag register.
 * Return < 0 if something fails.
 */
static int bq27532_battery_read_health(struct bq27532_device_info *di)
{
	int tval;

	tval = BQ27532_read_i2c(di, BQ27532_REG_FLAGS);
	if (tval < 0) {
		dev_err(di->dev, "error reading flag register:%d\n", tval);
		return tval;
	}

	if (tval & BQ27XXX_FLAG_SOCF)
		tval = POWER_SUPPLY_HEALTH_DEAD;
	else if (overtemperature(di, tval))
		tval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		tval = POWER_SUPPLY_HEALTH_GOOD;

	return tval;
}

static void copy_to_dm_buf_big_endian(struct bq27532_device_info *di,
	u8 *buf, u8 offset, u8 sz, u32 val)
{
	dev_dbg(di->dev, "%s: offset %d sz %d val %d\n",
		__func__, offset, sz, val);

	switch (sz) {
	case 1:
		buf[offset] = (u8) val;
		break;
	case 2:
		put_unaligned_be16((u16) val, &buf[offset]);
		break;
	case 4:
		put_unaligned_be32(val, &buf[offset]);
		break;
	default:
		dev_err(di->dev, "%s: bad size for dm parameter - %d",
			__func__, sz);
		break;
	}
}

kal_int32 bq27532_battery_current(void)
{
	kal_int32 curr;

	if (!battery_exist)
		return 1000;

	curr = BQ27532_read_i2c(di_info, BQ27532_REG_AVG_CURR);
	if (curr < 0) {
		dev_err(di_info->dev, "error reading current\n");
		return curr;
	}

	return curr;
}

kal_int32 bq27532_battery_current_now(void)
{
	kal_int32 now_curr;

	if (!battery_exist)
		return 1000;

	now_curr = BQ27532_read_i2c(di_info, BQ27532_REG_NOW_CURR);
	if (now_curr < 0) {
		dev_err(di_info->dev, "error reading current\n");
		return now_curr;
	}

	return now_curr;
}

kal_int32 bq27532_battery_voltage(void)
{
	kal_int32 volt;

	if (!battery_exist)
		return 3800;

	volt = BQ27532_read_i2c(di_info, BQ27532_REG_VOLT);

	if (volt < 0) {
		dev_err(di_info->dev, "error reading voltage\n");
		return volt;
	}

	return volt;
}

kal_int32 bq27532_battery_read_temperature(void)
{
	kal_int32 temp;

	if (!battery_exist)
		return 25;

	/*NTC Temperature*/
	temp = BQ27532_read_i2c(di_info, BQ27532_REG_TEMP);
	if (temp < 0) {
		dev_err(di_info->dev, "error reading temperature\n");
		return temp;
	}

	temp = temp / 10 - 273;

	/*解决SOC回升问题*/
	init_temp = get_rtc_spare_fg_value();
	if (init_temp == 0) {
		set_rtc_spare_fg_value(15);
		init_temp = 15;
	} else if (init_temp > 25) {
		printk("The last temp is negtive\n");
		init_temp = init_temp - 256;
	}
	if ((abs(temp - init_temp) >= 5) && (temp < 25)) {
		if (temp == 0) {
			init_temp = 1;
		} else {
			init_temp = temp;
		}
		set_rtc_spare_fg_value(init_temp);
		i2c_smbus_write_word_data(di_info->client, BQ27532_REG_CTRL, 0x001E);
	}

	return temp;
}

int force_get_tbat(kal_bool update)
{
	int bat_temp;

	bat_temp = bq27532_battery_read_temperature();

	return bat_temp;
}
EXPORT_SYMBOL(force_get_tbat);

kal_int32 bq27532_battery_read_int_temperature(void)
{
	kal_int32 temp;

	if (!battery_exist)
		return 25;

	temp = BQ27532_read_i2c(di_info, BQ27532_REG_INT_TEMP);
	if (temp < 0) {
		dev_err(di_info->dev, "error reading temperature\n");
		return temp;
	}

	temp = temp / 10 - 273;

	return temp;
}

kal_int32 bq27532_battery_read_soc(void)
{
	kal_int32 soc = 0;

	if (!battery_exist)
		return 19;

	soc = BQ27532_read_i2c(di_info, BQ27532_REG_SOC);
	if (soc < 0)
		dev_dbg(di_info->dev, "error reading relative State-of-Charge\n");

	return soc;
}

kal_int32 bq27532_battery_read_truesoc(void)
{
	kal_int32 truesoc;

	if (!battery_exist)
		return 19;

	truesoc = BQ27532_read_i2c(di_info, 0x74);
	if (truesoc < 0)
		dev_dbg(di_info->dev, "error reading Truesoc\n");

	return truesoc;
}


kal_int32 bq27532_battery_read_fullchargecapacity(void)
{
	kal_int32 fullcapacity, capacity;

	if (!battery_exist)
		return 0;

	fullcapacity = BQ27532_read_i2c(di_info, BQ27532_REG_FULLCHARGECAPCITY);
	capacity = BQ27532_read_i2c(di_info, 0x0E);
	printk("%s:**************fullchargecapacity %d, capacity %d\n", __func__, fullcapacity, capacity);
	if (fullcapacity < 0){
		dev_dbg(di_info->dev, "error reading relative State-of-Charge\n");
		battery_exist = 0;
	}
	return fullcapacity;
}

kal_int32 bq27532_get_battery_data(int update)
{
	static kal_int32 control, temp, voltage, flag, remg_cap, full_cap;
	static kal_int32 curr, remainingCapacityUnfiltered, fullChargeCapacityUnfiltered;
	static kal_int32 truesoc, fineqpass;
	static kal_int32 int_temp, soc;

	if (!battery_exist)
		return 0;

	if (update)
	{
	control = BQ27532_read_i2c(di_info, BQ27532_REG_CTRL);
	temp = bq27532_battery_read_temperature();
	int_temp = bq27532_battery_read_int_temperature();
	voltage = bq27532_battery_voltage();
	flag = BQ27532_read_i2c(di_info, BQ27532_REG_FLAGS);
	curr = BQ27532_read_i2c(di_info, BQ27532_REG_AVG_CURR);
	full_cap = BQ27532_read_i2c(di_info, BQ27532_REG_FULLCHARGECAPCITY);
	remg_cap = BQ27532_read_i2c(di_info, BQ27532_REG_RM_CAP);
	remainingCapacityUnfiltered = BQ27532_read_i2c(di_info, 0x6C);
	fullChargeCapacityUnfiltered = BQ27532_read_i2c(di_info, 0x70);
	truesoc = BQ27532_read_i2c(di_info, 0x74);
	fineqpass = BQ27532_read_i2c(di_info, 0x24);
	soc = bq27532_battery_read_soc();
	}

#if 1 
	printk("control 0x%02x, temp %d, int_temp %d,voltage %d, flag 0x%02x, remg_cap %d, full_cap %d,curr %d, \n"
			"remainingCapacityUnfiltered %d, fullChargeCapacityUnfiltered %d,truesoc %d, fineqpass %d, soc %d\n",
			control, temp, int_temp, voltage, flag, remg_cap, full_cap, curr,
			remainingCapacityUnfiltered, fullChargeCapacityUnfiltered, truesoc, fineqpass, soc);
#endif
}

static int bq27532_battery_read_fw_version(struct bq27532_device_info *di)
{
	BQ27532_write_i2c(di, BQ27532_REG_CTRL, FW_VER_SUBCMD);

	msleep(10);

	return BQ27532_read_i2c(di, BQ27532_REG_CTRL);
}

static int bq27532_battery_read_device_type(struct bq27532_device_info *di)
{
	BQ27532_write_i2c(di, BQ27532_REG_CTRL, DEV_TYPE_SUBCMD);

	msleep(10);

	return BQ27532_read_i2c(di, BQ27532_REG_CTRL);
}

static int bq27532_battery_read_dataflash_version(struct bq27532_device_info *di)
{
	BQ27532_write_i2c(di, BQ27532_REG_CTRL, DF_VER_SUBCMD);

	msleep(10);

	return BQ27532_read_i2c(di, BQ27532_REG_CTRL);
}

static ssize_t show_firmware_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27532_device_info *di = dev_get_drvdata(dev);
	int ver;

	ver = bq27532_battery_read_fw_version(di);

	return sprintf(buf, "0x%x\n", ver);
}

static ssize_t show_dataflash_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27532_device_info *di = dev_get_drvdata(dev);
	int ver;

	ver = bq27532_battery_read_dataflash_version(di);

	return sprintf(buf, "0x%x\n", ver);
}

static ssize_t show_device_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27532_device_info *di = dev_get_drvdata(dev);
	int dev_type;

	dev_type = bq27532_battery_read_device_type(di);

	return sprintf(buf, "0x%x\n", dev_type);
}

static ssize_t show_reset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bq27532_device_info *di = dev_get_drvdata(dev);

	bq27532_battery_reset(di);

	return sprintf(buf, "okay\n");
}

static ssize_t store_enable_dump_regs(struct device *dev,
		struct device_attribute *attr, char *buf, size_t count)
{
	sscanf(buf, "%d", &dump_regs);
	return count;
}

void dump_battery_status(void)
{
	printk("[dump battery_status][bq27532]: ");
	bq27532_get_battery_data(0);
}
EXPORT_SYMBOL(dump_battery_status);


static DEVICE_ATTR(fw_version, S_IRUGO, show_firmware_version, NULL);
static DEVICE_ATTR(df_version, S_IRUGO, show_dataflash_version, NULL);
static DEVICE_ATTR(device_type, S_IRUGO, show_device_type, NULL);
static DEVICE_ATTR(reset, S_IRUGO, show_reset, NULL);
static DEVICE_ATTR(enable_dump_regs, S_IWUGO, NULL, store_enable_dump_regs);

static struct attribute *bq27532_attributes[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_df_version.attr,
	&dev_attr_device_type.attr,
	&dev_attr_reset.attr,
	&dev_attr_enable_dump_regs.attr,
	NULL
};

static const struct attribute_group bq27532_attr_group = {
	.attrs = bq27532_attributes,
};

static inline int fuelgauge_info_dump(void)
{
	int ret, err;
	static int i;
	struct file *fp = NULL;
	mm_segment_t pos;
	char buf[16] = {0};
	char time_buf[32] = {0};
	struct timespec ts;
	struct rtc_time tm;
	static kal_bool done = KAL_FALSE;
	int buf1;

	/* get the system current time */
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	sprintf(time_buf, "%d-%02d-%02d %02d:%02d:%02d %s",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour + 8, tm.tm_min, tm.tm_sec, " ");

	pos = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open("/data/fuelgauge_datalog.txt", O_RDWR | O_APPEND | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		pr_info("create file failed******\n");
		err = PTR_ERR(fp);
		if (err != -ENOENT )
			pr_err("%s:open the file failed\n", __func__);
		set_fs(pos);
		return err;
	}

	if (done == KAL_FALSE) {
		/* record the current system time */
		err = fp->f_op->write(fp, time_buf, 21, &fp->f_pos);

		for (i = 0; i <= 0x27; i++) {
			sprintf(buf, "0x%02x %s", i," ");
			err = fp->f_op->write(fp, buf, 6, &fp->f_pos);
		}
		done = KAL_TRUE;
	}

	/* record the current system time */
	err = fp->f_op->write(fp, time_buf, 21, &fp->f_pos);

	for (i = 0; i <= 0x27; i++) {
		buf1 = BQ27532_read_i2c(di_info, i);
		if (i == 0x27) {
			sprintf(buf, "%04x %s", buf1,"\n");
		} else {
			sprintf(buf, "%04x %s", buf1," ");
		}
		err = fp->f_op->write(fp, buf, 6, &fp->f_pos);
	}

	set_fs(pos);
	filp_close(fp, NULL);

	return ret;
}

#if defined(CONFIG_BATTERY_INFO_DUMP_SUPPORT)
static void bq27532_dump_regs(struct work_struct *work)
{
	if (dump_regs) {
		fuelgauge_info_dump();
		schedule_delayed_work_on(0, &di_info->dump_dwork, REGS_DUMP_TIMES);
	}
}
#endif

void bq27532_sync_truesoc(void)
{
	int value;

	mutex_lock(&di_info->bat_i2c_access);
	i2c_smbus_write_word_data(di_info->client, BQ27532_REG_CTRL, 0x001E);
	msleep(2000);
	mutex_unlock(&di_info->bat_i2c_access);
}

static void bq27532_irq_handle(struct work_struct *work)
{
	int soc = 0;
	static int last_soc = -1;

	soc = bq27532_battery_read_soc();

	if ((soc == 0) || (soc != last_soc)) {
		pr_info("Update soc:%d\n", soc);
		last_soc = soc;
		wake_up_bat();
	}
}

static void bq27532_isr(void)
{
	wake_lock_timeout(&di_info->fg_int, 2*HZ);
        schedule_delayed_work_on(0, &di_info->irq_dwork, 0);
}

static void bq27532_irq_init(void)
{
	mt_set_gpio_dir(GPIO_BAT_ALERT_INT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_BAT_ALERT_INT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_BAT_ALERT_INT_PIN, GPIO_PULL_UP);
        mt_eint_set_sens(CUST_EINT_LOWBATDET_NUM, MT_EDGE_SENSITIVE);
        mt_eint_set_hw_debounce(CUST_EINT_LOWBATDET_NUM, 0);
        
        mt_eint_registration(CUST_EINT_LOWBATDET_NUM, EINTF_TRIGGER_FALLING, bq27532_isr, 0);

        mt_eint_unmask(CUST_EINT_LOWBATDET_NUM);
}

static int verify_configd_parameters(struct bq27532_device_info *di)
{
	u16 old_config_d = 0;

	enter_cfg_update_mode(di);
	msleep(200);

	i2c_smbus_write_byte_data(di->client, BLOCK_DATA_CONTROL, 0);
	msleep(5);
	i2c_smbus_write_byte_data(di->client, BLOCK_DATA_CLASS, 64);
	msleep(5);

	/*get op config D data*/
	i2c_smbus_write_byte_data(di->client, DATA_BLOCK, (6 >> 5));
	msleep(5);
	old_config_d = i2c_smbus_read_byte_data(di->client, BLOCK_DATA+6%32);

    	seal(di);
	msleep(200);

	printk("%s:, old_config_d 0x%02x\n", __func__, old_config_d);

	if (old_config_d == 0x42) {
		return 1;	
	} else {
		return 0;
	}
}


static int verify_t_parameters(struct bq27532_device_info *di)
{
	u8 old_data_tr1 = 0, old_data_tr2 = 0;
	u8 old_data_tc1 = 0, old_data_tc2 = 0;
	u16 trise_data = 0; 
	u16 tconstant_data = 0;

	enter_cfg_update_mode(di);
	msleep(200);

	i2c_smbus_write_byte_data(di->client, BLOCK_DATA_CONTROL, 0);
	msleep(5);
	i2c_smbus_write_byte_data(di->client, BLOCK_DATA_CLASS, 82);
	msleep(5);

	/*get t_rise data*/
	i2c_smbus_write_byte_data(di->client, DATA_BLOCK, (12 >> 5));
	msleep(5);
	old_data_tr1 = i2c_smbus_read_byte_data(di->client, BLOCK_DATA+12%32);
	i2c_smbus_write_byte_data(di->client, DATA_BLOCK, (13 >> 5));
	msleep(5);
	old_data_tr2 = i2c_smbus_read_byte_data(di->client, BLOCK_DATA+13%32);
	trise_data = old_data_tr1 * 256 + old_data_tr2;  

	/*get t_constant data*/
	i2c_smbus_write_byte_data(di->client, DATA_BLOCK, (14 >> 5));
	msleep(5);
	old_data_tc1 = i2c_smbus_read_byte_data(di->client, BLOCK_DATA+14%32);
	i2c_smbus_write_byte_data(di->client, DATA_BLOCK, (15 >> 5));
	msleep(5);
	old_data_tc2 = i2c_smbus_read_byte_data(di->client, BLOCK_DATA+15%32);
	tconstant_data = old_data_tc1 * 256 + old_data_tc2;  

    	seal(di);
	msleep(200);

	printk("%s:, trise_data %d, tconstant_data %d\n", __func__, trise_data, tconstant_data);
	if (di->battery_cell == "ATL") {
		if ((trise_data == 37) && (tconstant_data == 720))
			return 1;
		else
			return 0;
	} else if (di->battery_cell == "SONY") {
		if ((trise_data == 25) && (tconstant_data == 453))
			return 1;
		else
			return 0;
	} else {
		return 1;
	}
}

static void rom_mode_gauge_dm_init(struct bq27532_device_info *di)
{
	u16 t_rise = 25, t_time_constant = 453;
	u16 t_predict_time=800;
	u16 final_voltage =0;
	u8 op_config_d = 0x42;

	if (di->battery_cell == "ATL") {
		t_rise = 37;
		t_time_constant = 720;
	} else if (di->battery_cell == "SONY") {
		t_rise = 25;
		t_time_constant = 453;
	} else {
		return;
	}
	enter_cfg_update_mode(di);
	msleep(200);

	update_dm_block(di, 82,12, (t_rise&0xff00) >> 8);
	msleep(50);
	update_dm_block(di, 82,13,t_rise&0x00ff);
	msleep(50);

	update_dm_block(di, 82,14,(t_time_constant&0xff00)>>8);
	msleep(50);
	update_dm_block(di, 82,15,t_time_constant&0x00ff);

	update_dm_block(di, 80,60,(t_predict_time&0xff00)>>8);
	msleep(50);
	update_dm_block(di, 80,61,t_predict_time&0x00ff);
	msleep(50);

	update_dm_block(di, 49,4,(final_voltage&0xff00)>>8);
	msleep(50);
	update_dm_block(di, 49,5,final_voltage&0x00ff);
	msleep(50);
	//byte operations
	/*禁止AMB_PRED的功能，这个功能会在充电过程中去计算一个温差值用于补偿电量计温度的误差*/
	update_dm_block(di, 64,6,op_config_d);
	msleep(50);
	update_dm_block(di, 57,28,1);
	exit_cfg_update_mode(di);
	msleep(200);
}

static int __init bq27532_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq27532_device_info *di;
	int retval = 0;
	int fw_ver, device_id;	
	int value = 0, value1 = 0;
	char *battery_pack;
	kal_int32 fullcapacity;
	int version_flag = 0;
	int verify_t_flag = 0;
	int verify_amb_pred = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA)
			|| !i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	di->dev = &client->dev;
	di->client = client;
	i2c_set_clientdata(client, di);
	mutex_init(&di->bat_i2c_access);

	fullcapacity = BQ27532_read_i2c(di, BQ27532_REG_FULLCHARGECAPCITY);
	if (fullcapacity <= 0){
		dev_dbg(di->dev, "error reading relative State-of-Charge, No Battery\n");
		battery_exist = 0;
	}
	
	/*Get the battery pack manufacturer */
	/* 0x01:NVT
	   0x02:飞毛腿
	   0x03是兴旺达
	*/
	i2c_smbus_write_word_data(client, 0x3F, 0x01);
	value = i2c_smbus_read_byte_data(client, 0x5F);
	if (value == 0x01) {
		battery_pack = "NVT_";
	} else if (value == 0x02) {
		battery_pack = "SCUD_";
	} else if (value == 0x03) {
		battery_pack = "SWD_";
	} else {
		battery_pack = "UNKOWN_";
	}

	/*Get the battery cell manufacturer */
	i2c_smbus_write_word_data(client, BQ27532_REG_CTRL, 0x0008);
	value1 = BQ27532_read_i2c(di, BQ27532_REG_CTRL);
	if (value1 == 0x3460) {
		di->battery_cell = "ATL";
		printk("ATL电芯\n");
	} else if (value1 == 0x3481) {
		di->battery_cell = "SONY";
		printk("SONY电芯\n");
	} else {
		di->battery_cell = "UNKOWN";
	}

	BMT_status.manufacturer_name = strcat(battery_pack, di->battery_cell); 

	if (battery_exist) {
		/* --------------解决低温和SOC回升问题----------------------------*/
		/*检测Block 28参数，如果不是0x01，则根据CHEM-ID对Final voltage,T Rise,T time constant,
		  T PredictAmbient Time Constant 以及RLXSMEN (OpConfig D bit 7)，Block 28参数修改
		  */
		/*Step1:解锁sealed模式下，读取Block28参数，确认当前电池是否已烧录最新数据*/
		i2c_smbus_write_word_data(client, 0x3F, 0x01);
		version_flag = i2c_smbus_read_byte_data(client, 0x5C);
	//	verify_t_flag = verify_t_parameters(di);
		verify_amb_pred = verify_configd_parameters(di);
		/*Step2:参数修改*/
		if ((version_flag == 0) || (verify_amb_pred == 0)) {
			rom_mode_gauge_dm_init(di);
		}
	}
	/*-----------------------------end------------------------------------*/
	retval = sysfs_create_group(&client->dev.kobj, &bq27532_attr_group);
	if (retval) 
		dev_err(&client->dev, "could not create sysfs files\n");

	retval = meizu_sysfslink_register_n(di->dev, "fuelgauge");
	if (retval < 0)
		dev_err(&client->dev, "could not create meizu sysfs files\n");

	di_info = di;

	wake_lock_init(&di->fg_int, WAKE_LOCK_SUSPEND, "FG_INT suspend wakelock");

        INIT_DELAYED_WORK(&di_info->irq_dwork, bq27532_irq_handle);
	bq27532_irq_init();

#ifdef CONFIG_BATTERY_INFO_DUMP_SUPPORT
        INIT_DELAYED_WORK(&di_info->dump_dwork, bq27532_dump_regs);
        schedule_delayed_work_on(0, &di_info->dump_dwork, REGS_DUMP_TIMES);
#endif

	return 0;

batt_failed_3:
	kfree(di);
	return retval;
}

static int bq27532_battery_remove(struct i2c_client *client)
{
	struct bq27532_device_info *di = i2c_get_clientdata(client);

	i2c_set_clientdata(di->client, NULL);
	kfree(di);

	return 0;
}

static int bq27532_suspend(struct i2c_client *client, pm_message_t mesg)
{
        mt_eint_unmask(CUST_EINT_LOWBATDET_NUM);

	return 0;
}

static int bq27532_resume(struct i2c_client *client)
{
	mt_eint_mask(CUST_EINT_LOWBATDET_NUM);	

	return 0;
}

static const struct i2c_device_id bq27532_id[] = {
	{ "bq27532", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, bq27532_id);

static struct i2c_driver bq27532_battery_driver = {
	.driver = {
		.name = "bq27532",
		.owner = THIS_MODULE,
	},
	.probe = bq27532_battery_probe,
	.remove = bq27532_battery_remove,
	.suspend = bq27532_suspend,
	.resume = bq27532_resume,
	.id_table = bq27532_id,
};

static inline int __init bq27532_battery_i2c_init(void)
{
	i2c_register_board_info(BQ27532_BUSNUM, &i2c_bq27532, 1);
	int ret = i2c_add_driver(&bq27532_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register bq27532 i2c driver\n");

	return ret;
}

static inline void __exit bq27532_battery_i2c_exit(void)
{
	i2c_del_driver(&bq27532_battery_driver);
}

/*
 * Module stuff
 */

static int __init bq27532_battery_init(void)
{
	int ret;

	ret = bq27532_battery_i2c_init();
	if (ret)
		return ret;

	return ret;
}
module_init(bq27532_battery_init);

static void __exit bq27532_battery_exit(void)
{
	bq27532_battery_i2c_exit();
}
module_exit(bq27532_battery_exit);

MODULE_DESCRIPTION("bq27532 battery monitor driver");
MODULE_LICENSE("GPL");
