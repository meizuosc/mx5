/*
* Copyright (C) 2012 Invensense, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/
#define pr_fmt(fmt) "inv_mpu: " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/crc32.h>


#include "inv_mpu_iio.h"

#ifdef CONFIG_DTS_INV_MPU_IIO
#include "inv_mpu_dts.h"
#endif

static const struct inv_hw_s hw_info[INV_NUM_PARTS] = {
	{128, "ICM20645"},
	{128, "ICM10340"},
};
static int debug_mem_read_addr = 0x900;
static char debug_reg_addr = 0x6;

/*
 * inv_firmware_loaded_store() -  calling this function will change
 *                        firmware load
 */
static ssize_t inv_firmware_loaded_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	int result, data;

	result = kstrtoint(buf, 10, &data);
	if (result)
		return -EINVAL;

	if (data)
		return -EINVAL;
	st->chip_config.firmware_loaded = 0;

	return count;

}

static int inv_dry_run_dmp(struct iio_dev *indio_dev)
{
	struct inv_mpu_state *st = iio_priv(indio_dev);

	st->smd.on = 1;
	inv_check_sensor_on(st);
	st->trigger_state = EVENT_TRIGGER;
	set_inv_enable(indio_dev);
	msleep(DRY_RUN_TIME);
	st->smd.on = 0;
	inv_check_sensor_on(st);
	st->trigger_state = EVENT_TRIGGER;
	set_inv_enable(indio_dev);

	return 0;
}

static ssize_t _dmp_bias_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int result, data;

	if (!st->chip_config.firmware_loaded)
		return -EINVAL;

	result = inv_switch_power_in_lp(st, true);
	if (result)
		return result;

	result = kstrtoint(buf, 10, &data);
	if (result)
		goto dmp_bias_store_fail;
	switch (this_attr->address) {
	case ATTR_DMP_ACCEL_X_DMP_BIAS:
		if (data)
			st->sensor_acurracy_flag[SENSOR_ACCEL_ACCURACY] =
							DEFAULT_ACCURACY;
		result = write_be32_to_mem(st, data, ACCEL_BIAS_X);
		if (result)
			goto dmp_bias_store_fail;
		st->input_accel_dmp_bias[0] = data;
		break;
	case ATTR_DMP_ACCEL_Y_DMP_BIAS:
		result = write_be32_to_mem(st, data, ACCEL_BIAS_Y);
		if (result)
			goto dmp_bias_store_fail;
		st->input_accel_dmp_bias[1] = data;
		break;
	case ATTR_DMP_ACCEL_Z_DMP_BIAS:
		result = write_be32_to_mem(st, data, ACCEL_BIAS_Z);
		if (result)
			goto dmp_bias_store_fail;
		st->input_accel_dmp_bias[2] = data;
		break;
	case ATTR_DMP_GYRO_X_DMP_BIAS:
		if (data)
			st->sensor_acurracy_flag[SENSOR_GYRO_ACCURACY] =
							DEFAULT_ACCURACY;
		result = write_be32_to_mem(st, data, GYRO_BIAS_X);
		if (result)
			goto dmp_bias_store_fail;
		st->input_gyro_dmp_bias[0] = data;
		break;
	case ATTR_DMP_GYRO_Y_DMP_BIAS:
		result = write_be32_to_mem(st, data, GYRO_BIAS_Y);
		if (result)
			goto dmp_bias_store_fail;
		st->input_gyro_dmp_bias[1] = data;
		break;
	case ATTR_DMP_GYRO_Z_DMP_BIAS:
		result = write_be32_to_mem(st, data, GYRO_BIAS_Z);
		if (result)
			goto dmp_bias_store_fail;
		st->input_gyro_dmp_bias[2] = data;
		break;
	case ATTR_DMP_SC_AUTH:
		result = write_be32_to_mem(st, data, st->aut_key_in);
		if (result)
			goto dmp_bias_store_fail;
		result = write_be32_to_mem(st, 0, st->aut_key_out);
		if (result)
			goto dmp_bias_store_fail;
		/* dry run DMP to get the auth key output */
		inv_dry_run_dmp(indio_dev);
		break;
	case ATTR_DMP_MAGN_X_DMP_BIAS:
		result = write_be32_to_mem(st, data, CPASS_BIAS_X);
		if (result)
			goto dmp_bias_store_fail;
		st->input_compass_dmp_bias[0] = data;
		break;
	case ATTR_DMP_MAGN_Y_DMP_BIAS:
		result = write_be32_to_mem(st, data, CPASS_BIAS_Y);
		if (result)
			goto dmp_bias_store_fail;
		st->input_compass_dmp_bias[1] = data;
		break;
	case ATTR_DMP_MAGN_Z_DMP_BIAS:
		result = write_be32_to_mem(st, data, CPASS_BIAS_Z);
		if (result)
			goto dmp_bias_store_fail;
		st->input_compass_dmp_bias[2] = data;
		break;
	case ATTR_DMP_MISC_GYRO_RECALIBRATION:
		result = write_be32_to_mem(st, 0, GYRO_LAST_TEMPR);
		if (result)
			goto dmp_bias_store_fail;
		break;
	case ATTR_DMP_MISC_ACCEL_RECALIBRATION:
	{
		u8 d[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		int i;
		u32 d1[] = {0, 0, 0, 0,
				0, 0, 0, 0, 0, 0,
				3276800, 3276800, 3276800, 3276800};
		u32 *w_d;

		if (data) {
			result = inv_write_2bytes(st, ACCEL_CAL_RESET, 1);
			if (result)
				goto dmp_bias_store_fail;
			result = mem_w(ACCEL_PRE_SENSOR_DATA, ARRAY_SIZE(d), d);
			w_d = d1;
		} else {
			w_d = st->accel_covariance;
		}
		for (i = 0; i < ARRAY_SIZE(d1); i++) {
			result = write_be32_to_mem(st, w_d[i],
					ACCEL_COVARIANCE + i * sizeof(int));
			if (result)
				goto dmp_bias_store_fail;
		}

		break;
	}
	case ATTR_DMP_PARAMS_ACCEL_CALIBRATION_THRESHOLD:
		result = write_be32_to_mem(st, data, ACCEL_VARIANCE_THRESH);
		if (result)
			goto dmp_bias_store_fail;
		st->accel_calib_threshold = data;
		break;
	/* this serves as a divider of calibration rate, 0->225, 3->55 */
	case ATTR_DMP_PARAMS_ACCEL_CALIBRATION_RATE:
		if (data < 0)
			data = 0;
		result = inv_write_2bytes(st, ACCEL_CAL_RATE, data);
		if (result)
			goto dmp_bias_store_fail;
		st->accel_calib_rate = data;
		break;
	case ATTR_DMP_DEBUG_MEM_READ:
		debug_mem_read_addr = data;
		break;
	case ATTR_DMP_DEBUG_MEM_WRITE:
		inv_write_2bytes(st, debug_mem_read_addr, data);
		break;
	default:
		break;
	}

dmp_bias_store_fail:
	if (result)
		return result;
	result = inv_switch_power_in_lp(st, false);
	if (result)
		return result;

	return count;
}


static int inv_set_accel_bias_reg(struct inv_mpu_state *st,
		int accel_bias, int axis)
{
	int accel_reg_bias;
	u8 addr;
	u8 d[2];
	int result = 0;

	inv_set_bank(st, BANK_SEL_1);

	switch (axis) {
	case 0:
		/* X */
		addr = REG_XA_OFFS_H;
		break;
	case 1:
		/* Y */
		addr = REG_YA_OFFS_H;
		break;
	case 2:
		/* Z* */
		addr = REG_ZA_OFFS_H;
		break;
	default:
		result = -EINVAL;
		goto accel_bias_set_err;
	}

	result = inv_plat_read(st, addr, 2, d);
	if (result)
		goto accel_bias_set_err;
	accel_reg_bias = ((int)d[0]<<8) | d[1];

	/* accel_bias is 2g scaled by 1<<16.
	 * Convert to 16g, and mask bit0 */
	accel_reg_bias -= ((accel_bias / 8 / 65536) & ~1);

	d[0] = (accel_reg_bias >> 8) & 0xff;
	d[1] = (accel_reg_bias) & 0xff;
	result = inv_plat_single_write(st, addr, d[0]);
	if (result)
		goto accel_bias_set_err;
	result = inv_plat_single_write(st, addr + 1, d[1]);
	if (result)
		goto accel_bias_set_err;

accel_bias_set_err:
	inv_set_bank(st, BANK_SEL_0);
	return result;
}

static int inv_set_gyro_bias_reg(struct inv_mpu_state *st,
		const int gyro_bias, int axis)
{
	int gyro_reg_bias;
	u8 addr;
	u8 d[2];
	int result = 0;

	inv_set_bank(st, BANK_SEL_2);

	switch (axis) {
	case 0:
		/* X */
		addr = REG_XG_OFFS_USR_H;
		break;
	case 1:
		/* Y */
		addr = REG_YG_OFFS_USR_H;
		break;
	case 2:
		/* Z */
		addr = REG_ZG_OFFS_USR_H;
		break;
	default:
		result = -EINVAL;
		goto gyro_bias_set_err;
	}

	/* gyro_bias is 2000dps scaled by 1<<16.
	 * Convert to 1000dps */
	gyro_reg_bias = (-gyro_bias * 2 / 65536);

	d[0] = (gyro_reg_bias >> 8) & 0xff;
	d[1] = (gyro_reg_bias) & 0xff;
	result = inv_plat_single_write(st, addr, d[0]);
	if (result)
		goto gyro_bias_set_err;
	result = inv_plat_single_write(st, addr + 1, d[1]);
	if (result)
		goto gyro_bias_set_err;

gyro_bias_set_err:
	inv_set_bank(st, BANK_SEL_0);
	return result;
}

static ssize_t _bias_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int result, data;

	result = inv_switch_power_in_lp(st, true);
	if (result)
		return result;

	result = kstrtoint(buf, 10, &data);
	if (result)
		goto bias_store_fail;
	switch (this_attr->address) {
	case ATTR_ACCEL_X_OFFSET:
		result = inv_set_accel_bias_reg(st, data, 0);
		if (result)
			goto bias_store_fail;
		st->input_accel_bias[0] = data;
		break;
	case ATTR_ACCEL_Y_OFFSET:
		result = inv_set_accel_bias_reg(st, data, 1);
		if (result)
			goto bias_store_fail;
		st->input_accel_bias[1] = data;
		break;
	case ATTR_ACCEL_Z_OFFSET:
		result = inv_set_accel_bias_reg(st, data, 2);
		if (result)
			goto bias_store_fail;
		st->input_accel_bias[2] = data;
		break;
	case ATTR_GYRO_X_OFFSET:
		result = inv_set_gyro_bias_reg(st, data, 0);
		if (result)
			goto bias_store_fail;
		st->input_gyro_bias[0] = data;
		break;
	case ATTR_GYRO_Y_OFFSET:
		result = inv_set_gyro_bias_reg(st, data, 1);
		if (result)
			goto bias_store_fail;
		st->input_gyro_bias[1] = data;
		break;
	case ATTR_GYRO_Z_OFFSET:
		result = inv_set_gyro_bias_reg(st, data, 2);
		if (result)
			goto bias_store_fail;
		st->input_gyro_bias[2] = data;
		break;
	default:
		break;
	}

bias_store_fail:
	if (result)
		return result;
	result = inv_switch_power_in_lp(st, false);
	if (result)
		return result;

	return count;
}
static ssize_t inv_dmp_bias_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	int result;

	mutex_lock(&indio_dev->mlock);
	result = _dmp_bias_store(dev, attr, buf, count);
	mutex_unlock(&indio_dev->mlock);

	return result;
}

static ssize_t inv_bias_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	int result;

	mutex_lock(&indio_dev->mlock);
	result = _bias_store(dev, attr, buf, count);
	mutex_unlock(&indio_dev->mlock);

	return result;
}

static ssize_t inv_debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int result, data;

	result = kstrtoint(buf, 10, &data);
	if (result)
		return result;
	return count;
	switch (this_attr->address) {
	case ATTR_DMP_LP_EN_OFF:
		st->chip_config.lp_en_mode_off = !!data;
		inv_switch_power_in_lp(st, !!data);
		break;
	case ATTR_DMP_CLK_SEL:
		st->chip_config.clk_sel = !!data;
		inv_switch_power_in_lp(st, !!data);
		break;
	case ATTR_DEBUG_REG_ADDR:
		debug_reg_addr = data;
		break;
	case ATTR_DEBUG_REG_WRITE:
		inv_plat_single_write(st, debug_reg_addr, data);
		break;
	case ATTR_DEBUG_WRITE_CFG:
		break;
	}
	return count;
}
static ssize_t _misc_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int result, data;

	if (!st->chip_config.firmware_loaded)
		return -EINVAL;
	result = inv_switch_power_in_lp(st, true);
	if (result)
		return result;
	result = kstrtoint(buf, 10, &data);
	if (result)
		return result;
	switch (this_attr->address) {
	case ATTR_DMP_LOW_POWER_GYRO_ON:
		st->chip_config.low_power_gyro_on = !!data;
		break;
	case ATTR_DMP_DEBUG_DETERMINE_ENGINE_ON:
		st->debug_determine_engine_on = !!data;
		break;
	case ATTR_GYRO_SCALE:
		if (data > 3)
			return -EINVAL;
		st->chip_config.fsr = data;
		result = inv_set_gyro_sf(st);

		return result;
	case ATTR_ACCEL_SCALE:
		if (data > 3)
			return -EINVAL;
		st->chip_config.accel_fs = data;
		result = inv_set_accel_sf(st);

		return result;
	case ATTR_DMP_PED_INT_ON:
		result = inv_write_cntl(st, PEDOMETER_INT_EN << 8, !!data,
							MOTION_EVENT_CTL);
		if (result)
			return result;
		st->ped.int_on = !!data;

		return 0;
	case ATTR_DMP_PED_STEP_THRESH:
		result = inv_write_2bytes(st, PEDSTD_SB, data);
		if (result)
			return result;
		st->ped.step_thresh = data;

		return 0;
	case ATTR_DMP_PED_INT_THRESH:
		result = inv_write_2bytes(st, PEDSTD_SB2, data);
		if (result)
			return result;
		st->ped.int_thresh = data;

		return 0;
	default:
		return -EINVAL;
	}
	st->trigger_state = MISC_TRIGGER;
	result = set_inv_enable(indio_dev);

	return result;
}

/*
 * inv_misc_attr_store() -  calling this function will store current
 *                        dmp parameter settings
 */
static ssize_t inv_misc_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	int result;

	mutex_lock(&indio_dev->mlock);
	result = _misc_attr_store(dev, attr, buf, count);
	mutex_unlock(&indio_dev->mlock);
	if (result)
		return result;

	return count;
}

static ssize_t _debug_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int result, data;

	if (!st->chip_config.firmware_loaded)
		return -EINVAL;
	if (!st->debug_determine_engine_on)
		return -EINVAL;

	result = kstrtoint(buf, 10, &data);
	if (result)
		return result;
	switch (this_attr->address) {
	case ATTR_DMP_IN_ANGLVEL_ACCURACY_ENABLE:
		st->sensor_accuracy[SENSOR_GYRO_ACCURACY].on = !!data;
		break;
	case ATTR_DMP_IN_ACCEL_ACCURACY_ENABLE:
		st->sensor_accuracy[SENSOR_ACCEL_ACCURACY].on = !!data;
		break;
	case ATTR_DMP_ACCEL_CAL_ENABLE:
		st->accel_cal_enable = !!data;
		break;
	case ATTR_DMP_GYRO_CAL_ENABLE:
		st->gyro_cal_enable = !!data;
		break;
	case ATTR_DMP_EVENT_INT_ON:
		st->chip_config.dmp_event_int_on = !!data;
		break;
	case ATTR_DMP_ON:
		st->chip_config.dmp_on = !!data;
		break;
	case ATTR_GYRO_ENABLE:
		st->chip_config.gyro_enable = !!data;
		break;
	case ATTR_ACCEL_ENABLE:
		st->chip_config.accel_enable = !!data;
		break;
	case ATTR_COMPASS_ENABLE:
		st->chip_config.compass_enable = !!data;
		break;
	default:
		return -EINVAL;
	}
	st->trigger_state = DEBUG_TRIGGER;
	result = set_inv_enable(indio_dev);
	if (result)
		return result;

	return count;
}

/*
 * inv_debug_attr_store() -  calling this function will store current
 *                        dmp parameter settings
 */
static ssize_t inv_debug_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	int result;

	mutex_lock(&indio_dev->mlock);
	result = _debug_attr_store(dev, attr, buf, count);
	mutex_unlock(&indio_dev->mlock);

	return result;
}

static int inv_rate_convert(struct inv_mpu_state *st, int ind, int data)
{
	int t, out, out1, out2;

	out = MPU_INIT_SENSOR_RATE;
	if ((SENSOR_L_MAG == ind) || (SENSOR_L_MAG_WAKE == ind)) {
		if ((MSEC_PER_SEC / st->slave_compass->rate_scale) < data)
			out = MSEC_PER_SEC / st->slave_compass->rate_scale;
		else
			out = data;
	} else if ((SENSOR_L_ALS == ind) || (SENSOR_L_ALS_WAKE == ind)) {
		if (data > MAX_ALS_RATE)
			out = MAX_ALS_RATE;
		else
			out = data;
	} else if ((SENSOR_L_PRESSURE == ind) ||
					(SENSOR_L_PRESSURE_WAKE == ind)) {
		if (data > MAX_PRESSURE_RATE)
			out = MAX_PRESSURE_RATE;
		else
			out = data;
	} else {
		t = MPU_DEFAULT_DMP_FREQ / data;
		if (!t)
			t = 1;
		out1 = MPU_DEFAULT_DMP_FREQ / (t + 1);
		out2 = MPU_DEFAULT_DMP_FREQ / t;
		if (abs(out1 - data) < abs(out2 - data))
			out = out1;
		else
			out = out2;
	}

	return out;
}

static ssize_t inv_sensor_rate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	return sprintf(buf, "%d\n", st->sensor_l[this_attr->address].rate);
}
static ssize_t inv_sensor_rate_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int data, rate, ind;
	int result;

	if (!st->chip_config.firmware_loaded) {
		pr_err("sensor_rate_store: firmware not loaded\n");
		return -EINVAL;
	}
	result = kstrtoint(buf, 10, &data);
	if (result)
		return -EINVAL;
	if (data <= 0) {
		pr_err("sensor_rate_store: invalid data=%d\n", data);
		return -EINVAL;
	}

	ind = this_attr->address;
	rate = inv_rate_convert(st, ind, data);

	INV_INFO("sensor:%d, data:%d, rate:%d, ori_rate:%d\n", ind, data, rate, st->sensor_l[ind].rate);

	if (rate == st->sensor_l[ind].rate)
		return count;
	mutex_lock(&indio_dev->mlock);
	st->sensor_l[ind].rate = rate;
	st->trigger_state = DATA_TRIGGER;
	inv_check_sensor_rate(st);
	inv_check_sensor_on(st);
	if (st->sensor_l[ind].on) {
		result = set_inv_enable(indio_dev);
		if (result) {
			mutex_unlock(&indio_dev->mlock);
			return result;
		}
	}
	mutex_unlock(&indio_dev->mlock);

	return count;
}

static ssize_t inv_sensor_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	return sprintf(buf, "%d\n", st->sensor_l[this_attr->address].on);
}
static ssize_t inv_sensor_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int data, on, ind;
	int result;

	INV_INFO("sensor enable\n");

	if (!st->chip_config.firmware_loaded) {
		pr_err("sensor_on store: firmware not loaded\n");
		return -EINVAL;
	}
	result = kstrtoint(buf, 10, &data);
	if (result)
		return -EINVAL;
	if (data < 0) {
		pr_err("sensor_on_store: invalid data=%d\n", data);
		return -EINVAL;
	}
	ind = this_attr->address;
	on = !!data;
	if (on == st->sensor_l[ind].on)
		return count;
	mutex_lock(&indio_dev->mlock);
	st->sensor_l[ind].on = on;

	INV_INFO("sensor:%d--en:%d, enter mutex lock\n",ind,on);

	mutex_lock(&st->inv_suspend_mutex);
	if( ind==SENSOR_L_ACCEL_WAKE ) {
		INV_INFO("wake acc,sensor:%d,--on:%d\n", ind, on);
		if( !!on ) {
			INV_INFO("will enable wake acc, sensor:%d--on:%d, irq_st:%s\n", ind, on, st->irq_should_disable == DISABLE?"disable":"enable");
			if( st->irq_should_disable == DISABLE ) {
				enable_irq(st->irq);
			}

			atomic_set(&st->acc_wake, ENABLE);
		} else {
			INV_INFO("will enable wake acc, sensor:%d--on:%d, irq_st:%s\n", ind, on, st->irq_should_disable == DISABLE?"disable":"enable");
			if( st->irq_should_disable == DISABLE ) {
				disable_irq_nosync(st->irq);
			}

			atomic_set(&st->acc_wake, DISABLE);
		}
	}
	mutex_unlock(&st->inv_suspend_mutex);

	st->trigger_state = RATE_TRIGGER;
	inv_check_sensor_on(st);
	inv_check_sensor_rate(st);
	if (on && (!st->sensor_l[ind].rate)) {
		mutex_unlock(&indio_dev->mlock);
		return count;
	}
	result = set_inv_enable(indio_dev);
	mutex_unlock(&indio_dev->mlock);
	if (result)
		return result;

	return count;
}

static ssize_t inv_check_l_step(struct inv_mpu_state *st)
{
	if (st->step_counter_l_on || st->step_counter_wake_l_on)
		st->ped.on = true;
	else
		st->ped.on = false;

	return 0;
}
static ssize_t _basic_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int data;
	int result;

	if (!st->chip_config.firmware_loaded)
		return -EINVAL;
	result = kstrtoint(buf, 10, &data);
	if (result || (data < 0))
		return -EINVAL;

	switch (this_attr->address) {
	case ATTR_DMP_PED_ON:
		if ((!!data) == st->ped.on)
			return count;
		st->ped.on = !!data;
		break;
	case ATTR_DMP_SMD_ENABLE:
		if ((!!data) == st->smd.on)
			return count;
		st->smd.on = !!data;
		break;
	case ATTR_DMP_TILT_ENABLE:
		if ((!!data) == st->chip_config.tilt_enable)
			return count;
		st->chip_config.tilt_enable = !!data;
		break;
	case ATTR_DMP_PICK_UP_ENABLE:
		if ((!!data) == st->chip_config.pick_up_enable)
			return count;
		st->chip_config.pick_up_enable = !!data;
		break;
	case ATTR_DMP_STEP_DETECTOR_ON:
		st->step_detector_l_on = !!data;
		break;
	case ATTR_DMP_STEP_DETECTOR_WAKE_ON:
		st->step_detector_wake_l_on = !!data;
		break;
	case ATTR_DMP_ACTIVITY_ON:
		if ((!!data) == st->chip_config.activity_on)
			return count;
		st->chip_config.activity_on = !!data;
		break;
	case ATTR_DMP_STEP_COUNTER_ON:
		st->step_counter_l_on = !!data;
		break;
	case ATTR_DMP_STEP_COUNTER_WAKE_ON:
		st->step_counter_wake_l_on = !!data;
		break;
	case ATTR_DMP_BATCHMODE_TIMEOUT:
		if (data == st->batch.timeout)
			return count;
		st->batch.timeout = data;
		break;
	default:
		return -EINVAL;
	};
	inv_check_l_step(st);
	inv_check_sensor_on(st);

	st->trigger_state = EVENT_TRIGGER;
	result = set_inv_enable(indio_dev);
	if (result)
		return result;

	return count;
}

/*
 * inv_basic_attr_store() -  calling this function will store current
 *                        non-dmp parameter settings
 */
static ssize_t inv_basic_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	int result;

	mutex_lock(&indio_dev->mlock);
	result = _basic_attr_store(dev, attr, buf, count);

	mutex_unlock(&indio_dev->mlock);

	return result;
}

/*
 * inv_attr_bias_show() -  calling this function will show current
 *                        dmp gyro/accel bias.
 */
static ssize_t _attr_bias_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int axes, addr, result, dmp_bias;
	int sensor_type;

	switch (this_attr->address) {
	case ATTR_ANGLVEL_X_CALIBBIAS:
		return sprintf(buf, "%d\n", st->gyro_bias[0]);
	case ATTR_ANGLVEL_Y_CALIBBIAS:
		return sprintf(buf, "%d\n", st->gyro_bias[1]);
	case ATTR_ANGLVEL_Z_CALIBBIAS:
		return sprintf(buf, "%d\n", st->gyro_bias[2]);
	case ATTR_ACCEL_X_CALIBBIAS:
		return sprintf(buf, "%d\n", st->accel_bias[0]);
	case ATTR_ACCEL_Y_CALIBBIAS:
		return sprintf(buf, "%d\n", st->accel_bias[1]);
	case ATTR_ACCEL_Z_CALIBBIAS:
		return sprintf(buf, "%d\n", st->accel_bias[2]);
	case ATTR_DMP_ACCEL_X_DMP_BIAS:
		axes = 0;
		addr = ACCEL_BIAS_X;
		sensor_type = SENSOR_ACCEL;
		break;
	case ATTR_DMP_ACCEL_Y_DMP_BIAS:
		axes = 1;
		addr = ACCEL_BIAS_Y;
		sensor_type = SENSOR_ACCEL;
		break;
	case ATTR_DMP_ACCEL_Z_DMP_BIAS:
		axes = 2;
		addr = ACCEL_BIAS_Z;
		sensor_type = SENSOR_ACCEL;
		break;
	case ATTR_DMP_GYRO_X_DMP_BIAS:
		axes = 0;
		addr = GYRO_BIAS_X;
		sensor_type = SENSOR_GYRO;
		break;
	case ATTR_DMP_GYRO_Y_DMP_BIAS:
		axes = 1;
		addr = GYRO_BIAS_Y;
		sensor_type = SENSOR_GYRO;
		break;
	case ATTR_DMP_GYRO_Z_DMP_BIAS:
		axes = 2;
		addr = GYRO_BIAS_Z;
		sensor_type = SENSOR_GYRO;
		break;
	case ATTR_DMP_MAGN_X_DMP_BIAS:
		axes = 0;
		addr = CPASS_BIAS_X;
		sensor_type = SENSOR_COMPASS;
		break;
	case ATTR_DMP_MAGN_Y_DMP_BIAS:
		axes = 1;
		addr = CPASS_BIAS_Y;
		sensor_type = SENSOR_COMPASS;
		break;
	case ATTR_DMP_MAGN_Z_DMP_BIAS:
		axes = 2;
		addr = CPASS_BIAS_Z;
		sensor_type = SENSOR_COMPASS;
		break;
	case ATTR_DMP_SC_AUTH:
		axes = 0;
		addr = st->aut_key_out;
		sensor_type = -1;
		break;
	default:
		return -EINVAL;
	}
	result = inv_switch_power_in_lp(st, true);
	if (result)
		return result;
	result = read_be32_from_mem(st, &dmp_bias, addr);
	if (result)
		return result;
	inv_switch_power_in_lp(st, false);
	if (SENSOR_GYRO == sensor_type)
		st->input_gyro_dmp_bias[axes] = dmp_bias;
	else if (SENSOR_ACCEL == sensor_type)
		st->input_accel_dmp_bias[axes] = dmp_bias;
	else if (SENSOR_COMPASS == sensor_type)
		st->input_compass_dmp_bias[axes] = dmp_bias;
	else if (sensor_type != -1)
		return -EINVAL;

	return sprintf(buf, "%d\n", dmp_bias);
}

static ssize_t inv_attr_bias_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	int result;

	mutex_lock(&indio_dev->mlock);
	result = _attr_bias_show(dev, attr, buf);

	mutex_unlock(&indio_dev->mlock);

	return result;
}

/*
 * inv_attr_show() -  calling this function will show current
 *                        dmp parameters.
 */
static ssize_t inv_attr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int result;
	s8 *m;

	switch (this_attr->address) {
	case ATTR_GYRO_SCALE:
	{
		const s16 gyro_scale[] = {250, 500, 1000, 2000};

		return sprintf(buf, "%d\n", gyro_scale[st->chip_config.fsr]);
	}
	case ATTR_ACCEL_SCALE:
	{
		const s16 accel_scale[] = {2, 4, 8, 16};
		return sprintf(buf, "%d\n",
					accel_scale[st->chip_config.accel_fs]);
	}
	case ATTR_COMPASS_SCALE:
		st->slave_compass->get_scale(st, &result);

		return sprintf(buf, "%d\n", result);
	case ATTR_COMPASS_SENSITIVITY_X:
		return sprintf(buf, "%d\n", st->chip_info.compass_sens[0]);
	case ATTR_COMPASS_SENSITIVITY_Y:
		return sprintf(buf, "%d\n", st->chip_info.compass_sens[1]);
	case ATTR_COMPASS_SENSITIVITY_Z:
		return sprintf(buf, "%d\n", st->chip_info.compass_sens[2]);
	case ATTR_GYRO_ENABLE:
		return sprintf(buf, "%d\n", st->chip_config.gyro_enable);
	case ATTR_ACCEL_ENABLE:
		return sprintf(buf, "%d\n", st->chip_config.accel_enable);
	case ATTR_DMP_ACCEL_CAL_ENABLE:
		return sprintf(buf, "%d\n", st->accel_cal_enable);
	case ATTR_DMP_GYRO_CAL_ENABLE:
		return sprintf(buf, "%d\n", st->gyro_cal_enable);
	case ATTR_DMP_DEBUG_DETERMINE_ENGINE_ON:
		return sprintf(buf, "%d\n", st->debug_determine_engine_on);
	case ATTR_DMP_PARAMS_ACCEL_CALIBRATION_THRESHOLD:
		return sprintf(buf, "%d\n", st->accel_calib_threshold);
	case ATTR_DMP_PARAMS_ACCEL_CALIBRATION_RATE:
		return sprintf(buf, "%d\n", st->accel_calib_rate);
	case ATTR_FIRMWARE_LOADED:
		return sprintf(buf, "%d\n", st->chip_config.firmware_loaded);
	case ATTR_DMP_ON:
		return sprintf(buf, "%d\n", st->chip_config.dmp_on);
	case ATTR_DMP_BATCHMODE_TIMEOUT:
		return sprintf(buf, "%d\n", st->batch.timeout);
	case ATTR_DMP_EVENT_INT_ON:
		return sprintf(buf, "%d\n", st->chip_config.dmp_event_int_on);
	case ATTR_DMP_PED_INT_ON:
		return sprintf(buf, "%d\n", st->ped.int_on);
	case ATTR_DMP_PED_ON:
		return sprintf(buf, "%d\n", st->ped.on);
	case ATTR_DMP_PED_STEP_THRESH:
		return sprintf(buf, "%d\n", st->ped.step_thresh);
	case ATTR_DMP_PED_INT_THRESH:
		return sprintf(buf, "%d\n", st->ped.int_thresh);
	case ATTR_DMP_SMD_ENABLE:
		return sprintf(buf, "%d\n", st->smd.on);
	case ATTR_DMP_TILT_ENABLE:
		return sprintf(buf, "%d\n", st->chip_config.tilt_enable);
	case ATTR_DMP_PICK_UP_ENABLE:
		return sprintf(buf, "%d\n", st->chip_config.pick_up_enable);
	case ATTR_DMP_LOW_POWER_GYRO_ON:
		return sprintf(buf, "%d\n", st->chip_config.low_power_gyro_on);
	case ATTR_DMP_LP_EN_OFF:
		return sprintf(buf, "%d\n", st->chip_config.lp_en_mode_off);
	case ATTR_COMPASS_ENABLE:
		return sprintf(buf, "%d\n", st->chip_config.compass_enable);
	case ATTR_DMP_STEP_COUNTER_ON:
		return sprintf(buf, "%d\n", st->step_counter_l_on);
	case ATTR_DMP_STEP_COUNTER_WAKE_ON:
		return sprintf(buf, "%d\n", st->step_counter_wake_l_on);
	case ATTR_DMP_STEP_DETECTOR_ON:
		return sprintf(buf, "%d\n", st->step_detector_l_on);
	case ATTR_DMP_STEP_DETECTOR_WAKE_ON:
		return sprintf(buf, "%d\n", st->step_detector_wake_l_on);
	case ATTR_DMP_ACTIVITY_ON:
		return sprintf(buf, "%d\n", st->chip_config.activity_on);
	case ATTR_DMP_IN_ANGLVEL_ACCURACY_ENABLE:
		return sprintf(buf, "%d\n",
				st->sensor_accuracy[SENSOR_GYRO_ACCURACY].on);
	case ATTR_DMP_IN_ACCEL_ACCURACY_ENABLE:
		return sprintf(buf, "%d\n",
				st->sensor_accuracy[SENSOR_ACCEL_ACCURACY].on);
	case ATTR_GYRO_MATRIX:
		m = st->plat_data.orientation;
		return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
	case ATTR_ACCEL_MATRIX:
		m = st->plat_data.orientation;
		return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
	case ATTR_COMPASS_MATRIX:
		if (st->plat_data.sec_slave_type ==
				SECONDARY_SLAVE_TYPE_COMPASS)
			m =
			st->plat_data.secondary_orientation;
		else
			return -ENODEV;
		return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
	case ATTR_SECONDARY_NAME:
	{
		const char *n[] = {"NULL", "AK8975", "AK8972", "AK8963",
					"MLX90399", "AK09911", "AK09912"};

		switch (st->plat_data.sec_slave_id) {
		case COMPASS_ID_AK8975:
			return sprintf(buf, "%s\n", n[1]);
		case COMPASS_ID_AK8972:
			return sprintf(buf, "%s\n", n[2]);
		case COMPASS_ID_AK8963:
			return sprintf(buf, "%s\n", n[3]);
		case COMPASS_ID_MLX90399:
			return sprintf(buf, "%s\n", n[4]);
		case COMPASS_ID_AK09911:
			return sprintf(buf, "%s\n", n[5]);
		case COMPASS_ID_AK09912:
			return sprintf(buf, "%s\n", n[6]);
		default:
			return sprintf(buf, "%s\n", n[0]);
		}
	}
	case ATTR_DMP_DEBUG_MEM_READ:
	{
		int out;

		inv_switch_power_in_lp(st, true);
		result = read_be32_from_mem(st, &out, debug_mem_read_addr);
		if (result)
			return result;
		inv_switch_power_in_lp(st, false);
		return sprintf(buf, "0x%x\n", out);
	}
	case ATTR_GYRO_SF:
		return sprintf(buf, "%d\n", st->gyro_sf);
	case ATTR_ANGLVEL_X_ST_CALIBBIAS:
		return sprintf(buf, "%d\n", st->gyro_st_bias[0]);
	case ATTR_ANGLVEL_Y_ST_CALIBBIAS:
		return sprintf(buf, "%d\n", st->gyro_st_bias[1]);
	case ATTR_ANGLVEL_Z_ST_CALIBBIAS:
		return sprintf(buf, "%d\n", st->gyro_st_bias[2]);
	case ATTR_ACCEL_X_ST_CALIBBIAS:
		return sprintf(buf, "%d\n", st->accel_st_bias[0]);
	case ATTR_ACCEL_Y_ST_CALIBBIAS:
		return sprintf(buf, "%d\n", st->accel_st_bias[1]);
	case ATTR_ACCEL_Z_ST_CALIBBIAS:
		return sprintf(buf, "%d\n", st->accel_st_bias[2]);
	case ATTR_GYRO_X_OFFSET:
		return sprintf(buf, "%d\n", st->input_gyro_bias[0]);
	case ATTR_GYRO_Y_OFFSET:
		return sprintf(buf, "%d\n", st->input_gyro_bias[1]);
	case ATTR_GYRO_Z_OFFSET:
		return sprintf(buf, "%d\n", st->input_gyro_bias[2]);
	case ATTR_ACCEL_X_OFFSET:
		return sprintf(buf, "%d\n", st->input_accel_bias[0]);
	case ATTR_ACCEL_Y_OFFSET:
		return sprintf(buf, "%d\n", st->input_accel_bias[1]);
	case ATTR_ACCEL_Z_OFFSET:
		return sprintf(buf, "%d\n", st->input_accel_bias[2]);
	default:
		return -EPERM;
	}
}

static ssize_t inv_attr64_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int result;
	u64 tmp;
	u32 ped;

	mutex_lock(&indio_dev->mlock);
	if (!st->chip_config.dmp_on) {
		mutex_unlock(&indio_dev->mlock);
		return -EINVAL;
	}
	result = 0;
	switch (this_attr->address) {
	case ATTR_DMP_PEDOMETER_STEPS:
		inv_switch_power_in_lp(st, true);
		result = inv_get_pedometer_steps(st, &ped);
		result |= inv_read_pedometer_counter(st);
		tmp = (u64)st->ped.step + (u64)ped;
		inv_switch_power_in_lp(st, false);

		INV_INFO("stepcounter prev_steps:%u, ped:%u, steps:%llu\n", st->prev_steps, ped,tmp);
		break;
	case ATTR_DMP_PEDOMETER_TIME:
		inv_switch_power_in_lp(st, true);
		result = inv_get_pedometer_time(st, &ped);
		tmp = (u64)st->ped.time + ((u64)ped) * MS_PER_PED_TICKS;
		inv_switch_power_in_lp(st, false);
		break;
	case ATTR_DMP_PEDOMETER_COUNTER:
		tmp = st->ped.last_step_time;
		break;
	default:
		tmp = 0;
		result = -EINVAL;
		break;
	}

	mutex_unlock(&indio_dev->mlock);
	if (result)
		return -EINVAL;
	return sprintf(buf, "%lld\n", tmp);
}

static ssize_t inv_attr64_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int result;
	u8 d[4] = {0, 0, 0, 0};
	u64 data;

	mutex_lock(&indio_dev->mlock);
	if (!st->chip_config.firmware_loaded) {
		mutex_unlock(&indio_dev->mlock);
		return -EINVAL;
	}
	result = inv_switch_power_in_lp(st, true);
	if (result) {
		mutex_unlock(&indio_dev->mlock);
		return result;
	}
	result = kstrtoull(buf, 10, &data);
	if (result)
		goto attr64_store_fail;
	switch (this_attr->address) {
	case ATTR_DMP_PEDOMETER_STEPS:
		result = mem_w(PEDSTD_STEPCTR, ARRAY_SIZE(d), d);
		if (result)
			goto attr64_store_fail;
		st->ped.step = data;
		break;
	case ATTR_DMP_PEDOMETER_TIME:
		result = mem_w(PEDSTD_TIMECTR, ARRAY_SIZE(d), d);
		if (result)
			goto attr64_store_fail;
		st->ped.time = data;
		break;
	default:
		result = -EINVAL;
		break;
	}
attr64_store_fail:
	mutex_unlock(&indio_dev->mlock);
	result = inv_switch_power_in_lp(st, false);
	if (result)
		return result;

	return count;
}

static ssize_t inv_self_test(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	int res;

	mutex_lock(&indio_dev->mlock);
	res = inv_hw_self_test(st);
	set_inv_enable(indio_dev);
	mutex_unlock(&indio_dev->mlock);

	return sprintf(buf, "%d\n", res);
}

/*
 *  inv_temperature_show() - Read temperature data directly from registers.
 */
static ssize_t inv_temperature_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	int result, scale_t;
	short temp;
	u8 data[2];

	mutex_lock(&indio_dev->mlock);
	result = inv_switch_power_in_lp(st, true);
	if (result) {
		mutex_unlock(&indio_dev->mlock);
		return result;
	}

	result = inv_plat_read(st, REG_TEMPERATURE, 2, data);
	mutex_unlock(&indio_dev->mlock);
	if (result) {
		pr_err("Could not read temperature register.\n");
		return result;
	}
	result = inv_switch_power_in_lp(st, false);
	if (result)
		return result;
	temp = (s16)(be16_to_cpup((short *)&data[0]));
	scale_t = TEMPERATURE_OFFSET +
		inv_q30_mult((int)temp << MPU_TEMP_SHIFT, TEMPERATURE_SCALE);

	INV_I2C_INC_TEMPREAD(1);
	return sprintf(buf, "%d %lld\n", scale_t, get_time_ns());
}

/*
 * inv_smd_show() -  calling this function showes smd interrupt.
 *                         This event must use poll.
 */
static ssize_t inv_smd_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "1\n");
}

/*
 * inv_ped_show() -  calling this function showes pedometer interrupt.
 *                         This event must use poll.
 */
static ssize_t inv_ped_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "1\n");
}
static ssize_t inv_activity_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->activity_size);
}
static ssize_t inv_tilt_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "1\n");
}

static ssize_t inv_pick_up_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "1\n");
}

/*
 *  inv_reg_dump_show() - Register dump for testing.
 */
static ssize_t inv_reg_dump_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ii;
	char data;
	int bytes_printed = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	inv_set_bank(st, BANK_SEL_0);
	bytes_printed += sprintf(buf + bytes_printed, "bank 0\n");

	for (ii = 0; ii < 0x7F; ii++) {
		/* don't read fifo r/w register */
		if ((ii == REG_MEM_R_W) || (ii == REG_FIFO_R_W))
			data = 0;
		else
			inv_plat_read(st, ii, 1, &data);
		bytes_printed += sprintf(buf + bytes_printed, "%#2x: %#2x\n",
					 ii, data);
	}
	inv_set_bank(st, BANK_SEL_1);
	bytes_printed += sprintf(buf + bytes_printed, "bank 1\n");
	for (ii = 0; ii < 0x2A; ii++) {
		inv_plat_read(st, ii, 1, &data);
		bytes_printed += sprintf(buf + bytes_printed, "%#2x: %#2x\n",
					 ii, data);
	}
	inv_set_bank(st, BANK_SEL_2);
	bytes_printed += sprintf(buf + bytes_printed, "bank 2\n");
	for (ii = 0; ii < 0x55; ii++) {
		inv_plat_read(st, ii, 1, &data);
		bytes_printed += sprintf(buf + bytes_printed, "%#2x: %#2x\n",
					 ii, data);
	}
	inv_set_bank(st, BANK_SEL_3);
	bytes_printed += sprintf(buf + bytes_printed, "bank 3\n");
	for (ii = 0; ii < 0x18; ii++) {
		inv_plat_read(st, ii, 1, &data);
		bytes_printed += sprintf(buf + bytes_printed, "%#2x: %#2x\n",
					 ii, data);
	}
	inv_set_bank(st, BANK_SEL_0);
	set_inv_enable(indio_dev);
	mutex_unlock(&indio_dev->mlock);

	return bytes_printed;
}

static ssize_t inv_flush_batch_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	int result, data;

	result = kstrtoint(buf, 10, &data);
	if (result)
		return result;

	mutex_lock(&indio_dev->mlock);
	result = inv_flush_batch_data(indio_dev, data);
	mutex_unlock(&indio_dev->mlock);

	return count;
}

static const struct iio_chan_spec inv_mpu_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(INV_MPU_SCAN_TIMESTAMP),
};

static DEVICE_ATTR(poll_smd, S_IRUGO, inv_smd_show, NULL);
static DEVICE_ATTR(poll_pedometer, S_IRUGO, inv_ped_show, NULL);
static DEVICE_ATTR(poll_activity, S_IRUGO, inv_activity_show, NULL);
static DEVICE_ATTR(poll_tilt, S_IRUGO, inv_tilt_show, NULL);
static DEVICE_ATTR(poll_pick_up, S_IRUGO, inv_pick_up_show, NULL);

/* special run time sysfs entry, read only */
static DEVICE_ATTR(debug_reg_dump, S_IRUGO | S_IWUGO, inv_reg_dump_show, NULL);
static DEVICE_ATTR(out_temperature, S_IRUGO | S_IWUGO,
						inv_temperature_show, NULL);
static DEVICE_ATTR(misc_self_test, S_IRUGO | S_IWUGO, inv_self_test, NULL);

#ifdef INTERFACE
static DEVICE_ATTR(acc_self_test, S_IRUGO | S_IWUGO, inv_self_test, NULL);
#endif

#if SUSPEND_DISABLE_IRQ
static int irq_disable = 1;
/*
 * get_irq_state()>0, disable irq;
 * 				  <0, enable  irq;
 */
int get_irq_state(void)
{
	return (irq_disable>0?DISABLE:ENABLE);
}
static ssize_t irq_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "irq:%s\n", (get_irq_state()==DISABLE)?"disable":"enable");
}
static ssize_t irq_state_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int result, data;

	result = kstrtoint(buf, 10, &data);
	if (result)
		return result;
	irq_disable = data;
	INV_INFO("irq state:%s\n", (irq_disable>0)?"disable":"enable");
	return count;
}

static DEVICE_ATTR(irq_state, S_IRUGO | S_IWUGO, irq_state_show, irq_state_store);
#endif

extern int debug;
static ssize_t debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "debug_en:%s--level:%d\n", (debug>=1)?"enable":"disable",debug);
}
static ssize_t debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int result, data;

	result = kstrtoint(buf, 10, &data);
	if (result)
		return result;
	debug = data;
	INV_INFO("debug_en level:%d\n", debug);
	return count;
}
static DEVICE_ATTR(inv_debug_en, S_IRUGO | S_IWUGO, debug_show, debug_store);

extern ssize_t iic_state_show(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(inv_iic_state, S_IRUGO | S_IWUGO, iic_state_show, NULL);

static ssize_t inv_dump_stack_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	dump_stack();
	return sprintf(buf, "inv will dump stack\n");
}
static DEVICE_ATTR(inv_dump_stack, S_IRUGO | S_IWUGO, inv_dump_stack_show, NULL);



static IIO_DEVICE_ATTR(info_anglvel_matrix, S_IRUGO, inv_attr_show, NULL,
	ATTR_GYRO_MATRIX);
static IIO_DEVICE_ATTR(info_accel_matrix, S_IRUGO, inv_attr_show, NULL,
	ATTR_ACCEL_MATRIX);
static IIO_DEVICE_ATTR(info_magn_matrix, S_IRUGO, inv_attr_show, NULL,
	ATTR_COMPASS_MATRIX);
static IIO_DEVICE_ATTR(info_secondary_name, S_IRUGO, inv_attr_show, NULL,
	ATTR_SECONDARY_NAME);
static IIO_DEVICE_ATTR(info_gyro_sf, S_IRUGO, inv_attr_show, NULL,
						ATTR_GYRO_SF);
/* write only sysfs */
static DEVICE_ATTR(misc_flush_batch, S_IWUGO, NULL, inv_flush_batch_store);

/* sensor on/off sysfs control */
static IIO_DEVICE_ATTR(in_accel_enable, S_IRUGO | S_IWUGO,
		inv_sensor_on_show, inv_sensor_on_store, SENSOR_L_ACCEL);
static IIO_DEVICE_ATTR(in_anglvel_enable, S_IRUGO | S_IWUGO,
		inv_sensor_on_show, inv_sensor_on_store, SENSOR_L_GYRO);
static IIO_DEVICE_ATTR(in_magn_enable, S_IRUGO | S_IWUGO,
		inv_sensor_on_show, inv_sensor_on_store, SENSOR_L_MAG);
static IIO_DEVICE_ATTR(in_accel_wake_enable, S_IRUGO | S_IWUGO,
		inv_sensor_on_show, inv_sensor_on_store, SENSOR_L_ACCEL_WAKE);
static IIO_DEVICE_ATTR(in_anglvel_wake_enable, S_IRUGO | S_IWUGO,
		inv_sensor_on_show, inv_sensor_on_store, SENSOR_L_GYRO_WAKE);
static IIO_DEVICE_ATTR(in_magn_wake_enable, S_IRUGO | S_IWUGO,
		inv_sensor_on_show, inv_sensor_on_store, SENSOR_L_MAG_WAKE);
static IIO_DEVICE_ATTR(in_6quat_enable, S_IRUGO | S_IWUGO,
		inv_sensor_on_show, inv_sensor_on_store, SENSOR_L_SIXQ);
static IIO_DEVICE_ATTR(in_p6quat_enable, S_IRUGO | S_IWUGO,
		inv_sensor_on_show, inv_sensor_on_store, SENSOR_L_PEDQ);
static IIO_DEVICE_ATTR(in_pressure_enable, S_IRUGO | S_IWUGO,
		inv_sensor_on_show, inv_sensor_on_store, SENSOR_L_PRESSURE);
static IIO_DEVICE_ATTR(in_pressure_wake_enable, S_IRUGO | S_IWUGO,
	inv_sensor_on_show, inv_sensor_on_store, SENSOR_L_PRESSURE_WAKE);
static IIO_DEVICE_ATTR(in_als_px_enable, S_IRUGO | S_IWUGO,
		inv_sensor_on_show, inv_sensor_on_store, SENSOR_L_ALS);
static IIO_DEVICE_ATTR(in_als_px_wake_enable, S_IRUGO | S_IWUGO,
		inv_sensor_on_show, inv_sensor_on_store, SENSOR_L_ALS_WAKE);

/* sensor rate sysfs control */
static IIO_DEVICE_ATTR(in_accel_rate, S_IRUGO | S_IWUGO,
	inv_sensor_rate_show, inv_sensor_rate_store, SENSOR_L_ACCEL);
static IIO_DEVICE_ATTR(in_anglvel_rate, S_IRUGO | S_IWUGO,
	inv_sensor_rate_show, inv_sensor_rate_store, SENSOR_L_GYRO);
static IIO_DEVICE_ATTR(in_magn_rate, S_IRUGO | S_IWUGO,
	inv_sensor_rate_show, inv_sensor_rate_store, SENSOR_L_MAG);
static IIO_DEVICE_ATTR(in_accel_wake_rate, S_IRUGO | S_IWUGO,
	inv_sensor_rate_show, inv_sensor_rate_store, SENSOR_L_ACCEL_WAKE);
static IIO_DEVICE_ATTR(in_anglvel_wake_rate, S_IRUGO | S_IWUGO,
	inv_sensor_rate_show, inv_sensor_rate_store, SENSOR_L_GYRO_WAKE);
static IIO_DEVICE_ATTR(in_magn_wake_rate, S_IRUGO | S_IWUGO,
	inv_sensor_rate_show, inv_sensor_rate_store, SENSOR_L_MAG_WAKE);
static IIO_DEVICE_ATTR(in_6quat_rate, S_IRUGO | S_IWUGO,
	inv_sensor_rate_show, inv_sensor_rate_store, SENSOR_L_SIXQ);
static IIO_DEVICE_ATTR(in_p6quat_rate, S_IRUGO | S_IWUGO,
	inv_sensor_rate_show, inv_sensor_rate_store, SENSOR_L_PEDQ);
static IIO_DEVICE_ATTR(in_pressure_rate, S_IRUGO | S_IWUGO,
	inv_sensor_rate_show, inv_sensor_rate_store, SENSOR_L_PRESSURE);
static IIO_DEVICE_ATTR(in_pressure_wake_rate, S_IRUGO | S_IWUGO,
	inv_sensor_rate_show, inv_sensor_rate_store, SENSOR_L_PRESSURE_WAKE);
static IIO_DEVICE_ATTR(in_als_px_rate, S_IRUGO | S_IWUGO,
	inv_sensor_rate_show, inv_sensor_rate_store, SENSOR_L_ALS);
static IIO_DEVICE_ATTR(in_als_px_wake_rate, S_IRUGO | S_IWUGO,
	inv_sensor_rate_show, inv_sensor_rate_store, SENSOR_L_ALS_WAKE);

/* debug determine engine related sysfs */
static IIO_DEVICE_ATTR(debug_anglvel_accuracy_enable, S_IRUGO | S_IWUGO,
				inv_attr_show, inv_debug_attr_store,
				ATTR_DMP_IN_ANGLVEL_ACCURACY_ENABLE);
static IIO_DEVICE_ATTR(debug_accel_accuracy_enable, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_debug_attr_store, ATTR_DMP_IN_ACCEL_ACCURACY_ENABLE);
static IIO_DEVICE_ATTR(debug_gyro_cal_enable, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_debug_attr_store, ATTR_DMP_GYRO_CAL_ENABLE);
static IIO_DEVICE_ATTR(debug_accel_cal_enable, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_debug_attr_store, ATTR_DMP_ACCEL_CAL_ENABLE);

static IIO_DEVICE_ATTR(debug_gyro_enable, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_debug_attr_store, ATTR_GYRO_ENABLE);
static IIO_DEVICE_ATTR(debug_accel_enable, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_debug_attr_store, ATTR_ACCEL_ENABLE);
static IIO_DEVICE_ATTR(debug_compass_enable, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_debug_attr_store, ATTR_COMPASS_ENABLE);
static IIO_DEVICE_ATTR(debug_dmp_on, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_debug_attr_store, ATTR_DMP_ON);
static IIO_DEVICE_ATTR(debug_dmp_event_int_on, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_debug_attr_store, ATTR_DMP_EVENT_INT_ON);
static IIO_DEVICE_ATTR(debug_mem_read, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_dmp_bias_store, ATTR_DMP_DEBUG_MEM_READ);
static IIO_DEVICE_ATTR(debug_mem_write, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_dmp_bias_store, ATTR_DMP_DEBUG_MEM_WRITE);

static IIO_DEVICE_ATTR(misc_batchmode_timeout, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_basic_attr_store, ATTR_DMP_BATCHMODE_TIMEOUT);

static IIO_DEVICE_ATTR(info_firmware_loaded, S_IRUGO | S_IWUGO, inv_attr_show,
	inv_firmware_loaded_store, ATTR_FIRMWARE_LOADED);

/* engine scale */
static IIO_DEVICE_ATTR(in_accel_scale, S_IRUGO | S_IWUGO, inv_attr_show,
	inv_misc_attr_store, ATTR_ACCEL_SCALE);
static IIO_DEVICE_ATTR(in_anglvel_scale, S_IRUGO | S_IWUGO, inv_attr_show,
	inv_misc_attr_store, ATTR_GYRO_SCALE);
static IIO_DEVICE_ATTR(in_magn_scale, S_IRUGO | S_IWUGO, inv_attr_show,
	NULL, ATTR_COMPASS_SCALE);

static IIO_DEVICE_ATTR(in_magn_sensitivity_x, S_IRUGO | S_IWUGO, inv_attr_show,
	NULL, ATTR_COMPASS_SENSITIVITY_X);
static IIO_DEVICE_ATTR(in_magn_sensitivity_y, S_IRUGO | S_IWUGO, inv_attr_show,
	NULL, ATTR_COMPASS_SENSITIVITY_Y);
static IIO_DEVICE_ATTR(in_magn_sensitivity_z, S_IRUGO | S_IWUGO, inv_attr_show,
	NULL, ATTR_COMPASS_SENSITIVITY_Z);

static IIO_DEVICE_ATTR(debug_low_power_gyro_on, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_misc_attr_store, ATTR_DMP_LOW_POWER_GYRO_ON);
static IIO_DEVICE_ATTR(debug_lp_en_off, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_debug_store, ATTR_DMP_LP_EN_OFF);
static IIO_DEVICE_ATTR(debug_clock_sel, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_debug_store, ATTR_DMP_CLK_SEL);
static IIO_DEVICE_ATTR(debug_reg_write, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_debug_store, ATTR_DEBUG_REG_WRITE);
static IIO_DEVICE_ATTR(debug_cfg_write, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_debug_store, ATTR_DEBUG_WRITE_CFG);
static IIO_DEVICE_ATTR(debug_reg_write_addr, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_debug_store, ATTR_DEBUG_REG_ADDR);

static IIO_DEVICE_ATTR(in_accel_x_calibbias, S_IRUGO | S_IWUGO,
			inv_attr_bias_show, NULL, ATTR_ACCEL_X_CALIBBIAS);
static IIO_DEVICE_ATTR(in_accel_y_calibbias, S_IRUGO | S_IWUGO,
			inv_attr_bias_show, NULL, ATTR_ACCEL_Y_CALIBBIAS);
static IIO_DEVICE_ATTR(in_accel_z_calibbias, S_IRUGO | S_IWUGO,
			inv_attr_bias_show, NULL, ATTR_ACCEL_Z_CALIBBIAS);

static IIO_DEVICE_ATTR(in_anglvel_x_calibbias, S_IRUGO | S_IWUGO,
			inv_attr_bias_show, NULL, ATTR_ANGLVEL_X_CALIBBIAS);
static IIO_DEVICE_ATTR(in_anglvel_y_calibbias, S_IRUGO | S_IWUGO,
			inv_attr_bias_show, NULL, ATTR_ANGLVEL_Y_CALIBBIAS);
static IIO_DEVICE_ATTR(in_anglvel_z_calibbias, S_IRUGO | S_IWUGO,
			inv_attr_bias_show, NULL, ATTR_ANGLVEL_Z_CALIBBIAS);

static IIO_DEVICE_ATTR(in_accel_x_dmp_bias, S_IRUGO | S_IWUGO,
	inv_attr_bias_show, inv_dmp_bias_store, ATTR_DMP_ACCEL_X_DMP_BIAS);
static IIO_DEVICE_ATTR(in_accel_y_dmp_bias, S_IRUGO | S_IWUGO,
	inv_attr_bias_show, inv_dmp_bias_store, ATTR_DMP_ACCEL_Y_DMP_BIAS);
static IIO_DEVICE_ATTR(in_accel_z_dmp_bias, S_IRUGO | S_IWUGO,
	inv_attr_bias_show, inv_dmp_bias_store, ATTR_DMP_ACCEL_Z_DMP_BIAS);

static IIO_DEVICE_ATTR(in_anglvel_x_dmp_bias, S_IRUGO | S_IWUGO,
	inv_attr_bias_show, inv_dmp_bias_store, ATTR_DMP_GYRO_X_DMP_BIAS);
static IIO_DEVICE_ATTR(in_anglvel_y_dmp_bias, S_IRUGO | S_IWUGO,
	inv_attr_bias_show, inv_dmp_bias_store, ATTR_DMP_GYRO_Y_DMP_BIAS);
static IIO_DEVICE_ATTR(in_anglvel_z_dmp_bias, S_IRUGO | S_IWUGO,
	inv_attr_bias_show, inv_dmp_bias_store, ATTR_DMP_GYRO_Z_DMP_BIAS);

static IIO_DEVICE_ATTR(in_accel_x_st_calibbias, S_IRUGO | S_IWUGO,
			inv_attr_show, NULL, ATTR_ACCEL_X_ST_CALIBBIAS);
static IIO_DEVICE_ATTR(in_accel_y_st_calibbias, S_IRUGO | S_IWUGO,
			inv_attr_show, NULL, ATTR_ACCEL_Y_ST_CALIBBIAS);
static IIO_DEVICE_ATTR(in_accel_z_st_calibbias, S_IRUGO | S_IWUGO,
			inv_attr_show, NULL, ATTR_ACCEL_Z_ST_CALIBBIAS);

#ifdef INTERFACE
static IIO_DEVICE_ATTR(acc_x_calibbias, S_IRUGO | S_IWUGO,
			inv_attr_show, NULL, ATTR_ACCEL_X_ST_CALIBBIAS);
static IIO_DEVICE_ATTR(acc_y_calibbias, S_IRUGO | S_IWUGO,
			inv_attr_show, NULL, ATTR_ACCEL_Y_ST_CALIBBIAS);
static IIO_DEVICE_ATTR(acc_z_calibbias, S_IRUGO | S_IWUGO,
			inv_attr_show, NULL, ATTR_ACCEL_Z_ST_CALIBBIAS);
#endif

static IIO_DEVICE_ATTR(in_anglvel_x_st_calibbias, S_IRUGO | S_IWUGO,
			inv_attr_show, NULL, ATTR_ANGLVEL_X_ST_CALIBBIAS);
static IIO_DEVICE_ATTR(in_anglvel_y_st_calibbias, S_IRUGO | S_IWUGO,
			inv_attr_show, NULL, ATTR_ANGLVEL_Y_ST_CALIBBIAS);
static IIO_DEVICE_ATTR(in_anglvel_z_st_calibbias, S_IRUGO | S_IWUGO,
			inv_attr_show, NULL, ATTR_ANGLVEL_Z_ST_CALIBBIAS);

static IIO_DEVICE_ATTR(in_accel_x_offset, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_bias_store, ATTR_ACCEL_X_OFFSET);
static IIO_DEVICE_ATTR(in_accel_y_offset, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_bias_store, ATTR_ACCEL_Y_OFFSET);
static IIO_DEVICE_ATTR(in_accel_z_offset, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_bias_store, ATTR_ACCEL_Z_OFFSET);

#ifdef INTERFACE
static IIO_DEVICE_ATTR(acc_x_offset, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_bias_store, ATTR_ACCEL_X_OFFSET);
static IIO_DEVICE_ATTR(acc_y_offset, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_bias_store, ATTR_ACCEL_Y_OFFSET);
static IIO_DEVICE_ATTR(acc_z_offset, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_bias_store, ATTR_ACCEL_Z_OFFSET);
#endif



static IIO_DEVICE_ATTR(in_anglvel_x_offset, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_bias_store, ATTR_GYRO_X_OFFSET);
static IIO_DEVICE_ATTR(in_anglvel_y_offset, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_bias_store, ATTR_GYRO_Y_OFFSET);
static IIO_DEVICE_ATTR(in_anglvel_z_offset, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_bias_store, ATTR_GYRO_Z_OFFSET);

static IIO_DEVICE_ATTR(in_sc_auth, S_IRUGO | S_IWUGO,
	inv_attr_bias_show, inv_dmp_bias_store, ATTR_DMP_SC_AUTH);

static IIO_DEVICE_ATTR(debug_determine_engine_on, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_misc_attr_store, ATTR_DMP_DEBUG_DETERMINE_ENGINE_ON);
static IIO_DEVICE_ATTR(misc_gyro_recalibration, S_IRUGO | S_IWUGO, NULL,
	inv_dmp_bias_store, ATTR_DMP_MISC_GYRO_RECALIBRATION);
static IIO_DEVICE_ATTR(misc_accel_recalibration, S_IRUGO | S_IWUGO, NULL,
	inv_dmp_bias_store, ATTR_DMP_MISC_ACCEL_RECALIBRATION);
static IIO_DEVICE_ATTR(params_accel_calibration_threshold, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_dmp_bias_store,
			ATTR_DMP_PARAMS_ACCEL_CALIBRATION_THRESHOLD);
static IIO_DEVICE_ATTR(params_accel_calibration_rate, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_dmp_bias_store,
				ATTR_DMP_PARAMS_ACCEL_CALIBRATION_RATE);

static IIO_DEVICE_ATTR(in_step_detector_enable, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_basic_attr_store, ATTR_DMP_STEP_DETECTOR_ON);
static IIO_DEVICE_ATTR(in_step_detector_wake_enable, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_basic_attr_store, ATTR_DMP_STEP_DETECTOR_WAKE_ON);
static IIO_DEVICE_ATTR(in_step_counter_enable, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_basic_attr_store, ATTR_DMP_STEP_COUNTER_ON);
static IIO_DEVICE_ATTR(in_step_counter_wake_enable, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_basic_attr_store, ATTR_DMP_STEP_COUNTER_WAKE_ON);
static IIO_DEVICE_ATTR(in_activity_enable, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_basic_attr_store, ATTR_DMP_ACTIVITY_ON);

static IIO_DEVICE_ATTR(event_smd_enable, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_basic_attr_store, ATTR_DMP_SMD_ENABLE);

static IIO_DEVICE_ATTR(event_tilt_enable, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_basic_attr_store, ATTR_DMP_TILT_ENABLE);

static IIO_DEVICE_ATTR(event_pick_up_enable, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_basic_attr_store, ATTR_DMP_PICK_UP_ENABLE);

static IIO_DEVICE_ATTR(params_pedometer_int_on, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_misc_attr_store, ATTR_DMP_PED_INT_ON);
static IIO_DEVICE_ATTR(event_pedometer_enable, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_basic_attr_store, ATTR_DMP_PED_ON);
static IIO_DEVICE_ATTR(params_pedometer_step_thresh, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_misc_attr_store, ATTR_DMP_PED_STEP_THRESH);
static IIO_DEVICE_ATTR(params_pedometer_int_thresh, S_IRUGO | S_IWUGO,
	inv_attr_show, inv_misc_attr_store, ATTR_DMP_PED_INT_THRESH);

static IIO_DEVICE_ATTR(out_pedometer_steps, S_IRUGO | S_IWUGO, inv_attr64_show,
	inv_attr64_store, ATTR_DMP_PEDOMETER_STEPS);
static IIO_DEVICE_ATTR(out_pedometer_time, S_IRUGO | S_IWUGO, inv_attr64_show,
	inv_attr64_store, ATTR_DMP_PEDOMETER_TIME);
static IIO_DEVICE_ATTR(out_pedometer_counter, S_IRUGO | S_IWUGO,
			inv_attr64_show, NULL, ATTR_DMP_PEDOMETER_COUNTER);

static const struct attribute *inv_raw_attributes[] = {
	&dev_attr_debug_reg_dump.attr,
	&dev_attr_out_temperature.attr,
	&dev_attr_misc_flush_batch.attr,
	&dev_attr_misc_self_test.attr,
#ifdef INTERFACE
	&dev_attr_acc_self_test.attr,
#endif
#if SUSPEND_DISABLE_IRQ
	&dev_attr_irq_state.attr,
#endif
	&dev_attr_inv_debug_en.attr,
	&dev_attr_inv_iic_state.attr,
	&dev_attr_inv_dump_stack.attr,



	&iio_dev_attr_in_sc_auth.dev_attr.attr,
	&iio_dev_attr_in_accel_enable.dev_attr.attr,
	&iio_dev_attr_in_accel_wake_enable.dev_attr.attr,
	&iio_dev_attr_info_accel_matrix.dev_attr.attr,
	&iio_dev_attr_in_accel_scale.dev_attr.attr,
	&iio_dev_attr_info_firmware_loaded.dev_attr.attr,
	&iio_dev_attr_misc_batchmode_timeout.dev_attr.attr,
	&iio_dev_attr_in_accel_rate.dev_attr.attr,
	&iio_dev_attr_in_accel_wake_rate.dev_attr.attr,
	&iio_dev_attr_info_secondary_name.dev_attr.attr,
	&iio_dev_attr_debug_mem_read.dev_attr.attr,
	&iio_dev_attr_debug_mem_write.dev_attr.attr,
};

static const struct attribute *inv_debug_attributes[] = {
	&iio_dev_attr_debug_accel_enable.dev_attr.attr,
	&iio_dev_attr_debug_dmp_event_int_on.dev_attr.attr,
	&iio_dev_attr_debug_low_power_gyro_on.dev_attr.attr,
	&iio_dev_attr_debug_lp_en_off.dev_attr.attr,
	&iio_dev_attr_debug_clock_sel.dev_attr.attr,
	&iio_dev_attr_debug_reg_write.dev_attr.attr,
	&iio_dev_attr_debug_reg_write_addr.dev_attr.attr,
	&iio_dev_attr_debug_cfg_write.dev_attr.attr,
	&iio_dev_attr_debug_dmp_on.dev_attr.attr,
	&iio_dev_attr_debug_accel_cal_enable.dev_attr.attr,
	&iio_dev_attr_debug_accel_accuracy_enable.dev_attr.attr,
	&iio_dev_attr_debug_determine_engine_on.dev_attr.attr,
	&iio_dev_attr_debug_gyro_enable.dev_attr.attr,
	&iio_dev_attr_debug_gyro_cal_enable.dev_attr.attr,
	&iio_dev_attr_debug_anglvel_accuracy_enable.dev_attr.attr,
	&iio_dev_attr_debug_compass_enable.dev_attr.attr,
	&iio_dev_attr_params_pedometer_step_thresh.dev_attr.attr,
	&iio_dev_attr_params_pedometer_int_thresh.dev_attr.attr,
	&iio_dev_attr_misc_gyro_recalibration.dev_attr.attr,
	&iio_dev_attr_misc_accel_recalibration.dev_attr.attr,
	&iio_dev_attr_params_accel_calibration_threshold.dev_attr.attr,
	&iio_dev_attr_params_accel_calibration_rate.dev_attr.attr,
};

static const struct attribute *inv_gyro_attributes[] = {
	&iio_dev_attr_info_anglvel_matrix.dev_attr.attr,
	&iio_dev_attr_in_anglvel_enable.dev_attr.attr,
	&iio_dev_attr_in_anglvel_wake_enable.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale.dev_attr.attr,
	&iio_dev_attr_in_anglvel_rate.dev_attr.attr,
	&iio_dev_attr_in_anglvel_wake_rate.dev_attr.attr,
	&iio_dev_attr_in_6quat_enable.dev_attr.attr,
	&iio_dev_attr_in_6quat_rate.dev_attr.attr,
	&iio_dev_attr_in_p6quat_enable.dev_attr.attr,
	&iio_dev_attr_in_p6quat_rate.dev_attr.attr,
	&iio_dev_attr_info_gyro_sf.dev_attr.attr,
};

static const struct attribute *inv_bias_attributes[] = {
	&iio_dev_attr_in_accel_x_dmp_bias.dev_attr.attr,
	&iio_dev_attr_in_accel_y_dmp_bias.dev_attr.attr,
	&iio_dev_attr_in_accel_z_dmp_bias.dev_attr.attr,
	&iio_dev_attr_in_accel_x_calibbias.dev_attr.attr,
	&iio_dev_attr_in_accel_y_calibbias.dev_attr.attr,
	&iio_dev_attr_in_accel_z_calibbias.dev_attr.attr,
};

static const struct attribute *inv_gyro_bias_attributes[] = {
	&iio_dev_attr_in_anglvel_x_dmp_bias.dev_attr.attr,
	&iio_dev_attr_in_anglvel_y_dmp_bias.dev_attr.attr,
	&iio_dev_attr_in_anglvel_z_dmp_bias.dev_attr.attr,
	&iio_dev_attr_in_anglvel_x_calibbias.dev_attr.attr,
	&iio_dev_attr_in_anglvel_y_calibbias.dev_attr.attr,
	&iio_dev_attr_in_anglvel_z_calibbias.dev_attr.attr,
};
static const struct attribute *inv_bias_st_attributes[] = {
	&iio_dev_attr_in_anglvel_x_st_calibbias.dev_attr.attr,
	&iio_dev_attr_in_anglvel_y_st_calibbias.dev_attr.attr,
	&iio_dev_attr_in_anglvel_z_st_calibbias.dev_attr.attr,
	&iio_dev_attr_in_accel_x_st_calibbias.dev_attr.attr,
	&iio_dev_attr_in_accel_y_st_calibbias.dev_attr.attr,
	&iio_dev_attr_in_accel_z_st_calibbias.dev_attr.attr,
	&iio_dev_attr_in_anglvel_x_offset.dev_attr.attr,
	&iio_dev_attr_in_anglvel_y_offset.dev_attr.attr,
	&iio_dev_attr_in_anglvel_z_offset.dev_attr.attr,
	&iio_dev_attr_in_accel_x_offset.dev_attr.attr,
	&iio_dev_attr_in_accel_y_offset.dev_attr.attr,
	&iio_dev_attr_in_accel_z_offset.dev_attr.attr,

#ifdef INTERFACE
	&iio_dev_attr_acc_x_calibbias.dev_attr.attr,
	&iio_dev_attr_acc_y_calibbias.dev_attr.attr,
	&iio_dev_attr_acc_z_calibbias.dev_attr.attr,
	&iio_dev_attr_acc_x_offset.dev_attr.attr,
	&iio_dev_attr_acc_y_offset.dev_attr.attr,
	&iio_dev_attr_acc_z_offset.dev_attr.attr,
#endif
};
static const struct attribute *inv_compass_attributes[] = {
	&iio_dev_attr_in_magn_sensitivity_x.dev_attr.attr,
	&iio_dev_attr_in_magn_sensitivity_y.dev_attr.attr,
	&iio_dev_attr_in_magn_sensitivity_z.dev_attr.attr,
	&iio_dev_attr_in_magn_scale.dev_attr.attr,
	&iio_dev_attr_info_magn_matrix.dev_attr.attr,
	&iio_dev_attr_in_magn_enable.dev_attr.attr,
	&iio_dev_attr_in_magn_rate.dev_attr.attr,
	&iio_dev_attr_in_magn_wake_enable.dev_attr.attr,
	&iio_dev_attr_in_magn_wake_rate.dev_attr.attr,
};

static const struct attribute *inv_pedometer_attributes[] = {
	&dev_attr_poll_pedometer.attr,
	&dev_attr_poll_activity.attr,
	&dev_attr_poll_tilt.attr,
	&dev_attr_poll_pick_up.attr,
	&iio_dev_attr_params_pedometer_int_on.dev_attr.attr,
	&iio_dev_attr_event_pedometer_enable.dev_attr.attr,
	&iio_dev_attr_event_tilt_enable.dev_attr.attr,
	&iio_dev_attr_event_pick_up_enable.dev_attr.attr,
	&iio_dev_attr_in_step_counter_enable.dev_attr.attr,
	&iio_dev_attr_in_step_counter_wake_enable.dev_attr.attr,
	&iio_dev_attr_in_step_detector_enable.dev_attr.attr,
	&iio_dev_attr_in_step_detector_wake_enable.dev_attr.attr,
	&iio_dev_attr_in_activity_enable.dev_attr.attr,
	&iio_dev_attr_out_pedometer_steps.dev_attr.attr,
	&iio_dev_attr_out_pedometer_time.dev_attr.attr,
	&iio_dev_attr_out_pedometer_counter.dev_attr.attr,
};
static const struct attribute *inv_smd_attributes[] = {
	&dev_attr_poll_smd.attr,
	&iio_dev_attr_event_smd_enable.dev_attr.attr,
};

static const struct attribute *inv_pressure_attributes[] = {
	&iio_dev_attr_in_pressure_enable.dev_attr.attr,
	&iio_dev_attr_in_pressure_rate.dev_attr.attr,
	&iio_dev_attr_in_pressure_wake_enable.dev_attr.attr,
	&iio_dev_attr_in_pressure_wake_rate.dev_attr.attr,
};

static const struct attribute *inv_als_attributes[] = {
	&iio_dev_attr_in_als_px_enable.dev_attr.attr,
	&iio_dev_attr_in_als_px_rate.dev_attr.attr,
	&iio_dev_attr_in_als_px_wake_enable.dev_attr.attr,
	&iio_dev_attr_in_als_px_wake_rate.dev_attr.attr,
};

static struct attribute *inv_attributes[
	ARRAY_SIZE(inv_raw_attributes) +
	ARRAY_SIZE(inv_pedometer_attributes) +
	ARRAY_SIZE(inv_smd_attributes) +
	ARRAY_SIZE(inv_compass_attributes) +
	ARRAY_SIZE(inv_pressure_attributes) +
	ARRAY_SIZE(inv_als_attributes) +
	ARRAY_SIZE(inv_bias_attributes) +
	ARRAY_SIZE(inv_gyro_attributes) +
	ARRAY_SIZE(inv_gyro_bias_attributes) +
	ARRAY_SIZE(inv_debug_attributes) +
	1
];

static const struct attribute_group inv_attribute_group = {
	.name = "mpu",
	.attrs = inv_attributes
};

static const struct iio_info mpu_info = {
	.driver_module = THIS_MODULE,
	.attrs = &inv_attribute_group,
};

/*
 *  inv_check_chip_type() - check and setup chip type.
 */
int inv_check_chip_type(struct iio_dev *indio_dev, const char *name)
{
	int result;
	int t_ind;
	struct inv_chip_config_s *conf;
	struct mpu_platform_data *plat;
	struct inv_mpu_state *st;

	st = iio_priv(indio_dev);
	conf = &st->chip_config;
	plat = &st->plat_data;

	if (!strcmp(name, "icm20645"))
		st->chip_type = ICM20645;
	else if (!strcmp(name, "icm10340"))
		st->chip_type = ICM10340;
	else
		return -EPERM;
	if (ICM20645 == st->chip_type) {
		st->dmp_image_size = DMP_IMAGE_SIZE_20645;
		st->dmp_start_address = DMP_START_ADDR_20645;
		st->aut_key_in = SC_AUT_INPUT_20645;
		st->aut_key_out = SC_AUT_OUTPUT_20645;
		st->chip_config.has_gyro = 1;
	} else {
		st->dmp_image_size = DMP_IMAGE_SIZE_10340;
		st->dmp_start_address = DMP_START_ADDR_10340;
		st->aut_key_in = SC_AUT_INPUT_10340;
		st->aut_key_out = SC_AUT_OUTPUT_10340;
		st->chip_config.has_gyro = 0;
	}
	st->hw  = &hw_info[st->chip_type];
	result = inv_mpu_initialize(st);
	if (result)
		return result;

	t_ind = 0;
	memcpy(&inv_attributes[t_ind], inv_raw_attributes,
					sizeof(inv_raw_attributes));
	t_ind += ARRAY_SIZE(inv_raw_attributes);

	memcpy(&inv_attributes[t_ind], inv_pedometer_attributes,
					sizeof(inv_pedometer_attributes));
	t_ind += ARRAY_SIZE(inv_pedometer_attributes);

	memcpy(&inv_attributes[t_ind], inv_smd_attributes,
					sizeof(inv_smd_attributes));
	t_ind += ARRAY_SIZE(inv_smd_attributes);

	if (st->chip_config.has_compass) {
		memcpy(&inv_attributes[t_ind], inv_compass_attributes,
		       sizeof(inv_compass_attributes));
		t_ind += ARRAY_SIZE(inv_compass_attributes);
	}
	if (st->chip_config.has_pressure) {
		memcpy(&inv_attributes[t_ind], inv_pressure_attributes,
		       sizeof(inv_pressure_attributes));
		t_ind += ARRAY_SIZE(inv_pressure_attributes);
	}
	if (st->chip_config.has_als) {
		memcpy(&inv_attributes[t_ind], inv_als_attributes,
		       sizeof(inv_als_attributes));
		t_ind += ARRAY_SIZE(inv_als_attributes);
	}
	if (ICM10340 == st->chip_type) {
		memcpy(&inv_attributes[t_ind], inv_bias_attributes,
		       sizeof(inv_bias_attributes));
		t_ind += ARRAY_SIZE(inv_bias_attributes);
	} else {
		memcpy(&inv_attributes[t_ind], inv_gyro_attributes,
		       sizeof(inv_gyro_attributes));
		t_ind += ARRAY_SIZE(inv_gyro_attributes);
	}

	memcpy(&inv_attributes[t_ind], inv_bias_st_attributes,
		   sizeof(inv_bias_st_attributes));
	t_ind += ARRAY_SIZE(inv_bias_st_attributes);

	inv_attributes[t_ind] = NULL;

	indio_dev->channels = inv_mpu_channels;
	indio_dev->num_channels = ARRAY_SIZE(inv_mpu_channels);

	indio_dev->info = &mpu_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;
	INIT_KFIFO(st->kf);

	return result;
}

/*
 * inv_dmp_firmware_write() -  calling this function will load the firmware.
 */
static ssize_t inv_dmp_firmware_write(struct file *fp, struct kobject *kobj,
	struct bin_attribute *attr, char *buf, loff_t pos, size_t size)
{
	int result, offset;
	struct iio_dev *indio_dev;
	struct inv_mpu_state *st;
	u32 crc_value;
/* svn 14169 from Tan. file name :dmp3Default_L_20645_SA.c */
#define FIRMWARE_CRC_20645           0xaee26119
/* svn 14277 from Tan. file name: dmp3Default_L_SA.c */
#define FIRMWARE_CRC_10340           0xdb402d1a

	indio_dev = dev_get_drvdata(container_of(kobj, struct device, kobj));
	st = iio_priv(indio_dev);

	if (!st->firmware) {
		st->firmware = kmalloc(st->dmp_image_size, GFP_KERNEL);
		if (!st->firmware) {
			pr_err("no memory while loading firmware\n");
			return -ENOMEM;
		}
	}
	offset = pos;
	memcpy(st->firmware + pos, buf, size);
	if ((!size) && (st->dmp_image_size != pos)) {
		pr_err("wrong size for DMP firmware 0x%08x vs 0x%08x\n",
						offset, st->dmp_image_size);
		kfree(st->firmware);
		st->firmware = 0;
		return -EINVAL;
	}
	if (st->dmp_image_size == (pos + size)) {
		result = crc32(0, st->firmware, st->dmp_image_size);
		if (st->chip_type == ICM20645)
			crc_value = FIRMWARE_CRC_20645;
		else
			crc_value = FIRMWARE_CRC_10340;

		if (crc_value != result) {
			pr_err("firmware CRC error - 0x%08x vs 0x%08x\n",
							result, crc_value);
			return -EINVAL;
		}
		mutex_lock(&indio_dev->mlock);
		result = inv_firmware_load(st);
		kfree(st->firmware);
		st->firmware = 0;
		mutex_unlock(&indio_dev->mlock);
		if (result) {
			pr_err("firmware load failed\n");
			return result;
		}
	}

	return size;
}

static ssize_t inv_dmp_firmware_read(struct file *filp,
				struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	int result, offset;
	struct iio_dev *indio_dev;
	struct inv_mpu_state *st;

	indio_dev = dev_get_drvdata(container_of(kobj, struct device, kobj));
	st = iio_priv(indio_dev);

	if (!st->chip_config.firmware_loaded)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);
	offset = off;
	result = inv_dmp_read(st, offset, count, buf);
	set_inv_enable(indio_dev);
	mutex_unlock(&indio_dev->mlock);
	if (result)
		return result;

	return count;
}

 int inv_soft_iron_matrix_write(struct file *fp, struct kobject *kobj,
	struct bin_attribute *attr, char *buf, loff_t pos, size_t size)
{
	struct iio_dev *indio_dev;
	struct inv_mpu_state *st;
	int m[NINE_ELEM], *n, *r;
	int i, j, k;

	indio_dev = dev_get_drvdata(container_of(kobj, struct device, kobj));
	st = iio_priv(indio_dev);

	if (!st->chip_config.firmware_loaded)
		return -EINVAL;
	if (size != SOFT_IRON_MATRIX_SIZE) {
		pr_err("wrong size for soft iron matrix 0x%08x vs 0x%08x\n",
						(unsigned int)size, (unsigned int)SOFT_IRON_MATRIX_SIZE);
		return -EINVAL;
	}
	n = st->current_compass_matrix;
	r = st->final_compass_matrix;
	for (i = 0; i < NINE_ELEM; i++)
		memcpy((u8 *)&m[i], &buf[i * sizeof(int)], sizeof(int));

	for (i = 0; i < THREE_AXES; i++) {
		for (j = 0; j < THREE_AXES; j++) {
			r[i * THREE_AXES + j] = 0;
			for (k = 0; k < THREE_AXES; k++)
				r[i * THREE_AXES + j] +=
					inv_q30_mult(m[i * THREE_AXES + k],
							n[j + k * THREE_AXES]);
		}
	}

	return size;
}

int inv_accel_covariance_write(struct file *fp, struct kobject *kobj,
		struct bin_attribute *attr, char *buf, loff_t pos, size_t size)
{
	int i;
	struct iio_dev *indio_dev;
	struct inv_mpu_state *st;

	indio_dev = dev_get_drvdata(container_of(kobj, struct device, kobj));
	st = iio_priv(indio_dev);

	if (size != ACCEL_COVARIANCE_SIZE)
		return -EINVAL;

	for (i = 0; i < COVARIANCE_SIZE; i++)
		memcpy((u8 *)&st->accel_covariance[i],
					&buf[i * sizeof(int)], sizeof(int));

	return size;
}

static int inv_dmp_covar_read(struct inv_mpu_state *st,
						int off, int size, u8 *buf)
{
	int data, result, i;

	result = inv_switch_power_in_lp(st, true);
	if (result)
		return result;
	inv_stop_dmp(st);
	for (i = 0; i < COVARIANCE_SIZE * sizeof(int); i += sizeof(int)) {
		result = read_be32_from_mem(st, (u32 *)&data, off + i);
		if (result)
			return result;
		memcpy(buf + i, (u8 *)&data, sizeof(int));
	}

	return 0;
}

int inv_accel_covariance_read(struct file *filp,
				struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	int result;
	struct iio_dev *indio_dev;
	struct inv_mpu_state *st;

	indio_dev = dev_get_drvdata(container_of(kobj, struct device, kobj));
	st = iio_priv(indio_dev);

	if (!st->chip_config.firmware_loaded)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);
	result = inv_dmp_covar_read(st, ACCEL_COVARIANCE, count, buf);
	set_inv_enable(indio_dev);
	mutex_unlock(&indio_dev->mlock);
	if (result)
		return result;

	return count;
}

int inv_activity_read(struct file *filp,
				struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	int copied;
	struct iio_dev *indio_dev;
	struct inv_mpu_state *st;
	u8 ddd[128];

	indio_dev = dev_get_drvdata(container_of(kobj, struct device, kobj));
	st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	copied = kfifo_out(&st->kf, ddd, count);
	memcpy(buf, ddd, copied);
	mutex_unlock(&indio_dev->mlock);

	return copied;
}

/*
 *  inv_create_dmp_sysfs() - create binary sysfs dmp entry.
 */
static struct bin_attribute dmp_firmware = {
	.attr = {
		.name = "misc_bin_dmp_firmware",
		.mode = S_IRUGO | S_IWUGO
	},
	.read = inv_dmp_firmware_read,
	.write = inv_dmp_firmware_write,
};
#if 0
static const struct bin_attribute soft_iron_matrix = {
	.attr = {
		.name = "misc_bin_soft_iron_matrix",
		.mode = S_IRUGO | S_IWUGO
	},
	.size = SOFT_IRON_MATRIX_SIZE,
	.write = inv_soft_iron_matrix_write,
};

static const struct bin_attribute accel_covariance = {
	.attr = {
		.name = "misc_bin_accel_covariance",
		.mode = S_IRUGO | S_IWUGO
	},
	.size = ACCEL_COVARIANCE_SIZE,
	.write = inv_accel_covariance_write,
	.read =  inv_accel_covariance_read,
};

static const struct bin_attribute activity_bin = {
	.attr = {
		.name = "misc_bin_activity_out",
		.mode = S_IRUGO
	},
	.size = HARDWARE_FIFO_SIZE,
	.read =  inv_activity_read,
};
#endif
#ifdef INTERFACE
extern int meizu_sysfslink_register_n(struct device *dev, char *name);
#endif

int inv_create_dmp_sysfs(struct iio_dev *ind)
{
	int result;
	struct inv_mpu_state *st = iio_priv(ind);

	dmp_firmware.size = st->dmp_image_size;

#ifdef INTERFACE
	meizu_sysfslink_register_n(&ind->dev, "acc");
#endif
	result = sysfs_create_bin_file(&ind->dev.kobj, &dmp_firmware);
	if (result)
		return result;
#if 0
	result = sysfs_create_bin_file(&ind->dev.kobj, &soft_iron_matrix);
	if (result)
		return result;

	result = sysfs_create_bin_file(&ind->dev.kobj, &accel_covariance);
	if (result)
		return result;
	result = sysfs_create_bin_file(&ind->dev.kobj, &activity_bin);
#endif
	return result;
}
