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

#include "inv_mpu_iio.h"

static int inv_set_flip_pickup_gesture_params_V3(struct inv_mpu_state *st)
{
	int var_error_alpha;
	int still_threshold;
	int middle_still_threshold;
	int not_still_threshold;
	int vibration_rejection_threshold;
	int maximum_pickup_time_threshold;
	int pickup_timeout_threshold;
	int still_consistency_count_threshold;
	int motion_consistency_count_threshold;
	int vibration_count_threshold;
	int steady_tilt_threshold;
	int steady_tilt_upper_threshold;
	int accel_z_flat_threshold_minus;
	int accel_z_flat_threshold_plus;
	int device_in_pocket_threshold;
	int result;

        var_error_alpha                     = 107374182L;
        still_threshold                     = 4L;
        middle_still_threshold              = 10L;
        not_still_threshold                 = 40L;
        vibration_rejection_threshold       = 65100L;
        maximum_pickup_time_threshold       = 30L;
        pickup_timeout_threshold            = 150L;
        still_consistency_count_threshold   = 80;
        motion_consistency_count_threshold  = 10;
        vibration_count_threshold           = 3;
        steady_tilt_threshold               = 6710886L;
        steady_tilt_upper_threshold         = 140928614L;
        accel_z_flat_threshold_minus        = 60397978L;
        accel_z_flat_threshold_plus         = 6710886L;
        device_in_pocket_threshold          = 100L;

	result  = write_be32_to_mem(st, var_error_alpha, FP_VAR_ALPHA);
	result += write_be32_to_mem(st, still_threshold, FP_STILL_TH);
	result += write_be32_to_mem(st, middle_still_threshold,
							FP_MID_STILL_TH);
	result += write_be32_to_mem(st, not_still_threshold,
							FP_NOT_STILL_TH);
	result += write_be32_to_mem(st, vibration_rejection_threshold,
							FP_VIB_REJ_TH);
	result += write_be32_to_mem(st, maximum_pickup_time_threshold,
							FP_MAX_PICKUP_T_TH);
	result += write_be32_to_mem(st, pickup_timeout_threshold,
							FP_PICKUP_TIMEOUT_TH);
	result += write_be32_to_mem(st, still_consistency_count_threshold,
							FP_STILL_CONST_TH);
	result += write_be32_to_mem(st, motion_consistency_count_threshold,
							FP_MOTION_CONST_TH);
	result += write_be32_to_mem(st, vibration_count_threshold,
							FP_VIB_COUNT_TH);
	result += write_be32_to_mem(st, steady_tilt_threshold,
							FP_STEADY_TILT_TH);
	result += write_be32_to_mem(st, steady_tilt_upper_threshold,
							FP_STEADY_TILT_UP_TH);
	result += write_be32_to_mem(st, accel_z_flat_threshold_minus,
							FP_Z_FLAT_TH_MINUS);
	result += write_be32_to_mem(st, accel_z_flat_threshold_plus,
							FP_Z_FLAT_TH_PLUS);
	result += write_be32_to_mem(st, device_in_pocket_threshold,
							FP_DEV_IN_POCKET_TH);

	return result;
}

static int inv_load_firmware(struct inv_mpu_state *st)
{
	int bank, write_size;
	int result, size;
	u16 memaddr;
	u8 *data;

	data = st->firmware;
	size = st->dmp_image_size - DMP_OFFSET;
	for (bank = 0; size > 0; bank++, size -= write_size) {
		if (size > MPU_MEM_BANK_SIZE)
			write_size = MPU_MEM_BANK_SIZE;
		else
			write_size = size;
		memaddr = (bank << 8);
		if (!bank) {
			memaddr = DMP_OFFSET;
			write_size = MPU_MEM_BANK_SIZE - DMP_OFFSET;
			data += DMP_OFFSET;
		}
		result = mem_w(memaddr, write_size, data);
		if (result) {
			pr_err("error writing firmware:%d\n", bank);
			return result;
		}
		data += write_size;
	}

	return 0;
}

static int inv_verify_firmware(struct inv_mpu_state *st)
{
	int bank, write_size, size;
	int result;
	u16 memaddr;
	u8 firmware[MPU_MEM_BANK_SIZE];
	u8 *data;

	data = st->firmware;
	size = st->dmp_image_size - DMP_OFFSET;
	for (bank = 0; size > 0; bank++, size -= write_size) {
		if (size > MPU_MEM_BANK_SIZE)
			write_size = MPU_MEM_BANK_SIZE;
		else
			write_size = size;

		memaddr = (bank << 8);
		if (!bank) {
			memaddr = DMP_OFFSET;
			write_size = MPU_MEM_BANK_SIZE - DMP_OFFSET;
			data += DMP_OFFSET;
		}
		result = mem_r(memaddr, write_size, firmware);
		if (result)
			return result;
		if (0 != memcmp(firmware, data, write_size)) {
			pr_err("load data error, bank=%d\n", bank);
			return -EINVAL;
		}
		data += write_size;
	}
	return 0;
}

static int inv_compass_dmp_cal(struct inv_mpu_state *st)
{
	s8 *compass_m, *m;
	s8 trans[NINE_ELEM];
	s32 tmp_m[NINE_ELEM];
	int i, j, k;
	int sens[THREE_AXES];
	int *adj;
	int scale, shift;

	compass_m = st->plat_data.secondary_orientation;
	m = st->plat_data.orientation;
	for (i = 0; i < THREE_AXES; i++)
		for (j = 0; j < THREE_AXES; j++)
			trans[THREE_AXES * j + i] = m[THREE_AXES * i + j];

	adj = st->current_compass_matrix;
	st->slave_compass->get_scale(st, &scale);

	if ((COMPASS_ID_AK8975 == st->plat_data.sec_slave_id) ||
			(COMPASS_ID_AK8972 == st->plat_data.sec_slave_id) ||
			(COMPASS_ID_AK8963 == st->plat_data.sec_slave_id) ||
			(COMPASS_ID_AK09912 == st->plat_data.sec_slave_id))
		shift = AK89XX_SHIFT;
	else
		shift = AK99XX_SHIFT;

	for (i = 0; i < THREE_AXES; i++) {
		sens[i] = st->chip_info.compass_sens[i] + 128;
		sens[i] = inv_q30_mult(sens[i] << shift, scale);
	}

	for (i = 0; i < NINE_ELEM; i++) {
		adj[i] = compass_m[i] * sens[i % THREE_AXES];
		tmp_m[i] = 0;
	}
	for (i = 0; i < THREE_AXES; i++)
		for (j = 0; j < THREE_AXES; j++)
			for (k = 0; k < THREE_AXES; k++)
				tmp_m[THREE_AXES * i + j] +=
					trans[THREE_AXES * i + k] *
						adj[THREE_AXES * k + j];

	for (i = 0; i < NINE_ELEM; i++)
		st->final_compass_matrix[i] = adj[i];
	for (i = 0; i < 9; i++)
		tmp_m[i] = 0;
	tmp_m[0] = (1 << 30);
	tmp_m[4] = (1 << 30);
	tmp_m[8] = (1 << 30);

	return 0;
}

static int inv_write_gyro_sf(struct inv_mpu_state *st)
{
	int result;

	result = write_be32_to_mem(st, st->gyro_sf, GYRO_SF);

	return result;
}

static int inv_setup_dmp_firmware(struct inv_mpu_state *st)
{
	int result;
	u8 v[4] = {0, 0};

	result = mem_w(DATA_OUT_CTL1, 2, v);
	if (result)
		return result;
	result = mem_w(DATA_OUT_CTL2, 2, v);
	if (result)
		return result;
	result = mem_w(MOTION_EVENT_CTL, 2, v);
	if (result)
		return result;

	result = inv_write_gyro_sf(st);
	if (result) {
		pr_err("dmp loading eror:inv_write_gyro_sf\n");
		return result;
	}
	if (st->chip_config.has_compass) {
		result = inv_compass_dmp_cal(st);
		if (result)
			return result;
	}
	result = inv_set_flip_pickup_gesture_params_V3(st);

	return result;
}
/*
 * inv_firmware_load() -  calling this function will load the firmware.
 */
int inv_firmware_load(struct inv_mpu_state *st)
{
	int result;

	result = inv_switch_power_in_lp(st, true);
	if (result) {
		pr_err("load firmware set power error\n");
		goto firmware_write_fail;
	}
	result = inv_stop_dmp(st);
	if (result) {
		pr_err("load firmware:stop dmp error\n");
		goto firmware_write_fail;
	}
	result = inv_load_firmware(st);
	if (result) {
		pr_err("load firmware:load firmware eror\n");
		goto firmware_write_fail;
	}
	result = inv_verify_firmware(st);
	if (result) {
		pr_err("load firmware:verify firmware error\n");
		goto firmware_write_fail;
	}
	result = inv_setup_dmp_firmware(st);
	if (result)
		pr_err("load firmware:setup dmp error\n");
firmware_write_fail:
	result |= inv_set_power(st, false);
	if (result) {
		pr_err("load firmware:shuting down power error\n");
		return result;
	}

	st->chip_config.firmware_loaded = 1;

	return 0;
}

int inv_dmp_read(struct inv_mpu_state *st, int off, int size, u8 *buf)
{
	int bank, write_size, data, result;
	u16 memaddr;

	data = 0;
	result = inv_switch_power_in_lp(st, true);
	if (result)
		return result;
	inv_stop_dmp(st);
	for (bank = (off >> 8); size > 0; bank++, size -= write_size,
					data += write_size) {
		if (size > MPU_MEM_BANK_SIZE)
			write_size = MPU_MEM_BANK_SIZE;
		else
			write_size = size;
		memaddr = (bank << 8);
		result = mem_r(memaddr, write_size, &buf[data]);

		if (result)
			return result;
	}

	return 0;
}
