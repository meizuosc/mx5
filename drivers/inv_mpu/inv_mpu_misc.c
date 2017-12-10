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

int inv_get_pedometer_steps(struct inv_mpu_state *st, u32 *ped)
{
	int r;

	r = read_be32_from_mem(st, ped, PEDSTD_STEPCTR);
	INV_INFO("stepcounter prev_steps:%u, steps:%u\n", st->prev_steps, *ped);
	return r;
}
int inv_get_pedometer_time(struct inv_mpu_state *st, int *ped)
{
	int r;

	r = read_be32_from_mem(st, ped, PEDSTD_TIMECTR);

	return r;
}

int inv_read_pedometer_counter(struct inv_mpu_state *st)
{
	int result;
	u32 last_step_counter, curr_counter;
	u64 counter;

	result = read_be32_from_mem(st, &last_step_counter, STPDET_TIMESTAMP);
	if (result)
		return result;
	if (0 != last_step_counter) {
		result = read_be32_from_mem(st, &curr_counter, DMPRATE_CNTR);
		if (result)
			return result;
		counter = inv_get_cntr_diff(curr_counter, last_step_counter);
		st->ped.last_step_time = get_time_ns() - counter *
						st->eng_info[ENGINE_ACCEL].dur;
	}

	return 0;
}

int inv_check_sensor_on(struct inv_mpu_state *st)
{
	int i;

	for (i = 0; i < SENSOR_NUM_MAX; i++)
		st->sensor[i].on = false;
	st->chip_config.wake_on = false;
	for (i = 0; i < SENSOR_L_NUM_MAX; i++) {
		if (st->sensor_l[i].on && st->sensor_l[i].rate) {
			st->sensor[st->sensor_l[i].base].on = true;
			st->chip_config.wake_on |= st->sensor_l[i].wake_on;
		}
	}
	if (st->step_detector_wake_l_on ||
			st->step_counter_wake_l_on ||
			st->chip_config.pick_up_enable ||
			st->smd.on ||
			st->chip_config.tilt_enable)
		st->chip_config.wake_on = true;

	return 0;
}

int inv_check_sensor_rate(struct inv_mpu_state *st)
{
	int i;

	for (i = 0; i < SENSOR_NUM_MAX; i++)
		st->sensor[i].rate = 0;
	for (i = 0; i < SENSOR_L_NUM_MAX; i++) {
		if (st->sensor_l[i].on) {
			st->sensor[st->sensor_l[i].base].rate = max(
				st->sensor[st->sensor_l[i].base].rate,
				st->sensor_l[i].rate);
		}
	}
	for (i = 0; i < SENSOR_L_NUM_MAX; i++) {
		if (st->sensor_l[i].on) {
			if (st->sensor_l[i].rate)
				st->sensor_l[i].div =
					st->sensor[st->sensor_l[i].base].rate
						/ st->sensor_l[i].rate;
			else
				st->sensor_l[i].div = 0xffff;
		}
	}

	return 0;
}


