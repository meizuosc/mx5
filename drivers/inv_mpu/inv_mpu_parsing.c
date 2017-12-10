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

#include "inv_mpu_iio.h"

int debug = 0;

static int inv_process_dmp_data(struct inv_mpu_state *st);
static char iden[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

static int inv_update_dmp_ts(struct inv_mpu_state *st, int ind)
{
	int i;
	u32 counter;
	u64 ts;
	enum INV_ENGINE en_ind;

	ts = st->last_run_time - st->sensor[ind].time_calib;
	counter = st->sensor[ind].sample_calib;
	en_ind = st->sensor[ind].engine_base;

	if (ts < 2 * NSEC_PER_SEC)
		return 0;

	if (!st->sensor[ind].calib_flag) {
		st->sensor[ind].sample_calib = 0;
		st->sensor[ind].time_calib = st->last_run_time;
		st->sensor[ind].calib_flag = 1;
		return 0;
	}

	if (ts > 4 * ((u64)NSEC_PER_SEC)) {
#if 0
		while (ts > (4 * (u64)NSEC_PER_SEC)) {
			ts -= st->sensor[ind].dur;
			counter--;
		}
#else
		u32 count = 0;
		u64 sub = ts - (4 * (u64)NSEC_PER_SEC);
		int dur = st->sensor[ind].dur;

		count = sub/dur;
		if( sub%dur ) {
			count++;
		}
		counter -= count;
		ts -= count*dur;
#endif
	}
	if ((counter > 0) &&
		(st->last_run_time - st->eng_info[en_ind].last_update_time >
		2 * NSEC_PER_SEC)) {
		st->sensor[ind].dur = ((u32)ts) / counter;
		st->eng_info[en_ind].dur = st->sensor[ind].dur /
							st->sensor[ind].div;
		st->eng_info[en_ind].base_time = (st->eng_info[en_ind].dur /
						st->eng_info[en_ind].divider) *
						st->eng_info[en_ind].orig_rate;
		st->eng_info[en_ind].last_update_time = st->last_run_time;
		for (i = 0; i < SENSOR_NUM_MAX; i++) {
			if (st->sensor[i].on &&
					(st->sensor[i].engine_base == en_ind))
				st->sensor[i].dur = st->sensor[i].div *
						st->eng_info[en_ind].dur;
		}

	}
	st->sensor[ind].sample_calib = 0;
	st->sensor[ind].time_calib = st->last_run_time;

	return 0;
}

static int be32_to_int(u8 *d)
{
	return (d[0] << 24) | (d[1] << 16) | (d[2] << 8) | d[3];
}

int inv_apply_soft_iron(struct inv_mpu_state *st, s16 *out_1, s32 *out_2)
{
	int *r, i, j;
	s64 tmp;

	r = st->final_compass_matrix;
	for (i = 0; i < THREE_AXES; i++) {
		tmp = 0;
		for (j = 0; j < THREE_AXES; j++)
			tmp  +=
			(s64)r[i * THREE_AXES + j] * (((int)out_1[j]) << 16);
		out_2[i] = (int)(tmp >> 30);
	}

	return 0;
}

static void inv_convert_and_push_16bytes(struct inv_mpu_state *st, u16 hdr,
							u8 *d, u64 t, s8 *m)
{
	int i, j;
	s32 in[3], out[3];

	for (i = 0; i < 3; i++)
		in[i] = be32_to_int(d + i * 4);
	/* multiply with orientation matrix can be optimized like this */
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			if (m[i * 3 + j])
				out[i] = in[j] * m[i * 3 + j];

	inv_push_16bytes_buffer(st, hdr, t, out);
}

static void inv_convert_and_push_8bytes(struct inv_mpu_state *st, u16 hdr,
							u8 *d, u64 t, s8 *m)
{
	int i, j;
	s16 in[3], out[3];

	for (i = 0; i < 3; i++)
		in[i] = be16_to_cpup((__be16 *)(d + i * 2));

	/* multiply with orientation matrix can be optimized like this */
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			if (m[i * 3 + j])
				out[i] = in[j] * m[i * 3 + j];

	inv_push_8bytes_buffer(st, hdr, t, out);
}

static int inv_push_sensor(struct inv_mpu_state *st, int ind, u64 t, u8 *d)
{
	int i, res;
	s16 out_1[3];
	s32 out_2[3];
	u16 hdr;

	res = 0;

	switch (ind) {
	case SENSOR_ACCEL:
		inv_convert_and_push_16bytes(st, ind, d, t, iden);
		break;
	case SENSOR_GYRO:
		inv_convert_and_push_8bytes(st, ind, d, t, iden);
		break;
	case SENSOR_COMPASS:
		for (i = 0; i < 6; i++)
			st->fifo_data[i] = d[i];
		res = st->slave_compass->read_data(st, out_1);
		st->compass_var = be32_to_int(d + 3 * 2);
		/* bad compass handling */
		if ((out_1[0] == BAD_COMPASS_DATA) &&
				(out_1[1] == BAD_COMPASS_DATA) &&
				(out_1[2] == BAD_COMPASS_DATA))
			hdr = COMPASS_HDR_2;
		out_2[0] = out_1[0];
		out_2[1] = out_1[1];
		out_2[2] = out_1[2];
		//inv_apply_soft_iron(st, out_1, out_2);
		inv_push_16bytes_buffer_compass(st, ind, t, out_2);
		break;
	case SENSOR_ALS:
		for (i = 0; i < 8; i++)
			st->fifo_data[i] = d[i];
		if (st->chip_config.has_als) {
			res = st->slave_als->read_data(st, out_1);
			inv_push_8bytes_buffer(st, ind, t, out_1);

			return res;
		}
		break;
	case SENSOR_SIXQ:
		inv_convert_and_push_16bytes(st, ind, d, t, iden);
		break;
	case SENSOR_PEDQ:
		inv_convert_and_push_8bytes(st, ind, d, t, iden);
		break;
	case SENSOR_PRESSURE:
		for (i = 0; i < 6; i++)
			st->fifo_data[i] = d[i];
		if (st->chip_config.has_pressure) {
			res = st->slave_pressure->read_data(st, out_1);
			inv_push_8bytes_buffer(st, ind, t, out_1);
		} else {
			for (i = 0; i < 3; i++)
				out_1[i] = be16_to_cpup((__be16 *)(d + i * 2));

			inv_push_8bytes_buffer(st, ind, t, out_1);
		}
		break;
	case SENSOR_CALIB_GYRO:
		inv_convert_and_push_16bytes(st, ind, d, t, iden);
		break;
	default:
		break;
	}

	return res;
}

static int inv_get_packet_size(struct inv_mpu_state *st, u16 hdr,
							u32 *pk_size, u8 *dptr)
{
	int i, size;
	u16 hdr2;

	size = HEADER_SZ;
	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (hdr & st->sensor[i].output) {
			if (st->sensor[i].on)
				size += st->sensor[i].sample_size;
			else
				return -EINVAL;
		}
	}
	if (hdr & HEADER2_SET) {
		size += HEADER2_SZ;
		hdr2 = be16_to_cpup((__be16 *)(dptr + 2));
		for (i = 0; i < SENSOR_ACCURACY_NUM_MAX; i++) {
			if (hdr2 & st->sensor_accuracy[i].output) {
				if (st->sensor_accuracy[i].on)
					size +=
					st->sensor_accuracy[i].sample_size;
				else
					return -EINVAL;
			}
		}
		if (hdr2 & ACT_RECOG_SET) {
			if (st->chip_config.activity_eng_on) {
				size += ACT_RECOG_SZ;
			} else {
				pr_err("ERROR: activity should not be here\n");
				return -EINVAL;
			}
		}
		if (hdr2 & GYRO_OFF_MASK) {
			if (st->chip_config.activity_eng_on) {
				size += GYRO_AUTO_OFF_SZ;
			} else {
				pr_err("ERROR: activity should not be here\n");
				return -EINVAL;
			}
		}
		if (hdr2 & FLIP_PICKUP_SET) {
			if (st->chip_config.pick_up_enable) {
				size += FLIP_PICKUP_SZ;
			} else {
				pr_err("ERROR: pick up should not be here\n");
			}
		}
	}
	if ((!st->chip_config.step_indicator_on) && (hdr & PED_STEPIND_SET)) {
		pr_err("ERROR: step inditor should not be here=%x\n", hdr);
		return -EINVAL;
	}
	if (hdr & PED_STEPDET_SET) {
		if (st->chip_config.step_detector_on) {
			size += PED_STEPDET_TIMESTAMP_SZ;
		} else {
			pr_err("ERROR: step detector should not be here\n");
			return -EINVAL;
		}
	}
	*pk_size = size;
	
	if( size<=0 ) {
		INV_INFO("size is (%d), error!\n",size);
	}

	return size>0?0:-EINVAL;
}

static int inv_get_dmp_ts(struct inv_mpu_state *st, int i)
{
	st->sensor[i].ts += (st->sensor[i].dur + st->sensor[i].ts_adj);
	if (st->header_count == 1) {
		inv_update_dmp_ts(st, i);
	}

	return 0;
}

static int inv_process_step_det(struct inv_mpu_state *st, u8 *dptr)
{
	u32 tmp;
	u64 t;
	s16 s[3];

	tmp = be32_to_int(dptr);
	tmp = inv_get_cntr_diff(st->start_dmp_counter, tmp);
	t = st->last_run_time - (u64)tmp * st->eng_info[ENGINE_ACCEL].dur;
	if (st->curr_steps > st->step_det_count)
		tmp = st->curr_steps - st->step_det_count;
	else
		tmp = 0;
	if (st->batch.on)
		inv_send_steps(st, tmp, t);
	if (st->step_detector_l_on)
		inv_push_8bytes_buffer(st, STEP_DETECTOR_HDR, t, s);
	if (st->step_detector_wake_l_on)
		inv_push_8bytes_buffer(st, STEP_DETECTOR_WAKE_HDR, t, s);

	return 0;
}

static int inv_parse_packet(struct inv_mpu_state *st, u16 hdr, u8 *dptr)
{
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	int i, r;
	u32 tmp;
	s16 s[3];
	u8 w;
	u64 t;
	u16 hdr2 = 0;
	bool data_header;

	t = 0;
	if (hdr & HEADER2_SET) {
		hdr2 = be16_to_cpup((__be16 *)(dptr));
		dptr += HEADER2_SZ;
	}

	data_header = false;
	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (hdr & st->sensor[i].output) {
			inv_get_dmp_ts(st, i);
			st->sensor[i].sample_calib++;
			inv_push_sensor(st, i, st->sensor[i].ts, dptr);
			dptr += st->sensor[i].sample_size;
			t = st->sensor[i].ts;
			data_header = true;
		}
	}
	if (data_header)
		st->header_count--;
	if (hdr & PED_STEPDET_SET) {
		inv_process_step_det(st, dptr);
		dptr += PED_STEPDET_TIMESTAMP_SZ;
		st->step_det_count--;
	}

	if (hdr & PED_STEPIND_MASK)
		inv_push_step_indicator(st, t);
	if (hdr2) {
		if (hdr2 & ACT_RECOG_SET) {
			tmp = be32_to_int(dptr + 2);
			tmp = inv_get_cntr_diff(tmp, st->start_dmp_counter);

			t = st->step_detector_base_ts + (u64)tmp *
						st->eng_info[ENGINE_ACCEL].dur;
			s[0] = be16_to_cpup((__be16 *)(dptr));
			s[1] = 0;
			s[2] = 0;
			inv_push_8bytes_kf(st, ACTIVITY_HDR, t, s);
			dptr += ACT_RECOG_SZ;
		}
		if (hdr2 & GYRO_OFF_MASK) {
			if (!st->chip_config.gyro_enable) {
				r = inv_plat_read(st, REG_PWR_MGMT_2, 1, &w);
				w |= BIT_PWR_GYRO_STBY;
				r = inv_plat_single_write(st,
							REG_PWR_MGMT_2, w);
				if (r)
					return r;
			}
			dptr += GYRO_AUTO_OFF_SZ;
		}
		if (hdr2 & FLIP_PICKUP_SET) {
			sysfs_notify(&indio_dev->dev.kobj, NULL, "poll_pick_up");
			dptr += FLIP_PICKUP_SZ;
			st->chip_config.pick_up_enable = 0;
			inv_check_sensor_on(st);
			set_inv_enable(indio_dev);
		}
	}
	return 0;
}

static int inv_bound_timestamp(struct inv_mpu_state *st)
{
	s64 elaps_time, thresh1, thresh2;
	int i;

#define INV_TIME_CALIB_THRESHOLD_1 2
#define JITTER_THRESH (NSEC_PER_MSEC >> 2)
#define DELAY_THRESH  (2 * NSEC_PER_MSEC)

	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (st->sensor[i].on) {
			if (st->sensor[i].count) {
				elaps_time = ((u64)(st->sensor[i].dur)) *
							st->sensor[i].count;
				thresh1 = st->last_run_time - elaps_time -
								JITTER_THRESH;
				thresh2 = thresh1 - st->sensor[i].dur;
				if (thresh1 < 0)
					thresh1 = 0;
				if (thresh2 < 0)
					thresh2 = 0;
				st->sensor[i].ts_adj = 0;
				if (st->sensor[i].ts > thresh1)
					st->sensor[i].ts_adj =
						(int)(thresh1 -
							st->sensor[i].ts);
				if (st->time_calib_counter >=
						INV_TIME_CALIB_THRESHOLD_1) {
					if (st->sensor[i].ts < thresh2)
						st->sensor[i].ts_adj =
							(int)(thresh2 -
							st->sensor[i].ts);
				} else {
					st->sensor[i].ts = st->last_run_time -
								elaps_time -
								DELAY_THRESH;
				}
				if (st->sensor[i].ts_adj &&
						(st->sensor[i].count > 1))
					st->sensor[i].ts_adj /=
							st->sensor[i].count;
			} else if (st->time_calib_counter <
						INV_TIME_CALIB_THRESHOLD_1) {
				st->sensor[i].ts = st->reset_ts;
			}
		}
	}

	return 0;
}

static int inv_pre_parse_packet(struct inv_mpu_state *st, u16 hdr, u8 *dptr)
{
	int i;
	u16 hdr2 = 0;
	bool data_header;

	if (hdr & HEADER2_SET) {
		hdr2 = be16_to_cpup((__be16 *)(dptr));
		dptr += HEADER2_SZ;
	}

	data_header = false;
	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (hdr & st->sensor[i].output) {
			st->sensor[i].count++;
			dptr += st->sensor[i].sample_size;
			data_header = true;
		}
	}
	if (data_header)
		st->header_count++;
	if (hdr & PED_STEPDET_SET) {
		st->step_det_count++;
		dptr += PED_STEPDET_TIMESTAMP_SZ;
	}
	if (hdr2) {
		if (hdr2 & ACT_RECOG_SET)
			dptr += ACT_RECOG_SZ;
		if (hdr2 & FLIP_PICKUP_SET)
			dptr += FLIP_PICKUP_SZ;

		for (i = 0; i < SENSOR_ACCURACY_NUM_MAX; i++) {
			if (hdr2 & st->sensor_accuracy[i].output)
				dptr += st->sensor_accuracy[i].sample_size;
		}
	}
	return 0;
}

static int inv_get_step_params(struct inv_mpu_state *st)
{
	int result;

	result = inv_switch_power_in_lp(st, true);
	result |= read_be32_from_mem(st, &st->curr_steps, PEDSTD_STEPCTR);
	result |= read_be32_from_mem(st, &st->start_dmp_counter, DMPRATE_CNTR);
	result |= inv_switch_power_in_lp(st, false);

	return result;
}
static int inv_prescan_data(struct inv_mpu_state *st, u8 *dptr, int len)
{
	int res, pk_size, i;
	bool done_flag;
	u16 hdr;

	done_flag = false;
	st->header_count = 0;
	st->step_det_count = 0;
	st->time_calib_counter++;
	for (i = 0; i < SENSOR_NUM_MAX; i++)
		st->sensor[i].count = 0;
	while (!done_flag) {
		if (len > HEADER_SZ) {
			hdr = (u16)be16_to_cpup((__be16 *)(dptr));
			if (!hdr) {
				pr_err("error header zero\n");
				st->left_over_size = 0;
				return -EINVAL;
			}
			res = inv_get_packet_size(st, hdr, &pk_size, dptr);
			if (res) {
				if (!st->chip_config.is_asleep)
					pr_err("error in header parsing=%x\n",
									hdr);
				st->left_over_size = 0;

				return -EINVAL;
			}
			if (len >= pk_size) {
				inv_pre_parse_packet(st, hdr, dptr + HEADER_SZ);
				len -= pk_size;
				dptr += pk_size;
			} else {
				done_flag = true;
			}
		} else {
			done_flag = true;
		}
	}
	if (st->step_det_count)
		inv_get_step_params(st);
	inv_bound_timestamp(st);

	return 0;
}

static u8 fifo_data[HARDWARE_FIFO_SIZE];
static int inv_process_dmp_data(struct inv_mpu_state *st)
{
	int total_bytes, tmp, res, fifo_count, pk_size;
	u8 *dptr, *d;
	u16 hdr;
	u8 data[2];
	bool done_flag;


	res = inv_plat_read(st, REG_FIFO_COUNT_H, FIFO_COUNT_BYTE, data);
	if (res)
		return res;
	fifo_count = be16_to_cpup((__be16 *)(data));
	if (!fifo_count)
		return 0;

	if (fifo_count > HARDWARE_FIFO_SIZE || fifo_count<0){
             inv_reset_fifo(st, true);
               return 0;
    }

	st->fifo_count = fifo_count;
	d = fifo_data;

	if (st->left_over_size > LEFT_OVER_BYTES) {
		st->left_over_size = 0;
		return -EINVAL;
	}

	if( fifo_count+st->left_over_size >  HARDWARE_FIFO_SIZE || st->left_over_size<0 ) {
		st->left_over_size = 0;
	}

	if (st->left_over_size > 0)
		memcpy(d, st->left_over, st->left_over_size);


	if(debug>1) {INV_INFO("fifo_count:%d,left_over_size:%d,fifo+left:%d>512bytes??\n",
			fifo_count,st->left_over_size, fifo_count+st->left_over_size); dump_stack();}

	dptr = d + st->left_over_size;
	total_bytes = fifo_count;

	while (total_bytes > 0) {
		if (total_bytes < MAX_READ_SIZE)
			tmp = total_bytes;
		else
			tmp = MAX_READ_SIZE;
		res = inv_plat_read(st, REG_FIFO_R_W, tmp, dptr);
		if (res < 0)
			return res;
		dptr += tmp;
		total_bytes -= tmp;

		if(debug>2) {INV_INFO("total_bytes:%d,tmp:%d\n",total_bytes,tmp); dump_stack();}
	}
	dptr = d;
	total_bytes = fifo_count + st->left_over_size;
	res = inv_prescan_data(st, dptr, total_bytes);
	if (res)
		return -EINVAL;
	dptr = d;
	done_flag = false;
	while (!done_flag) {
		if (total_bytes > HEADER_SZ) {

			if( dptr-d>=HARDWARE_FIFO_SIZE ) {
				return -EINVAL;
			}

			hdr = (u16)be16_to_cpup((__be16 *)(dptr));
			res = inv_get_packet_size(st, hdr, &pk_size, dptr);
			if (res) {
				pr_err("error in header parsing=%x\n", hdr);
				st->left_over_size = 0;

				return -EINVAL;
			}
			if (total_bytes >= pk_size) {
				inv_parse_packet(st, hdr, dptr + HEADER_SZ);
				total_bytes -= pk_size;
				dptr += pk_size;
			} else {
				done_flag = true;
			}
		} else {
			done_flag = true;
		}
		if(debug>2) {INV_INFO("total_bytes:%d,pk_size:%d\n",total_bytes,pk_size); dump_stack();}
	}
	st->left_over_size = total_bytes;
	if (st->left_over_size > LEFT_OVER_BYTES) {
		st->left_over_size = 0;
		return -EINVAL;
	}

	if (st->left_over_size)
		memcpy(st->left_over, dptr, st->left_over_size);

	if(debug>1) {INV_INFO("total_bytes:%d,left_over_size:%d, exit func\n",total_bytes,st->left_over_size); dump_stack();}

	return 0;
}

static int inv_process_update_ts(struct inv_mpu_state *st, enum INV_SENSORS t)
{
	u32 diff, dur, adj;
	u64 ts;

	ts = st->ts_for_calib - st->sensor[t].time_calib;

	if (ts < 2 * NSEC_PER_SEC)
		return 0;
	if (ts > 4 * ((u64)NSEC_PER_SEC)) {
		while (ts > (4 * (u64)NSEC_PER_SEC)) {
			ts >>= 1;
			st->sensor[t].sample_calib >>= 1;
		}
	}
	if (!st->sensor[t].sample_calib) {
		st->sensor[t].time_calib = st->ts_for_calib;
		return 0;
	}

	diff = (u32)ts;
	dur = diff  / st->sensor[t].sample_calib;
	adj = abs(dur - st->sensor[t].dur);
	adj >>= 3;
	if (adj > (st->sensor[t].dur >> 5))
		adj = (st->sensor[t].dur >> 5);
	if (dur > st->sensor[t].dur)
		st->sensor[t].dur += adj;
	else
		st->sensor[t].dur -= adj;
	st->sensor[t].sample_calib = 0;
	st->sensor[t].time_calib = st->ts_for_calib;

	return 0;

}

static int inv_push_data(struct inv_mpu_state *st, enum INV_SENSORS type)
{
	int result;
	int i, fifo_count, bpm, read_size, f;
	u8 *dptr;
	u8 data[2];
	s16 out[3];
	s32 out_2[3];

	st->ts_for_calib = get_time_ns();
	result = inv_plat_read(st, REG_FIFO_COUNT_H, FIFO_COUNT_BYTE, data);
	if (result)
		return result;
	fifo_count = be16_to_cpup((__be16 *)(data));
	if (!fifo_count)
		return 0;

	dptr = fifo_data;
	if (type == SENSOR_COMPASS)
		bpm = 8;
	else
		bpm = 6;
	f = fifo_count;
	while (f >= bpm) {
		if (f < MAX_READ_SIZE)
			read_size = f;
		else
			read_size = MAX_READ_SIZE;
		read_size = read_size / bpm;
		read_size *= bpm;
		result = inv_plat_read(st, REG_FIFO_R_W, read_size, dptr);
		if (result < 0)
			return result;
		dptr += read_size;
		f -= read_size;
	}
	dptr = fifo_data;
	while (fifo_count >= bpm) {
		st->sensor[type].ts += st->sensor[type].dur;
		if (type == SENSOR_ACCEL) {
			for (i = 0; i < 3; i++) {
				out[i] = be16_to_cpup((__be16 *)(dptr + i * 2));
				out_2[i] = (out[i] << 15);
			}
			inv_push_16bytes_buffer(st, type,
						st->sensor[type].ts, out_2);
		} else if (type == SENSOR_COMPASS) {
			for (i = 0; i < 3; i++)
				out[i] = (short)((dptr[i * 2 + 1] << 8) |
							dptr[i * 2 + 2]);
			inv_push_8bytes_buffer(st, type,
						st->sensor[type].ts, out);
		} else {
			for (i = 0; i < 3; i++)
				out[i] = be16_to_cpup((__be16 *)(dptr + i * 2));
			inv_push_8bytes_buffer(st, type,
						st->sensor[type].ts, out);
		}
		fifo_count -= bpm;
		dptr += bpm;
		st->sensor[type].sample_calib++;
	}
	inv_process_update_ts(st, type);

	return 0;
}

static int inv_set_fifo_read_data(struct inv_mpu_state *st,
					enum INV_SENSORS type, u8 cfg)
{
	int res;

	res = inv_plat_single_write(st, REG_FIFO_CFG, cfg);
	if (res)
		return res;
	res = inv_push_data(st, type);

	return res;
}

static int inv_process_non_dmp_data(struct inv_mpu_state *st)
{
	int res;
	u8 cfg;

	res = 0;
	if (st->sensor[SENSOR_GYRO].on && st->sensor[SENSOR_ACCEL].on) {
		cfg = (BIT_MULTI_FIFO_CFG | BIT_GYRO_FIFO_NUM);
		res = inv_set_fifo_read_data(st, SENSOR_GYRO, cfg);
		if (res)
			return res;
		cfg = (BIT_MULTI_FIFO_CFG | BIT_ACCEL_FIFO_NUM);
		res = inv_set_fifo_read_data(st, SENSOR_ACCEL, cfg);
	} else {
		if (st->sensor[SENSOR_GYRO].on)
			res = inv_push_data(st, SENSOR_GYRO);
		if (st->sensor[SENSOR_ACCEL].on)
			res = inv_push_data(st, SENSOR_ACCEL);
		if (st->sensor[SENSOR_COMPASS].on)
			res = inv_push_data(st, SENSOR_COMPASS);
	}

	return res;
}

static int inv_get_gyro_bias(struct inv_mpu_state *st, int *bias)
{
	int b_addr[] = {GYRO_BIAS_X, GYRO_BIAS_Y, GYRO_BIAS_Z};
	int i, r;

	for (i = 0; i < THREE_AXES; i++) {
		r = read_be32_from_mem(st, &bias[i], b_addr[i]);
		if (r)
			return r;
	}

	return 0;
}
static int inv_process_temp_comp(struct inv_mpu_state *st)
{
	u8 d[2];
	int r, l1, scale_t, curr_temp, i;
	s16 temp;
	s64 tmp, recp;
	bool update_slope;
	struct inv_temp_comp *t_c;
	int s_addr[] = {GYRO_SLOPE_X, GYRO_SLOPE_Y, GYRO_SLOPE_Z};

#define TEMP_COMP_WIDTH  4
#define TEMP_COMP_MID_L  (12 + TEMP_COMP_WIDTH)
#define TEMP_COMP_MID_H  (32 + TEMP_COMP_WIDTH)

	if (st->last_run_time - st->last_temp_comp_time < (NSEC_PER_SEC >> 1))
		return 0;
	st->last_temp_comp_time = st->last_run_time;
	if ((!st->gyro_cal_enable) ||
		(!st->chip_config.gyro_enable) ||
		(!st->chip_config.accel_enable))
		return 0;
	r = inv_plat_read(st, REG_TEMPERATURE, 2, d);
	if (r)
		return r;
	temp = (s16)(be16_to_cpup((short *)d));
	scale_t = TEMPERATURE_OFFSET +
		inv_q30_mult((int)temp << MPU_TEMP_SHIFT, TEMPERATURE_SCALE);
	curr_temp = (scale_t >> MPU_TEMP_SHIFT);

	update_slope = false;
	/* check the lower part of the temperature */
	l1 = abs(curr_temp - TEMP_COMP_MID_L);
	l1 = l1 - TEMP_COMP_WIDTH;
	l1 = l1 - TEMP_COMP_WIDTH;
	t_c = &st->temp_comp;
	if (l1 < 0) {
		t_c->t_lo = temp;
		r = inv_get_gyro_bias(st, t_c->b_lo);
		if (r)
			return r;
		t_c->has_low = true;
		update_slope = true;
	}

	l1 = abs(curr_temp - TEMP_COMP_MID_H);
	l1 = l1 - TEMP_COMP_WIDTH;
	l1 = l1 - TEMP_COMP_WIDTH;
	if (l1 < 0) {
		t_c->t_hi = temp;
		r = inv_get_gyro_bias(st, t_c->b_hi);
		if (r)
			return r;
		t_c->has_high = true;
		update_slope = true;
	}
	if (t_c->has_high && t_c->has_low && update_slope) {
		if (t_c->t_hi != t_c->t_lo) {
			recp = (1 << 30) / (t_c->t_hi - t_c->t_lo);
			for (i = 0; i < THREE_AXES; i++) {
				tmp = recp * (t_c->b_hi[i] - t_c->b_lo[i]);
				t_c->slope[i] = (tmp >> 15);
				r = write_be32_to_mem(st,
						t_c->slope[i], s_addr[i]);
				if (r)
					return r;
			}
		}
	}

	return 0;
}
static int inv_process_dmp_interrupt(struct inv_mpu_state *st)
{
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	u8 d[1];
	int result;
	u32 step=0;


#define DMP_INT_SMD             0x04
#define DMP_INT_PED             0x08

	if ((!st->smd.on) && (!st->ped.on))
		return 0;

	result = inv_plat_read(st, REG_DMP_INT_STATUS, 1, d);
	if (result)
		return result;

	if (d[0] & DMP_INT_SMD) {
		sysfs_notify(&indio_dev->dev.kobj, NULL, "poll_smd");
		st->smd.on = false;
		st->trigger_state = EVENT_TRIGGER;
		set_inv_enable(indio_dev);
	}
	step = -1;
	if (st->ped.on && (!st->batch.on)) {
		inv_switch_power_in_lp(st, true);
		if (st->ped.int_on) {
			if (d[0] & DMP_INT_PED) {
				sysfs_notify(&indio_dev->dev.kobj, NULL,
							"poll_pedometer");
				inv_get_pedometer_steps(st, &step);
			}
		} else {
			inv_get_pedometer_steps(st, &step);
		}
		inv_switch_power_in_lp(st, false);
		if ((step != -1) && (step != st->prev_steps)) {
			inv_send_steps(st, step, st->last_run_time);
			st->prev_steps = step;
		}
	}
	return 0;
}
/*
 *  inv_read_fifo() - Transfer data from FIFO to ring buffer.
 */
irqreturn_t inv_read_fifo(int irq, void *dev_id)
 {
 	struct inv_mpu_state *st = (struct inv_mpu_state *)dev_id;
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	int result, min_run_time;
	u64 pts1;

#define NON_DMP_MIN_RUN_TIME (40 * NSEC_PER_MSEC)

	down(&st->suspend_resume_sema);
	mutex_lock(&indio_dev->mlock);

	if(debug>1) INV_INFO("enter mutex lock\n");

	st->last_run_time = get_time_ns();
	if (st->chip_config.is_asleep)
		goto end_read_fifo;
	inv_switch_power_in_lp(st, true);
	if (st->chip_config.dmp_on) {
		st->activity_size = 0;
		result = inv_process_dmp_interrupt(st);
		if (result)
			goto end_read_fifo;
		result = inv_process_temp_comp(st);
		if (result)
			goto end_read_fifo;
		result = inv_process_dmp_data(st);
		if (st->activity_size > 0)
			sysfs_notify(&indio_dev->dev.kobj,
							NULL, "poll_activity");
	} else {
		pts1 = get_time_ns();
		if ((!st->sensor[SENSOR_COMPASS].on))
			min_run_time = NON_DMP_MIN_RUN_TIME;
		else
			min_run_time = min((int)NON_DMP_MIN_RUN_TIME,
						st->sensor[SENSOR_COMPASS].dur);

		if (pts1 - st->last_run_time < min_run_time)
			goto end_read_fifo;
		else
			st->last_run_time = pts1;
		result = inv_process_non_dmp_data(st);
	}
	if (result)
		goto err_reset_fifo;

end_read_fifo:
	inv_switch_power_in_lp(st, false);
	mutex_unlock(&indio_dev->mlock);
	up(&st->suspend_resume_sema);

	if(debug>1) INV_INFO("exit mutex lock0\n");

	return IRQ_HANDLED;

err_reset_fifo:
	if ((!st->chip_config.gyro_enable) &&
		(!st->chip_config.accel_enable) &&
		(!st->chip_config.slave_enable) &&
		(!st->chip_config.pressure_enable)) {
		inv_switch_power_in_lp(st, false);
		mutex_unlock(&indio_dev->mlock);
		up(&st->suspend_resume_sema);

		if(debug>1) INV_INFO("exit mutex lock1\n");

		return IRQ_HANDLED;
	}

	pr_err("error to reset fifo\n");
	inv_reset_fifo(st, true);
	inv_switch_power_in_lp(st, false);
	mutex_unlock(&indio_dev->mlock);
	up(&st->suspend_resume_sema);

	if(debug>1) INV_INFO("exit mutex lock2\n");

	return IRQ_HANDLED;
}
 
int inv_flush_batch_data(struct iio_dev *indio_dev, int data)
{
	struct inv_mpu_state *st = iio_priv(indio_dev);
	int result;
	u8 w;

	if (!(iio_buffer_enabled(indio_dev)))
		return -EINVAL;

	if (st->batch.on) {
		inv_switch_power_in_lp(st, true);
		st->last_run_time = get_time_ns();
		result = inv_plat_read(st, REG_USER_CTRL, 1, &w);
		w &= ~BIT_DMP_EN;
		result = inv_plat_single_write(st, REG_USER_CTRL, w);
		result = write_be32_to_mem(st, 0, BM_BATCH_CNTR);
		w |= BIT_DMP_EN;
		result = inv_plat_single_write(st, REG_USER_CTRL, w);

		inv_process_dmp_data(st);
		inv_switch_power_in_lp(st, false);
		inv_push_marker_to_buffer(st, END_MARKER, data);
		return result;
	}
	inv_push_marker_to_buffer(st, EMPTY_MARKER, data);

	return 0;
}
