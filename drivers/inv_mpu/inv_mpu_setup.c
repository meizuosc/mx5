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
#include "inv_mpu_iio.h"

struct inv_local_store {
	u8 reg_lp_config;
	u8 reg_delay_enable;
	u8 reg_delay_time;
	u8 reg_gyro_smplrt;
	bool wom_on;
	bool activity_eng_on;
	int accel_cal_ind;
};

static struct inv_local_store local;

struct inv_accel_cal_params {
	int freq;
	int rate;
	int bac_rate;
	int gain;
	int alpha;
	int a;
};

static struct inv_accel_cal_params accel_cal_para[] = {
	{
		.freq = 1000,
	},
	{
		.freq = 225,
		.rate = 0,
		.bac_rate = 3,
		.gain = DEFAULT_ACCEL_GAIN,
		.alpha = 1026019965,
		.a     = 47721859,
	},
	{
		.freq = 112,
		.gain = DEFAULT_ACCEL_GAIN_112,
		.rate = 0,
		.bac_rate = 1,
		.alpha = 977872018,
		.a     = 95869806,
	},
	{
		.freq = 56,
		.gain = PED_ACCEL_GAIN,
		.rate = 0,
		.bac_rate = 0,
		.alpha = 882002213,
		.a     = 191739611,
	},
	{
		.freq = 15,
		.gain = DEFAULT_ACCEL_GAIN,
		.rate = 0,
		.bac_rate = 0,
		.alpha = 357913941,
		.a     = 715827883,
	},
	{
		.freq = 5,
		.gain = DEFAULT_ACCEL_GAIN,
		.rate = 0,
		.bac_rate = 0,
		.alpha = 107374182,
		.a     = 966367642,
	},
};

static int accel_gyro_rate[] = {5, 6, 7, 8, 9, 10, 11, 12, 13,
				14, 15, 17, 18, 22, 25, 28, 32, 37, 45,
							56, 75, 112, 225};

static int inv_set_batchmode(struct inv_mpu_state *st, bool enable)
{
	if (enable)
		st->cntl2 |= BATCH_MODE_EN;

	return 0;
}
static int inv_calc_engine_dur(struct inv_engine_info *ei)
{
	if (!ei->running_rate)
		return -EINVAL;

	ei->dur = ei->base_time / ei->orig_rate;
	ei->dur *= ei->divider;

	return 0;
}
static int inv_batchmode_calc(struct inv_mpu_state *st)
{
	int b, timeout;
	int i, bps;
	enum INV_ENGINE eng;

	bps = 0;
	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (st->sensor[i].on) {
			bps += (st->sensor[i].sample_size + 2) *
							st->sensor[i].rate;
		}
	}
	if (bps) {
		b = st->batch.timeout * bps;
		if ((b > (FIFO_SIZE * MSEC_PER_SEC)) &&
						(!st->batch.overflow_on))
			timeout = FIFO_SIZE * MSEC_PER_SEC / bps;
		else
			timeout = st->batch.timeout;
	} else {
		if (st->chip_config.step_detector_on ||
					st->step_counter_l_on ||
					st->step_counter_wake_l_on ||
					st->chip_config.activity_eng_on){
			timeout = st->batch.timeout;
		} else {
			return -EINVAL;
		}
	}
	if (st->chip_config.gyro_enable)
		eng = ENGINE_GYRO;
	else if (st->chip_config.accel_enable)
		eng =ENGINE_ACCEL;
	else
		eng = ENGINE_I2C;
	b = st->eng_info[eng].dur / USEC_PER_MSEC;
	st->batch.engine_base = eng;
	st->batch.counter = timeout * USEC_PER_MSEC / b;

	if (st->batch.counter)
		st->batch.on = true;

	return 0;
}

static int inv_set_default_batch(struct inv_mpu_state *st)
{
	if (st->batch.max_rate > DEFAULT_BATCH_RATE) {
		st->batch.default_on = true;
		st->batch.counter = DEFAULT_BATCH_TIME * NSEC_PER_MSEC /
					st->eng_info[ENGINE_GYRO].dur;
	}

	return 0;
}
int inv_batchmode_setup(struct inv_mpu_state *st)
{
	int r;
	bool on;
	s16 mask[ENGINE_NUM_MAX] = {1, 2, 8, 8};

	st->batch.on = false;
	st->batch.default_on = false;
	if (st->batch.timeout > 0) {
		r = inv_batchmode_calc(st);
		if (r)
			return r;
	} else {
		r = inv_set_default_batch(st);
		if (r)
			return r;
	}

	on = (st->batch.on || st->batch.default_on);

	if (on) {
		r = write_be32_to_mem(st, 0, BM_BATCH_CNTR);
		if (r)
			return r;
		r = write_be32_to_mem(st, st->batch.counter, BM_BATCH_THLD);
		if (r)
			return r;
		r = inv_write_2bytes(st, BM_BATCH_MASK,
						mask[st->batch.engine_base]);
		if (r)
			return r;
		r = inv_write_2bytes(st, BM_FIFO_SIZE, FIFO_SIZE);
		if (r)
			return r;
	}

	r = inv_set_batchmode(st, on);

	return r;
}

static int inv_turn_on_fifo(struct inv_mpu_state *st)
{
	u8 w, x;
	int r;

	/* clear FIFO data */
	r = inv_plat_single_write(st, REG_FIFO_RST, MAX_5_BIT_VALUE);
	if (r)
		return r;
	r = inv_plat_single_write(st, REG_FIFO_RST, MAX_5_BIT_VALUE - 1);
	if (r)
		return r;
	/* turn on FIFO output data  for non-DMP mode */
	w = 0;
	x = 0;
	if (!st->chip_config.dmp_on) {
		if (st->sensor[SENSOR_GYRO].on)
			w |= BITS_GYRO_FIFO_EN;
		if (st->sensor[SENSOR_ACCEL].on)
			w |= BIT_ACCEL_FIFO_EN;
		if (st->sensor[SENSOR_COMPASS].on)
			x |= BIT_SLV_0_FIFO_EN;
	}
	r = inv_plat_single_write(st, REG_FIFO_EN_2, w);
	if (r)
		return r;
	r = inv_plat_single_write(st, REG_FIFO_EN, x);
	if (r)
		return r;
	/* turn on interrupt */
	if (st->chip_config.dmp_on) {
		w = BIT_DMP_INT_EN;
		x = BIT_FIFO_OVERFLOW_EN_0;
		r = inv_plat_single_write(st, REG_INT_ENABLE, w);
		if (r)
			return r;
		r = inv_plat_single_write(st, REG_INT_ENABLE_2, x);
	} else {
		w = 0;
		if (st->sensor[SENSOR_GYRO].on && st->sensor[SENSOR_ACCEL].on)
			w = (BIT_DATA_RDY_0_EN | BIT_DATA_RDY_1_EN);
		else
			w = BIT_DATA_RDY_0_EN;
		r = inv_plat_single_write(st, REG_INT_ENABLE_1, w);
	}

	w = BIT_FIFO_EN;
	if (st->chip_config.dmp_on)
		w |= BIT_DMP_EN;
	if (st->chip_config.slave_enable)
			w |= BIT_I2C_MST_EN;
	r = inv_plat_single_write(st, REG_USER_CTRL, w | st->i2c_dis);

	return r;
}

/*
 *  inv_reset_fifo() - Reset FIFO related registers.
 */
int inv_reset_fifo(struct inv_mpu_state *st, bool turn_off)
{
	int r, i;

	st->last_run_time = get_time_ns();
	st->reset_ts = st->last_run_time;
	r = inv_turn_on_fifo(st);
	if (r)
		return r;

	st->last_temp_comp_time = st->last_run_time;
	st->left_over_size        = 0;
	for (i = 0; i < SENSOR_NUM_MAX; i++)
		st->sensor[i].calib_flag = 0;

	st->time_calib_counter = 0;
	return 0;
}

static int inv_turn_on_engine(struct inv_mpu_state *st)
{
	u8 w;
	int r;


	if (st->chip_config.gyro_enable | st->chip_config.accel_enable) {
		w = BIT_PWR_PRESSURE_STBY;
		if (!st->chip_config.gyro_enable)
			w |= BIT_PWR_GYRO_STBY;
		if (!st->chip_config.accel_enable)
			w |= BIT_PWR_ACCEL_STBY;
	} else {
		w = (BIT_PWR_GYRO_STBY |
				BIT_PWR_ACCEL_STBY |
				BIT_PWR_PRESSURE_STBY);
	}
	r = inv_plat_single_write(st, REG_PWR_MGMT_2, w);
	if (r)
		return r;

	if (st->chip_config.has_compass) {
		if (st->chip_config.compass_enable)
			r = st->slave_compass->resume(st);
		else
			r = st->slave_compass->suspend(st);
		if (r)
			return r;
	}
	if (st->chip_config.has_als) {
		if (st->sensor[SENSOR_ALS].on)
			r = st->slave_als->resume(st);
		else
			r = st->slave_als->suspend(st);
		if (r)
			return r;
	}
	if (st->chip_config.has_pressure) {
		if (st->chip_config.pressure_enable)
			r = st->slave_pressure->resume(st);
		else
			r = st->slave_pressure->suspend(st);
		if (r)
			return r;
	}

	/* secondary cycle mode should be set all the time */
	w = BIT_I2C_MST_CYCLE;
	if (st->chip_config.low_power_gyro_on)
		w |= BIT_GYRO_CYCLE;
	w |= BIT_ACCEL_CYCLE;
	if (w != local.reg_lp_config) {
		r = inv_plat_single_write(st, REG_LP_CONFIG, w);
		if (r)
			return r;
		local.reg_lp_config = w;
	}

	return 0;
}

static int inv_setup_dmp_rate(struct inv_mpu_state *st)
{
	int i, result;
	int div[SENSOR_NUM_MAX];
	bool d_flag;

	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (st->sensor[i].on) {
			if (!st->sensor[i].rate) {
				pr_err("sensor %d rate is zero\n", i);
				return -EINVAL;
			}
			div[i] =
			st->eng_info[st->sensor[i].engine_base].running_rate /
							st->sensor[i].rate;
			if (!div[i])
				div[i] = 1;
			st->sensor[i].rate = st->eng_info
				[st->sensor[i].engine_base].running_rate /
				div[i];
		}
	}
	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (st->sensor[i].on) {
			st->cntl |= st->sensor[i].output;
			st->sensor[i].dur =
				st->eng_info[st->sensor[i].engine_base].dur *
						div[i];
			if (div[i] != st->sensor[i].div) {
				st->sensor[i].div = div[i];
				result = inv_write_2bytes(st,
					st->sensor[i].odr_addr, div[i] - 1);
				if (result)
					return result;
			}
		}
	}

	d_flag = 0;
	for (i = 0; i < SENSOR_ACCURACY_NUM_MAX; i++) {
		if (st->sensor_accuracy[i].on)
			st->cntl2 |= st->sensor_accuracy[i].output;
		d_flag |= st->sensor_accuracy[i].on;
	}
	d_flag |= st->chip_config.activity_eng_on;
	d_flag |= st->chip_config.pick_up_enable;
	if (d_flag)
		st->cntl |= HEADER2_SET;

	if (st->chip_config.step_indicator_on)
		st->cntl |= PED_STEPIND_SET;
	if (st->chip_config.step_detector_on)
		st->cntl |= PED_STEPDET_SET;
	if (st->chip_config.geomag_enable)
		st->cntl2 |= GEOMAG_EN;
	if (st->chip_config.activity_eng_on) {
		st->cntl2 |= ACT_RECOG_SET;
		st->cntl2 |= GYRO_OFF_MASK;
	}
	if (st->chip_config.pick_up_enable)
		st->cntl2 |= FLIP_PICKUP_SET;

	if (!st->chip_config.dmp_event_int_on) {
		result = inv_batchmode_setup(st);
		if (result)
			return result;
	}

	return 0;
}

static int inv_set_div(struct inv_mpu_state *st, int a_d, int g_d)
{
	int result;

	result = inv_set_bank(st, BANK_SEL_2);
	if (result)
		return result;

	if (local.reg_gyro_smplrt != g_d) {
		result = inv_plat_single_write(st, REG_GYRO_SMPLRT_DIV, g_d);
		if (result)
			return result;
		local.reg_gyro_smplrt = g_d;
	}
	result = inv_plat_single_write(st, REG_ACCEL_SMPLRT_DIV_2, a_d);
	if (result)
		return result;
	result = inv_set_bank(st, BANK_SEL_0);

	return result;
}

static int inv_set_rate(struct inv_mpu_state *st)
{
	int g_d, a_d, result;

	if (st->chip_config.dmp_on) {
		result = inv_setup_dmp_rate(st);
		if (result)
			return result;
	} else {
		st->eng_info[ENGINE_GYRO].running_rate =
						st->sensor[SENSOR_GYRO].rate;
		st->eng_info[ENGINE_ACCEL].running_rate =
						st->sensor[SENSOR_ACCEL].rate;
		st->eng_info[ENGINE_GYRO].divider =
					st->eng_info[ENGINE_GYRO].orig_rate /
					st->sensor[SENSOR_GYRO].rate;
		st->eng_info[ENGINE_ACCEL].divider =
					st->eng_info[ENGINE_ACCEL].orig_rate /
					st->sensor[SENSOR_ACCEL].rate;
	}

	g_d = st->eng_info[ENGINE_GYRO].divider - 1;
	a_d = st->eng_info[ENGINE_ACCEL].divider - 1;
	result = inv_set_div(st, a_d, g_d);

	return result;
}

static int inv_set_fifo_size(struct inv_mpu_state *st)
{
	int result;
	u8 cfg, ind;

	if (st->chip_config.dmp_on) {

		/* use one FIFO in DMP mode */
		cfg = BIT_SINGLE_FIFO_CFG;
	} else {
		ind = 0;
		if (st->sensor[SENSOR_GYRO].on)
			ind++;
		if (st->sensor[SENSOR_ACCEL].on)
			ind++;
		if (st->sensor[SENSOR_COMPASS].on)
			ind++;
		if (ind > 1)
			cfg = BIT_MULTI_FIFO_CFG;
		else
			cfg = BIT_SINGLE_FIFO_CFG;
	}
	result = inv_plat_single_write(st, REG_FIFO_CFG, cfg);
	if (result)
		return result;

	return 0;
}

/*
 *  inv_set_fake_secondary() - set fake secondary I2C such that
 *                           I2C data in the same position.
 */
static int inv_set_fake_secondary(struct inv_mpu_state *st)
{
	int r;
	u8 bytes, ind;

	/* may need to saturate the master I2C counter like Scorpion did */
	r = inv_set_bank(st, BANK_SEL_3);
	if (r)
		return r;
	if (st->sec_set.delay_enable != local.reg_delay_enable) {
		r = inv_plat_single_write(st, REG_I2C_MST_DELAY_CTRL,
						st->sec_set.delay_enable);
		if (r)
			return r;
		local.reg_delay_enable = st->sec_set.delay_enable;
	}
	if (st->sec_set.delay_time != local.reg_delay_time) {
		r = inv_plat_single_write(st, REG_I2C_SLV4_CTRL,
					st->sec_set.delay_time);
		if (r)
			return r;
		local.reg_delay_time = st->sec_set.delay_time;
	}
	/* odr config is changed during slave setup */
	r = inv_plat_single_write(st, REG_I2C_MST_ODR_CONFIG,
						st->sec_set.odr_config);
	if (r)
		return r;
	r = inv_set_bank(st, BANK_SEL_0);
	if (r)
		return r;

	/*111, 110 */
	if (st->chip_config.compass_enable && st->sensor[SENSOR_ALS].on)
		return 0;
	/* 100 */
	if (st->chip_config.compass_enable &&
		(!st->sensor[SENSOR_ALS].on) &&
		(!st->chip_config.pressure_enable))
		return 0;
	r = inv_set_bank(st, BANK_SEL_3);
	if (r)
		return r;

	if (st->chip_config.pressure_enable) {
		/* 001 */
		if ((!st->chip_config.compass_enable) &&
					(!st->sensor[SENSOR_ALS].on)) {
			r = inv_read_secondary(st, 0,
						st->plat_data.aux_i2c_addr,
						BMP280_DIG_T1_LSB_REG,
						DATA_AKM_99_BYTES_DMP);
			if (r)
				return r;
			r = inv_read_secondary(st, 2,
						st->plat_data.aux_i2c_addr,
						BMP280_DIG_T1_LSB_REG,
						DATA_ALS_BYTES_DMP);
			r = inv_set_bank(st, BANK_SEL_0);

			return r;
		}

		if (st->chip_config.compass_enable &&
					(!st->sensor[SENSOR_ALS].on)) {
			/* 101 */
			ind = 2;
			if ((COMPASS_ID_AK09911 ==
						st->plat_data.sec_slave_id) ||
					(COMPASS_ID_AK09912 ==
						st->plat_data.sec_slave_id))
				bytes = DATA_ALS_BYTES_DMP;
			else
				bytes = DATA_ALS_BYTES_DMP +
					DATA_AKM_99_BYTES_DMP -
					DATA_AKM_89_BYTES_DMP;
		} else { /* 011 */
			ind = 0;
			bytes = DATA_AKM_99_BYTES_DMP;
		}
		r = inv_read_secondary(st, ind, st->plat_data.aux_i2c_addr,
						BMP280_DIG_T1_LSB_REG, bytes);
	} else { /* compass disabled; als enabled, pressure disabled 010 */
		r = inv_read_secondary(st, 0, st->plat_data.read_only_i2c_addr,
				APDS9900_AILTL_REG, DATA_AKM_99_BYTES_DMP);
	}
	if (r)
		return r;
	r = inv_set_bank(st, BANK_SEL_0);

	return r;
}

static int inv_set_ICM20628_secondary(struct inv_mpu_state *st)
{
	int rate, compass_rate, pressure_rate, als_rate, min_rate, base;
	int mst_odr_config, d, delay;

	if (st->chip_config.compass_enable)
		compass_rate = st->chip_config.compass_rate;
	else
		compass_rate = 0;
	if (st->chip_config.pressure_enable)
		pressure_rate = st->sensor[SENSOR_PRESSURE].rate;
	else
		pressure_rate = 0;
	if (st->sensor[SENSOR_ALS].on)
		als_rate = st->sensor[SENSOR_ALS].rate;
	else
		als_rate = 0;
	if (compass_rate)
		rate = compass_rate;
	else
		rate = max(pressure_rate, als_rate);
	mst_odr_config = 0;
	min_rate = BASE_SAMPLE_RATE;
	while (rate < min_rate) {
		mst_odr_config++;
		min_rate >>= 1;
	}
	base = BASE_SAMPLE_RATE / (1 << mst_odr_config);
	if (base < rate) {
		mst_odr_config--;
		base = BASE_SAMPLE_RATE / (1 << mst_odr_config);
	}
	if (mst_odr_config < MIN_MST_ODR_CONFIG)
		mst_odr_config = MIN_MST_ODR_CONFIG;
	if (compass_rate) {
		if (mst_odr_config > MAX_MST_ODR_CONFIG)
			mst_odr_config = MAX_MST_ODR_CONFIG;
	} else {
		if (mst_odr_config > MAX_MST_NON_COMPASS_ODR_CONFIG)
			mst_odr_config = MAX_MST_NON_COMPASS_ODR_CONFIG;
	}

	base = BASE_SAMPLE_RATE / (1 << mst_odr_config);
	if ((!st->chip_config.gyro_enable) &&
						(!st->chip_config.accel_enable)) {
		st->eng_info[ENGINE_I2C].running_rate = base;
		st->eng_info[ENGINE_I2C].divider = (1 << mst_odr_config);
	}
	inv_calc_engine_dur(&st->eng_info[ENGINE_I2C]);

	d = 0;
	if (d > 0)
		d -= 1;
	if (d > MAX_5_BIT_VALUE)
		d = MAX_5_BIT_VALUE;

	/* I2C_MST_DLY is set to slow down secondary I2C */
	if (d)
		delay = 0x1F;
	else
		delay = 0;

	st->sec_set.delay_enable = delay;
	st->sec_set.delay_time = d;
	st->sec_set.odr_config = mst_odr_config;

	return 0;
}

static int inv_set_master_delay(struct inv_mpu_state *st)
{

	if (!st->chip_config.slave_enable)
		return 0;
	inv_set_ICM20628_secondary(st);

	return 0;
}

static void inv_enable_accel_cal_V3(struct inv_mpu_state *st, u8 enable)
{
	if (enable)
		st->motion_event_cntl |= (ACCEL_CAL_EN << 8);

	return;
}

static void inv_enable_gyro_cal_V3(struct inv_mpu_state *st, u8 enable)
{
	if (enable)
		st->motion_event_cntl |= (GYRO_CAL_EN << 8);

	return;
}

static void inv_enable_compass_cal_V3(struct inv_mpu_state *st, u8 enable)
{
	if (enable)
		st->motion_event_cntl |= COMPASS_CAL_EN;

	return;
}

static int inv_set_wom(struct inv_mpu_state *st)
{
	int result;
	u8 d[4] = {0, 0, 0, 0};

	if (st->chip_config.wom_on)
		d[3] = 1;

	if (local.wom_on != st->chip_config.wom_on) {
		result = mem_w(WOM_ENABLE, ARRAY_SIZE(d), d);
		if (result)
			return result;
		local.wom_on = st->chip_config.wom_on;
	}

	inv_write_2bytes(st, 0x8a, st->chip_config.gyro_enable |
						(st->chip_config.accel_enable << 1) |
						(st->chip_config.slave_enable << 3));

	return 0;
}

static void inv_setup_events(struct inv_mpu_state *st)
{
	if (st->ped.engine_on)
		st->motion_event_cntl |= (PEDOMETER_EN << 8);
	if (st->smd.on)
		st->motion_event_cntl |= (SMD_EN << 8);
	if (st->ped.int_on)
		st->motion_event_cntl |= (PEDOMETER_INT_EN << 8);
	if (st->chip_config.pick_up_enable)
		st->motion_event_cntl |= (FLIP_PICKUP_EN << 8);

}
static int inv_setup_sensor_interrupt(struct inv_mpu_state *st)
{
	int i, ind, rate;
	u16 cntl;

	cntl = 0;
	ind = -1;
	rate = 0;

	if (st->batch.on) {
		for (i = 0; i < SENSOR_NUM_MAX; i++) {
			if (st->sensor[i].on)
				cntl |= st->sensor[i].output;
		}
	}
	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (st->sensor[i].on) {
			if (st->sensor[i].rate > rate) {
				ind = i;
				rate = st->sensor[i].rate;
			}
		}
	}

	if (ind != -1)
		cntl |= st->sensor[ind].output;
	if (st->chip_config.step_detector_on)
		cntl |= PED_STEPDET_SET;
	if (st->chip_config.activity_eng_on)
		cntl |= HEADER2_SET;
	if (st->chip_config.pick_up_enable)
		cntl |= HEADER2_SET;

	return inv_write_2bytes(st, DATA_INTR_CTL, cntl);
}

static int inv_setup_dmp(struct inv_mpu_state *st)
{
	int result, i, tmp, min_diff, ind;

	result = inv_setup_sensor_interrupt(st);
	if (result)
		return result;

	i = 0;
	ind = 0;
	min_diff = accel_cal_para[0].freq;
	while (i < ARRAY_SIZE(accel_cal_para)) {
		tmp = abs(accel_cal_para[i].freq -
				st->eng_info[ENGINE_ACCEL].running_rate);
		if (tmp < min_diff) {
			min_diff = tmp;
			ind = i;
		}
		i++;
	}
	i = ind;
	if (i != local.accel_cal_ind) {
		result = write_be32_to_mem(st,
				accel_cal_para[i].bac_rate, BAC_RATE);
		if (result)
			return result;
		result = write_be32_to_mem(st,
				accel_cal_para[i].bac_rate, FP_RATE);
		if (result)
			return result;

		if (ICM10340 == st->chip_type) {
			result = inv_write_2bytes(st,
					ACCEL_CAL_RATE, accel_cal_para[i].rate);
			if (result)
				return result;
			result = write_be32_to_mem(st,
				accel_cal_para[i].alpha, ACCEL_ALPHA_VAR);
			if (result)
				return result;
			result = write_be32_to_mem(st,
					accel_cal_para[i].a, ACCEL_A_VAR);
			if (result)
				return result;
			result = write_be32_to_mem(st,
				accel_cal_para[i].gain, ACCEL_ONLY_GAIN);
			if (result)
				return result;
			local.accel_cal_ind = i;
		}
	}

	if (st->chip_config.activity_eng_on != local.activity_eng_on) {
		result = write_be32_to_mem(st, 0, BAC_STATE);
		if (result)
			return result;

		result = write_be32_to_mem(st, 0, BAC_STATE_PREV);
		if (result)
			return result;
		result = write_be32_to_mem(st, 0, BAC_ACT_ON);
		if (result)
			return result;
		result = write_be32_to_mem(st, 0, BAC_ACT_OFF);
		if (result)
			return result;
		result = write_be32_to_mem(st, 0, BAC_STILL_S_F);
		if (result)
			return result;
		result = write_be32_to_mem(st, 0, BAC_RUN_S_F);
		if (result)
			return result;
		result = write_be32_to_mem(st, 0, BAC_DRIVE_S_F);
		if (result)
			return result;
		result = write_be32_to_mem(st, 0, BAC_WALK_S_F);
		if (result)
			return result;
		result = write_be32_to_mem(st, 0, BAC_SMD_S_F);
		if (result)
			return result;
		result = write_be32_to_mem(st, 0, BAC_BIKE_S_F);
		if (result)
			return result;
		result = write_be32_to_mem(st, 0, BAC_E1_SHORT);
		if (result)
			return result;
		result = write_be32_to_mem(st, 0, BAC_E2_SHORT);
		if (result)
			return result;
		result = write_be32_to_mem(st, 0, BAC_E3_SHORT);
		if (result)
			return result;
		result = write_be32_to_mem(st, 0, BAC_E4_SHORT);
		if (result)
			return result;
		result = write_be32_to_mem(st, 0, BAC_VAR_RUN);
		if (result)
			return result;
		result = write_be32_to_mem(st, 1, BAC_TILT_INIT);
		if (result)
			return result;

		local.activity_eng_on = st->chip_config.activity_eng_on;
	}
	inv_enable_accel_cal_V3(st, st->accel_cal_enable);
	inv_enable_gyro_cal_V3(st, st->gyro_cal_enable);
	inv_enable_compass_cal_V3(st, st->calib_compass_on);
	if (st->ped.engine_on) {
		result = write_be32_to_mem(st, 0, DMPRATE_CNTR);
		if (result)
			return result;
	}

	inv_setup_events(st);

	result = inv_set_wom(st);
	if (result)
		return result;

	result = inv_write_2bytes(st, DATA_OUT_CTL1, st->cntl);
	if (result)
		return result;
	result = inv_write_2bytes(st, DATA_OUT_CTL2, st->cntl2);
	if (result)
		return result;
	result = inv_write_2bytes(st, MOTION_EVENT_CTL, st->motion_event_cntl);

	if (st->chip_config.gyro_enable) {
		if (st->eng_info[ENGINE_GYRO].running_rate ==
				MPU_DEFAULT_DMP_FREQ)
			result = write_be32_to_mem(st, st->gyro_sf, GYRO_SF);
		else
			result = write_be32_to_mem(st,
						st->gyro_sf << 1, GYRO_SF);
	}

	return result;
}
static int inv_get_accel_gyro_rate(int compass_rate)
{
	int i;

	i = 0;
	while((i < ARRAY_SIZE(accel_gyro_rate)) &&
			compass_rate > accel_gyro_rate[i])
		i++;

	return accel_gyro_rate[i];
}
static int inv_determine_engine(struct inv_mpu_state *st)
{
	int i;
	bool a_en, g_en, c_en, p_en, data_on, ped_on;
	int compass_rate, accel_rate, gyro_rate;
	u32 base_time;

	a_en = false;
	g_en = false;
	c_en = false;
	p_en = false;
	ped_on = false;
	data_on = false;
	compass_rate = 0;

	for (i = 0; i < SENSOR_NUM_MAX; i++) {
		if (st->sensor[i].on) {
			data_on = true;
			a_en |= st->sensor[i].a_en;
			g_en |= st->sensor[i].g_en;
			c_en |= st->sensor[i].c_en;
			p_en |= st->sensor[i].p_en;
			if (st->sensor[i].c_en)
				compass_rate =
					max(compass_rate, st->sensor[i].rate);
			if (st->sensor[i].p_en)
				compass_rate =
					max(compass_rate, st->sensor[i].rate);
		}
	}
	st->chip_config.activity_eng_on = (st->chip_config.activity_on |
					st->chip_config.tilt_enable);
	if (st->step_detector_l_on ||
		st->step_detector_wake_l_on ||
		(st->ped.on && st->batch.timeout))
		st->chip_config.step_detector_on = true;
	else
		st->chip_config.step_detector_on = false;
	if (st->chip_config.step_detector_on ||
					st->chip_config.step_indicator_on ||
					st->chip_config.activity_eng_on) {
		ped_on = true;
		data_on = true;
	}
	if (st->smd.on)
		ped_on = true;
	if(st->ped.on && (!st->batch.timeout))
		st->ped.int_on = 1;
	else
		st->ped.int_on = 0;

	if (st->ped.on || ped_on)
		st->ped.engine_on = true;
	else
		st->ped.engine_on = false;
	if (st->ped.engine_on)
		a_en = true;

	if (st->chip_config.pick_up_enable)
		a_en = true;
	if (data_on)
		st->chip_config.dmp_event_int_on = 0;
	else
		st->chip_config.dmp_event_int_on = 1;

	if (st->chip_config.dmp_event_int_on)
		st->chip_config.wom_on = 1;
	else
		st->chip_config.wom_on = 0;

	if (compass_rate > MAX_COMPASS_RATE)
		compass_rate = MAX_COMPASS_RATE;
	if (compass_rate < MIN_COMPASS_RATE)
		compass_rate = MIN_COMPASS_RATE;
	st->chip_config.compass_rate = compass_rate;
	if (st->sensor[SENSOR_SIXQ].on || st->sensor[SENSOR_PEDQ].on) {
		/* if 6 Q or 9 Q is on, set gyro/accel to default rate */
		accel_rate = MPU_DEFAULT_DMP_FREQ;
		gyro_rate  = MPU_DEFAULT_DMP_FREQ;
	} else {
		if (st->ped.engine_on) {
			if (st->sensor[SENSOR_ACCEL].on) {
				if (st->sensor[SENSOR_ACCEL].rate < PEDOMETER_FREQ) {
					accel_rate = PEDOMETER_FREQ;
				} else {
					accel_rate = st->sensor[SENSOR_ACCEL].rate;
				}
			} else {
				accel_rate = PEDOMETER_FREQ;
			}
		} else if (st->chip_config.pick_up_enable) {
			if (st->sensor[SENSOR_ACCEL].on)
				accel_rate = max(PEDOMETER_FREQ,
						st->sensor[SENSOR_ACCEL].rate);
			else
				accel_rate = PEDOMETER_FREQ;
		} else {
			accel_rate = max(st->sensor[SENSOR_ACCEL].rate, MPU_INIT_SENSOR_RATE);
		}
		gyro_rate = MPU_INIT_SENSOR_RATE;
		if (st->sensor[SENSOR_GYRO].on)
			gyro_rate = max(gyro_rate,
						st->sensor[SENSOR_GYRO].rate);
		if (st->sensor[SENSOR_CALIB_GYRO].on)
			gyro_rate = max(gyro_rate,
					st->sensor[SENSOR_CALIB_GYRO].rate);
	}
	if (g_en) {
		if (a_en)
			gyro_rate = max(gyro_rate, accel_rate);
		if (c_en || p_en) {
			if (gyro_rate < compass_rate)
				gyro_rate = inv_get_accel_gyro_rate(compass_rate);
		}
		accel_rate = gyro_rate;
		compass_rate = gyro_rate;
	} else if (a_en) {
		if (c_en || p_en) {
			if (accel_rate < compass_rate)
				accel_rate = inv_get_accel_gyro_rate(compass_rate);
		}
		compass_rate = accel_rate;
		gyro_rate = accel_rate;
	}

	st->eng_info[ENGINE_GYRO].running_rate = gyro_rate;
	st->eng_info[ENGINE_ACCEL].running_rate = accel_rate;
	st->eng_info[ENGINE_PRESSURE].running_rate = MPU_DEFAULT_DMP_FREQ;
	st->eng_info[ENGINE_I2C].running_rate = compass_rate;
	/* engine divider for pressure and compass is set later */
	st->eng_info[ENGINE_GYRO].divider =
				(BASE_SAMPLE_RATE / MPU_DEFAULT_DMP_FREQ) *
				(MPU_DEFAULT_DMP_FREQ /
				st->eng_info[ENGINE_GYRO].running_rate);
	st->eng_info[ENGINE_ACCEL].divider =
				(BASE_SAMPLE_RATE / MPU_DEFAULT_DMP_FREQ) *
				(MPU_DEFAULT_DMP_FREQ /
				st->eng_info[ENGINE_ACCEL].running_rate);
	st->eng_info[ENGINE_I2C].divider =
				(BASE_SAMPLE_RATE / MPU_DEFAULT_DMP_FREQ) *
				(MPU_DEFAULT_DMP_FREQ /
				st->eng_info[ENGINE_ACCEL].running_rate);

	base_time = NSEC_PER_SEC;

	st->eng_info[ENGINE_GYRO].base_time = base_time;
	st->eng_info[ENGINE_ACCEL].base_time = base_time;
	st->eng_info[ENGINE_I2C].base_time = base_time;

	inv_calc_engine_dur(&st->eng_info[ENGINE_GYRO]);
	inv_calc_engine_dur(&st->eng_info[ENGINE_ACCEL]);

	if (st->debug_determine_engine_on)
		return 0;

	st->chip_config.gyro_enable = g_en;
	st->chip_config.accel_enable = a_en;
	st->chip_config.compass_enable = c_en;
	st->calib_compass_on = c_en;
	st->chip_config.pressure_enable = p_en;
	st->chip_config.dmp_on = 1;
	st->chip_config.geomag_enable = 0;

	if (c_en || st->sensor[SENSOR_ALS].on || p_en)
		st->chip_config.slave_enable = 1;
	else
		st->chip_config.slave_enable = 0;
	if (st->sensor[SENSOR_ALS].on)
		st->chip_config.als_enable = 1;
	else
		st->chip_config.als_enable = 0;

	st->gyro_cal_enable = 0;
	if ((st->chip_type == ICM10340) && a_en)
		st->accel_cal_enable = 1;
	else
		st->accel_cal_enable = 0;

	/* setting up accuracy output */
	if (st->sensor[SENSOR_ACCEL].on && (st->chip_type == ICM10340))
		st->sensor_accuracy[SENSOR_ACCEL_ACCURACY].on = true;
	else
		st->sensor_accuracy[SENSOR_ACCEL_ACCURACY].on = false;

	if (st->sensor[SENSOR_CALIB_GYRO].on)
		st->sensor_accuracy[SENSOR_GYRO_ACCURACY].on  = true;
	else
		st->sensor_accuracy[SENSOR_GYRO_ACCURACY].on = false;

	st->cntl = 0;
	st->cntl2 = 0;
	st->motion_event_cntl = 0;

	inv_set_master_delay(st);

	return 0;
}

/*
 *  set_inv_enable() - enable function.
 */
int set_inv_enable(struct iio_dev *indio_dev)
{
	int result;
	struct inv_mpu_state *st = iio_priv(indio_dev);

	long long ts1, ts2;

	ts1 = get_time_ns();
	result = inv_switch_power_in_lp(st, true);
	if (result)
		return result;

	result = inv_stop_dmp(st);
	if (result)
		return result;
	inv_determine_engine(st);
	if ((!st->chip_config.gyro_enable) &&
		(!st->chip_config.accel_enable) &&
		(!st->chip_config.slave_enable) &&
		(!st->chip_config.pressure_enable)) {
		inv_set_power(st, false);
		ts2 = get_time_ns();
		pr_info("enable sleep takes=%d ms\n", (int)((ts2-ts1) >> 20));

		return 0;
	}

	result = inv_set_rate(st);
	if (result) {
		pr_err("inv_set_rate error\n");
		return result;
	}
	if (st->chip_config.dmp_on) {
		result = inv_setup_dmp(st);
		if (result) {
			pr_err("setup dmp error\n");
			return result;
		}
	}
	result = inv_turn_on_engine(st);
	if (result) {
		pr_err("inv_turn_on_engine error\n");
		return result;
	}
	result = inv_set_fifo_size(st);
	if (result) {
		pr_err("inv_set_fifo_size error\n");
		return result;
	}
	result = inv_set_fake_secondary(st);
	if (result)
		return result;

	result = inv_reset_fifo(st, false);
	if (result)
		return result;
	result = inv_switch_power_in_lp(st, false);

	ts2 = get_time_ns();
	//pr_info("enable takes=%d ms\n", (int)((ts2-ts1) >> 20));
	//pr_info("cntl=%x, cntl2=%x, motion=%x\n",
		//		st->cntl, st->cntl2, st->motion_event_cntl);

	return result;
}
