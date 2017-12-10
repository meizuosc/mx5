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

s64 get_time_ns(void)
{
	struct timespec ts;

	ktime_get_ts(&ts);

	return timespec_to_ns(&ts);
}

int inv_q30_mult(int a, int b)
{
#define DMP_MULTI_SHIFT                 30
	u64 temp;
	int result;

	temp = (u64)a * b;
	result = (int)(temp >> DMP_MULTI_SHIFT);

	return result;
}

/* inv_read_secondary(): set secondary registers for reading.
			The chip must be set as bank 3 before calling.
 */
int inv_read_secondary(struct inv_mpu_state *st, int ind, int addr,
							int reg, int len)
{
	int result;

	result = inv_plat_single_write(st, st->slv_reg[ind].addr,
						INV_MPU_BIT_I2C_READ | addr);
	if (result)
		return result;
	result = inv_plat_single_write(st, st->slv_reg[ind].reg, reg);
	if (result)
		return result;
	result = inv_plat_single_write(st, st->slv_reg[ind].ctrl,
						INV_MPU_BIT_SLV_EN | len);

	return result;
}

int inv_execute_read_secondary(struct inv_mpu_state *st, int ind, int addr,
						int reg, int len, u8 *d)
{
	int result;

	inv_set_bank(st, BANK_SEL_3);
	result = inv_read_secondary(st, ind, addr, reg, len);
	if (result)
		return result;
	inv_set_bank(st, BANK_SEL_0);
	result = inv_plat_single_write(st, REG_USER_CTRL, st->i2c_dis |
							BIT_I2C_MST_EN);
	msleep(SECONDARY_INIT_WAIT);
	result = inv_plat_single_write(st, REG_USER_CTRL, st->i2c_dis);
	if (result)
		return result;
	result = inv_plat_read(st, REG_EXT_SLV_SENS_DATA_00, len, d);

	return result;
}

/* inv_write_secondary(): set secondary registers for writing.
			The chip must be set as bank 3 before calling.
 */
int inv_write_secondary(struct inv_mpu_state *st, int ind, int addr,
							int reg, int v)
{
	int result;

	result = inv_plat_single_write(st, st->slv_reg[ind].addr, addr);
	if (result)
		return result;
	result = inv_plat_single_write(st, st->slv_reg[ind].reg, reg);
	if (result)
		return result;
	result = inv_plat_single_write(st, st->slv_reg[ind].ctrl,
						INV_MPU_BIT_SLV_EN | 1);

	result = inv_plat_single_write(st, st->slv_reg[ind].d0, v);

	return result;
}

int inv_execute_write_secondary(struct inv_mpu_state *st, int ind, int addr,
								int reg, int v)
{
	int result;

	inv_set_bank(st, BANK_SEL_3);
	result = inv_write_secondary(st, ind, addr, reg, v);
	if (result)
		return result;
	inv_set_bank(st, BANK_SEL_0);
	result = inv_plat_single_write(st, REG_USER_CTRL, st->i2c_dis |
								BIT_I2C_MST_EN);
	msleep(SECONDARY_INIT_WAIT);
	result = inv_plat_single_write(st, REG_USER_CTRL, st->i2c_dis);

	return result;
}
int inv_set_power(struct inv_mpu_state *st, bool power_on)
{
	u8 d;
	int r;

	if ((!power_on) == st->chip_config.is_asleep)
		return 0;

	d = BIT_CLK_PLL;
	if (!power_on)
		d |= BIT_SLEEP;

	r = inv_plat_single_write(st, REG_PWR_MGMT_1, d);
	if (r)
		return r;

	if (power_on)
		usleep_range(REG_UP_TIME_USEC, REG_UP_TIME_USEC);

	st->chip_config.is_asleep = !power_on;

	return 0;
}

int inv_stop_dmp(struct inv_mpu_state *st)
{
	return inv_plat_single_write(st, REG_USER_CTRL, st->i2c_dis);
}

static int inv_lp_en_off_mode(struct inv_mpu_state *st, bool on)
{
	int r;

	if (!st->chip_config.is_asleep)
		return 0;

	r = inv_plat_single_write(st, REG_PWR_MGMT_1, BIT_CLK_PLL);
	st->chip_config.is_asleep = 0;

	return r;
}

static int inv_lp_en_on_mode(struct inv_mpu_state *st, bool on)
{
	int r;
	u8 w;

	if ((!st->chip_config.is_asleep)  &&
					((!on) == st->chip_config.lp_en_set))
			return 0;

	w = BIT_CLK_PLL;
	if (!on)
		w |= BIT_LP_EN;
	r = inv_plat_single_write(st, REG_PWR_MGMT_1, w);
	st->chip_config.is_asleep = 0;
	st->chip_config.lp_en_set = (!on);

	return r;
}

int inv_switch_power_in_lp(struct inv_mpu_state *st, bool on)
{
	int r;

	if (st->chip_config.lp_en_mode_off)
		r = inv_lp_en_off_mode(st, on);
	else
		r = inv_lp_en_on_mode(st, on);

	return r;
}

int inv_set_bank(struct inv_mpu_state *st, u8 bank)
{
	int r;

	r = inv_plat_single_write(st, REG_BANK_SEL, bank);

	return r;
}

int write_be32_to_mem(struct inv_mpu_state *st, u32 data, int addr)
{
	cpu_to_be32s(&data);
	return mem_w(addr, sizeof(data), (u8 *)&data);
}

int read_be32_from_mem(struct inv_mpu_state *st, u32 *o, int addr)
{
	int result;
	u32 d;

	result = mem_r(addr, 4, (u8 *)&d);
	*o = be32_to_cpup((__be32 *)(&d));

	return result;
}

u32 inv_get_cntr_diff(u32 curr_counter, u32 prev)
{
	u32 diff;

	if (curr_counter > prev)
		diff = curr_counter - prev;
	else
		diff = 0xffffffff - prev + curr_counter + 1;

	return diff;
}

int inv_write_2bytes(struct inv_mpu_state *st, int addr, int data)
{
	u8 d[2];

	if (data < 0 || data > USHRT_MAX)
		return -EINVAL;

	d[0] = (u8)((data >> 8) & 0xff);
	d[1] = (u8)(data & 0xff);

	return mem_w(addr, ARRAY_SIZE(d), d);
}

/**
 *  inv_write_cntl() - Write control word to designated address.
 *  @st:	Device driver instance.
 *  @wd:        control word.
 *  @en:	enable/disable.
 *  @cntl:	control address to be written.
 */
int inv_write_cntl(struct inv_mpu_state *st, u16 wd, bool en, int cntl)
{
	int result;
	u8 reg[2], d_out[2];

	result = mem_r(cntl, 2, d_out);
	if (result)
		return result;
	reg[0] = ((wd >> 8) & 0xff);
	reg[1] = (wd & 0xff);
	if (!en) {
		d_out[0] &= ~reg[0];
		d_out[1] &= ~reg[1];
	} else {
		d_out[0] |= reg[0];
		d_out[1] |= reg[1];
	}
	result = mem_w(cntl, 2, d_out);

	return result;
}
