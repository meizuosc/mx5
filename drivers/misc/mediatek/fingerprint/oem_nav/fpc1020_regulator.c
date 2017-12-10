/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#define DEBUG

//#include <linux/regulator/consumer.h>

#ifndef CONFIG_OF
#include <linux/spi/fpc1020.h>
#include <linux/spi/fpc1020_common.h>
#include <linux/spi/fpc1020_regulator.h>
#else
#include <linux/of.h>
#include "fpc1020.h"
#include "fpc1020_common.h"
#include "fpc1020_regulator.h"
#endif


#ifdef CONFIG_MZ_MA01
#include <mach/mt_pm_ldo.h>

#include <mach/mt_gpio.h>

static void fpc1020_hw_power(bool bonoff)
{
	printk("%s:set VDDA %s",__func__,bonoff?"on":"off");
	
	if(bonoff)
		mt_set_gpio_out(GPIO_FPS_EN_PIN, GPIO_OUT_ONE);
	else
		mt_set_gpio_out(GPIO_FPS_EN_PIN, GPIO_OUT_ZERO);

}
/* -------------------------------------------------------------------- */
int fpc1020_regulator_configure(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	mt_set_gpio_mode(GPIO_FPS_EN_PIN, GPIO_FPS_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_FPS_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(GPIO_FPS_EN_PIN, GPIO_PULL_DISABLE);

	//fpc1020->vdd_tx = MT6331_POWER_LDO_VIBR;

	return error;
}


/* -------------------------------------------------------------------- */
int fpc1020_regulator_release(fpc1020_data_t *fpc1020)
{

	fpc1020->power_enabled = false;
	fpc1020_hw_power(false);

	return 0;
}


/* -------------------------------------------------------------------- */
int fpc1020_regulator_set(fpc1020_data_t *fpc1020, bool enable)
{
	int error = 0;
	
	dev_dbg(&fpc1020->spi->dev, "%s %s\n", __func__,enable?"on":"off");

	if( fpc1020->power_enabled == enable)
		return error;

	fpc1020_hw_power(enable);
	
	fpc1020->power_enabled = enable;

	return error;
}

#endif

#ifdef CONFIG_MZ_M85
#include <mach/mt_pm_ldo.h>

static void fpc1020_hw_power(bool bonoff)
{
	printk("%s:set VDDA %s",__func__,bonoff?"on":"off");
	if(bonoff)
		hwPowerOn(MT6331_POWER_LDO_VIBR, VOL_1800, "fingerprint");
	else
		hwPowerDown(MT6331_POWER_LDO_VIBR, "fingerprint");

}
/* -------------------------------------------------------------------- */
int fpc1020_regulator_configure(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	//fpc1020->vdd_tx = MT6331_POWER_LDO_VIBR;

	return error;
}


/* -------------------------------------------------------------------- */
int fpc1020_regulator_release(fpc1020_data_t *fpc1020)
{

	fpc1020_regulator_set(fpc1020, false);

	return 0;
}


/* -------------------------------------------------------------------- */
int fpc1020_regulator_set(fpc1020_data_t *fpc1020, bool enable)
{
	int error = 0;
	
	dev_dbg(&fpc1020->spi->dev, "%s %s\n", __func__,enable?"on":"off");

	if( fpc1020->power_enabled == enable)
		return error;		

	fpc1020_hw_power(enable);
	
	fpc1020->power_enabled = enable;

	return error;
}


#endif

#ifdef CONFIG_MZ_MXX

/* -------------------------------------------------------------------- */
int fpc1020_regulator_configure(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020->vdd_tx = regulator_get(&fpc1020->spi->dev, "vdd_tx");
	if (IS_ERR(fpc1020->vdd_tx)) {
		error = PTR_ERR(fpc1020->vdd_tx);
		dev_err(&fpc1020->spi->dev,
			"Regulator get failed, vdd_tx, error=%d\n", error);
		goto supply_err;
	}

	if (regulator_count_voltages(fpc1020->vdd_tx) > 0) {
		error = regulator_set_voltage(fpc1020->vdd_tx,
						SUPPLY_TX_MIN, SUPPLY_TX_MAX);
		if (error) {
			dev_err(&fpc1020->spi->dev,
				"regulator set(tx) failed, error=%d\n", error);
			goto supply_err;
		}
	}

return 0;

supply_err:
	fpc1020_regulator_release(fpc1020);
	return error;
}


/* -------------------------------------------------------------------- */
int fpc1020_regulator_release(fpc1020_data_t *fpc1020)
{
	if (fpc1020->vdd_tx != NULL) {
		regulator_put(fpc1020->vdd_tx);
		fpc1020->vdd_tx = NULL;
	}

	fpc1020->power_enabled = false;

	return 0;
}


/* -------------------------------------------------------------------- */
int fpc1020_regulator_set(fpc1020_data_t *fpc1020, bool enable)
{
	int error = 0;

	if (fpc1020->vdd_tx == NULL) {
		dev_err(&fpc1020->spi->dev,
			"Regulators not set\n");
			return -EINVAL;
	}

	if (enable) {
		dev_dbg(&fpc1020->spi->dev, "%s on\n", __func__);

		/****** 
		  Do we really need to set current?
		  How would it affect the vibrator that shares this regulator?

		regulator_set_optimum_mode(fpc1020->vcc_spi,
					SUPPLY_SPI_REQ_CURRENT);
		*/

		error = (regulator_is_enabled(fpc1020->vdd_tx) == 0) ?
					regulator_enable(fpc1020->vdd_tx) : 0;

		if (error) {
			dev_err(&fpc1020->spi->dev,
				"Regulator vdd_tx enable failed, error=%d\n",
				error);
			goto out_err;
		}
	} else {
		dev_dbg(&fpc1020->spi->dev, "%s off\n", __func__);

		error = (fpc1020->power_enabled &&
			regulator_is_enabled(fpc1020->vdd_tx) > 0) ?
				 regulator_disable(fpc1020->vdd_tx) : 0;

		if (error) {
			dev_err(&fpc1020->spi->dev,
				"Regulator vdd_tx disable failed, error=%d\n",
				error);
			goto out_err;
		}
	}

	fpc1020->power_enabled = enable;

	return 0;

out_err:
	fpc1020_regulator_release(fpc1020);
	return error;
}

#endif



