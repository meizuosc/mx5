/*
 *  stmvl6180.c - Linux kernel module for STM VL6180 FlightSense Time-of-Flight
 *
 *  Copyright (C) 2014 STMicroelectronics Imaging Division.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _STMVL6180_H
#define _STMVL6180_H
/*
 * Defines
 */
#define DRIVER_VERSION		"1.1"

#define STMVL6180_MAGIC 'A'

//Device Registers
#define VL6180_MODEL_ID_REG				0x0000
#define VL6180_MODEL_REV_MAJOR_REG		0x0001
#define VL6180_MODEL_REV_MINOR_REG		0x0002
#define VL6180_MODULE_REV_MAJOR_REG		0x0003
#define VL6180_MODULE_REV_MINOR_REG		0x0004

#define VL6180_REVISION_ID_REG			0x0005
#define VL6180_REVISION_ID_REG_BYTES	1
#define VL6180_DATE_HI_REG				0x0006
#define VL6180_DATE_HI_REG_BYTES		1
#define VL6180_DATE_LO_REG				0x0007
#define VL6180_DATE_LO_REG_BYTES		1
#define VL6180_TIME_REG					0x0008
#define VL6180_TIME_REG_BYTES			2
#define VL6180_CODE_REG					0x000a
#define VL6180_CODE_REG_BYTES			1
#define VL6180_FIRMWARE_REVISION_ID_REG	0x000b
#define VL6180_FIRMWARE_REVISION_ID_REG_BYTES 1
/******************************** VL6180 registers*********************/
#define IDENTIFICATION__MODEL_ID				0x000
#define IDENTIFICATION__REVISION_ID				0x002
#define FIRMWARE__BOOTUP						0x119
#define RESULT__RANGE_STATUS					0x04D
#define GPIO_HV_PAD01__CONFIG					0x132
#define SYSRANGE__MAX_CONVERGENCE_TIME			0x01C
#define SYSRANGE__RANGE_CHECK_ENABLES			0x02D
#define SYSRANGE__MAX_CONVERGENCE_TIME			0x01C
#define SYSRANGE__EARLY_CONVERGENCE_ESTIMATE	0x022
#define SYSTEM__FRESH_OUT_OF_RESET				0x016
#define SYSRANGE__PART_TO_PART_RANGE_OFFSET		0x024
#define SYSRANGE__CROSSTALK_COMPENSATION_RATE	0x01E
#define SYSRANGE__CROSSTALK_VALID_HEIGHT		0x021
#define SYSRANGE__RANGE_IGNORE_VALID_HEIGHT		0x025
#define SYSRANGE__RANGE_IGNORE_THRESHOLD		0x026
#define SYSRANGE__MAX_AMBIENT_LEVEL_MULT		0x02C
#define SYSALS__INTERMEASUREMENT_PERIOD			0x03E
#define SYSRANGE__INTERMEASUREMENT_PERIOD		0x01B
#define SYSRANGE__START							0x018
#define RESULT__RANGE_VAL						0x062
#define RESULT__RANGE_STRAY						0x063
#define RESULT__RANGE_RAW						0x064
#define RESULT__RANGE_RETURN_SIGNAL_COUNT		0x06C
#define RESULT__RANGE_REFERENCE_SIGNAL_COUNT	0x070
#define RESULT__RANGE_RETURN_AMB_COUNT			0x074
#define RESULT__RANGE_REFERENCE_AMB_COUNT		0x078
#define RESULT__RANGE_RETURN_CONV_TIME			0x07C
#define RESULT__RANGE_REFERENCE_CONV_TIME		0x080
#define SYSTEM__INTERRUPT_CLEAR					0x015
#define RESULT__INTERRUPT_STATUS_GPIO			0x04F
#define SYSTEM__MODE_GPIO1						0x011
#define SYSTEM__INTERRUPT_CONFIG_GPIO			0x014
#define RANGE__RANGE_SCALER						0x096
#define SYSRANGE__PART_TO_PART_RANGE_OFFSET		0x024
/******************************** VL6180 registers*********************/
/**
 * range data structure
 */
typedef struct
{
	unsigned int m_range;
	unsigned int m_trueRange_mm;
	unsigned int m_rawRange_mm;
	unsigned int m_rtnRate;
	unsigned int m_refRate;
	unsigned int m_rtnAmbRate;
	unsigned int m_refAmbRate;
	unsigned int m_ConvTime;
	unsigned int m_rtnSignalCount;
	unsigned int m_refSignalCount;
	unsigned int m_rtnAmbientCount;
	unsigned int m_refAmbientCount;
	unsigned int m_rtnConvTime;
	unsigned int m_refConvTime;
	int m_strayLightFactor;
}RangeData;

/*
 *  driver data structs
 */
struct stmvl6180_data {
	struct i2c_client *client;

	unsigned int is_6180; 
	unsigned int enable;
	/* Range Data */
	RangeData rangeData;

	/* Register Data for tool */
	unsigned int register_addr;
	unsigned int register_bytes;
	
	/* Debug */
	unsigned int enableDebug;
};

/******************************** IOCTL definitions******************/
#define VL6180_IOCTL_INIT 		_IOWR(STMVL6180_MAGIC, 10, u32)
#define VL6180_IOCTL_GETDATA 		_IOR(STMVL6180_MAGIC, 11, u32)
#define VL6180_IOCTL_GETDATAS 		_IOR(STMVL6180_MAGIC, 12, RangeData)
/******************************** IOCTL definitions******************/
#endif
