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

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include "../camera/kd_camera_hw.h"
#include "stmvl6180.h"
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#define LENS_I2C_BUSNUM 0
#define E2PROM_WRITE_ID 0xA8
#define DEVICE_WRITE_ID	0x52

//#define DEFAULT_CROSSTALK	 0//4 // Already coded in 9.7 format
#define DEFAULT_CROSSTALK	 0x09//4 // Already coded in 9.7 format

// Filter defines
#define FILTERNBOFSAMPLES		10
#define FILTERSTDDEVSAMPLES		6
#define MINFILTERSTDDEVSAMPLES	3
#define MINFILTERVALIDSTDDEVSAMPLES	4
#define FILTERINVALIDDISTANCE	65535

//distance filter
#define DISTANCE_FILTER

static struct i2c_board_info __initdata kd_lens_dev={ I2C_BOARD_INFO("STMVL6180", 0x29)};

#define STMVL6180_DRVNAME "STMVL6180"

#define STMVL6180DB	printk

static spinlock_t stmvl6180_spinlock;

static struct i2c_client * stmvl6180_i2c_client = NULL;

static dev_t stmvl6180_devno;
static struct cdev * stmvl6180_chardrv = NULL;
static struct class *actuator_class = NULL;

static const struct i2c_device_id stmvl6180_i2c_id[] = {{STMVL6180_DRVNAME,0},{}};

uint32_t MeasurementIndex = 0;
// Distance Filter global variables
uint32_t Default_ZeroVal = 0;
uint32_t Default_VAVGVal = 0;
uint32_t NoDelay_ZeroVal = 0;
uint32_t NoDelay_VAVGVal = 0;
uint32_t Previous_VAVGDiff = 0;
uint16_t LastTrueRange[FILTERNBOFSAMPLES];
uint32_t LastReturnRates[FILTERNBOFSAMPLES];
uint32_t PreviousRangeStdDev = 0;
uint32_t PreviousStdDevLimit = 0;
uint32_t PreviousReturnRateStdDev = 0;
uint16_t StdFilteredReads = 0;
uint32_t m_chipid = 0;
uint16_t LastMeasurements[8] = {0,0,0,0,0,0,0,0};
uint16_t AverageOnXSamples = 4;
uint16_t CurrentIndex = 0;
uint16_t return_rate = 0;

#ifdef DISTANCE_FILTER
void VL6180_InitDistanceFilter(void);
uint16_t VL6180_DistanceFilter(uint16_t m_trueRange_mm, uint16_t m_rawRange_mm, uint32_t m_rtnSignalRate, uint32_t m_rtnAmbientRate, uint16_t errorCode);
uint32_t VL6180_StdDevDamper(uint32_t AmbientRate, uint32_t SignalRate, uint32_t StdDevLimitLowLight, uint32_t StdDevLimitLowLightSNR, uint32_t StdDevLimitHighLight, uint32_t StdDevLimitHighLightSNR);
#endif

extern int camera_flight_poweron(char *mode_name, u8 on);
extern int camera_flight_gpio(u8 on);

extern int laser_data_write_emmc(int *data, int size);
extern int laser_data_read_emmc(int *data, int size);

/*******************************************************************************
* WriteRegI2C
********************************************************************************/
int stmvl6180_i2c_write(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId)
{
    int  i4RetValue = 0;
    int retry = 3;

	spin_lock(&stmvl6180_spinlock);
	stmvl6180_i2c_client->addr = (i2cId >> 1);
	stmvl6180_i2c_client->ext_flag = (stmvl6180_i2c_client->ext_flag)&(~I2C_DMA_FLAG);
	spin_unlock(&stmvl6180_spinlock);

    do {
	i4RetValue = i2c_master_send(stmvl6180_i2c_client, a_pSendData, a_sizeSendData);
	if (i4RetValue != a_sizeSendData) {
	    STMVL6180DB("[stmvl6180] I2C send failed!!, Addr = 0x%x, Data = 0x%x \n", a_pSendData[0], a_pSendData[1]);
	}
	else {
	    break;
	}
	udelay(50);
    } while ((retry--) > 0);

    return 0;
}

/*******************************************************************************
* ReadRegI2C
********************************************************************************/
int stmvl6180_i2c_read(u8 *a_pSendData , u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId)
{
	int  i4RetValue = 0;

	spin_lock(&stmvl6180_spinlock);
	stmvl6180_i2c_client->addr = (i2cId >> 1);
	stmvl6180_i2c_client->ext_flag = (stmvl6180_i2c_client->ext_flag)&(~I2C_DMA_FLAG);
	spin_unlock(&stmvl6180_spinlock);

	i4RetValue = i2c_master_send(stmvl6180_i2c_client, a_pSendData, a_sizeSendData);
	if (i4RetValue != a_sizeSendData) {
		STMVL6180DB("[stmvl6180] I2C send failed!!, Addr = 0x%x\n", a_pSendData[0]);
	        return -1;
	}

	i4RetValue = i2c_master_recv(stmvl6180_i2c_client, (u8 *)a_pRecvData, a_sizeRecvData);
	if (i4RetValue != a_sizeRecvData) {
		STMVL6180DB("[stmvl6180] I2C read failed!! \n");
	        return -1;
	}

	return 0;
}

static int stmvl6180_open(struct inode *a_pstInode, struct file *a_pstFile)
{
	return 0;
}

static int stmvl6180_release(struct inode *a_pstInode, struct file *a_pstFile)
{
	return 0;
}

/* 8 bits i2c write */
int vl6180_i2c_write_8bits(unsigned int addr, unsigned int data)
{
	u8 sendcmd[3] = {(u8)((addr >> 8) & 0xFF), (u8)(addr & 0xFF), (u8)(data & 0xFF)};
	stmvl6180_i2c_write(sendcmd, sizeof(sendcmd), DEVICE_WRITE_ID);
	return 0;
}

/* 8 bits i2c read */
int vl6180_i2c_read_8bits(unsigned int addr, uint16_t *pdata)
{
	u8 recv=0;
	u8 sendcmd[2] = {(u8)((addr >> 8) & 0xFF), (u8)(addr & 0xFF)};
	stmvl6180_i2c_read(sendcmd, sizeof(sendcmd), &recv, 1, DEVICE_WRITE_ID);
	*pdata = (uint16_t)recv;
	return 0;
}

/* 32 bits i2c read */
int vl6180_i2c_read_32bits(unsigned int addr,  unsigned int *pdata)
{
	u8 buf[4];
	u8 sendcmd[2] = {(u8)((addr >> 8) & 0xFF), (u8)(addr & 0xFF)};
	stmvl6180_i2c_read(sendcmd, sizeof(sendcmd), buf, 4, DEVICE_WRITE_ID);
	*pdata = (unsigned int)( (unsigned int)(buf[0] <<24)
				| (unsigned int)((buf[1])<<16)
				| (unsigned int)((buf[2])<<8)
				| (unsigned int)(buf[3]));
	return 0;
} 

#ifdef DISTANCE_FILTER
void VL6180_InitDistanceFilter(void)
{
	int i;

	MeasurementIndex = 0;

	Default_ZeroVal = 0;
	Default_VAVGVal = 0;
	NoDelay_ZeroVal = 0;
	NoDelay_VAVGVal = 0;
	Previous_VAVGDiff = 0;

	StdFilteredReads = 0;
	PreviousRangeStdDev = 0;
	PreviousReturnRateStdDev = 0;

	for (i = 0; i < FILTERNBOFSAMPLES; i++){
		LastTrueRange[i] = FILTERINVALIDDISTANCE;
		LastReturnRates[i] = 0;
	}
}

uint32_t VL6180_StdDevDamper(uint32_t AmbientRate, uint32_t SignalRate, uint32_t StdDevLimitLowLight, uint32_t StdDevLimitLowLightSNR, uint32_t StdDevLimitHighLight, uint32_t StdDevLimitHighLightSNR)
{
	uint32_t newStdDev;
	uint16_t SNR;

	if (AmbientRate > 0)
		SNR = (uint16_t)((100 * SignalRate) / AmbientRate);
	else
		SNR = 9999;

	if (SNR >= StdDevLimitLowLightSNR){
		newStdDev = StdDevLimitLowLight;
	}
	else{
		if (SNR <= StdDevLimitHighLightSNR)
			newStdDev = StdDevLimitHighLight;
		else{
			newStdDev = (uint32_t)(StdDevLimitHighLight + (SNR - StdDevLimitHighLightSNR) * (int)(StdDevLimitLowLight - StdDevLimitHighLight) / (StdDevLimitLowLightSNR - StdDevLimitHighLightSNR));
		}
	}

	return newStdDev;
}

uint16_t VL6180_DistanceFilter(uint16_t m_trueRange_mm, uint16_t m_rawRange_mm, uint32_t m_rtnSignalRate, uint32_t m_rtnAmbientRate, uint16_t errorCode)
{
	uint16_t m_newTrueRange_mm = 0;

	uint16_t i;
	uint16_t bypassFilter = 0;

	uint16_t registerValue;
   // uint16_t dataByte;
	uint32_t register32BitsValue1;
	uint32_t register32BitsValue2;

	uint16_t ValidDistance = 0;
	uint16_t MaxOrInvalidDistance = 0;

	uint16_t WrapAroundFlag = 0;
	uint16_t NoWrapAroundFlag = 0;
	uint16_t NoWrapAroundHighConfidenceFlag = 0;

	uint16_t FlushFilter = 0;
	uint32_t RateChange = 0;

	uint16_t StdDevSamples = 0;
	uint32_t StdDevDistanceSum = 0;
	uint32_t StdDevDistanceMean = 0;
	uint32_t StdDevDistance = 0;
	uint32_t StdDevRateSum = 0;
	uint32_t StdDevRateMean = 0;
	uint32_t StdDevRate = 0;
	uint32_t StdDevLimitWithTargetMove = 0;

	uint32_t VAVGDiff;
	uint32_t IdealVAVGDiff;
	uint32_t MinVAVGDiff;
	uint32_t MaxVAVGDiff;

	// Filter Parameters
	uint16_t WrapAroundLowRawRangeLimit = 20;
	uint32_t WrapAroundLowReturnRateLimit = 800;
	uint16_t WrapAroundLowRawRangeLimit2 = 55;
	uint32_t WrapAroundLowReturnRateLimit2 = 300;

	uint32_t WrapAroundLowReturnRateFilterLimit = 600;
	uint16_t WrapAroundHighRawRangeFilterLimit = 350;
	uint32_t WrapAroundHighReturnRateFilterLimit = 900;

	uint32_t WrapAroundMaximumAmbientRateFilterLimit = 7500;

	// Temporal filter data and flush values
	uint32_t MinReturnRateFilterFlush = 75;
	uint32_t MaxReturnRateChangeFilterFlush = 50;

	// STDDEV values and damper values
	uint32_t StdDevLimit = 300;
	uint32_t StdDevLimitLowLight = 300;
	uint32_t StdDevLimitLowLightSNR = 30; // 0.3
	uint32_t StdDevLimitHighLight = 2500;
	uint32_t StdDevLimitHighLightSNR = 5; //0.05

	uint32_t StdDevHighConfidenceSNRLimit = 8;

	uint32_t StdDevMovingTargetStdDevLimit = 90000;
	uint32_t StdDevMovingTargetReturnRateLimit = 3500;
	uint32_t StdDevMovingTargetStdDevForReturnRateLimit = 5000;

	uint32_t MAX_VAVGDiff = 1800;

	// WrapAroundDetection variables
	uint16_t WrapAroundNoDelayCheckPeriod = 2;

	// Reads Filtering values
	uint16_t StdFilteredReadsIncrement = 2;
	uint16_t StdMaxFilteredReads = 4;

	// End Filter Parameters

	MaxOrInvalidDistance = (uint16_t)(255 * 3);

	// Check if distance is Valid or not
	switch (errorCode){
	case 0x0C:
		m_trueRange_mm = MaxOrInvalidDistance;
		ValidDistance = 0;
		break;
	case 0x0D:
		m_trueRange_mm = MaxOrInvalidDistance;
		ValidDistance = 1;
		break;
	default:
		if (m_rawRange_mm >= MaxOrInvalidDistance){
			ValidDistance = 0;
		}
		else{
			ValidDistance = 1;
		}
		break;
	}
	m_newTrueRange_mm = m_trueRange_mm;

	// Checks on low range data
	if ((m_rawRange_mm < WrapAroundLowRawRangeLimit) && (m_rtnSignalRate < WrapAroundLowReturnRateLimit)){
		//Not Valid distance
		m_newTrueRange_mm = MaxOrInvalidDistance;
		bypassFilter = 1;
	}
	if ((m_rawRange_mm < WrapAroundLowRawRangeLimit2) && (m_rtnSignalRate < WrapAroundLowReturnRateLimit2)){
		//Not Valid distance
		m_newTrueRange_mm = MaxOrInvalidDistance;
		bypassFilter = 1;
	}

	// Checks on Ambient rate level
	if (m_rtnAmbientRate > WrapAroundMaximumAmbientRateFilterLimit){
		// Too high ambient rate
		FlushFilter = 1;
		bypassFilter = 1;
	}
	// Checks on Filter flush
	if (m_rtnSignalRate < MinReturnRateFilterFlush){
		// Completely lost target, so flush the filter
		FlushFilter = 1;
		bypassFilter = 1;
	}
	if (LastReturnRates[0] != 0){
		if (m_rtnSignalRate > LastReturnRates[0])
			RateChange = (100 * (m_rtnSignalRate - LastReturnRates[0])) / LastReturnRates[0];
		else
			RateChange = (100 * (LastReturnRates[0] - m_rtnSignalRate)) / LastReturnRates[0];
	}
	else
		RateChange = 0;
	if (RateChange > MaxReturnRateChangeFilterFlush){
		FlushFilter = 1;
	}

	if (FlushFilter == 1){
		MeasurementIndex = 0;
		for (i = 0; i < FILTERNBOFSAMPLES; i++){
			LastTrueRange[i] = FILTERINVALIDDISTANCE;
			LastReturnRates[i] = 0;
		}
	}
	else{
		for (i = (uint16_t)(FILTERNBOFSAMPLES - 1); i > 0; i--){
			LastTrueRange[i] = LastTrueRange[i - 1];
			LastReturnRates[i] = LastReturnRates[i - 1];
		}
	}
	if (ValidDistance == 1)
		LastTrueRange[0] = m_trueRange_mm;
	else
		LastTrueRange[0] = FILTERINVALIDDISTANCE;
	LastReturnRates[0] = m_rtnSignalRate;

	// Check if we need to go through the filter or not
	if (!(((m_rawRange_mm < WrapAroundHighRawRangeFilterLimit) && (m_rtnSignalRate < WrapAroundLowReturnRateFilterLimit)) ||
		((m_rawRange_mm >= WrapAroundHighRawRangeFilterLimit) && (m_rtnSignalRate < WrapAroundHighReturnRateFilterLimit))
		))
		bypassFilter = 1;

	// Check which kind of measurement has been made
	vl6180_i2c_read_8bits( 0x01AC, &registerValue);

	// Read data for filtering
	vl6180_i2c_read_32bits(0x010C, &register32BitsValue1);
	vl6180_i2c_read_32bits(0x0110, &register32BitsValue2);
	if (registerValue == 0x3E){
		Default_ZeroVal = register32BitsValue1;
		Default_VAVGVal = register32BitsValue2;
	}
	else{
		NoDelay_ZeroVal = register32BitsValue1;
		NoDelay_VAVGVal = register32BitsValue2;
	}

	if (bypassFilter == 1){
		// Do not go through the filter
		if (registerValue != 0x3E)
		{
			vl6180_i2c_write_8bits( 0x01AC, 0x3E);
		}
		// Set both Defaut and NoDelay To same value
		Default_ZeroVal = register32BitsValue1;
		Default_VAVGVal = register32BitsValue2;
		NoDelay_ZeroVal = register32BitsValue1;
		NoDelay_VAVGVal = register32BitsValue2;
		MeasurementIndex = 0;

		// Return immediately
		return m_newTrueRange_mm;
	}

	if (MeasurementIndex % WrapAroundNoDelayCheckPeriod == 0){
		vl6180_i2c_write_8bits( 0x01AC, 0x3F);
	}
	else{
		vl6180_i2c_write_8bits( 0x01AC, 0x3E);
	}

	MeasurementIndex = (uint16_t)(MeasurementIndex + 1);

	// Computes current VAVGDiff
	if (Default_VAVGVal > NoDelay_VAVGVal)
		VAVGDiff = Default_VAVGVal - NoDelay_VAVGVal;
	else
		VAVGDiff = 0;
	Previous_VAVGDiff = VAVGDiff;

	// Check the VAVGDiff
	if(Default_ZeroVal>NoDelay_ZeroVal)
		IdealVAVGDiff = Default_ZeroVal - NoDelay_ZeroVal;
	else
		IdealVAVGDiff = NoDelay_ZeroVal - Default_ZeroVal;
	if (IdealVAVGDiff > MAX_VAVGDiff)
		MinVAVGDiff = IdealVAVGDiff - MAX_VAVGDiff;
	else
		MinVAVGDiff = 0;
	MaxVAVGDiff = IdealVAVGDiff + MAX_VAVGDiff;
	if (VAVGDiff < MinVAVGDiff || VAVGDiff > MaxVAVGDiff){
		WrapAroundFlag = 1;
	}
	else{
		// Go through filtering check

		// StdDevLimit Damper on SNR
		StdDevLimit = VL6180_StdDevDamper(m_rtnAmbientRate, m_rtnSignalRate, StdDevLimitLowLight, StdDevLimitLowLightSNR, StdDevLimitHighLight, StdDevLimitHighLightSNR);

		// Standard deviations computations
		StdDevSamples = 0;
		StdDevDistanceSum = 0;
		StdDevDistanceMean = 0;
		StdDevDistance = 0;
		StdDevRateSum = 0;
		StdDevRateMean = 0;
		StdDevRate = 0;
		for (i = 0; (i < FILTERNBOFSAMPLES) && (StdDevSamples < FILTERSTDDEVSAMPLES); i++){
			if (LastTrueRange[i] != FILTERINVALIDDISTANCE){
				StdDevSamples = (uint16_t)(StdDevSamples + 1);
				StdDevDistanceSum = (uint32_t)(StdDevDistanceSum + LastTrueRange[i]);
				StdDevRateSum = (uint32_t)(StdDevRateSum + LastReturnRates[i]);
			}
		}
		if (StdDevSamples > 0){
			StdDevDistanceMean = (uint32_t)(StdDevDistanceSum / StdDevSamples);
			StdDevRateMean = (uint32_t)(StdDevRateSum / StdDevSamples);
		}
		StdDevSamples = 0;
		StdDevDistanceSum = 0;
		StdDevRateSum = 0;
		for (i = 0; (i < FILTERNBOFSAMPLES) && (StdDevSamples < FILTERSTDDEVSAMPLES); i++){
			if (LastTrueRange[i] != FILTERINVALIDDISTANCE){
				StdDevSamples = (uint16_t)(StdDevSamples + 1);
				StdDevDistanceSum = (uint32_t)(StdDevDistanceSum + (int)(LastTrueRange[i] - StdDevDistanceMean) * (int)(LastTrueRange[i] - StdDevDistanceMean));
				StdDevRateSum = (uint32_t)(StdDevRateSum + (int)(LastReturnRates[i] - StdDevRateMean) * (int)(LastReturnRates[i] - StdDevRateMean));
			}
		}
		if (StdDevSamples >= MINFILTERSTDDEVSAMPLES){
			StdDevDistance = (uint16_t)(StdDevDistanceSum / StdDevSamples);
			StdDevRate = (uint16_t)(StdDevRateSum / StdDevSamples);
		}
		else{
			StdDevDistance = 0;
			StdDevRate = 0;
		}

		// Check Return rate standard deviation
		if (StdDevRate < StdDevMovingTargetStdDevLimit){
			if (StdDevSamples < MINFILTERVALIDSTDDEVSAMPLES){
				m_newTrueRange_mm = MaxOrInvalidDistance;
			}
			else{
				// Check distance standard deviation
				if (StdDevRate < StdDevMovingTargetReturnRateLimit)
					StdDevLimitWithTargetMove = StdDevLimit + (((StdDevMovingTargetStdDevForReturnRateLimit - StdDevLimit) * StdDevRate) / StdDevMovingTargetReturnRateLimit);
				else
					StdDevLimitWithTargetMove = StdDevMovingTargetStdDevForReturnRateLimit;

				if ((StdDevDistance * StdDevHighConfidenceSNRLimit) < StdDevLimitWithTargetMove){
					NoWrapAroundHighConfidenceFlag = 1;
				}
				else{
					if (StdDevDistance < StdDevLimitWithTargetMove){
						if (StdDevSamples >= MINFILTERVALIDSTDDEVSAMPLES){
							NoWrapAroundFlag = 1;
						}
						else{
							m_newTrueRange_mm = MaxOrInvalidDistance;
						}
					}
					else{
						WrapAroundFlag = 1;
					}
				}
			}
		}
		else{
			WrapAroundFlag = 1;
		}
	}

	if (m_newTrueRange_mm == MaxOrInvalidDistance){
		if (StdFilteredReads > 0)
			StdFilteredReads = (uint16_t)(StdFilteredReads - 1);
	}
	else{
		if (WrapAroundFlag == 1){
			m_newTrueRange_mm = MaxOrInvalidDistance;
			StdFilteredReads = (uint16_t)(StdFilteredReads + StdFilteredReadsIncrement);
			if (StdFilteredReads > StdMaxFilteredReads)
				StdFilteredReads = StdMaxFilteredReads;
		}
		else{
			if (NoWrapAroundFlag == 1){
				if (StdFilteredReads > 0){
					m_newTrueRange_mm = MaxOrInvalidDistance;
					if (StdFilteredReads > StdFilteredReadsIncrement)
						StdFilteredReads = (uint16_t)(StdFilteredReads - StdFilteredReadsIncrement);
					else
						StdFilteredReads = 0;
				}
			}
			else{
				if (NoWrapAroundHighConfidenceFlag == 1){
					StdFilteredReads = 0;
				}
			}
		}
	}
	PreviousRangeStdDev = StdDevDistance;
	PreviousReturnRateStdDev = StdDevRate;
	PreviousStdDevLimit = StdDevLimitWithTargetMove;

	return m_newTrueRange_mm;
}
#endif	/* DISTANCE_FILTER */


static struct timeval t_start;
static struct timeval t_end;
unsigned long diff_timeval(struct timeval st, struct timeval et)
{
	return ((et.tv_sec * 1000000 + et.tv_usec) - (st.tv_sec * 1000000 + st.tv_usec));
}

int vl6180_init(void)
{
	int rc = 0;
	int i;
	int8_t offsetByte;
	uint16_t modelID = 0;
	uint16_t revID = 0;
	int8_t rangeTemp = 0;
	uint16_t chipidRange = 0;
	uint16_t CrosstalkHeight;
	uint16_t IgnoreThreshold;
	uint16_t IgnoreThresholdHeight;
	uint16_t dataByte;
	uint16_t ambpart2partCalib1 = 0;
	uint16_t ambpart2partCalib2 = 0;
#ifdef USE_INTERRUPTS
	uint16_t chipidgpio = 0;
#endif
	STMVL6180DB("[stmvl6180] vl6180_init ENTER!\n");

	vl6180_i2c_read_8bits( IDENTIFICATION__MODEL_ID, &modelID);
	vl6180_i2c_read_8bits( IDENTIFICATION__REVISION_ID, &revID);
	STMVL6180DB("[stmvl6180] Model ID : 0x%X, REVISION ID : 0x%X\n", modelID, revID);

	//waitForStandby
	for(i=0; i<100; i++){
		vl6180_i2c_read_8bits( FIRMWARE__BOOTUP, &modelID);
		if( (modelID & 0x01) == 1 ){
			i=100;
		}
	}

	//range device ready
	for(i=0; i<100; i++){
		vl6180_i2c_read_8bits( RESULT__RANGE_STATUS, &modelID);
		if( (modelID & 0x01) == 1){
			i = 100;
		}
	}

	vl6180_i2c_write_8bits( 0x0207, 0x01);
	vl6180_i2c_write_8bits( 0x0208, 0x01);
	vl6180_i2c_write_8bits( 0x0133, 0x01);
	vl6180_i2c_write_8bits( 0x0096, 0x00);
	vl6180_i2c_write_8bits( 0x0097, 0x54);
	vl6180_i2c_write_8bits( 0x00e3, 0x00);
	vl6180_i2c_write_8bits( 0x00e4, 0x04);
	vl6180_i2c_write_8bits( 0x00e5, 0x02);
	vl6180_i2c_write_8bits( 0x00e6, 0x01);
	vl6180_i2c_write_8bits( 0x00e7, 0x03);
	vl6180_i2c_write_8bits( 0x00f5, 0x02);
	vl6180_i2c_write_8bits( 0x00D9, 0x05);
	
	// AMB P2P calibration
	vl6180_i2c_read_8bits(SYSTEM__FRESH_OUT_OF_RESET, &dataByte);
	if(dataByte==0x01)
	{
		vl6180_i2c_read_8bits( 0x26, &dataByte);
		ambpart2partCalib1 = dataByte<<8;
		vl6180_i2c_read_8bits( 0x27, &dataByte);
		ambpart2partCalib1 = ambpart2partCalib1 + dataByte;
		vl6180_i2c_read_8bits( 0x28, &dataByte);
		ambpart2partCalib2 = dataByte<<8;
		vl6180_i2c_read_8bits( 0x29, &dataByte);
		ambpart2partCalib2 = ambpart2partCalib2 + dataByte;
		if(ambpart2partCalib1!=0)
		{
			// p2p calibrated
			vl6180_i2c_write_8bits( 0xDA, (ambpart2partCalib1>>8)&0xFF);
			vl6180_i2c_write_8bits( 0xDB, ambpart2partCalib1&0xFF);
			vl6180_i2c_write_8bits( 0xDC, (ambpart2partCalib2>>8)&0xFF);
			vl6180_i2c_write_8bits( 0xDD, ambpart2partCalib2&0xFF);
		}
		else
		{
			// No p2p Calibration, use default settings
			vl6180_i2c_write_8bits( 0xDB, 0xCE);
			vl6180_i2c_write_8bits( 0xDC, 0x03);
			vl6180_i2c_write_8bits( 0xDD, 0xF8);
		}
	}
	
	vl6180_i2c_write_8bits( 0x009f, 0x00);
	vl6180_i2c_write_8bits( 0x00a3, 0x28);
	vl6180_i2c_write_8bits( 0x00b7, 0x00);
	vl6180_i2c_write_8bits( 0x00bb, 0x28);
	vl6180_i2c_write_8bits( 0x00b2, 0x09);
	vl6180_i2c_write_8bits( 0x00ca, 0x09);
	vl6180_i2c_write_8bits( 0x0198, 0x01);
	vl6180_i2c_write_8bits( 0x01b0, 0x17);
	vl6180_i2c_write_8bits( 0x01ad, 0x00);
	vl6180_i2c_write_8bits( 0x00FF, 0x05);
	vl6180_i2c_write_8bits( 0x0100, 0x05);
	vl6180_i2c_write_8bits( 0x0199, 0x05);
	vl6180_i2c_write_8bits( 0x0109, 0x07);
	vl6180_i2c_write_8bits( 0x010a, 0x30);
	vl6180_i2c_write_8bits( 0x003f, 0x46);
	vl6180_i2c_write_8bits( 0x01a6, 0x1b);
	vl6180_i2c_write_8bits( 0x01ac, 0x3e);
	vl6180_i2c_write_8bits( 0x01a7, 0x1f);
	vl6180_i2c_write_8bits( 0x0103, 0x01);
	vl6180_i2c_write_8bits( 0x0030, 0x00);
	vl6180_i2c_write_8bits( 0x001b, 0x0A);
	vl6180_i2c_write_8bits( 0x003e, 0x0A);
	vl6180_i2c_write_8bits( 0x0131, 0x04);
	vl6180_i2c_write_8bits( 0x0011, 0x10);
	vl6180_i2c_write_8bits( 0x0014, 0x24);
	vl6180_i2c_write_8bits( 0x0031, 0xFF);
	vl6180_i2c_write_8bits( 0x00d2, 0x01);
	vl6180_i2c_write_8bits( 0x00f2, 0x01);

	// RangeSetMaxConvergenceTime
	vl6180_i2c_write_8bits( SYSRANGE__MAX_CONVERGENCE_TIME, 0x3F);
	vl6180_i2c_write_8bits( SYSRANGE__MAX_AMBIENT_LEVEL_MULT, 0xFF);//SNR
	
	vl6180_i2c_read_8bits(SYSTEM__FRESH_OUT_OF_RESET, &dataByte);
	if(dataByte==0x01)
	{
		//readRangeOffset
		vl6180_i2c_read_8bits( SYSRANGE__PART_TO_PART_RANGE_OFFSET, &dataByte);
		rangeTemp = (int8_t)dataByte;
		if(dataByte > 0x7F) {
			rangeTemp -= 0xFF;
		}
		rangeTemp /= 3;
		rangeTemp = rangeTemp +1; //roundg
		//Range_Set_Offset
		offsetByte = *((u8*)(&rangeTemp)); // round
		vl6180_i2c_write_8bits( SYSRANGE__PART_TO_PART_RANGE_OFFSET,(u8)offsetByte);
	}
	
	// ClearSystemFreshOutofReset
	vl6180_i2c_write_8bits( SYSTEM__FRESH_OUT_OF_RESET, 0x0);
	
	// VL6180 CrossTalk
	vl6180_i2c_write_8bits( SYSRANGE__CROSSTALK_COMPENSATION_RATE,(DEFAULT_CROSSTALK>>8)&0xFF);
	vl6180_i2c_write_8bits( SYSRANGE__CROSSTALK_COMPENSATION_RATE+1,DEFAULT_CROSSTALK&0xFF);
	
	CrosstalkHeight = 40;
	vl6180_i2c_write_8bits( SYSRANGE__CROSSTALK_VALID_HEIGHT,CrosstalkHeight&0xFF);
	
	
	// Will ignore all low distances (<100mm) with a low return rate
	IgnoreThreshold = 64; // 64 = 0.5Mcps
	IgnoreThresholdHeight = 33; // 33 * scaler3 = 99mm
	vl6180_i2c_write_8bits( SYSRANGE__RANGE_IGNORE_THRESHOLD, (IgnoreThreshold>>8)&0xFF);
	vl6180_i2c_write_8bits( SYSRANGE__RANGE_IGNORE_THRESHOLD+1,IgnoreThreshold&0xFF);
	vl6180_i2c_write_8bits( SYSRANGE__RANGE_IGNORE_VALID_HEIGHT,IgnoreThresholdHeight&0xFF);
	
	vl6180_i2c_read_8bits( SYSRANGE__RANGE_CHECK_ENABLES, &dataByte);
	dataByte = dataByte & 0xFE; // off ECE
	dataByte = dataByte | 0x02; // on ignore thr
	vl6180_i2c_write_8bits( SYSRANGE__RANGE_CHECK_ENABLES, dataByte);
	
	// Init of Averaging samples
	for(i=0; i<8;i++){
		LastMeasurements[i]=65535; // 65535 means no valid data
	}
	CurrentIndex = 0;

#ifdef USE_INTERRUPTS
	// SetSystemInterruptConfigGPIORanging
	vl6180_i2c_read_8bits( SYSTEM__INTERRUPT_CONFIG_GPIO, &chipidgpio);
	vl6180_i2c_write_8bits( SYSTEM__INTERRUPT_CONFIG_GPIO, (chipidgpio | 0x04));
#endif

	//RangeSetSystemMode
	chipidRange = 0x01;
	vl6180_i2c_write_8bits( SYSRANGE__START, chipidRange);
	do_gettimeofday(&t_start);

#ifdef DISTANCE_FILTER
	VL6180_InitDistanceFilter();
#endif

	return rc;
}

uint16_t vl6180_getDistance(void)
{
	uint16_t dist = 0;
	uint16_t chipidcount = 0;
	uint32_t m_rawRange_mm=0;
	uint32_t m_rtnConvTime=0;
	uint32_t m_rtnSignalRate=0;
	uint32_t m_rtnAmbientRate=0;
	uint32_t m_rtnSignalCount = 0;
	uint32_t m_refSignalCount = 0;
	uint32_t m_rtnAmbientCount =0;
	uint32_t m_refAmbientCount =0;
	uint32_t m_refConvTime =0;
	uint32_t m_refSignalRate =0;
	uint32_t m_refAmbientRate =0;
	uint32_t cRtnSignalCountMax = 0x7FFFFFFF;
	uint32_t  cDllPeriods = 6;
	uint32_t rtnSignalCountUInt = 0;
	uint32_t  calcConvTime = 0;
	uint16_t chipidRangeStart = 0;
	u8 val1 = 0, val2 = 0;
	uint16_t statusCode = 0;
	uint16_t errorCode = 0;
	struct stmvl6180_data *data = i2c_get_clientdata(stmvl6180_i2c_client);
	unsigned long diff;
	
	vl6180_i2c_read_8bits( SYSRANGE__START, &chipidRangeStart);
	//Read Error Code
	vl6180_i2c_read_8bits( RESULT__RANGE_STATUS, &statusCode);
	errorCode = statusCode>>4;


	if(((statusCode&0x01)==0x01)&&(chipidRangeStart==0x00)){
		do_gettimeofday(&t_end);
		vl6180_i2c_read_8bits(0x66, &val1);
		vl6180_i2c_read_8bits(0x67, &val2);
		return_rate = (val1 << 8) | (val2 & 0xFF);
		vl6180_i2c_read_8bits( RESULT__RANGE_VAL, &dist);

		dist *= 3;

		vl6180_i2c_read_8bits( RESULT__RANGE_RAW, &chipidcount);
		m_rawRange_mm = (uint32_t)chipidcount;

		vl6180_i2c_read_32bits(RESULT__RANGE_RETURN_SIGNAL_COUNT, &rtnSignalCountUInt);

		if(rtnSignalCountUInt > cRtnSignalCountMax){
			rtnSignalCountUInt = 0;
		}

		m_rtnSignalCount  = rtnSignalCountUInt;

		vl6180_i2c_read_32bits(RESULT__RANGE_REFERENCE_SIGNAL_COUNT, &m_refSignalCount);
		vl6180_i2c_read_32bits(RESULT__RANGE_RETURN_AMB_COUNT, &m_rtnAmbientCount);
		vl6180_i2c_read_32bits(RESULT__RANGE_REFERENCE_AMB_COUNT, &m_refAmbientCount);
		vl6180_i2c_read_32bits(RESULT__RANGE_RETURN_CONV_TIME, &m_rtnConvTime);
		vl6180_i2c_read_32bits(RESULT__RANGE_REFERENCE_CONV_TIME, &m_refConvTime);

		calcConvTime = m_refConvTime;
		if (m_rtnConvTime > m_refConvTime){
			calcConvTime = m_rtnConvTime;
		}
		if(calcConvTime==0)
			calcConvTime=63000;

		m_rtnSignalRate  = (m_rtnSignalCount*1000)/calcConvTime;
		m_refSignalRate  = (m_refSignalCount*1000)/calcConvTime;
		m_rtnAmbientRate = (m_rtnAmbientCount * cDllPeriods*1000)/calcConvTime;
		m_refAmbientRate = (m_rtnAmbientCount * cDllPeriods*1000)/calcConvTime;
		//printk("m_rtnSignalRate is %d, m_rtnAmbientRate is %d\n",m_rtnSignalRate,m_rtnAmbientRate);
#ifdef DISTANCE_FILTER
		dist = VL6180_DistanceFilter(dist, m_rawRange_mm*3, m_rtnSignalRate, m_rtnAmbientRate, errorCode);
#endif
		diff = diff_timeval(t_start, t_end);
		printk("====stmvl6180_diff:%lu==return_rate:%d==rtnsignalcount:%d==rtnconvtime:%d===dist:%d====\n", diff, return_rate, rtnSignalCountUInt, m_rtnConvTime, dist);

		// Start new measurement
		vl6180_i2c_write_8bits( SYSRANGE__START, 0x01);
		do_gettimeofday(&t_start);
		m_chipid = dist;
		data->rangeData.m_range = dist;
		data->rangeData.m_rtnRate = m_rtnSignalRate;
		data->rangeData.m_refRate = m_refSignalRate;
		data->rangeData.m_rtnAmbRate = m_rtnAmbientRate;
		data->rangeData.m_refAmbRate = m_refAmbientRate;
		data->rangeData.m_rawRange_mm = m_rawRange_mm*3;
		data->rangeData.m_ConvTime = calcConvTime;
	}
	else{
		// Return immediately with previous value
		dist = m_chipid;
	}
	return dist;

}

static long stmvl6180_compat_ioctl(struct file *filp, unsigned int cmd,
						unsigned long arg)
{
	void __user *data = compat_ptr(arg);
        return filp->f_op->unlocked_ioctl(filp, cmd, (unsigned long)data);
}

static long stmvl6180_ioctl(struct file *a_pstFile, unsigned int cmd,
						unsigned long arg)
{
	int rc=0;
	void __user *p = arg;

	u32 data=0;
	switch (cmd) {
	case VL6180_IOCTL_INIT:	   /* init.  */
	{
		vl6180_init();
		return 0;
	}
	case VL6180_IOCTL_GETDATA:	  /* Get proximity value only */
	{
		data = vl6180_getDistance();
		//printk("vl6180_getDistance return %ld\n",data);
		return put_user(data, (u32 *)p);
		//return data;
	}
	case VL6180_IOCTL_GETDATAS:	 /* Get all range data */
	{
		struct stmvl6180_data *pRange_Data= NULL ;
		
		data = vl6180_getDistance();
		if (stmvl6180_i2c_client != NULL)
			pRange_Data = i2c_get_clientdata(stmvl6180_i2c_client);	
		if (pRange_Data == NULL)
			return -EFAULT;
		if (copy_to_user((RangeData *)p, &(pRange_Data->rangeData), sizeof(RangeData))) {
			rc = -EFAULT;
		}	 
		return rc;   
	}
	default:
		return -EINVAL;
	}
	return rc;
}

static const struct file_operations g_stmvl6180_fops = 
{
    .owner = THIS_MODULE,
    .open = stmvl6180_open,
    .release = stmvl6180_release,
    .unlocked_ioctl = stmvl6180_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = stmvl6180_compat_ioctl,
#endif
};

static u8 flight_mode = 0;

static ssize_t flight_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	char *p = buf;
	unsigned long data=0;
	struct stmvl6180_data *st_data = i2c_get_clientdata(stmvl6180_i2c_client);

	switch (flight_mode) {
	case 1:
		data = vl6180_getDistance();
		p += sprintf(p, "[RAW]test get RAW distance(mm): %u\n", st_data->rangeData.m_rawRange_mm);
		p += sprintf(p, "test get distance(mm): %lu\n", data);
		p += sprintf(p, "return signal rate: %d\n", return_rate);
		if (return_rate < 26)
			p += sprintf(p, "[PASS]Laser assemble ok!\n");
		else
			p += sprintf(p, "[NO PASS]Laser assemble not ok!\n");

		return (p - buf);
	case 2:
		while(1) {
			data = vl6180_getDistance();
			if (flight_mode == 5)
				break;
		};
		break;
	default:
		break;
	}
	return sprintf(buf, "error flight mode!\n");
}

static ssize_t flight_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int val;
	sscanf(buf, "%d", &val);
	switch (val) {
	case 1:
		camera_flight_gpio(1);
		break;
	case 2:
		vl6180_init();
		flight_mode = 1;
		break;
	case 3:
		vl6180_init();
		flight_mode = 2;
		break;
	case 4:
		camera_flight_gpio(0);
		break;
	default:
		flight_mode = 5;
		break;
	}

	return count;
}


static struct device_attribute dev_attr_ctrl = {
	.attr = {.name = "flight_ctrl", .mode = 0644},
	.show = flight_show,
	.store = flight_store,
};

inline static int Register_stmvl6180_CharDrv(void)
{
    struct device* flight_device = NULL;

    //Allocate char driver no.
    if( alloc_chrdev_region(&stmvl6180_devno, 0, 1, STMVL6180_DRVNAME) )
    {
        STMVL6180DB("[stmvl6180] Allocate device no failed\n");
        return -EAGAIN;
    }

    //Allocate driver
    stmvl6180_chardrv = cdev_alloc();

    if(NULL == stmvl6180_chardrv)
    {
        unregister_chrdev_region(stmvl6180_devno, 1);

        STMVL6180DB("[stmvl6180] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(stmvl6180_chardrv, &g_stmvl6180_fops);

    stmvl6180_chardrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(stmvl6180_chardrv, stmvl6180_devno, 1))
    {
        STMVL6180DB("[stmvl6180] Attatch file operation failed\n");

        unregister_chrdev_region(stmvl6180_devno, 1);

        return -EAGAIN;
    }

    actuator_class = class_create(THIS_MODULE, "flight_sense");
    if (IS_ERR(actuator_class)) {
        int ret = PTR_ERR(actuator_class);
        STMVL6180DB("[stmvl6180]Unable to create class, err = %d\n", ret);
        return ret;            
    }

    flight_device = device_create(actuator_class, NULL, stmvl6180_devno, NULL, STMVL6180_DRVNAME);

    if(NULL == flight_device)
    {
        return -EIO;
    }
    
	device_create_file(flight_device, &dev_attr_ctrl);
    return 0;
}

inline static void unregister_stmvl6180_chardrv(void)
{
    STMVL6180DB("[stmvl6180] %s - Start\n", __func__);

    //Release char driver
    cdev_del(stmvl6180_chardrv);

    unregister_chrdev_region(stmvl6180_devno, 1);
    
    device_destroy(actuator_class, stmvl6180_devno);

    class_destroy(actuator_class);

    STMVL6180DB("[stmvl6180] %s - End\n", __func__);    
}

static int stmvl6180_i2c_remove(struct i2c_client *client) {
	struct stmvl6180_data *data = i2c_get_clientdata(stmvl6180_i2c_client);

	devm_kfree(&client->dev, data);
	return 0;
}

/* Kirby: add new-style driver {*/
static int stmvl6180_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i4RetValue = 0;
    struct stmvl6180_data *data;

    data = devm_kzalloc(&client->dev, sizeof(struct stmvl6180_data), GFP_KERNEL);
    if (!data) {
        STMVL6180DB("[STMVL6180] alloc memory failed!\n");
	return -ENOMEM;
    }

    data->client = client;
    i2c_set_clientdata(client, data);
    stmvl6180_i2c_client = client;
    
    stmvl6180_i2c_client->addr = stmvl6180_i2c_client->addr >> 1;
    
    //Register char driver
    i4RetValue = Register_stmvl6180_CharDrv();

    if(i4RetValue){

        STMVL6180DB("[STMVL6180] register char device failed!\n");

	devm_kfree(&client->dev, data);
        return i4RetValue;
    }

    spin_lock_init(&stmvl6180_spinlock);

    return 0;
}
struct i2c_driver stmvl6180_i2c_driver = {
    .probe = stmvl6180_i2c_probe,
    .remove = stmvl6180_i2c_remove,                           
    .driver.name = STMVL6180_DRVNAME,                 
    .id_table = stmvl6180_i2c_id,                             
};  

static int stmvl6180_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&stmvl6180_i2c_driver);
}

static int stmvl6180_remove(struct platform_device *pdev)
{
    i2c_del_driver(&stmvl6180_i2c_driver);
    return 0;
}

static int stmvl6180_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int stmvl6180_resume(struct platform_device *pdev)
{
    return 0;
}

/* platform structure */
static struct platform_driver stmvl6180_driver = {
    .probe	= stmvl6180_probe,
    .remove	= stmvl6180_remove,
    .suspend	= stmvl6180_suspend,
    .resume	= stmvl6180_resume,
    .driver	= {
        .name	= STMVL6180_DRVNAME,
        .owner	= THIS_MODULE,
    }
};

static struct platform_device stmvl6180_device = {
    .name = STMVL6180_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init stmvl6180_i2C_init(void)
{
    i2c_register_board_info(LENS_I2C_BUSNUM, &kd_lens_dev, 1);
	
    if(platform_driver_register(&stmvl6180_driver)){
        STMVL6180DB("[STMVL6180]failed to register stmvl6180 driver.\n");
        return -ENODEV;
    }

    if (platform_device_register(&stmvl6180_device)) {
        STMVL6180DB("[STMVL6180]failed to register stmvl6180 device.\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit stmvl6180_i2C_exit(void)
{
	platform_driver_unregister(&stmvl6180_driver);
}

module_init(stmvl6180_i2C_init);
module_exit(stmvl6180_i2C_exit);

MODULE_DESCRIPTION("stmvl6180 Flight-Sense module driver");
MODULE_LICENSE("GPL");
