/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 OV5670otp_common.h
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 OV5670 OTP common header file
 *
 ****************************************************************************/
#ifndef _OV5670_OTP_COMMON_H
#define _OV5670_OTP_COMMON_H

typedef struct qtech_otp_struct{
	int flag; // bit[7]: 0 no otp info, 1 valid otp info,bit[6]: 0 no otp wb, 1 valib otp wb 
	int module_id;
	int ir_id;
	int production_year;
	int production_month;
	int production_day;
	int awb_rg_msb;
	int awb_bg_msb;
	int awb_lsb;
}qtech_otp_struct;

typedef struct sunny_otp_struct{
	int flag; // bit[7]: 0 no otp info, 1 valid otp info,bit[6]: 0 no otp wb, 1 valib otp wb 
	int module_id;
	int ir_id;
	int production_year;
	int production_month;
	int production_day;
	int awb_rg_msb;
	int awb_bg_msb;
	int awb_lsb;
}sunny_otp_struct;

typedef struct avc_otp_struct{
	int flag; // bit[7]: 0 no otp info, 1 valid otp info,bit[6]: 0 no otp wb, 1 valib otp wb 
	int module_id;
	int ir_id;
	int production_year;
	int production_month;
	int production_day;
	int awb_rg_msb;
	int awb_bg_msb;
	int awb_lsb;
}avc_otp_struct;

typedef struct ofilm_otp_struct{
	int flag; // bit[7]: 0 no otp info, 1 valid otp info,bit[6]: 0 no otp wb, 1 valib otp wb 
	int module_id;
	int ir_id;
	int production_year;
	int production_month;
	int production_day;
	int awb_rg_msb;
	int awb_bg_msb;
	int awb_lsb;
}ofilm_otp_struct;

#endif 

