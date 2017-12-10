/*****************************************************************************
 *
 * Filename:
 * ---------
 *    linear_charging.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
  * $Revision:   1.0  $
 * $Modtime:   11 Aug 2005 10:28:16  $
 * $Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc  $
 *
 * 03 05 2015 wy.chuang
 * [ALPS01921641] [L1_merge] for PMIC and charging
 * .
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/kernel.h>
#include <mach/battery_common.h>
#include <mach/charging.h>
#include "cust_charging.h"
#include <mach/mt_boot.h>
#include <mach/battery_meter.h>
#include <linux/delay.h>
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include "cust_eint.h"
#include <mach/upmu_common.h>

#ifdef CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT
#include <mach/diso.h>
#endif

 /* ============================================================ // */
 /* define */
 /* ============================================================ // */
 /* cut off to full */
#define POST_CHARGING_TIME		30 * 60		/* 30mins */
#define FULL_CHECK_TIMES		6

 /* ============================================================ // */
 /* global variable */
 /* ============================================================ // */
kal_uint32 g_bcct_flag = 0;
kal_uint32 g_bcct_value = 0;
kal_uint32 g_full_check_count = 0;
CHR_CURRENT_ENUM g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
CHR_CURRENT_ENUM g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
kal_uint32 g_usb_state = USB_UNCONFIGURED;
static bool usb_unlimited=false;
BATTERY_VOLTAGE_ENUM cv_voltage;

extern int g_platform_boot_mode;

#define ADAPTER_DET_RETRY 5
int adapter_detect_count = 0;
static kal_uint32 chg_voltage[ADAPTER_DET_RETRY] = {0};
kal_bool vindpm_status = KAL_FALSE;

static kal_uint32 batt_temp_status = TEMP_POS_NORMAL;
extern void get_batt_temp_status(void *data);

void BATTERY_SetUSBState(int usb_state_value)
{
#if defined(CONFIG_POWER_EXT)
	battery_log(BAT_LOG_CRTI, "[BATTERY_SetUSBState] in FPGA/EVB, no service\r\n");
#else
	if ((usb_state_value < USB_SUSPEND) || ((usb_state_value > USB_CONFIGURED))) {
		battery_log(BAT_LOG_CRTI,
				    "[BATTERY] BAT_SetUSBState Fail! Restore to default value\r\n");
		usb_state_value = USB_UNCONFIGURED;
	} else {
		battery_log(BAT_LOG_CRTI, "[BATTERY] BAT_SetUSBState Success! Set %d\r\n",
				    usb_state_value);
		g_usb_state = usb_state_value;
	}
#endif
}

kal_uint32 get_charging_setting_current(void)
{
	return g_temp_CC_value;
}

bool get_usb_current_unlimited(void)
{
	if (BMT_status.charger_type == STANDARD_HOST || BMT_status.charger_type == CHARGING_HOST)
		return usb_unlimited;
	else
		return false;
}

void set_usb_current_unlimited(bool enable)
{
	if (g_boot_mode == LOW_POWER_OFF_CHARGING_BOOT || g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT) {
		usb_unlimited = false;
	} else {
		usb_unlimited = enable;
	}
}

//add by willcai for configing the current by vchg  begin
/*
This function  config the vbat  current from vchg 
ICHG=IBAT*VBAT/(VCHG*0.78)  VCHG<10v
ICHG=IBAT*VBAT/(VCHG*0.73)  VCHG>10v

*/
static kal_uint32 bmt_find_closest_current_level(kal_uint32 current_level)
{
    //add by willcai  8-13
	kal_uint32 vchg_level;

	battery_log(BAT_LOG_CRTI,"[BATTERY] bmt_find_closest_current_level  BMT_status.bat_vol=%d BMT_status.charger_vol=%d!\r\n",
			BMT_status.bat_vol ,BMT_status.charger_vol);
	current_level= current_level/100;
	if(BMT_status.charger_vol <10000)//2014-8-16 willcai modify 
	{
		vchg_level =(50*BMT_status.bat_vol *current_level)/(BMT_status.charger_vol*39);
	} else {
		vchg_level =(100*BMT_status.bat_vol *current_level)/(BMT_status.charger_vol*73);
	}

	vchg_level=100*vchg_level;
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] bmt_find_closest_current_level  vchg_level=%d !\r\n",vchg_level);

	battery_log(BAT_LOG_CRTI, "[BATTERY] bmt_find_closest_current_level  vchg_level=%d !\r\n",vchg_level);
	
	if(vchg_level>CHARGE_CURRENT_1650_00_MA)
	{
		current_level=CHARGE_CURRENT_1700_00_MA;
	}
	else if(vchg_level>CHARGE_CURRENT_1525_00_MA)
	{
		current_level=CHARGE_CURRENT_1600_00_MA;

	}
	else if(vchg_level>CHARGE_CURRENT_1450_00_MA)
	{
		current_level=CHARGE_CURRENT_1500_00_MA;

	}
	else if(vchg_level>CHARGE_CURRENT_1350_00_MA)
	{
		current_level=CHARGE_CURRENT_1400_00_MA;

	}
	else if(vchg_level>CHARGE_CURRENT_1250_00_MA)
	{
		current_level=CHARGE_CURRENT_1300_00_MA;

	}
	else if(vchg_level>CHARGE_CURRENT_1150_00_MA)
	{
		current_level=CHARGE_CURRENT_1200_00_MA;

	}
	else if(vchg_level>CHARGE_CURRENT_1050_00_MA)
	{
		current_level=CHARGE_CURRENT_1100_00_MA;

	}
	else if(vchg_level>CHARGE_CURRENT_950_00_MA)
	{
		current_level=CHARGE_CURRENT_1000_00_MA;

	}
	else if(vchg_level>CHARGE_CURRENT_850_00_MA)
	{
		current_level=CHARGE_CURRENT_900_00_MA;

	}
	else if(vchg_level>CHARGE_CURRENT_750_00_MA)
	{
		current_level=CHARGE_CURRENT_800_00_MA;

	}
	else if(vchg_level>CHARGE_CURRENT_650_00_MA)
	{
		current_level=CHARGE_CURRENT_700_00_MA;

	}
	else if(vchg_level>CHARGE_CURRENT_550_00_MA)
	{
		current_level=CHARGE_CURRENT_600_00_MA;

	}
	else if(vchg_level>CHARGE_CURRENT_450_00_MA)
	{
		current_level=CHARGE_CURRENT_500_00_MA;

	}
	else if(vchg_level>CHARGE_CURRENT_350_00_MA)
	{
		current_level=CHARGE_CURRENT_400_00_MA;

	}
	else 
	{
		current_level=CHARGE_CURRENT_300_00_MA;
	}
       return current_level;	
}
//end


void select_charging_curret_bcct(void)
{
	if (BMT_status.charger_type != STANDARD_HOST) {
		g_temp_input_CC_value = T10_CHARGER_CURRENT;
		g_temp_CC_value = T10_CHARGER_CURRENT;
	} else {
		g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		g_temp_CC_value = CHARGE_CURRENT_500_00_MA;
	}
}

static void pchr_turn_on_charging(void);
kal_uint32 set_bat_charging_current_limit(int current_limit)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] set_bat_charging_current_limit (%d)\r\n",
			    current_limit);

	if (current_limit != -1) {
		g_bcct_flag = 1;
		g_bcct_value = current_limit;

		if (current_limit < 70)
			g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
		else if (current_limit < 200)
			g_temp_CC_value = CHARGE_CURRENT_70_00_MA;
		else if (current_limit < 300)
			g_temp_CC_value = CHARGE_CURRENT_200_00_MA;
		else if (current_limit < 400)
			g_temp_CC_value = CHARGE_CURRENT_300_00_MA;
		else if (current_limit < 450)
			g_temp_CC_value = CHARGE_CURRENT_400_00_MA;
		else if (current_limit < 550)
			g_temp_CC_value = CHARGE_CURRENT_450_00_MA;
		else if (current_limit < 650)
			g_temp_CC_value = CHARGE_CURRENT_550_00_MA;
		else if (current_limit < 700)
			g_temp_CC_value = CHARGE_CURRENT_650_00_MA;
		else if (current_limit < 800)
			g_temp_CC_value = CHARGE_CURRENT_700_00_MA;
		else if (current_limit < 900)
			g_temp_CC_value = CHARGE_CURRENT_800_00_MA;
		else if (current_limit < 1000)
			g_temp_CC_value = CHARGE_CURRENT_900_00_MA;
		else if (current_limit < 1100)
			g_temp_CC_value = CHARGE_CURRENT_1000_00_MA;
		else if (current_limit < 1200)
			g_temp_CC_value = CHARGE_CURRENT_1100_00_MA;
		else if (current_limit < 1300)
			g_temp_CC_value = CHARGE_CURRENT_1200_00_MA;
		else if (current_limit < 1400)
			g_temp_CC_value = CHARGE_CURRENT_1300_00_MA;
		else if (current_limit < 1500)
			g_temp_CC_value = CHARGE_CURRENT_1400_00_MA;
		else if (current_limit < 1600)
			g_temp_CC_value = CHARGE_CURRENT_1500_00_MA;
		else if (current_limit == 1600)
			g_temp_CC_value = CHARGE_CURRENT_1600_00_MA;
		else
			g_temp_CC_value = CHARGE_CURRENT_450_00_MA;
	} else {
		/* change to default current setting */
		g_bcct_flag = 0;
	}

	return g_bcct_flag;
}

#define CHARGER_CURRENT_1_2A_THRESHOLD 4500
void select_charging_curret(void)
{
	kal_uint32 current_level = 0;
	kal_uint32 charger_voltage = 0, temp_voltage = 0;;
	static kal_uint32 sum_charger_voltage = 0;
	kal_uint32 charging_enable = KAL_TRUE;
	int i, j;
	kal_uint32 dpm_mode = 0;
	kal_uint32 dpm_gt = 0;

	if (g_ftm_battery_flag) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] FTM charging : %d\r\n",
				    charging_level_data[0]);
		g_temp_CC_value = charging_level_data[0];

		if (g_temp_CC_value == CHARGE_CURRENT_450_00_MA) {
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		} else {
			g_temp_input_CC_value = CHARGE_CURRENT_MAX;
			g_temp_CC_value = AC_CHARGER_CURRENT;

			battery_log(BAT_LOG_CRTI, "[BATTERY] set_ac_current \r\n");
		}
	} else {
		if (BMT_status.charger_type == STANDARD_HOST) {
#ifdef CONFIG_USB_IF
			{
				g_temp_input_CC_value = CHARGE_CURRENT_MAX;
				if (g_usb_state == USB_SUSPEND) {
					g_temp_CC_value = USB_CHARGER_CURRENT_SUSPEND;
				} else if (g_usb_state == USB_UNCONFIGURED) {
					g_temp_CC_value = USB_CHARGER_CURRENT_UNCONFIGURED;
				} else if (g_usb_state == USB_CONFIGURED) {
					g_temp_CC_value = USB_CHARGER_CURRENT_CONFIGURED;
				} else {
					g_temp_CC_value = USB_CHARGER_CURRENT_UNCONFIGURED;
				}

				battery_log(BAT_LOG_CRTI,
						    "[BATTERY] STANDARD_HOST CC mode charging : %d on %d state\r\n",
						    g_temp_CC_value, g_usb_state);
			}
#else
			{
				g_temp_input_CC_value = USB_CHARGER_CURRENT;
				g_temp_CC_value = USB_CHARGER_CURRENT;
			}
#endif
		} else if (BMT_status.charger_type == NONSTANDARD_CHARGER) {
			g_temp_input_CC_value = NON_STD_AC_CHARGER_CURRENT;
			g_temp_CC_value = NON_STD_AC_CHARGER_CURRENT;
		} else if (BMT_status.charger_type == STANDARD_CHARGER) {
			g_temp_input_CC_value = AC_CHARGER_CURRENT;
			g_temp_CC_value = AC_CHARGER_CURRENT;
		} else if (BMT_status.charger_type == CHARGING_HOST) {
			g_temp_input_CC_value = CHARGING_HOST_CHARGER_CURRENT;
			g_temp_CC_value = CHARGING_HOST_CHARGER_CURRENT;
		} else if (BMT_status.charger_type == APPLE_2_1A_CHARGER) {
			g_temp_input_CC_value = APPLE_2_1A_CHARGER_CURRENT;
			g_temp_CC_value = APPLE_2_1A_CHARGER_CURRENT;
		} else if (BMT_status.charger_type == APPLE_1_0A_CHARGER) {
			g_temp_input_CC_value = APPLE_1_0A_CHARGER_CURRENT;
			g_temp_CC_value = APPLE_1_0A_CHARGER_CURRENT;
		} else if (BMT_status.charger_type == APPLE_0_5A_CHARGER) {
			g_temp_input_CC_value = APPLE_0_5A_CHARGER_CURRENT;
			g_temp_CC_value = APPLE_0_5A_CHARGER_CURRENT;
		} else {
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
			g_temp_CC_value = CHARGE_CURRENT_500_00_MA;
		}

		if (BMT_status.charger_type != CHARGER_UNKNOWN &&
				BMT_status.charger_type != STANDARD_HOST) {
			g_temp_CC_value = AC_CHARGER_CURRENT;
			battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
			battery_charging_control(CHARGING_CMD_SET_CURRENT, &g_temp_CC_value);
			dpm_gt = mt6332_upmu_get_rgs_chr_gt_dpm();
			dpm_mode = mt6332_upmu_get_rgs_vin_dpm_mode();
			charger_voltage = battery_meter_get_charger_voltage();

			printk("%s:dpm_gt %d, dpm_mode %d, charger_voltage %d\n", __func__,
					dpm_gt, dpm_mode, charger_voltage);

			if (dpm_mode == 1 || dpm_gt == 0) {
				g_temp_CC_value = MEIZU_1_2A_CHARGER_CURRENT; 
				vindpm_status = KAL_TRUE;
			} else {
				g_temp_CC_value = AC_CHARGER_CURRENT; 
			}
			adapter_detect_count++;
		}
		#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		if (DISO_data.diso_state.cur_vdc_state == DISO_ONLINE) {
			g_temp_input_CC_value = AC_CHARGER_CURRENT;
			g_temp_CC_value = AC_CHARGER_CURRENT;
		}
		#endif
	}
}

static kal_uint32 charging_full_check(void)
{
	kal_uint32 status;

	battery_charging_control(CHARGING_CMD_GET_CHARGING_STATUS, &status);
	if (status == KAL_TRUE) {
		g_full_check_count++;
		if (g_full_check_count >= FULL_CHECK_TIMES) {
			return KAL_TRUE;
		} else
			return KAL_FALSE;
	} else {
		g_full_check_count = 0;
		return status;
	}
}

static void pchr_turn_on_charging(void)
{
	kal_uint32 charging_enable = KAL_TRUE;
	kal_uint32 current_level = 0;

	#if defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
	if(KAL_TRUE == BMT_status.charger_exist)
		charging_enable = KAL_TRUE;
	else 
		charging_enable = KAL_FALSE;
	#endif

	if (BMT_status.bat_charging_state == CHR_ERROR) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] Charger Error, turn OFF charging !\n");

		charging_enable = KAL_FALSE;

	} else if ((g_platform_boot_mode == META_BOOT) || (g_platform_boot_mode == ADVMETA_BOOT)) {
		battery_log(BAT_LOG_CRTI,
				    "[BATTERY] In meta or advanced meta mode, disable charging.\n");
		charging_enable = KAL_FALSE;
	} else {
		/*HW initialization */
		battery_charging_control(CHARGING_CMD_INIT, NULL);

		battery_log(BAT_LOG_FULL, "charging_hw_init\n");

		/* Set Charging Current */
		if (get_usb_current_unlimited()) {
			if (g_bcct_flag == 1) {
				g_temp_input_CC_value = T10_CHARGER_CURRENT;
				g_temp_CC_value = T10_CHARGER_CURRENT;
				g_bcct_flag = 0;
			} else {
				g_temp_input_CC_value = USB_CHARGER_ONLY_CURRENT;
				g_temp_CC_value = USB_CHARGER_ONLY_CURRENT;
			}
			battery_log(BAT_LOG_FULL,
					    "USB_CURRENT_UNLIMITED, use USB_CHARGER_ONLY_CURRENT\n");
		} else if (g_bcct_flag == 1) {
			select_charging_curret_bcct();
			g_bcct_flag = 0;
			battery_log(BAT_LOG_FULL, "[BATTERY] select_charging_curret_bcct !\n");
		} else {
			if ((adapter_detect_count < ADAPTER_DET_RETRY) && (vindpm_status == KAL_FALSE)) {
				select_charging_curret();
				//current_level = bmt_find_closest_current_level(g_temp_CC_value);
				//g_temp_CC_value = current_level;
			}
			battery_log(BAT_LOG_FULL, "[BATTERY] select_charging_curret !\n");
		}
		printk("[BATTERY] Default CC mode charging : %d, input current = %d\r\n",
				    g_temp_CC_value, g_temp_input_CC_value);
		if (g_temp_CC_value == CHARGE_CURRENT_0_00_MA
		    || g_temp_input_CC_value == CHARGE_CURRENT_0_00_MA) {
			charging_enable = KAL_FALSE;
			battery_log(BAT_LOG_CRTI,
					    "[BATTERY] charging current is set 0mA, turn off charging !\r\n");
		} else {
			battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT,
						 &g_temp_input_CC_value);
			battery_charging_control(CHARGING_CMD_SET_CURRENT, &g_temp_CC_value);

			/*Set CV Voltage */
#if !defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
#ifdef HIGH_BATTERY_VOLTAGE_SUPPORT
			cv_voltage = BATTERY_VOLT_04_350000_V;
#else
			cv_voltage = BATTERY_VOLT_04_200000_V;
#endif
			battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE, &cv_voltage);
#endif
		}
	}

	/* enable/disable charging */
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	battery_log(BAT_LOG_FULL, "[BATTERY] pchr_turn_on_charging(), enable =%d !\r\n",
			    charging_enable);
}


PMU_STATUS BAT_PreChargeModeAction(void)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] Pre-CC mode charge, timer=%d on %d !!\n\r",
			    BMT_status.PRE_charging_time, BMT_status.total_charging_time);

	BMT_status.PRE_charging_time += BAT_TASK_PERIOD;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;

	/*  Enable charger */
	pchr_turn_on_charging();

	if (BMT_status.UI_SOC == 100) {
		BMT_status.bat_charging_state = CHR_BATFULL;
		BMT_status.bat_full = KAL_TRUE;
		g_charging_full_reset_bat_meter = KAL_TRUE;
	} else if (BMT_status.bat_vol > V_PRE2CC_THRES) {
		BMT_status.bat_charging_state = CHR_CC;
	}

	return PMU_STATUS_OK;
}


PMU_STATUS BAT_ConstantCurrentModeAction(void)
{
	battery_log(BAT_LOG_FULL, "[BATTERY] CC mode charge, timer=%d on %d !!\n\r",
			    BMT_status.CC_charging_time, BMT_status.total_charging_time);

	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time += BAT_TASK_PERIOD;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;

	/*  Enable charger */
	pchr_turn_on_charging();

	if (charging_full_check() == KAL_TRUE) {
		BMT_status.bat_charging_state = CHR_BATFULL;
		BMT_status.bat_full = KAL_TRUE;
		g_charging_full_reset_bat_meter = KAL_TRUE;
	}

	return PMU_STATUS_OK;
}


PMU_STATUS BAT_BatteryFullAction(void)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] Battery full !!\n\r");

	BMT_status.bat_full = KAL_TRUE;
	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;
	BMT_status.bat_in_recharging_state = KAL_FALSE;

	if (charging_full_check() == KAL_FALSE) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] Battery Re-charging !!\n\r");

		BMT_status.bat_in_recharging_state = KAL_TRUE;
		BMT_status.bat_charging_state = CHR_CC;
	}

	return PMU_STATUS_OK;
}


PMU_STATUS BAT_BatteryHoldAction(void)
{
	kal_uint32 charging_enable;

	battery_log(BAT_LOG_CRTI, "[BATTERY] Hold mode !!\n\r");

	if (BMT_status.bat_vol < TALKING_RECHARGE_VOLTAGE || g_call_state == CALL_IDLE) {
		BMT_status.bat_charging_state = CHR_CC;
		battery_log(BAT_LOG_CRTI,
				    "[BATTERY] Exit Hold mode and Enter CC mode !!\n\r");
	}

	/*  Disable charger */
	charging_enable = KAL_FALSE;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	return PMU_STATUS_OK;
}


PMU_STATUS BAT_BatteryStatusFailAction(void)
{
	kal_uint32 charging_enable;

	battery_log(BAT_LOG_CRTI, "[BATTERY] BAD Battery status... Charging Stop !!\n\r");

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
	if ((g_temp_status == TEMP_ABOVE_POS_60) || (g_temp_status == TEMP_BELOW_NEG_10)) {
		temp_error_recovery_chr_flag = KAL_FALSE;
	}
	if ((temp_error_recovery_chr_flag == KAL_FALSE) && (g_temp_status != TEMP_ABOVE_POS_60)
	    && (g_temp_status != TEMP_BELOW_NEG_10)) {
		temp_error_recovery_chr_flag = KAL_TRUE;
		BMT_status.bat_charging_state = CHR_PRE;
	}
#endif

	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;

	/*  Disable charger */
	charging_enable = KAL_FALSE;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	return PMU_STATUS_OK;
}


void mt_battery_charging_algorithm(void)
{
	battery_charging_control(CHARGING_CMD_RESET_WATCH_DOG_TIMER, NULL);

	/* set the temperature charging control */
	get_batt_temp_status(&batt_temp_status);
	if (batt_temp_status == TEMP_POS_T10) {
		g_bcct_flag = 1;
		adapter_detect_count = 0;
		vindpm_status = KAL_FALSE;
	}

	switch (BMT_status.bat_charging_state) {
	case CHR_PRE:
		BAT_PreChargeModeAction();
		break;

	case CHR_CC:
		BAT_ConstantCurrentModeAction();
		break;

	case CHR_BATFULL:
		BAT_BatteryFullAction();
		break;

	case CHR_HOLD:
		BAT_BatteryHoldAction();
		break;

	case CHR_ERROR:
		BAT_BatteryStatusFailAction();
		break;
	}

	battery_charging_control(CHARGING_CMD_DUMP_REGISTER, NULL);
}
