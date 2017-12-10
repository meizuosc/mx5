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
#include <mach/bq2589x_reg.h>

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#if !defined(TA_AC_CHARGING_CURRENT)
#include "cust_pe.h"
#endif
#endif

#include <mach/bq27532_battery.h>
#include <mach/mt_idle.h>
#include <uapi/linux/sched.h>
#include <asm-generic/bug.h>

 /* ============================================================ // */
 /* define */
 /* ============================================================ // */
 /* cut off to full */
#define POST_CHARGING_TIME		30 * 60		/* 30mins */
#define FULL_CHECK_TIMES		6

 /* ============================================================ // */
 /* global variable */
 /* ============================================================ // */
kal_uint32 g_changed_cv = 0;
static kal_bool cv_changed_flag = KAL_FALSE;
kal_uint32 g_bcct_flag = 0;
kal_uint32 g_bcct_value = 0;
kal_uint32 g_full_check_count = 0;
CHR_CURRENT_ENUM g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
CHR_CURRENT_ENUM g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
kal_uint32 g_usb_state = USB_UNCONFIGURED;
static bool usb_unlimited=false;
  /* ///////////////////////////////////////////////////////////////////////////////////////// */
  /* // PUMP EXPRESS */
  /* ///////////////////////////////////////////////////////////////////////////////////////// */
#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
struct wake_lock TA_charger_suspend_lock;
kal_bool ta_check_chr_type = KAL_TRUE;
kal_bool ta_cable_out_occur = KAL_FALSE;
kal_bool is_ta_connect = KAL_FALSE;
kal_bool ta_vchr_tuning = KAL_TRUE;
int ta_v_chr_org = 0;
#endif


  /* ///////////////////////////////////////////////////////////////////////////////////////// */
  /* // JEITA */
  /* ///////////////////////////////////////////////////////////////////////////////////////// */
#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
int g_temp_status = TEMP_POS_10_TO_POS_45;
kal_bool temp_error_recovery_chr_flag = KAL_TRUE;
#endif

static batt_temp_state_enum batt_temp_status = TEMP_POS_NORMAL;
static batt_temp_state_enum batt_temp_pre_status = TEMP_POS_NORMAL;
extern void get_batt_temp_status(void *data);
bool usb_error_check = KAL_FALSE;

 /* ============================================================ // */
 /* function prototype */
 /* ============================================================ // */


 /* ============================================================ // */
 /* extern variable */
 /* ============================================================ // */
extern int g_platform_boot_mode;

 /* ============================================================ // */
 /* extern function */
 /* ============================================================ // */


 /* ============================================================ // */
void BATTERY_SetUSBState(int usb_state_value)
{
#if defined(CONFIG_POWER_EXT)
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY_SetUSBState] in FPGA/EVB, no service\r\n");
#else
	if ((usb_state_value < USB_SUSPEND) || ((usb_state_value > USB_CONFIGURED))) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY] BAT_SetUSBState Fail! Restore to default value\r\n");
		usb_state_value = USB_UNCONFIGURED;
	} else {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BAT_SetUSBState Success! Set %d\r\n",
				    usb_state_value);
		g_usb_state = usb_state_value;
	}
#endif
}


kal_uint32 get_charging_setting_current(void)
{
	return g_temp_CC_value;
}

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
static DEFINE_MUTEX(ta_mutex);

static void set_ta_charging_current(void)
{
	printk("%s:**fast_charge_current_1C %d\n", __func__, fast_charge_current_1C);
	if (fast_charge_current_1C == KAL_TRUE) {
		//g_temp_input_CC_value = TA_AC_12V_INPUT_CURRENT;  //TA = 12V		
		//g_temp_CC_value = TA_AC_2A_CHARGING_CURRENT;
		g_temp_input_CC_value = TA_AC_9V_INPUT_CURRENT;  //TA = 9V		
		g_temp_CC_value = TA_AC_2_5A_CHARGING_CURRENT;
	} else {
		g_temp_input_CC_value = TA_AC_1_5C_INPUT_CURRENT;  //TA = 12V		
		//g_temp_CC_value = TA_AC_4_5A_CHARGING_CURRENT; //TA=12v
		g_temp_CC_value = TA_AC_4_0A_CHARGING_CURRENT; //TA=9v
	}
}

void mtk_ta_reset_vchr(void)
{
	CHR_CURRENT_ENUM chr_current = CHARGE_CURRENT_1500_00_MA;
	CHR_CURRENT_ENUM chr_current_reset = CHARGE_CURRENT_70_00_MA;

	battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT,&chr_current);
	battery_charging_control(CHARGING_CMD_SET_ONLYONE_INPUT_CURRENT,&chr_current_reset);
	msleep(250);    // reset Vchr to 5V

	printk("mtk_ta_reset_vchr()reset Vchr to 5V \n");
}

#if 1
static void mtk_ta_increase(void)
{
	kal_bool ta_current_pattern = KAL_TRUE;  // TRUE = increase

	if(ta_cable_out_occur == KAL_FALSE)
	{
		battery_charging_control(CHARGING_CMD_SET_TA_CURRENT_PATTERN, &ta_current_pattern);
	}
	else
	{
		ta_check_chr_type = KAL_TRUE;
		battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_increase() Cable out \n");
	}
}
#endif

#if 0
static void mtk_ta_increase(void)
{
	bq2589x_enable_pumpx(1);
	bq2589x_pumpx_up();
	msleep(2500);
}
#endif
static kal_bool mtk_ta_retry_increase(void)
{
	int real_v_chrA;
	int real_v_chrB;
	kal_bool retransmit = KAL_TRUE;
	kal_uint32 retransmit_count=0;
	
	do {
		/*
		   SCHED_FIFO类型的优先级，param.sched_priority数值越大优先级越大,
		   线程化中断的优先级是50, 防止阻塞中断，设置需要小于50
		  */
		set_bat_thread_priority(41);

		real_v_chrA = bq2589x_adc_read_charger_volt();
		mtk_ta_increase();  //increase TA voltage to 7V
		real_v_chrB = bq2589x_adc_read_charger_volt();

		if(real_v_chrB - real_v_chrA >= 1000) {	/* 1.0V */
			retransmit = KAL_FALSE;
		} else {
			retransmit_count++;
			printk("mtk_ta_detector(): retransmit_count =%d, chrA=%d, chrB=%d\n", 
				retransmit_count, real_v_chrA, real_v_chrB);
		}

		if((retransmit_count == 5) || (BMT_status.charger_exist == KAL_FALSE)) {
			retransmit = KAL_FALSE;
		}

		set_bat_thread_priority(-10);
	} while((retransmit == KAL_TRUE) && (ta_cable_out_occur == KAL_FALSE));

	printk("mtk_ta_retry_increase() real_v_chrA=%d, real_v_chrB=%d, retry=%d\n", 
			real_v_chrA, real_v_chrB,retransmit_count);

	if(retransmit_count == 5)
		return KAL_FALSE;	
	else
		return KAL_TRUE;	
}

void mtk_ta_detector(void)
{
	int real_v_chrB = 0;

	printk("mtk_ta_detector() start\n");

	ta_v_chr_org = bq2589x_adc_read_charger_volt();
	mtk_ta_retry_increase();
	real_v_chrB = bq2589x_adc_read_charger_volt();

	if(((ta_v_chr_org != 0) && (real_v_chrB - ta_v_chr_org >= 1000)) || real_v_chrB >= 6000)
		is_ta_connect = KAL_TRUE;
	else
		is_ta_connect = KAL_FALSE;
		
	printk("mtk_ta_detector() end, is_ta_connect=%d, ta_v_chr_org %d, real_v_chrB %d\n",
			is_ta_connect, ta_v_chr_org, real_v_chrB);
}

static void mtk_ta_init(void)
{
	is_ta_connect = KAL_FALSE;
	ta_cable_out_occur = KAL_FALSE;
	BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_350000_V;

#ifdef TA_9V_SUPPORT
	ta_vchr_tuning = KAL_FALSE;
#endif

	battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE, &cv_voltage);
	battery_charging_control(CHARGING_CMD_INIT,NULL);
}

static void battery_pump_express_charger_check(void)
{
	if (KAL_TRUE == ta_check_chr_type &&
		((STANDARD_CHARGER == BMT_status.charger_type) ||
		 (NONSTANDARD_CHARGER == BMT_status.charger_type))) {
		idle_lock_spm(IDLE_SPM_LOCK_PUMP_CHARGER);
		mutex_lock(&ta_mutex);
		wake_lock(&TA_charger_suspend_lock);

		bq2589x_disable_ico(1);
		msleep(1000);
		mtk_ta_reset_vchr();
		mtk_ta_init();

		mtk_ta_detector();

		/* need to re-check if the charger plug out during ta detector */
		if(KAL_TRUE == ta_cable_out_occur)
			ta_check_chr_type = KAL_TRUE;
		else
			ta_check_chr_type = KAL_FALSE;

		wake_unlock(&TA_charger_suspend_lock);
		mutex_unlock(&ta_mutex);
		idle_unlock_spm(IDLE_SPM_LOCK_PUMP_CHARGER);
	} else {
		battery_xlog_printk(BAT_LOG_CRTI, 
		"Stop battery_pump_express_charger_check, SOC=%d, ta_check_chr_type = %d, charger_type = %d \n", 
		BMT_status.SOC, ta_check_chr_type, BMT_status.charger_type);
	}
}

static void battery_pump_express_algorithm_start(void)
{
	kal_int32 charger_vol;
	kal_uint32 charging_enable = KAL_FALSE;

	idle_lock_spm(IDLE_SPM_LOCK_PUMP_CHARGER);
	mutex_lock(&ta_mutex);
	wake_lock(&TA_charger_suspend_lock);

	if(KAL_TRUE == is_ta_connect) {
		/* check cable impedance */
		charger_vol = bq2589x_adc_read_charger_volt();
		if(KAL_FALSE == ta_vchr_tuning) {
			mtk_ta_retry_increase();	/* increase TA voltage to 9V */
			mtk_ta_retry_increase();	/* increase TA voltage to 12V */
			set_ta_charging_current();
			ta_vchr_tuning = KAL_TRUE;
		} else if (charger_vol < 6500) {
			printk("%s:the charger vol is %d\n", __func__, charger_vol);
			battery_charging_control(CHARGING_CMD_ENABLE,&charging_enable);
			mtk_ta_reset_vchr();
			mtk_ta_retry_increase();	/* increase TA voltage to 7V */
			mtk_ta_retry_increase();	/* increase TA voltage to 9V */
		} else if(BMT_status.SOC > TA_STOP_BATTERY_SOC) {
			/* disable charging, avoid Iterm issue */
			battery_charging_control(CHARGING_CMD_ENABLE,&charging_enable);
			mtk_ta_reset_vchr();	//decrease TA voltage to 5V
			charger_vol = bq2589x_adc_read_charger_volt();
			if(abs(charger_vol - ta_v_chr_org) <= 1000)	/* 1.0V */
				is_ta_connect = KAL_FALSE;
			printk("Stop battery_pump_express_algorithm, SOC=%d is_ta_connect =%d, TA_STOP_BATTERY_SOC: %d\n",
				BMT_status.SOC, is_ta_connect, TA_STOP_BATTERY_SOC);
		}
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] check cable impedance, VA(%d) VB(%d) delta(%d).\n", 
		ta_v_chr_org, charger_vol, charger_vol - ta_v_chr_org);

		battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_algorithm() end\n");
	} else {
		printk("%s:It's not a TA charger, bypass TA algorithm\n", __func__);
	}

	wake_unlock(&TA_charger_suspend_lock);
	mutex_unlock(&ta_mutex);
	idle_unlock_spm(IDLE_SPM_LOCK_PUMP_CHARGER);
}
#endif

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)

static BATTERY_VOLTAGE_ENUM select_jeita_cv(void)
{
	BATTERY_VOLTAGE_ENUM cv_voltage;

	if (g_temp_status == TEMP_ABOVE_POS_60) {
		cv_voltage = JEITA_TEMP_ABOVE_POS_60_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_POS_45_TO_POS_60) {
		cv_voltage = JEITA_TEMP_POS_45_TO_POS_60_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_POS_10_TO_POS_45) {
#ifdef HIGH_BATTERY_VOLTAGE_SUPPORT
		cv_voltage = BATTERY_VOLT_04_340000_V;
#else
		cv_voltage = JEITA_TEMP_POS_10_TO_POS_45_CV_VOLTAGE;
#endif
	} else if (g_temp_status == TEMP_POS_0_TO_POS_10) {
		cv_voltage = JEITA_TEMP_POS_0_TO_POS_10_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_NEG_10_TO_POS_0) {
		cv_voltage = JEITA_TEMP_NEG_10_TO_POS_0_CV_VOLTAGE;
	} else if (g_temp_status == TEMP_BELOW_NEG_10) {
		cv_voltage = JEITA_TEMP_BELOW_NEG_10_CV_VOLTAGE;
	} else {
		cv_voltage = BATTERY_VOLT_04_200000_V;
	}

	return cv_voltage;
}

PMU_STATUS do_jeita_state_machine(void)
{
	BATTERY_VOLTAGE_ENUM cv_voltage;

	/* JEITA battery temp Standard */

	if (BMT_status.temperature >= TEMP_POS_60_THRESHOLD) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY] Battery Over high Temperature(%d) !!\n\r",
				    TEMP_POS_60_THRESHOLD);

		g_temp_status = TEMP_ABOVE_POS_60;

		return PMU_STATUS_FAIL;
	} else if (BMT_status.temperature > TEMP_POS_45_THRESHOLD)	/* control 45c to normal behavior */
	{
		if ((g_temp_status == TEMP_ABOVE_POS_60)
		    && (BMT_status.temperature >= TEMP_POS_60_THRES_MINUS_X_DEGREE)) {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
					    TEMP_POS_60_THRES_MINUS_X_DEGREE,
					    TEMP_POS_60_THRESHOLD);

			return PMU_STATUS_FAIL;
		} else {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
					    TEMP_POS_45_THRESHOLD, TEMP_POS_60_THRESHOLD);

			g_temp_status = TEMP_POS_45_TO_POS_60;
		}
	} else if (BMT_status.temperature >= TEMP_POS_10_THRESHOLD) {
		if (((g_temp_status == TEMP_POS_45_TO_POS_60)
		     && (BMT_status.temperature >= TEMP_POS_45_THRES_MINUS_X_DEGREE))
		    || ((g_temp_status == TEMP_POS_0_TO_POS_10)
			&& (BMT_status.temperature <= TEMP_POS_10_THRES_PLUS_X_DEGREE))) {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature not recovery to normal temperature charging mode yet!!\n\r");
		} else {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[BATTERY] Battery Normal Temperature between %d and %d !!\n\r",
					    TEMP_POS_10_THRESHOLD, TEMP_POS_45_THRESHOLD);
			g_temp_status = TEMP_POS_10_TO_POS_45;
		}
	} else if (BMT_status.temperature >= TEMP_POS_0_THRESHOLD) {
		if ((g_temp_status == TEMP_NEG_10_TO_POS_0 || g_temp_status == TEMP_BELOW_NEG_10)
		    && (BMT_status.temperature <= TEMP_POS_0_THRES_PLUS_X_DEGREE)) {
			if (g_temp_status == TEMP_NEG_10_TO_POS_0) {
				battery_xlog_printk(BAT_LOG_CRTI,
						    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
						    TEMP_POS_0_THRES_PLUS_X_DEGREE,
						    TEMP_POS_10_THRESHOLD);
			}
			if (g_temp_status == TEMP_BELOW_NEG_10) {
				battery_xlog_printk(BAT_LOG_CRTI,
						    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
						    TEMP_POS_0_THRESHOLD,
						    TEMP_POS_0_THRES_PLUS_X_DEGREE);
				return PMU_STATUS_FAIL;
			}
		} else {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
					    TEMP_POS_0_THRESHOLD, TEMP_POS_10_THRESHOLD);

			g_temp_status = TEMP_POS_0_TO_POS_10;
		}
	} else if (BMT_status.temperature >= TEMP_NEG_10_THRESHOLD) {
		if ((g_temp_status == TEMP_BELOW_NEG_10)
		    && (BMT_status.temperature <= TEMP_NEG_10_THRES_PLUS_X_DEGREE)) {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
					    TEMP_NEG_10_THRESHOLD, TEMP_NEG_10_THRES_PLUS_X_DEGREE);

			return PMU_STATUS_FAIL;
		} else {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[BATTERY] Battery Temperature between %d and %d !!\n\r",
					    TEMP_NEG_10_THRESHOLD, TEMP_POS_0_THRESHOLD);

			g_temp_status = TEMP_NEG_10_TO_POS_0;
		}
	} else {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY] Battery below low Temperature(%d) !!\n\r",
				    TEMP_NEG_10_THRESHOLD);
		g_temp_status = TEMP_BELOW_NEG_10;

		return PMU_STATUS_FAIL;
	}

	/* set CV after temperature changed */

	cv_voltage = select_jeita_cv();
	battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE, &cv_voltage);

	return PMU_STATUS_OK;
}


static void set_jeita_charging_current(void)
{
#ifdef CONFIG_USB_IF
	if (BMT_status.charger_type == STANDARD_HOST)
		return;
#endif

	if (g_temp_status == TEMP_NEG_10_TO_POS_0) {
		g_temp_CC_value = CHARGE_CURRENT_350_00_MA;
		g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] JEITA set charging current : %d\r\n",
				    g_temp_CC_value);
	}
}

#endif

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

void select_charging_curret_bcct(void)
{
	BATTERY_VOLTAGE_ENUM cv_voltage;

	switch (batt_temp_status) {
		case TEMP_POS_T10:
			g_temp_CC_value = CHARGING_0_3C_CHARGER_CURRENT; 
			g_temp_input_CC_value = CHARGING_0_3C_CHARGER_CURRENT; 
			cv_voltage = BATTERY_VOLT_04_350000_V;
			break;
		case TEMP_POS_T15:
        		if(is_ta_connect == KAL_TRUE) {
				g_temp_CC_value = TA_AC_1C_CHARGING_CURRENT; 
				g_temp_input_CC_value = TA_AC_1C_CHARGING_CURRENT; 
			} else {
				g_temp_CC_value = AC_CHARGER_CURRENT; 
				g_temp_input_CC_value = AC_CHARGER_CURRENT; 
			}
			cv_voltage = BATTERY_VOLT_04_350000_V;
			break;
		case TEMP_POS_T20:
			if (is_ta_connect == KAL_TRUE && g_changed_cv == 0) {
				//g_temp_CC_value = TA_AC_4_5A_CHARGING_CURRENT;//12V
				g_temp_CC_value = TA_AC_4_0A_CHARGING_CURRENT;//9V
				g_temp_input_CC_value = TA_AC_1_5C_INPUT_CURRENT;
				cv_voltage = BATTERY_VOLT_04_200000_V;
			} else {
				g_temp_CC_value = AC_CHARGER_CURRENT; 
				g_temp_input_CC_value = AC_CHARGER_CURRENT;
				cv_voltage = BATTERY_VOLT_04_350000_V;
			}
			break;
		case TEMP_POS_T42:
			g_temp_CC_value = AC_CHARGER_CURRENT;
			g_temp_input_CC_value = AC_CHARGER_CURRENT;
			cv_voltage = BATTERY_VOLT_04_350000_V;
			break;
		default:
			break;
	}
	battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE, &cv_voltage);
}

static void pchr_turn_on_charging(void);
kal_uint32 set_bat_charging_current_limit(int current_limit)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] set_bat_charging_current_limit (%d)\r\n",
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

	/* wake_up_bat(); */
	pchr_turn_on_charging();

	return g_bcct_flag;
}

static void usb_error_detect_check(void)
{
	int real_v_chrA, real_v_chrB;

	printk("%s:\n", __func__);
	bq2589x_set_en_hiz_minor(0);
	bq2589x_disable_ico(0);
	msleep(20);
	bq2589x_enable_pumpx(1);
	bq2589x_pumpx_up();
	usb_error_check = KAL_TRUE;
	BMT_status.charger_vol = bq2589x_adc_read_charger_volt();
	if (BMT_status.charger_vol > 6000) {
		BMT_status.charger_type = STANDARD_CHARGER;
	}
}

void select_charging_curret(void)
{
	int vdpm_status, idpm_status;
	int idpm_limit, ico_status;
	kal_uint32 charging_enable = KAL_TRUE;
	u8 reg_value;
	BATTERY_VOLTAGE_ENUM cv_voltage;
	int i = 0;

	if (g_ftm_battery_flag) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] FTM charging : %d\r\n",
				    charging_level_data[0]);
		g_temp_CC_value = charging_level_data[0];

		if (g_temp_CC_value == CHARGE_CURRENT_450_00_MA) {
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
		} else {
			g_temp_input_CC_value = CHARGE_CURRENT_MAX;
			g_temp_CC_value = AC_CHARGER_CURRENT;

			battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] set_ac_current \r\n");
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

				battery_xlog_printk(BAT_LOG_CRTI,
						    "[BATTERY] STANDARD_HOST CC mode charging : %d on %d state\r\n",
						    g_temp_CC_value, g_usb_state);
			}
#else
			{
				g_temp_input_CC_value = USB_CHARGER_CURRENT;
				g_temp_CC_value = USB_CHARGER_CURRENT;
				if (usb_error_check == KAL_FALSE) {
					if (wait_for_completion_timeout(&completion_dwork, 2*HZ) == 0)//timeout
						usb_error_detect_check();
					init_completion(&completion_dwork);
				}
				cv_voltage = BATTERY_VOLT_04_350000_V;
				battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE, &cv_voltage);
				return;
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
		} else {
			g_temp_input_CC_value = CHARGE_CURRENT_500_00_MA;
			g_temp_CC_value = CHARGE_CURRENT_500_00_MA;
		}
	
		if (BMT_status.adapter_num == ADAPTER_OUTPUT_0A && BMT_status.charger_type != STANDARD_HOST) {
			printk("%s:The Adapter is not fast charger, so enable ICO\n", __func__);

			battery_charging_control(CHARGING_CMD_SET_ADC_STOP, NULL);

			g_temp_input_CC_value = CHARGING_CURRENT_ICO;
			g_temp_CC_value = CHARGING_CURRENT_ICO;

			bq2589x_disable_ico(0);
			bq2589x_force_ico();
			battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT,&g_temp_input_CC_value);
			battery_charging_control(CHARGING_CMD_SET_CURRENT,&g_temp_CC_value);

			battery_charging_control(CHARGING_CMD_ENABLE_MAJOR, &charging_enable);

			msleep(1000);
			ico_status = bq2589x_get_ico_status();
			if (ico_status == 0) {
				msleep(500);
				ico_status = bq2589x_get_ico_status();
			}
			bq2589x_get_vindpm_status(&vdpm_status);
			bq2589x_get_iindpm_status(&idpm_status);

			if (ico_status) {
				idpm_limit = bq2589x_get_idpm_limit();

				printk("%s:ico_status %d, vdpm_status %d, idpm_status %d, idpm_limit %d\n",
						__func__, ico_status, vdpm_status, idpm_status, idpm_limit);
				
				if (idpm_limit <= CHARGE_CURRENT_400_00_MA / 100) {
					printk("%s:the charger is working error\n", __func__);
					BMT_status.adapter_num = ADAPTER_OUTPUT_0A;
				} else if (idpm_limit <= CHARGE_CURRENT_1300_00_MA / 100) {
					printk("%s:It is 5V/1.2A Charger\n", __func__);
					BMT_status.adapter_num = ADAPTER_OUTPUT_1_2A;
				} else if (idpm_limit <= CHARGE_CURRENT_1600_00_MA / 100) {
					BMT_status.adapter_num = ADAPTER_OUTPUT_1_5A;
					printk("%s:It is 5V/1.5A Charger\n", __func__);
				} else {
					BMT_status.adapter_num = ADAPTER_OUTPUT_2_0A;
					printk("%s:It is 5V/2A Charger\n", __func__);
				}
			} else {
				BMT_status.adapter_num = ADAPTER_OUTPUT_0A;
			}
			if (BMT_status.adapter_num == ADAPTER_OUTPUT_0A) {
				g_temp_input_CC_value = ADAPTER_OUTPUT_1_2A;
				g_temp_CC_value = ADAPTER_OUTPUT_1_2A;
			} else {
				g_temp_input_CC_value = BMT_status.adapter_num;
				g_temp_CC_value = BMT_status.adapter_num;
			}
		} else {
			g_temp_input_CC_value = BMT_status.adapter_num;
			g_temp_CC_value = BMT_status.adapter_num;
		}

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
		set_jeita_charging_current();
#endif
	}

	cv_voltage = BATTERY_VOLT_04_350000_V;
	battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE, &cv_voltage);
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
	int curr;

#if !defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
	BATTERY_VOLTAGE_ENUM cv_voltage;
#endif
	kal_uint32 charging_enable = KAL_TRUE;

	if (BMT_status.bat_charging_state == CHR_ERROR) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Charger Error, turn OFF charging !\n");

		charging_enable = KAL_FALSE;

	} else if ((g_platform_boot_mode == META_BOOT) || (g_platform_boot_mode == ADVMETA_BOOT)) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY] In meta or advanced meta mode, disable charging.\n");
		charging_enable = KAL_FALSE;
	} else {
		/*HW initialization */
		battery_charging_control(CHARGING_CMD_INIT, NULL);
		battery_xlog_printk(BAT_LOG_FULL, "charging_hw_init\n");
		/*get the VBUS and VBAT voltage to adjust the VINDPM THRESHOLD,
		 advoid to occour CHARGER IC enter sleep mode*/
		if (BMT_status.SOC < 80) {
			/*change the vindpm threshold*/
			bq2589x_set_input_volt_limit(0x11); //VIN DPM check 4.3V
		} else {
			bq2589x_set_input_volt_limit(0x13); //VIN DPM check 4.5V
		}

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	if (batt_temp_status == TEMP_POS_NORMAL || (batt_temp_status == TEMP_POS_T20
				&& g_changed_cv == 0)
			|| batt_temp_status == TEMP_POS_T15)
		battery_pump_express_algorithm_start();
#endif

		/* Set Charging Current */
		if (get_usb_current_unlimited()) {
			g_temp_input_CC_value = USB_CHARGER_INPUT_CURRENT;
			g_temp_CC_value = USB_CHARGER_ONLY_CURRENT;
			if (batt_temp_status == TEMP_POS_T10) {
				g_temp_input_CC_value = CHARGING_0_3C_CHARGER_CURRENT;
				g_temp_CC_value = CHARGING_0_3C_CHARGER_CURRENT;
			}
			battery_xlog_printk(BAT_LOG_FULL,
					    "USB_CURRENT_UNLIMITED, use AC_CHARGER_CURRENT\n");
		} else if (g_bcct_flag == 1 && BMT_status.charger_type != STANDARD_HOST) {
			select_charging_curret_bcct();
			g_bcct_flag = 0;
			printk("[BATTERY] select_charging_curret_bcct, batt_temp_status %d, batt_temp_pre_status %d!\n", 
					batt_temp_status, batt_temp_pre_status);
		} else if (is_ta_connect == KAL_TRUE) {
			set_ta_charging_current();
			battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT,
						 &g_temp_input_CC_value);
			battery_charging_control(CHARGING_CMD_SET_CURRENT, &g_temp_CC_value);

			printk("[BATTERY] set_ta_charging_current!");
		} else {
			select_charging_curret();
			printk("[BATTERY] select_charging_curret!");
		}
		if (usb_error_check == KAL_TRUE) {
			BMT_status.charger_vol = bq2589x_adc_read_charger_volt();
			if (BMT_status.charger_vol > 6000) {
				BMT_status.charger_type = STANDARD_CHARGER;
				ta_check_chr_type = KAL_TRUE;
				ta_cable_out_occur = KAL_FALSE;
			}
		}
#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
		if ((cmd_discharging == -1) && (batt_temp_status == TEMP_POS_NORMAL || (batt_temp_status == TEMP_POS_T20
					&& g_changed_cv == 0)
				|| batt_temp_status == TEMP_POS_T15))
			battery_pump_express_charger_check();
#endif
		printk("[BATTERY] Default CC mode charging : %d, input current = %d\r\n",
				    g_temp_CC_value, g_temp_input_CC_value);
		if (g_temp_CC_value == CHARGE_CURRENT_0_00_MA
		    || g_temp_input_CC_value == CHARGE_CURRENT_0_00_MA) {

			charging_enable = KAL_FALSE;

			battery_xlog_printk(BAT_LOG_CRTI,
					    "[BATTERY] charging current is set 0mA, turn off charging !\r\n");
		} else {
			battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT,
						 &g_temp_input_CC_value);
			battery_charging_control(CHARGING_CMD_SET_CURRENT, &g_temp_CC_value);
		}

		/* enable/disable charging */
		if (is_ta_connect == KAL_TRUE) {
			bq2589x_set_en_hiz_minor(0);
			battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
		} else {
			bq2589x_set_en_hiz_minor(1);
			battery_charging_control(CHARGING_CMD_ENABLE_MAJOR, &charging_enable);
		}

		BMT_status.chrg_stat = bq2589x_get_charging_status();
		if (BMT_status.chrg_stat == BQ2589X_CHARGE_DONE && BMT_status.SOC != 100) {
			printk("bq27532_sync_truesoc():RECHRG START\n");
			bq27532_sync_truesoc();
			charging_enable = KAL_FALSE;
			battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
			msleep(500);
			charging_enable = KAL_TRUE;
			bq2589x_enable_charger();
			msleep(500);
			battery_charging_control(CHARGING_CMD_ENABLE_MAJOR, &charging_enable);
			BMT_status.chrg_stat = BQ2589X_CHARGE_RECHG;
			curr = bq2589x_adc_read_charge_current1();
			if (curr == 0) {
				WARN(1, "********RECHRG ERROR***********\n");
			}
		}
	}
	battery_xlog_printk(BAT_LOG_FULL, "[BATTERY] pchr_turn_on_charging(), enable =%d !\r\n",
			    charging_enable);
}

PMU_STATUS BAT_PreChargeModeAction(void)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Pre-CC mode charge, timer=%d on %d !!\n\r",
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
	} else if (BMT_status.bat_vol > V_PRE2CC_THRES) {
		BMT_status.bat_charging_state = CHR_CC;
	}

	return PMU_STATUS_OK;
}


PMU_STATUS BAT_ConstantCurrentModeAction(void)
{
	battery_xlog_printk(BAT_LOG_FULL, "[BATTERY] CC mode charge, timer=%d on %d !!\n\r",
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
	}

	return PMU_STATUS_OK;
}


PMU_STATUS BAT_BatteryFullAction(void)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Battery full !!\n\r");

	BMT_status.bat_full = KAL_TRUE;
	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;
	BMT_status.bat_in_recharging_state = KAL_FALSE;

	if (charging_full_check() == KAL_FALSE) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Battery Re-charging !!\n\r");

		BMT_status.bat_in_recharging_state = KAL_TRUE;
		BMT_status.bat_charging_state = CHR_CC;
	}

	return PMU_STATUS_OK;
}


PMU_STATUS BAT_BatteryHoldAction(void)
{
	kal_uint32 charging_enable;

	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Hold mode !!\n\r");

	if (BMT_status.bat_vol < TALKING_RECHARGE_VOLTAGE || g_call_state == CALL_IDLE) {
		BMT_status.bat_charging_state = CHR_CC;
		battery_xlog_printk(BAT_LOG_CRTI,
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

	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BAD Battery status... Charging Stop !!\n\r");

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

static void mt_bcct_check(void)
{
	kal_uint32 charging_enable = KAL_FALSE;
	kal_int32 bat_vol;

	/* set the temperature charging control */
	get_batt_temp_status(&batt_temp_status);
	if ((BMT_status.bat_charging_state != CHR_ERROR) &&
			(batt_temp_status != TEMP_POS_NORMAL)) {
	    g_bcct_flag = 1;
	}
	if (batt_temp_status == TEMP_POS_T20) {
		bat_vol = bq27532_battery_voltage();	
		if (bat_vol > 4200) {
			g_changed_cv = 1;
		} else {
			g_changed_cv = 0;
			cv_changed_flag = KAL_FALSE;
		}
	} else {
		g_changed_cv = 0;
		cv_changed_flag = KAL_FALSE;
	}

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	if ((batt_temp_pre_status != batt_temp_status) &&
			(batt_temp_status == TEMP_POS_NORMAL ||
			 (batt_temp_status == TEMP_POS_T20 && g_changed_cv == 0)
			|| batt_temp_status == TEMP_POS_T15)) {
		 is_ta_connect = KAL_FALSE;    
		 ta_check_chr_type = KAL_TRUE;
		 ta_cable_out_occur = KAL_FALSE;
	} else if (((batt_temp_pre_status != batt_temp_status) &&
			(batt_temp_status == TEMP_POS_T10 ||
			 batt_temp_status == TEMP_POS_T42)) ||
			((g_changed_cv == 1) && cv_changed_flag == KAL_FALSE) ||
			(BMT_status.mcharger_enable == NORMAL_CHARGER)) {
		 is_ta_connect = KAL_FALSE; 
		 if (g_changed_cv == 1)
			 cv_changed_flag = KAL_TRUE;
		/* disable charging, avoid Iterm issue */
		battery_charging_control(CHARGING_CMD_ENABLE,&charging_enable);
		mtk_ta_reset_vchr();
	 }

	batt_temp_pre_status = batt_temp_status;
#endif
}

void mt_battery_charging_algorithm(void)
{
	kal_uint32 charger_fault_status = KAL_FALSE;
	kal_uint32 charging_enable = KAL_FALSE;

	charger_fault_status = bq2589x_charger_fault_status();
	if (charger_fault_status == KAL_TRUE) {
		bq2589x_set_chargevoltage(0x20); //VREG 4.352V
		charging_enable = KAL_FALSE;
		battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
		msleep(500);
		charging_enable = KAL_TRUE;
		battery_charging_control(CHARGING_CMD_ENABLE_MAJOR, &charging_enable);
		charger_fault_status = KAL_FALSE;
		bq2589x_charger_fault_reset(&charger_fault_status);
	}
	mt_bcct_check();

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	if ((cmd_discharging == -1) && (batt_temp_status == TEMP_POS_NORMAL || (batt_temp_status == TEMP_POS_T20
				&& g_changed_cv == 0) || batt_temp_status == TEMP_POS_T15)
			&& (BMT_status.mcharger_enable == QUICK_CHARGER))
		battery_pump_express_charger_check();
#endif
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
