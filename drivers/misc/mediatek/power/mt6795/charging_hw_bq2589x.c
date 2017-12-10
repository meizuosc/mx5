/*****************************************************************************
 *
 * Filename:
 * ---------
 *    charging_pmic.c
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
#include <mach/charging.h>
#include <mach/upmu_common.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <mach/upmu_hw.h>
#include <linux/xlog.h>
#include <linux/delay.h>
#include <mach/mt_sleep.h>
#include <mach/mt_boot.h>
#include <mach/system.h>
#include <cust_charging.h>
#include <mach/upmu_common.h>
#include <mach/pmic_mt6331_6332_sw.h>
#include <linux/tsu6721-muic.h>
#include <mach/bq2589x_reg.h>


#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))

kal_bool charging_type_det_done = KAL_TRUE;
extern kal_uint32 upmu_get_rgs_chrdet(void);
extern kal_uint32 upmu_get_reg_value(kal_uint32 reg);

extern void mt_power_off(void );	

static kal_uint32 g_wake_reason_is_none = 0;
static kal_uint32 charging_error = false;
static kal_uint32 charging_get_error_state(void);
static kal_uint32 charging_set_error_state(void *data);
kal_bool gFG_Is_Charging = KAL_FALSE;

#ifdef MTK_PUMP_EXPRESS_SUPPORT
extern kal_bool is_ta_check_done;
#endif

const kal_uint32 VBAT_CV_VTH[]=
{
	3840000,    3856000,    3872000,    3888000,
	3904000,    3920000,    3936000,    3952000,
	3968000,    3984000,    4000000,    4016000,
	4032000,    4048000,    4064000,    4080000,
	4096000,    4112000,    4128000,    4144000,
	4160000,    4176000,    4192000,    4208000,
	4224000,    4240000,    4256000,    4272000,
	4288000,    4304000,    4320000,    4336000,
	4352000,    4368000,    4384000,    4400000,
	4416000,    4432000,    4448000,    4464000,
	4480000,    4496000,    4512000,    4528000,
	4544000,    4560000,    4576000,    4592000,
	4608000
};

const kal_uint32 CS_VTH[]=
{
	0,	6400,	12800,	19200,
	25600,  51200,  57600,  64000,
	70400,  76800,  83200,  89600,
	96000,  102400, 108800, 115200,
	121600, 128000, 134400, 140800,
	147200, 153600, 160000, 166400,
	172800, 179200, 185600, 192000,
	198400, 204800, 211200, 217600,
	224000, 230400, 236800, 243200,
	249600
}; 

 const kal_uint32 INPUT_CS_VTH[]=
 {
	 CHARGE_CURRENT_100_00_MA,  CHARGE_CURRENT_200_00_MA,	CHARGE_CURRENT_300_00_MA,  CHARGE_CURRENT_400_00_MA,
	 CHARGE_CURRENT_500_00_MA,  CHARGE_CURRENT_600_00_MA,	CHARGE_CURRENT_700_00_MA,  CHARGE_CURRENT_800_00_MA,
	 CHARGE_CURRENT_900_00_MA,  CHARGE_CURRENT_1000_00_MA,	CHARGE_CURRENT_1100_00_MA,  CHARGE_CURRENT_1200_00_MA,
	 CHARGE_CURRENT_1300_00_MA,  CHARGE_CURRENT_1400_00_MA,	CHARGE_CURRENT_1500_00_MA,  CHARGE_CURRENT_1600_00_MA,
	 CHARGE_CURRENT_1700_00_MA, CHARGE_CURRENT_1800_00_MA,  CHARGE_CURRENT_1900_00_MA,  CHARGE_CURRENT_2000_00_MA,
	 CHARGE_CURRENT_2100_00_MA, CHARGE_CURRENT_2200_00_MA,  CHARGE_CURRENT_2300_00_MA,  CHARGE_CURRENT_2400_00_MA,
	 CHARGE_CURRENT_2500_00_MA, CHARGE_CURRENT_2600_00_MA,  CHARGE_CURRENT_2700_00_MA,  CHARGE_CURRENT_2800_00_MA,
	 CHARGE_CURRENT_2900_00_MA, CHARGE_CURRENT_3000_00_MA,  CHARGE_CURRENT_3100_00_MA
 }; 

 const kal_uint32 VCDT_HV_VTH[]=
 {
	  BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V,	  BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_350000_V,
	  BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V,	  BATTERY_VOLT_04_500000_V,   BATTERY_VOLT_04_550000_V,
	  BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V,	  BATTERY_VOLT_06_500000_V,   BATTERY_VOLT_07_000000_V,
	  BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V,	  BATTERY_VOLT_09_500000_V,   BATTERY_VOLT_10_500000_V		  
 };

 extern bool mt_usb_is_device(void);
 
 // ============================================================ //
 kal_uint32 charging_value_to_parameter(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	if (val < array_size)
	{
		return parameter[val];
	}
	else
	{
		battery_xlog_printk(BAT_LOG_CRTI, "Can't find the parameter \r\n");	
		return parameter[0];
	}
}

 
 kal_uint32 charging_parameter_to_value(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	kal_uint32 i;

    battery_xlog_printk(BAT_LOG_CRTI, "array_size = %d \r\n", array_size);
    
	for(i=0;i<array_size;i++)
	{
		if ((val > *(parameter + i)) && (val < *(parameter + i +1)))
		{
			return i + 1;
		} else if (val == *(parameter + i)) {
			return i;
	    	}
	}

    battery_xlog_printk(BAT_LOG_CRTI, "NO register value match. val=%d\r\n", val);
	//TODO: ASSERT(0);	// not find the value
	return 0;
}

 static kal_uint32 bmt_find_closest_level(const kal_uint32 *pList,kal_uint32 number,kal_uint32 level)
 {
	 kal_uint32 i;
	 kal_uint32 max_value_in_last_element;
 
	 if(pList[0] < pList[1])
		 max_value_in_last_element = KAL_TRUE;
	 else
		 max_value_in_last_element = KAL_FALSE;
 
	 if(max_value_in_last_element == KAL_TRUE)
	 {
		 for(i = (number-1); i != 0; i--)	 //max value in the last element
		 {
			 if(pList[i] <= level)
			 {
				 return pList[i];
			 }	  
		 }

 		 battery_xlog_printk(BAT_LOG_CRTI, "Can't find closest level, small value first \r\n");
		 return pList[0];
		 //return CHARGE_CURRENT_0_00_MA;
	 }
	 else
	 {
		 for(i = 0; i< number; i++)  // max value in the first element
		 {
			 if(pList[i] <= level)
			 {
				 return pList[i];
			 }	  
		 }

		 battery_xlog_printk(BAT_LOG_CRTI, "Can't find closest level, large value first \r\n"); 	 
		 return pList[number -1];
  		 //return CHARGE_CURRENT_0_00_MA;
	 }
 }

 static kal_uint32 charging_hw_init(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	
	mt6332_upmu_set_rg_bc12_bb_ctrl(1);    //BC11_BB_CTRL    
    	mt6332_upmu_set_rg_bc12_rst(1);        //BC11_RST

	bq2589x_adc_start(0x0);
	bq2589x_force_vindpm(0x1);
	bq2589x_set_sys_min(0x5); //Minimum system voltage 3.2V	
	bq2589x_set_iprechg(0x7); //Precharge current 512mA
	bq2589x_set_iterm(0x2); //Termination current 192mA
	bq2589x_set_chargevoltage(0x20); //VREG 4.352V
	bq2589x_set_batlowv(0x1); //BATLOWV 3.0V
	bq2589x_set_vrechg(0x0); //VRECHG 0.1V (4.108V)
	bq2589x_set_en_term(0x1); //Enable termination
	bq2589x_set_en_timer(0x1); //Disable charge timer
	bq2589x_set_ir_comp_resistance(0x2);//ir comp 40毫欧
	//the max ir compensation voltage(above Charge Voltage Limit):120MV
	bq2589x_set_ir_comp_voltage_clamp(0x4); 

	return status;
}

static kal_uint32 charging_dump_register(void *data)
 {
 	kal_uint32 status = STATUS_OK;

	battery_xlog_printk(BAT_LOG_CRTI, "charging_dump_register\r\n");

	bq2589x_dump_register();
   	
	return status;
 }

void  tbl_charger_otg_vbus(kal_uint32 mode)
{
	kal_uint32 state = mode&0x1;

	if(state){
	//	mt_set_gpio_out(GPIO_OTG_DRVVBUS_PIN, GPIO_OUT_ONE);
		bq2589x_enable_otg();
	}else{
		bq2589x_disable_otg();
	//	mt_set_gpio_out(GPIO_OTG_DRVVBUS_PIN, GPIO_OUT_ZERO);  
	}
}
EXPORT_SYMBOL(tbl_charger_otg_vbus);

 static kal_uint32 charging_enable_major(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32*)(data);

	if(KAL_TRUE == enable)
	{
		bq2589x_stop_charging1();
		bq2589x_start_charging();//I2C2
	}
	else
	{
		bq2589x_stop_charging();
		bq2589x_stop_charging1();
	}

	return status;
 }

 static kal_uint32 charging_enable_minor(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32*)(data);

	if(KAL_TRUE == enable)
	{
		bq2589x_stop_charging();
		bq2589x_start_charging1();//I2C2
	}
	else
	{
		bq2589x_stop_charging();
		bq2589x_stop_charging1();
	}

	return status;
 }

 static kal_uint32 charging_enable(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32*)(data);

	if(KAL_TRUE == enable)
	{
		printk("%s:*************************\n", __func__);
		bq2589x_start_charging();
		bq2589x_start_charging1();
	}
	else
	{
		bq2589x_stop_charging();
		bq2589x_stop_charging1();
	}

	return status;
 }

 static kal_uint32 charging_set_cv_voltage(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint16 register_value;
	kal_uint32 cv_value = *(kal_uint32 *)(data);	
	
	if(cv_value == BATTERY_VOLT_04_200000_V)
	{
	    //use nearest value
		cv_value = 4208000;
 	} 

	register_value = charging_parameter_to_value(VBAT_CV_VTH, GETARRAYNUM(VBAT_CV_VTH), cv_value);
	bq2589x_set_chargevoltage(register_value);

	return status;
 } 	

 static kal_uint32 charging_get_current(void *data)
 {
    kal_uint32 status = STATUS_OK;
    kal_uint8 ret_val=0;    
	   
    ret_val = bq2589x_adc_read_charge_current();

    *(kal_uint32 *)data = ret_val;
   	
    return status;
 }  

 static kal_uint32 charging_set_current(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 current_value = *(kal_uint32 *)data;

	bq2589x_set_chargecurrent(current_value);
	bq2589x_set_chargecurrent1(current_value);

	return status;
 } 	

 static kal_uint32 charging_set_input_current(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_current = *(kal_uint32 *)data;
    
	bq2589x_set_input_current_limit(set_chr_current);
	bq2589x_set_input_current_limit1(set_chr_current);

	return status;
 } 	

 static kal_uint32 charging_get_charging_status(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 ret_val;

	ret_val = bq2589x_get_charging_status();
	
	/* 00:NOT CHARGING*/
	if(ret_val)
		*(kal_uint32 *)data = KAL_TRUE;
	else
		*(kal_uint32 *)data = KAL_FALSE;
	
	return status;
 } 	

 static kal_uint32 charging_reset_watch_dog_timer(void *data)
 {
	 kal_uint32 status = STATUS_OK;
	 kal_uint32 rest_enable = *(kal_uint32*)(data);
 
     battery_xlog_printk(BAT_LOG_CRTI, "charging_reset_watch_dog_timer\r\n");
 
     if (rest_enable == KAL_TRUE) {
	     bq2589x_set_watchdog(BQ2589X_WDT_160S);
	     bq2589x_set_wdt_rst();
     } else {
	     bq2589x_set_watchdog(BQ2589X_WDT_DISABLE);
     }

     return status;
 }
 
  static kal_uint32 charging_set_hv_threshold(void *data)
  {
	 kal_uint32 status = STATUS_OK;
 
	 return status;
  }
 
  static kal_uint32 charging_get_hv_status(void *data)
  {
	   kal_uint32 status = STATUS_OK;
 
	   *(kal_bool*)(data) = mt6332_upmu_get_rgs_chr_hv_det();
	   
	   return status;
  }

 static kal_uint32 charging_get_battery_status(void *data)
 {
	   kal_uint32 status = STATUS_OK;
 
 	   mt6332_upmu_set_rg_baton_tdet_en(1);	
	   mt6332_upmu_set_rg_baton_en(1);
	   *(kal_bool*)(data) = mt6332_upmu_get_rg_int_status_vbaton_undet();
	   
	   return status;
 }

 static kal_uint32 charging_get_charger_det_status(void *data)
 {
	   kal_uint32 status = STATUS_OK;
	   kal_bool pg_stat = KAL_FALSE;
	   kal_int32 val = 0;
	   kal_bool det_stat = KAL_FALSE;

	   pmic_config_interface(0x10A, 0x1, 0xF, 8);
	   pmic_config_interface(0x10A, 0x17,0xFF,0);
	   pmic_read_interface(0x108,   &val,0x1, 1);
  
	   pg_stat = bq2589x_get_pg_stat();
	   if (val == 1 || pg_stat == 1) {
		det_stat = KAL_TRUE; 
	   } else {
		det_stat = KAL_FALSE;
	   }
	   *(kal_bool*)(data) = det_stat; 

	   if (pg_stat == KAL_TRUE) {
		gFG_Is_Charging = KAL_TRUE;
	   } else {
		gFG_Is_Charging = KAL_FALSE;
	   }

	   return status;
 }

kal_bool charging_type_detection_done(void)
{
	 return charging_type_det_done;
}

 static kal_uint32 charging_get_charger_type(void *data)
 {
	 kal_uint32 status = STATUS_OK;
	 CHARGER_TYPE charger_type = CHARGER_UNKNOWN;
#if defined(CONFIG_POWER_EXT)
	 *(CHARGER_TYPE*)(data) = STANDARD_HOST;
#else
	 tsu6721_charger_type(&charger_type);
	*(CHARGER_TYPE*)(data) = charger_type;
	printk("%s:*********charger_type %d,%d\n", __func__, charger_type, *(CHARGER_TYPE*)(data));
	charging_type_det_done = KAL_TRUE;
#endif    
	 return status;
}

static kal_uint32 charging_get_is_pcm_timer_trigger(void *data)
{
    kal_uint32 status = STATUS_OK;

#if 1
    if(slp_get_wake_reason() == WR_PCM_TIMER)
    {
        g_wake_reason_is_none = 0;
        *(kal_bool*)(data) = KAL_TRUE;
    }
    else if((slp_get_wake_reason() == WR_NONE) || (slp_get_wake_reason() == WR_WAKE_SRC))
    {
        if (g_wake_reason_is_none < 20) {
            g_wake_reason_is_none++;
            *(kal_bool*)(data) = KAL_FALSE;
        }
        else
        {
            g_wake_reason_is_none = 0;
            *(kal_bool*)(data) = KAL_TRUE;
        }
    }
    else
        *(kal_bool*)(data) = KAL_FALSE;

    battery_log(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n", slp_get_wake_reason());
#else
    *(kal_bool*)(data) = KAL_FALSE;
#endif

    return status;
}

static kal_uint32 charging_set_platform_reset(void *data)
{
    kal_uint32 status = STATUS_OK;

    battery_xlog_printk(BAT_LOG_CRTI, "charging_set_platform_reset\n");
 
    arch_reset(0,NULL);
        
    return status;
}

static kal_uint32 charging_get_platfrom_boot_mode(void *data)
{
    kal_uint32 status = STATUS_OK;
  
    *(kal_uint32*)(data) = get_boot_mode();

    battery_xlog_printk(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());
         
    return status;
}

static kal_uint32 charging_set_power_off(void *data)
{
    kal_uint32 status = STATUS_OK;
  
    battery_xlog_printk(BAT_LOG_CRTI, "charging_set_power_off\n");
    mt_power_off();
         
    return status;
}

static kal_uint32 charging_get_power_source(void *data)
{
	return STATUS_UNSUPPORTED;
}

static kal_uint32 charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;	
}

static void swchr_flow_normal(kal_uint32 chr_cur_val)
{
	bq2589x_stop_charging1(); //STOP THE I2C0 bq25892 charger IC
	bq2589x_start_charging(); //ENABLE I2C2

	bq2589x_set_input_current_limit(chr_cur_val);
}

static void ta_set_chr_current(kal_uint32 chr_current)
{
  	swchr_flow_normal(chr_current);

	battery_xlog_printk(BAT_LOG_CRTI,"[0x%x]=0x%x,[0x%x]=0x%x\n", 
        0x8068, upmu_get_reg_value(0x8068),
        0x8078, upmu_get_reg_value(0x8078)
        );
}

static kal_uint32 charging_set_ta_current_pattern(void *data)
{
	kal_uint32 status = STATUS_OK;
	kal_uint32 array_size; 
	kal_uint32 set_ta_on_current_reg_value = CHARGE_CURRENT_900_00_MA;
	kal_uint32 set_ta_off_current_reg_value= CHARGE_CURRENT_100_00_MA;
	kal_uint32 increase = *(kal_uint32*)(data);

	//CURRENT is higher than 550mA is "1"
	//CURRENT is lower than 130mA is "0"

	pmic_config_interface(0x8D36,0x1,0x1,11); //[11]=0x1
	battery_xlog_printk(BAT_LOG_CRTI, "[charging_set_ta_current_pattern] [0x%x]=0x%x\n", 
	0x8D36, upmu_get_reg_value(0x8D36)
	);

	if(increase == KAL_TRUE)
	{
	 printk("mtk_ta_increase() start (%d), on %d\n", set_ta_off_current_reg_value, set_ta_on_current_reg_value);

	 ta_set_chr_current(set_ta_off_current_reg_value); 
	 msleep(100);

	 // patent start
	 ta_set_chr_current(set_ta_on_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_increase() on 1\n");
	 msleep(100);
	 ta_set_chr_current(set_ta_off_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_increase() off 1\n");
	 msleep(100);

	 
	 ta_set_chr_current(set_ta_on_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_increase() on 2\n");
	 msleep(100);
	 ta_set_chr_current(set_ta_off_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_increase() off 2\n");
	 msleep(100);


	 ta_set_chr_current(set_ta_on_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_increase() on 3\n");
	 msleep(300);
	 ta_set_chr_current(set_ta_off_current_reg_value);
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_increase() off 3\n");
	 msleep(100);

	 
	 ta_set_chr_current(set_ta_on_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_increase() on 4\n");
	 msleep(300);
	 ta_set_chr_current(set_ta_off_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_increase() off 4\n");
	 msleep(100);

	 
	 ta_set_chr_current(set_ta_on_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_increase() on 5\n");
	 msleep(300);
	 ta_set_chr_current(set_ta_off_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_increase() off 5\n");
	 msleep(100);


	 ta_set_chr_current(set_ta_on_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_increase() on 6\n");
	 msleep(500);
	 // patent end

	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_increase() end \n");
	}
	else    //decrease
	{
	 printk( "mtk_ta_decrease() start\n");

	 ta_set_chr_current(set_ta_off_current_reg_value); 
	 msleep(100);

	 // patent start    
	 ta_set_chr_current(set_ta_on_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_decrease() on 1");
	 msleep(300);
	 ta_set_chr_current(set_ta_off_current_reg_value);
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_decrease() off 1");
	 msleep(100);
	 
	 
	 ta_set_chr_current(set_ta_on_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_decrease() on 2");
	 msleep(300);
	 ta_set_chr_current(set_ta_off_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_decrease() off 2");
	 msleep(100);

	 
	 ta_set_chr_current(set_ta_on_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_decrease() on 3");
	 msleep(300);
	 ta_set_chr_current(set_ta_off_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_decrease() off 3");
	 msleep(100);

	  
	 ta_set_chr_current(set_ta_on_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_decrease() on 4");
	 msleep(100);
	 ta_set_chr_current(set_ta_off_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_decrease() off 4");
	 msleep(100);

	 
	 ta_set_chr_current(set_ta_on_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_decrease() on 5");
	 msleep(100);
	 ta_set_chr_current(set_ta_off_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_decrease() off 5");
	 msleep(100);

	 ta_set_chr_current(set_ta_on_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_decrease() on 6");
	 msleep(500);        

	 ta_set_chr_current(set_ta_off_current_reg_value); 
	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_decrease() off 6");
	 msleep(50);
	 // patent end

	 battery_xlog_printk(BAT_LOG_CRTI, "mtk_ta_decrease() end \n"); 

//	 ta_set_chr_current(set_ta_on_current_reg_value); 
	}

	return status;
}

static kal_uint32 charging_get_error_state(void)
{
	return charging_error;
}

static kal_uint32 charging_set_error_state(void *data)
{
	kal_uint32 status = STATUS_OK;
	charging_error = *(kal_uint32*)(data);
	
	return status;
}

static kal_uint32 charging_set_onlyone_input_current(void *data)
{
 	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_current = *(kal_uint32 *)data;
    
	bq2589x_stop_charging1();
	bq2589x_start_charging();//I2C2
	bq2589x_set_input_current_limit(set_chr_current);

	return status;
}

static kal_uint32 charging_set_adc_start(void *data)
{
 	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32*)(data);

	bq2589x_adc_start(enable);

	return status;
}

static kal_uint32 charging_set_adc_stop(void *data)
{
 	kal_uint32 status = STATUS_OK;

	bq2589x_adc_stop();

	return status;
}

 static kal_uint32 (* const charging_func[CHARGING_CMD_NUMBER])(void *data)=
 {
 	  charging_hw_init
	,charging_dump_register  	
	,charging_enable
	,charging_set_cv_voltage
	,charging_get_current
	,charging_set_current
	,charging_set_input_current
	,charging_get_charging_status
	,charging_reset_watch_dog_timer
	,charging_set_hv_threshold
	,charging_get_hv_status
	,charging_get_battery_status
	,charging_get_charger_det_status
	,charging_get_charger_type
	,charging_get_is_pcm_timer_trigger
	,charging_set_platform_reset
	,charging_get_platfrom_boot_mode
        ,charging_set_power_off
	,charging_get_power_source
	,charging_get_csdac_full_flag
	,charging_set_ta_current_pattern
	,charging_set_error_state
	,charging_set_onlyone_input_current
	,charging_set_adc_start
	,charging_set_adc_stop
	,charging_enable_major
	,charging_enable_minor
 };

 /*
 * FUNCTION
 *		Internal_chr_control_handler
 *
 * DESCRIPTION															 
 *		 This function is called to set the charger hw
 *
 * CALLS  
 *
 * PARAMETERS
 *		None
 *	 
 * RETURNS
 *		
 *
 * GLOBALS AFFECTED
 *	   None
 */
 kal_int32 chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
 {
	 kal_int32 status;
	 if(cmd < CHARGING_CMD_NUMBER)
		 status = charging_func[cmd](data);
	 else
		 return STATUS_UNSUPPORTED;
 
	 return status;
 }


