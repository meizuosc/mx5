#include <linux/kernel.h>
#include <mach/eint.h>
#include <mach/mt_gpio.h>
#include "cust_gpio_usage.h"
#include "cust_eint.h"
#include <cust_pmic.h>
#include <mach/system.h>
#include "mach/mtk_thermal_monitor.h"
#include "mach/mt_typedefs.h"
#include "mach/mt_thermal.h"

#include <cust_pmic.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>
#include <mach/mt_pmic_wrap.h>
#include <mach/pmic_mt6331_6332_sw.h>

#include <mach/system.h>
#include "mach/mtk_thermal_monitor.h"
#include "mach/mt_typedefs.h"
#include "mach/mt_thermal.h"

#include "usb_thermal.h"

extern int pwork_state;
extern struct mutex usb_temp_mutex;
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern int IMM_IsAdcInitReady(void);

typedef struct{
    INT32 APTemp;
    INT32 TemperatureR;
}AP_TEMPERATURE;

#define HIGH GPIO_OUT_ONE
#define LOW  GPIO_OUT_ZERO

void usb_thermal_gpio_init(void) 
{
	/*
	 * default: gpio low
	 */
	mt_set_gpio_mode(GPIO_USB_EN_PIN, GPIO_USB_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_USB_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_USB_EN_PIN, GPIO_OUT_ONE);
}
EXPORT_SYMBOL(usb_thermal_gpio_init);


void usb_thermal_gpio_set(unsigned char value) 
{
	mutex_lock(&usb_temp_mutex);
	if( !!value ) {
//		if( pwork_state==ON )
		{
			mt_set_gpio_out(GPIO_USB_EN_PIN, GPIO_OUT_ONE);
		}
	} else {
		mt_set_gpio_out(GPIO_USB_EN_PIN, GPIO_OUT_ZERO);
	}
	mutex_unlock(&usb_temp_mutex);
}
EXPORT_SYMBOL(usb_thermal_gpio_set);

int usb_thermal_gpio_get(void)
{
	mt_get_gpio_out(GPIO_USB_EN_PIN);
}
EXPORT_SYMBOL(usb_thermal_gpio_get);


//NTCG063JF103(10K)
AP_TEMPERATURE AP_Temperature_Table[] = {
	    {-40,188500},
	    {-35,144290},
	    {-30,111330},
		{-25,86560},
	    {-20,67790},
	    {-15,53460},
	    {-10,42450},
	    { -5,33930},
	    {  0,27280},
	    {  5,22070},
	    { 10,17960},
	    { 15,14700},
	    { 20,12090},
	    { 25,10000},//10K
	    { 30,8310},
	    { 35,6940},
	    { 40,5830},
	    { 45,4910},
	    { 50,4160},
	    { 55,3540},
	    { 60,3020},
	    { 65,2590},
	    { 70,2230},
	    { 75,1920},
	    { 80,1670},
	    { 85,1450},
	    { 90,1270},
	    { 95,1110},
	    { 100,975},
	    { 105,860},
	    { 110,760},
	    { 115,674},
	    { 120,599},
	    { 125,534}
};
static int g_RAP_pull_up_R = 39000;//39K,pull up resister
static int g_TAP_over_critical_low = 188500;//base on 10K NTC temp default value -40 deg
static int g_RAP_pull_up_voltage = 1800;//1.8V ,pull up voltage
static int g_AP_TemperatureR = 0;

/* convert register to temperature  */
static INT16 APtThermistorConverTemp(INT32 Res)
{
    int i=0;
    int asize=0;
    INT32 RES1=0,RES2=0;
    INT32 TAP_Value=-200,TMP1=0,TMP2=0;

	asize=(sizeof(AP_Temperature_Table)/sizeof(AP_TEMPERATURE));
	//xlog_printk(ANDROID_LOG_INFO, "Power/AP_Thermal", "APtThermistorConverTemp() : asize = %d, Res = %d\n",asize,Res);
    if(Res>=AP_Temperature_Table[0].TemperatureR)
    {
        TAP_Value = -40;//min
        INFOR("temp less than min, err! Res(%d)>max(%d)\n", Res, AP_Temperature_Table[0].TemperatureR);
    }
    else if(Res<=AP_Temperature_Table[asize-1].TemperatureR)
    {
        TAP_Value = 125;//max
        INFOR("temp more than max, err! Res(%d)<min(%d)\n", Res, AP_Temperature_Table[asize-1].TemperatureR);
    }
    else
    {
        RES1=AP_Temperature_Table[0].TemperatureR;
        TMP1=AP_Temperature_Table[0].APTemp;
		//xlog_printk(ANDROID_LOG_INFO, "Power/AP_Thermal", "%d : RES1 = %d,TMP1 = %d\n",__LINE__,RES1,TMP1);

        for(i=0;i < asize;i++)
        {
            if(Res>=AP_Temperature_Table[i].TemperatureR)
            {
                RES2=AP_Temperature_Table[i].TemperatureR;
                TMP2=AP_Temperature_Table[i].APTemp;
                //xlog_printk(ANDROID_LOG_INFO, "Power/AP_Thermal", "%d :i=%d, RES2 = %d,TMP2 = %d\n",__LINE__,i,RES2,TMP2);
                break;
            }
            else
            {
                RES1=AP_Temperature_Table[i].TemperatureR;
                TMP1=AP_Temperature_Table[i].APTemp;
                //xlog_printk(ANDROID_LOG_INFO, "Power/AP_Thermal", "%d :i=%d, RES1 = %d,TMP1 = %d\n",__LINE__,i,RES1,TMP1);
            }
        }

        TAP_Value = (((Res-RES2)*TMP1)+((RES1-Res)*TMP2))/(RES1-RES2);
    }

    INFOR("TAP_Value:%d,Res:%d,RES1:%d,RES2:%d,TMP1:%d,TMP2:%d\n",TAP_Value,Res,RES1,RES2,TMP1,TMP2);
    return TAP_Value;
}

/* convert ADC_AP_temp_volt to register */
/*Volt to Temp formula same with 6589*/
static INT16 APtVoltToTemp(UINT32 dwVolt)
{
    INT32 TRes;
    INT32 dwVCriAP = 0;
    INT32 APTMP = -100;

    //SW workaround-----------------------------------------------------
    //dwVCriAP = (TAP_OVER_CRITICAL_LOW * 1800) / (TAP_OVER_CRITICAL_LOW + 39000);
    //dwVCriAP = (TAP_OVER_CRITICAL_LOW * RAP_PULL_UP_VOLT) / (TAP_OVER_CRITICAL_LOW + RAP_PULL_UP_R);
    dwVCriAP = (g_TAP_over_critical_low * g_RAP_pull_up_voltage) / (g_TAP_over_critical_low + g_RAP_pull_up_R);//1491

    if(dwVolt > dwVCriAP)
    {
        TRes = g_TAP_over_critical_low;
    }
    else
    {
        //TRes = (39000*dwVolt) / (1800-dwVolt);
       // TRes = (RAP_PULL_UP_R*dwVolt) / (RAP_PULL_UP_VOLT-dwVolt);
        TRes = (g_RAP_pull_up_R*dwVolt) / (g_RAP_pull_up_voltage-dwVolt);
    }
    //------------------------------------------------------------------

    g_AP_TemperatureR = TRes;

    /* convert register to temperature */
    APTMP = APtThermistorConverTemp(TRes);

    return APTMP;
}

extern int PMIC_IMM_GetOneChannelValue(upmu_adc_chl_list_enum dwChannel, int deCount, int trimd);
int usb_thermal_get_temp(void)
{
	int temp = 0, repeat_times = 1, volt = 0, data[4] = {0,}, ret_value;


#if 0
	if( IMM_IsAdcInitReady() == 0 )
	{
        INFOR("AUXADC is not ready\n");
		return 0;
	}
	ret_value = IMM_GetOneChannelValue(12, data, &temp);
	if( ret_value ) {
		INFOR("get adc value failed\n");
	}

#else
	temp = PMIC_IMM_GetOneChannelValue(AUX_ADCVIN0_AP , repeat_times , 2);// ADC_ADCVIN0_AP;
#endif
	INFOR("temp:%d  raw\n", temp);
//	volt = temp*1800/4096; // adc:12bit; vdd:1800mv
	volt = temp*3200/32768;// adc:15bit; input range:3200mV
//	temp = pmic_raw_to_temp(volt);
	INFOR("volt:%d mV\n", volt);

	temp = APtVoltToTemp(volt);
	INFOR("temp:%d °C\n", temp);
	return temp;
}
EXPORT_SYMBOL(usb_thermal_get_temp);





















