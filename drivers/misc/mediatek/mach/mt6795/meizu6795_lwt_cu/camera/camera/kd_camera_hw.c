#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         xlog_printk(ANDROID_LOG_ERR, PFX , fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                   xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##args); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif

extern void ISP_MCLK1_EN(BOOL En);
extern void ISP_MCLK2_EN(BOOL En);
extern void ISP_MCLK3_EN(BOOL En);
extern void vcm_smooth_disable(char *mode_name);
extern void vcm_smooth_enable(char *mode_name);

#define IDX_PS_ARRAY 4
#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3

u32 flash_pinset[13] = {
	SUB_FLASH_EN_PIN,
	SUB_FLASH_EN_PIN_M_GPIO,
	GPIO_OUT_ONE,              /* ON state */
	GPIO_OUT_ZERO,             /* OFF state */
	SUB_FLASH_STROBE_PIN,
	SUB_FLASH_STROBE_PIN_M_GPIO,
	GPIO_OUT_ONE,              /* ON state */
	GPIO_OUT_ZERO,             /* OFF state */
	SUB_FLASH_TORCH_PIN,
	SUB_FLASH_TORCH_PIN_M_GPIO,
	GPIO_OUT_ONE,              /* ON state */
	GPIO_OUT_ZERO,             /* OFF state */
	GPIO_CAMERA_INVALID
};

int flash_gpio_en(flash_gpio gpio_pin, u8 on)
{
	u32 pin_idx = gpio_pin * IDX_PS_ARRAY;
	if (pin_idx > (GPIO_FLASH_NUM * IDX_PS_ARRAY)) {
		PK_DBG("[CAMERA SENSOR] Invalid flash gpio pin.\n");
		return -EINVAL;
	}

	if (on) {
		if (GPIO_CAMERA_INVALID != flash_pinset[pin_idx]) {
			if(mt_set_gpio_mode(flash_pinset[pin_idx], flash_pinset[pin_idx+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(flash_pinset[pin_idx], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(flash_pinset[pin_idx], flash_pinset[pin_idx+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		}
	} else {
		if (GPIO_CAMERA_INVALID != flash_pinset[pin_idx]) {
			if(mt_set_gpio_mode(flash_pinset[pin_idx], flash_pinset[pin_idx+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(flash_pinset[pin_idx], GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(flash_pinset[pin_idx], flash_pinset[pin_idx+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		}
	}
	return 0;
}

/*
 * Camera VCM device power on func.
 */
int camera_af_poweron(char *mode_name, BOOL on)
{
	if (on) {
                if(mt_set_gpio_mode(CAMERA_CMPDN_PIN, CAMERA_CMPDN_PIN_M_GPIO)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(CAMERA_CMPDN_PIN, GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(CAMERA_CMPDN_PIN, GPIO_OUT_ONE)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to enable AF power(%d)\n",CAMERA_POWER_VCAM_A2);
			return -EIO;
		}
	} else {
                if(mt_set_gpio_mode(CAMERA_CMPDN_PIN, CAMERA_CMPDN_PIN_M_GPIO)){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(CAMERA_CMPDN_PIN, GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(CAMERA_CMPDN_PIN, GPIO_OUT_ZERO)){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
		{
			PK_DBG("[CAMERA SENSOR] Fail to disable AF power(%d)\n",CAMERA_POWER_VCAM_A2);
			return -EIO;
		}
	}

	return 0;
}

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{

u32 pinSetIdx = 0;//default main sensor
u32 tmp_idx = 0;

u32 pinSet[3][8] = {
                        //for main sensor
                     {  CAMERA_CMRST_PIN,
                        CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,              /* ON state */
                        GPIO_OUT_ZERO,             /* OFF state */
                        CAMERA_CMPDN_PIN,
                        CAMERA_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for sub sensor
                     {  CAMERA_CMRST1_PIN,
                        CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                        CAMERA_CMPDN1_PIN,
                        CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for main_2 sensor
                     {   CAMERA_CMRST2_PIN,
                        CAMERA_CMRST2_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,               /* ON state */
                        GPIO_OUT_ZERO,              /* OFF state */
                        CAMERA_CMPDN2_PIN,
                        CAMERA_CMPDN2_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     }
                   };

    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }
    else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
        pinSetIdx = 2;
    }

   
    //power ON
    if (On) {

	ISP_MCLK1_EN(1);

        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
	}
        if(pinSetIdx) {
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
	}
	}
        //VCAM_IO
        if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
        {
            PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
            goto _kdCISModulePowerOn_exit_;
        }

        if(pinSetIdx) {
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
	}
	}

        //VCAM_A
        if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
        {
            PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
            goto _kdCISModulePowerOn_exit_;
        }
 
        //DVDD
        if ((currSensorName && (0 == strcmp(currSensorName,"imx135mipiraw")))||
            (currSensorName && (0 == strcmp(currSensorName,"imx220mipiraw")))||
            (currSensorName && (0 == strcmp(currSensorName,"imx214mipiraw"))))
        {
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1000,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                 goto _kdCISModulePowerOn_exit_;
            }
        }
        else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5648_MIPI_RAW, currSensorName)))
        {
            if(TRUE != hwPowerOn(SUB_CAMERA_POWER_VCAM_D, VOL_1500,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                 goto _kdCISModulePowerOn_exit_;
            }
        }            
        else {
            if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
                 goto _kdCISModulePowerOn_exit_;
            }
        }
 
        //AF_VCC
        if(pinSetIdx == 0 ) {
		vcm_smooth_enable(mode_name);
	}

        //enable active sensor
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            usleep_range(10000, 11000);
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
	}

	usleep_range(2000, 3000);

	/* Disable unused camera sensor */
	tmp_idx = (pinSetIdx + 1) % 2;
        if (GPIO_CAMERA_INVALID != pinSet[tmp_idx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[tmp_idx][IDX_PS_CMRST],pinSet[tmp_idx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[tmp_idx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[tmp_idx][IDX_PS_CMRST],pinSet[tmp_idx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
	}
	if (tmp_idx) {
        if (GPIO_CAMERA_INVALID != pinSet[tmp_idx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[tmp_idx][IDX_PS_CMPDN],pinSet[tmp_idx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[tmp_idx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[tmp_idx][IDX_PS_CMPDN],pinSet[tmp_idx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
	}
	}
    }
    else {//power OFF
	ISP_MCLK1_EN(0);

        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
	}
        if(pinSetIdx) {
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMPDN]) {
                if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
                if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
                if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
	}
	}

        //DVDD
        if (currSensorName && (0 == strcmp(currSensorName,"imx135mipiraw"))) 
        {
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to OFF core power(%d)\n",CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }
        }
        else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5648_MIPI_RAW, currSensorName)))
        {
            if(TRUE != hwPowerDown(SUB_CAMERA_POWER_VCAM_D,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to OFF core power(%d)\n",SUB_CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }
        }            
        else {
            if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name))
            {
                 PK_DBG("[CAMERA SENSOR] Fail to OFF core power(%d)\n",CAMERA_POWER_VCAM_D);
                 goto _kdCISModulePowerOn_exit_;
            }
        
        }

        if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
            PK_DBG("[CAMERA SENSOR] Fail to OFF analog power(%d)\n",CAMERA_POWER_VCAM_A);
            //return -EIO;
            goto _kdCISModulePowerOn_exit_;
        }

        if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2, mode_name)) {
            PK_DBG("[CAMERA SENSOR] Fail to OFF digital power(%d)\n",CAMERA_POWER_VCAM_D2);
            //return -EIO;
            goto _kdCISModulePowerOn_exit_;
        }

        if(pinSetIdx == 0 ) {
		vcm_smooth_disable(mode_name);
	}
    }//

    return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;
    
}

EXPORT_SYMBOL(kdCISModulePowerOn);
EXPORT_SYMBOL(flash_gpio_en);
EXPORT_SYMBOL(camera_af_poweron);
