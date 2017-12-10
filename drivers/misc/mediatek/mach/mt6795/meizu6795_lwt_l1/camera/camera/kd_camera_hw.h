#ifndef _KD_CAMERA_HW_H_
#define _KD_CAMERA_HW_H_
 

#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include "pmic_drv.h"

//
//Analog 
#define CAMERA_POWER_VCAM_A PMIC_APP_MAIN_CAMERA_POWER_A
//main camera Digital 
#define CAMERA_POWER_VCAM_D PMIC_APP_MAIN_CAMERA_POWER_D
// sub camera Digital 
#define SUB_CAMERA_POWER_VCAM_D PMIC_APP_SUB_CAMERA_POWER_D
//AF 
#define CAMERA_POWER_VCAM_A2 PMIC_APP_MAIN_CAMERA_POWER_AF
//digital io
#define CAMERA_POWER_VCAM_D2 PMIC_APP_MAIN_CAMERA_POWER_IO

/* Flight-sense power */
#define FLIGHT_POWER_A	PMIC_APP_AMBIENT_LIGHT_SENSOR_VDD

//defined in DCT tool 
//Main sensor
#define CAMERA_CMRST_PIN            GPIO_CAMERA_XCLR_PIN 
#define CAMERA_CMRST_PIN_M_GPIO     GPIO_CAMERA_XCLR_PIN_M_GPIO

//FRONT sensor
#define CAMERA_CMRST1_PIN           GPIO_CAMERA_XCLFCM
#define CAMERA_CMRST1_PIN_M_GPIO    GPIO_CAMERA_XCLFCM_M_GPIO 

/* Flight-sense enable pin */
#define CAMERA_LASER_EN_PIN		GPIO_LASER_EN_PIN
#define CAMERA_LASER_EN_PIN_M_GPIO	GPIO_LASER_EN_PIN_M_GPIO

// Define I2C Bus Num
#define SUPPORT_I2C_BUS_NUM1        0
#define SUPPORT_I2C_BUS_NUM2        2

#endif
