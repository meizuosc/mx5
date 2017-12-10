#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"
#include "../camera/kd_camera_hw.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "[sub_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        pr_warning(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __FUNCTION__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

#define I2C_WRITE_ID	0xC6

extern int flash_gpio_en(flash_gpio gpio_pin, u8 on);

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */
static struct work_struct workTimeOut;
static int g_timeOutTimeMs=0;
static u32 strobe_Res = 0;
static kal_uint32 g_duty = 0;
/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);

#define MAX_DUTY	13
static kal_uint32 sub_torch_duty[6][2] = {
{25, 8},	//{current, reg_code}
{50, 17},
{75, 26},
{100, 35},
{125, 44},
{150, 53}
};

static kal_uint32 sub_flash_duty[14][2] = {
{25, 0},	//{current, reg_code}
{50, 1},
{75, 2},
{100, 3},
{125, 4},
{150, 5},
{175, 6},
{200, 7},
{225, 8},
{250, 9},
{275, 11},
{300, 12},
{325, 13},
{350, 14}
};

static kal_uint8 read_flash_reg(kal_uint8 addr)
{
	u8 reg_val = 0;
	flashlight_ReadRegI2C((u8 *)&addr, 1, (u8 *)&reg_val, 1, I2C_WRITE_ID);
	return reg_val;
}

static void write_flash_reg(kal_uint8 addr, kal_uint8 data)
{
	u8 send_cmd[2] = {(u8)(addr & 0xFF), (u8)(data & 0xFF)};
	flashlight_WriteRegI2C(send_cmd, 2, I2C_WRITE_ID);
}

static int FL_Enable(void)
{
	PK_DBG("FL_Enable-");
	if (g_duty < 6) {
		write_flash_reg(0x01, 0x9B);
		flash_gpio_en(SUB_GPIO_FLASH_TORCH_EN, 1);
	} else {
		write_flash_reg(0x01, 0xAF);
		flash_gpio_en(SUB_GPIO_FLASH_STROBE_EN, 1);
	}

    return 0;
}

static int FL_Disable(void)
{
	write_flash_reg(0x01, 0x80);
	if (g_duty < 6) {
		flash_gpio_en(SUB_GPIO_FLASH_TORCH_EN, 0);
	} else {
		flash_gpio_en(SUB_GPIO_FLASH_STROBE_EN, 0);
	}
	return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
	u8 reg_val = 0;

	if (duty > MAX_DUTY) {
		PK_DBG("Invalid FLASHLIGHT DUTY!\n");
		return -EINVAL;
	}

	g_duty = duty;
	if (duty < 6) {
		write_flash_reg(0x05, (u8)(sub_torch_duty[duty][1] & 0xFF));

		/* torch current ramp time */
		reg_val = read_flash_reg(0x08);
		reg_val &= 0x8F;
		write_flash_reg(0x08, reg_val);
	} else {
		write_flash_reg(0x03, (u8)(sub_flash_duty[duty][1] & 0xFF));

		/* set flash time-out 400ms */
		reg_val = read_flash_reg(0x08);
		reg_val |= 0x0F;
		write_flash_reg(0x08, reg_val);
	}
	return 0;
}

static int FL_Init(void)
{
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    flash_gpio_en(SUB_GPIO_FLASH_EN, 1);
    INIT_WORK(&workTimeOut, work_timeOutFunc);
    return 0;
}

static int FL_Uninit(void)
{
    FL_Disable();
    flash_gpio_en(SUB_GPIO_FLASH_EN, 0);
    return 0;
}

static int FL_hasLowPowerDetect(void)
{
	return 1;
}

static int detLowPowerStart(void)
{

    return 0;
}


static int detLowPowerEnd(void)
{
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/
static struct hrtimer g_timeOutTimer;

static int g_b1stInit=1;

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
}



static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}

static void timerInit(void)
{
	if(g_b1stInit==1)
	{
		g_b1stInit=0;
	  	INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs=1000; //1s
		hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
		g_timeOutTimer.function=ledTimeOutCallback;
	}



}

static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	int temp;
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	//kal_uint8 valTemp;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			if (arg > 4000)
				arg = 4000;
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",(int)arg);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",(int)arg);
    		FL_dim_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",(int)arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",(int)arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
/*
    	case FLASH_IOC_PRE_ON:
    		PK_DBG("FLASH_IOC_PRE_ON\n");
			FL_preOn();
    		break;
    	case FLASH_IOC_GET_PRE_ON_TIME_MS:
    		PK_DBG("FLASH_IOC_GET_PRE_ON_TIME_MS: %d\n",(int)arg);
    		temp=13;
    		if(copy_to_user((void __user *) arg , (void*)&temp , 4))
            {
                PK_DBG(" ioctl copy to user failed\n");
                return -1;
            }
    		break;
*/
        case FLASH_IOC_SET_REG_ADR:
            PK_DBG("FLASH_IOC_SET_REG_ADR: %d\n",(int)arg);
            //g_reg = arg;
            break;
        case FLASH_IOC_SET_REG_VAL:
            PK_DBG("FLASH_IOC_SET_REG_VAL: %d\n",(int)arg);
            //g_val = arg;
            break;
        case FLASH_IOC_SET_REG:
          //  PK_DBG("FLASH_IOC_SET_REG: %d %d\n",g_reg, g_val);

            break;

        case FLASH_IOC_GET_REG:
            PK_DBG("FLASH_IOC_GET_REG: %d\n",(int)arg);

            //i4RetValue = valTemp;
            //PK_DBG("FLASH_IOC_GET_REG: v=%d\n",valTemp);
            break;

        case FLASH_IOC_HAS_LOW_POWER_DETECT:
    		PK_DBG("FLASH_IOC_HAS_LOW_POWER_DETECT");
    		temp=FL_hasLowPowerDetect();
    		if(copy_to_user((void __user *) arg , (void*)&temp , 4))
            {
                PK_DBG(" ioctl copy to user failed\n");
                return -1;
            }
    		break;
    	case FLASH_IOC_LOW_POWER_DETECT_START:
    		detLowPowerStart();
    		break;
    	case FLASH_IOC_LOW_POWER_DETECT_END:
    		i4RetValue = detLowPowerEnd();
    		break;

        default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}

static int sub_strobe_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;
}

static int sub_strobe_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;


        spin_unlock_irq(&g_strobeSMPLock);
    	FL_Uninit();
    }
    PK_DBG(" Done\n");
    return 0;

}

FLASHLIGHT_FUNCTION_STRUCT	subStrobeFunc=
{
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &subStrobeFunc;
    }
    return 0;
}





