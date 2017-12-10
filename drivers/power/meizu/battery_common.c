/*****************************************************************************
 *
 * Filename:
 * ---------
 *    battery_common.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines functions of mt6323 Battery charging algorithm
 *   and the Anroid Battery service for updating the battery status
 *
 * Author:
 * -------
 * Oscar Liu
 *
 ****************************************************************************/
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <linux/suspend.h>

#include <asm/scatterlist.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/mt_sleep.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpt.h>
#include <mach/mt_boot.h>

#include <cust_charging.h>
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>
#include <mach/charging.h>
#include <mach/battery_common.h>
#include "cust_charging.h"
#include <mach/mt_boot.h>
#include "mach/mtk_rtc.h"
#include <mach/bq2589x_reg.h>
#include <mach/bq27532_battery.h>
#include <linux/meizu-sys.h>
#include <linux/sched/rt.h>
#include <linux/switch.h>

#if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
#include "cust_pe.h"
#endif


int Enable_BATDRV_LOG = 0;
/* static struct proc_dir_entry *proc_entry; */
char proc_bat_data[32];

PMU_ChargerStruct BMT_status;

int g_battery_thermal_throttling_flag = 3;	/* 0:nothing, 1:enable batTT&chrTimer, 2:disable batTT&chrTimer, 3:enable batTT, disable chrTimer */
int battery_cmd_thermal_test_mode = 0;
int battery_cmd_thermal_test_mode_value = 0;
int g_battery_tt_check_flag = 0;	/* 0:default enable check batteryTT, 1:default disable check batteryTT */

struct wake_lock battery_suspend_lock;
CHARGING_CONTROL battery_charging_control;
unsigned int g_BatteryNotifyCode = 0x0000;
unsigned int g_BN_TestMode = 0x0000;
kal_bool g_bat_init_flag = 0;
unsigned int g_call_state = CALL_IDLE;
int g_platform_boot_mode = 0;
struct timespec g_bat_time_before_sleep;
int g_smartbook_update = 0;

#if defined(MTK_TEMPERATURE_RECHARGE_SUPPORT)
kal_uint32 g_batt_temp_status = TEMP_POS_NORMAL;
#endif

kal_bool battery_suspended = KAL_FALSE;
#ifdef MTK_ENABLE_AGING_ALGORITHM
extern U32 suspend_time;
#endif

#if defined(CUST_SYSTEM_OFF_VOLTAGE)
#define SYSTEM_OFF_VOLTAGE CUST_SYSTEM_OFF_VOLTAGE
#endif

int charging_level_data[1] = { 0 };

kal_bool g_ftm_battery_flag = KAL_FALSE;
#if !defined(CONFIG_POWER_EXT)
static int g_wireless_state;
#endif

#define BAT_MS_TO_NS(x) (x * 1000 * 1000)
static kal_bool bat_thread_timeout = KAL_FALSE;
static kal_bool chr_wake_up_bat = KAL_FALSE;	/* charger in/out to wake up battery thread */
static DEFINE_MUTEX(bat_mutex);
static DEFINE_MUTEX(charger_type_mutex);
static DECLARE_WAIT_QUEUE_HEAD(bat_thread_wq);
static struct hrtimer charger_hv_detect_timer;
static struct task_struct *charger_hv_detect_thread = NULL;
static kal_bool charger_hv_detect_flag = KAL_FALSE;
static DECLARE_WAIT_QUEUE_HEAD(charger_hv_detect_waiter);
static struct hrtimer battery_kthread_timer;
static kal_bool g_battery_soc_ready = KAL_FALSE;

extern char* saved_command_line;

int cmd_discharging = -1;
static int adjust_power = -1;
static int suspend_discharging = -1;

static bool temperature_debug = KAL_FALSE;
static bool batvol_debug = KAL_FALSE;

static int low_bat_test = 0;

static struct task_struct *bat_thread_task = NULL;

static bool charger_full_state = KAL_FALSE;

static kal_bool pre_charger_exist = KAL_FALSE;

struct wireless_data {
	struct power_supply psy;
	int WIRELESS_ONLINE;
};

struct ac_data {
	struct power_supply psy;
	int AC_ONLINE;
};

struct usb_data {
	struct power_supply psy;
	int USB_ONLINE;
};

struct battery_data {
	struct power_supply psy;
	int BAT_STATUS;
	int BAT_HEALTH;
	int BAT_PRESENT;
	int BAT_TECHNOLOGY;
	int BAT_CAPACITY;
	/* Add for Battery Service */
	int BAT_batt_vol;
	int BAT_batt_temp;
	int BAT_BatteryAverageCurrent;
	int BAT_BatterySenseVoltage;
	int BAT_ChargerVoltage;
	int adjust_power;
};

static enum power_supply_property wireless_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_CAPACITY,
	/* Add for Battery Service */
	POWER_SUPPLY_PROP_batt_vol,
	POWER_SUPPLY_PROP_batt_temp,
	//POWER_SUPPLY_PROP_BatteryAverageCurrent,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_ChargerVoltage,
	/* ADB CMD Discharging */
	POWER_SUPPLY_PROP_adjust_power,
};



/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // extern function */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* extern void mt_power_off(void); */
extern bool mt_usb_is_device(void);
#if defined(CONFIG_USB_MTK_HDRC) || defined(CONFIG_USB_MU3D_DRV)
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);
#else
#define mt_usb_connect() do { } while (0)
#define mt_usb_disconnect() do { } while (0)
#endif
/* extern int set_rtc_spare_fg_value(int val); */

void check_battery_exist(void);
void charging_suspend_enable(void)
{
    U32 charging_enable = true;

    suspend_discharging = 0;
    battery_charging_control(CHARGING_CMD_ENABLE,&charging_enable);
}

void charging_suspend_disable(void)
{
    U32 charging_enable = false;

    suspend_discharging = 1;
    battery_charging_control(CHARGING_CMD_ENABLE,&charging_enable);
}

int read_tbat_value(void)
{
	kal_int32 temperature;
	temperature = bq27532_battery_read_temperature();
	return temperature;
}

int get_charger_detect_status(void)
{
	kal_bool chr_status;

	battery_charging_control(CHARGING_CMD_GET_CHARGER_DET_STATUS, &chr_status);
	return chr_status;
}

#if defined(CONFIG_MTK_POWER_EXT_DETECT)
kal_bool bat_is_ext_power(void)
{
	kal_bool pwr_src = 0;

	battery_charging_control(CHARGING_CMD_GET_POWER_SOURCE, &pwr_src);
	battery_xlog_printk(BAT_LOG_FULL, "[BAT_IS_EXT_POWER] is_ext_power = %d\n", pwr_src);
	return pwr_src;
}
#endif
/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // PMIC PCHR Related APIs */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
kal_bool upmu_is_chr_det(void)
{
#if !defined(CONFIG_POWER_EXT)
	kal_uint32 tmp32;
#endif	

    if(battery_charging_control == NULL)
        battery_charging_control = chr_control_interface;

#if defined(CONFIG_POWER_EXT)
	/* return KAL_TRUE; */
	return get_charger_detect_status();
#else
        if (suspend_discharging==1)
        return KAL_FALSE;

	tmp32 = get_charger_detect_status();

#ifdef CONFIG_MTK_POWER_EXT_DETECT
	if (KAL_TRUE == bat_is_ext_power())
		return tmp32;
#endif

	if (tmp32 == 0) {
		return KAL_FALSE;
	} else {
		if (mt_usb_is_device()) {
			battery_xlog_printk(BAT_LOG_FULL,
					    "[upmu_is_chr_det] Charger exist and USB is not host\n");

			return KAL_TRUE;
		} else {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[upmu_is_chr_det] Charger exist but USB is host\n");

			return KAL_FALSE;
		}
	}
#endif
}
EXPORT_SYMBOL(upmu_is_chr_det);

void wake_up_bat(void)
{
	printk("[BATTERY] wake_up_bat. \r\n");

	chr_wake_up_bat = KAL_TRUE;
	bat_thread_timeout = KAL_TRUE;
#ifdef MTK_ENABLE_AGING_ALGORITHM
	suspend_time = 0;
#endif
	wake_up(&bat_thread_wq);
}
EXPORT_SYMBOL(wake_up_bat);

static ssize_t bat_log_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	if (copy_from_user(&proc_bat_data, buff, len)) {
		battery_xlog_printk(BAT_LOG_FULL, "bat_log_write error.\n");
		return -EFAULT;
	}

	if (proc_bat_data[0] == '1') {
		battery_xlog_printk(BAT_LOG_CRTI, "enable battery driver log system\n");
		Enable_BATDRV_LOG = 1;
	} else if (proc_bat_data[0] == '2') {
		battery_xlog_printk(BAT_LOG_CRTI, "enable battery driver log system:2\n");
		Enable_BATDRV_LOG = 2;
	} else {
		battery_xlog_printk(BAT_LOG_CRTI, "Disable battery driver log system\n");
		Enable_BATDRV_LOG = 0;
	}

	return len;
}

static const struct file_operations bat_proc_fops = {
	.write = bat_log_write,
};

int init_proc_log(void)
{
	int ret = 0;

#if 1
	proc_create("batdrv_log", 0644, NULL, &bat_proc_fops);
	battery_xlog_printk(BAT_LOG_CRTI, "proc_create bat_proc_fops\n");
#else
	proc_entry = create_proc_entry("batdrv_log", 0644, NULL);

	if (proc_entry == NULL) {
		ret = -ENOMEM;
		battery_xlog_printk(BAT_LOG_FULL, "init_proc_log: Couldn't create proc entry\n");
	} else {
		proc_entry->write_proc = bat_log_write;
		battery_xlog_printk(BAT_LOG_CRTI, "init_proc_log loaded.\n");
	}
#endif

	return ret;
}

static int wireless_get_property(struct power_supply *psy,
				 enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;
	struct wireless_data *data = container_of(psy, struct wireless_data, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = data->WIRELESS_ONLINE;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int ac_get_property(struct power_supply *psy,
			   enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;
	struct ac_data *data = container_of(psy, struct ac_data, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = data->AC_ONLINE;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int usb_get_property(struct power_supply *psy,
			    enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;
	struct usb_data *data = container_of(psy, struct usb_data, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
#if defined(CONFIG_POWER_EXT)
		/* #if 0 */
		data->USB_ONLINE = 1;
		val->intval = data->USB_ONLINE;
#else
#if defined(CONFIG_MTK_POWER_EXT_DETECT)
		if (KAL_TRUE == bat_is_ext_power())
			data->USB_ONLINE = 1;
#endif
		val->intval = data->USB_ONLINE;
#endif
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int battery_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
    struct battery_data *data = container_of(psy, struct battery_data, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_batt_temp:
	    BMT_status.temperature= val->intval;
	    temperature_debug = true;
	    wake_up_bat();
	    break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
	    BMT_status.manufacturer_name = val->strval;
	    break;
	case POWER_SUPPLY_PROP_batt_vol:
	    BMT_status.bat_vol = val->intval;
	    batvol_debug = true;
	    break;
	default:
	    break;
	}
	return 0;
}

static int battery_get_property(struct power_supply *psy,
				enum power_supply_property psp, union power_supply_propval *val)
{
	int ret = 0;
	kal_int32 curr, curr_now;
	struct battery_data *data = container_of(psy, struct battery_data, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (charger_full_state == KAL_TRUE) {
			data->BAT_STATUS = POWER_SUPPLY_STATUS_FULL;
		}
		val->intval = data->BAT_STATUS;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = data->BAT_HEALTH;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = data->BAT_PRESENT;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = data->BAT_TECHNOLOGY;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = BMT_status.manufacturer_name;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (low_bat_test == 1)
			val->intval = 21;
		else
			val->intval = data->BAT_CAPACITY;
		break;
	case POWER_SUPPLY_PROP_batt_vol:
		if (low_bat_test == 1)
			val->intval = 3800;
		else
			val->intval = data->BAT_batt_vol;
		break;
	case POWER_SUPPLY_PROP_batt_temp:
		val->intval = data->BAT_batt_temp;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		curr = bq27532_battery_current();
		if (BMT_status.charger_exist == KAL_TRUE) {
			BMT_status.ICharging = curr;
		} else {
			BMT_status.ICharging = curr - 65535;
		}
		val->intval = BMT_status.ICharging;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		curr_now = bq27532_battery_current_now();
		if (BMT_status.charger_exist == KAL_TRUE) {
			val->intval = curr_now;
		} else {
			val->intval = curr_now - 65535;
		}
		break;
	case POWER_SUPPLY_PROP_ChargerVoltage:
		val->intval = data->BAT_ChargerVoltage;
		break;
	case POWER_SUPPLY_PROP_adjust_power :
		val->intval = data->adjust_power;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/* wireless_data initialization */
static struct wireless_data wireless_main = {
	.psy = {
		.name = "wireless",
		.type = POWER_SUPPLY_TYPE_WIRELESS,
		.properties = wireless_props,
		.num_properties = ARRAY_SIZE(wireless_props),
		.get_property = wireless_get_property,
		},
	.WIRELESS_ONLINE = 0,
};

/* ac_data initialization */
static struct ac_data ac_main = {
	.psy = {
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.properties = ac_props,
		.num_properties = ARRAY_SIZE(ac_props),
		.get_property = ac_get_property,
		},
	.AC_ONLINE = 0,
};

/* usb_data initialization */
static struct usb_data usb_main = {
	.psy = {
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.properties = usb_props,
		.num_properties = ARRAY_SIZE(usb_props),
		.get_property = usb_get_property,
		},
	.USB_ONLINE = 0,
};

/* battery_data initialization */
static struct battery_data battery_main = {
	.psy = {
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = battery_props,
		.num_properties = ARRAY_SIZE(battery_props),
		.get_property = battery_get_property,
		.set_property = battery_set_property,	
		},
/* CC: modify to have a full power supply status */
#if defined(CONFIG_POWER_EXT)
	.BAT_STATUS = POWER_SUPPLY_STATUS_FULL,
	.BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD,
	.BAT_PRESENT = 1,
	.BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION,
	.BAT_CAPACITY = 100,
	.BAT_batt_vol = 4200,
	.BAT_batt_temp = 22,
	/* ADB CMD discharging*/
	.adjust_power = -1,
#else
	.BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING,
	.BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD,
	.BAT_PRESENT = 1,
	.BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION,
	.BAT_CAPACITY = 10,
	.BAT_batt_vol = 0,
	.BAT_batt_temp = 0,
	/* ADB CMD discharging*/
	.adjust_power = -1,
#endif
};

#if !defined(CONFIG_POWER_EXT)
static ssize_t show_Charger_Type(struct device *dev,struct device_attribute *attr,
					char *buf)
{
    UINT32 chr_ype = CHARGER_UNKNOWN;
    chr_ype = BMT_status.charger_exist ? BMT_status.charger_type : CHARGER_UNKNOWN;

    battery_xlog_printk(BAT_LOG_CRTI, "CHARGER_TYPE = %d\n",chr_ype);
    return sprintf(buf, "%u\n", chr_ype);
}
static ssize_t store_Charger_Type(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[EM] Not Support Write Function\n");
    return size;
}
static DEVICE_ATTR(Charger_Type, 0664, show_Charger_Type, store_Charger_Type);

#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
static ssize_t show_Pump_Express(struct device *dev,struct device_attribute *attr,
					char *buf)
{
    #if defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
    int icount = 20;  //max debouncing time 20 * 0.2 sec

    if (KAL_TRUE == ta_check_chr_type &&
      STANDARD_CHARGER == BMT_status.charger_type &&
      BMT_status.SOC >= TA_START_BATTERY_SOC &&
      BMT_status.SOC < TA_STOP_BATTERY_SOC)
    {
        battery_xlog_printk(BAT_LOG_CRTI, "[%s]Wait for PE detection\n", __func__);
        do
        {
            icount--;
            msleep(200);
        }while(icount && ta_check_chr_type);
    }
    #endif

    battery_xlog_printk(BAT_LOG_CRTI, "Pump express = %d\n",is_ta_connect);    
    return sprintf(buf, "%d\n", is_ta_connect);
}
static ssize_t store_Pump_Express(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	int pump_express;
	sscanf(buf, "%d", &pump_express);
	if (pump_express == 1) { //fast charger pump up
		mtk_ta_detector();
	} else if (pump_express == 0) { // pump down to 5V
		mtk_ta_reset_vchr();
	}
	battery_xlog_printk(BAT_LOG_CRTI, "Pump express= %d\n",pump_express);    
    return size;
}
static DEVICE_ATTR(Pump_Express, 0664, show_Pump_Express, store_Pump_Express);
#endif

static ssize_t show_charger_enable_major(struct device *dev,struct device_attribute *attr,
					char *buf)
{
	int enable;

	battery_charging_control(CHARGING_CMD_GET_CHARGER_DET_STATUS, &enable);
	return sprintf(buf, "%d\n", enable);
}

static ssize_t store_charger_enable_major(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
	int enable_major;

	enable_major = simple_strtol(buf, NULL, 10);

	battery_charging_control(CHARGING_CMD_ENABLE_MAJOR, &enable_major);

	return size;
}
static DEVICE_ATTR(charger_enable_major, 0664, show_charger_enable_major, store_charger_enable_major);

static ssize_t show_charger_enable_minor(struct device *dev,struct device_attribute *attr,
					char *buf)
{
	int enable;

	battery_charging_control(CHARGING_CMD_GET_CHARGER_DET_STATUS, &enable);
	return sprintf(buf, "%d\n", enable);
}

static ssize_t store_charger_enable_minor(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
	int enable_minor;

	enable_minor = simple_strtol(buf, NULL, 10);
	battery_charging_control(CHARGING_CMD_ENABLE_MINOR, &enable_minor);

	return size;
}
static DEVICE_ATTR(charger_enable_minor, 0664, show_charger_enable_minor, store_charger_enable_minor);

static ssize_t show_fake_cap_enable(struct device *dev,struct device_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%d\n", low_bat_test);
}

static void mt_battery_update_status(void);
static ssize_t store_fake_cap_enable(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{

	low_bat_test = simple_strtol(buf, NULL, 10);
	mt_battery_update_status();

	return size;
}
static DEVICE_ATTR(fake_cap_enable, 0664, show_fake_cap_enable, store_fake_cap_enable);

/* mcharger_enable's function is enable or disable fast charger
	0:disable fast charging
	1:enable fast charging
	defalut:1
 */
static ssize_t charger_show_limit_fastcharging(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", BMT_status.mcharger_enable);
}

static ssize_t charger_store_limit_fastcharging(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0, ret = 0;

	ret = kstrtoint(buf, 10, &val);
	if (ret == -ERANGE || ret == -EINVAL)
		return ret;
	BMT_status.mcharger_enable = val;
	wake_up_bat();

	return count;
}
static DEVICE_ATTR(fastcharging, 0644, charger_show_limit_fastcharging,
		charger_store_limit_fastcharging);

static void mt_battery_update_EM(struct battery_data *bat_data)
{
	int chrg_stat;
	kal_uint32 battery_voltage = 0; 
	int i = 0, count_vol = 0;
	int truesoc = 0;
	static int counter = 0;

	do {
		battery_voltage = bq27532_battery_voltage(); 
		if ((battery_voltage <= SYSTEM_OFF_VOLTAGE) ||
				(BMT_status.charger_exist == KAL_TRUE)&&
				(battery_voltage < 3350))
			count_vol++;
		i++;
	} while(i <= 1);
	if (count_vol == 2) {
		bat_data->BAT_CAPACITY = 0;	
	} else {
		bat_data->BAT_CAPACITY = BMT_status.UI_SOC;
	}
	/*modify the 99% staying long time*/
	if ((BMT_status.UI_SOC == 99) && (BMT_status.charger_exist == KAL_TRUE)) {
		truesoc = bq27532_battery_read_truesoc();	
		if ((counter >= 5) && (truesoc == 100)) {
			BMT_status.UI_SOC = truesoc;
		} else if (truesoc == 100)
			counter++;
		else
			counter = 0;
	} else if (BMT_status.charger_exist == KAL_FALSE) {
		counter = 0;
	}
	bat_data->BAT_CAPACITY = BMT_status.UI_SOC;
	bat_data->BAT_BatteryAverageCurrent = BMT_status.ICharging;
	bat_data->BAT_BatterySenseVoltage = BMT_status.bat_vol;
	bat_data->BAT_ChargerVoltage = BMT_status.charger_vol;

	/*00 – Not Charging, 01 – Pre-charge, 10 – Fast Charging,
	  11 – Charge Termination Done*/
	chrg_stat = bq2589x_get_charging_status();
	if ((BMT_status.UI_SOC == 100 && chrg_stat == 3) && (BMT_status.charger_exist == KAL_TRUE)) {
		bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_FULL;
		charger_full_state = KAL_TRUE;
	}

#ifdef CONFIG_MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
	if (bat_data->BAT_CAPACITY <= 0)
		bat_data->BAT_CAPACITY = 1;

	battery_xlog_printk(BAT_LOG_CRTI,
			    "BAT_CAPACITY=1, due to define CONFIG_MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION\r\n");
#endif
}

static void battery_high_percent_tracking()
{
	static int time_counter = 0;

	if (BMT_status.SOC >= 92) {
		if (pre_charger_exist == KAL_TRUE) {
			BMT_status.UI_SOC = BMT_status.SOC;		
		} else if (BMT_status.SOC < BMT_status.Pre_ui_soc) {
			BMT_status.UI_SOC = BMT_status.SOC+1;
		} else {
			BMT_status.UI_SOC = BMT_status.SOC;
		}
	} else if (BMT_status.UI_SOC > BMT_status.SOC) {
		if (time_counter == 3) {
			BMT_status.UI_SOC--;
			time_counter = 0;
		} else  {
			time_counter++;
		}
	} else {
		time_counter = 0;
		BMT_status.UI_SOC = BMT_status.SOC;
	}
	if (BMT_status.UI_SOC > 100)
		BMT_status.UI_SOC = 100;
}

static void battery_update(struct battery_data *bat_data)
{
	struct power_supply *bat_psy = &bat_data->psy;
	kal_bool resetBatteryMeter = KAL_FALSE;
	INT32 truesoc = 0;	

	truesoc = bq27532_battery_read_truesoc();	
	bat_data->BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION;
	bat_data->BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD;
	bat_data->BAT_batt_vol = BMT_status.bat_vol;
	bat_data->BAT_batt_temp = BMT_status.temperature * 10;
	bat_data->BAT_PRESENT = BMT_status.bat_exist;

	if (BMT_status.charger_exist == KAL_TRUE) {
		if (BMT_status.UI_SOC - BMT_status.SOC == 1) {
			BMT_status.UI_SOC = BMT_status.SOC+1;
		} else {
			BMT_status.UI_SOC = BMT_status.SOC; 
		}
		if (charger_full_state == KAL_TRUE) {
			BMT_status.UI_SOC = 100;
		}
	} else {
		if (truesoc == 100 || BMT_status.SOC == 100) {
			BMT_status.UI_SOC = 100;
		} else {
			//battery_high_percent_tracking();
			BMT_status.UI_SOC = BMT_status.SOC;
		}
	}
	if( (BMT_status.charger_exist == KAL_TRUE) && (BMT_status.bat_charging_state != CHR_ERROR) ) {     
		if ( BMT_status.bat_exist ) {  /* charging */
			bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CHARGING;  
		} else {	 /* No Battery, Only Charger */
			bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_UNKNOWN;
		}
	} else {	/* Only Battery */
		bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING;
		if (BMT_status.bat_vol <= SYSTEM_OFF_VOLTAGE) {
			BMT_status.UI_SOC = 0;
		}
	} 
	mt_battery_update_EM(bat_data);

	if (cmd_discharging == 1) {
		bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CMD_DISCHARGING;
	}
	if (adjust_power != -1) {
			bat_data->adjust_power = adjust_power;
			battery_xlog_printk(BAT_LOG_CRTI, "adjust_power=(%d)\n", adjust_power);
	}

	BMT_status.Pre_ui_soc = BMT_status.SOC;
	power_supply_changed(bat_psy);
}

static void wireless_update(struct wireless_data *wireless_data)
{
	struct power_supply *wireless_psy = &wireless_data->psy;

	if (BMT_status.charger_exist == KAL_TRUE || g_wireless_state) {
		if ((BMT_status.charger_type == WIRELESS_CHARGER) || g_wireless_state) {
			wireless_data->WIRELESS_ONLINE = 1;
			wireless_psy->type = POWER_SUPPLY_TYPE_WIRELESS;
		} else {
			wireless_data->WIRELESS_ONLINE = 0;
		}
	} else {
		wireless_data->WIRELESS_ONLINE = 0;
	}

	power_supply_changed(wireless_psy);
}

static void ac_update(struct ac_data *ac_data)
{
	struct power_supply *ac_psy = &ac_data->psy;

	if (BMT_status.charger_exist == KAL_TRUE) {
		if ((BMT_status.charger_type != STANDARD_HOST) &&
				(BMT_status.charger_type != CHARGING_HOST)) {
			ac_data->AC_ONLINE = 1;
			ac_psy->type = POWER_SUPPLY_TYPE_MAINS;
		} else {
			ac_data->AC_ONLINE = 0;
		}
	} else {
		ac_data->AC_ONLINE = 0;
	}

	power_supply_changed(ac_psy);
}

static void usb_update(struct usb_data *usb_data)
{
	struct power_supply *usb_psy = &usb_data->psy;

	if (BMT_status.charger_exist == KAL_TRUE) {
		if ((BMT_status.charger_type == STANDARD_HOST) ||
		    (BMT_status.charger_type == CHARGING_HOST)) {
			usb_data->USB_ONLINE = 1;
			usb_psy->type = POWER_SUPPLY_TYPE_USB;
		} else {
			usb_data->USB_ONLINE = 0;
		}
	} else {
		usb_data->USB_ONLINE = 0;
	}

	power_supply_changed(usb_psy);
}

#endif

kal_bool pmic_chrdet_status(void)
{
	if (upmu_is_chr_det() == KAL_TRUE) {
		return KAL_TRUE;
	} else {
		battery_xlog_printk(BAT_LOG_CRTI, "[pmic_chrdet_status] No charger\r\n");
		return KAL_FALSE;
	}
}

kal_bool bat_is_charger_exist(void)
{
	return get_charger_detect_status();
}


kal_bool bat_is_charging_full(void)
{
	if ((BMT_status.bat_full == KAL_TRUE) && (BMT_status.bat_in_recharging_state == KAL_FALSE))
		return KAL_TRUE;
	else
		return KAL_FALSE;
}


kal_uint32 bat_get_ui_percentage(void)
{
	/* for plugging out charger in recharge phase, using SOC as UI_SOC */
	if (chr_wake_up_bat == KAL_TRUE)
		return BMT_status.SOC;
	else
		return BMT_status.UI_SOC;
}

/* Full state --> recharge voltage --> full state */
kal_uint32 bat_is_recharging_phase(void)
{
	return (BMT_status.bat_in_recharging_state || BMT_status.bat_full == KAL_TRUE);
}


int get_bat_charging_current_level(void)
{
	CHR_CURRENT_ENUM charging_current;

	battery_charging_control(CHARGING_CMD_GET_CURRENT, &charging_current);

	return charging_current;
}

#if defined(MTK_TEMPERATURE_RECHARGE_SUPPORT)
PMU_STATUS do_batt_temp_state_machine(void)
{
	char *battery_cell;
	int board_temperature;

	if (BMT_status.temperature == ERR_CHARGE_TEMPERATURE) {
		return PMU_STATUS_FAIL;
	}
	
	board_temperature = mtkts_AP_get_hw_temp() / 1000;
	battery_cell = strchr(BMT_status.manufacturer_name,'_');
	*(battery_cell++);
	/*
		ATL电芯:
		0‘C-10’C	0.3C CC to 4.35V, CV to 0.02C
		10’C-20’C	1.5C CC to 4.2V, 0.5C CC to 4.35V, CV to 0.02C.
		20’C-45’C	1.5C CC to 4.35V. CV to 0.02C
		45’C-55’C 	0.5C CC to 4.1V, CV to 0.02C
	*/
	if (!strcmp(battery_cell,"ATL")) {
		if (BMT_status.temperature < MIN_CHARGE_TEMPERATURE) {
			printk("[BATTERY] Battery Under 0OC Temperature or NTC fail !!\n\r");
			g_batt_temp_status = TEMP_POS_LOW;
			return PMU_STATUS_FAIL;
		} else if (BMT_status.temperature < T10_CHARGE_TEMPERATURE) {
			printk("[BATTERY] Battery Under Temperature 10oC!!\n\r"); 
			g_batt_temp_status = TEMP_POS_T10;
			BMT_status.bat_charging_state = CHR_PRE;
			return PMU_STATUS_OK;
		} else if (BMT_status.temperature < T20_CHARGE_TEMPERATURE) {
			printk("[BATTERY] Battery Temperature Under temperature 20oC\n\r"); 
			g_batt_temp_status = TEMP_POS_T20;
			BMT_status.bat_charging_state = CHR_PRE;
			return PMU_STATUS_OK;
		} else if (BMT_status.temperature <= T45_CHARGE_TEMPERATURE) {
			printk("[BATTERY] Battery Temperature Under Temperature 45oC!!,board_temperature %d\n\r",
					board_temperature);
			if (g_batt_temp_status == TEMP_POS_T42) {
				if (max(board_temperature, BMT_status.temperature) < T38_CHARGE_TEMPERATURE) {
					g_batt_temp_status = TEMP_POS_NORMAL;
				}
			} else {
				g_batt_temp_status = TEMP_POS_NORMAL;
			}
			if (max(board_temperature, BMT_status.temperature) >= T42_CHARGE_TEMPERATURE) {
				g_batt_temp_status = TEMP_POS_T42;
			}
			BMT_status.bat_charging_state = CHR_PRE;
			return PMU_STATUS_OK;
		} else if (BMT_status.temperature > T45_CHARGE_TEMPERATURE){
			printk("[BATTERY] Battery Over Temperature !!\n\r");
			g_batt_temp_status = TEMP_POS_HIGH;
			return PMU_STATUS_FAIL;
		} else {
			g_batt_temp_status = TEMP_POS_NORMAL;
		}
	} else {//!strcmp(battery_cell,"SONY"))
	/*
		SONY电芯:
		0‘C-10’C	0.3C CC to 4.35V, CV to 0.02C
		10’C-15’C	1.0C CC to 4.35V, CV to 0.02C.
		15’C-45’C	1.5C CC to 4.35V. CV to 0.02C
		45’C-55’C 	0.5C CC to 4.1V, CV to 0.02C
	*/
		if (BMT_status.temperature < MIN_CHARGE_TEMPERATURE) {
			printk("[BATTERY] Battery Under 0OC Temperature or NTC fail !!\n\r");
			g_batt_temp_status = TEMP_POS_LOW;
			return PMU_STATUS_FAIL;
		} else if (BMT_status.temperature < T10_CHARGE_TEMPERATURE) {
			printk("[BATTERY] Battery Under Temperature 10oC!!\n\r"); 
			g_batt_temp_status = TEMP_POS_T10;
			BMT_status.bat_charging_state = CHR_PRE;
			return PMU_STATUS_OK;
		} else if (BMT_status.temperature < T15_CHARGE_TEMPERATURE) {
			printk("[BATTERY] Battery Temperature Under temperature 15oC\n\r"); 
			g_batt_temp_status = TEMP_POS_T15;
			BMT_status.bat_charging_state = CHR_PRE;
			return PMU_STATUS_OK;
		} else if (BMT_status.temperature <= T45_CHARGE_TEMPERATURE) {
			printk("[BATTERY] Battery Temperature Under Temperature 45oC!!\n\r");
			if (g_batt_temp_status == TEMP_POS_T42) {
				if (max(board_temperature, BMT_status.temperature) < T38_CHARGE_TEMPERATURE) {
					g_batt_temp_status = TEMP_POS_NORMAL;
				}
			} else {
				g_batt_temp_status = TEMP_POS_NORMAL;
			}
			if (max(board_temperature, BMT_status.temperature) >= T42_CHARGE_TEMPERATURE) {
				g_batt_temp_status = TEMP_POS_T42;
			}
			BMT_status.bat_charging_state = CHR_PRE;
			return PMU_STATUS_OK;
		} else if (BMT_status.temperature > T45_CHARGE_TEMPERATURE){
			printk("[BATTERY] Battery Over Temperature !!\n\r");
			g_batt_temp_status = TEMP_POS_HIGH;
			return PMU_STATUS_FAIL;
		} else {
			g_batt_temp_status = TEMP_POS_NORMAL;
		}
	}
	return PMU_STATUS_OK;
}
#endif

void get_batt_temp_status(void *data)
{
   *(kal_uint32*)(data) = g_batt_temp_status;
}
EXPORT_SYMBOL(get_batt_temp_status);

unsigned long BAT_Get_Battery_Voltage(int polling_mode)
{
	unsigned long ret_val = 0;

#if defined(CONFIG_POWER_EXT)
	ret_val = 4000;
#else
    ret_val=bq2589x_adc_read_battery_volt();
#endif

	return ret_val;
}

void mt_battery_GetBatteryData(void)
{
	kal_int32 curr, volt;

	if (batvol_debug == KAL_FALSE)
		BMT_status.bat_vol = bq27532_battery_voltage(); 
	curr = bq27532_battery_current();
	if (BMT_status.charger_exist == KAL_TRUE) {
		BMT_status.ICharging = curr;
	} else {
		BMT_status.ICharging = curr - 65535;
	}
	if (temperature_debug == KAL_FALSE)
		BMT_status.temperature = bq27532_battery_read_temperature();
	BMT_status.SOC = bq27532_battery_read_soc();
	BMT_status.charger_vol = bq2589x_adc_read_charger_volt();
	bq27532_battery_read_fullchargecapacity();
	
	bq27532_get_battery_data(1);
}

static PMU_STATUS mt_battery_CheckBatteryTemp(void)
{
	PMU_STATUS status = PMU_STATUS_OK;

#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)

	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] support JEITA, temperature=%d\n",
			    BMT_status.temperature);

	if (do_jeita_state_machine() == PMU_STATUS_FAIL) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] JEITA : fail\n");
		status = PMU_STATUS_FAIL;
	}
#else

#if defined(MTK_TEMPERATURE_RECHARGE_SUPPORT)
	if (do_batt_temp_state_machine() == PMU_STATUS_FAIL) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Batt temp check : fail\n");
		status = PMU_STATUS_FAIL;
	}
#else
#ifdef BAT_LOW_TEMP_PROTECT_ENABLE
	if ((BMT_status.temperature < MIN_CHARGE_TEMPERATURE)
	    || (BMT_status.temperature == ERR_CHARGE_TEMPERATURE)) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY] Battery Under Temperature or NTC fail !!\n\r");
		status = PMU_STATUS_FAIL;
	}
#endif
	if (BMT_status.temperature > MAX_CHARGE_TEMPERATURE) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Battery Over Temperature !!\n\r");
		status = PMU_STATUS_FAIL;
	}
#endif

#endif

	return status;
}

static PMU_STATUS mt_battery_CheckChargerVoltage(void)
{
	PMU_STATUS status = PMU_STATUS_OK;

	if (BMT_status.charger_exist == KAL_TRUE) {
#if (V_CHARGER_ENABLE == 1)
		if (BMT_status.charger_vol <= V_CHARGER_MIN) {
			battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY]Charger under voltage!!\r\n");
			BMT_status.bat_charging_state = CHR_ERROR;
			status = PMU_STATUS_FAIL;
		}
#endif
		if (BMT_status.charger_vol >= V_CHARGER_MAX) {
			battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY]Charger over voltage !!\r\n");
			BMT_status.charger_protect_status = charger_OVER_VOL;
			BMT_status.bat_charging_state = CHR_ERROR;
			status = PMU_STATUS_FAIL;
		}
	}

	return status;
}

static PMU_STATUS mt_battery_CheckChargingTime(void)
{
	PMU_STATUS status = PMU_STATUS_OK;

	if ((g_battery_thermal_throttling_flag == 2) || (g_battery_thermal_throttling_flag == 3)) {
		battery_xlog_printk(BAT_LOG_FULL,
				    "[TestMode] Disable Safty Timer. bat_tt_enable=%d, bat_thr_test_mode=%d, bat_thr_test_value=%d\n",
				    g_battery_thermal_throttling_flag,
				    battery_cmd_thermal_test_mode,
				    battery_cmd_thermal_test_mode_value);

	} else {
		/* Charging OT */
		if (BMT_status.total_charging_time >= MAX_CHARGING_TIME) {
			battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Charging Over Time.\n");

			status = PMU_STATUS_FAIL;
		}
	}

	return status;

}

#if defined(STOP_CHARGING_IN_TAKLING)
static PMU_STATUS mt_battery_CheckCallState(void)
{
	PMU_STATUS status = PMU_STATUS_OK;

	if ((g_call_state == CALL_ACTIVE) && (BMT_status.bat_vol > V_CC2TOPOFF_THRES))
		status = PMU_STATUS_FAIL;

	return status;
}
#endif

static void mt_battery_checkchargingstat(void)
{
	kal_uint32 charging_enable = KAL_FALSE;
	int charging_stat = 0;

	charging_stat = bq2589x_get_charging_status();
	if (charging_stat == BQ2589X_CHRG_STAT_CHGDONE && BMT_status.UI_SOC != 100) {
		printk("%s:Charge Termination Done\n", __func__);
		charging_enable = KAL_FALSE;
		battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
		msleep(250);
		charging_enable = KAL_TRUE;
		battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
	}
}

static void mt_battery_CheckBatteryStatus(void)
{
	battery_xlog_printk(BAT_LOG_FULL, "[mt_battery_CheckBatteryStatus] cmd_discharging=(%d)\n",
			    cmd_discharging);
	if (cmd_discharging == 1) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[mt_battery_CheckBatteryStatus] cmd_discharging=(%d)\n",
				    cmd_discharging);
		BMT_status.bat_charging_state = CHR_ERROR;
		battery_charging_control(CHARGING_CMD_SET_ERROR_STATE, &cmd_discharging);	
		return;
	} else if (cmd_discharging == 0) {
		BMT_status.bat_charging_state = CHR_PRE;
		battery_charging_control(CHARGING_CMD_SET_ERROR_STATE, &cmd_discharging);
		cmd_discharging = -1;
	}
	if (mt_battery_CheckBatteryTemp() != PMU_STATUS_OK) {
		BMT_status.bat_charging_state = CHR_ERROR;
		return;
	}

	if (mt_battery_CheckChargerVoltage() != PMU_STATUS_OK) {
		BMT_status.bat_charging_state = CHR_ERROR;
		return;
	}
#if defined(STOP_CHARGING_IN_TAKLING)
	if (mt_battery_CheckCallState() != PMU_STATUS_OK) {
		BMT_status.bat_charging_state = CHR_HOLD;
		return;
	}
#endif

	if (mt_battery_CheckChargingTime() != PMU_STATUS_OK) {
		BMT_status.bat_charging_state = CHR_ERROR;
		return;
	}
	mt_battery_checkchargingstat();
}

static void mt_battery_notify_TotalChargingTime_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0005_TOTAL_CHARGINGTIME)
	if ((g_battery_thermal_throttling_flag == 2) || (g_battery_thermal_throttling_flag == 3)) {
		battery_xlog_printk(BAT_LOG_FULL,
				    "[TestMode] Disable Safty Timer : no UI display\n");
	} else {
		if (BMT_status.total_charging_time >= MAX_CHARGING_TIME)
			/* if(BMT_status.total_charging_time >= 60) //test */
		{
			g_BatteryNotifyCode |= 0x0010;
			battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Charging Over Time\n");
		} else {
			g_BatteryNotifyCode &= ~(0x0010);
		}
	}

	battery_xlog_printk(BAT_LOG_CRTI,
			    "[BATTERY] BATTERY_NOTIFY_CASE_0005_TOTAL_CHARGINGTIME (%x)\n",
			    g_BatteryNotifyCode);
#endif
}

static void mt_battery_notify_VBat_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0004_VBAT)
	if (BMT_status.bat_vol > 4350)
		/* if(BMT_status.bat_vol > 3800) //test */
	{
		g_BatteryNotifyCode |= 0x0008;
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] bat_vlot(%ld) > 4350mV\n",
				    BMT_status.bat_vol);
	} else {
		g_BatteryNotifyCode &= ~(0x0008);
	}

	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BATTERY_NOTIFY_CASE_0004_VBAT (%x)\n",
			    g_BatteryNotifyCode);

#endif
}

static void mt_battery_notify_ICharging_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0003_ICHARGING)
	if ((BMT_status.ICharging > 1000) && (BMT_status.total_charging_time > 300)) {
		g_BatteryNotifyCode |= 0x0004;
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] I_charging(%ld) > 1000mA\n",
				    BMT_status.ICharging);
	} else {
		g_BatteryNotifyCode &= ~(0x0004);
	}

	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BATTERY_NOTIFY_CASE_0003_ICHARGING (%x)\n",
			    g_BatteryNotifyCode);

#endif
}

static void mt_battery_notify_VBatTemp_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0002_VBATTEMP)

	if (BMT_status.temperature >= MAX_WARNING_TEMPERATURE) {
		g_BatteryNotifyCode |= 0x0002;
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] bat_temp(%d) out of range(too high)\n",
				    BMT_status.temperature);
	}
#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
	else if (BMT_status.temperature < TEMP_NEG_10_THRESHOLD) {
		g_BatteryNotifyCode |= 0x0020;
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] bat_temp(%d) out of range(too low)\n",
				    BMT_status.temperature);
	}
#else
#ifdef BAT_LOW_TEMP_PROTECT_ENABLE
	else if (BMT_status.temperature <= MIN_WARNING_TEMPERATURE) {
		g_BatteryNotifyCode |= 0x0020;
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] bat_temp(%d) out of range(too low)\n",
				    BMT_status.temperature);
	}
#endif
#endif

	battery_xlog_printk(BAT_LOG_FULL, "[BATTERY] BATTERY_NOTIFY_CASE_0002_VBATTEMP (%x)\n",
			    g_BatteryNotifyCode);

#endif
}


static void mt_battery_notify_VCharger_check(void)
{
#if defined(BATTERY_NOTIFY_CASE_0001_VCHARGER)
	if (BMT_status.charger_vol > V_CHARGER_MAX) {
		g_BatteryNotifyCode |= 0x0001;
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BMT_status.charger_vol(%d) > %d mV\n",
				    BMT_status.charger_vol, V_CHARGER_MAX);
	} else {
		g_BatteryNotifyCode &= ~(0x0001);
	}
	if (g_BatteryNotifyCode != 0x0000)
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY] BATTERY_NOTIFY_CASE_0001_VCHARGER (%x)\n",
				    g_BatteryNotifyCode);
#endif
}

static void mt_battery_notify_UI_test(void)
{
	if (g_BN_TestMode == 0x0001) {
		g_BatteryNotifyCode = 0x0001;
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0001_VCHARGER\n");
	} else if (g_BN_TestMode == 0x0002) {
		g_BatteryNotifyCode = 0x0002;
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0002_VBATTEMP\n");
	} else if (g_BN_TestMode == 0x0003) {
		g_BatteryNotifyCode = 0x0004;
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0003_ICHARGING\n");
	} else if (g_BN_TestMode == 0x0004) {
		g_BatteryNotifyCode = 0x0008;
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0004_VBAT\n");
	} else if (g_BN_TestMode == 0x0005) {
		g_BatteryNotifyCode = 0x0010;
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0005_TOTAL_CHARGINGTIME\n");
	} else {
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Unknown BN_TestMode Code : %x\n",
				    g_BN_TestMode);
	}
}

void mt_battery_notify_check(void)
{
	g_BatteryNotifyCode = 0x0000;

	if (g_BN_TestMode == 0x0000) {	/* for normal case */
		battery_xlog_printk(BAT_LOG_FULL, "[BATTERY] mt_battery_notify_check\n");

		mt_battery_notify_VCharger_check();

		mt_battery_notify_VBatTemp_check();

		mt_battery_notify_ICharging_check();

		mt_battery_notify_VBat_check();

		mt_battery_notify_TotalChargingTime_check();
	} else {		/* for UI test */

		mt_battery_notify_UI_test();
	}
}

static void mt_battery_thermal_check(void)
{
	if ((g_battery_thermal_throttling_flag == 1) || (g_battery_thermal_throttling_flag == 3)) {
		if (battery_cmd_thermal_test_mode == 1) {
			BMT_status.temperature = battery_cmd_thermal_test_mode_value;
			battery_xlog_printk(BAT_LOG_FULL,
					    "[Battery] In thermal_test_mode , Tbat=%d\n",
					    BMT_status.temperature);
		}
#if defined(CONFIG_MTK_JEITA_STANDARD_SUPPORT)
		/* ignore default rule */
#else
		if (BMT_status.temperature >= 60) {
#if defined(CONFIG_POWER_EXT)
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[BATTERY] CONFIG_POWER_EXT, no update battery update power down.\n");
#else
			{
				if ((g_platform_boot_mode == META_BOOT)
				    || (g_platform_boot_mode == ADVMETA_BOOT)
				    || (g_platform_boot_mode == ATE_FACTORY_BOOT)) {
					battery_xlog_printk(BAT_LOG_FULL,
							    "[BATTERY] boot mode = %d, bypass temperature check\n",
							    g_platform_boot_mode);
				} else {
					struct battery_data *bat_data = &battery_main;
					struct power_supply *bat_psy = &bat_data->psy;

					battery_xlog_printk(BAT_LOG_CRTI,
							    "[Battery] Tbat(%d)>=60, system need power down.\n",
							    BMT_status.temperature);

					bat_data->BAT_CAPACITY = 0;

					power_supply_changed(bat_psy);

					if (BMT_status.charger_exist == KAL_TRUE) {
						/* can not power down due to charger exist, so need reset system */
						battery_charging_control
						    (CHARGING_CMD_SET_PLATFORM_RESET, NULL);
					}
					/* avoid SW no feedback */
					battery_charging_control(CHARGING_CMD_SET_POWER_OFF, NULL);
					/* mt_power_off(); */
				}
			}
#endif
		}
#endif

	}

}

static void mt_battery_update_status(void)
{
#if defined(CONFIG_POWER_EXT)
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] CONFIG_POWER_EXT, no update Android.\n");
#else
	{
		wireless_update(&wireless_main);
		battery_update(&battery_main);
		ac_update(&ac_main);
		usb_update(&usb_main);
	}

#endif
}

CHARGER_TYPE mt_charger_type_detection(void)
{
	CHARGER_TYPE CHR_Type_num = CHARGER_UNKNOWN;

	mutex_lock(&charger_type_mutex);

#if defined(CONFIG_MTK_WIRELESS_CHARGER_SUPPORT)
	battery_charging_control(CHARGING_CMD_GET_CHARGER_TYPE, &CHR_Type_num);
	BMT_status.charger_type = CHR_Type_num;
#else
	if (BMT_status.charger_type == CHARGER_UNKNOWN ||
			BMT_status.charger_type == NONSTANDARD_CHARGER) {
		battery_charging_control(CHARGING_CMD_GET_CHARGER_TYPE, &CHR_Type_num);
		BMT_status.charger_type = CHR_Type_num;

#if defined(CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)&&(defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT))
 	    if (BMT_status.UI_SOC == 100)
		{
			BMT_status.bat_charging_state = CHR_BATFULL;
			BMT_status.bat_full = KAL_TRUE;
		}	

		if (BMT_status.bat_vol > 0)
		{
        	mt_battery_update_status();
		}
		
#endif
	}
#endif
	mutex_unlock(&charger_type_mutex);

	return BMT_status.charger_type;
}

CHARGER_TYPE mt_get_charger_type(void)
{
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
    return STANDARD_HOST;
#else
	return BMT_status.charger_type;
#endif
}

static void mt_battery_charger_detect_check(void)
{
	if (upmu_is_chr_det() == KAL_TRUE) {
		wake_lock(&battery_suspend_lock);

		BMT_status.charger_exist = KAL_TRUE;

#if defined(CONFIG_MTK_WIRELESS_CHARGER_SUPPORT)
		mt_charger_type_detection();

		if ((BMT_status.charger_type == STANDARD_HOST)
		    || (BMT_status.charger_type == CHARGING_HOST)) {
			mt_usb_connect();
		}
#else
		if (BMT_status.charger_type == CHARGER_UNKNOWN) {
			mt_charger_type_detection();

			if ((BMT_status.charger_type == STANDARD_HOST)
			    || (BMT_status.charger_type == CHARGING_HOST)) {
				mt_usb_connect();
			}
		}
#endif

		printk("[BAT_thread]Cable in, CHR_Type_num=%d\r\n", BMT_status.charger_type);

	} else {
		wake_unlock(&battery_suspend_lock);

		BMT_status.charger_exist = KAL_FALSE;
		BMT_status.charger_type = CHARGER_UNKNOWN;
		BMT_status.bat_full = KAL_FALSE;
		BMT_status.bat_in_recharging_state = KAL_FALSE;
		BMT_status.bat_charging_state = CHR_PRE;
		BMT_status.total_charging_time = 0;
		BMT_status.PRE_charging_time = 0;
		BMT_status.CC_charging_time = 0;
		BMT_status.TOPOFF_charging_time = 0;
		BMT_status.POSTFULL_charging_time = 0;
		usb_error_check = KAL_FALSE;

		printk("[BAT_thread]Cable out \r\n");

		mt_usb_disconnect();
	}
}

static void mt_kpoc_power_off_check(void)
{
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	battery_xlog_printk(BAT_LOG_CRTI,
			    "[mt_kpoc_power_off_check] , chr_vol=%d, boot_mode=%d\r\n", BMT_status.charger_vol,
			    g_platform_boot_mode);
	if (g_platform_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
	    || g_platform_boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
		if ((upmu_is_chr_det() == KAL_FALSE) && (BMT_status.charger_vol < 2500))	/* vbus < 2.5V */
		{
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[bat_thread_kthread] Unplug Charger/USB In Kernel Power Off Charging Mode!  Shutdown OS!\r\n");
			battery_charging_control(CHARGING_CMD_SET_POWER_OFF, NULL);
		}
	}
#endif
}

void do_chrdet_int_task(void)
{
	u8 reg11 = 0, reg0c = 0;
	if (g_bat_init_flag == KAL_TRUE) {
		bq2589x_sed_read_byte(&reg11, BQ2589X_REG_11);
		bq2589x_sed_read_byte(&reg0c, BQ2589X_REG_0C);
		printk("%s:BQ2589X_REG_11 0x%02x, BQ2589X_REG_0C 0x%02x\n", __func__,
				reg11, reg0c);
		if (upmu_is_chr_det() == KAL_TRUE) {
			printk("[do_chrdet_int_task] charger exist!\n");
			mt_battery_charger_detect_check();
			if (BMT_status.charger_type == CHARGER_UNKNOWN)
				BMT_status.charger_exist = KAL_FALSE;
			else
				BMT_status.charger_exist = KAL_TRUE;

			wake_lock(&battery_suspend_lock);
			BMT_status.adapter_num = ADAPTER_OUTPUT_0A;

#if defined(CONFIG_POWER_EXT)
			mt_usb_connect();
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[do_chrdet_int_task] call mt_usb_connect() in EVB\n");
#elif defined(CONFIG_MTK_POWER_EXT_DETECT)
			if (KAL_TRUE == bat_is_ext_power()) {
				mt_usb_connect();
				battery_xlog_printk(BAT_LOG_CRTI,
						    "[do_chrdet_int_task] call mt_usb_connect() in EVB\n");
				return;
			}
#endif
		} else {
			printk("[do_chrdet_int_task] charger NOT exist!\n");
			battery_charging_control(CHARGING_CMD_SET_ADC_STOP, NULL);
			BMT_status.charger_exist = KAL_FALSE;
			charger_full_state = KAL_FALSE;
			BMT_status.adapter_num = ADAPTER_OUTPUT_0A;

#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
			if (g_platform_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT
			    || g_platform_boot_mode == LOW_POWER_OFF_CHARGING_BOOT) {
				battery_xlog_printk(BAT_LOG_CRTI,
						    "[pmic_thread_kthread] Unplug Charger/USB In Kernel Power Off Charging Mode!  Shutdown OS!\r\n");
				battery_charging_control(CHARGING_CMD_SET_POWER_OFF, NULL);
				/* mt_power_off(); */
			}
#endif

			wake_unlock(&battery_suspend_lock);

#if defined(CONFIG_POWER_EXT)
			mt_usb_disconnect();
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[do_chrdet_int_task] call mt_usb_disconnect() in EVB\n");
#elif defined(CONFIG_MTK_POWER_EXT_DETECT)
			if (KAL_TRUE == bat_is_ext_power()) {
				mt_usb_disconnect();
				battery_xlog_printk(BAT_LOG_CRTI,
						    "[do_chrdet_int_task] call mt_usb_disconnect() in EVB\n");
				return;
			}
#endif
			#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
				 is_ta_connect = KAL_FALSE;    
				 ta_check_chr_type = KAL_TRUE;
				 ta_cable_out_occur = KAL_TRUE;
				 ta_v_chr_org = 0;
			#endif
		}

		/* Place charger detection and battery update here is used to speed up charging icon display. */
		mt_battery_charger_detect_check();
		if (BMT_status.UI_SOC == 100 && BMT_status.charger_exist == KAL_TRUE) {
			BMT_status.bat_charging_state = CHR_BATFULL;
			BMT_status.bat_full = KAL_TRUE;
		}

		if (BMT_status.bat_vol > 0) {
			mt_battery_update_status();
		}

		wake_up_bat();
	} else {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[do_chrdet_int_task] battery thread not ready, will do after bettery init.\n");
	}
}

void BAT_thread(void)
{
	kal_bool enable = KAL_TRUE;

	mt_battery_charger_detect_check();
	mt_battery_GetBatteryData();
	if (BMT_status.charger_exist == KAL_TRUE) {
		check_battery_exist();
	}
	mt_battery_notify_check();

	if ((BMT_status.charger_exist == KAL_TRUE) || (otg_state == 1)) {
		battery_charging_control(CHARGING_CMD_RESET_WATCH_DOG_TIMER, &enable);
	}

	if (BMT_status.charger_exist == KAL_TRUE) {	
		if ((g_platform_boot_mode == LOW_POWER_OFF_CHARGING_BOOT)
				|| (g_platform_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT)) {
		}
		mt_battery_CheckBatteryStatus();
		mt_battery_charging_algorithm();
	}

	mt_battery_update_status();
	mt_kpoc_power_off_check();

	pre_charger_exist = BMT_status.charger_exist;
}

int bat_thread_kthread(void *x)
{
	ktime_t ktime = ktime_set(3, 0);	/* 10s, 10* 1000 ms */

	/* Run on a process content */
	while (1) {
		mutex_lock(&bat_mutex);
          
		if ((chargin_hw_init_done == KAL_TRUE) && (battery_suspended == KAL_FALSE))
			BAT_thread();

		mutex_unlock(&bat_mutex);

		battery_xlog_printk(BAT_LOG_FULL, "wait event \n" );

		wait_event_interruptible(bat_thread_wq, (bat_thread_timeout == KAL_TRUE));

		bat_thread_timeout = KAL_FALSE;
		hrtimer_start(&battery_kthread_timer, ktime, HRTIMER_MODE_REL);
		ktime = ktime_set(BAT_TASK_PERIOD, 0);	/* 10s, 10* 1000 ms */

		if( BMT_status.charger_exist == KAL_TRUE && BMT_status.charger_type == CHARGER_UNKNOWN){
			msleep(1000);
			wake_up_bat();
		}
	}

	return 0;
}

void bat_thread_wakeup(void)
{
	battery_xlog_printk(BAT_LOG_FULL, "******** battery : bat_thread_wakeup  ********\n");

	bat_thread_timeout = KAL_TRUE;
#ifdef MTK_ENABLE_AGING_ALGORITHM
    suspend_time = 0;
#endif
	wake_up(&bat_thread_wq);
}

void check_battery_exist(void)
{
#if defined(CONFIG_DIS_CHECK_BATTERY)
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Disable check battery exist.\n");
#else
	kal_uint32 baton_count = 0;
	kal_uint32 charging_enable = KAL_FALSE;
	kal_uint32 battery_status;
	kal_uint32 i;

	for (i = 0; i < 3; i++) {
		battery_charging_control(CHARGING_CMD_GET_BATTERY_STATUS, &battery_status);
		baton_count += battery_status;

	}

	if (baton_count >= 3) {
		if ((g_platform_boot_mode == META_BOOT) || (g_platform_boot_mode == ADVMETA_BOOT)
		    || (g_platform_boot_mode == ATE_FACTORY_BOOT)) {
			battery_xlog_printk(BAT_LOG_FULL,
					    "[BATTERY] boot mode = %d, bypass battery check\n",
					    g_platform_boot_mode);
		} else {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[BATTERY] Battery is not exist, power off FAN5405 and system (%d)\n",
					    baton_count);

			battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
			battery_charging_control(CHARGING_CMD_SET_POWER_OFF, NULL);
		}
	}
#endif
}


int charger_hv_detect_sw_thread_handler(void *unused)
{
	ktime_t ktime;
	kal_uint32 charging_enable;
	kal_uint32 hv_voltage = V_CHARGER_MAX*1000;
	kal_bool hv_status = KAL_TRUE;

	do {
		ktime = ktime_set(0, BAT_MS_TO_NS(1000));

		if (chargin_hw_init_done)
			battery_charging_control(CHARGING_CMD_SET_HV_THRESHOLD, &hv_voltage);

		wait_event_interruptible(charger_hv_detect_waiter,
					 (charger_hv_detect_flag == KAL_TRUE));

		if ((upmu_is_chr_det() == KAL_TRUE)) {
			check_battery_exist();
		}

		charger_hv_detect_flag = KAL_FALSE;

		if (chargin_hw_init_done)
			battery_charging_control(CHARGING_CMD_GET_HV_STATUS, &hv_status);

		if (hv_status == KAL_TRUE) {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[charger_hv_detect_sw_thread_handler] charger hv\n");

			charging_enable = KAL_FALSE;
			if (chargin_hw_init_done)
				battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
		} else {
			battery_xlog_printk(BAT_LOG_FULL,
					    "[charger_hv_detect_sw_thread_handler] upmu_chr_get_vcdt_hv_det() != 1\n");
		}

		hrtimer_start(&charger_hv_detect_timer, ktime, HRTIMER_MODE_REL);

	} while (!kthread_should_stop());

	return 0;
}

enum hrtimer_restart charger_hv_detect_sw_workaround(struct hrtimer *timer)
{
	charger_hv_detect_flag = KAL_TRUE;
	wake_up_interruptible(&charger_hv_detect_waiter);

	battery_xlog_printk(BAT_LOG_FULL, "[charger_hv_detect_sw_workaround]\n");

	return HRTIMER_NORESTART;
}

void charger_hv_detect_sw_workaround_init(void)
{
	ktime_t ktime;

	ktime = ktime_set(0, BAT_MS_TO_NS(2000));
	hrtimer_init(&charger_hv_detect_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	charger_hv_detect_timer.function = charger_hv_detect_sw_workaround;
	hrtimer_start(&charger_hv_detect_timer, ktime, HRTIMER_MODE_REL);

	charger_hv_detect_thread =
	    kthread_run(charger_hv_detect_sw_thread_handler, 0,
			"mtk charger_hv_detect_sw_workaround");
	if (IS_ERR(charger_hv_detect_thread)) {
		battery_xlog_printk(BAT_LOG_FULL,
				    "[%s]: failed to create charger_hv_detect_sw_workaround thread\n",
				    __func__);
	}
	check_battery_exist();
	battery_xlog_printk(BAT_LOG_CRTI, "charger_hv_detect_sw_workaround_init : done\n");
}


enum hrtimer_restart battery_kthread_hrtimer_func(struct hrtimer *timer)
{
	bat_thread_wakeup();

	return HRTIMER_NORESTART;
}

void battery_kthread_hrtimer_init(void)
{
	ktime_t ktime;

	ktime = ktime_set(1, 0);	/* 3s, 10* 1000 ms */
	hrtimer_init(&battery_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	battery_kthread_timer.function = battery_kthread_hrtimer_func;
	hrtimer_start(&battery_kthread_timer, ktime, HRTIMER_MODE_REL);

	battery_xlog_printk(BAT_LOG_CRTI, "battery_kthread_hrtimer_init : done\n");
}


static void get_charging_control(void)
{
	battery_charging_control = chr_control_interface;
}

void set_bat_thread_priority(int pri_val)
{
	struct sched_param param = { .sched_priority = 0 };

	param.sched_priority = pri_val;
	if (pri_val > 0) {
		sched_setscheduler(bat_thread_task, SCHED_FIFO, &param);
		set_current_state(TASK_INTERRUPTIBLE);
	} else {
		sched_setscheduler(bat_thread_task, SCHED_NORMAL, &param);
	}
}

static int battery_probe(struct platform_device *dev)
{
	int ret = 0;
	char* temp_strptr;

	battery_xlog_printk(BAT_LOG_CRTI, "******** battery driver probe!! ********\n");

	get_charging_control();

	battery_charging_control(CHARGING_CMD_GET_PLATFORM_BOOT_MODE, &g_platform_boot_mode);
	battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] g_platform_boot_mode = %d\n ",
			    g_platform_boot_mode);

#if defined (CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)
	if (g_boot_mode == LOW_POWER_OFF_CHARGING_BOOT || g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT) {
		temp_strptr = kzalloc(strlen(saved_command_line)+strlen(" androidboot.mode=charger")+1, GFP_KERNEL);
		strcpy(temp_strptr, saved_command_line);
		strcat(temp_strptr, " androidboot.mode=charger");
		saved_command_line = temp_strptr;
	}
#endif

	wake_lock_init(&battery_suspend_lock, WAKE_LOCK_SUSPEND, "battery suspend wakelock");
	#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
	wake_lock_init(&TA_charger_suspend_lock, WAKE_LOCK_SUSPEND, "TA charger suspend wakelock");  
	#endif

	/* Integrate with Android Battery Service */
	ret = power_supply_register(&(dev->dev), &ac_main.psy);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] power_supply_register AC Fail !!\n");
		return ret;
	}
	battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] power_supply_register AC Success !!\n");

	ret = power_supply_register(&(dev->dev), &usb_main.psy);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BAT_probe] power_supply_register USB Fail !!\n");
		return ret;
	}
	battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] power_supply_register USB Success !!\n");

	ret = power_supply_register(&(dev->dev), &wireless_main.psy);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BAT_probe] power_supply_register WIRELESS Fail !!\n");
		return ret;
	}
	battery_xlog_printk(BAT_LOG_CRTI,
			    "[BAT_probe] power_supply_register WIRELESS Success !!\n");

	ret = power_supply_register(&(dev->dev), &battery_main.psy);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "[BAT_probe] power_supply_register Battery Fail !!\n");
		return ret;
	}
	battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] power_supply_register Battery Success !!\n");

#if !defined(CONFIG_POWER_EXT)

#ifdef CONFIG_MTK_POWER_EXT_DETECT
	if (KAL_TRUE == bat_is_ext_power()) {
		battery_main.BAT_STATUS = POWER_SUPPLY_STATUS_FULL;
		battery_main.BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD;
		battery_main.BAT_PRESENT = 1;
		battery_main.BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION;
		battery_main.BAT_CAPACITY = 100;
		battery_main.BAT_batt_vol = 4200;
		battery_main.BAT_batt_temp = 220;

		g_bat_init_flag = KAL_TRUE;
		return 0;
	}
#endif
	/* For EM */
	{
		int ret_device_file = 0;
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_Charger_Type);
#if defined(CONFIG_MTK_PUMP_EXPRESS_SUPPORT) || defined(CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT)
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_Pump_Express);
#endif
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_charger_enable_major);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_charger_enable_minor);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_fake_cap_enable);
		ret_device_file = device_create_file(&(dev->dev), &dev_attr_fastcharging);
	}

	meizu_sysfslink_register(&(dev->dev));

	/* Initialization BMT Struct */
	BMT_status.bat_exist = KAL_TRUE;	/* phone must have battery */
	BMT_status.charger_exist = KAL_FALSE;	/* for default, no charger */
	BMT_status.bat_vol = 0;
	BMT_status.ICharging = 0;
	BMT_status.temperature = 0;
	BMT_status.charger_vol = 0;
	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;
	BMT_status.SOC = 0;
	BMT_status.UI_SOC = 0;
	battery_main.BAT_CAPACITY = bq27532_battery_read_soc();
	BMT_status.mcharger_enable = QUICK_CHARGER;

	BMT_status.bat_charging_state = CHR_PRE;
	BMT_status.bat_in_recharging_state = KAL_FALSE;
	BMT_status.bat_full = KAL_FALSE;
	BMT_status.adapter_num = ADAPTER_OUTPUT_0A;

	/* battery kernel thread for 10s check and charger in/out event */
	/* Replace GPT timer by hrtime */
	battery_kthread_hrtimer_init();

	bat_thread_task = kthread_create(bat_thread_kthread, NULL, "bat_thread_kthread");
	set_bat_thread_priority(-10);
	get_task_struct(bat_thread_task);
	wake_up_process(bat_thread_task);
	battery_xlog_printk(BAT_LOG_CRTI, "[battery_probe] bat_thread_kthread Done\n");

	charger_hv_detect_sw_workaround_init();

	/*LOG System Set */
	init_proc_log();
	
#else
	//keep HW alive
	charger_hv_detect_sw_workaround_init();
#endif
	g_bat_init_flag = KAL_TRUE;
	
	return 0;

}

static void battery_timer_pause(void)
{
	struct timespec xts, tom;

    //battery_xlog_printk(BAT_LOG_CRTI, "******** battery driver suspend!! ********\n" );
#ifdef CONFIG_POWER_EXT
#else

#ifdef CONFIG_MTK_POWER_EXT_DETECT
	if (KAL_TRUE == bat_is_ext_power())
		return 0;
#endif
	mutex_lock(&bat_mutex);
	//cancel timer
	hrtimer_cancel(&battery_kthread_timer);
	hrtimer_cancel(&charger_hv_detect_timer);

	battery_suspended = KAL_TRUE;
	mutex_unlock(&bat_mutex);

	battery_xlog_printk(BAT_LOG_CRTI, "@bs=1@\n" );
#endif

    get_xtime_and_monotonic_and_sleep_offset(&xts, &tom, &g_bat_time_before_sleep);
}

static void battery_timer_resume(void)
{
#ifdef CONFIG_POWER_EXT
#else
	kal_bool is_pcm_timer_trigger = KAL_FALSE;
	struct timespec xts, tom, bat_time_after_sleep;
    ktime_t ktime, hvtime;

#ifdef CONFIG_MTK_POWER_EXT_DETECT
	if (KAL_TRUE == bat_is_ext_power())
		return 0;
#endif

    ktime = ktime_set(BAT_TASK_PERIOD, 0);  // 10s, 10* 1000 ms
    hvtime = ktime_set(0, BAT_MS_TO_NS(2000));

	get_xtime_and_monotonic_and_sleep_offset(&xts, &tom, &bat_time_after_sleep);
	battery_charging_control(CHARGING_CMD_GET_IS_PCM_TIMER_TRIGGER,&is_pcm_timer_trigger);

	if(is_pcm_timer_trigger == KAL_TRUE)
	{	
		mutex_lock(&bat_mutex);
		BAT_thread();
		mutex_unlock(&bat_mutex);
	}
	else
	{
		battery_xlog_printk(BAT_LOG_CRTI, "battery resume NOT by pcm timer!!\n" );
	}

	if(g_call_state == CALL_ACTIVE && (bat_time_after_sleep.tv_sec - g_bat_time_before_sleep.tv_sec >= TALKING_SYNC_TIME))	// phone call last than x min
	{
		BMT_status.UI_SOC = bq27532_battery_read_soc();
		battery_xlog_printk(BAT_LOG_CRTI, "Sync UI SOC to SOC immediately\n" );
	}	

	mutex_lock(&bat_mutex);
    
	//restore timer
	hrtimer_start(&battery_kthread_timer, ktime, HRTIMER_MODE_REL);
	hrtimer_start(&charger_hv_detect_timer, hvtime, HRTIMER_MODE_REL);
        
	battery_suspended = KAL_FALSE;

	battery_xlog_printk(BAT_LOG_CRTI, "@bs=0@\n");
	mutex_unlock(&bat_mutex);
	
#endif
}

static int battery_remove(struct platform_device *dev)
{
	meizu_sysfslink_unregister(&(dev->dev));
	battery_xlog_printk(BAT_LOG_CRTI, "******** battery driver remove!! ********\n");

	return 0;
}

static void battery_shutdown(struct platform_device *dev)
{
	battery_xlog_printk(BAT_LOG_CRTI, "******** battery driver shutdown!! ********\n");

}

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Battery Notify API */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static ssize_t show_BatteryNotify(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[Battery] show_BatteryNotify : %x\n",
			    g_BatteryNotifyCode);

	return sprintf(buf, "%u\n", g_BatteryNotifyCode);
}

static ssize_t store_BatteryNotify(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t size)
{
	char *pvalue = NULL;
	unsigned int reg_BatteryNotifyCode = 0;
	battery_xlog_printk(BAT_LOG_CRTI, "[Battery] store_BatteryNotify\n");
	if (buf != NULL && size != 0) {
		battery_xlog_printk(BAT_LOG_CRTI, "[Battery] buf is %s and size is %Zu\n", buf,
				    size);
		reg_BatteryNotifyCode = simple_strtoul(buf, &pvalue, 16);
		g_BatteryNotifyCode = reg_BatteryNotifyCode;
		battery_xlog_printk(BAT_LOG_CRTI, "[Battery] store code : %x\n",
				    g_BatteryNotifyCode);
	}
	return size;
}

static DEVICE_ATTR(BatteryNotify, 0664, show_BatteryNotify, store_BatteryNotify);

static ssize_t show_BN_TestMode(struct device *dev, struct device_attribute *attr, char *buf)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[Battery] show_BN_TestMode : %x\n", g_BN_TestMode);
	return sprintf(buf, "%u\n", g_BN_TestMode);
}

static ssize_t store_BN_TestMode(struct device *dev, struct device_attribute *attr, const char *buf,
				 size_t size)
{
	char *pvalue = NULL;
	unsigned int reg_BN_TestMode = 0;
	battery_xlog_printk(BAT_LOG_CRTI, "[Battery] store_BN_TestMode\n");
	if (buf != NULL && size != 0) {
		battery_xlog_printk(BAT_LOG_CRTI, "[Battery] buf is %s and size is %Zu\n", buf,
				    size);
		reg_BN_TestMode = simple_strtoul(buf, &pvalue, 16);
		g_BN_TestMode = reg_BN_TestMode;
		battery_xlog_printk(BAT_LOG_CRTI, "[Battery] store g_BN_TestMode : %x\n",
				    g_BN_TestMode);
	}
	return size;
}

static DEVICE_ATTR(BN_TestMode, 0664, show_BN_TestMode, store_BN_TestMode);

static ssize_t battery_cmd_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	int len = 0, bat_tt_enable = 0, bat_thr_test_mode = 0, bat_thr_test_value = 0;
	char desc[32];

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d %d %d", &bat_tt_enable, &bat_thr_test_mode, &bat_thr_test_value) == 3) {
		g_battery_thermal_throttling_flag = bat_tt_enable;
		battery_cmd_thermal_test_mode = bat_thr_test_mode;
		battery_cmd_thermal_test_mode_value = bat_thr_test_value;

		battery_xlog_printk(BAT_LOG_CRTI,
				    "bat_tt_enable=%d, bat_thr_test_mode=%d, bat_thr_test_value=%d\n",
				    g_battery_thermal_throttling_flag,
				    battery_cmd_thermal_test_mode,
				    battery_cmd_thermal_test_mode_value);

		return count;
	} else {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "  bad argument, echo [bat_tt_enable] [bat_thr_test_mode] [bat_thr_test_value] > battery_cmd\n");
	}

	return -EINVAL;
}

static int proc_utilization_show(struct seq_file *m, void *v)
{
	seq_printf(m,
		   "=> g_battery_thermal_throttling_flag=%d,\nbattery_cmd_thermal_test_mode=%d,\nbattery_cmd_thermal_test_mode_value=%d\n",
		   g_battery_thermal_throttling_flag, battery_cmd_thermal_test_mode,
		   battery_cmd_thermal_test_mode_value);

	seq_printf(m, "=> get_usb_current_unlimited=%d,\ncmd_discharging = %d\n",
		   get_usb_current_unlimited(), cmd_discharging);
	return 0;
}

static int proc_utilization_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_utilization_show, NULL);
}

static const struct file_operations battery_cmd_proc_fops = {
	.open = proc_utilization_open,
	.read = seq_read,
	.write = battery_cmd_write,
};

static ssize_t current_cmd_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	int len = 0;
	char desc[32];
	int cmd_current_unlimited = false;
	U32 charging_enable = false;
	CHR_CURRENT_ENUM g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';

	if (sscanf(desc, "%d %d", &cmd_current_unlimited, &cmd_discharging) == 2) {
		set_usb_current_unlimited(cmd_current_unlimited);
		if (cmd_discharging == 1) {
			charging_enable = false;
			adjust_power = -1;
			battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT, &g_temp_input_CC_value);
		} else if (cmd_discharging == 0) {
			charging_enable = true;
			adjust_power = -1;
		}
		battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

		battery_xlog_printk(BAT_LOG_CRTI,
				    "[current_cmd_write] cmd_current_unlimited=%d, cmd_discharging=%d\n",
				    cmd_current_unlimited, cmd_discharging);
		return count;
	} else {
		battery_xlog_printk(BAT_LOG_CRTI, "  bad argument, echo [enable] > current_cmd\n");
	}

	return -EINVAL;
}

static int current_cmd_read(struct seq_file *m, void *v)
{
	kal_bool charging_enable = false;

	cmd_discharging = 1;
	charging_enable = false;
	adjust_power = -1;

	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	battery_xlog_printk(BAT_LOG_CRTI,
			    "[current_cmd_write] cmd_discharging=%d\n", cmd_discharging);

	return 0;
}

static int proc_utilization_open_cur_stop(struct inode *inode, struct file *file)
{
	return single_open(file, current_cmd_read, NULL);
}
static ssize_t discharging_cmd_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	int len = 0;
	char desc[32];
	U32 charging_enable = false;
    
	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)) {
		return 0;
	}
	desc[len] = '\0';
    
	if (sscanf(desc, "%d %d", &charging_enable, &adjust_power) == 2) {
		battery_xlog_printk(BAT_LOG_CRTI, "[current_cmd_write] adjust_power = %d\n", adjust_power);
		return count;
	} else {
		battery_xlog_printk(BAT_LOG_CRTI, "  bad argument, echo [enable] > current_cmd\n");
	}

    	return -EINVAL;
}

static const struct file_operations discharging_cmd_proc_fops = { 
	.open  = proc_utilization_open, 
	.read  = seq_read,
	.write = discharging_cmd_write,
};

static const struct file_operations current_cmd_proc_fops = {
	.open = proc_utilization_open_cur_stop,
	.read = seq_read,
	.write = current_cmd_write,
};

static int mt_batteryNotify_probe(struct platform_device *dev)
{
	int ret_device_file = 0;
	/* struct proc_dir_entry *entry = NULL; */
	struct proc_dir_entry *battery_dir = NULL;

	battery_xlog_printk(BAT_LOG_CRTI, "******** mt_batteryNotify_probe!! ********\n");

	ret_device_file = device_create_file(&(dev->dev), &dev_attr_BatteryNotify);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_BN_TestMode);

	battery_dir = proc_mkdir("mtk_battery_cmd", NULL);
	if (!battery_dir) {
		pr_err("[%s]: mkdir /proc/mtk_battery_cmd failed\n", __func__);
	} else {
#if 1
		proc_create("battery_cmd", S_IRUGO | S_IWUSR, battery_dir, &battery_cmd_proc_fops);
		battery_xlog_printk(BAT_LOG_CRTI, "proc_create battery_cmd_proc_fops\n");

		proc_create("current_cmd", S_IRUGO | S_IWUSR, battery_dir, &current_cmd_proc_fops);
		battery_xlog_printk(BAT_LOG_CRTI, "proc_create current_cmd_proc_fops\n");
		proc_create("discharging_cmd", S_IRUGO | S_IWUSR, battery_dir, &discharging_cmd_proc_fops);
		battery_xlog_printk(BAT_LOG_CRTI, "proc_create discharging_cmd_proc_fops\n");
            

#else
		entry = create_proc_entry("battery_cmd", S_IRUGO | S_IWUSR, battery_dir);
		if (entry) {
			entry->read_proc = battery_cmd_read;
			entry->write_proc = battery_cmd_write;
		}
#endif
	}

	battery_xlog_printk(BAT_LOG_CRTI, "******** mtk_battery_cmd!! ********\n");

	return 0;

}
#ifdef CONFIG_OF
static const struct of_device_id mt_battery_of_match[] = {
	{ .compatible = "mediatek,battery", },
	{},
};

MODULE_DEVICE_TABLE(of, mt_battery_of_match);
#endif

static int battery_pm_suspend(struct device *device)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return ret;
}

static int battery_pm_resume(struct device *device)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return ret;
}

static int battery_pm_freeze(struct device *device)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return ret;
}

static int battery_pm_restore(struct device *device)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return ret;
}

static int battery_pm_restore_noirq(struct device *device)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return ret;
}

struct dev_pm_ops battery_pm_ops = {
	.suspend = battery_pm_suspend,
	.resume = battery_pm_resume,
	.freeze = battery_pm_freeze,
	.thaw = battery_pm_restore,
	.restore = battery_pm_restore,
	.restore_noirq = battery_pm_restore_noirq,
};

#if defined(CONFIG_OF) || defined(BATTERY_MODULE_INIT)
struct platform_device battery_device = {
    .name   = "battery",
    .id        = -1,
};
#endif

static struct platform_driver battery_driver = {
	.probe = battery_probe,
	.remove = battery_remove,
	.shutdown = battery_shutdown,
	.driver = {
		.name = "battery",
		.pm = &battery_pm_ops,
	},
};

#ifdef CONFIG_OF
static int battery_dts_probe(struct platform_device *dev)
{
	int ret = 0;
	battery_xlog_printk(BAT_LOG_CRTI, "******** battery_dts_probe!! ********\n");

	battery_device.dev.of_node = dev->dev.of_node;
	ret = platform_device_register(&battery_device);
    if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "****[battery_dts_probe] Unable to register device (%d)\n", ret);
		return ret;
	}
	return 0;

}

static struct platform_driver battery_dts_driver = {
	.probe = battery_dts_probe,
	.remove = NULL,
	.shutdown = NULL,
	.driver = {
		   .name = "battery-dts",
        #ifdef CONFIG_OF 
 		.of_match_table = mt_battery_of_match,
        #endif
	},
};

//--------------------------------------------------------

static const struct of_device_id mt_bat_notify_of_match[] = {
	{ .compatible = "mediatek,bat_notify", },
	{},
};

MODULE_DEVICE_TABLE(of, mt_bat_notify_of_match);
#endif

struct platform_device MT_batteryNotify_device = {
	.name = "mt-battery",
	.id = -1,
};

static struct platform_driver mt_batteryNotify_driver = {
	.probe = mt_batteryNotify_probe,
	.driver = {
		   .name = "mt-battery",
	},
};

#ifdef CONFIG_OF
static int mt_batteryNotify_dts_probe(struct platform_device *dev)
{
	int ret = 0;
	/* struct proc_dir_entry *entry = NULL; */
	struct proc_dir_entry *battery_dir = NULL;

	battery_xlog_printk(BAT_LOG_CRTI, "******** mt_batteryNotify_dts_probe!! ********\n");

	MT_batteryNotify_device.dev.of_node = dev->dev.of_node;
	ret = platform_device_register(&MT_batteryNotify_device);
    if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "****[mt_batteryNotify_dts] Unable to register device (%d)\n", ret);
		return ret;
	}
	return 0;

}


static struct platform_driver mt_batteryNotify_dts_driver = {
	.probe = mt_batteryNotify_dts_probe,
	.driver = {
		   .name = "mt-dts-battery",
        #ifdef CONFIG_OF
		.of_match_table = mt_bat_notify_of_match,    
        #endif
	},
};
#endif
//--------------------------------------------------------

static int battery_pm_event(struct notifier_block *notifier, unsigned long pm_event, void *unused)
{
	switch(pm_event) {
	case PM_HIBERNATION_PREPARE: /* Going to hibernate */
	case PM_RESTORE_PREPARE: /* Going to restore a saved image */
	case PM_SUSPEND_PREPARE: /* Going to suspend the system */
		pr_warn("[%s] pm_event %lu\n", __func__, pm_event);
		battery_timer_pause();
		return NOTIFY_DONE;
	case PM_POST_HIBERNATION: /* Hibernation finished */
	case PM_POST_SUSPEND: /* Suspend finished */
	case PM_POST_RESTORE: /* Restore failed */
		pr_warn("[%s] pm_event %lu\n", __func__, pm_event);
		battery_timer_resume();
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static struct notifier_block battery_pm_notifier_block = {
    .notifier_call = battery_pm_event,
    .priority = 0,
};

static int __init battery_init(void)
{
	int ret;

	printk("battery_init\n");

#ifdef CONFIG_OF
	//
#else
    
#ifdef BATTERY_MODULE_INIT
	ret = platform_device_register(&battery_device);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "****[battery_device] Unable to device register(%d)\n", ret);
		return ret;
	}
#endif
#endif

	ret = platform_driver_register(&battery_driver);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "****[battery_driver] Unable to register driver (%d)\n", ret);
		return ret;
	}
	/* battery notofy UI */
#ifdef CONFIG_OF
    //
#else
	ret = platform_device_register(&MT_batteryNotify_device);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "****[mt_batteryNotify] Unable to device register(%d)\n", ret);
		return ret;
	}
#endif
	ret = platform_driver_register(&mt_batteryNotify_driver);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,
				    "****[mt_batteryNotify] Unable to register driver (%d)\n", ret);
		return ret;
	}
#ifdef CONFIG_OF
	ret = platform_driver_register(&battery_dts_driver);
	ret = platform_driver_register(&mt_batteryNotify_dts_driver);
#endif	
	ret = register_pm_notifier(&battery_pm_notifier_block);
	if (ret)
		printk("[%s] failed to register PM notifier %d\n", __func__, ret);

	battery_xlog_printk(BAT_LOG_CRTI, "****[battery_driver] Initialization : DONE !!\n");
	return 0;
}

#ifdef BATTERY_MODULE_INIT
late_initcall(battery_init);
#else
static void __exit battery_exit(void)
{
}
module_init(battery_init);
module_exit(battery_exit);
#endif

MODULE_AUTHOR("Oscar Liu");
MODULE_DESCRIPTION("Battery Device Driver");
MODULE_LICENSE("GPL");
