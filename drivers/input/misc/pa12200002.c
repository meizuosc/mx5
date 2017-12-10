#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/sysfs.h>
#include <linux/ctype.h>
#include <linux/hwmon-sysfs.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <mach/eint.h>
#include <mach/mt_gpio.h>
#include <mach/irqs.h>
#include <linux/input.h>
#include "cust_eint.h"
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <linux/sensors_io.h>
#include <linux/suspend.h>
#include <linux/meizu-sys.h>
#include <linux/input/pa12200002.h>


// led current : mA
#if CONFIG_MZ_MA01
#define PA12_LED_CURR PA12_LED_CURR150
#else
#define PA12_LED_CURR PA12_LED_CURR100
#endif

//#define	SUNLIGHT_ALS_USED
extern unsigned int mt_eint_get_polarity(unsigned int eint_num);

static struct i2c_board_info i2c_txc[] = {
	{
	    I2C_BOARD_INFO("ps", 0x1e)
	}
};

struct txc_data *txc_info = NULL;

// I2C read one byte data from register 
static int i2c_read_reg(struct i2c_client *client,u8 reg,u8 *data)
{
  	u8 databuf[2]; 
	int res = 0;
	databuf[0]= reg;
	
	mutex_lock(&txc_info->i2c_lock);
	res = i2c_master_send(client,databuf,0x1);
	if(res <= 0)
	{
		INFOR("i2c_master_send function err\n");
		mutex_unlock(&txc_info->i2c_lock);
		return res;
	}
	res = i2c_master_recv(client,data,0x1);
	if(res <= 0)
	{
		INFOR("i2c_master_recv function err\n");
		mutex_unlock(&txc_info->i2c_lock);
		return res;
	}

	mutex_unlock(&txc_info->i2c_lock);
	return 0;
}
// I2C Write one byte data to register
static int i2c_write_reg(struct i2c_client *client,u8 reg,u8 value)
{
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = reg;   
	databuf[1] = value;
	
	mutex_lock(&txc_info->i2c_lock);
	res = i2c_master_send(client,databuf,0x2);
	if (res < 0){
		INFOR("i2c_master_send function err\n");
		mutex_unlock(&txc_info->i2c_lock);
		return res;
	}
	mutex_unlock(&txc_info->i2c_lock);
	return 0;
}

static int pa122_read_file(char *filename,u8* param) 
{
	struct file  *fop;
	mm_segment_t old_fs;

	fop = filp_open(filename,O_RDONLY,0);
	if(IS_ERR(fop))
	{
		INFOR("Filp_open error!! Path = %s\n",filename);
		return -1;
	}

	old_fs = get_fs();  
	set_fs(get_ds()); //set_fs(KERNEL_DS);  
	     
	fop->f_op->llseek(fop,0,0);
	fop->f_op->read(fop, param, 1, &fop->f_pos);     

	set_fs(old_fs);  

	filp_close(fop,NULL);

	return 0;
}

static ssize_t pa122_write_file(char *filename,u8* param) 
{
	struct file  *fop;
	mm_segment_t old_fs;	 

	fop = filp_open(filename,O_CREAT | O_RDWR,0664);
	if(IS_ERR(fop))
	{
		INFOR("Create file error!! Path = %s\n",filename);
		return -1;
	}

	old_fs = get_fs();  
	set_fs(get_ds()); //set_fs(KERNEL_DS);  

	fop->f_op->llseek(fop, 0, SEEK_SET);
	fop->f_op->write(fop, (char *)param, sizeof(param), &fop->f_pos);   
	set_fs(old_fs);  

	filp_close(fop,NULL);

	return 0;
}

static int pa122_load_calibration_param(struct i2c_client *client)
{
	int res;
	u8 buftemp[1] = {0};
	struct txc_data *data = i2c_get_clientdata(client);

	/* Check ps calibration file */
	if(pa122_read_file(PS_CAL_FILE_PATH,buftemp) < 0)
	{
		INFOR("Use Default ps offset , x-talk = %d\n", data->ps_calibvalue);
		i2c_write_reg(client, REG_PS_OFFSET, data->ps_calibvalue); 
		return -1;
	}
	else
	{
		INFOR("Use PS Cal file , x-talk = %d\n",buftemp[0]);
		data->ps_calibvalue = buftemp[0];
		/* Write ps offset value to register 0x10 */
		i2c_write_reg(client, REG_PS_OFFSET, data->ps_calibvalue); 
	}
	return 0;
}

static int pa12201001_init(struct txc_data *data)
{
	int ret = 0;
	u8 sendvalue = 0;
	struct i2c_client *client = data->client;

	data->pa122_sys_run_cal = 0;
	data->ps_calibvalue = PA12_PS_OFFSET_DEFAULT + PA12_PS_OFFSET_EXTRA;
	data->fast_calib_flag = 0;
	data->ps_data = PS_UNKONW;
	data->irq_wake_enabled = 0;
	data->debug = 0;

#ifdef	SUNLIGHT_ALS_USED
	/* Sun light */
	ret = i2c_write_reg(client,REG_CFG0, PA12_ALS_GAIN4000);
#endif

	ret = i2c_write_reg(client,REG_CFG1,PA12_LED_CURR | PA12_PS_PRST4);// REG_CFG3, invalid opration PA12_LED_CURR==150mA

	ret = i2c_write_reg(client,REG_PS_SET, 0x03); //PSET, Normal Mode

      sendvalue= PA12_PS_INT_HYSTERESIS | PA12_PS_PERIOD12;
      ret=i2c_write_reg(client,REG_CFG3,sendvalue);

 // Set PS threshold
      sendvalue=PA12_PS_FAR_TH_HIGH;
      ret=i2c_write_reg(client,REG_PS_TH,sendvalue); //set TH threshold
	    
      sendvalue=PA12_PS_NEAR_TH_LOW;	
      ret=i2c_write_reg(client,REG_PS_TL,sendvalue); //set TL threshold
      
      ret = i2c_write_reg(client, REG_PS_OFFSET, PA12_PS_OFFSET_DEFAULT);
      if(ret < 0)
      {	
    	  INFOR("i2c_send function err\n");
    	  return ret;
      }

      return ret;
}

#ifdef	SUNLIGHT_ALS_USED
static int pa12201001_set_ps_mode(struct i2c_client *client)
{
  u8 sendvalue=0, regdata = 0;
  int res = 0;
  
  if (txc_info->ps_enable) {
      sendvalue= PA12_LED_CURR | PA12_PS_PRST4 | PA12_ALS_PRST4;
      res=i2c_write_reg(client,REG_CFG1,sendvalue);

    // Interrupt Setting	 
      res=i2c_write_reg(client,REG_CFG2, (PA12_INT_ALS_PS_BOTH| PA12_PS_MODE_NORMAL
			      | PA12_PS_INTF_INACTIVE | PA12_ALS_INTF_INACTIVE)); //set int mode

  /* Sun Light */
  	res = i2c_write_reg(client, REG_ALS_TH_LSB, PA12_ALS_TH_LSB_SUN_LIGHT_ON);
  	res = i2c_write_reg(client, REG_ALS_TH_MSB, PA12_ALS_TH_MSB_SUN_LIGHT_ON);
  	res = i2c_write_reg(client, REG_ALS_TL_LSB, PA12_ALS_TL_LSB_SUN_LIGHT_ON);
	res = i2c_write_reg(client, REG_ALS_TL_MSB, PA12_ALS_TL_MSB_SUN_LIGHT_ON);
  } else {
      sendvalue= PA12_LED_CURR | PA12_PS_PRST4;
      res=i2c_write_reg(client,REG_CFG1,sendvalue);

    // Interrupt Setting	 
      res = i2c_read_reg(client, REG_CFG2, &regdata);
      sendvalue = (regdata & 0x03) | PA12_INT_PS | PA12_PS_MODE_NORMAL;
      res=i2c_write_reg(client,REG_CFG2, sendvalue); //set int mode
  }
  return 0 ;
}


#else

static int pa12201001_set_ps_mode(struct i2c_client *client)
{
  u8 sendvalue=0, regdata = 0;
  int res = 0;
  
  sendvalue= PA12_LED_CURR | PA12_PS_PRST4;
  res=i2c_write_reg(client,REG_CFG1,sendvalue);

  /* Prevent interrupt form ALS, because ALS in on to prevent ps data being non-stable under specific angle in outdoor place */
  res = i2c_write_reg(client, REG_ALS_TH_LSB, 0xFF);
  res = i2c_write_reg(client, REG_ALS_TH_MSB, 0xFF);
  res = i2c_write_reg(client, REG_ALS_TL_LSB, 0x00);
  res = i2c_write_reg(client, REG_ALS_TL_MSB, 0x00);

  // Interrupt Setting	   
      res = i2c_read_reg(client, REG_CFG2, &regdata);
      sendvalue = (regdata & 0x03) | PA12_INT_PS | PA12_PS_MODE_NORMAL;//PS interrup only and use normal mode
      res=i2c_write_reg(client,REG_CFG2, sendvalue); //set int mode

  return 0 ;
}

#endif


//PS enable function
int pa12201001_enable_ps(struct i2c_client *client, int enable)
{
  int res;
  u8 regdata=0;
  u8 sendvalue=0;
	
  if(enable == 1) //PS ON
  {
     INFOR("pa12201001 enable ps sensor\n");
     res=i2c_read_reg(client,REG_CFG0,&regdata); //Read Status
     if(res<0){
    	 INFOR("i2c_read function err\n");
		return res;
     }else{

     	//sendvalue=regdata & 0xFD; //clear bit
     	//sendvalue=sendvalue | 0x02; //0x02 PS Flag
     	sendvalue=regdata & 0xFC; //clear bit-0 & bit-1
#ifdef	SUNLIGHT_ALS_USED
     	sendvalue=sendvalue | 0x03; //0x03 PS On & ALS On
#else
		//sendvalue=sendvalue | 0x02; //0x02 PS On
     	sendvalue=sendvalue | 0x03; //0x02 PS On & ALS On to prevent strong IR causing ps data being non-stable under specific angle in outdoor place
#endif

     	res=i2c_write_reg(client,REG_CFG0,sendvalue); //Write PS enable 
     
    	 if(res<0){
    		 INFOR("i2c_write function err\n");
		     return res;
          }	  		 	
         res=i2c_read_reg(client,REG_CFG0,&regdata); //Read Status
     	APS_LOG("CFG0 Status: %d\n",regdata);
      }
    }else{       //PS OFF
			
       INFOR("pa12201001 disable ps sensor\n");
       res=i2c_read_reg(client,REG_CFG0,&regdata); //Read Status
       if(res<0){
    	   INFOR("i2c_read function err\n");
		  return res;
       }else{
    	   INFOR("CFG0 Status: %d\n",regdata);
		
          //sendvalue=regdata & 0xFD; //clear bit
          sendvalue=regdata & 0xFC; //clear bit-0 & bit-1
          res=i2c_write_reg(client,REG_CFG0,sendvalue); //Write PS disable

	      res = i2c_read_reg(client, REG_CFG2, &regdata);
	      sendvalue = regdata & 0xFD; //Clear PS interrupt flag

	      usleep_range(15000, 15000);
		
          if(res<0){
        	  INFOR("i2c_write function err\n");
			 return res;
		 }	  	
       }
     }
	
     return 0;
} 

//Read PS Count : 8 bit
int pa12201001_read_ps(struct i2c_client *client, u8 *data)
{
   int res;
   u8 psdata = 0;
	
  // APS_FUN(f);
    res = i2c_read_reg(client,REG_PS_DATA,data); //Read PS Data
    //psdata = i2c_smbus_read_byte_data(client, REG_PS_DATA); 
   if(res < 0){
	   INFOR("i2c_send function err\n");
   }

   return res;
}

//Read ALS Count : 16 bit
int pa12201001_read_als(struct i2c_client *client, u16 *data)
{
   int res;
   u8 LSB = 0;
   u8 MSB = 0;
	
  // APS_FUN(f);
    res = i2c_read_reg(client, REG_ALS_DATA_LSB, &LSB); //Read PS Data
    res = i2c_read_reg(client, REG_ALS_DATA_MSB, &MSB); //Read PS Data
    *data = (MSB << 8) | LSB; 
    //psdata = i2c_smbus_read_byte_data(client, REG_PS_DATA); 
   if(res < 0){
	   INFOR("i2c_send function err\n");
   }

   return res;
}

void pa12_swap(u8 *x, u8 *y)
{
        u8 temp = *x;
        *x = *y;
        *y = temp;
}

static int pa122_run_fast_calibration(struct i2c_client *client)
{

	struct txc_data *data = i2c_get_clientdata(client);
	int i = 0;
	int j = 0;	
	u16 sum_of_pdata = 0;
	u8  xtalk_temp = 0;
    	u8 temp_pdata[4], cfg0data = 0,cfg2data = 0,cfg3data = 0;
   	unsigned int ArySize = 4;
	
   	INFOR("START proximity sensor calibration\n");

	i2c_write_reg(client, REG_PS_TH, 0xFF);
	i2c_write_reg(client, REG_PS_TL, 0);

	i2c_read_reg(client, REG_CFG0, &cfg0data);
	i2c_read_reg(client, REG_CFG2, &cfg2data);
	i2c_read_reg(client, REG_CFG3, &cfg3data);

	/*Offset mode & disable intr from ps*/
	i2c_write_reg(client, REG_CFG2, cfg2data & 0x33); 	
	
	/*PS sleep time 6.5ms */
	i2c_write_reg(client, REG_CFG3, cfg3data & 0xC7); 	

	/*Set crosstalk = 0*/
	i2c_write_reg(client, REG_PS_OFFSET, 0x00); 

	/*PS On*/
	//i2c_write_reg(client, REG_CFG0, cfg0data | 0x02); 
	
	/*PS On ALS On to prevent strong IR causing ps data being non-stable under specific angle in outdoor place*/
	i2c_write_reg(client, REG_CFG0, cfg0data | 0x03); 

	usleep_range(50 * 1000, 50 * 1000);

	for(i = 0; i < 4; i++)
	{
		usleep_range(7000, 7000);
		i2c_read_reg(client,REG_PS_DATA,temp_pdata+i);
		INFOR("temp_data = %d\n", temp_pdata[i]);
	}	
	
	/* pdata sorting */
	for (i = 0; i < ArySize - 1; i++)
		for (j = i+1; j < ArySize; j++)
			if (temp_pdata[i] > temp_pdata[j])
				pa12_swap(temp_pdata + i, temp_pdata + j);	
	
	/* calculate the cross-talk using central 2 data */
	for (i = 1; i < 3; i++) 
	{
		INFOR("%s: temp_pdata = %d\n", __func__, temp_pdata[i]);
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}

	xtalk_temp = sum_of_pdata/2;
	INFOR("%s: sum_of_pdata = %d   calibvalue = %d\n",
                        __func__, sum_of_pdata, xtalk_temp);
	
	/* Restore Data */
	i2c_write_reg(client, REG_CFG0, cfg0data);
	i2c_write_reg(client, REG_CFG2, cfg2data | 0xC0); //make sure return normal mode
	i2c_write_reg(client, REG_CFG3, cfg3data);

	i2c_write_reg(client, REG_PS_TH, PA12_PS_FAR_TH_HIGH);
	i2c_write_reg(client, REG_PS_TL, PA12_PS_NEAR_TH_LOW);

	if (((xtalk_temp + PA12_PS_OFFSET_EXTRA) > data->factory_calibvalue) && (xtalk_temp < PA12_PS_OFFSET_MAX) 
				&& xtalk_temp + PA12_PS_OFFSET_EXTRA < data->factory_calibvalue + 15)
	{ 	
		INFOR("Fast calibrated data=%d\n",xtalk_temp);
		data->fast_calib_flag = 1;
		data->fast_calibvalue = xtalk_temp + PA12_PS_OFFSET_EXTRA;
		/* Write offset value to 0x10 */
		i2c_write_reg(client, REG_PS_OFFSET, xtalk_temp + PA12_PS_OFFSET_EXTRA);
		return xtalk_temp + PA12_PS_OFFSET_EXTRA;
	}
	else
	{
		INFOR("Fast calibration fail, calibvalue=%d, use factory_calibvalue %d\n",xtalk_temp, data->factory_calibvalue);
		data->fast_calib_flag = 0;
		i2c_write_reg(client, REG_PS_OFFSET, data->factory_calibvalue);
		xtalk_temp = data->factory_calibvalue;
		
		return xtalk_temp;
    }
}

static int pa122_run_calibration(struct txc_data *data)
{
    struct i2c_client *client = data->client;
	int i, j;	
	int ret;
	u16 sum_of_pdata = 0;
	u8 temp_pdata[20],buftemp[1],cfg0data=0,cfg2data=0;
	unsigned int ArySize = 20;
	unsigned int cal_check_flag = 0;	
	u8 value=0;
	int calibvalue;

	INFOR("%s: START proximity sensor calibration\n", __func__);

	i2c_write_reg(client, REG_PS_TH, 0xFF);
	i2c_write_reg(client, REG_PS_TL, 0);

RECALIBRATION:
	data->pa122_sys_run_cal = 0;
	
	sum_of_pdata = 0;

	ret = i2c_read_reg(client, REG_CFG0, &cfg0data);
	ret = i2c_read_reg(client, REG_CFG2, &cfg2data);
	
	/*Set to offset mode & disable interrupt from ps*/
	ret = i2c_write_reg(client, REG_CFG2, cfg2data & 0x33); 

	/*Set crosstalk = 0*/
	ret = i2c_write_reg(client, REG_PS_OFFSET, 0x00);
	if (ret < 0) {
		INFOR("%s: txc_write error\n", __func__);
	    /* Restore CFG2 (Normal mode) and Measure base x-talk */
	    ret = i2c_write_reg(client, REG_CFG0, cfg0data);
	    ret = i2c_write_reg(client, REG_CFG2, cfg2data | 0xC0); 
	    return ret;
	}

	/*PS On*/
	ret = i2c_write_reg(client, REG_CFG0, cfg0data | 0x02); 
	usleep_range(50000, 50000);

	for(i = 0; i < 20; i++)
	{
		usleep_range(15000, 15000);
		ret = i2c_read_reg(client,REG_PS_DATA,temp_pdata+i);
		INFOR("temp_data = %d\n", temp_pdata[i]);
	}	
	
	/* pdata sorting */
	for (i = 0; i < ArySize - 1; i++)
		for (j = i+1; j < ArySize; j++)
			if (temp_pdata[i] > temp_pdata[j])
				pa12_swap(temp_pdata + i, temp_pdata + j);	
	
	/* calculate the cross-talk using central 10 data */
	for (i = 5; i < 15; i++) 
	{
		INFOR("%s: temp_pdata = %d\n", __func__, temp_pdata[i]);
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}
	calibvalue = sum_of_pdata/10;

	/* Restore CFG2 (Normal mode) and Measure base x-talk */
	ret = i2c_write_reg(client, REG_CFG0, cfg0data);
	ret = i2c_write_reg(client, REG_CFG2, cfg2data | 0xC0); 

	i2c_write_reg(client, REG_PS_TH, PA12_PS_FAR_TH_HIGH);
	i2c_write_reg(client, REG_PS_TL, PA12_PS_NEAR_TH_LOW);
 	
	if (calibvalue < PA12_PS_OFFSET_MAX) {
		data->ps_calibvalue = calibvalue + PA12_PS_OFFSET_EXTRA;
	} else {
		INFOR("%s: invalid calibrated data, calibvalue %d\n", __func__, calibvalue);

		if(cal_check_flag == 0)
		{
			INFOR("RECALIBRATION start\n");
			cal_check_flag = 1;
			goto RECALIBRATION;
		}
		else
		{
			INFOR(" CALIBRATION FAIL -> cross_talk is set to DEFAULT\n");
			ret = i2c_write_reg(client, REG_PS_OFFSET, data->factory_calibvalue);

			data->pa122_sys_run_cal = 0;
			return -EINVAL;
		}
	}

CROSSTALKBASE_RECALIBRATION:
	
	/*PS On*/
	//ret = i2c_write_reg(client, REG_CFG0, cfg0data | 0x02); 
	/*PS On ALS On to prevent strong IR causing ps data being non-stable under specific angle in outdoor place*/
	ret = i2c_write_reg(client, REG_CFG0, cfg0data | 0x03); 

	/*Write offset value to register 0x10*/
	ret = i2c_write_reg(client, REG_PS_OFFSET, data->ps_calibvalue);
	
	for(i = 0; i < 10; i++)
	{
		usleep_range(15000, 15000);
		ret = i2c_read_reg(client,REG_PS_DATA,temp_pdata+i);
		INFOR("temp_data = %d\n", temp_pdata[i]);
	}	
 
     	/* pdata sorting */
	for (i = 0; i < 9; i++)
	{
	    for (j = i+1; j < 10; j++)
	    {
		if (temp_pdata[i] > temp_pdata[j])
			pa12_swap(temp_pdata + i, temp_pdata + j);   
	    }
	}

	/* calculate the cross-talk_base using central 5 data */
	sum_of_pdata = 0;

	for (i = 3; i < 8; i++) 
	{
		INFOR("temp_pdata = %d\n", temp_pdata[i]);
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}
	
	data->calibvalue_base = sum_of_pdata/5;

	if(data->calibvalue_base > 0) 
	{
		if (data->calibvalue_base >= 8)
			data->ps_calibvalue += data->calibvalue_base/8;
		else
			data->ps_calibvalue += 1;
		goto CROSSTALKBASE_RECALIBRATION;
	} else if (data->calibvalue_base == 0) {
		data->factory_calibvalue = data->ps_calibvalue;
		data->pa122_sys_run_cal = 1;
	}

  	 /* Restore CFG0  */
	ret = i2c_write_reg(client, REG_CFG0, cfg0data);

	INFOR("%s: FINISH proximity sensor calibration, %d\n", __func__, data->ps_calibvalue);
	return ret;
}

static void txc_set_enable(struct txc_data *txc, int enable)
{
	struct i2c_client *client = txc->client;
	int ret,ps_data, en;

	mutex_lock(&txc->enable_lock);
	if( txc->wakeup_enable||txc->non_wake_up_en ) {
		en = 1;
	} else {
		en = 0;
	}

	if ( enable && (!txc->ps_enable) ) {
		mt_eint_mask(CUST_EINT_INTI_INT_NUM);
		pa12201001_set_ps_mode(client);
#if defined(PA12_FAST_CAL)
		pa122_run_fast_calibration(client);
		//i2c_write_reg(client, REG_PS_OFFSET, txc_info->factory_calibvalue);
#endif
	}

	INFOR("enable(%d),non/wake_en(%d),ps_en(%d)  ***before\n",enable, en,txc->ps_enable);

	if( enable ) {
		mt_eint_mask(CUST_EINT_INTI_INT_NUM);
		if( (!txc->ps_enable) ) {
			ret = pa12201001_enable_ps(client, 1);
		}
		usleep_range(50000, 50000); //Must wait 50ms for ps data to refresh. if ps irq is running,wait
		wake_lock_timeout(&txc->ps_wake_lock, 2*HZ);
		input_report_abs(txc->input_dev, ABS_DISTANCE, PS_UNKONW);
		input_sync(txc->input_dev);
		usleep_range(5000, 5000);

		if( txc->ps_data==PS_UNKONW ) {
			ps_data = mt_get_gpio_in(GPIO_IR_EINT_PIN);
			ps_data = (ps_data==1)?PS_FAR:PS_NEAR;
			input_report_abs(txc->input_dev, ABS_DISTANCE, ps_data);
			input_sync(txc->input_dev);
			INFOR("***********%s,gpio:%d***********\n", ps_data==PS_NEAR?"near":"far", mt_get_gpio_in(GPIO_IR_EINT_PIN));
		} else {
			input_report_abs(txc->input_dev, ABS_DISTANCE, txc->ps_data);
			input_sync(txc->input_dev);
			INFOR("***********%s,gpio:%d***********\n",
					txc->ps_data==PS_NEAR?"near":(txc->ps_data==PS_FAR?"far":"near"), mt_get_gpio_in(GPIO_IR_EINT_PIN));
		}

		txc->ps_enable = 1;
		mt_eint_unmask(CUST_EINT_INTI_INT_NUM);
	} else {
		if( !en ) {
			ret = pa12201001_enable_ps(client, 0);
			if( ret<0 ) {
				INFOR("disable ps failed\n");
			}
			input_report_abs(txc->input_dev, ABS_DISTANCE, PS_UNKONW);
			input_sync(txc->input_dev);
			txc->ps_data = PS_UNKONW;
			txc->ps_enable = 0;
			INFOR("***********disable all ps,unkown***********\n");
		}
	}
	INFOR("enable(%d),non/wake_en(%d),ps_en(%d)  ***after\n",enable, en,txc->ps_enable);

	mutex_unlock(&txc->enable_lock);
}

static ssize_t txc_ps_enable_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int enabled;

	enabled = txc_info->ps_enable;

	return sprintf(buf, "%d, %d, %d\n", enabled, txc_info->factory_calibvalue, txc_info->ps_calibvalue);
}

static ssize_t txc_ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int enable = simple_strtol(buf, NULL, 10);

	INFOR("%s:enable %d\n", __func__, enable);
	txc_set_enable(txc_info, enable);

	return count;
}

static ssize_t txc_ps_data_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	u8 ps_data;
	    
	ret = i2c_read_reg(client, REG_PS_DATA, &ps_data); //Read PS Data

	INFOR("ps data is %d \n", ps_data);

	return sprintf(buf, "%d\n", ps_data);
}

static ssize_t txc_als_data_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	u8 msb, lsb;
	u16 als_data;
	    
	ret = i2c_read_reg(client, REG_ALS_DATA_LSB, &lsb); 
	ret = i2c_read_reg(client, REG_ALS_DATA_MSB, &msb);

	als_data = (msb << 8) | lsb;
	INFOR("als data is %d \n", als_data);

	return sprintf(buf, "%d\n", als_data);
}


static ssize_t pa12200001_show_reg(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 regdata;
	int res=0;
	int count=0;
	int i=0	;

	for(i;i <17 ;i++)
	{
		res=i2c_read_reg(client,0x00+i,&regdata);
		if(res<0)
		{
		   break;
		}
		else
		count+=sprintf(buf+count,"[%x] = (%x)\n",0x00+i,regdata);
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t pa12200001_store_send(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned int addr, cmd;


	if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		INFOR("invalid format: '%s'\n", buf);
		return 0;
	}

  i2c_write_reg(client, addr, cmd);
	//****************************
	return count;
}

static ssize_t txc_calibration_store(struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	int calibration = simple_strtol(buf, NULL, 10);
    
	if (calibration) {
	    	pa122_run_calibration(txc_info);
    	}

	return count;
}

static ssize_t txc_calibvalue_result_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int res = 0;
	if (txc_info->pa122_sys_run_cal == 0)
		res = -1;
	else
		res = txc_info->ps_calibvalue;

	return sprintf(buf, "%d\n", res);
}

static ssize_t txc_calibvalue_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int res = 0;
	if (txc_info->pa122_sys_run_cal == 0)
		res = -1;
	else
		res = txc_info->ps_calibvalue;

	return sprintf(buf, "%d\n", txc_info->factory_calibvalue);
}

static ssize_t txc_calibvalue_store(struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	int ret;

	txc_info->factory_calibvalue = simple_strtol(buf, NULL, 10);

	return count;
}

static ssize_t txc_batch_store(struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	int ps_batch = simple_strtol(buf, NULL, 10);

	if (ps_batch == 1) {
	    input_report_abs(txc_info->input_dev, ABS_DISTANCE, PS_UNKONW);
	    input_sync(txc_info->input_dev);
	}
	return count;
}

static ssize_t gesture_show(struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	return sprintf(buf, "%d\n", txc_info->gesture);
}

static ssize_t gesture_store(struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	txc_info->gesture = simple_strtol(buf, NULL, 10);

	return count;
}

static ssize_t mobile_leather_show(struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	return sprintf(buf, "%d\n", txc_info->mobile_leather);
}

static ssize_t mobile_leather_store(struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	txc_info->mobile_leather = simple_strtol(buf, NULL, 10);

	return count;
}

static ssize_t non_wakeup_enable_show(struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	return sprintf(buf, "%d\n", txc_info->non_wake_up_en);
}

static ssize_t non_wakeup_enable_store(struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	mutex_lock(&txc_info->enable_lock);
	txc_info->non_wake_up_en = simple_strtol(buf, NULL, 10);
	mutex_unlock(&txc_info->enable_lock);

	return count;
}

static ssize_t wakeup_enable_show(struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	return sprintf(buf, "%d\n", txc_info->wakeup_enable);
}

static ssize_t wakeup_enable_store(struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	mutex_lock(&txc_info->enable_lock);
	txc_info->wakeup_enable = simple_strtol(buf, NULL, 10);
	mutex_unlock(&txc_info->enable_lock);

	return count;
}

static ssize_t debug_show(struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	return sprintf(buf, "%d\n", txc_info->debug);
}

static ssize_t debug_store(struct device *dev, struct device_attribute *attr, const char *buf,
	size_t count)
{
	txc_info->debug = simple_strtol(buf, NULL, 10);

	return count;
}

static ssize_t gpio_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", txc_info->irq_gpio);
}

/* sysfs attributes operation function*/
static DEVICE_ATTR(ps_enable, 0664, txc_ps_enable_show, txc_ps_enable_store);
static DEVICE_ATTR(ps_data, 0664, txc_ps_data_show, NULL);
static DEVICE_ATTR(als_data, 0664, txc_als_data_show, NULL);
static DEVICE_ATTR(reg, 0664, pa12200001_show_reg, pa12200001_store_send);
static DEVICE_ATTR(ps_calibration, 0664, NULL, txc_calibration_store);
static DEVICE_ATTR(ps_calibbias, 0664, txc_calibvalue_result_show, NULL);
static DEVICE_ATTR(ps_offset, 0664, txc_calibvalue_show, txc_calibvalue_store);
static DEVICE_ATTR(ps_batch, 0664, NULL, txc_batch_store);
static DEVICE_ATTR(ps_gesture, 0664, gesture_show, gesture_store);
static DEVICE_ATTR(mobile_leather, 0664, mobile_leather_show, mobile_leather_store);
static DEVICE_ATTR(ps_non_wakeup_enable, 0664, non_wakeup_enable_show, non_wakeup_enable_store);
static DEVICE_ATTR(ps_wakeup_enable, 	 0664, wakeup_enable_show, wakeup_enable_store);
static DEVICE_ATTR(debug, 0664, debug_show, debug_store);
static DEVICE_ATTR(ps_irq_gpio, 0664, gpio_status, NULL);

static struct attribute *txc_attributes[] = {
	&dev_attr_ps_enable.attr,
	&dev_attr_ps_data.attr,
	&dev_attr_als_data.attr,
	&dev_attr_reg.attr,
	&dev_attr_ps_calibration.attr,
	&dev_attr_ps_calibbias.attr,
	&dev_attr_ps_offset.attr,
	&dev_attr_ps_batch.attr,
	&dev_attr_ps_gesture.attr,
	&dev_attr_mobile_leather.attr,
	&dev_attr_ps_non_wakeup_enable.attr,
	&dev_attr_ps_wakeup_enable.attr,
	&dev_attr_debug.attr,
	&dev_attr_ps_irq_gpio.attr,
	NULL,
};

static struct attribute_group txc_attribute_group = {
	.attrs = txc_attributes,
};

static void txc_ps_handler(struct work_struct *work)
{
    struct txc_data *txc = container_of(work, struct txc_data, ps_dwork.work);
    struct i2c_client *client = txc_info->client;
    u8 psdata=0;
    u16 alsdata = 0;
    int ps_data = PS_UNKONW;
    u8 sendvalue;
    int res;
    u8 data;
    int ret;

    ret = pa12201001_read_ps(client,&psdata);
#ifdef	SUNLIGHT_ALS_USED
    ret = pa12201001_read_als(client, &alsdata);
#endif

    if (ret < 0) {
		INFOR(" txc_write error\n");
		goto enable_irq;
    }
	
#ifdef	SUNLIGHT_ALS_USED
	if(alsdata > PA12_SUN_LIGHT_ON)
	{
		i2c_write_reg(client, REG_ALS_TH_LSB, PA12_ALS_TH_LSB_SUN_LIGHT_OFF);
		i2c_write_reg(client, REG_ALS_TH_MSB, PA12_ALS_TH_MSB_SUN_LIGHT_OFF);
		i2c_write_reg(client, REG_ALS_TL_LSB, PA12_ALS_TL_LSB_SUN_LIGHT_OFF);
		i2c_write_reg(client, REG_ALS_TL_MSB, PA12_ALS_TL_MSB_SUN_LIGHT_OFF);
		i2c_write_reg(client, REG_PS_OFFSET, 0);
	}
	else if(alsdata < PA12_SUN_LIGHT_OFF)
	{
		i2c_write_reg(client, REG_ALS_TH_LSB, PA12_ALS_TH_LSB_SUN_LIGHT_ON);
		i2c_write_reg(client, REG_ALS_TH_MSB, PA12_ALS_TH_MSB_SUN_LIGHT_ON);
		i2c_write_reg(client, REG_ALS_TL_LSB, PA12_ALS_TL_LSB_SUN_LIGHT_ON);
		i2c_write_reg(client, REG_ALS_TL_MSB, PA12_ALS_TL_MSB_SUN_LIGHT_ON);
		if (txc->fast_calib_flag)
			i2c_write_reg(client, REG_PS_OFFSET, txc->fast_calibvalue);
		else
			i2c_write_reg(client, REG_PS_OFFSET, txc->ps_calibvalue);
	}
#endif	

	INFOR("irq func ps_data:%d-%s prev; th:%d\n", txc->ps_data, txc->ps_data==0?"near":(txc->ps_data==1?"far":"unkown"), psdata );

	if (txc->ps_data == PS_UNKONW || txc->ps_data == PS_FAR) {
		if(psdata > PA12_PS_FAR_TH_HIGH){
		    mt_eint_set_polarity(CUST_EINT_INTI_INT_NUM, EINTF_TRIGGER_RISING);
		    ps_data = PS_NEAR;

		    sendvalue= PA12_LED_CURR | PA12_PS_PRST2 | PA12_ALS_PRST4;
		    res=i2c_write_reg(client,REG_CFG1,sendvalue);
		    if (res < 0) {
			    INFOR("txc_write error\n");
			    goto enable_irq;
		    }

		    sendvalue= PA12_PS_INT_HYSTERESIS | PA12_PS_PERIOD6;
		    res=i2c_write_reg(client,REG_CFG3,sendvalue);
		    if (res < 0) {
		    	INFOR("txc_write error\n");
			    goto enable_irq;
		    }
		} else if (psdata < PA12_PS_NEAR_TH_LOW) {
			ps_data = PS_FAR;
			mt_eint_set_polarity(CUST_EINT_INTI_INT_NUM, MT_EINT_POL_NEG);
		}
	} else if (txc->ps_data == PS_NEAR) {
		if(psdata < PA12_PS_NEAR_TH_LOW){
			mt_eint_set_polarity(CUST_EINT_INTI_INT_NUM, MT_EINT_POL_NEG);
			ps_data = PS_FAR;
			
#ifdef	SUNLIGHT_ALS_USED
			sendvalue= PA12_LED_CURR | PA12_PS_PRST4 | PA12_ALS_PRST4;
#else
			sendvalue= PA12_LED_CURR | PA12_PS_PRST4;
#endif
			res=i2c_write_reg(client,REG_CFG1,sendvalue);
			if (res < 0) {
				INFOR("txc_write error\n");
				goto enable_irq;
			}
			sendvalue= PA12_PS_INT_HYSTERESIS | PA12_PS_PERIOD12;
		    res=i2c_write_reg(client,REG_CFG3,sendvalue);
		    if (res < 0) {
		    	INFOR("txc_write error\n");
			    goto enable_irq;
		   	}
		}
	}

	if (txc->ps_data != ps_data) {
		wake_lock_timeout(&txc->ps_wake_lock, 2*HZ);
		txc->ps_data = ps_data;
		input_report_abs(txc->input_dev, ABS_DISTANCE, ps_data);
		input_sync(txc->input_dev);
		if (ps_data == PS_NEAR) {
			INFOR("***********near***********\n");
			txc_info->irq_gpio = 0;
		} else if (ps_data == PS_FAR) {
			INFOR("****************far***************\n");
			txc_info->irq_gpio = 1;
		}
	}
	ret = i2c_read_reg(txc->client, REG_CFG2, &data);
	if (ret < 0) {
		INFOR("%s: txc_read error\n", __func__);
	    goto enable_irq;
	}
	data &= 0xfe;
	ret = i2c_write_reg(txc->client, REG_CFG2, data);
	if (ret < 0) {
		INFOR("%s: txc_write error\n", __func__);
	    goto enable_irq;
	}

enable_irq:
    mt_eint_unmask(CUST_EINT_INTI_INT_NUM);
}

static void txc_irq_handler(void)
{
	queue_delayed_work(txc_info->queue, &txc_info->ps_dwork, 0);
}

static int txc_irq_init(struct txc_data *txc)
{
	int ret = 0;
	int irq;

	mt_set_gpio_dir(GPIO_IR_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_IR_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_IR_EINT_PIN, GPIO_PULL_UP);
	mt_set_gpio_mode(GPIO_IR_EINT_PIN, GPIO_MODE_00);
	mt_eint_set_sens(CUST_EINT_INTI_INT_NUM, MT_EDGE_SENSITIVE);
	mt_eint_registration(CUST_EINT_INTI_INT_NUM, EINTF_TRIGGER_FALLING, txc_irq_handler, 0);
	mt_eint_unmask(CUST_EINT_INTI_INT_NUM);

	return ret;
}

static int txc_create_input(struct txc_data *txc)
{
	int ret;
	struct input_dev *dev;

	dev = input_allocate_device();
	if (!dev) {
		INFOR("%s()->%d:can not alloc memory to txc input device!\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	set_bit(EV_ABS, dev->evbit);
	input_set_capability(dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(dev, ABS_DISTANCE, 0, 1, 0, 0);  /*the max value 1bit*/
	dev->name = "pa122";
	dev->dev.parent = &txc->client->dev;

	ret = input_register_device(dev);
	if (ret < 0) {
		INFOR("%s()->%d:can not register txc input device!\n",
			__func__, __LINE__);
		input_free_device(dev);
		return ret;
	}

	txc->input_dev = dev;
	input_set_drvdata(txc->input_dev, txc);

	return 0;
}

static int ps_notifier_handler(struct notifier_block *this, unsigned long event, void *ptr)
{
	struct txc_data *data = container_of(this, struct txc_data, ps_pm_notifier);
	
	if (!data->ps_enable) {
		if (event == PM_SUSPEND_PREPARE)
			mt_eint_mask(CUST_EINT_INTI_INT_NUM);
		else if (event == PM_POST_SUSPEND)
			mt_eint_unmask(CUST_EINT_INTI_INT_NUM);
	}
	INFOR("ps enable(%d), state(%s)\n", data->ps_enable, event==PM_SUSPEND_PREPARE?"suspend":"resume");
	return event;
}

static int txc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct txc_data *txc;

	/*request private data*/
	txc = kzalloc(sizeof(struct txc_data), GFP_KERNEL);
	if (!txc) {
		INFOR("%s()->%d:can not alloc memory to private data !\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	/*set client private data*/
	txc->client = client;
	txc->ps_enable = 0;
	i2c_set_clientdata(client, txc);
	txc_info = txc;
	txc_info->irq_gpio = 1;
	txc->ps_dev = &client->dev;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		INFOR("%s()->%d:i2c adapter don't support i2c operation!\n", __func__, __LINE__);
		return -ENODEV;
	}
	mutex_init(&txc->enable_lock);
	mutex_init(&txc->i2c_lock);
	wake_lock_init(&txc->ps_wake_lock, WAKE_LOCK_SUSPEND, "ps_wake_lock");

	/*create input device for reporting data*/
	ret = txc_create_input(txc);
	if (ret < 0) {
		INFOR("%s()->%d:can not create input device!\n", __func__, __LINE__);
		return ret;
	}
	
	ret = pa12201001_init(txc);
	if (ret < 0) {
		INFOR("%s()->%d:pa12201001_init failed!\n", __func__, __LINE__);
		return ret;
	}

	ret = sysfs_create_group(&client->dev.kobj, &txc_attribute_group);
	if (ret < 0) {
		INFOR("%s()->%d:can not create sysfs group attributes!\n", __func__, __LINE__);
		return ret;
	}

	meizu_sysfslink_register(txc->ps_dev);

	/*register ps pm notifier */
	txc->ps_pm_notifier.notifier_call = ps_notifier_handler;
	#if CONFIG_MZ_MA01
	#else
		register_pm_notifier(&txc->ps_pm_notifier);
	#endif
	INIT_DELAYED_WORK(&txc->ps_dwork, txc_ps_handler);
	txc_irq_init(txc);

	txc->queue = create_singlethread_workqueue("pa1212wq");
	txc_info = txc;

	INFOR("%s: probe ok!!, client addr is 0x%02x, txc->ps_data:%d\n", __func__, client->addr, txc->ps_data);
	return 0;
}

static const struct i2c_device_id txc_id[] = {
	{ "ps", 0 },
	{},
};

static int txc_remove(struct i2c_client *client)
{
	struct txc_data *txc = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &txc_attribute_group);
	input_unregister_device(txc->input_dev);
	input_free_device(txc->input_dev);
	meizu_sysfslink_unregister(txc->ps_dev);
	unregister_pm_notifier(&txc->ps_pm_notifier);
	kfree(txc);

	return 0;
}

#if CONFIG_MZ_MA01
static int ps_suspend(struct device *dev)
{
	int ps_gpio = mt_get_gpio_in(GPIO_IR_EINT_PIN);	
	enable_irq_wake(CUST_EINT_INTI_INT_NUM);
	if (!txc_info->wakeup_enable) {
		mt_eint_mask(CUST_EINT_INTI_INT_NUM);
//		mt_set_gpio_mode(GPIO_IR_EINT_PIN, GPIO_MODE_02);
		txc_info->irq_wake_enabled = 1;
	}
	ps_gpio = mt_get_gpio_in(GPIO_IR_EINT_PIN);
	INFOR("ps_gpio(%d), ps enable(%d), wakeen(%d)\n", ps_gpio, txc_info->ps_enable, txc_info->wakeup_enable);
	return 0;
}

static int ps_resume(struct device *dev)
{
	int ps_gpio = mt_get_gpio_in(GPIO_IR_EINT_PIN);	

	disable_irq_wake(CUST_EINT_INTI_INT_NUM);
	if (txc_info->irq_wake_enabled) {
		mt_set_gpio_mode(GPIO_IR_EINT_PIN, GPIO_MODE_00);
		mt_eint_unmask(CUST_EINT_INTI_INT_NUM);
		txc_info->irq_wake_enabled = 0;
	}
	ps_gpio = mt_get_gpio_in(GPIO_IR_EINT_PIN);
	INFOR("ps_gpio(%d), ps enable(%d), wakeen(%d)\n", ps_gpio, txc_info->ps_enable, txc_info->irq_wake_enabled);
	return 0;
}
#else
static int ps_suspend(struct device *dev)
{
	int ps_gpio = mt_get_gpio_in(GPIO_IR_EINT_PIN);	
	if (!txc_info->wakeup_enable) {
		mt_eint_mask(CUST_EINT_INTI_INT_NUM);
		mt_set_gpio_mode(GPIO_IR_EINT_PIN, GPIO_MODE_02);
		txc_info->irq_wake_enabled = 1;
	}
	INFOR("ps_gpio(%d), ps enable(%d), wakeen(%d)\n", ps_gpio, txc_info->ps_enable, txc_info->wakeup_enable);
	return 0;
}

static int ps_resume(struct device *dev)
{
	int ps_gpio = mt_get_gpio_in(GPIO_IR_EINT_PIN);	

	if (txc_info->irq_wake_enabled) {
		mt_set_gpio_mode(GPIO_IR_EINT_PIN, GPIO_MODE_00);
		mt_eint_unmask(CUST_EINT_INTI_INT_NUM);
		txc_info->irq_wake_enabled = 0;
	}
	INFOR("ps_gpio(%d), ps enable(%d), wakeen(%d)\n", ps_gpio, txc_info->ps_enable, txc_info->irq_wake_enabled);
	return 0;
}
#endif

static const struct dev_pm_ops ps_pm_ops = {
	.suspend = ps_suspend,
	.resume	= ps_resume,
};

static struct i2c_driver txc_driver = {
	.driver = {
		.name	= TXC_DEV_NAME,
		.owner	= THIS_MODULE,
		.pm    = &ps_pm_ops,   
	},
	.probe	= txc_probe,
	.remove = txc_remove,
	.id_table = txc_id,
};

static int __init txc_init(void)
{
	i2c_register_board_info(3, i2c_txc, 1);
	return i2c_add_driver(&txc_driver);
}

static void __exit txc_exit(void)
{
	i2c_del_driver(&txc_driver);
}

module_init(txc_init);
module_exit(txc_exit);

