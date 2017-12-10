/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 S5K3L8ofilmmipiraw_sensor.c
 *
 * Project:
 * --------
 *	 ALPS MT6735
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *	PengtaoFan
 *  20150624: the first driver from ov8858
 *  20150706: add pip 15fps setting
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k3l8ofilmmipiraw_Sensor.h"

#define FANPENGTAO
#define PFX "S5K3L8_ofilm_camera_sensor"
#define LOG_1 LOG_INF("S5K3L8,MIPI 4LANE\n")
//#define LOG_WRN(format, args...) xlog_printk(ANDROID_LOG_WARN ,PFX, "[%S] " format, __FUNCTION__, ##args)
//#defineLOG_INF(format, args...) xlog_printk(ANDROID_LOG_INFO ,PFX, "[%s] " format, __FUNCTION__, ##args)
//#define LOG_DBG(format, args...) xlog_printk(ANDROID_LOG_DEBUG ,PFX, "[%S] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)	xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

u8 *s5k3l8_ofilm_otp_buf;
#define	MAX_READ_WRITE_SIZE	8
#define	OTP_DATA_SIZE	3400
#define	OTP_START_ADDR	0x0000
#define	E2PROM_WRITE_ID	0xA0
#define	PDAF_DATA_SIZE	1404
#define	PDAF_DATA_OFFSET	0x79B

static kal_uint8 mode_change = 0;

static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = S5K3L8_OFILM_SENSOR_ID & 0x00FFFF,		//Sensor ID Value: 0x30C8//record sensor id defined in Kd_imgsensor.h
	
	.checksum_value = 0xd14be45,		//checksum value for Camera Auto Test
	
	.pre = {
		.pclk = 566000000,				//record different mode's pclk
		.linelength  = 5808,				//record different mode's linelength
		.framelength = 3234,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 2104,		//record different mode's width of grabwindow
		.grabwindow_height = 1560,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 566000000,				//record different mode's pclk
		.linelength  = 5808,				//record different mode's linelength
		.framelength = 3234,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 4208,		//record different mode's width of grabwindow
		.grabwindow_height = 3120,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
#if 0 //fps =24
	.cap1 = {							//capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 448000000,				//record different mode's pclk
		.linelength  = 5808,				//record different mode's linelength
		.framelength = 3206,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 4208,		//record different mode's width of grabwindow
		.grabwindow_height = 3120,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 240,	
	},
#endif
	.cap1 = {							//capture for PIP 15ps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
		.pclk = 400000000,				//record different mode's pclk
		.linelength  = 5808,				//record different mode's linelength
		.framelength = 4589,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 4208,		//record different mode's width of grabwindow
		.grabwindow_height = 3120,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 150,	
	},
	.normal_video = {
		.pclk = 566000000,				//record different mode's pclk
		.linelength  = 5808,				//record different mode's linelength
		.framelength = 3234,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 2104,		//record different mode's width of grabwindow
		.grabwindow_height = 1560,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.hs_video = {
		.pclk = 566000000,				//record different mode's pclk
		.linelength  = 5808,				//record different mode's linelength
		.framelength = 812,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1052,		//record different mode's width of grabwindow
		.grabwindow_height = 780,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 1200,	
	},
	.slim_video = {
		.pclk = 566000000,				//record different mode's pclk
		.linelength  = 5808,				//record different mode's linelength
		.framelength = 3234,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 1052,		//record different mode's width of grabwindow
		.grabwindow_height = 780,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,
	},
	/* 4K video */
	.custom2 = {
		.pclk = 566000000,				//record different mode's pclk
		.linelength  = 5808,				//record different mode's linelength
		.framelength = 3234,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width  = 3840,		//record different mode's width of grabwindow
		.grabwindow_height = 2176,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},

	.margin = 16,			//sensor framelength & shutter margin
	.min_shutter = 10,		//min shutter
	.max_frame_length = 0xFFFF,//REG0x0202 <=REG0x0340-5//max framelength by sensor register's limitation
	.ae_shut_delay_frame = 0,	//shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
	.ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	.ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 6,	  //support sensor mode num ,don't support Slow motion
	
	.cap_delay_frame = 0,		//enter capture delay frame num
	.pre_delay_frame = 0, 		//enter preview delay frame num
	.video_delay_frame = 0,		//enter video delay frame num
	.hs_video_delay_frame = 0,	//enter high speed video  delay frame num
	.slim_video_delay_frame = 0,//enter slim video delay frame num
	.custom2_delay_frame = 0, 
	
	.isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANUAL,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,//sensor output first pixel color
	.mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
	.mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
	.i2c_addr_table = {0x5a, 0x20, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.orientation = 90,                //mirrorflip information, 0, 90, 180, 270 degree.
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x200,					//current shutter
	.gain = 0x200,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = KAL_FALSE, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x20,//record current sensor's i2c write id
};


/* Sensor output window information*/
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[6] =
{
 { 4224, 3136,	  8,  	8, 4208, 3120, 2104, 1560,   0,	0, 2104, 1560, 	 0, 0, 2104, 1560}, // Preview 
 { 4224, 3136,	  8,  	8, 4208, 3120, 4208, 3120,   0,	0, 4208, 3120, 	 0, 0, 4208, 3120}, // capture 
 { 4224, 3136,	  8,  	8, 4208, 3120, 2104, 1560,   0,	0, 2104, 1560, 	 0, 0, 2104, 1560}, // video 
 { 4224, 3136,	  8,    8, 4208, 3120, 1052, 780,   0,	0, 1052,  780, 	 0, 0, 1052,  780}, //hight speed video
 { 4224, 3136,	  8,    8, 4208, 3120, 1052, 780,   0,	0, 1052,  780, 	 0, 0, 1052,  780}, //slim video
 { 4224, 3136,	192,  488, 3840, 2176, 3840, 2176,   0,	0, 3840, 2176, 	 0, 0, 3840, 2176}, //4K video
};

static SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
    .i4OffsetX =  28,
    .i4OffsetY = 31,
    .i4PitchX  = 64,
    .i4PitchY  = 64,
    .i4PairNum  =16,
    .i4SubBlkW  =16,
    .i4SubBlkH  =16,//need check xb.pang
    .i4PosL = {{28,31},{80,31},{44,35},{64,35},{32,51},{76,51},{48,55},{60,55},{48,63},{60,63},{32,67},{76,67},{44,83},{64,83},{28,87},{80,87}},
    .i4PosR = {{28,35},{80,35},{44,39},{64,39},{32,47},{76,47},{48,51},{60,51},{48,67},{60,67},{32,71},{76,71},{44,79},{64,79},{28,83},{80,83}},
};

static kal_uint16 read_cmos_sensor_byte(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };

    iReadRegI2C(pu_send_cmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
    return get_byte;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };

    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);
    return get_byte;
}

static void write_cmos_sensor_byte(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};

    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};

    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}

static int s5k3l8_read_otp(u16 addr, u8 *buf)
{
	int ret = 0;
	u8 pu_send_cmd[2] = {(u8)(addr >> 8), (u8)(addr & 0xFF)};

	ret = iReadRegI2C(pu_send_cmd, 2, (u8*)buf, 1, E2PROM_WRITE_ID);
	if (ret < 0)
		LOG_INF("read data from s5k3l8 ofilm otp e2prom failed!\n");

	return ret;
}

static void s5k3l8_read_otp_burst(u16 addr, u8 *otp_buf)
{
	int i;
	int ret;
	u8 pu_send_cmd[2];

	for (i = 0; i < OTP_DATA_SIZE; i += MAX_READ_WRITE_SIZE) {
		pu_send_cmd[0] = (u8)(addr >> 8);
		pu_send_cmd[1] = (u8)(addr & 0xFF);

		if (i + MAX_READ_WRITE_SIZE > OTP_DATA_SIZE)
			ret = iReadRegI2C(pu_send_cmd, 2, (u8 *)(otp_buf + i), (OTP_DATA_SIZE - i), E2PROM_WRITE_ID);
		else
			ret = iReadRegI2C(pu_send_cmd, 2, (u8 *)(otp_buf + i), MAX_READ_WRITE_SIZE, E2PROM_WRITE_ID);

		if (ret < 0)
			LOG_INF("read lsc table from s5k3l8 ofilm otp e2prom failed!\n");

		addr += MAX_READ_WRITE_SIZE;
	}
}

static void read_s5k3l8_pdaf_data(kal_uint16 addr, BYTE* data, kal_uint32 size)
{
	copy_to_user((void __user *)data,(void *)(s5k3l8_ofilm_otp_buf + PDAF_DATA_OFFSET), PDAF_DATA_SIZE);
}

static void set_dummy()
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);	  
	write_cmos_sensor(0x0342, imgsensor.line_length & 0xFFFF);
}	/*	set_dummy  */


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable = %d. \n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */


static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;
	   
	LOG_INF("Enter shutter!\n");
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	// Update Shutter
	write_cmos_sensor(0x0202, (shutter) & 0xFFFF);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

	//LOG_INF("frame_length = %d ", frame_length);
	
}	/*	write_shutter  */


static void long_exp_setting(kal_uint16 frame_length, kal_uint16 shutter)
{
	LOG_INF("Enter:%s, framelength =%d, shutter:%d.\n", __func__, frame_length, shutter);
	/* PLL CLK setting; Pix_clk = 50Mhz */
	write_cmos_sensor(0x0306, 0x007D);	//PLL Multi 125
	write_cmos_sensor(0x0302, 0x0008);	//vt_sys_clk_div
	write_cmos_sensor(0x030E, 0x0080);	//128 mipi pll M
	write_cmos_sensor(0x3008, 0x0002);	//mipi s-divider
	write_cmos_sensor(0x31E4, 0x0058);	//#SenAnalog_AIG_DefCclkFreqMhz 0058   default : 02BC //test tg

	write_cmos_sensor(0x0342, 0x3F86);	//LLP
	write_cmos_sensor(0x0340, frame_length & 0xFFFF);
	write_cmos_sensor(0x0202, shutter & 0xFFFF);	//CintR
	write_cmos_sensor(0x316A, 0x0000);	//OUTIF threshold
	write_cmos_sensor(0x303E, shutter & 0xFFFF);	//#smiaRegs_vendor_sensor_coarse_on_2nd_frame
	write_cmos_sensor(0x3046, 0x0020);	//#smiaRegs_vendor_sensor_short_analog_gain_on_2nd_frame 0020
	write_cmos_sensor(0x3048, frame_length & 0xFFFF);	//#smiaRegs_vendor_sensor_timing_frame_lines_1st_frame
	write_cmos_sensor(0x304A, 0x0100);	//#smiaRegs_vendor_sensor_b_use_2nd_frame_settings
}

/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
	unsigned long flags;
	u8 long_exp_flag = 0;
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;
	kal_uint32 line_length = 0;
	kal_uint32 origi_line_length;
	kal_uint32 origi_shutter = 0,usr_shutter = shutter;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	origi_line_length = imgsensor.line_length;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	LOG_INF("Enter! shutter =%d \n", shutter);

	/* Just be called in capture mode with long exp */
	if (usr_shutter == 2)
		write_cmos_sensor(0x0100, 0x0000);

	//write_shutter(shutter);
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;

	if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) {
		origi_shutter = shutter;
		shutter = origi_shutter * 50 / 16262 * imgsensor.line_length / 566;
		long_exp_flag = 1;
	}

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);

	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	// Framelength should be an even number
	shutter = (shutter >> 1) << 1;
	imgsensor.frame_length = (imgsensor.frame_length >> 1) << 1;
	
	if (long_exp_flag) {
		long_exp_setting(imgsensor.frame_length, shutter);
	} else if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
		else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
		write_cmos_sensor(0x0342, imgsensor.line_length & 0xFFFF);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
		write_cmos_sensor(0x0342, imgsensor.line_length & 0xFFFF);
	}

	// Update Shutter
	write_cmos_sensor(0x0202, shutter & 0xFFFF);
	LOG_INF("Exit! shutter =%d, framelength =%d, line_length:%d;\n", shutter,imgsensor.frame_length, imgsensor.line_length);

	imgsensor.line_length = origi_line_length;

	/* Just be called in capture mode with long exp, return long_exp mode to normal mode */
	if (usr_shutter == 2) {
		write_cmos_sensor(0x316A, 0x00A0);	// OUTIF threshold
		write_cmos_sensor(0x304A, 0x0000);	//#smiaRegs_vendor_sensor_b_use_2nd_frame_settings
		write_cmos_sensor(0x0100, 0x0100);
	}
}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
	//gain = 64 = 1x real gain.
    reg_gain = gain/2;
	//reg_gain = reg_gain & 0xFFFF;
	return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	LOG_INF("set_gain %d \n", gain);
  //gain = 64 = 1x real gain.
	kal_uint16 reg_gain;
	if (gain < BASEGAIN || gain > 16 * BASEGAIN) {
		LOG_INF("Error gain setting");
		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 16 * BASEGAIN)
			gain = 16 * BASEGAIN;		 
	}

    reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain; 
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	write_cmos_sensor(0x0204, (reg_gain&0xFFFF));    

	/*
	 * WORKAROUND! stream on after set shutter/gain, which will get
	 * first valid frame.
	 */
	if (mode_change && ((imgsensor.sensor_mode == IMGSENSOR_MODE_CAPTURE)
	    || (imgsensor.sensor_mode == IMGSENSOR_MODE_PREVIEW))) {
		write_cmos_sensor(0x0100, 0x0100);
		mode_change = 0;
	}

	return gain;
}	/*	set_gain  */

//ihdr_write_shutter_gain not support for s5k3l8
static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
	if (imgsensor.ihdr_en) {
		
		spin_lock(&imgsensor_drv_lock);
			if (le > imgsensor.min_frame_length - imgsensor_info.margin)		
				imgsensor.frame_length = le + imgsensor_info.margin;
			else
				imgsensor.frame_length = imgsensor.min_frame_length;
			if (imgsensor.frame_length > imgsensor_info.max_frame_length)
				imgsensor.frame_length = imgsensor_info.max_frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
			if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;
			
			
		// Extend frame length first
		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);

		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);	 
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		
		write_cmos_sensor(0x3512, (se << 4) & 0xFF); 
		write_cmos_sensor(0x3511, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3510, (se >> 12) & 0x0F); 

		set_gain(gain);
	}

}



static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	spin_lock(&imgsensor_drv_lock);
    imgsensor.mirror= image_mirror; 
    spin_unlock(&imgsensor_drv_lock);
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor_byte(0x0101,0X00); //GR
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor_byte(0x0101,0X01); //R
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor_byte(0x0101,0X02); //B	
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor_byte(0x0101,0X03); //GB
			break;
		default: 
			LOG_INF("Error image_mirror setting\n");
	}

}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/
static void sensor_init(void)
{
  LOG_INF("%s.\n", __func__);
    write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0xFFFF);
	write_cmos_sensor(0x6216, 0xFFFF);
	write_cmos_sensor(0x6218, 0x0000);
	write_cmos_sensor(0x621A, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x2450);
	write_cmos_sensor(0x6F12, 0x0448);
	write_cmos_sensor(0x6F12, 0x0349);
	write_cmos_sensor(0x6F12, 0x0160);
	write_cmos_sensor(0x6F12, 0xC26A);
	write_cmos_sensor(0x6F12, 0x511A);
	write_cmos_sensor(0x6F12, 0x8180);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x48B8);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2588);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x16C0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x10B5);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x5DF8);
	write_cmos_sensor(0x6F12, 0x2748);
	write_cmos_sensor(0x6F12, 0x4078);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x6F12, 0x0AD0);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x5CF8);
	write_cmos_sensor(0x6F12, 0x2549);
	write_cmos_sensor(0x6F12, 0xB1F8);
	write_cmos_sensor(0x6F12, 0x1403);
	write_cmos_sensor(0x6F12, 0x4200);
	write_cmos_sensor(0x6F12, 0x2448);
	write_cmos_sensor(0x6F12, 0x4282);
	write_cmos_sensor(0x6F12, 0x91F8);
	write_cmos_sensor(0x6F12, 0x9610);
	write_cmos_sensor(0x6F12, 0x4187);
	write_cmos_sensor(0x6F12, 0x10BD);
	write_cmos_sensor(0x6F12, 0x70B5);
	write_cmos_sensor(0x6F12, 0x0446);
	write_cmos_sensor(0x6F12, 0x2148);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0x4068);
	write_cmos_sensor(0x6F12, 0x86B2);
	write_cmos_sensor(0x6F12, 0x050C);
	write_cmos_sensor(0x6F12, 0x3146);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x4CF8);
	write_cmos_sensor(0x6F12, 0x2046);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x4EF8);
	write_cmos_sensor(0x6F12, 0x14F8);
	write_cmos_sensor(0x6F12, 0x680F);
	write_cmos_sensor(0x6F12, 0x6178);
	write_cmos_sensor(0x6F12, 0x40EA);
	write_cmos_sensor(0x6F12, 0x4100);
	write_cmos_sensor(0x6F12, 0x1749);
	write_cmos_sensor(0x6F12, 0xC886);
	write_cmos_sensor(0x6F12, 0x1848);
	write_cmos_sensor(0x6F12, 0x2278);
	write_cmos_sensor(0x6F12, 0x007C);
	write_cmos_sensor(0x6F12, 0x4240);
	write_cmos_sensor(0x6F12, 0x1348);
	write_cmos_sensor(0x6F12, 0xA230);
	write_cmos_sensor(0x6F12, 0x8378);
	write_cmos_sensor(0x6F12, 0x43EA);
	write_cmos_sensor(0x6F12, 0xC202);
	write_cmos_sensor(0x6F12, 0x0378);
	write_cmos_sensor(0x6F12, 0x4078);
	write_cmos_sensor(0x6F12, 0x9B00);
	write_cmos_sensor(0x6F12, 0x43EA);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0x0243);
	write_cmos_sensor(0x6F12, 0xD0B2);
	write_cmos_sensor(0x6F12, 0x0882);
	write_cmos_sensor(0x6F12, 0x3146);
	write_cmos_sensor(0x6F12, 0x2846);
	write_cmos_sensor(0x6F12, 0xBDE8);
	write_cmos_sensor(0x6F12, 0x7040);
	write_cmos_sensor(0x6F12, 0x0122);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x2AB8);
	write_cmos_sensor(0x6F12, 0x10B5);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x8701);
	write_cmos_sensor(0x6F12, 0x0B48);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x2DF8);
	write_cmos_sensor(0x6F12, 0x084C);
	write_cmos_sensor(0x6F12, 0x0022);
	write_cmos_sensor(0x6F12, 0xAFF2);
	write_cmos_sensor(0x6F12, 0x6D01);
	write_cmos_sensor(0x6F12, 0x2060);
	write_cmos_sensor(0x6F12, 0x0848);
	write_cmos_sensor(0x6F12, 0x00F0);
	write_cmos_sensor(0x6F12, 0x25F8);
	write_cmos_sensor(0x6F12, 0x6060);
	write_cmos_sensor(0x6F12, 0x10BD);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x0550);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x0C60);
	write_cmos_sensor(0x6F12, 0x4000);
	write_cmos_sensor(0x6F12, 0xD000);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x2580);
	write_cmos_sensor(0x6F12, 0x2000);
	write_cmos_sensor(0x6F12, 0x16F0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x2221);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x2249);
	write_cmos_sensor(0x6F12, 0x42F2);
	write_cmos_sensor(0x6F12, 0x351C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x42F2);
	write_cmos_sensor(0x6F12, 0xE11C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x40F2);
	write_cmos_sensor(0x6F12, 0x077C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x42F2);
	write_cmos_sensor(0x6F12, 0x492C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x4BF2);
	write_cmos_sensor(0x6F12, 0x453C);
	write_cmos_sensor(0x6F12, 0xC0F2);
	write_cmos_sensor(0x6F12, 0x000C);
	write_cmos_sensor(0x6F12, 0x6047);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x30C8);
	write_cmos_sensor(0x6F12, 0x0157);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0003);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1082);
	write_cmos_sensor(0x6F12, 0x8010);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x31CE, 0x0001);
	write_cmos_sensor(0x0200, 0x00C6);
	write_cmos_sensor(0x3734, 0x0010);
	write_cmos_sensor(0x3736, 0x0001);
	write_cmos_sensor(0x3738, 0x0001);
	write_cmos_sensor(0x37CC, 0x0001);
	write_cmos_sensor(0x3744, 0x0100);
	write_cmos_sensor(0x3762, 0x0105);
	write_cmos_sensor(0x3764, 0x0105);
	write_cmos_sensor(0x376A, 0x00F0);
	write_cmos_sensor(0x344A, 0x000F);
	write_cmos_sensor(0x344C, 0x003D);
	write_cmos_sensor(0xF460, 0x0030);
	write_cmos_sensor(0xF414, 0x24C2);
	write_cmos_sensor(0xF416, 0x0183);
	write_cmos_sensor(0xF468, 0x0405);
	write_cmos_sensor(0x3424, 0x0A07);
	write_cmos_sensor(0x3426, 0x0F07);
	write_cmos_sensor(0x3428, 0x0F07);
	write_cmos_sensor(0x341E, 0x0804);
	write_cmos_sensor(0x3420, 0x0C0C);
	write_cmos_sensor(0x3422, 0x2D2D);
	write_cmos_sensor(0xF462, 0x003A);
	write_cmos_sensor(0x3450, 0x0010);
	write_cmos_sensor(0x3452, 0x0010);
	write_cmos_sensor(0xF446, 0x0020);
	write_cmos_sensor(0xF44E, 0x000C);
	write_cmos_sensor(0x31FA, 0x0007);
	write_cmos_sensor(0x31FC, 0x0161);
	write_cmos_sensor(0x31FE, 0x0009);
	write_cmos_sensor(0x3200, 0x000C);
	write_cmos_sensor(0x3202, 0x007F);
	write_cmos_sensor(0x3204, 0x00A2);
	write_cmos_sensor(0x3206, 0x007D);
	write_cmos_sensor(0x3208, 0x00A4);
	write_cmos_sensor(0x3334, 0x00A7);
	write_cmos_sensor(0x3336, 0x00A5);
	write_cmos_sensor(0x3338, 0x0033);
	write_cmos_sensor(0x333A, 0x0006);
	write_cmos_sensor(0x333C, 0x009F);
	write_cmos_sensor(0x333E, 0x008C);
	write_cmos_sensor(0x3340, 0x002D);
	write_cmos_sensor(0x3342, 0x000A);
	write_cmos_sensor(0x3344, 0x002F);
	write_cmos_sensor(0x3346, 0x0008);
	write_cmos_sensor(0x3348, 0x009F);
	write_cmos_sensor(0x334A, 0x008C);
	write_cmos_sensor(0x334C, 0x002D);
	write_cmos_sensor(0x334E, 0x000A);
	write_cmos_sensor(0x3350, 0x000A);
	write_cmos_sensor(0x320A, 0x007B);
	write_cmos_sensor(0x320C, 0x0161);
	write_cmos_sensor(0x320E, 0x007F);
	write_cmos_sensor(0x3210, 0x015F);
	write_cmos_sensor(0x3212, 0x007B);
	write_cmos_sensor(0x3214, 0x00B0);
	write_cmos_sensor(0x3216, 0x0009);
	write_cmos_sensor(0x3218, 0x0038);
	write_cmos_sensor(0x321A, 0x0009);
	write_cmos_sensor(0x321C, 0x0031);
	write_cmos_sensor(0x321E, 0x0009);
	write_cmos_sensor(0x3220, 0x0038);
	write_cmos_sensor(0x3222, 0x0009);
	write_cmos_sensor(0x3224, 0x007B);
	write_cmos_sensor(0x3226, 0x0001);
	write_cmos_sensor(0x3228, 0x0010);
	write_cmos_sensor(0x322A, 0x00A2);
	write_cmos_sensor(0x322C, 0x00B1);
	write_cmos_sensor(0x322E, 0x0002);
	write_cmos_sensor(0x3230, 0x015D);
	write_cmos_sensor(0x3232, 0x0001);
	write_cmos_sensor(0x3234, 0x015D);
	write_cmos_sensor(0x3236, 0x0001);
	write_cmos_sensor(0x3238, 0x000B);
	write_cmos_sensor(0x323A, 0x0016);
	write_cmos_sensor(0x323C, 0x000D);
	write_cmos_sensor(0x323E, 0x001C);
	write_cmos_sensor(0x3240, 0x000D);
	write_cmos_sensor(0x3242, 0x0054);
	write_cmos_sensor(0x3244, 0x007B);
	write_cmos_sensor(0x3246, 0x00CC);
	write_cmos_sensor(0x3248, 0x015D);
	write_cmos_sensor(0x324A, 0x007E);
	write_cmos_sensor(0x324C, 0x0095);
	write_cmos_sensor(0x324E, 0x0085);
	write_cmos_sensor(0x3250, 0x009D);
	write_cmos_sensor(0x3252, 0x008D);
	write_cmos_sensor(0x3254, 0x009D);
	write_cmos_sensor(0x3256, 0x007E);
	write_cmos_sensor(0x3258, 0x0080);
	write_cmos_sensor(0x325A, 0x0001);
	write_cmos_sensor(0x325C, 0x0005);
	write_cmos_sensor(0x325E, 0x0085);
	write_cmos_sensor(0x3260, 0x009D);
	write_cmos_sensor(0x3262, 0x0001);
	write_cmos_sensor(0x3264, 0x0005);
	write_cmos_sensor(0x3266, 0x007E);
	write_cmos_sensor(0x3268, 0x0080);
	write_cmos_sensor(0x326A, 0x0053);
	write_cmos_sensor(0x326C, 0x007D);
	write_cmos_sensor(0x326E, 0x00CB);
	write_cmos_sensor(0x3270, 0x015E);
	write_cmos_sensor(0x3272, 0x0001);
	write_cmos_sensor(0x3274, 0x0005);
	write_cmos_sensor(0x3276, 0x0009);
	write_cmos_sensor(0x3278, 0x000C);
	write_cmos_sensor(0x327A, 0x007E);
	write_cmos_sensor(0x327C, 0x0098);
	write_cmos_sensor(0x327E, 0x0009);
	write_cmos_sensor(0x3280, 0x000C);
	write_cmos_sensor(0x3282, 0x007E);
	write_cmos_sensor(0x3284, 0x0080);
	write_cmos_sensor(0x3286, 0x0044);
	write_cmos_sensor(0x3288, 0x0163);
	write_cmos_sensor(0x328A, 0x0045);
	write_cmos_sensor(0x328C, 0x0047);
	write_cmos_sensor(0x328E, 0x007D);
	write_cmos_sensor(0x3290, 0x0080);
	write_cmos_sensor(0x3292, 0x015F);
	write_cmos_sensor(0x3294, 0x0162);
	write_cmos_sensor(0x3296, 0x007D);
	write_cmos_sensor(0x3298, 0x0000);
	write_cmos_sensor(0x329A, 0x0000);
	write_cmos_sensor(0x329C, 0x0000);
	write_cmos_sensor(0x329E, 0x0000);
	write_cmos_sensor(0x32A0, 0x0008);
	write_cmos_sensor(0x32A2, 0x0010);
	write_cmos_sensor(0x32A4, 0x0018);
	write_cmos_sensor(0x32A6, 0x0020);
	write_cmos_sensor(0x32A8, 0x0000);
	write_cmos_sensor(0x32AA, 0x0008);
	write_cmos_sensor(0x32AC, 0x0010);
	write_cmos_sensor(0x32AE, 0x0018);
	write_cmos_sensor(0x32B0, 0x0020);
	write_cmos_sensor(0x32B2, 0x0020);
	write_cmos_sensor(0x32B4, 0x0020);
	write_cmos_sensor(0x32B6, 0x0020);
	write_cmos_sensor(0x32B8, 0x0000);
	write_cmos_sensor(0x32BA, 0x0000);
	write_cmos_sensor(0x32BC, 0x0000);
	write_cmos_sensor(0x32BE, 0x0000);
	write_cmos_sensor(0x32C0, 0x0000);
	write_cmos_sensor(0x32C2, 0x0000);
	write_cmos_sensor(0x32C4, 0x0000);
	write_cmos_sensor(0x32C6, 0x0000);
	write_cmos_sensor(0x32C8, 0x0000);
	write_cmos_sensor(0x32CA, 0x0000);
	write_cmos_sensor(0x32CC, 0x0000);
	write_cmos_sensor(0x32CE, 0x0000);
	write_cmos_sensor(0x32D0, 0x0000);
	write_cmos_sensor(0x32D2, 0x0000);
	write_cmos_sensor(0x32D4, 0x0000);
	write_cmos_sensor(0x32D6, 0x0000);
	write_cmos_sensor(0x32D8, 0x0000);
	write_cmos_sensor(0x32DA, 0x0000);
	write_cmos_sensor(0x32DC, 0x0000);
	write_cmos_sensor(0x32DE, 0x0000);
	write_cmos_sensor(0x32E0, 0x0000);
	write_cmos_sensor(0x32E2, 0x0000);
	write_cmos_sensor(0x32E4, 0x0000);
	write_cmos_sensor(0x32E6, 0x0000);
	write_cmos_sensor(0x32E8, 0x0000);
	write_cmos_sensor(0x32EA, 0x0000);
	write_cmos_sensor(0x32EC, 0x0000);
	write_cmos_sensor(0x32EE, 0x0000);
	write_cmos_sensor(0x32F0, 0x0000);
	write_cmos_sensor(0x32F2, 0x0000);
	write_cmos_sensor(0x32F4, 0x000A);
	write_cmos_sensor(0x32F6, 0x0002);
	write_cmos_sensor(0x32F8, 0x0008);
	write_cmos_sensor(0x32FA, 0x0010);
	write_cmos_sensor(0x32FC, 0x0020);
	write_cmos_sensor(0x32FE, 0x0028);
	write_cmos_sensor(0x3300, 0x0038);
	write_cmos_sensor(0x3302, 0x0040);
	write_cmos_sensor(0x3304, 0x0050);
	write_cmos_sensor(0x3306, 0x0058);
	write_cmos_sensor(0x3308, 0x0068);
	write_cmos_sensor(0x330A, 0x0070);
	write_cmos_sensor(0x330C, 0x0080);
	write_cmos_sensor(0x330E, 0x0088);
	write_cmos_sensor(0x3310, 0x0098);
	write_cmos_sensor(0x3312, 0x00A0);
	write_cmos_sensor(0x3314, 0x00B0);
	write_cmos_sensor(0x3316, 0x00B8);
	write_cmos_sensor(0x3318, 0x00C8);
	write_cmos_sensor(0x331A, 0x00D0);
	write_cmos_sensor(0x331C, 0x00E0);
	write_cmos_sensor(0x331E, 0x00E8);
	write_cmos_sensor(0x3320, 0x0017);
	write_cmos_sensor(0x3322, 0x002F);
	write_cmos_sensor(0x3324, 0x0047);
	write_cmos_sensor(0x3326, 0x005F);
	write_cmos_sensor(0x3328, 0x0077);
	write_cmos_sensor(0x332A, 0x008F);
	write_cmos_sensor(0x332C, 0x00A7);
	write_cmos_sensor(0x332E, 0x00BF);
	write_cmos_sensor(0x3330, 0x00D7);
	write_cmos_sensor(0x3332, 0x00EF);
	write_cmos_sensor(0x3352, 0x00A5);
	write_cmos_sensor(0x3354, 0x00AF);
	write_cmos_sensor(0x3356, 0x0187);
	write_cmos_sensor(0x3358, 0x0000);
	write_cmos_sensor(0x335A, 0x009E);
	write_cmos_sensor(0x335C, 0x016B);
	write_cmos_sensor(0x335E, 0x0015);
	write_cmos_sensor(0x3360, 0x00A5);
	write_cmos_sensor(0x3362, 0x00AF);
	write_cmos_sensor(0x3364, 0x01FB);
	write_cmos_sensor(0x3366, 0x0000);
	write_cmos_sensor(0x3368, 0x009E);
	write_cmos_sensor(0x336A, 0x016B);
	write_cmos_sensor(0x336C, 0x0015);
	write_cmos_sensor(0x336E, 0x00A5);
	write_cmos_sensor(0x3370, 0x00A6);
	write_cmos_sensor(0x3372, 0x0187);
	write_cmos_sensor(0x3374, 0x0000);
	write_cmos_sensor(0x3376, 0x009E);
	write_cmos_sensor(0x3378, 0x016B);
	write_cmos_sensor(0x337A, 0x0015);
	write_cmos_sensor(0x337C, 0x00A5);
	write_cmos_sensor(0x337E, 0x00A6);
	write_cmos_sensor(0x3380, 0x01FB);
	write_cmos_sensor(0x3382, 0x0000);
	write_cmos_sensor(0x3384, 0x009E);
	write_cmos_sensor(0x3386, 0x016B);
	write_cmos_sensor(0x3388, 0x0015);
	write_cmos_sensor(0x319A, 0x0005);
	write_cmos_sensor(0x1006, 0x0005);
	write_cmos_sensor(0x3416, 0x0001);
	write_cmos_sensor(0x308C, 0x0008);
	write_cmos_sensor(0x307C, 0x0240);
	write_cmos_sensor(0x375E, 0x0050);
	write_cmos_sensor(0x31CE, 0x0101);
	write_cmos_sensor(0x374E, 0x0007);
	write_cmos_sensor(0x3460, 0x0001);
	write_cmos_sensor(0x3052, 0x0002);
	write_cmos_sensor(0x3058, 0x0100);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x108A);
	write_cmos_sensor(0x6F12, 0x0359);
	write_cmos_sensor(0x6F12, 0x0100);  
	write_cmos_sensor(0x6028, 0x4000);///add
	write_cmos_sensor(0x602A, 0x1124);
	write_cmos_sensor(0x6F12, 0x4100);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x112C);
	write_cmos_sensor(0x6F12, 0x4100);
	write_cmos_sensor(0x6F12, 0x0000);	

////////////////add sensor BPC //////////////////////////////

	write_cmos_sensor(0x6028, 0x4000);   //20151116   
	write_cmos_sensor(0x0B08, 0x0100);

	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x11EE);
	write_cmos_sensor(0x6F12, 0x0000);	// 00-off, 01-on 	//#gisp_dspcl_afit_ManualMode,
	write_cmos_sensor(0x6F12, 0x0000);	// Manual Noise Index,

	write_cmos_sensor(0x602A, 0x11D6);
	write_cmos_sensor(0x6F12, 0x1000);	// gisp_dspcl_afit_VecsInds_0_ AGx1		// 200011D6
	write_cmos_sensor(0x6F12, 0x8000);	// gisp_dspcl_afit_VecsInds_1_ AGx2   // 200011D8
	write_cmos_sensor(0x6F12, 0xE000);	// gisp_dspcl_afit_VecsInds_2_ AGx14   // 200011DA
	write_cmos_sensor(0x6F12, 0xFFFF);	// gisp_dspcl_afit_VecsInds_3_ AGx16  // 200011DC

	write_cmos_sensor(0x602A, 0x1096);
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__0_				 //0x20001096	// No-Binning GAINX1
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__1_				 //0x20001098	
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__2_				 //0x2000109A	// COLD_low_Gr  
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__3_				 //0x2000109C // COLD_delta_Gr
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__4_				 //0x2000109E // COLD_low_R   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__5_				 //0x200010A0 // COLD_delta_R 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__6_				 //0x200010A2 // COLD_low_B   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__7_				 //0x200010A4	// COLD_delta_B
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__8_				 //0x200010A6	// COLD_low_Gb  
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__9_				 //0x200010A8 // COLD_delta_Gb
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__10_				 //0x200010AA
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__11_				 //0x200010AC
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__12_				 //0x200010AE	// HOT_low_Gr  	
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__13_				 //0x200010B0 // HOT_delta_Gr
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__14_				 //0x200010B2 // HOT_low_R   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__15_				 //0x200010B4 // HOT_delta_R 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__16_				 //0x200010B6 // HOT_low_B   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__17_				 //0x200010B8	// HOT_delta_B 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__18_				 //0x200010BA	// HOT_low_Gb  
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__19_				 //0x200010BC // HOT_delta_Gb
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__20_				 //0x200010BE	// Binning GAINX1
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__21_				 //0x200010C0
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__22_				 //0x200010C2	// COLD_low_Gr  
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__23_				 //0x200010C4 // COLD_delta_Gr
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__24_				 //0x200010C6 // COLD_low_R   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__25_				 //0x200010C8 // COLD_delta_R 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__26_				 //0x200010CA // COLD_low_B   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__27_				 //0x200010CC // COLD_delta_B 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__28_				 //0x200010CE // COLD_low_Gb  
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__29_				 //0x200010D0 // COLD_delta_Gb
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__30_				 //0x200010D2                 
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__31_				 //0x200010D4                 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__32_				 //0x200010D6 // HOT_low_Gr  	
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__33_				 //0x200010D8 // HOT_delta_Gr 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__34_				 //0x200010DA // HOT_low_R    
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__35_				 //0x200010DC // HOT_delta_R  
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__36_				 //0x200010DE // HOT_low_B    
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__37_				 //0x200010E0 // HOT_delta_B  
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_0__38_				 //0x200010E2 // HOT_low_Gb   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_0__39_				 //0x200010E4 // HOT_delta_Gb 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__0_				 //0x200010E6	// No-Binning GAINX8
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__1_				 //0x200010E8
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__2_				 //0x200010EA	// COLD_low_Gr  
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__3_				 //0x200010EC // COLD_delta_Gr
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__4_				 //0x200010EE // COLD_low_R   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__5_				 //0x200010F0 // COLD_delta_R 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__6_				 //0x200010F2 // COLD_low_B   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__7_				 //0x200010F4 // COLD_delta_B 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__8_				 //0x200010F6 // COLD_low_Gb  
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__9_				 //0x200010F8 // COLD_delta_Gb
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__10_				 //0x200010FA                 
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__11_				 //0x200010FC                 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__12_				 //0x200010FE // HOT_low_Gr  	
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__13_				 //0x20001100 // HOT_delta_Gr 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__14_				 //0x20001102 // HOT_low_R    
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__15_				 //0x20001104 // HOT_delta_R  
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__16_				 //0x20001106 // HOT_low_B    
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__17_				 //0x20001108 // HOT_delta_B  
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__18_				 //0x2000110A // HOT_low_Gb   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__19_				 //0x2000110C // HOT_delta_Gb 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__20_				 //0x2000110E	// Binning GAINX8
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__21_				 //0x20001110
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__22_				 //0x20001112	// COLD_low_Gr  
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__23_				 //0x20001114 // COLD_delta_Gr
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__24_				 //0x20001116 // COLD_low_R   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__25_				 //0x20001118 // COLD_delta_R 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__26_				 //0x2000111A // COLD_low_B   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__27_				 //0x2000111C // COLD_delta_B 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__28_				 //0x2000111E // COLD_low_Gb  
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__29_				 //0x20001120 // COLD_delta_Gb
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__30_				 //0x20001122                 
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__31_				 //0x20001124                 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__32_				 //0x20001126 // HOT_low_Gr  	
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__33_				 //0x20001128 // HOT_delta_Gr 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__34_				 //0x2000112A // HOT_low_R    
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__35_				 //0x2000112C // HOT_delta_R  
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__36_				 //0x2000112E // HOT_low_B    
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__37_				 //0x20001130 // HOT_delta_B  
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_1__38_				 //0x20001132 // HOT_low_Gb   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_1__39_				 //0x20001134 // HOT_delta_Gb 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__0_				 //0x20001136	// No-Binning GAINX16
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__1_				 //0x20001138
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__2_				 //0x2000113A	// COLD_low_Gr  
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__3_				 //0x2000113C // COLD_delta_Gr
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__4_				 //0x2000113E // COLD_low_R   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__5_				 //0x20001140 // COLD_delta_R 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__6_				 //0x20001142 // COLD_low_B   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__7_				 //0x20001144 // COLD_delta_B 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__8_				 //0x20001146 // COLD_low_Gb  
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__9_				 //0x20001148 // COLD_delta_Gb
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__10_				 //0x2000114A                 
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__11_				 //0x2000114C                 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__12_				 //0x2000114E // HOT_low_Gr  	
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__13_				 //0x20001150 // HOT_delta_Gr 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__14_				 //0x20001152 // HOT_low_R    
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__15_				 //0x20001154 // HOT_delta_R  
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__16_				 //0x20001156 // HOT_low_B    
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__17_				 //0x20001158 // HOT_delta_B  
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__18_				 //0x2000115A // HOT_low_Gb   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__19_				 //0x2000115C // HOT_delta_Gb 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__20_				 //0x2000115E	// Binning GAINX16
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__21_				 //0x20001160
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__22_				 //0x20001162	// COLD_low_Gr  
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__23_				 //0x20001164 // COLD_delta_Gr
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__24_				 //0x20001166 // COLD_low_R   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__25_				 //0x20001168 // COLD_delta_R 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__26_				 //0x2000116A // COLD_low_B   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__27_				 //0x2000116C // COLD_delta_B 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__28_				 //0x2000116E // COLD_low_Gb  
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__29_				 //0x20001170 // COLD_delta_Gb
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__30_				 //0x20001172                 
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__31_				 //0x20001174                 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__32_				 //0x20001176 // HOT_low_Gr  	
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__33_				 //0x20001178 // HOT_delta_Gr 
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__34_				 //0x2000117A // HOT_low_R    
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__35_				 //0x2000117C // HOT_delta_R  
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__36_				 //0x2000117E // HOT_low_B    
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__37_				 //0x20001180 // HOT_delta_B  
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_2__38_				 //0x20001182 // HOT_low_Gb   
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_2__39_				 //0x20001184 // HOT_delta_Gb                          
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__0_				 //0x20001186	// No-Binning GAINX32
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__1_				 //0x20001188
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__2_				 //0x2000118A	// COLD_low_Gr  
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__3_				 //0x2000118C // COLD_delta_Gr
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__4_				 //0x2000118E // COLD_low_R   
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__5_				 //0x20001190 // COLD_delta_R 
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__6_				 //0x20001192 // COLD_low_B   
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__7_				 //0x20001194 // COLD_delta_B 
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__8_				 //0x20001196 // COLD_low_Gb  
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__9_				 //0x20001198 // COLD_delta_Gb
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__10_				 //0x2000119A                 
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__11_				 //0x2000119C                 
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__12_				 //0x2000119E // HOT_low_Gr  	
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__13_				 //0x200011A0 // HOT_delta_Gr 
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__14_				 //0x200011A2 // HOT_low_R    
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__15_				 //0x200011A4 // HOT_delta_R  
	write_cmos_sensor(0x6F12, 0x00FF);	// #gisp_dspcl_afit_BaseVecs_3__16_				 //0x200011A6 // HOT_low_B    
	write_cmos_sensor(0x6F12, 0x0078);	// #gisp_dspcl_afit_BaseVecs_3__17_				 //0x200011A8 // HOT_delta_B  
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__18_				 //0x200011AA // HOT_low_Gb   
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__19_				 //0x200011AC // HOT_delta_Gb 
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__20_				 //0x200011AE	// Binning GAINX32
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__21_				 //0x200011B0
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__22_				 //0x200011B2	// COLD_low_Gr  
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__23_				 //0x200011B4 // COLD_delta_Gr
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__24_				 //0x200011B6 // COLD_low_R   
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__25_				 //0x200011B8 // COLD_delta_R 
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__26_				 //0x200011BA // COLD_low_B   
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__27_				 //0x200011BC // COLD_delta_B 
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__28_				 //0x200011BE // COLD_low_Gb  
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__29_				 //0x200011C0 // COLD_delta_Gb
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__30_				 //0x200011C2                 
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__31_				 //0x200011C4                 
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__32_				 //0x200011C6 // HOT_low_Gr  	
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__33_				 //0x200011C8 // HOT_delta_Gr 
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__34_				 //0x200011CA // HOT_low_R    
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__35_				 //0x200011CC // HOT_delta_R  
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__36_				 //0x200011CE // HOT_low_B    
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__37_				 //0x200011D0 // HOT_delta_B  
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__38_				 //0x200011D2 // HOT_low_Gb   
	write_cmos_sensor(0x6F12, 0x0000);	// #gisp_dspcl_afit_BaseVecs_3__39_				 //0x200011D4 // HOT_delta_Gb 
 ////////////////////////add sensor BPC  END///////////////////////////


 
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0100, 0x0000);

}	/*	sensor_init  */


static void preview_setting(void)
{
	LOG_INF("%s.\n", __func__);
	//$MV1[MCLK:24,Width:2104,Height:1560,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0100, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x0F74);
	write_cmos_sensor(0x6F12, 0x0040);	 // 64
	write_cmos_sensor(0x6F12, 0x0040);	 // 64
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0344, 0x0008);	 // 8
	write_cmos_sensor(0x0346, 0x0008);	 // 8
	write_cmos_sensor(0x0348, 0x1077);	 // 4215
	write_cmos_sensor(0x034A, 0x0C37);	 // 3127
	write_cmos_sensor(0x034C, 0x0838);	 // 2104
	write_cmos_sensor(0x034E, 0x0618);	 // 1560
	write_cmos_sensor(0x0900, 0x0112);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0003);
	write_cmos_sensor(0x0400, 0x0001);
	write_cmos_sensor(0x0404, 0x0020);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0110, 0x0002);
	write_cmos_sensor(0x0136, 0x1800);	 // 24MHz
	write_cmos_sensor(0x0304, 0x0006);	 // 6
	write_cmos_sensor(0x0306, 0x00B1);	 // 177
	write_cmos_sensor(0x0302, 0x0001);	 // 1
	write_cmos_sensor(0x0300, 0x0005);	 // 5
	write_cmos_sensor(0x030C, 0x0006);	 // 6
	write_cmos_sensor(0x030E, 0x0119);	 // 281
	write_cmos_sensor(0x3008, 0x0000);	//mipi s-divider
	write_cmos_sensor(0x31E4, 0x02BC);	//#SenAnalog_AIG_DefCclkFreqMhz 0058   default : 02BC //test tg
	write_cmos_sensor(0x030A, 0x0001);	 // 1
	write_cmos_sensor(0x0308, 0x0008);	 // 8
	write_cmos_sensor(0x0342, 0x16B0);	 // 5808
	write_cmos_sensor(0x0340, 0x0CA2);	 // 3234
	write_cmos_sensor(0x0202, 0x0200);	 // 512
	write_cmos_sensor(0x0200, 0x00C6);	 // 198
	write_cmos_sensor(0x0B04, 0x0101);	//M.BPC_On
	write_cmos_sensor(0x0B08, 0x0100);	//D.BPC_On,0x0000,20151104
	write_cmos_sensor(0x0B00, 0x0007);	//LSC_Off
	write_cmos_sensor(0x316A, 0x00A0);	// OUTIF threshold
	write_cmos_sensor(0x304A, 0x0000);	//#smiaRegs_vendor_sensor_b_use_2nd_frame_settings
}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("%s, currefps:%d\n",__func__, currefps);

#if 1
	//$MV1[MCLK:24,Width:4208,Height:3120,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0100, 0x0000);

	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x0F74);
	write_cmos_sensor(0x6F12, 0x0040);	 // 64
	write_cmos_sensor(0x6F12, 0x0040);	 // 64
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0344, 0x0008);	 // 8
	write_cmos_sensor(0x0346, 0x0008);	 // 8
	write_cmos_sensor(0x0348, 0x1077);	 // 4215
	write_cmos_sensor(0x034A, 0x0C37);	 // 3127
	write_cmos_sensor(0x034C, 0x1070);	 // 4208
	write_cmos_sensor(0x034E, 0x0C30);	 // 3120
	write_cmos_sensor(0x0900, 0x0011);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0001);
	write_cmos_sensor(0x0400, 0x0000);
	write_cmos_sensor(0x0404, 0x0010);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0110, 0x0002);
	write_cmos_sensor(0x0136, 0x1800);	 // 24MHz
	write_cmos_sensor(0x0304, 0x0006);	 // 6
	write_cmos_sensor(0x0306, 0x00B1);	 // 177
	write_cmos_sensor(0x0302, 0x0001);	 // 1
	write_cmos_sensor(0x0300, 0x0005);	 // 5
	write_cmos_sensor(0x030C, 0x0006);	 // 6
	write_cmos_sensor(0x030E, 0x0119);	 // 281
	write_cmos_sensor(0x3008, 0x0000);	//mipi s-divider
	write_cmos_sensor(0x31E4, 0x02BC);	//#SenAnalog_AIG_DefCclkFreqMhz 0058   default : 02BC //test tg
	write_cmos_sensor(0x030A, 0x0001);	 // 1
	write_cmos_sensor(0x0308, 0x0008);	 // 8
	write_cmos_sensor(0x0342, 0x16B0);	 // 5808
	write_cmos_sensor(0x0340, 0x0CA2);	 // 3234
	write_cmos_sensor(0x0202, 0x0200);	 // 512
	write_cmos_sensor(0x0200, 0x00C6);	 // 198
	write_cmos_sensor(0x0B04, 0x0101);	//M.BPC_On
	write_cmos_sensor(0x0B08, 0x0100);	//D.BPC_On,0x0000,20151104
	write_cmos_sensor(0x0B00, 0x0007);	//LSC_Off
	write_cmos_sensor(0x316A, 0x00A0);	// OUTIF threshold
	write_cmos_sensor(0x304A, 0x0000);	//#smiaRegs_vendor_sensor_b_use_2nd_frame_settings
#else
	if (currefps == 300) {
  //$MV1[MCLK:24,Width:4208,Height:3120,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]
  write_cmos_sensor(0x0100, 0x0000);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0F74);
  write_cmos_sensor(0x6F12, 0x0040);
  write_cmos_sensor(0x6F12, 0x0040);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x0344, 0x0008);
  write_cmos_sensor(0x0346, 0x0008);
  write_cmos_sensor(0x0348, 0x1077);
  write_cmos_sensor(0x034A, 0x0C37);
  write_cmos_sensor(0x034C, 0x1070);
  write_cmos_sensor(0x034E, 0x0C30);
  write_cmos_sensor(0x0900, 0x0011);
  write_cmos_sensor(0x0380, 0x0001);
  write_cmos_sensor(0x0382, 0x0001);
  write_cmos_sensor(0x0384, 0x0001);
  write_cmos_sensor(0x0386, 0x0001);
  write_cmos_sensor(0x0400, 0x0000);
  write_cmos_sensor(0x0404, 0x0010);
  write_cmos_sensor(0x0114, 0x0300);
  write_cmos_sensor(0x0110, 0x0002);
  write_cmos_sensor(0x0136, 0x1800);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x0306, 0x00AF);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0300, 0x0005);
  write_cmos_sensor(0x030C, 0x0006);
  write_cmos_sensor(0x030E, 0x0119);
  write_cmos_sensor(0x030A, 0x0001);
  write_cmos_sensor(0x0308, 0x0008);
  write_cmos_sensor(0x0342, 0x16B0);
  write_cmos_sensor(0x0340, 0x0C86);
  write_cmos_sensor(0x0202, 0x0200);
  write_cmos_sensor(0x0200, 0x00C6);
  write_cmos_sensor(0x0B04, 0x0101);
  write_cmos_sensor(0x0B08, 0x0100);	//D.BPC_On,0x0000,20151104
  write_cmos_sensor(0x0B00, 0x0007);
  write_cmos_sensor(0x316A, 0x00A0);
  write_cmos_sensor(0x0100, 0x0100);

	}
	else if (currefps == 240) {
		LOG_INF("else if (currefps == 240)\n");
  //$MV1[MCLK:24,Width:4208,Height:3120,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]
  write_cmos_sensor(0x0100, 0x0000);
  write_cmos_sensor(0x6028, 0x2000);
  write_cmos_sensor(0x602A, 0x0F74);
  write_cmos_sensor(0x6F12, 0x0040);
  write_cmos_sensor(0x6F12, 0x0040);
  write_cmos_sensor(0x6028, 0x4000);
  write_cmos_sensor(0x0344, 0x0008);
  write_cmos_sensor(0x0346, 0x0008);
  write_cmos_sensor(0x0348, 0x1077);
  write_cmos_sensor(0x034A, 0x0C37);
  write_cmos_sensor(0x034C, 0x1070);
  write_cmos_sensor(0x034E, 0x0C30);
  write_cmos_sensor(0x0900, 0x0011);
  write_cmos_sensor(0x0380, 0x0001);
  write_cmos_sensor(0x0382, 0x0001);
  write_cmos_sensor(0x0384, 0x0001);
  write_cmos_sensor(0x0386, 0x0001);
  write_cmos_sensor(0x0400, 0x0000);
  write_cmos_sensor(0x0404, 0x0010);
  write_cmos_sensor(0x0114, 0x0300);
  write_cmos_sensor(0x0110, 0x0002);
  write_cmos_sensor(0x0136, 0x1800);
  write_cmos_sensor(0x0304, 0x0006);
  write_cmos_sensor(0x0306, 0x008C);
  write_cmos_sensor(0x0302, 0x0001);
  write_cmos_sensor(0x0300, 0x0005);
  write_cmos_sensor(0x030C, 0x0006);
  write_cmos_sensor(0x030E, 0x0119);
  write_cmos_sensor(0x030A, 0x0001);
  write_cmos_sensor(0x0308, 0x0008);
  write_cmos_sensor(0x0342, 0x16B0);
  write_cmos_sensor(0x0340, 0x0C86);
  write_cmos_sensor(0x0202, 0x0200);
  write_cmos_sensor(0x0200, 0x00C6);
  write_cmos_sensor(0x0B04, 0x0101);
  write_cmos_sensor(0x0B08, 0x0100);	//D.BPC_On,0x0000,20151104
  write_cmos_sensor(0x0B00, 0x0007);
  write_cmos_sensor(0x316A, 0x00A0);
  write_cmos_sensor(0x0100, 0x0100);

	}
	else if (currefps == 150) {
//PIP 15fps settings,ç›¸æ¯”Full 30fps
//    -VT : 560-> 400M
//    -Frame length: 3206-> 4589
//   -Linelength: 5808ä¸è®Š
//
//$MV1[MCLK:24,Width:4208,Height:3120,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]
		LOG_INF("else if (currefps == 150)\n");
//$MV1[MCLK:24,Width:4208,Height:3120,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]
write_cmos_sensor(0x0100, 0x0000);
write_cmos_sensor(0x6028, 0x2000);	
write_cmos_sensor(0x602A, 0x0F74);	
write_cmos_sensor(0x6F12, 0x0040);	 // 64
write_cmos_sensor(0x6F12, 0x0040);	 // 64
write_cmos_sensor(0x6028, 0x4000);	
write_cmos_sensor(0x0344, 0x0008);	 // 8
write_cmos_sensor(0x0346, 0x0008);	 // 8
write_cmos_sensor(0x0348, 0x1077);	 // 4215
write_cmos_sensor(0x034A, 0x0C37);	 // 3127
write_cmos_sensor(0x034C, 0x1070);	 // 4208
write_cmos_sensor(0x034E, 0x0C30);	 // 3120
write_cmos_sensor(0x0900, 0x0011);	
write_cmos_sensor(0x0380, 0x0001);	
write_cmos_sensor(0x0382, 0x0001);	
write_cmos_sensor(0x0384, 0x0001);	
write_cmos_sensor(0x0386, 0x0001);	
write_cmos_sensor(0x0400, 0x0000);	
write_cmos_sensor(0x0404, 0x0010);	
write_cmos_sensor(0x0114, 0x0300);	
write_cmos_sensor(0x0110, 0x0002);	
write_cmos_sensor(0x0136, 0x1800);	 // 24MHz
write_cmos_sensor(0x0304, 0x0006);	 // 6
write_cmos_sensor(0x0306, 0x007D);	 // 125
write_cmos_sensor(0x0302, 0x0001);	 // 1
write_cmos_sensor(0x0300, 0x0005);	 // 5
write_cmos_sensor(0x030C, 0x0006);	 // 6
write_cmos_sensor(0x030E, 0x00C8);	 // 
write_cmos_sensor(0x030A, 0x0001);	 // 1
write_cmos_sensor(0x0308, 0x0008);	 // 8
write_cmos_sensor(0x0342, 0x16B0);	 // 5808
write_cmos_sensor(0x0340, 0x11ED);	 // 4589
write_cmos_sensor(0x0202, 0x0200);	 // 512
write_cmos_sensor(0x0200, 0x00C6);	 // 198
write_cmos_sensor(0x0B04, 0x0101);	//M.BPC_On
write_cmos_sensor(0x0B08, 0x0100);	//D.BPC_On,0x0000,20151104
write_cmos_sensor(0x0B00, 0x0007);	//LSC_Off
write_cmos_sensor(0x316A, 0x00A0);	// OUTIF threshold
write_cmos_sensor(0x0100, 0x0100);
	}
	else { //default fps =15
//PIP 15fps settings,ç›¸æ¯”Full 30fps
//    -VT : 560-> 400M
//    -Frame length: 3206-> 4589
//   -Linelength: 5808ä¸è®Š
//
//$MV1[MCLK:24,Width:4208,Height:3120,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]
		LOG_INF("else  150fps\n");
//$MV1[MCLK:24,Width:4208,Height:3120,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]
write_cmos_sensor(0x0100, 0x0000);
write_cmos_sensor(0x6028, 0x2000);	
write_cmos_sensor(0x602A, 0x0F74);	
write_cmos_sensor(0x6F12, 0x0040);	 // 64
write_cmos_sensor(0x6F12, 0x0040);	 // 64
write_cmos_sensor(0x6028, 0x4000);	
write_cmos_sensor(0x0344, 0x0008);	 // 8
write_cmos_sensor(0x0346, 0x0008);	 // 8
write_cmos_sensor(0x0348, 0x1077);	 // 4215
write_cmos_sensor(0x034A, 0x0C37);	 // 3127
write_cmos_sensor(0x034C, 0x1070);	 // 4208
write_cmos_sensor(0x034E, 0x0C30);	 // 3120
write_cmos_sensor(0x0900, 0x0011);	
write_cmos_sensor(0x0380, 0x0001);	
write_cmos_sensor(0x0382, 0x0001);	
write_cmos_sensor(0x0384, 0x0001);	
write_cmos_sensor(0x0386, 0x0001);	
write_cmos_sensor(0x0400, 0x0000);	
write_cmos_sensor(0x0404, 0x0010);	
write_cmos_sensor(0x0114, 0x0300);	
write_cmos_sensor(0x0110, 0x0002);	
write_cmos_sensor(0x0136, 0x1800);	 // 24MHz
write_cmos_sensor(0x0304, 0x0006);	 // 6
write_cmos_sensor(0x0306, 0x007D);	 // 125
write_cmos_sensor(0x0302, 0x0001);	 // 1
write_cmos_sensor(0x0300, 0x0005);	 // 5
write_cmos_sensor(0x030C, 0x0006);	 // 6
write_cmos_sensor(0x030E, 0x00C8);	 // 
write_cmos_sensor(0x030A, 0x0001);	 // 1
write_cmos_sensor(0x0308, 0x0008);	 // 8
write_cmos_sensor(0x0342, 0x16B0);	 // 5808
write_cmos_sensor(0x0340, 0x11ED);	 // 4589
write_cmos_sensor(0x0202, 0x0200);	 // 512
write_cmos_sensor(0x0200, 0x00C6);	 // 198
write_cmos_sensor(0x0B04, 0x0101);	//M.BPC_On
write_cmos_sensor(0x0B08, 0x0100);	//D.BPC_On,0x0000,20151104
write_cmos_sensor(0x0B00, 0x0007);	//LSC_Off
write_cmos_sensor(0x316A, 0x00A0);	// OUTIF threshold
write_cmos_sensor(0x0100, 0x0100);
	}
#endif
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("%s, currefps:%d\n",__func__, currefps);
	preview_setting();
	write_cmos_sensor(0x0100, 0x0100);
}

static void hd_4k_video_setting()
{
	LOG_INF("%s.\n", __func__);
	//$MV1[MCLK:24,Width:3840,Height:2160,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0100, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x0F74);
	write_cmos_sensor(0x6F12, 0x0040);	 // 64
	write_cmos_sensor(0x6F12, 0x0040);	 // 64
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0344, 0x00C0);	 // 192
	write_cmos_sensor(0x0346, 0x01E8);	 // 488
	write_cmos_sensor(0x0348, 0x0FBF);	 // 4031
	write_cmos_sensor(0x034A, 0x0A67);	 // 2663
	write_cmos_sensor(0x034C, 0x0F00);	 // 3840
	write_cmos_sensor(0x034E, 0x0880);	 // 2176
	write_cmos_sensor(0x0900, 0x0011);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0001);
	write_cmos_sensor(0x0400, 0x0000);
	write_cmos_sensor(0x0404, 0x0010);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0110, 0x0002);
	write_cmos_sensor(0x0136, 0x1800);	 // 24MHz
	write_cmos_sensor(0x0304, 0x0006);	 // 6
	write_cmos_sensor(0x0306, 0x00B1);	 // 177
	write_cmos_sensor(0x0302, 0x0001);	 // 1
	write_cmos_sensor(0x0300, 0x0005);	 // 5
	write_cmos_sensor(0x030C, 0x0006);	 // 6
	write_cmos_sensor(0x030E, 0x0119);	 // 281
	write_cmos_sensor(0x3008, 0x0000);	//mipi s-divider
	write_cmos_sensor(0x31E4, 0x02BC);	//#SenAnalog_AIG_DefCclkFreqMhz 0058   default : 02BC //test tg
	write_cmos_sensor(0x030A, 0x0001);	 // 1
	write_cmos_sensor(0x0308, 0x0008);	 // 8
	write_cmos_sensor(0x0342, 0x16B0);	 // 5808
	write_cmos_sensor(0x0340, 0x0CA2);	 // 3234
	write_cmos_sensor(0x0202, 0x0200);	 // 512
	write_cmos_sensor(0x0200, 0x00C6);	 // 198
	write_cmos_sensor(0x0B04, 0x0101);	//M.BPC_On
	write_cmos_sensor(0x0B08, 0x0100);	//D.BPC_On,0x0000,20151104
	write_cmos_sensor(0x0B00, 0x0007);	//LSC_Off
	write_cmos_sensor(0x316A, 0x00A0);	// OUTIF threshold
	write_cmos_sensor(0x304A, 0x0000);	//#smiaRegs_vendor_sensor_b_use_2nd_frame_settings
	write_cmos_sensor(0x0100, 0x0100);
}

static void hs_video_setting()
{
	LOG_INF("%s.\n", __func__);
	//$MV1[MCLK:24,Width:1052,Height:780,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0100, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x0F74);
	write_cmos_sensor(0x6F12, 0x0040);	 // 64
	write_cmos_sensor(0x6F12, 0x0040);	 // 64
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0344, 0x0008);	 // 8
	write_cmos_sensor(0x0346, 0x0008);	 // 8
	write_cmos_sensor(0x0348, 0x1077);	 // 4215);
	write_cmos_sensor(0x034A, 0x0C37);	 // 3127);
	write_cmos_sensor(0x034C, 0x041C);	 // 1052);
	write_cmos_sensor(0x034E, 0x030C);	 // 780);
	write_cmos_sensor(0x0900, 0x0114);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0007);
	write_cmos_sensor(0x0400, 0x0001);
	write_cmos_sensor(0x0404, 0x0040);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0110, 0x0002);
	write_cmos_sensor(0x0136, 0x1800);	 // 24MHz
	write_cmos_sensor(0x0304, 0x0006);	 // 6
	write_cmos_sensor(0x0306, 0x00B1);	 // 177
	write_cmos_sensor(0x0302, 0x0001);	 // 1
	write_cmos_sensor(0x0300, 0x0005);	 // 5
	write_cmos_sensor(0x030C, 0x0006);	 // 6
	write_cmos_sensor(0x030E, 0x0119);	 // 281
	write_cmos_sensor(0x3008, 0x0000);	//mipi s-divider
	write_cmos_sensor(0x31E4, 0x02BC);	//#SenAnalog_AIG_DefCclkFreqMhz 0058   default : 02BC //test tg
	write_cmos_sensor(0x030A, 0x0001);	 // 1
	write_cmos_sensor(0x0308, 0x0008);	 // 8
	write_cmos_sensor(0x0342, 0x16B0);	 // 5808
	write_cmos_sensor(0x0340, 0x032C);	 // 812
	write_cmos_sensor(0x0202, 0x0200);	 // 512
	write_cmos_sensor(0x0200, 0x00C6);	 // 198
	write_cmos_sensor(0x0B04, 0x0101);	//M.BPC_On
	write_cmos_sensor(0x0B08, 0x0100);	//D.BPC_On,0x0000,20151104
	write_cmos_sensor(0x0B00, 0x0007);	//LSC_Off
	write_cmos_sensor(0x316A, 0x00A0);	// OUTIF threshold
	write_cmos_sensor(0x304A, 0x0000);	//#smiaRegs_vendor_sensor_b_use_2nd_frame_settings
	write_cmos_sensor(0x0100, 0x0100);
}

static void slim_video_setting()
{
	LOG_INF("%s.\n", __func__);
	//$MV1[MCLK:24,Width:1052,Height:780,Format:MIPI_Raw10,mipi_lane:4,mipi_datarate:1124,pvi_pclk_inverse:0]
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0100, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x0F74);
	write_cmos_sensor(0x6F12, 0x0040);	 // 64
	write_cmos_sensor(0x6F12, 0x0040);	 // 64
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0344, 0x0008);	 // 8
	write_cmos_sensor(0x0346, 0x0008);	 // 8
	write_cmos_sensor(0x0348, 0x1077);	 // 4215);
	write_cmos_sensor(0x034A, 0x0C37);	 // 3127);
	write_cmos_sensor(0x034C, 0x041C);	 // 1052);
	write_cmos_sensor(0x034E, 0x030C);	 // 780);
	write_cmos_sensor(0x0900, 0x0114);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0001);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0007);
	write_cmos_sensor(0x0400, 0x0001);
	write_cmos_sensor(0x0404, 0x0040);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0x0110, 0x0002);
	write_cmos_sensor(0x0136, 0x1800);	 // 24MHz
	write_cmos_sensor(0x0304, 0x0006);	 // 6
	write_cmos_sensor(0x0306, 0x00B1);	 // 177
	write_cmos_sensor(0x0302, 0x0001);	 // 1
	write_cmos_sensor(0x0300, 0x0005);	 // 5
	write_cmos_sensor(0x030C, 0x0006);	 // 6
	write_cmos_sensor(0x030E, 0x0119);	 // 281
	write_cmos_sensor(0x3008, 0x0000);	//mipi s-divider
	write_cmos_sensor(0x31E4, 0x02BC);	//#SenAnalog_AIG_DefCclkFreqMhz 0058   default : 02BC //test tg
	write_cmos_sensor(0x030A, 0x0001);	 // 1
	write_cmos_sensor(0x0308, 0x0008);	 // 8
	write_cmos_sensor(0x0342, 0x16B0);	 // 5808
	write_cmos_sensor(0x0340, 0x0CA2);	 // 3234
	write_cmos_sensor(0x0202, 0x0200);	 // 512
	write_cmos_sensor(0x0200, 0x00C6);	 // 198
	write_cmos_sensor(0x0B04, 0x0101);	//M.BPC_On
	write_cmos_sensor(0x0B08, 0x0100);	//D.BPC_On,0x0000,20151104
	write_cmos_sensor(0x0B00, 0x0007);	//LSC_Off
	write_cmos_sensor(0x316A, 0x00A0);	// OUTIF threshold
	write_cmos_sensor(0x304A, 0x0000);	//#smiaRegs_vendor_sensor_b_use_2nd_frame_settings
	write_cmos_sensor(0x0100, 0x0100);
}


/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	u8 module_id = 0;
	kal_uint8 i2c_write_id;
	int ret = 0;
	int j;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		i2c_write_id = imgsensor.i2c_write_id;
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001));
			ret = s5k3l8_read_otp(0x0001, &module_id);
			if ((ret >= 0) && (*sensor_id == imgsensor_info.sensor_id) && (module_id == 0x05)) {
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x, module_id:0x%x\n", imgsensor.i2c_write_id,*sensor_id, module_id);
				*sensor_id = S5K3L8_OFILM_SENSOR_ID;
				goto otp_read;
			}	
			LOG_INF("Read sensor id fail, addr: 0x%x, sensor_id:0x%x, module_id:0x%x\n", imgsensor.i2c_write_id,*sensor_id, module_id);
			retry--;
		} while(retry > 0);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = i2c_write_id;
		spin_unlock(&imgsensor_drv_lock);
		i++;
		retry = 1;
	}
	if ((ret < 0) || (*sensor_id != imgsensor_info.sensor_id)
	    || (module_id != 0x05)) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

otp_read:
	/*
	 * read lsc calibration from OTP E2PROM.
	 */
	s5k3l8_ofilm_otp_buf = (u8 *)kzalloc(OTP_DATA_SIZE, GFP_KERNEL);

	/* read lsc calibration from E2PROM */
	s5k3l8_read_otp_burst(OTP_START_ADDR, s5k3l8_ofilm_otp_buf);
	for (j = 0; j < OTP_DATA_SIZE; j++) {
		LOG_INF("========s5k3l8_ofilm_otp RegIndex-%d=====val:0x%x======\n", j, *(s5k3l8_ofilm_otp_buf + j));
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0; 
	LOG_1;
	//LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n");
	
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	do {
		sensor_id = ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001));
		if (sensor_id == imgsensor_info.sensor_id) {
			LOG_INF("i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id,sensor_id,imgsensor_info.sensor_id);	
			break;
		}
		LOG_INF("Read sensor id fail, i2c write id: 0x%x, ReadOut sensor id: 0x%x, imgsensor_info.sensor_id:0x%x.\n", imgsensor.i2c_write_id,sensor_id,imgsensor_info.sensor_id);	
		retry--;
	} while(retry > 0);

	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	
	/* initail sequence write in  */
	sensor_init();
	
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x2D00;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("%s.\n", __func__);

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	mode_change = 1;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.current_fps = 300;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(IMAGE_NORMAL);	
#if 0
	mdelay(10);
	#ifdef FANPENGTAO
	int i=0;
	for(i=0; i<10; i++){
		LOG_INF("delay time = %d, the frame no = %d\n", i*10, read_cmos_sensor(0x0005));
		mdelay(10);
	}
	#endif
#endif
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	mode_change = 1;
	LOG_INF("capture30fps: use cap30FPS's setting: %d fps!\n",imgsensor.current_fps/10);
	imgsensor.pclk = imgsensor_info.cap.pclk;
	imgsensor.line_length = imgsensor_info.cap.linelength;
	imgsensor.frame_length = imgsensor_info.cap.framelength;  
	imgsensor.min_frame_length = imgsensor_info.cap.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
#if 0
	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) {
		LOG_INF("capture30fps: use cap30FPS's setting: %d fps!\n",imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} 
	else  
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
		//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		LOG_INF("cap115fps: use cap1's setting: %d fps!\n",imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	else  { //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		LOG_INF("Warning:=== current_fps %d fps is not support, so use cap1's setting\n",imgsensor.current_fps/10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
#endif
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps); 
	set_mirror_flip(IMAGE_NORMAL);	
//	mdelay(10);

#if 0
	if(imgsensor.test_pattern == KAL_TRUE)
	{
		//write_cmos_sensor(0x5002,0x00);
  }
#endif

	return ERROR_NONE;
}	/* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(IMAGE_NORMAL);	
	
	
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("%s.\n", __func__);
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}
	
/*************************************************************************
* FUNCTION
* Custom2
*
* DESCRIPTION
*   This function start the sensor Custom2.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
    imgsensor.pclk = imgsensor_info.custom2.pclk;
    //imgsensor.video_mode = KAL_FALSE;
    imgsensor.line_length = imgsensor_info.custom2.linelength;
    imgsensor.frame_length = imgsensor_info.custom2.framelength; 
    imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hd_4k_video_setting();
    set_mirror_flip(IMAGE_NORMAL);	
    return ERROR_NONE;
}   /*  Custom2   */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("%s.\n", __func__);
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;		

	
	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;
	
	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;

	sensor_resolution->SensorCustom2Width  = imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height     = imgsensor_info.custom2.grabwindow_height;

	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	
	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame; 

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = 1;
	
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num; 
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x 
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

			break;	 
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
	   
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 

			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:			
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc; 

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc; 

			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
			sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;
			break;
		default:			
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}
	
	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;	  
        case MSDK_SCENARIO_ID_CUSTOM2:
            Custom2(image_window, sensor_config_data); //hd_4k_video
            break;
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate) 
{
	kal_uint32 frame_length;
  
	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;			
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			if(framerate==300)
			{
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			else
			{
			frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			}
			set_dummy();			
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();			
        case MSDK_SCENARIO_ID_CUSTOM2:
            frame_length = imgsensor_info.custom2.pclk / framerate * 10 / imgsensor_info.custom2.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.custom2.framelength) ? (frame_length - imgsensor_info.custom2.framelength) : 0;
            if (imgsensor.dummy_line < 0)
                imgsensor.dummy_line = 0;
            imgsensor.frame_length = imgsensor_info.custom2.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
           // set_dummy();            
            break; 
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			set_dummy();	
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}	
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;		
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO: 
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
        case MSDK_SCENARIO_ID_CUSTOM2:
            *framerate = imgsensor_info.custom2.max_framerate;
            break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
   write_cmos_sensor(0x3734, 0x0001);
   write_cmos_sensor(0x0600, 0x0308);
	} else {
   write_cmos_sensor(0x3734, 0x0010);
   write_cmos_sensor(0x0600, 0x0300);
	}	 
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    unsigned long long *feature_return_para=(unsigned long long *) feature_para;

    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    //add for s5k3l8 pdaf
    SET_PD_BLOCK_INFO_T *PDAFinfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    LOG_INF("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_GET_PDAF_DATA:
		//add for s5k3l8 pdaf
		read_s5k3l8_pdaf_data((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
		break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);

            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    copy_to_user((void __user *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    copy_to_user((void __user *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    copy_to_user((void __user *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    copy_to_user((void __user *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
		case MSDK_SCENARIO_ID_CUSTOM2:
                    copy_to_user((void __user *)wininfo,(void *)&imgsensor_winsize_info[5],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    copy_to_user((void __user *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
			break;
        //add for s5k3l8 pdaf
        case SENSOR_FEATURE_GET_PDAF_INFO:
            LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n", (UINT32)*feature_data);
            PDAFinfo= (SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			copy_to_user((void __user *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(SET_PD_BLOCK_INFO_T));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    break;
            }
            break;
        case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
            LOG_INF("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n", (UINT32)*feature_data);
            //PDAF capacity enable or not, ov13853 only full size support PDAF
            switch (*feature_data) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0; // video & capture use same setting
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
                default:
                    *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
                    break;
            }
            break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        case SENSOR_FEATURE_GET_SENSOR_ORIENTATION:
            LOG_INF("sensor orientation:%d\n", imgsensor.orientation);
            *feature_return_para_32 = imgsensor.orientation;
            *feature_para_len=4;
            break;
        default:
            break;
    }

    return ERROR_NONE;
}    /*    feature_control()  */


static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};


//UINT32 S5K3L8_OFILM_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
UINT32 S5K3L8_OFILM_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	S5K3L8_OFILM_MIPI_RAW_SensorInit	*/



