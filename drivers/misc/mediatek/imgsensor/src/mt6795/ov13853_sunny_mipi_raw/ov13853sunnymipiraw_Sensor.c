/*****************************************************************************
 *
 * Filename:
 * ---------
 *     OV13853sunnymipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
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

#include "ov13853sunnymipiraw_Sensor.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "OV13853_sunny_camera_sensor"
#define LOG_1 LOG_INF("OV13853,MIPI 4LANE\n")
#define LOG_2 LOG_INF("preview 2096*1552@30fps,640Mbps/lane; video 4192*3104@30fps,1.2Gbps/lane; capture 13M@30fps,1.2Gbps/lane\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)    xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

static int ov13853_sunny_chip_ver = OV13853_R2A;

u8 *sunny_otp_buf;
#define	MAX_READ_WRITE_SIZE	8
#define	OTP_DATA_SIZE	3400
#define	OTP_START_ADDR	0x0000
#define	E2PROM_WRITE_ID	0xA0
#define	PDAF_DATA_SIZE	1372
#define	PDAF_DATA_OFFSET	0x79B

static kal_uint8 mode_change = 0;

struct ov13853_sunny_write_buffer {
	u8 addr[2];
	u8 data[MAX_READ_WRITE_SIZE];
};

static imgsensor_info_struct imgsensor_info = {
    .sensor_id = OV13853_SUNNY_SENSOR_ID & 0x00FFFF,        //record sensor id defined in Kd_imgsensor.h

    .checksum_value = 0xbde6b5f8,//0xf86cfdf4,        //checksum value for Camera Auto Test

    .pre = {
        .pclk = 240000000,                //record different mode's pclk
        .linelength = 2400,                //record different mode's linelength
        .framelength = 3328,            //record different mode's framelength
        .startx = 0,                    //record different mode's startx of grabwindow
        .starty = 0,                    //record different mode's starty of grabwindow
        .grabwindow_width = 2104,        //record different mode's width of grabwindow
        .grabwindow_height = 1560,        //record different mode's height of grabwindow
        /*     following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario    */
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        /*     following for GetDefaultFramerateByScenario()    */
        .max_framerate = 300,
    },
    .cap = {
        .pclk = 480000000,
        .linelength = 4800,
        .framelength = 3328,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4224, //4224,
        .grabwindow_height = 3136,//3136,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,

    },
    .cap1 = {                            //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
        .pclk = 240000000,
        .linelength = 4800,
        .framelength = 3328,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 4208,
        .grabwindow_height = 3120,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 150,    //less than 13M(include 13M),cap1 max framerate is 24fps,16M max framerate is 20fps, 20M max framerate is 15fps
    },
    .hd_4k_video = {
        .pclk = 480000000,
        .linelength = 4800,
        .framelength = 3328,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3840, //3840,
        .grabwindow_height = 2176,//2176,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,
    },
    .normal_video = {
        .pclk = 240000000,
        .linelength = 2400,
        .framelength = 3328,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 2112,
        .grabwindow_height = 1188,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,
    },
    .hs_video = {
        .pclk = 246000000,
        .linelength = 2400,
        .framelength = 854,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1052,
        .grabwindow_height = 780,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 1200,
    },
    .slim_video = {
        .pclk = 240000000,
        .linelength = 2400,
        .framelength = 3328,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1052,
        .grabwindow_height = 780,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,
    },
    .margin = 16,            //sensor framelength & shutter margin
    .min_shutter = 10,        //min shutter
    .max_frame_length = 0x7ff0,//max framelength by sensor register's limitation
    .ae_shut_delay_frame = 0,    //shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
    .ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
    .ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
    .ihdr_support = 0,      //1, support; 0,not support
    .ihdr_le_firstline = 0,  //1,le first ; 0, se first
    .sensor_mode_num = 6,      //support sensor mode num

    .cap_delay_frame = 0,        //enter capture delay frame num
    .pre_delay_frame = 0,         //enter preview delay frame num
    .video_delay_frame = 0,        //enter video delay frame num
    .hs_video_delay_frame = 0,    //enter high speed video  delay frame num
    .slim_video_delay_frame = 0,//enter slim video delay frame num
    .hd_4k_video_delay_frame = 0,       //enter hd_4k_video delay frame num

    .isp_driving_current = ISP_DRIVING_8MA, //mclk driving current
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,//sensor_interface_type
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_MANUAL,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,//sensor output first pixel color
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
    .mipi_lane_num = SENSOR_MIPI_4_LANE,//mipi lane num
    .i2c_addr_table = {0x6c, 0x20, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static imgsensor_struct imgsensor = {
    .mirror = IMAGE_H_MIRROR,                //mirrorflip information
    .orientation = 90,                //mirrorflip information, 0, 90, 180, 270 degree.
    .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x3D0,                    //current shutter
    .gain = 0x100,                        //current gain
    .dummy_pixel = 0,                    //current dummypixel
    .dummy_line = 0,                    //current dummyline
    .current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE,        //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_en = 0, //sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x20,//record current sensor's i2c write id
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[6] =
{{ 4236, 3136, 12, 0, 4224, 3136, 2112, 1568, 4, 4, 2104, 1560, 0, 0, 2104, 1560}, // Preview 
 { 4244, 3152, 12, 0, 4232, 3152, 4232, 3152, 4, 8, 4224, 3136, 0, 0, 4224, 3136}, // capture 
 { 4256, 3152, 0, 8, 4256, 3144, 2128, 1572, 8, 192, 2112, 1188, 0, 0, 2112, 1188}, // video
// {  4256, 3152, 20, 12,  4216, 3128, 4216, 3128, 4, 4, 4208, 3120, 0, 0, 4192, 3104}, // video 
// { 4256, 3152, 0, 376, 4256, 2392, 1064, 598, 8, 2, 1056, 594, 208, 56, 640, 480}, //hight speed video 
 { 4236, 3136, 12, 0, 4224, 3136, 1056, 784, 2, 2, 1052, 780, 0, 0, 1052, 780},// hight speed video  video
 { 4236, 3136, 12, 0, 4224, 3136, 1056, 784, 2, 2, 1052, 780, 0, 0, 1052, 780},// slim video
 { 4244, 3152, 12, 0, 4232, 3152, 4232, 3152, 196, 488, 3840, 2176, 0, 0, 3840, 2176}}; // 4K video 
#if 0
//add for ov13853 pdaf
static SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
    .i4OffsetX =  0,
    .i4OffsetY = 0,
    .i4PitchX  = 32,
    .i4PitchY  = 32,
    .i4PairNum  =8,
    .i4SubBlkW  =16,
    .i4SubBlkH  =8,//need check xb.pang
    .i4PosL = {{2,6},{18,6},{10,10},{26,10},{6,22},{22,22},{14,26},{30,26}},
    .i4PosR = {{2,2},{18,2},{10,14},{26,14},{6,18},{22,18},{14,30},{30,30}},
};
#else

static SET_PD_BLOCK_INFO_T imgsensor_pd_info =
{
    .i4OffsetX =  64,
    .i4OffsetY = 64,
    .i4PitchX  = 32,
    .i4PitchY  = 32,
    .i4PairNum  =8,
    .i4SubBlkW  =16,
    .i4SubBlkH  =8,//need check xb.pang
    .i4PosL = {{74,66},{90,66},{66,78},{82,78},{74,82},{90,82},{66,94},{82,94}},
    .i4PosR = {{74,70},{90,70},{66,74},{82,74},{74,86},{90,86},{66,90},{82,90}},
};

static SET_PD_BLOCK_INFO_T imgsensor_pd_info_v2 =
{
    .i4OffsetX =  64,
    .i4OffsetY = 64,
    .i4PitchX  = 32,
    .i4PitchY  = 32,
    .i4PairNum  =8,
    .i4SubBlkW  =16,
    .i4SubBlkH  =8,//need check xb.pang
    .i4PosL = {{78,70},{94,70},{70,74},{86,74},{78,86},{94,86},{70,90},{86,90}},
    .i4PosR = {{78,66},{94,66},{70,78},{86,78},{78,82},{94,82},{70,94},{86,94}},
};

#endif
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;

    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

    return get_byte;
}

static void write_cmos_sensor_burst(kal_uint32 addr, u8 *reg_buf, kal_uint32 size)
{
	struct ov13853_sunny_write_buffer buf;
	int i;
	int ret;

	for (i = 0; i < size; i += MAX_READ_WRITE_SIZE) {
		buf.addr[0] = (u8)(addr >> 8);
		buf.addr[1] = (u8)(addr & 0xFF);
		if ((i + MAX_READ_WRITE_SIZE) > size) {
			memcpy(buf.data, (reg_buf + i), (size - i));
			ret = iBurstWriteReg((u8 *)&buf, (size - i + 2), imgsensor.i2c_write_id);
		} else {
			memcpy(buf.data, (reg_buf + i), MAX_READ_WRITE_SIZE);
			ret = iBurstWriteReg((u8 *)&buf, (MAX_READ_WRITE_SIZE + 2), imgsensor.i2c_write_id);
		}

		if (ret < 0)
			LOG_INF("write burst reg into sensor failed!\n");

		addr += MAX_READ_WRITE_SIZE;
	}
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static int ov13853_read_otp(u16 addr, u8 *buf)
{
	int ret = 0;
	u8 pu_send_cmd[2] = {(u8)(addr >> 8), (u8)(addr & 0xFF)};

	ret = iReadRegI2C(pu_send_cmd, 2, (u8*)buf, 1, E2PROM_WRITE_ID);
	if (ret < 0)
		LOG_INF("read data from sunny otp e2prom failed!\n");

	return ret;
}

static void ov13853_read_otp_burst(u16 addr, u8 *otp_buf)
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
			LOG_INF("read lsc table from sunny otp e2prom failed!\n");

		addr += MAX_READ_WRITE_SIZE;
	}
}

static void read_ov13853_pdaf_data(kal_uint16 addr, BYTE* data, kal_uint32 size)
{
	copy_to_user((void __user *)data,(void *)(sunny_otp_buf + PDAF_DATA_OFFSET), PDAF_DATA_SIZE);
}

static void set_dummy()
{
    u8 reg_blk[4] = {(char)((imgsensor.line_length>>8)&0xFF), (char)(imgsensor.line_length&0xFF),
			(char)((imgsensor.frame_length>>8)&0x7F), (char)(imgsensor.frame_length&0xFF)};
    LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
    /* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
    write_cmos_sensor_burst(0x380c, reg_blk, sizeof(reg_blk));
}    /*    set_dummy  */

static int get_sensor_temp()
{
	unsigned char temp = 0;
	/* temp read trigger */
	write_cmos_sensor(0x4D12, 0x01);
	temp = read_cmos_sensor(0x4D13);

	return (temp - 64);
}

static kal_uint32 return_sensor_id()
{
    return ((read_cmos_sensor(0x300A) << 8) | read_cmos_sensor(0x300B));
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
    kal_int16 dummy_line;
    kal_uint32 frame_length = imgsensor.frame_length;
    //unsigned long flags;

    LOG_INF("framerate = %d, min framelength should enable = %d \n", framerate,min_framelength_en);

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
}    /*    set_max_framerate  */



/*************************************************************************
* FUNCTION
*    set_shutter
*
* DESCRIPTION
*    This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*    iShutter : exposured lines
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint32 shutter)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    kal_uint32 frame_length = 0;
    kal_uint32 line_length = 0;
    kal_uint32 origi_line_length;
    kal_uint32 origi_shutter = 0,usr_shutter = shutter;
    kal_uint16 b2d_flag = 0;
    u8 reg_shutter[3];
    u8 reg_blk[4];

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    origi_line_length = imgsensor.line_length;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	LOG_INF("Enter! shutter =%d \n", shutter);

	/* Just be called in capture mode with long exp */
	if (usr_shutter == 3)
		write_cmos_sensor(0x0100, 0x00);

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
	shutter = origi_shutter * 120 / 480;
	if (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) {
		line_length = origi_shutter * 120 / (imgsensor_info.max_frame_length - imgsensor_info.margin) * imgsensor.line_length / 480;
		line_length = (line_length >> 1) << 1;
		imgsensor.line_length = line_length;
	}
	write_cmos_sensor(0x0303, 0x03);	//MIPI rate 300Mbps;
	write_cmos_sensor(0x3612, 0x2F);	//PCLK = 120Mhz;
	write_cmos_sensor(0x4837, 0x35);	//MIPI rate 300Mbps/lane;

	/* DPC setting for long exposure */
	write_cmos_sensor(0x5101, 0x00);
	write_cmos_sensor(0x5102, 0x00);
	write_cmos_sensor(0x5103, 0x00);
	write_cmos_sensor(0x5104, 0x00);
	write_cmos_sensor(0x5105, 0x00);
	write_cmos_sensor(0x5106, 0x00);
	write_cmos_sensor(0x5107, 0x00);
	write_cmos_sensor(0x5108, 0x00);
	write_cmos_sensor(0x5109, 0x00);
	write_cmos_sensor(0x510d, 0xff);
	write_cmos_sensor(0x510e, 0x0f);
	write_cmos_sensor(0x510f, 0xff);
	write_cmos_sensor(0x5110, 0xff);
	write_cmos_sensor(0x5111, 0x15);

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

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else {
		// Extend frame length
/*
		write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8)&0x7f);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x380c, imgsensor.line_length >> 8);
		write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);
*/
        }
    } else {
        // Extend frame length
/*
        write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8)&0x7f);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
	write_cmos_sensor(0x380c, imgsensor.line_length >> 8);
	write_cmos_sensor(0x380d, imgsensor.line_length & 0xFF);
*/
    }

	spin_lock(&imgsensor_drv_lock);
	if (imgsensor.frame_length > imgsensor.last_frame_length)
		b2d_flag = 1;
	else
		b2d_flag = 0;
	spin_unlock(&imgsensor_drv_lock);

	reg_shutter[0] = (u8)((shutter>>12) & 0x0F);
	reg_shutter[1] = (u8)((shutter>>4) & 0xFF);
	reg_shutter[2] = (u8)((shutter<<4) & 0xF0);

	reg_blk[0] = (u8)((imgsensor.line_length>>8)&0xFF);
	reg_blk[1] = (u8)(imgsensor.line_length&0xFF);
	reg_blk[2] = (u8)((imgsensor.frame_length>>8)&0x7F);
	reg_blk[3] = (u8)(imgsensor.frame_length&0xFF);

	if (b2d_flag == 1){
		write_cmos_sensor_burst(0x380c, reg_blk, sizeof(reg_blk));
		/* Update Shutter */
		write_cmos_sensor_burst(0x3500, reg_shutter, sizeof(reg_shutter));
	} else {
		write_cmos_sensor_burst(0x3500, reg_shutter, sizeof(reg_shutter));
		write_cmos_sensor_burst(0x380c, reg_blk, sizeof(reg_blk));
	}

    LOG_INF("Exit! shutter =%d, framelength =%d, line_length:%d, b2d_flag:%d;\n", shutter,imgsensor.frame_length, imgsensor.line_length, b2d_flag);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.line_length = origi_line_length;
    imgsensor.last_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);

	/* Just be called in capture mode with long exp */
	if (usr_shutter == 3) {
	/* DPC setting for normal mode */
	write_cmos_sensor(0x5101, 0x00);
	write_cmos_sensor(0x5102, 0x00);
	write_cmos_sensor(0x5103, 0x00);
	write_cmos_sensor(0x5104, 0x00);
	write_cmos_sensor(0x5105, 0x00);
	write_cmos_sensor(0x5106, 0x00);
	write_cmos_sensor(0x5107, 0x00);
	write_cmos_sensor(0x5108, 0x00);
	write_cmos_sensor(0x5109, 0x00);
	write_cmos_sensor(0x510d, 0xfe);
	write_cmos_sensor(0x510e, 0x00);
	write_cmos_sensor(0x510f, 0xfd);
	write_cmos_sensor(0x5110, 0xf0);
	write_cmos_sensor(0x5111, 0x10);
	write_cmos_sensor(0x0100, 0x01);
	}

     LOG_INF("sensor temperature: %d.\n", get_sensor_temp());
}    /*    set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 iReg = 0x0000;
	kal_uint16 iGain=gain;

	if (ov13853_sunny_chip_ver == OV13853_R1A)
	{
		iReg = gain*32/BASEGAIN;
		if(iReg < 0x20)
		{
			iReg = 0x20;
		}
		if(iReg > 0xfc)
		{
			iReg = 0xfc;
		}
		//SENSORDB("[OV13853Gain2Reg]: isp gain:%d,sensor gain:0x%x\n",iGain,iReg);
	}
	else if(ov13853_sunny_chip_ver == OV13853_R2A)
	{
		iReg = gain*16/BASEGAIN;
		if(iReg < 0x10)
		{
			iReg = 0x10;
		}
		if(iReg > 0xf8)
		{
			iReg = 0xf8;
		}
	}
	return iReg;//ov13853. sensorGlobalGain
}

/*************************************************************************
* FUNCTION
*    set_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    iGain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;
 
		reg_gain = gain2reg(gain);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.gain = reg_gain; 
		spin_unlock(&imgsensor_drv_lock);
		LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
	
		write_cmos_sensor(0x350a, reg_gain >> 8);
		write_cmos_sensor(0x350b, reg_gain & 0xFF);    
		
		/*
		 * WORKAROUND! stream on after set shutter/gain, which will get
		 * first valid frame.
		 */
		if (mode_change && ((imgsensor.sensor_mode == IMGSENSOR_MODE_CAPTURE)
		    || (imgsensor.sensor_mode == IMGSENSOR_MODE_PREVIEW))) {
			write_cmos_sensor(0x0100, 0x01);
			mode_change = 0;
		}
		return gain;
}    /*    set_gain  */

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
	write_cmos_sensor(0x380e, (imgsensor.frame_length >> 8)&0x7f);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);

        write_cmos_sensor(0x3502, (le << 4) & 0xFF);
        write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
        write_cmos_sensor(0x3500, (le >> 12) & 0x0F);

        write_cmos_sensor(0x3508, (se << 4) & 0xFF);
        write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
        write_cmos_sensor(0x3506, (se >> 12) & 0x0F);

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

    switch (image_mirror) {
				case IMAGE_NORMAL:
					write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xFB) | 0x00));
					write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xFB) | 0x00));
					break;
				case IMAGE_H_MIRROR:
					write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xFB) | 0x00));
					write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xFF) | 0x04));
					break;
				case IMAGE_V_MIRROR:
					write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xFF) | 0x04));
					write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xFB) | 0x00));		
					break;
				case IMAGE_HV_MIRROR:
					write_cmos_sensor(0x3820,((read_cmos_sensor(0x3820) & 0xFF) | 0x04));
					write_cmos_sensor(0x3821,((read_cmos_sensor(0x3821) & 0xFF) | 0x04));
					break;
				default:
					LOG_INF("Error image_mirror setting\n");
    }

}

/*************************************************************************
* FUNCTION
*    night_mode
*
* DESCRIPTION
*    This function night mode of sensor.
*
* PARAMETERS
*    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/
}    /*    night_mode    */

static void sensor_init(void)
{
	LOG_INF("sensor init\n");
	write_cmos_sensor(0x0103, 0x01);
	write_cmos_sensor(0x3661, 0x20);
	write_cmos_sensor(0x0303, 0x01);	//for 600Mbps;
	write_cmos_sensor(0x030a, 0x00);
	write_cmos_sensor(0x300f, 0x11);
	write_cmos_sensor(0x3010, 0x01);
	write_cmos_sensor(0x3011, 0x76);
	write_cmos_sensor(0x3012, 0x41);
	write_cmos_sensor(0x3013, 0x12);
	write_cmos_sensor(0x3014, 0x11);
	write_cmos_sensor(0x301f, 0x03);
	write_cmos_sensor(0x3106, 0x00);
	write_cmos_sensor(0x3210, 0x47);
	write_cmos_sensor(0x3500, 0x00);
	write_cmos_sensor(0x3501, 0x60);
	write_cmos_sensor(0x3502, 0x00);
	write_cmos_sensor(0x3506, 0x00);
	write_cmos_sensor(0x3507, 0x02);
	write_cmos_sensor(0x3508, 0x00);
	write_cmos_sensor(0x350a, 0x00);
	write_cmos_sensor(0x350b, 0x80);
	write_cmos_sensor(0x350e, 0x00);
	write_cmos_sensor(0x350f, 0x10);
	write_cmos_sensor(0x351a, 0x00);
	write_cmos_sensor(0x351b, 0x10);
	write_cmos_sensor(0x351c, 0x00);
	write_cmos_sensor(0x351d, 0x20);
	write_cmos_sensor(0x351e, 0x00);
	write_cmos_sensor(0x351f, 0x40);
	write_cmos_sensor(0x3520, 0x00);
	write_cmos_sensor(0x3521, 0x80);
	write_cmos_sensor(0x3600, 0xc0);
	write_cmos_sensor(0x3601, 0xfc);
	write_cmos_sensor(0x3602, 0x02);
	write_cmos_sensor(0x3603, 0x78);
	write_cmos_sensor(0x3604, 0xb1);
	write_cmos_sensor(0x3605, 0x95);
	write_cmos_sensor(0x3606, 0x73);
	write_cmos_sensor(0x3607, 0x07);
	write_cmos_sensor(0x3609, 0x40);
	write_cmos_sensor(0x360a, 0x30);
	write_cmos_sensor(0x360b, 0x91);
	write_cmos_sensor(0x360c, 0x09);
	write_cmos_sensor(0x360f, 0x02);
	write_cmos_sensor(0x3611, 0x10);
	write_cmos_sensor(0x3612, 0x27);
	write_cmos_sensor(0x3613, 0x33);
	write_cmos_sensor(0x3615, 0x0c);
	write_cmos_sensor(0x3616, 0x0e);
	write_cmos_sensor(0x3641, 0x02);
	write_cmos_sensor(0x3660, 0x82);
	write_cmos_sensor(0x3668, 0x54);
	write_cmos_sensor(0x3669, 0x00);
	write_cmos_sensor(0x366a, 0x3f);
	write_cmos_sensor(0x3667, 0xa0);
	write_cmos_sensor(0x3702, 0x40);
	write_cmos_sensor(0x3703, 0x44);
	write_cmos_sensor(0x3704, 0x2c);
	write_cmos_sensor(0x3705, 0x01);
	write_cmos_sensor(0x3706, 0x15);
	write_cmos_sensor(0x3707, 0x44);
	write_cmos_sensor(0x3708, 0x3c);
	write_cmos_sensor(0x3709, 0x1f);
	write_cmos_sensor(0x370a, 0x27);
	write_cmos_sensor(0x370b, 0x3c);
	write_cmos_sensor(0x3720, 0x55);
	write_cmos_sensor(0x3722, 0x84);
	write_cmos_sensor(0x3726, 0x22);
	write_cmos_sensor(0x3727, 0x44);
	write_cmos_sensor(0x3728, 0x40);
	write_cmos_sensor(0x3729, 0x00);
	write_cmos_sensor(0x372a, 0x00);
	write_cmos_sensor(0x372b, 0x02);
	write_cmos_sensor(0x372e, 0x22);
	write_cmos_sensor(0x372f, 0xa0);
	write_cmos_sensor(0x3730, 0x00);
	write_cmos_sensor(0x3731, 0x00);
	write_cmos_sensor(0x3732, 0x00);
	write_cmos_sensor(0x3733, 0x00);
	write_cmos_sensor(0x3710, 0x28);
	write_cmos_sensor(0x3716, 0x03);
	write_cmos_sensor(0x3718, 0x1c);
	write_cmos_sensor(0x3719, 0x0c);
	write_cmos_sensor(0x371a, 0x08);
	write_cmos_sensor(0x371c, 0xfc);
	write_cmos_sensor(0x3748, 0x00);
	write_cmos_sensor(0x3760, 0x13);
	write_cmos_sensor(0x3761, 0x33);
	write_cmos_sensor(0x3762, 0x86);
	write_cmos_sensor(0x3763, 0x16);
	write_cmos_sensor(0x3767, 0x24);
	write_cmos_sensor(0x3768, 0x06);
	write_cmos_sensor(0x3769, 0x45);
	write_cmos_sensor(0x376c, 0x23);
	write_cmos_sensor(0x376f, 0x80);
	write_cmos_sensor(0x3773, 0x06);
	write_cmos_sensor(0x3d84, 0x00);
	write_cmos_sensor(0x3d85, 0x17);
	write_cmos_sensor(0x3d8c, 0x73);
	write_cmos_sensor(0x3d8d, 0xbf);
	write_cmos_sensor(0x3800, 0x00);
	write_cmos_sensor(0x3801, 0x00);
	write_cmos_sensor(0x3802, 0x00);
	write_cmos_sensor(0x3803, 0x08);
	write_cmos_sensor(0x3804, 0x10);
	write_cmos_sensor(0x3805, 0x9f);
	write_cmos_sensor(0x3806, 0x0c);
	write_cmos_sensor(0x3807, 0x4f);
	write_cmos_sensor(0x3808, 0x08);
	write_cmos_sensor(0x3809, 0x40);
	write_cmos_sensor(0x380a, 0x06);
	write_cmos_sensor(0x380b, 0x20);
	write_cmos_sensor(0x380c, 0x09);
	write_cmos_sensor(0x380d, 0x60);
	write_cmos_sensor(0x380e, 0x0d);
	write_cmos_sensor(0x380f, 0x00);
	write_cmos_sensor(0x3810, 0x00);
	write_cmos_sensor(0x3811, 0x08);
	write_cmos_sensor(0x3812, 0x00);
	write_cmos_sensor(0x3813, 0x02);
	write_cmos_sensor(0x3814, 0x31);
	write_cmos_sensor(0x3815, 0x31);
	write_cmos_sensor(0x3820, 0x01);
	write_cmos_sensor(0x3821, 0x06);
	write_cmos_sensor(0x3823, 0x00);
	write_cmos_sensor(0x3826, 0x00);
	write_cmos_sensor(0x3827, 0x02);
	write_cmos_sensor(0x3834, 0x00);
	write_cmos_sensor(0x3835, 0x1c);
	write_cmos_sensor(0x3836, 0x08);
	write_cmos_sensor(0x3837, 0x02);
	write_cmos_sensor(0x4000, 0xf1);
	write_cmos_sensor(0x4001, 0x00);
	write_cmos_sensor(0x4005, 0x40);
	write_cmos_sensor(0x4006, 0x01);
	write_cmos_sensor(0x4007, 0x01);
	write_cmos_sensor(0x400b, 0x0c);
	write_cmos_sensor(0x4011, 0x00);
	write_cmos_sensor(0x401a, 0x00);
	write_cmos_sensor(0x401b, 0x00);
	write_cmos_sensor(0x401c, 0x00);
	write_cmos_sensor(0x401d, 0x00);
	write_cmos_sensor(0x401f, 0x00);
	write_cmos_sensor(0x4020, 0x00);
	write_cmos_sensor(0x4021, 0xe4);
	write_cmos_sensor(0x4022, 0x04);
	write_cmos_sensor(0x4023, 0xd7);
	write_cmos_sensor(0x4024, 0x05);
	write_cmos_sensor(0x4025, 0xbc);
	write_cmos_sensor(0x4026, 0x05);
	write_cmos_sensor(0x4027, 0xbf);
	write_cmos_sensor(0x4028, 0x00);
	write_cmos_sensor(0x4029, 0x02);
	write_cmos_sensor(0x402a, 0x04);
	write_cmos_sensor(0x402b, 0x08);
	write_cmos_sensor(0x402c, 0x02);
	write_cmos_sensor(0x402d, 0x02);
	write_cmos_sensor(0x402e, 0x0c);
	write_cmos_sensor(0x402f, 0x08);
	write_cmos_sensor(0x403d, 0x2c);
	write_cmos_sensor(0x403f, 0x7f);
	write_cmos_sensor(0x4041, 0x07);
	write_cmos_sensor(0x4500, 0x82);
	write_cmos_sensor(0x4501, 0x78);	//H digital skip for PD pixel; 3C for H digital bin;
	write_cmos_sensor(0x458b, 0x00);
	write_cmos_sensor(0x459c, 0x00);
	write_cmos_sensor(0x459d, 0x00);
	write_cmos_sensor(0x459e, 0x00);
	write_cmos_sensor(0x4601, 0x83);
	write_cmos_sensor(0x4602, 0x22);
	write_cmos_sensor(0x4603, 0x01);
	write_cmos_sensor(0x4837, 0x1b);
	write_cmos_sensor(0x4d00, 0x04);
	write_cmos_sensor(0x4d01, 0x42);
	write_cmos_sensor(0x4d02, 0xd1);
	write_cmos_sensor(0x4d03, 0x90);
	write_cmos_sensor(0x4d04, 0x66);
	write_cmos_sensor(0x4d05, 0x65);
	write_cmos_sensor(0x4d0b, 0x00);
	write_cmos_sensor(0x5000, 0x0e);
	write_cmos_sensor(0x5001, 0x01);
	write_cmos_sensor(0x5002, 0x07);
	write_cmos_sensor(0x5003, 0x4f);
	write_cmos_sensor(0x5004, 0x0d);
	write_cmos_sensor(0x5013, 0x40);
	write_cmos_sensor(0x501c, 0x00);
	write_cmos_sensor(0x501d, 0x10);
	write_cmos_sensor(0x510f, 0xfc);
	write_cmos_sensor(0x5110, 0xf0);
	write_cmos_sensor(0x5111, 0x10);
	write_cmos_sensor(0x5115, 0x3c);
	write_cmos_sensor(0x5116, 0x30);
	write_cmos_sensor(0x5117, 0x10);
	write_cmos_sensor(0x536d, 0x02);
	write_cmos_sensor(0x536e, 0x67);
	write_cmos_sensor(0x536f, 0x01);
	write_cmos_sensor(0x5370, 0x4c);
	write_cmos_sensor(0x5400, 0x00);
	write_cmos_sensor(0x5400, 0x00);
	write_cmos_sensor(0x5401, 0x61);
	write_cmos_sensor(0x5402, 0x00);
	write_cmos_sensor(0x5403, 0x00);
	write_cmos_sensor(0x5404, 0x00);
	write_cmos_sensor(0x5405, 0x40);
	write_cmos_sensor(0x540c, 0x05);
	write_cmos_sensor(0x5501, 0x00);
	write_cmos_sensor(0x5b00, 0x00);
	write_cmos_sensor(0x5b01, 0x00);
	write_cmos_sensor(0x5b02, 0x01);
	write_cmos_sensor(0x5b03, 0xff);
	write_cmos_sensor(0x5b04, 0x02);
	write_cmos_sensor(0x5b05, 0x6c);
	write_cmos_sensor(0x5b09, 0x02);
	write_cmos_sensor(0x5e00, 0x00);
	write_cmos_sensor(0x5e10, 0x1c);
	write_cmos_sensor(0x0100, 0x01);
	write_cmos_sensor(0x0102, 0x01);
	write_cmos_sensor(0x5b20, 0x02);
	write_cmos_sensor(0x5b21, 0x37);
	write_cmos_sensor(0x5b22, 0x0c);
	write_cmos_sensor(0x5b23, 0x37);
	write_cmos_sensor(0x5100, 0x30);
	write_cmos_sensor(0x5101, 0x00);
	write_cmos_sensor(0x5102, 0x00);
	write_cmos_sensor(0x5103, 0x00);
	write_cmos_sensor(0x5104, 0x00);
	write_cmos_sensor(0x5105, 0x00);
	write_cmos_sensor(0x5106, 0x00);
	write_cmos_sensor(0x5107, 0x00);
	write_cmos_sensor(0x5108, 0x00);
	write_cmos_sensor(0x5109, 0x00);
	write_cmos_sensor(0x510d, 0xfe);
	write_cmos_sensor(0x510e, 0x00);
	write_cmos_sensor(0x510f, 0xfd);
	write_cmos_sensor(0x5110, 0xf0);
	write_cmos_sensor(0x5111, 0x10);
	write_cmos_sensor(0x5112, 0x77);
	write_cmos_sensor(0x5113, 0x07);
	write_cmos_sensor(0x5116, 0x3f);
	write_cmos_sensor(0x0100, 0x0);
}    /*    sensor_init  */


static void preview_setting(void)
{
	LOG_INF("preview setting\n");
	/*
	 * @@2112x1568_30fps
	 * 2112x1568_30fps_H_skip_V_Digbin_without_PD_600Mbps
	 * 100 99 2112 1568
	 * 102 3601 BB8
	 * XVCLK=24Mhz, PCLK=4x60Mhz, MIPI 600Mbps, DACCLK=240Mhz
	 * line_length=2400,frame_length=3328
	 */
	write_cmos_sensor(0x0100, 0x00);
	write_cmos_sensor(0x0300, 0x00);
	write_cmos_sensor(0x0301, 0x00);
	write_cmos_sensor(0x0302, 0x32);
	write_cmos_sensor(0x0303, 0x01);	//for 600Mbps;
	write_cmos_sensor(0x3500, 0x00);
	write_cmos_sensor(0x3501, 0x60);
	write_cmos_sensor(0x3612, 0x27);
	write_cmos_sensor(0x3614, 0x28);
	write_cmos_sensor(0x370a, 0x27);
	write_cmos_sensor(0x3729, 0x00);
	write_cmos_sensor(0x372a, 0x00);
	write_cmos_sensor(0x372f, 0xa0);
	write_cmos_sensor(0x3718, 0x1c);
	write_cmos_sensor(0x3801, 0x0C);
	write_cmos_sensor(0x3802, 0x00);
	write_cmos_sensor(0x3803, 0x00);
	write_cmos_sensor(0x3804, 0x10);
	write_cmos_sensor(0x3805, 0x8B);
	write_cmos_sensor(0x3806, 0x0c);
	write_cmos_sensor(0x3807, 0x3F);
	write_cmos_sensor(0x3808, 0x08);
	write_cmos_sensor(0x3809, 0x38);
	write_cmos_sensor(0x380a, 0x06);
	write_cmos_sensor(0x380b, 0x18);
	write_cmos_sensor(0x380c, 0x09);
	write_cmos_sensor(0x380d, 0x60);
	write_cmos_sensor(0x380e, 0x0d);
	write_cmos_sensor(0x380f, 0x00);
	write_cmos_sensor(0x3810, 0x00);
	write_cmos_sensor(0x3811, 0x04);
	write_cmos_sensor(0x3812, 0x00);
	write_cmos_sensor(0x3813, 0x04);
	write_cmos_sensor(0x3814, 0x31);
	write_cmos_sensor(0x3815, 0x31);
	write_cmos_sensor(0x3820, 0x01);
	write_cmos_sensor(0x3821, 0x06);
	write_cmos_sensor(0x3834, 0x00);
	write_cmos_sensor(0x3836, 0x08);
	write_cmos_sensor(0x3837, 0x02);
	write_cmos_sensor(0x4020, 0x00);
	write_cmos_sensor(0x4021, 0xe4);
	write_cmos_sensor(0x4022, 0x04);
	write_cmos_sensor(0x4023, 0xd7);
	write_cmos_sensor(0x4024, 0x05);
	write_cmos_sensor(0x4025, 0xbc);
	write_cmos_sensor(0x4026, 0x05);
	write_cmos_sensor(0x4027, 0xbf);
	write_cmos_sensor(0x402a, 0x04);
	write_cmos_sensor(0x402b, 0x08);
	write_cmos_sensor(0x402c, 0x02);
	write_cmos_sensor(0x402e, 0x0c);
	write_cmos_sensor(0x402f, 0x08);
	write_cmos_sensor(0x4501, 0x78);	//H digital skip for PD pixel; 3C for H digital bin;
	write_cmos_sensor(0x4601, 0x83);
	write_cmos_sensor(0x4603, 0x01);
	write_cmos_sensor(0x4837, 0x1b);
	write_cmos_sensor(0x5401, 0x61);
	write_cmos_sensor(0x5405, 0x40);
	write_cmos_sensor(0x5b20, 0x00);
	write_cmos_sensor(0x5b21, 0x37);
	write_cmos_sensor(0x5b22, 0x0c);
	write_cmos_sensor(0x5b23, 0x37);

	write_cmos_sensor(0x5101, 0x00);
	write_cmos_sensor(0x5102, 0x00);
	write_cmos_sensor(0x5103, 0x00);
	write_cmos_sensor(0x5104, 0x00);
	write_cmos_sensor(0x5105, 0x00);
	write_cmos_sensor(0x5106, 0x00);
	write_cmos_sensor(0x5107, 0x00);
	write_cmos_sensor(0x5108, 0x00);
	write_cmos_sensor(0x5109, 0x00);
	write_cmos_sensor(0x510d, 0xfe);
	write_cmos_sensor(0x510e, 0x00);
	write_cmos_sensor(0x510f, 0xfd);
	write_cmos_sensor(0x5110, 0xf0);
	write_cmos_sensor(0x5111, 0x10);
}    /*    preview_setting  */

static int capture_first_flag = 0;
static int pre_currefps = 0;
static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("capture setting! currefps:%d\n",currefps);	
#if 1
        write_cmos_sensor(0x0100, 0x00); //
	write_cmos_sensor(0x0300, 0x00);
	write_cmos_sensor(0x0301, 0x00);
	write_cmos_sensor(0x0302, 0x32);
	write_cmos_sensor(0x0303, 0x00);	//for 1200Mbps;
	write_cmos_sensor(0x3501, 0xc0);
	write_cmos_sensor(0x3612, 0x07);
	write_cmos_sensor(0x3614, 0x28);
	write_cmos_sensor(0x370a, 0x24);
	write_cmos_sensor(0x3729, 0x00);
	write_cmos_sensor(0x372a, 0x04);
	write_cmos_sensor(0x372f, 0xa0);
	write_cmos_sensor(0x3718, 0x1c);
	write_cmos_sensor(0x3801, 0x0C);
	write_cmos_sensor(0x3802, 0x00);
	write_cmos_sensor(0x3803, 0x00);
	write_cmos_sensor(0x3804, 0x10);
	write_cmos_sensor(0x3805, 0x93);
	write_cmos_sensor(0x3806, 0x0c);
	write_cmos_sensor(0x3807, 0x4F);
	write_cmos_sensor(0x3808, 0x10);
	write_cmos_sensor(0x3809, 0x80);
	write_cmos_sensor(0x380a, 0x0c);
	write_cmos_sensor(0x380b, 0x40);
	write_cmos_sensor(0x380c, 0x12);
	write_cmos_sensor(0x380d, 0xc0);
	write_cmos_sensor(0x380e, 0x0d);
	write_cmos_sensor(0x380f, 0x00);
	write_cmos_sensor(0x3810, 0x00);
	write_cmos_sensor(0x3811, 0x04);
	write_cmos_sensor(0x3812, 0x00);
	write_cmos_sensor(0x3813, 0x08);
	write_cmos_sensor(0x3814, 0x11);
	write_cmos_sensor(0x3815, 0x11);
	write_cmos_sensor(0x3820, 0x00);
	write_cmos_sensor(0x3821, 0x04);
	write_cmos_sensor(0x3834, 0x00);
	write_cmos_sensor(0x3836, 0x04);
	write_cmos_sensor(0x3837, 0x01);
	write_cmos_sensor(0x4020, 0x02);
	write_cmos_sensor(0x4021, 0x4C);
	write_cmos_sensor(0x4022, 0x0E);
	write_cmos_sensor(0x4023, 0x37);
	write_cmos_sensor(0x4024, 0x0F);
	write_cmos_sensor(0x4025, 0x1C);
	write_cmos_sensor(0x4026, 0x0F);
	write_cmos_sensor(0x4027, 0x1F);
	write_cmos_sensor(0x402a, 0x04);
	write_cmos_sensor(0x402b, 0x08);
	write_cmos_sensor(0x402c, 0x02);
	write_cmos_sensor(0x402e, 0x0c);
	write_cmos_sensor(0x402f, 0x08);
	write_cmos_sensor(0x4501, 0x38);
	write_cmos_sensor(0x4601, 0x04);
	write_cmos_sensor(0x4603, 0x00);
	write_cmos_sensor(0x4837, 0x0d);
	write_cmos_sensor(0x5401, 0x71);
	write_cmos_sensor(0x5405, 0x80);
	write_cmos_sensor(0x5b20, 0x02);
	write_cmos_sensor(0x5b21, 0x91);
	write_cmos_sensor(0x5b22, 0x02);
	write_cmos_sensor(0x5b23, 0x91);

	write_cmos_sensor(0x5101, 0x00);
	write_cmos_sensor(0x5102, 0x00);
	write_cmos_sensor(0x5103, 0x00);
	write_cmos_sensor(0x5104, 0x00);
	write_cmos_sensor(0x5105, 0x00);
	write_cmos_sensor(0x5106, 0x00);
	write_cmos_sensor(0x5107, 0x00);
	write_cmos_sensor(0x5108, 0x00);
	write_cmos_sensor(0x5109, 0x00);
	write_cmos_sensor(0x510d, 0xfe);
	write_cmos_sensor(0x510e, 0x00);
	write_cmos_sensor(0x510f, 0xfd);
	write_cmos_sensor(0x5110, 0xf0);
	write_cmos_sensor(0x5111, 0x10);
#else
if (pre_currefps != currefps)
{
	capture_first_flag = 0;
	pre_currefps = currefps;
}
else
{
	capture_first_flag = 1;
}
if (capture_first_flag == 0)
{

    if (currefps == 150) { //full size 15fps for PIP
        write_cmos_sensor(0x0100, 0x00); //
		write_cmos_sensor(0x0300, 0x00); //	;for 600Mbps
		write_cmos_sensor(0x0301, 0x00); //
		write_cmos_sensor(0x0302, 0x32); //
		write_cmos_sensor(0x0303, 0x01); // 
		write_cmos_sensor(0x3501, 0xc0); // 
		write_cmos_sensor(0x3612, 0x27); // 
		write_cmos_sensor(0x370a, 0x24); // 
		write_cmos_sensor(0x3729, 0x00); // 
		write_cmos_sensor(0x372a, 0x04); // 
		write_cmos_sensor(0x3718, 0x10); // 
		write_cmos_sensor(0x372f, 0xa0); // 
		write_cmos_sensor(0x3801, 0x0C); // 
		write_cmos_sensor(0x3802, 0x00); //
		write_cmos_sensor(0x3803, 0x00); // 
		write_cmos_sensor(0x3805, 0x93); // 
		write_cmos_sensor(0x3806, 0x0c); //
		write_cmos_sensor(0x3807, 0x4F); // 
		write_cmos_sensor(0x3808, 0x10); // 
		write_cmos_sensor(0x3809, 0x80); // 
		write_cmos_sensor(0x380a, 0x0c); // 
		write_cmos_sensor(0x380b, 0x40); // 
		//write_cmos_sensor(0x380c, 0x12); // 
		//write_cmos_sensor(0x380d, 0xc0); // 
		//write_cmos_sensor(0x380e, 0x0d); // 
		//write_cmos_sensor(0x380f, 0x00); // 
		write_cmos_sensor(0x380c, ((imgsensor_info.cap1.linelength >> 8) & 0xFF)); // hts = 2688
      	write_cmos_sensor(0x380d, (imgsensor_info.cap1.linelength & 0xFF));        // hts
      	write_cmos_sensor(0x380e, ((imgsensor_info.cap1.framelength >> 8) & 0xFF));  // vts = 1984
      	write_cmos_sensor(0x380f, (imgsensor_info.cap1.framelength & 0xFF));         // vts
		write_cmos_sensor(0x3810, 0x00); // 
		write_cmos_sensor(0x3811, 0x04); // 
		write_cmos_sensor(0x3813, 0x08); // 
		write_cmos_sensor(0x3814, 0x11); // 
		write_cmos_sensor(0x3815, 0x11); // 
		write_cmos_sensor(0x3820, 0x00); // 
		write_cmos_sensor(0x3821, 0x04); // 
		write_cmos_sensor(0x3836, 0x04); // 
		write_cmos_sensor(0x3837, 0x01); // 
		write_cmos_sensor(0x4020, 0x02); // 
		write_cmos_sensor(0x4021, 0x4C); // 
		write_cmos_sensor(0x4022, 0x0E); // 
		write_cmos_sensor(0x4023, 0x37); // 
		write_cmos_sensor(0x4024, 0x0F); // 
		write_cmos_sensor(0x4025, 0x1C); // 
		write_cmos_sensor(0x4026, 0x0F); // 
		write_cmos_sensor(0x4027, 0x1F); // 
		write_cmos_sensor(0x4501, 0x38); // 
		write_cmos_sensor(0x4601, 0x04); // 
		write_cmos_sensor(0x4603, 0x00); // 
		write_cmos_sensor(0x4837, 0x1b); // 
		write_cmos_sensor(0x5401, 0x71); // 
		write_cmos_sensor(0x5405, 0x80); // 
		write_cmos_sensor(0x5b21, 0x91); //
		write_cmos_sensor(0x5b22, 0x02); // 
		write_cmos_sensor(0x5b23, 0x91); // 
		write_cmos_sensor(0x0100, 0x01); 

    } else {   //30fps            //30fps for Normal capture & ZSD
 			//full size @ 30fps
      		write_cmos_sensor(0x0100, 0x00); //
			write_cmos_sensor(0x0300, 0x00); //	;for 1200Mbps
			write_cmos_sensor(0x0301, 0x00); //
			write_cmos_sensor(0x0302, 0x32); //
			write_cmos_sensor(0x0303, 0x00); //;01 
			write_cmos_sensor(0x3501, 0xc0); // 
			write_cmos_sensor(0x3612, 0x07); //;27 
			write_cmos_sensor(0x370a, 0x24); // 
			write_cmos_sensor(0x3729, 0x00); // 
			write_cmos_sensor(0x372a, 0x04); // 
			write_cmos_sensor(0x3718, 0x10); // 
			write_cmos_sensor(0x372f, 0xa0); // 
			write_cmos_sensor(0x3801, 0x0C); // 
			write_cmos_sensor(0x3802, 0x00); //
			write_cmos_sensor(0x3803, 0x00); // 
			write_cmos_sensor(0x3805, 0x93); // 
			write_cmos_sensor(0x3806, 0x0c); //
			write_cmos_sensor(0x3807, 0x4F); // 
			write_cmos_sensor(0x3808, 0x10); // 
			write_cmos_sensor(0x3809, 0x80); // 
			write_cmos_sensor(0x380a, 0x0c); // 
			write_cmos_sensor(0x380b, 0x40); // 
			//write_cmos_sensor(0x380c, 0x12); // 
			//write_cmos_sensor(0x380d, 0xc0); // 
			//write_cmos_sensor(0x380e, 0x0d); // 
			//write_cmos_sensor(0x380f, 0x00); // 
			write_cmos_sensor(0x380c, ((imgsensor_info.cap.linelength >> 8) & 0xFF)); // hts = 2688
      		write_cmos_sensor(0x380d, (imgsensor_info.cap.linelength & 0xFF));        // hts
      		write_cmos_sensor(0x380e, ((imgsensor_info.cap.framelength >> 8) & 0xFF));  // vts = 1984
      		write_cmos_sensor(0x380f, (imgsensor_info.cap.framelength & 0xFF));         // vts
			write_cmos_sensor(0x3810, 0x00); // 
			write_cmos_sensor(0x3811, 0x04); // 
			write_cmos_sensor(0x3813, 0x08); // 
			write_cmos_sensor(0x3814, 0x11); // 
			write_cmos_sensor(0x3815, 0x11); // 
			write_cmos_sensor(0x3820, 0x00); // 
			write_cmos_sensor(0x3821, 0x04); // 
			write_cmos_sensor(0x3836, 0x04); // 
			write_cmos_sensor(0x3837, 0x01); // 
			write_cmos_sensor(0x4020, 0x02); // 
			write_cmos_sensor(0x4021, 0x4C); // 
			write_cmos_sensor(0x4022, 0x0E); // 
			write_cmos_sensor(0x4023, 0x37); // 
			write_cmos_sensor(0x4024, 0x0F); // 
			write_cmos_sensor(0x4025, 0x1C); // 
			write_cmos_sensor(0x4026, 0x0F); // 
			write_cmos_sensor(0x4027, 0x1F); // 
			write_cmos_sensor(0x4501, 0x38); // 
			write_cmos_sensor(0x4601, 0x04); // 
			write_cmos_sensor(0x4603, 0x00); // 
			write_cmos_sensor(0x4837, 0x0d); //;1b 
			write_cmos_sensor(0x5401, 0x71); // 
			write_cmos_sensor(0x5405, 0x80); // 
			write_cmos_sensor(0x5b21, 0x91); //
			write_cmos_sensor(0x5b22, 0x02); // 
			write_cmos_sensor(0x5b23, 0x91); // 
			write_cmos_sensor(0x0100, 0x01); ////


        if (imgsensor.ihdr_en) {

    } else {

    }

    }
	mdelay(20);	
	capture_first_flag = 1;

}

msleep(20);
#endif
}

static void hd_4k_video_setting(void)
{
	LOG_INF("hd 4K video setting!\n");
        write_cmos_sensor(0x0100, 0x00);
	write_cmos_sensor(0x0300, 0x00);
	write_cmos_sensor(0x0301, 0x00);
	write_cmos_sensor(0x0302, 0x32);
	write_cmos_sensor(0x0303, 0x00);	//for 1200Mbps;
	write_cmos_sensor(0x3501, 0xc0);
	write_cmos_sensor(0x3612, 0x07);
	write_cmos_sensor(0x3614, 0x28);
	write_cmos_sensor(0x370a, 0x24);
	write_cmos_sensor(0x3729, 0x00);
	write_cmos_sensor(0x372a, 0x04);
	write_cmos_sensor(0x372f, 0xa0);
	write_cmos_sensor(0x3718, 0x1c);
	write_cmos_sensor(0x3801, 0x0C);
	write_cmos_sensor(0x3802, 0x00);
	write_cmos_sensor(0x3803, 0x00);
	write_cmos_sensor(0x3804, 0x10);
	write_cmos_sensor(0x3805, 0x93);
	write_cmos_sensor(0x3806, 0x0c);
	write_cmos_sensor(0x3807, 0x4F);
	write_cmos_sensor(0x3808, 0x0f);
	write_cmos_sensor(0x3809, 0x00);
	write_cmos_sensor(0x380a, 0x08);
	write_cmos_sensor(0x380b, 0x80);
	write_cmos_sensor(0x380c, 0x12);
	write_cmos_sensor(0x380d, 0xc0);
	write_cmos_sensor(0x380e, 0x0d);
	write_cmos_sensor(0x380f, 0x00);
	write_cmos_sensor(0x3810, 0x00);
	write_cmos_sensor(0x3811, 0xc4);
	write_cmos_sensor(0x3812, 0x01);
	write_cmos_sensor(0x3813, 0xE8);
	write_cmos_sensor(0x3814, 0x11);
	write_cmos_sensor(0x3815, 0x11);
	write_cmos_sensor(0x3820, 0x00);
	write_cmos_sensor(0x3821, 0x04);
	write_cmos_sensor(0x3834, 0x00);
	write_cmos_sensor(0x3836, 0x04);
	write_cmos_sensor(0x3837, 0x01);
	write_cmos_sensor(0x4020, 0x02);
	write_cmos_sensor(0x4021, 0x4C);
	write_cmos_sensor(0x4022, 0x0E);
	write_cmos_sensor(0x4023, 0x37);
	write_cmos_sensor(0x4024, 0x0F);
	write_cmos_sensor(0x4025, 0x1C);
	write_cmos_sensor(0x4026, 0x0F);
	write_cmos_sensor(0x4027, 0x1F);
	write_cmos_sensor(0x402a, 0x04);
	write_cmos_sensor(0x402b, 0x08);
	write_cmos_sensor(0x402c, 0x02);
	write_cmos_sensor(0x402e, 0x0c);
	write_cmos_sensor(0x402f, 0x08);
	write_cmos_sensor(0x4501, 0x38);
	write_cmos_sensor(0x4601, 0x04);
	write_cmos_sensor(0x4603, 0x00);
	write_cmos_sensor(0x4837, 0x0d);
	write_cmos_sensor(0x5401, 0x71);
	write_cmos_sensor(0x5405, 0x80);
	write_cmos_sensor(0x5b20, 0x00);
	write_cmos_sensor(0x5b21, 0x91);
	write_cmos_sensor(0x5b22, 0x02);
	write_cmos_sensor(0x5b23, 0x91);
	write_cmos_sensor(0x0100, 0x01);
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("normal video setting! currefps:%d\n",currefps);
	write_cmos_sensor(0x0100, 0x00);
	write_cmos_sensor(0x0300, 0x00);
	write_cmos_sensor(0x0301, 0x00);
	write_cmos_sensor(0x0302, 0x32);
	write_cmos_sensor(0x0303, 0x01);	//for 600Mbps
	write_cmos_sensor(0x3501, 0x60);
	write_cmos_sensor(0x3612, 0x27);
	write_cmos_sensor(0x3614, 0x28);
	write_cmos_sensor(0x370a, 0x27);
	write_cmos_sensor(0x3729, 0x00);
	write_cmos_sensor(0x372a, 0x00);
	write_cmos_sensor(0x372f, 0xa0);
	write_cmos_sensor(0x3718, 0x1c);
	write_cmos_sensor(0x3801, 0x00);
	write_cmos_sensor(0x3802, 0x00);
	write_cmos_sensor(0x3803, 0x08);
	write_cmos_sensor(0x3804, 0x10);
	write_cmos_sensor(0x3805, 0x9f);
	write_cmos_sensor(0x3806, 0x0c);
	write_cmos_sensor(0x3807, 0x4f);
	write_cmos_sensor(0x3808, 0x08);
	write_cmos_sensor(0x3809, 0x40);
	write_cmos_sensor(0x380a, 0x04);
	write_cmos_sensor(0x380b, 0xa4);
	write_cmos_sensor(0x380c, 0x09);
	write_cmos_sensor(0x380d, 0x60);
	write_cmos_sensor(0x380e, 0x0d);
	write_cmos_sensor(0x380f, 0x00);
	write_cmos_sensor(0x3810, 0x00);
	write_cmos_sensor(0x3811, 0x08);
	write_cmos_sensor(0x3812, 0x00);
	write_cmos_sensor(0x3813, 0xc0);
	write_cmos_sensor(0x3814, 0x31);
	write_cmos_sensor(0x3815, 0x31);
	write_cmos_sensor(0x3820, 0x01);
	write_cmos_sensor(0x3821, 0x06);
	write_cmos_sensor(0x3834, 0x00);
	write_cmos_sensor(0x3836, 0x08);
	write_cmos_sensor(0x3837, 0x02);
	write_cmos_sensor(0x4020, 0x00);
	write_cmos_sensor(0x4021, 0xe4);
	write_cmos_sensor(0x4022, 0x04);
	write_cmos_sensor(0x4023, 0xd7);
	write_cmos_sensor(0x4024, 0x05);
	write_cmos_sensor(0x4025, 0xbc);
	write_cmos_sensor(0x4026, 0x05);
	write_cmos_sensor(0x4027, 0xbf);
	write_cmos_sensor(0x402a, 0x04);
	write_cmos_sensor(0x402b, 0x08);
	write_cmos_sensor(0x402c, 0x02);
	write_cmos_sensor(0x402e, 0x0c);
	write_cmos_sensor(0x402f, 0x08);
	write_cmos_sensor(0x4501, 0x78);	//H digital skip for PD pixel; 3C for H digital bin
	write_cmos_sensor(0x4601, 0x83);
	write_cmos_sensor(0x4603, 0x01);
	write_cmos_sensor(0x4837, 0x1b);
	write_cmos_sensor(0x5401, 0x61);
	write_cmos_sensor(0x5405, 0x40);
	write_cmos_sensor(0x5b20, 0x00);
	write_cmos_sensor(0x5b21, 0x37);
	write_cmos_sensor(0x5b22, 0x0c);
	write_cmos_sensor(0x5b23, 0x37);
	write_cmos_sensor(0x0100, 0x01); //
}
static void hs_video_setting()
{
    LOG_INF("hs video setting!\n");

#if 0
 	write_cmos_sensor(0x0100, 0x00); //
	write_cmos_sensor(0x0300, 0x01); //	;for 640Mbps
	write_cmos_sensor(0x0301, 0x00); //
	write_cmos_sensor(0x0302, 0x28); //
	write_cmos_sensor(0x0303, 0x00); //
	write_cmos_sensor(0x3501, 0x60); // 
	write_cmos_sensor(0x3612, 0x27); // 
	write_cmos_sensor(0x370a, 0x27); //	;ppchg_offset shift 
	write_cmos_sensor(0x3729, 0x00); //	;VFPN line shift 
	write_cmos_sensor(0x372a, 0x00); //
	write_cmos_sensor(0x3718, 0x10); //  
	write_cmos_sensor(0x372f, 0x90); // 
	write_cmos_sensor(0x3801, 0x00); // 
	write_cmos_sensor(0x3802, 0x03); // 
	write_cmos_sensor(0x3803, 0x58); // ;shift 2 rows in V direction 
	write_cmos_sensor(0x3805, 0x9f); // 
	write_cmos_sensor(0x3806, 0x09); // 
	write_cmos_sensor(0x3807, 0x01); // 
	write_cmos_sensor(0x3808, 0x05); // 
	write_cmos_sensor(0x3809, 0x00); // 
	write_cmos_sensor(0x380a, 0x02); // 
	write_cmos_sensor(0x380b, 0xD0); // 
	//write_cmos_sensor(0x380c, 0x09); // 
	//write_cmos_sensor(0x380d, 0x60); // 
	//write_cmos_sensor(0x380e, 0x03); // 
	//write_cmos_sensor(0x380f, 0x3f); // 
	write_cmos_sensor(0x380c, ((imgsensor_info.hs_video.linelength >> 8) & 0xFF)); // hts = 2688
    write_cmos_sensor(0x380d, (imgsensor_info.hs_video.linelength & 0xFF));        // hts
    write_cmos_sensor(0x380e, ((imgsensor_info.hs_video.framelength >> 8) & 0xFF));  // vts = 1984
    write_cmos_sensor(0x380f, (imgsensor_info.hs_video.framelength & 0xFF));         // vts
	write_cmos_sensor(0x3810, 0x03); // 
	write_cmos_sensor(0x3811, 0x44); // 
	write_cmos_sensor(0x3813, 0x02); // 
	write_cmos_sensor(0x3814, 0x31); // 
	write_cmos_sensor(0x3815, 0x31); // 
	write_cmos_sensor(0x3820, 0x00); //
	write_cmos_sensor(0x3821, 0x06); // 
	write_cmos_sensor(0x3836, 0x08); // 
	write_cmos_sensor(0x3837, 0x02); // 
	write_cmos_sensor(0x4020, 0x00); // 
	write_cmos_sensor(0x4021, 0xe4); // 
	write_cmos_sensor(0x4022, 0x04); // 
	write_cmos_sensor(0x4023, 0xd7); // 
	write_cmos_sensor(0x4024, 0x05); // 
	write_cmos_sensor(0x4025, 0xbc); // 
	write_cmos_sensor(0x4026, 0x05); // 
	write_cmos_sensor(0x4027, 0xbf); // 
	write_cmos_sensor(0x4501, 0x78); //	;H digital skip for PD pixel; 3C for H digital bin
	write_cmos_sensor(0x4601, 0x04); //
	write_cmos_sensor(0x4603, 0x00); //
	write_cmos_sensor(0x4837, 0x19); // 
	write_cmos_sensor(0x5401, 0x61); // 
	write_cmos_sensor(0x5405, 0x40); // 
	write_cmos_sensor(0x5b21, 0x37); // 
	write_cmos_sensor(0x5b22, 0x0c); // 
	write_cmos_sensor(0x5b23, 0x37); // 
#else
	write_cmos_sensor(0x0100, 0x00);
	write_cmos_sensor(0x0300, 0x00);
	write_cmos_sensor(0x0301, 0x00);
	write_cmos_sensor(0x0302, 0x32);
	write_cmos_sensor(0x0303, 0x01);	//for 600Mbps;
	write_cmos_sensor(0x3501, 0x30);
	write_cmos_sensor(0x3612, 0x27);
	write_cmos_sensor(0x3614, 0x29);
	write_cmos_sensor(0x370a, 0xa9);
	write_cmos_sensor(0x3729, 0x00);
	write_cmos_sensor(0x372a, 0x00);
	write_cmos_sensor(0x372f, 0x88);
	write_cmos_sensor(0x3718, 0x10);
	write_cmos_sensor(0x3801, 0x0C);
	write_cmos_sensor(0x3802, 0x00);
	write_cmos_sensor(0x3803, 0x00);
	write_cmos_sensor(0x3804, 0x10);
	write_cmos_sensor(0x3805, 0x8B);
	write_cmos_sensor(0x3806, 0x0c);
	write_cmos_sensor(0x3807, 0x3F);
	write_cmos_sensor(0x3808, 0x04);
	write_cmos_sensor(0x3809, 0x1C);
	write_cmos_sensor(0x380a, 0x03);
	write_cmos_sensor(0x380b, 0x0C);
	write_cmos_sensor(0x380c, 0x09);
	write_cmos_sensor(0x380d, 0x60);
	write_cmos_sensor(0x380e, 0x03);
	write_cmos_sensor(0x380f, 0x56);
	write_cmos_sensor(0x3810, 0x00);
	write_cmos_sensor(0x3811, 0x02);
	write_cmos_sensor(0x3812, 0x00);
	write_cmos_sensor(0x3813, 0x02);
	write_cmos_sensor(0x3814, 0x31);
	write_cmos_sensor(0x3815, 0x35);
	write_cmos_sensor(0x3820, 0x02);
	write_cmos_sensor(0x3821, 0x06);
	write_cmos_sensor(0x3834, 0x02);
	write_cmos_sensor(0x3836, 0x08);
	write_cmos_sensor(0x3836, 0x08);
	write_cmos_sensor(0x3837, 0x04);
	write_cmos_sensor(0x4020, 0x00);
	write_cmos_sensor(0x4021, 0xe4);
	write_cmos_sensor(0x4022, 0x03);
	write_cmos_sensor(0x4023, 0x3f);
	write_cmos_sensor(0x4024, 0x04);
	write_cmos_sensor(0x4025, 0x20);
	write_cmos_sensor(0x4026, 0x04);
	write_cmos_sensor(0x4027, 0x25);
	write_cmos_sensor(0x402a, 0x02);
	write_cmos_sensor(0x402b, 0x04);
	write_cmos_sensor(0x402c, 0x06);
	write_cmos_sensor(0x402e, 0x08);
	write_cmos_sensor(0x402f, 0x04);
	write_cmos_sensor(0x4501, 0x3c);
	write_cmos_sensor(0x4601, 0x04);
	write_cmos_sensor(0x4603, 0x00);
	write_cmos_sensor(0x4837, 0x1b);
	write_cmos_sensor(0x5401, 0x51);
	write_cmos_sensor(0x5405, 0x20);
	write_cmos_sensor(0x5b20, 0x00);
	write_cmos_sensor(0x5b21, 0x37);
	write_cmos_sensor(0x5b22, 0x0c);
	write_cmos_sensor(0x5b23, 0x37);
#endif
	write_cmos_sensor(0x0100, 0x01);
}

static void slim_video_setting()
{
    LOG_INF("slim video setting!");
#if 0
	write_cmos_sensor(0x0100, 0x00); //
	write_cmos_sensor(0x0300, 0x01); // ;for 640Mbps
	write_cmos_sensor(0x0301, 0x00); //
	write_cmos_sensor(0x0302, 0x28); //
	write_cmos_sensor(0x0303, 0x00); //
	write_cmos_sensor(0x3501, 0x60); // 
	write_cmos_sensor(0x3612, 0x27); // 
	write_cmos_sensor(0x370a, 0x27); // ;ppchg_offset shift 
	write_cmos_sensor(0x3729, 0x00); // ;VFPN line shift 
	write_cmos_sensor(0x372a, 0x00); //
	write_cmos_sensor(0x3718, 0x10); //  
	write_cmos_sensor(0x372f, 0x90); // 
	write_cmos_sensor(0x3801, 0x00); // 
	write_cmos_sensor(0x3802, 0x03); // 
	write_cmos_sensor(0x3803, 0x58); // ;shift 2 rows in V direction 
	write_cmos_sensor(0x3805, 0x9f); // 
	write_cmos_sensor(0x3806, 0x09); // 
	write_cmos_sensor(0x3807, 0x01); // 
	write_cmos_sensor(0x3808, 0x05); // 
	write_cmos_sensor(0x3809, 0x00); // 
	write_cmos_sensor(0x380a, 0x02); // 
	write_cmos_sensor(0x380b, 0xD0); // 
	//write_cmos_sensor(0x380c, 0x09); // 
	//write_cmos_sensor(0x380d, 0x60); // 
	//write_cmos_sensor(0x380e, 0x0c); // 
	//write_cmos_sensor(0x380f, 0xfc); // 
	write_cmos_sensor(0x380C, ((imgsensor_info.slim_video.linelength >> 8) & 0xFF)); // hts = 9600                      
	write_cmos_sensor(0x380D,     (imgsensor_info.slim_video.linelength & 0xFF));        // hts          
	write_cmos_sensor(0x380E,   ((imgsensor_info.slim_video.framelength >> 8) & 0xFF));  // vts = 834    
	write_cmos_sensor(0x380F,   (imgsensor_info.slim_video.framelength & 0xFF));         // vts       
	write_cmos_sensor(0x3810, 0x03); // 
	write_cmos_sensor(0x3811, 0x44); // 
	write_cmos_sensor(0x3813, 0x02); // 
	write_cmos_sensor(0x3814, 0x31); // 
	write_cmos_sensor(0x3815, 0x31); // 
	write_cmos_sensor(0x3820, 0x00); //
	write_cmos_sensor(0x3821, 0x06); // 
	write_cmos_sensor(0x3836, 0x08); // 
	write_cmos_sensor(0x3837, 0x02); // 
	write_cmos_sensor(0x4020, 0x00); // 
	write_cmos_sensor(0x4021, 0xe4); // 
	write_cmos_sensor(0x4022, 0x04); // 
	write_cmos_sensor(0x4023, 0xd7); // 
	write_cmos_sensor(0x4024, 0x05); // 
	write_cmos_sensor(0x4025, 0xbc); // 
	write_cmos_sensor(0x4026, 0x05); // 
	write_cmos_sensor(0x4027, 0xbf); // 
	write_cmos_sensor(0x4501, 0x78); // ;H digital skip for PD pixel; 3C for H digital bin
	write_cmos_sensor(0x4601, 0x04); //
	write_cmos_sensor(0x4603, 0x00); //
	write_cmos_sensor(0x4837, 0x19); // 
	write_cmos_sensor(0x5401, 0x61); // 
	write_cmos_sensor(0x5405, 0x40); // 
	write_cmos_sensor(0x5b21, 0x37); // 
	write_cmos_sensor(0x5b22, 0x0c); // 
	write_cmos_sensor(0x5b23, 0x37); // 
#else
	write_cmos_sensor(0x0100, 0x00);
	write_cmos_sensor(0x0300, 0x00);
	write_cmos_sensor(0x0301, 0x00);
	write_cmos_sensor(0x0302, 0x32);
	write_cmos_sensor(0x0303, 0x01);	//for 600Mbps;
	write_cmos_sensor(0x3501, 0x30);
	write_cmos_sensor(0x3612, 0x27);
	write_cmos_sensor(0x3614, 0x28);
	write_cmos_sensor(0x370a, 0xa9);
	write_cmos_sensor(0x3729, 0x00);
	write_cmos_sensor(0x372a, 0x00);
	write_cmos_sensor(0x372f, 0x88);
	write_cmos_sensor(0x3718, 0x10);
	write_cmos_sensor(0x3801, 0x0C);
	write_cmos_sensor(0x3802, 0x00);
	write_cmos_sensor(0x3803, 0x00);
	write_cmos_sensor(0x3804, 0x10);
	write_cmos_sensor(0x3805, 0x8B);
	write_cmos_sensor(0x3806, 0x0c);
	write_cmos_sensor(0x3807, 0x3F);
	write_cmos_sensor(0x3808, 0x04);
	write_cmos_sensor(0x3809, 0x1C);
	write_cmos_sensor(0x380a, 0x03);
	write_cmos_sensor(0x380b, 0x0C);
	write_cmos_sensor(0x380c, 0x09);
	write_cmos_sensor(0x380d, 0x60);
	write_cmos_sensor(0x380e, 0x0d);
	write_cmos_sensor(0x380f, 0x00);
	write_cmos_sensor(0x3810, 0x00);
	write_cmos_sensor(0x3811, 0x02);
	write_cmos_sensor(0x3812, 0x00);
	write_cmos_sensor(0x3813, 0x02);
	write_cmos_sensor(0x3814, 0x31);
	write_cmos_sensor(0x3815, 0x35);
	write_cmos_sensor(0x3820, 0x02);
	write_cmos_sensor(0x3821, 0x06);
	write_cmos_sensor(0x3834, 0x02);
	write_cmos_sensor(0x3836, 0x08);
	write_cmos_sensor(0x3836, 0x08);
	write_cmos_sensor(0x3837, 0x04);
	write_cmos_sensor(0x4020, 0x00);
	write_cmos_sensor(0x4021, 0xe4);
	write_cmos_sensor(0x4022, 0x03);
	write_cmos_sensor(0x4023, 0x3f);
	write_cmos_sensor(0x4024, 0x04);
	write_cmos_sensor(0x4025, 0x20);
	write_cmos_sensor(0x4026, 0x04);
	write_cmos_sensor(0x4027, 0x25);
	write_cmos_sensor(0x402a, 0x02);
	write_cmos_sensor(0x402b, 0x04);
	write_cmos_sensor(0x402c, 0x06);
	write_cmos_sensor(0x402e, 0x08);
	write_cmos_sensor(0x402f, 0x04);
	write_cmos_sensor(0x4501, 0x3c);
	write_cmos_sensor(0x4601, 0x04);
	write_cmos_sensor(0x4603, 0x00);
	write_cmos_sensor(0x4837, 0x1b);
	write_cmos_sensor(0x5401, 0x51);
	write_cmos_sensor(0x5405, 0x20);
	write_cmos_sensor(0x5b20, 0x00);
	write_cmos_sensor(0x5b21, 0x37);
	write_cmos_sensor(0x5b22, 0x0c);
	write_cmos_sensor(0x5b23, 0x37);
#endif
	write_cmos_sensor(0x0100, 0x01);
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("enable: %d\n", enable);

    if (enable) {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x5E00, 0x80);
    } else {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x5E00, 0x00);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*    get_imgsensor_id
*
* DESCRIPTION
*    This function get the sensor ID
*
* PARAMETERS
*    *sensorID : return the sensor ID
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    u8 module_id = 0;
    u8 module_ver = 1;
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
            *sensor_id = return_sensor_id();
	    ret = ov13853_read_otp(0x0001, &module_id);
	    if ((ret >= 0) && (*sensor_id == imgsensor_info.sensor_id) && (module_id == 0x07)) {
		LOG_INF("i2c write id: 0x%x, sensor id: 0x%x, module_id:0x%x\n", imgsensor.i2c_write_id,*sensor_id, module_id);
		*sensor_id = OV13853_SUNNY_SENSOR_ID;
                goto otp_read;
            }
	    LOG_INF("Read sensor id fail, addr: 0x%x, sensor_id:0x%x, module_id:0x%x\n", imgsensor.i2c_write_id,*sensor_id, module_id);
            retry--;
        } while(retry > 0);
        spin_lock(&imgsensor_drv_lock);
	imgsensor.i2c_write_id = i2c_write_id;
        spin_unlock(&imgsensor_drv_lock);
        i++;
        retry = 2;
    }
    if ((ret < 0) || (*sensor_id != imgsensor_info.sensor_id)
	    || (module_id != 0x07)) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

otp_read:
	/*
	 * read lsc calibration from OTP E2PROM.
	 */
	sunny_otp_buf = (u8 *)kzalloc(OTP_DATA_SIZE, GFP_KERNEL);

	/* read lsc calibration from E2PROM */
	ov13853_read_otp_burst(OTP_START_ADDR, sunny_otp_buf);
	for (j = 0; j < OTP_DATA_SIZE; j++) {
		LOG_INF("========ov13853_sunny_otp RegIndex-%d=====val:0x%x======\n", j, *(sunny_otp_buf + j));
	}

	ov13853_read_otp(0x0006, &module_ver);
	if (module_ver == 1) {
		imgsensor.orientation = 270;
		imgsensor.mirror = IMAGE_V_MIRROR;
	 } else if (module_ver == 2) {
		imgsensor.orientation = 90;
		imgsensor.mirror = IMAGE_H_MIRROR;
	}

	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*    open
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
    LOG_1;
    LOG_2;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    do {
            sensor_id = return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
    } while(retry > 0);

    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;

#if 0 
		if ((read_cmos_sensor(0x302a))==0xb1)
		{
				LOG_INF("----R1A---- \n");
				ov13853_sunny_chip_ver = OV13853_R1A;
		}else if((read_cmos_sensor(0x302a))==0xb2)
		{
				LOG_INF("----R2A---- \n");
				ov13853_sunny_chip_ver = OV13853_R2A;
		}
#endif
    /* initail sequence write in  */
    sensor_init();

    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.last_frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    //for zsd multi capture setting
	capture_first_flag = 0;
	pre_currefps = 0;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}    /*    open  */



/*************************************************************************
* FUNCTION
*    close
*
* DESCRIPTION
*
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
    LOG_INF("%s.\n", __func__);

    /*No Need to implement this function*/

    return ERROR_NONE;
}    /*    close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*    This function start the sensor preview.
*
* PARAMETERS
*    *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*    None
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
    imgsensor.last_frame_length = imgsensor.frame_length;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    set_mirror_flip(imgsensor.mirror);
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /*    preview   */

/*************************************************************************
* FUNCTION
*    capture
*
* DESCRIPTION
*    This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*    None
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
    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
        LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
    imgsensor.pclk = imgsensor_info.cap.pclk;
    imgsensor.line_length = imgsensor_info.cap.linelength;
    imgsensor.frame_length = imgsensor_info.cap.framelength;
    imgsensor.last_frame_length = imgsensor.frame_length;
    imgsensor.min_frame_length = imgsensor_info.cap.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
#if 0
    if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    } else {
        if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap.max_framerate/10);
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
#endif
    spin_unlock(&imgsensor_drv_lock);
    capture_setting(imgsensor.current_fps);
	  set_mirror_flip(imgsensor.mirror);
    return ERROR_NONE;
}    /* capture() */

static kal_uint32 hd_4k_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s\n", __func__);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_4K_VIDEO;
    imgsensor.pclk = imgsensor_info.hd_4k_video.pclk;
    imgsensor.line_length = imgsensor_info.hd_4k_video.linelength;
    imgsensor.frame_length = imgsensor_info.hd_4k_video.framelength;
    imgsensor.last_frame_length = imgsensor.frame_length;
    imgsensor.min_frame_length = imgsensor_info.hd_4k_video.framelength;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hd_4k_video_setting();
    set_mirror_flip(imgsensor.mirror);
    return ERROR_NONE;
}

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.last_frame_length = imgsensor.frame_length;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    normal_video_setting(imgsensor.current_fps);	
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
		set_mirror_flip(imgsensor.mirror);
    return ERROR_NONE;
}    /*    normal_video   */

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
    imgsensor.last_frame_length = imgsensor.frame_length;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();	
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
		set_mirror_flip(imgsensor.mirror);
    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("%s.\n", __func__);

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.last_frame_length = imgsensor.frame_length;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
		set_mirror_flip(imgsensor.mirror);
    return ERROR_NONE;
}    /*    slim_video     */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("%s.\n", __func__);
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;

    sensor_resolution->SensorCustom2Width = imgsensor_info.hd_4k_video.grabwindow_width;
    sensor_resolution->SensorCustom2Height = imgsensor_info.hd_4k_video.grabwindow_height;

    return ERROR_NONE;
}    /*    get_resolution    */

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
    sensor_info->Custom2DelayFrame = imgsensor_info.hd_4k_video_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
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
    sensor_info->SensorHightSampling = 0;    // 0 is default 1x
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
            sensor_info->SensorGrabStartX = imgsensor_info.hd_4k_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hd_4k_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hd_4k_video.mipi_data_lp2hs_settle_dc;

            break;
        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }

    return ERROR_NONE;
}    /*    get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);
    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			capture_first_flag = 0;
			pre_currefps = 0;
            preview(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            capture(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			capture_first_flag = 0;
			pre_currefps = 0;
            normal_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			capture_first_flag = 0;
			pre_currefps = 0;
            hs_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
			capture_first_flag = 0;
			pre_currefps = 0;
            slim_video(image_window, sensor_config_data);
            break;
	case MSDK_SCENARIO_ID_CUSTOM2:
            hd_4k_video(image_window, sensor_config_data);
            break;
        default:
            LOG_INF("Error ScenarioId setting");
			capture_first_flag = 0;
			pre_currefps = 0;
            preview(image_window, sensor_config_data);
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}    /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{//
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
    set_dummy();

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
//            set_dummy();
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
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            }
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            break;
	case MSDK_SCENARIO_ID_CUSTOM2:
            frame_length = imgsensor_info.hd_4k_video.pclk / framerate * 10 / imgsensor_info.hd_4k_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hd_4k_video.framelength) ? (frame_length - imgsensor_info.hd_4k_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.hd_4k_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            break;
        default:  //coding with  preview scenario by default
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
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
            *framerate = imgsensor_info.hd_4k_video.max_framerate;
            break;
        default:
            break;
    }

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
          //add for ov13853 pdaf
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
          //add for ov13853 pdaf
			read_ov13853_pdaf_data((kal_uint16 )(*feature_data),(char*)(uintptr_t)(*(feature_data+1)),(kal_uint32)(*(feature_data+2)));
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
			//for disable ihdr
			/*
			if(!imgsensor.ihdr_en)
			{
				write_cmos_sensor(0x38210, 0x04);
				write_cmos_sensor(0x38360, 0x04);
				write_cmos_sensor(0x38370, 0x01);
				write_cmos_sensor(0x37670, 0x24);
				write_cmos_sensor(0x40290, 0x02);
				write_cmos_sensor(0x372e0, 0x82);
				write_cmos_sensor(0x40010, 0x00);
				write_cmos_sensor(0x37180, 0x10);
			}
			*/
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
        //add for ov13853 pdaf
        case SENSOR_FEATURE_GET_PDAF_INFO:
            LOG_INF("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n", (UINT32)*feature_data);
            PDAFinfo= (SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		    if (imgsensor.orientation == 270)
			copy_to_user((void __user *)PDAFinfo,(void *)&imgsensor_pd_info,sizeof(SET_PD_BLOCK_INFO_T));
		    else
			copy_to_user((void __user *)PDAFinfo,(void *)&imgsensor_pd_info_v2,sizeof(SET_PD_BLOCK_INFO_T));
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

UINT32 OV13853_SUNNY_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}    /*    OV13853_SUNNY_MIPI_RAW_SensorInit    */
