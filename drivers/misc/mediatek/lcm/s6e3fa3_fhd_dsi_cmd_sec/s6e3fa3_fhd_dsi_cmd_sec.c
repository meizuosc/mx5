#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"
#include <cust_gpio_usage.h>
#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)
#define REGFLAG_DELAY 0xffe
#define REGFLAG_END_OF_TABLE 0xfff


/*OSC CONFIG
 *
 * TP ID:mz_system_lcd_id & 0x07 (0,2,4)(1,3)
 *
 * */

unsigned int mz_system_lcd_id = 0x324101;

#ifndef BUILD_LK
#define lcm_print(fmt, args...) printk(fmt, ##args)
#else
#define lcm_print(fmt, args...) printf(fmt, ##args)
//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

//#define POWER_MODE_5


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmd_by_cmdq_dual(handle,cmd,count,ppara,force_update)    lcm_util.dsi_set_cmdq_V23(handle,cmd,count,ppara,force_update);
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE							1

static unsigned int id_code = 0;
struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table sec_init_code[] = {
};

static struct LCM_setting_table sec_slpout[] = {
	/*sleep out*/
	{0x11, 1, {0}},
	{REGFLAG_DELAY, 20, {}},
	/*TE on (must)*/
	{0x35, 1, {0}},
	{0xFC, 2, {0x5A, 0x5A}},
	{0xB0, 1, {0x1E}},
	{0xFD, 1 ,{0x9E}},
	{0xFC, 2, {0xA5,0xA5}},
	{0x53, 1, {0x20}},
	{0x55, 1, {0x00}},
	{0x51, 1, {0x0}},
	{REGFLAG_DELAY, 180, {}},
	//{0x2c, 1, {1}},
	//{0x3c, 1, {1}},
	{REGFLAG_END_OF_TABLE, 0, {}},
};

static struct LCM_setting_table sec_slpout_v2[] = {
	/*sleep out*/
	{0x11, 1, {0}},
	{REGFLAG_DELAY, 20, {}},
	/*TE on (must)*/
	{0x35, 1, {0}},
	{0xFC, 2, {0x5A, 0x5A}},
	{0xB0, 1, {0x1E}},
	{0xFD, 1 ,{0x9E}},
	{0xFC, 2, {0xA5,0xA5}},
	{0xF0, 2, {0x5A, 0x5A}},
	{0xB0, 1, {0x01}},
	{0xB6, 1 ,{0x01}},
	{0xB4, 1 ,{0x4F}}, //ACL setting
	{0xF0, 2, {0xA5,0xA5}},
	{0x53, 1, {0x20}},
	{0x55, 1, {0x00}},
	{0x51, 1, {0x0}},
	{REGFLAG_DELAY, 80, {}},
	//{0x2c, 1, {1}},
	//{0x3c, 1, {1}},
	{REGFLAG_END_OF_TABLE, 0, {}},
};

static struct LCM_setting_table sec_dispon[] = {
	/*display on*/
	{0x29, 1, {0}},
	{REGFLAG_DELAY, 0, {}},
	{REGFLAG_END_OF_TABLE, 0, {}},
};

static struct LCM_setting_table sec_slpin[] = {
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 5, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table sec_slpin_dispoff[] = {
    // Sleep Mode On
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},

    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 150, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table sec_hbmoff_1frm[] = {
	{0x53, 1, {0x20}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table sec_hbmoff_32frm[] = {
	{0x53, 1, {0x28}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table sec_hbmon_1frm[] = {
	{0x53, 1, {0xE0}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table sec_hbmon_32frm[] = {
	{0x53, 1, {0xE8}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

/*ACL Mode cabc percent*/
static struct LCM_setting_table sec_acl_per[] = {
	{0x55, 1, {0x0}},//0 off, 1 30%, 2 40%, 3 70%
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table sec_bri[] = {
	{0x51, 1, {0xff}},//0 off, 1 30%, 2 40%, 3 70%
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table te_on[] = {
	{0x35, 1, {0}},//0 off, 1 30%, 2 40%, 3 70%
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table te_off[] = {
	{0x34, 1, {0}},//0 off, 1 30%, 2 40%, 3 70%
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for (i = 0; i < count; i++) {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {
        case REGFLAG_DELAY :
			if (table[i].count == 0)
				break;
            if(table[i].count <= 10) 
                UDELAY(table[i].count * 1000);
            else
                MDELAY(table[i].count);
            break;

        case REGFLAG_END_OF_TABLE :
            break;

        default:
            dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;
		params->physical_width = 69;//69.21/68.31
		params->physical_height = 121;//121.44 122.3

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		//params->dsi.mode   = SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE;
		params->dsi.mode   = BURST_VDO_MODE;
#endif
		//params->dsi.esd_check_enable = 1;  //for esd test.

		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
		params->dsi.packet_size = 256;
		//params->dsi.clk_lp_per_line_enable = 1;
		// Video mode setting
		params->dsi.intermediat_buffer_num = 0;
		params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count = FRAME_WIDTH * 3;
		
		params->dsi.vertical_sync_active				= 3;
		params->dsi.vertical_backporch					= 6;
		params->dsi.vertical_frontporch					= 56;
		params->dsi.vertical_active_line				= FRAME_HEIGHT;
		params->dsi.vertical_frontporch_for_low_power = 400; // for saving power in screen idle

		params->dsi.horizontal_sync_active				= 4; //+ backprot = 36
		params->dsi.horizontal_backporch				= 32;
		params->dsi.horizontal_frontporch				= 64;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	    //params->dsi.LPX=8; 

		// Bit rate calculation
		params->dsi.PLL_CLOCK = 449;
		//1 Every lane speed
	//	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	//	params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4
	//	params->dsi.fbk_div =0x13;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
}
static struct LCM_setting_table sec_init_id[] = {
	{0xb9, 3,  {0xff,0x83,0x99}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0, {}},
};

static void lcm_power_switch(int enable)
{
	if (enable) {
		//because VBAT is already on when the system is startup.
		//so here we only enable VCI and VDDI power
#ifndef BUILD_LK
		//VCI
		//VDDI
		pmic_ldo_vol_sel(MT6331_POWER_LDO_VGP1, VOL_3300);
		pmic_ldo_enable(MT6331_POWER_LDO_VGP1, KAL_TRUE);

		pmic_ldo_vol_sel(MT6331_POWER_LDO_VGP3, VOL_1800);
		pmic_ldo_enable(MT6331_POWER_LDO_VGP3, KAL_TRUE);
#else
		mt6331_upmu_set_rg_vgp1_vosel(7); //3.3v
		mt6331_upmu_set_rg_vgp1_en(1);
		mt6331_upmu_set_rg_vgp3_vosel(3);
		mt6331_upmu_set_rg_vgp3_en(1);
#endif
		//SET_RESET_PIN(1);
		mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
		MDELAY(10);
		mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
		UDELAY(50);
		mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
		MDELAY(25);
	} else {
		//VDDI
#ifndef BUILD_LK
		pmic_ldo_enable(MT6331_POWER_LDO_VGP1, KAL_FALSE);
		pmic_ldo_enable(MT6331_POWER_LDO_VGP3, KAL_FALSE);
#else
		mt6331_upmu_set_rg_vgp1_en(0);
		mt6331_upmu_set_rg_vgp3_en(0);
#endif
		//SET_RESET_PIN(0);
	}
}
static void lcm_brightness_set(int brightness)
{
	unsigned int data_array[16];

	lcm_print("[LCM] set brightness %d\n", brightness);
	if (brightness > 0xff) {
		lcm_print("Set Amoled brightness error! range 0~0xff\n");
		return;
	}
	sec_bri[0].para_list[0] = brightness;
	push_table(sec_bri,
		sizeof(sec_bri)/sizeof(struct LCM_setting_table), 1);
}
static void lcm_brightness_set_cmdq(void *handle, int brightness)
{
	unsigned int data_array[16];

	lcm_print("[LCM] set brightness %d\n", brightness);
	if (brightness > 0xff) {
		lcm_print("Set Amoled brightness error! range 0~0xff\n");
		return;
	}
	unsigned int cmd = 0x51;
	unsigned int count = 1;
	unsigned int value = brightness;
	dsi_set_cmd_by_cmdq_dual(handle, cmd, count, &value, 1);
}

static void lcm_init(void)
{
	unsigned int array[16];
	unsigned char c3[4] = {0,};
	int ret = 0;
	int i = 0;
	lcm_print("[LCM] %s\n", __func__);

	lcm_power_switch(true);
#ifdef BUILD_LK
	read_reg_v2(0xDA, &c3[0], 1);
	//id_code |= (unsigned int)(id_code_dc<<16);
	read_reg_v2(0xDb, &c3[1], 1);
	//id_code |= (unsigned int)(id_code_dc<<8);
	read_reg_v2(0xDc, &c3[2], 1);
	//id_code |= id_code_dc;
	id_code = (c3[2] | (c3[1]<<8) | (c3[0] << 16));
	lcm_print("[LCM] ID CODE %x\n", id_code);
	/*
	read_reg_v2(0xB6, read_buf, 23);
	for(i = 0; i < 23; i++)
		lcm_print("B6h[%d][0x%x]\n", i, read_buf[i]);
	read_reg_v2(0x04, read_buf, 3);
	for(i = 0; i < 3; i++)
		lcm_print("04h[%d][0x%x]\n", i, read_buf[i]);
	*/
	if (c3[0] == 0x24) {
		push_table(sec_slpout,
			sizeof(sec_slpout)/sizeof(struct LCM_setting_table), 1);
	} else {
		push_table(sec_slpout_v2,
			sizeof(sec_slpout_v2)/sizeof(struct LCM_setting_table), 1);
	}
#else
	lcm_print("[LCM] ID CODE %x\n", mz_system_lcd_id);
	if (mz_system_lcd_id & 0xff0000 == 0x240000) {
		push_table(sec_slpout,
			sizeof(sec_slpout)/sizeof(struct LCM_setting_table), 1);
	} else {
		push_table(sec_slpout_v2,
			sizeof(sec_slpout_v2)/sizeof(struct LCM_setting_table), 1);
	}
#endif
	push_table(sec_dispon,
		sizeof(sec_dispon)/sizeof(struct LCM_setting_table), 1);
}
//#define LCM_POWER_ENABLE
static void lcm_suspend(void)
{
	push_table(sec_slpin_dispoff,
		sizeof(sec_slpin_dispoff)/sizeof(struct LCM_setting_table), 1);
#ifdef LCM_POWER_ENABLE
	lcm_power_switch(false);
#else
	MDELAY(60);
#endif
}

static void lcm_suspend_power(void)
{
	lcm_power_switch(false);
}
static void lcm_resume(void)
{
	lcm_print("resume @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
#ifdef LCM_POWER_ENABLE
	lcm_init();
#else
#ifndef BUILD_LK
	if (mz_system_lcd_id & 0xff0000 == 0x240000) {
		push_table(sec_slpout,
			sizeof(sec_slpout)/sizeof(struct LCM_setting_table), 1);
	} else {
		push_table(sec_slpout_v2,
			sizeof(sec_slpout_v2)/sizeof(struct LCM_setting_table), 1);
	}
#endif
	push_table(sec_dispon,
		sizeof(sec_dispon)/sizeof(struct LCM_setting_table), 1);
#endif
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif
#ifdef BUILD_LK
static unsigned int lcm_compare_id(void)
{
	return id_code;
}
#endif

static unsigned int lcm_set_te(unsigned int onoff)
{
	if (onoff) {
		push_table(te_on,
			sizeof(te_on)/sizeof(struct LCM_setting_table), 1);
	} else {
		push_table(te_off,
			sizeof(te_off)/sizeof(struct LCM_setting_table), 1);
	}
}
static unsigned int lcm_set_hbm(void *handle, unsigned int onoff)
{
	unsigned int data_array[16];

	lcm_print("[LCM] set hbm %d\n", onoff);
	unsigned int cmd = 0x53;
	unsigned int count = 1;
	unsigned int value = 0;

	if (onoff)
		value = 0xE0;
	else
		value = 0x20;
	dsi_set_cmd_by_cmdq_dual(handle, cmd, count, &value, 1);
	value = 0x0;
	dsi_set_cmd_by_cmdq_dual(handle, 0x55, 1, &value, 1);
}

LCM_DRIVER s6e3fa3_fhd_dsi_cmd_sec_lcm_drv = 
{
    .name			= "s6e3fa3_fhd_dsi_cmd_sec",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
#ifdef BUILD_LK
	.suspend_power  = lcm_suspend_power,
	.compare_id     = lcm_compare_id,
#endif
	.resume         = lcm_resume,
	.set_backlight	= lcm_brightness_set,
	.set_backlight_cmdq	= lcm_brightness_set_cmdq,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
	.set_te			= lcm_set_te,
#ifndef BUILD_LK
	.set_hbm		= lcm_set_hbm,
#endif
};
