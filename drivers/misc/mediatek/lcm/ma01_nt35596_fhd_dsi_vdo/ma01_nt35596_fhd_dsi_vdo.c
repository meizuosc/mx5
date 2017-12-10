#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
	#include <platform/upmu_common.h>
#include <platform/ddp_dsi.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <mach/mt_pm_ldo.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)
#define REGFLAG_DELAY 0xffe
#define REGFLAG_END_OF_TABLE 0xfff

#define LCM_ID_NT35590 (0x90)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#ifdef BUILD_LK
#define lcm_print(str, args...) printf(str, ##args);
#else
#define lcm_print(str, args...) printk(str, ##args);
#endif


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   

#define dsi_lcm_set_gpio_out(pin, out)										lcm_util.set_gpio_out(pin, out)
#define dsi_lcm_set_gpio_mode(pin, mode)									lcm_util.set_gpio_mode(pin, mode)
#define dsi_lcm_set_gpio_dir(pin, dir)										lcm_util.set_gpio_dir(pin, dir)
#define dsi_lcm_set_gpio_pull_enable(pin, en)								lcm_util.set_gpio_pull_enable)(pin, en)

#define   LCM_DSI_CMD_MODE							0
#define AUO_CE


struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};
static struct LCM_setting_table auo_ce[] = {
	{0xFF,1, {0xEE}}, 
	{0xFB,1, {0x01}},   
	{0x7C,1, {0x31}},   
	{0xFF,1, {0x01}},   
	{0xFB,1, {0x01}},   
	{0x00,1, {0x01}},   
	{0x01,1, {0x55}},   
	{0x02,1, {0x40}},   
	{0x05,1, {0x50}},   
	{0x06,1, {0x4A}},   
	{0x07,1, {0x29}},   
	{0x08,1, {0x0C}},   
	{0x0B,1, {0x87}},   
	{0x0C,1, {0x87}},   
	{0x0E,1, {0xB0}},   
	{0x0F,1, {0xB3}},   
	{0x14,1, {0x4A}},
	{0x15,1, {0x15}},
	{0x16,1, {0x15}},
	{0x18,1, {0x00}},
	{0x19,1, {0x77}},
	{0x1A,1, {0x55}},
	{0x1B,1, {0x13}},
	{0x1C,1, {0x00}},
	{0x1D,1, {0x00}},
	{0x1E,1, {0x00}},
	{0x1F,1, {0x00}},//red looks like old NVT TP Noise Setting
	{0x58,1, {0x82}},
	{0x59,1, {0x02}},
	{0x5A,1, {0x02}},
	{0x5B,1, {0x02}},
	{0x5C,1, {0x82}},
	{0x5D,1, {0x82}},
	{0x5E,1, {0x02}},
	{0x5F,1, {0x02}},
	{0x60,1, {0x0F}},
	{0x66,1, {0x00}},
	{0x72,1, {0x31}},
	{0xFF,1, {0x05}},
	{0xFB,1, {0x01}},
	{0x00,1, {0x01}},
	{0x01,1, {0x0B}},
	{0x02,1, {0x0C}},
	{0x03,1, {0x09}},
	{0x04,1, {0x0A}},
	{0x05,1, {0x1A}},
	{0x06,1, {0x10}},
	{0x07,1, {0x00}},
	{0x08,1, {0x1A}},
	{0x09,1, {0x00}},
	{0x0A,1, {0x00}},
	{0x0B,1, {0x00}},
	{0x0C,1, {0x00}},
	{0x0D,1, {0x13}},
	{0x0E,1, {0x15}},
	{0x0F,1, {0x17}},
	{0x10,1, {0x01}},
	{0x11,1, {0x0B}},
	{0x12,1, {0x0C}},
	{0x13,1, {0x09}},
	{0x14,1, {0x0A}},
	{0x15,1, {0x1A}},
	{0x16,1, {0x10}},
	{0x17,1, {0x10}},
	{0x18,1, {0x1A}},
	{0x19,1, {0x00}},
	{0x1A,1, {0x00}},
	{0x1B,1, {0x00}},
	{0x1C,1, {0x00}},
	{0x1D,1, {0x13}},
	{0x1E,1, {0x15}},
	{0x1F,1, {0x17}},
	{0x20,1, {0x00}},
	{0x21,1, {0x01}},
	{0x22,1, {0x00}},
	{0x23,1, {0x36}},
	{0x24,1, {0x36}},
	{0x25,1, {0x6D}},
	{0x29,1, {0xD8}},
	{0x2A,1, {0x2A}},
	{0x2B,1, {0x00}},
	{0xB6,1, {0x89}},
	{0xB7,1, {0x14}},
	{0xB8,1, {0x05}},
	{0x4B,1, {0x04}},
	{0x4C,1, {0x11}},
	{0x4D,1, {0x00}},
	{0x4E,1, {0x00}},
	{0x4F,1, {0x11}},
	{0x50,1, {0x11}},
	{0x51,1, {0x00}},
	{0x52,1, {0x00}},
	{0x53,1, {0x06}},
	{0x54,1, {0x75}},
	{0x55,1, {0x6D}},
	{0x5B,1, {0x44}},
	{0x5C,1, {0x00}},
	{0x5F,1, {0x74}},
	{0x60,1, {0x75}},
	{0x63,1, {0xFF}},
	{0x64,1, {0x00}},
	{0x67,1, {0x04}},
	{0x68,1, {0x04}},
	{0x6C,1, {0x00}},
	{0x7A,1, {0x80}},
	{0x7B,1, {0x91}},
	{0x7C,1, {0xD8}},
	{0x7D,1, {0x60}},
	{0x7E,1, {0x02}},
	{0x7F,1, {0x14}},
	{0x81,1, {0x05}},
	{0x82,1, {0x03}},
	{0x80,1, {0x00}},
	{0x83,1, {0x00}},
	{0x93,1, {0x06}},
	{0x94,1, {0x30}},
	{0x84,1, {0x05}},
	{0x85,1, {0x05}},
	{0x8A,1, {0x33}},
	{0x9B,1, {0x0F}},
	{0xA4,1, {0x0F}},
	{0xE7,1, {0x80}},
	{0xFF,1, {0x03}},
	//{REGFLAG_DELAY, 1000, {}},
	{0xFB,1, {0x01}},
	//VIVID_C2
	{0xFF,1, {0x03}},
	{REGFLAG_DELAY, 1, {}},
	{0xFB,1, {0x01}},
	{0x00,1,{0x00}},
	{0x01,1,{0x04}},
	{0x02,1,{0x08}},
	{0x03,1,{0x0C}},
	{0x04,1,{0x10}},
	{0x05,1,{0x14}},
	{0x06,1,{0x18}},
	{0x07,1,{0x20}},
	{0x08,1,{0x24}},
	{0x09,1,{0x28}},
	{0x0A,1,{0x30}},
	{0x0B,1,{0x38}},
	{0x0C,1,{0x38}},
	{0x0D,1,{0x30}},
	{0x0E,1,{0x28}},
	{0x0F,1,{0x20}},
	{0x10,1,{0x10}},
	{0x11,1,{0x00}},
	{0x12,1,{0x00}},
	{0x13,1,{0x00}},
	{0x1B,1,{0x19}},
	{0x1C,1,{0x1C}},
	{0x1D,1,{0x21}},
	{0x1E,1,{0x22}},
	{0x1F,1,{0x23}},
	{0x20,1,{0x23}},
	{0x21,1,{0x26}},
	{0x22,1,{0x28}},
	{0x23,1,{0x28}},
	{0x24,1,{0x28}},
	{0x25,1,{0x28}},
	{0x26,1,{0x1E}},
	{0x27,1,{0x14}},
	{0x28,1,{0x0A}},
	{0x29,1,{0x0A}},
	{0x2A,1,{0x08}},
	{0x2B,1,{0x08}},
	{0x2F,1,{0x08}},
	{0x30,1,{0x08}},
	{0x31,1,{0x08}},
	{0x32,1,{0x82}},
	{0x33,1,{0x84}},
	{0x34,1,{0x86}},
	{0x35,1,{0x08}},
	{0x36,1,{0x0A}},
	{0x37,1,{0x0C}},
	{0x38,1,{0x0E}},
	{0x39,1,{0x10}},
	{0x3A,1,{0x12}},
	{0x3B,1,{0x14}},
	{0x3F,1,{0x16}},
	{0x40,1,{0x18}},
	{0x41,1,{0x1A}},
	{0x42,1,{0x18}},
	{0x43,1,{0x16}},
	{0x44,1,{0x14}},
	{0x45,1,{0x10}},
	{0x46,1,{0x0E}},
	{0x47,1,{0x0C}},
	{0x48,1,{0x0A}},
	{0x49,1,{0x09}},
	{0x4A,1,{0x08}},
	{0x4B,1,{0x86}},
	{0x4C,1,{0x84}},
	{0x4D,1,{0x04}},
	{0x4E,1,{0x0A}},
	{0x4F,1,{0x0F}},
	{0x1A,1,{0x00}},
	{0x53,1,{0x07}},
	{0x54,1,{0x00}},
	{0x55,1,{0x77}},
	{0x5B,1,{0x70}},
	{0x58,1,{0xEF}},
	{0x5F,1,{0x24}},
	{0x63,1,{0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table init_code[] = {
	//esd 
	{0XFF, 1, {0XEE}},
	{0XFB, 1, {0X01}},
	{0x18, 1, {0x40}},
	{REGFLAG_DELAY, 10, {}},
	{0x18, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},
#if 1
	//tp for improve
	{0XFF, 1, {0X05}},
	{0XFB, 1, {0X01}},
	{0x4D, 1, {0x00}},
	{0x4E, 1, {0x00}},
	{0x4F, 1, {0x11}},
	{0x50, 1, {0x11}},

	{0X53, 1, {0X06}},
	{0X54, 1, {0X75}},
	{0X7E, 1, {0X02}},
	{0X7F, 1, {0X14}},

	{0X81, 1, {0X05}},
	{0X84, 1, {0X05}},
	{0X85, 1, {0X05}},

	{0XFF, 1, {0X01}},
	{0XFB, 1, {0X01}},
	{0x0E, 1, {0xA5}},
	{0x0F, 1, {0xA8}},
	{0x15, 1, {0x12}},
	{0x16, 1, {0x12}},

	{0x58, 1, {0x82}},
	{0x59, 1, {0x02}},
	{0x5A, 1, {0x02}},
	{0x5B, 1, {0x02}},
	{0x5C, 1, {0x82}},
	{0x5D, 1, {0x82}},
	{0x5E, 1, {0x02}},
	{0x5F, 1, {0x02}},
	{0X60, 1, {0X0F}},
	{0X66, 1, {0X00}},
#endif
	//normal 
	{0XFF, 1, {0XEE}},
	{0x7C, 1, {0x31}},
	{0XFB, 1, {0X01}},
	{0XFF, 1, {0X00}},
	{0x55, 1, {0xB0}},

	{0XD3, 1, {0X08}},
	{0XD4, 1, {0X06}},
#ifdef ESD_SUPPORT
	{0X35, 1, {0X00}},
#endif
	{0X51, 1, {0X80}},// do not use pwm
	{0X53, 1, {0X00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table slp_out[] = {
	// Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table display_on[] = {
	// Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 40, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};



static struct LCM_setting_table slp_in[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 0, {}},
	// Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 100, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY :
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

static void init_lcm_registers(void)
{
#ifdef AUO_CE
	push_table(auo_ce, 
			sizeof(auo_ce)/sizeof(struct LCM_setting_table), 1);
#endif
	push_table(init_code, 
			sizeof(init_code)/sizeof(struct LCM_setting_table), 1);
	push_table(slp_out, 
			sizeof(slp_out)/sizeof(struct LCM_setting_table), 1);
	push_table(display_on, 
			sizeof(display_on)/sizeof(struct LCM_setting_table), 1);
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

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

//	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
//	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;
	params->lcm_if = LCM_INTERFACE_DSI0;
	params->lcm_cmd_if = LCM_INTERFACE_DSI0;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = 66;
	params->physical_height = 110;
	// enable tearing-free
	params->dbi.te_mode = LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
#ifdef ESD_SUPPORT
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	//esd check
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

#endif
	// DSI
#if LCM_DSI_CMD_MODE 
	params->dsi.mode   = CMD_MODE;//BURST_VDO_MODE;
#else
	params->dsi.mode   = BURST_VDO_MODE;
#endif
	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size = 256;

	// Video mode setting		
	params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count = FRAME_WIDTH * 3;

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 6;
	params->dsi.vertical_frontporch	= 6;
	params->dsi.vertical_active_line = FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active = 6;
	params->dsi.horizontal_backporch = 15;
	params->dsi.horizontal_frontporch = 90;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	// Bit rate calculation
	//1 Every lane speed
	//params->dsi.pll_div1 = 0;// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	//params->dsi.pll_div2 = 0;// div2=0,1,2,3;div1_real=1,2,4,4	
	//params->dsi.fbk_div = 0x12;// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
	//or else set PLL_clk to trigger disp_drv calculate the above div automatically.
	params->dsi.PLL_CLOCK = 460;

}

static void lcm_power_on(bool value)
{
	lcm_print(" %s %s\n", __func__, value ? "on":"false");
	if (value) {
		//reset --> L
		//lcm_reset(0);
		SET_RESET_PIN(0);
		//MDELAY(1);
		//VSP5V
		mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
		//VSN5V
		mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
		MDELAY(10);
		//reset ---> H
		//lcm_reset(1);
		SET_RESET_PIN(1);
		MDELAY(5);
		SET_RESET_PIN(0); 
		MDELAY(5);
		SET_RESET_PIN(1);
		MDELAY(5);
		SET_RESET_PIN(0); 
		MDELAY(5);
		SET_RESET_PIN(1);
		MDELAY(20);
	} else {
		//VSN5V
		mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
		//VSP5V
		mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
		MDELAY(10);
		//reset ---> H
		//lcm_reset(0);
		SET_RESET_PIN(0);
	}
	lcm_print(" %s %s\n", __func__, value ? "on":"false");
}
static unsigned int lcm_compare_id(void)
{
	int lcd_id = 0xff;

	lcd_id = mt_get_gpio_in(GPIO_LCD_ID1);
	lcd_id = (lcd_id << 1) | mt_get_gpio_in(GPIO_LCD_ID2);

	if (lcd_id != 0x3)
		return 0xff;

	return lcd_id;
}
static void lcm_init(void)
{
	lcm_print(" %s %d\n", __func__, __LINE__);
	lcm_power_on(true);	
	init_lcm_registers();
	lcm_print(" %s %d\n", __func__, __LINE__);
}

static void lcm_suspend(void)
{
	push_table(slp_in,
		sizeof(slp_in)/sizeof(struct LCM_setting_table), 1);
	lcm_power_on(false);
}

static void lcm_resume(void)
{
	lcm_print(" %s %d\n", __func__, __LINE__);
	lcm_power_on(true);	
	init_lcm_registers();
	lcm_print(" %s %d\n", __func__, __LINE__);
}

#ifdef ESD_SUPPORT

static unsigned int lcm_esd_check(void)
{
    unsigned int result = TRUE;
    unsigned int data_array[16];
    unsigned char buffer[16] = {0};

    data_array[0] = 0x00013700;
    dsi_set_cmdq(data_array, 1, 1);

    read_reg_v2(0x0A, buffer, 1);
    if (buffer[0] == 0x9C)
        result = FALSE;

    return result;
}

static unsigned int lcm_esd_recover(void)
{
    lcm_init();

    return TRUE;
}

#endif

LCM_DRIVER ma01_nt35596_fhd_dsi_vdo = 
{
	.name			= "ma01_nt35596_fhd_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id  	= lcm_compare_id,
#if LCM_DSI_CMD_MODE 
	.update 		= lcm_update,
#endif
#ifdef ESD_SUPPORT
    .esd_check      = lcm_esd_check,
    .esd_recover    = lcm_esd_recover,
#endif

};
