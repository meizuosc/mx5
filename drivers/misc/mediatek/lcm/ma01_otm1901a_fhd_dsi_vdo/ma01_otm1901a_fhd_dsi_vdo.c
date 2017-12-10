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
#define dsi_lcm_set_gpio_pull_enable(pin, en)								lcm_util.set_gpio_pull_enable(pin, en)

#define   LCM_DSI_CMD_MODE							0

struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table init_code[] = {
	{0x00, 1, {0x00}},
	{0xff, 3, {0x19, 0x01, 0x01}},
	{0x00, 1, {0x80}},
	{0xff, 2, {0x19, 0x01}},
	{REGFLAG_DELAY, 10, {}},
	{0x00, 1, {0x00}},
	{0x1c, 1, {0x33}},
	{0x00, 1, {0xa0}},
	{0xC1, 1, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table slp_out[] = {
	{0x00, 1, {0x00}},
	{0xff, 3, {0x19, 0x01, 0x01}},
	{0x00, 1, {0x80}},
	{0xff, 2, {0x19, 0x01}},
	{REGFLAG_DELAY, 10, {}},
	{0x00, 1, {0x00}},
	{0x1c, 1, {0x33}},
	{0x00, 1, {0xa0}},
	{0xC1, 1, {0x00}},
//pixel column inversion setting
#if 1
    {0x00, 1, {0xB3}},
    {0xC0, 1, {0x00}},
    {0x00, 1, {0xBC}},
    {0xC0, 1, {0xCC}},

	{0x00, 1, {0xF2}},
	{0xC1, 3, {0x80, 0x02, 0x02}},
///////////////////////////////////////////////////////////////////////////////////////////////
	//Add CE code 
	{0x00, 1 , {0xA0}}, 
	{0xD6, 12 ,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}}, 

	{0x00, 1 , {0xB0}}, 
	{0xD6, 12 ,{0x26,0x26,0x13,0x9A,0x1A,0x1A,0x1A,0x80,0x33,0x33,0x33,0x33}}, 

	{0x00, 1 , {0xC0}}, 
	{0xD6, 12 ,{0x61,0x1A,0x00,0x66,0x11,0x11,0x11,0x55,0x22,0x22,0x22,0x22}}, 

	{0x00, 1 , {0xD0}}, 
	{0xD6, 12 ,{0x54,0x54,0x00,0x33,0x09,0x09,0x09,0x2B,0x11,0x11,0x11,0x11}}, 

	{0x00, 1 , {0x00}}, 
	{0x81, 1 ,{0x83}}, 
///////////////////////////////////////////////////////////////////////////////////////////////
	{0x00, 1 , {0xA2}},
	{0xC1, 1 ,{0x30}}, //Timeout off 
	{0x00, 1, {0x00}},
	{0xff, 3, {0xff, 0xff, 0xff}},
#endif
#ifdef ESD_SUPPORT
	{0x35, 1, {0x00}},
#endif
	// Sleep Out
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table display_on[] = {
	// Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 5, {}},
	{0x00, 1, {0x00}},
	{0x99, 2, {0x11,0x11}}, // CMD1 lock
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table slp_in[] = {
	{0x00, 1, {0x00}},
	{0x99, 2, {0x95,0x27}}, // CMD1 unlock
        // All pixel off
	{0x22, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},
        // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 80, {}},

	// Display off sequence
	{0x28, 1, {0x00}},
	//{REGFLAG_DELAY, 10, {}},
	{0x00, 1, {0x00}},
	{0x99, 2, {0x11,0x11}}, // CMD1 lock
	//{REGFLAG_DELAY, 10, {}},

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
	push_table(slp_out, 
			sizeof(slp_out)/sizeof(struct LCM_setting_table), 1);
//	push_table(init_code, 
//			sizeof(init_code)/sizeof(struct LCM_setting_table), 1);
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
	params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;// LCM_DBI_TE_MODE_VSYNC_ONLY;
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
	params->dsi.vertical_backporch = 8;
	params->dsi.vertical_frontporch	= 8;
	params->dsi.vertical_active_line = FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active = 20;
	params->dsi.horizontal_backporch = 40;
	params->dsi.horizontal_frontporch = 60;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	// Bit rate calculation
	//1 Every lane speed
	//params->dsi.pll_div1 = 0;// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
	//params->dsi.pll_div2 = 0;// div2=0,1,2,3;div1_real=1,2,4,4	
	//params->dsi.fbk_div = 0x12;// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
	//or else set PLL_clk to trigger disp_drv calculate the above div automatically.
	params->dsi.PLL_CLOCK = 465;
}

static void lcm_power_on(bool value)
{
	lcm_print(" %s %s\n", __func__, value ? "on":"false");
	if (value) {
		//reset --> L
		SET_RESET_PIN(0);
		MDELAY(1);
		//VSP5V
		mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
		//VSN5V
		MDELAY(5);
		mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ONE);
		MDELAY(5);
		SET_RESET_PIN(1);
		MDELAY(5);
		SET_RESET_PIN(0);
		MDELAY(10);
		SET_RESET_PIN(1);
		MDELAY(10);
	} else {
		SET_RESET_PIN(0);
		MDELAY(10);
		//VSN5V
		mt_set_gpio_out(GPIO_LCD_BIAS_ENN_PIN, GPIO_OUT_ZERO);
		MDELAY(5);
		mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
		//VSP5V
		MDELAY(1);
	}
}
static unsigned int lcm_compare_id(void)
{
	int lcd_id = 0xff;

	lcd_id = mt_get_gpio_in(GPIO_LCD_ID1);
	lcd_id = (lcd_id << 1) | mt_get_gpio_in(GPIO_LCD_ID2);

	if (lcd_id != 0)
		return 0xff;

	return lcd_id;
}
static void lcm_init(void)
{
	lcm_power_on(true);	
	init_lcm_registers();
}

static void lcm_suspend(void)
{
	push_table(slp_in,
		sizeof(slp_in)/sizeof(struct LCM_setting_table), 1);
	lcm_power_on(false);
}

static void lcm_resume(void)
{
	lcm_power_on(true);	
	init_lcm_registers();
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

LCM_DRIVER ma01_otm1901a_fhd_dsi_vdo = 
{
	.name			= "ma01_otm1901a_fhd_dsi_vdo",
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
