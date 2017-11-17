#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
	#include <linux/xlog.h>
	#include <mach/mt_pm_ldo.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)

#define LCM_ID_OTM1901       (0xF5)
#define GPIO_LCD_RST_EN      (GPIO28 | 0x80000000)
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#define REGFLAG_DELAY                                           0xFC
#define REGFLAG_END_OF_TABLE                                0xFD   // END OF REGISTERS MARKER


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
static void lcm_init_power(void)
{
  mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);

#ifdef BUILD_LK
  mt6325_upmu_set_rg_vgp1_en(1);
#else
    hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_3000, "LCM_DRV");
#endif
}

static void lcm_suspend_power(void)
{
#ifdef BUILD_LK
    mt6325_upmu_set_rg_vgp1_en(0);
#else
//    lcm_init_power();
    printk("%s, begin\n", __func__);
    hwPowerDown(MT6325_POWER_LDO_VGP1, "LCM_DRV");
    printk("%s, end\n", __func__);
#endif
  mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);

  mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
}

static void lcm_resume_power(void)
{
  mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
#ifdef BUILD_LK
  mt6325_upmu_set_rg_vgp1_en(1);
#else
  printk("%s, begin\n", __func__);
  hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_3000, "LCM_DRV");
  printk("%s, end\n", __func__);
#endif
}

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};
//update initial param for IC nt35520 0.01
static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28,0,{}},
	{0x10,0,{}},
	{REGFLAG_DELAY, 120, {}}
};
	
//update initial param for IC nt35520 0.01
static struct LCM_setting_table lcm_initialization_setting[] = 
{   
    {0x00,1,{0x00}},
    {0xFF,4,{0x19,0x01,0x01,0x00}},
    {0x00,1,{0x80}},
    {0xFF,2,{0x19,0x01}},
    {0x00,1,{0x00}},
    {0x1C,1,{0x33}},//Can not OTP
    {0x00,1,{0xA0}},
    {0xC1,1,{0xE8}},
    {0x00,1,{0xA7}},
    {0xC1,1,{0x00}},
    {0x00,1,{0x90}},
    {0xC0,6,{0x00,0x2F,0x00,0x00,0x00,0x01}},
    {0x00,1,{0xC0}},
    {0xC0,6,{0x00,0x2F,0x00,0x00,0x00,0x01}},
    {0x00,1,{0x9A}},
    {0xC0,1,{0x1E}},
    {0x00,1,{0xAC}},
    {0xC0,1,{0x06}},
    {0x00,1,{0xDC}},
    {0xC0,1,{0x06}},
    {0x00,1,{0x81}},
    {0xA5,1,{0x04}},
    {0x00,1,{0x84}},
    {0xc4,1,{0x20}},
    {0x00,1,{0xa5}},
    {0xb3,1,{0x1d}},
    {0x00,1,{0x92}},
    {0xE9,1,{0x00}},
    {0x00,1,{0x90}},
    {0xF3,1,{0x01}},
    
    {0x00,1,{0xf7}},
    {0xc3,4,{0x04,0x18,0x04,0x04}},
    
    {0x00,1,{0xB4}},
    {0xC0,1,{0x80}},   //forward  80   backward D0
    
    {0x00,1,{0x93}},
    {0xC5,1,{0x19}},
    {0x00,1,{0x95}},
    {0xC5,1,{0x2D}},
    
    {0x00,1,{0x97}},
    {0xC5,1,{0x14}},
    
    {0x00,1,{0x99}},
    {0xC5,1,{0x29}},
    {0x00,1,{0x00}},
    {0xD8,2,{0x1D,0x1D}},
    
    {0x00,1,{0x00}},
    
    {0xe1,24,{0x00,0x07,0x0e,0x17,0x1f,0x26,0x32,0x44,0x4e,0x63,0x71,0x7b,0x7b,0x72,0x6c,0x5a,0x46,0x34,0x26,0x1f,0x16,0x0b,0x08,0x03}},
    {0x00,1,{0x00}},
    {0xe2,24,{0x00,0x07,0x0e,0x17,0x1f,0x26,0x32,0x44,0x4e,0x63,0x71,0x7b,0x7b,0x72,0x6c,0x5a,0x46,0x34,0x26,0x1f,0x16,0x0b,0x08,0x03}},
    {0x00,1 ,{0x00}},
    {0xe3,24,{0x00,0x07,0x0e,0x17,0x1f,0x26,0x32,0x44,0x4e,0x63,0x71,0x7b,0x7b,0x72,0x6c,0x5a,0x46,0x34,0x26,0x1f,0x16,0x0b,0x08,0x03}},
    {0x00,1, {0x00}},
    {0xe4,24,{0x00,0x07,0x0e,0x17,0x1f,0x26,0x32,0x44,0x4e,0x63,0x71,0x7b,0x7b,0x72,0x6c,0x5a,0x46,0x34,0x26,0x1f,0x16,0x0b,0x08,0x03}},
    {0x00,1, {0x00}},
    {0xe5,24,{0x00,0x07,0x0e,0x17,0x1f,0x26,0x32,0x44,0x4e,0x63,0x71,0x7b,0x7b,0x72,0x6c,0x5a,0x46,0x34,0x26,0x1f,0x16,0x0b,0x08,0x03}},
    {0x00,1, {0x00}},
    {0xe6,24,{0x00,0x07,0x0e,0x17,0x1f,0x26,0x32,0x44,0x4e,0x63,0x71,0x7b,0x7b,0x72,0x6c,0x5a,0x46,0x34,0x26,0x1f,0x16,0x0b,0x08,0x03}},
    
    {0x00,1,{0x00}},
    {0xd9,2,{0x00,0xB4}},
    
    {0x00,1,{0x81}},
    {0xA5,1,{0x07}},
    
    {0x00,1,{0x9D}},
    {0xC5,1,{0x77}},
    
    {0x00,1,{0xB3}},
    {0xC0,1,{0xCC}},
    {0x00,1,{0xBC}},
    {0xC0,1,{0x00}},
    {0x00,1,{0x80}},
    {0xC0,14,{0x00,0x87,0x00,0x0A,0x0A,0x00,0x87,0x0A,0x0A,0x00,0x87,0x00,0x0A,0x0A}},
    {0x00,1,{0xF0}},
    {0xC3,6,{0x22,0x02,0x00,0x00,0x00,0x0C}},
    {0x00,1,{0xA0}},
    {0xC0,7,{0x00,0x00,0x00,0x00,0x03,0x22,0x03}},
    {0x00,1,{0xD0}},
    {0xC0,7,{0x00,0x00,0x00,0x00,0x03,0x22,0x03}},
    {0x00,1,{0x90}},
    {0xC2,8,{0x83,0x01,0x00,0x00,0x82,0x01,0x00,0x00}},
    {0x00,1,{0x80}},
    {0xC3,12,{0x82,0x02,0x03,0x00,0x03,0x84,0x81,0x03,0x03,0x00,0x03,0x84}},
    {0x00,1,{0x90}},
    {0xC3,12,{0x00,0x01,0x03,0x00,0x03,0x84,0x01,0x02,0x03,0x00,0x03,0x84}},
    {0x00,1,{0x80}},
    {0xCC,15,{0x09,0x0A,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x28,0x28,0x28,0x28,0x28}},
    {0x00,1,{0x90}},
    {0xCC,15,{0x0A,0x09,0x14,0x13,0x12,0x11,0x15,0x16,0x17,0x18,0x28,0x28,0x28,0x28,0x28}},
    {0x00,1,{0xA0}},
    {0xCC,15,{0x1D,0x1E,0x1F,0x19,0x1A,0x1B,0x1C,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27}},
    {0x00,1,{0xB0}},
    {0xCC,8,{0x01,0x02,0x03,0x05,0x06,0x07,0x04,0x08}},
    {0x00,1,{0xC0}},
    {0xCC,12,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x77}},
    {0x00,1,{0xD0}},
    {0xCC,12,{0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x77}},
    {0x00,1,{0x80}},
    {0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0x00,1,{0x90}},
    {0xCB,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0x00,1,{0xA0}},
    {0xCB,15,{0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0x00,1,{0xB0}},
    {0xCB,15,{0x00,0x01,0xFD,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0x00,1,{0xC0}},
    {0xCB,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x77,0x77}},
    {0x00,1,{0xD0}},
    {0xCB,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x77,0x77}},
    {0x00,1,{0xE0}},
    {0xCB,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x77,0x77}},
    {0x00,1,{0xF0}},
    {0xCB,8,{0x01,0x01,0x01,0x00,0x00,0x00,0x77,0x77}},
    {0x00,1,{0x80}},
    {0xCD,15,{0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x02,0x12,0x11,0x3F,0x04,0x3F}},
    {0x00,1,{0x90}},
    {0xCD,11,{0x06,0x3c,0x3F,0x26,0x26,0x26,0x21,0x20,0x1F,0x26,0x26}},
    {0x00,1,{0xA0}},
    {0xCD,15,{0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x01,0x12,0x11,0x3F,0x03,0x3F}},
    {0x00,1,{0xB0}},
    {0xCD,11,{0x05,0x3c,0x3F,0x26,0x26,0x26,0x21,0x20,0x1F,0x26,0x26}},
    
    {0x00,1,{0x9B}},
    {0xC5,2,{0x55,0x55}},
    
    {0x00,1,{0x80}},
    {0xC4,1,{0x15}},
    
    {0x00,1,{0x00}},
    {0xFF,3,{0xFF,0xFF,0xFF}},

	{0x11,0,{0}},	//	Return	To	CMD1		
	{REGFLAG_DELAY, 150, {}},												
	{0x29,0,{}},
    {REGFLAG_DELAY, 50, {}},		
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                if(table[i].count <= 10)
                    MDELAY(table[i].count);
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
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{

		memset(params, 0, sizeof(LCM_PARAMS));

		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = BURST_VDO_MODE;
        #endif

		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting
		params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
//		params->dsi.word_count=720*3;


		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 2;///===6
		params->dsi.vertical_frontporch					= 10;///===18
		params->dsi.vertical_active_line				= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active				= 20;
		params->dsi.horizontal_backporch				= 90;
		params->dsi.horizontal_frontporch				= 70;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
		//1 Every lane speed
/*
		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4
		params->dsi.fbk_div =15;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
*/
     params->dsi.PLL_CLOCK =450;

}

static void lcm_init(void)
{
#if 1
    mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
    MDELAY(10);
    mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
    MDELAY(10);
    mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
    MDELAY(120);
#else
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(120);
#endif
push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1); 
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];

    mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
    MDELAY(50);
    mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
    MDELAY(50);

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
}


static void lcm_resume(void)
{
	lcm_init();
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

	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif
#if 1
static unsigned int lcm_compare_id(void)
{
	unsigned int id=0;
	unsigned int id0, id1, id2, id3;
	unsigned char buffer[4];
	unsigned int array[16];

#if 1
    mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
    MDELAY(10);
    mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
    MDELAY(10);
    mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
    MDELAY(120);
#else
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(120);
#endif
	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xF4, buffer, 4);
	id 	= buffer[0]; //we only need ID
	id0 = buffer[0];
	id1 = buffer[1];
	id2 = buffer[2];
	id3 = buffer[3];	
#ifdef BUILD_LK
	printf("%s, LK otm1901 debug: otm1901 id = 0x%08x, id0 = 0x%08x, id1 = 0x%08x, id2 = 0x%08x, id3 = 0x%08x\n", __func__, id, id0, id1, id2, id3);
#else
	printk("%s, kernel otm1901 horse debug: otm1901 id = 0x%08x, id0 = 0x%08x, id1 = 0x%08x, id2 = 0x%08x, id3 = 0x%08x\n", __func__, id, id0, id1, id2, id3);
#endif

    if(id == LCM_ID_OTM1901)
    	return 1;
    else
        return 0;
}
#endif

LCM_DRIVER otm1901_boe52_xyl_fhd_lcm_drv=
{
    .name			= "otm1901_boe52_xyl_fhd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    .init_power        = lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,

    };
