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

#define LCM_ID_RM63417          0x3417
#define GPIO_LCD_RST_EN      (GPIO28 | 0x80000000)
#define GPIO_LCD_POWER_EN      (GPIO144 | 0x80000000)
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
    mt_set_gpio_mode(GPIO_LCD_POWER_EN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCD_POWER_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCD_POWER_EN, GPIO_OUT_ZERO);
    MDELAY(10);
    mt_set_gpio_out(GPIO_LCD_POWER_EN, GPIO_OUT_ONE);
    MDELAY(20);
}


static void lcm_suspend_power(void)
{
    mt_set_gpio_out(GPIO_LCD_POWER_EN, GPIO_OUT_ZERO);
}

static void lcm_resume_power(void)
{   
    mt_set_gpio_out(GPIO_LCD_POWER_EN, GPIO_OUT_ONE);
}

static void lcd_power_en(unsigned char enabled)
{
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

static void init_lcm_registers(void)
{
unsigned int data_array[16];
data_array[0] = 0x00022902;
data_array[1] = 0x000004b0;
dsi_set_cmdq(data_array, 2, 1);
data_array[0] = 0x00000500;
dsi_set_cmdq(data_array, 1, 1);
data_array[0] = 0x00000500;
dsi_set_cmdq(data_array, 1, 1);
data_array[0] = 0x00072902;
data_array[1] = 0x000014b3;
data_array[2] = 0x00000000;
dsi_set_cmdq(data_array, 3, 1);
data_array[0] = 0x00032902;
data_array[1] = 0x00c33ab6;
dsi_set_cmdq(data_array, 2, 1);
data_array[0] = 0x00232902;
data_array[1] = 0x106084c1;
data_array[2] = 0xce6fffeb;
data_array[3] = 0x1217ffff;
data_array[4] = 0x31ae7358;
data_array[5] = 0xffffc620;
data_array[6] = 0x5ffff31f;
data_array[7] = 0x10101010;
data_array[8] = 0x22010200;
data_array[9] = 0x00010022;
dsi_set_cmdq(data_array, 10, 1);
data_array[0] = 0x00082902;
data_array[1] = 0x80f731c2;
data_array[2] = 0x00000a08;
dsi_set_cmdq(data_array, 3, 1);
data_array[0] = 0x00042902;
data_array[1] = 0x000001c3;
dsi_set_cmdq(data_array, 2, 1);
data_array[0] = 0x00172902;
data_array[1] = 0x000070c4;
data_array[2] = 0x00040000;
data_array[3] = 0x060c0000;
data_array[4] = 0x00000000;
data_array[5] = 0x33000033;
data_array[6] = 0x00060c00;
dsi_set_cmdq(data_array, 7, 1);
data_array[0] = 0x00292902;
data_array[1] = 0x006977c6;
data_array[2] = 0x00690069;
data_array[3] = 0x00000000;
data_array[4] = 0x00690069;
data_array[5] = 0x07191069;
data_array[6] = 0x69007700;
data_array[7] = 0x69006900;
data_array[8] = 0x00000000;
data_array[9] = 0x69006900;
data_array[10] = 0x19106900;
data_array[11] = 0x00000007;
dsi_set_cmdq(data_array, 12, 1);
data_array[0] = 0x000a2902;
data_array[1] = 0x3ffc31cb;
data_array[2] = 0x0000008c;
data_array[3] = 0x0000c000;
dsi_set_cmdq(data_array, 4, 1);
data_array[0] = 0x001f2902;
data_array[1] = 0x0e0a01c7;
data_array[2] = 0x473b2d1e;
data_array[3] = 0x4d433b58;
data_array[4] = 0x7f796560;
data_array[5] = 0x1e0e0a01;
data_array[6] = 0x58473b2d;
data_array[7] = 0x604d433b;
data_array[8] = 0x007f7965;
dsi_set_cmdq(data_array, 9, 1);
data_array[0] = 0x00142902;
data_array[1] = 0x000000c8;
data_array[2] = 0x00fc0000;
data_array[3] = 0x00000000;
data_array[4] = 0x000000fc;
data_array[5] = 0x00fc0000;
dsi_set_cmdq(data_array, 6, 1);
data_array[0] = 0x00022902;
data_array[1] = 0x00000bcc;
dsi_set_cmdq(data_array, 2, 1);
data_array[0] = 0x000b2902;
data_array[1] = 0xbb8144d0;
data_array[2] = 0x194c5959;
data_array[3] = 0x00000419;
dsi_set_cmdq(data_array, 4, 1);
data_array[0] = 0x001a2902;
data_array[1] = 0xbb331bd3;
data_array[2] = 0x3333b3bb;
data_array[3] = 0x00010033;
data_array[4] = 0x00a0d8a0;
data_array[5] = 0x3b335757;
data_array[6] = 0x3d077237;
data_array[7] = 0x000077bf;
dsi_set_cmdq(data_array, 8, 1);
data_array[0] = 0x00082902;
data_array[1] = 0x000006d5;
data_array[2] = 0x40014001;
dsi_set_cmdq(data_array, 3, 1);
data_array[0] = 0x00082902;
data_array[1] = 0x000006d5;
data_array[2] = 0x40014001;
dsi_set_cmdq(data_array, 3, 1);
data_array[0] = 0x00022902;
data_array[1] = 0x000001d6;
dsi_set_cmdq(data_array, 2, 1);
data_array[0] = 0x00062902;
data_array[1] = 0xff3f00de;
data_array[2] = 0x00000010;
dsi_set_cmdq(data_array, 3, 1);
data_array[0] = 0x00023902;
data_array[1] = 0x00000035;
dsi_set_cmdq(data_array, 2, 1);
data_array[0] = 0x00290500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(150);
data_array[0] = 0x00110500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(200);
}

//update initial param for IC nt35520 0.01
static struct LCM_setting_table lcm_initialization_setting[] = 
{   
                   {0xb0,1,{0x04}},
      {0xb3, 6 , {0x14,0x00,0x00,0x00,0x00,0x00}},
      {0xb6, 2 , {0x3A,0xC3}},
      {0xc1, 34 , {0x84,0x60,0x10,0xEB,0xFF,0x6F,0xCE,0xFF,0xFF,0x17,0x12,0x58,0x73,0xAE,0x31,
               0x20,0xC6,0xFF,0xFF,0x1F,0xF3,0xFF,0x5F,0x10,0x10,0x10,0x10,0x00,0x02,0x01,0x22,0x22,0x00,0x01}},

      {0xc2,  7 , {0x31,0xF7,0x80,0x08,0x0A,0x00,0x00}},
      {0xc3,  3 , {0x01,0x00,0x00}},

      {0xc4,  22 , {0x70,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x0C,0x06,0x00,0x00,0x00,0x00,
               0x33,0x00,0x00,0x33,0x00,0x0C,0x06}},

      {0xc6,  40 ,{0x77,0x69,0x00,0x69,0x00,0x69,0x00,0x00,0x00,0x00,0x00,0x69,0x00,0x69,0x00,0x69,
          0x10,0x19,0x07,0x00,0x77,0x00,0x69,0x00,0x69,0x00,0x69,0x00,0x00,0x00,0x00,0x00,0x69,0x00,0x69,0x00,0x69,
          0x10,0x19,0x07}},

      {0xcb,  9 , {0x31,0xFC,0x3F,0x8C,0x00,0x00,0x00,0x00,0xC0}},

        {0xc7,  30 ,{0x01,0x0A,0x0E,0x1E,0x2D,0x3B,0x47,0x58,0x3B,0x43,0x4D,0x60,0x65,0x79,0x7F,
        0x01,0x0A,0x0E,0x1E,0x2D,0x3B,0x47,0x58,0x3B,0x43,0x4D,0x60,0x65,0x79,0x7F}},

      {0xc8,  19 , {0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,0x00,0xFC,0x00,0x00,0x00,
          0x00,0x00,0xFC,0x00}},


      {0xcc, 1 , {0x0B}},
      {0xd0, 10 , {0x44,0x81,0xBB,0x59,0x59,0x4C,0x19,0x19,0x04,0x00}},

      {0xd3, 25 , {0x1B,0x33,0xBB,0xBB,0xB3,0x33,0x33,0x33,0x00,0x01,0x00,0xA0,0xD8,0xA0,0x00,
          0x57,0x57,0x33,0x3B,0x37,0x72,0x07,0x3D,0xBF,0x77}},

      {0xd5, 7 , {0x06,0x00,0x00,0x01,0x40,0x01,0x40}},
      {0xd5, 7 , {0x06,0x00,0x00,0x01,0x40,0x01,0x40}},
      {0xd6, 1 , {0x01}},
      {0xde, 5 , {0x00,0x3F,0xFF,0x10,0x00}},
      {0x35, 1 , {0x00}},
 
        
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	{0x11,0,{0}},	//	Return	To	CMD1		
	
	{REGFLAG_DELAY, 150, {}},												
	{0x29,0,{}},
  {REGFLAG_DELAY, 50, {}},		
	//{0x51,1,{0xFF}},	//	write	display	brightness		
    //candle
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
		params->dsi.vertical_backporch					= 8;///===6
		params->dsi.vertical_frontporch					= 10;///===18
		params->dsi.vertical_active_line				= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active				= 10;
		params->dsi.horizontal_backporch				= 90;
		params->dsi.horizontal_frontporch				= 120;
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
//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1); 
    init_lcm_registers();
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
  unsigned char buffer[4];
  unsigned int array[16];

//    lcd_power_en(0);
//    lcd_power_en(1);

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


  array[0] = 0x00022902;
  array[1] = 0x000000b0;
  dsi_set_cmdq(array, 2, 1);

  array[0] = 0x00043700;// read id return two byte,version and id
  dsi_set_cmdq(array, 1, 1);

  read_reg_v2(0xbf, buffer, 4);
  id = buffer[2]<<8 | buffer[3]; //we only need ID

#ifdef BUILD_LK
    printf("%s id= x%08x,id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x,id3 = 0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2],buffer[3]);
#else
    printk("%s id= x%08x,id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x,id3 = 0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2],buffer[3]);
#endif

    return (0x3417 == id) ? 1 : 0;

}
#endif

LCM_DRIVER r63417_lg52_truly_fhd_lcm_drv =
{
    .name			= "r63417_lg52_truly_fhd",
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
