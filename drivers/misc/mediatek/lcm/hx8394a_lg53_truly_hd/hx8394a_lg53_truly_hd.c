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

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#define LCM_ID (0x94)
#define GPIO_LCD_RST_EN      (GPIO28 | 0x80000000)
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------


extern int disp_bls_set_backlight(unsigned int level);

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
static bool lcm_is_init = false;

#define DCT_H561 1

static void lcm_init_power(void)
{
#if (defined(DCT_H561))
#ifdef BUILD_LK
  SET_RESET_PIN(0);
  mt6325_upmu_set_rg_vgp1_en(1);
  MDELAY(50);
#else
  if(lcm_is_init == false)
  {
    printk("%s, begin\n", __func__);
    hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");
    printk("%s, end\n", __func__);
    lcm_is_init = true;
  }
#endif
#endif
}


static void lcm_suspend_power(void)
{
#if (defined(DCT_H561))
#ifdef BUILD_LK
  mt6325_upmu_set_rg_vgp1_en(0);
#else
  lcm_init_power();
  printk("%s, begin\n", __func__);
  hwPowerDown(MT6325_POWER_LDO_VGP1, "LCM_DRV");
  printk("%s, end\n", __func__);
#endif
#endif

}

static void lcm_resume_power(void)
{
#if (defined(DCT_H561))
#ifdef BUILD_LK
  mt6325_upmu_set_rg_vgp1_en(1);
#else
  printk("%s, begin\n", __func__);
  hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");
  printk("%s, end\n", __func__);
#endif
#endif

}

static void lcd_power_en(unsigned char enabled)
{
}

static void init_lcm_registers(void)
{
}


static void lcm_register(void)
{
	unsigned int data_array[30];

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	
	data_array[0] = 0x00043902;
	data_array[1] = 0x9483ffb9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00023902;
	data_array[1] = 0x000013ba;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00113902;
	data_array[1] = 0x040001b1;
	data_array[2] = 0x11110189;
	data_array[3] = 0x3f3f332b;
	data_array[4] = 0xe6011247;
	data_array[5] = 0x000000e2;
	dsi_set_cmdq(data_array, 6, 1);////

	data_array[0] = 0x00073902;
	data_array[1] = 0x09c800b2;
	data_array[2] = 0x00330000;
	dsi_set_cmdq(data_array, 3, 1);
	data_array[0] = 0x00173902;
	data_array[1] = 0x320880b4;
	data_array[2] = 0x15320010;
	data_array[3] = 0x08103208;
	data_array[4] = 0x04430433;
	data_array[5] = 0x06430437;
	data_array[6] = 0x00066161;
	dsi_set_cmdq(data_array, 7, 1);
	
	data_array[0] = 0x00053902;
	data_array[1] = 0x001000c7;
	data_array[2] = 0x00000010;
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0] = 0x00053902;
	data_array[1] = 0x100206bf;
	data_array[2] = 0x00000004;
	dsi_set_cmdq(data_array, 3, 1);
	
	//data_array[0] = 0x00033902;
	//data_array[1] = 0x00170cc0;
	//dsi_set_cmdq(data_array, 2, 1);
	
	
	data_array[0] = 0x00373902;
	data_array[1] = 0x000000d5;
	data_array[2] = 0x01000a00;
	data_array[3] = 0x0000cc00;
	data_array[4] = 0x88888800;
	data_array[5] = 0x88889988;
	data_array[6] = 0x23bbaa88;
	data_array[7] = 0x01456701;
	data_array[8] = 0x88888823;
	data_array[9] = 0x88888888;
	data_array[10] = 0x88998888;
	data_array[11] = 0x54aabb88;
	data_array[12] = 0x32321076;
	data_array[13] = 0x88888810;
	data_array[14] = 0x00013c88;
	dsi_set_cmdq(data_array, 15, 1);

	data_array[0] = 0x00023902;
	data_array[1] = 0x000009cc;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x002b3902;
	data_array[1] = 0x110e01e0;
	data_array[2] = 0x1c3f3f3f;
	data_array[3] = 0x0e0d0739;
	data_array[4] = 0x14111311;
	data_array[5] = 0x0e011b0f;
	data_array[6] = 0x3f3f3f11;
	data_array[7] = 0x0d07391c;
	data_array[8] = 0x1113110e;
	data_array[9] = 0x091b0f14;
	data_array[10] = 0x09110716;
	data_array[11] = 0x00110716;
	dsi_set_cmdq(data_array, 12, 1);
	
	data_array[0] = 0x00023902;
	data_array[1] = 0x0000fab6;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00023902;
	data_array[1] = 0x000032d4;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(150);
	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(50);

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
	
	lcm_register();
    //disp_bls_set_backlight(512);
	//dsi_set_cmdq_V3(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);
	//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}
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

#if 0//(LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
#endif
	
    params->dbi.te_mode                 = LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;


	// DSI
	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;

    // Video mode setting
    params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory     leakage
	// Video mode setting		
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 12;
	params->dsi.vertical_frontporch					= 15;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 40;
	params->dsi.horizontal_backporch				= 86;
	params->dsi.horizontal_frontporch				= 86;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
	
	//params->dsi.LPX=8; 

	// Bit rate calculation
	params->dsi.PLL_CLOCK = 240;//220,258,285
    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
}
static void lcm_suspend(void)
{
	unsigned int data_array[16];
	//unsigned char buffer[2];

#if 0//ndef BUILD_LK
	data_array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0xFE, buffer, 1);
	printk("%s, kernel nt35596 horse debug: nt35596 id = 0x%08x\n", __func__, buffer[0]);
#endif

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
        MDELAY(120);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
        MDELAY(120);

        mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
        MDELAY(50);
        mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
        MDELAY(50);

}


static void lcm_resume(void)
{
	//unsigned int data_array[16];
	//unsigned char buffer[2];

	lcm_init();

#if 0//ndef BUILD_LK
	data_array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0xFE, buffer, 1);
	printk("%s, kernel nt35596 horse debug: nt35596 id = 0x%08x\n", __func__, buffer[0]);
#endif

	//TC358768_DCS_write_1A_0P(0x11); // Sleep Out
	//MDELAY(150);

	//TC358768_DCS_write_1A_0P(0x29); // Display On

}
static unsigned int lcm_compare_id(void)
{
	int array[4];
	char buffer[5];
	char id_high=0;
	char id_low=0;
	int id=0;
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
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(200);
#endif

	array[0] = 0x00043902;
	array[1] = 0x9483FFB9;
	dsi_set_cmdq(array, 2, 1);	

	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0]; //we only need ID
	
	#ifdef BUILD_LK
		printf("hx8394 uboot %s \n", __func__);
		printf("%s id = 0x%08x \n", __func__, id);
	#else
		printk("hx8394 kernel %s \n", __func__);
		printk("%s id = 0x%08x \n", __func__, id);
	#endif
	  
	return (id == LCM_ID) ? 1 : 0;
}

LCM_DRIVER hx8394a_lg53_truly_hd_lcm_drv =
 {
     .name           = "hx8394_lg53_truly_fhd",
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
