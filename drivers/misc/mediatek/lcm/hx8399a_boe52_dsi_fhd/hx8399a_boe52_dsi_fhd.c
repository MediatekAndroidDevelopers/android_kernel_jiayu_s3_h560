/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#ifdef BUILD_LK
     #include <platform/upmu_common.h>
     #include <platform/mt_gpio.h>
     #include <platform/mt_i2c.h>
     #include <platform/mt_pmic.h>
     #include <string.h>
#else
    #include <linux/string.h>
    #if defined(BUILD_UBOOT)
        #include <asm/arch/mt_gpio.h>
    #else
        #include <linux/xlog.h>
        #include <mach/mt_gpio.h>
        #include <mach/mt_pm_ldo.h>
        #include <mach/upmu_common.h>
    #endif
#endif
#include "lcm_drv.h"
#include <cust_gpio_usage.h>

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)

#define LCM_ID (0x94)
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

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

#define   LCM_DSI_CMD_MODE							0


static void lcm_register(void)
{

	unsigned int data_array[40];

    data_array[0] = 0x00043902;                          
    data_array[1] = 0x9983ffB9; 
    dsi_set_cmdq(&data_array, 2, 1); 
    MDELAY(1);
  
    data_array[0] = 0x00023902;                          
    data_array[1] = 0x000000D2; 
    dsi_set_cmdq(&data_array, 2, 1); 
    MDELAY(1);
    
	data_array[0] = 0x00133902;                          
    data_array[1] = 0x2D6400B1; 
	data_array[2] = 0x2209002D;
	data_array[3] = 0x54F17122;
	data_array[4] = 0x0000005E;
    dsi_set_cmdq(&data_array, 5, 1); 
    MDELAY(1);

    data_array[0] = 0x000B3902;                          
    data_array[1] = 0x008000B2; 
	data_array[2] = 0x230A0A7F; 
	data_array[3] = 0x0001024D; 
    dsi_set_cmdq(&data_array, 4, 1); 
    MDELAY(1);

    data_array[0] = 0x002D3902;                          
    data_array[1] = 0x024400B4; 
	data_array[2] = 0x00000604; 
	data_array[3] = 0x00000900; 
    data_array[4] = 0x01100001;
	data_array[5] = 0x00710502;
	data_array[6] = 0x04024404;
	data_array[7] = 0x00000006;
	data_array[8] = 0x01000009;
	data_array[9] = 0x02011000;
	data_array[10] = 0x04000105;
	data_array[11] = 0x00000044;
    dsi_set_cmdq(&data_array, 12, 1);
	MDELAY(1);

	data_array[0] = 0x00203902;                          
    data_array[1] = 0x000100D3; 
    data_array[2] = 0x04040000;
    data_array[3] = 0x00041032; 
    data_array[4] = 0x00000004; 
    data_array[5] = 0x00000000; 
    data_array[6] = 0x21000000; 
    data_array[7] = 0x00130505;
    data_array[8] = 0x08050000; 
    dsi_set_cmdq(&data_array, 9, 1);
	MDELAY(1);
	 
	data_array[0] = 0x00213902;
    data_array[1] = 0x210000D5; 
    data_array[2] = 0x18191920;
    data_array[3] = 0x01000018; 
    data_array[4] = 0x03181800; 
    data_array[5] = 0x00191902; 
    data_array[6] = 0x00000000; 
    data_array[7] = 0x32000000;
    data_array[8] = 0x30313132;
    data_array[9] = 0x00000030;
    dsi_set_cmdq(&data_array, 10, 1);
	MDELAY(1);


	data_array[0] = 0x00213902;
    data_array[1] = 0x204040D6; 
    data_array[2] = 0x19181821;
    data_array[3] = 0x02404019;
    data_array[4] = 0x00181803; 
    data_array[5] = 0x40191901; 
    data_array[6] = 0x40404040; 
    data_array[7] = 0x32404040;
    data_array[8] = 0x30313132;
    data_array[9] = 0x00000030;
    dsi_set_cmdq(&data_array, 10, 1);
	MDELAY(1);


	data_array[0] = 0x00313902;
    data_array[1] = 0x000000D8; 
    data_array[2] = 0x00000000;
    data_array[3] = 0x00000000;
    data_array[4] = 0x00000000; 
    data_array[5] = 0x00000000; 
    data_array[6] = 0x00000000; 
    data_array[7] = 0x00000000;
    data_array[8] = 0x00000000;
    data_array[9] = 0x00000000;
    data_array[10] = 0x00000000;
    data_array[11] = 0xEAAEAA00;
    data_array[12] = 0xEAAEAAAA;
    data_array[13] = 0x000000AA;
    dsi_set_cmdq(&data_array, 14, 1);
	MDELAY(1);


	data_array[0] = 0x002C3902;
    data_array[1] = 0x060500E0; 
    data_array[2] = 0x163F302E;
    data_array[3] = 0x0E0D093A;
    data_array[4] = 0x13121411; 
    data_array[5] = 0x18081812; 
    data_array[6] = 0x05001508; 
    data_array[7] = 0x3F302E06;
    data_array[8] = 0x0D093A16;
    data_array[9] = 0x1214110E;
    data_array[10] = 0x08181213;
    data_array[11] = 0x00150818;
    dsi_set_cmdq(&data_array, 12, 1);
	MDELAY(1);

    data_array[0] = 0x00033902;
    data_array[1] = 0x001818B6;
    dsi_set_cmdq(&data_array, 2, 1); 
    MDELAY(1);

    data_array[0] = 0x00033902;
    data_array[1] = 0x009101C0;
    dsi_set_cmdq(&data_array, 2, 1); 
    MDELAY(1);

    data_array[0] = 0x00023902;
    data_array[1] = 0x000008CC;
    dsi_set_cmdq(&data_array, 2, 1); 
    MDELAY(1);

    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(150);//200

    data_array[0] = 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(30);

}

static void lcm_init(void)
{

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(120);	
	
	lcm_register();
	//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

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



static void lcm_get_params(LCM_PARAMS *params)
{

	memset(params, 0, sizeof(LCM_PARAMS));
	
	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
//	params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
	params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
#endif
	
//    params->dbi.te_mode                 = LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_mode                 = LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;


	// DSI
	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
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

/* Vanzo:songlixin on: Tue, 24 Mar 2015 09:17:27 +0800
 */
 #ifdef VANZO_LCM_ESD_CHECK_SUPPORT
    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable      = 1;
    params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1c;
#endif
// End of Vanzo:songlixin
}
static void lcm_suspend(void)
{
	unsigned int data_array[16];
	//unsigned char buffer[2];

  /*data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
        MDELAY(20);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
        MDELAY(120);*/
        SET_RESET_PIN(1);
        MDELAY(10);
	      SET_RESET_PIN(0);
      	MDELAY(10);
	      SET_RESET_PIN(1);
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

//  data_array[0]= 0x00290508; //HW bug, so need send one HS packet
//  dsi_set_cmdq(data_array, 1, 1);

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);

}
#endif


static unsigned int lcm_compare_id(void)
{
	int array[4];
	char buffer[5];
	char id_high=0;
	char id_low=0;
	int id=0;

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(200);

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

LCM_DRIVER hx8399a_boe52_dsi_fhd_lcm_drv=
 {
     .name           = "hx8399a_boe52_dsi_fhd",
     .set_util_funcs = lcm_set_util_funcs,
     .get_params     = lcm_get_params,
     .init           = lcm_init,
     .suspend        = lcm_suspend,
     .resume         = lcm_resume,
     //.compare_id     = lcm_compare_id,
 #if (LCM_DSI_CMD_MODE)
     .update         = lcm_update,
 #endif
     .init_power        = lcm_init_power,
     .resume_power = lcm_resume_power,
     .suspend_power = lcm_suspend_power,
 
     };
