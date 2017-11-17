/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

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

#define GPIO_LCD_RST_EN      (GPIO28 | 0x80000000)

#ifndef TRUE
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

static unsigned int lcm_esd_test = FALSE;	///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	(lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update))
#define dsi_set_cmdq(pdata, queue_size, force_update)		(lcm_util.dsi_set_cmdq(pdata, queue_size, force_update))
#define wrtie_cmd(cmd)										(lcm_util.dsi_write_cmd(cmd))
#define write_regs(addr, pdata, byte_nums)					(lcm_util.dsi_write_regs(addr, pdata, byte_nums))
#define read_reg(cmd)										(lcm_util.dsi_dcs_read_lcm_reg(cmd))
#define read_reg_v2(cmd, buffer, buffer_size)   			(lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size))

#define dsi_lcm_set_gpio_out(pin, out)										lcm_util.set_gpio_out(pin, out)
#define dsi_lcm_set_gpio_mode(pin, mode)									lcm_util.set_gpio_mode(pin, mode)
#define dsi_lcm_set_gpio_dir(pin, dir)										lcm_util.set_gpio_dir(pin, dir)
#define dsi_lcm_set_gpio_pull_enable(pin, en)								lcm_util.set_gpio_pull_enable(pin, en)

#define   LCM_DSI_CMD_MODE							0
#define GPIO_65132_EN (GPIO62 | 0x80000000) 
#define LCM_ID_RM63315          0x3315
static bool lcm_is_init = false;

static void lcm_init_power(void)
{
}

static void lcm_suspend_power(void)
{

}

static void lcm_resume_power(void)
{
}

static void init_lcm_registers(void)
{
	unsigned int data_array[16];
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(200);

     data_array[0]=0x00022902;
    data_array[1]=0x000004b0;//cmd mode
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00072902;//Interface_setting(Video mode)
    data_array[1]=0x000014b3;//cmd mode
    data_array[2]=0x00000000;//cmd mode
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]=0x00032902;//DSI_control
    data_array[1]=0x00b33ab6;//cmd mode
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00032902;//Back_Light_Control_4
    data_array[1]=0x000000c0;//cmd mode
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00232902;//Display_Setting
    data_array[1]=0x106084c1;//cmd mode//0x406004c1
    data_array[2]=0xce6fffeb;//cmd mode
    data_array[3]=0x0217ffff;//cmd mode
    data_array[4]=0xb1ae7358;//cmd mode
    data_array[5]=0xffffc620;//cmd mode
    data_array[6]=0x5ffff31f;//cmd mode
    data_array[7]=0x10101010;//cmd mode
    data_array[8]=0x22010200;//cmd mode
    data_array[9]=0x00010022;//cmd mode
    dsi_set_cmdq(data_array, 10, 1);

    data_array[0]=0x00082902;// (sub pix-inv:0x30)
    data_array[1]=0x80f731c2;//cmd mode
    data_array[2]=0x0000080a;//cmd mode
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]=0x000172902;////Source_Timing_Setting
    data_array[1]=0x000070c4;//cmd mode
    data_array[2]=0x00040000;//cmd mode
    data_array[3]=0x060c0000;//cmd mode
    data_array[4]=0x00000000;//cmd mode
    data_array[5]=0x00000400;//cmd mode
    data_array[6]=0x00060c00;//cmd mode
    dsi_set_cmdq(data_array, 7, 1);

    data_array[0]=0x00292902;
    data_array[1]=0x006900c6;//cmd mode
    data_array[2]=0x00690069;//cmd mode
    data_array[3]=0x00000000;//cmd mode
    data_array[4]=0x00690069;//cmd mode
    data_array[5]=0x07191069;//cmd mode
    data_array[6]=0x69000100;//cmd mode
    data_array[7]=0x69006900;//cmd mode
    data_array[8]=0x00000000;//cmd mode
    data_array[9]=0x69006900;//cmd mode
    data_array[10]=0x19106900;//cmd mode
    data_array[11]=0x00000007;//cmd mode
	  dsi_set_cmdq(data_array, 12, 1);

   data_array[0]=0x000a2902;//Panel PIN Control
    data_array[1]=0x3ffc31cb;//cmd mode
    data_array[2]=0x0000008c;//cmd mode
    data_array[3]=0x0000c000;//cmd mode
	  dsi_set_cmdq(data_array, 4, 1);

    data_array[0]=0x00022902;//Panel Interface Control (Type B)
    data_array[1]=0x00000bcc;//cmd mode
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x000b2902;//Power Setting 1
    data_array[1]=0xbb8111d0;//cmd mode
    data_array[2]=0x194c1e1e;//cmd mode
   data_array[3]=0x00000c19;//cmd mode
    dsi_set_cmdq(data_array, 4, 1);

    data_array[0]=0x001a2902;//Power Setting for Internal Power
    data_array[1]=0xbb331bd3;//cmd mode
    data_array[2]=0x3333b3bb;//cmd mode
    data_array[3]=0x00010033;//cmd mode
    data_array[4]=0x0da0d8a0;//cmd mode
    data_array[5]=0x3b334e4e;//cmd mode
    data_array[6]=0x3d077222;//cmd mode
    data_array[7]=0x000033bf;//cmd mode
	  dsi_set_cmdq(data_array, 8, 1);

    data_array[0]=0x00082902;
    data_array[1]=0x000006d5;//cmd mode
    data_array[2]=0x32015101;//cmd mode
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]=0x001f2902;//sequence_Timing_Control_for_Power_On
    data_array[1]=0x110a01c7;//cmd mode
    data_array[2]=0x3e332618;//cmd mode
    data_array[3]=0x52423850;//cmd mode
    data_array[4]=0x776e6760;//cmd mode
    data_array[5]=0x18110a01;//cmd mode
    data_array[6]=0x503e3326;//cmd mode
    data_array[7]=0x60524238;//cmd mode
    data_array[8]=0x00776e67;//cmd mode
    dsi_set_cmdq(data_array, 9, 1);


    data_array[0]=0x00142902;//sequence_Timing_Control_for_Power_On
    data_array[1]=0x000001c8;//cmd mode
    data_array[2]=0x00fc0000;//cmd mode
    data_array[3]=0x00000000;//cmd mode
    data_array[4]=0x000000fc;//cmd mode
    data_array[5]=0x00fc0000;//cmd mode
    dsi_set_cmdq(data_array, 6, 1);

    data_array[0]=0x00023902;//Panel Interface Control (Type B)
    data_array[1]=0x0000c036;//cmd mode
    dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(100);



}
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS * util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS * params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
#else
	params->dsi.mode = BURST_VDO_MODE;	//SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
#endif

	// DSI
	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	// Video mode setting
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;


	params->dsi.vertical_sync_active = 1;
	params->dsi.vertical_backporch = 1;
	params->dsi.vertical_frontporch = 6;
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 10;
	params->dsi.horizontal_backporch = 118;
	params->dsi.horizontal_frontporch = 118;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	//params->dsi.LPX=8;

	// Bit rate calculation
	//1 Every lane speed
    params->dsi.PLL_CLOCK =450;

    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 1;
    params->dsi.lcm_esd_check_table[0].cmd 			= 0x0a;
    params->dsi.lcm_esd_check_table[0].count 		= 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;
}

static void lcm_init(void)
{
	unsigned int data_array[16];
    mt_set_gpio_mode(GPIO_65132_EN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_EN, GPIO_OUT_ONE);

	mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(150);
	init_lcm_registers();
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];
	//unsigned char buffer[2];

	mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
	MDELAY(20);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(20);

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	data_array[0]=0x00022902;
	data_array[1]=0x000004b0;//cmd mode
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);
	data_array[0]=0x00022902;
	data_array[1]=0x000001b1;//cmd mode
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(60);
	mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_01);
    mt_set_gpio_mode(GPIO_65132_EN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_EN, GPIO_OUT_ZERO);

}


static void lcm_resume(void)
{
#ifndef BUILD_LK
	lcm_init();
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

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] =
	    (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] =
	    (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[4];
	unsigned int array[16];

    mt_set_gpio_mode(GPIO_65132_EN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_EN, GPIO_OUT_ONE);

	mt_set_gpio_mode(GPIO_LCD_RST_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_RST_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ZERO);
	MDELAY(10);
	mt_set_gpio_out(GPIO_LCD_RST_EN, GPIO_OUT_ONE);
	MDELAY(100);

    array[0]=0x00022902;//Panel Interface Control (Type B)
    array[1]=0x000004b0;//cmd mode
    dsi_set_cmdq(array, 2, 1);

	array[0] = 0x00053700;	// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xBF, buffer, 4);
	id = (buffer[2] << 8) | buffer[3] ;		//we only need ID
#ifdef BUILD_LK
	printf("%s, LK r63315 debug: r63315 id = 0x%08x\n", __func__,
	       id);
#else
	printk("%s, kernel r63315 horse debug: r63315 id = 0x%08x\n",
	       __func__, id);
#endif
    if(id == LCM_ID_RM63315)
      return 1;
    else
      return 0;
}

LCM_DRIVER r63315_fhd_dsi_lcm_drv = {
	.name = "r63315_lg50_truly",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
    .init_power     = lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,

#if (LCM_DSI_CMD_MODE)
	.update = lcm_update,
#endif
};
