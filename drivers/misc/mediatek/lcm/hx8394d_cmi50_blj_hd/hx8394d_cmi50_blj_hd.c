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

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#define LCM_ID (0x94)
#define GPIO_LCD_RST_EN      (GPIO28 | 0x80000000)
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};
LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode1_cmd;
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
unsigned cmd;
unsigned char count;
unsigned char para_list[64];
};

#define   LCM_DSI_CMD_MODE							0

static void lcm_register(void)
{
    unsigned int data_array[16];

	data_array[0]=0x00043902;
	data_array[1]=0x9483FFB9;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0]=0x00033902;
	data_array[1]=0x008333BA; // Set MIPI
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0]=0x00103902;
	data_array[1]=0x12126CB1;// Set Power
	data_array[2]=0xF1110434;//34
	data_array[3]=0x2354FF80;
	data_array[4]=0x58D2c080;
	dsi_set_cmdq(&data_array, 5, 1);

	data_array[0]=0x000C3902;
	data_array[1]=0x0E6400b2;
	data_array[2]=0x081C320D;
	data_array[3]=0x004D1C08;
	dsi_set_cmdq(&data_array, 4, 1);

	data_array[0]=0x000D3902;
	data_array[1]=0x51FF00B4; // Set CYC
	data_array[2]=0x035A595A;
	data_array[3]=0x0170015A;
	data_array[4]=0x00000070;
	dsi_set_cmdq(&data_array, 5, 1);


	data_array[0]=0x00263902;
	data_array[1]=0x000700D3; //SET GIP
	data_array[2]=0x00100740;
	data_array[3]=0x00081008;
	data_array[4]=0x0E155408;
	data_array[5]=0x15020E05;
	data_array[6]=0x47060506;
	data_array[7]=0x4B0A0A44;
	data_array[8]=0x08070710;
	data_array[9]=0x0A000000;
	data_array[10]=0x00000100;
	dsi_set_cmdq(&data_array, 11, 1);

	data_array[0]=0x002D3902;
	data_array[1]=0x1B1A1AD5; //SET GIP
	data_array[2]=0x0201001B;
	data_array[3]=0x06050403;
	data_array[4]=0x0A090807;
	data_array[5]=0x1825240B;
	data_array[6]=0x18272618;
	data_array[7]=0x18181818;
	data_array[8]=0x18181818;
	data_array[9]=0x18181818;
	data_array[10]=0x20181818;
	data_array[11]=0x18181821;
	data_array[12]=0x00000018;
	dsi_set_cmdq(&data_array, 13, 1);

	data_array[0]=0x002D3902;
	data_array[1]=0x1B1A1AD6; //SET GIP
	data_array[2]=0x090A0B1B;
	data_array[3]=0x05060708;
	data_array[4]=0x01020304;
	data_array[5]=0x58202100;
	data_array[6]=0x18262758;
	data_array[7]=0x18181818;
	data_array[8]=0x18181818;
	data_array[9]=0x18181818;
	data_array[10]=0x25181818;
	data_array[11]=0x18181824;
	data_array[12]=0x00000018;
	dsi_set_cmdq(&data_array, 13, 1);

	data_array[0]=0x002B3902;
	data_array[1]=0x1B1910E0;
	data_array[2]=0x21352f1f;
	data_array[3]=0x0C0A073b;
	data_array[4]=0x13110e17;
	data_array[5]=0x15091312;
	data_array[6]=0x19101e18;
	data_array[7]=0x352f1f1B;
	data_array[8]=0x0A073b21;
	data_array[9]=0x110e170c;
	data_array[10]=0x09131213;
	data_array[11]=0x001e1815;
	dsi_set_cmdq(&data_array, 12, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x000009CC;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0]=0x00023902;
	data_array[1]=0x007070B6;//SET VCOM
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0]=0x00053902;
	data_array[1]=0x00C000C7;
	data_array[2]=0x000000C0;
	dsi_set_cmdq(&data_array, 3, 1);

	data_array[0] = 0x00110500; //0x11,exit sleep mode,1byte
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(200);

	data_array[0] = 0x00290500; //0x11,exit sleep mode,1byte
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(10);
}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(150);

	lcm_register();
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

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   =SYNC_PULSE_VDO_MODE;// BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
#endif

    // DSI
    /* Command mode setting */
    //1 Three lane or Four lane
    params->dsi.LANE_NUM				= LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Video mode setting
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active				= 4;// 3    2
    params->dsi.vertical_backporch					= 12;// 20   1
    params->dsi.vertical_frontporch					= 15; // 1  12
    params->dsi.vertical_active_line				= FRAME_HEIGHT;

    params->dsi.horizontal_sync_active				= 36;// 50  2
    params->dsi.horizontal_backporch				= 90;
    params->dsi.horizontal_frontporch				= 90;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

    //params->dsi.LPX=8;
    // Bit rate calculation
    params->dsi.PLL_CLOCK = 208;
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];
	//unsigned char buffer[2];

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
    MDELAY(20);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
}

static void lcm_resume(void)
{
	unsigned int data_array[16];

	data_array[0] = 0x00110500; // Sleep Out
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);
	data_array[0] = 0x00290500; // Display On
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(20);
}

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
    printf("%s id = 0x%08x \n", __func__, id);
#else
    printk("%s id = 0x%08x \n", __func__, id);
#endif

	return (id == LCM_ID) ? 1 : 0;
}


static void lcm_init_power(void)
{
}

static void lcm_suspend_power(void)
{
}

static void lcm_resume_power(void)
{
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hx8394d_cmi50_blj_hd_lcm_drv =
{
	.name           = "hx8394d_cmi50_blj_hd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    .init_power     = lcm_init_power,
    .resume_power   = lcm_resume_power,
    .suspend_power  = lcm_suspend_power,
};
