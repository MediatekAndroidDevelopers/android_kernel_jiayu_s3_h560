/*
 * Device driver for monitoring ambient light intensity in (lux)
 * proximity detection (prox), and Gesture functionality within the
 * AMS-TAOS TMG3992/3.
 *
 * Copyright (c) 2013, AMS-TAOS USA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>

#include <mach/mt_gpio.h>
#include <mach/irqs.h>
#include <mach/eint.h>
#include <cust_eint.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <cust_alsps.h>
#include "gesture.h"
#include <tmg399x.h>
#include <alsps.h>
#include <linux/batch.h>


#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_ERR APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_ERR APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_ERR APS_TAG fmt, ##args)
extern int hwmsen_get_interrupt_data(int sensor, hwm_sensor_data *data);
static int mapping_als_value(struct tmg399x_chip *chip,u16 als);

static int tmg399x_probe(struct i2c_client *client, const struct i2c_device_id *idp);
static int tmg399x_remove(struct i2c_client *client);
static int tmg399x_suspend(struct device *dev);
static int tmg399x_resume(struct device *dev);
static int i2c_tmg399x_probe(struct platform_device *pdev);
static int i2c_tmg399x_remove(struct platform_device *pdev);

struct tmg399x_parameters *get_tmg_arglist(void);
extern struct alsps_hw *tmg399x_get_cust_alsps_hw(void);
  
static int  tmg399x_local_init(void);
//static int  tmg399x_remove(void);
static int tmg399x_init_flag =-1; // 0<==>OK -1 <==> fail

static struct sensor_init_info tmg399x_init_info = {
    .name = "tmg399x",
    .init = tmg399x_local_init,
    .uninit = tmg399x_remove,
};

static u8 const tmg399x_ids[] = {
	0xAB,
	0xAA,
	0x9E,
	0x9F,
	0x84,
	0x60,
	0x49,
};

static char const *tmg399x_names[] = {
	"tmg399x",
	"tmg399x",
	"tmg399x",
	"tmg399x",
	"tmg399x",
	"tmg399x",
	"tmg399x",
};

static u8 const restorable_regs[] = {
	TMG399X_ALS_TIME,
	TMG399X_WAIT_TIME,
	TMG399X_PERSISTENCE,
	TMG399X_CONFIG_1,
	TMG399X_PRX_PULSE,
	TMG399X_GAIN,
	TMG399X_CONFIG_2,
	TMG399X_PRX_OFFSET_NE,
	TMG399X_PRX_OFFSET_SW,
	TMG399X_CONFIG_3,
};

static u8 const prox_gains[] = {
	1,
	2,
	4,
	8
};

static u8 const als_gains[] = {
	1,
	4,
	16,
	64
};

static u8 const prox_pplens[] = {
	4,
	8,
	16,
	32
};

static u8 const led_drives[] = {
	100,
	50,
	25,
	12
};

static u16 const led_boosts[] = {
	100,
	150,
	200,
	300
};

static struct lux_segment segment_default[] = {
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
};

static struct tmg399x_parameters param_default = {
	.als_time = 0xFE, /* 5.6ms */
	.als_gain = AGAIN_64,
	.wait_time = 0xFF, /* 2.78ms */
	.prox_th_min = 0,
	.prox_th_max = 255,
	.persist = PRX_PERSIST(0) | ALS_PERSIST(0),
	.als_prox_cfg1 = 0x60,
	.prox_pulse = PPLEN_32US | PRX_PULSE_CNT(5),
	.prox_gain = PGAIN_4,
	.ldrive = PDRIVE_100MA,
	.als_prox_cfg2 = LEDBOOST_150 | 0x01,
	.prox_offset_ne = 0,
	.prox_offset_sw = 0,
	.als_prox_cfg3 = 0,

	.ges_entry_th = 50,
	.ges_exit_th = 255,
	.ges_cfg1 = FIFOTH_1 | GEXMSK_ALL | GEXPERS_1,
	.ges_cfg2 = GGAIN_4 | GLDRIVE_100 | GWTIME_3,
	.ges_offset_n = 0,
	.ges_offset_s = 0,
	.ges_pulse = GPLEN_32US | GES_PULSE_CNT(5),
	.ges_offset_w = 0,
	.ges_offset_e = 0,
	.ges_dimension = GBOTH_PAIR,
};


/* for gesture and proximity offset calibartion */
static bool docalibration = true;
static bool pstatechanged = false;
static u8 caloffsetstate = START_CALOFF;
static u8 caloffsetdir = DIR_NONE;
static u8 callowtarget = 8;
static u8 calhightarget = 8;
static volatile int ges_mode = 0;
static volatile int gesture = 0;
static volatile int ps_data = 0;
extern void process_rgbc_prox_ges_raw_data(struct tmg399x_chip *chip, u8 type, u8* data, u8 datalen);
extern void init_params_rgbc_prox_ges(void);
extern void set_visible_data_mode(struct tmg399x_chip *chip);
void tmg399x_rgbc_poll_handle(unsigned long data);

static struct tmg399x_chip *gchip;

static int tmg399x_i2c_read(struct tmg399x_chip *chip, u8 reg, u8 *val)
{
	int ret;

	s32 read;
	struct i2c_client *client = chip->client;
	reg += I2C_ADDR_OFFSET;
	ret = i2c_smbus_write_byte(client, reg);
	if (ret < 0) {
		mdelay(3);
		ret = i2c_smbus_write_byte(client, reg);
		if (ret < 0) {
			printk(KERN_ERR"%s: failed 2x to write register %x\n",
					__func__, reg);
			return ret;
		}
	}

	read = i2c_smbus_read_byte(client);
	if (read < 0) {
		mdelay(3);
		read = i2c_smbus_read_byte(client);
		if (read < 0) {
			printk(KERN_ERR"%s: failed read from register %x\n",
					__func__, reg);
		}
		return ret;
	}

	*val = (u8)read;
	return 0;
}

static int tmg399x_i2c_write(struct tmg399x_chip *chip, u8 reg, u8 val)
{
	int ret;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		mdelay(3);
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret < 0) {
			printk(KERN_ERR"%s: failed to write register %x err= %d\n",
					__func__, reg, ret);
		}
	}
	return ret;
}


static int tmg399x_i2c_ram_blk_read(struct tmg399x_chip *chip,
		u8 reg, u8 *val, int size)
{
	int ret;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret =  i2c_smbus_read_i2c_block_data(client,
			reg, size, val);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed 2X at address %x (%d bytes)\n",
				__func__, reg, size);
	}

	return ret;
}

static int tmg399x_flush_regs(struct tmg399x_chip *chip)
{
	unsigned i;
	int ret;
	u8 reg;

	printk("%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		ret = tmg399x_i2c_write(chip, reg, chip->shadow[reg]);
		if (ret < 0) {
			printk(KERN_ERR "%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}

	return ret;
}

static int tmg399x_irq_clr(struct tmg399x_chip *chip, u8 int2clr)
{
	int ret, ret2;

	ret = i2c_smbus_write_byte(chip->client, int2clr);
	if (ret < 0) {
		mdelay(3);
		ret2 = i2c_smbus_write_byte(chip->client, int2clr);
		if (ret2 < 0) {
			printk(KERN_ERR "%s: failed 2x, int to clr %02x\n",
					__func__, int2clr);
		}
		return ret2;
	}

	return ret;
}

static int tmg399x_update_enable_reg(struct tmg399x_chip *chip)
{
	int ret;

	ret = tmg399x_i2c_write(chip, TMG399X_CONTROL,
			chip->shadow[TMG399X_CONTROL]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_4,
			chip->shadow[TMG399X_GES_CFG_4]);

	return ret;
}

static int tmg399x_set_als_gain(struct tmg399x_chip *chip, int gain)
{
	int ret;
	u8 ctrl_reg  = chip->shadow[TMG399X_GAIN] & ~TMG399X_ALS_GAIN_MASK;

	switch (gain) {
	case 1:
		ctrl_reg |= AGAIN_1;
		break;
	case 4:
		ctrl_reg |= AGAIN_4;
		break;
	case 16:
		ctrl_reg |= AGAIN_16;
		break;
	case 64:
		ctrl_reg |= AGAIN_64;
		break;
	default:
		break;
	}

	ret = tmg399x_i2c_write(chip, TMG399X_GAIN, ctrl_reg);
	if (!ret) {
		chip->shadow[TMG399X_GAIN] = ctrl_reg;
		chip->params.als_gain = ctrl_reg & TMG399X_ALS_GAIN_MASK;
	}
	return ret;
}

static void tmg399x_calc_cpl(struct tmg399x_chip *chip)
{
	u32 cpl;
	u32 sat;
	u8 atime = chip->shadow[TMG399X_ALS_TIME];

	cpl = 256 - chip->shadow[TMG399X_ALS_TIME];
	cpl *= TMG399X_ATIME_PER_100;
	cpl /= 100;
	cpl *= als_gains[chip->params.als_gain];

	sat = min_t(u32, MAX_ALS_VALUE, (u32)(256 - atime) << 10);
	sat = sat * 8 / 10;
	chip->als_inf.cpl = cpl;
	chip->als_inf.saturation = sat;
}

static int tmg399x_get_lux(struct tmg399x_chip *chip)
{
	u32 rp1, gp1, bp1, cp1;
	u32 lux = 0;
	u32 cct;
	u32 sat;
	u32 sf;

	/* use time in ms get scaling factor */
	tmg399x_calc_cpl(chip);
	sat = chip->als_inf.saturation;

	if (!chip->als_gain_auto) {
		if (chip->als_inf.clear_raw <= MIN_ALS_VALUE) {
			printk("%s: darkness\n", __func__);
			lux = 0;
			goto exit;
		} else if (chip->als_inf.clear_raw >= sat) {
			printk("%s: saturation, keep lux & cct\n", __func__);
			lux = chip->als_inf.lux;
			goto exit;
		}
	} else {
		u8 gain = als_gains[chip->params.als_gain];
		int ret = -EIO;

		if (gain == 16 && chip->als_inf.clear_raw >= sat) {
			ret = tmg399x_set_als_gain(chip, 1);
		} else if (gain == 16 &&
				chip->als_inf.clear_raw < GAIN_SWITCH_LEVEL) {
			ret = tmg399x_set_als_gain(chip, 64);
		} else if ((gain == 64 &&
					chip->als_inf.clear_raw >= (sat - GAIN_SWITCH_LEVEL)) ||
				(gain == 1 && chip->als_inf.clear_raw < GAIN_SWITCH_LEVEL)) {
			ret = tmg399x_set_als_gain(chip, 16);
		}
		if (!ret) {
			printk(KERN_ERR"%s: gain adjusted, skip\n",	__func__);
			tmg399x_calc_cpl(chip);
			ret = -EAGAIN;
			lux = chip->als_inf.lux;
			goto exit;
		}

		if (chip->als_inf.clear_raw <= MIN_ALS_VALUE) {
			printk("%s: darkness\n", __func__);
			lux = 0;
			goto exit;
		} else if (chip->als_inf.clear_raw >= sat) {
			printk("%s: saturation, keep lux\n",
					__func__);
			lux = chip->als_inf.lux;
			goto exit;
		}
	}

	/* remove ir from counts*/
	rp1 = chip->als_inf.red_raw - chip->als_inf.ir;
	gp1 = chip->als_inf.green_raw - chip->als_inf.ir;
	bp1 = chip->als_inf.blue_raw - chip->als_inf.ir;
	cp1 = chip->als_inf.clear_raw - chip->als_inf.ir;

	if (!chip->als_inf.cpl) {
		printk("%s: zero cpl. Setting to 1\n",
				__func__);
		chip->als_inf.cpl = 1;
	}

	if (chip->als_inf.red_raw > chip->als_inf.ir)
		lux += segment_default[chip->device_index].r_coef * rp1;
	else
		printk(KERN_ERR "%s: lux rp1 = %d\n",
				__func__,
				(segment_default[chip->device_index].r_coef * rp1));

	if (chip->als_inf.green_raw > chip->als_inf.ir)
		lux += segment_default[chip->device_index].g_coef * gp1;
	else
		printk(KERN_ERR"%s: lux gp1 = %d\n",
				__func__,
				(segment_default[chip->device_index].g_coef * rp1));

	if (chip->als_inf.blue_raw > chip->als_inf.ir)
		lux -= segment_default[chip->device_index].b_coef * bp1;
	else
		printk(KERN_ERR "%s: lux bp1 = %d\n",
				__func__,
				(segment_default[chip->device_index].b_coef * rp1));

	sf = chip->als_inf.cpl;

	if (sf > 131072)
		goto error;

	lux /= sf;
	lux *= segment_default[chip->device_index].d_factor;
	lux += 500;
	lux /= 1000;
	chip->als_inf.lux = (u16) lux;

	cct = ((segment_default[chip->device_index].ct_coef * bp1) / rp1) +
		segment_default[chip->device_index].ct_offset;

	chip->als_inf.cct = (u16) cct;

	printk( "chip->als_inf.lux =%d,chip->als_inf.cct=%d\n",chip->als_inf.lux ,chip->als_inf.cct);
exit:
	return 0;

error:
	printk(KERN_ERR "ERROR Scale factor = %d", sf);

	return 1;
}

static int tmg399x_ges_init(struct tmg399x_chip *chip, int on)
{
	int ret;

	if (on) {
		if (chip->pdata) {
			chip->params.ges_entry_th = chip->pdata->parameters.ges_entry_th;
			chip->params.ges_exit_th = chip->pdata->parameters.ges_exit_th;
			chip->params.ges_cfg1 = chip->pdata->parameters.ges_cfg1;
			chip->params.ges_cfg2 = chip->pdata->parameters.ges_cfg2;
			chip->params.ges_offset_n = chip->pdata->parameters.ges_offset_n;
			chip->params.ges_offset_s = chip->pdata->parameters.ges_offset_s;
			chip->params.ges_pulse = chip->pdata->parameters.ges_pulse;
			chip->params.ges_offset_w = chip->pdata->parameters.ges_offset_w;
			chip->params.ges_offset_e = chip->pdata->parameters.ges_offset_e;
			chip->params.ges_dimension = chip->pdata->parameters.ges_dimension;
		} else {
			chip->params.ges_entry_th = param_default.ges_entry_th;
			chip->params.ges_exit_th = param_default.ges_exit_th;
			chip->params.ges_cfg1 = param_default.ges_cfg1;
			chip->params.ges_cfg2 = param_default.ges_cfg2;
			chip->params.ges_offset_n = param_default.ges_offset_n;
			chip->params.ges_offset_s = param_default.ges_offset_s;
			chip->params.ges_pulse = param_default.ges_pulse;
			chip->params.ges_offset_w = param_default.ges_offset_w;
			chip->params.ges_offset_e = param_default.ges_offset_e;
			chip->params.ges_dimension = param_default.ges_dimension;
		}
	}

	/* Initial gesture registers */
	chip->shadow[TMG399X_GES_ENTH]  = chip->params.ges_entry_th;
	chip->shadow[TMG399X_GES_EXTH]  = chip->params.ges_exit_th;
	chip->shadow[TMG399X_GES_CFG_1] = chip->params.ges_cfg1;
	chip->shadow[TMG399X_GES_CFG_2] = chip->params.ges_cfg2;
	chip->shadow[TMG399X_GES_OFFSET_N] = chip->params.ges_offset_n;
	chip->shadow[TMG399X_GES_OFFSET_S] = chip->params.ges_offset_s;
	chip->shadow[TMG399X_GES_PULSE] = chip->params.ges_pulse;
	chip->shadow[TMG399X_GES_OFFSET_W] = chip->params.ges_offset_w;
	chip->shadow[TMG399X_GES_OFFSET_E] = chip->params.ges_offset_e;
	chip->shadow[TMG399X_GES_CFG_3] = chip->params.ges_dimension;

	ret = tmg399x_i2c_write(chip, TMG399X_GES_ENTH,
			chip->shadow[TMG399X_GES_ENTH]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_EXTH,
			chip->shadow[TMG399X_GES_EXTH]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_1,
			chip->shadow[TMG399X_GES_CFG_1]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_2,
			chip->shadow[TMG399X_GES_CFG_2]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_N,
			chip->shadow[TMG399X_GES_OFFSET_N]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_S,
			chip->shadow[TMG399X_GES_OFFSET_S]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_PULSE,
			chip->shadow[TMG399X_GES_PULSE]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_W,
			chip->shadow[TMG399X_GES_OFFSET_W]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_E,
			chip->shadow[TMG399X_GES_OFFSET_E]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_3,
			chip->shadow[TMG399X_GES_CFG_3]);

	return ret;
}

static int tmg399x_ges_enable(struct tmg399x_chip *chip, int on)
{
	int ret = 0;

	printk( "%s: on = %d\n", __func__, on);
	if (on) {
		/* initialize */
		ret |= tmg399x_ges_init(chip, 1);
		if (ret < 0)
			return ret;

		chip->shadow[TMG399X_CONTROL] |= (TMG399X_EN_PWR_ON |
				TMG399X_EN_PRX | TMG399X_EN_PRX_IRQ | TMG399X_EN_GES);
		chip->shadow[TMG399X_GES_CFG_4] |= TMG399X_GES_EN_IRQ;
		ret |= tmg399x_update_enable_reg(chip);

		mdelay(3);
		set_visible_data_mode(chip);
	} else {
		chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_GES;
		chip->shadow[TMG399X_GES_CFG_4] &= ~ TMG399X_GES_EN_IRQ;
		if (!chip->prx_enabled)
			chip->shadow[TMG399X_CONTROL] &= ~(TMG399X_EN_PRX | TMG399X_EN_PRX_IRQ);
		if (!chip->prx_enabled && !chip->als_enabled)
			chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_PWR_ON;
		ret = tmg399x_update_enable_reg(chip);
	}
	if (!ret)
		chip->ges_enabled = on;

	return ret;
}

static int tmg399x_prox_enable(struct tmg399x_chip *chip, int on)
{
	int ret;
	struct alsps_hw *aphw=tmg399x_get_cust_alsps_hw();

	printk( "%s: on = %d\n", __func__, on);
	if (on) {
		chip->shadow[TMG399X_CONTROL] |= (TMG399X_EN_PWR_ON |
				TMG399X_EN_PRX );
		if(aphw->polling_mode_ps == 0)
			chip->shadow[TMG399X_CONTROL] |= TMG399X_EN_PRX_IRQ;
		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
		mdelay(3);
		set_visible_data_mode(chip);
	} else {
		if (!chip->ges_enabled)
			chip->shadow[TMG399X_CONTROL] &= ~(TMG399X_EN_PRX_IRQ | TMG399X_EN_PRX);
		if (!chip->ges_enabled && !chip->als_enabled)
			chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_PWR_ON;
		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;

		chip->prx_inf.raw = 0;
	}
	if (!ret)
		chip->prx_enabled = on;

	return ret;
}

static int tmg399x_als_enable(struct tmg399x_chip *chip, int on)
{
	int ret;
	struct alsps_hw *aphw=tmg399x_get_cust_alsps_hw();
	dev_info(&chip->client->dev, "%s: on = %d\n", __func__, on);
	if (on) {
		/* set up timer for RGBC polling */
		chip->rgbc_poll_flag = false;
		setup_timer(&chip->rgbc_timer,
				tmg399x_rgbc_poll_handle, (unsigned long)chip);
		chip->rgbc_timer.expires = jiffies + HZ/10;
		add_timer(&chip->rgbc_timer);

		/* use auto gain setting */
		chip->als_gain_auto = true;

		chip->shadow[TMG399X_CONTROL] |=
			(TMG399X_EN_PWR_ON | TMG399X_EN_ALS );
		if(aphw->polling_mode_als == 0){
			chip->shadow[TMG399X_CONTROL] |= TMG399X_EN_ALS_IRQ;
		}
		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
		mdelay(3);
	} else {
		del_timer(&chip->rgbc_timer);

		chip->shadow[TMG399X_CONTROL] &=
			~(TMG399X_EN_ALS | TMG399X_EN_ALS_IRQ);

		if (!chip->ges_enabled && !chip->prx_enabled)
			chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_PWR_ON;
		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;

		chip->als_inf.lux = 0;
		chip->als_inf.cct = 0;
	}
	if (!ret)
		chip->als_enabled = on;

	return ret;
}

static int tmg399x_wait_enable(struct tmg399x_chip *chip, int on)
{
	int ret;

	printk( "%s: on = %d\n", __func__, on);
	if (on) {
		chip->shadow[TMG399X_CONTROL] |= TMG399X_EN_WAIT;

		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
		mdelay(3);
	} else {
		chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_WAIT;

		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
	}
	if (!ret)
		chip->wait_enabled = on;

	return ret;
}

static ssize_t tmg399x_ges_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ges_enabled);
}

static ssize_t tmg399x_ges_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		tmg399x_ges_enable(chip, 1);
	else
		tmg399x_ges_enable(chip, 0);

	mutex_unlock(&chip->lock);
	return size;
}
static ssize_t tmg399x_ges_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", ges_mode);
}

static ssize_t tmg399x_ges_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		ges_mode = 1;
	else
		ges_mode = 0;

	mutex_unlock(&chip->lock);
	return size;
}
static ssize_t tmg399x_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", gesture);
}

static ssize_t tmg399x_gesture_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	gesture = value;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_enabled);
}

static ssize_t tmg399x_prox_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		tmg399x_prox_enable(chip, 1);
	else
		tmg399x_prox_enable(chip, 0);

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_als_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_enabled);
}

static ssize_t tmg399x_als_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;
	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		tmg399x_als_enable(chip, 1);
	else
		tmg399x_als_enable(chip, 0);

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_wait_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->wait_enabled);
}

static ssize_t tmg399x_wait_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		tmg399x_wait_enable(chip, 1);
	else
		tmg399x_wait_enable(chip, 0);

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_als_itime_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int t;
	t = 256 - chip->params.als_time;
	t *= TMG399X_ATIME_PER_100;
	t /= 100;
	return snprintf(buf, PAGE_SIZE, "%d (in ms)\n", t);
}

static ssize_t tmg399x_als_itime_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long itime;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &itime);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (itime > 712 || itime < 3) {
		printk(KERN_ERR"als integration time range [3,712]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	itime *= 100;
	itime /= TMG399X_ATIME_PER_100;
	itime = (256 - itime);
	chip->shadow[TMG399X_ALS_TIME] = (u8)itime;
	chip->params.als_time = (u8)itime;
	ret = tmg399x_i2c_write(chip, TMG399X_ALS_TIME,
			chip->shadow[TMG399X_ALS_TIME]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_wait_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int t;
	t = 256 - chip->params.wait_time;
	t *= TMG399X_ATIME_PER_100;
	t /= 100;
	if (chip->params.als_prox_cfg1 & WLONG)
		t *= 12;
	return snprintf(buf, PAGE_SIZE, "%d (in ms)\n", t);
}

static ssize_t tmg399x_wait_time_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long time;
	int ret;
	u8 cfg1;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &time);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (time > 8540 || time < 3) {
		printk(KERN_ERR	"wait time range [3,8540]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	cfg1 = chip->shadow[TMG399X_CONFIG_1] & ~0x02;
	if (time > 712) {
		cfg1 |= WLONG;
		time /= 12;
	}

	time *= 100;
	time /= TMG399X_ATIME_PER_100;
	time = (256 - time);
	chip->shadow[TMG399X_WAIT_TIME] = (u8)time;
	chip->params.wait_time = (u8)time;
	chip->shadow[TMG399X_CONFIG_1] = cfg1;
	chip->params.als_prox_cfg1 = cfg1;
	ret = tmg399x_i2c_write(chip, TMG399X_WAIT_TIME,
			chip->shadow[TMG399X_WAIT_TIME]);
	ret |= tmg399x_i2c_write(chip, TMG399X_CONFIG_1,
			chip->shadow[TMG399X_CONFIG_1]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_persist_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.persist & 0xF0) >> 4);
}

static ssize_t tmg399x_prox_persist_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long persist;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &persist);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (persist > 15) {
		dev_err(&chip->client->dev,
				"prox persistence range [0,15]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	chip->shadow[TMG399X_PERSISTENCE] &= 0x0F;
	chip->shadow[TMG399X_PERSISTENCE] |= (((u8)persist << 4) & 0xF0);
	chip->params.persist = chip->shadow[TMG399X_PERSISTENCE];
	ret = tmg399x_i2c_write(chip, TMG399X_PERSISTENCE,
			chip->shadow[TMG399X_PERSISTENCE]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_als_persist_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.persist & 0x0F));
}

static ssize_t tmg399x_als_persist_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long persist;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &persist);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (persist > 15) {
		printk(KERN_ERR"als persistence range [0,15]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	chip->shadow[TMG399X_PERSISTENCE] &= 0xF0;
	chip->shadow[TMG399X_PERSISTENCE] |= ((u8)persist & 0x0F);
	chip->params.persist = chip->shadow[TMG399X_PERSISTENCE];
	ret = tmg399x_i2c_write(chip, TMG399X_PERSISTENCE,
			chip->shadow[TMG399X_PERSISTENCE]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_pulse_len_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = (chip->params.prox_pulse & 0xC0) >> 6;
	return snprintf(buf, PAGE_SIZE, "%duS\n", prox_pplens[i]);
}

static ssize_t tmg399x_prox_pulse_len_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long length;
	int ret;
	u8 ppulse;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &length);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (length != 4 && length != 8 &&
			length != 16 &&	length != 32) {
		printk(KERN_ERR"pulse length set: {4, 8, 16, 32}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	ppulse = chip->shadow[TMG399X_PRX_PULSE] & 0x3F;
	switch (length){
	case 4:
		ppulse |= PPLEN_4US;
		break;
	case 8:
		ppulse |= PPLEN_8US;
		break;
	case 16:
		ppulse |= PPLEN_16US;
		break;
	case 32:
		ppulse |= PPLEN_32US;
		break;
	default:
		break;
	}
	chip->shadow[TMG399X_PRX_PULSE] = ppulse;
	chip->params.prox_pulse = ppulse;
	ret = tmg399x_i2c_write(chip, TMG399X_PRX_PULSE,
			chip->shadow[TMG399X_PRX_PULSE]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_pulse_cnt_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.prox_pulse & 0x3F) + 1);
}

static ssize_t tmg399x_prox_pulse_cnt_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long count;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &count);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (count > 64 || count == 0) {
		printk(KERN_ERR	"prox pulse count range [1,64]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	count -= 1;

	chip->shadow[TMG399X_PRX_PULSE] &= 0xC0;
	chip->shadow[TMG399X_PRX_PULSE] |= ((u8)count & 0x3F);
	chip->params.prox_pulse = chip->shadow[TMG399X_PRX_PULSE];
	ret = tmg399x_i2c_write(chip, TMG399X_PRX_PULSE,
			chip->shadow[TMG399X_PRX_PULSE]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_gain_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = chip->params.prox_gain >> 2;
	return snprintf(buf, PAGE_SIZE, "%d\n", prox_gains[i]);
}

static ssize_t tmg399x_prox_gain_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long gain;
	int ret;
	u8 ctrl_reg;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &gain);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (gain != 1 && gain != 2 && gain != 4 && gain != 8) {
		printk(KERN_ERR"prox gain set: {1, 2, 4, 8, 16}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	ctrl_reg = chip->shadow[TMG399X_GAIN] & ~TMG399X_PRX_GAIN_MASK;
	switch (gain){
	case 1:
		ctrl_reg |= PGAIN_1;
		break;
	case 2:
		ctrl_reg |= PGAIN_2;
		break;
	case 4:
		ctrl_reg |= PGAIN_4;
		break;
	case 8:
		ctrl_reg |= PGAIN_8;
		break;
	default:
		break;
	}

	ret = tmg399x_i2c_write(chip, TMG399X_GAIN, ctrl_reg);
	if (!ret) {
		chip->shadow[TMG399X_GAIN] = ctrl_reg;
		chip->params.prox_gain = ctrl_reg & TMG399X_PRX_GAIN_MASK;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_led_drive_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%dmA\n",
			led_drives[chip->params.ldrive]);
}

static ssize_t tmg399x_led_drive_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long ldrive;
	int ret;
	u8 ctrl_reg;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &ldrive);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (ldrive != 100 && ldrive != 50 &&
			ldrive != 25 && ldrive != 12) {
		printk(KERN_ERR	"led drive set: {100, 50, 25, 12}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	ctrl_reg = chip->shadow[TMG399X_GAIN] & ~TMG399X_LDRIVE_MASK;
	switch (ldrive){
	case 100:
		ctrl_reg |= PDRIVE_100MA;
		chip->params.ldrive = 0;
		break;
	case 50:
		ctrl_reg |= PDRIVE_50MA;
		chip->params.ldrive = 1;
		break;
	case 25:
		ctrl_reg |= PDRIVE_25MA;
		chip->params.ldrive = 2;
		break;
	case 12:
		ctrl_reg |= PDRIVE_12MA;
		chip->params.ldrive = 3;
		break;
	default:
		break;
	}
	chip->shadow[TMG399X_GAIN] = ctrl_reg;
	ret = tmg399x_i2c_write(chip, TMG399X_GAIN, chip->shadow[TMG399X_GAIN]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_als_gain_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d (%s)\n",
			als_gains[chip->params.als_gain],
			chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmg399x_als_gain_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long gain;
	int i = 0;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &gain);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (gain != 0 && gain != 1 && gain != 4 &&
			gain != 16 && gain != 64) {
		printk(KERN_ERR	"als gain set: {0(auto), 1, 4, 16, 64}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	while (i < sizeof(als_gains)) {
		if (gain == als_gains[i])
			break;
		i++;
	}

	if (gain) {
		chip->als_gain_auto = false;
		ret = tmg399x_set_als_gain(chip, als_gains[i]);
		if (!ret)
			tmg399x_calc_cpl(chip);
	} else {
		chip->als_gain_auto = true;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_led_boost_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = (chip->params.als_prox_cfg2 & 0x30) >> 4;
	return snprintf(buf, PAGE_SIZE, "%d percents\n", led_boosts[i]);
}

static ssize_t tmg399x_led_boost_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long lboost;
	int ret;
	u8 cfg2;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &lboost);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (lboost != 100 && lboost != 150 &&
			lboost != 200 && lboost != 300) {
		printk(KERN_ERR"led boost set: {100, 150, 200, 300}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	cfg2 = chip->shadow[TMG399X_CONFIG_2] & ~0x30;
	switch (lboost){
	case 100:
		cfg2 |= LEDBOOST_100;
		break;
	case 150:
		cfg2 |= LEDBOOST_150;
		break;
	case 200:
		cfg2 |= LEDBOOST_200;
		break;
	case 300:
		cfg2 |= LEDBOOST_300;
		break;
	default:
		break;
	}
	chip->shadow[TMG399X_CONFIG_2] = cfg2;
	chip->params.als_prox_cfg2 = cfg2;
	ret = tmg399x_i2c_write(chip, TMG399X_CONFIG_2,
			chip->shadow[TMG399X_CONFIG_2]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_sat_irq_en_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.als_prox_cfg2 & 0x80) >> 7);
}

static ssize_t tmg399x_sat_irq_en_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool psien;
	int ret;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &psien)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	chip->shadow[TMG399X_CONFIG_2] &= 0x7F;
	if (psien)
		chip->shadow[TMG399X_CONFIG_2] |= PSIEN;
	chip->params.als_prox_cfg2 = chip->shadow[TMG399X_CONFIG_2];
	ret = tmg399x_i2c_write(chip, TMG399X_CONFIG_2,
			chip->shadow[TMG399X_CONFIG_2]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_offset_ne_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			chip->params.prox_offset_ne);
}

static ssize_t tmg399x_prox_offset_ne_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	long offset_ne;
	int ret;
	u8 offset = 0;

	mutex_lock(&chip->lock);
	ret = kstrtol(buf, 10, &offset_ne);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (offset_ne > 127 || offset_ne < -127) {
		printk(KERN_ERR "prox offset range [-127, 127]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (offset_ne < 0)
		offset = 128 - offset_ne;
	else
		offset = offset_ne;

	ret = tmg399x_i2c_write(chip, TMG399X_PRX_OFFSET_NE, offset);
	if (!ret) {
		chip->params.prox_offset_ne = (s8)offset_ne;
		chip->shadow[TMG399X_PRX_OFFSET_NE] = offset;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_offset_sw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			chip->params.prox_offset_sw);
}

static ssize_t tmg399x_prox_offset_sw_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	long offset_sw;
	int ret;
	u8 offset = 0;

	mutex_lock(&chip->lock);
	ret = kstrtol(buf, 10, &offset_sw);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (offset_sw > 127 || offset_sw < -127) {
		printk(KERN_ERR"prox offset range [-127, 127]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (offset_sw < 0)
		offset = 128 - offset_sw;
	else
		offset = offset_sw;

	ret = tmg399x_i2c_write(chip, TMG399X_PRX_OFFSET_SW, offset);
	if (!ret) {
		chip->params.prox_offset_sw = (s8)offset_sw;
		chip->shadow[TMG399X_PRX_OFFSET_SW] = offset;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_mask_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%.2x\n",
			chip->params.als_prox_cfg3 & 0x0F);
}

static ssize_t tmg399x_prox_mask_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long prx_mask;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtol(buf, 10, &prx_mask);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (prx_mask > 15) {
		printk(KERN_ERR "prox mask range [0, 15]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if ((prx_mask == 5) || (prx_mask == 7) ||
			(prx_mask == 10) || (prx_mask == 11) ||
			(prx_mask == 13) || (prx_mask == 14))
		prx_mask |= PCMP;

	chip->shadow[TMG399X_CONFIG_3] &= 0xD0;
	chip->shadow[TMG399X_CONFIG_3] |= (u8)prx_mask;
	chip->params.als_prox_cfg3 = chip->shadow[TMG399X_CONFIG_3];
	ret = tmg399x_i2c_write(chip, TMG399X_CONFIG_3,
			chip->shadow[TMG399X_CONFIG_3]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_device_prx_raw(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.raw);
}

static ssize_t tmg399x_device_prx_detected(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.detected);
}

static ssize_t tmg399x_ges_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "n:%d s:%d w:%d e:%d\n",
			chip->params.ges_offset_n, chip->params.ges_offset_s,
			chip->params.ges_offset_w, chip->params.ges_offset_e);
}

static ssize_t tmg399x_lux_table_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	struct lux_segment *s = chip->segment;
	int i, k;

	for (i = k = 0; i < chip->segment_num; i++)
		k += snprintf(buf + k, PAGE_SIZE - k,
				"%d:%d,%d,%d,%d,%d,%d\n", i,
				s[i].d_factor,
				s[i].r_coef,
				s[i].g_coef,
				s[i].b_coef,
				s[i].ct_coef,
				s[i].ct_offset
				);
	return k;
}

static ssize_t tmg399x_lux_table_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i;
	u32 d_factor, r_coef, g_coef, b_coef, ct_coef, ct_offset;

	mutex_lock(&chip->lock);
	if (7 != sscanf(buf, "%10d:%10d,%10d,%10d,%10d,%10d,%10d",
				&i, &d_factor, &r_coef, &g_coef, &b_coef, &ct_coef, &ct_offset)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (i >= chip->segment_num) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	chip->segment[i].d_factor = d_factor;
	chip->segment[i].r_coef = r_coef;
	chip->segment[i].g_coef = g_coef;
	chip->segment[i].b_coef = b_coef;
	chip->segment[i].ct_coef = ct_coef;
	chip->segment[i].ct_offset = ct_offset;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_auto_gain_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n",
			chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmg399x_auto_gain_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		chip->als_gain_auto = true;
	else
		chip->als_gain_auto = false;

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_device_als_lux(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.lux);
}

static ssize_t tmg399x_als_red_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.red_raw);
}

static ssize_t tmg399x_als_green_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.green_raw);
}

static ssize_t tmg399x_als_blue_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.blue_raw);
}

static ssize_t tmg399x_als_clear_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.clear_raw);
}

static ssize_t tmg399x_als_cct_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cct);
}

static struct device_attribute prox_attrs[] = {
	__ATTR(ges_mode_state, 0666, tmg399x_ges_mode_show,
			tmg399x_ges_mode_store),
	__ATTR(gesture, 0666, tmg399x_gesture_show,
			tmg399x_gesture_store),
	__ATTR(prx_power_state, 0666, tmg399x_prox_enable_show,
			tmg399x_prox_enable_store),
	__ATTR(ges_power_state, 0666, tmg399x_ges_enable_show,
			tmg399x_ges_enable_store),
	__ATTR(prx_persist, 0666, tmg399x_prox_persist_show,
			tmg399x_prox_persist_store),
	__ATTR(prx_pulse_length, 0666, tmg399x_prox_pulse_len_show,
			tmg399x_prox_pulse_len_store),
	__ATTR(prx_pulse_count, 0666, tmg399x_prox_pulse_cnt_show,
			tmg399x_prox_pulse_cnt_store),
	__ATTR(prx_gain, 0666, tmg399x_prox_gain_show,
			tmg399x_prox_gain_store),
	__ATTR(led_drive, 0666, tmg399x_led_drive_show,
			tmg399x_led_drive_store),
	__ATTR(led_boost, 0666, tmg399x_led_boost_show,
			tmg399x_led_boost_store),
	__ATTR(prx_sat_irq_en, 0666, tmg399x_sat_irq_en_show,
			tmg399x_sat_irq_en_store),
	__ATTR(prx_offset_ne, 0666, tmg399x_prox_offset_ne_show,
			tmg399x_prox_offset_ne_store),
	__ATTR(prx_offset_sw, 0666, tmg399x_prox_offset_sw_show,
			tmg399x_prox_offset_sw_store),
	__ATTR(prx_mask, 0666, tmg399x_prox_mask_show,
			tmg399x_prox_mask_store),
	__ATTR(prx_raw, 0666, tmg399x_device_prx_raw, NULL),
	__ATTR(prx_detect, 0666, tmg399x_device_prx_detected, NULL),
	__ATTR(ges_offset, 0666, tmg399x_ges_offset_show, NULL),
};

static struct device_attribute als_attrs[] = {
	__ATTR(als_power_state, 0666, tmg399x_als_enable_show,
			tmg399x_als_enable_store),
	__ATTR(wait_time_en, 0666, tmg399x_wait_enable_show,
			tmg399x_wait_enable_store),
	__ATTR(als_Itime, 0666, tmg399x_als_itime_show,
			tmg399x_als_itime_store),
	__ATTR(wait_time, 0666, tmg399x_wait_time_show,
			tmg399x_wait_time_store),
	__ATTR(als_persist, 0666, tmg399x_als_persist_show,
			tmg399x_als_persist_store),
	__ATTR(als_gain, 0666, tmg399x_als_gain_show,
			tmg399x_als_gain_store),
	__ATTR(lux_table, 0000, tmg399x_lux_table_show,
			tmg399x_lux_table_store),
	__ATTR(als_auto_gain, 0666, tmg399x_auto_gain_enable_show,
			tmg399x_auto_gain_enable_store),
	__ATTR(als_lux, 0666, tmg399x_device_als_lux, NULL),
	__ATTR(als_red, 0666, tmg399x_als_red_show, NULL),
	__ATTR(als_green, 0666, tmg399x_als_green_show, NULL),
	__ATTR(als_blue, 0666, tmg399x_als_blue_show, NULL),
	__ATTR(als_clear, 0666, tmg399x_als_clear_show, NULL),
	__ATTR(als_cct, 0666, tmg399x_als_cct_show, NULL),
};
static ssize_t tmg399x_show_ps(struct device_driver *ddri, char *buf)
{
    int ps;
    ps = ps_data << 2;
    if(ps > 1023) ps = 1023;
	return snprintf(buf, PAGE_SIZE, "%d\n", ps);
}
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, tmg399x_show_ps,    NULL);
static struct device_attribute *tmg399x_attr_list[] = {
	&driver_attr_ps,
};
static int tmg399x_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(tmg399x_attr_list)/sizeof(tmg399x_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, tmg399x_attr_list[idx]))
		{
			APS_ERR("driver_create_file (%s) = %d\n", tmg399x_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int tmg399x_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(tmg399x_attr_list)/sizeof(tmg399x_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, tmg399x_attr_list[idx]);
	}

	return err;
}

void tmg399x_report_prox(struct tmg399x_chip *chip, u8 detected)
{
	struct alsps_hw *aphw=tmg399x_get_cust_alsps_hw();
	hwm_sensor_data hsd;
	if (chip->p_idev) {
		chip->prx_inf.detected = detected;
		input_report_abs(chip->p_idev, ABS_DISTANCE,
				chip->prx_inf.detected ? 0 : 1);
		input_sync(chip->p_idev);
		if(0 == aphw->polling_mode_ps){
			hsd.values[0] = (chip->prx_inf.detected ? 0 : 1);
			hsd.value_divide=1;
			hwmsen_get_interrupt_data(ID_PROXIMITY,&hsd);
		}
	}
}
EXPORT_SYMBOL_GPL(tmg399x_report_prox);

void tmg399x_report_ges(struct tmg399x_chip *chip, int ges_report)
{
	struct alsps_hw *aphw=tmg399x_get_cust_alsps_hw();
	hwm_sensor_data hsd;
	static int ctrm=0;
	int x1, y1, x2, y2;
	int delta_x, delta_y, i;
	int reportv;

	x1 = y1 = x2 = y2 = 0;
	switch (ges_report) {
	case 0:
		x1 = 800;
		x2 = 400;
		y1 = y2 = 1000;
		reportv = GES_DOWN;
		break;
	case 2:
		x1 = x2 = 500;
		y1 = 1200;
		y2 = 800;
		reportv = GES_LEFT;
		break;
	case 4:
		x1 = 400;
		x2 = 800;
		y1 = y2 = 1000;
		reportv = GES_UP;
		break;
	case 6:
		x1 = x2 = 500;
		y1 = 800;
		y2 = 1200;
		reportv = GES_RIGHT;
		break;
	default:
		return;
	}
	if(ges_mode)
	{
		gesture = reportv;
	}
	else{
		/*
		   if(0 == aphw->polling_mode_gesture){
		   ctrm = (ctrm++)%0xF;
		   hsd.values[0] = (ctrm << 4) | (reportv & 0x0F);
		   hsd.values[1] = hsd.values[2] = 0;
		   hsd.value_divide=1;
		   hwmsen_get_interrupt_data(ID_GESTURE,&hsd);
		   }
		   */
		delta_x = (x2 - x1) / 5;
		delta_y = (y2 - y1) / 5;
		for (i = 0; i < 5; i++) {
			//input_report_abs(chip->p_idev, ABS_MT_TRACKING_ID, 0);
			input_report_key(chip->p_idev, BTN_TOUCH, 1);
			input_report_abs(chip->p_idev, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(chip->p_idev, ABS_MT_POSITION_Y, (y1 + delta_y * i));
			input_report_abs(chip->p_idev, ABS_MT_POSITION_X, (x1 + delta_x * i));
			//input_report_abs(chip->p_idev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(chip->p_idev);
			input_sync(chip->p_idev);
			msleep(1);
		}
		input_report_key(chip->p_idev, BTN_TOUCH, 0);
		input_mt_sync(chip->p_idev);
		input_sync(chip->p_idev);
	}
}
EXPORT_SYMBOL_GPL(tmg399x_report_ges);

void tmg399x_report_als(struct tmg399x_chip *chip)
{
	int lux;
	struct alsps_hw *aphw=tmg399x_get_cust_alsps_hw();
	hwm_sensor_data hsd;

	chip->rgbc_poll_flag = false;
	tmg399x_get_lux(chip);
	mod_timer(&chip->rgbc_timer, jiffies + HZ/10);

	printk(KERN_INFO "lux:%d cct:%d\n", chip->als_inf.lux, chip->als_inf.cct);
	lux = chip->als_inf.lux;
	input_report_abs(chip->a_idev, ABS_MISC, lux);
	input_sync(chip->a_idev);
	if(0 == aphw->polling_mode_als){
		hsd.values[0]=mapping_als_value(chip,lux); //for mtk
		hsd.value_divide=1;
		hwmsen_get_interrupt_data(ID_LIGHT,&hsd);
	}
}
EXPORT_SYMBOL_GPL(tmg399x_report_als);

static u8 tmg399x_ges_nswe_min(struct tmg399x_ges_nswe nswe)
{
	u8 min = nswe.north;
	if (nswe.south < min) min = nswe.south;
	if (nswe.west < min) min = nswe.west;
	if (nswe.east < min) min = nswe.east;
	return min;
}

static u8 tmg399x_ges_nswe_max(struct tmg399x_ges_nswe nswe)
{
	u8 max = nswe.north;
	if (nswe.south > max) max = nswe.south;
	if (nswe.west > max) max = nswe.west;
	if (nswe.east > max) max = nswe.east;
	return max;
}

static bool tmg399x_change_prox_offset(struct tmg399x_chip *chip, u8 state, u8 direction)
{
	u8 offset;

	switch (state) {
	case CHECK_PROX_NE:
		if (direction == DIR_UP) {
			/* negtive offset will increase the results */
			if (chip->params.prox_offset_ne == -127)
				return false;
			chip->params.prox_offset_ne--;
		} else if (direction == DIR_DOWN) {
			/* positive offset will decrease the results */
			if (chip->params.prox_offset_ne == 127)
				return false;
			chip->params.prox_offset_ne++;
		}
		/* convert int value to offset */
		INT2OFFSET(offset, chip->params.prox_offset_ne);
		chip->shadow[TMG399X_PRX_OFFSET_NE] = offset;
		tmg399x_i2c_write(chip, TMG399X_PRX_OFFSET_NE, offset);
		break;
	case CHECK_PROX_SW:
		if (direction == DIR_UP) {
			if (chip->params.prox_offset_sw == -127)
				return false;
			chip->params.prox_offset_sw--;
		} else if (direction == DIR_DOWN) {
			if (chip->params.prox_offset_sw == 127)
				return false;
			chip->params.prox_offset_sw++;
		}
		INT2OFFSET(offset, chip->params.prox_offset_sw);
		chip->shadow[TMG399X_PRX_OFFSET_SW] = offset;
		tmg399x_i2c_write(chip, TMG399X_PRX_OFFSET_SW, offset);
		break;
	default:
		break;
	}

	return true;
}

static void tmg399x_cal_prox_offset(struct tmg399x_chip *chip, u8 prox)
{
	/* start to calibrate the offset of prox */
	if (caloffsetstate == START_CALOFF) {
		/* mask south and west diode */
		chip->shadow[TMG399X_CONFIG_3] = 0x06;
		tmg399x_i2c_write(chip, TMG399X_CONFIG_3,
				chip->shadow[TMG399X_CONFIG_3]);
		pstatechanged = true;
		caloffsetstate = CHECK_PROX_NE;
		caloffsetdir = DIR_NONE;
		return;
	}

	/* calibrate north and east diode of prox */
	if (caloffsetstate == CHECK_PROX_NE) {
		/* only one direction one time, up or down */
		if ((caloffsetdir != DIR_DOWN) && (prox < callowtarget/2)) {
			caloffsetdir = DIR_UP;
			if (tmg399x_change_prox_offset(chip, CHECK_PROX_NE, DIR_UP))
				return;
		} else if ((caloffsetdir != DIR_UP) && (prox > calhightarget/2)) {
			caloffsetdir = DIR_DOWN;
			if (tmg399x_change_prox_offset(chip, CHECK_PROX_NE, DIR_DOWN))
				return;
		}

		/* north and east diode offset calibration complete, mask
		   north and east diode and start to calibrate south and west diode */
		chip->shadow[TMG399X_CONFIG_3] = 0x09;
		tmg399x_i2c_write(chip, TMG399X_CONFIG_3,
				chip->shadow[TMG399X_CONFIG_3]);
		pstatechanged = true;
		caloffsetstate = CHECK_PROX_SW;
		caloffsetdir = DIR_NONE;
		return;
	}

	/* calibrate south and west diode of prox */
	if (caloffsetstate == CHECK_PROX_SW) {
		if ((caloffsetdir != DIR_DOWN) && (prox < callowtarget/2)) {
			caloffsetdir = DIR_UP;
			if (tmg399x_change_prox_offset(chip, CHECK_PROX_SW, DIR_UP))
				return;
		} else if ((caloffsetdir != DIR_UP) && (prox > calhightarget/2)) {
			caloffsetdir = DIR_DOWN;
			if (tmg399x_change_prox_offset(chip, CHECK_PROX_SW, DIR_DOWN))
				return;
		}

		/* prox offset calibration complete, mask none diode,
		   start to calibrate gesture offset */
		chip->shadow[TMG399X_CONFIG_3] = 0x00;
		tmg399x_i2c_write(chip, TMG399X_CONFIG_3,
				chip->shadow[TMG399X_CONFIG_3]);
		pstatechanged = true;
		caloffsetstate = CHECK_NSWE_ZERO;
		caloffsetdir = DIR_NONE;
		return;
	}
}

static int tmg399x_change_ges_offset(struct tmg399x_chip *chip, struct tmg399x_ges_nswe nswe_data, u8 direction)
{
	u8 offset;
	int ret = false;

	/* calibrate north diode */
	if (direction == DIR_UP) {
		if (nswe_data.north < callowtarget) {
			/* negtive offset will increase the results */
			if (chip->params.ges_offset_n == -127)
				goto cal_ges_offset_south;
			chip->params.ges_offset_n--;
		}
	} else if (direction == DIR_DOWN) {
		if (nswe_data.north > calhightarget) {
			/* positive offset will decrease the results */
			if (chip->params.ges_offset_n == 127)
				goto cal_ges_offset_south;
			chip->params.ges_offset_n++;
		}
	}
	/* convert int value to offset */
	INT2OFFSET(offset, chip->params.ges_offset_n);
	chip->shadow[TMG399X_GES_OFFSET_N] = offset;
	tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_N, offset);
	ret = true;

cal_ges_offset_south:
	/* calibrate south diode */
	if (direction == DIR_UP) {
		if (nswe_data.south < callowtarget) {
			if (chip->params.ges_offset_s == -127)
				goto cal_ges_offset_west;
			chip->params.ges_offset_s--;
		}
	} else if (direction == DIR_DOWN) {
		if (nswe_data.south > calhightarget) {
			if (chip->params.ges_offset_s == 127)
				goto cal_ges_offset_west;
			chip->params.ges_offset_s++;
		}
	}
	INT2OFFSET(offset, chip->params.ges_offset_s);
	chip->shadow[TMG399X_GES_OFFSET_S] = offset;
	tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_S, offset);
	ret = true;

cal_ges_offset_west:
	/* calibrate west diode */
	if (direction == DIR_UP) {
		if (nswe_data.west < callowtarget) {
			if (chip->params.ges_offset_w == -127)
				goto cal_ges_offset_east;
			chip->params.ges_offset_w--;
		}
	} else if (direction == DIR_DOWN) {
		if (nswe_data.west > calhightarget) {
			if (chip->params.ges_offset_w == 127)
				goto cal_ges_offset_east;
			chip->params.ges_offset_w++;
		}
	}
	INT2OFFSET(offset, chip->params.ges_offset_w);
	chip->shadow[TMG399X_GES_OFFSET_W] = offset;
	tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_W, offset);
	ret = true;

cal_ges_offset_east:
	/* calibrate east diode */
	if (direction == DIR_UP) {
		if (nswe_data.east < callowtarget) {
			if (chip->params.ges_offset_e == -127)
				goto cal_ges_offset_exit;
			chip->params.ges_offset_e--;
		}
	} else if (direction == DIR_DOWN) {
		if (nswe_data.east > calhightarget) {
			if (chip->params.ges_offset_e == 127)
				goto cal_ges_offset_exit;
			chip->params.ges_offset_e++;
		}
	}
	INT2OFFSET(offset, chip->params.ges_offset_e);
	chip->shadow[TMG399X_GES_OFFSET_E] = offset;
	tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_E, offset);
	ret = true;

cal_ges_offset_exit:
	return ret;
}

static void tmg399x_cal_ges_offset(struct tmg399x_chip *chip, struct tmg399x_ges_nswe* nswe_data, u8 len)
{
	u8 i;
	u8 min;
	u8 max;
	for (i = 0; i < len; i++) {
		if (caloffsetstate == CHECK_NSWE_ZERO) {
			min = tmg399x_ges_nswe_min(nswe_data[i]);
			max = tmg399x_ges_nswe_max(nswe_data[i]);

			/* only one direction one time, up or down */
			if ((caloffsetdir != DIR_DOWN) && (min <= callowtarget)) {
				caloffsetdir = DIR_UP;
				if (tmg399x_change_ges_offset(chip, nswe_data[i], DIR_UP))
					return;
			} else if ((caloffsetdir != DIR_UP) && (max > calhightarget)) {
				caloffsetdir = DIR_DOWN;
				if (tmg399x_change_ges_offset(chip, nswe_data[i], DIR_DOWN))
					return;
			}

			/* calibration is ok */
			caloffsetstate = CALOFF_OK;
			caloffsetdir = DIR_NONE;
			docalibration = false;
			break;
		}
	}
}

void tmg399x_start_calibration(struct tmg399x_chip *chip)
{
	docalibration = true;
	caloffsetstate = START_CALOFF;
	/* entry threshold is set min 0, exit threshold is set max 255,
	   and NSWE are all masked for exit, gesture state will be force
	   enter and exit every cycle */
	chip->params.ges_entry_th = 0;
	chip->shadow[TMG399X_GES_ENTH] = 0;
	tmg399x_i2c_write(chip, TMG399X_GES_ENTH, 0);
	chip->params.ges_exit_th = 255;
	chip->shadow[TMG399X_GES_EXTH] = 255;
	tmg399x_i2c_write(chip, TMG399X_GES_EXTH, 255);
	chip->params.ges_cfg1 |= GEXMSK_ALL;
	chip->shadow[TMG399X_GES_CFG_1] = chip->params.ges_cfg1;
	tmg399x_i2c_write(chip, TMG399X_GES_CFG_1, chip->params.ges_cfg1);
}
EXPORT_SYMBOL_GPL(tmg399x_start_calibration);

void tmg399x_set_ges_thresh(struct tmg399x_chip *chip, u8 entry, u8 exit)
{
	/* set entry and exit threshold for gesture state enter and exit */
	chip->params.ges_entry_th = entry;
	chip->shadow[TMG399X_GES_ENTH] = entry;
	tmg399x_i2c_write(chip, TMG399X_GES_ENTH, entry);
	chip->params.ges_exit_th = exit;
	chip->shadow[TMG399X_GES_EXTH] = exit;
	tmg399x_i2c_write(chip, TMG399X_GES_EXTH, exit);
	chip->params.ges_cfg1 &= ~GEXMSK_ALL;
	chip->shadow[TMG399X_GES_CFG_1] = chip->params.ges_cfg1;
	tmg399x_i2c_write(chip, TMG399X_GES_CFG_1, chip->params.ges_cfg1);
}
EXPORT_SYMBOL_GPL(tmg399x_set_ges_thresh);

void tmg399x_rgbc_poll_handle(unsigned long data)
{
	struct tmg399x_chip *chip = (struct tmg399x_chip *)data;
	chip->rgbc_poll_flag = true;
}

void tmg399x_update_rgbc(struct tmg399x_chip *chip)
{
}
EXPORT_SYMBOL_GPL(tmg399x_update_rgbc);

static int tmg399x_check_and_report(struct tmg399x_chip *chip)
{
	int ret;
	u8 status;
	u8 numofdset;
	u8 len;
	mutex_lock(&chip->lock);
	mt_eint_mask(CUST_EINT_ALS_NUM);
	ret = tmg399x_i2c_read(chip, TMG399X_STATUS,
			&chip->shadow[TMG399X_STATUS]);
	if (ret < 0) {
		printk(KERN_ERR"%s: failed to read tmg399x status\n",
				__func__);
		goto exit_clr;
	}

	status = chip->shadow[TMG399X_STATUS];
	if ((status & TMG399X_ST_PRX_IRQ) == TMG399X_ST_PRX_IRQ) {
		/* read prox raw data */
		tmg399x_i2c_read(chip, TMG399X_PRX_CHAN,
				&chip->shadow[TMG399X_PRX_CHAN]);
		ps_data = chip->shadow[TMG399X_PRX_CHAN];
		if (chip->prx_enabled)
			chip->prx_inf.raw = chip->shadow[TMG399X_PRX_CHAN];
		/* ignore the first prox data when proximity state changed */
		if (pstatechanged) {
			pstatechanged = false;
		} else {
			if (docalibration && caloffsetstate != CHECK_NSWE_ZERO) {
				/* do prox offset calibration */
				tmg399x_cal_prox_offset(chip, chip->shadow[TMG399X_PRX_CHAN]);
			} else {
				/* process prox data */
				process_rgbc_prox_ges_raw_data(chip, PROX_DATA, (u8*)&chip->shadow[TMG399X_PRX_CHAN], 1);
			}
		}
		/* clear the irq of prox */
		tmg399x_irq_clr(chip, TMG399X_CMD_PROX_INT_CLR);
	}

	if ((status & TMG399X_ST_GES_IRQ) == TMG399X_ST_GES_IRQ) {
		len = 0;
		/* get how many data sets in fifo */
		tmg399x_i2c_read(chip, TMG399X_GES_FLVL, &numofdset);

		/* read gesture data from fifo to SW buffer */
		tmg399x_i2c_ram_blk_read(chip, TMG399X_GES_NFIFO,
				(u8 *)&chip->ges_raw_data[len], 4);
		/* calculate number of gesture data sets */
		len += numofdset;
		if (len == 32) {
			printk (KERN_INFO "gesture buffer overflow!\n");
			len = 0;
			mutex_unlock(&chip->lock);
			return false;
		}

		if (docalibration && caloffsetstate != CALOFF_OK) {
			/* do gesture offset calibration */
			tmg399x_cal_ges_offset(chip, (struct tmg399x_ges_nswe*)chip->ges_raw_data, 1);
		} else {
			process_rgbc_prox_ges_raw_data(chip, GES_DATA, (u8*)chip->ges_raw_data, 4);
		}
	}

	if ((status & TMG399X_ST_ALS_IRQ) == TMG399X_ST_ALS_IRQ) {
		tmg399x_i2c_ram_blk_read(chip, TMG399X_CLR_CHANLO,
				(u8 *)&chip->shadow[TMG399X_CLR_CHANLO], 8);
		process_rgbc_prox_ges_raw_data(chip, RGBC_DATA, (u8*)&chip->shadow[TMG399X_CLR_CHANLO], 8);
		tmg399x_irq_clr(chip, TMG399X_CMD_ALS_INT_CLR);
	}

exit_clr:
	mutex_unlock(&chip->lock);

	return ret;
}

static void tmg399x_irq_work(struct work_struct *work)
{
	struct tmg399x_chip *chip =
		container_of(work, struct tmg399x_chip, irq_work);
	tmg399x_check_and_report(chip);
	mt_eint_unmask(CUST_EINT_ALS_NUM);
	//	enable_irq(chip->client->irq);
};

static irqreturn_t tmg399x_irq(void)
{
	struct device *dev = &gchip->client->dev;
	//mutex_lock(&chip->lock);
	if (gchip->in_suspend) {
		printk( "%s: in suspend\n", __func__);
		gchip->irq_pending = 1;
		disable_irq_nosync(gchip->client->irq);
		goto bypass;
	}

	schedule_work(&gchip->irq_work);
bypass:
	return IRQ_HANDLED;
}

static int tmg399x_set_segment_table(struct tmg399x_chip *chip,
		struct lux_segment *segment, int seg_num)
{
	int i;
	struct device *dev = &chip->client->dev;

	chip->seg_num_max = chip->pdata->segment_num ?
		chip->pdata->segment_num : ARRAY_SIZE(segment_default);
	/*
	   if (!chip->segment) {
	   printk( "%s: allocating segment table\n", __func__);
	   chip->segment = kzalloc(sizeof(*chip->segment) *
	   chip->seg_num_max, GFP_KERNEL);
	   if (!chip->segment) {
	   printk( "%s: no memory!\n", __func__);
	   return -ENOMEM;
	   }
	   }
	   */
	if (seg_num > chip->seg_num_max) {
		printk( "%s: %d segment requested, %d applied\n",
				__func__, seg_num, chip->seg_num_max);
		chip->segment_num = chip->seg_num_max;
	} else {
		chip->segment_num = seg_num;
	}
	/*
	   memcpy(chip->segment, segment,
	   chip->segment_num * sizeof(*chip->segment));
	   printk( "%s: %d segment requested, %d applied\n", __func__,
	   seg_num, chip->seg_num_max);
	   for (i = 0; i < chip->segment_num; i++)
	   printk("seg %d: d_factor %d, r_coef %d, g_coef %d, b_coef %d, ct_coef %d ct_offset %d\n",
	   i, chip->segment[i].d_factor, chip->segment[i].r_coef,
	   chip->segment[i].g_coef, chip->segment[i].b_coef,
	   chip->segment[i].ct_coef, chip->segment[i].ct_offset);
	   */
	return 0;
}

static void tmg399x_set_defaults(struct tmg399x_chip *chip)
{
	struct device *dev = &chip->client->dev;

	if (chip->pdata) {
		printk( "%s: Loading pltform data\n", __func__);
		chip->params.als_time = chip->pdata->parameters.als_time;
		chip->params.als_gain = chip->pdata->parameters.als_gain;
		chip->params.wait_time = chip->pdata->parameters.wait_time;
		chip->params.prox_th_min = chip->pdata->parameters.prox_th_min;
		chip->params.prox_th_max = chip->pdata->parameters.prox_th_max;
		chip->params.persist = chip->pdata->parameters.persist;
		chip->params.als_prox_cfg1 = chip->pdata->parameters.als_prox_cfg1;
		chip->params.prox_pulse = chip->pdata->parameters.prox_pulse;
		chip->params.prox_gain = chip->pdata->parameters.prox_gain;
		chip->params.ldrive = chip->pdata->parameters.ldrive;
		chip->params.als_prox_cfg2 = chip->pdata->parameters.als_prox_cfg2;
		chip->params.prox_offset_ne = chip->pdata->parameters.prox_offset_ne;
		chip->params.prox_offset_sw = chip->pdata->parameters.prox_offset_sw;
		chip->params.als_prox_cfg3 = chip->pdata->parameters.als_prox_cfg3;
	} else {
		printk( "%s: use defaults\n", __func__);
		chip->params.als_time = param_default.als_time;
		chip->params.als_gain = param_default.als_gain;
		chip->params.wait_time = param_default.wait_time;
		chip->params.prox_th_min = param_default.prox_th_min;
		chip->params.prox_th_max = param_default.prox_th_max;
		chip->params.persist = param_default.persist;
		chip->params.als_prox_cfg1 = param_default.als_prox_cfg1;
		chip->params.prox_pulse = param_default.prox_pulse;
		chip->params.prox_gain = param_default.prox_gain;
		chip->params.ldrive = param_default.ldrive;
		chip->params.als_prox_cfg2 = param_default.als_prox_cfg2;
		chip->params.prox_offset_ne = param_default.prox_offset_ne;
		chip->params.prox_offset_sw = param_default.prox_offset_sw;
		chip->params.als_prox_cfg3 = param_default.als_prox_cfg3;
	}

	chip->als_gain_auto = false;

	/* Initial proximity threshold */
/* Vanzo:yangzhihong on: Fri, 06 Feb 2015 16:04:17 +0800
	chip->shadow[TMG399X_PRX_MINTHRESHLO] = 0;//chip->params.prox_th_min;
 */
	chip->shadow[TMG399X_PRX_MINTHRESHLO] = chip->params.prox_th_min;
// End of Vanzo:yangzhihong

	chip->shadow[TMG399X_PRX_MAXTHRESHHI] = chip->params.prox_th_max;
	tmg399x_i2c_write(chip, TMG399X_PRX_MINTHRESHLO,
			chip->shadow[TMG399X_PRX_MINTHRESHLO]);
	tmg399x_i2c_write(chip, TMG399X_PRX_MAXTHRESHHI,
			chip->shadow[TMG399X_PRX_MAXTHRESHHI]);

	tmg399x_i2c_write(chip, TMG399X_ALS_MINTHRESHLO, 0x00);
	tmg399x_i2c_write(chip, TMG399X_ALS_MINTHRESHHI, 0x00);
	tmg399x_i2c_write(chip, TMG399X_ALS_MAXTHRESHLO, 0xFF);
	tmg399x_i2c_write(chip, TMG399X_ALS_MAXTHRESHHI, 0xFF);

	chip->shadow[TMG399X_ALS_TIME]      = chip->params.als_time;
	chip->shadow[TMG399X_WAIT_TIME]     = chip->params.wait_time;
	chip->shadow[TMG399X_PERSISTENCE]   = chip->params.persist;
	chip->shadow[TMG399X_CONFIG_1]      = chip->params.als_prox_cfg1;
	chip->shadow[TMG399X_PRX_PULSE]     = chip->params.prox_pulse;
	chip->shadow[TMG399X_GAIN]          = chip->params.als_gain |
		chip->params.prox_gain | chip->params.ldrive;
	chip->shadow[TMG399X_CONFIG_2]      = chip->params.als_prox_cfg2;
	chip->shadow[TMG399X_PRX_OFFSET_NE] = chip->params.prox_offset_ne;
	chip->shadow[TMG399X_PRX_OFFSET_SW] = chip->params.prox_offset_sw;
	chip->shadow[TMG399X_CONFIG_3]      = chip->params.als_prox_cfg3;
}

static int tmg399x_get_id(struct tmg399x_chip *chip, u8 *id, u8 *rev)
{
	int ret;
	ret = tmg399x_i2c_read(chip, TMG399X_REVID, rev);
	ret |= tmg399x_i2c_read(chip, TMG399X_CHIPID, id);
	return ret;
}

static int tmg399x_pltf_power_on(struct tmg399x_chip *chip)
{
	int ret = 0;
	if (chip->pdata->platform_power) {
		ret = chip->pdata->platform_power(&chip->client->dev,
				POWER_ON);
		mdelay(10);
	}
	chip->unpowered = ret != 0;
	return ret;
}

static int tmg399x_pltf_power_off(struct tmg399x_chip *chip)
{
	int ret = 0;
	if (chip->pdata->platform_power) {
		ret = chip->pdata->platform_power(&chip->client->dev,
				POWER_OFF);
		chip->unpowered = ret == 0;
	} else {
		chip->unpowered = false;
	}
	return ret;
}

static int tmg399x_power_on(struct tmg399x_chip *chip)
{
	int ret;
	ret = tmg399x_pltf_power_on(chip);
	if (ret)
		return ret;
	printk("%s: chip was off, restoring regs\n",
			__func__);
	return tmg399x_flush_regs(chip);
}

static int tmg399x_prox_idev_open(struct input_dev *idev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(&idev->dev);
	int ret;
	bool als = chip->a_idev && chip->a_idev->users;

	printk("%s\n", __func__);
	mutex_lock(&chip->lock);
	if (chip->unpowered) {
		ret = tmg399x_power_on(chip);
		if (ret)
			goto chip_on_err;
	}
	ret = tmg399x_prox_enable(chip, 1);
	if (ret && !als)
		tmg399x_pltf_power_off(chip);
chip_on_err:
	mutex_unlock(&chip->lock);
	return ret;
}

static void tmg399x_prox_idev_close(struct input_dev *idev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(&idev->dev);

	printk( "%s\n", __func__);
	mutex_lock(&chip->lock);
	tmg399x_prox_enable(chip, 0);
	if (!chip->a_idev || !chip->a_idev->users)
		tmg399x_pltf_power_off(chip);
	mutex_unlock(&chip->lock);
}

static int tmg399x_als_idev_open(struct input_dev *idev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(&idev->dev);
	int ret;
	bool prox = chip->p_idev && chip->p_idev->users;

	printk("%s\n", __func__);
	mutex_lock(&chip->lock);
	if (chip->unpowered) {
		ret = tmg399x_power_on(chip);
		if (ret)
			goto chip_on_err;
	}
	ret = tmg399x_als_enable(chip, 1);
	if (ret && !prox)
		tmg399x_pltf_power_off(chip);
chip_on_err:
	mutex_unlock(&chip->lock);
	return ret;
}

static void tmg399x_als_idev_close(struct input_dev *idev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(&idev->dev);
	printk("%s\n", __func__);
	mutex_lock(&chip->lock);
	tmg399x_als_enable(chip, 0);
	if (!chip->p_idev || !chip->p_idev->users)
		tmg399x_pltf_power_off(chip);
	mutex_unlock(&chip->lock);
}

static int tmg399x_add_sysfs_interfaces(struct device *dev,
		struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, a + i);
	printk(KERN_ERR"%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void tmg399x_remove_sysfs_interfaces(struct device *dev,
		struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		device_remove_file(dev, a + i);
}
//MTK hwsensor,avoid code hal,add hj
static int mapping_als_value(struct tmg399x_chip *chip,u16 als)
{
	int idx;
	int invalid = 0,als_level_num;
	struct alsps_hw *hw = tmg399x_get_cust_alsps_hw();

	als_level_num=sizeof(hw->als_level)/sizeof(hw->als_level[0]);
	for(idx = 0; idx < als_level_num; idx++)
	{
		if(als < hw->als_level[idx])
		{
			break;
		}
	}
	if(idx >= als_level_num + 1)
	{
		printk(KERN_ERR"exceed range\n");
		idx = als_level_num;
	}
	if(chip->als_enabled){
		//		printk("ALS: raw data %05d => value = %05d\n", als, hw->als_value[idx]);
		return hw->als_value[idx];
	}else{
		//		printk("als not open\n");
		return -1;
	}
}
static int gesture_sensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err=0,value;
	struct tmg399x_chip *chip = self;
	hwm_sensor_data *sensor_data;

	printk("ges cmd=%d\n",command);
	switch (command)
	{
	case SENSOR_DELAY:
		break;
	case SENSOR_ENABLE:
		if((buff_in == NULL) || (size_in < sizeof(int)))
		{
			printk(KERN_ERR"Enable als sensor parameter error!\n");
			err = -EINVAL;
		}
		else
		{
			value = *(int *)buff_in;
			mutex_lock(&chip->lock);
			if(value)
				err=tmg399x_ges_enable(chip, 1);
			else
				err=tmg399x_ges_enable(chip, 0);
			mutex_unlock(&chip->lock);
		}
		break;
	case SENSOR_GET_DATA:
		sensor_data = (hwm_sensor_data *)buff_out;

		sensor_data->values[0] = GES_RIGHT;
		sensor_data->value_divide = 1;
		sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
		break;
	default:
		err=-1;
		printk(KERN_ERR"no params\n");
		break;
	}
	return err;
}
static int als_sensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err=0,value;
	struct tmg399x_chip *chip = self;
	hwm_sensor_data *sensor_data;

	switch (command)
	{
	case SENSOR_DELAY:
		break;
	case SENSOR_ENABLE:
		if((buff_in == NULL) || (size_in < sizeof(int)))
		{
			printk(KERN_ERR"Enable als sensor parameter error!\n");
			err = -EINVAL;
		}
		else
		{
			value = *(int *)buff_in;
			mutex_lock(&chip->lock);
			if(value)
			{
				tmg399x_als_enable(chip, 1);
			}
			else
			{
				tmg399x_als_enable(chip, 0);
			}
			mutex_unlock(&chip->lock);
		}
		break;
	case SENSOR_GET_DATA:
		sensor_data = (hwm_sensor_data *)buff_out;
		//			tmg399x_get_als(chip);
		err=tmg399x_get_lux(chip);
		sensor_data->values[0] = mapping_als_value(chip,chip->als_inf.lux);
		printk("tmg3993 get als value=%d\n",sensor_data->values[0]);
		sensor_data->value_divide = 1;
		sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
		break;
	default:
		err=-1;
		printk(KERN_ERR"no params\n");
		break;
	}
	return err;
}
static int prx_sensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err=0,value;
	struct tmg399x_chip *chip = self;
	hwm_sensor_data *sensor_data;

	switch (command)
	{
	case SENSOR_DELAY:
		break;
	case SENSOR_ENABLE:
		if((buff_in == NULL) || (size_in < sizeof(int)))
		{
			printk(KERN_ERR"Enable als sensor parameter error!\n");
			err = -EINVAL;
		}
		else
		{
			value = *(int *)buff_in;
			mutex_lock(&chip->lock);
			if(value)
				tmg399x_prox_enable(chip, 1);
			else
				tmg399x_prox_enable(chip, 0);
			mutex_unlock(&chip->lock);
		}
		break;
	case SENSOR_GET_DATA:
		sensor_data = (hwm_sensor_data *)buff_out;
		//			tmg399x_get_prox(chip);
		sensor_data->values[0] = !!!chip->prx_inf.detected;
		printk("tmg339x get als vaule = %d\n",sensor_data->values[0]);
		sensor_data->value_divide = 1;
		sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
		break;
	default:
		err=-1;
		printk(KERN_ERR"no params\n");
		break;
	}
	return err;
}
static struct platform_driver tmg399x_alsps_driver = {
	.probe      = i2c_tmg399x_probe,
	.remove     = i2c_tmg399x_remove,
	.driver     = {
		.name  = "als_ps",
		//.owner = THIS_MODULE,
	}
};
static struct i2c_device_id tmg399x_idtable[] = {
	{ "tmg3993", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, tmg399x_idtable);

static const struct dev_pm_ops tmg399x_pm_ops = {
	.suspend = tmg399x_suspend,
	.resume  = tmg399x_resume,
};

static struct i2c_driver tmg399x_driver = {
	.driver = {
		.name = "tmg3993",
		.pm = &tmg399x_pm_ops,
	},
	.id_table = tmg399x_idtable,
	.probe = tmg399x_probe,
	.remove = tmg399x_remove,
};

static int tmg399x_probe(struct i2c_client *client,
		const struct i2c_device_id *idp)
{
	int i, ret;
	u8 id, rev;
	struct device *dev = &client->dev;
	static struct tmg399x_chip *chip;
	struct tmg399x_i2c_platform_data *pdata = dev->platform_data;

	bool powered = 0;
	//MTK way
	struct hwmsen_object obj_ps, obj_als,obj_gesture;
	struct alsps_hw *aphw=tmg399x_get_cust_alsps_hw();
	//
	printk("X %s: client->irq = %d\n", __func__, client->irq);
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_ERR"%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}
	if (!pdata) {
		printk(KERN_ERR "%s: platform data required\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}
	if (!(pdata->prox_name || pdata->als_name) || client->irq < 0) {
		printk(KERN_ERR"%s: no reason to run.\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}
	if (pdata->platform_init) {
		ret = pdata->platform_init();
		if (ret)
			goto init_failed;
	}
	if (pdata->platform_power) {
		ret = pdata->platform_power(dev, POWER_ON);
		if (ret) {
			printk(KERN_ERR"%s: pltf power on failed\n", __func__);
			goto pon_failed;
		}
		powered = true;
		mdelay(10);
	}
	chip = kmalloc(sizeof(struct tmg399x_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}

	memset(chip,0,sizeof(struct tmg399x_chip));

	gchip=chip;
	chip->client = client;
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);

	chip->seg_num_max = chip->pdata->segment_num ?
		chip->pdata->segment_num : ARRAY_SIZE(segment_default);
	if (chip->pdata->segment)
		ret = tmg399x_set_segment_table(chip, chip->pdata->segment,
				chip->pdata->segment_num);
	else
		ret =  tmg399x_set_segment_table(chip, segment_default,
				ARRAY_SIZE(segment_default));
	if (ret)
		goto set_segment_failed;
	{
		ret = tmg399x_get_id(chip, &id, &rev);
		if (ret < 0)
			printk(KERN_ERR"%s: failed to get tmg399x id\n",
					__func__);

		printk(KERN_ERR"tmg339x,%s: device id:%02x device rev:%02x\n", __func__,
				id, rev);
	}
	for (i = 0; i < ARRAY_SIZE(tmg399x_ids); i++) {
		if (id == tmg399x_ids[i])
			break;
	}
	if (i < ARRAY_SIZE(tmg399x_names)) {
		printk(KERN_ERR"%s: '%s rev. %d' detected i=%d\n", __func__,
				tmg399x_names[i], rev),i;
		chip->device_index = i;
	} else {
		printk( "%s: not supported chip id\n", __func__);
		ret = -EOPNOTSUPP;
		goto id_failed;
	}
	pdata->parameters.prox_th_min=aphw->ps_threshold_low;
	pdata->parameters.prox_th_max=aphw->ps_threshold_high;
	mutex_init(&chip->lock);

	/* disable all */
	tmg399x_prox_enable(chip, 0);
	tmg399x_ges_enable(chip, 0);
	tmg399x_als_enable(chip, 0);

	tmg399x_set_defaults(chip);
	ret = tmg399x_flush_regs(chip);
	if (ret)
		goto flush_regs_failed;
	if (pdata->platform_power) {
		pdata->platform_power(dev, POWER_OFF);
		powered = false;
		chip->unpowered = true;
	}

	/* gesture processing initialize */
	init_params_rgbc_prox_ges();

	if (!pdata->prox_name)
		goto bypass_prox_idev;
	chip->p_idev = input_allocate_device();
	if (!chip->p_idev) {
		printk(KERN_ERR"%s: no memory for input_dev '%s'\n",
				__func__, pdata->prox_name);
		ret = -ENODEV;
		goto input_p_alloc_failed;
	}
	chip->p_idev->name = pdata->prox_name;
	chip->p_idev->id.bustype = BUS_I2C;
/*********** register input device  **************/
	set_bit(EV_ABS, chip->p_idev->evbit);
	set_bit(EV_KEY, chip->p_idev->evbit);
	set_bit(ABS_X, chip->p_idev->absbit);
	set_bit(ABS_Y, chip->p_idev->absbit);
	set_bit(ABS_PRESSURE, chip->p_idev->absbit);
	set_bit(BTN_TOUCH, chip->p_idev->keybit);
	set_bit(INPUT_PROP_DIRECT, chip->p_idev->propbit);

	set_bit(ABS_DISTANCE, chip->p_idev->absbit);
	set_bit(ABS_MT_TRACKING_ID, chip->p_idev->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, chip->p_idev->absbit);
	set_bit(ABS_MT_TOUCH_MINOR, chip->p_idev->absbit);
	set_bit(ABS_MT_POSITION_X, chip->p_idev->absbit);
	set_bit(ABS_MT_POSITION_Y, chip->p_idev->absbit);

	input_set_abs_params(chip->p_idev, ABS_DISTANCE, 0, 1, 0, 0);
	input_set_abs_params(chip->p_idev, ABS_MT_POSITION_X, 0, 1080, 0, 0);
	input_set_abs_params(chip->p_idev, ABS_MT_POSITION_Y, 0, 1920, 0, 0);
	input_set_abs_params(chip->p_idev, ABS_MT_TOUCH_MAJOR, 0, 100, 0, 0);
	input_set_abs_params(chip->p_idev, ABS_MT_TOUCH_MINOR, 0, 100, 0, 0);
	input_set_abs_params(chip->p_idev, ABS_X, 0, 1080, 0, 0);
	input_set_abs_params(chip->p_idev, ABS_Y, 0, 1920, 0, 0);
	input_abs_set_res(chip->p_idev, ABS_X, 1080);
	input_abs_set_res(chip->p_idev, ABS_Y, 1920);
	input_set_abs_params(chip->p_idev, ABS_PRESSURE, 0, 255, 0, 0);
	chip->p_idev->open = tmg399x_prox_idev_open;
	chip->p_idev->close = tmg399x_prox_idev_close;

	dev_set_drvdata(&chip->p_idev->dev, chip);
	ret = input_register_device(chip->p_idev);
	if (ret) {
		input_free_device(chip->p_idev);
		printk(KERN_ERR"%s: cant register input '%s'\n",
				__func__, pdata->prox_name);
		goto input_p_register_failed;
	}

	ret = tmg399x_create_attr(&(tmg399x_init_info.platform_diver_addr->driver));
	if (ret)
		goto input_p_sysfs_failed;
	ret = tmg399x_add_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
	if (ret)
		goto input_p_sysfs_failed;
bypass_prox_idev:
	if (!pdata->als_name)
		goto bypass_als_idev;
	chip->a_idev = input_allocate_device();
	if (!chip->a_idev) {
		printk(KERN_ERR"%s: no memory for input_dev '%s'\n",
				__func__, pdata->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	chip->a_idev->name = pdata->als_name;
	chip->a_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->a_idev->evbit);
	set_bit(ABS_MISC, chip->a_idev->absbit);
	input_set_abs_params(chip->a_idev, ABS_MISC, 0, 65535, 0, 0);
	chip->a_idev->open = tmg399x_als_idev_open;
	chip->a_idev->close = tmg399x_als_idev_close;

	dev_set_drvdata(&chip->a_idev->dev, chip);
	ret = input_register_device(chip->a_idev);
	if (ret) {
		input_free_device(chip->a_idev);
		printk(KERN_ERR"%s: cant register input '%s'\n",
				__func__, pdata->prox_name);
		goto input_a_register_failed;
	}
	ret = tmg399x_add_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
	if (ret)
		goto input_a_sysfs_failed;

	init_timer(&chip->rgbc_timer);

bypass_als_idev:
	INIT_WORK(&chip->irq_work, tmg399x_irq_work);

	obj_ps.self=chip;
	obj_ps.polling=aphw->polling_mode_ps;
	obj_ps.sensor_operate = prx_sensor_operate;
	if((ret = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		printk(KERN_ERR"#h#j#  prox attach fail = %d\n", ret);
		goto input_a_sysfs_failed;
	}
	obj_als.self=chip;
	obj_als.polling=aphw->polling_mode_als;
	obj_als.sensor_operate = als_sensor_operate;
	if((ret = hwmsen_attach(ID_LIGHT, &obj_als)))
	{
		printk(KERN_ERR"#h#j# als attach fail = %d\n", ret);
		goto input_a_sysfs_failed;
	}
	/*
	obj_gesture.self=chip;
	obj_gesture.polling=aphw->polling_mode_gesture;
	obj_gesture.sensor_operate = gesture_sensor_operate;
	if((ret = hwmsen_attach(ID_GESTURE, &obj_gesture)))
	{
		printk(KERN_ERR"#h#j# gesture attach fail = %d\n", ret);
		goto input_a_sysfs_failed;
	}
	*/
#if 0
	ret = request_threaded_irq(client->irq, NULL, &tmg399x_irq,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			dev_name(dev), chip);
	if (ret) {
		printk( "Failed to request irq %d\n", client->irq);
		goto irq_register_fail;
	}
#else
	//use mtk way,keep interrupt for gesture
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	//	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINTF_TRIGGER_LOW, tmg399x_irq, 0);

	mt_eint_unmask(CUST_EINT_ALS_NUM);

#endif

	//	INIT_WORK(&chip->work_thresh, tmg399x_beam_thread);
    tmg399x_init_flag = 0;

	printk("Probe ok.\n");
	return 0;

irq_register_fail:
	if (chip->a_idev) {
		tmg399x_remove_sysfs_interfaces(&chip->a_idev->dev,
				als_attrs, ARRAY_SIZE(als_attrs));
input_a_sysfs_failed:
		input_unregister_device(chip->a_idev);
input_a_register_failed:
		input_free_device(chip->a_idev);
	}
input_a_alloc_failed:
	if (chip->p_idev) {
		tmg399x_remove_sysfs_interfaces(&chip->p_idev->dev,
				prox_attrs, ARRAY_SIZE(prox_attrs));
input_p_sysfs_failed:
		input_unregister_device(chip->p_idev);
input_p_register_failed:
		input_free_device(chip->p_idev);
	}
input_p_alloc_failed:
flush_regs_failed:
id_failed:
	//	kfree(chip->segment);
set_segment_failed:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
malloc_failed:
	if (powered && pdata->platform_power)
		pdata->platform_power(dev, POWER_OFF);
pon_failed:
	if (pdata->platform_teardown)
		pdata->platform_teardown(dev);
init_failed:
	printk(KERN_ERR"Probe failed.\n");
    tmg399x_init_flag = -1;
	return ret;
}

static int tmg399x_suspend(struct device *dev)
{
	return 0;
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	struct tmg399x_i2c_platform_data *pdata = dev->platform_data;

	printk("%s\n", __func__);
	mutex_lock(&chip->lock);
	chip->in_suspend = 1;

	if (chip->p_idev && chip->p_idev->users) {
		if (pdata->proximity_can_wake) {
			printk("set wake on proximity\n");
			chip->wake_irq = 1;
		} else {
			printk("proximity off\n");
			tmg399x_prox_enable(chip, 0);
		}
	}
	if (chip->a_idev && chip->a_idev->users) {
		if (pdata->als_can_wake) {
			printk("set wake on als\n");
			chip->wake_irq = 1;
		} else {
			printk("als off\n");
			tmg399x_als_enable(chip, 0);
		}
	}
	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 1);
	} else if (!chip->unpowered) {
		printk("powering off\n");
		tmg399x_pltf_power_off(chip);
	}
	mutex_unlock(&chip->lock);

	return 0;
}

static int tmg399x_resume(struct device *dev)
{
	return 0;
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool als_on, prx_on;
	int ret = 0;
	mutex_lock(&chip->lock);
	prx_on = chip->p_idev && chip->p_idev->users;
	als_on = chip->a_idev && chip->a_idev->users;
	chip->in_suspend = 0;

	printk( "%s: powerd %d, als: needed %d  enabled %d",
			__func__, !chip->unpowered, als_on,
			chip->als_enabled);
	printk(" %s: prox: needed %d  enabled %d\n",
			__func__, prx_on, chip->prx_enabled);

	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 0);
		chip->wake_irq = 0;
	}
	if (chip->unpowered && (prx_on || als_on)) {
		printk( "powering on\n");
		ret = tmg399x_power_on(chip);
		if (ret)
			goto err_power;
	}
	if (prx_on && !chip->prx_enabled)
		(void)tmg399x_prox_enable(chip, 1);
	if (als_on && !chip->als_enabled)
		(void)tmg399x_als_enable(chip, 1);
	if (chip->irq_pending) {
		printk("%s: pending interrupt\n", __func__);
		chip->irq_pending = 0;
		(void)tmg399x_check_and_report(chip);
		enable_irq(chip->client->irq);
	}
err_power:
	mutex_unlock(&chip->lock);

	return 0;
}

static int tmg399x_remove(struct i2c_client *client)
{
	struct tmg399x_chip *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
	//	free_irq(client->irq, chip);
	if (chip->a_idev) {
		tmg399x_remove_sysfs_interfaces(&chip->a_idev->dev,
				als_attrs, ARRAY_SIZE(als_attrs));
		input_unregister_device(chip->a_idev);
	}
	if (chip->p_idev) {
		tmg399x_delete_attr(&(tmg399x_init_info.platform_diver_addr->driver));
		tmg399x_remove_sysfs_interfaces(&chip->p_idev->dev,
				prox_attrs, ARRAY_SIZE(prox_attrs));
		input_unregister_device(chip->p_idev);
	}
	if (chip->pdata->platform_teardown)
		chip->pdata->platform_teardown(&client->dev);
	i2c_set_clientdata(client, NULL);
	//	kfree(chip->segment);
	kfree(chip);
	mutex_unlock(&chip->lock);
	return 0;
}


//device register
static int board_tmg399x_init(void)
{
	printk("board_tmg399x_init CALLED\n");

	//    mt_eint_unmask(CUST_EINT_ALS_NUM);
	return 0;
}

static int board_tmg399x_power(struct device *dev, enum tmg399x_pwr_state state)
{
	printk("board_tmg399x_power CALLED\n");

	//setup_pin_mux(tmg399x_pin_mux);
	return 0;
}

static void board_tmg399x_teardown(struct device *dev)
{
	printk("board_tmg399x_teardow CALLED\n");
}
static const struct lux_segment tmg399x_segment[] = {
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
};
struct tmg399x_i2c_platform_data tmg399x_data = {
	.platform_power = board_tmg399x_power,
	.platform_init = board_tmg399x_init,
	.platform_teardown = board_tmg399x_teardown,
	.prox_name = "tmg399x_proximity",
	.als_name = "tmg399x_als",
	.parameters = {
		.als_time = 0xFE, /* 5.6ms */
		.als_gain = AGAIN_64,
		.wait_time = 0xFF, /* 2.78ms */
		.prox_th_min = 0,
		.prox_th_max = 255,
		.persist = PRX_PERSIST(0) | ALS_PERSIST(0),
		.als_prox_cfg1 = 0x60,
		.prox_pulse = PPLEN_16US | PRX_PULSE_CNT(10),
		.prox_gain = PGAIN_4,
		.ldrive = PDRIVE_100MA,
		.als_prox_cfg2 = LEDBOOST_150 | 0x01,
		.prox_offset_ne = 0,
		.prox_offset_sw = 0,
		.als_prox_cfg3 = 0x00,

		.ges_entry_th = 0x20,
		.ges_exit_th = 0x10,
		.ges_cfg1 = FIFOTH_1 | GEXMSK_ALL | GEXPERS_1,
		.ges_cfg2 = GGAIN_1 | GLDRIVE_100 | GWTIME_3,
		.ges_offset_n = 0,
		.ges_offset_s = 0,
		.ges_pulse = GPLEN_16US | GES_PULSE_CNT(10),
		.ges_offset_w = 0,
		.ges_offset_e = 0,
		.ges_dimension = GBOTH_PAIR,
	},
	.beam_settings = {
		.beam_cfg  = IRBEAM_CFG,
		.beam_carr = IRBEAM_CARR,
		.beam_ns   = IRBEAM_NS,
		.beam_isd  = IRBEAM_ISD,
		.beam_np   = IRBEAM_NP,
		.beam_ipd  = IRBEAM_IPD,
		.beam_div  = IRBEAM_DIV,
	},
	.als_can_wake = false,
	.proximity_can_wake = true,
	.segment = 0,//(struct lux_segment *) tmg399x_segment,
	.segment_num = 0,//ARRAY_SIZE(tmg399x_segment),

};
static struct i2c_board_info __initdata tmg3993=
{
	I2C_BOARD_INFO("tmg3993", 0x39),
	.irq = CUST_EINT_ALS_NUM,
	.platform_data = &tmg399x_data,
};
static inline void register_boardinfo(void)
{
	struct alsps_hw *aphw= tmg399x_get_cust_alsps_hw();
	struct tmg399x_parameters *parameters=get_tmg_arglist();

	memcpy(&tmg399x_data.parameters,parameters,sizeof(struct tmg399x_parameters));

	i2c_register_board_info(aphw->i2c_num, &tmg3993, 1);
}
static int tmg399x_local_init(void)
{
   struct alsps_hw *hw = tmg399x_get_cust_alsps_hw();

	if(i2c_add_driver(&tmg399x_driver))	{
		return -1;
	}
	return 0;
}
#if 0
static int tmg399x_remove(struct platform_device *pdev)
{
	i2c_del_driver(&tmg399x_driver);
	return 0;
}
static int i2c_tmg399x_probe(struct platform_device *pdev)
{
	if(i2c_add_driver(&tmg399x_driver))	{
		return -1;
	}
	return 0;
}
static int i2c_tmg399x_remove(struct platform_device *pdev)
{
	i2c_del_driver(&tmg399x_driver);
	return 0;
}
#endif
static int tmg399x_init(void)
{
	register_boardinfo();
    alsps_driver_add(&tmg399x_init_info);
#if 0
	if(platform_driver_register(&tmg399x_alsps_driver))
	{
		return -ENODEV;
	}
#endif
	return 0;
}

static void __exit tmg399x_exit(void)
{
}

module_init(tmg399x_init);
module_exit(tmg399x_exit);

MODULE_AUTHOR("J. August Brenner<jon.brenner@ams.com>");
MODULE_AUTHOR("Byron Shi<byron.shi@ams.com>");
MODULE_DESCRIPTION("AMS-TAOS tmg3992/3 Ambient, Proximity, Gesture sensor driver");
MODULE_LICENSE("GPL");
