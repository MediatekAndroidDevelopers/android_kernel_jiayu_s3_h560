#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 1,
    .polling_mode_ps =0,
    .polling_mode_als =0,
    .polling_mode_gesture =0,

//superdragonpt modified, 20151031
//Original Values
    //.als_level = {10,50,100,150,200,400,600,1000,1500,2000,4000,6000,8000,10000,11500},
    //.als_value = {10,200,500,1000,2000,3000,4000,5000,6000,7000,8000,9000,9000,10240,10240,10240},
//More smoother ambient LUX values --> Transitions
    .als_level  = {10,20,40,60,80,100,150,200,400,600,1000,1500,2000,4000,6000,8000,10000,11500},
    .als_value  = {10,50,75,100,150,200,250,300,400,500,1000,2000,3000,4000,5000,6000,7000,8000,9000,9000,10240,10240,10240},
    .ps_threshold_high = 110,  //0~255 //Stock LP Jiayu 80
    .ps_threshold_low = 70,   //0~255 //Stock LP Jiayu 50
};
struct alsps_hw *tmg399x_get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <tmg399x.h>
struct tmg399x_parameters parameters = {
    .als_time = 0xFE, /* 5.6ms */
    .als_gain = AGAIN_64,
    .wait_time = 0xFF, /* 2.78ms */
    .prox_th_min = 0,
    .prox_th_max = 255,
    .persist = PRX_PERSIST(0) | ALS_PERSIST(0),
    .als_prox_cfg1 = 0x60,
    .prox_pulse = PPLEN_4US | PRX_PULSE_CNT(8),
    .prox_gain = PGAIN_4,
    .ldrive = PDRIVE_100MA,
    .als_prox_cfg2 = LEDBOOST_150 | 0x01,
    .prox_offset_ne = 0,
    .prox_offset_sw = 0,
    .als_prox_cfg3 = 0x00,

    .ges_entry_th = 0,
    .ges_exit_th = 255,
    .ges_cfg1 = FIFOTH_1 | GEXMSK_ALL | GEXPERS_2,
    .ges_cfg2 = GGAIN_1 | GLDRIVE_100 | GWTIME_3,
    .ges_offset_n = 0,
    .ges_offset_s = 0,
    .ges_pulse = GPLEN_32US | GES_PULSE_CNT(16),
    .ges_offset_w = 0,
    .ges_offset_e = 0,
    .ges_dimension = GBOTH_PAIR,
};
struct tmg399x_parameters *get_tmg_arglist(void)
{
	return &parameters;
}

