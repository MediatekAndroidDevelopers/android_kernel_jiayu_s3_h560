#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 1,
    .polling_mode_ps =0,
    .polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    //.i2c_addr   = {0x0C, 0x48, 0x78, 0x00},
    .als_level  = {5,20,50,100,200,300,500,800,1200,2000,3000,4000,5000,10000,65535},
    .als_value  = {300,800,1200,1800,2500,3000,3500,4000,5000,6000,7000,8000,9000,9000,10240,10240},
    .ps_threshold_high = 400,
    .ps_threshold_low = 200,
};
struct alsps_hw *TMD2772_get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}

int TMD2772_CMM_PPCOUNT_VALUE = 0x05;
int TMD2772_CMM_CONTROL_VALUE = 0x2C;
int TMD2772_ZOOM_TIME = 4;

