/*
* Copyright (C) 2011-2016 MediaTek Inc.
*/

#ifdef DFT_TAG
#undef DFT_TAG
#endif
#define DFT_TAG         "[GPS-MOD-INIT]"

#include "gps_drv_init_xtra.h"

int do_gps_drv_init_xtra(int chip_id)
{
	int i_ret = -1;
#ifdef CONFIG_MTK_COMBO_GPS
	WMT_DETECT_INFO_FUNC("start to do gps xtra driver init\n");
	i_ret = mtk_wcn_stpgps_drv_init_xtra();
	WMT_DETECT_INFO_FUNC("finish gps xtra driver init, i_ret:%d\n", i_ret);
#else
	WMT_DETECT_INFO_FUNC("CONFIG_MTK_COMBO_GPS second solution is not defined\n");
#endif
	return i_ret;

}

