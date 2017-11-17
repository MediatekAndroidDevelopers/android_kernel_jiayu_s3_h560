/*
* Copyright (C) 2011-2016 MediaTek Inc.
*/

#define DFT_TAG         "[WMT-MOD-INIT-EXTRA]"

#include "common_drv_init_xtra.h"

static int do_combo_common_drv_init_xtra(int chip_id)
{
	int i_ret = 0;

#ifdef MTK_WCN_COMBO_CHIP_SUPPORT
	int i_ret_tmp = 0;

	WMT_DETECT_DBG_FUNC("start to do combo driver init extra, chipid:0x%08x\n", chip_id);

	i_ret_tmp = mtk_wcn_hif_sdio_drv_init_xtra();
	i_ret += i_ret_tmp;
	WMT_DETECT_DBG_FUNC("HIF-SDIO driver init extra, i_ret:%d\n", i_ret);

	i_ret_tmp = mtk_wcn_combo_common_drv_init_xtra();
	i_ret += i_ret_tmp;
	WMT_DETECT_DBG_FUNC("COMBO COMMON driver init extra, i_ret:%d\n", i_ret);

	i_ret_tmp = mtk_wcn_stp_uart_drv_init_xtra();
	i_ret += i_ret_tmp;
	WMT_DETECT_DBG_FUNC("STP-UART driver init extra, i_ret:%d\n", i_ret);

	i_ret_tmp = mtk_wcn_stp_sdio_drv_init_xtra();
	i_ret += i_ret_tmp;
	WMT_DETECT_DBG_FUNC("STP-SDIO driver init extra, i_ret:%d\n", i_ret);

#else
	i_ret = -1;
	WMT_DETECT_ERR_FUNC("COMBO chip 6630 is not supported, please check CONFIG_MTK_COMBO_CHIP in kernel config\n");
#endif
	WMT_DETECT_DBG_FUNC("finish combo driver init extra\n");
	return i_ret;
}

static int do_soc_common_drv_init_xtra(int chip_id)
{
	int i_ret = 0;

#ifdef MTK_WCN_SOC_CHIP_SUPPORT
	int i_ret_tmp = 0;

	WMT_DETECT_DBG_FUNC("start to do soc common driver xtra init, chipid:0x%08x\n", chip_id);

	i_ret_tmp = mtk_wcn_soc_common_drv_init_xtra();
	i_ret += i_ret_tmp;
	WMT_DETECT_DBG_FUNC("COMBO COMMON driver xtra init, i_ret:%d\n", i_ret);

#else
	i_ret = -1;
	WMT_DETECT_ERR_FUNC("SOC chip 6630 is not supported, please check CONFIG_MTK_COMBO_CHIP in kernel config\n");
#endif

	WMT_DETECT_DBG_FUNC("TBD........\n");
	return i_ret;
}

int do_common_drv_init_xtra(int chip_id)
{
	int i_ret = 0;

	WMT_DETECT_INFO_FUNC("start to do common driver xtra init, chipid:0x%08x\n", chip_id);

	switch (chip_id) {
	case 0x6620:
	case 0x6628:
	case 0x6630:
		i_ret = do_combo_common_drv_init_xtra(chip_id);
		break;
	case 0x6572:
	case 0x6582:
	case 0x6592:
	case 0x6571:
	case 0x8127:
	case 0x6752:
	case 0x6735:
	case 0x8163:
	case 0x6580:
	case 0x6755:
	case 0x6797:
		i_ret = do_soc_common_drv_init_xtra(chip_id);
		break;
	}

	WMT_DETECT_INFO_FUNC("finish common driver xtra init\n");

	return i_ret;
}
