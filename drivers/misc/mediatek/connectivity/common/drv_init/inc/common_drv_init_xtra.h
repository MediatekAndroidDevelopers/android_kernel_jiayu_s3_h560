/*
* Copyright (C) 2011-2016 MediaTek Inc.
*/

extern int do_common_drv_init_xtra(int chip_id);

/*defined in common xtra combo mtk 3.18*/
#ifdef MTK_WCN_COMBO_CHIP_SUPPORT
extern int mtk_wcn_combo_common_drv_init_xtra(void);
extern int mtk_wcn_hif_sdio_drv_init_xtra(void);
extern int mtk_wcn_stp_uart_drv_init_xtra(void);
extern int mtk_wcn_stp_sdio_drv_init_xtra(void);
#endif

#ifdef MTK_WCN_SOC_CHIP_SUPPORT
extern int mtk_wcn_soc_common_drv_init_xtra(void);
#endif

#endif

