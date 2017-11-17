ifeq ($(CONFIG_MTK_COMBO),y)
	ccflags-y += -I$(src)/inc/
	ccflags-y += -I$(src)/../

ifneq ($(filter "MT6630",$(CONFIG_MTK_COMBO_CHIP)),)
	ccflags-y += -D MTK_WCN_WLAN_GEN3
endif
ifneq ($(filter "CONSYS_6752",$(CONFIG_MTK_COMBO_CHIP)),)
	ccflags-y += -D MTK_WCN_WLAN_GEN3
else ifneq ($(filter "CONSYS_%",$(CONFIG_MTK_COMBO_CHIP)),)
	ccflags-y += -D MTK_WCN_WLAN_GEN2
endif

	obj-y += gps_drv_init_xtra.o
        #obj-y += gps_drv_sdio.o //superdragonpt not for here
        #obj-y += gps_drv_6630.o //unused


endif
