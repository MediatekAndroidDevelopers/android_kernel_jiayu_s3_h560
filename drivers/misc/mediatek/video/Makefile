ifneq ($(filter y,$(CONFIG_ARCH_MT8127) $(CONFIG_ARCH_MT7623) $(CONFIG_ARCH_MT8590)),)
CONFIG_MTK_PLATFORM := mt8127
endif

ifeq (y, $(filter y, $(CONFIG_ARCH_MT6570) $(CONFIG_ARCH_MT6580) $(CONFIG_ARCH_MT6755) $(CONFIG_ARCH_MT6735) $(CONFIG_ARCH_MT6735M) $(CONFIG_ARCH_MT6753)))
subdir-ccflags-y += -DCOMMON_DISP_LOG
endif

ifneq ($(CONFIG_MTK_LCM), y)
	obj-y += mtdummy/
else
	######################################
	obj-$(CONFIG_MTK_FB) += $(subst ",,$(CONFIG_MTK_PLATFORM))/
	obj-$(CONFIG_ARCH_MT6735) += common/
	obj-$(CONFIG_ARCH_MT6735M) += common/
        obj-$(CONFIG_ARCH_MT6752) += common/ #superdragonpt>import and use common part for our ARCHs
	obj-$(CONFIG_ARCH_MT6753) += common/ #superdragonpt>import and use common part for our ARCHs
	obj-$(CONFIG_ARCH_MT6570) += common/
	obj-$(CONFIG_ARCH_MT6580) += common/
	obj-$(CONFIG_ARCH_MT6755) += common/
	obj-$(CONFIG_ARCH_MT6797) += common/
	obj-$(CONFIG_ARCH_MT6757) += common/
	obj-$(CONFIG_ARCH_MT8167) += common/
	obj-$(CONFIG_ARCH_ELBRUS) += common/
endif

