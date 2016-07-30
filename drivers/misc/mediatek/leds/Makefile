include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_MTK_LEDS) += leds_drv.o

ifneq ($(strip $(VANZO_KERNEL_BREATHLIGHT_BY_VALUE)),)
CONFIG_VANZO_KERNEL_BREATHLIGHT_BY_VALUE:=$(VANZO_KERNEL_BREATHLIGHT_BY_VALUE)
endif

# Vanzo:songlixin on: Fri, 24 Apr 2015 09:34:19 +0800
vanzo_kernel_breathlight_by_value:= $(addsuffix /, $(shell echo $(CONFIG_VANZO_KERNEL_BREATHLIGHT_BY_VALUE)))
$(info "the vanzo_kernel_breathlight_by_value:$(vanzo_kernel_breathlight_by_value)")
ifneq ($(vanzo_kernel_breathlight_by_value),)
obj-y += $(vanzo_kernel_breathlight_by_value)
endif
# End of Vanzo:songlixin

obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/

