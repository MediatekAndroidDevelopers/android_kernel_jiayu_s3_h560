
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y   += core/
obj-y   += dct/dct/
endif
# Vanzo:songlixin on: Fri, 30 Jan 2015 21:23:15 +0800
# move to common dir
# obj-y	+=  camera/camera/
# End of Vanzo: songlixin

#obj-y   +=  flashlight/constant_flashlight/
obj-y   +=  vibrator/
obj-y   +=  leds/
obj-y   +=  keypad/
obj-y   +=  sound/
obj-y   +=  accdet/

ifeq ($(CONFIG_MTK_HDMI_SUPPORT),y)
obj-y	+=  hdmi/
endif

# Vanzo:wangfei on: Wed, 12 Nov 2014 21:06:42 +0800
project_name:=$(shell echo $(VANZO_INNER_PROJECT_NAME))
ifneq ($(project_name),)
$(addprefix $(TOPDIR)$(LOCAL_PATH)/,$(LOCAL_PREBUILT_OBJ_FILES))
custom_kernel_alsps:= $(addprefix alsps/,$(addsuffix /, $(shell echo $(CUSTOM_KERNEL_ALSPS))))
custom_kernel_accelerometer:= $(addprefix accelerometer/,$(addsuffix /, $(shell echo $(CUSTOM_KERNEL_ACCELEROMETER))))
custom_kernel_magnetometer:= $(addprefix magnetometer/,$(addsuffix /, $(shell echo $(CUSTOM_KERNEL_MAGNETOMETER))))
custom_kernel_gyroscope:= $(addprefix gyroscope/,$(addsuffix /, $(shell echo $(CUSTOM_KERNEL_GYROSCOPE))))
custom_kernel_barometer:= $(addprefix barometer/,$(addsuffix /, $(shell echo $(CUSTOM_KERNEL_BAROMETER))))

$(info "the  custom_kernel_alsps:$(custom_kernel_alsps),the custom_kernel_accelerometer:$(custom_kernel_accelerometer),the custom_kernel_magnetometer:$(custom_kernel_magnetometer)")
ifneq ($(custom_kernel_alsps),)
obj-y += $(custom_kernel_alsps)
endif
ifneq ($(custom_kernel_accelerometer),)
obj-y += $(custom_kernel_accelerometer)
endif
ifneq ($(custom_kernel_magnetometer),)
obj-y += $(custom_kernel_magnetometer)
endif
ifneq ($(custom_kernel_gyroscope),)
obj-y += $(custom_kernel_gyroscope)
endif
ifneq ($(custom_kernel_barometer),)
obj-y += $(custom_kernel_barometer)
endif

custom_kernel_flashlight:= $(addsuffix /, $(shell echo $(CUSTOM_KERNEL_FLASHLIGHT)))
$(info "the custom_kernel_flashlight:$(custom_kernel_flashlight)")
ifneq ($(custom_kernel_flashlight),)
obj-y += flashlight/$(custom_kernel_flashlight)
else
$(info "Warning!Why custom_kernel_flashlight is empty?")
endif

else
# End of Vanzo:wangfei
obj-$(CONFIG_CUSTOM_KERNEL_ALSPS) += alsps/
obj-$(CONFIG_CUSTOM_KERNEL_ACCELEROMETER) += accelerometer/
obj-$(CONFIG_CUSTOM_KERNEL_GYROSCOPE) += gyroscope/
obj-$(CONFIG_CUSTOM_KERNEL_MAGNETOMETER) += magnetometer/
obj-$(CONFIG_CUSTOM_KERNEL_BAROMETER) += barometer/
# Vanzo:wangfei on: Tue, 06 Jan 2015 11:03:11 +0800
# added for new sensor config structure
endif
# End of Vanzo:wangfei

ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
obj-y	+= sensorHub/
endif
