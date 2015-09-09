include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors
obj-y   +=  barometer.o barometer_factory.o
# Vanzo:wangfei on: Wed, 12 Nov 2014 21:06:42 +0800
project_name:=$(shell echo $(VANZO_INNER_PROJECT_NAME))

ifneq ($(project_name),)
custom_kernel_barometer:= $(addsuffix /, $(shell echo $(CUSTOM_KERNEL_BAROMETER)))
ifneq ($(custom_kernel_barometer),)
obj-y += $(custom_kernel_barometer)
endif
else
# End of Vanzo:wangfei
ifeq ($(CONFIG_MTK_BMP180_NEW),y)
obj-y += BMP180-new/
endif
# Vanzo:wangfei on: Tue, 06 Jan 2015 10:34:15 +0800
# added for new structure config barometer
endif
# End of Vanzo:wangfei
