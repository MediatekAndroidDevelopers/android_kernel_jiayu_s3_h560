include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/goodix_hotknot/
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/goodix_hotknot/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  goodix_tool.o
obj-y	+=  goodix_driver.o
obj-y	+=  goodix_update.o

