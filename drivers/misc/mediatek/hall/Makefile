#
# Makefile for misc devices that really don't fit anywhere else.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

HALL_LISTS := $(subst ",,$(CUSTOM_KERNEL_HALL))
obj-y += $(foreach HALL,$(HALL_LISTS),$(HALL)/)

