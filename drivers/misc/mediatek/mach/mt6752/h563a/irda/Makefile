#
# Makefile for misc devices that really don't fit anywhere else.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom
IRDA_LISTS := $(subst ",,$(CUSTOM_KERNEL_IRDA))
obj-y += $(foreach IRDA,$(IRDA_LISTS),$(IRDA)/)
