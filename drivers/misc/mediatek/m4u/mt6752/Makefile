include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -O0 -gdwarf-2
ccflags-y += -I$(srctree)/drivers/staging/android/ion
ccflags-y += -I$(srctree)/drivers/misc/mediatek/gud/$(MTK_PLATFORM)/gud/MobiCoreKernelApi/public
ccflags-y += -I$(srctree)/drivers/misc/mediatek/gud/$(MTK_PLATFORM)/gud/MobiCoreKernelApi/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include/trustzone/m4u
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include/mach

obj-y += m4u.o m4u_mva.o m4u_pgtable.o m4u_hw.o m4u_debug.o

