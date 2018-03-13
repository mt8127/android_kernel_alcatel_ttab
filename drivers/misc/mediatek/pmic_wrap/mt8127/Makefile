include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y +=  pwrap_hal.o

ifeq ($(MTK_ALPS_BOX_SUPPORT), yes)
ccflags-y += -DMTK_ALPS_BOX_SUPPORT
endif
