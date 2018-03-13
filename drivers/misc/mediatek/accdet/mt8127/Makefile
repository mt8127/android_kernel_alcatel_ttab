include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_MTK_ACCDET) := accdet.o

ccflags-y += -I$(MTK_PATH_PLATFORM)/drivers/power\
             -I$(MTK_PATH_SOURCE)/drivers/power\


ifeq ($(MTK_ALPS_BOX_SUPPORT),yes)
ccflags-y += -DMTK_ALPS_BOX_SUPPORT
endif