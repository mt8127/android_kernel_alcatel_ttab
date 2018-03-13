include $(srctree)/drivers/misc/mediatek/Makefile.custom

vcodec_kernel_driver-objs :=        \
    videocodec_kernel_driver.o      \
    common/val/src/val.o			\
    common/drv/src/drv_base.o

ifeq ($(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT),y)
  ccflags-y += -DMTK_SEC_VIDEO_PATH_SUPPORT
endif

obj-$(CONFIG_MTK_VIDEOCODEC_DRIVER) += vcodec_kernel_driver.o

ccflags-y += \
    -I$(srctree)/drivers/misc/mediatek/videocodec/$(subst ",,$(CONFIG_MTK_PLATFORM))/common/drv/inc  	\
    -I$(srctree)/include/linux/vcodec                   \
    -I$(srctree)/include/linux/vcodec/kernel \
    -Wno-error=format

###ccflags-y += \
###    -I$(MTK_PATH_PLATFORM)/drivers/videocodec/common/val/inc		\
###    -I$(MTK_PATH_PLATFORM)/drivers/videocodec/common/hal/inc		\
###    -I$(MTK_PATH_PLATFORM)/drivers/videocodec/common/drv/inc		\
###    -I$(MTK_PATH_PLATFORM)/../../../kernel/include/linux/vcodec			\
###    -I$(MTK_PATH_PLATFORM)/../../../kernel/include/linux/vcodec/kernel
