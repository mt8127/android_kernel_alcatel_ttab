# drivers/dispsys
include $(srctree)/drivers/misc/mediatek/Makefile.custom
ifeq ($(CONFIG_MTK_OVERLAY_ENGINE_SUPPORT), y)
ccflags-y += -I$(MTK_PATH_PLATFORM)/drivers/videox/
ccflags-y += -DMTK_OVERLAY_ENGINE_SUPPORT
ifeq ($(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT), y)
ccflags-y += -I$(srctree)/include -I$(srctree)/include/trustzone \
             -I$(MTK_PATH_SOURCE)/../kernel/include
ccflags-y += -DMTK_SEC_VIDEO_PATH_SUPPORT
endif
endif

ccflags-y += -I$(srctree)/drivers/misc/mediatek/video \
             -I$(srctree)/drivers/misc/mediatek/video/$(subst ",,$(CONFIG_MTK_PLATFORM)) \
             -I$(srctree)/drivers/misc/mediatek/video/$(subst ",,$(CONFIG_MTK_PLATFORM))/$(subst ",,$(CONFIG_MTK_PLATFORM)) \
             -I$(srctree)/drivers/misc/mediatek/dispsys/$(subst ",,$(CONFIG_MTK_PLATFORM)) \
     	     -I$(srctree)/drivers/staging/android/ion		\

ccflags-y += -I$(srctree)/include/trustzone/tz_cross/
ccflags-y += -I$(srctree)/include/trustzone/

#TODO: the header file under "protect-bsp" folder does not exist for customer release codebase.

obj-y += ddp_drv.o
obj-y += ddp_ovl.o
obj-y += ddp_bls.o
obj-y += ddp_color.o
# obj-y += ddp_tdshp.o
obj-y += ddp_rdma.o
obj-y += ddp_wdma.o
# obj-y += ddp_scl.o
# obj-y += ddp_rot.o
obj-y += ddp_path.o
obj-y += ddp_debug.o
obj-y += ddp_cmdq.o
obj-y += ddp_cmdq_debug.o
#obj-y += ddp_cmdq_record.o
#obj-y += ddp_cmdq_test.o
# obj-y += ddp_dpfd.o
obj-y += ddp_aal.o
obj-y += data_rgb888_64x64.o
obj-y += data_rgb888_64x64_golden.o
obj-y += ddp_cmdq_sec.o

# EOF

ifeq ($(MTK_ALPS_BOX_SUPPORT), yes)
ccflags-y += -DMTK_ALPS_BOX_SUPPORT
endif
