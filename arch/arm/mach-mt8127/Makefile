
# Makefile for the linux kernel.
#

include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/
ccflags-y += -I$(srctree)/include/linux

# Fix-me: marked others for early porting
ifeq ($(MTK_INTERNAL), yes)
ifeq ($(MTK_USE_RESERVED_EXT_MEM), yes)
KBUILD_CFLAGS += -DMTK_USE_RESERVED_EXT_MEM
endif
endif

ifeq ($(MTK_SECURITY_SW_SUPPORT), yes)
ifeq ($(MTK_SEC_FASTBOOT_UNLOCK_SUPPORT), yes)
KBUILD_CFLAGS += -DMTK_SEC_FASTBOOT_UNLOCK_SUPPORT
endif
endif

ifeq ($(MTK_TB_WIFI_3G_MODE), WIFI_ONLY)
KBUILD_CFLAGS += -DMTK_TB_WIFI_3G_MODE_WIFI_ONLY
endif

obj-y := core.o timer.o mt_dbg.o
obj-y += mt_devs.o mt_gpt.o board.o 
obj-y += ca7_timer.o
#obj-y += mt_dma.o
obj-y += mt_reg_dump.o
obj-y += mt_pm_init.o
ifeq ($(CONFIG_SMP),y)
ccflags-y += -I$(srctree)
obj-y += mt-smp.o hotplug.o mt-headsmp.o mt_hotplug.o mt_hotplug_mechanism.o
endif
# TO-FIX add # for do early porting in JB migration
obj-y += eint.o mt_freqhopping.o
obj-$(CONFIG_MT65XX_TRACER) += pmu_v7.o mon_interface.o mt_mon.o mt_emi_bm.o
obj-y += camera_isp.o
obj-y += camera_sysram.o 
obj-y += camera_pipe_mgr.o
#obj-y += camera_eis.o
#obj-y += camera_fdvt.o
ifeq ($(MTK_ALPS_BOX_SUPPORT), yes)
obj-y += mt_pm_ldo_dummy.o
else
obj-y += mt_pm_ldo.o
endif
obj-y += innercache.o mt_l2c.o

ccflags-y += -I$(MTK_PATH_PLATFORM)/drivers/power \
             -I$(MTK_PATH_PLATFORM)/drivers/gpu/mali/mali/include \
             -I$(MTK_PATH_SOURCE)/drivers/wdk
#obj-y += mt_spm.o mt_spm_sleep.o mt_spm_idle.o mt_spm_mtcmos.o
obj-y += mt_spm.o mt_spm_sleep.o mt_spm_mtcmos.o
ccflags-$(CONFIG_MTK_IN_HOUSE_TEE_SUPPORT) += -I$(srctree)/include/trustzone/
obj-y += mt_sleep.o
#obj-y += mt_cpufreq.o mt_gpufreq.o mt_cpu_ss.o
obj-y += mt_cpufreq.o mt_cpu_ss.o
obj-y += mt_emifreq.o
obj-y += mt_dormant.o cpu_dormant.o
obj-y += mt_dcm.o mt_idle.o
obj-y += mt_clkmgr_common.o
obj-y += mt_clkmgr.o
obj-y += sbchk_base.o
obj-y += mtk_ccci_helper.o
obj-y += mtk_eemcs_helper.o

ifneq ($(CONFIG_MTK_CCCI_DEVICES),y)
ccflags-y  += -D__USING_DUMMY_CCCI_API__
endif

obj-y += mt_ptp.o
obj-y += mssv.o

# Hibernation
#AFLAGS_swsusp.o	:= -DTEXT_OFFSET=$(TEXT_OFFSET)
obj-$(CONFIG_HIBERNATION)	+= cpu_hibernate.o swsusp.o

KBUILD_AFLAGS	:=$(KBUILD_AFLAGS:-msoft-float=-Wa,-mfpu=neon)

ifeq ($(MTK_ALPS_BOX_SUPPORT), yes)
ccflags-y += -DMTK_ALPS_BOX_SUPPORT
endif

# Module project related part
obj-y += $(ARCH_MTK_PROJECT)/
