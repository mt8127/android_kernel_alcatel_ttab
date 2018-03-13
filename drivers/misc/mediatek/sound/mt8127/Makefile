include $(srctree)/drivers/misc/mediatek/Makefile.custom
ifeq ($(CONFIG_MTK_PMIC_MT6397),y)
  ccflags-y += -DMTK_PMIC_MT6397
	obj-$(CONFIG_MTK_SOUND) += AudDrv_Kernel.o AudDrv_Clk.o AudDrv_Afe.o AudDrv_Ana_6397.o
else
	obj-$(CONFIG_MTK_SOUND) += AudDrv_Kernel.o AudDrv_Clk.o AudDrv_Afe.o AudDrv_Ana_6323.o
endif