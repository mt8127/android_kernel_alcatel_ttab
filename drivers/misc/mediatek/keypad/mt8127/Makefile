include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(CONFIG_MTK_PMIC_MT6397),y)
  ccflags-y += -DMTK_PMIC_MT6397
endif

obj-$(CONFIG_MTK_KEYPAD) := hal_kpd.o
