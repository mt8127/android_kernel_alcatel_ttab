include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/include/trustzone
ccflags-y += -I$(srctree)/drivers/misc/mediatek/smi
ccflags-y += -I$(srctree)/drivers/misc/mediatek/smi/$(subst ",,$(CONFIG_MTK_PLATFORM))
ccflags-y += -I$(srctree)/drivers/misc/mediatek/dispsys/$(subst ",,$(CONFIG_MTK_PLATFORM))

obj-y += m4u.o m4u_kernel.o m4u_priv.o
#obj-y += m4u_slt.o
