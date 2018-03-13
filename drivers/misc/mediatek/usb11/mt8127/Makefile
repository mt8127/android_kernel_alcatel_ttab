obj-$(CONFIG_MTK_USBFSH) := musbfsh_core.o musbfsh_mt65xx.o

include $(srctree)/drivers/misc/mediatek/Makefile.custom
ccflags-y += -I$(srctree)/drivers/usb/host/usb11/
ccflags-y += -I$(srctree)/drivers/usb/host/usb11/$(CONFIG_MTK_PLATFORM)/
ccflags-y += -I$(srctree)/include/linux/
ccflags-y += -Werror
