include $(srctree)/drivers/misc/mediatek/Makefile.custom

# $(CONFIG_MTK_PLATFORM)
$(warning "xilog project is:$(ARCH_MTK_PROJECT)")
ifeq ($(ARCH_MTK_PROJECT), tb8127p4)
	obj-y				+= ov5647_Sensor_p4.o
else
  	ifeq ($(ARCH_MTK_PROJECT), tb8127p4_25)
		obj-y				+= ov5647_Sensor_p4_25.o
	else
		ifeq ($(ARCH_MTK_PROJECT), tb8127p1_25)
			obj-y				+= ov5647_Sensor_p1_25.o
		else
			obj-y				+= ov5647_Sensor.o
		endif
	endif
endif

