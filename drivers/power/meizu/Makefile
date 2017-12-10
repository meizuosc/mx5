include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-$(CONFIG_MZ_SMART_BATTERY) += battery_common.o

ifeq ($(CONFIG_BQ2589X_SUPPORT),y)
    obj-$(CONFIG_MZ_SMART_BATTERY) += switch_charging.o 
endif    
