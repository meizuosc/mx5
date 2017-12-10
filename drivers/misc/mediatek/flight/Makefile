#
# Makefile for misc devices that really don't fit anywhere else.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifeq ($(CONFIG_ARCH_MT6592),y)
else 
	ifeq ($(CONFIG_ARCH_MT6582),y)
	else 
	    ifeq ($(CONFIG_ARCH_MT6752),y)
	    else
	        obj-$(CONFIG_MTK_FLIGHT_STMVL6180_SUPPORT) += mt6795/stmvl6180/
	    endif
	endif
endif
