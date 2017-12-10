include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(MTK_CUSTOM_PATH)/videox/mt6795/
obj-$(CONFIG_MTK_LEDS) += leds_drv.o

obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/

