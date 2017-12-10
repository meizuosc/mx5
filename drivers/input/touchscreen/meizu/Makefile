include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/meizu/S3508/src/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/meizu/S3508/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/meizu/

ifeq ($(CONFIG_TOUCHSCREEN_MEIZU_S3508),y)
obj-y	+=  S3508/
endif

ifeq ($(CONFIG_TOUCHSCREEN_MEIZU_FTS3AD54),y)
obj-y	+=  FTS3AD54/
endif

obj-y   +=  mtk_tpd.o
obj-y   +=  tpd_button.o
obj-y   +=  tpd_calibrate.o
obj-y   +=  tpd_debug.o
obj-y   +=  tpd_default.o
obj-y   +=  tpd_init.o
obj-y   +=  tpd_misc.o
obj-y   +=  tpd_setting.o
obj-y   +=  touch_filter.o

