include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/meizu/S3508/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/meizu/
ccflags-y += -DM85_S3508_GESTURE_SUPPORT

obj-y	+=  synaptics_dsx_core.o
obj-y	+=  synaptics_dsx_i2c.o
obj-y	+=  synaptics_dsx_rmi_dev.o
obj-y	+=  meizu_touchpanel_gesture_core.o
obj-y	+=  synaptics_dsx_test_reporting.o
obj-y	+=  synaptics_dsx_fw_update.o
