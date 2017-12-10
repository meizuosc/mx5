include $(srctree)/drivers/misc/mediatek/Makefile.custom
#
# Makefile for Invensense inv-mpu-iio/inv-mpu-iio-spi device.
#
ifeq ($(CONFIG_INV_MPU_IIO),y)
obj-y += inv_mpu_i2c.o
obj-y += inv_mpu_core.o
obj-y += inv_mpu_init.o
obj-y += inv_mpu_ring.o
obj-y += inv_mpu_parsing.o
obj-y += inv_mpu_misc.o
obj-y += inv_mpu_load_dmp.o
obj-y += inv_mpu_common.o
obj-y += inv_mpu_selftest.o
obj-y += inv_mpu_setup.o
obj-y += inv_slave_compass.o
obj-y += inv_slave_pressure.o
obj-y += inv_slave_als.o
obj-y += inv_custom_board.o

# compile Invensense MPU IIO driver as DTS
ifeq ($(CONFIG_DTS_INV_MPU_IIO), y)
obj-y += inv_mpu_dts.o
endif
endif
ifeq ($(CONFIG_INV_MPU_IIO_SPI),y)
obj-y += inv_mpu_spi.o
obj-y += inv_mpu_core.o
obj-y += inv_mpu_init.o
obj-y += inv_mpu_ring.o
obj-y += inv_mpu_parsing.o
obj-y += inv_mpu_misc.o
obj-y += inv_mpu_load_dmp.o
obj-y += inv_mpu_common.o
obj-y += inv_mpu_selftest.o
obj-y += inv_mpu_setup.o
obj-y += inv_slave_compass.o
obj-y += inv_slave_pressure.o
obj-y += inv_slave_als.o
obj-y += inv_custom_board.o
endif

ccflags-y += -DLINUX_KERNEL_3_10
