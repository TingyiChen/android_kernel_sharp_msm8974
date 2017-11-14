#
# Makefile for Invensense inv-mpu-iio device.
#
# SHMDS_ID_00102 mod S
# obj-$(CONFIG_INV_MPU_IIO) += inv-mpu-iio.o
# SHMDS_ID_0102_03 mod S
#obj-$(CONFIG_MPU_SENSORS_MPU9150) += inv-mpu-iio.o
obj-$(CONFIG_MPU_SENSORS_MPU6515) += inv-mpu-iio.o
# SHMDS_ID_0102_03 mod E
# SHMDS_ID_00102 mod E

inv-mpu-iio-objs := inv_mpu_core.o
inv-mpu-iio-objs += inv_mpu_ring.o
inv-mpu-iio-objs += inv_mpu_trigger.o
inv-mpu-iio-objs += inv_mpu_misc.o
inv-mpu-iio-objs += inv_mpu3050_iio.o
inv-mpu-iio-objs += dmpDefaultMPU6050.o
inv-mpu-iio-objs += inv_slave_compass.o
inv-mpu-iio-objs += inv_slave_pressure.o

CFLAGS_inv_mpu_core.o      += -Idrivers/staging/iio
CFLAGS_inv_mpu_ring.o      += -Idrivers/staging/iio
CFLAGS_inv_mpu_trigger.o   += -Idrivers/staging/iio
CFLAGS_inv_mpu_misc.o      += -Idrivers/staging/iio
CFLAGS_inv_mpu3050_iio.o   += -Idrivers/staging/iio
CFLAGS_dmpDefaultMPU6050.o += -Idrivers/staging/iio
CFLAGS_inv_slave_compass.o   += -Idrivers/staging/iio
CFLAGS_inv_slave_pressure.o   += -Idrivers/staging/iio

# the Bosch BMA250 driver is added to the inv-mpu device driver because it
# must be connected to an MPU3050 device on the secondary slave bus.
ifeq ($(CONFIG_INV_IIO_MPU3050_ACCEL_SLAVE_BMA250), y)
inv-mpu-iio-objs += inv_slave_bma250.o
CFLAGS_inv_slave_bma250.o   += -Idrivers/staging/iio
endif

# SHMDS_ID_1101_01 add S
EXTRA_CFLAGS += -D SHMDS_DETECT
# SHMDS_ID_1101_01 add E

# compile Invensense MPU IIO driver as DTS
ifeq ($(CONFIG_DTS_INV_MPU_IIO), y)
inv-mpu-iio-objs += inv_mpu_dts.o
CFLAGS_inv_mpu_dts.o   += -Idrivers/staging/iio
endif

