#
# Makefile for NVidia Invensense nvi-mpu device.
#

obj-$(CONFIG_NVI_MPU) += nvi-mpu.o

nvi-mpu-objs := nvi.o
nvi-mpu-objs += invensense.o
nvi-mpu-objs += dmpDefaultMPU6050.o

CFLAGS_nvi.o			+= -Idrivers/iio
CFLAGS_invensense.o		+= -Idrivers/iio
CFLAGS_dmpDefaultMPU6050.o	+= -Idrivers/iio

