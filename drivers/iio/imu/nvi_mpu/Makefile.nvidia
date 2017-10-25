#
# Makefile for NVidia Invensense nvi-mpu device.
#

obj-$(CONFIG_NVI_MPU) += nvi-mpu.o

nvi-mpu-objs := nvi.o
nvi-mpu-objs += nvi_icm.o
nvi-mpu-objs += nvi_mpu.o
nvi-mpu-objs += nvi_dmp_icm.o
nvi-mpu-objs += nvi_dmp_mpu.o

CFLAGS_nvi.o			+= -Idrivers/iio
# Uncomment next line to enable TRACING
# CFLAGS_nvi.o			+= -DENABLE_TRACE -DCREATE_TRACE_POINTS
#
CFLAGS_nvi_icm.o		+= -Idrivers/iio
CFLAGS_nvi_mpu.o		+= -Idrivers/iio
CFLAGS_nvi_dmp_icm.o		+= -Idrivers/iio
CFLAGS_nvi_dmp_mpu.o		+= -Idrivers/iio

