#
# Makefile for industrial I/O pressure drivers
#

# When adding new entries keep the list in alphabetical order

obj-$(CONFIG_NVI_BMPX80) += nvi-bmpX80.o
ifdef CONFIG_NVI_MPU
	CFLAGS_nvi_bmpX80.o += -DBMP_NVI_MPU_SUPPORT=1
endif
nvi-bmpX80-objs += nvi_bmpX80.o
