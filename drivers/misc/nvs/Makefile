#
# Makefile for NVS (NVidia Sensor framework).
#
obj-$(CONFIG_NVS) += nvs_vreg.o nvs_of_dt.o nvs_timestamp.o nvs_dsm.o \
		     nvs_auto.o
obj-$(CONFIG_NVS_LIGHT) += nvs_light.o
obj-$(CONFIG_NVS_PROXIMITY) += nvs_proximity.o
obj-$(CONFIG_NVS_IIO) += nvs_iio.o
obj-$(CONFIG_NVS_INPUT) += nvs_input.o nvs_sysfs.o
obj-$(CONFIG_NVS_RELAY) += nvs_relay.o nvs_sysfs.o

ifeq ($(CONFIG_NVS_RELAY), y)
	CFLAGS_nvs_dsm.o += -DNVS_KIF_RELAY=1
	CFLAGS_nvs_auto.o += -DNVS_KIF_RELAY=1
endif
ifeq ($(CONFIG_NVS_IIO), y)
	CFLAGS_nvs_dsm.o += -DNVS_KIF_IIO=1
	CFLAGS_nvs_auto.o += -DNVS_KIF_IIO=1
endif
ifeq ($(CONFIG_NVS_INPUT), y)
	CFLAGS_nvs_dsm.o += -DNVS_KIF_INPUT=1
	CFLAGS_nvs_auto.o += -DNVS_KIF_INPUT=1
endif

