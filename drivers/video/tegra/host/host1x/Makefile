GCOV_PROFILE := y

EXTRA_CFLAGS += -Idrivers/video/tegra/host

nvhost-host1x-objs  = \
	host1x_syncpt.o \
	host1x_channel.o \
	host1x_intr.o \
	host1x_cdma.o \
	host1x_debug.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-host1x.o
