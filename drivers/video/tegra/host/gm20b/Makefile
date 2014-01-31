GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Wno-multichar

nvhost-gm20b-objs  = \
	hal_gm20b.o \
	ltc_gm20b.o \
	gr_gm20b.o \
	gr_gm20b.o \
	fb_gm20b.o

obj-$(CONFIG_TEGRA_GK20A) += nvhost-gm20b.o
