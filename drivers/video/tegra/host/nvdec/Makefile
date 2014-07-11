GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Iarch/arm/mach-tegra

nvhost-nvdec-objs  = \
	nvdec.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-nvdec.o
