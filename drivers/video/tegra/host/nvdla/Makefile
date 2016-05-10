GCOV_PROFILE := y
ccflags-y += -I../kernel-nvhost/drivers/video/tegra/host
ccflags-y += -I../kernel-nvhost-t18x/drivers/video/tegra/host
ccflags-y += -I../kernel-nvhost-t19x/drivers/video/tegra/host
ccflags-y += -I../kernel-nvhost-t19x/include/linux
ccflags-y += -Werror

nvhost-nvdla-objs  = \
		nvdla.o

obj-$(CONFIG_TEGRA_GRHOST_NVDLA) += nvhost-nvdla.o
