GCOV_PROFILE := y
ccflags-y += -I../nvhost/drivers/video/tegra/host
ccflags-y += -I../nvhost-t18x/drivers/video/tegra/host
ccflags-y += -I../nvhost-t19x/drivers/video/tegra/host
ccflags-y += -I../nvhost-t19x/include/linux
ccflags-y += -Werror

nvhost-pva-objs  = \
		pva.o \
		pva_ioctl.o


obj-$(CONFIG_TEGRA_GRHOST) += nvhost-pva.o

