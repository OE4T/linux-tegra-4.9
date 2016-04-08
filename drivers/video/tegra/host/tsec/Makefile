GCOV_PROFILE := y
ccflags-y += -I../kernel-nvhost/drivers/video/tegra/host
ccflags-y += -I../kernel-display/drivers/video/tegra/dc
ccflags-y += -Werror

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
ccflags-y += -I../kernel-nvhost-t18x/drivers/video/tegra/host/
endif

nvhost-tsec-objs  = \
	tsec.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-tsec.o
