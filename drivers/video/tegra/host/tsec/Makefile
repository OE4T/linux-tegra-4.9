GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/dc
ccflags-y += -Werror

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
ccflags-y += -I../kernel-t18x/drivers/video/tegra/host/
endif

nvhost-tsec-objs  = \
	tsec.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-tsec.o
