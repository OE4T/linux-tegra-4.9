GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
ccflags-y += -I../kernel-t18x/drivers/video/tegra/host/
endif

nvhost-flcn-objs  = \
	flcn.o \

obj-y += nvhost-flcn.o
