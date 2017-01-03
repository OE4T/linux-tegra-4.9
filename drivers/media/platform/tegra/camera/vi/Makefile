GCOV_PROFILE := y
ccflags-y += -I../nvhost/drivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/camera
ccflags-y += -Idrivers/media/platform/tegra
ccflags-y += -Werror

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
ccflags-y += -I../t18x/drivers/video/tegra/host/
ccflags-y += -I../nvhost/include
obj-y += mc_common.o core.o channel.o graph.o vi2_fops.o vi4_fops.o
endif

ifeq ($(CONFIG_ARCH_TEGRA_210_SOC),y)
ccflags-y += -I../nvhost/include
ccflags-y += -I../t18x/drivers/video/tegra/host/
obj-y += mc_common.o core.o channel.o graph.o vi2_fops.o
endif
