GCOV_PROFILE := y
ccflags-y += -I../nvhost/drivers/video/tegra/host
ccflags-y += -Idrivers/media/platform/tegra
ccflags-y += -Werror

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
ccflags-y += -I../t18x/drivers/video/tegra/host
nvhost-vi-tpg-objs += tpg_t18x.o
else
nvhost-vi-tpg-objs += tpg_t21x.o
endif


obj-$(CONFIG_VIDEO_TEGRA_VI_TPG) += nvhost-vi-tpg.o
