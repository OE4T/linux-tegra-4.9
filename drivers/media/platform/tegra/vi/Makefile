GCOV_PROFILE := y
ccflags-y += -I../nvhost/drivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/camera
ccflags-y += -Idrivers/media/platform/tegra/
ccflags-y += -Werror

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
ccflags-y += -I../t18x/drivers/video/tegra/host/
endif

ifneq (,$(filter $(CONFIG_VIDEO_TEGRA_VI),y m))
obj-y += vi_irq.o
obj-y += tegra_vi.o
endif

nvhost-vi-objs += vi.o
obj-$(CONFIG_VIDEO_TEGRA_VI) += nvhost-vi.o
