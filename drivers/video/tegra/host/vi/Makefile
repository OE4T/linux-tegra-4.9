GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/camera
ccflags-y += -Werror

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
ccflags-y += -I../kernel-t18x/drivers/video/tegra/host/
endif

ifneq (,$(filter $(CONFIG_TEGRA_GRHOST_VI),y m))
ifneq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
obj-y += vi_irq.o
endif
obj-y += tegra_vi.o
endif

obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += \
	../../../../../../kernel-t18x/drivers/video/tegra/host/vi/host_vi_notify.o

nvhost-vi-objs = \
		vi.o \

obj-$(CONFIG_TEGRA_GRHOST_VI) += nvhost-vi.o
