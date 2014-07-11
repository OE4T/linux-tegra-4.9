GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/camera
ccflags-y += -Werror

ifneq (,$(filter $(CONFIG_TEGRA_GRHOST_VI),y m))
obj-y += vi_irq.o
obj-y += tegra_vi.o
endif

nvhost-vi-objs = \
		vi.o \

obj-$(CONFIG_TEGRA_GRHOST_VI) += nvhost-vi.o
