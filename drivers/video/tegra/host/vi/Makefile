GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/camera
ccflags-y += -Werror

obj-y += vi_irq.o
obj-y += tegra_vi.o

nvhost-vi-objs = \
		vi.o \

obj-$(CONFIG_TEGRA_GRHOST_VI) += nvhost-vi.o
