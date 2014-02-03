GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/camera
ccflags-y += -Werror

nvhost-vi-objs  = \
		vi.o

obj-y += tegra_vi.o
obj-$(CONFIG_TEGRA_GRHOST_VI) += nvhost-vi.o
