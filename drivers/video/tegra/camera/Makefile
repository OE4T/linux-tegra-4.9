GCOV_PROFILE := y
ccflags-y += -I../kernel-nvhost/drivers/video/tegra/host
ccflags-y += -Idrivers/media/platform/tegra/vi

ifdef CONFIG_TEGRA_CAMERA_PLATFORM
obj-y += tegra_camera_platform.o
obj-y += tegra_camera_dev_mfi.o
endif
