GCOV_PROFILE := y
EXTRA_CFLAGS += -Idrivers/video/tegra/host
ccflags-y += -Idrivers/media/platform/tegra/vi

# camera*.c and camera*.h are legacy code no longer in use
ifdef CONFIG_TEGRA_CAMERA_PLATFORM
obj-y += tegra_camera_platform.o
obj-y += tegra_camera_dev_mfi.o
endif
