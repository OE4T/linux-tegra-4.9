GCOV_PROFILE := y
EXTRA_CFLAGS += -Idrivers/video/tegra/host

# camera*.c and camera*.h are legacy code no longer in use
ifdef CONFIG_TEGRA_CAMERA_PLATFORM
obj-y += tegra_camera_platform.o
endif
