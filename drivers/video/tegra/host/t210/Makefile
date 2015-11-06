GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/camera
ccflags-y += -Idrivers/media/platform/tegra/vi
ccflags-y += -Idrivers/video/tegra/host/t124
ccflags-y += -Wno-multichar
ccflags-y += -Werror

nvhost-t210-objs  = \
        t210.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-t210.o
