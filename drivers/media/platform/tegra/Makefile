GCOV_PROFILE := y

#subdir-ccflags-y := -Werror
#
# Makefile for the video capture/playback device drivers.
#
ifndef CONFIG_ARCH_TEGRA_210_SOC
obj-y	+= camera/
endif
obj-y	+= vi/
obj-$(CONFIG_VIDEO_CAMERA)	+= regmap_util.o
obj-$(CONFIG_VIDEO_TEGRA_VI_TPG) += tpg/

obj-$(CONFIG_TEGRA_MIPI_CAL)	+= mipical/
