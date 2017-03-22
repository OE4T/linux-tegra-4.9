GCOV_PROFILE := y

subdir-ccflags-y := -Werror
#
# Makefile for the video capture/playback device drivers.
#
obj-y				+= csi/
obj-y				+= camera/
obj-y				+= vi/
obj-$(CONFIG_VIDEO_CAMERA)	+= regmap_util.o
obj-$(CONFIG_VIDEO_TEGRA_VI_TPG) += tpg/

