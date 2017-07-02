GCOV_PROFILE := y
ccflags-y += -I$(srctree)/../nvidia/drivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/camera
ccflags-y += -Idrivers/media/platform/tegra/camera
#ccflags-y += -I../t18x/drivers/video/tegra/host
ccflags-y += -I$(srctree)/../t19x/drivers/video/tegra/host
ccflags-y += -I$(srctree)/../t19x/include
ccflags-y += -Wno-multichar
ccflags-y += -Werror

nvhost-t194-objs  = \
        t194.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-t194.o
