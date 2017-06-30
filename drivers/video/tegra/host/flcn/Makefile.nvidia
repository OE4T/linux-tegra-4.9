GCOV_PROFILE := y
ccflags-y += -I$(srctree)/../nvidia/drivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/host

ifeq ($(CONFIG_TEGRA_T19X_GRHOST),y)
ccflags-y += -I$(srctree)/../t19x/drivers/video/tegra/host/
endif

nvhost-flcn-objs  = \
	flcn.o \

obj-y += nvhost-flcn.o
