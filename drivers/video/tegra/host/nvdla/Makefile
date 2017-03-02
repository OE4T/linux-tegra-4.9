GCOV_PROFILE := y
ccflags-y += -I$(srctree)/../nvhost/drivers/video/tegra/host
ccflags-y += -I$(srctree)/../nvhost-t19x/drivers/video/tegra/host
ccflags-y += -I$(srctree)/../nvhost-t19x/include
ccflags-y += -Werror

nvhost-nvdla-objs  = \
		nvdla.o \
		nvdla_ioctl.o \
		nvdla_queue.o \
		nvdla_debug.o

obj-$(CONFIG_TEGRA_GRHOST_NVDLA) += nvhost-nvdla.o
