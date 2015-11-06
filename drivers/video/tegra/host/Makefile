GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Idrivers/devfreq
ccflags-y += -Wno-multichar

subdir-ccflags-y := -Werror

nvhost-objs = \
	nvhost_acm.o \
	nvhost_syncpt.o \
	nvhost_cdma.o \
	nvhost_intr.o \
	nvhost_channel.o \
	nvhost_job.o \
	dev.o \
	debug.o \
	bus_client.o \
	chip_support.o \
	nvhost_scale.o \
	nvhost_vm.o \
	scale_emc.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost.o
obj-$(CONFIG_TEGRA_GRHOST) += host1x/
obj-$(CONFIG_TEGRA_GRHOST) += tsec/
obj-$(CONFIG_TEGRA_GRHOST) += nvdec/
obj-$(CONFIG_TEGRA_GRHOST) += isp/
obj-$(CONFIG_TEGRA_GRHOST) += t124/
obj-$(CONFIG_TEGRA_GRHOST) += flcn/
obj-$(CONFIG_TEGRA_GRHOST) += t210/

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
obj-$(CONFIG_TEGRA_GRHOST) += ../../../../../kernel-t18x/drivers/video/tegra/host/
endif

obj-$(CONFIG_TEGRA_GRHOST_SYNC) += nvhost_sync.o

obj-$(CONFIG_TEGRA_GRHOST) += vhost/
