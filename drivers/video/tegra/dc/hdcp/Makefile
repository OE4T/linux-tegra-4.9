GOV_PROFILE := y
subdir-ccflags-y := -Werror
ccflags-y += -I$(srctree)/../nvidia/drivers/video/tegra/host
ccflags-y += -I$(srctree)/../nvidia/drivers/video/tegra/dc
ccflags-y += -I$(srctree)/../nvidia/drivers/platform/tegra
ccflags-y += -I$(srctree)/arch/arm/mach-tegra

ifneq ($(CONFIG_TEGRA_NVDISPLAY),y)
obj-$(CONFIG_TEGRA_HDMIHDCP) += hdmihdcp.o
obj-$(CONFIG_TEGRA_HDMIHDCP) += nvhdcp_hdcp22_methods.o
obj-$(CONFIG_TEGRA_DPHDCP) += dphdcp.o
ccflags-$(CONFIG_TEGRA_HDMIHDCP) += -DCONFIG_HDCP
ccflags-$(CONFIG_TEGRA_DPHDCP) += -DCONFIG_DPHDCP
endif
ifeq ($(CONFIG_TEGRA_NVDISPLAY),y)
ifeq ($(CONFIG_TRUSTY),y)
obj-$(CONFIG_TEGRA_HDMIHDCP) += hdmihdcp.o
obj-$(CONFIG_TEGRA_HDMIHDCP) += nvhdcp_hdcp22_methods.o
obj-$(CONFIG_TEGRA_DPHDCP) += dphdcp.o
ccflags-$(CONFIG_TEGRA_HDMIHDCP) += -DCONFIG_HDCP
ccflags-$(CONFIG_TEGRA_DPHDCP) += -DCONFIG_DPHDCP
endif
endif
