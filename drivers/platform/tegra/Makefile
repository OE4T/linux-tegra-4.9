KBUILD_CFLAGS += -I$(srctree)/arch/arm/mach-tegra \
		 -I$(srctree)/arch/arm/mach-tegra/include/ \
		 -I$(srctree)/drivers/platform/tegra/

ccflags-y += -Werror
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-platform-common.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra18_speedo.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += panel-simulator.o
obj-$(CONFIG_TEGRA_HSP) += tegra-hsp.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra-mce.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra_chipid.o
obj-$(CONFIG_TEGRA_IPC) += tegra-ivc-client.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra-roc.o
obj-$(CONFIG_TEGRA186_PMC) += tegra186-pmc.o
obj-$(CONFIG_TEGRA_KFUSE) += tegra18_kfuse.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-pmc-iopower.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-aowake.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-system-config.o
obj-$(CONFIG_TEGRA_CAMERA_RTCPU) += tegra-camera-rtcpu.o
ifeq ($(CONFIG_PM_SLEEP),y)
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += pm-irq-t18x.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += wakeups-t18x.o
endif
obj-y += mc/
obj-$(CONFIG_TEGRA_AST) += tegra_ast.o
obj-y += powergate/
obj-$(CONFIG_TEGRA_AON) += tegra-aon.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra18_cpu_debugfs.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += reboot-t18x.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += pm-tegra186.o
obj-$(CONFIG_TEGRA_AON) += tegra186_aondbg.o
obj-$(CONFIG_TEGRA_CAMERA_RTCPU) += tegra-sysfs-mbox.o
obj-$(CONFIG_TEGRA_NVADSP) += nvadsp/
obj-$(CONFIG_TEGRA_ARI_MCA) += ari_mca.o
obj-$(CONFIG_TEGRA_18X_SERROR) += tegra18_serr.o
obj-$(CONFIG_TEGRA_BRIDGE_MCA) += bridge_mca.o
obj-$(CONFIG_TEGRA_A57_SERR) += tegra18_a57_serr.o
