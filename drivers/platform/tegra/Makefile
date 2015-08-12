KBUILD_CFLAGS += -I$(srctree)/arch/arm/mach-tegra -I$(srctree)/arch/arm/mach-tegra/include/
ccflags-y += -Werror
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra18_speedo.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += panel-simulator.o
obj-$(CONFIG_TEGRA_HSP) += tegra-hsp.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra-mce.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra_chipid.o
obj-$(CONFIG_TEGRA_IPC) += tegra-ivc-client.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra-roc.o
obj-$(CONFIG_TEGRA186_PMC) += tegra186-pmc.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-pmc-iopower.o
ifeq ($(CONFIG_PM_SLEEP),y)
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += pm-irq-t18x.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += wakeups-t18x.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += pm-tegra186.o
endif
obj-y += mc/
obj-$(CONFIG_TEGRA_AST) += tegra_ast.o
obj-y += powergate/
obj-$(CONFIG_TEGRA_AON) += tegra-aon.o
