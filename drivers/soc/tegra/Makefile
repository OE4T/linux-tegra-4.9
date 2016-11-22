subdir-ccflags-y += -Werror
obj-y += fuse/

obj-y += common.o
obj-y += flowctrl.o
obj-y += pmc.o
obj-y += cvb.o

obj-$(CONFIG_ARCH_TEGRA_210_SOC)	+= pm-tegra210.o
obj-$(CONFIG_TEGRA_DVFS)       += tegra-dvfs.o
obj-$(CONFIG_TEGRA_210_DVFS)   += tegra210-dvfs.o
