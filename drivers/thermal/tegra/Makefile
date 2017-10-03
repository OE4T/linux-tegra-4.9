obj-$(CONFIG_TEGRA_SOCTHERM)	+= tegra-soctherm.o

tegra-soctherm-y				:= soctherm.o tsensor-fuse.o
tegra-soctherm-$(CONFIG_ARCH_TEGRA_124_SOC)	+= tegra124-soctherm.o
tegra-soctherm-$(CONFIG_ARCH_TEGRA_132_SOC)	+= tegra132-soctherm.o
tegra-soctherm-$(CONFIG_ARCH_TEGRA_210_SOC)	+= tegra210-soctherm.o

obj-y += $(call tegra-path,nvidia/,drivers/thermal/tegra/)
