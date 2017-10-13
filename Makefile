old-dtb := $(dtb-y)
dtb-y :=
makefile-path := platform/t19x/stardust/kernel-dts

ifeq ($(CONFIG_ARCH_TEGRA_19x_SOC),y)
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra194-p2888-1000-p2822-1000.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra194-p2888-1000-e3366-1000.dtb
endif

ifneq ($(dtb-y),)
dtb-y := $(addprefix $(makefile-path)/,$(dtb-y))
endif

dtb-y += $(old-dtb)
