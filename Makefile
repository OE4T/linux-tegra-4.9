old-dtb := $(dtb-y)
old-dtbo := $(dtbo-y)
dtb-y :=
dtbo-y :=
makefile-path := platform/t19x/galen-industrial/kernel-dts

dtb-$(CONFIG_ARCH_TEGRA_19x_SOC) += tegra194-p2888-0008-e3366-1199.dtb
dtb-$(CONFIG_ARCH_TEGRA_19x_SOC) += tegra194-p2888-0008-p2822-0000.dtb
dtb-$(CONFIG_ARCH_TEGRA_19x_SOC) += tegra194-p2888-0008-p2822-0000-maxn.dtb
dtb-$(CONFIG_ARCH_TEGRA_19x_SOC) += tegra194-p2888-0008-p2822-0000-noecc.dtb
dtb-$(CONFIG_ARCH_TEGRA_19x_SOC) += tegra194-p2888-0008-p2822-0000-safety.dtb

ifneq ($(dtb-y),)
dtb-y := $(addprefix $(makefile-path)/,$(dtb-y))
endif
ifneq ($(dtbo-y),)
dtbo-y := $(addprefix $(makefile-path)/,$(dtbo-y))
endif

dtb-y += $(old-dtb)
dtbo-y += $(old-dtbo)
