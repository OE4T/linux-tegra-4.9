old-dtb := $(dtb-y)
dtb-y :=
makefile-path := platform/t210/batuu/kernel-dts

dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0003-p3542-0000.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0003-p3542-0000-hdmi-dsi.dtb

ifneq ($(dtb-y),)
dtb-y := $(addprefix $(makefile-path)/,$(dtb-y))
endif

dtb-y += $(old-dtb)
