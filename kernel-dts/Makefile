old-dtb := $(dtb-y)
dtb-y :=
makefile-path := platform/t210/porg/kernel-dts

dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-porg-p3448-0000-a00.dtb

ifneq ($(dtb-y),)
dtb-y := $(addprefix $(makefile-path)/,$(dtb-y))
endif

dtb-y += $(old-dtb)
