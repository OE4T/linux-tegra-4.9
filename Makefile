old-dtb := $(dtb-y)
dtb-y :=
makefile-path := platform/t19x/jakku/kernel-dts

dtb-$(CONFIG_ARCH_TEGRA_19x_SOC) += tegra194-p3668-0000-p3449-0000.dtb

ifneq ($(dtb-y),)
dtb-y := $(addprefix $(makefile-path)/,$(dtb-y))
endif

dtb-y += $(old-dtb)
