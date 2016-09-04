old-dtb := $(dtb-y)
dtb-y :=
makefile-path := platform/t18x/quill/kernel-dts

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
DTC_FLAGS += -i $(tegra-dtstree)/$(makefile-path)
DTCCPP_FLAGS += -I$(tegra-dtstree)/$(makefile-path)
endif

dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-quill-p3310-1000-a00-00-base.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-quill-p3310-1000-a00-00-ape-cam.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-quill-p3310-1000-a00-00-edp.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-quill-p3310-1000-a00-00-dsi-hdmi-dp.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-quill-p3310-1000-a00-00-dsi-dp.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-quill-p3310-1000-a00-00-no-rtcpu.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-quill-p3310-1000-a00-00-base-mods-display.dtb
dtb-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-quill-p3310-1000-a00-00-imx274.dtb

ifneq ($(dtb-y),)
dtb-y := $(addprefix $(makefile-path)/,$(dtb-y))
endif

dtb-y += $(old-dtb)
