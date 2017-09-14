GCOV_PROFILE_pinctrl-tegra186-padctl.o := y

ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/pinctrl
ccflags-y += -I$(srctree)/drivers/pinctrl/tegra
ccflags-y += -I$(srctree)/drivers/video/tegra/host
ccflags-y += -I$(srctree)/../nvidia/drivers/video/tegra/host
ccflags-y += -I$(srctree)/../nvidia/include

obj-$(CONFIG_PINCTRL_TEGRA186)  += pinctrl-tegra186.o
obj-$(CONFIG_PINCTRL_TEGRA186_PADCTL) += pinctrl-tegra186-padctl.o
obj-$(CONFIG_PINCTRL_TEGRA186_DPAUX)  += pinctrl-tegra186-dpaux.o
