ifdef CONFIG_ARCH_TEGRA_19x_SOC

GCOV_PROFILE := y
subdir-ccflags-y := -Werror
ccflags-y += -I$(srctree)/../nvidia/drivers/video/tegra/dc
ccflags-y += -I$(srctree)/../nvidia/drivers/video/tegra/dc/nvdisp

obj-$(CONFIG_TEGRA_NVDISPLAY) += sor_t19x.o
obj-$(CONFIG_TEGRA_NVDISPLAY) += dp_t19x.o
obj-$(CONFIG_TEGRA_NVDISPLAY) += nvdisp/

endif
