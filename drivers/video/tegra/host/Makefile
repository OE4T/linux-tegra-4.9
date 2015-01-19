GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -I../kernel-t18x/drivers/video/tegra/host
ccflags-y += -Idrivers/devfreq
ccflags-y += -Wno-multichar

subdir-ccflags-y := -Werror

obj-$(CONFIG_TEGRA_GRHOST) += t186/
obj-$(CONFIG_TEGRA_GRHOST) += nvcsi/
obj-$(CONFIG_TEGRA_GRHOST) += isp/
