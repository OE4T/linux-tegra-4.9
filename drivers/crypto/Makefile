GCOV_PROFILE := y

ccflags-y += -I$(srctree)/../kernel-nvhost-t18x/drivers/video/tegra/host
ccflags-y += -I$(srctree)/../kernel-nvhost/drivers/video/tegra/host
ccflags-y += -I$(srctree)/../kernel-nvhost/include
obj-$(CONFIG_CRYPTO_DEV_TEGRA_SE_USE_HOST1X_INTERFACE) += tegra-se-nvhost.o

obj-$(CONFIG_CRYPTO_DEV_TEGRA_ELLIPTIC_SE) += elliptic/
