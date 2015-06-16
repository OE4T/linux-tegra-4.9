ccflags-$(CONFIG_DMADEVICES_DEBUG)  := -DDEBUG
ccflags-$(CONFIG_DMADEVICES_VDEBUG) += -DVERBOSE_DEBUG

ccflags-y += -Werror
ccflags-$(CONFIG_DMADEVICES) += -I$(srctree)/drivers/dma
obj-$(CONFIG_TEGRA186_GPC_DMA) += tegra186-gpc-dma.o
# The funky manipulation of $(srctree) is for dynamically finding the name of
# the kernel root dir.
obj-$(CONFIG_TEGRA186_ADMA) += ../../../$(lastword $(subst /, ,$(srctree)))/drivers/dma/tegra210-adma.o
