#
# Memory controller code.
#

GCOV_PROFILE := y

ccflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra \
             -I$(srctree)/drivers/platform/tegra/include

obj-y                                   += mc.o

obj-y                                   += mcerr.o
obj-y                                   += mcerr-t21.o

obj-y                                   += latency_allowance.o
obj-y                                   += tegra21x_la.o

obj-$(CONFIG_TEGRA_ISOMGR)              += isomgr.o
obj-$(CONFIG_TEGRA_BWMGR)               += emc_bwmgr.o pmqos_bwmgr_client.o
obj-$(CONFIG_TEGRA_BWMGR)               += emc_bwmgr-t21x.o emc_bwmgr-t18x.o

obj-y                                   += tegra_emc.o
obj-y                                   += tegra_emc_timers.o

obj-y                                   += tegra-mc-sid.o
obj-y                                   += mcerr_ecc_t18x.o
obj-y                                   += mc_addr_translate.o
obj-y                                   += mcerr-t18x.o
obj-y                                   += tegra18_emc.o
obj-y                                   += tegra18x_la.o
obj-y                                   += tegra186-mc-sid.o
