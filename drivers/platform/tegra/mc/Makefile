#
# Memory controller code.
#

GCOV_PROFILE := y

ccflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra \
             -I$(srctree)/drivers/platform/tegra/include

obj-y                                   += mc.o

# MC error reporting.
obj-y                                   += mcerr.o
obj-y					+= mcerr-t21.o

# Latency allowance
obj-y                                   += latency_allowance.o
obj-y                                   += tegra21x_la.o

obj-${CONFIG_TEGRA_ISOMGR}              += isomgr.o
obj-$(CONFIG_TEGRA_BWMGR)               += emc_bwmgr.o pmqos_bwmgr_client.o
obj-$(CONFIG_TEGRA_BWMGR)               += emc_bwmgr-t21x.o emc_bwmgr-t18x.o

ifeq ($(CONFIG_TEGRA_MC),y)

ifneq ($(CONFIG_ARCH_TEGRA_210_SOC),y)
obj-y += tegra_emc.o
obj-y 					+= tegra_emc_timers.o
endif

obj-y                                   += tegra-mc-sid.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) 	+= mcerr_ecc_t18x.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) 	+= mc_addr_translate.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) 	+= mcerr-t18x.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) 	+= tegra18_emc.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) 	+= tegra18x_la.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) 	+= tegra186-mc-sid.o
endif
