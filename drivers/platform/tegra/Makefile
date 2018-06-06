#
# Makefile for linux/drivers/platform/tegra
# Tegra Platform-Specific Drivers
#

GCOV_PROFILE := y

subdir-ccflags-y += -Werror

obj-$(CONFIG_TEGRA_BOOTLOADER_DEBUG)    += tegra_bootloader_debug.o
obj-$(CONFIG_TEGRA_BOOTLOADER_DEBUG_INIT) += tegra_bootloader_debug_init.o

ifeq ($(CONFIG_TEGRA_PTM),y)
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)	+= tegra_ptm_t210.o
endif

obj-y += nvdumper/
obj-$(CONFIG_TEGRA_PMC) += pmc.o
obj-$(CONFIG_TEGRA210_BOOTROM_PMC) += tegra-bootrom-pmc-config.o

obj-y += pm_domains.o

obj-$(CONFIG_TEGRA_WAKEUP_MONITOR)      += tegra_wakeup_monitor.o

obj-y += powergate/

obj-$(CONFIG_TEGRA_THERMAL_THROTTLE)        += tegra_throttle.o

obj-$(CONFIG_PSTORE_RAM) += tegra_ramoops.o

obj-$(CONFIG_TEGRA_CPU_VOLT_CAP)	+= tegra_volt_cap.o

obj-y += bond_out.o

obj-$(CONFIG_NV_TEGRA_IVC)		+= tegra-ivc.o

obj-$(CONFIG_TEGRA_WDT_RECOVERY)        += wdt-recovery.o

obj-y					+= tegra_prod.o

obj-y					+= tegra-hv-mode.o

obj-$(CONFIG_TEGRA_FIQ_DEBUGGER)        += tegra_fiq_debugger.o

obj-$(CONFIG_TEGRA_FIRMWARES_CLASS)	+= firmwares.o
obj-$(CONFIG_TEGRA_FIRMWARES_INVENTORY)	+= firmwares-all.o

