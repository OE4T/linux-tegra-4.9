#
# Makefile for linux/drivers/platform/tegra
# Tegra Platform-Specific Drivers
#

GCOV_PROFILE := y

subdir-ccflags-y += -Werror
subdir-ccflags-y += -I$(srctree)/drivers/platform/tegra/include \
		    -I$(srctree)/../nvidia/drivers/platform/tegra/ \
		    -I$(srctree)/drivers/video/tegra/dc/panel/ \
		    -I$(srctree)/arch/arm/mach-tegra

ifeq ($(CONFIG_ARCH_TEGRA_21x_SOC),y)
obj-$(CONFIG_USB_SUPPORT)		+= tegra_usb_pmc.o
endif
obj-$(CONFIG_TEGRA_BOOTLOADER_DEBUG)    += tegra_bootloader_debug.o
obj-$(CONFIG_TEGRA_BOOTLOADER_DEBUG_INIT) += tegra_bootloader_debug_init.o

obj-y					+= tegra_vpr.o
obj-y					+= smc-calls.o

# The wildcard hack is necessary for kernel's make clean.
ifneq ($(wildcard $(srctree)/../nvidia/drivers/platform/tegra/nvadsp/Makefile*),)
obj-$(CONFIG_TEGRA_NVADSP) += ../../../../nvidia/drivers/platform/tegra/nvadsp/
endif


obj-$(CONFIG_TEGRA_MC) += mc/

ifeq ($(CONFIG_TEGRA_PTM),y)
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)         += tegra_ptm_t210.o
endif

obj-y += nvdumper/
obj-$(CONFIG_TEGRA_PMC) += pmc.o
obj-$(CONFIG_TEGRA210_BOOTROM_PMC) += tegra-bootrom-pmc-config.o
obj-$(CONFIG_PM) += pm.o
obj-$(CONFIG_PM) += pm_debugfs.o

obj-y += pm_domains.o

obj-$(CONFIG_TEGRA_WAKEUP_MONITOR)      += tegra_wakeup_monitor.o

obj-y += powergate/

obj-$(CONFIG_TEGRA_THERMAL_THROTTLE)        += tegra_throttle.o

obj-$(CONFIG_TEGRA_CLUSTER_CONTROL) += sysfs-cluster.o
obj-$(CONFIG_TEGRA_HMP_CLUSTER_CONTROL) += sysfs-cluster.o

obj-$(CONFIG_PSTORE_RAM) += tegra_ramoops.o

obj-$(CONFIG_TEGRA_CENTRAL_ACTMON) += central_actmon/
obj-$(CONFIG_TEGRA_CPU_VOLT_CAP)	+= tegra_volt_cap.o

obj-$(CONFIG_PM_SLEEP) += tegra-wakeups.o

ccflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra
asflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra





obj-y += bond_out.o

obj-$(CONFIG_TEGRA_HMP_CLUSTER_CONTROL)      += cluster-control.o

obj-$(CONFIG_TEGRA_IVC)			+= tegra-ivc.o

obj-y					+= mselect.o

obj-$(CONFIG_TEGRA_WDT_RECOVERY)        += wdt-recovery.o

obj-y					+= tegra_prod.o

obj-y					+= tegra-hv-mode.o

obj-$(CONFIG_TEGRA_FIQ_DEBUGGER)        += tegra_fiq_debugger.o

obj-$(CONFIG_TEGRA_FIRMWARES_CLASS)	+= firmwares.o
obj-$(CONFIG_TEGRA_FIRMWARES_INVENTORY)	+= firmwares-all.o
obj-$(CONFIG_TEGRA_HV_XHCI_DEBUG)	+= tegra-hv-xhci-debug.o

obj-$(CONFIG_TEGRA_HSP)			+= tegra-hsp.o
obj-$(CONFIG_TEGRA_HSP)			+= tegra186-hsp.o

obj-y					+= tegra_fd.o
