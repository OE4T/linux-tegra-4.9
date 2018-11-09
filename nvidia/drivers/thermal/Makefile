ccflags-y += -I$(srctree.nvidia)/include
ccflags-y += -I$(srctree.nvidia)/drivers/thermal
ccflags-y += -I$(srctree)/drivers/thermal

obj-$(CONFIG_NV_TEGRA_BPMP) += tegra_bpmp_thermal.o
obj-$(CONFIG_TEGRA_THERMAL_THROTTLE) += tegra_throttle.o
obj-$(CONFIG_TEGRA_THERMAL_THROTTLE) += tegra_thermal_throttle.o

thermal_sys-$(CONFIG_THERMAL_GOV_PID) += pid_thermal_gov.o
