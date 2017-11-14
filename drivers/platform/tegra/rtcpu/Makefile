ccflags-y := -Werror
ccflags-y += -I$(srctree)/include
ccflags-y += -I$(srctree)
ccflags-y += -I$(srctree.nvidia)
ccflags-y += -I$(srctree.nvidia)/drivers/platform/tegra/rtcpu

obj-y += debug.o
obj-y += tegra-sysfs-mbox.o
obj-y += vi-notify.o
obj-y += hsp-mbox.o
obj-y += tegra-rtcpu-trace.o
obj-y += camchar.o
obj-y += clk-group.o
obj-y += device-group.o
obj-y += reset-group.o
obj-y += rtcpu-monitor.o
obj-y += tegra-ivc-rpc.o tegra-ivc-rpc-test.o
obj-y += capture-ivc.o
obj-y += syncpt-display-channel.o

ifdef CONFIG_ARCH_TEGRA_19x_SOC
obj-y += tegra-rce-rm.o
endif
