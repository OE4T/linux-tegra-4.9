GCOV_PROFILE := y
subdir-ccflags-y := -Werror
ccflags-y += -Idrivers/video/tegra
ccflags-y += -I../nvidia/drivers/video/tegra/dc
ccflags-y += -I$(srctree)/../nvidia/drivers/video/tegra/dc
ccflags-y += -I../nvidia/drivers/video/tegra/dc/panel
ccflags-y += -I$(srctree)/../nvidia/drivers/video/tegra/dc/panel
ifeq ($(CONFIG_ARCH_TEGRA_19x_SOC),y)
    ccflags-y += -I$(srctree)/../t19x/drivers/video/tegra/dc
endif

obj-y += nvdisp.o
obj-y += nvdisp_win.o
obj-y += nvdisp_stub.o
obj-y += nvdisp_cursor.o
obj-y += nvdisp_config.o
obj-y += nvdisp_sd.o
obj-y += nvdisp_bandwidth.o
obj-y += nvdisp_crc.o
obj-y += nvdisp_lut.o
obj-y += nvdisp_csc.o

