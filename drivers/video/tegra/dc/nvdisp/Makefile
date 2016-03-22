GCOV_PROFILE := y
subdir-ccflags-y := -Werror
ccflags-y += -Idrivers/video/tegra -I../kernel-display/drivers/video/tegra/dc -I$(srctree)/arch/arm/mach-tegra
obj-y += nvdisp.o
obj-y += nvdisp_win.o
obj-y += nvdisp_stub.o
obj-y += nvdisp_cursor.o
obj-y += nvdisp_config.o
obj-y += nvdisp_sd.o
obj-y += nvdisp_bandwidth.o
obj-$(CONFIG_TEGRA_LUT_V2) += nvdisp_lut.o
obj-$(CONFIG_TEGRA_CSC_V2) += nvdisp_csc.o

