nvgpu-t19x := ../../../../kernel-nvgpu-t19x/drivers/gpu/nvgpu

nvgpu-y += \
	$(nvgpu-t19x)/gv11b/mc_gv11b.o  \
	$(nvgpu-t19x)/gv11b/hal_gv11b.o \
	$(nvgpu-t19x)/gv11b/gr_gv11b.o

nvgpu-$(CONFIG_TEGRA_GK20A) += $(nvgpu-t19x)/gv11b/platform_gv11b_tegra.o
