nvgpu-t19x := ../../../../kernel-nvgpu-t19x/drivers/gpu/nvgpu

nvgpu-y += \
	$(nvgpu-t19x)/gv11b/gv11b.o  \
	$(nvgpu-t19x)/gv11b/mc_gv11b.o  \
	$(nvgpu-t19x)/gv11b/ltc_gv11b.o  \
	$(nvgpu-t19x)/gv11b/hal_gv11b.o \
	$(nvgpu-t19x)/gv11b/gr_gv11b.o \
	$(nvgpu-t19x)/gv11b/fecs_trace_gv11b.o \
	$(nvgpu-t19x)/gv11b/fb_gv11b.o \
	$(nvgpu-t19x)/gv11b/fifo_gv11b.o \
	$(nvgpu-t19x)/gv11b/mm_gv11b.o \
	$(nvgpu-t19x)/gv11b/ce2_gv11b.o \
	$(nvgpu-t19x)/gv11b/gr_ctx_gv11b.o \
	$(nvgpu-t19x)/gv11b/pmu_gv11b.o \
	$(nvgpu-t19x)/gv11b/therm_gv11b.o

nvgpu-$(CONFIG_TEGRA_GK20A) += $(nvgpu-t19x)/gv11b/platform_gv11b_tegra.o
