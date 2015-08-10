nvgpu-t18x := ../../../../kernel-t18x/drivers/gpu/nvgpu

nvgpu-y += \
	$(nvgpu-t18x)/gp10b/gr_gp10b.o  \
	$(nvgpu-t18x)/gp10b/gr_ctx_gp10b.o  \
	$(nvgpu-t18x)/gp10b/ce2_gp10b.o \
	$(nvgpu-t18x)/gp10b/mc_gp10b.o  \
	$(nvgpu-t18x)/gp10b/fifo_gp10b.o  \
	$(nvgpu-t18x)/gp10b/ltc_gp10b.o \
	$(nvgpu-t18x)/gp10b/mm_gp10b.o \
	$(nvgpu-t18x)/gp10b/fb_gp10b.o \
	$(nvgpu-t18x)/gp10b/pmu_gp10b.o \
	$(nvgpu-t18x)/gp10b/hal_gp10b.o \
	$(nvgpu-t18x)/gp10b/rpfb_gp10b.o \
	$(nvgpu-t18x)/gp10b/gp10b_gating_reglist.o \
	$(nvgpu-t18x)/gp10b/regops_gp10b.o \
	$(nvgpu-t18x)/gp10b/cde_gp10b.o \
	$(nvgpu-t18x)/gp10b/therm_gp10b.o \
	$(nvgpu-t18x)/gp10b/fecs_trace_gp10b.o \
	$(nvgpu-t18x)/gp10b/gp10b_sysfs.o \
	$(nvgpu-t18x)/gp10b/gp10b.o

nvgpu-$(CONFIG_TEGRA_GK20A) += $(nvgpu-t18x)/gp10b/platform_gp10b_tegra.o

nvgpu-$(CONFIG_TEGRA_GR_VIRTUALIZATION) += \
	$(nvgpu-t18x)/vgpu/gp10b/vgpu_hal_gp10b.o  \
	$(nvgpu-t18x)/vgpu/gp10b/vgpu_gr_gp10b.o  \
	$(nvgpu-t18x)/vgpu/gp10b/vgpu_mm_gp10b.o \
	$(nvgpu-t18x)/vgpu/gp10b/vgpu_fifo_gp10b.o
