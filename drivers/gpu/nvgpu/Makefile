nvgpu-t19x := ../../../../nvgpu-t19x/drivers/gpu/nvgpu

nvgpu-y += \
	$(nvgpu-t19x)/common/mm/gmmu_t19x.o \
	$(nvgpu-t19x)/common/linux/ioctl_tsg_t19x.o \
	$(nvgpu-t19x)/gv11b/gv11b.o  \
	$(nvgpu-t19x)/gv11b/mc_gv11b.o  \
	$(nvgpu-t19x)/gv11b/ltc_gv11b.o  \
	$(nvgpu-t19x)/gv11b/hal_gv11b.o \
	$(nvgpu-t19x)/gv11b/gv11b_gating_reglist.o \
	$(nvgpu-t19x)/gv11b/gr_gv11b.o \
	$(nvgpu-t19x)/gv11b/fb_gv11b.o \
	$(nvgpu-t19x)/gv11b/fifo_gv11b.o \
	$(nvgpu-t19x)/gv11b/mm_gv11b.o \
	$(nvgpu-t19x)/gv11b/ce_gv11b.o \
	$(nvgpu-t19x)/gv11b/gr_ctx_gv11b.o \
	$(nvgpu-t19x)/gv11b/pmu_gv11b.o \
	$(nvgpu-t19x)/gv11b/acr_gv11b.o \
	$(nvgpu-t19x)/gv11b/subctx_gv11b.o \
	$(nvgpu-t19x)/gv11b/regops_gv11b.o \
	$(nvgpu-t19x)/gv100/mm_gv100.o \
	$(nvgpu-t19x)/gv100/gr_ctx_gv100.o \
	$(nvgpu-t19x)/gv100/fb_gv100.o \
	$(nvgpu-t19x)/gv100/hal_gv100.o

nvgpu-$(CONFIG_TEGRA_GK20A) += $(nvgpu-t19x)/gv11b/platform_gv11b_tegra.o
nvgpu-$(CONFIG_TEGRA_GK20A_NVHOST) += $(nvgpu-t19x)/common/linux/nvhost_t19x.o

nvgpu-$(CONFIG_TEGRA_GR_VIRTUALIZATION) += \
	$(nvgpu-t19x)/vgpu/gv11b/platform_gv11b_vgpu_tegra.o \
	$(nvgpu-t19x)/vgpu/gv11b/vgpu_hal_gv11b.o \
	$(nvgpu-t19x)/vgpu/gv11b/vgpu_gr_gv11b.o \
	$(nvgpu-t19x)/vgpu/gv11b/vgpu_fifo_gv11b.o \
	$(nvgpu-t19x)/vgpu/gv11b/vgpu_subctx_gv11b.o
