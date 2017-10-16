ifeq ($(CONFIG_ARCH_TEGRA_19x_SOC),y)
nvgpu-y += \
	common/mm/gmmu_t19x.o \
	common/linux/ioctl_tsg_t19x.o \
	common/linux/ioctl_ctrl_t19x.o \
	common/linux/io_t19x.o \
	common/linux/module_t19x.o \
	common/linux/pci_t19x.o \
	gv11b/gv11b.o  \
	gv11b/css_gr_gv11b.o  \
	gv11b/dbg_gpu_gv11b.o  \
	gv11b/mc_gv11b.o  \
	gv11b/ltc_gv11b.o  \
	gv11b/hal_gv11b.o \
	gv11b/gv11b_gating_reglist.o \
	gv11b/gr_gv11b.o \
	gv11b/fb_gv11b.o \
	gv11b/fifo_gv11b.o \
	gv11b/mm_gv11b.o \
	gv11b/ce_gv11b.o \
	gv11b/gr_ctx_gv11b.o \
	gv11b/pmu_gv11b.o \
	gv11b/acr_gv11b.o \
	gv11b/subctx_gv11b.o \
	gv11b/regops_gv11b.o \
	gv11b/therm_gv11b.o \
	gv100/mm_gv100.o \
	gv100/gr_ctx_gv100.o \
	gv100/fb_gv100.o \
	gv100/bios_gv100.o \
	gv100/fifo_gv100.o \
	gv100/gr_gv100.o \
	gv100/regops_gv100.o \
	gv100/hal_gv100.o

nvgpu-$(CONFIG_TEGRA_GK20A) += gv11b/platform_gv11b_tegra.o
nvgpu-$(CONFIG_TEGRA_GK20A_NVHOST) += common/linux/nvhost_t19x.o

nvgpu-$(CONFIG_TEGRA_GR_VIRTUALIZATION) += \
	vgpu/gv11b/platform_gv11b_vgpu_tegra.o \
	vgpu/gv11b/vgpu_hal_gv11b.o \
	vgpu/gv11b/vgpu_gr_gv11b.o \
	vgpu/gv11b/vgpu_fifo_gv11b.o \
	vgpu/gv11b/vgpu_subctx_gv11b.o \
	vgpu/gv11b/vgpu_tsg_gv11b.o
endif
