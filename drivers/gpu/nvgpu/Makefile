GCOV_PROFILE := y

ccflags-y += -I$(srctree.nvgpu)/drivers/gpu/nvgpu/include
ccflags-y += -I$(srctree.nvgpu)/drivers/gpu/nvgpu
ccflags-y += -I$(srctree.nvgpu)/drivers/gpu/nvgpu/common
ccflags-y += -I$(srctree.nvgpu)/include
ccflags-y += -I$(srctree.nvgpu)/include/uapi
ccflags-y += -I$(srctree.nvgpu-next)/drivers/gpu/nvgpu

ccflags-y += -I$(srctree)/drivers/devfreq

ccflags-y += -Wno-multichar
ccflags-y += -Werror
ccflags-y += -Wno-error=cpp
ifeq ($(VERSION),4)
ccflags-y += -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
endif

# Turn off when this is fixed upstream, if ever.
ccflags-y += -D__NVGPU_PREVENT_UNTRUSTED_SPECULATION

obj-$(CONFIG_GK20A) := nvgpu.o

# OS independent parts of nvgpu. The work to collect files here
# is in progress.

nvgpu-y += common/bus/bus.o \
	common/bus/bus_gk20a.o \
	common/bus/bus_gm20b.o

nvgpu-y += \
	common/linux/kmem.o \
	common/linux/timers.o \
	common/linux/ioctl.o \
	common/linux/ioctl_ctrl.o \
	common/linux/ioctl_as.o \
	common/linux/ioctl_channel.o \
	common/linux/ioctl_tsg.o \
	common/linux/ioctl_dbg.o \
	common/linux/ioctl_clk_arb.o \
	common/linux/log.o \
	common/linux/cond.o \
	common/linux/nvgpu_mem.o \
	common/linux/dma.o \
	common/linux/driver_common.o \
	common/linux/firmware.o \
	common/linux/thread.o \
	common/linux/vm.o \
	common/linux/intr.o \
	common/linux/sysfs.o \
	common/linux/io.o \
	common/linux/io_usermode.o \
	common/linux/rwsem.o \
	common/linux/comptags.o \
	common/linux/dmabuf.o \
	common/linux/sched.o \
	common/linux/channel.o \
	common/linux/ce2.o \
	common/linux/sim.o \
	common/linux/sim_pci.o \
	common/linux/os_sched.o \
	common/linux/nvlink.o \
	common/mm/nvgpu_allocator.o \
	common/mm/bitmap_allocator.o \
	common/mm/buddy_allocator.o \
	common/mm/page_allocator.o \
	common/mm/lockless_allocator.o \
	common/mm/gmmu.o \
	common/mm/pd_cache.o \
	common/mm/vm.o \
	common/mm/vm_area.o \
	common/mm/nvgpu_mem.o \
	common/mm/comptags.o \
	common/mm/mm.o \
	common/enabled.o \
	common/nvlink.o \
	common/pramin.o \
	common/semaphore.o \
	common/as.o \
	common/rbtree.o \
	common/vbios/bios.o \
	common/falcon/falcon.o \
	common/pmu/pmu.o \
	common/pmu/pmu_ipc.o \
	common/pmu/pmu_fw.o \
	common/pmu/pmu_pg.o \
	common/pmu/pmu_perfmon.o \
	common/pmu/pmu_debug.o \
	common/ltc.o \
	common/clock_gating/gm20b_gating_reglist.o \
	common/clock_gating/gp106_gating_reglist.o \
	common/clock_gating/gp10b_gating_reglist.o \
	common/clock_gating/gv100_gating_reglist.o \
	common/clock_gating/gv11b_gating_reglist.o \
	gk20a/gk20a.o \
	gk20a/pramin_gk20a.o \
	gk20a/ce2_gk20a.o \
	gk20a/fifo_gk20a.o \
	gk20a/channel_gk20a.o \
	gk20a/channel_sync_gk20a.o \
	gk20a/dbg_gpu_gk20a.o \
	gk20a/regops_gk20a.o \
	gk20a/gr_gk20a.o \
	gk20a/mm_gk20a.o \
	gk20a/pmu_gk20a.o \
	gk20a/flcn_gk20a.o \
	gk20a/priv_ring_gk20a.o \
	gk20a/fence_gk20a.o \
	gk20a/therm_gk20a.o \
	gk20a/gr_ctx_gk20a_sim.o \
	gk20a/gr_ctx_gk20a.o \
	gk20a/fb_gk20a.o \
	gk20a/hal.o \
	gk20a/tsg_gk20a.o \
	gk20a/fecs_trace_gk20a.o \
	gk20a/mc_gk20a.o \
	gm20b/hal_gm20b.o \
	gm20b/ltc_gm20b.o \
	gm20b/gr_gm20b.o \
	gm20b/clk_gm20b.o \
	gm20b/fb_gm20b.o \
	gm20b/fifo_gm20b.o \
	gm20b/gr_ctx_gm20b.o \
	gm20b/acr_gm20b.o \
	gm20b/pmu_gm20b.o \
	gm20b/mm_gm20b.o \
	gm20b/regops_gm20b.o \
	gm20b/therm_gm20b.o \
	gm20b/fuse_gm20b.o \
	boardobj/boardobj.o \
	boardobj/boardobjgrp.o \
	boardobj/boardobjgrpmask.o \
	boardobj/boardobjgrp_e255.o \
	boardobj/boardobjgrp_e32.o \
	common/sim.o \
	common/sim_pci.o

nvgpu-$(CONFIG_GK20A_VIDMEM) += \
	common/mm/vidmem.o \
	common/linux/vidmem.o

nvgpu-$(CONFIG_DEBUG_FS) += \
	common/linux/debug.o \
	common/linux/debug_gr.o \
	common/linux/debug_fifo.o \
	common/linux/debug_ce.o \
	common/linux/debug_pmu.o \
	common/linux/debug_sched.o \
	common/linux/debug_mm.o \
	common/linux/debug_allocator.o \
	common/linux/debug_hal.o \
	common/linux/debug_clk.o \
	common/linux/debug_xve.o

ifeq ($(CONFIG_NVGPU_TRACK_MEM_USAGE),y)
nvgpu-$(CONFIG_DEBUG_FS) += \
	common/linux/debug_kmem.o
endif

nvgpu-$(CONFIG_GK20A_CTXSW_TRACE) += \
	common/linux/ctxsw_trace.o

nvgpu-$(CONFIG_TEGRA_GK20A) += \
	common/linux/module.o \
	common/linux/module_usermode.o \
	common/linux/soc.o \
	common/linux/fuse.o \
	common/linux/platform_gk20a_tegra.o \
	common/linux/platform_gp10b_tegra.o \
	common/linux/platform_gv11b_tegra.o

nvgpu-$(CONFIG_SYNC) += common/linux/sync_sema_android.o \
		common/linux/os_fence_android.o \
		common/linux/os_fence_android_sema.o

ifeq ($(CONFIG_TEGRA_GK20A_NVHOST), y)
nvgpu-$(CONFIG_SYNC) += common/linux/os_fence_android_syncpt.o
endif

nvgpu-$(CONFIG_GK20A_PCI) += common/linux/pci.o \
	common/linux/pci_usermode.o \

nvgpu-$(CONFIG_TEGRA_GK20A_NVHOST) += common/linux/nvhost.o

nvgpu-$(CONFIG_TEGRA_GR_VIRTUALIZATION) += \
	vgpu/ltc_vgpu.o \
	vgpu/gr_vgpu.o \
	vgpu/fifo_vgpu.o \
	vgpu/ce2_vgpu.o \
	vgpu/mm_vgpu.o \
	vgpu/vgpu.o \
	vgpu/dbg_vgpu.o \
	vgpu/tsg_vgpu.o \
	vgpu/gm20b/vgpu_gr_gm20b.o \
	vgpu/gp10b/vgpu_hal_gp10b.o  \
	vgpu/gp10b/vgpu_gr_gp10b.o  \
	vgpu/gp10b/vgpu_fuse_gp10b.o  \
	vgpu/gp10b/vgpu_mm_gp10b.o \
	vgpu/gv11b/vgpu_gv11b.o \
	vgpu/gv11b/vgpu_hal_gv11b.o \
	vgpu/gv11b/vgpu_gr_gv11b.o \
	vgpu/gv11b/vgpu_fifo_gv11b.o \
	vgpu/gv11b/vgpu_subctx_gv11b.o \
	vgpu/gv11b/vgpu_tsg_gv11b.o \
	common/linux/vgpu/platform_vgpu_tegra.o \
	common/linux/vgpu/fecs_trace_vgpu.o \
	common/linux/vgpu/clk_vgpu.o \
	common/linux/vgpu/css_vgpu.o \
	common/linux/vgpu/sysfs_vgpu.o \
	common/linux/vgpu/vgpu_ivc.o \
	common/linux/vgpu/vgpu_ivm.o \
	common/linux/vgpu/vgpu_linux.o \
	common/linux/vgpu/gv11b/platform_gv11b_vgpu_tegra.o

nvgpu-$(CONFIG_COMMON_CLK) += \
	common/linux/clk.o

nvgpu-$(CONFIG_GK20A_DEVFREQ) += \
	common/linux/scale.o

nvgpu-$(CONFIG_GK20A_CYCLE_STATS) += \
	gk20a/css_gr_gk20a.o

nvgpu-y += \
	gp10b/gr_gp10b.o  \
	gp10b/gr_ctx_gp10b.o  \
	gp10b/ce_gp10b.o \
	gp10b/mc_gp10b.o  \
	gp10b/fifo_gp10b.o  \
	gp10b/ltc_gp10b.o \
	gp10b/mm_gp10b.o \
	gp10b/fb_gp10b.o \
	gp10b/pmu_gp10b.o \
	gp10b/hal_gp10b.o \
	gp10b/rpfb_gp10b.o \
	gp10b/regops_gp10b.o \
	gp10b/therm_gp10b.o \
	gp10b/fecs_trace_gp10b.o \
	gp10b/priv_ring_gp10b.o \
	gp10b/gp10b.o \
	gp10b/fuse_gp10b.o \
	gp106/hal_gp106.o \
	gp106/mm_gp106.o \
	gp106/flcn_gp106.o \
	gp106/pmu_gp106.o \
	gp106/mclk_gp106.o \
	gp106/gr_gp106.o \
	gp106/gr_ctx_gp106.o \
	gp106/acr_gp106.o \
	gp106/sec2_gp106.o \
	gp106/fifo_gp106.o \
	gp106/fb_gp106.o \
	gp106/regops_gp106.o \
	gp106/bios_gp106.o \
	gp106/fuse_gp106.o \
	gv11b/gv11b.o  \
	gv11b/css_gr_gv11b.o  \
	gv11b/dbg_gpu_gv11b.o  \
	gv11b/mc_gv11b.o  \
	gv11b/ltc_gv11b.o  \
	gv11b/hal_gv11b.o \
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
	gv100/flcn_gv100.o \
	gv100/mc_gv100.o \
	gv100/nvlink_gv100.o \
	gv100/hal_gv100.o \
	gv100/pmu_gv100.o \
	pstate/pstate.o \
	clk/clk_vin.o \
	clk/clk_fll.o \
	clk/clk_domain.o \
	clk/clk_prog.o \
	clk/clk_vf_point.o \
	clk/clk_arb.o \
	clk/clk_freq_controller.o \
	perf/vfe_var.o \
	perf/vfe_equ.o \
	perf/perf.o \
	clk/clk.o \
	gp106/clk_gp106.o \
	gp106/clk_arb_gp106.o \
	gp106/xve_gp106.o \
	gp106/therm_gp106.o \
	gp106/xve_gp106.o \
	pmgr/pwrdev.o \
	pmgr/pmgr.o \
	pmgr/pmgrpmu.o \
	pmgr/pwrmonitor.o \
	pmgr/pwrpolicy.o \
	volt/volt_rail.o \
	volt/volt_dev.o \
	volt/volt_policy.o \
	volt/volt_pmu.o \
	therm/thrm.o \
	therm/thrmdev.o \
	therm/thrmchannel.o \
	therm/thrmpmu.o \
	lpwr/rppg.o \
	lpwr/lpwr.o

nvgpu-$(CONFIG_NVGPU_SUPPORT_CDE) += \
	common/linux/cde.o \
	common/linux/cde_gm20b.o \
	common/linux/cde_gp10b.o

ifeq ($(CONFIG_DEBUG_FS),y)
nvgpu-$(CONFIG_NVGPU_SUPPORT_CDE) += \
	common/linux/debug_cde.o
endif
