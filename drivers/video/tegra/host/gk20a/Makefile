
GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Idrivers/devfreq
ccflags-y += -Wno-multichar
ccflags-y += -Werror

obj-$(CONFIG_GK20A) += \
	gk20a.o \
	ctrl_gk20a.o \
	fifo_gk20a.o \
	channel_gk20a.o \
	cdma_gk20a.o \
	debug_gk20a.o \
	dbg_gpu_gk20a.o \
	regops_gk20a.o \
	gr_gk20a.o \
	kind_gk20a.o \
	mm_gk20a.o \
	pmu_gk20a.o \
	priv_ring_gk20a.o \
	clk_gk20a.o \
	therm_gk20a.o \
	gr_ctx_gk20a_sim.o \
	gr_ctx_gk20a.o \
	gk20a_gating_reglist.o \
	gk20a_scale.o \
	gk20a_sysfs.o \
	ltc_gk20a.o \
	hal.o \
	hal_gk20a.o

obj-$(CONFIG_GK20A) += platform_gk20a_generic.o
obj-$(CONFIG_TEGRA_GK20A) += platform_gk20a_tegra.o
