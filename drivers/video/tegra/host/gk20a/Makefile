
GCOV_PROFILE := y
EXTRA_CFLAGS += -Idrivers/video/tegra/host

nvhost-gk20a-objs  = \
	gk20a.o \
	fifo_gk20a.o \
	channel_gk20a.o \
	cdma_gk20a.o \
	debug_gk20a.o \
	gr_gk20a.o \
	kind_gk20a.o \
	mm_gk20a.o \
	pmu_gk20a.o \
	priv_ring_gk20a.o \
	clk_gk20a.o \
	therm_gk20a.o

ifneq ($(CONFIG_TEGRA_GK20A_NETLIST_FIRMWARE),y)
nvhost-gk20a-objs += gr_ctx_gk20a_sim.o
else
nvhost-gk20a-objs += gr_ctx_gk20a.o
endif

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-gk20a.o
