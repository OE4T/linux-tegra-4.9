GCOV_PROFILE := y
ccflags-y += -Idrivers/gpu/nvgpu
ccflags-y += -Wno-multichar
ccflags-y += -Werror

obj-$(CONFIG_GK20A)  = \
	hal_gm20b.o \
	ltc_gm20b.o \
	gr_gm20b.o \
	gr_gm20b.o \
	fb_gm20b.o \
	fifo_gm20b.o \
	gr_ctx_gm20b.o \
	gm20b_gating_reglist.o \
	acr_gm20b.o \
	pmu_gm20b.o \
	clk_gm20b.o \
	mm_gm20b.o \
	regops_gm20b.o
