GCOV_PROFILE := y
ccflags-y += -Idrivers/gpu/nvgpu
ccflags-y += -Wno-multichar

obj-$(CONFIG_TEGRA_GR_VIRTUALIZATION)  = \
	ltc_vgpu.o \
	gr_vgpu.o \
	fifo_vgpu.o \
	mm_vgpu.o \
	vgpu.o
