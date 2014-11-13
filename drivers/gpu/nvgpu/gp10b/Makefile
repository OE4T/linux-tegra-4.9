GCOV_PROFILE := y

ccflags-$(CONFIG_GK20A) += -I$(srctree)/../kernel/drivers/gpu/nvgpu
ccflags-$(CONFIG_GK20A) += -I$(srctree)/../kernel/include
ccflags-$(CONFIG_GK20A) += -I$(srctree)/../kernel-t18x/drivers/gpu/nvgpu
ccflags-$(CONFIG_GK20A) += -I$(srctree)/../kernel-t18x/include
ccflags-$(CONFIG_GK20A) += -I$(srctree)/../kernel-t18x/include/uapi

ccflags-$(CONFIG_GK20A) += -Wno-multichar

obj-$(CONFIG_GK20A)  += \
	gr_gp10b.o  \
	mc_gp10b.o  \
	ltc_gp10b.o \
	mm_gp10b.o \
	fb_gp10b.o \
	hal_gp10b.o

obj-$(CONFIG_TEGRA_GK20A) += platform_gp10b_tegra.o
