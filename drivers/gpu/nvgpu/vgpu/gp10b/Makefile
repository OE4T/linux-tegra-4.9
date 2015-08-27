GCOV_PROFILE := y

ccflags-$(CONFIG_GK20A) += -I$(srctree)/drivers/gpu/nvgpu
ccflags-$(CONFIG_GK20A) += -I$(srctree)/include
ccflags-$(CONFIG_GK20A) += -I$(srctree)/../kernel-t18x/drivers/gpu/nvgpu
ccflags-$(CONFIG_GK20A) += -I$(srctree)/../kernel-t18x/include
ccflags-$(CONFIG_GK20A) += -I$(srctree)/../kernel-t18x/include/uapi

ccflags-$(CONFIG_GK20A) += -Wno-multichar
ccflags-y += -Werror

obj-$(CONFIG_GK20A)  += \
	vgpu_hal_gp10b.o  \
	vgpu_gr_gp10b.o  \
	vgpu_mm_gp10b.o \
	vgpu_fifo_gp10b.o
