GCOV_PROFILE := y

ccflags-$(CONFIG_GK20A) += -I$(srctree)/drivers/gpu/nvgpu
ccflags-$(CONFIG_GK20A) += -I$(srctree)/include/linux
ccflags-$(CONFIG_GK20A) += -I$(srctree)/../kernel-t18x/include
ccflags-$(CONFIG_GK20A) += -Wno-multichar
ccflags-$(CONFIG_GK20A) += -Werror

obj-$(CONFIG_GK20A)	+= gp10b/
obj-$(CONFIG_TEGRA_GR_VIRTUALIZATION) += vgpu/gp10b/
