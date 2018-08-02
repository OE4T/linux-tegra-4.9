GCOV_PROFILE := y

subdir-ccflags-y := -Werror

ifeq ($(CONFIG_ARM64),y)
ccflags-y += -Iarch/arm/mach-tegra/include
endif

ccflags-y += -Iarch/arm/mach-tegra
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/nvmap/
ccflags-y += -I$(srctree.nvidia)/include/

obj-y += nvmap_alloc.o
obj-y += nvmap_dev.o
obj-y += nvmap_fault.o
obj-y += nvmap_heap.o
obj-y += nvmap_init.o
obj-y += nvmap_tag.o
obj-y += nvmap_stats.o

obj-y += nv2_client.o
obj-y += nv2_dmabuf.o
obj-y += nv2_dmabuf_ops.o
obj-y += nv2_handle.o
obj-y += nv2_handle_mm.o
obj-y += nv2_handle_map_ops.o
obj-y += nv2_handle_vma.o
obj-y += nv2_handle_print.o
obj-y += nv2_handle_ref.o
obj-y += nv2_heap_alloc.o
obj-y += nv2_ioctl.o
obj-y += nv2_ioctl_cache.o
obj-y += nv2_vma.o
obj-y += nv2_cache.o
obj-y += nv2_carveout.o
obj-y += nv2_handle_map.o
obj-y += nv2_helper.o

obj-$(CONFIG_NVMAP_PAGE_POOLS) += nvmap_pp.o

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
obj-y += nvmap_cache_nvmap_t18x.o
endif

ifdef CONFIG_ARCH_TEGRA_19x_SOC

obj-y += nvmap_init_t19x.o
obj-y += nvmap_dmabuf_t19x.o
obj-y += nvmap_cache_t19x.o

endif
