GCOV_PROFILE := y
ccflags-y += -I../nvhost/drivers/video/tegra/host
ccflags-y += -I../t19x/drivers/video/tegra/host
ccflags-y += -Idrivers/devfreq
ccflags-y += -Wno-multichar

subdir-ccflags-y := -Werror
obj-y  += nvhost_queue.o \
	  nvhost_buffer.o

obj-$(CONFIG_TEGRA_GRHOST) += t194/
obj-$(CONFIG_TEGRA_GRHOST_PVA) += pva/
obj-$(CONFIG_TEGRA_GRHOST_NVDLA) += nvdla/
