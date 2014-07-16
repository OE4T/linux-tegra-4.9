GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -I../kernel-t18x/drivers/video/tegra/host
ccflags-y += -Werror

nvhost-nvcsi-objs  = \
		nvcsi.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-nvcsi.o
