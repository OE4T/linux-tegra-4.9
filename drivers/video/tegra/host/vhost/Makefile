GCOV_PROFILE := y
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Werror

nvhost-vhost-objs  = \
	vhost_intr.o \
	vhost_syncpt.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-vhost.o
