GCOV_PROFILE := y
ccflags-y += -I$(srctree)/../nvhost/drivers/video/tegra/host
ccflags-y += -I$(srctree)/../nvhost-t19x/drivers/video/tegra/host
ccflags-y += -I$(srctree)/../nvhost-t19x/include
ccflags-y += -Werror

nvhost-minimal-pm-objs  = \
		nvhost_minimal_pm.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-minimal-pm.o
