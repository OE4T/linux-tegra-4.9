subdir-ccflags-y			+= -Werror

obj-$(CONFIG_ARCH_TEGRA_21x_SOC)	+= bpmp_t210.o
obj-$(CONFIG_ARCH_TEGRA_21x_SOC)	+= mail_t210.o
obj-$(CONFIG_DEBUG_FS)			+= fwdebug.o

obj-y					+= bpmp.o
obj-y					+= mail.o
obj-y					+= mailman.o

obj-$(CONFIG_ARCH_TEGRA_18x_SOC)	+= mail_native_ivc.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC)	+= mail_t186.o
obj-$(CONFIG_TEGRA_HV_MANAGER)		+= bpmp_virt.o
obj-$(CONFIG_TEGRA_HV_MANAGER)		+= mail_virt_ivc.o

ifndef CONFIG_ARCH_TEGRA_21x_SOC
obj-y					+= bpmp_tty.o
endif

GCOV_PROFILE				:= y
