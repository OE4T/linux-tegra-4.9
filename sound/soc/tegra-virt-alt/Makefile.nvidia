GCOV_PROFILE := y

subdir-ccflags-y := -Werror
ccflags-y += -I$(src)/../tegra-alt
ccflags-y += -I$(src)/../codecs

# Tegra platform Support
snd-soc-tegra-virt-alt-ivc-objs := tegra_virt_alt_ivc.o

obj-$(CONFIG_SND_SOC_TEGRA_VIRT_IVC_COMM_DEFS) += snd-soc-tegra-virt-alt-ivc.o

snd-soc-tegra210-virt-alt-admaif-objs := tegra210_virt_alt_admaif.o \
					tegra_asoc_xbar_virt_alt.o \
					tegra_asoc_util_virt_alt.o \
					tegra_asoc_machine_virt_alt.o \

snd-soc-tegra-virt-t210ref-pcm-objs := tegra_virt_ref_alt.o

obj-$(CONFIG_SND_SOC_TEGRA_VIRT_T210REF_PCM) += snd-soc-tegra210-virt-alt-admaif.o
obj-$(CONFIG_SND_SOC_TEGRA_VIRT_T210REF_PCM) += snd-soc-tegra-virt-t210ref-pcm.o
