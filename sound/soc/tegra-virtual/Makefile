GCOV_PROFILE := y

subdir-ccflags-y := -Werror
ccflags-y += -I$(src)/../tegra-alt
ccflags-y += -I$(src)/../codecs

# Tegra platform Support
snd-soc-tegra124-virt-slave-apbif-objs := tegra124_virt_apbif_slave.o
snd-soc-tegra124-virt-master-apbif-objs := tegra124_virt_apbif_master.o
snd-soc-tegra-virt-utils-objs := tegra_virt_utils.o
snd-soc-tegra-alt-machine-objs := ../tegra-alt/tegra_asoc_machine_alt.o

obj-$(CONFIG_SND_SOC_TEGRA124_VIRT_APBIF_SLAVE) += snd-soc-tegra124-virt-slave-apbif.o
obj-$(CONFIG_SND_SOC_TEGRA124_VIRT_APBIF_SLAVE) += snd-soc-tegra-virt-utils.o
obj-$(CONFIG_SND_SOC_TEGRA124_VIRT_APBIF_MASTER) += snd-soc-tegra124-virt-master-apbif.o

# Tegra machine Support for slave
snd-soc-tegra-virt-vcm30t124-slave-objs := tegra_virt_vcm30t124_slave.o
obj-$(CONFIG_SND_SOC_TEGRA_VIRT_VCM30T124_SLAVE) += snd-soc-tegra-virt-vcm30t124-slave.o

# Tegra machine Support for master
snd-soc-tegra-virt-vcm30t124-master-objs := ../tegra-alt/tegra_vcm30t124_alt.o
obj-$(CONFIG_SND_SOC_TEGRA_VCM30T124_MASTER) += snd-soc-tegra-virt-vcm30t124-master.o
obj-$(CONFIG_SND_SOC_TEGRA_VCM30T124_MASTER) += snd-soc-tegra-alt-machine.o
