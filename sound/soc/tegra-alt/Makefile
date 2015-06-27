KBUILD_CFLAGS += -I$(srctree)/sound/soc/tegra-alt

snd-soc-tegra186-alt-asrc-objs := tegra186_asrc_alt.o
snd-soc-tegra186-alt-arad-objs := tegra186_arad_alt.o
snd-soc-tegra186-alt-afc-objs := tegra186_afc_alt.o
snd-soc-tegra186-alt-xbar-objs := tegra186_xbar_alt.o

obj-$(CONFIG_SND_SOC_TEGRA186_ASRC_ALT) += snd-soc-tegra186-alt-asrc.o
obj-$(CONFIG_SND_SOC_TEGRA186_ARAD_ALT) += snd-soc-tegra186-alt-arad.o
obj-$(CONFIG_SND_SOC_TEGRA210_AFC_ALT) += snd-soc-tegra186-alt-afc.o
obj-$(CONFIG_SND_SOC_TEGRA210_XBAR_ALT) += snd-soc-tegra186-alt-xbar.o

# The funky manipulation of $(srctree) is for dynamically finding the name of
# the kernel root dir.
snd-soc-tegra-alt-t186ref-objs := tegra_t186ref_alt.o tegra_asoc_machine_alt_t18x.o \
	../../../../$(lastword $(subst /, ,$(srctree)))/sound/soc/tegra-alt/tegra_asoc_machine_alt.o

obj-$(CONFIG_SND_SOC_TEGRA_T186REF_ALT) += snd-soc-tegra-alt-t186ref.o
