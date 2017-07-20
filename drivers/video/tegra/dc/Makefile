GCOV_PROFILE := y
subdir-ccflags-y := -Werror
ccflags-y += -I$(srctree)/../nvidia/drivers/video/tegra/host
ifeq ($(CONFIG_TEGRA_NVDISPLAY),y)
ccflags-y += -I$(srctree)/../nvidia/drivers/video/tegra/dc
ccflags-y += -I$(srctree)/../nvidia/drivers/platform/tegra
endif
ccflags-y += -I$(srctree)/arch/arm/mach-tegra
obj-y += dc_common.o dc.o mode.o clock.o window.o dc_client.o
ifneq ($(CONFIG_TEGRA_NVDISPLAY),y)
obj-y += bandwidth.o
endif
obj-y += csc.o
obj-y += lut.o
obj-y += cursor.o
obj-y += of_dc.o
obj-$(CONFIG_TEGRA_HDMI2FPD_DS90UH949) += hdmi2fpd_ds90uh949.o
#obj-y += dsi2lvds.o
obj-$(CONFIG_TEGRA_HDMI2DSI_TC358870) += hdmi2dsi_tc358870.o
obj-$(CONFIG_TEGRA_DSI2EDP_TC358767) += tc358767_dsi2edp.o
obj-$(CONFIG_TEGRA_DSI2EDP_SN65DSI86) += sn65dsi86_dsi2edp.o
obj-$(CONFIG_TEGRA_DSI2LVDS_SN65DSI85) += sn65dsi85_dsi2lvds.o
obj-$(CONFIG_TEGRA_LVDS2FPDL_DS90UB947) += ds90ub947-q1.o
obj-$(CONFIG_TEGRA_DS90UH948Q_DESER)    += ds90uh948q_deser.o
obj-$(CONFIG_TEGRA_EDP2LVDS_PS8625) += ps8625_edp2lvds.o
ccflags-y +=-DCOMMON_MIPICAL_SUPPORTED\
	-I$(srctree)/../nvidia/drivers/media/platform/tegra

ifneq ($(CONFIG_TEGRA_NVDISPLAY),y)
obj-$(CONFIG_TEGRA_HDMIHDCP) += hdmihdcp.o
obj-$(CONFIG_TEGRA_HDMIHDCP) += nvhdcp_hdcp22_methods.o
obj-$(CONFIG_TEGRA_DPHDCP) += dphdcp.o
ccflags-$(CONFIG_TEGRA_HDMIHDCP) += -DCONFIG_HDCP
ccflags-$(CONFIG_TEGRA_DPHDCP) += -DCONFIG_DPHDCP
endif
ifeq ($(CONFIG_TEGRA_NVDISPLAY),y)
ifeq ($(CONFIG_TRUSTY),y)
obj-$(CONFIG_TEGRA_HDMIHDCP) += hdmihdcp.o
obj-$(CONFIG_TEGRA_HDMIHDCP) += nvhdcp_hdcp22_methods.o
obj-$(CONFIG_TEGRA_DPHDCP) += dphdcp.o
ccflags-$(CONFIG_TEGRA_HDMIHDCP) += -DCONFIG_HDCP
ccflags-$(CONFIG_TEGRA_DPHDCP) += -DCONFIG_DPHDCP
endif
endif
obj-y += edid.o edid_quirks.o
ifneq ($(CONFIG_TEGRA_NVDISPLAY),y)
obj-$(CONFIG_TEGRA_NVSD) += nvsd.o
endif
obj-y += dsi.o
obj-$(CONFIG_DEBUG_FS) += dsi_debug.o
obj-$(CONFIG_DEBUG_FS) += tegra_dsi_csi_test.o
obj-y += dc_sysfs.o
obj-y += dc_config.o
obj-y += sor.o
obj-y += dpaux.o
obj-$(CONFIG_DEBUG_FS) += dp_debug.o
obj-$(CONFIG_TEGRA_DP) += dp.o
obj-$(CONFIG_TEGRA_DP) += dp_lt.o
obj-$(CONFIG_TEGRA_DP) += dp_auto.o
obj-$(CONFIG_TEGRA_DP) += hpd.o
obj-$(CONFIG_TEGRA_NVSR) += nvsr.o
obj-$(CONFIG_TEGRA_DC) += ext/
obj-$(CONFIG_TEGRA_HDMI2_0) += hdmi2.0.o
obj-$(CONFIG_TEGRA_HDMI2GMSL_MAX929x) += max929x_hdmi2gmsl.o
obj-$(CONFIG_TEGRA_HDA_DC) += hda_dc.o
obj-$(CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT) += fake_panel.o null_or.o
obj-$(CONFIG_TEGRA_VRR) += vrr.o
obj-$(CONFIG_TEGRA_NVDISPLAY) += dsi_padctrl.o
obj-$(CONFIG_TEGRA_HDMIVRR) += hdmivrr.o
obj-y += panel/
obj-$(CONFIG_TEGRA_CEC_SUPPORT) += ../../../misc/tegra-cec/
obj-y += crc.o
