BCT_FILES_PATH := hardware/nvidia/platform/t19x/common/bct

PRODUCT_COPY_FILES += \
    $(BCT_FILES_PATH)/gpio-intmap/tegra194-mb1-bct-gpio-int-to-all-int0.cfg:tegra194-mb1-bct-gpio-int-to-all-int0.cfg	\
    $(BCT_FILES_PATH)/uphy-lane/tegra194-mb1-bct-uphy-lane-ufs-x1-lane10.cfg:tegra194-mb1-bct-uphy-lane-ufs-x1-lane10.cfg    \
    $(BCT_FILES_PATH)/uphy-lane/tegra194-mb1-bct-uphy-lane-ufs-x1-lane11.cfg:tegra194-mb1-bct-uphy-lane-ufs-x1-lane11.cfg    \
    $(BCT_FILES_PATH)/uphy-lane/tegra194-mb1-bct-uphy-lane-ufs-x2.cfg:tegra194-mb1-bct-uphy-lane-ufs-x2.cfg  \
    $(BCT_FILES_PATH)/uphy-lane/tegra194-mb1-bct-uphy-lane-sata.cfg:tegra194-mb1-bct-uphy-lane-sata.cfg      \
    $(BCT_FILES_PATH)/uphy-lane/tegra194-mb1-bct-uphy-lane-ufs-x1-sata.cfg:tegra194-mb1-bct-uphy-lane-ufs-x1-sata.cfg
