BCT_FILES_PATH := hardware/nvidia/platform/t19x/stardust/bct

PRODUCT_COPY_FILES += \
    $(BCT_FILES_PATH)/pmic/tegra194-mb1-bct-pmic-p2888-0000-p2822-0000.cfg:tegra194-mb1-bct-pmic-p2888-0000-p2822-0000.cfg \
    $(BCT_FILES_PATH)/gpioint/tegra194-mb1-bct-gpioint-p2888-0000-p2822-0000.cfg:tegra194-mb1-bct-gpioint-p2888-0000-p2822-0000.cfg \
    $(BCT_FILES_PATH)/reset/tegra194-mb1-bct-reset-p2888-0000-p2822-0000.cfg:tegra194-mb1-bct-reset-p2888-0000-p2822-0000.cfg
