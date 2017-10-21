BCT_FILES_PATH := hardware/nvidia/platform/t19x/galen/bct

PRODUCT_COPY_FILES += \
    $(BCT_FILES_PATH)/pmic/tegra194-mb1-bct-pmic-p2888-0000-p2822-0000.cfg:tegra194-mb1-bct-pmic-p2888-0000-p2822-0000.cfg \
    $(BCT_FILES_PATH)/gpioint/tegra194-mb1-bct-gpioint-p2888-0000-p2822-0000.cfg:tegra194-mb1-bct-gpioint-p2888-0000-p2822-0000.cfg \
    $(BCT_FILES_PATH)/reset/tegra194-mb1-bct-reset-p2888-0000-p2822-0000.cfg:tegra194-mb1-bct-reset-p2888-0000-p2822-0000.cfg \
    $(BCT_FILES_PATH)/pinmux/tegra19x-mb1-pinmux-p2888-0000-p2822-0000.cfg:tegra19x-mb1-pinmux-p2888-0000-p2822-0000.cfg \
    $(BCT_FILES_PATH)/padvoltage/tegra19x-mb1-padvoltage-p2888-0000-p2822-0000.cfg:tegra19x-mb1-padvoltage-p2888-0000-p2822-0000.cfg \
    $(BCT_FILES_PATH)/prod/tegra19x-mb1-prod-p2888-0000-p2822-0000.cfg:tegra19x-mb1-prod-p2888-0000-p2822-0000.cfg
