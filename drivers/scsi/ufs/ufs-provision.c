/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *      Abhinav Site    <asite@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifdef CONFIG_DEBUG_FS

#include "ufs-provision.h"
#include "ufs-tegra.h"

#define START_PROVISIONING 1
#define MAX_LUN_COUNT 8
#define CONFIG_DESC_SIZE 144
#define UNIT_DESC_SIZE 16
#define CONFIG_DESC_HEADER_SIZE 16
#define ACTIVE_MODE 0x01
#define EQUAL_PRIORITY 0x7F

/* UNIT DESCRIPTOR FIELD OFFSETS */
#define LUENABLE_OFFSET 0
#define BOOTLUN_ID_OFFSET 1
#define LU_WRITE_PROTECT_OFFSET 2
#define MEMORY_TYPE_OFFSET 3
#define NUM_ALLOC_UNITS_OFFSET 4
#define DATA_RELIABILITY_OFFSET 8
#define LOGICAL_BLK_SIZE_OFFSET 9
#define PROV_TYPE_OFFSET 10
#define CONTEXT_CAP_OFFSET 11

/* CONFIGURATION DESCRIPTOR HEADER FIELDS OFFSETS */
#define LENGTH_OFFSET 0
#define DESC_TYPE_OFFFSET 1
#define INIT_PWR_MODE_OFFSET 5
#define HIGH_PRIORITY_LUN_OFFSET 6

void populate_desc_header(u8 *lun_desc_buf)
{
	lun_desc_buf[LENGTH_OFFSET] = CONFIG_DESC_SIZE;
	lun_desc_buf[DESC_TYPE_OFFFSET] = QUERY_DESC_IDN_CONFIGURAION;
	lun_desc_buf[INIT_PWR_MODE_OFFSET] = ACTIVE_MODE;
	lun_desc_buf[HIGH_PRIORITY_LUN_OFFSET] = EQUAL_PRIORITY;
}

int validate_refclk_value(struct ufs_hba *hba, u32 refclk_value)
{
	if (refclk_value > 3) {
		dev_err(hba->dev, "%s: Bad bRefClkFreq value\n"
				"Input Value: 0x%02x\n"
				"Valid Values are:\n"
				"0x00: 19.2Mhz\n"
				"0x01: 26Mhz\n"
				"0x02: 38.4Mhz\n"
				"0x03: 52Mhz\n"
				, __func__, refclk_value);
		return -EINVAL;
	}
	return 0;
}

int validate_desc_header(struct ufs_hba *hba, u8 *lun_desc_buf)
{
	int i, err = 0;
	u8 desc_param;

	#define GET_PARAM_VAL(i, offset) \
			((lun_desc_buf + (i+1)*(UNIT_DESC_SIZE))[(offset)])

	for (i = 0; i < MAX_LUN_COUNT; i++) {

		desc_param = GET_PARAM_VAL(i, LUENABLE_OFFSET);
		if (desc_param != 0 && desc_param != 1) {
			dev_err(hba->dev,
				"%s: Bad bLUenable for LUN%d:\n"
				"Input value: 0x%02x\n"
				"Valid values are: 0x0 and 0x1\n"
				, __func__, i, desc_param);
			err = -EINVAL;
		}

		desc_param = GET_PARAM_VAL(i, BOOTLUN_ID_OFFSET);
		if (desc_param > 2) {
			dev_err(hba->dev,
				"%s: Bad bBootLunID for LUN%d:\n"
				"Input value: 0x%02x\n"
				"Valid values are: 0x0 to 0x2\n"
				, __func__, i, desc_param);
			err = -EINVAL;
		}

		desc_param = GET_PARAM_VAL(i, LU_WRITE_PROTECT_OFFSET);
		if (desc_param > 3) {
			dev_err(hba->dev,
				"%s: Bad bLUWriteProtect for LUN%d:\n"
				"Input value: 0x%02x\n"
				"Valid values are: 0x0 to 0x3\n"
				, __func__, i, desc_param);
			err = -EINVAL;
		}

		desc_param = GET_PARAM_VAL(i, MEMORY_TYPE_OFFSET);
		if (desc_param > 6) {
			dev_err(hba->dev, "%s: Bad bMemoryType for LUN%d:\n"
				"Input value: 0x%02x\n"
				"Valid values are: 0x0 to 0x6\n"
				, __func__, i, desc_param);
			err = -EINVAL;
		}

		desc_param = GET_PARAM_VAL(i, DATA_RELIABILITY_OFFSET);
		if (desc_param != 0 && desc_param != 1) {
			dev_err(hba->dev,
				"%s: Bad bDataReliability for LUN%d:\n"
				"Input value: 0x%02x\n"
				"Valid values are: 0x0 and 0x1\n"
				, __func__, i, desc_param);
			err = -EINVAL;
		}

		desc_param = GET_PARAM_VAL(i, PROV_TYPE_OFFSET);
		if (desc_param != 0 && desc_param != 2 && desc_param != 3) {
			dev_err(hba->dev,
				"%s: Bad bProvisioningType for LUN%d:\n"
				"Input value: 0x%02x\n"
				"Valid values are: 0x0, 0x2, and 0x3\n"
				, __func__, i, desc_param);
			err = -EINVAL;
		}
	}

	#undef GET_PARAM_VAL

	return err;
}

static int provision_debugfs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t provision_debugfs_read(struct file *file, char __user *buf,
				size_t count, loff_t *f_pos)
{
	return -EPERM;
}

static ssize_t program_lun_debugfs_write(struct file *file,
			const char __user *buf,	size_t count, loff_t *f_pos)
{
	int err, i;
	ssize_t ret;
	char *kbuf;
	u32 desc_lock;
	struct ufs_hba *hba = file->private_data;
	struct ufs_tegra_host *ufs_tegra = hba->priv;

	/*
	 * PROGRAM_LUN is set to 1 to trigeer  programming of LUNS.
	 * Any write to PROGRAM_LUN after pogramming LUNs is discarded.
	 * Below check prevents reprogramming LUNs in same boot cycle.
	 */
	if (ufs_tegra->program_lun == 1) {
		dev_err(hba->dev, "LUNs already programmed in this boot cycle\n"
				"Reboot to program again\n");
		return -EPERM;
	}

	kbuf = (char *)devm_kmalloc(hba->dev, sizeof(char)*count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	ret = simple_write_to_buffer(kbuf, sizeof(char)*count,
						f_pos, buf, count);
	if (ret < 0)
		goto out;

	kbuf[count] = '\0';

	err = kstrtol(kbuf, 10, &(ufs_tegra->program_lun));
	if (err) {
		dev_err(hba->dev, "Value passed should be an integer\n");
		ret = err;
		goto out;
	}

	if (ufs_tegra->program_lun == START_PROVISIONING) {

		/* Read bConfigDescLock */
		if (ufshcd_get_config_desc_lock(hba, &desc_lock)) {
			dev_err(hba->dev,
				"%s: Read bConfigDescrLock failed\n", __func__);
			goto out;
		}

		if (desc_lock != 0) {
			dev_err(hba->dev,
				"%s: Config Desciptor is locked\n"
				"Cannot program LUNs on the device. Aborting\n"
				, __func__);
			goto out;
		}

		/* Populate Config Desc Header */
		populate_desc_header(ufs_tegra->lun_desc_buf);

		/* Print descriptor array created */
		dev_info(hba->dev, "Configuration Descriptor array:\n");
		for (i = 0; i < CONFIG_DESC_SIZE; i++) {
			if ((i%16 == 0) && i)
				pr_info("\n");
			pr_info("0x%02x  ", (ufs_tegra->lun_desc_buf)[i]);
		}

		/* Validate unit desc data given by user */
		err = validate_desc_header(hba, ufs_tegra->lun_desc_buf);
		if (err) {
			dev_err(hba->dev,
				"%s: Descriptor Valdiation Failed\n", __func__);
			ret = err;
			goto out;
		}

		/* Program LUN */
		if (ufshcd_set_config_desc(hba, ufs_tegra->lun_desc_buf)) {
			dev_err(hba->dev,
				"%s: Failed to program LUNs\n", __func__);
		} else {
			dev_info(hba->dev,
				"%s: LUN Programming successful\n", __func__);
		}
	} else {
		dev_info(hba->dev, "%s:Skip programming LUNs\n", __func__);
	}
out:
	devm_kfree(hba->dev, kbuf);
	return ret;
}

static const struct file_operations program_lun_debugfs_ops = {
	.open           = provision_debugfs_open,
	.read           = provision_debugfs_read,
	.write          = program_lun_debugfs_write,
};

static ssize_t program_refclk_debugfs_write(struct file *file,
			const char __user *buf, size_t count, loff_t *f_pos)
{
	int err;
	ssize_t ret;
	char *kbuf;
	struct ufs_hba *hba = file->private_data;
	struct ufs_tegra_host *ufs_tegra = hba->priv;

	kbuf = (char *)devm_kmalloc(hba->dev, sizeof(char)*count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	ret = simple_write_to_buffer(kbuf, sizeof(char)*count,
						f_pos, buf, count);
	if (ret < 0)
		return ret;

	kbuf[count] = '\0';

	err = kstrtol(kbuf, 10, &(ufs_tegra->program_refclk));
	if (err) {
		dev_err(hba->dev, "Value passed should be an integer\n");
		ret = err;
		goto out;
	}

	if (ufs_tegra->program_refclk == START_PROVISIONING) {

		/* Validate refclk_value */
		err = validate_refclk_value(hba, ufs_tegra->refclk_value);
		if (err) {
			dev_err(hba->dev,
				"%s: Refclkfreq value valdiation failed\n",
				 __func__);
			ret = err;
			goto out;
		}

		/* Write brefclkFreq value */
		if (ufshcd_set_refclk_value(hba, &(ufs_tegra->refclk_value))) {
			dev_err(hba->dev,
				"%s: Write bRefClkFreq failed\n", __func__);
		}

		/* Read brefclkFreq value */
		if (ufshcd_get_refclk_value(hba, &(ufs_tegra->refclk_value))) {
			dev_err(hba->dev,
				"%s: Read bRefClkFreq failed\n", __func__);
		}
		dev_info(hba->dev, "%s: bRefclkFreq value is %d\n",
				__func__, ufs_tegra->refclk_value);
	} else {
		dev_info(hba->dev, "%s:Skip progamming refclkfreq\n", __func__);
	}
out:
	devm_kfree(hba->dev, kbuf);
	return ret;
}

static const struct file_operations refclk_debugfs_ops = {
	.open           = provision_debugfs_open,
	.read           = provision_debugfs_read,
	.write          = program_refclk_debugfs_write,
};

static int create_desc_debugfs_nodes(struct dentry *parent_lun_root,
							u8 *lun_desc_off)
{
	if (debugfs_create_x8("bLUenable", 0644, parent_lun_root,
			lun_desc_off + LUENABLE_OFFSET) == NULL)
		return -1;
	if (debugfs_create_x8("bBootLUNID", 0644, parent_lun_root,
			lun_desc_off + BOOTLUN_ID_OFFSET) == NULL)
		return -1;
	if (debugfs_create_x8("bLUWriteProtect", 0644, parent_lun_root,
			lun_desc_off + LU_WRITE_PROTECT_OFFSET) == NULL)
		return -1;
	if (debugfs_create_x8("bMemoryType", 0644, parent_lun_root,
				lun_desc_off + MEMORY_TYPE_OFFSET) == NULL)
		return -1;
	if (debugfs_create_x32("dNumAllocUnits", 0644, parent_lun_root,
		(u32 *)(lun_desc_off + NUM_ALLOC_UNITS_OFFSET)) == NULL)
		return -1;
	if (debugfs_create_x8("bDataReliability", 0644, parent_lun_root,
			lun_desc_off + DATA_RELIABILITY_OFFSET) == NULL)
		return -1;
	if (debugfs_create_x8("bLogicalBlocksize", 0644, parent_lun_root,
			lun_desc_off + LOGICAL_BLK_SIZE_OFFSET) == NULL)
		return -1;
	if (debugfs_create_x8("bProvisionType", 0644, parent_lun_root,
				lun_desc_off + PROV_TYPE_OFFSET) == NULL)
		return -1;
	if (debugfs_create_x16("wContextCapabilities", 0644,
		parent_lun_root,
		(u16 *)(lun_desc_off + CONTEXT_CAP_OFFSET)) == NULL)
		return -1;
	return 0;
}

void debugfs_provision_init(struct ufs_hba *hba, struct dentry *device_root)
{
	struct dentry *refclk_root = NULL, *lun_root = NULL, *tmp_lun_root;
	struct ufs_tegra_host *ufs_tegra = (struct ufs_tegra_host *)hba->priv;
	char lun_name[5];
	int i, err;

	refclk_root = debugfs_create_dir("ufs_refclk", device_root);
	if (refclk_root == NULL) {
		err = -1;
		goto out;
	}
	if (debugfs_create_x32("refclkfreq_value", 0644,
			refclk_root, &(ufs_tegra->refclk_value)) == NULL) {
		err = -1;
		goto out;
	}
	if (debugfs_create_file("program_refclkfreq", 0644,
			refclk_root, hba, &refclk_debugfs_ops) == NULL) {
		err = -1;
		goto out;
	}

	/* Create debugfs for LUN programming */

	ufs_tegra->lun_desc_buf = (u8 *)devm_kzalloc(hba->dev,
					CONFIG_DESC_SIZE, GFP_KERNEL);
	if (!ufs_tegra->lun_desc_buf) {
		dev_err(hba->dev,
			"No memory for Configuration Descriptor Array\n");
		err = -ENOMEM;
		goto out;
	}

	lun_root = debugfs_create_dir("ufs_luns", device_root);
	if (lun_root == NULL) {
		err = -1;
		goto out;
	}
	if (debugfs_create_file("program_lun", 0644,
			lun_root, hba, &program_lun_debugfs_ops) == NULL) {
		err = -1;
		goto out;
	}

	for (i = 0; i < MAX_LUN_COUNT; i++) {

		snprintf(lun_name, sizeof(lun_name), "lun%d", i);
		tmp_lun_root = debugfs_create_dir(lun_name, lun_root);
		if (tmp_lun_root == NULL) {
			err = -1;
			goto out;
		}

		/*
		 * Skip CONFIG_DESC_HEADER_SIZE i.e 16 bytes of desciptor
		 * First 16 Bytes of descriptor are for config desc header
		 */
		err = create_desc_debugfs_nodes(tmp_lun_root,
				ufs_tegra->lun_desc_buf +
				CONFIG_DESC_HEADER_SIZE + i*UNIT_DESC_SIZE);
		if (err)
			goto out;
	}
out:
	if (err) {
		dev_err(hba->dev, "Failed to create debugfs entries\n");
		debugfs_remove_recursive(lun_root);
		debugfs_remove_recursive(refclk_root);
	}
}
EXPORT_SYMBOL(debugfs_provision_init);

void debugfs_provision_exit(struct ufs_hba *hba)
{
	struct ufs_tegra_host *ufs_tegra = (struct ufs_tegra_host *)hba->priv;

	/* Free config desc buffer */
	devm_kfree(hba->dev, ufs_tegra->lun_desc_buf);
	ufs_tegra->lun_desc_buf = NULL;

}
EXPORT_SYMBOL(debugfs_provision_exit);
#endif

MODULE_LICENSE("GPL v2");
