/*
 * PVA Debug Information file
 *
 * Copyright (c) 2017, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include "dev.h"
#include "pva.h"

static void pva_read_crashdump(struct seq_file *s, struct pva_seg_info *seg_info)
{
	int i = 0;
	u32 *seg_addr = (u32 *) seg_info->addr;

	if (!seg_addr)
		return;

	for (i = 0; i < (seg_info->size >> 4);) {
		seq_printf(s, "0x%x 0x%x 0x%x 0x%x\n",
			seg_addr[i], seg_addr[i+1],
			seg_addr[i+2], seg_addr[i+3]);
		i = i + 4;
	}
}

static int pva_crashdump(struct seq_file *s, void *data)
{
	int err = 0;
	struct pva_crashdump_debugfs_entry *entry =
			(struct pva_crashdump_debugfs_entry *)s->private;
	struct pva *pva = entry->pva;

	err = nvhost_module_busy(pva->pdev);
	if (err) {
		nvhost_dbg_info("err in powering up pva\n");
		goto err_poweron;
	}

	pva_read_crashdump(s, &entry->seg_info);

	nvhost_module_idle(pva->pdev);

err_poweron:
	return err;
}

static int crashdump_open(struct inode *inode, struct file *file)
{
	return single_open(file, pva_crashdump, inode->i_private);
}

static const struct file_operations pva_crashdump_fops = {
	.open = crashdump_open,
	.read = seq_read,
	.release = single_release,
};

void pva_debugfs_init(struct platform_device *pdev)
{
	struct dentry *ret;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct pva *pva = pdata->private_data;
	struct dentry *de = pdata->debugfs;

	if (!de)
		return;

	pva->debugfs_entry_r5.pva = pva;
	pva->debugfs_entry_vpu0.pva = pva;
	pva->debugfs_entry_vpu1.pva = pva;

	ret = debugfs_create_file("r5_crashdump", S_IRUGO, de,
				&pva->debugfs_entry_r5, &pva_crashdump_fops);
	if (!ret)
		nvhost_dbg_info("Failed R5_crashdump file creation");

	ret = debugfs_create_file("vpu0_crashdump", S_IRUGO, de,
				&pva->debugfs_entry_vpu0, &pva_crashdump_fops);
	if (!ret)
		nvhost_dbg_info("Failed VPU0_crashdump file creation");


	ret = debugfs_create_file("vpu1_crashdump", S_IRUGO, de,
				&pva->debugfs_entry_vpu1, &pva_crashdump_fops);
	if (!ret)
		nvhost_dbg_info("Failed VPU1_crashdump file creation");

	ret = debugfs_create_u32("submit_mode", S_IRUGO | S_IWUSR, de,
				 &pva->submit_mode);
	if (!ret)
		nvhost_dbg_info("Failed to create submit mode selection file");
}
