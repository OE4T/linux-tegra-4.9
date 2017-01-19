/*
 * NVDLA debug utils
 *
 * Copyright (c) 2016, NVIDIA Corporation.  All rights reserved.
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

#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/nvhost.h>
#include "host1x/host1x.h"
#include "flcn/flcn.h"
#include "flcn/hw_flcn.h"
#include "dla_os_interface.h"
#include <linux/uaccess.h>

#include "nvdla/nvdla.h"
#include "nvdla_debug.h"

/*
 * Header in ring buffer consist (start, end) two uint32_t values.
 * Trace data content starts from the offset below.
 */
#define TRACE_DATA_OFFSET	(2 * sizeof(uint32_t))

static int nvdla_fw_ver_show(struct seq_file *s, void *unused)
{
	struct nvdla_device *nvdla_dev;
	struct platform_device *pdev;
	int err;

	nvdla_dev = (struct nvdla_device *)s->private;
	pdev = nvdla_dev->pdev;

	/* update fw_version if engine is not yet powered on */
	err = nvhost_module_busy(pdev);
	if (err)
		return err;
	nvhost_module_idle(pdev);

	seq_printf(s, "%u.%u.%u\n",
		((nvdla_dev->fw_version >> 16) & 0xff),
		((nvdla_dev->fw_version >> 8) & 0xff),
		(nvdla_dev->fw_version & 0xff));

	return 0;

}

static int nvdla_fw_ver_open(struct inode *inode, struct file *file)
{
	return single_open(file, nvdla_fw_ver_show, inode->i_private);
}

static const struct file_operations nvdla_fw_ver_fops = {
	.open		= nvdla_fw_ver_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int debug_dla_dump_show(struct seq_file *s, void *data)
{
	char *bufptr;
	struct flcn *m;
	struct nvdla_device *nvdla_dev;
	struct platform_device *pdev;
	uint32_t i = 0, cindex = 0;
	uint32_t offset = TRACE_DATA_OFFSET;
	uint32_t start, end, datasize;

	nvdla_dev = (struct nvdla_device *)s->private;
	pdev = nvdla_dev->pdev;
	m = get_flcn(pdev);

	if (!m)
		return 0;

	if (m->trace_dump_va) {
		bufptr = (char *)m->trace_dump_va;

		if (!strcmp(bufptr, ""))
			return 0;

		memcpy(&start, bufptr, sizeof(uint32_t));
		memcpy(&end, ((char *)bufptr + sizeof(uint32_t)),
			sizeof(uint32_t));

		i = start;

		if (start == (end + 1))
			datasize = (uint32_t)TRACE_BUFFER_SIZE - offset;
		else
			datasize = end - start;

		while (cindex < datasize) {
			seq_printf(s, "%c", bufptr[i]);
			i++;
			i = ((i - offset) % (TRACE_BUFFER_SIZE - offset)) +
				offset;
			cindex++;

			if ((bufptr[i] == '\n') && (cindex < datasize)) {
				seq_printf(s, "%c", bufptr[i]);

				/* skip extra new line chars */
				while ((bufptr[i] == '\n') &&
				    (cindex < datasize)) {
					i++;
					i = ((i - offset) %
						(TRACE_BUFFER_SIZE - offset)) +
						offset;
					cindex++;
				}
			}
		}

		seq_printf(s, "%c", '\n');
	}

	return 0;
}

static int debug_dla_trace_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_dla_dump_show, inode->i_private);
}

static const struct file_operations debug_dla_trace_fops = {
	.open		= debug_dla_trace_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void nvdla_debug_init(struct platform_device *pdev)
{
	struct dentry *ret;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;
	struct dentry *de = pdata->debugfs;

	if (!de)
		return;

	debugfs_create_u32("debug_mask", S_IRUGO | S_IWUSR, de,
			&nvdla_dev->dbg_mask);
	debugfs_create_u32("en_trace", S_IRUGO | S_IWUSR, de,
			&nvdla_dev->en_trace);
	debugfs_create_file("fw_version", S_IRUGO, de, nvdla_dev,
			&nvdla_fw_ver_fops);
	ret = debugfs_create_file("fw_trace", S_IRUGO,
			de, nvdla_dev, &debug_dla_trace_fops);
	if (!ret)
		nvdla_dbg_err(pdev, "Failed to create trace debug file!\n");
}
