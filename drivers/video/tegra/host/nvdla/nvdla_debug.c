/*
 * NVDLA debug utils
 *
 * Copyright (c) 2016 - 2017, NVIDIA Corporation.  All rights reserved.
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

#define dla_set_trace_enable(pdev, trace_enable)		\
	debug_set_trace_event_config(pdev, trace_enable,	\
			DLA_SET_TRACE_ENABLE);			\

#define dla_set_trace_event_mask(pdev, event_mask)		\
	debug_set_trace_event_config(pdev, event_mask,	\
			DLA_SET_TRACE_EVENT_MASK);		\

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

static int debug_dla_tracedump_show(struct seq_file *s, void *data)
{
	char *bufptr;
	struct nvdla_device *nvdla_dev;
	struct platform_device *pdev;
	uint32_t i = 0, cindex = 0;
	uint32_t offset = TRACE_DATA_OFFSET;
	uint32_t start, end, datasize;

	nvdla_dev = (struct nvdla_device *)s->private;
	pdev = nvdla_dev->pdev;

	if (nvdla_dev->trace_dump_va && nvdla_dev->trace_enable) {
		bufptr = (char *)nvdla_dev->trace_dump_va;

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

static int debug_dla_enable_trace_show(struct seq_file *s, void *data)
{
	struct nvdla_device *nvdla_dev = (struct nvdla_device *)s->private;

	seq_printf(s, "%u\n", nvdla_dev->trace_enable);
	return 0;
}

static int debug_dla_eventmask_show(struct seq_file *s, void *data)
{
	struct nvdla_device *nvdla_dev = (struct nvdla_device *)s->private;

	seq_printf(s, "%u\n", nvdla_dev->events_mask);
	return 0;
}

static int debug_dla_eventmask_help_show(struct seq_file *s, void *data)
{
	seq_printf(s, "%s\n",
		"\nDla Firmware has following different tracing categories:");
	seq_printf(s, "%s\n", "  BIT(0) -  Processor\n"
			      "  BIT(1) -  Falcon\n"
			      "  BIT(2) -  Events\n"
			      "  BIT(3) -  Scheduler Queue\n"
			      "  BIT(4) -  Operation Cache\n");
	seq_printf(s, "%s\n", "To enable all type of tracing events,"
			      "set all bits ( 0 - 4 ): ");
	seq_printf(s, "%s\n\n", "  echo 31 > events_mask");
	return 0;
}

static int debug_dla_bintracedump_show(struct seq_file *s, void *data)
{
	char *bufptr;
	struct nvdla_device *nvdla_dev;
	struct platform_device *pdev;
	uint32_t i = 0;
	uint32_t offset = TRACE_DATA_OFFSET;
	uint32_t start, end, datasize;

	nvdla_dev = (struct nvdla_device *)s->private;
	pdev = nvdla_dev->pdev;

	if (nvdla_dev->trace_dump_va && nvdla_dev->trace_enable) {
		bufptr = (char *)nvdla_dev->trace_dump_va;

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

		/* to read trace buffer from 0th index */
		i = 0;
		 /* in this case, datasize includes header data also */
		datasize += offset;

		/* Dump data in binary format. */
		while (i < datasize)
			seq_printf(s, "%c", bufptr[i++]);
	}

	return 0;
}

static int debug_dla_enable_trace_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_dla_enable_trace_show, inode->i_private);
}

static int debug_dla_eventmask_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_dla_eventmask_show, inode->i_private);
}

static int debug_dla_eventmask_help_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_dla_eventmask_help_show, inode->i_private);
}

static int debug_dla_trace_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_dla_tracedump_show, inode->i_private);
}

static int debug_dla_bintrace_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_dla_bintracedump_show, inode->i_private);
}

static int debug_set_trace_event_config(struct platform_device *pdev,
	u32 value, u32 sub_cmd)
{
	int err = 0;
	struct nvdla_cmd_mem_info trace_events_mem_info;
	struct dla_debug_config *trace_event;
	struct nvdla_cmd_data cmd_data;

	/* make sure that device is powered on */
	err = nvhost_module_busy(pdev);
	if (err) {
		nvdla_dbg_err(pdev, "failed to power on\n");
		err = -ENODEV;
		goto fail_to_on;
	}

	/* assign memory for command */
	err = nvdla_get_cmd_memory(pdev, &trace_events_mem_info);
	if (err) {
		nvdla_dbg_err(pdev, "dma alloc for command failed");
		goto alloc_failed;
	}

	trace_event = (struct dla_debug_config *)trace_events_mem_info.va;
	trace_event->sub_cmd = sub_cmd;
	trace_event->data = (u64)value;

	/* prepare command data */
	cmd_data.method_id = DLA_CMD_SET_DEBUG;
	cmd_data.method_data = ALIGNED_DMA(trace_events_mem_info.pa);
	cmd_data.wait = true;

	/* pass set debug command to falcon */
	err = nvdla_send_cmd(pdev, &cmd_data);

	/* free memory allocated for trace event command */
	nvdla_put_cmd_memory(pdev, trace_events_mem_info.index);

	if (err != 0) {
		nvdla_dbg_err(pdev, "failed to send set debug command");
		goto send_cmd_failed;
	}

	nvhost_module_idle(pdev);
	return err;

send_cmd_failed:
alloc_failed:
	nvhost_module_idle(pdev);
fail_to_on:
	return err;
}

static ssize_t debug_dla_eventmask_set(struct file *file,
	const char __user *buffer, size_t count, loff_t *off)
{
	int ret;
	u32 val;
	struct platform_device *pdev;
	struct nvdla_device *nvdla_dev;
	struct seq_file *p = file->private_data;
	char str[] = "0123456789abcdef";

	nvdla_dev = (struct nvdla_device *)p->private;
	pdev = nvdla_dev->pdev;
	count = min_t(size_t, strlen(str), count);
	if (copy_from_user(str, buffer, count))
		return -EFAULT;

	mutex_lock(&p->lock);
	/* get value entered by user in variable val */
	ret = sscanf(str, "%u", &val);
	/* Check valid values for event_mask */
	if (ret == 1 && val <= 31)
		nvdla_dev->events_mask = val;
	mutex_unlock(&p->lock);

	if (ret != 1) {
		nvdla_dbg_err(pdev, "Incorrect input!");
		goto invalid_input;
	}

	/*
	 * Currently only five trace categories are added,
	 * and hence only five bits are being used to enable/disable
	 * the trace categories.
	 */
	if (val > 31) {
		nvdla_dbg_err(pdev,
			"invalid input, please"
			" check /d/nvdla*/firmware/trace/events/help");
		ret = -EINVAL;
		goto invalid_input;
	}

	/*  set event_mask config  */
	ret = dla_set_trace_event_mask(pdev, nvdla_dev->events_mask);
	if (ret) {
		nvdla_dbg_err(pdev, "failed to set event mask.");
		goto set_event_mask_failed;
	}

	return count;

set_event_mask_failed:
invalid_input:
	return ret;
}

static ssize_t debug_dla_enable_trace_set(struct file *file,
	const char __user *buffer, size_t count, loff_t *off)
{
	int ret;
	u32 val;
	struct nvdla_device *nvdla_dev;
	struct platform_device *pdev;
	struct seq_file *p = file->private_data;
	char str[] = "0123456789abcdef";

	nvdla_dev = (struct nvdla_device *)p->private;
	pdev = nvdla_dev->pdev;
	count = min_t(size_t, strlen(str), count);
	if (copy_from_user(str, buffer, count))
		return -EFAULT;

	mutex_lock(&p->lock);
	/* get value entered by user in variable val */
	ret = sscanf(str, "%u", &val);
	/* Check valid values for trace_enable */
	if (ret == 1 && (val == 0 || val == 1))
		nvdla_dev->trace_enable = val;
	mutex_unlock(&p->lock);

	if (ret != 1) {
		nvdla_dbg_err(pdev, "Incorrect input!");
		goto invalid_input;
	}

	if (val != 0 && val != 1) {
		nvdla_dbg_err(pdev,
			"invalid input, please"
			" enter 0(disable) or 1(enable)!");
		ret = -EINVAL;
		goto invalid_input;
	}

	/*  set trace_enable config  */
	ret = dla_set_trace_enable(pdev, nvdla_dev->trace_enable);
	if (ret) {
		nvdla_dbg_err(pdev, "failed to enable trace events.");
		goto set_trace_enable_failed;
	}

	return count;

set_trace_enable_failed:
invalid_input:
	return ret;
}

static const struct file_operations debug_dla_enable_trace_fops = {
		.open		= debug_dla_enable_trace_open,
		.read		= seq_read,
		.llseek		= seq_lseek,
		.release	= single_release,
		.write		= debug_dla_enable_trace_set,
};

static const struct file_operations debug_dla_eventmask_fops = {
		.open		= debug_dla_eventmask_open,
		.read		= seq_read,
		.llseek		= seq_lseek,
		.release	= single_release,
		.write		= debug_dla_eventmask_set,
};

static const struct file_operations debug_dla_eventmask_help_fops = {
		.open		= debug_dla_eventmask_help_open,
		.read		= seq_read,
		.llseek		= seq_lseek,
		.release	= single_release,
};

static const struct file_operations debug_dla_event_trace_fops = {
		.open		= debug_dla_trace_open,
		.read		= seq_read,
		.llseek		= seq_lseek,
		.release	= single_release,
};

static const struct file_operations debug_dla_bin_event_trace_fops = {
		.open		= debug_dla_bintrace_open,
		.read		= seq_read,
		.llseek		= seq_lseek,
		.release	= single_release,
};

static void dla_fw_debugfs_init(struct platform_device *pdev)
{
	struct dentry *fw_dir, *fw_trace, *events;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;
	struct dentry *dla_debugfs_root = pdata->debugfs;

	if (!dla_debugfs_root)
		return;

	fw_dir = debugfs_create_dir("firmware", dla_debugfs_root);
	if (!fw_dir)
		return;

	if (!debugfs_create_file("version", S_IRUGO, fw_dir,
			nvdla_dev, &nvdla_fw_ver_fops))
		goto trace_failed;

	fw_trace = debugfs_create_dir("trace", fw_dir);
	if (!fw_trace)
		goto trace_failed;

	if (!debugfs_create_file("enable", S_IRUGO | S_IWUSR, fw_trace,
			nvdla_dev, &debug_dla_enable_trace_fops))
		goto trace_failed;

	if (!debugfs_create_file("text_trace", S_IRUGO, fw_trace,
			nvdla_dev, &debug_dla_event_trace_fops))
		goto trace_failed;

	if (!debugfs_create_file("bin_trace", S_IRUGO, fw_trace,
			nvdla_dev, &debug_dla_bin_event_trace_fops))
		goto trace_failed;

	events = debugfs_create_dir("events", fw_trace);
	if (!events)
		goto event_failed;

	if (!debugfs_create_file("category", S_IWUSR | S_IRUGO, events,
			nvdla_dev, &debug_dla_eventmask_fops)) {
		goto event_failed;
	}

	if (!debugfs_create_file("help", S_IRUGO, events,
			nvdla_dev, &debug_dla_eventmask_help_fops)) {
		goto event_failed;
	}

	return;

event_failed:
	debugfs_remove_recursive(events);
	return;

trace_failed:
	debugfs_remove_recursive(fw_dir);
}

void nvdla_debug_init(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvdla_device *nvdla_dev = pdata->private_data;
	struct dentry *de = pdata->debugfs;

	if (!de)
		return;

	debugfs_create_u32("debug_mask", S_IRUGO | S_IWUSR, de,
			&nvdla_dev->dbg_mask);
	debugfs_create_u32("en_trace", S_IRUGO | S_IWUSR, de,
			&nvdla_dev->en_trace);

	dla_fw_debugfs_init(pdev);
}
