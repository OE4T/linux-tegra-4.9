/*
 * drivers/video/tegra/dc/dsi_debug.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/moduleparam.h>
#include <linux/export.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include "dc_reg.h"
#include "dc_priv.h"
#include "dev.h"
#include "dsi_regs.h"
#include "dsi.h"
/* HACK! This needs to come from DT */
#include "../../../../arch/arm/mach-tegra/iomap.h"

#ifdef CONFIG_DEBUG_FS

static int dbg_dsi_show(struct seq_file *s, void *unused)
{
	struct tegra_dc_dsi_data *dsi = s->private;
	unsigned long i = 0, j = 0;
	u32 col = 0;
	u32 base[MAX_DSI_INSTANCE] = {TEGRA_DSI_BASE, TEGRA_DSIB_BASE};

	if (!dsi->enabled) {
		seq_puts(s, "DSI controller suspended\n");
		return 0;
	}

	tegra_dc_io_start(dsi->dc);
	tegra_dsi_clk_enable(dsi);

	/* mem dd dump */
	for (i = 0; i < dsi->max_instances; i++) {
		for (col = 0, j = 0; j < 0x64; j++) {
			if (col == 0)
				seq_printf(s, "%08lX:", base[i] + 4*j);
			seq_printf(s, "%c%08lX", col == 2 ? '-' : ' ',
				tegra_dsi_controller_readl(dsi, j, i));
			if (col == 3) {
				seq_puts(s, "\n");
				col = 0;
			} else
				col++;
		}
		seq_puts(s, "\n");
	}

#define DUMP_REG(a)	seq_printf(s, "%-45s | %#05x | %#010lx |\n", \
					#a, a, tegra_dsi_readl(dsi, a));

	DUMP_REG(DSI_INCR_SYNCPT_CNTRL);
	DUMP_REG(DSI_INCR_SYNCPT_ERROR);
	DUMP_REG(DSI_CTXSW);
	DUMP_REG(DSI_POWER_CONTROL);
	DUMP_REG(DSI_INT_ENABLE);
	DUMP_REG(DSI_HOST_DSI_CONTROL);
	DUMP_REG(DSI_CONTROL);
	DUMP_REG(DSI_SOL_DELAY);
	DUMP_REG(DSI_MAX_THRESHOLD);
	DUMP_REG(DSI_TRIGGER);
	DUMP_REG(DSI_TX_CRC);
	DUMP_REG(DSI_STATUS);
	DUMP_REG(DSI_INIT_SEQ_CONTROL);
	DUMP_REG(DSI_INIT_SEQ_DATA_0);
	DUMP_REG(DSI_INIT_SEQ_DATA_1);
	DUMP_REG(DSI_INIT_SEQ_DATA_2);
	DUMP_REG(DSI_INIT_SEQ_DATA_3);
	DUMP_REG(DSI_INIT_SEQ_DATA_4);
	DUMP_REG(DSI_INIT_SEQ_DATA_5);
	DUMP_REG(DSI_INIT_SEQ_DATA_6);
	DUMP_REG(DSI_INIT_SEQ_DATA_7);
	DUMP_REG(DSI_PKT_SEQ_0_LO);
	DUMP_REG(DSI_PKT_SEQ_0_HI);
	DUMP_REG(DSI_PKT_SEQ_1_LO);
	DUMP_REG(DSI_PKT_SEQ_1_HI);
	DUMP_REG(DSI_PKT_SEQ_2_LO);
	DUMP_REG(DSI_PKT_SEQ_2_HI);
	DUMP_REG(DSI_PKT_SEQ_3_LO);
	DUMP_REG(DSI_PKT_SEQ_3_HI);
	DUMP_REG(DSI_PKT_SEQ_4_LO);
	DUMP_REG(DSI_PKT_SEQ_4_HI);
	DUMP_REG(DSI_PKT_SEQ_5_LO);
	DUMP_REG(DSI_PKT_SEQ_5_HI);
	DUMP_REG(DSI_DCS_CMDS);
	DUMP_REG(DSI_PKT_LEN_0_1);
	DUMP_REG(DSI_PKT_LEN_2_3);
	DUMP_REG(DSI_PKT_LEN_4_5);
	DUMP_REG(DSI_PKT_LEN_6_7);
	DUMP_REG(DSI_PHY_TIMING_0);
	DUMP_REG(DSI_PHY_TIMING_1);
	DUMP_REG(DSI_PHY_TIMING_2);
	DUMP_REG(DSI_BTA_TIMING);
	DUMP_REG(DSI_TIMEOUT_0);
	DUMP_REG(DSI_TIMEOUT_1);
	DUMP_REG(DSI_TO_TALLY);
	DUMP_REG(DSI_PAD_CONTROL);
	DUMP_REG(DSI_PAD_CONTROL_CD);
	DUMP_REG(DSI_PAD_CD_STATUS);
	DUMP_REG(DSI_VID_MODE_CONTROL);
	DUMP_REG(DSI_PAD_CONTROL_0_VS1);
	DUMP_REG(DSI_PAD_CONTROL_CD_VS1);
	DUMP_REG(DSI_PAD_CD_STATUS_VS1);
	DUMP_REG(DSI_PAD_CONTROL_1_VS1);
	DUMP_REG(DSI_PAD_CONTROL_2_VS1);
	DUMP_REG(DSI_PAD_CONTROL_3_VS1);
	DUMP_REG(DSI_PAD_CONTROL_4_VS1);
	DUMP_REG(DSI_GANGED_MODE_CONTROL);
	DUMP_REG(DSI_GANGED_MODE_START);
	DUMP_REG(DSI_GANGED_MODE_SIZE);
#undef DUMP_REG

	tegra_dsi_clk_disable(dsi);
	tegra_dc_io_end(dsi->dc);

	return 0;
}

static int dbg_dsi_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dsi_show, inode->i_private);
}

static const struct file_operations dbg_fops = {
	.open = dbg_dsi_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static u32 max_ret_payload_size;
static u32 panel_reg_addr;

static ssize_t read_panel_get(struct seq_file *s, void *unused)
{
	struct tegra_dc_dsi_data *dsi = s->private;
	struct tegra_dc *dc = dsi->dc;
	int err = 0;
	u8 buf[300] = {0};
	int j = 0 , b = 0 , k;
	u32 payload_size = 0;

	if (!dsi->enabled) {
		dev_info(&dc->ndev->dev, " controller suspended\n");
	return -EINVAL;
}

	seq_printf(s, "max ret payload size:0x%x\npanel reg addr:0x%x\n",
					max_ret_payload_size, panel_reg_addr);
	if (max_ret_payload_size == 0) {
		seq_puts(s, "echo was not successful\n");
	return err;
}
	err = tegra_dsi_read_data(dsi->dc, dsi,
				max_ret_payload_size,
				panel_reg_addr, buf);

	seq_printf(s, " Read data[%d] ", b);

	for (b = 1; b < (max_ret_payload_size+1); b++) {
		j = (b*4)-1;
		for (k = j; k > (j-4); k--)
			if ((k%4) == 0 && b != max_ret_payload_size) {
				seq_printf(s, " %x  ", buf[k]);
				seq_printf(s, "\n Read data[%d] ", b);
			}
			else
				seq_printf(s, " %x ", buf[k]);
	}
	seq_puts(s, "\n");

	switch (buf[0]) {
	case DSI_ESCAPE_CMD:
		seq_printf(s, "escape cmd[0x%x]\n", buf[0]);
		break;
	case DSI_ACK_NO_ERR:
		seq_printf(s,
			"Panel ack, no err[0x%x]\n", buf[0]);
		goto fail;
		break;
	default:
		seq_puts(s, "Invalid read response\n");
		break;
	}

	switch (buf[4] & 0xff) {
	case GEN_LONG_RD_RES:
		/* Fall through */
	case DCS_LONG_RD_RES:
		payload_size = (buf[5] |
				(buf[6] << 8)) & 0xFFFF;
		seq_printf(s, "Long read response Packet\n"
				"payload_size[0x%x]\n", payload_size);
		break;
	case GEN_1_BYTE_SHORT_RD_RES:
		/* Fall through */
	case DCS_1_BYTE_SHORT_RD_RES:
		payload_size = 1;
		seq_printf(s, "Short read response Packet\n"
			"payload_size[0x%x]\n", payload_size);
		break;
	case GEN_2_BYTE_SHORT_RD_RES:
		/* Fall through */
	case DCS_2_BYTE_SHORT_RD_RES:
		payload_size = 2;
		seq_printf(s, "Short read response Packet\n"
			"payload_size[0x%x]\n", payload_size);
		break;
	case ACK_ERR_RES:
		payload_size = 2;
		seq_printf(s, "Acknowledge error report response\n"
			"Packet payload_size[0x%x]\n", payload_size);
		break;
	default:
		seq_puts(s, "Invalid response packet\n");
		break;
	}
fail:
	return err;
}

static ssize_t read_panel_set(struct file *file, const char  *buf,
						size_t count, loff_t *off)
{
	struct seq_file *s = file->private_data;
	struct tegra_dc_dsi_data *dsi = s->private;
	struct tegra_dc *dc = dsi->dc;

	sscanf(buf, "%x %x", &max_ret_payload_size, &panel_reg_addr);
		dev_info(&dc->ndev->dev, "max ret payload size:0x%x\npanel reg addr:0x%x\n",
				max_ret_payload_size, panel_reg_addr);

		return count;
}

static int read_panel_open(struct inode *inode, struct file *file)
{
	return single_open(file, read_panel_get, inode->i_private);
};

static const struct file_operations read_panel_fops = {
	.open = read_panel_open,
	.read = seq_read,
	.write = read_panel_set,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct dentry *dsidir;

void tegra_dc_dsi_debug_create(struct tegra_dc_dsi_data *dsi)
{
	struct dentry *retval;

	dsidir = debugfs_create_dir("tegra_dsi", NULL);
	if (!dsidir)
		return;
	retval = debugfs_create_file("regs", S_IRUGO, dsidir, dsi,
		&dbg_fops);
	if (!retval)
		goto free_out;
	retval = debugfs_create_file("read_panel", S_IRUGO|S_IWUSR, dsidir,
				dsi, &read_panel_fops);
	if (!retval)
		goto free_out;
	return;
free_out:
	debugfs_remove_recursive(dsidir);
	dsidir = NULL;
	return;
}
EXPORT_SYMBOL(tegra_dc_dsi_debug_create);
#else
void tegra_dc_dsi_debug_create(struct tegra_dc_dsi_data *dsi)
{}
EXPORT_SYMBOL(tegra_dc_dsi_debug_create);

#endif
