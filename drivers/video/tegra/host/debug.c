/*
 * drivers/video/tegra/host/debug.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Erik Gilling <konkers@android.com>
 *
 * Copyright (C) 2011 NVIDIA Corporation
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

#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <asm/io.h>

#include "dev.h"

struct output {
	void (*fn)(void *ctx, const char* str, size_t len);
	void *ctx;
	char buf[256];
};

static void write_to_seqfile(void *ctx, const char* str, size_t len)
{
	seq_write((struct seq_file *)ctx, str, len);
}

static void write_to_printk(void *ctx, const char* str, size_t len)
{
	printk("%s", str);
}

static void output(struct output *o, const char* fmt, ...)
{
	va_list args;
	int len;

	va_start(args, fmt);
	len = vsnprintf(o->buf, sizeof(o->buf), fmt, args);
	va_end(args);
	o->fn(o->ctx, o->buf, len);
}

enum {
	NVHOST_DBG_STATE_CMD = 0,
	NVHOST_DBG_STATE_DATA = 1,
	NVHOST_DBG_STATE_GATHER = 2
};

static int show_channel_command(struct output *o, u32 val, int *count)
{
	unsigned mask;
	unsigned subop;

	switch (val >> 28) {
	case 0x0:
		mask = val & 0x3f;
		if (mask) {
			output(o, "SETCL(class=%03x, offset=%03x, mask=%02x, [",
				   val >> 6 & 0x3ff, val >> 16 & 0xfff, mask);
			*count = hweight8(mask);
			return NVHOST_DBG_STATE_DATA;
		} else {
			output(o, "SETCL(class=%03x)\n", val >> 6 & 0x3ff);
			return NVHOST_DBG_STATE_CMD;
		}

	case 0x1:
		output(o, "INCR(offset=%03x, [", val >> 16 & 0xfff);
		*count = val & 0xffff;
		return NVHOST_DBG_STATE_DATA;

	case 0x2:
		output(o, "NONINCR(offset=%03x, [", val >> 16 & 0xfff);
		*count = val & 0xffff;
		return NVHOST_DBG_STATE_DATA;

	case 0x3:
		mask = val & 0xffff;
		output(o, "MASK(offset=%03x, mask=%03x, [",
			   val >> 16 & 0xfff, mask);
		*count = hweight16(mask);
		return NVHOST_DBG_STATE_DATA;

	case 0x4:
		output(o, "IMM(offset=%03x, data=%03x)\n",
			   val >> 16 & 0xfff, val & 0xffff);
		return NVHOST_DBG_STATE_CMD;

	case 0x5:
		output(o, "RESTART(offset=%08x)\n", val << 4);
		return NVHOST_DBG_STATE_CMD;

	case 0x6:
		output(o, "GATHER(offset=%03x, insert=%d, type=%d, count=%04x, addr=[",
			   val >> 16 & 0xfff, val >> 15 & 0x1, val >> 14 & 0x1,
			   val & 0x3fff);
		*count = val & 0x3fff; // TODO: insert
		return NVHOST_DBG_STATE_GATHER;

	case 0xe:
		subop = val >> 24 & 0xf;
		if (subop == 0)
			output(o, "ACQUIRE_MLOCK(index=%d)\n", val & 0xff);
		else if (subop == 1)
			output(o, "RELEASE_MLOCK(index=%d)\n", val & 0xff);
		else
			output(o, "EXTEND_UNKNOWN(%08x)\n", val);
		return NVHOST_DBG_STATE_CMD;

	default:
		return NVHOST_DBG_STATE_CMD;
	}
}

static void show_channel_gather(struct output *o, phys_addr_t phys_addr,
				u32 words);

static void show_channel_word(struct output *o, int *state, int *count,
				     u32 addr, u32 val)
{
	switch (*state) {
	case NVHOST_DBG_STATE_CMD:
		if (addr)
			output(o, "%08x: %08x:", addr, val);
		else
			output(o, "%08x:", val);

		*state = show_channel_command(o, val, count);
		if (*state == NVHOST_DBG_STATE_DATA && *count == 0) {
			*state = NVHOST_DBG_STATE_CMD;
			output(o, "])\n");
		}
		break;

	case NVHOST_DBG_STATE_DATA:
		(*count)--;
		output(o, "%08x%s", val, *count > 0 ? ", " : "])\n");
		if (*count == 0)
			*state = NVHOST_DBG_STATE_CMD;
		break;

	case NVHOST_DBG_STATE_GATHER:
		*state = NVHOST_DBG_STATE_CMD;
		output(o, "%08x]):\n", val);
		show_channel_gather(o, val, *count);
		break;
	}
}

/*
 * TODO: This uses ioremap_xxx on memory which is deprecated.
 * Also, it won't work properly with SMMU.
 */
static void show_channel_gather(struct output *o, phys_addr_t phys_addr,
				u32 words)
{
	phys_addr_t map_base = phys_addr & PAGE_MASK;
	phys_addr_t map_end = (phys_addr + words * 4 + PAGE_SIZE - 1) & PAGE_MASK;
	phys_addr_t map_size = map_end - map_base;
	phys_addr_t map_offset = phys_addr - map_base;
	void *map_addr = ioremap_nocache(map_base, map_size);
	int state = NVHOST_DBG_STATE_CMD;
	int count, i;

	if (!map_addr)
		return;
	for (i = 0; i < words; i++)
		show_channel_word(o, &state, &count, phys_addr + i * 4,
				readl(map_addr + map_offset + i * 4));
	iounmap(map_addr);
}

static void show_channel_pair(struct output *o, u32 addr,
				u32 w0, u32 w1)
{
	int state = NVHOST_DBG_STATE_CMD;
	int count;

	show_channel_word(o, &state, &count, addr, w0);
	show_channel_word(o, &state, &count, addr, w1);
}

static void show_channel_cdma(struct nvhost_master *m,
			struct output *o, int chid)
{
	struct nvhost_channel *channel = m->channels + chid;
	u32 dmaput, dmaget, dmactrl;
	u32 cbstat, cbread;
	u32 val, base, baseval;
	u32 pbw[2];

	dmaput = readl(channel->aperture + HOST1X_CHANNEL_DMAPUT);
	dmaget = readl(channel->aperture + HOST1X_CHANNEL_DMAGET);
	dmactrl = readl(channel->aperture + HOST1X_CHANNEL_DMACTRL);
	cbread = readl(m->aperture + HOST1X_SYNC_CBREAD(chid));
	cbstat = readl(m->aperture + HOST1X_SYNC_CBSTAT(chid));

	output(o, "%d-%s (%d): ", chid,
		channel->mod.name, atomic_read(&channel->mod.refcount));

	if ((dmactrl & 1) || !channel->cdma.push_buffer.mapped) {
		output(o, "inactive\n\n");
		return;
	}

	switch (cbstat) {
	case 0x00010008:
		output(o, "waiting on syncpt %d val %d\n",
			cbread >> 24, cbread & 0xffffff);
		break;

	case 0x00010009:
		base = (cbread >> 16) & 0xff;
		val = readl(m->aperture + HOST1X_SYNC_SYNCPT_BASE(base));
		baseval = val & 0xffff;
		val = cbread & 0xffff;
		output(o, "waiting on syncpt %d val %d "
			  "(base %d = %d; offset = %d)\n",
			cbread >> 24, baseval + val,
			base, baseval, val);
		break;

	default:
		output(o, "active class %02x, offset %04x, val %08x\n",
			cbstat >> 16, cbstat & 0xffff, cbread);
		break;
	}

	nvhost_cdma_peek(&channel->cdma, dmaget, -1, pbw);
	show_channel_pair(o, chid, pbw[0], pbw[1]);
	output(o, "\n");
}

void show_channel_fifo(struct nvhost_master *m,
			struct output *o, int chid)
{
	u32 val, rd_ptr, wr_ptr, start, end;
	int state, count;

	val = readl(m->aperture + HOST1X_CHANNEL_FIFOSTAT);
	if (val & (1 << 10))
		return;

	writel(0x0, m->aperture + HOST1X_SYNC_CFPEEK_CTRL);
	writel((1 << 31) | (chid << 16),
		m->aperture + HOST1X_SYNC_CFPEEK_CTRL);

	val = readl(m->aperture + HOST1X_SYNC_CFPEEK_PTRS);
	rd_ptr = val & 0x1ff;
	wr_ptr = (val >> 16) & 0x1ff;

	val = readl(m->aperture + HOST1X_SYNC_CF_SETUP(chid));
	start = val & 0x1ff;
	end = (val >> 16) & 0x1ff;

	state = NVHOST_DBG_STATE_CMD;
	output(o, "%d: fifo:\n", chid);

	do {
		writel(0x0, m->aperture + HOST1X_SYNC_CFPEEK_CTRL);
		writel((1 << 31) | (chid << 16) | rd_ptr,
			m->aperture + HOST1X_SYNC_CFPEEK_CTRL);
		val = readl(m->aperture + HOST1X_SYNC_CFPEEK_READ);

		show_channel_word(o, &state, &count, 0, val);

		if (rd_ptr == end)
			rd_ptr = start;
		else
			rd_ptr++;
	} while (rd_ptr != wr_ptr);

	if (state == NVHOST_DBG_STATE_DATA)
		output(o, ", ...])\n");
	output(o, "\n");

	writel(0x0, m->aperture + HOST1X_SYNC_CFPEEK_CTRL);
}

static void show_channels(struct nvhost_master *m, struct output *o)
{
	int i;
	output(o, "---- channels ----\n");
	for (i = 0; i < NVHOST_NUMCHANNELS; i++) {
		show_channel_cdma(m, o, i);
		show_channel_fifo(m, o, i);
	}
}

static void show_mlocks(struct nvhost_master *m, struct output *o)
{
	u32 __iomem *mlo_regs = m->sync_aperture + HOST1X_SYNC_MLOCK_OWNER_0;
	int i;

	output(o, "---- mlocks ----\n");
	for (i = 0; i < NV_HOST1X_NB_MLOCKS; i++) {
		u32 owner = readl(mlo_regs + i);
		if (owner & 0x1)
			output(o, "%d: locked by channel %d\n",
				i, (owner >> 8) & 0xf);
		else if (owner & 0x2)
			output(o, "%d: locked by cpu\n", i);
		else
			output(o, "%d: unlocked\n", i);
	}
	output(o, "\n");
}

static void show_syncpts(struct nvhost_master *m, struct output *o)
{
	int i;

	output(o, "---- syncpts ----\n");
	for (i = 0; i < NV_HOST1X_SYNCPT_NB_PTS; i++) {
		u32 max = nvhost_syncpt_read_max(&m->syncpt, i);
		if (!max)
			continue;
		output(o, "id %d (%s) min %d max %d\n",
			i, nvhost_syncpt_name(i),
			nvhost_syncpt_update_min(&m->syncpt, i), max);

	}
	output(o, "\n");
}

static void show_all(struct nvhost_master *m, struct output *o)
{
	nvhost_module_busy(&m->mod);

	show_mlocks(m, o);
	show_syncpts(m, o);
	show_channels(m, o);

	nvhost_module_idle(&m->mod);
}


#ifdef CONFIG_DEBUG_FS
static int nvhost_debug_show(struct seq_file *s, void *unused)
{
	struct output o = {
		.fn = write_to_seqfile,
		.ctx = s
	};
	show_all(s->private, &o);
	return 0;
}

static int nvhost_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, nvhost_debug_show, inode->i_private);
}

static const struct file_operations nvhost_debug_fops = {
	.open		= nvhost_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void nvhost_debug_init(struct nvhost_master *master)
{
	debugfs_create_file("tegra_host", S_IRUGO, NULL,
			master, &nvhost_debug_fops);
}
#else
void nvhost_debug_init(struct nvhost_master *master)
{
}
#endif

void nvhost_debug_dump(struct nvhost_master *master)
{
	struct output o = {
		.fn = write_to_printk
	};
	show_all(master, &o);
}
