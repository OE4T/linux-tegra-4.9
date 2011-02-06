/*
 * drivers/video/tegra/host/nvhost_3dctx.c
 *
 * Tegra Graphics Host 3d hardware context
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "nvhost_hwctx.h"
#include "dev.h"

#include <linux/slab.h>

#define NV_WAR_789194 1

#ifdef CONFIG_ARCH_TEGRA_3x_SOC
static bool s_is_v1 = true;
static int s_nr_gpus = 2;
#else
static bool s_is_v1 = false;
static int s_nr_gpus = 1;
#endif

const struct hwctx_reginfo ctxsave_regs_3d_global[] = {
	HWCTX_REGINFO(0, 0xe00,    4, DIRECT),
	HWCTX_REGINFO(0, 0xe05,   30, DIRECT),
	HWCTX_REGINFO(0, 0xe25,    2, DIRECT),
	HWCTX_REGINFO(0, 0xe28,    2, DIRECT),
	HWCTX_REGINFO(1, 0xe30,   16, DIRECT),
	HWCTX_REGINFO(0, 0x001,    2, DIRECT),
	HWCTX_REGINFO(0, 0x00c,   10, DIRECT),
	HWCTX_REGINFO(0, 0x100,   34, DIRECT),
	HWCTX_REGINFO(0, 0x124,    2, DIRECT),
	HWCTX_REGINFO(0, 0x200,    5, DIRECT),
	HWCTX_REGINFO(0, 0x205, 1024, INDIRECT),
	HWCTX_REGINFO(0, 0x207, 1024, INDIRECT),
	HWCTX_REGINFO(0, 0x209,    1, DIRECT),
	HWCTX_REGINFO(0, 0x300,   64, DIRECT),
	HWCTX_REGINFO(0, 0x343,   25, DIRECT),
	HWCTX_REGINFO(0, 0x363,    2, DIRECT),
	HWCTX_REGINFO(0, 0x400,   16, DIRECT),
	HWCTX_REGINFO(0, 0x411,    1, DIRECT),
	HWCTX_REGINFO(1, 0x412,    1, DIRECT),
	HWCTX_REGINFO(0, 0x500,    4, DIRECT),
	HWCTX_REGINFO(0, 0x520,   32, DIRECT),
	HWCTX_REGINFO(0, 0x540,   64, INDIRECT),
	HWCTX_REGINFO(0, 0x600,   16, INDIRECT_4X),
	HWCTX_REGINFO(0, 0x603,  128, INDIRECT),
	HWCTX_REGINFO(0, 0x608,    4, DIRECT),
	HWCTX_REGINFO(0, 0x60e,    1, DIRECT),
	HWCTX_REGINFO(0, 0x700,   64, INDIRECT),
	HWCTX_REGINFO(0, 0x710,   50, DIRECT),
	HWCTX_REGINFO(1, 0x750,   16, DIRECT),
	HWCTX_REGINFO(0, 0x800,   16, INDIRECT_4X),
	HWCTX_REGINFO(0, 0x803,  512, INDIRECT),
	HWCTX_REGINFO(0, 0x805,   64, INDIRECT),
	HWCTX_REGINFO(0, 0x820,   32, DIRECT),
	HWCTX_REGINFO(0, 0x900,   64, INDIRECT),
	HWCTX_REGINFO(0, 0x902,    2, DIRECT),
	HWCTX_REGINFO(1, 0x90a,    1, DIRECT),
	HWCTX_REGINFO(0, 0xa02,   10, DIRECT),
	HWCTX_REGINFO(1, 0xb04,    1, DIRECT),
	HWCTX_REGINFO(1, 0xb06,   13, DIRECT),
};

const struct hwctx_reginfo ctxsave_regs_3d_pergpu[] = {
	HWCTX_REGINFO(0, 0xe04,    1, DIRECT),
	HWCTX_REGINFO(0, 0xe2a,    1, DIRECT),
	HWCTX_REGINFO(1, 0x413,    1, DIRECT),
	HWCTX_REGINFO(1, 0x90b,    1, DIRECT),
	HWCTX_REGINFO(1, 0xe41,    1, DIRECT),
};

struct save_info {
	u32 *ptr;
	unsigned int save_count;
	unsigned int restore_count;
	unsigned int save_incrs;
	unsigned int restore_incrs;
};

struct ctx_saver {
	unsigned int version;
	void (*save_begin)(u32 *ptr);
	unsigned int save_begin_size;
	void (*save_direct)(u32 *ptr, u32 start_reg, u32 count);
	unsigned int save_direct_size;
	void (*save_indirect)(u32 *ptr, u32 offset_reg, u32 offset,
			u32 data_reg, u32 count);
	unsigned int save_indirect_size;
	void (*save_end)(u32 *ptr);
	unsigned int save_end_size;
	unsigned short save_incrs;
	unsigned short save_thresh_offset;
	struct nvhost_hwctx *(*ctx3d_alloc)(struct nvhost_channel *ch);
	void (*ctx3d_save_push)(struct nvhost_cdma *cdma,
				struct nvhost_hwctx *ctx);
	void (*ctx3d_save_service)(struct nvhost_hwctx *ctx);
};


/*** restore ***/

static unsigned int restore_size = 0;
static unsigned int restore_gpu1_offset = 0;
static unsigned int restore_incrs = 0;

static void restore_begin(u32 *ptr)
{
	/* set class to host */
	ptr[0] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					NV_CLASS_HOST_INCR_SYNCPT_BASE, 1);
	/* increment sync point base */
	ptr[1] = nvhost_class_host_incr_syncpt_base(NVWAITBASE_3D,
						restore_incrs);
	/* set class to 3D */
	ptr[2] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0);
	/* program PSEQ_QUAD_ID */
	ptr[3] = nvhost_opcode_imm(0x545, 0);
}
#define RESTORE_BEGIN_SIZE 4

static void restore_direct(u32 *ptr, u32 start_reg, u32 count)
{
	ptr[0] = nvhost_opcode_incr(start_reg, count);
}
#define RESTORE_DIRECT_SIZE 1

static void restore_indirect(u32 *ptr, u32 offset_reg, u32 offset,
			u32 data_reg, u32 count)
{
	ptr[0] = nvhost_opcode_imm(offset_reg, offset);
	ptr[1] = nvhost_opcode_nonincr(data_reg, count);
}
#define RESTORE_INDIRECT_SIZE 2

static void restore_end(u32 *ptr)
{
	/* syncpt increment to track restore gather. */
	ptr[0] = nvhost_opcode_imm(0x0, 0x100 | NVSYNCPT_3D);
}
#define RESTORE_END_SIZE 1

static u32 *setup_restore_regs_v0(u32 *ptr,
			const struct hwctx_reginfo *regs,
			unsigned int nr_regs)
{
	const struct hwctx_reginfo *rend = regs + nr_regs;

	for ( ; regs != rend; ++regs) {
		u32 offset = regs->offset;
		u32 count = regs->count;
		u32 indoff = offset + 1;
		if (regs->version > 0)
			continue;
		switch (regs->type) {
		case HWCTX_REGINFO_DIRECT:
			restore_direct(ptr, offset, count);
			ptr += RESTORE_DIRECT_SIZE;
			break;
		case HWCTX_REGINFO_INDIRECT_4X:
			++indoff;
			/* fall through */
		case HWCTX_REGINFO_INDIRECT:
			restore_indirect(ptr, offset, 0, indoff, count);
			ptr += RESTORE_INDIRECT_SIZE;
			break;
		}
		ptr += count;
	}
	return ptr;
}

static void setup_restore_v0(u32 *ptr)
{
	restore_begin(ptr);
	ptr += RESTORE_BEGIN_SIZE;

	ptr = setup_restore_regs_v0(ptr,
			ctxsave_regs_3d_global,
			ARRAY_SIZE(ctxsave_regs_3d_global));

	ptr = setup_restore_regs_v0(ptr,
			ctxsave_regs_3d_pergpu,
			ARRAY_SIZE(ctxsave_regs_3d_pergpu));

	restore_end(ptr);

	wmb();
}


/*** save ***/

/* the same context save command sequence is used for all contexts. */
static struct nvmap_handle_ref *save_buf = NULL;
static u32 save_phys = 0;
static unsigned int save_size = 0;
static unsigned int save_incrs = 0;
static unsigned int save_thresh = 0;

static void __init setup_save_regs(const struct ctx_saver *saver,
			struct save_info *info,
			const struct hwctx_reginfo *regs,
			unsigned int nr_regs)
{
	const struct hwctx_reginfo *rend = regs + nr_regs;
	u32 *ptr = info->ptr;
	unsigned int save_count = info->save_count;
	unsigned int restore_count = info->restore_count;

	for ( ; regs != rend; ++regs) {
		u32 offset = regs->offset;
		u32 count = regs->count;
		u32 indoff = offset + 1;
		if (regs->version > saver->version)
			continue;
		switch (regs->type) {
		case HWCTX_REGINFO_DIRECT:
			if (ptr) {
				saver->save_direct(ptr, offset, count);
				ptr += saver->save_direct_size;
			}
			save_count += saver->save_direct_size;
			restore_count += RESTORE_DIRECT_SIZE;
			break;
		case HWCTX_REGINFO_INDIRECT_4X:
			++indoff;
			/* fall through */
		case HWCTX_REGINFO_INDIRECT:
			if (ptr) {
				saver->save_indirect(ptr, offset, 0,
						indoff, count);
				ptr += saver->save_indirect_size;
			}
			save_count += saver->save_indirect_size;
			restore_count += RESTORE_INDIRECT_SIZE;
			break;
		}
		if (ptr) {
			memset(ptr, 0, count * 4);
			ptr += count;
		}
		save_count += count;
		restore_count += count;
	}

	info->ptr = ptr;
	info->save_count = save_count;
	info->restore_count = restore_count;
}

static void __init switch_gpu(struct save_info *info,
			unsigned int save_src_gpu,
			u32 save_dest_gpus,
			u32 restore_dest_gpus)
{
#if NV_WAR_789194
	if (info->ptr) {
		info->ptr[0] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID,
						0, 0);
		info->ptr[1] = nvhost_opcode_nonincr(0x905, 2);
		info->ptr[2] = nvhost_opcode_imm(0, 0x200 | NVSYNCPT_3D);
		info->ptr[3] = nvhost_opcode_imm(0xb00, restore_dest_gpus);
		info->ptr[4] = nvhost_opcode_imm(0, 0x200 | NVSYNCPT_3D);
		info->ptr[5] = nvhost_opcode_imm(0xb00, save_dest_gpus);
		info->ptr[6] = nvhost_opcode_imm(0xb01, save_src_gpu);
		info->ptr += 7;
	}
	info->save_count += 7;
	info->restore_count += 2;
	info->save_incrs += 1;
	info->restore_incrs += 1;
#else
	if (info->ptr) {
		info->ptr[0] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID,
						0x905, 1);
		info->ptr[1] = nvhost_opcode_imm(0xb00, restore_dest_gpus);
		info->ptr[2] = nvhost_opcode_imm(0xb00, save_dest_gpus);
		info->ptr[3] = nvhost_opcode_imm(0xb01, save_src_gpu);
		info->ptr += 4;
	}
	info->save_count += 4;
	info->restore_count += 1;
#endif
}

static void __init setup_save(const struct ctx_saver *saver, u32 *ptr)
{
	struct save_info info = {
		ptr,
		saver->save_begin_size,
		RESTORE_BEGIN_SIZE,
		saver->save_incrs,
		1
	};
	bool is_sli = (s_nr_gpus == 2);
	BUG_ON(s_nr_gpus > 2);

	if (info.ptr) {
		saver->save_begin(info.ptr);
		info.ptr += saver->save_begin_size;
	}

	/* read from gpu0, write cmds through gpu0, restore to gpu0+gpu1 */
	if (is_sli)
		switch_gpu(&info, 0, 1, 3);

	/* save regs that are common to both gpus */
	setup_save_regs(saver, &info,
			ctxsave_regs_3d_global,
			ARRAY_SIZE(ctxsave_regs_3d_global));

	/* read from gpu0, write cmds through gpu0, restore to gpu0 */
	if (is_sli)
		switch_gpu(&info, 0, 1, 1);

	/* save gpu0-specific regs */
	setup_save_regs(saver, &info,
			ctxsave_regs_3d_pergpu,
			ARRAY_SIZE(ctxsave_regs_3d_pergpu));

	if (is_sli) {
		/* read from gpu1, write cmds through gpu1, restore to gpu1 */
		switch_gpu(&info, 1, 2, 2);
		/* note offset at which gpu 1 restore starts */
		restore_gpu1_offset = info.restore_count;
		/* save gpu1-specific regs */
		setup_save_regs(saver, &info,
				ctxsave_regs_3d_pergpu,
				ARRAY_SIZE(ctxsave_regs_3d_pergpu));
	}

	/* read from gpu0, write cmds through gpu1, restore to gpu0+gpu1 */
	if (is_sli)
		switch_gpu(&info, 0, 2, 3);

	if (info.ptr) {
		saver->save_end(info.ptr);
		info.ptr += saver->save_end_size;
	}

	wmb();

	save_size = info.save_count + saver->save_end_size;
	restore_size = info.restore_count + RESTORE_END_SIZE;
	save_incrs = info.save_incrs;
	save_thresh = save_incrs - saver->save_thresh_offset;
	restore_incrs = info.restore_incrs;
}


/*** v0 saver ***/

static void save_push_v0(struct nvhost_cdma *cdma,
			struct nvhost_hwctx *ctx)
{
	nvhost_cdma_push(cdma,
			nvhost_opcode_gather(save_size),
			save_phys);
}

static void __init save_begin_v0(u32 *ptr)
{
	/* 3d: when done, increment syncpt to base+1 */
	ptr[0] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0);
	ptr[1] = nvhost_opcode_imm(0, 0x100 | NVSYNCPT_3D); /* incr 1 */
	/* host: wait for syncpt base+1 */
	ptr[2] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					NV_CLASS_HOST_WAIT_SYNCPT_BASE, 1);
	ptr[3] = nvhost_class_host_wait_syncpt_base(NVSYNCPT_3D,
						NVWAITBASE_3D, 1);
	/* host: signal context read thread to start reading */
	ptr[4] = nvhost_opcode_imm(0, NVSYNCPT_3D); /* incr 2 */
}
#define SAVE_BEGIN_V0_SIZE 5

static void __init save_direct_v0(u32 *ptr, u32 start_reg, u32 count)
{
	ptr[0] = nvhost_opcode_nonincr(NV_CLASS_HOST_INDOFF, 1);
	ptr[1] = nvhost_class_host_indoff_reg_read(NV_HOST_MODULE_GR3D,
						start_reg, true);
	ptr[2] = nvhost_opcode_nonincr(NV_CLASS_HOST_INDDATA, count);
}
#define SAVE_DIRECT_V0_SIZE 3

static void __init save_indirect_v0(u32 *ptr, u32 offset_reg, u32 offset,
			u32 data_reg, u32 count)
{
	ptr[0] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID,
					offset_reg, 1);
	ptr[1] = offset;
	ptr[2] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					NV_CLASS_HOST_INDOFF, 1);
	ptr[3] = nvhost_class_host_indoff_reg_read(NV_HOST_MODULE_GR3D,
						data_reg, false);
	ptr[4] = nvhost_opcode_nonincr(NV_CLASS_HOST_INDDATA, count);
}
#define SAVE_INDIRECT_V0_SIZE 5

static void __init save_end_v0(u32 *ptr)
{
	/* Wait for context read service to finish (cpu incr 3) */
	ptr[0] = nvhost_opcode_nonincr(NV_CLASS_HOST_WAIT_SYNCPT_BASE, 1);
	ptr[1] = nvhost_class_host_wait_syncpt_base(NVSYNCPT_3D,
						NVWAITBASE_3D, save_incrs);
	/* Advance syncpoint base */
	ptr[2] = nvhost_opcode_nonincr(NV_CLASS_HOST_INCR_SYNCPT_BASE, 1);
	ptr[3] = nvhost_class_host_incr_syncpt_base(NVWAITBASE_3D, save_incrs);
	/* set class back to the unit */
	ptr[4] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0);
}
#define SAVE_END_V0_SIZE 5

static void save_registers_from_fifo(u32 *ptr, unsigned int count,
					void __iomem *chan_regs,
					unsigned int *pending)
{
	unsigned int entries = *pending;
	while (count) {
		unsigned int num;

		while (!entries) {
			/* query host for number of entries in fifo */
			entries = nvhost_channel_fifostat_outfentries(
				readl(chan_regs + HOST1X_CHANNEL_FIFOSTAT));
			if (!entries)
				cpu_relax();
			/* TODO: [ahowe 2010-06-14] timeout */
		}
		num = min(entries, count);
		entries -= num;
		count -= num;

		while (num & ~0x3) {
			u32 arr[4];
			arr[0] = readl(chan_regs + HOST1X_CHANNEL_INDDATA);
			arr[1] = readl(chan_regs + HOST1X_CHANNEL_INDDATA);
			arr[2] = readl(chan_regs + HOST1X_CHANNEL_INDDATA);
			arr[3] = readl(chan_regs + HOST1X_CHANNEL_INDDATA);
			memcpy(ptr, arr, 4*sizeof(u32));
			ptr += 4;
			num -= 4;
		}
		while (num--)
			*ptr++ = readl(chan_regs + HOST1X_CHANNEL_INDDATA);
	}
	*pending = entries;
}

static u32 *save_regs_v0(u32 *ptr, unsigned int *pending,
			void __iomem *chan_regs,
			const struct hwctx_reginfo *regs,
			unsigned int nr_regs)
{
	const struct hwctx_reginfo *rend = regs + nr_regs;

	for ( ; regs != rend; ++regs) {
		u32 count = regs->count;
		if (regs->version > 0)
			continue;
		switch (regs->type) {
		case HWCTX_REGINFO_DIRECT:
			ptr += RESTORE_DIRECT_SIZE;
			break;
		case HWCTX_REGINFO_INDIRECT:
		case HWCTX_REGINFO_INDIRECT_4X:
			ptr += RESTORE_INDIRECT_SIZE;
			break;
		}
		save_registers_from_fifo(ptr, count, chan_regs, pending);
		ptr += count;
	}
	return ptr;
}


/*** v1 saver ***/

static void save_push_v1(struct nvhost_cdma *cdma,
			struct nvhost_hwctx *ctx)
{
	/* wait for 3d idle */
	nvhost_cdma_push(cdma,
			nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0),
			nvhost_opcode_imm(0, 0x100 | NVSYNCPT_3D));
	nvhost_cdma_push(cdma,
			nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					NV_CLASS_HOST_WAIT_SYNCPT_BASE, 1),
			nvhost_class_host_wait_syncpt_base(NVSYNCPT_3D,
							NVWAITBASE_3D, 1));
	/* back to 3d */
	nvhost_cdma_push(cdma,
			nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0),
			NVHOST_OPCODE_NOOP);
	/* set gpu1 and gpu0's register read memory output addresses,
	   and send their reads to memory */
	if (s_nr_gpus == 2) {
		nvhost_cdma_push(cdma,
				nvhost_opcode_imm(0xb00, 2),
				nvhost_opcode_imm(0xe40, 1));
		nvhost_cdma_push(cdma,
				nvhost_opcode_nonincr(0x904, 1),
				ctx->restore_phys + restore_gpu1_offset * 4);
#if NV_WAR_789194
		nvhost_cdma_push(cdma,
				NVHOST_OPCODE_NOOP,
				nvhost_opcode_imm(0, 0x200 | NVSYNCPT_3D));
#endif
	}
	nvhost_cdma_push(cdma,
			nvhost_opcode_imm(0xb00, 1),
			nvhost_opcode_imm(0xe40, 1));
	nvhost_cdma_push(cdma,
			nvhost_opcode_nonincr(0x904, 1),
			ctx->restore_phys);
	/* gather the save buffer */
	nvhost_cdma_push(cdma,
			nvhost_opcode_gather(save_size),
			save_phys);
}

static void __init save_begin_v1(u32 *ptr)
{
	ptr[0] = nvhost_opcode_nonincr(0x905, RESTORE_BEGIN_SIZE);
	restore_begin(ptr + 1);
	ptr += RESTORE_BEGIN_SIZE;
}
#define SAVE_BEGIN_V1_SIZE (1 + RESTORE_BEGIN_SIZE)

static void __init save_direct_v1(u32 *ptr, u32 start_reg, u32 count)
{
#if RESTORE_DIRECT_SIZE != 1
#error whoops! code is optimized for RESTORE_DIRECT_SIZE == 1
#endif
	ptr[0] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0x905, 1);
	restore_direct(ptr + 1, start_reg, count);
	ptr += RESTORE_DIRECT_SIZE;
	ptr[1] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					NV_CLASS_HOST_INDOFF, 1);
	ptr[2] = nvhost_class_host_indoff_reg_read(NV_HOST_MODULE_GR3D,
						start_reg, true);
	/* TODO could do this in the setclass if count < 6 */
	ptr[3] = nvhost_opcode_nonincr(NV_CLASS_HOST_INDDATA, count);
}
#define SAVE_DIRECT_V1_SIZE (4 + RESTORE_DIRECT_SIZE)

static void __init save_indirect_v1(u32 *ptr, u32 offset_reg, u32 offset,
			u32 data_reg, u32 count)
{
	ptr[0] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0);
	ptr[1] = nvhost_opcode_nonincr(0x905, RESTORE_INDIRECT_SIZE);
	restore_indirect(ptr + 2, offset_reg, offset, data_reg, count);
	ptr += RESTORE_INDIRECT_SIZE;
	ptr[2] = nvhost_opcode_imm(offset_reg, offset);
	ptr[3] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					NV_CLASS_HOST_INDOFF, 1);
	ptr[4] = nvhost_class_host_indoff_reg_read(NV_HOST_MODULE_GR3D,
						data_reg, false);
	ptr[5] = nvhost_opcode_nonincr(NV_CLASS_HOST_INDDATA, count);
}
#define SAVE_INDIRECT_V1_SIZE (6 + RESTORE_INDIRECT_SIZE)

static void __init save_end_v1(u32 *ptr)
{
#if RESTORE_END_SIZE != 1
#error whoops! code is optimized for RESTORE_END_SIZE == 1
#endif
	/* write end of restore buffer */
	ptr[0] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0x905, 1);
	restore_end(ptr + 1);
	ptr += RESTORE_END_SIZE;
	/* reset to sli if necessary */
#if NV_WAR_789194
	ptr[1] = nvhost_opcode_imm(0, 0x200 | NVSYNCPT_3D);
	ptr += 1;
#endif
	ptr[1] = nvhost_opcode_imm(0xb00, (1 << s_nr_gpus) - 1);
	/* op_done syncpt incr to flush FDC */
	ptr[2] = nvhost_opcode_imm(0, 0x100 | NVSYNCPT_3D);
	/* host wait for that syncpt incr, and advance the wait base */
	ptr[3] = nvhost_opcode_setclass(NV_HOST1X_CLASS_ID,
					NV_CLASS_HOST_WAIT_SYNCPT_BASE,
					nvhost_mask2(
						NV_CLASS_HOST_WAIT_SYNCPT_BASE,
						NV_CLASS_HOST_INCR_SYNCPT_BASE));
	ptr[4] = nvhost_class_host_wait_syncpt_base(NVSYNCPT_3D,
						NVWAITBASE_3D, save_incrs - 1);
	ptr[5] = nvhost_class_host_incr_syncpt_base(NVWAITBASE_3D,
						save_incrs);
	/* set class back to 3d */
	ptr[6] = nvhost_opcode_setclass(NV_GRAPHICS_3D_CLASS_ID, 0, 0);
	/* send reg reads back to host */
	ptr[7] = nvhost_opcode_imm(0xe40, 0);
	/* final syncpt increment to release waiters */
	ptr[8] = nvhost_opcode_imm(0, NVSYNCPT_3D);
}
#if NV_WAR_789194
#define SAVE_END_V1_SIZE (10 + RESTORE_END_SIZE)
#else
#define SAVE_END_V1_SIZE (9 + RESTORE_END_SIZE)
#endif


/*** ctx3d ***/

static struct nvhost_hwctx *ctx3d_alloc_common(struct nvhost_channel *ch,
					bool map_restore)
{
	struct nvmap_client *nvmap = ch->dev->nvmap;
	struct nvhost_hwctx *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;
	ctx->restore = nvmap_alloc(nvmap, restore_size * 4, 32,
		map_restore ? NVMAP_HANDLE_WRITE_COMBINE
			    : NVMAP_HANDLE_UNCACHEABLE);
	if (IS_ERR_OR_NULL(ctx->restore)) {
		kfree(ctx);
		return NULL;
	}

	if (map_restore) {
		ctx->restore_virt = nvmap_mmap(ctx->restore);
		if (!ctx->restore_virt) {
			nvmap_free(nvmap, ctx->restore);
			kfree(ctx);
			return NULL;
		}
	} else {
		ctx->restore_virt = NULL;
	}

	kref_init(&ctx->ref);
	ctx->channel = ch;
	ctx->valid = false;
	ctx->save = save_buf;
	ctx->save_incrs = save_incrs;
	ctx->save_thresh = save_thresh;
	ctx->restore_phys = nvmap_pin(nvmap, ctx->restore);
	ctx->restore_size = restore_size;
	ctx->restore_incrs = restore_incrs;
	return ctx;
}

static struct nvhost_hwctx *ctx3d_alloc_v0(struct nvhost_channel *ch)
{
	struct nvhost_hwctx *ctx = ctx3d_alloc_common(ch, true);
	if (ctx)
		setup_restore_v0(ctx->restore_virt);
	return ctx;
}

static struct nvhost_hwctx *ctx3d_alloc_v1(struct nvhost_channel *ch)
{
	return ctx3d_alloc_common(ch, false);
}

static void ctx3d_get(struct nvhost_hwctx *ctx)
{
	kref_get(&ctx->ref);
}

static void ctx3d_free(struct kref *ref)
{
	struct nvhost_hwctx *ctx = container_of(ref, struct nvhost_hwctx, ref);
	struct nvmap_client *nvmap = ctx->channel->dev->nvmap;

	if (ctx->restore_virt)
		nvmap_munmap(ctx->restore, ctx->restore_virt);
	nvmap_unpin(nvmap, ctx->restore);
	nvmap_free(nvmap, ctx->restore);
	kfree(ctx);
}

static void ctx3d_put(struct nvhost_hwctx *ctx)
{
	kref_put(&ctx->ref, ctx3d_free);
}

static void ctx3d_save_service(struct nvhost_hwctx *ctx)
{
	u32 *ptr = (u32 *)ctx->restore_virt + RESTORE_BEGIN_SIZE;
	unsigned int pending = 0;

	ptr = save_regs_v0(ptr, &pending, ctx->channel->aperture,
			ctxsave_regs_3d_global,
			ARRAY_SIZE(ctxsave_regs_3d_global));

	ptr = save_regs_v0(ptr, &pending, ctx->channel->aperture,
			ctxsave_regs_3d_pergpu,
			ARRAY_SIZE(ctxsave_regs_3d_pergpu));

	wmb();
	nvhost_syncpt_cpu_incr(&ctx->channel->dev->syncpt, NVSYNCPT_3D);
}


/*** savers ***/

static const struct ctx_saver v0_saver __initconst = {
	.version = 0,
	.save_begin = save_begin_v0,
	.save_begin_size = SAVE_BEGIN_V0_SIZE,
	.save_direct = save_direct_v0,
	.save_direct_size = SAVE_DIRECT_V0_SIZE,
	.save_indirect = save_indirect_v0,
	.save_indirect_size = SAVE_INDIRECT_V0_SIZE,
	.save_end = save_end_v0,
	.save_end_size = SAVE_END_V0_SIZE,
	.save_incrs = 3,
	.save_thresh_offset = 1,
	.ctx3d_alloc = ctx3d_alloc_v0,
	.ctx3d_save_push = save_push_v0,
	.ctx3d_save_service = ctx3d_save_service
};

static const struct ctx_saver v1_saver __initconst = {
	.version = 1,
	.save_begin = save_begin_v1,
	.save_begin_size = SAVE_BEGIN_V1_SIZE,
	.save_direct = save_direct_v1,
	.save_direct_size = SAVE_DIRECT_V1_SIZE,
	.save_indirect = save_indirect_v1,
	.save_indirect_size = SAVE_INDIRECT_V1_SIZE,
	.save_end = save_end_v1,
	.save_end_size = SAVE_END_V1_SIZE,
#if NV_WAR_789194
	.save_incrs = 5,
#else
	.save_incrs = 3,
#endif
	.save_thresh_offset = 0,
	.ctx3d_alloc = ctx3d_alloc_v1,
	.ctx3d_save_push = save_push_v1,
	.ctx3d_save_service = NULL
};


/*** nvhost_3dctx ***/

int __init nvhost_3dctx_handler_init(struct nvhost_hwctx_handler *h)
{
	const struct ctx_saver *saver = s_is_v1 ? &v1_saver : &v0_saver;
	struct nvhost_channel *ch;
	struct nvmap_client *nvmap;
	u32 *save_ptr;

	ch = container_of(h, struct nvhost_channel, ctxhandler);
	nvmap = ch->dev->nvmap;

	setup_save(saver, NULL);

	save_buf = nvmap_alloc(nvmap, save_size * 4, 32,
				NVMAP_HANDLE_WRITE_COMBINE);
	if (IS_ERR(save_buf)) {
		int err = PTR_ERR(save_buf);
		save_buf = NULL;
		return err;
	}

	save_ptr = nvmap_mmap(save_buf);
	if (!save_ptr) {
		nvmap_free(nvmap, save_buf);
		save_buf = NULL;
		return -ENOMEM;
	}

	save_phys = nvmap_pin(nvmap, save_buf);

	setup_save(saver, save_ptr);

	h->alloc = saver->ctx3d_alloc;
	h->save_push = saver->ctx3d_save_push;
	h->save_service = saver->ctx3d_save_service;
	h->get = ctx3d_get;
	h->put = ctx3d_put;

	return 0;
}

/* TODO: [ahatala 2010-05-27] */
int __init nvhost_mpectx_handler_init(struct nvhost_hwctx_handler *h)
{
	return 0;
}
