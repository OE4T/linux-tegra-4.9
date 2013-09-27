/*
 *
 * Tegra GK20A GPU Debugger Driver Register Ops
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>
#include <linux/err.h>
#include <linux/nvhost_dbg_gpu_ioctl.h>

#include "dev.h"
#include "nvhost_hwctx.h"
/*#include "nvhost_acm.h"*/
#include "gk20a.h"
#include "gr_gk20a.h"
#include "dbg_gpu_gk20a.h"
#include "regops_gk20a.h"

static bool validate_reg_ops(struct dbg_session_gk20a *dbg_s,
			     u32 *ctx_rd_count, u32 *ctx_wr_count,
			     struct nvhost_dbg_gpu_reg_op *ops,
			     u32 op_count);


int exec_regops_gk20a(struct dbg_session_gk20a *dbg_s,
		      struct nvhost_dbg_gpu_reg_op *ops,
		      u64 num_ops)
{
	int err = 0, i;
	struct channel_gk20a *ch = NULL;
	struct gk20a *g = dbg_s->g;
	/*struct gr_gk20a *gr = &g->gr;*/
	u32 data32_lo = 0, data32_hi = 0;
	u32 ctx_rd_count = 0, ctx_wr_count = 0;
	bool skip_read_lo = false, skip_read_hi = false;
	bool ok;

	nvhost_dbg(dbg_fn | dbg_gpu_dbg, "");

	ch = dbg_s->ch;

	ok = validate_reg_ops(dbg_s,
			      &ctx_rd_count, &ctx_wr_count,
			      ops, num_ops);
	if (!ok) {
		dev_err(dbg_s->dev, "invalid op(s)");
		err = -EINVAL;
		/* each op has its own err/status */
		goto clean_up;
	}

	for (i = 0; i < num_ops; i++) {
		/* if it isn't global then it is done in the ctx ops... */
		if (ops[i].type != REGOP(TYPE_GLOBAL))
			continue;

		switch (ops[i].op) {

		case REGOP(READ_32):
			ops[i].value_hi = 0;
			ops[i].value_lo = gk20a_readl(g, ops[i].offset);
			nvhost_dbg(dbg_gpu_dbg, "read_32 0x%08x from 0x%08x",
				   ops[i].value_lo, ops[i].offset);

			break;

		case REGOP(READ_64):
			ops[i].value_lo = gk20a_readl(g, ops[i].offset);
			ops[i].value_hi =
				gk20a_readl(g, ops[i].offset + 4);

			nvhost_dbg(dbg_gpu_dbg, "read_64 0x%08x:%08x from 0x%08x",
				   ops[i].value_hi, ops[i].value_lo,
				   ops[i].offset);
		break;

		case REGOP(WRITE_32):
		case REGOP(WRITE_64):
			/* some of this appears wonky/unnecessary but
			   we've kept it for compat with existing
			   debugger code.  just in case... */
			if (ops[i].and_n_mask_lo == ~(u32)0) {
				data32_lo = ops[i].value_lo;
				skip_read_lo = true;
			}

			if ((ops[i].op == REGOP(WRITE_64)) &&
			    (ops[i].and_n_mask_hi == ~(u32)0)) {
				data32_hi = ops[i].value_hi;
				skip_read_hi = true;
			}

			/* read first 32bits */
			if (unlikely(skip_read_lo == false)) {
				data32_lo = gk20a_readl(g, ops[i].offset);
				data32_lo &= ~ops[i].and_n_mask_lo;
				data32_lo |= ops[i].value_lo;
			}

			/* if desired, read second 32bits */
			if ((ops[i].op == REGOP(WRITE_64)) &&
			    !skip_read_hi) {
				data32_hi = gk20a_readl(g, ops[i].offset + 4);
				data32_hi &= ~ops[i].and_n_mask_hi;
				data32_hi |= ops[i].value_hi;
			}

			/* now update first 32bits */
			gk20a_writel(g, ops[i].offset, data32_lo);
			nvhost_dbg(dbg_gpu_dbg, "Wrote 0x%08x to 0x%08x ",
				   data32_lo, ops[i].offset);
			/* if desired, update second 32bits */
			if (ops[i].op == REGOP(WRITE_64)) {
				gk20a_writel(g, ops[i].offset + 4, data32_hi);
				nvhost_dbg(dbg_gpu_dbg, "Wrote 0x%08x to 0x%08x ",
					   data32_hi, ops[i].offset + 4);

			}


			break;

		/* shouldn't happen as we've already screened */
		default:
			BUG();
			err = -EINVAL;
			goto clean_up;
			break;
		}
	}

	if (ctx_wr_count | ctx_rd_count) {
		err = gr_gk20a_exec_ctx_ops(ch, ops, num_ops,
					    ctx_wr_count, ctx_rd_count);
		if (err) {
			dev_warn(dbg_s->dev,
				 "failed to perform ctx ops\n");
			goto clean_up;
		}
	}

 clean_up:
	nvhost_dbg(dbg_gpu_dbg, "ret=%d", err);
	return err;

}


static int validate_reg_op_info(struct dbg_session_gk20a *dbg_s,
				struct nvhost_dbg_gpu_reg_op *op)
{
	int err = 0;

	op->status = REGOP(STATUS_SUCCESS);

	switch (op->op) {
	case REGOP(READ_32):
	case REGOP(READ_64):
	case REGOP(WRITE_32):
	case REGOP(WRITE_64):
		break;
	default:
		op->status |= REGOP(STATUS_UNSUPPORTED_OP);
		/*nvhost_err(dbg_s->dev, "Invalid regops op %d!", op->op);*/
		err = -EINVAL;
		break;
	}

	switch (op->type) {
	case REGOP(TYPE_GLOBAL):
	case REGOP(TYPE_GR_CTX):
	case REGOP(TYPE_GR_CTX_TPC):
	case REGOP(TYPE_GR_CTX_SM):
	case REGOP(TYPE_GR_CTX_CROP):
	case REGOP(TYPE_GR_CTX_ZROP):
	case REGOP(TYPE_GR_CTX_QUAD):
		break;
	/*
	case NVHOST_DBG_GPU_REG_OP_TYPE_FB:
	*/
	default:
		op->status |= REGOP(STATUS_INVALID_TYPE);
		/*nvhost_err(dbg_s->dev, "Invalid regops type %d!", op->type);*/
		err = -EINVAL;
		break;
	}

	return err;
}

static int validate_global_regop(struct dbg_session_gk20a *dbg_s,
				 struct nvhost_dbg_gpu_reg_op *op)
{
	return 0;
}

static int validate_context_regop(struct dbg_session_gk20a *dbg_s,
				  struct nvhost_dbg_gpu_reg_op *op)
{
	return 0;
}

static int validate_reg_op_offset(struct dbg_session_gk20a *dbg_s,
				  struct nvhost_dbg_gpu_reg_op *op)
{
	int err;
	u32 buf_offset_lo, buf_offset_addr, num_offsets;
	bool is_ctx_op = reg_op_is_gr_ctx(op->type);

	op->status = 0;
	/*TBD: get this size from the register resource directly */
	if (!is_ctx_op) {
		err = validate_global_regop(dbg_s, op);
		if (err) {
			op->status = REGOP(STATUS_INVALID_OFFSET);
			return -EINVAL;
		}
		return 0;
	}

	/* it's a context-relative op */
	if (!dbg_s->ch) {
		nvhost_err(dbg_s->dev, "can't perform ctx regop unless bound");
		op->status = REGOP(STATUS_UNSUPPORTED_OP);
		return -ENODEV;

	}

	err = validate_context_regop(dbg_s, op);
	if (err) {
		op->status = REGOP(STATUS_INVALID_OFFSET);
		return -EINVAL;
	}

	err = gr_gk20a_get_ctx_buffer_offsets(dbg_s->g,
					      op->offset,
					      1,
					      &buf_offset_lo,
					      &buf_offset_addr,
					      &num_offsets,
					      op->type == REGOP(TYPE_GR_CTX_QUAD),
					      op->quad);
	if (err) {
		op->status |= REGOP(STATUS_INVALID_OFFSET);
		return -EINVAL;
	}
	if (!buf_offset_lo) {
		op->status |= REGOP(STATUS_INVALID_OFFSET);
		return -EINVAL;
	}

	return 0;
}

static bool validate_reg_ops(struct dbg_session_gk20a *dbg_s,
			    u32 *ctx_rd_count, u32 *ctx_wr_count,
			    struct nvhost_dbg_gpu_reg_op *ops,
			    u32 op_count)
{
	u32 i;
	int err;
	bool ok = true;

	/* keep going until the end so every op can get
	 * a separate error code if needed */
	for (i = 0; i < op_count; i++) {

		err = validate_reg_op_info(dbg_s, &ops[i]);
		ok &= !err;

		if (reg_op_is_gr_ctx(ops[i].type)) {
			if (reg_op_is_read(ops[i].op))
				(*ctx_rd_count)++;
			else
				(*ctx_wr_count)++;
		}

		err = validate_reg_op_offset(dbg_s, &ops[i]);
		ok &= !err;
	}

	nvhost_dbg_fn("ctx_wrs:%d ctx_rds:%d\n", *ctx_wr_count, *ctx_rd_count);

	return ok;
}
