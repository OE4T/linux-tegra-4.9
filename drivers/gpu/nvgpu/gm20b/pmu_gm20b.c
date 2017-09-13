/*
 * GM20B PMU
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
*
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <nvgpu/timers.h>
#include <nvgpu/pmu.h>
#include <nvgpu/fuse.h>
#include <nvgpu/enabled.h>

#include "gk20a/gk20a.h"
#include "gk20a/pmu_gk20a.h"

#include "acr_gm20b.h"
#include "pmu_gm20b.h"

#include <nvgpu/hw/gm20b/hw_gr_gm20b.h>
#include <nvgpu/hw/gm20b/hw_pwr_gm20b.h>
#include <nvgpu/hw/gm20b/hw_fuse_gm20b.h>

/*!
 * Structure/object which single register write need to be done during PG init
 * sequence to set PROD values.
 */
struct pg_init_sequence_list {
	u32 regaddr;
	u32 writeval;
};

#define gm20b_dbg_pmu(fmt, arg...) \
	gk20a_dbg(gpu_dbg_pmu, fmt, ##arg)


/* PROD settings for ELPG sequencing registers*/
static struct pg_init_sequence_list _pginitseq_gm20b[] = {
		{ 0x0010ab10, 0x8180},
		{ 0x0010e118, 0x83828180},
		{ 0x0010e068, 0},
		{ 0x0010e06c, 0x00000080},
		{ 0x0010e06c, 0x00000081},
		{ 0x0010e06c, 0x00000082},
		{ 0x0010e06c, 0x00000083},
		{ 0x0010e06c, 0x00000084},
		{ 0x0010e06c, 0x00000085},
		{ 0x0010e06c, 0x00000086},
		{ 0x0010e06c, 0x00000087},
		{ 0x0010e06c, 0x00000088},
		{ 0x0010e06c, 0x00000089},
		{ 0x0010e06c, 0x0000008a},
		{ 0x0010e06c, 0x0000008b},
		{ 0x0010e06c, 0x0000008c},
		{ 0x0010e06c, 0x0000008d},
		{ 0x0010e06c, 0x0000008e},
		{ 0x0010e06c, 0x0000008f},
		{ 0x0010e06c, 0x00000090},
		{ 0x0010e06c, 0x00000091},
		{ 0x0010e06c, 0x00000092},
		{ 0x0010e06c, 0x00000093},
		{ 0x0010e06c, 0x00000094},
		{ 0x0010e06c, 0x00000095},
		{ 0x0010e06c, 0x00000096},
		{ 0x0010e06c, 0x00000097},
		{ 0x0010e06c, 0x00000098},
		{ 0x0010e06c, 0x00000099},
		{ 0x0010e06c, 0x0000009a},
		{ 0x0010e06c, 0x0000009b},
		{ 0x0010ab14, 0x00000000},
		{ 0x0010ab18, 0x00000000},
		{ 0x0010e024, 0x00000000},
		{ 0x0010e028, 0x00000000},
		{ 0x0010e11c, 0x00000000},
		{ 0x0010e120, 0x00000000},
		{ 0x0010ab1c, 0x02010155},
		{ 0x0010e020, 0x001b1b55},
		{ 0x0010e124, 0x01030355},
		{ 0x0010ab20, 0x89abcdef},
		{ 0x0010ab24, 0x00000000},
		{ 0x0010e02c, 0x89abcdef},
		{ 0x0010e030, 0x00000000},
		{ 0x0010e128, 0x89abcdef},
		{ 0x0010e12c, 0x00000000},
		{ 0x0010ab28, 0x74444444},
		{ 0x0010ab2c, 0x70000000},
		{ 0x0010e034, 0x74444444},
		{ 0x0010e038, 0x70000000},
		{ 0x0010e130, 0x74444444},
		{ 0x0010e134, 0x70000000},
		{ 0x0010ab30, 0x00000000},
		{ 0x0010ab34, 0x00000001},
		{ 0x00020004, 0x00000000},
		{ 0x0010e138, 0x00000000},
		{ 0x0010e040, 0x00000000},
};

int gm20b_pmu_setup_elpg(struct gk20a *g)
{
	int ret = 0;
	u32 reg_writes;
	u32 index;

	gk20a_dbg_fn("");

	if (g->elpg_enabled) {
		reg_writes = ((sizeof(_pginitseq_gm20b) /
				sizeof((_pginitseq_gm20b)[0])));
		/* Initialize registers with production values*/
		for (index = 0; index < reg_writes; index++) {
			gk20a_writel(g, _pginitseq_gm20b[index].regaddr,
				_pginitseq_gm20b[index].writeval);
		}
	}

	gk20a_dbg_fn("done");
	return ret;
}

static void pmu_handle_acr_init_wpr_msg(struct gk20a *g, struct pmu_msg *msg,
			void *param, u32 handle, u32 status)
{
	gk20a_dbg_fn("");

	gm20b_dbg_pmu("reply PMU_ACR_CMD_ID_INIT_WPR_REGION");

	if (msg->msg.acr.acrmsg.errorcode == PMU_ACR_SUCCESS)
		g->pmu_lsf_pmu_wpr_init_done = 1;
	gk20a_dbg_fn("done");
}


int gm20b_pmu_init_acr(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_cmd cmd;
	u32 seq;

	gk20a_dbg_fn("");

	/* init ACR */
	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = PMU_UNIT_ACR;
	cmd.hdr.size = PMU_CMD_HDR_SIZE +
	  sizeof(struct pmu_acr_cmd_init_wpr_details);
	cmd.cmd.acr.init_wpr.cmd_type = PMU_ACR_CMD_ID_INIT_WPR_REGION;
	cmd.cmd.acr.init_wpr.regionid = 0x01;
	cmd.cmd.acr.init_wpr.wproffset = 0x00;
	gm20b_dbg_pmu("cmd post PMU_ACR_CMD_ID_INIT_WPR_REGION");
	nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
			pmu_handle_acr_init_wpr_msg, pmu, &seq, ~0);

	gk20a_dbg_fn("done");
	return 0;
}

void pmu_handle_fecs_boot_acr_msg(struct gk20a *g, struct pmu_msg *msg,
			void *param, u32 handle, u32 status)
{

	gk20a_dbg_fn("");


	gm20b_dbg_pmu("reply PMU_ACR_CMD_ID_BOOTSTRAP_FALCON");

	gm20b_dbg_pmu("response code = %x\n", msg->msg.acr.acrmsg.falconid);
	g->pmu_lsf_loaded_falcon_id = msg->msg.acr.acrmsg.falconid;
	gk20a_dbg_fn("done");
}

static int pmu_gm20b_ctx_wait_lsf_ready(struct gk20a *g, u32 timeout_ms,
					u32 val)
{
	unsigned long delay = GR_FECS_POLL_INTERVAL;
	u32 reg;
	struct nvgpu_timeout timeout;

	gk20a_dbg_fn("");
	reg = gk20a_readl(g, gr_fecs_ctxsw_mailbox_r(0));

	nvgpu_timeout_init(g, &timeout, timeout_ms, NVGPU_TIMER_CPU_TIMER);

	do {
		reg = gk20a_readl(g, gr_fecs_ctxsw_mailbox_r(0));
		if (reg == val)
			return 0;
		nvgpu_udelay(delay);
	} while (!nvgpu_timeout_expired(&timeout));

	return -ETIMEDOUT;
}

void gm20b_pmu_load_lsf(struct gk20a *g, u32 falcon_id, u32 flags)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_cmd cmd;
	u32 seq;

	gk20a_dbg_fn("");

	gm20b_dbg_pmu("wprinit status = %x\n", g->pmu_lsf_pmu_wpr_init_done);
	if (g->pmu_lsf_pmu_wpr_init_done) {
		/* send message to load FECS falcon */
		memset(&cmd, 0, sizeof(struct pmu_cmd));
		cmd.hdr.unit_id = PMU_UNIT_ACR;
		cmd.hdr.size = PMU_CMD_HDR_SIZE +
		  sizeof(struct pmu_acr_cmd_bootstrap_falcon);
		cmd.cmd.acr.bootstrap_falcon.cmd_type =
		  PMU_ACR_CMD_ID_BOOTSTRAP_FALCON;
		cmd.cmd.acr.bootstrap_falcon.flags = flags;
		cmd.cmd.acr.bootstrap_falcon.falconid = falcon_id;
		gm20b_dbg_pmu("cmd post PMU_ACR_CMD_ID_BOOTSTRAP_FALCON: %x\n",
				falcon_id);
		nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
				pmu_handle_fecs_boot_acr_msg, pmu, &seq, ~0);
	}

	gk20a_dbg_fn("done");
	return;
}

int gm20b_load_falcon_ucode(struct gk20a *g, u32 falconidmask)
{
	u32  err = 0;
	u32 flags = PMU_ACR_CMD_BOOTSTRAP_FALCON_FLAGS_RESET_YES;
	unsigned long timeout = gk20a_get_gr_idle_timeout(g);

	/* GM20B PMU supports loading FECS only */
	if (!(falconidmask == (1 << LSF_FALCON_ID_FECS)))
		return -EINVAL;
	/* check whether pmu is ready to bootstrap lsf if not wait for it */
	if (!g->pmu_lsf_pmu_wpr_init_done) {
		pmu_wait_message_cond(&g->pmu,
				gk20a_get_gr_idle_timeout(g),
				&g->pmu_lsf_pmu_wpr_init_done, 1);
		/* check again if it still not ready indicate an error */
		if (!g->pmu_lsf_pmu_wpr_init_done) {
			nvgpu_err(g, "PMU not ready to load LSF");
			return -ETIMEDOUT;
		}
	}
	/* load FECS */
	gk20a_writel(g,
		gr_fecs_ctxsw_mailbox_clear_r(0), ~0x0);
	gm20b_pmu_load_lsf(g, LSF_FALCON_ID_FECS, flags);
	err = pmu_gm20b_ctx_wait_lsf_ready(g, timeout,
			0x55AA55AA);
	return err;
}

void gm20b_write_dmatrfbase(struct gk20a *g, u32 addr)
{
	gk20a_writel(g, pwr_falcon_dmatrfbase_r(), addr);
}

/*Dump Security related fuses*/
void pmu_dump_security_fuses_gm20b(struct gk20a *g)
{
	u32 val;

	nvgpu_err(g, "FUSE_OPT_SEC_DEBUG_EN_0: 0x%x",
			gk20a_readl(g, fuse_opt_sec_debug_en_r()));
	nvgpu_err(g, "FUSE_OPT_PRIV_SEC_EN_0: 0x%x",
			gk20a_readl(g, fuse_opt_priv_sec_en_r()));
	nvgpu_tegra_fuse_read_gcplex_config_fuse(g, &val);
	nvgpu_err(g, "FUSE_GCPLEX_CONFIG_FUSE_0: 0x%x", val);
}
