/*
 * Copyright (C) 2015-2016, NVIDIA CORPORATION. All rights reserved.
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

/*
 * TODO: The following commented out code has been copied from tegra21x_la.c.
 * The commented out code represents all the features that still need to be
 * added for T18x.
 */
#if 0
#include <asm/io.h>

#include <linux/platform/tegra/clock.h>

#include "la_priv.h"

#define ON_LPDDR4() (tegra_emc_get_dram_type() == DRAM_TYPE_LPDDR4)

#define MC_PTSA_MIN_DEFAULT_MASK				0x3f
#define MC_PTSA_MAX_DEFAULT_MASK				0x3f
#define MC_PTSA_RATE_DEFAULT_MASK				0xfff

#define LA_ST_LA_MINUS_SNAP_ARB_TO_ROW_SRT_EMCCLKS_FP	70000
#define LA_DRAM_WIDTH_BITS				64
#define LA_DISP_CATCHUP_FACTOR_FP			1100
#define MC_MAX_FREQ_MHZ					533
#define MAX_GRANT_DEC					511

#define EXP_TIME_EMCCLKS_FP				88000
#define MAX_LA_NSEC					7650
#define DDA_BW_MARGIN_FP				1100
#define ONE_DDA_FRAC_FP					10
#define CPU_RD_BW_PERC					9
#define CPU_WR_BW_PERC					1
#define MIN_CYCLES_PER_GRANT				2
#define EMEM_PTSA_MINMAX_WIDTH				5
#define RING1_FEEDER_SISO_ALLOC_DIV			2


static unsigned int emc_min_freq_mhz_fp;
static unsigned int emc_min_freq_mhz;
static unsigned int emc_max_freq_mhz;
static unsigned int hi_gd_fp;
static unsigned int lo_gd_fp;
static unsigned int hi_gd_fpa;
static unsigned int lo_gd_fpa;
static unsigned int low_freq_bw;
static unsigned int dda_div;

const struct disp_client *tegra_la_disp_clients_info;
static unsigned int total_dc0_bw;
static unsigned int total_dc1_bw;
static DEFINE_MUTEX(disp_and_camera_ptsa_lock);


/*
 * Gets the memory BW in MBps. @emc_freq should be in MHz.
 */
static u32 get_mem_bw_mbps(u32 dram_freq)
{
	return dram_freq * 16;
}


/*
 * This now also includes ring1 since that needs to have its PTSA updated
 * based on freq and usecase.
 */
static void t18x_calc_disp_and_camera_ptsa(void)
{
	struct ptsa_info *p = &cs->ptsa_info;
	unsigned int ve_bw_fp = cs->camera_bw_array[CAMERA_IDX(VI_W)] *
				DDA_BW_MARGIN_FP;
	unsigned int ve2_bw_fp = 0;
	unsigned int isp_bw_fp = 0;
	unsigned int total_dc0_bw_fp = total_dc0_bw * DDA_BW_MARGIN_FP;
	unsigned int total_dc1_bw_fp = total_dc1_bw * DDA_BW_MARGIN_FP;
	unsigned int low_freq_bw_fp = la_real_to_fp(low_freq_bw);
	unsigned int dis_frac_fp = LA_FPA_TO_FP(lo_gd_fpa * total_dc0_bw_fp /
						low_freq_bw_fp);
	unsigned int disb_frac_fp = LA_FPA_TO_FP(lo_gd_fpa * total_dc1_bw_fp /
						 low_freq_bw_fp);
	unsigned int total_iso_bw_fp = total_dc0_bw_fp + total_dc1_bw_fp;
	int max_max = (1 << EMEM_PTSA_MINMAX_WIDTH) - 1;
	int i = 0;

	if (cs->agg_camera_array[AGG_CAMERA_ID(VE2)].is_hiso) {
		ve2_bw_fp = (cs->camera_bw_array[CAMERA_IDX(ISP_RAB)] +
				cs->camera_bw_array[CAMERA_IDX(ISP_WAB)] +
				cs->camera_bw_array[CAMERA_IDX(ISP_WBB)]) *
				DDA_BW_MARGIN_FP;
	} else {
		ve2_bw_fp = LA_REAL_TO_FP(
				cs->camera_bw_array[CAMERA_IDX(ISP_RAB)] +
				cs->camera_bw_array[CAMERA_IDX(ISP_WAB)] +
				cs->camera_bw_array[CAMERA_IDX(ISP_WBB)]);
	}

	if (cs->agg_camera_array[AGG_CAMERA_ID(ISP)].is_hiso) {
		isp_bw_fp = (cs->camera_bw_array[CAMERA_IDX(ISP_RA)] +
				cs->camera_bw_array[CAMERA_IDX(ISP_WA)] +
				cs->camera_bw_array[CAMERA_IDX(ISP_WB)]) *
				DDA_BW_MARGIN_FP;
	} else {
		isp_bw_fp = LA_REAL_TO_FP(
				cs->camera_bw_array[CAMERA_IDX(ISP_RA)] +
				cs->camera_bw_array[CAMERA_IDX(ISP_WA)] +
				cs->camera_bw_array[CAMERA_IDX(ISP_WB)]);
	}

	cs->agg_camera_array[AGG_CAMERA_ID(VE)].bw_fp = ve_bw_fp;
	cs->agg_camera_array[AGG_CAMERA_ID(VE2)].bw_fp = ve2_bw_fp;
	cs->agg_camera_array[AGG_CAMERA_ID(ISP)].bw_fp = isp_bw_fp;

	for (i = 0; i < TEGRA_LA_AGG_CAMERA_NUM_CLIENTS; i++) {
		struct agg_camera_client_info *agg_client =
						&cs->agg_camera_array[i];

		if (agg_client->is_hiso) {
			agg_client->frac_fp = LA_FPA_TO_FP(
						lo_gd_fpa * agg_client->bw_fp /
						low_freq_bw_fp);
			agg_client->ptsa_min = (unsigned int)(-5) &
						MC_PTSA_MIN_DEFAULT_MASK;
			agg_client->ptsa_max = (unsigned int)(max_max) &
						MC_PTSA_MAX_DEFAULT_MASK;

			total_iso_bw_fp += agg_client->bw_fp;
		} else {
			agg_client->frac_fp = ONE_DDA_FRAC_FP;
			agg_client->ptsa_min = (unsigned int)(-2) &
						MC_PTSA_MIN_DEFAULT_MASK;
			agg_client->ptsa_max = (unsigned int)(0) &
						MC_PTSA_MAX_DEFAULT_MASK;
		}
	}

	MC_SET_INIT_PTSA(p, dis, -5, max_max);
	p->dis_ptsa_rate = fraction2dda_fp(
				dis_frac_fp,
				4,
				MC_PTSA_RATE_DEFAULT_MASK) &
		MC_PTSA_RATE_DEFAULT_MASK;

	MC_SET_INIT_PTSA(p, disb, -5, max_max);
	p->disb_ptsa_rate = fraction2dda_fp(
				disb_frac_fp,
				4,
				MC_PTSA_RATE_DEFAULT_MASK) &
		MC_PTSA_RATE_DEFAULT_MASK;

	p->ve_ptsa_min = cs->agg_camera_array[AGG_CAMERA_ID(VE)].ptsa_min &
					MC_PTSA_MIN_DEFAULT_MASK;
	p->ve_ptsa_max = cs->agg_camera_array[AGG_CAMERA_ID(VE)].ptsa_max &
					MC_PTSA_MAX_DEFAULT_MASK;
	p->ve_ptsa_rate = fraction2dda_fp(
				cs->agg_camera_array[AGG_CAMERA_ID(VE)].frac_fp,
				4,
				MC_PTSA_RATE_DEFAULT_MASK) &
		MC_PTSA_RATE_DEFAULT_MASK;

	p->ve2_ptsa_min = cs->agg_camera_array[AGG_CAMERA_ID(VE2)].ptsa_min &
					MC_PTSA_MIN_DEFAULT_MASK;
	p->ve2_ptsa_max = cs->agg_camera_array[AGG_CAMERA_ID(VE2)].ptsa_max &
					MC_PTSA_MAX_DEFAULT_MASK;
	p->ve2_ptsa_rate = fraction2dda_fp(
			cs->agg_camera_array[AGG_CAMERA_ID(VE2)].frac_fp,
			4,
			MC_PTSA_RATE_DEFAULT_MASK) &
		MC_PTSA_RATE_DEFAULT_MASK;

	p->isp_ptsa_min = cs->agg_camera_array[AGG_CAMERA_ID(ISP)].ptsa_min &
					MC_PTSA_MIN_DEFAULT_MASK;
	p->isp_ptsa_max = cs->agg_camera_array[AGG_CAMERA_ID(ISP)].ptsa_max &
					MC_PTSA_MAX_DEFAULT_MASK;
	p->isp_ptsa_rate = fraction2dda_fp(
			cs->agg_camera_array[AGG_CAMERA_ID(ISP)].frac_fp,
			4,
			MC_PTSA_RATE_DEFAULT_MASK) &
		MC_PTSA_RATE_DEFAULT_MASK;

	MC_SET_INIT_PTSA(p, ring1, -5, max_max);


	p->ring1_ptsa_rate = p->dis_ptsa_rate + p->disb_ptsa_rate +
		p->ve_ptsa_rate;
	p->ring1_ptsa_rate += cs->agg_camera_array[AGG_CAMERA_ID(VE2)].is_hiso ?
		p->ve2_ptsa_rate : 0;
	p->ring1_ptsa_rate += cs->agg_camera_array[AGG_CAMERA_ID(ISP)].is_hiso ?
		p->isp_ptsa_rate : 0;

	if (ON_LPDDR4())
		p->ring1_ptsa_rate /= 2;

	/* These need to be read from the registers since the MC/EMC clock may
	   be different than last time these were read into *p. */
	p->ring1_ptsa_rate += mc_readl(MC_MLL_MPCORER_PTSA_RATE) +
		mc_readl(MC_FTOP_PTSA_RATE);

	if (p->ring1_ptsa_rate == 0)
		p->ring1_ptsa_rate = 0x1;
}

static void t18x_update_display_ptsa_rate(unsigned int *disp_bw_array)
{
	struct ptsa_info *p = &cs->ptsa_info;

	t18x_calc_disp_and_camera_ptsa();

	mc_writel(p->ring1_ptsa_min, MC_RING1_PTSA_MIN);
	mc_writel(p->ring1_ptsa_max, MC_RING1_PTSA_MAX);
	mc_writel(p->ring1_ptsa_rate, MC_RING1_PTSA_RATE);

	mc_writel(p->dis_ptsa_min, MC_DIS_PTSA_MIN);
	mc_writel(p->dis_ptsa_max, MC_DIS_PTSA_MAX);
	mc_writel(p->dis_ptsa_rate, MC_DIS_PTSA_RATE);

	mc_writel(p->disb_ptsa_min, MC_DISB_PTSA_MIN);
	mc_writel(p->disb_ptsa_max, MC_DISB_PTSA_MAX);
	mc_writel(p->disb_ptsa_rate, MC_DISB_PTSA_RATE);
}

static int t18x_update_camera_ptsa_rate(enum tegra_la_id id,
					unsigned int bw_mbps,
					int is_hiso)
{
	struct ptsa_info *p = NULL;
	int ret_code = 0;

	mutex_lock(&disp_and_camera_ptsa_lock);

	if (!is_camera_client(id)) {
		/* Non-camera clients should be handled by t18x_set_la(...) or
		   t18x_set_disp_la(...). */
		pr_err("%s: Ignoring request from a non-camera client.\n",
			__func__);
		pr_err("%s: Non-camera clients should be handled by "
			"t18x_set_la(...) or t18x_set_disp_la(...).\n",
			__func__);
		ret_code = -1;
		goto exit;
	}

	if ((id == ID(VI_W)) &&
		(!is_hiso)) {
		pr_err("%s: VI is stating that its not HISO.\n", __func__);
		pr_err("%s: Ignoring and assuming that VI is HISO because VI "
			"is always supposed to be HISO.\n",
			__func__);
		is_hiso = 1;
	}


	p = &cs->ptsa_info;

	if (id == ID(VI_W)) {
		cs->agg_camera_array[AGG_CAMERA_ID(VE)].is_hiso = is_hiso;
	} else if ((id == ID(ISP_RAB)) ||
			(id == ID(ISP_WAB)) ||
			(id == ID(ISP_WBB))) {
		cs->agg_camera_array[AGG_CAMERA_ID(VE2)].is_hiso = is_hiso;
	} else {
		cs->agg_camera_array[AGG_CAMERA_ID(ISP)].is_hiso = is_hiso;
	}

	cs->camera_bw_array[CAMERA_LA_IDX(id)] = bw_mbps;

	t18x_calc_disp_and_camera_ptsa();

	mc_writel(p->ring1_ptsa_min, MC_RING1_PTSA_MIN);
	mc_writel(p->ring1_ptsa_max, MC_RING1_PTSA_MAX);
	mc_writel(p->ring1_ptsa_rate, MC_RING1_PTSA_RATE);

	mc_writel(p->ve_ptsa_min, MC_VE_PTSA_MIN);
	mc_writel(p->ve_ptsa_max, MC_VE_PTSA_MAX);
	mc_writel(p->ve_ptsa_rate, MC_VE_PTSA_RATE);

	mc_writel(p->ve2_ptsa_min, MC_VE2_PTSA_MIN);
	mc_writel(p->ve2_ptsa_max, MC_VE2_PTSA_MAX);
	mc_writel(p->ve2_ptsa_rate, MC_VE2_PTSA_RATE);

	mc_writel(p->isp_ptsa_min, MC_ISP_PTSA_MIN);
	mc_writel(p->isp_ptsa_max, MC_ISP_PTSA_MAX);
	mc_writel(p->isp_ptsa_rate, MC_ISP_PTSA_RATE);

exit:
	mutex_unlock(&disp_and_camera_ptsa_lock);

	return ret_code;
}


static unsigned int t18x_min_la(struct dc_to_la_params *disp_params)
{
	unsigned int min_la_fp = disp_params->drain_time_usec_fp *
				1000 /
				cs->ns_per_tick;

	/* round up */
	if (min_la_fp % LA_FP_FACTOR != 0)
		min_la_fp += LA_FP_FACTOR;

	return LA_FP_TO_REAL(min_la_fp);
}

static int t18x_handle_disp_la(enum tegra_la_id id,
			       unsigned long emc_freq_hz,
			       unsigned int bw_mbps,
			       struct dc_to_la_params disp_params,
			       int write_la)
{
	int idx = 0;
	struct la_client_info *ci = NULL;
	long long la_to_set = 0;
	unsigned int dvfs_time_nsec = 0;
	unsigned int dvfs_buffering_reqd_bytes = 0;
	unsigned int thresh_dvfs_bytes = 0;
	unsigned int total_buf_sz_bytes = 0;
	int effective_mccif_buf_sz = 0;
	long long la_bw_upper_bound_nsec_fp = 0;
	long long la_bw_upper_bound_nsec = 0;
	long long la_nsec = 0;

	if (!is_display_client(id)) {
		/* Non-display clients should be handled by t18x_set_la(...). */
		return -1;
	}

	mutex_lock(&disp_and_camera_ptsa_lock);
	total_dc0_bw = disp_params.total_dc0_bw;
	total_dc1_bw = disp_params.total_dc1_bw;
	cs->update_display_ptsa_rate(cs->disp_bw_array);
	mutex_unlock(&disp_and_camera_ptsa_lock);

	idx = cs->id_to_index[id];
	ci = &cs->la_info_array[idx];
	la_to_set = 0;
	dvfs_time_nsec =
		tegra_get_dvfs_clk_change_latency_nsec(emc_freq_hz / 1000);
	dvfs_buffering_reqd_bytes = bw_mbps *
					dvfs_time_nsec /
					LA_USEC_TO_NSEC_FACTOR;

	thresh_dvfs_bytes =
			disp_params.thresh_lwm_bytes +
			dvfs_buffering_reqd_bytes +
			disp_params.spool_up_buffering_adj_bytes;
	total_buf_sz_bytes =
		cs->disp_clients[DISP_CLIENT_LA_ID(id)].line_buf_sz_bytes +
		cs->disp_clients[DISP_CLIENT_LA_ID(id)].mccif_size_bytes;
	effective_mccif_buf_sz =
		(cs->disp_clients[DISP_CLIENT_LA_ID(id)].line_buf_sz_bytes >
		thresh_dvfs_bytes) ?
		cs->disp_clients[DISP_CLIENT_LA_ID(id)].mccif_size_bytes :
		total_buf_sz_bytes - thresh_dvfs_bytes;

	if (effective_mccif_buf_sz < 0)
		return -1;

	la_bw_upper_bound_nsec_fp = effective_mccif_buf_sz *
					LA_FP_FACTOR /
					bw_mbps;
	la_bw_upper_bound_nsec_fp = la_bw_upper_bound_nsec_fp *
					LA_FP_FACTOR /
					LA_DISP_CATCHUP_FACTOR_FP;
	la_bw_upper_bound_nsec_fp =
		la_bw_upper_bound_nsec_fp -
		(LA_ST_LA_MINUS_SNAP_ARB_TO_ROW_SRT_EMCCLKS_FP +
		 EXP_TIME_EMCCLKS_FP) /
		(emc_freq_hz / LA_HZ_TO_MHZ_FACTOR);
	la_bw_upper_bound_nsec_fp *= LA_USEC_TO_NSEC_FACTOR;
	la_bw_upper_bound_nsec = LA_FP_TO_REAL(la_bw_upper_bound_nsec_fp);


	la_nsec = min(la_bw_upper_bound_nsec,
			(long long)MAX_LA_NSEC);

	la_to_set = min((long long)(la_nsec/cs->ns_per_tick),
			(long long)MC_LA_MAX_VALUE);

	if ((la_to_set < t18x_min_la(&disp_params)) || (la_to_set > 255))
		return -1;

	if (write_la)
		program_la(ci, la_to_set);
	return 0;
}

static int t18x_set_disp_la(enum tegra_la_id id,
			    unsigned long emc_freq_hz,
			    unsigned int bw_mbps,
			    struct dc_to_la_params disp_params)
{
	return t18x_handle_disp_la(id, emc_freq_hz, bw_mbps, disp_params, 1);
}

static int t18x_check_disp_la(enum tegra_la_id id,
			      unsigned long emc_freq_hz,
			      unsigned int bw_mbps,
			      struct dc_to_la_params disp_params)
{
	return t18x_handle_disp_la(id, emc_freq_hz, bw_mbps, disp_params, 0);
}

#endif


#include <linux/types.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/resource.h>
#include <linux/of_address.h>

#include <asm/io.h>

#include <linux/platform/tegra/tegra_emc.h>
#include <linux/platform/tegra/mc.h>
#include <linux/platform/tegra/latency_allowance.h>
#include <linux/platform/tegra/emc_bwmgr.h>

#include "la_priv.h"


#define EMEM_CHANNEL_ENABLE_MASK		0xf
#define ECC_ENABLE_MASK				0x1
#define T18X_2_STAGE_ECC_ISO_DDA_FACTOR_FP	1400U

/*
 * For T18X we need varying fixed point accuracies. "fp2" variables provide an
 * accuracy of 1/100. And "fp5" variables provide an accuracy of 1/100000.
 */
#define T18X_LA_FP2_FACTOR			100U
#define T18X_LA_REAL_TO_FP2(val)		((val) * T18X_LA_FP2_FACTOR)
#define T18X_LA_FP_TO_FP2(val)			((val) / 10U)
#define T18X_LA_FP5_TO_FPA(val)			((val) / 10U)

#define T18X_LA(a, r, i, ct, sr, la, clk)		\
{							\
	.reg_addr = MC_LATENCY_ALLOWANCE_ ## a,		\
	.mask = MASK(r),				\
	.shift = SHIFT(r),				\
	.id = ID(i),					\
	.name = __stringify(i),				\
	.client_type = TEGRA_LA_ ## ct ## _CLIENT,	\
	.min_scaling_ratio = sr,			\
	.init_la = la,					\
	.la_ref_clk_mhz = clk				\
}

#define T18X_MC_SET_INIT_PTSA_MIN_MAX(p, client, tt, min, max)		\
	do {								\
		(p)->client ## _traffic_type = TEGRA_LA_ ## tt;		\
		(p)->client ## _ptsa_min = (unsigned int)(min) &	\
			MC_PTSA_MIN_DEFAULT_MASK;			\
		(p)->client ## _ptsa_max = (unsigned int)(max) &	\
			MC_PTSA_MAX_DEFAULT_MASK;			\
	} while (0)

#define T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, client, tt, min, max, rate) \
	do {								\
		(p)->client ## _traffic_type = TEGRA_LA_ ## tt;		\
		(p)->client ## _ptsa_min = (unsigned int)(min) &	\
			MC_PTSA_MIN_DEFAULT_MASK;			\
		(p)->client ## _ptsa_max = (unsigned int)(max) &	\
			MC_PTSA_MAX_DEFAULT_MASK;			\
		(p)->client ## _ptsa_rate = (unsigned int)(rate) &	\
			MC_PTSA_RATE_DEFAULT_MASK;			\
	} while (0)

static struct la_chip_specific *cs;
static unsigned int dram_type;
static unsigned int num_channels;
static unsigned int dram_emc_freq_factor;
static unsigned int hi_freq_fp;
static unsigned int lo_freq_fp;
static unsigned int hub_dda_div;
static unsigned int r0_dda_div;
static unsigned int dram_width_bytes;
static unsigned int hi_gd_fpa;
static unsigned int hi_gd_fp5;
static unsigned int lo_gd_fpa;
static unsigned int lo_gd_fp5;
static unsigned int iso_dda_factor_fp;

static struct la_client_info t18x_la_info_array[] = {
	T18X_LA(AFI_0, 7 : 0,  AFIR, DYNAMIC_READ, 0, 105, 250),
	T18X_LA(AFI_0, 23 : 16, AFIW, WRITE, 0, 128, 0),
	T18X_LA(AON_0, 7 : 0, AONR, CONSTANT_READ, 0, 10, 0),
	T18X_LA(AON_0, 23 : 16, AONW, WRITE, 0, 128, 0),
	T18X_LA(AONDMA_0, 7 : 0, AONDMAR, DYNAMIC_READ, 1, 189, 102),
	T18X_LA(AONDMA_0, 23 : 16, AONDMAW, WRITE, 0, 128, 0),
	T18X_LA(APEDMA_0, 7 : 0, APEDMAR, DYNAMIC_READ, 1, 188, 102),
	T18X_LA(APEDMA_0, 23 : 16, APEDMAW, WRITE, 0, 128, 0),
	T18X_LA(APE_0, 7 : 0, APER, CONSTANT_READ, 0, 10, 0),
	T18X_LA(APE_0, 23 : 16, APEW, WRITE, 0, 128, 0),
	T18X_LA(AXIS_0, 7 : 0, AXISR, DYNAMIC_READ, 1, 124, 204),
	T18X_LA(AXIS_0, 23 : 16, AXISW, WRITE, 0, 128, 0),
	T18X_LA(BPMP_0, 7 : 0, BPMPR, CONSTANT_READ, 0, 10, 0),
	T18X_LA(BPMP_0, 23 : 16, BPMPW, WRITE, 0, 128, 0),
	T18X_LA(BPMPDMA_0, 7 : 0, BPMPDMAR, DYNAMIC_READ, 1, 189, 102),
	T18X_LA(BPMPDMA_0, 23 : 16, BPMPDMAW, WRITE, 0, 128, 0),
	T18X_LA(EQOS_0, 7 : 0, EQOSR, DYNAMIC_READ, 0, 42, 600),
	T18X_LA(EQOS_0, 23 : 16, EQOSW, WRITE, 0, 128, 0),
	T18X_LA(ETR_0, 7 : 0, ETRR, CONSTANT_READ, 0, 80, 0),
	T18X_LA(ETR_0, 23 : 16, ETRW, WRITE, 0, 128, 0),
	T18X_LA(GPU_0, 7 : 0, GPUSRD, DYNAMIC_READ, 0, 32, 800),
	T18X_LA(GPU_0, 23 : 16, GPUSWR, WRITE, 0, 128, 0),
	T18X_LA(GPU2_0, 7 : 0, GPUSRD2, DYNAMIC_READ, 0, 32, 800),
	T18X_LA(GPU2_0, 23 : 16, GPUSWR2, WRITE, 0, 128, 0),
	T18X_LA(HDA_0, 7 : 0, HDAR, CONSTANT_READ, 0, 36, 0),
	T18X_LA(HDA_0, 23 : 16, HDAW, WRITE, 0, 128, 0),
	T18X_LA(HC_0, 7 : 0, HOST1X_DMAR, DYNAMIC_READ, 1, 189, 102),
	T18X_LA(ISP2_0, 7 : 0, ISP_RA, DYNAMIC_READ, 0, 83, 307),
	T18X_LA(ISP2_1, 7 : 0, ISP_WA, WRITE, 0, 128, 0),
	T18X_LA(ISP2_1, 23 : 16, ISP_WB, WRITE, 0, 128, 0),
	T18X_LA(MPCORE_0, 7 : 0, MPCORER, CONSTANT_READ, 0, 4, 0),
	T18X_LA(MPCORE_0, 23 : 16, MPCOREW, WRITE, 0, 128, 0),
	T18X_LA(NVDEC_0, 7 : 0, NVDECR, DYNAMIC_READ, 0, 58, 203),
	T18X_LA(NVDEC_0, 23 : 16, NVDECW, WRITE, 0, 128, 0),
	T18X_LA(NVDISPLAY_0, 7 : 0, NVDISPLAYR, DISPLAY_READ, 0, 0, 0),
	T18X_LA(NVENC_0, 7 : 0, NVENCSRD, DYNAMIC_READ, 0, 34, 535),
	T18X_LA(NVENC_0, 23 : 16, NVENCSWR, WRITE, 0, 128, 0),
	T18X_LA(NVJPG_0, 7 : 0, NVJPGSRD, DYNAMIC_READ, 0, 127, 197),
	T18X_LA(NVJPG_0, 23 : 16, NVJPGSWR, WRITE, 0, 128, 0),
	T18X_LA(PTC_0, 7 : 0, PTCR, CONSTANT_READ, 0, 0, 0),
	T18X_LA(SATA_0, 7 : 0, SATAR, DYNAMIC_READ, 1, 57, 450),
	T18X_LA(SATA_0, 23 : 16, SATAW, WRITE, 0, 128, 0),
	T18X_LA(SCE_0, 7 : 0, SCER, CONSTANT_READ, 0, 10, 0),
	T18X_LA(SCE_0, 23 : 16, SCEW, WRITE, 0, 128, 0),
	T18X_LA(SCEDMA_0, 7 : 0, SCEDMAR, DYNAMIC_READ, 1, 189, 102),
	T18X_LA(SCEDMA_0, 23 : 16, SCEDMAW, WRITE, 0, 128, 0),
	T18X_LA(SDMMC_0, 7 : 0, SDMMCR, DYNAMIC_READ, 1, 271, 30),
	T18X_LA(SDMMC_0, 23 : 16, SDMMCW, WRITE, 0, 128, 0),
	T18X_LA(SDMMCA_0, 7 : 0, SDMMCRA, DYNAMIC_READ, 1, 269, 30),
	T18X_LA(SDMMCA_0, 23 : 16, SDMMCWA, WRITE, 0, 128, 0),
	T18X_LA(SDMMCAA_0, 7 : 0, SDMMCRAA, DYNAMIC_READ, 1, 271, 30),
	T18X_LA(SDMMCAA_0, 23 : 16, SDMMCWAA, WRITE, 0, 128, 0),
	T18X_LA(SDMMCAB_0, 7 : 0, SDMMCRAB, DYNAMIC_READ, 1, 241, 67),
	T18X_LA(SDMMCAB_0, 23 : 16, SDMMCWAB, WRITE, 0, 128, 0),
	T18X_LA(SE_0, 7 : 0, SESRD, DYNAMIC_READ, 0, 122, 208),
	T18X_LA(SE_0, 23 : 16, SESWR, WRITE, 0, 128, 0),
	T18X_LA(TSEC_0, 7 : 0, TSECSRD, DYNAMIC_READ, 1, 189, 102),
	T18X_LA(TSEC_0, 23 : 16, TSECSWR, WRITE, 0, 128, 0),
	T18X_LA(TSECB_0, 7 : 0, TSECBSRD, DYNAMIC_READ, 1, 189, 102),
	T18X_LA(TSECB_0, 23 : 16, TSECBSWR, WRITE, 0, 128, 0),
	T18X_LA(UFSHC_0, 7 : 0, UFSHCR, DYNAMIC_READ, 0, 135, 187),
	T18X_LA(UFSHC_0, 23 : 16, UFSHCW, WRITE, 0, 128, 0),
	T18X_LA(VI2_0, 7 : 0, VI_W, WRITE, 0, 128, 0),
	T18X_LA(VIC_0, 7 : 0, VICSRD, DYNAMIC_READ, 0, 32, 800),
	T18X_LA(VIC_0, 23 : 16, VICSWR, WRITE, 0, 128, 0),
	T18X_LA(XUSB_1, 7 : 0, XUSB_DEVR, DYNAMIC_READ, 1, 123, 204),
	T18X_LA(XUSB_1, 23 : 16, XUSB_DEVW, WRITE, 0, 128, 0),
	T18X_LA(XUSB_0, 7 : 0, XUSB_HOSTR, DYNAMIC_READ, 1, 123, 204),
	T18X_LA(XUSB_0, 23 : 16, XUSB_HOSTW, WRITE, 0, 128, 0),

	/* end of list */
	T18X_LA(ROC_DMA_R_0, 0 : 0, MAX_ID, CONSTANT_READ, 0, 0, 0)
};


static inline unsigned int __t18x_fraction2dda_fp(unsigned int fraction_fpa,
					unsigned int div,
					unsigned int mask,
					enum la_traffic_type traffic_type)
{
	unsigned int dda = 0;
	int i = 0;
	unsigned int r = 0;

	fraction_fpa /= div;

	for (i = 0; i < EMEM_PTSA_RATE_WIDTH; i++) {
		fraction_fpa *= 2;
		r = LA_FPA_TO_REAL(fraction_fpa);
		dda = (dda << 1) | (unsigned int)(r);
		fraction_fpa -= LA_REAL_TO_FPA(r);
	}
	if (fraction_fpa > 0) {
		/* Do not round up if the calculated dda is at the mask value
		   already, it will overflow */
		if (dda != mask) {
			if (traffic_type != TEGRA_LA_NISO ||
				fraction_fpa >= 5000 ||
				dda == 0) {
				/* to round up dda value */
				dda++;
			}
		}
	}

	return min(dda & mask, (unsigned int)MAX_DDA_RATE);
}

static inline unsigned int t18x_fraction2dda_fp(unsigned int fraction_fp,
					unsigned int div,
					unsigned int mask,
					enum la_traffic_type traffic_type)
{
	unsigned int fraction_fpa = LA_FP_TO_FPA(fraction_fp);

	return __t18x_fraction2dda_fp(fraction_fpa, div, mask, traffic_type);
}

static inline unsigned int t18x_bw2fraction_fpa(unsigned int bw_mbps)
{
	return (lo_gd_fpa * T18X_LA_REAL_TO_FP2(bw_mbps)) /
		(T18X_LA_FP_TO_FP2(lo_freq_fp) * dram_width_bytes);
}

static void program_ptsa(void)
{
	struct ptsa_info *p = &cs->ptsa_info;

	mc_writel(p->ptsa_grant_dec, MC_PTSA_GRANT_DECREMENT);

	WRITE_PTSA_MIN_MAX_RATE(p, dis, DIS);
	WRITE_PTSA_MIN_MAX_RATE(p, ve, VE);
	WRITE_PTSA_MIN_MAX_RATE(p, isp, ISP);
	WRITE_PTSA_MIN_MAX_RATE(p, apedmapc, APEDMAPC);
	WRITE_PTSA_MIN_MAX_RATE(p, eqospc, EQOSPC);
	WRITE_PTSA_MIN_MAX_RATE(p, ring1_rd_nb, RING1_RD_NB);
	WRITE_PTSA_MIN_MAX_RATE(p, ring1_wr_nb, RING1_WR_NB);
	WRITE_PTSA_MIN_MAX_RATE(p, ring1_rd_b, RING1_RD_B);
	WRITE_PTSA_MIN_MAX_RATE(p, ring1_wr_b, RING1_WR_B);
	WRITE_PTSA_MIN_MAX_RATE(p, ring2, RING2);
	WRITE_PTSA_MIN_MAX_RATE(p, mll_mpcorer, MLL_MPCORER);
	WRITE_PTSA_MIN_MAX_RATE(p, smmu, SMMU_SMMU);
	WRITE_PTSA_MIN_MAX_RATE(p, bpmpdmapc, BPMPDMAPC);

	WRITE_PTSA_MIN_MAX_RATE(p, aondmapc, AONDMAPC);
	WRITE_PTSA_MIN_MAX_RATE(p, aonpc, AONPC);
	WRITE_PTSA_MIN_MAX_RATE(p, apb, APB);
	WRITE_PTSA_MIN_MAX_RATE(p, aud, AUD);
	WRITE_PTSA_MIN_MAX_RATE(p, bpmppc, BPMPPC);
	WRITE_PTSA_MIN_MAX_RATE(p, dfd, DFD);
	WRITE_PTSA_MIN_MAX_RATE(p, ftop, FTOP);
	WRITE_PTSA_MIN_MAX_RATE(p, gk, GK);
	WRITE_PTSA_MIN_MAX_RATE(p, gk2, GK2);
	WRITE_PTSA_MIN_MAX_RATE(p, hdapc, HDAPC);
	WRITE_PTSA_MIN_MAX_RATE(p, host, HOST);
	WRITE_PTSA_MIN_MAX_RATE(p, jpg, JPG);
	WRITE_PTSA_MIN_MAX_RATE(p, mse, MSE);
	WRITE_PTSA_MIN_MAX_RATE(p, mse2, MSE2);
	WRITE_PTSA_MIN_MAX_RATE(p, nic, NIC);
	WRITE_PTSA_MIN_MAX_RATE(p, nvd, NVD);
	WRITE_PTSA_MIN_MAX_RATE(p, nvd3, NVD3);
	WRITE_PTSA_MIN_MAX_RATE(p, pcx, PCX);
	WRITE_PTSA_MIN_MAX_RATE(p, roc_dma_r, ROC_DMA_R);
	WRITE_PTSA_MIN_MAX_RATE(p, sax, SAX);
	WRITE_PTSA_MIN_MAX_RATE(p, scedmapc, SCEDMAPC);
	WRITE_PTSA_MIN_MAX_RATE(p, scepc, SCEPC);
	WRITE_PTSA_MIN_MAX_RATE(p, sd, SD);
	WRITE_PTSA_MIN_MAX_RATE(p, sdm, SDM);
	WRITE_PTSA_MIN_MAX_RATE(p, sdm1, SDM1);
	WRITE_PTSA_MIN_MAX_RATE(p, ufshcpc, UFSHCPC);
	WRITE_PTSA_MIN_MAX_RATE(p, usbd, USBD);
	WRITE_PTSA_MIN_MAX_RATE(p, usbx, USBX);
	WRITE_PTSA_MIN_MAX_RATE(p, vicpc, VICPC);
	WRITE_PTSA_MIN_MAX_RATE(p, vicpc3, VICPC3);

	/* update shadowed registers */
	mc_writel(1, MC_TIMING_CONTROL);
}

static void save_ptsa(void)
{
	struct ptsa_info *p = &cs->ptsa_info;

	p->ptsa_grant_dec = mc_readl(MC_PTSA_GRANT_DECREMENT);

	READ_PTSA_MIN_MAX_RATE(p, dis, DIS);
	READ_PTSA_MIN_MAX_RATE(p, ve, VE);
	READ_PTSA_MIN_MAX_RATE(p, isp, ISP);
	READ_PTSA_MIN_MAX_RATE(p, apedmapc, APEDMAPC);
	READ_PTSA_MIN_MAX_RATE(p, eqospc, EQOSPC);
	READ_PTSA_MIN_MAX_RATE(p, ring1_rd_nb, RING1_RD_NB);
	READ_PTSA_MIN_MAX_RATE(p, ring1_wr_nb, RING1_WR_NB);
	READ_PTSA_MIN_MAX_RATE(p, ring1_rd_b, RING1_RD_B);
	READ_PTSA_MIN_MAX_RATE(p, ring1_wr_b, RING1_WR_B);
	READ_PTSA_MIN_MAX_RATE(p, ring2, RING2);
	READ_PTSA_MIN_MAX_RATE(p, mll_mpcorer, MLL_MPCORER);
	READ_PTSA_MIN_MAX_RATE(p, smmu, SMMU_SMMU);
	READ_PTSA_MIN_MAX_RATE(p, bpmpdmapc, BPMPDMAPC);

	READ_PTSA_MIN_MAX_RATE(p, aondmapc, AONDMAPC);
	READ_PTSA_MIN_MAX_RATE(p, aonpc, AONPC);
	READ_PTSA_MIN_MAX_RATE(p, apb, APB);
	READ_PTSA_MIN_MAX_RATE(p, aud, AUD);
	READ_PTSA_MIN_MAX_RATE(p, bpmppc, BPMPPC);
	READ_PTSA_MIN_MAX_RATE(p, dfd, DFD);
	READ_PTSA_MIN_MAX_RATE(p, ftop, FTOP);
	READ_PTSA_MIN_MAX_RATE(p, gk, GK);
	READ_PTSA_MIN_MAX_RATE(p, gk2, GK2);
	READ_PTSA_MIN_MAX_RATE(p, hdapc, HDAPC);
	READ_PTSA_MIN_MAX_RATE(p, host, HOST);
	READ_PTSA_MIN_MAX_RATE(p, jpg, JPG);
	READ_PTSA_MIN_MAX_RATE(p, mse, MSE);
	READ_PTSA_MIN_MAX_RATE(p, mse2, MSE2);
	READ_PTSA_MIN_MAX_RATE(p, nic, NIC);
	READ_PTSA_MIN_MAX_RATE(p, nvd, NVD);
	READ_PTSA_MIN_MAX_RATE(p, nvd3, NVD3);
	READ_PTSA_MIN_MAX_RATE(p, pcx, PCX);
	READ_PTSA_MIN_MAX_RATE(p, roc_dma_r, ROC_DMA_R);
	READ_PTSA_MIN_MAX_RATE(p, sax, SAX);
	READ_PTSA_MIN_MAX_RATE(p, scedmapc, SCEDMAPC);
	READ_PTSA_MIN_MAX_RATE(p, scepc, SCEPC);
	READ_PTSA_MIN_MAX_RATE(p, sd, SD);
	READ_PTSA_MIN_MAX_RATE(p, sdm, SDM);
	READ_PTSA_MIN_MAX_RATE(p, sdm1, SDM1);
	READ_PTSA_MIN_MAX_RATE(p, ufshcpc, UFSHCPC);
	READ_PTSA_MIN_MAX_RATE(p, usbd, USBD);
	READ_PTSA_MIN_MAX_RATE(p, usbx, USBX);
	READ_PTSA_MIN_MAX_RATE(p, vicpc, VICPC);
	READ_PTSA_MIN_MAX_RATE(p, vicpc3, VICPC3);
}

static void t18x_init_ptsa(void)
{
	unsigned int dram_freq_mhz = tegra_bwmgr_get_emc_rate() /
					LA_HZ_TO_MHZ_FACTOR;
	unsigned int emc_freq_mhz = dram_freq_mhz / dram_emc_freq_factor;
	unsigned int gd_fp5 = (lo_gd_fp5 * T18X_LA_REAL_TO_FP2(dram_freq_mhz)) /
				(T18X_LA_FP_TO_FP2(lo_freq_fp));
	unsigned int gd_fpa = T18X_LA_FP5_TO_FPA(gd_fp5);
	unsigned int gd_int, gd_frac;
	struct ptsa_info *p = &cs->ptsa_info;
	unsigned int ring1_nb_bw;
	unsigned int cpu_rd_bw;
	unsigned int eqos_bw;

	if (gd_fpa >= LA_REAL_TO_FPA(1)) {
		gd_int = 1;
		gd_fpa -= LA_REAL_TO_FPA(1);
	} else {
		gd_int = 0;
	}
	gd_frac = __t18x_fraction2dda_fp(gd_fpa,
					 1,
					 MC_PTSA_RATE_DEFAULT_MASK,
					 TEGRA_LA_HISO);
	p->ptsa_grant_dec = (gd_int << 12) | gd_frac;

	/* initialize PTSA min/max values */
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, aondmapc, SISO, 1, 1, 0);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, aonpc, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, apb, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX(p, apedmapc, HISO, -5, 31);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, aud, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, bpmpdmapc, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, bpmppc, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, dfd, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX(p, dis, HISO, -5, 31);
	T18X_MC_SET_INIT_PTSA_MIN_MAX(p, eqospc, HISO, -5, 31);
	T18X_MC_SET_INIT_PTSA_MIN_MAX(p, ftop, NISO, -2, 0);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, gk, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, gk2, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, hdapc, SISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, host, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, isp, SISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, jpg, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX(p, mll_mpcorer, NISO, -4, 4);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, mse, SISO, 1, 1, 0);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, mse2, SISO, 1, 1, 0);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, nic, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, nvd, SISO, 1, 1, 0);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, nvd3, SISO, 1, 1, 0);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, pcx, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, roc_dma_r, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, ring1_rd_b, NISO, 62, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX(p, ring1_rd_nb, HISO, -5, 31);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, ring1_wr_b, NISO, 62, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX(p, ring1_wr_nb, HISO, -5, 31);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, ring2, NISO, -2, 0, 12);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, sax, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, scedmapc, SISO, 1, 1, 0);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, scepc, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, sd, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, sdm, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, sdm1, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, smmu, NISO, 1, 1, 0);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, ufshcpc, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, usbd, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, usbx, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX(p, ve, HISO, 1, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, vicpc, NISO, -2, 0, 1);
	T18X_MC_SET_INIT_PTSA_MIN_MAX_RATE(p, vicpc3, NISO, -2, 0, 1);


	ring1_nb_bw = emc_freq_mhz * 2 * dram_width_bytes * 70 / 100;
	p->ring1_rd_nb_ptsa_rate =
		__t18x_fraction2dda_fp(t18x_bw2fraction_fpa(ring1_nb_bw),
					r0_dda_div,
					MC_PTSA_RATE_DEFAULT_MASK,
					p->ring1_rd_nb_traffic_type);
	p->ring1_rd_nb_ptsa_rate = max((unsigned int)(1),
					p->ring1_rd_nb_ptsa_rate) &
					MC_PTSA_RATE_DEFAULT_MASK;
	p->ring1_wr_nb_ptsa_rate = p->ring1_rd_nb_ptsa_rate;

	p->ring1_rd_b_ptsa_rate = (unsigned int)(1) & MC_PTSA_RATE_DEFAULT_MASK;
	p->ring1_wr_b_ptsa_rate = (unsigned int)(1) & MC_PTSA_RATE_DEFAULT_MASK;

	p->ring2_ptsa_rate = (unsigned int)(12) & MC_PTSA_RATE_DEFAULT_MASK;

	cpu_rd_bw = emc_freq_mhz * 2 * dram_width_bytes * 10 / 100;
	p->mll_mpcorer_ptsa_rate =
		__t18x_fraction2dda_fp(t18x_bw2fraction_fpa(cpu_rd_bw),
					r0_dda_div,
					MC_PTSA_RATE_DEFAULT_MASK,
					p->mll_mpcorer_traffic_type);
	p->mll_mpcorer_ptsa_rate = max((unsigned int)(1),
					p->mll_mpcorer_ptsa_rate) &
					MC_PTSA_RATE_DEFAULT_MASK;

	p->bpmpdmapc_ptsa_rate = (unsigned int)(1) & MC_PTSA_RATE_DEFAULT_MASK;

	eqos_bw = LA_FP_TO_REAL(250 * T18X_2_STAGE_ECC_ISO_DDA_FACTOR_FP);
	p->eqospc_ptsa_rate =
		__t18x_fraction2dda_fp(t18x_bw2fraction_fpa(eqos_bw),
					hub_dda_div,
					MC_PTSA_RATE_DEFAULT_MASK,
					p->eqospc_traffic_type);

	program_ptsa();
}

static int t18x_set_la(enum tegra_la_id id,
			unsigned int bw_mbps)
{
	int idx = cs->id_to_index[id];
	struct la_client_info *ci = &cs->la_info_array[idx];
	unsigned int emc_freq_mhz = tegra_bwmgr_get_emc_rate() /
					LA_HZ_TO_MHZ_FACTOR /
					dram_emc_freq_factor;
	unsigned int la_to_set_fp = 0, la_to_set = 0;

	if (ci->client_type == TEGRA_LA_DYNAMIC_READ_CLIENT) {
		if (id == ID(NVENCSRD)) {
			la_to_set_fp =
				max(LA_REAL_TO_FP(35),
					(33900 * 535) / emc_freq_mhz);
			la_to_set_fp = min(LA_REAL_TO_FP(MC_LA_MAX_VALUE),
						la_to_set_fp);
			la_to_set = LA_FP_TO_REAL(la_to_set_fp);
		} else if (id == ID(NVDECR)) {
			la_to_set_fp =
				max(LA_REAL_TO_FP(35),
					(LA_REAL_TO_FP(58) * 203) /
					emc_freq_mhz);
			la_to_set_fp = min(LA_REAL_TO_FP(MC_LA_MAX_VALUE),
						la_to_set_fp);
			la_to_set = LA_FP_TO_REAL(la_to_set_fp);
		} else {
			la_to_set_fp =
				max(LA_REAL_TO_FP(ci->min_scaling_ratio),
					LA_REAL_TO_FP(ci->la_ref_clk_mhz) /
					emc_freq_mhz);
			la_to_set_fp =
				min(LA_REAL_TO_FP(MC_LA_MAX_VALUE),
					ci->init_la * la_to_set_fp);
			/* rounding */
			la_to_set_fp += 500;
			la_to_set = LA_FP_TO_REAL(la_to_set_fp);
		}
	} else if (ci->client_type == TEGRA_LA_CONSTANT_READ_CLIENT) {
		la_to_set = ci->init_la;
	} else if (ci->client_type == TEGRA_LA_DISPLAY_READ_CLIENT) {
		/* Display clients should be handled by
		   t18x_set_disp_la(...). */
		return -1;
	} else if (ci->client_type == TEGRA_LA_WRITE_CLIENT) {
		unsigned int emc_period_ns_fp = LA_REAL_TO_FP(1000) /
						emc_freq_mhz;

		la_to_set_fp = min(LA_REAL_TO_FP(MC_LA_MAX_VALUE),
					LA_REAL_TO_FP(128) * emc_period_ns_fp /
					1250);
		la_to_set = LA_FP_TO_REAL(la_to_set_fp);
	} else {
		/* error */
		pr_err("%s: Unknown client type\n",
			__func__);
		return -1;
	}

	program_la(ci, la_to_set);
	return 0;
}

void tegra_la_get_t18x_specific(struct la_chip_specific *cs_la)
{
/*
 * TODO: The following commented out code has been copied from tegra21x_la.c.
 * The commented out code represents all the features that still need to be
 * added for T18x.
 */
#if 0
	int i = 0;

	cs_la->ns_per_tick = 30;
	cs_la->atom_size = 64;
	cs_la->la_max_value = MC_LA_MAX_VALUE;

	cs_la->la_params.fp_factor = LA_FP_FACTOR;
	cs_la->la_params.la_real_to_fp = la_real_to_fp;
	cs_la->la_params.la_fp_to_real = la_fp_to_real;
	cs_la->la_params.static_la_minus_snap_arb_to_row_srt_emcclks_fp =
			LA_ST_LA_MINUS_SNAP_ARB_TO_ROW_SRT_EMCCLKS_FP;
	cs_la->la_params.dram_width_bits = LA_DRAM_WIDTH_BITS;
	cs_la->la_params.disp_catchup_factor_fp = LA_DISP_CATCHUP_FACTOR_FP;

	cs_la->update_display_ptsa_rate = t18x_update_display_ptsa_rate;
	cs_la->update_camera_ptsa_rate = t18x_update_camera_ptsa_rate;
	cs_la->set_disp_la = t18x_set_disp_la;
	cs_la->set_disp_la = t18x_check_disp_la;

	if (ON_LPDDR4()) {
		emc_min_freq_mhz_fp = 25000;
		emc_min_freq_mhz = 25;
		emc_max_freq_mhz = 2132;
		dda_div = 1;
	} else {
		emc_min_freq_mhz_fp = 12500;
		emc_min_freq_mhz = 12;
		emc_max_freq_mhz = 1200;
		dda_div = 2;
	}

	low_freq_bw = emc_min_freq_mhz_fp * 2 * LA_DRAM_WIDTH_BITS / 8;
	low_freq_bw /= 1000;
	tegra_la_disp_clients_info = cs_la->disp_clients;

	/* set some entries to zero */
	for (i = 0; i < NUM_CAMERA_CLIENTS; i++)
		cs_la->camera_bw_array[i] = 0;
	for (i = 0; i < TEGRA_LA_AGG_CAMERA_NUM_CLIENTS; i++) {
		cs_la->agg_camera_array[i].bw_fp = 0;
		cs_la->agg_camera_array[i].frac_fp = 0;
		cs_la->agg_camera_array[i].ptsa_min = 0;
		cs_la->agg_camera_array[i].ptsa_max = 0;
		cs_la->agg_camera_array[i].is_hiso = false;
	}

	/* set mccif_size_bytes values */
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0A)].mccif_size_bytes = 6144;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0B)].mccif_size_bytes = 6144;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0C)].mccif_size_bytes =
									11520;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAYD)].mccif_size_bytes = 4672;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_T)].mccif_size_bytes = 4672;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_HC)].mccif_size_bytes = 4992;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0AB)].mccif_size_bytes =
									11520;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0BB)].mccif_size_bytes =
									6144;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0CB)].mccif_size_bytes =
									6144;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_HCB)].mccif_size_bytes =
									4992;

	/* set line_buf_sz_bytes values */
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0A)].line_buf_sz_bytes =
									151552;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0B)].line_buf_sz_bytes =
									112640;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0C)].line_buf_sz_bytes =
									112640;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAYD)].line_buf_sz_bytes = 18432;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_T)].line_buf_sz_bytes =
									18432;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_HC)].line_buf_sz_bytes = 320;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0AB)].line_buf_sz_bytes =
									112640;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0BB)].line_buf_sz_bytes =
									112640;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0CB)].line_buf_sz_bytes =
									112640;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_HCB)].line_buf_sz_bytes =
									320;

	/* set win_type values */
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0A)].win_type =
						TEGRA_LA_DISP_WIN_TYPE_FULL;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0B)].win_type =
						TEGRA_LA_DISP_WIN_TYPE_FULLA;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0C)].win_type =
						TEGRA_LA_DISP_WIN_TYPE_FULLA;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAYD)].win_type =
						TEGRA_LA_DISP_WIN_TYPE_SIMPLE;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_HC)].win_type =
						TEGRA_LA_DISP_WIN_TYPE_CURSOR;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_T)].win_type =
						TEGRA_LA_DISP_WIN_TYPE_SIMPLE;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0AB)].win_type =
						TEGRA_LA_DISP_WIN_TYPE_FULLB;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0BB)].win_type =
						TEGRA_LA_DISP_WIN_TYPE_FULLB;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_0CB)].win_type =
						TEGRA_LA_DISP_WIN_TYPE_FULLB;
	cs_la->disp_clients[DISP_CLIENT_ID(DISPLAY_HCB)].win_type =
						TEGRA_LA_DISP_WIN_TYPE_CURSOR;
#endif

	int i;
	unsigned int channel_enable;
	unsigned int adj_lo_freq_fp;
	int dram_ecc_enabled;
	unsigned int client_traffic_type_config_2;

	cs_la->la_info_array = t18x_la_info_array;
	cs_la->la_info_array_size = ARRAY_SIZE(t18x_la_info_array);

	cs_la->init_ptsa = t18x_init_ptsa;
	cs_la->set_la = t18x_set_la;
	cs_la->save_ptsa = save_ptsa;
	cs_la->program_ptsa = program_ptsa;
	cs_la->suspend = la_suspend;
	cs_la->resume = la_resume;
	cs = cs_la;

	dram_type = tegra_emc_get_dram_type();

	channel_enable = mc_readl(MC_EMEM_ADR_CFG_CHANNEL_ENABLE) &
					EMEM_CHANNEL_ENABLE_MASK;
	num_channels = 0;
	for (i = 0; i < 4; i++) {
		if (channel_enable & 0x1)
			num_channels++;
		channel_enable >>= 1;
	}

	dram_emc_freq_factor = 1;
	hi_gd_fpa = 14998;
	hi_gd_fp5 = 149976;

	if (dram_type == DRAM_TYPE_DDR3) {
		hi_freq_fp = LA_REAL_TO_FP(1200);
		lo_freq_fp = 12500;
		hub_dda_div = 2;
		r0_dda_div = 1;
		hi_gd_fpa = 19998;
		hi_gd_fp5 = 199976;

		if (num_channels == 1)
			dram_width_bytes = 16;
		else if (num_channels == 2)
			dram_width_bytes = 32;
	} else if (dram_type == DRAM_TYPE_LPDDR4) {
		dram_emc_freq_factor = 2;
		hi_freq_fp = LA_REAL_TO_FP(2132);
		lo_freq_fp = LA_REAL_TO_FP(25);
		hub_dda_div = 1;
		r0_dda_div = 2;

		if (num_channels == 2)
			dram_width_bytes = 16;
		else if (num_channels == 4)
			dram_width_bytes = 32;
	}

	adj_lo_freq_fp = lo_freq_fp / 2;
	lo_gd_fpa = (hi_gd_fpa * adj_lo_freq_fp) / (hi_freq_fp / 2);
	lo_gd_fp5 = (hi_gd_fp5 * adj_lo_freq_fp) / (hi_freq_fp / 2);

	dram_ecc_enabled = mc_readl(MC_ECC_CONTROL) & ECC_ENABLE_MASK;
	if (dram_ecc_enabled)
		iso_dda_factor_fp = 1400;
	else
		iso_dda_factor_fp = 1200;

	/* Make ISPWA and ISPWB blocking clients */
	client_traffic_type_config_2 =
				mc_readl(MC_CLIENT_TRAFFIC_TYPE_CONFIG_2);
	mc_writel(client_traffic_type_config_2 | 0xc0,
			MC_CLIENT_TRAFFIC_TYPE_CONFIG_2);

	/* Set arbiter iso client types */
	mc_writel(0x1, MC_EMEM_ARB_ISOCHRONOUS_0);
	mc_writel(0x0, MC_EMEM_ARB_ISOCHRONOUS_1);
	mc_writel(0x0, MC_EMEM_ARB_ISOCHRONOUS_2);
	mc_writel(0x0, MC_EMEM_ARB_ISOCHRONOUS_3);
	mc_writel(0x80044000, MC_EMEM_ARB_ISOCHRONOUS_4);
	mc_writel(0x2, MC_EMEM_ARB_ISOCHRONOUS_5);
}
