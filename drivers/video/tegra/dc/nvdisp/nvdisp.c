/*
 * drivers/video/tegra/dc/nvdisplay/nvdisp.c
 *
 * Copyright (c) 2014-2015, NVIDIA CORPORATION, All rights reserved.
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


#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/dma-mapping.h>

#include <mach/dc.h>
#include <mach/fb.h>

#include "nvdisp.h"
#include "nvdisp_priv.h"
#include "dc_config.h"
#include "dc_priv.h"
#include "dp.h"
#include "hw_nvdisp_nvdisp.h"
#include "hw_win_nvdisp.h"

/* static struct tegra_dc_nvdisp	tegra_nvdisp; */
DEFINE_MUTEX(tegra_nvdisp_lock);

#define NVDISP_INPUT_LUT_SIZE   257
#define NVDISP_OUTPUT_LUT_SIZE  1025

int tegra_nvdisp_set_output_lut(struct tegra_dc *dc,
					struct tegra_dc_lut *lut)
{
	tegra_dc_writel(dc,
			tegra_dc_reg_l32(lut->phy_addr),
			nvdisp_output_lut_base_r());
	tegra_dc_writel(dc,
			tegra_dc_reg_h32(lut->phy_addr),
			nvdisp_output_lut_base_hi_r());
	tegra_dc_writel(dc, nvdisp_output_lut_ctl_size_1025_f(),
			nvdisp_output_lut_ctl_r());

	return 0;
}

static int nvdisp_allocate_lut(struct tegra_dc *dc,
				struct tegra_dc_lut *lut)
{
	if (!lut)
		return -ENOMEM;

	/* Allocate the memory for LUT */
	lut->size = NVDISP_OUTPUT_LUT_SIZE * sizeof(u64);
	lut->rgb = (u64 *)dma_zalloc_coherent(&dc->ndev->dev, lut->size,
			&lut->phy_addr, GFP_KERNEL);
	if (!lut->rgb)
		return -ENOMEM;

	return 0;
}

static int nvdisp_alloc_output_lut(struct tegra_dc *dc)
{
	struct tegra_dc_lut *lut;

	lut = &dc->cmu;

	if (nvdisp_allocate_lut(dc, lut))
		return -ENOMEM;

	/* Set the LUT address in HW register */
	tegra_nvdisp_set_output_lut(dc, lut);

	return 0;

}

static int nvdisp_alloc_input_lut(struct tegra_dc *dc,
					struct tegra_dc_win *win,
					bool winlut)
{
	struct tegra_dc_lut *lut;

	if (winlut)
		lut = &win->lut;
	else
		lut = &dc->fb_lut;

	if (nvdisp_allocate_lut(dc, lut))
		return -ENOMEM;

	return 0;
}

static int _tegra_nvdisp_init_once(struct tegra_dc *dc)
{
	int ret = 0;
	int i;
	char syncpt_name[] = "disp_a";

	mutex_lock(&tegra_nvdisp_lock);

	/* Init sycpt ids */
	dc->valid_windows = 0x3f; /* Assign all windows to this head */
	for (i = 0; i < DC_N_WINDOWS; ++i, ++syncpt_name[5]) {
		struct tegra_dc_win *win = tegra_dc_get_window(dc, i);

		win->syncpt.id = nvhost_get_syncpt_client_managed(dc->ndev,
								syncpt_name);

		/* allocate input LUT memory and assign to HW */
		if (nvdisp_alloc_input_lut(dc, win, true))
			goto INIT_ERR;

		/* init default CSC */
		tegra_nvdisp_init_csc_defaults(&win->csc);
	}

	dc->valid_windows = 0;

	goto INIT_EXIT;

INIT_ERR:
	for (i = 0; i < DC_N_WINDOWS; ++i) {
		struct tegra_dc_lut *lut;
		struct tegra_dc_win *win = tegra_dc_get_window(dc, i);

		/* Allocate the memory for Input LUT & fb LUT*/
		lut = &win->lut;
		if (lut->rgb)
			dma_free_coherent(&dc->ndev->dev, lut->size,
				(void *)lut->rgb, lut->phy_addr);
	}

INIT_EXIT:
	mutex_unlock(&tegra_nvdisp_lock);
	return ret;

}

int tegra_nvdisp_program_mode(struct tegra_dc *dc, struct tegra_dc_mode
				     *mode)
{
	unsigned long v_back_porch;
	unsigned long v_front_porch;
	unsigned long v_sync_width;
	unsigned long v_active;

	if (!dc->mode.pclk)
		return 0;

	v_back_porch = mode->v_back_porch;
	v_front_porch = mode->v_front_porch;
	v_sync_width = mode->v_sync_width;
	v_active = mode->v_active;

	if (mode->vmode == FB_VMODE_INTERLACED) {
		v_back_porch /= 2;
		v_front_porch /= 2;
		v_sync_width /= 2;
		v_active /= 2;
	}

	tegra_dc_get(dc);

	/* IMP related updates */
	dc->new_bw_kbps = tegra_dc_calc_min_bandwidth(dc);
	tegra_dc_program_bandwidth(dc, true);

	tegra_dc_writel(dc,
		nvdisp_sync_width_h_f(mode->h_sync_width) |
		nvdisp_sync_width_v_f(v_sync_width),
		nvdisp_sync_width_r());
	if ((dc->out->type == TEGRA_DC_OUT_DP) ||
		(dc->out->type == TEGRA_DC_OUT_FAKE_DP) ||
		(dc->out->type == TEGRA_DC_OUT_NVSR_DP) ||
		(dc->out->type == TEGRA_DC_OUT_LVDS)) {
		tegra_dc_writel(dc,
			nvdisp_back_porch_h_f(mode->h_back_porch) |
			nvdisp_back_porch_v_f(
				(v_back_porch - mode->v_ref_to_sync)),
			nvdisp_back_porch_r());
		tegra_dc_writel(dc,
			nvdisp_front_porch_h_f(mode->h_front_porch) |
			nvdisp_front_porch_v_f(
				(v_front_porch + mode->v_ref_to_sync)),
			nvdisp_front_porch_r());
	} else {
		tegra_dc_writel(dc,
			nvdisp_back_porch_h_f(mode->h_back_porch) |
			nvdisp_back_porch_v_f(v_back_porch),
			nvdisp_back_porch_r());
		tegra_dc_writel(dc,
			nvdisp_front_porch_h_f(mode->h_front_porch) |
			nvdisp_front_porch_v_f(v_front_porch),
			nvdisp_front_porch_r());
	}
	tegra_dc_writel(dc,
			nvdisp_active_h_f(mode->h_active) |
			nvdisp_active_v_f(v_active),
			nvdisp_active_r());


#if defined(CONFIG_TEGRA_DC_INTERLACE)
	if (mode->vmode == FB_VMODE_INTERLACED)
		tegra_dc_writel(dc, INTERLACE_MODE_ENABLE |
			INTERLACE_START_FIELD_1
			| INTERLACE_STATUS_FIELD_1,
			nvdisp_interlace_ctl_r());
	else
		tegra_dc_writel(dc, INTERLACE_MODE_DISABLE,
			nvdisp_interlace_ctl_r());

	if (mode->vmode == FB_VMODE_INTERLACED) {
		tegra_dc_writel(dc,
			nvdisp_interlace_fld2_width_v_f(v_sync_width),
			nvdisp_interlace_fld2_width_r());
		tegra_dc_writel(dc,
			nvdisp_interlace_fld2_bporch_v_f(v_back_porch + 1),
			nvdisp_interlace_fld2_bporch_r());
		tegra_dc_writel(dc,
			nvdisp_interlace_fld2_active_v_f(v_active),
			nvdisp_interlace_fld2_active_r());
		tegra_dc_writel(dc,
			nvdisp_interlace_fld2_fporch_v_f(v_front_porch),
			nvdisp_interlace_fld2_fporch_r());
	}
#endif

	/* TODO: MIPI/CRT/HDMI clock cals */
	/* TODO: confirm shift clock still exists in Parker */

#ifdef CONFIG_SWITCH
	switch_set_state(&dc->modeset_switch,
			 (mode->h_active << 16) | mode->v_active);
#endif

	tegra_dc_writel(dc, nvdisp_cmd_state_ctrl_general_act_req_enable_f(),
			nvdisp_cmd_state_ctrl_r());

	if (dc->out_ops && dc->out_ops->modeset_notifier)
		dc->out_ops->modeset_notifier(dc);

	tegra_dc_put(dc);

	dc->mode_dirty = false;

	trace_display_mode(dc, &dc->mode);
	return 0;
}


int tegra_nvdisp_init(struct tegra_dc *dc)
{
	/*Lut alloc is needed per dc */
	if (!dc->fb_lut.rgb) {
		if (nvdisp_alloc_input_lut(dc, NULL, false))
			return -ENOMEM;
	}

	/* Output LUT is needed per dc */
	if (!(dc->cmu.rgb)) {
		if (nvdisp_alloc_output_lut(dc))
			return -ENOMEM;
	}

	/* Only need init once no matter how many dc objects */
	if (dc->ctrl_num)
		return 0;

	return _tegra_nvdisp_init_once(dc);
}

static int tegra_nvdisp_set_control(struct tegra_dc *dc)
{
	u32 protocol = nvdisp_sor_control_protocol_custom_f();
	u32 reg      = nvdisp_sor_control_r();

	/* Set the protocol type in DT and use from there
	 * Current setting are default ones.
	 */

	if (dc->out->type == TEGRA_DC_OUT_HDMI)	{
		protocol = nvdisp_sor1_control_protocol_tmdsa_f();
		reg = nvdisp_sor1_control_r();
	} else if ((dc->out->type == TEGRA_DC_OUT_DP) ||
		(dc->out->type == TEGRA_DC_OUT_NVSR_DP) ||
		(dc->out->type == TEGRA_DC_OUT_FAKE_DP)) {
		protocol = nvdisp_sor_control_protocol_dpa_f();
		reg = nvdisp_sor_control_r();
	} else if ((dc->out->type == TEGRA_DC_OUT_DSI) ||
		(dc->out->type == TEGRA_DC_OUT_FAKE_DSIA) ||
		(dc->out->type == TEGRA_DC_OUT_FAKE_DSIB) ||
		(dc->out->type == TEGRA_DC_OUT_FAKE_DSI_GANGED)) {
		protocol = nvdisp_dsi_control_protocol_dsia_f();
		reg = nvdisp_dsi_control_r();
	}

	tegra_dc_writel(dc, protocol, reg);
	tegra_dc_enable_general_act(dc);
	return 0;
}

static int tegra_nvdisp_head_init(struct tegra_dc *dc)
{
	u32 int_enable;
	u32 int_mask;
	u32 i, val;

	/* Init syncpt */
	tegra_dc_writel(dc, nvdisp_incr_syncpt_cntrl_no_stall_f(1),
		nvdisp_incr_syncpt_cntrl_r());

	/* Disabled this feature as unit fpga hang on enabling this*/
	if (!tegra_platform_is_linsim())
		tegra_dc_writel(dc, nvdisp_cont_syncpt_vsync_en_enable_f() |
			(NVSYNCPT_VBLANK0 + dc->ctrl_num),
			nvdisp_cont_syncpt_vsync_r());

	/* Init interrupts */
	/* Setting Int type */
	tegra_dc_writel(dc, 0x3C001004, nvdisp_int_type_r());
	/* Setting all the Int polarity to high */
	tegra_dc_writel(dc, 0x3D8010F6, nvdisp_int_polarity_r());

	/* enable interrupts for vblank, frame_end and underflows */
	int_enable = nvdisp_cmd_int_status_frame_end_f(1) |
			nvdisp_cmd_int_status_v_blank_f(1) |
			nvdisp_cmd_int_status_uf_f(1);
	/* for panels with one-shot mode enable tearing effect interrupt */
	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		int_enable |= MSF_INT;
	/* Todo: also need to enable interrupts for SD3, DSC etc */

	tegra_dc_writel(dc, int_enable, nvdisp_cmd_int_enable_r());

	int_mask = nvdisp_cmd_int_status_uf_f(1);
	tegra_dc_writel(dc, int_mask, nvdisp_cmd_int_mask_r());

	tegra_dc_writel(dc, nvdisp_state_access_write_mux_assembly_f() |
		nvdisp_state_access_read_mux_active_f(),
		nvdisp_state_access_r());

	tegra_dc_writel(dc, 0x00000000, nvdisp_background_color_r());

	for_each_set_bit(i, &dc->valid_windows, DC_N_WINDOWS) {
		struct tegra_dc_win *win = tegra_dc_get_window(dc, i);

		BUG_ON(!win);

		/* refuse to operate on invalid syncpts */
		if (WARN_ON(win->syncpt.id == NVSYNCPT_INVALID))
			continue;

		if (!nvhost_syncpt_read_ext_check(dc->ndev,
						win->syncpt.id, &val))
			win->syncpt.min = win->syncpt.max = val;
	}

	dc->crc_pending = false;

	/* set mode */
	tegra_nvdisp_program_mode(dc, &dc->mode);

	/*set display control */
	tegra_nvdisp_set_control(dc);

	return 0;
}

static int tegra_nvdisp_postcomp_init(struct tegra_dc *dc)
{
	return 0;
}

static int tegra_nvdisp_rg_init(struct tegra_dc *dc)
{
	return 0;
}

static int tegra_nvdisp_cursor_init(struct tegra_dc *dc)
{
	return 0;
}

int tegra_nvdisp_head_disable(struct tegra_dc *dc)
{
	int idx;

	/* Detach windows from the head */
	for_each_set_bit(idx, &dc->pdata->win_mask, DC_N_WINDOWS) {
		if (tegra_nvdisp_detach_win(dc, idx))
			dev_err(&dc->ndev->dev,
				"failed to detach window %d\n", idx);
		else
			dev_dbg(&dc->ndev->dev,
				"Window %d detached from head %d\n", idx,
				dc->ctrl_num);
	}

	return 0;
}

int tegra_nvdisp_head_enable(struct tegra_dc *dc)
{
	int i;
	int res;
	int idx;

	if (WARN_ON(!dc || !dc->out || !dc->out_ops))
		return false;

	/* TODO: confirm power domains for parker */
	/* tegra_dc_unpowergate_locked(dc); */

	tegra_dc_get(dc);

	/* Enable OR -- need to enable the connection first */
	if (dc->out->enable)
		dc->out->enable(&dc->ndev->dev);

	/* TODO: clock setup */
	/*tegra_dc_power_on(dc);*/

	/* Mask interrupts duirng init */
	tegra_dc_writel(dc, 0, DC_CMD_INT_MASK);

	enable_irq(dc->irq);

	res = tegra_nvdisp_head_init(dc);
	res |= tegra_nvdisp_postcomp_init(dc);
	res |= tegra_nvdisp_rg_init(dc);
	res |= tegra_nvdisp_cursor_init(dc);

	if (res) {
		dev_err(&dc->ndev->dev, "%s, failed head enable\n", __func__);
		goto failed_enable;
	}

	if (dc->out_ops && dc->out_ops->enable)
		dc->out_ops->enable(dc);

	/* force a full blending update */
	for (i = 0; i < DC_N_WINDOWS; i++)
		dc->blend.z[i] = -1;

	tegra_dc_ext_enable(dc->ext);
	trace_display_enable(dc);

	if (dc->out->postpoweron)
		dc->out->postpoweron(&dc->ndev->dev);

	if (dc->out_ops && dc->out_ops->postpoweron)
		dc->out_ops->postpoweron(dc);

	tegra_log_resume_time();
	/*
	 * We will need to reinitialize the display the next time panel
	 * is enabled.
	 */
	dc->out->flags &= ~TEGRA_DC_OUT_INITIALIZED_MODE;

	/* Assign windows to this head */
	for_each_set_bit(idx, &dc->pdata->win_mask, DC_N_WINDOWS) {
		if (tegra_nvdisp_assign_win(dc, idx))
			dev_err(&dc->ndev->dev,
				"failed to assign window %d\n", idx);
		else
			dev_dbg(&dc->ndev->dev,
				"Window %d assigned to head %d\n", idx,
				dc->ctrl_num);
	}

	/* Enable RG underflow logging */
	tegra_dc_writel(dc, nvdisp_rg_underflow_enable_enable_f() |
		nvdisp_rg_underflow_mode_red_f(),
		nvdisp_rg_underflow_r());

	tegra_dc_put(dc);
	return 0;

failed_enable:
	tegra_dc_writel(dc, 0, DC_CMD_INT_MASK);
	disable_irq_nosync(dc->irq);
	tegra_dc_clear_bandwidth(dc);
	if (dc->out && dc->out->disable)
		dc->out->disable(&dc->ndev->dev);
	tegra_dc_put(dc);

	/* TODO: disable DC clock */
	return -EINVAL;
}

struct tegra_fb_info *tegra_nvdisp_fb_register(struct platform_device *ndev,
	struct tegra_dc *dc, struct tegra_fb_data *fb_data,
	struct resource *fb_mem)
{
	void *virt_addr = NULL;

	/* Assign the given window to current dc */
	if (!tegra_dc_get_window(dc, fb_data->win)) {
		dev_err(&ndev->dev, "%s, failed to get window %d for head %d\n",
			__func__, fb_data->win, dc->ctrl_num);
		return ERR_PTR(-ENOENT);
	}

	/* Allocate FBMem if not already allocated */
	if (!fb_mem->start || !fb_mem->end) {
		int fb_size = fb_data->xres * fb_data->yres *
			fb_data->bits_per_pixel / 8;

		if (!fb_size)
			return ERR_PTR(-ENOENT);

		virt_addr = dma_alloc_writecombine(&ndev->dev, fb_size,
			&fb_mem->start, GFP_KERNEL);
		if (!virt_addr) {
			dev_err(&ndev->dev, "Failed to allocate FBMem\n");
			return ERR_PTR(-ENOENT);
		}
		fb_mem->end = fb_mem->start + fb_size - 1;
		dev_info(&ndev->dev, "Allocated %d as FBmem\n", fb_size);
	}

	return tegra_fb_register(ndev, dc, fb_data, fb_mem, virt_addr);
}

void tegra_nvdisp_enable_crc(struct tegra_dc *dc)
{
	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	tegra_dc_writel(dc, nvdisp_crc_control_enable_enable_f() |
		nvdisp_crc_control_input_data_active_data_f(),
		nvdisp_crc_control_r());

	tegra_dc_enable_general_act(dc);
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	/* Register a client of frame_end interrupt */
	tegra_dc_config_frame_end_intr(dc, true);
}

void tegra_nvdisp_disable_crc(struct tegra_dc *dc)
{
	/* Unregister a client of frame_end interrupt */
	tegra_dc_config_frame_end_intr(dc, false);

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	tegra_dc_writel(dc, 0x0, nvdisp_crc_control_r());
	tegra_dc_enable_general_act(dc);

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
}

u32 tegra_nvdisp_read_rg_crc(struct tegra_dc *dc)
{
	int crc = 0;
	int val = 0;

	if (!dc) {
		pr_err("Failed to get dc: NULL parameter.\n");
		goto crc_error;
	}

	/* If gated quitely return */
	if (!tegra_dc_is_powered(dc))
		return 0;

#ifdef INIT_COMPLETION
	INIT_COMPLETION(dc->crc_complete);
#else
	reinit_completion(&dc->crc_complete);
#endif
	if (dc->crc_pending &&
	    wait_for_completion_interruptible(&dc->crc_complete)) {
		pr_err("CRC read interrupted.\n");
		goto crc_error;
	}

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	val = tegra_dc_readl(dc, nvdisp_rg_crca_r());

	/* tegrasim seems to need more time to set the
	 * CRCA valid bit. So adding an infinite
	 * polling loop for tegrasim
	 */
	if (tegra_platform_is_linsim()) {
		while (val <= 0) {
			val = tegra_dc_readl(dc, nvdisp_rg_crca_r());
			msleep(100);
		}
	}

	if (val & nvdisp_rg_crca_valid_true_f())
		crc = tegra_dc_readl(dc, nvdisp_rg_crcb_r());
	/* clear the error bit if set */
	if (val & nvdisp_rg_crca_error_true_f())
		tegra_dc_writel(dc, nvdisp_rg_crca_error_true_f(),
			nvdisp_rg_crca_r());
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
crc_error:
	return crc;
}


void tegra_nvdisp_underflow_handler(struct tegra_dc *dc)
{
	u32 reg = tegra_dc_readl(dc, nvdisp_rg_underflow_r());
	dc->stats.underflows++;

	if (dc->underflow_mask & NVDISP_UF_INT)
		dc->stats.underflow_frames +=
				nvdisp_rg_underflow_frames_uflowed_v(reg);

	/* Clear the sticky bit and counter */
	tegra_dc_writel(dc,
		nvdisp_rg_underflow_frames_uflowed_rst_trigger_f() |
		nvdisp_rg_underflow_uflowed_clr_f(),
		nvdisp_rg_underflow_r());

	/* Do we need to see whether the reset is done */
}

static int tegra_nvdisp_set_color_control(struct tegra_dc *dc)
{
	u32 color_control;

	switch (dc->out->depth) {

	case 30:
		color_control = nvdisp_color_ctl_base_color_size_30bits_f();
		break;
	case 24:
		color_control = nvdisp_color_ctl_base_color_size_24bits_f();
		break;
	case 18:
		color_control = nvdisp_color_ctl_base_color_size_18bits_f();
		break;
	default:
		color_control = nvdisp_color_ctl_base_color_size_6bits_f();
		break;
	}

	switch (dc->out->dither) {
	case TEGRA_DC_UNDEFINED_DITHER:
	case TEGRA_DC_DISABLE_DITHER:
		color_control |= nvdisp_color_ctl_dither_ctl_disable_f();
		break;
	case TEGRA_DC_ORDERED_DITHER:
		color_control |= nvdisp_color_ctl_dither_ctl_ordered_f();
		break;
	case TEGRA_DC_TEMPORAL_DITHER:
		color_control |= nvdisp_color_ctl_dither_ctl_temporal_f();
		break;
	case TEGRA_DC_ERRACC_DITHER:
		color_control |= nvdisp_color_ctl_dither_ctl_err_acc_f();
		break;
	default:
		dev_err(&dc->ndev->dev, "Error: Unsupported dithering mode\n");
	}

#if defined(CONFIG_TEGRA_DC_CMU_V2)
	if (dc->cmu_enabled)
		color_control |= nvdisp_color_ctl_cmu_enable_f();
#endif
	/* TO DO - dither rotation, dither offset, dither phase */

	tegra_dc_writel(dc, color_control,
			nvdisp_color_ctl_r());
	return 0;
}

#if defined(CONFIG_TEGRA_DC_CMU_V2)
void tegra_dc_cache_cmu(struct tegra_dc *dc, struct tegra_dc_cmu *src_cmu)
{
	/* copy the data to DC lut */
	memcpy(dc->cmu.rgb, src_cmu->rgb, sizeof(*src_cmu));
	dc->cmu_dirty = true;
}

static void _tegra_nvdisp_update_cmu(struct tegra_dc *dc,
					struct tegra_dc_lut *cmu)
{
	dc->cmu_enabled = dc->pdata->cmu_enable;
	if (!dc->cmu_enabled)
		return;

	/* Not disabling the cmu here - will
	 * consider it if there is any corruption on
	 * updating cmu while it is running
	 */
	tegra_nvdisp_set_output_lut(dc, cmu);
	dc->cmu_dirty = false;
}

int tegra_nvdisp_update_cmu(struct tegra_dc *dc, struct tegra_dc_lut *cmu)
{
	mutex_lock(&dc->lock);
	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return 0;
	}

	tegra_dc_get(dc);

	_tegra_nvdisp_update_cmu(dc, cmu);
	tegra_nvdisp_set_color_control(dc);
	tegra_dc_writel(dc,
			nvdisp_cmd_state_ctrl_general_act_req_enable_f(),
			nvdisp_cmd_state_ctrl_r());

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	return 0;
}
EXPORT_SYMBOL(tegra_nvdisp_update_cmu);
#endif
