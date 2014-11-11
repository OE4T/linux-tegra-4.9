/*
 * drivers/video/tegra/dc/nvdisplay/nvdis.c
 *
 * Copyright (c) 2014, NVIDIA CORPORATION, All rights reserved.
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
#include "dpaux.h"
#include "hw_nvdisp_nvdisp.h"
#include "hw_win_nvdisp.h"

/* static struct tegra_dc_nvdisp	tegra_nvdisp; */
DEFINE_MUTEX(tegra_nvdisp_lock);


int _tegra_nvdisp_init_once(struct tegra_dc *dc)
{
	int i;
	char syncpt_name[] = "disp_a";

	mutex_lock(&tegra_nvdisp_lock);

	/* Init sycpt ids */
	dc->valid_windows = 0x3f; /* Assign all windows to this head */
	for (i = 0; i < DC_N_WINDOWS; ++i, ++syncpt_name[5]) {
		struct tegra_dc_win *win = tegra_dc_get_window(dc, i);
		win->syncpt.id = nvhost_get_syncpt_client_managed(syncpt_name);
	}
	dc->valid_windows = 0;

	/* Clock init */
	nvdisp_clock_init(dc);

	mutex_unlock(&tegra_nvdisp_lock);
	return 0;
}

static int tegra_nvdisp_program_mode(struct tegra_dc *dc, struct tegra_dc_mode
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

	/* print_mode(dc, mode, __func__); */

	tegra_dc_get(dc);

	/* IMP related updates */
	dc->new_bw_kbps = tegra_dc_calc_min_bandwidth(dc);
	tegra_dc_program_bandwidth(dc, true);

	tegra_dc_writel(dc, mode->h_sync_width | (v_sync_width << 16),
			DC_DISP_SYNC_WIDTH);
	if ((dc->out->type == TEGRA_DC_OUT_DP) ||
		(dc->out->type == TEGRA_DC_OUT_FAKE_DP) ||
		(dc->out->type == TEGRA_DC_OUT_NVSR_DP) ||
		(dc->out->type == TEGRA_DC_OUT_LVDS)) {
		tegra_dc_writel(dc, mode->h_back_porch |
			((v_back_porch - mode->v_ref_to_sync) << 16),
			DC_DISP_BACK_PORCH);
		tegra_dc_writel(dc, mode->h_front_porch |
			((v_front_porch + mode->v_ref_to_sync) << 16),
			DC_DISP_FRONT_PORCH);
	} else {
		tegra_dc_writel(dc, mode->h_back_porch |
			(v_back_porch << 16),
			DC_DISP_BACK_PORCH);
		tegra_dc_writel(dc, mode->h_front_porch |
			(v_front_porch << 16),
			DC_DISP_FRONT_PORCH);
	}
	tegra_dc_writel(dc, mode->h_active | (v_active << 16),
			DC_DISP_DISP_ACTIVE);


#if defined(CONFIG_TEGRA_DC_INTERLACE)
	if (mode->vmode == FB_VMODE_INTERLACED)
		tegra_dc_writel(dc, INTERLACE_MODE_ENABLE |
			INTERLACE_START_FIELD_1
			| INTERLACE_STATUS_FIELD_1,
			DC_DISP_INTERLACE_CONTROL);
	else
		tegra_dc_writel(dc, INTERLACE_MODE_DISABLE,
			DC_DISP_INTERLACE_CONTROL);

	if (mode->vmode == FB_VMODE_INTERLACED) {
		tegra_dc_writel(dc, (mode->h_ref_to_sync |
			((mode->h_sync_width + mode->h_back_porch +
			mode->h_active + mode->h_front_porch) >> 1)
			<< 16), DC_DISP_INTERLACE_FIELD2_REF_TO_SYNC);
		tegra_dc_writel(dc, mode->h_sync_width |
			(v_sync_width << 16),
			DC_DISP_INTERLACE_FIELD2_SYNC_WIDTH);
		tegra_dc_writel(dc, mode->h_back_porch |
			((v_back_porch + 1) << 16),
			DC_DISP_INTERLACE_FIELD2_BACK_PORCH);
		tegra_dc_writel(dc, mode->h_active |
			(v_active << 16),
			DC_DISP_INTERLACE_FIELD2_DISP_ACTIVE);
		tegra_dc_writel(dc, mode->h_front_porch |
			(v_front_porch << 16),
			DC_DISP_INTERLACE_FIELD2_FRONT_PORCH);
	}
#endif

	/* TODO: MIPI/CRT/HDMI clock cals */
	/* TODO: confirm shift clock still exists in Parker */

#ifdef CONFIG_SWITCH
	switch_set_state(&dc->modeset_switch,
			 (mode->h_active << 16) | mode->v_active);
#endif

	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	if (dc->out_ops && dc->out_ops->modeset_notifier)
		dc->out_ops->modeset_notifier(dc);

	tegra_dc_put(dc);

	dc->mode_dirty = false;

	trace_display_mode(dc, &dc->mode);
	return 0;
}


int tegra_nvdisp_init(struct tegra_dc *dc)
{
	/* Only need init once no matter how many dc objects */
	if (dc->ctrl_num)
		return 0;
	return _tegra_nvdisp_init_once(dc);
}

static int tegra_nvdisp_head_init(struct tegra_dc *dc)
{
	u32 int_enable;
	u32 int_mask;

	/* Init syncpt */
	tegra_dc_writel(dc, nvdisp_incr_syncpt_cntrl_no_stall_f(1),
		nvdisp_incr_syncpt_cntrl_r());
	tegra_dc_writel(dc, nvdisp_cont_syncpt_vsync_en_enable_f() |
		(NVSYNCPT_VBLANK0 + dc->ctrl_num),
		nvdisp_cont_syncpt_vsync_r());

	/* Init interrupts */
	/* All interrupts are edge trigger, and polarity is high */
	tegra_dc_writel(dc, 0xffffffff, nvdisp_int_type_r());

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
		nvdisp_state_access_read_mux_assembly_f(),
		nvdisp_state_access_r());

	tegra_dc_writel(dc, 0x00000000, nvdisp_background_color_r());

	dc->crc_pending = false;
	/* set mode */
	tegra_nvdisp_program_mode(dc, &dc->mode);

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

int tegra_nvdisp_head_enable(struct tegra_dc *dc)
{
	int i;
	int res;
	struct device_node *np_dpaux;
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

	np_dpaux = of_find_node_by_path(DPAUX_NODE);
	if (np_dpaux || !dc->ndev->dev.of_node)
		tegra_dpaux_pad_power(dc, TEGRA_DPAUX_INSTANCE_0, false);

	if (dc->out_ops && dc->out_ops->enable)
		dc->out_ops->enable(dc);

	/* force a full blending update */
	for (i = 0; i < DC_N_WINDOWS; i++)
		dc->blend.z[i] = -1;

	tegra_dc_ext_enable(dc->ext);
	trace_display_enable(dc);

	/* tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL); */

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

	/* Allocate FBMem if not already allcoated */
	if (!fb_mem->start || !fb_mem->end) {
		int fb_size = ((fb_data->xres + 63)/64) * fb_data->yres *
			fb_data->bits_per_pixel;

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

	INIT_COMPLETION(dc->crc_complete);
	if (dc->crc_pending &&
	    wait_for_completion_interruptible(&dc->crc_complete)) {
		pr_err("CRC read interrupted.\n");
		goto crc_error;
	}

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	val = tegra_dc_readl(dc, nvdisp_rg_crca_r());
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
