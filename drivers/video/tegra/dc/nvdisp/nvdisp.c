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
#include "dc_reg.h"
#include "dc_config.h"
#include "dc_priv.h"
#include "dp.h"
#include "dpaux.h"

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

	mutex_unlock(&tegra_nvdisp_lock);
	return 0;
}

static int tegra_nvdisp_program_mode(struct tegra_dc *dc, struct tegra_dc_mode
				     *mode)
{
	unsigned long val;
	unsigned long v_back_porch;
	unsigned long v_front_porch;
	unsigned long v_sync_width;
	unsigned long v_active;

	if (!dc->mode.pclk)
		return 0;

	/* Temporarily commenting . This variable is currently being
	 * initialized  on dc_mode_override function which is not
	 * set for fake panel. Recheck whether we need this checking
	 */
	/*if (!dc->initialized) {
		dev_info(&dc->ndev->dev, "Skipping %s.\n", __func__);
		return 0;
	}*/

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

	tegra_dc_writel(dc, 0x0, DC_DISP_DISP_TIMING_OPTIONS);
	/* tegra_dc_writel(dc, mode->h_ref_to_sync | (mode->v_ref_to_sync << 16), */
	/* 		DC_DISP_REF_TO_SYNC); */
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

	tegra_dc_writel(dc, DE_SELECT_ACTIVE | DE_CONTROL_NORMAL,
			DC_DISP_DATA_ENABLE_OPTIONS);

	/* TODO: MIPI/CRT/HDMI clock cals */
	val = 0;
	if (!(dc->out->type == TEGRA_DC_OUT_DSI ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSIA ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSIB ||
		dc->out->type == TEGRA_DC_OUT_FAKE_DSI_GANGED ||
		dc->out->type == TEGRA_DC_OUT_HDMI)) {
		val = DISP_DATA_FORMAT_DF1P1C;

		if (dc->out->align == TEGRA_DC_ALIGN_MSB)
			val |= DISP_DATA_ALIGNMENT_MSB;
		else
			val |= DISP_DATA_ALIGNMENT_LSB;

		if (dc->out->order == TEGRA_DC_ORDER_RED_BLUE)
			val |= DISP_DATA_ORDER_RED_BLUE;
		else
			val |= DISP_DATA_ORDER_BLUE_RED;
	}
	tegra_dc_writel(dc, val, DC_DISP_DISP_INTERFACE_CONTROL);

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
	dc->vblank_syncpt = NVSYNCPT_VBLANK0;

	/* Only need init once no matter how many dc objects */
	if (dc->ctrl_num)
		return 0;
	return _tegra_nvdisp_init_once(dc);
}

static int tegra_nvdisp_head_init(struct tegra_dc *dc)
{
	u32 int_enable;

	/* Init syncpt */
	tegra_dc_writel(dc, 0x00000100, DC_CMD_GENERAL_INCR_SYNCPT_CNTRL);
	/* TODO: confirm this is still required */
	/* Init MC controls */
	tegra_dc_writel(dc, 0x00000100 | dc->vblank_syncpt,
			DC_CMD_CONT_SYNCPT_VSYNC);

	/* Init interrupts */
	tegra_dc_writel(dc, 0x00004700, DC_CMD_INT_TYPE);
	tegra_dc_writel(dc, WIN_A_OF_INT | WIN_B_OF_INT | WIN_C_OF_INT |
		WIN_T_UF_INT | WIN_D_UF_INT | HC_UF_INT |
		WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT,
		DC_CMD_INT_POLARITY);
	tegra_dc_writel(dc, 0x00202020, DC_DISP_MEM_HIGH_PRIORITY);
	tegra_dc_writel(dc, 0x00010101, DC_DISP_MEM_HIGH_PRIORITY_TIMER);
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	tegra_dc_writel(dc, 0x00000000, DC_DISP_DISP_MISC_CONTROL);
#endif
	/* enable interrupts for vblank, frame_end and underflows */
	int_enable = (FRAME_END_INT | V_BLANK_INT | ALL_UF_INT());
	/* for panels with one-shot mode enable tearing effect interrupt */
	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		int_enable |= MSF_INT;

	tegra_dc_writel(dc, int_enable, DC_CMD_INT_ENABLE);
	tegra_dc_writel(dc, ALL_UF_INT(), DC_CMD_INT_MASK);
	/* tegra_dc_init_vpulse2_int(dc); */

	tegra_dc_writel(dc, WRITE_MUX_ASSEMBLY | READ_MUX_ASSEMBLY,
		DC_CMD_STATE_ACCESS);

#if !defined(CONFIG_TEGRA_DC_BLENDER_GEN2)
	tegra_dc_writel(dc, 0x00000000, DC_DISP_BORDER_COLOR);
#else
	tegra_dc_writel(dc, 0x00000000, DC_DISP_BLEND_BACKGROUND_COLOR);
#endif

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

	if (WARN_ON(!dc || !dc->out || !dc->out_ops))
		return false;

	/* TODO: confirm power domains for parker */
	/* tegra_dc_unpowergate_locked(dc); */

	tegra_dc_get(dc);

	/* Enable OR -- need to enable the connection first */
	if (dc->out->enable)
		dc->out->enable(&dc->ndev->dev);

	/* TODO: clock setup */

	tegra_dc_power_on(dc);

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

	tegra_dc_put(dc);
	return 0;

failed_enable:
	tegra_dc_writel(dc, 0, DC_CMD_INT_MASK);
	disable_irq_nosync(dc->irq);
	tegra_dc_clear_bandwidth(dc);
	if (dc->out && dc->out->disable)
		dc->out->disable();
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
	if (!tegra_dc_get_window(dc, fb_data->win) &&
		tegra_nvdisp_assign_win(dc, fb_data->win)) {
		dev_err(&ndev->dev, "Cannot assign window %d to head %d\n",
			fb_data->win, dc->ctrl_num);
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
