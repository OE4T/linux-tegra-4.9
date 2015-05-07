/*
 * tegra210_adsp_alt.c - Tegra ADSP audio driver
 *
 * Author: Sumit Bhattacharya <sumitb@nvidia.com>
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/of.h>
#include <../arch/arm/mach-tegra/iomap.h>
#include <linux/completion.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/tegra_pm_domains.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <linux/tegra_nvadsp.h>
#include <linux/irqchip/tegra-agic.h>
#include <linux/of_device.h>

#include <sound/pcm.h>
#include <sound/core.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/compress_driver.h>
#include <sound/dmaengine_pcm.h>
#include <sound/tegra_nvfx.h>
#include <sound/tegra_nvfx_apm.h>
#include <sound/tegra_nvfx_plugin.h>

#include "tegra210_adsp_alt.h"

#define DRV_NAME "tegra210-adsp"

/* Flag to enable/disable loading of ADSP firmware */
#define ENABLE_ADSP 1

static struct tegra210_adsp_app_desc {
	const char *name;
	const char *fw_name;
	const char *wt_name;
	uint32_t param_type;
	uint32_t reg_start;
	uint32_t reg_end;
	nvadsp_app_handle_t handle;
} adsp_app_minimal[] = {
	{"apm", "nvapm.elf", NULL, SNDRV_CTL_ELEM_TYPE_NONE,
		TEGRA210_ADSP_APM_IN1, TEGRA210_ADSP_APM_IN8},
	{"adma", "nvadma.elf", NULL, SNDRV_CTL_ELEM_TYPE_NONE,
		TEGRA210_ADSP_PLUGIN_ADMA1, TEGRA210_ADSP_PLUGIN_ADMA4},
};

static struct tegra210_adsp_app_desc *adsp_app_desc;
static unsigned int adsp_app_count; /* total number of apps initialized */

/* ADSP APP specific structure */
struct tegra210_adsp_app {
	const struct tegra210_adsp_app_desc *desc;
	nvadsp_app_info_t *info;
	plugin_shared_mem_t *plugin;
	apm_shared_state_t *apm; /* For a plugin it stores parent apm data */
	struct nvadsp_mbox rx_mbox;
	uint32_t reg;
	uint32_t adma_chan; /* Valid for only ADMA app */
	uint32_t fe:1; /* Whether the app is used as a FE APM */
	uint32_t connect:1; /* if app is connected to a source */
	void *private_data;
	int (*msg_handler)(struct tegra210_adsp_app *, apm_msg_t *);
};

struct tegra210_adsp_pcm_rtd {
	struct device *dev;
	struct snd_pcm_substream *substream;
	struct tegra210_adsp_app *fe_apm;
};

struct tegra210_adsp_compr_rtd {
	struct device *dev;
	struct snd_dma_buffer buf;
	struct snd_compr_stream *cstream;
	struct snd_codec codec;
	struct tegra210_adsp_app *fe_apm;
	int is_draining;
};

struct tegra210_adsp {
	struct device *dev;
	struct tegra210_adsp_app apps[TEGRA210_ADSP_VIRT_REG_MAX];
	atomic_t reg_val[TEGRA210_ADSP_VIRT_REG_MAX];
	DECLARE_BITMAP(adma_usage, TEGRA210_ADSP_ADMA_CHANNEL_COUNT);
	uint32_t i2s_rate;
	struct mutex mutex;
	int init_done;
	struct tegra210_adsp_path {
		uint32_t fe_reg;
		uint32_t be_reg;
	} pcm_path[ADSP_FE_END+1][2];
};

static const struct snd_pcm_hardware adsp_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.channels_min		= 1,
	.channels_max		= 2,
	.period_bytes_min	= 128,
	.period_bytes_max	= PAGE_SIZE * 2,
	.periods_min		= 1,
	.periods_max		= 8,
	.buffer_bytes_max	= PAGE_SIZE * 8,
	.fifo_size		= 4,
};

/* Following structure is ALSA-Compress specific */
static struct snd_compr_caps
	tegra210_adsp_compr_caps[SND_COMPRESS_CAPTURE + 1] = {
	[SND_COMPRESS_PLAYBACK] = {
		.num_codecs = 2,
		.direction = SND_COMPRESS_PLAYBACK,
		.min_fragment_size = 1024,
		.max_fragment_size = 1024 * 1024,        /* 1 MB */
		.min_fragments = 2,
		.max_fragments = 1024,
		.codecs = {
			[0] = SND_AUDIOCODEC_MP3,
			[1] = SND_AUDIOCODEC_AAC,
		},
	},
	[SND_COMPRESS_CAPTURE] = {
		.num_codecs = 0,
		.direction = SND_COMPRESS_CAPTURE,
	},
};

/* Following structure is ALSA-Compress specific */
static struct snd_compr_codec_caps adsp_compr_codec_caps[] = {
	[SND_AUDIOCODEC_MP3] = {
		.codec = SND_AUDIOCODEC_MP3,
		.num_descriptors = 1,
		.descriptor = {
			[0] = {
				.max_ch = 2,
				.sample_rates = {
					[0] = SNDRV_PCM_RATE_8000_48000,
				},
				.bit_rate = {
					[0] = 32000,
					[1] = 64000,
					[2] = 128000,
					[3] = 256000,
					[4] = 320000,
				},
				.num_bitrates = 5,
				.rate_control =
					SND_RATECONTROLMODE_CONSTANTBITRATE |
					SND_RATECONTROLMODE_VARIABLEBITRATE,
				.profiles = 0,
				.modes = SND_AUDIOCHANMODE_MP3_STEREO,
				.formats = SND_AUDIOSTREAMFORMAT_UNDEFINED,
				.min_buffer = 1024,
			},
		},
	},
	[SND_AUDIOCODEC_AAC] = {
		.codec = SND_AUDIOCODEC_AAC,
		.num_descriptors = 1,
		.descriptor = {
			[0] = {
				.max_ch = 2,
				.sample_rates = {
					[0] = SNDRV_PCM_RATE_8000_48000,
				},
				.bit_rate = {
					[0] = 32000,
					[1] = 64000,
					[2] = 128000,
					[3] = 256000,
					[4] = 320000,
				},
				.num_bitrates = 5,
				.rate_control =
					SND_RATECONTROLMODE_CONSTANTBITRATE |
					SND_RATECONTROLMODE_VARIABLEBITRATE,
				.profiles = SND_AUDIOPROFILE_AAC,
				.modes = SND_AUDIOMODE_AAC_LC,
				.formats = SND_AUDIOSTREAMFORMAT_MP4ADTS,
				.min_buffer = 1024,
			},
		},
	},
};

static status_t tegra210_adsp_msg_handler(uint32_t msg, void *data);

/*
 * Utility functions
 */
/* ADSP virtual register read/write functions */
static uint32_t tegra210_adsp_reg_read(struct tegra210_adsp *adsp, uint32_t reg)
{
	return atomic_read(&adsp->reg_val[reg]);
}

static void tegra210_adsp_reg_write(struct tegra210_adsp *adsp,
				uint32_t reg, uint32_t val)
{
	atomic_set(&adsp->reg_val[reg], val);
	dev_vdbg(adsp->dev, "%s : 0x%x -> 0x%x\n", __func__, reg, val);
}

static void tegra210_adsp_reg_update_bits(struct tegra210_adsp *adsp,
					uint32_t reg, uint32_t mask,
					uint32_t val)
{
	uint32_t temp;

	temp = tegra210_adsp_reg_read(adsp, reg);
	val = (val & mask) | (temp & ~mask);
	tegra210_adsp_reg_write(adsp, reg, val);

	dev_vdbg(adsp->dev, "%s : 0x%x -> 0x%x\n", __func__, reg, val);
}

/* API to get source widget id connected to a widget */
static uint32_t tegra210_adsp_get_source(struct tegra210_adsp *adsp,
					 uint32_t reg)
{
	uint32_t source;

	source = tegra210_adsp_reg_read(adsp, reg);
	source &= TEGRA210_ADSP_WIDGET_SOURCE_MASK;
	source >>= TEGRA210_ADSP_WIDGET_SOURCE_SHIFT;

	return source;
}
/* ADSP shared memory allocate/free functions */
static int tegra210_adsp_preallocate_dma_buffer(struct device *dev, size_t size,
				struct snd_dma_buffer *buf)
{
	dev_vdbg(dev, "%s : size %d.", __func__, (uint32_t)size);

	buf->area = nvadsp_alloc_coherent(size, &buf->addr, GFP_KERNEL);
	if (!buf->area) {
		dev_err(dev, "Failed to pre-allocated DMA buffer.");
		return -ENOMEM;
	}

	buf->bytes = size;
	buf->private_data = NULL;
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = dev;

	return 0;
}

static void tegra210_adsp_deallocate_dma_buffer(struct snd_dma_buffer *buf)
{
	dev_vdbg(buf->dev.dev, "%s : size %d.", __func__, (uint32_t)buf->bytes);

	if (!buf->area)
		return;

	nvadsp_free_coherent(buf->bytes, buf->area, buf->addr);
	buf->area = NULL;
	buf->addr = 0;
}

/* ADSP OS boot and init API */
static int tegra210_adsp_init(struct tegra210_adsp *adsp)
{
	int i, ret = 0;

	mutex_lock(&adsp->mutex);
	ret = nvadsp_os_load();
	if (ret < 0) {
		dev_err(adsp->dev, "Failed to load OS.");
		goto exit;
	}

	if (nvadsp_os_start()) {
		dev_err(adsp->dev, "Failed to start OS");
		goto exit;
	}

	/* Load ADSP audio apps */
	for (i = 0; i < adsp_app_count; i++) {
		adsp_app_desc[i].handle = nvadsp_app_load(
				adsp_app_desc[i].name,
				adsp_app_desc[i].fw_name);
		if (!adsp_app_desc[i].handle) {
			dev_err(adsp->dev, "Failed to load app %s",
						adsp_app_desc[i].name);
			goto exit;
		}
	}
	adsp->init_done = 1;

exit:
	mutex_unlock(&adsp->mutex);
	return ret;
}

static void tegra210_adsp_deinit(struct tegra210_adsp *adsp)
{
	mutex_lock(&adsp->mutex);
	if (adsp->init_done) {
		/* TODO : Stop ADSP OS if possible */
		adsp->init_done = 0;
	}
	mutex_unlock(&adsp->mutex);
}

/* ADSP-CPU message send-receive utility functions */
static int tegra210_adsp_get_msg(apm_shared_state_t *apm, apm_msg_t *apm_msg)
{
	apm_msg->msgq_msg.size = MSGQ_MSG_WSIZE(apm_msg_t) -
		MSGQ_MESSAGE_HEADER_WSIZE;
	return msgq_dequeue_message(&apm->msgq_send.msgq,
		&apm_msg->msgq_msg);
}

static int tegra210_adsp_send_msg(apm_shared_state_t *apm,
				  apm_msg_t *apm_msg, uint32_t flags)
{
	int ret = 0;

	ret = msgq_queue_message(&apm->msgq_recv.msgq, &apm_msg->msgq_msg);
	if (ret < 0)
		return ret;

	if (flags & TEGRA210_ADSP_MSG_FLAG_HOLD)
		return 0;

	return nvadsp_hwmbox_send_data(apm->mbox_id,
		apm_cmd_msg_ready,
		NVADSP_MBOX_SMSG);
}

static int tegra210_adsp_send_remove_msg(struct tegra210_adsp_app *app,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_fx_remove_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_fx_remove_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_fx_remove_all;

	return tegra210_adsp_send_msg(app->apm, &apm_msg, flags);
}

static int tegra210_adsp_send_connect_msg(struct tegra210_adsp_app *src,
					struct tegra210_adsp_app *dst,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_fx_connect_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_fx_connect_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_fx_connect;
	apm_msg.msg.fx_connect_params.plugin_src.pvoid = IS_APM_IN(src->reg) ?
		NULL : src->plugin->plugin.pvoid;
	apm_msg.msg.fx_connect_params.pin_src = 0;
	apm_msg.msg.fx_connect_params.plugin_dst.pvoid = IS_APM_OUT(dst->reg) ?
		NULL : dst->plugin->plugin.pvoid;
	apm_msg.msg.fx_connect_params.pin_dst = 0;

	return tegra210_adsp_send_msg(src->apm, &apm_msg, flags);
}

static int tegra210_adsp_send_io_buffer_msg(struct tegra210_adsp_app *app,
					dma_addr_t addr, size_t size,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_io_buffer_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_io_buffer_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_set_io_buffer;
	apm_msg.msg.io_buffer_params.pin_type = IS_APM_IN(app->reg) ?
		NVFX_PIN_TYPE_INPUT : NVFX_PIN_TYPE_OUTPUT;
	apm_msg.msg.io_buffer_params.pin_id = 0;
	apm_msg.msg.io_buffer_params.addr.ptr = (uint64_t)addr;
	apm_msg.msg.io_buffer_params.size = size;

	return tegra210_adsp_send_msg(app->apm, &apm_msg, flags);
}

static int tegra210_adsp_send_period_size_msg(struct tegra210_adsp_app *app,
					size_t size, uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size =
		MSGQ_MSG_WSIZE(apm_notification_params_t);
	apm_msg.msg.call_params.size =
		sizeof(apm_notification_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_set_notification_size;
	apm_msg.msg.notification_params.pin_type = IS_APM_IN(app->reg) ?
		NVFX_PIN_TYPE_INPUT : NVFX_PIN_TYPE_OUTPUT;
	apm_msg.msg.notification_params.pin_id = 0;
	apm_msg.msg.notification_params.size = size;

	return tegra210_adsp_send_msg(app->apm, &apm_msg, flags);
}

static int tegra210_adsp_adma_params_msg(struct tegra210_adsp_app *app,
					nvfx_adma_init_params_t *params,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_fx_set_param_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_fx_set_param_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_fx_set_param;
	apm_msg.msg.fx_set_param_params.plugin.pvoid =
		app->plugin->plugin.pvoid;

	params->call_params.size = sizeof(nvfx_adma_init_params_t);
	params->call_params.method = nvfx_adma_method_init;
	memcpy(&apm_msg.msg.fx_set_param_params.params, params,
		sizeof(*params));

	return tegra210_adsp_send_msg(app->apm, &apm_msg, flags);
}

static int tegra210_adsp_send_state_msg(struct tegra210_adsp_app *app,
					int32_t state, uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(nvfx_set_state_params_t);
	apm_msg.msg.call_params.size = sizeof(nvfx_set_state_params_t);
	apm_msg.msg.call_params.method = nvfx_method_set_state;
	apm_msg.msg.state_params.state = state;

	return tegra210_adsp_send_msg(app->apm, &apm_msg, flags);
}

static int tegra210_adsp_send_flush_msg(struct tegra210_adsp_app *app,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(nvfx_flush_params_t);
	apm_msg.msg.call_params.size = sizeof(nvfx_flush_params_t);
	apm_msg.msg.call_params.method = nvfx_method_flush;

	return tegra210_adsp_send_msg(app->apm, &apm_msg, flags);
}

static int tegra210_adsp_send_reset_msg(struct tegra210_adsp_app *app,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(nvfx_reset_params_t);
	apm_msg.msg.call_params.size = sizeof(nvfx_reset_params_t);
	apm_msg.msg.call_params.method = nvfx_method_reset;

	return tegra210_adsp_send_msg(app->apm, &apm_msg, flags);
}

static int tegra210_adsp_send_eos_msg(struct tegra210_adsp_app *app,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_eos_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_eos_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_set_eos;

	return tegra210_adsp_send_msg(app->apm, &apm_msg, flags);
}

static int tegra210_adsp_send_pos_msg(struct tegra210_adsp_app *app,
					uint32_t pos, uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_position_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_position_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_set_position;
	apm_msg.msg.position_params.pin_type = IS_APM_IN(app->reg) ?
		NVFX_PIN_TYPE_INPUT : NVFX_PIN_TYPE_OUTPUT;
	apm_msg.msg.position_params.pin_id = 0;
	apm_msg.msg.position_params.offset = pos;

	return tegra210_adsp_send_msg(app->apm, &apm_msg, flags);
}

/* ADSP app init/de-init APIs */
static int tegra210_adsp_app_init(struct tegra210_adsp *adsp,
				struct tegra210_adsp_app *app)
{
	int ret = 0;

	/* If app is already open or it is APM output pin don't open app */
	if (app->info || IS_APM_OUT(app->reg))
		return 0;

	app->info = nvadsp_app_init(app->desc->handle, NULL);
	if (!app->info) {
		dev_err(adsp->dev, "Failed to init app %s(%s).",
			app->desc->name, app->desc->fw_name);
		return -ENODEV;
	}

	app->plugin = PLUGIN_SHARED_MEM(app->info->mem.shared);
	if (IS_APM_IN(app->reg)) {
		uint32_t apm_out_reg = APM_OUT_START +
					(app->reg - APM_IN_START);
		struct tegra210_adsp_app *apm_out = &adsp->apps[apm_out_reg];

		app->apm = APM_SHARED_STATE(app->info->mem.shared);

		ret = nvadsp_mbox_open(&app->rx_mbox,
			&app->apm->mbox_id,
			app->desc->name, tegra210_adsp_msg_handler, app);
		if (ret < 0) {
			dev_err(adsp->dev, "Failed to init app %s(%s).",
				app->desc->name, app->desc->fw_name);
			goto err_app_exit;
		}

		ret = nvadsp_app_start(app->info);
		if (ret < 0) {
			dev_err(adsp->dev, "Failed to start adsp app");
			goto err_mbox_close;
		}
		/* Copy APM IN app data to APM OUT app */
		apm_out->info = app->info;
		apm_out->plugin = app->plugin;
		apm_out->apm = app->apm;
	} else if (IS_ADMA(app->reg)) {
		app->adma_chan = find_first_zero_bit(adsp->adma_usage,
					TEGRA210_ADSP_ADMA_CHANNEL_COUNT);
		if (app->adma_chan >= TEGRA210_ADSP_ADMA_CHANNEL_COUNT) {
			dev_err(adsp->dev, "All ADMA channels are busy");
			return -EBUSY;
		}
		__set_bit(app->adma_chan, adsp->adma_usage);

		app->adma_chan += TEGRA210_ADSP_ADMA_CHANNEL_START;
	}
	return 0;

err_mbox_close:
	nvadsp_mbox_close(&app->rx_mbox);
err_app_exit:
	app->info = NULL;
	return 0;
}

static void tegra210_adsp_app_deinit(struct tegra210_adsp *adsp,
				struct tegra210_adsp_app *app)
{
	if (!app || !app->desc)
		return;

	if (app->info && !IS_APM_OUT(app->reg)) {
		app->info = NULL;
		app->plugin = NULL;
		app->apm = NULL;

		if (IS_APM_IN(app->reg)) {
			uint32_t apm_out_reg = APM_OUT_START +
					(app->reg - APM_IN_START);
			struct tegra210_adsp_app *apm_out =
					&adsp->apps[apm_out_reg];

			nvadsp_mbox_close(&app->rx_mbox);
			nvadsp_app_stop(app->info);
			apm_out->info = NULL;
			apm_out->plugin = NULL;
			apm_out->apm = NULL;
		} else if (IS_ADMA(app->reg)) {
			__clear_bit(app->adma_chan -
				TEGRA210_ADSP_ADMA_CHANNEL_START,
				adsp->adma_usage);
		}
	}
}

/* API to connect two APMs */
static int tegra210_adsp_connect_apm(struct tegra210_adsp *adsp,
				struct tegra210_adsp_app *app)
{
	uint32_t source = tegra210_adsp_get_source(adsp, app->reg);
	struct tegra210_adsp_app *src = &adsp->apps[source];
	int ret = 0;

	/* If both APMs are in connected state no need to
	   send connect message */
	if (app->connect && src->connect)
		return 0;

	dev_vdbg(adsp->dev, "Connecting APM 0x%x -> 0x%x",
		src->reg, app->reg);

	ret = tegra210_adsp_send_connect_msg(src, app,
		TEGRA210_ADSP_MSG_FLAG_HOLD);
	if (ret < 0) {
		dev_err(adsp->dev, "Connect msg failed. err %d.", ret);
		return ret;
	}
	return 1;
}

/* Recursive function to connect plugins under a APM
   Returns BE/FE on the pcm path */
static int tegra210_adsp_connect_plugin(struct tegra210_adsp *adsp,
					struct tegra210_adsp_app *app,
					uint32_t *apm_in_src)
{
	struct tegra210_adsp_app *src;
	uint32_t source;
	int ret = 0;

	source = tegra210_adsp_get_source(adsp, app->reg);
	if (!IS_ADSP_APP(source))
		return -ENODEV;

	src = &adsp->apps[source];
	if (!IS_APM_IN(src->reg)) {
		ret = tegra210_adsp_connect_plugin(adsp, src, apm_in_src);
		if (ret < 0)
			return ret;
	} else {
		source = tegra210_adsp_get_source(adsp, src->reg);
		if (IS_APM_OUT(source)) {
			/* connect plugins inside next APM */
			ret = tegra210_adsp_connect_plugin(adsp,
				&adsp->apps[source], apm_in_src);
			if (ret < 0)
				return ret;
			/* connect APM_IN to APM_OUT */
			ret = tegra210_adsp_connect_apm(adsp, src);
			if (ret < 0)
				return ret;
		} else {
			/* return if APM_IN is not
			   connected to valid inputs */
			if (!IS_ADSP_FE(source) &&
				!IS_ADSP_ADMAIF(source))
				return -ENODEV;
			if (apm_in_src)
				*apm_in_src = source;
		}
	}
	app->apm = src->apm;

	/* If App is already connected and source connections have not changed
	   no need to again send connect message */
	if (!ret && app->connect)
		return 0;

	dev_vdbg(adsp->dev, "Connecting plugin 0x%x -> 0x%x",
		src->reg, app->reg);

	ret = tegra210_adsp_send_connect_msg(src, app,
		TEGRA210_ADSP_MSG_FLAG_HOLD);
	if (ret < 0) {
		dev_err(adsp->dev, "Connect msg failed. err %d.", ret);
		return ret;
	}
	app->connect = 1;

	/* return 1 if new connection is established */
	return 1;
}

/* Manages FE/BE plugins and deletes if fe_apm is specified */
static void tegra210_adsp_manage_plugin(struct tegra210_adsp *adsp,
		uint32_t end_reg, uint32_t apm_out, struct tegra210_adsp_app *fe_apm)
{
	uint32_t j, fe_reg, be_reg;

	if (IS_ADSP_FE(end_reg)) {
		/* manage playback path */
		fe_reg = end_reg;
		be_reg = 0;
		for (j = ADSP_ADMAIF_START; j <= ADSP_ADMAIF_END; j++) {
			if (tegra210_adsp_get_source(adsp, j) == apm_out) {
				be_reg = j;
				break;
			}
		}
		if (be_reg && fe_reg) {
			if (fe_apm) {
				dev_vdbg(adsp->dev, "Remove playback FE %d -- BE %d pair",
					fe_reg, be_reg);
				tegra210_adsp_send_remove_msg(fe_apm,
						TEGRA210_ADSP_MSG_FLAG_SEND);
				adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_PLAYBACK].fe_reg = 0;
				adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_PLAYBACK].be_reg = 0;
			} else {
				dev_vdbg(adsp->dev, "Found playback FE %d -- BE %d pair",
					fe_reg, be_reg);
				adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_PLAYBACK].fe_reg = fe_reg;
				adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_PLAYBACK].be_reg = be_reg;
			}
		}
	} else if (IS_ADSP_ADMAIF(end_reg)) {
		/* manage record path */
		fe_reg = 0;
		be_reg = end_reg;
		for (j = ADSP_FE_START; j <= ADSP_FE_END; j++) {
			if (tegra210_adsp_get_source(adsp, j) == apm_out) {
				fe_reg = j;
				break;
			}
		}
		if (be_reg && fe_reg) {
			if (fe_apm) {
				dev_vdbg(adsp->dev, "Remove record FE %d -- BE %d pair",
					fe_reg, be_reg);
				tegra210_adsp_send_remove_msg(fe_apm,
						TEGRA210_ADSP_MSG_FLAG_SEND);
				adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_CAPTURE].fe_reg = 0;
				adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_CAPTURE].be_reg = 0;
			} else {
				dev_vdbg(adsp->dev, "Found playback FE %d -- BE %d pair",
					fe_reg, be_reg);
				adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_CAPTURE].fe_reg = fe_reg;
				adsp->pcm_path[fe_reg][SNDRV_PCM_STREAM_CAPTURE].be_reg = be_reg;
			}
		}
	}
}

/* Iterate over all APMs and establish pending connections */
static int tegra210_adsp_update_connection(struct tegra210_adsp *adsp)
{
	int i, ret;
	uint32_t end_reg;

	for (i = APM_OUT_START; i <= APM_OUT_END; i++) {
		ret = tegra210_adsp_connect_plugin(adsp, &adsp->apps[i], &end_reg);
		if (ret >= 0) {
			/* Record FE/BE pair for every successful connection */
			tegra210_adsp_manage_plugin(adsp, end_reg, i, NULL);
		}
	}

	return 0;
}

/* Remove the plugin connections inside associated APM */
static int tegra210_adsp_remove_connection(struct tegra210_adsp *adsp,
		struct tegra210_adsp_app *plugin)
{
	struct tegra210_adsp_app *app;
	uint32_t i, source, apm_out = TEGRA210_ADSP_NONE;

	if (!IS_ADSP_APP(plugin->reg))
		return 0;

	for (i = APM_OUT_START; i <= APM_OUT_END; i++) {
		app =  &adsp->apps[i];
		/* if the path is already broken, do not continue */
		if (!app->connect)
			continue;
		while (IS_ADSP_APP(app->reg)) {
			if (app->reg == plugin->reg) {
				apm_out = i;
				break;
			}
			source = tegra210_adsp_get_source(adsp, app->reg);
			app =  &adsp->apps[source];
		}
		if (apm_out != TEGRA210_ADSP_NONE)
			break;
	}
	/* if plugin is not part of any APM, return here */
	if (apm_out == TEGRA210_ADSP_NONE)
		return 0;

	/* disconnect the plugins inside APM */
	app = &adsp->apps[apm_out];
	while (!IS_APM_IN(app->reg)) {
		source = tegra210_adsp_get_source(adsp, app->reg);
		if (!IS_ADSP_APP(source))
			break;
		app->connect = 0;
		app = &adsp->apps[source];
	}

	/* delete the plugins inside APM */
	if (IS_APM_IN(app->reg)) {
		/* clear the FE/BE list */
		tegra210_adsp_manage_plugin(adsp,
			tegra210_adsp_get_source(adsp, app->reg),
			apm_out, app);
	}
	return 0;
}

/* ADSP mailbox message handler */
static status_t tegra210_adsp_msg_handler(uint32_t msg, void *data)
{
	struct tegra210_adsp_app *app = data;
	apm_msg_t apm_msg;
	int ret = 0;

	switch (msg) {
	case apm_cmd_msg_ready: {
		ret = tegra210_adsp_get_msg(app->apm, &apm_msg);
		if (ret < 0) {
			pr_err("Dequeue failed %d.", ret);
			break;
		}

		if (app->msg_handler)
			return app->msg_handler(app, &apm_msg);
	}
	break;
	default:
		pr_err("Unsupported mailbox msg %d.", msg);
	}
	return 0;
}

static int tegra210_adsp_pcm_msg_handler(struct tegra210_adsp_app *app,
					apm_msg_t *apm_msg)
{
	struct tegra210_adsp_pcm_rtd *prtd = app->private_data;

	switch (apm_msg->msg.call_params.method) {
	case nvfx_apm_method_set_position:
		snd_pcm_period_elapsed(prtd->substream);
		break;
	default:
		dev_err(prtd->dev, "Unsupported cmd %d.",
			apm_msg->msg.call_params.method);
	}
	return 0;
}

static int tegra210_adsp_compr_msg_handler(struct tegra210_adsp_app *app,
					   apm_msg_t *apm_msg)
{
	struct tegra210_adsp_compr_rtd *prtd = app->private_data;

	switch (apm_msg->msg.call_params.method) {
	case nvfx_apm_method_set_position:
		snd_compr_fragment_elapsed(prtd->cstream);
		break;
	case nvfx_apm_method_set_eos: {
		if (!prtd->is_draining) {
			dev_warn(prtd->dev, "EOS reached before drain");
			break;
		}
		snd_compr_drain_notify(prtd->cstream);
		prtd->is_draining = 0;
	}
	break;
	default:
		dev_err(prtd->dev, "Unsupported cmd %d.",
			apm_msg->msg.call_params.method);
	}
	return 0;
}

/* Compress call-back APIs */
static int tegra210_adsp_compr_open(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->device->private_data;
	struct tegra210_adsp *adsp =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct tegra210_adsp_compr_rtd *prtd;
	uint32_t fe_reg = rtd->codec_dai->id;
	int i;

	dev_vdbg(adsp->dev, "%s : DAI ID %d", __func__, rtd->codec_dai->id);

	if (!adsp->init_done)
		return -ENODEV;

	if (!adsp->pcm_path[fe_reg][cstream->direction].fe_reg ||
		!adsp->pcm_path[fe_reg][cstream->direction].be_reg) {
		dev_err(adsp->dev, "Broken Path%d - FE not linked to BE", fe_reg);
		return -EPIPE;
	}

	prtd = devm_kzalloc(adsp->dev, sizeof(struct tegra210_adsp_compr_rtd),
		GFP_KERNEL);
	if (!prtd) {
		dev_err(adsp->dev, "Failed to allocate adsp rtd.");
		return -ENOMEM;
	}

	/* Find out the APM connected with ADSP-FE DAI */
	for (i = APM_IN_START; i <= APM_IN_END; i++) {
		struct tegra210_adsp_app *app = &adsp->apps[i];
		uint32_t source = tegra210_adsp_get_source(adsp, app->reg);

		if (source == fe_reg) {
			app->msg_handler = tegra210_adsp_compr_msg_handler;
			app->private_data = prtd;
			app->fe = 1;
			prtd->fe_apm = app;
			break;
		}
	}

	if (!prtd->fe_apm) {
		dev_err(adsp->dev, "No FE APM found\n");
		devm_kfree(adsp->dev, prtd);
		return -ENODEV;
	}

	prtd->cstream = cstream;
	prtd->dev = adsp->dev;
	cstream->runtime->private_data = prtd;
	return 0;
}

static int tegra210_adsp_compr_free(struct snd_compr_stream *cstream)
{
	struct tegra210_adsp_compr_rtd *prtd = cstream->runtime->private_data;

	if (!prtd)
		return -ENODEV;

	tegra210_adsp_send_reset_msg(prtd->fe_apm,
		TEGRA210_ADSP_MSG_FLAG_SEND);

	prtd->fe_apm->fe = 0;
	devm_kfree(prtd->dev, prtd);

	return 0;
}

static int tegra210_adsp_compr_set_params(struct snd_compr_stream *cstream,
			struct snd_compr_params *params)
{
	struct tegra210_adsp_compr_rtd *prtd = cstream->runtime->private_data;
	int ret = 0;

	if (!prtd)
		return -ENODEV;

	dev_vdbg(prtd->dev, "%s codec %d rate %d chan %d frag size %d frag %d",
		 __func__, params->codec.id,
		 snd_pcm_rate_bit_to_rate(params->codec.sample_rate),
		 params->codec.ch_in, params->buffer.fragment_size,
		 params->buffer.fragments);

	ret = tegra210_adsp_preallocate_dma_buffer(prtd->dev,
		params->buffer.fragment_size * params->buffer.fragments,
		&prtd->buf);
	if (ret < 0)
		return ret;

	ret = tegra210_adsp_send_io_buffer_msg(prtd->fe_apm, prtd->buf.addr,
					prtd->buf.bytes,
					TEGRA210_ADSP_MSG_FLAG_HOLD);
	if (ret < 0) {
		dev_err(prtd->dev, "IO buffer send msg failed. err %d.", ret);
		return ret;
	}

	ret = tegra210_adsp_send_period_size_msg(prtd->fe_apm,
					params->buffer.fragment_size,
					TEGRA210_ADSP_MSG_FLAG_HOLD);
	if (ret < 0) {
		dev_err(prtd->dev, "Period size send msg failed. err %d.", ret);
		return ret;
	}

	memcpy(&prtd->codec, &params->codec, sizeof(struct snd_codec));
	return 0;
}

static int tegra210_adsp_compr_get_params(struct snd_compr_stream *cstream,
			struct snd_codec *codec)
{
	struct tegra210_adsp_compr_rtd *prtd = cstream->runtime->private_data;

	memcpy(codec, &prtd->codec, sizeof(struct snd_codec));
	return 0;
}

static int tegra210_adsp_compr_trigger(struct snd_compr_stream *cstream,
					int cmd)
{
	struct tegra210_adsp_compr_rtd *prtd = cstream->runtime->private_data;
	int ret = 0;

	dev_vdbg(prtd->dev, "%s : cmd %d", __func__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
			nvfx_state_active,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state start.");
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
				nvfx_state_active,
				TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state resume");
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
			nvfx_state_inactive,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state stop");
			return ret;
		}

		ret = tegra210_adsp_send_flush_msg(prtd->fe_apm,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to reset");
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
			nvfx_state_inactive,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state pause");
			return ret;
		}
		break;
	case SND_COMPR_TRIGGER_DRAIN:
		prtd->is_draining = 1;
		ret = tegra210_adsp_send_eos_msg(prtd->fe_apm,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state drain");
			return ret;
		}
		break;
	default:
		dev_err(prtd->dev, "Unsupported state.");
		return -EINVAL;
	}
	return 0;
}

static int tegra210_adsp_compr_copy(struct snd_compr_stream *cstream,
			char __user *buf, size_t count)
{
	struct tegra210_adsp_compr_rtd *prtd = cstream->runtime->private_data;
	struct snd_compr_runtime *runtime = cstream->runtime;
	void *dstn;
	size_t copy;
	u64 app_pointer;

	dev_vdbg(prtd->dev, "%s : size %d", __func__, (uint32_t)count);

	if (!count)
		return 0;

	app_pointer = div64_u64(runtime->total_bytes_available,
				runtime->buffer_size);
	app_pointer = runtime->total_bytes_available -
			(app_pointer * runtime->buffer_size);
	dstn = prtd->buf.area + app_pointer;

	if (count < runtime->buffer_size - app_pointer) {
		if (copy_from_user(dstn, buf, count))
			return -EFAULT;
	} else {
		copy = runtime->buffer_size - app_pointer;
		if (copy_from_user(dstn, buf, copy))
			return -EFAULT;
		if (copy_from_user(prtd->buf.area, buf + copy, count - copy))
			return -EFAULT;
	}
	tegra210_adsp_send_pos_msg(prtd->fe_apm,
	    (runtime->total_bytes_available + count) % runtime->buffer_size,
	    TEGRA210_ADSP_MSG_FLAG_SEND);

	return count;
}

static int tegra210_adsp_compr_pointer(struct snd_compr_stream *cstream,
			struct snd_compr_tstamp *tstamp)
{
	struct tegra210_adsp_compr_rtd *prtd = cstream->runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = cstream->device->private_data;
	struct tegra210_adsp *adsp =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct tegra210_adsp_app *app = prtd->fe_apm;
	nvfx_shared_state_t *shared = &app->apm->nvfx_shared_state;
	uint32_t frames_played = ((shared->output[0].bytes >> 2) *
		snd_pcm_rate_bit_to_rate(prtd->codec.sample_rate)) /
		adsp->i2s_rate;

	tstamp->byte_offset = shared->input[0].bytes %
		cstream->runtime->buffer_size;
	tstamp->copied_total = shared->input[0].bytes;
	tstamp->pcm_frames = frames_played;
	/* TODO : calculate IO frames correctly */
	tstamp->pcm_io_frames = frames_played;
	tstamp->sampling_rate =
		snd_pcm_rate_bit_to_rate(prtd->codec.sample_rate);

	dev_vdbg(prtd->dev, "%s off %d copied %d pcm %d pcm io %d",
		 __func__, (int)tstamp->byte_offset, (int)tstamp->copied_total,
		 (int)tstamp->pcm_frames, (int)tstamp->pcm_io_frames);
	return 0;
}

static int tegra210_adsp_compr_get_caps(struct snd_compr_stream *cstream,
			struct snd_compr_caps *caps)
{
	if (cstream->direction == SND_COMPRESS_PLAYBACK)
		memcpy(caps, &tegra210_adsp_compr_caps[SND_COMPRESS_PLAYBACK],
			sizeof(struct snd_compr_caps));
	else
		memcpy(caps, &tegra210_adsp_compr_caps[SND_COMPRESS_CAPTURE],
			sizeof(struct snd_compr_caps));

	return 0;
}

static int tegra210_adsp_compr_codec_caps(struct snd_compr_stream *cstream,
			struct snd_compr_codec_caps *codec_caps)
{
	struct tegra210_adsp_compr_rtd *prtd = cstream->runtime->private_data;

	dev_vdbg(prtd->dev, "%s : codec %d", __func__, codec_caps->codec);

	if (!codec_caps->codec)
		codec_caps->codec = prtd->codec.id;

	switch (codec_caps->codec) {
	case SND_AUDIOCODEC_MP3:
		memcpy(codec_caps, &adsp_compr_codec_caps[SND_AUDIOCODEC_MP3],
			sizeof(struct snd_compr_codec_caps));
		return 0;
	case SND_AUDIOCODEC_AAC:
		memcpy(codec_caps, &adsp_compr_codec_caps[SND_AUDIOCODEC_AAC],
			sizeof(struct snd_compr_codec_caps));
		return 0;
	default:
		dev_err(prtd->dev, "Unsupported codec %d", codec_caps->codec);
		return -EINVAL;
	}
	return 0;
}

static struct snd_compr_ops tegra210_adsp_compr_ops = {

	.open = tegra210_adsp_compr_open,
	.free = tegra210_adsp_compr_free,
	.set_params = tegra210_adsp_compr_set_params,
	.get_params = tegra210_adsp_compr_get_params,
	.trigger = tegra210_adsp_compr_trigger,
	.pointer = tegra210_adsp_compr_pointer,
	.copy = tegra210_adsp_compr_copy,
	.get_caps = tegra210_adsp_compr_get_caps,
	.get_codec_caps = tegra210_adsp_compr_codec_caps,
};

/* PCM APIs */
static int tegra210_adsp_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra210_adsp *adsp =
		snd_soc_platform_get_drvdata(rtd->platform);
	struct tegra210_adsp_pcm_rtd *prtd;
	uint32_t fe_reg = rtd->codec_dai->id;
	uint32_t source;
	int i, ret = 0;

	dev_vdbg(adsp->dev, "%s", __func__);

	if (!adsp->pcm_path[fe_reg][substream->stream].fe_reg ||
		!adsp->pcm_path[fe_reg][substream->stream].be_reg) {
		dev_err(adsp->dev, "Broken Path%d - FE not linked to BE", fe_reg);
		return -EPIPE;
	}

	prtd = devm_kzalloc(adsp->dev, sizeof(struct tegra210_adsp_pcm_rtd),
		GFP_KERNEL);
	if (!prtd) {
		dev_err(adsp->dev, "Failed to allocate adsp rtd.");
		return -ENOMEM;
	}

	/* Find out the APM connected with ADSP-FE DAI */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = APM_IN_START; i < APM_IN_END; i++) {
			struct tegra210_adsp_app *app = &adsp->apps[i];

			source = tegra210_adsp_get_source(adsp, app->reg);
			if (source == fe_reg) {
				prtd->fe_apm = app;
				break;
			}
		}
	} else {
		source = tegra210_adsp_get_source(adsp, fe_reg);
		if (IS_APM_OUT(source)) {
			uint32_t apm_in_reg =
				APM_IN_START + (source - APM_OUT_START);
			adsp->apps[apm_in_reg].msg_handler =
				tegra210_adsp_pcm_msg_handler;
			adsp->apps[apm_in_reg].private_data = prtd;
			prtd->fe_apm = &adsp->apps[source];
		}
	}

	if (!prtd->fe_apm) {
		dev_err(adsp->dev, "No FE APM found\n");
		devm_kfree(adsp->dev, prtd);
		return -ENODEV;
	}
	prtd->fe_apm->msg_handler = tegra210_adsp_pcm_msg_handler;
	prtd->fe_apm->private_data = prtd;
	prtd->fe_apm->fe = 1;

	/* Set HW params now that initialization is complete */
	snd_soc_set_runtime_hwparams(substream, &adsp_pcm_hardware);

	/* Ensure period size is multiple of 4 */
	ret = snd_pcm_hw_constraint_step(substream->runtime, 0,
		SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 0x4);
	if (ret) {
		dev_err(adsp->dev, "failed to set constraint %d\n", ret);
		return ret;
	}
	substream->runtime->private_data = prtd;
	prtd->substream = substream;
	prtd->dev = adsp->dev;

	return 0;
}

static int tegra210_adsp_pcm_close(struct snd_pcm_substream *substream)
{
	struct tegra210_adsp_pcm_rtd *prtd = substream->runtime->private_data;

	dev_vdbg(prtd->dev, "%s", __func__);

	if (prtd) {
		tegra210_adsp_send_reset_msg(prtd->fe_apm,
			TEGRA210_ADSP_MSG_FLAG_SEND);

		prtd->fe_apm->fe = 1;
		devm_kfree(prtd->dev, prtd);
	}

	return 0;
}

static int tegra210_adsp_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct tegra210_adsp_pcm_rtd *prtd = substream->runtime->private_data;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	int ret = 0;

	dev_vdbg(prtd->dev, "%s rate %d chan %d bps %d"
		"period size %d buffer size %d",
		 __func__, params_rate(params), params_channels(params),
		 snd_pcm_format_width(params_format(params)),
		 params_period_size(params),
		 params_buffer_bytes(params));

	ret = tegra210_adsp_send_io_buffer_msg(prtd->fe_apm, buf->addr,
					params_buffer_bytes(params),
					TEGRA210_ADSP_MSG_FLAG_HOLD);
	if (ret < 0)
		return ret;

	ret = tegra210_adsp_send_period_size_msg(prtd->fe_apm,
					params_period_size(params),
					TEGRA210_ADSP_MSG_FLAG_HOLD);
	if (ret < 0)
		return ret;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	return 0;
}

static int tegra210_adsp_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static int tegra210_adsp_pcm_trigger(struct snd_pcm_substream *substream,
				     int cmd)
{
	struct tegra210_adsp_pcm_rtd *prtd = substream->runtime->private_data;
	int ret = 0;

	dev_vdbg(prtd->dev, "%s : state %d", __func__, cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
			nvfx_state_active,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state");
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
			nvfx_state_inactive,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state");
			return ret;
		}

		ret = tegra210_adsp_send_flush_msg(prtd->fe_apm,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to reset");
			return ret;
		}
		break;
	default:
		dev_err(prtd->dev, "Unsupported state.");
		return -EINVAL;
	}

	return 0;
}

static int tegra210_adsp_pcm_ack(struct snd_pcm_substream *substream)
{
	struct tegra210_adsp_pcm_rtd *prtd = substream->runtime->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	size_t pos;
	int ret = 0;

	dev_vdbg(prtd->dev, "%s %d", __func__, (int)runtime->control->appl_ptr);

	pos = frames_to_bytes(runtime,
		runtime->control->appl_ptr % runtime->buffer_size);
	ret = tegra210_adsp_send_pos_msg(prtd->fe_apm, pos,
		TEGRA210_ADSP_MSG_FLAG_SEND);
	if (ret < 0) {
		dev_err(prtd->dev, "Failed to send write position.");
		return ret;
	}

	return ret;
}

static snd_pcm_uframes_t tegra210_adsp_pcm_pointer(
		struct snd_pcm_substream *substream)
{
	struct tegra210_adsp_pcm_rtd *prtd = substream->runtime->private_data;
	struct tegra210_adsp_app *app = prtd->fe_apm;
	size_t bytes, pos;

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		bytes = app->apm->nvfx_shared_state.output[0].bytes;
	else
		bytes = app->apm->nvfx_shared_state.input[0].bytes;

	pos = bytes % frames_to_bytes(substream->runtime,
		substream->runtime->buffer_size);

	/* TODO : If SRC in path do size conversion */

	dev_vdbg(prtd->dev, "%s bytes %zu position %zu", __func__, bytes, pos);
	return bytes_to_frames(substream->runtime, pos);
}

static struct snd_pcm_ops tegra210_adsp_pcm_ops = {
	.open		= tegra210_adsp_pcm_open,
	.close		= tegra210_adsp_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= tegra210_adsp_pcm_hw_params,
	.hw_free	= tegra210_adsp_pcm_hw_free,
	.trigger	= tegra210_adsp_pcm_trigger,
	.pointer	= tegra210_adsp_pcm_pointer,
	.ack		= tegra210_adsp_pcm_ack,
};

static int tegra210_adsp_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
#if ENABLE_ADSP
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	int ret = 0;

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		struct snd_pcm_substream *substream =
			pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;

		ret = tegra210_adsp_preallocate_dma_buffer(card->dev,
					adsp_pcm_hardware.buffer_bytes_max,
					&substream->dma_buffer);

		if (ret)
			return ret;
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		struct snd_pcm_substream *substream =
			pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;

		ret = tegra210_adsp_preallocate_dma_buffer(card->dev,
					adsp_pcm_hardware.buffer_bytes_max,
					&substream->dma_buffer);
		if (ret)
			goto err;
	}

	return 0;

err:
	tegra210_adsp_deallocate_dma_buffer(
		&pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream->dma_buffer);
	return ret;
#else
	return 0;
#endif
}

static void tegra210_adsp_pcm_free(struct snd_pcm *pcm)
{
	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		int stream = SNDRV_PCM_STREAM_PLAYBACK;

		tegra210_adsp_deallocate_dma_buffer(
			&pcm->streams[stream].substream->dma_buffer);
	}
	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		int stream = SNDRV_PCM_STREAM_CAPTURE;

		tegra210_adsp_deallocate_dma_buffer(
			&pcm->streams[stream].substream->dma_buffer);
	}
}

/* ADSP-ADMAIF codec driver HW-params. Used for configuring ADMA */
static int tegra210_adsp_admaif_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct tegra210_adsp *adsp = snd_soc_dai_get_drvdata(dai);
	struct tegra210_adsp_app *app;
	nvfx_adma_init_params_t adma_params;
	uint32_t be_reg = dai->id;
	uint32_t admaif_id = be_reg - ADSP_ADMAIF_START + 1;
	uint32_t source;
	int i, ret;

	dev_vdbg(adsp->dev, "%s : stream %d admaif %d\n",
		__func__, substream->stream, admaif_id);

	adma_params.mode = ADMA_MODE_CONTINUOUS;
	adma_params.ahub_channel = admaif_id;
	adma_params.periods = 2; /* We need ping-pong buffers for ADMA */

	/* Set DMA params connected with ADSP-BE */
	/* As a COCEC DAI, CAPTURE is transmit */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		app = &adsp->apps[be_reg];
		source = tegra210_adsp_get_source(adsp, app->reg);

		app = &adsp->apps[source];
		if (!IS_APM_OUT(app->reg))
			return 0;

		source = tegra210_adsp_get_source(adsp, app->reg);
		app = &adsp->apps[source];
		if (!IS_ADMA(app->reg))
			return 0;

		adma_params.adma_channel = app->adma_chan;
		adma_params.direction = ADMA_MEMORY_TO_AHUB;
		adma_params.event.pvoid = app->apm->output_event.pvoid;

		ret = tegra210_adsp_adma_params_msg(app, &adma_params,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		if (ret < 0) {
			dev_err(adsp->dev, "ADMA params msg failed. %d.", ret);
			return ret;
		}
	} else {
		for (i = ADMA_START; i < ADMA_END; i++) {
			app = &adsp->apps[i];
			source = tegra210_adsp_get_source(adsp, app->reg);
			if (!IS_APM_IN(source))
				continue;

			app = &adsp->apps[source];
			source = tegra210_adsp_get_source(adsp, app->reg);
			if (source != be_reg)
				continue;

			app = &adsp->apps[i];
			adma_params.adma_channel = app->adma_chan;
			adma_params.direction = ADMA_AHUB_TO_MEMORY;
			adma_params.event.pvoid = app->apm->input_event.pvoid;

			ret = tegra210_adsp_adma_params_msg(app,
					&adma_params,
					TEGRA210_ADSP_MSG_FLAG_SEND);
			if (ret < 0) {
				dev_err(adsp->dev, "ADMA params msg failed");
				return ret;
			}
		}
	}
	return 0;
}

/* ADSP platform driver read/write call-back */
static int tegra210_adsp_read(struct snd_soc_component *component,
		unsigned int reg, unsigned int *val)
{
	struct snd_soc_platform *platform =
				snd_soc_component_to_platform(component);
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);

	dev_vdbg(adsp->dev, "%s [0x%x] -> 0x%x\n", __func__, reg,
		tegra210_adsp_reg_read(adsp, reg));

	*val = tegra210_adsp_reg_read(adsp, reg);
	return 0;
}

static int tegra210_adsp_write(struct snd_soc_component *component,
		unsigned int reg,
		unsigned int val)
{
	struct snd_soc_platform *platform =
				snd_soc_component_to_platform(component);
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);

	dev_vdbg(adsp->dev, "%s [0x%x] -> 0x%x\n", __func__, reg, val);

	tegra210_adsp_reg_write(adsp, reg, val);
	return 0;
}

/* DAPM ENUM MUX get/put callbacks */
static int tegra210_adsp_mux_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
			snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_platform *platform = snd_soc_dapm_to_platform(dapm);
	struct soc_enum *e =
		(struct soc_enum *)kcontrol->private_value;
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);
	uint32_t val = tegra210_adsp_reg_read(adsp, e->reg);

	ucontrol->value.integer.value[0] =
		(val & TEGRA210_ADSP_WIDGET_SOURCE_MASK) >> e->shift_l;
	return 0;
}

static int tegra210_adsp_mux_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
			snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct snd_soc_platform *platform = snd_soc_dapm_to_platform(dapm);
	uint32_t val = ucontrol->value.enumerated.item[0];
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);
	struct tegra210_adsp_app *app;
	uint32_t cur_val = 0;
	int ret = 0;

	if (!adsp->init_done)
		return -ENODEV;

	if (e->reg >= TEGRA210_ADSP_VIRT_REG_MAX)
		return -EINVAL;

	/* Init or de-init app based on connection */
	if (IS_ADSP_APP(e->reg)) {
		app = &adsp->apps[e->reg];
		cur_val = tegra210_adsp_get_source(adsp, e->reg);
		if (cur_val != val) {
			if (app->connect) {
				/* remove existing connections if any */
				tegra210_adsp_remove_connection(adsp, app);
			}
			app->connect = 0;
		}

		if (val == TEGRA210_ADSP_NONE) {
			tegra210_adsp_app_deinit(adsp, app);
		} else {
			ret = tegra210_adsp_app_init(adsp, app);
			if (ret < 0) {
				dev_err(adsp->dev, "Failed to init app.");
				return -ENODEV;
			}
		}
	}
	tegra210_adsp_reg_update_bits(adsp, e->reg,
		TEGRA210_ADSP_WIDGET_SOURCE_MASK, val << e->shift_l);
	tegra210_adsp_update_connection(adsp);

	snd_soc_dapm_mux_update_power(dapm, kcontrol, val, e, NULL);
	return 1;
}

/* ALSA control get/put call-back implementation */
static int tegra210_adsp_init_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct tegra210_adsp *adsp = snd_soc_component_get_drvdata(cmpnt);

	ucontrol->value.enumerated.item[0] = adsp->init_done;
	return 0;
}

static int tegra210_adsp_init_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct tegra210_adsp *adsp = snd_soc_component_get_drvdata(cmpnt);
	int init = ucontrol->value.enumerated.item[0];
	int ret = 0;

	if (init == adsp->init_done)
		return 0;

	if (init) {
		ret = tegra210_adsp_init(adsp);
		if (ret < 0) {
			dev_err(adsp->dev, "Failed to init ADSP.");
			return ret;
		}
	} else {
		tegra210_adsp_deinit(adsp);
	}

	return 1;
}

/* DAPM widget event */
static int tegra210_adsp_widget_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_platform *platform = snd_soc_dapm_to_platform(w->dapm);
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);
	struct tegra210_adsp_app *app;

	if (!IS_ADSP_APP(w->reg))
		return 0;

	app = &adsp->apps[w->reg];
	/* For FE apm state change will be handled from trigger call back */
	if (app->fe)
		return 0;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		if (IS_APM_IN(w->reg)) {
			/* Request higher ADSP clock when starting stream.
			 * Actmon takes care of adjusting frequency later. */
			adsp_update_dfs(500000, 1);
			tegra210_adsp_send_state_msg(app, nvfx_state_active,
			TEGRA210_ADSP_MSG_FLAG_SEND);
		}
	} else {
		if (IS_APM_IN(w->reg)) {
			tegra210_adsp_send_state_msg(app, nvfx_state_inactive,
				TEGRA210_ADSP_MSG_FLAG_HOLD);
			tegra210_adsp_send_reset_msg(app,
				TEGRA210_ADSP_MSG_FLAG_SEND);
		}
	}

	return 0;
}

static struct snd_soc_dai_ops tegra210_adsp_admaif_dai_ops = {
	.hw_params	= tegra210_adsp_admaif_hw_params,
};

static struct snd_soc_dai_driver tegra210_adsp_dai[] = {
	{
		.name = "ADSP PCM",
		.playback = {
			.stream_name = "ADSP PCM Receive",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
			.stream_name = "ADSP PCM Transmit",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
	{
		.name = "ADSP COMPR1",
		.compress_dai = 1,
		.playback = {
			.stream_name = "ADSP COMPR1 Receive",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
	{
		.name = "ADSP COMPR2",
		.compress_dai = 1,
		.playback = {
			.stream_name = "ADSP COMPR2 Receive",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
};

#define ADSP_FE_CODEC_DAI(idx)					\
	{							\
		.name = "ADSP-FE" #idx,				\
		.id = ADSP_FE_START + (idx - 1),			\
		.playback = {					\
			.stream_name = "ADSP-FE" #idx " Receive",\
			.channels_min = 1,			\
			.channels_max = 2,			\
			.rates = SNDRV_PCM_RATE_8000_48000,	\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,	\
		},						\
		.capture = {					\
			.stream_name = "ADSP-FE" #idx " Transmit",\
			.channels_min = 1,			\
			.channels_max = 2,			\
			.rates = SNDRV_PCM_RATE_8000_48000,	\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,	\
		},						\
	}

#define ADSP_ADMAIF_CODEC_DAI(idx)				\
	{							\
		.name = "ADSP-ADMAIF" #idx,			\
		.id = ADSP_ADMAIF_START + (idx - 1),		\
		.playback = {					\
		.stream_name = "ADSP-ADMAIF" #idx " Receive",	\
			.channels_min = 1,			\
			.channels_max = 2,			\
			.rates = SNDRV_PCM_RATE_8000_48000,	\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,	\
		},						\
		.capture = {					\
			.stream_name = "ADSP-ADMAIF" #idx " Transmit",\
			.channels_min = 1,			\
			.channels_max = 2,			\
			.rates = SNDRV_PCM_RATE_8000_48000,	\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,	\
		},						\
		.ops = &tegra210_adsp_admaif_dai_ops,		\
	}

static struct snd_soc_dai_driver tegra210_adsp_codec_dai[] = {
	ADSP_FE_CODEC_DAI(1),
	ADSP_FE_CODEC_DAI(2),
	ADSP_FE_CODEC_DAI(3),
	ADSP_FE_CODEC_DAI(4),
	ADSP_FE_CODEC_DAI(5),
	ADSP_ADMAIF_CODEC_DAI(1),
	ADSP_ADMAIF_CODEC_DAI(2),
	ADSP_ADMAIF_CODEC_DAI(3),
	ADSP_ADMAIF_CODEC_DAI(4),
	ADSP_ADMAIF_CODEC_DAI(5),
	ADSP_ADMAIF_CODEC_DAI(6),
	ADSP_ADMAIF_CODEC_DAI(7),
	ADSP_ADMAIF_CODEC_DAI(8),
	ADSP_ADMAIF_CODEC_DAI(9),
	ADSP_ADMAIF_CODEC_DAI(10),
};

/* This array is linked with tegra210_adsp_virt_widgets enum defines. Any thing
   changed in enum define should be also reflected here and vice-versa */
static const char * const tegra210_adsp_mux_texts[] = {
	"None",
	"ADSP-FE1",
	"ADSP-FE2",
	"ADSP-FE3",
	"ADSP-FE4",
	"ADSP-FE5",
	"ADSP-ADMAIF1",
	"ADSP-ADMAIF2",
	"ADSP-ADMAIF3",
	"ADSP-ADMAIF4",
	"ADSP-ADMAIF5",
	"ADSP-ADMAIF6",
	"ADSP-ADMAIF7",
	"ADSP-ADMAIF8",
	"ADSP-ADMAIF9",
	"ADSP-ADMAIF10",
	"APM-IN1",
	"APM-IN2",
	"APM-IN3",
	"APM-IN4",
	"APM-IN5",
	"APM-IN6",
	"APM-IN7",
	"APM-IN8",
	"APM-OUT1",
	"APM-OUT2",
	"APM-OUT3",
	"APM-OUT4",
	"APM-OUT5",
	"APM-OUT6",
	"APM-OUT7",
	"APM-OUT8",
	"ADMA1",
	"ADMA2",
	"ADMA3",
	"ADMA4",
	"PLUGIN1",
	"PLUGIN2",
	"PLUGIN3",
	"PLUGIN4",
	"PLUGIN5",
	"PLUGIN6",
	"PLUGIN7",
	"PLUGIN8",
	"PLUGIN9",
	"PLUGIN10",
};

#define ADSP_MUX_ENUM_CTRL_DECL(ename, reg)		\
	SOC_ENUM_SINGLE_DECL(ename##_enum, reg,		\
		TEGRA210_ADSP_WIDGET_SOURCE_SHIFT,		\
		tegra210_adsp_mux_texts);				\
	static const struct snd_kcontrol_new ename##_ctrl =		\
		SOC_DAPM_ENUM_EXT("ADSP Route", ename##_enum,		\
			tegra210_adsp_mux_get, tegra210_adsp_mux_put)

static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe1, TEGRA210_ADSP_FRONT_END1);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe2, TEGRA210_ADSP_FRONT_END2);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe3, TEGRA210_ADSP_FRONT_END3);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe4, TEGRA210_ADSP_FRONT_END4);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_fe5, TEGRA210_ADSP_FRONT_END5);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif1, TEGRA210_ADSP_ADMAIF1);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif2, TEGRA210_ADSP_ADMAIF2);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif3, TEGRA210_ADSP_ADMAIF3);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif4, TEGRA210_ADSP_ADMAIF4);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif5, TEGRA210_ADSP_ADMAIF5);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif6, TEGRA210_ADSP_ADMAIF6);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif7, TEGRA210_ADSP_ADMAIF7);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif8, TEGRA210_ADSP_ADMAIF8);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif9, TEGRA210_ADSP_ADMAIF9);
static ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif10, TEGRA210_ADSP_ADMAIF10);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in1, TEGRA210_ADSP_APM_IN1);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in2, TEGRA210_ADSP_APM_IN2);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in3, TEGRA210_ADSP_APM_IN3);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in4, TEGRA210_ADSP_APM_IN4);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in5, TEGRA210_ADSP_APM_IN5);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in6, TEGRA210_ADSP_APM_IN6);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in7, TEGRA210_ADSP_APM_IN7);
static ADSP_MUX_ENUM_CTRL_DECL(apm_in8, TEGRA210_ADSP_APM_IN8);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out1, TEGRA210_ADSP_APM_OUT1);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out2, TEGRA210_ADSP_APM_OUT2);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out3, TEGRA210_ADSP_APM_OUT3);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out4, TEGRA210_ADSP_APM_OUT4);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out5, TEGRA210_ADSP_APM_OUT5);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out6, TEGRA210_ADSP_APM_OUT6);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out7, TEGRA210_ADSP_APM_OUT7);
static ADSP_MUX_ENUM_CTRL_DECL(apm_out8, TEGRA210_ADSP_APM_OUT8);
static ADSP_MUX_ENUM_CTRL_DECL(adma1, TEGRA210_ADSP_PLUGIN_ADMA1);
static ADSP_MUX_ENUM_CTRL_DECL(adma2, TEGRA210_ADSP_PLUGIN_ADMA2);
static ADSP_MUX_ENUM_CTRL_DECL(adma3, TEGRA210_ADSP_PLUGIN_ADMA3);
static ADSP_MUX_ENUM_CTRL_DECL(adma4, TEGRA210_ADSP_PLUGIN_ADMA4);
static ADSP_MUX_ENUM_CTRL_DECL(plugin1, TEGRA210_ADSP_PLUGIN1);
static ADSP_MUX_ENUM_CTRL_DECL(plugin2, TEGRA210_ADSP_PLUGIN2);
static ADSP_MUX_ENUM_CTRL_DECL(plugin3, TEGRA210_ADSP_PLUGIN3);
static ADSP_MUX_ENUM_CTRL_DECL(plugin4, TEGRA210_ADSP_PLUGIN4);
static ADSP_MUX_ENUM_CTRL_DECL(plugin5, TEGRA210_ADSP_PLUGIN5);
static ADSP_MUX_ENUM_CTRL_DECL(plugin6, TEGRA210_ADSP_PLUGIN6);
static ADSP_MUX_ENUM_CTRL_DECL(plugin7, TEGRA210_ADSP_PLUGIN7);
static ADSP_MUX_ENUM_CTRL_DECL(plugin8, TEGRA210_ADSP_PLUGIN8);
static ADSP_MUX_ENUM_CTRL_DECL(plugin9, TEGRA210_ADSP_PLUGIN9);
static ADSP_MUX_ENUM_CTRL_DECL(plugin10, TEGRA210_ADSP_PLUGIN10);

#define ADSP_EP_WIDGETS(sname, ename)					\
	SND_SOC_DAPM_AIF_IN(sname " RX", NULL, 0, SND_SOC_NOPM, 0, 0),	\
	SND_SOC_DAPM_AIF_OUT(sname " TX", NULL, 0, SND_SOC_NOPM, 0, 0),	\
	SND_SOC_DAPM_MUX(sname " MUX", SND_SOC_NOPM, 0, 0, &ename##_ctrl)

#define ADSP_WIDGETS(sname, ename, reg)					\
	SND_SOC_DAPM_AIF_OUT_E(sname " TX", NULL, 0, reg,		\
		TEGRA210_ADSP_WIDGET_EN_SHIFT, 0, tegra210_adsp_widget_event, \
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),		\
	SND_SOC_DAPM_MUX(sname " MUX", SND_SOC_NOPM, 0, 0, &ename##_ctrl)

static const struct snd_soc_dapm_widget tegra210_adsp_widgets[] = {
	ADSP_EP_WIDGETS("ADSP-FE1", adsp_fe1),
	ADSP_EP_WIDGETS("ADSP-FE2", adsp_fe2),
	ADSP_EP_WIDGETS("ADSP-FE3", adsp_fe3),
	ADSP_EP_WIDGETS("ADSP-FE4", adsp_fe4),
	ADSP_EP_WIDGETS("ADSP-FE5", adsp_fe5),
	ADSP_EP_WIDGETS("ADSP-ADMAIF1", adsp_admaif1),
	ADSP_EP_WIDGETS("ADSP-ADMAIF2", adsp_admaif2),
	ADSP_EP_WIDGETS("ADSP-ADMAIF3", adsp_admaif3),
	ADSP_EP_WIDGETS("ADSP-ADMAIF4", adsp_admaif4),
	ADSP_EP_WIDGETS("ADSP-ADMAIF5", adsp_admaif5),
	ADSP_EP_WIDGETS("ADSP-ADMAIF6", adsp_admaif6),
	ADSP_EP_WIDGETS("ADSP-ADMAIF7", adsp_admaif7),
	ADSP_EP_WIDGETS("ADSP-ADMAIF8", adsp_admaif8),
	ADSP_EP_WIDGETS("ADSP-ADMAIF9", adsp_admaif9),
	ADSP_EP_WIDGETS("ADSP-ADMAIF10", adsp_admaif10),
	ADSP_WIDGETS("APM-IN1", apm_in1, TEGRA210_ADSP_APM_IN1),
	ADSP_WIDGETS("APM-IN2", apm_in2, TEGRA210_ADSP_APM_IN2),
	ADSP_WIDGETS("APM-IN3", apm_in3, TEGRA210_ADSP_APM_IN3),
	ADSP_WIDGETS("APM-IN4", apm_in4, TEGRA210_ADSP_APM_IN4),
	ADSP_WIDGETS("APM-IN5", apm_in5, TEGRA210_ADSP_APM_IN5),
	ADSP_WIDGETS("APM-IN6", apm_in6, TEGRA210_ADSP_APM_IN6),
	ADSP_WIDGETS("APM-IN7", apm_in7, TEGRA210_ADSP_APM_IN7),
	ADSP_WIDGETS("APM-IN8", apm_in8, TEGRA210_ADSP_APM_IN8),
	ADSP_WIDGETS("APM-OUT1", apm_out1, TEGRA210_ADSP_APM_OUT1),
	ADSP_WIDGETS("APM-OUT2", apm_out2, TEGRA210_ADSP_APM_OUT2),
	ADSP_WIDGETS("APM-OUT3", apm_out3, TEGRA210_ADSP_APM_OUT3),
	ADSP_WIDGETS("APM-OUT4", apm_out4, TEGRA210_ADSP_APM_OUT4),
	ADSP_WIDGETS("APM-OUT5", apm_out5, TEGRA210_ADSP_APM_OUT5),
	ADSP_WIDGETS("APM-OUT6", apm_out6, TEGRA210_ADSP_APM_OUT6),
	ADSP_WIDGETS("APM-OUT7", apm_out7, TEGRA210_ADSP_APM_OUT7),
	ADSP_WIDGETS("APM-OUT8", apm_out8, TEGRA210_ADSP_APM_OUT8),
	ADSP_WIDGETS("ADMA1", adma1, TEGRA210_ADSP_PLUGIN_ADMA1),
	ADSP_WIDGETS("ADMA2", adma2, TEGRA210_ADSP_PLUGIN_ADMA2),
	ADSP_WIDGETS("ADMA3", adma3, TEGRA210_ADSP_PLUGIN_ADMA3),
	ADSP_WIDGETS("ADMA4", adma4, TEGRA210_ADSP_PLUGIN_ADMA4),
	ADSP_WIDGETS("PLUGIN1", plugin1, TEGRA210_ADSP_PLUGIN1),
	ADSP_WIDGETS("PLUGIN2", plugin2, TEGRA210_ADSP_PLUGIN2),
	ADSP_WIDGETS("PLUGIN3", plugin3, TEGRA210_ADSP_PLUGIN3),
	ADSP_WIDGETS("PLUGIN4", plugin4, TEGRA210_ADSP_PLUGIN4),
	ADSP_WIDGETS("PLUGIN5", plugin5, TEGRA210_ADSP_PLUGIN5),
	ADSP_WIDGETS("PLUGIN6", plugin6, TEGRA210_ADSP_PLUGIN6),
	ADSP_WIDGETS("PLUGIN7", plugin7, TEGRA210_ADSP_PLUGIN7),
	ADSP_WIDGETS("PLUGIN8", plugin8, TEGRA210_ADSP_PLUGIN8),
	ADSP_WIDGETS("PLUGIN9", plugin9, TEGRA210_ADSP_PLUGIN9),
	ADSP_WIDGETS("PLUGIN10", plugin10, TEGRA210_ADSP_PLUGIN10),
};

#define ADSP_EP_ROUTES(name)					\
	{ name " MUX",		"ADSP-FE1", "ADSP-FE1 RX"},	\
	{ name " MUX",		"ADSP-FE2", "ADSP-FE2 RX"},	\
	{ name " MUX",		"ADSP-FE3", "ADSP-FE3 RX"},	\
	{ name " MUX",		"ADSP-FE4", "ADSP-FE4 RX"},	\
	{ name " MUX",		"ADSP-FE5", "ADSP-FE5 RX"},	\
	{ name " MUX",		"ADSP-ADMAIF1", "ADSP-ADMAIF1 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF2", "ADSP-ADMAIF2 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF3", "ADSP-ADMAIF3 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF4", "ADSP-ADMAIF4 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF5", "ADSP-ADMAIF5 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF6", "ADSP-ADMAIF6 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF7", "ADSP-ADMAIF7 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF8", "ADSP-ADMAIF8 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF9", "ADSP-ADMAIF9 RX"}, \
	{ name " MUX",		"ADSP-ADMAIF10", "ADSP-ADMAIF10 RX"}

#define ADSP_APM_IN_ROUTES(name)				\
	{ name " MUX",	"APM-IN1",	"APM-IN1 TX"},		\
	{ name " MUX",	"APM-IN2",	"APM-IN2 TX"},		\
	{ name " MUX",	"APM-IN3",	"APM-IN3 TX"},		\
	{ name " MUX",	"APM-IN4",	"APM-IN4 TX"},		\
	{ name " MUX",	"APM-IN5",	"APM-IN5 TX"},		\
	{ name " MUX",	"APM-IN6",	"APM-IN6 TX"},		\
	{ name " MUX",	"APM-IN7",	"APM-IN7 TX"},		\
	{ name " MUX",	"APM-IN8",	"APM-IN8 TX"}

#define ADSP_APM_OUT_ROUTES(name)				\
	{ name " MUX",		"APM-OUT1", "APM-OUT1 TX"},	\
	{ name " MUX",		"APM-OUT2", "APM-OUT2 TX"},	\
	{ name " MUX",		"APM-OUT3", "APM-OUT3 TX"},	\
	{ name " MUX",		"APM-OUT4", "APM-OUT4 TX"},	\
	{ name " MUX",		"APM-OUT5", "APM-OUT5 TX"},	\
	{ name " MUX",		"APM-OUT6", "APM-OUT6 TX"},	\
	{ name " MUX",		"APM-OUT7", "APM-OUT7 TX"},	\
	{ name " MUX",		"APM-OUT8", "APM-OUT8 TX"}

#define ADSP_ADMA_ROUTES(name)					\
	{ name " MUX",	"ADMA1",	"ADMA1 TX"},		\
	{ name " MUX",	"ADMA2",	"ADMA2 TX"},		\
	{ name " MUX",	"ADMA3",	"ADMA3 TX"},		\
	{ name " MUX",	"ADMA4",	"ADMA4 TX"}

#define ADSP_PLUGIN_ROUTES(name)					\
	{ name " MUX",	"PLUGIN1",	"PLUGIN1 TX"},		\
	{ name " MUX",	"PLUGIN2",	"PLUGIN2 TX"},		\
	{ name " MUX",	"PLUGIN3",	"PLUGIN3 TX"},		\
	{ name " MUX",	"PLUGIN4",	"PLUGIN4 TX"},		\
	{ name " MUX",	"PLUGIN5",	"PLUGIN5 TX"},		\
	{ name " MUX",	"PLUGIN6",	"PLUGIN6 TX"},		\
	{ name " MUX",	"PLUGIN7",	"PLUGIN7 TX"},		\
	{ name " MUX",	"PLUGIN8",	"PLUGIN8 TX"},		\
	{ name " MUX",	"PLUGIN9",	"PLUGIN9 TX"},		\
	{ name " MUX",	"PLUGIN10",	"PLUGIN10 TX"}

#define ADSP_EP_MUX_ROUTES(name)				\
	{ name " RX",		NULL, name " Receive"},		\
	{ name " Transmit",	NULL, name " TX"},		\
	{ name " TX",		NULL, name " MUX"},		\
	ADSP_APM_OUT_ROUTES(name)

#define ADSP_APM_IN_MUX_ROUTES(name)				\
	{ name " TX",		NULL, name " MUX"},		\
	ADSP_EP_ROUTES(name),					\
	ADSP_APM_OUT_ROUTES(name)

#define ADSP_APM_OUT_MUX_ROUTES(name)				\
	{ name " TX",		NULL, name " MUX"},		\
	ADSP_ADMA_ROUTES(name),					\
	ADSP_PLUGIN_ROUTES(name)

#define ADSP_PLUGIN_MUX_ROUTES(name)				\
	{ name " TX",		NULL, name " MUX"},		\
	ADSP_APM_IN_ROUTES(name),					\
	ADSP_PLUGIN_ROUTES(name),					\
	ADSP_ADMA_ROUTES(name)

#define ADSP_ADMA_MUX_ROUTES(name)				\
	{ name " TX",		NULL, name " MUX"},		\
	ADSP_APM_IN_ROUTES(name),				\
	ADSP_PLUGIN_ROUTES(name)

static const struct snd_soc_dapm_route tegra210_adsp_routes[] = {
	ADSP_EP_MUX_ROUTES("ADSP-FE1"),
	ADSP_EP_MUX_ROUTES("ADSP-FE2"),
	ADSP_EP_MUX_ROUTES("ADSP-FE3"),
	ADSP_EP_MUX_ROUTES("ADSP-FE4"),
	ADSP_EP_MUX_ROUTES("ADSP-FE5"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF1"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF2"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF3"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF4"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF5"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF6"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF7"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF8"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF9"),
	ADSP_EP_MUX_ROUTES("ADSP-ADMAIF10"),

	ADSP_APM_IN_MUX_ROUTES("APM-IN1"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN2"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN3"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN4"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN5"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN6"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN7"),
	ADSP_APM_IN_MUX_ROUTES("APM-IN8"),

	ADSP_APM_OUT_MUX_ROUTES("APM-OUT1"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT2"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT3"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT4"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT5"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT6"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT7"),
	ADSP_APM_OUT_MUX_ROUTES("APM-OUT8"),

	ADSP_ADMA_MUX_ROUTES("ADMA1"),
	ADSP_ADMA_MUX_ROUTES("ADMA2"),
	ADSP_ADMA_MUX_ROUTES("ADMA3"),
	ADSP_ADMA_MUX_ROUTES("ADMA4"),

	ADSP_PLUGIN_MUX_ROUTES("PLUGIN1"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN2"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN3"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN4"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN5"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN6"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN7"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN8"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN9"),
	ADSP_PLUGIN_MUX_ROUTES("PLUGIN10"),
};

static void tegra210_adsp_wt_replace(char *dest, const char *src)
{
	if (!dest || !src)
		return;

	if (strstr(dest, " TX")) {
		strcpy(dest, src);
		strcat(dest, " TX");
	} else if (strstr(dest, " RX")) {
		strcpy(dest, src);
		strcat(dest, " RX");
	} else if (strstr(dest, " MUX")) {
		strcpy(dest, src);
		strcat(dest, " MUX");
	} else {
		strcpy(dest, src);
	}
}

static void tegra210_adsp_route_modify(const char *wt_default,
				const char *wt_from_dt)
{
	int i;

	if (!wt_default || !wt_from_dt)
		return;

	/* Modify dapm routing table */
	for (i = TEGRA210_ADSP_ROUTE_BASE;
		i < ARRAY_SIZE(tegra210_adsp_routes); i++) {
		/* replace sink name */
		if (tegra210_adsp_routes[i].sink)
			if (strstr(tegra210_adsp_routes[i].sink, wt_default))
				tegra210_adsp_wt_replace(
					(char *)tegra210_adsp_routes[i].sink,
					wt_from_dt);
		/* replace control name */
		if (tegra210_adsp_routes[i].control)
			if (strstr(tegra210_adsp_routes[i].control, wt_default))
				tegra210_adsp_wt_replace(
					(char *)tegra210_adsp_routes[i].control,
					wt_from_dt);
		/* replace source name */
		if (tegra210_adsp_routes[i].source)
			if (strstr(tegra210_adsp_routes[i].source, wt_default))
				tegra210_adsp_wt_replace(
					(char *)tegra210_adsp_routes[i].source,
					wt_from_dt);
	}
}

static int tegra210_adsp_param_info(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_info *uinfo)
{
	struct soc_bytes *params = (void *)kcontrol->private_value;

	if (params->mask == SNDRV_CTL_ELEM_TYPE_INTEGER) {
		params->num_regs = 128;
		uinfo->value.integer.min = 0;
		uinfo->value.integer.max = 0x7fffffff;
	}
	uinfo->type = params->mask;
	uinfo->count = params->num_regs;

	return 0;
}

static int tegra210_adsp_get_param(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_bytes *params = (void *)kcontrol->private_value;

	if (params->mask == SNDRV_CTL_ELEM_TYPE_INTEGER)
		memset(ucontrol->value.integer.value, 0,
			params->num_regs * sizeof(long));
	else
		memset(ucontrol->value.bytes.data, 0,
			params->num_regs);

	return 0;
}

/* tegra210_adsp_set_param - sets plugin parameters
 * @default: byte_format
 * @byte_format: nvfx_call_params_t based structure
 * @int_format: <plugin_method>,<#params>,<param1>,<param2>,....
 */
static int tegra210_adsp_set_param(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_bytes *params = (void *)kcontrol->private_value;
	struct snd_soc_component *cmpnt = snd_soc_kcontrol_component(kcontrol);
	struct tegra210_adsp *adsp = snd_soc_component_get_drvdata(cmpnt);
	struct tegra210_adsp_app *app = &adsp->apps[params->base];
	apm_msg_t apm_msg;

	if (!adsp->init_done) {
		dev_warn(adsp->dev, "ADSP is not booted yet\n");
		return 0;
	}

	if (!app->plugin) {
		dev_warn(adsp->dev, "Plugin not yet initialized\n");
		return 0;
	}

	apm_msg.msgq_msg.size = MSGQ_MSG_WSIZE(apm_fx_set_param_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_fx_set_param_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_fx_set_param;
	apm_msg.msg.fx_set_param_params.plugin.pvoid =
		app->plugin->plugin.pvoid;

	switch (params->mask) {
	case SNDRV_CTL_ELEM_TYPE_INTEGER:
	{
		int32_t num_params, i;
		/* check number of params to pass */
		num_params = (int32_t)ucontrol->value.integer.value[1];
		if (num_params < 1) {
			dev_warn(adsp->dev, "No params to pass to the plugin\n");
			return 0;
		}
		apm_msg.msg.fx_set_param_params.params[0] =
			(sizeof(nvfx_call_params_t) +
			num_params * sizeof(int32_t));

		/* initialize the method */
		apm_msg.msg.fx_set_param_params.params[1] =
			(int32_t)ucontrol->value.integer.value[0];

		/* copy parameters */
		for (i = 0; i < num_params; i++)
			apm_msg.msg.fx_set_param_params.params[i + 2] =
				(int32_t)ucontrol->value.integer.value[i + 2];
	}
	break;

	case SNDRV_CTL_ELEM_TYPE_BYTES:
	{
		nvfx_call_params_t *call_params =
			(nvfx_call_params_t *)ucontrol->value.bytes.data;

		/* copy parameters */
		memcpy(&apm_msg.msg.fx_set_param_params.params,
			call_params, call_params->size);
	}
	break;

	default:
		return -EINVAL;
	}

	return tegra210_adsp_send_msg(app->apm, &apm_msg,
		TEGRA210_ADSP_MSG_FLAG_SEND);
}

/* Maximum 128 integers or 512 bytes allowed */
#define SND_SOC_PARAM_EXT(xname, xbase)		\
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	\
	.name = xname,		\
	.info = tegra210_adsp_param_info,		\
	.get = tegra210_adsp_get_param,		\
	.put = tegra210_adsp_set_param,		\
	.private_value =		\
		((unsigned long)&(struct soc_bytes)		\
		{.base = xbase, .num_regs = 512,		\
		.mask = SNDRV_CTL_ELEM_TYPE_BYTES}) }

static const struct snd_kcontrol_new tegra210_adsp_controls[] = {
	SOC_SINGLE_BOOL_EXT("ADSP init", 0,
		tegra210_adsp_init_get, tegra210_adsp_init_put),
	SND_SOC_PARAM_EXT("PLUGIN1 set params",
		TEGRA210_ADSP_PLUGIN1),
	SND_SOC_PARAM_EXT("PLUGIN2 set params",
		TEGRA210_ADSP_PLUGIN2),
	SND_SOC_PARAM_EXT("PLUGIN3 set params",
		TEGRA210_ADSP_PLUGIN3),
	SND_SOC_PARAM_EXT("PLUGIN4 set params",
		TEGRA210_ADSP_PLUGIN4),
	SND_SOC_PARAM_EXT("PLUGIN5 set params",
		TEGRA210_ADSP_PLUGIN5),
	SND_SOC_PARAM_EXT("PLUGIN6 set params",
		TEGRA210_ADSP_PLUGIN6),
	SND_SOC_PARAM_EXT("PLUGIN7 set params",
		TEGRA210_ADSP_PLUGIN7),
	SND_SOC_PARAM_EXT("PLUGIN8 set params",
		TEGRA210_ADSP_PLUGIN8),
	SND_SOC_PARAM_EXT("PLUGIN9 set params",
		TEGRA210_ADSP_PLUGIN9),
	SND_SOC_PARAM_EXT("PLUGIN10 set params",
		TEGRA210_ADSP_PLUGIN10),
};

static const struct snd_soc_component_driver tegra210_adsp_component = {
	.name		= "tegra210-adsp",
	.dapm_widgets		= tegra210_adsp_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tegra210_adsp_widgets),
	.dapm_routes		= tegra210_adsp_routes,
	.num_dapm_routes	= ARRAY_SIZE(tegra210_adsp_routes),
	.controls		= tegra210_adsp_controls,
	.num_controls		= ARRAY_SIZE(tegra210_adsp_controls),
};

static int tegra210_adsp_codec_probe(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_codec_driver tegra210_adsp_codec = {
	.probe = tegra210_adsp_codec_probe,
	.idle_bias_off = 1,
};

static int tegra210_adsp_pcm_probe(struct snd_soc_platform *platform)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(&platform->component);
	dapm->idle_bias_off = 1;
	platform->component.read = tegra210_adsp_read;
	platform->component.write = tegra210_adsp_write;
	return 0;
}

static struct snd_soc_platform_driver tegra210_adsp_platform = {
	.ops			= &tegra210_adsp_pcm_ops,
	.compr_ops		= &tegra210_adsp_compr_ops,
	.pcm_new		= tegra210_adsp_pcm_new,
	.pcm_free		= tegra210_adsp_pcm_free,
	.probe			= tegra210_adsp_pcm_probe,
};

static u64 tegra_dma_mask = DMA_BIT_MASK(32);

static const struct of_device_id tegra210_adsp_audio_of_match[] = {
	{ .compatible = "nvidia,tegra210-adsp-audio", },
	{},
};

static int tegra210_adsp_audio_platform_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node, *subnp;
	const struct of_device_id *match;
	struct soc_bytes *controls;
	struct tegra210_adsp *adsp;
	int i, j, wt_idx, mux_idx, ret = 0;
	unsigned int compr_ops = 1;
	char plugin_info[20];

	pr_info("tegra210_adsp_audio_platform_probe: platform probe started\n");

	match = of_match_device(tegra210_adsp_audio_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		return -ENODEV;
	}

	adsp = devm_kzalloc(&pdev->dev, sizeof(*adsp), GFP_KERNEL);
	if (!adsp) {
		dev_err(&pdev->dev, "Can't allocate tegra210_adsp_ctx\n");
		return -ENOMEM;
	}
	dev_set_drvdata(&pdev->dev, adsp);
	adsp->dev = &pdev->dev;

	/* TODO: Add mixer control to set I2S playback rate */
	adsp->i2s_rate = 48000;
	mutex_init(&adsp->mutex);
	pdev->dev.dma_mask = &tegra_dma_mask;
	pdev->dev.coherent_dma_mask = tegra_dma_mask;

	tegra_ape_pd_add_device(&pdev->dev);
	pm_genpd_dev_need_save(&pdev->dev, true);
	pm_genpd_dev_need_restore(&pdev->dev, true);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev))
		goto err_pm_disable;

	pm_runtime_get_sync(&pdev->dev);
	/* HACK : Should be handled through dma-engine */
	for (i = 0; i < TEGRA210_ADSP_ADMA_CHANNEL_COUNT; i++) {
		ret = tegra_agic_route_interrupt(
			INT_ADMA_EOT0 + TEGRA210_ADSP_ADMA_CHANNEL_START + i,
			TEGRA_AGIC_ADSP);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to route INT to ADSP");
			goto err_pm_disable;
		}
	}
	/* HACK end */
	pm_runtime_put(&pdev->dev);

	for (i = 0; i < TEGRA210_ADSP_VIRT_REG_MAX; i++)
		adsp->apps[i].reg = i;

	/* get the plugin count */
	if (of_property_read_u32(pdev->dev.of_node,
				"num-plugin",
				&adsp_app_count) < 0) {
		dev_warn(&pdev->dev, "Missing ADSP plugin count\n");
		adsp_app_count = 0;
	}

	/* allocate memory for app descritors */
	adsp_app_desc = devm_kzalloc(&pdev->dev,
				(adsp_app_count + ARRAY_SIZE(adsp_app_minimal))
				* sizeof(*adsp_app_desc), GFP_KERNEL);
	if (!adsp_app_desc) {
		dev_err(&pdev->dev, "Can't allocate tegra210_adsp_app descriptor\n");
		ret = -ENOMEM;
		goto err_pm_disable;
	}

	/* parse the plugin, firmware, widget names and params */
	for (i = 0; i < adsp_app_count; i++) {
		memset((void *)plugin_info, '\0', 20);
		sprintf(plugin_info, "plugin-info-%d", i+1);
		subnp = of_get_child_by_name(np, plugin_info);
		if (subnp) {
			if (of_property_read_string(subnp, "plugin-name",
				&adsp_app_desc[i].name)) {
				dev_err(&pdev->dev,
					"Missing property plugin-name\n");
				ret = -EINVAL;
				goto err_pm_disable;
			}
			if (of_property_read_string(subnp, "firmware-name",
				&adsp_app_desc[i].fw_name)) {
				dev_err(&pdev->dev,
					"Missing property firmware-name\n");
				ret = -EINVAL;
				goto err_pm_disable;
			}
			if (of_property_read_string(subnp, "widget-name",
				&adsp_app_desc[i].wt_name)) {
				dev_warn(&pdev->dev,
					"Missing property widget-name for %s\n",
					adsp_app_desc[i].name);
				adsp_app_desc[i].wt_name = NULL;
			} else {
				if (adsp_app_desc[i].wt_name) {
					/* override the widget names from DT if any */
					mux_idx = TEGRA210_ADSP_PLUGIN1 + i;
					wt_idx = TEGRA210_ADSP_WIDGET_BASE + (2*i);
					tegra210_adsp_route_modify(
						tegra210_adsp_mux_texts[mux_idx],
						adsp_app_desc[i].wt_name);
					strcpy((char *)tegra210_adsp_widgets[wt_idx].name,
						adsp_app_desc[i].wt_name);
					strcpy((char *)tegra210_adsp_widgets[wt_idx+1].name,
						adsp_app_desc[i].wt_name);
					strcpy((char *)tegra210_adsp_controls[i+1].name,
						adsp_app_desc[i].wt_name);
					strcat((char *)tegra210_adsp_widgets[wt_idx].name,
						" TX");
					strcat((char *)tegra210_adsp_widgets[wt_idx+1].name,
						" MUX");
					strcat((char *)tegra210_adsp_controls[i+1].name,
						" set params");
					strcpy((char *)tegra210_adsp_mux_texts[mux_idx],
						adsp_app_desc[i].wt_name);
				}
			}
			if (of_property_read_u32(subnp, "param-type",
				&adsp_app_desc[i].param_type)) {
				dev_warn(&pdev->dev,
					"Default param-type to BYTE for %s\n",
					adsp_app_desc[i].name);
				adsp_app_desc[i].param_type =
					SNDRV_CTL_ELEM_TYPE_BYTES;
			} else {
				/* override the param-type from DT if any */
				controls =
					(void *)tegra210_adsp_controls[i+1].private_value;
				controls->mask = adsp_app_desc[i].param_type;
			}
			adsp_app_desc[i].reg_start = TEGRA210_ADSP_PLUGIN1 + i;
			adsp_app_desc[i].reg_end = TEGRA210_ADSP_PLUGIN1 + i;
		} else {
			dev_err(&pdev->dev,
				"Property '%s' missing or invalid\n",
				plugin_info);
			ret = -EINVAL;
			goto err_pm_disable;
		}
	}

	/* copy basic apps needed */
	memcpy(&adsp_app_desc[adsp_app_count],
			&adsp_app_minimal[0], sizeof(adsp_app_minimal));
	adsp_app_count += ARRAY_SIZE(adsp_app_minimal);

	for (i = 0; i < adsp_app_count; i++) {
		for (j = adsp_app_desc[i].reg_start;
			j <= adsp_app_desc[i].reg_end; j++)
			adsp->apps[j].desc = &adsp_app_desc[i];
	}

	/* enable/disable compr-ops from DT */
	of_property_read_u32(pdev->dev.of_node, "compr-ops", &compr_ops);
	if (!compr_ops)
		tegra210_adsp_platform.compr_ops = NULL;

	ret = snd_soc_register_platform(&pdev->dev, &tegra210_adsp_platform);
	if (ret) {
		dev_err(&pdev->dev, "Could not register platform: %d\n", ret);
		goto err_pm_disable;
	}

	ret = snd_soc_register_component(&pdev->dev, &tegra210_adsp_component,
			tegra210_adsp_dai, ARRAY_SIZE(tegra210_adsp_dai));
	if (ret) {
		dev_err(&pdev->dev, "Could not register component: %d\n", ret);
		goto err_unregister_platform;
	}

	ret = snd_soc_register_codec(&pdev->dev, &tegra210_adsp_codec,
				     tegra210_adsp_codec_dai,
				     ARRAY_SIZE(tegra210_adsp_codec_dai));
	if (ret != 0) {
		dev_err(&pdev->dev, "Could not register CODEC: %d\n", ret);
		goto err_unregister_platform;
	}

	pr_info("tegra210_adsp_audio_platform_probe probe successfull.");
	return 0;

err_unregister_platform:
	snd_soc_unregister_platform(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
	tegra_ape_pd_remove_device(&pdev->dev);
	return ret;
}

static int tegra210_adsp_audio_platform_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	tegra_ape_pd_remove_device(&pdev->dev);
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver tegra210_adsp_audio_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_adsp_audio_of_match,
	},
	.probe = tegra210_adsp_audio_platform_probe,
	.remove = tegra210_adsp_audio_platform_remove,
};
module_platform_driver(tegra210_adsp_audio_driver);

MODULE_AUTHOR("Sumit Bhattacharya <sumitb@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 ADSP Audio driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra210_adsp_audio_of_match);

