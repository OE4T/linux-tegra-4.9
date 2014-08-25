/*
 * tegra210_adsp_alt.c - Tegra ADSP audio driver
 *
 * Author: Sumit Bhattacharya <sumitb@nvidia.com>
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/tegra_nvadsp.h>
#include <linux/irqchip/tegra-agic.h>

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
#define ENABLE_ADSP 0

static struct tegra210_adsp_app_desc {
	const char name[NVADSP_NAME_SZ];
	const char fw_name[NVADSP_NAME_SZ];
	const uint32_t reg_start;
	const uint32_t reg_end;
	nvadsp_app_handle_t handle;
} adsp_app_desc[] = {
	{"apm", "nvapm.elf",
		TEGRA210_ADSP_APM_IN1, TEGRA210_ADSP_APM_IN8},
	{"mp3dec", "nvmp3dec.elf",
		TEGRA210_ADSP_PLUGIN_MP3_DEC1, TEGRA210_ADSP_PLUGIN_MP3_DEC2},
	{"adma", "nvadma.elf",
		TEGRA210_ADSP_PLUGIN_ADMA1, TEGRA210_ADSP_PLUGIN_ADMA4},
	{"spkprot", "nvspkprot.elf",
		TEGRA210_ADSP_PLUGIN_SPKPROT, TEGRA210_ADSP_PLUGIN_SPKPROT},
};

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
	struct work_struct work;
	struct tegra210_adsp_app apps[TEGRA210_ADSP_VIRT_REG_MAX];
	atomic_t reg_val[TEGRA210_ADSP_VIRT_REG_MAX];
	DECLARE_BITMAP(adma_usage, TEGRA210_ADSP_ADMA_CHANNEL_COUNT);
	struct mutex mutex;
	int init_done;
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
struct snd_compr_caps tegra210_adsp_compr_caps[SND_COMPRESS_CAPTURE + 1] = {
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
struct snd_compr_codec_caps adsp_compr_codec_caps[] = {
	[SND_AUDIOCODEC_MP3] = {
		.codec = SND_AUDIOCODEC_MP3,
		.num_descriptors = 1,
		.descriptor = {
			[0] = {
				.max_ch = 2,
				.sample_rates = SNDRV_PCM_RATE_8000_48000,
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
				.sample_rates = SNDRV_PCM_RATE_8000_48000,
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
	for (i = 0; i < ARRAY_SIZE(adsp_app_desc); i++) {
		 adsp_app_desc[i].handle = nvadsp_app_load(
				adsp_app_desc[i].name,
				adsp_app_desc[i].fw_name);
		if (ret < 0) {
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

static void tegra210_adsp_init_work(struct work_struct *work)
{
	struct tegra210_adsp *adsp =
		container_of(work, struct tegra210_adsp, work);
	int retry = 10;
	int ret = 0;

	while (retry) {
		ret = tegra210_adsp_init(adsp);
		if (!ret)
			break;

		dev_warn(adsp->dev, "Failed to load adsp OS. Will retry.");
		msleep(5000);
		retry--;
	}
}

/* ADSP-CPU message send-receive utility functions */
static int tegra210_adsp_get_msg(apm_shared_state_t *apm, apm_msg_t *apm_msg)
{
	apm_msg->msgq_msg.size = MSGQ_MSG_SIZE(apm_msg_t) -
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

static int tegra210_adsp_send_connect_msg(struct tegra210_adsp_app *src,
					struct tegra210_adsp_app *dst,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_SIZE(apm_fx_connect_params_t);
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

	apm_msg.msgq_msg.size = MSGQ_MSG_SIZE(apm_io_buffer_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_io_buffer_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_set_io_buffer;
	apm_msg.msg.io_buffer_params.pin_type = IS_APM_IN(app->reg) ?
		NVFX_PIN_TYPE_INPUT : NVFX_PIN_TYPE_OUTPUT;
	apm_msg.msg.io_buffer_params.pin_id = 0;
	apm_msg.msg.io_buffer_params.addr = (variant_t)addr;
	apm_msg.msg.io_buffer_params.size = size;

	return tegra210_adsp_send_msg(app->apm, &apm_msg, flags);
}

static int tegra210_adsp_send_period_size_msg(struct tegra210_adsp_app *app,
					size_t size, uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size =
		MSGQ_MSG_SIZE(apm_notification_params_t);
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

	apm_msg.msgq_msg.size = MSGQ_MSG_SIZE(apm_fx_set_param_params_t);
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

	apm_msg.msgq_msg.size = MSGQ_MSG_SIZE(nvfx_set_state_params_t);
	apm_msg.msg.call_params.size = sizeof(nvfx_set_state_params_t);
	apm_msg.msg.call_params.method = nvfx_method_set_state;
	apm_msg.msg.state_params.state = state;

	return tegra210_adsp_send_msg(app->apm, &apm_msg, flags);
}

static int tegra210_adsp_send_reset_msg(struct tegra210_adsp_app *app,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_SIZE(nvfx_reset_params_t);
	apm_msg.msg.call_params.size = sizeof(nvfx_reset_params_t);
	apm_msg.msg.call_params.method = nvfx_method_reset;

	return tegra210_adsp_send_msg(app->apm, &apm_msg, flags);
}

static int tegra210_adsp_send_eos_msg(struct tegra210_adsp_app *app,
					uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_SIZE(apm_eos_params_t);
	apm_msg.msg.call_params.size = sizeof(apm_eos_params_t);
	apm_msg.msg.call_params.method = nvfx_apm_method_set_eos;

	return tegra210_adsp_send_msg(app->apm, &apm_msg, flags);
}

static int tegra210_adsp_send_pos_msg(struct tegra210_adsp_app *app,
					uint32_t pos, uint32_t flags)
{
	apm_msg_t apm_msg;

	apm_msg.msgq_msg.size = MSGQ_MSG_SIZE(apm_position_params_t);
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
		nvadsp_mbox_close(&app->rx_mbox);
		nvadsp_app_stop(app->info);
		app->info = NULL;
		app->plugin = NULL;
		app->apm = NULL;

		if (IS_APM_IN(app->reg)) {
			uint32_t apm_out_reg = APM_OUT_START +
					(app->reg - APM_IN_START);
			struct tegra210_adsp_app *apm_out =
					&adsp->apps[apm_out_reg];

			apm_out->info = NULL;
			apm_out->plugin = NULL;
			apm_out->apm = NULL;
		}
	}
}

/* Recursive function to connect plugins under a APM */
static int tegra210_adsp_connect_plugin(struct tegra210_adsp *adsp,
					struct tegra210_adsp_app *app)
{
	struct tegra210_adsp_app *src;
	uint32_t source;
	int ret = 0;

	source = tegra210_adsp_get_source(adsp, app->reg);
	if (!IS_ADSP_APP(source))
		return -ENODEV;

	src = &adsp->apps[source];
	if (!IS_APM_IN(src->reg)) {
		ret = tegra210_adsp_connect_plugin(adsp, src);
		if (ret < 0)
			return ret;
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

/* API to connect two APMs */
static int tegra210_adsp_connect_apm(struct tegra210_adsp *adsp,
				struct tegra210_adsp_app *app)
{
	struct tegra210_adsp_app *src;
	uint32_t source;
	int ret = 0;

	source = tegra210_adsp_get_source(adsp, app->reg);
	if (!IS_ADSP_APP(source))
		return -EPIPE;

	src = &adsp->apps[source];

	if (IS_APM_OUT(src->reg)) {
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
	return -EPIPE;
}

/* Iterate over all APMs and establish pending connections */
static int tegra210_adsp_update_connection(struct tegra210_adsp *adsp)
{
	int i;

	for (i = APM_OUT_START; i <= APM_OUT_END; i++)
		tegra210_adsp_connect_plugin(adsp, &adsp->apps[i]);

	for (i = APM_IN_START; i <= APM_IN_END; i++)
		tegra210_adsp_connect_apm(adsp, &adsp->apps[i]);

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
		tegra210_adsp_send_state_msg(prtd->fe_apm,
			nvfx_state_inactive, 0);
		tegra210_adsp_send_reset_msg(prtd->fe_apm, 0);
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

	if (prtd) {
		prtd->fe_apm->fe = 0;
		devm_kfree(prtd->dev, prtd);
	}

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
			nvfx_state_active, 0);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state start.");
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
				nvfx_state_active, 0);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state resume");
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
			nvfx_state_inactive, 0);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state stop");
			return ret;
		}

		ret = tegra210_adsp_send_reset_msg(prtd->fe_apm, 0);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to reset");
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
			nvfx_state_inactive, 0);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state pause");
			return ret;
		}
		break;
	case SND_COMPR_TRIGGER_DRAIN:
		prtd->is_draining = 1;
		ret = tegra210_adsp_send_eos_msg(prtd->fe_apm, 0);
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
	    (runtime->total_bytes_available + count) % runtime->buffer_size, 0);

	return count;
}

static int tegra210_adsp_compr_pointer(struct snd_compr_stream *cstream,
			struct snd_compr_tstamp *tstamp)
{
	struct tegra210_adsp_compr_rtd *prtd = cstream->runtime->private_data;
	struct tegra210_adsp_app *app = prtd->fe_apm;
	nvfx_shared_state_t *shared = &app->apm->nvfx_shared_state;

	tstamp->byte_offset = shared->input[0].bytes %
		cstream->runtime->buffer_size;
	tstamp->copied_total = shared->input[0].bytes;
	tstamp->pcm_frames = shared->output[0].bytes / 4;
	/* TODO : calculate IO frames correctly */
	tstamp->pcm_io_frames = shared->output[0].bytes / 4;
	tstamp->sampling_rate = prtd->codec.sample_rate;

	/* TODO : If SRC in path do size conversion */

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
			nvfx_state_active, 0);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state");
			return ret;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		ret = tegra210_adsp_send_state_msg(prtd->fe_apm,
			nvfx_state_inactive, 0);
		if (ret < 0) {
			dev_err(prtd->dev, "Failed to set state");
			return ret;
		}

		ret = tegra210_adsp_send_reset_msg(prtd->fe_apm, 0);
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
	ret = tegra210_adsp_send_pos_msg(prtd->fe_apm, pos, 0);
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

static int tegra210_adsp_pcm_probe(struct snd_soc_platform *platform)
{
	platform->dapm.idle_bias_off = 1;
	return 0;
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

	adma_params.adma_channel = find_first_zero_bit(adsp->adma_usage,
					TEGRA210_ADSP_ADMA_CHANNEL_COUNT);
	if (adma_params.adma_channel >= TEGRA210_ADSP_ADMA_CHANNEL_COUNT) {
		dev_err(adsp->dev, "All ADMA channels are busy");
		return -EBUSY;
	}
	__set_bit(adma_params.adma_channel, adsp->adma_usage);

	adma_params.adma_channel += TEGRA210_ADSP_ADMA_CHANNEL_START;
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

		app->adma_chan = adma_params.adma_channel;
		adma_params.direction = ADMA_MEMORY_TO_AHUB;
		adma_params.event.pvoid = app->apm->output_event.pvoid;

		ret = tegra210_adsp_adma_params_msg(app, &adma_params, 0);
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
			app->adma_chan = adma_params.adma_channel;
			adma_params.direction = ADMA_AHUB_TO_MEMORY;
			adma_params.event.pvoid = app->apm->input_event.pvoid;

			ret = tegra210_adsp_adma_params_msg(app,
					&adma_params, 0);
			if (ret < 0) {
				dev_err(adsp->dev, "ADMA params msg failed");
				return ret;
			}
		}
	}
	return 0;
}

/* ADSP platform driver read/write call-back */
unsigned int tegra210_adsp_read(struct snd_soc_platform *platform,
		unsigned int reg)
{
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);

	dev_vdbg(adsp->dev, "%s [0x%x] -> 0x%x\n", __func__, reg,
		tegra210_adsp_reg_read(adsp, reg));

	return tegra210_adsp_reg_read(adsp, reg);
}

int tegra210_adsp_write(struct snd_soc_platform *platform, unsigned int reg,
		unsigned int val)
{
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);

	dev_vdbg(adsp->dev, "%s [0x%x] -> 0x%x\n", __func__, reg, val);

	tegra210_adsp_reg_write(adsp, reg, val);
	return 0;
}

/* DAPM ENUM MUX get/put callbacks */
static int tegra210_adsp_mux_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *widget = wlist->widgets[0];
	struct snd_soc_platform *platform = widget->platform;
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
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *w = wlist->widgets[0];
	struct snd_soc_platform *platform = w->platform;
	uint32_t val = ucontrol->value.enumerated.item[0];
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);
	struct tegra210_adsp_app *app;
	uint32_t cur_val = 0;
	int ret = 0;

	/* Init or de-init app based on connection */
	if (IS_ADSP_APP(e->reg)) {
		app = &adsp->apps[e->reg];
		cur_val = tegra210_adsp_get_source(adsp, e->reg);
		if (cur_val != val)
			app->connect = 0;

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

	snd_soc_dapm_mux_update_power(w, kcontrol, val, e);
	return 1;
}

/* ALSA control get/put call-back implementation */
static int tegra210_adsp_init_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_platform *platform = snd_kcontrol_chip(kcontrol);
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);

	ucontrol->value.enumerated.item[0] = adsp->init_done;
	return 0;
}

static int tegra210_adsp_init_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_platform *platform = snd_kcontrol_chip(kcontrol);
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);
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
	struct snd_soc_platform *platform = w->platform;
	struct tegra210_adsp *adsp = snd_soc_platform_get_drvdata(platform);
	struct tegra210_adsp_app *app;

	if (!IS_ADSP_APP(w->reg))
		return 0;

	app = &adsp->apps[w->reg];
	/* For FE apm state change will be handled from trigger call back */
	if (app->fe)
		return 0;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		if (IS_APM_IN(w->reg))
			tegra210_adsp_send_state_msg(app, nvfx_state_active, 0);
	} else {
		if (IS_APM_IN(w->reg)) {
			tegra210_adsp_send_state_msg(app, nvfx_state_inactive,
				TEGRA210_ADSP_MSG_FLAG_HOLD);
			tegra210_adsp_send_reset_msg(app, 0);
		} else if (IS_ADMA(w->reg)) {
			__clear_bit(app->adma_chan -
				TEGRA210_ADSP_ADMA_CHANNEL_START,
				adsp->adma_usage);
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
	"MP3-DEC1",
	"MP3-DEC2",
	"AAC-DEC1",
	"AAC-DEC2",
	"SPKPROT-SW",
};

#define ADSP_MUX_ENUM_CTRL_DECL(ename, reg)				\
	SOC_ENUM_SINGLE_DECL(ename##_enum, reg,				\
		TEGRA210_ADSP_WIDGET_SOURCE_SHIFT, tegra210_adsp_mux_texts); \
	const struct snd_kcontrol_new ename##_ctrl =			\
		SOC_DAPM_ENUM_EXT("ADSP Route", ename##_enum,		\
			tegra210_adsp_mux_get, tegra210_adsp_mux_put)

ADSP_MUX_ENUM_CTRL_DECL(adsp_fe1, TEGRA210_ADSP_FRONT_END1);
ADSP_MUX_ENUM_CTRL_DECL(adsp_fe2, TEGRA210_ADSP_FRONT_END2);
ADSP_MUX_ENUM_CTRL_DECL(adsp_fe3, TEGRA210_ADSP_FRONT_END3);
ADSP_MUX_ENUM_CTRL_DECL(adsp_fe4, TEGRA210_ADSP_FRONT_END4);
ADSP_MUX_ENUM_CTRL_DECL(adsp_fe5, TEGRA210_ADSP_FRONT_END5);
ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif1, TEGRA210_ADSP_ADMAIF1);
ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif2, TEGRA210_ADSP_ADMAIF2);
ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif3, TEGRA210_ADSP_ADMAIF3);
ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif4, TEGRA210_ADSP_ADMAIF4);
ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif5, TEGRA210_ADSP_ADMAIF5);
ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif6, TEGRA210_ADSP_ADMAIF6);
ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif7, TEGRA210_ADSP_ADMAIF7);
ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif8, TEGRA210_ADSP_ADMAIF8);
ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif9, TEGRA210_ADSP_ADMAIF9);
ADSP_MUX_ENUM_CTRL_DECL(adsp_admaif10, TEGRA210_ADSP_ADMAIF10);
ADSP_MUX_ENUM_CTRL_DECL(apm_in1, TEGRA210_ADSP_APM_IN1);
ADSP_MUX_ENUM_CTRL_DECL(apm_in2, TEGRA210_ADSP_APM_IN2);
ADSP_MUX_ENUM_CTRL_DECL(apm_in3, TEGRA210_ADSP_APM_IN3);
ADSP_MUX_ENUM_CTRL_DECL(apm_in4, TEGRA210_ADSP_APM_IN4);
ADSP_MUX_ENUM_CTRL_DECL(apm_in5, TEGRA210_ADSP_APM_IN5);
ADSP_MUX_ENUM_CTRL_DECL(apm_in6, TEGRA210_ADSP_APM_IN6);
ADSP_MUX_ENUM_CTRL_DECL(apm_in7, TEGRA210_ADSP_APM_IN7);
ADSP_MUX_ENUM_CTRL_DECL(apm_in8, TEGRA210_ADSP_APM_IN8);
ADSP_MUX_ENUM_CTRL_DECL(apm_out1, TEGRA210_ADSP_APM_OUT1);
ADSP_MUX_ENUM_CTRL_DECL(apm_out2, TEGRA210_ADSP_APM_OUT2);
ADSP_MUX_ENUM_CTRL_DECL(apm_out3, TEGRA210_ADSP_APM_OUT3);
ADSP_MUX_ENUM_CTRL_DECL(apm_out4, TEGRA210_ADSP_APM_OUT4);
ADSP_MUX_ENUM_CTRL_DECL(apm_out5, TEGRA210_ADSP_APM_OUT5);
ADSP_MUX_ENUM_CTRL_DECL(apm_out6, TEGRA210_ADSP_APM_OUT6);
ADSP_MUX_ENUM_CTRL_DECL(apm_out7, TEGRA210_ADSP_APM_OUT7);
ADSP_MUX_ENUM_CTRL_DECL(apm_out8, TEGRA210_ADSP_APM_OUT8);
ADSP_MUX_ENUM_CTRL_DECL(adma1, TEGRA210_ADSP_PLUGIN_ADMA1);
ADSP_MUX_ENUM_CTRL_DECL(adma2, TEGRA210_ADSP_PLUGIN_ADMA2);
ADSP_MUX_ENUM_CTRL_DECL(adma3, TEGRA210_ADSP_PLUGIN_ADMA3);
ADSP_MUX_ENUM_CTRL_DECL(adma4, TEGRA210_ADSP_PLUGIN_ADMA4);
ADSP_MUX_ENUM_CTRL_DECL(mp3_dec1, TEGRA210_ADSP_PLUGIN_MP3_DEC1);
ADSP_MUX_ENUM_CTRL_DECL(mp3_dec2, TEGRA210_ADSP_PLUGIN_MP3_DEC2);
ADSP_MUX_ENUM_CTRL_DECL(aac_dec1, TEGRA210_ADSP_PLUGIN_AAC_DEC1);
ADSP_MUX_ENUM_CTRL_DECL(aac_dec2, TEGRA210_ADSP_PLUGIN_AAC_DEC2);
ADSP_MUX_ENUM_CTRL_DECL(spkprot, TEGRA210_ADSP_PLUGIN_SPKPROT);

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
	ADSP_WIDGETS("MP3-DEC1", mp3_dec1, TEGRA210_ADSP_PLUGIN_MP3_DEC1),
	ADSP_WIDGETS("MP3-DEC2", mp3_dec2, TEGRA210_ADSP_PLUGIN_MP3_DEC2),
	ADSP_WIDGETS("AAC-DEC1", aac_dec1, TEGRA210_ADSP_PLUGIN_AAC_DEC1),
	ADSP_WIDGETS("AAC-DEC2", aac_dec2, TEGRA210_ADSP_PLUGIN_AAC_DEC2),
	ADSP_WIDGETS("SPKPROT-SW", spkprot, TEGRA210_ADSP_PLUGIN_SPKPROT),
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

#define ADSP_DEC_ROUTES(name)					\
	{ name " MUX",	"MP3-DEC1",	"MP3-DEC1 TX"},		\
	{ name " MUX",	"MP3-DEC2",	"MP3-DEC2 TX"},		\
	{ name " MUX",	"AAC-DEC1",	"AAC-DEC1 TX"},		\
	{ name " MUX",	"AAC-DEC2",	"AAC-DEC2 TX"}

#define ADSP_SPKPROT_ROUTES(name)				\
	{ name " MUX",	"SPKPROT-SW",	"SPKPROT-SW TX"}

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
	ADSP_DEC_ROUTES(name)

#define ADSP_DEC_MUX_ROUTES(name)				\
	{ name " TX",		NULL, name " MUX"},		\
	ADSP_APM_IN_ROUTES(name)

#define ADSP_ADMA_MUX_ROUTES(name)				\
	{ name " TX",		NULL, name " MUX"},		\
	ADSP_APM_IN_ROUTES(name),				\
	ADSP_DEC_ROUTES(name),					\
	ADSP_SPKPROT_ROUTES(name)

#define ADSP_SPKPROT_MUX_ROUTES(name)				\
	{ name " TX",		NULL, name " MUX"},		\
	ADSP_ADMA_ROUTES(name)

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

	ADSP_DEC_MUX_ROUTES("MP3-DEC1"),
	ADSP_DEC_MUX_ROUTES("MP3-DEC2"),
	ADSP_DEC_MUX_ROUTES("AAC-DEC1"),
	ADSP_DEC_MUX_ROUTES("AAC-DEC2"),

	ADSP_SPKPROT_MUX_ROUTES("SPKPROT-SW"),
};

static const struct snd_kcontrol_new tegra210_adsp_controls[] = {
	SOC_SINGLE_BOOL_EXT("ADSP init", 0,
		tegra210_adsp_init_get, tegra210_adsp_init_put),
};

static const struct snd_soc_component_driver tegra210_adsp_component = {
	.name		= "tegra210-adsp",
};

static int tegra210_adsp_codec_probe(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_codec_driver tegra210_adsp_codec = {
	.probe = tegra210_adsp_codec_probe,
};

static struct snd_soc_platform_driver tegra210_adsp_platform = {
	.ops			= &tegra210_adsp_pcm_ops,
	.compr_ops		= &tegra210_adsp_compr_ops,
	.pcm_new		= tegra210_adsp_pcm_new,
	.pcm_free		= tegra210_adsp_pcm_free,
	.probe			= tegra210_adsp_pcm_probe,
	.read			= tegra210_adsp_read,
	.write			= tegra210_adsp_write,
	.dapm_widgets		= tegra210_adsp_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(tegra210_adsp_widgets),
	.dapm_routes		= tegra210_adsp_routes,
	.num_dapm_routes	= ARRAY_SIZE(tegra210_adsp_routes),
	.controls		= tegra210_adsp_controls,
	.num_controls		= ARRAY_SIZE(tegra210_adsp_controls),
};

static u64 tegra_dma_mask = DMA_BIT_MASK(32);
static int tegra210_adsp_audio_platform_probe(struct platform_device *pdev)
{
	struct tegra210_adsp *adsp;
	int i, j, ret = 0;

	pr_info("tegra210_adsp_audio_platform_probe: platform probe started\n");

	adsp = devm_kzalloc(&pdev->dev, sizeof(*adsp), GFP_KERNEL);
	if (!adsp) {
		dev_err(&pdev->dev, "Can't allocate tegra30_adsp_ctx\n");
		return -ENOMEM;
	}
	dev_set_drvdata(&pdev->dev, adsp);
	adsp->dev = &pdev->dev;

	mutex_init(&adsp->mutex);
	pdev->dev.dma_mask = &tegra_dma_mask;
	pdev->dev.coherent_dma_mask = tegra_dma_mask;

	/* HACK : Should be handled through dma-engine */
	for (i = 0; i < TEGRA210_ADSP_ADMA_CHANNEL_COUNT; i++) {
		ret = tegra_agic_route_interrupt(
			INT_ADMA_EOT0 + TEGRA210_ADSP_ADMA_CHANNEL_START + i,
			TEGRA_AGIC_ADSP);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to route INT to ADSP");
			return ret;
		}
	}
	/* HACK end */

	INIT_WORK(&adsp->work, tegra210_adsp_init_work);

	for (i = 0; i < TEGRA210_ADSP_VIRT_REG_MAX; i++)
		adsp->apps[i].reg = i;

	for (i = 0; i < ARRAY_SIZE(adsp_app_desc); i++) {
		for (j = adsp_app_desc[i].reg_start;
			j <= adsp_app_desc[i].reg_end; j++)
			adsp->apps[j].desc = &adsp_app_desc[i];
	}

	ret = snd_soc_register_platform(&pdev->dev, &tegra210_adsp_platform);
	if (ret) {
		dev_err(&pdev->dev, "Could not register platform: %d\n", ret);
		goto err;
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

#if ENABLE_ADSP
	schedule_work(&adsp->work);
#endif

	pr_info("tegra210_adsp_audio_platform_probe probe successfull.");
	return 0;

err_unregister_platform:
	snd_soc_unregister_platform(&pdev->dev);
err:
	return ret;
}

static int tegra210_adsp_audio_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static const struct of_device_id tegra210_adsp_audio_of_match[] = {
	{ .compatible = "nvidia,tegra210-adsp-audio", },
	{},
};

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

