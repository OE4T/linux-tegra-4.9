/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>

#include <linux/slab.h>
#include <linux/workqueue.h>

#include <linux/tegra_nvadsp.h>

#include "dev.h"

/* #define VERBOSE_OUTPUT_LPTHREAD */

struct adsp_lpthread_shared_state_t {
	uint16_t mbox_id;
};

enum adsp_lpthread_mbx_cmd {
	adsp_lpthread_cmd_resume = 0,
	adsp_lpthread_cmd_pause,
	adsp_lpthread_cmd_close,
};

static struct nvadsp_mbox mbox;
static struct adsp_lpthread_shared_state_t *adsp_lpthread;

int adsp_lpthread_init(bool is_adsp_suspended)
{
	nvadsp_app_handle_t handle;
	nvadsp_app_info_t *app_info;
	int ret;

#ifdef VERBOSE_OUTPUT_LPTHREAD
	pr_info("ADSP_LPTHREAD_INIT(): %d\n", (int)is_adsp_suspended);
#endif

	handle = nvadsp_app_load("adsp_lpthread", "adsp_lpthread.elf");
	if (!handle)
		return -1;

	app_info = nvadsp_app_init(handle, NULL);
	if (!app_info) {
		pr_info("unable to init app adsp_lpthread\n");
		return -2;
	}

	ret = nvadsp_app_start(app_info);
	if (ret) {
		pr_info("unable to start app adsp_lpthread\n");
		return -3;
	}

	adsp_lpthread = (struct adsp_lpthread_shared_state_t *)app_info->mem.shared;
	ret = nvadsp_mbox_open(&mbox, &adsp_lpthread->mbox_id, "adsp_lpthread", NULL, NULL);
	if (ret) {
		pr_info("Failed to open mbox %d", adsp_lpthread->mbox_id);
		return -4;
	}

	/* Start timer is adsp is not in suspended state */
	if (!is_adsp_suspended) {
#ifdef VERBOSE_OUTPUT_LPTHREAD
		pr_info("Attempting resume() from init()\n");
#endif
		ret = adsp_lpthread_resume();
		return ret;
	}

	return 0;
}

int adsp_lpthread_resume(void)
{
	int ret;
#ifdef VERBOSE_OUTPUT_LPTHREAD
	pr_info("ADSP_LPTHREAD_RESUME()\n");
#endif
	ret = nvadsp_mbox_send(&mbox, adsp_lpthread_cmd_resume, NVADSP_MBOX_SMSG, 0, 0);
	if (ret)
		pr_info("nvadsp_mbox_send() in adsp_lpthread_resume() failed: %d, ret = %d\n", adsp_lpthread->mbox_id, ret);
	return ret;
}

int adsp_lpthread_pause(void)
{
	int ret;
#ifdef VERBOSE_OUTPUT_LPTHREAD
	pr_info("ADSP_LPTHREAD_PAUSE()\n");
#endif
	ret = nvadsp_mbox_send(&mbox, adsp_lpthread_cmd_pause, NVADSP_MBOX_SMSG, 0, 0);
	if (ret)
		pr_info("nvadsp_mbox_send() in adsp_lpthread_pause() failed: %d, ret = %d\n", adsp_lpthread->mbox_id, ret);
	return ret;
}

int adsp_lpthread_exit(void)
{
	int ret;
#ifdef VERBOSE_OUTPUT_LPTHREAD
	pr_info("ADSP_LPTHREAD_EXIT()\n");
#endif
	ret = nvadsp_mbox_send(&mbox, adsp_lpthread_cmd_close, NVADSP_MBOX_SMSG, 0, 0);
	if (ret)
		pr_info("nvadsp_mbox_send() in adsp_lpthread_exit() failed: %d, ret = %d\n", adsp_lpthread->mbox_id, ret);
	nvadsp_mbox_close(&mbox);
	return ret;
}
