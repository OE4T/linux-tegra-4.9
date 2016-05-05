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
#include <linux/debugfs.h>
#include <linux/platform_device.h>

#include <linux/tegra_nvadsp.h>

#include "dev.h"

#define RW_MODE (S_IWUSR | S_IRUGO)

/* #define VERBOSE_OUTPUT_LPTHREAD */

struct adsp_lpthread_shared_state_t {
	uint16_t mbox_id;
};

enum adsp_lpthread_mbx_cmd {
	adsp_lpthread_cmd_resume = 0,
	adsp_lpthread_cmd_pause,
	adsp_lpthread_cmd_close,
};

struct adsp_lpthread {
	bool lpthread_initialized;
	bool adsp_os_suspended;
	bool lpthread_to_be_paused;
	bool lpthread_to_be_resumed;
	bool lpthread_to_be_closed;
	bool lpthread_init_and_closed;
};

static struct adsp_lpthread lpthread_obj;
static struct adsp_lpthread *lpthread;

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

static int adsp_usage_set(void *data, u64 val)
{
	int ret = 0;
	switch (val) {
	case 1:
		if (lpthread->adsp_os_suspended && !lpthread->lpthread_initialized) {
			if (lpthread->lpthread_init_and_closed)  {
				lpthread->lpthread_init_and_closed = false;
				lpthread->lpthread_initialized = true;
				lpthread->lpthread_to_be_resumed = true;
				pr_info("App already initialized\n");
				break;
			}
#ifdef VERBOSE_OUTPUT_LPTHREAD
			pr_info("Starting os\n");
#endif
			if (nvadsp_os_start()) {
				pr_info("Unable to restart os\n");
				break;
			}
			ret = adsp_lpthread_init(true);
			lpthread->lpthread_initialized = true;
#ifdef VERBOSE_OUTPUT_LPTHREAD
			pr_info("Initialized lpthread. Suspending OS\n");
#endif
			if (nvadsp_os_suspend()) {
				pr_info("Unable to restart os\n");
				break;
			}
			lpthread->lpthread_to_be_resumed = true;
		} else if (!lpthread->lpthread_initialized) {
			if (lpthread->lpthread_init_and_closed)  {
				lpthread->lpthread_init_and_closed = false;
				lpthread->lpthread_initialized = true;
				lpthread->lpthread_to_be_resumed = true;
				pr_info("App already initialized\n");
				break;
			}
			ret = adsp_lpthread_init(lpthread->adsp_os_suspended);
			lpthread->lpthread_initialized = true;
		} else if (lpthread->adsp_os_suspended) {
#ifdef VERBOSE_OUTPUT_LPTHREAD
			pr_info("App to be resumed\n");
#endif
			lpthread->lpthread_to_be_resumed = true;
			lpthread->lpthread_to_be_paused = false;
			ret = 0;
		} else {
			ret = adsp_lpthread_resume();
		}
		break;
	case 2:
		if (lpthread->adsp_os_suspended && lpthread->lpthread_initialized) {
#ifdef VERBOSE_OUTPUT_LPTHREAD
			pr_info("App to be paused\n");
#endif
			lpthread->lpthread_to_be_resumed = false;
			lpthread->lpthread_to_be_paused = true;
			ret = 0;
		} else if (lpthread->lpthread_initialized) {
			ret = adsp_lpthread_pause();
		} else {
			pr_info("App not initialized. echo 1 > adsp_usage to init\n");
			ret = 0;
		}
		break;
	case 0:
			if (lpthread->adsp_os_suspended && lpthread->lpthread_initialized) {
#ifdef VERBOSE_OUTPUT_LPTHREAD
			pr_info("App to be closed\n");
#endif
			lpthread->lpthread_to_be_closed = true;
			lpthread->lpthread_init_and_closed = true;
			lpthread->lpthread_initialized = false;
			lpthread->lpthread_to_be_resumed = false;
			ret = 0;
		} else if (lpthread->lpthread_initialized) {
			ret = adsp_lpthread_exit();
			lpthread->lpthread_initialized = false;
		} else {
			pr_info("App not initialized. echo 1 > adsp_usage to init\n");
			ret = 0;
		}
		break;
	default:
		pr_info("Invalid input\n");
		pr_info("echo 1 > adsp_usage to init/resume\n");
		pr_info("echo 2 > adsp_usage to pause\n");
		pr_info("echo 0 > adsp_usage to stop\n");
		ret = 0;
	}
	return ret;
}

static int adsp_usage_get(void *data, u64 *val)
{
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(adsp_usage_fops, adsp_usage_get, adsp_usage_set, "%llu\n");

static int lpthread_debugfs_init(struct nvadsp_drv_data *drv)
{
	int ret = -ENOMEM;
	struct dentry *d, *dir;

	if (!drv->adsp_debugfs_root)
		return ret;
	dir = debugfs_create_dir("adsp_lpthread", drv->adsp_debugfs_root);
	if (!dir)
		return ret;

	d = debugfs_create_file(
			"adsp_usage", RW_MODE, dir, NULL, &adsp_usage_fops);
	if (!d)
		return ret;

	return 0;
}

int adsp_lpthread_debugfs_init(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv = platform_get_drvdata(pdev);
	int ret = -EINVAL;

	lpthread = &lpthread_obj;

	ret = lpthread_debugfs_init(drv);
	if (!ret)
		pr_info(" lpthread_debugfs_init() ret = %d\n", ret);

	drv->lpthread_initialized = true;
	lpthread->adsp_os_suspended = false;

	return 0;
}

int adsp_lpthread_debugfs_exit(struct platform_device *pdev)
{
	status_t ret = 0;
	struct nvadsp_drv_data *drv = platform_get_drvdata(pdev);

	if (!drv->lpthread_initialized)
		ret =  -EINVAL;
	drv->lpthread_initialized = false;

	return ret;
}

int adsp_lpthread_debugfs_set_suspend(bool is_suspended)
{
	lpthread->adsp_os_suspended = is_suspended;
#ifdef VERBOSE_OUTPUT_LPTHREAD
	pr_info("ADSP_OS_SUSPENDED = %d\n", (int)is_suspended);
#endif
	return 0;
}

int adsp_lpthread_debugfs_callback(void)
{
	int ret = 0;

	if (lpthread->lpthread_initialized) {
		if (lpthread->lpthread_to_be_resumed) {
			lpthread->lpthread_to_be_resumed = false;
			ret = adsp_lpthread_resume();
		} else if (lpthread->lpthread_to_be_paused) {
			lpthread->lpthread_to_be_paused = false;
			ret = adsp_lpthread_pause();
		}
		return ret;
	}

	if (lpthread->lpthread_to_be_closed) {
		lpthread->lpthread_to_be_closed = false;
		lpthread->lpthread_init_and_closed = false;
		ret = adsp_lpthread_exit();
		return ret;
	}

#ifdef VERBOSE_OUTPUT_LPTHREAD
	pr_info("to_be_paused = %d\n", (int)lpthread->lpthread_to_be_paused);
	pr_info("to_be_resumed = %d\n", (int)lpthread->lpthread_to_be_resumed);
	pr_info("to_be_closed = %d\n", (int)lpthread->lpthread_to_be_closed);
	pr_info("init_and_closed = %d\n", (int)lpthread->lpthread_init_and_closed);
	pr_info("initialized = %d\n", (int)lpthread->lpthread_initialized);
#endif

	return ret;
}
