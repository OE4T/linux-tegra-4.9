/*
 * Tegra Graphics Virtualization Host functions for HOST1X
 *
 * Copyright (c) 2014-2015, NVIDIA Corporation. All rights reserved.
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

#include "vhost.h"

static inline int vhost_comm_init(struct platform_device *pdev)
{
	size_t queue_sizes[] = { TEGRA_VHOST_QUEUE_SIZES };

	return tegra_gr_comm_init(pdev, TEGRA_GR_COMM_CTX_CLIENT, 3,
				queue_sizes, TEGRA_VHOST_QUEUE_CMD,
				ARRAY_SIZE(queue_sizes));
}

static inline void vhost_comm_deinit(void)
{
	size_t queue_sizes[] = { TEGRA_VHOST_QUEUE_SIZES };

	tegra_gr_comm_deinit(TEGRA_GR_COMM_CTX_CLIENT, TEGRA_VHOST_QUEUE_CMD,
			ARRAY_SIZE(queue_sizes));
}

int vhost_virt_moduleid(int moduleid)
{
	switch (moduleid) {
	case NVHOST_MODULE_NONE:
		return TEGRA_VHOST_MODULE_HOST;
	case NVHOST_MODULE_ISP:
		return TEGRA_VHOST_MODULE_ISP;
	case (1 << 16) | NVHOST_MODULE_ISP:
		return (1 << 16) | TEGRA_VHOST_MODULE_ISP;
	case NVHOST_MODULE_VI:
		return TEGRA_VHOST_MODULE_VI;
	case (1 << 16) | NVHOST_MODULE_VI:
		return (1 << 16) | TEGRA_VHOST_MODULE_VI;
	case NVHOST_MODULE_MSENC:
		return TEGRA_VHOST_MODULE_MSENC;
	case NVHOST_MODULE_VIC:
		return TEGRA_VHOST_MODULE_VIC;
	default:
		pr_err("module %d not virtualized\n", moduleid);
		return -1;
	}
}

static u64 vhost_virt_connect(int moduleid)
{
	struct tegra_vhost_cmd_msg msg;
	struct tegra_vhost_connect_params *p = &msg.params.connect;
	int err;

	msg.cmd = TEGRA_VHOST_CMD_CONNECT;
	p->module = vhost_virt_moduleid(moduleid);
	if (p->module == -1)
		return 0;

	err = vhost_sendrecv(&msg);

	return (err || msg.ret) ? 0 : p->handle;
}

int vhost_sendrecv(struct tegra_vhost_cmd_msg *msg)
{
	void *handle;
	size_t size = sizeof(*msg);
	size_t size_out = size;
	void *data = msg;
	int err;

	err = tegra_gr_comm_sendrecv(TEGRA_GR_COMM_CTX_CLIENT,
				tegra_gr_comm_get_server_vmid(),
				TEGRA_VHOST_QUEUE_CMD, &handle, &data, &size);
	if (!err) {
		WARN_ON(size < size_out);
		memcpy(msg, data, size_out);
		tegra_gr_comm_release(handle);
	}

	return err;
}

int nvhost_virt_init(struct platform_device *dev, int moduleid)
{
	struct nvhost_virt_ctx *virt_ctx =
				kzalloc(sizeof(*virt_ctx), GFP_KERNEL);
	int err;

	if (!virt_ctx)
		return -ENOMEM;

	/* If host1x, init comm */
	if (moduleid == NVHOST_MODULE_NONE) {
		err = vhost_comm_init(dev);
		if (err) {
			dev_err(&dev->dev, "failed to init comm interface\n");
			goto fail;
		}
	}

	virt_ctx->handle = vhost_virt_connect(moduleid);
	if (!virt_ctx->handle) {
		dev_err(&dev->dev,
			"failed to connect to server node\n");
		if (moduleid == NVHOST_MODULE_NONE)
			vhost_comm_deinit();
		err = -ENOMEM;
		goto fail;
	}

	nvhost_set_virt_data(dev, virt_ctx);
	return 0;

fail:
	kfree(virt_ctx);
	return err;
}

void nvhost_virt_deinit(struct platform_device *dev)
{
	struct nvhost_virt_ctx *virt_ctx = nvhost_get_virt_data(dev);

	if (virt_ctx) {
		/* FIXME: add virt disconnect */
		vhost_comm_deinit();
		kfree(virt_ctx);
	}
}
