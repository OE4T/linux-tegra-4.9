/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/tegra-ivc.h>
#include <soc/tegra/chip-id.h>
#include <linux/tegra-cpufreq.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/platform/tegra/tegra18_cpu_map.h>

struct cpu_rate_msg {
	uint32_t rate_khz;
	uint8_t cl;
};

union msg_data {
	struct cpu_rate_msg cpu_rate;
};

#define TEGRA_CPUFREQ_IVC_MSG_LEN sizeof(union msg_data)

struct tegra_cpufreq_ivc_msg {
	uint32_t msg_id;
	uint32_t len;
	union msg_data buffer;
};

struct tegra_cpufreq_ivc_data {
	struct mutex mlock;
	struct tegra_cpufreq_ivc_msg cpufreq_ivc_msg;
	wait_queue_head_t wq;
	struct tegra_hv_ivc_cookie *cookie;
};

static struct tegra_cpufreq_ivc_data ivc;

static irqreturn_t hv_tegra_cpufreq_ivc_isr(int irq, void *dev_id)
{
	struct tegra_cpufreq_ivc_data *ivck =
			(struct tegra_cpufreq_ivc_data *)dev_id;

	if (tegra_hv_ivc_can_write(ivck->cookie))
		wake_up(&ivck->wq);

	return IRQ_HANDLED;
}

int parse_hv_dt_data(struct device_node *dn)
{
	int err = 0;
	uint32_t ivc_queue;
	struct device_node *hv_dn;
	struct tegra_cpufreq_ivc_data *ivck = &ivc;

	hv_dn = of_parse_phandle(dn, "ivc_queue", 0);
	if (hv_dn == NULL) {
		pr_err("Failed to parse phandle of ivc prop\n");
		err = -EINVAL;
		goto err_out;
	}

	err = of_property_read_u32_index(dn, "ivc_queue", 1,
					&ivc_queue);

	if (err != 0) {
		pr_err("Failed to read IVC property ID\n");
		err = -EINVAL;
		goto err_out_free;
	}

	init_waitqueue_head(&ivck->wq);
	mutex_init(&ivck->mlock);
	ivck->cookie = tegra_hv_ivc_reserve(hv_dn, ivc_queue, NULL);

	if (IS_ERR_OR_NULL(ivck->cookie)) {
		pr_err("Failed to reserve ivc queue %d\n",
				ivc_queue);
		err = -EINVAL;
		goto err_out_free;
	}

	err = request_threaded_irq(ivck->cookie->irq,
				hv_tegra_cpufreq_ivc_isr,
				NULL,
				0, "hv-tegra-cpufreq", ivck);
	if (err) {
		tegra_hv_ivc_unreserve(ivck->cookie);
		err = -ENOMEM;
		goto err_out_free;
	}
	/* set ivc channel to invalid state */
	tegra_hv_ivc_channel_reset(ivck->cookie);

err_out:
	return err;

err_out_free:
	of_node_put(hv_dn);
	return err;
}

static bool tegra_cpufreq_ivc_can_write(struct tegra_hv_ivc_cookie *cookie)
{
	if (tegra_hv_ivc_can_write(cookie))
		return true;
	else {
		pr_info("\nIVC Queue is Full\n");
		return false;
	}
}

int tegra_cpufreq_tx_ivc_msg(uint32_t id, uint32_t len, void *msg_buf)
{
	struct tegra_cpufreq_ivc_data *ivck = &ivc;
	struct tegra_cpufreq_ivc_msg *ivc_msg = NULL;
	uint32_t size = sizeof(struct tegra_cpufreq_ivc_msg);
	int ret = 0;

	if ((len > TEGRA_CPUFREQ_IVC_MSG_LEN) || (id > MAX_IVC_MSG_ID))
		return -EINVAL;

	mutex_lock(&ivck->mlock);

	ivc_msg = &ivck->cpufreq_ivc_msg;
	memset(&ivc_msg->buffer, 0, TEGRA_CPUFREQ_IVC_MSG_LEN);
	ivc_msg->msg_id = id;
	ivc_msg->len = len;
	memcpy(&ivc_msg->buffer, msg_buf, len);

	while (tegra_hv_ivc_channel_notified(ivck->cookie))
		/* Waiting for the channel to be ready */;

	if (!tegra_cpufreq_ivc_can_write(ivck->cookie))
		wait_event(ivck->wq, tegra_cpufreq_ivc_can_write(ivck->cookie));

	ret = tegra_hv_ivc_write(ivck->cookie,
				(const void *)ivc_msg,
				size);
	if (ret != size) {
		pr_err("\n%s: Write failed %d %d\n", __func__, size, ret);
		ret = -EINVAL;
	} else {
		ret = 0;
	}

	mutex_unlock(&ivck->mlock);

	return ret;
}
EXPORT_SYMBOL(tegra_cpufreq_tx_ivc_msg);

void tegra_update_cpu_speed_hv(uint32_t rate, uint8_t cpu)
{
	int ret = 0;
	struct cpu_rate_msg cpu_rate;

	cpu_rate.cl = tegra18_logical_to_cluster(cpu);
	cpu_rate.rate_khz = rate;

	ret = tegra_cpufreq_tx_ivc_msg(TEGRA_CPU_FREQ_SET_RATE,
					sizeof(cpu_rate),
					&cpu_rate);

	if (ret)
		pr_err("\n%s: Update cpu rate %dkHz for cluster:%d failed\n",
							__func__,
							cpu_rate.rate_khz,
							cpu_rate.cl);
	return;
}
