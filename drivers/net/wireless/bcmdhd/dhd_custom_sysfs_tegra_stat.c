/*
 * drivers/net/wireless/bcmdhd/dhd_custom_sysfs_tegra_stat.c
 *
 * NVIDIA Tegra Sysfs for BCMDHD driver
 *
 * Copyright (C) 2014-2015 NVIDIA Corporation. All rights reserved.
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

#include "dhd_custom_sysfs_tegra.h"
#include "bcmutils.h"
#include "wlioctl.h"
#include "wldev_common.h"

struct net_device *dhd_custom_sysfs_tegra_histogram_stat_netdev;

static void
stat_work_func(struct work_struct *work);

static unsigned int stat_delay_ms;
static unsigned int stat_rate_ms = 10 * 1000;
static DECLARE_DELAYED_WORK(stat_work, stat_work_func);

void
tegra_sysfs_histogram_stat_work_run(unsigned int ms)
{
	stat_delay_ms = ms;
}

void
tegra_sysfs_histogram_stat_work_start(void)
{
//	pr_info("%s\n", __func__);
	if (stat_rate_ms > 0)
		schedule_delayed_work(&stat_work,
			msecs_to_jiffies(stat_rate_ms));
}

void
tegra_sysfs_histogram_stat_work_stop(void)
{
//	pr_info("%s\n", __func__);
	cancel_delayed_work_sync(&stat_work);
}

static void
stat_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct net_device *net = dhd_custom_sysfs_tegra_histogram_stat_netdev;
	char *netif = net ? net->name : "";
	wl_cnt_t *cnt;
	int i;

	UNUSED_PARAMETER(dwork);

//	pr_info("%s\n", __func__);

	/* create stat request */
	cnt = kmalloc(sizeof(wl_cnt_t), GFP_KERNEL);
	if (!cnt) {
//		pr_err("%s: kmalloc(wl_cnt_t) failed\n", __func__);
		goto fail;
	}

	/* send stat request */
	if (wldev_iovar_getbuf(net, "counters", NULL, 0,
		(void *) cnt, sizeof(wl_cnt_t), NULL) != BCME_OK) {
//		pr_err("%s: wldev_iovar_getbuf() failed\n", __func__);
		kfree(cnt);
		goto fail;
	}

	/* log stat request */
	for (i = 0; i < sizeof(wl_cnt_t); i += 64) {
		tcpdump_pkt_save('a' + i / 64,
			netif,
			__func__,
			__LINE__,
			((unsigned char *) cnt) + i,
			(i + 64) <= sizeof(wl_cnt_t)
				? 64 : sizeof(wl_cnt_t) - i,
			0);
	}
	kfree(cnt);

	/* schedule next stat */
fail:
	if (stat_delay_ms) {
		stat_delay_ms = 0;
		msleep(stat_delay_ms);
		schedule_delayed_work(&stat_work, 0);
		return;
	}
	schedule_delayed_work(&stat_work,
		msecs_to_jiffies(stat_rate_ms));

}

ssize_t
tegra_sysfs_histogram_stat_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	static int i;

//	pr_info("%s\n", __func__);

	if (!i) {
		i++;
		strcpy(buf, "dummy stat!");
		return strlen(buf);
	} else {
		i = 0;
		return 0;
	}
}

ssize_t
tegra_sysfs_histogram_stat_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int err;
	unsigned int uint;

//	pr_info("%s\n", __func__);

	if (strncmp(buf, "enable", 6) == 0) {
		pr_info("%s: starting stat delayed work...\n", __func__);
		tegra_sysfs_histogram_stat_work_start();
	} else if (strncmp(buf, "disable", 7) == 0) {
		pr_info("%s: stopping stat delayed work...\n", __func__);
		tegra_sysfs_histogram_stat_work_stop();
	} else if (strncmp(buf, "rate ", 5) == 0) {
		err = kstrtouint(buf + 5, 0, &uint);
		if (err < 0) {
			pr_err("%s: invalid stat rate (ms)\n", __func__);
			return count;
		}
		pr_info("%s: set stat rate (ms) %u\n", __func__, uint);
		stat_rate_ms = uint;
	} else {
		pr_err("%s: unknown command\n", __func__);
	}

	return count;
}
