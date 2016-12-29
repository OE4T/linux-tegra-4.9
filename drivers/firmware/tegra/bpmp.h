/*
 * Copyright (c) 2013-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _DRIVERS_BPMP_H
#define _DRIVERS_BPMP_H

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/tegra_bpmp.h>

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#define NR_CHANNELS		14
#define NR_THREAD_CH		7
#else
#define NR_CHANNELS		12
#define NR_THREAD_CH		4
#endif

#define DO_ACK			(1 << 0)
#define RING_DOORBELL		(1 << 1)

struct fops_entry {
	char *name;
	const struct file_operations *fops;
	mode_t mode;
};

struct mb_data {
	int32_t code;
	int32_t flags;
	u8 data[MSG_DATA_MIN_SZ];
} __packed;

struct channel_data {
	struct mb_data *ib;
	struct mb_data *ob;
};

extern struct channel_data channel_area[NR_CHANNELS];
extern char firmware_tag[32];

struct dentry *bpmp_init_debug(struct platform_device *pdev);
int bpmp_init_cpuidle_debug(struct dentry *root);

extern int connected;

int bpmp_mail_init_prepare(void);
int bpmp_mail_init(struct device_node *of_node);
int __bpmp_do_ping(void);
int bpmp_create_attrs(const struct fops_entry *fent, struct dentry *parent,
		void *data);
int bpmp_mailman_init(void);
void bpmp_handle_mail(int mrq, int ch);

void bpmp_ring_doorbell(int ch);
int bpmp_thread_ch_index(int ch);
int bpmp_ob_channel(void);
int bpmp_thread_ch(int idx);
int bpmp_init_irq(void);
int bpmp_connect(struct device_node *of_node);
void tegra_bpmp_resume(void);
void bpmp_handle_irq(int ch);

bool bpmp_master_free(int ch);
bool bpmp_slave_signalled(int ch);
bool bpmp_master_acked(int ch);
void bpmp_signal_slave(int ch);
void bpmp_free_master(int ch);

#if IS_ENABLED(CONFIG_POWERGATE_TEGRA_BPMP)
int tegra_bpmp_init_powergate(struct platform_device *pdev);
#else
static inline int tegra_bpmp_init_powergate(struct tegra_bpmp *bpmp)
{
	return 0;
}
#endif

#endif
