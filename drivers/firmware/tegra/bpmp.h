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

struct mail_ops {
	int (*init_prepare)(void);
	int (*init_irq)(void);
	int (*connect)(const struct mail_ops *ops, struct device_node *of_node);
	void (*resume)(void);

	struct ivc *(*ivc_obj)(int ch);
	int (*ob_channel)(void);
	int (*thread_ch)(int idx);
	int (*thread_ch_index)(int ch);

	bool (*master_free)(const struct mail_ops *ops, int ch);
	void (*free_master)(const struct mail_ops *ops, int ch);
	bool (*master_acked)(const struct mail_ops *ops, int ch);
	void (*signal_slave)(const struct mail_ops *ops, int ch);
	bool (*slave_signalled)(const struct mail_ops *ops, int ch);
	void (*ring_doorbell)(int ch);
	void (*return_data)(const struct mail_ops *ops, int ch,
			int code, void *data, int sz);
};

extern struct mail_ops chip_mail_ops;
extern struct channel_data channel_area[NR_CHANNELS];
extern char firmware_tag[32];

struct dentry *bpmp_init_debug(struct platform_device *pdev);
int bpmp_init_cpuidle_debug(struct dentry *root);
int bpmp_mail_init_prepare(void);
int bpmp_mail_init(const struct mail_ops *ops, struct device_node *of_node);
int __bpmp_do_ping(void);
int bpmp_create_attrs(const struct fops_entry *fent, struct dentry *parent,
		void *data);
int bpmp_mailman_init(void);
void bpmp_handle_mail(int mrq, int ch);
void tegra_bpmp_resume(void);
void bpmp_handle_irq(int ch);

#endif
