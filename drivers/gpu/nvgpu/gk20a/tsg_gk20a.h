/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __TSG_GK20A_H_
#define __TSG_GK20A_H_

#define NVGPU_INVALID_TSG_ID (-1)

bool gk20a_is_channel_marked_as_tsg(struct channel_gk20a *ch);

int gk20a_tsg_dev_release(struct inode *inode, struct file *filp);
int gk20a_tsg_dev_open(struct inode *inode, struct file *filp);
int gk20a_tsg_open(struct gk20a *g, struct file *filp);
long gk20a_tsg_dev_ioctl(struct file *filp,
			 unsigned int cmd, unsigned long arg);

int gk20a_init_tsg_support(struct gk20a *g, u32 tsgid);

int gk20a_tsg_unbind_channel(struct channel_gk20a *ch);

struct tsg_gk20a {
	struct gk20a *g;

	bool in_use;
	int tsgid;

	struct kref refcount;

	struct list_head ch_list;
	int num_active_channels;
	struct mutex ch_list_lock;

	struct gr_ctx_desc *tsg_gr_ctx;

	struct vm_gk20a *vm;
};

#endif /* __TSG_GK20A_H_ */
