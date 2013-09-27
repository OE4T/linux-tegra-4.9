/*
 * Tegra GK20A GPU Debugger Driver
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __DBG_GPU_GK20A_H_
#define __DBG_GPU_GK20A_H_

/* module debug driver interface */
int gk20a_dbg_gpu_dev_release(struct inode *inode, struct file *filp);
int gk20a_dbg_gpu_dev_open(struct inode *inode, struct file *filp);
long gk20a_dbg_gpu_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

/* used by profiler driver interface */
int gk20a_prof_gpu_dev_open(struct inode *inode, struct file *filp);

struct dbg_gpu_session_ops {
	int (*exec_reg_ops)(struct dbg_session_gk20a *dbg_s,
			    struct nvhost_dbg_gpu_reg_op *ops,
			    u64 num_ops);
};


struct dbg_session_gk20a {
	/* dbg session id used for trace/prints */
	int id;

	/* profiler session, if any */
	bool is_profiler;

	/*
	 * There can be different versions of the whitelists
	 * between both global and per-context sets; as well
	 * as between debugger and profiler interfaces.
	 */
	struct regops_whitelist *global;
	struct regops_whitelist *per_context;

	/* gpu module vagaries */
	struct device             *dev;
	struct platform_device    *pdev;
	struct nvhost_device_data *pdata;
	struct gk20a              *g;

	/* bound hwctx and channel, if any */
	struct file          *hwctx_f;
	struct channel_gk20a *ch;

	/* session operations */
	struct dbg_gpu_session_ops *ops;
};

extern struct dbg_gpu_session_ops dbg_gpu_session_ops_gk20a;

#endif /* __DBG_GPU_GK20A_H_ */
