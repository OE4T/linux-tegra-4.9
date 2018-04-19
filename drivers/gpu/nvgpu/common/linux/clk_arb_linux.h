/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __NVGPU_CLK_ARB_LINUX_H__
#define __NVGPU_CLK_ARB_LINUX_H__

#include <nvgpu/types.h>
#include <nvgpu/bitops.h>
#include <nvgpu/lock.h>
#include <nvgpu/kmem.h>
#include <nvgpu/atomic.h>
#include <nvgpu/bug.h>
#include <nvgpu/kref.h>
#include <nvgpu/log.h>
#include <nvgpu/barrier.h>
#include <nvgpu/cond.h>

#include "gk20a/gk20a.h"
#include "clk/clk.h"
#include "pstate/pstate.h"
#include "lpwr/lpwr.h"
#include "volt/volt.h"

/*
 * The defines here should finally move to clk_arb.h, once these are
 * refactored to be free of Linux fields.
 */
struct nvgpu_clk_arb {
	struct nvgpu_spinlock sessions_lock;
	struct nvgpu_spinlock users_lock;

	struct nvgpu_mutex pstate_lock;
	struct list_head users;
	struct list_head sessions;
	struct llist_head requests;

	struct gk20a *g;
	int status;

	struct nvgpu_clk_arb_target actual_pool[2];
	struct nvgpu_clk_arb_target *actual;

	u16 gpc2clk_default_mhz;
	u16 mclk_default_mhz;
	u32 voltuv_actual;

	u16 gpc2clk_min, gpc2clk_max;
	u16 mclk_min, mclk_max;

	struct work_struct update_fn_work;
	struct workqueue_struct *update_work_queue;
	struct work_struct vf_table_fn_work;
	struct workqueue_struct *vf_table_work_queue;

	struct nvgpu_cond request_wq;

	struct nvgpu_clk_vf_table *current_vf_table;
	struct nvgpu_clk_vf_table vf_table_pool[2];
	u32 vf_table_index;

	u16 *mclk_f_points;
	nvgpu_atomic_t req_nr;

	u32 mclk_f_numpoints;
	u16 *gpc2clk_f_points;
	u32 gpc2clk_f_numpoints;

	nvgpu_atomic64_t alarm_mask;
	struct nvgpu_clk_notification_queue notification_queue;

#ifdef CONFIG_DEBUG_FS
	struct nvgpu_clk_arb_debug debug_pool[2];
	struct nvgpu_clk_arb_debug *debug;
	bool debugfs_set;
#endif
};

struct nvgpu_clk_dev {
	struct nvgpu_clk_session *session;
	union {
		struct list_head link;
		struct llist_node node;
	};
	struct nvgpu_cond readout_wq;
	nvgpu_atomic_t poll_mask;
	u16 gpc2clk_target_mhz;
	u16 mclk_target_mhz;
	u32 alarms_reported;
	nvgpu_atomic_t enabled_mask;
	struct nvgpu_clk_notification_queue queue;
	u32 arb_queue_head;
	struct nvgpu_ref refcount;
};

struct nvgpu_clk_session {
	bool zombie;
	struct gk20a *g;
	struct nvgpu_ref refcount;
	struct list_head link;
	struct llist_head targets;

	struct nvgpu_clk_arb_target target_pool[2];
	struct nvgpu_clk_arb_target *target;
};

#endif /* __NVGPU_CLK_ARB_LINUX_H__ */

