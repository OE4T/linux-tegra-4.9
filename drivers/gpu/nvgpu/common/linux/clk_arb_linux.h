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
#include <nvgpu/list.h>

#include "gk20a/gk20a.h"
#include "clk/clk.h"
#include "pstate/pstate.h"
#include "lpwr/lpwr.h"
#include "volt/volt.h"

/*
 * The defines here should finally move to clk_arb.h, once these are
 * refactored to be free of Linux fields.
 */

enum clk_arb_work_item_type {
	CLK_ARB_WORK_UPDATE_VF_TABLE,
	CLK_ARB_WORK_UPDATE_ARB
};

struct nvgpu_clk_arb_work_item {
	enum clk_arb_work_item_type item_type;
	struct nvgpu_clk_arb *arb;
	struct nvgpu_list_node worker_item;
};

struct nvgpu_clk_arb {
	struct nvgpu_spinlock sessions_lock;
	struct nvgpu_spinlock users_lock;
	struct nvgpu_spinlock requests_lock;

	struct nvgpu_mutex pstate_lock;
	struct nvgpu_list_node users;
	struct nvgpu_list_node sessions;
	struct nvgpu_list_node requests;

	struct gk20a *g;
	int status;

	struct nvgpu_clk_arb_target actual_pool[2];
	struct nvgpu_clk_arb_target *actual;

	u16 gpc2clk_default_mhz;
	u16 mclk_default_mhz;
	u32 voltuv_actual;

	u16 gpc2clk_min, gpc2clk_max;
	u16 mclk_min, mclk_max;

	struct nvgpu_clk_arb_work_item update_vf_table_work_item;
	struct nvgpu_clk_arb_work_item update_arb_work_item;

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
		struct nvgpu_list_node link;
		struct nvgpu_list_node node;
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
	struct nvgpu_list_node link;
	struct nvgpu_list_node targets;

	struct nvgpu_spinlock session_lock;
	struct nvgpu_clk_arb_target target_pool[2];
	struct nvgpu_clk_arb_target *target;
};

static inline struct nvgpu_clk_session *
nvgpu_clk_session_from_link(struct nvgpu_list_node *node)
{
	return (struct nvgpu_clk_session *)
	   ((uintptr_t)node - offsetof(struct nvgpu_clk_session, link));
};

static inline struct nvgpu_clk_dev *
nvgpu_clk_dev_from_node(struct nvgpu_list_node *node)
{
	return (struct nvgpu_clk_dev *)
	   ((uintptr_t)node - offsetof(struct nvgpu_clk_dev, node));
};

static inline struct nvgpu_clk_dev *
nvgpu_clk_dev_from_link(struct nvgpu_list_node *node)
{
	return (struct nvgpu_clk_dev *)
	   ((uintptr_t)node - offsetof(struct nvgpu_clk_dev, link));
};

static inline struct nvgpu_clk_arb_work_item *
nvgpu_clk_arb_work_item_from_worker_item(struct nvgpu_list_node *node)
{
	return (struct nvgpu_clk_arb_work_item *)
	   ((uintptr_t)node - offsetof(struct nvgpu_clk_arb_work_item, worker_item));
};

void nvgpu_clk_arb_worker_enqueue(struct gk20a *g,
		struct nvgpu_clk_arb_work_item *work_item);
#endif /* __NVGPU_CLK_ARB_LINUX_H__ */

