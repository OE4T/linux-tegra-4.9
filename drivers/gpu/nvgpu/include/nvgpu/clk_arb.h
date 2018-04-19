/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef __NVGPU_CLK_ARB_H__
#define __NVGPU_CLK_ARB_H__

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

#define MAX_F_POINTS 256
#define DEFAULT_EVENT_NUMBER 32

struct nvgpu_clk_dev;
struct nvgpu_clk_arb_target;
struct nvgpu_clk_notification_queue;
struct nvgpu_clk_session;

#define VF_POINT_INVALID_PSTATE ~0U
#define VF_POINT_SET_PSTATE_SUPPORTED(a, b) ((a)->pstates |= (1UL << (b)))
#define VF_POINT_GET_PSTATE(a)	(((a)->pstates) ?\
	__fls((a)->pstates) :\
	VF_POINT_INVALID_PSTATE)
#define VF_POINT_COMMON_PSTATE(a, b)	(((a)->pstates & (b)->pstates) ?\
	__fls((a)->pstates & (b)->pstates) :\
	VF_POINT_INVALID_PSTATE)

/* Local Alarms */
#define EVENT(alarm)	(0x1UL << NVGPU_GPU_EVENT_##alarm)

#define LOCAL_ALARM_MASK (EVENT(ALARM_LOCAL_TARGET_VF_NOT_POSSIBLE) | \
				EVENT(VF_UPDATE))

#define _WRAPGTEQ(a, b) ((a-b) > 0)

struct nvgpu_clk_notification {
	u32 notification;
	u64 timestamp;
};

struct nvgpu_clk_notification_queue {
	u32 size;
	nvgpu_atomic_t head;
	nvgpu_atomic_t tail;
	struct nvgpu_clk_notification *notifications;
};

struct nvgpu_clk_vf_point {
	u16 pstates;
	union {
		struct {
			u16 gpc_mhz;
			u16 sys_mhz;
			u16 xbar_mhz;
		};
		u16 mem_mhz;
	};
	u32 uvolt;
	u32 uvolt_sram;
};

struct nvgpu_clk_vf_table {
	u32 mclk_num_points;
	struct nvgpu_clk_vf_point *mclk_points;
	u32 gpc2clk_num_points;
	struct nvgpu_clk_vf_point *gpc2clk_points;
};
#ifdef CONFIG_DEBUG_FS
struct nvgpu_clk_arb_debug {
	s64 switch_max;
	s64 switch_min;
	u64 switch_num;
	s64 switch_avg;
	s64 switch_std;
};
#endif

struct nvgpu_clk_arb_target {
	u16 mclk;
	u16 gpc2clk;
	u32 pstate;
};

int nvgpu_clk_arb_init_arbiter(struct gk20a *g);

int nvgpu_clk_arb_get_arbiter_clk_range(struct gk20a *g, u32 api_domain,
		u16 *min_mhz, u16 *max_mhz);

int nvgpu_clk_arb_get_arbiter_actual_mhz(struct gk20a *g,
		u32 api_domain, u16 *actual_mhz);

int nvgpu_clk_arb_get_arbiter_effective_mhz(struct gk20a *g,
		u32 api_domain, u16 *effective_mhz);

int nvgpu_clk_arb_get_arbiter_clk_f_points(struct gk20a *g,
	u32 api_domain, u32 *max_points, u16 *fpoints);

u32 nvgpu_clk_arb_get_arbiter_clk_domains(struct gk20a *g);
bool nvgpu_clk_arb_is_valid_domain(struct gk20a *g, u32 api_domain);

void nvgpu_clk_arb_cleanup_arbiter(struct gk20a *g);

int nvgpu_clk_arb_install_session_fd(struct gk20a *g,
		struct nvgpu_clk_session *session);

int nvgpu_clk_arb_init_session(struct gk20a *g,
		struct nvgpu_clk_session **_session);

void nvgpu_clk_arb_release_session(struct gk20a *g,
		struct nvgpu_clk_session *session);

int nvgpu_clk_arb_commit_request_fd(struct gk20a *g,
	struct nvgpu_clk_session *session, int request_fd);

int nvgpu_clk_arb_set_session_target_mhz(struct nvgpu_clk_session *session,
		int fd, u32 api_domain, u16 target_mhz);

int nvgpu_clk_arb_get_session_target_mhz(struct nvgpu_clk_session *session,
		u32 api_domain, u16 *target_mhz);

int nvgpu_clk_arb_install_event_fd(struct gk20a *g,
	struct nvgpu_clk_session *session, int *event_fd, u32 alarm_mask);

int nvgpu_clk_arb_install_request_fd(struct gk20a *g,
	struct nvgpu_clk_session *session, int *event_fd);

void nvgpu_clk_arb_schedule_vf_table_update(struct gk20a *g);

int nvgpu_clk_arb_get_current_pstate(struct gk20a *g);

void nvgpu_clk_arb_pstate_change_lock(struct gk20a *g, bool lock);

void nvgpu_clk_arb_send_thermal_alarm(struct gk20a *g);

void nvgpu_clk_arb_schedule_alarm(struct gk20a *g, u32 alarm);

void nvgpu_clk_arb_free_session(struct nvgpu_ref *refcount);

void nvgpu_clk_arb_free_fd(struct nvgpu_ref *refcount);

int nvgpu_clk_notification_queue_alloc(struct gk20a *g,
				struct nvgpu_clk_notification_queue *queue,
				size_t events_number);

void nvgpu_clk_notification_queue_free(struct gk20a *g,
		struct nvgpu_clk_notification_queue *queue);
#ifdef CONFIG_DEBUG_FS
int nvgpu_clk_arb_debugfs_init(struct gk20a *g);
#endif
#endif /* __NVGPU_CLK_ARB_H__ */

