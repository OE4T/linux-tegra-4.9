/*
 * Tegra GK20A GPU Debugger/Profiler Driver
 *
 * Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/fs.h>
#include <linux/file.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/nvhost.h>
#include <uapi/linux/nvgpu.h>

#include "gk20a.h"
#include "gr_gk20a.h"
#include "dbg_gpu_gk20a.h"
#include "regops_gk20a.h"
#include "hw_therm_gk20a.h"
#include "hw_gr_gk20a.h"
#include "hw_perf_gk20a.h"

struct dbg_gpu_session_ops dbg_gpu_session_ops_gk20a = {
	.exec_reg_ops = exec_regops_gk20a,
};

/* silly allocator - just increment session id */
static atomic_t session_id = ATOMIC_INIT(0);
static int generate_session_id(void)
{
	return atomic_add_return(1, &session_id);
}

static int alloc_session(struct dbg_session_gk20a **_dbg_s)
{
	struct dbg_session_gk20a *dbg_s;
	*_dbg_s = NULL;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	dbg_s = kzalloc(sizeof(*dbg_s), GFP_KERNEL);
	if (!dbg_s)
		return -ENOMEM;

	dbg_s->id = generate_session_id();
	dbg_s->ops = &dbg_gpu_session_ops_gk20a;
	*_dbg_s = dbg_s;
	return 0;
}

static int gk20a_dbg_gpu_do_dev_open(struct inode *inode,
		struct file *filp, bool is_profiler)
{
	struct dbg_session_gk20a *dbg_session;
	struct gk20a *g;

	struct platform_device *pdev;
	struct device *dev;

	int err;

	if (!is_profiler)
		g = container_of(inode->i_cdev,
				 struct gk20a, dbg.cdev);
	else
		g = container_of(inode->i_cdev,
				 struct gk20a, prof.cdev);
	pdev = g->dev;
	dev  = &pdev->dev;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "dbg session: %s", dev_name(dev));

	err  = alloc_session(&dbg_session);
	if (err)
		return err;

	filp->private_data = dbg_session;
	dbg_session->pdev  = pdev;
	dbg_session->dev   = dev;
	dbg_session->g     = g;
	dbg_session->is_profiler = is_profiler;
	dbg_session->is_pg_disabled = false;
	/* For vgpu, all power-gating features are currently disabled
	 * in the server. Set is_pg_disable to true to reflect this
	 * on the client side. */
	if (gk20a_gpu_is_virtual(pdev))
		dbg_session->is_pg_disabled = true;

	INIT_LIST_HEAD(&dbg_session->dbg_s_list_node);
	init_waitqueue_head(&dbg_session->dbg_events.wait_queue);
	dbg_session->dbg_events.events_enabled = false;
	dbg_session->dbg_events.num_pending_events = 0;

	return 0;
}

/* used in scenarios where the debugger session can take just the inter-session
 * lock for performance, but the profiler session must take the per-gpu lock
 * since it might not have an associated channel. */
static void gk20a_dbg_session_mutex_lock(struct dbg_session_gk20a *dbg_s)
{
	if (dbg_s->is_profiler)
		mutex_lock(&dbg_s->g->dbg_sessions_lock);
	else
		mutex_lock(&dbg_s->ch->dbg_s_lock);
}

static void gk20a_dbg_session_mutex_unlock(struct dbg_session_gk20a *dbg_s)
{
	if (dbg_s->is_profiler)
		mutex_unlock(&dbg_s->g->dbg_sessions_lock);
	else
		mutex_unlock(&dbg_s->ch->dbg_s_lock);
}

static void gk20a_dbg_gpu_events_enable(struct dbg_session_gk20a *dbg_s)
{
	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	gk20a_dbg_session_mutex_lock(dbg_s);

	dbg_s->dbg_events.events_enabled = true;
	dbg_s->dbg_events.num_pending_events = 0;

	gk20a_dbg_session_mutex_unlock(dbg_s);
}

static void gk20a_dbg_gpu_events_disable(struct dbg_session_gk20a *dbg_s)
{
	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	gk20a_dbg_session_mutex_lock(dbg_s);

	dbg_s->dbg_events.events_enabled = false;
	dbg_s->dbg_events.num_pending_events = 0;

	gk20a_dbg_session_mutex_unlock(dbg_s);
}

static void gk20a_dbg_gpu_events_clear(struct dbg_session_gk20a *dbg_s)
{
	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	gk20a_dbg_session_mutex_lock(dbg_s);

	if (dbg_s->dbg_events.events_enabled &&
			dbg_s->dbg_events.num_pending_events > 0)
		dbg_s->dbg_events.num_pending_events--;

	gk20a_dbg_session_mutex_unlock(dbg_s);
}

static int gk20a_dbg_gpu_events_ctrl(struct dbg_session_gk20a *dbg_s,
			  struct nvgpu_dbg_gpu_events_ctrl_args *args)
{
	int ret = 0;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "dbg events ctrl cmd %d", args->cmd);

	if (!dbg_s->ch) {
		gk20a_err(dev_from_gk20a(dbg_s->g),
			   "no channel bound to dbg session\n");
		return -EINVAL;
	}

	switch (args->cmd) {
	case NVGPU_DBG_GPU_EVENTS_CTRL_CMD_ENABLE:
		gk20a_dbg_gpu_events_enable(dbg_s);
		break;

	case NVGPU_DBG_GPU_EVENTS_CTRL_CMD_DISABLE:
		gk20a_dbg_gpu_events_disable(dbg_s);
		break;

	case NVGPU_DBG_GPU_EVENTS_CTRL_CMD_CLEAR:
		gk20a_dbg_gpu_events_clear(dbg_s);
		break;

	default:
		gk20a_err(dev_from_gk20a(dbg_s->g),
			   "unrecognized dbg gpu events ctrl cmd: 0x%x",
			   args->cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

unsigned int gk20a_dbg_gpu_dev_poll(struct file *filep, poll_table *wait)
{
	unsigned int mask = 0;
	struct dbg_session_gk20a *dbg_s = filep->private_data;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	poll_wait(filep, &dbg_s->dbg_events.wait_queue, wait);

	gk20a_dbg_session_mutex_lock(dbg_s);

	if (dbg_s->dbg_events.events_enabled &&
			dbg_s->dbg_events.num_pending_events > 0) {
		gk20a_dbg(gpu_dbg_gpu_dbg, "found pending event on session id %d",
				dbg_s->id);
		gk20a_dbg(gpu_dbg_gpu_dbg, "%d events pending",
				dbg_s->dbg_events.num_pending_events);
		mask = (POLLPRI | POLLIN);
	}

	gk20a_dbg_session_mutex_unlock(dbg_s);

	return mask;
}

int gk20a_dbg_gpu_dev_open(struct inode *inode, struct file *filp)
{
	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");
	return gk20a_dbg_gpu_do_dev_open(inode, filp, false /* not profiler */);
}

int gk20a_prof_gpu_dev_open(struct inode *inode, struct file *filp)
{
	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");
	return gk20a_dbg_gpu_do_dev_open(inode, filp, true /* is profiler */);
}

void gk20a_dbg_gpu_post_events(struct channel_gk20a *ch)
{
	struct dbg_session_gk20a *dbg_s;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	/* guard against the session list being modified */
	mutex_lock(&ch->dbg_s_lock);

	list_for_each_entry(dbg_s, &ch->dbg_s_list, dbg_s_list_node) {
		if (dbg_s->dbg_events.events_enabled) {
			gk20a_dbg(gpu_dbg_gpu_dbg, "posting event on session id %d",
					dbg_s->id);
			gk20a_dbg(gpu_dbg_gpu_dbg, "%d events pending",
					dbg_s->dbg_events.num_pending_events);

			dbg_s->dbg_events.num_pending_events++;

			wake_up_interruptible_all(&dbg_s->dbg_events.wait_queue);
		}
	}

	mutex_unlock(&ch->dbg_s_lock);
}


static int dbg_set_powergate(struct dbg_session_gk20a *dbg_s,
				__u32  powermode);

static int dbg_unbind_channel_gk20a(struct dbg_session_gk20a *dbg_s)
{
	struct channel_gk20a *ch_gk20a = dbg_s->ch;
	struct gk20a *g = dbg_s->g;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	/* wasn't bound to start with ? */
	if (!ch_gk20a) {
		gk20a_dbg(gpu_dbg_gpu_dbg | gpu_dbg_fn, "not bound already?");
		return -ENODEV;
	}

	mutex_lock(&g->dbg_sessions_lock);
	mutex_lock(&ch_gk20a->dbg_s_lock);

	--g->dbg_sessions;

	/* Powergate enable is called here as possibility of dbg_session
	 * which called powergate disable ioctl, to be killed without calling
	 * powergate enable ioctl
	 */
	dbg_set_powergate(dbg_s, NVGPU_DBG_GPU_POWERGATE_MODE_ENABLE);

	dbg_s->ch = NULL;
	fput(dbg_s->ch_f);
	dbg_s->ch_f = NULL;

	list_del_init(&dbg_s->dbg_s_list_node);

	mutex_unlock(&ch_gk20a->dbg_s_lock);
	mutex_unlock(&g->dbg_sessions_lock);

	return 0;
}

int gk20a_dbg_gpu_dev_release(struct inode *inode, struct file *filp)
{
	struct dbg_session_gk20a *dbg_s = filp->private_data;

	gk20a_dbg(gpu_dbg_gpu_dbg | gpu_dbg_fn, "%s", dev_name(dbg_s->dev));

	/* unbind if it was bound */
	if (dbg_s->ch)
		dbg_unbind_channel_gk20a(dbg_s);

	kfree(dbg_s);
	return 0;
}

static int dbg_bind_channel_gk20a(struct dbg_session_gk20a *dbg_s,
			  struct nvgpu_dbg_gpu_bind_channel_args *args)
{
	struct file *f;
	struct gk20a *g;
	struct channel_gk20a *ch;

	gk20a_dbg(gpu_dbg_fn|gpu_dbg_gpu_dbg, "%s fd=%d",
		   dev_name(dbg_s->dev), args->channel_fd);

	if (args->channel_fd == ~0)
		return dbg_unbind_channel_gk20a(dbg_s);

	/* even though get_file_channel is doing this it releases it as well */
	/* by holding it here we'll keep it from disappearing while the
	 * debugger is in session */
	f = fget(args->channel_fd);
	if (!f)
		return -ENODEV;

	ch = gk20a_get_channel_from_file(args->channel_fd);
	if (!ch) {
		gk20a_dbg_fn("no channel found for fd");
		fput(f);
		return -EINVAL;
	}

	g = dbg_s->g;
	gk20a_dbg_fn("%s hwchid=%d", dev_name(dbg_s->dev), ch->hw_chid);

	mutex_lock(&g->dbg_sessions_lock);
	mutex_lock(&ch->dbg_s_lock);

	dbg_s->ch_f = f;
	dbg_s->ch = ch;
	list_add(&dbg_s->dbg_s_list_node, &dbg_s->ch->dbg_s_list);

	g->dbg_sessions++;

	mutex_unlock(&ch->dbg_s_lock);
	mutex_unlock(&g->dbg_sessions_lock);
	return 0;
}

static int nvgpu_ioctl_channel_reg_ops(struct dbg_session_gk20a *dbg_s,
				struct nvgpu_dbg_gpu_exec_reg_ops_args *args);

static int nvgpu_ioctl_powergate_gk20a(struct dbg_session_gk20a *dbg_s,
				struct nvgpu_dbg_gpu_powergate_args *args);

static int nvgpu_dbg_gpu_ioctl_smpc_ctxsw_mode(struct dbg_session_gk20a *dbg_s,
			      struct nvgpu_dbg_gpu_smpc_ctxsw_mode_args *args);

static int nvgpu_dbg_gpu_ioctl_suspend_resume_sm(
		struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_suspend_resume_all_sms_args *args);

static int gk20a_perfbuf_map(struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_perfbuf_map_args *args);

static int gk20a_perfbuf_unmap(struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_perfbuf_unmap_args *args);

static int gk20a_dbg_pc_sampling(struct dbg_session_gk20a *dbg_s,
			  struct nvgpu_dbg_gpu_pc_sampling_args *args)
{
	struct channel_gk20a *ch = dbg_s->ch;
	struct gk20a *g = ch->g;

	gk20a_dbg_fn("");

	return g->ops.gr.update_pc_sampling ?
		g->ops.gr.update_pc_sampling(ch, args->enable) : -EINVAL;
}
long gk20a_dbg_gpu_dev_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	struct dbg_session_gk20a *dbg_s = filp->private_data;
	struct gk20a *g = get_gk20a(dbg_s->pdev);
	u8 buf[NVGPU_DBG_GPU_IOCTL_MAX_ARG_SIZE];
	int err = 0;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "");

	if ((_IOC_TYPE(cmd) != NVGPU_DBG_GPU_IOCTL_MAGIC) ||
	    (_IOC_NR(cmd) == 0) ||
	    (_IOC_NR(cmd) > NVGPU_DBG_GPU_IOCTL_LAST))
		return -EINVAL;

	BUG_ON(_IOC_SIZE(cmd) > NVGPU_DBG_GPU_IOCTL_MAX_ARG_SIZE);

	memset(buf, 0, sizeof(buf));
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	if (!g->gr.sw_ready) {
		err = gk20a_busy(g->dev);
		if (err)
			return err;

		gk20a_idle(g->dev);
	}

	switch (cmd) {
	case NVGPU_DBG_GPU_IOCTL_BIND_CHANNEL:
		err = dbg_bind_channel_gk20a(dbg_s,
			     (struct nvgpu_dbg_gpu_bind_channel_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_REG_OPS:
		err = nvgpu_ioctl_channel_reg_ops(dbg_s,
			   (struct nvgpu_dbg_gpu_exec_reg_ops_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_POWERGATE:
		err = nvgpu_ioctl_powergate_gk20a(dbg_s,
			   (struct nvgpu_dbg_gpu_powergate_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_EVENTS_CTRL:
		err = gk20a_dbg_gpu_events_ctrl(dbg_s,
			   (struct nvgpu_dbg_gpu_events_ctrl_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_SMPC_CTXSW_MODE:
		err = nvgpu_dbg_gpu_ioctl_smpc_ctxsw_mode(dbg_s,
			   (struct nvgpu_dbg_gpu_smpc_ctxsw_mode_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_SUSPEND_RESUME_ALL_SMS:
		err = nvgpu_dbg_gpu_ioctl_suspend_resume_sm(dbg_s,
		       (struct nvgpu_dbg_gpu_suspend_resume_all_sms_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_PERFBUF_MAP:
		err = gk20a_perfbuf_map(dbg_s,
		       (struct nvgpu_dbg_gpu_perfbuf_map_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_PERFBUF_UNMAP:
		err = gk20a_perfbuf_unmap(dbg_s,
		       (struct nvgpu_dbg_gpu_perfbuf_unmap_args *)buf);
		break;

	case NVGPU_DBG_GPU_IOCTL_PC_SAMPLING:
		err = gk20a_dbg_pc_sampling(dbg_s,
			   (struct nvgpu_dbg_gpu_pc_sampling_args *)buf);
		break;

	default:
		gk20a_err(dev_from_gk20a(g),
			   "unrecognized dbg gpu ioctl cmd: 0x%x",
			   cmd);
		err = -ENOTTY;
		break;
	}

	gk20a_dbg(gpu_dbg_gpu_dbg, "ret=%d", err);

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg,
				   buf, _IOC_SIZE(cmd));

	return err;
}

/* In order to perform a context relative op the context has
 * to be created already... which would imply that the
 * context switch mechanism has already been put in place.
 * So by the time we perform such an opertation it should always
 * be possible to query for the appropriate context offsets, etc.
 *
 * But note: while the dbg_gpu bind requires the a channel fd,
 * it doesn't require an allocated gr/compute obj at that point...
 */
static bool gr_context_info_available(struct dbg_session_gk20a *dbg_s,
				      struct gr_gk20a *gr)
{
	int err;

	mutex_lock(&gr->ctx_mutex);
	err = !gr->ctx_vars.golden_image_initialized;
	mutex_unlock(&gr->ctx_mutex);
	if (err)
		return false;
	return true;

}

static int nvgpu_ioctl_channel_reg_ops(struct dbg_session_gk20a *dbg_s,
				struct nvgpu_dbg_gpu_exec_reg_ops_args *args)
{
	int err = 0, powergate_err = 0;
	bool is_pg_disabled = false;

	struct device *dev = dbg_s->dev;
	struct gk20a *g = get_gk20a(dbg_s->pdev);
	struct nvgpu_dbg_gpu_reg_op *ops;
	u64 ops_size = sizeof(ops[0]) * args->num_ops;

	gk20a_dbg_fn("%d ops, total size %llu", args->num_ops, ops_size);

	if (!dbg_s->ops) {
		gk20a_err(dev, "can't call reg_ops on an unbound debugger session");
		return -EINVAL;
	}

	if (!dbg_s->is_profiler && !dbg_s->ch) {
		gk20a_err(dev, "bind a channel before regops for a debugging session");
		return -EINVAL;
	}

	/* be sure that ctx info is in place */
	if (!gk20a_gpu_is_virtual(dbg_s->pdev) &&
		!gr_context_info_available(dbg_s, &g->gr)) {
		gk20a_err(dev, "gr context data not available\n");
		return -ENODEV;
	}

	ops = kzalloc(ops_size, GFP_KERNEL);
	if (!ops) {
		gk20a_err(dev, "Allocating memory failed!");
		return -ENOMEM;
	}

	gk20a_dbg_fn("Copying regops from userspace");

	if (copy_from_user(ops, (void __user *)(uintptr_t)args->ops,
							ops_size)) {
		dev_err(dev, "copy_from_user failed!");
		err = -EFAULT;
		goto clean_up;
	}

	/* since exec_reg_ops sends methods to the ucode, it must take the
	 * global gpu lock to protect against mixing methods from debug sessions
	 * on other channels */
	mutex_lock(&g->dbg_sessions_lock);

	if (!dbg_s->is_pg_disabled) {
		powergate_err = dbg_set_powergate(dbg_s,
					NVGPU_DBG_GPU_POWERGATE_MODE_DISABLE);
		is_pg_disabled = true;
	}

	if (!powergate_err) {
		err = dbg_s->ops->exec_reg_ops(dbg_s, ops, args->num_ops);
		/* enable powergate, if previously disabled */
		if (is_pg_disabled) {
			powergate_err = dbg_set_powergate(dbg_s,
					NVGPU_DBG_GPU_POWERGATE_MODE_ENABLE);
		}
	}

	mutex_unlock(&g->dbg_sessions_lock);

	if (!err && powergate_err)
		err = powergate_err;

	if (err) {
		gk20a_err(dev, "dbg regops failed");
		goto clean_up;
	}

	gk20a_dbg_fn("Copying result to userspace");

	if (copy_to_user((void __user *)(uintptr_t)args->ops, ops, ops_size)) {
		dev_err(dev, "copy_to_user failed!");
		err = -EFAULT;
		goto clean_up;
	}

 clean_up:
	kfree(ops);
	return err;
}

static int dbg_set_powergate(struct dbg_session_gk20a *dbg_s,
				__u32  powermode)
{
	int err = 0;
	struct gk20a *g = get_gk20a(dbg_s->pdev);

	 /* This function must be called with g->dbg_sessions_lock held */

	gk20a_dbg(gpu_dbg_fn|gpu_dbg_gpu_dbg, "%s powergate mode = %d",
		   dev_name(dbg_s->dev), powermode);

	switch (powermode) {
	case NVGPU_DBG_GPU_POWERGATE_MODE_DISABLE:
		/* save off current powergate, clk state.
		 * set gpu module's can_powergate = 0.
		 * set gpu module's clk to max.
		 * while *a* debug session is active there will be no power or
		 * clocking state changes allowed from mainline code (but they
		 * should be saved).
		 */
		/* Allow powergate disable if the current dbg_session doesn't
		 * call a powergate disable ioctl and the global
		 * powergating_disabled_refcount is zero
		 */

		if ((dbg_s->is_pg_disabled == false) &&
		    (g->dbg_powergating_disabled_refcount++ == 0)) {

			gk20a_dbg(gpu_dbg_gpu_dbg | gpu_dbg_fn, "module busy");
			err = gk20a_busy(g->dev);
			if (err)
				return err;

			err = gk20a_busy(dbg_s->pdev);
			if (err)
				return -EPERM;

			/*do elpg disable before clock gating */
			if (support_gk20a_pmu(g->dev))
				gk20a_pmu_disable_elpg(g);
			g->ops.clock_gating.slcg_gr_load_gating_prod(g,
					false);
			g->ops.clock_gating.slcg_perf_load_gating_prod(g,
					false);
			g->ops.clock_gating.slcg_ltc_load_gating_prod(g,
					false);
			gr_gk20a_init_blcg_mode(g, BLCG_RUN, ENGINE_GR_GK20A);

			g->elcg_enabled = false;
			gr_gk20a_init_elcg_mode(g, ELCG_RUN, ENGINE_GR_GK20A);
			gr_gk20a_init_elcg_mode(g, ELCG_RUN, ENGINE_CE2_GK20A);

		}

		dbg_s->is_pg_disabled = true;
		break;

	case NVGPU_DBG_GPU_POWERGATE_MODE_ENABLE:
		/* restore (can) powergate, clk state */
		/* release pending exceptions to fault/be handled as usual */
		/*TBD: ordering of these? */

		/* Re-enabling powergate as no other sessions want
		 * powergate disabled and the current dbg-sessions had
		 * requested the powergate disable through ioctl
		*/
		if (dbg_s->is_pg_disabled &&
		    --g->dbg_powergating_disabled_refcount == 0) {

			g->elcg_enabled = true;
			gr_gk20a_init_elcg_mode(g, ELCG_AUTO, ENGINE_GR_GK20A);
			gr_gk20a_init_elcg_mode(g, ELCG_AUTO, ENGINE_CE2_GK20A);
			gr_gk20a_init_blcg_mode(g, BLCG_AUTO, ENGINE_GR_GK20A);

			g->ops.clock_gating.slcg_gr_load_gating_prod(g,
					g->slcg_enabled);
			g->ops.clock_gating.slcg_perf_load_gating_prod(g,
					g->slcg_enabled);

			if (support_gk20a_pmu(g->dev))
				gk20a_pmu_enable_elpg(g);

			gk20a_dbg(gpu_dbg_gpu_dbg | gpu_dbg_fn, "module idle");
			gk20a_idle(dbg_s->pdev);
			gk20a_idle(g->dev);
		}

		dbg_s->is_pg_disabled = false;
		break;

	default:
		gk20a_err(dev_from_gk20a(g),
			   "unrecognized dbg gpu powergate mode: 0x%x",
			   powermode);
		err = -ENOTTY;
		break;
	}

	gk20a_dbg(gpu_dbg_fn|gpu_dbg_gpu_dbg, "%s powergate mode = %d done",
		   dev_name(dbg_s->dev), powermode);
	return err;
}

static int nvgpu_ioctl_powergate_gk20a(struct dbg_session_gk20a *dbg_s,
				struct nvgpu_dbg_gpu_powergate_args *args)
{
	int err;
	struct gk20a *g = get_gk20a(dbg_s->pdev);
	gk20a_dbg_fn("%s  powergate mode = %d",
		      dev_name(dbg_s->dev), args->mode);

	mutex_lock(&g->dbg_sessions_lock);
	err = dbg_set_powergate(dbg_s, args->mode);
	mutex_unlock(&g->dbg_sessions_lock);
	return  err;
}

static int nvgpu_dbg_gpu_ioctl_smpc_ctxsw_mode(struct dbg_session_gk20a *dbg_s,
			       struct nvgpu_dbg_gpu_smpc_ctxsw_mode_args *args)
{
	int err;
	struct gk20a *g = get_gk20a(dbg_s->pdev);
	struct channel_gk20a *ch_gk20a;

	gk20a_dbg_fn("%s smpc ctxsw mode = %d",
		     dev_name(dbg_s->dev), args->mode);

	/* Take the global lock, since we'll be doing global regops */
	mutex_lock(&g->dbg_sessions_lock);

	ch_gk20a = dbg_s->ch;

	if (!ch_gk20a) {
		gk20a_err(dev_from_gk20a(dbg_s->g),
			  "no bound channel for smpc ctxsw mode update\n");
		err = -EINVAL;
		goto clean_up;
	}

	err = gr_gk20a_update_smpc_ctxsw_mode(g, ch_gk20a,
		      args->mode == NVGPU_DBG_GPU_SMPC_CTXSW_MODE_CTXSW);
	if (err) {
		gk20a_err(dev_from_gk20a(dbg_s->g),
			  "error (%d) during smpc ctxsw mode update\n", err);
		goto clean_up;
	}

	err = g->ops.regops.apply_smpc_war(dbg_s);

 clean_up:
	mutex_unlock(&g->dbg_sessions_lock);
	return  err;
}

static int nvgpu_dbg_gpu_ioctl_suspend_resume_sm(
		struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_suspend_resume_all_sms_args *args)
{
	struct gk20a *g = get_gk20a(dbg_s->pdev);
	struct channel_gk20a *ch = dbg_s->ch;
	bool ch_is_curr_ctx;
	int err = 0, action = args->mode;

	gk20a_dbg(gpu_dbg_fn | gpu_dbg_gpu_dbg, "action: %d", args->mode);

	mutex_lock(&g->dbg_sessions_lock);

	/* Suspend GPU context switching */
	/* Disable channel switching.
	 * at that point the hardware state can be inspected to
	 * determine if the context we're interested in is current.
	 */
	err = gr_gk20a_disable_ctxsw(g);
	if (err) {
		gk20a_err(dev_from_gk20a(g), "unable to stop gr ctxsw");
		/* this should probably be ctx-fatal... */
		goto clean_up;
	}

	/* find out whether the current channel is resident */
	ch_is_curr_ctx = gk20a_is_channel_ctx_resident(ch);

	if (ch_is_curr_ctx) {
		switch (action) {
		case NVGPU_DBG_GPU_SUSPEND_ALL_SMS:
			gk20a_suspend_all_sms(g);
			break;

		case NVGPU_DBG_GPU_RESUME_ALL_SMS:
			gk20a_resume_all_sms(g);
			break;
		}
	} else {
		switch (action) {
		case NVGPU_DBG_GPU_SUSPEND_ALL_SMS:
			/* Disable the channel */
			channel_gk20a_disable(ch);
			break;

		case NVGPU_DBG_GPU_RESUME_ALL_SMS:
			/* Enable the channel */
			channel_gk20a_enable(ch);
			break;
		}
	}

	/* Resume GPU context switching */
	err = gr_gk20a_enable_ctxsw(g);
	if (err)
		gk20a_err(dev_from_gk20a(g), "unable to restart ctxsw!\n");

 clean_up:
	mutex_unlock(&g->dbg_sessions_lock);
	return  err;
}

static int gk20a_perfbuf_map(struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_perfbuf_map_args *args)
{
	struct gk20a *g = dbg_s->g;
	int err;
	u32 virt_size;
	u32 virt_addr_lo;
	u32 virt_addr_hi;
	u32 inst_pa_page;

	if (!g->allow_all)
		return -EACCES;

	err = gk20a_vm_map_buffer(&g->mm.pmu.vm,
			args->dmabuf_fd,
			&args->offset,
			0,
			0,
			0,
			args->mapping_size,
			NULL);
	if (err)
		return err;

	/* perf output buffer may not cross a 4GB boundary - with a separate va
	 * smaller than that, it won't */
	virt_size = u64_lo32(args->mapping_size);
	virt_addr_lo = u64_lo32(args->offset);
	virt_addr_hi = u64_hi32(args->offset);
	/* but check anyway */
	if (args->offset + virt_size > SZ_4G) {
		gk20a_vm_unmap_buffer(&g->mm.pmu.vm, args->offset, NULL);
		return -EINVAL;
	}

	/* address and size are aligned to 32 bytes, the lowest bits read back
	 * as zeros */
	gk20a_writel(g, perf_pmasys_outbase_r(), virt_addr_lo);
	gk20a_writel(g, perf_pmasys_outbaseupper_r(),
			perf_pmasys_outbaseupper_ptr_f(virt_addr_hi));
	gk20a_writel(g, perf_pmasys_outsize_r(), virt_size);

	/* this field is aligned to 4K */
	inst_pa_page = gk20a_mem_phys(&g->mm.hwpm.inst_block) >> 12;

	/* A write to MEM_BLOCK triggers the block bind operation. MEM_BLOCK
	 * should be written last */
	gk20a_writel(g, perf_pmasys_mem_block_r(),
			perf_pmasys_mem_block_base_f(inst_pa_page) |
			perf_pmasys_mem_block_valid_true_f() |
			perf_pmasys_mem_block_target_lfb_f());

	return 0;
}

static int gk20a_perfbuf_unmap(struct dbg_session_gk20a *dbg_s,
		struct nvgpu_dbg_gpu_perfbuf_unmap_args *args)
{
	struct gk20a *g = dbg_s->g;

	if (!g->allow_all)
		return -EACCES;

	gk20a_writel(g, perf_pmasys_outbase_r(), 0);
	gk20a_writel(g, perf_pmasys_outbaseupper_r(),
			perf_pmasys_outbaseupper_ptr_f(0));
	gk20a_writel(g, perf_pmasys_outsize_r(), 0);

	gk20a_writel(g, perf_pmasys_mem_block_r(),
			perf_pmasys_mem_block_base_f(0) |
			perf_pmasys_mem_block_valid_false_f() |
			perf_pmasys_mem_block_target_f(0));

	gk20a_vm_unmap_buffer(&g->mm.pmu.vm, args->offset, NULL);

	return 0;
}
