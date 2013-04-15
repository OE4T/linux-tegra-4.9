/*
 * drivers/video/tegra/host/host1x/host1x_actmon.c
 *
 * Tegra Graphics Host Actmon support
 *
 * Copyright (c) 2012-2013, NVIDIA Corporation.
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

#include <linux/nvhost.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include "dev.h"
#include "chip_support.h"

enum init_e {
	ACTMON_OFF = 0,
	ACTMON_READY = 1,
	ACTMON_SLEEP = 2
};

struct actmon_status_t {

	/* Set to 1 if actmon has been initialized */
	enum init_e init;

	/* Counters for debug usage */
	int above_wmark;
	int below_wmark;

	/* Store actmon period. clks_per_sample can be used even when host1x is
	 * not active. */
	long usecs_per_sample;
	long clks_per_sample;

	int k;
};

static struct actmon_status_t actmon_status;

/*******************************************************************************
 * host1x_actmon_update_sample_period_safe(host)
 *
 * This function updates frequency specific values on actmon using the current
 * host1x frequency. The function should be called only when host1x is active.
 *
 * Actmon takes samples 3d activity every clock cycle. If 3d is active,
 * the internal counter is increased by one. Depending on value of
 * host1x_sync_actmon_status_gr3d_mon_act_f(x), these internal values
 * grow either to 255 (1) or 65535 (0). After growing up to this value,
 * the actual counter is increased by one.
 *
 * Each period takes a number of counter increments. If we wish a period
 * to be a known time, we get the number of counter increments by:
 *
 *          f_{host}
 *   clks = --------- * t_{sample_period}.
 *             256
 *
 * This is used as a base value for determining a proper value for raw sample
 * period value.
 ******************************************************************************/

static void host1x_actmon_update_sample_period_safe(struct nvhost_master *host)
{
	void __iomem *sync_regs = host->sync_aperture;
	long freq_mhz, clks_per_sample;
	struct nvhost_device_data *pdata = platform_get_drvdata(host->dev);

	/* We use MHz and us instead of Hz and s due to numerical limitations */
	freq_mhz = clk_get_rate(pdata->clk[0]) / 1000000;
	clks_per_sample = (freq_mhz * actmon_status.usecs_per_sample) / 256;
	actmon_status.clks_per_sample = clks_per_sample;

	writel(host1x_sync_actmon_status_sample_period_f(clks_per_sample)
		| host1x_sync_actmon_status_status_source_f(
		host1x_sync_actmon_status_status_source_usec_v()),
		sync_regs + host1x_sync_actmon_status_r());

}

static int host1x_actmon_init(struct nvhost_master *host)
{
	void __iomem *sync_regs = host->sync_aperture;
	u32 val;
	unsigned long timeout = jiffies + msecs_to_jiffies(25);

	if (actmon_status.init == ACTMON_READY)
		return 0;

	nvhost_module_busy(host->dev);

	if (actmon_status.init == ACTMON_OFF) {
		actmon_status.usecs_per_sample = 160;
		actmon_status.above_wmark = 0;
		actmon_status.below_wmark = 0;
		actmon_status.k = 6;
	}

	/* Initialize average */
	writel(0, sync_regs + host1x_sync_actmon_init_avg_r());

	/* Default count weight - 1 for per unit actmons */
	writel(1, sync_regs + host1x_sync_actmon_count_weight_r());

	/* Wait for actmon to be disabled */
	do {
		val = readl(sync_regs + host1x_sync_actmon_status_r());
	} while (!time_after(jiffies, timeout) &&
			val & host1x_sync_actmon_status_gr3d_mon_act_f(1));

	WARN_ON(time_after(jiffies, timeout));

	/* Write (normalised) sample period. */
	host1x_actmon_update_sample_period_safe(host);

	/* Clear interrupt status */
	writel(0xffffffff, sync_regs + host1x_sync_actmon_intr_status_r());

	val = readl(sync_regs + host1x_sync_actmon_ctrl_r());
	/* Enable periodic mode */
	val |= host1x_sync_actmon_ctrl_enb_periodic_f(1);
	/* Moving avg IIR filter window size 2^6=128 */
	val |= host1x_sync_actmon_ctrl_k_val_f(actmon_status.k);
	/* Enable ACTMON */
	val |= host1x_sync_actmon_ctrl_enb_f(1);
	writel(val, sync_regs + host1x_sync_actmon_ctrl_r());

	actmon_status.init = ACTMON_READY;
	nvhost_module_idle(host->dev);
	return 0;
}

static void host1x_actmon_deinit(struct nvhost_master *host)
{
	void __iomem *sync_regs = host->sync_aperture;
	u32 val;

	if (actmon_status.init != ACTMON_READY)
		return;

	nvhost_module_busy(host->dev);

	/* Disable actmon */
	val = readl(sync_regs + host1x_sync_actmon_ctrl_r());
	val &= ~host1x_sync_actmon_ctrl_enb_m();
	val &= ~host1x_sync_actmon_ctrl_enb_periodic_m();
	val &= ~host1x_sync_actmon_ctrl_avg_above_wmark_en_m();
	val &= ~host1x_sync_actmon_ctrl_avg_below_wmark_en_m();
	writel(val, sync_regs + host1x_sync_actmon_ctrl_r());

	/*  Write sample period */
	writel(host1x_sync_actmon_status_sample_period_f(0)
		| host1x_sync_actmon_status_status_source_f(
			host1x_sync_actmon_status_status_source_usec_v()),
			sync_regs + host1x_sync_actmon_status_r());
	/* Clear interrupt status */
	writel(0xffffffff, sync_regs + host1x_sync_actmon_intr_status_r());

	actmon_status.init = ACTMON_SLEEP;
	nvhost_module_idle(host->dev);
}

static int host1x_actmon_avg(struct nvhost_master *host, u32 *val)
{
	void __iomem *sync_regs = host->sync_aperture;

	if (actmon_status.init != ACTMON_READY) {
		*val = 0;
		return 0;
	}

	nvhost_module_busy(host->dev);
	*val = readl(sync_regs + host1x_sync_actmon_avg_count_r());
	nvhost_module_idle(host->dev);
	rmb();

	return 0;
}

static int host1x_actmon_avg_norm(struct nvhost_master *host, u32 *avg)
{
	void __iomem *sync_regs = host->sync_aperture;
	long val;

	if (actmon_status.init != ACTMON_READY) {
		*avg = 0;
		return 0;
	}

	nvhost_module_busy(host->dev);
	/* Read load from hardware */
	val = readl(sync_regs + host1x_sync_actmon_avg_count_r());
	/* Undocumented feature: AVG value is not scaled. */
	*avg = (val * 1000) / ((1 + actmon_status.clks_per_sample) * 256);
	nvhost_module_idle(host->dev);
	rmb();

	return 0;
}

static int host1x_actmon_above_wmark_count(struct nvhost_master *host)
{
	return actmon_status.above_wmark;
}

static int host1x_actmon_below_wmark_count(struct nvhost_master *host)
{
	return actmon_status.below_wmark;
}

static void host1x_actmon_update_sample_period(struct nvhost_master *host)
{
	/* No sense to update actmon if actmon is inactive */
	if (actmon_status.init != ACTMON_READY)
		return;

	nvhost_module_busy(host->dev);
	host1x_actmon_update_sample_period_safe(host);
	nvhost_module_idle(host->dev);
}

static void host1x_actmon_set_sample_period_norm(struct nvhost_master *host,
							long usecs)
{
	actmon_status.usecs_per_sample = usecs;
	host1x_actmon_update_sample_period(host);
}

static void host1x_actmon_set_k(struct nvhost_master *host, u32 k)
{
	void __iomem *sync_regs = host->sync_aperture;
	long val;

	actmon_status.k = k;

	val = readl(sync_regs + host1x_sync_actmon_ctrl_r());
	val &= ~(host1x_sync_actmon_ctrl_k_val_m());
	val |= host1x_sync_actmon_ctrl_k_val_f(actmon_status.k);
	writel(val, sync_regs + host1x_sync_actmon_ctrl_r());
}

static u32 host1x_actmon_get_k(struct nvhost_master *host)
{
	return actmon_status.k;
}

static long host1x_actmon_get_sample_period(struct nvhost_master *host)
{
	return actmon_status.clks_per_sample;
}

static long host1x_actmon_get_sample_period_norm(struct nvhost_master *host)
{
	return actmon_status.usecs_per_sample;
}

static int actmon_below_wmark_show(struct seq_file *s, void *unused)
{
	struct nvhost_master *host = s->private;
	seq_printf(s, "%d\n", actmon_op().below_wmark_count(host));
	return 0;
}

static int actmon_below_wmark_open(struct inode *inode, struct file *file)
{
	return single_open(file, actmon_below_wmark_show, inode->i_private);
}

static const struct file_operations actmon_below_wmark_fops = {
	.open		= actmon_below_wmark_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int actmon_above_wmark_show(struct seq_file *s, void *unused)
{
	struct nvhost_master *host = s->private;
	seq_printf(s, "%d\n", actmon_op().above_wmark_count(host));
	return 0;
}

static int actmon_above_wmark_open(struct inode *inode, struct file *file)
{
	return single_open(file, actmon_above_wmark_show, inode->i_private);
}

static const struct file_operations actmon_above_wmark_fops = {
	.open		= actmon_above_wmark_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int actmon_avg_show(struct seq_file *s, void *unused)
{
	struct nvhost_master *host = s->private;
	u32 avg;
	int err;

	err = actmon_op().read_avg(host, &avg);
	if (!err)
		seq_printf(s, "%d\n", avg);
	return err;
}

static int actmon_avg_open(struct inode *inode, struct file *file)
{
	return single_open(file, actmon_avg_show, inode->i_private);
}

static const struct file_operations actmon_avg_fops = {
	.open		= actmon_avg_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int actmon_avg_norm_show(struct seq_file *s, void *unused)
{
	struct nvhost_master *host = s->private;
	u32 avg;
	int err;

	err = actmon_op().read_avg_norm(host, &avg);
	if (!err)
		seq_printf(s, "%d\n", avg);
	return err;
}

static int actmon_avg_norm_open(struct inode *inode, struct file *file)
{
	return single_open(file, actmon_avg_norm_show, inode->i_private);
}

static const struct file_operations actmon_avg_norm_fops = {
	.open		= actmon_avg_norm_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int actmon_sample_period_show(struct seq_file *s, void *unused)
{
	struct nvhost_master *host = s->private;
	long period = actmon_op().get_sample_period(host);
	seq_printf(s, "%ld\n", period);
	return 0;
}

static int actmon_sample_period_open(struct inode *inode, struct file *file)
{
	return single_open(file, actmon_sample_period_show, inode->i_private);
}

static const struct file_operations actmon_sample_period_fops = {
	.open		= actmon_sample_period_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int actmon_sample_period_norm_show(struct seq_file *s, void *unused)
{
	struct nvhost_master *host = s->private;
	long period = actmon_op().get_sample_period_norm(host);
	seq_printf(s, "%ld\n", period);
	return 0;
}

static int actmon_sample_period_norm_open(struct inode *inode,
						struct file *file)
{
	return single_open(file, actmon_sample_period_norm_show,
		inode->i_private);
}

static int actmon_sample_period_norm_write(struct file *file,
				const char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct nvhost_master *host = s->private;
	char buffer[40];
	int buf_size;
	unsigned long period;

	memset(buffer, 0, sizeof(buffer));
	buf_size = min(count, (sizeof(buffer)-1));

	if (copy_from_user(buffer, user_buf, buf_size))
		return -EFAULT;

	if (kstrtoul(buffer, 10, &period))
		return -EINVAL;

	actmon_op().set_sample_period_norm(host, period);

	return count;
}

static const struct file_operations actmon_sample_period_norm_fops = {
	.open		= actmon_sample_period_norm_open,
	.read		= seq_read,
	.write          = actmon_sample_period_norm_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};



static int actmon_k_show(struct seq_file *s, void *unused)
{
	struct nvhost_master *host = s->private;
	long period = actmon_op().get_k(host);
	seq_printf(s, "%ld\n", period);
	return 0;
}

static int actmon_k_open(struct inode *inode, struct file *file)
{
	return single_open(file, actmon_k_show, inode->i_private);
}

static int actmon_k_write(struct file *file,
				const char __user *user_buf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct nvhost_master *host = s->private;
	char buffer[40];
	int buf_size;
	unsigned long k;

	memset(buffer, 0, sizeof(buffer));
	buf_size = min(count, (sizeof(buffer)-1));

	if (copy_from_user(buffer, user_buf, buf_size))
		return -EFAULT;

	if (kstrtoul(buffer, 10, &k))
		return -EINVAL;

	actmon_op().set_k(host, k);

	return count;
}

static const struct file_operations actmon_k_fops = {
	.open		= actmon_k_open,
	.read		= seq_read,
	.write          = actmon_k_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void host1x_actmon_debug_init(struct nvhost_master *master,
				     struct dentry *de)
{
	debugfs_create_file("3d_actmon_k", S_IRUGO, de,
			master, &actmon_k_fops);
	debugfs_create_file("3d_actmon_sample_period", S_IRUGO, de,
			master, &actmon_sample_period_fops);
	debugfs_create_file("3d_actmon_sample_period_norm", S_IRUGO, de,
			master, &actmon_sample_period_norm_fops);
	debugfs_create_file("3d_actmon_avg_norm", S_IRUGO, de,
			master, &actmon_avg_norm_fops);
	debugfs_create_file("3d_actmon_avg", S_IRUGO, de,
			master, &actmon_avg_fops);
	debugfs_create_file("3d_actmon_above_wmark", S_IRUGO, de,
			master, &actmon_above_wmark_fops);
	debugfs_create_file("3d_actmon_below_wmark", S_IRUGO, de,
			master, &actmon_below_wmark_fops);
}

static const struct nvhost_actmon_ops host1x_actmon_ops = {
	.init = host1x_actmon_init,
	.deinit = host1x_actmon_deinit,
	.read_avg = host1x_actmon_avg,
	.above_wmark_count = host1x_actmon_above_wmark_count,
	.below_wmark_count = host1x_actmon_below_wmark_count,
	.read_avg_norm = host1x_actmon_avg_norm,
	.update_sample_period = host1x_actmon_update_sample_period,
	.set_sample_period_norm = host1x_actmon_set_sample_period_norm,
	.get_sample_period_norm = host1x_actmon_get_sample_period_norm,
	.get_sample_period = host1x_actmon_get_sample_period,
	.get_k = host1x_actmon_get_k,
	.set_k = host1x_actmon_set_k,
	.debug_init = host1x_actmon_debug_init,
};
