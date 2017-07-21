/*
 * adsp_dfs.c
 *
 * adsp dynamic frequency scaling
 *
 * Copyright (C) 2014-2017, NVIDIA Corporation. All rights reserved.
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

#include <linux/tegra_nvadsp.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/clk/tegra.h>
#include <linux/seq_file.h>
#include <asm/cputime.h>
#include <linux/slab.h>

#include "dev.h"
#include "ape_actmon.h"
#include "os.h"

#ifndef CONFIG_TEGRA_ADSP_ACTMON
void actmon_rate_change(unsigned long freq, bool override)
{

}
#endif

#define MBOX_TIMEOUT 5000 /* in ms */
#define HOST_ADSP_DFS_MBOX_ID 3

enum adsp_dfs_reply {
	ACK,
	NACK,
};

/*
 * Freqency in Hz.The frequency always needs to be a multiple of 12.8 Mhz and
 * should be extended with a slab 38.4 Mhz.
 */
static unsigned long adsp_cpu_freq_table[] = {
	MIN_ADSP_FREQ,
	MIN_ADSP_FREQ * 2,
	MIN_ADSP_FREQ * 3,
	MIN_ADSP_FREQ * 4,
	MIN_ADSP_FREQ * 5,
	MIN_ADSP_FREQ * 6,
	MIN_ADSP_FREQ * 7,
	MIN_ADSP_FREQ * 8,
	MIN_ADSP_FREQ * 9,
	MIN_ADSP_FREQ * 10,
	MIN_ADSP_FREQ * 11,
	MIN_ADSP_FREQ * 12,
	MIN_ADSP_FREQ * 13,
	MIN_ADSP_FREQ * 14,
	MIN_ADSP_FREQ * 15,
	MIN_ADSP_FREQ * 16,
	MIN_ADSP_FREQ * 17,
	MIN_ADSP_FREQ * 18,
	MIN_ADSP_FREQ * 19,
	MIN_ADSP_FREQ * 20,
	MIN_ADSP_FREQ * 21,
};

struct adsp_dfs_policy {
	bool enable;
/* update_freq_flag = TRUE, ADSP ACKed the new freq
 *		= FALSE, ADSP NACKed the new freq
 */
	bool update_freq_flag;

	const char *clk_name;
	unsigned long min;    /* in kHz */
	unsigned long max;    /* in kHz */
	unsigned long cur;    /* in kHz */
	unsigned long cpu_min;    /* ADSP min freq(KHz). Remain unchanged */
	unsigned long cpu_max;    /* ADSP max freq(KHz). Remain unchanged */

	struct clk *adsp_clk;
	struct clk *aclk_clk;
	struct notifier_block rate_change_nb;
	struct nvadsp_mbox mbox;

#ifdef CONFIG_DEBUG_FS
	struct dentry *root;
#endif
	unsigned long ovr_freq;
};

struct adsp_freq_stats {
	struct device *dev;
	unsigned long long last_time;
	int last_index;
	u64 time_in_state[sizeof(adsp_cpu_freq_table) \
		/ sizeof(adsp_cpu_freq_table[0])];
	int state_num;
};

static struct adsp_dfs_policy *policy;
static struct adsp_freq_stats freq_stats;
static struct device *device;
static struct clk *ape_emc_clk;


static DEFINE_MUTEX(policy_mutex);

static bool is_os_running(struct device *dev)
{
	struct platform_device *pdev;
	struct nvadsp_drv_data *drv_data;

	if (!dev)
		return false;

	pdev = to_platform_device(dev);
	drv_data = platform_get_drvdata(pdev);

	if (!drv_data->adsp_os_running) {
		dev_dbg(&pdev->dev, "%s: adsp os is not loaded\n", __func__);
		return false;
	}
	return true;
}

static int adsp_clk_get(struct adsp_dfs_policy *policy)
{
	struct device_node *node = device->of_node;
	int ret = 0;

	if (IS_ENABLED(CONFIG_COMMON_CLK))
		policy->adsp_clk = devm_clk_get(device, "adsp");
	else
		policy->adsp_clk = clk_get_sys(NULL, policy->clk_name);

	if (IS_ERR_OR_NULL(policy->adsp_clk)) {
		dev_err(device, "unable to find adsp clock\n");
		ret = PTR_ERR(policy->adsp_clk);
	}

	if (!of_device_is_compatible(node, "nvidia,tegra210-adsp")) {
		policy->aclk_clk = devm_clk_get(device, "aclk");

		if (IS_ERR_OR_NULL(policy->aclk_clk)) {
			dev_err(device, "unable to find aclk clock\n");
			ret = PTR_ERR(policy->aclk_clk);
		}
	}

	return ret;
}

static void adsp_clk_put(struct adsp_dfs_policy *policy)
{
	if (policy->adsp_clk) {
		if (IS_ENABLED(CONFIG_COMMON_CLK))
			devm_clk_put(device, policy->adsp_clk);
		else
			clk_put(policy->adsp_clk);
	}

	if (policy->aclk_clk)
		devm_clk_put(device, policy->aclk_clk);
}

static int adsp_clk_set_rate(struct adsp_dfs_policy *policy,
			     unsigned long freq_hz)
{
	struct device_node *node = device->of_node;
	int ret;

	if (of_device_is_compatible(node, "nvidia,tegra210-adsp"))
		ret = clk_set_rate(policy->adsp_clk, freq_hz);
	else
		ret = clk_set_rate(policy->aclk_clk, freq_hz);

	return ret;
}

static unsigned long adsp_clk_get_rate(struct adsp_dfs_policy *policy)
{
	return clk_get_rate(policy->adsp_clk);
}

/* Expects and returns freq in Hz as table is formmed in terms of Hz */
static unsigned long adsp_get_target_freq(unsigned long tfreq, int *index)
{
	int i;
	int size = sizeof(adsp_cpu_freq_table) / sizeof(adsp_cpu_freq_table[0]);

	if (tfreq <= adsp_cpu_freq_table[0]) {
		*index = 0;
		return adsp_cpu_freq_table[0];
	}

	if (tfreq >= adsp_cpu_freq_table[size - 1]) {
		*index = size - 1;
		return adsp_cpu_freq_table[size - 1];
	}

	for (i = 1; i < size; i++) {
		if ((tfreq <= adsp_cpu_freq_table[i]) &&
				(tfreq > adsp_cpu_freq_table[i - 1])) {
			*index = i;
			return adsp_cpu_freq_table[i];
		}
	}

	return 0;
}

static void adspfreq_stats_update(void)
{
	unsigned long long cur_time;

	cur_time = get_jiffies_64();
	freq_stats.time_in_state[freq_stats.last_index] += cur_time -
		freq_stats.last_time;
	freq_stats.last_time = cur_time;
}

/* adsp clock rate change notifier callback */
static int adsp_dfs_rc_callback(
	struct notifier_block *nb, unsigned long rate, void *v)
{
	unsigned long freq = rate / 1000;
	int old_index, new_index = 0;

	/* update states */
	adspfreq_stats_update();

	old_index = freq_stats.last_index;
	adsp_get_target_freq(rate, &new_index);
	if (old_index != new_index)
		freq_stats.last_index = new_index;

	if (policy->ovr_freq && freq == policy->ovr_freq) {
		/* Re-init ACTMON when user requested override freq is met */
		actmon_rate_change(freq, true);
		policy->ovr_freq = 0;
	} else
		actmon_rate_change(freq, false);

	return NOTIFY_OK;
};

static struct adsp_dfs_policy dfs_policy =  {
	.enable = 1,
	.clk_name = "adsp_cpu",
	.rate_change_nb = {
		.notifier_call = adsp_dfs_rc_callback,
	},
};

static int adsp_update_freq_handshake(unsigned long tfreq_hz, int index)
{
	struct nvadsp_mbox *mbx = &policy->mbox;
	enum adsp_dfs_reply reply;
	int ret;

	dev_dbg(device, "sending change in freq(hz):%lu\n", tfreq_hz);
	/*
	 * Ask adsp to do action upon change in freq. ADSP and Host need to
	 * maintain the same freq table.
	 */
	ret = nvadsp_mbox_send(mbx, index,
				NVADSP_MBOX_SMSG, true, 100);
	if (ret) {
		dev_err(device, "%s:host to adsp, mbox_send failure. ret:%d\n",
			__func__, ret);
		policy->update_freq_flag = false;
		goto err_out;
	}

	ret = nvadsp_mbox_recv(&policy->mbox, &reply, true, MBOX_TIMEOUT);
	if (ret) {
		dev_err(device, "%s:host to adsp, mbox_receive failure. ret:%d\n",
			__func__, ret);
		policy->update_freq_flag = false;
		goto err_out;
	}

	switch (reply) {
	case ACK:
		/* Set Update freq flag */
		dev_dbg(device, "adsp freq change status:ACK\n");
		policy->update_freq_flag = true;
		break;
	case NACK:
		/* Set Update freq flag */
		dev_dbg(device, "adsp freq change status:NACK\n");
		policy->update_freq_flag = false;
		break;
	default:
		dev_err(device, "Error: adsp freq change status\n");
	}

	dev_dbg(device, "%s:status received from adsp: %s, tfreq(hz):%lu\n",
		__func__,
		policy->update_freq_flag == true ? "ACK" : "NACK",
		tfreq_hz);
err_out:
	return ret;
}

/*
 * update_freq - update adsp freq and ask adsp to change timer as
 * change in adsp freq.
 * freq_khz - target frequency in KHz
 * return - final freq got set.
 *		- 0, incase of error.
 *
 * Note - Policy->cur would be updated via rate
 * change notifier, when freq is changed in hw
 *
 */
static unsigned long update_freq(unsigned long freq_khz)
{
	struct nvadsp_drv_data *drv = dev_get_drvdata(device);
	unsigned long tfreq_hz, old_freq_khz;
	u32 efreq;
	int index;
	int ret;

	if (!is_os_running(device)) {
		dev_err(device, "adsp os is not running\n");
		return 0;
	}

	tfreq_hz = adsp_get_target_freq(freq_khz * 1000, &index);
	if (!tfreq_hz) {
		dev_err(device, "unable get the target freq\n");
		return 0;
	}

	old_freq_khz = policy->cur;

	if ((tfreq_hz / 1000) == old_freq_khz) {
		dev_dbg(device, "old and new target_freq is same\n");
		return 0;
	}

	ret = adsp_clk_set_rate(policy, tfreq_hz);
	if (ret) {
		dev_err(device, "failed to set adsp freq:%luhz err:%d\n",
			tfreq_hz, ret);
		policy->update_freq_flag = false;
		return 0;
	}

	efreq = adsp_to_emc_freq(tfreq_hz / 1000);

	if (IS_ENABLED(CONFIG_COMMON_CLK)) {
		tegra_bwmgr_set_emc(drv->bwmgr, efreq * 1000,
				    TEGRA_BWMGR_SET_EMC_FLOOR);
	} else {
		ret = clk_set_rate(ape_emc_clk, efreq * 1000);
		if (ret) {
			dev_err(device, "failed to set ape.emc clk:%d\n", ret);
			policy->update_freq_flag = false;
			goto err_out;
		}
	}

	/*
	 * On tegra > t210, as os_args->adsp_freq_hz is used to know adsp cpu
	 * clk rate and there is no need to set up timer prescalar. So skip
	 * communicating adsp cpu clk rate update to adspos using mbox
	 */
	if (!of_device_is_compatible(device->of_node, "nvidia,tegra210-adsp"))
		policy->update_freq_flag = true;
	else
		adsp_update_freq_handshake(tfreq_hz, index);

	/*
	 * Use os_args->adsp_freq_hz to update adsp cpu clk rate
	 * for adspos firmware, which uses this shared variable
	 * to get the clk rate for EDF, etc.
	 */
	if (policy->update_freq_flag) {
		struct nvadsp_shared_mem *sm = drv->shared_adsp_os_data;

		sm->os_args.adsp_freq_hz = tfreq_hz;
	}

err_out:
	if (!policy->update_freq_flag) {
		ret = adsp_clk_set_rate(policy, old_freq_khz * 1000);
		if (ret) {
			dev_err(device, "failed to resume adsp freq(khz):%lu\n",
				old_freq_khz);
			policy->update_freq_flag = false;
		}

		efreq = adsp_to_emc_freq(old_freq_khz);
		if (IS_ENABLED(CONFIG_COMMON_CLK)) {
			tegra_bwmgr_set_emc(drv->bwmgr, efreq * 1000,
					    TEGRA_BWMGR_SET_EMC_FLOOR);
		} else {
			ret = clk_set_rate(ape_emc_clk, efreq * 1000);
			if (ret) {
				dev_err(device,
					"failed to set ape.emc clk:%d\n", ret);
				policy->update_freq_flag = false;
			}
		}

		tfreq_hz = old_freq_khz * 1000;
	}
	return tfreq_hz / 1000;
}

/* Set adsp dfs policy min freq(Khz) */
static int policy_min_set(void *data, u64 val)
{
	int ret = -EINVAL;
	unsigned long min = (unsigned long)val;

	if (!is_os_running(device))
		return ret;

	mutex_lock(&policy_mutex);
	if (!policy->enable) {
		dev_err(device, "adsp dfs policy is not enabled\n");
		goto exit_out;
	}

	if (min == policy->min)
		goto exit_out;
	else if (min < policy->cpu_min)
		min = policy->cpu_min;
	else if (min >= policy->cpu_max)
		min = policy->cpu_max;

	if (min > policy->cur) {
		min = update_freq(min);
		if (min)
			policy->cur = min;
	}

	if (min)
		policy->min = min;

	ret = 0;
exit_out:
	mutex_unlock(&policy_mutex);
	return ret;
}

#ifdef CONFIG_DEBUG_FS

#define RW_MODE (S_IWUSR | S_IRUSR)
#define RO_MODE S_IRUSR

/* Get adsp dfs staus: 0: disabled, 1: enabled */
static int dfs_enable_get(void *data, u64 *val)
{
	mutex_lock(&policy_mutex);
	*val = policy->enable;
	mutex_unlock(&policy_mutex);

	return 0;
}

/* Enable/disable adsp dfs */
static int dfs_enable_set(void *data, u64 val)
{
	mutex_lock(&policy_mutex);
	policy->enable = (bool) val;
	mutex_unlock(&policy_mutex);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(enable_fops, dfs_enable_get,
	dfs_enable_set, "%llu\n");

/* Get adsp dfs policy min freq(KHz) */
static int policy_min_get(void *data, u64 *val)
{
	if (!is_os_running(device))
		return -EINVAL;

	mutex_lock(&policy_mutex);
	*val = policy->min;
	mutex_unlock(&policy_mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(min_fops, policy_min_get,
	policy_min_set, "%llu\n");

/* Get adsp dfs policy max freq(KHz) */
static int policy_max_get(void *data, u64 *val)
{
	if (!is_os_running(device))
		return -EINVAL;

	mutex_lock(&policy_mutex);
	*val = policy->max;
	mutex_unlock(&policy_mutex);
	return 0;
}

/* Set adsp dfs policy max freq(KHz) */
static int policy_max_set(void *data, u64 val)
{
	int ret = -EINVAL;
	unsigned long max = (unsigned long)val;

	if (!is_os_running(device))
		return ret;

	mutex_lock(&policy_mutex);
	if (!policy->enable) {
		dev_err(device, "adsp dfs policy is not enabled\n");
		goto exit_out;
	}

	if (!max || ((max > policy->cpu_max) || (max == policy->max)))
		goto exit_out;

	else if (max <=  policy->cpu_min)
		max = policy->cpu_min;

	if (max < policy->cur)
		max = update_freq(max);

	if (max)
		policy->cur = policy->max = max;
	ret = 0;
exit_out:
	mutex_unlock(&policy_mutex);
	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(max_fops, policy_max_get,
	policy_max_set, "%llu\n");

/* Get adsp dfs policy's current freq */
static int policy_cur_get(void *data, u64 *val)
{
	if (!is_os_running(device))
		return -EINVAL;

	mutex_lock(&policy_mutex);
	*val = policy->cur;
	mutex_unlock(&policy_mutex);

	return 0;
}

/* Set adsp dfs policy cur freq(Khz) */
static int policy_cur_set(void *data, u64 val)
{
	int ret = -EINVAL;
	unsigned long cur = (unsigned long)val;

	if (!is_os_running(device))
		return ret;

	mutex_lock(&policy_mutex);
	if (policy->enable) {
		dev_err(device, "adsp dfs is enabled, should be disabled first\n");
		goto exit_out;
	}

	if (!cur || cur == policy->cur)
		goto exit_out;

	/* Check tfreq policy sanity */
	if (cur < policy->min)
		cur = policy->min;
	else if (cur > policy->max)
		cur = policy->max;

	cur = update_freq(cur);
	if (cur)
		policy->cur = cur;
	ret = 0;
exit_out:
	mutex_unlock(&policy_mutex);
	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(cur_fops, policy_cur_get,
	policy_cur_set, "%llu\n");

/*
 * Print residency in each freq levels
 */
static void dump_stats_table(struct seq_file *s, struct adsp_freq_stats *fstats)
{
	int i;

	mutex_lock(&policy_mutex);
	if (is_os_running(device))
		adspfreq_stats_update();

	for (i = 0; i < fstats->state_num; i++) {
		seq_printf(s, "%lu %llu\n",
			(long unsigned int)(adsp_cpu_freq_table[i] / 1000),
			cputime64_to_clock_t(fstats->time_in_state[i]));
	}
	mutex_unlock(&policy_mutex);
}

static int show_time_in_state(struct seq_file *s, void *data)
{
	struct adsp_freq_stats *fstats =
		(struct adsp_freq_stats *) (s->private);

	dump_stats_table(s, fstats);
	return 0;
}

static int stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_time_in_state, inode->i_private);
}

static const struct file_operations time_in_state_fops = {
	.open = stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int adsp_dfs_debugfs_init(struct platform_device *pdev)
{
	int ret = -ENOMEM;
	struct dentry *d, *root;
	struct nvadsp_drv_data *drv = platform_get_drvdata(pdev);

	if (!drv->adsp_debugfs_root)
		return ret;

	root = debugfs_create_dir("adsp_dfs", drv->adsp_debugfs_root);
	if (!root)
		return ret;

	policy->root = root;

	d = debugfs_create_file("enable", RW_MODE, root, NULL,
		&enable_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("min_freq", RW_MODE, root, NULL,
		&min_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("max_freq", RW_MODE, root,
		NULL, &max_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("cur_freq", RW_MODE, root, NULL,
		&cur_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("time_in_state", RO_MODE,
					root, &freq_stats,
					&time_in_state_fops);
	if (!d)
		goto err_out;

	return 0;

err_out:
	debugfs_remove_recursive(root);
	policy->root = NULL;
	dev_err(&pdev->dev,
	"unable to create adsp logger debug fs file\n");
	return ret;
}
#endif

/*
 * Set target freq.
 * @params:
 * freq: adsp freq in KHz
 */
void adsp_cpu_set_rate(unsigned long freq)
{
	mutex_lock(&policy_mutex);

	if (!policy->enable) {
		dev_dbg(device, "adsp dfs policy is not enabled\n");
		goto exit_out;
	}

	if (freq < policy->min)
		freq = policy->min;
	else if (freq > policy->max)
		freq = policy->max;

	freq = update_freq(freq);
	if (freq)
		policy->cur = freq;
exit_out:
	mutex_unlock(&policy_mutex);
}

/*
 * Override adsp freq and reinit actmon counters
 *
 * @params:
 * freq: adsp freq in KHz
 * return - final freq set
 *	  - 0 incase of error
 *
 */
unsigned long adsp_override_freq(unsigned long req_freq_khz)
{
	unsigned long ret_freq = 0, freq;
	int index;

	if (!is_os_running(device)) {
		pr_err("%s: adsp os is not in running state.\n", __func__);
		return 0;
	}

	mutex_lock(&policy_mutex);

	freq = req_freq_khz;

	if (freq < policy->min)
		freq = policy->min;
	else if (freq > policy->max)
		freq = policy->max;

	freq = adsp_get_target_freq(freq * 1000, &index);
	if (!freq) {
		dev_warn(device,
			 "req freq:%lukhz. unable get the target freq.\n",
			 req_freq_khz);
		goto exit_out;
	}
	freq = freq / 1000; /* In KHz */

	if (freq == policy->cur) {
		ret_freq = freq;
		goto exit_out;
	}

	policy->ovr_freq = freq;
	ret_freq = update_freq(freq);
	if (ret_freq)
		policy->cur = ret_freq;

	if (ret_freq != freq) {
		dev_warn(device,
			 "req freq:%lukhz. freq override to %lukhz rejected.\n",
			 req_freq_khz, freq);
		policy->ovr_freq = 0;
		goto exit_out;
	}

exit_out:
	mutex_unlock(&policy_mutex);
	return ret_freq;
}
EXPORT_SYMBOL(adsp_override_freq);

/*
 * Set min ADSP freq.
 *
 * @params:
 * freq: adsp freq in KHz
 */
void adsp_update_dfs_min_rate(unsigned long freq)
{
	policy_min_set(NULL, freq);
}
EXPORT_SYMBOL(adsp_update_dfs_min_rate);

/* Enable / disable dynamic freq scaling */
void adsp_update_dfs(bool val)
{
	mutex_lock(&policy_mutex);
	policy->enable = val;
	mutex_unlock(&policy_mutex);
}

/* Should be called after ADSP os is loaded */
int adsp_dfs_core_init(struct platform_device *pdev)
{
	int size = sizeof(adsp_cpu_freq_table) / sizeof(adsp_cpu_freq_table[0]);
	struct nvadsp_drv_data *drv = platform_get_drvdata(pdev);
	uint16_t mid = HOST_ADSP_DFS_MBOX_ID;
	int ret = 0;
	u32 efreq;

	if (drv->dfs_initialized)
		return 0;

	device = &pdev->dev;
	policy = &dfs_policy;

	ret = adsp_clk_get(policy);
	if (ret)
		goto end;

	if (IS_ENABLED(CONFIG_COMMON_CLK)) {
		drv->bwmgr = tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_APE_ADSP);
		if (IS_ERR_OR_NULL(drv->bwmgr)) {
			dev_err(&pdev->dev, "unable to register bwmgr\n");
			ret = PTR_ERR(drv->bwmgr);
			goto end;
		}
	} else {
		/* Change emc freq as per the adsp to emc lookup table */
		ape_emc_clk = clk_get_sys("ape", "emc");
		if (IS_ERR_OR_NULL(ape_emc_clk)) {
			dev_err(device, "unable to find ape.emc clock\n");
			ret = PTR_ERR(ape_emc_clk);
			goto end;
		}

		ret = clk_prepare_enable(ape_emc_clk);
		if (ret) {
			dev_err(device, "unable to enable ape.emc clock\n");
			goto end;
		}
	}

	policy->max = policy->cpu_max = drv->adsp_freq; /* adsp_freq in KHz */

	policy->min = policy->cpu_min = adsp_cpu_freq_table[0] / 1000;

	policy->cur = adsp_clk_get_rate(policy) / 1000;

	efreq = adsp_to_emc_freq(policy->cur);

	if (IS_ENABLED(CONFIG_COMMON_CLK)) {
		tegra_bwmgr_set_emc(drv->bwmgr, efreq * 1000,
				    TEGRA_BWMGR_SET_EMC_FLOOR);
	} else {
		ret = clk_set_rate(ape_emc_clk, efreq * 1000);
		if (ret) {
			dev_err(device, "failed to set ape.emc clk:%d\n", ret);
			goto end;
		}
	}

	adsp_get_target_freq(policy->cur * 1000, &freq_stats.last_index);
	freq_stats.last_time = get_jiffies_64();
	freq_stats.state_num = size;
	freq_stats.dev = &pdev->dev;
	memset(&freq_stats.time_in_state, 0, sizeof(freq_stats.time_in_state));

	ret = nvadsp_mbox_open(&policy->mbox, &mid, "dfs_comm", NULL, NULL);
	if (ret) {
		dev_info(&pdev->dev, "unable to open mailbox. ret:%d\n", ret);
		goto end;
	}

#if !defined(CONFIG_COMMON_CLK)
	if (policy->rate_change_nb.notifier_call) {
		/*
		 * "adsp_cpu" clk is a shared user of parent adsp_cpu_bus clk;
		 * rate change notification should come from bus clock itself.
		 */
		struct clk *p = clk_get_parent(policy->adsp_clk);
		if (!p) {
			dev_err(&pdev->dev, "Failed to find adsp cpu parent clock\n");
			ret = -EINVAL;
			goto end;
		}

		ret = tegra_register_clk_rate_notifier(p,
			&policy->rate_change_nb);
		if (ret) {
			dev_err(&pdev->dev, "rate change notifier err: %s\n",
			policy->clk_name);
			nvadsp_mbox_close(&policy->mbox);
			goto end;
		}
	}
#endif

#ifdef CONFIG_DEBUG_FS
	adsp_dfs_debugfs_init(pdev);
#endif
	drv->dfs_initialized = true;

	dev_dbg(&pdev->dev, "adsp dfs initialized ....\n");
	return ret;
end:

	adsp_clk_put(policy);

	if (IS_ENABLED(CONFIG_COMMON_CLK) && drv->bwmgr) {
		tegra_bwmgr_set_emc(drv->bwmgr, 0,
				    TEGRA_BWMGR_SET_EMC_FLOOR);
		tegra_bwmgr_unregister(drv->bwmgr);
	} else if (ape_emc_clk) {
		clk_disable_unprepare(ape_emc_clk);
		clk_put(ape_emc_clk);
	}

	return ret;
}

int adsp_dfs_core_exit(struct platform_device *pdev)
{
	status_t ret = 0;
	struct nvadsp_drv_data *drv = platform_get_drvdata(pdev);

	/* return if dfs is not initialized */
	if (!drv->dfs_initialized)
		return -ENODEV;

	ret = nvadsp_mbox_close(&policy->mbox);
	if (ret)
		dev_info(&pdev->dev,
		"adsp dfs exit failed: mbox close error. ret:%d\n", ret);

#if !defined(CONFIG_COMMON_CLK)
	tegra_unregister_clk_rate_notifier(clk_get_parent(policy->adsp_clk),
					   &policy->rate_change_nb);
#endif
	adsp_clk_put(policy);

	if (IS_ENABLED(CONFIG_COMMON_CLK) && drv->bwmgr) {
		tegra_bwmgr_set_emc(drv->bwmgr, 0,
				    TEGRA_BWMGR_SET_EMC_FLOOR);
		tegra_bwmgr_unregister(drv->bwmgr);
	} else if (ape_emc_clk) {
		clk_disable_unprepare(ape_emc_clk);
		clk_put(ape_emc_clk);
	}

	drv->dfs_initialized = false;
	dev_dbg(&pdev->dev, "adsp dfs has exited ....\n");

	return ret;
}
