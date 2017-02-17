/*
 * Copyright (c) 2015-2017 NVIDIA CORPORATION. All rights reserved.
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

#include <linux/tegra-camera-rtcpu.h>

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/irqchip/tegra-agic.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/seq_buf.h>
#include <linux/slab.h>
#include <linux/tegra-firmwares.h>
#include <linux/tegra-hsp.h>
#include <linux/tegra-ivc-bus.h>
#include <linux/tegra_pm_domains.h>
#include <linux/tegra-rtcpu-monitor.h>
#include <linux/tegra-rtcpu-trace.h>
#include <linux/wait.h>

#include <dt-bindings/memory/tegra-swgroup.h>

#include "soc/tegra/camrtc-commands.h"
#include "soc/tegra/camrtc-ctrl-commands.h"

#ifndef RTCPU_FW_SM3_VERSION
#define RTCPU_FW_SM3_VERSION (3)
#endif

enum tegra_cam_rtcpu_id {
	TEGRA_CAM_RTCPU_SCE,
	TEGRA_CAM_RTCPU_APE,
};

struct tegra_cam_rtcpu_pdata {
	const char *name;
	int (*suspend_core)(struct device *);
	int (*check_fw)(struct device *);
	void (*config_core)(struct device *);
	const char * const *clock_names;
	const struct rtcpu_clock_rates {
		u32 slow, fast;
	} *clock_rates;
	const char * const *reset_names;
	const char * const *reg_names;
	const char * const *irq_names;
	enum tegra_cam_rtcpu_id id;
	u32 num_clocks;
	u32 num_resets;
	u32 num_regs;
	u32 num_irqs;
};

/* Register specifics */
#define TEGRA_APS_FRSC_SC_CTL_0			0x0
#define TEGRA_APS_FRSC_SC_MODEIN_0		0x14
#define TEGRA_PM_R5_CTRL_0			0x40
#define TEGRA_PM_PWR_STATUS_0			0x20

#define TEGRA_R5R_SC_DISABLE			0x5
#define TEGRA_FN_MODEIN				0x29527
#define TEGRA_PM_FWLOADDONE			0x2
#define TEGRA_PM_WFIPIPESTOPPED			0x200000

#define AMISC_ADSP_STATUS			0x14
#define AMISC_ADSP_L2_IDLE			BIT(31)
#define AMISC_ADSP_L2_CLKSTOPPED		BIT(30)

static int tegra_sce_cam_suspend_core(struct device *dev);
static int tegra_sce_cam_check_fw(struct device *dev);
static void tegra_sce_cam_config_core(struct device *dev);
static int tegra_ape_cam_suspend_core(struct device *dev);
static irqreturn_t tegra_camrtc_adsp_wfi_handler(int irq, void *data);
static int tegra_cam_rtcpu_runtime_resume(struct device *dev);

static const char * const sce_clock_names[] = {
	"sce-apb",
	"sce-cpu-nic",
};

static const struct rtcpu_clock_rates
sce_clock_rates[ARRAY_SIZE(sce_clock_names)] = {
	/* Slow rate (RTPM),  fast rate */
	{ .slow = 102000000, .fast = 102000000, },
	{ .slow = 115200000, .fast = 473600000, },
};

static const char * const sce_reset_names[] = {
	/* Group 1 */
	"tsctnsce",
	"sce-pm",
	"sce-dbgresetn",
	"sce-presetdbgn",
	"sce-actmon",
	"sce-dma",
	"sce-tke",
	"sce-gte",
	"sce-cfg",
	/* Group 2: nSYSPORRESET, nRESET */
	"sce-nreset",
	"sce-nsysporeset",
};

struct sce_resets {
	struct reset_control *tsctnsce;
	struct reset_control *sce_pm;
	struct reset_control *sce_dbgresetn;
	struct reset_control *sce_presetdbgn;
	struct reset_control *sce_actmon;
	struct reset_control *sce_dma;
	struct reset_control *sce_tke;
	struct reset_control *sce_gte;
	struct reset_control *sce_cfg;
	/* Group 2: nSYSPORRESET, nRESET */
	struct reset_control *sce_nreset;
	struct reset_control *sce_nsysporeset;
};

static const char * const sce_reg_names[] = {
	"sce-cfg",
	"sce-pm",
	"sce-fw",
};

static const char * const sce_irq_names[] = {
};

static const struct tegra_cam_rtcpu_pdata sce_pdata = {
	.name = "sce",
	.suspend_core = tegra_sce_cam_suspend_core,
	.check_fw = tegra_sce_cam_check_fw,
	.config_core = tegra_sce_cam_config_core,
	.clock_names = sce_clock_names,
	.clock_rates = sce_clock_rates,
	.reset_names = sce_reset_names,
	.id = TEGRA_CAM_RTCPU_SCE,
	.num_clocks = ARRAY_SIZE(sce_clock_names),
	.num_resets = ARRAY_SIZE(sce_reset_names),
	.reg_names = sce_reg_names,
	.num_regs = ARRAY_SIZE(sce_reg_names),
	.irq_names = sce_irq_names,
	.num_irqs = ARRAY_SIZE(sce_irq_names),
};

static const char * const ape_reg_names[] = {
	"ape-amisc",
};

static const char * const ape_clock_names[] = {
	"apb2ape",
	"ape",
	"adsp",
	"adspneon",
};

static const struct rtcpu_clock_rates
ape_clock_rates[ARRAY_SIZE(ape_clock_names)] = {
	/* Slow rate (RTPM),  fast rate */
	{ .slow = 0, .fast = 0, },
	{ .slow = 0, .fast = 0, },
	{ .slow = 0, .fast = 0, },
	{ .slow = 0, .fast = 0, },
};

static const char * const ape_reset_names[] = {
	"adsp-all"
};

static const char * const ape_irq_names[] = {
	"adsp-wfi",
};

static const struct tegra_cam_rtcpu_pdata ape_pdata = {
	.name = "ape",
	.suspend_core = tegra_ape_cam_suspend_core,
	.clock_names = ape_clock_names,
	.clock_rates = ape_clock_rates,
	.reset_names = ape_reset_names,
	.id = TEGRA_CAM_RTCPU_APE,
	.num_clocks = ARRAY_SIZE(ape_clock_names),
	.num_resets = ARRAY_SIZE(ape_reset_names),
	.reg_names = ape_reg_names,
	.num_regs = ARRAY_SIZE(ape_reg_names),
	.irq_names = ape_irq_names,
	.num_irqs = ARRAY_SIZE(ape_irq_names),
};

#define NV(p) "nvidia," #p
#define MAX(x, y) (x > y ? x : y)
#define NUM(names) MAX(ARRAY_SIZE(sce_##names), ARRAY_SIZE(ape_##names))

struct tegra_cam_rtcpu {
	const char *name;
	struct tegra_ivc_bus *ivc;
	struct tegra_hsp_sm_pair *sm_pair;
	struct tegra_rtcpu_trace *tracer;
	struct {
		struct mutex mutex;
		wait_queue_head_t response_waitq;
		wait_queue_head_t empty_waitq;
		atomic_t response;
		u32 timeout;
	} cmd;
	u32 fw_version;
	u8 fw_hash[RTCPU_FW_HASH_SIZE];
	union {
		void __iomem *regs[NUM(reg_names)];
		struct {
			void __iomem *cfg_base;
			void __iomem *pm_base;
			void __iomem *fw_base;
		};
		struct {
			void __iomem *amisc_base;
		};
	};
	struct clk *clocks[NUM(clock_names)];
	union {
		struct sce_resets sce_resets;
		struct reset_control *resets[NUM(reset_names)];
	};
	union {
		int irqs[NUM(irq_names)];
		struct {
			int adsp_wfi_irq;
		};
	};
	const struct tegra_cam_rtcpu_pdata *pdata;
	struct tegra_camrtc_mon *monitor;
	bool powered;
	bool boot_sync_done;
	bool online;
};

static int tegra_camrtc_mbox_exchange(struct device *dev,
				u32 command, long *timeout);

static struct reset_control *tegra_camrtc_reset_control_get(
	struct device *dev, const char *id)
{
	/*
	 * Depending on firmware, not all resets are available.
	 * Unavailable resets are removed from device tree but
	 * reset_control_get() returns -EPROBE_DEFER for them.
	 */
	if (of_property_match_string(dev->of_node, "reset-names", id) < 0)
		return ERR_PTR(-ENOENT);
	return devm_reset_control_get(dev, id);
}

static void __iomem *tegra_cam_ioremap(struct device *dev, int index)
{
	struct resource mem;
	int err = of_address_to_resource(dev->of_node, index, &mem);
	if (err)
		return IOMEM_ERR_PTR(err);

	/* NOTE: assumes size is large enough for caller */
	return devm_ioremap_resource(dev, &mem);
}

static void __iomem *tegra_cam_ioremap_byname(struct device *dev,
					const char *name)
{
	int index = of_property_match_string(dev->of_node, "reg-names", name);
	if (index < 0)
		return IOMEM_ERR_PTR(-ENOENT);
	return tegra_cam_ioremap(dev, index);
}

static int tegra_camrtc_get_resources(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	const struct tegra_cam_rtcpu_pdata *pdata = rtcpu->pdata;
	int i, err;

#define GET_RESOURCES(_res_, _get_, _warn_, _toerr) \
	for (i = 0; i < pdata->num_##_res_##s; i++) { \
		rtcpu->_res_##s[i] = _get_(dev, pdata->_res_##_names[i]); \
		err = _toerr(rtcpu->_res_##s[i]); \
		if (err == 0) \
			continue; \
		rtcpu->_res_##s[i] = 0; \
		if (err == -EPROBE_DEFER) { \
			dev_info(dev, "defer %s probe because %s %s\n", \
				rtcpu->name, #_res_, pdata->_res_##_names[i]); \
			return err; \
		} \
		if (_warn_ && err != -ENODATA && err != -ENOENT) \
			dev_warn(dev, "%s %s not available: %d\n", #_res_, \
				pdata->_res_##_names[i], err); \
	}

#define _PTR2ERR(x) (IS_ERR(x) ? PTR_ERR(x) : 0)

	GET_RESOURCES(clock, devm_clk_get, true, _PTR2ERR);
	GET_RESOURCES(reset, tegra_camrtc_reset_control_get, true, _PTR2ERR);
	GET_RESOURCES(reg, tegra_cam_ioremap_byname, true, _PTR2ERR);

#undef _PTR2ERR

	return 0;
}

static int tegra_camrtc_get_irqs(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	const struct tegra_cam_rtcpu_pdata *pdata = rtcpu->pdata;
	int i, err;

	/*
	 * AGIC can be touched only after APE is fully powered on.
	 *
	 * This can be called only after runtime resume.
	 */

#define _get_irq(_dev, _name) of_irq_get_byname(_dev->of_node, _name)
#define _int2err(x) ((x) < 0 ? (x) : 0)

	GET_RESOURCES(irq, _get_irq, true, _int2err);

#undef _get_irq
#undef _int2err

	return 0;
}

static int tegra_camrtc_enable_clks(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(rtcpu->clocks); i++) {
		if (rtcpu->clocks[i] == NULL)
			continue;

		ret = clk_prepare_enable(rtcpu->clocks[i]);
		if (ret) {
			dev_err(dev, "turning on clock %s failed: %d\n",
				rtcpu->pdata->clock_names[i], ret);
			return ret;
		}
	}

	return 0;
}

static int tegra_camrtc_deassert_resets(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(rtcpu->resets); i++) {
		if (rtcpu->resets[i] == NULL)
			continue;

		ret = reset_control_deassert(rtcpu->resets[i]);
		if (ret) {
			dev_err(dev, "reset %s failed: %d\n",
				rtcpu->pdata->reset_names[i], ret);
			return ret;
		}
	}

	return 0;
}

static void tegra_camrtc_disable_clks(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < ARRAY_SIZE(rtcpu->clocks); i++) {
		if (rtcpu->clocks[i] != NULL)
			clk_disable_unprepare(rtcpu->clocks[i]);
	}
}

static void tegra_camrtc_assert_resets(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int j, ret;

	for (j = 0; j < ARRAY_SIZE(rtcpu->resets); j++) {
		int i = ARRAY_SIZE(rtcpu->resets) - 1 - j;

		if (rtcpu->resets[i] == NULL)
			continue;

		ret = reset_control_assert(rtcpu->resets[i]);
		if (ret) {
			dev_err(dev, "asserting reset %s failed: %d\n",
				rtcpu->pdata->reset_names[i], ret);
		}
	}
}

/*
 * Send the PM_SUSPEND command to remote core FW.
 */
static int tegra_camrtc_cmd_pm_suspend(struct device *dev, long *timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	u32 command = RTCPU_COMMAND(PM_SUSPEND, 0);
	int expect = RTCPU_COMMAND(PM_SUSPEND,
			(CAMRTC_PM_CTRL_STATE_SUSPEND << 8) |
			CAMRTC_PM_CTRL_STATUS_OK);
	int err;

	mutex_lock(&rtcpu->cmd.mutex);
	err = tegra_camrtc_mbox_exchange(dev, command, timeout);
	mutex_unlock(&rtcpu->cmd.mutex);

	if (err == expect)
		return 0;

	dev_WARN(dev, "PM_SUSPEND failed: 0x%08x\n", (unsigned)err);
	if (err >= 0)
		err = -EIO;
	return err;
}

static int tegra_sce_cam_wait_for_wfi(struct device *dev, long *timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	long delay_stride = HZ / 50;

	if (rtcpu->pm_base == NULL)
		return 0;

	/* Poll for WFI assert.*/
	for (;;) {
		u32 val = readl(rtcpu->pm_base + TEGRA_PM_PWR_STATUS_0);

		if ((val & TEGRA_PM_WFIPIPESTOPPED) == 0)
			break;

		if (*timeout < 0) {
			dev_WARN(dev, "timeout waiting for WFI\n");
			return -EBUSY;
		}

		msleep(delay_stride);
		*timeout -= delay_stride;
	}

	return 0;
}

static int tegra_sce_cam_suspend_core(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	long timeout = 2 * rtcpu->cmd.timeout;
	int err;

	err = tegra_camrtc_cmd_pm_suspend(dev, &timeout);
	if (err)
		return err;

	return tegra_sce_cam_wait_for_wfi(dev, &timeout);
}

static int tegra_sce_cam_check_fw(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	if (rtcpu->fw_base != NULL && readl(rtcpu->fw_base) == 0) {
		dev_info(dev, "no firmware");
		return -ENODEV;
	}

	return 0;
}

static irqreturn_t tegra_camrtc_adsp_wfi_handler(int irq, void *data)
{
	struct completion *entered_wfi = data;

	disable_irq_nosync(irq);

	complete(entered_wfi);

	return IRQ_HANDLED;
}

static int tegra_ape_cam_wait_for_l2_idle(struct device *dev, long *timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	long delay_stride = HZ / 50;

	if (rtcpu->amisc_base == NULL) {
		dev_WARN(dev, "iobase \"ape-amisc\" missing\n");
		return 0;
	}

	/* Poll for L2 idle.*/
	for (;;) {
		u32 val = readl(rtcpu->amisc_base + AMISC_ADSP_STATUS);
		u32 mask = AMISC_ADSP_L2_IDLE;

		if ((val & mask) == mask)
			break;

		if (*timeout <= 0) {
			dev_WARN(dev, "timeout waiting for L2 idle\n");
			return -EBUSY;
		}

		msleep(delay_stride);
		*timeout -= delay_stride;
	}

	return 0;
}

static int tegra_ape_cam_wait_for_wfi(struct device *dev, long *timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	DECLARE_COMPLETION_ONSTACK(entered_wfi);
	unsigned int irq = rtcpu->adsp_wfi_irq;
	int err;

	if (irq <= 0) {
		dev_WARN(dev, "irq \"adsp-wfi\" missing\n");
		return 0;
	}

	err = request_threaded_irq(irq,
			tegra_camrtc_adsp_wfi_handler, NULL,
			IRQF_TRIGGER_HIGH,
			"adsp-wfi", &entered_wfi);
	if (err) {
		dev_WARN(dev, "cannot request for %s interrupt: %d\n",
			"adsp-wfi", err);
		return err;
	}

	*timeout = wait_for_completion_timeout(&entered_wfi, *timeout);

	free_irq(irq, &entered_wfi);

	if (*timeout == 0) {
		dev_WARN(dev, "timeout waiting for WFI\n");
		return -EBUSY;
	}

	return 0;
}


static int tegra_ape_cam_suspend_core(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	long timeout = 2 * rtcpu->cmd.timeout;
	int err;

	err = tegra_camrtc_cmd_pm_suspend(dev, &timeout);
	if (err)
		return err;

	err = tegra_ape_cam_wait_for_wfi(dev, &timeout);
	if (err)
		return err;

	return tegra_ape_cam_wait_for_l2_idle(dev, &timeout);
}

static int tegra_camrtc_suspend_core(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	rtcpu->boot_sync_done = false;

	return rtcpu->pdata->suspend_core(dev);
}

static void tegra_camrtc_set_online(struct device *dev, bool online)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	/* No need to report if there is no bus yet */
	if (IS_ERR_OR_NULL(rtcpu->ivc))
		return;

	if (online == rtcpu->online)
		return;

	if (online) {
		/* Activate the IVC services in firmware */
		int ret = tegra_ivc_bus_boot_sync(rtcpu->ivc);
		if (ret < 0) {
			dev_err(dev, "ivc-bus boot sync failed: %d\n", ret);
			return;
		}
	}

	rtcpu->online = online;
	tegra_ivc_bus_ready(rtcpu->ivc, online);
}

static u32 tegra_camrtc_full_notify(void *data, u32 response)
{
	struct tegra_cam_rtcpu *rtcpu = data;

	atomic_set(&rtcpu->cmd.response, response);
	wake_up(&rtcpu->cmd.response_waitq);

	return 0;
}

static void tegra_camrtc_empty_notify(void *data, u32 empty_value)
{
	struct tegra_cam_rtcpu *rtcpu = data;

	wake_up(&rtcpu->cmd.empty_waitq);
}

static long tegra_camrtc_wait_for_empty(struct device *dev,
				long timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	if (timeout == 0)
		timeout = 2 * HZ;

	timeout = wait_event_interruptible_timeout(
		rtcpu->cmd.empty_waitq,
		tegra_hsp_sm_pair_is_empty(rtcpu->sm_pair),
		timeout);

	return timeout;
}

static int tegra_camrtc_mbox_exchange(struct device *dev,
					u32 command, long *timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

#define INVALID_RESPONSE (0x80000000U)

	*timeout = tegra_camrtc_wait_for_empty(dev, *timeout);
	if (*timeout <= 0) {
		dev_err(dev, "command: 0x%08x: empty mailbox%s timeout\n",
			command,
			tegra_hsp_sm_pair_is_empty(rtcpu->sm_pair) ?
			" interrupt" : "");
		return -ETIMEDOUT;
	}

	atomic_set(&rtcpu->cmd.response, INVALID_RESPONSE);

	tegra_hsp_sm_pair_write(rtcpu->sm_pair, command);

	*timeout = wait_event_interruptible_timeout(
		rtcpu->cmd.response_waitq,
		atomic_read(&rtcpu->cmd.response) != INVALID_RESPONSE,
		*timeout);
	if (*timeout <= 0) {
		dev_err(dev, "command: 0x%08x: response timeout\n", command);
		return -ETIMEDOUT;
	}

	return (int)atomic_read(&rtcpu->cmd.response);
}

int tegra_camrtc_command(struct device *dev, u32 command, long timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int response;

	if (timeout == 0)
		timeout = rtcpu->cmd.timeout;

	mutex_lock(&rtcpu->cmd.mutex);

	response = tegra_camrtc_mbox_exchange(dev, command, &timeout);

	mutex_unlock(&rtcpu->cmd.mutex);

	return response;
}
EXPORT_SYMBOL(tegra_camrtc_command);

int tegra_camrtc_prefix_command(struct device *dev,
				u32 prefix, u32 command, long timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int response;

	if (timeout == 0)
		timeout = rtcpu->cmd.timeout;

	mutex_lock(&rtcpu->cmd.mutex);

	prefix = RTCPU_COMMAND(PREFIX, prefix);
	response = tegra_camrtc_mbox_exchange(dev, prefix, &timeout);

	if (RTCPU_GET_COMMAND_ID(response) == RTCPU_CMD_PREFIX)
		response = tegra_camrtc_mbox_exchange(dev, command, &timeout);

	mutex_unlock(&rtcpu->cmd.mutex);

	return response;
}
EXPORT_SYMBOL(tegra_camrtc_prefix_command);

static void tegra_sce_cam_config_core(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	u32 val;

	if (rtcpu->pm_base == NULL)
		return;

	val = readl(rtcpu->pm_base + TEGRA_PM_R5_CTRL_0);

	/* Skip configuring core if it is already unhalted */
	if ((val & TEGRA_PM_FWLOADDONE) != 0) {
		dev_info(dev, "already unhalted\n");
		goto already_configured;
	}

	/* Configure R5 core */
	if (rtcpu->cfg_base) {
		/* Disable R5R and smartcomp in camera mode */
		writel(TEGRA_R5R_SC_DISABLE,
			rtcpu->cfg_base + TEGRA_APS_FRSC_SC_CTL_0);

		/* Enable JTAG/Coresight */
		writel(TEGRA_FN_MODEIN,
			rtcpu->cfg_base + TEGRA_APS_FRSC_SC_MODEIN_0);

		mmiowb();

		/* Reset R5 */
		if (rtcpu->sce_resets.sce_nsysporeset != NULL)
			reset_control_reset(rtcpu->sce_resets.sce_nsysporeset);
	}

	writel(val | TEGRA_PM_FWLOADDONE,
		rtcpu->pm_base + TEGRA_PM_R5_CTRL_0);

	mmiowb();

already_configured:
	/*
	 * Make sure we always find FWLOADDONE set to 0 after crash.
	 * Clearing FWLOADDONE does not affect R5 once it has been set.
	 */
	writel(val & ~TEGRA_PM_FWLOADDONE,
		rtcpu->pm_base + TEGRA_PM_R5_CTRL_0);
}

static int tegra_camrtc_poweron(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int ret;

	if (rtcpu->powered)
		return 0;

	/* APE power domain may misbehave and try to resume while probing */
	if (rtcpu->sm_pair == NULL)
		return 0;

	/* Power on and let core run */
	ret = tegra_camrtc_enable_clks(dev);
	if (ret) {
		dev_err(dev, "failed to turn on %s clocks: %d\n",
			rtcpu->name, ret);
		return ret;
	}

	ret = tegra_camrtc_deassert_resets(dev);
	if (ret)
		return ret;

	rtcpu->powered = true;

	if (rtcpu->pdata->config_core)
		rtcpu->pdata->config_core(dev);

	return 0;
}

static void tegra_camrtc_poweroff(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	if (!rtcpu->powered)
		return;

	rtcpu->powered = false;

	tegra_camrtc_assert_resets(dev);
	tegra_camrtc_disable_clks(dev);
}

static int tegra_camrtc_boot_sync(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int ret;
	u32 command;

	if (rtcpu->boot_sync_done)
		return 0;

	/*
	 * Handshake FW version before continuing with the boot
	 */
	command = RTCPU_COMMAND(INIT, 0);
	ret = tegra_camrtc_command(dev, command, 0);
	if (ret < 0)
		return ret;
	if (ret != command) {
		dev_err(dev, "RTCPU sync problem (response=0x%08x)\n", ret);
		return -EIO;
	}

	command = RTCPU_COMMAND(FW_VERSION, RTCPU_FW_SM3_VERSION);
	ret = tegra_camrtc_command(dev, command, 0);
	if (ret < 0)
		return ret;
	if (RTCPU_GET_COMMAND_ID(ret) != RTCPU_CMD_FW_VERSION ||
		RTCPU_GET_COMMAND_VALUE(ret) < RTCPU_FW_SM4_VERSION) {
		dev_err(dev, "RTCPU version mismatch (response=0x%08x)\n", ret);
		return -EIO;
	}

	rtcpu->fw_version = RTCPU_GET_COMMAND_VALUE(ret);
	rtcpu->boot_sync_done = true;

	/*
	 * Enable trace
	 */
	if (rtcpu->tracer) {
		ret = tegra_rtcpu_trace_boot_sync(rtcpu->tracer);
		if (ret < 0) {
			dev_err(dev, "trace boot sync failed: %d\n", ret);
			goto error;
		}
	}

	return 0;

error:
	return ret;
}

/*
 * RTCPU boot sequence
 */
static int tegra_camrtc_boot(struct device *dev)
{
	int ret;

	ret = tegra_camrtc_poweron(dev);
	if (ret)
		return ret;

	ret = tegra_camrtc_boot_sync(dev);
	if (ret == 0) {
		tegra_camrtc_set_online(dev, true);
	}

	return 0;
}

int tegra_camrtc_iovm_setup(struct device *dev, dma_addr_t iova)
{
	u32 command = RTCPU_COMMAND(CH_SETUP, iova >> 8);
	int ret = tegra_camrtc_command(dev, command, 0);
	if (ret < 0)
		return ret;

	if (RTCPU_GET_COMMAND_ID(ret) == RTCPU_CMD_ERROR) {
		u32 error = RTCPU_GET_COMMAND_VALUE(ret);
		dev_err(dev, "IOVM setup error: %u\n", error);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_camrtc_iovm_setup);

static int tegra_camrtc_get_fw_hash(struct device *dev,
				u8 hash[RTCPU_FW_HASH_SIZE])
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int ret, i;
	u32 value;

	if (rtcpu->fw_version < RTCPU_FW_SM2_VERSION) {
		dev_info(dev, "fw version %u has no sha1\n", rtcpu->fw_version);
		return -EIO;
	}

	for (i = 0; i < RTCPU_FW_HASH_SIZE; i++) {
		ret = tegra_camrtc_command(dev, RTCPU_COMMAND(FW_HASH, i), 0);
		value = RTCPU_GET_COMMAND_VALUE(ret);

		if (ret < 0 ||
			RTCPU_GET_COMMAND_ID(ret) != RTCPU_CMD_FW_HASH ||
			value > (u8)~0) {
			dev_warn(dev, "FW_HASH problem (0x%08x)\n", ret);
			return -EIO;
		}

		hash[i] = value;
	}

	return 0;
}

ssize_t tegra_camrtc_print_version(struct device *dev,
					char *buf, size_t size)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	struct seq_buf s;
	int i;

	seq_buf_init(&s, buf, size);
	seq_buf_printf(&s, "version cpu=%s cmd=%u sha1=",
		rtcpu->name, rtcpu->fw_version);

	for (i = 0; i < RTCPU_FW_HASH_SIZE; i++)
		seq_buf_printf(&s, "%02x", rtcpu->fw_hash[i]);

	return seq_buf_used(&s);
}
EXPORT_SYMBOL(tegra_camrtc_print_version);

static void tegra_camrtc_log_fw_version(struct device *dev)
{
	char version[TEGRA_CAMRTC_VERSION_LEN];

	tegra_camrtc_print_version(dev, version, sizeof(version));

	dev_info(dev, "firmware %s\n", version);
}

static int tegra_cam_rtcpu_runtime_suspend(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	const struct tegra_cam_rtcpu_pdata *pdata = rtcpu->pdata;
	int i;
	int err;

	err = tegra_camrtc_suspend_core(dev);
	/* Try full reset if an error occurred while suspending core. */
	if (WARN(err < 0, "RTCPU suspend failed, resetting it")) {
		/* runtime_resume() powers RTCPU back on */
		tegra_camrtc_poweroff(dev);
	}

	for (i = 0; i < ARRAY_SIZE(rtcpu->clocks); i++) {
		if (rtcpu->clocks[i] == NULL || pdata->clock_rates[i].slow == 0)
			continue;
		clk_set_rate(rtcpu->clocks[i], pdata->clock_rates[i].slow);
	}

	return 0;
}

static int tegra_cam_rtcpu_runtime_resume(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	const struct tegra_cam_rtcpu_pdata *pdata = rtcpu->pdata;
	int ret;
	int i;

	ret = tegra_camrtc_poweron(dev);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(rtcpu->clocks); i++) {
		if (rtcpu->clocks[i] == NULL || pdata->clock_rates[i].fast == 0)
			continue;
		clk_set_rate(rtcpu->clocks[i], pdata->clock_rates[i].fast);
	}

	ret = tegra_camrtc_boot_sync(dev);
	if (ret)
		return ret;

	tegra_camrtc_set_online(dev, true);

	return 0;
}

static int tegra_camrtc_mbox_init(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	struct device_node *hsp_node;

	if (!IS_ERR_OR_NULL(rtcpu->sm_pair))
		return 0;

	mutex_init(&rtcpu->cmd.mutex);
	init_waitqueue_head(&rtcpu->cmd.response_waitq);
	init_waitqueue_head(&rtcpu->cmd.empty_waitq);
	hsp_node = of_get_child_by_name(dev->of_node, "hsp");
	rtcpu->sm_pair = of_tegra_hsp_sm_pair_by_name(hsp_node,
					"cmd-pair", tegra_camrtc_full_notify,
					tegra_camrtc_empty_notify, rtcpu);
	of_node_put(hsp_node);
	if (IS_ERR(rtcpu->sm_pair)) {
		int ret = PTR_ERR(rtcpu->sm_pair);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to obtain %s mbox pair: %d\n",
				rtcpu->name, ret);
		rtcpu->sm_pair = NULL;
		return ret;
	}

	return 0;
}

static int tegra_cam_rtcpu_remove(struct platform_device *pdev)
{
	struct tegra_cam_rtcpu *rtcpu = platform_get_drvdata(pdev);
	bool pm_is_active = pm_runtime_active(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);

	tegra_camrtc_set_online(&pdev->dev, false);

	if (rtcpu->sm_pair) {
		if (pm_is_active)
			tegra_camrtc_suspend_core(&pdev->dev);
		tegra_hsp_sm_pair_free(rtcpu->sm_pair);
		rtcpu->sm_pair = NULL;
	}

	tegra_rtcpu_trace_destroy(rtcpu->tracer);
	tegra_camrtc_poweroff(&pdev->dev);
	tegra_pd_remove_device(&pdev->dev);
	tegra_cam_rtcpu_mon_destroy(rtcpu->monitor);
	tegra_ivc_bus_destroy(rtcpu->ivc);

	return 0;
}

static int tegra_cam_rtcpu_probe(struct platform_device *pdev)
{
	struct tegra_cam_rtcpu *rtcpu;
	const struct tegra_cam_rtcpu_pdata *pdata;
	struct device *dev = &pdev->dev;
	int ret;
	const char *name;
	uint32_t timeout;

	pdata = of_device_get_match_data(dev);
	if (pdata == NULL) {
		dev_err(dev, "no device match\n");
		return -ENODEV;
	}

	name = pdata->name;
	of_property_read_string(dev->of_node, NV(cpu-name), &name);

	dev_dbg(dev, "probing RTCPU on %s\n", name);

	rtcpu = devm_kzalloc(dev, sizeof(*rtcpu), GFP_KERNEL);
	if (rtcpu == NULL)
		return -ENOMEM;

	rtcpu->pdata = pdata;
	rtcpu->name = name;
	platform_set_drvdata(pdev, rtcpu);

	ret = tegra_camrtc_get_resources(dev);
	if (ret)
		goto fail;

	timeout = 2000;
	of_property_read_u32(dev->of_node, NV(cmd-timeout), &timeout);
	rtcpu->cmd.timeout = msecs_to_jiffies(timeout);

	timeout = 60000;
	ret = of_property_read_u32(dev->of_node, NV(autosuspend-delay-ms), &timeout);
	if (ret == 0) {
		pm_runtime_use_autosuspend(dev);
		pm_runtime_set_autosuspend_delay(&pdev->dev, timeout);
	}

	rtcpu->tracer = tegra_rtcpu_trace_create(dev);

	ret = tegra_camrtc_mbox_init(dev);
	if (ret)
		goto fail;

	/* Enable runtime power management */
	pm_runtime_enable(dev);

	/* Power on device */
	pm_runtime_get_sync(dev);

	/* Clocks are on, resets are deasserted, we can touch the hardware */
	ret = pdata->check_fw ? pdata->check_fw(dev) : 0;
	if (ret)
		goto put_and_fail;

	/* Tegra-agic driver routes IRQs when probing, do it when powered */
	ret = tegra_camrtc_get_irqs(dev);
	if (ret)
		goto put_and_fail;

	rtcpu->ivc = tegra_ivc_bus_create(dev);
	if (IS_ERR(rtcpu->ivc)) {
		ret = PTR_ERR(rtcpu->ivc);
		goto put_and_fail;
	}

	rtcpu->monitor = tegra_camrtc_mon_create(dev);
	if (IS_ERR(rtcpu->monitor)) {
		ret = PTR_ERR(rtcpu->monitor);
		goto put_and_fail;
	}

	if (pdata->id == TEGRA_CAM_RTCPU_APE) {
		/* APE power domain powergates APE block when suspending */
		/* This won't do */
		pm_runtime_get(dev);
	}

	ret = tegra_camrtc_get_fw_hash(dev, rtcpu->fw_hash);
	if (ret == 0)
		devm_tegrafw_register(dev,
			name != pdata->name ? name :  "camrtc",
			TFW_NORMAL, tegra_camrtc_print_version, NULL);

	tegra_camrtc_set_online(dev, true);

	pm_runtime_put(dev);

	/* Print firmware version */
	tegra_camrtc_log_fw_version(dev);

	dev_dbg(dev, "successfully probed RTCPU on %s\n", name);

	return 0;

put_and_fail:
	pm_runtime_put(dev);
fail:
	tegra_cam_rtcpu_remove(pdev);
	return ret;
}

int tegra_camrtc_reboot(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int ret;

	if (pm_runtime_suspended(dev))
		return -EIO;

	if (!rtcpu->powered)
		return -EIO;

	rtcpu->boot_sync_done = false;

	tegra_camrtc_assert_resets(dev);

	tegra_camrtc_set_online(dev, false);

	ret = tegra_camrtc_deassert_resets(dev);
	if (ret)
		return ret;

	if (rtcpu->pdata->config_core)
		rtcpu->pdata->config_core(dev);

	return tegra_camrtc_boot(dev);
}
EXPORT_SYMBOL(tegra_camrtc_reboot);

int tegra_camrtc_restore(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	if (rtcpu->monitor)
		return tegra_camrtc_mon_restore_rtcpu(rtcpu->monitor);
	else
		return tegra_camrtc_reboot(dev);
}
EXPORT_SYMBOL(tegra_camrtc_restore);

bool tegra_camrtc_is_rtcpu_alive(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	return rtcpu->online;
}
EXPORT_SYMBOL(tegra_camrtc_is_rtcpu_alive);

static int tegra_camrtc_halt(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	tegra_camrtc_set_online(dev, false);

	if (!rtcpu->powered)
		return 0;

	if (!pm_runtime_suspended(dev))
		tegra_camrtc_suspend_core(dev);

	tegra_camrtc_poweroff(dev);

	return 0;
}

static void tegra_cam_rtcpu_shutdown(struct platform_device *pdev)
{
	tegra_camrtc_halt(&pdev->dev);
}

static const struct of_device_id tegra_cam_rtcpu_of_match[] = {
	{
		.compatible = NV(tegra186-sce-ivc), .data = &sce_pdata
	},
	{
		.compatible = NV(tegra186-ape-ivc), .data = &ape_pdata
	},
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_cam_rtcpu_of_match);

static const struct dev_pm_ops tegra_cam_rtcpu_pm_ops = {
	.suspend = tegra_camrtc_halt,
	.resume = tegra_camrtc_poweron,
	.runtime_suspend = tegra_cam_rtcpu_runtime_suspend,
	.runtime_resume = tegra_cam_rtcpu_runtime_resume,
};

static struct platform_driver tegra_cam_rtcpu_driver = {
	.driver = {
		.name	= "tegra186-cam-rtcpu",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_cam_rtcpu_of_match),
#ifdef CONFIG_PM
		.pm = &tegra_cam_rtcpu_pm_ops,
#endif
	},
	.probe = tegra_cam_rtcpu_probe,
	.remove = tegra_cam_rtcpu_remove,
	.shutdown = tegra_cam_rtcpu_shutdown,
};
module_platform_driver(tegra_cam_rtcpu_driver);

MODULE_DESCRIPTION("CAMERA RTCPU driver");
MODULE_AUTHOR("NVIDIA");
MODULE_LICENSE("GPL v2");
