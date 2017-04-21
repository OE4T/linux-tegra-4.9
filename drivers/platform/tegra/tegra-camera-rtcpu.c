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
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
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

/* Register specifics */
#define TEGRA_APS_FRSC_SC_CTL_0			0x0
#define TEGRA_APS_FRSC_SC_MODEIN_0		0x14
#define TEGRA_PM_R5_CTRL_0			0x40
#define TEGRA_PM_PWR_STATUS_0			0x20

#define TEGRA_R5R_SC_DISABLE			0x5
#define TEGRA_FN_MODEIN				0x29527
#define TEGRA_PM_FWLOADDONE			0x2
#define TEGRA_PM_WFIPIPESTOPPED			0x200000

static const char * const sce_clock_names[] = {
	"sce-apb",
	"sce-cpu-nic",
};

static const struct rtcpu_clock_rates {
	unsigned slow, fast;
} sce_clock_rates[] = {
	/* Slow rate (RTPM),  fast rate */
	{ .slow = 102000000, .fast = 102000000, },
	{ .slow = 115200000, .fast = 473600000, },
};

static inline void bugme(void)
{
BUILD_BUG_ON(ARRAY_SIZE(sce_clock_names) != ARRAY_SIZE(sce_clock_rates));
}

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

static const char * const ape_clock_names[] = {
	"ahub",
	"apb2ape",
	"ape",
	"adsp",
	"adspneon",
};

static const char * const ape_reset_names[] = {
	"adsp-all"
};

static const char * const ape_reg_names[] = {
};

enum tegra_cam_rtcpu_id {
	TEGRA_CAM_RTCPU_SCE,
	TEGRA_CAM_RTCPU_APE,
};

struct tegra_cam_rtcpu_pdata {
	const char *name;
	const char * const *clock_names;
	const struct rtcpu_clock_rates *clock_rates;
	const char * const *reset_names;
	const char * const *reg_names;
	enum tegra_cam_rtcpu_id id;
	u32 num_clocks;
	u32 num_resets;
	u32 num_regs;
};

static const struct tegra_cam_rtcpu_pdata sce_pdata = {
	.name = "sce",
	.clock_names = sce_clock_names,
	.clock_rates = sce_clock_rates,
	.reset_names = sce_reset_names,
	.id = TEGRA_CAM_RTCPU_SCE,
	.num_clocks = ARRAY_SIZE(sce_clock_names),
	.num_resets = ARRAY_SIZE(sce_reset_names),
	.reg_names = sce_reg_names,
	.num_regs = ARRAY_SIZE(sce_reg_names),
};

static const struct tegra_cam_rtcpu_pdata ape_pdata = {
	.name = "ape",
	.clock_names = ape_clock_names,
	.reset_names = ape_reset_names,
	.id = TEGRA_CAM_RTCPU_APE,
	.num_clocks = ARRAY_SIZE(ape_clock_names),
	.num_resets = ARRAY_SIZE(ape_reset_names),
	.reg_names = ape_reg_names,
	.num_regs = ARRAY_SIZE(ape_reg_names),
};

#define NV(p) "nvidia," #p

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
		atomic_t emptied;
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
	};
	struct clk *clocks[NUM(clock_names)];
	union {
		struct sce_resets sce_resets;
		struct reset_control *resets[NUM(reset_names)];
	};
	const struct tegra_cam_rtcpu_pdata *pdata;
	struct tegra_camrtc_mon *monitor;
	bool power_domain;
	bool online;
};

struct reset_control *tegra_camrtc_reset_control_get(struct device *dev,
						const char *id)
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

static int tegra_cam_rtcpu_get_resources(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	const struct tegra_cam_rtcpu_pdata *pdata = rtcpu->pdata;
	int i, err;

#define GET_RESOURCES(_res_, _get_, _warn_) \
	for (i = 0; i < pdata->num_##_res_##s; i++) { \
		rtcpu->_res_##s[i] = _get_(dev, pdata->_res_##_names[i]); \
		if (!IS_ERR(rtcpu->_res_##s[i])) \
			continue; \
		err = PTR_ERR(rtcpu->_res_##s[i]); \
		rtcpu->_res_##s[i] = NULL; \
		if (err == -EPROBE_DEFER) { \
			dev_info(dev, "defer %s probe because %s %s\n", \
				rtcpu->name, #_res_, pdata->_res_##_names[i]); \
			return err; \
		} \
		if (_warn_ && err != -ENODATA && err != -ENOENT) \
			dev_err(dev, "%s %s not available: %d\n", #_res_, \
				pdata->_res_##_names[i], err); \
	}

	GET_RESOURCES(clock, devm_clk_get, true);
	GET_RESOURCES(reset, tegra_camrtc_reset_control_get, true);
	GET_RESOURCES(reg, tegra_cam_ioremap_byname, true);

	return 0;
}

static int tegra_cam_rtcpu_clk_disable_unprepare(struct clk *clk)
{
	clk_disable_unprepare(clk);

	return 0;
}

static int tegra_cam_rtcpu_apply_clks(struct device *dev,
					int (*func)(struct clk *clk))
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(rtcpu->clocks); i++) {
		if (rtcpu->clocks[i] == NULL)
			continue;

		ret = (*func)(rtcpu->clocks[i]);
		if (ret) {
			dev_err(dev, "clock %s failed: %d\n",
				rtcpu->pdata->clock_names[i], ret);
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

static int tegra_camrtc_wait_for_wfi(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	long timeout = HZ, delay_stride = HZ / 50;

	if (rtcpu->pm_base == NULL)
		return 0;

	/* Poll for WFI assert.*/
	for (;;) {
		u32 val = readl(rtcpu->pm_base + TEGRA_PM_PWR_STATUS_0);

		if ((val & TEGRA_PM_WFIPIPESTOPPED) == 0)
			break;

		if (timeout < 0)
			return -EBUSY;

		msleep(delay_stride);
		timeout -= delay_stride;
	}

	return 0;
}

static void tegra_camrtc_set_online(struct device *dev, bool online)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	if (online != rtcpu->online) {
		rtcpu->online = online;
		tegra_ivc_bus_ready(rtcpu->ivc, online);
	}
}

/*
 * Send the PM_SUSPEND command to remote core FW and
 * perform necessary checks to confirm if RTCPU is
 * indeed suspended.
 */
static int tegra_cam_rtcpu_cmd_remote_suspend(struct device *dev)
{
	int err;
	int expect = RTCPU_COMMAND(PM_SUSPEND,
				(CAMRTC_PM_CTRL_STATE_SUSPEND << 8) |
				CAMRTC_PM_CTRL_STATUS_OK);

	/* Suspend the RTCPU */
	err = tegra_camrtc_command(dev, RTCPU_COMMAND(PM_SUSPEND, 0), 0);
	if (err != expect) {
		dev_err(dev, "suspend failed: 0x%08x\n", (unsigned)err);
		if (err >= 0)
			return -EIO;
		return err;
	}

	err = tegra_camrtc_wait_for_wfi(dev);
	if (err) {
		dev_err(dev, "failed to go into WFI\n");
		return err;
	}

	return 0;
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

	atomic_set(&rtcpu->cmd.emptied, 1);
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
		/* Make sure IRQ has been handled */
		atomic_read(&rtcpu->cmd.emptied) != 0 &&
		tegra_hsp_sm_pair_is_empty(rtcpu->sm_pair),
		timeout);

	if (timeout > 0)
		atomic_set(&rtcpu->cmd.emptied, 0);

	return timeout;
}

static int tegra_camrtc_mbox_exchange(struct device *dev,
					u32 command, long *timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

#define INVALID_RESPONSE (0x80000000U)

	*timeout = tegra_camrtc_wait_for_empty(dev, *timeout);
	if (*timeout <= 0) {
		dev_err(dev, "command: 0x%08x: empty mailbox timeout\n", command);
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

static int tegra_camrtc_poweron(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	u32 val = 0;
	int ret;

	ret = tegra_cam_rtcpu_apply_clks(dev, clk_prepare_enable);
	if (ret) {
		dev_err(dev, "failed to turn on %s clocks: %d\n",
			rtcpu->name, ret);
		return ret;
	}

	ret = tegra_camrtc_deassert_resets(dev);
	if (ret) {
		dev_err(dev, "failed to deassert %s resets: %d\n",
			rtcpu->name, ret);
		return ret;
	}

	if (rtcpu->pm_base) {
		val = readl(rtcpu->pm_base + TEGRA_PM_R5_CTRL_0);

		/* Skip configuring core if it is already unhalted */
		if ((val & TEGRA_PM_FWLOADDONE) != 0) {
			writel(val & ~TEGRA_PM_FWLOADDONE,
				rtcpu->pm_base + TEGRA_PM_R5_CTRL_0);
			return 0;
		}
	}

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

	if (rtcpu->pm_base) {
		writel(val | TEGRA_PM_FWLOADDONE,
			rtcpu->pm_base + TEGRA_PM_R5_CTRL_0);

		mmiowb();

		/*
		 * Make sure we always find FWLOADDONE set to 0 after crash.
		 * Clearing FWLOADDONE does not affect R5 once it has been set.
		 */
		writel(val & ~TEGRA_PM_FWLOADDONE,
			rtcpu->pm_base + TEGRA_PM_R5_CTRL_0);
	}

	return 0;
}

int tegra_camrtc_poweroff(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int ret;

	if (rtcpu->pm_base) {
		u32 val = readl(rtcpu->pm_base + TEGRA_PM_R5_CTRL_0);
		writel(val & ~TEGRA_PM_FWLOADDONE,
			rtcpu->pm_base + TEGRA_PM_R5_CTRL_0);
	}

	tegra_camrtc_assert_resets(dev);

	ret = tegra_cam_rtcpu_apply_clks(dev,
				tegra_cam_rtcpu_clk_disable_unprepare);
	if (ret) {
		dev_err(dev, "failed to turn off %s clocks: %d\n",
			rtcpu->name, ret);
		return ret;
	}

	return 0;
}

/*
 * RTCPU boot sequence
 */
static int tegra_camrtc_boot_sync(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	u32 command;
	int ret;

	dev_info(dev, "booting %s with Camera RTCPU FW\n", rtcpu->name);

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
		RTCPU_GET_COMMAND_VALUE(ret) < RTCPU_FW_VERSION) {
		dev_err(dev, "RTCPU version mismatch (response=0x%08x)\n", ret);
		return -EIO;
	}

	rtcpu->fw_version = RTCPU_GET_COMMAND_VALUE(ret);

	/*
	 * Service level boot handshake
	 */
	if (rtcpu->fw_version >= RTCPU_FW_SM3_VERSION) {
		ret = tegra_rtcpu_trace_boot_sync(rtcpu->tracer);
		if (ret < 0) {
			dev_err(dev, "trace boot sync failed: %d\n", ret);
			goto error;
		}
		ret = tegra_ivc_bus_boot_sync(rtcpu->ivc);
		if (ret < 0) {
			dev_err(dev, "IVC boot sync failed: %d\n", ret);
			goto error;
		}
	}

	tegra_camrtc_set_online(dev, true);
	return 0;

error:
	return ret;
}

int tegra_camrtc_boot(struct device *dev)
{
	int err = tegra_camrtc_poweron(dev);

	if (err)
		return err;

	return tegra_camrtc_boot_sync(dev);
}
EXPORT_SYMBOL(tegra_camrtc_boot);

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

	if (pdata->clock_rates == NULL)
		return 0;

	for (i = 0; i < ARRAY_SIZE(rtcpu->clocks); i++) {
		if (rtcpu->clocks[i] == NULL)
			continue;
		clk_set_rate(rtcpu->clocks[i], pdata->clock_rates[i].slow);
	}

	return 0;
}

static int tegra_cam_rtcpu_runtime_resume(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	const struct tegra_cam_rtcpu_pdata *pdata = rtcpu->pdata;
	int i;

	if (pdata->clock_rates == NULL)
		return 0;

	for (i = 0; i < ARRAY_SIZE(rtcpu->clocks); i++) {
		if (rtcpu->clocks[i] == NULL)
			continue;
		clk_set_rate(rtcpu->clocks[i], pdata->clock_rates[i].fast);
	}

	return 0;
}

static int tegra_cam_rtcpu_remove(struct platform_device *pdev)
{
	struct tegra_cam_rtcpu *rtcpu = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);

	if (rtcpu->power_domain)
		tegra_pd_remove_device(&pdev->dev);

	tegra_cam_rtcpu_mon_destroy(rtcpu->monitor);
	tegra_ivc_bus_destroy(rtcpu->ivc);
	tegra_hsp_sm_pair_free(rtcpu->sm_pair);

	return 0;
}

static int tegra_cam_rtcpu_probe(struct platform_device *pdev)
{
	struct tegra_cam_rtcpu *rtcpu;
	const struct tegra_cam_rtcpu_pdata *pdata;
	struct device *dev = &pdev->dev;
	struct device_node *hsp_node;
	int ret;
	const struct of_device_id *match;
	const char *name;

	match = of_match_device(tegra_cam_rtcpu_of_match, dev);
	if (match == NULL) {
		dev_err(dev, "Device match not found\n");
		return -ENODEV;
	}
	pdata = match->data;
	name = pdata->name;
	of_property_read_string(dev->of_node, NV(cpu-name), &name);

	dev_dbg(dev, "probing RTCPU on %s\n", name);

	rtcpu = devm_kzalloc(dev, sizeof(*rtcpu), GFP_KERNEL);
	if (rtcpu == NULL)
		return -ENOMEM;

	rtcpu->pdata = pdata;
	rtcpu->name = name;
	platform_set_drvdata(pdev, rtcpu);

	ret = tegra_cam_rtcpu_get_resources(dev);
	if (ret)
		goto fail;

	if (rtcpu->fw_base != NULL && readl(rtcpu->fw_base) == 0) {
		dev_info(dev, "no firmware");
		ret = -ENODEV;
		goto fail;
	}

	ret = of_property_read_u32(dev->of_node, NV(cmd-timeout),
				&rtcpu->cmd.timeout);
	if (ret == 0)
		rtcpu->cmd.timeout = msecs_to_jiffies(rtcpu->cmd.timeout);
	else
		rtcpu->cmd.timeout = 2 * HZ;

	mutex_init(&rtcpu->cmd.mutex);
	init_waitqueue_head(&rtcpu->cmd.response_waitq);
	init_waitqueue_head(&rtcpu->cmd.empty_waitq);
	hsp_node = of_get_child_by_name(dev->of_node, "hsp");
	rtcpu->sm_pair = of_tegra_hsp_sm_pair_by_name(hsp_node,
					"cmd-pair", tegra_camrtc_full_notify,
					tegra_camrtc_empty_notify, rtcpu);
	of_node_put(hsp_node);
	if (IS_ERR(rtcpu->sm_pair)) {
		ret = PTR_ERR(rtcpu->sm_pair);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to obtain %s mbox pair: %d\n",
				name, ret);
		goto fail;
	}

	rtcpu->tracer = tegra_rtcpu_trace_create(dev);

	rtcpu->ivc = tegra_ivc_bus_create(dev);
	if (IS_ERR(rtcpu->ivc)) {
		ret = PTR_ERR(rtcpu->ivc);
		goto fail;
	}

	rtcpu->monitor = tegra_camrtc_mon_create(dev);
	if (IS_ERR(rtcpu->monitor)) {
		ret = PTR_ERR(rtcpu->monitor);
		goto fail;
	}

	if (of_property_read_bool(dev->of_node, "power-domains")) {
		rtcpu->power_domain = true;
		tegra_pd_add_device(dev);
	}

	/* enable power management */
	pm_runtime_enable(dev);

	/* set resume state */
	pm_runtime_get_sync(dev);

	if (pdata->id == TEGRA_CAM_RTCPU_APE) {
		/* APE power domain powergates APE block when suspending */
		/* This won't do */
		pm_runtime_get(dev);
	}

	/* Boot RTCPU */
	ret = tegra_camrtc_boot(dev);
	if (ret)
		goto boot_fail;

	/* Print firmware version */
	ret = tegra_camrtc_get_fw_hash(dev, rtcpu->fw_hash);
	if (ret == 0)
		devm_tegrafw_register(dev,
			name != pdata->name ? name :  "camrtc",
			TFW_NORMAL, tegra_camrtc_print_version, NULL);

	/* set idle to slow down clock while idle mode */
	pm_runtime_put(dev);

	tegra_camrtc_log_fw_version(dev);

	dev_dbg(dev, "successfully probed RTCPU on %s\n", name);

	return 0;

boot_fail:
	pm_runtime_put_sync(dev);
fail:
	tegra_cam_rtcpu_remove(pdev);
	return ret;
}

static void tegra_cam_rtcpu_shutdown(struct platform_device *pdev)
{
	tegra_camrtc_halt(&pdev->dev);
}

int tegra_camrtc_restore(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	return tegra_camrtc_mon_restore_rtcpu(rtcpu->monitor);
}
EXPORT_SYMBOL(tegra_camrtc_restore);

bool tegra_camrtc_is_rtcpu_alive(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	return rtcpu->online;
}
EXPORT_SYMBOL(tegra_camrtc_is_rtcpu_alive);

int tegra_camrtc_halt(struct device *dev)
{
	int ret;

	ret = tegra_cam_rtcpu_cmd_remote_suspend(dev);
	if (ret)
		dev_warn(dev, "failed to suspend firmware: %d (ignored)\n",
			ret);

	ret = tegra_camrtc_poweroff(dev);
	if (ret)
		return ret;

	tegra_camrtc_set_online(dev, false);

	return 0;
}
EXPORT_SYMBOL(tegra_camrtc_halt);

static const struct dev_pm_ops tegra_cam_rtcpu_pm_ops = {
	.suspend = tegra_camrtc_halt,
	.resume_early = tegra_camrtc_poweron,
	.resume = tegra_camrtc_boot_sync,
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
