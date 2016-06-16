/*
 * Copyright (c) 2015-2016 NVIDIA CORPORATION. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/tegra_ast.h>
#include <linux/tegra-hsp.h>
#include <linux/tegra-ivc-bus.h>
#include <linux/wait.h>

#include <dt-bindings/memory/tegra-swgroup.h>
#include <dt-bindings/memory/tegra186-swgroup.h>

/* Register specifics */
#define TEGRA_APS_FRSC_SC_CTL_0			0x0
#define TEGRA_SCE_APS_FRSC_SC_MODEIN_0		0x14
#define TEGRA_SCE_EVP_RESET_ADDR_0		0x20
#define TEGRA_SCEPM_R5_CTRL_0			0x40

#define TEGRA_SCE_R5R_SC_DISABLE		0x5
#define TEGRA_SCE_FN_MODEIN			0x29527
#define TEGRA_SCE_FWLOADDONE			0x2

enum {
	RTCPU_CMD_INIT = 0,
	RTCPU_CMD_FW_VERSION = 1,
	RTCPU_CMD_IVC_READY,
	RTCPU_CMD_ERROR = 0x7f,
};

#define RTCPU_COMMAND(id, value)	((RTCPU_CMD_ ## id << 24) | value)
#define RTCPU_GET_COMMAND_ID(value)	(((value) >> 24) & 0x7f)
#define RTCPU_GET_COMMAND_VALUE(value)	((value) & 0xffffff)

#define RTCPU_FW_VERSION (1)

static const char * const sce_clock_names[] = {
	"sce-apb",
};

static const char * const sce_reset_names[] = {
	"sce-pm",
	"sce-nsysporeset",
	"sce-nreset",
	"sce-dbgresetn",
	"sce-presetdbgn",
	"sce-actmon",
	"sce-dma",
	"sce-hsp",
	"tsctnsce",
	"sce-tke",
	"sce-gte",
	"sce-cfg",
};

static const char * const ape_clock_names[] = {
	"ahub",
	"apb2ape",
	"ape",
	"adsp",
	"adspneon",
};

static const char * const ape_reset_names[] = {
	"adspdbg",
	"adspintf",
	"adspneon",
	"adspperiph",
	"adspscu",
	"adspwdt",
	"ape",
	"adsp",
};

static struct clk *sce_clocks[ARRAY_SIZE(sce_clock_names)];
static struct clk *ape_clocks[ARRAY_SIZE(sce_clock_names)];
static struct reset_control *sce_resets[ARRAY_SIZE(sce_reset_names)];
static struct reset_control *ape_resets[ARRAY_SIZE(ape_reset_names)];

enum tegra_cam_rtcpu_id {
	TEGRA_CAM_RTCPU_SCE,
	TEGRA_CAM_RTCPU_APE,
};

struct tegra_cam_rtcpu_pdata {
	const char * const *clock_names;
	const char * const *reset_names;
	struct clk **clocks;
	struct reset_control **resets;
	enum tegra_cam_rtcpu_id id;
	u32 sid;
	u32 num_clocks;
	u32 num_resets;
};

static const struct tegra_cam_rtcpu_pdata sce_pdata = {
	.clock_names = sce_clock_names,
	.reset_names = sce_reset_names,
	.clocks = sce_clocks,
	.resets = sce_resets,
	.id = TEGRA_CAM_RTCPU_SCE,
	.sid = TEGRA_SID_SCE,
	.num_clocks = ARRAY_SIZE(sce_clock_names),
	.num_resets = ARRAY_SIZE(sce_reset_names),
};

static const struct tegra_cam_rtcpu_pdata ape_pdata = {
	.clock_names = ape_clock_names,
	.reset_names = ape_reset_names,
	.clocks = ape_clocks,
	.resets = ape_resets,
	.id = TEGRA_CAM_RTCPU_APE,
	.sid = TEGRA_SID_APE,
	.num_clocks = ARRAY_SIZE(ape_clock_names),
	.num_resets = ARRAY_SIZE(ape_reset_names),
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

struct tegra_cam_rtcpu {
	struct tegra_ivc_bus *ivc;
	struct {
		struct mutex mutex;
		struct tegra_hsp_sm_pair pair;
		wait_queue_head_t response_waitq;
		wait_queue_head_t empty_waitq;
		atomic_t response;
		atomic_t emptied;
	} cmd;
	union {
		struct {
			void __iomem *sce_cfg_base;
			void __iomem *sce_pm_base;
		} __packed rtcpu_sce;
	};
	const struct tegra_cam_rtcpu_pdata *rtcpu_pdata;
};

static int tegra_cam_rtcpu_get_clks_resets(struct device *dev)
{
	struct tegra_cam_rtcpu *cam_rtcpu;
	const struct tegra_cam_rtcpu_pdata *pdata;
	int i;

	cam_rtcpu = dev_get_drvdata(dev);
	pdata = cam_rtcpu->rtcpu_pdata;

	/* Get clocks and resets */
	for (i = 0; i < pdata->num_clocks; i++) {
		pdata->clocks[i] = devm_clk_get(dev, pdata->clock_names[i]);
		if (IS_ERR(pdata->clocks[i])) {
			if (PTR_ERR(pdata->clocks[i]) == -EPROBE_DEFER)
				return PTR_ERR(pdata->clocks[i]);

			dev_err(dev, "clock %s not found: %ld\n",
				pdata->clock_names[i],
				PTR_ERR(pdata->clocks[i]));
		}
	}

	for (i = 0; i < pdata->num_resets; i++) {
		pdata->resets[i] =
			devm_reset_control_get(dev, pdata->reset_names[i]);
		if (IS_ERR(pdata->resets[i])) {
			if (PTR_ERR(pdata->resets[i]) == -EPROBE_DEFER)
				return PTR_ERR(pdata->resets[i]);

			dev_err(dev, "reset %s not found: %ld\n",
				pdata->reset_names[i],
				PTR_ERR(pdata->resets[i]));
		}
	}

	return 0;
}

static int tegra_cam_rtcpu_apply_clks(struct device *dev,
				int (*func)(struct clk *clk))
{
	struct tegra_cam_rtcpu *cam_rtcpu;
	const struct tegra_cam_rtcpu_pdata *pdata;
	int i, ret;

	cam_rtcpu = dev_get_drvdata(dev);
	pdata = cam_rtcpu->rtcpu_pdata;

	for (i = 0; i < pdata->num_clocks; i++) {
		if (IS_ERR(pdata->clocks[i]))
			continue;
		ret = (*func)(pdata->clocks[i]);
		if (ret) {
			dev_err(dev, "clock %s failed: %d\n",
				pdata->clock_names[i], ret);
		}
	}

	return 0;
}

static int tegra_cam_rtcpu_apply_resets(struct device *dev,
			int (*func)(struct reset_control *))
{
	struct tegra_cam_rtcpu *cam_rtcpu;
	const struct tegra_cam_rtcpu_pdata *pdata;
	int i, ret;

	cam_rtcpu = dev_get_drvdata(dev);
	pdata = cam_rtcpu->rtcpu_pdata;

	for (i = 0; i < pdata->num_resets; i++) {
		if (IS_ERR(pdata->resets[i]))
			continue;
		ret = (*func)(pdata->resets[i]);
		if (ret) {
			dev_err(dev, "reset %s failed: %d\n",
				pdata->reset_names[i], ret);
		}
	}

	return 0;
}

int tegra_camrtc_reset(struct device *dev)
{
	return tegra_cam_rtcpu_apply_resets(dev, reset_control_reset);
}
EXPORT_SYMBOL(tegra_camrtc_reset);

int tegra_camrtc_set_halt(struct device *dev, bool halt)
{
	struct tegra_cam_rtcpu *cam_rtcpu = dev_get_drvdata(dev);
	u32 reg_val;

	if (cam_rtcpu->rtcpu_pdata->id == TEGRA_CAM_RTCPU_SCE) {
		reg_val = readl(cam_rtcpu->rtcpu_sce.sce_pm_base +
				TEGRA_SCEPM_R5_CTRL_0);

		if (halt) {
			reg_val &= ~TEGRA_SCE_FWLOADDONE;
		} else {
			/* Set FW load done bit to unhalt */
			reg_val |= TEGRA_SCE_FWLOADDONE;
		}

		dev_info(dev, "sce gets %s", halt ? "halted" : "unhalted");

		writel(reg_val, cam_rtcpu->rtcpu_sce.sce_pm_base +
		       TEGRA_SCEPM_R5_CTRL_0);

		return 0;
	} else {
		dev_emerg(dev, "don't know how to halt/unhalt APE");

		return -ENOSYS;
	}
}
EXPORT_SYMBOL(tegra_camrtc_set_halt);

int tegra_camrtc_get_halt(struct device *dev, bool *halt)
{
	struct tegra_cam_rtcpu *cam_rtcpu = dev_get_drvdata(dev);
	u32 reg_val;

	if (dev == NULL)
		return -EINVAL;

	if (cam_rtcpu->rtcpu_pdata->id == TEGRA_CAM_RTCPU_SCE) {
		reg_val = readl(cam_rtcpu->rtcpu_sce.sce_pm_base +
				TEGRA_SCEPM_R5_CTRL_0);

		*halt = (reg_val & TEGRA_SCE_FWLOADDONE) == 0;

		dev_info(dev, "sce is %s", *halt ? "halted" : "unhalted");

		return 0;
	} else {
		dev_emerg(dev, "don't know how to halt/unhalt APE");

		return -ENOSYS;
	}
}
EXPORT_SYMBOL(tegra_camrtc_get_halt);

void tegra_camrtc_ready(struct device *dev)
{
	struct tegra_cam_rtcpu *cam_rtcpu = dev_get_drvdata(dev);

	if (cam_rtcpu->rtcpu_pdata->id == TEGRA_CAM_RTCPU_SCE) {
		/* Disable SCE R5R and smartcomp in camera mode */
		writel(TEGRA_SCE_R5R_SC_DISABLE,
			cam_rtcpu->rtcpu_sce.sce_cfg_base +
					TEGRA_APS_FRSC_SC_CTL_0);

		/* Enable JTAG/Coresight */
		writel(TEGRA_SCE_FN_MODEIN,
			cam_rtcpu->rtcpu_sce.sce_cfg_base +
					TEGRA_SCE_APS_FRSC_SC_MODEIN_0);
	}
}
EXPORT_SYMBOL(tegra_camrtc_ready);

static u32 tegra_camrtc_full_notify(struct tegra_hsp_sm_pair *pair,
				u32 response)
{
	struct tegra_cam_rtcpu *cam_rtcpu;

	cam_rtcpu = container_of(pair, struct tegra_cam_rtcpu, cmd.pair);

	atomic_set(&cam_rtcpu->cmd.response, response);

	wake_up(&cam_rtcpu->cmd.response_waitq);

	return 0;
}

static void tegra_camrtc_empty_notify(struct tegra_hsp_sm_pair *pair,
				u32 empty_value)
{
	struct tegra_cam_rtcpu *cam_rtcpu;

	cam_rtcpu = container_of(pair, struct tegra_cam_rtcpu, cmd.pair);

	atomic_set(&cam_rtcpu->cmd.emptied, 1);

	wake_up(&cam_rtcpu->cmd.empty_waitq);
}

static long tegra_camrtc_wait_for_empty(struct device *dev,
				long timeout)
{
	struct tegra_cam_rtcpu *cam_rtcpu = dev_get_drvdata(dev);

	if (timeout == 0)
		timeout = 2 * HZ;

	timeout = wait_event_interruptible_timeout(
		cam_rtcpu->cmd.empty_waitq,
		/* Make sure IRQ has been handled */
		atomic_read(&cam_rtcpu->cmd.emptied) != 0 &&
		tegra_hsp_sm_pair_is_empty(&cam_rtcpu->cmd.pair),
		timeout);

	if (timeout > 0)
		atomic_set(&cam_rtcpu->cmd.emptied, 0);

	return timeout;
}

static int tegra_camrtc_command(struct device *dev, u32 command,
				long timeout)
{
	struct tegra_cam_rtcpu *cam_rtcpu = dev_get_drvdata(dev);
	int response;

#define INVALID_RESPONSE (0x80000000U)

	if (timeout == 0)
		timeout = 2 * HZ;

	mutex_lock(&cam_rtcpu->cmd.mutex);

	timeout = tegra_camrtc_wait_for_empty(dev, timeout);
	if (timeout <= 0) {
		dev_err(dev, "Timed out waiting for empty mailbox");
		response = -ETIMEDOUT;
		goto done;
	}

	atomic_set(&cam_rtcpu->cmd.response, INVALID_RESPONSE);

	tegra_hsp_sm_pair_write(&cam_rtcpu->cmd.pair, command);

	timeout = wait_event_interruptible_timeout(
		cam_rtcpu->cmd.response_waitq,
		atomic_read(&cam_rtcpu->cmd.response) != INVALID_RESPONSE,
		timeout);
	if (timeout <= 0) {
		dev_err(dev, "Timed out waiting for response");
		response = -ETIMEDOUT;
		goto done;
	}

	response = (int)atomic_read(&cam_rtcpu->cmd.response);

done:
	mutex_unlock(&cam_rtcpu->cmd.mutex);

	return response;
}

int tegra_camrtc_boot(struct device *dev)
{
	struct tegra_cam_rtcpu *cam_rtcpu = dev_get_drvdata(dev);
	int ret;

	if (tegra_camrtc_wait_for_empty(dev, 0) <= 0) {
		dev_err(dev, "Timeout for HSP SM empty interrupt");
		return -ETIMEDOUT;
	}

	/*
	 * Intermediate firmware skips negotiation unless there is a
	 * command in mailbox when RTCPU gets unhalted.
	 *
	 * Insert a random command (INIT 16) to mailbox before
	 * unhalting.
	 */
	tegra_hsp_sm_pair_write(&cam_rtcpu->cmd.pair,
		RTCPU_COMMAND(INIT, 16));

	tegra_camrtc_ready(dev);

	if (cam_rtcpu->rtcpu_pdata->id == TEGRA_CAM_RTCPU_SCE) {
		/* Unhalt SCE */
		tegra_camrtc_set_halt(dev, false);

		dev_info(dev, "booting SCE with Camera RTCPU FW");
	} else {
		dev_info(dev, "booting APE with Camera RTCPU FW");
	}

	/*
	 * Handshake FW version before continueing with the boot
	 */
	ret = tegra_camrtc_command(dev,
		RTCPU_COMMAND(INIT, 0), 0);
	if (ret < 0)
		return ret;

	if (ret != RTCPU_COMMAND(INIT, 0)) {
		dev_err(dev, "RTCPU sync problem (response=0x%08x)", ret);
		return -EIO;
	}

	ret = tegra_camrtc_command(dev,
		RTCPU_COMMAND(FW_VERSION, RTCPU_FW_VERSION), 0);
	if (ret < 0)
		return ret;

	if (ret != RTCPU_COMMAND(FW_VERSION, RTCPU_FW_VERSION)) {
		dev_err(dev, "RTCPU version mismatch (response=0x%08x)", ret);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_camrtc_boot);

int tegra_camrtc_ivc_setup_ready(struct device *dev)
{
	u32 command = RTCPU_COMMAND(IVC_READY, RTCPU_FW_VERSION);
	int ret;

	ret = tegra_camrtc_command(dev, command, 0);
	if (ret < 0)
		return ret;

	if (ret != command) {
		dev_err(dev, "IVC setup problem (response=0x%08x)", ret);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_camrtc_ivc_setup_ready);

static int tegra_cam_rtcpu_probe(struct platform_device *pdev)
{
	struct tegra_cam_rtcpu *cam_rtcpu;
	struct device *dev = &pdev->dev;
	struct device_node *hsp_node;
	int ret;
	const struct of_device_id *match;

	dev_dbg(dev, "probing\n");

	cam_rtcpu = devm_kzalloc(dev, sizeof(*cam_rtcpu), GFP_KERNEL);
	if (!cam_rtcpu)
		return -ENOMEM;
	platform_set_drvdata(pdev, cam_rtcpu);

	match = of_match_device(tegra_cam_rtcpu_of_match, dev);
	if (match == NULL) {
		dev_err(dev, "Device match not found\n");
		return -ENODEV;
	}

	cam_rtcpu->rtcpu_pdata = match->data;

	if (cam_rtcpu->rtcpu_pdata->id == TEGRA_CAM_RTCPU_SCE) {
		/* EVP addresses are needed to set up the reset vector */
		/* EVP should be mapped if the (APE) FW is reloaded by kernel */

		/* SCE PM addr */
		cam_rtcpu->rtcpu_sce.sce_pm_base =
			tegra_ast_map_byname(dev, "sce-pm");
		if (!cam_rtcpu->rtcpu_sce.sce_pm_base) {
			dev_err(dev, "failed to map SCE PM space.\n");
			return -EINVAL;
		}

		/* SCE CFG addr */
		cam_rtcpu->rtcpu_sce.sce_cfg_base =
			tegra_ast_map_byname(dev, "sce-cfg");
		if (!cam_rtcpu->rtcpu_sce.sce_cfg_base) {
			dev_err(dev, "failed to map SCE CFG space.\n");
			return -EINVAL;
		}
	}

	ret = tegra_cam_rtcpu_get_clks_resets(dev);
	if (ret) {
		dev_err(dev, "failed to get clocks/resets: %d\n", ret);
		return ret;
	}

	ret = tegra_cam_rtcpu_apply_clks(dev, clk_prepare_enable);
	if (ret) {
		dev_err(dev, "failed to turn on clocks: %d\n", ret);
		return ret;
	}

	tegra_cam_rtcpu_apply_resets(dev, reset_control_deassert);

	mutex_init(&cam_rtcpu->cmd.mutex);
	init_waitqueue_head(&cam_rtcpu->cmd.response_waitq);
	init_waitqueue_head(&cam_rtcpu->cmd.empty_waitq);
	cam_rtcpu->cmd.pair.notify_full = tegra_camrtc_full_notify;
	cam_rtcpu->cmd.pair.notify_empty = tegra_camrtc_empty_notify;
	hsp_node = of_get_child_by_name(dev->of_node, "hsp");
	ret = of_tegra_hsp_sm_pair_by_name(hsp_node, "cmd-pair",
					&cam_rtcpu->cmd.pair);
	of_node_put(hsp_node);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to obtain mbox pair: %d\n", ret);
		return ret;
	}

	ret = tegra_camrtc_boot(dev);
	if (ret)
		goto fail;

	cam_rtcpu->ivc = tegra_ivc_bus_create(dev,
						cam_rtcpu->rtcpu_pdata->sid);
	if (IS_ERR(cam_rtcpu->ivc)) {
		ret = PTR_ERR(cam_rtcpu->ivc);
		goto fail;
	}

	ret = tegra_camrtc_ivc_setup_ready(dev);
	if (ret)
		goto fail;

	dev_dbg(dev, "probe successful\n");
	return 0;

fail:
	tegra_ivc_bus_destroy(cam_rtcpu->ivc);
	tegra_hsp_sm_pair_free(&cam_rtcpu->cmd.pair);
	return ret;
}


static int tegra_cam_rtcpu_remove(struct platform_device *pdev)
{
	struct tegra_cam_rtcpu *cam_rtcpu = platform_get_drvdata(pdev);

	tegra_ivc_bus_destroy(cam_rtcpu->ivc);
	tegra_hsp_sm_pair_free(&cam_rtcpu->cmd.pair);

	return 0;
}

static int tegra_cam_rtcpu_resume(struct device *dev)
{
	/*
	 * TODO:
	 * Call tegra_cam_rtcpu_clks_resets_on() and tegra_cam_rtcpu_boot()?
	 */

	return 0;
}

static int tegra_cam_rtcpu_suspend(struct device *dev)
{
	/*
	 * TODO:
	 *
	 * 1. Signal(through new control channel) RTCPU to finish off pending
	 *    tasks and assert standbyWFI.
	 * 2. Wait until acknowledgement from RTCPU for standbyWFI assert.
	 * 3. Once ack recieved check SCEPM_PWR_STATUS_0 register for
	 *    WFIPIPESTOPPED to confirm standbyWFI assert is done.
	 * 4. If OK proceed for SC7.
	 * 5. Disable clocks and assert resets.
	 */

	return 0;
}

static const struct dev_pm_ops tegra_cam_rtcpu_pm_ops = {
	.suspend = tegra_cam_rtcpu_suspend,
	.resume = tegra_cam_rtcpu_resume,
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
};
module_platform_driver(tegra_cam_rtcpu_driver);

MODULE_DESCRIPTION("CAMERA RTCPU driver");
MODULE_AUTHOR("NVIDIA");
MODULE_LICENSE("GPL v2");
