/*
 *
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/sysfs.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/tegra-hsp.h>
#include <dt-bindings/thermal/tegra194-soctherm.h>
#include <soc/tegra/chip-id.h>

#define REG_BANK_SIZE				0x30
#define CPU_REG_OFFSET				0x30
#define GPU_REG_OFFSET				0x38
#define PRIORITY_REG_OFFSET			0x44
#define THROTTLE_CTRL_BASE			0x500

#define CPU_OFFSET(i)				(THROTTLE_CTRL_BASE + ((i) * REG_BANK_SIZE) + \
										CPU_REG_OFFSET)
#define GPU_OFFSET(i)				(THROTTLE_CTRL_BASE + ((i) * REG_BANK_SIZE) + \
										GPU_REG_OFFSET)
#define PRIORITY_OFFSET(i)			(THROTTLE_CTRL_BASE + ((i) * REG_BANK_SIZE) + \
										PRIORITY_REG_OFFSET)

#define EDP_OC_OC1_STATS_0			0x4a8
#define EDP_OC_STATS(i)				(EDP_OC_OC1_STATS_0 + ((i) * 4))

#define EDP_OC_OC1_THRESH_CNT_0		0x414
#define EDP_OC_THRESH_CNT(i)		(EDP_OC_OC1_THRESH_CNT_0 + ((i) * 0x14))

#define EDP_OC_THROT_VEC_CNT		SOCTHERM_THROT_VEC_INVALID

struct throttlectrl_info {
	unsigned int priority;
	unsigned int cpu_depth;
	unsigned int gpu_depth;
};

struct edp_oc_info {
	unsigned int id;
	unsigned int irq_cnt;
};

struct tegra_oc_event {
	struct device *hwmon;
	void __iomem *hsp_base;
	int32_t irq;
	void __iomem *soctherm_base;
	struct throttlectrl_info throttle_ctrl[EDP_OC_THROT_VEC_CNT];
	struct edp_oc_info edp_oc[EDP_OC_THROT_VEC_CNT];
};

static struct tegra_oc_event tegra_oc;

static unsigned int tegra_oc_readl(unsigned int offset)
{
	return __raw_readl(tegra_oc.soctherm_base + offset);
}

static irqreturn_t tegra_oc_event_raised(int irq, void *arg)
{
	int i;

	/* Write 0 to TAG bit for HSP SM acknowledge */
	__raw_writel(0, tegra_oc.hsp_base);

	/* Read all oc stats registers */
	for (i = 0; i < SOCTHERM_EDP_OC_INVALID; i++) {
		tegra_oc.edp_oc[i].irq_cnt = tegra_oc_readl(EDP_OC_STATS(i)) /
									(tegra_oc_readl(EDP_OC_THRESH_CNT(i)) + 1);
	}

	return IRQ_NONE;
}

static void tegra_get_throtctrl_vectors(void)
{
	int i;

	for (i = 0; i < SOCTHERM_THROT_VEC_INVALID; i++) {
		tegra_oc.throttle_ctrl[i].priority = tegra_oc_readl(PRIORITY_OFFSET(i));
		tegra_oc.throttle_ctrl[i].cpu_depth = tegra_oc_readl(CPU_OFFSET(i));
		tegra_oc.throttle_ctrl[i].gpu_depth = tegra_oc_readl(GPU_OFFSET(i));
	}
}

static ssize_t irq_count_show(struct device *dev, struct device_attribute *attr,
								char *buf)
{
	struct sensor_device_attribute *sensor_attr = container_of(attr,
			struct sensor_device_attribute, dev_attr);

	return sprintf(buf, "%u\n", tegra_oc.edp_oc[sensor_attr->index].irq_cnt);
}

static ssize_t priority_show(struct device *dev, struct device_attribute *attr,
								char *buf)
{
	struct sensor_device_attribute *sensor_attr = container_of(attr,
			struct sensor_device_attribute, dev_attr);

	return sprintf(buf, "%u\n",
			tegra_oc.throttle_ctrl[sensor_attr->index].priority);
}

static ssize_t cpu_thrtl_ctrl_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sensor_device_attribute *sensor_attr = container_of(attr,
			struct sensor_device_attribute, dev_attr);

	return sprintf(buf, "%u\n",
			tegra_oc.throttle_ctrl[sensor_attr->index].cpu_depth);
}

static ssize_t gpu_thrtl_ctrl_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sensor_device_attribute *sensor_attr = container_of(attr,
			struct sensor_device_attribute, dev_attr);

	return sprintf(buf, "%u\n",
			tegra_oc.throttle_ctrl[sensor_attr->index].gpu_depth);
}

static SENSOR_DEVICE_ATTR(oc1_irq_cnt, 0444, irq_count_show, NULL, SOCTHERM_EDP_OC1);
static SENSOR_DEVICE_ATTR(oc1_priority, 0444, priority_show, NULL, SOCTHERM_EDP_OC1);
static SENSOR_DEVICE_ATTR(oc1_cpu_throttle_ctrl, 0444, cpu_thrtl_ctrl_show, NULL,
								SOCTHERM_EDP_OC1);
static SENSOR_DEVICE_ATTR(oc1_gpu_throttle_ctrl, 0444, gpu_thrtl_ctrl_show, NULL,
								SOCTHERM_EDP_OC1);

static struct attribute *t194_oc1_attrs[] = {
	&sensor_dev_attr_oc1_irq_cnt.dev_attr.attr,
	&sensor_dev_attr_oc1_priority.dev_attr.attr,
	&sensor_dev_attr_oc1_cpu_throttle_ctrl.dev_attr.attr,
	&sensor_dev_attr_oc1_gpu_throttle_ctrl.dev_attr.attr,
	NULL,
};

static const struct attribute_group t194_oc1_data = {
	.attrs = t194_oc1_attrs,
	NULL,
};

static SENSOR_DEVICE_ATTR(oc2_irq_cnt, 0444, irq_count_show, NULL, SOCTHERM_EDP_OC2);
static SENSOR_DEVICE_ATTR(oc2_priority, 0444, priority_show, NULL, SOCTHERM_EDP_OC2);
static SENSOR_DEVICE_ATTR(oc2_cpu_throttle_ctrl, 0444, cpu_thrtl_ctrl_show, NULL,
								SOCTHERM_EDP_OC2);
static SENSOR_DEVICE_ATTR(oc2_gpu_throttle_ctrl, 0444, gpu_thrtl_ctrl_show, NULL,
								SOCTHERM_EDP_OC2);

static struct attribute *t194_oc2_attrs[] = {
	&sensor_dev_attr_oc2_irq_cnt.dev_attr.attr,
	&sensor_dev_attr_oc2_priority.dev_attr.attr,
	&sensor_dev_attr_oc2_cpu_throttle_ctrl.dev_attr.attr,
	&sensor_dev_attr_oc2_gpu_throttle_ctrl.dev_attr.attr,
	NULL,
};

static const struct attribute_group t194_oc2_data = {
	.attrs = t194_oc2_attrs,
	NULL,
};

static SENSOR_DEVICE_ATTR(oc3_irq_cnt, 0444, irq_count_show, NULL, SOCTHERM_EDP_OC3);
static SENSOR_DEVICE_ATTR(oc3_priority, 0444, priority_show, NULL, SOCTHERM_EDP_OC3);
static SENSOR_DEVICE_ATTR(oc3_cpu_throttle_ctrl, 0444, cpu_thrtl_ctrl_show, NULL,
								SOCTHERM_EDP_OC3);
static SENSOR_DEVICE_ATTR(oc3_gpu_throttle_ctrl, 0444, gpu_thrtl_ctrl_show, NULL,
								SOCTHERM_EDP_OC3);

static struct attribute *t194_oc3_attrs[] = {
	&sensor_dev_attr_oc3_irq_cnt.dev_attr.attr,
	&sensor_dev_attr_oc3_priority.dev_attr.attr,
	&sensor_dev_attr_oc3_cpu_throttle_ctrl.dev_attr.attr,
	&sensor_dev_attr_oc3_gpu_throttle_ctrl.dev_attr.attr,
	NULL,
};

static const struct attribute_group t194_oc3_data = {
	.attrs = t194_oc3_attrs,
	NULL,
};

static SENSOR_DEVICE_ATTR(oc4_irq_cnt, 0444, irq_count_show, NULL, SOCTHERM_EDP_OC4);
static SENSOR_DEVICE_ATTR(oc4_priority, 0444, priority_show, NULL, SOCTHERM_EDP_OC4);
static SENSOR_DEVICE_ATTR(oc4_cpu_throttle_ctrl, 0444, cpu_thrtl_ctrl_show, NULL,
								SOCTHERM_EDP_OC4);
static SENSOR_DEVICE_ATTR(oc4_gpu_throttle_ctrl, 0444, gpu_thrtl_ctrl_show, NULL,
								SOCTHERM_EDP_OC4);

static struct attribute *t194_oc4_attrs[] = {
	&sensor_dev_attr_oc4_irq_cnt.dev_attr.attr,
	&sensor_dev_attr_oc4_priority.dev_attr.attr,
	&sensor_dev_attr_oc4_cpu_throttle_ctrl.dev_attr.attr,
	&sensor_dev_attr_oc4_gpu_throttle_ctrl.dev_attr.attr,
	NULL,
};

static const struct attribute_group t194_oc4_data = {
	.attrs = t194_oc4_attrs,
	NULL,
};

static SENSOR_DEVICE_ATTR(oc5_irq_cnt, 0444, irq_count_show, NULL, SOCTHERM_EDP_OC5);
static SENSOR_DEVICE_ATTR(oc5_priority, 0444, priority_show, NULL, SOCTHERM_EDP_OC5);
static SENSOR_DEVICE_ATTR(oc5_cpu_throttle_ctrl, 0444, cpu_thrtl_ctrl_show, NULL,
								SOCTHERM_EDP_OC5);
static SENSOR_DEVICE_ATTR(oc5_gpu_throttle_ctrl, 0444, gpu_thrtl_ctrl_show, NULL,
								SOCTHERM_EDP_OC5);

static struct attribute *t194_oc5_attrs[] = {
	&sensor_dev_attr_oc5_irq_cnt.dev_attr.attr,
	&sensor_dev_attr_oc5_priority.dev_attr.attr,
	&sensor_dev_attr_oc5_cpu_throttle_ctrl.dev_attr.attr,
	&sensor_dev_attr_oc5_gpu_throttle_ctrl.dev_attr.attr,
	NULL,
};

static const struct attribute_group t194_oc5_data = {
	.attrs = t194_oc5_attrs,
	NULL,
};

static SENSOR_DEVICE_ATTR(oc6_irq_cnt, 0444, irq_count_show, NULL, SOCTHERM_EDP_OC6);
static SENSOR_DEVICE_ATTR(oc6_priority, 0444, priority_show, NULL, SOCTHERM_EDP_OC6);
static SENSOR_DEVICE_ATTR(oc6_cpu_throttle_ctrl, 0444, cpu_thrtl_ctrl_show, NULL,
								SOCTHERM_EDP_OC6);
static SENSOR_DEVICE_ATTR(oc6_gpu_throttle_ctrl, 0444, gpu_thrtl_ctrl_show, NULL,
								SOCTHERM_EDP_OC6);

static struct attribute *t194_oc6_attrs[] = {
	&sensor_dev_attr_oc6_irq_cnt.dev_attr.attr,
	&sensor_dev_attr_oc6_priority.dev_attr.attr,
	&sensor_dev_attr_oc6_cpu_throttle_ctrl.dev_attr.attr,
	&sensor_dev_attr_oc6_gpu_throttle_ctrl.dev_attr.attr,
	NULL,
};

static const struct attribute_group t194_oc6_data = {
	.attrs = t194_oc6_attrs,
	NULL,
};

static const struct attribute_group *t194_oc_groups[] = {
	&t194_oc1_data,
	&t194_oc2_data,
	&t194_oc3_data,
	&t194_oc4_data,
	&t194_oc5_data,
	&t194_oc6_data,
	NULL,
};

static const struct of_device_id tegra_oc_event_of_match[] = {
	{ .compatible = "nvidia,tegra194-oc-event",
		.data = (void *)&t194_oc_groups
	},
	{}
};
MODULE_DEVICE_TABLE(of, tegra_oc_event_of_match);

static int tegra_oc_event_remove(struct platform_device *pdev)
{
	if (tegra_platform_is_silicon()) {
		iounmap(tegra_oc.soctherm_base);
		iounmap(tegra_oc.hsp_base);
		devm_hwmon_device_unregister(tegra_oc.hwmon);
	}
	dev_info(&pdev->dev, "remove\n");

	return 0;
}

static int tegra_oc_event_probe(struct platform_device *pdev)
{
	int ret;
	unsigned long flags = IRQF_ONESHOT | IRQF_SHARED | IRQF_PROBE_SHARED;
	const struct of_device_id *match;
	struct device_node *np = pdev->dev.of_node;

	match = of_match_node(tegra_oc_event_of_match, np);
	if (!match)
		return -ENODEV;

	if (tegra_platform_is_silicon()) {
		tegra_oc.soctherm_base = of_iomap(pdev->dev.of_node, 0);
		if (!tegra_oc.soctherm_base) {
			dev_err(&pdev->dev, "Unable to map soctherm register memory");
			return PTR_ERR(tegra_oc.soctherm_base);
		}

		tegra_oc.hsp_base = of_iomap(pdev->dev.of_node, 1);
		if (!tegra_oc.hsp_base) {
			dev_err(&pdev->dev, "Unable to map hsp register memory");
			iounmap(tegra_oc.soctherm_base);
			return PTR_ERR(tegra_oc.soctherm_base);
		}

		tegra_get_throtctrl_vectors();

		tegra_oc.hwmon = devm_hwmon_device_register_with_groups(&pdev->dev,
				"soctherm_oc", &tegra_oc,
				(const struct attribute_group **)match->data);
		if (IS_ERR(tegra_oc.hwmon)) {
			dev_err(&pdev->dev, "Failed to register hwmon device\n");
			iounmap(tegra_oc.soctherm_base);
			iounmap(tegra_oc.hsp_base);
			return PTR_ERR(tegra_oc.hwmon);
		}

		tegra_oc.irq = platform_get_irq(pdev, 0);
		ret = request_irq(tegra_oc.irq, tegra_oc_event_raised,
				flags, "tegra-oc-event", &tegra_oc);
		if (ret < 0) {
			iounmap(tegra_oc.soctherm_base);
			iounmap(tegra_oc.hsp_base);
			devm_hwmon_device_unregister(tegra_oc.hwmon);
			dev_err(&pdev->dev, "Failed to request irq\n");
			return ret;
		}
	}

	dev_info(&pdev->dev, "OC driver initialized");
	return 0;
}

static struct platform_driver tegra_oc_event_driver = {
	.driver = {
		.name = "tegra-oc-event",
		.owner = THIS_MODULE,
		.of_match_table = tegra_oc_event_of_match,
	},
	.probe = tegra_oc_event_probe,
	.remove = tegra_oc_event_remove,
};

module_platform_driver(tegra_oc_event_driver);
MODULE_AUTHOR("Mantravadi Karthik <mkarthik@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra Over Current Event Driver");
MODULE_LICENSE("GPL v2");
