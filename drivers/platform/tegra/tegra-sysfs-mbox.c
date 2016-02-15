/*
 * Tegra sysfs mbox driver for IVC channels
 *
 * Copyright (c) 2015-2016 NVIDIA Corporation.  All rights reserved.
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

#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/tegra-ivc-bus.h>

#define NV(p) "nvidia," #p

struct sysfs_msg {
	struct tegra_ivc_mbox_msg header;
	u8 data[];
};

struct sysfs_mbox {
	struct mbox_client client;
	struct mbox_chan *mbox;
	size_t frame_size;
	struct sysfs_msg *rx;
};


static void sysfs_mbox_rx(struct mbox_client *cl, void *data)
{
	struct sysfs_mbox *tsm;
	struct tegra_ivc_mbox_msg *msg = data;

	pr_debug("rtcpu mbox rx msg\n");
	print_hex_dump(KERN_DEBUG, "", DUMP_PREFIX_OFFSET,
			16, 1, msg->data, msg->length, true);

	tsm = container_of(cl, struct sysfs_mbox, client);

	tsm->rx->header.length = msg->length;
	tsm->rx->header.data = tsm->rx->data;

	memcpy(tsm->rx->data, msg->data, msg->length);
}

static void sysfs_mbox_tx_done(struct mbox_client *cl, void *data, int err)
{
	if (!err)
		pr_debug("rtcpu mbox echo tx done\n");
	else
		pr_err("rtcpu mbox echo tx error: %d\n", err);

	kfree(data);
}

static ssize_t sysfs_mbox_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct sysfs_mbox *tsm = dev_get_drvdata(dev);
	struct tegra_ivc_mbox_msg *msg = &tsm->rx->header;
	ssize_t count = msg->length;

	msg->length = 0;

	memcpy(buf, msg->data, count);

	return count;
}

static ssize_t sysfs_mbox_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sysfs_mbox *tsm = dev_get_drvdata(dev);
	struct sysfs_msg *msg;
	size_t msize = offsetof(struct sysfs_msg, data[count]);
	int ret;

	if (count > tsm->frame_size) {
		dev_err(dev, "data size %zu > mbox frame size %zu\n",
			count, tsm->frame_size);
		return -EINVAL;
	}

	msg = kzalloc(msize, GFP_KERNEL);
	if (unlikely(msg == NULL))
		return -ENOMEM;

	memcpy(msg->data, buf, count);

	msg->header.length = count;
	msg->header.data = msg->data;

	dev_dbg(dev, "mbox send\n");
	print_hex_dump(KERN_DEBUG, "", DUMP_PREFIX_OFFSET,
			16, 1, buf, count, true);

	ret = mbox_send_message(tsm->mbox, &msg->header);

	if (ret >= 0)
		ret = count;
	else if (ret == -EINVAL)
		kfree(msg);

	return ret;
}

static ssize_t sysfs_mbox_get_ivc_frame_size(struct device *dev)
{
	struct of_phandle_args spec;
	struct device_node *snp, *chnp;
	int n, ret;
	u32 frame_size;

	if (of_parse_phandle_with_args(dev->of_node, "mboxes",
				"#mbox-cells", 0, &spec)) {
		dev_err(dev, "can't parse <%s> property\n", "mboxes");
		return -ENODEV;
	}

	n = spec.args[0];
	chnp = NULL;

	for_each_child_of_node(spec.np, snp) {
		if (strcmp(snp->name, "ivc-channels"))
			continue;

		for_each_child_of_node(snp, chnp) {
			if (n-- == 0)
				break;
		}

		if (chnp)
			break;
	}

	if (chnp == NULL) {
		dev_err(dev, "ivc-channel not found\n");
		return -EPROBE_DEFER;
	}

	ret = of_property_read_u32(chnp, NV(frame-size), &frame_size);
	if (ret) {
		dev_err(dev, "can't parse <%s> property\n", NV(frame-size));
		dev_err(dev, "parse on %s\n", chnp->name);
		return ret;
	}

	dev_dbg(dev, "<%s> is %u\n", NV(frame-size), frame_size);

	if (frame_size > PAGE_SIZE)
		return -EINVAL;

	return frame_size;
}

static const DEVICE_ATTR(mbox, 0600, sysfs_mbox_show, sysfs_mbox_store);

/* Platform device */
static int sysfs_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sysfs_mbox *tsm;
	ssize_t frame_size;

	tsm = devm_kzalloc(dev, sizeof(*tsm), GFP_KERNEL);
	if (unlikely(tsm == NULL))
		return -ENOMEM;

	tsm->client.dev = dev;
	tsm->client.tx_block = true;
	tsm->client.tx_tout = 500;
	tsm->client.knows_txdone = false;
	tsm->client.rx_callback = sysfs_mbox_rx;
	tsm->client.tx_done = sysfs_mbox_tx_done;

	frame_size = sysfs_mbox_get_ivc_frame_size(dev);
	if (frame_size < 0)
		return frame_size;

	tsm->frame_size = frame_size;

	tsm->rx = devm_kzalloc(dev,
		offsetof(struct sysfs_msg, data[frame_size]), GFP_KERNEL);
	if (unlikely(tsm->rx == NULL))
		return -ENOMEM;

	tsm->mbox = mbox_request_channel(&tsm->client, 0);
	if (IS_ERR(tsm->mbox)) {
		dev_err(dev, "no mbox found");
		return PTR_ERR(tsm->mbox);
	}

	dev_set_drvdata(dev, tsm);

	device_create_file(dev, &dev_attr_mbox);

	return 0;
}

static int __exit sysfs_mbox_remove(struct platform_device *pdev)
{
	struct sysfs_mbox *tsm = dev_get_drvdata(&pdev->dev);

	mbox_free_channel(tsm->mbox);

	return 0;
}

static const struct of_device_id sysfs_mbox_of_match[] = {
	{ .compatible = "nvidia,tegra186-sysfs-mbox" },
	{ },
};

static struct platform_driver sysfs_mbox_driver = {
	.probe = sysfs_mbox_probe,
	.remove = __exit_p(sysfs_mbox_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "tegra-sysfs-mbox",
		.of_match_table = sysfs_mbox_of_match,
	},
};

static int __init sysfs_mbox_init(void)
{
	return platform_driver_register(&sysfs_mbox_driver);
}

static void __exit sysfs_mbox_exit(void)
{
	platform_driver_unregister(&sysfs_mbox_driver);
}

module_init(sysfs_mbox_init);
module_exit(sysfs_mbox_exit);
