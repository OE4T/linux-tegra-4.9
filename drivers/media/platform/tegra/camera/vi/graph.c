/*
 * NVIDIA Media controller graph management
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Bryan Wu <pengw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <soc/tegra/chip-id.h>
#include <linux/tegra_pm_domains.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-of.h>
#include <media/tegra_v4l2_camera.h>
#include <media/mc_common.h>
#include <media/csi.h>

#include "nvcsi/nvcsi.h"

/* -----------------------------------------------------------------------------
 * Graph Management
 */

static struct tegra_vi_graph_entity *
tegra_vi_graph_find_entity(struct tegra_mc_vi *vi,
		       const struct device_node *node)
{
	struct tegra_vi_graph_entity *entity;

	list_for_each_entry(entity, &vi->entities, list) {
		if (entity->node == node)
			return entity;
	}

	return NULL;
}

static int tegra_vi_graph_build_one(struct tegra_mc_vi *vi,
				    struct tegra_vi_graph_entity *entity)
{
	u32 link_flags = MEDIA_LNK_FL_ENABLED;
	struct media_entity *local;
	struct media_entity *remote;
	struct media_pad *local_pad;
	struct media_pad *remote_pad;
	struct tegra_vi_graph_entity *ent;
	struct v4l2_of_link link;
	struct device_node *ep = NULL;
	struct device_node *next;
	int ret = 0;

	if (!entity->subdev) {
		dev_err(vi->dev, "%s:No subdev under entity, skip linking\n",
				__func__);
		return 0;
	}

	local = entity->entity;
	dev_dbg(vi->dev, "creating links for entity %s\n", local->name);

	do {
		/* Get the next endpoint and parse its link. */
		next = of_graph_get_next_endpoint(entity->node, ep);
		if (next == NULL)
			break;

		ep = next;

		dev_dbg(vi->dev, "processing endpoint %s\n", ep->full_name);

		ret = v4l2_of_parse_link(ep, &link);
		if (ret < 0) {
			dev_err(vi->dev, "failed to parse link for %s\n",
				ep->full_name);
			continue;
		}

		/* Skip sink ports, they will be processed from the other end of
		 * the link.
		 */
		if (link.local_port >= local->num_pads) {
			dev_err(vi->dev, "invalid port number %u on %s\n",
				link.local_port, link.local_node->full_name);
			v4l2_of_put_link(&link);
			ret = -EINVAL;
			break;
		}

		local_pad = &local->pads[link.local_port];

		if (local_pad->flags & MEDIA_PAD_FL_SINK) {
			dev_dbg(vi->dev, "skipping sink port %s:%u\n",
				link.local_node->full_name, link.local_port);
			v4l2_of_put_link(&link);
			continue;
		}

		/* Skip channel entity , they will be processed separately. */
		if (link.remote_node == vi->dev->of_node) {
			dev_dbg(vi->dev, "skipping channel port %s:%u\n",
				link.local_node->full_name, link.local_port);
			v4l2_of_put_link(&link);
			continue;
		}

		/* Find the remote entity. */
		ent = tegra_vi_graph_find_entity(vi, link.remote_node);
		if (ent == NULL) {
			dev_err(vi->dev, "no entity found for %s\n",
				link.remote_node->full_name);
			v4l2_of_put_link(&link);
			ret = -EINVAL;
			break;
		}

		remote = ent->entity;

		if (link.remote_port >= remote->num_pads) {
			dev_err(vi->dev, "invalid port number %u on %s\n",
				link.remote_port, link.remote_node->full_name);
			v4l2_of_put_link(&link);
			ret = -EINVAL;
			break;
		}

		remote_pad = &remote->pads[link.remote_port];

		v4l2_of_put_link(&link);

		if (media_entity_find_link(local_pad, remote_pad))
			continue;

		/* Create the media link. */
		dev_dbg(vi->dev, "creating %s:%u -> %s:%u link\n",
			local->name, local_pad->index,
			remote->name, remote_pad->index);

		ret = tegra_media_create_link(local, local_pad->index, remote,
				remote_pad->index, link_flags);
		if (ret < 0) {
			dev_err(vi->dev,
				"failed to create %s:%u -> %s:%u link\n",
				local->name, local_pad->index,
				remote->name, remote_pad->index);
			break;
		}
	} while (next);

	return ret;
}

static int tegra_vi_graph_build_links(struct tegra_mc_vi *vi)
{
	u32 link_flags = MEDIA_LNK_FL_ENABLED;
	struct device_node *node = vi->dev->of_node;
	struct media_entity *source;
	struct media_entity *sink;
	struct media_pad *source_pad;
	struct media_pad *sink_pad;
	struct tegra_vi_graph_entity *ent;
	struct v4l2_of_link link;
	struct device_node *ep = NULL;
	struct device_node *next;
	struct tegra_channel *chan;
	int ret = 0;

	dev_dbg(vi->dev, "creating links for channels\n");

	chan = list_first_entry(&vi->vi_chans, struct tegra_channel, list);
	do {
		/* Get the next endpoint and parse its link. */
		next = of_graph_get_next_endpoint(node, ep);
		if (next == NULL || !of_device_is_available(next))
			break;

		/* Device not registered */
		if (!chan->init_done) {
			chan = list_next_entry(chan, list);
			continue;
		}

		ep = next;

		dev_dbg(vi->dev, "processing endpoint %s\n", ep->full_name);

		ret = v4l2_of_parse_link(ep, &link);
		if (ret < 0) {
			dev_err(vi->dev, "failed to parse link for %s\n",
				ep->full_name);
			continue;
		}

		if (link.local_port >= vi->num_channels) {
			dev_err(vi->dev, "wrong channel number for port %u\n",
				link.local_port);
			v4l2_of_put_link(&link);
			ret = -EINVAL;
			break;
		}

		dev_dbg(vi->dev, "creating link for channel %s\n",
			chan->video.name);

		/* Find the remote entity. */
		ent = tegra_vi_graph_find_entity(vi, link.remote_node);
		if (ent == NULL) {
			dev_err(vi->dev, "no entity found for %s\n",
				link.remote_node->full_name);
			v4l2_of_put_link(&link);
			ret = -EINVAL;
			break;
		}

		if (ent->entity == NULL) {
			dev_err(vi->dev, "entity not bounded %s\n",
				link.remote_node->full_name);
			continue;
		}

		source = ent->entity;
		source_pad = &source->pads[link.remote_port];
		sink = &chan->video.entity;
		sink_pad = &chan->pad;

		v4l2_of_put_link(&link);

		if (media_entity_find_link(source_pad, sink_pad))
			goto next_chan;

		/* Create the media link. */
		dev_dbg(vi->dev, "creating %s:%u -> %s:%u link\n",
			source->name, source_pad->index,
			sink->name, sink_pad->index);

		ret = tegra_media_create_link(source, source_pad->index,
				sink, sink_pad->index, link_flags);
		if (ret < 0) {
			dev_err(vi->dev,
				"failed to create %s:%u -> %s:%u link\n",
				source->name, source_pad->index,
				sink->name, sink_pad->index);
			break;
		}

next_chan:
		if (tegra_channel_init_subdevices(chan) < 0) {
			dev_err(vi->dev, "Failed to initialize sub-devices\n");
			chan->init_done = false;
		}

		chan = list_next_entry(chan, list);
	} while (next != NULL);

	return ret;
}

static int tegra_vi_graph_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct tegra_mc_vi *vi =
		container_of(notifier, struct tegra_mc_vi, notifier);
	struct tegra_vi_graph_entity *entity;
	int ret;

	dev_dbg(vi->dev, "notify complete, all subdevs registered\n");

	/* Create links for every entity. */
	list_for_each_entry(entity, &vi->entities, list) {
		if (entity->entity != NULL) {
			ret = tegra_vi_graph_build_one(vi, entity);
			if (ret < 0)
				return ret;
		}
	}

	/* Create links for channels */
	ret = tegra_vi_graph_build_links(vi);
	if (ret < 0)
		return ret;

	ret = v4l2_device_register_subdev_nodes(&vi->v4l2_dev);
	if (ret < 0)
		dev_err(vi->dev, "failed to register subdev nodes\n");

	vi->link_status++;

	return ret;
}

static int tegra_vi_graph_notify_bound(struct v4l2_async_notifier *notifier,
				   struct v4l2_subdev *subdev,
				   struct v4l2_async_subdev *asd)
{
	struct tegra_mc_vi *vi =
		container_of(notifier, struct tegra_mc_vi, notifier);
	struct tegra_vi_graph_entity *entity;

	/* Locate the entity corresponding to the bound subdev and store the
	 * subdev pointer.
	 */
	list_for_each_entry(entity, &vi->entities, list) {
		if (entity->node != subdev->dev->of_node &&
			entity->node != subdev->of_node)
			continue;

		if (entity->subdev) {
			dev_err(vi->dev, "duplicate subdev for node %s\n",
				entity->node->full_name);
			return -EINVAL;
		}

		dev_dbg(vi->dev, "subdev %s bound\n", subdev->name);
		entity->entity = &subdev->entity;
		entity->subdev = subdev;
		vi->subdevs_bound++;
		return 0;
	}

	dev_err(vi->dev, "no entity for subdev %s\n", subdev->name);
	return -EINVAL;
}

void tegra_vi_graph_cleanup(struct tegra_mc_vi *vi)
{
	struct tegra_vi_graph_entity *entityp;
	struct tegra_vi_graph_entity *entity;

	v4l2_async_notifier_unregister(&vi->notifier);

	list_for_each_entry_safe(entity, entityp, &vi->entities, list) {
		of_node_put(entity->node);
		list_del(&entity->list);
	}
}

int tegra_vi_get_port_info(struct tegra_channel *chan,
			struct device_node *node, unsigned int index)
{
	struct device_node *ep = NULL;
	struct device_node *ports;
	struct device_node *port;
	int value = 0xFFFF;
	int ret = 0, i;

	ports = of_get_child_by_name(node, "ports");
	if (ports == NULL)
		ports = node;

	for_each_child_of_node(ports, port) {
		if (!port->name || of_node_cmp(port->name, "port"))
			continue;

		ret = of_property_read_u32(port, "reg", &value);
		if (ret < 0)
			continue;

		if (value != index)
			continue;

		for_each_child_of_node(port, ep) {
			if (!ep->name || of_node_cmp(ep->name, "endpoint"))
				continue;

			/* Get CSI port */
			ret = of_property_read_u32(ep, "csi-port", &value);
			if (ret < 0)
				dev_err(&chan->video.dev, "csi port error\n");
			chan->port[0] = value;

			/* Get number of data lanes for the endpoint */
			ret = of_property_read_u32(ep, "bus-width", &value);
			if (ret < 0)
				dev_err(&chan->video.dev, "num lanes error\n");
			chan->numlanes = value;

			if (value > 12) {
				dev_err(&chan->video.dev, "num lanes >12!\n");
				return -EINVAL;
			}
			/*
			 * for numlanes greater than 4 multiple CSI bricks
			 * are needed to capture the image, the logic below
			 * checks for numlanes > 4 and add a new CSI brick
			 * as a valid port. Loops around the three CSI
			 * bricks to add as many ports necessary.
			 */
			value -= 4;
			for (i = 1; value > 0; i++, value -= 4) {
				int next_port = chan->port[i-1] + 2;

				next_port = (next_port % (PORT_F + 1));
				chan->port[i] = next_port;
			}
		}
	}

	return ret;
}

static int tegra_vi_graph_parse_one(struct tegra_mc_vi *vi,
				struct device_node *node)
{
	struct device_node *ep = NULL;
	struct device_node *next;
	struct device_node *remote = NULL;
	struct tegra_vi_graph_entity *entity;
	int ret = 0;

	dev_dbg(vi->dev, "parsing node %s\n", node->full_name);

	do {
		/* Parse all the remote entities and put them into the list */
		next = of_graph_get_next_endpoint(node, ep);
		if (next == NULL || !of_device_is_available(next))
			break;
		ep = next;

		dev_dbg(vi->dev, "handling endpoint %s\n", ep->full_name);

		remote = of_graph_get_remote_port_parent(ep);
		if (!remote) {
			ret = -EINVAL;
			break;
		}

		/* Skip entities that we have already processed. */
		if (remote == vi->dev->of_node ||
		    tegra_vi_graph_find_entity(vi, remote) ||
		    !of_device_is_available(remote))
			continue;

		entity = devm_kzalloc(vi->dev, sizeof(*entity),
				GFP_KERNEL);
		if (entity == NULL) {
			ret = -ENOMEM;
			break;
		}

		entity->node = remote;
		entity->asd.match_type = V4L2_ASYNC_MATCH_OF;
		entity->asd.match.of.node = remote;
		list_add_tail(&entity->list, &vi->entities);
		vi->num_subdevs++;
	} while (next);

	return ret;
}

int tegra_vi_tpg_graph_init(struct tegra_mc_vi *mc_vi)
{
	int err = 0;
	u32 link_flags = MEDIA_LNK_FL_ENABLED;
	struct tegra_csi_device *csi = mc_vi->csi;
	struct tegra_channel *vi_it;
	struct tegra_csi_channel *csi_it;

	if (!csi) {
		dev_err(mc_vi->dev, "CSI is NULL\n");
		return -EINVAL;
	}
	mc_vi->num_subdevs = mc_vi->num_channels;
	vi_it = mc_vi->tpg_start;
	csi_it = csi->tpg_start;

	list_for_each_entry_from(vi_it, &mc_vi->vi_chans, list) {
		/* Device not registered */
		if (!vi_it->init_done)
			continue;

		list_for_each_entry_from(csi_it, &csi->csi_chans, list) {
			struct media_entity *source = &csi_it->subdev.entity;
			struct media_entity *sink = &vi_it->video.entity;
			struct media_pad *source_pad = csi_it->pads;
			struct media_pad *sink_pad = &vi_it->pad;

			vi_it->bypass = 0;
			err = v4l2_device_register_subdev(&mc_vi->v4l2_dev,
					&csi_it->subdev);
			if (err) {
				dev_err(mc_vi->dev,
					"%s:Fail to register subdev\n",
					__func__);
				goto register_fail;
			}
			dev_dbg(mc_vi->dev, "creating %s:%u -> %s:%u link\n",
				source->name, source_pad->index,
				sink->name, sink_pad->index);

			err = tegra_media_create_link(source, source_pad->index,
					sink, sink_pad->index, link_flags);
			if (err < 0) {
				dev_err(mc_vi->dev,
					"failed to create %s:%u -> %s:%u link\n",
					source->name, source_pad->index,
					sink->name, sink_pad->index);
				goto register_fail;
			}
			err = tegra_channel_init_subdevices(vi_it);
			if (err) {
				dev_err(mc_vi->dev,
					"%s:Init subdevice error\n", __func__);
				goto register_fail;
			}
			csi_it = list_next_entry(csi_it, list);
			break;
		}
	}

	return 0;
register_fail:
	csi_it = csi->tpg_start;
	list_for_each_entry_from(csi_it, &csi->csi_chans, list)
		v4l2_device_unregister_subdev(&csi_it->subdev);
	return err;
}

int tegra_vi_graph_init(struct tegra_mc_vi *vi)
{
	struct tegra_vi_graph_entity *entity;
	struct v4l2_async_subdev **subdevs = NULL;
	unsigned int num_subdevs = 0;
	int ret = 0, i;

	/*
	 * Walk the links to parse the full graph. Start by parsing the
	 * composite node and then parse entities in turn. The list_for_each
	 * loop will handle entities added at the end of the list while walking
	 * the links.
	 */
	ret = tegra_vi_graph_parse_one(vi, vi->dev->of_node);
	if (ret < 0)
		return 0;

	list_for_each_entry(entity, &vi->entities, list) {
		ret = tegra_vi_graph_parse_one(vi, entity->node);
		if (ret < 0)
			break;
	}

	if (!vi->num_subdevs) {
		dev_dbg(vi->dev, "warning: no subdev found in graph\n");
		goto done;
	}

	/* Register the subdevices notifier. */
	num_subdevs = vi->num_subdevs;
	subdevs = devm_kzalloc(vi->dev, sizeof(*subdevs) * num_subdevs,
			       GFP_KERNEL);
	if (subdevs == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	/*
	 * Add code to check for sensors and
	 * set TPG mode for VI if no sensors found
	 * logic varies for different platforms
	 */
	i = 0;
	list_for_each_entry(entity, &vi->entities, list)
		subdevs[i++] = &entity->asd;

	vi->notifier.subdevs = subdevs;
	vi->notifier.num_subdevs = num_subdevs;
	vi->notifier.bound = tegra_vi_graph_notify_bound;
	vi->notifier.complete = tegra_vi_graph_notify_complete;
	vi->link_status = 0;
	vi->subdevs_bound = 0;

	ret = v4l2_async_notifier_register(&vi->v4l2_dev, &vi->notifier);
	if (ret < 0) {
		dev_err(vi->dev, "notifier registration failed\n");
		goto done;
	}

	if (!vi->link_status) {
		if (vi->subdevs_bound) {
			ret = tegra_vi_graph_notify_complete(&vi->notifier);
			if (ret < 0)
				goto done;
		}
	}

	return 0;

done:
	if (ret < 0)
		tegra_vi_graph_cleanup(vi);

	return ret;
}
