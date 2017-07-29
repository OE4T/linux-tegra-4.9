/*
 * crc.c: CRC functions for tegradc EXT device
 *
 * Copyright (c) 2017, NVIDIA CORPORATION, All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/atomic.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>

#include "dc.h"
#include "dc_priv_defs.h"
#include "dc_priv.h"
#include "dc_reg.h"
#include <video/tegra_dc_ext.h>

#define TEGRA_DC_FLIP_BUF_CAPACITY 1024 /* in units of number of elements */
#define TEGRA_DC_CRC_BUF_CAPACITY 1024 /* in units of number of elements */
#define CRC_COMPLETE_TIMEOUT msecs_to_jiffies(1000)

/* Flip values are 16 bit unsigned integer values. To avoid ugly type casts for
 * complicated checks involved in situations when flip IDs are recycled, define
 * a limit on how far into the future can the user query the CRCs
 */
#define TEGRA_DC_CRC_GET_MAX_FLIP_LOOKAHEAD 128

static const char *flip_state_literals[] = {
	"queued", "dequeued", "flipped", "skipped"
};

/* type = 0 => flips, 1=> CRC */
__maybe_unused
static void _tegra_dc_ring_buf_print(struct tegra_dc_ring_buf *buf, u16 i)
{
	struct tegra_dc_flip_buf_ele *flip;
	struct tegra_dc_crc_buf_ele *crc;
	int iter;

	switch (buf->type) {
	case TEGRA_DC_RING_BUF_FLIP:
		flip = (struct tegra_dc_flip_buf_ele *)
			(buf->data + i * sizeof(*flip));
		pr_info("%llu-%s ", flip->id, flip_state_literals[flip->state]);
		break;
	case TEGRA_DC_RING_BUF_CRC:
		crc = (struct tegra_dc_crc_buf_ele *)
			(buf->data + i * sizeof(*crc));

		for (iter = 0; iter < DC_N_WINDOWS; iter++)
			pr_info("%llu-%1d ", crc->matching_flips[iter].id,
				crc->matching_flips[iter].valid);
		pr_info("%4d-%1d ", crc->rg.crc, crc->rg.valid);
		pr_info("%4d-%1d ", crc->comp.crc, crc->comp.valid);
		pr_info("%4d-%1d ", crc->sor.crc, crc->sor.valid);

		for (iter = 0; iter < TEGRA_DC_MAX_CRC_REGIONS; iter++)
			pr_info("%4d-%1d ", crc->regional[iter].crc,
					    crc->regional[iter].valid);
		break;
	}
}

__maybe_unused
static void tegra_dc_ring_buf_print(struct tegra_dc_ring_buf *buf)
{
	u16 i;

	pr_info("==================================================\n");
	pr_info("head=%d, tail=%d, size=%d\n", buf->head, buf->tail, buf->size);

	if (buf->size) {
		if (buf->head > buf->tail) {
			for (i = buf->tail; i < buf->head; i++)
				_tegra_dc_ring_buf_print(buf, i);
		} else { /* Head has rolled over */
			for (i = buf->tail; i < buf->capacity; i++)
				_tegra_dc_ring_buf_print(buf, i);
			for (i = 0; i < buf->head; i++)
				_tegra_dc_ring_buf_print(buf, i);
		}
	}
	pr_info("==================================================\n");
}

static inline size_t _get_bytes_per_ele(struct tegra_dc_ring_buf *buf)
{
	switch (buf->type) {
	case TEGRA_DC_RING_BUF_FLIP:
		return sizeof(struct tegra_dc_flip_buf_ele);
	case TEGRA_DC_RING_BUF_CRC:
		return sizeof(struct tegra_dc_crc_buf_ele);
	default:
		return 0;
	}
}

/* Retrieve an in buffer pointer for element at index @idx
 * @in_buf_ptr is the output parameter to be filled by the API
 */
static int tegra_dc_ring_buf_peek(struct tegra_dc_ring_buf *buf, u16 idx,
				  char **in_buf_ptr)
{
	size_t bytes = _get_bytes_per_ele(buf);

	if (buf->size == 0 || idx >= buf->capacity)
		return -EINVAL;

	*in_buf_ptr = (buf->data + idx * bytes);

	return 0;
}

/* Remove the least recently buffered element */
static int tegra_dc_ring_buf_remove(struct tegra_dc_ring_buf *buf)
{
	if (buf->size == 0)
		return -EINVAL;

	buf->tail = (buf->tail + 1) % buf->capacity;
	buf->size--;

	return 0;
}

/* Add a buffer element at the head of the buffer
 * @src is the memory pointer from where data for the buffer element is copied
 * if @in_buf_ptr is not NULL, the caller receives in buffer pointer to the
 * element
 */
void tegra_dc_ring_buf_add(struct tegra_dc_ring_buf *buf, void *src,
			  char **in_buf_ptr)
{
	size_t bytes = _get_bytes_per_ele(buf);
	void *dst = buf->data + buf->head * bytes;

	/* If the buffer is full, drop the least recently used item */
	if (buf->size == buf->capacity)
		tegra_dc_ring_buf_remove(buf);

	memcpy(dst, src, bytes);
	if (in_buf_ptr)
		*in_buf_ptr = dst;

	buf->head = (buf->head + 1) % buf->capacity;
	buf->size++;
}

/* Called when enabling the DC head.
 * Avoid calling it when disabling the DC head so as to avoid any issues caused
 * by an impending or a missing flush of values to the registers
 */
void tegra_dc_crc_reset(struct tegra_dc *dc)
{
	if (tegra_dc_is_t21x())
		tegra_dc_writel(dc, 0x00, DC_COM_CRC_CONTROL);
	else if (tegra_dc_is_nvdisplay())
		tegra_nvdisp_crc_reset(dc);
}

/* Called when disabling the DC head
 * This function resets only the SW state associated with the CRC mechanism
 */
void tegra_dc_crc_deinit(struct tegra_dc *dc)
{
	if (!dc->crc_initialized)
		return;

	kfree(dc->flip_buf.data);
	kfree(dc->crc_buf.data);

	dc->flip_buf.size = 0;
	dc->flip_buf.head = 0;
	dc->flip_buf.tail = 0;
	dc->crc_buf.size = 0;
	dc->crc_buf.head = 0;
	dc->crc_buf.tail = 0;

	atomic_set(&dc->crc_ref_cnt.global, 0);
	atomic_set(&dc->crc_ref_cnt.rg_comp_sor, 0);
	atomic_set(&dc->crc_ref_cnt.regional, 0);
	dc->crc_ref_cnt.legacy = false;

	mutex_destroy(&dc->flip_buf.lock);
	mutex_destroy(&dc->crc_buf.lock);

	dc->crc_initialized = false;
}

static long tegra_dc_crc_init(struct tegra_dc *dc)
{
	dc->flip_buf.capacity = TEGRA_DC_FLIP_BUF_CAPACITY;
	dc->crc_buf.capacity = TEGRA_DC_CRC_BUF_CAPACITY;

	dc->flip_buf.type = TEGRA_DC_RING_BUF_FLIP;
	dc->crc_buf.type = TEGRA_DC_RING_BUF_CRC;

	dc->flip_buf.data = kcalloc(TEGRA_DC_FLIP_BUF_CAPACITY,
				    sizeof(struct tegra_dc_flip_buf_ele),
				    GFP_KERNEL);
	if (!dc->flip_buf.data)
		return -ENOMEM;

	dc->crc_buf.data = kcalloc(TEGRA_DC_CRC_BUF_CAPACITY,
				   sizeof(struct tegra_dc_crc_buf_ele),
				   GFP_KERNEL);
	if (!dc->crc_buf.data)
		return -ENOMEM;

	mutex_init(&dc->flip_buf.lock);
	mutex_init(&dc->crc_buf.lock);

	dc->crc_initialized = true;

	return 0;
}

static int tegra_dc_crc_t21x_rg_en_dis(struct tegra_dc *dc,
				       struct tegra_dc_ext_crc_conf *conf,
				       bool enable)
{
	int ret = 0x00;
	u32 reg = 0x00;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	if (enable) {
		reg = CRC_ALWAYS_ENABLE | CRC_ENABLE_ENABLE;
		if (conf->input_data == TEGRA_DC_EXT_CRC_INPUT_DATA_ACTIVE_DATA)
			reg |= CRC_INPUT_DATA_ACTIVE_DATA;

		atomic_inc(&dc->crc_ref_cnt.rg_comp_sor);

		tegra_dc_writel(dc, reg, DC_COM_CRC_CONTROL);
		tegra_dc_activate_general_channel(dc);
	} else {
		if (atomic_dec_return(&dc->crc_ref_cnt.rg_comp_sor) == 0) {
			tegra_dc_writel(dc, 0x00, DC_COM_CRC_CONTROL);
			tegra_dc_activate_general_channel(dc);
		}
	}

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	if (enable) {
		if (atomic_inc_return(&dc->crc_ref_cnt.global) == 1) {
			ret = tegra_dc_config_frame_end_intr(dc, true);
			if (ret)
				atomic_dec(&dc->crc_ref_cnt.rg_comp_sor);
		}
	} else {
		if (atomic_dec_return(&dc->crc_ref_cnt.global) == 0) {
			ret = tegra_dc_config_frame_end_intr(dc, false);
			if (ret)
				atomic_inc(&dc->crc_ref_cnt.rg_comp_sor);
		}
	}

	return ret;
}

static int tegra_dc_crc_t21x_en_dis(struct tegra_dc *dc,
				    struct tegra_dc_ext_crc_arg *arg, bool en)
{
	int ret = 0;
	struct tegra_dc_ext_crc_conf *conf =
				(struct tegra_dc_ext_crc_conf *)arg->conf;
	u8 iter;

	for (iter = 0; iter < arg->num_conf; iter++) {
		switch (conf[iter].type) {
		case TEGRA_DC_EXT_CRC_TYPE_RG:
			ret = tegra_dc_crc_t21x_rg_en_dis(dc, &conf[iter], en);
			if (ret)
				return ret;
			break;
		case TEGRA_DC_EXT_CRC_TYPE_OR:
			return -ENOTSUPP; /*TODO: Implement this */
		default:
			return -ENOTSUPP;
		}
	}

	return ret;
}

static int tegra_dc_crc_nvdisp_en_dis(struct tegra_dc *dc,
				      struct tegra_dc_ext_crc_arg *arg, bool en)
{
	int ret;
	struct tegra_dc_ext_crc_conf *conf =
				(struct tegra_dc_ext_crc_conf *)arg->conf;
	u8 iter;

	for (iter = 0; iter < arg->num_conf; iter++) {
		if (en)
			ret = tegra_nvdisp_crc_enable(dc, conf + iter);
		else
			ret = tegra_nvdisp_crc_disable(dc, conf + iter);
		if (ret)
			return ret;
	}

	tegra_dc_activate_general_channel(dc);

	return 0;
}

long tegra_dc_crc_enable(struct tegra_dc *dc, struct tegra_dc_ext_crc_arg *arg)
{
	int ret;

	if (!dc->enabled)
		return -ENODEV;

	if (dc->crc_ref_cnt.legacy)
		return -EBUSY;

	if (!dc->crc_initialized) {
		ret = tegra_dc_crc_init(dc);
		if (ret)
			return ret;
	}

	if (tegra_dc_is_t21x())
		return tegra_dc_crc_t21x_en_dis(dc, arg, true);
	else if (tegra_dc_is_nvdisplay())
		return tegra_dc_crc_nvdisp_en_dis(dc, arg, true);
	else
		return -ENOTSUPP;
}

long tegra_dc_crc_disable(struct tegra_dc *dc,
			  struct tegra_dc_ext_crc_arg *arg)
{
	if (!dc->enabled)
		return -ENODEV;

	if (!dc->crc_initialized)
		return -EPERM;

	if (dc->crc_ref_cnt.legacy)
		return -EBUSY;

	if (tegra_dc_is_t21x())
		return tegra_dc_crc_t21x_en_dis(dc, arg, false);
	else if (tegra_dc_is_nvdisplay())
		return tegra_dc_crc_nvdisp_en_dis(dc, arg, false);
	else
		return -ENOTSUPP;
}

/* Get the Least Recently Matched (lrm) flip ID.
 * Our best estimate is to find this flip ID at the tail of the buffer.
 * The caller has the option to retrieve the least recently matched flip ID or
 * the corresponding CRC buffer element
 */
static int _find_lrm(struct tegra_dc *dc, u64 *flip_id,
		     struct tegra_dc_crc_buf_ele *crc_ele_out)
{
	u64 lrm = 0x0; /* Default value when no flips are matched */
	int ret = 0;
	struct tegra_dc_crc_buf_ele *crc_ele = NULL;

	ret = tegra_dc_ring_buf_peek(&dc->crc_buf, dc->crc_buf.tail,
				     (char **)&crc_ele);
	if (ret)
		goto done;

	if (crc_ele->matching_flips[0].valid)
		lrm = crc_ele->matching_flips[0].id;

	if (crc_ele_out)
		memcpy(crc_ele_out, crc_ele, sizeof(*crc_ele));

done:
	if (flip_id)
		*flip_id = lrm;

	return ret;
}

static inline u16 prev_idx(struct tegra_dc_ring_buf *buf, u16 idx)
{
	return idx ? idx - 1 : buf->capacity - 1;
}

/* Get the Most Recently Matched (mrm) flip ID.
 * Our best estimate is to find this flip ID at the head of the buffer.
 * The caller has the option to retrieve the most recently matched flip ID or
 * the corresponding CRC buffer element
 */
static int _find_mrm(struct tegra_dc *dc, u64 *flip_id,
		     struct tegra_dc_crc_buf_ele *crc_ele_out)
{
	/* Default value when no flips are matched */
	u64 mrm = U64_MAX;
	u16 peek_idx;
	int ret = 0, iter;
	struct tegra_dc_crc_buf_ele *crc_ele = NULL;

	peek_idx = prev_idx(&dc->crc_buf, dc->crc_buf.head);

	ret = tegra_dc_ring_buf_peek(&dc->crc_buf, peek_idx, (char **)&crc_ele);
	if (ret)
		goto done;

	for (iter = 0;
	     iter < DC_N_WINDOWS && crc_ele->matching_flips[iter].valid;
	     iter++)
		mrm = crc_ele->matching_flips[iter].id;

	if (crc_ele_out)
		memcpy(crc_ele_out, crc_ele, sizeof(*crc_ele));

done:
	if (flip_id)
		*flip_id = mrm;

	return ret;
}

static bool _is_flip_out_of_bounds(struct tegra_dc *dc, u64 flip_id)
{
	u64 lrm, mrm; /* Least and most recently matched flips */

	_find_lrm(dc, &lrm, NULL);
	_find_mrm(dc, &mrm, NULL);

	if (mrm < lrm) {
		dev_err(&dc->ndev->dev, "flip IDs have overflowed "
					"lrm = %llu, mrm = %llu\n", lrm, mrm);
		return true;
	}

	if (flip_id < lrm ||
	    flip_id > mrm + TEGRA_DC_CRC_GET_MAX_FLIP_LOOKAHEAD) {
		dev_err(&dc->ndev->dev, "flip_id %llu out of bounds\n",
				flip_id);
		return true;
	}

	return false;
}

/* Scan the CRC buffer from @start_idx (inclusive) to @end_idx (exclusive)
 * to find the element matched with @flip_id
 */
static int _scan_crc_buf(struct tegra_dc *dc, u16 start_idx, u16 end_idx,
			 u64 flip_id, struct tegra_dc_crc_buf_ele *crc_ele)
{
	u16 peek_idx = start_idx;
	struct tegra_dc_ring_buf *buf = &dc->crc_buf;
	struct tegra_dc_crc_buf_ele *crc_iter = NULL;
	int iter;

	while (peek_idx != end_idx) {
		tegra_dc_ring_buf_peek(buf, peek_idx, (char **)&crc_iter);

		for (iter = 0; iter < DC_N_WINDOWS; iter++) {
			if (!crc_iter->matching_flips[iter].valid)
				break;

			if (crc_iter->matching_flips[iter].id == flip_id) {
				memcpy(crc_ele, crc_iter, sizeof(*crc_iter));
				return 0;
			}
		}

		peek_idx = prev_idx(buf, peek_idx);
	}

	return -EAGAIN;
}

static int _find_crc_in_buf(struct tegra_dc *dc, u64 flip_id,
			    struct tegra_dc_crc_buf_ele *crc_ele)
{
	int ret = -EAGAIN;
	u16 start_idx, end_idx;
	struct tegra_dc_ring_buf *buf = &dc->crc_buf;

	mutex_lock(&buf->lock);

	if (flip_id == U64_MAX) {
		ret = _find_mrm(dc, NULL, crc_ele);
		goto done;
	}

	if (_is_flip_out_of_bounds(dc, flip_id)) {
		ret = -ENODATA;
		goto done;
	}

	/* At this point, we are committed to return a CRC value to the user,
	 * even if one is yet to be generated in the imminent future
	 */
	end_idx = prev_idx(buf, buf->tail);
	start_idx = prev_idx(buf, buf->head);

	while (ret == -EAGAIN) {
		ret = _scan_crc_buf(dc, start_idx, end_idx, flip_id, crc_ele);
		if (!ret || ret != -EAGAIN)
			goto done;

		/* Control reaching here implies the flip being requested is yet
		 * to be matched at a certain frame end interrupt, hence wait on
		 * the event
		 */
		mutex_unlock(&buf->lock);
		reinit_completion(&dc->crc_complete);
		if (!wait_for_completion_timeout(&dc->crc_complete,
						 CRC_COMPLETE_TIMEOUT)) {
			dev_err(&dc->ndev->dev, "CRC read timed out\n");
			ret = -ETIME;
			goto done;
		}
		mutex_lock(&buf->lock);

		end_idx = prev_idx(buf, start_idx);
		start_idx = prev_idx(buf, buf->head);
	}

done:
	mutex_unlock(&buf->lock);
	return ret;
}

static int tegra_dc_crc_t21x_collect(struct tegra_dc *dc,
				     struct tegra_dc_crc_buf_ele *ele)
{
	bool valids = false; /* Logical OR of valid fields of individual CRCs */

	if (!atomic_read(&dc->crc_ref_cnt.rg_comp_sor))
		return 0;

	ele->rg.crc = tegra_dc_readl(dc, DC_COM_CRC_CHECKSUM_LATCHED);
	ele->rg.valid = 1;

	valids = ele->rg.valid || ele->sor.valid;

	return valids ? 0 : -EINVAL;
}

long tegra_dc_crc_get(struct tegra_dc *dc, struct tegra_dc_ext_crc_arg *arg)
{
	int ret = 0;
	struct tegra_dc_ext_crc_conf *conf =
				(struct tegra_dc_ext_crc_conf *)arg->conf;
	struct tegra_dc_crc_buf_ele crc_ele;
	u8 id, iter;

	if (!dc->enabled)
		return -ENODEV;

	if (!dc->crc_initialized)
		return -EPERM;

	if (dc->crc_ref_cnt.legacy)
		return -EBUSY;

	ret = _find_crc_in_buf(dc, arg->flip_id, &crc_ele);
	if (ret)
		return ret;

	for (iter = 0; iter < arg->num_conf; iter++) {
		switch (conf[iter].type) {
		case TEGRA_DC_EXT_CRC_TYPE_RG:
			conf[iter].crc.valid = crc_ele.rg.valid;
			conf[iter].crc.val = crc_ele.rg.crc;
			break;
		case TEGRA_DC_EXT_CRC_TYPE_COMP:
			if (tegra_dc_is_nvdisplay()) {
				conf[iter].crc.valid = crc_ele.comp.valid;
				conf[iter].crc.val = crc_ele.comp.crc;
			} else {
				conf[iter].crc.valid = 0;
				conf[iter].crc.val = 0;
				ret = -ENOTSUPP;
			}
			break;
		case TEGRA_DC_EXT_CRC_TYPE_RG_REGIONAL:
			if (tegra_dc_is_nvdisplay()) {
				id = conf[iter].region.id;
				conf[iter].crc.valid =
						crc_ele.regional[id].valid;
				conf[iter].crc.val = crc_ele.regional[id].crc;
			} else {
				conf[iter].crc.valid = 0;
				conf[iter].crc.val = 0;
				ret = -ENOTSUPP;
			}
			break;
		case TEGRA_DC_EXT_CRC_TYPE_OR:
			conf[iter].crc.valid = 0;
			conf[iter].crc.val = 0;
			return -ENOTSUPP; /* TODO: Implement this */
			/* break; */
		default:
			ret = -EINVAL;
		}
	}

	return ret;
}

int tegra_dc_crc_process(struct tegra_dc *dc)
{
	int ret = 0, matched = 0;
	struct tegra_dc_crc_buf_ele crc_ele;
	struct tegra_dc_flip_buf_ele *flip_ele;

	memset(&crc_ele, 0, sizeof(crc_ele));

	/* Collect all CRCs generated by the HW */
	if (tegra_dc_is_t21x())
		ret = tegra_dc_crc_t21x_collect(dc, &crc_ele);
	else if (tegra_dc_is_nvdisplay())
		ret = tegra_nvdisp_crc_collect(dc, &crc_ele);

	if (ret)
		return ret;

	mutex_lock(&dc->flip_buf.lock);

	/* Before doing any work, check if there are flips to match */
	if (!dc->flip_buf.size) {
		mutex_unlock(&dc->flip_buf.lock);
		return 0;
	}

	/* Mark all least recently buffered flips in FLIPPED state as
	 * matching_flips
	 */
	ret = tegra_dc_ring_buf_peek(&dc->flip_buf, dc->flip_buf.tail,
				     (char **)&flip_ele);

	while (!ret && flip_ele->state == TEGRA_DC_FLIP_STATE_FLIPPED) {
		crc_ele.matching_flips[matched].id = flip_ele->id;
		crc_ele.matching_flips[matched].valid = true;
		matched++;
		WARN_ON(matched >= DC_N_WINDOWS);

		tegra_dc_ring_buf_remove(&dc->flip_buf);

		ret = tegra_dc_ring_buf_peek(&dc->flip_buf, dc->flip_buf.tail,
					     (char **)&flip_ele);
	}

	/* Enqueue CRC element in the CRC ring buffer */
	if (matched) {
		mutex_lock(&dc->crc_buf.lock);
		tegra_dc_ring_buf_add(&dc->crc_buf, &crc_ele, NULL);
		mutex_unlock(&dc->crc_buf.lock);
	}

	mutex_unlock(&dc->flip_buf.lock);
	return ret;
}

