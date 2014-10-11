/*
 * drivers/video/tegra/dc/hdmihdcp.c
 *
 * Copyright (c) 2014, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <asm/atomic.h>

#include <mach/dc.h>
#include <mach/kfuse.h>

#include <video/nvhdcp.h>

#include "dc_reg.h"
#include "dc_priv.h"
#include "edid.h"
#include "hdmi2.0.h"
#include "sor.h"
#include "sor_regs.h"
#include "hdmihdcp.h"
#include "hdmi_reg.h"
#include "host1x/host1x01_hardware.h"
#include "tsec/tsec.h"
#include "class_ids.h"
#include "tsec_drv.h"
#include "tsec/tsec_methods.h"
#include "nvhdcp_hdcp22_methods.h"

DECLARE_WAIT_QUEUE_HEAD(wq_worker);

/* for 0x40 Bcaps */
#define BCAPS_REPEATER (1 << 6)
#define BCAPS_READY (1 << 5)
#define BCAPS_11 (1 << 1) /* used for both Bcaps and Ainfo */

/* for 0x41 Bstatus */
#define BSTATUS_MAX_DEVS_EXCEEDED	(1 << 7)
#define BSTATUS_MAX_CASCADE_EXCEEDED	(1 << 11)

/* for HDCP2.2 */
#define HDCP_READY			1
#define HDCP_REAUTH			2
#define HDCP_REAUTH_MASK		(1 << 11)
#define HDCP22_PROTOCOL			1
#define HDCP1X_PROTOCOL			0
#define HDCP_DEBUG                      0

#ifdef VERBOSE_DEBUG
#define nvhdcp_vdbg(...)	\
		pr_debug("nvhdcp: " __VA_ARGS__)
#else
#define nvhdcp_vdbg(...)		\
({						\
	if (0)					\
		pr_debug("nvhdcp: " __VA_ARGS__); \
	0;					\
})
#endif
#define nvhdcp_debug(...)	\
		pr_debug("nvhdcp: " __VA_ARGS__)
#define nvhdcp_err(...)	\
		pr_err("nvhdcp: Error: " __VA_ARGS__)
#define nvhdcp_info(...)	\
		pr_info("nvhdcp: " __VA_ARGS__)

struct tegra_dc *tegra_dc_hdmi_get_dc(struct tegra_hdmi *hdmi)
{
	return hdmi ? hdmi->dc : NULL;
}

static inline u32 nvhdcp_sor_readl(struct tegra_hdmi *hdmi, u32 reg)
{
	return readl(hdmi->sor->base + reg * 4);
}

static inline void nvhdcp_sor_writel(struct tegra_hdmi *hdmi,
	u32 val, u32 reg)
{
	writel(val, hdmi->sor->base + reg * 4);
}

static inline bool nvhdcp_is_plugged(struct tegra_nvhdcp *nvhdcp)
{
	rmb();
	return nvhdcp->plugged;
}

static inline bool nvhdcp_set_plugged(struct tegra_nvhdcp *nvhdcp, bool plugged)
{
	nvhdcp->plugged = plugged;
	wmb();
	return plugged;
}

static int nvhdcp_i2c_read(struct tegra_nvhdcp *nvhdcp, u8 reg,
					size_t len, void *data)
{
	int status;
	int retries = 15;
	struct i2c_msg msg[] = {
		{
			.addr = 0x74 >> 1, /* primary link */
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = 0x74 >> 1, /* primary link */
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};
	struct tegra_dc *dc = tegra_dc_hdmi_get_dc(nvhdcp->hdmi);

	tegra_dc_ddc_enable(dc, true);
	do {
		if (!nvhdcp_is_plugged(nvhdcp)) {
			nvhdcp_err("disconnect during i2c xfer\n");
			tegra_dc_ddc_enable(dc, false);
			return -EIO;
		}
		status = i2c_transfer(nvhdcp->client->adapter,
			msg, ARRAY_SIZE(msg));
		if ((status < 0) && (retries > 1))
			msleep(250);
	} while ((status < 0) && retries--);
	tegra_dc_ddc_enable(dc, false);

	if (status < 0) {
		nvhdcp_err("i2c xfer error %d\n", status);
		return status;
	}

	return 0;
}

static int nvhdcp_i2c_write(struct tegra_nvhdcp *nvhdcp, u8 reg,
					size_t len, const void *data)
{
	int status;
	u8 buf[len + 1];
	struct i2c_msg msg[] = {
		{
			.addr = 0x74 >> 1, /* primary link */
			.flags = 0,
			.len = len + 1,
			.buf = buf,
		},
	};
	int retries = 15;
	struct tegra_dc *dc = tegra_dc_hdmi_get_dc(nvhdcp->hdmi);

	buf[0] = reg;
	memcpy(buf + 1, data, len);

	tegra_dc_ddc_enable(dc, true);
	do {
		if (!nvhdcp_is_plugged(nvhdcp)) {
			nvhdcp_err("disconnect during i2c xfer\n");
			tegra_dc_ddc_enable(dc, false);
			return -EIO;
		}
		status = i2c_transfer(nvhdcp->client->adapter,
			msg, ARRAY_SIZE(msg));
		if ((status < 0) && (retries > 1))
			msleep(250);
	} while ((status < 0) && retries--);
	tegra_dc_ddc_enable(dc, false);

	if (status < 0) {
		nvhdcp_err("i2c xfer error %d\n", status);
		return status;
	}

	return 0;
}

static inline int nvhdcp_i2c_read8(struct tegra_nvhdcp *nvhdcp, u8 reg, u8 *val)
{
	return nvhdcp_i2c_read(nvhdcp, reg, 1, val);
}

static inline int nvhdcp_i2c_write8(struct tegra_nvhdcp *nvhdcp, u8 reg, u8 val)
{
	return nvhdcp_i2c_write(nvhdcp, reg, 1, &val);
}

static inline int nvhdcp_i2c_read16(struct tegra_nvhdcp *nvhdcp,
					u8 reg, u16 *val)
{
	u8 buf[2];
	int e;

	e = nvhdcp_i2c_read(nvhdcp, reg, sizeof(buf), buf);
	if (e)
		return e;

	if (val)
		*val = buf[0] | (u16)buf[1] << 8;

	return 0;
}

static int nvhdcp_i2c_read40(struct tegra_nvhdcp *nvhdcp, u8 reg, u64 *val)
{
	u8 buf[5];
	int e, i;
	u64 n;

	e = nvhdcp_i2c_read(nvhdcp, reg, sizeof(buf), buf);
	if (e)
		return e;

	for (i = 0, n = 0; i < 5; i++) {
		n <<= 8;
		n |= buf[4 - i];
	}

	if (val)
		*val = n;

	return 0;
}

static int nvhdcp_i2c_write40(struct tegra_nvhdcp *nvhdcp, u8 reg, u64 val)
{
	char buf[5];
	int i;
	for (i = 0; i < 5; i++) {
		buf[i] = val;
		val >>= 8;
	}
	return nvhdcp_i2c_write(nvhdcp, reg, sizeof(buf), buf);
}

static int nvhdcp_i2c_write64(struct tegra_nvhdcp *nvhdcp, u8 reg, u64 val)
{
	char buf[8];
	int i;
	for (i = 0; i < 8; i++) {
		buf[i] = val;
		val >>= 8;
	}
	return nvhdcp_i2c_write(nvhdcp, reg, sizeof(buf), buf);
}

/* 64-bit link encryption session random number */
static inline u64 get_an(struct tegra_hdmi *hdmi)
{
	u64 r;
	r = (u64)nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_AN_MSB) << 32;
	r |= nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_AN_LSB);
	return r;
}

/* 64-bit upstream exchange random number */
static inline void set_cn(struct tegra_hdmi *hdmi, u64 c_n)
{
	nvhdcp_sor_writel(hdmi, (u32)c_n, NV_SOR_TMDS_HDCP_CN_LSB);
	nvhdcp_sor_writel(hdmi, c_n >> 32, NV_SOR_TMDS_HDCP_CN_MSB);
}


/* 40-bit transmitter's key selection vector */
static inline u64 get_aksv(struct tegra_hdmi *hdmi)
{
	u64 r;
	r = (u64)nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_AKSV_MSB) << 32;
	r |= nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_AKSV_LSB);
	return r;
}

/* 40-bit receiver's key selection vector */
static inline void set_bksv(struct tegra_hdmi *hdmi, u64 b_ksv, bool repeater)
{
	if (repeater)
		b_ksv |= (u64)REPEATER << 32;
	nvhdcp_sor_writel(hdmi, (u32)b_ksv, NV_SOR_TMDS_HDCP_BKSV_LSB);
	nvhdcp_sor_writel(hdmi, b_ksv >> 32, NV_SOR_TMDS_HDCP_BKSV_MSB);
}


/* 40-bit software's key selection vector */
static inline void set_cksv(struct tegra_hdmi *hdmi, u64 c_ksv)
{
	nvhdcp_sor_writel(hdmi, (u32)c_ksv, NV_SOR_TMDS_HDCP_CKSV_LSB);
	nvhdcp_sor_writel(hdmi, c_ksv >> 32, NV_SOR_TMDS_HDCP_CKSV_MSB);
}

/* 40-bit connection state */
static inline u64 get_cs(struct tegra_hdmi *hdmi)
{
	u64 r;
	r = (u64)nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_CS_MSB) << 32;
	r |= nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_CS_LSB);
	return r;
}

/* 40-bit upstream key selection vector */
static inline u64 get_dksv(struct tegra_hdmi *hdmi)
{
	u64 r;
	r = (u64)nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_DKSV_MSB) << 32;
	r |= nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_DKSV_LSB);
	return r;
}

/* 64-bit encrypted M0 value */
static inline u64 get_mprime(struct tegra_hdmi *hdmi)
{
	u64 r;
	r = (u64)nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_MPRIME_MSB) << 32;
	r |= nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_MPRIME_LSB);
	return r;
}

static inline u16 get_transmitter_ri(struct tegra_hdmi *hdmi)
{
	return nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_RI);
}

static inline int get_receiver_ri(struct tegra_nvhdcp *nvhdcp, u16 *r)
{
	return nvhdcp_i2c_read16(nvhdcp, 0x8, r); /* long read */
}

static int get_bcaps(struct tegra_nvhdcp *nvhdcp, u8 *b_caps)
{
	return nvhdcp_i2c_read8(nvhdcp, 0x40, b_caps);
}

static int get_ksvfifo(struct tegra_nvhdcp *nvhdcp,
					unsigned num_bksv_list, u64 *ksv_list)
{
	u8 *buf, *p;
	int e;
	unsigned i;
	size_t buf_len = num_bksv_list * 5;

	if (!ksv_list || num_bksv_list > TEGRA_NVHDCP_MAX_DEVS)
		return -EINVAL;

	if (num_bksv_list == 0)
		return 0;

	buf = kmalloc(buf_len, GFP_KERNEL);
	if (IS_ERR_OR_NULL(buf))
		return -ENOMEM;

	e = nvhdcp_i2c_read(nvhdcp, 0x43, buf_len, buf);
	if (e) {
		kfree(buf);
		return e;
	}

	/* load 40-bit keys from repeater into array of u64 */
	p = buf;
	for (i = 0; i < num_bksv_list; i++) {
		ksv_list[i] = p[0] | ((u64)p[1] << 8) | ((u64)p[2] << 16)
				| ((u64)p[3] << 24) | ((u64)p[4] << 32);
		p += 5;
	}

	kfree(buf);
	return 0;
}

/* get V' 160-bit SHA-1 hash from repeater */
static int get_vprime(struct tegra_nvhdcp *nvhdcp, u8 *v_prime)
{
	int e, i;

	for (i = 0; i < 20; i += 4) {
		e = nvhdcp_i2c_read(nvhdcp, 0x20 + i, 4, v_prime + i);
		if (e)
			return e;
	}
	return 0;
}


/* set or clear RUN_YES */
static void hdcp_ctrl_run(struct tegra_hdmi *hdmi, bool v)
{
	u32 ctrl;

	if (v) {
		ctrl = nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_CTRL);
		ctrl |= HDCP_RUN_YES;
	} else {
		ctrl = 0;
	}

	nvhdcp_sor_writel(hdmi, ctrl, NV_SOR_TMDS_HDCP_CTRL);
}

/* wait for any bits in mask to be set in NV_SOR_TMDS_HDCP_CTRL
 * sleeps up to 120mS */
static int wait_hdcp_ctrl(struct tegra_hdmi *hdmi, u32 mask, u32 *v)
{
	int retries = 13;
	u32 ctrl;

	do {
		ctrl = nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_CTRL);
		if ((ctrl & mask)) {
			if (v)
				*v = ctrl;
			break;
		}
		if (retries > 1)
			usleep_range(10, 15);
	} while (--retries);
	if (!retries) {
		nvhdcp_err("ctrl read timeout (mask=0x%x)\n", mask);
		return -EIO;
	}
	return 0;
}

/* wait for bits in mask to be set to value in NV_SOR_KEY_CTRL
 * waits up to 100mS */
static int wait_key_ctrl(struct tegra_hdmi *hdmi, u32 mask, u32 value)
{
	int retries = 101;
	u32 ctrl;

	do {
		usleep_range(1, 2);
		ctrl = nvhdcp_sor_readl(hdmi, NV_SOR_KEY_CTRL);
		if (((ctrl ^ value) & mask) == 0)
			break;
	} while (--retries);
	if (!retries) {
		nvhdcp_err("key ctrl read timeout (mask=0x%x)\n", mask);
		return -EIO;
	}
	return 0;
}

/* check that key selection vector is well formed.
 * NOTE: this function assumes KSV has already been checked against
 * revocation list.
 */
static int verify_ksv(u64 k)
{
	unsigned i;

	/* count set bits, must be exactly 20 set to be valid */
	for (i = 0; k; i++)
		k ^= k & -k;

	return  (i != 20) ? -EINVAL : 0;
}

static int get_nvhdcp_state(struct tegra_nvhdcp *nvhdcp,
			struct tegra_nvhdcp_packet *pkt)
{
	int	i;

	mutex_lock(&nvhdcp->lock);
	if (nvhdcp->state != STATE_LINK_VERIFY) {
		memset(pkt, 0, sizeof(*pkt));
		pkt->packet_results = TEGRA_NVHDCP_RESULT_LINK_FAILED;
	} else {
		pkt->num_bksv_list = nvhdcp->num_bksv_list;
		for (i = 0; i < pkt->num_bksv_list; i++)
			pkt->bksv_list[i] = nvhdcp->bksv_list[i];
		pkt->b_status = nvhdcp->b_status;
		pkt->b_ksv = nvhdcp->b_ksv;
		memcpy(pkt->v_prime, nvhdcp->v_prime, sizeof(nvhdcp->v_prime));
		pkt->packet_results = TEGRA_NVHDCP_RESULT_SUCCESS;
		pkt->hdcp22 = nvhdcp->hdcp22;
	}
	mutex_unlock(&nvhdcp->lock);
	return 0;
}

/* get Status and Kprime signature - READ_S on TMDS0_LINK0 only */
static int get_s_prime(struct tegra_nvhdcp *nvhdcp,
			struct tegra_nvhdcp_packet *pkt)
{
	struct tegra_hdmi *hdmi = nvhdcp->hdmi;
	u32 sp_msb, sp_lsb1, sp_lsb2;
	int e;

	/* if connection isn't authenticated ... */
	mutex_lock(&nvhdcp->lock);
	if (nvhdcp->state != STATE_LINK_VERIFY) {
		memset(pkt, 0, sizeof(*pkt));
		pkt->packet_results = TEGRA_NVHDCP_RESULT_LINK_FAILED;
		e = 0;
		goto err;
	}

	pkt->packet_results = TEGRA_NVHDCP_RESULT_UNSUCCESSFUL;

	/* we will be taking c_n, c_ksv as input */
	if (!(pkt->value_flags & TEGRA_NVHDCP_FLAG_CN)
			|| !(pkt->value_flags & TEGRA_NVHDCP_FLAG_CKSV)) {
		nvhdcp_err("missing value_flags (0x%x)\n", pkt->value_flags);
		e = -EINVAL;
		goto err;
	}

	pkt->value_flags = 0;

	pkt->a_ksv = nvhdcp->a_ksv;
	pkt->a_n = nvhdcp->a_n;
	pkt->value_flags = TEGRA_NVHDCP_FLAG_AKSV | TEGRA_NVHDCP_FLAG_AN;

	nvhdcp_vdbg("%s():cn %llx cksv %llx\n", __func__, pkt->c_n, pkt->c_ksv);

	set_cn(hdmi, pkt->c_n);

	nvhdcp_sor_writel(hdmi, TMDS0_LINK0 | READ_S,
					NV_SOR_TMDS_HDCP_CMODE);

	set_cksv(hdmi, pkt->c_ksv);

	e = wait_hdcp_ctrl(hdmi, SPRIME_VALID, NULL);
	if (e) {
		nvhdcp_err("Sprime read timeout\n");
		pkt->packet_results = TEGRA_NVHDCP_RESULT_UNSUCCESSFUL;
		e = -EIO;
		goto err;
	}

	msleep(50);

	/* read 56-bit Sprime plus 16 status bits */
	sp_msb = nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_SPRIME_MSB);
	sp_lsb1 = nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_SPRIME_LSB1);
	sp_lsb2 = nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_SPRIME_LSB2);

	/* top 8 bits of LSB2 and bottom 8 bits of MSB hold status bits. */
	pkt->hdcp_status = (sp_msb << 8) | (sp_lsb2 >> 24);
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_S;

	/* 56-bit Kprime */
	pkt->k_prime = ((u64)(sp_lsb2 & 0xffffff) << 32) | sp_lsb1;
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_KP;

	/* is connection state supported? */
	if (sp_msb & STATUS_CS) {
		pkt->cs = get_cs(hdmi);
		pkt->value_flags |= TEGRA_NVHDCP_FLAG_CS;
	}

	/* load Dksv */
	pkt->d_ksv = get_dksv(hdmi);
	if (verify_ksv(pkt->d_ksv)) {
		nvhdcp_err("Dksv invalid!\n");
		pkt->packet_results = TEGRA_NVHDCP_RESULT_UNSUCCESSFUL;
		e = -EIO; /* treat bad Dksv as I/O error */
	}
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_DKSV;

	/* copy current Bksv */
	pkt->b_ksv = nvhdcp->b_ksv;
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_BKSV;

	pkt->packet_results = TEGRA_NVHDCP_RESULT_SUCCESS;
	mutex_unlock(&nvhdcp->lock);
	return 0;

err:
	mutex_unlock(&nvhdcp->lock);
	return e;
}

/* get M prime - READ_M on TMDS0_LINK0 only */
static inline int get_m_prime(struct tegra_nvhdcp *nvhdcp,
				struct tegra_nvhdcp_packet *pkt)
{
	struct tegra_hdmi *hdmi = nvhdcp->hdmi;
	int e;

	pkt->packet_results = TEGRA_NVHDCP_RESULT_UNSUCCESSFUL;

	/* if connection isn't authenticated ... */
	mutex_lock(&nvhdcp->lock);
	if (nvhdcp->state != STATE_LINK_VERIFY) {
		memset(pkt, 0, sizeof(*pkt));
		pkt->packet_results = TEGRA_NVHDCP_RESULT_LINK_FAILED;
		e = 0;
		goto err;
	}

	pkt->a_ksv = nvhdcp->a_ksv;
	pkt->a_n = nvhdcp->a_n;
	pkt->value_flags = TEGRA_NVHDCP_FLAG_AKSV | TEGRA_NVHDCP_FLAG_AN;

	set_cn(hdmi, pkt->c_n);

	nvhdcp_sor_writel(hdmi, TMDS0_LINK0 | READ_M,
					NV_SOR_TMDS_HDCP_CMODE);

	/* Cksv write triggers Mprime update */
	set_cksv(hdmi, pkt->c_ksv);

	e = wait_hdcp_ctrl(hdmi, MPRIME_VALID, NULL);
	if (e) {
		nvhdcp_err("Mprime read timeout\n");
		e = -EIO;
		goto err;
	}
	msleep(50);

	/* load Mprime */
	pkt->m_prime = get_mprime(hdmi);
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_MP;

	pkt->b_status = nvhdcp->b_status;
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_BSTATUS;

	/* copy most recent KSVFIFO, if it is non-zero */
	pkt->num_bksv_list = nvhdcp->num_bksv_list;
	if (nvhdcp->num_bksv_list) {
		BUILD_BUG_ON(sizeof(pkt->bksv_list) !=
				sizeof(nvhdcp->bksv_list));
		memcpy(pkt->bksv_list, nvhdcp->bksv_list,
			nvhdcp->num_bksv_list * sizeof(*pkt->bksv_list));
		pkt->value_flags |= TEGRA_NVHDCP_FLAG_BKSVLIST;
	}

	/* copy v_prime */
	BUILD_BUG_ON(sizeof(pkt->v_prime) != sizeof(nvhdcp->v_prime));
	memcpy(pkt->v_prime, nvhdcp->v_prime, sizeof(nvhdcp->v_prime));
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_V;

	/* load Dksv */
	pkt->d_ksv = get_dksv(hdmi);
	if (verify_ksv(pkt->d_ksv)) {
		nvhdcp_err("Dksv invalid!\n");
		e = -EIO;
		goto err;
	}
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_DKSV;

	/* copy current Bksv */
	pkt->b_ksv = nvhdcp->b_ksv;
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_BKSV;

	pkt->packet_results = TEGRA_NVHDCP_RESULT_SUCCESS;
	mutex_unlock(&nvhdcp->lock);
	return 0;

err:
	mutex_unlock(&nvhdcp->lock);
	return e;
}

static int load_kfuse(struct tegra_hdmi *hdmi)
{
	unsigned buf[KFUSE_DATA_SZ / 4];
	int e, i;
	u32 ctrl;
	u32 tmp;
	int retries;

	/* copy load kfuse into buffer - only needed for early Tegra parts */
	e = tegra_kfuse_read(buf, sizeof(buf));
	if (e) {
		nvhdcp_err("Kfuse read failure\n");
		return e;
	}

	/* write the kfuse to HDMI SRAM */

	nvhdcp_sor_writel(hdmi, 1, NV_SOR_KEY_CTRL); /* LOAD_KEYS */

	/* issue a reload */
	ctrl = nvhdcp_sor_readl(hdmi, NV_SOR_KEY_CTRL);
	nvhdcp_sor_writel(hdmi, ctrl | PKEY_RELOAD_TRIGGER
					| LOCAL_KEYS , NV_SOR_KEY_CTRL);

	e = wait_key_ctrl(hdmi, PKEY_LOADED, PKEY_LOADED);
	if (e) {
		nvhdcp_err("key reload timeout\n");
		return -EIO;
	}

	nvhdcp_sor_writel(hdmi, 0, NV_SOR_KEY_SKEY_INDEX);

	/* wait for SRAM to be cleared */
	retries = 6;
	do {
		tmp = nvhdcp_sor_readl(hdmi, NV_SOR_KEY_DEBUG0);
		if ((tmp & 1) == 0)
			break;
		if (retries > 1)
			mdelay(1);
	} while (--retries);
	if (!retries) {
		nvhdcp_err("key SRAM clear timeout\n");
		return -EIO;
	}

	for (i = 0; i < KFUSE_DATA_SZ / 4; i += 4) {

		/* load 128-bits*/
		nvhdcp_sor_writel(hdmi, buf[i], NV_SOR_KEY_HDCP_KEY_0);
		nvhdcp_sor_writel(hdmi, buf[i+1], NV_SOR_KEY_HDCP_KEY_1);
		nvhdcp_sor_writel(hdmi, buf[i+2], NV_SOR_KEY_HDCP_KEY_2);
		nvhdcp_sor_writel(hdmi, buf[i+3], NV_SOR_KEY_HDCP_KEY_3);

		/* trigger LOAD_HDCP_KEY */
		nvhdcp_sor_writel(hdmi, 0x100, NV_SOR_KEY_HDCP_KEY_TRIG);

		tmp = LOCAL_KEYS | WRITE16;
		if (i)
			tmp |= AUTOINC;
		nvhdcp_sor_writel(hdmi, tmp, NV_SOR_KEY_CTRL);

		/* wait for WRITE16 to complete */
		e = wait_key_ctrl(hdmi, 0x10, 0); /* WRITE16 */
		if (e) {
			nvhdcp_err("key write timeout\n");
			return -EIO;
		}
	}

	return 0;
}

static int verify_link(struct tegra_nvhdcp *nvhdcp, bool wait_ri)
{
	struct tegra_hdmi *hdmi = nvhdcp->hdmi;
	int retries = 3;
	u16 old, rx, tx;
	int e;

	old = 0;
	rx = 0;
	tx = 0;
	/* retry 3 times to deal with I2C link issues */
	do {
		if (wait_ri)
			old = get_transmitter_ri(hdmi);

		e = get_receiver_ri(nvhdcp, &rx);
		if (!e) {
			if (!rx) {
				nvhdcp_err("Ri is 0!\n");
				return -EINVAL;
			}

			tx = get_transmitter_ri(hdmi);
		} else {
			rx = ~tx;
			msleep(50);
		}

	} while (wait_ri && --retries && old != tx);

	nvhdcp_debug("R0 Ri poll:rx=0x%04x tx=0x%04x\n", rx, tx);

	if (!nvhdcp_is_plugged(nvhdcp)) {
		nvhdcp_err("aborting verify links - lost hdmi connection\n");
		return -EIO;
	}

	if (rx != tx)
		return -EINVAL;

	return 0;
}

static int get_repeater_info(struct tegra_nvhdcp *nvhdcp)
{
	int e, retries;
	u8 b_caps;
	u16 b_status;

	nvhdcp_vdbg("repeater found:fetching repeater info\n");

	/* wait up to 5 seconds for READY on repeater */
	retries = 51;
	do {
		if (!nvhdcp_is_plugged(nvhdcp)) {
			nvhdcp_err("disconnect while waiting for repeater\n");
			return -EIO;
		}

		e = get_bcaps(nvhdcp, &b_caps);
		if (!e && (b_caps & BCAPS_READY)) {
			nvhdcp_debug("Bcaps READY from repeater\n");
			break;
		}
		if (retries > 1)
			msleep(100);
	} while (--retries);
	if (!retries) {
		nvhdcp_err("repeater Bcaps read timeout\n");
		return -ETIMEDOUT;
	}

	memset(nvhdcp->v_prime, 0, sizeof(nvhdcp->v_prime));
	e = get_vprime(nvhdcp, nvhdcp->v_prime);
	if (e) {
		nvhdcp_err("repeater Vprime read failure!\n");
		return e;
	}

	e = nvhdcp_i2c_read16(nvhdcp, 0x41, &b_status);
	if (e) {
		nvhdcp_err("Bstatus read failure!\n");
		return e;
	}

	if (b_status & BSTATUS_MAX_DEVS_EXCEEDED) {
		nvhdcp_err("repeater:max devices (0x%04x)\n", b_status);
		return -EINVAL;
	}

	if (b_status & BSTATUS_MAX_CASCADE_EXCEEDED) {
		nvhdcp_err("repeater:max cascade (0x%04x)\n", b_status);
		return -EINVAL;
	}

	nvhdcp->b_status = b_status;
	nvhdcp->num_bksv_list = b_status & 0x7f;
	nvhdcp_vdbg("Bstatus 0x%x (devices: %d)\n",
				b_status, nvhdcp->num_bksv_list);

	memset(nvhdcp->bksv_list, 0, sizeof(nvhdcp->bksv_list));
	e = get_ksvfifo(nvhdcp, nvhdcp->num_bksv_list, nvhdcp->bksv_list);
	if (e) {
		nvhdcp_err("repeater:could not read KSVFIFO (err %d)\n", e);
		return e;
	}

	return 0;
}

static int nvhdcp_ake_init_send(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_write(nvhdcp, 0x60, SIZE_AKE_INIT, buf);
	return e;
}

static int nvhdcp_ake_cert_receive(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_read(nvhdcp, 0x80, SIZE_AKE_SEND_CERT, buf);
	return e;
}

static int nvhdcp_ake_no_stored_km_send(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_write(nvhdcp, 0x60, SIZE_AKE_NO_STORED_KM, buf);
	return e;
}

static int nvhdcp_ake_hprime_receive(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_read(nvhdcp, 0x80, SIZE_AKE_SEND_HPRIME, buf);
	return e;
}

static int nvhdcp_ake_pairing_info_receive(struct tegra_nvhdcp *nvhdcp,
	u8 *buf)
{
	int e;
	e = nvhdcp_i2c_read(nvhdcp, 0x80, SIZE_AKE_SEND_PAIRING_INFO, buf);
	return e;
}

static int nvhdcp_lc_init_send(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_write(nvhdcp, 0x60, SIZE_LC_INIT, buf);
	return e;
}

static int nvhdcp_lc_lprime_receive(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_read(nvhdcp, 0x80, SIZE_LC_SEND_LPRIME, buf);
	return e;
}

static int nvhdcp_ske_eks_send(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_write(nvhdcp, 0x60, SIZE_SKE_SEND_EKS, buf);
	return e;
}

static int nvhdcp_receiverid_list_receive(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_read(nvhdcp, 0x80, SIZE_SEND_RCVR_ID_LIST, buf);
	return e;
}

static int nvhdcp_rptr_ack_send(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_write(nvhdcp, 0x60, SIZE_SEND_RPTR_ACK, buf);
	return e;
}

static int nvhdcp_rptr_stream_manage_send(struct tegra_nvhdcp *nvhdcp,
	u8 *buf)
{
	int e;
	e = nvhdcp_i2c_write(nvhdcp, 0x60, SIZE_SEND_RPTR_STREAM_MANAGE, buf);
	return e;
}

static int nvhdcp_rptr_stream_ready_receive(struct tegra_nvhdcp *nvhdcp,
	u8 *buf)
{
	int e;
	e = nvhdcp_i2c_read(nvhdcp, 0x80, SIZE_SEND_RPTR_STREAM_READY, buf);
	return e;
}

static int nvhdcp_poll(struct tegra_nvhdcp *nvhdcp, int timeout, int status)
{
	int e;
	u16 val;
	s64 start_time;
	s64 end_time;
	struct timespec tm;
	ktime_get_ts(&tm);
	start_time = timespec_to_ns(&tm);
	while (1) {
		ktime_get_ts(&tm);
		end_time = timespec_to_ns(&tm);
		if (end_time - start_time >= timeout*1000000)
			return -ETIMEDOUT;
		else {
			e = nvhdcp_i2c_read(nvhdcp, 0x70, 2, &val);
			if (e) {
				nvhdcp_err("nvhdcp_poll_ready failed\n");
				goto exit;
			}
			if (status == HDCP_READY) {
				if (val)
					break;
			} else if (status == HDCP_REAUTH) {
				if (cpu_to_be16(val) & HDCP_REAUTH_MASK)
					break;
			}
		}
	}
	e = 0;
exit:
	return e;
}

static int nvhdcp_poll_ready(struct tegra_nvhdcp *nvhdcp, int timeout)
{
	int e;
	e = nvhdcp_poll(nvhdcp, timeout, HDCP_READY);
	return e;
}

static int nvhdcp_poll_rptr_reauth(struct tegra_nvhdcp *nvhdcp, int timeout)
{
	int e;
	e = nvhdcp_poll(nvhdcp, timeout, HDCP_REAUTH);
	if (e == -ETIMEDOUT)
		return 0;
	else
		return -1;
}

static int tsec_hdcp_authentication(struct tegra_nvhdcp *nvhdcp,
				struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	u8 version = 2;
	u16 caps = 0;
	u16 txcaps = 0x0;
	err =  tsec_hdcp_readcaps(hdcp_context);
	if (err)
		goto exit;
	err =  tsec_hdcp_init(hdcp_context);
	if (err)
		goto exit;
	err =  tsec_hdcp_create_session(hdcp_context);
	if (err)
		goto exit;
	err =  tsec_hdcp_exchange_info(hdcp_context,
			HDCP_EXCHANGE_INFO_GET_TMTR_INFO,
			&version,
			&caps);
	if (err)
		goto exit;
	hdcp_context->msg.txcaps_version = version;
	hdcp_context->msg.txcaps_capmask = txcaps;
	hdcp_context->msg.ake_init_msg_id = ID_AKE_INIT;

	err = nvhdcp_ake_init_send(nvhdcp,
		(u8 *)&hdcp_context->msg.ake_init_msg_id);
	if (err)
		goto exit;

	err = nvhdcp_poll_ready(nvhdcp, 1000);
	if (err)
		goto exit;
	err = nvhdcp_ake_cert_receive(nvhdcp,
		&hdcp_context->msg.ake_send_cert_msg_id);
	if (err)
		goto exit;
	if (hdcp_context->msg.ake_send_cert_msg_id != ID_AKE_SEND_CERT) {
		nvhdcp_err("Not ID_AKE_SEND_CERT but %d instead\n",
			hdcp_context->msg.ake_send_cert_msg_id);
		err = -EINVAL;
		goto exit;
	}
	err =  tsec_hdcp_verify_cert(hdcp_context);
	if (err)
		goto exit;
	err =  tsec_hdcp_update_rrx(hdcp_context);
	if (err)
		goto exit;
	err =  tsec_hdcp_generate_ekm(hdcp_context);
	if (err)
		goto exit;
	hdcp_context->msg.ake_no_stored_km_msg_id = ID_AKE_NO_STORED_KM;
	err = nvhdcp_ake_no_stored_km_send(nvhdcp,
		&hdcp_context->msg.ake_no_stored_km_msg_id);
	if (err)
		goto exit;
	err =  tsec_hdcp_exchange_info(hdcp_context,
		HDCP_EXCHANGE_INFO_SET_RCVR_INFO,
		&hdcp_context->msg.rxcaps_version,
		&hdcp_context->msg.rxcaps_capmask);
	if (err)
		goto exit;
	err =  tsec_hdcp_revocation_check(hdcp_context);
	if (err)
		goto exit;
	err = nvhdcp_poll_ready(nvhdcp, 1000);
	if (err)
		goto exit;
	err = nvhdcp_ake_hprime_receive(nvhdcp,
		&hdcp_context->msg.ake_send_hprime_msg_id);
	if (err)
		goto exit;
	if (hdcp_context->msg.ake_send_hprime_msg_id != ID_AKE_SEND_HPRIME) {
		nvhdcp_err("Not ID_AKE_SEND_HPRIME but %d instead\n",
			hdcp_context->msg.ake_send_hprime_msg_id);
		err = -EINVAL;
		goto exit;
	}
	err =  tsec_hdcp_verify_hprime(hdcp_context);
	if (err)
		goto exit;
	err = nvhdcp_poll_ready(nvhdcp, 200);
	if (err)
		goto exit;
	err = nvhdcp_ake_pairing_info_receive(nvhdcp,
		&hdcp_context->msg.ake_send_pairing_info_msg_id);
	if (err)
		goto exit;
	if (hdcp_context->msg.ake_send_pairing_info_msg_id !=
	ID_AKE_SEND_PAIRING_INFO) {
		nvhdcp_err("Not ID_AKE_SEND_PAIRING_INFO but %d instead\n",
			hdcp_context->msg.ake_send_hprime_msg_id);
		err = -EINVAL;
		goto exit;
	}
	err =  tsec_hdcp_encrypt_pairing_info(hdcp_context);
	if (err)
		goto exit;
	err =  tsec_hdcp_generate_lc_init(hdcp_context);
	if (err)
		goto exit;
	hdcp_context->msg.lc_init_msg_id = ID_LC_INIT;
	err = nvhdcp_lc_init_send(nvhdcp, &hdcp_context->msg.lc_init_msg_id);
	if (err)
		goto exit;
	err = nvhdcp_poll_ready(nvhdcp, 20);
	if (err)
		goto exit;
	err = nvhdcp_lc_lprime_receive(nvhdcp,
		&hdcp_context->msg.lc_send_lprime_msg_id);
	if (err)
		goto exit;
	if (hdcp_context->msg.lc_send_lprime_msg_id != ID_LC_SEND_LPRIME) {
		nvhdcp_err("Not ID_LC_SEND_LPRIME but %d instead\n",
			hdcp_context->msg.lc_send_lprime_msg_id);
		err = -EINVAL;
		goto exit;
	}
	err =  tsec_hdcp_verify_lprime(hdcp_context);
	if (err)
		goto exit;
	err =  tsec_hdcp_ske_init(hdcp_context);
	if (err)
		goto exit;
	hdcp_context->msg.ske_send_eks_msg_id = ID_SKE_SEND_EKS;
	err = nvhdcp_ske_eks_send(nvhdcp,
		&hdcp_context->msg.ske_send_eks_msg_id);
	if (err)
		goto exit;
	if (hdcp_context->msg.rxcaps_capmask & HDCP_22_REPEATER) {
		err = nvhdcp_poll_ready(nvhdcp, 2000);
		if (err)
			goto exit;
		err = nvhdcp_receiverid_list_receive(nvhdcp,
				&hdcp_context->msg.send_receiverid_list_msg_id);
		if (err)
			goto exit;
		if (hdcp_context->msg.send_receiverid_list_msg_id !=
		ID_SEND_RCVR_ID_LIST) {
			nvhdcp_err("Not ID_SEND_RCVR_ID_LIST but %d instead\n",
				hdcp_context->msg.send_receiverid_list_msg_id);
			err = -EINVAL;
			goto exit;
		}
		err =  tsec_hdcp_verify_vprime(hdcp_context);
		if (err)
			goto exit;
		hdcp_context->msg.rptr_send_ack_msg_id = ID_SEND_RPTR_ACK;
		err = nvhdcp_rptr_ack_send(nvhdcp,
			&hdcp_context->msg.rptr_send_ack_msg_id);
		if (err)
			goto exit;
		/* Poll for 5secs to see if Rxstatus has ReAuth bit set */
		err = nvhdcp_poll_rptr_reauth(nvhdcp, 5);
		if (err)
			goto exit;
		hdcp_context->msg.rptr_auth_stream_manage_msg_id =
			ID_SEND_RPTR_STREAM_MANAGE;
		memset(hdcp_context->msg.seq_num_m, 0x0, HDCP_SIZE_SEQ_NUM_V_8);
		/* Num of streams = 1, only video, big endian */
		hdcp_context->msg.k = 0x0100;
		/* STREAM_ID = 0 and Type = 0  */
		hdcp_context->msg.streamid_type[0] = 0x0000;
		err = nvhdcp_rptr_stream_manage_send(nvhdcp,
			&hdcp_context->msg.rptr_auth_stream_manage_msg_id);
		if (err)
			goto exit;
		err = nvhdcp_poll_ready(nvhdcp, 100);
		if (err)
			goto exit;
		err = nvhdcp_rptr_stream_ready_receive(nvhdcp,
			&hdcp_context->msg.rptr_auth_stream_ready_msg_id);
		if (err)
			goto exit;
		if (hdcp_context->msg.rptr_auth_stream_ready_msg_id !=
		ID_SEND_RPTR_STREAM_READY) {
			nvhdcp_err("Not ID_SEND_RPTR_STREAM_READY but %d\n",
			hdcp_context->msg.rptr_auth_stream_ready_msg_id);
			err = -EINVAL;
			goto exit;
		}
		err =  tsec_hdcp_rptr_stream_ready(hdcp_context);
		if (err)
			goto exit;
	}

	nvhdcp_info("HDCP Authentication successful!\n");

exit:
	if (err)
		nvhdcp_err("HDCP authentication failed with err %d\n", err);
	return err;
}

static void nvhdcp_downstream_worker(struct work_struct *work)
{
	struct tegra_nvhdcp *nvhdcp =
		container_of(to_delayed_work(work), struct tegra_nvhdcp, work);
	struct tegra_hdmi *hdmi = nvhdcp->hdmi;
	struct tegra_dc *dc = tegra_dc_hdmi_get_dc(hdmi);
	int e;
	u8 b_caps;
	u32 tmp;
	u32 res;

	nvhdcp_vdbg("%s():started thread %s\n", __func__, nvhdcp->name);
	tegra_dc_io_start(dc);

	mutex_lock(&nvhdcp->lock);
	if (nvhdcp->state == STATE_OFF) {
		nvhdcp_err("nvhdcp failure - giving up\n");
		goto err;
	}
	nvhdcp->state = STATE_UNAUTHENTICATED;

	/* check plug state to terminate early in case flush_workqueue() */
	if (!nvhdcp_is_plugged(nvhdcp)) {
		nvhdcp_err("worker started while unplugged!\n");
		goto lost_hdmi;
	}
	nvhdcp_vdbg("%s():hpd=%d\n", __func__, nvhdcp->plugged);

	nvhdcp->a_ksv = 0;
	nvhdcp->b_ksv = 0;
	nvhdcp->a_n = 0;

	e = get_bcaps(nvhdcp, &b_caps);
	if (e) {
		nvhdcp_err("Bcaps read failure\n");
		goto failure;
	}

	nvhdcp_vdbg("read Bcaps = 0x%02x\n", b_caps);

	nvhdcp_vdbg("kfuse loading ...\n");

	/* repeater flag in Bskv must be configured before loading fuses */
	set_bksv(hdmi, 0, (b_caps & BCAPS_REPEATER));

	e = load_kfuse(hdmi);
	if (e) {
		nvhdcp_err("kfuse could not be loaded\n");
		goto failure;
	}

	hdcp_ctrl_run(hdmi, 1);

	nvhdcp_vdbg("wait AN_VALID ...\n");

	/* wait for hardware to generate HDCP values */
	e = wait_hdcp_ctrl(hdmi, AN_VALID | SROM_ERR, &res);
	if (e) {
		nvhdcp_err("An key generation timeout\n");
		goto failure;
	}
	if (res & SROM_ERR) {
		nvhdcp_err("SROM error\n");
		goto failure;
	}

	msleep(25);

	nvhdcp->a_ksv = get_aksv(hdmi);
	nvhdcp->a_n = get_an(hdmi);
	nvhdcp_vdbg("Aksv is 0x%016llx\n", nvhdcp->a_ksv);
	nvhdcp_vdbg("An is 0x%016llx\n", nvhdcp->a_n);
	if (verify_ksv(nvhdcp->a_ksv)) {
		nvhdcp_err("Aksv verify failure! (0x%016llx)\n", nvhdcp->a_ksv);
		goto disable;
	}

	/* write Ainfo to receiver - set 1.1 only if b_caps supports it */
	e = nvhdcp_i2c_write8(nvhdcp, 0x15, b_caps & BCAPS_11);
	if (e) {
		nvhdcp_err("Ainfo write failure\n");
		goto failure;
	}

	/* write An to receiver */
	e = nvhdcp_i2c_write64(nvhdcp, 0x18, nvhdcp->a_n);
	if (e) {
		nvhdcp_err("An write failure\n");
		goto failure;
	}

	nvhdcp_vdbg("wrote An = 0x%016llx\n", nvhdcp->a_n);

	/* write Aksv to receiver - triggers auth sequence */
	e = nvhdcp_i2c_write40(nvhdcp, 0x10, nvhdcp->a_ksv);
	if (e) {
		nvhdcp_err("Aksv write failure\n");
		goto failure;
	}

	nvhdcp_vdbg("wrote Aksv = 0x%010llx\n", nvhdcp->a_ksv);

	/* bail out if unplugged in the middle of negotiation */
	if (!nvhdcp_is_plugged(nvhdcp))
		goto lost_hdmi;

	/* get Bksv from receiver */
	e = nvhdcp_i2c_read40(nvhdcp, 0x00, &nvhdcp->b_ksv);
	if (e) {
		nvhdcp_err("Bksv read failure\n");
		goto failure;
	}
	nvhdcp_vdbg("Bksv is 0x%016llx\n", nvhdcp->b_ksv);
	if (verify_ksv(nvhdcp->b_ksv)) {
		nvhdcp_err("Bksv verify failure!\n");
		goto failure;
	}

	nvhdcp_vdbg("read Bksv = 0x%010llx from device\n", nvhdcp->b_ksv);

	set_bksv(hdmi, nvhdcp->b_ksv, (b_caps & BCAPS_REPEATER));

	nvhdcp_vdbg("loaded Bksv into controller\n");

	e = wait_hdcp_ctrl(hdmi, R0_VALID, NULL);
	if (e) {
		nvhdcp_err("R0 read failure!\n");
		goto failure;
	}

	nvhdcp_vdbg("R0 valid\n");

	msleep(100); /* can't read R0' within 100ms of writing Aksv */

	nvhdcp_vdbg("verifying links ...\n");

	e = verify_link(nvhdcp, false);
	if (e) {
		nvhdcp_err("link verification failed err %d\n", e);
		goto failure;
	}

	/* if repeater then get repeater info */
	if (b_caps & BCAPS_REPEATER) {
		e = get_repeater_info(nvhdcp);
		if (e) {
			nvhdcp_err("get repeater info failed\n");
			goto failure;
		}
	}

	tmp = nvhdcp_sor_readl(hdmi, NV_SOR_TMDS_HDCP_CTRL);
	tmp |= CRYPT_ENABLED;
	if (b_caps & BCAPS_11) /* HDCP 1.1 ? */
		tmp |= ONEONE_ENABLED;
	nvhdcp_sor_writel(hdmi, tmp, NV_SOR_TMDS_HDCP_CTRL);

	nvhdcp_vdbg("CRYPT enabled\n");

	nvhdcp->state = STATE_LINK_VERIFY;
	nvhdcp_info("link verified!\n");

	while (1) {
		if (!nvhdcp_is_plugged(nvhdcp))
			goto lost_hdmi;

		if (nvhdcp->state != STATE_LINK_VERIFY)
			goto failure;

		e = verify_link(nvhdcp, true);
		if (e) {
			nvhdcp_err("link verification failed err %d\n", e);
			goto failure;
		}
		mutex_unlock(&nvhdcp->lock);
		tegra_dc_io_end(dc);
		wait_event_interruptible_timeout(wq_worker,
			!nvhdcp_is_plugged(nvhdcp), msecs_to_jiffies(1500));
		tegra_dc_io_start(dc);
		mutex_lock(&nvhdcp->lock);

	}

failure:
	nvhdcp->fail_count++;
	if (nvhdcp->fail_count > 5) {
		nvhdcp_err("nvhdcp failure - too many failures, giving up!\n");
	} else {
		nvhdcp_err("nvhdcp failure - renegotiating in 1 second\n");
		if (!nvhdcp_is_plugged(nvhdcp))
			goto lost_hdmi;
		queue_delayed_work(nvhdcp->downstream_wq, &nvhdcp->work,
						msecs_to_jiffies(1000));
	}

lost_hdmi:
	nvhdcp->state = STATE_UNAUTHENTICATED;
	hdcp_ctrl_run(hdmi, 0);

err:
	mutex_unlock(&nvhdcp->lock);
	tegra_dc_io_end(dc);
	return;
disable:
	nvhdcp->state = STATE_OFF;
	nvhdcp_set_plugged(nvhdcp, false);
	mutex_unlock(&nvhdcp->lock);
	tegra_dc_io_end(dc);
	return;
}

static int link_integrity_check(struct tegra_nvhdcp *nvhdcp)
{
	u16 rx_status = 0;

	nvhdcp_i2c_read16(nvhdcp, HDCP_RX_STATUS, &rx_status);
	return rx_status & HDCP_RX_STATUS_MSG_REAUTH_REQ;
}

static void nvhdcp2_downstream_worker(struct work_struct *work)
{
	struct tegra_nvhdcp *nvhdcp =
		container_of(to_delayed_work(work), struct tegra_nvhdcp, work);
	struct tegra_hdmi *hdmi = nvhdcp->hdmi;
	struct tegra_dc *dc = tegra_dc_hdmi_get_dc(hdmi);
	int e;
	struct hdcp_context_t hdcp_context;
	e = tsec_hdcp_create_context(&hdcp_context);
	if (e)
		goto err;

	nvhdcp_vdbg("%s():started thread %s\n", __func__, nvhdcp->name);
	tegra_dc_io_start(dc);

	mutex_lock(&nvhdcp->lock);
	if (nvhdcp->state == STATE_OFF) {
		nvhdcp_err("nvhdcp failure - giving up\n");
		goto err;
	}
	nvhdcp->state = STATE_UNAUTHENTICATED;

	/* check plug state to terminate early in case flush_workqueue() */
	if (!nvhdcp_is_plugged(nvhdcp)) {
		nvhdcp_err("worker started while unplugged!\n");
		goto lost_hdmi;
	}
	nvhdcp_vdbg("%s():hpd=%d\n", __func__, nvhdcp->plugged);

	if (tsec_hdcp_authentication(nvhdcp, &hdcp_context))
		goto failure;

	mdelay(250);
	e = tsec_hdcp_session_ctrl(&hdcp_context,
		HDCP_SESSION_CTRL_FLAG_ACTIVATE);
	if (e) {
		nvhdcp_info("tsec_hdcp_session_ctrl failed\n");
		goto failure;
	}
	nvhdcp_info("HDCP 2.2 crypt enabled!\n");

	/* bail out if unplugged in the middle of negotiation */
	if (!nvhdcp_is_plugged(nvhdcp))
		goto lost_hdmi;

	nvhdcp_vdbg("link integrity check ...\n");
	e = link_integrity_check(nvhdcp);
	if (e) {
		nvhdcp_err("link integrity check failed err %d\n", e);
		goto failure;
	}

	nvhdcp->state = STATE_LINK_VERIFY;
	nvhdcp_info("link integrity check passed!\n");

	while (1) {
		if (!nvhdcp_is_plugged(nvhdcp))
			goto lost_hdmi;

		if (nvhdcp->state != STATE_LINK_VERIFY)
			goto failure;

		e = link_integrity_check(nvhdcp);
		if (e) {
			nvhdcp_err("link integrity check failed err %d\n", e);
			goto failure;
		}
		mutex_unlock(&nvhdcp->lock);
		tegra_dc_io_end(dc);
		wait_event_interruptible_timeout(wq_worker,
			!nvhdcp_is_plugged(nvhdcp), msecs_to_jiffies(1000));
		tegra_dc_io_start(dc);
		mutex_lock(&nvhdcp->lock);

	}

failure:

	nvhdcp->fail_count++;
	if (nvhdcp->fail_count > 5) {
		nvhdcp_err("nvhdcp failure - too many failures, giving up!\n");
	} else {
		nvhdcp_err("nvhdcp failure - renegotiating in 1 second\n");
		if (!nvhdcp_is_plugged(nvhdcp))
			goto lost_hdmi;
		queue_delayed_work(nvhdcp->downstream_wq, &nvhdcp->work,
						msecs_to_jiffies(1000));
	}

lost_hdmi:
	nvhdcp->state = STATE_UNAUTHENTICATED;

err:
	mutex_unlock(&nvhdcp->lock);
	tegra_dc_io_end(dc);
	e = tsec_hdcp_free_context(&hdcp_context);
	return;
}

static int tegra_nvhdcp_on(struct tegra_nvhdcp *nvhdcp)
{
	u8 hdcp2version;
	nvhdcp->state = STATE_UNAUTHENTICATED;
	if (nvhdcp_is_plugged(nvhdcp)) {
		nvhdcp->fail_count = 0;
		nvhdcp_i2c_read8(nvhdcp, HDCP_HDCP2_VERSION, &hdcp2version);
		if (hdcp2version & HDCP_HDCP2_VERSION_HDCP22_YES) {
			INIT_DELAYED_WORK(&nvhdcp->work,
				nvhdcp2_downstream_worker);
			nvhdcp->hdcp22 = HDCP22_PROTOCOL;
		} else {
			INIT_DELAYED_WORK(&nvhdcp->work,
				nvhdcp_downstream_worker);
			nvhdcp->hdcp22 = HDCP1X_PROTOCOL;
		}
		queue_delayed_work(nvhdcp->downstream_wq, &nvhdcp->work,
						msecs_to_jiffies(100));
	}
	return 0;
}

static int tegra_nvhdcp_off(struct tegra_nvhdcp *nvhdcp)
{
	mutex_lock(&nvhdcp->lock);
	nvhdcp->state = STATE_OFF;
	nvhdcp_set_plugged(nvhdcp, false);
	mutex_unlock(&nvhdcp->lock);
	wake_up_interruptible(&wq_worker);
	cancel_delayed_work_sync(&nvhdcp->work);
	return 0;
}

void tegra_nvhdcp_set_plug(struct tegra_nvhdcp *nvhdcp, bool hpd)
{
	nvhdcp_debug("hdmi hotplug detected (hpd = %d)\n", hpd);

	if (hpd) {
		nvhdcp_set_plugged(nvhdcp, true);
		tegra_nvhdcp_on(nvhdcp);
	} else {
		tegra_nvhdcp_off(nvhdcp);
	}
}

int tegra_nvhdcp_set_policy(struct tegra_nvhdcp *nvhdcp, int pol)
{
	if (pol == TEGRA_NVHDCP_POLICY_ALWAYS_ON) {
		nvhdcp_info("using \"always on\" policy.\n");
		if (atomic_xchg(&nvhdcp->policy, pol) != pol) {
			/* policy changed, start working */
			tegra_nvhdcp_on(nvhdcp);
		}
	} else {
		/* unsupported policy */
		return -EINVAL;
	}

	return 0;
}

static int tegra_nvhdcp_renegotiate(struct tegra_nvhdcp *nvhdcp)
{
	mutex_lock(&nvhdcp->lock);
	nvhdcp->state = STATE_RENEGOTIATE;
	mutex_unlock(&nvhdcp->lock);
	tegra_nvhdcp_on(nvhdcp);
	return 0;
}

void tegra_nvhdcp_suspend(struct tegra_nvhdcp *nvhdcp)
{
	if (!nvhdcp)
		return;
	tegra_nvhdcp_off(nvhdcp);
}

void tegra_nvhdcp_resume(struct tegra_nvhdcp *nvhdcp)
{
	if (!nvhdcp)
		return;
	tegra_nvhdcp_renegotiate(nvhdcp);
}

static int tegra_nvhdcp_recv_capable(struct tegra_nvhdcp *nvhdcp)
{
	if (!nvhdcp)
		return 0;

	if (nvhdcp->state == STATE_LINK_VERIFY)
		return 1;
	else {
		__u64 b_ksv;
		/* get Bksv from receiver */
		if (!nvhdcp_i2c_read40(nvhdcp, 0x00, &b_ksv))
			return !verify_ksv(b_ksv);
	}
	return 0;
}

static long nvhdcp_dev_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct tegra_nvhdcp *nvhdcp = filp->private_data;
	struct tegra_nvhdcp_packet *pkt;
	int e = -ENOTTY;

	switch (cmd) {
	case TEGRAIO_NVHDCP_ON:
		return tegra_nvhdcp_on(nvhdcp);

	case TEGRAIO_NVHDCP_OFF:
		return tegra_nvhdcp_off(nvhdcp);

	case TEGRAIO_NVHDCP_SET_POLICY:
		return tegra_nvhdcp_set_policy(nvhdcp, arg);

	case TEGRAIO_NVHDCP_READ_M:
		pkt = kmalloc(sizeof(*pkt), GFP_KERNEL);
		if (!pkt)
			return -ENOMEM;
		if (copy_from_user(pkt, (void __user *)arg, sizeof(*pkt))) {
			e = -EFAULT;
			goto kfree_pkt;
		}
		e = get_m_prime(nvhdcp, pkt);
		if (copy_to_user((void __user *)arg, pkt, sizeof(*pkt))) {
			e = -EFAULT;
			goto kfree_pkt;
		}
		kfree(pkt);
		return e;

	case TEGRAIO_NVHDCP_READ_S:
		pkt = kmalloc(sizeof(*pkt), GFP_KERNEL);
		if (!pkt)
			return -ENOMEM;
		if (copy_from_user(pkt, (void __user *)arg, sizeof(*pkt))) {
			e = -EFAULT;
			goto kfree_pkt;
		}
		e = get_s_prime(nvhdcp, pkt);
		if (copy_to_user((void __user *)arg, pkt, sizeof(*pkt))) {
			e = -EFAULT;
			goto kfree_pkt;
		}
		kfree(pkt);
		return e;

	case TEGRAIO_NVHDCP_RENEGOTIATE:
		e = tegra_nvhdcp_renegotiate(nvhdcp);
		break;

	case TEGRAIO_NVHDCP_HDCP_STATE:
		pkt = kmalloc(sizeof(*pkt), GFP_KERNEL);
		if (!pkt)
			return -ENOMEM;
		e = get_nvhdcp_state(nvhdcp, pkt);
		if (copy_to_user((void __user *)arg, pkt, sizeof(*pkt))) {
			e = -EFAULT;
			goto kfree_pkt;
		}
		kfree(pkt);
		return e;

	case TEGRAIO_NVHDCP_RECV_CAPABLE:
		{
			__u32 recv_capable = tegra_nvhdcp_recv_capable(nvhdcp);
			if (copy_to_user((void __user *)arg, &recv_capable,
				sizeof(recv_capable)))
				return -EFAULT;
			return 0;
		}
	}

	return e;
kfree_pkt:
	kfree(pkt);
	return e;
}

static int nvhdcp_dev_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *miscdev = filp->private_data;
	struct tegra_nvhdcp *nvhdcp =
		container_of(miscdev, struct tegra_nvhdcp, miscdev);
	filp->private_data = nvhdcp;
	return 0;
}

static int nvhdcp_dev_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

static const struct file_operations nvhdcp_fops = {
	.owner          = THIS_MODULE,
	.llseek         = no_llseek,
	.unlocked_ioctl = nvhdcp_dev_ioctl,
	.open           = nvhdcp_dev_open,
	.release        = nvhdcp_dev_release,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = nvhdcp_dev_ioctl,
#endif
};

/* we only support one AP right now, so should only call this once. */
struct tegra_nvhdcp *tegra_nvhdcp_create(struct tegra_hdmi *hdmi,
			int id, int bus)
{
	static struct tegra_nvhdcp *nvhdcp; /* prevent multiple calls */
	struct i2c_adapter *adapter;
	int e;

	if (nvhdcp)
		return ERR_PTR(-EMFILE);

	nvhdcp = kzalloc(sizeof(*nvhdcp), GFP_KERNEL);
	if (!nvhdcp)
		return ERR_PTR(-ENOMEM);

	nvhdcp->id = id;
	snprintf(nvhdcp->name, sizeof(nvhdcp->name), "nvhdcp%u", id);
	nvhdcp->hdmi = hdmi;
	mutex_init(&nvhdcp->lock);

	strlcpy(nvhdcp->info.type, nvhdcp->name, sizeof(nvhdcp->info.type));
	nvhdcp->bus = bus;
	nvhdcp->info.addr = 0x74 >> 1;
	nvhdcp->info.platform_data = nvhdcp;
	nvhdcp->fail_count = 0;

	adapter = i2c_get_adapter(bus);
	if (!adapter) {
		nvhdcp_err("can't get adapter for bus %d\n", bus);
		e = -EBUSY;
		goto free_nvhdcp;
	}

	nvhdcp->client = i2c_new_device(adapter, &nvhdcp->info);
	i2c_put_adapter(adapter);

	if (!nvhdcp->client) {
		nvhdcp_err("can't create new device\n");
		e = -EBUSY;
		goto free_nvhdcp;
	}

	nvhdcp->state = STATE_UNAUTHENTICATED;

	nvhdcp->downstream_wq = create_singlethread_workqueue(nvhdcp->name);

	nvhdcp->miscdev.minor = MISC_DYNAMIC_MINOR;
	nvhdcp->miscdev.name = nvhdcp->name;
	nvhdcp->miscdev.fops = &nvhdcp_fops;

	e = misc_register(&nvhdcp->miscdev);
	if (e)
		goto free_workqueue;

	nvhdcp_vdbg("%s(): created misc device %s\n", __func__, nvhdcp->name);

	return nvhdcp;
free_workqueue:
	destroy_workqueue(nvhdcp->downstream_wq);
	i2c_release_client(nvhdcp->client);
free_nvhdcp:
	kfree(nvhdcp);
	nvhdcp_err("unable to create device.\n");
	return ERR_PTR(e);
}

void tegra_nvhdcp_destroy(struct tegra_nvhdcp *nvhdcp)
{
	misc_deregister(&nvhdcp->miscdev);
	tegra_nvhdcp_off(nvhdcp);
	destroy_workqueue(nvhdcp->downstream_wq);
	i2c_release_client(nvhdcp->client);
	kfree(nvhdcp);
}
