/*
 * Cryptographic API.
 * drivers/crypto/tegra-se-nvhost.c
 *
 * Support for Tegra Security Engine hardware crypto algorithms.
 *
 * Copyright (c) 2015-2016, NVIDIA Corporation. All Rights Reserved.
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
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/tegra-soc.h>
#include <linux/nvhost.h>
#include <crypto/scatterwalk.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/internal/rng.h>
#include <crypto/internal/hash.h>
#include <crypto/sha.h>
#include <linux/tegra_pm_domains.h>

#include "tegra-se-nvhost.h"
#define NV_SE1_CLASS_ID		0x3A
#define NV_SE2_CLASS_ID		0x3B
#define NV_SE3_CLASS_ID		0x3C
#define NV_SE4_CLASS_ID		0x3D
#include "../video/tegra/host/t186/hardware_t186.h"
#include "nvhost_job.h"
#include "nvhost_channel.h"
#include "nvhost_acm.h"

#define DRIVER_NAME	"tegra-se-nvhost"

#define __nvhost_opcode_nonincr(x, y)	nvhost_opcode_nonincr((x) / 4, (y))
#define __nvhost_opcode_incr(x, y)	nvhost_opcode_incr((x) / 4, (y))

/* Security Engine operation modes */
enum tegra_se_aes_op_mode {
	SE_AES_OP_MODE_CBC,	/* Cipher Block Chaining (CBC) mode */
	SE_AES_OP_MODE_ECB,	/* Electronic Codebook (ECB) mode */
	SE_AES_OP_MODE_CTR,	/* Counter (CTR) mode */
	SE_AES_OP_MODE_OFB,	/* Output feedback (CFB) mode */
	SE_AES_OP_MODE_CMAC,	/* Cipher-based MAC (CMAC) mode */
	SE_AES_OP_MODE_RNG_DRBG,	/* Deterministic Random Bit Generator */
	SE_AES_OP_MODE_SHA1,	/* Secure Hash Algorithm-1 (SHA1) mode */
	SE_AES_OP_MODE_SHA224,	/* Secure Hash Algorithm-224  (SHA224) mode */
	SE_AES_OP_MODE_SHA256,	/* Secure Hash Algorithm-256  (SHA256) mode */
	SE_AES_OP_MODE_SHA384,	/* Secure Hash Algorithm-384  (SHA384) mode */
	SE_AES_OP_MODE_SHA512	/* Secure Hash Algorithm-512  (SHA512) mode */
};

/* Security Engine key table type */
enum tegra_se_key_table_type {
	SE_KEY_TABLE_TYPE_KEY,	/* Key */
	SE_KEY_TABLE_TYPE_ORGIV,	/* Original IV */
	SE_KEY_TABLE_TYPE_UPDTDIV	/* Updated IV */
};

struct tegra_se_dev;

/* Security Engine request context */
struct tegra_se_req_context {
	enum tegra_se_aes_op_mode op_mode; /* Security Engine operation mode */
	bool encrypt;	/* Operation type */
	u32 config;
	u32 crypto_config;
	struct tegra_se_dev *se_dev;
};

struct tegra_se_chipdata {
	unsigned long aes_freq;
};

/* Security Engine Linked List */
struct tegra_se_ll {
	dma_addr_t addr; /* DMA buffer address */
	u32 data_len; /* Data length in DMA buffer */
};

struct tegra_se_dev {
	struct platform_device *pdev;
	struct device *dev;
	void __iomem *io_regs;	/* se device memory/io */
	void __iomem *pmc_io_reg;	/* pmc device memory/io */
	spinlock_t lock;	/* spin lock */
	struct mutex	mtx;
	struct clk *pclk;	/* Security Engine clock */
	struct clk *enclk;	/* Security Engine clock */
	struct crypto_queue queue; /* Security Engine crypto queue */
	struct tegra_se_slot *slot_list;	/* pointer to key slots */
	struct tegra_se_rsa_slot *rsa_slot_list; /* rsa key slot pointer */
	struct tegra_se_chipdata *chipdata; /* chip specific data */
	u32 *src_ll_buf;        /* pointer to source linked list buffer */
	dma_addr_t src_ll_buf_adr; /* Source linked list buffer dma address */
	u32 src_ll_size;        /* Size of source linked list buffer */
	u32 *dst_ll_buf;        /* pointer to destination linked list buffer */
	dma_addr_t dst_ll_buf_adr; /* Destination linked list dma address */
	u32 dst_ll_size;        /* Size of destination linked list buffer */
	struct tegra_se_ll *src_ll;
	struct tegra_se_ll *dst_ll;
	u32 syncpt_id;
	bool work_q_busy;	/* Work queue busy status */
	struct nvhost_channel *channel;
	struct work_struct se_work;
	struct workqueue_struct *se_work_q;
	int syncpt_id_num;
	int se_dev_num;
	u32 *aes_cmdbuf_cpuvaddr;
	dma_addr_t aes_cmdbuf_iova;
};

static struct tegra_se_dev *sg_tegra_se_dev[4];

/* Security Engine AES context */
struct tegra_se_aes_context {
	struct tegra_se_dev *se_dev;	/* Security Engine device */
	struct ablkcipher_request *req;
	struct tegra_se_slot *slot;	/* Security Engine key slot */
	u32 keylen;	/* key length in bits */
	u32 op_mode;	/* AES operation mode */
};

/* Security Engine random number generator context */
struct tegra_se_rng_context {
	struct tegra_se_dev *se_dev;	/* Security Engine device */
	struct ablkcipher_request *req;
	struct tegra_se_slot *slot;	/* Security Engine key slot */
	u32 *dt_buf;	/* Destination buffer pointer */
	dma_addr_t dt_buf_adr;	/* Destination buffer dma address */
	u32 *rng_buf;	/* RNG buffer pointer */
	dma_addr_t rng_buf_adr;	/* RNG buffer dma address */
	bool use_org_iv;	/* Tells whether original IV is be used
				or not. If it is false updated IV is used*/
};

/* Security Engine SHA context */
struct tegra_se_sha_context {
	struct tegra_se_dev	*se_dev;	/* Security Engine device */
	u32 op_mode;	/* SHA operation mode */
};

/* Security Engine AES CMAC context */
struct tegra_se_aes_cmac_context {
	struct tegra_se_dev *se_dev;	/* Security Engine device */
	struct tegra_se_slot *slot;	/* Security Engine key slot */
	u32 keylen;	/* key length in bits */
	u8 K1[TEGRA_SE_KEY_128_SIZE];	/* Key1 */
	u8 K2[TEGRA_SE_KEY_128_SIZE];	/* Key2 */
	dma_addr_t dma_addr;	/* DMA address of local buffer */
	u32 buflen;	/* local buffer length */
	u8	*buffer;	/* local buffer pointer */
};

/* Security Engine key slot */
struct tegra_se_slot {
	struct list_head node;
	u8 slot_num;	/* Key slot number */
	bool available; /* Tells whether key slot is free to use */
};

static struct tegra_se_slot ssk_slot = {
	.slot_num = 15,
	.available = false,
};

static struct tegra_se_slot srk_slot = {
	.slot_num = 0,
	.available = false,
};

static LIST_HEAD(key_slot);
static LIST_HEAD(rsa_key_slot);
static DEFINE_SPINLOCK(rsa_key_slot_lock);
static DEFINE_SPINLOCK(key_slot_lock);
static DEFINE_MUTEX(se_hw_lock);

#define RNG_RESEED_INTERVAL	0x00773594

/* create a work for handling the async transfers */
static void tegra_se_work_handler(struct work_struct *work);

static DEFINE_DMA_ATTRS(attrs);
static DEFINE_MUTEX(aes_cmdbuf_lock);
static unsigned int rsa_opcode_start_addr;
static unsigned int sha_opcode_start_addr;

static int force_reseed_count;

static int se_num;

#define GET_MSB(x)  ((x) >> (8*sizeof(x)-1))
static void tegra_se_leftshift_onebit(u8 *in_buf, u32 size, u8 *org_msb)
{
	u8 carry;
	u32 i;

	*org_msb = GET_MSB(in_buf[0]);

	/* left shift one bit */
	in_buf[0] <<= 1;
	for (carry = 0, i = 1; i < size; i++) {
		carry = GET_MSB(in_buf[i]);
		in_buf[i-1] |= carry;
		in_buf[i] <<= 1;
	}
}

static inline void se_writel(struct tegra_se_dev *se_dev,
	unsigned int val, unsigned int reg_offset)
{
	writel(val, se_dev->io_regs + reg_offset);
}

static inline unsigned int se_readl(struct tegra_se_dev *se_dev,
	unsigned int reg_offset)
{
	unsigned int val;

	val = readl(se_dev->io_regs + reg_offset);
	return val;
}

static void tegra_se_free_key_slot(struct tegra_se_slot *slot)
{
	if (slot) {
		spin_lock(&key_slot_lock);
		slot->available = true;
		spin_unlock(&key_slot_lock);
	}
}

static struct tegra_se_slot *tegra_se_alloc_key_slot(void)
{
	struct tegra_se_slot *slot = NULL;
	bool found = false;

	spin_lock(&key_slot_lock);
	list_for_each_entry(slot, &key_slot, node) {
		if (slot->available) {
			slot->available = false;
			found = true;
			break;
		}
	}
	spin_unlock(&key_slot_lock);
	return found ? slot : NULL;
}

static int tegra_init_key_slot(struct tegra_se_dev *se_dev)
{
	int i;

	se_dev->slot_list = kzalloc(sizeof(struct tegra_se_slot) *
					TEGRA_SE_KEYSLOT_COUNT, GFP_KERNEL);
	if (se_dev->slot_list == NULL) {
		dev_err(se_dev->dev, "slot list memory allocation failed\n");
		return -ENOMEM;
	}
	spin_lock_init(&key_slot_lock);
	spin_lock(&key_slot_lock);
	for (i = 0; i < TEGRA_SE_KEYSLOT_COUNT; i++) {
		/*
		 * Slot 0 and 15 are reserved and will not be added to the
		 * free slots pool. Slot 0 is used for SRK generation and
		 * Slot 15 is used for SSK operation
		 */
		if ((i == srk_slot.slot_num) || (i == ssk_slot.slot_num))
			continue;
		se_dev->slot_list[i].available = true;
		se_dev->slot_list[i].slot_num = i;
		INIT_LIST_HEAD(&se_dev->slot_list[i].node);
		list_add_tail(&se_dev->slot_list[i].node, &key_slot);
	}
	spin_unlock(&key_slot_lock);

	return 0;
}

static int tegra_se_alloc_ll_buf(struct tegra_se_dev *se_dev,
	u32 num_src_sgs, u32 num_dst_sgs)
{
	if (se_dev->src_ll_buf || se_dev->dst_ll_buf) {
		dev_err(se_dev->dev,
			"trying to allocate memory to allocated memory\n");
		return -EBUSY;
	}

	if (num_src_sgs) {
		se_dev->src_ll_size =
			(sizeof(struct tegra_se_ll) * num_src_sgs) +
				sizeof(u32);
		se_dev->src_ll_buf = dma_alloc_coherent(se_dev->dev,
					se_dev->src_ll_size,
					&se_dev->src_ll_buf_adr, GFP_KERNEL);
		if (!se_dev->src_ll_buf) {
			dev_err(se_dev->dev,
				"can not allocate src lldma buffer\n");
			return -ENOMEM;
		}
	}
	if (num_dst_sgs) {
		se_dev->dst_ll_size =
				(sizeof(struct tegra_se_ll) * num_dst_sgs) +
						sizeof(u32);
		se_dev->dst_ll_buf = dma_alloc_coherent(se_dev->dev,
					se_dev->dst_ll_size,
					&se_dev->dst_ll_buf_adr, GFP_KERNEL);
		if (!se_dev->dst_ll_buf) {
			dev_err(se_dev->dev,
				"can not allocate dst ll dma buffer\n");
			return -ENOMEM;
		}
	}
	return 0;
}

static void tegra_se_free_ll_buf(struct tegra_se_dev *se_dev)
{
	if (se_dev->src_ll_buf) {
		dma_free_coherent(se_dev->dev, se_dev->src_ll_size,
			se_dev->src_ll_buf, se_dev->src_ll_buf_adr);
		se_dev->src_ll_buf = NULL;
	}

	if (se_dev->dst_ll_buf) {
		dma_free_coherent(se_dev->dev, se_dev->dst_ll_size,
			se_dev->dst_ll_buf, se_dev->dst_ll_buf_adr);
		se_dev->dst_ll_buf = NULL;
	}
}

static u32 tegra_se_get_config(struct tegra_se_dev *se_dev,
	enum tegra_se_aes_op_mode mode, bool encrypt, u32 key_len)
{
	u32 val = 0;

	switch (mode) {
	case SE_AES_OP_MODE_CBC:
	case SE_AES_OP_MODE_CMAC:
		if (encrypt) {
			val = SE_CONFIG_ENC_ALG(ALG_AES_ENC);
			if (key_len == TEGRA_SE_KEY_256_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY256);
			else if (key_len == TEGRA_SE_KEY_192_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY192);
			else
				val |= SE_CONFIG_ENC_MODE(MODE_KEY128);
			val |= SE_CONFIG_DEC_ALG(ALG_NOP);
		} else {
			val = SE_CONFIG_DEC_ALG(ALG_AES_DEC);
			if (key_len == TEGRA_SE_KEY_256_SIZE)
				val |= SE_CONFIG_DEC_MODE(MODE_KEY256);
			else if (key_len == TEGRA_SE_KEY_192_SIZE)
				val |= SE_CONFIG_DEC_MODE(MODE_KEY192);
			else
				val |= SE_CONFIG_DEC_MODE(MODE_KEY128);
		}
		if (mode == SE_AES_OP_MODE_CMAC)
			val |= SE_CONFIG_DST(DST_HASHREG);
		else
			val |= SE_CONFIG_DST(DST_MEMORY);
		break;

	case SE_AES_OP_MODE_RNG_DRBG:
		val = SE_CONFIG_ENC_ALG(ALG_RNG) |
			SE_CONFIG_ENC_MODE(MODE_KEY192) |
				SE_CONFIG_DST(DST_MEMORY);
		break;

	case SE_AES_OP_MODE_ECB:
		if (encrypt) {
			val = SE_CONFIG_ENC_ALG(ALG_AES_ENC);
			if (key_len == TEGRA_SE_KEY_256_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY256);
			else if (key_len == TEGRA_SE_KEY_192_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY192);
			else
				val |= SE_CONFIG_ENC_MODE(MODE_KEY128);
		} else {
			val = SE_CONFIG_DEC_ALG(ALG_AES_DEC);
			if (key_len == TEGRA_SE_KEY_256_SIZE)
				val |= SE_CONFIG_DEC_MODE(MODE_KEY256);
			else if (key_len == TEGRA_SE_KEY_192_SIZE)
				val |= SE_CONFIG_DEC_MODE(MODE_KEY192);
			else
				val |= SE_CONFIG_DEC_MODE(MODE_KEY128);
		}
		val |= SE_CONFIG_DST(DST_MEMORY);
		break;
	case SE_AES_OP_MODE_CTR:
		if (encrypt) {
			val = SE_CONFIG_ENC_ALG(ALG_AES_ENC);
			if (key_len == TEGRA_SE_KEY_256_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY256);
			else if (key_len == TEGRA_SE_KEY_192_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY192);
			else
				val |= SE_CONFIG_ENC_MODE(MODE_KEY128);
		} else {
			val = SE_CONFIG_DEC_ALG(ALG_AES_DEC);
			if (key_len == TEGRA_SE_KEY_256_SIZE) {
				val |= SE_CONFIG_DEC_MODE(MODE_KEY256);
				val |= SE_CONFIG_ENC_MODE(MODE_KEY256);
			} else if (key_len == TEGRA_SE_KEY_192_SIZE) {
				val |= SE_CONFIG_DEC_MODE(MODE_KEY192);
				val |= SE_CONFIG_ENC_MODE(MODE_KEY192);
			} else {
				val |= SE_CONFIG_DEC_MODE(MODE_KEY128);
				val |= SE_CONFIG_ENC_MODE(MODE_KEY128);
			}
		}
		val |= SE_CONFIG_DST(DST_MEMORY);
		break;
	case SE_AES_OP_MODE_OFB:
		if (encrypt) {
			val = SE_CONFIG_ENC_ALG(ALG_AES_ENC);
			if (key_len == TEGRA_SE_KEY_256_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY256);
			else if (key_len == TEGRA_SE_KEY_192_SIZE)
				val |= SE_CONFIG_ENC_MODE(MODE_KEY192);
			else
				val |= SE_CONFIG_ENC_MODE(MODE_KEY128);
		} else {
			val = SE_CONFIG_DEC_ALG(ALG_AES_DEC);
			if (key_len == TEGRA_SE_KEY_256_SIZE) {
				val |= SE_CONFIG_DEC_MODE(MODE_KEY256);
				val |= SE_CONFIG_ENC_MODE(MODE_KEY256);
			} else if (key_len == TEGRA_SE_KEY_192_SIZE) {
				val |= SE_CONFIG_DEC_MODE(MODE_KEY192);
				val |= SE_CONFIG_ENC_MODE(MODE_KEY192);
			} else {
				val |= SE_CONFIG_DEC_MODE(MODE_KEY128);
				val |= SE_CONFIG_ENC_MODE(MODE_KEY128);
			}
		}
		val |= SE_CONFIG_DST(DST_MEMORY);
		break;

	case SE_AES_OP_MODE_SHA1:
		val = SE_CONFIG_ENC_ALG(ALG_SHA) |
			SE_CONFIG_ENC_MODE(MODE_SHA1) |
				SE_CONFIG_DST(DST_HASHREG);
		break;
	case SE_AES_OP_MODE_SHA224:
		val = SE_CONFIG_ENC_ALG(ALG_SHA) |
			SE_CONFIG_ENC_MODE(MODE_SHA224) |
				SE_CONFIG_DST(DST_HASHREG);
		break;
	case SE_AES_OP_MODE_SHA256:
		val = SE_CONFIG_ENC_ALG(ALG_SHA) |
			SE_CONFIG_ENC_MODE(MODE_SHA256) |
				SE_CONFIG_DST(DST_HASHREG);
		break;
	case SE_AES_OP_MODE_SHA384:
		val = SE_CONFIG_ENC_ALG(ALG_SHA) |
			SE_CONFIG_ENC_MODE(MODE_SHA384) |
				SE_CONFIG_DST(DST_HASHREG);
		break;
	case SE_AES_OP_MODE_SHA512:
		val = SE_CONFIG_ENC_ALG(ALG_SHA) |
			SE_CONFIG_ENC_MODE(MODE_SHA512) |
				SE_CONFIG_DST(DST_HASHREG);
		break;
	default:
		dev_warn(se_dev->dev, "Invalid operation mode\n");
		break;
	}

	return val;
}

static void tegra_se_complete_callback(void *priv, int nr_completed)
{
	int ret = 0;
	struct ablkcipher_request *req = priv;
	req->base.complete(&req->base, ret);
}

static void se_nvhost_write_method(u32 *buf, u32 op1, u32 op2, u32 *offset)
{
	int i = 0;

	buf[i++] = op1;
	buf[i++] = op2;
	*offset = *offset + 2;
}

static int tegra_se_channel_submit_gather(struct tegra_se_dev *se_dev,
			struct ablkcipher_request *req,
			u32 *cpuvaddr, dma_addr_t iova,
			u32 offset, u32 num_words)
{
	struct nvhost_job *job = NULL;
	u32 syncpt_id = 0;
	int err = 0;

	job = nvhost_job_alloc(se_dev->channel, 1, 0, 0, 1);
	if (!job) {
		dev_err(se_dev->dev, "Nvhost Job allocation failed\n");
		return -ENOMEM;
	}

	syncpt_id = se_dev->syncpt_id;

	/* initialize job data */
	se_dev->channel->syncpts[0] = syncpt_id;
	job->sp->id = syncpt_id;
	job->sp->incrs = 1;
	job->num_syncpts = 1;

	/* push increment after work has been completed */
	se_nvhost_write_method(&cpuvaddr[num_words],
		nvhost_opcode_nonincr(
			host1x_uclass_incr_syncpt_r(), 1),
		nvhost_class_host_incr_syncpt(
			host1x_uclass_incr_syncpt_cond_op_done_v(),
			syncpt_id), &num_words);

	err = nvhost_job_add_client_gather_address(job, num_words,
				NV_SE1_CLASS_ID + se_dev->se_dev_num, iova);
	if (err) {
		dev_err(se_dev->dev, "Nvhost failed to add gather\n");
		goto exit;
	}

	err = nvhost_channel_submit(job);
	if (err) {
		dev_err(se_dev->dev, "Nvhost submit failed\n");
		goto exit;
	}

	/* wait until host1x has processed work */
	nvhost_syncpt_wait_timeout_ext(se_dev->pdev, job->sp->id,
		job->sp->fence,
		(u32)MAX_SCHEDULE_TIMEOUT,
		NULL, NULL);
exit:
	nvhost_job_put(job);
	job = NULL;
	return err;
}

static int tegra_se_send_ctr_seed(struct tegra_se_dev *se_dev, u32 *pdata,
	unsigned int opcode_addr, int *cmdbuf_cnt)
{
	u32 j;
	int err = 0;
	u32 cmdbuf_num_words = 0, i = 0;

	se_dev->aes_cmdbuf_cpuvaddr[i++] =
		__nvhost_opcode_incr(opcode_addr + 0x18, 4);

	for (j = 0; j < SE_CRYPTO_CTR_REG_COUNT; j++)
		se_dev->aes_cmdbuf_cpuvaddr[i++] = pdata[j];

	cmdbuf_num_words = i;
	*cmdbuf_cnt = i;
	return err;
}

static int tegra_se_send_key_data(struct tegra_se_dev *se_dev,
			u8 *pdata, u32 data_len, u8 slot_num,
		enum tegra_se_key_table_type type, unsigned int opcode_addr,
		int *cmdbuf_cnt)
{
	u32 data_size;
	u32 *pdata_buf = (u32 *)pdata;
	u8 pkt = 0, quad = 0;
	u32 val = 0, j;
	u32 cmdbuf_num_words = 0, i = 0;
	int err = 0;

	if (pdata_buf == NULL)
		return -ENOMEM;

	if ((type == SE_KEY_TABLE_TYPE_KEY) && (slot_num == ssk_slot.slot_num)) {
		return -EINVAL;
	}

	if (type == SE_KEY_TABLE_TYPE_ORGIV)
		quad = QUAD_ORG_IV;
	else if (type == SE_KEY_TABLE_TYPE_UPDTDIV)
		quad = QUAD_UPDTD_IV;
	else
		quad = QUAD_KEYS_128;

	se_dev->aes_cmdbuf_cpuvaddr[i++] =
		__nvhost_opcode_nonincr(opcode_addr + 0x34, 1);
	se_dev->aes_cmdbuf_cpuvaddr[i++] = SE_OPERATION_WRSTALL(WRSTALL_TRUE) |
						SE_OPERATION_OP(OP_DUMMY);

	data_size = SE_KEYTABLE_QUAD_SIZE_BYTES;

	do {
		pkt = SE_KEYTABLE_SLOT(slot_num) | SE_KEYTABLE_QUAD(quad);
		for (j = 0; j < data_size; j += 4, data_len -= 4) {
			se_dev->aes_cmdbuf_cpuvaddr[i++] =
				__nvhost_opcode_nonincr(opcode_addr + 0xB8, 1);
			val = (SE_KEYTABLE_PKT(pkt) | (j / 4));
			se_dev->aes_cmdbuf_cpuvaddr[i++] = val;

			se_dev->aes_cmdbuf_cpuvaddr[i++] =
				__nvhost_opcode_incr(opcode_addr + 0xBC, 1);
			se_dev->aes_cmdbuf_cpuvaddr[i++] = *pdata_buf++;
		}
		data_size = data_len;
		quad = QUAD_KEYS_256;
	} while (data_len);

	se_dev->aes_cmdbuf_cpuvaddr[i++] =
		__nvhost_opcode_nonincr(opcode_addr + 0x34, 1);
	se_dev->aes_cmdbuf_cpuvaddr[i++] = SE_OPERATION_OP(OP_DUMMY);

	cmdbuf_num_words = i;
	*cmdbuf_cnt = i;

	if (type == SE_KEY_TABLE_TYPE_KEY) {
			err = tegra_se_channel_submit_gather(se_dev, NULL,
				se_dev->aes_cmdbuf_cpuvaddr,
				se_dev->aes_cmdbuf_iova, 0, cmdbuf_num_words);
	}

	return err;
}

static u32 tegra_se_get_crypto_config(struct tegra_se_dev *se_dev,
	enum tegra_se_aes_op_mode mode, bool encrypt, u8 slot_num, bool org_iv)
{
	u32 val = 0;
	unsigned long freq = 0;

	switch (mode) {
	case SE_AES_OP_MODE_CMAC:
	case SE_AES_OP_MODE_CBC:
		if (encrypt) {
			val = SE_CRYPTO_INPUT_SEL(INPUT_MEMORY) |
				SE_CRYPTO_VCTRAM_SEL(VCTRAM_AESOUT) |
				SE_CRYPTO_XOR_POS(XOR_TOP) |
				SE_CRYPTO_CORE_SEL(CORE_ENCRYPT);
		} else {
			val = SE_CRYPTO_INPUT_SEL(INPUT_MEMORY) |
				SE_CRYPTO_VCTRAM_SEL(VCTRAM_PREVAHB) |
				SE_CRYPTO_XOR_POS(XOR_BOTTOM) |
				SE_CRYPTO_CORE_SEL(CORE_DECRYPT);
		}
		freq = se_dev->chipdata->aes_freq;
		break;
	case SE_AES_OP_MODE_RNG_DRBG:
		val = SE_CRYPTO_INPUT_SEL(INPUT_RANDOM) |
			SE_CRYPTO_XOR_POS(XOR_BYPASS) |
			SE_CRYPTO_CORE_SEL(CORE_ENCRYPT);
		if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA11)
			val = val | SE_CRYPTO_KEY_INDEX(slot_num);
		break;
	case SE_AES_OP_MODE_ECB:
		if (encrypt) {
			val = SE_CRYPTO_INPUT_SEL(INPUT_MEMORY) |
				SE_CRYPTO_XOR_POS(XOR_BYPASS) |
				SE_CRYPTO_CORE_SEL(CORE_ENCRYPT);
		} else {
			val = SE_CRYPTO_INPUT_SEL(INPUT_MEMORY) |
				SE_CRYPTO_XOR_POS(XOR_BYPASS) |
				SE_CRYPTO_CORE_SEL(CORE_DECRYPT);
		}
		freq = se_dev->chipdata->aes_freq;
		break;
	case SE_AES_OP_MODE_CTR:
		val = SE_CRYPTO_INPUT_SEL(INPUT_LNR_CTR) |
			SE_CRYPTO_VCTRAM_SEL(VCTRAM_MEMORY) |
			SE_CRYPTO_XOR_POS(XOR_BOTTOM) |
			SE_CRYPTO_CORE_SEL(CORE_ENCRYPT);
		freq = se_dev->chipdata->aes_freq;
		break;
	case SE_AES_OP_MODE_OFB:
		val = SE_CRYPTO_INPUT_SEL(INPUT_AESOUT) |
			SE_CRYPTO_VCTRAM_SEL(VCTRAM_MEMORY) |
			SE_CRYPTO_XOR_POS(XOR_BOTTOM) |
			SE_CRYPTO_CORE_SEL(CORE_ENCRYPT);
		freq = se_dev->chipdata->aes_freq;
		break;
	default:
		dev_warn(se_dev->dev, "Invalid operation mode\n");
		break;
	}

	if (mode == SE_AES_OP_MODE_CTR) {
		val |= SE_CRYPTO_HASH(HASH_DISABLE) |
			SE_CRYPTO_KEY_INDEX(slot_num) |
			SE_CRYPTO_CTR_CNTN(1);
	} else {
		val |= SE_CRYPTO_HASH(HASH_DISABLE) |
			SE_CRYPTO_KEY_INDEX(slot_num) |
			(org_iv ? SE_CRYPTO_IV_SEL(IV_ORIGINAL) :
			SE_CRYPTO_IV_SEL(IV_UPDATED));
	}

	/* enable hash for CMAC */
	if (mode == SE_AES_OP_MODE_CMAC)
		val |= SE_CRYPTO_HASH(HASH_ENABLE);

	if (mode == SE_AES_OP_MODE_RNG_DRBG) {
		/* Make sure engine is powered ON*/
		nvhost_module_busy(se_dev->pdev);

		if (force_reseed_count <= 0) {
			se_writel(se_dev,
				SE_RNG_CONFIG_MODE(DRBG_MODE_FORCE_RESEED)|
				SE_RNG_CONFIG_SRC(DRBG_SRC_ENTROPY),
				SE_RNG_CONFIG_REG_OFFSET);
		force_reseed_count = RNG_RESEED_INTERVAL;
		} else {
			se_writel(se_dev,
				SE_RNG_CONFIG_MODE(DRBG_MODE_NORMAL)|
				SE_RNG_CONFIG_SRC(DRBG_SRC_ENTROPY),
				SE_RNG_CONFIG_REG_OFFSET);
		}
		--force_reseed_count;

		se_writel(se_dev, RNG_RESEED_INTERVAL,
			SE_RNG_RESEED_INTERVAL_REG_OFFSET);

		/* Power off device after register access done */
		nvhost_module_idle(se_dev->pdev);
	}
	return val;
}

static int tegra_se_send_sha_data(struct tegra_se_dev *se_dev,
			struct tegra_se_req_context *req_ctx, u32 count)
{
	int j, k;
	int err = 0;
	u32 cmdbuf_num_words = 0, i = 0;
	u32 *cmdbuf_cpuvaddr = NULL;
	dma_addr_t cmdbuf_iova = 0;

	sha_opcode_start_addr = SE4_SHA_CONFIG_REG_OFFSET;

	cmdbuf_cpuvaddr = dma_alloc_attrs(se_dev->dev->parent, SZ_4K,
				 &cmdbuf_iova, GFP_KERNEL, &attrs);
	if (!cmdbuf_cpuvaddr)
		return -ENOMEM;

	cmdbuf_cpuvaddr[i++] = __nvhost_opcode_incr(sha_opcode_start_addr, 4);
	cmdbuf_cpuvaddr[i++] = req_ctx->config;
	cmdbuf_cpuvaddr[i++] = SE4_HW_INIT_HASH(HW_INIT_HASH_ENABLE);
	cmdbuf_cpuvaddr[i++] = se_dev->src_ll->addr;
	cmdbuf_cpuvaddr[i++] = (u32)(SE_ADDR_HI_MSB(MSB(se_dev->src_ll->addr)) |
			SE_ADDR_HI_SZ(se_dev->src_ll->data_len)); /* in-hi */

	cmdbuf_cpuvaddr[i++] =
		__nvhost_opcode_incr(sha_opcode_start_addr + 0x18, 8);

	/* Repeat for SHA_MSG_LENGTH & SHA_MSG_LEFT */
	for (k = 0; k < 2; k++) {
		for (j = 0; j < 4; j++) {
			if (!j)
				cmdbuf_cpuvaddr[i++] = (count * 8);
			else
				cmdbuf_cpuvaddr[i++] = 0;
		}
	}
	cmdbuf_cpuvaddr[i++] =
		__nvhost_opcode_nonincr(sha_opcode_start_addr + 0x78, 1);
	cmdbuf_cpuvaddr[i++] = SE_OPERATION_LASTBUF(LASTBUF_TRUE) |
				SE_OPERATION_OP(OP_START);

	cmdbuf_num_words = i;
	err = tegra_se_channel_submit_gather(se_dev, NULL,
			cmdbuf_cpuvaddr, cmdbuf_iova,
			0, cmdbuf_num_words);
	dma_free_attrs(se_dev->dev->parent, SZ_4K,
			cmdbuf_cpuvaddr, cmdbuf_iova, &attrs);
	return err;
}

static void tegra_se_read_cmac_result(struct tegra_se_dev *se_dev,
	u8 *pdata, u32 nbytes, bool swap32)
{
	u32 *result = (u32 *)pdata;
	u32 i;

	/* Make SE engine is powered ON */
	nvhost_module_busy(se_dev->pdev);

	for (i = 0; i < nbytes/4; i++) {
		result[i] = se_readl(se_dev, SE_CMAC_RESULT_REG_OFFSET +
				(i * sizeof(u32)));
		if (swap32)
			result[i] = be32_to_cpu(result[i]);
	}

	nvhost_module_idle(se_dev->pdev);
}


static void tegra_se_read_hash_result(struct tegra_se_dev *se_dev,
	u8 *pdata, u32 nbytes, bool swap32)
{
	u32 *result = (u32 *)pdata;
	u32 i;

	/* Make SE engine is powered ON */
	nvhost_module_busy(se_dev->pdev);

	for (i = 0; i < nbytes/4; i++) {
		result[i] = se_readl(se_dev, SE_HASH_RESULT_REG_OFFSET +
				(i * sizeof(u32)));
		if (swap32)
			result[i] = be32_to_cpu(result[i]);
	}

	nvhost_module_idle(se_dev->pdev);
}

static int tegra_se_send_data(struct tegra_se_dev *se_dev,
	struct tegra_se_req_context *req_ctx, struct ablkcipher_request *req,
		u32 nbytes, unsigned int opcode_addr, int cmdbuf_cnt)
{
	u32 cmdbuf_num_words = 0, i = 0;
	int err = 0;
	u32 total;
	int restart_op;
	struct tegra_se_ll *src_ll = se_dev->src_ll;
	struct tegra_se_ll *dst_ll = se_dev->dst_ll;

	i = cmdbuf_cnt;
	total = nbytes;
	/* Create Gather Buffer Command */

	se_dev->aes_cmdbuf_cpuvaddr[i++] =
		__nvhost_opcode_nonincr(opcode_addr + 0x34, 1);
	se_dev->aes_cmdbuf_cpuvaddr[i++] = SE_OPERATION_WRSTALL(WRSTALL_TRUE) |
					SE_OPERATION_OP(OP_DUMMY);

	while (total) {
		if (total == nbytes) {
			se_dev->aes_cmdbuf_cpuvaddr[i++] =
				__nvhost_opcode_nonincr(opcode_addr + 0x28, 1);
			se_dev->aes_cmdbuf_cpuvaddr[i++] = ((nbytes/16) - 1);

			se_dev->aes_cmdbuf_cpuvaddr[i++] =
				__nvhost_opcode_incr(opcode_addr, 6);
			se_dev->aes_cmdbuf_cpuvaddr[i++] = req_ctx->config;
			se_dev->aes_cmdbuf_cpuvaddr[i++] = req_ctx->crypto_config;
		} else {
			se_dev->aes_cmdbuf_cpuvaddr[i++] =
			__nvhost_opcode_incr(opcode_addr + 8, 4);
		}

		se_dev->aes_cmdbuf_cpuvaddr[i++] = (u32)(src_ll->addr);
		se_dev->aes_cmdbuf_cpuvaddr[i++] =
			(u32)(SE_ADDR_HI_MSB(MSB(src_ll->addr))
				| SE_ADDR_HI_SZ(src_ll->data_len));
		se_dev->aes_cmdbuf_cpuvaddr[i++] = (u32)(dst_ll->addr);
		se_dev->aes_cmdbuf_cpuvaddr[i++] =
			(u32)(SE_ADDR_HI_MSB(MSB(dst_ll->addr))
				| SE_ADDR_HI_SZ(dst_ll->data_len));

		if (req_ctx->op_mode == SE_AES_OP_MODE_CMAC)
			restart_op = OP_RESTART_IN;
		else if (req_ctx->op_mode == SE_AES_OP_MODE_RNG_DRBG)
			restart_op = OP_RESTART_OUT;
		else
			restart_op = OP_RESTART_INOUT;

		se_dev->aes_cmdbuf_cpuvaddr[i++] =
			__nvhost_opcode_nonincr(opcode_addr + 0x34, 1);

		if (total == nbytes) {
			if (total == src_ll->data_len)
				se_dev->aes_cmdbuf_cpuvaddr[i++] =
					SE_OPERATION_LASTBUF(LASTBUF_TRUE)
					| SE_OPERATION_OP(OP_START);
			else
				se_dev->aes_cmdbuf_cpuvaddr[i++] =
					SE_OPERATION_LASTBUF(LASTBUF_FALSE)
					| SE_OPERATION_OP(OP_START);
		} else {
			if (total == src_ll->data_len)
				se_dev->aes_cmdbuf_cpuvaddr[i++] =
					SE_OPERATION_LASTBUF(LASTBUF_TRUE)
					| SE_OPERATION_OP(restart_op);
			else
				se_dev->aes_cmdbuf_cpuvaddr[i++] =
					SE_OPERATION_LASTBUF(LASTBUF_FALSE)
					| SE_OPERATION_OP(restart_op);
		}

		total -= src_ll->data_len;
		src_ll++;
		dst_ll++;
	}

	cmdbuf_num_words = i;
	err = tegra_se_channel_submit_gather(se_dev, req,
			se_dev->aes_cmdbuf_cpuvaddr, se_dev->aes_cmdbuf_iova,
			0, cmdbuf_num_words);

	return err;
}

static int tegra_map_sg(struct device *dev, struct scatterlist *sg,
			unsigned int nents, enum dma_data_direction dir,
			struct tegra_se_ll *se_ll, u32 total)
{
	u32 total_loop = 0;

	total_loop = total;
	while (sg) {
		dma_map_sg(dev, sg, 1, dir);
		se_ll->addr = sg_dma_address(sg);
		se_ll->data_len = min(sg->length, total_loop);
		total_loop -= min(sg->length, total_loop);
		sg = scatterwalk_sg_next(sg);
		se_ll++;
	}
	return nents;
}

static void tegra_unmap_sg(struct device *dev, struct scatterlist *sg,
				enum dma_data_direction dir, u32 total)
{
	while (sg) {
		dma_unmap_sg(dev, sg, 1, dir);
			sg = scatterwalk_sg_next(sg);
	}
}

static int tegra_se_count_sgs(struct scatterlist *sl, u32 nbytes, int *chained)
{
	struct scatterlist *sg = sl;
	int sg_nents = 0;

	*chained = 0;
	while (sg) {
		sg_nents++;
		nbytes -= min(sl->length, nbytes);
		if (!sg_is_last(sg) && (sg + 1)->length == 0)
			*chained = 1;
		sg = scatterwalk_sg_next(sg);
	}

	return sg_nents;
}

static int tegra_se_setup_ablk_req(struct tegra_se_dev *se_dev,
	struct ablkcipher_request *req)
{
	struct scatterlist *src_sg, *dst_sg;
	u32 total, num_src_sgs, num_dst_sgs;
	int src_chained, dst_chained;
	int ret = 0;

	num_src_sgs = tegra_se_count_sgs(req->src, req->nbytes, &src_chained);
	num_dst_sgs = tegra_se_count_sgs(req->dst, req->nbytes, &dst_chained);

	if ((num_src_sgs > SE_MAX_SRC_SG_COUNT) ||
		(num_dst_sgs > SE_MAX_DST_SG_COUNT)) {
			dev_err(se_dev->dev, "num of SG buffers are more\n");
			return -EINVAL;
	}

	*se_dev->src_ll_buf = num_src_sgs-1;
	*se_dev->dst_ll_buf = num_dst_sgs-1;

	se_dev->src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);
	se_dev->dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf + 1);

	src_sg = req->src;
	dst_sg = req->dst;
	total = req->nbytes;

	if (total) {
		tegra_map_sg(se_dev->dev, src_sg, 1,
			DMA_TO_DEVICE, se_dev->src_ll, total);
		tegra_map_sg(se_dev->dev, dst_sg, 1,
			DMA_FROM_DEVICE, se_dev->dst_ll, total);
		WARN_ON(src_sg->length != dst_sg->length);
	}
	return ret;
}

static void tegra_se_dequeue_complete_req(struct tegra_se_dev *se_dev,
	struct ablkcipher_request *req)
{
	struct scatterlist *src_sg, *dst_sg;
	u32 total;

	if (req) {
		src_sg = req->src;
		dst_sg = req->dst;
		total = req->nbytes;
		tegra_unmap_sg(se_dev->dev, dst_sg,  DMA_FROM_DEVICE, total);
		tegra_unmap_sg(se_dev->dev, src_sg,  DMA_TO_DEVICE, total);
	}
}

static void tegra_se_process_new_req(struct crypto_async_request *async_req)
{
	struct tegra_se_dev *se_dev;
	struct ablkcipher_request *req = ablkcipher_request_cast(async_req);
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);
	struct tegra_se_aes_context *aes_ctx =
			crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	int ret = 0;
	unsigned int opcode_addr;
	int cmdbuf_cnt = 0;

	se_dev = req_ctx->se_dev;
	mutex_lock(&se_dev->mtx);

	opcode_addr = SE2_AES1_CONFIG_REG_OFFSET;

	/* write IV */
	if (req->info) {
		if (req_ctx->op_mode == SE_AES_OP_MODE_CTR) {
			ret = tegra_se_send_ctr_seed(se_dev, (u32 *)req->info,
					opcode_addr, &cmdbuf_cnt);
		} else {
			ret = tegra_se_send_key_data(se_dev, req->info,
				TEGRA_SE_AES_IV_SIZE, aes_ctx->slot->slot_num,
			SE_KEY_TABLE_TYPE_ORGIV, opcode_addr, &cmdbuf_cnt);
		}
	}

	tegra_se_setup_ablk_req(se_dev, req);
	req_ctx->config = tegra_se_get_config(se_dev, req_ctx->op_mode,
				req_ctx->encrypt, aes_ctx->keylen);
	req_ctx->crypto_config = tegra_se_get_crypto_config(se_dev,
		req_ctx->op_mode, req_ctx->encrypt, aes_ctx->slot->slot_num,
			req->info ? true : false);
	ret = tegra_se_send_data(se_dev, req_ctx, req, req->nbytes,
				opcode_addr, cmdbuf_cnt);

	tegra_se_dequeue_complete_req(se_dev, req);
	mutex_unlock(&se_dev->mtx);
	if (req)
		tegra_se_complete_callback(req, 1);
}

static void tegra_se_work_handler(struct work_struct *work)
{
	struct tegra_se_dev *se_dev = container_of(work,
					struct tegra_se_dev, se_work);
	struct crypto_async_request *async_req = NULL;
	struct crypto_async_request *backlog = NULL;
	unsigned long flags;

	do {
		spin_lock_irqsave(&se_dev->lock, flags);
		backlog = crypto_get_backlog(&se_dev->queue);
		async_req = crypto_dequeue_request(&se_dev->queue);
		if (!async_req)
			se_dev->work_q_busy = false;
		spin_unlock_irqrestore(&se_dev->lock, flags);

		if (backlog) {
			backlog->complete(backlog, -EINPROGRESS);
			backlog = NULL;
		}

		if (async_req) {
			tegra_se_process_new_req(async_req);
			async_req = NULL;
		}
	} while (se_dev->work_q_busy);
}

static int tegra_se_aes_queue_req(struct tegra_se_dev *se_dev,
				struct ablkcipher_request *req)
{
	unsigned long flags;
	bool idle = true;
	int err = 0;
	int chained;

	if (!tegra_se_count_sgs(req->src, req->nbytes, &chained))
		return -EINVAL;

	spin_lock_irqsave(&se_dev->lock, flags);
	err = ablkcipher_enqueue_request(&se_dev->queue, req);
	if (se_dev->work_q_busy)
		idle = false;
	spin_unlock_irqrestore(&se_dev->lock, flags);

	if (idle) {
		spin_lock_irqsave(&se_dev->lock, flags);
		se_dev->work_q_busy = true;
		spin_unlock_irqrestore(&se_dev->lock, flags);
		queue_work(se_dev->se_work_q, &se_dev->se_work);
	}

	return err;
}

static int tegra_se_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->encrypt = true;
	req_ctx->op_mode = SE_AES_OP_MODE_CBC;
	req_ctx->se_dev = sg_tegra_se_dev[1];
	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->encrypt = false;
	req_ctx->op_mode = SE_AES_OP_MODE_CBC;
	req_ctx->se_dev = sg_tegra_se_dev[1];
	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_ecb_encrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->encrypt = true;
	req_ctx->op_mode = SE_AES_OP_MODE_ECB;
	req_ctx->se_dev = sg_tegra_se_dev[1];
	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_ecb_decrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->encrypt = false;
	req_ctx->op_mode = SE_AES_OP_MODE_ECB;
	req_ctx->se_dev = sg_tegra_se_dev[1];
	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_ctr_encrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->encrypt = true;
	req_ctx->op_mode = SE_AES_OP_MODE_CTR;
	req_ctx->se_dev = sg_tegra_se_dev[1];
	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_ctr_decrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->encrypt = false;
	req_ctx->op_mode = SE_AES_OP_MODE_CTR;
	req_ctx->se_dev = sg_tegra_se_dev[1];
	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_ofb_encrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->encrypt = true;
	req_ctx->op_mode = SE_AES_OP_MODE_OFB;
	req_ctx->se_dev = sg_tegra_se_dev[1];
	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_ofb_decrypt(struct ablkcipher_request *req)
{
	struct tegra_se_req_context *req_ctx = ablkcipher_request_ctx(req);

	req_ctx->encrypt = false;
	req_ctx->op_mode = SE_AES_OP_MODE_OFB;
	req_ctx->se_dev = sg_tegra_se_dev[1];
	return tegra_se_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_se_aes_setkey(struct crypto_ablkcipher *tfm,
	const u8 *key, u32 keylen)
{
	struct tegra_se_aes_context *ctx = crypto_ablkcipher_ctx(tfm);
	struct tegra_se_dev *se_dev = NULL;
	struct tegra_se_slot *pslot;
	u8 *pdata = (u8 *)key;
	int ret = 0;
	int cmdbuf_cnt;

	se_dev = sg_tegra_se_dev[1];
	if (!ctx || !se_dev) {
		pr_err("invalid context or dev");
		return -EINVAL;
	}
	ctx->se_dev = se_dev;

	if ((keylen != TEGRA_SE_KEY_128_SIZE) &&
		(keylen != TEGRA_SE_KEY_192_SIZE) &&
		(keylen != TEGRA_SE_KEY_256_SIZE)) {
		dev_err(se_dev->dev, "invalid key size");
		return -EINVAL;
	}

	mutex_lock(&se_dev->mtx);
	if (key) {
		if (!ctx->slot || (ctx->slot &&
		    ctx->slot->slot_num == ssk_slot.slot_num)) {
			pslot = tegra_se_alloc_key_slot();
			if (!pslot) {
				dev_err(se_dev->dev, "no free key slot\n");
				mutex_unlock(&se_dev->mtx);
				return -ENOMEM;
			}
			ctx->slot = pslot;
		}
		ctx->keylen = keylen;
	} else {
		tegra_se_free_key_slot(ctx->slot);
		ctx->slot = &ssk_slot;
		ctx->keylen = AES_KEYSIZE_128;
	}
	/* load the key */
	ret = tegra_se_send_key_data(se_dev, pdata, keylen, ctx->slot->slot_num,
		SE_KEY_TABLE_TYPE_KEY, SE2_AES1_CONFIG_REG_OFFSET, &cmdbuf_cnt);
	mutex_unlock(&se_dev->mtx);
	return 0;
}

static int tegra_se_aes_cra_init(struct crypto_tfm *tfm)
{
	tfm->crt_ablkcipher.reqsize = sizeof(struct tegra_se_req_context);

	return 0;
}

static void tegra_se_aes_cra_exit(struct crypto_tfm *tfm)
{
	struct tegra_se_aes_context *ctx = crypto_tfm_ctx(tfm);

	tegra_se_free_key_slot(ctx->slot);
	ctx->slot = NULL;
}

static int tegra_se_rng_drbg_init(struct crypto_tfm *tfm)
{
	struct tegra_se_rng_context *rng_ctx = crypto_tfm_ctx(tfm);
	struct tegra_se_dev *se_dev = sg_tegra_se_dev[0];
	mutex_lock(&se_dev->mtx);

	rng_ctx->se_dev = se_dev;
	rng_ctx->dt_buf = dma_alloc_coherent(se_dev->dev, TEGRA_SE_RNG_DT_SIZE,
		&rng_ctx->dt_buf_adr, GFP_KERNEL);
	if (!rng_ctx->dt_buf) {
		dev_err(se_dev->dev, "can not allocate rng dma buffer");
		mutex_unlock(&se_dev->mtx);
		return -ENOMEM;
	}

	rng_ctx->rng_buf = dma_alloc_coherent(rng_ctx->se_dev->dev,
		TEGRA_SE_RNG_DT_SIZE, &rng_ctx->rng_buf_adr, GFP_KERNEL);
	if (!rng_ctx->rng_buf) {
		dev_err(se_dev->dev, "can not allocate rng dma buffer");
		dma_free_coherent(rng_ctx->se_dev->dev, TEGRA_SE_RNG_DT_SIZE,
					rng_ctx->dt_buf, rng_ctx->dt_buf_adr);
		mutex_unlock(&se_dev->mtx);
		return -ENOMEM;
	}
	mutex_unlock(&se_dev->mtx);

	return 0;
}

static int tegra_se_rng_drbg_get_random(struct crypto_rng *tfm,
	u8 *rdata, u32 dlen)
{
	struct tegra_se_rng_context *rng_ctx = crypto_rng_ctx(tfm);
	struct tegra_se_dev *se_dev = rng_ctx->se_dev;
	u8 *rdata_addr;
	int ret = 0, j, num_blocks, data_len = 0;

	struct tegra_se_req_context *req_ctx =
		kzalloc(sizeof(struct tegra_se_req_context), GFP_KERNEL);
	if (!req_ctx) {
		dev_err(se_dev->dev,
			"memory allocation failed for drbg req_ctx\n");
		return -ENOMEM;
	}

	num_blocks = (dlen / TEGRA_SE_RNG_DT_SIZE);
	data_len = (dlen % TEGRA_SE_RNG_DT_SIZE);
	if (data_len == 0)
		num_blocks = num_blocks - 1;

	mutex_lock(&se_dev->mtx);
	req_ctx->op_mode = SE_AES_OP_MODE_RNG_DRBG;

	*se_dev->src_ll_buf = 0;
	*se_dev->dst_ll_buf = 0;
	se_dev->src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);
	se_dev->dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf + 1);

	req_ctx->config = tegra_se_get_config(se_dev, req_ctx->op_mode, true,
		TEGRA_SE_KEY_128_SIZE);
	req_ctx->crypto_config = tegra_se_get_crypto_config(se_dev,
			req_ctx->op_mode, true, 0, true);

	for (j = 0; j <= num_blocks; j++) {
		se_dev->src_ll->addr = rng_ctx->dt_buf_adr;
		se_dev->src_ll->data_len = TEGRA_SE_RNG_DT_SIZE;
		se_dev->dst_ll->addr = rng_ctx->rng_buf_adr;
		se_dev->dst_ll->data_len = TEGRA_SE_RNG_DT_SIZE;

		ret = tegra_se_send_data(se_dev,
			req_ctx, NULL, TEGRA_SE_RNG_DT_SIZE,
			SE1_AES0_CONFIG_REG_OFFSET, 0);
		if (!ret) {
			rdata_addr = (rdata + (j * TEGRA_SE_RNG_DT_SIZE));

			if (data_len && num_blocks == j) {
				memcpy(rdata_addr, rng_ctx->rng_buf, data_len);
			} else {
				memcpy(rdata_addr,
					rng_ctx->rng_buf, TEGRA_SE_RNG_DT_SIZE);
			}
		} else {
			dlen = 0;
		}
	}

	mutex_unlock(&se_dev->mtx);
	kfree(req_ctx);
	return dlen;
}

static int tegra_se_rng_drbg_reset(struct crypto_rng *tfm, u8 *seed, u32 slen)
{
	return 0;
}

static void tegra_se_rng_drbg_exit(struct crypto_tfm *tfm)
{
	struct tegra_se_rng_context *rng_ctx = crypto_tfm_ctx(tfm);

	if (rng_ctx->dt_buf) {
		dma_free_coherent(rng_ctx->se_dev->dev, TEGRA_SE_RNG_DT_SIZE,
			rng_ctx->dt_buf, rng_ctx->dt_buf_adr);
	}

	if (rng_ctx->rng_buf) {
		dma_free_coherent(rng_ctx->se_dev->dev, TEGRA_SE_RNG_DT_SIZE,
			rng_ctx->rng_buf, rng_ctx->rng_buf_adr);
	}
	rng_ctx->se_dev = NULL;
}

static int tegra_se_sha_init(struct ahash_request *req)
{
	return 0;
}

static int tegra_se_sha_update(struct ahash_request *req)
{
	return 0;
}

static int tegra_se_sha_finup(struct ahash_request *req)
{
	return 0;
}

static int tegra_se_sha_final(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct tegra_se_sha_context *sha_ctx = crypto_ahash_ctx(tfm);
	struct tegra_se_req_context *req_ctx = ahash_request_ctx(req);
	struct tegra_se_dev *se_dev;
	struct scatterlist *src_sg;
	u32 total, num_sgs;
	int err = 0;
	int chained;

	if (!req->nbytes)
		return -EINVAL;

	if (crypto_ahash_digestsize(tfm) == SHA1_DIGEST_SIZE)
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA1;

	if (crypto_ahash_digestsize(tfm) == SHA224_DIGEST_SIZE)
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA224;

	if (crypto_ahash_digestsize(tfm) == SHA256_DIGEST_SIZE)
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA256;

	if (crypto_ahash_digestsize(tfm) == SHA384_DIGEST_SIZE)
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA384;

	if (crypto_ahash_digestsize(tfm) == SHA512_DIGEST_SIZE)
		sha_ctx->op_mode = SE_AES_OP_MODE_SHA512;

	se_dev = sg_tegra_se_dev[3];

	num_sgs = tegra_se_count_sgs(req->src, req->nbytes, &chained);
	if ((num_sgs > SE_MAX_SRC_SG_COUNT)) {
		dev_err(se_dev->dev, "num of SG buffers are more\n");
		return -EINVAL;
	}

	*se_dev->src_ll_buf = num_sgs - 1;
	se_dev->src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);

	src_sg = req->src;
	total = req->nbytes;
	if (total)
		tegra_map_sg(se_dev->dev,
			src_sg, 1, DMA_TO_DEVICE, se_dev->src_ll, total);

	req_ctx->config = tegra_se_get_config(se_dev,
					sha_ctx->op_mode, false, 0);
	err = tegra_se_send_sha_data(se_dev, req_ctx, req->nbytes);
	if (err)
		goto sha_fail;

	tegra_se_read_hash_result(se_dev, req->result,
				crypto_ahash_digestsize(tfm), true);

	if ((sha_ctx->op_mode == SE_AES_OP_MODE_SHA384) ||
		(sha_ctx->op_mode == SE_AES_OP_MODE_SHA512)) {
		u32 *result = (u32 *)req->result;
		u32 temp, i;

		for (i = 0; i < crypto_ahash_digestsize(tfm)/4;
			i += 2) {
			temp = result[i];
			result[i] = result[i+1];
			result[i+1] = temp;
		}
	}
sha_fail:
	src_sg = req->src;
	total = req->nbytes;
	if (total)
		tegra_unmap_sg(se_dev->dev, src_sg, DMA_TO_DEVICE, total);

	return err;
}

static int tegra_se_sha_digest(struct ahash_request *req)
{
	return tegra_se_sha_init(req) ?: tegra_se_sha_final(req);
}

static int tegra_se_sha_cra_init(struct crypto_tfm *tfm)
{
	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct tegra_se_sha_context));
	return 0;
}

static void tegra_se_sha_cra_exit(struct crypto_tfm *tfm)
{
	/* do nothing */
}

static int tegra_se_aes_cmac_init(struct ahash_request *req)
{

	return 0;
}

static int tegra_se_aes_cmac_update(struct ahash_request *req)
{
	return 0;
}

static int tegra_se_aes_cmac_final(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct tegra_se_aes_cmac_context *cmac_ctx = crypto_ahash_ctx(tfm);
	struct tegra_se_req_context *req_ctx = ahash_request_ctx(req);
	struct tegra_se_dev *se_dev;
	struct scatterlist *src_sg;
	struct sg_mapping_iter miter;
	u32 num_sgs, blocks_to_process, last_block_bytes = 0, bytes_to_copy = 0;
	u8 piv[TEGRA_SE_AES_IV_SIZE];
	int total, ret = 0, i = 0;
	bool padding_needed = false;
	unsigned long flags;
	unsigned int sg_flags = SG_MITER_ATOMIC;
	u8 *temp_buffer = NULL;
	bool use_orig_iv = true;
	int chained;
	int cmdbuf_cnt;

	se_dev = sg_tegra_se_dev[1];
	req_ctx->op_mode = SE_AES_OP_MODE_CMAC;
	blocks_to_process = req->nbytes / TEGRA_SE_AES_BLOCK_SIZE;
	/* num of bytes less than block size */
	if ((req->nbytes % TEGRA_SE_AES_BLOCK_SIZE) || !blocks_to_process) {
		padding_needed = true;
		last_block_bytes = req->nbytes % TEGRA_SE_AES_BLOCK_SIZE;
	} else {
		/* decrement num of blocks */
		blocks_to_process--;
		if (blocks_to_process) {
			/* there are blocks to process and find last block
				bytes */
			last_block_bytes = req->nbytes -
				(blocks_to_process * TEGRA_SE_AES_BLOCK_SIZE);
		} else {
			/* this is the last block and equal to block size */
			last_block_bytes = req->nbytes;
		}
	}

	/* first process all blocks except last block */
	if (blocks_to_process) {
		num_sgs = tegra_se_count_sgs(req->src, req->nbytes, &chained);
		if (num_sgs > SE_MAX_SRC_SG_COUNT) {
			dev_err(se_dev->dev, "num of SG buffers are more\n");
			goto out;
		}
		*se_dev->src_ll_buf = num_sgs - 1;
		se_dev->src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);

		src_sg = req->src;
		total = blocks_to_process * TEGRA_SE_AES_BLOCK_SIZE;

		tegra_map_sg(se_dev->dev,
			src_sg, 1, DMA_TO_DEVICE, se_dev->src_ll, total);

		req_ctx->config = tegra_se_get_config(se_dev, req_ctx->op_mode,
				true, cmac_ctx->keylen);
		/* write zero IV */
		memset(piv, 0, TEGRA_SE_AES_IV_SIZE);

		ret = tegra_se_send_key_data(se_dev, piv, TEGRA_SE_AES_IV_SIZE,
			cmac_ctx->slot->slot_num, SE_KEY_TABLE_TYPE_ORGIV,
				SE2_AES1_CONFIG_REG_OFFSET, &cmdbuf_cnt);

		req_ctx->crypto_config = tegra_se_get_crypto_config(se_dev,
			req_ctx->op_mode, true, cmac_ctx->slot->slot_num, true);

		ret = tegra_se_send_data(se_dev, req_ctx, NULL, total,
					SE2_AES1_CONFIG_REG_OFFSET, 0);

		tegra_se_read_cmac_result(se_dev, piv,
				TEGRA_SE_AES_CMAC_DIGEST_SIZE, false);
		src_sg = req->src;
		tegra_unmap_sg(se_dev->dev, src_sg,  DMA_TO_DEVICE, total);
		use_orig_iv = false;
	}

	/* get the last block bytes from the sg_dma buffer using miter */
	src_sg = req->src;
	num_sgs = tegra_se_count_sgs(req->src, req->nbytes, &chained);
	sg_flags |= SG_MITER_FROM_SG;
	sg_miter_start(&miter, req->src, num_sgs, sg_flags);
	local_irq_save(flags);
	total = 0;
	cmac_ctx->buffer = dma_alloc_coherent(se_dev->dev,
				TEGRA_SE_AES_BLOCK_SIZE,
				&cmac_ctx->dma_addr, GFP_KERNEL);
	if (!cmac_ctx->buffer)
		goto out;

	temp_buffer = cmac_ctx->buffer;
	while (sg_miter_next(&miter) && total < req->nbytes) {
		unsigned int len;
		len = min(miter.length, (size_t)(req->nbytes - total));
		if ((req->nbytes - (total + len)) <= last_block_bytes) {
			bytes_to_copy =
				last_block_bytes -
				(req->nbytes - (total + len));
			memcpy(temp_buffer, miter.addr + (len - bytes_to_copy),
				bytes_to_copy);
			last_block_bytes -= bytes_to_copy;
			temp_buffer += bytes_to_copy;
		}
		total += len;
	}
	sg_miter_stop(&miter);
	local_irq_restore(flags);

	/* process last block */
	if (padding_needed) {
		/* pad with 0x80, 0, 0 ... */
		last_block_bytes = req->nbytes % TEGRA_SE_AES_BLOCK_SIZE;
		cmac_ctx->buffer[last_block_bytes] = 0x80;
		for (i = last_block_bytes+1; i < TEGRA_SE_AES_BLOCK_SIZE; i++)
			cmac_ctx->buffer[i] = 0;
		/* XOR with K2 */
		for (i = 0; i < TEGRA_SE_AES_BLOCK_SIZE; i++)
			cmac_ctx->buffer[i] ^= cmac_ctx->K2[i];
	} else {
		/* XOR with K1 */
		for (i = 0; i < TEGRA_SE_AES_BLOCK_SIZE; i++)
			cmac_ctx->buffer[i] ^= cmac_ctx->K1[i];
	}

	*se_dev->src_ll_buf = 0;
	se_dev->src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);

	se_dev->src_ll->addr = cmac_ctx->dma_addr;
	se_dev->src_ll->data_len = TEGRA_SE_AES_BLOCK_SIZE;

	if (use_orig_iv) {
		/* use zero IV, this is when num of bytes is
			less <= block size */
		memset(piv, 0, TEGRA_SE_AES_IV_SIZE);
		ret = tegra_se_send_key_data(se_dev, piv, TEGRA_SE_AES_IV_SIZE,
			cmac_ctx->slot->slot_num, SE_KEY_TABLE_TYPE_ORGIV,
				SE2_AES1_CONFIG_REG_OFFSET, &cmdbuf_cnt);
	} else {
		ret = tegra_se_send_key_data(se_dev, piv, TEGRA_SE_AES_IV_SIZE,
			cmac_ctx->slot->slot_num, SE_KEY_TABLE_TYPE_UPDTDIV,
			SE2_AES1_CONFIG_REG_OFFSET, &cmdbuf_cnt);
	}

	req_ctx->config = tegra_se_get_config(se_dev, req_ctx->op_mode,
			true, cmac_ctx->keylen);
	req_ctx->crypto_config = tegra_se_get_crypto_config(se_dev,
		req_ctx->op_mode, true, cmac_ctx->slot->slot_num, use_orig_iv);

	ret = tegra_se_send_data(se_dev,
			req_ctx, NULL, TEGRA_SE_AES_BLOCK_SIZE, SE2_AES1_CONFIG_REG_OFFSET, 0);
	tegra_se_read_cmac_result(se_dev, req->result,
				TEGRA_SE_AES_CMAC_DIGEST_SIZE, false);

out:
	if (cmac_ctx->buffer) {
		dma_free_coherent(se_dev->dev, TEGRA_SE_AES_BLOCK_SIZE,
			cmac_ctx->buffer, cmac_ctx->dma_addr);
	}

	return ret;
}

static int tegra_se_aes_cmac_setkey(struct crypto_ahash *tfm, const u8 *key,
		unsigned int keylen)
{
	struct tegra_se_aes_cmac_context *ctx = crypto_ahash_ctx(tfm);
	struct tegra_se_req_context *req_ctx = NULL;
	struct tegra_se_dev *se_dev;
	struct tegra_se_slot *pslot;
	u8 piv[TEGRA_SE_AES_IV_SIZE];
	u32 *pbuf;
	dma_addr_t pbuf_adr;
	int ret = 0;
	u8 const rb = 0x87;
	u8 msb;
	int cmdbuf_cnt;

	se_dev = sg_tegra_se_dev[1];

	if (!ctx) {
		dev_err(se_dev->dev, "invalid context");
		return -EINVAL;
	}

	req_ctx = kzalloc(sizeof(struct tegra_se_req_context), GFP_KERNEL);
	if (!req_ctx) {
		dev_err(se_dev->dev,
			"memory allocation failed for cmac req_ctx\n");
		return -ENOMEM;
	}
	if ((keylen != TEGRA_SE_KEY_128_SIZE) &&
		(keylen != TEGRA_SE_KEY_192_SIZE) &&
		(keylen != TEGRA_SE_KEY_256_SIZE)) {
		dev_err(se_dev->dev, "invalid key size");
		ret = -EINVAL;
		goto free_ctx;
	}

	if (key) {
		if (!ctx->slot || (ctx->slot &&
		    ctx->slot->slot_num == ssk_slot.slot_num)) {
			pslot = tegra_se_alloc_key_slot();
			if (!pslot) {
				dev_err(se_dev->dev, "no free key slot\n");
				ret = -ENOMEM;
				goto free_ctx;
			}
			ctx->slot = pslot;
		}
		ctx->keylen = keylen;
	} else {
		tegra_se_free_key_slot(ctx->slot);
		ctx->slot = &ssk_slot;
		ctx->keylen = AES_KEYSIZE_128;
	}

	pbuf = dma_alloc_coherent(se_dev->dev, TEGRA_SE_AES_BLOCK_SIZE,
		&pbuf_adr, GFP_KERNEL);
	if (!pbuf) {
		dev_err(se_dev->dev, "can not allocate dma buffer");
		ret = -ENOMEM;
		goto free_ctx;
	}
	memset(pbuf, 0, TEGRA_SE_AES_BLOCK_SIZE);

	*se_dev->src_ll_buf = 0;
	*se_dev->dst_ll_buf = 0;
	se_dev->src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);
	se_dev->dst_ll = (struct tegra_se_ll *)(se_dev->dst_ll_buf + 1);

	se_dev->src_ll->addr = pbuf_adr;
	se_dev->src_ll->data_len = TEGRA_SE_AES_BLOCK_SIZE;
	se_dev->dst_ll->addr = pbuf_adr;
	se_dev->dst_ll->data_len = TEGRA_SE_AES_BLOCK_SIZE;

	/* load the key */
	ret = tegra_se_send_key_data(se_dev, (u8 *)key, keylen,
			ctx->slot->slot_num, SE_KEY_TABLE_TYPE_KEY,
			SE2_AES1_CONFIG_REG_OFFSET, &cmdbuf_cnt);
	if (ret) {
		dev_err(se_dev->dev,
			"tegra_se_send_key_data for loading cmac key failed\n");
		goto out;
	}

	/* write zero IV */
	memset(piv, 0, TEGRA_SE_AES_IV_SIZE);

	/* load IV */
	ret = tegra_se_send_key_data(se_dev, piv, TEGRA_SE_AES_IV_SIZE,
			ctx->slot->slot_num, SE_KEY_TABLE_TYPE_ORGIV,
			SE2_AES1_CONFIG_REG_OFFSET, &cmdbuf_cnt);
	if (ret) {
		dev_err(se_dev->dev,
			"tegra_se_send_key_data for loading cmac iv failed\n");
		goto out;
	}

	/* config crypto algo */
	req_ctx->config = tegra_se_get_config(se_dev, SE_AES_OP_MODE_CBC,
				true, keylen);
	req_ctx->crypto_config = tegra_se_get_crypto_config(se_dev,
		SE_AES_OP_MODE_CBC, true, ctx->slot->slot_num, true);

	ret = tegra_se_send_data(se_dev,
			req_ctx, NULL, TEGRA_SE_AES_BLOCK_SIZE,
			SE2_AES1_CONFIG_REG_OFFSET, 0);
	if (ret) {
		dev_err(se_dev->dev,
			"tegra_se_aes_cmac_setkey:: start op failed\n");
		goto out;
	}

	/* compute K1 subkey */
	memcpy(ctx->K1, pbuf, TEGRA_SE_AES_BLOCK_SIZE);
	tegra_se_leftshift_onebit(ctx->K1, TEGRA_SE_AES_BLOCK_SIZE, &msb);
	if (msb)
		ctx->K1[TEGRA_SE_AES_BLOCK_SIZE - 1] ^= rb;

	/* compute K2 subkey */
	memcpy(ctx->K2, ctx->K1, TEGRA_SE_AES_BLOCK_SIZE);
	tegra_se_leftshift_onebit(ctx->K2, TEGRA_SE_AES_BLOCK_SIZE, &msb);

	if (msb)
		ctx->K2[TEGRA_SE_AES_BLOCK_SIZE - 1] ^= rb;

out:
	if (pbuf) {
		dma_free_coherent(se_dev->dev, TEGRA_SE_AES_BLOCK_SIZE,
			pbuf, pbuf_adr);
	}
free_ctx:
	kfree(req_ctx);
	return ret;
}

static int tegra_se_aes_cmac_digest(struct ahash_request *req)
{
	return tegra_se_aes_cmac_init(req) ?: tegra_se_aes_cmac_final(req);
}

static int tegra_se_aes_cmac_finup(struct ahash_request *req)
{
	return 0;
}

static int tegra_se_aes_cmac_cra_init(struct crypto_tfm *tfm)
{
	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct tegra_se_aes_cmac_context));

	return 0;
}

static void tegra_se_aes_cmac_cra_exit(struct crypto_tfm *tfm)
{
	struct tegra_se_aes_cmac_context *ctx = crypto_tfm_ctx(tfm);

	tegra_se_free_key_slot(ctx->slot);
	ctx->slot = NULL;
}

/* Security Engine rsa key slot */
struct tegra_se_rsa_slot {
	struct list_head node;
	u8 slot_num;	/* Key slot number */
	bool available; /* Tells whether key slot is free to use */
};

/* Security Engine AES RSA context */
struct tegra_se_aes_rsa_context {
	struct tegra_se_dev *se_dev;	/* Security Engine device */
	struct tegra_se_rsa_slot *slot;	/* Security Engine rsa key slot */
	u32 mod_len;
	u32 exp_len;
};

static void tegra_se_rsa_free_key_slot(struct tegra_se_rsa_slot *slot)
{
	if (slot) {
		spin_lock(&rsa_key_slot_lock);
		slot->available = true;
		spin_unlock(&rsa_key_slot_lock);
	}
}

static struct tegra_se_rsa_slot *tegra_se_alloc_rsa_key_slot(void)
{
	struct tegra_se_rsa_slot *slot = NULL;
	bool found = false;

	spin_lock(&rsa_key_slot_lock);
	list_for_each_entry(slot, &rsa_key_slot, node) {
		if (slot->available) {
			slot->available = false;
			found = true;
			break;
		}
	}
	spin_unlock(&rsa_key_slot_lock);
	return found ? slot : NULL;
}

static int tegra_init_rsa_key_slot(struct tegra_se_dev *se_dev)
{
	int i;

	se_dev->rsa_slot_list = kzalloc(sizeof(struct tegra_se_rsa_slot) *
					TEGRA_SE_RSA_KEYSLOT_COUNT, GFP_KERNEL);
	if (se_dev->rsa_slot_list == NULL) {
		dev_err(se_dev->dev, "rsa slot list memory allocation failed\n");
		return -ENOMEM;
	}
	spin_lock_init(&rsa_key_slot_lock);
	spin_lock(&rsa_key_slot_lock);
	for (i = 0; i < TEGRA_SE_RSA_KEYSLOT_COUNT; i++) {
		se_dev->rsa_slot_list[i].available = true;
		se_dev->rsa_slot_list[i].slot_num = i;
		INIT_LIST_HEAD(&se_dev->rsa_slot_list[i].node);
		list_add_tail(&se_dev->rsa_slot_list[i].node, &rsa_key_slot);
	}
	spin_unlock(&rsa_key_slot_lock);

	return 0;
}

static int tegra_se_rsa_init(struct ahash_request *req)
{
	return 0;
}

static int tegra_se_rsa_update(struct ahash_request *req)
{
	return 0;
}

static int tegra_se_rsa_final(struct ahash_request *req)
{
	return 0;
}

static int tegra_se_send_rsa_data(struct tegra_se_dev *se_dev,
				struct tegra_se_aes_rsa_context *rsa_ctx)
{
	u32 *cmdbuf_cpuvaddr = NULL;
	dma_addr_t cmdbuf_iova = 0;
	u32 cmdbuf_num_words = 0, i = 0;
	int err = 0;
	u32 val = 0;

	cmdbuf_cpuvaddr = dma_alloc_attrs(se_dev->dev->parent, SZ_4K,
				 &cmdbuf_iova, GFP_KERNEL, &attrs);
	if (!cmdbuf_cpuvaddr)
		return -ENOMEM;

	cmdbuf_cpuvaddr[i++] =
		__nvhost_opcode_nonincr(rsa_opcode_start_addr + 0x20, 1);
	cmdbuf_cpuvaddr[i++] = SE_OPERATION_WRSTALL(WRSTALL_TRUE);

	val = SE_CONFIG_ENC_ALG(ALG_RSA) |
		SE_CONFIG_DEC_ALG(ALG_NOP) |
		SE_CONFIG_DST(DST_RSAREG);
	cmdbuf_cpuvaddr[i++] = __nvhost_opcode_incr(rsa_opcode_start_addr, 6);
	cmdbuf_cpuvaddr[i++] = val;
	cmdbuf_cpuvaddr[i++] = RSA_KEY_SLOT(rsa_ctx->slot->slot_num);
	cmdbuf_cpuvaddr[i++] = (rsa_ctx->mod_len / 64) - 1;
	cmdbuf_cpuvaddr[i++] = (rsa_ctx->exp_len / 4);
	cmdbuf_cpuvaddr[i++] = (u32)(se_dev->src_ll->addr);
	cmdbuf_cpuvaddr[i++] = (u32)(SE_ADDR_HI_MSB(MSB(se_dev->src_ll->addr))
				| SE_ADDR_HI_SZ(se_dev->src_ll->data_len));

	cmdbuf_cpuvaddr[i++] =
		__nvhost_opcode_nonincr(rsa_opcode_start_addr + 0x20, 1);
	cmdbuf_cpuvaddr[i++] = SE_OPERATION_WRSTALL(WRSTALL_TRUE)
		| SE_OPERATION_LASTBUF(LASTBUF_TRUE)
			| SE_OPERATION_OP(OP_START);

	cmdbuf_num_words = i;

	err = tegra_se_channel_submit_gather(se_dev, NULL,
			cmdbuf_cpuvaddr, cmdbuf_iova,
			0, cmdbuf_num_words);

	dma_free_attrs(se_dev->dev->parent,
		SZ_4K, cmdbuf_cpuvaddr, cmdbuf_iova, &attrs);
	return err;
}

static int tegra_se_rsa_setkey(struct crypto_ahash *tfm, const u8 *key,
		unsigned int keylen)
{
	struct tegra_se_aes_rsa_context *ctx = crypto_ahash_ctx(tfm);
	struct tegra_se_dev *se_dev;
	u32 module_key_length = 0;
	u32 exponent_key_length = 0;
	u32 pkt, val;
	u32 key_size_words;
	u32 key_word_size = 4;
	u32 *pkeydata = (u32 *)key;
	s32 j = 0;
	struct tegra_se_rsa_slot *pslot;
	u32 cmdbuf_num_words = 0, i = 0;
	u32 *cmdbuf_cpuvaddr = NULL;
	dma_addr_t cmdbuf_iova = 0;
	int err = 0;

	se_dev = sg_tegra_se_dev[2];

	if (!ctx || !key)
		return -EINVAL;

	/* Allocate rsa key slot */
	if (!ctx->slot) {
		pslot = tegra_se_alloc_rsa_key_slot();
		if (!pslot) {
			dev_err(se_dev->dev, "no free key slot\n");
			return -ENOMEM;
		}
		ctx->slot = pslot;
	}

	module_key_length = (keylen >> 16);
	exponent_key_length = (keylen & (0xFFFF));

	if (!(((module_key_length / 64) >= 1) &&
			((module_key_length / 64) <= 4)))
		return -EINVAL;

	ctx->mod_len = module_key_length;
	ctx->exp_len = exponent_key_length;

	rsa_opcode_start_addr = SE3_RSA_CONFIG_REG_OFFSET;

	cmdbuf_cpuvaddr = dma_alloc_attrs(se_dev->dev->parent, SZ_64K,
				&cmdbuf_iova, GFP_KERNEL, &attrs);
	if (!cmdbuf_cpuvaddr)
		return -ENOMEM;

	cmdbuf_cpuvaddr[i++] =
		__nvhost_opcode_nonincr(rsa_opcode_start_addr + 0x20, 1);
	cmdbuf_cpuvaddr[i++] = SE_OPERATION_WRSTALL(WRSTALL_TRUE);

	if (exponent_key_length) {
		key_size_words = (exponent_key_length / key_word_size);
		/* Write exponent */
		for (j = (key_size_words - 1); j >= 0; j--) {
			pkt = RSA_KEY_NUM(ctx->slot->slot_num) |
				RSA_KEY_TYPE(RSA_KEY_TYPE_EXP) |
				RSA_KEY_PKT_WORD_ADDR(j);
			val = SE_RSA_KEYTABLE_PKT(pkt);
			cmdbuf_cpuvaddr[i++] =
			__nvhost_opcode_nonincr(rsa_opcode_start_addr + 0x148,
				1);
			cmdbuf_cpuvaddr[i++] = val;
			cmdbuf_cpuvaddr[i++] =
			__nvhost_opcode_nonincr(rsa_opcode_start_addr + 0x14C,
				1);
			cmdbuf_cpuvaddr[i++] = *pkeydata++;
		}
	}

	if (module_key_length) {
		key_size_words = (module_key_length / key_word_size);
		/* Write modulus */
		for (j = (key_size_words - 1); j >= 0; j--) {
			pkt = RSA_KEY_NUM(ctx->slot->slot_num) |
				RSA_KEY_TYPE(RSA_KEY_TYPE_MOD) |
				RSA_KEY_PKT_WORD_ADDR(j);
			val = SE_RSA_KEYTABLE_PKT(pkt);
			cmdbuf_cpuvaddr[i++] =
			__nvhost_opcode_nonincr(rsa_opcode_start_addr + 0x148,
				1);
			cmdbuf_cpuvaddr[i++] = val;
			cmdbuf_cpuvaddr[i++] =
			__nvhost_opcode_nonincr(rsa_opcode_start_addr + 0x14C,
				1);
			cmdbuf_cpuvaddr[i++] = *pkeydata++;
		}
	}

	cmdbuf_cpuvaddr[i++] =
		__nvhost_opcode_nonincr(rsa_opcode_start_addr + 0x20, 1);
	cmdbuf_cpuvaddr[i++] = SE_OPERATION_WRSTALL(WRSTALL_TRUE)
		| SE_OPERATION_LASTBUF(LASTBUF_TRUE)
			| SE_OPERATION_OP(OP_DUMMY);
	cmdbuf_num_words = i;

	err = tegra_se_channel_submit_gather(se_dev, NULL,
			cmdbuf_cpuvaddr, cmdbuf_iova,
			0, cmdbuf_num_words);

	dma_free_attrs(se_dev->dev->parent,
		SZ_64K, cmdbuf_cpuvaddr, cmdbuf_iova, &attrs);
	return err;
}

static void tegra_se_read_rsa_result(struct tegra_se_dev *se_dev,
	u8 *pdata, unsigned int nbytes)
{
	u32 *result = (u32 *)pdata;
	u32 i;

	/* Make SE is powered ON*/
	nvhost_module_busy(se_dev->pdev);

	for (i = 0; i < nbytes / 4; i++)
		result[i] = se_readl(se_dev, SE_RSA_OUTPUT +
				(i * sizeof(u32)));

	nvhost_module_idle(se_dev->pdev);
}

static int tegra_se_rsa_digest(struct ahash_request *req)
{
	struct crypto_ahash *tfm = NULL;
	struct tegra_se_aes_rsa_context *rsa_ctx = NULL;
	struct tegra_se_dev *se_dev;
	struct scatterlist *src_sg;
	u32 num_sgs;
	int total, err = 0;
	int chained;

	se_dev = sg_tegra_se_dev[2];
	if (!req)
		return -EINVAL;

	tfm = crypto_ahash_reqtfm(req);

	if (!tfm)
		return -EINVAL;

	rsa_ctx = crypto_ahash_ctx(tfm);

	if (!rsa_ctx || !rsa_ctx->slot)
		return -EINVAL;

	if (!req->nbytes)
		return -EINVAL;

	if ((req->nbytes < TEGRA_SE_RSA512_DIGEST_SIZE) ||
			(req->nbytes > TEGRA_SE_RSA2048_DIGEST_SIZE))
		return -EINVAL;

	num_sgs = tegra_se_count_sgs(req->src, req->nbytes, &chained);
	if (num_sgs > SE_MAX_SRC_SG_COUNT) {
		dev_err(se_dev->dev, "num of SG buffers are more\n");
		return -EINVAL;
	}

	*se_dev->src_ll_buf = num_sgs - 1;
	se_dev->src_ll = (struct tegra_se_ll *)(se_dev->src_ll_buf + 1);

	src_sg = req->src;
	total = req->nbytes;

	tegra_map_sg(se_dev->dev,
		src_sg, 1, DMA_TO_DEVICE, se_dev->src_ll, total);

	err = tegra_se_send_rsa_data(se_dev, rsa_ctx);

	tegra_se_read_rsa_result(se_dev, req->result, req->nbytes);

	src_sg = req->src;
	total = req->nbytes;
	if (total)
		tegra_unmap_sg(se_dev->dev, src_sg, DMA_FROM_DEVICE, total);
	return err;
}

static int tegra_se_rsa_finup(struct ahash_request *req)
{
	return 0;
}

static int tegra_se_rsa_cra_init(struct crypto_tfm *tfm)
{
	return 0;
}

static void tegra_se_rsa_cra_exit(struct crypto_tfm *tfm)
{
	struct tegra_se_aes_rsa_context *ctx = crypto_tfm_ctx(tfm);

	tegra_se_rsa_free_key_slot(ctx->slot);
	ctx->slot = NULL;
}

static struct crypto_alg aes_algs[] = {
	{
		.cra_name = "cbc(aes)",
		.cra_driver_name = "cbc-aes-tegra",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = TEGRA_SE_AES_BLOCK_SIZE,
		.cra_ctxsize  = sizeof(struct tegra_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_se_aes_cra_init,
		.cra_exit = tegra_se_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = TEGRA_SE_AES_MIN_KEY_SIZE,
			.max_keysize = TEGRA_SE_AES_MAX_KEY_SIZE,
			.ivsize = TEGRA_SE_AES_IV_SIZE,
			.setkey = tegra_se_aes_setkey,
			.encrypt = tegra_se_aes_cbc_encrypt,
			.decrypt = tegra_se_aes_cbc_decrypt,
		}
	}, {
		.cra_name = "rng_drbg",
		.cra_driver_name = "rng_drbg-aes-tegra",
		.cra_priority = 100,
		.cra_flags = CRYPTO_ALG_TYPE_RNG,
		.cra_ctxsize = sizeof(struct tegra_se_rng_context),
		.cra_type = &crypto_rng_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_se_rng_drbg_init,
		.cra_exit = tegra_se_rng_drbg_exit,
		.cra_u = {
			.rng = {
				.rng_make_random = tegra_se_rng_drbg_get_random,
				.rng_reset = tegra_se_rng_drbg_reset,
				.seedsize = TEGRA_SE_RNG_SEED_SIZE,
			}
		}
	}, {
		.cra_name = "ecb(aes)",
		.cra_driver_name = "ecb-aes-tegra",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = TEGRA_SE_AES_BLOCK_SIZE,
		.cra_ctxsize  = sizeof(struct tegra_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_se_aes_cra_init,
		.cra_exit = tegra_se_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = TEGRA_SE_AES_MIN_KEY_SIZE,
			.max_keysize = TEGRA_SE_AES_MAX_KEY_SIZE,
			.ivsize = TEGRA_SE_AES_IV_SIZE,
			.setkey = tegra_se_aes_setkey,
			.encrypt = tegra_se_aes_ecb_encrypt,
			.decrypt = tegra_se_aes_ecb_decrypt,
		}
	}, {
		.cra_name = "ctr(aes)",
		.cra_driver_name = "ctr-aes-tegra",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = TEGRA_SE_AES_BLOCK_SIZE,
		.cra_ctxsize  = sizeof(struct tegra_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_se_aes_cra_init,
		.cra_exit = tegra_se_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = TEGRA_SE_AES_MIN_KEY_SIZE,
			.max_keysize = TEGRA_SE_AES_MAX_KEY_SIZE,
			.ivsize = TEGRA_SE_AES_IV_SIZE,
			.setkey = tegra_se_aes_setkey,
			.encrypt = tegra_se_aes_ctr_encrypt,
			.decrypt = tegra_se_aes_ctr_decrypt,
			.geniv = "eseqiv",
		}
	}, {
		.cra_name = "ofb(aes)",
		.cra_driver_name = "ofb-aes-tegra",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = TEGRA_SE_AES_BLOCK_SIZE,
		.cra_ctxsize  = sizeof(struct tegra_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_se_aes_cra_init,
		.cra_exit = tegra_se_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = TEGRA_SE_AES_MIN_KEY_SIZE,
			.max_keysize = TEGRA_SE_AES_MAX_KEY_SIZE,
			.ivsize = TEGRA_SE_AES_IV_SIZE,
			.setkey = tegra_se_aes_setkey,
			.encrypt = tegra_se_aes_ofb_encrypt,
			.decrypt = tegra_se_aes_ofb_decrypt,
			.geniv = "eseqiv",
		}
	}
};

static struct ahash_alg hash_algs[] = {
	{
		.init = tegra_se_aes_cmac_init,
		.update = tegra_se_aes_cmac_update,
		.final = tegra_se_aes_cmac_final,
		.finup = tegra_se_aes_cmac_finup,
		.digest = tegra_se_aes_cmac_digest,
		.setkey = tegra_se_aes_cmac_setkey,
		.halg.digestsize = TEGRA_SE_AES_CMAC_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "cmac(aes)",
			.cra_driver_name = "tegra-se-cmac(aes)",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = TEGRA_SE_AES_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_aes_cmac_context),
			.cra_alignmask = 0,
			.cra_module	= THIS_MODULE,
			.cra_init	= tegra_se_aes_cmac_cra_init,
			.cra_exit	= tegra_se_aes_cmac_cra_exit,
		}
	}, {
		.init = tegra_se_sha_init,
		.update = tegra_se_sha_update,
		.final = tegra_se_sha_final,
		.finup = tegra_se_sha_finup,
		.digest = tegra_se_sha_digest,
		.halg.digestsize = SHA1_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "sha1",
			.cra_driver_name = "tegra-se-sha1",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA1_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_sha_cra_init,
			.cra_exit = tegra_se_sha_cra_exit,
		}
	}, {
		.init = tegra_se_sha_init,
		.update = tegra_se_sha_update,
		.final = tegra_se_sha_final,
		.finup = tegra_se_sha_finup,
		.digest = tegra_se_sha_digest,
		.halg.digestsize = SHA224_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "sha224",
			.cra_driver_name = "tegra-se-sha224",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA224_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_sha_cra_init,
			.cra_exit = tegra_se_sha_cra_exit,
		}
	}, {
		.init = tegra_se_sha_init,
		.update = tegra_se_sha_update,
		.final = tegra_se_sha_final,
		.finup = tegra_se_sha_finup,
		.digest = tegra_se_sha_digest,
		.halg.digestsize = SHA256_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "sha256",
			.cra_driver_name = "tegra-se-sha256",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA256_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_sha_cra_init,
			.cra_exit = tegra_se_sha_cra_exit,
		}
	}, {
		.init = tegra_se_sha_init,
		.update = tegra_se_sha_update,
		.final = tegra_se_sha_final,
		.finup = tegra_se_sha_finup,
		.digest = tegra_se_sha_digest,
		.halg.digestsize = SHA384_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "sha384",
			.cra_driver_name = "tegra-se-sha384",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA384_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_sha_cra_init,
			.cra_exit = tegra_se_sha_cra_exit,
		}
	}, {
		.init = tegra_se_sha_init,
		.update = tegra_se_sha_update,
		.final = tegra_se_sha_final,
		.finup = tegra_se_sha_finup,
		.digest = tegra_se_sha_digest,
		.halg.digestsize = SHA512_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "sha512",
			.cra_driver_name = "tegra-se-sha512",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA512_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_sha_cra_init,
			.cra_exit = tegra_se_sha_cra_exit,
		}
	}, {
		.init = tegra_se_rsa_init,
		.update = tegra_se_rsa_update,
		.final = tegra_se_rsa_final,
		.finup = tegra_se_rsa_finup,
		.digest = tegra_se_rsa_digest,
		.setkey = tegra_se_rsa_setkey,
		.halg.digestsize = TEGRA_SE_RSA512_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "rsa512",
			.cra_driver_name = "tegra-se-rsa512",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = TEGRA_SE_RSA512_DIGEST_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_aes_cmac_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_rsa_cra_init,
			.cra_exit = tegra_se_rsa_cra_exit,
		}
	}, {
		.init = tegra_se_rsa_init,
		.update = tegra_se_rsa_update,
		.final = tegra_se_rsa_final,
		.finup = tegra_se_rsa_finup,
		.digest = tegra_se_rsa_digest,
		.setkey = tegra_se_rsa_setkey,
		.halg.digestsize = TEGRA_SE_RSA1024_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "rsa1024",
			.cra_driver_name = "tegra-se-rsa1024",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = TEGRA_SE_RSA1024_DIGEST_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_aes_cmac_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_rsa_cra_init,
			.cra_exit = tegra_se_rsa_cra_exit,
		}
	}, {
		.init = tegra_se_rsa_init,
		.update = tegra_se_rsa_update,
		.final = tegra_se_rsa_final,
		.finup = tegra_se_rsa_finup,
		.digest = tegra_se_rsa_digest,
		.setkey = tegra_se_rsa_setkey,
		.halg.digestsize = TEGRA_SE_RSA1536_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "rsa1536",
			.cra_driver_name = "tegra-se-rsa1536",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = TEGRA_SE_RSA1536_DIGEST_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_aes_cmac_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_rsa_cra_init,
			.cra_exit = tegra_se_rsa_cra_exit,
		}
	}, {
		.init = tegra_se_rsa_init,
		.update = tegra_se_rsa_update,
		.final = tegra_se_rsa_final,
		.finup = tegra_se_rsa_finup,
		.digest = tegra_se_rsa_digest,
		.setkey = tegra_se_rsa_setkey,
		.halg.digestsize = TEGRA_SE_RSA2048_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "rsa2048",
			.cra_driver_name = "tegra-se-rsa2048",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = TEGRA_SE_RSA2048_DIGEST_SIZE,
			.cra_ctxsize = sizeof(struct tegra_se_aes_cmac_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_se_rsa_cra_init,
			.cra_exit = tegra_se_rsa_cra_exit,
		}
	}
};

static bool is_algo_supported(struct tegra_se_dev *se_dev, const char *algo)
{
	return true;
}

static struct tegra_se_chipdata tegra18_se_chipdata = {
	.aes_freq = 600000000,
};

static struct nvhost_device_data nvhost_se1_info = {
	.clocks = {{"se", UINT_MAX},
		   {"emc", UINT_MAX},
		   {"entropy", UINT_MAX}, {} },
	NVHOST_MODULE_NO_POWERGATE_ID,
	.can_powergate          = true,
	.powergate_delay        = 500,
	.class = NV_SE1_CLASS_ID,
	.private_data = &tegra18_se_chipdata,
	.serialize = 1,
	.push_work_done = 1,
};

static struct nvhost_device_data nvhost_se2_info = {
	.clocks = {{"se", UINT_MAX},
		   {"emc", UINT_MAX},
		   {"entropy", UINT_MAX}, {} },
	NVHOST_MODULE_NO_POWERGATE_ID,
	.can_powergate          = true,
	.powergate_delay        = 500,
	.class = NV_SE2_CLASS_ID,
	.private_data = &tegra18_se_chipdata,
	.serialize = 1,
	.push_work_done = 1,
};

static struct nvhost_device_data nvhost_se3_info = {
	.clocks = {{"se", UINT_MAX},
		   {"emc", UINT_MAX},
		   {"entropy", UINT_MAX}, {} },
	NVHOST_MODULE_NO_POWERGATE_ID,
	.can_powergate          = true,
	.powergate_delay        = 500,
	.class = NV_SE3_CLASS_ID,
	.private_data = &tegra18_se_chipdata,
	.serialize = 1,
	.push_work_done = 1,
};

static struct nvhost_device_data nvhost_se4_info = {
	.clocks = {{"se", UINT_MAX},
		   {"emc", UINT_MAX},
		   {"entropy", UINT_MAX}, {} },
	NVHOST_MODULE_NO_POWERGATE_ID,
	.can_powergate          = true,
	.powergate_delay        = 500,
	.class = NV_SE4_CLASS_ID,
	.private_data = &tegra18_se_chipdata,
	.serialize = 1,
	.push_work_done = 1,
};

static struct of_device_id tegra_se_of_match[] = {
	{
		.compatible = "nvidia,tegra186-se-nvhost",
		.data = &nvhost_se1_info,
	}, {
		.compatible = "nvidia,tegra186-se-nvhost",
		.data = &nvhost_se2_info,
	}, {
		.compatible = "nvidia,tegra186-se-nvhost",
		.data = &nvhost_se3_info,
	}, {
		.compatible = "nvidia,tegra186-se-nvhost",
		.data = &nvhost_se4_info,
	},
};
MODULE_DEVICE_TABLE(of, tegra_se_of_match);

static int tegra_se_probe(struct platform_device *pdev)
{
	struct tegra_se_dev *se_dev = NULL;
	struct nvhost_device_data *pdata = NULL;
	const struct of_device_id *match;
	int err = 0, i = 0;
	char se_nvhost_name[15];

	se_dev = kzalloc(sizeof(struct tegra_se_dev), GFP_KERNEL);
	if (!se_dev) {
		dev_err(&pdev->dev, "memory allocation failed\n");
		return -ENOMEM;
	}

	if (pdev->dev.of_node) {
		match = of_match_device(of_match_ptr(tegra_se_of_match),
				&pdev->dev);
		if (!match) {
			dev_err(&pdev->dev, "Error: No device match found\n");
			kfree(se_dev);
			return -ENODEV;
		}
		pdata = (struct nvhost_device_data *)match->data;
	} else {
		pdata =
		(struct nvhost_device_data *)pdev->id_entry->driver_data;
	}

	spin_lock_init(&se_dev->lock);
	crypto_init_queue(&se_dev->queue, TEGRA_SE_CRYPTO_QUEUE_LENGTH);

	se_dev->dev = &pdev->dev;
	se_dev->pdev = pdev;

	mutex_init(&pdata->lock);
	pdata->pdev = pdev;

	/* store chipdata inside se_dev and store se_dev into private_data */
	se_dev->chipdata = pdata->private_data;
	pdata->private_data = se_dev;

	/* store the pdata into drvdata */
	platform_set_drvdata(pdev, pdata);

	err = nvhost_client_device_get_resources(pdev);
	if (err) {
		dev_err(se_dev->dev,
			"nvhost_client_device_get_resources failed for SE%d\n",
			se_num + 1);
		return err;
	}

	err = nvhost_module_init(pdev);
	if (err) {
		dev_err(se_dev->dev,
			"nvhost_module_init failed for SE%d\n", se_num + 1);
		return err;
	}

#ifdef CONFIG_PM_GENERIC_DOMAINS
	err = nvhost_module_add_domain(&pdata->pd, pdev);
	if (err) {
		dev_err(se_dev->dev,
		"nvhost_module_add_domain failed for SE%d\n", se_num + 1);
		return err;
	}
#endif

	err = nvhost_client_device_init(pdev);
	if (err) {
		dev_err(se_dev->dev,
		"nvhost_client_device_init failed for SE%d\n", se_num + 1);
		return err;
	}

	err = nvhost_channel_map(pdata, &se_dev->channel, pdata);
	if (err) {
		dev_err(se_dev->dev, "Nvhost Channel map failed\n");
		return err;
	}

	se_dev->io_regs = pdata->aperture[0];

	if (se_num == 0) {
		err = tegra_init_key_slot(se_dev);
		if (err) {
			dev_err(se_dev->dev, "init_key_slot failed\n");
			goto fail;
		}
	}
	if (se_num == 2) {
		err = tegra_init_rsa_key_slot(se_dev);
		if (err) {
			dev_err(se_dev->dev, "init_rsa_key_slot failed\n");
			goto fail;
		}
	}

	mutex_init(&se_dev->mtx);
	INIT_WORK(&se_dev->se_work, tegra_se_work_handler);
	se_dev->se_work_q = alloc_workqueue("se_work_q",
				WQ_HIGHPRI | WQ_UNBOUND, 16);
	if (!se_dev->se_work_q) {
		dev_err(se_dev->dev, "alloc_workqueue failed\n");
		goto fail;
	}

	err = tegra_se_alloc_ll_buf(se_dev, SE_MAX_SRC_SG_COUNT,
			SE_MAX_DST_SG_COUNT);
	if (err) {
		dev_err(se_dev->dev, "can not allocate ll dma buffer\n");
		goto ll_alloc_fail;
	}

	if (se_num == 0) {
		for (i = 0; i < 2; i++) {
			if (is_algo_supported(se_dev, aes_algs[i].cra_name)) {
				INIT_LIST_HEAD(&aes_algs[i].cra_list);
				err = crypto_register_alg(&aes_algs[i]);
				if (err) {
					dev_err(se_dev->dev,
					"crypto_register_alg failed index[%d]\n",
					i);
					goto reg_fail;
				}
			}
		}
	}

	if (se_num == 1) {
		for (i = 2; i < ARRAY_SIZE(aes_algs); i++) {
			if (is_algo_supported(se_dev, aes_algs[i].cra_name)) {
				INIT_LIST_HEAD(&aes_algs[i].cra_list);
				err = crypto_register_alg(&aes_algs[i]);
				if (err) {
					dev_err(se_dev->dev,
					"crypto_register_alg failed index[%d]\n",
					i);
					goto reg_fail;
				}
			}
		}

		if (is_algo_supported(se_dev,
				hash_algs[0].halg.base.cra_name)) {
			err = crypto_register_ahash(&hash_algs[0]);
			if (err) {
				dev_err(se_dev->dev,
				"crypto_register_ahash alg failed index[0]\n");
				goto reg_fail;
			}
		}
	}

	if (se_num == 3) {
		for (i = 1; i < 6; i++) {
			if (is_algo_supported(se_dev,
				hash_algs[i].halg.base.cra_name)) {
				err = crypto_register_ahash(&hash_algs[i]);
				if (err) {
					dev_err(se_dev->dev,
					"crypto_register_ahash alg"
					"failed index[%d]\n", i);
					goto reg_fail;
				}
			}
		}
	}

	if (se_num == 2) {
		for (i = 6; i < ARRAY_SIZE(hash_algs); i++) {
			if (is_algo_supported(se_dev,
				hash_algs[i].halg.base.cra_name)) {
				err = crypto_register_ahash(&hash_algs[i]);
				if (err) {
					dev_err(se_dev->dev,
					"crypto_register_ahash"
					"alg failed index[%d]\n", i);
					goto reg_fail;
				}
			}
		}
	}

	/* Make sure engine is powered ON with clk enabled */
	err = nvhost_module_busy(pdev);
	if (err) {
		dev_err(se_dev->dev, "nvhost_module_busy failed for se_dev\n");
		goto reg_fail;
	}

	/* RNG register only exists in se0/se1 */
	if (se_num == 0) {
		se_writel(se_dev,
			SE_RNG_SRC_CONFIG_RO_ENT_SRC(DRBG_RO_ENT_SRC_ENABLE) |
		SE_RNG_SRC_CONFIG_RO_ENT_SRC_LOCK(DRBG_RO_ENT_SRC_LOCK_ENABLE),
				SE_RNG_SRC_CONFIG_REG_OFFSET);
	}
	/* Power OFF after SE register update */
	nvhost_module_idle(pdev);

	sprintf(se_nvhost_name, "158%d0000.se", (se_num+1));
	se_dev->syncpt_id = nvhost_get_syncpt_host_managed(se_dev->pdev,
						0, se_nvhost_name);
	if (!se_dev->syncpt_id) {
		err = -EINVAL;
		dev_err(se_dev->dev, "Cannot get syncpt_id for SE%d\n",
				se_num + 1);
		goto reg_fail;
	}

	se_dev->src_ll = kzalloc(sizeof(struct tegra_se_ll), GFP_KERNEL);
	se_dev->dst_ll = kzalloc(sizeof(struct tegra_se_ll), GFP_KERNEL);

	se_dev->aes_cmdbuf_cpuvaddr = dma_alloc_attrs(se_dev->dev->parent,
			SZ_16K, &se_dev->aes_cmdbuf_iova, GFP_KERNEL, &attrs);
	if (!se_dev->aes_cmdbuf_cpuvaddr)
		goto cmd_buf_alloc_fail;

	se_dev->se_dev_num = se_num;
	sg_tegra_se_dev[se_num] = se_dev;
	se_num++;

	dev_info(se_dev->dev, "%s: complete", __func__);
	return 0;

cmd_buf_alloc_fail:
	if (se_dev->src_ll)
		kfree(se_dev->src_ll);
	if (se_dev->dst_ll)
		kfree(se_dev->dst_ll);
	nvhost_syncpt_put_ref_ext(se_dev->pdev, se_dev->syncpt_id);

reg_fail:
	tegra_se_free_ll_buf(se_dev);
ll_alloc_fail:
	if (se_dev->se_work_q)
		destroy_workqueue(se_dev->se_work_q);
fail:
	platform_set_drvdata(pdev, NULL);
	kfree(se_dev);
	for (i = 0; i < 4; i++)
		sg_tegra_se_dev[i] = NULL;

	return err;
}

static int tegra_se_remove(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct tegra_se_dev *se_dev = pdata->private_data;
	int i;

	if (!se_dev)
		return -ENODEV;

	cancel_work_sync(&se_dev->se_work);
	if (se_dev->se_work_q)
		destroy_workqueue(se_dev->se_work_q);

	tegra_se_free_ll_buf(se_dev);

	for (i = 0; i < ARRAY_SIZE(aes_algs); i++)
		crypto_unregister_alg(&aes_algs[i]);

	for (i = 0; i < ARRAY_SIZE(hash_algs); i++)
		crypto_unregister_ahash(&hash_algs[i]);

	nvhost_syncpt_put_ref_ext(se_dev->pdev, se_dev->syncpt_id);
	nvhost_client_device_release(pdev);

	dma_free_attrs(se_dev->dev->parent, SZ_16K,
			se_dev->aes_cmdbuf_cpuvaddr,
			se_dev->aes_cmdbuf_iova, &attrs);

	kfree(se_dev);
	for (i = 0; i < 4; i++)
		sg_tegra_se_dev[i] = NULL;
	return 0;
}

static struct platform_driver tegra_se_driver = {
	.probe  = tegra_se_probe,
	.remove = tegra_se_remove,
	.driver = {
		.name   = "tegra-se-nvhost",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_se_of_match),
	},
};

static struct of_device_id tegra_se_domain_match[] = {
	{.compatible = "nvidia,tegra186-se-pd",
	.data = (struct nvhost_device_data *)&nvhost_se1_info},
	{},
};

static int __init tegra_se_module_init(void)
{
	int ret;

	ret = nvhost_domain_init(tegra_se_domain_match);
	if (ret)
		return ret;
	return  platform_driver_register(&tegra_se_driver);
}

static void __exit tegra_se_module_exit(void)
{
	platform_driver_unregister(&tegra_se_driver);
}

module_init(tegra_se_module_init);
module_exit(tegra_se_module_exit);

MODULE_DESCRIPTION("Tegra Crypto algorithm support using Host1x Interface");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL");
MODULE_ALIAS("tegra-se-nvhost");
