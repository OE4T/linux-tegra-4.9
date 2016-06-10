/*
 * Cryptographic API.
 * drivers/crypto/tegra-hv-vse.c
 *
 * Support for Tegra Virtual Security Engine hardware crypto algorithms.
 *
 * Copyright (c) 2016, NVIDIA Corporation. All Rights Reserved.
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
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <crypto/scatterwalk.h>
#include <crypto/algapi.h>
#include <crypto/internal/rng.h>
#include <crypto/internal/hash.h>
#include <crypto/sha.h>
#include <linux/delay.h>
#include <linux/tegra-ivc.h>
#include <linux/iommu.h>

#define TEGRA_HV_VSE_SHA_MAX_LL_NUM 26
#define TEGRA_HV_VSE_AES_MAX_LL_NUM 17
#define TEGRA_HV_VSE_CRYPTO_QUEUE_LENGTH 50
#define TEGRA_VIRTUAL_SE_KEY_128_SIZE 16
#define TEGRA_HV_VSE_AES_CMAC_MAX_LL_NUM 36

/* Security Engine Linked List */
struct tegra_virtual_se_ll {
	dma_addr_t addr; /* DMA buffer address */
	u32 data_len; /* Data length in DMA buffer */
};

struct tegra_virtual_se_dev {
	struct device *dev;
	u32 stream_id;
	/* lock for Crypto queue access*/
	spinlock_t lock;
	/* Security Engine crypto queue */
	struct crypto_queue queue;
	/* Work queue busy status */
	bool work_q_busy;
	struct work_struct se_work;
	struct workqueue_struct *vse_work_q;
	struct mutex mtx;
};

struct tegra_virtual_se_addr {
	u32 lo;
	u32 hi;
};

struct tegra_virtual_se_linklist {
	u8 number;
	struct tegra_virtual_se_addr addr[TEGRA_HV_VSE_SHA_MAX_LL_NUM];
};

struct tegra_virtual_se_aes_linklist {
	u8 number;
	struct tegra_virtual_se_addr addr[TEGRA_HV_VSE_AES_MAX_LL_NUM];
};

union tegra_virtual_se_aes_args {
	struct keyiv {
		u8 slot;
		u8 length;
		u8 type;
		u8 data[32];
		u8 oiv[16];
		u8 uiv[16];
	} key;
	struct aes_encdec {
		u8 streamid;
		u8 keyslot;
		u8 key_length;
		u8 mode;
		u8 ivsel;
		u8 lctr[16];
		u8 ctr_cntn;
		u32 data_length;
		u8 src_ll_num;
		struct tegra_virtual_se_addr
			src_addr[TEGRA_HV_VSE_AES_MAX_LL_NUM];
		u8 dst_ll_num;
		struct tegra_virtual_se_addr
			dst_addr[TEGRA_HV_VSE_AES_MAX_LL_NUM];
	} op;
	struct aes_cmac {
		u8 streamid;
		u8 keyslot;
		u8 key_length;
		u8 ivsel;
		u32 data_length;
		u64 dst;
		struct tegra_virtual_se_aes_linklist src;
	} op_cmac;
	struct aes_rng {
		u8 streamid;
		u32 data_length;
		u64 dst;
	} op_rng;
};

union tegra_virtual_se_rsa_args {
	struct key {
		u8 slot;
		u32 length;
		u8 data[256];
	} key;
	struct encdec {
		u64 p_src;
		u64 p_dst;
		u32 mod_length;
		u32 exp_length;
		u8 streamid;
		u8 keyslot;
	} op;
};

struct tegra_virtual_se_sha_args {
	u32 msg_block_length;
	u32 msg_total_length;
	u32 msg_left_length;
	u8 mode;
	u8 streamid;
	u32 hash[0x10];
	u64 dst;
	struct tegra_virtual_se_linklist src;
};

struct tegra_virtual_se_ivc_resp_msg_t {
	u8 engine;
	u8 tag;
	u8 status;
	u8 keyslot;
};

struct tegra_virtual_se_ivc_tx_msg_t {
	u8 engine;
	u8 tag;
	u8 cmd;
	union {
		union tegra_virtual_se_aes_args aes;
		struct tegra_virtual_se_sha_args sha;
		union tegra_virtual_se_rsa_args rsa;
	} args;
};

/* Security Engine SHA context */
struct tegra_virtual_se_sha_context {
	/* Security Engine device */
	struct tegra_virtual_se_dev *se_dev;
	/* SHA operation mode */
	u32 op_mode;
	unsigned int digest_size;
	u8 mode;
	u32 hashblock_size;
};

struct sha_zero_length_vector {
	unsigned int size;
	char *digest;
};

/* Tegra Virtual Security Engine operation modes */
enum tegra_virtual_se_op_mode {
	/* Secure Hash Algorithm-1 (SHA1) mode */
	VIRTUAL_SE_OP_MODE_SHA1,
	/* Secure Hash Algorithm-224  (SHA224) mode */
	VIRTUAL_SE_OP_MODE_SHA224 = 4,
	/* Secure Hash Algorithm-256  (SHA256) mode */
	VIRTUAL_SE_OP_MODE_SHA256,
	/* Secure Hash Algorithm-384  (SHA384) mode */
	VIRTUAL_SE_OP_MODE_SHA384,
	/* Secure Hash Algorithm-512  (SHA512) mode */
	VIRTUAL_SE_OP_MODE_SHA512,
};

/* Security Engine AES context */
struct tegra_virtual_se_aes_context {
	/* Security Engine device */
	struct tegra_virtual_se_dev *se_dev;
	struct ablkcipher_request *req;
	/* Security Engine key slot */
	u32 aes_keyslot;
	/* key length in bytes */
	u32 keylen;
	/* AES operation mode */
	u32 op_mode;
	/* Is key slot */
	bool is_key_slot_allocated;
};

enum tegra_virtual_se_aes_op_mode {
	AES_CBC,
	AES_ECB,
	AES_CTR,
	AES_OFB,
};

/* Security Engine request context */
struct tegra_virtual_se_aes_req_context {
	/* Security Engine device */
	struct tegra_virtual_se_dev *se_dev;
	/* Security Engine operation mode */
	enum tegra_virtual_se_aes_op_mode op_mode;
	/* Operation type */
	bool encrypt;
	/* Engine id */
	u8 engine_id;
};

/* Security Engine AES CMAC context */
struct tegra_virtual_se_aes_cmac_context {
	u32 aes_keyslot;
	/* key length in bits */
	u32 keylen;
	/* Key1 */
	u8 K1[TEGRA_VIRTUAL_SE_KEY_128_SIZE];
	/* Key2 */
	u8 K2[TEGRA_VIRTUAL_SE_KEY_128_SIZE];
	bool is_key_slot_allocated;
};

enum se_engine_id {
	VIRTUAL_SE_AES0,
	VIRTUAL_SE_AES1,
	VIRTUAL_SE_RSA,
	VIRTUAL_SE_SHA,
	VIRTUAL_MAX_SE_ENGINE_NUM,
};

#define VIRTUAL_SE_CMD_SHA_HASH 1

enum tegra_virual_se_aes_iv_type {
	AES_ORIGINAL_IV,
	AES_UPDATED_IV,
};

#define VIRTUAL_SE_AES_BLOCK_SIZE 16
#define VIRTUAL_SE_AES_MIN_KEY_SIZE 16
#define VIRTUAL_SE_AES_MAX_KEY_SIZE 32
#define VIRTUAL_SE_AES_IV_SIZE 16
#define VIRTUAL_SE_CMD_AES_ALLOC_KEY 1
#define VIRTUAL_SE_CMD_AES_RELEASE_KEY 2
#define VIRTUAL_SE_CMD_AES_SET_KEY 3
#define VIRTUAL_SE_CMD_AES_ENCRYPT 4
#define VIRTUAL_SE_CMD_AES_DECRYPT 5
#define VIRTUAL_SE_CMD_AES_CMAC 6

#define VIRTUAL_SE_SHA_HASH_BLOCK_SIZE_512BIT (512 / 8)
#define VIRTUAL_SE_SHA_HASH_BLOCK_SIZE_1024BIT (1024 / 8)

#define VIRTUAL_SE_CMD_RSA_ALLOC_KEY   1
#define VIRTUAL_SE_CMD_RSA_RELEASE_KEY 2
#define VIRTUAL_SE_CMD_RSA_SET_EXPKEY  3
#define VIRTUAL_SE_CMD_RSA_SET_MODKEY  4
#define VIRTUAL_SE_CMD_RSA_ENCDEC      5

#define TEGRA_VIRTUAL_SE_TIMEOUT_1S 1000000

#define TEGRA_VIRTUAL_SE_RSA512_DIGEST_SIZE    64
#define TEGRA_VIRTUAL_SE_RSA1024_DIGEST_SIZE   128
#define TEGRA_VIRTUAL_SE_RSA1536_DIGEST_SIZE   192
#define TEGRA_VIRTUAL_SE_RSA2048_DIGEST_SIZE   256

#define TEGRA_VIRTUAL_SE_SHA_MAX_BUFFER_SIZE 0x1000000

#define AES_KEYTBL_TYPE_KEY 1
#define AES_KEYTBL_TYPE_OIV 2
#define AES_KEYTBL_TYPE_UIV 4

#define TEGRA_VIRTUAL_SE_AES_IV_SIZE 16
#define TEGRA_VIRTUAL_SE_AES_LCTR_SIZE 16
#define TEGRA_VIRTUAL_SE_AES_LCTR_CNTN 1

#define TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE 16
#define TEGRA_VIRUTAL_SE_AES_CMAC_DIGEST_SIZE 16

/* Security Engine RSA context */
struct tegra_virtual_se_rsa_context {
	struct tegra_virtual_se_dev *se_dev;
	u32 rsa_keyslot;
	u32 exponent_length;
	u32 module_length;
	bool key_alloated;
};

/* Lock for IVC channel */
static DEFINE_MUTEX(se_ivc_lock);

static struct tegra_hv_ivc_cookie *g_ivck;
static struct tegra_virtual_se_dev *g_virtual_se_dev[VIRTUAL_MAX_SE_ENGINE_NUM];

#define GET_MSB(x)  ((x) >> (8*sizeof(x)-1))
static void tegra_virtual_se_leftshift_onebit(u8 *in_buf, u32 size, u8 *org_msb)
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

static int tegra_hv_vse_send_ivc(
	struct tegra_virtual_se_dev *se_dev,
	struct tegra_hv_ivc_cookie *pivck,
	void *pbuf,
	int length)
{
	u32 timeout;

	timeout = TEGRA_VIRTUAL_SE_TIMEOUT_1S;
	mutex_lock(&se_ivc_lock);
	while (tegra_hv_ivc_channel_notified(pivck) != 0) {
		if (!timeout) {
			mutex_unlock(&se_ivc_lock);
			dev_err(se_dev->dev, "ivc reset timeout\n");
			return -EINVAL;
		}
		udelay(1);
		timeout--;
	}

	timeout = TEGRA_VIRTUAL_SE_TIMEOUT_1S;
	while (tegra_hv_ivc_can_write(pivck) == 0) {
		if (!timeout) {
			mutex_unlock(&se_ivc_lock);
			dev_err(se_dev->dev, "ivc send message timeout\n");
			return -EINVAL;
		}
		udelay(1);
		timeout--;
	}

	tegra_hv_ivc_write(pivck, pbuf, length);
	mutex_unlock(&se_ivc_lock);
	return 0;
}

static int tegra_hv_vse_read_ivc(
	struct tegra_virtual_se_dev *se_dev,
	struct tegra_hv_ivc_cookie *pivck,
	void *pbuf,
	int size)
{
	int timeout;

	timeout = TEGRA_VIRTUAL_SE_TIMEOUT_1S;

	mutex_lock(&se_ivc_lock);
	while (tegra_hv_ivc_read(pivck, pbuf, size) == -ENOMEM) {
		if (!timeout) {
			mutex_unlock(&se_ivc_lock);
			dev_err(se_dev->dev, "ivc read message timeout\n");
			return -EINVAL;
		}
		udelay(1);
		timeout--;
	}
	mutex_unlock(&se_ivc_lock);
	return 0;
}

static int tegra_hv_vse_count_sgs(struct scatterlist *sl, u32 nbytes)
{
	struct scatterlist *sg = sl;
	int sg_nents = 0;

	while (sg) {
		sg = scatterwalk_sg_next(sg);
		sg_nents++;
		nbytes -= min(sl->length, nbytes);
		if (!nbytes)
			break;
	}

	return sg_nents;
}

static int tegra_hv_vse_sha_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct tegra_virtual_se_sha_context *sha_ctx = crypto_ahash_ctx(tfm);

	if (!req)
		return -EINVAL;

	sha_ctx->digest_size = crypto_ahash_digestsize(tfm);
	switch (sha_ctx->digest_size) {
	case SHA1_DIGEST_SIZE:
		sha_ctx->mode = VIRTUAL_SE_OP_MODE_SHA1;
		sha_ctx->hashblock_size = VIRTUAL_SE_SHA_HASH_BLOCK_SIZE_512BIT;
		break;
	case SHA224_DIGEST_SIZE:
		sha_ctx->mode = VIRTUAL_SE_OP_MODE_SHA224;
		sha_ctx->hashblock_size = VIRTUAL_SE_SHA_HASH_BLOCK_SIZE_512BIT;
		break;
	case SHA256_DIGEST_SIZE:
		sha_ctx->mode = VIRTUAL_SE_OP_MODE_SHA256;
		sha_ctx->hashblock_size = VIRTUAL_SE_SHA_HASH_BLOCK_SIZE_512BIT;
		break;
	case SHA384_DIGEST_SIZE:
		sha_ctx->mode = VIRTUAL_SE_OP_MODE_SHA384;
		sha_ctx->hashblock_size =
			VIRTUAL_SE_SHA_HASH_BLOCK_SIZE_1024BIT;
		break;
	case SHA512_DIGEST_SIZE:
		sha_ctx->mode = VIRTUAL_SE_OP_MODE_SHA512;
		sha_ctx->hashblock_size =
			VIRTUAL_SE_SHA_HASH_BLOCK_SIZE_1024BIT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int  tegra_hv_vse_sha_update(struct ahash_request *req)
{
	return 0;
}

static int  tegra_hv_vse_sha_finup(struct ahash_request *req)
{
	return 0;
}

static int  tegra_hv_vse_sha_final(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct tegra_virtual_se_sha_context *sha_ctx = crypto_ahash_ctx(tfm);
	struct tegra_virtual_se_ivc_tx_msg_t ivc_tx;
	struct tegra_virtual_se_ivc_resp_msg_t ivc_rx;
	struct scatterlist *sg;
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_SHA];
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	u32 total_len;
	int i = 0;
	int err = 0;
	u32 num_sgs;
	u32 sg_count = 0;
	dma_addr_t dst_buf_addr;
	int len, process_len;
	u32 addr, addr_offset;
	u32 list_count  = 0;
	u32 mode;
	struct sha_zero_length_vector zero_vec[] = {
		{
			.size = SHA1_DIGEST_SIZE,
			.digest = "\xda\x39\xa3\xee\x5e\x6b\x4b\x0d"
				  "\x32\x55\xbf\xef\x95\x60\x18\x90"
				  "\xaf\xd8\x07\x09",
		}, {
			.size = SHA224_DIGEST_SIZE,
			.digest = "\xd1\x4a\x02\x8c\x2a\x3a\x2b\xc9"
				  "\x47\x61\x02\xbb\x28\x82\x34\xc4"
				  "\x15\xa2\xb0\x1f\x82\x8e\xa6\x2a"
				  "\xc5\xb3\xe4\x2f",
		}, {
			.size = SHA256_DIGEST_SIZE,
			.digest = "\xe3\xb0\xc4\x42\x98\xfc\x1c\x14"
				  "\x9a\xfb\xf4\xc8\x99\x6f\xb9\x24"
				  "\x27\xae\x41\xe4\x64\x9b\x93\x4c"
				  "\xa4\x95\x99\x1b\x78\x52\xb8\x55",
		}, {
			.size = SHA384_DIGEST_SIZE,
			.digest = "\x38\xb0\x60\xa7\x51\xac\x96\x38"
				  "\x4c\xd9\x32\x7e\xb1\xb1\xe3\x6a"
				  "\x21\xfd\xb7\x11\x14\xbe\x07\x43"
				  "\x4c\x0c\xc7\xbf\x63\xf6\xe1\xda"
				  "\x27\x4e\xde\xbf\xe7\x6f\x65\xfb"
				  "\xd5\x1a\xd2\xf1\x48\x98\xb9\x5b",
		}, {
			.size = SHA512_DIGEST_SIZE,
			.digest = "\xcf\x83\xe1\x35\x7e\xef\xb8\xbd"
				  "\xf1\x54\x28\x50\xd6\x6d\x80\x07"
				  "\xd6\x20\xe4\x05\x0b\x57\x15\xdc"
				  "\x83\xf4\xa9\x21\xd3\x6c\xe9\xce"
				  "\x47\xd0\xd1\x3c\x5d\x85\xf2\xb0"
				  "\xff\x83\x18\xd2\x87\x7e\xec\x2f"
				  "\x63\xb9\x31\xbd\x47\x41\x7a\x81"
				  "\xa5\x38\x32\x7a\xf9\x27\xda\x3e",
		}
	};

	if (!req)
		return -EINVAL;

	if (!req->nbytes) {
		/*
		 *  SW WAR for zero length SHA operation since
		 *  SE HW can't accept zero length SHA operation.
		 */
		if (sha_ctx->mode == VIRTUAL_SE_OP_MODE_SHA1)
			mode = VIRTUAL_SE_OP_MODE_SHA1;
		else
			mode = sha_ctx->mode - VIRTUAL_SE_OP_MODE_SHA224 + 1;
		memcpy(req->result, zero_vec[mode].digest, zero_vec[mode].size);

		return 0;
	}

	sg = req->src;
	num_sgs = tegra_hv_vse_count_sgs(sg, req->nbytes);
	if (num_sgs > TEGRA_HV_VSE_SHA_MAX_LL_NUM) {
		dev_err(se_dev->dev,
			"\n Unsupported number of linked list %d\n", i);
		return -EINVAL;
	}

	dst_buf_addr = dma_map_single(se_dev->dev,
				req->result,
				sha_ctx->digest_size,
				DMA_FROM_DEVICE);

	ivc_tx.engine = VIRTUAL_SE_SHA;
	ivc_tx.cmd = VIRTUAL_SE_CMD_SHA_HASH;
	ivc_tx.args.sha.dst = (u64)dst_buf_addr;
	ivc_tx.args.sha.streamid = se_dev->stream_id;
	ivc_tx.args.sha.mode = sha_ctx->mode;
	ivc_tx.args.sha.msg_total_length = req->nbytes;

	total_len = req->nbytes;
	while (sg && total_len) {
		dma_map_sg(se_dev->dev, sg, 1, DMA_TO_DEVICE);
		len = min(sg->length, total_len);
		addr = sg_dma_address(sg);
		addr_offset = 0;
		while (len >= TEGRA_VIRTUAL_SE_SHA_MAX_BUFFER_SIZE) {
			process_len = (TEGRA_VIRTUAL_SE_SHA_MAX_BUFFER_SIZE -
				sha_ctx->hashblock_size);
			ivc_tx.args.sha.src.addr[i].lo =
				addr + addr_offset;
			ivc_tx.args.sha.src.addr[i++].hi = process_len;
			addr_offset += process_len;
			total_len -= process_len;
			len -= process_len;
			list_count++;
			if (list_count > TEGRA_HV_VSE_SHA_MAX_LL_NUM) {
				dev_err(se_dev->dev,
					"Unsupported no. of list %d\n", i);
				goto exit;
			}
		}
		if (len) {
			ivc_tx.args.sha.src.addr[i].lo =
				addr + addr_offset;
			ivc_tx.args.sha.src.addr[i++].hi = len;
			list_count++;
			if (list_count > TEGRA_HV_VSE_SHA_MAX_LL_NUM) {
				dev_err(se_dev->dev,
					"Unsupported no. of list %d\n", i);
				goto exit;
			}
		}
		total_len -= len;
		sg = scatterwalk_sg_next(sg);
		sg_count++;
	}

	ivc_tx.args.sha.src.number = i;
	ivc_tx.args.sha.msg_block_length = req->nbytes;
	ivc_tx.args.sha.msg_left_length = req->nbytes;

	err = tegra_hv_vse_send_ivc(se_dev, pivck, &ivc_tx, sizeof(ivc_tx));
	if (err)
		goto exit;

	err = tegra_hv_vse_read_ivc(se_dev, pivck, &ivc_rx, sizeof(ivc_rx));
	if (err)
		goto exit;

	if (ivc_rx.status)
		err = ivc_rx.status;

exit:
	sg = req->src;
	while (sg && sg_count--) {
		dma_unmap_sg(se_dev->dev, sg, 1, DMA_TO_DEVICE);
		sg = scatterwalk_sg_next(sg);
	}

	dma_unmap_single(se_dev->dev,
		dst_buf_addr, sha_ctx->digest_size, DMA_FROM_DEVICE);

	return err;
}

static int  tegra_hv_vse_sha_digest(struct ahash_request *req)
{
	return  tegra_hv_vse_sha_init(req) ?: tegra_hv_vse_sha_final(req);
}

static int  tegra_hv_vse_sha_cra_init(struct crypto_tfm *tfm)
{
	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct tegra_virtual_se_sha_context));
	return 0;
}

static void  tegra_hv_vse_sha_cra_exit(struct crypto_tfm *tfm)
{
	/* do nothing */
}

static int tegra_hv_vse_rsa_init(struct ahash_request *req)
{
	return 0;
}

static int tegra_hv_vse_rsa_update(struct ahash_request *req)
{
	return 0;
}

static int tegra_hv_vse_rsa_final(struct ahash_request *req)
{
	return 0;
}

static int tegra_hv_vse_rsa_digest(struct ahash_request *req)
{
	struct crypto_ahash *tfm = NULL;
	struct tegra_virtual_se_rsa_context *rsa_ctx;
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_RSA];
	u32 num_sgs;
	int err = 0;
	dma_addr_t dma_addr_out;
	struct tegra_virtual_se_ivc_tx_msg_t ivc_tx;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	struct tegra_virtual_se_ivc_resp_msg_t ivc_rx;

	if (!req)
		return -EINVAL;

	tfm = crypto_ahash_reqtfm(req);
	if (!tfm)
		return -EINVAL;

	rsa_ctx = crypto_ahash_ctx(tfm);
	if (!rsa_ctx)
		return -EINVAL;

	if (!rsa_ctx->key_alloated) {
		dev_err(se_dev->dev, "RSA key not allocated\n");
		return -EINVAL;
	}

	if (!req->nbytes)
		return -EINVAL;

	if ((req->nbytes < TEGRA_VIRTUAL_SE_RSA512_DIGEST_SIZE) ||
			(req->nbytes > TEGRA_VIRTUAL_SE_RSA2048_DIGEST_SIZE))
		return -EINVAL;

	num_sgs = tegra_hv_vse_count_sgs(req->src, req->nbytes);
	if (num_sgs > 1) {
		dev_err(se_dev->dev, "num of SG buffers are more\n");
		return -EINVAL;
	}
	dma_addr_out = dma_map_single(se_dev->dev,
		req->result, req->nbytes, DMA_FROM_DEVICE);

	dma_map_sg(se_dev->dev, req->src, 1, DMA_TO_DEVICE);

	ivc_tx.engine = VIRTUAL_SE_RSA;
	ivc_tx.cmd = VIRTUAL_SE_CMD_RSA_ENCDEC;
	ivc_tx.args.rsa.op.keyslot = rsa_ctx->rsa_keyslot;
	ivc_tx.args.rsa.op.exp_length = rsa_ctx->exponent_length;
	ivc_tx.args.rsa.op.mod_length = rsa_ctx->module_length;
	ivc_tx.args.rsa.op.p_src = sg_dma_address(req->src);
	ivc_tx.args.rsa.op.p_dst = dma_addr_out;
	ivc_tx.args.rsa.op.streamid = se_dev->stream_id;

	err = tegra_hv_vse_send_ivc(se_dev, pivck, &ivc_tx, sizeof(ivc_tx));
	if (err)
		goto exit;

	err = tegra_hv_vse_read_ivc(se_dev, pivck, &ivc_rx, sizeof(ivc_rx));
	if (err)
		goto exit;

	if (ivc_rx.status)
		err = ivc_rx.status;

exit:
	dma_unmap_sg(se_dev->dev, req->src, 1, DMA_TO_DEVICE);
	dma_unmap_single(se_dev->dev,
		dma_addr_out,  req->nbytes, DMA_FROM_DEVICE);

	return err;
}



static int tegra_hv_vse_rsa_setkey(struct crypto_ahash *tfm, const u8 *key,
		unsigned int keylen)
{
	struct tegra_virtual_se_rsa_context *rsa_ctx = crypto_ahash_ctx(tfm);
	struct tegra_virtual_se_ivc_tx_msg_t ivc_tx;
	struct tegra_virtual_se_ivc_resp_msg_t ivc_rx;
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_RSA];
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	u32 module_key_length = 0;
	u32 exponent_key_length = 0;
	u32 *pkeydata = (u32 *)key;
	u32 *ivc_key_data;
	int err = 0;
	int i = 0;

	if (!rsa_ctx)
		return -EINVAL;

	if (!rsa_ctx->key_alloated) {
		/* Allocate RSA key slot */
		ivc_tx.engine = VIRTUAL_SE_RSA;
		ivc_tx.cmd = VIRTUAL_SE_CMD_RSA_ALLOC_KEY;

		err = tegra_hv_vse_send_ivc(se_dev,
				pivck,
				&ivc_tx,
				sizeof(ivc_tx));
		if (err)
			return err;

		err = tegra_hv_vse_read_ivc(se_dev,
				pivck,
				&ivc_rx,
				sizeof(ivc_rx));
		if (err)
			return err;

		if (ivc_rx.status) {
			dev_err(se_dev->dev,
				"Error from Read IVC %d\n", ivc_rx.status);
			return ivc_rx.status;
		}
		rsa_ctx->key_alloated = true;
		rsa_ctx->rsa_keyslot = ivc_rx.keyslot;
	}

	exponent_key_length = (keylen & (0xFFFF));
	module_key_length = (keylen >> 16);
	rsa_ctx->exponent_length = exponent_key_length;
	rsa_ctx->module_length = module_key_length;

	if (exponent_key_length) {
		/* Send RSA Exponent Key */
		ivc_tx.engine = VIRTUAL_SE_RSA;
		ivc_tx.cmd = VIRTUAL_SE_CMD_RSA_SET_EXPKEY;
		ivc_tx.args.rsa.key.slot = rsa_ctx->rsa_keyslot;
		ivc_tx.args.rsa.key.length = exponent_key_length;

		ivc_key_data = (u32 *)ivc_tx.args.rsa.key.data;
		for (i = ((exponent_key_length / 4) - 1); i >= 0; i--)
			*(ivc_key_data + i) = *pkeydata++;

		err = tegra_hv_vse_send_ivc(se_dev,
				pivck,
				&ivc_tx,
				sizeof(ivc_tx));
		if (err)
			return err;

		err = tegra_hv_vse_read_ivc(se_dev,
				pivck,
				&ivc_rx,
				sizeof(ivc_rx));
		if (err)
			return err;

		if (ivc_rx.status) {
			dev_err(se_dev->dev,
				"Error from Read IVC %d\n", ivc_rx.status);
			return ivc_rx.status;
		}
	}

	if (module_key_length) {
		/* Send RSA Module Key */
		ivc_tx.engine = VIRTUAL_SE_RSA;
		ivc_tx.cmd = VIRTUAL_SE_CMD_RSA_SET_MODKEY;
		ivc_tx.args.rsa.key.slot = rsa_ctx->rsa_keyslot;
		ivc_tx.args.rsa.key.length = module_key_length;

		ivc_key_data = (u32 *)ivc_tx.args.rsa.key.data;
		for (i = ((module_key_length / 4) - 1); i >= 0; i--)
			*(ivc_key_data + i) = *pkeydata++;

		err = tegra_hv_vse_send_ivc(se_dev,
				pivck,
				&ivc_tx,
				sizeof(ivc_tx));
		if (err)
			return err;

		err = tegra_hv_vse_read_ivc(se_dev,
				pivck,
				&ivc_rx,
				sizeof(ivc_rx));
		if (err)
			return err;

		if (ivc_rx.status) {
			dev_err(se_dev->dev,
				"Error from Read IVC %d\n", ivc_rx.status);
			return ivc_rx.status;
		}
	}

	return 0;
}

static int tegra_hv_vse_rsa_finup(struct ahash_request *req)
{
	return 0;
}

static int tegra_hv_vse_rsa_cra_init(struct crypto_tfm *tfm)
{
	return 0;
}
static void tegra_hv_vse_rsa_cra_exit(struct crypto_tfm *tfm)
{
	struct tegra_virtual_se_rsa_context *rsa_ctx = crypto_tfm_ctx(tfm);
	struct tegra_virtual_se_ivc_tx_msg_t ivc_tx;
	struct tegra_virtual_se_ivc_resp_msg_t ivc_rx;
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_RSA];
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	int err;

	if (!rsa_ctx || !rsa_ctx->key_alloated)
		return;

	ivc_tx.engine = VIRTUAL_SE_RSA;
	ivc_tx.cmd = VIRTUAL_SE_CMD_RSA_RELEASE_KEY;
	ivc_tx.args.rsa.key.slot = rsa_ctx->rsa_keyslot;

	err = tegra_hv_vse_send_ivc(se_dev, pivck, &ivc_tx, sizeof(ivc_tx));
	if (err)
		return;

	err = tegra_hv_vse_read_ivc(se_dev, pivck, &ivc_rx, sizeof(ivc_rx));
	if (err)
		return;

	if (ivc_rx.status) {
		dev_err(se_dev->dev, "Error from Read IVC %d\n", ivc_rx.status);
		return;
	}
}

static int tegra_hv_vse_aes_set_keyiv(struct tegra_virtual_se_dev *se_dev,
	u8 *data, u32 keylen, u8 keyslot, u8 type)
{
	struct tegra_virtual_se_ivc_tx_msg_t ivc_tx;
	struct tegra_virtual_se_ivc_resp_msg_t ivc_rx;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	int err;

	ivc_tx.engine = VIRTUAL_SE_AES1;
	ivc_tx.cmd = VIRTUAL_SE_CMD_AES_SET_KEY;
	ivc_tx.args.aes.key.slot = keyslot;
	ivc_tx.args.aes.key.type = type;

	if (type & AES_KEYTBL_TYPE_KEY) {
		ivc_tx.args.aes.key.length = keylen;
		memcpy(ivc_tx.args.aes.key.data, data, keylen);
	}

	if (type & AES_KEYTBL_TYPE_OIV)
		memcpy(ivc_tx.args.aes.key.oiv, data,
			TEGRA_VIRTUAL_SE_AES_IV_SIZE);

	if (type & AES_KEYTBL_TYPE_UIV)
		memcpy(ivc_tx.args.aes.key.uiv, data,
			TEGRA_VIRTUAL_SE_AES_IV_SIZE);

	err = tegra_hv_vse_send_ivc(se_dev, pivck, &ivc_tx, sizeof(ivc_tx));
	if (err)
		return err;

	err = tegra_hv_vse_read_ivc(se_dev, pivck, &ivc_rx, sizeof(ivc_rx));
	if (err)
		return err;

	if (ivc_rx.status) {
		dev_err(se_dev->dev, "Error from Read IVC %d\n", ivc_rx.status);
		return err;
	}

	return 0;
}

static void tegra_hv_vse_process_new_req(struct crypto_async_request *async_req)
{
	struct tegra_virtual_se_dev *se_dev;
	struct ablkcipher_request *req = ablkcipher_request_cast(async_req);
	struct tegra_virtual_se_aes_req_context *req_ctx =
			ablkcipher_request_ctx(req);
	struct tegra_virtual_se_aes_context *aes_ctx =
			crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	struct tegra_virtual_se_ivc_tx_msg_t ivc_tx;
	struct tegra_virtual_se_ivc_resp_msg_t ivc_rx;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	struct scatterlist *src_sg, *dst_sg;
	int src_sg_count = 0, dst_sg_count = 0;
	int err = 0;
	u32 total_len;
	int i;

	se_dev = req_ctx->se_dev;
	if (!aes_ctx->is_key_slot_allocated) {
		dev_err(se_dev->dev, "AES Key slot not allocated\n");
		err = -EINVAL;
		goto err_exit;
	}

	ivc_tx.engine = req_ctx->engine_id;

	if (req_ctx->encrypt == true)
		ivc_tx.cmd = VIRTUAL_SE_CMD_AES_ENCRYPT;
	else
		ivc_tx.cmd = VIRTUAL_SE_CMD_AES_DECRYPT;

	if (req->info) {
		if (req_ctx->op_mode == AES_CTR) {
			memcpy(ivc_tx.args.aes.op.lctr, req->info,
				TEGRA_VIRTUAL_SE_AES_LCTR_SIZE);
			ivc_tx.args.aes.op.ctr_cntn  =
				TEGRA_VIRTUAL_SE_AES_LCTR_CNTN;
		} else {
			err = tegra_hv_vse_aes_set_keyiv(se_dev, req->info,
				aes_ctx->keylen,
				aes_ctx->aes_keyslot,
				AES_KEYTBL_TYPE_OIV);
			if (err)
				goto err_exit;
		}
	}

	ivc_tx.args.aes.op.keyslot = aes_ctx->aes_keyslot;
	ivc_tx.args.aes.op.key_length = aes_ctx->keylen;
	ivc_tx.args.aes.op.streamid = se_dev->stream_id;
	ivc_tx.args.aes.op.mode = req_ctx->op_mode;

	ivc_tx.args.aes.op.ivsel = AES_ORIGINAL_IV;
	src_sg = req->src;
	dst_sg = req->dst;

	i = 0;
	total_len = req->nbytes;
	while (src_sg && total_len) {
		dma_map_sg(se_dev->dev, src_sg, 1, DMA_TO_DEVICE);
		ivc_tx.args.aes.op.src_addr[i].lo = sg_dma_address(src_sg);
		ivc_tx.args.aes.op.src_addr[i++].hi =
				min(src_sg->length, total_len);
		total_len -= min(src_sg->length, total_len);
		src_sg_count++;
		src_sg = scatterwalk_sg_next(src_sg);
	}
	ivc_tx.args.aes.op.src_ll_num = i;

	i = 0;
	total_len = req->nbytes;
	while (dst_sg && total_len) {
		dma_map_sg(se_dev->dev, dst_sg, 1, DMA_FROM_DEVICE);
		ivc_tx.args.aes.op.dst_addr[i].lo = sg_dma_address(dst_sg);
		ivc_tx.args.aes.op.dst_addr[i++].hi =
				min(dst_sg->length, total_len);
		total_len -= min(dst_sg->length, total_len);
		dst_sg_count++;
		dst_sg = scatterwalk_sg_next(dst_sg);
	}
	ivc_tx.args.aes.op.dst_ll_num = i;
	ivc_tx.args.aes.op.data_length = req->nbytes;

	err = tegra_hv_vse_send_ivc(se_dev, pivck, &ivc_tx, sizeof(ivc_tx));
	if (err)
		goto exit;

	err = tegra_hv_vse_read_ivc(se_dev, pivck, &ivc_rx, sizeof(ivc_rx));
	if (err)
		goto exit;

	if (ivc_rx.status)
		err = ivc_rx.status;

exit:
	src_sg = req->src;
	while (src_sg && src_sg_count--) {
		dma_unmap_sg(se_dev->dev, src_sg, 1, DMA_TO_DEVICE);
		src_sg = scatterwalk_sg_next(src_sg);
	}

	dst_sg = req->dst;
	while (dst_sg && dst_sg_count--) {
		dma_unmap_sg(se_dev->dev, dst_sg, 1, DMA_FROM_DEVICE);
		src_sg = scatterwalk_sg_next(dst_sg);
	}

err_exit:
	if (req->base.complete)
		req->base.complete(&req->base, err);
}

static void tegra_hv_vse_work_handler(struct work_struct *work)
{
	struct tegra_virtual_se_dev *se_dev = container_of(work,
					struct tegra_virtual_se_dev, se_work);
	struct crypto_async_request *async_req = NULL;
	struct crypto_async_request *backlog = NULL;
	unsigned long flags;

	mutex_lock(&se_dev->mtx);
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
			tegra_hv_vse_process_new_req(async_req);
			async_req = NULL;
		}
	} while (se_dev->work_q_busy);
	mutex_unlock(&se_dev->mtx);
}

static int tegra_hv_vse_aes_queue_req(struct tegra_virtual_se_dev *se_dev,
				struct ablkcipher_request *req)
{
	unsigned long flags;
	bool idle = true;
	int err = 0;

	if (req->nbytes % VIRTUAL_SE_AES_BLOCK_SIZE)
		return -EINVAL;

	if (!tegra_hv_vse_count_sgs(req->src, req->nbytes))
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
		queue_work(se_dev->vse_work_q, &se_dev->se_work);
	}

	return err;
}

static int tegra_hv_vse_aes_cra_init(struct crypto_tfm *tfm)
{
	tfm->crt_ablkcipher.reqsize =
		sizeof(struct tegra_virtual_se_aes_req_context);

	return 0;
}

static void tegra_hv_vse_aes_cra_exit(struct crypto_tfm *tfm)
{
	struct tegra_virtual_se_aes_context *ctx = crypto_tfm_ctx(tfm);
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	struct tegra_virtual_se_ivc_tx_msg_t ivc_tx;
	struct tegra_virtual_se_ivc_resp_msg_t ivc_rx;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	int err;

	if (!ctx)
		return;

	if (!ctx->is_key_slot_allocated)
		return;

	/* Allocate AES key slot */
	ivc_tx.engine = VIRTUAL_SE_AES1;
	ivc_tx.cmd = VIRTUAL_SE_CMD_AES_RELEASE_KEY;
	ivc_tx.args.aes.key.slot = ctx->aes_keyslot;

	err = tegra_hv_vse_send_ivc(se_dev, pivck, &ivc_tx, sizeof(ivc_tx));
	if (err)
		return;

	err = tegra_hv_vse_read_ivc(se_dev, pivck, &ivc_rx, sizeof(ivc_rx));
	if (err)
		return;

	if (ivc_rx.status) {
		dev_err(se_dev->dev, "Error from Read IVC %d\n", ivc_rx.status);
		return;
	}

}

static int tegra_hv_vse_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	struct tegra_virtual_se_aes_req_context *req_ctx =
		ablkcipher_request_ctx(req);

	req_ctx->encrypt = true;
	req_ctx->op_mode = AES_CBC;
	req_ctx->engine_id = VIRTUAL_SE_AES1;
	req_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	return tegra_hv_vse_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_hv_vse_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	struct tegra_virtual_se_aes_req_context *req_ctx =
			ablkcipher_request_ctx(req);

	req_ctx->encrypt = false;
	req_ctx->op_mode = AES_CBC;
	req_ctx->engine_id = VIRTUAL_SE_AES1;
	req_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	return tegra_hv_vse_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_hv_vse_aes_ecb_encrypt(struct ablkcipher_request *req)
{
	struct tegra_virtual_se_aes_req_context *req_ctx =
		ablkcipher_request_ctx(req);

	req_ctx->encrypt = true;
	req_ctx->op_mode = AES_ECB;
	req_ctx->engine_id = VIRTUAL_SE_AES1;
	req_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	return tegra_hv_vse_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_hv_vse_aes_ecb_decrypt(struct ablkcipher_request *req)
{
	struct tegra_virtual_se_aes_req_context *req_ctx =
			ablkcipher_request_ctx(req);

	req_ctx->encrypt = false;
	req_ctx->op_mode = AES_ECB;
	req_ctx->engine_id = VIRTUAL_SE_AES1;
	req_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	return tegra_hv_vse_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_hv_vse_aes_ctr_encrypt(struct ablkcipher_request *req)
{
	struct tegra_virtual_se_aes_req_context *req_ctx =
		ablkcipher_request_ctx(req);

	req_ctx->encrypt = true;
	req_ctx->op_mode = AES_CTR;
	req_ctx->engine_id = VIRTUAL_SE_AES1;
	req_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	return tegra_hv_vse_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_hv_vse_aes_ctr_decrypt(struct ablkcipher_request *req)
{
	struct tegra_virtual_se_aes_req_context *req_ctx =
			ablkcipher_request_ctx(req);

	req_ctx->encrypt = false;
	req_ctx->op_mode = AES_CTR;
	req_ctx->engine_id = VIRTUAL_SE_AES1;
	req_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	return tegra_hv_vse_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_hv_vse_aes_ofb_encrypt(struct ablkcipher_request *req)
{
	struct tegra_virtual_se_aes_req_context *req_ctx =
		ablkcipher_request_ctx(req);

	req_ctx->encrypt = true;
	req_ctx->op_mode = AES_OFB;
	req_ctx->engine_id = VIRTUAL_SE_AES1;
	req_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	return tegra_hv_vse_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_hv_vse_aes_ofb_decrypt(struct ablkcipher_request *req)
{
	struct tegra_virtual_se_aes_req_context *req_ctx =
			ablkcipher_request_ctx(req);

	req_ctx->encrypt = false;
	req_ctx->op_mode = AES_OFB;
	req_ctx->engine_id = VIRTUAL_SE_AES1;
	req_ctx->se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	return tegra_hv_vse_aes_queue_req(req_ctx->se_dev, req);
}

static int tegra_hv_vse_cmac_init(struct ahash_request *req)
{

	return 0;
}

static int tegra_hv_vse_cmac_update(struct ahash_request *req)
{

	return 0;
}

static int tegra_hv_vse_cmac_final(struct ahash_request *req)
{
	struct tegra_virtual_se_aes_cmac_context *cmac_ctx =
			crypto_ahash_ctx(crypto_ahash_reqtfm(req));
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	struct tegra_virtual_se_ivc_tx_msg_t ivc_tx;
	struct tegra_virtual_se_ivc_resp_msg_t ivc_rx;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	struct scatterlist *src_sg;
	struct sg_mapping_iter miter;
	u32 num_sgs, blocks_to_process, last_block_bytes = 0, bytes_to_copy = 0;
	unsigned int total_len, i = 0;
	bool padding_needed = false;
	unsigned long flags;
	unsigned int sg_flags = SG_MITER_ATOMIC;
	u8 *temp_buffer = NULL;
	bool use_orig_iv = true;
	int src_sg_count = 0;
	int process_len;
	dma_addr_t cmac_dma_addr;
	u8 *cmac_buffer = NULL;
	dma_addr_t piv_buf_dma_addr;
	u8 *piv_buf = NULL;
	dma_addr_t result_dma_addr;
	int err = 0;

	blocks_to_process = req->nbytes / TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;
	/* num of bytes less than block size */
	if ((req->nbytes % TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE) ||
		!blocks_to_process) {
		padding_needed = true;
		last_block_bytes =
			req->nbytes % TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;
	} else {
		/* decrement num of blocks */
		blocks_to_process--;
		last_block_bytes = TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;
	}

	ivc_tx.engine = VIRTUAL_SE_AES1;
	ivc_tx.cmd = VIRTUAL_SE_CMD_AES_CMAC;
	src_sg = req->src;
	num_sgs = tegra_hv_vse_count_sgs(src_sg, req->nbytes);
	if (num_sgs > TEGRA_HV_VSE_AES_CMAC_MAX_LL_NUM) {
		dev_err(se_dev->dev,
			"\n Unsupported number of linked list %d\n", i);
		return -EINVAL;
	}

	piv_buf = dma_alloc_coherent(se_dev->dev,
			TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
			&piv_buf_dma_addr, GFP_KERNEL);
	if (!piv_buf) {
		dev_err(se_dev->dev, "can not allocate piv buffer");
		return -ENOMEM;
	}

	/* first process all blocks except last block */
	if (blocks_to_process) {
		total_len = blocks_to_process * TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;
		process_len = total_len;
		ivc_tx.args.aes.op_cmac.keyslot = cmac_ctx->aes_keyslot;
		ivc_tx.args.aes.op_cmac.key_length = cmac_ctx->keylen;
		ivc_tx.args.aes.op_cmac.streamid = se_dev->stream_id;
		ivc_tx.args.aes.op_cmac.ivsel = AES_ORIGINAL_IV;

		src_sg = req->src;
		while (src_sg && total_len) {
			dma_map_sg(se_dev->dev, src_sg, 1, DMA_TO_DEVICE);
			ivc_tx.args.aes.op_cmac.src.addr[i].lo =
				sg_dma_address(src_sg);
			ivc_tx.args.aes.op_cmac.src.addr[i++].hi =
				min(src_sg->length, total_len);
			total_len -= min(src_sg->length, total_len);
			src_sg_count++;
			src_sg = scatterwalk_sg_next(src_sg);
		}

		ivc_tx.args.aes.op_cmac.src.number = i;
		ivc_tx.args.aes.op_cmac.data_length = process_len;
		ivc_tx.args.aes.op_cmac.dst = piv_buf_dma_addr;

		err = tegra_hv_vse_send_ivc(se_dev,
			pivck,
			&ivc_tx,
			sizeof(ivc_tx));
		if (err)
			goto exit;

		err = tegra_hv_vse_read_ivc(se_dev,
				pivck, &ivc_rx, sizeof(ivc_rx));
		if (err)
			goto exit;

		if (ivc_rx.status) {
			err = ivc_rx.status;
			goto exit;
		}

		use_orig_iv = false;
	}

	/* get the last block bytes from the sg_dma buffer using miter */
	src_sg = req->src;
	num_sgs = tegra_hv_vse_count_sgs(req->src, req->nbytes);
	sg_flags |= SG_MITER_FROM_SG;
	sg_miter_start(&miter, req->src, num_sgs, sg_flags);
	local_irq_save(flags);
	total_len = 0;
	cmac_buffer = dma_alloc_coherent(se_dev->dev,
				TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
				&cmac_dma_addr, GFP_KERNEL);
	if (!cmac_buffer)
		goto exit;

	temp_buffer = cmac_buffer;
	while (sg_miter_next(&miter) && total_len < req->nbytes) {
		unsigned int len;

		len = min(miter.length, (size_t)(req->nbytes - total_len));
		if ((req->nbytes - (total_len + len)) <= last_block_bytes) {
			bytes_to_copy =
				last_block_bytes -
				(req->nbytes - (total_len + len));
			memcpy(temp_buffer, miter.addr + (len - bytes_to_copy),
				bytes_to_copy);
			last_block_bytes -= bytes_to_copy;
			temp_buffer += bytes_to_copy;
		}
		total_len += len;
	}
	sg_miter_stop(&miter);
	local_irq_restore(flags);

	/* process last block */
	if (padding_needed) {
		/* pad with 0x80, 0, 0 ... */
		last_block_bytes =
			req->nbytes % TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;
		cmac_buffer[last_block_bytes] = 0x80;
		for (i = last_block_bytes+1;
			i < TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE; i++)
			cmac_buffer[i] = 0;
		/* XOR with K2 */
		for (i = 0; i < TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE; i++)
			cmac_buffer[i] ^= cmac_ctx->K2[i];
	} else {
		/* XOR with K1 */
		for (i = 0; i < TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE; i++)
			cmac_buffer[i] ^= cmac_ctx->K1[i];
	}

	ivc_tx.args.aes.op_cmac.src.addr[0].lo = cmac_dma_addr;
	ivc_tx.args.aes.op_cmac.src.addr[0].hi =
		TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;

	if (use_orig_iv) {
		ivc_tx.args.aes.op_cmac.ivsel = AES_ORIGINAL_IV;
	} else {
		ivc_tx.args.aes.op_cmac.ivsel = AES_UPDATED_IV;
		err = tegra_hv_vse_aes_set_keyiv(se_dev, piv_buf,
				cmac_ctx->keylen,
				cmac_ctx->aes_keyslot,
				AES_KEYTBL_TYPE_UIV);
		if (err)
			goto exit;
	}

	result_dma_addr = dma_map_single(se_dev->dev,
				req->result,
				TEGRA_VIRUTAL_SE_AES_CMAC_DIGEST_SIZE,
				DMA_FROM_DEVICE);
	ivc_tx.args.aes.op_cmac.src.number = 1;
	ivc_tx.args.aes.op_cmac.keyslot = cmac_ctx->aes_keyslot;
	ivc_tx.args.aes.op_cmac.key_length = cmac_ctx->keylen;
	ivc_tx.args.aes.op_cmac.streamid = se_dev->stream_id;
	ivc_tx.args.aes.op_cmac.dst = result_dma_addr;
	ivc_tx.args.aes.op_cmac.data_length = TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;
	err = tegra_hv_vse_send_ivc(se_dev, pivck, &ivc_tx, sizeof(ivc_tx));
	if (err)
		goto unmap_exit;

	err = tegra_hv_vse_read_ivc(se_dev, pivck, &ivc_rx, sizeof(ivc_rx));
	if (err)
		goto unmap_exit;

	if (ivc_rx.status) {
		err = ivc_rx.status;
		goto unmap_exit;
	}

unmap_exit:
	dma_unmap_single(se_dev->dev, result_dma_addr,
		TEGRA_VIRUTAL_SE_AES_CMAC_DIGEST_SIZE, DMA_FROM_DEVICE);

exit:
	src_sg = req->src;
	while (src_sg && src_sg_count--) {
		dma_unmap_sg(se_dev->dev, src_sg, 1, DMA_TO_DEVICE);
		src_sg = scatterwalk_sg_next(src_sg);
	}

	if (cmac_buffer)
		dma_free_coherent(se_dev->dev, TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
			cmac_buffer, cmac_dma_addr);
	if (piv_buf)
		dma_free_coherent(se_dev->dev, TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
			piv_buf, piv_buf_dma_addr);

	return err;
}

static int tegra_hv_vse_cmac_finup(struct ahash_request *req)
{

	return 0;
}

static int tegra_hv_vse_cmac_digest(struct ahash_request *req)
{

	return tegra_hv_vse_cmac_init(req) ?: tegra_hv_vse_cmac_final(req);
}

static int tegra_hv_vse_cmac_setkey(struct crypto_ahash *tfm, const u8 *key,
		unsigned int keylen)
{
	struct tegra_virtual_se_aes_cmac_context *ctx =
			crypto_tfm_ctx(crypto_ahash_tfm(tfm));
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	struct tegra_virtual_se_ivc_tx_msg_t ivc_tx;
	struct tegra_virtual_se_ivc_resp_msg_t ivc_rx;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	int err = 0;
	u8 piv[TEGRA_VIRTUAL_SE_AES_IV_SIZE];
	u32 *pbuf;
	dma_addr_t pbuf_adr;
	u8 const rb = 0x87;
	u8 msb;

	if (!ctx)
		return -EINVAL;

	if (!ctx->is_key_slot_allocated) {
		/* Allocate AES key slot */
		ivc_tx.engine = VIRTUAL_SE_AES1;
		ivc_tx.cmd = VIRTUAL_SE_CMD_AES_ALLOC_KEY;

		err = tegra_hv_vse_send_ivc(se_dev, pivck, &ivc_tx,
				sizeof(ivc_tx));
		if (err)
			return err;

		err = tegra_hv_vse_read_ivc(se_dev, pivck, &ivc_rx,
				sizeof(ivc_rx));
		if (err)
			return err;

		if (ivc_rx.status) {
			dev_err(se_dev->dev, "Error from Read IVC %d\n",
				ivc_rx.status);
			return ivc_rx.status;
		}
		ctx->aes_keyslot = ivc_rx.keyslot;
		ctx->is_key_slot_allocated = true;
	}

	ctx->keylen = keylen;
	err = tegra_hv_vse_aes_set_keyiv(se_dev, (u8 *)key, keylen,
			ctx->aes_keyslot, AES_KEYTBL_TYPE_KEY);
	if (err)
		goto exit;

	memset(piv, 0, TEGRA_VIRTUAL_SE_AES_IV_SIZE);
	err = tegra_hv_vse_aes_set_keyiv(se_dev, piv,
				ctx->keylen,
				ctx->aes_keyslot,
				AES_KEYTBL_TYPE_OIV);
	if (err)
		goto exit;

	pbuf = dma_alloc_coherent(se_dev->dev, TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
		&pbuf_adr, GFP_KERNEL);
	if (!pbuf) {
		dev_err(se_dev->dev, "can not allocate dma buffer");
		err = -ENOMEM;
		goto exit;
	}
	memset(pbuf, 0, TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE);

	ivc_tx.engine = VIRTUAL_SE_AES1;
	ivc_tx.cmd = VIRTUAL_SE_CMD_AES_ENCRYPT;
	ivc_tx.args.aes.op.keyslot = ctx->aes_keyslot;
	ivc_tx.args.aes.op.key_length = ctx->keylen;
	ivc_tx.args.aes.op.streamid = se_dev->stream_id;
	ivc_tx.args.aes.op.mode = AES_CBC;
	ivc_tx.args.aes.op.ivsel = AES_ORIGINAL_IV;
	ivc_tx.args.aes.op.src_addr[0].lo = pbuf_adr;
	ivc_tx.args.aes.op.src_addr[0].hi = TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;
	ivc_tx.args.aes.op.src_ll_num = 1;
	ivc_tx.args.aes.op.dst_addr[0].lo = pbuf_adr;
	ivc_tx.args.aes.op.dst_addr[0].hi = TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;
	ivc_tx.args.aes.op.dst_ll_num = 1;
	ivc_tx.args.aes.op.data_length = TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE;

	err = tegra_hv_vse_send_ivc(se_dev, pivck, &ivc_tx, sizeof(ivc_tx));
	if (err)
		goto free_exit;

	err = tegra_hv_vse_read_ivc(se_dev, pivck, &ivc_rx, sizeof(ivc_rx));
	if (err)
		goto free_exit;

	if (ivc_rx.status) {
		err = ivc_rx.status;
		goto free_exit;
	}

	/* compute K1 subkey */
	memcpy(ctx->K1, pbuf, TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE);

	tegra_virtual_se_leftshift_onebit(ctx->K1,
		TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
		&msb);
	if (msb)
		ctx->K1[TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE - 1] ^= rb;

	/* compute K2 subkey */
	memcpy(ctx->K2,
		ctx->K1,
		TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE);
	tegra_virtual_se_leftshift_onebit(ctx->K2,
		TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
		&msb);

	if (msb)
		ctx->K2[TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE - 1] ^= rb;

free_exit:
	if (pbuf) {
		dma_free_coherent(se_dev->dev, TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
			pbuf, pbuf_adr);
	}

exit:
	return err;
}

static int tegra_hv_vse_cmac_cra_init(struct crypto_tfm *tfm)
{
	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
			 sizeof(struct tegra_virtual_se_aes_cmac_context));

	return 0;
}

static void tegra_hv_vse_cmac_cra_exit(struct crypto_tfm *tfm)
{
	struct tegra_virtual_se_aes_cmac_context *ctx = crypto_tfm_ctx(tfm);
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	struct tegra_virtual_se_ivc_tx_msg_t ivc_tx;
	struct tegra_virtual_se_ivc_resp_msg_t ivc_rx;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	int err;

	if (!ctx)
		return;

	if (!ctx->is_key_slot_allocated)
		return;

	/* Allocate AES key slot */
	ivc_tx.engine = VIRTUAL_SE_AES1;
	ivc_tx.cmd = VIRTUAL_SE_CMD_AES_RELEASE_KEY;
	ivc_tx.args.aes.key.slot = ctx->aes_keyslot;
	err = tegra_hv_vse_send_ivc(se_dev, pivck, &ivc_tx, sizeof(ivc_tx));
	if (err)
		return;

	err = tegra_hv_vse_read_ivc(se_dev, pivck, &ivc_rx, sizeof(ivc_rx));
	if (err)
		return;

	if (ivc_rx.status) {
		dev_err(se_dev->dev, "Error from Read IVC %d\n", ivc_rx.status);
		return;
	}
	ctx->is_key_slot_allocated = false;
}

static int tegra_hv_vse_aes_setkey(struct crypto_ablkcipher *tfm,
	const u8 *key, u32 keylen)
{
	struct tegra_virtual_se_aes_context *ctx = crypto_ablkcipher_ctx(tfm);
	struct tegra_virtual_se_dev *se_dev = g_virtual_se_dev[VIRTUAL_SE_AES1];
	struct tegra_virtual_se_ivc_tx_msg_t ivc_tx;
	struct tegra_virtual_se_ivc_resp_msg_t ivc_rx;
	struct tegra_hv_ivc_cookie *pivck = g_ivck;
	int err;

	if (!ctx)
		return -EINVAL;

	if (!ctx->is_key_slot_allocated) {
		/* Allocate AES key slot */
		ivc_tx.engine = VIRTUAL_SE_AES1;
		ivc_tx.cmd = VIRTUAL_SE_CMD_AES_ALLOC_KEY;

		err = tegra_hv_vse_send_ivc(se_dev, pivck, &ivc_tx,
				sizeof(ivc_tx));
		if (err)
			return err;

		err = tegra_hv_vse_read_ivc(se_dev, pivck, &ivc_rx,
				sizeof(ivc_rx));
		if (err)
			return err;

		if (ivc_rx.status) {
			dev_err(se_dev->dev, "Error from Read IVC %d\n",
				ivc_rx.status);
			return ivc_rx.status;
		}
		ctx->aes_keyslot = ivc_rx.keyslot;
		ctx->is_key_slot_allocated = true;
	}

	ctx->keylen = keylen;
	err = tegra_hv_vse_aes_set_keyiv(se_dev, (u8 *)key, keylen,
			ctx->aes_keyslot, AES_KEYTBL_TYPE_KEY);

	return err;
}

static struct crypto_alg aes_algs[] = {
	{
		.cra_name = "cbc(aes)",
		.cra_driver_name = "cbc-aes-tegra",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = VIRTUAL_SE_AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct tegra_virtual_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_hv_vse_aes_cra_init,
		.cra_exit = tegra_hv_vse_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = VIRTUAL_SE_AES_MIN_KEY_SIZE,
			.max_keysize = VIRTUAL_SE_AES_MAX_KEY_SIZE,
			.ivsize = VIRTUAL_SE_AES_IV_SIZE,
			.setkey = tegra_hv_vse_aes_setkey,
			.encrypt = tegra_hv_vse_aes_cbc_encrypt,
			.decrypt = tegra_hv_vse_aes_cbc_decrypt,
		}
	}, {
		.cra_name = "ecb(aes)",
		.cra_driver_name = "ecb-aes-tegra",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = VIRTUAL_SE_AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct tegra_virtual_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_hv_vse_aes_cra_init,
		.cra_exit = tegra_hv_vse_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = VIRTUAL_SE_AES_MIN_KEY_SIZE,
			.max_keysize = VIRTUAL_SE_AES_MAX_KEY_SIZE,
			.ivsize = VIRTUAL_SE_AES_IV_SIZE,
			.setkey = tegra_hv_vse_aes_setkey,
			.encrypt = tegra_hv_vse_aes_ecb_encrypt,
			.decrypt = tegra_hv_vse_aes_ecb_decrypt,
		}
	}, {
		.cra_name = "ctr(aes)",
		.cra_driver_name = "ctr-aes-tegra",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = VIRTUAL_SE_AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct tegra_virtual_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_hv_vse_aes_cra_init,
		.cra_exit = tegra_hv_vse_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = VIRTUAL_SE_AES_MIN_KEY_SIZE,
			.max_keysize = VIRTUAL_SE_AES_MAX_KEY_SIZE,
			.ivsize = VIRTUAL_SE_AES_IV_SIZE,
			.setkey = tegra_hv_vse_aes_setkey,
			.encrypt = tegra_hv_vse_aes_ctr_encrypt,
			.decrypt = tegra_hv_vse_aes_ctr_decrypt,
			.geniv = "eseqiv",
		}
	}, {
		.cra_name = "ofb(aes)",
		.cra_driver_name = "ofb-aes-tegra",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize = VIRTUAL_SE_AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct tegra_virtual_se_aes_context),
		.cra_alignmask = 0,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_hv_vse_aes_cra_init,
		.cra_exit = tegra_hv_vse_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = VIRTUAL_SE_AES_MIN_KEY_SIZE,
			.max_keysize = VIRTUAL_SE_AES_MAX_KEY_SIZE,
			.ivsize = VIRTUAL_SE_AES_IV_SIZE,
			.setkey = tegra_hv_vse_aes_setkey,
			.encrypt = tegra_hv_vse_aes_ofb_encrypt,
			.decrypt = tegra_hv_vse_aes_ofb_decrypt,
				.geniv = "eseqiv",
		}
	},
};

static struct ahash_alg cmac_alg = {
	.init = tegra_hv_vse_cmac_init,
	.update = tegra_hv_vse_cmac_update,
	.final = tegra_hv_vse_cmac_final,
	.finup = tegra_hv_vse_cmac_finup,
	.digest = tegra_hv_vse_cmac_digest,
	.setkey = tegra_hv_vse_cmac_setkey,
	.halg.digestsize = TEGRA_VIRUTAL_SE_AES_CMAC_DIGEST_SIZE,
	.halg.base = {
		.cra_name = "cmac(aes)",
		.cra_driver_name = "tegra-hv-vse-cmac(aes)",
		.cra_priority = 300,
		.cra_flags = CRYPTO_ALG_TYPE_AHASH,
		.cra_blocksize = TEGRA_VIRTUAL_SE_AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct tegra_virtual_se_aes_cmac_context),
		.cra_alignmask = 0,
		.cra_module = THIS_MODULE,
		.cra_init = tegra_hv_vse_cmac_cra_init,
		.cra_exit = tegra_hv_vse_cmac_cra_exit,
	}
};

static struct ahash_alg sha_algs[] = {
	{
		.init =  tegra_hv_vse_sha_init,
		.update =  tegra_hv_vse_sha_update,
		.final =  tegra_hv_vse_sha_final,
		.finup =  tegra_hv_vse_sha_finup,
		.digest =  tegra_hv_vse_sha_digest,
		.halg.digestsize = SHA1_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "sha1",
			.cra_driver_name = "tegra-hv-vse-sha1",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA1_BLOCK_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init =  tegra_hv_vse_sha_cra_init,
			.cra_exit =  tegra_hv_vse_sha_cra_exit,
		}
	}, {
		.init =  tegra_hv_vse_sha_init,
		.update =  tegra_hv_vse_sha_update,
		.final =  tegra_hv_vse_sha_final,
		.finup =  tegra_hv_vse_sha_finup,
		.digest =  tegra_hv_vse_sha_digest,
		.halg.digestsize = SHA224_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "sha224",
			.cra_driver_name = "tegra-hv-vse-sha224",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA224_BLOCK_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init =  tegra_hv_vse_sha_cra_init,
			.cra_exit =  tegra_hv_vse_sha_cra_exit,
		}
	}, {
		.init =  tegra_hv_vse_sha_init,
		.update =  tegra_hv_vse_sha_update,
		.final =  tegra_hv_vse_sha_final,
		.finup =  tegra_hv_vse_sha_finup,
		.digest =  tegra_hv_vse_sha_digest,
		.halg.digestsize = SHA256_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "sha256",
			.cra_driver_name = "tegra-hv-vse-sha256",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA256_BLOCK_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init =  tegra_hv_vse_sha_cra_init,
			.cra_exit =  tegra_hv_vse_sha_cra_exit,
		}
	}, {
		.init =  tegra_hv_vse_sha_init,
		.update =  tegra_hv_vse_sha_update,
		.final =  tegra_hv_vse_sha_final,
		.finup =  tegra_hv_vse_sha_finup,
		.digest =  tegra_hv_vse_sha_digest,
		.halg.digestsize = SHA384_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "sha384",
			.cra_driver_name = "tegra-hv-vse-sha384",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA384_BLOCK_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init =  tegra_hv_vse_sha_cra_init,
			.cra_exit =  tegra_hv_vse_sha_cra_exit,
		}
	}, {
		.init =  tegra_hv_vse_sha_init,
		.update =  tegra_hv_vse_sha_update,
		.final =  tegra_hv_vse_sha_final,
		.finup =  tegra_hv_vse_sha_finup,
		.digest =  tegra_hv_vse_sha_digest,
		.halg.digestsize = SHA512_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "sha512",
			.cra_driver_name = "tegra-hv-vse-sha512",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = SHA512_BLOCK_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_sha_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init =  tegra_hv_vse_sha_cra_init,
			.cra_exit =  tegra_hv_vse_sha_cra_exit,
		}
	},
};

static struct ahash_alg rsa_algs[] = {
	{
		.init = tegra_hv_vse_rsa_init,
		.update = tegra_hv_vse_rsa_update,
		.final = tegra_hv_vse_rsa_final,
		.finup = tegra_hv_vse_rsa_finup,
		.digest = tegra_hv_vse_rsa_digest,
		.setkey = tegra_hv_vse_rsa_setkey,
		.halg.digestsize = TEGRA_VIRTUAL_SE_RSA512_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "rsa512",
			.cra_driver_name = "tegra-hv-vse-rsa512",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = TEGRA_VIRTUAL_SE_RSA512_DIGEST_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_rsa_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_hv_vse_rsa_cra_init,
			.cra_exit = tegra_hv_vse_rsa_cra_exit,
		}
	}, {
		.init = tegra_hv_vse_rsa_init,
		.update = tegra_hv_vse_rsa_update,
		.final = tegra_hv_vse_rsa_final,
		.finup = tegra_hv_vse_rsa_finup,
		.digest = tegra_hv_vse_rsa_digest,
		.setkey = tegra_hv_vse_rsa_setkey,
		.halg.digestsize = TEGRA_VIRTUAL_SE_RSA1024_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "rsa1024",
			.cra_driver_name = "tegra-hv-vse-rsa1024",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = TEGRA_VIRTUAL_SE_RSA1024_DIGEST_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_rsa_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_hv_vse_rsa_cra_init,
			.cra_exit = tegra_hv_vse_rsa_cra_exit,
		}
	}, {
		.init = tegra_hv_vse_rsa_init,
		.update = tegra_hv_vse_rsa_update,
		.final = tegra_hv_vse_rsa_final,
		.finup = tegra_hv_vse_rsa_finup,
		.digest = tegra_hv_vse_rsa_digest,
		.setkey = tegra_hv_vse_rsa_setkey,
		.halg.digestsize = TEGRA_VIRTUAL_SE_RSA1536_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "rsa1536",
			.cra_driver_name = "tegra-hv-vse-rsa1536",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = TEGRA_VIRTUAL_SE_RSA1536_DIGEST_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_rsa_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_hv_vse_rsa_cra_init,
			.cra_exit = tegra_hv_vse_rsa_cra_exit,
		}
	}, {
		.init = tegra_hv_vse_rsa_init,
		.update = tegra_hv_vse_rsa_update,
		.final = tegra_hv_vse_rsa_final,
		.finup = tegra_hv_vse_rsa_finup,
		.digest = tegra_hv_vse_rsa_digest,
		.setkey = tegra_hv_vse_rsa_setkey,
		.halg.digestsize = TEGRA_VIRTUAL_SE_RSA2048_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "rsa2048",
			.cra_driver_name = "tegra-hv-vse-rsa2048",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH,
			.cra_blocksize = TEGRA_VIRTUAL_SE_RSA2048_DIGEST_SIZE,
			.cra_ctxsize =
				sizeof(struct tegra_virtual_se_rsa_context),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = tegra_hv_vse_rsa_cra_init,
			.cra_exit = tegra_hv_vse_rsa_cra_exit,
		}
	}
};

static struct of_device_id tegra_hv_vse_of_match[] = {
	{
		.compatible = "nvidia,tegra186-hv-vse",
	},
};
MODULE_DEVICE_TABLE(of, tegra_hv_vse_of_match);

static int tegra_hv_vse_probe(struct platform_device *pdev)
{
	struct tegra_virtual_se_dev *se_dev = NULL;
	int err = 0;
	int i;
	unsigned int ivc_id;
	unsigned int engine_id;

	se_dev = devm_kzalloc(&pdev->dev,
				sizeof(struct tegra_virtual_se_dev),
				GFP_KERNEL);
	if (!se_dev)
		return -ENOMEM;

	se_dev->dev = &pdev->dev;
	err = of_property_read_u32(pdev->dev.of_node, "se-engine-id",
				   &engine_id);
	if (err) {
		dev_err(&pdev->dev, "se-engine-id property not present\n");
		err = -ENODEV;
		goto exit;
	}
	se_dev->stream_id = iommu_get_hwid(pdev->dev.archdata.iommu,
			&pdev->dev, 0);
	dev_info(se_dev->dev, "Virtual SE Stream ID: %d", se_dev->stream_id);

	if (!g_ivck) {
		err = of_property_read_u32(pdev->dev.of_node, "ivc", &ivc_id);
		if (err) {
			dev_err(&pdev->dev, "ivc property not present\n");
			err = -ENODEV;
			goto exit;
		}
		dev_info(se_dev->dev, "Virtual SE channel number: %d", ivc_id);

		g_ivck = tegra_hv_ivc_reserve(NULL, ivc_id, NULL);
		if (IS_ERR_OR_NULL(g_ivck)) {
			dev_err(&pdev->dev, "Failed reserve channel number\n");
			err = -ENODEV;
			goto exit;
		}
		tegra_hv_ivc_channel_reset(g_ivck);
	}
	g_virtual_se_dev[engine_id] = se_dev;

	if (engine_id == VIRTUAL_SE_AES1) {
		INIT_WORK(&se_dev->se_work, tegra_hv_vse_work_handler);
		crypto_init_queue(&se_dev->queue,
			TEGRA_HV_VSE_CRYPTO_QUEUE_LENGTH);
		spin_lock_init(&se_dev->lock);
		mutex_init(&se_dev->mtx);
		se_dev->vse_work_q = alloc_workqueue("vse_work_q",
					WQ_HIGHPRI | WQ_UNBOUND, 16);
		if (!se_dev->vse_work_q) {
			err = -ENOMEM;
			dev_err(se_dev->dev, "alloc_workqueue failed\n");
			goto exit;
		}
		for (i = 0; i < ARRAY_SIZE(aes_algs); i++) {
			err = crypto_register_alg(&aes_algs[i]);
			if (err) {
				dev_err(&pdev->dev,
					"aes alg register failed idx[%d]\n", i);
				goto exit;
			}
		}

		err = crypto_register_ahash(&cmac_alg);
		if (err) {
			dev_err(&pdev->dev,
				"cmac alg register failed. Err %d\n", err);
			goto exit;
		}
	}

	if (engine_id == VIRTUAL_SE_SHA) {
		for (i = 0; i < ARRAY_SIZE(sha_algs); i++) {
			err = crypto_register_ahash(&sha_algs[i]);
			if (err) {
				dev_err(&pdev->dev,
					"sha alg register failed idx[%d]\n", i);
				goto exit;
			}
		}
	}

	if (engine_id == VIRTUAL_SE_RSA) {
		for (i = 0; i < ARRAY_SIZE(rsa_algs); i++) {
			err = crypto_register_ahash(&rsa_algs[i]);
			if (err) {
				dev_err(&pdev->dev,
					"RSA alg register failed idx[%d]\n", i);
				goto exit;
			}
		}
	}

	return 0;

exit:
	return err;
}

static int tegra_hv_vse_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sha_algs); i++)
		crypto_unregister_ahash(&sha_algs[i]);

	for (i = 0; i < ARRAY_SIZE(rsa_algs); i++)
		crypto_unregister_ahash(&rsa_algs[i]);

	return 0;
}

static struct platform_driver tegra_hv_vse_driver = {
	.probe  = tegra_hv_vse_probe,
	.remove = tegra_hv_vse_remove,
	.driver = {
		.name   = "tegra_hv_vse",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_hv_vse_of_match),
	},
};

static int __init tegra_hv_vse_module_init(void)
{
	return  platform_driver_register(&tegra_hv_vse_driver);
}

static void __exit tegra_hv_vse_module_exit(void)
{
	platform_driver_unregister(&tegra_hv_vse_driver);
}

module_init(tegra_hv_vse_module_init);
module_exit(tegra_hv_vse_module_exit);

MODULE_AUTHOR("Mallikarjun Kasoju <mkasoju@nvidia.com>");
MODULE_DESCRIPTION("Virtual Security Engine driver over Tegra Hypervisor IVC channel");
MODULE_LICENSE("GPL");
