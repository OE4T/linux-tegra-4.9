/*
 * Driver for Tegra Security Engine
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#ifndef _CRYPTO_TEGRA_SE_H
#define _CRYPTO_TEGRA_SE_H

#include <crypto/hash.h>
#include <crypto/sha.h>

#define PFX	"tegra-se: "

#define TEGRA_SE_CRA_PRIORITY	300
#define TEGRA_SE_COMPOSITE_PRIORITY 400
#define TEGRA_SE_CRYPTO_QUEUE_LENGTH 50
#define SE_MAX_SRC_SG_COUNT		50
#define SE_MAX_DST_SG_COUNT		50

#define TEGRA_SE_KEYSLOT_COUNT		16

/* SE register definitions */
#define SE_CONFIG_REG_OFFSET		0x014
#define SE_CONFIG_ENC_ALG_SHIFT		12
#define SE_CONFIG_DEC_ALG_SHIFT		8
#define ALG_AES_ENC		1
#define ALG_RNG			2
#define ALG_SHA			3
#define ALG_AES_DEC		1
#define SE_CONFIG_ENC_ALG(x)		(x << SE_CONFIG_ENC_ALG_SHIFT)
#define SE_CONFIG_DEC_ALG(x)		(x << SE_CONFIG_DEC_ALG_SHIFT)
#define SE_CONFIG_DST_SHIFT			2
#define DST_MEMORY		0
#define DST_HASHREG		1
#define DST_KEYTAB		2
#define DST_SRK			3
#define SE_CONFIG_DST(x)			(x << SE_CONFIG_DST_SHIFT)
#define SE_CONFIG_ENC_MODE_SHIFT	24
#define SE_CONFIG_DEC_MODE_SHIFT	16
#define MODE_KEY128		0
#define MODE_KEY192		1
#define MODE_KEY256		2
#define MODE_SHA1		0
#define MODE_SHA224		4
#define MODE_SHA256		5
#define MODE_SHA384		6
#define MODE_SHA512		7
#define SE_CONFIG_ENC_MODE(x)		(x << SE_CONFIG_ENC_MODE_SHIFT)
#define SE_CONFIG_DEC_MODE(x)		(x << SE_CONFIG_DEC_MODE_SHIFT)

#define SE_KEYTABLE_REG_OFFSET		0x31c
#define SE_KEYTABLE_SLOT_SHIFT		4
#define SE_KEYTABLE_SLOT(x)			(x << SE_KEYTABLE_SLOT_SHIFT)
#define SE_KEYTABLE_QUAD_SHIFT		2
#define QUAD_KEYS_128	0
#define QUAD_KEYS_192	1
#define QUAD_KEYS_256	1
#define QUAD_ORG_IV		2
#define QUAD_UPDTD_IV	3
#define SE_KEYTABLE_QUAD(x)			(x << SE_KEYTABLE_QUAD_SHIFT)
#define SE_KEYTABLE_OP_TYPE_SHIFT	9
#define OP_READ			0
#define OP_WRITE		1
#define SE_KEYTABLE_OP_TYPE(x)		(x << SE_KEYTABLE_OP_TYPE_SHIFT)
#define SE_KEYTABLE_TABLE_SEL_SHIFT		8
#define TABLE_KEYIV		0
#define TABLE_SCHEDULE	1
#define SE_KEYTABLE_TABLE_SEL(x)	(x << SE_KEYTABLE_TABLE_SEL_SHIFT)
#define SE_KEYTABLE_PKT_SHIFT		0
#define SE_KEYTABLE_PKT(x)			(x << SE_KEYTABLE_PKT_SHIFT)

#define SE_CRYPTO_REG_OFFSET		0x304
#define SE_CRYPTO_HASH_SHIFT		0
#define HASH_DISABLE	0
#define HASH_ENABLE		1
#define SE_CRYPTO_HASH(x)			(x << SE_CRYPTO_HASH_SHIFT)
#define SE_CRYPTO_XOR_POS_SHIFT		1
#define XOR_BYPASS		0
#define XOR_TOP			2
#define XOR_BOTTOM		3
#define SE_CRYPTO_XOR_POS(x)		(x << SE_CRYPTO_XOR_POS_SHIFT)
#define SE_CRYPTO_INPUT_SEL_SHIFT	3
#define INPUT_AHB		0
#define INPUT_LFSR		1
#define INPUT_AESOUT	2
#define INPUT_LNR_CTR	3
#define SE_CRYPTO_INPUT_SEL(x)		(x << SE_CRYPTO_INPUT_SEL_SHIFT)
#define SE_CRYPTO_VCTRAM_SEL_SHIFT	5
#define VCTRAM_AHB		0
#define VCTRAM_AESOUT	2
#define VCTRAM_PREVAHB	3
#define SE_CRYPTO_VCTRAM_SEL(x)		(x << SE_CRYPTO_VCTRAM_SEL_SHIFT)
#define SE_CRYPTO_IV_SEL_SHIFT		7
#define IV_ORIGINAL		0
#define IV_UPDATED		1
#define SE_CRYPTO_IV_SEL(x)			(x << SE_CRYPTO_IV_SEL_SHIFT)
#define SE_CRYPTO_CORE_SEL_SHIFT	8
#define CORE_DECRYPT	0
#define CORE_ENCRYPT	1
#define SE_CRYPTO_CORE_SEL(x)		(x << SE_CRYPTO_CORE_SEL_SHIFT)
#define SE_CRYPTO_CTR_VAL_SHIFT		11
#define SE_CRYPTO_CTR_VAL(x)		(x << SE_CRYPTO_CTR_VAL_SHIFT)
#define SE_CRYPTO_KEY_INDEX_SHIFT	24
#define SE_CRYPTO_KEY_INDEX(x)		(x << SE_CRYPTO_KEY_INDEX_SHIFT)
#define SE_CRYPTO_CTR_CNTN_SHIFT	11
#define SE_CRYPTO_CTR_CNTN(x)		(x << SE_CRYPTO_CTR_CNTN_SHIFT)

#define SE_CRYPTO_CTR_REG_COUNT		4
#define SE_CRYPTO_CTR_REG_OFFSET	0x308

#define SE_OPERATION_REG_OFFSET		0x008
#define SE_OPERATION_SHIFT			0
#define OP_ABORT		0
#define OP_SRART		1
#define OP_RESTART		2
#define OP_CTX_SAVE		3
#define SE_OPERATION(x)				(x << SE_OPERATION_SHIFT)


#define SE_INT_ENABLE_REG_OFFSET	0x00c
#define SE_INT_STATUS_REG_OFFSET	0x010
#define INT_DISABLE		0
#define INT_ENABLE		1
#define INT_UNSET		0
#define INT_SET			1
#define SE_INT_OP_DONE_SHIFT		4
#define SE_INT_OP_DONE(x)			(x << SE_INT_OP_DONE_SHIFT)
#define SE_INT_ERROR_SHIFT		16
#define SE_INT_ERROR(x)			(x << SE_INT_ERROR_SHIFT)


#define SE_IN_LL_ADDR_REG_OFFSET	0x018
#define SE_OUT_LL_ADDR_REG_OFFSET	0x024

#define SE_KEYTABLE_DATA0_REG_OFFSET	0x320
#define SE_KEYTABLE_REG_MAX_DATA		16

#define SE_BLOCK_COUNT_REG_OFFSET	0x318

#define SE_SPARE_0_REG_OFFSET		0x80c

#define SE_SHA_CONFIG_REG_OFFSET	0x200
#define SHA_DISABLE		0
#define SHA_ENABLE		1

#define SE_SHA_MSG_LENGTH_REG_OFFSET	0x204
#define SE_SHA_MSG_LEFT_REG_OFFSET		0x214


#define SE_HASH_RESULT_REG_COUNT	16
#define SE_HASH_RESULT_REG_OFFSET	0x030


#define TEGRA_SE_KEY_256_SIZE		32
#define TEGRA_SE_KEY_192_SIZE		24
#define TEGRA_SE_KEY_128_SIZE		16
#define TEGRA_SE_AES_BLOCK_SIZE		16
#define TEGRA_SE_AES_MIN_KEY_SIZE	16
#define TEGRA_SE_AES_MAX_KEY_SIZE	32
#define TEGRA_SE_AES_IV_SIZE		16
#define TEGRA_SE_RNG_IV_SIZE		16
#define TEGRA_SE_RNG_DT_SIZE		16
#define TEGRA_SE_RNG_KEY_SIZE		16
#define TEGRA_SE_RNG_SEED_SIZE		(TEGRA_SE_RNG_IV_SIZE + \
						TEGRA_SE_RNG_KEY_SIZE + \
						TEGRA_SE_RNG_DT_SIZE)
#define TEGRA_SE_AES_CMAC_DIGEST_SIZE	16


/* Security Engine operation modes */
enum tegra_se_aes_op_mode {
	SE_AES_OP_MODE_CBC,	/* Cipher Block Chaining (CBC) mode */
	SE_AES_OP_MODE_ECB,	/* Electronic Codebook (ECB) mode */
	SE_AES_OP_MODE_CTR,	/* Counter (CTR) mode */
	SE_AES_OP_MODE_OFB,	/* Output feedback (CFB) mode */
	SE_AES_OP_MODE_RNG_X931,	/* Random number generator (RNG) mode */
	SE_AES_OP_MODE_CMAC,	/* Cipher-based MAC (CMAC) mode */
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

/* Security Engine request context */
struct tegra_se_req_context {
	enum tegra_se_aes_op_mode op_mode; /* Security Engine operation mode */
	bool encrypt;	/* Operation type */
};

/* Security Engine AES context */
struct tegra_se_aes_context {
	struct tegra_se_dev *se_dev;	/* Security Engine device */
	struct tegra_se_slot *slot;	/* Security Engine key slot */
	u32 keylen;	/* key length in bits */
	u32 op_mode;	/* AES operation mode */
};

/* Security Engine randon number generator context */
struct tegra_se_rng_context {
	struct tegra_se_dev *se_dev;	/* Security Engine device */
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

/* Security Engine Linked List */
struct tegra_se_ll {
	dma_addr_t addr; /* DMA buffer address */
	u32 data_len; /* Data length in DMA buffer */
};

/* Security Engine device */
struct tegra_se_dev {
	struct device *dev;
	void __iomem *io_reg;	/* device memory/io */
	int			irq;		/* irq allocated */
	spinlock_t lock;	/* spin lock */
	struct clk *pclk;	/* Security Engine clock */
	struct crypto_queue queue; /* Security Engine crypto queue */
	struct tegra_se_slot *slot_list;	/* pointer to key slots */
	u32 *src_ll_buf;	/* pointer to source linked list buffer */
	dma_addr_t src_ll_buf_adr; /* Source linked list buffer dma address */
	u32 src_ll_size;	/* Size of source linked list buffer */
	u32 *dst_ll_buf;	/* pointer to destination linked list buffer */
	dma_addr_t dst_ll_buf_adr; /* Destination linked list dma address */
	u32 dst_ll_size;	/* Size of destination linked list buffer */
	struct completion complete;	/* Tells the task completion */
	bool work_q_busy;/* Work queue busy status */
};

#endif /* _CRYPTO_TEGRA_SE_H */
