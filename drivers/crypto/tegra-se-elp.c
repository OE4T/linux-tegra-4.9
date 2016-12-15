/*
 * Cryptographic API.
 * drivers/crypto/tegra-se-elp.c
 *
 * Support for Tegra Security Engine Elliptic crypto algorithms.
 *
 * Copyright (c) 2015-2016, NVIDIA Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation, and may be copied,
 * distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/clk/tegra.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/tegra-soc.h>
#include <crypto/internal/rng.h>
#include <crypto/internal/kpp.h>
#include <crypto/kpp.h>
#include <linux/fips.h>
#include <crypto/ecdh.h>

#include "tegra-se-elp.h"

#define DRIVER_NAME	"tegra-se-elp"

#define NR_RES	2
#define PKA1	0
#define RNG1	1

#define TEGRA_SE_MUTEX_WDT_UNITS	0x600000
#define RNG1_TIMEOUT			2000	/*micro seconds*/
#define PKA1_TIMEOUT			40000	/*micro seconds*/
#define RAND_128			16	/*bytes*/
#define RAND_256			32	/*bytes*/
#define ADV_STATE_FREQ			3
#define ECC_MAX_WORDS	20
#define WORD_SIZE_BYTES	4

enum tegra_se_pka_rsa_type {
	RSA_EXP_MOD,
	RSA_CRT_KEY_SETUP,
	RSA_CRT,
};

enum tegra_se_pka_ecc_type {
	ECC_POINT_MUL,
	ECC_POINT_ADD,
	ECC_POINT_DOUBLE,
	ECC_POINT_VER,
	ECC_SHAMIR_TRICK,
};

enum tegra_se_elp_precomp_vals {
	PRECOMP_RINV,
	PRECOMP_M,
	PRECOMP_R2,
};

enum tegra_se_elp_rng1_cmd {
	RNG1_CMD_NOP,
	RNG1_CMD_GEN_NOISE,
	RNG1_CMD_GEN_NONCE,
	RNG1_CMD_CREATE_STATE,
	RNG1_CMD_RENEW_STATE,
	RNG1_CMD_REFRESH_ADDIN,
	RNG1_CMD_GEN_RANDOM,
	RNG1_CMD_ADVANCE_STATE,
	RNG1_CMD_KAT,
	RNG1_CMD_ZEROIZE = 15,
};

static char *rng1_cmd[] = {
	"RNG1_CMD_NOP",
	"RNG1_CMD_GEN_NOISE",
	"RNG1_CMD_GEN_NONCE",
	"RNG1_CMD_CREATE_STATE",
	"RNG1_CMD_RENEW_STATE",
	"RNG1_CMD_REFRESH_ADDIN",
	"RNG1_CMD_GEN_RANDOM",
	"RNG1_CMD_ADVANCE_STATE",
	"RNG1_CMD_KAT",
	"RNG1_CMD_ZEROIZE"
};

struct tegra_se_chipdata {
	bool use_key_slot;
};

struct tegra_se_elp_dev {
	struct device *dev;
	void __iomem *io_reg[2];
	struct clk *c;
	struct tegra_se_slot *slot_list;
	const struct tegra_se_chipdata *chipdata;
	struct tegra_se_elp_pka_request *pka_req;
	u32 *rdata;
};

/* TODO: Planning to remove global elp_dev once crypto framework
 * APIs are re-routed to this driver from a context
 */
static struct tegra_se_elp_dev *elp_dev;

struct tegra_se_elp_rng_request {
	int size;
	u32 *rdata;
	u32 *rdata1;
	u32 *rdata2;
	u32 *rdata3;
	bool test_full_cmd_flow;
	bool adv_state_on;
};

struct tegra_se_elp_pka_request {
	struct tegra_se_elp_dev *se_dev;
	struct tegra_se_slot *slot;
	u32 *message;
	u32 *result;
	u32 *exponent;
	u32 *modulus;
	u32 *m;
	u32 *r2;
	u32 *rinv;
	int op_mode;
	int size;
	int ecc_type;
	int rsa_type;
	u32 *curve_param_a;
	u32 *curve_param_b;
	u32 *order;
	u32 *base_pt_x;
	u32 *base_pt_y;
	u32 *res_pt_x;
	u32 *res_pt_y;
	u32 *key;
	bool pv_ok;
};

/* Security Engine key slot */
struct tegra_se_slot {
	struct list_head node;
	u8 slot_num;	/* Key slot number */
	atomic_t available; /* Tells whether key slot is free to use */
};

static LIST_HEAD(key_slot);

static u32 pka_op_size[] = {512, 768, 1024, 1536, 2048, 3072, 4096, 160, 192,
				224, 256, 384, 512, 640};

struct tegra_se_ecdh_context {
	struct tegra_se_elp_dev *se_dev;
	unsigned int curve_id;
	u32 private_key[ECC_MAX_WORDS];
	u32 public_key[2 * ECC_MAX_WORDS];
	u32 shared_secret[ECC_MAX_WORDS];
};

static struct tegra_se_ecc_point *tegra_se_ecc_alloc_point(
				struct tegra_se_elp_dev *se_dev, int nwords)
{
	struct tegra_se_ecc_point *p = devm_kzalloc(se_dev->dev,
					sizeof(struct tegra_se_ecc_point),
					GFP_KERNEL);
	int len = nwords * WORD_SIZE_BYTES;

	if (!p)
		return NULL;

	p->x = devm_kzalloc(se_dev->dev, len, GFP_KERNEL);
	if (!p->x)
		goto free_pt;

	p->y = devm_kzalloc(se_dev->dev, len, GFP_KERNEL);
	if (!p->y)
		goto exit;

	return p;
exit:
	devm_kfree(se_dev->dev, p->x);
free_pt:
	devm_kfree(se_dev->dev, p);
	return NULL;
}

static void tegra_se_ecc_free_point(struct tegra_se_elp_dev *se_dev,
				    struct tegra_se_ecc_point *p)
{
	if (!p)
		return;

	devm_kfree(se_dev->dev, p->x);
	devm_kfree(se_dev->dev, p->y);
	devm_kfree(se_dev->dev, p);
}

const struct tegra_se_ecc_curve *tegra_se_ecc_get_curve(unsigned int curve_id)
{
	switch (curve_id) {
	/* In FIPS mode only allow P256 and higher */
	case ECC_CURVE_NIST_P192:
		return fips_enabled ? NULL : &curve_p192;
	case ECC_CURVE_NIST_P224:
		return fips_enabled ? NULL : &curve_p224;
	case ECC_CURVE_NIST_P256:
		return &curve_p256;
	case ECC_CURVE_NIST_P384:
		return &curve_p384;
	case ECC_CURVE_NIST_P521:
		return &curve_p521;
	default:
		return NULL;
	}
}

static inline void tegra_se_ecc_swap(const u32 *in, u32 *out, int nwords)
{
	int i;

	for (i = 0; i < nwords; i++)
		out[i] = swab32(in[nwords - 1 - i]);
}

static bool tegra_se_ecc_vec_is_zero(const u32 *vec, int nbytes)
{
	unsigned int zerobuf[ECC_MAX_WORDS * WORD_SIZE_BYTES] = {0};

	return !memcmp((u8 *)vec, zerobuf, nbytes);
}

/* Returns true if vec1 > vec2 */
static bool tegra_se_ecc_vec_cmp(const u8 *vec1, const u8 *vec2,
				 unsigned int nbytes)
{
	int i;

	for (i = nbytes - 1; i >= 0; i--) {
		if (vec1[i] > vec2[i])
			return true;
		else if (vec1[i] < vec2[i])
			return false;
	}

	return false;
}

static bool tegra_se_ecdh_params_is_valid(struct ecdh *params)
{
	const u32 *private_key = (const u32 *)params->key;
	int private_key_len = params->key_size;
	const struct tegra_se_ecc_curve *curve = tegra_se_ecc_get_curve(
							params->curve_id);
	const u32 *order = curve->n;
	int nbytes = curve->nbytes;

	if (!nbytes || !private_key)
		return false;

	if (private_key_len != nbytes)
		return false;

	if (tegra_se_ecc_vec_is_zero(private_key, nbytes))
		return false;

	/* Make sure the private key is in the range [1, n-1]. */
	if (!tegra_se_ecc_vec_cmp((u8 *)order, (u8 *)private_key, nbytes))
		return false;

	return true;
}

static int tegra_se_ecdh_set_params(struct tegra_se_ecdh_context *ctx,
				    struct ecdh *params)
{
	if (!tegra_se_ecdh_params_is_valid(params))
		return -EINVAL;

	ctx->curve_id = params->curve_id;

	memcpy(ctx->private_key, params->key, params->key_size);

	return 0;
}

static inline u32 num_words(int mode)
{
	u32 words = 0;

	switch (mode) {
	case SE_ELP_OP_MODE_ECC160:
	case SE_ELP_OP_MODE_ECC192:
	case SE_ELP_OP_MODE_ECC224:
	case SE_ELP_OP_MODE_ECC256:
		words = pka_op_size[SE_ELP_OP_MODE_ECC256] / 32;
		break;
	case SE_ELP_OP_MODE_RSA512:
	case SE_ELP_OP_MODE_ECC384:
	case SE_ELP_OP_MODE_ECC512:
		words = pka_op_size[SE_ELP_OP_MODE_RSA512] / 32;
		break;
	case SE_ELP_OP_MODE_RSA768:
	case SE_ELP_OP_MODE_RSA1024:
	case SE_ELP_OP_MODE_ECC521:
		words = pka_op_size[SE_ELP_OP_MODE_RSA1024] / 32;
		break;
	case SE_ELP_OP_MODE_RSA1536:
	case SE_ELP_OP_MODE_RSA2048:
		words = pka_op_size[SE_ELP_OP_MODE_RSA2048] / 32;
		break;
	case SE_ELP_OP_MODE_RSA3072:
	case SE_ELP_OP_MODE_RSA4096:
		words = pka_op_size[SE_ELP_OP_MODE_RSA4096] / 32;
		break;
	default:
		dev_warn(elp_dev->dev, "Invalid operation mode\n");
		break;
	}

	return words;
}

static inline void se_elp_writel(struct tegra_se_elp_dev *se_dev, int elp_type,
				 unsigned int val, unsigned int reg_offset)
{
	writel(val, se_dev->io_reg[elp_type] + reg_offset);
}

static inline unsigned int se_elp_readl(struct tegra_se_elp_dev *se_dev,
					int elp_type, unsigned int reg_offset)
{
	return readl(se_dev->io_reg[elp_type] + reg_offset);
}

static void tegra_se_pka_free_key_slot(struct tegra_se_slot *slot)
{
	if (!slot)
		return;

	atomic_set(&slot->available, 1);
}

static struct tegra_se_slot *tegra_se_pka_alloc_key_slot(void)
{
	struct tegra_se_slot *slot;
	bool found = false;

	list_for_each_entry(slot, &key_slot, node) {
		if (atomic_read(&slot->available)) {
			atomic_set(&slot->available, 0);
			found = true;
			break;
		}
	}

	return found ? slot : NULL;
}

static int tegra_se_pka_init_key_slot(struct tegra_se_elp_dev *se_dev)
{
	int i;

	se_dev->slot_list = devm_kzalloc(se_dev->dev,
					 sizeof(struct tegra_se_slot) *
					 TEGRA_SE_PKA_KEYSLOT_COUNT,
					 GFP_KERNEL);
	if (!se_dev->slot_list)
		return -ENOMEM;

	for (i = 0; i < TEGRA_SE_PKA_KEYSLOT_COUNT; i++) {
		atomic_set(&se_dev->slot_list[i].available, 1);
		se_dev->slot_list[i].slot_num = i;
		INIT_LIST_HEAD(&se_dev->slot_list[i].node);
		list_add_tail(&se_dev->slot_list[i].node, &key_slot);
	}

	return 0;
}

static u32 tegra_se_check_trng_op(struct tegra_se_elp_dev *se_dev)
{
	u32 trng_val;
	u32 val = se_elp_readl(se_dev, PKA1,
			       TEGRA_SE_ELP_PKA_TRNG_STATUS_OFFSET);

	trng_val = TEGRA_SE_ELP_PKA_TRNG_STATUS_SECURE(ELP_TRUE) |
			TEGRA_SE_ELP_PKA_TRNG_STATUS_NONCE(ELP_FALSE) |
			TEGRA_SE_ELP_PKA_TRNG_STATUS_SEEDED(ELP_TRUE) |
			TEGRA_SE_ELP_PKA_TRNG_STATUS_LAST_RESEED(
						TRNG_LAST_RESEED_HOST);
	if ((val & trng_val) ||
	    (val & TEGRA_SE_ELP_PKA_TRNG_STATUS_LAST_RESEED
					(TRNG_LAST_RESEED_RESEED)))
		return 0;

	return -EINVAL;
}

static u32 tegra_se_set_trng_op(struct tegra_se_elp_dev *se_dev)
{
	u32 val, i = 0;

	se_elp_writel(se_dev, PKA1,
		      TEGRA_SE_ELP_PKA_TRNG_SMODE_SECURE(ELP_ENABLE) |
		      TEGRA_SE_ELP_PKA_TRNG_SMODE_NONCE(ELP_DISABLE),
		      TEGRA_SE_ELP_PKA_TRNG_SMODE_OFFSET);
	se_elp_writel(se_dev, PKA1,
		      TEGRA_SE_ELP_PKA_CTRL_CONTROL_AUTO_RESEED(ELP_ENABLE),
		      TEGRA_SE_ELP_PKA_CTRL_CONTROL_OFFSET);

	/* Poll seeded status */
	do {
		if (i > PKA1_TIMEOUT) {
			dev_err(se_dev->dev,
				"Poll TRNG seeded status timed out\n");
			return -EINVAL;
		}
		udelay(1);
		val = se_elp_readl(se_dev, PKA1,
				   TEGRA_SE_ELP_PKA_TRNG_STATUS_OFFSET);
		i++;
	} while (val & TEGRA_SE_ELP_PKA_TRNG_STATUS_SEEDED(ELP_FALSE));

	return 0;
}

static void tegra_se_restart_pka_mutex_wdt(struct tegra_se_elp_dev *se_dev)
{
	se_elp_writel(se_dev, PKA1, TEGRA_SE_MUTEX_WDT_UNITS,
		      TEGRA_SE_ELP_PKA_MUTEX_WATCHDOG_OFFSET);
}

static u32 tegra_se_acquire_pka_mutex(struct tegra_se_elp_dev *se_dev)
{
	u32 val, i = 0;

	/* Acquire pka mutex */
	do {
		if (i > PKA1_TIMEOUT) {
			dev_err(se_dev->dev, "Acquire PKA Mutex timed out\n");
			return -EINVAL;
		}
		udelay(1);
		val = se_elp_readl(se_dev, PKA1, TEGRA_SE_ELP_PKA_MUTEX_OFFSET);
		i++;
	} while (val != 0x01);

	/* One unit is 256 SE Cycles */
	tegra_se_restart_pka_mutex_wdt(se_dev);
	se_elp_writel(se_dev, PKA1, TEGRA_SE_ELP_PKA_MUTEX_TIMEOUT_ACTION,
		      TEGRA_SE_ELP_PKA_MUTEX_TIMEOUT_ACTION_OFFSET);

	return 0;
}

static void tegra_se_release_pka_mutex(struct tegra_se_elp_dev *se_dev)
{
	se_elp_writel(se_dev, PKA1, 0x01,
		      TEGRA_SE_ELP_PKA_MUTEX_RELEASE_OFFSET);
}

static inline u32 pka_bank_start(u32 bank)
{
	return PKA_BANK_START_A + (bank * 0x400);
}

static inline u32 reg_bank_offset(u32 bank, u32 idx, u32 mode)
{
	return pka_bank_start(bank) + ((idx * 4) * num_words(mode));
}

static void tegra_se_fill_pka_opmem_addr(struct tegra_se_elp_dev *se_dev,
					 struct tegra_se_elp_pka_request *req)
{
	u32 i;
	int len = 0;
	int nwords = req->size / 4;
	int nwords_521 = pka_op_size[SE_ELP_OP_MODE_ECC521] / 32;
	u32 *MOD, *M, *R2, *EXP, *MSG;
	u32 *A, *B, *PX, *PY, *K, *QX, *QY;

	MOD = req->modulus;
	M = req->m;
	R2 = req->r2;

	/* TODO: The following code will be split into RSA and ECC specific
	 * APIs as part of Bug 200240635
	 */
	switch (req->op_mode) {
	case SE_ELP_OP_MODE_RSA512:
	case SE_ELP_OP_MODE_RSA768:
	case SE_ELP_OP_MODE_RSA1024:
	case SE_ELP_OP_MODE_RSA1536:
	case SE_ELP_OP_MODE_RSA2048:
	case SE_ELP_OP_MODE_RSA3072:
	case SE_ELP_OP_MODE_RSA4096:
		EXP = req->exponent;
		MSG = req->message;

		for (i = 0; i < nwords; i++) {
			se_elp_writel(se_dev, PKA1, *EXP++, reg_bank_offset(
				      TEGRA_SE_ELP_PKA_RSA_EXP_BANK,
				      TEGRA_SE_ELP_PKA_RSA_EXP_ID,
				      req->op_mode) + (i * 4));
			se_elp_writel(se_dev, PKA1, *MSG++, reg_bank_offset(
				      TEGRA_SE_ELP_PKA_RSA_MSG_BANK,
				      TEGRA_SE_ELP_PKA_RSA_MSG_ID,
				      req->op_mode) + (i * 4));
		}
		break;

	case SE_ELP_OP_MODE_ECC160:
	case SE_ELP_OP_MODE_ECC192:
	case SE_ELP_OP_MODE_ECC224:
	case SE_ELP_OP_MODE_ECC256:
	case SE_ELP_OP_MODE_ECC384:
	case SE_ELP_OP_MODE_ECC512:
	case SE_ELP_OP_MODE_ECC521:
		A = req->curve_param_a;

		if (req->op_mode == SE_ELP_OP_MODE_ECC521) {
			for (i = 0; i < nwords; i++)
				se_elp_writel(se_dev, PKA1, *MOD++,
					      reg_bank_offset(
						TEGRA_SE_ELP_PKA_MOD_BANK,
						TEGRA_SE_ELP_PKA_MOD_ID,
						req->op_mode) + (i * 4));

			for (i = nwords; i < nwords_521; i++)
				se_elp_writel(se_dev, PKA1, 0x0,
					      reg_bank_offset(
						TEGRA_SE_ELP_PKA_MOD_BANK,
						TEGRA_SE_ELP_PKA_MOD_ID,
						req->op_mode) + (i * 4));
		}

		for (i = 0; i < nwords; i++)
			se_elp_writel(se_dev, PKA1, *A++, reg_bank_offset(
					TEGRA_SE_ELP_PKA_ECC_A_BANK,
					TEGRA_SE_ELP_PKA_ECC_A_ID,
					req->op_mode) + (i * 4));

		if (req->op_mode == SE_ELP_OP_MODE_ECC521) {
			for (i = nwords; i < nwords_521; i++)
				se_elp_writel(se_dev, PKA1, 0x0,
					      reg_bank_offset(
						TEGRA_SE_ELP_PKA_ECC_A_BANK,
						TEGRA_SE_ELP_PKA_ECC_A_ID,
						req->op_mode) + (i * 4));
		}

		if (req->ecc_type != ECC_POINT_DOUBLE) {
			PX = req->base_pt_x;
			PY = req->base_pt_y;
			for (i = 0; i < nwords; i++) {
				se_elp_writel(se_dev, PKA1, *PX++,
					      reg_bank_offset(
						TEGRA_SE_ELP_PKA_ECC_XP_BANK,
						TEGRA_SE_ELP_PKA_ECC_XP_ID,
						req->op_mode) + (i * 4));

				se_elp_writel(se_dev, PKA1, *PY++,
					      reg_bank_offset(
						TEGRA_SE_ELP_PKA_ECC_YP_BANK,
						TEGRA_SE_ELP_PKA_ECC_YP_ID,
						req->op_mode) + (i * 4));
			}
			if (req->op_mode == SE_ELP_OP_MODE_ECC521) {
				for (i = nwords; i < nwords_521; i++) {
					se_elp_writel(se_dev, PKA1, 0x0,
						       reg_bank_offset(
						TEGRA_SE_ELP_PKA_ECC_XP_BANK,
						TEGRA_SE_ELP_PKA_ECC_XP_ID,
						req->op_mode) + (i * 4));
					se_elp_writel(se_dev, PKA1, 0x0,
						       reg_bank_offset(
						TEGRA_SE_ELP_PKA_ECC_YP_BANK,
						TEGRA_SE_ELP_PKA_ECC_YP_ID,
						req->op_mode) + (i * 4));
				}
			}
		}

		if (req->ecc_type == ECC_POINT_VER ||
		    req->ecc_type == ECC_SHAMIR_TRICK) {
			/* For shamir trick, curve_param_b is parameter k
			 * and k should be of size CTRL_BASE_RADIX
			 */
			B = req->curve_param_b;
			for (i = 0; i < nwords; i++)
				se_elp_writel(se_dev, PKA1, *B++,
							  reg_bank_offset(
						TEGRA_SE_ELP_PKA_ECC_B_BANK,
						TEGRA_SE_ELP_PKA_ECC_B_ID,
						req->op_mode) + (i * 4));

			if (req->ecc_type == ECC_SHAMIR_TRICK)
				len = num_words(req->op_mode);

			if (req->ecc_type == ECC_POINT_VER &&
			    req->op_mode == SE_ELP_OP_MODE_ECC521)
				len = nwords_521;

			for (i = nwords; i < len; i++)
				se_elp_writel(se_dev, PKA1, 0x0,
					      reg_bank_offset(
						TEGRA_SE_ELP_PKA_ECC_B_BANK,
						TEGRA_SE_ELP_PKA_ECC_B_ID,
						req->op_mode) + (i * 4));
		}

		if (req->ecc_type == ECC_POINT_ADD ||
		    req->ecc_type == ECC_SHAMIR_TRICK ||
		    req->ecc_type == ECC_POINT_DOUBLE) {
			QX = req->res_pt_x;
			QY = req->res_pt_y;
			for (i = 0; i < nwords; i++) {
				se_elp_writel(se_dev, PKA1, *QX++,
					      reg_bank_offset(
						TEGRA_SE_ELP_PKA_ECC_XQ_BANK,
						TEGRA_SE_ELP_PKA_ECC_XQ_ID,
						req->op_mode) + (i * 4));

				se_elp_writel(se_dev, PKA1, *QY++,
					      reg_bank_offset(
						TEGRA_SE_ELP_PKA_ECC_YQ_BANK,
						TEGRA_SE_ELP_PKA_ECC_YQ_ID,
						req->op_mode) + (i * 4));
			}
			if (req->op_mode == SE_ELP_OP_MODE_ECC521) {
				for (i = nwords; i < nwords_521; i++) {
					se_elp_writel(se_dev, PKA1, 0x0,
						       reg_bank_offset(
						TEGRA_SE_ELP_PKA_ECC_XQ_BANK,
						TEGRA_SE_ELP_PKA_ECC_XQ_ID,
						req->op_mode) + (i * 4));

					se_elp_writel(se_dev, PKA1, 0x0,
						       reg_bank_offset(
						TEGRA_SE_ELP_PKA_ECC_YQ_BANK,
						TEGRA_SE_ELP_PKA_ECC_YQ_ID,
						req->op_mode) + (i * 4));
				}
			}
		}

		if (req->ecc_type == ECC_POINT_MUL ||
		    req->ecc_type == ECC_SHAMIR_TRICK) {
			/* For shamir trick, key is parameter l
			 * and k for ECC_POINT_MUL and l for ECC_SHAMIR_TRICK
			 * should be of size CTRL_BASE_RADIX
			 */
			K = req->key;
			for (i = 0; i < nwords; i++)
				se_elp_writel(se_dev, PKA1, *K++,
					      reg_bank_offset(
						TEGRA_SE_ELP_PKA_ECC_K_BANK,
						TEGRA_SE_ELP_PKA_ECC_K_ID,
						req->op_mode) + (i * 4));

			for (i = nwords; i < num_words(req->op_mode); i++)
				se_elp_writel(se_dev, PKA1, 0x0,
							reg_bank_offset(
						TEGRA_SE_ELP_PKA_ECC_K_BANK,
						TEGRA_SE_ELP_PKA_ECC_K_ID,
						req->op_mode) + (i * 4));
		}
		break;
	}
}

static u32 pka_ctrl_base(u32 mode)
{
	struct tegra_se_elp_dev *se_dev = elp_dev;
	u32 val, base_radix;

	val = num_words(mode) * 32;
	switch (val) {
	case PKA_OP_SIZE_256:
		base_radix = TEGRA_SE_ELP_PKA_CTRL_BASE_256;
		break;
	case PKA_OP_SIZE_512:
		base_radix = TEGRA_SE_ELP_PKA_CTRL_BASE_512;
		break;
	case PKA_OP_SIZE_1024:
		base_radix = TEGRA_SE_ELP_PKA_CTRL_BASE_1024;
		break;
	case PKA_OP_SIZE_2048:
		base_radix = TEGRA_SE_ELP_PKA_CTRL_BASE_2048;
		break;
	case PKA_OP_SIZE_4096:
		base_radix = TEGRA_SE_ELP_PKA_CTRL_BASE_4096;
		break;
	default:
		dev_warn(se_dev->dev, "Invalid size: using PKA_OP_SIZE_256\n");
		base_radix = TEGRA_SE_ELP_PKA_CTRL_BASE_256;
		break;
	}

	return base_radix;
}

static void tegra_se_program_pka_regs(struct tegra_se_elp_dev *se_dev,
				      struct tegra_se_elp_pka_request *req)
{
	u32 val;

	se_elp_writel(se_dev, PKA1, 0, TEGRA_SE_ELP_PKA_FLAGS_OFFSET);
	se_elp_writel(se_dev, PKA1, 0, TEGRA_SE_ELP_PKA_FSTACK_PTR_OFFSET);

	/* TODO: The following code will be split into RSA and ECC specific
	 * APIs as part of Bug 200240635
	 */
	switch (req->op_mode) {
	case SE_ELP_OP_MODE_RSA512:
	case SE_ELP_OP_MODE_RSA768:
	case SE_ELP_OP_MODE_RSA1024:
	case SE_ELP_OP_MODE_RSA1536:
	case SE_ELP_OP_MODE_RSA2048:
	case SE_ELP_OP_MODE_RSA3072:
	case SE_ELP_OP_MODE_RSA4096:
		se_elp_writel(se_dev, PKA1,
			      TEGRA_SE_ELP_PKA_RSA_MOD_EXP_PRG_ENTRY_VAL,
			      TEGRA_SE_ELP_PKA_PRG_ENTRY_OFFSET);
		se_elp_writel(se_dev, PKA1,
			      TEGRA_SE_ELP_PKA_INT_ENABLE_IE_IRQ_EN(ELP_ENABLE),
			      TEGRA_SE_ELP_PKA_INT_ENABLE_OFFSET);

		val =
		TEGRA_SE_ELP_PKA_CTRL_BASE_RADIX(pka_ctrl_base(req->op_mode))
			| TEGRA_SE_ELP_PKA_CTRL_PARTIAL_RADIX(
					pka_op_size[req->op_mode] / 32);
		val |= TEGRA_SE_ELP_PKA_CTRL_GO(TEGRA_SE_ELP_PKA_CTRL_GO_START);
		se_elp_writel(se_dev, PKA1, val, TEGRA_SE_ELP_PKA_CTRL_OFFSET);
		break;

	case SE_ELP_OP_MODE_ECC160:
	case SE_ELP_OP_MODE_ECC192:
	case SE_ELP_OP_MODE_ECC224:
	case SE_ELP_OP_MODE_ECC256:
	case SE_ELP_OP_MODE_ECC384:
	case SE_ELP_OP_MODE_ECC512:
	case SE_ELP_OP_MODE_ECC521:
		if (req->ecc_type == ECC_POINT_MUL) {
			se_elp_writel(
				se_dev, PKA1,
				TEGRA_SE_ELP_PKA_ECC_POINT_MUL_PRG_ENTRY_VAL,
				TEGRA_SE_ELP_PKA_PRG_ENTRY_OFFSET);
			/*clear F0 for binding val*/
			se_elp_writel(se_dev, PKA1,
				      TEGRA_SE_ELP_PKA_FLAGS_FLAG_F0(
					ELP_DISABLE),
				      TEGRA_SE_ELP_PKA_FLAGS_OFFSET);
		} else if (req->ecc_type == ECC_POINT_ADD) {
			se_elp_writel(
				se_dev, PKA1,
				TEGRA_SE_ELP_PKA_ECC_POINT_ADD_PRG_ENTRY_VAL,
				TEGRA_SE_ELP_PKA_PRG_ENTRY_OFFSET);
		} else if (req->ecc_type == ECC_POINT_DOUBLE) {
			se_elp_writel(
				se_dev, PKA1,
				TEGRA_SE_ELP_PKA_ECC_POINT_DOUBLE_PRG_ENTRY_VAL,
				TEGRA_SE_ELP_PKA_PRG_ENTRY_OFFSET);
		} else if (req->ecc_type == ECC_POINT_VER) {
			se_elp_writel(se_dev, PKA1,
				      TEGRA_SE_ELP_PKA_ECC_ECPV_PRG_ENTRY_VAL,
				      TEGRA_SE_ELP_PKA_PRG_ENTRY_OFFSET);
		} else {
			se_elp_writel(
				se_dev, PKA1,
				TEGRA_SE_ELP_PKA_ECC_SHAMIR_TRICK_PRG_ENTRY_VAL,
				TEGRA_SE_ELP_PKA_PRG_ENTRY_OFFSET);
		}

		se_elp_writel(se_dev, PKA1,
			      TEGRA_SE_ELP_PKA_INT_ENABLE_IE_IRQ_EN(ELP_ENABLE),
			      TEGRA_SE_ELP_PKA_INT_ENABLE_OFFSET);

		if (req->op_mode == SE_ELP_OP_MODE_ECC521) {
			se_elp_writel(se_dev, PKA1,
				      TEGRA_SE_ELP_PKA_FLAGS_FLAG_F1(
					ELP_ENABLE),
				      TEGRA_SE_ELP_PKA_FLAGS_OFFSET);
		}

		se_elp_writel(se_dev, PKA1,
			      TEGRA_SE_ELP_PKA_CTRL_BASE_RADIX
				(pka_ctrl_base(req->op_mode)) |
			      TEGRA_SE_ELP_PKA_CTRL_PARTIAL_RADIX
				(pka_op_size[req->op_mode] / 32) |
			      TEGRA_SE_ELP_PKA_CTRL_GO
				(TEGRA_SE_ELP_PKA_CTRL_GO_START),
			      TEGRA_SE_ELP_PKA_CTRL_OFFSET);
		break;
	default:
		dev_warn(se_dev->dev, "Invalid operation mode\n");
		break;
	}
}

static int tegra_se_check_pka_op_done(struct tegra_se_elp_dev *se_dev)
{
	u32 val, i = 0;
	u32 abnormal_val;

	/* poll pka done status*/
	do {
		if (i > PKA1_TIMEOUT) {
			dev_err(se_dev->dev, "PKA Done status timed out\n");
			return -EINVAL;
		}
		udelay(1);
		val = se_elp_readl(se_dev, PKA1,
				   TEGRA_SE_ELP_PKA_STATUS_OFFSET);
		i++;
	} while (!(val & TEGRA_SE_ELP_PKA_STATUS_IRQ_STAT(ELP_ENABLE)));

	val = se_elp_readl(se_dev, PKA1, TEGRA_SE_ELP_PKA_RETURN_CODE_OFFSET);

	abnormal_val = TEGRA_SE_ELP_PKA_RETURN_CODE_STOP_REASON(
			TEGRA_SE_ELP_PKA_RETURN_CODE_STOP_REASON_ABNORMAL);

	if (abnormal_val & val) {
		dev_err(se_dev->dev, "PKA Operation ended Abnormally\n");
		return -EINVAL;
	}
	/* Write Status Register to acknowledge interrupt */
	val = se_elp_readl(se_dev, PKA1, TEGRA_SE_ELP_PKA_STATUS_OFFSET);
	se_elp_writel(se_dev, PKA1, val, TEGRA_SE_ELP_PKA_STATUS_OFFSET);

	return 0;
}

static void tegra_se_read_pka_result(struct tegra_se_elp_dev *se_dev,
				     struct tegra_se_elp_pka_request *req)
{
	u32 val, i;
	u32 *RES = req->result;
	u32 *QX = req->res_pt_x;
	u32 *QY = req->res_pt_y;

	/* TODO: The following code will be split into RSA and ECC specific
	 * APIs as part of Bug 200240635
	 */
	switch (req->op_mode) {
	case SE_ELP_OP_MODE_RSA512:
	case SE_ELP_OP_MODE_RSA768:
	case SE_ELP_OP_MODE_RSA1024:
	case SE_ELP_OP_MODE_RSA1536:
	case SE_ELP_OP_MODE_RSA2048:
	case SE_ELP_OP_MODE_RSA3072:
	case SE_ELP_OP_MODE_RSA4096:
		for (i = 0; i < req->size / 4; i++) {
			*RES = se_elp_readl(se_dev, PKA1, reg_bank_offset(
					   TEGRA_SE_ELP_PKA_RSA_RESULT_BANK,
					   TEGRA_SE_ELP_PKA_RSA_RESULT_ID,
					   req->op_mode) + (i * 4));
			RES++;
		}
		break;

	case SE_ELP_OP_MODE_ECC160:
	case SE_ELP_OP_MODE_ECC192:
	case SE_ELP_OP_MODE_ECC224:
	case SE_ELP_OP_MODE_ECC256:
	case SE_ELP_OP_MODE_ECC384:
	case SE_ELP_OP_MODE_ECC512:
	case SE_ELP_OP_MODE_ECC521:
		if (req->ecc_type == ECC_POINT_VER) {
			val = se_elp_readl(se_dev, PKA1,
					   TEGRA_SE_ELP_PKA_FLAGS_OFFSET);
			if (val & TEGRA_SE_ELP_PKA_FLAGS_FLAG_ZERO(ELP_ENABLE))
				req->pv_ok = true;
			else
				req->pv_ok = false;
		} else if (req->ecc_type == ECC_POINT_DOUBLE) {
			for (i = 0; i < req->size / 4; i++) {
				*QX = se_elp_readl(se_dev, PKA1,
						   reg_bank_offset(
						   TEGRA_SE_ELP_PKA_ECC_XP_BANK,
						   TEGRA_SE_ELP_PKA_ECC_XP_ID,
						   req->op_mode) + (i * 4));
				QX++;
			}
			for (i = 0; i < req->size / 4; i++) {
				*QY = se_elp_readl(se_dev, PKA1,
						   reg_bank_offset(
						   TEGRA_SE_ELP_PKA_ECC_YP_BANK,
						   TEGRA_SE_ELP_PKA_ECC_YP_ID,
						   req->op_mode) + (i * 4));
				QY++;
			}
		} else {
			for (i = 0; i < req->size / 4; i++) {
				*QX = se_elp_readl(se_dev, PKA1,
						   reg_bank_offset(
						   TEGRA_SE_ELP_PKA_ECC_XQ_BANK,
						   TEGRA_SE_ELP_PKA_ECC_XQ_ID,
						   req->op_mode) + (i * 4));
				QX++;
			}
			for (i = 0; i < req->size / 4; i++) {
				*QY = se_elp_readl(se_dev, PKA1,
						   reg_bank_offset(
						   TEGRA_SE_ELP_PKA_ECC_YQ_BANK,
						   TEGRA_SE_ELP_PKA_ECC_YQ_ID,
						   req->op_mode) + (i * 4));
				QY++;
			}
		}
		break;
	}
}

enum tegra_se_elp_pka_keyslot_field {
	EXPONENT,
	MOD_RSA,
	M_RSA,
	R2_RSA,
	PARAM_A,
	PARAM_B,
	MOD_ECC,
	XP,
	YP,
	XQ,
	YQ,
	KEY,
	M_ECC,
	R2_ECC,
};

static void tegra_se_set_pka_key(struct tegra_se_elp_dev *se_dev,
				 enum tegra_se_elp_op_mode mode,
				 struct tegra_se_elp_pka_request *req)
{
	u32 i;
	u32 slot_num = req->slot->slot_num;
	u32 *MOD, *M, *R2, *EXP, *MSG;
	u32 *A, *B, *PX, *PY, *K;

	MOD = req->modulus;
	M = req->m;
	R2 = req->r2;

	/* TODO: The following code will be split into RSA and ECC specific
	 * APIs as part of Bug 200240635
	 */
	switch (mode) {
	case SE_ELP_OP_MODE_RSA512:
	case SE_ELP_OP_MODE_RSA768:
	case SE_ELP_OP_MODE_RSA1024:
	case SE_ELP_OP_MODE_RSA1536:
	case SE_ELP_OP_MODE_RSA2048:
	case SE_ELP_OP_MODE_RSA3072:
	case SE_ELP_OP_MODE_RSA4096:
		EXP = req->exponent;
		MSG = req->message;
		for (i = 0; i < req->size / 4; i++) {
			se_elp_writel(se_dev, PKA1,
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD
						(EXPONENT) |
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_elp_writel(se_dev, PKA1, *EXP++,
				      TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_elp_writel(se_dev, PKA1,
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD
						(MOD_RSA) |
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_elp_writel(se_dev, PKA1, *MOD++,
				      TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_elp_writel(se_dev, PKA1,
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD
						(M_RSA) |
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_elp_writel(se_dev, PKA1, *M++,
				      TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_elp_writel(se_dev, PKA1,
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD
						(R2_RSA) |
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_elp_writel(se_dev, PKA1, *R2++,
				      TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET
						(slot_num));
		}
		break;

	case SE_ELP_OP_MODE_ECC160:
	case SE_ELP_OP_MODE_ECC192:
	case SE_ELP_OP_MODE_ECC224:
	case SE_ELP_OP_MODE_ECC256:
	case SE_ELP_OP_MODE_ECC384:
	case SE_ELP_OP_MODE_ECC512:
	case SE_ELP_OP_MODE_ECC521:
		A = req->curve_param_a;
		B = req->curve_param_b;
		PX = req->base_pt_x;
		PY = req->base_pt_y;
		K = req->key;
		for (i = 0; i < req->size / 4; i++) {
			se_elp_writel(se_dev, PKA1,
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD
						(PARAM_A) |
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_elp_writel(se_dev, PKA1, *A++,
				      TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_elp_writel(se_dev, PKA1,
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD
						(PARAM_B) |
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_elp_writel(se_dev, PKA1, *B++,
				      TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_elp_writel(se_dev, PKA1,
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD
						(MOD_ECC) |
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_elp_writel(se_dev, PKA1, *MOD++,
				      TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_elp_writel(se_dev, PKA1,
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD(XP) |
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_elp_writel(se_dev, PKA1, *PX++,
				      TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_elp_writel(se_dev, PKA1,
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD(YP) |
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET
					(slot_num));
			se_elp_writel(se_dev, PKA1, *PY++,
				      TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_elp_writel(se_dev, PKA1,
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD(KEY) |
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_elp_writel(se_dev, PKA1, *K++,
				      TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_elp_writel(se_dev, PKA1,
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD
						(M_ECC) |
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_elp_writel(se_dev, PKA1, *M++,
				      TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET
						(slot_num));

			se_elp_writel(se_dev, PKA1,
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD
						(R2_ECC) |
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				      TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET
						(slot_num));
			se_elp_writel(se_dev, PKA1, *R2++,
				      TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET
						(slot_num));
		}
		break;
	}
}

static int tegra_se_elp_pka_precomp(struct tegra_se_elp_dev *se_dev,
				    struct tegra_se_elp_pka_request *req,
				    u32 op)
{
	int ret, i;
	u32 *MOD = req->modulus;
	u32 *RINV = req->rinv;
	u32 *M = req->m;
	u32 *R2 = req->r2;

	if (req->op_mode == SE_ELP_OP_MODE_ECC521)
		return 0;

	se_elp_writel(se_dev, PKA1, 0, TEGRA_SE_ELP_PKA_FLAGS_OFFSET);
	se_elp_writel(se_dev, PKA1, 0, TEGRA_SE_ELP_PKA_FSTACK_PTR_OFFSET);

	if (op == PRECOMP_RINV) {
		for (i = 0; i < req->size / 4; i++) {
			se_elp_writel(se_dev, PKA1, *MOD++, reg_bank_offset(
				      TEGRA_SE_ELP_PKA_MOD_BANK,
				      TEGRA_SE_ELP_PKA_MOD_ID,
				      req->op_mode) + (i * 4));
		}
		se_elp_writel(se_dev, PKA1,
			      TEGRA_SE_ELP_PKA_RSA_RINV_PRG_ENTRY_VAL,
			      TEGRA_SE_ELP_PKA_PRG_ENTRY_OFFSET);
	} else if (op == PRECOMP_M) {
		se_elp_writel(se_dev, PKA1,
			      TEGRA_SE_ELP_PKA_RSA_M_PRG_ENTRY_VAL,
			      TEGRA_SE_ELP_PKA_PRG_ENTRY_OFFSET);
	} else {
		se_elp_writel(se_dev, PKA1,
			      TEGRA_SE_ELP_PKA_RSA_R2_PRG_ENTRY_VAL,
			      TEGRA_SE_ELP_PKA_PRG_ENTRY_OFFSET);
	}

	se_elp_writel(se_dev, PKA1,
		      TEGRA_SE_ELP_PKA_INT_ENABLE_IE_IRQ_EN(ELP_ENABLE),
		      TEGRA_SE_ELP_PKA_INT_ENABLE_OFFSET);
	se_elp_writel(se_dev, PKA1,
		      TEGRA_SE_ELP_PKA_CTRL_BASE_RADIX(
			pka_ctrl_base(req->op_mode)) |
		      TEGRA_SE_ELP_PKA_CTRL_PARTIAL_RADIX(req->size / 4) |
		      TEGRA_SE_ELP_PKA_CTRL_GO(TEGRA_SE_ELP_PKA_CTRL_GO_START),
		      TEGRA_SE_ELP_PKA_CTRL_OFFSET);

	ret = tegra_se_check_pka_op_done(se_dev);
	if (ret)
		return ret;

	if (op == PRECOMP_RINV) {
		for (i = 0; i < req->size / 4; i++) {
			*RINV = se_elp_readl(se_dev, PKA1, reg_bank_offset(
					     TEGRA_SE_ELP_PKA_RINV_BANK,
					     TEGRA_SE_ELP_PKA_RINV_ID,
					     req->op_mode) + (i * 4));
			RINV++;
		}
	} else if (op == PRECOMP_M) {
		for (i = 0; i < req->size / 4; i++) {
			*M = se_elp_readl(se_dev, PKA1, reg_bank_offset(
					  TEGRA_SE_ELP_PKA_M_BANK,
					  TEGRA_SE_ELP_PKA_M_ID,
					  req->op_mode) + (i * 4));
			M++;
		}
	} else {
		for (i = 0; i < req->size / 4; i++) {
			*R2 = se_elp_readl(se_dev, PKA1, reg_bank_offset(
					   TEGRA_SE_ELP_PKA_R2_BANK,
					   TEGRA_SE_ELP_PKA_R2_ID,
					   req->op_mode) + (i * 4));
			R2++;
		}
	}

	return ret;
}

static int tegra_se_elp_pka_do(struct tegra_se_elp_dev *se_dev,
			       struct tegra_se_elp_pka_request *req)
{
	int ret;
	u32 val;
	struct tegra_se_slot *pslot;

	if (se_dev->chipdata->use_key_slot) {
		if (!req->slot) {
			pslot = tegra_se_pka_alloc_key_slot();
			if (!pslot) {
				dev_err(se_dev->dev, "no free key slot\n");
				return -ENOMEM;
			}
			req->slot = pslot;
		}
		tegra_se_set_pka_key(se_dev, req->op_mode, req);
		/* Set LOAD_KEY */
		val = se_elp_readl(se_dev, PKA1,
				   TEGRA_SE_ELP_PKA_CTRL_CONTROL_OFFSET);
		val |= TEGRA_SE_ELP_PKA_CTRL_CONTROL_LOAD_KEY(ELP_ENABLE);
		se_elp_writel(se_dev, PKA1, val,
			      TEGRA_SE_ELP_PKA_CTRL_CONTROL_OFFSET);

		/*Write KEYSLOT Number */
		val = se_elp_readl(se_dev, PKA1,
				   TEGRA_SE_ELP_PKA_CTRL_CONTROL_OFFSET);
		val |=
		TEGRA_SE_ELP_PKA_CTRL_CONTROL_KEYSLOT(req->slot->slot_num);
		se_elp_writel(se_dev, PKA1, val,
			      TEGRA_SE_ELP_PKA_CTRL_CONTROL_OFFSET);
	} else {
		tegra_se_fill_pka_opmem_addr(se_dev, req);
	}

	tegra_se_program_pka_regs(se_dev, req);

	ret = tegra_se_check_pka_op_done(se_dev);
	if (ret)
		return ret;

	tegra_se_read_pka_result(se_dev, req);

	if (se_dev->chipdata->use_key_slot)
		tegra_se_pka_free_key_slot(req->slot);

	return ret;
}

static int tegra_se_elp_pka_init(struct tegra_se_elp_pka_request *req)
{
	struct tegra_se_elp_dev *se_dev = elp_dev;

	if (req->op_mode == SE_ELP_OP_MODE_ECC521)
		return 0;

	req->rinv = devm_kzalloc(se_dev->dev, req->size, GFP_KERNEL);
	if (!req->rinv)
		return -ENOMEM;

	req->m = devm_kzalloc(se_dev->dev, req->size, GFP_KERNEL);
	if (!req->m) {
		devm_kfree(se_dev->dev, req->rinv);
		return -ENOMEM;
	}

	req->r2 = devm_kzalloc(se_dev->dev, req->size, GFP_KERNEL);
	if (!req->r2) {
		devm_kfree(se_dev->dev, req->m);
		devm_kfree(se_dev->dev, req->rinv);
		return -ENOMEM;
	}

	return 0;
}

static void tegra_se_elp_pka_exit(struct tegra_se_elp_pka_request *req)
{
	struct tegra_se_elp_dev *se_dev = elp_dev;

	if (req->op_mode == SE_ELP_OP_MODE_ECC521)
		return;

	devm_kfree(se_dev->dev, req->r2);
	devm_kfree(se_dev->dev, req->m);
	devm_kfree(se_dev->dev, req->rinv);
}

static void tegra_se_release_rng_mutex(struct tegra_se_elp_dev *se_dev)
{
	se_elp_writel(se_dev, RNG1, 0x01, TEGRA_SE_ELP_RNG_MUTEX_OFFSET);
}

static u32 tegra_se_acquire_rng_mutex(struct tegra_se_elp_dev *se_dev)
{
	u32 val, i = 0;

	/* Acquire rng mutex */
	do {
		if (i > RNG1_TIMEOUT) {
			dev_err(se_dev->dev, "Acquire RNG1 Mutex timed out\n");
			return -EINVAL;
		}
		udelay(1);
		val = se_elp_readl(se_dev, RNG1, TEGRA_SE_ELP_RNG_MUTEX_OFFSET);
		i++;
	} while (val != 0x01);

	/* One unit is 256 SE Cycles */
	se_elp_writel(se_dev, RNG1, TEGRA_SE_MUTEX_WDT_UNITS,
		      TEGRA_SE_ELP_RNG_MUTEX_WATCHDOG_OFFSET);
	se_elp_writel(se_dev, RNG1, TEGRA_SE_ELP_RNG_MUTEX_TIMEOUT_ACTION,
		      TEGRA_SE_ELP_RNG_MUTEX_TIMEOUT_ACTION_OFFSET);

	return 0;
}

static u32 tegra_se_check_rng_status(struct tegra_se_elp_dev *se_dev)
{
	static bool rng1_first = true;
	bool secure_mode;
	u32 val, i = 0;

	/*Wait until RNG is Idle */
	do {
		if (i > RNG1_TIMEOUT) {
			dev_err(se_dev->dev, "RNG1 Idle timed out\n");
			return -EINVAL;
		}
		udelay(1);
		val = se_elp_readl(se_dev, RNG1,
				   TEGRA_SE_ELP_RNG_STATUS_OFFSET);
		i++;
	} while (val & TEGRA_SE_ELP_RNG_STATUS_BUSY(ELP_TRUE));

	if (rng1_first) {
		val = se_elp_readl(se_dev, RNG1,
				   TEGRA_SE_ELP_RNG_STATUS_OFFSET);
		if (val & TEGRA_SE_ELP_RNG_STATUS_SECURE(STATUS_SECURE))
			secure_mode = true;
		else
			secure_mode = false;

		/*Check health test is ok*/
		val = se_elp_readl(se_dev, RNG1,
				   TEGRA_SE_ELP_RNG_ISTATUS_OFFSET);
		if (secure_mode)
			val &= TEGRA_SE_ELP_RNG_ISTATUS_DONE(ISTATUS_ACTIVE);
		else
			val &= TEGRA_SE_ELP_RNG_ISTATUS_DONE(ISTATUS_ACTIVE) |
			TEGRA_SE_ELP_RNG_ISTATUS_NOISE_RDY(ISTATUS_ACTIVE);
		if (!val) {
			dev_err(se_dev->dev,
				"Wrong Startup value in RNG_ISTATUS Reg\n");
			return -EINVAL;
		}
		rng1_first = false;
	}

	val = se_elp_readl(se_dev, RNG1, TEGRA_SE_ELP_RNG_ISTATUS_OFFSET);
	se_elp_writel(se_dev, RNG1, val, TEGRA_SE_ELP_RNG_ISTATUS_OFFSET);

	val = se_elp_readl(se_dev, RNG1, TEGRA_SE_ELP_RNG_ISTATUS_OFFSET);
	if (val) {
		dev_err(se_dev->dev, "RNG_ISTATUS Reg is not cleared\n");
		return -EINVAL;
	}

	return 0;
}

static void tegra_se_set_rng1_mode(unsigned int mode)
{
	struct tegra_se_elp_dev *se_dev = elp_dev;

	/*no additional input mode*/
	se_elp_writel(se_dev, RNG1, mode, TEGRA_SE_ELP_RNG_SE_MODE_OFFSET);
}

static void tegra_se_set_rng1_smode(bool secure, bool nonce)
{
	u32 val = 0;
	struct tegra_se_elp_dev *se_dev = elp_dev;

	if (secure)
		val = TEGRA_SE_ELP_RNG_SE_SMODE_SECURE(SMODE_SECURE);
	if (nonce)
		val = TEGRA_SE_ELP_RNG_SE_SMODE_NONCE(ELP_ENABLE);

	/* need to write twice, switch secure/promiscuous
	 * mode would reset other bits
	 */
	se_elp_writel(se_dev, RNG1, val, TEGRA_SE_ELP_RNG_SE_SMODE_OFFSET);
	se_elp_writel(se_dev, RNG1, val, TEGRA_SE_ELP_RNG_SE_SMODE_OFFSET);
}

static int tegra_se_execute_rng1_ctrl_cmd(unsigned int cmd)
{
	u32 val, stat, i = 0;
	bool secure_mode;
	struct tegra_se_elp_dev *se_dev = elp_dev;

	se_elp_writel(se_dev, RNG1, 0xFFFFFFFF, TEGRA_SE_ELP_RNG_INT_EN_OFFSET);
	se_elp_writel(se_dev, RNG1, 0xFFFFFFFF, TEGRA_SE_ELP_RNG_IE_OFFSET);

	val = se_elp_readl(se_dev, RNG1, TEGRA_SE_ELP_RNG_STATUS_OFFSET);
	secure_mode = !!(val & TEGRA_SE_ELP_RNG_STATUS_SECURE(STATUS_SECURE));

	switch (cmd) {
	case RNG1_CMD_GEN_NONCE:
	case RNG1_CMD_CREATE_STATE:
	case RNG1_CMD_RENEW_STATE:
	case RNG1_CMD_REFRESH_ADDIN:
	case RNG1_CMD_GEN_RANDOM:
	case RNG1_CMD_ADVANCE_STATE:
		stat = TEGRA_SE_ELP_RNG_ISTATUS_DONE(ISTATUS_ACTIVE);
		break;
	case RNG1_CMD_GEN_NOISE:
		if (secure_mode)
			stat = TEGRA_SE_ELP_RNG_ISTATUS_DONE(ISTATUS_ACTIVE);
		else
			stat = TEGRA_SE_ELP_RNG_ISTATUS_DONE(ISTATUS_ACTIVE) |
			TEGRA_SE_ELP_RNG_ISTATUS_NOISE_RDY(ISTATUS_ACTIVE);
		break;
	case RNG1_CMD_KAT:
		stat = TEGRA_SE_ELP_RNG_ISTATUS_KAT_COMPLETED(ISTATUS_ACTIVE);
		break;
	case RNG1_CMD_ZEROIZE:
		stat = TEGRA_SE_ELP_RNG_ISTATUS_ZEROIZED(ISTATUS_ACTIVE);
		break;
	case RNG1_CMD_NOP:
	default:
		dev_err(se_dev->dev,
			"Cmd %d has nothing to do (or) invalid\n", cmd);
		dev_err(se_dev->dev, "RNG1 cmd failure: %s\n", rng1_cmd[cmd]);
		return -EINVAL;
	}
	se_elp_writel(se_dev, RNG1, cmd, TEGRA_SE_ELP_RNG_CTRL_OFFSET);

	do {
		if (i > RNG1_TIMEOUT) {
			dev_err(se_dev->dev, "\nRNG1 ISTAT poll timed out\n");
			return -EINVAL;
		}
		udelay(1);
		val = se_elp_readl(se_dev, RNG1,
				   TEGRA_SE_ELP_RNG_ISTATUS_OFFSET);
		i++;
	} while (val != stat);

	val = se_elp_readl(se_dev, RNG1, TEGRA_SE_ELP_RNG_IE_OFFSET);
	val = se_elp_readl(se_dev, RNG1, TEGRA_SE_ELP_RNG_INT_EN_OFFSET);

	i = 0;
	do {
		if (i > RNG1_TIMEOUT) {
			dev_err(se_dev->dev, "RNG1 INT status timed out\n");
			return -EINVAL;
		}
		udelay(1);
		val = se_elp_readl(se_dev, RNG1,
				   TEGRA_SE_ELP_RNG_INT_STATUS_OFFSET);
		i++;
	} while (!(val & TEGRA_SE_ELP_RNG_INT_STATUS_EIP0(STATUS_ACTIVE)));

	se_elp_writel(se_dev, RNG1, stat, TEGRA_SE_ELP_RNG_ISTATUS_OFFSET);
	val = se_elp_readl(se_dev, RNG1, TEGRA_SE_ELP_RNG_INT_STATUS_OFFSET);
	if (val & TEGRA_SE_ELP_RNG_INT_STATUS_EIP0(STATUS_ACTIVE)) {
		dev_err(se_dev->dev,
			"RNG1 intr not cleared (0x%x) after cmd %d execution\n",
			val, cmd);
		dev_err(se_dev->dev, "RNG1 Command Failure: %s\n",
			((cmd == RNG1_CMD_ZEROIZE) ? rng1_cmd[9] :
			rng1_cmd[cmd]));
		return -EINVAL;
	}

	return 0;
}

static int tegra_se_check_rng1_result(struct tegra_se_elp_rng_request *req)
{
	u32 i, val;
	struct tegra_se_elp_dev *se_dev = elp_dev;

	for (i = 0; i < 4; i++) {
		val = se_elp_readl(se_dev, RNG1,
				   TEGRA_SE_ELP_RNG_RAND0_OFFSET + i * 4);
		if (!val) {
			dev_err(se_dev->dev, "No random data from RAND\n");
			return -EINVAL;
		}
		se_dev->rdata[i] = val;
	}

	return 0;
}

static int tegra_se_check_rng1_alarms(void)
{
	u32 val;
	struct tegra_se_elp_dev *se_dev = elp_dev;

	val = se_elp_readl(se_dev, RNG1, TEGRA_SE_ELP_RNG_ALARMS_OFFSET);
	if (val) {
		dev_err(se_dev->dev, "RNG Alarms not cleared (0x%x)\n", val);
		return -EINVAL;
	}

	return 0;
}

static void tegra_se_rng1_feed_npa_data(void)
{
	int i;
	u32 data, r;
	struct tegra_se_elp_dev *se_dev = elp_dev;

	for (i = 0; i < 16; i++) {
		get_random_bytes(&r, sizeof(int));
		data = r & 0xffffffff;
		se_elp_writel(se_dev, RNG1, data,
			      TEGRA_SE_ELP_RNG_NPA_DATA0_OFFSET + i * 4);
	}
}

static int tegra_se_elp_rng_do(struct tegra_se_elp_dev *se_dev,
			       struct tegra_se_elp_rng_request *req)
{
	u32 *rand_num;
	int i, j, k, ret;
	bool adv_state = false;

	rand_num = devm_kzalloc(se_dev->dev,
				(sizeof(*rand_num) * (RAND_256 / 4)),
				GFP_KERNEL);
	if (!rand_num)
		return -ENOMEM;

	tegra_se_set_rng1_smode(true, false);
	tegra_se_set_rng1_mode(RNG1_MODE_SEC_ALG);
	/* Generate Noise */
	ret = tegra_se_execute_rng1_ctrl_cmd(RNG1_CMD_GEN_NOISE);
	if (ret)
		return ret;
	ret = tegra_se_execute_rng1_ctrl_cmd(RNG1_CMD_CREATE_STATE);
	if (ret)
		return ret;

	for (i = 0; i < req->size/RAND_128; i++) {
		if (i && req->adv_state_on && (i % ADV_STATE_FREQ == 0)) {
			ret = tegra_se_execute_rng1_ctrl_cmd
						(RNG1_CMD_ADVANCE_STATE);
			if (ret)
				return ret;
			adv_state = true;
		}
		ret = tegra_se_execute_rng1_ctrl_cmd(RNG1_CMD_GEN_RANDOM);
		if (ret)
			return ret;

		ret = tegra_se_check_rng1_result(req);
		if (ret) {
			dev_err(se_dev->dev, "RNG1 Failed for Sub-Step 1\n");
			return ret;
		}

		for (k = (4 * i), j = 0; k < 4 * (i + 1); k++, j++)
			rand_num[k] = se_dev->rdata[j];

		if (adv_state) {
			ret = tegra_se_execute_rng1_ctrl_cmd
						(RNG1_CMD_ADVANCE_STATE);
			if (ret)
				return ret;
			if ((i+1) < req->size/RAND_128) {
				ret = tegra_se_execute_rng1_ctrl_cmd
							(RNG1_CMD_GEN_NOISE);
				if (ret)
					return ret;
				ret = tegra_se_execute_rng1_ctrl_cmd
							(RNG1_CMD_RENEW_STATE);
				if (ret)
					return ret;
			}
		}
	}

	for (k = 0; k < (RAND_256 / 4); k++)
		req->rdata[k] = rand_num[k];

	tegra_se_check_rng1_alarms();

	if (!req->test_full_cmd_flow)
		return ret;

	ret = tegra_se_execute_rng1_ctrl_cmd(RNG1_CMD_ZEROIZE);
	if (ret)
		return ret;

	tegra_se_set_rng1_smode(true, false);
	tegra_se_set_rng1_mode(RNG1_MODE_ADDIN_PRESENT);

	ret = tegra_se_execute_rng1_ctrl_cmd(RNG1_CMD_GEN_NOISE);
	if (ret)
		return ret;
	tegra_se_rng1_feed_npa_data();
	ret = tegra_se_execute_rng1_ctrl_cmd(RNG1_CMD_CREATE_STATE);
	if (ret)
		return ret;
	tegra_se_rng1_feed_npa_data();
	ret = tegra_se_execute_rng1_ctrl_cmd(RNG1_CMD_REFRESH_ADDIN);
	if (ret)
		return ret;

	for (i = 0; i < req->size/RAND_128; i++) {
		if (i && req->adv_state_on && (i % ADV_STATE_FREQ == 0)) {
			ret = tegra_se_execute_rng1_ctrl_cmd
						(RNG1_CMD_ADVANCE_STATE);
			if (ret)
				return ret;
			tegra_se_rng1_feed_npa_data();
			ret = tegra_se_execute_rng1_ctrl_cmd
						(RNG1_CMD_REFRESH_ADDIN);
			if (ret)
				return ret;
			adv_state = true;
		}
		ret = tegra_se_execute_rng1_ctrl_cmd(RNG1_CMD_GEN_RANDOM);
		if (ret)
			return ret;

		ret = tegra_se_check_rng1_result(req);
		if (ret) {
			dev_err(se_dev->dev, "RNG1 Failed for Sub-Step 2\n");
			return ret;
		}

		for (k = (4 * i), j = 0; k < 4 * (i + 1); k++, j++)
			rand_num[k] = se_dev->rdata[j];

		if (adv_state) {
			ret = tegra_se_execute_rng1_ctrl_cmd
						(RNG1_CMD_ADVANCE_STATE);
			if (ret)
				return ret;
			if ((i + 1) < req->size / RAND_128) {
				ret = tegra_se_execute_rng1_ctrl_cmd
							(RNG1_CMD_GEN_NOISE);
				if (ret)
					return ret;
				tegra_se_rng1_feed_npa_data();
				ret = tegra_se_execute_rng1_ctrl_cmd
							(RNG1_CMD_RENEW_STATE);
				if (ret)
					return ret;
				tegra_se_rng1_feed_npa_data();
				ret = tegra_se_execute_rng1_ctrl_cmd
						(RNG1_CMD_REFRESH_ADDIN);
				if (ret)
					return ret;
			}
		}
	}

	for (k = 0; k < (RAND_256 / 4); k++)
		req->rdata1[k] = rand_num[k];

	tegra_se_check_rng1_alarms();

	ret = tegra_se_execute_rng1_ctrl_cmd(RNG1_CMD_ZEROIZE);
	if (ret)
		return ret;

	tegra_se_set_rng1_smode(true, true);
	tegra_se_set_rng1_mode(RNG1_MODE_ADDIN_PRESENT);

	tegra_se_rng1_feed_npa_data();
	ret = tegra_se_execute_rng1_ctrl_cmd(RNG1_CMD_GEN_NONCE);
	if (ret)
		return ret;
	tegra_se_rng1_feed_npa_data();
	ret = tegra_se_execute_rng1_ctrl_cmd(RNG1_CMD_GEN_NONCE);
	if (ret)
		return ret;
	tegra_se_rng1_feed_npa_data();
	ret = tegra_se_execute_rng1_ctrl_cmd(RNG1_CMD_CREATE_STATE);
	if (ret)
		return ret;
	tegra_se_rng1_feed_npa_data();
	ret = tegra_se_execute_rng1_ctrl_cmd(RNG1_CMD_REFRESH_ADDIN);
	if (ret)
		return ret;

	for (i = 0; i < req->size / RAND_128; i++) {
		if (i && req->adv_state_on && (i % ADV_STATE_FREQ == 0)) {
			ret = tegra_se_execute_rng1_ctrl_cmd
					(RNG1_CMD_ADVANCE_STATE);
			if (ret)
				return ret;
			tegra_se_rng1_feed_npa_data();
			ret =
			tegra_se_execute_rng1_ctrl_cmd(RNG1_CMD_REFRESH_ADDIN);
			if (ret)
				return ret;
			adv_state = true;
		}
		ret = tegra_se_execute_rng1_ctrl_cmd(RNG1_CMD_GEN_RANDOM);
		if (ret)
			return ret;

		ret = tegra_se_check_rng1_result(req);
		if (ret) {
			dev_err(se_dev->dev, "RNG1 Failed for Sub-Step 3\n");
			return ret;
		}

		for (k = (4 * i), j = 0; k < 4 * (i + 1); k++, j++)
			rand_num[k] = se_dev->rdata[j];

		if (adv_state) {
			ret = tegra_se_execute_rng1_ctrl_cmd
						(RNG1_CMD_ADVANCE_STATE);
			if (ret)
				return ret;
			if ((i + 1) < req->size / RAND_128) {
				ret = tegra_se_execute_rng1_ctrl_cmd
							(RNG1_CMD_GEN_NONCE);
				if (ret)
					return ret;
				tegra_se_rng1_feed_npa_data();
				ret = tegra_se_execute_rng1_ctrl_cmd
							(RNG1_CMD_RENEW_STATE);
				if (ret)
					return ret;
				tegra_se_rng1_feed_npa_data();
				ret = tegra_se_execute_rng1_ctrl_cmd
						(RNG1_CMD_REFRESH_ADDIN);
				if (ret)
					return ret;
			}
		}
	}

	for (k = 0; k < (RAND_256 / 4); k++)
		req->rdata2[k] = rand_num[k];

	tegra_se_check_rng1_alarms();

	return tegra_se_execute_rng1_ctrl_cmd(RNG1_CMD_ZEROIZE);
}

int tegra_se_elp_rng_op(struct tegra_se_elp_rng_request *req)
{
	struct tegra_se_elp_dev *se_dev = elp_dev;
	int ret;

	clk_prepare_enable(se_dev->c);
	ret = tegra_se_acquire_rng_mutex(se_dev);
	if (ret) {
		dev_err(se_dev->dev, "RNG1 Mutex acquire failed\n");
		clk_disable_unprepare(se_dev->c);
		return ret;
	}

	ret = tegra_se_check_rng_status(se_dev);
	if (ret) {
		dev_err(se_dev->dev, "RNG1 initial state is wrong\n");
		goto rel_mutex;
	}

	ret = tegra_se_elp_rng_do(se_dev, req);
rel_mutex:
	tegra_se_release_rng_mutex(se_dev);
	clk_disable_unprepare(se_dev->c);

	return ret;
}
EXPORT_SYMBOL(tegra_se_elp_rng_op);

int tegra_se_elp_pka_op(struct tegra_se_elp_pka_request *req)
{
	struct tegra_se_elp_dev *se_dev = elp_dev;
	int ret;

	clk_prepare_enable(se_dev->c);
	ret = tegra_se_acquire_pka_mutex(se_dev);
	if (ret) {
		dev_err(se_dev->dev, "PKA1 Mutex acquire failed\n");
		goto clk_dis;
	}

	ret = tegra_se_elp_pka_init(req);
	if (ret)
		goto rel_mutex;

	ret = tegra_se_check_trng_op(se_dev);
	if (ret)
		ret = tegra_se_set_trng_op(se_dev);
	if (ret) {
		dev_err(se_dev->dev, "set_trng_op Failed\n");
		goto exit;
	}

	ret = tegra_se_elp_pka_precomp(se_dev, req, PRECOMP_RINV);
	if (ret) {
		dev_err(se_dev->dev,
			"RINV: tegra_se_elp_pka_precomp Failed(%d)\n", ret);
		goto exit;
	}
	ret = tegra_se_elp_pka_precomp(se_dev, req, PRECOMP_M);
	if (ret) {
		dev_err(se_dev->dev,
			"M: tegra_se_elp_pka_precomp Failed(%d)\n", ret);
		goto exit;
	}
	ret = tegra_se_elp_pka_precomp(se_dev, req, PRECOMP_R2);
	if (ret) {
		dev_err(se_dev->dev,
			"R2: tegra_se_elp_pka_precomp Failed(%d)\n", ret);
		goto exit;
	}
	ret = tegra_se_elp_pka_do(se_dev, req);
exit:
	tegra_se_elp_pka_exit(req);
rel_mutex:
	tegra_se_release_pka_mutex(se_dev);
clk_dis:
	clk_disable_unprepare(se_dev->c);

	return ret;
}
EXPORT_SYMBOL(tegra_se_elp_pka_op);

static int tegra_se_ecc_point_mult(struct tegra_se_ecc_point *result,
				   const struct tegra_se_ecc_point *point,
				   const u32 *private,
				   const struct tegra_se_ecc_curve *curve,
				   int nbytes)
{
	struct tegra_se_elp_pka_request ecc_req;
	int ret;

	ecc_req.op_mode = curve->mode;
	ecc_req.size = nbytes;
	ecc_req.ecc_type = ECC_POINT_MUL;
	ecc_req.curve_param_a = curve->a;
	ecc_req.modulus = curve->p;
	ecc_req.base_pt_x = point->x;
	ecc_req.base_pt_y = point->y;
	ecc_req.res_pt_x = result->x;
	ecc_req.res_pt_y = result->y;
	ecc_req.key = (u32 *)private;

	ret = tegra_se_elp_pka_op(&ecc_req);

	return ret;
}

static int tegra_se_ecdh_compute_shared_secret(struct tegra_se_elp_dev *se_dev,
					       unsigned int cid,
					       const u32 *private_key,
					       const u32 *public_key,
					       u32 *secret)
{
	struct tegra_se_ecc_point *product, *pk;
	const struct tegra_se_ecc_curve *curve = tegra_se_ecc_get_curve(cid);
	int nbytes = curve->nbytes;
	int nwords = nbytes / WORD_SIZE_BYTES;
	int ret = -ENOMEM;
	u32 priv[ECC_MAX_WORDS];

	if (!private_key || !public_key)
		return -EINVAL;

	pk = tegra_se_ecc_alloc_point(se_dev, nwords);
	if (!pk)
		return ret;

	product = tegra_se_ecc_alloc_point(se_dev, nwords);
	if (!product)
		goto exit;

	tegra_se_ecc_swap(public_key, pk->x, nwords);
	tegra_se_ecc_swap(&public_key[nwords], pk->y, nwords);
	tegra_se_ecc_swap(private_key, priv, nwords);

	ret = tegra_se_ecc_point_mult(product, pk, priv, curve, nbytes);
	if (ret)
		goto err_pt_mult;

	tegra_se_ecc_swap(product->x, secret, nwords);

	if (tegra_se_ecc_vec_is_zero(product->x, nbytes) ||
	    tegra_se_ecc_vec_is_zero(product->y, nbytes))
		ret = -ENODATA;
err_pt_mult:
	tegra_se_ecc_free_point(se_dev, product);
exit:
	tegra_se_ecc_free_point(se_dev, pk);

	return ret;
}

static int tegra_se_ecdh_gen_pub_key(struct tegra_se_elp_dev *se_dev,
				     unsigned int cid, const u32 *private_key,
				     u32 *public_key)
{
	struct tegra_se_ecc_point *G;
	int ret;
	const struct tegra_se_ecc_curve *curve = tegra_se_ecc_get_curve(cid);
	int nbytes = curve->nbytes;
	int nwords = nbytes / WORD_SIZE_BYTES;
	u32 priv[ECC_MAX_WORDS];

	if (!private_key)
		return -EINVAL;

	tegra_se_ecc_swap(private_key, priv, nwords);

	G = tegra_se_ecc_alloc_point(se_dev, nwords);
	if (!G)
		return -ENOMEM;

	ret = tegra_se_ecc_point_mult(G, &curve->g, priv, curve, nbytes);
	if (ret)
		goto err_pt_mult;

	if (tegra_se_ecc_vec_is_zero(G->x, nbytes) ||
	    tegra_se_ecc_vec_is_zero(G->y, nbytes))
		ret = -ENODATA;

	tegra_se_ecc_swap(G->x, public_key, nwords);
	tegra_se_ecc_swap(G->y, &public_key[nwords], nwords);

err_pt_mult:
	tegra_se_ecc_free_point(se_dev, G);
	return ret;
}

static int tegra_se_ecdh_compute_value(struct kpp_request *req)
{
	struct crypto_kpp *tfm;
	struct tegra_se_ecdh_context *ctx;
	int nbytes, ret, cnt;
	void *buffer;
	const struct tegra_se_ecc_curve *curve;

	if (!req)
		return -EINVAL;

	tfm = crypto_kpp_reqtfm(req);
	if (!tfm)
		return -ENODATA;

	ctx = kpp_tfm_ctx(tfm);
	if (!ctx)
		return -ENODATA;

	ctx->se_dev = elp_dev;

	curve = tegra_se_ecc_get_curve(ctx->curve_id);
	if (!curve)
		return -ENOTSUPP;

	nbytes = curve->nbytes;

	if (req->src) {
		cnt = sg_copy_to_buffer(req->src, 1, ctx->public_key,
					2 * nbytes);
		if (cnt != 2 * nbytes)
			return -ENODATA;

		ret = tegra_se_ecdh_compute_shared_secret(ctx->se_dev,
							  ctx->curve_id,
							  ctx->private_key,
							  ctx->public_key,
							  ctx->shared_secret);
		if (ret < 0)
			return ret;

		buffer = ctx->shared_secret;

		cnt = sg_copy_from_buffer(req->dst, 1, buffer, nbytes);
		if (cnt != nbytes)
			return -ENODATA;
	} else {
		ret = tegra_se_ecdh_gen_pub_key(ctx->se_dev, ctx->curve_id,
						ctx->private_key,
						ctx->public_key);
		if (ret < 0)
			return ret;

		buffer = ctx->public_key;

		cnt = sg_copy_from_buffer(req->dst, 1, buffer, 2 * nbytes);
		if (cnt != 2 * nbytes)
			return -ENODATA;
	}

	return 0;
}

static int tegra_se_ecdh_set_secret(struct crypto_kpp *tfm, void *buf,
				    unsigned int len)
{
	struct tegra_se_ecdh_context *ctx = kpp_tfm_ctx(tfm);
	struct ecdh params;
	int ret;

	ret = crypto_ecdh_decode_key(buf, len, &params);
	if (ret)
		return ret;

	ret = tegra_se_ecdh_set_params(ctx, &params);
	if (ret)
		return ret;

	return 0;
}

static int tegra_se_ecdh_max_size(struct crypto_kpp *tfm)
{
	struct tegra_se_ecdh_context *ctx = kpp_tfm_ctx(tfm);
	const struct tegra_se_ecc_curve *curve =
			tegra_se_ecc_get_curve(ctx->curve_id);
	int nbytes = curve->nbytes;

	/* Public key is made of two coordinates */
	return 2 * nbytes;
}

static struct kpp_alg ecdh_algs[] = {
	{
	.set_secret = tegra_se_ecdh_set_secret,
	.generate_public_key = tegra_se_ecdh_compute_value,
	.compute_shared_secret = tegra_se_ecdh_compute_value,
	.max_size = tegra_se_ecdh_max_size,
	.base = {
		.cra_name = "ecdh",
		.cra_driver_name = "tegra-se-ecdh",
		.cra_priority = 300,
		.cra_module = THIS_MODULE,
		.cra_ctxsize = sizeof(struct tegra_se_ecdh_context),
		}
	}
};

static struct tegra_se_chipdata tegra18_se_chipdata = {
	.use_key_slot = false,
};

static int tegra_se_elp_probe(struct platform_device *pdev)
{
	struct tegra_se_elp_dev *se_dev;
	struct resource *res;
	int err, i;

	se_dev = devm_kzalloc(&pdev->dev, sizeof(struct tegra_se_elp_dev),
			      GFP_KERNEL);
	if (!se_dev)
		return -ENOMEM;

	se_dev->chipdata = of_device_get_match_data(&pdev->dev);

	platform_set_drvdata(pdev, se_dev);
	se_dev->dev = &pdev->dev;

	for (i = 0; i < NR_RES; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		se_dev->io_reg[i] = devm_ioremap_resource(se_dev->dev, res);
		if (IS_ERR(se_dev->io_reg[i]))
			return PTR_ERR(se_dev->io_reg[i]);
	}

	se_dev->c = devm_clk_get(se_dev->dev, "se");
	if (IS_ERR(se_dev->c)) {
		dev_err(se_dev->dev, "se clk_get_sys failed: %ld\n",
			PTR_ERR(se_dev->c));
		return PTR_ERR(se_dev->c);
	}

	err = clk_prepare_enable(se_dev->c);
	if (err) {
		dev_err(se_dev->dev, "clk enable failed for se\n");
		return err;
	}

	se_dev->rdata = devm_kzalloc(se_dev->dev,
				     sizeof(u32) * 4, GFP_KERNEL);
	if (!se_dev->rdata) {
		err = -ENOMEM;
		goto exit;
	}

	elp_dev = se_dev;

	err = tegra_se_pka_init_key_slot(se_dev);
	if (err) {
		dev_err(se_dev->dev, "tegra_se_pka_init_key_slot failed\n");
		goto exit;
	}

	err = crypto_register_kpp(&ecdh_algs[0]);
	if (err)
		dev_err(se_dev->dev, "kpp registeration failed for ECDH\n");
exit:
	clk_disable_unprepare(se_dev->c);

	if (!err)
		dev_info(se_dev->dev, "%s: complete", __func__);
	else
		crypto_unregister_kpp(&ecdh_algs[0]);

	return err;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_se_elp_suspend(struct device *dev)
{
	struct tegra_se_elp_dev *se_dev = dev_get_drvdata(dev);

	/* This is needed as ATF (ARM Trusted Firmware) needs SE clk in SC7
	 * cycle and ATF does not have access to BPMP to enable the clk by
	 * itself. So, currently this is handled in linux driver.
	 */
	clk_prepare_enable(se_dev->c);

	return 0;
}

static int tegra_se_elp_resume(struct device *dev)
{
	struct tegra_se_elp_dev *se_dev = dev_get_drvdata(dev);

	clk_disable_unprepare(se_dev->c);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops tegra_se_elp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tegra_se_elp_suspend, tegra_se_elp_resume)
};

static const struct of_device_id tegra_se_elp_of_match[] = {
	{
		.compatible = "nvidia,tegra186-se-elp",
		.data = &tegra18_se_chipdata,
	},
};
MODULE_DEVICE_TABLE(of, tegra_se_elp_of_match);

static struct platform_driver tegra_se_elp_driver = {
	.probe  = tegra_se_elp_probe,
	.driver = {
		.name   = "tegra-se-elp",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_se_elp_of_match),
		.pm = &tegra_se_elp_pm_ops,
	},
};
module_platform_driver(tegra_se_elp_driver);

MODULE_DESCRIPTION("Tegra Elliptic Crypto algorithm support");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL");
MODULE_ALIAS("tegra-se-elp");

