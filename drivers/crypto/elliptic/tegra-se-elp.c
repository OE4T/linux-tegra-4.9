/*
 * Cryptographic API.
 * drivers/crypto/tegra-se.c
 *
 * Support for Tegra Security Engine hardware crypto algorithms.
 *
 * Copyright (c) 2015, NVIDIA Corporation. All Rights Reserved.
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
#include <crypto/scatterwalk.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/internal/rng.h>
#include <crypto/internal/hash.h>
#include <crypto/sha.h>
#include <linux/tegra_pm_domains.h>

#include "tegra-se-elp.h"

#define DRIVER_NAME	"tegra-se-elp"

#define PKA1	0
#define RNG1	1

#define TEGRA_SE_PKA_MUTEX_WDT_UNITS	0x600000

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

/* Security Engine operation modes */
enum tegra_se_elp_op_mode {
	SE_ELP_OP_MODE_RSA512,
	SE_ELP_OP_MODE_RSA768,
	SE_ELP_OP_MODE_RSA1024,
	SE_ELP_OP_MODE_RSA1536,
	SE_ELP_OP_MODE_RSA2048,
	SE_ELP_OP_MODE_RSA3072,
	SE_ELP_OP_MODE_RSA4096,
	SE_ELP_OP_MODE_ECC160,
	SE_ELP_OP_MODE_ECC192,
	SE_ELP_OP_MODE_ECC224,
	SE_ELP_OP_MODE_ECC256,
	SE_ELP_OP_MODE_ECC384,
	SE_ELP_OP_MODE_ECC512,
	SE_ELP_OP_MODE_ECC521,
};

struct tegra_se_chipdata {
	bool use_key_slot;
};

struct tegra_se_elp_dev {
	struct platform_device *pdev;
	struct device *dev;
	void __iomem *io_reg[2];
	struct tegra_se_slot *slot_list;
	struct tegra_se_chipdata *chipdata;
	struct tegra_se_elp_pka_request *pka_req;
};

static struct tegra_se_elp_dev *elp_dev;

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
	bool available; /* Tells whether key slot is free to use */
};

static LIST_HEAD(key_slot);
static DEFINE_SPINLOCK(key_slot_lock);

static u32 pka_op_size[16] = {512, 768, 1024, 1536, 2048, 3072, 4096, 160, 192,
				224, 256, 384, 512, 640};

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
		dev_warn(elp_dev->dev, "%s: Invalid operation mode\n",
					__func__);
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
	unsigned int val;

	val = readl(se_dev->io_reg[elp_type] + reg_offset);

	return val;
}

static void tegra_se_pka_free_key_slot(struct tegra_se_slot *slot)
{
	if (slot) {
		spin_lock(&key_slot_lock);
		slot->available = true;
		spin_unlock(&key_slot_lock);
	}
}

static struct tegra_se_slot *tegra_se_pka_alloc_key_slot(void)
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

static int tegra_se_pka_init_key_slot(void)
{
	struct tegra_se_elp_dev *se_dev = elp_dev;
	int i;

	se_dev->slot_list = kzalloc(sizeof(struct tegra_se_slot) *
				TEGRA_SE_PKA_KEYSLOT_COUNT, GFP_KERNEL);
	if (se_dev->slot_list == NULL) {
		dev_err(se_dev->dev, "slot list memory allocation failed\n");
		return -ENOMEM;
	}

	spin_lock_init(&key_slot_lock);
	spin_lock(&key_slot_lock);

	for (i = 0; i < TEGRA_SE_PKA_KEYSLOT_COUNT; i++) {
		se_dev->slot_list[i].available = true;
		se_dev->slot_list[i].slot_num = i;
		INIT_LIST_HEAD(&se_dev->slot_list[i].node);
		list_add_tail(&se_dev->slot_list[i].node, &key_slot);
	}

	spin_unlock(&key_slot_lock);
	return 0;
}

static u32 tegra_se_check_trng_op(struct tegra_se_elp_dev *se_dev)
{
	u32 val = 0;

	val = se_elp_readl(se_dev, PKA1, TEGRA_SE_ELP_PKA_TRNG_STATUS_OFFSET);

	if ((val & TEGRA_SE_ELP_PKA_TRNG_STATUS_SECURE(TRUE)) &&
		(val & TEGRA_SE_ELP_PKA_TRNG_STATUS_NONCE(FALSE)) &&
		(val & TEGRA_SE_ELP_PKA_TRNG_STATUS_SEEDED(TRUE)) &&
		((val &
		TEGRA_SE_ELP_PKA_TRNG_STATUS_LAST_RESEED(TRNG_LAST_RESEED_HOST))
		|| (val &
	TEGRA_SE_ELP_PKA_TRNG_STATUS_LAST_RESEED(TRNG_LAST_RESEED_RESEED))))
		return 0;
	else
		return -EINVAL;
}

static u32 tegra_se_set_trng_op(struct tegra_se_elp_dev *se_dev)
{
	u32 val = 0;
	u32 err = 0;

	se_elp_writel(se_dev, PKA1,
		TEGRA_SE_ELP_PKA_TRNG_SMODE_SECURE(ENABLE) |
		TEGRA_SE_ELP_PKA_TRNG_SMODE_NONCE(DISABLE),
		TEGRA_SE_ELP_PKA_TRNG_SMODE_OFFSET);
	se_elp_writel(se_dev, PKA1,
		TEGRA_SE_ELP_PKA_CTRL_CONTROL_AUTO_RESEED(ENABLE),
		TEGRA_SE_ELP_PKA_CTRL_CONTROL_OFFSET);

	/* Poll seeded status */
	val = se_elp_readl(se_dev, PKA1, TEGRA_SE_ELP_PKA_TRNG_STATUS_OFFSET);
	while (val & TEGRA_SE_ELP_PKA_TRNG_STATUS_SEEDED(FALSE))
		val = se_elp_readl(se_dev, PKA1,
			TEGRA_SE_ELP_PKA_TRNG_STATUS_OFFSET);

	if (val & TEGRA_SE_ELP_PKA_TRNG_STATUS_SEEDED(FALSE)) {
		dev_err(se_dev->dev, "\nAbrupt end of Seeding operation\n");
		err = -EINVAL;
	}

	return err;
}

static void tegra_se_restart_pka_mutex_wdt(struct tegra_se_elp_dev *se_dev)
{
	se_elp_writel(se_dev, PKA1, TEGRA_SE_PKA_MUTEX_WDT_UNITS,
			TEGRA_SE_ELP_PKA_MUTEX_WATCHDOG_OFFSET);
}

static u32 tegra_se_acquire_pka_mutex(struct tegra_se_elp_dev *se_dev)
{
	u32 val = 0;
	u32 err = 0;

	/* TODO: Add timeout for while loop */
	/* Acquire pka mutex */
	val = se_elp_readl(se_dev, PKA1, TEGRA_SE_ELP_PKA_MUTEX_OFFSET);
	while (val != 0x01)
		val = se_elp_readl(se_dev, PKA1, TEGRA_SE_ELP_PKA_MUTEX_OFFSET);

	/* One unit is 256 SE Cycles */
	tegra_se_restart_pka_mutex_wdt(se_dev);
	se_elp_writel(se_dev, PKA1, TEGRA_SE_ELP_PKA_MUTEX_TIMEOUT_ACTION,
			TEGRA_SE_ELP_PKA_MUTEX_TIMEOUT_ACTION_OFFSET);

	return err;
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
	u32 i = 0;
	u32 *MOD, *M, *R2, *EXP, *MSG;
	u32 *A, *B, *PX, *PY, *K, *QX, *QY;

	MOD = req->modulus;
	M = req->m;
	R2 = req->r2;

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

		for (i = 0; i < req->size/4; i++) {
			se_elp_writel(se_dev, PKA1, *EXP++,
				reg_bank_offset(TEGRA_SE_ELP_PKA_RSA_EXP_BANK,
					TEGRA_SE_ELP_PKA_RSA_EXP_ID,
					req->op_mode) + (i*4));
			se_elp_writel(se_dev, PKA1, *MSG++,
				reg_bank_offset(TEGRA_SE_ELP_PKA_RSA_MSG_BANK,
					TEGRA_SE_ELP_PKA_RSA_MSG_ID,
					req->op_mode) + (i*4));
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
			for (i = 0; i < req->size/4; i++)
				se_elp_writel(se_dev, PKA1, *MOD++,
				reg_bank_offset(TEGRA_SE_ELP_PKA_MOD_BANK,
				TEGRA_SE_ELP_PKA_MOD_ID, req->op_mode) + (i*4));
		}

		for (i = 0; i < req->size/4; i++) {
			se_elp_writel(se_dev, PKA1, *A++,
				reg_bank_offset(TEGRA_SE_ELP_PKA_ECC_A_BANK,
					TEGRA_SE_ELP_PKA_ECC_A_ID,
					req->op_mode) + (i*4));
		}
		if (req->ecc_type != ECC_POINT_DOUBLE) {
			PX = req->base_pt_x;
			PY = req->base_pt_y;
			for (i = 0; i < req->size/4; i++) {
				se_elp_writel(se_dev, PKA1, *PX++,
				reg_bank_offset(TEGRA_SE_ELP_PKA_ECC_XP_BANK,
					TEGRA_SE_ELP_PKA_ECC_XP_ID,
					req->op_mode) + (i*4));

				se_elp_writel(se_dev, PKA1, *PY++,
				reg_bank_offset(TEGRA_SE_ELP_PKA_ECC_YP_BANK,
					TEGRA_SE_ELP_PKA_ECC_YP_ID,
					req->op_mode) + (i*4));
			}
		}

		if (req->ecc_type == ECC_POINT_VER ||
				req->ecc_type == ECC_SHAMIR_TRICK) {
			/* For shamir trick, curve_param_b is paramater k */
			B = req->curve_param_b;
			for (i = 0; i < req->size/4; i++)
				se_elp_writel(se_dev, PKA1, *B++,
				reg_bank_offset(TEGRA_SE_ELP_PKA_ECC_B_BANK,
					TEGRA_SE_ELP_PKA_ECC_B_ID,
					req->op_mode) + (i*4));
		}

		if (req->ecc_type == ECC_POINT_ADD ||
				req->ecc_type == ECC_SHAMIR_TRICK ||
				req->ecc_type == ECC_POINT_DOUBLE) {
			QX = req->res_pt_x;
			QY = req->res_pt_y;
			for (i = 0; i < req->size/4; i++) {
				se_elp_writel(se_dev, PKA1, *QX++,
				reg_bank_offset(TEGRA_SE_ELP_PKA_ECC_XQ_BANK,
					TEGRA_SE_ELP_PKA_ECC_XQ_ID,
					req->op_mode) + (i*4));
				se_elp_writel(se_dev, PKA1, *QY++,
				reg_bank_offset(TEGRA_SE_ELP_PKA_ECC_YQ_BANK,
					TEGRA_SE_ELP_PKA_ECC_YQ_ID,
					req->op_mode) + (i*4));
			}
		}

		if (req->ecc_type == ECC_POINT_MUL ||
				req->ecc_type == ECC_SHAMIR_TRICK) {
			/* For shamir trick, key is paramater l */
			K = req->key;
			for (i = 0; i < 128/4; i++) {
				se_elp_writel(se_dev, PKA1, *K++,
				reg_bank_offset(TEGRA_SE_ELP_PKA_ECC_K_BANK,
					TEGRA_SE_ELP_PKA_ECC_K_ID,
					req->op_mode) + (i*4));
			}
		}
		break;
	}
}

static u32 pka_ctrl_base(u32 mode)
{
	struct tegra_se_elp_dev *se_dev = elp_dev;
	u32 val = 0, base_radix = 0;

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
		dev_warn(se_dev->dev, "%s: Invalid base radix size\n",
						__func__);
		break;
	}
	return base_radix;
}

static bool is_a_m3(u32 *param_a)
{
	return false;
}

static void tegra_se_program_pka_regs(struct tegra_se_elp_dev *se_dev,
					struct tegra_se_elp_pka_request *req)
{
	u32 val = 0;

	se_elp_writel(se_dev, PKA1, 0, TEGRA_SE_ELP_PKA_FLAGS_OFFSET);
	se_elp_writel(se_dev, PKA1, 0, TEGRA_SE_ELP_PKA_FSTACK_PTR_OFFSET);

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
			TEGRA_SE_ELP_PKA_INT_ENABLE_IE_IRQ_EN(ENABLE),
			TEGRA_SE_ELP_PKA_INT_ENABLE_OFFSET);

		val =
		TEGRA_SE_ELP_PKA_CTRL_BASE_RADIX(pka_ctrl_base(req->op_mode))
			| TEGRA_SE_ELP_PKA_CTRL_PARTIAL_RADIX(req->size/4);
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
			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_ECC_POINT_MUL_PRG_ENTRY_VAL,
				TEGRA_SE_ELP_PKA_PRG_ENTRY_OFFSET);
			if (req->op_mode != SE_ELP_OP_MODE_ECC521) {
				if (is_a_m3(req->curve_param_a))
					se_elp_writel(se_dev, PKA1,
					TEGRA_SE_ELP_PKA_FLAGS_FLAG_F3(ENABLE),
						TEGRA_SE_ELP_PKA_FLAGS_OFFSET);
			}
			/*clear F0 for binding val*/
			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_FLAGS_FLAG_F0(DISABLE),
				TEGRA_SE_ELP_PKA_FLAGS_OFFSET);
		} else if (req->ecc_type == ECC_POINT_ADD) {
			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_ECC_POINT_ADD_PRG_ENTRY_VAL,
				TEGRA_SE_ELP_PKA_PRG_ENTRY_OFFSET);
		} else if (req->ecc_type == ECC_POINT_DOUBLE) {
			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_ECC_POINT_DOUBLE_PRG_ENTRY_VAL,
				TEGRA_SE_ELP_PKA_PRG_ENTRY_OFFSET);
		} else if (req->ecc_type == ECC_POINT_VER) {
			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_ECC_ECPV_PRG_ENTRY_VAL,
				TEGRA_SE_ELP_PKA_PRG_ENTRY_OFFSET);
		} else {
			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_ECC_SHAMIR_TRICK_PRG_ENTRY_VAL,
				TEGRA_SE_ELP_PKA_PRG_ENTRY_OFFSET);

			if (req->op_mode != SE_ELP_OP_MODE_ECC521) {
				if (is_a_m3(req->curve_param_a))
					se_elp_writel(se_dev, PKA1,
					TEGRA_SE_ELP_PKA_FLAGS_FLAG_F3(ENABLE),
						TEGRA_SE_ELP_PKA_FLAGS_OFFSET);
			}
		}

		se_elp_writel(se_dev, PKA1,
			TEGRA_SE_ELP_PKA_INT_ENABLE_IE_IRQ_EN(ENABLE),
			TEGRA_SE_ELP_PKA_INT_ENABLE_OFFSET);

		if (req->op_mode == SE_ELP_OP_MODE_ECC521) {
			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_FLAGS_FLAG_F1(ENABLE),
				TEGRA_SE_ELP_PKA_FLAGS_OFFSET);
		}

		se_elp_writel(se_dev, PKA1,
		TEGRA_SE_ELP_PKA_CTRL_BASE_RADIX(pka_ctrl_base(req->op_mode)) |
			TEGRA_SE_ELP_PKA_CTRL_PARTIAL_RADIX(req->size/4) |
		TEGRA_SE_ELP_PKA_CTRL_GO(TEGRA_SE_ELP_PKA_CTRL_GO_START),
			TEGRA_SE_ELP_PKA_CTRL_OFFSET);
		break;
	default:
		dev_warn(se_dev->dev, "Invalid operation mode\n");
		break;
	}
}

static int tegra_se_check_pka_op_done(struct tegra_se_elp_dev *se_dev)
{
	u32 val = 0;

	val = se_elp_readl(se_dev, PKA1, TEGRA_SE_ELP_PKA_STATUS_OFFSET);
	while (!(val & TEGRA_SE_ELP_PKA_STATUS_IRQ_STAT(ENABLE)))
		val = se_elp_readl(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_STATUS_OFFSET);

	if (!(val & TEGRA_SE_ELP_PKA_STATUS_IRQ_STAT(ENABLE))) {
		dev_err(se_dev->dev, "\nAbrupt end of operation\n");
		return -EINVAL;
	}

	val = se_elp_readl(se_dev, PKA1, TEGRA_SE_ELP_PKA_RETURN_CODE_OFFSET);
	if ((val | 0xFF00FFFF) != 0xFF00FFFF) {
		dev_err(se_dev->dev, "\nPKA Operation ended Abnormally\n");
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
	switch (req->op_mode) {

	case SE_ELP_OP_MODE_RSA512:
	case SE_ELP_OP_MODE_RSA768:
	case SE_ELP_OP_MODE_RSA1024:
	case SE_ELP_OP_MODE_RSA1536:
	case SE_ELP_OP_MODE_RSA2048:
	case SE_ELP_OP_MODE_RSA3072:
	case SE_ELP_OP_MODE_RSA4096:
		for (i = 0; i < req->size/4; i++) {
			val = se_elp_readl(se_dev, PKA1,
			reg_bank_offset(TEGRA_SE_ELP_PKA_RSA_RESULT_BANK,
				TEGRA_SE_ELP_PKA_RSA_RESULT_ID,
				req->op_mode) + (i*4));
			*RES = be32_to_cpu(val);
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
			if (val & TEGRA_SE_ELP_PKA_FLAGS_FLAG_ZERO(ENABLE))
				req->pv_ok = true;
			else
				req->pv_ok = false;
		} else if (req->ecc_type == ECC_POINT_DOUBLE) {
			for (i = 0; i < req->size/4; i++) {
				val = se_elp_readl(se_dev, PKA1,
				reg_bank_offset(TEGRA_SE_ELP_PKA_ECC_XP_BANK,
					TEGRA_SE_ELP_PKA_ECC_XP_ID,
					req->op_mode) + (i*4));
				*QX = be32_to_cpu(val);
				QX++;
			}
			for (i = 0; i < req->size/4; i++) {
				val = se_elp_readl(se_dev, PKA1,
				reg_bank_offset(TEGRA_SE_ELP_PKA_ECC_YP_BANK,
					TEGRA_SE_ELP_PKA_ECC_YP_ID,
					req->op_mode) + (i*4));
				*QY = be32_to_cpu(val);
				QY++;
			}
		} else {
			for (i = 0; i < req->size/4; i++) {
				val = se_elp_readl(se_dev, PKA1,
				reg_bank_offset(TEGRA_SE_ELP_PKA_ECC_XQ_BANK,
					TEGRA_SE_ELP_PKA_ECC_XQ_ID,
					req->op_mode) + (i*4));
				*QX = be32_to_cpu(val);
				QX++;
			}
			for (i = 0; i < req->size/4; i++) {
				val = se_elp_readl(se_dev, PKA1,
				reg_bank_offset(TEGRA_SE_ELP_PKA_ECC_YQ_BANK,
					TEGRA_SE_ELP_PKA_ECC_YQ_ID,
					req->op_mode) + (i*4));
				*QY = be32_to_cpu(val);
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
	enum tegra_se_elp_op_mode mode, struct tegra_se_elp_pka_request *req)
{
	u32 i = 0;
	u32 slot_num = req->slot->slot_num;
	u32 *MOD, *M, *R2, *EXP, *MSG;
	u32 *A, *B, *PX, *PY, *K;

		MOD = req->modulus;
		M = req->m;
		R2 = req->r2;


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
		for (i = 0; i < req->size/4; i++) {
			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD(EXPONENT) |
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET(slot_num));
			se_elp_writel(se_dev, PKA1, *EXP++,
				TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET(slot_num));

			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD(MOD_RSA) |
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET(slot_num));
			se_elp_writel(se_dev, PKA1, *MOD++,
				TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET(slot_num));

			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD(M_RSA) |
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET(slot_num));
			se_elp_writel(se_dev, PKA1, *M++,
				TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET(slot_num));

			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD(R2_RSA) |
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET(slot_num));
			se_elp_writel(se_dev, PKA1, *R2++,
				TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET(slot_num));
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
		for (i = 0; i < req->size/4; i++) {
			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD(PARAM_A) |
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET(slot_num));
			se_elp_writel(se_dev, PKA1, *A++,
				TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET(slot_num));

			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD(PARAM_B) |
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET(slot_num));
			se_elp_writel(se_dev, PKA1, *B++,
				TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET(slot_num));

			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD(MOD_ECC) |
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET(slot_num));
			se_elp_writel(se_dev, PKA1, *MOD++,
				TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET(slot_num));

			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD(XP) |
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET(slot_num));
			se_elp_writel(se_dev, PKA1, *PX++,
				TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET(slot_num));

			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD(YP) |
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET(slot_num));
			se_elp_writel(se_dev, PKA1, *PY++,
				TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET(slot_num));

			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD(KEY) |
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET(slot_num));
			se_elp_writel(se_dev, PKA1, *K++,
				TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET(slot_num));

			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD(M_ECC) |
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET(slot_num));
			se_elp_writel(se_dev, PKA1, *M++,
				TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET(slot_num));

			se_elp_writel(se_dev, PKA1,
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_FIELD(R2_ECC) |
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_WORD(i),
				TEGRA_SE_ELP_PKA_KEYSLOT_ADDR_OFFSET(slot_num));
			se_elp_writel(se_dev, PKA1, *R2++,
				TEGRA_SE_ELP_PKA_KEYSLOT_DATA_OFFSET(slot_num));
		}
		break;
	}
}

static int tegra_se_elp_pka_precomp(struct tegra_se_elp_dev *se_dev,
			struct tegra_se_elp_pka_request *req, u32 op)
{
	int ret = 0, i;
	u32 *MOD = req->modulus;
	u32 *RINV = req->rinv;
	u32 *M = req->m;
	u32 *R2 = req->r2;

	if (req->op_mode == SE_ELP_OP_MODE_ECC521)
		return 0;

	se_elp_writel(se_dev, PKA1, 0, TEGRA_SE_ELP_PKA_FLAGS_OFFSET);
	se_elp_writel(se_dev, PKA1, 0, TEGRA_SE_ELP_PKA_FSTACK_PTR_OFFSET);

	if (op == PRECOMP_RINV) {
		for (i = 0; i < req->size/4; i++) {
			se_elp_writel(se_dev, PKA1, *MOD++,
				reg_bank_offset(TEGRA_SE_ELP_PKA_MOD_BANK,
				TEGRA_SE_ELP_PKA_MOD_ID, req->op_mode) + (i*4));
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
			TEGRA_SE_ELP_PKA_INT_ENABLE_IE_IRQ_EN(ENABLE),
			TEGRA_SE_ELP_PKA_INT_ENABLE_OFFSET);
	se_elp_writel(se_dev, PKA1,
		TEGRA_SE_ELP_PKA_CTRL_BASE_RADIX(pka_ctrl_base(req->op_mode)) |
		TEGRA_SE_ELP_PKA_CTRL_PARTIAL_RADIX(req->size/4) |
		TEGRA_SE_ELP_PKA_CTRL_GO(TEGRA_SE_ELP_PKA_CTRL_GO_START),
		TEGRA_SE_ELP_PKA_CTRL_OFFSET);

	ret = tegra_se_check_pka_op_done(se_dev);
	if (ret)
		return -EINVAL;

	if (op == PRECOMP_RINV) {
		for (i = 0; i < req->size/4; i++) {
			*RINV = se_elp_readl(se_dev, PKA1,
				reg_bank_offset(TEGRA_SE_ELP_PKA_RINV_BANK,
				TEGRA_SE_ELP_PKA_RINV_ID,
					req->op_mode) + (i*4));
			RINV++;
		}
	} else if (op == PRECOMP_M) {
		for (i = 0; i < req->size/4; i++) {
			*M = se_elp_readl(se_dev, PKA1,
				reg_bank_offset(TEGRA_SE_ELP_PKA_M_BANK,
				TEGRA_SE_ELP_PKA_M_ID, req->op_mode) + (i*4));
			M++;
		}
	} else {
		for (i = 0; i < req->size/4; i++) {
			*R2 = se_elp_readl(se_dev, PKA1,
				reg_bank_offset(TEGRA_SE_ELP_PKA_R2_BANK,
				TEGRA_SE_ELP_PKA_R2_ID, req->op_mode) + (i*4));
			R2++;
		}
	}
	return ret;
}

static int tegra_se_elp_pka_do(struct tegra_se_elp_dev *se_dev,
				struct tegra_se_elp_pka_request *req)
{
	int ret = 0;
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
		val |= TEGRA_SE_ELP_PKA_CTRL_CONTROL_LOAD_KEY(ENABLE);
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

	tegra_se_check_pka_op_done(se_dev);

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

	req->rinv = kzalloc(req->size, GFP_KERNEL);
	if (!req->rinv) {
		dev_err(se_dev->dev, "\n%s: Memory alloc fail for req->rinv\n",
					__func__);
		return -ENOMEM;
	}

	req->m = kzalloc(req->size, GFP_KERNEL);
	if (!req->m) {
		dev_err(se_dev->dev, "\n%s: Memory alloc fail for req->m\n",
					__func__);
		kfree(req->rinv);
		return -ENOMEM;
	}

	req->r2 = kzalloc(req->size, GFP_KERNEL);
	if (!req->r2) {
		dev_err(se_dev->dev, "\n%s: Memory alloc fail for req->r2\n",
					__func__);
		kfree(req->m);
		kfree(req->rinv);
		return -ENOMEM;
	}

	return 0;
}

static void tegra_se_elp_pka_exit(struct tegra_se_elp_pka_request *req)
{
	if (req->op_mode == SE_ELP_OP_MODE_ECC521)
		return;

	kfree(req->r2);
	kfree(req->m);
	kfree(req->rinv);
}

int tegra_se_elp_pka_op(struct tegra_se_elp_pka_request *req)
{
	struct tegra_se_elp_dev *se_dev = elp_dev;
	int ret = 0;

	ret = tegra_se_acquire_pka_mutex(se_dev);
	if (ret) {
		dev_err(se_dev->dev, "\nPKA Mutex acquire failed\n");
		return ret;
	}

	ret = tegra_se_elp_pka_init(req);
	if (ret)
		goto mutex_rel;

	ret = tegra_se_check_trng_op(se_dev);
	if (ret)
		ret = tegra_se_set_trng_op(se_dev);
	if (ret) {
		dev_err(se_dev->dev, "\nset_trng_op Failed\n");
		goto exit;
	}
	ret = tegra_se_elp_pka_precomp(se_dev, req, PRECOMP_RINV);
	if (ret) {
		dev_err(se_dev->dev,
		"\ntegra_se_elp_pka_precomp Failed(%d) for RINV\n", ret);
		goto exit;
	}
	ret = tegra_se_elp_pka_precomp(se_dev, req, PRECOMP_M);
	if (ret) {
		dev_err(se_dev->dev,
			"\ntegra_se_elp_pka_precomp Failed(%d) for M\n", ret);
		goto exit;
	}
	ret = tegra_se_elp_pka_precomp(se_dev, req, PRECOMP_R2);
	if (ret) {
		dev_err(se_dev->dev,
			"\ntegra_se_elp_pka_precomp Failed(%d) for R2\n", ret);
		goto exit;
	}
	tegra_se_elp_pka_do(se_dev, req);
exit:
	tegra_se_elp_pka_exit(req);
mutex_rel:
	tegra_se_release_pka_mutex(se_dev);
	return ret;
}
EXPORT_SYMBOL(tegra_se_elp_pka_op);

static struct tegra_se_chipdata tegra18_se_chipdata = {
	.use_key_slot = false,
};

static struct of_device_id tegra_se_elp_of_match[] = {
	{
		.compatible = "nvidia,tegra186-se-elp",
		.data = &tegra18_se_chipdata,
	},
};
MODULE_DEVICE_TABLE(of, tegra_se_elp_of_match);

static int tegra_se_elp_probe(struct platform_device *pdev)
{
	struct tegra_se_elp_dev *se_dev = NULL;
	struct resource *res = NULL;
	const struct of_device_id *match;
	int err = 0;

	se_dev = kzalloc(sizeof(struct tegra_se_elp_dev), GFP_KERNEL);
	if (!se_dev) {
		dev_err(&pdev->dev, "se_dev memory allocation failed\n");
		return -ENOMEM;
	}

	if (pdev->dev.of_node) {
		match = of_match_device(of_match_ptr(tegra_se_elp_of_match),
				&pdev->dev);
		if (!match) {
			dev_err(&pdev->dev, "Error: No device match found\n");
			kfree(se_dev);
			return -ENODEV;
		}
		se_dev->chipdata = (struct tegra_se_chipdata *)match->data;
	} else {
		se_dev->chipdata =
			(struct tegra_se_chipdata *)pdev->id_entry->driver_data;
	}

	platform_set_drvdata(pdev, se_dev);
	se_dev->dev = &pdev->dev;
	se_dev->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err = -ENXIO;
		dev_err(se_dev->dev, "platform_get_resource failed for pka1\n");
		goto fail;
	}

	se_dev->io_reg[0] = ioremap(res->start, resource_size(res));
	if (!se_dev->io_reg[0]) {
		err = -ENOMEM;
		dev_err(se_dev->dev, "ioremap failed for pka1\n");
		goto fail;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		err = -ENXIO;
		dev_err(se_dev->dev, "platform_get_resource failed for rng1\n");
		goto res_fail;
	}

	se_dev->io_reg[1] = ioremap(res->start, resource_size(res));
	if (!se_dev->io_reg[1]) {
		err = -ENOMEM;
		dev_err(se_dev->dev, "ioremap failed for rng1\n");
		goto res_fail;
	}

	elp_dev = se_dev;

	err = tegra_se_pka_init_key_slot();
	if (err) {
		dev_err(se_dev->dev, "tegra_se_pka_init_key_slot failed\n");
		goto kslt_fail;
	}

	dev_info(se_dev->dev, "%s: complete", __func__);
	return 0;

kslt_fail:
	iounmap(se_dev->io_reg[1]);
res_fail:
	iounmap(se_dev->io_reg[0]);
fail:
	platform_set_drvdata(pdev, NULL);
	kfree(se_dev);
	elp_dev = NULL;

	return err;
}

static int tegra_se_elp_remove(struct platform_device *pdev)
{
	struct tegra_se_elp_dev *se_dev = platform_get_drvdata(pdev);
	int i;

	if (!se_dev)
		return -ENODEV;
	for (i = 0; i < 2; i++)
		iounmap(se_dev->io_reg[i]);
	kfree(se_dev);
	elp_dev = NULL;

	return 0;
}

static struct platform_device_id tegra_dev_se_elp_devtype[] = {
	{
		.name = "tegra-se-elp",
		.driver_data = (unsigned long)&tegra18_se_chipdata,
	}
};

static struct platform_driver tegra_se_elp_driver = {
	.probe  = tegra_se_elp_probe,
	.remove = tegra_se_elp_remove,
	.id_table = tegra_dev_se_elp_devtype,
	.driver = {
		.name   = "tegra-se-elp",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_se_elp_of_match),
	},
};

static int __init tegra_se_elp_module_init(void)
{
	return  platform_driver_register(&tegra_se_elp_driver);
}

static void __exit tegra_se_elp_module_exit(void)
{
	platform_driver_unregister(&tegra_se_elp_driver);
}

module_init(tegra_se_elp_module_init);
module_exit(tegra_se_elp_module_exit);

MODULE_DESCRIPTION("Tegra Elliptic Crypto algorithm support");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL");
MODULE_ALIAS("tegra-se-elp");

