/*
 * Copyright (c) 2010-2016, NVIDIA Corporation. All Rights Reserved.
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

#ifndef __TEGRA_CRYPTODEV_H
#define __TEGRA_CRYPTODEV_H

#include <crypto/aes.h>

#include <asm-generic/ioctl.h>

/* ioctl arg = 1 if you want to use ssk. arg = 0 to use normal key */
#define TEGRA_CRYPTO_IOCTL_NEED_SSK		_IOWR(0x98, 100, int)

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#define TEGRA_CRYPTO_IOCTL_ELP_RSA_REQ	\
		_IOWR(0x98, 106, struct tegra_se_elp_pka_request)
#define TEGRA_CRYPTO_IOCTL_ELP_ECC_REQ	\
		_IOWR(0x98, 107, struct tegra_se_elp_pka_request)
#define TEGRA_CRYPTO_IOCTL_ELP_RNG_REQ	\
		_IOWR(0x98, 108, struct tegra_se_elp_rng_request)
#endif

#define TEGRA_CRYPTO_MAX_KEY_SIZE	AES_MAX_KEY_SIZE
#define RSA_KEY_SIZE		512
#define TEGRA_CRYPTO_IV_SIZE	AES_BLOCK_SIZE
#define DEFAULT_RNG_BLK_SZ	16
#define TEGRA_CRYPTO_KEY_256_SIZE	32
#define TEGRA_CRYPTO_KEY_192_SIZE	24
#define TEGRA_CRYPTO_KEY_128_SIZE	16

#define CRYPTO_KEY_LEN_MASK	0x3FF

/* the seed consists of 16 bytes of key + 16 bytes of init vector */
#define TEGRA_CRYPTO_RNG_SEED_SIZE	AES_KEYSIZE_128 + DEFAULT_RNG_BLK_SZ
#define TEGRA_CRYPTO_RNG_SIZE	SZ_16

#define TEGRA_CRYPTO_ECB	0
#define TEGRA_CRYPTO_CBC	1
#define TEGRA_CRYPTO_OFB	2
#define TEGRA_CRYPTO_CTR	3

#ifdef CONFIG_ARCH_TEGRA_18x_SOC

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
	struct tegra_se_elp_dev *se_dev;	/* Security Engine device */
	struct tegra_se_slot *slot;	/* Security Engine key slot */
	char *message;
	char *result;
	char *exponent;
	char *modulus;
	char *m;
	char *r2;
	char *rinv;
	int op_mode;
	int size;
	int ecc_type;
	int rsa_type;
	char *curve_param_a;
	char *curve_param_b;
	char *order;
	char *base_pt_x;
	char *base_pt_y;
	char *res_pt_x;
	char *res_pt_y;
	char *key;
	bool pv_ok;
};

int tegra_se_elp_pka_op(struct tegra_se_elp_pka_request *req);
int tegra_se_elp_rng_op(struct tegra_se_elp_rng_request *req);
#endif

/* a pointer to this struct needs to be passed to:
 * TEGRA_CRYPTO_IOCTL_PROCESS_REQ
 */
struct tegra_crypt_req {
	int op; /* e.g. TEGRA_CRYPTO_ECB */
	bool encrypt;
	char key[TEGRA_CRYPTO_MAX_KEY_SIZE];
	unsigned int keylen;
	char iv[TEGRA_CRYPTO_IV_SIZE];
	int ivlen;
	u8 *plaintext;
	u8 *result;
	int plaintext_sz;
	int skip_key;
	int skip_iv;
};
#define TEGRA_CRYPTO_IOCTL_PROCESS_REQ	\
		_IOWR(0x98, 101, struct tegra_crypt_req)

#ifdef CONFIG_COMPAT
struct tegra_crypt_req_32 {
	int op; /* e.g. TEGRA_CRYPTO_ECB */
	bool encrypt;
	char key[TEGRA_CRYPTO_MAX_KEY_SIZE];
	unsigned int keylen;
	char iv[TEGRA_CRYPTO_IV_SIZE];
	int ivlen;
	__u32 plaintext;
	__u32 result;
	int plaintext_sz;
	int skip_key;
	int skip_iv;
};
#define TEGRA_CRYPTO_IOCTL_PROCESS_REQ_32	\
		_IOWR(0x98, 101, struct tegra_crypt_req_32)
#endif

/* pointer to this struct should be passed to:
 * TEGRA_CRYPTO_IOCTL_SET_SEED
 * TEGRA_CRYPTO_IOCTL_GET_RANDOM
 */
struct tegra_rng_req {
	u8 seed[TEGRA_CRYPTO_RNG_SEED_SIZE];
	u8 *rdata; /* random generated data */
	int nbytes; /* random data length */
	int type;
};
#define TEGRA_CRYPTO_IOCTL_SET_SEED	\
		_IOWR(0x98, 102, struct tegra_rng_req)
#define TEGRA_CRYPTO_IOCTL_GET_RANDOM	\
		_IOWR(0x98, 103, struct tegra_rng_req)

#ifdef CONFIG_COMPAT
struct tegra_rng_req_32 {
	u8 seed[TEGRA_CRYPTO_RNG_SEED_SIZE];
	__u32 rdata; /* random generated data */
	int nbytes; /* random data length */
	int type;
};
#define TEGRA_CRYPTO_IOCTL_SET_SEED_32	\
		_IOWR(0x98, 102, struct tegra_rng_req_32)
#define TEGRA_CRYPTO_IOCTL_GET_RANDOM_32	\
		_IOWR(0x98, 103, struct tegra_rng_req_32)
#endif

enum tegra_rsa_op_mode {
	RSA_INIT,
	RSA_SET_PUB,
	RSA_SET_PRIV,
	RSA_ENCRYPT,
	RSA_DECRYPT,
	RSA_SIGN,
	RSA_VERIFY,
	RSA_EXIT,
};

struct tegra_rsa_req {
	char *key;
	char *message;
	char *result;
	int algo;
	int keylen;
	int msg_len;
	int modlen;
	int pub_explen;
	int prv_explen;
	int skip_key;
	enum tegra_rsa_op_mode op_mode;
};
#define TEGRA_CRYPTO_IOCTL_RSA_REQ	\
		_IOWR(0x98, 105, struct tegra_rsa_req)

struct tegra_rsa_req_ahash {
	char *key;
	char *message;
	char *result;
	int algo;
	int keylen;
	int msg_len;
	int modlen;
	int pub_explen;
	int prv_explen;
	int skip_key;
};
#define TEGRA_CRYPTO_IOCTL_RSA_REQ_AHASH	\
		_IOWR(0x98, 110, struct tegra_rsa_req_ahash)

#ifdef CONFIG_COMPAT
struct tegra_rsa_req_32 {
	__u32 key;
	__u32 message;
	__u32 result;
	int algo;
	int keylen;
	int msg_len;
	int modlen;
	int pub_explen;
	int prv_explen;
	int skip_key;
};
#define TEGRA_CRYPTO_IOCTL_RSA_REQ_32	\
		_IOWR(0x98, 105, struct tegra_rsa_req_32)
#endif

struct tegra_sha_req {
	char key[TEGRA_CRYPTO_MAX_KEY_SIZE];
	int keylen;
	unsigned char *algo;
	unsigned char *plaintext;
	unsigned char *result;
	int plaintext_sz;
};
#define TEGRA_CRYPTO_IOCTL_GET_SHA	\
		_IOWR(0x98, 104, struct tegra_sha_req)

#ifdef CONFIG_COMPAT
struct tegra_sha_req_32 {
	char key[TEGRA_CRYPTO_MAX_KEY_SIZE];
	int keylen;
	__u32 algo;
	__u32 plaintext;
	__u32 result;
	int plaintext_sz;
};
#define TEGRA_CRYPTO_IOCTL_GET_SHA_32	\
		_IOWR(0x98, 104, struct tegra_sha_req_32)
#endif

#endif
