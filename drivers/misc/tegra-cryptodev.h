/*
 * Copyright (c) 2010-2014, NVIDIA Corporation. All Rights Reserved.
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

#ifdef CONFIG_COMPAT
#define user_ptr(p) ((void __user *)(__u64)(p))
#else
#define user_ptr(p) (p)
#endif

/* ioctl arg = 1 if you want to use ssk. arg = 0 to use normal key */
#define TEGRA_CRYPTO_IOCTL_NEED_SSK	_IOWR(0x98, 100, int)
#define TEGRA_CRYPTO_IOCTL_PROCESS_REQ	_IOWR(0x98, 101, int*)
#define TEGRA_CRYPTO_IOCTL_SET_SEED	_IOWR(0x98, 102, int*)
#define TEGRA_CRYPTO_IOCTL_GET_RANDOM	_IOWR(0x98, 103, int*)
#define TEGRA_CRYPTO_IOCTL_GET_SHA	_IOWR(0x98, 104, int*)
#define TEGRA_CRYPTO_IOCTL_RSA_REQ	_IOWR(0x98, 105, int*)

#define TEGRA_CRYPTO_MAX_KEY_SIZE	AES_MAX_KEY_SIZE
#define RSA_KEY_SIZE		512
#define TEGRA_CRYPTO_IV_SIZE	AES_BLOCK_SIZE
#define DEFAULT_RNG_BLK_SZ	16

/* the seed consists of 16 bytes of key + 16 bytes of init vector */
#define TEGRA_CRYPTO_RNG_SEED_SIZE	AES_KEYSIZE_128 + DEFAULT_RNG_BLK_SZ
#define TEGRA_CRYPTO_RNG_SIZE	SZ_16

/* encrypt/decrypt operations */
#define TEGRA_CRYPTO_ECB	BIT(0)
#define TEGRA_CRYPTO_CBC	BIT(1)
#define TEGRA_CRYPTO_OFB	BIT(2)
#define TEGRA_CRYPTO_CTR	BIT(3)
#define TEGRA_CRYPTO_CMAC	BIT(4)
#define TEGRA_CRYPTO_RNG	BIT(5)

/* a pointer to this struct needs to be passed to:
 * TEGRA_CRYPTO_IOCTL_PROCESS_REQ
 */
struct tegra_crypt_req {
	int op; /* e.g. TEGRA_CRYPTO_ECB */
	bool encrypt;
	char key[TEGRA_CRYPTO_MAX_KEY_SIZE];
	int keylen;
	char iv[TEGRA_CRYPTO_IV_SIZE];
	int ivlen;
#ifdef CONFIG_COMPAT
	u32 plaintext;
	u32 result;
#else
	u8 *plaintext;
	u8 *result;
#endif
	int plaintext_sz;
	int skip_key;
	int skip_iv;
};

/* pointer to this struct should be passed to:
 * TEGRA_CRYPTO_IOCTL_SET_SEED
 * TEGRA_CRYPTO_IOCTL_GET_RANDOM
 */
struct tegra_rng_req {
	u8 seed[TEGRA_CRYPTO_RNG_SEED_SIZE];
#ifdef CONFIG_COMPAT
	u32 rdata; /* random generated data */
#else
	u8 *rdata; /* random generated data */
#endif
	int nbytes; /* random data length */
	int type;
};

struct tegra_rsa_req {
#ifdef CONFIG_COMPAT
	u32 key;
	u32 message;
	u32 result;
#else
	char *key;
	char *message;
	char *result;
#endif
	int algo;
	int keylen;
	int msg_len;
	int modlen;
	int pub_explen;
	int prv_explen;
	int skip_key;
};

struct tegra_sha_req {
	char key[TEGRA_CRYPTO_MAX_KEY_SIZE];
	int keylen;
#ifdef CONFIG_COMPAT
	u32 algo;
	u32 plaintext;
	u32 result;
#else
	unsigned char *algo;
	unsigned char *plaintext;
	unsigned char *result;
#endif
	int plaintext_sz;
};

#endif
