/*
 * drivers/misc/tegra-cryptodev.c
 *
 * crypto dev node for NVIDIA tegra aes hardware
 *
 * Copyright (c) 2010-2017, NVIDIA Corporation. All Rights Reserved.
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
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/crypto.h>
#include <linux/scatterlist.h>
#include <linux/uaccess.h>
#include <soc/tegra/chip-id.h>
#include <crypto/rng.h>
#include <crypto/hash.h>
#include <linux/platform/tegra/common.h>
#include <soc/tegra/fuse.h>
#include <crypto/akcipher.h>

#include "tegra-cryptodev.h"

#define NBUFS 2
#define XBUFSIZE 8
#define RNG_DRBG 1
#define RNG 0

struct tegra_crypto_ctx {
	/*ecb, cbc, ofb, ctr */
	struct crypto_ablkcipher *aes_tfm[4];
	/* rsa512, rsa1024, rsa1536, rsa2048 */
	struct crypto_akcipher *rsa_tfm[4];
	/* rsa512, rsa768, rsa1024, rsa1536, rsa2048, rsa3072, rsa4096 */
	struct crypto_akcipher *pka1_rsa_tfm;
	/* sha1, sha224, sha256, sha384, sha512, cmac */
	struct crypto_ahash *sha_tfm[6];
	struct crypto_rng *rng;
	struct crypto_rng *rng_drbg;
	u8 seed[TEGRA_CRYPTO_RNG_SEED_SIZE];
	int use_ssk;
};

struct tegra_crypto_completion {
	struct completion restart;
	int req_err;
};

static int alloc_bufs(unsigned long *buf[NBUFS])
{
	int i;

	for (i = 0; i < NBUFS; i++) {
		buf[i] = (void *)__get_free_page(GFP_KERNEL);
		if (!buf[i])
			goto err_free_buf;
	}

	return 0;

err_free_buf:
	while (i-- > 0)
		free_page((unsigned long)buf[i]);

	return -ENOMEM;
}

static void free_bufs(unsigned long *buf[NBUFS])
{
	int i;

	for (i = 0; i < NBUFS; i++)
		free_page((unsigned long)buf[i]);
}

static int tegra_crypto_dev_open(struct inode *inode, struct file *filp)
{
	struct tegra_crypto_ctx *ctx;
	int ret = 0;

	ctx = kzalloc(sizeof(struct tegra_crypto_ctx), GFP_KERNEL);
	if (!ctx) {
		pr_err("no memory for context\n");
		return -ENOMEM;
	}

	/* CBC tfm is allocated during device_open itself
	 * for (LP0) CTX_SAVE test that is performed using CBC
	 */
	ctx->aes_tfm[TEGRA_CRYPTO_CBC] =
		crypto_alloc_ablkcipher("cbc-aes-tegra",
		CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC, 0);
	if (IS_ERR(ctx->aes_tfm[TEGRA_CRYPTO_CBC])) {
		pr_err("Failed to load transform for cbc-aes-tegra: %ld\n",
			PTR_ERR(ctx->aes_tfm[TEGRA_CRYPTO_CBC]));
		ret = PTR_ERR(ctx->aes_tfm[TEGRA_CRYPTO_CBC]);
		kfree(ctx);
		return ret;
	}
	filp->private_data = ctx;
	return ret;
}

static int tegra_crypto_dev_release(struct inode *inode, struct file *filp)
{
	struct tegra_crypto_ctx *ctx = filp->private_data;

	if (ctx->aes_tfm[TEGRA_CRYPTO_CBC])
		crypto_free_ablkcipher(ctx->aes_tfm[TEGRA_CRYPTO_CBC]);
	kfree(ctx);
	filp->private_data = NULL;
	return 0;
}

static void tegra_crypt_complete(struct crypto_async_request *req, int err)
{
	struct tegra_crypto_completion *done = req->data;

	if (err != -EINPROGRESS) {
		done->req_err = err;
		complete(&done->restart);
	}
}

static int process_crypt_req(struct file *filp, struct tegra_crypto_ctx *ctx,
				struct tegra_crypt_req *crypt_req)
{
	struct crypto_ablkcipher *tfm;
	struct ablkcipher_request *req = NULL;
	struct scatterlist in_sg;
	struct scatterlist out_sg;
	unsigned long *xbuf[NBUFS];
	int ret = 0, size = 0;
	unsigned long total = 0;
	const u8 *key = NULL;
	struct tegra_crypto_completion tcrypt_complete;
	char aes_algo[5][10] = {"ecb(aes)", "cbc(aes)", "ofb(aes)", "ctr(aes)"};

	if (crypt_req->op != TEGRA_CRYPTO_CBC) {
		if (crypt_req->op >= TEGRA_CRYPTO_MAX)
			return -EINVAL;

		tfm = crypto_alloc_ablkcipher(aes_algo[crypt_req->op],
			CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC, 0);
		if (IS_ERR(tfm)) {
			pr_err("Failed to load transform for %s: %ld\n",
				aes_algo[crypt_req->op], PTR_ERR(tfm));
			ret = PTR_ERR(tfm);
			goto out;
		}

		ctx->aes_tfm[crypt_req->op] = tfm;
		filp->private_data = ctx;
	} else {
		tfm = ctx->aes_tfm[TEGRA_CRYPTO_CBC];
	}

	req = ablkcipher_request_alloc(tfm, GFP_KERNEL);
	if (!req) {
		pr_err("%s: Failed to allocate request\n", __func__);
		ret = -ENOMEM;
		goto free_tfm;
	}

	if (((crypt_req->keylen &
		CRYPTO_KEY_LEN_MASK) != TEGRA_CRYPTO_KEY_128_SIZE) &&
		((crypt_req->keylen &
		CRYPTO_KEY_LEN_MASK) != TEGRA_CRYPTO_KEY_192_SIZE) &&
		((crypt_req->keylen &
		CRYPTO_KEY_LEN_MASK) != TEGRA_CRYPTO_KEY_256_SIZE)) {
		ret = -EINVAL;
		pr_err("crypt_req keylen invalid");
		goto process_req_out;
	}

	crypto_ablkcipher_clear_flags(tfm, ~0);

	if (!ctx->use_ssk)
		key = crypt_req->key;

	if (!crypt_req->skip_key) {
		ret = crypto_ablkcipher_setkey(tfm, key, crypt_req->keylen);
		if (ret < 0) {
			pr_err("setkey failed");
			goto process_req_out;
		}
	}

	ret = alloc_bufs(xbuf);
	if (ret < 0) {
		pr_err("alloc_bufs failed");
		goto process_req_out;
	}

	init_completion(&tcrypt_complete.restart);

	ablkcipher_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,
		tegra_crypt_complete, &tcrypt_complete);

	total = crypt_req->plaintext_sz;
	while (total > 0) {
		size = min(total, PAGE_SIZE);
		ret = copy_from_user((void *)xbuf[0],
			(void __user *)crypt_req->plaintext, size);
		if (ret) {
			ret = -EFAULT;
			pr_debug("%s: copy_from_user failed (%d)\n", __func__, ret);
			goto process_req_buf_out;
		}
		sg_init_one(&in_sg, xbuf[0], size);
		sg_init_one(&out_sg, xbuf[1], size);

		if (!crypt_req->skip_iv)
			ablkcipher_request_set_crypt(req, &in_sg,
				&out_sg, size, crypt_req->iv);
		else
			ablkcipher_request_set_crypt(req, &in_sg,
				&out_sg, size, NULL);

		reinit_completion(&tcrypt_complete.restart);

		tcrypt_complete.req_err = 0;

		ret = crypt_req->encrypt ?
			crypto_ablkcipher_encrypt(req) :
			crypto_ablkcipher_decrypt(req);
		if ((ret == -EINPROGRESS) || (ret == -EBUSY)) {
			/* crypto driver is asynchronous */
			ret = wait_for_completion_interruptible(&tcrypt_complete.restart);

			if (ret < 0)
				goto process_req_buf_out;

			if (tcrypt_complete.req_err < 0) {
				ret = tcrypt_complete.req_err;
				goto process_req_buf_out;
			}
		} else if (ret < 0) {
			pr_debug("%scrypt failed (%d)\n",
				crypt_req->encrypt ? "en" : "de", ret);
			goto process_req_buf_out;
		}

		ret = copy_to_user((void __user *)crypt_req->result,
			(const void *)xbuf[1], size);
		if (ret) {
			ret = -EFAULT;
			pr_debug("%s: copy_to_user failed (%d)\n", __func__,
					ret);
			goto process_req_buf_out;
		}

		total -= size;
		crypt_req->result += size;
		crypt_req->plaintext += size;
	}

process_req_buf_out:
	free_bufs(xbuf);
process_req_out:
	ablkcipher_request_free(req);
free_tfm:
	if (crypt_req->op != TEGRA_CRYPTO_CBC)
		crypto_free_ablkcipher(tfm);
out:
	return ret;
}

static int sha_async_hash_op(struct ahash_request *req,
				struct tegra_crypto_completion *tr,
				int ret)
{
	if (ret == -EINPROGRESS || ret == -EBUSY) {
		ret = wait_for_completion_interruptible(&tr->restart);
		if (!ret)
			ret = tr->req_err;
		reinit_completion(&tr->restart);
	}
	return ret;
}

static int tegra_crypt_rsa(struct file *filp, struct tegra_crypto_ctx *ctx,
				struct tegra_rsa_req *rsa_req)
{
	struct crypto_akcipher *tfm = NULL;
	struct akcipher_request *req;
	struct scatterlist sg[2];
	void *src_buff, *dst_buff;
	int ret = 0;
	unsigned long *xbuf[XBUFSIZE];
	struct tegra_crypto_completion rsa_complete;

	if (rsa_req->op_mode == RSA_INIT) {
		tfm = crypto_alloc_akcipher("rsa-pka0",
					CRYPTO_ALG_TYPE_AKCIPHER, 0);
		if (IS_ERR(tfm)) {
			pr_err("Failed to load transform for rsa-pka0: %ld\n",
					PTR_ERR(tfm));
			ret = PTR_ERR(tfm);
			goto out;
		}

		ctx->rsa_tfm[rsa_req->algo] = tfm;
		filp->private_data = ctx;
		goto out;
	} else {
		ctx = filp->private_data;
		tfm =  ctx->rsa_tfm[rsa_req->algo];
	}

	if (rsa_req->op_mode == RSA_SET_PUB) {
		if (!(rsa_req->keylen)) {
			ret = -EINVAL;
			goto out;
		}

		if (!rsa_req->skip_key) {
			ret = crypto_akcipher_set_pub_key(tfm,
					rsa_req->key, rsa_req->keylen);
			if (ret) {
				pr_err("alg: rsa: set_pub_key failed\n");
				goto out;
			}
		}
	} else if (rsa_req->op_mode == RSA_SET_PRIV) {
		if (!(rsa_req->keylen)) {
			ret = -EINVAL;
			goto out;
		}

		if (!rsa_req->skip_key) {
			ret = crypto_akcipher_set_priv_key(tfm,
					rsa_req->key, rsa_req->keylen);
			if (ret) {
				pr_err("alg: rsa: set_priv_key failed\n");
				goto out;
			}
		}
	} else if (rsa_req->op_mode == RSA_ENCRYPT ||
		rsa_req->op_mode == RSA_DECRYPT	|| rsa_req->op_mode == RSA_SIGN
		|| rsa_req->op_mode == RSA_VERIFY) {
		req = akcipher_request_alloc(tfm, GFP_KERNEL);
		if (!req) {
			pr_err("alg: rsa: Failed to allocate request: %s\n",
					__func__);
			ret = -ENOMEM;
			goto out;
		}

		ret = alloc_bufs(xbuf);
		if (ret < 0) {
			pr_err("alloc_bufs failed");
			goto buf_fail;
		}

		init_completion(&rsa_complete.restart);
		rsa_complete.req_err = 0;

		src_buff = xbuf[0];
		dst_buff = xbuf[1];

		ret = copy_from_user(src_buff, (void __user *)rsa_req->message,
			rsa_req->msg_len);
		if (ret) {
			ret = -EFAULT;
			pr_err("%s: copy_from_user failed\n", __func__);
			goto rsa_fail;
		}

		memset(dst_buff, 0, rsa_req->msg_len);

		sg_init_one(&sg[0], src_buff, rsa_req->msg_len);
		sg_init_one(&sg[1], dst_buff, rsa_req->msg_len);

		akcipher_request_set_crypt(req, &sg[0], &sg[1],
					rsa_req->msg_len, rsa_req->msg_len);

		if (rsa_req->op_mode == RSA_ENCRYPT) {
			ret = crypto_akcipher_encrypt(req);
			if (ret) {
				pr_err("alg: rsa: encrypt failed\n");
				goto rsa_fail;
			}
		} else if (rsa_req->op_mode == RSA_DECRYPT) {
			ret = crypto_akcipher_decrypt(req);
			if (ret) {
				pr_err("alg: rsa: decrypt failed\n");
				goto rsa_fail;
			}
		} else if (rsa_req->op_mode == RSA_SIGN) {
			ret = crypto_akcipher_sign(req);
			if (ret) {
				pr_err("alg: rsa: sign failed\n");
				goto rsa_fail;
			}
		} else if (rsa_req->op_mode == RSA_VERIFY) {
			ret = crypto_akcipher_verify(req);
			if (ret) {
				pr_err("alg: rsa: verification failed\n");
				goto rsa_fail;
			}
		}

		ret = copy_to_user((void __user *)rsa_req->result,
				(const void *)xbuf[1], rsa_req->msg_len);
		if (ret) {
			ret = -EFAULT;
			pr_err("alg: rsa: copy_to_user failed (%d)\n", ret);
		}
rsa_fail:
		free_bufs(xbuf);
buf_fail:
		akcipher_request_free(req);
	} else if (rsa_req->op_mode == RSA_EXIT) {
		crypto_free_akcipher(tfm);
	} else {
		pr_err("alg: rsa: invalid rsa operation\n");
	}
out:
	return ret;
}

static int tegra_crypt_rsa_ahash(struct file *filp, struct tegra_crypto_ctx *ctx,
				struct tegra_rsa_req_ahash *rsa_req_ah)
{
	struct crypto_ahash *tfm = NULL;
	struct ahash_request *req = NULL;
	struct scatterlist sg[1];
	char *result = NULL;
	void *hash_buff;
	int ret = 0;
	unsigned long *xbuf[XBUFSIZE];
	struct tegra_crypto_completion rsa_complete;
	char rsa_algo[4][10] = {"rsa512", "rsa1024", "rsa1536", "rsa2048"};

	tfm = crypto_alloc_ahash(rsa_algo[rsa_req_ah->algo],
					CRYPTO_ALG_TYPE_AHASH, 0);
	if (IS_ERR(tfm)) {
	pr_err("Failed to load transform for %s: %ld\n",
			rsa_algo[rsa_req_ah->algo], PTR_ERR(tfm));
		ret = PTR_ERR(tfm);
		goto out;
	}

	filp->private_data = ctx;

	req = ahash_request_alloc(tfm, GFP_KERNEL);
	if (!req) {
		pr_err("alg: hash: Failed to allocate request: %s\n", __func__);
		ret = -ENOMEM;
		goto req_fail;
	}
	ret = alloc_bufs(xbuf);
	 if (ret < 0) {
		pr_err("alloc_bufs failed");
		goto buf_fail;
	}

	init_completion(&rsa_complete.restart);
	rsa_complete.req_err = 0;

	result = kzalloc(rsa_req_ah->keylen >> 16, GFP_KERNEL);
	if (!result) {
		pr_err("\nresult alloc fail\n");
		goto result_fail;
	}

	hash_buff = xbuf[0];

	ret = copy_from_user(hash_buff, (void __user *)rsa_req_ah->message,
		rsa_req_ah->msg_len);
	if (ret) {
		ret = -EFAULT;
		pr_err("%s: copy_from_user failed\n", __func__);
		goto rsa_fail;
	}

	sg_init_one(&sg[0], hash_buff, rsa_req_ah->msg_len);
	if (!(rsa_req_ah->keylen))
		goto rsa_fail;

	if (!rsa_req_ah->skip_key) {
		ret = crypto_ahash_setkey(tfm,
				rsa_req_ah->key, rsa_req_ah->keylen);
		if (ret) {
			pr_err("alg: hash: setkey failed\n");
			goto rsa_fail;
		}
	}

	ahash_request_set_crypt(req, sg, result, rsa_req_ah->msg_len);

	ret = crypto_ahash_digest(req);

	if (ret == -EINPROGRESS || ret == -EBUSY) {
		ret = wait_for_completion_interruptible(&rsa_complete.restart);
		if (!ret)
			ret = rsa_complete.req_err;
		reinit_completion(&rsa_complete.restart);
	}

	if (ret) {
		pr_err("alg: hash: digest failed\n");
	goto rsa_fail;
	}

	ret = copy_to_user((void __user *)rsa_req_ah->result, (const void *)result,
		crypto_ahash_digestsize(tfm));
	if (ret) {
		ret = -EFAULT;
	pr_err("alg: hash: copy_to_user failed (%d)\n", ret);
	}

rsa_fail:
	kfree(result);
result_fail:
	free_bufs(xbuf);
buf_fail:
	ahash_request_free(req);
req_fail:
	crypto_free_ahash(tfm);
out:
	return ret;
}

static int tegra_crypt_pka1_rsa(struct file *filp, struct tegra_crypto_ctx *ctx,
				struct tegra_pka1_rsa_request *rsa_req)
{
	struct crypto_akcipher *tfm = NULL;
	struct akcipher_request *req;
	struct scatterlist sg[2];
	int ret = 0;
	int len;
	unsigned long *xbuf[XBUFSIZE];
	struct tegra_crypto_completion rsa_complete;

	if (rsa_req->op_mode == RSA_INIT) {
		tfm = crypto_alloc_akcipher("rsa", CRYPTO_ALG_TYPE_AKCIPHER, 0);
		if (IS_ERR(tfm)) {
			pr_err("Failed to load transform for rsa: %ld\n",
				PTR_ERR(tfm));
			return PTR_ERR(tfm);
		}
		ctx->pka1_rsa_tfm = tfm;
		filp->private_data = ctx;

		return 0;
	}

	ctx = filp->private_data;
	tfm =  ctx->pka1_rsa_tfm;

	if (rsa_req->op_mode == RSA_SET_PUB) {
		if (!(rsa_req->keylen)) {
			ret = -EINVAL;
			goto out;
		}

		ret = crypto_akcipher_set_pub_key(tfm, rsa_req->key,
						  rsa_req->keylen);
		if (ret) {
			pr_err("alg: pka1:rsa: set_pub_key failed\n");
			goto out;
		}
	} else if (rsa_req->op_mode == RSA_SET_PRIV) {
		if (!(rsa_req->keylen)) {
			ret = -EINVAL;
			goto out;
		}

		ret = crypto_akcipher_set_priv_key(tfm, rsa_req->key,
						   rsa_req->keylen);
		if (ret) {
			pr_err("alg: pka1:rsa: set_priv_key failed\n");
			goto out;
		}
	} else if (rsa_req->op_mode == RSA_ENCRYPT ||
		   rsa_req->op_mode == RSA_DECRYPT ||
		   rsa_req->op_mode == RSA_SIGN ||
		   rsa_req->op_mode == RSA_VERIFY) {
		req = akcipher_request_alloc(tfm, GFP_KERNEL);
		if (!req) {
			pr_err("alg: rsa: Failed to allocate request: %s\n",
				__func__);
			ret = -ENOMEM;
			goto out;
		}

		ret = alloc_bufs(xbuf);
		if (ret < 0) {
			pr_err("alloc_bufs failed");
			goto buf_fail;
		}

		init_completion(&rsa_complete.restart);
		rsa_complete.req_err = 0;

		len = crypto_akcipher_maxsize(tfm);
		if (len < 0)
			return -EINVAL;

		ret = copy_from_user((void *)xbuf[0],
				     (void __user *)rsa_req->message, len);
		if (ret) {
			ret = -EFAULT;
			pr_debug("%s: copy_from_user failed (%d)\n",
				 __func__, ret);
			goto copy_fail;
		}

		sg_init_one(&sg[0], xbuf[0], len);
		sg_init_one(&sg[1], xbuf[1], len);

		akcipher_request_set_crypt(req, &sg[0], &sg[1], len, len);

		if (rsa_req->op_mode == RSA_ENCRYPT) {
			ret = crypto_akcipher_encrypt(req);
			if (ret) {
				pr_err("alg: pka1:rsa: encrypt failed\n");
				goto copy_fail;
			}
		} else if (rsa_req->op_mode == RSA_DECRYPT) {
			ret = crypto_akcipher_decrypt(req);
			if (ret) {
				pr_err("alg: pka1:rsa: decrypt failed\n");
				goto copy_fail;
			}
		} else if (rsa_req->op_mode == RSA_SIGN) {
			ret = crypto_akcipher_sign(req);
			if (ret) {
				pr_err("alg: pka1:rsa: sign failed\n");
				goto copy_fail;
			}
		} else if (rsa_req->op_mode == RSA_VERIFY) {
			ret = crypto_akcipher_verify(req);
			if (ret) {
				pr_err("alg: pka1:rsa: verification failed\n");
				goto copy_fail;
			}
		}

		ret = copy_to_user((void __user *)rsa_req->result,
				   (const void *)xbuf[1], len);
		if (ret) {
			ret = -EFAULT;
			pr_err("alg: pka1:rsa: copy_to_user failed (%d)\n",
				ret);
		}
copy_fail:
		free_bufs(xbuf);
buf_fail:
		akcipher_request_free(req);
	} else if (rsa_req->op_mode == RSA_EXIT) {
		crypto_free_akcipher(tfm);
	} else {
		pr_err("alg: pka1:rsa: invalid rsa operation\n");
	}
out:
	if (ret)
		crypto_free_akcipher(tfm);
	return ret;
}

static int tegra_crypto_sha(struct file *filp, struct tegra_crypto_ctx *ctx,
				struct tegra_sha_req *sha_req)
{

	struct crypto_ahash *tfm;
	struct scatterlist sg[1];
	char result[64];
	char algo[64];
	struct ahash_request *req;
	struct tegra_crypto_completion sha_complete;
	void *hash_buff;
	unsigned long *xbuf[XBUFSIZE];
	int ret = -ENOMEM, i;
	char sha_algo[6][10] = {"sha1", "sha224", "sha256",
				"sha384", "sha512", "cmac(aes)"};

	if (sha_req->plaintext_sz > PAGE_SIZE) {
		pr_err("alg:hash: invalid plaintext_sz for sha_req\n");
		return -EINVAL;
	}

	if (strncpy_from_user(algo, sha_req->algo, sizeof(algo) - 1) < 0) {
		pr_err("alg:hash: invalid algo for sha_req\n");
		return -EFAULT;
	}
	algo[sizeof(algo) - 1] = '\0';

	tfm = crypto_alloc_ahash(algo, 0, 0);
	if (IS_ERR(tfm)) {
		pr_err("alg:hash:Failed to load transform for %s:%ld\n",
			algo, PTR_ERR(tfm));
		goto out_alloc;
	}

	for (i = 0; i < 6; i++) {
		if (!strcmp(sha_req->algo, sha_algo[i])) {
			ctx->sha_tfm[i] = tfm;
			break;
		}
	}
	filp->private_data = ctx;

	req = ahash_request_alloc(tfm, GFP_KERNEL);
	if (!req) {
		pr_err("alg:hash:Failed to allocate request for %s\n", algo);
		goto out_noreq;
	}

	ret = alloc_bufs(xbuf);
	if (ret < 0) {
		pr_err("alloc_bufs failed");
		goto out_buf;
	}

	init_completion(&sha_complete.restart);
	sha_complete.req_err = 0;

	memset(result, 0, 64);

	hash_buff = xbuf[0];

	ret = copy_from_user((void *)hash_buff,
			     (void __user *)sha_req->plaintext,
			     sha_req->plaintext_sz);
	if (ret) {
		ret = -EFAULT;
		pr_err("%s: copy_from_user failed (%d)\n", __func__, ret);
			goto out;
	}

	sg_init_one(&sg[0], hash_buff, sha_req->plaintext_sz);

	if (sha_req->keylen) {
		crypto_ahash_clear_flags(tfm, ~0);
		ret = crypto_ahash_setkey(tfm, sha_req->key,
					  sha_req->keylen);
		if (ret) {
			pr_err("alg:hash:setkey failed on %s:ret=%d\n",
				algo, ret);

			goto out;
		}
	}

	ahash_request_set_crypt(req, sg, result, sha_req->plaintext_sz);

	ret = sha_async_hash_op(req, &sha_complete, crypto_ahash_init(req));
	if (ret) {
		pr_err("alg: hash: init failed for %s: ret=%d\n", algo, ret);
		goto out;
	}

	ret = sha_async_hash_op(req, &sha_complete, crypto_ahash_update(req));
	if (ret) {
		pr_err("alg: hash: update failed for %s: ret=%d\n", algo, ret);
		goto out;
	}

	ret = sha_async_hash_op(req, &sha_complete, crypto_ahash_final(req));
	if (ret) {
		pr_err("alg: hash: final failed for %s: ret=%d\n", algo, ret);
		goto out;
	}

	ret = copy_to_user((void __user *)sha_req->result,
		(const void *)result, crypto_ahash_digestsize(tfm));
	if (ret) {
		ret = -EFAULT;
		pr_err("alg: hash: copy_to_user failed (%d) for %s\n",
			ret, algo);
	}

out:
	free_bufs(xbuf);

out_buf:
	ahash_request_free(req);

out_noreq:
	crypto_free_ahash(tfm);

out_alloc:
	return ret;
}

static long tegra_crypto_dev_ioctl(struct file *filp,
	unsigned int ioctl_num, unsigned long arg)
{
	struct tegra_crypto_ctx *ctx = filp->private_data;
	struct crypto_rng *tfm = NULL;
	struct tegra_pka1_rsa_request pka1_rsa_req;
	struct tegra_se_pka1_ecc_request pka1_ecc_req;
	struct tegra_se_rng1_request rng1_req;
	struct tegra_crypt_req crypt_req;
	struct tegra_rng_req rng_req;
	struct tegra_sha_req sha_req;
	struct tegra_rsa_req rsa_req;
	struct tegra_rsa_req_ahash rsa_req_ah;
#ifdef CONFIG_COMPAT
	struct tegra_crypt_req_32 crypt_req_32;
	struct tegra_rng_req_32 rng_req_32;
	struct tegra_sha_req_32 sha_req_32;
	int i = 0;
#endif
	char *rng;
	int ret = 0;

	switch (ioctl_num) {
	case TEGRA_CRYPTO_IOCTL_NEED_SSK:
		ctx->use_ssk = (int)arg;
		break;

#ifdef CONFIG_COMPAT
	case TEGRA_CRYPTO_IOCTL_PROCESS_REQ_32:
		ret = copy_from_user(&crypt_req_32, (void __user *)arg,
			sizeof(struct tegra_crypt_req_32));
		if (ret) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}
		crypt_req.op = crypt_req_32.op;
		crypt_req.encrypt = crypt_req_32.encrypt;
		crypt_req.skip_key = crypt_req_32.skip_key;
		crypt_req.skip_iv = crypt_req_32.skip_iv;
		for (i = 0; i < crypt_req_32.keylen; i++)
			crypt_req.key[i] = crypt_req_32.key[i];
		crypt_req.keylen = crypt_req_32.keylen;
		for (i = 0; i < TEGRA_CRYPTO_IV_SIZE; i++)
			crypt_req.iv[i] = crypt_req_32.iv[i];
		crypt_req.ivlen = crypt_req_32.ivlen;
		crypt_req.plaintext =
			(u8 __user *)(void *)(__u64)(crypt_req_32.plaintext);
		crypt_req.plaintext_sz = crypt_req_32.plaintext_sz;
		crypt_req.result =
			(u8 __user *)(void *)(__u64)(crypt_req_32.result);

		ret = process_crypt_req(filp, ctx, &crypt_req);
		break;
#endif
	case TEGRA_CRYPTO_IOCTL_PROCESS_REQ:
		ret = copy_from_user(&crypt_req, (void __user *)arg,
			sizeof(crypt_req));
		if (ret) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}

		ret = process_crypt_req(filp, ctx, &crypt_req);
		break;

#ifdef CONFIG_COMPAT
	case TEGRA_CRYPTO_IOCTL_SET_SEED_32:
		if (copy_from_user(&rng_req_32, (void __user *)arg,
			sizeof(rng_req_32))) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}

		for (i = 0; i < TEGRA_CRYPTO_RNG_SEED_SIZE; i++)
			rng_req.seed[i] = rng_req_32.seed[i];
		rng_req.type = rng_req_32.type;
		/* fall through */
#endif

	case TEGRA_CRYPTO_IOCTL_SET_SEED:
		if (ioctl_num == TEGRA_CRYPTO_IOCTL_SET_SEED) {
			if (copy_from_user(&rng_req, (void __user *)arg,
				sizeof(rng_req))) {
				pr_err("%s: copy_from_user fail(%d)\n",
						__func__, ret);
				return -EFAULT;
			}
		}

		memcpy(ctx->seed, rng_req.seed, TEGRA_CRYPTO_RNG_SEED_SIZE);

		if (rng_req.type == RNG_DRBG) {
			if (tegra_get_chip_id() == TEGRA20 ||
				tegra_get_chip_id() == TEGRA30) {
				return -EINVAL;
			}
			ctx->rng_drbg = crypto_alloc_rng("rng_drbg-aes-tegra",
				CRYPTO_ALG_TYPE_RNG, 0);
			if (IS_ERR(ctx->rng_drbg)) {
				pr_err("Failed to load transform for "
				"rng_drbg tegra: %ld\n",
					PTR_ERR(ctx->rng_drbg));
				ret = PTR_ERR(ctx->rng_drbg);
				goto out;
			}
			tfm = ctx->rng_drbg;
			filp->private_data = ctx;
			ret = crypto_rng_reset(ctx->rng_drbg, ctx->seed,
				crypto_rng_seedsize(ctx->rng_drbg));
		} else {
			ctx->rng = crypto_alloc_rng("rng-aes-tegra",
				CRYPTO_ALG_TYPE_RNG, 0);
			if (IS_ERR(ctx->rng)) {
				pr_err("Failed to load transform for "
					"tegra rng: %ld\n", PTR_ERR(ctx->rng));
				ret = PTR_ERR(ctx->rng);
				goto out;
			}
			tfm = ctx->rng;
			filp->private_data = ctx;
			ret = crypto_rng_reset(ctx->rng, ctx->seed,
				crypto_rng_seedsize(ctx->rng));
		}
		if (ret)
			pr_err("crypto_rng_reset failed: %d\n", ret);
		crypto_free_rng(tfm);
		break;

#ifdef CONFIG_COMPAT
	case TEGRA_CRYPTO_IOCTL_GET_RANDOM_32:
		if (copy_from_user(&rng_req_32, (void __user *)arg,
			sizeof(rng_req_32))) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}

		rng_req.nbytes = rng_req_32.nbytes;
		rng_req.type = rng_req_32.type;
		rng_req.rdata = (u8 __user *)(void *)(__u64)rng_req_32.rdata;
		/* fall through */
#endif

	case TEGRA_CRYPTO_IOCTL_GET_RANDOM:
		if (ioctl_num == TEGRA_CRYPTO_IOCTL_GET_RANDOM) {
			if (copy_from_user(&rng_req, (void __user *)arg,
				sizeof(rng_req))) {
				pr_err("%s: copy_from_user fail(%d)\n",
						__func__, ret);
				return -EFAULT;
			}
		}

		rng = kzalloc(rng_req.nbytes, GFP_KERNEL);
		if (!rng) {
			if (rng_req.type == RNG_DRBG)
				pr_err("mem alloc for rng_drbg fail");
			else
				pr_err("mem alloc for rng fail");

			return -ENODATA;
		}

		if (rng_req.type == RNG_DRBG) {
			if (tegra_get_chip_id() == TEGRA20 ||
				tegra_get_chip_id() == TEGRA30) {
				ret = -EINVAL;
				goto rng_out;
			}
			ctx->rng_drbg = crypto_alloc_rng("rng_drbg-aes-tegra",
				CRYPTO_ALG_TYPE_RNG, 0);
			if (IS_ERR(ctx->rng_drbg)) {
				pr_err("Failed to load transform for "
				"rng_drbg tegra: %ld\n",
						PTR_ERR(ctx->rng_drbg));
				ret = PTR_ERR(ctx->rng_drbg);
				goto rng_out;
			}
			tfm = ctx->rng_drbg;
			filp->private_data = ctx;
			ret = crypto_rng_get_bytes(ctx->rng_drbg, rng,
				rng_req.nbytes);
		} else {
			ctx->rng = crypto_alloc_rng("rng-aes-tegra",
				CRYPTO_ALG_TYPE_RNG, 0);
			if (IS_ERR(ctx->rng)) {
				pr_err("Failed to load transform for "
					"tegra rng: %ld\n", PTR_ERR(ctx->rng));
				ret = PTR_ERR(ctx->rng);
				goto rng_out;
			}
			tfm = ctx->rng;
			filp->private_data = ctx;
			ret = crypto_rng_get_bytes(ctx->rng, rng,
				rng_req.nbytes);
		}

		if (ret != rng_req.nbytes) {
			if (rng_req.type == RNG_DRBG)
				pr_err("rng_drbg failed");
			else
				pr_err("rng failed");
			ret = -ENODATA;
			goto free_tfm;
		}

		ret = copy_to_user((void __user *)rng_req.rdata,
			(const void *)rng, rng_req.nbytes);
		if (ret) {
			ret = -EFAULT;
			pr_err("%s: copy_to_user fail(%d)\n", __func__, ret);
			goto free_tfm;
		}
free_tfm:
		crypto_free_rng(tfm);
rng_out:
		kfree(rng);
		break;

#ifdef CONFIG_COMPAT
	case TEGRA_CRYPTO_IOCTL_GET_SHA_32:
		ret = copy_from_user(&sha_req_32, (void __user *)arg,
			sizeof(sha_req_32));
		if (ret) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}
		for (i = 0; i < sha_req_32.keylen; i++)
			sha_req.key[i] = sha_req_32.key[i];
		sha_req.keylen = sha_req_32.keylen;
		sha_req.algo =
		(unsigned char __user *)(void *)(__u64)(sha_req_32.algo);
		sha_req.plaintext =
		(unsigned char __user *)(void *)(__u64)(sha_req_32.plaintext);
		sha_req.plaintext_sz = sha_req_32.plaintext_sz;
		sha_req.result =
		(unsigned char __user *)(void *)(__u64)(sha_req_32.result);

		ret = tegra_crypto_sha(filp, ctx, &sha_req);
		break;
#endif

	case TEGRA_CRYPTO_IOCTL_GET_SHA:
		if (tegra_get_chip_id() == TEGRA20)
			return -EINVAL;
		if (copy_from_user(&sha_req, (void __user *)arg,
				sizeof(sha_req))) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}
		ret = tegra_crypto_sha(filp, ctx, &sha_req);
		break;

	case TEGRA_CRYPTO_IOCTL_RSA_REQ_AHASH:
		if (copy_from_user(&rsa_req_ah, (void __user *)arg,
			sizeof(rsa_req_ah))) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}

		ret = tegra_crypt_rsa_ahash(filp, ctx, &rsa_req_ah);
		break;

	case TEGRA_CRYPTO_IOCTL_RSA_REQ:
		if (copy_from_user(&rsa_req, (void __user *)arg,
			sizeof(rsa_req))) {
			pr_err("%s: copy_from_user fail(%d)\n", __func__, ret);
			return -EFAULT;
		}

		ret = tegra_crypt_rsa(filp, ctx, &rsa_req);
		break;

	case TEGRA_CRYPTO_IOCTL_PKA1_RSA_REQ:
		if (copy_from_user(&pka1_rsa_req, (void __user *)arg,
			sizeof(pka1_rsa_req))) {
			pr_err("%s: copy_from_user fail(%d) for pka1_rsa_req\n",
				__func__, ret);
			return -EFAULT;
		}

		ret = tegra_crypt_pka1_rsa(filp, ctx, &pka1_rsa_req);
		break;

	case TEGRA_CRYPTO_IOCTL_PKA1_ECC_REQ:
		if (copy_from_user(&pka1_ecc_req, (void __user *)arg,
				   sizeof(pka1_ecc_req))) {
			ret = -EFAULT;
			pr_err("%s: copy_from_user fail(%d) for pka1_ecc_req\n",
				__func__, ret);
			return ret;
		}
		ret = tegra_se_pka1_ecc_op(&pka1_ecc_req);
		if (ret) {
			pr_debug("\ntegra_se_pka1_ecc_op failed(%d) for ECC\n",
				 ret);
			return ret;
		}

		ret = copy_to_user((void __user *)arg, &pka1_ecc_req,
				   sizeof(pka1_ecc_req));
		if (ret) {
			ret = -EFAULT;
			pr_debug("%s: copy_to_user failed (%d)\n", __func__,
				 ret);
			return ret;
		}
		break;

	case TEGRA_CRYPTO_IOCTL_RNG1_REQ:
		if (copy_from_user(&rng1_req, (void __user *)arg,
				   sizeof(rng1_req))) {
			ret = -EFAULT;
			pr_err("%s: copy_from_user fail(%d) for rng1_req\n",
				__func__, ret);
			return ret;
		}

		ret = tegra_se_rng1_op(&rng1_req);
		if (ret) {
			pr_debug("\ntegra_se_rng1_op failed(%d) for RNG1\n",
				  ret);
			return ret;
		}

		ret = copy_to_user((void __user *)arg, &rng1_req,
				   sizeof(rng1_req));
		if (ret) {
			ret = -EFAULT;
			pr_debug("%s: copy_to_user failed (%d) for rng1_req\n",
				 __func__, ret);
			return ret;
		}
		break;
	default:
		pr_debug("invalid ioctl code(%d)", ioctl_num);
		return -EINVAL;
	}
out:
	return ret;
}

static const struct file_operations tegra_crypto_fops = {
	.owner = THIS_MODULE,
	.open = tegra_crypto_dev_open,
	.release = tegra_crypto_dev_release,
	.unlocked_ioctl = tegra_crypto_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl =  tegra_crypto_dev_ioctl,
#endif
};

static struct miscdevice tegra_crypto_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tegra-crypto",
	.fops = &tegra_crypto_fops,
};

static int __init tegra_crypto_dev_init(void)
{
	return misc_register(&tegra_crypto_device);
}

late_initcall(tegra_crypto_dev_init);

MODULE_DESCRIPTION("Tegra AES hw device node.");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPLv2");
