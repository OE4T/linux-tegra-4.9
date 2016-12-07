/*
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/tegra_bpmp.h>
#include "bpmp.h"

struct seqbuf {
	char *buf;
	size_t pos;
	size_t size;
};

static void seqbuf_init(struct seqbuf *seqbuf, void *buf, size_t size)
{
	seqbuf->buf = buf;
	seqbuf->size = size;
	seqbuf->pos = 0;
}

static size_t seqbuf_avail(struct seqbuf *seqbuf)
{
	return seqbuf->pos < seqbuf->size ? seqbuf->size - seqbuf->pos : 0;
}

static size_t seqbuf_status(struct seqbuf *seqbuf)
{
	return seqbuf->pos <= seqbuf->size ? 0 : -EOVERFLOW;
}

static int seqbuf_eof(struct seqbuf *seqbuf)
{
	return seqbuf->pos >= seqbuf->size;
}

static int seqbuf_read(struct seqbuf *seqbuf, void *buf, size_t nbyte)
{
	nbyte = min(nbyte, seqbuf_avail(seqbuf));
	memcpy(buf, seqbuf->buf + seqbuf->pos,
			min(nbyte, seqbuf_avail(seqbuf)));
	seqbuf->pos += nbyte;
	return seqbuf_status(seqbuf);
}

static int seqbuf_read_u32(struct seqbuf *seqbuf, u32 *v)
{
	int err;
	err = seqbuf_read(seqbuf, v, 4);
	*v = le32_to_cpu(*v);
	return err;
}

static const char *seqbuf_strget(struct seqbuf *seqbuf)
{
	const char *ptr = seqbuf->buf + seqbuf->pos;
	seqbuf->pos += strnlen(seqbuf->buf + seqbuf->pos, seqbuf_avail(seqbuf));
	seqbuf->pos++;

	if (seqbuf_status(seqbuf))
		return NULL;
	else
		return ptr;
}

static int seqbuf_seek(struct seqbuf *seqbuf, ssize_t offset)
{
	seqbuf->pos += offset;
	return seqbuf_status(seqbuf);
}

static const char *root_path;

static const char *get_filename(const struct file *file, char *buf, int size)
{
	const char *filename;
	size_t root_len = strlen(root_path);

	filename = dentry_path(file->f_path.dentry, buf, size);
	if (IS_ERR_OR_NULL(filename))
		return NULL;

	if (strlen(filename) < root_len ||
			strncmp(filename, root_path, root_len))
		return NULL;

	filename += root_len;

	return filename;
}

static int bpmp_debugfs_read(uint32_t name, uint32_t sz_name,
		dma_addr_t data, size_t sz_data, uint32_t *nbytes)
{
	struct mrq_debugfs_request rq;
	struct mrq_debugfs_response re;
	int r;

	if (sz_name < 0 || sz_data < 0)
		return -EINVAL;

	rq.cmd = cpu_to_le32(CMD_DEBUGFS_READ);
	rq.fop.fnameaddr = cpu_to_le32(name);
	rq.fop.fnamelen = cpu_to_le32(sz_name);
	rq.fop.dataaddr = cpu_to_le32(data);
	rq.fop.datalen = cpu_to_le32(sz_data);

	r = tegra_bpmp_send_receive(MRQ_DEBUGFS, &rq, sizeof(rq),
			&re, sizeof(re));
	if (r)
		return r;

	*nbytes = re.fop.nbytes;

	return 0;
}

static int debugfs_show(struct seq_file *m, void *p)
{
	struct file *file = m->private;
	const size_t namesize = SZ_256;
	char *databuf = NULL;
	char *namebuf = NULL;
	dma_addr_t dataphys;
	dma_addr_t namephys;
	const char *filename;
	size_t off;
	uint32_t nbytes;
	int len;
	int ret;

	namebuf = tegra_bpmp_alloc_coherent(namesize, &namephys, GFP_KERNEL);
	if (!namebuf)
		return -ENOMEM;

	filename = get_filename(file, namebuf, namesize);
	if (!filename) {
		ret = -ENOENT;
		goto out;
	}

	databuf = tegra_bpmp_alloc_coherent(m->size, &dataphys, GFP_KERNEL);
	if (!databuf) {
		ret = -ENOMEM;
		goto out;
	}

	off = filename - namebuf;
	len = strlen(filename);

	ret = bpmp_debugfs_read(namephys + off, len, dataphys,
			m->size, &nbytes);

	if (!ret)
		seq_write(m, databuf, nbytes);

out:
	tegra_bpmp_free_coherent(namesize, namebuf, namephys);

	if (databuf)
		tegra_bpmp_free_coherent(m->size, databuf, dataphys);

	return ret;
}

static int debugfs_open(struct inode *inode, struct file *file)
{
	return single_open_size(file, debugfs_show, file, SZ_128K);
}

static int bpmp_debugfs_write(uint32_t name, size_t sz_name,
		uint32_t data, size_t sz_data)
{
	struct mrq_debugfs_request rq;

	if (sz_name < 0 || sz_data < 0)
		return -EINVAL;

	rq.cmd = cpu_to_le32(CMD_DEBUGFS_WRITE);
	rq.fop.fnameaddr = cpu_to_le32(name);
	rq.fop.fnamelen = cpu_to_le32(sz_name);
	rq.fop.dataaddr = cpu_to_le32(data);
	rq.fop.datalen = cpu_to_le32(sz_data);

	return tegra_bpmp_send_receive(MRQ_DEBUGFS, &rq, sizeof(rq),
			NULL, 0);
}

static ssize_t debugfs_store(struct file *file, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	const size_t namesize = SZ_256;
	char *databuf = NULL;
	char *namebuf = NULL;
	const char *filename;
	ssize_t ret;
	dma_addr_t phys_data;
	dma_addr_t phys_name;
	size_t off;
	int len;

	databuf = tegra_bpmp_alloc_coherent(count, &phys_data, GFP_KERNEL);
	if (!databuf)
		return -ENOMEM;

	namebuf = tegra_bpmp_alloc_coherent(namesize, &phys_name, GFP_KERNEL);
	if (!namebuf) {
		ret = -ENOMEM;
		goto out;
	}

	if (copy_from_user(databuf, buf, count)) {
		ret = -EFAULT;
		goto out;
	}

	filename = get_filename(file, namebuf, namesize);
	if (!filename) {
		ret = -EFAULT;
		goto out;
	}

	off = filename - namebuf;
	len = strlen(filename);

	ret = bpmp_debugfs_write(phys_name + off, len, phys_data, count);

out:
	tegra_bpmp_free_coherent(namesize, namebuf, phys_name);

	if (databuf)
		tegra_bpmp_free_coherent(count, databuf, phys_data);

	return ret ?: count;
}

static const struct file_operations debugfs_fops = {
	.open		= debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write		= debugfs_store,
	.release	= single_release,
};

static int bpmp_populate_dir(struct seqbuf *seqbuf, struct dentry *parent,
		u32 depth)
{
	int err;

	while (!seqbuf_eof(seqbuf)) {
		u32 d, t;
		const char *name;
		struct dentry *dentry;

		seqbuf_read_u32(seqbuf, &d);
		if (d < depth) {
			seqbuf_seek(seqbuf, -4);
			/* go up a level */
			return 0;
		}
		seqbuf_read_u32(seqbuf, &t);
		name = seqbuf_strget(seqbuf);

		if (seqbuf_status(seqbuf))
			return seqbuf_status(seqbuf);

		if (d != depth) {
			/* malformed data received from BPMP */
			return -EIO;
		}

		if (t & DEBUGFS_S_ISDIR) {
			dentry = debugfs_create_dir(name, parent);
			if (IS_ERR_OR_NULL(dentry))
				return dentry ? PTR_ERR(dentry) : -ENOMEM;
			err = bpmp_populate_dir(seqbuf, dentry, depth+1);
			if (err)
				return err;
		} else {
			umode_t mode;
			mode = t & DEBUGFS_S_IRUSR ? S_IRUSR : 0;
			mode |= t & DEBUGFS_S_IWUSR ? S_IWUSR : 0;
			dentry = debugfs_create_file(name, mode,
					parent, NULL,
					&debugfs_fops);
			if (IS_ERR_OR_NULL(dentry))
				return -ENOMEM;
		}
	}

	return 0;
}

static DEFINE_MUTEX(lock);
static struct dentry *bpmp_debugfs_root;
static char root_path_buf[256];

static int bpmp_fwdebug_recreate(void *buf, size_t bufsize, struct dentry *root)
{
	struct seqbuf seqbuf;
	int err;

	mutex_lock(&lock);

	debugfs_remove_recursive(bpmp_debugfs_root);

	bpmp_debugfs_root = debugfs_create_dir("debug", root);
	if (IS_ERR_OR_NULL(bpmp_debugfs_root)) {
		pr_err("failed to create bpmp debugfs directory\n");
		bpmp_debugfs_root = NULL;
		mutex_unlock(&lock);
		return -ENOMEM;
	}

	root_path = dentry_path_raw(bpmp_debugfs_root, root_path_buf,
			sizeof(root_path_buf));
	if (IS_ERR_OR_NULL(root_path)) {
		pr_err("failed to figure out bpmp root path\n");
		err = root_path ? PTR_ERR(root_path) : -ENOENT;
		goto clean;
	}

	seqbuf_init(&seqbuf, buf, bufsize);
	err = bpmp_populate_dir(&seqbuf, bpmp_debugfs_root, 0);
	if (err)
		goto clean;

	mutex_unlock(&lock);
	return 0;

clean:
	debugfs_remove_recursive(bpmp_debugfs_root);
	bpmp_debugfs_root = NULL;
	mutex_unlock(&lock);
	return err;
}

static int bpmp_debugfs_dumpdir(uint32_t addr, size_t size, uint32_t *nbytes)
{
	struct mrq_debugfs_request rq;
	struct mrq_debugfs_response re;
	int r;

	rq.cmd = cpu_to_le32(CMD_DEBUGFS_DUMPDIR);
	rq.dumpdir.dataaddr = cpu_to_le32(addr);
	rq.dumpdir.datalen = cpu_to_le32(size);

	r = tegra_bpmp_send_receive(MRQ_DEBUGFS, &rq, sizeof(rq),
			&re, sizeof(re));
	if (r)
		return r;

	*nbytes = re.dumpdir.nbytes;

	return 0;
}

int bpmp_fwdebug_init(struct dentry *root)
{
	dma_addr_t phys;
	void *virt;
	const int sz = SZ_256K;
	uint32_t nbytes;
	int ret;

	if (WARN_ON(!root))
		return -EINVAL;

	virt = tegra_bpmp_alloc_coherent(sz, &phys, GFP_KERNEL);
	if (!virt) {
		pr_err("%s: dma_alloc_coherent() failed\n", __func__);
		return -ENOMEM;
	}

	ret = bpmp_debugfs_dumpdir(phys, sz, &nbytes);
	if (ret) {
		pr_err("bpmp_debugfs_dumpdir() failed (%d)\n", ret);
		goto out;
	}

	ret = bpmp_fwdebug_recreate(virt, nbytes, root);
	if (ret) {
		pr_err("create_bpmp_debugfs() failed (%d)\n", ret);
		goto out;
	}

	pr_info("bpmp: mounted debugfs mirror\n");

out:
	tegra_bpmp_free_coherent(sz, virt, phys);

	return ret;
}
