/*
 * drivers/misc/tegra-profiler/mmap.c
 *
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/sched.h>

#include <linux/tegra_profiler.h>

#include "quadd.h"
#include "mmap.h"
#include "comm.h"
#include "hrt.h"

#define TMP_BUFFER_SIZE			(PATH_MAX + sizeof(u64))
#define QUADD_MMAP_TREE_MAX_LEVEL	32

static void
put_mmap_sample(struct quadd_mmap_data *s, char *filename,
		size_t length, unsigned long pgoff,
		int is_file_exists, pid_t __tgid)
{
	u64 mmap_ed = 0;
	struct quadd_record_data r;
	struct quadd_iovec vec[4];
	u64 pgoff_val = (u64)pgoff << PAGE_SHIFT;
	u32 tgid = (u32)__tgid;

	r.record_type = QUADD_RECORD_TYPE_MMAP;

	memcpy(&r.mmap, s, sizeof(*s));

	r.mmap.time = quadd_get_time();
	r.mmap.filename_length = length;

	if (is_file_exists)
		mmap_ed |= QUADD_MMAP_ED_IS_FILE_EXISTS;

	vec[0].base = &pgoff_val;
	vec[0].len = sizeof(pgoff_val);

	vec[1].base = &mmap_ed;
	vec[1].len = sizeof(mmap_ed);

	vec[2].base = filename;
	vec[2].len = length;

	vec[3].base = &tgid;
	vec[3].len = sizeof(tgid);

	pr_debug("[%d] MMAP: tid: %u,pid: %u,'%s',%#llx-%#llx(%llx,%#llx)\n",
		 smp_processor_id(), s->pid, tgid, filename,
		 s->addr, s->addr + s->len, s->len, pgoff_val);

	quadd_put_sample_this_cpu(&r, vec, ARRAY_SIZE(vec));
}

static void
process_mmap(struct vm_area_struct *vma, struct task_struct *task,
	     char *buf, size_t buf_size)
{
	pid_t tgid;
	int is_file_exists;
	struct file *vm_file;
	char *file_name;
	struct quadd_mmap_data sample;
	size_t length, length_aligned;

	if (!(vma->vm_flags & VM_EXEC))
		return;

	vm_file = vma->vm_file;
	if (vm_file) {
		file_name = file_path(vm_file, buf, PATH_MAX);
		if (IS_ERR(file_name))
			return;

		length = strlen(file_name) + 1;
		is_file_exists = 1;
	} else {
		const char *name = NULL;

		name = arch_vma_name(vma);
		if (!name) {
			struct mm_struct *mm = vma->vm_mm;

			if (!mm) {
				name = "[vdso]";
			} else if (vma->vm_start <= mm->start_brk &&
				   vma->vm_end >= mm->brk) {
				name = "[heap]";
			} else if (vma->vm_start <= mm->start_stack &&
				   vma->vm_end >= mm->start_stack) {
				name = "[stack]";
			}
		}

		if (name)
			strlcpy(buf, name, buf_size);
		else
			snprintf(buf, buf_size, "[vma:%08lx-%08lx]",
				 vma->vm_start, vma->vm_end);

		file_name = buf;
		length = strlen(file_name) + 1;

		is_file_exists = 0;
	}

	length_aligned = ALIGN(length, sizeof(u64));

	sample.pid = task_pid_nr(task);
	tgid = task_tgid_nr(task);
	sample.user_mode = 1;

	sample.addr = vma->vm_start;
	sample.len = vma->vm_end - vma->vm_start;

	put_mmap_sample(&sample, file_name, length_aligned,
			vma->vm_pgoff, is_file_exists, tgid);
}

void quadd_process_mmap(struct vm_area_struct *vma, struct task_struct *task)
{
	char *buf;

	buf = kzalloc(TMP_BUFFER_SIZE, GFP_ATOMIC);
	if (!buf)
		return;

	preempt_disable();
	process_mmap(vma, task, buf, TMP_BUFFER_SIZE);
	preempt_enable();

	kfree(buf);
}

static void get_process_vmas(struct task_struct *task)
{
	char *buf;
	struct mm_struct *mm;
	struct vm_area_struct *vma;

	if (!task)
		return;

	mm = get_task_mm(task);
	if (!mm)
		return;

	down_read(&mm->mmap_sem);

	buf = kzalloc(TMP_BUFFER_SIZE, GFP_ATOMIC);
	if (!buf)
		goto out_put_mm;

	preempt_disable();
	for (vma = mm->mmap; vma; vma = vma->vm_next)
		process_mmap(vma, task, buf, TMP_BUFFER_SIZE);
	preempt_enable();

	kfree(buf);

out_put_mm:
	up_read(&mm->mmap_sem);
	mmput(mm);
}

static void
__get_process_vmas(struct task_struct *task,
		   struct mm_struct *mm, char *buf, size_t buf_size)
{
	struct vm_area_struct *vma;

	for (vma = mm->mmap; vma; vma = vma->vm_next)
		process_mmap(vma, task, buf, buf_size);
}

static int
is_sample_process(struct task_struct *from, struct pid *root_pid)
{
	struct task_struct *p;

	for (p = from; p != &init_task;) {
		if (task_pid_nr(p) == pid_nr(root_pid))
			return 1;

		rcu_read_lock();
		p = rcu_dereference(p->real_parent);
		rcu_read_unlock();
	}

	return 0;
}

static void get_all_processes(struct pid *root_pid)
{
	char *buf;
	struct task_struct *p;

	buf = kzalloc(TMP_BUFFER_SIZE, GFP_ATOMIC);
	if (!buf)
		return;

	read_lock(&tasklist_lock);
	for_each_process(p) {
		struct mm_struct *mm;

		if (p->flags & PF_KTHREAD)
			continue;

		if (root_pid && !is_sample_process(p, root_pid))
			continue;

		task_lock(p);

		if (unlikely(!p->mm))
			goto __continue;
		mm = p->mm;

		if (!down_read_trylock(&mm->mmap_sem))
			goto __continue;

		__get_process_vmas(p, mm, buf, TMP_BUFFER_SIZE);

		up_read(&mm->mmap_sem);
__continue:
		task_unlock(p);
	}
	read_unlock(&tasklist_lock);

	kfree(buf);
}

void quadd_get_mmaps(struct quadd_ctx *ctx)
{
	struct pid *pid;
	struct task_struct *task;
	struct quadd_parameters *param = &ctx->param;

	if (!quadd_mode_is_sampling(ctx))
		return;

	if (quadd_mode_is_sample_all(ctx)) {
		get_all_processes(NULL);
		return;
	}

	pid = find_vpid(param->pids[0]);

	task = get_pid_task(pid, PIDTYPE_PID);
	if (!task)
		return;

	if (quadd_mode_is_sample_tree(ctx))
		get_all_processes(pid);
	else
		get_process_vmas(task);

	put_task_struct(task);
}
