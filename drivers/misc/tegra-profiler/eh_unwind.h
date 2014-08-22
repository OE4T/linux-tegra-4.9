/*
 * drivers/misc/tegra-profiler/eh_unwind.h
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __QUADD_EH_UNWIND_H__
#define __QUADD_EH_UNWIND_H__

struct pt_regs;
struct quadd_callchain;
struct quadd_ctx;
struct quadd_extables;
struct task_struct;
struct quadd_extabs_mmap;

unsigned int
quadd_aarch32_get_user_callchain_ut(struct pt_regs *regs,
				    struct quadd_callchain *cc,
				    struct task_struct *task);

int quadd_unwind_init(void);
void quadd_unwind_deinit(void);

int quadd_unwind_start(struct task_struct *task);
void quadd_unwind_stop(void);

int quadd_unwind_set_extab(struct quadd_extables *extabs,
			   struct quadd_extabs_mmap *mmap);
void quadd_unwind_delete_mmap(struct quadd_extabs_mmap *mmap);

int
quadd_aarch32_is_ex_entry_exist(struct pt_regs *regs,
				unsigned long addr,
				struct task_struct *task);

void
quadd_unwind_set_tail_info(unsigned long vm_start,
			   unsigned long tf_start,
			   unsigned long tf_end);

struct quadd_extabs_mmap;

struct extab_info {
	unsigned long addr;
	unsigned long length;

	unsigned long mmap_offset;
};

struct extables {
	struct extab_info extab;
	struct extab_info exidx;
};

struct ex_region_info {
	unsigned long vm_start;
	unsigned long vm_end;

	struct extables tabs;
	struct quadd_extabs_mmap *mmap;

	struct list_head list;

	unsigned long tf_start;
	unsigned long tf_end;
};

long quadd_search_ex_region(unsigned long key, struct ex_region_info *ri);

#endif	/* __QUADD_EH_UNWIND_H__ */
