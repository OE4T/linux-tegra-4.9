/*
 * drivers/misc/tegra-profiler/dwarf_unwind.h
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

#ifndef __QUADD_DWARF_UNWIND_H
#define __QUADD_DWARF_UNWIND_H

struct pt_regs;
struct quadd_callchain;
struct task_struct;

unsigned int
quadd_aarch64_get_user_callchain_ut(struct pt_regs *regs,
				    struct quadd_callchain *cc,
				    struct task_struct *task);

int
quadd_aarch64_is_ex_entry_exist(struct pt_regs *regs,
				unsigned long addr,
				struct task_struct *task);

int quadd_dwarf_unwind_start(void);
void quadd_dwarf_unwind_stop(void);
int quadd_dwarf_unwind_init(void);

#endif  /* __QUADD_DWARF_UNWIND_H */
