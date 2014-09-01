/*
 * drivers/misc/tegra-profiler/dwarf_unwind.c
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/err.h>

#include <asm/unaligned.h>

#include <linux/tegra_profiler.h>

#include "comm.h"
#include "backtrace.h"
#include "eh_unwind.h"
#include "dwarf.h"
#include "dwarf_unwind.h"

enum {
	DW_WHERE_UNDEF,		/* register isn't saved at all */
	DW_WHERE_SAME,		/* register has same value as in prev. frame */
	DW_WHERE_CFAREL,	/* register saved at CFA-relative address */
	DW_WHERE_REG,		/* register saved in another register */
	DW_WHERE_EXPR,		/* register saved */
	DW_WHERE_VAL_OFFSET,	/* value offset */
	DW_WHERE_VAL_EXPR,	/* register has computed value */
};

#define QUADD_AARCH64_REGISTERS	32

enum regs {
	FP = 29,
	LR = 30,
	SP = 31,
};

enum {
	DW_SEC_TYPE_IDX,
	DW_SEC_TYPE_TAB,
};

union dw_loc {
	unsigned long reg;
	long offset;
	const unsigned char *exp;
};

struct reg_info {
	int where;
	union dw_loc loc;
};

enum {
	DW_CFA_UNSET,
	DW_CFA_REG_OFFSET,
	DW_CFA_EXP,
} cfa_how;

struct dw_eh_frame_hdr {
	unsigned char version;
	unsigned char eh_frame_ptr_enc;
	unsigned char fde_count_enc;
	unsigned char table_enc;
};

struct dw_fde_table {
	s32 initial_loc;
	s32 fde;
};

struct regs_state {
	struct reg_info reg[QUADD_AARCH64_REGISTERS];

	long cfa_offset;
	unsigned int cfa_register;

	unsigned char *cfa_expr;
	unsigned int cfa_expr_len;

	int cfa_how;
};

#define DW_MAX_RS_STACK_DEPTH	8

struct dwarf_cpu_context {
	struct regs_state rs_stack[DW_MAX_RS_STACK_DEPTH];
	int depth;
};

struct quadd_dwarf_context {
	struct dwarf_cpu_context * __percpu cpu_ctx;
	atomic_t started;
};

struct stackframe {
	unsigned long pc;
	unsigned long lr;
	unsigned long sp;
	unsigned long fp;

	struct regs_state rs;
	struct regs_state rs_initial;

	unsigned long cfa;
};

struct dw_cie {
	unsigned long offset;
	unsigned long length;

	unsigned char *aug_string;
	unsigned long aug_size;

	unsigned char fde_encoding;
	unsigned char lsda_encoding;

	unsigned long code_align_factor;
	long data_align_factor;

	unsigned int initial_insn_len;
	unsigned char *initial_insn;

	int z_aug;
	unsigned int retaddr_reg;
	void *personality;

	unsigned char *data;
};

struct dw_fde {
	unsigned long offset;
	unsigned long length;

	unsigned long cie_pointer;
	struct dw_cie *cie;

	unsigned long initial_location;
	unsigned long address_range;

	unsigned int insn_length;
	unsigned char *instructions;

	unsigned char *data;
};


struct eh_sec_data {
	size_t length;
	unsigned char *data;
};

typedef	u64 dw_word_t;

#define read_user_data(addr, retval)				\
({								\
	long ret;						\
								\
	pagefault_disable();					\
	ret = __get_user(retval, addr);				\
	pagefault_enable();					\
								\
	if (ret) {						\
		pr_debug("%s: failed for address: %p\n",	\
			 __func__, addr);			\
		ret = -QUADD_URC_EACCESS;			\
	}							\
								\
	ret;							\
})

static struct quadd_dwarf_context ctx;

static inline int
validate_addr(struct ex_region_info *ri,
	      unsigned long addr,
	      unsigned long nbytes,
	      int st)
{
	struct extab_info *ei;
	struct quadd_extabs_mmap *mmap;
	unsigned long start, end;

	mmap = ri->mmap;
	ei = (st == DW_SEC_TYPE_IDX) ? &ri->tabs.exidx : &ri->tabs.extab;

	start = (unsigned long)mmap->data + ei->mmap_offset;
	end = start + ei->length;

	if (unlikely(addr < start || addr > end - nbytes)) {
		pr_err_once("%s: error: addr: %#lx, len: %ld, data: %#lx-%#lx\n",
			    __func__, addr, nbytes, start, end);
		return 0;
	}

	return 1;
}

static inline u8
read_mmap_data_u8(struct ex_region_info *ri,
		  const u8 *addr, int st, long *err)
{
	unsigned long a = (unsigned long)addr;

	if (unlikely(!validate_addr(ri, a, sizeof(*addr), st))) {
		*err = -QUADD_URC_EACCESS;
		return 0;
	}

	*err = 0;
	return *addr;
}

static inline u16
read_mmap_data_u16(struct ex_region_info *ri,
		   const u16 *addr, int st, long *err)
{
	unsigned long a = (unsigned long)addr;

	if (unlikely(!validate_addr(ri, a, sizeof(*addr), st))) {
		*err = -QUADD_URC_EACCESS;
		return 0;
	}

	*err = 0;

	return get_unaligned(addr);
}

static inline s16
read_mmap_data_s16(struct ex_region_info *ri,
		   const s16 *addr, int st, long *err)
{
	unsigned long a = (unsigned long)addr;

	if (unlikely(!validate_addr(ri, a, sizeof(*addr), st))) {
		*err = -QUADD_URC_EACCESS;
		return 0;
	}

	*err = 0;

	return get_unaligned(addr);
}

static inline u32
read_mmap_data_u32(struct ex_region_info *ri,
		   const u32 *addr, int st, long *err)
{
	unsigned long a = (unsigned long)addr;

	if (unlikely(!validate_addr(ri, a, sizeof(*addr), st))) {
		*err = -QUADD_URC_EACCESS;
		return 0;
	}

	*err = 0;

	return get_unaligned(addr);
}

static inline s32
read_mmap_data_s32(struct ex_region_info *ri,
		   const s32 *addr, int st, long *err)
{
	unsigned long a = (unsigned long)addr;

	if (unlikely(!validate_addr(ri, a, sizeof(*addr), st))) {
		*err = -QUADD_URC_EACCESS;
		return 0;
	}

	*err = 0;

	return get_unaligned(addr);
}

static inline s64
read_mmap_data_s64(struct ex_region_info *ri,
		   const s64 *addr, int st, long *err)
{
	unsigned long a = (unsigned long)addr;

	if (unlikely(!validate_addr(ri, a, sizeof(*addr), st))) {
		*err = -QUADD_URC_EACCESS;
		return 0;
	}

	*err = 0;

	return get_unaligned(addr);
}

static inline u64
read_mmap_data_u64(struct ex_region_info *ri,
		   const u64 *addr, int st, long *err)
{
	unsigned long a = (unsigned long)addr;

	if (unlikely(!validate_addr(ri, a, sizeof(*addr), st))) {
		*err = -QUADD_URC_EACCESS;
		return 0;
	}

	*err = 0;

	return get_unaligned(addr);
}

static inline unsigned long
ex_addr_to_mmap_addr(unsigned long addr,
		     struct ex_region_info *ri, int st)
{
	unsigned long offset;
	struct extab_info *ei;

	ei = (st == DW_SEC_TYPE_IDX) ? &ri->tabs.exidx : &ri->tabs.extab;
	offset = addr - ei->addr;

	return ei->mmap_offset + offset + (unsigned long)ri->mmap->data;
}

static inline unsigned long
mmap_addr_to_ex_addr(unsigned long addr,
		     struct ex_region_info *ri, int st)
{
	unsigned long offset;
	struct extab_info *ei;

	ei = (st == DW_SEC_TYPE_IDX) ? &ri->tabs.exidx : &ri->tabs.extab;
	offset = addr - ei->mmap_offset - (unsigned long)ri->mmap->data;

	return ei->addr + offset;
}

static inline int validate_regnum(struct regs_state *rs, int regnum)
{
	if (unlikely(regnum >= ARRAY_SIZE(rs->reg))) {
		pr_err_once("error: invalid reg: %d\n", regnum);
		return 0;
	}

	return 1;
}

static inline void
set_rule_offset(struct regs_state *rs, int regnum, int where, long offset)
{
	struct reg_info *r;

	if (!validate_regnum(rs, regnum))
		return;

	r = &rs->reg[regnum];

	r->where = where;
	r->loc.offset = offset;
}

static inline void
set_rule_reg(struct regs_state *rs, int regnum, int where, unsigned long reg)
{
	struct reg_info *r;

	if (!validate_regnum(rs, regnum))
		return;

	r = &rs->reg[regnum];

	r->where = where;
	r->loc.reg = reg;
}

static inline void
set_rule_exp(struct regs_state *rs, int regnum,
	     int where, const unsigned char *exp)
{
	struct reg_info *r;

	if (!validate_regnum(rs, regnum))
		return;

	r = &rs->reg[regnum];

	r->where = where;
	r->loc.exp = exp;
}

static inline void
set_rule(struct regs_state *rs, int regnum, int where, long value)
{
	set_rule_offset(rs, regnum, where, value);
}

static inline unsigned long
dw_bst_get_initial_loc(const struct dw_fde_table *fi,
		       unsigned long data_base)
{
	return data_base + fi->initial_loc;
}

static inline unsigned long
dw_bst_get_fde_addr(const struct dw_fde_table *fi,
		    unsigned long data_base)
{
	return data_base + fi->fde;
}

static inline unsigned long
dwarf_read_uleb128(struct ex_region_info *ri,
		   unsigned char *addr,
		   unsigned long *ret,
		   int st,
		   long *err)
{
	unsigned long result;
	unsigned char byte;
	int shift, count;

	result = 0;
	shift = 0;
	count = 0;

	while (1) {
		byte = read_mmap_data_u8(ri, addr, st, err);
		if (*err)
			return 0;

		addr++;
		count++;

		result |= (byte & 0x7f) << shift;
		shift += 7;

		if (!(byte & 0x80))
			break;
	}

	*ret = result;

	return count;
}

static inline unsigned long
dwarf_read_sleb128(struct ex_region_info *ri,
		   unsigned char *addr,
		   long *ret,
		   int st,
		   long *err)
{
	unsigned char byte;
	long result, shift;
	int num_bits;
	int count;

	result = 0;
	shift = 0;
	count = 0;

	while (1) {
		byte = read_mmap_data_u8(ri, addr, st, err);
		if (*err)
			return 0;

		addr++;
		result |= (byte & 0x7f) << shift;
		shift += 7;
		count++;

		if (!(byte & 0x80))
			break;
	}

	num_bits = 8 * sizeof(result);

	if ((shift < num_bits) && (byte & 0x40))
		result |= (-1 << shift);

	*ret = result;

	return count;
}

static inline unsigned int
dw_cfa_opcode(unsigned int insn)
{
	return insn & 0xc0;
}

static inline unsigned int
dw_cfa_operand(unsigned int insn)
{
	return insn & 0x3f;
}

static int
dwarf_read_encoded_value(struct ex_region_info *ri,
			 void *addr,
			 void *pcrel_base,
			 unsigned long *val,
			 char encoding,
			 int st)
{
	int count = 0;
	long stmp = 0, err = 0;
	unsigned long utmp, res = 0;

	pr_debug("encoding: %#x\n", encoding);

	if (encoding == DW_EH_PE_omit) {
		pr_debug("DW_EH_PE_omit\n");

		*val = 0;
		return 0;
	} else if (encoding == DW_EH_PE_aligned) {
		unsigned long aligned = ALIGN((unsigned long)addr,
					      sizeof(dw_word_t));

		pr_debug("DW_EH_PE_aligned\n");

		if (sizeof(dw_word_t) == 4) {
			*val = read_mmap_data_u32(ri, (u32 *)aligned, st, &err);
		} else if (sizeof(dw_word_t) == 8) {
			*val = read_mmap_data_u64(ri, (u64 *)aligned, st, &err);
		} else {
			pr_err_once("%s: error: encoding\n", __func__);
			return -QUADD_URC_TBL_IS_CORRUPT;
		}

		if (err)
			return err;

		return sizeof(dw_word_t);
	}

	switch (encoding & 0x0f) {
	case DW_EH_PE_absptr:
		pr_debug("%s: absptr encoding\n", __func__);

		if (sizeof(dw_word_t) == 4) {
			*val = read_mmap_data_u32(ri, (u32 *)addr, st, &err);
		} else if (sizeof(dw_word_t) == 8) {
			*val = read_mmap_data_u64(ri, (u64 *)addr, st, &err);
		} else {
			pr_err_once("error: wrong dwarf size\n");
			return -QUADD_URC_UNHANDLED_INSTRUCTION;
		}

		if (err)
			return err;

		return sizeof(dw_word_t);

	case DW_EH_PE_sdata2:
	case DW_EH_PE_udata2:
		pr_debug("encoding: DW_EH_PE_sdata2\n");
		stmp = read_mmap_data_s16(ri, (s16 *)addr, st, &err);
		if (err)
			return err;

		count += sizeof(s16);
		break;

	case DW_EH_PE_sdata4:
	case DW_EH_PE_udata4:
		pr_debug("encoding: DW_EH_PE_udata4/sdata4\n");
		stmp = read_mmap_data_s32(ri, (s32 *)addr, st, &err);
		if (err)
			return err;

		count += sizeof(s32);
		break;

	case DW_EH_PE_sdata8:
	case DW_EH_PE_udata8:
		pr_debug("encoding: DW_EH_PE_udata8\n");
		stmp = read_mmap_data_s64(ri, (s64 *)addr, st, &err);
		if (err)
			return err;

		count += sizeof(s64);
		break;

	case DW_EH_PE_uleb128:
		pr_debug("encoding: DW_EH_PE_uleb128\n");
		count += dwarf_read_uleb128(ri, addr, &utmp, st, &err);
		if (err)
			return err;

		stmp = utmp;
		break;

	case DW_EH_PE_sleb128:
		pr_debug("encoding: DW_EH_PE_sleb128\n");
		count += dwarf_read_sleb128(ri, addr, &stmp, st, &err);
		if (err)
			return err;

		break;

	default:
		pr_warn_once("%s: warning: encoding: %#x\n",
			     __func__, encoding & 0x0f);
		return -QUADD_URC_UNHANDLED_INSTRUCTION;
	}

	switch (encoding & 0x70) {
	case DW_EH_PE_absptr:
		pr_debug("DW_EH_PE_absptr\n");
		res = stmp;
		break;

	case DW_EH_PE_pcrel:
		pr_debug("DW_EH_PE_pcrel, pcrel_base: %p, stmp: %ld\n",
			 pcrel_base, stmp);
		res = (unsigned long)pcrel_base + stmp;
		break;

	case DW_EH_PE_textrel:
		pr_warn_once("warning: DW_EH_PE_textrel\n");
		return -QUADD_URC_UNHANDLED_INSTRUCTION;

	case DW_EH_PE_datarel:
		pr_warn_once("warning: DW_EH_PE_datarel\n");
		return -QUADD_URC_UNHANDLED_INSTRUCTION;

	case DW_EH_PE_funcrel:
		pr_warn_once("warning: DW_EH_PE_funcrel\n");
		return -QUADD_URC_UNHANDLED_INSTRUCTION;

	default:
		pr_warn_once("%s: warning: encoding: %#x\n",
			     __func__, encoding & 0x70);
		return -QUADD_URC_UNHANDLED_INSTRUCTION;
	}

	if (res != 0) {
		if (encoding & DW_EH_PE_indirect) {
			pr_debug("DW_EH_PE_indirect\n");

			if (sizeof(dw_word_t) == 4) {
				res = read_mmap_data_u32(ri, (u32 *)res,
							 st, &err);
			} else if (sizeof(dw_word_t) == 8) {
				res = read_mmap_data_u64(ri, (u64 *)res,
							 st, &err);
			} else {
				pr_err_once("error: wrong dwarf size\n");
				return -QUADD_URC_UNHANDLED_INSTRUCTION;
			}

			/* we ignore links to unloaded sections */
			if (err)
				res = 0;
		}
	}

	*val = res;

	return count;
}

static long
dwarf_cfa_exec_insns(struct ex_region_info *ri,
		     unsigned char *insn_start,
		     unsigned char *insn_end,
		     struct dw_cie *cie,
		     struct stackframe *sf,
		     unsigned long pc)
{
	unsigned char insn;
	unsigned char *c_insn;
	unsigned int expr_len, delta;
	unsigned long utmp, reg;
	long offset, stmp, err = 0;
	struct regs_state *rs, *rs_initial, *rs_stack;
	struct dwarf_cpu_context *cpu_ctx = this_cpu_ptr(ctx.cpu_ctx);

	rs = &sf->rs;
	rs_initial = &sf->rs_initial;

	rs_stack = cpu_ctx->rs_stack;
	cpu_ctx->depth = 0;

	c_insn = insn_start;

	while (c_insn < insn_end && sf->pc <= pc) {
		insn = read_mmap_data_u8(ri, c_insn++, DW_SEC_TYPE_TAB, &err);
		if (err)
			return err;

		switch (dw_cfa_opcode(insn)) {
		case DW_CFA_advance_loc:
			delta = dw_cfa_operand(insn);
			delta *= cie->code_align_factor;
			sf->pc += delta;
			pr_debug("DW_CFA_advance_loc: pc: %#lx --> %#lx (delta: %#x)\n",
				sf->pc - delta, sf->pc, delta);
			continue;

		case DW_CFA_offset:
			reg = dw_cfa_operand(insn);
			c_insn += dwarf_read_uleb128(ri, c_insn, &utmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			offset = utmp * cie->data_align_factor;
			set_rule_offset(rs, reg, DW_WHERE_CFAREL, offset);
			pr_debug("DW_CFA_offset: reg: r%lu, offset(addr): %#lx (%ld)\n",
				reg, offset, offset);
			continue;

		case DW_CFA_restore:
			reg = dw_cfa_operand(insn);

			if (!validate_regnum(rs, reg))
				break;

			rs->reg[reg] = rs_initial->reg[reg];
			pr_debug("DW_CFA_restore: reg: r%lu\n", reg);
			continue;
		}

		switch (insn) {
		case DW_CFA_nop:
			pr_debug("DW_CFA_nop\n");
			continue;

		case DW_CFA_advance_loc1:
			delta = read_mmap_data_u8(ri, c_insn++,
						  DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			sf->pc += delta * cie->code_align_factor;
			pr_debug("DW_CFA_advance_loc1: pc: %#lx --> %#lx (delta: %#lx)\n",
				sf->pc - delta * cie->code_align_factor, sf->pc,
				delta * cie->code_align_factor);
			break;

		case DW_CFA_advance_loc2:
			delta = read_mmap_data_u16(ri, (u16 *)c_insn,
						   DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			c_insn += 2;
			sf->pc += delta * cie->code_align_factor;
			pr_debug("DW_CFA_advance_loc2: pc: %#lx --> %#lx (delta: %#lx)\n",
				sf->pc - delta * cie->code_align_factor, sf->pc,
				delta * cie->code_align_factor);
			break;

		case DW_CFA_advance_loc4:
			delta = read_mmap_data_u32(ri, (u32 *)c_insn,
						   DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			c_insn += 4;
			sf->pc += delta * cie->code_align_factor;
			pr_debug("DW_CFA_advance_loc4: pc: %#lx --> %#lx (delta: %#lx)\n",
				sf->pc - delta * cie->code_align_factor, sf->pc,
				delta * cie->code_align_factor);
			break;

		case DW_CFA_offset_extended:
			c_insn += dwarf_read_uleb128(ri, c_insn, &utmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			reg = utmp;
			c_insn += dwarf_read_uleb128(ri, c_insn, &utmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			offset = utmp * cie->data_align_factor;
			pr_debug("DW_CFA_offset_extended: reg: r%lu, offset: %#lx\n",
				 reg, offset);
			break;

		case DW_CFA_restore_extended:
			c_insn += dwarf_read_uleb128(ri, c_insn, &reg,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			pr_debug("DW_CFA_restore_extended: reg: r%lu\n", reg);
			break;

		case DW_CFA_undefined:
			c_insn += dwarf_read_uleb128(ri, c_insn, &reg,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			set_rule(rs, reg, DW_WHERE_UNDEF, 0);
			pr_debug("DW_CFA_undefined: reg: r%lu\n", reg);
			break;

		case DW_CFA_def_cfa:
			c_insn += dwarf_read_uleb128(ri, c_insn, &utmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			rs->cfa_register = utmp;
			c_insn += dwarf_read_uleb128(ri, c_insn, &utmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			rs->cfa_offset = utmp;
			pr_debug("DW_CFA_def_cfa: cfa_register: r%u, cfa_offset: %ld (%#lx)\n",
				 rs->cfa_register, rs->cfa_offset,
				 rs->cfa_offset);
			break;

		case DW_CFA_def_cfa_register:
			c_insn += dwarf_read_uleb128(ri, c_insn, &utmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			rs->cfa_register = utmp;
			pr_debug("DW_CFA_def_cfa_register: cfa_register: r%u\n",
			       rs->cfa_register);
			break;

		case DW_CFA_def_cfa_offset:
			c_insn += dwarf_read_uleb128(ri, c_insn, &utmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			rs->cfa_offset = utmp;
			pr_debug("DW_CFA_def_cfa_offset: cfa_offset: %ld (%#lx)\n",
			       rs->cfa_offset, rs->cfa_offset);
			break;

		case DW_CFA_def_cfa_expression:
			c_insn += dwarf_read_uleb128(ri, c_insn, &utmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			expr_len = utmp;

			rs->cfa_expr = c_insn;
			rs->cfa_expr_len = expr_len;
			rs->cfa_how = DW_CFA_EXP;
			c_insn += expr_len;

			pr_debug("DW_CFA_def_cfa_expression: expr_len: %#x\n",
				 expr_len);
			break;

		case DW_CFA_expression:
			c_insn += dwarf_read_uleb128(ri, c_insn, &reg,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			set_rule_exp(rs, reg, DW_WHERE_EXPR, c_insn);

			c_insn += dwarf_read_uleb128(ri, c_insn, &utmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			c_insn += utmp;

			pr_debug("DW_CFA_expression: reg: r%lu\n", reg);
			break;

		case DW_CFA_offset_extended_sf:
			c_insn += dwarf_read_uleb128(ri, c_insn, &reg,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			c_insn += dwarf_read_sleb128(ri, c_insn, &stmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			offset = stmp * cie->data_align_factor;
			set_rule_offset(rs, reg, DW_WHERE_CFAREL, offset);
			pr_debug("DW_CFA_offset_extended_sf: reg: r%lu, offset: %#lx\n",
				 reg, offset);
			break;

		case DW_CFA_val_offset:
			c_insn += dwarf_read_uleb128(ri, c_insn, &reg,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			c_insn += dwarf_read_uleb128(ri, c_insn, &utmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			offset = utmp * cie->data_align_factor;
			set_rule_offset(rs, reg, DW_WHERE_VAL_OFFSET, offset);
			pr_debug("DW_CFA_val_offset: reg: r%lu, offset(addr): %#lx\n",
				 reg, offset);
			break;

		case DW_CFA_val_offset_sf:
			c_insn += dwarf_read_uleb128(ri, c_insn, &reg,
				DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			c_insn += dwarf_read_sleb128(ri, c_insn, &stmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			offset = stmp * cie->data_align_factor;
			set_rule_offset(rs, reg, DW_WHERE_VAL_OFFSET, offset);
			pr_debug("DW_CFA_val_offset_sf: reg: r%lu, offset(addr): %#lx\n",
				 reg, offset);
			break;

		case DW_CFA_GNU_args_size:
			c_insn += dwarf_read_uleb128(ri, c_insn, &utmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			pr_debug("DW_CFA_GNU_args_size: offset: %#lx\n", utmp);
			break;

		case DW_CFA_GNU_negative_offset_extended:
			c_insn += dwarf_read_uleb128(ri, c_insn, &reg,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			c_insn += dwarf_read_uleb128(ri, c_insn, &utmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			offset = utmp * cie->data_align_factor;
			set_rule_offset(rs, reg, DW_WHERE_CFAREL, -offset);
			pr_debug("DW_CFA_GNU_negative_offset_extended: reg: r%lu, offset: %#lx\n",
				 reg, offset);
			break;

		case DW_CFA_remember_state:
			pr_debug("DW_CFA_remember_state\n");

			if (cpu_ctx->depth >= DW_MAX_RS_STACK_DEPTH) {
				pr_warn_once("error: rs stack was overflowed\n");
				return 0;
			}

			rs_stack[cpu_ctx->depth++] = *rs;
			break;

		case DW_CFA_restore_state:
			pr_debug("DW_CFA_restore_state\n");

			if (cpu_ctx->depth == 0) {
				pr_warn_once("error: rs stack error\n");
				return 0;
			}

			*rs = rs_stack[--cpu_ctx->depth];
			break;

		case DW_CFA_def_cfa_sf:
			c_insn += dwarf_read_uleb128(ri, c_insn, &utmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			c_insn += dwarf_read_sleb128(ri, c_insn, &stmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			rs->cfa_register = utmp;
			rs->cfa_offset = stmp * cie->data_align_factor;
			rs->cfa_how = DW_CFA_REG_OFFSET;

			pr_debug("DW_CFA_def_cfa_sf: cfa_register: r%u, cfa_offset: %ld (%#lx)\n",
				rs->cfa_register, rs->cfa_offset,
				rs->cfa_offset);
			break;

		case DW_CFA_def_cfa_offset_sf:
			c_insn += dwarf_read_sleb128(ri, c_insn, &stmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			rs->cfa_offset = stmp * cie->data_align_factor;
			pr_debug("DW_CFA_def_cfa_offset_sf: cfa_offset: %ld (%#lx)\n",
				rs->cfa_offset, rs->cfa_offset);
			break;

		case DW_CFA_same_value:
			c_insn += dwarf_read_uleb128(ri, c_insn, &reg,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			set_rule(rs, reg, DW_WHERE_SAME, 0);
			pr_debug("DW_CFA_same_value: reg: r%lu\n", reg);
			break;

		case DW_CFA_val_expression:
			c_insn += dwarf_read_uleb128(ri, c_insn, &reg,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			set_rule_exp(rs, reg, DW_WHERE_VAL_EXPR, c_insn);
			c_insn += dwarf_read_uleb128(ri, c_insn, &utmp,
						     DW_SEC_TYPE_TAB, &err);
			if (err)
				return err;

			c_insn += utmp;
			pr_debug("DW_CFA_val_expression: reg: r%lu\n", reg);
			break;

		default:
			pr_warn_once("warning: unhandled dwarf instr %#x\n",
				     insn);
			break;
		}
	}

	return 0;
}

static long
decode_cie_entry(struct ex_region_info *ri,
		 struct dw_cie *cie,
		 unsigned char *entry,
		 size_t length)
{
	long err;
	unsigned long utmp;
	unsigned char *p, *end, *aug;
	unsigned int cie_version, id, len, max_len;

	p = entry;
	end = entry + length;

	p += sizeof(u32);

	id = read_mmap_data_u32(ri, (u32 *)p, DW_SEC_TYPE_TAB, &err);
	if (err)
		return err;

	p += sizeof(u32);

	if (id != 0)
		return -QUADD_URC_TBL_IS_CORRUPT;

	cie_version = read_mmap_data_u8(ri, p++, DW_SEC_TYPE_TAB, &err);
	if (err)
		return err;

	if (cie_version != 1 && cie_version != 3) {
		pr_err_once("error: wrong cie_version: %u\n", cie_version);
		return -QUADD_URC_TBL_IS_CORRUPT;
	}

	if (p >= end)
		return -QUADD_URC_TBL_IS_CORRUPT;

	max_len = end - p - 1;
	len = strnlen((const char *)p, max_len);
	if (len == max_len)
		return -QUADD_URC_TBL_IS_CORRUPT;

	cie->aug_string = p;
	p += len + 1;

	pr_debug("aug_string: %s\n", cie->aug_string);

	p += dwarf_read_uleb128(ri, p, &cie->code_align_factor,
				DW_SEC_TYPE_TAB, &err);
	if (err)
		return err;

	p += dwarf_read_sleb128(ri, p, &cie->data_align_factor,
				DW_SEC_TYPE_TAB, &err);
	if (err)
		return err;

	if (cie_version == 1) {
		cie->retaddr_reg = read_mmap_data_u8(ri, p++,
						     DW_SEC_TYPE_TAB, &err);
		if (err)
			return err;
	} else {
		p += dwarf_read_uleb128(ri, p, &utmp, DW_SEC_TYPE_TAB, &err);
		if (err)
			return err;

		cie->retaddr_reg = utmp;
	}

	pr_debug("address column: %u\n", cie->retaddr_reg);

	aug = cie->aug_string;
	cie->z_aug = 0;

	cie->initial_insn = NULL;
	cie->initial_insn_len = 0;

	if (*aug == 'z') {
		p += dwarf_read_uleb128(ri, p, &cie->aug_size,
					DW_SEC_TYPE_TAB, &err);
		if (err)
			return err;

		cie->initial_insn = p + cie->aug_size;
		aug++;

		cie->z_aug = 1;
	} else {
		pr_warn_once("warning: !aug_z\n");
	}

	cie->fde_encoding = 0;
	cie->lsda_encoding = DW_EH_PE_omit;
	cie->personality = NULL;

	while (*aug != '\0') {
		if (p >= end)
			return -QUADD_URC_TBL_IS_CORRUPT;

		if (*aug == 'L') {
			cie->lsda_encoding = read_mmap_data_u8(ri, p++,
							       DW_SEC_TYPE_TAB,
							       &err);
			if (err)
				return err;

			aug++;
		} else if (*aug == 'R') {
			cie->fde_encoding = read_mmap_data_u8(ri, p++,
							      DW_SEC_TYPE_TAB,
							      &err);
			if (err)
				return err;

			aug++;
			pr_debug("fde_encoding: %#x\n", cie->fde_encoding);
		} else if (*aug == 'P') {
			int count;
			void *pcrel_base;
			unsigned char handler_encoding;
			unsigned long personality;

			handler_encoding = *p++;

			pcrel_base = (void *)
				mmap_addr_to_ex_addr((unsigned long)p,
						     ri, DW_SEC_TYPE_TAB);

			count = dwarf_read_encoded_value(ri, p, pcrel_base,
							 &personality,
							 handler_encoding,
							 DW_SEC_TYPE_TAB);
			if (count < 0) {
				pr_err_once("%s: error: personality routine\n",
					    __func__);
				return count;
			}
			p += count;

			pr_debug("personality: %#lx\n", personality);
			cie->personality = (void *)personality;
			aug++;
		} else if (*aug == 'S') {
			aug++;
			pr_debug("%s: aug: S\n", __func__);
		} else {
			pr_warn_once("%s: warning: unknown aug\n", __func__);
			return -QUADD_URC_UNHANDLED_INSTRUCTION;
		}
	}

	if (p > end) {
		pr_err_once("%s: error: cie\n", __func__);
		return -QUADD_URC_TBL_IS_CORRUPT;
	}

	if (p == end)
		return 0;

	if (!cie->initial_insn)
		cie->initial_insn = p;

	cie->initial_insn_len = end - cie->initial_insn;

	return 0;
}

static long
decode_fde_entry(struct ex_region_info *ri,
		 struct dw_fde *fde,
		 unsigned char *entry,
		 size_t length)
{
	int count;
	long err = 0;
	unsigned long utmp;
	unsigned char *p, *end, *pcrel_base;
	struct dw_cie *cie = fde->cie;

	p = entry;
	end = entry + length;

	p += sizeof(u32);
	p += sizeof(u32);

	pcrel_base = (unsigned char *)
		mmap_addr_to_ex_addr((unsigned long)p, ri, DW_SEC_TYPE_TAB);

	count = dwarf_read_encoded_value(ri, p, pcrel_base,
					 &fde->initial_location,
					 cie->fde_encoding,
					 DW_SEC_TYPE_TAB);
	if (count < 0)
		return count;

	p += count;

	fde->address_range = read_mmap_data_u32(ri, (u32 *)p,
						DW_SEC_TYPE_TAB, &err);
	if (err)
		return err;

	p += sizeof(u32);

	pr_debug("init location: %#lx\n", fde->initial_location);
	pr_debug("address_range: %#lx\n", fde->address_range);

	if (cie->z_aug) {
		p += dwarf_read_uleb128(ri, p, &utmp, DW_SEC_TYPE_TAB, &err);
		if (err)
			return err;

		p += utmp;
	}

	if (p > end) {
		pr_err_once("%s: error: incorrect fde\n", __func__);
		return -QUADD_URC_TBL_IS_CORRUPT;
	}

	fde->insn_length = end - p;

	if (fde->insn_length > 0)
		fde->instructions = p;
	else
		fde->instructions = NULL;

	return 0;
}

static const struct dw_fde_table *
dwarf_bst_find_idx(unsigned long data_base,
		   struct dw_fde_table *fde_table,
		   unsigned long length,
		   unsigned long addr)
{
	unsigned long initial_loc;
	struct dw_fde_table *start, *stop;
	struct dw_fde_table *mid = NULL;

	if (unlikely(!length))
		return NULL;

	start = fde_table;
	stop = start + length - 1;

	initial_loc = dw_bst_get_initial_loc(start, data_base);
	if (addr < initial_loc)
		return NULL;

	initial_loc = dw_bst_get_initial_loc(stop, data_base);
	if (addr >= initial_loc)
		return NULL;

	while (start < stop - 1) {
		mid = start + ((stop - start) >> 1);

		initial_loc = dw_bst_get_initial_loc(mid, data_base);

		if (addr < initial_loc)
			stop = mid;
		else
			start = mid;
	}

	return start;
}

static struct dw_fde_table *
dwarf_get_bs_table(struct ex_region_info *ri,
		   void *data,
		   unsigned long length,
		   unsigned long data_base,
		   unsigned long *nr_entries)
{
	int count;
	unsigned char *p, *end;
	struct dw_fde_table *bst;
	unsigned long fde_count, frame_ptr;
	struct dw_eh_frame_hdr *hdr = data;

	if (length <= sizeof(*hdr))
		return NULL;

	end = data + length;

	if (hdr->version != 1) {
		pr_warn_once("warning: unknown eh hdr format\n");
		return NULL;
	}
	p = (unsigned char *)(hdr + 1);

	if (hdr->eh_frame_ptr_enc != DW_EH_PE_omit) {
		count = dwarf_read_encoded_value(ri, p, (void *)data_base,
						 &frame_ptr,
						 hdr->eh_frame_ptr_enc,
						 DW_SEC_TYPE_IDX);
		if (count < 0)
			return NULL;

		p += count;
	}

	if (hdr->fde_count_enc == DW_EH_PE_omit)
		return NULL;

	count = dwarf_read_encoded_value(ri, p, (void *)data_base,
					 &fde_count, hdr->fde_count_enc,
					 DW_SEC_TYPE_IDX);
	if (count < 0)
		return NULL;

	p += count;

	if (p >= end)
		return NULL;

	if (fde_count * sizeof(*bst) !=  end - p)
		return NULL;

	if (hdr->table_enc != (DW_EH_PE_datarel | DW_EH_PE_sdata4)) {
		pr_warn_once("warning: unknown eh hdr format\n");
		return NULL;
	}

	bst = (struct dw_fde_table *)p;
	*nr_entries = fde_count;

	return bst;
}

static long
dwarf_decode_fde_cie(struct ex_region_info *ri,
		     unsigned char *fde_p,
		     struct dw_cie *cie,
		     struct dw_fde *fde)
{
	u32 *p;
	long err;
	unsigned char *cie_p;
	unsigned long cie_pointer, length;
	unsigned char *frame_start;
	unsigned long frame_len, addr;

	addr = ri->tabs.extab.addr;

	frame_start = (unsigned char *)
		ex_addr_to_mmap_addr(addr, ri, DW_SEC_TYPE_TAB);
	frame_len = ri->tabs.extab.length;

	pr_debug("eh frame: %p - %p\n",
		 frame_start, frame_start + frame_len);

	p = (u32 *)fde_p;

	length = read_mmap_data_u32(ri, p++, DW_SEC_TYPE_TAB, &err);
	if (err)
		return err;

	if (length == 0xffffffff) {
		pr_warn_once("warning: 64-bit .eh_frame is not supported\n");
		return -QUADD_URC_UNHANDLED_INSTRUCTION;
	}

	fde->offset = fde_p - frame_start;
	fde->length = length + sizeof(u32);

	pr_debug("FDE: fde_p: %p, offset: %#lx, len: %#lx\n",
		 fde_p, fde->offset, fde->length);

	cie_pointer = read_mmap_data_u32(ri, p, DW_SEC_TYPE_TAB, &err);
	if (err)
		return err;

	fde->cie_pointer = cie_pointer;
	cie_p = (unsigned char *)p - cie_pointer;

	length = read_mmap_data_u32(ri, (u32 *)cie_p, DW_SEC_TYPE_TAB, &err);
	if (err)
		return err;

	if (length == 0xffffffff) {
		pr_warn_once("warning: 64-bit .eh_frame is not supported\n");
		return -QUADD_URC_UNHANDLED_INSTRUCTION;
	}

	cie->offset = cie_p - frame_start;
	cie->length = length + sizeof(u32);

	pr_debug("CIE: cie_p: %p, offset: %#lx, len: %#lx\n",
		 cie_p, cie->offset, cie->length);

	err = decode_cie_entry(ri, cie, cie_p, cie->length);
	if (err < 0)
		return err;

	fde->cie = cie;

	err = decode_fde_entry(ri, fde, fde_p, fde->length);
	if (err < 0)
		return err;

	return 0;
}

static void *
dwarf_find_fde(struct ex_region_info *ri,
	       void *data,
	       unsigned long length,
	       unsigned long pc)
{
	long err;
	const struct dw_fde_table *fi;
	unsigned long fde_count = 0, data_base;
	unsigned long fde_addr, init_loc;
	struct dw_fde_table *bst;

	data_base = ri->tabs.exidx.addr;

	bst = dwarf_get_bs_table(ri, data, length, data_base, &fde_count);
	if (!bst || fde_count == 0) {
		pr_warn_once("warning: bs_table\n");
		return NULL;
	}

	fi = &bst[fde_count - 1];
	init_loc = dw_bst_get_initial_loc(fi, data_base);

	if (pc >= init_loc) {
		unsigned long start, end;

		fde_addr = dw_bst_get_fde_addr(fi, data_base);
		fde_addr = ex_addr_to_mmap_addr(fde_addr, ri, DW_SEC_TYPE_TAB);

		if (pc == init_loc)
			return (void *)fde_addr;

		if (ri->tf_end > 0) {
			start = ri->tf_start;
			end = ri->tf_end;
		} else {
			struct dw_cie cie;
			struct dw_fde fde;

			err = dwarf_decode_fde_cie(ri, (void *)fde_addr,
						   &cie, &fde);
			if (err < 0)
				return NULL;

			start = fde.initial_location;
			end = start + fde.address_range;

			quadd_unwind_set_tail_info(ri->vm_start, start, end);
		}

		return (pc >= start && pc < end) ?
		       (void *)fde_addr : NULL;
	}

	fi = dwarf_bst_find_idx(data_base, bst, fde_count, pc);
	if (!fi)
		return NULL;

	fde_addr = dw_bst_get_fde_addr(fi, data_base);
	fde_addr = ex_addr_to_mmap_addr(fde_addr, ri, DW_SEC_TYPE_TAB);

	return (void *)fde_addr;
}

static long
dwarf_decode(struct ex_region_info *ri,
	     struct dw_cie *cie,
	     struct dw_fde *fde,
	     unsigned long pc)
{
	long err;
	unsigned char *fde_p;
	unsigned char *hdr_start;
	unsigned long hdr_len, addr;

	addr = ri->tabs.exidx.addr;

	hdr_start = (unsigned char *)
		ex_addr_to_mmap_addr(addr, ri, DW_SEC_TYPE_IDX);
	hdr_len = ri->tabs.exidx.length;

	pr_debug("eh frame hdr: %p - %p\n",
		 hdr_start, hdr_start + hdr_len);

	fde_p = dwarf_find_fde(ri, hdr_start, hdr_len, pc);
	if (!fde_p)
		return -QUADD_URC_IDX_NOT_FOUND;

	err = dwarf_decode_fde_cie(ri, fde_p, cie, fde);
	if (err < 0)
		return err;

	if (pc < fde->initial_location ||
	    pc >= fde->initial_location + fde->address_range) {
		pr_debug("pc is not in range: %#lx - %#lx\n",
			 fde->initial_location,
			 fde->initial_location + fde->address_range);
		return -QUADD_URC_IDX_NOT_FOUND;
	}

	return 0;
}

static long
unwind_frame(struct ex_region_info *ri,
	     struct stackframe *sf,
	     struct vm_area_struct *vma_sp,
	     unsigned int *unw_type)
{
	long err;
	unsigned char *insn_end;
	unsigned long addr, return_addr, fp;
	struct dw_fde fde;
	struct dw_cie cie;
	unsigned long pc = sf->pc;
	struct regs_state *rs, *rs_initial;

	err = dwarf_decode(ri, &cie, &fde, pc);
	if (err < 0)
		return err;

	sf->pc = fde.initial_location;

	rs = &sf->rs;
	rs_initial = &sf->rs_initial;

	set_rule(rs, LR, DW_WHERE_UNDEF, 0);

	if (cie.initial_insn) {
		insn_end = cie.initial_insn + cie.initial_insn_len;
		err = dwarf_cfa_exec_insns(ri, cie.initial_insn,
					   insn_end, &cie, sf, pc);
		if (err)
			return err;
	}

	memcpy(rs_initial, rs, sizeof(*rs));

	if (fde.instructions) {
		insn_end = fde.instructions + fde.insn_length;
		err = dwarf_cfa_exec_insns(ri, fde.instructions,
					   insn_end, fde.cie, sf, pc);
		if (err)
			return err;
	}

	if (!sf->cfa)
		sf->cfa = sf->sp + rs->cfa_offset;
	else
		sf->cfa += rs->cfa_offset;

	pr_debug("pc: %#lx, lr: %#lx\n", sf->pc, sf->lr);
	pr_debug("sp: %#lx, fp: %#lx\n", sf->sp, sf->fp);

	pr_debug("fp rule: %#lx/%ld\n",
		 rs->reg[FP].loc.reg, rs->reg[FP].loc.offset);
	pr_debug("lr rule: %#lx/%ld\n",
		 rs->reg[LR].loc.reg, rs->reg[LR].loc.offset);

	pr_debug("cfa_offset: %ld (%#lx)\n",
		 rs->cfa_offset, rs->cfa_offset);
	pr_debug("cfa_register: %u\n", rs->cfa_register);
	pr_debug("sf->cfa: %#lx\n", sf->cfa);

	if (rs->reg[LR].where == DW_WHERE_CFAREL) {
		addr = sf->cfa + rs->reg[LR].loc.offset;
		pr_debug("lr: cfa addr: %#lx\n", addr);

		if (!validate_stack_addr(addr, vma_sp, sizeof(unsigned long)))
			return -QUADD_URC_SP_INCORRECT;

		err = read_user_data((unsigned long *)addr, return_addr);
		if (err < 0)
			return err;

		*unw_type = QUADD_UNW_TYPE_UT;
	} else {
		return_addr = sf->lr;
		*unw_type = QUADD_UNW_TYPE_LR_UT;
	}

	if (!validate_pc_addr(return_addr, sizeof(unsigned long)))
		return -QUADD_URC_PC_INCORRECT;

	sf->pc = return_addr;

	if (rs->reg[FP].where == DW_WHERE_CFAREL) {
		addr = sf->cfa + rs->reg[FP].loc.offset;
		pr_debug("fp: cfa addr: %#lx\n", addr);

		if (!validate_stack_addr(addr, vma_sp, sizeof(unsigned long)))
			return -QUADD_URC_SP_INCORRECT;

		err = read_user_data((unsigned long *)addr, fp);
		if (err < 0)
			return err;

		sf->fp = fp;
	}

	sf->sp = sf->cfa;

	return 0;
}

static void
unwind_backtrace(struct quadd_callchain *cc,
		 struct ex_region_info *ri,
		 struct stackframe *sf,
		 struct vm_area_struct *vma_sp,
		 struct task_struct *task)
{
	unsigned int unw_type = QUADD_UNW_TYPE_UT;
	struct ex_region_info ri_new;

	cc->unw_rc = QUADD_URC_FAILURE;

	while (1) {
		long err;
		int nr_added;
		struct vm_area_struct *vma_pc;
		unsigned long addr, where = sf->pc;
		struct mm_struct *mm = task->mm;

		if (!mm)
			break;

		if (!validate_stack_addr(sf->sp, vma_sp, sizeof(sf->sp))) {
			cc->unw_rc = -QUADD_URC_SP_INCORRECT;
			break;
		}

		vma_pc = find_vma(mm, sf->pc);
		if (!vma_pc)
			break;

		addr = ri->tabs.exidx.addr;

		if (!is_vma_addr(addr, vma_pc, sizeof(unsigned long))) {
			err = quadd_search_ex_region(vma_pc->vm_start, &ri_new);
			if (err) {
				cc->unw_rc = QUADD_URC_TBL_NOT_EXIST;
				break;
			}

			ri = &ri_new;
		}

		err = unwind_frame(ri, sf, vma_sp, &unw_type);
		if (err < 0) {
			cc->unw_rc = -err;
			break;
		}

		pr_debug("function at [<%08lx>] from [<%08lx>]\n",
			 where, sf->pc);

		cc->curr_sp = sf->sp;
		cc->curr_fp = sf->fp;
		cc->curr_pc = sf->pc;

		nr_added = quadd_callchain_store(cc, sf->pc, unw_type);
		if (nr_added == 0)
			break;
	}
}

int
quadd_aarch64_is_ex_entry_exist(struct pt_regs *regs,
				unsigned long addr,
				struct task_struct *task)
{
	long err;
	unsigned char *fde_p;
	struct ex_region_info ri;
	unsigned char *hdr_start;
	unsigned long hdr_len, a;
	struct vm_area_struct *vma;
	struct mm_struct *mm = task->mm;

	if (!regs || !mm)
		return 0;

	vma = find_vma(mm, addr);
	if (!vma)
		return 0;

	err = quadd_search_ex_region(vma->vm_start, &ri);
	if (err)
		return 0;

	a = ri.tabs.exidx.addr;

	hdr_start = (unsigned char *)
		ex_addr_to_mmap_addr(a, &ri, DW_SEC_TYPE_IDX);
	hdr_len = ri.tabs.exidx.length;

	fde_p = dwarf_find_fde(&ri, hdr_start, hdr_len, addr);
	if (!fde_p)
		return 0;

	return 1;
}

unsigned int
quadd_aarch64_get_user_callchain_ut(struct pt_regs *regs,
				    struct quadd_callchain *cc,
				    struct task_struct *task)
{
	long err;
	int i, nr_prev = cc->nr;
	unsigned long ip, lr, sp, fp;
	struct vm_area_struct *vma, *vma_sp;
	struct mm_struct *mm = task->mm;
	struct ex_region_info ri;
	struct stackframe sf;

	if (!regs || !mm)
		return 0;

	if (cc->unw_rc == QUADD_URC_LEVEL_TOO_DEEP)
		return nr_prev;

	cc->unw_rc = QUADD_URC_FAILURE;

	if (nr_prev > 0) {
		ip = cc->curr_pc;
		sp = cc->curr_sp;
		fp = cc->curr_fp;
		lr = 0;
	} else {
		ip = instruction_pointer(regs);
		lr = quadd_user_link_register(regs);
		sp = quadd_user_stack_pointer(regs);
		fp = quadd_get_user_frame_pointer(regs);
	}

	pr_debug("%s: pc: %#lx, lr: %#lx\n", __func__, ip, lr);
	pr_debug("%s: sp: %#lx, fp: %#lx\n", __func__, sp, fp);

	sf.pc = ip;
	sf.lr = lr;
	sf.sp = sp;
	sf.fp = fp;

	sf.cfa = 0;

	for (i = 0; i < ARRAY_SIZE(sf.rs.reg); i++)
		set_rule(&sf.rs, i, DW_WHERE_UNDEF, 0);

	vma = find_vma(mm, ip);
	if (!vma)
		return 0;

	vma_sp = find_vma(mm, sp);
	if (!vma_sp)
		return 0;

	err = quadd_search_ex_region(vma->vm_start, &ri);
	if (err) {
		cc->unw_rc = QUADD_URC_TBL_NOT_EXIST;
		return 0;
	}

	unwind_backtrace(cc, &ri, &sf, vma_sp, task);

	return cc->nr;
}

int quadd_dwarf_unwind_start(void)
{
	if (!atomic_cmpxchg(&ctx.started, 0, 1)) {
		ctx.cpu_ctx = alloc_percpu(struct dwarf_cpu_context);
		if (!ctx.cpu_ctx)
			return -ENOMEM;
	}

	return 0;
}

void quadd_dwarf_unwind_stop(void)
{
	if (atomic_cmpxchg(&ctx.started, 1, 0))
		free_percpu(ctx.cpu_ctx);
}

int quadd_dwarf_unwind_init(void)
{
	atomic_set(&ctx.started, 0);
	return 0;
}
