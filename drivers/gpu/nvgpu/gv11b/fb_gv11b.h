/*
 * GV11B FB
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef _NVGPU_GV11B_FB
#define _NVGPU_GV11B_FB

#define	STALL_REG_INDEX			0
#define	NONSTALL_REG_INDEX		1

#define	NONREPLAY_REG_INDEX		0
#define	REPLAY_REG_INDEX		1

#define	FAULT_BUF_DISABLED		0
#define	FAULT_BUF_ENABLED		1

#define	FAULT_BUF_INVALID		0
#define	FAULT_BUF_VALID			1

#define	HUB_INTR_TYPE_OTHER		1	/* bit 0 */
#define	HUB_INTR_TYPE_NONREPLAY		2	/* bit 1 */
#define	HUB_INTR_TYPE_REPLAY		4	/* bit 2 */
#define	HUB_INTR_TYPE_ECC_UNCORRECTED	8	/* bit 3 */
#define	HUB_INTR_TYPE_ACCESS_COUNTER	16	/* bit 4 */
#define	HUB_INTR_TYPE_ALL		(HUB_INTR_TYPE_OTHER | \
					 HUB_INTR_TYPE_NONREPLAY | \
					 HUB_INTR_TYPE_REPLAY | \
					 HUB_INTR_TYPE_ECC_UNCORRECTED | \
					 HUB_INTR_TYPE_ACCESS_COUNTER)

#define	FAULT_TYPE_OTHER_AND_NONREPLAY		0
#define	FAULT_TYPE_REPLAY			1

struct gk20a;

void gv11b_fb_init_fs_state(struct gk20a *g);
void gv11b_fb_init_cbc(struct gk20a *g, struct gr_gk20a *gr);
void gv11b_fb_reset(struct gk20a *g);
void gv11b_fb_hub_isr(struct gk20a *g);

u32 gv11b_fb_is_fault_buf_enabled(struct gk20a *g,
				 unsigned int index);
void gv11b_fb_fault_buf_set_state_hw(struct gk20a *g,
		 unsigned int index, unsigned int state);
void gv11b_fb_fault_buf_configure_hw(struct gk20a *g, unsigned int index);
void gv11b_fb_enable_hub_intr(struct gk20a *g,
	 unsigned int index, unsigned int intr_type);
void gv11b_fb_disable_hub_intr(struct gk20a *g,
	 unsigned int index, unsigned int intr_type);
bool gv11b_fb_mmu_fault_pending(struct gk20a *g);

noinline_for_stack void gv11b_init_uncompressed_kind_map(void);
void gv11b_init_kind_attr(void);
#endif
