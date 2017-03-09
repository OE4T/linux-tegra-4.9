/*
 * GV11B Fifo
 *
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef FIFO_GV11B_H
#define FIFO_GV11B_H

#define FIFO_INVAL_PBDMA_ID	((u32)~0)

/* engine context-switch request occurred while the engine was in reset */
#define SCHED_ERROR_CODE_ENGINE_RESET      0x00000005

/*
* ERROR_CODE_BAD_TSG indicates that Host encountered a badly formed TSG header
* or a badly formed channel type runlist entry in the runlist. This is typically
* caused by encountering a new TSG entry in the middle of a TSG definition.
* A channel type entry having wrong runqueue selector can also cause this.
* Additionally this error code can indicate when a channel is encountered on
* the runlist which is outside of a TSG.
*/
#define SCHED_ERROR_CODE_BAD_TSG           0x00000020


struct gpu_ops;
void gv11b_init_fifo(struct gpu_ops *gops);
#endif
