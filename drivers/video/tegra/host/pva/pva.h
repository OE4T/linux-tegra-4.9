/*
 * drivers/video/tegra/host/pva/pva.h
 *
 * Tegra PVA header
 *
 * Copyright (c) 2016 NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_PVA_H__
#define __NVHOST_PVA_H__

#include <linux/dma-attrs.h>
#include <linux/mutex.h>

#include "nvhost_queue.h"
#include "pva_regs.h"

extern const struct file_operations tegra_pva_ctrl_ops;

/**
 * Queue count of 8 is maintained per PVA.
 */
#define MAX_PVA_QUEUE_COUNT 8

/**
 * struct pva_fw - struct to handle the PVA firmware information
 *
 * @hdr:		pointer to the pva_code_hdr struct
 * @size:		firmware file size
 * @booted:		variable to check whether boot completed
 * @ucode_phys:		physical address of dram for ucode image
 * @ucode_mapped:	virtual address of dram for ucode image
 * @priv2_buffer_phys:	physical address of extra memory allocated for ucode
 * @priv2_buffer_mapped:virtual address of extra memory allocated for ucode
 * @priv2_buffer_size:	extra buffer size allocated for ucode
 * @attrs:		dma_attrs struct information
 * @trace_buffer_size:	buffer size for trace log
 *
 */

struct pva_fw {
	struct pva_ucode_hdr *hdr;

	size_t size;

	dma_addr_t ucode_phys;
	void *ucode_mapped;
	dma_addr_t priv2_buffer_phys;
	void *priv2_buffer_mapped;
	size_t priv2_buffer_size;
	struct dma_attrs attrs;

	u32 trace_buffer_size;
};

/**
 * struct pva - Driver private data, shared with all applications
 *
 * @pdev:			Pointer to the PVA device
 * @pool:			Pointer to Queue table available for the PVA
 * @fw_info:			firmware information struct
 * @irq:			IRQ number obtained on registering the module
 * @mailbox_mutex:		Mutex to avoid concurrent mailbox accesses
 * @mailbox_waitq:		Mailbox waitqueue for response waiters
 * @mailbox_status_regs:	Response is stored into this structure temporarily
 * @mailbox_status:		Status of the mailbox interface
 *
 */
struct pva {
	struct platform_device *pdev;
	struct nvhost_queue_pool *pool;
	struct pva_fw fw_info;

	int irq;

	wait_queue_head_t mailbox_waitqueue;
	struct pva_mailbox_status_regs mailbox_status_regs;
	enum pva_mailbox_status mailbox_status;
	struct mutex mailbox_mutex;
};

/**
 * pva_finalize_poweron() - Finalize the PVA Power-on-Sequence.
 *
 * @pdev:	Pointer to PVA device
 *
 * Return:	0 on Success or negative error code
 *
 * This function called from host subsystem driver after the PVA
 * partition has been brought up, clocks enabled and reset deasserted.
 * In production mode, the function needs to wait until the ready  bit
 * within the PVA aperture has been set. After that enable the PVA IRQ.
 * Register the queue priorities on the PVA.
 */
int pva_finalize_poweron(struct platform_device *pdev);

/**
 * pva_prepare_poweroff() - Prepare PVA poweroff.
 *
 * @pdev:	Pointer to PVA device
 *
 * Return:	0 on Success or negative error code
 *
 * This function called from host subsystem driver before turning off
 * the PVA. The function should turn off the PVA IRQ.
 */
int pva_prepare_poweroff(struct platform_device *pdev);
/**
 * pva_register_isr() - Register PVA ISR.
 *
 * @pdev:	Pointer to PVA device
 *
 * Return:	0 on Success or negative error code
 *
 * This function called from driver to register the
 * PVA ISR with IRQ.
 */
int pva_register_isr(struct platform_device *dev);
#endif
