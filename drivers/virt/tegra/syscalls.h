/*
 * Copyright (C) 2014, NVIDIA CORPORATION. All rights reserved.
 *
 * Hypervisor interfaces
 *
 * This header is BSD licensed so anyone can use the definitions to implement
 * compatible drivers/servers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of NVIDIA CORPORATION nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL NVIDIA CORPORATION OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __VMM_SYSCALLS_H__
#define __VMM_SYSCALLS_H__

#define HVC_NR_READ_STAT	1
#define HVC_NR_READ_IVC		2
#define HVC_NR_READ_GID		3
#define HVC_NR_RAISE_IRQ	4
#define HVC_NR_READ_NGUESTS	5

#define GUEST_PRIMARY		0
#define GUEST_IVC_SERVER	0

#ifndef __ASSEMBLY__

/*
 * IVC communication information for each unique guest pair.
 */
struct hyp_ivc_info {
	unsigned long base;	/* Base of private shared memory segments */
	unsigned long size;	/* Size of shared memory segments */
	unsigned long virq_base;	/* Base of virtual interrupts */
	unsigned int  virq_total;	/* Number of irqs allocated per guest */
};

int hyp_read_gid(unsigned int *gid);
int hyp_read_nguests(unsigned int *nguests);
int hyp_read_ivc_info(struct hyp_ivc_info *data, int guestid);
int hyp_raise_irq(unsigned int irq, unsigned int vmid);

/* ASM prototypes */
extern int hvc_read_gid(void *);
extern int hvc_read_ivc_info(void *, int guestid);
extern int hvc_read_nguests(void *);
extern int hvc_raise_irq(unsigned int irq, unsigned int vmid);

#endif /* !__ASSEMBLY__ */

#endif /* __VMM_SYSCALLS_H__ */
