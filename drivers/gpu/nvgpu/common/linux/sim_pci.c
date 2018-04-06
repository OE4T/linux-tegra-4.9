/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/io.h>
#include <linux/highmem.h>
#include <linux/platform_device.h>

#include <nvgpu/log.h>
#include <nvgpu/linux/vm.h>

#include "gk20a/gk20a.h"
#include "os_linux.h"
#include "sim.h"
#include "hw_sim_pci.h"

static inline void sim_writel(struct sim_gk20a_linux *sim_linux, u32 r, u32 v)
{
	writel(v, sim_linux->regs + r);
}

static inline u32 sim_readl(struct sim_gk20a_linux *sim_linux, u32 r)
{
	return readl(sim_linux->regs + r);
}

static void kunmap_and_free_iopage(void **kvaddr, struct page **page)
{
	if (*kvaddr) {
		kunmap(*kvaddr);
		*kvaddr = NULL;
	}
	if (*page) {
		__free_page(*page);
		*page = NULL;
	}
}

static void gk20a_free_sim_support(struct gk20a *g)
{
	struct sim_gk20a_linux *sim_linux =
		container_of(g->sim, struct sim_gk20a_linux, sim);
	/* free sim mappings, bfrs */
	kunmap_and_free_iopage(&sim_linux->send_bfr.kvaddr,
			       &sim_linux->send_bfr.page);

	kunmap_and_free_iopage(&sim_linux->recv_bfr.kvaddr,
			       &sim_linux->recv_bfr.page);

	kunmap_and_free_iopage(&sim_linux->msg_bfr.kvaddr,
			       &sim_linux->msg_bfr.page);
}

static void gk20a_remove_sim_support(struct sim_gk20a *s)
{
	struct gk20a *g = s->g;
	struct sim_gk20a_linux *sim_linux =
		container_of(g->sim, struct sim_gk20a_linux, sim);

	if (sim_linux->regs)
		sim_writel(sim_linux, sim_config_r(), sim_config_mode_disabled_v());
	gk20a_free_sim_support(g);

	if (sim_linux->regs) {
		iounmap(sim_linux->regs);
		sim_linux->regs = NULL;
	}

	nvgpu_kfree(g, sim_linux);
	g->sim = NULL;
}

static int alloc_and_kmap_iopage(struct gk20a *g,
				 void **kvaddr,
				 u64 *phys,
				 struct page **page)
{
	int err = 0;
	*page = alloc_page(GFP_KERNEL);

	if (!*page) {
		err = -ENOMEM;
		nvgpu_err(g, "couldn't allocate io page");
		goto fail;
	}

	*kvaddr = kmap(*page);
	if (!*kvaddr) {
		err = -ENOMEM;
		nvgpu_err(g, "couldn't kmap io page");
		goto fail;
	}
	*phys = page_to_phys(*page);
	return 0;

 fail:
	kunmap_and_free_iopage(kvaddr, page);
	return err;

}

static inline u32 sim_msg_header_size(void)
{
	return 32U;
}

static inline u32 *sim_msg_bfr(struct gk20a *g, u32 byte_offset)
{
	struct sim_gk20a_linux *sim_linux =
		container_of(g->sim, struct sim_gk20a_linux, sim);
	return (u32 *)(sim_linux->msg_bfr.kvaddr + byte_offset);
}

static inline u32 *sim_msg_hdr(struct gk20a *g, u32 byte_offset)
{
	return sim_msg_bfr(g, byte_offset); /* starts at 0 */
}

static inline u32 *sim_msg_param(struct gk20a *g, u32 byte_offset)
{
	/* starts after msg header/cmn */
	return sim_msg_bfr(g, byte_offset + sim_msg_header_size());
}

static inline void sim_write_hdr(struct gk20a *g, u32 func, u32 size)
{
	*sim_msg_hdr(g, sim_msg_header_version_r()) =
		sim_msg_header_version_major_tot_v() |
		sim_msg_header_version_minor_tot_v();
	*sim_msg_hdr(g, sim_msg_signature_r()) = sim_msg_signature_valid_v();
	*sim_msg_hdr(g, sim_msg_result_r())    = sim_msg_result_rpc_pending_v();
	*sim_msg_hdr(g, sim_msg_spare_r())     = sim_msg_spare__init_v();
	*sim_msg_hdr(g, sim_msg_function_r())  = func;
	*sim_msg_hdr(g, sim_msg_length_r())    = size + sim_msg_header_size();
}

static inline u32 sim_escape_read_hdr_size(void)
{
	return 12U;
}

static u32 *sim_send_ring_bfr(struct gk20a *g, u32 byte_offset)
{
	struct sim_gk20a_linux *sim_linux =
		container_of(g->sim, struct sim_gk20a_linux, sim);
	return (u32 *)(sim_linux->send_bfr.kvaddr + byte_offset);
}

static int rpc_send_message(struct gk20a *g)
{
	/* calculations done in units of u32s */
	u32 send_base = sim_send_put_pointer_v(g->sim->send_ring_put) * 2;
	u32 dma_offset = send_base + sim_dma_r()/sizeof(u32);
	u32 dma_hi_offset = send_base + sim_dma_hi_r()/sizeof(u32);
	struct sim_gk20a_linux *sim_linux =
		container_of(g->sim, struct sim_gk20a_linux, sim);

	*sim_send_ring_bfr(g, dma_offset*sizeof(u32)) =
		sim_dma_target_phys_pci_coherent_f() |
		sim_dma_status_valid_f() |
		sim_dma_size_4kb_f() |
		sim_dma_addr_lo_f(sim_linux->msg_bfr.phys >> PAGE_SHIFT);

	*sim_send_ring_bfr(g, dma_hi_offset*sizeof(u32)) =
		u64_hi32(sim_linux->msg_bfr.phys);

	*sim_msg_hdr(g, sim_msg_sequence_r()) = g->sim->sequence_base++;

	g->sim->send_ring_put = (g->sim->send_ring_put + 2 * sizeof(u32)) %
		PAGE_SIZE;

	/* Update the put pointer. This will trap into the host. */
	sim_writel(sim_linux, sim_send_put_r(), g->sim->send_ring_put);

	return 0;
}

static inline u32 *sim_recv_ring_bfr(struct gk20a *g, u32 byte_offset)
{
	struct sim_gk20a_linux *sim_linux =
		container_of(g->sim, struct sim_gk20a_linux, sim);
	return (u32 *)(sim_linux->recv_bfr.kvaddr + byte_offset);
}

static int rpc_recv_poll(struct gk20a *g)
{
	u64 recv_phys_addr;
	struct sim_gk20a_linux *sim_linux =
		container_of(g->sim, struct sim_gk20a_linux, sim);

	/* Poll the recv ring get pointer in an infinite loop */
	do {
		g->sim->recv_ring_put = sim_readl(sim_linux, sim_recv_put_r());
	} while (g->sim->recv_ring_put == g->sim->recv_ring_get);

	/* process all replies */
	while (g->sim->recv_ring_put != g->sim->recv_ring_get) {
		/* these are in u32 offsets */
		u32 dma_lo_offset =
			sim_recv_put_pointer_v(g->sim->recv_ring_get)*2 + 0;
		u32 dma_hi_offset = dma_lo_offset + 1;
		u32 recv_phys_addr_lo = sim_dma_addr_lo_v(
				*sim_recv_ring_bfr(g, dma_lo_offset*4));
		u32 recv_phys_addr_hi = sim_dma_hi_addr_v(
				*sim_recv_ring_bfr(g, dma_hi_offset*4));

		recv_phys_addr = (u64)recv_phys_addr_hi << 32 |
				 (u64)recv_phys_addr_lo << PAGE_SHIFT;

		if (recv_phys_addr != sim_linux->msg_bfr.phys) {
			nvgpu_err(g, "Error in RPC reply");
			return -EINVAL;
		}

		/* Update GET pointer */
		g->sim->recv_ring_get = (g->sim->recv_ring_get + 2*sizeof(u32)) %
			PAGE_SIZE;

		sim_writel(sim_linux, sim_recv_get_r(), g->sim->recv_ring_get);

		g->sim->recv_ring_put = sim_readl(sim_linux, sim_recv_put_r());
	}

	return 0;
}

static int issue_rpc_and_wait(struct gk20a *g)
{
	int err;

	err = rpc_send_message(g);
	if (err) {
		nvgpu_err(g, "failed rpc_send_message");
		return err;
	}

	err = rpc_recv_poll(g);
	if (err) {
		nvgpu_err(g, "failed rpc_recv_poll");
		return err;
	}

	/* Now check if RPC really succeeded */
	if (*sim_msg_hdr(g, sim_msg_result_r()) != sim_msg_result_success_v()) {
		nvgpu_err(g, "received failed status!");
		return -EINVAL;
	}
	return 0;
}

static int gk20a_sim_esc_readl(struct gk20a *g, char *path, u32 index, u32 *data)
{
	int err;
	size_t pathlen = strlen(path);
	u32 data_offset;

	sim_write_hdr(g, sim_msg_function_sim_escape_read_v(),
		      sim_escape_read_hdr_size());
	*sim_msg_param(g, 0) = index;
	*sim_msg_param(g, 4) = sizeof(u32);
	data_offset = roundup(pathlen + 1, sizeof(u32));
	*sim_msg_param(g, 8) = data_offset;
	strcpy((char *)sim_msg_param(g, 0xc), path);

	err = issue_rpc_and_wait(g);

	if (!err)
		memcpy(data, sim_msg_param(g, data_offset + 0xc), sizeof(u32));
	return err;
}

static bool _nvgpu_pci_is_simulation(struct gk20a *g, u32 sim_base)
{
	u32 cfg;
	bool is_simulation = false;

	cfg = nvgpu_readl(g, sim_base + sim_config_r());
	if (sim_config_mode_v(cfg) == sim_config_mode_enabled_v())
		is_simulation = true;

	return is_simulation;
}

int nvgpu_pci_init_sim_support(struct gk20a *g)
{
	int err = 0;
	u64 phys;
	struct sim_gk20a_linux *sim_linux;
	bool is_simulation;
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	/* initialize sim aperture */
	is_simulation = _nvgpu_pci_is_simulation(g, sim_r());
	__nvgpu_set_enabled(g, NVGPU_IS_FMODEL, is_simulation);

	if (!is_simulation)
		return 0;

	sim_linux = nvgpu_kzalloc(g, sizeof(*sim_linux));
	if (!sim_linux)
		goto fail;

	g->sim = &sim_linux->sim;
	sim_linux->regs = l->regs + sim_r();

	/* allocate sim event/msg buffers */
	err = alloc_and_kmap_iopage(g, &sim_linux->send_bfr.kvaddr,
				    &sim_linux->send_bfr.phys,
				    &sim_linux->send_bfr.page);

	err = err || alloc_and_kmap_iopage(g, &sim_linux->recv_bfr.kvaddr,
					   &sim_linux->recv_bfr.phys,
					   &sim_linux->recv_bfr.page);

	err = err || alloc_and_kmap_iopage(g, &sim_linux->msg_bfr.kvaddr,
					   &sim_linux->msg_bfr.phys,
					   &sim_linux->msg_bfr.page);

	if (!(sim_linux->send_bfr.kvaddr && sim_linux->recv_bfr.kvaddr &&
	      sim_linux->msg_bfr.kvaddr)) {
		nvgpu_err(g, "couldn't allocate all sim buffers");
		goto fail;
	}

	/* mark send ring invalid */
	sim_writel(sim_linux, sim_send_ring_r(), sim_send_ring_status_invalid_f());

	/* read get pointer and make equal to put */
	g->sim->send_ring_put = sim_readl(sim_linux, sim_send_get_r());
	sim_writel(sim_linux, sim_send_put_r(), g->sim->send_ring_put);

	/* write send ring address and make it valid */
	phys = sim_linux->send_bfr.phys;
	sim_writel(sim_linux, sim_send_ring_hi_r(),
		   sim_send_ring_hi_addr_f(u64_hi32(phys)));
	sim_writel(sim_linux, sim_send_ring_r(),
		   sim_send_ring_status_valid_f() |
		   sim_send_ring_target_phys_pci_coherent_f() |
		   sim_send_ring_size_4kb_f() |
		   sim_send_ring_addr_lo_f(phys >> PAGE_SHIFT));

	/* repeat for recv ring (but swap put,get as roles are opposite) */
	sim_writel(sim_linux, sim_recv_ring_r(), sim_recv_ring_status_invalid_f());

	/* read put pointer and make equal to get */
	g->sim->recv_ring_get = sim_readl(sim_linux, sim_recv_put_r());
	sim_writel(sim_linux, sim_recv_get_r(), g->sim->recv_ring_get);

	/* write send ring address and make it valid */
	phys = sim_linux->recv_bfr.phys;
	sim_writel(sim_linux, sim_recv_ring_hi_r(),
		   sim_recv_ring_hi_addr_f(u64_hi32(phys)));
	sim_writel(sim_linux, sim_recv_ring_r(),
		   sim_recv_ring_status_valid_f() |
		   sim_recv_ring_target_phys_pci_coherent_f() |
		   sim_recv_ring_size_4kb_f() |
		   sim_recv_ring_addr_lo_f(phys >> PAGE_SHIFT));

	g->sim->remove_support = gk20a_remove_sim_support;
	g->sim->esc_readl = gk20a_sim_esc_readl;
	return 0;

 fail:
	gk20a_free_sim_support(g);
	return err;
}
