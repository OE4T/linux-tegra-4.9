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
 */

#include <linux/pci.h>

#include <nvgpu/bios.h>
#include <nvgpu/kmem.h>
#include <nvgpu/nvgpu_common.h>
#include <nvgpu/timers.h>
#include <nvgpu/firmware.h>
#include <nvgpu/falcon.h>
#include <nvgpu/enabled.h>

#include "gk20a/gk20a.h"
#include "gm20b/fifo_gm20b.h"
#include "bios_gm206.h"
#include "gp106/mclk_gp106.h"
#ifdef CONFIG_DEBUG_FS
#include "common/linux/os_linux.h"
#endif

#include <nvgpu/hw/gm206/hw_pwr_gm206.h>
#include <nvgpu/hw/gm206/hw_mc_gm206.h>
#include <nvgpu/hw/gm206/hw_top_gm206.h>

#define PMU_BOOT_TIMEOUT_DEFAULT	100 /* usec */
#define PMU_BOOT_TIMEOUT_MAX		2000000 /* usec */
#define BIOS_OVERLAY_NAME "bios-%04x.rom"
#define BIOS_OVERLAY_NAME_FORMATTED "bios-xxxx.rom"
#define ROM_FILE_PAYLOAD_OFFSET 0xa00
#define BIOS_SIZE 0x40000

static void upload_code(struct gk20a *g, u32 dst,
			u8 *src, u32 size, u8 port, bool sec)
{
	nvgpu_flcn_copy_to_imem(g->pmu.flcn, dst, src, size, port, sec,
		dst >> 8);
}

static void upload_data(struct gk20a *g, u32 dst, u8 *src, u32 size, u8 port)
{
	u32 i, words;
	u32 *src_u32 = (u32 *)src;
	u32 blk;

	gk20a_dbg_info("upload %d bytes to %x", size, dst);

	words = DIV_ROUND_UP(size, 4);

	blk = dst >> 8;

	gk20a_dbg_info("upload %d words to %x blk %d",
			words, dst, blk);
	gk20a_writel(g, pwr_falcon_dmemc_r(port),
		pwr_falcon_dmemc_offs_f(dst >> 2) |
		pwr_falcon_dmemc_blk_f(blk) |
		pwr_falcon_dmemc_aincw_f(1));

	for (i = 0; i < words; i++)
		gk20a_writel(g, pwr_falcon_dmemd_r(port), src_u32[i]);
}

static int gm206_bios_devinit(struct gk20a *g)
{
	int err = 0;
	int devinit_completed;
	struct nvgpu_timeout timeout;

	gk20a_dbg_fn("");

	if (nvgpu_flcn_reset(g->pmu.flcn)) {
		err = -ETIMEDOUT;
		goto out;
	}

	upload_code(g, g->bios.devinit.bootloader_phys_base,
			g->bios.devinit.bootloader,
			g->bios.devinit.bootloader_size,
			0, 0);
	upload_code(g, g->bios.devinit.phys_base,
			g->bios.devinit.ucode,
			g->bios.devinit.size,
			0, 1);
	upload_data(g, g->bios.devinit.dmem_phys_base,
			g->bios.devinit.dmem,
			g->bios.devinit.dmem_size,
			0);
	upload_data(g, g->bios.devinit_tables_phys_base,
			g->bios.devinit_tables,
			g->bios.devinit_tables_size,
			0);
	upload_data(g, g->bios.devinit_script_phys_base,
			g->bios.bootscripts,
			g->bios.bootscripts_size,
			0);

	nvgpu_flcn_bootstrap(g->pmu.flcn, g->bios.devinit.code_entry_point);

	nvgpu_timeout_init(g, &timeout,
			   PMU_BOOT_TIMEOUT_MAX /
				PMU_BOOT_TIMEOUT_DEFAULT,
			   NVGPU_TIMER_RETRY_TIMER);
	do {
		devinit_completed = pwr_falcon_cpuctl_halt_intr_v(
				gk20a_readl(g, pwr_falcon_cpuctl_r())) &&
				    top_scratch1_devinit_completed_v(
				gk20a_readl(g, top_scratch1_r()));
		nvgpu_udelay(PMU_BOOT_TIMEOUT_DEFAULT);
	} while (!devinit_completed && !nvgpu_timeout_expired(&timeout));

	if (nvgpu_timeout_peek_expired(&timeout))
		err = -ETIMEDOUT;

	nvgpu_flcn_clear_halt_intr_status(g->pmu.flcn,
		gk20a_get_gr_idle_timeout(g));

out:
	gk20a_dbg_fn("done");
	return err;
}

static int gm206_bios_preos(struct gk20a *g)
{
	int err = 0;

	gk20a_dbg_fn("");

	if (nvgpu_flcn_reset(g->pmu.flcn)) {
		err = -ETIMEDOUT;
		goto out;
	}

	upload_code(g, g->bios.preos.bootloader_phys_base,
			g->bios.preos.bootloader,
			g->bios.preos.bootloader_size,
			0, 0);
	upload_code(g, g->bios.preos.phys_base,
			g->bios.preos.ucode,
			g->bios.preos.size,
			0, 1);
	upload_data(g, g->bios.preos.dmem_phys_base,
			g->bios.preos.dmem,
			g->bios.preos.dmem_size,
			0);

	nvgpu_flcn_bootstrap(g->pmu.flcn, g->bios.preos.code_entry_point);

	if (nvgpu_flcn_wait_for_halt(g->pmu.flcn,
		PMU_BOOT_TIMEOUT_MAX / 1000)) {
		err = -ETIMEDOUT;
		goto out;
	}

	nvgpu_flcn_clear_halt_intr_status(g->pmu.flcn,
			gk20a_get_gr_idle_timeout(g));

out:
	gk20a_dbg_fn("done");
	return err;
}

int gm206_bios_init(struct gk20a *g)
{
	unsigned int i;
#ifdef CONFIG_DEBUG_FS
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct dentry *d;
#endif
	struct nvgpu_firmware *bios_fw;
	int err;
	struct pci_dev *pdev = to_pci_dev(dev_from_gk20a(g));
	char rom_name[sizeof(BIOS_OVERLAY_NAME_FORMATTED)];

	gk20a_dbg_fn("");

	snprintf(rom_name, sizeof(rom_name), BIOS_OVERLAY_NAME, pdev->device);
	gk20a_dbg_info("checking for VBIOS overlay %s", rom_name);
	bios_fw = nvgpu_request_firmware(g, rom_name,
			NVGPU_REQUEST_FIRMWARE_NO_WARN |
			NVGPU_REQUEST_FIRMWARE_NO_SOC);
	if (bios_fw) {
		gk20a_dbg_info("using VBIOS overlay");
		g->bios.size = bios_fw->size - ROM_FILE_PAYLOAD_OFFSET;
		g->bios.data = vmalloc(g->bios.size);
		if (!g->bios.data) {
			err = -ENOMEM;
			goto free_firmware;
		}

		memcpy(g->bios.data, &bios_fw->data[ROM_FILE_PAYLOAD_OFFSET],
		       g->bios.size);

		nvgpu_release_firmware(g, bios_fw);
	} else {
		gk20a_dbg_info("reading bios from EEPROM");
		g->bios.size = BIOS_SIZE;
		g->bios.data = nvgpu_vmalloc(g, BIOS_SIZE);
		if (!g->bios.data)
			return -ENOMEM;
		g->ops.xve.disable_shadow_rom(g);
		for (i = 0; i < g->bios.size/4; i++) {
			u32 val = be32_to_cpu(gk20a_readl(g, 0x300000 + i*4));

			g->bios.data[(i*4)] = (val >> 24) & 0xff;
			g->bios.data[(i*4)+1] = (val >> 16) & 0xff;
			g->bios.data[(i*4)+2] = (val >> 8) & 0xff;
			g->bios.data[(i*4)+3] = val & 0xff;
		}
		g->ops.xve.enable_shadow_rom(g);
	}

	err = nvgpu_bios_parse_rom(g);
	if (err)
		return err;

	if (g->gpu_characteristics.vbios_version < g->vbios_min_version) {
		nvgpu_err(g, "unsupported VBIOS version %08x",
				g->gpu_characteristics.vbios_version);
		return -EINVAL;
	}

	/* WAR for HW2.5 RevA (INA3221 is missing) */
	if ((g->pci_vendor_id == PCI_VENDOR_ID_NVIDIA) &&
		(g->pci_device_id == 0x1c75) &&
		(g->gpu_characteristics.vbios_version == 0x86065300)) {
			g->power_sensor_missing = true;
	}

#ifdef CONFIG_DEBUG_FS
	g->bios_blob.data = g->bios.data;
	g->bios_blob.size = g->bios.size;

	d = debugfs_create_blob("bios", S_IRUGO, l->debugfs,
			&g->bios_blob);
	if (!d)
		nvgpu_err(g, "No debugfs?");
#endif

	gk20a_dbg_fn("done");

	err = gm206_bios_devinit(g);
	if (err) {
		nvgpu_err(g, "devinit failed");
		return err;
	}

	if (nvgpu_is_enabled(g, NVGPU_PMU_RUN_PREOS)) {
		err = gm206_bios_preos(g);
		if (err) {
			nvgpu_err(g, "pre-os failed");
			return err;
		}
	}

	return 0;

free_firmware:
	nvgpu_release_firmware(g, bios_fw);
	return err;
}
