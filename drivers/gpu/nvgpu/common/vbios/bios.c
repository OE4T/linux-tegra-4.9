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

#include <nvgpu/bios.h>
#include <nvgpu/hw/gp106/hw_gc6_gp106.h>

#include "gk20a/gk20a.h"

#define BIT_HEADER_ID 0xb8ff
#define BIT_HEADER_SIGNATURE 0x00544942
#define PCI_EXP_ROM_SIG 0xaa55
#define PCI_EXP_ROM_SIG_NV 0x4e56

#define INIT_DONE 0x71
#define INIT_RESUME 0x72
#define INIT_CONDITION 0x75
#define INIT_XMEMSEL_ZM_NV_REG_ARRAY 0x8f

struct condition_entry {
	u32 cond_addr;
	u32 cond_mask;
	u32 cond_compare;
} __packed;

static u16 nvgpu_bios_rdu16(struct gk20a *g, int offset)
{
	u16 val = (g->bios.data[offset+1] << 8) + g->bios.data[offset];
	return val;
}

static u32 nvgpu_bios_rdu32(struct gk20a *g, int offset)
{
	u32 val = (g->bios.data[offset+3] << 24) +
		  (g->bios.data[offset+2] << 16) +
		  (g->bios.data[offset+1] << 8) +
		  g->bios.data[offset];
	return val;
}

struct bit {
	u16 id;
	u32 signature;
	u16 bcd_version;
	u8 header_size;
	u8 token_size;
	u8 token_entries;
	u8 header_checksum;
} __packed;

#define TOKEN_ID_BIOSDATA 0x42
#define TOKEN_ID_NVINIT_PTRS 0x49
#define TOKEN_ID_FALCON_DATA 0x70
#define TOKEN_ID_PERF_PTRS 0x50
#define TOKEN_ID_CLOCK_PTRS 0x43
#define TOKEN_ID_VIRT_PTRS 0x56
#define TOKEN_ID_MEMORY_PTRS 0x4D

#define MEMORY_PTRS_V1 1
#define MEMORY_PTRS_V2 2

struct memory_ptrs_v1 {
	u8 rsvd0[2];
	u8 mem_strap_data_count;
	u16 mem_strap_xlat_tbl_ptr;
	u8 rsvd1[8];
} __packed;

struct memory_ptrs_v2 {
	u8 mem_strap_data_count;
	u16 mem_strap_xlat_tbl_ptr;
	u8 rsvd[14];
} __packed;

struct biosdata {
	u32 version;
	u8 oem_version;
	u8 checksum;
	u16 int15callbackspost;
	u16 int16callbackssystem;
	u16 boardid;
	u16 framecount;
	u8 biosmoddate[8];
} __packed;

struct nvinit_ptrs {
	u16 initscript_table_ptr;
	u16 macro_index_table_ptr;
	u16 macro_table_ptr;
	u16 condition_table_ptr;
	u16 io_condition_table_ptr;
	u16 io_flag_condition_table_ptr;
	u16 init_function_table_ptr;
	u16 vbios_private_table_ptr;
	u16 data_arrays_table_ptr;
	u16 pcie_settings_script_ptr;
	u16 devinit_tables_ptr;
	u16 devinit_tables_size;
	u16 bootscripts_ptr;
	u16 bootscripts_size;
	u16 nvlink_config_data_ptr;
} __packed;

struct falcon_data_v2 {
	u32 falcon_ucode_table_ptr;
} __packed;

struct falcon_ucode_table_hdr_v1 {
	u8 version;
	u8 header_size;
	u8 entry_size;
	u8 entry_count;
	u8 desc_version;
	u8 desc_size;
} __packed;

struct falcon_ucode_table_entry_v1 {
	u8 application_id;
	u8 target_id;
	u32 desc_ptr;
} __packed;

#define TARGET_ID_PMU 0x01
#define APPLICATION_ID_DEVINIT 0x04
#define APPLICATION_ID_PRE_OS 0x01

#define FALCON_UCODE_FLAGS_VERSION_AVAILABLE 0x1
#define FALCON_UCODE_IS_VERSION_AVAILABLE(hdr)           \
	((hdr.v2.v_desc & FALCON_UCODE_FLAGS_VERSION_AVAILABLE) == \
	FALCON_UCODE_FLAGS_VERSION_AVAILABLE)

/*
 * version is embedded in bits 8:15 of the header on version 2+
 * and the header length in bits 16:31
 */

#define FALCON_UCODE_GET_VERSION(hdr) \
	((hdr.v2.v_desc >> 8) & 0xff)

#define FALCON_UCODE_GET_DESC_SIZE(hdr) \
	((hdr.v2.v_desc >> 16) & 0xffff)

struct falcon_ucode_desc_v1 {
	union {
		u32 v_desc;
		u32 stored_size;
	} hdr_size;
	u32 uncompressed_size;
	u32 virtual_entry;
	u32 interface_offset;
	u32 imem_phys_base;
	u32 imem_load_size;
	u32 imem_virt_base;
	u32 imem_sec_base;
	u32 imem_sec_size;
	u32 dmem_offset;
	u32 dmem_phys_base;
	u32 dmem_load_size;
} __packed;

struct falcon_ucode_desc_v2 {
	u32 v_desc;
	u32 stored_size;
	u32 uncompressed_size;
	u32 virtual_entry;
	u32 interface_offset;
	u32 imem_phys_base;
	u32 imem_load_size;
	u32 imem_virt_base;
	u32 imem_sec_base;
	u32 imem_sec_size;
	u32 dmem_offset;
	u32 dmem_phys_base;
	u32 dmem_load_size;
	u32 alt_imem_load_size;
	u32 alt_dmem_load_size;
} __packed;

union falcon_ucode_desc {
	struct falcon_ucode_desc_v1 v1;
	struct falcon_ucode_desc_v2 v2;
};

struct application_interface_table_hdr_v1 {
	u8 version;
	u8 header_size;
	u8 entry_size;
	u8 entry_count;
} __packed;

struct application_interface_entry_v1 {
	u32 id;
	u32 dmem_offset;
} __packed;

#define APPINFO_ID_DEVINIT 0x01

struct devinit_engine_interface {
	u16 version;
	u16 size;
	u16 application_version;
	u16 application_features;
	u32 tables_phys_base;
	u32 tables_virt_base;
	u32 script_phys_base;
	u32 script_virt_base;
	u32 script_virt_entry;
	u16 script_size;
	u8 memory_strap_count;
	u8 reserved;
	u32 memory_information_table_virt_base;
	u32 empty_script_virt_base;
	u32 cond_table_virt_base;
	u32 io_cond_table_virt_base;
	u32 data_arrays_table_virt_base;
	u32 gpio_assignment_table_virt_base;
} __packed;

struct pci_exp_rom {
	u16 sig;
	u8 reserved[0x16];
	u16 pci_data_struct_ptr;
	u32 size_of_block;
} __packed;

struct pci_data_struct {
	u32 sig;
	u16 vendor_id;
	u16 device_id;
	u16 device_list_ptr;
	u16 pci_data_struct_len;
	u8 pci_data_struct_rev;
	u8 class_code[3];
	u16 image_len;
	u16 vendor_rom_rev;
	u8 code_type;
	u8 last_image;
	u16 max_runtime_image_len;
} __packed;

struct pci_ext_data_struct {
	u32 sig;
	u16 nv_pci_data_ext_rev;
	u16 nv_pci_data_ext_len;
	u16 sub_image_len;
	u8 priv_last_image;
	u8 flags;
} __packed;

static void nvgpu_bios_parse_bit(struct gk20a *g, int offset);

int nvgpu_bios_parse_rom(struct gk20a *g)
{
	int offset = 0;
	int last = 0;
	bool found = false;
	unsigned int i;

	while (!last) {
		struct pci_exp_rom *pci_rom;
		struct pci_data_struct *pci_data;
		struct pci_ext_data_struct *pci_ext_data;

		pci_rom = (struct pci_exp_rom *)&g->bios.data[offset];
		gk20a_dbg_fn("pci rom sig %04x ptr %04x block %x",
				pci_rom->sig, pci_rom->pci_data_struct_ptr,
				pci_rom->size_of_block);

		if (pci_rom->sig != PCI_EXP_ROM_SIG &&
		    pci_rom->sig != PCI_EXP_ROM_SIG_NV) {
			nvgpu_err(g, "invalid VBIOS signature");
			return -EINVAL;
		}

		pci_data =
			(struct pci_data_struct *)
			&g->bios.data[offset + pci_rom->pci_data_struct_ptr];
		gk20a_dbg_fn("pci data sig %08x len %d image len %x type %x last %d max %08x",
				pci_data->sig, pci_data->pci_data_struct_len,
				pci_data->image_len, pci_data->code_type,
				pci_data->last_image,
				pci_data->max_runtime_image_len);

		if (pci_data->code_type == 0x3) {
			pci_ext_data = (struct pci_ext_data_struct *)
				&g->bios.data[(offset +
					       pci_rom->pci_data_struct_ptr +
					       pci_data->pci_data_struct_len +
					       0xf)
					      & ~0xf];
			gk20a_dbg_fn("pci ext data sig %08x rev %x len %x sub_image_len %x priv_last %d flags %x",
					pci_ext_data->sig,
					pci_ext_data->nv_pci_data_ext_rev,
					pci_ext_data->nv_pci_data_ext_len,
					pci_ext_data->sub_image_len,
					pci_ext_data->priv_last_image,
					pci_ext_data->flags);

			gk20a_dbg_fn("expansion rom offset %x",
					pci_data->image_len * 512);
			g->bios.expansion_rom_offset =
				pci_data->image_len * 512;
			offset += pci_ext_data->sub_image_len * 512;
			last = pci_ext_data->priv_last_image;
		} else {
			offset += pci_data->image_len * 512;
			last = pci_data->last_image;
		}
	}

	gk20a_dbg_info("read bios");
	for (i = 0; i < g->bios.size - 6; i++) {
		if (nvgpu_bios_rdu16(g, i) == BIT_HEADER_ID &&
		    nvgpu_bios_rdu32(g, i+2) ==  BIT_HEADER_SIGNATURE) {
			nvgpu_bios_parse_bit(g, i);
			found = true;
		}
	}

	if (!found)
		return -EINVAL;
	else
		return 0;
}

static void nvgpu_bios_parse_biosdata(struct gk20a *g, int offset)
{
	struct biosdata biosdata;

	memcpy(&biosdata, &g->bios.data[offset], sizeof(biosdata));
	gk20a_dbg_fn("bios version %x, oem version %x",
			biosdata.version,
			biosdata.oem_version);

	g->gpu_characteristics.vbios_version = biosdata.version;
	g->gpu_characteristics.vbios_oem_version = biosdata.oem_version;
}

static void nvgpu_bios_parse_nvinit_ptrs(struct gk20a *g, int offset)
{
	struct nvinit_ptrs nvinit_ptrs;

	memcpy(&nvinit_ptrs, &g->bios.data[offset], sizeof(nvinit_ptrs));
	gk20a_dbg_fn("devinit ptr %x size %d", nvinit_ptrs.devinit_tables_ptr,
			nvinit_ptrs.devinit_tables_size);
	gk20a_dbg_fn("bootscripts ptr %x size %d", nvinit_ptrs.bootscripts_ptr,
			nvinit_ptrs.bootscripts_size);

	g->bios.devinit_tables = &g->bios.data[nvinit_ptrs.devinit_tables_ptr];
	g->bios.devinit_tables_size = nvinit_ptrs.devinit_tables_size;
	g->bios.bootscripts = &g->bios.data[nvinit_ptrs.bootscripts_ptr];
	g->bios.bootscripts_size = nvinit_ptrs.bootscripts_size;
	g->bios.condition_table_ptr = nvinit_ptrs.condition_table_ptr;
}

static void nvgpu_bios_parse_memory_ptrs(struct gk20a *g, int offset, u8 version)
{
	struct memory_ptrs_v1 v1;
	struct memory_ptrs_v2 v2;

	switch (version) {
	case MEMORY_PTRS_V1:
		memcpy(&v1, &g->bios.data[offset], sizeof(v1));
		g->bios.mem_strap_data_count = v1.mem_strap_data_count;
		g->bios.mem_strap_xlat_tbl_ptr = v1.mem_strap_xlat_tbl_ptr;
		return;
	case MEMORY_PTRS_V2:
		memcpy(&v2, &g->bios.data[offset], sizeof(v2));
		g->bios.mem_strap_data_count = v2.mem_strap_data_count;
		g->bios.mem_strap_xlat_tbl_ptr = v2.mem_strap_xlat_tbl_ptr;
		return;
	default:
		nvgpu_err(g, "unknown vbios memory table version %x", version);
		return;
	}
}

static void nvgpu_bios_parse_devinit_appinfo(struct gk20a *g, int dmem_offset)
{
	struct devinit_engine_interface interface;

	memcpy(&interface, &g->bios.devinit.dmem[dmem_offset], sizeof(interface));
	gk20a_dbg_fn("devinit version %x tables phys %x script phys %x size %d",
			interface.version,
			interface.tables_phys_base,
			interface.script_phys_base,
			interface.script_size);

	if (interface.version != 1)
		return;
	g->bios.devinit_tables_phys_base = interface.tables_phys_base;
	g->bios.devinit_script_phys_base = interface.script_phys_base;
}

static int nvgpu_bios_parse_appinfo_table(struct gk20a *g, int offset)
{
	struct application_interface_table_hdr_v1 hdr;
	int i;

	memcpy(&hdr, &g->bios.data[offset], sizeof(hdr));

	gk20a_dbg_fn("appInfoHdr ver %d size %d entrySize %d entryCount %d",
			hdr.version, hdr.header_size,
			hdr.entry_size, hdr.entry_count);

	if (hdr.version != 1)
		return 0;

	offset += sizeof(hdr);
	for (i = 0; i < hdr.entry_count; i++) {
		struct application_interface_entry_v1 entry;

		memcpy(&entry, &g->bios.data[offset], sizeof(entry));

		gk20a_dbg_fn("appInfo id %d dmem_offset %d",
				entry.id, entry.dmem_offset);

		if (entry.id == APPINFO_ID_DEVINIT)
			nvgpu_bios_parse_devinit_appinfo(g, entry.dmem_offset);

		offset += hdr.entry_size;
	}

	return 0;
}

static int nvgpu_bios_parse_falcon_ucode_desc(struct gk20a *g,
		struct nvgpu_bios_ucode *ucode, int offset)
{
	union falcon_ucode_desc udesc;
	struct falcon_ucode_desc_v2 desc;
	u8 version;
	u16 desc_size;

	memcpy(&udesc, &g->bios.data[offset], sizeof(udesc));

	if (FALCON_UCODE_IS_VERSION_AVAILABLE(udesc)) {
		version = FALCON_UCODE_GET_VERSION(udesc);
		desc_size = FALCON_UCODE_GET_DESC_SIZE(udesc);
	} else {
		version = 1;
		desc_size = sizeof(udesc.v1);
	}

	switch (version) {
	case 1:
		desc.stored_size = udesc.v1.hdr_size.stored_size;
		desc.uncompressed_size = udesc.v1.uncompressed_size;
		desc.virtual_entry = udesc.v1.virtual_entry;
		desc.interface_offset = udesc.v1.interface_offset;
		desc.imem_phys_base = udesc.v1.imem_phys_base;
		desc.imem_load_size = udesc.v1.imem_load_size;
		desc.imem_virt_base = udesc.v1.imem_virt_base;
		desc.imem_sec_base = udesc.v1.imem_sec_base;
		desc.imem_sec_size = udesc.v1.imem_sec_size;
		desc.dmem_offset = udesc.v1.dmem_offset;
		desc.dmem_phys_base = udesc.v1.dmem_phys_base;
		desc.dmem_load_size = udesc.v1.dmem_load_size;
		break;
	case 2:
		memcpy(&desc, &udesc, sizeof(udesc.v2));
		break;
	default:
		gk20a_dbg_info("invalid version");
		return -EINVAL;
	}

	gk20a_dbg_info("falcon ucode desc version %x len %x", version, desc_size);

	gk20a_dbg_info("falcon ucode desc stored size %x uncompressed size %x",
			desc.stored_size, desc.uncompressed_size);
	gk20a_dbg_info("falcon ucode desc virtualEntry %x, interfaceOffset %x",
			desc.virtual_entry, desc.interface_offset);
	gk20a_dbg_info("falcon ucode IMEM phys base %x, load size %x virt base %x sec base %x sec size %x",
			desc.imem_phys_base, desc.imem_load_size,
			desc.imem_virt_base, desc.imem_sec_base,
			desc.imem_sec_size);
	gk20a_dbg_info("falcon ucode DMEM offset %x phys base %x, load size %x",
			desc.dmem_offset, desc.dmem_phys_base,
			desc.dmem_load_size);

	if (desc.stored_size != desc.uncompressed_size) {
		gk20a_dbg_info("does not match");
		return -EINVAL;
	}

	ucode->code_entry_point = desc.virtual_entry;
	ucode->bootloader = &g->bios.data[offset] + desc_size;
	ucode->bootloader_phys_base = desc.imem_phys_base;
	ucode->bootloader_size = desc.imem_load_size - desc.imem_sec_size;
	ucode->ucode = ucode->bootloader + ucode->bootloader_size;
	ucode->phys_base = ucode->bootloader_phys_base + ucode->bootloader_size;
	ucode->size = desc.imem_sec_size;
	ucode->dmem = ucode->bootloader + desc.dmem_offset;
	ucode->dmem_phys_base = desc.dmem_phys_base;
	ucode->dmem_size = desc.dmem_load_size;

	return nvgpu_bios_parse_appinfo_table(g,
			offset + desc_size +
			desc.dmem_offset + desc.interface_offset);
}

static int nvgpu_bios_parse_falcon_ucode_table(struct gk20a *g, int offset)
{
	struct falcon_ucode_table_hdr_v1 hdr;
	int i;

	memcpy(&hdr, &g->bios.data[offset], sizeof(hdr));
	gk20a_dbg_fn("falcon ucode table ver %d size %d entrySize %d entryCount %d descVer %d descSize %d",
			hdr.version, hdr.header_size,
			hdr.entry_size, hdr.entry_count,
			hdr.desc_version, hdr.desc_size);

	if (hdr.version != 1)
		return -EINVAL;

	offset += hdr.header_size;

	for (i = 0; i < hdr.entry_count; i++) {
		struct falcon_ucode_table_entry_v1 entry;

		memcpy(&entry, &g->bios.data[offset], sizeof(entry));

		gk20a_dbg_fn("falcon ucode table entry appid %x targetId %x descPtr %x",
				entry.application_id, entry.target_id,
				entry.desc_ptr);

		if (entry.target_id == TARGET_ID_PMU &&
		    entry.application_id == APPLICATION_ID_DEVINIT) {
			int err;

			err = nvgpu_bios_parse_falcon_ucode_desc(g,
					&g->bios.devinit, entry.desc_ptr);
			if (err)
				err = nvgpu_bios_parse_falcon_ucode_desc(g,
					&g->bios.devinit,
					entry.desc_ptr +
					 g->bios.expansion_rom_offset);

			if (err)
				nvgpu_err(g,
					  "could not parse devinit ucode desc");
		} else if (entry.target_id == TARGET_ID_PMU &&
		    entry.application_id == APPLICATION_ID_PRE_OS) {
			int err;

			err = nvgpu_bios_parse_falcon_ucode_desc(g,
					&g->bios.preos, entry.desc_ptr);
			if (err)
				err = nvgpu_bios_parse_falcon_ucode_desc(g,
					&g->bios.preos,
					entry.desc_ptr +
					 g->bios.expansion_rom_offset);

			if (err)
				nvgpu_err(g,
					  "could not parse preos ucode desc");
		}

		offset += hdr.entry_size;
	}

	return 0;
}

static void nvgpu_bios_parse_falcon_data_v2(struct gk20a *g, int offset)
{
	struct falcon_data_v2 falcon_data;
	int err;

	memcpy(&falcon_data, &g->bios.data[offset], sizeof(falcon_data));
	gk20a_dbg_fn("falcon ucode table ptr %x",
			falcon_data.falcon_ucode_table_ptr);
	err = nvgpu_bios_parse_falcon_ucode_table(g,
			falcon_data.falcon_ucode_table_ptr);
	if (err)
		err = nvgpu_bios_parse_falcon_ucode_table(g,
				falcon_data.falcon_ucode_table_ptr +
			g->bios.expansion_rom_offset);

	if (err)
		nvgpu_err(g, "could not parse falcon ucode table");
}

void *nvgpu_bios_get_perf_table_ptrs(struct gk20a *g,
		struct bit_token *ptoken, u8 table_id)
{
	u32 perf_table_id_offset = 0;
	u8 *perf_table_ptr = NULL;
	u8 data_size = 4;

	if (ptoken != NULL) {

		if (ptoken->token_id == TOKEN_ID_VIRT_PTRS) {
			perf_table_id_offset = *((u16 *)&g->bios.data[
				ptoken->data_ptr +
				(table_id * PERF_PTRS_WIDTH_16)]);
			data_size = PERF_PTRS_WIDTH_16;
		} else {
			perf_table_id_offset = *((u32 *)&g->bios.data[
				ptoken->data_ptr +
				(table_id * PERF_PTRS_WIDTH)]);
			data_size = PERF_PTRS_WIDTH;
		}
	} else
		return (void *)perf_table_ptr;

	if (table_id < (ptoken->data_size/data_size)) {

		gk20a_dbg_info("Perf_Tbl_ID-offset 0x%x Tbl_ID_Ptr-offset- 0x%x",
					(ptoken->data_ptr +
					(table_id * data_size)),
					perf_table_id_offset);

		if (perf_table_id_offset != 0) {
			/* check is perf_table_id_offset is > 64k */
			if (perf_table_id_offset & ~0xFFFF)
				perf_table_ptr =
					&g->bios.data[g->bios.expansion_rom_offset +
						perf_table_id_offset];
			else
				perf_table_ptr =
					&g->bios.data[perf_table_id_offset];
		} else
			nvgpu_warn(g, "PERF TABLE ID %d is NULL",
					table_id);
	} else
		nvgpu_warn(g, "INVALID PERF TABLE ID - %d ", table_id);

	return (void *)perf_table_ptr;
}

static void nvgpu_bios_parse_bit(struct gk20a *g, int offset)
{
	struct bit bit;
	struct bit_token bit_token;
	int i;

	gk20a_dbg_fn("");
	memcpy(&bit, &g->bios.data[offset], sizeof(bit));

	gk20a_dbg_info("BIT header: %04x %08x", bit.id, bit.signature);
	gk20a_dbg_info("tokens: %d entries * %d bytes",
			bit.token_entries, bit.token_size);

	offset += bit.header_size;
	for (i = 0; i < bit.token_entries; i++) {
		memcpy(&bit_token, &g->bios.data[offset], sizeof(bit_token));

		gk20a_dbg_info("BIT token id %d ptr %d size %d ver %d",
				bit_token.token_id, bit_token.data_ptr,
				bit_token.data_size, bit_token.data_version);

		switch (bit_token.token_id) {
		case TOKEN_ID_BIOSDATA:
			nvgpu_bios_parse_biosdata(g, bit_token.data_ptr);
			break;
		case TOKEN_ID_NVINIT_PTRS:
			nvgpu_bios_parse_nvinit_ptrs(g, bit_token.data_ptr);
			break;
		case TOKEN_ID_FALCON_DATA:
			if (bit_token.data_version == 2)
				nvgpu_bios_parse_falcon_data_v2(g,
						bit_token.data_ptr);
			break;
		case TOKEN_ID_PERF_PTRS:
			g->bios.perf_token =
				(struct bit_token *)&g->bios.data[offset];
			break;
		case TOKEN_ID_CLOCK_PTRS:
			g->bios.clock_token =
				(struct bit_token *)&g->bios.data[offset];
			break;
		case TOKEN_ID_VIRT_PTRS:
			g->bios.virt_token =
				(struct bit_token *)&g->bios.data[offset];
			break;
		case TOKEN_ID_MEMORY_PTRS:
			nvgpu_bios_parse_memory_ptrs(g, bit_token.data_ptr,
				bit_token.data_version);
		default:
			break;
		}

		offset += bit.token_size;
	}
	gk20a_dbg_fn("done");
}

static u32 __nvgpu_bios_readbyte(struct gk20a *g, u32 offset)
{
	return (u32) g->bios.data[offset];
}

u8 nvgpu_bios_read_u8(struct gk20a *g, u32 offset)
{
	return (u8) __nvgpu_bios_readbyte(g, offset);
}

s8 nvgpu_bios_read_s8(struct gk20a *g, u32 offset)
{
	u32 val;
	val = __nvgpu_bios_readbyte(g, offset);
	val = val & 0x80 ? (val | ~0xff) : val;

	return (s8) val;
}

u16 nvgpu_bios_read_u16(struct gk20a *g, u32 offset)
{
	u16 val;

	val = __nvgpu_bios_readbyte(g, offset) |
		(__nvgpu_bios_readbyte(g, offset+1) << 8);

	return val;
}

u32 nvgpu_bios_read_u32(struct gk20a *g, u32 offset)
{
	u32 val;

	val = __nvgpu_bios_readbyte(g, offset) |
		(__nvgpu_bios_readbyte(g, offset+1) << 8) |
		(__nvgpu_bios_readbyte(g, offset+2) << 16) |
		(__nvgpu_bios_readbyte(g, offset+3) << 24);

	return val;
}

static void nvgpu_bios_init_xmemsel_zm_nv_reg_array(struct gk20a *g, bool *condition,
	u32 reg, u32 stride, u32 count, u32 data_table_offset)
{
	u8 i;
	u32 data, strap, index;

	if (*condition) {

		strap = gk20a_readl(g, gc6_sci_strap_r()) & 0xf;

		index = g->bios.mem_strap_xlat_tbl_ptr ?
			nvgpu_bios_read_u8(g, g->bios.mem_strap_xlat_tbl_ptr +
				strap) : strap;

		for (i = 0; i < count; i++) {
			data = nvgpu_bios_read_u32(g, data_table_offset + ((i *
				g->bios.mem_strap_data_count + index) *
				sizeof(u32)));
			gk20a_writel(g, reg, data);
			reg += stride;
		}
	}
}

static void gp106_init_condition(struct gk20a *g, bool *condition,
	u32 condition_id)
{
	struct condition_entry entry;

	entry.cond_addr = nvgpu_bios_read_u32(g, g->bios.condition_table_ptr +
		sizeof(entry)*condition_id);
	entry.cond_mask = nvgpu_bios_read_u32(g, g->bios.condition_table_ptr +
		sizeof(entry)*condition_id + 4);
	entry.cond_compare = nvgpu_bios_read_u32(g, g->bios.condition_table_ptr +
		sizeof(entry)*condition_id + 8);

	if ((gk20a_readl(g, entry.cond_addr) & entry.cond_mask)
		!= entry.cond_compare) {
		*condition = false;
	}
}

int nvgpu_bios_execute_script(struct gk20a *g, u32 offset)
{
	u8 opcode;
	u32 ip;
	u32 operand[8];
	bool condition, end;
	int status = 0;

	ip = offset;
	condition = true;
	end = false;

	while (!end) {

		opcode = nvgpu_bios_read_u8(g, ip++);

		switch (opcode) {

		case INIT_XMEMSEL_ZM_NV_REG_ARRAY:
			operand[0] = nvgpu_bios_read_u32(g, ip);
			operand[1] = nvgpu_bios_read_u8(g, ip+4);
			operand[2] = nvgpu_bios_read_u8(g, ip+5);
			ip += 6;

			nvgpu_bios_init_xmemsel_zm_nv_reg_array(g, &condition,
				operand[0], operand[1], operand[2], ip);
			ip += operand[2] * sizeof(u32) *
				g->bios.mem_strap_data_count;
			break;

		case INIT_CONDITION:
			operand[0] = nvgpu_bios_read_u8(g, ip);
			ip++;

			gp106_init_condition(g, &condition, operand[0]);
			break;

		case INIT_RESUME:
			condition = true;
			break;

		case INIT_DONE:
			end = true;
			break;

		default:
			nvgpu_err(g, "opcode: 0x%02x", opcode);
			end = true;
			status = -EINVAL;
			break;
		}
	}

	return status;
}
