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
 */


#define  OFF_ERRLOGGER_0_ID_COREID_0            0x00000000
#define  OFF_ERRLOGGER_0_ID_REVISIONID_0        0x00000004
#define  OFF_ERRLOGGER_0_FAULTEN_0              0x00000008
#define  OFF_ERRLOGGER_0_ERRVLD_0               0x0000000c
#define  OFF_ERRLOGGER_0_ERRCLR_0               0x00000010
#define  OFF_ERRLOGGER_0_ERRLOG0_0              0x00000014
#define  OFF_ERRLOGGER_0_ERRLOG1_0              0x00000018
#define  OFF_ERRLOGGER_0_RESERVED_00_0          0x0000001c
#define  OFF_ERRLOGGER_0_ERRLOG3_0              0x00000020
#define  OFF_ERRLOGGER_0_ERRLOG4_0              0x00000024
#define  OFF_ERRLOGGER_0_ERRLOG5_0              0x00000028
#define  OFF_ERRLOGGER_0_STALLEN_0              0x00000038

#define  OFF_ERRLOGGER_1_ID_COREID_0            0x00000080
#define  OFF_ERRLOGGER_1_ID_REVISIONID_0        0x00000084
#define  OFF_ERRLOGGER_1_FAULTEN_0              0x00000088
#define  OFF_ERRLOGGER_1_ERRVLD_0               0x0000008c
#define  OFF_ERRLOGGER_1_ERRCLR_0               0x00000090
#define  OFF_ERRLOGGER_1_ERRLOG0_0              0x00000094
#define  OFF_ERRLOGGER_1_ERRLOG1_0              0x00000098
#define  OFF_ERRLOGGER_1_RESERVED_00_0          0x0000009c
#define  OFF_ERRLOGGER_1_ERRLOG3_0              0x000000A0
#define  OFF_ERRLOGGER_1_ERRLOG4_0              0x000000A4
#define  OFF_ERRLOGGER_1_ERRLOG5_0              0x000000A8
#define  OFF_ERRLOGGER_1_STALLEN_0              0x000000b8

#define  OFF_ERRLOGGER_2_ID_COREID_0            0x00000100
#define  OFF_ERRLOGGER_2_ID_REVISIONID_0        0x00000104
#define  OFF_ERRLOGGER_2_FAULTEN_0              0x00000108
#define  OFF_ERRLOGGER_2_ERRVLD_0               0x0000010c
#define  OFF_ERRLOGGER_2_ERRCLR_0               0x00000110
#define  OFF_ERRLOGGER_2_ERRLOG0_0              0x00000114
#define  OFF_ERRLOGGER_2_ERRLOG1_0              0x00000118
#define  OFF_ERRLOGGER_2_RESERVED_00_0          0x0000011c
#define  OFF_ERRLOGGER_2_ERRLOG3_0              0x00000120
#define  OFF_ERRLOGGER_2_ERRLOG4_0              0x00000124
#define  OFF_ERRLOGGER_2_ERRLOG5_0              0x00000128
#define  OFF_ERRLOGGER_2_STALLEN_0              0x00000138

#define DMAAPB_X_RAW_INTERRUPT_STATUS	0x2ec

#define CBB_BIT(_bit_) (1ULL << (_bit_))
#define CBB_MASK(_msb_, _lsb_) \
	((CBB_BIT(_msb_+1) - 1) & ~(CBB_BIT(_lsb_) - 1))
#define CBB_EXTRACT(_x_, _msb_, _lsb_)  \
	((_x_ & CBB_MASK(_msb_, _lsb_)) >> _lsb_)


#define get_cbb_errlog0_trans_opc(_x_)		CBB_EXTRACT(_x_, 4, 1)
#define get_cbb_errlog0_code(_x_)		CBB_EXTRACT(_x_, 10, 8)
#define get_cbb_errlog0_src(_x_)		CBB_EXTRACT(_x_, 27, 16)

#define get_cbb_errlog5_axi_id(_x_)		CBB_EXTRACT(_x_, 30, 23)
#define get_cbb_errlog5_mstr_id(_x_)		CBB_EXTRACT(_x_, 22, 19)
#define get_cbb_errlog5_vqc(_x_)		CBB_EXTRACT(_x_, 18, 17)
#define get_cbb_errlog5_grpsec(_x_)		CBB_EXTRACT(_x_, 16, 10)
#define get_cbb_errlog5_falconsec(_x_)		CBB_EXTRACT(_x_, 9, 8)
#define get_cbb_errlog5_axprot(_x_)		CBB_EXTRACT(_x_, 7, 5)
#define get_cbb_errlog5_non_modify(_x_)		CBB_EXTRACT(_x_, 4, 4)
#define get_cbb_errlog5_axcache(_x_)		CBB_EXTRACT(_x_, 3, 0)

#define get_cbb_routeid_initflow(_x_, _msb_, _lsb_) \
				CBB_EXTRACT(_x_, _msb_, _lsb_)
#define get_cbb_routeid_targflow(_x_, _msb_, _lsb_) \
				CBB_EXTRACT(_x_, _msb_, _lsb_)
#define get_cbb_routeid_targsubrange(_x_, _msb_, _lsb_) \
				CBB_EXTRACT(_x_, _msb_, _lsb_)
#define get_cbb_routeid_seqid(_x_, _msb_, _lsb_) \
				CBB_EXTRACT(_x_, _msb_, _lsb_)

struct tegra_noc_errors {
	char *errcode;
	char *src;
	char *type;
};

struct tegra_noc_packet_header {
	bool    lock;	// [0]
	u8      opc;	// [4:1]
	u8      errcode;// [10:8]= RD, RDW, RDL, RDX, WR, WRW, WRC, PRE, URG
	u16     len1;	// [27:16]
	bool    format;	// [31]  = 1 -> FlexNoC versions 2.7 & above
};

struct tegra_lookup_noc_aperture {
	u8      initflow;
	u8      targflow;
	u8      targ_subrange;
	u8	init_mapping;
	u32	init_localaddress;
	u8	targ_mapping;
	u32	targ_localaddress;
	u16	seqid;
};

struct tegra_cbb_errlog_record {
	struct list_head node;
	struct serr_hook *callback;
	char            *name;
	phys_addr_t     start;
	void            __iomem *vaddr;
	int		noc_secure_irq;
	int		noc_nonsecure_irq;
	u32             errlog0;
	u32             errlog1;
	u32             errlog2;
	u32             errlog3;
	u32             errlog4;
	u32             errlog5;
	u32             errlog6;	//RESERVED
	u32             errlog7;	//RESERVED
	u32             errlog8;	//RESERVED
	unsigned int    (*errvld)(void __iomem *addr);
	void            (*errclr)(void __iomem *addr);
	void            (*faulten)(void __iomem *addr);
	void            (*stallen)(void __iomem *addr);
	void		(*tegra_noc_parse_routeid)
				(struct tegra_lookup_noc_aperture *, u64);
	struct		tegra_lookup_noc_aperture *noc_aperture;
	int             max_noc_aperture;
	char		**tegra_noc_routeid_initflow;
	char		**tegra_noc_routeid_targflow;
	char		**tegra_cbb_master_id;
	bool		is_ax2apb_bridge_connected;
	u64		*axi2abp_bases;
	int		apb_bridge_cnt;
};

struct tegra_cbb_noc_data {
	char            *name;
	unsigned int    (*errvld)(void __iomem *addr);
	void            (*errclr)(void __iomem *addr);
	void            (*faulten)(void __iomem *addr);
	void            (*stallen)(void __iomem *addr);
	void		(*tegra_noc_parse_routeid)
				(struct tegra_lookup_noc_aperture *, u64);
	struct		tegra_lookup_noc_aperture *noc_aperture;
	int             max_error;
	int             max_noc_aperture;
	char		**tegra_noc_routeid_initflow;
	char		**tegra_noc_routeid_targflow;
	char		**tegra_cbb_master_id;
	bool		is_ax2apb_bridge_connected;
};

