/*
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

#ifndef _PVA_UCODE_HEADER_TYPES_H_
#define _PVA_UCODE_HEADER_TYPES_H_

/*
 * This file is distinct from the other uCode header file because it
 * defines constants/values that are used by the linker scripts and therefor
 * cannot have C structures (only pre-processor directives).
 */

/*
 * Define the length of a section header to be defined independently than
 * the C structure (it will be larger).  Picking a value that is easy to
 * compute.
 */
#define PVA_UCODE_SEG_HDR_LENGTH	128

#define PVA_UCODE_SEG_NONE	0	/* not a segment */
#define PVA_UCODE_SEG_EVP	1	/* EVP information */
#define PVA_UCODE_SEG_R5	2	/* R5 code/data */
#define PVA_UCODE_SEG_VPU_CODE	3	/* VPU code */
#define PVA_UCODE_SEG_VPU_DATA	4	/* VPU data */
#define PVA_UCODE_SEG_R5_APPL	5	/* R5 application code/data */
#define PVA_UCODE_SEG_IN_PARMS	6	/* application input parameter data */
#define PVA_UCODE_SEG_OUT_PARMS	7	/* application output parameter data */
#define PVA_UCODE_SEG_NEXT	8	/* must be last */

#define PVA_HDR_MAGIC		0x31415650	/* PVA1 in little endian */
#define PVA_HDR_VERSION		0x00010000	/* version 1.0 of the header */
#endif
