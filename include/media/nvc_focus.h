/* Copyright (C) 2012 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef __NVC_FOCUS_H__
#define __NVC_FOCUS_H__

/* NVC_FOCUS_CAP_VER0: invalid */
/* NVC_FOCUS_CAP_VER1:
 * __u32 version
 * __u32 actuator_range
 * __u32 settle_time
 */
#define NVC_FOCUS_CAP_VER1		1
/* NVC_FOCUS_CAP_VER2 adds:
 * __u32 focus_macro;
 * __u32 focus_hyper;
 * __u32 focus_infinity;
 */
#define NVC_FOCUS_CAP_VER2		2
#define NVC_FOCUS_CAP_VER		2 /* latest version */

enum nvc_focus_sts {
	NVC_FOCUS_STS_UNKNOWN		= 1,
	NVC_FOCUS_STS_NO_DEVICE,
	NVC_FOCUS_STS_INITIALIZING,
	NVC_FOCUS_STS_INIT_ERR,
	NVC_FOCUS_STS_WAIT_FOR_MOVE_END,
	NVC_FOCUS_STS_WAIT_FOR_SETTLE,
	NVC_FOCUS_STS_LENS_SETTLED,
	NVC_FOCUS_STS_FORCE32		= 0x7FFFFFFF
};

struct nvc_focus_nvc {
	__u32 focal_length;
	__u32 fnumber;
	__u32 max_aperature;
} __packed;

struct nvc_focus_cap {
	__u32 version;
	__u32 actuator_range;
	__u32 settle_time;
	__u32 focus_macro;
	__u32 focus_hyper;
	__u32 focus_infinity;
} __packed;

#endif /* __NVC_FOCUS_H__ */

