/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef INCLUDE_CAMRTC_CAPTURE_H
#define INCLUDE_CAMRTC_CAPTURE_H

#include "camrtc-common.h"

#pragma GCC diagnostic error "-Wpadded"
#define __CAPTURE_IVC_ALIGN __aligned(8)
#define __CAPTURE_DESCRIPTOR_ALIGN __aligned(64)

typedef uint64_t iova_t __CAPTURE_IVC_ALIGN;

#define SYNCPOINT_ID_INVALID	0xFFFFFFFFUL

typedef struct syncpoint_info {
	uint32_t id;
	uint32_t threshold;	/* When storing a fence */
	uint32_t gos_index;
	uint32_t gos_offset;
	iova_t shim_addr;
} syncpoint_info_t __CAPTURE_IVC_ALIGN;

#define VI_NUM_ATOMP_SURFACES	4

/* Generic */
#define VI_ATOMP_SURFACE0	0
#define VI_ATOMP_SURFACE1	1
#define VI_ATOMP_SURFACE2	2

/* Sensor embedded data */
#define VI_ATOMP_SURFACE_EMBEDDED 3

/* RAW */
#define VI_ATOMP_SURFACE_MAIN	VI_ATOMP_SURFACE0
/* PDAF pixels */
#define VI_ATOMP_SURFACE_PDAF	VI_ATOMP_SURFACE1

/* YUV */
#define VI_ATOMP_SURFACE_Y	VI_ATOMP_SURFACE0
#define	VI_ATOMP_SURFACE_UV	VI_ATOMP_SURFACE1 /* semi-planar */
#define	VI_ATOMP_SURFACE_U	VI_ATOMP_SURFACE1 /* planar */
#define	VI_ATOMP_SURFACE_V	VI_ATOMP_SURFACE2 /* planar */

/**
 * Describes RTCPU side resources for a capture pipe-line.
 *
 * The following parameters describe the capture descriptor ring buffer.
 *
 * @param requests: base address of a memory mapped ring buffer
 *                  containing capture requests. The size of the
 *                  buffer is queue_depth * request_size.
 * @param queue_depth: number of capture requests in the @a requests queue
 * @param request_size: size of the buffer reserved for each capture request.
 *
 * The following attributes indicate what resources need to be
 * allocated for the capture channel:
 *
 * @param channel_flags: a bitmask describing the set of non-shareable
 * HW resources that the capture channel will need. These HW resources
 * will be assigned to the new capture channel and will be owned by the
 * channel until it is released with CAPTURE_CHANNEL_RELEASE_REQ.
 *
 * The HW resources that can be assigned to a channel include a VI
 * channel, ISPBUF A/B interface (T18x only), Focus Metric Lite module
 * (FML).
 *
 * VI channels can have different capabilities. The flags are checked
 * against the VI channel capabilities to make sure the allocated VI
 * channel meets the requirements.  The following flags are defined:
 * <dl>
 * <dt>VIDEO:   <dd>Channel takes input from Video Interface (VI)
 * <dt>RAW:     <dd>Channel supports RAW Bayer output
 * <dt>PLANAR:  <dd>Channel supports planar YUV output
 * <dt>SEMI_PLANAR: <dd>Channel supports semi-planar YUV output
 * <dt>PDAF:    <dd>Channel supports phase-detection auto-focus
 * <dt>FMLITE:  <dd>Channel outputs to Focus Metric Lite module (FML)
 * <dt>EMBDATA: <dd>Channel outputs sensor embedded data
 * <dt>ISPA:    <dd>Channel outputs to ISPA
 * <dt>ISPB:    <dd>Channel outputs to ISPB
 * <dt>ISP_DIRECT: <dd>Channel outputs directly to selected ISP (ISO mode)
 * <dt>ISPSW:   <dd>Channel outputs to software ISP (reserved)
 * </dl>
 *
 * @param vi_channel_mask  A bit mask indicating which VI channels to
 * consider for allocation. This allows the client VM to statically
 * partition VI channels for its own purposes. The RTCPU will enforce
 * any partitioning between VMs.
 */
struct capture_channel_config {
	uint32_t channel_flags;

#define CAPTURE_CHANNEL_FLAG_VIDEO		0x0001U
#define CAPTURE_CHANNEL_FLAG_RAW		0x0002U
#define CAPTURE_CHANNEL_FLAG_PLANAR		0x0004U
#define CAPTURE_CHANNEL_FLAG_SEMI_PLANAR	0x0008U
#define CAPTURE_CHANNEL_FLAG_PDAF		0x0010U
#define CAPTURE_CHANNEL_FLAG_FMLITE		0x0020U
#define CAPTURE_CHANNEL_FLAG_EMBDATA		0x0040U
#define CAPTURE_CHANNEL_FLAG_ISPA		0x0080U
#define CAPTURE_CHANNEL_FLAG_ISPB		0x0100U
#define CAPTURE_CHANNEL_FLAG_ISP_DIRECT		0x0200U
#define CAPTURE_CHANNEL_FLAG_ISPSW		0x0400U
#define CAPTURE_CHANNEL_FLAG_RESET_ON_ERROR	0x0800U

	uint32_t channel_id;	/* rtcpu internal - set to zero */
	uint64_t vi_channel_mask;
	iova_t requests;
	uint32_t queue_depth;
	uint32_t request_size;
	struct syncpoint_info progress_sp;
	struct syncpoint_info embdata_sp;
	struct syncpoint_info linetimer_sp;
} __CAPTURE_IVC_ALIGN;

struct vi_channel_config {
	/* Flags */
	unsigned dt_enable:1;
	unsigned embdata_enable:1;
	unsigned flush_enable:1;
	unsigned flush_periodic:1;
	unsigned line_timer_enable:1;
	unsigned line_timer_periodic:1;
	unsigned pixfmt_enable:1;
	unsigned pixfmt_wide_enable:1;
	unsigned pixfmt_wide_endian:1;
	unsigned pixfmt_pdaf_replace_enable:1;
	unsigned ispbufa_enable:1;
	unsigned ispbufb_enable:1;
	unsigned fmlite_enable:1;
	unsigned __pad_flags:19;

	/* vi channel selector */
	struct {
		uint8_t datatype;
		uint8_t datatype_mask;
		uint8_t stream;
		uint8_t stream_mask;
		uint16_t vc;
		uint16_t vc_mask;
		uint16_t frameid;
		uint16_t frameid_mask;
		uint16_t dol;
		uint16_t dol_mask;
	} match;

	/* frame size and crop */
	struct vi_frame_config {
		uint16_t frame_x;
		uint16_t frame_y;
		uint32_t embed_x;
		uint32_t embed_y;
		struct {
			uint16_t x;
			uint16_t y;
		} skip;
		struct {
			uint16_t x;
			uint16_t y;
		} crop;
	} frame;

	/* Flush timer/slice height */
	uint16_t flush;

	/* Line timer trip-line */
	uint16_t line_timer;

	/* Datatype override */
	uint8_t dt_override;
	uint8_t __pad_dt[3];

	/* Pixel formatter */
	struct {
		uint16_t format;
		uint16_t __pad;
		struct {
			uint16_t crop_left;
			uint16_t crop_right;
			uint16_t crop_top;
			uint16_t crop_bottom;
			uint16_t replace_crop_left;
			uint16_t replace_crop_right;
			uint16_t replace_crop_top;
			uint16_t replace_crop_bottom;
			uint16_t last_pixel_x;
			uint16_t last_pixel_y;
			uint16_t replace_value;
			uint8_t format;
			uint8_t __pad_pdaf;
		} pdaf;
	} pixfmt;

	/* Pixel DPCM */
	struct {
		uint16_t strip_width;
		uint16_t strip_overfetch;

		/* Not for T186 or earlier */
		uint16_t chunk_first;
		uint16_t chunk_body;
		uint16_t chunk_penultimate;
		uint16_t chunk_last;
		uint8_t  mode;
		uint8_t  __pad_mode;
		uint16_t clamp;
	} dpcm;

	/* Atom packer */
	struct {
		struct {
			uint32_t offset;
			uint32_t offset_hi;
		} surface[VI_NUM_ATOMP_SURFACES];
		uint32_t surface_stride[VI_NUM_ATOMP_SURFACES];
		uint32_t dpcm_chunk_offset;
	} atomp;
} __CAPTURE_IVC_ALIGN;

struct capture_status {
	uint8_t src_stream;
	uint8_t virtual_channel;
	uint16_t frame_id;
	uint32_t status;

#define CAPTURE_STATUS_UNKNOWN			0
#define CAPTURE_STATUS_SUCCESS			1
#define CAPTURE_STATUS_CSIMUX_FRAME		2
#define CAPTURE_STATUS_CSIMUX_STREAM		3
#define CAPTURE_STATUS_CHANSEL_FAULT		4
#define CAPTURE_STATUS_CHANSEL_FAULT_FE		5
#define CAPTURE_STATUS_CHANSEL_COLLISION	6
#define CAPTURE_STATUS_CHANSEL_SHORT_FRAME	7
#define CAPTURE_STATUS_ATOMP_PACKER_OVERFLOW	8
#define CAPTURE_STATUS_ATOMP_FRAME_TRUNCATED	9
#define CAPTURE_STATUS_ATOMP_FRAME_TOSSED	10
#define CAPTURE_STATUS_ISPBUF_FIFO_OVERFLOW	11
#define CAPTURE_STATUS_SYNC_FAILURE		12
#define CAPTURE_STATUS_NOTIFIER_BACKEND_DOWN	13

	uint64_t sof_timestamp;
	uint64_t eof_timestamp;
	uint32_t err_data;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

#define VI_AFM_NUM_ROI			8
#define VI_AFM_NUM_TRANSFER_KNOTS	11

struct vi_fmlite_config {
	uint32_t vfm_prog;
	uint32_t vfm_ctrl;
	uint32_t vfm_black_level;
	uint32_t vfm_hdr_sample_map;
	uint32_t vfm_hdr_scale;
	uint32_t vfm_hdr_sat;
	uint32_t vfm_h_pi;
	uint32_t vfm_v_pi;
	uint32_t vfm_offset;
	uint32_t vfm_size;
	uint32_t vfm_hf_c0;
	uint32_t vfm_hf_c1;
	uint32_t vfm_hf_c2;
	uint32_t vfm_vf_c0;
	uint32_t vfm_vf_c1;
	uint32_t vfm_vf_c2;
	uint32_t vfm_vf_c3;
	uint32_t vfm_vf_c4;
	uint32_t ctrl;
	uint32_t color;
	uint32_t transfer_slope;
	uint32_t transfer_x;
	uint32_t transfer_y;
	uint32_t transfer_cubic_ctrl;
	uint32_t transfer_knots[VI_AFM_NUM_TRANSFER_KNOTS];
	uint32_t roi_pos[VI_AFM_NUM_ROI];
	uint32_t roi_size[VI_AFM_NUM_ROI];
	uint32_t trap_en;
	uint32_t hstart[VI_AFM_NUM_ROI];
	uint32_t vstart[VI_AFM_NUM_ROI];
	uint32_t slope[VI_AFM_NUM_ROI];
	uint32_t coeff01;
	uint32_t coeff23;
	uint32_t coeff45;
	uint32_t error;
} __CAPTURE_IVC_ALIGN;

struct vi_fmlite_result {					/* 72 bytes */
	uint32_t error;
	uint32_t __pad;
	uint64_t roi[VI_AFM_NUM_ROI];
} __CAPTURE_IVC_ALIGN;

/*
 * The compand configuration describes a piece-wise linear
 * tranformation function used by the VI companding module.
 */
#define VI_NUM_COMPAND_KNEEPTS 10
struct vi_compand_config {
	uint32_t base[VI_NUM_COMPAND_KNEEPTS];
	uint32_t scale[VI_NUM_COMPAND_KNEEPTS];
	uint32_t offset[VI_NUM_COMPAND_KNEEPTS];
} __CAPTURE_IVC_ALIGN;

/*
 * The phase-detection auto-focus data consists of special pixels that
 * will be extracted from a frame and written to a separate
 * surface. The PDAF pattern is shared by all capture channels and
 * should be configured before enabling PDAF pixel extraction for a
 * specific capture.
 *
 * Pixel { x, y } will be ouput to the PDAF surface (surface1) if the
 * bit at position (x % 32) in pattern[y % 32] is set.
 *
 * Pixel { x, y } in the main output surface (surface0) will be
 * replaced by a default pixel value if the bit at position (x % 32)
 * in pattern_replace[y % 32] is set.
 */
#define VI_PDAF_PATTERN_SIZE 32
struct vi_pdaf_config {
	uint32_t pattern[VI_PDAF_PATTERN_SIZE];
	uint32_t pattern_replace[VI_PDAF_PATTERN_SIZE];
} __CAPTURE_IVC_ALIGN;


/*
 * Configuration for VI SYNGEN unit.
 */
struct vi_syncgen_config {
	uint32_t hclk_div;
	uint8_t hclk_div_fmt;
	uint8_t xhs_width;
	uint8_t xvs_width;
	uint8_t xvs_to_xhs_delay;
	uint16_t cvs_interval;
	uint16_t __pad1;
	uint32_t __pad2;
} __CAPTURE_IVC_ALIGN;


struct capture_descriptor {
	uint32_t sequence;
	uint32_t capture_flags;

#define CAPTURE_FLAG_STATUS_REPORT_ENABLE	(1U << 0)
#define CAPTURE_FLAG_ERROR_REPORT_ENABLE	(1U << 1)

	uint32_t frame_timeout;			/**< Timeout in microseconds */

#define CAPTURE_PREFENCE_ARRAY_SIZE		2

	uint32_t prefence_count;
	struct syncpoint_info prefence[CAPTURE_PREFENCE_ARRAY_SIZE];

	struct vi_channel_config ch_cfg;
	struct vi_fmlite_config fm_cfg;

	/* Result record – written by RTCPU */
	struct capture_status status;
	/* FMLITE result – written by RTCPU */
	struct vi_fmlite_result fm_result;

	/* Pad to aligned size */
	uint32_t __pad[2];
} __CAPTURE_DESCRIPTOR_ALIGN;

#pragma GCC diagnostic ignored "-Wpadded"

#endif /* INCLUDE_CAMRTC_CAPTURE_H */
