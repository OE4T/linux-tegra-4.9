/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef INCLUDE_CAMRTC_CAPTURE_MESSAGES_H
#define INCLUDE_CAMRTC_CAPTURE_MESSAGES_H

#include "camrtc-capture.h"

#pragma GCC diagnostic error "-Wpadded"

/**
 * Standard message header for all capture IVC messages.
 *
 * Control Requests not associated with a specific channel
 * will use an opaque transaction ID rather than channel_id.
 * The transaction ID in the response message is copied from
 * the request message.
 *
 * @param msg_id	Message identifier.
 * @param channel_id	Channel identifier.
 * @param transaction	Transaction id.
 */
struct CAPTURE_MSG_HEADER {
	uint32_t msg_id;
	union {
		uint32_t channel_id;
		uint32_t transaction;
	};
} __CAPTURE_IVC_ALIGN;

/**
 * Message types for capture control channel messages.
 */
#define CAPTURE_CHANNEL_SETUP_REQ		U32_C(0x10)
#define CAPTURE_CHANNEL_SETUP_RESP		U32_C(0x11)
#define CAPTURE_CHANNEL_RESET_REQ		U32_C(0x12)
#define CAPTURE_CHANNEL_RESET_RESP		U32_C(0x13)
#define CAPTURE_CHANNEL_RELEASE_REQ		U32_C(0x14)
#define CAPTURE_CHANNEL_RELEASE_RESP		U32_C(0x15)
#define CAPTURE_COMPAND_CONFIG_REQ		U32_C(0x16)
#define CAPTURE_COMPAND_CONFIG_RESP		U32_C(0x17)
#define CAPTURE_PDAF_CONFIG_REQ			U32_C(0x18)
#define CAPTURE_PDAF_CONFIG_RESP		U32_C(0x19)
#define CAPTURE_SYNCGEN_ENABLE_REQ		U32_C(0x1A)
#define CAPTURE_SYNCGEN_ENABLE_RESP		U32_C(0x1B)
#define CAPTURE_SYNCGEN_DISABLE_REQ		U32_C(0x1C)
#define CAPTURE_SYNCGEN_DISABLE_RESP		U32_C(0x1D)

/**
 * Message types for capture channel messages.
 */
#define	CAPTURE_REQUEST_REQ			U32_C(0x01)
#define	CAPTURE_STATUS_IND			U32_C(0x02)

/**
 * Invalid message type. This can be used to
 * respond to an invalid request.
 */
#define CAPTURE_MSG_ID_INVALID			U32_C(0xFFFFFFFF)

/**
 * Result codes.
 */
#define CAPTURE_OK				U32_C(0)
#define CAPTURE_ERROR_INVALID_PARAMETER		U32_C(1)
#define CAPTURE_ERROR_NO_MEMORY			U32_C(2)
#define CAPTURE_ERROR_BUSY			U32_C(3)
#define CAPTURE_ERROR_NOT_SUPPORTED		U32_C(4)
#define CAPTURE_ERROR_NOT_INITIALIZED		U32_C(5)
#define CAPTURE_ERROR_OVERFLOW			U32_C(6)
#define CAPTURE_ERROR_NO_RESOURCES		U32_C(7)


/** Set up RTCPU side resources for a capture pipe-line.
 *
 * The client shall use the transaction id field in the
 * standard message header to assocuiate request and response.
 *
 * @param channel_config	Capture channel configuration.
 */
struct CAPTURE_CHANNEL_SETUP_REQ_MSG {
	struct capture_channel_config	channel_config;
} __CAPTURE_IVC_ALIGN;

/** Acknowledge capture channel setup request.
 *
 * The transaction id field in the standard message header
 * will be copied from the associated request.
 *
 * The setup response message returns a @a channel_id, which
 * identifies this set of resources and is used to refer to the
 * allocated capture channel in subsequent messages.
 *
 * @param result		Return value.
 * @param channel_id		Capture channel identifier for the new channel.
 * @param vi_channel_mask	Allocated VI channel(s).
 */
struct CAPTURE_CHANNEL_SETUP_RESP_MSG {
	int32_t result;
	uint32_t channel_id;
	uint64_t vi_channel_mask;
} __CAPTURE_IVC_ALIGN;

/** Reset a capture channel.
 *
 * Halt the associated VI channel. Flush the request queue for the
 * channel and increment syncpoints in the request queue to their target
 * values.
 *
 * @param reset_flags		Reset flags.
 */
struct CAPTURE_CHANNEL_RESET_REQ_MSG {
	uint32_t reset_flags;

/** Reset the channel without waiting for FE first. */
#define CAPTURE_CHANNEL_RESET_FLAG_IMMEDIATE	U32_C(0x01)

	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/** Acknowledge a capture channel reset.
 *
 * The response is sent after the RTCPU side channel cleanup is
 * complete.
 *
 * @param result	Return value.
 */
struct CAPTURE_CHANNEL_RESET_RESP_MSG {
	int32_t result;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/** Reset a capture channel and release all the associated resources.
 *
 * Halt the associated VI channel. Flush the request queue for the
 * channel and increment syncpoints in the request queue to their target
 * values.
 *
 * @param reset_flags		Reset flags.
 */
struct CAPTURE_CHANNEL_RELEASE_REQ_MSG {
	/** See CAPTURE_CHANNEL_RESET_REQ_MSG for details. */
	uint32_t reset_flags;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/** Acknowledge a capture channel release.
 *
 * The release is acknowledged after the channel cleanup is complete
 * and all resources have been freed on RTCPU.
 *
 * @param result	Return value.
 */
struct CAPTURE_CHANNEL_RELEASE_RESP_MSG {
	int32_t result;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/** Configure the piece-wise linear function used by the VI companding module.
 *
 * The companding table is shared by all capture channels and must be
 * configured before enabling companding for a specific capture.
 */
struct CAPTURE_COMPAND_CONFIG_REQ_MSG {
	struct vi_compand_config compand_config;
} __CAPTURE_IVC_ALIGN;

/** Acknowledge a companding configuration.
 */
struct CAPTURE_COMPAND_CONFIG_RESP_MSG {
	int32_t result;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/** Configure the PDAF pattern.
 *
 * @param	pdaf_config	PDAF configuration data.
 */
struct CAPTURE_PDAF_CONFIG_REQ_MSG {
	struct vi_pdaf_config pdaf_config;
} __CAPTURE_IVC_ALIGN;

/** Acknowledge CAPTURE_PDAF_CONFIG_REQ
 *
 * @param result		Return value.
 */
struct CAPTURE_PDAF_CONFIG_RESP_MSG {
	int32_t result;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/** Enable SLVS-EC synchronization
 *
 * Enable the generation of XVS and XHS synchronization signals for a
 * SLVS-EC sensor.
 */
struct CAPTURE_SYNCGEN_ENABLE_REQ_MSG {
	uint32_t unit;
	uint32_t __pad;
	struct vi_syncgen_config syncgen_config;
} __CAPTURE_IVC_ALIGN;

/** Acknowledge CAPTURE_SYNCGEN_ENABLE_REQ. */
struct CAPTURE_SYNCGEN_ENABLE_RESP_MSG {
	uint32_t unit;
	int32_t result;
} __CAPTURE_IVC_ALIGN;

/** Disable SLVS-EC synchronization
 *
 * Disable the generation of XVS and XHS synchronization signals for a
 * SLVS-EC sensor.
 */
struct CAPTURE_SYNCGEN_DISABLE_REQ_MSG {
	uint32_t unit;
	uint32_t syncgen_disable_flags;

/* Disable SYNCGEN without waiting for frame end */
#define CAPTURE_SYNCGEN_DISABLE_FLAG_IMMEDIATE	U32_C(0x01)

} __CAPTURE_IVC_ALIGN;

/** Acknowledge CAPTURE_SYNCGEN_DISABLE_REQ. */
struct CAPTURE_SYNCGEN_DISABLE_RESP_MSG {
	uint32_t unit;
	int32_t result;
} __CAPTURE_IVC_ALIGN;

/**
 * Message definition for capture control channel messages.
 */
struct CAPTURE_CONTROL_MSG {
	struct CAPTURE_MSG_HEADER header;
	union {
		struct CAPTURE_CHANNEL_SETUP_REQ_MSG channel_setup_req;
		struct CAPTURE_CHANNEL_SETUP_RESP_MSG channel_setup_resp;
		struct CAPTURE_CHANNEL_RESET_REQ_MSG channel_reset_req;
		struct CAPTURE_CHANNEL_RESET_RESP_MSG channel_reset_resp;
		struct CAPTURE_CHANNEL_RELEASE_REQ_MSG channel_release_req;
		struct CAPTURE_CHANNEL_RELEASE_RESP_MSG channel_release_resp;
		struct CAPTURE_COMPAND_CONFIG_REQ_MSG compand_config_req;
		struct CAPTURE_COMPAND_CONFIG_RESP_MSG compand_config_resp;
		struct CAPTURE_PDAF_CONFIG_REQ_MSG pdaf_config_req;
		struct CAPTURE_PDAF_CONFIG_RESP_MSG pdaf_config_resp;
		struct CAPTURE_SYNCGEN_ENABLE_REQ_MSG syncgen_enable_req;
		struct CAPTURE_SYNCGEN_ENABLE_RESP_MSG syncgen_enable_resp;
		struct CAPTURE_SYNCGEN_DISABLE_REQ_MSG syncgen_disable_req;
		struct CAPTURE_SYNCGEN_DISABLE_RESP_MSG syncgen_disable_resp;
	};
} __CAPTURE_IVC_ALIGN;



/**
 * Enqueue a new capture request on a capture channel.
 *
 * The request contains channel identifier and the capture sequence
 * number, which are required to schedule the capture request. The
 * actual capture programming is stored in the capture descriptor,
 * stored in a DRAM ring buffer set up with CAPTURE_CHANNEL_SETUP_REQ.
 *
 * The capture request descriptor with buffer_index=N can be located
 * within the ring buffer as follows:
 *
 * struct capture_descriptor *desc = requests + buffer_index * request_size;
 *
 * The capture request message is asynchronous. Capture completion is
 * indicated by incrementing the progress syncpoint a pre-calculated
 * number of times = 1 + <number of sub-frames>. The first increment
 * occurs at start-of-frame and the last increment occurs at
 * end-of-frame. The progress-syncpoint is used to synchronize with
 * down-stream engines. This model assumes that the capture client
 * knows the number of subframes used in the capture and has
 * programmed the VI accordingly.
 *
 * If the flag CAPTURE_FLAG_STATUS_REPORT_ENABLE is set in the capture
 * descriptor, RTCPU will store the capture status into status field
 * of the descriptor. RTCPU will also send a CAPTURE_STATUS_IND
 * message to indicate that capture has completed. The capture status
 * record contains information about the capture, such as CSI frame
 * number, start-of-frame and end-of-frame timestamps, as well as
 * error status.
 *
 * If the flag CAPTURE_FLAG_ERROR_REPORT_ENABLE is set, RTCPU will send a
 * CAPTURE_STATUS_IND upon an error, even if
 * CAPTURE_FLAG_STATUS_REPORT_ENABLE is not set.
 *
 * @param buffer_index	Buffer index identifying capture descriptor.
 */
struct CAPTURE_REQUEST_REQ_MSG {
	uint32_t buffer_index;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/** Capture status indication.
 *
 * The message is sent after the capture status record has been
 * written into the capture request descriptor.
 *
 * @param buffer_index	Buffer index identifying capture descriptor.
 */
struct CAPTURE_STATUS_IND_MSG {
	uint32_t buffer_index;
	uint32_t __pad;
} __CAPTURE_IVC_ALIGN;

/**
 * Message definition for capture channel messages.
 */
struct CAPTURE_MSG {
	struct CAPTURE_MSG_HEADER header;
	union {
		struct CAPTURE_REQUEST_REQ_MSG capture_request_req;
		struct CAPTURE_STATUS_IND_MSG capture_status_ind;
	};
} __CAPTURE_IVC_ALIGN;

#pragma GCC diagnostic ignored "-Wpadded"

#endif /* INCLUDE_CAMRTC_CAPTURE_MESSAGES_H */
