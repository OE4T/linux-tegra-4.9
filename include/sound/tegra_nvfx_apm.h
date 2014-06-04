/*
 * tegra_nvfx_apm.h - Shared APM interface between Tegra ADSP ALSA driver and
 *                    ADSP side user space code.
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _TEGRA_NVFX_APM_H_
#define _TEGRA_NVFX_APM_H_

#define NVFX_MAX_FX_IN_APM          4
#define NVFX_APM_CMD_QUEUE_WSIZE    1024

/**
 * apm_mbx_cmd:  commands exchanged using mailbox.
 *
 * @apm_cmd_none:           no operation.
 * @apm_cmd_msg_ready       message queue holds a new message.
 * @apm_cmd_msg_exit        exit command
 */
enum apm_mbx_cmd {
	apm_cmd_none = 0,
	apm_cmd_msg_ready,
	apm_cmd_msg_exit,
};

/**
 * APM methods
 *
 */
enum {
	/* NVFX APM params */
	nvfx_apm_method_fx_connect = nvfx_method_external_start,
	nvfx_apm_method_fx_remove,
	nvfx_apm_method_fx_set_param,

	nvfx_apm_method_set_io_buffer,
	nvfx_apm_method_set_notification_size,
	nvfx_apm_method_set_write_position,
	nvfx_apm_method_set_eos,

	nvfx_apm_method_eos_reached,            /* from ADSP to CPU */
	nvfx_apm_method_buffer_empty,           /* from ADSP to CPU */
};

typedef struct {
	nvfx_call_params_t call_params;
	uint32_t pin_type; /* NVFX_INPUT_PIN or NVFX_OUTPUT_PIN */
	uint32_t pin_id;
	variant_t addr;
	uint32_t size;
	uint32_t flags;
} apm_set_io_buffer_params_t;

/* apm_set_write_position */
typedef struct {
	nvfx_call_params_t call_params;
	uint32_t pin_id;
	uint32_t offset;
} apm_set_write_position_params_t;

/* apm_set_notification_size */
typedef struct {
	nvfx_call_params_t call_params;
	uint32_t pin_type; /* NVFX_INPUT_PORT or NVFX_OUTPUT_PORT */
	uint32_t pin_id;
	uint32_t size;
} apm_set_notification_size_params_t;

/* apm_set_eos */
typedef struct {
	nvfx_call_params_t call_params;
} apm_set_eos_params_t;

/* apm_eos_reached */
typedef struct {
	nvfx_call_params_t call_params;
} apm_eos_reached_params_t;

/* apm_buffer_empty */
typedef struct {
	nvfx_call_params_t call_params;
	uint32_t pin_type;          /* NVFX_INPUT_PIN or NVFX_OUTPUT_PIN */
	uint32_t pin_id;
} apm_buffer_empty_params_t;

/* Module specific structures */
typedef struct {
	nvfx_call_params_t call_params;
	variant_t plugin_src; /* pointer to plugin_t */
	int32_t port_src; /* input or output port */
	int32_t pin_src; /* pin id to connect */
	variant_t plugin_dst; /* pointer to plugin_t */
	int32_t port_dst;
	int32_t pin_dst;
} apm_fx_connect_params_t;

typedef struct {
	nvfx_call_params_t call_params;
	variant_t plugin;
} apm_fx_remove_params_t;

typedef struct {
	nvfx_call_params_t call_params;
	variant_t plugin;   /* pointer to plugin_t */
	int32_t params[NVFX_MAX_CALL_PARAMS_SIZE];
} apm_fx_set_param_params_t;

/* unified app message structure */
#pragma pack(4)
typedef union {
	msgq_message_t msgq_msg;
	struct {
		int32_t header[MSGQ_MESSAGE_HEADER_WSIZE];
		union {
			nvfx_call_params_t                 call_params;
			apm_set_io_buffer_params_t         io_buffer_params;
			apm_set_write_position_params_t    position_params;
			apm_set_notification_size_params_t notification_params;
			nvfx_set_state_params_t            set_state_params;
			nvfx_reset_params_t                set_reset_params;
			apm_set_eos_params_t               set_eos_params;
			apm_eos_reached_params_t           eos_reached_params;
			apm_buffer_empty_params_t          buffer_empty_params;
			apm_fx_connect_params_t            fx_connect_params;
			apm_fx_remove_params_t             fx_remove_params;
			apm_fx_set_param_params_t          fx_set_param_params;
		};
	} msg;
} apm_msg_t;

/* app message queue */
typedef union {
	msgq_t msgq;
	struct {
		int32_t header[MSGQ_HEADER_WSIZE];
		int32_t queue[NVFX_APM_CMD_QUEUE_WSIZE];
	} app_msgq;
} apm_msgq_t;
#pragma pack()

/**
 *
 */
typedef struct {
	nvfx_shared_state_t nvfx_shared_state;
	uint16_t    mbox_id;       /* mailbox for communication */
	apm_msgq_t  msgq_recv;
	apm_msgq_t  msgq_send;
	variant_t   sem_input; /* semaphore pointer for signaling data ready */
	variant_t   sem_output;/* semaphore pointer for signaling data needed */
} apm_shared_state_t;

typedef struct {
	variant_t plugin;
	/* NVFX specific shared memory follows */
} plugin_shared_mem_t;

#define PLUGIN_SHARED_MEM(x) ((plugin_shared_mem_t *)x)
#define APM_SHARED_STATE(x) (apm_shared_state_t *)(PLUGIN_SHARED_MEM(x) + 1)
#define NVFX_SHARED_STATE(x) (nvfx_shared_state_t *)(PLUGIN_SHARED_MEM(x) + 1)

#endif /* #ifndef _TEGRA_NVFX_APM_H_ */
