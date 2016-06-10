/*
 * Copyright (c) 2016 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef _TRUSTY_OTE_H_
#define __TRUSTY_OTE_H_
/*
 * NOTE: These OTE structures need to be in sync with the ones defined on
 * $TOP/ote/lib
 */

/*! Defines the maximum number of parameters in a te_operation_t */
#define TE_OPER_MAX_PARAMS 10

/*! Specifies the operation object's parameter types. */
typedef enum {
	TE_PARAM_TYPE_NONE		= 0x0,
	TE_PARAM_TYPE_INT_RO		= 0x1,
	TE_PARAM_TYPE_INT_RW		= 0x2,
	TE_PARAM_TYPE_MEM_RO		= 0x3,
	TE_PARAM_TYPE_MEM_RW		= 0x4,
	TE_PARAM_TYPE_PERSIST_MEM_RO	= 0x100,
	TE_PARAM_TYPE_PERSIST_MEM_RW	= 0x101
} te_oper_param_type_t;

/** Defines Open Trusted Environment (OTE) error codes. */
typedef enum {
        /// Indicates the operation was successful.
	OTE_SUCCESS		  = 0x00000000,
        /// Indicates the operation was successful.
	OTE_ERROR_NO_ERROR	  = 0x00000000,
        /// Indicates an unspecified error occurred.
	OTE_ERROR_GENERIC	  = 0xFFFF0000,
        /// Indicates access privileges are insufficient.
	OTE_ERROR_ACCESS_DENIED	  = 0xFFFF0001,
        /// Indicates the operation was cancelled.
	OTE_ERROR_CANCEL	  = 0xFFFF0002,
        /// Indicates a concurrent accesses conflict.
	OTE_ERROR_ACCESS_CONFLICT = 0xFFFF0003,
        /// Indicates data passed exceeds request.
	OTE_ERROR_EXCESS_DATA	  = 0xFFFF0004,
        /// Indicates input data is in an invalid format.
	OTE_ERROR_BAD_FORMAT	  = 0xFFFF0005,
        /// Indicates input parameters are invalid.
	OTE_ERROR_BAD_PARAMETERS  = 0xFFFF0006,
        /// Indicates the operation is invalid in its current state.
	OTE_ERROR_BAD_STATE	  = 0xFFFF0007,
        /// Indicates the requested data item was not found.
	OTE_ERROR_ITEM_NOT_FOUND  = 0xFFFF0008,
        /// Indicates the requested operation was not implemented.
	OTE_ERROR_NOT_IMPLEMENTED = 0xFFFF0009,
        /// Indicates the requested operation is not supported.
	OTE_ERROR_NOT_SUPPORTED	  = 0xFFFF000A,
        /// Indicates the data expected is missing.
	OTE_ERROR_NO_DATA	  = 0xFFFF000B,
        /// Indicates the system ran out of resources.
	OTE_ERROR_OUT_OF_MEMORY	  = 0xFFFF000C,
        /// Indicates the system is busy.
	OTE_ERROR_BUSY		  = 0xFFFF000D,
        /// Indicates that communication failed.
	OTE_ERROR_COMMUNICATION	  = 0xFFFF000E,
        /// Indicates a security fault was detected.
	OTE_ERROR_SECURITY	  = 0xFFFF000F,
        /// Indicates the supplied buffer is too short.
	OTE_ERROR_SHORT_BUFFER	  = 0xFFFF0010,
        /// Task administratively blocked, does not accept new sessions.
	OTE_ERROR_BLOCKED	  = 0xFFFF0011,
        /// Indicates no answer was received from the command target.
	OTE_ERROR_NO_ANSWER = 0xFFFF1003,
} te_error_t;

/*! Holds a pointer large enough to support 32- and 64-bit clients. */
typedef uint64_t	cmnptr_t;

/*! Holds the operation object parameters. */
typedef struct {
	uint32_t index;
	te_oper_param_type_t type;
	union {
		struct {
			uint32_t val;
		} Int;
		struct {
			cmnptr_t base;
			uint32_t len;
			uint32_t type;
		} Mem;
	} u;
	cmnptr_t next;
} te_oper_param_t;

/*! Holds operation object params in a static array instead of linked list */
typedef struct {
	uint32_t command;
	te_error_t status;
	uint32_t list_count;
	uint32_t interface_side;
	te_oper_param_t params[TE_OPER_MAX_PARAMS];
} te_operation_container_t;

#endif
