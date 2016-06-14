/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All Rights Reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property and
 * proprietary rights in and to this software and related documentation.  Any
 * use, reproduction, disclosure or distribution of this software and related
 * documentation without an express license agreement from NVIDIA Corporation
 * is strictly prohibited.
 */

#ifndef _DLA_INTERFACE_H_
#define _DLA_INTERFACE_H_

/* Commands from host to DLA Falcon */
#define DLA_CMD_PING			1
#define DLA_CMD_GET_STATUS		2
#define DLA_CMD_RESET			3
#define DLA_CMD_DLA_CONTROL		4
#define DLA_CMD_GET_QUEUE_STATUS	5
#define DLA_CMD_GET_STATISTICS		6
#define DLA_CMD_SUBMIT_TASK		7
#define DLA_CMD_SET_SCHEDULER		8
#define DLA_CMD_READ_INFO		9
#define DLA_CMD_SET_DEBUG		10
#define DLA_CMD_SET_REGIONS		11
#define DLA_CMD_QUEUE_SUSPEND		12
#define DLA_CMD_QUEUE_RESUME		13
#define DLA_CMD_QUEUE_FLUSH		14

/* Notifications from DLA Falcon to Host */
#define DLA_ERROR_NONE			0
#define DLA_ERROR_INVALID_METHOD	1
#define DLA_ERROR_EXCEPTION		2
#define DLA_ERROR_UNHANDLED_INTERRUPT	3
#define DLA_ERROR_INVALID_TASK		4
#define DLA_ERROR_INVALID_INPUT		5
#define DLA_ERROR_SWBREAKPT		6
#define DLA_DEBUG_PRINT			7

#endif
