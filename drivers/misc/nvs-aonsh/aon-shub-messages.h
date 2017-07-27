/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef _AON_SHUB_MESSAGES_H_
#define _AON_SHUB_MESSAGES_H_

#include <linux/types.h>
#include <linux/nvs.h>

#define TEGRA_IVC_ALIGN		64
#define AON_SHUB_MAX_DATA_SIZE	256

/* All the enums and the fields inside the structs described in this header
 * file supports only [u/s]X type, where X can be 8,16,32. For inter CPU
 * communication, it is more stable to use this type.
 */

/* This enum represents the types of requests to sensor hub associated
 * with AON.
 */
enum aon_shub_request_type {
	AON_SHUB_REQUEST_MIN = 1,
	AON_SHUB_REQUEST_INIT = 1,
	AON_SHUB_REQUEST_SYS = 2,
	AON_SHUB_REQUEST_SNSR_CFG = 3,
	AON_SHUB_REQUEST_ENABLE = 4,
	AON_SHUB_REQUEST_BATCH = 5,
	AON_SHUB_REQUEST_FLUSH = 6,
	AON_SHUB_REQUEST_PAYLOAD = 7,
	AON_SHUB_REQUEST_MAX = 7,
};

/* This enum represents the types of init requests to sensor hub associated
 * with AON.
 */
enum aon_shub_init_request_type {
	AON_SHUB_INIT_REQUEST_I2C = 1,
	AON_SHUB_INIT_REQUEST_SNSR = 2,
	AON_SHUB_INIT_REQUEST_SETUP = 3,
};

/* This enum represents the types of requests to AON sensor hub system
 * control.
 */
enum aon_shub_sys_request_type {
	AON_SHUB_SYS_REQUEST_DBG = 1,
	AON_SHUB_SYS_REQUEST_PM = 2,
	AON_SHUB_SYS_REQUEST_SNSR_CNT = 3,
};

/* TODO: avoid zeros to prevent
 * misrepresenting blank messages
 * This enum indicates the status of the request from CCPLEX.
 */
enum aon_shub_status {
	AON_SHUB_STATUS_OK = 0,
	AON_SHUB_STATUS_ERROR = 1,
};

/* This struct is used to represent sensor payload from the SHUB.
 *
 * Fields:
 * snsr_id:	Sensor handle to identify the sensor
 * ts:		Time stamp of the sample
 * x:		X-axis value
 * y:		Y-axis value
 * z:		Z-axis value
 */
struct sensor_payload_t {
	s32 snsr_id;
	u64 ts;
	u16 x;
	u16 y;
	u16 z;
};

/* This struct is used to represent data required to enable a sensor
 *  on the SHUB.
 * Fields:
 * snsr_id:	Sensor handle to identify the sensor
 * enable:	1 to enable the sensor
 *		0 to disable the sensor
 *		-1 to query the current enable state
 */
struct aon_shub_enable_request {
	s32 snsr_id;
	s32 enable;
};

/* This struct is used to represent data in response to a sensor enable request
 * to the SHUB.
 *
 * Fields:
 * enable:	enable state of the sensor
 */
struct aon_shub_enable_response {
	s32 enable;
};

/* This struct is used to represent batching data required for a batch
 * request to the SHUB.
 * Fields:
 * snsr_id:	Sensor handle to identify the sensor
 * flags:	Reporting mode of the sensor
 * period:	Sampling period
 * timeout:	Batch timeout until flush.
 */
struct aon_shub_batch_request {
	s32 snsr_id;
	s32 flags;
	u32 period;
	u32 timeout;
};

/* This struct is used to represent data required for a flush request
 *  to the SHUB.
 * Fields:
 * snsr_id:	Sensor handle to identify the sensor
 */
struct aon_shub_flush_request {
	s32 snsr_id;
};

/* This struct is used to represent data required for a debug request
 *  to the SHUB.
 * Fields:
 * flags:	Debug status flags
 */
struct aon_shub_dbg_request {
	u32 flags;
};

/* This struct is used to represent data required for a PM request
 * to the SHUB.
 * Fields:
 * flags:	pm status flags
 */
struct aon_shub_pm_request {
	u32 flags;
};

/* This struct is used to represent data required for an i2c controller init
 * request.
 *
 * Fields:
 * i2c_id:	I2C controller id.
 * clk_rate:	I2C controller clock rate.
 */
struct aon_shub_init_i2c_request {
	u32 i2c_id;
	u32 clk_rate;
};

/* This struct is used to represent data required for a sensor chips init
 * request.
 *
 * Fields:
 * chip_id_mask:	Bit mask of sensor chip ids.
 */
struct aon_shub_init_snsrs_request {
	u32 chip_id_mask;
};

/* This struct is used to represent data required for a sensor chip setup
 * request before init.
 *
 * Fields:
 * chip_id:	sensor chip id.
 * gpio:	GPIO number.
 * i2c_id:	I2C controller id.
 * i2c_addr:	I2C address of the device.
 */
struct aon_shub_init_setup_request {
	u32 chip_id;
	u32 gpio;
	u32 i2c_id;
	u8 i2c_addr;
};

/* This struct is used to represent data required for a init
 * request.
 * Fields:
 * req_type:	Indicates the type of init request.
 * i2c_init:	I2C controller init request.
 * snsr_init:	Sensor chip init request.
 * setup:	Sensor chip setup request.
 */
struct aon_shub_init_request {
	u32 req;
	union {
		struct aon_shub_init_i2c_request i2c_init;
		struct aon_shub_init_snsrs_request snsrs_init;
		struct aon_shub_init_setup_request setup;
	} data;
};

/* This struct is used to represent data required for a system specific
 * request rather than sensor specific.
 * Fields:
 * req_type:	Indicates the type of system state/ctrl request.
 * dbg:		DBG request
 * pm:		Power management related request
 */
struct aon_shub_sys_request {
	u32 req;
	union {
		struct aon_shub_dbg_request dbg;
		struct aon_shub_pm_request pm;
	} data;
};

/* This struct indicates the response of sensor cfg fetch request for a sensor
 * on AON SHUB.
 * Fields:
 */
struct aon_shub_snsrcfg_response {
	u8 name[32];
	u8 part[32];
	u8 vendor[32];
	s32 version;
	s32 snsr_id;
	s32 kbuf_sz;		/* kernel buffer size (n bytes) */
	s32 timestamp_sz;	/* hub: timestamp size (n bytes) */
	s32 snsr_data_n;
	u32 ch_n;
	s32 ch_sz;
	struct nvs_float max_range;
	struct nvs_float resolution;
	struct nvs_float milliamp;
	s32 delay_us_min;
	s32 delay_us_max;
	u32 fifo_rsrv_evnt_cnt;
	u32 fifo_max_evnt_cnt;
	u32 flags;
	s8 matrix[9];
	s32 uncal_lo;
	s32 uncal_hi;
	s32 cal_lo;
	s32 cal_hi;
	s32 thresh_lo;
	s32 thresh_hi;
	s32 report_n;
	u32 float_significance;
	/* global scale/offset allows for a dd1st order polynomial on the data
	 * e.g. data * scale + offset
	 */
	struct nvs_float scale;
	struct nvs_float offset;
	u32 ch_n_max;		/* NVS_CHANNEL_N_MAX */
};

/* This struct indicates the contents of sensor cfg fetch request for a sensor
 * on AON SHUB.
 * Fields:
 * index:	Index of the sensor
 */
struct aon_shub_snsrcfg_request {
	u32 index;
};

/* This struct is used to represent data in response to a system specific
 * request.
 * Fields:
 * snsr_cnt:	Sensor count
 */
struct aon_shub_sys_response {
	u32 snsr_cnt;
};

/* This struct is used to represent data in response to a init request.
 *
 * Fields:
 * init_type:	Init request type
 * status:	Status of the init request
 */
struct aon_shub_init_response {
	u32 init_type;
	u32 status;
};


/* This struct is used to represent sensor payload data of each sensor.
 *
 * Fields:
 * count:	Number of samples
 * data:	Sensor payload
 */
struct aon_shub_payload_response {
	u32 count;
	/* Max 2 physical sensors for now on a single device.
	 * This can be changed if required.
	 */
	struct sensor_payload_t data[2];
};

/* This structure indicates the contents of the response from the remote CPU
 * i.e SPE for the previously requested transaction via CCPLEX proxy driver.
 *
 * Fields:
 * data:	All the response is stored in this buf.
 */
struct aon_shub_xfer_response {
	u8 data[AON_SHUB_MAX_DATA_SIZE - (2 * sizeof(u32))];
};

/* This structure indicates the current SHUB request from CCPLEX to SPE for the
 * AON SHUB controller.
 *
 * Fields:
 * req_type:	Indicates the type of request.
 * data:	Union of structs of all the request types.
 */
struct aon_shub_request {
	/* enum aon_shub_request_type */
	u32 req_type;
	union {
		struct aon_shub_init_request init;
		struct aon_shub_sys_request sys;
		struct aon_shub_snsrcfg_request cfg;
		struct aon_shub_enable_request enable;
		struct aon_shub_batch_request batch;
		struct aon_shub_flush_request flush;
	} data;
};

/* This structure indicates the response for the SHUB request from SPE to CCPLEX
 * for the AON SHUB controller.
 *
 * Fields:
 * status:	Response in regard to the request i.e success/failure.
 * data:	Union of structs of all the response types.
 */
struct aon_shub_response {
	u32 status;
	/* enum aon_shub_request_type */
	u32 resp_type;
	union {
		struct aon_shub_init_response init;
		struct aon_shub_sys_response sys;
		struct aon_shub_snsrcfg_response cfg;
		struct aon_shub_xfer_response xfer;
		struct aon_shub_enable_response enable;
		struct aon_shub_payload_response payload;
	} data;
};

#endif
