/*
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _ABI_BPMP_ABI_H_
#define _ABI_BPMP_ABI_H_

#ifdef LK
#include <stdint.h>
#endif

#ifndef __ABI_PACKED
#define __ABI_PACKED __attribute__((packed))
#endif

#ifdef NO_GCC_EXTENSIONS
#define EMPTY char empty;
#define EMPTY_ARRAY 1
#else
#define EMPTY
#define EMPTY_ARRAY 0
#endif

/**
 * This header file documents the Application Binary Interface (ABI)
 * of BPMP processor complex
 *
 * Table of Contents
 *  1. Message format
 *  2. The Message Request (MRQ) codes
 *  3. Message payloads
 *   3.0 Ping message (MRQ_PING)
 *   3.1 TAG query (MRQ_QUERY_TAG)
 *   3.2 Do idle (MRQ_DO_IDLE)
 *   3.3 Tolerate idle (MRQ_TOLERATE_IDLE)
 *   3.4 Module load (MRQ_MODULE_LOAD)
 *   3.5 Module unload (MRQ_MODULE_UNLOAD)
 *   3.6 Switch cluster (MRQ_SWITCH_CLUSTER)
 *   3.7 Trace modify (MRQ_TRACE_MODIFY)
 *   3.8 Write trace (MRQ_WRITE_TRACE)
 *   3.9 Threaded ping (MRQ_THREADED_PING)
 *   3.10 Cpuidle usage (MRQ_CPUIDLE_USAGE)
 *   3.11 Module mail (MRQ_MODULE_MAIL)
 *   3.12 SCx enable (MRQ_SCX_ENABLE)
 *   3.13 Notify frozen CPUs (MRQ_CPU_FROZEN)
 *   3.14 BPMP idle usage (BRQ_BPMPIDLE_USAGE)
 *   3.15 Heap usage (MRQ_HEAP_USAGE)
 *   3.16 SCLK skip set rate (MRQ_SCLK_SKIP_SET_RATE)
 *   3.17 Enable suspend (MRQ_ENABLE_SUSPEND)
 *   3.18 PASR mask (MRQ_PASR_MASK)
 *   3.19 Debugfs (MRQ_DEBUGFS)
 *   3.20 Reset (MRQ_RESET)
 *   3.21 I2C (MRQ_I2C)
 *   3.22 Clock (MRQ_CLK)
 *   3.23 ABI Query (MRQ_QUERY_ABI)
 *   3.24 MC Flush (MRQ_MC_FLUSH)
 *   3.25 PG read state (MRQ_PG_READ_STATE)
 *   3.26 PG update state (MRQ_PG_UPDATE_STATE)
 *   3.27 Thermal (MRQ_THERMAL)
 *   3.28 CPU DVFS voltage hint (MRQ_CPU_VHINT)
 *   3.29 ABI ratchet (MRQ_ABI_RATCHET)
 *   3.30 Reset IPC channel (MRQ_IPC_RESET)
 *   3.31 EMC DVFS Latency (MRQ_EMC_DVFS_LATENCY)
 *   3.32 Waypoint2 (MRQ_SC7_WAYPOINT2)
 *   3.33 Start deep sleep entry (MRQ_SC7_ENTRY_START)
 *   3.34 MRQ_SC7_ENTRY_READY
 *   3.35 CC7 entry (MRQ_CC7_ENTRY)
 *   3.36 CC7 exit (MRQ_CC7_EXIT)
 *   3.37 MRQ_CC7_EXIT_DONE
 *   3.38 CCP3 state entry (MRQ_CCP3_ENTRY)
 *   3.39 CCP3 state exit (MRQ_CCP3_EXIT)
 *   3.40 SCx entry (MRQ_SCX_ENTRY)
 *   3.41 SCx exit (MRQ_SCX_EXIT)
 *   3.42 SCx exit done (MRQ_SCX_EXIT_DONE)
 *   3.43 SCx wake (MRQ_SCX_WAKE)
 *   3.44 SiMon enable (MRQ_SIMON_ENABLE)
 *   3.45 AVFS init (MRQ_AVFS_INIT)
 *   3.46 AVFS frequency caps (MRQ_AVFS_FREQ_CAPS)
 *   3.47 AVFS temp index (MRQ_AVFS_TEMP_IDX)
 *   3.48 firmware booted (MRQ_FW_BOOTED)
 *   3.49 System shutdown (MRQ_SHUTDOWN)
 *   3.50 EDVD integral check (MRQ_EDVD_CHECK)
 *  4. Enumerations
 *   4.1 CPU enumerations
 *   4.2 CPU Cluster enumerations
 *   4.3 System low power states enumerations
 *   4.4 Clock enumerations
 *   4.5 Reset enumerations
 *   4.6 Thermal sensor enumerations
 *   4.7 Power partition enumerations
 *  5. Error codes
 */

/**
 * 1. Message format
 *
 * 1.1 Request messages
 *  Every request has a common header (first 8 bytes) consisting of
 *  32bit MRQ code and 32bit flags fields. The bytes following are the
 *  actual MRQ specific payload.
 *
 */
struct mrq_request {
	uint32_t mrq;
	uint32_t flags;
	/* MRQ specific request payload begins starts here */
} __ABI_PACKED;

/**
 * 1.2 Response messages
 *
 *  Each response message has a common header (first 8 bytes)
 *  consiting of 32bit return value and 32bit flags field. The
 *  following bytes are message specific payload
 */
struct mrq_response {
	int32_t err;
	uint32_t flags;
	/* MRQ specific response payload begins here */
} __ABI_PACKED;

/**
 * Minimum needed size for message buffers
 */
#define MSG_MIN_SZ	128
#define MSG_DATA_MIN_SZ	120

/**
 * 2. The Message Request (MRQ) codes
 *
 *  Each service request that BPMP receives or sends is identified
 *  with MRQ code. The MRQ is the first 32bit integer of the IPC
 *  message.
 */
#define MRQ_PING		0
#define MRQ_QUERY_TAG		1
#define MRQ_DO_IDLE		2
#define MRQ_TOLERATE_IDLE	3
#define MRQ_MODULE_LOAD		4
#define MRQ_MODULE_UNLOAD	5
#define MRQ_SWITCH_CLUSTER	6
#define MRQ_TRACE_MODIFY	7
#define MRQ_WRITE_TRACE		8
#define MRQ_THREADED_PING	9
#define MRQ_CPUIDLE_USAGE	10
#define MRQ_MODULE_MAIL		11
#define MRQ_SCX_ENABLE		12
#define MRQ_CPU_FROZEN		13
#define MRQ_BPMPIDLE_USAGE	14
#define MRQ_HEAP_USAGE		15
#define MRQ_SCLK_SKIP_SET_RATE	16
#define MRQ_ENABLE_SUSPEND	17
#define MRQ_PASR_MASK		18
#define MRQ_DEBUGFS		19
#define MRQ_RESET		20
#define MRQ_I2C			21
#define MRQ_CLK			22
#define MRQ_QUERY_ABI		23
#define MRQ_MC_FLUSH		24
#define MRQ_PG_READ_STATE	25
#define MRQ_PG_UPDATE_STATE	26
#define MRQ_THERMAL		27
#define MRQ_CPU_VHINT		28
#define MRQ_ABI_RATCHET		29
#define MRQ_IPC_RESET		30

#define MRQ_EMC_DVFS_LATENCY	31
#define MRQ_SC7_WAYPOINT2	32
#define MRQ_SC7_ENTRY_START	33
#define MRQ_SC7_ENTRY_READY	34
#define MRQ_CC7_ENTRY		35
#define MRQ_CC7_EXIT		36
#define MRQ_CC7_EXIT_DONE	37
#define MRQ_CCP3_ENTRY		38
#define MRQ_CCP3_EXIT		39
#define MRQ_SCX_ENTRY		40
#define MRQ_SCX_EXIT		41
#define MRQ_SCX_EXIT_DONE	42
#define MRQ_SCX_WAKE		43
#define MRQ_SIMON_ENABLE	44
#define MRQ_AVFS_INIT		45
#define MRQ_AVFS_FREQ_CAPS	46
#define MRQ_AVFS_TEMP_IDX	47
#define MRQ_FW_BOOTED		48
#define MRQ_SHUTDOWN		49
#define MRQ_EDVD_CHECK		50
#define MRQ_DMCE_51		51
#define MRQ_DMCE_52		52
#define MRQ_DMCE_53		53
#define MRQ_DMCE_54		54
#define MRQ_DMCE_55		55
#define MRQ_DMCE_56		56
#define MRQ_DMCE_57		57
#define MRQ_DMCE_58		58
#define MRQ_DMCE_59		59
#define MRQ_DMCE_60		60
#define MRQ_DMCE_61		61
#define MRQ_DMCE_62		62
#define MRQ_DMCE_63		63

/**
 * 3. Message payloads
 *
 *  This section describes the payload of each message type
 */


/**
 * 3.0 Ping message (MRQ_PING)
 *
 * Platforms: All
 * Initiators: Any
 * Targets: Any
 *
 */

/**
 * struct mrq_ping_request
 * @challenge: arbitrarily chosen value
 *
 * Used by the sender of an %MRQ_PING message to request a pong from
 * recipient. The response from the recipient is computed based on
 * @challenge.
 *
 */
struct mrq_ping_request
{
	uint32_t challenge;
} __ABI_PACKED;

/**
 * struct mrq_ping_response
 * @reply: response to the MRQ_PING challege
 *
 * Sent in response to an %MRQ_PING message. @reply should be the
 * mrq_ping_request challenge left shifted by 1 with the carry-bit
 * dropped.
 *
 */
struct mrq_ping_response
{
	uint32_t reply;
} __ABI_PACKED;

/**
 * 3.1 TAG query (MRQ_QUERY_TAG)
 *
 * Platforms: All
 * Initiators: CCPLEX
 * Targets: BPMP
 */

/**
 *  struct mrq_query_tag_request
 * @addr: base address to store the firmware header
 *
 * Used by %MRQ_QUERY_TAG call to ask BPMP to fill in the memory
 * pointed by @addr with BPMP firmware header
 *
 * The sender is reponsible for ensuring that @addr is mapped in to
 * the recipients address map
 */
struct mrq_query_tag_request
{
	uint32_t addr;
} __ABI_PACKED;

/**
 * There is no payload data to be sent as a response for
 * %MRQ_QUERY_TAG request.
 */

/**
 * 3.2 Do idle (MRQ_DO_IDLE)
 *
 * Platforms: T210
 * Initiators: CCPLEX
 * Targets: BPMP
 *
 * This is T210 specific call
 */

/**
 * 3.3 Tolerate idle (MRQ_TOLERATE_IDLE)
 *
 * Platforms: T210
 * Initiators: CCPLEX
 * Targets: BPMP
 *
 * This is T210 specific call
 */

/**
 * 3.4 Module load (MRQ_MODULE_LOAD)
 *
 * Platforms: All
 * Initiators: CCPLEX
 * Targets: BPMP
 */

/**
 * struct mrq_module_load_request
 * @phys_addr: base address of the code to load
 * @size: size in bytes of code to load
 *
 * Used by %MRQ_MODULE_LOAD calls to ask the recipient to dynamically
 * load the code located at @phys_addr and having size @size
 * bytes. @phys_addr is treated as a void pointer.
 *
 * The recipient copies the code from @phys_addr to locally allocated
 * memory prior to responding to this message.
 *
 * TODO: document the module header format
 * TODO: warn about the security implications of supporting this call
 *
 * The sender is responsible for ensuring that the code is mapped in
 * the recipient's address map.
 *
 */
struct mrq_module_load_request
{
	uint32_t phys_addr;/* (void *) */
	uint32_t size;
} __ABI_PACKED;

/**
 * struct mrq_module_load_response
 * @base: handle to the loaded module
 *
 * TODO: document reply_code
 *
 */
struct mrq_module_load_response
{
	uint32_t base;
} __ABI_PACKED;

/**
 * 3.5 Module unload (MRQ_MODULE_UNLOAD)
 *
 * Platforms: All
 * Initiators: CCPLEX
 * Targets: BPMP
 */

/**
 * struct mrq_module_unload_request
 * @base: handle of the module to unload
 *
 * Used by MRQ_MODULE_UNLOAD calls to request that a previously loaded
 * module be unloaded.
 */
struct mrq_module_unload_request
{
	uint32_t base;
} __ABI_PACKED;

/**
 * 3.6 Switch cluster (MRQ_SWITCH_CLUSTER)
 *
 * Platforms: T210
 * Initiators: CCPLEX
 * Targets: BPMP
 *
 * This is T210 specific call
 */

/**
 * 3.7 Trace modify (MRQ_TRACE_MODIFY)
 *
 * Platforms: All
 * Initiators: CCPLEX
 * Targets: BPMP
 */

/**
 * struct mrq_trace_modify_request
 * @clr: bit mask of trace events to disable
 * @set: bit mask of trace events to enable
 *
 * Used by %MRQ_TRACE_MODIFY calls to enable or disable specify trace events.
 * @set takes precedence for any bit set in both @set and @clr.
 */
struct mrq_trace_modify_request
{
	uint32_t clr;
	uint32_t set;
} __ABI_PACKED;

/**
 * struct mrq_trace_modify_response
 * @mask: bit mask of trace event enable states
 *
 * Sent in repsonse to an %MRQ_TRACE_MODIFY message. @mask reflects the
 * state of which events are enabled after the recipient acted on the
 * message.
 *
 */
struct mrq_trace_modify_response
{
	uint32_t mask;
} __ABI_PACKED;

/**
 * 3.8 Write trace (MRQ_WRITE_TRACE)
 *
 * Platforms: All
 * Initiators: CCPLEX
 * Targets: BPMP
 */

/**
 * struct mrq_write_trace_request
 * @area: base address of output buffer
 * @size: size in bytes of the output buffer
 *
 * Used by %MRQ_WRITE_TRACE calls to ask the recipient to copy trace
 * data from the recipient's local buffer to the output buffer. @area
 * is treated as a byte-aligned pointer in the recipient's address
 * space.
 *
 * The sender is responsible for ensuring that the output
 * buffer is mapped in the recipient's address map. The recipient is
 * responsible for protecting its own code and data from accidental
 * overwrites.
 *
 */
struct mrq_write_trace_request
{
	uint32_t area; /* (char *) */
	uint32_t size;
} __ABI_PACKED;

/**
 * struct mrq_write_trace_response
 * @eof: flag whether any more data remains in local buffer
 *
 * Sent in response to an %MRQ_WRITE_TRACE message. @eof is 1 if the
 * entire local trace buffer has been drained to the output
 * buffer. @eof is 0 otherwise.
 *
 * Once this response is sent, the respondent will not access the
 * output buffer further.
 *
 * Reply_code is %-EINVAL if @size is zero or @area is NULL or @area
 * is in an illegal range. A positive value for reply_code indicates
 * the number of bytes written to @area.
 *
 */
struct mrq_write_trace_response
{
	uint32_t eof;
} __ABI_PACKED;

/**
 * 3.9 Threaded ping (MRQ_THREADED_PING)
 *
 * Platforms: All
 * Initiators: Any
 * Targets: BPMP
 */

struct mrq_threaded_ping_request
{
	uint32_t challenge;
} __ABI_PACKED;

struct mrq_threaded_ping_response
{
	uint32_t reply;
} __ABI_PACKED;

/**
 * 3.10 Cpuidle usage (MRQ_CPUIDLE_USAGE)
 *
 * Platforms: All
 * Initiators: Any
 * Targets: BPMP
 *
 * Reserved MRQ for internal use.
 */

/* TODO */

/**
 * 3.11 Module mail (MRQ_MODULE_MAIL)
 *
 * Platforms: All
 * Initiators: Any
 * Targets: BPMP
 *
 */

struct mrq_module_mail_request
{
	uint32_t base;
	uint8_t data[EMPTY_ARRAY]; /* module mail specific payload */
} __ABI_PACKED;

struct mrq_module_mail_response
{
	uint8_t data[EMPTY_ARRAY]; /* module mail specific response */
} __ABI_PACKED;

/**
 * 3.12 SCx enable (MRQ_SCX_ENABLE)
 *
 * Platforms: T210
 * Initiators: CCPLEX
 * Targets: BPMP
 *
 * This is T210 specific call
 */

/**
 * 3.13 Notify frozen CPUs (MRQ_CPU_FROZEN)
 *
 * Platforms: T210
 * Initiators: CCPLEX
 * Targets: BPMP
 *
 * Inform BPMP about the CPUs that are being frozen
 * as part of the System suspend process.
 *
 * This is T210 specific call
 */

/**
 * 3.14 BPMP idle usage (MRQ_BPMPIDLE_USAGE)
 *
 * Platforms: All
 * Initiators: Any
 * Targets: BPMP
 *
 * Reserved MRQ for internal use.
 */

struct mrq_bpmpidle_usage_request
{
	uint32_t state;
} __ABI_PACKED;

struct mrq_bpmpidle_usage_response
{
	uint64_t count;
	uint64_t time;
} __ABI_PACKED;

/**
 * 3.15 Heap usage (MRQ_HEAP_USAGE)
 *
 * Platforms: All
 * Initiators: Any
 * Targets: BPMP
 */

struct mrq_heap_usage_request
{
	EMPTY
} __ABI_PACKED;

struct mrq_heap_usage_response
{
	uint32_t heap_start; /* void * */
	uint32_t heap_len;
	uint32_t heap_free;
	uint32_t heap_max_chunk;
	uint32_t heap_low_watermark;
} __ABI_PACKED;

/**
 * 3.16 SCLK skip set rate (MRQ_SCLK_SKIP_SET_RATE)
 *
 * Platforms: T210
 * Initiators: CCPLEX
 * Targets: BPMP
 *
 * This is T210 specific call
 */

/**
 * 3.17 Enable suspend (MRQ_ENABLE_SUSPEND)
 *
 * Platforms: T210
 * Initiators: CCPLEX
 * Targets: BPMP
 *
 * This is T210 specific call
 */

/**
 * 3.18 PASR mask (MRQ_PASR_MASK)
 *
 * Platforms: T210
 * Initiators: CCPLEX
 * Targets: BPMP
 *
 * This is T210 specific call
 */

/**
 * 3.19 Debugfs (MRQ_DEBUGFS)
 *
 * Platforms: T186
 * Initiators: Any
 * Targets: BPMP
 */
enum mrq_debugfs_commands {
	CMD_DEBUGFS_READ = 1,
	CMD_DEBUGFS_WRITE = 2,
	CMD_DEBUGFS_DUMPDIR = 3,
	CMD_DEBUGFS_MAX
};

struct cmd_debugfs_fileop_request {
	uint32_t fnameaddr;
	uint32_t fnamelen;
	uint32_t dataaddr;
	uint32_t datalen;
} __ABI_PACKED;

struct cmd_debugfs_dumpdir_request {
	uint32_t dataaddr;
	uint32_t datalen;
} __ABI_PACKED;

struct cmd_debugfs_fileop_response {
	uint32_t reserved; /* always 0 */
	uint32_t nbytes;
} __ABI_PACKED;

struct cmd_debugfs_dumpdir_response {
	uint32_t reserved; /* always 0 */
	uint32_t nbytes;
} __ABI_PACKED;

/**
 * struct mrq_debugfs_request
 * @cmd: debugfs command
 * @fop: parameters for CMD_DEBUGFS_READ or CMD_DEBUGFS_WRITE command
 * @dumpdir: parameters for CMD_DEBUGFS_DUMPDIR
 */
struct mrq_debugfs_request {
	uint32_t cmd; /* enum mrq_debugfs_commands */
	union {
		struct cmd_debugfs_fileop_request fop;
		struct cmd_debugfs_dumpdir_request dumpdir;
	};
} __ABI_PACKED;

struct mrq_debugfs_response {
	int32_t reserved; /* always 0 */
	union {
		struct cmd_debugfs_fileop_response fop;
		struct cmd_debugfs_dumpdir_response dumpdir;
	};
} __ABI_PACKED;

/**
 * Debugfs enumerations
 */
#define DEBUGFS_S_ISDIR	(1 << 9)
#define DEBUGFS_S_IRUSR	(1 << 8)
#define DEBUGFS_S_IWUSR	(1 << 7)

/**
 * 3.20 Reset (MRQ_RESET)
 *
 * Platforms: T186
 * Initiators: Any
 * Targets: BPMP
 *
 */
enum mrq_reset_commands {
	CMD_RESET_ASSERT = 1,
	CMD_RESET_DEASSERT = 2,
	CMD_RESET_MODULE = 3,
	CMD_RESET_MAX, /* not part of ABI and subject to change */
};

/**
 * struct mrq_reset_request
 * @cmd: reset action to perform (enum mrq_reset_commands)
 * @reset_id: reset id
 *
 * Used by the sender of an %MRQ_RESET message to request BPMP to
 * assert or or deassert a given reset line.
 *
 */

struct mrq_reset_request {
	uint32_t cmd;
	uint32_t reset_id;
} __ABI_PACKED;

/**
 * There is no payload data to be sent as a response for
 * %MRQ_RESET request.
 */

/**
 * 3.21 I2C (MRQ_I2C)
 *
 * Platforms: T186
 * Initiators: Any
 * Targets: BPMP
 *
 * TODO
 *
 *   The serialized I2C format is following:
 *  [addr little-endian][flags little-endian][len little-endian][data if write]
 *  [addr little-endian][flags little-endian][len little-endian][data if write]
 *  ...
 *    addr, flags, and len are uint16_t, data is uint8_t[]
 *
 *  The flags are translated from Linux kernel representation to seriali2c
 *  representation. Any undefined flag being set causes an error.
 *
 *  The data is there only for writes. Reads have the data transferred in the
 *  other direction, and thus data is not present.
 */

#define TEGRA_I2C_IPC_MAX_IN_BUF_SIZE	(MSG_DATA_MIN_SZ - 12)
#define TEGRA_I2C_IPC_MAX_OUT_BUF_SIZE	(MSG_DATA_MIN_SZ - 4)

/* definition for I2C flags */
#define SERIALI2C_TEN           0x0010
#define SERIALI2C_RD            0x0001
#define SERIALI2C_STOP          0x8000
#define SERIALI2C_NOSTART       0x4000
#define SERIALI2C_REV_DIR_ADDR  0x2000
#define SERIALI2C_IGNORE_NAK    0x1000
#define SERIALI2C_NO_RD_ACK     0x0800
#define SERIALI2C_RECV_LEN      0x0400

/* I2C sub-MRQ commands */
enum {
	CMD_I2C_XFER = 1
};

/**
 * struct cmd_i2c_xfer_request
 *
 * TODO
 */
struct cmd_i2c_xfer_request {
	uint32_t bus_id;
	uint32_t data_size;
	uint8_t data_buf[TEGRA_I2C_IPC_MAX_IN_BUF_SIZE];
} __ABI_PACKED;

/**
 * struct cmd_i2c_xfer_response
 *
 * TODO
 */
struct cmd_i2c_xfer_response {
	uint32_t data_size;
	uint8_t data_buf[TEGRA_I2C_IPC_MAX_OUT_BUF_SIZE];
} __ABI_PACKED;

/**
 * struct mrq_i2c_request
 * @cmd: I2C sub MRQ command
 *
 * TODO
 */
struct mrq_i2c_request
{
	uint32_t cmd;
	struct cmd_i2c_xfer_request xfer;
} __ABI_PACKED;

/**
 * struct mrq_i2c_response
 *
 * TODO
 */
struct mrq_i2c_response
{
	struct cmd_i2c_xfer_response xfer;
} __ABI_PACKED;

/**
 * 3.22 Clock (MRQ_CLK)
 *
 * Platforms: T186
 * Initiators: Any
 * Targets: BPMP
 */
enum {
        CMD_CLK_GET_RATE = 1,
        CMD_CLK_SET_RATE = 2,
        CMD_CLK_ROUND_RATE = 3,
        CMD_CLK_GET_PARENT = 4,
        CMD_CLK_SET_PARENT = 5,
        CMD_CLK_IS_ENABLED = 6,
        CMD_CLK_ENABLE = 7,
        CMD_CLK_DISABLE = 8,
        CMD_CLK_PROPERTIES = 9,
        CMD_CLK_POSSIBLE_PARENTS = 10,
        CMD_CLK_NUM_POSSIBLE_PARENTS = 11,
        CMD_CLK_GET_POSSIBLE_PARENT = 12,
	CMD_CLK_RESET_REFCOUNTS = 13,
	CMD_CLK_GET_ALL_INFO = 14,
	CMD_CLK_GET_MAX_CLK_ID = 15,
        CMD_CLK_MAX,
};

#define MRQ_CLK_NAME_MAXLEN	40
#define MRQ_CLK_MAX_PARENTS	16

struct cmd_clk_get_rate_request {
	EMPTY
} __ABI_PACKED;

struct cmd_clk_get_rate_response {
	int64_t rate;
} __ABI_PACKED;

struct cmd_clk_set_rate_request {
	int32_t unused;
	int64_t rate;
} __ABI_PACKED;

struct cmd_clk_set_rate_response {
	int64_t rate;
} __ABI_PACKED;

struct cmd_clk_round_rate_request {
	int32_t unused;
	int64_t rate;
} __ABI_PACKED;

struct cmd_clk_round_rate_response {
	int64_t rate;
} __ABI_PACKED;

struct cmd_clk_get_parent_request {
	EMPTY
} __ABI_PACKED;

struct cmd_clk_get_parent_response {
	uint32_t parent_id;
} __ABI_PACKED;

struct cmd_clk_set_parent_request {
	uint32_t parent_id;
} __ABI_PACKED;

struct cmd_clk_set_parent_response {
	uint32_t parent_id;
} __ABI_PACKED;

struct cmd_clk_is_enabled_request {
	EMPTY
} __ABI_PACKED;

struct cmd_clk_is_enabled_response {
	int32_t state;
} __ABI_PACKED;

struct cmd_clk_enable_request {
	EMPTY
} __ABI_PACKED;

struct cmd_clk_enable_response {
	EMPTY
} __ABI_PACKED;

struct cmd_clk_disable_request {
	EMPTY
} __ABI_PACKED;

struct cmd_clk_disable_response {
	EMPTY
} __ABI_PACKED;

struct cmd_clk_properties_request {
	EMPTY
} __ABI_PACKED;

/* TODO: flags need to be spelled out here */
struct cmd_clk_properties_response {
	uint32_t flags;
} __ABI_PACKED;

struct cmd_clk_possible_parents_request {
	EMPTY
} __ABI_PACKED;

struct cmd_clk_possible_parents_response {
	uint8_t num_parents;
	uint8_t reserved[3];
	uint32_t parent_id[MRQ_CLK_MAX_PARENTS];
} __ABI_PACKED;

struct cmd_clk_num_possible_parents_request {
	EMPTY
} __ABI_PACKED;

struct cmd_clk_num_possible_parents_response {
	uint8_t num_parents;
} __ABI_PACKED;

struct cmd_clk_get_possible_parent_request {
	uint8_t parent_idx;
} __ABI_PACKED;

struct cmd_clk_get_possible_parent_response {
	uint32_t parent_id;
} __ABI_PACKED;

struct cmd_clk_reset_refcounts {
	EMPTY
} __ABI_PACKED;

struct cmd_clk_get_all_info_request {
	EMPTY
} __ABI_PACKED;

struct cmd_clk_get_all_info_response {
	uint32_t flags;
	uint32_t parent;
	uint32_t parents[MRQ_CLK_MAX_PARENTS];
	uint8_t num_parents;
	uint8_t name[MRQ_CLK_NAME_MAXLEN];
} __ABI_PACKED;

struct cmd_clk_get_max_clk_id_request {
	EMPTY
} __ABI_PACKED;

struct cmd_clk_get_max_clk_id_response {
	uint32_t max_id;
} __ABI_PACKED;


/**
 * struct mrq_clk_request
 * @cmd_and_id: Clock sub-cmd and clock id concatenated to 32bit word
 *   -bits[31..24] is the sub-cmd
 *   -bits[23..0] is the clock id
 * @data: sub-cmd specific data
 *
 *
 * Used by the sender of an %MRQ_CLK message to control clocks. The
 * clk_request is split into several sub-commands each of them having
 * their own data packet format:
 *  - get_rate
 *  - set_rate
 *  - round_rate
 *  - get_parent
 *  - set_parent
 *  - enable
 *  - disable
 *  - properties
 *  - num_possible_parents
 *  - get_possible_parent
 *
 */

struct mrq_clk_request {
	uint32_t cmd_and_id;
	union {
		struct cmd_clk_get_rate_request clk_get_rate;
		struct cmd_clk_set_rate_request clk_set_rate;
		struct cmd_clk_round_rate_request clk_round_rate;
		struct cmd_clk_get_parent_request clk_get_parent;
		struct cmd_clk_set_parent_request clk_set_parent;
		struct cmd_clk_enable_request clk_enable;
		struct cmd_clk_disable_request clk_disable;
		struct cmd_clk_is_enabled_request clk_is_enabled;
		struct cmd_clk_properties_request clk_properties;
		struct cmd_clk_possible_parents_request clk_possible_parents;
		struct cmd_clk_num_possible_parents_request clk_num_possible_parents;
		struct cmd_clk_get_possible_parent_request clk_get_possible_parent;
		struct cmd_clk_get_all_info_request clk_get_all_info;
		struct cmd_clk_get_max_clk_id_request clk_get_max_clk_id;
	};
} __ABI_PACKED;

/**
 * struct mrq_clk_response
 * @data: sub-cmd specific response packet
 *
 */

struct mrq_clk_response {
	union {
		struct cmd_clk_get_rate_response clk_get_rate;
		struct cmd_clk_set_rate_response clk_set_rate;
		struct cmd_clk_round_rate_response clk_round_rate;
		struct cmd_clk_get_parent_response clk_get_parent;
		struct cmd_clk_set_parent_response clk_set_parent;
		struct cmd_clk_enable_response clk_enable;
		struct cmd_clk_disable_response clk_disable;
		struct cmd_clk_is_enabled_response clk_is_enabled;
		struct cmd_clk_properties_response clk_properties;
		struct cmd_clk_possible_parents_response clk_possible_parents;
		struct cmd_clk_num_possible_parents_response clk_num_possible_parents;
		struct cmd_clk_get_possible_parent_response clk_get_possible_parent;
		struct cmd_clk_get_all_info_response clk_get_all_info;
		struct cmd_clk_get_max_clk_id_response clk_get_max_clk_id;
	};
} __ABI_PACKED;

/**
 * 3.23 ABI Query (MRQ_QUERY_ABI)
 *
 * Platforms: All
 * Initiators: Any
 * Targets: Any
 */

/**
 * struct mrq_query_abi_request
 * @mrq: MRQ code of a message to query
 *
 * Used by %MRQ_QUERY_ABI call to check if MRQ code @mrq is supported
 * by the recipient.
 */

struct mrq_query_abi_request {
	uint32_t mrq;
} __ABI_PACKED;

/**
 * struct mrq_query_abi_response
 * @status: return value
 *
 * Response to %MRQ_QUERY_ABI message. @status is 0 if requested MRQ
 * is supported, otherwise @status is -BPMP_ENODEV
 */
struct mrq_query_abi_response {
	int32_t status;
} __ABI_PACKED;


/**
 * 3.24 MC Flush (MRQ_MC_FLUSH)
 *
 * Platforms: T186
 * Initiators: Any
 * Targets: BPMP
 *
 */

struct mrq_mc_flush_request
{
	uint32_t mc_client_id;
	uint32_t request; /* enum mc_flush_req */
};

enum mc_flush_req {
	REQ_MC_FLUSH_START = 1,
	REQ_MC_FLUSH_DONE = 2,
};

/**
 * 3.25 PG read state (MRQ_PG_READ_STATE)
 *
 * Platforms: T186
 * Initiators: Any
 * Targets: BPMP
 */

/**
 * struct mrq_pg_read_state_request
 * @partition_id: ID of partition
 *
 * Used by %MRQ_PG_READ_STATE call to read the current state of a
 * partition.
 */
struct mrq_pg_read_state_request {
	uint32_t partition_id;
} __ABI_PACKED;

/**
 * struct mrq_pg_read_state_response
 * @sram_state: state of SRAM
 *    BIT[1:0] = 1 : SD, 2 : SLP, 4 : DSLP
 * @logic_state: state of logic (0 = Off, 1 = On)
 *
 * Response to %MRQ_PG_READ_STATE_REQUEST message. @status is 0 upon
 * success.  TODO/ARB Prashant to define possible errors.
 */
struct mrq_pg_read_state_response {
	uint32_t sram_state;
	uint32_t logic_state;
} __ABI_PACKED;

/**
 * 3.26 PG update state (MRQ_PG_UPDATE_STATE)
 *
 * Platforms: T186
 * Initiators: Any
 * Targets: BPMP
 */

/**
 * struct mrq_pg_update_state_request
 * @partition_id: ID of partition
 * @sram_state: new sram state
 *    BIT[0] = 0 : do not change SRAM state, 1 : change SRAM state
 *    BIT[3:1] = 0 : Active, 1 : SD, 2 : SLP, 4 : DSLP
 * @logic_state: new logic state
 *    BIT[0] = 0 : do not change SRAM state, 1 : change SRAM state
 *    BIT[1] = 0 : Power off, 1 : Power on
 * @clock_state: new clock state
 *    BIT[0] = 0 : do not change clock state, 1 : change clock state
 *    BIT[1] = 0 : Turn off clocks, 1 : Leave clocks on
 *
 * Used by %MRQ_PG_UPDATE_STATE call to request BPMP to change the
 * state of a power partition @partition_id. The new state to enter is
 * defined with @sram_state, @logic_state, and @clock_state.
 */
struct mrq_pg_update_state_request {
	uint32_t partition_id;
	uint32_t sram_state;
	uint32_t logic_state;
	uint32_t clock_state;
} __ABI_PACKED;

/**
 * BPMP responds to %MRQ_PG_UPDATE_STATE_REQUEST message without
 * additional paylaod data.
 */

/**
 *   3.27 Thermal (MRQ_THERMAL)
 *
 * Platforms: T186
 * Initiators: Any
 * Targets: BPMP
 */
enum mrq_thermal_host_to_bpmp_cmd {
	/*
	 * CMD_THERMAL_QUERY_ABI:
	 *   Check whether the BPMP driver supports the specified request type.
	 *
	 * Host needs to supply request parameters.
	 *
	 * Reply codes:
	 *   0		Specified request type is supported.
	 *   -BPMP_ENODEV	Specified request type is not supported.
	 */
	CMD_THERMAL_QUERY_ABI = 0,

	/*
	 * CMD_THERMAL_GET_TEMP:
	 *   Get the current temperature of the specified zone.
	 *
	 * Host needs to supply request parameters.
	 *
	 * Reply codes:
	 *   0		Temperature query succeeded.
	 *   -BPMP_EINVAL	Invalid request parameters.
	 *   -BPMP_ENOENT	No driver registered for thermal zone..
	 *   -BPMP_EFAULT	Problem reading temperature measurement.
	 */
	CMD_THERMAL_GET_TEMP = 1,

	/*
	 * CMD_THERMAL_SET_TRIP:
	 *   Enable or disable and set the lower and upper thermal limits
	 *   for a thermal trip point. Each zone has one trip point.
	 *
	 * Host needs to supply request parameters. Once the
	 * temperature hits a trip point, the BPMP will send a message
	 * to the CPU having MRQ=MRQ_THERMAL and
	 * type=CMD_THERMAL_HOST_TRIP_REACHED
	 *
	 * Reply codes:
	 *   0		Trip succesfully set.
	 *   -BPMP_EINVAL	Invalid request parameters.
	 *   -BPMP_ENOENT	No driver registered for thermal zone.
	 *   -BPMP_EFAULT	Problem setting trip point.
	 */
	CMD_THERMAL_SET_TRIP = 2,

	/*
	 * CMD_THERMAL_GET_NUM_ZONES
	 *   Get the number of supported thermal zones.
	 *
	 * No request parameters required.
	 *
	 * Reply codes:
	 *   0		Query succeeded.
	 */
	CMD_THERMAL_GET_NUM_ZONES = 3,

	CMD_THERMAL_HOST_TO_BPMP_NUM
};

enum mrq_thermal_bpmp_to_host_cmd {
	/*
	 * CMD_THERMAL_HOST_TRIP_REACHED:
	 *   Indication that the temperature for a zone has exceeded the
	 *   range indicated in the thermal trip point for the zone.
	 *
	 * BPMP needs to supply request parameters. Host only needs to
	 * acknowledge.
	 */
	CMD_THERMAL_HOST_TRIP_REACHED = 100,

	CMD_THERMAL_BPMP_TO_HOST_NUM
};

/*
 * Host->BPMP request data for request type CMD_THERMAL_QUERY_ABI
 *
 * zone: Request type for which to check existence.
 */
struct cmd_thermal_query_abi_request {
	uint32_t type;
} __ABI_PACKED;

/*
 * Host->BPMP request data for request type CMD_THERMAL_GET_TEMP
 *
 * zone: Number of thermal zone.
 */
struct cmd_thermal_get_temp_request {
	uint32_t zone;
} __ABI_PACKED;

/*
 * BPMP->Host reply data for request CMD_THERMAL_GET_TEMP
 *
 * error: 0 if request succeeded.
 *	-BPMP_EINVAL if request parameters were invalid.
 *      -BPMP_ENOENT if no driver was registered for the specified thermal zone.
 *      -BPMP_EFAULT for other thermal zone driver errors.
 * temp: Current temperature in millicelsius.
 */
struct cmd_thermal_get_temp_response {
	int32_t temp;
} __ABI_PACKED;

/*
 * Host->BPMP request data for request type CMD_THERMAL_SET_TRIP
 *
 * zone: Number of thermal zone.
 * low: Temperature of lower trip point in millicelsius
 * high: Temperature of upper trip point in millicelsius
 * enabled: 1 to enable trip point, 0 to disable trip point
 */
struct cmd_thermal_set_trip_request {
	uint32_t zone;
	int32_t low;
	int32_t high;
	uint32_t enabled;
} __ABI_PACKED;

/*
 * BPMP->Host request data for request type CMD_THERMAL_HOST_TRIP_REACHED
 *
 * zone: Number of thermal zone where trip point was reached.
 */
struct cmd_thermal_host_trip_reached_request {
	uint32_t zone;
} __ABI_PACKED;

/*
 * BPMP->Host reply data for request type CMD_THERMAL_GET_NUM_ZONES
 *
 * num: Number of supported thermal zones. The thermal zones are indexed
 *      starting from zero.
 */
struct cmd_thermal_get_num_zones_response {
	uint32_t num;
} __ABI_PACKED;

/*
 * Host->BPMP request data.
 *
 * Reply type is union mrq_thermal_bpmp_to_host_response.
 *
 * type: Type of request. Values listed in enum mrq_thermal_type.
 * data: Request type specific parameters.
 */
struct mrq_thermal_host_to_bpmp_request {
	uint32_t type;
	union {
		struct cmd_thermal_query_abi_request query_abi;
		struct cmd_thermal_get_temp_request get_temp;
		struct cmd_thermal_set_trip_request set_trip;
	};
} __ABI_PACKED;

/*
 * BPMP->Host request data.
 *
 * type: Type of request. Values listed in enum mrq_thermal_type.
 * data: Request type specific parameters.
 */
struct mrq_thermal_bpmp_to_host_request {
	uint32_t type;
	union {
		struct cmd_thermal_host_trip_reached_request host_trip_reached;
	};
} __ABI_PACKED;

/*
 * Data in reply to a Host->BPMP request.
 */
union mrq_thermal_bpmp_to_host_response {
	struct cmd_thermal_get_temp_response get_temp;
	struct cmd_thermal_get_num_zones_response get_num_zones;
} __ABI_PACKED;

/**
 *   3.28 CPU DVFS voltage hint (MRQ_CPU_VHINT)
 *
 * Platforms: T186
 * Initiators: CCPLEX
 * Targets: BPMP
 */

/**
 * struct mrq_cpu_vhint_request
 * @addr: 32bit address in IOVA pointing to struct cpu_vhint_data
 * @cluster_id: ID of cluster
 *
 * Used by %MRQ_CPU_VHINT call by CCPLEX to retrieve voltage hint data
 * from BPMP to memory space pointed by @addr. CCPLEX is responsible
 * to allocate sizeof(cpu_vhint_data) sized block of memory and
 * appropriately map it for BPMP before sending the request.
 */
struct mrq_cpu_vhint_request {
	uint32_t addr; /* struct cpu_vhint_data * */
	uint32_t cluster_id; /* enum cluster_id */
} __ABI_PACKED;

/**
 * As a response to %MRQ_CPU_VHINT message, BPMP populates the struct
 * cpu_vhint_data table at address @addr.
 */

/**
 * struct cpu_vhint_data
 * @ref_clk_hz: reference clock frequency in Hz
 * @pdiv: post divider value
 * @mdiv: input divider value
 * @ndiv_max: fMAX expressed with max NDIV value
 * @ndiv: table of ndiv values as a function of vINDEX (voltage index)
 * @ndiv_min: minimum allowed NDIV value
 * @vfloor: minimum allowed voltage hint value (as in vINDEX)
 * @vceil: maximum allowed voltage hint value (as in vINDEX)
 * @vindex_mult: post-multiplier for vindex value
 * @vindex_div: post-divider for vindex value
 *
 * Used by %MRQ_CPU_VHINT call to carry data pointed by @addr of
 * struct mrq_cpu_vhint_request
 */
struct cpu_vhint_data {
	uint32_t ref_clk_hz;
	uint16_t pdiv;
	uint16_t mdiv;
	uint16_t ndiv_max;
	uint16_t ndiv[80];
	uint16_t ndiv_min;
	uint16_t vfloor;
	uint16_t vceil;
	uint16_t vindex_mult;
	uint16_t vindex_div;
	uint16_t reserved[328]; /* reserved for future use */
} __ABI_PACKED;


/**
 * 3.29 ABI ratchet (MRQ_ABI_RATCHET)
 *
 * Platforms: T186
 * Initiators: Any
 * Targets: BPMP
 */

/**
 * BPMP_ABI_RATCHET_VALUE may increase for various reasons in a future
 * revision of this header file.
 * 1. That future revision deprecates some MRQ
 * 2. That future revision introduces a breaking change to an existing
 *    MRQ or
 * 3. A bug is discovered in an existing implementation of the BPMP-FW
 *    (or possibly one of its clients) which warrants deprecating that
 *    implementation.
 */
#define BPMP_ABI_RATCHET_VALUE 3

/**
 * struct mrq_abi_ratchet_request
 * @ratchet requester's ratchet value
 *
 * Used by %MRQ_ABI_RATCHET. @ratchet should be
 * %BPMP_ABI_RATCHET_VALUE from the ABI header against which the
 * requester was compiled.
 *
 * If @ratchet is less than BPMP's %BPMP_ABI_RATCHET_VALUE, BPMP may
 * reply with @err = -%BPMP_ERANGE to indicate that BPMP-FW cannot
 * interoperate correctly with the requester. Requester should cease
 * further communication with BPMP.
 *
 * Otherwise, @err must be 0.
 */
struct mrq_abi_ratchet_request {
  uint16_t ratchet;
};

/**
 * struct mrq_abi_ratchet_response
 * @ratchet BPMP's ratchet value
 *
 * Sent in response to %MRQ_ABI_RATCHET. @ratchet is
 * %BPMP_ABI_RATCHET_VALUE from the ABI header against which the
 * BPMP-FW was compiled.
 *
 * If @rachet is less than the requester's %BPMP_ABI_RATCHET_VALUE,
 * the requster must either interoperate with BPMP according to an ABI
 * header version with %BPMP_ABI_RATCHET_VALUE = @ratchet or cease
 * communication with BPMP.
 *
 * If @err is 0 and @ratchet is greater than or equal to the
 * requester's %BPMP_ABI_RATCHET_VALUE, the requester should continue
 * normal operation.
 */
struct mrq_abi_ratchet_response {
  uint16_t ratchet;
};

/*
 * 3.30 Reset IPC channel (MRQ_IPC_RESET)
 *
 * Platforms: T186 onwards
 * Initiators: Anyone except BPMP
 * Targets: BPMP
 *
 * Resets all IPC channels between the initiator and BPMP.  This MRQ is
 * intended to be used when the peer entity is going through a role
 * transition (such as when CCPLEX is finishing execution of bootloader
 * and starting Kernel).
 *
 * @channel: placeholder for extention; must be set to -1
 */
struct mrq_ipc_reset {
	int32_t channel;
} __ABI_PACKED;

/**
 * 3.31 EMC DVFS Latency (MRQ_EMC_DVFS_LATENCY)
 *
 * Platforms: T186
 * Initiators: CCPLEX
 * Targets: BPMP
 */

/**
 * struct emc_dvfs_latency
 * @freq: EMC frequency in Khz
 * @latency: EMC DVFS latency in nano secs
 *
 * Used by %MRQ_EMC_DVFS_LATENCY call to carry data pointed by @pairs of
 * struct mrq_emc_dvfs_latency_response
 */
struct emc_dvfs_latency {
	uint32_t freq;
	uint32_t latency;
} __ABI_PACKED;

#define EMC_DVFS_LATENCY_MAX_SIZE	14
/**
 * struct mrq_emc_dvfs_latency_response
 * @num_pairs: The number of pairs supported
 * @pairs: Array of pair {freq, latency}
 */
struct mrq_emc_dvfs_latency_response {
	uint32_t num_pairs;
	struct emc_dvfs_latency pairs[EMC_DVFS_LATENCY_MAX_SIZE];
} __ABI_PACKED;

/**
 * 3.32 Waypoint2 (MRQ_SC7_WAYPOINT2)
 *
 * Platforms: T186
 * Initiators: BPMP
 * Targets: DMCE
 */

/**
 * struct mrq_sc7_waypoint2_request
 * @cpu_id: CPU to boot
 *
 * Used by %MRQ_SC7_WAYPOINT2 as part of SC7 exit handshake to
 * indicate that BPMP has resumed and there is wake event for CCPLEX.
 */
struct mrq_sc7_waypoint2_request {
	uint32_t cpu_id; /* enum cpu_id */
} __ABI_PACKED;

/**
 * DMCE responds to %MRQ_SC7_WAYPOINT2 message without additional
 * payload data.
 */

/**
 * 3.33 Start deep sleep entry (MRQ_SC7_ENTRY_START)
 *
 * Platforms: T186
 * Initiators: DMCE
 * Targets: BPMP
 *
 * This message indicates that BPMP-FW should start deep sleep entry
 * sequence. MCE will ignore any messages except the one to prepare
 * for deep sleep from BPMP firmware after this. Ignoring means MCE
 * need not handle the message but just ack back the IPC. Full IPC
 * mechanism is used for this message.
 */

/**
 * struct mrq_sc7_entry_start_request
 * @cpu_id: This is the CPU ID (see enum cpu_id) from which deep sleep
 * entry is triggered for SC7 entry through system suspend and CPU
 * with minimum sleep time for SC7 entry through idle state. This
 * information is used on resume to decide which CPU to wake. For
 * system suspend resume must happen from last CPU as all other CPUs
 * would be in offline state. For SC7 from idle if the wake event is
 * RTC then resume must happen from the CPU with minimum wake
 * time. For SC8/shutdown/reboot, this parameter is un-used and can be
 * the core ID from which this state is triggered.
 *
 * @state_id: one of SC7, SC8 (see enum state_id).
 *
 * @sleep_time: [TSC cycles (relative)]. This parameter will be used
 * only for SC7 entry through idle state and not for system
 * suspend. For system suspend this value must be set to 0xFFFFFFFF by
 * MCE, BPMP will use it to distinguish between entry methods.
 *
 */
struct mrq_sc7_entry_start_request {
	uint32_t cpu_id; /* enum cpu_id */
	uint32_t state_id; /* enum state_id */
	uint32_t sleep_time;
} __ABI_PACKED;

/**
 * There is no payload data to be sent as a response for
 * %MRQ_SC7_ENTRY_START request.
 */

/**
 * 3.34 MRQ_SC7_ENTRY_READY
 *
 * Platforms: T186
 * Initiators: BPMP
 * Targets: DMCE
 *
 * This message indicates that BPMP firmware is ready to enter SC7/SC8
 * state, it has suspended its services and completed handshake with
 * SPE. This ensures that no system agent will be modifying memory
 * that CPU might have cached translations and there won’t be any
 * message from BPMP to MCE except doorbell ring for ROC
 * shutdown. After this message MCE can suspend its services e.g. IPC
 * and start waiting for DBELL ring from BPMP indicating MCE to
 * proceed with ROC shutdown. It will use full IPC protocol for
 * communication. This same IPC will get used during SC7 exit to
 * indicate that BPMP firmware has decided to re-enter SC7 and is
 * ready.
 *
 * This message has no additional payload data
 */

/**
 * 3.35 CC7 entry (MRQ_CC7_ENTRY)
 *
 * Platforms: T186
 * Initiators: DMCE
 * Targets: BPMP
 *
 */

/**
 * struct mrq_cc7_entry_request
 * @cluster_id: cluster to enter CC7 state.
 *
 * Used by %MRQ_CC7_ENTRY call by DMCE to request CC7 entry for a
 * given cluster.
 */
struct mrq_cc7_entry_request {
	uint32_t cluster_id; /* enum cluster_id */
} __ABI_PACKED;

/**
 * BPMP responds to %MRQ_CC7_ENTRY message without additional payload data.
 */

/**
 * 3.36 CC7 exit (MRQ_CC7_EXIT)
 *
 * Platforms: T186
 * Initiators: DMCE
 * Targets: BPMP
 */

/**
 * struct mrq_cc7_exit_request
 *
 * @cluster_id: cluster to exit CC7 state.
 *
 * Used by %MRQ_CC7_EXIT call to indicate that MCE have received wake
 * event for the cluster and requesting to turn on rail of this
 * cluster.
 */
struct mrq_cc7_exit_request {
	uint32_t cluster_id; /* enum cluster_id */
} __ABI_PACKED;

/**
 * BPMP responds to %MRQ_CC7_EXIT message without additional payload data.
 */


/**
 * 3.37 MRQ_CC7_EXIT_DONE
 *
 * Platforms: T186
 * Initiators: BPMP
 * Targets: DMCE
 */

/**
 * struct mrq_cc7_exit_done_request
 *
 * @cluster_id: cluster id
 *
 * Used by %MRQ_CC7_EXIT_DONE call to indicate that BPMP has completed
 * the CC7 sequence for cluster $cluster_id
 */
struct mrq_cc7_exit_done_request {
	uint32_t cluster_id; /* enum cluster_id */
} __ABI_PACKED;

/**
 * DMCE responds to %MRQ_CC7_EXIT_DONE message without additional payload data.
 */

/**
 * 3.38 CCP3 state entry (MRQ_CCP3_ENTRY)
 *
 * Platforms: T186
 * Initiators: DMCE
 * Targets: BPMP
 *
 * This message is sent by DMCE to indicate that all cores are in C3
 * or lower power state. There is no request or response payload data
 * associated.
 */

/**
 * 3.39 CCP3 state exit (MRQ_CCP3_EXIT)
 *
 * Platforms: T186
 * Initiators: DMCE
 * Targets: BPMP
 *
 * This message is sent by DMCE to indicate that a core in CCPLEX has
 * received a wake event and is being brought online. There is no
 * request or response payload data associated.
 */

/**
 * 3.40 SCx entry (MRQ_SCX_ENTRY)
 *
 * Platforms: T186
 * Initiators: DMCE
 * Targets: BPMP
 */

/**
 * struct mrq_scx_entry_request
 * @state_id: requested SCx state to enter
 *
 * Used by %MRQ_SCX_ENTRY calls to request entry to a specific system
 * low power state (SC state). The desired state is passed with
 * @state_id.
 */
struct mrq_scx_entry_request {
	uint32_t state_id; /* enum state_id */
} __ABI_PACKED;

/**
 * BPMP responds to %MRQ_SCX_ENTRY message without additional payload
 * data.
 */

/**
 * 3.41 SCx exit (MRQ_SCX_EXIT)
 *
 * Platforms: T186
 * Initiators: DMCE
 * Targets: BPMP
 */

/**
 * struct mrq_scx_exit_request
 * @state_id: TODO - why do we need this?
 *
 * Used by %MRQ_SCX_EXIT call by DMCE to indicate that a core in
 * CCPLEX has received wake event and is requesting exit from SCx
 * state.
 */
struct mrq_scx_exit_request {
	uint32_t state_id; /* TODO: why do we need this */
};

/**
 * BPMP responds to %MRQ_SCX_EXIT message with acknowledgement without
 * any payload data.
 */

/**
 * 3.42 SCx exit done (MRQ_SCX_EXIT_DONE)
 *
 * Platforms: T186
 * Initiators: BPMP
 * Targets: DMCE
 */

/**
 * This message indicates that BPMP has restored all system resource
 * and MCE can continue with SCx exit sequence. There is no request or
 * response payload data associated.
 */

/**
 * 3.43 SCx wake (MRQ_SCX_WAKE)
 *
 * Platforms: T186
 * Initiators: BPMP
 * Targets: DMCE
 */

/**
 * struct mrq_scx_wake
 * @cpu_id: CPU to boot
 *
 * Used by %MRQ_SCX_WAKE call to request DMCE to exit SCx state.
 */
struct mrq_scx_wake {
	uint32_t cpu_id;
} __ABI_PACKED;

/**
 * DMCE responds to %MRQ_SCX_WAKE message without additional payload data.
 */


/**
 * 3.44 SiMon enable (MRQ_SIMON_ENABLE)
 *
 * Platforms: T186
 * Initiators: DMCE
 * Targets: BPMP
 */

/**
 * struct mrq_simon_enable_request
 * @cluster_id: cluster of which SiMon to be enabled
 *
 * Used by %MRQ_SIMON_ENABLE call by DMCE to indicate BPMP that SiMon
 * and Vmin sensors are initialized and can be enabled. See also
 * MRQ_FW_BOOTED.
 */
struct mrq_simon_enable_request {
	uint32_t cluster_id; /* enum cluster_id */
};

/**
 * BPMP responds to %MRQ_SIMON_ENABLE message without additional
 * payload data.
 */

/**
 * 3.45 MRQ_AVFS_INIT
 *
 * Platforms: T186
 * Initiators: BPMP
 * Targets: DMCE
 */

/**
 * struct mrq_avfs_init_request
 * @phys_addr: physical addres of AVFS initialization data
 * @cluster_id: id of cluster to initialize
 *
 * Used by %MRQ_AVFS_INIT calls to ask the DMCE to initialize the AVFS
 * clocking hardware of cluster @cluster_id with data located at
 * @phys_addr. The format for data at @phys_addr is specified by
 * struct avfs_init_data.
 *
 */
struct mrq_avfs_init_request {
	uint64_t phys_addr; /* struct avfs_init_data * */
	uint32_t cluster_id; /* enum cluster_id */
} __ABI_PACKED;

/**
 * struct avfs_init_data
 * @lut: AVFS lut data
 *
 * Used by %MRQ_AVFS_INIT calls to carry data pointed by @phys_addr of
 * struct mrq_avfs_init_request
 */
struct avfs_init_data {
	uint32_t lut[200];
} __ABI_PACKED;

/**
 * BPMP responds to %MRQ_AVFS_INIT message without additional payload
 * data.
 */

/**
 * 3.46 AVFS frequency caps (MRQ_AVFS_FREQ_CAPS)
 *
 * Platforms: T186
 * Initiators: BPMP
 * Targets: DMCE
 */

/**
 * struct mrq_avfs_freq_caps_request
 * @ndivMB_0[64] – maximum allowed ndiv values for M cluster CPUs when
 *                 both clusters are ON
 * @ndivM_0[4] – maximum allowed ndiv values for M cluster when only
 *               M cluster is ON
 * @ndivMB_1[64] – maximum allowed ndiv values for B cluster CPUs then
 *                 both clusters are ON
 * @ndivB_1[16] – maximum allowed ndiv values for B cluster when only
 *                 B cluster is ON
 *
 * Used by %MRQ_AVFS_FREQ_CAPS call to ask DMCE to enforce maximum
 * allowed CPU frequencies for both clusters (NDIV value) for a given
 * number of online cores.
 *
 * Data in ndiv tables are organized as follows:
 * Data in ndivMB_?[] is indexed using (Mcore_mask << 4)| Bcore_mask
 * Data in ndivM_0[4] is indexed using Mcore_mask
 * Data in ndivB_1[16] is indexed using Bcore_mask
 *
 * Mcore_mask is 2 bit value, bit0 represents the state of CPU0, bit1 is CPU1
 * Bcore_mask is 4 bit value, bit0 is the state of CPU4, …, bit3 is
 * the state of CPU7
 *
 * Example for ndivMB_0[] array:
 *  ndivMB_0[0] - All cpus offline
 *  mdivMB_0[1] - CPU4 online
 *  ndivMB_0[2] - CPU5 online
 *  ...
 *  ndivMB_0[16] - CPU0 online
 *  ndivMB_0[17] - CPU0 and CPU4 online
 *  ...
 *  ndivMB_0[48] - CPU0 and CPU1 online
 *  ndivMB_0[49] - CPU0, CPU1, and CPU4 online
 *  ...
 *  ndivMB_0[63] - All CPUs online
 */

struct mrq_avfs_freq_caps_request {
	uint16_t ndivMB_0[64];
	uint16_t ndivM_0[4];
	uint16_t ndivMB_1[64];
	uint16_t ndivB_1[16];
} __ABI_PACKED;

/**
 * DMCE respond to %MRQ_AVFS_FREQ_CAPS message without additional
 * payload data.
 */

/**
 * 3.47 AVFS temp index (MRQ_AVFS_TEMP_IDX)
 *
 * Platforms: T186
 * Initiators: BPMP
 * Targets: DMCE
 */

/**
 * struct mrq_avfs_temp_index_request
 * @temp_index: temperature index for AVFS hardware
 * @cluster_mask: bitmask identifying the clusters to be updated
 *
 * Used by %MRQ_AVFS_TEMP_IDX call to ask DMCE to program the
 * temperature index register fields of AVFS clocking hardware. The
 * @cluster_mask field indicates which clusters to update: if i'th bit
 * of cluster_mask is set then cluster_id=i should be updated.
 */
struct mrq_avfs_temp_index_request {
	uint32_t temp_index;
	uint32_t cluster_mask;
} __ABI_PACKED;

/**
 * DMCE responds to %MRQ_AVFS_TEMP_IDX message without additional
 * payload data
 */

/**
 * 3.48 Firmware booted (MRQ_FW_BOOTED)
 *
 * Platforms: T186
 * Initiators: BPMP
 * Targets: DMCE
 */

/**
 * This message indicates that BPMP Firmware has been loaded and
 * booted and that it is ready to respond to messages from the
 * DMCE. There is no request payload data associated. In response
 * to this message DMCE will send present cpus and bsp.
 * BPMP firmware will enable SiMon for the present clusters. Also,
 * BPMP firmware assumes that CPU rail is enabled for all present
 * clusters.

 * @cpus_present[0] - Denver Core 0
 * @cpus_present[1] - Denver Core 1
 * @cpus_present[4] - A57 Core 0
 * @cpus_present[5] - A57 Core 1
 * @cpus_present[6] - A57 Core 2
 * @cpus_present[7] - A57 Core 3
 * @bsp - Boot CPU
 */
struct mrq_fw_booted_response {
	uint32_t cpus_present;
	uint32_t bsp;
} __ABI_PACKED;

/**
 * 3.49 System shutdown
 *
 * Platforms: T186
 * Initiators: DMCE
 * Targets: BPMP
 */

/**
 * This message indicates system shutdown request. ARM SW requests
 * system shutdown/reboot to DMCE through ARI. DMCE has no action
 * but just to forward this message to BPMP. BPMP will initiate
 * system shutdown/reboot after receiving this message, it may
 * include turning  off some rails in sequence and programming
 * PMIC. There is no response data associated with this message.
 * @state:
 *       0 - Power off
 *       1 - Reboot
 */

struct mrq_shutdown_request {
	uint32_t state;
} __ABI_PACKED;

/**
 * 3.50 EDVD integral check
 *
 * Platforms: T186
 * Initiators: BPMP
 * Targets: DMCE
 */

/**
 * Sent by BPMP to DMCE whenever DMCE should check the EDVD integral
 * errors for the clusters whose bits are set in @cluster_mask. . When
 * the absolute value of an integral error exceeds @err_thresh, DMCE
 * should clear the integrator.
 * @err_thresh : maximum acceptable value for the error integral
 * @cluster_mask: bit 0 corresponds to MCLUSTER, bit 1 to BCLUSTER
 */
struct mrq_edvd_check_request {
	uint32_t err_thresh;
	uint32_t cluster_mask;

} __ABI_PACKED;

/**
 *  4. Enumerations
 */

/**
 *   4.1 CPU enumerations
 *
 * See <mach-t186/system-t186.h>
 *
 *   4.2 CPU Cluster enumerations
 *
 * See <mach-t186/system-t186.h>
 *
 *   4.3 System low power state enumerations
 *
 * See <mach-t186/system-t186.h>
 */

/**
 *   4.4 Clock enumerations
 *
 * For clock enumerations, see <mach-t186/clk-t186.h>
 */

/**
 *   4.5 Reset enumerations
 *
 * For reset enumerations, see <mach-t186/reset-t186.h>
 */

/**
 *   4.6 Thermal sensor enumerations
 *
 * For thermal sensor enumerations, see <mach-t186/thermal-t186.h>
 */

/**
 *  5. Error codes
 *
 */
#define BPMP_ENOENT	2 /* No such file or directory */
#define BPMP_ENOHANDLER	3 /* No MRQ handler */
#define BPMP_EIO	5 /* I/O error */
#define BPMP_EBADCMD	6 /* Bad sub-MRQ command */
#define BPMP_ENOMEM	12 /* Not enough memory */
#define BPMP_EACCES	13 /* Permission denied */
#define BPMP_EFAULT	14 /* Bad address */
#define BPMP_ENODEV	19 /* No such device */
#define BPMP_EISDIR	21
#define BPMP_EINVAL	22 /* Invalid argument */
#define BPMP_ETIMEDOUT  23
#define BPMP_ERANGE	34

#endif
