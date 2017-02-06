/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _GPMUIFPERFMON_H_
#define _GPMUIFPERFMON_H_

/*perfmon task defines*/

#define PMU_DOMAIN_GROUP_PSTATE		0
#define PMU_DOMAIN_GROUP_GPC2CLK	1
#define PMU_DOMAIN_GROUP_NUM		2

#define PMU_PERFMON_FLAG_ENABLE_INCREASE	(0x00000001)
#define PMU_PERFMON_FLAG_ENABLE_DECREASE	(0x00000002)
#define PMU_PERFMON_FLAG_CLEAR_PREV		(0x00000004)

enum pmu_perfmon_cmd_start_fields {
	COUNTER_ALLOC
};

enum {
	PMU_PERFMON_CMD_ID_START = 0,
	PMU_PERFMON_CMD_ID_STOP  = 1,
	PMU_PERFMON_CMD_ID_INIT  = 2
};

struct pmu_perfmon_counter_v0 {
	u8 index;
	u8 flags;
	u8 group_id;
	u8 valid;
	u16 upper_threshold; /* units of 0.01% */
	u16 lower_threshold; /* units of 0.01% */
};

struct pmu_perfmon_counter_v2 {
	u8 index;
	u8 flags;
	u8 group_id;
	u8 valid;
	u16 upper_threshold; /* units of 0.01% */
	u16 lower_threshold; /* units of 0.01% */
	u32 scale;
};

struct pmu_perfmon_cmd_start_v3 {
	u8 cmd_type;
	u8 group_id;
	u8 state_id;
	u8 flags;
	struct pmu_allocation_v3 counter_alloc;
};

struct pmu_perfmon_cmd_start_v2 {
	u8 cmd_type;
	u8 group_id;
	u8 state_id;
	u8 flags;
	struct pmu_allocation_v2 counter_alloc;
};

struct pmu_perfmon_cmd_start_v1 {
	u8 cmd_type;
	u8 group_id;
	u8 state_id;
	u8 flags;
	struct pmu_allocation_v1 counter_alloc;
};

struct pmu_perfmon_cmd_start_v0 {
	u8 cmd_type;
	u8 group_id;
	u8 state_id;
	u8 flags;
	struct pmu_allocation_v0 counter_alloc;
};

struct pmu_perfmon_cmd_stop {
	u8 cmd_type;
};

struct pmu_perfmon_cmd_init_v3 {
	u8 cmd_type;
	u8 to_decrease_count;
	u8 base_counter_id;
	u32 sample_period_us;
	struct pmu_allocation_v3 counter_alloc;
	u8 num_counters;
	u8 samples_in_moving_avg;
	u16 sample_buffer;
};

struct pmu_perfmon_cmd_init_v2 {
	u8 cmd_type;
	u8 to_decrease_count;
	u8 base_counter_id;
	u32 sample_period_us;
	struct pmu_allocation_v2 counter_alloc;
	u8 num_counters;
	u8 samples_in_moving_avg;
	u16 sample_buffer;
};

struct pmu_perfmon_cmd_init_v1 {
	u8 cmd_type;
	u8 to_decrease_count;
	u8 base_counter_id;
	u32 sample_period_us;
	struct pmu_allocation_v1 counter_alloc;
	u8 num_counters;
	u8 samples_in_moving_avg;
	u16 sample_buffer;
};

struct pmu_perfmon_cmd_init_v0 {
	u8 cmd_type;
	u8 to_decrease_count;
	u8 base_counter_id;
	u32 sample_period_us;
	struct pmu_allocation_v0 counter_alloc;
	u8 num_counters;
	u8 samples_in_moving_avg;
	u16 sample_buffer;
};

struct pmu_perfmon_cmd {
	union {
		u8 cmd_type;
		struct pmu_perfmon_cmd_start_v0 start_v0;
		struct pmu_perfmon_cmd_start_v1 start_v1;
		struct pmu_perfmon_cmd_start_v2 start_v2;
		struct pmu_perfmon_cmd_start_v3 start_v3;
		struct pmu_perfmon_cmd_stop stop;
		struct pmu_perfmon_cmd_init_v0 init_v0;
		struct pmu_perfmon_cmd_init_v1 init_v1;
		struct pmu_perfmon_cmd_init_v2 init_v2;
		struct pmu_perfmon_cmd_init_v3 init_v3;
	};
};

struct pmu_zbc_cmd {
	u8 cmd_type;
	u8 pad;
	u16 entry_mask;
};

/* PERFMON MSG */
enum {
	PMU_PERFMON_MSG_ID_INCREASE_EVENT = 0,
	PMU_PERFMON_MSG_ID_DECREASE_EVENT = 1,
	PMU_PERFMON_MSG_ID_INIT_EVENT     = 2,
	PMU_PERFMON_MSG_ID_ACK            = 3
};

struct pmu_perfmon_msg_generic {
	u8 msg_type;
	u8 state_id;
	u8 group_id;
	u8 data;
};

struct pmu_perfmon_msg {
	union {
		u8 msg_type;
		struct pmu_perfmon_msg_generic gen;
	};
};

#endif /* _GPMUIFPERFMON_H_ */
