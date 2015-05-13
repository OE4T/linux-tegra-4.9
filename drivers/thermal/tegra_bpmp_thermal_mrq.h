/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *	Mikko Perttunen <mperttunen@nvidia.com>
 *	Aapo Vienamo	<avienamo@nvidia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __TEGRA_BPMP_THERMAL_MRQ_H__
#define __TEGRA_BPMP_THERMAL_MRQ_H__

enum mrq_thermal_host_to_bpmp_cmd {
	/*
	 * CMD_THERMAL_QUERY_ABI:
	 *   Check whether the BPMP driver supports the specified request type.
	 *
	 * Host needs to supply request parameters.
	 *
	 * Reply codes:
	 *   0		Specified request type is supported.
	 *   -ENODEV	Specified request type is not supported.
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
	 *   -EINVAL	Invalid request parameters.
	 *   -ENOENT	No driver registered for thermal zone..
	 *   -EFAULT	Problem reading temperature measurement.
	 */
	CMD_THERMAL_GET_TEMP = 1,

	/*
	 * CMD_THERMAL_SET_TRIP:
	 *   Enable or disable and set the lower and upper thermal limits
	 *   for a thermal trip point. Each zone has one trip point.
	 *
	 * Host needs to supply request parameters.
	 *
	 * Reply codes:
	 *   0		Trip succesfully set.
	 *   -EINVAL	Invalid request parameters.
	 *   -ENOENT	No driver registered for thermal zone.
	 *   -EFAULT	Problem setting trip point.
	 */
	CMD_THERMAL_SET_TRIP = 2,

	/*
	 *
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
struct cmd_thermal_query_abi_req {
	uint32_t type;
} __packed;

/*
 * Host->BPMP request data for request type CMD_THERMAL_GET_TEMP
 *
 * zone: Number of thermal zone.
 */
struct cmd_thermal_get_temp_req {
	uint32_t zone;
} __packed;

/*
 * BPMP->Host reply data for request CMD_THERMAL_GET_TEMP
 *
 * error: 0 if request succeeded. -EINVAL if request parameters were invalid.
 *        -ENOENT if no driver was registered for the specified thermal zone.
 *        -EFAULT for other thermal zone driver errors.
 * temp: Current temperature in millicelsius.
 */
struct cmd_thermal_get_temp_reply {
	int32_t temp;
} __packed;

/*
 * Host->BPMP request data for request type CMD_THERMAL_SET_TRIP
 *
 * zone: Number of thermal zone.
 * low: Temperature of lower trip point in millicelsius
 * high: Temperature of upper trip point in millicelsius
 * enabled: 1 to enable trip point, 0 to disable trip point
 */
struct cmd_thermal_set_trip_req {
	uint32_t zone;
	int32_t low;
	int32_t high;
	bool enabled;
} __packed;

/*
 * BPMP->Host request data for request type CMD_THERMAL_HOST_TRIP_REACHED
 *
 * zone: Number of thermal zone where trip point was reached.
 */
struct cmd_thermal_host_trip_reached_req {
	uint32_t zone;
} __packed;

/*
 * BPMP->Host reply data for request type CMD_THERMAL_GET_NUM_ZONES
 *
 * num: Number of supported thermal zones. The thermal zones are indexed
 *      starting from zero.
 */
struct cmd_thermal_get_num_zones_reply {
	uint32_t num;
} __packed;

/*
 * Host->BPMP request data.
 *
 * Reply type is union mrq_thermal_bpmp_to_host_reply.
 *
 * type: Type of request. Values listed in enum mrq_thermal_type.
 * data: Request type specific parameters.
 */
struct mrq_thermal_host_to_bpmp_req {
	uint32_t type;
	union {
		struct cmd_thermal_query_abi_req query_abi;
		struct cmd_thermal_get_temp_req get_temp;
		struct cmd_thermal_set_trip_req set_trip;
	};
} __packed;

/*
 * BPMP->Host request data.
 *
 * type: Type of request. Values listed in enum mrq_thermal_type.
 * data: Request type specific parameters.
 */
struct mrq_thermal_bpmp_to_host_req {
	uint32_t type;
	union {
		struct cmd_thermal_host_trip_reached_req host_trip_reached;
	};
} __packed;

/*
 * Data in reply to a Host->BPMP request.
 */
union mrq_thermal_bpmp_to_host_reply {
	struct cmd_thermal_get_temp_reply get_temp;
	struct cmd_thermal_get_num_zones_reply get_num_zones;
} __packed;


#endif /* __TEGRA_BPMP_THERMAL_MRQ_H__ */
