/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
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
#define pr_fmt(fmt) "vm-err-sample-handler: " fmt

#include <linux/module.h>
#include <linux/vm_err.h>

/* Bridge error details:
 * Note: These are redefined here only to allow user friendly messages
 * describing the error.
 * This must match with "Timeout error" value in t18x_axi_errors[]
 * in nvidia/drivers/platform/tegra/bridge_mca.c
 */
static const unsigned int BRIDGE_ERROR_TIMEOUT = 18;

/* This must match with "CCPLEX" value in src_ids
 * in nvidia/drivers/platform/tegra/bridge_mca.c
 */
static const unsigned int BRIDGE_SRC_ID_CCPLEX = 1;

/* This must match with corresponding HV definition in pct.h */
static const unsigned int GUEST_UNASSIGNED = 18;

static struct vm_err_handlers handlers;
static struct tegra_hv_config config;

static void print_bridge_error(const struct errData * const err_data)
{
	const struct async_bridgeErr * const br_err_data =
		&err_data->async_bridgeErr;
	unsigned int protection;

	pr_crit("Bridge error details\n");
	pr_crit("--------------------------------------\n");
	pr_crit("Err count %d: %s FAULT ADDR 0x%x status1 0x%x status2 0x%x\n",
		br_err_data->count, br_err_data->br_name, br_err_data->err_addr,
		br_err_data->err_status1, br_err_data->err_status2);

	pr_crit("\tDirection: %s\n", br_err_data->rw ? "READ" : "WRITE");
	pr_crit("\tBridge ID: 0x%x\n", br_err_data->br_id);
	pr_crit("\tError type: %u %s\n",
		br_err_data->err_type,
		(br_err_data->err_type == BRIDGE_ERROR_TIMEOUT) ?
			"(Timeout)" : "");

	pr_crit("\tLength: %d\n", br_err_data->length);
	protection = br_err_data->protection;
	pr_crit("\tProtection: 0x%x %s %s %s access\n", protection,
		(protection & 0x4) ? "Instruction" : "Data",
		(protection & 0x2) ? "Non-Secure" : "Secure",
		(protection & 0x1) ? "Privileged" : "Unprivileged");

	pr_crit("\tSource ID: 0x%x -- %s\n",
		br_err_data->src_id,
		(br_err_data->src_id == BRIDGE_SRC_ID_CCPLEX) ?
			" (CCPLEX)" : "");

	pr_crit("\tAXI_ID: 0x%x\n", br_err_data->axi_id);
	pr_crit("\tCache: 0x%x\n", br_err_data->cache);
	pr_crit("\tBurst: 0x%x\n", br_err_data->burst);
	pr_crit("--------------------------------------\n");
}

static void print_smmu_error(const struct errData * const err_data,
				const enum errReason reason)
{
	const struct async_smmuErr * const smmu_err_data =
		&err_data->async_smmuErr;

	pr_crit("SMMU error details\n");
	pr_crit("--------------------------------------\n");
	if (reason == REASON_ASYNC_SMMU_CB) {
		pr_crit("SMMU Context Bank %u error. StreamID: %d\n",
			smmu_err_data->cb_id, smmu_err_data->stream_id);
	} else if (reason == REASON_ASYNC_SMMU_GLOBAL) {
		pr_crit("Global SMMU fault. CB: %u. StreamID: %d\n",
			smmu_err_data->cb_id, smmu_err_data->stream_id);
	} else {
		pr_crit("Unexpected fault reason %d\n", reason);
	}
	pr_crit("FSR: 0x%x; FAR: 0x%llx; FSYND0: 0x%x; FSYND1: 0x%x\n",
			smmu_err_data->fsr, smmu_err_data->far,
			smmu_err_data->fsynr0, smmu_err_data->fsynr1);
	pr_crit("--------------------------------------\n");
}

static void print_mc_error(const struct errData * const err_data)
{
	const struct async_mcErr * const mc_err_data = &err_data->async_mcErr;

	pr_crit("Memory Controller error details\n");
	pr_crit("--------------------------------------\n");
	pr_crit("mc_err: base: 0x%llx, int_status: 0x%08x; err_status: 0x%08x;"
		" fault_addr: 0x%llx\n",
		mc_err_data->ch_base, mc_err_data->int_status,
		mc_err_data->err_status, mc_err_data->fault_addr);
	pr_crit("vcpuid %u, client_id %u, peripheral_id %d\n",
		mc_err_data->vcpuid, mc_err_data->client_id,
		mc_err_data->peripheral_id);
	pr_crit("--------------------------------------\n");
}

static void print_data_abort(const struct errData *const err_data)
{
	const struct sync_dataAbort * const data_abort =
		&err_data->sync_dataAbort;

	pr_crit("Data abort details\n");
	pr_crit("--------------------------------------\n");
	pr_crit("offending VCpu Id %u\n", data_abort->offendingVCpuId);
	(data_abort->isWrite) ?
		pr_crit("write access\n") : pr_crit("read access\n");
	pr_crit("access size %u\n", data_abort->accessSize);
	pr_crit("fault address: 0x%llx\n", data_abort->faultAddr);
	pr_crit("esr: 0x%x\n", data_abort->esrEl2);
	pr_crit("spsr_el2: 0x%llx\n", data_abort->spsrEl2);
	pr_crit("elr_el1: 0x%llx\n", data_abort->elrEl1);
	pr_crit("gprArray[0]: 0x%llx\n", data_abort->gprArray[0]);
	pr_crit("gprArray[15]: 0x%llx\n", data_abort->gprArray[15]);
	pr_crit("gprArray[30]: 0x%llx\n", data_abort->gprArray[30]);
	pr_crit("--------------------------------------\n");
}

static bool handle_async_err_details(const struct errData * const err_data)
{
	bool enter_bad_mode;

	if (err_data->errType != ASYNC) {
		pr_crit("%s: incorrect error type: %d\n", __func__,
			err_data->errType);
		/* Unexpected error type. Enter bad mode. */
		return true;
	}

	pr_info("%s: error reason: %s\n", __func__,
		fault_reason_desc[err_data->errReason]);
	switch (err_data->errReason) {
	case REASON_ASYNC_BRIDGE:
		print_bridge_error(err_data);
		/* Bridge error may not be fatal */
		enter_bad_mode = false;
		break;

	case REASON_ASYNC_SMMU_CB:
		print_smmu_error(err_data, err_data->errReason);
		/* SMMU context bank error may not be fatal */
		enter_bad_mode = false;
		break;

	case REASON_ASYNC_SMMU_GLOBAL:
		print_smmu_error(err_data, err_data->errReason);
		/* Can't recover from global SMMU error. */
		enter_bad_mode = true;
		break;

	case REASON_ASYNC_MC:
		print_mc_error(err_data);
		enter_bad_mode = false;
		break;

	default:
		pr_crit("%s: unhandled error. Reason id %d\n", __func__,
			err_data->errReason);
		enter_bad_mode = true;
		break;
	}

	return enter_bad_mode;
}

static bool handle_sync_err_details(const struct errData * const err_data)
{
	/* Currently only data abort error injection is supported */
	if (err_data->errReason != REASON_SYNC_DATA_ABORT) {
		pr_crit("%s: unexpected reason id %u\n", __func__,
			err_data->errReason);
		/* Invalid reason. Enter bad mode. */
		return true;
	}
	pr_info("%s: error reason: %s\n", __func__,
		fault_reason_desc[err_data->errReason]);
	print_data_abort(err_data);

	/* Recovery from sync error could be impossible. Enter bad mode. */
	return true;
}

static bool handle_peer_err_details(const struct errData * const err_data)
{
	bool enter_bad_mode;
	const unsigned int offender = err_data->offendingGuestId;

	if (offender >= config.num_guests) {
		if (offender != GUEST_UNASSIGNED) {
			pr_crit("%s: invalid offending peer guest id %u\n",
				__func__, offender);
			/* Unexpected. Cause reboot. */
			return true;
		}
		pr_crit("%s: HV can't attribute error to any guest\n",
			__func__);
	} else
		pr_crit("Peer error. Offending guest id = %u\n", offender);

	pr_crit("Error Type: %s\n", (err_data->errType == SYNC) ?
		"Synchronous" : "Asynchronous");

	if (err_data->errReason >= REASON_ENUM_SIZE) {
		pr_crit("%s: unexpected reason id %u\n", __func__,
			err_data->errReason);
		/* Unexpected. Cause reboot. */
		return true;
	}
	pr_crit("%s: error reason: %s\n", __func__,
		fault_reason_desc[err_data->errReason]);

	switch (err_data->errReason) {
	case REASON_ASYNC_BRIDGE:
		print_bridge_error(err_data);
		enter_bad_mode = false;
		break;

	case REASON_ASYNC_SMMU_CB:
	case REASON_ASYNC_SMMU_GLOBAL:
		print_smmu_error(err_data, err_data->errReason);
		enter_bad_mode = false;
		break;

	case REASON_ASYNC_MC:
		print_mc_error(err_data);
		enter_bad_mode = false;
		break;

	case REASON_SYNC_DATA_ABORT:
		print_data_abort(err_data);
		enter_bad_mode = false;
		break;

	default:
		pr_crit("%s: unhandled error. Reason id %d\n", __func__,
		err_data->errReason);
		enter_bad_mode = false;
		break;
	}

	if (offender == GUEST_UNASSIGNED)
		enter_bad_mode = true;

	return enter_bad_mode;
}

static bool self_async_err_handler(const struct errData *const err_data)
{
	return handle_async_err_details(err_data);
}

static bool self_sync_err_handler(const struct errData *const err_data)
{
	return handle_sync_err_details(err_data);
}

static bool peer_err_handler(const struct errData *const err_data)
{
	return handle_peer_err_details(err_data);
}

static int hooks_init(void)
{
	int ret;

	handlers.fn_self_async = self_async_err_handler;
	handlers.fn_self_sync = self_sync_err_handler;
	handlers.fn_peer =
		IS_ENABLED(CONFIG_TEGRA_EBP) ? NULL : peer_err_handler;

	ret = tegra_hv_register_vm_err_hooks(&handlers);
	if (ret)
		return ret;

	tegra_hv_get_config(&config);
	pr_info("%s: Guest Id %u\n", __func__, config.guest_id_self);

	/* EBP, being unprivileged, doesn't know about total guests */
	if (IS_ENABLED(CONFIG_TEGRA_EBP) == 0)
		pr_info("%s: Total guests %u\n", __func__, config.num_guests);

	return 0;
}

static void hooks_exit(void)
{
	struct vm_err_handlers handlers;

	handlers.fn_self_async = NULL;
	handlers.fn_self_sync = NULL;
	handlers.fn_peer = NULL;

	tegra_hv_register_vm_err_hooks(&handlers);
}
subsys_initcall(hooks_init);
module_exit(hooks_exit);

MODULE_AUTHOR("Nvidia Corporation");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Sample VM Error Handler");
