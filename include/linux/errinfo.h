/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef __INCLUDED_ERRINFO_H__
#define __INCLUDED_ERRINFO_H__

enum errReason {
	REASON_UNDEFINED = 0UL,
	REASON_ASYNC_SMMU_CB,
	REASON_ASYNC_SMMU_GLOBAL,
	REASON_ASYNC_BRIDGE,
	REASON_ASYNC_MC,
	REASON_SYNC_INSTR_ABORT,
	REASON_SYNC_DATA_ABORT,
	REASON_SYNC_OTHER,
	REASON_ENUM_SIZE
};

enum errType {
	SYNC = 0UL,
	ASYNC
};

struct __attribute__((__packed__)) async_metaData {
	uint64_t	rdIdx;
	uint64_t	wrIdx;
};

#define NAME_SIZE 64

struct __attribute__((__packed__)) async_bridgeErr {
	char		br_name[NAME_SIZE];
	unsigned int	err_addr;
	unsigned int	err_status1;
	unsigned int	err_status2;
	unsigned int	rw;
	unsigned int	err_type;
	unsigned int	length;
	unsigned int	br_id;
	unsigned int	src_id;
	unsigned int	axi_id;
	unsigned int	count;
	unsigned int	protection;
	unsigned int	burst;
	unsigned int	cache;
};

struct __attribute__((__packed__)) async_smmuErr {
	unsigned int	stream_id;
	unsigned int	cb_id;
	unsigned int	fsynr0;
	unsigned int	fsynr1;
	uint64_t	far;
	unsigned int	fsr;
};

struct __attribute__((__packed__))  async_mcErr {
	uint64_t	ch_base;
	unsigned int	int_status;
	unsigned int	err_status;
	uint64_t	fault_addr;
	unsigned int	vcpuid;		//0xffffU; /* IDLE_vCPU_ID */
	unsigned int	client_id;
	int32_t		peripheral_id;
};

struct __attribute__((__packed__)) sync_dataAbort {
	bool		isFilled;	//metadata field per VCpu
	bool		isWrite;
	uint8_t		accessSize;
	unsigned int	offendingVCpuId;
	unsigned int	esrEl2;
	uint64_t	faultAddr;
	uint64_t	spsrEl2;
	uint64_t	elrEl1;
	uint64_t	gprArray[31];
};

struct __attribute__((__packed__)) errData {
	unsigned int	offendingGuestId;
	enum errType	errType;
	enum errReason	errReason;
	union {
		// *A*synchronous
		struct async_bridgeErr	async_bridgeErr;
		struct async_smmuErr	async_smmuErr;
		struct async_mcErr	async_mcErr;
		// Synchronous
		struct sync_dataAbort	sync_dataAbort;
	};
};

/* VM shared memory for error information is allocated contiguously to store
 * Asynchronous(async) error information followed by the Synchronous(sync)
 * error information. HV has write access and the VM has read access to this
 * shared memory. The shared memory layout looks like:
 *
 * |--async-err-metadata--|--async-errors-array-|--sync-errors-array-|
 *
 * Size of async errors array = Max errors + 1(to avoid same empty and full
 * conditions of the buffer)
 * Size of sync errors array = 1 error per VCPU * number of VCPUs on a VM
 *
 * So for a give VM, shared memory has:
 *
 * |--------ASyncErrInfo----------------|-------SyncErrInfo-------------------|
 * |--------1bufferPerVM----------------|---VCpu0-buffer---|--VCpuN-buffer----|
 * |---metaData----|---errData----------|-metaData+errData-|-metaData+errData-|
 * |-rdIdx-|-wrIdx-|-Err1-|-Err2-|-ErrN-|-isFilled-|-Err1--|-isFilled-|-Err1--|
 */

struct __attribute__((__packed__)) errInfo {
	struct async_metaData	async_metaData;
	struct errData		errData[];
};

#endif
