#ifndef UAPI_SCSI_UFS_UFS_IOCTL_H_
#define UAPI_SCSI_UFS_UFS_IOCTL_H_

#include <linux/types.h>

/*
 *  IOCTL opcode for ufs queries has the following opcode after
 *  SCSI_IOCTL_GET_PCI
 */
#define UFS_IOCTL_QUERY			0x5388

/**
 * struct ufs_ioc_query_cmd - used to transfer ufs query command/data to and
 * from user via ioctl
 */
struct ufs_ioc_query_req {
	/* Query opcode to specify the type of Query operation */
	__u8 opcode;
	/* idn to provide more info on specific operation. */
	__u8 idn;
	/* index - optional in some cases */
	__u8 index;
	/* index - optional in some cases */
	__u8 selector;
	/* buf_size - buffer size in bytes pointed by buffer. */
	__u16 buf_size;
	/*
	 * user buffer pointer for query data.
	 * Note:
	 * For Read/Write Attribute this should be of 4 bytes
	 * For Read Flag this should be of 1 byte
	 * For Descriptor Read/Write size depends on the type of the descriptor
	 */
	__u8 *buffer;
};

#endif /* UAPI_SCSI_UFS_UFS_IOCTL_H_ */
