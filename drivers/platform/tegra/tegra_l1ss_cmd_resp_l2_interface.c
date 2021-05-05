/*
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
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

#include "tegra_l1ss.h"
#include "tegra_l1ss_cmd_resp_l2_interface.h"

int l1ss_cmd_resp_send_frame(const cmdresp_frame_ex_t *pCmdPkt,
			     nv_guard_3lss_layer_t NvGuardLayerId,
			     struct l1ss_data *ldata);

int cmd_resp_l1_callback_not_configured(const cmdresp_frame_ex_t *cmd_resp,
					struct l1ss_data *ldata) {
	PDEBUG("%s(%d) Command not implemented\n", __func__, __LINE__);
	return 0;
}

int cmd_resp_l1_user_rcv_register_notification(
					 const cmdresp_frame_ex_t *CmdRespFrame,
					 struct l1ss_data *ldata)
{
	nv_guard_grp_list_t lVar1;
	uint8_t *lPtr;

	lPtr = (uint8_t *)&lVar1;
	(void)memcpy(lPtr, &(CmdRespFrame->data[0]),
			sizeof(nv_guard_grp_list_t));
	PDEBUG("%s(%d) num_group=%d %d\n", __func__, __LINE__,
			lVar1.num_grp, lVar1.grp_list[0]);
	atomic_set(&ldata->cmd.notify_registered, 1);
	wake_up(&ldata->cmd.notify_waitq);

	return 0;
}

int user_send_service_status_notification(const nv_guard_srv_status_t *Var1,
					  nv_guard_3lss_layer_t Layer_Id,
					  struct l1ss_data *ldata) {
	cmdresp_frame_ex_t lCmdRespData = {0};
	//NvGuard_ReturnType_t lRet = NVGUARD_E_NOK;
	uint8_t lBytePos = 0U;
	const uint8_t *lPtr = (const uint8_t *)(Var1);
	unsigned long timeout;

	PDEBUG("SrvId = %d Status=%d ErrorInfoSize=%d ErrorInfo=%s\n",
			Var1->srv_id, Var1->status,
			Var1->error_info_size, Var1->error_info);
	(void)memcpy(&lCmdRespData.data[lBytePos], lPtr,
			sizeof(nv_guard_srv_status_t));
	lBytePos += (uint8_t)sizeof(nv_guard_srv_status_t);

	/* ToDo: Calclulate CRC
	 * lCmdRespData.e2ecf1_crc = 0xEFEF;
	 * lCmdRespData.e2ecf2 = 0xAA;
	 */
	cmd_resp_update_header(&(lCmdRespData.header),
			CMDRESPL1_CLASS1,
			CMDRESPL1_SERVICE_STATUS_NOTIFICATION,
			Layer_Id,
			false);

	PDEBUG("%s(%d) wait for register notify from SCE\n",
			__func__, __LINE__);
	timeout =
		wait_event_interruptible_timeout(ldata->cmd.notify_waitq,
						 (atomic_read(
						 &ldata->cmd.notify_registered)
						  == 1),
						 10 * HZ);
	if (timeout <= 0) {
		PDEBUG("Done ..but timeout ... wait for register notify\n");
		return -1;
	}
	PDEBUG("%s(%d) Done ..wait for register notify\n", __func__, __LINE__);

	l1ss_cmd_resp_send_frame(&lCmdRespData, Layer_Id, ldata);

	return 0;
}

int user_send_ist_mesg(const nv_guard_user_msg_t *var1,
					  nv_guard_3lss_layer_t layer_id,
					  struct l1ss_data *ldata) {
	cmdresp_frame_ex_t lCmdRespData = {0};
	//NvGuard_ReturnType_t lRet = NVGUARD_E_NOK;
	uint8_t lBytePos = 0U;
	const uint8_t *lPtr = (const uint8_t *)(var1);

	(void)memcpy(&lCmdRespData.data[lBytePos], lPtr,
			sizeof(nv_guard_user_msg_t));
	lBytePos += (uint8_t)sizeof(nv_guard_user_msg_t);

	cmd_resp_update_header(&(lCmdRespData.header),
			CMDRESPL1_CLASS2,
			CMDRESPL1_SEND_ISTMESG,
			layer_id,
			false);
	l1ss_cmd_resp_send_frame(&lCmdRespData, layer_id, ldata);

	return 0;
}
