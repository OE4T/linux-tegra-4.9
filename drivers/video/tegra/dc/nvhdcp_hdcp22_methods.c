/*
 * drivers/video/tegra/dc/nvhdcp_hdcp22_methods.c
 *
 * Copyright (c) 2014, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include "tsec/tsec_methods.h"
#include "nvhdcp_hdcp22_methods.h"
#include "tsec_drv.h"

#define hdcp_debug(...)       \
		pr_debug("hdcp: " __VA_ARGS__)
#define hdcp_err(...) \
		pr_err("hdcp: Error: " __VA_ARGS__)

#define HDCP22_SRM_PATH "etc/hdcpsrm/hdcp2x.srm"

u8 g_srm_2x_prod[] = {
0x91, 0x00, 0x00, 0x01, 0x01, 0x00, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x8B,
0xBE, 0x2D, 0x46, 0x05, 0x9F, 0x00, 0x78, 0x7B, 0xF2, 0x84, 0x79, 0x7F, 0xC4,
0xF5, 0xF6, 0xC4, 0x06, 0x36, 0xA1, 0x20, 0x2E, 0x57, 0xEC, 0x8C, 0xA6, 0x5C,
0xF0, 0x3A, 0x14, 0x38, 0xF0, 0xB7, 0xE3, 0x68, 0xF8, 0xB3, 0x64, 0x22, 0x55,
0x6B, 0x3E, 0xA9, 0xA8, 0x08, 0x24, 0x86, 0x55, 0x3E, 0x20, 0x0A, 0xDB, 0x0E,
0x5F, 0x4F, 0xD5, 0x0F, 0x33, 0x52, 0x01, 0xF3, 0x62, 0x54, 0x40, 0xF3, 0x43,
0x0C, 0xFA, 0xCD, 0x98, 0x1B, 0xA8, 0xB3, 0x77, 0xB7, 0xF8, 0xFA, 0xF7, 0x4D,
0x71, 0xFB, 0xB5, 0xBF, 0x98, 0x9F, 0x1A, 0x1E, 0x2F, 0xF2, 0xBA, 0x80, 0xAD,
0x20, 0xB5, 0x08, 0xBA, 0xF6, 0xB5, 0x08, 0x08, 0xCF, 0xBA, 0x49, 0x8D, 0xA5,
0x73, 0xD5, 0xDE, 0x2B, 0xEA, 0x07, 0x58, 0xA8, 0x08, 0x05, 0x66, 0xB8, 0xD5,
0x2B, 0x9C, 0x0B, 0x32, 0xF6, 0x5A, 0x61, 0xE4, 0x9B, 0xC2, 0xF6, 0xD1, 0xF6,
0x2D, 0x0C, 0x19, 0x06, 0x0E, 0x3E, 0xCE, 0x62, 0x97, 0x80, 0xFC, 0x50, 0x56,
0x15, 0xCB, 0xE1, 0xC7, 0x23, 0x4B, 0x52, 0x34, 0xC0, 0x9F, 0x85, 0xEA, 0xA9,
0x15, 0x8C, 0xDD, 0x7C, 0x78, 0xD6, 0xAD, 0x1B, 0xB8, 0x28, 0x1F, 0x50, 0xD4,
0xD5, 0x42, 0x29, 0xEC, 0xDC, 0xB9, 0xA1, 0xF4, 0x26, 0xFA, 0x43, 0xCC, 0xCC,
0xE7, 0xEA, 0xA5, 0xD1, 0x76, 0x4C, 0xDD, 0x92, 0x9B, 0x1B, 0x1E, 0x07, 0x89,
0x33, 0xFE, 0xD2, 0x35, 0x2E, 0x21, 0xDB, 0xF0, 0x31, 0x8A, 0x52, 0xC7, 0x1B,
0x81, 0x2E, 0x43, 0xF6, 0x59, 0xE4, 0xAD, 0x9C, 0xDB, 0x1E, 0x80, 0x4C, 0x8D,
0x3D, 0x9C, 0xC8, 0x2D, 0x96, 0x23, 0x2E, 0x7C, 0x14, 0x13, 0xEF, 0x4D, 0x57,
0xA2, 0x64, 0xDB, 0x33, 0xF8, 0xA9, 0x10, 0x56, 0xF4, 0x59, 0x87, 0x43, 0xCA,
0xFC, 0x54, 0xEA, 0x2B, 0x46, 0x7F, 0x8A, 0x32, 0x86, 0x25, 0x9B, 0x2D, 0x54,
0xC0, 0xF2, 0xEF, 0x8F, 0xE7, 0xCC, 0xFD, 0x5A, 0xB3, 0x3C, 0x4C, 0xBC, 0x51,
0x89, 0x4F, 0x41, 0x20, 0x7E, 0xF3, 0x2A, 0x90, 0x49, 0x5A, 0xED, 0x3C, 0x8B,
0x3D, 0x9E, 0xF7, 0xC1, 0xA8, 0x21, 0x99, 0xCF, 0x20, 0xCC, 0x17, 0xFC, 0xC7,
0xB6, 0x5F, 0xCE, 0xB3, 0x75, 0xB5, 0x27, 0x76, 0xCA, 0x90, 0x99, 0x2F, 0x80,
0x98, 0x9B, 0x19, 0x21, 0x6D, 0x53, 0x7E, 0x1E, 0xB9, 0xE6, 0xF3, 0xFD, 0xCB,
0x69, 0x0B, 0x10, 0xD6, 0x2A, 0xB0, 0x10, 0x5B, 0x43, 0x47, 0x11, 0xA4, 0x60,
0x28, 0x77, 0x1D, 0xB4, 0xB2, 0xC8, 0x22, 0xDB, 0x74, 0x3E, 0x64, 0x9D, 0xA8,
0xD9, 0xAA, 0xEA, 0xFC, 0xA8, 0xA5, 0xA7, 0xD0, 0x06, 0x88, 0xBB, 0xD7, 0x35,
0x4D, 0xDA, 0xC0, 0xB2, 0x11, 0x2B, 0xFA, 0xED, 0xBF, 0x2A, 0x34, 0xED, 0xA4,
0x30, 0x7E, 0xFD, 0xC5, 0x21, 0xB6
};

int tsec_hdcp_readcaps(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_read_caps_param read_caps_param;
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&read_caps_param,
		sizeof(struct hdcp_read_caps_param));
	tsec_send_method(hdcp_context, HDCP_READ_CAPS, 0);
	memcpy(&read_caps_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_read_caps_param));
	if (read_caps_param.ret_code) {
		hdcp_err("tsec_hdcp_readcaps: failed with error %d\n",
			read_caps_param.ret_code);
	}

	return err;
}

int tsec_hdcp_create_session(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_create_session_param create_session_param;

	create_session_param.no_of_streams = 1;
	create_session_param.session_type = 0;
	create_session_param.display_type = 1;
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&create_session_param,
		sizeof(struct hdcp_create_session_param));

	tsec_send_method(hdcp_context, HDCP_CREATE_SESSION, HDCP_MTHD_FLAGS_SB);
	memcpy(&create_session_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_create_session_param));
	if (create_session_param.ret_code) {
		hdcp_err("tsec_hdcp_create_session: failed with error %d\n",
			create_session_param.ret_code);
	}
	hdcp_context->session_id = create_session_param.session_id;
	hdcp_context->msg.rtx = create_session_param.rtx;
	err = create_session_param.ret_code;

	return err;
}

int tsec_hdcp_init(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_init_param init_param;

	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	init_param.flags = 0;
	init_param.chip_id = 0;

	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&init_param,
		sizeof(struct hdcp_init_param));

	tsec_send_method(hdcp_context, HDCP_INIT, HDCP_MTHD_FLAGS_SB);

	memcpy(&init_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_init_param));

	if (init_param.ret_code) {
		hdcp_err("tsec_hdcp_init: failed with error %d\n",
			init_param.ret_code);
	}
	err = init_param.ret_code;
	return err;
}

int tsec_hdcp_verify_cert(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_verify_cert_rx_param verify_cert_rx_param;
	memset(&verify_cert_rx_param, 0,
		sizeof(struct hdcp_verify_cert_rx_param));
	memset(hdcp_context->cpuvaddr_cert, 0, HDCP_CERT_SIZE);
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	verify_cert_rx_param.session_id = hdcp_context->session_id;
	memcpy((u8 *)hdcp_context->cpuvaddr_cert_aligned,
		hdcp_context->msg.cert_rx,
		HDCP_CERT_SIZE);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&verify_cert_rx_param,
		sizeof(struct hdcp_verify_cert_rx_param));
	tsec_send_method(hdcp_context,
		HDCP_VERIFY_CERT_RX,
		HDCP_MTHD_FLAGS_SB|HDCP_MTHD_FLAGS_CERT);
	memcpy(&verify_cert_rx_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_verify_cert_rx_param));

	if (verify_cert_rx_param.ret_code) {
		hdcp_err("tsec_hdcp_verify_cert: failed with error %d\n",
			verify_cert_rx_param.ret_code);
	}
	err = verify_cert_rx_param.ret_code;
	return err;

}

int tsec_hdcp_generate_ekm(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_generate_ekm_param generate_ekm_param;
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	generate_ekm_param.session_id = hdcp_context->session_id;
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&generate_ekm_param,
		sizeof(struct hdcp_generate_ekm_param));
	tsec_send_method(hdcp_context, HDCP_GENERATE_EKM, HDCP_MTHD_FLAGS_SB);
	memcpy(&generate_ekm_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_generate_ekm_param));
	if (generate_ekm_param.ret_code) {
		hdcp_err("tsec_hdcp_generate_ekm: failed with error %d\n",
			generate_ekm_param.ret_code);
		goto exit;
	}
	memcpy(hdcp_context->msg.ekm, generate_ekm_param.ekm, HDCP_SIZE_E_KM_8);
exit:
	err = generate_ekm_param.ret_code;
	return err;
}

int tsec_hdcp_verify_hprime(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_verify_hprime_param verify_hprime_param;
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	verify_hprime_param.session_id = hdcp_context->session_id;
	memcpy(verify_hprime_param.hprime,
		hdcp_context->msg.hprime,
		HDCP_SIZE_HPRIME_8);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&verify_hprime_param,
		sizeof(struct hdcp_verify_hprime_param));
	tsec_send_method(hdcp_context, HDCP_VERIFY_HPRIME, HDCP_MTHD_FLAGS_SB);
	memcpy(&verify_hprime_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_verify_hprime_param));
	if (verify_hprime_param.ret_code) {
		hdcp_err("tsec_hdcp_verify_hprime: failed with error %d\n",
		verify_hprime_param.ret_code);
	}
	err = verify_hprime_param.ret_code;
	return err;
}

int tsec_hdcp_encrypt_pairing_info(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_encrypt_pairing_info_param encrypt_pairing_info_param;
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	encrypt_pairing_info_param.session_id = hdcp_context->session_id;
	memcpy(encrypt_pairing_info_param.ekhkm,
		hdcp_context->msg.ekhkm,
		HDCP_SIZE_EKH_KM_8);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&encrypt_pairing_info_param,
		sizeof(struct hdcp_encrypt_pairing_info_param));
	tsec_send_method(hdcp_context,
		HDCP_ENCRYPT_PAIRING_INFO,
		HDCP_MTHD_FLAGS_SB);
	memcpy(&encrypt_pairing_info_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_encrypt_pairing_info_param));
	if (encrypt_pairing_info_param.ret_code) {
		hdcp_err("tsec_hdcp_encrypt_pairing_info: failed with error %d\n",
			encrypt_pairing_info_param.ret_code);
	}
	err = encrypt_pairing_info_param.ret_code;
	return err;
}

int tsec_hdcp_generate_lc_init(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_generate_lc_init_param generate_lc_init_param;
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	generate_lc_init_param.session_id = hdcp_context->session_id;
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&generate_lc_init_param,
		sizeof(struct hdcp_generate_lc_init_param));
	tsec_send_method(hdcp_context,
		HDCP_GENERATE_LC_INIT,
		HDCP_MTHD_FLAGS_SB);
	memcpy(&generate_lc_init_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_generate_lc_init_param));
	if (generate_lc_init_param.ret_code) {
		hdcp_err("tsec_hdcp_generate_lc_init: failed with error %d\n",
			generate_lc_init_param.ret_code);
		goto exit;
	}
	memcpy(&hdcp_context->msg.rn,
		&generate_lc_init_param.rn,
		HDCP_SIZE_RN_8);
exit:
	err = generate_lc_init_param.ret_code;
	return err;
}

int tsec_hdcp_verify_lprime(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_verify_lprime_param verify_lprime_param;
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	verify_lprime_param.session_id = hdcp_context->session_id;
	memcpy(verify_lprime_param.lprime,
		hdcp_context->msg.lprime,
		HDCP_SIZE_LPRIME_8);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&verify_lprime_param,
		sizeof(struct hdcp_verify_lprime_param));
	tsec_send_method(hdcp_context, HDCP_VERIFY_LPRIME, HDCP_MTHD_FLAGS_SB);
	memcpy(&verify_lprime_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_verify_lprime_param));
	if (verify_lprime_param.ret_code) {
		hdcp_err("tsec_hdcp_verify_lprime: failed with error %d\n",
			verify_lprime_param.ret_code);
	}
	err = verify_lprime_param.ret_code;
	return err;
}

int tsec_hdcp_ske_init(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_generate_ske_init_param generate_ske_init_param;
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	generate_ske_init_param.session_id = hdcp_context->session_id;
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&generate_ske_init_param,
		sizeof(struct hdcp_verify_lprime_param));
	tsec_send_method(hdcp_context,
		HDCP_GENERATE_SKE_INIT,
		HDCP_MTHD_FLAGS_SB);
	memcpy(&generate_ske_init_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_generate_ske_init_param));
	if (generate_ske_init_param.ret_code) {
		hdcp_err("tsec_hdcp_ske_init: failed with error %d\n",
			generate_ske_init_param.ret_code);
		goto exit;
	}
	memcpy(hdcp_context->msg.eks,
		(u8 *)generate_ske_init_param.eks,
		HDCP_SIZE_E_KS_8);
	memcpy(&hdcp_context->msg.riv,
		&generate_ske_init_param.riv,
		HDCP_SIZE_RIV_8);
exit:
	err = generate_ske_init_param.ret_code;
	return err;
}

int tsec_hdcp_session_ctrl(struct hdcp_context_t *hdcp_context, int flag)
{
	int err = 0;
	struct hdcp_session_ctrl_param session_ctrl_param;
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	session_ctrl_param.session_id = hdcp_context->session_id;
	session_ctrl_param.ctrl_flag = flag;
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&session_ctrl_param,
		sizeof(struct hdcp_session_ctrl_param));
	tsec_send_method(hdcp_context,
		HDCP_SESSION_CTRL,
		HDCP_MTHD_FLAGS_SB);
	memcpy(&session_ctrl_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_session_ctrl_param));
	if (session_ctrl_param.ret_code) {
		hdcp_err("tsec_hdcp_session_ctrl: failed with error %d\n",
			session_ctrl_param.ret_code);
		goto exit;
	}
exit:
	err = session_ctrl_param.ret_code;
	return err;
}

int tsec_hdcp_revocation_check(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct file *fp = NULL;
	mm_segment_t seg;
	struct hdcp_revocation_check_param revocation_check_param;
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	revocation_check_param.trans_id.session_id = hdcp_context->session_id;
	revocation_check_param.is_ver_hdcp2x = 1;
	fp = filp_open(HDCP22_SRM_PATH, O_RDONLY, 0);
	if (!fp) {
		hdcp_err("Opening SRM file failed!\n");
		return -ENOENT;
	}
	seg = get_fs();
	set_fs(get_ds());
	fp->f_op->read(fp, (u8 *)hdcp_context->cpuvaddr_srm,
			HDCP_SRM_SIZE, &fp->f_pos);
	set_fs(seg);
	revocation_check_param.srm_size = fp->f_pos;
	filp_close(fp, NULL);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&revocation_check_param,
		sizeof(struct hdcp_revocation_check_param));
	tsec_send_method(hdcp_context,
		HDCP_REVOCATION_CHECK,
		HDCP_MTHD_FLAGS_SB | HDCP_MTHD_FLAGS_SRM);
	memcpy(&revocation_check_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_revocation_check_param));
	if (revocation_check_param.ret_code) {
		hdcp_err("tsec_hdcp_revocation_check: failed with error %d\n",
			revocation_check_param.ret_code);
		goto exit;
	}
exit:
	err = revocation_check_param.ret_code;
	return err;
}

int tsec_hdcp_verify_vprime(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	u16 rxinfo;
	struct hdcp_verify_vprime_param verify_vprime_param;
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	/* SRM is already copied into the buffer during revocation check */
	/* Copy receiver id list into buf */
	memcpy(hdcp_context->cpuvaddr_rcvr_id_list,
		hdcp_context->msg.rcvr_id_list,
		HDCP_RCVR_ID_LIST_SIZE);
	memcpy(&verify_vprime_param.vprime,
		hdcp_context->msg.vprime,
		HDCP_SIZE_VPRIME_2X_8/2);
	verify_vprime_param.rxinfo = hdcp_context->msg.rxinfo;

	verify_vprime_param.trans_id.session_id = hdcp_context->session_id;
	/* Get the device count and depth from rx */
	rxinfo = cpu_to_be16(hdcp_context->msg.rxinfo);
	verify_vprime_param.device_count = (rxinfo & 0x01F0)>>4;
	verify_vprime_param.depth = (rxinfo & 0x0E00)>>9;
	verify_vprime_param.has_hdcp2_repeater = (rxinfo & 0x0002)>>1;
	verify_vprime_param.has_hdcp1_device = (rxinfo & 0x0001);
	verify_vprime_param.bstatus = 0;
	verify_vprime_param.is_ver_hdcp2x = 1;
	memcpy(&verify_vprime_param.seqnum,
		&hdcp_context->msg.seq_num,
		HDCP_SIZE_SEQ_NUM_V_8);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&verify_vprime_param,
		sizeof(struct hdcp_verify_vprime_param));
	tsec_send_method(hdcp_context,
	HDCP_VERIFY_VPRIME,
	HDCP_MTHD_FLAGS_SB|HDCP_MTHD_FLAGS_RECV_ID_LIST|HDCP_MTHD_FLAGS_SRM);
	memcpy(&verify_vprime_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_verify_vprime_param));
	if (verify_vprime_param.ret_code) {
		hdcp_err("tsec_hdcp_verify_vprime: failed with error %d\n",
			verify_vprime_param.ret_code);
	}
	err = verify_vprime_param.ret_code;
	if (!err)
		memcpy(hdcp_context->msg.v,
			verify_vprime_param.v128l,
			HDCP_SIZE_VPRIME_2X_8/2);
	return err;
}

int tsec_hdcp_exchange_info(struct hdcp_context_t *hdcp_context,
				u32 method_flag,
				u8 *version,
				u16 *caps)
{
	int err = 0;
	struct hdcp_exchange_info_param exchange_info;
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	exchange_info.session_id = hdcp_context->session_id;
	exchange_info.method_flag = method_flag;
	if (method_flag == HDCP_EXCHANGE_INFO_SET_RCVR_INFO) {
		exchange_info.info.set_rx_info.version = *version;
		exchange_info.info.set_rx_info.rcvr_caps_mask = *caps;
	} else if (method_flag == HDCP_EXCHANGE_INFO_SET_TMTR_INFO) {
		exchange_info.info.set_tx_info.version = *version;
		exchange_info.info.set_tx_info.tmtr_caps_mask = *caps;
	}
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&exchange_info,
		sizeof(struct hdcp_exchange_info_param));
	tsec_send_method(hdcp_context, HDCP_EXCHANGE_INFO, HDCP_MTHD_FLAGS_SB);
	memcpy(&exchange_info,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_exchange_info_param));
	if (exchange_info.ret_code) {
		hdcp_err("tsec_hdcp_exchange_info: failed with error %d\n",
			exchange_info.ret_code);
	}
	err = exchange_info.ret_code;
	if (!err) {
		if (method_flag == HDCP_EXCHANGE_INFO_GET_RCVR_INFO) {
			*version = exchange_info.info.get_rx_info.version;
			*caps = exchange_info.info.get_rx_info.rcvr_caps_mask;
		} else if (method_flag == HDCP_EXCHANGE_INFO_GET_TMTR_INFO) {
			*version = exchange_info.info.get_tx_info.version;
			*caps = exchange_info.info.get_tx_info.tmtr_caps_mask;
		}
	}
	return err;
}

int tsec_hdcp_update_rrx(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_update_session_param update_session_param;
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	update_session_param.session_id = hdcp_context->session_id;
	update_session_param.updmask   =
		(1 << HDCP_UPDATE_SESSION_MASK_RRX_PRESENT);
	memcpy(&update_session_param.rrx,
		&hdcp_context->msg.rrx,
		HDCP_SIZE_RRX_8);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&update_session_param,
		sizeof(struct hdcp_update_session_param));
	tsec_send_method(hdcp_context,
		HDCP_UPDATE_SESSION,
		HDCP_MTHD_FLAGS_SB);
	memcpy(&update_session_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_update_session_param));
	if (update_session_param.ret_code) {
		hdcp_err("tsec_hdcp_update_rrx: failed with error %d\n",
			update_session_param.ret_code);
	}
	err = update_session_param.ret_code;
	return err;
}

int tsec_hdcp_rptr_stream_ready(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_stream_manage_param    stream_manage_param;
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	stream_manage_param.session_id = hdcp_context->session_id;
	stream_manage_param.manage_flag = HDCP_STREAM_MANAGE_FLAG_READY;
	stream_manage_param.str_type[0][0] = 0;
	stream_manage_param.content_id[0][0] = 0x10;
	stream_manage_param.content_id[0][1] = 0x11;
	memcpy(stream_manage_param.mprime,
		hdcp_context->msg.mprime,
		HDCP_SIZE_MPRIME_8);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&stream_manage_param,
		sizeof(struct hdcp_stream_manage_param));
	tsec_send_method(hdcp_context, HDCP_STREAM_MANAGE, HDCP_MTHD_FLAGS_SB);
	memcpy(&stream_manage_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_stream_manage_param));
	/* Need to fix the ucode for this */
	if (stream_manage_param.ret_code) {
		hdcp_err("tsec_hdcp_rptr_auth_stream_manage: failed with error %d\n",
		stream_manage_param.ret_code);
	}
	err = stream_manage_param.ret_code;
	return err;
}
