/*
 * drivers/usb/ucsi/ucsi_ccg.c
 *
 * Copyright (C) 2017-2018 NVIDIA Corporation. All rights reserved.
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

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/string.h>
#include <linux/i2c.h>
#include <linux/extcon.h>

#include "ucsi.h"

/* CCGx dev info registers */
#define REG_DEVICE_MODE		0x0000
#define REG_INTR		0x0006
#define  DEV_INT		(1<<0)
#define  PORT0_INT		(1<<1)
#define  PORT1_INT		(1<<2)
#define  UCSI_READ_INT		(1<<7)
#define REG_JUMP_TO_BOOT	0x0007
#define  TO_BOOT		'J'
#define  TO_ALT_FW		'A'
#define REG_RESET_REQ		0x0008
#define  RESET_SIG		'R'
#define  CMD_RESET_I2C		0x0
#define  CMD_RESET_DEV		0x1
#define REG_ENTER_FLASHING	0x000A
#define  FLASH_ENTER_SIG	'P'
#define REG_VALIDATE_FW		0x000B
#define REG_FLASH_ROW_RW	0x000C
#define  FLASH_SIG		'F'
#define  FLASH_RD_CMD		0x0
#define  FLASH_WR_CMD		0x1
#define REG_READ_ALL_VER        0x0010
#define REG_PDPORT_ENABLE	0x002C
#define REG_RESPONSE            0x007E
#define  ASYNC_EVENT            (1<<7)
#define REG_FLASH_RW_MEM        0x0200
#define DEV_REG_IDX		REG_DEVICE_MODE

/* CCGx PD port registers */
#define REG_SELECT_SINK_PDO(x)	(0x1005 + (x * 0x1000))
#define  PDO_0			(1<<0)
#define  PDO_1                  (1<<1)
#define  PDO_2                  (1<<2)
#define  PDO_3                  (1<<3)
#define  PDO_4                  (1<<4)
#define REG_PD_STATUS(x)        (0x1008 + (x * 0x1000))
#define REG_TYPEC_STATUS(x)     (0x100C + (x * 0x1000))
#define REG_EVENT_MASK(x)	(0x1024 + (x * 0x1000))
#define  OC_DET			(1<<1)
#define	 OV_DET			(1<<2)
#define  CONN_DET		((1<<3) | (1<<4))
#define  PD_EVT_DET		((1<<5) | (1<<6))
#define REG_PD_RESPONSE(x)      (0x1400 + (x * 0x1000))
#define PD_PORT0_REG_IDX        0x1000
#define PD_PORT1_REG_IDX        0x2000

/* UCSI memory region */
#define REG_UCSI_VERSION	0xF000
#define REG_UCSI_CCI		0xF004
#define REG_UCSI_CONTROL_CMD	0xF008
#define REG_UCSI_MSG_IN		0xF010
#define REG_UCSI_MSG_OUT	0xF020

#define BOOT_WAIT_TIMEOUT_MS	200
#define CCG_CMD_TIMEOUT_MS	1000

#define CYACD_LINE_SIZE         527
#define CCG4_ROW_SIZE           256
#define FW1_NAME                "ccg_1.cyacd"
#define FW2_NAME                "ccg_2.cyacd"
#define FW1_METADATA_ROW        0x1FF
#define FW2_METADATA_ROW        0x1FE

enum enum_fw_mode {
	BOOT,   /* bootloader */
	FW1,    /* FW partition-1 */
	FW2,    /* FW partition-2 */
	FW_INVALID,
};

/* CCGx events & async msg codes */
#define RESET_COMPLETE		0x80
#define EVENT_INDEX		RESET_COMPLETE
#define PORT_CONNECT_DET	0x84
#define PORT_DISCONNECT_DET	0x85
#define ROLE_SWAP_COMPELETE	0x87

/* CCGx response codes */
enum ccg_resp_code {
	CMD_NO_RESP             = 0x00,
	CMD_SUCCESS             = 0x02,
	FLASH_DATA_AVAILABLE    = 0x03,
	CMD_INVALID             = 0x05,
	FLASH_UPDATE_FAIL       = 0x07,
	INVALID_FW              = 0x08,
	INVALID_ARG             = 0x09,
	CMD_NOT_SUPPORT         = 0x0A,
	TRANSACTION_FAIL        = 0x0C,
	PD_CMD_FAIL             = 0x0D,
	UNDEF_ERROR             = 0x0F,
	INVALID_RESP		= 0x10,
};

static const char * const ccg_resp_strs[] = {
	/* 0x00 */ "No Response.",
	/* 0x01 */ "0x01",
	/* 0x02 */ "HPI Command Success.",
	/* 0x03 */ "Flash Data Available in data memory.",
	/* 0x04 */ "0x04",
	/* 0x05 */ "Invalid Command.",
	/* 0x06 */ "0x06",
	/* 0x07 */ "Flash write operation failed.",
	/* 0x08 */ "Firmware validity check failed.",
	/* 0x09 */ "Command failed due to invalid arguments.",
	/* 0x0A */ "Command not supported in the current mode.",
	/* 0x0B */ "0x0B",
	/* 0x0C */ "Transaction Failed. GOOD_CRC was not received.",
	/* 0x0D */ "PD Command Failed.",
	/* 0x0E */ "0x0E",
	/* 0x0F */ "Undefined Error",
};

static const char * const ccg_evt_strs[] = {
	/* 0x80 */ "Reset Complete.",
	/* 0x81 */ "Message queue overflow detected.",
	/* 0x82 */ "Overcurrent Detected",
	/* 0x83 */ "Overvoltage Detected",
	/* 0x84 */ "Type-C Port Connect Detected",
	/* 0x85 */ "Type-C Port Disconnect Detected",
	/* 0x86 */ "PD Contract Negotiation Complete",
	/* 0x87 */ "SWAP Complete",
	/* 0x88 */ "0x88",
	/* 0x89 */ "0x89",
	/* 0x8A */ "PS_RDY Message Received",
	/* 0x8B */ "GotoMin Message Received.",
	/* 0x8C */ "Accept Message Received",
	/* 0x8D */ "Reject Message Received",
	/* 0x8E */ "Wait Message Received",
	/* 0x8F */ "Hard Reset Received",
	/* 0x90 */ "VDM Received",
	/* 0x91 */ "Source Capabilities Message Received",
	/* 0x92 */ "Sink Capabilities Message Received",
	/* 0x93 */ "Display Port Alternate Mode entered",
	/* 0x94 */ "Display Port device connected at UFP_U",
	/* 0x95 */ "Display port device not connected at UFP_U",
	/* 0x96 */ "Display port SID not found in Discover SID process",
	/* 0x97 */ "Multiple SVIDs discovered along with DisplayPort SID",
	/* 0x98 */ "DP Functionality not supported by Cable",
	/* 0x99 */ "Display Port Configuration not supported by UFP",
	/* 0x9A */ "Hard Reset Sent to Port Partner",
	/* 0x9B */ "Soft Reset Sent to Port Partner",
	/* 0x9C */ "Cable Reset Sent to EMCA",
	/* 0x9D */ "Source Disabled State Entered",
	/* 0x9E */ "Sender Response Timer Timeout",
	/* 0x9F */ "No VDM Response Received",
	/* 0xA0 */ "Unexpected Voltage on Vbus",
	/* 0xA1 */ "Type-C Error Recovery",
	/* 0xA2 */ "0xA2",
	/* 0xA3 */ "0xA3",
	/* 0xA4 */ "0xA4",
	/* 0xA5 */ "0xA5",
	/* 0xA6 */ "EMCA Detected",
	/* 0xA7 */ "0xA7",
	/* 0xA8 */ "0xA8",
	/* 0xA9 */ "0xA9",
	/* 0xAA */ "Rp Change Detected",
};

struct ccg_cmd {
	u16 reg;
	u32 data;
	int len;
};

struct ccg_resp {
	u8 code;
	u8 length;
};

struct ccg_dev_info {
	u8 fw_mode:2;
	u8 two_pd_ports:2;
	u8 row_size_256:2;
	u8:1; /* reserved */
	u8 HPIv2_mode:1;
	u8 bl_mode:1;
	u8 cfgtbl_invalid:1;
	u8 fw1_invalid:1;
	u8 fw2_invalid:1;
	u8:4; /* reserved */
	u16 silicon_id;
	u16 bl_last_row;
} __packed;

struct version_format {
	u16 build;
	u8 patch;
	u8 min:4;
	u8 maj:4;
};

struct version_info {
	struct version_format base;
	struct version_format app;
};

struct ccg_typec_pd_info {
	u8 dflt_data_role:2;
	u8 drd_dflt:1;
	u8 dflt_power_role:2;
	u8 drp_dflt:1;
	u8 cur_data_role:1;
	u8:1; /* b7 reserved */
	u8 cur_power_role:1;
	u8:1; /* b9 reserved */
	u8 pd_contract:1;
	u8 emca_present:1;
	u8 vconn_supplier:1;
	u8 vconn_status:1;
	u8 pd3_rp_status:1;
	u8 pd_rdy:1;
	u8 pd_rev:2;
	u8 pd3_supported:1;
	u8 ext_msg_supported:1;
	u8 emca_pd_rev:2;
	u8:2; /* b22-23 reserved */
	u8:8; /* b31-24 reserved */
	u8 connect_status:1;
	u8 cc_polarity:1;
	u8 dev_type:3;
	u8 ra_status:1;
	u8 typec_current:2;
} __packed;

struct ucsi_ccg_extcon {
	struct device dev;
	struct extcon_dev *edev;
	struct mutex lock;
	int port_id;
	int cstate;
};

struct ucsi_ccg {
	struct device *dev;
	struct i2c_client *i2c_cl;
	unsigned int irq;
	struct mutex lock;
	struct ucsi *ucsi;
	struct ucsi_ppm ppm;

	struct ucsi_ccg_extcon *typec_extcon[2];
	int port_num;

	/* CCG HPI communication flags */
	unsigned long flags;
#define RESET_PENDING	0
#define DEV_CMD_PENDING	1
#define PD0_CMD_PENDING 2
#define PD1_CMD_PENDING 3
#define INIT_PENDING	4
	struct ccg_resp dev_resp;
	struct ccg_resp pd_resp[2];
	u8 cmd_resp;
	struct completion hpi_cmd_done;
	struct completion reset_comp;
	struct ccg_dev_info info;
	struct ccg_typec_pd_info port[2];
};

static int ccg_reg_write
(struct ucsi_ccg *ccg, u16 reg, const void *data, unsigned int nbytes)
{
	struct i2c_client *client = ccg->i2c_cl;
	u8 buf[16];
	int ret;

	if (nbytes > (sizeof(buf) - 2))
		return -EINVAL;

	buf[0] = reg & 0xFF;
	buf[1] = reg >> 8;
	memcpy(buf + 2, data, nbytes);

	ret = i2c_master_send(client, buf, nbytes + 2);

	if (ret != nbytes + 2)
		return ret < 0 ? ret : -EIO;

	return 0;
}

static int ccg_reg_read
(struct ucsi_ccg *ccg, u16 reg, u8 *data, unsigned int nbytes)
{
	struct i2c_client *client = ccg->i2c_cl;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = (u8 *)&reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = nbytes,
			.buf = data,
		},
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;

	return 0;
}

static int ucsi_ccg_cmd(struct ucsi_ppm *ppm, struct ucsi_control *ctrl)
{
	struct ucsi_ccg *ccg = container_of(ppm, struct ucsi_ccg, ppm);
	int err;

	ppm->data->ctrl.raw_cmd = ctrl->raw_cmd;

	mutex_lock(&ccg->lock);
	err = ccg_reg_write(ccg, REG_UCSI_CONTROL_CMD,
			&ppm->data->ctrl.raw_cmd, 8);
	mutex_unlock(&ccg->lock);

	dev_dbg(ccg->dev, "Raw cmd 0x%08llx\n", ppm->data->ctrl.raw_cmd);

	return err;
}

static int ucsi_ccg_sync(struct ucsi_ppm *ppm)
{
	struct ucsi_ccg *ccg = container_of(ppm, struct ucsi_ccg, ppm);
	int err, i;

	err = ccg_reg_read(ccg, REG_UCSI_VERSION,
			   (u8 *)&ppm->data->version, 32);

	if (err)
		dev_err(ccg->dev, "ucsi_ccg_sync: i2c read failed\n");
	else {
		dev_dbg(ccg->dev, "raw_cci 0x%08x\n", ppm->data->raw_cci);

		if (ppm->data->cci.data_length) {
			for (i = 0; i < 4; i++)
				dev_dbg(ccg->dev, "msg_in[%d] 0x%08x\n",
						i, ppm->data->message_in[i]);
		}
	}

	return 0;
}

static void ucsi_ccg_notify(struct ucsi_ccg *ccg)
{
	ucsi_notify(ccg->ucsi);
}

static int get_typec_info(struct ucsi_ccg *ccg, int port)
{
	int ret;

	if (port > ccg->info.two_pd_ports)
		return -EINVAL;

	ret = ccg_reg_read(ccg, REG_PD_STATUS(port),
			(u8 *)(&ccg->port[port]),
			sizeof(ccg->port[port]));

	if (ret)
		return -EIO;

	return 0;
}

void show_typec_info(struct ucsi_ccg *ccg, int port)
{
	struct device *dev = ccg->dev;

	dev_info(dev, "port%d typec_info:\n"
		"dflt_data_role: %d\n"
		"drd_dflt: %d\n"
		"dflt_power_role: %d\n"
		"drp_dflt: %d\n"
		"cur_data_role: %s\n"
		"cur_power_role: %s\n"
		"pd_contract: %d\n"
		"pd_revison: %d\n"
		"connect_status: %d\n"
		"cc_polarity: %d\n"
		"dev_type: %d\n"
		"typec_current: %d\n", port, ccg->port[port].dflt_data_role,
		ccg->port[port].drd_dflt, ccg->port[port].dflt_power_role,
		ccg->port[port].drp_dflt,
		(ccg->port[port].cur_data_role) ? "DFP" : "UFP",
		(ccg->port[port].cur_power_role) ? "Source" : "Sink",
		ccg->port[port].pd_contract, ccg->port[port].pd_rev,
		ccg->port[port].connect_status, ccg->port[port].cc_polarity,
		ccg->port[port].dev_type, ccg->port[port].typec_current);
}

static int get_fw_info(struct ucsi_ccg *ccg)
{
	struct device *dev = ccg->dev;
	struct version_info version[3];
	struct version_info *v;
	int err, i;

	err = ccg_reg_read(ccg, REG_READ_ALL_VER, (u8 *)(&version), 24);
	if (err) {
		dev_err(dev, "read version failed\n");
		return err;
	}

	for (i = 1; i < 3; i++) {
		v = &version[i];
		dev_info(dev,
		"FW%d Version: %c%c v%x.%x%x, [Base %d.%d.%d.%d]\n",
		i, (v->app.build >> 8), (v->app.build & 0xFF),
		v->app.patch, v->app.maj, v->app.min,
		v->base.maj, v->base.min, v->base.patch,
		v->base.build);
	}

	err = ccg_reg_read(ccg, REG_DEVICE_MODE, (u8 *)(&ccg->info), 6);
	if (err) {
		dev_err(dev, "read device mode failed\n");
		return err;
	}

	dev_info(dev, "fw_mode: %d\n", ccg->info.fw_mode);
	dev_info(dev, "fw1_invalid: %d\n", ccg->info.fw1_invalid);
	dev_info(dev, "fw2_invalid: %d\n", ccg->info.fw2_invalid);
	dev_info(dev, "silicon_id: 0x%04x\n", ccg->info.silicon_id);

	return 0;
}

static inline bool invalid_resp(int code)
{
	return (code >= INVALID_RESP);
}

static inline bool invalid_evt(int code)
{
	return (code >= 0xAB) || (code < 0x80);
}

static void update_typec_extcon_state(struct ucsi_ccg *ccg, int index)
{
	struct ucsi_ccg_extcon *extcon;
	struct ccg_typec_pd_info *info;
	int last_cstate;

	info = &ccg->port[index];
	extcon = ccg->typec_extcon[index];

	if (!extcon || !info)
		return;

	mutex_lock(&extcon->lock);

	last_cstate = extcon->cstate;

	get_typec_info(ccg, index);

	if (!info->connect_status)
		extcon->cstate = EXTCON_NONE;
	else {
		if (!info->cur_data_role)
			extcon->cstate = EXTCON_USB;
		else
			extcon->cstate = EXTCON_USB_HOST;
	}

	if (last_cstate == extcon->cstate) {
		mutex_unlock(&extcon->lock);
		return;
	}

	if (last_cstate) {
		extcon_set_state_sync(extcon->edev, last_cstate, 0);
		dev_info(ccg->dev,
			"[typec-port%d] Cable state:0, cable id:%d\n",
			extcon->port_id, last_cstate);
	}

	if (extcon->cstate) {
		extcon_set_state_sync(extcon->edev, extcon->cstate, 1);
		dev_info(ccg->dev,
			"[typec-port%d] Cable state:%d, cable id:%d\n",
			extcon->port_id, !!extcon->cstate, extcon->cstate);
	}

	mutex_unlock(&extcon->lock);
}

static void ccg_pd_event(struct ucsi_ccg *ccg, int index)
{
	struct device *dev = ccg->dev;
	struct ccg_resp *resp = &ccg->pd_resp[index];

	if (ccg_reg_read(ccg, REG_PD_RESPONSE(index), (u8 *)resp, 2)) {
		dev_err(dev, "read REG_PD_RESPONSE(%d) failed\n", index);
		return;
	}
	dev_dbg(dev, "port%d event code: 0x%02x, data len: %d\n",
			index, resp->code, resp->length);

	if (resp->code & ASYNC_EVENT) {
		/* Ignore Async events before ccg init */
		if (test_bit(INIT_PENDING, &ccg->flags))
			return;

		if (!invalid_evt(resp->code)) {
			dev_info(dev, "port%d evt: %s\n",
				index, ccg_evt_strs[resp->code - EVENT_INDEX]);

			switch (resp->code) {
			case PORT_DISCONNECT_DET:
			case PORT_CONNECT_DET:
			case ROLE_SWAP_COMPELETE:
				update_typec_extcon_state(ccg, index);
				break;
			default:
				break;
			}
		} else
			dev_err(dev, "port%d invalid evt %d\n",
				index, resp->code);
	} else {
		if (test_bit(PD0_CMD_PENDING + index, &ccg->flags)) {
			ccg->cmd_resp = resp->code;
			complete(&ccg->hpi_cmd_done);
			clear_bit(PD0_CMD_PENDING + index, &ccg->flags);
		} else
			dev_err(dev,
				"PD port%d resp 0x%04x but no cmd pending\n",
				index, resp->code);
	}
}

static void ccg_int_handler(struct ucsi_ccg *ccg)
{
	struct device *dev = ccg->dev;
	u8 intval;

	/* Read INTR */
	if (ccg_reg_read(ccg, REG_INTR, (u8 *)(&intval), 1)) {
		dev_err(dev, "read REG_INTR failed\n");
		return;
	}

	/* UCSI event */
	if ((intval & UCSI_READ_INT) && !test_bit(INIT_PENDING, &ccg->flags))
		ucsi_ccg_notify(ccg);

	/* DEV event */
	if (intval & DEV_INT) {
		if (ccg_reg_read(
			ccg, REG_RESPONSE, (u8 *)(&ccg->dev_resp), 2)) {
			dev_err(dev, "read REG_RESPONSE failed\n");
			return;
		}
		dev_info(dev, "dev event code: 0x%02x, data len: %d\n",
				ccg->dev_resp.code, ccg->dev_resp.length);

		if (ccg->dev_resp.code & ASYNC_EVENT) {
			if (ccg->dev_resp.code == RESET_COMPLETE) {
				if (test_bit(RESET_PENDING, &ccg->flags)) {
					ccg->cmd_resp = ccg->dev_resp.code;
					complete(&ccg->reset_comp);
				}
				dev_info(dev, "CCG reset complete\n");
				get_fw_info(ccg);
			}

			if (!invalid_evt(ccg->dev_resp.code))
				dev_info(dev, "%s\n",
				ccg_evt_strs[ccg->dev_resp.code - EVENT_INDEX]);
			else
				dev_err(dev, "dev invalid evt %d\n",
				ccg->dev_resp.code);
		} else {
			if (test_bit(DEV_CMD_PENDING, &ccg->flags)) {
				ccg->cmd_resp = ccg->dev_resp.code;
				complete(&ccg->hpi_cmd_done);
				clear_bit(DEV_CMD_PENDING, &ccg->flags);
			} else
				dev_err(dev,
					"dev resp 0x%04x but no cmd pending\n",
					ccg->dev_resp.code);
		}
	}

	/* PD port0 event */
	if (intval & PORT0_INT)
		ccg_pd_event(ccg, 0);

	/* PD port1 event */
	if (intval & PORT1_INT)
		ccg_pd_event(ccg, 1);

	/* Clear INTR */
	if (intval && ccg_reg_write(ccg, REG_INTR, &intval, 1))
		dev_err(dev, "clear intr fail\n");
}

static irqreturn_t ccg_irq(int irq, void *data)
{
	struct ucsi_ccg *ccg = data;

	ccg_int_handler(ccg);

	return IRQ_HANDLED;
}

static int ucsi_ccg_setup_irq(struct ucsi_ccg *ccg)
{
	struct device *dev = ccg->dev;
	int err = 0;
	u32 irqflags;

	if (ccg->i2c_cl->irq)
		ccg->irq = ccg->i2c_cl->irq;
	else {
		dev_err(dev, "client->irq is NULL\n");
		return -1;
	}

	err = of_property_read_u32(dev->of_node, "ccg,irqflags", &irqflags);
	if (err) {
		dev_warn(dev, "Default irqflags: TRIGGER_LOW\n");
		irqflags = IRQF_TRIGGER_LOW;
	}

	err = devm_request_threaded_irq(dev, ccg->irq, NULL,
			ccg_irq, irqflags | IRQF_ONESHOT,
			"ccg_irq", ccg);

	return err;
}

/* Caller must hold ccg->lock */
static int ccg_send_command(struct ucsi_ccg *ccg, struct ccg_cmd *cmd)
{
	struct device *dev = ccg->dev;
	struct completion *comp;
	unsigned int timeout;
	int ret;

	switch (cmd->reg & 0xF000) {
	case DEV_REG_IDX:
		set_bit(DEV_CMD_PENDING, &ccg->flags);
		break;
	case PD_PORT0_REG_IDX:
		set_bit(PD0_CMD_PENDING, &ccg->flags);
		break;
	case PD_PORT1_REG_IDX:
		set_bit(PD1_CMD_PENDING, &ccg->flags);
		break;
	default:
		dev_err(dev, "%s: Invalid CMD REG\n", __func__);
		break;
	}

	ret = ccg_reg_write(ccg, cmd->reg, &cmd->data, cmd->len);
	if (ret) {
		dev_err(dev, "%s: write fail\n", __func__);
		return ret;
	}

	dev_dbg(dev, "%s: reg=0x%04x data=0x%08x\n",
			__func__, cmd->reg, cmd->data);

	if (test_bit(RESET_PENDING, &ccg->flags)) {
		comp = &ccg->reset_comp;
		timeout = BOOT_WAIT_TIMEOUT_MS;
	} else {
		comp = &ccg->hpi_cmd_done;
		timeout = CCG_CMD_TIMEOUT_MS;
	}

	if (!wait_for_completion_timeout(comp, msecs_to_jiffies(timeout))) {
		dev_err(dev, "%s: timeout\n", __func__);
		switch (cmd->reg & 0xF000) {
		case DEV_REG_IDX:
			clear_bit(DEV_CMD_PENDING, &ccg->flags);
			break;
		case PD_PORT0_REG_IDX:
			clear_bit(PD0_CMD_PENDING, &ccg->flags);
			break;
		case PD_PORT1_REG_IDX:
			clear_bit(PD1_CMD_PENDING, &ccg->flags);
			break;
		default:
			dev_err(dev, "%s: Invalid CMD REG\n", __func__);
			break;
		}
		return -ETIMEDOUT;
	}

	if (!invalid_resp(ccg->cmd_resp))
		dev_info(dev, "%s\n", ccg_resp_strs[ccg->cmd_resp]);
	else
		dev_err(dev, "unknown resp code %d\n", ccg->cmd_resp);

	return ccg->cmd_resp;
}

static int ccg_cmd_select_sink_pdo(struct ucsi_ccg *ccg, int port, int pdo_mask)
{
	struct ccg_cmd cmd;
	int ret;

	cmd.reg = REG_SELECT_SINK_PDO(port);
	cmd.data = pdo_mask;
	cmd.len = 1;

	mutex_lock(&ccg->lock);
	ret = ccg_send_command(ccg, &cmd);
	mutex_unlock(&ccg->lock);

	if (ret != CMD_SUCCESS) {
		dev_err(ccg->dev, "%s: failed ret=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int ccg_cmd_enter_flashing(struct ucsi_ccg *ccg)
{
	struct ccg_cmd cmd;
	int ret;

	cmd.reg = REG_ENTER_FLASHING;
	cmd.data = FLASH_ENTER_SIG;
	cmd.len = 1;

	mutex_lock(&ccg->lock);

	ret = ccg_send_command(ccg, &cmd);

	mutex_unlock(&ccg->lock);

	if (ret != CMD_SUCCESS) {
		dev_err(ccg->dev, "%s: failed ret=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int ccg_cmd_reset(struct ucsi_ccg *ccg)
{
	struct ccg_cmd cmd;
	u8 *p;
	int ret;

	p = (u8 *)&cmd.data;
	cmd.reg = REG_RESET_REQ;
	p[0] = RESET_SIG;
	p[1] = CMD_RESET_DEV;
	cmd.len = 2;

	mutex_lock(&ccg->lock);

	set_bit(RESET_PENDING, &ccg->flags);

	ret = ccg_send_command(ccg, &cmd);
	if (ret != RESET_COMPLETE)
		goto err_clear_flag;

	if (!wait_for_completion_timeout(&ccg->reset_comp,
				msecs_to_jiffies(BOOT_WAIT_TIMEOUT_MS))) {
		dev_err(ccg->dev, "%s: timeout\n", __func__);
		ret = -ETIMEDOUT;
		goto err_clear_flag;
	}

	ret = 0;

err_clear_flag:
	clear_bit(RESET_PENDING, &ccg->flags);

	mutex_unlock(&ccg->lock);

	return ret;
}

static int ccg_cmd_jump_boot_mode(struct ucsi_ccg *ccg, int bl_mode)
{
	struct ccg_cmd cmd;
	int ret;

	cmd.reg = REG_JUMP_TO_BOOT;

	if (bl_mode)
		cmd.data = TO_BOOT;
	else
		cmd.data = TO_ALT_FW;

	cmd.len = 1;

	mutex_lock(&ccg->lock);

	set_bit(RESET_PENDING, &ccg->flags);

	ret = ccg_send_command(ccg, &cmd);
	if (ret != RESET_COMPLETE)
		goto err_clear_flag;

	ret = 0;

err_clear_flag:
	clear_bit(RESET_PENDING, &ccg->flags);

	mutex_unlock(&ccg->lock);

	return ret;
}

#ifdef FW_FLASH_READ_VALIDATE
static int ccg_cmd_read_flash_row(struct ucsi_ccg *ccg, u16 row, u8 *buf)
{
	struct ccg_cmd cmd;
	u8 *p;
	int ret;

	/* Use the FLASH_ROW_READ_WRITE register to trigger reading of the */
	/* data from the desired flash row */
	p = (u8 *)&cmd.data;
	cmd.reg = REG_FLASH_ROW_RW;
	p[0] = FLASH_SIG;
	p[1] = FLASH_RD_CMD;
	p[2] = row & 0xFF;
	p[3] = row >> 8;
	cmd.len = 4;

	mutex_lock(&ccg->lock);

	ret = ccg_send_command(ccg, &cmd);
	if (ret != FLASH_DATA_AVAILABLE) {
		dev_err(ccg->dev, "%s: failed ret=%d\n", __func__, ret);
		return ret;
	}

	/* Read the data from the flash read/write memory */
	ret = ccg_reg_read(ccg, REG_FLASH_RW_MEM, buf, 256);

	mutex_unlock(&ccg->lock);

	return ret;
}
#endif

static int
ccg_cmd_write_flash_row(struct ucsi_ccg *ccg, u16 row, const void *data)
{
	struct i2c_client *client = ccg->i2c_cl;
	struct ccg_cmd cmd;
	u8 buf[CCG4_ROW_SIZE + 2];
	u8 *p;
	int ret;

	/* Copy the data into the flash read/write memory. */
	buf[0] = REG_FLASH_RW_MEM & 0xFF;
	buf[1] = REG_FLASH_RW_MEM >> 8;

	memcpy(buf + 2, data, CCG4_ROW_SIZE);

	mutex_lock(&ccg->lock);

	ret = i2c_master_send(client, buf, CCG4_ROW_SIZE + 2);
	if (ret != CCG4_ROW_SIZE + 2) {
		dev_err(ccg->dev, "REG_FLASH_RW_MEM write fail %d\n", ret);
		return ret < 0 ? ret : -EIO;
	}

	/* Use the FLASH_ROW_READ_WRITE register to trigger */
	/* writing of data to the desired flash row */
	p = (u8 *)&cmd.data;
	cmd.reg = REG_FLASH_ROW_RW;
	p[0] = FLASH_SIG;
	p[1] = FLASH_WR_CMD;
	p[2] = row & 0xFF;
	p[3] = row >> 8;
	cmd.len = 4;
	ret = ccg_send_command(ccg, &cmd);

	mutex_unlock(&ccg->lock);

	if (ret != CMD_SUCCESS) {
		dev_err(ccg->dev, "%s: failed ret=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int ccg_cmd_validate_fw(struct ucsi_ccg *ccg, unsigned int fwid)
{
	struct ccg_cmd cmd;
	int ret;

	cmd.reg = REG_VALIDATE_FW;
	cmd.data = fwid;
	cmd.len = 1;

	mutex_lock(&ccg->lock);

	ret = ccg_send_command(ccg, &cmd);

	mutex_unlock(&ccg->lock);

	if (ret != CMD_SUCCESS)
		return ret;

	return 0;
}

static int ccg_cmd_update_event_mask(struct ucsi_ccg *ccg, int port)
{
	struct ccg_cmd cmd;
	u16 mask = OC_DET | OV_DET | CONN_DET | PD_EVT_DET;
	int ret;

	cmd.reg = REG_EVENT_MASK(port);
	cmd.data = mask;
	cmd.len = 2;

	mutex_lock(&ccg->lock);
	ret = ccg_send_command(ccg, &cmd);
	mutex_unlock(&ccg->lock);

	if (ret != CMD_SUCCESS) {
		dev_err(ccg->dev, "%s: failed ret=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int do_flash(struct ucsi_ccg *ccg, const char *fw_name)
{
	const struct firmware *fw;
	struct device *dev = ccg->dev;
	const char *p, *s;
	const char *eof;
	int err, row, len, line_sz, line_cnt = 0;
	unsigned long start_time = jiffies;
	u8 *wr_buf;
#ifdef FW_FLASH_READ_VALIDATE
	u8 *rd_buf;
#endif

	dev_info(dev, "Start flash %s\n", fw_name);

	err = request_firmware(&fw, fw_name, dev);
	if (err) {
		dev_err(dev, "request %s failed err=%d\n", fw_name, err);
		return err;
	}

	eof = fw->data + fw->size;

#ifdef FW_FLASH_READ_VALIDATE
	rd_buf = kzalloc(CCG4_ROW_SIZE, GFP_KERNEL);
	if (!rd_buf)
		return -ENOMEM;
#endif
	wr_buf = kzalloc(CCG4_ROW_SIZE + 4, GFP_KERNEL);
	if (!wr_buf)
		return -ENOMEM;

	err = ccg_cmd_enter_flashing(ccg);
	if (err)
		goto release_mem;

	/*****************************************************************
	 * CCG firmware image (.cyacd) file line format
	 *
	 * :00rrrrllll[dd....]cc/r/n
	 *
	 * :00   header
	 * rrrr is row number to flash				(4 char)
	 * llll is data len to flash				(4 char)
	 * dd   is a data field represents one byte of data	(512 char)
	 * cc   is checksum					(2 char)
	 * \r\n newline
	 *
	 * Total length: 3 + 4 + 4 + 512 + 2 + 2 = 527
	 *
	 *****************************************************************/

	p = strchr(fw->data, ':');
	while (p < eof) {

		s = strnchr(p + 1, CYACD_LINE_SIZE, ':');

		if (s == NULL)
			s = eof;

		line_sz = s - p;

		if (line_sz != CYACD_LINE_SIZE) {
			dev_err(dev, "Bad FW format line_sz=%d\n", line_sz);
			err =  -EINVAL;
			goto release_mem;
		}

		if (hex2bin(wr_buf, p + 3, CCG4_ROW_SIZE + 4)) {
			err =  -EINVAL;
			goto release_mem;
		}

		row = (wr_buf[0] << 8) + wr_buf[1];
		len = (wr_buf[2] << 8) + wr_buf[3];

		if (len != CCG4_ROW_SIZE) {
			err =  -EINVAL;
			goto release_mem;
		}

		err = ccg_cmd_write_flash_row(ccg, row, wr_buf + 4);
		if (err) {
			dev_err(dev, "Write row %d failed %d\n", row, err);
			goto release_mem;
		}

		line_cnt++;
		p = s;

#ifdef FW_FLASH_READ_VALIDATE

		/* Use the FLASH_ROW_READ_WRITE register to trigger reading */
		/* of the data from the desired flash row and verify */

		err = ccg_cmd_read_flash_row(ccg, row, rd_buf);
		if (err) {
			dev_err(dev, "Read row %d failed %d\n", row, err);
			goto release_mem;
		} else {
			if (row == FW1_METADATA_ROW || row == FW2_METADATA_ROW)
				continue;

			if (memcmp(rd_buf, wr_buf + 4, CCG4_ROW_SIZE)) {
				dev_err(dev,
				"Row=%02x: rd isn't equal to wr\n", row);
				err = -EIO;
				goto release_mem;
			}
		}
#endif
	}

	dev_info(dev, "%s: Total %d row flashed. Time: %dms\n", __func__,
		 line_cnt, jiffies_to_msecs(jiffies - start_time));

	if (strncmp(fw_name, FW1_NAME, sizeof(FW1_NAME)))
		err = ccg_cmd_validate_fw(ccg, FW1);
	else
		err = ccg_cmd_validate_fw(ccg, FW2);

	if (err)
		dev_err(dev, "FW validate failed err=%d\n", err);

release_mem:
	release_firmware(fw);
	kfree(wr_buf);
#ifdef FW_FLASH_READ_VALIDATE
	kfree(rd_buf);
#endif
	return err;
}

/*******************************************************************************
 * CCG4 has two copies of the firmware in addition to the bootloader.
 * If the device is running FW1, FW2 can be updated with the new version.
 * Dual firmware mode allows the CCG device to stay in a PD contract and support
 * USB PD and Type-C functionality while a firmware update is in progress.
 ******************************************************************************/
static int ccg_fw_update(struct ucsi_ccg *ccg, int flashing_fw)
{
	int err;

	get_fw_info(ccg);

	if (flashing_fw == FW1) {
		if (ccg->info.fw_mode == FW1)
			ccg_cmd_jump_boot_mode(ccg, 0); // jump tp ALT FW
		err = do_flash(ccg, FW1_NAME);
	} else if (flashing_fw == FW2) {
		if (ccg->info.fw_mode == FW2)
			ccg_cmd_jump_boot_mode(ccg, 0);
		err = do_flash(ccg, FW2_NAME);
	} else {
		/* Flash both partiton */
		ccg_cmd_jump_boot_mode(ccg, 1); // jump to BOOT
		err = do_flash(ccg, FW1_NAME);
		err = do_flash(ccg, FW2_NAME);
	}

	return err;
}

static int ucsi_ccg_init(struct ucsi_ccg *ccg)
{
	struct device *dev = ccg->dev;
	int i;

	/* Asymmetric FW should always run on FW2 (primary FW) */
	if (ccg->info.fw_mode != FW2) {
		dev_err(dev, "CCG can't boot to FW mode\n");
		clear_bit(INIT_PENDING, &ccg->flags);
		return -EINVAL;
	}

	/* Perform initial detection */
	for (i = 0; i < ccg->port_num; i++)
		update_typec_extcon_state(ccg, i);

	/* Update event mask for ports */
	ccg_cmd_update_event_mask(ccg, 0);

	if (ccg->info.two_pd_ports)
		ccg_cmd_update_event_mask(ccg, 1);

	clear_bit(INIT_PENDING, &ccg->flags);

	ccg->ucsi = ucsi_register_ppm(dev, &ccg->ppm);
	if (IS_ERR(ccg->ucsi)) {
		dev_err(dev, "ucsi_register_ppm failed\n");
		return PTR_ERR(ccg->ucsi);
	}

	dev_info(dev, "%s: complete\n", __func__);

	return 0;
}

static ssize_t do_flash_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t n)
{
	struct i2c_client *i2c_cl = to_i2c_client(dev);
	struct ucsi_ccg *ccg = i2c_get_clientdata(i2c_cl);
	unsigned int mode;

	if (kstrtouint(buf, 10, &mode))
		return -EINVAL;

	ccg_fw_update(ccg, mode);

	return n;
}

static ssize_t do_flash_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "echo 1: flash FW1.\necho 2: flash FW2.\n"
			"echo 3: flash both.\n");
}

static ssize_t test_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t n)
{
	struct i2c_client *i2c_cl = to_i2c_client(dev);
	struct ucsi_ccg *ccg = i2c_get_clientdata(i2c_cl);
	unsigned int mode;

	if (kstrtouint(buf, 10, &mode))
		return -EINVAL;

	if (mode == 1)
		ccg_cmd_reset(ccg);
	else if (mode == 2) {
		get_typec_info(ccg, 0); // query port1 pd status
		show_typec_info(ccg, 0);
	} else if (mode == 3) {
		get_typec_info(ccg, 1); // query port2 pd status
		show_typec_info(ccg, 1);
	} else if (mode == 4)
		ccg_cmd_select_sink_pdo(ccg, 1, 0xF); // change P2 PDO to 15V
	else if (mode == 5)
		ccg_cmd_select_sink_pdo(ccg, 1, 0x1F); // change P2 PDO to 20V

	return n;
}

static ssize_t test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *i2c_cl = to_i2c_client(dev);
	struct ucsi_ccg *ccg = i2c_get_clientdata(i2c_cl);

	get_fw_info(ccg);

	return sprintf(buf,
		"1: reset\n"
		"2: P1 PD status\n"
		"3: P2 PD status\n"
		"4: selectPDO_15V\n"
		"5: selectPDO_20V\n");

}

static DEVICE_ATTR(do_flash, 0644, do_flash_show, do_flash_store);
static DEVICE_ATTR(test, 0644, test_show, test_store);

static struct attribute *
ucsi_ccg_sysfs_attrs[] = {
	&dev_attr_do_flash.attr,
	&dev_attr_test.attr,
	NULL,
};

static struct attribute_group
ucsi_ccg_attr_group = {
	.attrs = ucsi_ccg_sysfs_attrs,
};

static int ucsi_ccg_extcon_cables[] = {
	EXTCON_USB, EXTCON_USB_HOST, EXTCON_NONE
};

static struct device_node *
ucsi_ccg_find_typec_extcon_node(struct device *dev, unsigned int index)
{
	/*
	 * of_find_node_by_name() drops a reference, so make sure to grab one.
	 */
	struct device_node *np = of_node_get(dev->of_node);

	np = of_find_node_by_name(np, "typec-extcon");
	if (np) {
		char *name;

		name = kasprintf(GFP_KERNEL, "port-%u", index);
		np = of_find_node_by_name(np, name);
		kfree(name);
	} else
		dev_err(dev, "failed to find typec-extcon node\n");

	return np;
}

static struct ucsi_ccg_extcon *
ucsi_ccg_extcon_probe
(struct device *parent, unsigned int i, struct device_node *np)
{
	struct ucsi_ccg_extcon *extcon;
	int err;

	extcon = devm_kzalloc(parent, sizeof(*extcon), GFP_KERNEL);
	if (!extcon)
		return extcon;

	extcon->port_id = i;
	extcon->dev.parent = parent;
	extcon->dev.of_node = np;

	dev_set_name(&extcon->dev, "typec-extcon-%u", extcon->port_id);

	err = device_register(&extcon->dev);
	if (err)
		return NULL;

	extcon->edev = devm_extcon_dev_allocate(&extcon->dev,
						ucsi_ccg_extcon_cables);
	if (IS_ERR(extcon->edev))
		goto unregister;

	mutex_init(&extcon->lock);

	err = devm_extcon_dev_register(&extcon->dev, extcon->edev);
	if (err < 0)
		goto unregister;

	return extcon;

unregister:
	device_unregister(&extcon->dev);

	return NULL;
}

static int ucsi_ccg_setup_extcon(struct ucsi_ccg *ccg)
{
	struct device *dev = ccg->dev;
	int i;

	for (i = 0; i < ccg->port_num; i++) {
		struct ucsi_ccg_extcon *extcon;
		struct device_node *np;

		np = ucsi_ccg_find_typec_extcon_node(dev, i);
		if (!np || !of_device_is_available(np)) {
			of_node_put(np);
			continue;
		}

		extcon = ucsi_ccg_extcon_probe(dev, i, np);
		if (extcon) {
			dev_info(dev, "typec-port%d extcon dev created\n", i);
			ccg->typec_extcon[i] = extcon;
		} else
			return -EINVAL;
	}

	return 0;
}

static int ucsi_ccg_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ucsi_ccg *ccg;
	struct device *dev = &client->dev;
	int err = 0;

	ccg = devm_kzalloc(dev, sizeof(struct ucsi_ccg), GFP_KERNEL);

	if (ccg == NULL)
		return -ENOMEM;

	ccg->i2c_cl = client;
	ccg->dev = dev;

	mutex_init(&ccg->lock);

	init_completion(&ccg->hpi_cmd_done);
	init_completion(&ccg->reset_comp);

	err = get_fw_info(ccg);
	if (err) {
		dev_err(dev, "get_fw_info fail,, err=%d\n", err);
		return err;
	}

	if (ccg->info.two_pd_ports)
		ccg->port_num = 2;
	else
		ccg->port_num = 1;

	err = ucsi_ccg_setup_extcon(ccg);
	if (err) {
		dev_err(dev, "Setup extcon fail, err=%d\n", err);
		return err;
	}

	set_bit(INIT_PENDING, &ccg->flags);

	err = ucsi_ccg_setup_irq(ccg);
	if (err) {
		dev_err(dev, "Setup irq fail, err=%d\n", err);
		return err;
	}

	ccg->ppm.data = devm_kzalloc(dev, sizeof(struct ucsi_data), GFP_KERNEL);
	if (!ccg->ppm.data) {
		dev_err(dev, "failed to create ccg->ppm.data\n");
		return -ENOMEM;
	}

	ccg->ppm.cmd = ucsi_ccg_cmd;
	ccg->ppm.sync = ucsi_ccg_sync;

	ucsi_ccg_init(ccg);

	i2c_set_clientdata(client, ccg);

	err = sysfs_create_group(&dev->kobj, &ucsi_ccg_attr_group);
	if (err)
		dev_err(dev, "cannot create sysfs group: %d\n", err);

	return err;
}

static int ucsi_ccg_remove(struct i2c_client *client)
{
	struct ucsi_ccg *ccg = i2c_get_clientdata(client);

	if (ccg->irq)
		free_irq(ccg->irq, ccg);

	return 0;
}

static const struct of_device_id ucsi_ccg_of_match_table[] = {
	{ .compatible = "nvidia,ucsi_ccg", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ucsi_ccg_of_match_table);

static const struct i2c_device_id ucsi_ccg_i2c_ids[] = {
	{ "ucsi_ccg", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ucsi_ccg_i2c_ids);

static struct i2c_driver ucsi_ccg_driver = {
	.driver = {
		.name = "ucsi_ccg",
		.of_match_table = ucsi_ccg_of_match_table,
	},
	.id_table = ucsi_ccg_i2c_ids,
	.probe = ucsi_ccg_probe,
	.remove = ucsi_ccg_remove,
};
module_i2c_driver(ucsi_ccg_driver);

MODULE_AUTHOR("Allie Liu <alliel@nvidia.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("UCSI CCG driver");
