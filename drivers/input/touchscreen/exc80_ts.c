/*
 *
 * Touch Screen I2C Driver for EETI Controller
 *
 * Copyright (C) 2000-2017  eGalax_eMPIA Technology Inc. All rights reserved.
 * Copyright (c) 2017-2018 NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define RELEASE_DATE "2017/12/20"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/kfifo.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

/* Global define to enable function */
/* #define _SWITCH_XY */
/* #define _CONVERT_Y */

#define MAX_EVENTS		600
#define MAX_I2C_LEN		64U
#define FIFO_SIZE		8192
#define MAX_SUPPORT_POINT	16
#define REPORTID_VENDOR		0x03
#define REPORTID_MTOUCH		0x06
#define MAX_RESOLUTION		4095
#define MAX_Z_RESOLUTION	1023

/* running mode */
#define MODE_STOP	0
#define MODE_WORKING	1
#define MODE_IDLE	2
#define MODE_SUSPEND	3

struct tagMTContacts {
	unsigned char ID;
	signed char Status;
	unsigned short X;
	unsigned short Y;
	unsigned short Z;
};

struct _egalax_i2c {
	struct workqueue_struct *ktouch_wq;
	struct work_struct work_irq;
	struct delayed_work delay_work_ioctl;
	struct mutex mutex_wq;
	struct i2c_client *client;
	unsigned char work_state;
	unsigned char skip_packet;
	unsigned int ioctl_cmd;
	int interrupt_gpio;
	int reset_gpio;
	bool enable_high;
	wait_queue_head_t sysfs_query_queue;
	bool sysfs_query_wait;
	unsigned char sysfs_hook_cmd[3];
	unsigned char sysfs_cmd_result[MAX_I2C_LEN];
	struct regulator        *regulator_hv;
	struct regulator        *regulator_5v0;
	struct regulator        *regulator_3v3;
	struct regulator        *regulator_1v8;
};

struct egalax_char_dev {
	int OpenCnts;
	struct kfifo DataKFiFo;
	unsigned char *pFiFoBuf;
	spinlock_t FiFoLock;
	struct semaphore sem;
	wait_queue_head_t fifo_inq;
};

static struct _egalax_i2c *p_egalax_i2c_dev;
static struct egalax_char_dev *p_char_dev;
static atomic_t egalax_char_available = ATOMIC_INIT(1);
static atomic_t wait_command_ack = ATOMIC_INIT(0);
static struct input_dev *input_dev;
static struct tagMTContacts pContactBuf[MAX_SUPPORT_POINT];
static unsigned char input_report_buf[MAX_I2C_LEN+2];
static char fifo_read_buf[MAX_I2C_LEN];
static int TotalPtsCnt, RecvPtsCnt;

#define DBG_MODULE	0x00000001U
#define DBG_CDEV	0x00000002U
#define DBG_PROC	0x00000004U
#define DBG_POINT	0x00000008U
#define DBG_INT		0x00000010U
#define DBG_I2C		0x00000020U
#define DBG_SUSP	0x00000040U
#define DBG_INPUT	0x00000080U
#define DBG_CONST	0x00000100U
#define DBG_IDLE	0x00000200U
#define DBG_WAKEUP	0x00000400U
#define DBG_BUTTON	0x00000800U
static unsigned int DbgLevel = DBG_MODULE|DBG_SUSP;

#define PROC_FS_NAME	"egalax_dbg"
#define PROC_FS_MAX_LEN	8
static struct proc_dir_entry *dbgProcFile;

#define EGALAX_DBG(level, fmt, args...) \
do { if ((level & DbgLevel) > 0U) { \
pr_debug("egalax_i2c: " fmt, ## args); } \
} while (false)

static int egalax_I2C_read(unsigned char *pBuf, unsigned short len)
{
	struct i2c_msg xfer;

	if (pBuf == NULL)
		return -1;

	/* Read device data */
	xfer.addr = p_egalax_i2c_dev->client->addr;
	xfer.flags = I2C_M_RD;
	xfer.len = len;
	xfer.buf = pBuf;

	if (i2c_transfer(p_egalax_i2c_dev->client->adapter, &xfer, 1) != 1) {
		EGALAX_DBG(DBG_I2C, " %s: i2c transfer fail\n", __func__);
		return -EIO;
	}

	EGALAX_DBG(DBG_I2C, " %s: i2c transfer success\n", __func__);

	return 0;
}

static int egalax_I2C_write(unsigned short reg, unsigned char *pBuf,
			    unsigned short len)
{
	unsigned char cmdbuf[4+len];
	struct i2c_msg xfer;

	if (pBuf == NULL)
		return -1;

	cmdbuf[0] = reg & 0x00FFU;
	cmdbuf[1] = (reg >> 8) & 0x00FFU;
	cmdbuf[2] = (len+2) & 0x00FFU;
	cmdbuf[3] = ((len+2) >> 8) & 0x00FFU;
	memcpy(cmdbuf+4, pBuf, len);

	/* Write data to device */
	xfer.addr = p_egalax_i2c_dev->client->addr;
	xfer.flags = 0;
	xfer.len = sizeof(cmdbuf);
	xfer.buf = cmdbuf;

	if (i2c_transfer(p_egalax_i2c_dev->client->adapter, &xfer, 1) != 1) {
		EGALAX_DBG(DBG_I2C, " %s: i2c transfer fail\n", __func__);
		return -EIO;
	}

	EGALAX_DBG(DBG_I2C, " %s: i2c transfer success\n", __func__);

	return 0;
}

static int egalax_cdev_open(struct inode *inode, struct file *filp)
{
	if (!atomic_dec_and_test(&egalax_char_available)) {
		atomic_inc(&egalax_char_available);
		return -EBUSY;
	}

	p_char_dev->OpenCnts++;
	filp->private_data = p_char_dev;

	EGALAX_DBG(DBG_CDEV, " CDev open done!\n");
	try_module_get(THIS_MODULE);
	return 0;
}

static int egalax_cdev_release(struct inode *inode, struct file *filp)
{
	struct egalax_char_dev *cdev = filp->private_data;

	atomic_inc(&egalax_char_available);

	cdev->OpenCnts--;

	kfifo_reset(&cdev->DataKFiFo);

	EGALAX_DBG(DBG_CDEV, " CDev release done!\n");
	module_put(THIS_MODULE);
	return 0;
}

static ssize_t egalax_cdev_read(struct file *file, char __user *buf,
				size_t count, loff_t *offset)
{
	int read_cnt, ret, fifoLen;
	struct egalax_char_dev *cdev = file->private_data;

	if (down_interruptible(&cdev->sem))
		return -ERESTARTSYS;

	fifoLen = kfifo_len(&cdev->DataKFiFo);

	while (fifoLen < 1) {
		/* release the lock */
		up(&cdev->sem);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (wait_event_interruptible(cdev->fifo_inq,
					kfifo_len(&cdev->DataKFiFo) > 0)) {
			/* signal: tell the fs layer to handle it */
			return -ERESTARTSYS;
		}

		if (down_interruptible(&cdev->sem))
			return -ERESTARTSYS;
	}

	if (count > MAX_I2C_LEN)
		count = MAX_I2C_LEN;

	read_cnt = kfifo_out_locked(&cdev->DataKFiFo, fifo_read_buf, count,
				    &cdev->FiFoLock);

	EGALAX_DBG(DBG_CDEV, " \"%s\" reading fifo data count=%d\n",
		   current->comm, read_cnt);

	ret = copy_to_user(buf, fifo_read_buf, read_cnt) ? -EFAULT : read_cnt;

	up(&cdev->sem);

	return ret;
}

static ssize_t egalax_cdev_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *offset)
{
	struct egalax_char_dev *cdev = file->private_data;
	int ret = 0;
	char *tmp;

	if (down_interruptible(&cdev->sem))
		return -ERESTARTSYS;

	if (count > MAX_I2C_LEN)
		count = MAX_I2C_LEN;

	tmp = kzalloc(MAX_I2C_LEN, GFP_KERNEL);
	if (tmp == NULL) {
		up(&cdev->sem);
		return -ENOMEM;
	}

	if (copy_from_user(tmp, buf, count)) {
		up(&cdev->sem);
		kfree(tmp);
		return -EFAULT;
	}

	ret = egalax_I2C_write(0x0067, tmp, MAX_I2C_LEN);

	up(&cdev->sem);
	EGALAX_DBG(DBG_CDEV, " I2C writing %zu bytes.\n", count);
	kfree(tmp);

	return (ret == 0 ? count : -1);
}

static unsigned int egalax_cdev_poll(struct file *filp,
				     struct poll_table_struct *wait)
{
	struct egalax_char_dev *cdev = filp->private_data;
	unsigned int mask = 0;
	int fifoLen;

	down(&cdev->sem);
	poll_wait(filp, &cdev->fifo_inq,  wait);

	fifoLen = kfifo_len(&cdev->DataKFiFo);

	if (fifoLen > 0)
		mask |= POLLIN | POLLRDNORM;

	if ((FIFO_SIZE - fifoLen) > MAX_I2C_LEN)
		mask |= POLLOUT | POLLWRNORM;

	up(&cdev->sem);
	return mask;
}

static int egalax_proc_show(struct seq_file *seqfilp, void *v)
{
	seq_printf(seqfilp,
	"EETI I2C for All Points.\nDebug Level: 0x%08X\nRelease Date: %s\n",
		DbgLevel, RELEASE_DATE);

	return 0;
}

static int egalax_proc_open(struct inode *inode, struct file *filp)
{
	EGALAX_DBG(DBG_PROC, " \"%s\" call proc_open\n", current->comm);
	return single_open(filp, egalax_proc_show, NULL);
}

static ssize_t egalax_proc_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *offset)
{
	char procfs_buffer_size = 0;
	unsigned char procfs_buf[PROC_FS_MAX_LEN+1] = {0};
	unsigned int newLevel = 0;

	EGALAX_DBG(DBG_PROC, " \"%s\" call proc_write\n", current->comm);

	procfs_buffer_size = count;
	if (procfs_buffer_size > PROC_FS_MAX_LEN)
		procfs_buffer_size = PROC_FS_MAX_LEN+1;

	if (copy_from_user(procfs_buf, buf, procfs_buffer_size)) {
		EGALAX_DBG(DBG_PROC, " proc_write faied at copy_from_user\n");
		return -EFAULT;
	}

	if (!kstrtouint(procfs_buf, 16, &newLevel))
		DbgLevel = newLevel;

	EGALAX_DBG(DBG_PROC, " Switch Debug Level to 0x%08X\n", DbgLevel);

	return procfs_buffer_size;
}

static bool sys_sendcmd_wait(unsigned char *bySendCmd, int nSendCmdLen,
			     unsigned char *byHookCmd, int nHookCmdLen,
			     int nTimeOut)
{
	int i;
	bool bRet = true;

	memset(p_egalax_i2c_dev->sysfs_cmd_result, 0,
	       sizeof(p_egalax_i2c_dev->sysfs_cmd_result));

	for (i = 0; i < 3; i++) {
		if (i < nHookCmdLen)
			p_egalax_i2c_dev->sysfs_hook_cmd[i] = byHookCmd[i];
		else
			p_egalax_i2c_dev->sysfs_hook_cmd[i] = 0xFF;
	}
	p_egalax_i2c_dev->sysfs_query_wait = true;

	if (egalax_I2C_write(0x0067, bySendCmd, nSendCmdLen) != 0) {
		bRet = false;
	} else {
		wait_event_interruptible_timeout(
				p_egalax_i2c_dev->sysfs_query_queue,
				!p_egalax_i2c_dev->sysfs_query_wait,
				nTimeOut);

		if (p_egalax_i2c_dev->sysfs_query_wait)
			bRet = false;
		else
			bRet = true;
	}
	p_egalax_i2c_dev->sysfs_query_wait = false;
	return bRet;
}

#define OP_MODE_GET		0x00
#define OP_MODE_SET		0x01
static ssize_t sys_show_version(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned char SendCmdBuf[MAX_I2C_LEN] = {
			0x03, 0x04, 0x36, 0x91, 0x01, OP_MODE_GET};
	bool bRet = true;

	bRet = sys_sendcmd_wait(SendCmdBuf, MAX_I2C_LEN, SendCmdBuf+2, 3, HZ);
	if (bRet)
		return snprintf(buf, PAGE_SIZE, "Driver: %s  FW: %s\n",
			RELEASE_DATE, p_egalax_i2c_dev->sysfs_cmd_result+6);
	else
		return snprintf(buf, PAGE_SIZE, "Driver: %s  FW: Invalid\n",
				RELEASE_DATE);

}

static ssize_t sys_show_touchevent(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	unsigned char SendCmdBuf[MAX_I2C_LEN] = {
			0x03, 0x04, 0x36, 0x91, 0x02, OP_MODE_GET};
	bool bRet = true;
	int code = 0;

	bRet = sys_sendcmd_wait(SendCmdBuf, MAX_I2C_LEN, SendCmdBuf+2, 3, HZ);
	if (bRet) {
		code = p_egalax_i2c_dev->sysfs_cmd_result[6];
		code += (p_egalax_i2c_dev->sysfs_cmd_result[7]<<8);
		code += (p_egalax_i2c_dev->sysfs_cmd_result[8]<<16);
		code += (p_egalax_i2c_dev->sysfs_cmd_result[9]<<24);
		return snprintf(buf, PAGE_SIZE, "0x%08X\n", code);
	} else
		return snprintf(buf, PAGE_SIZE, "Invalid\n");
}

static ssize_t sys_show_reportmode(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	unsigned char SendCmdBuf[MAX_I2C_LEN] = {
			0x03, 0x04, 0x36, 0x91, 0x04, OP_MODE_GET};
	bool bRet = true;

	bRet = sys_sendcmd_wait(SendCmdBuf, MAX_I2C_LEN, SendCmdBuf+2, 3, HZ);
	if (bRet)
		return snprintf(buf, PAGE_SIZE, "%02X\n",
			       p_egalax_i2c_dev->sysfs_cmd_result[6]);
	else
		return snprintf(buf, PAGE_SIZE, "Invalid\n");
}

#define NV_REPORTMODE_MAX  0x06
static ssize_t sys_store_reportmode(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned char SendCmdBuf[MAX_I2C_LEN] = {
			0x03, 0x05, 0x36, 0x91, 0x04, OP_MODE_SET};
	bool bRet = true;
	char mode;

	if (count != 2)
		return -EINVAL;

	mode = buf[0]-'0';
	if (mode > NV_REPORTMODE_MAX || mode < 0)
		return -EINVAL;

	SendCmdBuf[6] = mode;

	bRet = sys_sendcmd_wait(SendCmdBuf, MAX_I2C_LEN, SendCmdBuf+2, 3, HZ);
	if (bRet)
		return count;
	else
		return -EIO;
}

static ssize_t sys_show_bypassmode(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	unsigned char SendCmdBuf[MAX_I2C_LEN] = {
			0x03, 0x04, 0x36, 0x91, 0x05, OP_MODE_GET};
	bool bRet = true;

	bRet = sys_sendcmd_wait(SendCmdBuf, MAX_I2C_LEN, SendCmdBuf+2, 3, HZ);
	if (bRet)
		return snprintf(buf, PAGE_SIZE, "%02X\n",
			       p_egalax_i2c_dev->sysfs_cmd_result[6]);
	else
		return snprintf(buf, PAGE_SIZE, "Invalid\n");
}

#define NV_BYPASSMODE_MAX  0x02
static ssize_t sys_store_bypassmode(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned char SendCmdBuf[MAX_I2C_LEN] = {
			0x03, 0x05, 0x36, 0x91, 0x05, OP_MODE_SET};
	bool bRet = true;
	char mode;

	if (count != 2)
		return -EINVAL;

	mode = buf[0]-'0';
	if (mode > NV_BYPASSMODE_MAX || mode < 0)
		return -EINVAL;

	SendCmdBuf[6] = mode;

	bRet = sys_sendcmd_wait(SendCmdBuf, MAX_I2C_LEN, SendCmdBuf+2, 3, HZ);
	if (bRet)
		return count;
	else
		return -EIO;
}

static DEVICE_ATTR(version, 0640, sys_show_version, NULL);
static DEVICE_ATTR(touch_event, 0640, sys_show_touchevent, NULL);
static DEVICE_ATTR(report_mode, 0640, sys_show_reportmode,
		   sys_store_reportmode);
static DEVICE_ATTR(bypass_mode, 0640, sys_show_bypassmode,
		   sys_store_bypassmode);

static struct attribute *egalax_attributes[] = {
	&dev_attr_version.attr,
	&dev_attr_touch_event.attr,
	&dev_attr_report_mode.attr,
	&dev_attr_bypass_mode.attr,
	NULL,
};

static const struct attribute_group egalax_attr_group = {
	.attrs = egalax_attributes,
};

#define STYLUS_MASK	0x10
#define MAX_POINT_PER_PACKET	5U
#define POINT_STRUCT_SIZE	10U
static void ProcessReport(unsigned char *buf,
			  struct _egalax_i2c *p_egalax_i2c)
{
	unsigned char i, index = 0, cnt_down = 0, cnt_up = 0, shift = 0;
	unsigned char status = 0;
	unsigned short contactID = 0, x = 0, y = 0, z = 0;

	if (TotalPtsCnt <= 0) {
		if ((buf[1] == 0) || (buf[1] > MAX_SUPPORT_POINT)) {
			EGALAX_DBG(DBG_POINT,
				   " NumsofContacts mismatch, skip packet\n");
			return;
		}

		TotalPtsCnt = buf[1];
		RecvPtsCnt = 0;
	} else if (buf[1] > 0) {
		TotalPtsCnt = 0;
		RecvPtsCnt = 0;
		EGALAX_DBG(DBG_POINT,
			   " NumsofContacts mismatch, skip packet\n");
		return;
	}

	while (index < MAX_POINT_PER_PACKET) {
		shift = index * POINT_STRUCT_SIZE + 2;
		status = buf[shift];
		contactID = buf[shift+1];
		x = ((buf[shift+3]<<8) + buf[shift+2]);
		y = ((buf[shift+5]<<8) + buf[shift+4]);
		z = ((buf[shift+7]<<8) + buf[shift+6]);

		if (contactID >= MAX_SUPPORT_POINT) {
			TotalPtsCnt = 0;
			RecvPtsCnt = 0;
			EGALAX_DBG(DBG_POINT, " Get error ContactID.\n");
			return;
		}

		EGALAX_DBG(DBG_POINT,
			   " Get Point[%d] Update: Status=%d X=%d Y=%d\n",
			   contactID, status, x, y);

	#ifdef _SWITCH_XY
		short tmp = x;

		x = y;
		y = tmp;
	#endif

	#ifdef _CONVERT_X
		x = MAX_RESOLUTION-x;
	#endif

	#ifdef _CONVERT_Y
		y = MAX_RESOLUTION-y;
	#endif

		pContactBuf[RecvPtsCnt].ID = contactID;
		pContactBuf[RecvPtsCnt].Status = status;
		pContactBuf[RecvPtsCnt].X = x;
		pContactBuf[RecvPtsCnt].Y = y;
		pContactBuf[RecvPtsCnt].Z = z;

		RecvPtsCnt++;
		index++;

		/* Recv all points, send input report */
		if (RecvPtsCnt == TotalPtsCnt) {
			for (i = 0; i < RecvPtsCnt; i++) {
				input_mt_slot(input_dev, pContactBuf[i].ID);
				if ((pContactBuf[i].Status &
					STYLUS_MASK) != 0) {
					input_mt_report_slot_state(input_dev,
						MT_TOOL_PEN,
						((pContactBuf[i].Status&0x01)
						!= 0));
				} else {
					input_mt_report_slot_state(input_dev,
						MT_TOOL_FINGER,
						((pContactBuf[i].Status&0x01)
						!= 0));
				}

				if ((pContactBuf[i].Status & 0x01) != 0) {
					input_report_abs(input_dev,
							ABS_MT_POSITION_X,
							pContactBuf[i].X);
					input_report_abs(input_dev,
							ABS_MT_POSITION_Y,
							pContactBuf[i].Y);
					input_report_abs(input_dev,
							ABS_MT_PRESSURE,
							pContactBuf[i].Z);
				}

				if ((pContactBuf[i].Status & 0x01) != 0)
					cnt_down++;
				else
					cnt_up++;
			}

			input_sync(input_dev);
			EGALAX_DBG(DBG_POINT,
			" Input sync point data done! (Down:%d Up:%d)\n",
				cnt_down, cnt_up);

			TotalPtsCnt = 0;
			RecvPtsCnt = 0;
			return;
		}
	}
}

static struct input_dev *allocate_Input_Dev(void)
{
	int ret;
	struct input_dev *pInputDev = NULL;

	pInputDev = input_allocate_device();
	if (pInputDev == NULL) {
		EGALAX_DBG(DBG_MODULE, " Failed to allocate input device\n");
		return NULL;
	}

	pInputDev->name = "eGalax_Touch_Screen";
	pInputDev->phys = "I2C";
	pInputDev->id.bustype = BUS_I2C;
	pInputDev->id.vendor = 0x0EEF;
	pInputDev->id.product = 0x0020;
	pInputDev->id.version = 0x0001;

	set_bit(EV_ABS, pInputDev->evbit);
	__set_bit(INPUT_PROP_DIRECT, pInputDev->propbit);
	input_mt_init_slots(pInputDev, MAX_SUPPORT_POINT, 0);
	input_set_abs_params(pInputDev, ABS_MT_POSITION_X, 0,
			     MAX_RESOLUTION, 0, 0);
	input_set_abs_params(pInputDev, ABS_MT_POSITION_Y, 0,
			     MAX_RESOLUTION, 0, 0);
	input_set_abs_params(pInputDev, ABS_MT_PRESSURE, 0,
			     MAX_Z_RESOLUTION, 0, 0);
	input_set_abs_params(pInputDev, ABS_MT_TOOL_TYPE, 0,
			     MT_TOOL_MAX, 0, 0);

	input_set_events_per_packet(pInputDev, MAX_EVENTS);

	ret = input_register_device(pInputDev);
	if (ret) {
		EGALAX_DBG(DBG_MODULE, " Unable to register input device.\n");
		input_free_device(pInputDev);
		pInputDev = NULL;
	}

	return pInputDev;
}

static int egalax_i2c_measure(struct _egalax_i2c *egalax_i2c)
{
	int ret = 0, frameLen = 0, loop = 3, i;

	EGALAX_DBG(DBG_INT, " egalax_i2c_measure\n");

	if (egalax_I2C_read(input_report_buf, MAX_I2C_LEN+2) < 0) {
		EGALAX_DBG(DBG_I2C, " I2C read input report fail!\n");
		return -1;
	}

	if ((DbgLevel & DBG_I2C) != 0U) {
		char dbgmsg[(MAX_I2C_LEN+2)*4];

		for (i = 0; i < (MAX_I2C_LEN+2); i++)
			snprintf(dbgmsg+(i*4), 4, "[%02X]",
				input_report_buf[i]);

		EGALAX_DBG(DBG_I2C, " Buf=%s\n", dbgmsg);
	}

	frameLen = input_report_buf[0] + (input_report_buf[1]<<8);
	EGALAX_DBG(DBG_I2C, " I2C read data with Len=%d\n", frameLen);

	if (frameLen == 0) {
		EGALAX_DBG(DBG_MODULE, " Device reset\n");
		return -1;
	}

	switch (input_report_buf[2]) {
	case REPORTID_MTOUCH:
		if (!egalax_i2c->skip_packet &&
		    egalax_i2c->work_state == MODE_WORKING) {
			ProcessReport(input_report_buf+2, egalax_i2c);
		}
		ret = 0;
		break;
	case REPORTID_VENDOR:
		atomic_set(&wait_command_ack, 1);
		EGALAX_DBG(DBG_I2C, " I2C get vendor command packet\n");

		if (egalax_i2c->sysfs_query_wait &&
			egalax_i2c->sysfs_hook_cmd[0] == input_report_buf[2+2]
			 && ((egalax_i2c->sysfs_hook_cmd[1] == 0xFF) ||
			egalax_i2c->sysfs_hook_cmd[1] == input_report_buf[2+3])
			&& ((egalax_i2c->sysfs_hook_cmd[2] == 0xFF) ||
			egalax_i2c->sysfs_hook_cmd[2] == input_report_buf[2+4])
			) {
			memcpy(egalax_i2c->sysfs_cmd_result,
			       input_report_buf+2, input_report_buf[2+1]+2);
			egalax_i2c->sysfs_query_wait = false;
			wake_up_interruptible(&egalax_i2c->sysfs_query_queue);
			break;
		}

		/* If someone reading now! put the data into the buffer! */
		if (p_char_dev->OpenCnts > 0) {
			loop = 3;
			do {
				ret = wait_event_timeout(p_char_dev->fifo_inq,
					(kfifo_avail(&p_char_dev->DataKFiFo) >=
					MAX_I2C_LEN), HZ);
			} while (ret <= 0 && --loop);

			/* fifo size is ready */
			if (ret > 0) {
				ret = kfifo_in_locked(&p_char_dev->DataKFiFo,
					(input_report_buf+2), MAX_I2C_LEN,
					&p_char_dev->FiFoLock);
				wake_up_interruptible(&p_char_dev->fifo_inq);
			} else {
				EGALAX_DBG(DBG_CDEV,
					" [Warning] fifo size is overflow.\n");
			}
		}

		break;
	default:
		EGALAX_DBG(DBG_I2C, " I2C read error data with hedaer=%d\n",
			   input_report_buf[2]);
		ret = -1;
		break;
	}

	return ret;
}

static void egalax_i2c_wq_irq(struct work_struct *work)
{
	struct _egalax_i2c *egalax_i2c =
			container_of(work, struct _egalax_i2c, work_irq);
	struct i2c_client *client = egalax_i2c->client;

	EGALAX_DBG(DBG_INT, " egalax_i2c_wq run\n");

	/* continue recv data */
	while (gpio_get_value(egalax_i2c->interrupt_gpio) == 0) {
		egalax_i2c_measure(egalax_i2c);
		schedule();
	}

	if (egalax_i2c->skip_packet > 0U)
		egalax_i2c->skip_packet = 0U;

	enable_irq(client->irq);

	EGALAX_DBG(DBG_INT, " egalax_i2c_wq leave\n");
}

static irqreturn_t egalax_i2c_interrupt(int irq, void *dev_id)
{
	struct _egalax_i2c *egalax_i2c = (struct _egalax_i2c *)dev_id;

	EGALAX_DBG(DBG_INT, " INT with irq:%d\n", irq);

	disable_irq_nosync(irq);

	queue_work(egalax_i2c->ktouch_wq, &egalax_i2c->work_irq);

	return IRQ_HANDLED;
}

static void egalax_i2c_senduppoint(void)
{
	int i = 0;

	EGALAX_DBG(DBG_SUSP, " %s\n", __func__);

	for (i = 0; i < MAX_SUPPORT_POINT; i++) {
		input_mt_slot(input_dev, pContactBuf[i].ID);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
		pContactBuf[i].Status = 0;
	}

	input_sync(input_dev);
	EGALAX_DBG(DBG_POINT, " Sent up point data done!\n");
}

static int egalax_power_off(void)
{
	int error;

	if(p_egalax_i2c_dev->enable_high)
		gpio_direction_output(p_egalax_i2c_dev->reset_gpio, 1);
	else
		gpio_direction_output(p_egalax_i2c_dev->reset_gpio, 0);
	error = regulator_enable(p_egalax_i2c_dev->regulator_hv);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_enable(p_egalax_i2c_dev->regulator_5v0);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_enable(p_egalax_i2c_dev->regulator_3v3);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_enable(p_egalax_i2c_dev->regulator_1v8);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);

	return 0;
}

static int egalax_power_on(void)
{
	int error;

	error = regulator_enable(p_egalax_i2c_dev->regulator_hv);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_enable(p_egalax_i2c_dev->regulator_5v0);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_enable(p_egalax_i2c_dev->regulator_1v8);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_enable(p_egalax_i2c_dev->regulator_3v3);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	usleep_range(1000, 5000);
	if(p_egalax_i2c_dev->enable_high)
		gpio_direction_output(p_egalax_i2c_dev->reset_gpio, 0);
	else
		gpio_direction_output(p_egalax_i2c_dev->reset_gpio, 1);

	return 0;
}

static int egalax_i2c_pm_suspend(struct i2c_client *client, pm_message_t mesg)
{
	EGALAX_DBG(DBG_SUSP, " Enter pm_suspend state:%d\n",
		   p_egalax_i2c_dev->work_state);

	if (!p_egalax_i2c_dev)
		goto fail_suspend;

	egalax_power_off();

	p_egalax_i2c_dev->work_state = MODE_SUSPEND;

	EGALAX_DBG(DBG_SUSP, " pm_suspend done!!\n");
	return 0;

fail_suspend:
	EGALAX_DBG(DBG_SUSP, " pm_suspend failed!!\n");
	return -1;
}

static int egalax_i2c_pm_resume(struct i2c_client *client)
{
	EGALAX_DBG(DBG_SUSP, " Enter pm_resume state:%d\n",
			p_egalax_i2c_dev->work_state);

	if (!p_egalax_i2c_dev)
		goto fail_resume;

	p_egalax_i2c_dev->work_state = MODE_WORKING;

	egalax_power_on();

	egalax_i2c_senduppoint();

	EGALAX_DBG(DBG_SUSP, " pm_resume done!!\n");
	return 0;

fail_resume:
	EGALAX_DBG(DBG_SUSP, " pm_resume failed!!\n");
	return -1;
}

static int egalax_i2c_ops_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	pm_message_t state;

	state.event = PM_EVENT_SUSPEND;
	EGALAX_DBG(DBG_SUSP, " %s\n", __func__);
	return egalax_i2c_pm_suspend(client, state);
}

static int egalax_i2c_ops_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	EGALAX_DBG(DBG_SUSP, " %s\n", __func__);
	return egalax_i2c_pm_resume(client);
}

static int request_dt(struct i2c_client *client)
{
	int result = 0;
	struct device_node *devnode;

	devnode = client->dev.of_node;
	if (devnode) {
		p_egalax_i2c_dev->interrupt_gpio = of_get_named_gpio(devnode,
							"irq-gpio", 0);
		p_egalax_i2c_dev->reset_gpio = of_get_named_gpio(devnode,
							"reset-gpio", 0);
		if (of_property_read_bool(devnode, "enable-active-high"))
			p_egalax_i2c_dev->enable_high = true;
		/* regulator */
		p_egalax_i2c_dev->regulator_hv = devm_regulator_get(
						&client->dev, "vdd-ts-hv");
		if (IS_ERR(p_egalax_i2c_dev->regulator_hv)) {
			EGALAX_DBG(DBG_MODULE,
				"vdd-12v regulator_get failed: %ld\n",
				PTR_ERR(p_egalax_i2c_dev->regulator_hv));
			return -EINVAL;
		}
		p_egalax_i2c_dev->regulator_5v0 = devm_regulator_get(
						&client->dev, "vdd-ts-5v0");
		if (IS_ERR(p_egalax_i2c_dev->regulator_5v0)) {
			EGALAX_DBG(DBG_MODULE,
				"vdd-5v regulator_get failed: %ld\n",
				PTR_ERR(p_egalax_i2c_dev->regulator_5v0));
			return -EINVAL;
		}

		p_egalax_i2c_dev->regulator_3v3 = devm_regulator_get(
						&client->dev, "vdd-ts-3v3");
		if (IS_ERR(p_egalax_i2c_dev->regulator_3v3)) {
			EGALAX_DBG(DBG_MODULE,
				"vdd 3v3 regulator_get failed: %ld\n",
				PTR_ERR(p_egalax_i2c_dev->regulator_3v3));
			return -EINVAL;
		}
		p_egalax_i2c_dev->regulator_1v8 = devm_regulator_get(
						&client->dev, "vdd-ts-1v8");
		if (IS_ERR(p_egalax_i2c_dev->regulator_1v8)) {
			EGALAX_DBG(DBG_MODULE,
				"vdd 18v regulator_get failed: %ld\n",
				PTR_ERR(p_egalax_i2c_dev->regulator_1v8));
			return -EINVAL;
		}
	}

	if (!gpio_is_valid(p_egalax_i2c_dev->interrupt_gpio)) {
		EGALAX_DBG(DBG_MODULE, " gpio[%d] is not valid\n",
			   p_egalax_i2c_dev->interrupt_gpio);
		return -EINVAL;
	}
	result = gpio_request(p_egalax_i2c_dev->interrupt_gpio, "irq-gpio");
	if (result < 0) {
		EGALAX_DBG(DBG_MODULE, " gpio_request[%d] failed: %d\n",
			   p_egalax_i2c_dev->interrupt_gpio, result);
		return -EINVAL;
	}
	gpio_direction_input(p_egalax_i2c_dev->interrupt_gpio);
	client->irq = gpio_to_irq(p_egalax_i2c_dev->interrupt_gpio);

	if (!gpio_is_valid(p_egalax_i2c_dev->reset_gpio)) {
		EGALAX_DBG(DBG_MODULE, " gpio[%d] is not valid\n",
			   p_egalax_i2c_dev->reset_gpio);
		return -EINVAL;
	}
	result = gpio_request(p_egalax_i2c_dev->reset_gpio, "rest-gpio");
	if (result < 0) {
		EGALAX_DBG(DBG_MODULE, " gpio_request[%d] failed: %d\n",
			   p_egalax_i2c_dev->reset_gpio, result);
		return -EINVAL;
	}

	return result;
}

static SIMPLE_DEV_PM_OPS((egalax_i2c_pm_ops), (egalax_i2c_ops_suspend),
			 (egalax_i2c_ops_resume));

static const struct file_operations egalax_cdev_fops = {
	.owner		= THIS_MODULE,
	.read		= egalax_cdev_read,
	.write		= egalax_cdev_write,
	.open		= egalax_cdev_open,
	.release	= egalax_cdev_release,
	.poll		= egalax_cdev_poll,
};

static struct miscdevice egalax_misc_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "touch",
	.fops	= &egalax_cdev_fops,
};

static int egalax_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *idp)
{
	int result;

	EGALAX_DBG(DBG_MODULE, " Start probe\n");

	p_egalax_i2c_dev = kzalloc(sizeof(struct _egalax_i2c), GFP_KERNEL);
	if (!p_egalax_i2c_dev) {
		EGALAX_DBG(DBG_MODULE, " Request memory failed\n");
		result = -ENOMEM;
		goto fail1;
	}
	p_egalax_i2c_dev->client = client;

	result = request_dt(client);
	if (result < 0) {
		EGALAX_DBG(DBG_MODULE, " Request DT failed\n");
		result = -ENODEV;
		goto fail1;
	}

	egalax_power_on();

	input_dev = allocate_Input_Dev();
	if (input_dev == NULL) {
		EGALAX_DBG(DBG_MODULE, " allocate_Input_Dev failed\n");
		result = -EINVAL;
		goto fail2;
	}
	EGALAX_DBG(DBG_MODULE, " Register input device done\n");

	mutex_init(&p_egalax_i2c_dev->mutex_wq);

	p_egalax_i2c_dev->ktouch_wq =
			create_singlethread_workqueue("egalax_touch_wq");
	INIT_WORK(&p_egalax_i2c_dev->work_irq, egalax_i2c_wq_irq);

	i2c_set_clientdata(client, p_egalax_i2c_dev);

	if (gpio_get_value(p_egalax_i2c_dev->interrupt_gpio))
		p_egalax_i2c_dev->skip_packet = 0;
	else
		p_egalax_i2c_dev->skip_packet = 1;

	p_egalax_i2c_dev->work_state = MODE_WORKING;

	result = request_irq(client->irq, egalax_i2c_interrupt,
			     IRQF_TRIGGER_LOW, client->name, p_egalax_i2c_dev);
	if (result) {
		EGALAX_DBG(DBG_MODULE, " Request irq(%d) failed\n",
			   client->irq);
		goto fail3;
	}
	EGALAX_DBG(DBG_MODULE, " Request irq(%d) gpio(%d) with result:%d\n",
		   client->irq, p_egalax_i2c_dev->interrupt_gpio, result);

	result = misc_register(&egalax_misc_dev);
	if (result) {
		EGALAX_DBG(DBG_MODULE, " misc device register failed\n");
		goto fail4;
	}
	result = sysfs_create_group(&egalax_misc_dev.this_device->kobj,
				    &egalax_attr_group);
	if (result) {
		EGALAX_DBG(DBG_MODULE,
			   " Failed to create sysfs attributes:%d\n", result);
		goto fail4;
	}

	init_waitqueue_head(&p_egalax_i2c_dev->sysfs_query_queue);
	p_egalax_i2c_dev->sysfs_query_wait = false;
	p_egalax_i2c_dev->sysfs_hook_cmd[0] = 0xFF;
	p_egalax_i2c_dev->sysfs_hook_cmd[1] = 0xFF;
	p_egalax_i2c_dev->sysfs_hook_cmd[2] = 0xFF;

	EGALAX_DBG(DBG_MODULE, " I2C probe done\n");
	return 0;

fail4:
	free_irq(client->irq, p_egalax_i2c_dev);
fail3:
	i2c_set_clientdata(client, NULL);
	destroy_workqueue(p_egalax_i2c_dev->ktouch_wq);
	input_unregister_device(input_dev);
	input_dev = NULL;
fail2:
	gpio_free(p_egalax_i2c_dev->interrupt_gpio);
fail1:
	kfree(p_egalax_i2c_dev);
	p_egalax_i2c_dev = NULL;

	EGALAX_DBG(DBG_MODULE, " I2C probe failed\n");
	return result;
}

static int egalax_i2c_remove(struct i2c_client *client)
{
	struct _egalax_i2c *egalax_i2c = i2c_get_clientdata(client);

	egalax_i2c->work_state = MODE_STOP;

	cancel_work_sync(&egalax_i2c->work_irq);

	if (client->irq) {
		disable_irq(client->irq);
		free_irq(client->irq, egalax_i2c);
	}

	gpio_free(egalax_i2c->interrupt_gpio);
	gpio_free(egalax_i2c->reset_gpio);

	if (egalax_i2c->ktouch_wq)
		destroy_workqueue(egalax_i2c->ktouch_wq);

	if (input_dev) {
		EGALAX_DBG(DBG_MODULE,  " Unregister input device\n");
		input_unregister_device(input_dev);
		input_dev = NULL;
	}

	i2c_set_clientdata(client, NULL);
	kfree(egalax_i2c);
	p_egalax_i2c_dev = NULL;

	return 0;
}

static const struct i2c_device_id egalax_i2c_idtable[] = {
	{ "egalax_i2c", 0 },
	{ }
};

static const struct of_device_id egalax_i2c_dt_ids[] = {
	{ .compatible = "eeti,exc80_ts" },
	{ }
};

MODULE_DEVICE_TABLE(i2c, egalax_i2c_idtable);

static struct i2c_driver egalax_i2c_driver = {
	.driver = {
		.name	= "egalax_i2c",
		.owner	= THIS_MODULE,
		.of_match_table = egalax_i2c_dt_ids,
		.pm		= &egalax_i2c_pm_ops,
	},
	.id_table	= egalax_i2c_idtable,
	.probe		= egalax_i2c_probe,
	.remove		= egalax_i2c_remove,
};

static const struct file_operations egalax_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= egalax_proc_open,
	.read		= seq_read,
	.write		= egalax_proc_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void egalax_i2c_ts_exit(void)
{
	if (p_char_dev) {
		kfree(p_char_dev->pFiFoBuf);
		kfree(p_char_dev);
		p_char_dev = NULL;
	}

	sysfs_remove_group(&egalax_misc_dev.this_device->kobj, &egalax_attr_group);

	misc_deregister(&egalax_misc_dev);

	i2c_del_driver(&egalax_i2c_driver);

	remove_proc_entry(PROC_FS_NAME, NULL);

	EGALAX_DBG(DBG_MODULE, " Exit driver done!\n");
}

static struct egalax_char_dev *setup_chardev(void)
{
	struct egalax_char_dev *pCharDev;

	pCharDev = kzalloc(1*sizeof(struct egalax_char_dev), GFP_KERNEL);
	if (!pCharDev)
		goto fail_cdev;

	spin_lock_init(&pCharDev->FiFoLock);
	pCharDev->pFiFoBuf = kzalloc(sizeof(unsigned char)*FIFO_SIZE,
				     GFP_KERNEL);
	if (!pCharDev->pFiFoBuf)
		goto fail_fifobuf;

	kfifo_init(&pCharDev->DataKFiFo, pCharDev->pFiFoBuf, FIFO_SIZE);
	if (!kfifo_initialized(&pCharDev->DataKFiFo))
		goto fail_kfifo;

	pCharDev->OpenCnts = 0;
	sema_init(&pCharDev->sem, 1);
	init_waitqueue_head(&pCharDev->fifo_inq);

	return pCharDev;

fail_kfifo:
	kfree(pCharDev->pFiFoBuf);
fail_fifobuf:
	kfree(pCharDev);
fail_cdev:
	return NULL;
}

static int egalax_i2c_ts_init(void)
{
	int result;

	p_egalax_i2c_dev = NULL;
	p_char_dev = NULL;
	input_dev = NULL;
	TotalPtsCnt = 0;
	RecvPtsCnt = 0;

	p_char_dev = setup_chardev();
	if (!p_char_dev) {
		result = -ENOMEM;
		goto fail;
	}

	dbgProcFile = proc_create(PROC_FS_NAME, 0660, NULL,
				  &egalax_proc_fops);
	if (dbgProcFile == NULL) {
		remove_proc_entry(PROC_FS_NAME, NULL);
		EGALAX_DBG(DBG_MODULE, " Could not initialize /proc/%s\n",
			   PROC_FS_NAME);
	}

	EGALAX_DBG(DBG_MODULE, " Driver init done!\n");
	return i2c_add_driver(&egalax_i2c_driver);

fail:
	egalax_i2c_ts_exit();
	return result;
}

module_init(egalax_i2c_ts_init);
module_exit(egalax_i2c_ts_exit);

MODULE_AUTHOR("EETI <touch_fae@eeti.com>");
MODULE_DESCRIPTION("egalax all points controller i2c driver");
MODULE_LICENSE("GPL");
