/*
 * Raydium RM31080 touchscreen driver
 *
 * Copyright (C) 2012 Raydium Semiconductor Corporation
 * Copyright (C) 2012 NVIDIA Corporation, All Rights Reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
/*=========================================================================*/
/*INCLUDED FILES */
/*=========================================================================*/
#include <linux/module.h>
#include <linux/input.h>	/* BUS_SPI */
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>	/* wake_up_process() */
#include <linux/kthread.h>	/* kthread_create(),kthread_run() */
#include <linux/uaccess.h>	/* copy_to_user(), */
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>

#include <linux/spi/rm31080a_ts.h>
#include <linux/spi/rm31080a_ctrl.h>
#include <../clock.h>
/*=========================================================================*/
/*DEFINITIONS */
/*=========================================================================*/
#define ENABLE_WORK_QUEUE
#define ENABLE_REPORT_TO_UART
#define ENABLE_RM31080_DEEP_SLEEP
#define ENABLE_AUTO_SCAN
/*#define ENABLE_SPEED_TEST_FUNCTION */
/*#define ENABLE_CALC_QUEUE_COUNT */
/*#define ENABLE_SPI_BURST_READ_WRITE */
/*#define ENABLE_SPI_SETTING */
#define ENABLE_SMOOTH_LEVEL
/*#define ENABLE_SUPPORT_4_7*/  /* for 4.7 inch display  */
#define ENABLE_NEW_INPUT_DEV

#define MAX_SPI_FREQ_HZ      50000000
#define TS_PEN_UP_TIMEOUT    msecs_to_jiffies(50)

#ifdef ENABLE_RAW_DATA_QUEUE
#define QUEUE_COUNT       128
#if ENABLE_T007_ST_SCAN
#define RAW_DATA_LENGTH  (RM_MAX_MT_COUNT + RM_MAX_CHANNEL_COUNT)
#else
#define RAW_DATA_LENGTH  2048
#endif

#define RM_SCAN_MODE_MANUAL          0x00
#define RM_SCAN_MODE_PREPARE_AUTO    0x01
#define RM_SCAN_MODE_AUTO_SCAN       0x02

#define RM_NEED_NONE                 0x00
#define RM_NEED_TO_SEND_SCAN         0x01
#define RM_NEED_TO_READ_RAW_DATA     0x02
#define RM_NEED_TO_SEND_SIGNAL       0x04
#endif


#ifdef ENABLE_SMOOTH_LEVEL
#define RM_SMOOTH_LEVEL_NORMAL    0
#define RM_SMOOTH_LEVEL_MAX       4
#endif

#ifdef ENABLE_WORK_QUEUE
#include <linux/workqueue.h>
#endif

#define rm_printk(msg...)	do { dev_info(&g_spi->dev, msg); } while (0)
#define rmd_printk(msg...)	do {  } while (0)
/*=========================================================================*/
/*STRUCTURE DECLARATION */
/*=========================================================================*/
/*TouchScreen Parameters */
struct rm31080a_ts_para {
	unsigned long ulHalPID;
	bool bInitFinish;
	bool bCalcFinish;
	bool bEnableScriber;
	bool bEnableAutoScan;
	bool bIsSuspended;

	struct mutex mutex_scan_mode;
#ifdef ENABLE_WORK_QUEUE
	struct workqueue_struct *rm_workqueue;
	struct work_struct rm_work;
#endif
#ifdef ENABLE_RAW_DATA_QUEUE
	u8 u8ScanModeState;
#endif

#ifdef ENABLE_SLOW_SCAN
	bool bEnableSlowScan;
	u32 u32SlowScanLevel;
#endif

#ifdef ENABLE_SMOOTH_LEVEL
	u32 u32SmoothLevel;
#endif

	u8 u8SelfTestStatus;
	u8 u8SelfTestResult;
	u8 u8Version;
	u8 u8Repeat;
};

struct rm31080_ts {
	const struct rm31080_bus_ops *bops;
	struct device *dev;
	struct input_dev *input;
	unsigned int irq;
	bool disabled;
	bool suspended;
	char phys[32];
	struct mutex access_mutex;
	struct regulator *regulator_3v3;
	struct regulator *regulator_1v8;
	struct notifier_block nb_3v3;
	struct notifier_block nb_1v8;
};

struct rm31080_bus_ops {
	u16 bustype;
	int (*read) (struct device *dev, u8 reg);
	int (*write) (struct device *dev, u8 reg, u16 val);
};

#ifdef ENABLE_RAW_DATA_QUEUE
struct rm31080_queue_info {
	u8(*pQueue)[RAW_DATA_LENGTH];
	u16 u16Front;
	u16 u16Rear;
};
#endif

/*=========================================================================*/
/*GLOBAL VARIABLES DECLARATION */
/*=========================================================================*/
struct input_dev *g_input_dev;
struct spi_device *g_spi;
struct rm31080a_ts_para g_stTs;

#ifdef ENABLE_RAW_DATA_QUEUE
struct rm31080_queue_info g_stQ;
#endif

#ifdef ENABLE_SLOW_SCAN
struct rm_cmd_slow_scan g_stCmdSlowScan[RM_SLOW_SCAN_LEVEL_COUNT];
#endif
/*========================================================================= */
/*FUNCTION DECLARATION */
/*========================================================================= */
static int rm31080_spi_checking(bool bInfinite);
/*=========================================================================
 * Description:
 *      Debug function: test speed.
 * Input:
 *      N/A
 * Output:
 *      1:succeed
 *      0:failed
 *========================================================================= */
#ifdef ENABLE_SPEED_TEST_FUNCTION
void my_calc_time(int iStart)
{
	static unsigned int u32Max = UINT_MAX;

	static long iTimebuffer[1000];
	static unsigned long long t1, t2;
	unsigned long nanosec_rem;
	static int iIndex;

	if (iStart) {
		t1 = cpu_clock(u32Max);
		return;
	} else
		t2 = cpu_clock(u32Max);

	t2 = t2 - t1;

	nanosec_rem = do_div(t2, 1000000000);

	if (t2) {		/*more than 1 Second */
		iTimebuffer[iIndex] = 999999;
	} else {
		iTimebuffer[iIndex] = nanosec_rem / 1000;/*micro second */
	}

	iIndex++;
	if (iIndex == 1000) {
		for (iIndex = 0; iIndex < 1000; iIndex++) {
			rm_printk("   %04d,%06d\n", iIndex,
				  (u32) iTimebuffer[iIndex]);
		}
		iIndex = 0;
	}

}
#endif				/*ENABLE_SPEED_TEST_FUNCTION */
/*=========================================================================
 * Description:
 *      RM31080 spi interface.
 * Input:
 *
 * Output:
 *      1:succeed
 *      0:failed
 *=========================================================================
 */
static int rm31080_spi_read(u8 u8addr, u8 *rxbuf, size_t len)
{
	static DEFINE_MUTEX(lock);

	int status;
	struct spi_message message;
	struct spi_transfer x[2];

	mutex_lock(&lock);

	spi_message_init(&message);
	memset(x, 0, sizeof x);

	u8addr |= 0x80;
	x[0].len = 1;
	x[0].tx_buf = &u8addr;
	spi_message_add_tail(&x[0], &message);

	x[1].len = len;
	x[1].rx_buf = rxbuf;
	spi_message_add_tail(&x[1], &message);

	/* It returns zero on succcess,else a negative error code. */
	status = spi_sync(g_spi, &message);

	mutex_unlock(&lock);

	if (status)
		return false;

	return true;
}

/*=========================================================================
 * Description:
 *      RM31080 spi interface.
 * Input:
 *
 * Output:
 *      1:succeed
 *      0:failed
 *=========================================================================
 */
static int rm31080_spi_write(u8 *txbuf, size_t len)
{
	static DEFINE_MUTEX(lock);
	int status;
	/*It returns zero on succcess,else a negative error code. */
	mutex_lock(&lock);

	status = spi_write(g_spi, txbuf, len);

	mutex_unlock(&lock);

	if (status)
		return false;

	return true;
}

/*=========================================================================
 * Description:
 *      RM31080 spi interface.
 * Input:
 *
 * Output:
 *      1:succeed
 *      0:failed
 *=========================================================================
 */
int rm31080_spi_byte_read(unsigned char u8Addr, unsigned char *pu8Value)
{
	return rm31080_spi_read(u8Addr, pu8Value, 1);
}

/*=========================================================================
 * Description:
 *      RM31080 spi interface.
 * Input:
 *
 * Output:
 *      1:succeed
 *      0:failed
 *=========================================================================
 */
int rm31080_spi_byte_write(unsigned char u8Addr, unsigned char u8Value)
{
	u8 buf[2];
	buf[0] = u8Addr;
	buf[1] = u8Value;
	return rm31080_spi_write(buf, 2);
}

/*=========================================================================
 * Description:
 *      RM31080 spi interface.
 * Input:
 *
 * Output:
 *      1:succeed
 *      0:failed
 *=========================================================================
 */
#ifdef ENABLE_SPI_BURST_READ_WRITE
static int rm31080_spi_burst_read(unsigned char u8Addr,
				unsigned char *pu8Value,
				u32 u32len)
{
	int ret;
	u8 *pMyBuf;

	pMyBuf = kmalloc(u32len, GFP_KERNEL);
	if (pMyBuf == NULL)
		return false;

	ret = rm31080_spi_read(u8Addr, pMyBuf, u32len);

	if (ret)
		memcpy(pu8Value, pMyBuf, u32len);

	kfree(pMyBuf);

	return ret;
}
#endif
/*=========================================================================
 * Description:
 *      RM31080 spi interface.
 * Input:
 *
 * Output:
 *      1:succeed
 *      0:failed
 *=========================================================================
 */
int rm31080_spi_burst_write(unsigned char *pBuf, unsigned int u32Len)
{
	u8 *pMyBuf;
	int ret;

	pMyBuf = kmalloc(u32Len, GFP_KERNEL);
	if (pMyBuf == NULL)
		return false;

	memcpy(pMyBuf, pBuf, u32Len);
	ret = rm31080_spi_write(pMyBuf, u32Len);
	kfree(pMyBuf);
	return ret;
}

/*=========================================================================*/
#ifdef ENABLE_AUTO_SCAN
void raydium_change_scan_mode(u8 u8TouchCount)
{
	static u32 u32NoTouchCount;
	u16 u16NTCountThd;

	u16NTCountThd = (u16) g_stCtrl.bTime2Idle * 10;

	if (u8TouchCount) {
		u32NoTouchCount = 0;
		return;
	}
	if (u32NoTouchCount < u16NTCountThd) {
		u32NoTouchCount++;
	} else if (g_stTs.u8ScanModeState == RM_SCAN_MODE_MANUAL) {
		if (g_stTs.bEnableAutoScan)
			g_stTs.u8ScanModeState = RM_SCAN_MODE_PREPARE_AUTO;
		u32NoTouchCount = 0;
	}
}
#endif				/*ENABLE_AUTO_SCAN */
/*=========================================================================
  report touch data for scriber

  =========================================================================*/
#ifdef ENABLE_REPORT_TO_UART
void raydium_report_to_uart_printf(unsigned char *ucData,
				unsigned char ucCount)
{
	unsigned char i;
	for (i = 0; i < ucCount; i++)
		rm_printk("%02X", ucData[i]);

	rm_printk("\n");
}

void raydium_report_to_uart(void *p)
{
	/*1=Tag,1=Touch count,4=(xH xL ,yH yL) ,12=max point,1=Check sum */
	unsigned char ucData[1 + 1 + (4 * 12) + 1];
	struct rm_touch_event *spTP;
	unsigned short usX, usY;
	int i, j;

	if (g_stTs.bEnableScriber == 0)
		return;

	spTP = (struct rm_touch_event *) p;

	ucData[0] = 0x8E;
	ucData[1] = spTP->ucTouchCount;
	j = 2;
	for (i = 0; i < spTP->ucTouchCount; i++) {
		usX = spTP->usX[i] + 1;
		usY = spTP->usY[i] + 1;
		ucData[j++] = ((usX >> 8) & 0xFF) | (spTP->ucID[i] << 4);
		ucData[j++] = ((usX) & 0xFF);
		ucData[j++] = ((usY >> 8) & 0xFF);
		ucData[j++] = ((usY) & 0xFF);
	}

	/*check sum */
	ucData[j] = 0;
	for (i = 0; i < j; i++)
		ucData[j] += ucData[i];

	ucData[j] = 0x100 - ucData[j];
	j++;

	/*print */
	raydium_report_to_uart_printf(ucData, j);
	if (spTP->ucTouchCount == 0) {
		raydium_report_to_uart_printf(ucData, j);
		raydium_report_to_uart_printf(ucData, j);
	}
}
#endif				/*ENABLE_REPORT_TO_UART */
#ifdef ENABLE_SUPPORT_4_7	/*for 4.7 inch display */
#define _RM_SCALE_LEFT		(40 + 0)
#define _RM_SCALE_RIGHT		(30 + 0)
#define _RM_SCALE_TOP		(16 + 0)
#define _RM_SCALE_BOTTOM	(30 + 0)
void raydium_scale_touch_point(struct rm_touch_event *spTP,
				int iMaxX, int iMaxY,
				int left, int right,
				int top, int bottom)
{
	int i;
	int iNewX, iNewY;
	iNewX = iMaxX - _RM_SCALE_LEFT - _RM_SCALE_RIGHT;
	iNewY = iMaxY - _RM_SCALE_TOP - _RM_SCALE_BOTTOM;

	for (i = 0; i < spTP->ucTouchCount; i++) {
		/* cut out */
		if (spTP->usX[i] < _RM_SCALE_LEFT)
			spTP->usX[i] = _RM_SCALE_LEFT;
		else if (spTP->usX[i] > (iMaxX - _RM_SCALE_RIGHT))
			spTP->usX[i] = (iMaxX - _RM_SCALE_RIGHT);
		if (spTP->usY[i] < _RM_SCALE_TOP)
			spTP->usY[i] = _RM_SCALE_TOP;
		else if (spTP->usY[i] > (iMaxY - _RM_SCALE_BOTTOM))
			spTP->usY[i] = (iMaxY - _RM_SCALE_BOTTOM);
		/* shift */
		spTP->usX[i] -= _RM_SCALE_LEFT;
		spTP->usY[i] -= _RM_SCALE_TOP;
		/* scale */
		if (iNewX)
			spTP->usX[i] = (int)spTP->usX[i] * iMaxX / iNewX;
		if (iNewY)
			spTP->usY[i] = (int)spTP->usY[i] * iMaxY / iNewY;
	}
}
#endif

void raydium_report_pointer(void *p)
{
	static unsigned char ucLastTouchCount;
	int i;
	int iCount;
	int iMaxX, iMaxY;
	struct rm_touch_event *spTP;
#ifdef ENABLE_SUPPORT_4_7
	struct rm_touch_event stTP;
	unsigned long missing;
#endif
	spTP = (struct rm_touch_event *) p;

#if ENABLE_RESOLUTION_SWITCH
	if ((g_stCtrl.u16ResolutionX != 0) &&
		(g_stCtrl.u16ResolutionY != 0)) {
		iMaxX = g_stCtrl.u16ResolutionX;
		iMaxY = g_stCtrl.u16ResolutionY;
	} else {
		iMaxX = RM_INPUT_RESOLUTION_X;
		iMaxY = RM_INPUT_RESOLUTION_Y;
	}
#else
	iMaxX = RM_INPUT_RESOLUTION_X;
	iMaxY = RM_INPUT_RESOLUTION_Y;
#endif

#ifdef ENABLE_SUPPORT_4_7
	missing = copy_from_user(&stTP, p, sizeof(stTP));
	if (missing == 0) {
		raydium_scale_touch_point(&stTP, iMaxX, iMaxY,
					_RM_SCALE_LEFT,
					_RM_SCALE_RIGHT,
					_RM_SCALE_TOP,
					_RM_SCALE_BOTTOM);
		spTP = &stTP;
	}
#endif

	iCount = max(ucLastTouchCount, spTP->ucTouchCount);
	if (iCount) {
		for (i = 0; i < iCount; i++) {
			if (i == 10)
				break;

			if (i < spTP->ucTouchCount) {
				input_report_abs(g_input_dev,
						 ABS_MT_TRACKING_ID,
						 spTP->ucID[i]);
				input_report_abs(g_input_dev,
						 ABS_MT_TOUCH_MAJOR,
						 spTP->usZ[i]);
				input_report_abs(g_input_dev,
						 ABS_MT_WIDTH_MAJOR,
						 spTP->usZ[i]);
				input_report_abs(g_input_dev,
						 ABS_MT_PRESSURE,
						 spTP->usZ[i]);

				if (spTP->usX[i] >= (iMaxX - 1))
					input_report_abs(g_input_dev,
							 ABS_MT_POSITION_X,
							 (iMaxX - 1));
				else
					input_report_abs(g_input_dev,
							 ABS_MT_POSITION_X,
							 spTP->usX[i]);

				if (spTP->usY[i] >= (iMaxY - 1))
					input_report_abs(g_input_dev,
							 ABS_MT_POSITION_Y,
							 (iMaxY - 1));
				else
					input_report_abs(g_input_dev,
							 ABS_MT_POSITION_Y,
							 spTP->usY[i]);
			}
			input_mt_sync(g_input_dev);
		}
		ucLastTouchCount = spTP->ucTouchCount;
		input_report_key(g_input_dev, BTN_TOUCH,
				 spTP->ucTouchCount > 0);
		input_sync(g_input_dev);
#ifdef ENABLE_REPORT_TO_UART
		raydium_report_to_uart(p);
#endif

	}
#ifdef ENABLE_AUTO_SCAN
	if (g_stCtrl.bfPowerMode)
		raydium_change_scan_mode(spTP->ucTouchCount);
#endif
}

/*=========================================================================
 * Description:
 *      RM31080 control functions.
 * Input:
 *      N/A
 * Output:
 *      1:succeed
 *      0:failed
 *=========================================================================
 */
#ifdef ENABLE_RAW_DATA_QUEUE
int rm31080_ctrl_read_raw_data(unsigned char *p)
{
	int ret;
#if ENABLE_T007B1_SETTING
#if ENABLE_T007_ST_SCAN
	char buf[3];
#endif

	if (g_stCtrl.bICVersion != T007A6) {
		ret = rm31080_spi_byte_write(RM31080B1_REG_BANK0_00H, 0x00);
		if (ret)
			ret = rm31080_spi_byte_write(RM31080B1_REG_BANK0_01H,
						0x00);

		if (ret)
			ret = rm31080_spi_read(RM31080B1_REG_BANK0_03H|0x80, p,
						g_stCtrl.u16DataLength);
	} else
#endif
	{
		ret = rm31080_spi_byte_write(RM31080_REG_01, 0x10);
		if (ret)
			ret = rm31080_spi_byte_write(RM31080_REG_02, 0x00);

		if (ret)
			ret = rm31080_spi_read(RM31080_REG_80, p,
						g_stCtrl.u16DataLength);
	}

#if ENABLE_T007_ST_SCAN
	if (g_stCtrl.bICVersion == T007A6)
		return ret;

	if (!ret)
		return ret;

	buf[0] = RM31080B1_REG_BANK0_00H;
	if (g_stCtrl.bICVersion == T007_VERSION_B) {
		buf[1] = (T007B1_ST_DATA_ADDR >> 8) & 0x1F;
		buf[2] = T007B1_ST_DATA_ADDR & 0xFF;
	} else if (g_stCtrl.bICVersion == T007_VERSION_C) {
		buf[1] = (T007C1_ST_DATA_ADDR >> 8) & 0x1F;
		buf[2] = T007C1_ST_DATA_ADDR & 0xFF;
	}
	ret = rm31080_spi_burst_write(buf, sizeof(buf));

	if (ret)
		ret = rm31080_spi_read(RM31080B1_REG_BANK0_03H|0x80,
					p + RM_MAX_MT_COUNT,/*Marty 20121015*/
					RM_MAX_CHANNEL_COUNT);
#endif
	return ret;
}

#ifdef ENABLE_AUTO_SCAN
void rm_set_idle(u8 OnOff)
{
	u8 reg_46h;
	switch (OnOff) {
	case 1:
		rm31080_spi_read(0x46 | 0x80, &reg_46h, 1);
		rm31080_spi_byte_write(0x46, reg_46h | 0x30);
		break;
	default:
		rm31080_spi_read(0x46 | 0x80, &reg_46h, 1);
		rm31080_spi_byte_write(0x46, reg_46h & 0x0F);
		break;
	}
}

void rm_set_auto(u8 OnOff)
{
	u8 reg_09h = 0;
	u8 reg_0Ah = 0;
	switch (OnOff) {
	case 1:
		rm31080_spi_read(0x09 | 0x80, &reg_09h, 1);
		rm31080_spi_byte_write(0x09, reg_09h | 0x50);
		rm31080_spi_read(0x0A|0x80, &reg_0Ah, 1);
		rm31080_spi_byte_write(0x0A, reg_0Ah&0x0F);
		break;
	default:
		rm31080_spi_read(0x09 | 0x80, &reg_09h, 1);
		rm31080_spi_byte_write(0x09, reg_09h & 0x0F);
		rm31080_spi_read(0x0A|0x80, &reg_0Ah, 1);
		rm31080_spi_byte_write(0x0A, reg_0Ah|0x40);
		break;
	}
}

void rm31080_ctrl_enter_auto_mode(void)
{
#if ENABLE_FILTER_SWITCH
	/*rm_printk("Enable Analog Filter\n"); */
#if ENABLE_T007B1_SETTING
	if (g_stCtrl.bICVersion == T007A6)
#endif
		rm31080_analog_filter_config(REPEAT_1);
#endif
	/*Enable auto scan */
	if (g_stCtrl.bfIdleMessage)
		rm_printk("Enter Auto Scan Mode\n");
#if ENABLE_T007B1_SETTING
#if ENABLE_T007B1_STABLE_IDLE_MODE
	if (g_stCtrl.bICVersion != T007A6) {
		if (!g_stCtrl.bDummyRunCycle)
			rm_set_idle(1);
		rm_set_repeat_times(g_stCtrl.bIdleRepeatTimes[0]);
	}
#endif
#endif
#if ENABLE_T007_ST_SCAN
	rm_set_auto(1);
#else
	rm31080_spi_byte_write(RM31080_REG_09, 0x10 | 0x40);
#endif
}

void rm31080_ctrl_leave_auto_mode(void)
{
	/*Disable auto scan */
	if (g_stCtrl.bfIdleMessage)
		rm_printk("Leave Auto Scan Mode\n");
#if ENABLE_T007B1_SETTING
#if ENABLE_T007B1_STABLE_IDLE_MODE
	if (g_stCtrl.bICVersion != T007A6) {
		if (!g_stCtrl.bDummyRunCycle)
			rm_set_idle(0);
		rm_set_repeat_times(g_stCtrl.bRepeatTimes[0]);
	}
#endif
#endif
#if ENABLE_T007_ST_SCAN
	rm_set_auto(0);
#else
	rm31080_spi_byte_write(RM31080_REG_09, 0x00);
#endif
#if ENABLE_FILTER_SWITCH
#if ENABLE_T007B1_SETTING
	if (g_stCtrl.bICVersion == T007A6)
#endif
		rm31080_filter_config();
#endif
}

void rm31080_ctrl_pause_auto_mode(void)
{
	u8 u8reg11;
	rm31080_spi_byte_write(RM31080_REG_09, 0x40);	/*disable auto scan */
	rm31080_spi_byte_read(RM31080_REG_11, &u8reg11);
	u8reg11 &= ~0x01;
	rm31080_spi_byte_write(RM31080_REG_11, u8reg11);/*set scan start=0 */
}
#endif	/*ENABLE_AUTO_SCAN */

static u32 rm31080_ctrl_configure(void)
{
	u32 u32Flag;

	switch (g_stTs.u8ScanModeState) {
	case RM_SCAN_MODE_MANUAL:
		u32Flag =
			RM_NEED_TO_SEND_SCAN | RM_NEED_TO_READ_RAW_DATA |
			RM_NEED_TO_SEND_SIGNAL;

#if NOISE_SUM_CHECK
		if (g_stTs.u8Repeat) {
			rm_printk("Change Noise Mode\n");
			rm_set_repeat_times(g_stTs.u8Repeat);
			g_stTs.u8Repeat = 0;
		}
#endif

		break;
#ifdef ENABLE_AUTO_SCAN
	case RM_SCAN_MODE_PREPARE_AUTO:
		rm31080_ctrl_enter_auto_mode();
		g_stTs.u8ScanModeState = RM_SCAN_MODE_AUTO_SCAN;
		u32Flag = RM_NEED_NONE;
		break;
	case RM_SCAN_MODE_AUTO_SCAN:
		rm31080_ctrl_leave_auto_mode();
		rm31080_ctrl_scan_start();
		g_stTs.u8ScanModeState = RM_SCAN_MODE_MANUAL;
#if ENABLE_T007B1_SETTING
		if (g_stCtrl.bICVersion != T007A6) {
			u32Flag =
				RM_NEED_TO_SEND_SCAN |
				RM_NEED_TO_READ_RAW_DATA |
				RM_NEED_TO_SEND_SIGNAL;
		} else
#endif
			u32Flag = RM_NEED_TO_SEND_SCAN;
		break;
#endif				/*ENABLE_AUTO_SCAN */
	default:
		u32Flag = RM_NEED_NONE;
		break;
	}

	return u32Flag;
}

#endif	/*ENABLE_RAW_DATA_QUEUE */
/*========================================================================= */
#ifdef ENABLE_RM31080_DEEP_SLEEP

static int rm31080_ctrl_suspend(struct rm31080_ts *ts)
{
	/* handle touch suspend */
	int error;
	/*Flow designed by Roger 20110930 */
	/*rm31080_ts_send_signal(g_stTs.ulHalPID,RM_SIGNAL_SUSPEND); */
	g_stTs.bInitFinish = 0;
	mutex_lock(&g_stTs.mutex_scan_mode);
	usleep_range(8000, 9000);/*msleep(8); */
	rm31080_ctrl_clear_int();
	/*disable auto scan */

#if ENABLE_T007B1_SETTING
	if (g_stCtrl.bICVersion == T007A6) {
#endif
		rm31080_spi_byte_write(RM31080_REG_09, 0x00);
		rm31080_spi_byte_write(RM31080_REG_10, 0x14);
		rm31080_ctrl_scan_start();
		rm31080_ctrl_scan_start();
		usleep_range(15000, 20000);/*msleep(15); */
#if ENABLE_T007B1_SETTING
	}
#endif
	rm31080_spi_byte_write(RM31080_REG_11, 0x06);

	/* 1) disable (3.3v) */
	if (ts->regulator_3v3) {
		error = regulator_disable(ts->regulator_3v3);
		if (error < 0)
			dev_err(&g_spi->dev,
			"raydium regulator 3.3V disable failed: %d\n", error);

		usleep_range(5000, 6000);
		/* notifier handles the rest of the touch suspend */
	}

	usleep_range(15000, 16000);
	mutex_unlock(&g_stTs.mutex_scan_mode);
	return 1;
}
#endif

/*========================================================================= */
static void rm31080_enter_manual_mode(void)
{
	flush_workqueue(g_stTs.rm_workqueue);

	if (g_stTs.u8ScanModeState == RM_SCAN_MODE_MANUAL)
		return;

	if (g_stTs.u8ScanModeState == RM_SCAN_MODE_PREPARE_AUTO) {
		g_stTs.u8ScanModeState = RM_SCAN_MODE_MANUAL;
		return;
	}

	if (g_stTs.u8ScanModeState == RM_SCAN_MODE_AUTO_SCAN) {
		rm31080_ctrl_leave_auto_mode();
		g_stTs.u8ScanModeState = RM_SCAN_MODE_MANUAL;
		usleep_range(10000, 12000);/*msleep(10); */
	}
}

/*=============================================================================
 * Description:
 *      Copy Config(Parameters) to HAL's Buffer
 * Input:
 *      p: HAL's buffer
 *      u32Len : buffer size
 * Output:
 *      1: succeed
 *      0: failed
 *=============================================================================
 */
static long rm31080_get_config(u8 *p, u32 u32Len)
{
	u32 u32Ret;
	struct rm_spi_ts_platform_data *pdata;

#if ENABLE_T007B1_SETTING
	u8 var;
	if (rm31080_spi_byte_read(RM31080_REG_7E, &var))
		g_stCtrl.bICVersion = var & 0xF0;
	else
		g_stCtrl.bICVersion = T007A6;
#endif

	pdata = g_input_dev->dev.parent->platform_data;

	switch (g_stCtrl.bICVersion) {
	case T007_VERSION_B:
		u32Ret = copy_to_user(p, pdata->config +
				(PARAMETER_AMOUNT*VERSION_B_PARAMETER_OFFSET),
				u32Len);
		break;
	case T007_VERSION_C:
		u32Ret = copy_to_user(p, pdata->config +
				(PARAMETER_AMOUNT*VERSION_C_PARAMETER_OFFSET),
				u32Len);
		break;
	default:
		u32Ret = copy_to_user(p, pdata->config, u32Len);
	}

/*
#if ENABLE_T007B1_SETTING
	if (g_stCtrl.bICVersion != T007A6)
		u32Ret = copy_to_user(p, pdata->config +
					PARAMETER_AMOUNT, u32Len);
	else
#endif
		u32Ret = copy_to_user(p, pdata->config, u32Len);
*/
	if (u32Ret != 0)
		return 0;
	return 1;
}

static u32 rm31080_get_platform_id(u8 *p)
{
	u32 u32Ret;
	struct rm_spi_ts_platform_data *pdata;
	pdata = g_input_dev->dev.parent->platform_data;
	u32Ret = copy_to_user(p, &pdata->platform_id,
				sizeof(pdata->platform_id));
	if (u32Ret != 0)
		return 0;
	return 1;
}

/*=========================================================================*/
int rm31080_ts_send_signal(int pid, int iInfo)
{
	struct siginfo info;
	struct task_struct *t;
	int ret = 0;

	static DEFINE_MUTEX(lock);

	if (!pid)
		return ret;

	mutex_lock(&lock);
	/* send the signal */
	memset(&info, 0, sizeof(struct siginfo));
	info.si_signo = RM_TS_SIGNAL;
	info.si_code = SI_QUEUE;
	/* this is bit of a trickery: SI_QUEUE is normally
	 * used by sigqueue from user space, and kernel space
	 * should use SI_KERNEL. But if SI_KERNEL is used the real_time data
	 * is not delivered to the user space signal handler function.
	 */
	info.si_int = iInfo;/*real time signals may have 32 bits of data. */

	rcu_read_lock();
	t = find_task_by_vpid(pid);
	rcu_read_unlock();
	if (t == NULL) {
		rmd_printk("no such pid\n");
		ret = -ENODEV;
	} else
		ret = send_sig_info(RM_TS_SIGNAL, &info, t);

	if (ret < 0)
		rmd_printk("error sending signal\n");

	mutex_unlock(&lock);
	return ret;
}

/*=============================================================================
 * Description:
 *      Queuing functions.
 * Input:
 *      N/A
 * Output:
 *      0:succeed
 *      others:error code
 *=============================================================================
 */
#ifdef ENABLE_RAW_DATA_QUEUE
static void rm31080_queue_reset(void)
{
	g_stQ.u16Rear = 0;
	g_stQ.u16Front = 0;
}

static int rm31080_queue_init(void)
{
	rm31080_queue_reset();
	g_stQ.pQueue = kmalloc(QUEUE_COUNT * RAW_DATA_LENGTH, GFP_KERNEL);
	if (g_stQ.pQueue == NULL)
		return -ENOMEM;
	return 0;
}

static void rm31080_queue_free(void)
{
	if (!g_stQ.pQueue)
		return;
	kfree(g_stQ.pQueue);
	g_stQ.pQueue = NULL;
}

#ifdef ENABLE_CALC_QUEUE_COUNT
static int rm31080_queue_get_current_count(void)
{
	if (g_stQ.u16Rear >= g_stQ.u16Front)
		return g_stQ.u16Rear - g_stQ.u16Front;

	return (QUEUE_COUNT - g_stQ.u16Front) + g_stQ.u16Rear;
}
#endif

/*=============================================================================
 * Description:
 *  About full/empty buffer distinction,
 *  There are a number of solutions like:
 *  1.Always keep one slot open.
 *  2.Use a fill count to distinguish the two cases.
 *  3.Use read and write counts to get the fill count from.
 *  4.Use absolute indices.
 *  we chose "keep one slot open" to make it simple and robust
 *  and also avoid race condition.
 * Input:
 *      N/A
 * Output:
 *      1:empty
 *      0:not empty
 *=============================================================================
 */
static int rm31080_queue_is_empty(void)
{
	if (g_stQ.u16Rear == g_stQ.u16Front)
		return 1;
	return 0;
}

/*=============================================================================
 * Description:
 *  check queue full.
 * Input:
 *      N/A
 * Output:
 *      1:full
 *      0:not full
 *=============================================================================
 */
static int rm31080_queue_is_full(void)
{
	u16 u16Front = g_stQ.u16Front;
	if (g_stQ.u16Rear + 1 == u16Front)
		return 1;

	if ((g_stQ.u16Rear == (QUEUE_COUNT - 1)) && (u16Front == 0))
		return 1;

	return 0;
}

static void *rm31080_enqueue_start(void)
{
	if (!g_stQ.pQueue)	/*error handling for no memory */
		return NULL;

	if (!rm31080_queue_is_full())
		return &g_stQ.pQueue[g_stQ.u16Rear];

	/* rm_printk("rm31080:touch service is busy,try again.\n"); */
	return NULL;
}

static void rm31080_enqueue_finish(void)
{
	if (g_stQ.u16Rear == (QUEUE_COUNT - 1))
		g_stQ.u16Rear = 0;
	else
		g_stQ.u16Rear++;
}

static void *rm31080_dequeue_start(void)
{
	if (!rm31080_queue_is_empty())
		return &g_stQ.pQueue[g_stQ.u16Front];

	return NULL;
}

static void rm31080_dequeue_finish(void)
{
	if (g_stQ.u16Front == (QUEUE_COUNT - 1))
		g_stQ.u16Front = 0;
	else
		g_stQ.u16Front++;
}

static long rm31080_queue_read_raw_data(u8 *p, u32 u32Len)
{
	u8 *pQueue;
	u32 u32Ret;
	pQueue = rm31080_dequeue_start();
	if (!pQueue)
		return 0;

	u32Ret = copy_to_user(p, pQueue, u32Len);
	if (u32Ret != 0)
		return 0;

	rm31080_dequeue_finish();
	return 1;
}
#endif				/*ENABLE_RAW_DATA_QUEUE */
/*=====================================================================*/
#ifdef ENABLE_WORK_QUEUE
static void rm_work_handler(struct work_struct *work)
{
	void *pKernelBuffer;
	u32 u32Flag;
	int iRet;

	if (g_stTs.bIsSuspended)
		return;

	mutex_lock(&g_stTs.mutex_scan_mode);

	iRet = rm31080_ctrl_clear_int();

	u32Flag = rm31080_ctrl_configure();

	if (u32Flag & RM_NEED_TO_SEND_SCAN)
		rm31080_ctrl_scan_start();

	if (u32Flag & RM_NEED_TO_READ_RAW_DATA) {
		pKernelBuffer = rm31080_enqueue_start();
		if (pKernelBuffer) {
			iRet = rm31080_ctrl_read_raw_data((u8 *) pKernelBuffer);
#if !NOISE_SUM_CHECK
			if (iRet)
				iRet = rm_noise_main((s8 *) pKernelBuffer);
#endif
			if (iRet)
				rm31080_enqueue_finish();
		}
	}
	mutex_unlock(&g_stTs.mutex_scan_mode);

	if (u32Flag & RM_NEED_TO_SEND_SIGNAL) {
		if (g_stTs.bCalcFinish) {
			g_stTs.bCalcFinish = 0;
			rm31080_ts_send_signal(g_stTs.ulHalPID, RM_SIGNAL_INTR);
		}
	}
}
#endif				/*ENABLE_WORK_QUEUE */
/*========================================================================= */
static void __rm31080_enable(struct rm31080_ts *ts)
{
	enable_irq(ts->irq);
}

static void __rm31080_disable(struct rm31080_ts *ts)
{
	disable_irq(ts->irq);
}

void rm31080_send_command(struct rm_cmd_list *rm_cmd_list)
{
	unsigned char i;
	unsigned char addr;
	unsigned char value;

	for (i = 0; i < rm_cmd_list->count; i++) {
		addr = rm_cmd_list->cmd[i].addr;
		value = rm_cmd_list->cmd[i].value;
		rmd_printk("No:[%d]addr:0x%x,value:0x%x\n", i, addr, value);
		rm31080_spi_byte_write(addr, value);
	}
}

#ifdef ENABLE_SLOW_SCAN
void rm31080_set_command(struct rm_cmd_list *cmd_list, char *buf, int count)
{
	int i;
	if (count > RM_SLOW_SCAN_CMD_COUNT)
		count = RM_SLOW_SCAN_CMD_COUNT;
	cmd_list->count = count;
	for (i = 0; i < count; i++) {
		cmd_list->cmd[i].addr  = buf[(i<<1) + 0];
		cmd_list->cmd[i].value = buf[(i<<1) + 1];
	}
}

static void rm31080_set_slowscan_para(u8 *p, int index)
{
	ssize_t missing;
	u8 buf[256];
	u8 size = p[0];
	missing = copy_from_user(buf, p, size);
	if (missing != 0)
		return;
	if (index > RM_SLOW_SCAN_LEVEL_MAX)
		return;
	rm31080_set_command((struct rm_cmd_list *)
		&g_stCmdSlowScan[index],
		&buf[1],
		(size - 1) >> 1);
}

void rm31080_slow_scan_init(void)
{
	int i;
	for (i = 0; i < RM_SLOW_SCAN_LEVEL_COUNT; i++)
		g_stCmdSlowScan[i].count = 0;
}

/*=============================================================================
 * Description:
 *      Context dependent touch system.
 *      Change scan speed for slowscan function.
 *      Change scan speed flow: (by CY,20120305)
 *      1.Disable auto scan ([0x09]bit4=0,[0x09]bit6=1)
 *      2.Clear Scan start bit ([0x11]bit0=0)
 *      3.Read Scan start bit until it equals 0
 *      4.Set LACTIVE and YACTIVE configuration
 *      5.Enable autoscan ([0x09]bit4=1,[0x09]bit6=1)
 *      6.Sleep 1 minisecond.
 *      7.Set Scan start bit ([0x11]bit0=1)
 * Input:
 *      N/A
 * Output:
 *      N/A
 *=============================================================================
 */
static void rm31080_ctrl_slowscan(u32 level)
{
	if (g_stTs.u8ScanModeState == RM_SCAN_MODE_AUTO_SCAN)
		rm31080_ctrl_pause_auto_mode();

	rm31080_ctrl_wait_for_scan_finish();

	rmd_printk("P005:change level:%d\n", level);

	if (level == RM_SLOW_SCAN_LEVEL_NORMAL)
		level = RM_SLOW_SCAN_LEVEL_20;

	if (level > RM_SLOW_SCAN_LEVEL_100)
		level = RM_SLOW_SCAN_LEVEL_MAX;

	rm31080_send_command((struct rm_cmd_list *)
				&g_stCmdSlowScan[level]);

	if (g_stTs.u8ScanModeState == RM_SCAN_MODE_AUTO_SCAN) {
		rm31080_ctrl_enter_auto_mode();
		usleep_range(1000, 2000);/*msleep(1);*/
		rm31080_ctrl_scan_start();
	}
}

static u32 rm31080_slowscan_round(u32 val)
{
	u32 i;
	for (i = 0; i < RM_SLOW_SCAN_LEVEL_COUNT; i++) {
		if ((i * RM_SLOW_SCAN_INTERVAL) >= val)
			break;
	}
	return i;

}

static ssize_t rm31080_slowscan_handler(const char *buf, size_t count)
{
	unsigned long val;
	ssize_t error;
	ssize_t ret;

	if (count < 2)
		return count;

	ret = (ssize_t) count;
	mutex_lock(&g_stTs.mutex_scan_mode);

	if (count == 2) {
		if (buf[0] == '0') {
			g_stTs.bEnableSlowScan = false;
			rm31080_ctrl_slowscan(g_stTs.u32SlowScanLevel);
		} else if (buf[0] == '1') {
			g_stTs.bEnableSlowScan = true;
			rm31080_ctrl_slowscan(RM_SLOW_SCAN_LEVEL_20);
		}
	} else if ((buf[0] == '2') && (buf[1] == ' ')) {
		error = strict_strtoul(&buf[2], 10, &val);
		if (error) {
			ret = error;
		} else {
			g_stTs.bEnableSlowScan = false;
			g_stTs.u32SlowScanLevel = rm31080_slowscan_round(val);
			rm31080_ctrl_slowscan(g_stTs.u32SlowScanLevel);
		}
	}

	mutex_unlock(&g_stTs.mutex_scan_mode);
	return ret;
}
#endif

static ssize_t rm31080_slowscan_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
#ifdef ENABLE_SLOW_SCAN
	return sprintf(buf, "Slow Scan:%s\nScan Rate:%dHz\n",
			g_stTs.bEnableSlowScan ?
			"Enabled" : "Disabled",
			g_stTs.bEnableSlowScan ?
			RM_SLOW_SCAN_LEVEL_20 * RM_SLOW_SCAN_INTERVAL :
			g_stTs.u32SlowScanLevel * RM_SLOW_SCAN_INTERVAL);

#else
	return sprintf(buf, "0\n");
#endif
}

static ssize_t rm31080_slowscan_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
#ifdef ENABLE_SLOW_SCAN
	return rm31080_slowscan_handler(buf, count);
#else
	return count;
#endif
}

static void rm31080_smooth_level_change(unsigned long val)
{
	int iInfo;

	if (val > RM_SMOOTH_LEVEL_MAX)
		return;

	g_stTs.u32SmoothLevel = val;

	iInfo = (RM_SIGNAL_PARA_SMOOTH << 24) |
			(val << 16) |
			RM_SIGNAL_CHANGE_PARA;

	rm31080_ts_send_signal(g_stTs.ulHalPID, iInfo);
}

static ssize_t rm31080_smooth_level_handler(const char *buf, size_t count)
{
	unsigned long val;
	ssize_t error;
	ssize_t ret;

	if (count != 2)
		return count;

	ret = (ssize_t) count;
	error = strict_strtoul(buf, 10, &val);
	if (error)
		ret = error;
	else
		rm31080_smooth_level_change(val);

	return ret;
}

static ssize_t rm31080_self_test_handler(const char *buf, size_t count)
{
	unsigned long val;
	ssize_t error;
	ssize_t ret;
	int iInfo;

	ret = (ssize_t) count;

	if (count != 2)
		return ret;

	if (g_stTs.u8SelfTestStatus == RM_SELF_TEST_STATUS_TESTING)
		return ret;
	g_stTs.u8SelfTestResult = RM_SELF_TEST_RESULT_PASS;

	error = strict_strtoul(buf, 10, &val);
	if (error) {
		ret = error;
	} else if (val == 0) {
		g_stTs.bInitFinish = 0;
		msleep(1000);
		rm31080_spi_checking(1);
	} else if ((val >= 0x01) && (val <= 0xFF)) {
		g_stTs.u8SelfTestStatus = RM_SELF_TEST_STATUS_TESTING;
		iInfo = (RM_SIGNAL_PARA_SELF_TEST << 24) |
				(val << 16) |
				RM_SIGNAL_CHANGE_PARA;
		rm31080_ts_send_signal(g_stTs.ulHalPID, iInfo);
	}

	return ret;
}

static ssize_t rm31080_smooth_level_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return sprintf(buf, "Smooth level:%d\n", g_stTs.u32SmoothLevel);
}

static ssize_t rm31080_smooth_level_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	rm31080_smooth_level_handler(buf, count);
	return count;
}

static ssize_t rm31080_self_test_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	return sprintf(buf, "Self_Test:Status:%d ,Result:%d\n",
					g_stTs.u8SelfTestStatus,
					g_stTs.u8SelfTestResult);
}

static ssize_t rm31080_self_test_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	rm31080_self_test_handler(buf, count);
	return count;
}

static ssize_t rm31080_version_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return sprintf(buf, "0x%02X\n", g_stTs.u8Version);
}

static ssize_t rm31080_version_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return count;
}

static void rm31080_ctrl_registry(void)
{
	int i;
	unsigned char buf;
	if (g_stTs.u8ScanModeState == RM_SCAN_MODE_AUTO_SCAN)
		rm31080_ctrl_pause_auto_mode();

	for (i = 4; i < 128; i++) {
		rm31080_spi_byte_read(i, &buf);
		rm_printk("BANK0 %2xh = 0x%2x\n", i, buf);
	}
	rm31080_spi_byte_write(0x7F, 0x01);
	for (i = 0; i < 128; i++) {
		rm31080_spi_byte_read(i, &buf);
		rm_printk("BANK1 %2xh = 0x%2x\n", i, buf);
	}
	rm31080_spi_byte_write(0x7F, 0x00);

	if (g_stTs.u8ScanModeState == RM_SCAN_MODE_AUTO_SCAN) {
		rm31080_ctrl_enter_auto_mode();
		usleep_range(1000, 2000);/*msleep(1); */
		rm31080_ctrl_scan_start();
	}
}

static ssize_t rm31080_registry_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	mutex_lock(&g_stTs.mutex_scan_mode);

	rm31080_ctrl_registry();

	mutex_unlock(&g_stTs.mutex_scan_mode);

	return sprintf(buf, "Finish!\n");
}

static ssize_t rm31080_registry_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(slowscan_enable, 0666, rm31080_slowscan_show,
			rm31080_slowscan_store);
static DEVICE_ATTR(smooth_level, 0666, rm31080_smooth_level_show,
			rm31080_smooth_level_store);
static DEVICE_ATTR(self_test, 0666, rm31080_self_test_show,
			rm31080_self_test_store);
static DEVICE_ATTR(version, 0666, rm31080_version_show,
			rm31080_version_store);
static DEVICE_ATTR(registry, 0666, rm31080_registry_show,
			rm31080_registry_store);

static struct attribute *rm_ts_attributes[] = {
	&dev_attr_slowscan_enable.attr,
	&dev_attr_smooth_level.attr,
	&dev_attr_self_test.attr,
	&dev_attr_version.attr,
	&dev_attr_registry.attr,
	NULL
};

static const struct attribute_group rm_ts_attr_group = {
	.attrs = rm_ts_attributes,
};

static int rm31080_input_open(struct input_dev *input)
{
	struct rm31080_ts *ts = input_get_drvdata(input);

	if (!ts->disabled && !ts->suspended)
		__rm31080_enable(ts);

	return 0;
}

static void rm31080_input_close(struct input_dev *input)
{
	struct rm31080_ts *ts = input_get_drvdata(input);

	if (!ts->disabled && !ts->suspended)
		__rm31080_disable(ts);
}

/*=========================================================================*/
static irqreturn_t rm31080_irq(int irq, void *handle)
{
	/*struct rm31080_ts *ts = handle; */
	if (!g_stTs.bInitFinish)
		return IRQ_HANDLED;

#ifdef ENABLE_WORK_QUEUE
	queue_work(g_stTs.rm_workqueue, &g_stTs.rm_work);
#endif
	return IRQ_HANDLED;
}

/*==========================================================================*/
static void rm31080_init_ts_structure_part(void)
{
	g_stTs.bInitFinish = 0;
	g_stTs.bCalcFinish = 0;
	g_stTs.bEnableScriber = 0;
	g_stTs.bIsSuspended = 0;
	g_stTs.bEnableAutoScan = 1;

	g_stTs.bEnableSlowScan = false;

#ifdef ENABLE_RAW_DATA_QUEUE
	g_stTs.u8ScanModeState = RM_SCAN_MODE_MANUAL;
#endif
	rm31080_ctrl_init();
}

void rm31080_disable_touch(void)
{
	g_stTs.bInitFinish = 0;
}

void rm31080_enable_touch(void)
{
	g_stTs.bInitFinish = 1;
}

void rm31080_set_autoscan(unsigned char val)
{
	g_stTs.bEnableAutoScan = val;
}

void rm31080_set_variable(unsigned int index, unsigned int arg)
{
	switch (index) {
	case RM_VARIABLE_SELF_TEST_RESULT:
		g_stTs.u8SelfTestResult = (u8) arg;
		g_stTs.u8SelfTestStatus = RM_SELF_TEST_STATUS_FINISH;
		break;
	case RM_VARIABLE_SCRIBER_FLAG:
		g_stTs.bEnableScriber = (bool) arg;
		break;
	case RM_VARIABLE_AUTOSCAN_FLAG:
		g_stTs.bEnableAutoScan = (bool) arg;
		break;
	case RM_VARIABLE_VERSION:
		g_stTs.u8Version = (u8) arg;
		dev_info(&g_spi->dev, "Raydium TS:Firmware v%d\n",
				g_stTs.u8Version);
		break;
	case RM_VARIABLE_REPEAT:
		rm_printk("Repeat %d\n", arg);
		g_stTs.u8Repeat = (u8) arg;
		break;
	default:
		break;
	}

}

static u32 rm31080_get_variable(unsigned int index, unsigned int arg)
{
	u32 ret = 0;
	switch (index) {
	case RM_VARIABLE_PLATFORM_ID:
		ret = rm31080_get_platform_id((u8 *) arg);
		break;
	default:
		break;
	}

	return ret;
}

static void rm31080_init_ts_structure(void)
{
	g_stTs.ulHalPID = 0;
	memset(&g_stTs, 0, sizeof(struct rm31080a_ts_para));

#ifdef ENABLE_SLOW_SCAN
	rm31080_slow_scan_init();
	g_stTs.u32SlowScanLevel = RM_SLOW_SCAN_LEVEL_140;
#endif

#ifdef ENABLE_WORK_QUEUE
	g_stTs.rm_workqueue = create_singlethread_workqueue("rm_work");
	INIT_WORK(&g_stTs.rm_work, rm_work_handler);
#endif
	mutex_init(&g_stTs.mutex_scan_mode);

}

/*=========================================================================*/
#ifdef ENABLE_SPI_SETTING
static int rm31080_spi_setting(u32 speed)
{
	int err;
	if ((speed == 0) || (speed > 18))
		return false;

	g_spi->max_speed_hz = speed * 1000 * 1000;
	err = spi_setup(g_spi);
	if (err) {
		dev_dbg(&g_spi->dev, "Change SPI setting failed\n");
		return false;
	}
	return true;
}
#endif
/*=========================================================================*/
static int rm31080_spi_checking(bool bInfinite)
{
	unsigned int i;
	unsigned int iTestCount = 0;
	unsigned char wbuf[] = { 0x50, 0x55, 0xAA, 0x00, 0xFF };
	unsigned char rbuf[sizeof(wbuf)];
	unsigned int iLen;
	int iFail;

#if ENABLE_T007B1_SETTING
	unsigned char var;
	if (rm31080_spi_byte_read(RM31080_REG_7E, &var)) {
		if ((var & 0xF0) != T007A6) {
			wbuf[0] = 0x6E;
			var = 0x01;
			rm31080_spi_byte_write(RM31080_REG_7F, var);
		}
	}
#endif

	rbuf[0] = wbuf[0] | 0x80;	/*address */
	iLen = sizeof(wbuf) - 1;
	do {
		spi_write(g_spi, wbuf, sizeof(wbuf));
		memset(&rbuf[1], 0, iLen);
		/*spi_write_then_read(g_spi, &rbuf[0], 1, &rbuf[1], iLen); */
		rm31080_spi_read(rbuf[0], &rbuf[1], iLen);

		/*compare */
		iFail = 0;
		for (i = 1; i < iLen + 1; i++) {
			if (wbuf[i] != rbuf[i]) {
				iFail = 1;
				break;
			}
		}

		if (iFail) {
			rm_printk("Raydium SPI Checking:Compare fail\n");
			rm_printk("SPI Speed:%d hz,Mode:%x\n",
					g_spi->max_speed_hz, g_spi->mode);
			for (i = 1; i < iLen + 1; i++) {
				rm_printk("[%02d]Write Data:0x%02x,", i,
					wbuf[i]);
				rm_printk("Read Data:0x%02x\n", rbuf[i]);
			}
			msleep(300);
		} else {
			iTestCount++;
			if ((iTestCount % 10000) == 0) {
				rm_printk("SPI test:%d times ok.", iTestCount);
				rm_printk("(Speed:%d hz,Mode:%x)\n",
				g_spi->max_speed_hz, g_spi->mode);
			}
		}
	} while (bInfinite);

	if (!iFail) {
#if ENABLE_T007B1_SETTING
		if (wbuf[0] == 0x6E) {
			var = 0x00;
			rm31080_spi_byte_write(RM31080_REG_7F, var);
		}
#endif
		rm_printk("Raydium SPI Checking: ok\n");
	}

	return !iFail;

}

static int rm31080_voltage_notifier_3v3(struct notifier_block *nb,
					unsigned long event, void *ignored)
{
	int error;
	struct rm31080_ts *ts = input_get_drvdata(g_input_dev);

	rm_printk("rm31080 REGULATOR EVENT:0x%x\n", (unsigned int)event);

	if (event & REGULATOR_EVENT_POST_ENABLE) {
		/* handle the rest of touch resume */
		/* 2. sleep 5ms */
		usleep_range(5000, 6000);

		/* 3. enable 1.8 */
		if (ts->regulator_1v8) {
			error = regulator_enable(ts->regulator_1v8);
			if (error < 0)
				dev_err(&g_spi->dev,
				"raydium regulator 1.8V enable failed: %d\n",
					error);
		}

	} else if (event & REGULATOR_EVENT_DISABLE) {
		/* handle the rest of touch suspend */
		/* 2) sleep 5ms */
		usleep_range(5000, 6000);
		/* 3) disable 1.8 */
		if (ts->regulator_1v8) {
			error = regulator_disable(ts->regulator_1v8);
			if (error < 0)
				dev_err(&g_spi->dev,
				"raydium regulator 1.8V disable failed: %d\n",
					error);
		}
	}

	return NOTIFY_OK;
}

static int rm31080_voltage_notifier_1v8(struct notifier_block *nb,
					unsigned long event, void *ignored)
{
	struct rm_spi_ts_platform_data *pdata;
	pdata = g_input_dev->dev.parent->platform_data;

	rm_printk("rm31080 REGULATOR EVENT:0x%x\n", (unsigned int)event);

	/* if (event & REGULATOR_EVENT_POST_ENABLE) { */
		/* 4. enable clock */

		/* 5. sleep 1ms */

	if (event & REGULATOR_EVENT_DISABLE) {
		/* 4) pull low reset */
		gpio_set_value(pdata->gpio_reset, 0);

		/* 5) disable clock */
	}
	return NOTIFY_OK;
}

/*=========================================================================*/
static void rm31080_start(struct rm31080_ts *ts) /* handle touch resume */
{
#ifdef ENABLE_RM31080_DEEP_SLEEP
	struct rm_spi_ts_platform_data *pdata;
#endif
	int error;

	if (!g_stTs.bIsSuspended)
		return;
	g_stTs.bIsSuspended = false;

	mutex_lock(&g_stTs.mutex_scan_mode);

#ifdef ENABLE_RM31080_DEEP_SLEEP
	pdata = g_input_dev->dev.parent->platform_data;
	/* 1. enable (3.3v) */
	if (ts->regulator_3v3) {
		error = regulator_enable(ts->regulator_3v3);
		if (error < 0)
			dev_err(&g_spi->dev,
			"raydium regulator 3.3V enable failed: %d\n", error);

		/* notifier handles the rest of the touch resume */
		msleep(20);
	}

	/* 5. pull high reset */
	gpio_set_value(pdata->gpio_reset, 0);
	msleep(120);
	gpio_set_value(pdata->gpio_reset, 1);
	usleep_range(15000, 19000);/*msleep(10); */

	rm31080_init_ts_structure_part();
	rm31080_ts_send_signal(g_stTs.ulHalPID, RM_SIGNAL_RESUME);
#elif defined(ENABLE_AUTO_SCAN)
	rm31080_ctrl_clear_int();
	rm31080_ctrl_scan_start();
#endif
	mutex_unlock(&g_stTs.mutex_scan_mode);

}

static void rm31080_stop(struct rm31080_ts *ts)
{
	if (g_stTs.bIsSuspended)
		return;

	g_stTs.bIsSuspended = true;

	flush_workqueue(g_stTs.rm_workqueue);

#ifdef ENABLE_RM31080_DEEP_SLEEP
	rm31080_ctrl_suspend(ts);
#endif
}

#ifdef CONFIG_PM
static int rm31080_suspend(struct device *dev)
{
	struct rm31080_ts *ts = dev_get_drvdata(dev);
	rm31080_stop(ts);
	return 0;
}

static int rm31080_resume(struct device *dev)
{
	struct rm31080_ts *ts = dev_get_drvdata(dev);
	rm31080_start(ts);
	return 0;
}

static const struct dev_pm_ops rm31080_pm_ops = {
	.suspend = rm31080_suspend,
	.resume = rm31080_resume,
};

#endif				/*CONFIG_PM */

/* NVIDIA 20121026*/
#ifdef ENABLE_NEW_INPUT_DEV
static int rm31080_input_enable(struct input_dev *in_dev)
{
	int error = 0;

#ifdef CONFIG_PM
	struct rm31080_ts *ts = input_get_drvdata(in_dev);

	error = rm31080_resume(ts->dev);
	if (error)
		dev_err(ts->dev, "%s: failed\n", __func__);
#endif

	return error;
}

static int rm31080_input_disable(struct input_dev *in_dev)
{
	int error = 0;

#ifdef CONFIG_PM
	struct rm31080_ts *ts = input_get_drvdata(in_dev);

	error = rm31080_suspend(ts->dev);
	if (error)
		dev_err(ts->dev, "%s: failed\n", __func__);
#endif

	return error;
}
#endif /*ENABLE_NEW_INPUT_DEV */

static void rm31080_set_input_resolution(unsigned int x, unsigned int y)
{
	input_set_abs_params(g_input_dev, ABS_X, 0, x - 1, 0, 0);
	input_set_abs_params(g_input_dev, ABS_Y, 0, y - 1, 0, 0);
	input_set_abs_params(g_input_dev, ABS_MT_POSITION_X, 0, x - 1, 0, 0);
	input_set_abs_params(g_input_dev, ABS_MT_POSITION_Y, 0, y - 1, 0, 0);
}

struct rm31080_ts *rm31080_input_init(struct device *dev, unsigned int irq,
					const struct rm31080_bus_ops *bops)
{

	struct rm31080_ts *ts;
	struct input_dev *input_dev;
	int err;

	if (!irq) {
		dev_err(dev, "no IRQ?\n");
		err = -EINVAL;
		goto err_out;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);

	input_dev = input_allocate_device();

	if (!ts || !input_dev) {
		dev_err(dev, "Failed to allocate memory\n");
		err = -ENOMEM;
		goto err_free_mem;
	}

	g_input_dev = input_dev;

	ts->bops = bops;
	ts->dev = dev;
	ts->input = input_dev;
	ts->irq = irq;

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(dev));

	input_dev->name = "raydium_ts";
	input_dev->phys = ts->phys;
	input_dev->dev.parent = dev;
	input_dev->id.bustype = bops->bustype;

	input_dev->open = rm31080_input_open;
	input_dev->close = rm31080_input_close;
#ifdef ENABLE_NEW_INPUT_DEV
	input_dev->enable = rm31080_input_enable;
	input_dev->disable = rm31080_input_disable;
	input_dev->enabled = true;
#endif
	input_dev->hint_events_per_packet = 256U;

	input_set_drvdata(input_dev, ts);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	rm31080_set_input_resolution(RM_INPUT_RESOLUTION_X,
						RM_INPUT_RESOLUTION_Y);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 0xFF, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 32, 0, 0);

	err = request_threaded_irq(ts->irq, NULL, rm31080_irq,
					IRQF_TRIGGER_RISING, dev_name(dev), ts);
	if (err) {
		dev_err(dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}
	mutex_init(&ts->access_mutex);

	__rm31080_disable(ts);

	err = sysfs_create_group(&dev->kobj, &rm_ts_attr_group);
	if (err)
		goto err_free_irq;

	err = input_register_device(input_dev);
	if (err)
		goto err_remove_attr;

	return ts;

 err_remove_attr:
	sysfs_remove_group(&dev->kobj, &rm_ts_attr_group);
 err_free_irq:
	free_irq(ts->irq, ts);
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
 err_out:
	return ERR_PTR(err);
}

static int dev_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int dev_release(struct inode *inode, struct file *filp)
{
	g_stTs.bInitFinish = 0;
	rm31080_enter_manual_mode();
	return 0;
}

static ssize_t
dev_read(struct file *filp, char __user * buf, size_t count, loff_t * pos)
{
	ssize_t missing, status;
	int ret;
	u8 *pMyBuf;

	pMyBuf = kmalloc(count, GFP_KERNEL);
	if (pMyBuf == NULL)
		return -ENOMEM;

	pMyBuf[0] = buf[0];
	ret = rm31080_spi_read(pMyBuf[0], pMyBuf, count);

	if (ret) {
		status = count;
		missing = copy_to_user(buf, pMyBuf, count);

		if (missing == status)
			status = -EFAULT;
		else
			status = status - missing;
	} else {
		status = -EFAULT;
		rmd_printk("rm31080_spi_read() fail\n");
	}

	kfree(pMyBuf);
	return status;
}

static ssize_t
dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *pos)
{
	u8 *pMyBuf;
	int ret;
	unsigned long missing;
	ssize_t status = 0;

	pMyBuf = kmalloc(count, GFP_KERNEL);
	if (pMyBuf == NULL)
		return -ENOMEM;

	missing = copy_from_user(pMyBuf, buf, count);
	if (missing == 0) {
		ret = rm31080_spi_write(pMyBuf, count);
		if (ret)
			status = count;
		else
			status = -EFAULT;
	} else
		status = -EFAULT;

	kfree(pMyBuf);
	return status;
}

/*==============================================================================
 * Description:
 *      I/O Control routine.
 * Input:
 *      file:
 *      cmd :
 *      arg :
 * Output:
 *      1: succeed
 *      0: failed
 * Note: To avoid context switch,
 *       please don't add debug message in this function.
 *==============================================================================
 */
static long dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = true;
	unsigned int index;
	index = (cmd >> 16) & 0xFFFF;
	switch (cmd & 0xFFFF) {
	case RM_IOCTL_REPORT_POINT:
		raydium_report_pointer((void *)arg);
		break;
	case RM_IOCTL_SET_HAL_PID:
		g_stTs.ulHalPID = arg;
		break;
	case RM_IOCTL_INIT_START:
		g_stTs.bInitFinish = 0;
		rm31080_enter_manual_mode();
		break;
	case RM_IOCTL_INIT_END:
		g_stTs.bInitFinish = 1;
		g_stTs.bCalcFinish = 1;
#ifdef ENABLE_RAW_DATA_QUEUE
		ret = rm31080_ctrl_scan_start();
#endif
		break;
	case RM_IOCTL_FINISH_CALC:
		g_stTs.bCalcFinish = 1;
		break;
	case RM_IOCTL_SCRIBER_CTRL:
		g_stTs.bEnableScriber = (bool) arg;
		break;
	case RM_IOCTL_AUTOSCAN_CTRL:
		g_stTs.bEnableAutoScan = (bool) arg;
		break;
#ifdef ENABLE_RAW_DATA_QUEUE
	case RM_IOCTL_READ_RAW_DATA:
		ret = rm31080_queue_read_raw_data((u8 *) arg, index);
		break;
#endif
	case RM_IOCTL_NOISE_CHECK:
		ret = rm31080_ctrl_get_noise_mode((u8 *) arg);
		break;
	case RM_IOCTL_GET_PARAMETER:
		rm31080_ctrl_get_parameter((void *)arg);

#if ENABLE_RESOLUTION_SWITCH
		rm31080_set_input_resolution(g_stCtrl.u16ResolutionX,
						g_stCtrl.u16ResolutionY);
#endif
		break;
	case RM_IOCTL_SET_PARAMETER:
		ret = rm31080_get_config((u8 *) arg, index);
		break;
	case RM_IOCTL_SEND_BASELINE:	/* Noise_Detector */
		rm31080_ctrl_set_baseline((void *)arg);
		break;
#if ENABLE_NEW_NOISE_MODE
	case RM_IOCTL_SEND_ANALOG_BASELINE:	/* Noise_Detector */
		rm31080_ctrl_set_analog_baseline((void *)arg);
		break;
#endif
	case RM_IOCTL_SET_VARIABLE:
		rm31080_set_variable(index, arg);
		break;

	case RM_IOCTL_GET_VARIABLE:
		ret = rm31080_get_variable(index, arg);
		break;
#ifdef ENABLE_SLOW_SCAN
	case RM_IOCTL_SET_SLOWSCAN_PARA:
		rm31080_set_slowscan_para((u8 *) arg, index);
		break;
#endif

	default:
		break;
	}
	return ret;
}

static const struct file_operations dev_fops = {
	.owner = THIS_MODULE,
	.open = dev_open,
	.release = dev_release,
	.read = dev_read,
	.write = dev_write,
	.unlocked_ioctl = dev_ioctl,
};

static struct miscdevice raydium_ts_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "raydium_ts",
	.fops = &dev_fops,
};

static const struct rm31080_bus_ops rm31080_spi_bus_ops = {
	.bustype = BUS_SPI,
};

static int rm31080_misc_init(void)
{
	int err;
	err = sysfs_create_group(&raydium_ts_miscdev.this_device->kobj,
				 &rm_ts_attr_group);
	if (err)
		return false;
	return true;
}

static void rm31080_init_regulator(struct rm31080_ts *ts)
{
	int error;

	ts->regulator_3v3 = devm_regulator_get(&g_spi->dev, "avdd");
	if (IS_ERR(ts->regulator_3v3)) {
		dev_err(&g_spi->dev, "Raydium TS: regulator_get failed: %ld\n",
			PTR_ERR(ts->regulator_3v3));
		goto err_null_regulator;
	}

	ts->regulator_1v8 = devm_regulator_get(&g_spi->dev, "dvdd");
	if (IS_ERR(ts->regulator_1v8)) {
		dev_err(&g_spi->dev, "Raydium TS: regulator_get failed: %ld\n",
			PTR_ERR(ts->regulator_1v8));
		goto err_put_regulator_3v3;
	}

	error = regulator_enable(ts->regulator_3v3);
	if (error < 0)
		dev_err(&g_spi->dev,
			"Raydium TS: regulator enable failed: %d\n", error);

	error = regulator_enable(ts->regulator_1v8);
	if (error < 0)
		dev_err(&g_spi->dev,
			"Raydium TS: regulator enable failed: %d\n", error);

	ts->nb_3v3.notifier_call = &rm31080_voltage_notifier_3v3;
	error = regulator_register_notifier(ts->regulator_3v3, &ts->nb_3v3);
	if (error) {
		dev_err(&g_spi->dev,
			"regulator notifier request failed: %d\n", error);
		goto err_disable_regulator;
	}

	ts->nb_1v8.notifier_call = &rm31080_voltage_notifier_1v8;
	error = regulator_register_notifier(ts->regulator_1v8, &ts->nb_1v8);
	if (error) {
		dev_err(&g_spi->dev,
			"regulator notifier request failed: %d\n", error);
		devm_regulator_put(ts->regulator_1v8);
		goto err_unregister_notifier_3v3;
	}

	return;

err_unregister_notifier_3v3:
	regulator_unregister_notifier(ts->regulator_3v3, &ts->nb_3v3);
err_disable_regulator:
	regulator_disable(ts->regulator_3v3);
	regulator_disable(ts->regulator_1v8);
err_put_regulator_1v8:
	devm_regulator_put(ts->regulator_1v8);
err_put_regulator_3v3:
	devm_regulator_put(ts->regulator_3v3);
err_null_regulator:
	ts->regulator_3v3 = NULL;
	ts->regulator_1v8 = NULL;
}

static int rm31080_spi_remove(struct spi_device *spi)
{
	struct rm31080_ts *ts = spi_get_drvdata(spi);

#ifdef ENABLE_RAW_DATA_QUEUE
	rm31080_queue_free();
#endif

#ifdef ENABLE_WORK_QUEUE
	if (g_stTs.rm_workqueue)
		destroy_workqueue(g_stTs.rm_workqueue);
#endif
	sysfs_remove_group(&raydium_ts_miscdev.this_device->kobj,
			   &rm_ts_attr_group);
	misc_deregister(&raydium_ts_miscdev);
	sysfs_remove_group(&ts->dev->kobj, &rm_ts_attr_group);
	free_irq(ts->irq, ts);
	input_unregister_device(ts->input);
	if (ts->regulator_3v3 && ts->regulator_1v8) {
		regulator_unregister_notifier(ts->regulator_3v3, &ts->nb_3v3);
		regulator_unregister_notifier(ts->regulator_1v8, &ts->nb_1v8);
		regulator_disable(ts->regulator_3v3);
		regulator_disable(ts->regulator_1v8);
	}
	kfree(ts);
	spi_set_drvdata(spi, NULL);
	return 0;
}

static int rm31080_spi_probe(struct spi_device *spi)
{
	struct rm31080_ts *ts;

	g_spi = spi;

	rm31080_init_ts_structure();

	ts = rm31080_input_init(&spi->dev, spi->irq, &rm31080_spi_bus_ops);
	if (IS_ERR(ts))
		return PTR_ERR(ts);
	spi_set_drvdata(spi, ts);

	rm31080_init_regulator(ts);

	if (spi->max_speed_hz > MAX_SPI_FREQ_HZ) {
		dev_err(&spi->dev, "SPI CLK %d Hz?\n", spi->max_speed_hz);
		goto err_unregister_notifier;
	}

	rm31080_init_ts_structure_part();

	if (!rm31080_spi_checking(0))
		goto err_unregister_notifier;

	if (misc_register(&raydium_ts_miscdev) != 0) {
		dev_err(&spi->dev, "Raydium TS: cannot register miscdev\n");
		return 0;
	}
	rm31080_misc_init();

#ifdef ENABLE_RAW_DATA_QUEUE
	rm31080_queue_init();
#endif
	return 0;

err_unregister_notifier:
	regulator_unregister_notifier(ts->regulator_3v3, &ts->nb_3v3);
	regulator_unregister_notifier(ts->regulator_1v8, &ts->nb_1v8);
err_disable_regulator:
	regulator_disable(ts->regulator_3v3);
	regulator_disable(ts->regulator_1v8);
	return -EINVAL;
}

static struct spi_driver rm31080_spi_driver = {
	.driver = {
			.name = "rm_ts_spidev",
			.bus = &spi_bus_type,
			.owner = THIS_MODULE,

#if defined(CONFIG_PM)
			.pm = &rm31080_pm_ops,
#endif
			},
	.probe = rm31080_spi_probe,
	.remove = rm31080_spi_remove,
};

static int __init rm31080_spi_init(void)
{
	return spi_register_driver(&rm31080_spi_driver);
}

static void __exit rm31080_spi_exit(void)
{
	spi_unregister_driver(&rm31080_spi_driver);
}

module_init(rm31080_spi_init);
module_exit(rm31080_spi_exit);

MODULE_AUTHOR("Valentine Hsu <valentine.hsu@rad-ic.com>");
MODULE_DESCRIPTION("Raydium touchscreen SPI bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:raydium-t007");
