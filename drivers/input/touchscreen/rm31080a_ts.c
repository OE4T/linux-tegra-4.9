/*

 * Raydium RM31080(T007) touchscreen (SPI bus) - Android version
 *
 * Copyright (C) 2011-2012 Raydium Inc.
 *
 * Licensed under the GPL-2 or later.
 *
 * Version : 0.04
 */

//=============================================================================
//INCLUDED FILES
//=============================================================================
#include <linux/input.h>	// BUS_SPI
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>	// wake_up_process()
#include <linux/kthread.h>	// kthread_create()、kthread_run()
#include <asm/uaccess.h>	// copy_to_user(),
#include <linux/miscdevice.h>
#include <asm/siginfo.h>	// siginfo
#include <linux/rcupdate.h>	// rcu_read_lock
#include <linux/sched.h>	// find_task_by_pid_type
#include <linux/syscalls.h>	// sys_clock_gettime()
#include <linux/module.h>

#include <linux/spi/rm31080a_ts.h>
//=============================================================================
//DEFINITIONS
//=============================================================================
#define ENABLE_WORK_QUEUE
#define ENABLE_REPORT_TO_UART
#define ENABLE_RM31080_DEEP_SLEEP
#define ENABLE_AUTO_SCAN
//#define ENABLE_AUTO_FREQ
//#define ENABLE_SPEED_TEST_FUNCTION
//#define ENABLE_TEST_AVERAGE
//#define ENABLE_CALC_QUEUE_COUNT

#define MAX_SPI_FREQ_HZ      50000000
#define TS_PEN_UP_TIMEOUT    msecs_to_jiffies(50)

#ifdef ENABLE_RAW_DATA_QUEUE
#define QUEUE_COUNT       128
#define RAW_DATA_LENGTH  2048

#define RM_SCAN_MODE_MANUAL          0x00
#define RM_SCAN_MODE_PREPARE_AUTO    0x01
#define RM_SCAN_MODE_AUTO_SCAN       0x02

#define RM_NEED_NONE                 0x00
#define RM_NEED_TO_SEND_SCAN         0x01
#define RM_NEED_TO_READ_RAW_DATA     0x02
#define RM_NEED_TO_SEND_SIGNAL       0x03
#endif

#ifdef ENABLE_WORK_QUEUE
#include <linux/workqueue.h>
#endif

//=============================================================================
//STRUCTURE DECLARATION
//=============================================================================
struct rm31080a_ts_para {
	unsigned long ulHalPID;
	bool bInitFinish;
	bool bCalcFinish;
	bool bEnableScriber;
	bool bEnableAutoScan;
	bool bIsSuspended;
	struct mutex mutex;
#ifdef ENABLE_WORK_QUEUE
	struct workqueue_struct *rm_workqueue;
	struct work_struct rm_work;
	bool bIsWorkQueueExecuting;
#endif
#ifdef ENABLE_RAW_DATA_QUEUE
	u8 u8ScanModeState;
#endif
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
};

struct rm31080_bus_ops {
	u16 bustype;
	int (*read) (struct device * dev, u8 reg);
	int (*multi_read) (struct device * dev, u8 first_reg, u8 count,
			   u16 * buf);
	int (*write) (struct device * dev, u8 reg, u16 val);
};

#ifdef ENABLE_RAW_DATA_QUEUE
struct rm31080_queue_info {
	u8(*pQueue)[RAW_DATA_LENGTH];
	u16 u16Front;
	u16 u16Rear;
};
#endif

//=============================================================================
//GLOBAL VARIABLES DECLARATION
//=============================================================================
struct input_dev *g_input_dev;
struct spi_device *g_spi;
struct rm31080a_ts_para g_stTs;

#ifdef ENABLE_RAW_DATA_QUEUE
struct rm31080_queue_info g_stQ;
#endif

//=============================================================================
//FUNCTION DECLARATION
//=============================================================================

//=============================================================================
// Description:
//      Debug function: test speed.
// Input:
//      N/A
// Output:
//      1:succeed
//      0:failed
//=============================================================================
#ifdef ENABLE_SPEED_TEST_FUNCTION
void my_calc_time(int iStart)
{
	static volatile unsigned int u32Max = UINT_MAX;

	static long iTimebuffer[1000];
	static unsigned long long t1, t2;
	unsigned long nanosec_rem;
	static int iIndex = 0;

	if (iStart) {
		t1 = cpu_clock(u32Max);
		return;
	} else
		t2 = cpu_clock(u32Max);

	t2 = t2 - t1;

	nanosec_rem = do_div(t2, 1000000000);

	if (t2) {			//more than 1 Second
		iTimebuffer[iIndex] = 999999;
	} else {
		iTimebuffer[iIndex] = nanosec_rem / 1000;	//micro second
	}

	iIndex++;
	if (iIndex == 1000) {
		for (iIndex = 0; iIndex < 1000; iIndex++) {
			printk("   %04d,%06d\n", iIndex,
			       (u32) iTimebuffer[iIndex]);
		}
		iIndex = 0;
	}

}
#endif				//ENABLE_SPEED_TEST_FUNCTION
//=============================================================================
// Description:
//      RM31080 spi interface.
// Input:
//      N/A
// Output:
//      1:succeed
//      0:failed
//=============================================================================
int rm31080_spi_read(u8 u8addr, u8 * rxbuf, size_t len)
{
	static DEFINE_MUTEX(lock);

	int status;
	struct spi_message message;
	struct spi_transfer x[2];

	if (!mutex_trylock(&lock)) {
		//printk("Raydium TS: rm31080_spi_read trylock fail\n");
		return -EINVAL;
	}

	spi_message_init(&message);
	memset(x, 0, sizeof x);

	u8addr |= 0x80;
	x[0].len = 1;
	x[0].tx_buf = &u8addr;
	spi_message_add_tail(&x[0], &message);

	x[1].len = len;
	x[1].rx_buf = rxbuf;
	spi_message_add_tail(&x[1], &message);

	status = spi_sync(g_spi, &message);

	mutex_unlock(&lock);
	return status;		// 0 = succeed
}

int rm31080_spi_write(u8 * txbuf, size_t len)
{
	return spi_write(g_spi, txbuf, len);
}

static int rm31080_spi_byte_read(u8 u8Addr, u8 * pu8Value)
{
	int iErrorCode;
	iErrorCode = rm31080_spi_read(u8Addr, pu8Value, 1);
	if (iErrorCode != 0) {
		return 0;	//fail
	}
	return 1;
}

static int rm31080_spi_byte_write(u8 u8Addr, u8 u8Value)
{
	int iErrorCode;
	u8 buf[2];
	buf[0] = u8Addr;
	buf[1] = u8Value;

	iErrorCode = rm31080_spi_write(buf, 2);

	if (iErrorCode != 0) {
		//printk("rm31080_spi_write_byte failed:Reg=%x", u8Addr);
		return 0;	//fail
	}
	return 1;
}

//=============================================================================
// Description:
//      RM31080 control functions.
// Input:
//      N/A
// Output:
//      1:succeed
//      0:failed
//=============================================================================
#ifdef ENABLE_RAW_DATA_QUEUE

#define RM31080_REG_01 0x01
#define RM31080_REG_02 0x02
#define RM31080_REG_09 0x09
#define RM31080_REG_0E 0x0E
#define RM31080_REG_10 0x10
#define RM31080_REG_11 0x11
#define RM31080_REG_1F 0x1F
#define RM31080_REG_40 0x40
#define RM31080_REG_41 0x41
#define RM31080_REG_80 0x80
#define RM31080_REG_F2 0xF2

#define RM31080_RAW_DATA_LENGTH 1530
static int rm31080_ctrl_clear_int(void)
{
	u8 u8Flag;
	return rm31080_spi_byte_read(RM31080_REG_F2, &u8Flag);
}

#ifdef ENABLE_AUTO_SCAN
void rm31080_ctrl_enter_auto_mode(void)
{
	//Enable auto scan
	rm31080_spi_byte_write(RM31080_REG_09, 0x10 | 0x40);
}

void rm31080_ctrl_leave_auto_mode(void)
{
	//Disable auto scan
	rm31080_spi_byte_write(RM31080_REG_09, 0x00);
}
#endif				//ENABLE_AUTO_SCAN

#ifdef ENABLE_RM31080_DEEP_SLEEP
static int rm31080_ctrl_suspend(void)
{
	//Flow designed by Roger 20110930
	//rm31080_ts_send_signal(g_stTs.ulHalPID,RM_SIGNAL_SUSPEND);
	g_stTs.bInitFinish = 0;
	msleep(8);
	rm31080_ctrl_clear_int();
	//disable auto scan
	rm31080_spi_byte_write(RM31080_REG_09, 0x00);
	rm31080_spi_byte_write(RM31080_REG_10, 0x14);
	rm31080_spi_byte_write(RM31080_REG_11, 0x17);
	msleep(15);
	rm31080_spi_byte_write(RM31080_REG_11, 0x06);
	return 1;
}
#endif

static int rm31080_ctrl_scan_start(void)
{
	return rm31080_spi_byte_write(RM31080_REG_11, 0x17);
}

static u32 rm31080_ctrl_configure(void)
{
	u32 u32Flag;

	switch (g_stTs.u8ScanModeState) {
	case RM_SCAN_MODE_MANUAL:
		u32Flag =
		    RM_NEED_TO_SEND_SCAN | RM_NEED_TO_READ_RAW_DATA |
		    RM_NEED_TO_SEND_SIGNAL;
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
		u32Flag =
		    RM_NEED_TO_SEND_SCAN | RM_NEED_TO_READ_RAW_DATA |
		    RM_NEED_TO_SEND_SIGNAL;
		break;
#endif				//ENABLE_AUTO_SCAN
	default:
		u32Flag = RM_NEED_NONE;
		break;
	}

	return u32Flag;
}

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
		msleep(10);
	}
}

static int rm31080_ctrl_read_raw_data(u8 * p)
{
	int iRet;
	iRet = rm31080_spi_byte_write(RM31080_REG_01, 0x10);
	if (iRet)
		iRet = rm31080_spi_byte_write(RM31080_REG_02, 0x00);

	if (iRet) {
		iRet = rm31080_spi_read(RM31080_REG_80, p, RM31080_RAW_DATA_LENGTH);	//return 0 =succeed
		iRet = !iRet;
	}

	if (!iRet) {
		//printk("rm31080 read raw data failed\n");
	}

	return iRet;
}
#endif				//ENABLE_RAW_DATA_QUEUE
//=============================================================================
// Description:
//      Queuing functions.
// Input:
//      N/A
// Output:
//      0:succeed
//      others:error code
//=============================================================================
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
	if (g_stQ.pQueue == NULL) {
		//printk("rm31080_queue_init failed\n");
		return -ENOMEM;
	}
	return 0;
}

static void rm31080_queue_free(void)
{
	if (!g_stQ.pQueue)
		return;
	kfree(g_stQ.pQueue);
	g_stQ.pQueue = NULL;
}

//=============================================================================
// Description:
//  About full/empty buffer distinction,
//  There are a number of solutions like:
//  1.Always keep one slot open.
//  2.Use a fill count to distinguish the two cases.
//  3.Use read and write counts to get the fill count from.
//  4.Use absolute indices.
//  we chose "keep one slot open" to make it simple and robust
//  and also avoid race condition.
// Input:
//      N/A
// Output:
//      1:empty
//      0:not empty
//=============================================================================
static int rm31080_queue_is_empty(void)
{
	if (g_stQ.u16Rear == g_stQ.u16Front)
		return 1;
	return 0;
}

//=============================================================================
// Description:
//  check queue full.
// Input:
//      N/A
// Output:
//      1:full
//      0:not full
//=============================================================================
static int rm31080_queue_is_full(void)
{
	if (g_stQ.u16Rear + 1 == g_stQ.u16Front)
		return 1;

	if ((g_stQ.u16Rear == (QUEUE_COUNT - 1)) && (g_stQ.u16Front == 0))
		return 1;

	return 0;
}

#ifdef ENABLE_CALC_QUEUE_COUNT
static int rm31080_queue_get_current_count(void)
{
	if (g_stQ.u16Rear >= g_stQ.u16Front)
		return g_stQ.u16Rear - g_stQ.u16Front;

	return (QUEUE_COUNT - g_stQ.u16Front) + g_stQ.u16Rear;
}
#endif
static void *rm31080_enqueue_start(void)
{
	if (!g_stQ.pQueue)	//error handling for no memory
		return NULL;

	if (!rm31080_queue_is_full())
		return &g_stQ.pQueue[g_stQ.u16Rear];

	//printk("rm31080 Queue full with Queue Count:%d\n", QUEUE_COUNT);
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

static long rm31080_queue_read_raw_data(u8 * p, u32 u32Len)
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
#endif				//ENABLE_RAW_DATA_QUEUE

//=============================================================================
// Description:
//      Copy Config(Parameters) to HAL's Buffer
// Input:
//      p: HAL's buffer
//      u32Len : buffer size
// Output:
//      1: succeed
//      0: failed
//=============================================================================
static long rm31080_get_config(u8 * p, u32 u32Len)
{
	u32 u32Ret;
	struct rm_spi_ts_platform_data *pdata;
	pdata = g_input_dev->dev.parent->platform_data;
	u32Ret = copy_to_user(p, pdata->config, u32Len);
	if (u32Ret != 0)
		return 0;
	return 1;
}

#ifdef ENABLE_AUTO_FREQ
void raydium_auto_freq()
{
	g_stTs.bInitFinish = 0;
	msleep(10);
	rm31080_ctrl_clear_int();

	//roger_auto_freq_detection();

	g_stTs.bInitFinish = 1;
	rm31080_ctrl_scan_start();

}
#endif				//ENABLE_TEST_AUTO_FREQ
//=============================================================================
#ifdef ENABLE_AUTO_SCAN
void raydium_change_scan_mode(u8 u8TouchCount)
{
	static u32 u32NoTouchCount = 0;
	if (u8TouchCount) {
		u32NoTouchCount = 0;
		return;
	}
	if (u32NoTouchCount < 100) {
		u32NoTouchCount++;
	} else if (g_stTs.u8ScanModeState == RM_SCAN_MODE_MANUAL) {
#ifdef ENABLE_AUTO_FREQ
		raydium_auto_freq();
#else
		if (g_stTs.bEnableAutoScan)
			g_stTs.u8ScanModeState = RM_SCAN_MODE_PREPARE_AUTO;
#endif
		u32NoTouchCount = 0;
	}
}
#endif				//ENABLE_AUTO_SCAN
//=============================================================================
//report touch data for scriber
//
//=============================================================================
#ifdef ENABLE_REPORT_TO_UART
void raydium_report_to_uart_printf(unsigned char *ucData, unsigned char ucCount)
{
	unsigned char i;
	for (i = 0; i < ucCount; i++) {
		printk("%02X", ucData[i]);
	}
	printk("\n");
}

void raydium_report_to_uart(void *p)
{
	unsigned char ucData[1 + 1 + (4 * 12) + 1];	//1=Tag,1=Touch count,4=(xH xL ,yH yL) ,12=max point,1=Check sum
	rm_touch_event *spTP;
	unsigned short usX, usY;
	int i, j;

	if (g_stTs.bEnableScriber == 0)
		return;

	spTP = (rm_touch_event *) p;

	ucData[0] = 0x8E;
	ucData[1] = spTP->ucTouchCount;
	j = 2;
	for (i = 0; i < spTP->ucTouchCount; i++) {
		usX = spTP->usX[i] + 1;	//1~1536
		usY = spTP->usY[i] + 1;	//1~960
		ucData[j++] = ((usX >> 8) & 0xFF) | (spTP->ucID[i] << 4);	//add id
		ucData[j++] = ((usX) & 0xFF);
		ucData[j++] = ((usY >> 8) & 0xFF);
		ucData[j++] = ((usY) & 0xFF);
	}

	//check sum
	ucData[j] = 0;
	for (i = 0; i < j; i++) {
		ucData[j] += ucData[i];
	}
	ucData[j] = 0x100 - ucData[j];
	j++;

	//print
	raydium_report_to_uart_printf(ucData, j);
	if (spTP->ucTouchCount == 0)	//send more , to avoid losing
	{
		raydium_report_to_uart_printf(ucData, j);
		raydium_report_to_uart_printf(ucData, j);
	}
}
#endif
//=============================================================================
void raydium_report_pointer(void *p)
{
	static unsigned char ucLastTouchCount = 0;
	int i;
	int iCount;
	rm_touch_event *spTP;
	spTP = (rm_touch_event *) p;

	iCount = max(ucLastTouchCount, spTP->ucTouchCount);
	if (iCount) {
		for (i = 0; i < iCount; i++) {
			if (i == 10)
				break;	//due to the "touch test" can't support great than 10 points

			if (i < spTP->ucTouchCount) {
				input_report_abs(g_input_dev,
						 ABS_MT_TRACKING_ID,
						 spTP->ucID[i]);

				input_report_abs(g_input_dev,
						 ABS_MT_TOUCH_MAJOR, 100);
				if (spTP->usX[i] >= (RM_INPUT_RESOLUTION_X - 1))
					input_report_abs(g_input_dev, ABS_MT_POSITION_X, (RM_INPUT_RESOLUTION_X - 1) - 1);	//fixed bug: OS scale fail
				else
					input_report_abs(g_input_dev,
							 ABS_MT_POSITION_X,
							 spTP->usX[i]);

				if (spTP->usY[i] >= (RM_INPUT_RESOLUTION_Y - 1))
					input_report_abs(g_input_dev, ABS_MT_POSITION_Y, (RM_INPUT_RESOLUTION_Y - 1) - 1);	//fixed bug: OS scale fail
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
	raydium_change_scan_mode(spTP->ucTouchCount);
#endif
}

//=============================================================================

//=============================================================================
int rm31080_ts_send_signal(int pid, int iInfo)
{
	struct siginfo info;
	struct task_struct *t;
	int ret;

	/* send the signal */
	memset(&info, 0, sizeof(struct siginfo));
	info.si_signo = RM_TS_SIGNAL;
	info.si_code = SI_QUEUE;	// this is bit of a trickery: SI_QUEUE is normally used by sigqueue from user space,
	// and kernel space should use SI_KERNEL. But if SI_KERNEL is used the real_time data
	// is not delivered to the user space signal handler function.
	info.si_int = iInfo;	//real time signals may have 32 bits of data.

	rcu_read_lock();
	t = find_task_by_vpid(pid);
	if (t == NULL) {
		//printk("no such pid\n");
		rcu_read_unlock();
		return -ENODEV;
	}
	rcu_read_unlock();
	ret = send_sig_info(RM_TS_SIGNAL, &info, t);	//send the signal
	if (ret < 0) {
		//printk("error sending signal\n");
		return ret;
	}

	return ret;
}

//=============================================================================
static void __rm31080_enable(struct rm31080_ts *ts)
{
	enable_irq(ts->irq);
}

static void __rm31080_disable(struct rm31080_ts *ts)
{
	disable_irq(ts->irq);
}

static void vtest_toggle(struct rm31080_ts *ts, bool disable)
{
	mutex_lock(&ts->input->mutex);

	if (!ts->suspended && ts->input->users != 0) {

		if (disable) {
			if (ts->disabled)
				__rm31080_enable(ts);
		} else {
			if (!ts->disabled)
				__rm31080_disable(ts);
		}
	}

	ts->disabled = disable;

	mutex_unlock(&ts->input->mutex);
}

static ssize_t vtest_disable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct rm31080_ts *ts = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ts->disabled);
}

static ssize_t vtest_disable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct rm31080_ts *ts = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	vtest_toggle(ts, val);

	return count;
}

static DEVICE_ATTR(disable, 0664, vtest_disable_show, vtest_disable_store);
static struct attribute *vtest_attributes[] = {
	&dev_attr_disable.attr,
	NULL
};

static const struct attribute_group vtest_attr_group = {
	.attrs = vtest_attributes,
};

static int rm31080_input_open(struct input_dev *input)
{
	struct rm31080_ts *ts = input_get_drvdata(input);

	/* protected by input->mutex */
	if (!ts->disabled && !ts->suspended)
		__rm31080_enable(ts);

	return 0;
}

static void rm31080_input_close(struct input_dev *input)
{
	struct rm31080_ts *ts = input_get_drvdata(input);

	/* protected by input->mutex */
	if (!ts->disabled && !ts->suspended)
		__rm31080_disable(ts);
}

//=============================================================================

#ifdef ENABLE_TEST_AVERAGE	//only for test
#define _AVERAGE_COUNT 2
s8 g_bAverageBuf[_AVERAGE_COUNT][2048];
int test_soft_average(s8 * pSource)
{
	static u8 u8AverageIndex = 0;
	static u8 u8StartAverage = 0;
	u16 i, j;
	s16 s16Sum;

	for (i = 0; i < RM31080_RAW_DATA_LENGTH; i++)	//RM31080_RAW_DATA_LENGTH =1530
		g_bAverageBuf[u8AverageIndex][i] = pSource[i] - 0x80;
	u8AverageIndex++;

	if (u8AverageIndex == _AVERAGE_COUNT) {
		u8StartAverage = 1;
		u8AverageIndex = 0;
	}
	else
	{
		u8StartAverage = 0;
	}

	if (u8StartAverage) {
		for (i = 0; i < RM31080_RAW_DATA_LENGTH; i++) {
			s16Sum = 0;
			for (j = 0; j < _AVERAGE_COUNT; j++)
				s16Sum += g_bAverageBuf[j][i];
			pSource[i] = (s16Sum / _AVERAGE_COUNT) + 0x80;
		}
		return 1;
	}
	return 0;
}
#endif

#ifdef ENABLE_WORK_QUEUE
//1.2
static void rm_work_handler(struct work_struct *work)
{
	void *pKernelBuffer;
	u32 u32Flag;
	int iRet;


	if (g_stTs.bIsSuspended) {
		//printk("rm_work_handler stops after suspend\n");
		return;
	}

	g_stTs.bIsWorkQueueExecuting = 1;

	iRet = rm31080_ctrl_clear_int();

	u32Flag = rm31080_ctrl_configure();

	if (u32Flag | RM_NEED_TO_SEND_SCAN) {
		rm31080_ctrl_scan_start();
	}

	if (u32Flag | RM_NEED_TO_READ_RAW_DATA) {
		pKernelBuffer = rm31080_enqueue_start();
		if (pKernelBuffer) {
			iRet = rm31080_ctrl_read_raw_data((u8 *) pKernelBuffer);
#ifdef ENABLE_TEST_AVERAGE
			if (iRet) {
				iRet = test_soft_average((s8 *) pKernelBuffer);
			}
#endif
			if (iRet) {
				rm31080_enqueue_finish();
			}
		}
	}

	if (u32Flag | RM_NEED_TO_SEND_SIGNAL) {
		if (g_stTs.bCalcFinish) {
			g_stTs.bCalcFinish = 0;
			rm31080_ts_send_signal(g_stTs.ulHalPID, RM_SIGNAL_INTR);
		}
	}
	g_stTs.bIsWorkQueueExecuting = 0;
}
#endif

static irqreturn_t rm31080_irq(int irq, void *handle)
{

	//struct rm31080_ts *ts = handle;
	if (!g_stTs.bInitFinish) {
		return IRQ_HANDLED;
	}

#ifdef ENABLE_WORK_QUEUE
	queue_work(g_stTs.rm_workqueue, &g_stTs.rm_work);
#endif

	return IRQ_HANDLED;
}

//=============================================================================
static void rm31080_init_ts_structure_part(void)
{
	g_stTs.bInitFinish = 0;
	g_stTs.bCalcFinish = 0;
	g_stTs.bEnableScriber = 0;
	g_stTs.bIsSuspended = 0;
	g_stTs.bEnableAutoScan = 1;

#ifdef ENABLE_RAW_DATA_QUEUE
	g_stTs.u8ScanModeState = RM_SCAN_MODE_MANUAL;
#endif
}

static void rm31080_init_ts_structure(void)
{
	g_stTs.ulHalPID = 0;

	memset(&g_stTs, 0, sizeof(struct rm31080a_ts_para));

#ifdef ENABLE_WORK_QUEUE
	g_stTs.rm_workqueue = create_singlethread_workqueue("rm_work");
	INIT_WORK(&g_stTs.rm_work, rm_work_handler);
	g_stTs.bIsWorkQueueExecuting = 0;
#endif
}

//=============================================================================
static void rm31080_start(struct rm31080_ts *ts)
{
#ifdef ENABLE_RM31080_DEEP_SLEEP
	struct rm_spi_ts_platform_data *pdata;
#endif

	if (!g_stTs.bIsSuspended)
		return;
	g_stTs.bIsSuspended = 0;

#ifdef ENABLE_RM31080_DEEP_SLEEP
	//flow designed by Roger //20110930
	pdata = g_input_dev->dev.parent->platform_data;
	gpio_set_value(pdata->gpio_reset, 0);
	msleep(120);
	gpio_set_value(pdata->gpio_reset, 1);
	msleep(10);
	rm31080_init_ts_structure_part();
	rm31080_ts_send_signal(g_stTs.ulHalPID, RM_SIGNAL_RESUME);
#elif defined(ENABLE_AUTO_SCAN)
	rm31080_ctrl_clear_int();
	rm31080_ctrl_scan_start();
#endif

}

static void rm31080_stop(struct rm31080_ts *ts)
{
	int iCount;
	if (g_stTs.bIsSuspended)
		return;

	iCount = 0;
	while (g_stTs.bIsWorkQueueExecuting) {
		//printk("Raydium TS: Work_Queue is Executing.\n");
		msleep(1);
		iCount++;
		if (iCount > 1000)
			break;
	}
	g_stTs.bIsSuspended = 1;

#ifdef ENABLE_RM31080_DEEP_SLEEP
	rm31080_ctrl_suspend();
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
#endif

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

	input_set_drvdata(input_dev, ts);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	__set_bit(ABS_PRESSURE, input_dev->absbit);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);


	input_set_abs_params(input_dev, ABS_X,
			     0, RM_INPUT_RESOLUTION_X - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, RM_INPUT_RESOLUTION_Y - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 1, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, RM_INPUT_RESOLUTION_X - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, RM_INPUT_RESOLUTION_Y - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 32, 0, 0);

	err = request_threaded_irq(ts->irq, NULL, rm31080_irq,
				   IRQF_TRIGGER_RISING, dev_name(dev), ts);
	if (err) {
		dev_err(dev, "irq %d busy?\n", ts->irq);
		goto err_free_mem;
	}

	mutex_init(&ts->access_mutex);

	__rm31080_disable(ts);

	err = sysfs_create_group(&dev->kobj, &vtest_attr_group);
	if (err)
		goto err_free_irq;

	err = input_register_device(input_dev);
	if (err)
		goto err_remove_attr;

	return ts;

 err_remove_attr:
	sysfs_remove_group(&dev->kobj, &vtest_attr_group);
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
	unsigned long missing;
	ssize_t status = 0;
	u8 *pMyBuf;

	pMyBuf = kmalloc(count, GFP_KERNEL);
	if (pMyBuf == NULL)
		return -ENOMEM;

	pMyBuf[0] = buf[0];
	status = rm31080_spi_read(pMyBuf[0], pMyBuf, count);

	if (status != 0) {
		//printk("rm31080_spi_read() fail\n");
	}
	status = count;
	missing = copy_to_user(buf, pMyBuf, count);

	if (missing == status)
		status = -EFAULT;
	else
		status = status - missing;

	kfree(pMyBuf);
	return status;
}

static ssize_t
dev_write(struct file *filp, const char __user * buf,
	  size_t count, loff_t * pos)
{
	u8 *pMyBuf;
	unsigned long missing;
	ssize_t status = 0;

	pMyBuf = kmalloc(count, GFP_KERNEL);
	if (pMyBuf == NULL)
		return -ENOMEM;

	missing = copy_from_user(pMyBuf, buf, count);
	if (missing == 0) {
		status = rm31080_spi_write(pMyBuf, count);
	} else
		status = -EFAULT;

	kfree(pMyBuf);
	return count;
}

//=============================================================================
// Description:
//      I/O Control routin.
// Input:
//      file:
//      cmd :
//      arg :
// Output:
//      1: succeed
//      0: failed
//=============================================================================
static long dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long ret = 1;
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
		ret =
		    rm31080_queue_read_raw_data((u8 *) arg,
						(cmd >> 16) & 0xFFFF);
		break;
#endif
	case RM_IOCTL_SET_PARAMETER:
		ret = rm31080_get_config((u8 *) arg, (cmd >> 16) & 0xFFFF);
		break;
	default:
		break;
	}
	return ret;
}

static struct file_operations dev_fops = {
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

	misc_deregister(&raydium_ts_miscdev);

	sysfs_remove_group(&ts->dev->kobj, &vtest_attr_group);
	free_irq(ts->irq, ts);
	input_unregister_device(ts->input);
	kfree(ts);
	spi_set_drvdata(spi, NULL);
	return 0;
}

static int rm31080_spi_probe(struct spi_device *spi)
{
	struct rm31080_ts *ts;

	rm31080_init_ts_structure();
	rm31080_init_ts_structure_part();

	if (spi->max_speed_hz > MAX_SPI_FREQ_HZ) {
		dev_err(&spi->dev, "SPI CLK %d Hz?\n", spi->max_speed_hz);
		return -EINVAL;
	}

	ts = rm31080_input_init(&spi->dev, spi->irq, &rm31080_spi_bus_ops);
	if (IS_ERR(ts))
		return PTR_ERR(ts);
	spi_set_drvdata(spi, ts);

	g_spi = spi;

	if (misc_register(&raydium_ts_miscdev) != 0) {
		dev_err(&spi->dev, "Raydium TS: cannot register miscdev\n");
		return 0;
	}
#ifdef ENABLE_RAW_DATA_QUEUE
	rm31080_queue_init();
#endif

	return 0;
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
module_init(rm31080_spi_init);

static void __exit rm31080_spi_exit(void)
{
	spi_unregister_driver(&rm31080_spi_driver);
}
module_exit(rm31080_spi_exit);

MODULE_AUTHOR("Valentine Hsu <valentine.hsu@rad-ic.com>");
MODULE_DESCRIPTION("Raydium touchscreen SPI bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:raydium-t007");
