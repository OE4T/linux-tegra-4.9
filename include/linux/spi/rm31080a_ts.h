#ifndef _RM31080A_TS_H_
#define _RM31080A_TS_H_

#define ENABLE_RAW_DATA_QUEUE
#define ENABLE_SLOW_SCAN

#define ENABLE_TOUCH_RESPONSE_TEST		1	/*Roger*/
#define ENABLE_RESOLUTION_SWITCH		1	/*Alex*/
#define ENABLE_FILTER_SWITCH			0	/*Cage*/
#define ENABLE_NEW_NOISE_MODE			0	/*Marty*/
#define NOISE_SUM_CHECK				0	/*Nelson*/
#define ENABLE_CALIBRATTION_BY_FIRMWARE		1
#define ENABLE_NEW_PARAMETER			1

#define ENABLE_T007B1_SETTING			1
#define ENABLE_T007B1_STABLE_IDLE_MODE		1

#define ENABLE_T007_ST_SCAN			0
	#define T007B1_ST_DATA_ADDR		0x1CA0
	#define T007C1_ST_DATA_ADDR		0x0514
	#define T007B1_ST_NO_SWAP		0
#define RM_MAX_CHANNEL_COUNT			128
#define RM_MAX_MT_COUNT				2048

/* Define for T007 A6/B1 IC version ckeck */
#define T007A6			0xD0
#define T007_VERSION_B		0xB0
	#define T007B1			0xB0
	#define T007B2			0xB1
#define T007_VERSION_C		0xC0

#define VERSION_A_PARAMETER_OFFSET 0x00
#define VERSION_B_PARAMETER_OFFSET 0x01
#define VERSION_C_PARAMETER_OFFSET 0x02
#define PARAMETER_AMOUNT 384

#define RM_IOCTL_REPORT_POINT			0x1001
#define RM_IOCTL_SET_HAL_PID			0x1002
#define RM_IOCTL_INIT_START			0x1003
#define RM_IOCTL_INIT_END			0x1004
#define RM_IOCTL_FINISH_CALC			0x1005
#define RM_IOCTL_SCRIBER_CTRL			0x1006
#define RM_IOCTL_READ_RAW_DATA			0x1007
#define RM_IOCTL_AUTOSCAN_CTRL			0x1008
#define RM_IOCTL_NOISE_CHECK			0x1009
#define RM_IOCTL_GET_PARAMETER			0x100A
#define RM_IOCTL_SET_PARAMETER			0x100B
#define RM_IOCTL_SEND_BASELINE			0x100C
#define RM_IOCTL_SEND_ANALOG_BASELINE		0x100D
#define RM_IOCTL_SET_VARIABLE			0x1010
	#define RM_VARIABLE_SELF_TEST_RESULT		0x01
	#define RM_VARIABLE_SCRIBER_FLAG		0x02
	#define RM_VARIABLE_AUTOSCAN_FLAG		0x03
	#define RM_VARIABLE_VERSION			0x04
	#define RM_VARIABLE_REPEAT			0x06
#define RM_IOCTL_GET_VARIABLE			0x1011
	#define RM_VARIABLE_PLATFORM_ID			0x01
#define RM_IOCTL_SET_SLOWSCAN_PARA		0x1012

#define RM_INPUT_RESOLUTION_X	4096
#define RM_INPUT_RESOLUTION_Y	4096

#define RM_TS_SIGNAL		44
#define RM_TS_MAX_POINTS	16

#define RM_SIGNAL_INTR		0x00000001
#define RM_SIGNAL_SUSPEND	0x00000002
#define RM_SIGNAL_RESUME	0x00000003
#define RM_SIGNAL_CHANGE_PARA	0x00000004
	#define RM_SIGNAL_PARA_SMOOTH		0x00
	#define RM_SIGNAL_PARA_SELF_TEST	0x01


#define RM_SELF_TEST_STATUS_FINISH	0
#define RM_SELF_TEST_STATUS_TESTING	1
#define RM_SELF_TEST_RESULT_FAIL	0
#define RM_SELF_TEST_RESULT_PASS	1


#define RM_PLATFORM_K007	0x00
#define RM_PLATFORM_K107	0x01
#define RM_PLATFORM_C210	0x02
#define RM_PLATFORM_D010	0x03
#define RM_PLATFORM_P005	0x04
#define RM_PLATFORM_A110	0x10
#define RM_PLATFORM_A10L	0x11

#ifdef ENABLE_SLOW_SCAN
#define RM_SLOW_SCAN_INTERVAL	20
#define RM_SLOW_SCAN_CMD_COUNT	0x10
enum RM_SLOW_SCAN_LEVELS {
	RM_SLOW_SCAN_LEVEL_NORMAL,
	RM_SLOW_SCAN_LEVEL_20,
	RM_SLOW_SCAN_LEVEL_40,
	RM_SLOW_SCAN_LEVEL_60,
	RM_SLOW_SCAN_LEVEL_80,
	RM_SLOW_SCAN_LEVEL_100,
	RM_SLOW_SCAN_LEVEL_120,
	RM_SLOW_SCAN_LEVEL_140,
	RM_SLOW_SCAN_LEVEL_160,
	RM_SLOW_SCAN_LEVEL_180,
	RM_SLOW_SCAN_LEVEL_200,
	RM_SLOW_SCAN_LEVEL_220,
	RM_SLOW_SCAN_LEVEL_240,
	RM_SLOW_SCAN_LEVEL_260,
	RM_SLOW_SCAN_LEVEL_280,
	RM_SLOW_SCAN_LEVEL_MAX,
	RM_SLOW_SCAN_LEVEL_COUNT
};
#endif


struct rm_cmd_element {
	unsigned char addr;
	unsigned char value;
};

struct rm_cmd_list {
	unsigned char count;
	struct rm_cmd_element cmd[0];
};

struct rm_cmd_slow_scan {
	unsigned char count;
	struct rm_cmd_element cmd[RM_SLOW_SCAN_CMD_COUNT];
};

struct rm_touch_event {
	unsigned char ucTouchCount;
	unsigned char ucID[RM_TS_MAX_POINTS];
	unsigned short usX[RM_TS_MAX_POINTS];
	unsigned short usY[RM_TS_MAX_POINTS];
	unsigned short usZ[RM_TS_MAX_POINTS];
};

struct rm_spi_ts_platform_data {
	int gpio_reset;
	int x_size;
	int y_size;
	unsigned char *config;
	int platform_id;
	unsigned char *name_of_clock;
};

int rm31080_spi_byte_write(unsigned char u8Addr, unsigned char u8Value);
int rm31080_spi_byte_read(unsigned char u8Addr, unsigned char *pu8Value);
int rm31080_spi_burst_write(unsigned char *pBuf, unsigned int u32Len);
void rm31080_disable_touch(void);
void rm31080_enable_touch(void);
void rm31080_set_autoscan(unsigned char val);

#endif	/*_RM31080A_TS_H_*/
