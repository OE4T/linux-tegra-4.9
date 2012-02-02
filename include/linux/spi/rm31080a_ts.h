#ifndef _RM31080A_TS_H_
#define _RM31080A_TS_H_

#define ENABLE_RAW_DATA_QUEUE

#define RM_IOCTL_REPORT_POINT    0x1001
#define RM_IOCTL_SET_HAL_PID     0x1002
#define RM_IOCTL_INIT_START      0x1003
#define RM_IOCTL_INIT_END        0x1004
#define RM_IOCTL_FINISH_CALC     0x1005
#define RM_IOCTL_SCRIBER_CTRL    0x1006
#define RM_IOCTL_READ_RAW_DATA   0x1007
#define RM_IOCTL_AUTOSCAN_CTRL   0x1008
#define RM_IOCTL_NOISE_CHECK     0x1009
#define RM_IOCTL_GET_PARAMETER   0x100A
#define RM_IOCTL_SET_PARAMETER   0x100B

#define RM_INPUT_RESOLUTION_X    4096
#define RM_INPUT_RESOLUTION_Y    4096

#define RM_TS_SIGNAL            44
#define RM_TS_MAX_POINTS        16

#define RM_SIGNAL_INTR          0x00000001
#define RM_SIGNAL_SUSPEND       0x00000002
#define RM_SIGNAL_RESUME        0x00000003

typedef struct {
	unsigned char ucTouchCount;
	unsigned char ucID[RM_TS_MAX_POINTS];
	unsigned short usX[RM_TS_MAX_POINTS];
	unsigned short usY[RM_TS_MAX_POINTS];
	unsigned short usZ[RM_TS_MAX_POINTS];
} rm_touch_event;


struct rm_spi_ts_platform_data{
	int gpio_reset;
	int x_size;
	int y_size;
	unsigned char* config;
};

#endif				//_RM31080A_TS_H_
