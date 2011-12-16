/* Copyright (C) 2011 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef __NVC_H__
#define __NVC_H__

#include <linux/ioctl.h>

struct nvc_param {
	int param;
	__u32 sizeofvalue;
	void *p_value;
} __packed;

#define NVC_PARAM_EXPOSURE		0
#define NVC_PARAM_GAIN			1
#define NVC_PARAM_FRAMERATE		2
#define NVC_PARAM_MAX_FRAMERATE		3
#define NVC_PARAM_INPUT_CLOCK		4
#define NVC_PARAM_LOCUS			5
#define NVC_PARAM_FLASH_CAPS		6
#define NVC_PARAM_FLASH_LEVEL		7
#define NVC_PARAM_FLASH_PIN_STATE	8
#define NVC_PARAM_TORCH_CAPS		9
#define NVC_PARAM_TORCH_LEVEL		10
#define NVC_PARAM_FOCAL_LEN		11
#define NVC_PARAM_MAX_APERTURE		12
#define NVC_PARAM_FNUMBER		13
#define NVC_PARAM_EXPOSURE_LIMITS	14
#define NVC_PARAM_GAIN_LIMITS		15
#define NVC_PARAM_FRAMERATE_LIMITS	16
#define NVC_PARAM_FRAME_RATES		17
#define NVC_PARAM_EXP_LATCH_TIME	19
#define NVC_PARAM_REGION_USED		20
#define NVC_PARAM_SELF_TEST		23
#define NVC_PARAM_STS			24
#define NVC_PARAM_TESTMODE		25
#define NVC_PARAM_EXPECTED_VALUES	26
#define NVC_PARAM_RESET			27
#define NVC_PARAM_OPTIMIZE_RES		28
#define NVC_PARAM_LINES_PER_SEC		30
#define NVC_PARAM_CAPS			31
#define NVC_PARAM_STEREO_CAP		33
#define NVC_PARAM_FOCUS_STEREO		34
#define NVC_PARAM_STEREO		35
#define NVC_PARAM_INHERENT_GAIN		36
#define NVC_PARAM_VIEW_ANGLE_H		37
#define NVC_PARAM_VIEW_ANGLE_V		38
#define NVC_PARAM_DEV_ID		46
#define NVC_PARAM_TEST_PATTERN		0x10000002
#define NVC_PARAM_SENSOR_TYPE		0x10000006
#define NVC_PARAM_I2C			1001

/* sync off */
#define NVC_SYNC_OFF			0
/* use only this device (the one receiving the call) */
#define NVC_SYNC_MASTER			1
/* use only the synced device (the "other" device) */
#define NVC_SYNC_SLAVE			2
/* use both synced devices at the same time */
#define NVC_SYNC_STEREO			3

#define NVC_RESET_HARD			0
#define NVC_RESET_SOFT			1

#define NVC_IOCTL_PWR_WR		_IOW('o', 102, int)
#define NVC_IOCTL_PWR_RD		_IOW('o', 103, int)
#define NVC_IOCTL_PARAM_WR		_IOW('o', 104, struct nvc_param)
#define NVC_IOCTL_PARAM_RD		_IOWR('o', 105, struct nvc_param)


#ifdef __KERNEL__

#include <linux/regulator/consumer.h>

/* The NVC_CFG_ defines are for the .cfg entry in the
 * platform data structure.
 */
/* Device not registered if not found */
#define NVC_CFG_NODEV			(1 << 0)
/* Don't return errors */
#define NVC_CFG_NOERR			(1 << 1)
/* Always go to _PWR_STDBY instead of _PWR_OFF */
#define NVC_CFG_OFF2STDBY		(1 << 2)
/* Init device at sys boot */
#define NVC_CFG_BOOT_INIT		(1 << 3)
/* Sync mode uses an I2C MUX to send at same time */
#define NVC_CFG_SYNC_I2C_MUX		(1 << 4)

/* Expected higher level power calls are:
 * 1 = OFF
 * 2 = STANDBY
 * 3 = ON
 * These will be multiplied by 2 before given to the driver's PM code that
 * uses the _PWR_ defines. This allows us to insert defines to give more power
 * granularity and still remain linear with regards to the power usage and
 * full power state transition latency for easy implementation of PM
 * algorithms.
 * The PM actions:
 * _PWR_ERR = Non-valid state.
 * _PWR_OFF_DELAYED = _PWR_OFF is called after a period of time.
 * _PWR_OFF = Device, regulators, clocks, etc is turned off.  The longest
 *            transition time to _PWR_ON is from this state.
 * _PWR_STDBY_OFF = Device is useless but powered.  No communication possible.
 *                  Device does not retain programming.  Main purpose is for
 *                  faster return to _PWR_ON without regulator delays.
 * _PWR_STDBY = Device is in standby.  Device retains programming.
 * _PWR_COMM = Device is powered enough to communicate with the device.
 * _PWR_ON = Device is at full power with active output.
 *
 * The kernel drivers treat these calls as guaranteed level of service.
 */

#define NVC_PWR_ERR			0
#define NVC_PWR_OFF_DELAYED		1 /* obsolete - never used */
#define NVC_PWR_OFF_FORCE		1
#define NVC_PWR_OFF			2
#define NVC_PWR_STDBY_OFF		3
#define NVC_PWR_STDBY			4
#define NVC_PWR_COMM			5
#define NVC_PWR_ON			6

struct nvc_regulator {
	bool vreg_flag;
	struct regulator *vreg;
	const char *vreg_name;
};

#endif /* __KERNEL__ */

#endif /* __NVC_H__ */

