/* Copyright (C) 2012 NVIDIA Corporation.
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
 * _PWR_OFF_FORCE = _PWR_OFF is forced regardless of standby mechanisms.
 * _PWR_OFF = Device, regulators, clocks, etc is turned off.  The longest
 *            transition time to _PWR_ON is from this state.
 * _PWR_STDBY_OFF = Device is useless but powered.  No communication possible.
 *                  Device does not retain programming.  Main purpose is for
 *                  faster return to _PWR_ON without regulator delays.
 * _PWR_STDBY = Device is in standby.  Device retains programming.
 * _PWR_COMM = Device is powered enough to communicate with the device.
 * _PWR_ON = Device is at full power with active output.
 *
 * The kernel drivers treat these calls as Guaranteed Level Of Service.
 */

#define NVC_PWR_ERR			0
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

/* The GPIO mechanism uses the _gpio_type in the device's header file as a key
 * to define all the possible GPIO's the device will need.  The key is used to
 * combine the GPIO's defined in the platform board file using the
 * nvc_gpio_pdata structure with the nvc_gpio structure in the nvc kernel
 * driver.
 */
struct nvc_gpio_pdata {
	/* use a _gpio_type enum from the device's header file */
	unsigned gpio_type;
	/* the GPIO system number */
	unsigned gpio;
	/* init_en is typically set to true for all GPIO's used by the driver.
	 * However, some GPIO's are used by multiple drivers (CSI MUX, reset,
	 * etc.).  In this case, this is set true for only one of the drivers
	 * that uses the GPIO and false for the others.  If the platform board
	 * file initializes the GPIO, then this is false for all of the drivers
	 * using the GPIO.
	 */
	bool init_en;
	/* this defines the assert level for the general purpose GPIO's
	 * (_GPIO_TYPE_GPx, etc.).  The _GPIO_TYPE_GPx can be used for a GPIO
	 * that the driver doesn't know about but is needed in order for the
	 * device to work (CSI select, regulator, etc.).  The driver will
	 * blindly assert the GPIO when the device is operational and deassert
	 * when the device is turned off.
	 */
	bool active_high;
};

struct nvc_gpio_init {
	/* key to match in nvc_gpio_pdata */
	unsigned gpio_type;
	/* same as in gpio.h */
	unsigned long flags;
	/* same as in gpio.h */
	const char *label;
	/* used instead of nvc_gpio_pdata.active_high if use_flags true */
	bool active_high;
	/* false if nvc_gpio_pdata.active_high used else flags is used */
	bool use_flags;
};

struct nvc_gpio {
	unsigned gpio; /* system GPIO number */
	bool own; /* gets set if driver initializes */
	bool active_high; /* used for GP GPIOs */
	bool flag; /* scratch flag for driver implementation */
};

#endif /* __KERNEL__ */

#endif /* __NVC_H__ */

