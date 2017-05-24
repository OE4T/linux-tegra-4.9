/**
 * Copyright (c) 2015-2016, NVIDIA Corporation.  All rights reserved.
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

#ifndef __IMX204_H__
#define __IMX204_H__

#include <linux/ioctl.h>  /* For IOCTL macros */
#include <media/nvc.h>
#include <media/nvc_image.h>

#define IMX204_IOCTL_WRITE	IOW('o', 1, struct imx204_write)

#define IMX204_EEPROM_ADDRESS		0x50
#define IMX204_EEPROM_SIZE		1024
#define IMX204_EEPROM_STR_SIZE		(IMX204_EEPROM_SIZE * 2)
#define IMX204_EEPROM_BLOCK_SIZE	(1 << 8)
#define IMX204_EEPROM_NUM_BLOCKS \
	 (IMX204_EEPROM_SIZE / IMX204_EEPROM_BLOCK_SIZE)

#define IMX204_OTP_CTRL_ADDR		0x0A00
#define IMX204_OTP_STATUS_ADDR		0x0A01
#define IMX204_OTP_PAGE_NUM_ADDR	0x0A02
#define IMX204_OTP_PAGE_START_ADDR	0x0A04
#define IMX204_OTP_PAGE_END_ADDR	0x0A43
#define IMX204_OTP_NUM_PAGES		(16)
#define IMX204_OTP_PAGE_SIZE \
	 (IMX204_OTP_PAGE_END_ADDR - IMX204_OTP_PAGE_START_ADDR + 1)
#define IMX204_OTP_SIZE \
	 (IMX204_OTP_PAGE_SIZE * IMX204_OTP_NUM_PAGES)
#define IMX204_OTP_STR_SIZE (IMX204_OTP_SIZE * 2)
#define IMX204_OTP_STATUS_IN_PROGRESS		0
#define IMX204_OTP_STATUS_READ_COMPLETE	1
#define IMX204_OTP_STATUS_READ_FAIL		5

#define IMX204_FUSE_ID_OTP_ROW_ADDR	0x0A36
#define IMX204_FUSE_ID_OTP_PAGE	19 /*0x13*/
#define IMX204_FUSE_ID_SIZE		11
#define IMX204_FUSE_ID_STR_SIZE	(IMX204_FUSE_ID_SIZE * 2)

struct imx204_write {
	__u8 tx_buf[256];
	__u32 len;
};

#ifdef __KERNEL__
struct imx204_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
	struct clk *mclk;
	unsigned int reset_gpio;
	unsigned int enable_hvs_gpio;
	unsigned int enable_spi_gpio;
	unsigned int pwr_state_gpio;
	unsigned int enable_pmic_gpio;
};

struct imx204_platform_data {
	const char *mclk_name; /* NULL for default default_mclk */
	unsigned int reset_gpio;
	unsigned int enable_spi_gpio;
	unsigned int enable_hvs_gpio;
	bool ext_reg;
	int (*power_on)(struct imx204_power_rail *pw);
	int (*power_off)(struct imx204_power_rail *pw);
};

int imx204_set_reset_ops(void *drvdata,
			 int (*power)(void *drvdata, int enable));

#endif /* __KERNEL__ */

#endif  /* __IMX204_H__ */
