/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _M_TTCAN_LINUX_H
#define  _M_TTCAN_LINUX_H

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/can/dev.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>

#include <asm/io.h>

#define MTTCAN_TX_INTR          (0x7 << 9)
#define MTTCAN_RX_FIFO_INTR     (0xFF)
#define MTTCAN_RX_HP_INTR       (0x1 << 8)
#define MTTCAN_TX_EV_FIFO_INTR  (0xF << 12)
#define MTTCAN_DRX_INTR         (0x1 << 19)

#define MTTCAN_ERR_INTR       (0x3FF7 << 16)
#define MTTCAN_BUS_OFF        (1 << 25)
#define MTTCAN_ERR_WARN       (1 << 24)
#define MTTCAN_ERR_PASS       (1 << 23)

#define MTT_CAN_NAPI_WEIGHT   (CONF_RX_FIFO_0_ELEMS + CONF_RX_FIFO_1_ELEMS \
				+ CONF_RX_BUFFER_ELEMS)
#define MTT_CAN_TX_OBJ_NUM    (CONF_TX_BUFFER_ELEMS + CONF_TX_FIFO_ELEMS)

#define MTTCAN_STOP_WAIT_MS   1000
#define MTTCAN_START_WAIT_MS   1000

struct mttcan_priv {
	struct can_priv can;
	struct ttcan_controller *ttcan;
	struct delayed_work can_work;
	struct napi_struct napi;
	struct net_device *dev;
	struct device *device;
	struct clk *clk;
	void __iomem *regs;
	void __iomem *mres;
	u32 irq_flags;
	u32 irq_ttflags;
	u32 tx_next;
	u32 tx_echo;
	u32 tx_object;
	u32 tx_obj_cancelled;
	void *priv;
	u32 irqstatus;
	u32 tt_irqstatus;
	u32 instance;
	int tt_intrs;
};

struct net_device *alloc_mttcan_dev(void);
void free_mttcan_can_dev(struct net_device *dev);
int register_mttcan_dev(struct net_device *dev);
void unregister_mttcan_can_dev(struct net_device *dev);
int mttcan_create_sys_files(struct device *dev);
void mttcan_delete_sys_files(struct device *dev);
#endif
