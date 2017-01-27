/*
 * drivers/i2c/busses/i2c-tegra.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Colin Cross <ccross@android.com>
 *
 * Copyright (C) 2010-2017 NVIDIA Corporation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/reset.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/iopoll.h>
#include <linux/of_gpio.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-gpio.h>
#include <linux/tegra_prod.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>

#include <asm/unaligned.h>

#define TEGRA_I2C_TIMEOUT (msecs_to_jiffies(1000))
#define BYTES_PER_FIFO_WORD 4

#define I2C_CNFG				0x000
#define I2C_CNFG_DEBOUNCE_CNT_SHIFT		12
#define I2C_CNFG_PACKET_MODE_EN			BIT(10)
#define I2C_CNFG_NEW_MASTER_FSM			BIT(11)
#define I2C_CNFG_MULTI_MASTER_MODE		BIT(17)
#define I2C_CMD_ADDR0                           0x004
#define I2C_CMD_ADDR1                           0x008
#define I2C_CMD_DATA1                           0x00C
#define I2C_CMD_DATA2                           0x010
#define I2C_STATUS				0x01C
#define I2C_SL_CNFG				0x020
#define I2C_SL_CNFG_NACK			BIT(1)
#define I2C_SL_CNFG_NEWSL			BIT(2)
#define I2C_SL_ADDR1				0x02c
#define I2C_SL_ADDR2				0x030
#define I2C_TLOW_SEXT                           0x034
#define I2C_TX_FIFO				0x050
#define I2C_RX_FIFO				0x054
#define I2C_PACKET_TRANSFER_STATUS		0x058
#define I2C_FIFO_CTRL				0x05c
#define I2C_FIFO_CTRL_TX_FLUSH			BIT(1)
#define I2C_FIFO_CTRL_RX_FLUSH			BIT(0)
#define I2C_FIFO_CTRL_RX_TRIG_1			0
#define I2C_FIFO_CTRL_RX_TRIG_4			3
#define I2C_FIFO_CTRL_RX_TRIG_8			7
#define I2C_FIFO_CTRL_TX_TRIG_1			0
#define I2C_FIFO_CTRL_TX_TRIG_4			3
#define I2C_FIFO_CTRL_TX_TRIG_8			7
#define I2C_FIFO_CTRL_TX_TRIG_SHIFT		5
#define I2C_FIFO_CTRL_RX_TRIG_SHIFT		2
#define I2C_FIFO_STATUS				0x060
#define I2C_FIFO_STATUS_TX_MASK			0xF0
#define I2C_FIFO_STATUS_TX_SHIFT		4
#define I2C_FIFO_STATUS_RX_MASK			0x0F
#define I2C_FIFO_STATUS_RX_SHIFT		0
#define I2C_INT_MASK				0x064
#define I2C_INT_STATUS				0x068
#define I2C_INT_BUS_CLR_DONE			BIT(11)
#define I2C_INT_PACKET_XFER_COMPLETE		BIT(7)
#define I2C_INT_ALL_PACKETS_XFER_COMPLETE	BIT(6)
#define I2C_INT_TX_FIFO_OVERFLOW		BIT(5)
#define I2C_INT_RX_FIFO_UNDERFLOW		BIT(4)
#define I2C_INT_NO_ACK				BIT(3)
#define I2C_INT_ARBITRATION_LOST		BIT(2)
#define I2C_INT_TX_FIFO_DATA_REQ		BIT(1)
#define I2C_INT_RX_FIFO_DATA_REQ		BIT(0)
#define I2C_CLK_DIVISOR				0x06c
#define I2C_CLK_DIVISOR_STD_FAST_MODE_SHIFT	16
#define I2C_CLK_MULTIPLIER_STD_FAST_MODE	8
#define I2C_CLK_DIVISOR_HS_MODE_MASK		0xFFFF

#define I2C_INTERRUPT_SET_REGISTER              0x074

#define DVC_CTRL_REG1				0x000
#define DVC_CTRL_REG1_INTR_EN			BIT(10)
#define DVC_CTRL_REG2				0x004
#define DVC_CTRL_REG3				0x008
#define DVC_CTRL_REG3_SW_PROG			BIT(26)
#define DVC_CTRL_REG3_I2C_DONE_INTR_EN		BIT(30)
#define DVC_STATUS				0x00c
#define DVC_STATUS_I2C_DONE_INTR		BIT(30)

#define I2C_ERR_NONE				0x00
#define I2C_ERR_NO_ACK				0x01
#define I2C_ERR_ARBITRATION_LOST		0x02
#define I2C_ERR_UNKNOWN_INTERRUPT		0x04
#define I2C_ERR_UNEXPECTED_STATUS		0x08

#define PACKET_HEADER0_HEADER_SIZE_SHIFT	28
#define PACKET_HEADER0_PACKET_ID_SHIFT		16
#define PACKET_HEADER0_CONT_ID_SHIFT		12
#define PACKET_HEADER0_PROTOCOL_I2C		BIT(4)
#define PACKET_HEADER0_CONT_ID_MASK		0xF

#define I2C_HEADER_HIGHSPEED_MODE		BIT(22)
#define I2C_HEADER_CONT_ON_NAK			BIT(21)
#define I2C_HEADER_SEND_START_BYTE		BIT(20)
#define I2C_HEADER_READ				BIT(19)
#define I2C_HEADER_10BIT_ADDR			BIT(18)
#define I2C_HEADER_IE_ENABLE			BIT(17)
#define I2C_HEADER_REPEAT_START			BIT(16)
#define I2C_HEADER_CONTINUE_XFER		BIT(15)
#define I2C_HEADER_MASTER_ADDR_SHIFT		12
#define I2C_HEADER_SLAVE_ADDR_SHIFT		1

#define I2C_BUS_CLEAR_CNFG			0x084
#define I2C_BC_SCLK_THRESHOLD			9
#define I2C_BC_SCLK_THRESHOLD_SHIFT		16
#define I2C_BC_STOP_COND			BIT(2)
#define I2C_BC_TERMINATE			BIT(1)
#define I2C_BC_ENABLE				BIT(0)

#define I2C_BUS_CLEAR_STATUS			0x088
#define I2C_BC_STATUS				BIT(0)

#define I2C_CONFIG_LOAD				0x08C
#define I2C_MSTR_CONFIG_LOAD			BIT(0)
#define I2C_SLV_CONFIG_LOAD			BIT(1)
#define I2C_TIMEOUT_CONFIG_LOAD			BIT(2)

#define I2C_CLKEN_OVERRIDE			0x090
#define I2C_MST_CORE_CLKEN_OVR			BIT(0)

#define I2C_CONFIG_LOAD_TIMEOUT			1000000

#define I2C_INTERFACE_TIMING_0                  0x94
#define I2C_TLOW_MASK                           0x3F
#define I2C_THIGH_SHIFT                         8
#define I2C_THIGH_MASK                          (0x3F << I2C_THIGH_SHIFT)

#define I2C_DEBUG_CONTROL                       0x0A4
#define I2C_MASTER_RESET_CONTROL		0x0A8

#define I2C_MAX_TRANSFER_LEN			4096
#define I2C_MAX_TRANSACTION_NUMBER		8
#define I2C_TOTAL_BUFFER_LEN			(I2C_MAX_TRANSFER_LEN * \
						I2C_MAX_TRANSACTION_NUMBER)
#define I2C_CONFIG_LOAD_TIMEOUT			1000000

/* Define speed modes */
#define I2C_STANDARD_MODE			100000
#define I2C_FAST_MODE				400000
#define I2C_FAST_MODE_PLUS			1000000
#define I2C_HS_MODE				3500000

/* Max DMA buffer size = max packet transfer length + size of pkt header */
#define I2C_DMA_MAX_BUF_LEN			(I2C_MAX_TRANSFER_LEN + 12)
#define DATA_DMA_DIR_TX				(1 << 0)
#define DATA_DMA_DIR_RX				(1 << 1)

/* Upto I2C_PIO_MODE_MAX_LEN bytes, controller will use PIO mode,
 * above this, controller will use DMA to fill FIFO.
 * Here MAX PIO len is 20 bytes excluding packet header
 */
#define I2C_PIO_MODE_MAX_LEN			(32)

/* Packet header size in words */
#define I2C_PACKET_HEADER_SIZE			(3)

/*
 * msg_end_type: The bus control which need to be send at end of transfer.
 * @MSG_END_STOP: Send stop pulse at end of transfer.
 * @MSG_END_REPEAT_START: Send repeat start at end of transfer.
 * @MSG_END_CONTINUE: The following on message is coming and so do not send
 *		stop or repeat start.
 */
enum msg_end_type {
	MSG_END_STOP,
	MSG_END_REPEAT_START,
	MSG_END_CONTINUE,
};

/**
 * struct tegra_i2c_hw_feature : Different HW support on Tegra
 * @has_continue_xfer_support: Continue transfer supports.
 * @has_per_pkt_xfer_complete_irq: Has enable/disable capability for transfer
 *		complete interrupt per packet basis.
 * @has_single_clk_source: The i2c controller has single clock source. Tegra30
 *		and earlier Socs has two clock sources i.e. div-clk and
 *		fast-clk.
 * @has_config_load_reg: Has the config load register to load the new
 *		configuration.
 * @clk_divisor_hs_mode: Clock divisor in HS mode.
 * @clk_divisor_std_fast_mode: Clock divisor in standard/fast mode. It is
 *		applicable if there is no fast clock source i.e. single clock
 *		source.
 */

struct tegra_i2c_hw_feature {
	bool has_continue_xfer_support;
	bool has_per_pkt_xfer_complete_irq;
	bool has_single_clk_source;
	bool has_config_load_reg;
	int clk_divisor_hs_mode;
	int clk_multiplier_hs_mode;
	int clk_divisor_std_fast_mode;
	u16 clk_divisor_fast_plus_mode;
	bool has_multi_master_mode;
	bool has_slcg_override_reg;
	bool has_sw_reset_reg;
	bool has_bus_clr_support;
	bool has_reg_write_buffering;
	bool has_slcg_support;
};

/**
 * struct tegra_i2c_dev	- per device i2c context
 * @dev: device reference for power management
 * @hw: Tegra i2c hw feature.
 * @adapter: core i2c layer adapter information
 * @div_clk: clock reference for div clock of i2c controller.
 * @fast_clk: clock reference for fast clock of i2c controller.
 * @base: ioremapped registers cookie
 * @cont_id: i2c controller id, used for for packet header
 * @irq: irq number of transfer complete interrupt
 * @is_dvc: identifies the DVC i2c controller, has a different register layout
 * @msg_complete: transfer completion notifier
 * @msg_err: error code for completed message
 * @msg_buf: pointer to current message data
 * @msg_buf_remaining: size of unsent data in the message buffer
 * @msg_read: identifies read transfers
 * @bus_clk_rate: current i2c bus clock rate
 * @is_suspended: prevents i2c controller accesses after suspend is called
 */
struct tegra_i2c_dev {
	struct device *dev;
	const struct tegra_i2c_hw_feature *hw;
	struct i2c_adapter adapter;
	struct clk *div_clk;
	struct clk *fast_clk;
	struct reset_control *rst;
	void __iomem *base;
	phys_addr_t phys_addr;
	int cont_id;
	int irq;
	bool irq_disabled;
	int is_dvc;
	struct completion msg_complete;
	int msg_add;
	int msg_err;
	u8 *msg_buf;
	size_t msg_buf_remaining;
	int msg_read;
	u32 bus_clk_rate;
	u16 clk_divisor_non_hs_mode;
	bool is_suspended;
	bool is_multimaster_mode;
	raw_spinlock_t xfer_lock;
	bool is_periph_reset_done;
	int scl_gpio;
	int sda_gpio;
	struct i2c_algo_bit_data bit_data;
	const struct i2c_algorithm *bit_algo;
	bool bit_banging_xfer_after_shutdown;
	bool is_shutdown;
	u32 low_clock_count;
	u32 high_clock_count;
	struct tegra_prod *prod_list;
	int clk_divisor_hs_mode;
	u16 hs_master_code;
	struct dma_async_tx_descriptor *rx_dma_desc;
	struct dma_chan *rx_dma_chan;
	u32 *rx_dma_buf;
	dma_addr_t rx_dma_phys;
	struct dma_async_tx_descriptor *tx_dma_desc;
	struct dma_chan *tx_dma_chan;
	u32 *tx_dma_buf;
	dma_addr_t tx_dma_phys;
	unsigned int dma_buf_size;
	bool is_curr_dma_xfer;
	struct completion rx_dma_complete;
	struct completion tx_dma_complete;
	int curr_direction;
	int rx_dma_len;
	dma_cookie_t rx_cookie;
	bool disable_dma_mode;
	bool is_clkon_always;
	u8 *msg_tx_buf;
	size_t msg_tx_remaining;
	u8 *msg_rx_buf;
	size_t msg_rx_remaining;
	bool has_rx;
	u8 *pio_buffer;
	int msg_num;
	struct i2c_msg *msgs;
	bool use_single_xfer_complete;
	bool disable_multi_pkt_mode;
};

static void dvc_writel(struct tegra_i2c_dev *i2c_dev, u32 val, unsigned long reg)
{
	writel(val, i2c_dev->base + reg);
}

static u32 dvc_readl(struct tegra_i2c_dev *i2c_dev, unsigned long reg)
{
	return readl(i2c_dev->base + reg);
}

/*
 * i2c_writel and i2c_readl will offset the register if necessary to talk
 * to the I2C block inside the DVC block
 */
static unsigned long tegra_i2c_reg_addr(struct tegra_i2c_dev *i2c_dev,
	unsigned long reg)
{
	if (i2c_dev->is_dvc)
		reg += (reg >= I2C_TX_FIFO) ? 0x10 : 0x40;
	return reg;
}

static void i2c_writel(struct tegra_i2c_dev *i2c_dev, u32 val,
	unsigned long reg)
{
	writel(val, i2c_dev->base + tegra_i2c_reg_addr(i2c_dev, reg));

	/* Read back register to make sure that register writes completed */
	if (i2c_dev->hw->has_reg_write_buffering) {
		if (reg != I2C_TX_FIFO)
			readl(i2c_dev->base + tegra_i2c_reg_addr(i2c_dev, reg));
	}
}

static u32 i2c_readl(struct tegra_i2c_dev *i2c_dev, unsigned long reg)
{
	return readl(i2c_dev->base + tegra_i2c_reg_addr(i2c_dev, reg));
}

static inline void tegra_i2c_gpio_setscl(void *data, int state)
{
	struct tegra_i2c_dev *i2c_dev = data;

	gpio_set_value(i2c_dev->scl_gpio, state);
}

static inline int tegra_i2c_gpio_getscl(void *data)
{
	struct tegra_i2c_dev *i2c_dev = data;

	return gpio_get_value(i2c_dev->scl_gpio);
}

static inline void tegra_i2c_gpio_setsda(void *data, int state)
{
	struct tegra_i2c_dev *i2c_dev = data;

	gpio_set_value(i2c_dev->sda_gpio, state);
}

static inline int tegra_i2c_gpio_getsda(void *data)
{
	struct tegra_i2c_dev *i2c_dev = data;

	return gpio_get_value(i2c_dev->sda_gpio);
}

static int tegra_i2c_gpio_request(struct tegra_i2c_dev *i2c_dev)
{
	int ret;

	ret = gpio_request_one(i2c_dev->scl_gpio,
				GPIOF_OUT_INIT_HIGH | GPIOF_OPEN_DRAIN,
				"i2c-gpio-scl");
	if (ret < 0) {
		dev_err(i2c_dev->dev, "GPIO request for gpio %d failed %d\n",
				i2c_dev->scl_gpio, ret);
		return ret;
	}

	ret = gpio_request_one(i2c_dev->sda_gpio,
				GPIOF_OUT_INIT_HIGH | GPIOF_OPEN_DRAIN,
				"i2c-gpio-sda");
	if (ret < 0) {
		dev_err(i2c_dev->dev, "GPIO request for gpio %d failed %d\n",
				i2c_dev->sda_gpio, ret);
		gpio_free(i2c_dev->scl_gpio);
		return ret;
	}
	return ret;
}

static void tegra_i2c_gpio_free(struct tegra_i2c_dev *i2c_dev)
{
	gpio_free(i2c_dev->scl_gpio);
	gpio_free(i2c_dev->sda_gpio);
}

static int tegra_i2c_gpio_xfer(struct i2c_adapter *adap,
	struct i2c_msg msgs[], int num)
{
	struct tegra_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	int ret;

	ret = tegra_i2c_gpio_request(i2c_dev);
	if (ret < 0)
		return ret;

	ret = i2c_dev->bit_algo->master_xfer(adap, msgs, num);
	if (ret < 0)
		dev_err(i2c_dev->dev, "i2c-bit-algo xfer failed %d\n", ret);

	tegra_i2c_gpio_free(i2c_dev);
	return ret;
}

static int tegra_i2c_gpio_init(struct tegra_i2c_dev *i2c_dev)
{
	struct i2c_algo_bit_data *bit_data = &i2c_dev->bit_data;

	bit_data->setsda = tegra_i2c_gpio_setsda;
	bit_data->getsda = tegra_i2c_gpio_getsda;
	bit_data->setscl = tegra_i2c_gpio_setscl;
	bit_data->getscl = tegra_i2c_gpio_getscl;
	bit_data->data = i2c_dev;
	bit_data->udelay = 5; /* 100KHz */
	bit_data->timeout = HZ; /* 10 ms*/
	i2c_dev->bit_algo = &i2c_bit_algo;
	i2c_dev->adapter.algo_data = bit_data;
	return 0;
}

static void tegra_i2c_rx_dma_complete(void *args)
{
	struct tegra_i2c_dev *i2c_dev = args;

	complete(&i2c_dev->rx_dma_complete);
}

static void tegra_i2c_tx_dma_complete(void *args)
{
	struct tegra_i2c_dev *i2c_dev = args;

	complete(&i2c_dev->tx_dma_complete);
}

static int tegra_i2c_start_tx_dma(struct tegra_i2c_dev *i2c_dev, int len)
{
	reinit_completion(&i2c_dev->tx_dma_complete);
	dev_dbg(i2c_dev->dev, "Starting tx dma for len:%d\n", len);
	i2c_dev->tx_dma_desc = dmaengine_prep_slave_single(i2c_dev->tx_dma_chan,
				i2c_dev->tx_dma_phys, len, DMA_MEM_TO_DEV,
				DMA_PREP_INTERRUPT |  DMA_CTRL_ACK);
	if (!i2c_dev->tx_dma_desc) {
		dev_err(i2c_dev->dev, "Not able to get desc for Tx\n");
		return -EIO;
	}

	i2c_dev->tx_dma_desc->callback = tegra_i2c_tx_dma_complete;
	i2c_dev->tx_dma_desc->callback_param = i2c_dev;

	dmaengine_submit(i2c_dev->tx_dma_desc);
	dma_async_issue_pending(i2c_dev->tx_dma_chan);
	return 0;
}

static int tegra_i2c_start_rx_dma(struct tegra_i2c_dev *i2c_dev, int len)
{
	reinit_completion(&i2c_dev->rx_dma_complete);

	dev_dbg(i2c_dev->dev, "Starting rx dma for len:%d\n", len);
	i2c_dev->rx_dma_desc = dmaengine_prep_slave_single(i2c_dev->rx_dma_chan,
				i2c_dev->rx_dma_phys, len, DMA_DEV_TO_MEM,
				DMA_PREP_INTERRUPT |  DMA_CTRL_ACK);
	if (!i2c_dev->rx_dma_desc) {
		dev_err(i2c_dev->dev, "Not able to get desc for Rx\n");
		return -EIO;
	}

	i2c_dev->rx_dma_desc->callback = tegra_i2c_rx_dma_complete;
	i2c_dev->rx_dma_desc->callback_param = i2c_dev;

	dmaengine_submit(i2c_dev->rx_dma_desc);
	dma_async_issue_pending(i2c_dev->rx_dma_chan);
	return 0;
}


static int tegra_i2c_init_dma_param(struct tegra_i2c_dev *i2c_dev,
							bool dma_to_memory)
{
	struct dma_chan *dma_chan;
	u32 *dma_buf;
	dma_addr_t dma_phys;
	int ret;
	struct dma_slave_config dma_sconfig;

	dma_chan = dma_request_slave_channel_reason(i2c_dev->dev,
						dma_to_memory ? "rx" : "tx");
	if (IS_ERR(dma_chan)) {
		ret = PTR_ERR(dma_chan);
		if (ret != -EPROBE_DEFER)
			dev_err(i2c_dev->dev,
				"Dma channel is not available: %d\n", ret);
		return ret;
	}

	dma_buf = dma_alloc_coherent(i2c_dev->dev, i2c_dev->dma_buf_size,
							&dma_phys, GFP_KERNEL);
	if (!dma_buf) {
		dev_err(i2c_dev->dev, "Not able to allocate the dma buffer\n");
		dma_release_channel(dma_chan);
		return -ENOMEM;
	}

	if (dma_to_memory) {
		dma_sconfig.src_addr = i2c_dev->phys_addr + I2C_RX_FIFO;
		dma_sconfig.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		dma_sconfig.src_maxburst = 0;
	} else {
		dma_sconfig.dst_addr = i2c_dev->phys_addr + I2C_TX_FIFO;
		dma_sconfig.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		dma_sconfig.dst_maxburst = 0;
	}

	ret = dmaengine_slave_config(dma_chan, &dma_sconfig);
	if (ret)
		goto scrub;
	if (dma_to_memory) {
		i2c_dev->rx_dma_chan = dma_chan;
		i2c_dev->rx_dma_buf = dma_buf;
		i2c_dev->rx_dma_phys = dma_phys;
	} else {
		i2c_dev->tx_dma_chan = dma_chan;
		i2c_dev->tx_dma_buf = dma_buf;
		i2c_dev->tx_dma_phys = dma_phys;
	}
	return 0;

scrub:
	dma_free_coherent(i2c_dev->dev, i2c_dev->dma_buf_size,
							dma_buf, dma_phys);
	dma_release_channel(dma_chan);
	return ret;
}

static void tegra_i2c_deinit_dma_param(struct tegra_i2c_dev *i2c_dev,
	bool dma_to_memory)
{
	u32 *dma_buf;
	dma_addr_t dma_phys;
	struct dma_chan *dma_chan;

	if (dma_to_memory) {
		dma_buf = i2c_dev->rx_dma_buf;
		dma_chan = i2c_dev->rx_dma_chan;
		dma_phys = i2c_dev->rx_dma_phys;
		i2c_dev->rx_dma_chan = NULL;
		i2c_dev->rx_dma_buf = NULL;
	} else {
		dma_buf = i2c_dev->tx_dma_buf;
		dma_chan = i2c_dev->tx_dma_chan;
		dma_phys = i2c_dev->tx_dma_phys;
		i2c_dev->tx_dma_buf = NULL;
		i2c_dev->tx_dma_chan = NULL;
	}
	if (!dma_chan)
		return;

	dma_free_coherent(i2c_dev->dev, i2c_dev->dma_buf_size,
			  dma_buf, dma_phys);
	dma_release_channel(dma_chan);
}

static void tegra_i2c_mask_irq(struct tegra_i2c_dev *i2c_dev, u32 mask)
{
	u32 int_mask;

	int_mask = i2c_readl(i2c_dev, I2C_INT_MASK) & ~mask;
	i2c_writel(i2c_dev, int_mask, I2C_INT_MASK);
}

static void tegra_i2c_unmask_irq(struct tegra_i2c_dev *i2c_dev, u32 mask)
{
	u32 int_mask;

	int_mask = i2c_readl(i2c_dev, I2C_INT_MASK) | mask;
	i2c_writel(i2c_dev, int_mask, I2C_INT_MASK);
}

static int tegra_i2c_flush_fifos(struct tegra_i2c_dev *i2c_dev)
{
	unsigned long timeout = jiffies + HZ;
	u32 val = i2c_readl(i2c_dev, I2C_FIFO_CTRL);

	val |= I2C_FIFO_CTRL_TX_FLUSH | I2C_FIFO_CTRL_RX_FLUSH;
	i2c_writel(i2c_dev, val, I2C_FIFO_CTRL);

	while (i2c_readl(i2c_dev, I2C_FIFO_CTRL) &
		(I2C_FIFO_CTRL_TX_FLUSH | I2C_FIFO_CTRL_RX_FLUSH)) {
		if (time_after(jiffies, timeout)) {
			dev_warn(i2c_dev->dev, "timeout waiting for fifo flush\n");
			return -ETIMEDOUT;
		}
		msleep(1);
	}
	return 0;
}

static int tegra_i2c_empty_rx_fifo(struct tegra_i2c_dev *i2c_dev)
{
	u32 val;
	int rx_fifo_avail;
	u8 *buf = i2c_dev->msg_rx_buf;
	size_t buf_remaining = i2c_dev->msg_rx_remaining;
	int words_to_transfer;
	u32 temp_fifo[8]; /* temporary word aligned buffer for reading fifo */

	if (!i2c_dev->msg_rx_remaining)
		return 0;

	val = i2c_readl(i2c_dev, I2C_FIFO_STATUS);
	rx_fifo_avail = (val & I2C_FIFO_STATUS_RX_MASK) >>
		I2C_FIFO_STATUS_RX_SHIFT;

	/* Rounds down to not include partial word at the end of buf */
	words_to_transfer = buf_remaining / BYTES_PER_FIFO_WORD;
	if (words_to_transfer > rx_fifo_avail)
		words_to_transfer = rx_fifo_avail;

	for (val = 0; val < words_to_transfer; val++)
		temp_fifo[val] = i2c_readl(i2c_dev, I2C_RX_FIFO);

	memcpy(buf, temp_fifo, words_to_transfer * BYTES_PER_FIFO_WORD);
	buf += words_to_transfer * BYTES_PER_FIFO_WORD;
	buf_remaining -= words_to_transfer * BYTES_PER_FIFO_WORD;
	rx_fifo_avail -= words_to_transfer;

	/*
	 * If there is a partial word at the end of buf, handle it manually to
	 * prevent overwriting past the end of buf
	 */
	if (rx_fifo_avail > 0 && buf_remaining > 0) {
		WARN_ON(buf_remaining > 3);
		val = i2c_readl(i2c_dev, I2C_RX_FIFO);
		val = cpu_to_le32(val);
		memcpy(buf, &val, buf_remaining);
		buf_remaining = 0;
		rx_fifo_avail--;
	}

	BUG_ON(rx_fifo_avail > 0 && buf_remaining > 0);
	i2c_dev->msg_rx_remaining = buf_remaining;
	i2c_dev->msg_rx_buf = buf;

	/*
	 * All bytes received, unmask RX_FIFO_DATA_REQ to prevent more
	 * interrupts from FIFO
	 */
	if (!i2c_dev->msg_rx_remaining)
		tegra_i2c_mask_irq(i2c_dev, I2C_INT_RX_FIFO_DATA_REQ);

	return 0;
}

static int tegra_i2c_fill_tx_fifo(struct tegra_i2c_dev *i2c_dev)
{
	u32 val;
	int tx_fifo_avail, words_to_transfer;
	u8 *buffer = i2c_dev->msg_tx_buf;
	u32 *buffer_u32 = (u32 *)buffer;
	size_t buf_remaining;

	WARN_ON(!IS_ALIGNED((unsigned long)buffer, BYTES_PER_FIFO_WORD));

	if (!i2c_dev->msg_tx_remaining)
		return 0;

	buf_remaining = i2c_dev->msg_tx_remaining;

	val = i2c_readl(i2c_dev, I2C_FIFO_STATUS);
	tx_fifo_avail = (val & I2C_FIFO_STATUS_TX_MASK) >>
		I2C_FIFO_STATUS_TX_SHIFT;

	/* Rounds down to not include partial word at the end of buf */
	words_to_transfer = buf_remaining / BYTES_PER_FIFO_WORD;

	if (words_to_transfer > tx_fifo_avail)
		words_to_transfer = tx_fifo_avail;

	for (val = 0; val < words_to_transfer; val++)
		i2c_writel(i2c_dev, buffer_u32[val], I2C_TX_FIFO);

	buffer += words_to_transfer * BYTES_PER_FIFO_WORD;
	buf_remaining -= words_to_transfer * BYTES_PER_FIFO_WORD;
	tx_fifo_avail -= words_to_transfer;

	/*
	 * If there is a partial word at the end of buf, handle it manually to
	 * prevent reading past the end of buf, which could cross a page
	 * boundary and fault.
	 */
	if (tx_fifo_avail > 0 && buf_remaining > 0) {
		memcpy(&val, buffer, buf_remaining);
		val = le32_to_cpu(val);
		buf_remaining = 0;
		tx_fifo_avail--;
		barrier();

		i2c_writel(i2c_dev, val, I2C_TX_FIFO);
	}

	WARN_ON(tx_fifo_avail > 0 && buf_remaining > 0);
	i2c_dev->msg_tx_remaining = buf_remaining;
	i2c_dev->msg_tx_buf = buffer;

	/*
	 * If tx_fifo_avail is more than bytes to be written then all
	 * bytes are received, hence mask TX_FIFO_DATA_REQ to prevent more
	 * interrupts from FIFO
	 */
	if (!i2c_dev->msg_tx_remaining)
		tegra_i2c_mask_irq(i2c_dev, I2C_INT_TX_FIFO_DATA_REQ);

	return 0;
}

/*
 * One of the Tegra I2C blocks is inside the DVC (Digital Voltage Controller)
 * block.  This block is identical to the rest of the I2C blocks, except that
 * it only supports master mode, it has registers moved around, and it needs
 * some extra init to get it into I2C mode.  The register moves are handled
 * by i2c_readl and i2c_writel
 */
static void tegra_dvc_init(struct tegra_i2c_dev *i2c_dev)
{
	u32 val;

	val = dvc_readl(i2c_dev, DVC_CTRL_REG3);
	val |= DVC_CTRL_REG3_SW_PROG;
	val |= DVC_CTRL_REG3_I2C_DONE_INTR_EN;
	dvc_writel(i2c_dev, val, DVC_CTRL_REG3);

	val = dvc_readl(i2c_dev, DVC_CTRL_REG1);
	val |= DVC_CTRL_REG1_INTR_EN;
	dvc_writel(i2c_dev, val, DVC_CTRL_REG1);
}

static int tegra_i2c_runtime_resume(struct device *dev)
{
	struct tegra_i2c_dev *i2c_dev = dev_get_drvdata(dev);
	int ret;

	ret = pinctrl_pm_select_default_state(i2c_dev->dev);
	if (ret)
		return ret;

	if (!i2c_dev->hw->has_single_clk_source) {
		ret = clk_enable(i2c_dev->fast_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Enabling fast clk failed, err %d\n", ret);
			return ret;
		}
	}

	ret = clk_enable(i2c_dev->div_clk);
	if (ret < 0) {
		dev_err(i2c_dev->dev,
			"Enabling div clk failed, err %d\n", ret);
		clk_disable(i2c_dev->fast_clk);
		return ret;
	}

	return 0;
}

static int tegra_i2c_runtime_suspend(struct device *dev)
{
	struct tegra_i2c_dev *i2c_dev = dev_get_drvdata(dev);

	clk_disable(i2c_dev->div_clk);
	if (!i2c_dev->hw->has_single_clk_source)
		clk_disable(i2c_dev->fast_clk);

	return pinctrl_pm_select_idle_state(i2c_dev->dev);
}

static int tegra_i2c_wait_for_config_load(struct tegra_i2c_dev *i2c_dev)
{
	unsigned long reg_offset;
	void __iomem *addr;
	u32 val;
	int err;

	if (i2c_dev->hw->has_config_load_reg) {
		reg_offset = tegra_i2c_reg_addr(i2c_dev, I2C_CONFIG_LOAD);
		addr = i2c_dev->base + reg_offset;
		i2c_writel(i2c_dev, I2C_MSTR_CONFIG_LOAD, I2C_CONFIG_LOAD);
		if (in_interrupt())
			err = readl_poll_timeout_atomic(addr, val, val == 0,
					1000, I2C_CONFIG_LOAD_TIMEOUT);
		else
			err = readl_poll_timeout(addr, val, val == 0,
					1000, I2C_CONFIG_LOAD_TIMEOUT);

		if (err) {
			dev_warn(i2c_dev->dev,
				 "timeout waiting for config load\n");
			return err;
		}
	}

	return 0;
}

static int tegra_i2c_set_clk_rate(struct tegra_i2c_dev *i2c_dev)
{
	u32 clk_multiplier = I2C_CLK_MULTIPLIER_STD_FAST_MODE;
	int ret = 0;


	switch (i2c_dev->bus_clk_rate) {
	case I2C_HS_MODE:
		clk_multiplier = i2c_dev->hw->clk_multiplier_hs_mode;
		clk_multiplier *= (i2c_dev->clk_divisor_hs_mode + 1);
		break;
	case I2C_FAST_MODE_PLUS:
	case I2C_STANDARD_MODE:
	case I2C_FAST_MODE:
	default:
		clk_multiplier = (i2c_dev->low_clock_count +
				  i2c_dev->high_clock_count + 2);
		clk_multiplier *= (i2c_dev->clk_divisor_non_hs_mode + 1);
		break;
	}

	ret = clk_set_rate(i2c_dev->div_clk,
			   i2c_dev->bus_clk_rate * clk_multiplier);
	if (ret) {
		dev_err(i2c_dev->dev, "Clock rate change failed %d\n", ret);
		return ret;
	}

	return ret;
}

static void tegra_i2c_config_prod_settings(struct tegra_i2c_dev *i2c_dev)
{
	char *prod_name;
	int ret;

	switch (i2c_dev->bus_clk_rate) {
	case I2C_FAST_MODE:
		prod_name = "prod_c_fm";
		break;
	case I2C_FAST_MODE_PLUS:
		prod_name = "prod_c_fmplus";
		break;
	case I2C_HS_MODE:
		prod_name = "prod_c_hs";
		break;
	case I2C_STANDARD_MODE:
	default:
		prod_name = "prod_c_sm";
		break;
	}

	ret = tegra_prod_set_by_name(&i2c_dev->base, prod_name,
				     i2c_dev->prod_list);
	if (ret == 0)
		dev_dbg(i2c_dev->dev, "setting prod: %s\n", prod_name);
}

static void tegra_i2c_get_clk_parameters(struct tegra_i2c_dev *i2c_dev)
{
	u32 val;

	val = i2c_readl(i2c_dev, I2C_INTERFACE_TIMING_0);
	i2c_dev->low_clock_count = val & I2C_TLOW_MASK;
	i2c_dev->high_clock_count = (val & I2C_THIGH_MASK) >> I2C_THIGH_SHIFT;

	val = i2c_readl(i2c_dev, I2C_CLK_DIVISOR);
	i2c_dev->clk_divisor_hs_mode = val & I2C_CLK_DIVISOR_HS_MODE_MASK;
	i2c_dev->clk_divisor_non_hs_mode = (val >>
			I2C_CLK_DIVISOR_STD_FAST_MODE_SHIFT);
}

static int tegra_i2c_init(struct tegra_i2c_dev *i2c_dev)
{
	u32 val;
	int err;
	u32 clk_divisor;

	err = pm_runtime_get_sync(i2c_dev->dev);
	if (err < 0) {
		dev_err(i2c_dev->dev, "runtime resume failed %d\n", err);
		return err;
	}
	if (i2c_dev->hw->has_sw_reset_reg) {
		if (i2c_dev->is_periph_reset_done) {
			/* If already controller reset is done through */
			/* clock reset control register, then use SW reset */
			i2c_writel(i2c_dev, 1, I2C_MASTER_RESET_CONTROL);
			udelay(2);
			i2c_writel(i2c_dev, 0, I2C_MASTER_RESET_CONTROL);
			goto skip_periph_reset;
		}
	}
	reset_control_assert(i2c_dev->rst);
	udelay(2);
	reset_control_deassert(i2c_dev->rst);
	i2c_dev->is_periph_reset_done = true;

skip_periph_reset:
	if (i2c_dev->is_dvc)
		tegra_dvc_init(i2c_dev);

	/* configuring below register to default as per TRM*/
	i2c_writel(i2c_dev, 0, I2C_TLOW_SEXT);
	i2c_writel(i2c_dev, 0, I2C_CMD_ADDR0);
	i2c_writel(i2c_dev, 0, I2C_CMD_ADDR1);
	i2c_writel(i2c_dev, 0, I2C_CMD_DATA1);
	i2c_writel(i2c_dev, 0, I2C_CMD_DATA2);
	i2c_writel(i2c_dev, 0, I2C_DEBUG_CONTROL);
	i2c_writel(i2c_dev, 0, I2C_INTERRUPT_SET_REGISTER);
	i2c_writel(i2c_dev, 0, I2C_FIFO_CTRL);

	val = I2C_CNFG_NEW_MASTER_FSM | I2C_CNFG_PACKET_MODE_EN;
	if (i2c_dev->bus_clk_rate != I2C_HS_MODE)
		val |= (0x2 << I2C_CNFG_DEBOUNCE_CNT_SHIFT);

	if (i2c_dev->hw->has_multi_master_mode)
		val |= I2C_CNFG_MULTI_MASTER_MODE;

	i2c_writel(i2c_dev, val, I2C_CNFG);
	i2c_writel(i2c_dev, 0, I2C_INT_MASK);

	/* Make sure clock divisor programmed correctly */
	if (i2c_dev->bus_clk_rate == I2C_HS_MODE) {
		i2c_dev->clk_divisor_hs_mode = i2c_dev->hw->clk_divisor_hs_mode;
	} else {
		val = i2c_readl(i2c_dev, I2C_CLK_DIVISOR);
		i2c_dev->clk_divisor_hs_mode = val & I2C_CLK_DIVISOR_HS_MODE_MASK;
	}

	clk_divisor = i2c_dev->clk_divisor_hs_mode;
	clk_divisor |= i2c_dev->clk_divisor_non_hs_mode <<
					I2C_CLK_DIVISOR_STD_FAST_MODE_SHIFT;
	i2c_writel(i2c_dev, clk_divisor, I2C_CLK_DIVISOR);

	if (i2c_dev->prod_list)
		tegra_i2c_config_prod_settings(i2c_dev);

	tegra_i2c_get_clk_parameters(i2c_dev);

	err = tegra_i2c_set_clk_rate(i2c_dev);
	if (err < 0)
		return err;

	if (!i2c_dev->is_dvc) {
		u32 sl_cfg = i2c_readl(i2c_dev, I2C_SL_CNFG);

		sl_cfg |= I2C_SL_CNFG_NACK | I2C_SL_CNFG_NEWSL;
		i2c_writel(i2c_dev, sl_cfg, I2C_SL_CNFG);
		i2c_writel(i2c_dev, 0xfc, I2C_SL_ADDR1);
		i2c_writel(i2c_dev, 0x00, I2C_SL_ADDR2);
	}

	val = (I2C_FIFO_CTRL_TX_TRIG_8 << I2C_FIFO_CTRL_TX_TRIG_SHIFT) |
		(I2C_FIFO_CTRL_RX_TRIG_1 << I2C_FIFO_CTRL_RX_TRIG_SHIFT);
	i2c_writel(i2c_dev, val, I2C_FIFO_CTRL);

	err = tegra_i2c_flush_fifos(i2c_dev);
	if (err)
		goto err;

	if (i2c_dev->is_multimaster_mode && i2c_dev->hw->has_slcg_override_reg)
		i2c_writel(i2c_dev, I2C_MST_CORE_CLKEN_OVR, I2C_CLKEN_OVERRIDE);

	err = tegra_i2c_wait_for_config_load(i2c_dev);
	if (err)
		goto err;

	if (i2c_dev->irq_disabled) {
		i2c_dev->irq_disabled = false;
		enable_irq(i2c_dev->irq);
	}

err:
	pm_runtime_put(i2c_dev->dev);
	return err;
}

static int tegra_i2c_disable_packet_mode(struct tegra_i2c_dev *i2c_dev)
{
	u32 cnfg;

	cnfg = i2c_readl(i2c_dev, I2C_CNFG);
	if (cnfg & I2C_CNFG_PACKET_MODE_EN)
		i2c_writel(i2c_dev, cnfg & ~I2C_CNFG_PACKET_MODE_EN, I2C_CNFG);

	return tegra_i2c_wait_for_config_load(i2c_dev);
}

static irqreturn_t tegra_i2c_isr(int irq, void *dev_id)
{
	u32 status_raw, status;
	const u32 status_err = I2C_INT_NO_ACK | I2C_INT_ARBITRATION_LOST;
	struct tegra_i2c_dev *i2c_dev = dev_id;
	unsigned long flags;
	u32 mask;
	bool is_curr_dma_xfer;

	/* Ignore status bits that we are not expecting */
	status_raw = i2c_readl(i2c_dev, I2C_INT_STATUS);
	mask = i2c_readl(i2c_dev, I2C_INT_MASK);
	status = status_raw & mask;

	raw_spin_lock_irqsave(&i2c_dev->xfer_lock, flags);
	is_curr_dma_xfer = i2c_dev->is_curr_dma_xfer;

	if (status == 0) {
		dev_warn(i2c_dev->dev, "irq status 0 %08x %08x %08x\n",
			 i2c_readl(i2c_dev, I2C_PACKET_TRANSFER_STATUS),
			 i2c_readl(i2c_dev, I2C_STATUS),
			 i2c_readl(i2c_dev, I2C_CNFG));
		i2c_dev->msg_err |= I2C_ERR_UNKNOWN_INTERRUPT;

		if (!i2c_dev->irq_disabled) {
			disable_irq_nosync(i2c_dev->irq);
			i2c_dev->irq_disabled = true;
		}
		goto err;
	}

	if (unlikely(status & status_err)) {
		tegra_i2c_disable_packet_mode(i2c_dev);
		if (status & I2C_INT_NO_ACK)
			i2c_dev->msg_err |= I2C_ERR_NO_ACK;
		if (status & I2C_INT_ARBITRATION_LOST)
			i2c_dev->msg_err |= I2C_ERR_ARBITRATION_LOST;
		if (status & I2C_INT_TX_FIFO_OVERFLOW)
			i2c_dev->msg_err |= I2C_INT_TX_FIFO_OVERFLOW;
		if (status & I2C_INT_RX_FIFO_UNDERFLOW)
			i2c_dev->msg_err |= I2C_INT_RX_FIFO_UNDERFLOW;
		goto err;
	}

	if (i2c_dev->hw->has_bus_clr_support && (status & I2C_INT_BUS_CLR_DONE))
		goto err;

	if (status & I2C_INT_RX_FIFO_DATA_REQ) {
		if (i2c_dev->msg_rx_remaining)
			tegra_i2c_empty_rx_fifo(i2c_dev);
		else {
			dev_err(i2c_dev->dev, "unexpected rx data request\n");
			i2c_dev->msg_err |= I2C_ERR_UNEXPECTED_STATUS;
			goto err;
		}
	}

	if (status & I2C_INT_TX_FIFO_DATA_REQ) {
		if (i2c_dev->msg_tx_remaining)
			tegra_i2c_fill_tx_fifo(i2c_dev);
		else {
			dev_err(i2c_dev->dev, "unexpected tx data request\n");
			i2c_dev->msg_err |= I2C_ERR_UNEXPECTED_STATUS;
			goto err;
		}
	}

	i2c_writel(i2c_dev, status, I2C_INT_STATUS);
	if (i2c_dev->is_dvc)
		dvc_writel(i2c_dev, DVC_STATUS_I2C_DONE_INTR, DVC_STATUS);

	if ((status & I2C_INT_ALL_PACKETS_XFER_COMPLETE)) {
		/* msg_[rt]x_remaining has to be set back to ZERO for DMA */
		if (is_curr_dma_xfer) {
			if (i2c_dev->curr_direction == DATA_DMA_DIR_TX)
				i2c_dev->msg_tx_remaining = 0;
			else
				i2c_dev->msg_rx_remaining = 0;
		}
		WARN_ON(i2c_dev->msg_tx_remaining || i2c_dev->msg_rx_remaining);

		if ((!i2c_dev->msg_rx_remaining) &&
				(!i2c_dev->msg_tx_remaining))
			complete(&i2c_dev->msg_complete);
	} else if (((status & I2C_INT_PACKET_XFER_COMPLETE) && i2c_dev->has_rx)
			|| ((status & I2C_INT_PACKET_XFER_COMPLETE)
			&& i2c_dev->use_single_xfer_complete)) {
		/* msg_[rt]x_remaining has to be set back to ZERO for DMA */
		if (is_curr_dma_xfer) {
			if (i2c_dev->curr_direction == DATA_DMA_DIR_TX)
				i2c_dev->msg_tx_remaining = 0;
			else
				i2c_dev->msg_rx_remaining = 0;
		}
		if ((!i2c_dev->msg_rx_remaining) &&
				(!i2c_dev->msg_tx_remaining))
			complete(&i2c_dev->msg_complete);
	}
	goto done;
err:
	/* An error occurred, mask all interrupts */
	mask = I2C_INT_NO_ACK | I2C_INT_ARBITRATION_LOST |
		I2C_INT_PACKET_XFER_COMPLETE | I2C_INT_TX_FIFO_DATA_REQ |
		I2C_INT_RX_FIFO_DATA_REQ | I2C_INT_ALL_PACKETS_XFER_COMPLETE |
		I2C_INT_RX_FIFO_UNDERFLOW | I2C_INT_TX_FIFO_OVERFLOW;

	if (i2c_dev->hw->has_bus_clr_support)
		mask |= I2C_INT_BUS_CLR_DONE;

	/* An error occurred, mask all interrupts */
	tegra_i2c_mask_irq(i2c_dev, mask);

	i2c_writel(i2c_dev, status, I2C_INT_STATUS);
	if (i2c_dev->is_dvc)
		dvc_writel(i2c_dev, DVC_STATUS_I2C_DONE_INTR, DVC_STATUS);

	if (i2c_dev->is_curr_dma_xfer) {
		if (i2c_dev->curr_direction == DATA_DMA_DIR_TX) {
			dmaengine_terminate_all(i2c_dev->tx_dma_chan);
			complete(&i2c_dev->tx_dma_complete);
		} else {
			dmaengine_terminate_all(i2c_dev->rx_dma_chan);
			complete(&i2c_dev->rx_dma_complete);
		}
	}

	complete(&i2c_dev->msg_complete);
done:
	raw_spin_unlock_irqrestore(&i2c_dev->xfer_lock, flags);
	return IRQ_HANDLED;
}

static int tegra_i2c_issue_bus_clear(struct tegra_i2c_dev *i2c_dev)
{
	int time_left, err;
	u32 reg;

	if (i2c_dev->hw->has_bus_clr_support) {
		reinit_completion(&i2c_dev->msg_complete);
		reg = (I2C_BC_SCLK_THRESHOLD << I2C_BC_SCLK_THRESHOLD_SHIFT) |
		      I2C_BC_STOP_COND | I2C_BC_TERMINATE;
		i2c_writel(i2c_dev, reg, I2C_BUS_CLEAR_CNFG);
		if (i2c_dev->hw->has_config_load_reg) {
			err = tegra_i2c_wait_for_config_load(i2c_dev);
			if (err)
				return err;
		}
		reg |= I2C_BC_ENABLE;
		i2c_writel(i2c_dev, reg, I2C_BUS_CLEAR_CNFG);
		tegra_i2c_unmask_irq(i2c_dev, I2C_INT_BUS_CLR_DONE);

		time_left = wait_for_completion_timeout(&i2c_dev->msg_complete,
							TEGRA_I2C_TIMEOUT);
		if (time_left == 0) {
			dev_err(i2c_dev->dev, "timed out for bus clear\n");
			return -ETIMEDOUT;
		}
		reg = i2c_readl(i2c_dev, I2C_BUS_CLEAR_STATUS);
		if (!(reg & I2C_BC_STATUS)) {
			dev_err(i2c_dev->dev, "Un-recovered Arb lost\n");
			return -EIO;
		}
	}

	return -EAGAIN;
}

static void tegra_i2c_config_fifo_trig(struct tegra_i2c_dev *i2c_dev,
		int len)
{
	u32 val, tx_trg, rx_trg;
	u8 maxburst;
	struct dma_slave_config dma_sconfig;

	if (len & 0xF) {
		tx_trg = I2C_FIFO_CTRL_TX_TRIG_1 << I2C_FIFO_CTRL_TX_TRIG_SHIFT;
		rx_trg = I2C_FIFO_CTRL_RX_TRIG_1 << I2C_FIFO_CTRL_RX_TRIG_SHIFT;
		val = tx_trg | rx_trg;
		maxburst = 1;
	} else if (((len) >> 4) & 0x1) {
		tx_trg = I2C_FIFO_CTRL_TX_TRIG_4 << I2C_FIFO_CTRL_TX_TRIG_SHIFT;
		rx_trg = I2C_FIFO_CTRL_RX_TRIG_4 << I2C_FIFO_CTRL_RX_TRIG_SHIFT;
		val = tx_trg | rx_trg;
		maxburst = 4;
	} else {
		tx_trg = I2C_FIFO_CTRL_TX_TRIG_8 << I2C_FIFO_CTRL_TX_TRIG_SHIFT;
		rx_trg = I2C_FIFO_CTRL_RX_TRIG_8 << I2C_FIFO_CTRL_RX_TRIG_SHIFT;
		val = tx_trg | rx_trg;
		maxburst = 8;
	}
	i2c_writel(i2c_dev, val, I2C_FIFO_CTRL);

	if (i2c_dev->curr_direction == DATA_DMA_DIR_TX) {
		dma_sconfig.dst_addr = i2c_dev->phys_addr + I2C_TX_FIFO;
		dma_sconfig.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		dma_sconfig.dst_maxburst = maxburst;
		dmaengine_slave_config(i2c_dev->tx_dma_chan, &dma_sconfig);
	} else {
		dma_sconfig.src_addr = i2c_dev->phys_addr + I2C_RX_FIFO;
		dma_sconfig.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		dma_sconfig.src_maxburst = maxburst;
		dmaengine_slave_config(i2c_dev->rx_dma_chan, &dma_sconfig);
	}
}

static int tegra_i2c_start_dma_xfer(struct tegra_i2c_dev *i2c_dev, u8 *buffer,
		size_t length, bool tx)
{
	int ret = 0;
	u32 int_mask;
	unsigned long flags = 0;

	i2c_dev->is_curr_dma_xfer = true;

	/* Enable error interrupts */
	int_mask = I2C_INT_NO_ACK | I2C_INT_ARBITRATION_LOST
					| I2C_INT_TX_FIFO_OVERFLOW;
	tegra_i2c_unmask_irq(i2c_dev, int_mask);

	if (tx) {
		i2c_dev->curr_direction = DATA_DMA_DIR_TX;
		dma_sync_single_for_cpu(i2c_dev->dev, i2c_dev->tx_dma_phys,
				i2c_dev->dma_buf_size, DMA_TO_DEVICE);

		memcpy(i2c_dev->tx_dma_buf, buffer, length);
		/* make the dma buffer to read by dma */
		dma_sync_single_for_device(i2c_dev->dev, i2c_dev->tx_dma_phys,
				i2c_dev->dma_buf_size, DMA_TO_DEVICE);

		length = ALIGN(length, BYTES_PER_FIFO_WORD);
		tegra_i2c_config_fifo_trig(i2c_dev, length);

		/* Acquire the lock before posting the data to FIFO */
		raw_spin_lock_irqsave(&i2c_dev->xfer_lock, flags);

		ret = tegra_i2c_start_tx_dma(i2c_dev, length);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Starting tx dma failed, err %d\n", ret);
			goto exit;
		}
	} else {
		u32 *buff_u32 = (u32 *)buffer;

		i2c_dev->curr_direction = DATA_DMA_DIR_RX;

		length = ALIGN(length, BYTES_PER_FIFO_WORD);
		tegra_i2c_config_fifo_trig(i2c_dev, length);

		ret = tegra_i2c_start_rx_dma(i2c_dev, length);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Starting rx dma failed, err %d\n", ret);
			return ret;
		}

		/* Acquire the lock before posting the data to FIFO */
		raw_spin_lock_irqsave(&i2c_dev->xfer_lock, flags);

		/* Transfer packet header through PIO */
		i2c_writel(i2c_dev, *(buff_u32++), I2C_TX_FIFO);
		i2c_writel(i2c_dev, *(buff_u32++), I2C_TX_FIFO);
		i2c_writel(i2c_dev, *(buff_u32++), I2C_TX_FIFO);
	}

	if (i2c_dev->hw->has_per_pkt_xfer_complete_irq)
		int_mask |= I2C_INT_PACKET_XFER_COMPLETE;

	int_mask |= I2C_INT_ALL_PACKETS_XFER_COMPLETE;
	tegra_i2c_unmask_irq(i2c_dev, int_mask);

exit:
	raw_spin_unlock_irqrestore(&i2c_dev->xfer_lock, flags);
	return ret;
}

static int tegra_i2c_start_pio_xfer(struct tegra_i2c_dev *i2c_dev, u8 *buffer,
		size_t length, bool tx)
{

	u32 val;
	u32 int_mask;
	u32 *buff_u32 = (u32 *)buffer;
	unsigned long flags = 0;

	i2c_dev->is_curr_dma_xfer = false;
	val = 7 << I2C_FIFO_CTRL_TX_TRIG_SHIFT |
		0 << I2C_FIFO_CTRL_RX_TRIG_SHIFT;
	i2c_writel(i2c_dev, val, I2C_FIFO_CTRL);

	/* Enable error interrupts */
	int_mask = I2C_INT_NO_ACK | I2C_INT_ARBITRATION_LOST |
		I2C_INT_TX_FIFO_OVERFLOW | I2C_INT_RX_FIFO_UNDERFLOW;
	tegra_i2c_unmask_irq(i2c_dev, int_mask);

	/* Acquire the lock before posting the data to FIFO */
	raw_spin_lock_irqsave(&i2c_dev->xfer_lock, flags);

	i2c_writel(i2c_dev, *(buff_u32++), I2C_TX_FIFO);
	i2c_writel(i2c_dev, *(buff_u32++), I2C_TX_FIFO);
	i2c_writel(i2c_dev, *(buff_u32++), I2C_TX_FIFO);

	if (tx)
		tegra_i2c_fill_tx_fifo(i2c_dev);

	if (i2c_dev->hw->has_per_pkt_xfer_complete_irq)
		int_mask |= I2C_INT_PACKET_XFER_COMPLETE;

	int_mask |= I2C_INT_ALL_PACKETS_XFER_COMPLETE;
	if (tx && i2c_dev->msg_tx_remaining)
		int_mask |= I2C_INT_TX_FIFO_DATA_REQ;
	else if (!tx && i2c_dev->msg_rx_remaining)
		int_mask |= I2C_INT_RX_FIFO_DATA_REQ;

	tegra_i2c_unmask_irq(i2c_dev, int_mask);

	raw_spin_unlock_irqrestore(&i2c_dev->xfer_lock, flags);

	return 0;
}

static int tegra_i2c_pre_xfer_config(struct tegra_i2c_dev *i2c_dev,
		u8 *buffer, size_t length, bool tx)
{
	tegra_i2c_flush_fifos(i2c_dev);
	if (tx) {
		i2c_dev->msg_tx_buf = buffer + I2C_PACKET_HEADER_SIZE * 4;
		i2c_dev->msg_tx_remaining = length - I2C_PACKET_HEADER_SIZE * 4;
		i2c_dev->msg_rx_buf = NULL;
		i2c_dev->msg_rx_remaining = 0;
	} else {
		i2c_dev->msg_rx_buf = buffer;
		i2c_dev->msg_rx_remaining = length;
		i2c_dev->msg_tx_buf = NULL;
		i2c_dev->msg_tx_remaining = 0;
	}
	i2c_writel(i2c_dev, 0, I2C_FIFO_CTRL);
	i2c_dev->msg_err = I2C_ERR_NONE;
	i2c_writel(i2c_dev, 0, I2C_INT_MASK);
	reinit_completion(&i2c_dev->msg_complete);

	return 0;
}

static int tegra_i2c_handle_xfer_error(struct tegra_i2c_dev *i2c_dev)
{
	int ret;

	/* Prints errors */
	if (i2c_dev->msg_err & I2C_ERR_UNKNOWN_INTERRUPT)
		dev_warn(i2c_dev->dev, "unknown interrupt Add 0x%02x\n",
				i2c_dev->msg_add);
	if (i2c_dev->msg_err & I2C_ERR_NO_ACK)
		dev_warn(i2c_dev->dev, "no acknowledge from address 0x%x\n",
				i2c_dev->msg_add);
	if (i2c_dev->msg_err & I2C_ERR_ARBITRATION_LOST)
		dev_warn(i2c_dev->dev, "arb lost in communicate to add 0x%x\n",
				i2c_dev->msg_add);
	if (i2c_dev->msg_err & I2C_INT_TX_FIFO_OVERFLOW)
		dev_warn(i2c_dev->dev, "Tx fifo overflow to add 0x%x\n",
				i2c_dev->msg_add);
	if (i2c_dev->msg_err & I2C_INT_RX_FIFO_UNDERFLOW)
		dev_warn(i2c_dev->dev, "Rx fifo underflow to add 0x%x\n",
				i2c_dev->msg_add);
	/*
	 * NACK interrupt is generated before the I2C controller generates
	 * the STOP condition on the bus. So wait for 2 clock periods
	 * before resetting the controller so that the STOP condition has
	 * been delivered properly.
	 */
	if (i2c_dev->msg_err == I2C_ERR_NO_ACK)
		udelay(DIV_ROUND_UP(2 * 1000000, i2c_dev->bus_clk_rate));

	ret = tegra_i2c_init(i2c_dev);
	if (ret) {
		WARN_ON(1);
		return ret;
	}

	/* Arbitration Lost occurs, Start recovery */
	if (i2c_dev->msg_err == I2C_ERR_ARBITRATION_LOST) {
		if (!i2c_dev->is_multimaster_mode)
			return tegra_i2c_issue_bus_clear(i2c_dev);
		return -EAGAIN;
	}

	if (i2c_dev->msg_err == I2C_ERR_NO_ACK)
		return -EREMOTEIO;

	return -EIO;
}

static void tegra_i2c_reg_dump(struct tegra_i2c_dev *i2c_dev)
{
	dev_err(i2c_dev->dev, "--- register dump for debugging ----\n");
	dev_err(i2c_dev->dev, "I2C_CNFG - 0x%x\n",
			i2c_readl(i2c_dev, I2C_CNFG));
	dev_err(i2c_dev->dev, "I2C_PACKET_TRANSFER_STATUS - 0x%x\n",
			i2c_readl(i2c_dev, I2C_PACKET_TRANSFER_STATUS));
	dev_err(i2c_dev->dev, "I2C_FIFO_CTRL - 0x%x\n",
			i2c_readl(i2c_dev, I2C_FIFO_CTRL));
	dev_err(i2c_dev->dev, "I2C_FIFO_STATUS - 0x%x\n",
			i2c_readl(i2c_dev, I2C_FIFO_STATUS));
	dev_err(i2c_dev->dev, "I2C_INT_MASK - 0x%x\n",
			i2c_readl(i2c_dev, I2C_INT_MASK));
	dev_err(i2c_dev->dev, "I2C_INT_STATUS - 0x%x\n",
			i2c_readl(i2c_dev, I2C_INT_STATUS));
}

static int tegra_i2c_xfer_msg(struct tegra_i2c_dev *i2c_dev, u8 *buffer,
		u32 length, bool tx)
{
	u32 int_mask = 0;
	unsigned long time_left;
	int ret;

	WARN_ON(!IS_ALIGNED((unsigned long)buffer, BYTES_PER_FIFO_WORD));
	ret = tegra_i2c_pre_xfer_config(i2c_dev, buffer, length, tx);
	if (ret)
		return ret;

	if ((length > I2C_PIO_MODE_MAX_LEN) && i2c_dev->tx_dma_chan
				&& i2c_dev->rx_dma_chan)
		ret = tegra_i2c_start_dma_xfer(i2c_dev, buffer, length, tx);
	else
		ret = tegra_i2c_start_pio_xfer(i2c_dev, buffer, length, tx);

	if (ret)
		return ret;

	dev_dbg(i2c_dev->dev, "unmasked irq: %02x\n",
		i2c_readl(i2c_dev, I2C_INT_MASK));

	if ((length > I2C_PIO_MODE_MAX_LEN) && i2c_dev->tx_dma_chan
				&& i2c_dev->rx_dma_chan) {
		if (i2c_dev->curr_direction == DATA_DMA_DIR_TX) {
			time_left = wait_for_completion_timeout(
					&i2c_dev->tx_dma_complete,
					TEGRA_I2C_TIMEOUT);
			if (time_left == 0) {
				dev_err(i2c_dev->dev, "tx dma timeout\n");
				dmaengine_terminate_all(i2c_dev->tx_dma_chan);
				goto end_xfer;
			}
		} else if (i2c_dev->curr_direction == DATA_DMA_DIR_RX) {
			time_left = wait_for_completion_timeout(
					&i2c_dev->rx_dma_complete,
					TEGRA_I2C_TIMEOUT);
			if (time_left == 0) {
				dev_err(i2c_dev->dev, "rx dma timeout\n");
				dmaengine_terminate_all(i2c_dev->rx_dma_chan);
				goto end_xfer;
			}
			if (likely(i2c_dev->msg_err == I2C_ERR_NONE)) {
				dma_sync_single_for_cpu(i2c_dev->dev,
						i2c_dev->rx_dma_phys,
						i2c_dev->dma_buf_size,
						DMA_FROM_DEVICE);
				memcpy(i2c_dev->pio_buffer, i2c_dev->rx_dma_buf,
						length);
				dma_sync_single_for_device(i2c_dev->dev,
						i2c_dev->rx_dma_phys,
						i2c_dev->dma_buf_size,
						DMA_FROM_DEVICE);
			}
		}
	}

	time_left = wait_for_completion_timeout(&i2c_dev->msg_complete,
						TEGRA_I2C_TIMEOUT);
	if (time_left == 0) {
		dev_err(i2c_dev->dev, "pio xfer timed out addr: 0x%x\n",
			i2c_dev->msg_add);
		tegra_i2c_reg_dump(i2c_dev);
		if (i2c_dev->is_curr_dma_xfer) {
			if (i2c_dev->curr_direction == DATA_DMA_DIR_TX)
				dmaengine_terminate_all(i2c_dev->tx_dma_chan);
			else if (i2c_dev->curr_direction == DATA_DMA_DIR_RX)
				dmaengine_terminate_all(i2c_dev->rx_dma_chan);
		}
	}

end_xfer:
	int_mask = i2c_readl(i2c_dev, I2C_INT_MASK);
	tegra_i2c_mask_irq(i2c_dev, int_mask);

	if (time_left == 0) {
		dev_err(i2c_dev->dev, "i2c transfer timed out addr: 0x%x\n",
			i2c_dev->msg_add);
		ret = tegra_i2c_init(i2c_dev);
		if (!ret)
			ret = -ETIMEDOUT;
		else
			WARN_ON(1);
		return ret;
	}

	dev_dbg(i2c_dev->dev, "transfer complete: %d %d %d\n",
		ret, completion_done(&i2c_dev->msg_complete), i2c_dev->msg_err);

	if (likely(i2c_dev->msg_err == I2C_ERR_NONE))
		return 0;

	return tegra_i2c_handle_xfer_error(i2c_dev);
}

static void tegra_i2c_prepare_buffer(struct tegra_i2c_dev *i2c_dev,
		struct i2c_msg *msg, enum msg_end_type end_st,
		bool chain, u8 *buffer)
{
	u32 io_header;
	u32 *buff_u32 = (u32 *)buffer;
	u32 packet_header[3];

	WARN_ON(!IS_ALIGNED((unsigned long)buffer, BYTES_PER_FIFO_WORD));

	/* Generic header packet */
	packet_header[0] = (0 << PACKET_HEADER0_HEADER_SIZE_SHIFT)
		| PACKET_HEADER0_PROTOCOL_I2C
		| (i2c_dev->cont_id << PACKET_HEADER0_CONT_ID_SHIFT)
		| (1 << PACKET_HEADER0_PACKET_ID_SHIFT);

	/* Payload size */
	packet_header[1] = msg->len - 1;

	/* IO header */
	io_header = 0;

	if (((!chain) && i2c_dev->has_rx) ||
		i2c_dev->use_single_xfer_complete)
		io_header |= I2C_HEADER_IE_ENABLE;

	if (end_st == MSG_END_CONTINUE)
		io_header |= I2C_HEADER_CONTINUE_XFER;
	else if (end_st == MSG_END_REPEAT_START)
		io_header |= I2C_HEADER_REPEAT_START;
	if (msg->flags & I2C_M_TEN) {
		io_header |= msg->addr;
		io_header |= I2C_HEADER_10BIT_ADDR;
	} else {
		io_header |=
			msg->addr << I2C_HEADER_SLAVE_ADDR_SHIFT;
	}
	if (msg->flags & I2C_M_IGNORE_NAK)
		io_header |= I2C_HEADER_CONT_ON_NAK;
	if (msg->flags & I2C_M_RD)
		io_header |= I2C_HEADER_READ;
	if (i2c_dev->bus_clk_rate == I2C_HS_MODE) {
		io_header |= I2C_HEADER_HIGHSPEED_MODE;
		io_header |= ((i2c_dev->hs_master_code & 0x7)
				<<  I2C_HEADER_MASTER_ADDR_SHIFT);
	}
	packet_header[2] = io_header;

	/* populate the packet header */
	*(buff_u32++) = packet_header[0];
	*(buff_u32++) = packet_header[1];
	*(buff_u32++) = packet_header[2];

	if (!(msg->flags & I2C_M_RD))
		memcpy(buff_u32, msg->buf, msg->len);
}

enum msg_end_type tegra_i2c_calc_end_bit(struct i2c_msg msgs[], int num, int i)
{
	if (i >= (num - 1))
		return MSG_END_STOP;

	if (msgs[i + 1].flags & I2C_M_NOSTART)
		return MSG_END_CONTINUE;

	return MSG_END_REPEAT_START;
}

static int tegra_i2c_xfer_tx(struct tegra_i2c_dev *i2c_dev,
		struct i2c_msg msgs[], int num, int strt_idx, int tx_num)
{
	struct i2c_msg temp_msg;
	u8 *buff = i2c_dev->pio_buffer;
	u32 tx_len = 0;
	u16 msg_len;
	int i;
	bool chain, temp_chain;
	enum msg_end_type end_t, temp_end_t;

	if (!tx_num)
		return 0;

	for (i = strt_idx; i < strt_idx + tx_num; i++) {
		/* Bailout if message length is ZERO */
		if (msgs[i].len == 0)
			return -EINVAL;

		end_t = tegra_i2c_calc_end_bit(msgs, num, i);

		chain = (i < (strt_idx + tx_num - 1)) ? true : false;

		/* Split into multiple 4096 byte packets if needed */
		memcpy(&temp_msg, &msgs[i], sizeof(struct i2c_msg));
		msg_len = msgs[i].len;
		do {
			if (msg_len > I2C_MAX_TRANSFER_LEN) {
				temp_chain = true;
				temp_end_t = MSG_END_CONTINUE;
				temp_msg.len = I2C_MAX_TRANSFER_LEN;
			} else {
				temp_chain = chain;
				temp_end_t = end_t;
				temp_msg.len = msg_len;
			}
			msg_len -= temp_msg.len;

			/* Fill packet header as well as the buffer */
			tegra_i2c_prepare_buffer(i2c_dev, &temp_msg,
					temp_end_t, temp_chain, buff);

			/*
			 * transmit length is word aligned to ensure
			 * writing to FIFO can be done word by word
			 */
			tx_len += ALIGN(temp_msg.len, 4) + 12;
			buff += ALIGN(temp_msg.len, 4) + 12;

			temp_msg.buf += temp_msg.len;
			/*
			 * Assumption: All packets in one chain can be processed
			 * with I2C_TOTAL_BUFFER_LEN (32KB) worth of memory ie
			 * hdr1(12bytes) + data1(l1) + ... + hdrN + dataN(lN)
			 * should be <= I2C_TOTAL_BUFFER_LEN
			 */
			if (tx_len > I2C_TOTAL_BUFFER_LEN)
				return -EINVAL;
		} while (msg_len);
	}

	return tegra_i2c_xfer_msg(i2c_dev, i2c_dev->pio_buffer, tx_len, true);
}

static int tegra_i2c_single_xfer_rx(struct tegra_i2c_dev *i2c_dev,
	struct i2c_msg *msg, enum msg_end_type end_t)
{
	int ret;

	/* Bailout if message length is ZERO */
	if (msg->len == 0)
		return -EINVAL;

	i2c_dev->use_single_xfer_complete = true;
	tegra_i2c_prepare_buffer(i2c_dev, msg, end_t, false,
			i2c_dev->pio_buffer);
	/*
	 * Packet header (12 bytes) is passed via pio_buffer (to be
	 * written to TX_FIFO) and the received bytes (via RX_FIFO)
	 * would later be written into the same buffer
	 */
	ret = tegra_i2c_xfer_msg(i2c_dev, i2c_dev->pio_buffer,
			msg->len, false);
	if (ret)
		return ret;

	memcpy(msg->buf, i2c_dev->pio_buffer, msg->len);

	return 0;
}

static int tegra_i2c_single_xfer_tx(struct tegra_i2c_dev *i2c_dev,
	struct i2c_msg *msg, enum msg_end_type end_t)
{
	u8 *buff = i2c_dev->pio_buffer;
	u32 tx_len;

	tx_len = ALIGN(msg->len, 4) + 12;
	/* Fill packet header as well as the buffer */
	tegra_i2c_prepare_buffer(i2c_dev, msg, end_t, false, buff);

	return tegra_i2c_xfer_msg(i2c_dev, i2c_dev->pio_buffer, tx_len, true);
}

static int tegra_i2c_single_xfer_msg(struct tegra_i2c_dev *i2c_dev,
	struct i2c_msg *msg, enum msg_end_type end_type)
{
	int ret;

	i2c_dev->use_single_xfer_complete = true;
	if (msg->flags & I2C_M_RD)
		ret = tegra_i2c_single_xfer_rx(i2c_dev, msg, end_type);
	else
		ret = tegra_i2c_single_xfer_tx(i2c_dev, msg, end_type);

	return ret;
}

static int tegra_i2c_split_i2c_msg_xfer(struct tegra_i2c_dev *i2c_dev,
	struct i2c_msg *msg, enum msg_end_type end_type)
{
	int size, len, ret;
	struct i2c_msg temp_msg;
	u8 *buf = msg->buf;
	enum msg_end_type temp_end_type;

	size = msg->len;
	temp_msg.flags = msg->flags;
	temp_msg.addr = msg->addr;
	temp_end_type = end_type;
	do {
		temp_msg.buf = buf;
		len = min(size, I2C_MAX_TRANSFER_LEN);
		temp_msg.len = len;
		size -= len;
		if ((len == I2C_MAX_TRANSFER_LEN) && size)
			end_type = MSG_END_CONTINUE;
		else
			end_type = temp_end_type;
		ret = tegra_i2c_single_xfer_msg(i2c_dev, &temp_msg, end_type);
		if (ret)
			return ret;
		buf += len;
	} while (size != 0);

	return ret;
}

static int tegra_i2c_multi_pkt_xfer(struct tegra_i2c_dev *i2c_dev,
	struct i2c_msg msgs[], int num)
{
	int i = 0;
	int ret = 0;

	/*
	 * I2C WR: Need to write header (12 bytes) + packet length to FIFO
	 * I2C RD: Need to write 12 bytes to FIFO and read packet length #bytes
	 *
	 * Note: All read transfers would be handled independently
	 *
	 * Permitted chaining scenarios
	 * a. WR -> WR -> WR
	 * b. RD
	 *
	 * Each read transaction should be independently handled
	 * as chaining is only done for write transactions.
	 */
	while (i < num) {
		enum msg_end_type end_t;
		int strt_idx = i;
		int tx_num;

		/* Calculate the number of TX transactions */
		for (tx_num = 0; (i < num) && !(msgs[i].flags & I2C_M_RD);
				i++, tx_num++)
			;

		i2c_dev->has_rx = (i < num) ? true : false;

		/* TX packets can be sent to the wire all at once */
		if (tx_num) {
			ret = tegra_i2c_xfer_tx(i2c_dev, msgs, num, strt_idx,
					tx_num);
			if (ret)
				goto exit;
		}

		for (; (i < num) && (msgs[i].flags & I2C_M_RD); i++) {
			end_t = tegra_i2c_calc_end_bit(msgs, num, i);

			/* RX packets can only be received one at a time */
			if (msgs[i].len > I2C_MAX_TRANSFER_LEN)
				ret = tegra_i2c_split_i2c_msg_xfer(i2c_dev,
						&msgs[i], end_t);
			else
				ret = tegra_i2c_single_xfer_rx(i2c_dev, &msgs[i],
						end_t);
			if (ret)
				goto exit;
		}
	}
exit:
	return ret ?: i;
}

static int tegra_i2c_single_pkt_xfer(struct tegra_i2c_dev *i2c_dev,
	struct i2c_msg msgs[], int num)
{
	int i;
	int ret = 0;

	for (i = 0; i < num; i++) {
		enum msg_end_type end_type = MSG_END_STOP;

		if (i < (num - 1)) {
			if (msgs[i + 1].flags & I2C_M_NOSTART)
				end_type = MSG_END_CONTINUE;
			else
				end_type = MSG_END_REPEAT_START;
		}
		if (msgs[i].len > I2C_MAX_TRANSFER_LEN)
			ret = tegra_i2c_split_i2c_msg_xfer(i2c_dev, &msgs[i],
					end_type);
		else
			ret = tegra_i2c_single_xfer_msg(i2c_dev, &msgs[i],
					end_type);
		if (ret)
			break;
	}

	return ret ? : i;
}

static int tegra_i2c_is_multi_pkt_supported(struct tegra_i2c_dev *i2c_dev,
	struct i2c_msg msgs[], int num)
{
	int i;

	if (i2c_dev->disable_multi_pkt_mode)
		return false;

	for (i = 0; i < num; i++) {
		if (msgs[i].flags & I2C_M_NOSTART)
			return false;
	}
	return true;
}

static int tegra_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
	int num)
{
	struct tegra_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	int ret = 0;

	/* Saving msg info for debug */
	i2c_dev->msg_add = msgs[0].addr;
	i2c_dev->msgs = msgs;
	i2c_dev->msg_num = num;
	i2c_dev->has_rx = false;

	if (i2c_dev->is_suspended)
		return -EBUSY;

	ret = pm_runtime_get_sync(i2c_dev->dev);
	if (ret < 0) {
		dev_err(i2c_dev->dev, "runtime resume failed %d\n", ret);
		return ret;
	}

	if (i2c_dev->is_shutdown && i2c_dev->bit_banging_xfer_after_shutdown)
		return tegra_i2c_gpio_xfer(adap, msgs, num);

	if (adap->bus_clk_rate != i2c_dev->bus_clk_rate) {
		i2c_dev->bus_clk_rate = adap->bus_clk_rate;
		ret = tegra_i2c_set_clk_rate(i2c_dev);
		if (ret < 0)
			return ret;
	}

	if (!i2c_dev->disable_dma_mode) {
		/*
		 * tegra_i2c_init_dma_param can return -EPROBE_DEFER
		 * whenever an i2c xfer is requested even before DMA
		 * driver is registered.
		 * Skip error return and proceed with PIO mode in that
		 * case
		 */
		if (!i2c_dev->rx_dma_chan) {
			ret = tegra_i2c_init_dma_param(i2c_dev, true);
			if (ret && (ret != -EPROBE_DEFER) && (ret != -ENODEV))
				return ret;
		}

		if (!i2c_dev->tx_dma_chan) {
			ret = tegra_i2c_init_dma_param(i2c_dev, false);
			if (ret && (ret != -EPROBE_DEFER) && (ret != -ENODEV))
				return ret;
		}
	}
	i2c_dev->use_single_xfer_complete = false;

	if (tegra_i2c_is_multi_pkt_supported(i2c_dev, msgs, num))
		ret = tegra_i2c_multi_pkt_xfer(i2c_dev, msgs, num);
	else
		ret = tegra_i2c_single_pkt_xfer(i2c_dev, msgs, num);

	pm_runtime_put(i2c_dev->dev);
	return ret;
}

static u32 tegra_i2c_func(struct i2c_adapter *adap)
{
	struct tegra_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	u32 ret = I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK) |
		  I2C_FUNC_10BIT_ADDR |	I2C_FUNC_PROTOCOL_MANGLING;

	if (i2c_dev->hw->has_continue_xfer_support)
		ret |= I2C_FUNC_NOSTART;
	return ret;
}

static void tegra_i2c_parse_dt(struct tegra_i2c_dev *i2c_dev)
{
	struct device_node *np = i2c_dev->dev->of_node;
	int ret;
	u32 prop;

	ret = of_property_read_u32(np, "clock-frequency",
			&i2c_dev->bus_clk_rate);
	if (ret)
		i2c_dev->bus_clk_rate = 100000; /* default clock rate */

	i2c_dev->is_multimaster_mode = of_property_read_bool(np,
			"multi-master");

	i2c_dev->scl_gpio = of_get_named_gpio(np, "scl-gpio", 0);
	i2c_dev->sda_gpio = of_get_named_gpio(np, "sda-gpio", 0);

	i2c_dev->bit_banging_xfer_after_shutdown = of_property_read_bool(np,
			"nvidia,bit-banging-xfer-after-shutdown");

	ret = of_property_read_u32(np, "nvidia,hs-master-code", &prop);
	if (!ret)
		i2c_dev->hs_master_code = prop;

	i2c_dev->disable_dma_mode = of_property_read_bool(np,
			"nvidia,disable-dma-mode");
	i2c_dev->is_clkon_always = of_property_read_bool(np,
			"nvidia,clock-always-on");
	i2c_dev->disable_multi_pkt_mode = of_property_read_bool(np,
			"nvidia,disable-multi-pkt-mode");
}

static const struct i2c_algorithm tegra_i2c_algo = {
	.master_xfer	= tegra_i2c_xfer,
	.functionality	= tegra_i2c_func,
};

/* payload size is only 12 bit */
static struct i2c_adapter_quirks tegra_i2c_quirks = {
	.max_read_len = 4096,
	.max_write_len = 4096,
};

static const struct tegra_i2c_hw_feature tegra20_i2c_hw = {
	.has_continue_xfer_support = false,
	.has_per_pkt_xfer_complete_irq = false,
	.has_single_clk_source = false,
	.clk_divisor_hs_mode = 3,
	.clk_multiplier_hs_mode = 12,
	.clk_divisor_std_fast_mode = 0,
	.clk_divisor_fast_plus_mode = 0,
	.has_config_load_reg = false,
	.has_multi_master_mode = false,
	.has_slcg_override_reg = false,
	.has_sw_reset_reg = false,
	.has_bus_clr_support = false,
	.has_reg_write_buffering = true,
	.has_slcg_support = false,
};

static const struct tegra_i2c_hw_feature tegra30_i2c_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = false,
	.has_single_clk_source = false,
	.clk_divisor_hs_mode = 3,
	.clk_multiplier_hs_mode = 12,
	.clk_divisor_std_fast_mode = 0,
	.clk_divisor_fast_plus_mode = 0,
	.has_config_load_reg = false,
	.has_multi_master_mode = false,
	.has_slcg_override_reg = false,
	.has_sw_reset_reg = false,
	.has_bus_clr_support = false,
	.has_reg_write_buffering = true,
	.has_slcg_support = false,
};

static const struct tegra_i2c_hw_feature tegra114_i2c_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = true,
	.has_single_clk_source = true,
	.clk_divisor_hs_mode = 1,
	.clk_multiplier_hs_mode = 3,
	.clk_divisor_std_fast_mode = 0x19,
	.clk_divisor_fast_plus_mode = 0x10,
	.has_config_load_reg = false,
	.has_multi_master_mode = false,
	.has_slcg_override_reg = false,
	.has_sw_reset_reg = false,
	.has_bus_clr_support = true,
	.has_reg_write_buffering = true,
	.has_slcg_support = false,
};

static const struct tegra_i2c_hw_feature tegra124_i2c_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = true,
	.has_single_clk_source = true,
	.clk_divisor_hs_mode = 2,
	.clk_multiplier_hs_mode = 13,
	.clk_divisor_std_fast_mode = 0x19,
	.clk_divisor_fast_plus_mode = 0x10,
	.has_config_load_reg = true,
	.has_multi_master_mode = false,
	.has_slcg_override_reg = true,
	.has_sw_reset_reg = false,
	.has_bus_clr_support = true,
	.has_reg_write_buffering = true,
	.has_slcg_support = false,
};

static const struct tegra_i2c_hw_feature tegra210_i2c_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = true,
	.has_single_clk_source = true,
	.clk_divisor_hs_mode = 2,
	.clk_multiplier_hs_mode = 13,
	.clk_divisor_std_fast_mode = 0x19,
	.clk_divisor_fast_plus_mode = 0x10,
	.has_config_load_reg = true,
	.has_multi_master_mode = true,
	.has_slcg_override_reg = true,
	.has_sw_reset_reg = false,
	.has_bus_clr_support = true,
	.has_reg_write_buffering = true,
	.has_slcg_support = false,
};

static const struct tegra_i2c_hw_feature tegra186_i2c_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = true,
	.has_single_clk_source = true,
	.clk_divisor_hs_mode = 2,
	.clk_multiplier_hs_mode = 13,
	.clk_divisor_std_fast_mode = 0x19,
	.clk_divisor_fast_plus_mode = 0x10,
	.has_config_load_reg = true,
	.has_multi_master_mode = true,
	.has_slcg_override_reg = true,
	.has_sw_reset_reg = true,
	.has_bus_clr_support = true,
	.has_reg_write_buffering = false,
	.has_slcg_support = true,
};

/* Match table for of_platform binding */
static const struct of_device_id tegra_i2c_of_match[] = {
	{ .compatible = "nvidia,tegra186-i2c", .data = &tegra186_i2c_hw, },
	{ .compatible = "nvidia,tegra210-i2c", .data = &tegra210_i2c_hw, },
	{ .compatible = "nvidia,tegra124-i2c", .data = &tegra124_i2c_hw, },
	{ .compatible = "nvidia,tegra114-i2c", .data = &tegra114_i2c_hw, },
	{ .compatible = "nvidia,tegra30-i2c", .data = &tegra30_i2c_hw, },
	{ .compatible = "nvidia,tegra20-i2c", .data = &tegra20_i2c_hw, },
	{ .compatible = "nvidia,tegra20-i2c-dvc", .data = &tegra20_i2c_hw, },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_i2c_of_match);

static int tegra_i2c_probe(struct platform_device *pdev)
{
	struct tegra_i2c_dev *i2c_dev;
	struct resource *res;
	struct clk *div_clk;
	struct clk *fast_clk;
	struct clk *parent_clk;
	void __iomem *base;
	phys_addr_t phys_addr;
	int irq;
	int ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	phys_addr = res->start;
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "no irq resource\n");
		return -EINVAL;
	}
	irq = res->start;

	div_clk = devm_clk_get(&pdev->dev, "div-clk");
	if (IS_ERR(div_clk)) {
		dev_err(&pdev->dev, "missing controller clock\n");
		return PTR_ERR(div_clk);
	}

	parent_clk = devm_clk_get(&pdev->dev, "parent");
	if (IS_ERR(parent_clk)) {
		dev_err(&pdev->dev, "Unable to get parent_clk err:%ld\n",
				PTR_ERR(parent_clk));
	} else {
		ret = clk_set_parent(div_clk, parent_clk);
		if (ret < 0)
			dev_warn(&pdev->dev, "Couldn't set parent clock : %d\n",
				ret);
	}

	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;

	i2c_dev->base = base;
	i2c_dev->phys_addr = phys_addr;
	i2c_dev->div_clk = div_clk;
	i2c_dev->adapter.algo = &tegra_i2c_algo;
	i2c_dev->adapter.quirks = &tegra_i2c_quirks;
	i2c_dev->irq = irq;
	i2c_dev->dev = &pdev->dev;
	i2c_dev->dma_buf_size = I2C_DMA_MAX_BUF_LEN;

	i2c_dev->rst = devm_reset_control_get(&pdev->dev, "i2c");
	if (IS_ERR(i2c_dev->rst)) {
		dev_err(&pdev->dev, "missing controller reset\n");
		return PTR_ERR(i2c_dev->rst);
	}

	tegra_i2c_parse_dt(i2c_dev);

	i2c_dev->hw = of_device_get_match_data(&pdev->dev);
	i2c_dev->is_dvc = of_device_is_compatible(pdev->dev.of_node,
						  "nvidia,tegra20-i2c-dvc");
	init_completion(&i2c_dev->msg_complete);
	raw_spin_lock_init(&i2c_dev->xfer_lock);
	init_completion(&i2c_dev->tx_dma_complete);
	init_completion(&i2c_dev->rx_dma_complete);

	if (!i2c_dev->hw->has_single_clk_source) {
		fast_clk = devm_clk_get(&pdev->dev, "fast-clk");
		if (IS_ERR(fast_clk)) {
			dev_err(&pdev->dev, "missing fast clock\n");
			return PTR_ERR(fast_clk);
		}
		i2c_dev->fast_clk = fast_clk;
	}

	i2c_dev->prod_list = devm_tegra_prod_get(&pdev->dev);
	if (IS_ERR_OR_NULL(i2c_dev->prod_list)) {
		dev_dbg(&pdev->dev, "Prod-setting not available\n");
		i2c_dev->prod_list = NULL;
	}

	platform_set_drvdata(pdev, i2c_dev);

	if (!i2c_dev->hw->has_single_clk_source) {
		ret = clk_prepare(i2c_dev->fast_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev, "Clock prepare failed %d\n", ret);
			return ret;
		}
	}

	i2c_dev->clk_divisor_non_hs_mode =
			i2c_dev->hw->clk_divisor_std_fast_mode;
	if (i2c_dev->hw->clk_divisor_fast_plus_mode &&
		(i2c_dev->bus_clk_rate == I2C_FAST_MODE_PLUS))
		i2c_dev->clk_divisor_non_hs_mode =
			i2c_dev->hw->clk_divisor_fast_plus_mode;

	ret = tegra_i2c_set_clk_rate(i2c_dev);
	if (ret < 0)
		return ret;

	ret = clk_prepare(i2c_dev->div_clk);
	if (ret < 0) {
		dev_err(i2c_dev->dev, "Clock prepare failed %d\n", ret);
		goto unprepare_fast_clk;
	}

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra_i2c_runtime_resume(&pdev->dev);
		if (ret < 0) {
			dev_err(&pdev->dev, "runtime resume failed\n");
			goto unprepare_div_clk;
		}
	}

	if (i2c_dev->is_multimaster_mode || i2c_dev->hw->has_slcg_support)
		i2c_dev->is_clkon_always = true;

	if (i2c_dev->is_clkon_always) {
		ret = clk_enable(i2c_dev->div_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev, "div_clk enable failed %d\n",
				ret);
			goto disable_rpm;
		}
	}

	if (!i2c_dev->disable_dma_mode) {
		ret = tegra_i2c_init_dma_param(i2c_dev, true);
		if (ret && (ret != -EPROBE_DEFER) && (ret != -ENODEV))
			goto disable_div_clk;
		ret = tegra_i2c_init_dma_param(i2c_dev, false);
		if (ret && (ret != -EPROBE_DEFER) && (ret != -ENODEV))
			goto disable_div_clk;
	}

	i2c_dev->pio_buffer = devm_kzalloc(&pdev->dev,
			I2C_TOTAL_BUFFER_LEN, GFP_KERNEL);
	if (!(i2c_dev->pio_buffer))
		return -ENOMEM;

	ret = tegra_i2c_init(i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize i2c controller\n");
		goto disable_div_clk;
	}

	ret = devm_request_irq(&pdev->dev, i2c_dev->irq,
			tegra_i2c_isr, 0, dev_name(&pdev->dev), i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq %i\n", i2c_dev->irq);
		goto disable_div_clk;
	}

	i2c_set_adapdata(&i2c_dev->adapter, i2c_dev);
	i2c_dev->adapter.owner = THIS_MODULE;
	i2c_dev->adapter.class = I2C_CLASS_DEPRECATED;
	strlcpy(i2c_dev->adapter.name, dev_name(&pdev->dev),
		sizeof(i2c_dev->adapter.name));
	i2c_dev->adapter.bus_clk_rate = i2c_dev->bus_clk_rate;
	i2c_dev->adapter.dev.parent = &pdev->dev;
	i2c_dev->adapter.nr = pdev->id;
	i2c_dev->adapter.dev.of_node = pdev->dev.of_node;

	ret = i2c_add_numbered_adapter(&i2c_dev->adapter);
	if (ret)
		goto disable_div_clk;
	i2c_dev->cont_id = i2c_dev->adapter.nr & PACKET_HEADER0_CONT_ID_MASK;
	tegra_i2c_gpio_init(i2c_dev);

	return 0;

disable_div_clk:
	if (i2c_dev->is_clkon_always)
		clk_disable(i2c_dev->div_clk);

disable_rpm:
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra_i2c_runtime_suspend(&pdev->dev);

unprepare_div_clk:
	clk_unprepare(i2c_dev->div_clk);

unprepare_fast_clk:
	if (!i2c_dev->hw->has_single_clk_source)
		clk_unprepare(i2c_dev->fast_clk);

	return ret;
}

static int tegra_i2c_remove(struct platform_device *pdev)
{
	struct tegra_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c_dev->adapter);

	if (i2c_dev->tx_dma_chan)
		tegra_i2c_deinit_dma_param(i2c_dev, false);
	if (i2c_dev->rx_dma_chan)
		tegra_i2c_deinit_dma_param(i2c_dev, true);

	if (i2c_dev->is_clkon_always)
		clk_disable(i2c_dev->div_clk);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra_i2c_runtime_suspend(&pdev->dev);

	clk_unprepare(i2c_dev->div_clk);
	if (!i2c_dev->hw->has_single_clk_source)
		clk_unprepare(i2c_dev->fast_clk);

	return 0;
}

static void tegra_i2c_shutdown(struct platform_device *pdev)
{
	struct tegra_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	dev_info(i2c_dev->dev, "Bus is shutdown down..\n");
	i2c_shutdown_adapter(&i2c_dev->adapter);
	i2c_dev->is_shutdown = true;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_i2c_suspend(struct device *dev)
{
	struct tegra_i2c_dev *i2c_dev = dev_get_drvdata(dev);

	i2c_lock_adapter(&i2c_dev->adapter);
	i2c_dev->is_suspended = true;

	if (i2c_dev->is_clkon_always)
		clk_disable(i2c_dev->div_clk);

	i2c_unlock_adapter(&i2c_dev->adapter);

	return 0;
}

static int tegra_i2c_resume(struct device *dev)
{
	struct tegra_i2c_dev *i2c_dev = dev_get_drvdata(dev);
	int ret;

	i2c_lock_adapter(&i2c_dev->adapter);

	ret = tegra_i2c_init(i2c_dev);
	if (!ret)
		i2c_dev->is_suspended = false;

	if (i2c_dev->is_clkon_always) {
		ret = clk_enable(i2c_dev->div_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev, "clock enable failed %d\n",
				ret);
			i2c_unlock_adapter(&i2c_dev->adapter);
			return ret;
		}
	}

	i2c_unlock_adapter(&i2c_dev->adapter);

	return ret;
}

static const struct dev_pm_ops tegra_i2c_pm = {
	SET_RUNTIME_PM_OPS(tegra_i2c_runtime_suspend, tegra_i2c_runtime_resume,
			   NULL)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(tegra_i2c_suspend, tegra_i2c_resume)
};
#define TEGRA_I2C_PM	(&tegra_i2c_pm)
#else
#define TEGRA_I2C_PM	NULL
#endif

static struct platform_driver tegra_i2c_driver = {
	.probe   = tegra_i2c_probe,
	.remove  = tegra_i2c_remove,
	.late_shutdown = tegra_i2c_shutdown,
	.driver  = {
		.name  = "tegra-i2c",
		.of_match_table = tegra_i2c_of_match,
		.pm    = TEGRA_I2C_PM,
	},
};

static int __init tegra_i2c_init_driver(void)
{
	return platform_driver_register(&tegra_i2c_driver);
}

static void __exit tegra_i2c_exit_driver(void)
{
	platform_driver_unregister(&tegra_i2c_driver);
}

subsys_initcall(tegra_i2c_init_driver);
module_exit(tegra_i2c_exit_driver);

MODULE_DESCRIPTION("nVidia Tegra2 I2C Bus Controller driver");
MODULE_AUTHOR("Colin Cross");
MODULE_LICENSE("GPL v2");
