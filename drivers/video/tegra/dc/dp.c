/*
 * drivers/video/tegra/dc/dp.c
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/tegra-soc.h>
#include <linux/clk/tegra.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <mach/dc.h>
#include <mach/fb.h>

#include "dp.h"
#include "sor.h"
#include "sor_regs.h"
#include "dpaux_regs.h"
#include "dc_priv.h"
#include "edid.h"

#ifdef CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT
#include "fake_panel.h"
#endif /*CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT*/

static bool tegra_dp_debug;
module_param(tegra_dp_debug, bool, 0644);
MODULE_PARM_DESC(tegra_dp_debug, "Enable to print all link configs");

static int tegra_dp_lt(struct tegra_dc_dp_data *dp);
static int tegra_dp_fast_lt(struct tegra_dc_dp_data *dp,
				struct tegra_dc_dp_link_config *cfg,
				bool handshake);
static void tegra_dp_link_config(struct tegra_dc_dp_data *dp);
static void tegra_dp_tpg(struct tegra_dc_dp_data *dp, u32 tp, u32 n_lanes);
static void tegra_dp_lt_config(struct tegra_dc_dp_data *dp,
				u32 pe[4], u32 vs[4], u32 pc[4]);
static bool tegra_dp_clock_recovery_status(struct tegra_dc_dp_data *dp);
static bool tegra_dp_channel_eq_status(struct tegra_dc_dp_data *dp);
static void tegra_dp_set_tx_pu(struct tegra_dc_dp_data *dp,
				u32 pe[4], u32 vs[4], u32 pc[4]);
static int tegra_dp_full_lt(struct tegra_dc_dp_data *dp);

static inline u32 tegra_dpaux_readl(struct tegra_dc_dp_data *dp, u32 reg)
{
	return readl(dp->aux_base + reg * 4);
}

static inline void tegra_dpaux_writel(struct tegra_dc_dp_data *dp,
	u32 reg, u32 val)
{
	writel(val, dp->aux_base + reg * 4);
}

static inline void tegra_dpaux_clk_enable(struct tegra_dc_dp_data *dp)
{
	clk_prepare_enable(dp->dpaux_clk);
}

static inline void tegra_dpaux_clk_disable(struct tegra_dc_dp_data *dp)
{
	clk_disable_unprepare(dp->dpaux_clk);
}

static inline void tegra_dp_clk_enable(struct tegra_dc_dp_data *dp)
{
	if (!tegra_is_clk_enabled(dp->parent_clk))
		clk_prepare_enable(dp->parent_clk);
}

static inline void tegra_dp_clk_disable(struct tegra_dc_dp_data *dp)
{
	if (tegra_is_clk_enabled(dp->parent_clk))
		clk_disable_unprepare(dp->parent_clk);
}

static inline void tegra_dpaux_write_field(struct tegra_dc_dp_data *dp,
					u32 reg, u32 mask, u32 val)
{
	u32 reg_val = tegra_dpaux_readl(dp, reg);
	reg_val &= ~mask;
	reg_val |= val;
	tegra_dpaux_writel(dp, reg, reg_val);
}

static inline void tegra_dp_int_en(struct tegra_dc_dp_data *dp, u32 intr)
{
	u32 val;

	/* clear pending interrupt */
	tegra_dpaux_writel(dp, DPAUX_INTR_AUX, intr);

	val = tegra_dpaux_readl(dp, DPAUX_INTR_EN_AUX);
	val |= intr;

	tegra_dpaux_writel(dp, DPAUX_INTR_EN_AUX, val);
}

static inline void tegra_dp_int_dis(struct tegra_dc_dp_data *dp, u32 intr)
{
	u32 val;

	val = tegra_dpaux_readl(dp, DPAUX_INTR_EN_AUX);
	val &= ~intr;

	tegra_dpaux_writel(dp, DPAUX_INTR_EN_AUX, val);
}

static inline void tegra_dp_enable_irq(u32 irq)
{
	if (tegra_platform_is_fpga())
		return;

	enable_irq(irq);
}

static inline void tegra_dp_disable_irq(u32 irq)
{
	if (tegra_platform_is_fpga())
		return;

	disable_irq(irq);
}

static inline unsigned long
tegra_dc_dpaux_poll_register(struct tegra_dc_dp_data *dp,
				u32 reg, u32 mask, u32 exp_val,
				u32 poll_interval_us, u32 timeout_ms)
{
	unsigned long	timeout_jf = jiffies + msecs_to_jiffies(timeout_ms);
	u32		reg_val	   = 0;

	do {
		usleep_range(poll_interval_us, poll_interval_us << 1);
		reg_val = tegra_dpaux_readl(dp, reg);
	} while (((reg_val & mask) != exp_val) &&
		time_after(timeout_jf, jiffies));

	if ((reg_val & mask) == exp_val)
		return 0;	/* success */
	dev_dbg(&dp->dc->ndev->dev,
		"dpaux_poll_register 0x%x: timeout\n", reg);
	return jiffies - timeout_jf + 1;
}

static inline int tegra_dpaux_wait_transaction(struct tegra_dc_dp_data *dp)
{
	int err = 0;

	if (unlikely(tegra_platform_is_fpga())) {
		if (tegra_dc_dpaux_poll_register(dp, DPAUX_DP_AUXCTL,
				DPAUX_DP_AUXCTL_TRANSACTREQ_MASK,
				DPAUX_DP_AUXCTL_TRANSACTREQ_DONE,
				100, DP_AUX_TIMEOUT_MS) != 0)
			err = -EFAULT;
	}

	if (likely(tegra_platform_is_silicon())) {
		reinit_completion(&dp->aux_tx);
		tegra_dp_int_en(dp, DPAUX_INTR_EN_AUX_TX_DONE);
		if (tegra_dpaux_readl(dp, DPAUX_DP_AUXCTL) &
				DPAUX_DP_AUXCTL_TRANSACTREQ_PENDING) {
			if (!wait_for_completion_timeout(&dp->aux_tx,
				msecs_to_jiffies(DP_AUX_TIMEOUT_MS)))
				err = -EBUSY;
		}
		tegra_dp_int_dis(dp, DPAUX_INTR_EN_AUX_TX_DONE);
	}

	if (err)
		dev_err(&dp->dc->ndev->dev, "dp: aux tx timeout\n");
	return err;
}

static int tegra_dp_aux_tx_config(struct tegra_dc_dp_data *dp,
				u32 cmd, u32 addr, bool addr_only,
				u32 data[], u32 size)
{
	int i;

	if (size > DP_AUX_MAX_BYTES)
		goto fail;

	switch (cmd) {
	case DPAUX_DP_AUXCTL_CMD_I2CWR:
	case DPAUX_DP_AUXCTL_CMD_I2CRD:
	case DPAUX_DP_AUXCTL_CMD_I2CREQWSTAT:
	case DPAUX_DP_AUXCTL_CMD_MOTWR:
	case DPAUX_DP_AUXCTL_CMD_MOTRD:
	case DPAUX_DP_AUXCTL_CMD_MOTREQWSTAT:
	case DPAUX_DP_AUXCTL_CMD_AUXWR:
	case DPAUX_DP_AUXCTL_CMD_AUXRD:
		tegra_dpaux_write_field(dp, DPAUX_DP_AUXCTL,
					DPAUX_DP_AUXCTL_CMD_MASK, cmd);
		break;
	default:
		goto fail;
	};

	tegra_dpaux_write_field(dp, DPAUX_DP_AUXCTL,
				DPAUX_DP_AUXCTL_CMDLEN_MASK,
				size);

	tegra_dpaux_write_field(dp, DPAUX_DP_AUXCTL,
				DPAUX_DP_AUXCTL_ADDRESS_ONLY_MASK,
				(addr_only ? DPAUX_DP_AUXCTL_ADDRESS_ONLY_TRUE :
				DPAUX_DP_AUXCTL_ADDRESS_ONLY_FALSE));

	tegra_dpaux_writel(dp, DPAUX_DP_AUXADDR, addr);
	for (i = 0; data && i < (DP_AUX_MAX_BYTES / 4); ++i)
		tegra_dpaux_writel(dp, DPAUX_DP_AUXDATA_WRITE_W(i), data[i]);

	return 0;
fail:
	return -EINVAL;
}

static int tegra_dc_dpaux_write_chunk_locked(struct tegra_dc_dp_data *dp,
	u32 cmd, u32 addr, u8 *data, u32 *size, u32 *aux_stat)
{
	int err = 0;
	u32 timeout_retries = DP_AUX_TIMEOUT_MAX_TRIES;
	u32 defer_retries	= DP_AUX_DEFER_MAX_TRIES;

	WARN_ON(!mutex_is_locked(&dp->dpaux_lock));

	switch (cmd) {
	case DPAUX_DP_AUXCTL_CMD_I2CWR:
	case DPAUX_DP_AUXCTL_CMD_MOTWR:
	case DPAUX_DP_AUXCTL_CMD_AUXWR:
		break;
	default:
		dev_err(&dp->dc->ndev->dev,
			"dp: invalid aux write cmd: 0x%x\n", cmd);
		return -EINVAL;
	};

	err = tegra_dp_aux_tx_config(dp, cmd, addr, false, (u32 *)data, *size);
	if (err < 0) {
		dev_err(&dp->dc->ndev->dev, "dp: incorrect aux tx params\n");
		return err;
	}

	if (tegra_platform_is_silicon()) {
		*aux_stat = tegra_dpaux_readl(dp, DPAUX_DP_AUXSTAT);
		if (!(*aux_stat & DPAUX_DP_AUXSTAT_HPD_STATUS_PLUGGED)) {
			dev_err(&dp->dc->ndev->dev, "dp: HPD is not detected\n");
			return -EFAULT;
		}
	}

	while ((timeout_retries > 0) && (defer_retries > 0)) {
		if ((timeout_retries != DP_AUX_TIMEOUT_MAX_TRIES) ||
		    (defer_retries != DP_AUX_DEFER_MAX_TRIES))
			usleep_range(DP_DPCP_RETRY_SLEEP_NS,
				DP_DPCP_RETRY_SLEEP_NS << 1);

		tegra_dpaux_write_field(dp, DPAUX_DP_AUXCTL,
					DPAUX_DP_AUXCTL_TRANSACTREQ_MASK,
					DPAUX_DP_AUXCTL_TRANSACTREQ_PENDING);

		if (tegra_dpaux_wait_transaction(dp))
			dev_err(&dp->dc->ndev->dev,
				"dp: aux write transaction timeout\n");

		*aux_stat = tegra_dpaux_readl(dp, DPAUX_DP_AUXSTAT);

		/* Ignore I2C errors on fpga */
		if (tegra_platform_is_fpga())
			*aux_stat &= ~DPAUX_DP_AUXSTAT_REPLYTYPE_I2CNACK;

		if ((*aux_stat & DPAUX_DP_AUXSTAT_TIMEOUT_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_RX_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_SINKSTAT_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_NO_STOP_ERROR_PENDING)) {
			if (timeout_retries-- > 0) {
				dev_dbg(&dp->dc->ndev->dev,
					"dp: aux write retry (0x%x) -- %d\n",
					*aux_stat, timeout_retries);
				/* clear the error bits */
				tegra_dpaux_writel(dp, DPAUX_DP_AUXSTAT,
					*aux_stat);
				continue;
			} else {
				dev_err(&dp->dc->ndev->dev,
					"dp: aux write got error (0x%x)\n",
					*aux_stat);
				return -EFAULT;
			}
		}

		if ((*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_I2CDEFER) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_DEFER)) {
			if (defer_retries-- > 0) {
				dev_dbg(&dp->dc->ndev->dev,
					"dp: aux write defer (0x%x) -- %d\n",
					*aux_stat, defer_retries);
				/* clear the error bits */
				tegra_dpaux_writel(dp, DPAUX_DP_AUXSTAT,
					*aux_stat);
				continue;
			} else {
				dev_err(&dp->dc->ndev->dev,
					"dp: aux write defer exceeds max retries "
					"(0x%x)\n",
					*aux_stat);
				return -EFAULT;
			}
		}

		if ((*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_MASK) ==
			DPAUX_DP_AUXSTAT_REPLYTYPE_ACK) {
			(*size)++;
			return 0;
		} else {
			dev_err(&dp->dc->ndev->dev,
				"dp: aux write failed (0x%x)\n", *aux_stat);
			return -EFAULT;
		}
	}
	/* Should never come to here */
	return -EFAULT;
}

int tegra_dc_dpaux_write(struct tegra_dc_dp_data *dp, u32 cmd, u32 addr,
	u8 *data, u32 *size, u32 *aux_stat)
{
	u32	cur_size = 0;
	u32	finished = 0;
	int	ret	 = 0;

	if (*size == 0) {
		dev_err(&dp->dc->ndev->dev,
			"dp: aux write size can't be 0\n");
		return -EINVAL;
	}

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return ret;

	mutex_lock(&dp->dpaux_lock);
	do {
		cur_size = *size - finished;
		if (cur_size >= DP_AUX_MAX_BYTES)
			cur_size = DP_AUX_MAX_BYTES - 1;
		else
			cur_size -= 1;

		ret = tegra_dc_dpaux_write_chunk_locked(dp, cmd, addr,
			data, &cur_size, aux_stat);

		finished += cur_size;
		addr += cur_size;
		data += cur_size;

		if (ret)
			break;
	} while (*size > finished);
	mutex_unlock(&dp->dpaux_lock);

	*size = finished;
	return ret;
}

static int tegra_dc_dpaux_read_chunk_locked(struct tegra_dc_dp_data *dp,
	u32 cmd, u32 addr, u8 *data, u32 *size, u32 *aux_stat)
{
	int err = 0;
	u32 timeout_retries = DP_AUX_TIMEOUT_MAX_TRIES;
	u32 defer_retries	= DP_AUX_DEFER_MAX_TRIES;

	WARN_ON(!mutex_is_locked(&dp->dpaux_lock));

	switch (cmd) {
	case DPAUX_DP_AUXCTL_CMD_I2CRD:
	case DPAUX_DP_AUXCTL_CMD_I2CREQWSTAT:
	case DPAUX_DP_AUXCTL_CMD_MOTREQWSTAT:
	case DPAUX_DP_AUXCTL_CMD_MOTRD:
	case DPAUX_DP_AUXCTL_CMD_AUXRD:
		break;
	default:
		dev_err(&dp->dc->ndev->dev,
			"dp: invalid aux read cmd: 0x%x\n", cmd);
		return -EINVAL;
	};

	err = tegra_dp_aux_tx_config(dp, cmd, addr, false, NULL, *size);
	if (err < 0) {
		dev_err(&dp->dc->ndev->dev, "dp: incorrect aux tx params\n");
		return err;
	}

	if (tegra_platform_is_silicon()) {
		*aux_stat = tegra_dpaux_readl(dp, DPAUX_DP_AUXSTAT);
		if (!(*aux_stat & DPAUX_DP_AUXSTAT_HPD_STATUS_PLUGGED)) {
			dev_err(&dp->dc->ndev->dev, "dp: HPD is not detected\n");
			return -EFAULT;
		}
	}

	while ((timeout_retries > 0) && (defer_retries > 0)) {
		if ((timeout_retries != DP_AUX_TIMEOUT_MAX_TRIES) ||
		    (defer_retries != DP_AUX_DEFER_MAX_TRIES))
			usleep_range(DP_DPCP_RETRY_SLEEP_NS,
				DP_DPCP_RETRY_SLEEP_NS << 1);

		tegra_dpaux_write_field(dp, DPAUX_DP_AUXCTL,
					DPAUX_DP_AUXCTL_TRANSACTREQ_MASK,
					DPAUX_DP_AUXCTL_TRANSACTREQ_PENDING);

		if (tegra_dpaux_wait_transaction(dp))
			dev_err(&dp->dc->ndev->dev,
				"dp: aux read transaction timeout\n");

		*aux_stat = tegra_dpaux_readl(dp, DPAUX_DP_AUXSTAT);

		/* Ignore I2C errors on fpga */
		if (tegra_platform_is_fpga())
			*aux_stat &= ~DPAUX_DP_AUXSTAT_REPLYTYPE_I2CNACK;

		if ((*aux_stat & DPAUX_DP_AUXSTAT_TIMEOUT_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_RX_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_SINKSTAT_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_NO_STOP_ERROR_PENDING)) {
			if (timeout_retries-- > 0) {
				dev_dbg(&dp->dc->ndev->dev,
					"dp: aux read retry (0x%x) -- %d\n",
					*aux_stat, timeout_retries);
				/* clear the error bits */
				tegra_dpaux_writel(dp, DPAUX_DP_AUXSTAT,
					*aux_stat);
				continue; /* retry */
			} else {
				dev_err(&dp->dc->ndev->dev,
					"dp: aux read got error (0x%x)\n",
					*aux_stat);
				return -EFAULT;
			}
		}

		if ((*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_I2CDEFER) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_DEFER)) {
			if (defer_retries-- > 0) {
				dev_dbg(&dp->dc->ndev->dev,
					"dp: aux read defer (0x%x) -- %d\n",
					*aux_stat, defer_retries);
				/* clear the error bits */
				tegra_dpaux_writel(dp, DPAUX_DP_AUXSTAT,
					*aux_stat);
				continue;
			} else {
				dev_err(&dp->dc->ndev->dev,
					"dp: aux read defer exceeds max retries "
					"(0x%x)\n", *aux_stat);
				return -EFAULT;
			}
		}

		if ((*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_MASK) ==
			DPAUX_DP_AUXSTAT_REPLYTYPE_ACK) {
			int i;
			u32 temp_data[4];

			for (i = 0; i < DP_AUX_MAX_BYTES/4; ++i)
				temp_data[i] = tegra_dpaux_readl(dp,
					DPAUX_DP_AUXDATA_READ_W(i));

			*size = ((*aux_stat) & DPAUX_DP_AUXSTAT_REPLY_M_MASK);
			memcpy(data, temp_data, *size);

			return 0;
		} else {
			dev_err(&dp->dc->ndev->dev,
				"dp: aux read failed (0x%x\n", *aux_stat);
			return -EFAULT;
		}
	}
	/* Should never come to here */
	return -EFAULT;
}

int tegra_dc_dpaux_read(struct tegra_dc_dp_data *dp, u32 cmd, u32 addr,
	u8 *data, u32 *size, u32 *aux_stat)
{
	u32	finished = 0;
	u32	cur_size;
	int	ret	 = 0;

	if (*size == 0) {
		dev_err(&dp->dc->ndev->dev,
			"dp: aux read size can't be 0\n");
		return -EINVAL;
	}

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return  ret;

	mutex_lock(&dp->dpaux_lock);
	do {
		cur_size = *size - finished;
		if (cur_size >= DP_AUX_MAX_BYTES)
			cur_size = DP_AUX_MAX_BYTES - 1;
		else
			cur_size -= 1;

		ret = tegra_dc_dpaux_read_chunk_locked(dp, cmd, addr,
			data, &cur_size, aux_stat);

		if (ret)
			break;

		/* cur_size should be the real size returned */
		addr += cur_size;
		data += cur_size;
		finished += cur_size;

	} while (*size > finished);
	mutex_unlock(&dp->dpaux_lock);

	*size = finished;
	return ret;
}

/* I2C read over DPAUX cannot handle more than 16B per transaction due to
 * DPAUX transaction limitation.
 * This requires breaking each read into multiple i2c write/read transaction */
static int tegra_dc_i2c_read(struct tegra_dc_dp_data *dp, u32 i2c_addr,
	u32 addr, u8 *data, u32 *size, u32 *aux_stat)
{
	u32	finished = 0;
	u32	cur_size;
	int	ret	 = 0;
	u32	len;
	u8	iaddr	 = (u8)addr;

	if (*size == 0) {
		dev_err(&dp->dc->ndev->dev,
			"dp: i2c read size can't be 0\n");
		return -EINVAL;
	}

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return ret;

	mutex_lock(&dp->dpaux_lock);
	do {
		cur_size = *size - finished;
		if (cur_size >= DP_AUX_MAX_BYTES)
			cur_size = DP_AUX_MAX_BYTES - 1;
		else
			cur_size -= 1;

		len = 0;
		CHECK_RET(tegra_dc_dpaux_write_chunk_locked(dp,
				DPAUX_DP_AUXCTL_CMD_I2CWR,
				i2c_addr, &iaddr, &len, aux_stat));
		CHECK_RET(tegra_dc_dpaux_read_chunk_locked(dp,
				DPAUX_DP_AUXCTL_CMD_I2CRD,
				i2c_addr, data, &cur_size, aux_stat));

		iaddr += cur_size;
		data += cur_size;
		finished += cur_size;
	} while (*size > finished);
	mutex_unlock(&dp->dpaux_lock);

	*size = finished;
	return ret;
}

static int tegra_dc_dp_dpcd_read(struct tegra_dc_dp_data *dp, u32 cmd,
	u8 *data_ptr)
{
	u32 size = 0;
	u32 status = 0;
	int ret = 0;

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return ret;

	mutex_lock(&dp->dpaux_lock);
	ret = tegra_dc_dpaux_read_chunk_locked(dp, DPAUX_DP_AUXCTL_CMD_AUXRD,
		cmd, data_ptr, &size, &status);
	mutex_unlock(&dp->dpaux_lock);
	if (ret)
		dev_err(&dp->dc->ndev->dev,
			"dp: Failed to read DPCD data. CMD 0x%x, Status 0x%x\n",
			cmd, status);

	return ret;
}

static int tegra_dc_dp_i2c_xfer(struct tegra_dc *dc, struct i2c_msg *msgs,
	int num)
{
	struct i2c_msg *pmsg;
	int i;
	u32 aux_stat;
	int status = 0;
	u32 len = 0;
	u32 start_addr;
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	for (i = 0; i < num; ++i) {
		pmsg = &msgs[i];

		if (!pmsg->flags) { /* write */
			/* Ignore the write-for-read command now as it is
			   already handled in the read operations */
		} else if (pmsg->flags & I2C_M_RD) { /* Read */
			len = pmsg->len;
			start_addr = 0;
			status = tegra_dc_i2c_read(dp, pmsg->addr, start_addr,
				pmsg->buf, &len, &aux_stat);
			if (status) {
				dev_err(&dp->dc->ndev->dev,
					"dp: Failed for I2C read"
					" addr:%d, size:%d, stat:0x%x\n",
					pmsg->addr, len, aux_stat);
				return status;
			}
		} else {
			/* No other functionalities are supported for now */
			dev_err(&dp->dc->ndev->dev,
				"dp: i2x_xfer: Unknown flag 0x%x\n",
				pmsg->flags);
			return -EINVAL;
		}
	}
	return i;
}


static int tegra_dc_dp_dpcd_write(struct tegra_dc_dp_data *dp, u32 cmd,
	u8 data)
{
	u32 size = 0;
	u32 status = 0;
	int ret;

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return 0;

	mutex_lock(&dp->dpaux_lock);
	ret = tegra_dc_dpaux_write_chunk_locked(dp, DPAUX_DP_AUXCTL_CMD_AUXWR,
		cmd, &data, &size, &status);
	mutex_unlock(&dp->dpaux_lock);
	if (ret)
		dev_err(&dp->dc->ndev->dev,
			"dp: Failed to write DPCD data. CMD 0x%x, Status 0x%x\n",
			cmd, status);
	return ret;
}

static inline int tegra_dp_dpcd_write_field(struct tegra_dc_dp_data *dp,
					u32 cmd, u8 mask, u8 data)
{
	u8 dpcd_data;
	int ret;

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return 0;

	might_sleep();

	CHECK_RET(tegra_dc_dp_dpcd_read(dp, cmd, &dpcd_data));
	dpcd_data &= ~mask;
	dpcd_data |= data;
	CHECK_RET(tegra_dc_dp_dpcd_write(dp, cmd, dpcd_data));

	return 0;
}

static inline u64 tegra_div64(u64 dividend, u32 divisor)
{
	do_div(dividend, divisor);
	return dividend;
}


#ifdef CONFIG_DEBUG_FS
static int dbg_dp_show(struct seq_file *s, void *unused)
{
	struct tegra_dc_dp_data *dp = s->private;

#define DUMP_REG(a) seq_printf(s, "%-32s  %03x	%08x\n",	\
		#a, a, tegra_dpaux_readl(dp, a))

	tegra_dc_io_start(dp->dc);
	tegra_dpaux_clk_enable(dp);

	DUMP_REG(DPAUX_INTR_EN_AUX);
	DUMP_REG(DPAUX_INTR_AUX);
	DUMP_REG(DPAUX_DP_AUXADDR);
	DUMP_REG(DPAUX_DP_AUXCTL);
	DUMP_REG(DPAUX_DP_AUXSTAT);
	DUMP_REG(DPAUX_HPD_CONFIG);
	DUMP_REG(DPAUX_HPD_IRQ_CONFIG);
	DUMP_REG(DPAUX_DP_AUX_CONFIG);
	DUMP_REG(DPAUX_HYBRID_PADCTL);
	DUMP_REG(DPAUX_HYBRID_SPARE);

	tegra_dpaux_clk_disable(dp);
	tegra_dc_io_end(dp->dc);

	return 0;
}

static int dbg_dp_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dp_show, inode->i_private);
}

static const struct file_operations dbg_fops = {
	.open		= dbg_dp_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct dentry *dpdir;

static void tegra_dc_dp_debug_create(struct tegra_dc_dp_data *dp)
{
	struct dentry *retval;

	dpdir = debugfs_create_dir("tegra_dp", NULL);
	if (!dpdir)
		return;
	retval = debugfs_create_file("regs", S_IRUGO, dpdir, dp, &dbg_fops);
	if (!retval)
		goto free_out;
	return;
free_out:
	debugfs_remove_recursive(dpdir);
	dpdir = NULL;
	return;
}
#else
static inline void tegra_dc_dp_debug_create(struct tegra_dc_dp_data *dp)
{ }
#endif

static void tegra_dpaux_enable(struct tegra_dc_dp_data *dp)
{
	/* do not enable interrupt for now. */
	tegra_dpaux_writel(dp, DPAUX_INTR_EN_AUX, 0x0);

	/* clear interrupt */
	tegra_dpaux_writel(dp, DPAUX_INTR_AUX, 0xffffffff);

	tegra_dpaux_writel(dp, DPAUX_HYBRID_PADCTL,
		DPAUX_HYBRID_PADCTL_AUX_DRVZ_OHM_50 |
		DPAUX_HYBRID_PADCTL_AUX_CMH_V0_70 |
		0x18 << DPAUX_HYBRID_PADCTL_AUX_DRVI_SHIFT |
		DPAUX_HYBRID_PADCTL_AUX_INPUT_RCV_ENABLE);

	tegra_dpaux_pad_power(dp->dc, true);
}

static int tegra_dp_panel_power_state(struct tegra_dc_dp_data *dp, u8 state)
{
	u32 retry = 0;
	int ret;

	do {
		ret = tegra_dc_dp_dpcd_write(dp, NV_DPCD_SET_POWER, state);
	} while ((retry++ < DP_POWER_ON_MAX_TRIES) && ret);

	return ret;
}

static void tegra_dc_dp_dump_link_cfg(struct tegra_dc_dp_data *dp,
	const struct tegra_dc_dp_link_config *cfg)
{
	if (!tegra_dp_debug)
		return;

	BUG_ON(!cfg);

	dev_info(&dp->dc->ndev->dev, "DP config: cfg_name               "
		"cfg_value\n");
	dev_info(&dp->dc->ndev->dev, "           Lane Count             %d\n",
		cfg->max_lane_count);
	dev_info(&dp->dc->ndev->dev, "           SupportEnhancedFraming %s\n",
		cfg->support_enhanced_framing ? "Y" : "N");
	dev_info(&dp->dc->ndev->dev, "           SupportAltScrmbRstFffe %s\n",
		cfg->alt_scramber_reset_cap ? "Y" : "N");
	dev_info(&dp->dc->ndev->dev, "           Bandwidth              %d\n",
		cfg->max_link_bw);
	dev_info(&dp->dc->ndev->dev, "           bpp                    %d\n",
		cfg->bits_per_pixel);
	dev_info(&dp->dc->ndev->dev, "           EnhancedFraming        %s\n",
		cfg->enhanced_framing ? "Y" : "N");
	dev_info(&dp->dc->ndev->dev, "           Scramble_enabled       %s\n",
		cfg->scramble_ena ? "Y" : "N");
	dev_info(&dp->dc->ndev->dev, "           LinkBW                 %d\n",
		cfg->link_bw);
	dev_info(&dp->dc->ndev->dev, "           lane_count             %d\n",
		cfg->lane_count);
	dev_info(&dp->dc->ndev->dev, "           activespolarity        %d\n",
		cfg->activepolarity);
	dev_info(&dp->dc->ndev->dev, "           active_count           %d\n",
		cfg->active_count);
	dev_info(&dp->dc->ndev->dev, "           tu_size                %d\n",
		cfg->tu_size);
	dev_info(&dp->dc->ndev->dev, "           active_frac            %d\n",
		cfg->active_frac);
	dev_info(&dp->dc->ndev->dev, "           watermark              %d\n",
		cfg->watermark);
	dev_info(&dp->dc->ndev->dev, "           hblank_sym             %d\n",
		cfg->hblank_sym);
	dev_info(&dp->dc->ndev->dev, "           vblank_sym             %d\n",
		cfg->vblank_sym);
};

/* Calcuate if given cfg can meet the mode request. */
/* Return true if mode is possible, false otherwise. */
static bool tegra_dc_dp_calc_config(struct tegra_dc_dp_data *dp,
	const struct tegra_dc_mode *mode,
	struct tegra_dc_dp_link_config *cfg)
{
	const u32	link_rate = 27 * cfg->link_bw * 1000 * 1000;
	const u64	f	  = 100000;	/* precision factor */

	u32	num_linkclk_line; /* Number of link clocks per line */
	u64	ratio_f; /* Ratio of incoming to outgoing data rate */
	u64	frac_f;
	u64	activesym_f;	/* Activesym per TU */
	u64	activecount_f;
	u32	activecount;
	u32	activepolarity;
	u64	approx_value_f;
	u32	activefrac		  = 0;
	u64	accumulated_error_f	  = 0;
	u32	lowest_neg_activecount	  = 0;
	u32	lowest_neg_activepolarity = 0;
	u32	lowest_neg_tusize	  = 64;
	u32	num_symbols_per_line;
	u64	lowest_neg_activefrac	  = 0;
	u64	lowest_neg_error_f	  = 64 * f;
	u64	watermark_f;

	int	i;
	bool	neg;
	unsigned long rate;

	cfg->is_valid = false;

	rate = tegra_dc_pclk_round_rate(dp->sor->dc, dp->sor->dc->mode.pclk);

	if (!link_rate || !cfg->lane_count || !rate ||
		!cfg->bits_per_pixel)
		return false;

	if ((u64)rate * cfg->bits_per_pixel >=
		(u64)link_rate * 8 * cfg->lane_count)
		return false;

	num_linkclk_line = (u32)tegra_div64(
		(u64)link_rate * mode->h_active, rate);

	ratio_f = (u64)rate * cfg->bits_per_pixel * f;
	ratio_f /= 8;
	ratio_f = tegra_div64(ratio_f, link_rate * cfg->lane_count);

	for (i = 64; i >= 32; --i) {
		activesym_f	= ratio_f * i;
		activecount_f	= tegra_div64(activesym_f, (u32)f) * f;
		frac_f		= activesym_f - activecount_f;
		activecount	= (u32)tegra_div64(activecount_f, (u32)f);

		if (frac_f < (f / 2)) /* fraction < 0.5 */
			activepolarity = 0;
		else {
			activepolarity = 1;
			frac_f = f - frac_f;
		}

		if (frac_f != 0) {
			frac_f = tegra_div64((f * f),  frac_f); /* 1/fraction */
			if (frac_f > (15 * f))
				activefrac = activepolarity ? 1 : 15;
			else
				activefrac = activepolarity ?
					(u32)tegra_div64(frac_f, (u32)f) + 1 :
					(u32)tegra_div64(frac_f, (u32)f);
		}

		if (activefrac == 1)
			activepolarity = 0;

		if (activepolarity == 1)
			approx_value_f = activefrac ? tegra_div64(
				activecount_f + (activefrac * f - f) * f,
				(activefrac * f)) :
				activecount_f + f;
		else
			approx_value_f = activefrac ?
				activecount_f + tegra_div64(f, activefrac) :
				activecount_f;

		if (activesym_f < approx_value_f) {
			accumulated_error_f = num_linkclk_line *
				tegra_div64(approx_value_f - activesym_f, i);
			neg = true;
		} else {
			accumulated_error_f = num_linkclk_line *
				tegra_div64(activesym_f - approx_value_f, i);
			neg = false;
		}

		if ((neg && (lowest_neg_error_f > accumulated_error_f)) ||
			(accumulated_error_f == 0)) {
			lowest_neg_error_f = accumulated_error_f;
			lowest_neg_tusize = i;
			lowest_neg_activecount = activecount;
			lowest_neg_activepolarity = activepolarity;
			lowest_neg_activefrac = activefrac;

			if (accumulated_error_f == 0)
				break;
		}
	}

	if (lowest_neg_activefrac == 0) {
		cfg->activepolarity = 0;
		cfg->active_count   = lowest_neg_activepolarity ?
			lowest_neg_activecount : lowest_neg_activecount - 1;
		cfg->tu_size	      = lowest_neg_tusize;
		cfg->active_frac    = 1;
	} else {
		cfg->activepolarity = lowest_neg_activepolarity;
		cfg->active_count   = (u32)lowest_neg_activecount;
		cfg->tu_size	      = lowest_neg_tusize;
		cfg->active_frac    = (u32)lowest_neg_activefrac;
	}

	dev_dbg(&dp->dc->ndev->dev,
		"dp: sor configuration: polarity: %d active count: %d "
		"tu size: %d, active frac: %d\n",
		cfg->activepolarity, cfg->active_count, cfg->tu_size,
		cfg->active_frac);

	watermark_f = tegra_div64(ratio_f * cfg->tu_size * (f - ratio_f), f);
	cfg->watermark = (u32)tegra_div64(watermark_f + lowest_neg_error_f,
		f) + cfg->bits_per_pixel / 4 - 1;
	num_symbols_per_line = (mode->h_active * cfg->bits_per_pixel) /
		(8 * cfg->lane_count);
	if (cfg->watermark > 30) {
		dev_dbg(&dp->dc->ndev->dev,
			"dp: sor setting: unable to get a good tusize, "
			"force watermark to 30.\n");
		cfg->watermark = 30;
		return false;
	} else if (cfg->watermark > num_symbols_per_line) {
		dev_dbg(&dp->dc->ndev->dev,
			"dp: sor setting: force watermark to the number "
			"of symbols in the line.\n");
		cfg->watermark = num_symbols_per_line;
		return false;
	}

	/* Refer to dev_disp.ref for more information. */
	/* # symbols/hblank = ((SetRasterBlankEnd.X + SetRasterSize.Width - */
	/*                      SetRasterBlankStart.X - 7) * link_clk / pclk) */
	/*                      - 3 * enhanced_framing - Y */
	/* where Y = (# lanes == 4) 3 : (# lanes == 2) ? 6 : 12 */
	cfg->hblank_sym = (int)tegra_div64((u64)(mode->h_back_porch +
			mode->h_front_porch + mode->h_sync_width - 7)
		* link_rate, rate)
		- 3 * cfg->enhanced_framing - (12 / cfg->lane_count);

	if (cfg->hblank_sym < 0)
		cfg->hblank_sym = 0;


	/* Refer to dev_disp.ref for more information. */
	/* # symbols/vblank = ((SetRasterBlankStart.X - */
	/*                      SetRasterBlankEen.X - 25) * link_clk / pclk) */
	/*                      - Y - 1; */
	/* where Y = (# lanes == 4) 12 : (# lanes == 2) ? 21 : 39 */
	cfg->vblank_sym = (int)tegra_div64((u64)(mode->h_active - 25)
		* link_rate, rate) - (36 / cfg->lane_count) - 4;

	if (cfg->vblank_sym < 0)
		cfg->vblank_sym = 0;

	cfg->is_valid = true;

	tegra_dc_dp_dump_link_cfg(dp, cfg);

	return true;
}

static int tegra_dp_init_max_link_cfg(struct tegra_dc_dp_data *dp,
					struct tegra_dc_dp_link_config *cfg)
{

#ifdef CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT
	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		tegra_dc_init_fake_panel_link_cfg(cfg);
	else
#endif
	 {
		u8 dpcd_data;
		int ret;

		CHECK_RET(tegra_dc_dp_dpcd_read(dp, NV_DPCD_MAX_LANE_COUNT,
			&dpcd_data));

		cfg->max_lane_count = dpcd_data & NV_DPCD_MAX_LANE_COUNT_MASK;
		cfg->tps3_supported =
		(dpcd_data & NV_DPCD_MAX_LANE_COUNT_TPS3_SUPPORTED_YES) ?
		true : false;

		cfg->support_enhanced_framing =
		(dpcd_data & NV_DPCD_MAX_LANE_COUNT_ENHANCED_FRAMING_YES) ?
		true : false;

		CHECK_RET(tegra_dc_dp_dpcd_read(dp, NV_DPCD_MAX_DOWNSPREAD,
			&dpcd_data));
		cfg->downspread =
		(dpcd_data & NV_DPCD_MAX_DOWNSPREAD_VAL_0_5_PCT) ?
		true : false;

		cfg->support_fast_lt = (dpcd_data &
		NV_DPCD_MAX_DOWNSPREAD_NO_AUX_HANDSHAKE_LT_T) ? true : false;

		CHECK_RET(tegra_dc_dp_dpcd_read(dp,
			NV_DPCD_TRAINING_AUX_RD_INTERVAL, &dpcd_data));
		cfg->aux_rd_interval = dpcd_data;

		CHECK_RET(tegra_dc_dp_dpcd_read(dp, NV_DPCD_MAX_LINK_BANDWIDTH,
			&cfg->max_link_bw));

		CHECK_RET(tegra_dc_dp_dpcd_read(dp, NV_DPCD_EDP_CONFIG_CAP,
			&dpcd_data));
		cfg->alt_scramber_reset_cap =
		(dpcd_data & NV_DPCD_EDP_CONFIG_CAP_ASC_RESET_YES) ?
		true : false;

		cfg->only_enhanced_framing =
		(dpcd_data & NV_DPCD_EDP_CONFIG_CAP_FRAMING_CHANGE_YES) ?
		true : false;

		cfg->edp_cap = (dpcd_data &
		NV_DPCD_EDP_CONFIG_CAP_DISPLAY_CONTROL_CAP_YES) ? true : false;
	}

	/* DP SOR supports either 18bpp or 24bpp out stream only */
	cfg->bits_per_pixel = (18 < dp->dc->out->depth) ? 24 : 18;

	cfg->lane_count = cfg->max_lane_count;

	cfg->link_bw = (dp->pdata && dp->pdata->link_bw) ?
			dp->pdata->link_bw : cfg->max_link_bw;

	cfg->enhanced_framing = cfg->support_enhanced_framing;

	tegra_dc_dp_calc_config(dp, dp->mode, cfg);

	dp->max_link_cfg = *cfg;
	return 0;
}

static int tegra_dc_dp_set_assr(struct tegra_dc_dp_data *dp, bool ena)
{
	int ret;

	u8 dpcd_data = ena ?
		NV_DPCD_EDP_CONFIG_SET_ASC_RESET_ENABLE :
		NV_DPCD_EDP_CONFIG_SET_ASC_RESET_DISABLE;

	CHECK_RET(tegra_dc_dp_dpcd_write(dp, NV_DPCD_EDP_CONFIG_SET,
			dpcd_data));

	/* Also reset the scrambler to 0xfffe */
	tegra_dc_sor_set_internal_panel(dp->sor, ena);
	return 0;
}


static int tegra_dp_set_link_bandwidth(struct tegra_dc_dp_data *dp, u8 link_bw)
{
	tegra_dc_sor_set_link_bandwidth(dp->sor, link_bw);

	/* Sink side */
	return tegra_dc_dp_dpcd_write(dp, NV_DPCD_LINK_BANDWIDTH_SET, link_bw);
}

static int tegra_dp_set_enhanced_framing(struct tegra_dc_dp_data *dp,
						bool enable)
{
	int ret;

	if (enable) {
		tegra_sor_write_field(dp->sor,
			NV_SOR_DP_LINKCTL(dp->sor->portnum),
			NV_SOR_DP_LINKCTL_ENHANCEDFRAME_ENABLE,
			NV_SOR_DP_LINKCTL_ENHANCEDFRAME_ENABLE);

		CHECK_RET(tegra_dp_dpcd_write_field(dp, NV_DPCD_LANE_COUNT_SET,
				NV_DPCD_LANE_COUNT_SET_ENHANCEDFRAMING_T,
				NV_DPCD_LANE_COUNT_SET_ENHANCEDFRAMING_T));
	}

	return 0;
}

static int tegra_dp_set_lane_count(struct tegra_dc_dp_data *dp, u8 lane_cnt)
{
	int ret;

	tegra_sor_power_lanes(dp->sor, lane_cnt, true);

	CHECK_RET(tegra_dp_dpcd_write_field(dp, NV_DPCD_LANE_COUNT_SET,
				NV_DPCD_LANE_COUNT_SET_MASK,
				lane_cnt));

	return 0;
}

static bool tegra_dp_lt_status(struct tegra_dc_dp_data *dp)
{
	bool cr_done, ce_done;

	cr_done = tegra_dp_clock_recovery_status(dp);
	if (!cr_done)
		return false;

	ce_done = tegra_dp_channel_eq_status(dp);

	return ce_done;
}

static void tegra_dp_link_cal(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_sor_data *sor = dp->sor;
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	u32 load_adj;

	switch (cfg->link_bw) {
	case SOR_LINK_SPEED_G1_62:
		load_adj = 0x3;
		break;
	case SOR_LINK_SPEED_G2_7:
		load_adj = 0x4;
		break;
	case SOR_LINK_SPEED_G5_4:
		load_adj = 0x6;
		break;
	default:
		BUG();
	}

	tegra_sor_write_field(sor, NV_SOR_PLL1,
			NV_SOR_PLL1_LOADADJ_DEFAULT_MASK,
			load_adj << NV_SOR_PLL1_LOADADJ_SHIFT);
}

static int _tegra_dp_fast_lt(struct tegra_dc_dp_data *dp,
				struct tegra_dc_dp_link_config *cfg,
				bool handshake)
{
	bool cr_done;
	bool lt_done;
	int ret = 0;
	u8 n_lanes = cfg->lane_count;

	if (!handshake) {
		tegra_sor_tpg(dp->sor, TRAINING_PATTERN_1, n_lanes);
		usleep_range(500, 600);

		if (cfg->tps3_supported)
			tegra_sor_tpg(dp->sor, TRAINING_PATTERN_3, n_lanes);
		else
			tegra_sor_tpg(dp->sor, TRAINING_PATTERN_2, n_lanes);
		usleep_range(500, 600);

		tegra_sor_tpg(dp->sor, TRAINING_PATTERN_DISABLE, n_lanes);

		/*
		 * If no handshake we have no means to identify immediately
		 * if lt passed. If loss of synchronization,
		 * hdp_irq will be raised.
		 */
		cfg->lt_data_valid = true;

		tegra_dc_dp_dump_link_cfg(dp, cfg);

		return 0;
	}

	cfg->lt_data_valid = false;

	tegra_dp_tpg(dp, TRAINING_PATTERN_1, n_lanes);
	tegra_dp_wait_aux_training(dp, true);
	cr_done = tegra_dp_clock_recovery_status(dp);
	cr_done ? : ({ret = -EINVAL; goto fail; });

	if (cfg->tps3_supported)
		tegra_dp_tpg(dp, TRAINING_PATTERN_3, n_lanes);
	else
		tegra_dp_tpg(dp, TRAINING_PATTERN_2, n_lanes);
	tegra_dp_wait_aux_training(dp, false);
	lt_done = tegra_dp_lt_status(dp);
	lt_done ? : ({ret = -EINVAL; goto fail; });

	cfg->lt_data_valid = true;

	tegra_dc_dp_dump_link_cfg(dp, cfg);
fail:
	tegra_dp_tpg(dp, TRAINING_PATTERN_DISABLE, n_lanes);
	return ret;
}

static int tegra_dp_fast_lt(struct tegra_dc_dp_data *dp,
				struct tegra_dc_dp_link_config *cfg,
				bool handshake)
{
	BUG_ON(!cfg || !cfg->is_valid);

	tegra_sor_precharge_lanes(dp->sor);

	if (!handshake)
		return _tegra_dp_fast_lt(dp, cfg, false);

	tegra_dp_lt_config(dp, cfg->preemphasis,
			cfg->drive_current, cfg->postcursor);

	return _tegra_dp_fast_lt(dp, cfg, true);
}

static inline void tegra_dp_save_link_config(struct tegra_dc_dp_data *dp,
				struct tegra_dc_dp_link_config *old_cfg)
{
	*old_cfg = dp->link_cfg;
}

static inline void tegra_dp_restore_link_config(struct tegra_dc_dp_data *dp,
				struct tegra_dc_dp_link_config *old_cfg)
{
		dp->link_cfg = *old_cfg;
		tegra_dp_link_config(dp);
}

static bool tegra_dp_lower_link_config(struct tegra_dc_dp_data *dp,
					struct tegra_dc_dp_link_config *cfg)
{
	struct tegra_dc_dp_link_config tmp_cfg;
	BUG_ON(!cfg);

	tegra_dp_save_link_config(dp, &tmp_cfg);
	cfg->is_valid = false;

	if (cfg->link_bw == SOR_LINK_SPEED_G1_62) {
		if (cfg->max_link_bw > SOR_LINK_SPEED_G1_62)
			cfg->link_bw = SOR_LINK_SPEED_G2_7;
		cfg->lane_count /= 2;
	} else if (cfg->link_bw == SOR_LINK_SPEED_G2_7)
		cfg->link_bw = SOR_LINK_SPEED_G1_62;
	else if (cfg->link_bw == SOR_LINK_SPEED_G5_4) {
		if (cfg->lane_count == 1) {
			cfg->link_bw = SOR_LINK_SPEED_G2_7;
			cfg->lane_count = cfg->max_lane_count;
		} else
			cfg->lane_count /= 2;
	} else {
		dev_err(&dp->dc->ndev->dev,
			"dp: Error link rate %d\n", cfg->link_bw);
		return false;
	}

	if (cfg->lane_count <= 0)
		goto fail;

	if (!tegra_dc_dp_calc_config(dp, dp->mode, cfg))
		goto fail;

	cfg->is_valid = true;

	tegra_dp_link_config(dp);

	return true;
fail:
	tegra_dp_restore_link_config(dp, &tmp_cfg);
	return false;
}

static int tegra_dp_lt(struct tegra_dc_dp_data *dp)
{
	int ret = -EFAULT;
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	struct tegra_dp_out *dp_pdata = dp->pdata;

	tegra_dp_clk_enable(dp);
	tegra_sor_config_dp_clk(dp->sor);

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return 0;

	mutex_lock(&dp->lt_lock);

	if (cfg->support_fast_lt && cfg->lt_data_valid) {
		ret = tegra_dp_fast_lt(dp, cfg, true);
		if (!ret)
			goto lt_success;
	} else if (dp->pdata && dp->pdata->n_lt_settings) {
		size_t cnt = 0;
		struct tegra_dc_dp_link_config lt_pass_cfg;
		size_t copy_bytes = sizeof(
			dp_pdata->lt_settings[0].lane_preemphasis[0]) * 4;

		for (; cnt < dp->pdata->n_lt_settings; cnt++) {
			struct tegra_dc_dp_lt_settings *lt_data =
						&dp_pdata->lt_settings[cnt];
			memcpy(cfg->preemphasis,
				lt_data->lane_preemphasis, copy_bytes);
			memcpy(cfg->drive_current,
				lt_data->drive_current, copy_bytes);
			memcpy(cfg->postcursor,
				lt_data->post_cursor, copy_bytes);
			ret = tegra_dp_fast_lt(dp, cfg, true);
			if (!ret)
				break;
		}

		/* Fast link training failed with platform data */
		if (ret)
			goto try_full_lt;

		/* Try lower link config */
		do {
			lt_pass_cfg = *cfg;
			if (!tegra_dp_lower_link_config(dp, cfg))
				break;
		} while (!(ret = tegra_dp_fast_lt(dp, cfg, true)));

		/* Try last known good link config */
		if (ret) {
			tegra_dp_restore_link_config(dp, &lt_pass_cfg);
			ret = tegra_dp_fast_lt(dp, cfg, true);
		}

		/* Fast link training failed with known good link config */
		if (unlikely(ret)) {
			dev_err(&dp->dc->ndev->dev,
				"dp: fast lt failed, trying full lt\n");
			goto try_full_lt;
		}
	}

try_full_lt:
	if (ret) {
		tegra_dp_restore_link_config(dp, &dp->max_link_cfg);
		ret = tegra_dp_full_lt(dp);
		if (ret < 0)
			dev_err(&dp->dc->ndev->dev, "dp: full lt failed.\n");
	}
lt_success:
	mutex_unlock(&dp->lt_lock);
	return ret;
}

static void tegra_dp_lt_worker(struct work_struct *work)
{
	struct tegra_dc_dp_data *dp =
		container_of(work, struct tegra_dc_dp_data, lt_work);

	if (!dp->enabled)
		return;

	tegra_dc_io_start(dp->dc);
	tegra_sor_clk_enable(dp->sor);
	tegra_dpaux_clk_enable(dp);

	tegra_sor_stop_dc(dp->sor);
	tegra_dp_lt(dp);
	tegra_sor_start_dc(dp->sor);

	tegra_dpaux_clk_disable(dp);
	tegra_sor_clk_disable(dp->sor);
	tegra_dc_io_end(dp->dc);
}

static irqreturn_t tegra_dp_irq(int irq, void *ptr)
{
	struct tegra_dc_dp_data *dp = ptr;
	struct tegra_dc *dc = dp->dc;
	u32 status;

	if (tegra_platform_is_fpga())
		return IRQ_NONE;

	tegra_dc_io_start(dc);

	/* clear pending bits */
	status = tegra_dpaux_readl(dp, DPAUX_INTR_AUX);
	tegra_dpaux_writel(dp, DPAUX_INTR_AUX, status);

	if (status & DPAUX_INTR_AUX_PLUG_EVENT_PENDING)
		complete_all(&dp->hpd_plug);

	if (status & DPAUX_INTR_AUX_TX_DONE_PENDING)
		complete_all(&dp->aux_tx);

	if (status & DPAUX_INTR_AUX_IRQ_EVENT_PENDING)
		schedule_work(&dp->lt_work);

	tegra_dc_io_end(dc);
	return IRQ_HANDLED;
}

static int tegra_dc_dp_init(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp;
	struct resource *res;
	struct resource *base_res;
	struct resource of_dp_res;
	void __iomem *base;
	struct clk *clk;
	struct clk *parent_clk;
	int err;
	u32 irq;
	struct device_node *np = dc->ndev->dev.of_node;

	struct device_node *np_dp =
		of_find_node_by_path("/host1x/dpaux");

	dp = devm_kzalloc(&dc->ndev->dev, sizeof(*dp), GFP_KERNEL);
	if (!dp)
		return -ENOMEM;

	if (np) {
		if (np_dp && of_device_is_available(np_dp)) {
			irq = of_irq_to_resource(np_dp, 0, NULL);
			if (!irq) {
				err = -ENOENT;
				goto err_free_dp;
			}
			of_address_to_resource(np_dp, 0, &of_dp_res);
			res = &of_dp_res;
		} else {
			err = -EINVAL;
			goto err_free_dp;
		}
	} else {
		irq = platform_get_irq_byname(dc->ndev, "irq_dp");
		if (irq <= 0) {
			dev_err(&dc->ndev->dev, "dp: no irq\n");
			err = -ENOENT;
			goto err_free_dp;
		}
		res = platform_get_resource_byname(dc->ndev,
			IORESOURCE_MEM, "dpaux");
	}
	if (!res) {
		dev_err(&dc->ndev->dev, "dp: no mem resources for dpaux\n");
		err = -EFAULT;
		goto err_free_dp;
	}

	base_res = devm_request_mem_region(&dc->ndev->dev,
		res->start, resource_size(res),
		dc->ndev->name);
	if (!base_res) {
		dev_err(&dc->ndev->dev, "dp: request_mem_region failed\n");
		err = -EFAULT;
		goto err_free_dp;
	}

	base = devm_ioremap(&dc->ndev->dev, res->start, resource_size(res));
	if (!base) {
		dev_err(&dc->ndev->dev, "dp: registers can't be mapped\n");
		err = -EFAULT;
		goto err_release_resource_reg;
	}

	clk = clk_get_sys("dpaux", NULL);
	if (IS_ERR_OR_NULL(clk)) {
		dev_err(&dc->ndev->dev, "dp: dc clock %s.edp unavailable\n",
			dev_name(&dc->ndev->dev));
		err = -EFAULT;
		goto err_iounmap_reg;
	}

	parent_clk = tegra_get_clock_by_name("pll_dp");
	if (IS_ERR_OR_NULL(parent_clk)) {
		dev_err(&dc->ndev->dev, "dp: clock pll_dp unavailable\n");
		err = -EFAULT;
		goto err_iounmap_reg;
	}

	if (!tegra_platform_is_fpga()) {
		if (request_threaded_irq(irq, NULL, tegra_dp_irq,
					IRQF_ONESHOT, "tegra_dp", dp)) {
			dev_err(&dc->ndev->dev,
				"dp: request_irq %u failed\n", irq);
			err = -EBUSY;
			goto err_get_clk;
		}
	}
	tegra_dp_disable_irq(irq);

	dp->dc = dc;
	dp->aux_base = base;
	dp->res = res;
	dp->aux_base_res = base_res;
	dp->dpaux_clk = clk;
	dp->parent_clk = parent_clk;
	dp->mode = &dc->mode;
	dp->sor = tegra_dc_sor_init(dc, &dp->link_cfg);
	dp->irq = irq;
	dp->pdata = dc->pdata->default_out->dp_out;

	if (IS_ERR_OR_NULL(dp->sor)) {
		err = PTR_ERR(dp->sor);
		dp->sor = NULL;
		goto err_get_clk;
	}

	if (dp->dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		dp->dp_edid = tegra_edid_create(dc, tegra_dc_dp_i2c_xfer);
		if (IS_ERR_OR_NULL(dp->dp_edid)) {
			dev_err(&dc->ndev->dev,
				"dp: failed to create edid obj\n");
			err = PTR_ERR(dp->dp_edid);
			goto err_edid_destroy;
		}
		tegra_dc_set_edid(dc, dp->dp_edid);
	}

	INIT_WORK(&dp->lt_work, tegra_dp_lt_worker);
	init_completion(&dp->hpd_plug);
	init_completion(&dp->aux_tx);

	mutex_init(&dp->dpaux_lock);
	mutex_init(&dp->lt_lock);

	tegra_dc_set_outdata(dc, dp);
	tegra_dc_dp_debug_create(dp);

	return 0;

err_edid_destroy:
	tegra_edid_destroy(dp->dp_edid);
err_get_clk:
	clk_put(clk);
err_iounmap_reg:
	devm_iounmap(&dc->ndev->dev, base);
err_release_resource_reg:
	devm_release_mem_region(&dc->ndev->dev,
		res->start,
		resource_size(res));

	if (!np_dp || !of_device_is_available(np_dp))
		release_resource(res);
err_free_dp:
	devm_kfree(&dc->ndev->dev, dp);

	return err;
}

static void tegra_dp_hpd_config(struct tegra_dc_dp_data *dp)
{
#define TEGRA_DP_HPD_UNPLUG_MIN_US	2000
#define TEGRA_DP_HPD_PLUG_MIN_US	250
#define TEGRA_DP_HPD_IRQ_MIN_US		250

	u32 val;

	val = TEGRA_DP_HPD_PLUG_MIN_US |
		(TEGRA_DP_HPD_UNPLUG_MIN_US <<
		DPAUX_HPD_CONFIG_UNPLUG_MIN_TIME_SHIFT);
	tegra_dpaux_writel(dp, DPAUX_HPD_CONFIG, val);

	tegra_dpaux_writel(dp, DPAUX_HPD_IRQ_CONFIG, TEGRA_DP_HPD_IRQ_MIN_US);

#undef TEGRA_DP_HPD_IRQ_MIN_US
#undef TEGRA_DP_HPD_PLUG_MIN_US
#undef TEGRA_DP_HPD_UNPLUG_MIN_US
}

static int tegra_dp_hpd_plug(struct tegra_dc_dp_data *dp)
{
#define TEGRA_DP_HPD_PLUG_TIMEOUT_MS	500
	u32 val;
	int err = 0;

	might_sleep();

	if (tegra_platform_is_fpga()) {
		msleep(TEGRA_DP_HPD_PLUG_TIMEOUT_MS);
		return 0;
	}

	reinit_completion(&dp->hpd_plug);
	tegra_dp_int_en(dp, DPAUX_INTR_EN_AUX_PLUG_EVENT);

	val = tegra_dpaux_readl(dp, DPAUX_DP_AUXSTAT);
	if (likely(val & DPAUX_DP_AUXSTAT_HPD_STATUS_PLUGGED))
		err = 0;
	else if (!wait_for_completion_timeout(&dp->hpd_plug,
		msecs_to_jiffies(TEGRA_DP_HPD_PLUG_TIMEOUT_MS)))
		err = -ENODEV;

	tegra_dp_int_dis(dp, DPAUX_INTR_EN_AUX_PLUG_EVENT);

	return err;

#undef TEGRA_DP_HPD_PLUG_TIMEOUT_MS
}

static void tegra_dp_set_tx_pu(struct tegra_dc_dp_data *dp, u32 pe[4],
				u32 vs[4], u32 pc[4])
{
	u32 n_lanes = dp->link_cfg.lane_count;
	int cnt = 1;
	u32 max_tx_pu = tegra_dp_tx_pu[pc[0]][vs[0]][pe[0]];

	if (dp->pdata && dp->pdata->tx_pu_disable) {
		tegra_sor_write_field(dp->sor,
				NV_SOR_DP_PADCTL(dp->sor->portnum),
				NV_SOR_DP_PADCTL_TX_PU_ENABLE,
				NV_SOR_DP_PADCTL_TX_PU_DISABLE);
		return;
	}

	for (; cnt < n_lanes; cnt++) {
		max_tx_pu = (max_tx_pu <
			tegra_dp_tx_pu[pc[cnt]][vs[cnt]][pe[cnt]]) ?
			tegra_dp_tx_pu[pc[cnt]][vs[cnt]][pe[cnt]] :
			max_tx_pu;
	}

	tegra_sor_write_field(dp->sor, NV_SOR_DP_PADCTL(dp->sor->portnum),
				NV_SOR_DP_PADCTL_TX_PU_VALUE_DEFAULT_MASK,
				(max_tx_pu <<
				NV_SOR_DP_PADCTL_TX_PU_VALUE_SHIFT |
				NV_SOR_DP_PADCTL_TX_PU_ENABLE));
}

static void tegra_dp_lt_config(struct tegra_dc_dp_data *dp,
				u32 pe[4], u32 vs[4], u32 pc[4])
{
	struct tegra_dc_sor_data *sor = dp->sor;
	u32 n_lanes = dp->link_cfg.lane_count;
	bool pc_supported = dp->link_cfg.tps3_supported;
	u32 cnt;
	u32 val;

	for (cnt = 0; cnt < n_lanes; cnt++) {
		u32 mask = 0;
		u32 pe_reg, vs_reg, pc_reg;
		u32 shift = 0;
		switch (cnt) {
		case 0:
			mask = NV_SOR_PR_LANE2_DP_LANE0_MASK;
			shift = NV_SOR_PR_LANE2_DP_LANE0_SHIFT;
			break;
		case 1:
			mask = NV_SOR_PR_LANE1_DP_LANE1_MASK;
			shift = NV_SOR_PR_LANE1_DP_LANE1_SHIFT;
			break;
		case 2:
			mask = NV_SOR_PR_LANE0_DP_LANE2_MASK;
			shift = NV_SOR_PR_LANE0_DP_LANE2_SHIFT;
			break;
		case 3:
			mask = NV_SOR_PR_LANE3_DP_LANE3_MASK;
			shift = NV_SOR_PR_LANE3_DP_LANE3_SHIFT;
			break;
		default:
			dev_err(&dp->dc->ndev->dev,
				"dp: incorrect lane cnt\n");
		}
		pe_reg = tegra_dp_pe_regs[pc[cnt]][vs[cnt]][pe[cnt]];
		vs_reg = tegra_dp_vs_regs[pc[cnt]][vs[cnt]][pe[cnt]];
		pc_reg = tegra_dp_pc_regs[pc[cnt]][vs[cnt]][pe[cnt]];
		tegra_sor_write_field(sor, NV_SOR_PR(sor->portnum),
						mask, (pe_reg << shift));
		tegra_sor_write_field(sor, NV_SOR_DC(sor->portnum),
						mask, (vs_reg << shift));
		if (pc_supported) {
			tegra_sor_write_field(
					sor, NV_SOR_POSTCURSOR(sor->portnum),
					mask, (pc_reg << shift));
		}
	}
	tegra_dp_set_tx_pu(dp, pe, vs, pc);
	usleep_range(15, 20);

	for (cnt = 0; cnt < n_lanes; cnt++) {
		u32 max_vs_flag = tegra_dp_is_max_vs(pe[cnt], vs[cnt]);
		u32 max_pe_flag = tegra_dp_is_max_pe(pe[cnt], vs[cnt]);

		val = (vs[cnt] << NV_DPCD_TRAINING_LANEX_SET_DC_SHIFT) |
			(max_vs_flag ?
			NV_DPCD_TRAINING_LANEX_SET_DC_MAX_REACHED_T :
			NV_DPCD_TRAINING_LANEX_SET_DC_MAX_REACHED_F) |
			(pe[cnt] << NV_DPCD_TRAINING_LANEX_SET_PE_SHIFT) |
			(max_pe_flag ?
			NV_DPCD_TRAINING_LANEX_SET_PE_MAX_REACHED_T :
			NV_DPCD_TRAINING_LANEX_SET_PE_MAX_REACHED_F);
		tegra_dc_dp_dpcd_write(dp,
			(NV_DPCD_TRAINING_LANE0_SET + cnt), val);
	}
	if (pc_supported) {
		for (cnt = 0; cnt < n_lanes / 2; cnt++) {
			u32 max_pc_flag0 = tegra_dp_is_max_pc(pc[cnt]);
			u32 max_pc_flag1 = tegra_dp_is_max_pc(pc[cnt + 1]);
			val = (pc[cnt] << NV_DPCD_LANEX_SET2_PC2_SHIFT) |
				(max_pc_flag0 ?
				NV_DPCD_LANEX_SET2_PC2_MAX_REACHED_T :
				NV_DPCD_LANEX_SET2_PC2_MAX_REACHED_F) |
				(pc[cnt + 1] <<
				NV_DPCD_LANEXPLUS1_SET2_PC2_SHIFT) |
				(max_pc_flag1 ?
				NV_DPCD_LANEXPLUS1_SET2_PC2_MAX_REACHED_T :
				NV_DPCD_LANEXPLUS1_SET2_PC2_MAX_REACHED_F);
			tegra_dc_dp_dpcd_write(dp,
				(NV_DPCD_TRAINING_LANE0_1_SET2 + cnt), val);
		}
	}
}

static bool tegra_dp_clock_recovery_status(struct tegra_dc_dp_data *dp)
{
	u32 cnt;
	u32 n_lanes = dp->link_cfg.lane_count;
	u8 data_ptr;

	for (cnt = 0; cnt < n_lanes / 2; cnt++) {
		tegra_dc_dp_dpcd_read(dp,
			(NV_DPCD_LANE0_1_STATUS + cnt), &data_ptr);

		if (n_lanes == 1)
			return (data_ptr & 0x1) ? true : false;
		else if (!(data_ptr & 0x1) ||
			!(data_ptr &
			(0x1 << NV_DPCD_STATUS_LANEXPLUS1_CR_DONE_SHIFT)))
			return false;
	}

	return true;
}

static void tegra_dp_lt_adjust(struct tegra_dc_dp_data *dp,
				u32 pe[4], u32 vs[4], u32 pc[4],
				bool pc_supported)
{
	size_t cnt;
	u8 data_ptr;
	u32 n_lanes = dp->link_cfg.lane_count;

	for (cnt = 0; cnt < n_lanes / 2; cnt++) {
		tegra_dc_dp_dpcd_read(dp,
			(NV_DPCD_LANE0_1_ADJUST_REQ + cnt), &data_ptr);
		pe[2 * cnt] = (data_ptr & NV_DPCD_ADJUST_REQ_LANEX_PE_MASK) >>
					NV_DPCD_ADJUST_REQ_LANEX_PE_SHIFT;
		vs[2 * cnt] = (data_ptr & NV_DPCD_ADJUST_REQ_LANEX_DC_MASK) >>
					NV_DPCD_ADJUST_REQ_LANEX_DC_SHIFT;
		pe[1 + 2 * cnt] =
			(data_ptr & NV_DPCD_ADJUST_REQ_LANEXPLUS1_PE_MASK) >>
					NV_DPCD_ADJUST_REQ_LANEXPLUS1_PE_SHIFT;
		vs[1 + 2 * cnt] =
			(data_ptr & NV_DPCD_ADJUST_REQ_LANEXPLUS1_DC_MASK) >>
					NV_DPCD_ADJUST_REQ_LANEXPLUS1_DC_SHIFT;
	}
	if (pc_supported) {
		tegra_dc_dp_dpcd_read(dp,
				NV_DPCD_ADJUST_REQ_POST_CURSOR2, &data_ptr);
		for (cnt = 0; cnt < n_lanes; cnt++) {
			pc[cnt] = (data_ptr >>
			NV_DPCD_ADJUST_REQ_POST_CURSOR2_LANE_SHIFT(cnt)) &
			NV_DPCD_ADJUST_REQ_POST_CURSOR2_LANE_MASK;
		}
	}
}

static int _tegra_dp_clk_recovery(struct tegra_dc_dp_data *dp, u32 pe[4],
					u32 vs[4], u32 pc[4], bool pc_supported,
					u32 n_lanes)
{
	bool cr_done = true;
	u32 vs_temp[4];
	u32 retry_cnt = 1;
retry:
	tegra_dp_lt_config(dp, pe, vs, pc);
	tegra_dp_wait_aux_training(dp, true);

	cr_done = tegra_dp_clock_recovery_status(dp);
	if (cr_done)
		return 0;

	memcpy(vs_temp, vs, sizeof(vs_temp));

	tegra_dp_lt_adjust(dp, pe, vs, pc, pc_supported);

	if (!memcmp(vs_temp, vs, sizeof(vs_temp))) {
		if (retry_cnt++ >= 5)
			return -EBUSY;
		goto retry;
	}

	return _tegra_dp_clk_recovery(dp, pe, vs, pc, pc_supported, n_lanes);
}

static int tegra_dp_clk_recovery(struct tegra_dc_dp_data *dp,
					u32 pe[4], u32 vs[4], u32 pc[4])
{
	u32 n_lanes = dp->link_cfg.lane_count;
	bool pc_supported = dp->link_cfg.tps3_supported;
	int err;

	tegra_dp_tpg(dp, TRAINING_PATTERN_1, n_lanes);

	err = _tegra_dp_clk_recovery(dp, pe, vs, pc, pc_supported, n_lanes);
	if (err < 0)
		tegra_dp_tpg(dp, TRAINING_PATTERN_DISABLE, n_lanes);

	return err;
}

static bool tegra_dp_channel_eq_status(struct tegra_dc_dp_data *dp)
{
	u32 cnt;
	u32 n_lanes = dp->link_cfg.lane_count;
	u8 data_ptr;
	bool ce_done = true;

	for (cnt = 0; cnt < n_lanes / 2; cnt++) {
		tegra_dc_dp_dpcd_read(dp,
			(NV_DPCD_LANE0_1_STATUS + cnt), &data_ptr);

		if (n_lanes == 1) {
			ce_done = (data_ptr &
			(0x1 << NV_DPCD_STATUS_LANEX_CHN_EQ_DONE_SHIFT)) &&
			(data_ptr &
			(0x1 << NV_DPCD_STATUS_LANEX_SYMBOL_LOCKED_SHFIT));
			break;
		} else if (!(data_ptr &
		(0x1 << NV_DPCD_STATUS_LANEX_CHN_EQ_DONE_SHIFT)) ||
		!(data_ptr &
		(0x1 << NV_DPCD_STATUS_LANEX_SYMBOL_LOCKED_SHFIT)) ||
		!(data_ptr &
		(0x1 << NV_DPCD_STATUS_LANEXPLUS1_CHN_EQ_DONE_SHIFT)) ||
		!(data_ptr &
		(0x1 << NV_DPCD_STATUS_LANEXPLUS1_SYMBOL_LOCKED_SHIFT))) {
			ce_done = false;
			break;
		}
	}

	if (ce_done) {
		tegra_dc_dp_dpcd_read(dp,
			NV_DPCD_LANE_ALIGN_STATUS_UPDATED, &data_ptr);
		if (!(data_ptr &
			NV_DPCD_LANE_ALIGN_STATUS_INTERLANE_ALIGN_DONE_YES))
			ce_done = false;
	}

	return ce_done;
}

static int _tegra_dp_channel_eq(struct tegra_dc_dp_data *dp, u32 pe[4],
				u32 vs[4], u32 pc[4], bool pc_supported,
				u32 n_lanes)
{
	bool cr_done = true;
	bool ce_done = true;
	u32 retry_cnt = 1;
retry:
	tegra_dp_wait_aux_training(dp, false);

	cr_done = tegra_dp_clock_recovery_status(dp);
	ce_done = tegra_dp_channel_eq_status(dp);

	if (!cr_done)
		goto fail;

	if (ce_done)
		return 0;

	if (++retry_cnt > 5)
		goto fail;

	tegra_dp_lt_adjust(dp, pe, vs, pc, pc_supported);

	tegra_dp_lt_config(dp, pe, vs, pc);

	goto retry;
fail:
	if (tegra_dp_lower_link_config(dp, &dp->link_cfg))
		return -EAGAIN;
	return -EBUSY;
}

static int tegra_dp_channel_eq(struct tegra_dc_dp_data *dp,
					u32 pe[4], u32 vs[4], u32 pc[4])
{
	u32 n_lanes = dp->link_cfg.lane_count;
	bool pc_supported = dp->link_cfg.tps3_supported;
	int err;
	u32 tp_src = TRAINING_PATTERN_2;

	if (pc_supported)
		tp_src = TRAINING_PATTERN_3;

	tegra_dp_tpg(dp, tp_src, n_lanes);

	err = _tegra_dp_channel_eq(dp, pe, vs, pc, pc_supported, n_lanes);

	tegra_dp_tpg(dp, TRAINING_PATTERN_DISABLE, n_lanes);

	return err;
}

static int tegra_dp_full_lt(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_sor_data *sor = dp->sor;
	int err;
	u32 pe[4] = {
		PRE_EMPHASIS_L0,
		PRE_EMPHASIS_L0,
		PRE_EMPHASIS_L0,
		PRE_EMPHASIS_L0
	};
	u32 vs[4] = {
		DRIVE_CURRENT_L0,
		DRIVE_CURRENT_L0,
		DRIVE_CURRENT_L0,
		DRIVE_CURRENT_L0
	};
	u32 pc[4] = {
		POST_CURSOR2_L0,
		POST_CURSOR2_L0,
		POST_CURSOR2_L0,
		POST_CURSOR2_L0
	};
	size_t copy_bytes = sizeof(pe[0]) * 4;
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;

	cfg->lt_data_valid = false;

	tegra_sor_precharge_lanes(sor);

retry_cr:
	memset(pe, PRE_EMPHASIS_L0, sizeof(pe));
	memset(vs, DRIVE_CURRENT_L0, sizeof(vs));
	memset(pc, POST_CURSOR2_L0, sizeof(pc));

	err = tegra_dp_clk_recovery(dp, pe, vs, pc);
	if (err < 0) {
		if (tegra_dp_lower_link_config(dp, &dp->link_cfg))
			goto retry_cr;

		dev_err(&dp->dc->ndev->dev, "dp: clk recovery failed\n");
		goto fail;
	}

	err = tegra_dp_channel_eq(dp, pe, vs, pc);
	if (err < 0) {
		if (err == -EAGAIN)
			goto retry_cr;

		dev_err(&dp->dc->ndev->dev,
			"dp: channel equalization failed\n");
		goto fail;
	}

	memcpy(cfg->preemphasis, pe, copy_bytes);
	memcpy(cfg->drive_current, vs, copy_bytes);
	memcpy(cfg->postcursor, pc, copy_bytes);

	dp->link_cfg.lt_data_valid = true;

	tegra_dc_dp_dump_link_cfg(dp, &dp->link_cfg);

	return 0;
fail:
	return err;
}

static void tegra_dp_dpcd_init(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	u32 size_ieee_oui = 3, auxstat;
	u8 data_ieee_oui_be[3] = {(NV_IEEE_OUI >> 16) & 0xff,
		(NV_IEEE_OUI >> 8) & 0xff,
		NV_IEEE_OUI & 0xff};

	if (cfg->is_valid)
		return;

	/* Check DP version */
	if (tegra_dc_dp_dpcd_read(dp, NV_DPCD_REV, &dp->revision))
		dev_err(&dp->dc->ndev->dev,
			"dp: failed to read the revision number from sink\n");

	if (tegra_dp_init_max_link_cfg(dp, cfg))
		dev_err(&dp->dc->ndev->dev,
			"dp: failed to init link configuration\n");

	tegra_dc_dpaux_write(dp, DPAUX_DP_AUXCTL_CMD_AUXWR,
		NV_DPCD_SOURCE_IEEE_OUI, data_ieee_oui_be, &size_ieee_oui,
		&auxstat);
}

static void tegra_dp_tpg(struct tegra_dc_dp_data *dp, u32 tp, u32 n_lanes)
{
	tegra_sor_tpg(dp->sor, tp, n_lanes);

	if (tp == TRAINING_PATTERN_DISABLE)
		tegra_dc_dp_dpcd_write(dp, NV_DPCD_TRAINING_PATTERN_SET,
			(tp | NV_DPCD_TRAINING_PATTERN_SET_SC_DISABLED_F));
	else
		tegra_dc_dp_dpcd_write(dp, NV_DPCD_TRAINING_PATTERN_SET,
			(tp | NV_DPCD_TRAINING_PATTERN_SET_SC_DISABLED_T));
}

static void tegra_dp_tu_config(struct tegra_dc_dp_data *dp,
				const struct tegra_dc_dp_link_config *cfg)
{
	struct tegra_dc_sor_data *sor = dp->sor;
	u32 reg_val;

	tegra_sor_write_field(sor, NV_SOR_DP_LINKCTL(sor->portnum),
			NV_SOR_DP_LINKCTL_TUSIZE_MASK,
			(cfg->tu_size << NV_SOR_DP_LINKCTL_TUSIZE_SHIFT));

	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_WATERMARK_MASK,
				cfg->watermark);

	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_ACTIVESYM_COUNT_MASK,
				(cfg->active_count <<
				NV_SOR_DP_CONFIG_ACTIVESYM_COUNT_SHIFT));

	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_ACTIVESYM_FRAC_MASK,
				(cfg->active_frac <<
				NV_SOR_DP_CONFIG_ACTIVESYM_FRAC_SHIFT));

	reg_val = cfg->activepolarity ?
		NV_SOR_DP_CONFIG_ACTIVESYM_POLARITY_POSITIVE :
		NV_SOR_DP_CONFIG_ACTIVESYM_POLARITY_NEGATIVE;
	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_ACTIVESYM_POLARITY_POSITIVE,
				reg_val);

	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_ACTIVESYM_CNTL_ENABLE,
				NV_SOR_DP_CONFIG_ACTIVESYM_CNTL_ENABLE);

	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_RD_RESET_VAL_NEGATIVE,
				NV_SOR_DP_CONFIG_RD_RESET_VAL_NEGATIVE);
}

static void tegra_dp_link_config(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	struct tegra_dc_sor_data *sor = dp->sor;

	BUG_ON(!cfg || !cfg->is_valid || !dp->mode);

	tegra_dp_set_link_bandwidth(dp, cfg->link_bw);
	tegra_dp_set_lane_count(dp, cfg->lane_count);
	tegra_dp_set_enhanced_framing(dp, cfg->enhanced_framing);

	if (cfg->alt_scramber_reset_cap)
		tegra_dc_dp_set_assr(dp, true);
	else
		tegra_dc_sor_set_internal_panel(dp->sor, false);

	tegra_dc_dp_dpcd_write(dp, NV_DPCD_MAIN_LINK_CHANNEL_CODING_SET,
			NV_DPCD_MAIN_LINK_CHANNEL_CODING_SET_ANSI_8B10B);

	tegra_dp_link_cal(dp);

	tegra_dp_tu_config(dp, cfg);
	tegra_sor_writel(sor, NV_SOR_LVDS, 0);

	tegra_dp_tpg(dp, TRAINING_PATTERN_DISABLE, cfg->lane_count);

	tegra_sor_port_enable(sor, true);
}

static int tegra_dp_edid(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc *dc = dp->dc;
	struct fb_monspecs specs;
	int err;

	memset(&specs, 0 , sizeof(specs));

	if (dp->dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		err = tegra_edid_get_monspecs(dp->dp_edid, &specs);
		if (err < 0) {
			dev_err(&dc->ndev->dev,
				"dp: Failed to get EDID data\n");
			goto fail;
		}

		/* set bpp if EDID provides primary color depth */
		dc->out->depth =
			dc->out->depth ? : specs.bpc ? specs.bpc * 3 : 18;
		dev_info(&dc->ndev->dev,
			"dp: EDID: %d bpc panel, set to %d bpp\n",
			 specs.bpc, dc->out->depth);

		/* in mm */
		dc->out->h_size = dc->out->h_size ? : specs.max_x * 10;
		dc->out->v_size = dc->out->v_size ? : specs.max_y * 10;
	}

	/*
	 * EDID specifies either the acutal screen sizes or
	 * the aspect ratios. The panel file can choose to
	 * trust the value as the actual sizes by leaving
	 * width/height to 0s
	 */
	dc->out->width = dc->out->width ? : dc->out->h_size;
	dc->out->height = dc->out->height ? : dc->out->v_size;

	if (!dc->out->modes)
		tegra_dc_set_fb_mode(dc, specs.modedb, false);

	tegra_dc_setup_clk(dc, dc->clk);
	kfree(specs.modedb);
	return 0;
fail:
	return err;
}

static inline void tegra_dp_reset(struct tegra_dc_dp_data *dp)
{
	tegra_periph_reset_assert(dp->dpaux_clk);
	mdelay(2);
	tegra_periph_reset_deassert(dp->dpaux_clk);
	mdelay(1);
}

static inline void tegra_dp_default_int(struct tegra_dc_dp_data *dp,
					bool enable)
{
	if (enable)
		tegra_dp_int_en(dp, DPAUX_INTR_EN_AUX_IRQ_EVENT);
	else
		tegra_dp_int_dis(dp, DPAUX_INTR_EN_AUX_IRQ_EVENT);
}

static void tegra_dc_dp_enable(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);
	int ret;

	tegra_dp_reset(dp);
	tegra_dpaux_clk_enable(dp);

	tegra_dc_io_start(dc);
	tegra_dpaux_enable(dp);

	if (dp->dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		tegra_dp_enable_irq(dp->irq);
		tegra_dp_default_int(dp, true);

		tegra_dp_hpd_config(dp);
		if (tegra_dp_hpd_plug(dp) < 0) {
			dev_info(&dc->ndev->dev,
				"dp: no panel/monitor plugged\n");
			goto error_enable;
		}
	}

	ret = tegra_dp_panel_power_state(dp, NV_DPCD_SET_POWER_VAL_D0_NORMAL);
	if (ret < 0) {
		dev_err(&dp->dc->ndev->dev,
			"dp: failed to power on panel (0x%x)\n", ret);
		goto error_enable;
	}

	if (dp->dp_edid && !dp->dp_edid->data)
		tegra_dp_edid(dp);

	tegra_dp_dpcd_init(dp);

	tegra_dc_sor_enable_dp(dp->sor);

	tegra_dp_link_config(dp);

	tegra_dp_lt(dp);

	tegra_dc_sor_attach(dp->sor);
	dp->enabled = true;
	tegra_dp_default_int(dp, false);
	tegra_dc_io_end(dc);
	return;

error_enable:
	tegra_dp_default_int(dp, false);
	tegra_dpaux_pad_power(dp->dc, false);
	tegra_dpaux_clk_disable(dp);
	tegra_dc_io_end(dc);
	return;
}

static void tegra_dc_dp_destroy(struct tegra_dc *dc)
{
	struct device_node *np_dp =
		of_find_node_by_path("/host1x/dpaux");
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	if (dp->sor)
		tegra_dc_sor_destroy(dp->sor);
	if (dp->dp_edid)
		tegra_edid_destroy(dp->dp_edid);
	clk_put(dp->dpaux_clk);
	clk_put(dp->parent_clk);
	devm_iounmap(&dc->ndev->dev, dp->aux_base);
	devm_release_mem_region(&dc->ndev->dev,
		dp->res->start,
		resource_size(dp->res));
	if (!np_dp || !of_device_is_available(np_dp))
		release_resource(dp->res);
	devm_kfree(&dc->ndev->dev, dp);
}

static void tegra_dc_dp_disable(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	if (!dp->enabled)
		return;

	cancel_work_sync(&dp->lt_work);

	tegra_dc_io_start(dc);

	if (dp->dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		tegra_dp_default_int(dp, false);
		tegra_dp_disable_irq(dp->irq);
	}

	tegra_dpaux_pad_power(dp->dc, false);

	/* Power down SOR */
	tegra_dc_sor_detach(dp->sor);
	tegra_dc_sor_disable(dp->sor, false);

	tegra_dpaux_clk_disable(dp);
	tegra_dp_clk_disable(dp);

	tegra_dc_io_end(dc);
	dp->enabled = false;
}

static long tegra_dc_dp_setup_clk(struct tegra_dc *dc, struct clk *clk)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);
	struct clk *dc_parent_clk;

	if (tegra_platform_is_fpga())
		return tegra_dc_pclk_round_rate(dc, dc->mode.pclk);

	if (clk == dc->clk) {
		dc_parent_clk = clk_get_sys(NULL,
				dc->out->parent_clk ? : "pll_d_out0");
		clk_set_parent(dc->clk, dc_parent_clk);
	}

	tegra_sor_setup_clk(dp->sor, clk, false);

	/* fixed pll_dp@270MHz */
	clk_set_rate(dp->parent_clk, 270000000);

	return tegra_dc_pclk_round_rate(dc, dc->mode.pclk);
}

/* used by tegra_dc_probe() to detect connection(HPD) status at boot */
static bool tegra_dc_dp_detect(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);
	u32 rd;

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return  false;

	tegra_dc_io_start(dc);
	tegra_dpaux_clk_enable(dp);
	rd = tegra_dpaux_readl(dp, DPAUX_DP_AUXSTAT);
	tegra_dpaux_clk_disable(dp);
	tegra_dc_io_end(dc);
	dev_info(&dc->ndev->dev,
		"dp: DPAUX_DP_AUXSTAT:0x%08x HPD:%splugged\n",
		rd, (DPAUX_DP_AUXSTAT_HPD_STATUS_PLUGGED & rd) ? "" : "un");
	return (DPAUX_DP_AUXSTAT_HPD_STATUS_PLUGGED & rd) ? true : false;
}


static void tegra_dc_dp_modeset_notifier(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	tegra_dc_io_start(dc);
	tegra_dpaux_clk_enable(dp);

	tegra_dc_sor_modeset_notifier(dp->sor, false);

	tegra_dpaux_clk_disable(dp);
	tegra_dc_io_end(dc);
}

struct tegra_dc_out_ops tegra_dc_dp_ops = {
	.init	   = tegra_dc_dp_init,
	.destroy   = tegra_dc_dp_destroy,
	.enable	   = tegra_dc_dp_enable,
	.disable   = tegra_dc_dp_disable,
	.detect    = tegra_dc_dp_detect,
	.setup_clk = tegra_dc_dp_setup_clk,
	.modeset_notifier = tegra_dc_dp_modeset_notifier,
};


