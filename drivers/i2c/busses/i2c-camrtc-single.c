/*
 * drivers/i2c/busses/i2c-tegra-single.c
 *
 * Copyright (C) 2017 NVIDIA Corporation. All rights reserved.
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

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/debugfs.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/delay.h>

#include <soc/tegra/tegra-ivc-rpc.h>

#include "camrtc-i2c-common.h"

/*
 * I2C IVC Single driver internal data structure
 */

#define TEGRA_I2C_SINGLE_MAX_DEV	4

/* Define speed modes */
#define I2C_STANDARD_MODE			100000
#define I2C_FAST_MODE				400000
#define I2C_FAST_MODE_PLUS			1000000
#define I2C_HS_MODE				3500000

#define I2C_CLK_DIVISOR				0x06c
#define I2C_CLK_DIVISOR_STD_FAST_MODE_SHIFT	16
#define I2C_CLK_MULTIPLIER_STD_FAST_MODE	8
#define I2C_CLK_DIVISOR_HS_MODE_MASK		0xFFFF

#define I2C_INTERFACE_TIMING_0                  0x94
#define I2C_TLOW_MASK                           0x3F
#define I2C_THIGH_SHIFT                         8
#define I2C_THIGH_MASK                          (0x3F << I2C_THIGH_SHIFT)

#define I2C_HS_INTERFACE_TIMING			0x9c
#define I2C_HS_TLOW_MASK			0x3F
#define I2C_HS_THIGH_SHIFT                         8
#define I2C_HS_THIGH_MASK			(0x3F << I2C_THIGH_SHIFT)

struct tegra_i2c_ivc_dev {
	/* IVC RPC */
	const char *name;
	struct tegra_ivc_channel *chan;
	bool is_taken;
	bool is_failed;
	atomic_t in_add_single;

	/* I2C device behind this IVC */
	u32 bus_id;
	u32 reg_base;
	u32 bus_clk_rate;

	/* statistics */
	struct {
		unsigned int xfer_requests;
		unsigned int total_bytes;
		unsigned int reads, read_bytes;
		unsigned int writes, write_bytes;
		unsigned int errors;
	} stat;

	/* RPC call for I2C_REQUEST_SINGLE */
	struct tegra_ivc_rpc_call_param rpc_i2c_req;
	u8 rpc_i2c_req_buf[CAMRTC_I2C_REQUEST_MAX_LEN];
	struct camrtc_rpc_i2c_response rpc_i2c_rsp;
	struct work_struct work;
};

static struct tegra_i2c_ivc_dev g_ivc_devs[TEGRA_I2C_SINGLE_MAX_DEV];

/*
 * I2C bus driver internal data structure
 */

struct tegra_i2c_single {
	struct device *dev;
	struct i2c_adapter adapter;
	struct tegra_i2c_ivc_dev *ivc_dev;
	bool is_shutdown;
	bool is_suspended;
	struct clk *div_clk;
	struct clk *slow_clk;
	struct reset_control *rst;
	void __iomem *base;
	u32 bus_clk_rate;
	bool is_clkon_always;
	u16 clk_divisor_non_hs_mode;
	u32 low_clock_count;
	u32 high_clock_count;
	u32 hs_low_clock_count;
	u32 hs_high_clock_count;
	int clk_divisor_hs_mode;
};

static u32 i2c_readl(struct tegra_i2c_single *i2c_dev, unsigned long reg)
{
	return readl(i2c_dev->base + reg);
}

static inline int tegra_i2c_clock_enable(struct tegra_i2c_single *i2c_dev)
{
	int ret;
	ret = clk_enable(i2c_dev->div_clk);
	if (ret < 0) {
		dev_err(i2c_dev->dev,
			"Enabling div clk failed, err %d\n", ret);
		goto err;
	}

	if (i2c_dev->slow_clk) {
		ret = clk_enable(i2c_dev->slow_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Enabling slow clk failed, err %d\n", ret);
			clk_disable(i2c_dev->div_clk);
		}
	}
err:
	return ret;
}

static inline void tegra_i2c_clock_disable(struct tegra_i2c_single *i2c_dev)
{
	clk_disable(i2c_dev->div_clk);
	if (i2c_dev->slow_clk)
		clk_disable(i2c_dev->slow_clk);
}

static void tegra_i2c_get_clk_parameters(struct tegra_i2c_single *i2c_dev)
{
	u32 val;

	val = i2c_readl(i2c_dev, I2C_INTERFACE_TIMING_0);
	i2c_dev->low_clock_count = val & I2C_TLOW_MASK;
	i2c_dev->high_clock_count = (val & I2C_THIGH_MASK) >> I2C_THIGH_SHIFT;

	val = i2c_readl(i2c_dev, I2C_HS_INTERFACE_TIMING);
	i2c_dev->hs_low_clock_count = val & I2C_HS_TLOW_MASK;
	i2c_dev->hs_high_clock_count = ((val & I2C_HS_THIGH_MASK)
		>> I2C_HS_THIGH_SHIFT);

	val = i2c_readl(i2c_dev, I2C_CLK_DIVISOR);
	i2c_dev->clk_divisor_hs_mode = val & I2C_CLK_DIVISOR_HS_MODE_MASK;
	i2c_dev->clk_divisor_non_hs_mode = (val >>
			I2C_CLK_DIVISOR_STD_FAST_MODE_SHIFT);
}

static int tegra_i2c_set_clk_rate(struct tegra_i2c_single *i2c_dev)
{
	u32 clk_multiplier = I2C_CLK_MULTIPLIER_STD_FAST_MODE;
	int ret = 0;


	switch (i2c_dev->bus_clk_rate) {
	case I2C_HS_MODE:
		clk_multiplier = (i2c_dev->hs_low_clock_count +
				i2c_dev->hs_high_clock_count + 2);
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

static int tegra_i2c_init(struct tegra_i2c_single *i2c_dev)
{
	u32 val;
	int err = 0;

	err = tegra_i2c_clock_enable(i2c_dev);
	if (err < 0) {
		dev_err(i2c_dev->dev, "Clock enable failed %d\n", err);
		return err;
	}

	reset_control_assert(i2c_dev->rst);
	udelay(2);
	reset_control_deassert(i2c_dev->rst);

	/* Make sure clock divisor programmed correctly */
	if (i2c_dev->bus_clk_rate == I2C_HS_MODE) {
		i2c_dev->clk_divisor_hs_mode = 2;
	} else {
		val = i2c_readl(i2c_dev, I2C_CLK_DIVISOR);
		i2c_dev->clk_divisor_hs_mode =
			val & I2C_CLK_DIVISOR_HS_MODE_MASK;
	}

	tegra_i2c_get_clk_parameters(i2c_dev);

	err = tegra_i2c_set_clk_rate(i2c_dev);
	if (err < 0)
		return err;

	tegra_i2c_clock_disable(i2c_dev);
	return err;
}

static void tegra_i2c_parse_dt(struct tegra_i2c_single *i2c_dev)
{
	struct device_node *np = i2c_dev->dev->of_node;
	i2c_dev->is_clkon_always = of_property_read_bool(np,
			"nvidia,clock-always-on");
}

static u32 tegra_i2c_get_clk_freq(struct device_node *np)
{
	int ret;
	u32 bus_clk_rate;

	ret = of_property_read_u32(np, "clock-frequency",
			&bus_clk_rate);
	if (ret)
		bus_clk_rate = 100000; /* default clock rate */

	return bus_clk_rate;
}

static u32 tegra_i2c_get_reg_base(struct device_node *np)
{
	u32 reg_vals[4] = {0};

	of_property_read_u32_array(np, "reg", reg_vals, 4);

	return reg_vals[1];
}

/*
 * I2C bus driver interface
 */
static int tegra_ivc_i2c_single_xfer(struct tegra_i2c_ivc_dev *ivc_dev,
	const struct i2c_msg *reqs, int num);

static int tegra_i2c_single_xfer(struct i2c_adapter *adap,
	struct i2c_msg msgs[], int num)
{
	struct tegra_i2c_single *i2c_dev = i2c_get_adapdata(adap);
	struct tegra_i2c_ivc_dev *ivc_dev = i2c_dev->ivc_dev;
	int ret;

	if (i2c_dev->is_suspended)
		return -EBUSY;

	tegra_ivc_channel_runtime_get(ivc_dev->chan);
	pm_runtime_get_sync(&adap->dev);
	ret = tegra_ivc_i2c_single_xfer(ivc_dev, msgs, num);
	pm_runtime_put(&adap->dev);
	tegra_ivc_channel_runtime_put(ivc_dev->chan);

	return ret;
}

static u32 tegra_i2c_single_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR | I2C_FUNC_NOSTART |
		I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm tegra_i2c_single_algo = {
	.master_xfer = tegra_i2c_single_xfer,
	.functionality = tegra_i2c_single_func,
};

/*
 * Probe and initialization
 */
static int tegra_i2c_probe(struct platform_device *pdev)
{
	struct tegra_i2c_single *i2c_dev;
	struct resource *res;
	struct clk *div_clk;
	struct clk *parent_clk;
	struct clk *slow_clk;
	void __iomem *base;
	int ret = 0;
	int i;
	struct tegra_i2c_ivc_dev *ivc_dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	/* Find an IVC channel */
	for (i = 0; i < TEGRA_I2C_SINGLE_MAX_DEV; ++i) {
		if (g_ivc_devs[i].is_taken) {
			if (g_ivc_devs[i].reg_base == (u32) res->start) {
				if (g_ivc_devs[i].is_failed)
					return -ENODEV;
				else
					break;
			}
		} else
			return -EPROBE_DEFER;
	}

	if (i == TEGRA_I2C_SINGLE_MAX_DEV)
		return -ENOMEM;

	ivc_dev = g_ivc_devs + i;

	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	div_clk = devm_clk_get(&pdev->dev, "div-clk");
	if (IS_ERR(div_clk)) {
		dev_err(&pdev->dev, "missing controller clock");
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

	slow_clk = devm_clk_get(&pdev->dev, "slow-clk");
	if (IS_ERR(slow_clk)) {
		dev_err(&pdev->dev, "missing slow clock\n");
		slow_clk = NULL;
	}

	/* Create a private data */
	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;

	i2c_dev->base = base;
	i2c_dev->div_clk = div_clk;
	i2c_dev->slow_clk = slow_clk;
	i2c_dev->dev = &pdev->dev;
	i2c_dev->ivc_dev = ivc_dev;
	i2c_dev->is_shutdown = false;
	i2c_dev->is_suspended = false;
	i2c_dev->rst = devm_reset_control_get(&pdev->dev, "i2c");
	if (IS_ERR(i2c_dev->rst)) {
		dev_err(&pdev->dev, "missing controller reset");
		return PTR_ERR(i2c_dev->rst);
	}

	i2c_dev->bus_clk_rate = tegra_i2c_get_clk_freq(
			i2c_dev->dev->of_node);
	tegra_i2c_parse_dt(i2c_dev);

	platform_set_drvdata(pdev, i2c_dev);

	i2c_dev->clk_divisor_non_hs_mode = 0x19;
	if (i2c_dev->bus_clk_rate == I2C_FAST_MODE_PLUS)
		i2c_dev->clk_divisor_non_hs_mode = 0x10;

	ret = clk_prepare(i2c_dev->div_clk);
	if (ret < 0) {
		dev_err(i2c_dev->dev, "Clock prepare failed %d\n", ret);
		goto fail;
	}
	if (i2c_dev->slow_clk) {
		ret = clk_prepare(i2c_dev->slow_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev, "slow clk prep failed %d\n", ret);
			goto unprepare_div_clk;
		}
	}

	ret = tegra_i2c_clock_enable(i2c_dev);
	if (ret < 0) {
		dev_err(i2c_dev->dev, "div_clk enable failed %d\n",
			ret);
		goto unprepare_slow_clk;
	}

	ret = tegra_i2c_init(i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize i2c controller");
		goto disable_clk;
	}

	/* Probe the I2C hardware */
	pm_runtime_enable(&pdev->dev);

	/* Register I2C bus */
	i2c_set_adapdata(&i2c_dev->adapter, i2c_dev);
	i2c_dev->adapter.owner = THIS_MODULE;
	i2c_dev->adapter.class = I2C_CLASS_DEPRECATED;
	strlcpy(i2c_dev->adapter.name, "Tegra CAMRTC I2C adapter",
		sizeof(i2c_dev->adapter.name));
	i2c_dev->adapter.algo = &tegra_i2c_single_algo;
	i2c_dev->adapter.dev.parent = &pdev->dev;
	i2c_dev->adapter.nr = pdev->id;
	i2c_dev->adapter.dev.of_node = pdev->dev.of_node;

	ret = i2c_add_numbered_adapter(&i2c_dev->adapter);
	if (ret) {
		dev_err(&pdev->dev, "Cannot add I2C adapter: %d\n", ret);
		goto disable_clk;
	}

	pm_runtime_enable(&i2c_dev->adapter.dev);

	BUG_ON(!i2c_dev->ivc_dev->chan->is_ready);

	return 0;

disable_clk:
	tegra_i2c_clock_disable(i2c_dev);

unprepare_slow_clk:
	if (i2c_dev->slow_clk)
		clk_unprepare(i2c_dev->slow_clk);

unprepare_div_clk:
	clk_unprepare(i2c_dev->div_clk);

fail:
	devm_kfree(&pdev->dev, i2c_dev);

	return ret;
}

static int tegra_i2c_remove(struct platform_device *pdev)
{
	struct tegra_i2c_single *i2c_dev = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c_dev->adapter);
	pm_runtime_disable(&i2c_dev->adapter.dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static void tegra_i2c_shutdown(struct platform_device *pdev)
{
	struct tegra_i2c_single *i2c_dev = platform_get_drvdata(pdev);

	dev_info(i2c_dev->dev, "Bus is shutdown down..\n");
	i2c_shutdown_adapter(&i2c_dev->adapter);
	i2c_dev->is_shutdown = true;
}

/*
 * I2C bus driver PM interface
 */

#ifdef CONFIG_PM_SLEEP
static int tegra_i2c_suspend(struct device *dev)
{
	struct tegra_i2c_single *i2c_dev = dev_get_drvdata(dev);

	i2c_lock_adapter(&i2c_dev->adapter);
	i2c_dev->ivc_dev->chan->is_ready = false;
	i2c_dev->is_suspended = true;
	i2c_unlock_adapter(&i2c_dev->adapter);

	return 0;
}

static int tegra_i2c_resume(struct device *dev)
{
	struct tegra_i2c_single *i2c_dev = dev_get_drvdata(dev);

	i2c_lock_adapter(&i2c_dev->adapter);
	i2c_dev->is_suspended = false;
	i2c_unlock_adapter(&i2c_dev->adapter);

	return 0;
}

static SIMPLE_DEV_PM_OPS(tegra_i2c_pm, tegra_i2c_suspend, tegra_i2c_resume);
#define TEGRA_I2C_PM	(&tegra_i2c_pm)
#else
#define TEGRA_I2C_PM	NULL
#endif

/*
 * I2C bus driver interface
 */

static const struct of_device_id tegra_i2c_of_match[] = {
	{ .compatible = "nvidia,tegra186-i2c-single", .data = NULL, },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_i2c_of_match);

static struct platform_driver tegra_i2c_driver = {
	.probe   = tegra_i2c_probe,
	.remove  = tegra_i2c_remove,
	.late_shutdown = tegra_i2c_shutdown,
	.driver  = {
		.name  = "tegra-i2c-single",
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

/*
 * I2C interface
 */
static int tegra_ivc_i2c_add_single(
	struct tegra_ivc_channel *chan);

static int tegra_ivc_i2c_single_xfer(struct tegra_i2c_ivc_dev *ivc_dev,
	const struct i2c_msg *reqs, int num)
{
	u8 *pbuf, *pprev_len;
	const struct i2c_msg *preq, *preq_end;
	int ret, len;
	u8 *read_ptr = NULL;
	int read_len = 0;

	if (num == 0)
		return 0;
	if (tegra_ivc_rpc_channel_is_suspended(ivc_dev->chan))
		return -EBUSY;
	if (!ivc_dev->chan->is_ready) {
		ret = tegra_ivc_i2c_add_single(ivc_dev->chan);
		if (ret != 0) {
			dev_err(&ivc_dev->chan->dev,
				"I2C device not ready\n");
			return ret;
		}
	}

	++ivc_dev->stat.xfer_requests;

	/* First byte is bus ID */

	pbuf = ivc_dev->rpc_i2c_req_buf + 1;
	len = 1;

	preq = reqs;
	preq_end = reqs + num;

	for (;;) {
		int bytes;
		bool is_read;

		if ((preq == preq_end) || ((len + 4 + preq->len) >
		    CAMRTC_I2C_REQUEST_MAX_LEN)) {
			struct camrtc_rpc_i2c_response *rpc_rsp;

			ivc_dev->rpc_i2c_req.request_len = len;
			ret = tegra_ivc_rpc_call(ivc_dev->chan,
				&ivc_dev->rpc_i2c_req);

			if (ret < 0) {
				++ivc_dev->stat.errors;
				dev_err(&ivc_dev->chan->dev,
					"I2C transaction to 0x%x failed: %d\n",
					reqs[0].addr, ret);
				return -EIO;
			}

			rpc_rsp = &ivc_dev->rpc_i2c_rsp;
			if (rpc_rsp->result)
				return -EIO;
			if (read_ptr) {
				memcpy(read_ptr, rpc_rsp->read_data, read_len);
				read_ptr = NULL;
			}

			if (preq == preq_end)
				break;

			pbuf = ivc_dev->rpc_i2c_req_buf + 1;
			len = 1;
		}

		is_read = (preq->flags & I2C_M_RD);

		if (!is_read) {
			pbuf[0] = 0;
			++ivc_dev->stat.writes;
			ivc_dev->stat.write_bytes += preq->len;
		} else {
			read_ptr = preq->buf;
			read_len = preq->len;
			pbuf[0] = CAMRTC_I2C_REQUEST_FLAG_READ;
			++ivc_dev->stat.reads;
			ivc_dev->stat.read_bytes += preq->len;
		}

		if ((preq->flags & I2C_M_NOSTART) == 0) {
			pbuf[1] = preq->addr & 0xff;
			if (preq->flags & I2C_M_TEN) {
				pbuf[0] |= CAMRTC_I2C_REQUEST_FLAG_TEN;
				pbuf[2] = (preq->addr >> 8) & 0xff;
				bytes = 2;
			} else {
				pbuf[2] = 0;
				bytes = 1;
			}
		} else {
			pbuf[0] |= CAMRTC_I2C_REQUEST_FLAG_NOSTART;
			/* slave address is don't care */
			pbuf[1] = 0;
			pbuf[2] = 0;
			bytes = 0;
		}

		pbuf[3] = preq->len;
		pprev_len = pbuf + 3;
		bytes += preq->len;

		pbuf += 4;
		len += 4;

		if (!is_read) {
			u8 *psrc = preq->buf;

			switch (preq->len) {
			case 4:
				*pbuf++ = *psrc++;
			case 3:
				*pbuf++ = *psrc++;
			case 2:
				*pbuf++ = *psrc++;
			case 1:
				*pbuf++ = *psrc++;
			case 0:
				break;
			default:
				memcpy(pbuf, psrc, preq->len);
				pbuf += preq->len;
				break;
			}

			len += preq->len;
		}
		++preq;

		/* Merge requests with NOSTART */
		while (preq != preq_end && (preq->flags & I2C_M_NOSTART)) {
			u8 *psrc = preq->buf;

			if ((len + preq->len) >
			    CAMRTC_I2C_REQUEST_MAX_LEN)
				continue;

			switch (preq->len) {
			case 4:
				*pbuf++ = *psrc++;
			case 3:
				*pbuf++ = *psrc++;
			case 2:
				*pbuf++ = *psrc++;
			case 1:
				*pbuf++ = *psrc++;
			case 0:
				break;
			default:
				memcpy(pbuf, psrc, preq->len);
				pbuf += preq->len;
				break;
			}

			*pprev_len += preq->len;
			len += preq->len;
			bytes += preq->len;

			if (!is_read)
				ivc_dev->stat.write_bytes += preq->len;
			else
				ivc_dev->stat.read_bytes += preq->len;

			++preq;
		}

		ivc_dev->stat.total_bytes += bytes;
	}

	return ret ? -EIO : num;
}

/*
 * IVC channel Debugfs
 */

#define DEFINE_SEQ_FOPS(_fops_, _show_) \
	static int _fops_ ## _open(struct inode *inode, struct file *file) \
	{ \
		return single_open(file, _show_, inode->i_private); \
	} \
	static const struct file_operations _fops_ = { \
		.open = _fops_ ## _open, \
		.read = seq_read, \
		.llseek = seq_lseek, \
		.release = single_release }

static int tegra_ivc_i2c_single_stat_show(
	struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *chan = file->private;
	struct tegra_i2c_ivc_dev *ivc_dev =
		tegra_ivc_channel_get_drvdata(chan);

	seq_printf(file, "Xfer requests: %u\n", ivc_dev->stat.xfer_requests);
	seq_printf(file, "Total bytes: %u\n", ivc_dev->stat.total_bytes);
	seq_printf(file, "Read requests: %u\n", ivc_dev->stat.reads);
	seq_printf(file, "Read bytes: %u\n", ivc_dev->stat.read_bytes);
	seq_printf(file, "Write requests: %u\n", ivc_dev->stat.writes);
	seq_printf(file, "Write bytes: %u\n", ivc_dev->stat.write_bytes);
	seq_printf(file, "Errors: %u\n", ivc_dev->stat.errors);

	return 0;
}

DEFINE_SEQ_FOPS(tegra_ivc_i2c_debugfs_stats,
	tegra_ivc_i2c_single_stat_show);

static void tegra_ivc_i2c_single_create_debugfs(
	struct tegra_ivc_channel *chan,
	struct dentry *debugfs_root)
{
	debugfs_create_file("stats", S_IRUGO,
		debugfs_root, chan,
		&tegra_ivc_i2c_debugfs_stats);
}

/*
 * IVC channel driver interface
 */

static int tegra_ivc_i2c_add_single(
	struct tegra_ivc_channel *chan)
{
	struct tegra_i2c_ivc_dev *ivc_dev =
		tegra_ivc_channel_get_drvdata(chan);
	struct camrtc_rpc_i2c_add_single rpc_add_single;
	u32 bus_id;
	int ret;

	if (ivc_dev->reg_base == 0) {
		dev_err(&chan->dev,
			"Invalid I2C device at 0x%08x\n",
			ivc_dev->reg_base);
		return -ENODEV;
	}

	if (atomic_dec_and_test(&ivc_dev->in_add_single) == false)
		return 0;

	rpc_add_single.reg_base = ivc_dev->reg_base;
	rpc_add_single.bus_clk_rate = ivc_dev->bus_clk_rate;

	/* Register an I2C device to CamRTC */
	ret = tegra_ivc_rpc_call_pl(chan,
		CAMRTC_RPC_REQ_I2C_ADD_SINGLE_DEV,
		sizeof(rpc_add_single), &rpc_add_single,
		TEGRA_IVC_RPC_RSP_RET_CODE,
		sizeof(bus_id), &bus_id,
		NULL, NULL, 0);
	if (ret < 0) {
		dev_err(&chan->dev,
			"Failed to register an I2C device at 0x%08x: %d\n",
			ivc_dev->reg_base, ret);
		ret = -EIO;
		goto fail_remove_chan;
	}

	dev_info(&chan->dev,
		"Registered an I2C device at 0x%08x to bus %08x\n",
		ivc_dev->reg_base, bus_id);

	ivc_dev->bus_id = bus_id;
	ivc_dev->rpc_i2c_req_buf[0] = bus_id;
	ivc_dev->rpc_i2c_req.request_id = CAMRTC_RPC_REQ_I2C_REQUEST_SINGLE;
	ivc_dev->rpc_i2c_req.request = ivc_dev->rpc_i2c_req_buf;
	ivc_dev->rpc_i2c_req.response_id = CAMRTC_RPC_RSP_I2C_RESPONSE;
	ivc_dev->rpc_i2c_req.response_len = sizeof(ivc_dev->rpc_i2c_rsp);
	ivc_dev->rpc_i2c_req.response = &ivc_dev->rpc_i2c_rsp;
	ivc_dev->rpc_i2c_req.callback = NULL;
	ivc_dev->rpc_i2c_req.callback_param = NULL;
	ivc_dev->rpc_i2c_req.timeout_ms = 100;
	ivc_dev->chan->is_ready = true;

	atomic_inc(&ivc_dev->in_add_single);

	return 0;

fail_remove_chan:
	tegra_ivc_rpc_channel_remove(chan);
	ivc_dev->is_failed = true;
	return 0;
}

static void tegra_ivc_i2c_add_single_worker(struct work_struct *work)
{
	int ret = 0;
	struct tegra_i2c_ivc_dev *ivc_dev = container_of(work,
					struct tegra_i2c_ivc_dev, work);
	struct tegra_ivc_channel *chan = ivc_dev->chan;

	ret = tegra_ivc_i2c_add_single(ivc_dev->chan);
	if (ret != 0)
		dev_err(&chan->dev, "I2C device not ready\n");
}

static int tegra_ivc_i2c_single_ready(
	struct tegra_ivc_channel *chan)
{
	struct tegra_i2c_ivc_dev *ivc_dev =
		tegra_ivc_channel_get_drvdata(chan);
	schedule_work(&ivc_dev->work);
	return 0;
}

static int tegra_ivc_i2c_single_pm_prepare(struct device *dev)
{
	return 0;
}

static struct tegra_ivc_rpc_ops tegra_ivc_rpc_user_ops = {
	.create_debugfs = tegra_ivc_i2c_single_create_debugfs,
	.ready = tegra_ivc_i2c_single_ready,
	.pm_prepare = tegra_ivc_i2c_single_pm_prepare,
};

/* Platform device */
static struct platform_device *tegra_i2c_get(struct device *dev)
{
	struct device_node *i2c_node;
	struct platform_device *i2c_pdev;

	i2c_node = of_parse_phandle(dev->of_node, "device", 0);
	if (i2c_node == NULL) {
		dev_err(dev, "cannot get VI device");
		return ERR_PTR(-ENODEV);
	}

	i2c_pdev = of_find_device_by_node(i2c_node);
	of_node_put(i2c_node);

	if (i2c_pdev == NULL)
		return ERR_PTR(-EPROBE_DEFER);

	return i2c_pdev;
}

static int tegra_ivc_rpc_i2c_single_probe(struct tegra_ivc_channel *chan)
{
	int ret;
	int i;
	struct tegra_i2c_ivc_dev *ivc_dev;
	struct platform_device *i2c_dev;
	struct device_node *np;

	/* Find an empty slot */
	for (i = 0; i < TEGRA_I2C_SINGLE_MAX_DEV; ++i) {
		if (!g_ivc_devs[i].is_taken)
			break;
	}

	if (i == TEGRA_I2C_SINGLE_MAX_DEV)
		return -ENOMEM;

	i2c_dev = tegra_i2c_get(&chan->dev);
	if (IS_ERR(i2c_dev)) {
		dev_err(&chan->dev, "Cannot read property device: %ld\n",
			PTR_ERR(i2c_dev));
		return PTR_ERR(i2c_dev);
	}

	np = i2c_dev->dev.of_node;

	ivc_dev = g_ivc_devs + i;

	/* Read properties */
	ret = of_property_read_string(chan->dev.of_node, "nvidia,service",
		&ivc_dev->name);
	if (ret) {
		dev_err(&chan->dev,
		"Cannot read property nvidia,service: %d\n", ret);
		return ret;
	}

	/* Register IVC/RPC channel */
	ret = tegra_ivc_rpc_channel_probe(chan, &tegra_ivc_rpc_user_ops);
	if (ret < 0) {
		dev_err(&chan->dev, "Cannot start IVC/RPC interface");
		goto fail_free_ivc_dev;
	}

	chan->is_ready = false;
	ivc_dev->is_taken = true;
	ivc_dev->is_failed = false;
	ivc_dev->reg_base = tegra_i2c_get_reg_base(np);
	ivc_dev->bus_clk_rate = tegra_i2c_get_clk_freq(np);
	atomic_set(&ivc_dev->in_add_single, 1);
	ivc_dev->chan = chan;
	tegra_ivc_channel_set_drvdata(chan, ivc_dev);
	INIT_WORK(&ivc_dev->work, tegra_ivc_i2c_add_single_worker);

	return 0;

fail_free_ivc_dev:
	return ret;
}

static void tegra_ivc_rpc_i2c_single_remove(struct tegra_ivc_channel *chan)
{
	tegra_ivc_rpc_channel_remove(chan);
}

static const struct of_device_id tegra_ivc_rpc_i2c_single_of_match[] = {
	{ .compatible = "nvidia,tegra186-camera-ivc-rpc-i2c-single", },
	{ },
};
TEGRA_IVC_RPC_DRIVER_DEFINE(i2c_single, "tegra-ivc-rpc-i2c-single")

MODULE_AUTHOR("Kai Lee <kailee@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra CAMRTC I2C driver");
MODULE_LICENSE("GPL");
