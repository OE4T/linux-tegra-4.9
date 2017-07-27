/*
 * Sensor Hub driver for NVIDIA's Tegra186 AON Sensor Processing Engine.
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/string.h>
#include <linux/of_address.h>
#include <linux/tegra-aon.h>
#include <linux/mailbox_client.h>
#include <linux/time.h>
#include <linux/time64.h>
#include <linux/timekeeping.h>

#include <asm/io.h>
#include <asm/arch_timer.h>

#include <aon-shub-messages.h>

#define READJUST_TS_SAMPLES (100)

#define AXIS_N		3
/* block period in ms */
#define TX_BLOCK_PERIOD 200
#define IVC_TIMEOUT	100

enum I2C_IDS {
	I2CID_MIN = 1,
	I2C2 = 1,
	I2C8 = 2,
	I2CID_MAX = 2,
};

struct aon_shub_sensor {
	char name[32];
	char part[32];
	char vendor[32];
	void *nvs_st;
	struct sensor_cfg cfg;
	bool genable;	/* global enable */
};

struct tegra_aon_shub {
	struct device		 *dev;
	struct completion	 *wait_on;
	struct mbox_client	 cl;
	struct mbox_chan	 *mbox;
	struct aon_shub_request	 *shub_req;
	struct aon_shub_response *shub_resp;
	/* Interface to the NVS framework */
	struct nvs_fn_if	 *nvs;
	struct aon_shub_sensor	 *snsr;
	struct mutex		shub_mutex;
	unsigned int		 snsr_cnt;
	u32			 i2c_clk_rates[I2CID_MAX];
	u32			 chip_id_mask;
	u32			 adjust_ts_counter;
	u64			 ts_res_ns;
	s64			 ts_adjustment;
};

/* This has to be a multiple of the cache line size */
static inline int ivc_min_frame_size(void)
{
	return cache_line_size();
}

static inline s64 get_ts_adjustment(u64 tsc_res)
{
	s64 tsc = 0;
	struct timespec64 ts;
	s64 delta = 0;
	s64 mono_time = 0;
	unsigned long flags;
	DEFINE_RAW_SPINLOCK(slock);

	raw_spin_lock_irqsave(&slock, flags);
	tsc = (s64)(arch_counter_get_cntvct() * tsc_res);

	ktime_get_ts64(&ts);
	mono_time = timespec_to_ns(&ts);

	delta = mono_time - tsc;
	raw_spin_unlock_irqrestore(&slock, flags);

	return delta;
}

static void tegra_aon_shub_mbox_rcv_msg(struct mbox_client *cl, void *rx_msg)
{
	struct tegra_aon_mbox_msg *msg = rx_msg;
	struct tegra_aon_shub *shub = dev_get_drvdata(cl->dev);
	struct aon_shub_response *shub_resp;
	int snsr_id;
	u32 i;
	s64 ts;

	shub_resp = (struct aon_shub_response *)msg->data;
	if (shub_resp->resp_type == AON_SHUB_REQUEST_PAYLOAD) {
		i = shub_resp->data.payload.count;
		if (i > ARRAY_SIZE(shub_resp->data.payload.data) || i == 0) {
			dev_err(shub->dev,
				"Invalid payload count\n");
			return;
		}
		if (shub->adjust_ts_counter == READJUST_TS_SAMPLES) {
			shub->ts_adjustment =
				get_ts_adjustment(shub->ts_res_ns);
			shub->adjust_ts_counter = 0;
		}
		shub->adjust_ts_counter++;
		while (i--) {
			snsr_id = shub_resp->data.payload.data[i].snsr_id;
			ts = (s64)shub_resp->data.payload.data[i].ts;
			ts += shub->ts_adjustment;
			shub_resp->data.payload.data[i].ts = (u64)ts;
			shub->nvs->handler(shub->snsr[snsr_id].nvs_st,
				&shub_resp->data.payload.data[i].x,
				shub_resp->data.payload.data[i].ts);
		}
	} else {
		memcpy(shub->shub_resp, msg->data, sizeof(*shub->shub_resp));
		complete(shub->wait_on);
	}
}

static int tegra_aon_shub_ivc_msg_send(struct tegra_aon_shub *shub, int len)
{
	int status;
	struct tegra_aon_mbox_msg msg;

	msg.length = len;
	msg.data = (void *)shub->shub_req;
	status = mbox_send_message(shub->mbox, (void *)&msg);
	if (status < 0) {
		dev_err(shub->dev, "mbox_send_message() failed with %d\n",
			status);
	} else {
		status = wait_for_completion_timeout(shub->wait_on,
							IVC_TIMEOUT);
		if (status == 0) {
			dev_err(shub->dev,
				"Timeout waiting for IVC response\n");
			return -ETIMEDOUT;
		}
		status = shub->shub_resp->status;
	}

	return status;
}

static int tegra_aon_shub_batch(void *client, int snsr_id, int flags,
				unsigned int period, unsigned int timeout)
{
	struct tegra_aon_shub *shub = (struct tegra_aon_shub *)client;
	int ret = 0;

	mutex_lock(&shub->shub_mutex);
	shub->shub_req->req_type = AON_SHUB_REQUEST_BATCH;
	shub->shub_req->data.batch.snsr_id = snsr_id;
	shub->shub_req->data.batch.flags = flags;
	shub->shub_req->data.batch.period = period;
	shub->shub_req->data.batch.timeout = timeout;
	ret = tegra_aon_shub_ivc_msg_send(shub,
					  sizeof(struct aon_shub_request));
	if (ret)
		dev_err(shub->dev, "%s : No response from AON SHUB...!\n",
			__func__);
	mutex_unlock(&shub->shub_mutex);

	return ret;
}

static int tegra_aon_shub_enable(void *client, int snsr_id, int enable)
{
	struct tegra_aon_shub *shub = (struct tegra_aon_shub *)client;
	int ret = 0;

	mutex_lock(&shub->shub_mutex);
	shub->shub_req->req_type = AON_SHUB_REQUEST_ENABLE;
	shub->shub_req->data.enable.snsr_id = snsr_id;
	shub->shub_req->data.enable.enable = enable;
	ret = tegra_aon_shub_ivc_msg_send(shub,
					  sizeof(struct aon_shub_request));
	if (ret) {
		dev_err(shub->dev, " %s : No response from AON SHUB...!\n",
			__func__);
	} else {
		ret = shub->shub_resp->data.enable.enable;
	}
	mutex_unlock(&shub->shub_mutex);

	return ret;
}

static struct nvs_fn_dev aon_shub_nvs_fn = {
	.enable	= tegra_aon_shub_enable,
	.batch	= tegra_aon_shub_batch,
};

#ifdef TEGRA_AON_SHUB_DBG_ENABLE
static void tegra_aon_shub_dbg_cfg(struct device *dev, struct sensor_cfg *cfg)
{
	unsigned int i;

	dev_dbg(dev, "name=%s\n", cfg->name);
	dev_dbg(dev, "name=%s\n", cfg->part);
	dev_dbg(dev, "name=%s\n", cfg->vendor);
	dev_dbg(dev, "snsr_id=%d\n", cfg->snsr_id);
	dev_dbg(dev, "timestamp_sz=%d\n", cfg->timestamp_sz);
	dev_dbg(dev, "snsr_data_n=%d\n", cfg->snsr_data_n);
	dev_dbg(dev, "kbuf_sz=%d\n", cfg->kbuf_sz);
	dev_dbg(dev, "ch_n=%u\n", cfg->ch_n);
	dev_dbg(dev, "ch_n_max=%u\n", cfg->ch_n_max);
	dev_dbg(dev, "ch_sz=%d\n", cfg->ch_sz);
	dev_dbg(dev, "delay_us_min=%u\n", cfg->delay_us_min);
	dev_dbg(dev, "delay_us_max=%u\n", cfg->delay_us_max);
	dev_dbg(dev, "matrix: ");
	for (i = 0; i < 9; i++)
		dev_dbg(dev, "%hhd ", cfg->matrix[i]);
	dev_dbg(dev, "\nScales:\n");
	for (i = 0; i < 3; i++) {
		dev_dbg(dev, " %d : ival: %u\n", i, cfg->scales[i].ival);
		dev_dbg(dev, " %d : fval: %u\n", i, cfg->scales[i].fval);
	}
	dev_dbg(dev, "maxrange:\n");
	dev_dbg(dev, " ival: %u ", cfg->max_range.ival);
	dev_dbg(dev, " fval: %u\n", cfg->max_range.fval);
	dev_dbg(dev, "resolution:\n");
	dev_dbg(dev, " ival: %u ", cfg->resolution.ival);
	dev_dbg(dev, " fval: %u\n", cfg->resolution.fval);
	dev_dbg(dev, "milliamp:\n");
	dev_dbg(dev, " ival: %u ", cfg->milliamp.ival);
	dev_dbg(dev, " fval: %u\n", cfg->milliamp.fval);
	dev_dbg(dev, "uncal_lo=%d\n", cfg->uncal_lo);
	dev_dbg(dev, "uncal_hi=%d\n", cfg->uncal_hi);
	dev_dbg(dev, "cal_lo=%d\n", cfg->cal_lo);
	dev_dbg(dev, "cal_hi=%d\n", cfg->cal_hi);
	dev_dbg(dev, "thresh_lo=%d\n", cfg->thresh_lo);
	dev_dbg(dev, "thresh_hi=%d\n", cfg->thresh_hi);
	dev_dbg(dev, "report_n=%d\n", cfg->report_n);
	dev_dbg(dev, "float_significance=%s\n",
			nvs_float_significances[cfg->float_significance]);
}
#endif

static int tegra_aon_shub_setup(struct tegra_aon_shub *shub,
				struct device_node *np)
{
	int ret;
	int i;
	struct device_node *cn;
	struct device *dev = shub->dev;
	struct aon_shub_init_setup_request *setup_req;
	u32 gpio, chip_id;
	u32 i2c_info[3];
	bool found;

	/* sanity check */
	if (!shub || !np)
		return -EINVAL;

	for_each_child_of_node(np, cn) {
		found = false;
		for (i = 0; i < shub->snsr_cnt; i++) {
			if (!strcmp(cn->name, shub->snsr[i].part)) {
				nvs_of_dt(cn, &shub->snsr[i].cfg, NULL);
				shub->snsr[i].genable = true;
				found = true;
			}
		}
		if (!found) {
			dev_err(dev, "No sensors on %s chip\n", cn->name);
			continue;
		}

		ret = of_property_read_u32(cn, "gpio", &gpio);
		if (ret) {
			dev_err(dev, "missing <%s> property\n", "gpio");
			return ret;
		}

		ret = of_property_read_u32_array(cn, "i2c_info", i2c_info, 3);
		if (ret) {
			dev_err(dev, "missing <%s> property\n", "i2c_info");
			return ret;
		}
		if (!(i2c_info[0] >= I2CID_MIN && i2c_info[0] <= I2CID_MAX)) {
			dev_err(dev, "Invalid I2C controller id\n");
			return -EINVAL;
		}

		ret = of_property_read_u32(cn, "chip_id", &chip_id);
		if (ret) {
			dev_err(dev, "missing <%s> property\n", "chip_id");
			return ret;
		}
		mutex_lock(&shub->shub_mutex);
		shub->shub_req->req_type = AON_SHUB_REQUEST_INIT;
		shub->shub_req->data.init.req = AON_SHUB_INIT_REQUEST_SETUP;
		setup_req = &shub->shub_req->data.init.data.setup;
		setup_req->chip_id = chip_id;
		setup_req->i2c_id = i2c_info[0];
		setup_req->i2c_addr = i2c_info[2];
		setup_req->gpio = gpio;
		ret = tegra_aon_shub_ivc_msg_send(shub,
					sizeof(struct aon_shub_request));
		if (ret) {
			dev_err(shub->dev, "No response from AON SHUB...!\n");
			goto err_exit;
		}
		if (shub->shub_resp->data.init.init_type !=
						AON_SHUB_INIT_REQUEST_SETUP) {
			ret = -EIO;
			goto err_exit;
		}

		ret = shub->shub_resp->data.init.status;
		if (ret) {
			dev_err(shub->dev, "%s setup failed ERR: %d\n",
				cn->name, ret);
			goto err_exit;
		}

		if (shub->i2c_clk_rates[i2c_info[0] - I2CID_MIN] < i2c_info[1])
			shub->i2c_clk_rates[i2c_info[0] - I2CID_MIN] =
								i2c_info[1];
		shub->chip_id_mask |= BIT(chip_id - 1);
		mutex_unlock(&shub->shub_mutex);
	}

	return 0;

err_exit:
	mutex_unlock(&shub->shub_mutex);
	return ret;
}

static inline int tegra_aon_shub_count_sensor_chips(struct device_node *dn)
{
	return of_get_child_count(dn);
}

static int tegra_aon_shub_get_snsr_cnt(struct tegra_aon_shub *shub)
{
	int ret = 0;

	mutex_lock(&shub->shub_mutex);
	shub->shub_req->req_type = AON_SHUB_REQUEST_SYS;
	shub->shub_req->data.sys.req = AON_SHUB_SYS_REQUEST_SNSR_CNT;
	ret = tegra_aon_shub_ivc_msg_send(shub,
					  sizeof(struct aon_shub_request));
	if (ret) {
		dev_err(shub->dev, "No response from AON SHUB...!\n");
		return ret;
	}
	shub->snsr_cnt = shub->shub_resp->data.sys.snsr_cnt;
	mutex_unlock(&shub->shub_mutex);

	return ret;
}

static int tegra_aon_shub_preinit(struct tegra_aon_shub *shub)
{
	int ret = 0;
	int i, j;
	int len;

	ret = tegra_aon_shub_get_snsr_cnt(shub);
	if (ret)
		return ret;

	shub->snsr = devm_kzalloc(shub->dev,
				sizeof(struct aon_shub_sensor) * shub->snsr_cnt,
				GFP_KERNEL);
	if (!shub->snsr)
		return -ENOMEM;

	mutex_lock(&shub->shub_mutex);
	for (i = 0; i < shub->snsr_cnt; i++) {
		shub->shub_req->req_type = AON_SHUB_REQUEST_SNSR_CFG;
		shub->shub_req->data.cfg.index = i;
		ret = tegra_aon_shub_ivc_msg_send(shub,
					sizeof(struct aon_shub_request));
		if (ret) {
			dev_err(shub->dev, "No response from AON SHUB..!\n");
			mutex_unlock(&shub->shub_mutex);
			return ret;
		}

		/* TODO: Should we rearrange sensor_cfg in nvs.h so that we
		 * can call memcpy() rather than individual field assignment.
		 */
		len = ARRAY_SIZE(shub->snsr[i].name);
		strncpy(shub->snsr[i].name,
			(char *)shub->shub_resp->data.cfg.name,
			len);
		shub->snsr[i].name[len - 1] = '\0';
		len = ARRAY_SIZE(shub->snsr[i].part);
		strncpy(shub->snsr[i].part,
			(char *)shub->shub_resp->data.cfg.part,
			len);
		shub->snsr[i].part[len - 1] = '\0';
		len = ARRAY_SIZE(shub->snsr[i].vendor);
		strncpy(shub->snsr[i].vendor,
			(char *)shub->shub_resp->data.cfg.vendor,
			len);
		shub->snsr[i].vendor[len - 1] = '\0';
		shub->snsr[i].cfg.name = shub->snsr[i].name;
		shub->snsr[i].cfg.part = shub->snsr[i].part;
		shub->snsr[i].cfg.vendor = shub->snsr[i].vendor;
		shub->snsr[i].cfg.version = shub->shub_resp->data.cfg.version;
		shub->snsr[i].cfg.snsr_id = shub->shub_resp->data.cfg.snsr_id;
		shub->snsr[i].cfg.kbuf_sz = shub->shub_resp->data.cfg.kbuf_sz;
		shub->snsr[i].cfg.timestamp_sz =
				shub->shub_resp->data.cfg.timestamp_sz;
		shub->snsr[i].cfg.snsr_data_n =
				shub->shub_resp->data.cfg.snsr_data_n;
		shub->snsr[i].cfg.ch_n = shub->shub_resp->data.cfg.ch_n;
		shub->snsr[i].cfg.ch_n_max = shub->shub_resp->data.cfg.ch_n_max;
		shub->snsr[i].cfg.ch_sz = shub->shub_resp->data.cfg.ch_sz;
		shub->snsr[i].cfg.max_range.ival =
				shub->shub_resp->data.cfg.max_range.ival;
		shub->snsr[i].cfg.max_range.fval =
				shub->shub_resp->data.cfg.max_range.fval;
		shub->snsr[i].cfg.resolution.ival =
				shub->shub_resp->data.cfg.resolution.ival;
		shub->snsr[i].cfg.resolution.fval =
				shub->shub_resp->data.cfg.resolution.fval;
		shub->snsr[i].cfg.milliamp.ival =
				shub->shub_resp->data.cfg.milliamp.ival;
		shub->snsr[i].cfg.milliamp.fval =
				shub->shub_resp->data.cfg.milliamp.fval;
		shub->snsr[i].cfg.delay_us_min =
				shub->shub_resp->data.cfg.delay_us_min;
		shub->snsr[i].cfg.delay_us_max =
				shub->shub_resp->data.cfg.delay_us_max;
		shub->snsr[i].cfg.fifo_rsrv_evnt_cnt =
				shub->shub_resp->data.cfg.fifo_rsrv_evnt_cnt;
		shub->snsr[i].cfg.fifo_max_evnt_cnt =
				shub->shub_resp->data.cfg.fifo_max_evnt_cnt;
		shub->snsr[i].cfg.flags = shub->shub_resp->data.cfg.flags;
		shub->snsr[i].cfg.uncal_lo = shub->shub_resp->data.cfg.uncal_lo;
		shub->snsr[i].cfg.uncal_hi = shub->shub_resp->data.cfg.uncal_hi;
		shub->snsr[i].cfg.cal_lo = shub->shub_resp->data.cfg.cal_lo;
		shub->snsr[i].cfg.cal_hi = shub->shub_resp->data.cfg.cal_hi;
		shub->snsr[i].cfg.thresh_lo =
				shub->shub_resp->data.cfg.thresh_lo;
		shub->snsr[i].cfg.thresh_hi =
				shub->shub_resp->data.cfg.thresh_hi;
		shub->snsr[i].cfg.float_significance =
				shub->shub_resp->data.cfg.float_significance;
		shub->snsr[i].cfg.scale.ival =
				shub->shub_resp->data.cfg.scale.ival;
		shub->snsr[i].cfg.scale.fval =
				shub->shub_resp->data.cfg.scale.fval;
		shub->snsr[i].cfg.offset.ival =
				shub->shub_resp->data.cfg.offset.ival;
		shub->snsr[i].cfg.offset.fval =
				shub->shub_resp->data.cfg.offset.fval;
		for (j = 0; j < 3; j++) {
			shub->snsr[i].cfg.scales[j].ival =
				shub->shub_resp->data.cfg.resolution.ival;
			shub->snsr[i].cfg.scales[j].fval =
				shub->shub_resp->data.cfg.resolution.fval;
		}
		for (j = 0; j < 9; j++) {
			shub->snsr[i].cfg.matrix[j] =
					shub->shub_resp->data.cfg.matrix[j];
		}
#ifdef TEGRA_AON_SHUB_DBG_ENABLE
		tegra_aon_shub_dbg_cfg(shub->dev, &shub->snsr[i].cfg);
#endif
	}
	mutex_unlock(&shub->shub_mutex);

	return ret;
}

static int tegra_aon_shub_init(struct tegra_aon_shub *shub)
{
	int ret = 0;
	int snsrs = 0;
	int i;
	bool i2c_inited = false;
	struct aon_shub_init_i2c_request *i2c_req;
	struct aon_shub_init_snsrs_request *snsrs_req;

	mutex_lock(&shub->shub_mutex);
	for (i = 0; i < ARRAY_SIZE(shub->i2c_clk_rates); i++) {
		if (!shub->i2c_clk_rates[i])
			continue;
		shub->shub_req->req_type = AON_SHUB_REQUEST_INIT;
		shub->shub_req->data.init.req = AON_SHUB_INIT_REQUEST_I2C;
		i2c_req = &shub->shub_req->data.init.data.i2c_init;
		i2c_req->i2c_id = i + 1;
		i2c_req->clk_rate = shub->i2c_clk_rates[i];
		ret = tegra_aon_shub_ivc_msg_send(shub,
					sizeof(struct aon_shub_request));
		if (ret) {
			dev_err(shub->dev,
				"%s : No response from AON SHUB...!\n",
				__func__);
			goto err_exit;
		}
		if (shub->shub_resp->data.init.init_type ==
						AON_SHUB_INIT_REQUEST_I2C) {
			if (shub->shub_resp->data.init.status) {
				dev_err(shub->dev, "I2C init failed\n");
				ret = -1;
				goto err_exit;
			}
		} else {
			dev_err(shub->dev,
				"Invalid response to I2C init request\n");
			goto err_exit;
		}
		i2c_inited = true;
	}
	if (!i2c_inited) {
		dev_err(shub->dev, "No I2C controllers inited\n");
		ret = -1;
		goto err_exit;
	}
	shub->shub_req->req_type = AON_SHUB_REQUEST_INIT;
	shub->shub_req->data.init.req = AON_SHUB_INIT_REQUEST_SNSR;
	snsrs_req = &shub->shub_req->data.init.data.snsrs_init;
	snsrs_req->chip_id_mask = shub->chip_id_mask;
	ret = tegra_aon_shub_ivc_msg_send(shub,
					sizeof(struct aon_shub_request));
	if (ret) {
		dev_err(shub->dev, "%s : No response from AON SHUB...!\n",
			__func__);
		goto err_exit;
	}
	if (shub->shub_resp->data.init.init_type ==
					AON_SHUB_INIT_REQUEST_SNSR) {
		if (shub->shub_resp->data.init.status) {
			dev_err(shub->dev, "Sensors init failed\n");
			ret = -1;
			goto err_exit;
		}
	} else {
		dev_err(shub->dev, "Invalid response to sensor init request\n");
		goto err_exit;
	}
	mutex_unlock(&shub->shub_mutex);

	shub->nvs = nvs_iio();
	if (shub->nvs == NULL)
		return -ENODEV;

	for (i = 0; i < shub->snsr_cnt; i++) {
		if (!shub->snsr[i].genable)
			continue;
		ret = shub->nvs->probe(&shub->snsr[i].nvs_st, (void *)shub,
					shub->dev,  &aon_shub_nvs_fn,
					&shub->snsr[i].cfg);
		if (!ret)
			snsrs++;
	}
	if (!snsrs)
		return -ENODEV;

	return 0;

err_exit:
	mutex_unlock(&shub->shub_mutex);
	return ret;
}

static int tegra_aon_shub_probe(struct platform_device *pdev)
{
	struct tegra_aon_shub *shub;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	int num_sensors;

	dev_dbg(dev, "AON SHUB driver probe()\n");

	if (!np) {
		dev_err(dev, "tegra_aon_shub: DT data required\n");
		return -EINVAL;
	}

	shub = devm_kzalloc(&pdev->dev, sizeof(*shub), GFP_KERNEL);
	if (!shub)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, shub);
	shub->dev = &pdev->dev;
	shub->cl.dev = &pdev->dev;
	shub->cl.tx_block = true;
	shub->cl.tx_tout = TX_BLOCK_PERIOD;
	shub->cl.knows_txdone = false;
	shub->cl.rx_callback = tegra_aon_shub_mbox_rcv_msg;
	shub->mbox = mbox_request_channel(&shub->cl, 0);
	if (IS_ERR(shub->mbox)) {
		ret = PTR_ERR(shub->mbox);
		if (ret != -EPROBE_DEFER)
			dev_warn(&pdev->dev, "can't get mailbox chan (%d)\n",
				 (int)PTR_ERR(shub->mbox));
		return ret;
	}
	dev_dbg(dev, "shub->mbox = %p\n", shub->mbox);

	shub->shub_req = devm_kzalloc(&pdev->dev, sizeof(*shub->shub_req),
					GFP_KERNEL);
	if (!shub->shub_req) {
		ret = -ENOMEM;
		goto exit_free_mbox;
	}

	shub->shub_resp = devm_kzalloc(&pdev->dev, sizeof(*shub->shub_resp),
					GFP_KERNEL);
	if (!shub->shub_resp) {
		ret = -ENOMEM;
		goto exit_free_mbox;
	}

	shub->wait_on = devm_kzalloc(&pdev->dev,
				sizeof(struct completion), GFP_KERNEL);
	if (!shub->wait_on) {
		ret = -ENOMEM;
		goto exit_free_mbox;
	}
	init_completion(shub->wait_on);
	mutex_init(&shub->shub_mutex);

	num_sensors = tegra_aon_shub_count_sensor_chips(np);
	if (num_sensors <= 0) {
		dev_err(dev, "No sensors on the shub\n");
		ret = -EINVAL;
		goto exit_free_mbox;
	}

	ret = tegra_aon_shub_preinit(shub);
	if (ret) {
		dev_err(dev, "shub pre-init failed\n");
		goto exit_free_mbox;
	}

	ret = tegra_aon_shub_setup(shub, np);
	if (ret) {
		dev_err(dev, "shub setup failed\n");
		goto exit_free_mbox;
	}

	ret = tegra_aon_shub_init(shub);
	if (ret)
		goto exit_free_mbox;

	shub->adjust_ts_counter = 0;
	#define _PICO_SECS (1000000000000ULL)
	shub->ts_res_ns = (_PICO_SECS / (u64)arch_timer_get_cntfrq())/1000;
	#undef _PICO_SECS
	shub->ts_adjustment = get_ts_adjustment(shub->ts_res_ns);

	dev_info(&pdev->dev, "tegra_aon_shub_driver_probe() OK\n");

	return 0;

exit_free_mbox:
	mbox_free_channel(shub->mbox);
	dev_err(&pdev->dev, "tegra_aon_shub_driver_probe() FAILED\n");

	return ret;
}

static int tegra_aon_shub_remove(struct platform_device *pdev)
{
	struct tegra_aon_shub *shub;

	shub  = dev_get_drvdata(&pdev->dev);
	mbox_free_channel(shub->mbox);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_aon_shub_suspend(struct device *dev)
{
	/* TODO: */
	return 0;
}

static int tegra_aon_shub_resume(struct device *dev)
{
	/* TODO: */
	return 0;
}

static const struct dev_pm_ops tegra_aon_shub_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tegra_aon_shub_suspend, tegra_aon_shub_resume)
};

#endif

static const struct of_device_id tegra_aon_shub_of_match[] = {
	{
		.compatible = "nvidia,tegra186_aon_shub",
	},
	{},
};
MODULE_DEVICE_TABLE(of, tegra_aon_shub_of_match);

static struct platform_driver tegra_aon_shub_driver = {
	.driver = {
		.name		= "tegra-aon-shub",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(tegra_aon_shub_of_match),
#ifdef CONFIG_PM_SLEEP
		.pm = &tegra_aon_shub_pm_ops,
#endif
	},
	.probe =	tegra_aon_shub_probe,
	.remove =	tegra_aon_shub_remove,
};
module_platform_driver(tegra_aon_shub_driver);

MODULE_DESCRIPTION("NVIDIA Tegra186 AON Sensor Hub Driver");
MODULE_AUTHOR("NVIDIA");
MODULE_LICENSE("GPL v2");
