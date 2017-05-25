/**
 * camera_common.h - utilities for tegra camera driver
 *
 * Copyright (c) 2015-2017, NVIDIA Corporation.  All rights reserved.
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

#ifndef __camera_common__
#define __camera_common__

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/v4l2-mediabus.h>
#include <linux/version.h>
#include <linux/videodev2.h>

#include <media/camera_version_utils.h>
#include <media/nvc_focus.h>
#include <media/sensor_common.h>
#include <media/soc_camera.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

/*
 * Scaling factor for converting a Q10.22 fixed point value
 * back to its original floating point value
 */
#define FIXED_POINT_SCALING_FACTOR (1ULL << 22)

struct reg_8 {
	u16 addr;
	u8 val;
};

struct reg_16 {
	u16 addr;
	u16 val;
};

struct camera_common_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
	struct regulator *vcmvdd;
	struct clk *mclk;
	unsigned int pwdn_gpio;
	unsigned int reset_gpio;
	unsigned int af_gpio;
	bool state;
};

struct camera_common_regulators {
	const char *avdd;
	const char *dvdd;
	const char *iovdd;
	const char *vcmvdd;
};

struct camera_common_pdata {
	const char *mclk_name; /* NULL for default default_mclk */
	const char *parentclk_name; /* NULL for no parent clock*/
	unsigned int pwdn_gpio;
	unsigned int reset_gpio;
	unsigned int af_gpio;
	bool ext_reg;
	int (*power_on)(struct camera_common_power_rail *pw);
	int (*power_off)(struct camera_common_power_rail *pw);
	struct camera_common_regulators regulators;
	bool use_cam_gpio;
	bool has_eeprom;
};

struct camera_common_eeprom_data {
	struct i2c_client *i2c_client;
	struct i2c_adapter *adap;
	struct i2c_board_info brd;
	struct regmap *regmap;
};

int
regmap_util_write_table_8(struct regmap *regmap,
			  const struct reg_8 table[],
			  const struct reg_8 override_list[],
			  int num_override_regs,
			  u16 wait_ms_addr, u16 end_addr);

int
regmap_util_write_table_16_as_8(struct regmap *regmap,
				const struct reg_16 table[],
				const struct reg_16 override_list[],
				int num_override_regs,
				u16 wait_ms_addr, u16 end_addr);

enum switch_state {
	SWITCH_OFF,
	SWITCH_ON,
};

static const s64 switch_ctrl_qmenu[] = {
	SWITCH_OFF, SWITCH_ON
};

/*
 * The memory buffers allocated from nvrm are aligned to
 * fullfill the hardware requirements:
 * - size in alignment with a multiple of 128K/64K bytes,
 * see CL http://git-master/r/256468 and bug 1321091.
 */
static const s64 size_align_ctrl_qmenu[] = {
	1, (64 * 1024), (128 * 1024),
};

struct camera_common_frmfmt {
	struct v4l2_frmsize_discrete	size;
	const int	*framerates;
	int	num_framerates;
	bool	hdr_en;
	int	mode;
};

struct camera_common_colorfmt {
	unsigned int			code;
	enum v4l2_colorspace		colorspace;
	int				pix_fmt;
	enum v4l2_xfer_func		xfer_func;
	enum v4l2_ycbcr_encoding	ycbcr_enc;
	enum v4l2_quantization		quantization;
};

struct camera_common_framesync {
	u32 inck;		/* kHz */
	u32 xhs;		/* in inck */
	u32 xvs;		/* in xhs */
	u32 fps;		/* frames in 1000 second */
};

struct camera_common_data;

struct camera_common_sensor_ops {
	int (*power_on)(struct camera_common_data *s_data);
	int (*power_off)(struct camera_common_data *s_data);
	int (*write_reg)(struct camera_common_data *s_data,
	  u16 addr, u8 val);
	int (*read_reg)(struct camera_common_data *s_data,
	  u16 addr, u8 *val);
	int (*get_framesync)(struct camera_common_data *s_data,
		struct camera_common_framesync *vshs);
};

struct camera_common_data {
	struct camera_common_sensor_ops		*ops;
	struct v4l2_ctrl_handler		*ctrl_handler;
	struct device				*dev;
	const struct camera_common_frmfmt	*frmfmt;
	const struct camera_common_colorfmt	*colorfmt;
	const struct camera_common_colorfmt	*color_fmts;
	struct dentry				*debugdir;
	struct camera_common_power_rail		*power;

	struct v4l2_subdev			subdev;
	struct v4l2_ctrl			**ctrls;

	struct sensor_properties		sensor_props;

	void	*priv;
	int	numctrls;
	int	csi_port;
	int	numlanes;
	int	mode;
	int	numfmts;
	int	num_color_fmts;
	int	def_mode, def_width, def_height;
	int	def_clk_freq;
	int	fmt_width, fmt_height;
	int	sensor_mode_id;
	bool	use_sensor_mode_id;
	bool	override_enable;
};

struct camera_common_focuser_data;

struct camera_common_focuser_ops {
	int (*power_on)(struct camera_common_focuser_data *s_data);
	int (*power_off)(struct camera_common_focuser_data *s_data);
	int (*load_config)(struct camera_common_focuser_data *s_data);
	int (*ctrls_init)(struct camera_common_focuser_data *s_data);
};

struct camera_common_focuser_data {
	struct camera_common_focuser_ops	*ops;
	struct v4l2_ctrl_handler		*ctrl_handler;
	struct v4l2_subdev			subdev;
	struct v4l2_ctrl			**ctrls;
	struct device				*dev;

	struct nv_focuser_config		config;
	void					*priv;
	int					pwr_dev;
	int					def_position;
};

static inline void msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base * 1000, delay_base * 1000 + 500);
}

static inline struct camera_common_data *to_camera_common_data(
	const struct device *dev)
{
	return container_of(dev_get_drvdata(dev),
			    struct camera_common_data, subdev);
}

static inline struct camera_common_focuser_data *to_camera_common_focuser_data(
	const struct device *dev)
{
	return container_of(dev_get_drvdata(dev),
			    struct camera_common_focuser_data, subdev);
}

int camera_common_g_ctrl(struct camera_common_data *s_data,
			 struct v4l2_control *control);

int camera_common_regulator_get(struct device *dev,
		       struct regulator **vreg, const char *vreg_name);
int camera_common_parse_clocks(struct device *dev,
			struct camera_common_pdata *pdata);
int camera_common_parse_ports(struct device *dev,
			      struct camera_common_data *s_data);

int camera_common_debugfs_show(struct seq_file *s, void *unused);
ssize_t camera_common_debugfs_write(
	struct file *file,
	char const __user *buf,
	size_t count,
	loff_t *offset);
int camera_common_debugfs_open(struct inode *inode, struct file *file);
void camera_common_remove_debugfs(struct camera_common_data *s_data);
void camera_common_create_debugfs(struct camera_common_data *s_data,
		const char *name);

const struct camera_common_colorfmt *camera_common_find_datafmt(
		unsigned int code);
int camera_common_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code);
int camera_common_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			unsigned int *code);
int camera_common_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf);
int camera_common_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf);
int camera_common_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf);
int camera_common_enum_framesizes(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_size_enum *fse);
int camera_common_enum_frameintervals(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_interval_enum *fie);
int camera_common_set_power(struct camera_common_data *data, int on);
int camera_common_s_power(struct v4l2_subdev *sd, int on);
void camera_common_dpd_disable(struct camera_common_data *s_data);
void camera_common_dpd_enable(struct camera_common_data *s_data);
int camera_common_g_mbus_config(struct v4l2_subdev *sd,
			      struct v4l2_mbus_config *cfg);
int camera_common_get_framesync(struct v4l2_subdev *sd,
		struct camera_common_framesync *vshs);

/* Focuser */
int camera_common_focuser_init(struct camera_common_focuser_data *s_data);
int camera_common_focuser_s_power(struct v4l2_subdev *sd, int on);

#endif /* __camera_common__ */
