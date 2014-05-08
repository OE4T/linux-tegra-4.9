/*
* Copyright (C) 2012 Invensense, Inc.
* Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _NVI_H_
#define _NVI_H_

#include <linux/i2c.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/mpu_iio.h>
#include <linux/regulator/consumer.h>

#define MPU6050_ID			(0x68)
#define MPU6500_ID			(0x70)
#define MPU6515_ID			(0x74)
#define MPU9250_ID			(0x71)
#define MPU9350_ID			(0x72)
#define ICM20628_ID			(0xA2)

#define NVI_BYPASS_TIMEOUT_MS		(1000)
#define NVI_DELAY_US_MAX		(256000)
#define NVI_DELAY_DEFAULT		(100000)
#define POWER_UP_TIME			(100)
#define REG_UP_TIME			(5)
#define POR_MS				(100)
#define GYRO_STARTUP_DELAY_NS		(100000000) /* 100ms */

#define NVI_FIFO_SAMPLE_SIZE_MAX	(38)
#define TIMESTAMP_FIFO_SIZE		(64)
#define FIFO_THRESHOLD			(800)
#define FIFO_RESERVED_EVENT_COUNT	(64)
#define FIFO_MAX_EVENT_COUNT		(1024)

#define MOTION_DURATION			(128)
#define MOTION_THRESHOLD		(128)
#define SELF_TEST_GYRO_FULL_SCALE	(250)
#define SELF_TEST_ACCEL_FULL_SCALE	(8)
#define SELF_TEST_ACCEL_6500_SCALE	(2)

#define AXIS_X				(0)
#define AXIS_Y				(1)
#define AXIS_Z				(2)
#define AXIS_N				(3)
#define DEV_MASTER			(0)
#define DEV_ACCEL			(1)
#define DEV_ANGLVEL			(2)
#define DEV_TEMP			(3)
#define DEV_TEMP_EN			(1 << 0)
#define DEV_TEMP_GYRO			(1 << 1)
#define DEV_N				(4)
#define DEV_AUX				(5)
#define DEV_DMP				(6)
#define EN_STDBY			(16)
#define EN_LPA				(17)
#define EN_FW				(18)
#define EN_SELF_TEST			(19)
#define DEV_MPU_MASK			((1 << DEV_ACCEL) | \
					(1 << DEV_ANGLVEL) | \
					(1 << DEV_TEMP))
#define DEV_PM_ON_FULL			(1 << DEV_ANGLVEL)
#define DEV_PM_ON			((1 << DEV_TEMP) | \
					(1 << DEV_AUX) | \
					(1 << DEV_DMP))
#define DEV_PM_LPA			((1 << EN_LPA) | (1 << DEV_ACCEL))
#define DEV_PM_STDBY			((1 << EN_STDBY) | (1 << EN_FW))
#define NVI_PM_ERR			(0)
#define NVI_PM_AUTO			(1)
#define NVI_PM_OFF_FORCE		(2)
#define NVI_PM_OFF			(3)
#define NVI_PM_STDBY			(4)
#define NVI_PM_ON_CYCLE			(5)
#define NVI_PM_ON			(6)
#define NVI_PM_ON_FULL			(7)
#define INV_CLK_INTERNAL		(0)
#define INV_CLK_PLL			(1)

#define NVI_DBG_SPEW_MSG		(1 << 0)
#define NVI_DBG_SPEW_AUX		(1 << 1)
#define NVI_DBG_SPEW_ANGLVEL		(1 << 2)
#define NVI_DBG_SPEW_TEMP		(1 << 3)
#define NVI_DBG_SPEW_ACCEL		(1 << 4)
#define NVI_DBG_SPEW_ACCEL_UC		(1 << 5)
#define NVI_DBG_SPEW_FIFO		(1 << 6)
#define NVI_DBG_SPEW_BUF		(1 << 7)
#define NVI_DBG_SPEW_IRQ		(1 << 8)
#define NVI_DBG_ACCEL_AXIS_X		(1 << 9)
#define NVI_DBG_ACCEL_AXIS_Y		(1 << 10)
#define NVI_DBG_ACCEL_AXIS_Z		(1 << 11)
#define NVI_DBG_ANGLVEL_AXIS_X		(1 << 12)
#define NVI_DBG_ANGLVEL_AXIS_Y		(1 << 13)
#define NVI_DBG_ANGLVEL_AXIS_Z		(1 << 14)
#define NVI_DBG_TEMP_VAL		(1 << 15)
/* register bits */
#define BITS_SELF_TEST_EN		(0xE0)
#define BIT_ACCEL_FCHOCIE_B		(0x08)
#define BIT_FIFO_SIZE_1K		(0x40)
#define BIT_ACCEL_OUT			(0x08)
#define BIT_GYRO_ZOUT			(0x10)
#define BIT_GYRO_YOUT			(0x20)
#define BIT_GYRO_XOUT			(0x40)
#define BITS_GYRO_OUT			(0x70)
#define BIT_TEMP_OUT			(0x80)
#define BIT_I2C_MST_P_NSR		(0x10)
#define BIT_SLV3_FIFO_EN		(0x20)
#define BIT_WAIT_FOR_ES			(0x40)
#define BIT_I2C_READ			(0x80)
#define BITS_I2C_SLV_CTRL_LEN		(0x0F)
#define BIT_I2C_SLV_REG_DIS		(0x10)
#define BIT_SLV_EN			(0x80)
#define BITS_I2C_MST_DLY		(0x1F)
#define BIT_BYPASS_EN			(0x02)
#define BIT_DATA_RDY_EN			(0x01)
#define BIT_DMP_INT_EN			(0x02)
#define BIT_FIFO_OVERFLOW		(0x10)
#define BIT_ZMOT_EN			(0x20)
#define BIT_MOT_EN			(0x40)
#define BIT_6500_WOM_EN			(0x40)
#define BIT_SLV0_DLY_EN			(0x01)
#define BIT_SLV1_DLY_EN			(0x02)
#define BIT_SLV2_DLY_EN			(0x04)
#define BIT_SLV3_DLY_EN			(0x08)
#define BIT_DELAY_ES_SHADOW		(0x80)
#define BIT_ACCEL_INTEL_MODE		(0x40)
#define BIT_ACCEL_INTEL_ENABLE		(0x80)
#define BITS_USER_CTRL_RST		(0x0F)
#define BIT_SIG_COND_RST		(0x01)
#define BIT_I2C_MST_RST			(0x02)
#define BIT_FIFO_RST			(0x04)
#define BIT_DMP_RST			(0x08)
#define BIT_I2C_MST_EN			(0x20)
#define BIT_FIFO_EN			(0x40)
#define BIT_DMP_EN			(0x80)
#define BIT_CLK_MASK			(0x07)
#define BIT_CYCLE			(0x20)
#define BIT_SLEEP			(0x40)
#define BIT_H_RESET			(0x80)
#define BIT_STBY_ZG			(0x01)
#define BIT_STBY_YG			(0x02)
#define BIT_STBY_XG			(0x04)
#define BIT_STBY_ZA			(0x08)
#define BIT_STBY_YA			(0x10)
#define BIT_STBY_XA			(0x20)
#define BIT_PWR_GYRO_STBY		(0x07)
#define BIT_PWR_ACCEL_STBY		(0x38)
#define BIT_LPA_FREQ			(0xC0)

#define AUX_PORT_MAX			(5)
#define AUX_PORT_SPECIAL		(4)
#define AUX_PORT_BYPASS			(-1)
#define AUX_EXT_DATA_REG_MAX		(24)
#define AUX_DEV_VALID_READ_LOOP_MAX	(20)
#define AUX_DEV_VALID_READ_DELAY_MS	(5)


enum nvi_part {
	MPU6050 = 1,
	MPU6500,
	MPU6515,
	ICM20628,
};

enum inv_accel_fs_e {
	INV_FS_02G = 0,
	INV_FS_04G,
	INV_FS_08G,
	INV_FS_16G,
	NUM_ACCEL_FSR
};

enum inv_fsr_e {
	INV_FSR_250DPS = 0,
	INV_FSR_500DPS,
	INV_FSR_1000DPS,
	INV_FSR_2000DPS,
	NUM_FSR
};

struct nvi_iio_float {
	int ival;
	int micro;
};

struct nvi_rr {
	struct nvi_iio_float max_range;
	struct nvi_iio_float resolution;
};

struct nvi_hal_dev {
	int version;
	struct nvi_rr *rr;
	struct nvi_iio_float scale;
	struct nvi_iio_float offset;
	const char *power_ma;
};

struct nvi_br {
	u8 bank;
	u8 reg;
};

struct nvi_hal_reg {
	struct nvi_br self_test_x_gyro;
	struct nvi_br self_test_y_gyro;
	struct nvi_br self_test_z_gyro;
	struct nvi_br self_test_x_accel;
	struct nvi_br self_test_y_accel;
	struct nvi_br self_test_z_accel;
	struct nvi_br xg_offset_h;
	struct nvi_br yg_offset_h;
	struct nvi_br zg_offset_h;
	struct nvi_br a_offset_h[AXIS_N];
	struct nvi_br smplrt_div;
	struct nvi_br config;
	struct nvi_br gyro_config;
	struct nvi_br accel_config;
	struct nvi_br accel_config2;
	struct nvi_br lp_accel_odr;
	struct nvi_br mot_thr;
	struct nvi_br mot_dur;
	struct nvi_br fifo_en;
	struct nvi_br i2c_mst_ctrl;
	struct nvi_br i2c_slv0_addr;
	struct nvi_br i2c_slv0_reg;
	struct nvi_br i2c_slv0_ctrl;
	struct nvi_br i2c_slv4_ctrl;
	struct nvi_br i2c_mst_status;
	struct nvi_br int_pin_cfg;
	struct nvi_br int_enable;
	struct nvi_br int_status;
	struct nvi_br accel_xout_h;
	struct nvi_br temp_out_h;
	struct nvi_br gyro_xout_h;
	struct nvi_br ext_sens_data_00;
	struct nvi_br i2c_slv0_do;
	struct nvi_br i2c_slv4_do;
	struct nvi_br i2c_slv4_di;
	struct nvi_br i2c_mst_delay_ctrl;
	struct nvi_br signal_path_reset;
	struct nvi_br accel_intel_ctrl;
	struct nvi_br user_ctrl;
	struct nvi_br pwr_mgmt_1;
	struct nvi_br pwr_mgmt_2;
	struct nvi_br fifo_count_h;
	struct nvi_br fifo_r_w;
	struct nvi_br who_am_i;
	struct nvi_br reg_bank_sel;
};

struct nvi_rc {
	u8 reg_bank_sel;
	u16 accel_offset[AXIS_N];
	u16 gyro_offset[AXIS_N];
	u8 smplrt_div;
	u8 config;
	u8 gyro_config;
	u8 accel_config;
	u8 accel_config2;
	u8 lp_accel_odr;
	u8 mot_thr;
	u8 mot_dur;
	u8 fifo_en;
	u8 i2c_mst_ctrl;
	u8 i2c_slv_addr[AUX_PORT_MAX];
	u8 i2c_slv_reg[AUX_PORT_MAX];
	u8 i2c_slv_ctrl[AUX_PORT_SPECIAL];
	u8 i2c_slv4_ctrl;
	u8 int_pin_cfg;
	u8 int_enable;
	u8 i2c_slv_do[AUX_PORT_SPECIAL];
	u8 i2c_slv4_do;
	u8 i2c_mst_delay_ctrl;
	u8 mot_detect_ctrl;
	u8 user_ctrl;
	u8 pwr_mgmt_1;
	u8 pwr_mgmt_2;
};

struct nvi_hal {
	enum nvi_part part;
	const char *part_name;
	unsigned int regs_n;
	unsigned int reg_bank_n;
	bool dmp;
	unsigned int fifo_size;
	unsigned int min_delay_us;
	const unsigned long *lpa_tbl;
	unsigned int lpa_tbl_n;
	const struct nvi_hal_reg *reg;
	const struct nvi_hal_dev *accel;
	const struct nvi_hal_dev *anglvel;
	const struct nvi_hal_dev *temp;
	int accel_self_test_scale;
	int anglvel_self_test_scale;
};

struct aux_port {
	struct nvi_mpu_port nmp;
	unsigned short ext_data_offset;
	bool hw_valid;
	bool hw_en;
	bool hw_do;
	bool enable;
	bool fifo_en;
	unsigned int batch_flags;
	unsigned int batch_period_us;
	unsigned int batch_timeout_ms;
};

struct aux_ports {
	struct aux_port port[AUX_PORT_MAX];
	s64 bypass_timeout_ns;
	unsigned int bypass_lock;
	u8 delay_hw;
	unsigned short ext_data_n;
	unsigned char ext_data[AUX_EXT_DATA_REG_MAX];
	unsigned char clock_i2c;
	bool reset_i2c;
	bool reset_fifo;
};

/**
 *  struct inv_chip_config_s - Cached chip configuration data.
 *  @fsr:		Full scale range.
 *  @lpf:		Digital low pass filter frequency.
 *  @accel_fs:		accel full scale range.
 *  @has_footer:	MPU3050 specific work around.
 *  @has_compass:	has compass or not.
 *  @has_pressure:      has pressure sensor or not.
 *  @enable:		master enable to enable output
 *  @accel_enable:	enable accel functionality
 *  @gyro_enable:	enable gyro functionality
 *  @is_asleep:		1 if chip is powered down.
 *  @dmp_on:		dmp is on/off.
 *  @dmp_int_on:        dmp interrupt on/off.
 *  @step_indicator_on: step indicate bit added to the sensor or not.
 *  @dmp_event_int_on:  dmp event interrupt on/off.
 *  @firmware_loaded:	flag indicate firmware loaded or not.
 *  @lpa_mod:		low power mode.
 *  @display_orient_on:	display orientation on/off.
 *  @normal_compass_measure: discard first compass data after reset.
 *  @normal_pressure_measure: discard first pressure data after reset.
 *  @smd_enable: disable/enable SMD function.
 *  @adjust_time: flag to indicate whether adjust chip clock or not.
 *  @smd_triggered: smd is triggered.
 *  @lpa_freq:		low power frequency
 *  @prog_start_addr:	firmware program start address.
 *  @fifo_rate:		current FIFO update rate.
 *  @bytes_per_datum: number of bytes for 1 sample data.
 */
struct inv_chip_config_s {
	u32 fsr:2;
	u32 lpf:3;
	u32 accel_fs:2;
	u32 has_footer:1;
	u32 has_compass:1;
	u32 has_pressure:1;
	u32 enable:1;
	u32 accel_enable:1;
	u32 gyro_enable:1;
	u32 is_asleep:1;
	u32 dmp_on:1;
	u32 dmp_int_on:1;
	u32 dmp_event_int_on:1;
	u32 step_indicator_on:1;
	u32 firmware_loaded:1;
	u32 lpa_mode:1;
	u32 display_orient_on:1;
	u32 normal_compass_measure:1;
	u32 normal_pressure_measure:1;
	u32 smd_enable:1;
	u32 adjust_time:1;
	u32 smd_triggered:1;
	u16 lpa_freq;
	u16 prog_start_addr;
	u16 fifo_rate;
	u16 bytes_per_datum;

	s64 gyro_start_delay_ns;
	unsigned int lpa_delay_us;
	unsigned int bypass_timeout_ms;
	unsigned int temp_fifo_en;
	unsigned int fifo_thr;
	unsigned int fifo_reserve;
	unsigned int fifo_max;
};

/**
 *  struct inv_chip_info_s - Chip related information.
 *  @product_id:	Product id.
 *  @product_revision:	Product revision.
 *  @silicon_revision:	Silicon revision.
 *  @software_revision:	software revision.
 *  @multi:		accel specific multiplier.
 *  @compass_sens:	compass sensitivity.
 *  @gyro_sens_trim:	Gyro sensitivity trim factor.
 *  @accel_sens_trim:    accel sensitivity trim factor.
 */
struct inv_chip_info_s {
	u8 product_id;
	u8 product_revision;
	u8 silicon_revision;
	u8 software_revision;
	u8 multi;
	u8 compass_sens[3];
	u32 gyro_sens_trim;
	u32 accel_sens_trim;
};

/**
 *  struct inv_tap structure to store tap data.
 *  @min_count:  minimum taps counted.
 *  @thresh:    tap threshold.
 *  @time:	tap time.
 *  @on: tap on/off.
 */
struct inv_tap {
	u16 min_count;
	u16 thresh;
	u16 time;
	bool on;
};

/**
 *  struct accel_mot_int_s structure to store motion interrupt data
 *  @mot_thr:    motion threshold.
 *  @mot_dur:    motion duration.
 *  @mot_on:     flag to indicate motion detection on;
 */
struct accel_mot_int {
	u16 mot_thr;
	u32 mot_dur;
	u8 mot_on:1;
};

/**
 * struct self_test_setting - self test settables from sysfs
 * samples: number of samples used in self test.
 * threshold: threshold fail/pass criterion in self test.
 *            This value is in the percentage multiplied by 100.
 *            So 14% would be 14.
 */
struct self_test_setting {
	u16 samples;
	u16 threshold;
};

/**
 * struct inv_smd significant motion detection structure.
 * @threshold: accel threshold for motion detection.
 * @delay: delay time to confirm 2nd motion.
 * @delay2: delay window parameter.
 */
struct inv_smd {
	u32 threshold;
	u32 delay;
	u32 delay2;
};

/**
 * struct inv_ped pedometer related data structure.
 * @step: steps taken.
 * @time: time taken during the period.
 * @last_step_time: last time the step is taken.
 * @step_thresh: step threshold to show steps.
 * @int_thresh: step threshold to generate interrupt.
 * @int_on:   pedometer interrupt enable/disable.
 * @on:  pedometer on/off.
 */
struct inv_ped {
	u64 step;
	u64 time;
	u64 last_step_time;
	u16 step_thresh;
	u16 int_thresh;
	bool int_on;
	bool on;
};


struct nvi_state {
	struct i2c_client *i2c;
	struct iio_trigger *trig;
	struct regulator_bulk_data vreg[2];
	struct notifier_block nb_vreg[2];
	struct mutex srlock;
	struct mpu_platform_data pdata;
	const struct nvi_hal *hal;	/* Hardware Abstraction Layer */
	struct nvi_rc rc;
	struct aux_ports aux;
	s64 vreg_en_ts[2];
	unsigned int info;		/* info data to return */
	unsigned int dbg;		/* debug flags */
	unsigned int errs;		/* error count */
	unsigned int master_enable;	/* global enable */
	unsigned int enable[DEV_N];	/* enable status */
	unsigned int poll_delay_us;	/* global sampling delay */
	unsigned int delay_us[DEV_N];	/* device sampling delay */
	unsigned int batch_flags[DEV_N]; /* batch flags */
	unsigned int batch_period_us[DEV_N]; /* batch period us */
	unsigned int batch_timeout_ms[DEV_N]; /* batch timeout ms */
	unsigned short i2c_addr;	/* I2C address */
	bool shutdown;
	bool suspend;
	bool flush;
	bool rc_dis;
	bool irq_dis;
	int pm;

	struct inv_chip_config_s chip_config;
	struct inv_chip_info_s chip_info;
	struct self_test_setting self_test;
	struct inv_tap tap;
	struct inv_ped ped;
	struct accel_mot_int mot_int;
	int accel_bias[AXIS_N];
	int gyro_bias[AXIS_N];
	s16 input_accel_offset[AXIS_N];
	s16 input_gyro_offset[AXIS_N];
	int input_accel_dmp_bias[AXIS_N];
	int input_gyro_dmp_bias[AXIS_N];
	s16 rom_accel_offset[AXIS_N];
	s16 rom_gyro_offset[AXIS_N];

	DECLARE_KFIFO(timestamps, s64, TIMESTAMP_FIFO_SIZE);
	spinlock_t time_stamp_lock;
	u16 fifo_sample_size;
	s16 accel_uc[AXIS_N];
	s16 accel[AXIS_N];
	s16 anglvel[AXIS_N];
	s16 temp;
	s64 ts;
	s64 fifo_ts;
	s64 push_ts;
	u8 buf[NVI_FIFO_SAMPLE_SIZE_MAX * 2]; /* (* 2)=FIFO OVERFLOW OFFSET */
#ifdef NVI_I2C_DEBUG_INTERFACE
	u16 dbg_i2c_addr;
	u8 dbg_reg;
#endif /* NVI_I2C_DEBUG_INTERFACE */
};

int nvi_i2c_read(struct nvi_state *st, u16 addr, u8 reg, u16 len, u8 *buf);
int nvi_i2c_rd(struct nvi_state *st, u8 bank, u8 reg, u16 len, u8 *buf);
int nvi_i2c_write(struct nvi_state *st, u16 addr, u16 len, u8 *buf);
int nvi_i2c_wr(struct nvi_state *st, u8 reg, u8 val);
int nvi_wr_accel_offset(struct nvi_state *st, unsigned int axis, u16 offset);
int nvi_wr_gyro_offset(struct nvi_state *st, unsigned int axis, u16 offset);
int nvi_wr_smplrt_div(struct nvi_state *st, u8 smplrt_div);
int nvi_wr_config(struct nvi_state *st, u8 val);
int nvi_wr_gyro_config(struct nvi_state *st, u8 test, u8 fsr);
int nvi_wr_accel_config2(struct nvi_state *st, u8 val);
int nvi_wr_accel_config(struct nvi_state *st, u8 test, u8 fsr, u8 hpf);
int nvi_wr_fifo_en(struct nvi_state *st, u8 fifo_en);
int nvi_wr_int_enable(struct nvi_state *st, u8 int_enable);
int nvi_wr_user_ctrl(struct nvi_state *st, u8 user_ctrl);
int nvi_user_ctrl_en(struct nvi_state *st, bool fifo_enable, bool i2c_enable);
int nvi_wr_pwr_mgmt_1(struct nvi_state *st, u8 pwr_mgmt_1);
int nvi_pm_wr(struct nvi_state *st, u8 pwr_mgmt_1, u8 pwr_mgmt_2, u8 lpa);
int nvi_pm(struct nvi_state *st, int pm_req);
int nvi_enable(struct iio_dev *indio_dev);

int inv_get_silicon_rev_mpu6050(struct nvi_state *st);
int inv_get_silicon_rev_mpu6500(struct nvi_state *st);
int inv_hw_self_test(struct iio_dev *indio_dev);
int inv_create_dmp_sysfs(struct iio_dev *ind);

u16 inv_dmp_get_address(u16 key);

#endif /* _NVI_H_ */

