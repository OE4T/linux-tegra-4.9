/* Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
* Copyright (C) 2012 Invensense, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/crc32.h>
#include <linux/mpu_iio.h>
#include <linux/iio/iio.h>

#include "nvi.h"
#include "dmpKey.h"

#define mem_w(a, b, c) \
	mpu_memory_write(st, st->i2c_addr, a, b, c)
#define mem_w_key(key, b, c) mpu_memory_write_unaligned(st, key, b, c)

enum inv_filter_e {
	INV_FILTER_256HZ_NOLPF2 = 0,
	INV_FILTER_188HZ,
	INV_FILTER_98HZ,
	INV_FILTER_42HZ,
	INV_FILTER_20HZ,
	INV_FILTER_10HZ,
	INV_FILTER_5HZ,
	INV_FILTER_2100HZ_NOLPF,
	NUM_FILTER
};

/*==== MPU6050B1 MEMORY ====*/
enum MPU_MEMORY_BANKS {
	MEM_RAM_BANK_0 = 0,
	MEM_RAM_BANK_1,
	MEM_RAM_BANK_2,
	MEM_RAM_BANK_3,
	MEM_RAM_BANK_4,
	MEM_RAM_BANK_5,
	MEM_RAM_BANK_6,
	MEM_RAM_BANK_7,
	MEM_RAM_BANK_8,
	MEM_RAM_BANK_9,
	MEM_RAM_BANK_10,
	MEM_RAM_BANK_11,
	MPU_MEM_NUM_RAM_BANKS,
	MPU_MEM_OTP_BANK_0 = 16
};

/* produces an unique identifier for each device based on the
   combination of product version and product revision */
struct prod_rev_map_t {
	u16 mpl_product_key;
	u8 silicon_rev;
	u16 gyro_trim;
	u16 accel_trim;
};

/* registers */
#define REG_XA_OFFS_L_TC		(0x07)
#define REG_PRODUCT_ID			(0x0C)
#define REG_ST_GCT_X			(0x0D)
#define REG_BANK_SEL			(0x6D)
#define REG_MEM_START_ADDR		(0x6E)
#define REG_MEM_RW			(0x6F)
#define REG_PRGM_STRT_ADDRH		(0x70)
#define REG_PRGM_STRT_ADDRL		(0x71)
#define REG_FIFO_COUNT_H		(0x72)
#define REG_FIFO_COUNT_L		(0x73)
#define REG_FIFO_R_W			(0x74)
#define REG_WHO_AM_I			(0x75)

/* data definitions */
#define DMP_START_ADDR           0x400
#define DMP_MASK_TAP             0x3f
#define DMP_MASK_DIS_ORIEN       0xC0
#define DMP_DIS_ORIEN_SHIFT      6

#define BYTES_FOR_DMP            8
#define BYTES_FOR_EVENTS         4
#define QUATERNION_BYTES         16
#define BYTES_PER_SENSOR         6
#define MPU3050_FOOTER_SIZE      2
#define FIFO_COUNT_BYTE          2
#define FIFO_SIZE                800
#define HARDWARE_FIFO_SIZE       1024
#define MAX_READ_SIZE            64
#define SENSOR_UP_TIME           30
#define INV_MPU_SAMPLE_RATE_CHANGE_STABLE 50
#define MPU_MEM_BANK_SIZE        256

/* data header defines */
#define PRESSURE_HDR             0x8000
#define ACCEL_HDR                0x4000
#define GYRO_HDR                 0x2000
#define COMPASS_HDR              0x1000
#define LPQUAT_HDR               0x0800
#define SIXQUAT_HDR              0x0400
#define PEDQUAT_HDR              0x0200
#define STEP_DETECTOR_HDR        0x0100
#define STEP_INDICATOR_MASK      0xf

#define MAX_BYTES_PER_SAMPLE     80
#define MAX_HW_FIFO_BYTES        (BYTES_PER_SENSOR * 2)
#define IIO_BUFFER_BYTES         8
#define HEADERED_NORMAL_BYTES    8
#define HEADERED_Q_BYTES         16

#define MPU6XXX_MAX_MOTION_THRESH (255*4)
#define MPU6050_MOTION_THRESH_SHIFT 5
#define MPU6500_MOTION_THRESH_SHIFT 2
#define MPU6050_MOTION_DUR_DEFAULT  1
#define MPU6050_MAX_MOTION_DUR   255
#define MPU_TEMP_SHIFT           16
#define LPA_FREQ_SHIFT           6
#define COMPASS_RATE_SCALE       10
#define MAX_GYRO_FS_PARAM        3
#define MAX_ACCEL_FS_PARAM        3
#define MAX_LPA_FREQ_PARAM       3
#define MPU_MAX_A_OFFSET_VALUE     16383
#define MPU_MIN_A_OFFSET_VALUE     -16384
#define MPU_MAX_G_OFFSET_VALUE     32767
#define MPU_MIN_G_OFFSET_VALUE     -32767
#define MPU6XXX_MAX_MPU_MEM      (256 * 12)

#define INIT_MOT_DUR             128
#define INIT_MOT_THR             128
#define INIT_ZMOT_DUR            128
#define INIT_ZMOT_THR            128
#define INIT_ST_SAMPLES          200
#define INIT_ST_MPU6050_SAMPLES  600
#define INIT_ST_THRESHOLD        50
#define INIT_PED_INT_THRESH      2
#define INIT_PED_THRESH          7
#define ST_THRESHOLD_MULTIPLIER  10
#define ST_MAX_SAMPLES           500
#define ST_MAX_THRESHOLD         100
#define DMP_INTERVAL_INIT       (5 * NSEC_PER_MSEC)
#define DMP_INTERVAL_MIN_ADJ    (50 * NSEC_PER_USEC)

/*---- MPU6500 ----*/
#define MPU6500_PRODUCT_REVISION 1
#define MPU6500_MEM_REV_ADDR     0x16
#define INV_MPU_REV_MASK         0x0F
#define MPU6500_REV              2
#define MPU_DMP_LOAD_START       0x20

#define GYRO_CONFIG_FSR_SHIFT    3
#define ACCEL_CONFIG_FSR_SHIFT    3
#define GYRO_DPS_SCALE           250
#define MEM_ADDR_PROD_REV        0x6
#define SOFT_PROD_VER_BYTES      5
#define CRC_FIRMWARE_SEED        0
#define SELF_TEST_SUCCESS        1
#define MS_PER_DMP_TICK          20
#define DMP_IMAGE_SIZE           2463

/* init parameters */
#define INIT_FIFO_RATE           50
#define INIT_DMP_OUTPUT_RATE     25
#define INIT_DUR_TIME           (NSEC_PER_SEC / INIT_FIFO_RATE)
#define INIT_TAP_THRESHOLD       100
#define INIT_TAP_TIME            100
#define INIT_TAP_MIN_COUNT       2
#define INIT_SAMPLE_DIVIDER      4
#define MPU_INIT_SMD_DELAY_THLD  3
#define MPU_INIT_SMD_DELAY2_THLD 1
#define MPU_INIT_SMD_THLD        1500
#define MPU_DEFAULT_DMP_FREQ     200
#define MPL_PROD_KEY(ver, rev)  (ver * 100 + rev)
#define NUM_OF_PROD_REVS (ARRAY_SIZE(prod_rev_map))
/*---- MPU6050 Silicon Revisions ----*/
#define MPU_SILICON_REV_A2                    1       /* MPU6050A2 Device */
#define MPU_SILICON_REV_B1                    2       /* MPU6050B1 Device */

#define BIT_PRFTCH_EN                         0x40
#define BIT_CFG_USER_BANK                     0x20
#define BITS_MEM_SEL                          0x1f

#define TIME_STAMP_TOR                        5
#define MAX_CATCH_UP                          5
#define DEFAULT_ACCEL_TRIM                    16384
#define DEFAULT_GYRO_TRIM                     131
#define MAX_FIFO_RATE                         1000
#define MAX_DMP_OUTPUT_RATE                   200
#define MIN_FIFO_RATE                         4
#define ONE_K_HZ                              1000
#define NS_PER_MS_SHIFT                       20
#define END_MARKER                            0x0010
#define EMPTY_MARKER                          0x0020

#define INT_SRC_TAP             0x01
#define INT_SRC_DISPLAY_ORIENT  0x08
#define INT_SRC_SHAKE           0x10

#define INV_X_AXIS_INDEX                  0x00
#define INV_Y_AXIS_INDEX                  0x01
#define INV_Z_AXIS_INDEX                  0x02

#define INV_ELEMENT_1                     0x0001
#define INV_ELEMENT_2                     0x0002
#define INV_ELEMENT_3                     0x0004
#define INV_ELEMENT_4                     0x0008
#define INV_ELEMENT_5                     0x0010
#define INV_ELEMENT_6                     0x0020
#define INV_ELEMENT_7                     0x0040
#define INV_ELEMENT_8                     0x0080
#define INV_ALL                           0xFFFF
#define INV_ELEMENT_MASK                  0x00FF
#define INV_GYRO_ACC_MASK                 0x007E
#define INV_ACCEL_MASK                    0x70
#define INV_GYRO_MASK                     0xE


/* DMP defines */
#define DMP_ORIENTATION_TIME            500
#define DMP_ORIENTATION_ANGLE           60
#define DMP_DEFAULT_FIFO_RATE           200
#define DMP_TAP_SCALE                   (767603923 / 5)
#define DMP_MULTI_SHIFT                 30
#define DMP_MULTI_TAP_TIME              500
#define DMP_SHAKE_REJECT_THRESH         100
#define DMP_SHAKE_REJECT_TIME           10
#define DMP_SHAKE_REJECT_TIMEOUT        10
#define DMP_ANGLE_SCALE                 15
#define DMP_PRECISION                   1000
#define DMP_MAX_DIVIDER                 4
#define DMP_MAX_MIN_TAPS                4
#define DMP_IMAGE_CRC_VALUE             0x972aae92

/*--- Test parameters defaults --- */
#define DEF_OLDEST_SUPP_PROD_REV        8
#define DEF_OLDEST_SUPP_SW_REV          2

/* sample rate */
#define DEF_SELFTEST_SAMPLE_RATE        0
/* full scale setting dps */
#define DEF_SELFTEST_GYRO_FS            (0 << 3)
#define DEF_SELFTEST_ACCEL_FS           (2 << 3)
#define DEF_SELFTEST_GYRO_SENS          (32768 / 250)
/* wait time before collecting data */
#define DEF_GYRO_WAIT_TIME              10
#define DEF_ST_STABLE_TIME              20
#define DEF_ST_6500_STABLE_TIME         20
#define DEF_GYRO_SCALE                  131
#define DEF_ST_PRECISION                1000
#define DEF_ST_ACCEL_FS_MG              8000UL
#define DEF_ST_SCALE                    (1L << 15)
#define DEF_ST_TRY_TIMES                2
#define DEF_ST_ACCEL_RESULT_SHIFT       1
#define DEF_ST_OTP0_THRESH              60
#define DEF_ST_ABS_THRESH               20
#define DEF_ST_TOR                      2

#define X                               0
#define Y                               1
#define Z                               2
/*---- MPU6050 notable product revisions ----*/
#define MPU_PRODUCT_KEY_B1_E1_5         105
#define MPU_PRODUCT_KEY_B2_F1           431
/* accelerometer Hw self test min and max bias shift (mg) */
#define DEF_ACCEL_ST_SHIFT_MIN          300
#define DEF_ACCEL_ST_SHIFT_MAX          950

#define DEF_ACCEL_ST_SHIFT_DELTA        500
#define DEF_GYRO_CT_SHIFT_DELTA         500
/* gyroscope Coriolis self test min and max bias shift (dps) */
#define DEF_GYRO_CT_SHIFT_MIN           10
#define DEF_GYRO_CT_SHIFT_MAX           105

/*---- MPU6500 Self Test Pass/Fail Criteria ----*/
/* Gyro Offset Max Value (dps) */
#define DEF_GYRO_OFFSET_MAX             20
/* Gyro Self Test Absolute Limits ST_AL (dps) */
#define DEF_GYRO_ST_AL                  60
/* Accel Self Test Absolute Limits ST_AL (mg) */
#define DEF_ACCEL_ST_AL_MIN             225
#define DEF_ACCEL_ST_AL_MAX             675
#define DEF_6500_ACCEL_ST_SHIFT_DELTA   500
#define DEF_6500_GYRO_CT_SHIFT_DELTA    500
#define DEF_ST_MPU6500_ACCEL_LPF        2
#define DEF_ST_6500_ACCEL_FS_MG         2000UL
#define DEF_SELFTEST_6500_ACCEL_FS      (0 << 3)

/* Note: The ST_AL values are only used when ST_OTP = 0,
 * i.e no factory self test values for reference
 */

/* NOTE: product entries are in chronological order */
static const struct prod_rev_map_t prod_rev_map[] = {
	/* prod_ver = 0 */
	{MPL_PROD_KEY(0,   1), MPU_SILICON_REV_A2, 131, 16384},
	{MPL_PROD_KEY(0,   2), MPU_SILICON_REV_A2, 131, 16384},
	{MPL_PROD_KEY(0,   3), MPU_SILICON_REV_A2, 131, 16384},
	{MPL_PROD_KEY(0,   4), MPU_SILICON_REV_A2, 131, 16384},
	{MPL_PROD_KEY(0,   5), MPU_SILICON_REV_A2, 131, 16384},
	{MPL_PROD_KEY(0,   6), MPU_SILICON_REV_A2, 131, 16384},
	/* prod_ver = 1 */
	{MPL_PROD_KEY(0,   7), MPU_SILICON_REV_A2, 131, 16384},
	{MPL_PROD_KEY(0,   8), MPU_SILICON_REV_A2, 131, 16384},
	{MPL_PROD_KEY(0,   9), MPU_SILICON_REV_A2, 131, 16384},
	{MPL_PROD_KEY(0,  10), MPU_SILICON_REV_A2, 131, 16384},
	{MPL_PROD_KEY(0,  11), MPU_SILICON_REV_A2, 131, 16384},
	{MPL_PROD_KEY(0,  12), MPU_SILICON_REV_A2, 131, 16384},
	{MPL_PROD_KEY(0,  13), MPU_SILICON_REV_A2, 131, 16384},
	{MPL_PROD_KEY(0,  14), MPU_SILICON_REV_A2, 131, 16384},
	{MPL_PROD_KEY(0,  15), MPU_SILICON_REV_A2, 131, 16384},
	{MPL_PROD_KEY(0,  27), MPU_SILICON_REV_A2, 131, 16384},
	/* prod_ver = 1 */
	{MPL_PROD_KEY(1,  16), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(1,  17), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(1,  18), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(1,  19), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(1,  20), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(1,  28), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(1,   1), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(1,   2), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(1,   3), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(1,   4), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(1,   5), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(1,   6), MPU_SILICON_REV_B1, 131, 16384},
	/* prod_ver = 2 */
	{MPL_PROD_KEY(2,   7), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(2,   8), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(2,   9), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(2,  10), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(2,  11), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(2,  12), MPU_SILICON_REV_B1, 131, 16384},
	{MPL_PROD_KEY(2,  29), MPU_SILICON_REV_B1, 131, 16384},
	/* prod_ver = 3 */
	{MPL_PROD_KEY(3,  30), MPU_SILICON_REV_B1, 131, 16384},
	/* prod_ver = 4 */
	{MPL_PROD_KEY(4,  31), MPU_SILICON_REV_B1, 131,  8192},
	{MPL_PROD_KEY(4,   1), MPU_SILICON_REV_B1, 131,  8192},
	{MPL_PROD_KEY(4,   3), MPU_SILICON_REV_B1, 131,  8192},
	/* prod_ver = 5 */
	{MPL_PROD_KEY(5,   3), MPU_SILICON_REV_B1, 131, 16384},
	/* prod_ver = 6 */
	{MPL_PROD_KEY(6,  19), MPU_SILICON_REV_B1, 131, 16384},
	/* prod_ver = 7 */
	{MPL_PROD_KEY(7,  19), MPU_SILICON_REV_B1, 131, 16384},
	/* prod_ver = 8 */
	{MPL_PROD_KEY(8,  19), MPU_SILICON_REV_B1, 131, 16384},
	/* prod_ver = 9 */
	{MPL_PROD_KEY(9,  19), MPU_SILICON_REV_B1, 131, 16384},
	/* prod_ver = 10 */
	{MPL_PROD_KEY(10, 19), MPU_SILICON_REV_B1, 131, 16384}
};

/*
*   List of product software revisions
*
*   NOTE :
*   software revision 0 falls back to the old detection method
*   based off the product version and product revision per the
*   table above
*/
static const struct prod_rev_map_t sw_rev_map[] = {
	{0,		     0,   0,     0},
	{1, MPU_SILICON_REV_B1, 131,  8192},	/* rev C */
	{2, MPU_SILICON_REV_B1, 131, 16384}	/* rev D */
};

static const u16 mpu_6500_st_tb[256] = {
	2620, 2646, 2672, 2699, 2726, 2753, 2781, 2808,
	2837, 2865, 2894, 2923, 2952, 2981, 3011, 3041,
	3072, 3102, 3133, 3165, 3196, 3228, 3261, 3293,
	3326, 3359, 3393, 3427, 3461, 3496, 3531, 3566,
	3602, 3638, 3674, 3711, 3748, 3786, 3823, 3862,
	3900, 3939, 3979, 4019, 4059, 4099, 4140, 4182,
	4224, 4266, 4308, 4352, 4395, 4439, 4483, 4528,
	4574, 4619, 4665, 4712, 4759, 4807, 4855, 4903,
	4953, 5002, 5052, 5103, 5154, 5205, 5257, 5310,
	5363, 5417, 5471, 5525, 5581, 5636, 5693, 5750,
	5807, 5865, 5924, 5983, 6043, 6104, 6165, 6226,
	6289, 6351, 6415, 6479, 6544, 6609, 6675, 6742,
	6810, 6878, 6946, 7016, 7086, 7157, 7229, 7301,
	7374, 7448, 7522, 7597, 7673, 7750, 7828, 7906,
	7985, 8065, 8145, 8227, 8309, 8392, 8476, 8561,
	8647, 8733, 8820, 8909, 8998, 9088, 9178, 9270,
	9363, 9457, 9551, 9647, 9743, 9841, 9939, 10038,
	10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,
	10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,
	11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,
	12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,
	13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,
	15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,
	16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,
	17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,
	19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,
	20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,
	22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,
	24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,
	26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,
	28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,
	30903, 31212, 31524, 31839, 32157, 32479, 32804
};

static const int accel_st_tb[31] = {
	340, 351, 363, 375, 388, 401, 414, 428,
	443, 458, 473, 489, 506, 523, 541, 559,
	578, 597, 617, 638, 660, 682, 705, 729,
	753, 779, 805, 832, 860, 889, 919
};

static const int gyro_6050_st_tb[31] = {
	3275, 3425, 3583, 3748, 3920, 4100, 4289, 4486,
	4693, 4909, 5134, 5371, 5618, 5876, 6146, 6429,
	6725, 7034, 7358, 7696, 8050, 8421, 8808, 9213,
	9637, 10080, 10544, 11029, 11537, 12067, 12622
};

char *wr_pr_debug_begin(u8 const *data, u32 len, char *string)
{
	int ii;
	string = kmalloc(len * 2 + 1, GFP_KERNEL);
	for (ii = 0; ii < len; ii++)
		sprintf(&string[ii * 2], "%02X", data[ii]);
	string[len * 2] = 0;
	return string;
}

char *wr_pr_debug_end(char *string)
{
	kfree(string);
	return "";
}

int mpu_memory_write(struct nvi_state *st, u8 mpu_addr, u16 mem_addr,
		     u32 len, u8 const *data)
{
	u8 bank[2];
	u8 addr[2];
	u8 buf[513];

	struct i2c_msg msgs[3];
	int res;

	if (!data || !st)
		return -EINVAL;

	if (len >= (sizeof(buf) - 1))
		return -ENOMEM;

	bank[0] = REG_BANK_SEL;
	bank[1] = mem_addr >> 8;

	addr[0] = REG_MEM_START_ADDR;
	addr[1] = mem_addr & 0xFF;

	buf[0] = REG_MEM_RW;
	memcpy(buf + 1, data, len);

	/* write message */
	msgs[0].addr = mpu_addr;
	msgs[0].flags = 0;
	msgs[0].buf = bank;
	msgs[0].len = sizeof(bank);

	msgs[1].addr = mpu_addr;
	msgs[1].flags = 0;
	msgs[1].buf = addr;
	msgs[1].len = sizeof(addr);

	msgs[2].addr = mpu_addr;
	msgs[2].flags = 0;
	msgs[2].buf = (u8 *)buf;
	msgs[2].len = len + 1;

#if CONFIG_DYNAMIC_DEBUG
	{
		char *write = 0;
		pr_debug("%s WM%02X%02X%02X%s%s - %d\n", st->hal->part_name,
			 mpu_addr, bank[1], addr[1],
			 wr_pr_debug_begin(data, len, write),
			 wr_pr_debug_end(write),
			 len);
	}
#endif

	res = i2c_transfer(st->i2c->adapter, msgs, 3);
	if (res != 3) {
		if (res >= 0)
			res = -EIO;
		return res;
	} else {
		return 0;
	}
}

int mpu_memory_read(struct nvi_state *st, u8 mpu_addr, u16 mem_addr,
		    u32 len, u8 *data)
{
	u8 bank[2];
	u8 addr[2];
	u8 buf;

	struct i2c_msg msgs[4];
	int res;

	if (!data || !st)
		return -EINVAL;

	bank[0] = REG_BANK_SEL;
	bank[1] = mem_addr >> 8;

	addr[0] = REG_MEM_START_ADDR;
	addr[1] = mem_addr & 0xFF;

	buf = REG_MEM_RW;

	/* write message */
	msgs[0].addr = mpu_addr;
	msgs[0].flags = 0;
	msgs[0].buf = bank;
	msgs[0].len = sizeof(bank);

	msgs[1].addr = mpu_addr;
	msgs[1].flags = 0;
	msgs[1].buf = addr;
	msgs[1].len = sizeof(addr);

	msgs[2].addr = mpu_addr;
	msgs[2].flags = 0;
	msgs[2].buf = &buf;
	msgs[2].len = 1;

	msgs[3].addr = mpu_addr;
	msgs[3].flags = I2C_M_RD;
	msgs[3].buf = data;
	msgs[3].len = len;

	res = i2c_transfer(st->i2c->adapter, msgs, 4);
	if (res != 4) {
		if (res >= 0)
			res = -EIO;
	} else
		res = 0;

#if CONFIG_DYNAMIC_DEBUG
	{
		char *read = 0;
		pr_debug("%s RM%02X%02X%02X%02X - %s%s\n", st->hal->part_name,
			 mpu_addr, bank[1], addr[1], len,
			 wr_pr_debug_begin(data, len, read),
			 wr_pr_debug_end(read));
	}
#endif

	return res;
}

int mpu_memory_write_unaligned(struct nvi_state *st, u16 key, int len,
								u8 const *d)
{
	u32 addr;
	int start, end;
	int len1, len2;
	int result = 0;

	if (len > MPU_MEM_BANK_SIZE)
		return -EINVAL;
	addr = inv_dmp_get_address(key);
	if (addr > MPU6XXX_MAX_MPU_MEM)
		return -EINVAL;

	start = (addr >> 8);
	end   = ((addr + len - 1) >> 8);
	if (start == end) {
		result = mpu_memory_write(st, st->i2c_addr, addr, len, d);
	} else {
		end <<= 8;
		len1 = end - addr;
		len2 = len - len1;
		result = mpu_memory_write(st, st->i2c_addr, addr, len1, d);
		result |= mpu_memory_write(st, st->i2c_addr, end, len2,
								d + len1);
	}

	return result;
}

/**
 *  index_of_key()- Inverse lookup of the index of an MPL product key .
 *  @key: the MPL product indentifier also referred to as 'key'.
 */
static short index_of_key(u16 key)
{
	int i;
	for (i = 0; i < NUM_OF_PROD_REVS; i++)
		if (prod_rev_map[i].mpl_product_key == key)
			return (short)i;
	return -EINVAL;
}

int inv_get_silicon_rev_mpu6500(struct nvi_state *st)
{
	struct inv_chip_info_s *chip_info = &st->chip_info;
	int result;
	u8 whoami, sw_rev;

	result = nvi_i2c_rd(st, 0, REG_WHO_AM_I, 1, &whoami);
	if (result)
		return result;
	if (whoami != MPU6500_ID && whoami != MPU9250_ID &&
			whoami != MPU9350_ID && whoami != MPU6515_ID)
		return -EINVAL;

	/*memory read need more time after power up */
	msleep(POWER_UP_TIME);
	result = mpu_memory_read(st, st->i2c_addr,
			MPU6500_MEM_REV_ADDR, 1, &sw_rev);
	sw_rev &= INV_MPU_REV_MASK;
	if (result)
		return result;
	if (sw_rev != 0)
		return -EINVAL;
	/* these values are place holders and not real values */
	chip_info->product_id = MPU6500_PRODUCT_REVISION;
	chip_info->product_revision = MPU6500_PRODUCT_REVISION;
	chip_info->silicon_revision = MPU6500_PRODUCT_REVISION;
	chip_info->software_revision = sw_rev;
	chip_info->gyro_sens_trim = DEFAULT_GYRO_TRIM;
	chip_info->accel_sens_trim = DEFAULT_ACCEL_TRIM;
	chip_info->multi = 1;

	return 0;
}

int inv_get_silicon_rev_mpu6050(struct nvi_state *st)
{
	int result;
	u8 prod_ver = 0x00, prod_rev = 0x00;
	struct prod_rev_map_t *p_rev;
	u8 bank =
	    (BIT_PRFTCH_EN | BIT_CFG_USER_BANK | MPU_MEM_OTP_BANK_0);
	u16 mem_addr = ((bank << 8) | MEM_ADDR_PROD_REV);
	u16 key;
	u8 regs[5];
	u16 sw_rev;
	short index;
	struct inv_chip_info_s *chip_info = &st->chip_info;

	result = nvi_i2c_rd(st, 0, REG_PRODUCT_ID, 1, &prod_ver);
	if (result)
		return result;
	prod_ver &= 0xf;
	/*memory read need more time after power up */
	msleep(POWER_UP_TIME);
	result = mpu_memory_read(st, st->i2c_addr, mem_addr, 1, &prod_rev);
	if (result)
		return result;
	prod_rev >>= 2;
	/* clean the prefetch and cfg user bank bits */
	result = nvi_i2c_wr(st, REG_BANK_SEL, 0);
	if (result)
		return result;
	/* get the software-product version, read from XA_OFFS_L */
	result = nvi_i2c_rd(st, 0 , REG_XA_OFFS_L_TC,
				SOFT_PROD_VER_BYTES, regs);
	if (result)
		return result;

	sw_rev = (regs[4] & 0x01) << 2 |	/* 0x0b, bit 0 */
		 (regs[2] & 0x01) << 1 |	/* 0x09, bit 0 */
		 (regs[0] & 0x01);		/* 0x07, bit 0 */
	/* if 0, use the product key to determine the type of part */
	if (sw_rev == 0) {
		key = MPL_PROD_KEY(prod_ver, prod_rev);
		if (key == 0)
			return -EINVAL;
		index = index_of_key(key);
		if (index < 0 || index >= NUM_OF_PROD_REVS)
			return -EINVAL;
		/* check MPL is compiled for this device */
		if (prod_rev_map[index].silicon_rev != MPU_SILICON_REV_B1)
			return -EINVAL;
		p_rev = (struct prod_rev_map_t *)&prod_rev_map[index];
	/* if valid, use the software product key */
	} else if (sw_rev < ARRAY_SIZE(sw_rev_map)) {
		p_rev = (struct prod_rev_map_t *)&sw_rev_map[sw_rev];
	} else {
		return -EINVAL;
	}
	chip_info->product_id = prod_ver;
	chip_info->product_revision = prod_rev;
	chip_info->silicon_revision = p_rev->silicon_rev;
	chip_info->software_revision = sw_rev;
	chip_info->gyro_sens_trim = p_rev->gyro_trim;
	chip_info->accel_sens_trim = p_rev->accel_trim;
	if (chip_info->accel_sens_trim == 0)
		chip_info->accel_sens_trim = DEFAULT_ACCEL_TRIM;
	chip_info->multi = DEFAULT_ACCEL_TRIM / chip_info->accel_sens_trim;
	if (chip_info->multi != 1)
		pr_info("multi is %d\n", chip_info->multi);
	return result;
}

static int inv_reset_offset_reg(struct nvi_state *st, bool en)
{
	int i, result;
	s16 gyro[3], accel[3];

	if (en) {
		for (i = 0; i < 3; i++) {
			gyro[i] = st->rom_gyro_offset[i];
			accel[i] = st->rom_accel_offset[i];
		}
	} else {
		for (i = 0; i < 3; i++) {
			gyro[i] = st->rom_gyro_offset[i] +
						st->input_gyro_offset[i];
			accel[i] = st->rom_accel_offset[i] +
					(st->input_accel_offset[i] << 1);
		}
	}
	for (i = 0; i < 3; i++) {
		result = nvi_wr_gyro_offset(st, i, (u16)gyro[i]);
		if (result)
			return result;
		result = nvi_wr_accel_offset(st, i, (u16)accel[i]);
		if (result)
			return result;
	}

	return 0;
}

/**
 *  inv_mpu_recover_setting() recover the old settings after everything is done
 */
static void inv_mpu_recover_setting(struct nvi_state *st)
{
	inv_reset_offset_reg(st, false);
}

/**
 *  read_accel_hw_self_test_prod_shift()- read the accelerometer hardware
 *                                         self-test bias shift calculated
 *                                         during final production test and
 *                                         stored in chip non-volatile memory.
 *  @st:  main data structure.
 *  @st_prod:   A pointer to an array of 3 elements to hold the values
 *              for production hardware self-test bias shifts returned to the
 *              user.
 *  @accel_sens: accel sensitivity.
 */
static int read_accel_hw_self_test_prod_shift(struct nvi_state *st,
					int *st_prod, int *accel_sens)
{
	u8 regs[4];
	u8 shift_code[3];
	int result, i;

	for (i = 0; i < 3; i++)
		st_prod[i] = 0;

	result = nvi_i2c_rd(st, 0, REG_ST_GCT_X, ARRAY_SIZE(regs), regs);
	if (result)
		return result;
	if ((0 == regs[0])  && (0 == regs[1]) &&
	    (0 == regs[2]) && (0 == regs[3]))
		return -EINVAL;
	shift_code[X] = ((regs[0] & 0xE0) >> 3) | ((regs[3] & 0x30) >> 4);
	shift_code[Y] = ((regs[1] & 0xE0) >> 3) | ((regs[3] & 0x0C) >> 2);
	shift_code[Z] = ((regs[2] & 0xE0) >> 3) |  (regs[3] & 0x03);
	for (i = 0; i < 3; i++)
		if (shift_code[i] != 0)
			st_prod[i] = accel_sens[i] *
					accel_st_tb[shift_code[i] - 1];

	return 0;
}

/**
* inv_check_accel_self_test()- check accel self test. this function returns
*                              zero as success. A non-zero return value
*                              indicates failure in self test.
*  @*st: main data structure.
*  @*reg_avg: average value of normal test.
*  @*st_avg:  average value of self test
*/
static int inv_check_accel_self_test(struct nvi_state *st,
						int *reg_avg, int *st_avg){
	int gravity, j, ret_val;
	int tmp;
	int st_shift_prod[AXIS_N], st_shift_cust[AXIS_N];
	int st_shift_ratio[AXIS_N];
	int accel_sens[AXIS_N];

	if (st->chip_info.software_revision < DEF_OLDEST_SUPP_SW_REV &&
	    st->chip_info.product_revision < DEF_OLDEST_SUPP_PROD_REV)
		return 0;
	ret_val = 0;
	tmp = DEF_ST_SCALE * DEF_ST_PRECISION / DEF_ST_ACCEL_FS_MG;
	for (j = 0; j < 3; j++)
		accel_sens[j] = tmp;

	if (MPL_PROD_KEY(st->chip_info.product_id,
			 st->chip_info.product_revision) ==
	    MPU_PRODUCT_KEY_B1_E1_5) {
		/* half sensitivity Z accelerometer parts */
		accel_sens[Z] /= 2;
	} else {
		/* half sensitivity X, Y, Z accelerometer parts */
		accel_sens[X] /= st->chip_info.multi;
		accel_sens[Y] /= st->chip_info.multi;
		accel_sens[Z] /= st->chip_info.multi;
	}
	gravity = accel_sens[Z];
	ret_val = read_accel_hw_self_test_prod_shift(st, st_shift_prod,
							accel_sens);
	if (ret_val)
		return ret_val;

	for (j = 0; j < 3; j++) {
		st_shift_cust[j] = abs(reg_avg[j] - st_avg[j]);
		if (st_shift_prod[j]) {
			tmp = st_shift_prod[j] / DEF_ST_PRECISION;
			st_shift_ratio[j] = abs(st_shift_cust[j] / tmp
				- DEF_ST_PRECISION);
			if (st_shift_ratio[j] > DEF_ACCEL_ST_SHIFT_DELTA)
				ret_val = 1;
		} else {
			if (st_shift_cust[j] <
				DEF_ACCEL_ST_SHIFT_MIN * gravity)
				ret_val = 1;
			if (st_shift_cust[j] >
				DEF_ACCEL_ST_SHIFT_MAX * gravity)
				ret_val = 1;
		}
	}

	return ret_val;
}

/**
* inv_check_6050_gyro_self_test() - check 6050 gyro self test. this function
*                                   returns zero as success. A non-zero return
*                                   value indicates failure in self test.
*  @*st: main data structure.
*  @*reg_avg: average value of normal test.
*  @*st_avg:  average value of self test
*/
static int inv_check_6050_gyro_self_test(struct nvi_state *st,
						int *reg_avg, int *st_avg){
	int result;
	int ret_val;
	int st_shift_prod[3], st_shift_cust[3], st_shift_ratio[3], i;
	u8 regs[3];

	if (st->chip_info.software_revision < DEF_OLDEST_SUPP_SW_REV &&
	    st->chip_info.product_revision < DEF_OLDEST_SUPP_PROD_REV)
		return 0;

	ret_val = 0;
	result = nvi_i2c_rd(st, 0, REG_ST_GCT_X, 3, regs);
	if (result)
		return result;
	regs[X] &= 0x1f;
	regs[Y] &= 0x1f;
	regs[Z] &= 0x1f;
	for (i = 0; i < 3; i++) {
		if (regs[i] != 0)
			st_shift_prod[i] = gyro_6050_st_tb[regs[i] - 1];
		else
			st_shift_prod[i] = 0;
	}
	st_shift_prod[1] = -st_shift_prod[1];

	for (i = 0; i < 3; i++) {
		st_shift_cust[i] =  st_avg[i] - reg_avg[i];
		if (st_shift_prod[i]) {
			st_shift_ratio[i] = abs(st_shift_cust[i] /
				st_shift_prod[i] - DEF_ST_PRECISION);
			if (st_shift_ratio[i] > DEF_GYRO_CT_SHIFT_DELTA)
				ret_val = 1;
		} else {
			if (st_shift_cust[i] < DEF_ST_PRECISION *
				DEF_GYRO_CT_SHIFT_MIN * DEF_SELFTEST_GYRO_SENS)
				ret_val = 1;
			if (st_shift_cust[i] > DEF_ST_PRECISION *
				DEF_GYRO_CT_SHIFT_MAX * DEF_SELFTEST_GYRO_SENS)
				ret_val = 1;
		}
	}
	/* check for absolute value passing criterion. Using DEF_ST_TOR
	 * for certain degree of tolerance */
	for (i = 0; i < 3; i++)
		if (abs(reg_avg[i]) > DEF_ST_TOR * DEF_ST_ABS_THRESH *
		    DEF_ST_PRECISION * DEF_GYRO_SCALE)
			ret_val = 1;

	return ret_val;
}

/**
* inv_check_6500_gyro_self_test() - check 6500 gyro self test. this function
*                                   returns zero as success. A non-zero return
*                                   value indicates failure in self test.
*  @*st: main data structure.
*  @*reg_avg: average value of normal test.
*  @*st_avg:  average value of self test
*/
static int inv_check_6500_gyro_self_test(struct nvi_state *st,
						int *reg_avg, int *st_avg) {
	u8 regs[3];
	int ret_val, result;
	int otp_value_zero = 0;
	int st_shift_prod[3], st_shift_cust[3], i;

	ret_val = 0;
	result = nvi_i2c_rd(st, st->hal->reg->self_test_x_gyro.bank,
			    st->hal->reg->self_test_x_gyro.reg, 3, regs);
	if (result)
		return result;
	pr_debug("%s self_test gyro shift_code - %02x %02x %02x\n",
		 st->hal->part_name, regs[0], regs[1], regs[2]);

	for (i = 0; i < 3; i++) {
		if (regs[i] != 0) {
			st_shift_prod[i] = mpu_6500_st_tb[regs[i] - 1];
		} else {
			st_shift_prod[i] = 0;
			otp_value_zero = 1;
		}
	}
	pr_debug("%s self_test gyro st_shift_prod - %+d %+d %+d\n",
		 st->hal->part_name, st_shift_prod[0], st_shift_prod[1],
		 st_shift_prod[2]);

	for (i = 0; i < 3; i++) {
		st_shift_cust[i] = st_avg[i] - reg_avg[i];
		if (!otp_value_zero) {
			/* Self Test Pass/Fail Criteria A */
			if (st_shift_cust[i] < DEF_6500_GYRO_CT_SHIFT_DELTA
						* st_shift_prod[i])
					ret_val = 1;
		} else {
			/* Self Test Pass/Fail Criteria B */
			if (st_shift_cust[i] < DEF_GYRO_ST_AL *
						DEF_SELFTEST_GYRO_SENS *
						DEF_ST_PRECISION)
				ret_val = 1;
		}
	}
	pr_debug("%s self_test gyro st_shift_cust - %+d %+d %+d\n",
		 st->hal->part_name, st_shift_cust[0], st_shift_cust[1],
		 st_shift_cust[2]);

	if (ret_val == 0) {
		/* Self Test Pass/Fail Criteria C */
		for (i = 0; i < 3; i++)
			if (abs(reg_avg[i]) > DEF_GYRO_OFFSET_MAX *
						DEF_SELFTEST_GYRO_SENS *
						DEF_ST_PRECISION)
				ret_val = 1;
	}

	return ret_val;
}

/**
* inv_check_6500_accel_self_test() - check 6500 accel self test. this function
*                                   returns zero as success. A non-zero return
*                                   value indicates failure in self test.
*  @*st: main data structure.
*  @*reg_avg: average value of normal test.
*  @*st_avg:  average value of self test
*/
static int inv_check_6500_accel_self_test(struct nvi_state *st,
						int *reg_avg, int *st_avg) {
	int ret_val, result;
	int st_shift_prod[3], st_shift_cust[3], st_shift_ratio[3], i;
	u8 regs[3];
	int otp_value_zero = 0;

#define ACCEL_ST_AL_MIN ((DEF_ACCEL_ST_AL_MIN * DEF_ST_SCALE \
				 / DEF_ST_6500_ACCEL_FS_MG) * DEF_ST_PRECISION)
#define ACCEL_ST_AL_MAX ((DEF_ACCEL_ST_AL_MAX * DEF_ST_SCALE \
				 / DEF_ST_6500_ACCEL_FS_MG) * DEF_ST_PRECISION)

	ret_val = 0;
	result = nvi_i2c_rd(st, st->hal->reg->self_test_x_accel.bank,
			    st->hal->reg->self_test_x_accel.reg, 3, regs);
	if (result)
		return result;
	pr_debug("%s self_test accel shift_code - %02x %02x %02x\n",
		 st->hal->part_name, regs[0], regs[1], regs[2]);

	for (i = 0; i < 3; i++) {
		if (regs[i] != 0) {
			st_shift_prod[i] = mpu_6500_st_tb[regs[i] - 1];
		} else {
			st_shift_prod[i] = 0;
			otp_value_zero = 1;
		}
	}
	pr_debug("%s self_test accel st_shift_prod - %+d %+d %+d\n",
		 st->hal->part_name, st_shift_prod[0], st_shift_prod[1],
		 st_shift_prod[2]);

	if (!otp_value_zero) {
		/* Self Test Pass/Fail Criteria A */
		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = st_avg[i] - reg_avg[i];
			st_shift_ratio[i] = abs(st_shift_cust[i] /
					st_shift_prod[i] - DEF_ST_PRECISION);
			if (st_shift_ratio[i] > DEF_6500_ACCEL_ST_SHIFT_DELTA)
				ret_val = 1;
		}
	} else {
		/* Self Test Pass/Fail Criteria B */
		for (i = 0; i < 3; i++) {
			st_shift_cust[i] = abs(st_avg[i] - reg_avg[i]);
			if (st_shift_cust[i] < ACCEL_ST_AL_MIN ||
					st_shift_cust[i] > ACCEL_ST_AL_MAX)
				ret_val = 1;
		}
	}
	pr_debug("%s self_test accel st_shift_cust - %+d %+d %+d\n",
		 st->hal->part_name, st_shift_cust[0], st_shift_cust[1],
		 st_shift_cust[2]);

	return ret_val;
}

/*
 *  inv_mpu_do_test() - do the actual test of self testing
 */
static int inv_mpu_do_test(struct nvi_state *st, int self_test_flag,
		int *gyro_result, int *accel_result)
{
	int result, i, j, packet_size;
	u8 data[BYTES_PER_SENSOR * 2], d, lpf;
	u16 fifo_en;
	int fifo_count, packet_count, ind, s;

	packet_size = BYTES_PER_SENSOR * 2;
	result = nvi_int_able(st, false);
	if (result)
		return result;
	/* disable the sensor output to FIFO */
	/* disable fifo reading */
	result = nvi_user_ctrl_en(st, false, false);
	if (result)
		return result;
	/* clear FIFO */
	result = nvi_wr_user_ctrl(st, BIT_FIFO_RST);
	if (result)
		return result;
	/* setup parameters */
	result = nvi_wr_gyro_config(st, (self_test_flag >> 5), 0,
				    DEF_SELFTEST_GYRO_FS, INV_FILTER_98HZ);
	if (result < 0)
		return result;

	result = nvi_wr_smplrt_div(st, DEV_ANGLVEL, DEF_SELFTEST_SAMPLE_RATE);
	if (result)
		return result;
	/* wait for the sampling rate change to stabilize */
	mdelay(INV_MPU_SAMPLE_RATE_CHANGE_STABLE);
	if (st->hal->part >= MPU6500) {
		d = DEF_SELFTEST_6500_ACCEL_FS;
		lpf = DEF_ST_MPU6500_ACCEL_LPF;
	} else {
		d = DEF_SELFTEST_ACCEL_FS;
		lpf = 0;
	}
	result = nvi_wr_accel_config(st, (self_test_flag >> 5), 0,
				     (d >> 3), lpf);
	if (result < 0)
		return result;

	/* wait for the output to get stable */
	if (self_test_flag) {
		if (st->hal->part >= MPU6500)
			msleep(DEF_ST_6500_STABLE_TIME);
		else
			msleep(DEF_ST_STABLE_TIME);
	}

	/* enable FIFO reading */
	result = nvi_wr_user_ctrl(st, BIT_FIFO_EN);
	if (result)
		return result;
	/* enable sensor output to FIFO */
	fifo_en = ((1 << st->hal->bit->accel_fifo_en) |
		   (1 << st->hal->bit->gyro_z_fifo_en) |
		   (1 << st->hal->bit->gyro_y_fifo_en) |
		   (1 << st->hal->bit->gyro_x_fifo_en));
	for (i = 0; i < AXIS_N; i++) {
		gyro_result[i] = 0;
		accel_result[i] = 0;
	}
	s = 0;
	while (s < st->self_test.samples) {
		/* enable sensor output to FIFO */
		result = nvi_wr_fifo_en(st, fifo_en);
		if (result)
			return result;
		mdelay(DEF_GYRO_WAIT_TIME);
		result = nvi_wr_fifo_en(st, 0);
		if (result)
			return result;

		result = nvi_i2c_rd(st, st->hal->reg->fifo_count_h.bank,
				    st->hal->reg->fifo_count_h.reg,
				    FIFO_COUNT_BYTE, data);
		if (result)
			return result;
		fifo_count = be16_to_cpup((__be16 *)(&data[0]));
		pr_debug("%s self_test fifo_count - %d\n",
			 st->hal->part_name, fifo_count);
		packet_count = fifo_count / packet_size;
		i = 0;
		while ((i < packet_count) && (s < st->self_test.samples)) {
			short vals[3];
			result = nvi_i2c_rd(st, st->hal->reg->fifo_r_w.bank,
					    st->hal->reg->fifo_r_w.reg,
					    packet_size, data);
			if (result)
				return result;
			ind = 0;
			for (j = 0; j < AXIS_N; j++) {
				vals[j] = (short)be16_to_cpup(
				    (__be16 *)(&data[ind + 2 * j]));
				accel_result[j] += vals[j];
			}
			ind += BYTES_PER_SENSOR;
			pr_debug(
			    "%s self_test accel data - %d %+d %+d %+d",
				 st->hal->part_name,
				 s, vals[0], vals[1], vals[2]);

			for (j = 0; j < AXIS_N; j++) {
				vals[j] = (short)be16_to_cpup(
					(__be16 *)(&data[ind + 2 * j]));
				gyro_result[j] += vals[j];
			}
			pr_debug("%s self_test gyro data - %d %+d %+d %+d",
				 st->hal->part_name,
				 s, vals[0], vals[1], vals[2]);

			s++;
			i++;
		}
	}

	for (j = 0; j < AXIS_N; j++) {
		accel_result[j] = accel_result[j] / s;
		accel_result[j] *= DEF_ST_PRECISION;
	}
	for (j = 0; j < AXIS_N; j++) {
		gyro_result[j] = gyro_result[j] / s;
		gyro_result[j] *= DEF_ST_PRECISION;
	}

	return 0;
}

/*
 *  inv_mpu_self_test() - main function to do hardware self test
 */
static int inv_mpu_self_test(struct nvi_state *st)
{
	int result;
	int gyro_bias_st[AXIS_N];
	int gyro_bias_regular[AXIS_N];
	int accel_bias_st[AXIS_N], accel_bias_regular[AXIS_N];
	int test_times;
	int i;
	char accel_result;
	char gyro_result;

	result = nvi_pm_wr(st, INV_CLK_PLL, 0, 0);
	if (result)
		return result;
	result = inv_reset_offset_reg(st, true);
	if (result)
		return result;
	accel_result = 0;
	gyro_result = 0;
	test_times = DEF_ST_TRY_TIMES;
	while (test_times > 0) {
		result = inv_mpu_do_test(st, 0, gyro_bias_regular,
					 accel_bias_regular);
		if (result == -EAGAIN)
			test_times--;
		else
			test_times = 0;
	}
	if (result)
		goto test_fail;
	pr_debug("%s self_test accel bias_regular - %+d %+d %+d\n",
		 st->hal->part_name, accel_bias_regular[0],
		 accel_bias_regular[1], accel_bias_regular[2]);
	pr_debug("%s self_test gyro bias_regular - %+d %+d %+d\n",
		 st->hal->part_name, gyro_bias_regular[0],
		 gyro_bias_regular[1], gyro_bias_regular[2]);

	for (i = 0; i < 3; i++) {
		st->gyro_bias[i] = gyro_bias_regular[i];
		st->accel_bias[i] = accel_bias_regular[i];
	}

	test_times = DEF_ST_TRY_TIMES;
	while (test_times > 0) {
		result = inv_mpu_do_test(st, BITS_SELF_TEST_EN, gyro_bias_st,
					 accel_bias_st);
		if (result == -EAGAIN)
			test_times--;
		else
			break;
	}
	if (result)
		goto test_fail;
	pr_debug("%s self_test accel bias_st - %+d %+d %+d\n",
		 st->hal->part_name, accel_bias_st[0], accel_bias_st[1],
		 accel_bias_st[2]);
	pr_debug("%s self_test gyro bias_st - %+d %+d %+d\n",
		 st->hal->part_name, gyro_bias_st[0], gyro_bias_st[1],
		 gyro_bias_st[2]);

	 if (MPU6050 == st->hal->part) {
		accel_result = !inv_check_accel_self_test(st,
			accel_bias_regular, accel_bias_st);
		gyro_result = !inv_check_6050_gyro_self_test(st,
			gyro_bias_regular, gyro_bias_st);
	} else if (st->hal->part >= MPU6500) {
		accel_result = !inv_check_6500_accel_self_test(st,
			accel_bias_regular, accel_bias_st);
		gyro_result = !inv_check_6500_gyro_self_test(st,
			gyro_bias_regular, gyro_bias_st);
	}

test_fail:
	inv_mpu_recover_setting(st);
	return (accel_result << DEF_ST_ACCEL_RESULT_SHIFT) | gyro_result;
}

#define ICM_DEV_NAME_ACCEL		"accel"
#define ICM_DEV_NAME_GYRO		"gyro"
/* full scale and LPF setting */
#define ICM_SELFTEST_GYRO_FS		0
#define ICM_SELFTEST_ACCEL_FS		7
/* register settings */
#define ICM_SELFTEST_GYRO_SMPLRT_DIV	10
#define ICM_SELFTEST_GYRO_AVGCFG	3
#define ICM_SELFTEST_ACCEL_SMPLRT_DIV	10
#define ICM_SELFTEST_ACCEL_DEC3_CFG	2
#define ICM_SELFTEST_GYRO_SENS		(32768 / 250)
/* wait time before collecting data */
#define ICM_MAX_PACKETS			20
#define ICM_SELFTEST_WAIT_TIME		(ICM_MAX_PACKETS * 10)
#define ICM_GYRO_ENGINE_UP_TIME		50
#define ICM_ST_STABLE_TIME		20
#define ICM_GYRO_SCALE			131
#define ICM_ST_PRECISION		1000
#define ICM_ST_ACCEL_FS_MG		2000UL
#define ICM_ST_SCALE			32768
#define ICM_ST_TRY_TIMES		2
#define ICM_ST_SAMPLES			200
#define ICM_ACCEL_ST_SHIFT_DELTA_MIN	500
#define ICM_ACCEL_ST_SHIFT_DELTA_MAX	1500
#define ICM_GYRO_CT_SHIFT_DELTA		500
/* Gyro Offset Max Value (dps) */
#define ICM_GYRO_OFFSET_MAX		20
/* Gyro Self Test Absolute Limits ST_AL (dps) */
#define ICM_GYRO_ST_AL			60
/* Accel Self Test Absolute Limits ST_AL (mg) */
#define ICM_ACCEL_ST_AL_MIN ((225 * ICM_ST_SCALE \
			      / ICM_ST_ACCEL_FS_MG) * ICM_ST_PRECISION)
#define ICM_ACCEL_ST_AL_MAX ((675 * ICM_ST_SCALE \
			      / ICM_ST_ACCEL_FS_MG) * ICM_ST_PRECISION)


static const u16 icm_st_tb[256] = {
	2620, 2646, 2672, 2699, 2726, 2753, 2781, 2808,
	2837, 2865, 2894, 2923, 2952, 2981, 3011, 3041,
	3072, 3102, 3133, 3165, 3196, 3228, 3261, 3293,
	3326, 3359, 3393, 3427, 3461, 3496, 3531, 3566,
	3602, 3638, 3674, 3711, 3748, 3786, 3823, 3862,
	3900, 3939, 3979, 4019, 4059, 4099, 4140, 4182,
	4224, 4266, 4308, 4352, 4395, 4439, 4483, 4528,
	4574, 4619, 4665, 4712, 4759, 4807, 4855, 4903,
	4953, 5002, 5052, 5103, 5154, 5205, 5257, 5310,
	5363, 5417, 5471, 5525, 5581, 5636, 5693, 5750,
	5807, 5865, 5924, 5983, 6043, 6104, 6165, 6226,
	6289, 6351, 6415, 6479, 6544, 6609, 6675, 6742,
	6810, 6878, 6946, 7016, 7086, 7157, 7229, 7301,
	7374, 7448, 7522, 7597, 7673, 7750, 7828, 7906,
	7985, 8065, 8145, 8227, 8309, 8392, 8476, 8561,
	8647, 8733, 8820, 8909, 8998, 9088, 9178, 9270,
	9363, 9457, 9551, 9647, 9743, 9841, 9939, 10038,
	10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,
	10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,
	11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,
	12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,
	13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,
	15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,
	16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,
	17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,
	19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,
	20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,
	22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,
	24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,
	26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,
	28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,
	30903, 31212, 31524, 31839, 32157, 32479, 32804
};

/**
* inv_icm_check_gyro_self_test() - check gyro self test.
*  this function returns zero as success. A non-zero return
*  value indicates failure in self test.
*  @*st: main data structure.
*  @*reg_avg: average value of normal test.
*  @*st_avg: average value of self test
*/
static int inv_icm_check_gyro_self_test(struct nvi_state *st,
					int *reg_avg, int *st_avg)
{
	u8 *regs;
	int ret_val;
	int otp_value_zero = 0;
	int st_shift_prod[AXIS_N];
	int st_shift_cust[AXIS_N];
	int i;

	ret_val = 0;
	regs = st->st_data_gyro;
	if (st->dbg & NVI_DBG_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s data: %02x %02x %02x\n",
			 __func__, regs[0], regs[1], regs[2]);
	for (i = 0; i < AXIS_N; i++) {
		if (regs[i] != 0) {
			st_shift_prod[i] = icm_st_tb[regs[i] - 1];
		} else {
			st_shift_prod[i] = 0;
			otp_value_zero = 1;
		}
	}
	if (st->dbg & NVI_DBG_SPEW_MSG)
		dev_info(&st->i2c->dev,
			 "%s st_shift_prod: %+d %+d %+d\n", __func__,
			 st_shift_prod[0], st_shift_prod[1], st_shift_prod[2]);
	for (i = 0; i < AXIS_N; i++) {
		st_shift_cust[i] = st_avg[i] - reg_avg[i];
		if (!otp_value_zero) {
			/* Self Test Pass/Fail Criteria A */
			if (st_shift_cust[i] < (ICM_GYRO_CT_SHIFT_DELTA *
						st_shift_prod[i])) {
				ret_val = 1;
				if (st->dbg & NVI_DBG_SPEW_MSG)
					dev_info(&st->i2c->dev,
						 "%s FAIL A axis %d\n",
						 __func__, i);
			}
		} else {
			/* Self Test Pass/Fail Criteria B */
			if (st_shift_cust[i] < (ICM_GYRO_ST_AL *
						ICM_SELFTEST_GYRO_SENS *
						ICM_ST_PRECISION)) {
				ret_val = 1;
				if (st->dbg & NVI_DBG_SPEW_MSG)
					dev_info(&st->i2c->dev,
						 "%s FAIL B axis %d\n",
						 __func__, i);
			}
		}
	}
	if (st->dbg & NVI_DBG_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s st_shift_cust: %+d %+d %+d\n",
			 __func__,
			 st_shift_cust[0], st_shift_cust[1], st_shift_cust[2]);
	if (ret_val == 0) {
		/* Self Test Pass/Fail Criteria C */
		for (i = 0; i < AXIS_N; i++) {
			if (abs(reg_avg[i]) > (ICM_GYRO_OFFSET_MAX *
					       ICM_SELFTEST_GYRO_SENS *
					       ICM_ST_PRECISION)) {
				ret_val = 1;
				if (st->dbg & NVI_DBG_SPEW_MSG)
					dev_info(&st->i2c->dev,
						 "%s FAIL C axis %d\n",
						 __func__, i);
			}
		}
	}
	return ret_val;
}

/**
* inv_icm_check_accel_self_test() - check accel self test.
*  this function returns zero as success. A non-zero return
*  value indicates failure in self test.
*  @*st: main data structure.
*  @*reg_avg: average value of normal test.
*  @*st_avg: average value of self test
*/
static int inv_icm_check_accel_self_test(struct nvi_state *st,
					 int *reg_avg, int *st_avg)
{
	int ret_val;
	int st_shift_prod[AXIS_N];
	int st_shift_cust[AXIS_N];
	int i;
	u8 *regs;
	int otp_value_zero = 0;

	ret_val = 0;
	regs = st->st_data_accel;
	if (st->dbg & NVI_DBG_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s data: %02x %02x %02x\n",
			 __func__, regs[0], regs[1], regs[2]);
	for (i = 0; i < AXIS_N; i++) {
		if (regs[i] != 0) {
			st_shift_prod[i] = icm_st_tb[regs[i] - 1];
		} else {
			st_shift_prod[i] = 0;
			otp_value_zero = 1;
		}
	}
	if (st->dbg & NVI_DBG_SPEW_MSG)
		dev_info(&st->i2c->dev,
			 "%s st_shift_prod: %+d %+d %+d\n", __func__,
			 st_shift_prod[0], st_shift_prod[1], st_shift_prod[2]);
	if (!otp_value_zero) {
		/* Self Test Pass/Fail Criteria A */
		for (i = 0; i < AXIS_N; i++) {
			st_shift_cust[i] = st_avg[i] - reg_avg[i];
			if (st_shift_cust[i] < (ICM_ACCEL_ST_SHIFT_DELTA_MIN *
						st_shift_prod[i])) {
				ret_val = 1;
				if (st->dbg & NVI_DBG_SPEW_MSG)
					dev_info(&st->i2c->dev,
						 "%s FAIL A (min) axis %d\n",
						 __func__, i);
			}
			if (st_shift_cust[i] > (ICM_ACCEL_ST_SHIFT_DELTA_MAX *
						st_shift_prod[i])) {
				ret_val = 1;
				if (st->dbg & NVI_DBG_SPEW_MSG)
					dev_info(&st->i2c->dev,
						 "%s FAIL A (max) axis %d\n",
						 __func__, i);
			}
		}
	} else {
		/* Self Test Pass/Fail Criteria B */
		for (i = 0; i < AXIS_N; i++) {
			st_shift_cust[i] = abs(st_avg[i] - reg_avg[i]);
			if ((st_shift_cust[i] < ICM_ACCEL_ST_AL_MIN) ||
				    (st_shift_cust[i] > ICM_ACCEL_ST_AL_MAX)) {
				ret_val = 1;
				if (st->dbg & NVI_DBG_SPEW_MSG)
					dev_info(&st->i2c->dev,
						 "%s FAIL B axis %d\n",
						 __func__, i);
			}
		}
	}
	if (st->dbg & NVI_DBG_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s st_shift_cust: %+d %+d %+d\n",
			 __func__,
			 st_shift_cust[0], st_shift_cust[1], st_shift_cust[2]);
	return ret_val;
}

static int inv_icm_setup_selftest(struct nvi_state *st)
{
	int result;

	/* Wake up and stop sensors */
	result = nvi_pm_wr(st, INV_CLK_PLL, BIT_PWR_PRESSURE_STBY |
			   BIT_PWR_ACCEL_STBY | BIT_PWR_GYRO_STBY, 0);
	if (result)
		return result;

	/* Perform a soft-reset of the chip
	 * This will clear any prior states in the chip
	 */
	result = nvi_wr_pwr_mgmt_1(st, BIT_H_RESET);
	if (result)
		return result;

	msleep(POWER_UP_TIME);
	/* Wake up */
	result = nvi_wr_pwr_mgmt_1(st, INV_CLK_PLL);
	if (result)
		return result;

	result = nvi_wr_user_ctrl(st, BIT_FIFO_EN);
	if (result)
		return result;

	/* Configure FIFO */
	result = nvi_i2c_wr(st, REG_FIFO_CFG, 0);
	if (result)
		return result;

	/* Set cycle mode */
	result = nvi_wr_lp_config(st, 0); /* this will default to 0x70 */
	if (result)
		return result;

	/* Configure FSR and DLPF */
	result = nvi_wr_smplrt_div(st, DEV_ANGLVEL,
				   ICM_SELFTEST_GYRO_SMPLRT_DIV);
	if (result)
		return result;

	result = nvi_wr_gyro_config(st, 0, ICM_SELFTEST_GYRO_AVGCFG,
				    ICM_SELFTEST_GYRO_FS, 0);
	if (result)
		return result;

	result = nvi_wr_smplrt_div(st, DEV_ACCEL,
				   ICM_SELFTEST_ACCEL_SMPLRT_DIV);
	if (result)
		return result;

	result = nvi_wr_accel_config(st, 0, ICM_SELFTEST_ACCEL_DEC3_CFG,
				     0, ICM_SELFTEST_ACCEL_FS + 1);
	if (result)
		return result;

	/* Read selftest values */
	result = nvi_i2c_rd(st, st->hal->reg->self_test_x_gyro.bank,
			    st->hal->reg->self_test_x_gyro.reg, AXIS_N,
			    &st->st_data_gyro[0]);
	if (result)
		return result;

	result = nvi_i2c_rd(st, st->hal->reg->self_test_x_accel.bank,
			    st->hal->reg->self_test_x_accel.reg, AXIS_N,
			    &st->st_data_accel[0]);
	if (result)
		return result;

	result = nvi_pm_wr(st, INV_CLK_PLL, BIT_PWR_PRESSURE_STBY, 0);
	if (result)
		return result;

	msleep(ICM_GYRO_ENGINE_UP_TIME);
	return result;
}

static int inv_icm_selftest_read_samples(struct nvi_state *st, int dev,
				     int *sum_result, int *s)
{
	u16 w;
	u16 fifo_count;
	s16 vals[AXIS_N];
	u8 d[ICM_MAX_PACKETS * BYTES_PER_SENSOR];
	char *dev_name;
	int r;
	int i;
	int j;
	int t;
	int packet_count;

	r = nvi_wr_fifo_en(st, 0);
	if (r)
		return r;

	/* Reset FIFO */
	r = nvi_wr_user_ctrl(st, BIT_FIFO_RST);
	if (r)
		return r;

	/* enable FIFO reading */
	r = nvi_wr_user_ctrl(st, BIT_FIFO_EN);
	if (r)
		return r;

	if (DEV_ANGLVEL == dev) {
		dev_name = ICM_DEV_NAME_GYRO;
		w = 1 << st->hal->bit->gyro_x_fifo_en;
		w |= 1 << st->hal->bit->gyro_y_fifo_en;
		w |= 1 << st->hal->bit->gyro_z_fifo_en;
	} else {
		dev_name = ICM_DEV_NAME_ACCEL;
		w = 1 << st->hal->bit->accel_fifo_en;
	}
	while (*s < ICM_ST_SAMPLES) {
		r = nvi_wr_fifo_en(st, w);
		if (r)
			return r;

		msleep(ICM_SELFTEST_WAIT_TIME);
		r = nvi_wr_fifo_en(st, 0);
		if (r)
			return r;

		r = nvi_i2c_rd(st, st->hal->reg->fifo_count_h.bank,
			       st->hal->reg->fifo_count_h.reg,
			       FIFO_COUNT_BYTE, d);
		if (r)
			return r;

		fifo_count = be16_to_cpup((__be16 *)(&d[0]));
		if (st->dbg & NVI_DBG_SPEW_MSG)
			dev_info(&st->i2c->dev, "%s fifo_count=%d\n",
				 __func__, fifo_count);
		if (ICM_MAX_PACKETS * BYTES_PER_SENSOR < fifo_count) {
			r = nvi_i2c_rd(st, st->hal->reg->fifo_r_w.bank,
				       st->hal->reg->fifo_r_w.reg,
				       ICM_MAX_PACKETS * BYTES_PER_SENSOR, d);
			packet_count = ICM_MAX_PACKETS;
		} else {
			r = nvi_i2c_rd(st, st->hal->reg->fifo_r_w.bank,
				       st->hal->reg->fifo_r_w.reg,
				       fifo_count, d);
			packet_count = fifo_count / BYTES_PER_SENSOR;
		}
		if (r)
			return r;

		i = 0;
		while (i < packet_count) {
			for (j = 0; j < AXIS_N; j++) {
				t = 2 * j + i * BYTES_PER_SENSOR;
				vals[j] = (s16)be16_to_cpup((__be16 *)(&d[t]));
				sum_result[j] += vals[j];
			}
			if (st->dbg & NVI_DBG_SPEW_MSG)
				dev_info(&st->i2c->dev,
					 "%s %s %d: %+d %+d %+d\n",
					 __func__, dev_name,
					 *s, vals[0], vals[1], vals[2]);
			(*s)++;
			i++;
		}
	}

	return 0;
}

/*
 *  inv_icm_do_test_accel() - do the actual test of self testing
 */
static int inv_icm_do_test_accel(struct nvi_state *st,
				 int *accel_result, int *accel_st_result)
{
	int result;
	int i;
	int j;
	int accel_s;

	for (i = 0; i < AXIS_N; i++) {
		accel_result[i] = 0;
		accel_st_result[i] = 0;
	}
	accel_s = 0;
	result = inv_icm_selftest_read_samples(st, DEV_ACCEL,
					       accel_result, &accel_s);
	if (result)
		return result;

	for (j = 0; j < AXIS_N; j++) {
		accel_result[j] = accel_result[j] / accel_s;
		accel_result[j] *= ICM_ST_PRECISION;
	}
	/* Set Self-Test Bit */
	result = nvi_wr_accel_config(st, 7, ICM_SELFTEST_ACCEL_DEC3_CFG,
				     0, ICM_SELFTEST_ACCEL_FS + 1);
	if (result)
		return result;

	msleep(ICM_ST_STABLE_TIME);
	accel_s = 0;
	result = inv_icm_selftest_read_samples(st, DEV_ACCEL,
					       accel_st_result, &accel_s);
	if (result)
		return result;

	for (j = 0; j < AXIS_N; j++) {
		accel_st_result[j] = accel_st_result[j] / accel_s;
		accel_st_result[j] *= ICM_ST_PRECISION;
	}
	if (st->dbg & NVI_DBG_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s %d, %d, %d\n", __func__,
			 accel_result[0], accel_result[1], accel_result[2]);

	return 0;
}

/*
 *  inv_icm_do_test_gyro() - do the actual test of self testing
 */
static int inv_icm_do_test_gyro(struct nvi_state *st, int *gyro_result,
				int *gyro_st_result)
{
	int result;
	int i;
	int j;
	int gyro_s;

	for (i = 0; i < AXIS_N; i++) {
		gyro_result[i] = 0;
		gyro_st_result[i] = 0;
	}
	gyro_s = 0;
	result = inv_icm_selftest_read_samples(st, DEV_ANGLVEL,
					       gyro_result, &gyro_s);
	if (result)
		return result;

	for (j = 0; j < AXIS_N; j++) {
		gyro_result[j] = gyro_result[j] / gyro_s;
		gyro_result[j] *= ICM_ST_PRECISION;
	}
	/* Set Self-Test Bit */
	result = nvi_wr_gyro_config(st, 7, ICM_SELFTEST_GYRO_AVGCFG, -1, -1);
	if (result)
		return result;

	msleep(ICM_ST_STABLE_TIME);
	gyro_s = 0;
	result = inv_icm_selftest_read_samples(st, DEV_ANGLVEL,
					       gyro_st_result, &gyro_s);
	if (result)
		return result;

	for (j = 0; j < AXIS_N; j++) {
		gyro_st_result[j] = gyro_st_result[j] / gyro_s;
		gyro_st_result[j] *= ICM_ST_PRECISION;
	}
	if (st->dbg & NVI_DBG_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s %d, %d, %d\n", __func__,
			 gyro_result[0], gyro_result[1], gyro_result[2]);
	return 0;
}

static int inv_icm_self_test_gyro(struct nvi_state *st)
{
	int result;
	int gyro_bias_st[AXIS_N];
	int gyro_bias_regular[AXIS_N];
	int test_times;
	int i;
	char gyro_result;

	result = inv_icm_setup_selftest(st);
	if (result)
		return result;

	test_times = ICM_ST_TRY_TIMES;
	while (test_times > 0) {
		result = inv_icm_do_test_gyro(st, gyro_bias_regular,
					      gyro_bias_st);
		if (result == -EAGAIN)
			test_times--;
		else
			test_times = 0;
	}
	if (result)
		return result;

	if (st->dbg & NVI_DBG_SPEW_MSG) {
		dev_info(&st->i2c->dev, "%s gyro bias_regular: %+d %+d %+d\n",
			 __func__, gyro_bias_regular[0], gyro_bias_regular[1],
			 gyro_bias_regular[2]);
		dev_info(&st->i2c->dev, "%s gyro bias_st: %+d %+d %+d\n",
			 __func__, gyro_bias_st[0], gyro_bias_st[1],
			 gyro_bias_st[2]);
	}
	for (i = 0; i < AXIS_N; i++)
		st->gyro_bias[i] = gyro_bias_regular[i] / ICM_ST_PRECISION;
	gyro_result = inv_icm_check_gyro_self_test(st, gyro_bias_regular,
						   gyro_bias_st);
	if (st->dbg & NVI_DBG_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s gyro_result %hhd\n",
			 __func__, gyro_result);

	return gyro_result;
}

static int inv_icm_self_test_accel(struct nvi_state *st)
{
	int result;
	int accel_bias_st[AXIS_N];
	int accel_bias_regular[AXIS_N];
	int test_times;
	int i;
	char accel_result;

	result = inv_icm_setup_selftest(st);
	if (result)
		return result;

	test_times = ICM_ST_TRY_TIMES;
	while (test_times > 0) {
		result = inv_icm_do_test_accel(st, accel_bias_regular,
					       accel_bias_st);
		if (result == -EAGAIN)
			test_times--;
		else
			break;
	}
	if (result)
		return result;

	if (st->dbg & NVI_DBG_SPEW_MSG) {
		dev_info(&st->i2c->dev, "%s accel bias_regular: %+d %+d %+d\n",
			 __func__, accel_bias_regular[0],
			 accel_bias_regular[1], accel_bias_regular[2]);
		dev_info(&st->i2c->dev, "%s accel bias_st: %+d %+d %+d\n",
			 __func__, accel_bias_st[0], accel_bias_st[1],
			 accel_bias_st[2]);
	}
	for (i = 0; i < AXIS_N; i++) {
		st->accel_bias[i] = accel_bias_regular[i] / ICM_ST_PRECISION;
	}
	accel_result = inv_icm_check_accel_self_test(st, accel_bias_regular,
						     accel_bias_st);
	if (st->dbg & NVI_DBG_SPEW_MSG)
		dev_info(&st->i2c->dev, "%s accel_result %hhd\n",
			 __func__, accel_result);
	return accel_result;
}

/*
 *  inv_hw_self_test() - main function to do hardware self test
 */
int inv_hw_self_test(struct nvi_state *st, int snsr_id)
{
	int ret;

	if (st->hal->part < ICM20628) {
		if (snsr_id == DEV_ACCEL) {
			ret = inv_mpu_self_test(st);
			ret >>= DEF_ST_ACCEL_RESULT_SHIFT;
		} else if (snsr_id == DEV_ANGLVEL) {
			ret = inv_mpu_self_test(st);
			ret &= DEF_ST_ACCEL_RESULT_SHIFT;
		} else {
			ret = 0;
		}
	} else {
		if (snsr_id == DEV_ACCEL)
			ret = inv_icm_self_test_accel(st);
		else if (snsr_id == DEV_ANGLVEL)
			ret = inv_icm_self_test_gyro(st);
		else
			ret = 0;
	}
	nvi_en(st);
	return ret;
}

int inv_icm_init(struct nvi_state *st)
{
	st->chip_info.multi = 1;
	return 0;
}

