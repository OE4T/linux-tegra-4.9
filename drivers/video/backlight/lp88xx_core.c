/*
 * TI LP88XX Backlight Core Driver
 *
 * Copyright 2016 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/backlight.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pwm.h>
#include <linux/slab.h>

#include "lp88xx.h"

#define LP88XX_NUM_REGIONS		6
#define DEFAULT_BL_NAME			"lcd-bl"
#define MAX_BRIGHTNESS			0xffff
#define LP88XX_MAX_INT_STATUS		3
#define LP88XX_INT_REG_OFFSET		2

#define LP88XX_REG_CAP2		0x06
#define LP88XX_CAP2_MASK		(BIT(12) | BIT(13) | BIT(14) | BIT(15))
#define LP88XX_CAP2_SHIFT		12

#define LP88XX_REG_BL_MODE		0x20
#define LP88XX_BL_MODE_MASK		(BIT(0) | BIT(1))
#define LP88XX_PWM_MODE			0
#define LP88XX_REGISTER_MODE		2
#define LP88XX_BL_EN			BIT(8)

#define LP88XX_REG_BRT_BASE		0x26

#define LP88XX_REG_GROUP1		0x2e
#define LP88XX_REG_GROUP2		0x30
#define LP88XX_GROUP_MASK		0x0f
#define LP88XX_GROUP_OFFSET		4

#define LP88XX_REG_USER_CONFIG		0x40
#define LP88XX_EN_ADV_SLOPE		BIT(4)
#define LP88XX_SLOPE_MASK		(BIT(5) | BIT(6) | BIT(7))
#define LP88XX_SLOPE_SHIFT		5

#define LP88XX_SLOPE_SHIFT		5
#define LP88XX_INT_STATUS1		0x54
#define LP88XX_INT_STATUS2		0x56
#define LP88XX_INT_STATUS3		0x58

#define LP88XX_REG_BRT_LED1		0x13c
#define LP88XX_REG_BRT_LED2		0x148
#define LP88XX_REG_BRT_LED3		0x154
#define LP88XX_REG_BRT_LED4		0x160
#define LP88XX_REG_BRT_LED5		0x16c

#define LP88XX_REG_DB_CTRL		0x178
#define LP88XX_LOAD_BRT			BIT(0)

#define LP88XX_MAX_PWM_HZ		20000

enum lp88xx_group_led {
	LP88XX_GROUP_LED0,
	LP88XX_GROUP_LED1,
	LP88XX_GROUP_LED2,
	LP88XX_GROUP_LED3,
	LP88XX_GROUP_LED4,
	LP88XX_GROUP_LED5,
};

enum lp88xx_region_id {
	LP88XX_REGION_BASE,
	LP88XX_REGION_LED1,
	LP88XX_REGION_LED2,
	LP88XX_REGION_LED3,
	LP88XX_REGION_LED4,
	LP88XX_REGION_LED5,
};

struct lp88xx_bl {
	struct lp88xx *lp;
	struct backlight_device *bldev;
	unsigned int reg_brt;
	bool is_db_used;	/* Double buffer used or not */
};

static int lp88xx_reg_read(struct lp88xx *lp, u16 reg, u16 *val)
{
	struct lp88xx_io *io = &lp->io;
	int ret;

	ret = io->read(lp->priv, reg, val);
	if (ret)
		dev_err(lp->dev, "IO read error: %d\n", ret);

	return ret;
}

static int lp88xx_reg_write(struct lp88xx *lp, u16 reg, u16 val)
{
	struct lp88xx_io *io = &lp->io;
	int ret;

	ret = io->write(lp->priv, reg, val);
	if (ret)
		dev_err(lp->dev, "IO write error: %d\n", ret);

	return ret;
}

static int lp88xx_reg_update(struct lp88xx *lp, u16 reg, u16 mask, u16 val)
{
	int ret;
	u16 tmp = 0;

	ret = lp88xx_reg_read(lp, reg, &tmp);
	if (ret < 0)
		return ret;

	tmp &= ~mask;
	tmp |= val;

	return lp88xx_reg_write(lp, reg, tmp);
}

static bool lp88xx_is_valid_region(enum lp88xx_region_id id)
{
	if (id < LP88XX_REGION_BASE || id > LP88XX_REGION_LED5)
		return false;

	return true;
}

static int lp88xx_update_region(struct lp88xx *lp, int group, int id)
{
	unsigned int reg;
	unsigned int shift;

	switch (group) {
	case LP88XX_GROUP_LED0:
	case LP88XX_GROUP_LED1:
	case LP88XX_GROUP_LED2:
	case LP88XX_GROUP_LED3:
		reg = LP88XX_REG_GROUP1;
		shift = group * LP88XX_GROUP_OFFSET;
		break;
	case LP88XX_GROUP_LED4:
	case LP88XX_GROUP_LED5:
		reg = LP88XX_REG_GROUP2;
		shift = (group - 4) * LP88XX_GROUP_OFFSET;
		break;
	default:
		return -EINVAL;
	}

	return lp88xx_reg_update(lp, reg, LP88XX_GROUP_MASK << shift,
				 id << shift);
}

static bool lp88xx_is_new_region(struct lp88xx *lp, int id)
{
	return !test_and_set_bit(id, &lp->region_used);
}

static bool lp88xx_is_db_used(int id)
{
	return id != LP88XX_REGION_BASE;
}

static int lp88xx_bl_on(struct lp88xx *lp, int on)
{
	u16 val;

	if (on)
		val = LP88XX_BL_EN;
	else
		val = 0;

	return lp88xx_reg_update(lp, LP88XX_REG_BL_MODE, LP88XX_BL_EN, val);
}

static int lp88xx_bl_update_status(struct backlight_device *bldev)
{
	struct lp88xx_bl *bl = bl_get_data(bldev);
	struct lp88xx *lp = bl->lp;
	u32 val = bldev->props.brightness;
	int duty;
	int ret;

	val = val * lp->max_dev_brt / lp->max_input_brt;

	if (val > 0)
		lp88xx_bl_on(lp, 1);
	else
		lp88xx_bl_on(lp, 0);

	/* PWM mode */
	if (lp->pwm) {
		duty = val * lp->period / bldev->props.max_brightness;
		pwm_config(lp->pwm, duty, lp->period);
		if (duty > 0)
			pwm_enable(lp->pwm);
		else
			pwm_disable(lp->pwm);

		return 0;
	}

	/* Register mode */
	ret = lp88xx_reg_write(lp, bl->reg_brt, val);
	if (ret)
		return ret;

	/* Additional command is required when double buffer is used */
	if (bl->is_db_used)
		return lp88xx_reg_write(lp, LP88XX_REG_DB_CTRL,
					LP88XX_LOAD_BRT);

	return 0;
}

static const struct backlight_ops lp88xx_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lp88xx_bl_update_status,
};

static int lp88xx_add_bl_device(struct lp88xx *lp, int id)
{
	struct lp88xx_bl *bl;
	struct device *dev = lp->dev;
	struct backlight_properties props;
	char name[64];
	const char *pname;
	unsigned int reg_brt[] = {
		[LP88XX_REGION_BASE] = LP88XX_REG_BRT_BASE,
		[LP88XX_REGION_LED1] = LP88XX_REG_BRT_LED1,
		[LP88XX_REGION_LED2] = LP88XX_REG_BRT_LED2,
		[LP88XX_REGION_LED3] = LP88XX_REG_BRT_LED3,
		[LP88XX_REGION_LED4] = LP88XX_REG_BRT_LED4,
		[LP88XX_REGION_LED5] = LP88XX_REG_BRT_LED5,
	};

	bl = devm_kzalloc(dev, sizeof(*bl), GFP_KERNEL);
	if (!bl)
		return -ENOMEM;

	bl->lp = lp;
	bl->reg_brt = reg_brt[id];
	bl->is_db_used = lp88xx_is_db_used(id);

	memset(name, 0, sizeof(name));
	if (!of_property_read_string_index(dev->of_node, "names",
		id, &pname))
		snprintf(name, sizeof(name), "%s", pname);
	else
		snprintf(name, sizeof(name), "%s:%d", DEFAULT_BL_NAME, id);

	props.type = BACKLIGHT_PLATFORM;
	props.max_brightness = lp->max_input_brt;
	props.brightness = 0;
	of_property_read_s32(dev->of_node, "init-brt", &props.brightness);

	bl->bldev = devm_backlight_device_register(dev, name, dev, bl,
						   &lp88xx_bl_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	backlight_update_status(bl->bldev);

	return 0;
}

static int lp88xx_update_bl_mode(struct lp88xx *lp)
{
	struct pwm_device *pwm;
	unsigned int period;

	pwm = devm_pwm_get(lp->dev, NULL);
	if (IS_ERR(pwm) && PTR_ERR(pwm) != -EPROBE_DEFER) {
		dev_info(lp->dev, "Unable to request PWM, use register mode.\n");
		return lp88xx_reg_update(lp, LP88XX_REG_BL_MODE,
					 LP88XX_BL_MODE_MASK,
					 LP88XX_REGISTER_MODE);
	}

	period = pwm_get_period(pwm);
	if (period == 0 || period > LP88XX_MAX_PWM_HZ) {
		dev_err(lp->dev, "Invalid PWM frequency: %d\n", period);
		return -EINVAL;
	}

	lp->pwm = pwm;
	lp->period = period;

	return lp88xx_reg_update(lp, LP88XX_REG_BL_MODE, LP88XX_BL_MODE_MASK,
				 LP88XX_PWM_MODE);
}

static int lp88xx_get_slope_time_index(struct lp88xx *lp, unsigned int msec)
{
	unsigned int table[] = { 0, 1, 2, 50, 100, 200, 300, 500, };
	int size = ARRAY_SIZE(table);
	int i;

	if (msec <= table[0])
		return 0;

	if (msec > table[size - 1])
		return size - 1;

	for (i = 1; i < size; i++) {
		if (msec == table[i])
			return i;

		/* Find the approximate index by looking up table */
		if (msec > table[i - 1] && msec < table[i]) {
			if (msec - table[i - 1] > table[i] - msec)
				return i;
			else
				return i - 1;
		}
	}

	return 0;
}

static irqreturn_t lp88xx_irq_handler(int irq, void *ptr)
{
	struct lp88xx *lp = ptr;
	u16 status[LP88XX_MAX_INT_STATUS];
	int offset = 0;
	int i, ret;

	for (i = 0; i < LP88XX_MAX_INT_STATUS; i++) {
		ret = lp88xx_reg_read(lp, LP88XX_INT_STATUS1 + offset,
				      &status[i]);
		if (ret)
			return IRQ_NONE;

		dev_info(lp->dev, "INT STATUS %d: 0x%.4x\n", i, status[i]);

		offset += LP88XX_INT_REG_OFFSET;
	}

	return IRQ_HANDLED;
}

int lp88xx_common_probe(struct device *dev, struct lp88xx *lp)
{
	struct device_node *np = dev->of_node;
	const __be32 *list;
	u32 *region;
	u32 slope_ms;
	u8 index = 0;
	u16 val = 0;
	int en_gpio, irq_gpio;
	int i, ret, size;

	/* HW enable pin control */
	en_gpio = of_get_named_gpio(np, "enable-gpios", 0);
	if (gpio_is_valid(en_gpio)) {
		ret = devm_gpio_request_one(dev, en_gpio, GPIOF_OUT_INIT_HIGH,
					    "lp88xx-en");
		if (ret) {
			dev_err(dev, "Enable pin request error: %d\n", ret);
			return ret;
		}

		usleep_range(1000, 1500);
	}

	/* Allocate backlight device upon parsing DT property */
	list = of_get_property(np, "region-map", &size);
	if (!list) {
		dev_err(dev, "Failed to get region map\n");
		return -EINVAL;
	}

	size /= sizeof(*list);
	if (size != LP88XX_NUM_REGIONS) {
		dev_err(dev, "Invalid region info. Size: %d\n", size);
		return -EINVAL;
	}

	region = devm_kzalloc(dev, sizeof(*region) * size, GFP_KERNEL);
	if (!region)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "region-map", region, size);
	if (ret) {
		dev_err(dev, "Failed to get region map: %d\n", ret);
		return ret;
	}

	ret = lp88xx_reg_read(lp, LP88XX_REG_CAP2, &val);
	if (ret) {
		dev_warn(dev, "warning: using default max brightness\n");
		lp->max_dev_brt = MAX_BRIGHTNESS;
	} else {
		val = (val & LP88XX_CAP2_MASK) >> LP88XX_CAP2_SHIFT;
		lp->max_dev_brt = (1U << val) - 1;
	}

	if (of_property_read_u32(dev->of_node, "max-input-brt",
		&lp->max_input_brt))
		lp->max_input_brt = lp->max_dev_brt;

	for (i = 0; i < size; i++) {
		if (!lp88xx_is_valid_region(region[i])) {
			dev_err(dev, "Invalid region ID: %d\n", region[i]);
			return -EINVAL;
		}

		ret = lp88xx_update_region(lp, i, region[i]);
		if (ret) {
			dev_err(dev, "Failed to update region ID: %d\n", ret);
			return ret;
		}

		if (!lp88xx_is_new_region(lp, region[i]))
			continue;

		ret = lp88xx_add_bl_device(lp, region[i]);
		if (ret) {
			dev_err(dev, "Failed to add backlight: %d\n", ret);
			return ret;
		}
	}

	/* Backlight mode configuration */
	ret = lp88xx_update_bl_mode(lp);
	if (ret) {
		dev_err(dev, "Failed to update backlight mode: %d\n", ret);
		return ret;
	}

	/* Configure slope time */
	if (!of_property_read_u32(np, "slope-time-ms", &slope_ms)) {
		index = lp88xx_get_slope_time_index(lp, slope_ms);
		ret = lp88xx_reg_update(lp, LP88XX_REG_USER_CONFIG,
					LP88XX_SLOPE_MASK,
					index << LP88XX_SLOPE_SHIFT);
		if (ret)
			return ret;
	}

	/* Apply advanced slope feature */
	if (of_property_read_bool(np, "advanced-slope-enabled")) {
		ret = lp88xx_reg_update(lp, LP88XX_REG_USER_CONFIG,
					LP88XX_EN_ADV_SLOPE,
					LP88XX_EN_ADV_SLOPE);
		if (ret)
			return ret;
	}

	/* Interrupt pin */
	irq_gpio = of_get_named_gpio(np, "irq-gpios", 0);
	if (gpio_is_valid(irq_gpio)) {
		ret = devm_gpio_request_one(dev, irq_gpio, GPIOF_DIR_IN,
					    "lp88xx-int");
		if (ret) {
			dev_err(dev, "IRQ pin request error: %d\n", ret);
			return ret;
		}

		return devm_request_threaded_irq(lp->dev, gpio_to_irq(irq_gpio),
					NULL, lp88xx_irq_handler,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					"lp88xx-irq", lp);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(lp88xx_common_probe);
