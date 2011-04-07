#include <linux/platform_device.h>
#include <linux/kernel.h>

#include <mach/dc.h>
#include <mach/fb.h>

#include "dc_reg.h"
#include "dc_priv.h"
#include "nvsd.h"

/****************
 * Current mode *
 ****************/
static ssize_t mode_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct nvhost_device *ndev = to_nvhost_device(device);
	struct tegra_dc *dc = nvhost_get_drvdata(ndev);
	struct tegra_dc_mode *m;
	ssize_t res;

	mutex_lock(&dc->lock);
	m = &dc->mode;
	res = snprintf(buf, PAGE_SIZE,
		"pclk: %d\n"
		"h_ref_to_sync: %d\n"
		"v_ref_to_sync: %d\n"
		"h_sync_width: %d\n"
		"v_sync_width: %d\n"
		"h_back_porch: %d\n"
		"v_back_porch: %d\n"
		"h_active: %d\n"
		"v_active: %d\n"
		"h_front_porch: %d\n"
		"v_front_porch: %d\n"
		"stereo_mode: %d\n",
		m->pclk, m->h_ref_to_sync, m->v_ref_to_sync,
		m->h_sync_width, m->v_sync_width,
		m->h_back_porch, m->v_back_porch,
		m->h_active, m->v_active,
		m->h_front_porch, m->v_front_porch,
		m->stereo_mode);
	mutex_unlock(&dc->lock);

	return res;
}

static DEVICE_ATTR(mode, S_IRUGO, mode_show, NULL);

/**************
 * DC Enabled *
 **************/
static ssize_t enable_show(struct device *device,
	struct device_attribute *attr, char *buf)
{
	struct nvhost_device *ndev = to_nvhost_device(device);
	struct tegra_dc *dc = nvhost_get_drvdata(ndev);
	ssize_t res;

	mutex_lock(&dc->lock);
	res = snprintf(buf, PAGE_SIZE, "%d\n", dc->enabled);
	mutex_unlock(&dc->lock);
	return res;
}

static ssize_t enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct nvhost_device *ndev = to_nvhost_device(dev);
	struct tegra_dc *dc = nvhost_get_drvdata(ndev);
	int enabled;

	enabled = simple_strtoul(buf, NULL, 10);

	if (enabled) {
		tegra_dc_enable(dc);
	} else {
		tegra_dc_disable(dc);
	}

	return count;
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, enable_show, enable_store);

/********
 * Init *
 ********/
void tegra_dc_remove_sysfs(struct device *dev)
{
	struct nvhost_device *ndev = to_nvhost_device(dev);
	struct tegra_dc *dc = nvhost_get_drvdata(ndev);
	struct tegra_dc_sd_settings *sd_settings = dc->out->sd_settings;

	device_remove_file(dev, &dev_attr_mode);
	device_remove_file(dev, &dev_attr_enable);

	if(sd_settings) {
		nvsd_remove_sysfs(dev);
	}
}

void tegra_dc_create_sysfs(struct device *dev)
{
	struct nvhost_device *ndev = to_nvhost_device(dev);
	struct tegra_dc *dc = nvhost_get_drvdata(ndev);
	struct tegra_dc_sd_settings *sd_settings = dc->out->sd_settings;
	int error = 0;

	error |= device_create_file(dev, &dev_attr_mode);
	error |= device_create_file(dev, &dev_attr_enable);

	if(sd_settings) {
		error |= nvsd_create_sysfs(dev);
	}

	if(error) {
		printk("Failed to create sysfs attributes!\n");
	}
}

