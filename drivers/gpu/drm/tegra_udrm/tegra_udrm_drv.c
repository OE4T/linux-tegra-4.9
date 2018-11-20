/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <drm/drmP.h>

#include <uapi/drm/tegra_udrm.h>

#define DRIVER_NAME "tegra-udrm"
#define DRIVER_DESC "Kernel DRM support for user mode DRM driver on NVIDIA Tegra Soc"
/* Change driver date every year. Doesn't really signify anything. */
#define DRIVER_DATE "20182809"
/* Increase the major number when changes to driver makes it incompatible with
 * user mode driver e.g. change in ioctl args.
 */
#define DRIVER_MAJOR 0
/* Increase the minor number for minor updates which won't break compatibility
 * with user mode driver.
 */
#define DRIVER_MINOR 0

MODULE_PARM_DESC(
		modeset,
		"Enable/Disable modesetting (1 = enable, 0 = disable (default))");
static bool tegra_udrm_modeset_module_param;
module_param_named(modeset, tegra_udrm_modeset_module_param, bool, 0400);

struct tegra_udrm_private {
	struct drm_device *drm;
};

struct tegra_udrm_device {
	struct drm_device *drm;
};

struct tegra_udrm_file {
	// TODO add storage for eventfd context and idr. idr will be
	// used to save dmabuf fd => offset mappings.
	int _reserved;
};

static int tegra_udrm_mmap(struct file *file, struct vm_area_struct *vma)
{
	// TODO use offset to find dmabuf fd sotred in tegra_udrm_mmap_
	// dmabuf_ioctl(). Create CPU mapping using the dmabuf fd.
	return -ENODEV;
}

static int tegra_udrm_dmabuf_mmap_ioctl(struct drm_device *drm,
	void *data, struct drm_file *file)
{
	// TODO store dmabuf fd passed by user-space in link list in file
	// priv and return offset which user space can use in mmap(2).
	return -ENODEV;
}

static int tegra_udrm_dmabuf_destroy_mappings_ioctl(struct drm_device *drm,
	void *data, struct drm_file *file)
{
	// TODO clear stored dmabuf fds and offsets.
	return -ENODEV;
}

static void tegra_udrm_preclose(struct drm_device *drm, struct drm_file *file)
{
	// TODO signal user space using stored eventfd context in
	// tegra_udrm_set_close_notify_fd_ioctl()
}

static int tegra_udrm_close_notify_ioctl(struct drm_device *drm,
	void *data, struct drm_file *file)
{
	// TODO create eventfd context for the event fd passed by user-space
	// and store the context in file priv.
	return -ENODEV;
}

static int tegra_udrm_send_vblank_event_ioctl(struct drm_device *drm,
	void *data, struct drm_file *file)
{
	// TODO inject event sent by user-space into drm frame work using
	// drm_send_event().
	return -ENODEV;
}

static const struct file_operations tegra_udrm_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = tegra_udrm_mmap,
	.poll = drm_poll,
	.read = drm_read,
#ifdef CONFIG_COMPAT
	.compat_ioctl = drm_compat_ioctl,
#endif
	.llseek = noop_llseek,
};

static const struct drm_ioctl_desc tegra_udrm_ioctls[] = {
	DRM_IOCTL_DEF_DRV(TEGRA_UDRM_DMABUF_MMAP,
		tegra_udrm_dmabuf_mmap_ioctl, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(TEGRA_UDRM_DMABUF_DESTROY_MAPPINGS,
		tegra_udrm_dmabuf_destroy_mappings_ioctl, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(TEGRA_UDRM_CLOSE_NOTIFY,
		tegra_udrm_close_notify_ioctl, DRM_AUTH),
	DRM_IOCTL_DEF_DRV(TEGRA_UDRM_SEND_VBLANK_EVENT,
		tegra_udrm_send_vblank_event_ioctl, DRM_AUTH),
};

static int tegra_udrm_open(struct drm_device *drm, struct drm_file *filp)
{
	struct tegra_udrm_file *fpriv;

	fpriv = kzalloc(sizeof(*fpriv), GFP_KERNEL);
	if (!fpriv)
		return -ENOMEM;

	filp->driver_priv = fpriv;

	return 0;
}

static struct drm_driver tegra_udrm_driver = {
	.open              = tegra_udrm_open,
	.preclose          = tegra_udrm_preclose,
	.ioctls            = tegra_udrm_ioctls,
	.num_ioctls        = ARRAY_SIZE(tegra_udrm_ioctls),
	.fops              = &tegra_udrm_fops,

	.name   = DRIVER_NAME,
	.desc   = DRIVER_DESC,
	.date   = DRIVER_DATE,
	.major  = DRIVER_MAJOR,
	.minor  = DRIVER_MINOR,
};

static int tegra_udrm_load(struct drm_device *drm)
{
	struct platform_device *pdev = to_platform_device(drm->dev);
	struct tegra_udrm_private *private;

	private = devm_kzalloc(drm->dev, sizeof(*private), GFP_KERNEL);
	if (private == NULL)
		return -ENOMEM;

	drm->dev_private = private;

	platform_set_drvdata(pdev, drm);

	return 0;
}

static int tegra_udrm_unload(struct drm_device *drm)
{
	drm->dev_private = NULL;

	return 0;
}

static int tegra_udrm_probe(struct platform_device *pdev)
{
	struct drm_driver *driver = &tegra_udrm_driver;
	struct drm_device *drm;
	int ret;

	drm = drm_dev_alloc(driver, &pdev->dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	ret = tegra_udrm_load(drm);
	if (ret)
		goto err_unref;

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_unload;

	DRM_INFO("Initialized %s %d.%d.%d %s on minor %d\n", driver->name,
		driver->major, driver->minor, driver->patchlevel,
		driver->date, drm->primary->index);

	return 0;

err_unload:
	tegra_udrm_unload(drm);

err_unref:
	drm_dev_unref(drm);

	return ret;
}

static int tegra_udrm_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	drm_dev_unregister(drm);
	tegra_udrm_unload(drm);
	drm_dev_unref(drm);

	return 0;
}

static const struct of_device_id tegra_udrm_of_table[] = {
	{.compatible = "nvidia,tegra-udrm"},
	{}
};

MODULE_DEVICE_TABLE(of, tegra_udrm_of_table);

MODULE_ALIAS("platform:tegra_udrm");
static struct platform_driver tegra_udrm_platform_driver = {
	.probe = tegra_udrm_probe,
	.remove = tegra_udrm_remove,
	.driver = {
		.name = "tegra_udrm",
		.of_match_table = tegra_udrm_of_table,
	},
};

static int __init tegra_udrm_init(void)
{
	if (!tegra_udrm_modeset_module_param)
		return -EINVAL;

	platform_driver_register(&tegra_udrm_platform_driver);

	return 0;
}

static void __exit tegra_udrm_exit(void)
{
	platform_driver_unregister(&tegra_udrm_platform_driver);
}

module_init(tegra_udrm_init);
module_exit(tegra_udrm_exit);

MODULE_AUTHOR("NVIDIA CORPORATION");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
