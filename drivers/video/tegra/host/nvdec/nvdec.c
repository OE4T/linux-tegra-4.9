/*
 * Tegra NVDEC Module Support
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/slab.h>         /* for kzalloc */
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/nvhost.h>
#include <linux/pm_runtime.h>
#include <mach/clk.h>
#include <asm/byteorder.h>      /* for parsing ucode image wrt endianness */
#include <linux/delay.h>	/* for udelay */
#include <linux/scatterlist.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include <linux/tegra-soc.h>

#include <linux/tegra_pm_domains.h>
#include <linux/nvhost_nvdec_ioctl.h>

#include "dev.h"
#include "nvdec.h"
#include "hw_nvdec.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "nvhost_scale.h"
#include "chip_support.h"
#include "t124/t124.h"
#include "t210/t210.h"
#include "iomap.h"

#define NVDEC_IDLE_TIMEOUT_DEFAULT	10000	/* 10 milliseconds */
#define NVDEC_IDLE_CHECK_PERIOD		10	/* 10 usec */

#if USE_NVDEC_BOOTLOADER
#define get_nvdec(ndev) ((struct nvdec **)(ndev)->dev.platform_data)
#else
#define get_nvdec(ndev) ((struct nvdec *)(ndev)->dev.platform_data)
#endif
#define set_nvdec(ndev, f) ((ndev)->dev.platform_data = f)

#define BRINGUP_NO_WPR 1

/* caller is responsible for freeing */
#if USE_NVDEC_BOOTLOADER
enum {
	host_nvdec_fw_bl = 0,
	host_nvdec_fw_ls
} host_nvdec_fw;

static char *nvdec_get_fw_name(struct platform_device *dev, int fw)
{
	char *fw_name;
	u8 maj, min;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	/* note size here is a little over...*/
	fw_name = kzalloc(32, GFP_KERNEL);
	if (!fw_name)
		return NULL;

	decode_nvdec_ver(pdata->version, &maj, &min);
	if (fw == host_nvdec_fw_bl) {
#if BRINGUP_NO_WPR
		sprintf(fw_name, "nvhost_nvdec_bl_no_wpr0%d%d.fw",
			maj, min);
#else
		if (tegra_platform_is_qt() || tegra_platform_is_linsim())
			sprintf(fw_name, "nvhost_nvdec_bl_no_wpr0%d%d.fw",
				maj, min);
		else
			sprintf(fw_name, "nvhost_nvdec_bl0%d%d.fw", maj, min);
#endif
	}
	else
		sprintf(fw_name, "nvhost_nvdec0%d%d.fw", maj, min);
	dev_info(&dev->dev, "fw name:%s\n", fw_name);

	return fw_name;
}
#else
static char *nvdec_get_fw_name(struct platform_device *dev)
{
	char *fw_name;
	u8 maj, min;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	/* note size here is a little over...*/
	fw_name = kzalloc(32, GFP_KERNEL);
	if (!fw_name)
		return NULL;

	decode_nvdec_ver(pdata->version, &maj, &min);
	sprintf(fw_name, "nvhost_nvdec0%d%d.fw", maj, min);
	dev_info(&dev->dev, "fw name:%s\n", fw_name);

	return fw_name;
}
#endif

static int nvdec_dma_wait_idle(struct platform_device *dev, u32 *timeout)
{
	nvhost_dbg_fn("");

	if (!*timeout)
		*timeout = NVDEC_IDLE_TIMEOUT_DEFAULT;

	do {
		u32 check = min_t(u32, NVDEC_IDLE_CHECK_PERIOD, *timeout);
		u32 dmatrfcmd = host1x_readl(dev, nvdec_dmatrfcmd_r());
		u32 idle_v = nvdec_dmatrfcmd_idle_v(dmatrfcmd);

		if (nvdec_dmatrfcmd_idle_true_v() == idle_v) {
			nvhost_dbg_fn("done");
			return 0;
		}

		udelay(NVDEC_IDLE_CHECK_PERIOD);
		*timeout -= check;
	} while (*timeout || !tegra_platform_is_silicon());

	dev_err(&dev->dev, "dma idle timeout");

	return -1;
}

static int nvdec_dma_pa_to_internal_256b(struct platform_device *dev,
		u32 offset, u32 internal_offset, bool imem)
{
	u32 cmd = nvdec_dmatrfcmd_size_256b_f();
	u32 pa_offset =  nvdec_dmatrffboffs_offs_f(offset);
	u32 i_offset = nvdec_dmatrfmoffs_offs_f(internal_offset);
	u32 timeout = 0; /* default*/

	if (imem)
		cmd |= nvdec_dmatrfcmd_imem_true_f();

	host1x_writel(dev, nvdec_dmatrfmoffs_r(), i_offset);
	host1x_writel(dev, nvdec_dmatrffboffs_r(), pa_offset);
	host1x_writel(dev, nvdec_dmatrfcmd_r(), cmd);

	return nvdec_dma_wait_idle(dev, &timeout);

}

static int nvdec_wait_idle(struct platform_device *dev, u32 *timeout)
{
	nvhost_dbg_fn("");

	if (!*timeout)
		*timeout = NVDEC_IDLE_TIMEOUT_DEFAULT;

	do {
		u32 check = min_t(u32, NVDEC_IDLE_CHECK_PERIOD, *timeout);
		u32 w = host1x_readl(dev, nvdec_idlestate_r());

		if (!w) {
			nvhost_dbg_fn("done");
			return 0;
		}
		udelay(NVDEC_IDLE_CHECK_PERIOD);
		*timeout -= check;
	} while (*timeout || !tegra_platform_is_silicon());

	return -1;
}

static int nvdec_wait_mem_scrubbing(struct platform_device *dev)
{
	int retries = NVDEC_IDLE_TIMEOUT_DEFAULT / NVDEC_IDLE_CHECK_PERIOD;
	nvhost_dbg_fn("");

	do {
		u32 w = host1x_readl(dev, nvdec_dmactl_r()) &
			(nvdec_dmactl_dmem_scrubbing_m() |
			 nvdec_dmactl_imem_scrubbing_m());

		if (!w) {
			nvhost_dbg_fn("done");
			return 0;
		}
		udelay(NVDEC_IDLE_CHECK_PERIOD);
	} while (--retries || !tegra_platform_is_silicon());

	nvhost_err(&dev->dev, "Falcon mem scrubbing timeout");
	return -ETIMEDOUT;
}

int nvdec_boot(struct platform_device *dev)
{
	u32 timeout;
	u32 offset;
	int err = 0;
#if USE_NVDEC_BOOTLOADER
	struct nvdec **m = get_nvdec(dev);
	u32 fb_data_offset = 0;
	u32 initial_dmem_offset = 0;
	struct nvdec_bl_shared_data shared_data;
	u32 wpr_addr_lo, wpr_addr_hi;

	/* check if firmware is loaded or not */
	if (!m || !m[0] || !m[0]->valid || !m[1] || !m[1]->valid) {
		dev_err(&dev->dev, "firmware not loaded");
		return -ENOMEDIUM;
	}

	err = nvdec_wait_mem_scrubbing(dev);
	if (err)
		return err;

	fb_data_offset = (m[0]->os.bin_data_offset +
				m[0]->os.data_offset)/(sizeof(u32));

	shared_data.ls_fw_start_addr = m[1]->phys >> 8;
	shared_data.ls_fw_size = m[1]->size;
	wpr_addr_lo = readl((u32 *)IO_TO_VIRT((
				MC_BASE_ADDR +
				MC_SECURITY_CARVEOUT1_BOM_0)));
	wpr_addr_hi = readl((u32 *)IO_TO_VIRT((
				MC_BASE_ADDR +
				MC_SECURITY_CARVEOUT1_BOM_HI_0)));
	/* Put the 40-bit addr formed by wpr_addr_hi and wpr_addr_lo
	   divided by 256 into 32-bit wpr_addr */
	shared_data.wpr_addr = (wpr_addr_hi << 24) + (wpr_addr_lo >> 8);
	shared_data.wpr_size = readl((u32 *)IO_TO_VIRT((
					MC_BASE_ADDR +
					MC_SECURITY_CARVEOUT1_SIZE_128KB_0)));
	shared_data.wpr_size *= 128*1024; /* multiply 128k */

	memcpy(&(m[0]->mapped[fb_data_offset + initial_dmem_offset]),
		&shared_data, sizeof(shared_data));

	host1x_writel(dev, nvdec_dmactl_r(), 0);
	host1x_writel(dev, nvdec_dmatrfbase_r(),
		(m[0]->phys + m[0]->os.bin_data_offset) >> 8);

	/* Write BL data to dmem */
	for (offset = 0; offset < m[0]->os.data_size; offset += 256) {
		nvdec_dma_pa_to_internal_256b(dev,
					   m[0]->os.data_offset + offset,
					   offset, false);
	}

	/* Write BL code to imem */
	host1x_writel(dev, nvdec_dmatrfbase_r(),
		(m[0]->phys + m[0]->os.bin_data_offset) >> 8);
	nvdec_dma_pa_to_internal_256b(dev, m[0]->os.code_offset, 0, true);
#else /* USE_NVDEC_BOOTLOADER */
	struct nvdec *m = get_nvdec(dev);

	/* check if firmware is loaded or not */
	if (!m || !m->valid)
		return -ENOMEDIUM;

	err = nvdec_wait_mem_scrubbing(dev);
	if (err)
		return err;

	host1x_writel(dev, nvdec_dmactl_r(), 0);
	host1x_writel(dev, nvdec_dmatrfbase_r(),
		(m->phys + m->os.bin_data_offset) >> 8);

	for (offset = 0; offset < m->os.data_size; offset += 256)
		nvdec_dma_pa_to_internal_256b(dev,
					   m->os.data_offset + offset,
					   offset, false);

	nvdec_dma_pa_to_internal_256b(dev, m->os.code_offset, 0, true);
#endif /* USE_NVDEC_BOOTLOADER */
	/* setup nvdec interrupts and enable interface */
	host1x_writel(dev, nvdec_irqmset_r(),
			(nvdec_irqmset_ext_f(0xff) |
				nvdec_irqmset_swgen1_set_f() |
				nvdec_irqmset_swgen0_set_f() |
				nvdec_irqmset_exterr_set_f() |
				nvdec_irqmset_halt_set_f()   |
				nvdec_irqmset_wdtmr_set_f()));
	host1x_writel(dev, nvdec_irqdest_r(),
			(nvdec_irqdest_host_ext_f(0xff) |
				nvdec_irqdest_host_swgen1_host_f() |
				nvdec_irqdest_host_swgen0_host_f() |
				nvdec_irqdest_host_exterr_host_f() |
				nvdec_irqdest_host_halt_host_f()));
	host1x_writel(dev, nvdec_itfen_r(),
			(nvdec_itfen_mthden_enable_f() |
				nvdec_itfen_ctxen_enable_f()));

	/* boot nvdec */
	host1x_writel(dev, nvdec_bootvec_r(), nvdec_bootvec_vec_f(0));
	host1x_writel(dev, nvdec_cpuctl_r(),
			nvdec_cpuctl_startcpu_true_f());

	timeout = 0; /* default */

	err = nvdec_wait_idle(dev, &timeout);
	if (err != 0) {
		dev_err(&dev->dev, "boot failed due to timeout");
		return err;
	}

	return 0;
}

#if USE_NVDEC_BOOTLOADER
static int nvdec_setup_ucode_image(struct platform_device *dev,
		u32 *ucode_ptr,
		const struct firmware *ucode_fw,
		struct nvdec *m)
{
#else
static int nvdec_setup_ucode_image(struct platform_device *dev,
		u32 *ucode_ptr,
		const struct firmware *ucode_fw)
{
	struct nvdec *m = get_nvdec(dev);
#endif
	/* image data is little endian. */
	struct nvdec_ucode_v1 ucode;
	int w;

	/* copy the whole thing taking into account endianness */
	for (w = 0; w < ucode_fw->size / sizeof(u32); w++)
		ucode_ptr[w] = le32_to_cpu(((u32 *)ucode_fw->data)[w]);

	ucode.bin_header = (struct nvdec_ucode_bin_header_v1 *)ucode_ptr;
	/* endian problems would show up right here */
	if (ucode.bin_header->bin_magic != 0x10de) {
		dev_err(&dev->dev,
			   "failed to get firmware magic");
		return -EINVAL;
	}
	if (ucode.bin_header->bin_ver != 1) {
		dev_err(&dev->dev,
			   "unsupported firmware version");
		return -ENOENT;
	}
	/* shouldn't be bigger than what firmware thinks */
	if (ucode.bin_header->bin_size > ucode_fw->size) {
		dev_err(&dev->dev,
			   "ucode image size inconsistency");
		return -EINVAL;
	}

	nvhost_dbg_info("ucode bin header: magic:0x%x ver:%d size:%d",
		ucode.bin_header->bin_magic,
		ucode.bin_header->bin_ver,
		ucode.bin_header->bin_size);
	nvhost_dbg_info("ucode bin header: os bin (header,data) offset size: 0x%x, 0x%x %d",
		ucode.bin_header->os_bin_header_offset,
		ucode.bin_header->os_bin_data_offset,
		ucode.bin_header->os_bin_size);
	ucode.os_header = (struct nvdec_ucode_os_header_v1 *)
		(((void *)ucode_ptr) + ucode.bin_header->os_bin_header_offset);

	nvhost_dbg_info("os ucode header: os code (offset,size): 0x%x, 0x%x",
		ucode.os_header->os_code_offset,
		ucode.os_header->os_code_size);
	nvhost_dbg_info("os ucode header: os data (offset,size): 0x%x, 0x%x",
		ucode.os_header->os_data_offset,
		ucode.os_header->os_data_size);
	nvhost_dbg_info("os ucode header: num apps: %d",
		ucode.os_header->num_apps);

	m->os.size = ucode.bin_header->os_bin_size;
	m->os.bin_data_offset = ucode.bin_header->os_bin_data_offset;
	m->os.code_offset = ucode.os_header->os_code_offset;
	m->os.data_offset = ucode.os_header->os_data_offset;
	m->os.data_size   = ucode.os_header->os_data_size;

	return 0;
}

#if USE_NVDEC_BOOTLOADER
int nvdec_read_ucode(struct platform_device *dev, const char *fw_name,
			struct nvdec *m)
{
#else
int nvdec_read_ucode(struct platform_device *dev, const char *fw_name)
{
	struct nvdec *m = get_nvdec(dev);
#endif
	const struct firmware *ucode_fw;
	int err;

	m->phys = 0;
	m->mapped = NULL;
	init_dma_attrs(&m->attrs);

	ucode_fw  = nvhost_client_request_firmware(dev, fw_name);
	if (!ucode_fw) {
		dev_err(&dev->dev, "failed to get nvdec firmware %s\n",
				fw_name);
		err = -ENOENT;
		return err;
	}

	m->size = ucode_fw->size;
	dma_set_attr(DMA_ATTR_READ_ONLY, &m->attrs);

	m->mapped = dma_alloc_attrs(&dev->dev,
			m->size, &m->phys,
			GFP_KERNEL, &m->attrs);
	if (!m->mapped) {
		dev_err(&dev->dev, "dma memory allocation failed");
		err = -ENOMEM;
		goto clean_up;
	}

#if USE_NVDEC_BOOTLOADER
	err = nvdec_setup_ucode_image(dev, m->mapped, ucode_fw, m);
#else
	err = nvdec_setup_ucode_image(dev, m->mapped, ucode_fw);
#endif
	if (err) {
		dev_err(&dev->dev, "failed to parse firmware image %s\n",
				fw_name);
		goto clean_up;
	}

	m->valid = true;

	release_firmware(ucode_fw);

	return 0;

clean_up:
	if (m->mapped) {
		dma_free_attrs(&dev->dev,
				m->size, m->mapped,
				m->phys, &m->attrs);
		m->mapped = NULL;
	}
	release_firmware(ucode_fw);
	return err;
}

#if USE_NVDEC_BOOTLOADER
int nvhost_nvdec_init(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	int err = 0;
	struct nvdec **m = get_nvdec(dev);
	char **fw_name;
	int i;

	nvhost_dbg_fn("in dev:%p", dev);

	/* check if firmware resources already allocated */
	if (m)
		return 0;

	m = kzalloc(2*sizeof(struct nvdec *), GFP_KERNEL);
	if (!m) {
		dev_err(&dev->dev, "couldn't allocate ucode ptr");
		return -ENOMEM;
	}
	set_nvdec(dev, m);
	nvhost_dbg_fn("primed dev:%p", dev);

	fw_name = kzalloc(2*sizeof(char *), GFP_KERNEL);
	if (!fw_name) {
		dev_err(&dev->dev, "couldn't allocate firmware ptr");
		kfree(m);
		return -ENOMEM;
	}

	fw_name[0] = nvdec_get_fw_name(dev, host_nvdec_fw_bl);
	if (!fw_name[0]) {
		dev_err(&dev->dev, "couldn't determine BL firmware name");
		return -EINVAL;
	}
	fw_name[1] = nvdec_get_fw_name(dev, host_nvdec_fw_ls);
	if (!fw_name[1]) {
		dev_err(&dev->dev, "couldn't determine LS firmware name");
		return -EINVAL;
	}

	for (i = 0; i < 2; i++) {
		m[i] = kzalloc(sizeof(struct nvdec), GFP_KERNEL);
		if (!m[i]) {
			dev_err(&dev->dev, "couldn't alloc ucode");
			kfree(fw_name[i]);
			return -ENOMEM;
		}

		err = nvdec_read_ucode(dev, fw_name[i], m[i]);
		kfree(fw_name[i]);
		fw_name[i] = 0;
		if (err || !m[i]->valid) {
			dev_err(&dev->dev, "ucode not valid");
			goto clean_up;
		}
	}
	kfree(fw_name);
	fw_name = NULL;

	if (pdata->scaling_init)
		nvhost_scale_hw_init(dev);

	return 0;

clean_up:
	dev_err(&dev->dev, "failed");
	return err;
}
#else
int nvhost_nvdec_init(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	int err = 0;
	struct nvdec *m = get_nvdec(dev);
	char *fw_name;

	nvhost_dbg_fn("in dev:%p", dev);

	/* check if firmware resources already allocated */
	if (m)
		return 0;

	fw_name = nvdec_get_fw_name(dev);
	if (!fw_name) {
		dev_err(&dev->dev, "couldn't determine firmware name");
		return -EINVAL;
	}

	m = kzalloc(sizeof(struct nvdec), GFP_KERNEL);
	if (!m) {
		dev_err(&dev->dev, "couldn't alloc ucode");
		kfree(fw_name);
		return -ENOMEM;
	}
	set_nvdec(dev, m);
	nvhost_dbg_fn("primed dev:%p", dev);

	err = nvdec_read_ucode(dev, fw_name);
	kfree(fw_name);
	fw_name = 0;

	if (err || !m->valid) {
		dev_err(&dev->dev, "ucode not valid");
		goto clean_up;
	}

	if (pdata->scaling_init)
		nvhost_scale_hw_init(dev);

	return 0;

clean_up:
	dev_err(&dev->dev, "failed");
	return err;
}
#endif

#if USE_NVDEC_BOOTLOADER
void nvhost_nvdec_deinit(struct platform_device *dev)
{
	struct nvdec **m = get_nvdec(dev);
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	int i;

	if (pdata->scaling_init)
		nvhost_scale_hw_deinit(dev);

	if (!m)
		return;

	for (i = 0; i < 2; i++) {
		if (!m[i])
			continue;
		/* unpin, free ucode memory */
		if (m[i]->mapped) {
			dma_free_attrs(&dev->dev,
					m[i]->size, m[i]->mapped,
					m[i]->phys, &m[i]->attrs);
			m[i]->mapped = NULL;
		}
		kfree(m[i]);
		m[i]->valid = false;
	}
	kfree(m);
	m = NULL;
	set_nvdec(dev, NULL);
}
#else
void nvhost_nvdec_deinit(struct platform_device *dev)
{
	struct nvdec *m = get_nvdec(dev);
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	if (pdata->scaling_init)
		nvhost_scale_hw_deinit(dev);

	if (!m)
		return;

	/* unpin, free ucode memory */
	if (m->mapped) {
		dma_free_attrs(&dev->dev,
				m->size, m->mapped,
				m->phys, &m->attrs);
		m->mapped = NULL;
	}
	set_nvdec(dev, NULL);
	m->valid = false;
	kfree(m);
}
#endif

int nvhost_nvdec_finalize_poweron(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	nvhost_module_reset(dev, false);

	if (!pdata->can_slcg) {
		host1x_writel(dev, nvdec_slcg_override_high_a_r(), 0xff);
		host1x_writel(dev, nvdec_slcg_override_low_a_r(), 0xffffffff);
	}

	host1x_writel(dev, 0x117c, 0x18004);

	host1x_writel(dev, 0x2314, 0x10940000);
	host1x_writel(dev, 0x2318, 0xff00a725);
	host1x_writel(dev, 0x2328, 0x0);
	host1x_writel(dev, 0x232c, 0x80000);
	host1x_writel(dev, 0x2330, 0xfffffff8);

	return nvdec_boot(dev);
}

static struct of_device_id tegra_nvdec_of_match[] = {
#ifdef TEGRA_21X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra210-nvdec",
		.data = (struct nvhost_device_data *)&t21_nvdec_info },
#endif
	{ },
};

static int nvdec_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata;
	struct nvdec_private *priv;

	pdata = container_of(inode->i_cdev,
		struct nvhost_device_data, ctrl_cdev);

	if (WARN_ONCE(pdata == NULL,
			"pdata not found, %s failed\n", __func__))
		return -ENODEV;

	if (WARN_ONCE(pdata->pdev == NULL,
			"device not found, %s failed\n", __func__))
		return -ENODEV;

	priv = kzalloc(sizeof(struct nvdec_private), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdata->pdev->dev,
			"couldn't allocate nvdec private");
		return -ENOMEM;
	}
	priv->pdev = pdata->pdev;
	atomic_set(&priv->refcnt, 0);

	file->private_data = priv;

	return 0;
}

long nvdec_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	struct nvdec_private *priv = file->private_data;
	struct platform_device *pdev = priv->pdev;
	int err;

	if (WARN_ONCE(pdev == NULL, "pdata not found, %s failed\n", __func__))
		return -ENODEV;

	if (_IOC_TYPE(cmd) != NVHOST_NVDEC_IOCTL_MAGIC)
		return -EFAULT;

	switch (cmd) {
	case NVHOST_NVDEC_IOCTL_POWERON:
		nvhost_nvdec_init(pdev);
		err = nvhost_module_busy(pdev);
		if (err)
			return err;

		atomic_inc(&priv->refcnt);
	break;
	case NVHOST_NVDEC_IOCTL_POWEROFF:
		if (atomic_dec_if_positive(&priv->refcnt)) {
			nvhost_module_idle(pdev);
			nvhost_nvdec_deinit(pdev);
		}
	break;
	default:
		dev_err(&pdev->dev,
			"%s: Unknown nvdec ioctl.\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int nvdec_release(struct inode *inode, struct file *file)
{
	struct nvdec_private *priv = file->private_data;

	nvhost_module_idle_mult(priv->pdev, atomic_read(&priv->refcnt));
	kfree(priv);

	return 0;
}
static int nvdec_probe(struct platform_device *dev)
{
	int err = 0;
	struct nvhost_device_data *pdata = NULL;

	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_nvdec_of_match, &dev->dev);
		if (match)
			pdata = (struct nvhost_device_data *)match->data;
	} else
		pdata = (struct nvhost_device_data *)dev->dev.platform_data;

	WARN_ON(!pdata);
	if (!pdata) {
		dev_info(&dev->dev, "no platform data\n");
		return -ENODATA;
	}

	err = nvhost_check_bondout(pdata->bond_out_id);
	if (err) {
		dev_err(&dev->dev, "No NVDEC unit present. err:%d", err);
		return err;
	}

	pdata->pdev = dev;

	mutex_init(&pdata->lock);

	platform_set_drvdata(dev, pdata);
	dev->dev.platform_data = NULL;

	/* get the module clocks to sane state */
	nvhost_module_init(dev);

#ifdef CONFIG_PM_GENERIC_DOMAINS
	pdata->pd.name = "nvdec";

	/* add module power domain and also add its domain
	 * as sub-domain of MC domain */
	err = nvhost_module_add_domain(&pdata->pd, dev);
#endif

	err = nvhost_client_device_get_resources(dev);
	if (err)
		return err;

	err = nvhost_client_device_init(dev);

	return 0;
}

static int __exit nvdec_remove(struct platform_device *dev)
{
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_put(&dev->dev);
	pm_runtime_disable(&dev->dev);
#else
	nvhost_module_disable_clk(&dev->dev);
#endif
	return 0;
}

static struct platform_driver nvdec_driver = {
	.probe = nvdec_probe,
	.remove = __exit_p(nvdec_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "nvdec",
#ifdef CONFIG_OF
		.of_match_table = tegra_nvdec_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	}
};

const struct file_operations tegra_nvdec_ctrl_ops = {
	.owner = THIS_MODULE,
	.open = nvdec_open,
	.unlocked_ioctl = nvdec_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nvdec_ioctl,
#endif
	.release = nvdec_release,
};

static int __init nvdec_init(void)
{
	return platform_driver_register(&nvdec_driver);
}

static void __exit nvdec_exit(void)
{
	platform_driver_unregister(&nvdec_driver);
}

module_init(nvdec_init);
module_exit(nvdec_exit);
