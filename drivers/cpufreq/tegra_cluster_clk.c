/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <asm/cputype.h>
#include <asm/cpu.h>

/*
 * Note: These enums should be aligned to the regs mentioned in the
 * device tree
*/
enum cluster {
	A57_CLUSTER,
	D_CLUSTER,
	MAX_CLUSTERS,
};

#define for_each_cluster(cl)	for (cl = A57_CLUSTER; \
					cl < MAX_CLUSTERS; cl++)
#define NUM_CLUSTER_CLK_REG	23

struct tcl_clk_drv_data {
	void __iomem **clk_regs;
	struct platform_device *pdev;
#ifdef CONFIG_DEBUG_FS
	struct dentry *clk_root;
#endif
};

static struct tcl_clk_drv_data *cl_clk_drv_data;

#ifdef CONFIG_DEBUG_FS
#define BUFF_SIZE		50
#define REG_SIZE		4
#define RW_MODE		(S_IWUSR | S_IRUGO)
#define RO_MODE		(S_IRUGO)
enum cluster a57, d;

struct denver_creg {
	unsigned long offset;
	const char *name;
	u64 val;
};

struct denver_creg denver_cregs[] = {
	{0x00001005, "CREG_DPMU_PCLUSTER0_PWR_CTRL", 0},
	{0x00001006, "CREG_DPMU_PCLUSTER1_PWR_CTRL", 0},
};

#define NR_CREGS (sizeof(denver_cregs) / sizeof(struct denver_creg))
enum creg_command {
	CREG_INDEX = 1,
	CREG_READ,
	CREG_WRITE
};
struct cpumask denver_cpumask;

static ssize_t cl_clk_write(struct file *file, const char __user *user_buf,
		size_t count, loff_t *ppos)
{
	struct device *dev = &cl_clk_drv_data->pdev->dev;
	enum cluster *cl = file->f_path.dentry->d_inode->i_private;
	char buf[BUFF_SIZE];
	int32_t offset = -1;
	uint32_t val;

	if (count >= sizeof(buf))
		return -EINVAL;

	if (strncpy_from_user(buf, user_buf, count) <= 0)
		return -EFAULT;

	/* terminate buffer and trim - white spaces may be appended
	 * at the end when invoked from shell command line */
	buf[count] = '\0';
	strim(buf);

	if (sscanf(buf, "%d:%x", &offset, &val) != 2)
		return -EINVAL;

	if ((offset < 0) || (offset >= NUM_CLUSTER_CLK_REG)) {
		dev_err(dev, "Incorrect Args\n");
		return -EFAULT;
	}
	writel(val, cl_clk_drv_data->clk_regs[*cl] + REG_SIZE * offset);
	return count;
}

static int dump_cl_clk_reg(struct seq_file *file, void *data)
{
	enum cluster *cl = file->private;
	int i;

	for (i = 0; i < NUM_CLUSTER_CLK_REG; i++) {
		seq_printf(file, "clk_priv_%d: %u\n", i,
			readl(cl_clk_drv_data->clk_regs[*cl] + (REG_SIZE * i)));
	}

	return 0;
}

static int cl_clk_open(struct inode *inode, struct file *file)
{
	return single_open(file, dump_cl_clk_reg, inode->i_private);
}

static const struct file_operations cl_clk_fops = {
	.open = cl_clk_open,
	.llseek = seq_lseek,
	.read = seq_read,
	.write = cl_clk_write,
	.release = single_release,
};

static void __denver_creg_get(void *data)
{
	struct denver_creg *reg = (struct denver_creg *)data;
	struct device *dev = &cl_clk_drv_data->pdev->dev;

	asm volatile (
	"	sys 0, c11, c0, 1, %1\n"
	"	sys 0, c11, c0, 0, %2\n"
	"	sys 0, c11, c0, 0, %3\n"
	"	sysl %0, 0, c11, c0, 0\n"
	: "=r" (reg->val)
	: "r" (reg->offset), "r" (CREG_INDEX), "r" (CREG_READ)
	);

	dev_dbg(dev, "CREG: read %s @ 0x%lx =  0x%llx\n",
			reg->name, reg->offset, reg->val);
}

static void __denver_creg_set(void *data)
{
	struct denver_creg *reg = (struct denver_creg *)data;
	struct device *dev = &cl_clk_drv_data->pdev->dev;

	dev_dbg(dev, "CREG: write %s @ 0x%lx =  0x%llx\n",
			reg->name, reg->offset, reg->val);

	asm volatile (
	"	sys 0, c11, c0, 1, %0\n"
	"	sys 0, c11, c0, 0, %1\n"
	"	sys 0, c11, c0, 1, %2\n"
	"	sys 0, c11, c0, 0, %3\n"
	:
	: "r" (reg->offset), "r" (CREG_INDEX),
	  "r" (reg->val), "r" (CREG_WRITE)
	);
}

static int denver_creg_get(void *data, u64 *val)
{
	struct denver_creg *reg = (struct denver_creg *)data;
	smp_call_function_any(&denver_cpumask, __denver_creg_get, reg, 1);
	*val = reg->val;

	return 0;
}

static int denver_creg_set(void *data, u64 val)
{
	struct denver_creg *reg = (struct denver_creg *)data;
	reg->val = val;
	smp_call_function_any(&denver_cpumask, __denver_creg_set, reg, 1);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(denver_creg_fops, denver_creg_get,
		denver_creg_set, "%llu\n");

static void set_denver_cpumask(void)
{
	int cpu_number;
	cpumask_clear(&denver_cpumask);

	for_each_possible_cpu(cpu_number) {
		struct cpuinfo_arm64 *cpuinfo = &per_cpu(cpu_data, cpu_number);
		u32 midr = cpuinfo->reg_midr;
		if (MIDR_IMPLEMENTOR(midr) != ARM_CPU_IMP_ARM)
			cpumask_set_cpu(cpu_number, &denver_cpumask);
	}
}

static int __init tcl_clk_debug_init(struct platform_device *pdev)
{
	struct tcl_clk_drv_data *drv_data = platform_get_drvdata(pdev);
	struct dentry *tcl_clk_root = drv_data->clk_root;
	struct device *dev = &pdev->dev;
	struct denver_creg *reg;
	int i;

	tcl_clk_root = debugfs_create_dir("tegra_cluster_clk", NULL);
	if (!tcl_clk_root)
		return -ENOMEM;

	a57 = A57_CLUSTER;
	if (!debugfs_create_file("a57_cluster", RW_MODE, tcl_clk_root,
		 &a57, &cl_clk_fops))
		goto err_out;

	d = D_CLUSTER;
	if (!debugfs_create_file("denver_cluster", RW_MODE, tcl_clk_root,
		 &d, &cl_clk_fops))
		goto err_out;

	set_denver_cpumask();

	for (i = 0; i < NR_CREGS; ++i) {
		reg = &denver_cregs[i];
		if (!debugfs_create_file(
			reg->name, S_IRUGO, tcl_clk_root,
			reg, &denver_creg_fops))
			goto err_out;
	}

	return 0;
err_out:
	dev_err(dev, "Unable to create debugfs nodes\n");
	debugfs_remove_recursive(tcl_clk_root);
	return -ENOMEM;
}
#endif
static int __init tcl_clk_parse_dt(struct platform_device *pdev)
{
	struct tcl_clk_drv_data *drv_data = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	struct resource *res = NULL;
	void __iomem *clk_reg;
	enum cluster cl;
	int ret = 0;

	drv_data->clk_regs =
		devm_kzalloc(dev, sizeof(void *) * MAX_CLUSTERS,
							GFP_KERNEL);
	if (!drv_data->clk_regs) {
		dev_err(dev, "Failed to allocate regs\n");
		return -ENOMEM;
	}

	for_each_cluster(cl) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, cl);
		if (!res) {
			dev_err(dev,
			"Failed to get resource with ID %d\n",
							cl);
			ret = -EINVAL;
			goto err_out;
		}

		clk_reg = devm_ioremap_resource(dev, res);
		if (IS_ERR(clk_reg)) {
			dev_err(dev, "Failed to iomap resource reg[%d]\n",
				cl);
			ret = PTR_ERR(clk_reg);
			goto err_out;
		}
		drv_data->clk_regs[cl] = clk_reg;
	}

	return 0;
err_out:
	for_each_cluster(cl)
		if (drv_data->clk_regs[cl])
			devm_iounmap(dev, drv_data->clk_regs[cl]);

	devm_kfree(dev, drv_data->clk_regs);
	return ret;
}
static int __init tcl_clk_probe(struct platform_device *pdev)
{
	struct tcl_clk_drv_data *drv_data;
	struct device *dev = &pdev->dev;
	int ret = 0;

	dev_info(dev, "in probe()...\n");

	drv_data = devm_kzalloc(dev, sizeof(*drv_data),
				GFP_KERNEL);
	if (!drv_data) {
		dev_err(&pdev->dev, "Failed to allocate driver data\n");
		ret = -ENOMEM;
		goto err_out;
	}

	platform_set_drvdata(pdev, drv_data);

	ret = tcl_clk_parse_dt(pdev);
	if (ret)
		goto err_out;
	cl_clk_drv_data = drv_data;
	cl_clk_drv_data->pdev = pdev;

#ifdef CONFIG_DEBUG_FS
	ret = tcl_clk_debug_init(pdev);
	if (ret)
		goto err_out;
#endif
	dev_info(dev, "passed\n");

	return 0;
err_out:
	dev_info(dev, "failed\n");
	devm_kfree(dev, drv_data);
	return ret;
}
static int tcl_clk_remove(struct platform_device *pdev)
{
	struct tcl_clk_drv_data *drv_data = platform_get_drvdata(pdev);
#ifdef CONFIG_DEBUG_FS
	struct dentry *tcl_clk_root = drv_data->clk_root;
#endif
	struct device *dev = &pdev->dev;
	enum cluster cl;

	for_each_cluster(cl)
		if (drv_data->clk_regs[cl])
			devm_iounmap(dev, drv_data->clk_regs[cl]);
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(tcl_clk_root);
#endif
	devm_kfree(dev, drv_data);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tcl_clk_of_match[] = {
	{ .compatible = "nvidia,t18x-cluster-clk-priv", .data = NULL, },
	{},
};
#endif

static struct platform_driver tegra_cluster_clk_driver __refdata = {
	.driver	= {
		.name	= "tegra_cluster_clk",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tcl_clk_of_match),
	},
	.probe		= tcl_clk_probe,
	.remove		= tcl_clk_remove,
};
module_platform_driver(tegra_cluster_clk_driver);

MODULE_AUTHOR("Puneet Saxena <puneets@nvidia.com>");
MODULE_DESCRIPTION("cluster clock driver for Nvidia Tegra18x");
MODULE_LICENSE("GPL");
