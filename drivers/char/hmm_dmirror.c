/*
 * Copyright 2013 Red Hat Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Authors: Jérôme Glisse <jglisse@redhat.com>
 */
/*
 * This is a dummy driver to exercice the HMM (heterogeneous memory management)
 * mirror API of the kernel. Userspace program register with the dummy device
 * to mirror their own address space and can use the device to read/write to
 * any valid virtual address.
 *
 * In some way it can also serve as an example driver for people wanting to use
 * HMM inside there own device driver.
 */
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/delay.h>
#include <linux/pagemap.h>
#include <linux/hmm.h>
#include <linux/vmalloc.h>
#include <linux/swap.h>
#include <linux/swapops.h>
#include <linux/platform_device.h>

#include <uapi/linux/hmm_dmirror.h>

struct dmirror_device;

struct dummy_bounce {
	void			*ptr;
	unsigned long		size;
	unsigned long		addr;
	unsigned long		cpages;
};

#define DPT_SHIFT PAGE_SHIFT
#define DPT_VALID (1 << 0)
#define DPT_WRITE (1 << 1)
#define DPT_DPAGE (1 << 2)

struct dmirror_pt {
	unsigned long		pgd[PTRS_PER_PGD];
	struct rw_semaphore	lock;
};

struct dmirror {
	struct dmirror_device	*mdevice;
	struct file		*filp;
	struct hmm_mirror	mirror;
	struct mm_struct	*mm;
	struct dmirror_pt	pt;
};

struct dmirror_device {
	dev_t			dev;
	struct cdev		cdevice;
	struct platform_device	*pdevice;
};

static inline unsigned long dmirror_pt_pgd(unsigned long addr)
{
	return (addr >> PGDIR_SHIFT) & (PTRS_PER_PGD - 1);
}

static inline unsigned long dmirror_pt_pud(unsigned long addr)
{
	return (addr >> PUD_SHIFT) & (PTRS_PER_PUD - 1);
}

static inline unsigned long dmirror_pt_pmd(unsigned long addr)
{
	return (addr >> PMD_SHIFT) & (PTRS_PER_PMD - 1);
}

static inline unsigned long dmirror_pt_pte(unsigned long addr)
{
	return (addr >> PAGE_SHIFT) & (PTRS_PER_PTE - 1);
}

static inline struct page *dmirror_pt_page(unsigned long dpte)
{
	if (!(dpte & DPT_VALID))
		return NULL;
	return pfn_to_page(dpte >> DPT_SHIFT);
}

static inline unsigned long dmirror_pt_from_page(struct page *page)
{
	if (!page)
		return 0;
	return (page_to_pfn(page) << DPT_SHIFT) | DPT_VALID;
}

static inline unsigned long dmirror_pt_pud_end(unsigned long addr)
{
	return (addr & PGDIR_MASK) + ((long)PTRS_PER_PUD << PUD_SHIFT);
}

static inline unsigned long dmirror_pt_pmd_end(unsigned long addr)
{
	return (addr & PUD_MASK) + ((long)PTRS_PER_PMD << PMD_SHIFT);
}

static inline unsigned long dmirror_pt_pte_end(unsigned long addr)
{
	return (addr & PMD_MASK) + ((long)PTRS_PER_PTE << PAGE_SHIFT);
}

typedef int (*dmirror_walk_cb_t)(struct dmirror *dmirror,
				 unsigned long start,
				 unsigned long end,
				 unsigned long *dpte,
				 void *private);

static int dummy_pt_walk(struct dmirror *dmirror,
			 dmirror_walk_cb_t cb,
			 unsigned long start,
			 unsigned long end,
			 void *private,
			 bool populate)
{
	unsigned long *dpgd = &dmirror->pt.pgd[dmirror_pt_pgd(start)];
	unsigned long addr = start & PAGE_MASK;

	BUG_ON(start == end);

	for (; addr != end; dpgd++) {
		unsigned long pud_end, *dpud;
		struct page *pud_page;

		pud_end = min(end, dmirror_pt_pud_end(addr));
		pud_page = dmirror_pt_page(*dpgd);
		if (!pud_page) {
			if (!populate) {
				addr = pud_end;
				continue;
			}
			pud_page = alloc_page(GFP_HIGHUSER | __GFP_ZERO);
			if (!pud_page) {
				return -ENOMEM;
			}
			*dpgd = dmirror_pt_from_page(pud_page);
		}
		dpud = kmap(pud_page);
		dpud = &dpud[dmirror_pt_pud(addr)];
		for (; addr != pud_end; dpud++) {
			unsigned long pmd_end, *dpmd;
			struct page *pmd_page;

			pmd_end = min(end, dmirror_pt_pmd_end(addr));
			pmd_page = dmirror_pt_page(*dpud);
			if (!pmd_page) {
				if (!populate) {
					addr = pmd_end;
					continue;
				}
				pmd_page = alloc_page(GFP_HIGHUSER | __GFP_ZERO);
				if (!pmd_page) {
					kunmap(pud_page);
					return -ENOMEM;
				}
				*dpud = dmirror_pt_from_page(pmd_page);
			}
			dpmd = kmap(pmd_page);
			dpmd = &dpmd[dmirror_pt_pmd(addr)];
			for (; addr != pmd_end; dpmd++) {
				unsigned long *dpte, pte_end;
				struct page *pte_page;
				int ret;

				pte_end = min(end, dmirror_pt_pte_end(addr));
				pte_page = dmirror_pt_page(*dpmd);
				if (!pte_page) {
					if (!populate) {
						addr = pte_end;
						continue;
					}
					pte_page = alloc_page(GFP_HIGHUSER | __GFP_ZERO);
					if (!pte_page) {
						kunmap(pmd_page);
						kunmap(pud_page);
						return -ENOMEM;
					}
					*dpmd = dmirror_pt_from_page(pte_page);
				}
				dpte = kmap(pte_page);
				dpte = &dpte[dmirror_pt_pte(addr)];
				ret = cb(dmirror, addr, pte_end, dpte, private);
				kunmap(pte_page);
				addr = pte_end;
				if (ret) {
					kunmap(pmd_page);
					kunmap(pud_page);
					return ret;
				}
			}
			kunmap(pmd_page);
			addr = pmd_end;
		}
		kunmap(pud_page);
		addr = pud_end;
	}

	return 0;
}

int dummy_bounce_init(struct dummy_bounce *bounce,
		      unsigned long size,
		      unsigned long addr)
{
	bounce->addr = addr;
	bounce->size = size;
	bounce->ptr = vmalloc(size);
	if (!bounce->ptr)
		return -ENOMEM;
	return 0;
}

int dummy_bounce_copy_from(struct dummy_bounce *bounce, unsigned long addr)
{
	unsigned long end = (addr & PAGE_MASK) + bounce->size;
	char __user *uptr = (void __user *)(addr & PAGE_MASK);
	void *ptr = bounce->ptr;

	for (; addr < end; addr += PAGE_SIZE, ptr += PAGE_SIZE, uptr += PAGE_SIZE) {
		int ret;

		ret = copy_from_user(ptr, uptr, PAGE_SIZE);
		if (ret)
			return ret;
	}

	return 0;
}

int dummy_bounce_copy_to(struct dummy_bounce *bounce, unsigned long addr)
{
	unsigned long end = (addr & PAGE_MASK) + bounce->size;
	char __user *uptr = (void __user *)(addr & PAGE_MASK);
	void *ptr = bounce->ptr;

	for (; addr < end; addr += PAGE_SIZE, ptr += PAGE_SIZE, uptr += PAGE_SIZE) {
		int ret;

		ret = copy_to_user(uptr, ptr, PAGE_SIZE);
		if (ret)
			return ret;
	}

	return 0;
}

void dummy_bounce_fini(struct dummy_bounce *bounce)
{
	vfree(bounce->ptr);
}

static int dummy_do_update(struct dmirror *dmirror,
			   unsigned long addr,
			   unsigned long end,
			   unsigned long *dpte,
			   void *private)
{
	for (; addr < end; addr += PAGE_SIZE, ++dpte) {
		/* Clear pte */
		*dpte = 0;
	}

	return 0;
}

static void dummy_update(struct hmm_mirror *mirror,
			 enum hmm_update_type update,
			 unsigned long start,
			 unsigned long end)
{
	struct dmirror *dmirror = container_of(mirror, struct dmirror, mirror);

	down_write(&dmirror->pt.lock);
	dummy_pt_walk(dmirror, dummy_do_update, start, end, NULL, false);
	up_write(&dmirror->pt.lock);
}

static const struct hmm_mirror_ops dmirror_ops = {
	.sync_cpu_device_pagetables	= &dummy_update,
};


static int dmirror_pt_init(struct dmirror *dmirror)
{
	init_rwsem(&dmirror->pt.lock);
	return 0;
}

/* dmirror_new() - allocate and initialize dummy mirror struct.
 *
 * @mdevice: The dummy device this mirror is associated with.
 * @filp: The active device file descriptor this mirror is associated with.
 */
static struct dmirror *dmirror_new(struct dmirror_device *mdevice,
				   struct file *filp)
{
	struct mm_struct *mm = get_task_mm(current);
	struct dmirror *dmirror;
	int r;

	if (!mm)
		return NULL;

	/* Mirror this process address space */
	dmirror = kzalloc(sizeof(*dmirror), GFP_KERNEL);
	if (dmirror == NULL)
		return NULL;
	dmirror->mdevice = mdevice;
	dmirror->filp = filp;
	if (dmirror_pt_init(dmirror)) {
		kfree(dmirror);
		return NULL;
	}

	dmirror->mm = mm;
	dmirror->mirror.ops = &dmirror_ops;
	down_write(&mm->mmap_sem);
	r = hmm_mirror_register(&dmirror->mirror, mm);
	up_write(&mm->mmap_sem);
	mmput(mm);

	if (r) {
		kfree(dmirror);
		return NULL;
	}

	return dmirror;
}

static void dmirror_del(struct dmirror *dmirror)
{
	hmm_mirror_unregister(&dmirror->mirror);
	kfree(dmirror);
}


/*
 * Below are the file operation for the dummy device file. Only ioctl matter.
 *
 * Note this is highly specific to the dummy device driver and should not be
 * construed as an example on how to design the API a real device driver would
 * expose to userspace.
 */
static ssize_t dummy_fops_read(struct file *filp,
			       char __user *buf,
			       size_t count,
			       loff_t *ppos)
{
	return -EINVAL;
}

static ssize_t dummy_fops_write(struct file *filp,
				const char __user *buf,
				size_t count,
				loff_t *ppos)
{
	return -EINVAL;
}

static int dummy_fops_mmap(struct file *filp, struct vm_area_struct *vma)
{
	/* Forbid mmap of the dummy device file. */
	return -EINVAL;
}

static int dummy_fops_open(struct inode *inode, struct file *filp)
{
	struct cdev *cdev = inode->i_cdev;
	struct dmirror_device *mdevice;
	struct dmirror *dmirror;

	/* No exclusive opens. */
	if (filp->f_flags & O_EXCL)
		return -EINVAL;

	mdevice = container_of(cdev, struct dmirror_device, cdevice);
	dmirror = dmirror_new(mdevice, filp);
	filp->private_data = dmirror;

	return dmirror ? 0 : -ENOMEM;
}

static int dummy_fops_release(struct inode *inode, struct file *filp)
{
	struct dmirror *dmirror;

	if (!filp->private_data)
		return 0;

	dmirror = filp->private_data;
	dmirror_del(dmirror);
	filp->private_data = NULL;

	return 0;
}

struct dummy_fault {
	hmm_pfn_t		*pfns;
	unsigned long		start;
	unsigned long		missing;
	bool			write;
	bool			invalid;
};

static int dummy_do_fault(struct dmirror *dmirror,
			  unsigned long addr,
			  unsigned long end,
			  unsigned long *dpte,
			  void *private)
{
	struct dummy_fault *dfault = private;
	unsigned long idx = (addr - dfault->start) >> PAGE_SHIFT;
	hmm_pfn_t *pfns = dfault->pfns;

	for (; addr < end; addr += PAGE_SIZE, ++dpte, ++idx) {
		struct page *page;

		/*
		 * Special pfn are device memory ie page inserted inside the
		 * CPU page table with either vm_insert_pfn or vm_insert_page
		 * in both case we assume that device can not access this
		 * memory safely.
		 *
		 * The HMM_PFN_ERROR is if it is accessing invalid memory
		 * either because of memory error (hardware detected memory
		 * corruption) or more likely because of truncate on mmap
		 * file.
		 */
		if ((pfns[idx] & (HMM_PFN_SPECIAL | HMM_PFN_ERROR))) {
			dfault->invalid = true;
			continue;
		}
		if (!(pfns[idx] & HMM_PFN_VALID)) {
			dfault->missing++;
			continue;
		}
		page = hmm_pfn_t_to_page(pfns[idx]);
		*dpte = dmirror_pt_from_page(page);
		if (pfns[idx] & HMM_PFN_WRITE) {
			*dpte |= DPT_WRITE;
		} else if (dfault->write) {
			dfault->missing++;
		}
	}

	return 0;
}

static int dummy_fault(struct dmirror *dmirror,
		       unsigned long start,
		       unsigned long end,
		       bool write)
{
	struct mm_struct *mm = dmirror->mm;
	unsigned long addr = start;
	hmm_pfn_t pfns[64];

	do {
		struct vm_area_struct *vma;
		struct dummy_fault dfault;
		struct hmm_range range;
		unsigned long next;
		int ret;

		down_read(&mm->mmap_sem);
		next = min(addr + (64 << PAGE_SHIFT), end);
		vma = find_vma_intersection(mm, addr, end);
		if (!vma) {
			up_read(&mm->mmap_sem);
			return -EFAULT;
		}
		if (!(vma->vm_flags & VM_READ)) {
			up_read(&mm->mmap_sem);
			return -EFAULT;
		}
		if (write && !(vma->vm_flags & VM_WRITE)) {
			up_read(&mm->mmap_sem);
			return -EFAULT;
		}
		addr = max(vma->vm_start, addr);
		next = min(min(addr + (64 << PAGE_SHIFT), end), vma->vm_end);

		ret = hmm_vma_fault(vma, &range, addr, next, pfns, write, false);
		switch (ret) {
		case 0:
			break;
		case -EAGAIN:
			continue;
		default:
			up_read(&mm->mmap_sem);
			return ret;
		}
		down_read(&dmirror->pt.lock);
		if (!hmm_vma_range_done(vma, &range)) {
			up_read(&dmirror->pt.lock);
			up_read(&mm->mmap_sem);
			continue;
		}

		dfault.invalid = false;
		dfault.write = write;
		dfault.missing = 0;
		dfault.start = addr;
		dfault.pfns = pfns;
		ret = dummy_pt_walk(dmirror, dummy_do_fault,
				    addr, next, &dfault, true);
		up_read(&dmirror->pt.lock);
		up_read(&mm->mmap_sem);
		if (ret)
			return ret;
		if (dfault.invalid)
			return -EFAULT;

		if (!dfault.missing) {
			addr = next;
		} else {
			return -EFAULT;
		}
	} while (addr != end);

	return 0;
}

static int dummy_do_read(struct dmirror *dmirror,
			 unsigned long addr,
			 unsigned long end,
			 unsigned long *dpte,
			 void *private)
{
	struct dmirror_device *mdevice = dmirror->mdevice;
	struct dummy_bounce *bounce = private;
	void *ptr;

	ptr = bounce->ptr + ((addr - bounce->addr) & PAGE_MASK);

	for (; addr < end; addr += PAGE_SIZE, ++dpte) {
		struct page *page;
		void *tmp;

		page = dmirror_pt_page(*dpte);
		if (!page) {
			return -ENOENT;
		}

		tmp = kmap(page);
		memcpy(ptr, tmp, PAGE_SIZE);
		kunmap(page);

		ptr += PAGE_SIZE;
		bounce->cpages++;
	}

	return 0;
}

static int dummy_read(struct dmirror *dmirror,
		      struct hmm_dmirror_read *dread)
{
	struct dummy_bounce bounce;
	unsigned long start, end;
	unsigned long size;
	int ret;

	if ((dread->ptr & (~PAGE_MASK)) || (dread->addr & (~PAGE_MASK)))
		return -EINVAL;
	if (dread->addr >= (dread->addr + (dread->npages << PAGE_SHIFT)))
		return -EINVAL;

	start = dread->addr & PAGE_MASK;
	size = (dread->npages << PAGE_SHIFT);
	end = start + (dread->npages << PAGE_SHIFT);

	ret = dummy_bounce_init(&bounce, size, start);
	if (ret)
		return ret;

again:
	dread->dpages = 0;
	bounce.cpages = 0;
	down_read(&dmirror->pt.lock);
	ret = dummy_pt_walk(dmirror, dummy_do_read,
			    start, end, &bounce, true);
	up_read(&dmirror->pt.lock);

	if (ret == -ENOENT) {
		ret = dummy_fault(dmirror, start, end, false);
		if (ret) {
			dummy_bounce_fini(&bounce);
			return ret;
		}
		goto again;
	}

	ret = dummy_bounce_copy_to(&bounce, dread->ptr);
	dread->cpages = bounce.cpages;
	dummy_bounce_fini(&bounce);
	return ret;
}

static int dummy_do_write(struct dmirror *dmirror,
			  unsigned long addr,
			  unsigned long end,
			  unsigned long *dpte,
			  void *private)
{
	struct dmirror_device *mdevice = dmirror->mdevice;
	struct dummy_bounce *bounce = private;
	void *ptr;

	ptr = bounce->ptr + ((addr - bounce->addr) & PAGE_MASK);

	for (; addr < end; addr += PAGE_SIZE, ++dpte) {
		struct page *page;
		void *tmp;

		page = dmirror_pt_page(*dpte);
		if (!page || !(*dpte & DPT_WRITE))
			return -ENOENT;

		tmp = kmap(page);
		memcpy(tmp, ptr, PAGE_SIZE);
		kunmap(page);

		ptr += PAGE_SIZE;
		bounce->cpages++;
	}

	return 0;
}

static int dummy_write(struct dmirror *dmirror,
		       struct hmm_dmirror_write *dwrite)
{
	struct dummy_bounce bounce;
	unsigned long start, end;
	unsigned long size;
	int ret;

	if ((dwrite->ptr & (~PAGE_MASK)) || (dwrite->addr & (~PAGE_MASK)))
		return -EINVAL;
	if (dwrite->addr >= (dwrite->addr + (dwrite->npages << PAGE_SHIFT)))
		return -EINVAL;

	start = dwrite->addr & PAGE_MASK;
	size = (dwrite->npages << PAGE_SHIFT);
	end = start + (dwrite->npages << PAGE_SHIFT);

	ret = dummy_bounce_init(&bounce, size, dwrite->addr & PAGE_MASK);
	if (ret)
		return ret;
	ret = dummy_bounce_copy_from(&bounce, dwrite->ptr);
	if (ret)
		return ret;

again:
	bounce.cpages = 0;
	dwrite->dpages = 0;
	down_read(&dmirror->pt.lock);
	ret = dummy_pt_walk(dmirror, dummy_do_write,
			    start, end, &bounce, true);
	up_read(&dmirror->pt.lock);

	if (ret == -ENOENT) {
		ret = dummy_fault(dmirror, start, end, true);
		if (ret) {
			dummy_bounce_fini(&bounce);
			return ret;
		}
		goto again;
	}

	dwrite->cpages = bounce.cpages;
	dummy_bounce_fini(&bounce);
	return 0;
}

static long dummy_fops_unlocked_ioctl(struct file *filp,
				      unsigned int command,
				      unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct hmm_dmirror_write dwrite;
	struct hmm_dmirror_read dread;
	struct dmirror *dmirror;
	int ret;

	dmirror = filp->private_data;
	if (!dmirror)
		return -EINVAL;

	switch (command) {
	case HMM_DMIRROR_READ:
		ret = copy_from_user(&dread, uarg, sizeof(dread));
		if (ret)
			return ret;

		ret = dummy_read(dmirror, &dread);
		if (ret)
			return ret;

		return copy_to_user(uarg, &dread, sizeof(dread));
	case HMM_DMIRROR_WRITE:
		ret = copy_from_user(&dwrite, uarg, sizeof(dwrite));
		if (ret)
			return ret;

		ret = dummy_write(dmirror, &dwrite);
		if (ret)
			return ret;

		return copy_to_user(uarg, &dwrite, sizeof(dwrite));
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct file_operations dmirror_fops = {
	.read		= dummy_fops_read,
	.write		= dummy_fops_write,
	.mmap		= dummy_fops_mmap,
	.open		= dummy_fops_open,
	.release	= dummy_fops_release,
	.unlocked_ioctl = dummy_fops_unlocked_ioctl,
	.llseek		= default_llseek,
	.owner		= THIS_MODULE,
};

static int dmirror_probe(struct platform_device *pdev)
{
	struct dmirror_device *mdevice = platform_get_drvdata(pdev);
	int ret;

	ret = alloc_chrdev_region(&mdevice->dev, 0, 1, "HMM_DMIRROR");
	if (ret < 0) {
		return ret;
	}

	cdev_init(&mdevice->cdevice, &dmirror_fops);
	ret = cdev_add(&mdevice->cdevice, mdevice->dev, 1);
	if (ret) {
		unregister_chrdev_region(mdevice->dev, 1);
		return ret;
	}

	return 0;
}

static int dmirror_remove(struct platform_device *pdev)
{
	struct dmirror_device *mdevice = platform_get_drvdata(pdev);

	cdev_del(&mdevice->cdevice);
	unregister_chrdev_region(mdevice->dev, 1);
	return 0;
}


static struct platform_device *dmirror_platform_device;

static struct platform_driver dmirror_device_driver = {
	.probe		= dmirror_probe,
	.remove		= dmirror_remove,
	.driver		= {
		.name	= "HMM_DMIRROR",
	},
};

static int __init hmm_dmirror_init(void)
{
	struct dmirror_device *mdevice;
	int ret;

	mdevice = kzalloc(sizeof(*mdevice), GFP_KERNEL);
	if (!mdevice)
		return -ENOMEM;

	dmirror_platform_device = platform_device_alloc("HMM_DMIRROR", -1);
	if (!dmirror_platform_device) {
		kfree(mdevice);
		return -ENOMEM;
	}
	platform_set_drvdata(dmirror_platform_device, mdevice);
	mdevice->pdevice = dmirror_platform_device;

	ret = platform_device_add(dmirror_platform_device);
	if (ret < 0) {
		platform_device_put(dmirror_platform_device);
		return ret;
	}

	ret = platform_driver_register(&dmirror_device_driver);
	if (ret < 0) {
		platform_device_unregister(dmirror_platform_device);
		return ret;
	}

	pr_info("hmm_dmirror loaded THIS IS A DANGEROUS MODULE !!!\n");
	return 0;
}

static void __exit hmm_dmirror_exit(void)
{
	struct dmirror_device *mdevice;

	mdevice = platform_get_drvdata(dmirror_platform_device);
	platform_driver_unregister(&dmirror_device_driver);
	platform_device_unregister(dmirror_platform_device);
	kfree(mdevice);
}

module_init(hmm_dmirror_init);
module_exit(hmm_dmirror_exit);
MODULE_LICENSE("GPL");
