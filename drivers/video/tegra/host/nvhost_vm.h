/*
 * Tegra Graphics Host Virtual Memory
 *
 * Copyright (c) 2014-2015, NVIDIA Corporation. All rights reserved.
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

#ifndef NVHOST_VM_H
#define NVHOST_VM_H

#include <linux/kref.h>

struct platform_device;
struct nvhost_vm_pin;
struct dma_buf;
struct dma_buf_attachment;
struct sg_table;

struct nvhost_vm {
	struct platform_device *pdev;

	struct kref kref;	/* reference to this VM */
	struct mutex mutex;

	/* rb-tree of buffers mapped into this VM */
	struct rb_root buffer_list;

	/* count of application viewed buffers mapped into this VM */
	unsigned int num_user_mapped_buffers;

	/* used by hardware layer */
	void *private_data;

	/* to track all vms in the system */
	struct list_head vm_list;

	/* does this context required hardware backing? */
	bool enable_hw;
};

struct nvhost_vm_buffer {
	struct nvhost_vm *vm;

	/* buffer attachment */
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;

	/* context specific view to the buffer */
	dma_addr_t addr;
	size_t size;

	struct kref kref;	/* reference to this buffer */

	/* bookkeeping */
	unsigned int user_map_count;	/* application view to the buffer */
	unsigned int submit_map_count;	/* hw view to this buffer */
	struct rb_node node;

	/* used by hardware layer */
	void *private_data;
};

struct nvhost_vm_static_buffer {
	struct sg_table *sgt;

	void *vaddr;
	dma_addr_t paddr;
	size_t size;

	/* list of all statically mapped buffers */
	struct list_head list;
};

/**
 * nvhost_vm_get_id - get hw identifier of this vm
 *	@vm: Pointer to nvhost_vm structure
 *
 * This function returns hardware identifier of the given vm.
 */
int nvhost_vm_get_id(struct nvhost_vm *vm);

/**
 * nvhost_vm_map_static - map allocated area to iova
 *	@pdev: pointer to host1x or host1x client device
 *	@vaddr: kernel virtual address
 *	@paddr: desired physical address for this buffer
 *	@size: size of the buffer (in bytes)
 *
 * This call maps given area to all existing (and future) address spaces.
 * The mapping is permanent and cannot be removed. User of this API is
 * responsible to ensure that the backing memory is not released at any
 * point.
 *
 * Return 0 on succcess, error otherwise. Base address is returned
 * in address pointer.
 *
 */
int nvhost_vm_map_static(struct platform_device *pdev,
			 void *vaddr, dma_addr_t paddr,
			 size_t size);

/**
 * nvhost_vm_pin_buffers - Pin mapped buffers to the hardware
 *	@vm: Pointer to nvhost_vm structure
 *
 * This functions pins all currently mapped buffers (from the application view)
 * into device address space. Even if the user space tries to unmap these
 * buffers, the buffers remain valid for hardware until the function
 * nvhost_vm_unpin_buffers() has been called.
 *
 * Returns reference to the nvhost_vm_pin instance.
 *
 */
struct nvhost_vm_pin *nvhost_vm_pin_buffers(struct nvhost_vm *vm);

/**
 * nvhost_vm_unpin_buffers - unpins given set of buffers
 *	@vm: Pointer to nvhost_vm structure
 *	@pin: pointer to pin structure from nvhost_vm_pin_buffers()
 *
 * This call unpins the given set of buffers from the hardware. If there are
 * no remaining pins or maps from user space, this call will remove the buffers
 * from the vm structure.
 *
 * No return value
 *
 */
void nvhost_vm_unpin_buffers(struct nvhost_vm *vm, struct nvhost_vm_pin *pin);

/**
 * nvhost_vm_unmap_dmabuf - unmap dmabuf
 *	@vm: Pointer to nvhost_vm structure
 *	@dmabuf: pointer to dmabuf
 *
 * This call unmaps given buffer from vm. If reference counter of the mapping
 * goes to 0 and no hardware pins remain, the buffer is really unmapped and the
 * buffer reference to vm is dropped.
 *
 * No return value
 *
 */
void nvhost_vm_unmap_dmabuf(struct nvhost_vm *vm, struct dma_buf *dmabuf);

/**
 * nvhost_vm_map_dmabuf - map dmabuf to address spacae
 *	@vm: Pointer to nvhost_vm structure
 *	@dmabuf: pointer to dmabuf
 *	@addr: base address for mapping
 *
 * This call maps given area to address space.
 *
 * Each mapped buffer takes a reference on the vm. This ensures that the
 * domain cannot be dropped as long as there is a single mapped buffer.
 *
 * If same buffer (based on dmabuf pointer) is mapped multiple times, the
 * reference counter of the buffer is increased and the same virtual
 * address is returned.
 *
 * Return 0 on succcess, error otherwise. Base address is returned
 * in address pointer.
 *
 */
int nvhost_vm_map_dmabuf(struct nvhost_vm *vm, struct dma_buf *dmabuf,
			 dma_addr_t *addr);

/**
 * nvhost_vm_put - Drop reference to vm
 *	@vm: Pointer to nvhost_vm structure
 *
 * Drops reference to vm. When refcount goes to 0, vm resources are released.
 *
 * No return value
 */
void nvhost_vm_put(struct nvhost_vm *vm);

/**
 * nvhost_vm_get - Get reference to vm
 *	@vm: Pointer to nvhost_vm structure
 *
 * No return value
 */
void nvhost_vm_get(struct nvhost_vm *vm);

/**
 * nvhost_vm_allocate - Allocate vm to hold buffers
 *	@pdev: pointer to the host1x client device
 *
 * This function allocates IOMMU domain to hold buffers and makes
 * initializations to lists, mutexes, bitmaps, etc. to keep track of mappings.
 *
 * Returns pointer to nvhost_vm on success, 0 otherwise.
 */
struct nvhost_vm *nvhost_vm_allocate(struct platform_device *pdev);

#endif
