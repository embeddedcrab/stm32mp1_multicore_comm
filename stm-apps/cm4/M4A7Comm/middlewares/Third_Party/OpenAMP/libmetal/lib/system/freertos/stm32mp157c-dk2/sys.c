/*
 * Copyright (c) 2018, Linaro Inc. and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * @file	generic/template/sys.c
 * @brief	machine specific system primitives implementation.
 */

#include <metal/io.h>
#include <metal/sys.h>
#include <metal/utilities.h>

#include <metal/device.h>
#include <stdint.h>


#include "openamp_conf.h"


#define BUS_NAME        "generic"
#define SHM_DEV_NAME    "3ed80000.shm"
#define DEFAULT_PAGE_SHIFT (-1UL)
#define DEFAULT_PAGE_MASK  (-1UL)


const metal_phys_addr_t metal_phys[] = {
	SHM_START_ADDRESS /**< shared memory base address */
};


/* Define metal devices table for IPI, shared memory and TTC devices.
 * Linux system uses device tree to describe devices. Unlike Linux,
 * there is no standard device abstraction for FreeRTOS system, we
 * uses libmetal devices structure to describe the devices we used in
 * the example.
 * The IPI, shared memory and TTC devices are memory mapped
 * devices. For this type of devices, it is required to provide
 * accessible memory mapped regions, and interrupt information.
 * In FreeRTOS system, the memory mapping is flat. As you can see
 * in the table before, we set the virtual address "virt" the same
 * as the physical address.
 */
static struct metal_device metal_dev_table[] = {
	{
		/* Shared memory management device */
		.name = SHM_DEV_NAME,
		.bus = NULL,
		.num_regions = 1,
		.regions = {
			{
				.virt = (void *)SHM_START_ADDRESS,
				.physmap = &metal_phys[0],
				.size = 0x1000000,
				.page_shift = DEFAULT_PAGE_SHIFT,
				.page_mask = DEFAULT_PAGE_MASK,
				.mem_flags = 0,
				.ops = {NULL},
			}
		},
		.node = {NULL},
		.irq_num = 0,
		.irq_info = NULL,
	}
};


/**
 * Extern global variables
 */
struct metal_device *shm_dev = NULL;



/**
 * @brief platform_register_metal_device() - Statically Register libmetal
 *        devices.
 *        This function registers the IPI, shared memory and
 *        TTC devices to the libmetal generic bus.
 *        Libmetal uses bus structure to group the devices. Before you can
 *        access the device with libmetal device operation, you will need to
 *        register the device to a libmetal supported bus.
 *        For non-Linux system, libmetal only supports "generic" bus, which is
 *        used to manage the memory mapped devices.
 *
 * @return 0 - succeeded, non-zero for failures.
 */
int platform_register_metal_device(void)
{
	unsigned int i;
	int ret;
	struct metal_device *dev;

	for (i = 0; i < 1/*sizeof(metal_dev_table)/sizeof(struct metal_device)*/; i++)
	{
		dev = &metal_dev_table[i];
		ret = metal_register_generic_device(dev);
		if (ret)
			return ret;
	}
	return 0;
}


/**
 * @brief open_metal_devices() - Open registered libmetal devices.
 *        This function opens all the registered libmetal devices.
 *
 * @return 0 - succeeded, non-zero for failures.
 */
int open_metal_devices(void)
{
	int ret;

	/* Open shared memory device */
	ret = metal_device_open(BUS_NAME, SHM_DEV_NAME, &shm_dev);
	if (ret) {
		goto out;
	}

out:
	return ret;
}


/**
 * @brief close_metal_devices() - close libmetal devices
 *        This function closes all the libmetal devices which have
 *        been opened.
 *
 */
void close_metal_devices(void)
{
	/* Close shared memory device */
	if (shm_dev)
		metal_device_close(shm_dev);
}


void sys_irq_restore_enable(unsigned int flags)
{
	metal_unused(flags);
	/* Add implementation here */
}

unsigned int sys_irq_save_disable(void)
{
	return 0;
	/* Add implementation here */
}

void sys_irq_enable(unsigned int vector)
{
	metal_unused(vector);

	/* Add implementation here */
}

void sys_irq_disable(unsigned int vector)
{
	metal_unused(vector);

	/* Add implementation here */
}

void metal_machine_cache_flush(void *addr, unsigned int len)
{
	metal_unused(addr);
	metal_unused(len);

	/* Add implementation here */
}

void metal_machine_cache_invalidate(void *addr, unsigned int len)
{
	metal_unused(addr);
	metal_unused(len);

	/* Add implementation here */
}

void metal_generic_default_poll(void)
{
	metal_asm volatile("wfi");
}

void *metal_machine_io_mem_map(void *va, metal_phys_addr_t pa,
			       size_t size, unsigned int flags)
{
	metal_unused(pa);
	metal_unused(size);
	metal_unused(flags);

	/* Add implementation here */

	return va;
}
