/* Copyright (c) 2011, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#ifdef CONFIG_ION_MSM
#include <linux/ion.h>
#endif
#include <linux/memory.h>
#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif
#include <asm/mach/map.h>
#include <asm/hardware/gic.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <mach/msm_iomap.h>
#ifdef CONFIG_ION_MSM
#include <mach/ion.h>
#endif
#include <mach/msm_memtypes.h>
#include "clock.h"

#define MSM_KERNEL_EBI1_MEM_SIZE	0x280000
#ifdef CONFIG_FB_MSM_HDMI_AS_PRIMARY
#define MSM_ION_SF_SIZE 0x4000000 /* 64 Mbytes */
#else
#define MSM_ION_SF_SIZE 0x2800000 /* 40 Mbytes */
#endif
#define MSM_ION_MM_FW_SIZE	0x200000 /* (2MB) */
#define MSM_ION_MM_SIZE		0x7800000 /* (120MB) */
#define MSM_ION_QSECOM_SIZE	0x100000 /* (1MB) */
#define MSM_ION_MFC_SIZE	SZ_8K
#define MSM_ION_AUDIO_SIZE	0x2B4000
#define MSM_ION_HEAP_NUM	8

#ifdef CONFIG_KERNEL_PMEM_EBI_REGION
static unsigned kernel_ebi1_mem_size = MSM_KERNEL_EBI1_MEM_SIZE;
static int __init kernel_ebi1_mem_size_setup(char *p)
{
	kernel_ebi1_mem_size = memparse(p, NULL);
	return 0;
}
early_param("kernel_ebi1_mem_size", kernel_ebi1_mem_size_setup);
#endif

static struct memtype_reserve msm_copper_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static int msm_copper_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

#ifdef CONFIG_ION_MSM
static struct ion_cp_heap_pdata cp_mm_ion_pdata = {
	.permission_type = IPT_TYPE_MM_CARVEOUT,
	.align = PAGE_SIZE,
};

static struct ion_cp_heap_pdata cp_mfc_ion_pdata = {
	.permission_type = IPT_TYPE_MFC_SHAREDMEM,
	.align = PAGE_SIZE,
};

static struct ion_co_heap_pdata co_ion_pdata = {
	.adjacent_mem_id = INVALID_HEAP_ID,
	.align = PAGE_SIZE,
};

static struct ion_co_heap_pdata fw_co_ion_pdata = {
	.adjacent_mem_id = ION_CP_MM_HEAP_ID,
	.align = SZ_128K,
};

/**
 * These heaps are listed in the order they will be allocated. Due to
 * video hardware restrictions and content protection the FW heap has to
 * be allocated adjacent (below) the MM heap and the MFC heap has to be
 * allocated after the MM heap to ensure MFC heap is not more than 256MB
 * away from the base address of the FW heap.
 * However, the order of FW heap and MM heap doesn't matter since these
 * two heaps are taken care of by separate code to ensure they are adjacent
 * to each other.
 * Don't swap the order unless you know what you are doing!
 */
static struct ion_platform_data ion_pdata = {
	.nr = MSM_ION_HEAP_NUM,
	.heaps = {
		{
			.id	= ION_SYSTEM_HEAP_ID,
			.type	= ION_HEAP_TYPE_SYSTEM,
			.name	= ION_VMALLOC_HEAP_NAME,
		},
		{
			.id	= ION_CP_MM_HEAP_ID,
			.type	= ION_HEAP_TYPE_CP,
			.name	= ION_MM_HEAP_NAME,
			.size	= MSM_ION_MM_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &cp_mm_ion_pdata,
		},
		{
			.id	= ION_MM_FIRMWARE_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_MM_FIRMWARE_HEAP_NAME,
			.size	= MSM_ION_MM_FW_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &fw_co_ion_pdata,
		},
		{
			.id	= ION_CP_MFC_HEAP_ID,
			.type	= ION_HEAP_TYPE_CP,
			.name	= ION_MFC_HEAP_NAME,
			.size	= MSM_ION_MFC_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &cp_mfc_ion_pdata,
		},
		{
			.id	= ION_SF_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_SF_HEAP_NAME,
			.size	= MSM_ION_SF_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_ion_pdata,
		},
		{
			.id	= ION_IOMMU_HEAP_ID,
			.type	= ION_HEAP_TYPE_IOMMU,
			.name	= ION_IOMMU_HEAP_NAME,
		},
		{
			.id	= ION_QSECOM_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_QSECOM_HEAP_NAME,
			.size	= MSM_ION_QSECOM_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_ion_pdata,
		},
		{
			.id	= ION_AUDIO_HEAP_ID,
			.type	= ION_HEAP_TYPE_CARVEOUT,
			.name	= ION_AUDIO_HEAP_NAME,
			.size	= MSM_ION_AUDIO_SIZE,
			.memory_type = ION_EBI_TYPE,
			.extra_data = (void *) &co_ion_pdata,
		},
	}
};

static struct platform_device ion_dev = {
	.name = "ion-msm",
	.id = 1,
	.dev = { .platform_data = &ion_pdata },
};

static void reserve_ion_memory(void)
{
	msm_copper_reserve_table[MEMTYPE_EBI1].size += MSM_ION_MM_SIZE;
	msm_copper_reserve_table[MEMTYPE_EBI1].size += MSM_ION_MM_FW_SIZE;
	msm_copper_reserve_table[MEMTYPE_EBI1].size += MSM_ION_SF_SIZE;
	msm_copper_reserve_table[MEMTYPE_EBI1].size += MSM_ION_MFC_SIZE;
	msm_copper_reserve_table[MEMTYPE_EBI1].size += MSM_ION_QSECOM_SIZE;
	msm_copper_reserve_table[MEMTYPE_EBI1].size += MSM_ION_AUDIO_SIZE;
#ifdef CONFIG_KERNEL_PMEM_EBI_REGION
	msm_copper_reserve_table[MEMTYPE_EBI1].size += kernel_ebi1_mem_size;
#endif
}
#endif

static void __init msm_copper_calculate_reserve_sizes(void)
{
#ifdef CONFIG_ION_MSM
	reserve_ion_memory();
#endif
}

static struct reserve_info msm_copper_reserve_info __initdata = {
	.memtype_reserve_table = msm_copper_reserve_table,
	.calculate_reserve_sizes = msm_copper_calculate_reserve_sizes,
	.paddr_to_memtype = msm_copper_paddr_to_memtype,
};

static void __init msm_copper_early_memory(void)
{
	reserve_info = &msm_copper_reserve_info;
}

void __init msm_copper_reserve(void)
{
	msm_reserve();
}

static int __init gpiomux_init(void)
{
	int rc;

	rc = msm_gpiomux_init(NR_GPIO_IRQS);
	if (rc) {
		pr_err("%s: msm_gpiomux_init failed %d\n", __func__, rc);
		return rc;
	}

	return 0;
}

void __init msm_copper_add_devices(void)
{
#ifdef CONFIG_ION_MSM
	platform_device_register(&ion_dev);
#endif
}

static struct of_device_id msm_copper_gic_match[] __initdata = {
	{ .compatible = "qcom,msm-qgic2", },
	{}
};

void __init msm_copper_init_irq(void)
{
	gic_init(0, GIC_PPI_START, MSM_QGIC_DIST_BASE,
			(void *)MSM_QGIC_CPU_BASE);

	irq_domain_generate_simple(msm_copper_gic_match,
		COPPER_QGIC_DIST_PHYS, GIC_SPI_START);
}

static struct clk_lookup msm_clocks_dummy[] = {
	CLK_DUMMY("core_clk",	BLSP2_UART_CLK,	"msm_serial_hsl.0",	OFF),
	CLK_DUMMY("iface_clk",	BLSP2_UART_CLK,	"msm_serial_hsl.0",	OFF),
	CLK_DUMMY("core_clk",	SDC1_CLK,	NULL,			OFF),
	CLK_DUMMY("iface_clk",	SDC1_P_CLK,	NULL,			OFF),
	CLK_DUMMY("core_clk",	SDC3_CLK,	NULL,			OFF),
	CLK_DUMMY("iface_clk",	SDC3_P_CLK,	NULL,			OFF),
	CLK_DUMMY("phy_clk", NULL, "msm_otg", OFF),
	CLK_DUMMY("core_clk", NULL, "msm_otg", OFF),
	CLK_DUMMY("alt_core_clk", NULL, "msm_otg", OFF),
	CLK_DUMMY("iface_clk", NULL, "msm_otg", OFF),
	CLK_DUMMY("dfab_clk",	DFAB_CLK,	NULL, 0),
	CLK_DUMMY("dma_bam_pclk",	DMA_BAM_P_CLK,	NULL, 0),
	CLK_DUMMY("mem_clk",	NULL,	NULL, 0),
};

struct clock_init_data msm_dummy_clock_init_data __initdata = {
	.table = msm_clocks_dummy,
	.size = ARRAY_SIZE(msm_clocks_dummy),
};

static struct of_dev_auxdata msm_copper_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("qcom,msm-lsuart-v14", 0xF991F000, \
			"msm_serial_hsl.0", NULL),
	OF_DEV_AUXDATA("qcom,hsusb-otg", 0xF9A55000, \
			"msm_otg", NULL),
	{}
};

void __init msm_copper_init(struct of_dev_auxdata **adata)
{
	if (gpiomux_init())
		pr_err("%s: gpiomux_init() failed\n", __func__);
	msm_clock_init(&msm_dummy_clock_init_data);

	*adata = msm_copper_auxdata_lookup;
}

void __init msm_copper_very_early(void)
{
	msm_copper_early_memory();
}
