
/**
 * @file mali_platform.c
 * Platform specific Mali driver functions for:
 * - Realview Versatile platforms with ARM11 Mpcore and virtex 5.
 * - Versatile Express platforms with ARM Cortex-A9 and virtex 6.
 */
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/pm.h>
#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif
#include <asm/io.h>
#include <linux/mali/mali_utgard.h>
#include "mali_kernel_common.h"
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>

#include "arm_core_scaling.h"
#include "mali_executor.h"
#include "platform_pmm.h"
#include "mach/mt_reg_base.h"

static int mali_core_scaling_enable = 0;
extern unsigned int current_sample_utilization;

void mali_gpu_utilization_callback(struct mali_gpu_utilization_data *data);
#if defined(CONFIG_MALI_DVFS)
int  mali_gpu_set_freq(int setting_clock_step);
void mali_gpu_get_clock_info(struct mali_gpu_clock **data);
int  mali_gpu_get_freq(void);
#endif

#if defined(CONFIG_ARCH_REALVIEW)
static u32 mali_read_phys(u32 phys_addr);
static void mali_write_phys(u32 phys_addr, u32 value);
#endif

#ifndef CONFIG_MALI_DT
static void mali_platform_device_release(struct device *device);

#if defined(CONFIG_ARCH_VEXPRESS)

#if defined(CONFIG_ARM64)
/* Juno + Mali-450 MP6 in V7 FPGA */
static struct resource mali_gpu_resources_m450_mp6[] = {
	MALI_GPU_RESOURCES_MALI450_MP6_PMU(0x6F040000, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200)
};

#else
static struct resource mali_gpu_resources_m450_mp8[] = {
	MALI_GPU_RESOURCES_MALI450_MP8_PMU(0xFC040000, -1, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 68)
};

static struct resource mali_gpu_resources_m450_mp6[] = {
	MALI_GPU_RESOURCES_MALI450_MP6_PMU(0xFC040000, -1, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 70, 68)
};

static struct resource mali_gpu_resources_m450_mp4[] = {
	MALI_GPU_RESOURCES_MALI450_MP4_PMU(0xFC040000, -1, 70, 70, 70, 70, 70, 70, 70, 70, 70, 68)
};
#endif /* CONFIG_ARM64 */

#elif defined(CONFIG_ARCH_REALVIEW)

static struct resource mali_gpu_resources_m300[] = {
	MALI_GPU_RESOURCES_MALI300_PMU(0xC0000000, -1, -1, -1, -1)
};

static struct resource mali_gpu_resources_m400_mp1[] = {
	MALI_GPU_RESOURCES_MALI400_MP1_PMU(0xC0000000, -1, -1, -1, -1)
};

static struct resource mali_gpu_resources_m400_mp2[] = {
	MALI_GPU_RESOURCES_MALI400_MP2_PMU(0xC0000000, -1, -1, -1, -1, -1, -1)
};

#endif
#endif

static struct mali_gpu_device_data mali_gpu_data = {
#ifndef CONFIG_MALI_DT
	.pmu_switch_delay = 0xFF, /* do not have to be this high on FPGA, but it is good for testing to have a delay */
	.max_job_runtime = 60000, /* 60 seconds */
#if defined(CONFIG_ARCH_VEXPRESS)
	.shared_mem_size = 1024 * 1024 * 1024,	/* 1GB */
#endif
#endif

#if defined(CONFIG_ARCH_REALVIEW)
	.dedicated_mem_start = 0x80000000, /* Physical start address (use 0xD0000000 for old indirect setup) */
	.dedicated_mem_size = 0x10000000, /* 256MB */
#endif
#if defined(CONFIG_ARM64)
	.fb_start = 0x5f000000,
	.fb_size = 0x91000000,
#else
	.fb_start = 0x80000000,
	.fb_size = 0x80000000,
#endif
	.control_interval = 200, /* 200ms */
	.utilization_callback = mali_gpu_utilization_callback,
#if defined(CONFIG_MALI_DVFS)
	.get_clock_info = mali_gpu_get_clock_info,
	.get_freq = mali_gpu_get_freq,
	.set_freq = mali_gpu_set_freq,
#else
	.get_clock_info = NULL,
	.get_freq = NULL,
	.set_freq = NULL,
#endif
};

#ifndef CONFIG_MALI_DT
static struct resource mali_gpu_resources[] =
{
    MALI_GPU_RESOURCES_MALI450_MP4(
                    IO_VIRT_TO_PHYS(MALI_BASE),
                    MT_MFG_IRQ0_ID,
                    MT_MFG_IRQ1_ID,
                    MT_MFG_IRQ2_ID,
                    MT_MFG_IRQ3_ID,
                    MT_MFG_IRQ4_ID,
                    MT_MFG_IRQ5_ID,
                    MT_MFG_IRQ6_ID,
                    MT_MFG_IRQ7_ID,
                    MT_MFG_IRQ8_ID,
                    MT_MFG_IRQ9_ID,
                    MT_MFG_IRQ10_ID
                )
};

static struct resource mali_gpu_resources_MP3[] =
{
    MALI_GPU_RESOURCES_MALI450_MP3(
                    IO_VIRT_TO_PHYS(MALI_BASE),
                    MT_MFG_IRQ0_ID,
                    MT_MFG_IRQ1_ID,
                    MT_MFG_IRQ2_ID,
                    MT_MFG_IRQ3_ID,
                    MT_MFG_IRQ4_ID,
                    MT_MFG_IRQ5_ID,
                    MT_MFG_IRQ6_ID,
                    MT_MFG_IRQ7_ID,                   
                    MT_MFG_IRQ10_ID
                )
};

static struct resource mali_gpu_resources_MP2[] =
{
    MALI_GPU_RESOURCES_MALI450_MP2(
                    IO_VIRT_TO_PHYS(MALI_BASE),
                    MT_MFG_IRQ0_ID,
                    MT_MFG_IRQ1_ID,
                    MT_MFG_IRQ2_ID,
                    MT_MFG_IRQ3_ID,
                    MT_MFG_IRQ4_ID,
                    MT_MFG_IRQ5_ID,
                    MT_MFG_IRQ10_ID
                )
};

static struct platform_device mali_gpu_device = {
	.name = MALI_GPU_NAME_UTGARD,
	.id = 0,
    .num_resources = ARRAY_SIZE(mali_gpu_resources),
    .resource = (struct resource *)&mali_gpu_resources,
    .dev.platform_data = &mali_gpu_data,
	.dev.release = mali_platform_device_release,
	.dev.coherent_dma_mask = DMA_BIT_MASK(32),
	.dev.dma_mask = &mali_gpu_device.dev.coherent_dma_mask,

#if defined(CONFIG_ARM64)
	.dev.archdata.dma_ops = &noncoherent_swiotlb_dma_ops,
#endif
};

static u32 get_devinfo() {
	/*TODO: replace this with get_devinfo_with_index*/
    return *(volatile u32 *)0xf0206174;
}
static u32 get_gpuinfo() {
	/*TODO: replace this with get_devinfo_with_index*/
    return *(volatile u32 *)0xf0206040;
}

#define MALI_REASSIGN_RESOURCE(device, X) \
do {\
	device->resource = (struct resource *)&(X);\
	device->num_resources = ARRAY_SIZE((X));\
}while(0)

static void update_dev_info(struct platform_device * device ) {
    u32 info = get_devinfo();
	MALI_DEBUG_PRINT(1, ("devinfo %#x\n", info));

    /*if(0x0 == (info & (0x1 << 31))) { t or b*/
    /*T*/
    u32 gpuinfo = get_gpuinfo();
	MALI_DEBUG_PRINT(1, ("gpuinfo %#x\n", gpuinfo));
    u32 pp = (gpuinfo & 0x60000) >> 17;
    if(pp == 0x1) {
		MALI_DEBUG_PRINT(1, ("Found devinfo of MP3 %s\n", __FUNCTION__));
		MALI_REASSIGN_RESOURCE(device, mali_gpu_resources_MP3);
    } else if(pp == 0x2 || pp == 0x3) {
 		MALI_DEBUG_PRINT(1, ("Found devinfo of MP2 %s, %d\n", __FUNCTION__, pp));
		MALI_REASSIGN_RESOURCE(device, mali_gpu_resources_MP2);
    } else {
#ifdef MTK_NR_MALI_PP
#if (MTK_NR_MALI_PP == 3)
        MALI_DEBUG_PRINT(1, ("Mali MP3 %s (MTK_NR_MALI_PP)\n", __FUNCTION__));
        MALI_REASSIGN_RESOURCE(device, mali_gpu_resources_MP3);            
#elif (MTK_NR_MALI_PP == 2)
        MALI_DEBUG_PRINT(1, ("Mali MP2 %s (MTK_NR_MALI_PP)\n", __FUNCTION__));
        MALI_REASSIGN_RESOURCE(device, mali_gpu_resources_MP2);            
#else
		MALI_DEBUG_PRINT(1, ("Default MP4 %s, ignore cfg: %d\n", __FUNCTION__, MTK_NR_MALI_PP));
#endif
#else 
        MALI_DEBUG_PRINT(1, ("Default MP4 %s\n", __FUNCTION__));
#endif
	}
}

extern unsigned int get_max_DRAM_size (void);
int mali_platform_device_register(void)
{
    int num_pp_cores = 4; //TODO: Need specify if we are using diff config
    int err = -1;
    MALI_DEBUG_PRINT(1, ("%s\n", __FUNCTION__));
    mali_gpu_data.shared_mem_size = get_max_DRAM_size();

    update_dev_info(&mali_gpu_device);
    
    err = platform_device_register(&mali_gpu_device);
	            
    if (0 == err) 
    {
        mali_pmm_init(&mali_gpu_device);
        mali_core_scaling_init(num_pp_cores);

        return 0;
    }

    MALI_DEBUG_PRINT(1, ("%s err=%d\n",__FUNCTION__, err));

    platform_device_unregister(&mali_gpu_device);

    return err;
}

void mali_platform_device_unregister(void)
{
	MALI_DEBUG_PRINT(4, ("mali_platform_device_unregister() called\n"));

	mali_core_scaling_term();
	platform_device_unregister(&mali_gpu_device);

	platform_device_put(&mali_gpu_device);

#if defined(CONFIG_ARCH_REALVIEW)
	mali_write_phys(0xC0010020, 0x9); /* Restore default (legacy) memory mapping */
#endif
}

static void mali_platform_device_release(struct device *device)
{
	MALI_DEBUG_PRINT(4, ("mali_platform_device_release() called\n"));
}

#else /* CONFIG_MALI_DT */

static int mali_pm_suspend(struct device *device)
{
	int ret = 0;

	MALI_DEBUG_PRINT(3, ("Mali PM:%s\n", __func__));

	if (NULL != device->driver && NULL != device->driver->pm
	    && NULL != device->driver->pm->suspend) {
		/* Need to notify Mali driver about this event */
		ret = device->driver->pm->suspend(device);
	}

	/* _mali_osk_pm_delete_callback_timer();*/
	mali_platform_power_mode_change(device, MALI_POWER_MODE_DEEP_SLEEP);

	return ret;
}

static int mali_pm_resume(struct device *device)
{
	int ret = 0;

	MALI_DEBUG_PRINT(3, ("Mali PM: %s\n", __func__));

	mali_platform_power_mode_change(device, MALI_POWER_MODE_ON);

	if (NULL != device->driver && NULL != device->driver->pm
	    && NULL != device->driver->pm->resume) {
		/* Need to notify Mali driver about this event */
		ret = device->driver->pm->resume(device);
	}

	return ret;
}

#ifdef CONFIG_PM_RUNTIME
static int mali_runtime_suspend(struct device *device)
{
    int ret = 0;

	MALI_DEBUG_PRINT(4, ("mali_runtime_suspend() called\n"));

	if (NULL != device->driver && NULL != device->driver->pm
	    && NULL != device->driver->pm->runtime_suspend) {
		/* Need to notify Mali driver about this event */
		ret = device->driver->pm->runtime_suspend(device);
	}

	mali_platform_power_mode_change(device, MALI_POWER_MODE_LIGHT_SLEEP);

	return ret;
}

static int mali_runtime_resume(struct device *device)
{
	int ret = 0;

	MALI_DEBUG_PRINT(4, ("mali_runtime_resume() called\n"));

	mali_platform_power_mode_change(device, MALI_POWER_MODE_ON);

	if (NULL != device->driver && NULL != device->driver->pm
	    && NULL != device->driver->pm->runtime_resume) {
		/* Need to notify Mali driver about this event */
		ret = device->driver->pm->runtime_resume(device);
	}

	return ret;
}

static int mali_runtime_idle(struct device *device)
{
	MALI_DEBUG_PRINT(4, ("mali_runtime_idle() called\n"));

	if (NULL != device->driver && NULL != device->driver->pm
	    && NULL != device->driver->pm->runtime_idle) {
		/* Need to notify Mali driver about this event */
		int ret = device->driver->pm->runtime_idle(device);
		if (0 != ret)
			return ret;
	}

	pm_runtime_suspend(device);

	return 0;
}
#endif /* CONFIG_PM_RUNTIME */

static const struct dev_pm_ops mali_gpu_device_type_pm_ops = {
	.suspend = mali_pm_suspend,
	.resume = mali_pm_resume,
	.freeze = mali_pm_suspend,
	.thaw = mali_pm_resume,
	.restore = mali_pm_resume,

#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend = mali_runtime_suspend,
	.runtime_resume = mali_runtime_resume,
	.runtime_idle = mali_runtime_idle,
#endif
};

static struct device_type mali_gpu_device_device_type = {
	.pm = &mali_gpu_device_type_pm_ops,
};

int mali_platform_device_init(struct platform_device *device)
{
	int num_pp_cores = 4;
	int err = -1;
#if defined(CONFIG_ARCH_REALVIEW)
	u32 m400_gp_version;
#endif

	/* Detect present Mali GPU and connect the correct resources to the device */
#if defined(CONFIG_ARCH_VEXPRESS)

#if defined(CONFIG_ARM64)
	if (mali_read_phys(0x6F000000) == 0x40601450) {
		MALI_DEBUG_PRINT(4, ("Registering Mali-450 MP6 device\n"));
		num_pp_cores = 6;
	}
#else
	if (mali_read_phys(0xFC000000) == 0x00000450) {
		MALI_DEBUG_PRINT(4, ("Registering Mali-450 MP8 device\n"));
		num_pp_cores = 8;
	} else if (mali_read_phys(0xFC000000) == 0x40400450) {
		MALI_DEBUG_PRINT(4, ("Registering Mali-450 MP4 device\n"));
		num_pp_cores = 4;
	}
#endif

#elif defined(CONFIG_ARCH_REALVIEW)

	m400_gp_version = mali_read_phys(0xC000006C);
	if ((m400_gp_version & 0xFFFF0000) == 0x0C070000) {
		MALI_DEBUG_PRINT(4, ("Registering Mali-300 device\n"));
		num_pp_cores = 1;
		mali_write_phys(0xC0010020, 0xA); /* Enable direct memory mapping for FPGA */
	} else if ((m400_gp_version & 0xFFFF0000) == 0x0B070000) {
		u32 fpga_fw_version = mali_read_phys(0xC0010000);
		if (fpga_fw_version == 0x130C008F || fpga_fw_version == 0x110C008F) {
			/* Mali-400 MP1 r1p0 or r1p1 */
			MALI_DEBUG_PRINT(4, ("Registering Mali-400 MP1 device\n"));
			num_pp_cores = 1;
			mali_write_phys(0xC0010020, 0xA); /* Enable direct memory mapping for FPGA */
		} else if (fpga_fw_version == 0x130C000F) {
			/* Mali-400 MP2 r1p1 */
			MALI_DEBUG_PRINT(4, ("Registering Mali-400 MP2 device\n"));
			num_pp_cores = 2;
			mali_write_phys(0xC0010020, 0xA); /* Enable direct memory mapping for FPGA */
		}
	}
#endif


	if (mali_pmm_init(device))
		return err;
	
	device->dev.type = &mali_gpu_device_device_type;
	
	err = platform_device_add_data(device, &mali_gpu_data, sizeof(mali_gpu_data));

	if (0 == err) {
		
#ifdef CONFIG_PM_RUNTIME
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
		pm_runtime_set_autosuspend_delay(&(device->dev), 2000);
		pm_runtime_use_autosuspend(&(device->dev));
#endif
		pm_runtime_enable(&(device->dev));
#endif
		MALI_DEBUG_ASSERT(0 < num_pp_cores);
		mali_core_scaling_init(num_pp_cores);
	}

	return err;
}

int mali_platform_device_deinit(struct platform_device *device)
{
	/*MALI_IGNORE(device);*/

	MALI_DEBUG_PRINT(4, ("mali_platform_device_deinit() called\n"));

	mali_core_scaling_term();

	mali_pmm_deinit(device);

#if defined(CONFIG_ARCH_REALVIEW)
	mali_write_phys(0xC0010020, 0x9); /* Restore default (legacy) memory mapping */
#endif

	return 0;
}

#endif /* CONFIG_MALI_DT */

#if defined(CONFIG_ARCH_REALVIEW)
static u32 mali_read_phys(u32 phys_addr)
{
	u32 phys_addr_page = phys_addr & 0xFFFFE000;
	u32 phys_offset    = phys_addr & 0x00001FFF;
	u32 map_size       = phys_offset + sizeof(u32);
	u32 ret = 0xDEADBEEF;
	void *mem_mapped = ioremap_nocache(phys_addr_page, map_size);
	if (NULL != mem_mapped) {
		ret = (u32)ioread32(((u8 *)mem_mapped) + phys_offset);
		iounmap(mem_mapped);
	}

	return ret;
}

static void mali_write_phys(u32 phys_addr, u32 value)
{
	u32 phys_addr_page = phys_addr & 0xFFFFE000;
	u32 phys_offset    = phys_addr & 0x00001FFF;
	u32 map_size       = phys_offset + sizeof(u32);
	void *mem_mapped = ioremap_nocache(phys_addr_page, map_size);
	if (NULL != mem_mapped) {
		iowrite32(value, ((u8 *)mem_mapped) + phys_offset);
		iounmap(mem_mapped);
	}
}
#endif

static int param_set_core_scaling(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_int(val, kp);

	if (1 == mali_core_scaling_enable) {
		mali_core_scaling_sync(mali_executor_get_num_cores_enabled());
	}
	return ret;
}

static struct kernel_param_ops param_ops_core_scaling = {
	.set = param_set_core_scaling,
	.get = param_get_int,
};

module_param_cb(mali_core_scaling_enable, &param_ops_core_scaling, &mali_core_scaling_enable, 0644);
MODULE_PARM_DESC(mali_core_scaling_enable, "1 means to enable core scaling policy, 0 means to disable core scaling policy");

void mali_gpu_utilization_callback(struct mali_gpu_utilization_data *data)
{
	if (1 == mali_core_scaling_enable) {
		mali_core_scaling_update(data);
	}
	current_sample_utilization = (unsigned int)data->utilization_gpu;
}

#if defined(CONFIG_MALI_DVFS)
int  mali_gpu_set_freq(int setting_clock_step)
{
	MALI_DEBUG_PRINT(1, ("mali_gpu_set_freq : incomplete\n"));
}

int  mali_gpu_get_freq(void)
{
	/* return clock_step */
	MALI_DEBUG_PRINT(1, ("mali_gpu_get_freq : incomplete\n"));

	return 0;
}

static struct mali_gpu_clk_item clk_item[] = { {455,1} };
static struct mali_gpu_clock mali_clock_info =
{
	.item = &clk_item[0],
	.num_of_steps = 1,
};

void mali_gpu_get_clock_info(struct mali_gpu_clock **data)
{
	MALI_DEBUG_PRINT(1, ("mali_gpu_set_freq : incomplete\n"));
	*data = &mali_clock_info;
}
#endif
