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
#include <mach/mt_irq.h>
#include "arm_core_scaling.h"
#include "platform_pmm.h"
#include "mali_pm.h"
#include "mali_osk.h"
#include "mt_reg_base.h"

static void mali_platform_device_release(struct device *device);
static int mali_pm_suspend(struct device *device);
static int mali_pm_resume(struct device *device);


static struct mali_gpu_device_data mali_gpu_data =
{
    // System memory
    .shared_mem_size = 1024 * 1024 * 1024, /* 1GB */
    // Framebuffer physical address, only for validation usage
    .fb_start = 0x80000000,
    .fb_size  = 0x80000000,
    // DVFS
    .utilization_interval = 200, /* ms */
    .utilization_callback = mali_pmm_utilization_handler, /*<utilization function>,*/
};

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


static struct dev_pm_ops mali_gpu_device_type_pm_ops =
{
    .suspend = mali_pm_suspend,
    .resume  = mali_pm_resume, 
    .freeze  = mali_pm_suspend, 
    .thaw    = mali_pm_resume,   
	.restore = mali_pm_resume,
	
#ifdef CONFIG_PM_RUNTIME
	.runtime_suspend = mali_runtime_suspend,
	.runtime_resume  = mali_runtime_resume,
	.runtime_idle   = mali_runtime_idle,
#endif
};

static struct device_type mali_gpu_device_device_type =
{
	.pm = &mali_gpu_device_type_pm_ops,
};


static struct platform_device mali_gpu_device =
{
   .name = MALI_GPU_NAME_UTGARD,
   .id = 0,
   .num_resources = ARRAY_SIZE(mali_gpu_resources),
   .resource = (struct resource *)&mali_gpu_resources,
   .dev.platform_data = &mali_gpu_data,
   .dev.release = mali_platform_device_release,
	.dev.coherent_dma_mask = DMA_BIT_MASK(32),    
	 /// Ideally .dev.pm_domain should be used instead, as this is the new framework designed
	 /// to control the power of devices.	 
	.dev.type = &mali_gpu_device_device_type /// We should probably use the pm_domain instead of type on newer kernels
};


extern u32 get_devinfo_with_index(u32 index);

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
    int err = -1;
    int num_pp_cores = 4; //TODO: Need specify if we are using diff config
    MALI_DEBUG_PRINT(1, ("%s\n", __FUNCTION__));
    mali_gpu_data.shared_mem_size = get_max_DRAM_size();

    update_dev_info(&mali_gpu_device);
    
    err = platform_device_register(&mali_gpu_device);
	            
    if (0 == err) 
    {         
        mali_pmm_init();
        
#ifdef CONFIG_PM_RUNTIME
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
				pm_runtime_set_autosuspend_delay(&(mali_gpu_device.dev), 1000);
				pm_runtime_use_autosuspend(&(mali_gpu_device.dev));
#endif
				pm_runtime_enable(&(mali_gpu_device.dev));
#endif

#if defined(__MALI_CORE_SCALING_ENABLE__)
        mali_core_scaling_init(num_pp_cores);
#endif        
        return 0;
    }

    MALI_DEBUG_PRINT(1, ("%s err=%d\n",__FUNCTION__, err));

    platform_device_unregister(&mali_gpu_device);

    return err;
}

void mali_platform_device_unregister(void)
{
    MALI_DEBUG_PRINT(1, ("%s\n", __FUNCTION__));    
    
#if defined(__MALI_CORE_SCALING_ENABLE__)    
    mali_core_scaling_term();
#endif
    
    mali_pmm_deinit();
 
    platform_device_unregister(&mali_gpu_device);
}

static void mali_platform_device_release(struct device *device)
{
    MALI_DEBUG_PRINT(1, ("%s\n", __FUNCTION__));
}

static int mali_pm_suspend(struct device *device)
{
    int ret = 0;

    MALI_DEBUG_PRINT(1, ("Mali PM:%s\n", __FUNCTION__));

    if (NULL != device->driver &&
        NULL != device->driver->pm &&
        NULL != device->driver->pm->suspend)
    {
        /* Need to notify Mali driver about this event */
        ret = device->driver->pm->suspend(device);
    }
   
    _mali_osk_pm_delete_callback_timer();
    mali_platform_power_mode_change(MALI_POWER_MODE_DEEP_SLEEP);

    return ret;
}

static int mali_pm_resume(struct device *device)
{
    int ret = 0;

    MALI_DEBUG_PRINT(1, ("Mali PM: %s\n", __FUNCTION__));

    mali_platform_power_mode_change(MALI_POWER_MODE_ON);

    if (NULL != device->driver &&
        NULL != device->driver->pm &&
        NULL != device->driver->pm->resume)
    {
        /* Need to notify Mali driver about this event */
        ret = device->driver->pm->resume(device);
    }

    return ret;
}



#if 0//because not used
static int mali_pm_freeze(struct device *device)
{
    int ret = 0;
    
    MALI_DEBUG_PRINT(1, ("Mali PM: %s\n", __FUNCTION__));
    
    if (NULL != device->driver &&
        NULL != device->driver->pm &&
        NULL != device->driver->pm->freeze)
    {
        /* Need to notify Mali driver about this event */
        ret = device->driver->pm->freeze(device);
    }

    return ret;
}

static int mali_pm_thaw(struct device *device)
{
    int ret = 0;

    MALI_DEBUG_PRINT(1, ("Mali PM: %s\n", __FUNCTION__));

    if (NULL != device->driver &&
        NULL != device->driver->pm &&
        NULL != device->driver->pm->thaw)
    {
        /* Need to notify Mali driver about this event */
        ret = device->driver->pm->thaw(device);
    }

    return ret;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int mali_runtime_suspend(struct device *device)
{
	int ret = 0;

	MALI_DEBUG_PRINT(4, ("mali_runtime_suspend() called\n"));

	if (NULL != device->driver &&
	    NULL != device->driver->pm &&
	    NULL != device->driver->pm->runtime_suspend)
	{
		/* Need to notify Mali driver about this event */
		ret = device->driver->pm->runtime_suspend(device);
	}

	mali_platform_power_mode_change(MALI_POWER_MODE_LIGHT_SLEEP);

	return ret;
}

static int mali_runtime_resume(struct device *device)
{
	int ret = 0;

	MALI_DEBUG_PRINT(4, ("mali_runtime_resume() called\n"));

	mali_platform_power_mode_change(MALI_POWER_MODE_ON);

	if (NULL != device->driver &&
	    NULL != device->driver->pm &&
	    NULL != device->driver->pm->runtime_resume)
	{
		/* Need to notify Mali driver about this event */
		ret = device->driver->pm->runtime_resume(device);
	}

	return ret;
}

static int mali_runtime_idle(struct device *device)
{
	MALI_DEBUG_PRINT(4, ("mali_runtime_idle() called\n"));

	if (NULL != device->driver &&
	    NULL != device->driver->pm &&
	    NULL != device->driver->pm->runtime_idle)
	{
		/* Need to notify Mali driver about this event */
		int ret = device->driver->pm->runtime_idle(device);
		if (0 != ret)
		{
			return ret;
		}
	}

	pm_runtime_suspend(device);

	return 0;
}
#endif /// CONFIG_PM_RUNTIME
