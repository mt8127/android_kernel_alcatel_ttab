#include <linux/mali/mali_utgard.h>
#include "mali_kernel_common.h"
#include "mali_osk.h"
#include "platform_pmm.h"
#include "mach/mt_gpufreq.h"
#include <asm/atomic.h>
#include "arm_core_scaling.h"

#if defined(CONFIG_MALI400_PROFILING)
#include "mali_osk_profiling.h"
#endif


extern unsigned long (*mtk_thermal_get_gpu_loading_fp)(void);
extern unsigned int (*mtk_get_gpu_loading_fp)(void);

static int bPoweroff;

/// #define __POWER_CLK_CTRL_SYNC__
/// For MFG sub-system clock control API
#include <mach/mt_clkmgr.h>
#include <linux/kernel.h>

static unsigned int current_sample_utilization = 0;

#if defined(__MALI_CORE_SCALING_ENABLE__)
   extern int mali_core_scaling_enable;
#endif

static DEFINE_SPINLOCK(mali_pwr_lock);

#define mfg_pwr_lock(flags) \
do { \
    spin_lock_irqsave(&mali_pwr_lock, flags); \
} while(0)

#define mfg_pwr_unlock(flags) \
do { \
    spin_unlock_irqrestore(&mali_pwr_lock, flags); \
} while(0)

extern u32 get_devinfo_with_index(u32 index);
static u32 check_need_univpll() {
    u32 info = *(volatile u32 *)0xf0206174;
	/*get_devinfo_with_index (15);*/
    /*if(0x0 == (info & (0x1 << 31))) { t or b?*/
        /*T*/
	u32 devinfo = *(volatile u32* )0xf0206040;/*get_devinfo_with_index(3);*/
    if(devinfo & 0x80000) {
        MALI_DEBUG_PRINT(1, ("GPU use univ with devinfo 0x%x\n", devinfo));
        return 1;
    } else {
#ifdef MTK_MALI_UNIV
        MALI_DEBUG_PRINT(1, ("GPU use univ with MTK_MALI_UNIV\n"));
        return 1;
#else
        return 0;
#endif  
    }
    
}
static int _need_univpll = 0;

void mali_pmm_init(void)
{
    MALI_DEBUG_PRINT(1, ("%s\n", __FUNCTION__));

	_need_univpll = check_need_univpll();
	
    MALI_DEBUG_PRINT(1, ("need univ src pll %d\n", _need_univpll));

    mtk_thermal_get_gpu_loading_fp = gpu_get_current_utilization;
    mtk_get_gpu_loading_fp = gpu_get_current_utilization;
	
	unsigned long flags;

    /* Because clkmgr may do 'default on' for some clock.
       We check the clock state on init and set power state atomic.
    */
    mfg_pwr_lock(flags);
    MALI_DEBUG_PRINT(1, ("MFG G3D init enable if it is on\n"));
    if(clock_is_on(MT_CG_MFG_G3D)) {
        MALI_DEBUG_PRINT(1, ("MFG G3D default on\n"));
        atomic_set((atomic_t *)&bPoweroff, 0);
        /* Need call enable first for 'default on' clocks. 
         * Canbe removed if clkmgr remove this requirement.
         */
        enable_clock(MT_CG_DISP0_SMI_COMMON, "MFG");
        enable_clock(MT_CG_MFG_G3D, "MFG");
    } else {
        MALI_DEBUG_PRINT(1, ("MFG G3D init default off\n"));
        atomic_set((atomic_t *)&bPoweroff, 1);
    }
    mfg_pwr_unlock(flags);
    mali_platform_power_mode_change(MALI_POWER_MODE_ON);
}

void mali_pmm_deinit(void)
{
    MALI_DEBUG_PRINT(1, ("%s\n", __FUNCTION__));

    mali_platform_power_mode_change(MALI_POWER_MODE_DEEP_SLEEP);
}

/* this function will be called periodically with sampling period 200ms~1000ms */
void mali_pmm_utilization_handler(struct mali_gpu_utilization_data *data)
{
   current_sample_utilization = (unsigned int )data->utilization_gpu;
   
   MALI_DEBUG_PRINT(4, ("%s: GPU utilization=%d\n", __FUNCTION__, current_sample_utilization));
	
#if defined(__MALI_CORE_SCALING_ENABLE__)  	
	if (1 == mali_core_scaling_enable) {
		mali_core_scaling_update(data);
	}
#endif	
}

unsigned long gpu_get_current_utilization(void)
{
    return (current_sample_utilization * 100)/256;
}



void g3d_power_domain_control(int bpower_on)
{
   if (bpower_on)
   {
      MALI_DEBUG_PRINT(2,("enable_subsys \n"));
      //enable_subsys(SYS_MFG, "G3D_MFG");
   }
   else
   {
      MALI_DEBUG_PRINT(2,("disable_subsys_force \n"));
      //disable_subsys(SYS_MFG, "G3D_MFG");
   }
}



void mali_platform_power_mode_change(mali_power_mode power_mode)
{
   unsigned long flags;
   switch (power_mode)
   {
      case MALI_POWER_MODE_ON:
         mfg_pwr_lock(flags);
         MALI_DEBUG_PRINT(3, ("Mali platform: Got MALI_POWER_MODE_ON event, %s\n",
                              atomic_read((atomic_t *)&bPoweroff) ? "powering on" : "already on"));
         if (atomic_read((atomic_t *)&bPoweroff) == 1)
         {
            /*Leave this to undepend ref count of clkmgr*/
            if (!clock_is_on(MT_CG_MFG_G3D))
            {
         		MALI_DEBUG_PRINT(3,("MFG enable_clock \n"));
				if(_need_univpll) {
					enable_pll(UNIVPLL, "GPU"); 
				}
                enable_clock(MT_CG_DISP0_SMI_COMMON, "MFG");
                enable_clock(MT_CG_MFG_G3D, "MFG");
				if(_need_univpll) {
					clkmux_sel(MT_MUX_MFG, 6, "GPU");
				}
            }

#if defined(CONFIG_MALI400_PROFILING)
            _mali_osk_profiling_add_event(MALI_PROFILING_EVENT_TYPE_SINGLE |
                  MALI_PROFILING_EVENT_CHANNEL_GPU |
                  MALI_PROFILING_EVENT_REASON_SINGLE_GPU_FREQ_VOLT_CHANGE, 500,
                  1200/1000, 0, 0, 0);

#endif
            atomic_set((atomic_t *)&bPoweroff, 0);
         }
         mfg_pwr_unlock(flags);
         break;
      case MALI_POWER_MODE_LIGHT_SLEEP:
      case MALI_POWER_MODE_DEEP_SLEEP:

         mfg_pwr_lock(flags);
         MALI_DEBUG_PRINT(3, ("Mali platform: Got %s event, %s\n", power_mode ==
                  MALI_POWER_MODE_LIGHT_SLEEP ?  "MALI_POWER_MODE_LIGHT_SLEEP" :
                  "MALI_POWER_MODE_DEEP_SLEEP",  atomic_read((atomic_t *)&bPoweroff) ? "already off" : "powering off"));
       
         if (atomic_read((atomic_t *)&bPoweroff) == 0)
         { 
            //trace_printk("[GPU power] MFG OFF\n");
            if (clock_is_on(MT_CG_MFG_G3D))
            {
        		MALI_DEBUG_PRINT(3,("MFG disable_clock \n"));
                disable_clock(MT_CG_MFG_G3D, "MFG");
                disable_clock(MT_CG_DISP0_SMI_COMMON, "MFG");
				if(_need_univpll) {
					disable_pll(UNIVPLL, "GPU"); 
				}
            }

#if defined(CONFIG_MALI400_PROFILING)
            _mali_osk_profiling_add_event(MALI_PROFILING_EVENT_TYPE_SINGLE |
                  MALI_PROFILING_EVENT_CHANNEL_GPU |
                  MALI_PROFILING_EVENT_REASON_SINGLE_GPU_FREQ_VOLT_CHANGE, 0, 0, 0, 0, 0);
#endif
            atomic_set((atomic_t *)&bPoweroff, 1);
         }

         mfg_pwr_unlock(flags);
         break;
   }
}
