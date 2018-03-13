#ifndef __DISP_DRV_PLATFORM_H__
#define __DISP_DRV_PLATFORM_H__

#ifdef BUILD_UBOOT
#include <config.h>
#include <common.h>
#include <version.h>
#include <stdarg.h>
#include <linux/types.h>
#include <lcd.h>
#include <video_fb.h>
#include <mmc.h>
#include <part.h>
#include <fat.h>
#include <malloc.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/arch/boot_mode.h>
#include <asm/arch/mt65xx.h>
#include <asm/arch/mt65xx_typedefs.h>
#include <asm/arch/disp_drv.h>
#include <asm/arch/lcd_drv.h>
#include <asm/arch/dpi_drv.h>
#include <asm/arch/dsi_drv.h>
#include <asm/arch/lcd_reg.h>
#include <asm/arch/dpi_reg.h>
#include <asm/arch/dsi_reg.h>
#include <asm/arch/disp_assert_layer.h>
#include <asm/arch/disp_drv_log.h>
#include <asm/arch/mt65xx_disp_drv.h>
#include "lcm_drv.h"


#undef CONFIG_MTK_M4U_SUPPORT
#undef CONFIG_MTK_HDMI_SUPPORT
#define DEFINE_SEMAPHORE(x)  
#define down_interruptible(x) 0
#define up(x)                
#define DBG_OnTriggerLcd()   

#else
#include <linux/dma-mapping.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/m4u.h>
//#include <mach/mt6585_pwm.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_irq.h>
//#include <mach/boot.h>
#include <board-custom.h>
#include <linux/disp_assert_layer.h>
#include "ddp_hal.h"
#include "ddp_drv.h"
#include "ddp_path.h"
#include "ddp_rdma.h"
#include "dsi_drv.h"
#endif

///LCD HW feature options for MT6575
#define MTK_LCD_HW_SIF_VERSION      2       ///for MT6575, we naming it is V2 because MT6516/73 is V1...
#define MTKFB_NO_M4U
#define MT65XX_NEW_DISP
//#define MTK_LCD_HW_3D_SUPPORT
#define ALIGN_TO(x, n)  \
	(((x) + ((n) - 1)) & ~((n) - 1))
#define MTK_FB_ALIGNMENT 16
#define MTK_FB_SYNC_SUPPORT
#ifndef MTK_OVERLAY_ENGINE_SUPPORT
#define MTK_OVL_DECOUPLE_SUPPORT
#endif

#if defined(MTK_ALPS_BOX_SUPPORT)
#define MTK_HDMI_MAIN_PATH 1 
#else
#define MTK_HDMI_MAIN_PATH 0
#endif

#define MTK_HDMI_MAIN_PATH_TEST 0
#define MTK_HDMI_MAIN_PATH_TEST_SIZE 0
#define MTK_DISABLE_HDMI_BUFFER_FROM_RDMA1 1

#define HDMI_DISP_WIDTH 1920
#define HDMI_DISP_HEIGHT 1080

#define HDMI_DEFAULT_RESOLUTION HDMI_VIDEO_1920x1080p_60Hz

#endif //__DISP_DRV_PLATFORM_H__
