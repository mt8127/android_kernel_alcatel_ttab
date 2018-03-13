#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <generated/autoconf.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/param.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <linux/xlog.h>
#include <asm/io.h>

#include <mach/irqs.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_irq.h>
#include <mach/irqs.h>
#include <mach/mt_clkmgr.h> // ????
#include <mach/mt_irq.h>
#include <mach/sync_write.h>
#include <mach/m4u.h>

#include "ddp_hal.h"
#include "ddp_drv.h"
#include "ddp_reg.h"
#include "ddp_path.h"
#include "ddp_debug.h"
#include "ddp_bls.h"
#include "ddp_rdma.h"
#include "ddp_wdma.h"
#include "ddp_ovl.h"
#include "debug.h"

/*for MTK_HDMI_MAIN_PATH*/
#include "disp_drv_platform.h"

#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
#include <tz_cross/trustzone.h>
#include <tz_cross/tz_ddp.h>
#include <mach/m4u_port.h>
#include <tz_cross/ta_mem.h>
#include "trustzone/kree/system.h"
#include "trustzone/kree/mem.h"
#endif

extern unsigned int gOvlSecure;
extern unsigned int gOvlLayerSecure[4];
extern unsigned int gOvlLayerSecureLast[4];
extern unsigned int gMemOutSecure;
extern unsigned int gBitbltSecure;

#define DISP_INDEX_OFFSET 0x0  // must be consistent with ddp_rdma.c

unsigned int gMutexID = 0;
unsigned int gMemOutMutexID = 1;
extern unsigned int decouple_addr;
extern BOOL DISP_IsDecoupleMode(void);
extern UINT32 DISP_GetScreenWidth(void);
extern UINT32 DISP_GetScreenHeight(void);

UINT32 fb_width = 0;
UINT32 fb_height = 0;
static DEFINE_MUTEX(DDPMutexLock0);
static DEFINE_MUTEX(DDPMutexLock1);
static DEFINE_MUTEX(DDPMutexLock2);
static DEFINE_MUTEX(DDPMutexLock3);
static DEFINE_MUTEX(DDPMutexLockDefault);




DECLARE_WAIT_QUEUE_HEAD(mem_out_wq);
unsigned int mem_out_done = 0;

static void disp_reg_backup_module(DISP_MODULE_ENUM module);
static void disp_reg_restore_module(DISP_MODULE_ENUM module);
extern void DpEngine_COLORonConfig(unsigned int srcWidth,unsigned int srcHeight);
extern    void DpEngine_COLORonInit(void);
extern BOOL DISP_IsVideoMode(void);
extern void _DISP_DumpLayer(OVL_CONFIG_STRUCT* pLayer);


#ifdef DDP_USE_CLOCK_API
static int disp_reg_backup(void);
static int disp_reg_restore(void);
#endif

extern unsigned int disp_ms2jiffies(unsigned long ms);

extern unsigned char pq_debug_flag;
static DECLARE_WAIT_QUEUE_HEAD(g_disp_mutex_wq);
static unsigned int g_disp_mutex_reg_update[4] = {0, 0, 0, 0};

// Mutex register update timeout intr
#define DDP_MUTEX_INTR_BIT 0x0200

#if defined(MTK_OVERLAY_ENGINE_SUPPORT)
void (*gdisp_path_register_ovl_rdma_callback)(unsigned int param);
#endif

#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
// these 2 APIs are used for accessing ddp_session / ddp_mem_session with TEE
extern KREE_SESSION_HANDLE ddp_session_handle(void);
extern KREE_SESSION_HANDLE ddp_mem_session_handle(void);
#endif

#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
#ifndef MTK_OVERLAY_ENGINE_SUPPORT
static int updateSecureOvl(int secure)
{
    MTEEC_PARAM param[4];
    unsigned int paramTypes;
    TZ_RESULT ret;
    param[0].value.a = (secure == 0 ? 0 : 1);
    DISP_DBG("[DAPC] change secure protection to %d \n", param[0].value.a);
    param[1].value.a = 3; // OVL
    paramTypes = TZ_ParamTypes2(TZPT_VALUE_INPUT,TZPT_VALUE_INPUT);
    ret = KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_SET_DAPC_MODE, paramTypes, param);
    if(ret!= TZ_RESULT_SUCCESS)
    {
        DISP_ERR("KREE_TeeServiceCall(TZCMD_DDP_SET_DAPC_MODE) fail, ret=%d \n", ret);
    }

    return 0;
}
#endif
#endif

static void _disp_path_mutex_reg_update_cb(unsigned int param)
{
    unsigned int mutexID;

    for (mutexID = 0; mutexID<4; mutexID++)
    {
        if (param & (1 << mutexID))
        {
            g_disp_mutex_reg_update[mutexID] = 1;
            wake_up_interruptible(&g_disp_mutex_wq);
            
            MMProfileLogEx(DDP_MMP_Events.Mutex[mutexID], MMProfileFlagPulse, mutexID, mutexID);
        }
        if (param & (DDP_MUTEX_INTR_BIT << mutexID)) {
        	DISP_ERR("mutex%d register update timeout! commit=0x%x, mod=0x%x \n",
        			mutexID, DISP_REG_GET(DISP_REG_CONFIG_REG_COMMIT),
        			DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(mutexID)));
        	// clear intr status
        	DISP_REG_SET(DISP_REG_CONFIG_MUTEX_INTSTA, 0);
        	// dump reg info
        	disp_dump_reg(DISP_MODULE_CONFIG);
        	disp_dump_reg(DISP_MODULE_MUTEX);
        	disp_dump_reg(DISP_MODULE_OVL);
        	disp_dump_reg(DISP_MODULE_RDMA0);
        	disp_dump_reg(DISP_MODULE_COLOR);
        	disp_dump_reg(DISP_MODULE_BLS);
        }
    }
}

#if 1
#include "dsi_drv.h"
static void disp_path_reset(void) 
{
	// Reset OVL    
 	OVLReset();

	// Reset RDMA1
	RDMAReset(0);

	// Reset Color
	DpEngine_COLORonInit();
	DpEngine_COLORonConfig(DISP_GetScreenWidth(), DISP_GetScreenHeight());

	// Reset BLS
	DISP_REG_SET(DISP_REG_BLS_RST, 0x1);
	DISP_REG_SET(DISP_REG_BLS_RST, 0x0);
	disp_bls_init(DISP_GetScreenWidth(), DISP_GetScreenHeight());

    // reset DSI
    DSI_Reset();
    msleep(1);
    DSI_Start();
}
#endif

int disp_path_ovl_reset(void)
{
    static unsigned int ovl_hw_reset = 0;

    DISP_REG_SET(DISP_REG_OVL_RST, 0x1);              // soft reset
    DISP_REG_SET(DISP_REG_OVL_RST, 0x0);
    DISP_MSG("after sw reset in intr, flow_ctrl=0x%x \n", DISP_REG_GET(DISP_REG_OVL_FLOW_CTRL_DBG));
    if(((DISP_REG_GET(DISP_REG_OVL_FLOW_CTRL_DBG)&0x3ff) != 0x1) &&
    		((DISP_REG_GET(DISP_REG_OVL_FLOW_CTRL_DBG)&0x3ff) != 0x2))
    {
      // HW reset will reset all registers to power on state, so need to backup and restore
      disp_reg_backup_module(DISP_MODULE_OVL);
      DISP_REG_SET(DISP_REG_CONFIG_MMSYS_SW_RST_B, ~(1<<8));
      DISP_REG_SET(DISP_REG_CONFIG_MMSYS_SW_RST_B, ~0);
      disp_reg_restore_module(DISP_MODULE_OVL);
      DISP_MSG("after hw reset, flow_ctrl=0x%x \n", DISP_REG_GET(DISP_REG_OVL_FLOW_CTRL_DBG));

      // if ovl hw reset dose not have effect, will reset whole path
      if((ovl_hw_reset > 0) && (DISP_IsDecoupleMode() == 0))
      {
         DISP_MSG("disp_path_reset called! \n");
         disp_path_reset();
      }
      ovl_hw_reset++;      
    }

    return 0;
}

unsigned int gNeedToRecover = 0; // for UT debug test
#ifdef DDP_USE_CLOCK_API
extern unsigned int* pRegBackup;
int disp_intr_restore(void);
static void disp_path_recovery(unsigned int mutexID)
{
	if(gNeedToRecover || (DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(mutexID))==0))
	{
		DISP_ERR("mutex%d_mod invalid, try to recover!\n", mutexID);
		gNeedToRecover = 0;
		if (*pRegBackup != DDP_UNBACKED_REG_MEM)
		{
			disp_reg_restore();
			disp_intr_restore();
		}
		else
		{
			DISP_ERR("recover failed!\n");
		}
	}
}
#endif

unsigned int disp_mutex_lock_cnt[MUTEX_RESOURCE_NUM] = {0};
unsigned int disp_mutex_unlock_cnt[MUTEX_RESOURCE_NUM] = {0};
int disp_path_get_mutex(void)
{
    if(pq_debug_flag == 3)
    {
        return 0;
    }
    else
    {
        return disp_path_get_mutex_(gMutexID);
    }
}

int disp_path_get_mutex_(int mutexID)
{
    unsigned int cnt=0;
#ifdef DDP_USE_CLOCK_API
    // If mtcmos is shutdown once a while unfortunately
    disp_path_recovery(mutexID);
#endif
    disp_register_irq(DISP_MODULE_MUTEX, _disp_path_mutex_reg_update_cb);
    //DISP_MSG("disp_path_get_mutex %d, %d\n", disp_mutex_lock_cnt[mutexID]++, mutexID);
	if(mutexID == 0)
		mutex_lock(&DDPMutexLock0);
	else if(mutexID == 1)
		mutex_lock(&DDPMutexLock1);
	else if(mutexID == 2)
		mutex_lock(&DDPMutexLock2);
	else if(mutexID == 3)
		mutex_lock(&DDPMutexLock3);
	else
		mutex_lock(&DDPMutexLockDefault);

    MMProfileLogEx(DDP_MMP_Events.Mutex[mutexID], MMProfileFlagStart, 
        DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(mutexID)), DISP_REG_GET(DISP_REG_CONFIG_MUTEX_SOF(mutexID)));
    DISP_REG_SET(DISP_REG_CONFIG_MUTEX_EN(mutexID), 1);
    
    DISP_REG_SET(DISP_REG_CONFIG_MUTEX(mutexID), 1);
    DISP_REG_SET_FIELD(REG_FLD(1, mutexID), DISP_REG_CONFIG_MUTEX_INTSTA, 0);

    while(((DISP_REG_GET(DISP_REG_CONFIG_MUTEX(mutexID))& DISP_INT_MUTEX_BIT_MASK) != DISP_INT_MUTEX_BIT_MASK))
    {
        cnt++;
		udelay(1);
        if(cnt>10000)
        {
            DISP_ERR("disp_path_get_mutex() %d timeout!DISP_REG_CONFIG_MUTEX=0x%08x \n", mutexID,DISP_REG_GET(DISP_REG_CONFIG_MUTEX(mutexID)));

            disp_clock_check();
            disp_dump_reg(DISP_MODULE_MUTEX);
            disp_dump_reg(DISP_MODULE_OVL);
            disp_dump_reg(DISP_MODULE_RDMA0);
            disp_dump_reg(DISP_MODULE_COLOR);
            disp_dump_reg(DISP_MODULE_BLS);
            MMProfileLogEx(DDP_MMP_Events.Mutex[mutexID], MMProfileFlagPulse, 
                DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(mutexID)), DISP_REG_GET(DISP_REG_CONFIG_MUTEX_SOF(mutexID)));
            break;
        }
    }

    return 0;
}
int disp_path_release_mutex(void)
{
    if(pq_debug_flag == 3)
    {
        return 0;
    }
    else
    {
        g_disp_mutex_reg_update[gMutexID] = 0;
        return disp_path_release_mutex_(gMutexID);
    }
}

extern int disp_path_release_mutex1_(int mutexID);

int disp_path_release_mutex1(void)
{
    if(pq_debug_flag == 3)
    {
        return 0;
    }
    else
    {
        g_disp_mutex_reg_update[gMutexID] = 0;
        return disp_path_release_mutex1_(gMutexID);
    }
}

// check engines' clock bit and enable bit before unlock mutex
// todo: modify bit info according to new chip change
#define DDP_SMI_COMMON_POWER_BIT     (1<<0)
#define DDP_SMI_LARB0_POWER_BIT      (1<<1)
#define DDP_OVL_POWER_BIT     (1<<8)
#define DDP_RDMA0_POWER_BIT   (1<<7)
#define DDP_WDMA0_POWER_BIT   (1<<6)
#define DDP_BLS_POWER_BIT     (1<<5)
#define DDP_COLOR_POWER_BIT   (1<<4)

#define DDP_OVL_UPDATE_BIT     (1<<3)
#define DDP_RDMA0_UPDATE_BIT   (1<<10)
#define DDP_COLOR_UPDATE_BIT   (1<<7)
#define DDP_BLS_UPDATE_BIT     (1<<9)
#define DDP_WDMA0_UPDATE_BIT   (1<<6)

// Mutex0 reg update intr, reg update timeout intr
#define DDP_MUTEX_INTEN_BIT	0x0201
// LSB 10bit
#define DDP_OVL_IDLE_BIT       (1<<0)
#define DDP_OVL_WAIT_BIT       (1<<1)

int disp_check_engine_status(int mutexID)
{
    int result = 0;
    unsigned int engine = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(mutexID)); 
    unsigned int mutex_inten = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTEN);

    if((DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0)&DDP_SMI_LARB0_POWER_BIT) != 0)
    {
        result = -1;
        DISP_ERR("smi clk abnormal, clk=0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0));
    }
    if( (mutex_inten & DDP_MUTEX_INTEN_BIT) != DDP_MUTEX_INTEN_BIT )
    {
    	DISP_ERR("mutex0 inten abnormal, inten=0x%x \n", mutex_inten);
    	DISP_REG_SET(DISP_REG_CONFIG_MUTEX_INTEN, mutex_inten | DDP_MUTEX_INTEN_BIT);
    }

    if(engine&DDP_OVL_UPDATE_BIT) //OVL
    {
    	// clock on and enabled
        if(DISP_REG_GET(DISP_REG_OVL_EN)==0 || 
           (DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0)&DDP_OVL_POWER_BIT) != 0)
        {
            result = -1;
            DISP_ERR("ovl abnormal, en=%d, clk=0x%x \n", 
                DISP_REG_GET(DISP_REG_OVL_EN), 
                DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0));
        }
        // ROI w, h >= 2
        if( ((DISP_REG_GET(DISP_REG_OVL_ROI_SIZE) & 0x0fff) < 2) ||
        		(((DISP_REG_GET(DISP_REG_OVL_ROI_SIZE) >> 16) & 0x0fff) < 2) )
        {
        	result = -1;
        	DISP_ERR("ovl abnormal, roiW=%d, roiH=%d \n",
        			DISP_REG_GET(DISP_REG_OVL_ROI_SIZE)&0x0fff,
        			(DISP_REG_GET(DISP_REG_OVL_ROI_SIZE) >> 16)&0x0fff);
        }
        // OVL at IDLE state(0x1), if en=0; OVL at WAIT state(0x2), if en=1
        if((DISP_IsVideoMode()==0) &&
           ((DISP_REG_GET(DISP_REG_OVL_FLOW_CTRL_DBG)&0x3ff) != 0x1) &&
           ((DISP_REG_GET(DISP_REG_OVL_FLOW_CTRL_DBG)&0x3ff) != 0x2))
        {
        	// Reset hw if it happens, to prevent last frame error from affect the
        	// next frame transfer
        	result = -1;
        	DISP_ERR("ovl abnormal, flow_ctrl=0x%x, add_con=0x%x \n", 
        	    DISP_REG_GET(DISP_REG_OVL_FLOW_CTRL_DBG),
        	    DISP_REG_GET(DISP_REG_OVL_ADDCON_DBG));
        	disp_clock_check();
        	ddp_dump_info(DISP_MODULE_CONFIG);
            ddp_dump_info(DISP_MODULE_MUTEX);
            ddp_dump_info(DISP_MODULE_OVL);
        	disp_path_ovl_reset();
        }
    }
    if(engine&DDP_RDMA0_UPDATE_BIT) //RDMA0
    {
        if((DISP_REG_GET(DISP_REG_RDMA_GLOBAL_CON)&0x1) ==0 || 
           (DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0)&DDP_RDMA0_POWER_BIT) != 0)
        {
            result = -1;
            DISP_ERR("rdma0 abnormal, en=%d, clk=0x%x \n", 
                DISP_REG_GET(DISP_REG_RDMA_GLOBAL_CON), 
                DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0));
        }
    }    
    if(engine&DDP_COLOR_UPDATE_BIT) //COLOR
    {

    }
    if(engine&DDP_WDMA0_UPDATE_BIT) //WDMA0
    {
		// [DAPC] when the engine is already in DAPC protected status, actually it cannot read the register	
        #if 0
		if(DISP_REG_GET(DISP_REG_WDMA_EN)==0 || 
           (DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0)&DDP_WDMA0_POWER_BIT) != 0)
        {
            result = -1;
            DISP_ERR("wdma0 abnormal, en=%d, clk=0x%x \n", 
                DISP_REG_GET(DISP_REG_WDMA_EN), 
                DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0));
        }
		#endif
    }
    if(engine&DDP_BLS_UPDATE_BIT) //BLS
    {

    }

    if(result!=0)
    {
        DISP_ERR("engine status error before release mutex, engine=0x%x, mutexID=%d \n", engine, mutexID);
        ddp_dump_info(DISP_MODULE_CONFIG);
        ddp_dump_info(DISP_MODULE_MUTEX);
        ddp_dump_info(DISP_MODULE_OVL);
        ddp_dump_info(DISP_MODULE_RDMA0);
        ddp_dump_info(DISP_MODULE_DSI_CMD);
        disp_dump_reg(DISP_MODULE_OVL);
    }

    return result;

}


int disp_path_release_mutex_(int mutexID)
{
//  unsigned int reg = 0;
//    unsigned int cnt = 0;
    disp_check_engine_status(mutexID);
    DISP_REG_SET(DISP_REG_CONFIG_MUTEX(mutexID), 0);
#if 0
    while(((DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTSTA) & (1<<mutexID)) != (1<<mutexID)))
    {
        if((DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTSTA) & (1<<(mutexID+6))) == (1<<(mutexID+6)))
        {
            DISP_ERR("disp_path_release_mutex() timeout! \n");
            disp_dump_reg(DISP_MODULE_CONFIG);
            //print error engine
            reg = DISP_REG_GET(DISP_REG_CONFIG_REG_COMMIT);
            if(reg!=0)
            {
                  if(reg&(1<<3))  { DISP_MSG(" OVL update reg timeout! \n"); disp_dump_reg(DISP_MODULE_OVL); }
                  if(reg&(1<<10))  { DISP_MSG(" RDMA1 update reg timeout! \n"); disp_dump_reg(DISP_MODULE_RDMA1); }
                  if(reg&(1<<7))  { DISP_MSG(" COLOR update reg timeout! \n"); disp_dump_reg(DISP_MODULE_COLOR); }
                  if(reg&(1<<9))  { DISP_MSG(" BLS update reg timeout! \n"); disp_dump_reg(DISP_MODULE_BLS); }
                  if(reg&(1<<6))  { DISP_MSG(" WDMA0 update reg timeout! \n"); disp_dump_reg(DISP_MODULE_WDMA); }
            }  
                     
            //reset mutex
            DISP_REG_SET(DISP_REG_CONFIG_MUTEX_RST(mutexID), 1);
            DISP_REG_SET(DISP_REG_CONFIG_MUTEX_RST(mutexID), 0);
            DISP_MSG("mutex reset done! \n");
            MMProfileLogEx(DDP_MMP_Events.Mutex0, MMProfileFlagPulse, mutexID, 1);
            break;
        }

        cnt++;
        if(cnt>1000)
        {
            DISP_ERR("disp_path_release_mutex() timeout! \n");
            MMProfileLogEx(DDP_MMP_Events.Mutex0, MMProfileFlagPulse, mutexID, 2);
            break;
        }
    }

    // clear status
    reg = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTSTA);
    reg &= ~(1<<mutexID);
    reg &= ~(1<<(mutexID+6));
    DISP_REG_SET(DISP_REG_CONFIG_MUTEX_INTSTA, reg);
#endif
    MMProfileLogEx(DDP_MMP_Events.Mutex[mutexID], MMProfileFlagEnd, 
        DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(mutexID)), DISP_REG_GET(DISP_REG_CONFIG_MUTEX_SOF(mutexID)));
    if(mutexID == 0)
		mutex_unlock(&DDPMutexLock0);
	else if(mutexID == 1)
		mutex_unlock(&DDPMutexLock1);
	else if(mutexID == 2)
		mutex_unlock(&DDPMutexLock2);
	else if(mutexID == 3)
		mutex_unlock(&DDPMutexLock3);
	else
		mutex_unlock(&DDPMutexLockDefault);
    //DISP_MSG("disp_path_release_mutex %d \n", disp_mutex_unlock_cnt[mutexID]++);

    return 0;
}
int disp_path_release_mutex1_(int mutexID)
{

    disp_check_engine_status(mutexID);

    DISP_REG_SET(DISP_REG_CONFIG_MUTEX(mutexID), 0);


    MMProfileLogEx(DDP_MMP_Events.Mutex[mutexID], MMProfileFlagEnd, 
        DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(mutexID)), DISP_REG_GET(DISP_REG_CONFIG_MUTEX_SOF(mutexID)));

    return 0;
}

#if 0
static int disp_path_release_soft_mutex(void)
{

    //mutex_unlock(&DpEngineMutexLock);

    return 0;
}
#endif

int disp_path_wait_reg_update(unsigned int mutexID)
{
    int ret = 0;
    // use timeout version instead of wait-forever
    ret = wait_event_interruptible_timeout(
                    g_disp_mutex_wq,
                    g_disp_mutex_reg_update[mutexID],
                    disp_ms2jiffies(1000) );
                    
    /*wake-up from sleep*/
    if(ret==0) // timeout
    {
        DISP_ERR("disp_path_wait_reg_update timeout \n");
        disp_dump_reg(DISP_MODULE_CONFIG); 
        disp_dump_reg(DISP_MODULE_MUTEX);
        disp_dump_reg(DISP_MODULE_OVL);
        disp_dump_reg(DISP_MODULE_RDMA0);
        disp_dump_reg(DISP_MODULE_COLOR);
        disp_dump_reg(DISP_MODULE_BLS);
    }
    else if(ret<0) // intr by a signal
    {
        DISP_ERR("disp_path_wait_reg_update  intr by a signal ret=%d \n", ret);
    }
    
    return 0;
}
int disp_path_change_tdshp_status(unsigned int layer, unsigned int enable)
{
    return 0;
}

///============================================================================
// OVL decouple @{
///========================
void _disp_path_wdma_callback(unsigned int param);

int disp_path_get_mem_read_mutex (void) {
	disp_path_get_mutex_(gMutexID);
	return 0;
}

int disp_path_release_mem_read_mutex (void) {
	disp_path_release_mutex_(gMutexID);
	return 0;
}

int disp_path_get_mem_write_mutex (void) {
	disp_path_get_mutex_(gMemOutMutexID);
	return 0;
}

int disp_path_release_mem_write_mutex (void) {
	disp_path_release_mutex_(gMemOutMutexID);
	return 0;
}

#if 0
static int disp_path_get_m4u_moduleId (DISP_MODULE_ENUM module) {
	int m4u_module = M4U_PORT_UNKNOWN;
	switch (module) {
	case DISP_MODULE_OVL:
		m4u_module = DISP_OVL_0;
		break;
	case DISP_MODULE_RDMA0:
		m4u_module = DISP_RDMA;
		break;
    case DISP_MODULE_RDMA1:
		m4u_module = DISP_RDMA1;
		break;
	case DISP_MODULE_WDMA:
		m4u_module = DISP_WDMA;
		break;
	default:
		break;
	}

	return m4u_module;
}
#endif

static int disp_path_init_m4u_port (DISP_MODULE_ENUM module) {
	int m4u_module = M4U_PORT_UNKNOWN;
    M4U_PORT_STRUCT portStruct;
    
	switch (module) {
	case DISP_MODULE_OVL:
		m4u_module = DISP_OVL_0;
		break;
	case DISP_MODULE_RDMA0:
		m4u_module = DISP_RDMA;
		break;
    case DISP_MODULE_RDMA1:
		m4u_module = DISP_RDMA1;
		break;
	case DISP_MODULE_WDMA:
		m4u_module = DISP_WDMA;
		break;
	default:
		break;
	}
    
    portStruct.ePortID = m4u_module;		   //hardware port ID, defined in M4U_PORT_ID_ENUM
    portStruct.Virtuality = 1;
    portStruct.Security = 0;
    portStruct.domain = 3;            //domain : 0 1 2 3
    portStruct.Distance = 1;
    portStruct.Direction = 0;
    m4u_config_port(&portStruct);
    return 0;
}

#if 0
static int disp_path_deinit_m4u_port (DISP_MODULE_ENUM module) {
	int m4u_module = M4U_PORT_UNKNOWN;
	switch (module) {
	case DISP_MODULE_OVL:
		m4u_module = DISP_OVL_0;
		break;
	case DISP_MODULE_RDMA0:
		m4u_module = DISP_RDMA;
		break;
    case DISP_MODULE_RDMA1:
		m4u_module = DISP_RDMA1;
		break;
	case DISP_MODULE_WDMA:
		m4u_module = DISP_WDMA;
		break;
	default:
		break;
	}
    M4U_PORT_STRUCT portStruct;
    portStruct.ePortID = m4u_module;		   //hardware port ID, defined in M4U_PORT_ID_ENUM
    portStruct.Virtuality = 0;
    portStruct.Security = 0;
    portStruct.domain = 0;            //domain : 0 1 2 3
    portStruct.Distance = 1;
    portStruct.Direction = 0;
    m4u_config_port(&portStruct);
    return 0;
}
#endif

/**
 * In decouple mode, disp_config_update_kthread will call this to reconfigure
 * next buffer to RDMA->LCM
 */
int disp_path_config_rdma (RDMA_CONFIG_STRUCT* pRdmaConfig) {
	DISP_DBG("[DDP] config_rdma(), idx=%d, mode=%d, in_fmt=%d, addr=0x%x, out_fmt=%d, pitch=%d, x=%d, y=%d, byteSwap=%d, rgbSwap=%d \n ",
			pRdmaConfig->idx,
			pRdmaConfig->mode,       	 // direct link mode
			pRdmaConfig->inputFormat,    // inputFormat
			pRdmaConfig->address,        // address
			pRdmaConfig->outputFormat,   // output format
			pRdmaConfig->pitch,       	 // pitch
			pRdmaConfig->width,       	 // width
			pRdmaConfig->height,      	 // height
			pRdmaConfig->isByteSwap,  	 // byte swap
			pRdmaConfig->isRGBSwap);
	// TODO: just need to reconfig buffer address now! because others are configured before
	//RDMASetAddress(pRdmaConfig->idx, pRdmaConfig->address);
	RDMAConfig(pRdmaConfig->idx, pRdmaConfig->mode, pRdmaConfig->inputFormat, pRdmaConfig->address,
			pRdmaConfig->outputFormat, pRdmaConfig->pitch, pRdmaConfig->width,
			pRdmaConfig->height, pRdmaConfig->isByteSwap, pRdmaConfig->isRGBSwap);
	return 0;
}

/**
 * In decouple mode, disp_ovl_worker_kthread will call this to reconfigure
 * next output buffer to OVL-->WDMA
 */
int disp_path_config_wdma (struct disp_path_config_mem_out_struct* pConfig) {
	DISP_DBG("[DDP] config_wdma(), idx=%d, dstAddr=%x, out_fmt=%d\n",
			0,
			pConfig->dstAddr,
			pConfig->outFormat);
	// TODO: just need to reconfig buffer address now! because others are configured before
	WDMAConfigAddress(0, pConfig->dstAddr);

	return 0;
}

/**
 * switch decouple:
 * 1. save mutex0(gMutexID) contents
 * 2. reconfigure OVL->WDMA within mutex0(gMutexID)
 * 3. move engines in mutex0(gMutexID) to mutex1(gLcmMutexID)
 * 4. reconfigure MMSYS_OVL_MOUT
 * 5. reconfigure RDMA mode
 */
int disp_path_switch_ovl_mode (struct disp_path_config_ovl_mode_t *pConfig) {
    int reg_mutex_mod;
    //int reg_mutex_sof;

    if (DISP_IsDecoupleMode()) {
    	//reg_mutex_sof = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_SOF(gMutexID));
    	reg_mutex_mod = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(gMutexID));
        // ovl mout
        DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, 1<<1);   // ovl_mout output to wdma0
    	DISP_REG_SET(DISP_REG_CONFIG_MUTEX_INTEN , 0x0603);
    	DISP_REG_SET(DISP_REG_CONFIG_MUTEX_INTSTA, (1<<gMemOutMutexID)|(1<<gMutexID));
        // mutex0
        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(gMutexID), reg_mutex_mod&~(1<<3)); 	//remove OVL
        // mutex1
        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_EN(gMemOutMutexID), 1); 					//enable mutex1
        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(gMemOutMutexID), (1<<3)|(1<<6)); 	//OVL, WDMA
        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_SOF(gMemOutMutexID), 0);					//single mode

        disp_register_irq(DISP_MODULE_WDMA, _disp_path_wdma_callback);

        // config wdma0
        WDMAReset(0);
        WDMAConfig(0,
        		WDMA_INPUT_FORMAT_ARGB,
        		pConfig->roi.width,
        		pConfig->roi.height,
        		pConfig->roi.x,
        		pConfig->roi.y,
        		pConfig->roi.width,
        		pConfig->roi.height,
        		pConfig->format,
        		pConfig->address,
        		pConfig->roi.width,
        		1,
        		0);
        WDMAStart(0);

        // config rdma
        RDMAConfig(0,
        		RDMA_MODE_MEMORY,       		// memory mode
        		pConfig->format,    			// inputFormat
        		pConfig->address,       		// address, will be reset later
        		RDMA_OUTPUT_FORMAT_ARGB,     	// output format
        		pConfig->pitch,         		// pitch
        		pConfig->roi.width,
        		pConfig->roi.height,
        		0,                      		//byte swap
        		0);                     		// is RGB swap
        RDMAStart(0);
        //disp_dump_reg(DISP_MODULE_WDMA);
    } else {
    	//reg_mutex_sof = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_SOF(gMemOutMutexID));
    	reg_mutex_mod = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(gMutexID));
        // ovl mout
        DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, 1<<0);   // ovl_mout output to rdma
    	//DISP_REG_SET(DISP_REG_CONFIG_MUTEX_EN(gMutexID), 1);
    	DISP_REG_SET(DISP_REG_CONFIG_MUTEX_INTEN , 0x0201);
    	DISP_REG_SET(DISP_REG_CONFIG_MUTEX_INTSTA, 1<<gMutexID);
        // mutex0
        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(gMutexID), reg_mutex_mod|(1<<3)); //Add OVL
        //DISP_REG_SET(DISP_REG_CONFIG_MUTEX_SOF(gMutexID), reg_mutex_sof);
        // mutex1
        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(gMemOutMutexID), 0);
        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_EN(gMemOutMutexID), 0); //disable mutex1

        // config rdma
        RDMAConfig(0,
        		RDMA_MODE_DIRECT_LINK,       // direct link mode
        		pConfig->format,    		 // inputFormat
        		pConfig->address,            // address
        		RDMA_OUTPUT_FORMAT_ARGB,     // output format
        		pConfig->pitch,         	 // pitch
        		pConfig->roi.width,
        		pConfig->roi.height,
        		0,                           // byte swap
        		0);                          // is RGB swap

        disp_unregister_irq(DISP_MODULE_WDMA, _disp_path_wdma_callback);
        RDMAStart(0);
    }

    return 0;
}

int disp_path_wait_frame_done(void) {
    int ret = 0;
    ret = wait_event_interruptible_timeout(mem_out_wq, mem_out_done, disp_ms2jiffies(100) );
    if(ret==0)  {
        DISP_ERR("disp_path_wait_frame_done timeout \n");
        ddp_dump_info(DISP_MODULE_CONFIG);
        ddp_dump_info(DISP_MODULE_MUTEX);
        ddp_dump_info(DISP_MODULE_OVL);
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
		if(0 == (gOvlSecure|gOvlLayerSecure[0]|gOvlLayerSecure[1]|gOvlLayerSecure[2]|gOvlLayerSecure[3]))	//if secured skip dump wdma info for DAPC
#endif
		ddp_dump_info(DISP_MODULE_WDMA);
        disp_dump_reg(DISP_MODULE_OVL);
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
		if(0 == (gOvlSecure|gOvlLayerSecure[0]|gOvlLayerSecure[1]|gOvlLayerSecure[2]|gOvlLayerSecure[3]))
#endif
        	disp_dump_reg(DISP_MODULE_WDMA);
        // disp_path_ovl_reset();
    } else if(ret<0) {
        DISP_ERR("disp_path_wait_frame_done intr by a signal ret=%d \n", ret);
    }
    mem_out_done = 0;
    return ret;
}
// OVL decouple @}
///========================

//#define MANUAL_DEBUG
/**
 * In D-link mode, disp_config_update_kthread will call this to reconfigure next
 * buffer to OVL
 * In Decouple mode, disp_ovl_worker_kthread will call this to reconfigure next
 * buffer to OVL
 */
#ifndef MTK_OVERLAY_ENGINE_SUPPORT

int gOvlSecureLast = 0;
int gOvlSecureTag = 0;

int disp_path_config_layer(OVL_CONFIG_STRUCT* pOvlConfig)
{
//    unsigned int reg_addr;

    DISP_DBG("[DDP] config_layer(), layer=%d, en=%d, source=%d, fmt=%d, addr=0x%x, src(%d, %d), pitch=%d, dst(%d, %d, %d, %d), keyEn=%d, key=%d, aen=%d, alpha=%d, isTdshp=%d, idx=%d, sec=%d \n ",
        pOvlConfig->layer,   // layer
        pOvlConfig->layer_en,
        pOvlConfig->source,   // data source (0=memory)
        pOvlConfig->fmt,
        pOvlConfig->addr, // addr
        pOvlConfig->src_x,  // x
        pOvlConfig->src_y,  // y
        pOvlConfig->src_pitch, //pitch, pixel number
        pOvlConfig->dst_x,  // x
        pOvlConfig->dst_y,  // y
        pOvlConfig->dst_w, // width
        pOvlConfig->dst_h, // height
        pOvlConfig->keyEn,  //color key
        pOvlConfig->key,  //color key
        pOvlConfig->aen, // alpha enable
        pOvlConfig->alpha,
        pOvlConfig->isTdshp,
        pOvlConfig->buff_idx,
        pOvlConfig->security);

    // config overlay
    MMProfileLogEx(DDP_MMP_Events.Debug, MMProfileFlagPulse, pOvlConfig->layer, pOvlConfig->layer_en);
    if(pOvlConfig->security!=OVL_LAYER_SECURE_BUFFER)
    {
        #ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
        // if OVL already in secure state (DAPC protected)
        if (gOvlSecure == 1)
        {
            MTEEC_PARAM param[4];
            unsigned int paramTypes;
            TZ_RESULT ret;

            param[0].value.a = pOvlConfig->layer;
            param[1].value.a = pOvlConfig->layer_en;
            paramTypes = TZ_ParamTypes2(TZPT_VALUE_INPUT,TZPT_VALUE_INPUT);
            ret = KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_OVL_LAYER_SWITCH, paramTypes, param);
            if(ret!= TZ_RESULT_SUCCESS)
            {
                DISP_ERR("[DAPC] KREE_TeeServiceCall(TZCMD_DDP_OVL_LAYER_SWITCH) fail, ret=%d \n", ret);
            }
        }
        else
        {
        #endif
            OVLLayerSwitch(pOvlConfig->layer, pOvlConfig->layer_en);
        #ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
        }
        #endif
    }
    else
    {
        #ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
        MTEEC_PARAM param[4];
        unsigned int paramTypes;
        TZ_RESULT ret;
        
        param[0].value.a = pOvlConfig->layer;
        param[1].value.a = pOvlConfig->layer_en;
        paramTypes = TZ_ParamTypes2(TZPT_VALUE_INPUT,TZPT_VALUE_INPUT);
        ret = KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_OVL_LAYER_SWITCH, paramTypes, param);
        if(ret!= TZ_RESULT_SUCCESS)
        {
            DISP_ERR("KREE_TeeServiceCall(TZCMD_DDP_OVL_LAYER_SWITCH) fail, ret=%d \n", ret);
        }
        #else
        DISP_ERR("error, do not support security==OVL_LAYER_SECURE_BUFFER! \n");
        #endif
    }

    if(pOvlConfig->layer_en!=0)
    {
        if(pOvlConfig->security!=OVL_LAYER_SECURE_BUFFER)
        {
            #ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
            if (gOvlSecure == 1)
            {
				//[8127] if the layered frame is secured but the layer is not, we have to sync the non-secured mva to secured page table
				//m4u_do_sync_nonsec_sec_pgtable(pOvlConfig->addr,unsigned int size, int port);
				
				MTEEC_PARAM param[4];
                unsigned int paramTypes;
                TZ_RESULT ret;
                KREE_SHAREDMEM_HANDLE p_ovl_config_layer_share_handle = 0;
                KREE_SHAREDMEM_PARAM  sharedParam;
                //p_ovl_config_layer_share_handle = disp_register_share_memory(pOvlConfig, sizeof(OVL_CONFIG_STRUCT));
                // Register shared memory
                sharedParam.buffer = pOvlConfig;
                sharedParam.size = sizeof(OVL_CONFIG_STRUCT);
                ret = KREE_RegisterSharedmem (ddp_mem_session_handle(), &p_ovl_config_layer_share_handle, &sharedParam);
                if (ret != TZ_RESULT_SUCCESS)
                {
                    DISP_ERR("disp_register_share_memory Error: %d, line:%d, ddp_mem_session(%x)", ret, __LINE__, (unsigned int)ddp_mem_session_handle());
                    return 0;
                }

                param[0].memref.handle = (uint32_t) p_ovl_config_layer_share_handle;
                param[0].memref.offset = 0;
                param[0].memref.size = sizeof(OVL_CONFIG_STRUCT);
                param[1].value.a = pOvlConfig->security; // wether the display buffer is a secured one
                paramTypes = TZ_ParamTypes2(TZPT_MEMREF_INPUT, TZPT_VALUE_INPUT);
                DISP_MSG("config_layer handle=0x%x \n", param[0].memref.handle);

                ret = KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_OVL_LAYER_CONFIG, paramTypes, param);
                if(ret!= TZ_RESULT_SUCCESS)
                {
                    DISP_ERR("TZCMD_DDP_OVL_LAYER_CONFIG fail, ret=%d \n", ret);
                }

                ret = KREE_UnregisterSharedmem(ddp_mem_session_handle(), p_ovl_config_layer_share_handle);
                if (ret)
                {
                    DISP_ERR("UREE_UnregisterSharedmem p_ovl_config_layer_share_handle Error: %d, line:%d, ddp_mem_session(%x)", ret, __LINE__, (unsigned int)ddp_mem_session_handle());
                }
            }
            else
            {
            #endif
            OVLLayerConfig(pOvlConfig->layer,   // layer
                       pOvlConfig->source,   // data source (0=memory)
                       pOvlConfig->fmt,
                       pOvlConfig->addr, // addr
                       pOvlConfig->src_x,  // x
                       pOvlConfig->src_y,  // y
                       pOvlConfig->src_pitch, //pitch, pixel number
                       pOvlConfig->dst_x,  // x
                       pOvlConfig->dst_y,  // y
                       pOvlConfig->dst_w, // width
                       pOvlConfig->dst_h, // height
                       pOvlConfig->keyEn,  //color key
                       pOvlConfig->key,  //color key
                       pOvlConfig->aen, // alpha enable
                       pOvlConfig->alpha); // alpha
                       
            #ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
            }
            #endif
            gOvlLayerSecure[pOvlConfig->layer] = 0;
        }
        else
        {
            #ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
            MTEEC_PARAM param[4];
            unsigned int paramTypes;
            TZ_RESULT ret;
            KREE_SHAREDMEM_HANDLE p_ovl_config_layer_share_handle = 0;
            KREE_SHAREDMEM_PARAM  sharedParam;
            //p_ovl_config_layer_share_handle = disp_register_share_memory(pOvlConfig, sizeof(OVL_CONFIG_STRUCT));
            // Register shared memory
            sharedParam.buffer = pOvlConfig;
            sharedParam.size = sizeof(OVL_CONFIG_STRUCT);
            ret = KREE_RegisterSharedmem (ddp_mem_session_handle(), &p_ovl_config_layer_share_handle, &sharedParam);
            if (ret != TZ_RESULT_SUCCESS)
            {
              DISP_ERR("disp_register_share_memory Error: %d, line:%d, ddp_mem_session(%x)", ret, __LINE__, (unsigned int)ddp_mem_session_handle());
              return 0;
            }

            param[0].memref.handle = (uint32_t) p_ovl_config_layer_share_handle;
            param[0].memref.offset = 0;
            param[0].memref.size = sizeof(OVL_CONFIG_STRUCT);    
            param[1].value.a = pOvlConfig->security;
            paramTypes = TZ_ParamTypes2(TZPT_MEMREF_INPUT, TZPT_VALUE_INPUT);
            DISP_MSG("config_layer handle=0x%x \n", param[0].memref.handle);

            ret = KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_OVL_LAYER_CONFIG, paramTypes, param);
            if(ret!= TZ_RESULT_SUCCESS)
            {
              DISP_ERR("TZCMD_DDP_OVL_LAYER_CONFIG fail, ret=%d \n", ret);
            }

            ret = KREE_UnregisterSharedmem(ddp_mem_session_handle(), p_ovl_config_layer_share_handle); 
            if (ret)
            {
              DISP_ERR("UREE_UnregisterSharedmem p_ovl_config_layer_share_handle Error: %d, line:%d, ddp_mem_session(%x)", ret, __LINE__, (unsigned int)ddp_mem_session_handle());
            }
            #else
            DISP_ERR("error, do not support security==OVL_LAYER_SECURE_BUFFER! \n");
            #endif
            gOvlLayerSecure[pOvlConfig->layer] = 1;
        }
          
        #ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
        // then according the result above, we know if the OVL module should be in secure protection
        // every time the {gOvlSecure} determined by the status of 4 layers:
        //   if one or more layer is set to be secure, then the OVL shall be in secure state.
        //   if all layers are not secured, {gOvlSecure} is set to 0.
        if(gOvlSecure != (gOvlLayerSecure[0]|gOvlLayerSecure[1]|gOvlLayerSecure[2]|gOvlLayerSecure[3]))
        {
            gOvlSecure = gOvlLayerSecure[0]|gOvlLayerSecure[1]|gOvlLayerSecure[2]|gOvlLayerSecure[3];
            disp_register_intr(MT6582_DISP_OVL_IRQ_ID, gOvlSecure);

            if(pOvlConfig->security!=OVL_LAYER_SECURE_BUFFER)
            {
                if (gOvlSecureTag == 1)
                {
                    gOvlSecureTag = 0; // clear the tag
                }
                else
                {
                    gOvlSecure = 0;
                }
            }
            else
            {
                if (gOvlSecureTag == 0)
                    gOvlSecureTag = 1; // the tag shall be set
            }

            // we use {gOvlSecureLast} to record the last secure status of OVL
            if ((int)gOvlSecure != gOvlSecureLast)
            {
                gOvlSecureLast = (int)gOvlSecure;
                updateSecureOvl(gOvlSecure); // and change DAPC protection level
            }
            DISP_MSG("[DAPC] gOvlSecure=%d, gOvlSecureLast=%d, < %d, %d, %d, %d \n",
                    gOvlSecure, gOvlSecureLast, gOvlLayerSecure[0], gOvlLayerSecure[1], gOvlLayerSecure[2], gOvlLayerSecure[3]);
        }

        // config each layer to secure mode
		#if 0  //this is duplicated config so is removed by monica 
		{
            // if the secure setting of each layer changed
            if(gOvlLayerSecure[pOvlConfig->layer] != gOvlLayerSecureLast[pOvlConfig->layer])
            {
                MTEEC_PARAM param[4];
                unsigned int paramTypes;
                TZ_RESULT ret;
                gOvlLayerSecureLast[pOvlConfig->layer] = gOvlLayerSecure[pOvlConfig->layer];
                param[0].value.a = DISP_OVL_0 + pOvlConfig->layer;
                param[1].value.a = gOvlLayerSecure[pOvlConfig->layer];
                paramTypes = TZ_ParamTypes2(TZPT_VALUE_INPUT,TZPT_VALUE_INPUT);
                ret = KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_SET_SECURE_MODE, paramTypes, param);
                if(ret!= TZ_RESULT_SUCCESS)
                {
                    DISP_ERR("KREE_TeeServiceCall(TZCMD_DDP_SET_SECURE_MODE) fail, ret=%d \n", ret);
                }
            }
        }    
		#endif
        #endif

    }
    else
    {
        OVLLayerConfig(pOvlConfig->layer,   // layer
                       OVL_LAYER_SOURCE_MEM,   // data source (0=memory)
                       eRGB888, //fmt
                       1, // addr 
                       0,  // x
                       0,  // y
                       0, //pitch, pixel number
                       0,  // x
                       0,  // y
                       0, // width
                       0, // height
                       0,  //color key
                       0,  //color key
                       0, // alpha enable
                       0); // alpha
    }

    return 0;
}

#endif

int disp_path_config_layer_addr(unsigned int layer, unsigned int addr)
{
    unsigned int reg_addr;

    DISP_DBG("[DDP]disp_path_config_layer_addr(), layer=%d, addr=0x%x\n ", layer, addr);

    if(gOvlSecure == 0) // secure is 0
    {
        switch(layer)
        {
            case 0:
                DISP_REG_SET(DISP_REG_OVL_L0_ADDR, addr);
                reg_addr = DISP_REG_OVL_L0_ADDR;
                break;
            case 1:
                DISP_REG_SET(DISP_REG_OVL_L1_ADDR, addr);
                reg_addr = DISP_REG_OVL_L1_ADDR;
                break;
            case 2:
                DISP_REG_SET(DISP_REG_OVL_L2_ADDR, addr);
                reg_addr = DISP_REG_OVL_L2_ADDR;
                break;
            case 3:
                DISP_REG_SET(DISP_REG_OVL_L3_ADDR, addr);
                reg_addr = DISP_REG_OVL_L3_ADDR;
                break;
            default:
                DISP_ERR("unknow layer=%d \n", layer);
        }
    }
    else
    {
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
        MTEEC_PARAM param[4];
        unsigned int paramTypes;
        TZ_RESULT ret;
        
        param[0].value.a = layer;
        param[1].value.a = addr;
        paramTypes = TZ_ParamTypes3(TZPT_VALUE_INPUT,TZPT_VALUE_INPUT,TZPT_VALUE_OUTPUT);
        ret = KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_OVL_CONFIG_LAYER_ADDR,paramTypes, param);
        if(ret!= TZ_RESULT_SUCCESS)
        {
            DISP_ERR("KREE_TeeServiceCall(TZCMD_DDP_OVL_CONFIG_LAYER_ADDR) fail, ret=%d \n", ret);
        }
        else
        {
            DISP_DBG("disp_path_config_layer_addr, reg=0x%x \n", param[2].value.a);
        }
#else
        DISP_ERR("error, do not support security==OVL_LAYER_SECURE_BUFFER! \n");
#endif        
    }
   
    return 0;
}

void _disp_path_wdma_callback(unsigned int param)
{
    mem_out_done = 1;
    wake_up_interruptible(&mem_out_wq);
}

void disp_path_wait_mem_out_done(void)
{
    int ret = 0;

    ret = wait_event_interruptible_timeout(
                    mem_out_wq, 
                    mem_out_done, 
                    disp_ms2jiffies(100) );
                    

    if(ret==0) // timeout
    {
        DISP_ERR("disp_path_wait_mem_out_done timeout \n");
        ddp_dump_info(DISP_MODULE_CONFIG); 
        ddp_dump_info(DISP_MODULE_MUTEX); 
        ddp_dump_info(DISP_MODULE_OVL); 
        ddp_dump_info(DISP_MODULE_WDMA); 

        disp_dump_reg(DISP_MODULE_OVL);
        disp_dump_reg(DISP_MODULE_WDMA); 

        // disp_path_ovl_reset();
        // WDMAReset(0);
    }
    else if(ret<0) // intr by a signal
    {
        DISP_ERR("disp_path_wait_mem_out_done intr by a signal ret=%d \n", ret);
    }
    
    mem_out_done = 0;    
}

// for video mode, if there are more than one frame between memory_out done and disable WDMA
// mem_out_done will be set to 1, next time user trigger disp_path_wait_mem_out_done() will return 
// directly, in such case, screen capture will dose not work for one time. 
// so we add this func to make sure disp_path_wait_mem_out_done() will be execute everytime.
void disp_path_clear_mem_out_done_flag(void)
{
    mem_out_done = 0;    
}

int  disp_path_query()
{
    return 0;
}


// just mem->ovl->wdma0->mem, used in suspend mode screen capture
// have to call this function set pConfig->enable=0 to reset configuration
// should call clock_on()/clock_off() if use this function in suspend mode
int disp_path_config_mem_out_without_lcd(struct disp_path_config_mem_out_struct* pConfig)
{
    static unsigned int reg_mutex_mod;
    static unsigned int reg_mutex_sof;
    static unsigned int reg_mout;
    
    DISP_DBG(" disp_path_config_mem_out(), enable = %d, outFormat=%d, dstAddr=0x%x, ROI(%d,%d,%d,%d) \n",
            pConfig->enable,
            pConfig->outFormat,            
            pConfig->dstAddr,  
            pConfig->srcROI.x, 
            pConfig->srcROI.y, 
            pConfig->srcROI.width, 
            pConfig->srcROI.height);
            
    if(pConfig->enable==1 && pConfig->dstAddr==0)
    {
          DISP_ERR("pConfig->dstAddr==0! \n");
    }

    if(pConfig->enable==1)
    {
        mem_out_done = 0;
        disp_register_irq(DISP_MODULE_WDMA, _disp_path_wdma_callback);

        // config wdma0
        WDMAReset(0);
        WDMAConfig(0, 
                   WDMA_INPUT_FORMAT_ARGB, 
                   pConfig->srcROI.width,
                   pConfig->srcROI.height,
                   0,
                   0,
                   pConfig->srcROI.width, 
                   pConfig->srcROI.height, 
                   pConfig->outFormat, 
                   pConfig->dstAddr, 
                   pConfig->srcROI.width,
                   1, 
                   0);      
        WDMAStart(0);

        // mutex module
        reg_mutex_mod = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(gMutexID));
        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(gMutexID), (1<<3)|(1<<6)); //ovl, wdma0

        // mutex sof
        reg_mutex_sof = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_SOF(gMutexID));
        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_SOF(gMutexID), 0); //single mode
                
        // ovl mout
        reg_mout = DISP_REG_GET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN);
        DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, 0x2);   // ovl_mout output to wdma0
        
        //disp_dump_reg(DISP_MODULE_WDMA);
    }
    else
    {
        // mutex
        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(gMutexID), reg_mutex_mod);
        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_SOF(gMutexID), reg_mutex_sof);         
        // ovl mout
        DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, reg_mout);        

        disp_unregister_irq(DISP_MODULE_WDMA, _disp_path_wdma_callback);
    }

    return 0;
}
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
// add wdma0 into the path
// should call get_mutex() / release_mutex for this func
int disp_path_config_mem_out(struct disp_path_config_mem_out_struct* pConfig){
	disp_path_config_mem_out_(pConfig, 0);
	return 0;
}
int disp_path_config_mem_out_(struct disp_path_config_mem_out_struct* pConfig, int OvlSecure)
{
    unsigned int reg;
#if 1    
    DISP_MSG(" disp_path_config_mem_out(), enable = %d, outFormat=%d, dstAddr=0x%x, ROI(%d,%d,%d,%d) \n",
            pConfig->enable,
            pConfig->outFormat,            
            pConfig->dstAddr,  
            pConfig->srcROI.x, 
            pConfig->srcROI.y, 
            pConfig->srcROI.width, 
            pConfig->srcROI.height);
#endif
    if(pConfig->enable==1 && pConfig->dstAddr==0)
    {
          DISP_ERR("pConfig->dstAddr==0! \n");
    }

    if(pConfig->enable==1)
    {
        OVLROI(pConfig->srcROI.width, // width
            pConfig->srcROI.height, // height
            0);// background B
            
        mem_out_done = 0;
        disp_register_irq(DISP_MODULE_WDMA, _disp_path_wdma_callback);
        // config wdma0
		//modify for switch; copy from disp_path_config_OVL_WDMA
		if(OvlSecure == 1)
		{
			MTEEC_PARAM param[4];
			unsigned int paramTypes;
			TZ_RESULT ret;
			KREE_SHAREDMEM_HANDLE p_wdma_config_share_handle = 0;
			KREE_SHAREDMEM_PARAM  sharedParam;

			// Register shared memory
			sharedParam.buffer = pConfig;
			sharedParam.size = sizeof(struct disp_path_config_mem_out_struct);
			ret = KREE_RegisterSharedmem (ddp_mem_session_handle(), &p_wdma_config_share_handle, &sharedParam);
			if (ret != TZ_RESULT_SUCCESS)
			{
				DISP_ERR("disp_register_share_memory Error: %d, line:%d, ddp_mem_session(%d)", ret, __LINE__, (unsigned int)ddp_mem_session_handle());
				return 0;
			}
			
			param[0].memref.handle = (uint32_t) p_wdma_config_share_handle;
			param[0].memref.offset = 0;
			param[0].memref.size = sizeof(struct disp_path_config_mem_out_struct);
			param[1].value.a = pConfig->security;
			//if (!bMemOutEnabled) //it was not mem out last time  8127 always resets and restarts wdma
			//{
				param[2].value.a = 1;   //if do wdmaRest and wdmaStart
			//}
			//else
			//{
			//	param[2].value.a = 0;	//no need to reset again
			//}	
			paramTypes = TZ_ParamTypes3(TZPT_MEMREF_INPUT, TZPT_VALUE_INPUT, TZPT_VALUE_INPUT);
			DISP_MSG("[memory_out]wdma config handle=0x%x ifReset=%d\n", param[0].memref.handle, param[2].value.a);
			
			ret = KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_WDMA_CONFIG, paramTypes, param);
# if 0  //todo for secured-wfd
			if(pConfig->outFormat==eYUV_420_3P)
			{
				WDMAConfigUV(0, 
				pConfig->dstAddr+pConfig->srcROI.width*pConfig->srcROI.height, 
				pConfig->dstAddr+pConfig->srcROI.width*pConfig->srcROI.height*5/4, 
				pConfig->srcROI.width);
			} 
#endif
			if(ret!= TZ_RESULT_SUCCESS)
			{
				DISP_ERR("TZCMD_DDP_WDMA_CONFIG fail, ret=%d \n", ret);
			}
			
			ret = KREE_UnregisterSharedmem(ddp_mem_session_handle(), p_wdma_config_share_handle); 
			if (ret)
			{
				DISP_ERR("UREE_UnregisterSharedmem p_wdma_config_share_handle Error: %d, line:%d, ddp_mem_session(%d)", ret, __LINE__, (unsigned int)ddp_mem_session_handle());
			}		
		}
		else
		{
        WDMAReset(0);
        WDMAConfig(0, 
                   WDMA_INPUT_FORMAT_ARGB, 
                   pConfig->srcROI.width,
                   pConfig->srcROI.height,
                   0,
                   0,
                   pConfig->srcROI.width, 
                   pConfig->srcROI.height, 
                   pConfig->outFormat, 
                   pConfig->dstAddr, 
                   pConfig->srcROI.width,
                   1, 
                   0);      
					if(pConfig->outFormat==eYUV_420_3P)					
					{
						WDMAConfigUV(0, 
						pConfig->dstAddr+pConfig->srcROI.width*pConfig->srcROI.height, 
						pConfig->dstAddr+pConfig->srcROI.width*pConfig->srcROI.height*5/4, 
						pConfig->srcROI.width);
					} 
			        WDMAStart(0);
		}
        // mutex
        //reg = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(gMutexID));
        //DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(gMutexID), reg|(1<<6)); //wdma0=6
        
        // ovl mout
        //reg = DISP_REG_GET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN);
        //DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, reg|(0x2));   // ovl_mout output to rdma & wdma
        
        //disp_dump_reg(DISP_MODULE_WDMA);
    }
    else
    {
        // mutex
        //reg = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(gMutexID));
        //DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(gMutexID), reg&(~(1<<6))); //wdma0=6
        
        // ovl mout
        //reg = DISP_REG_GET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN);
        //DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, reg&(~0x2));   // ovl_mout output to bls
        
        // config wdma0
        //WDMAReset(0);
        disp_unregister_irq(DISP_MODULE_WDMA, _disp_path_wdma_callback);
    }

    return 0;
}
#else
// add wdma0 into the path
// should call get_mutex() / release_mutex for this func
int disp_path_config_mem_out(struct disp_path_config_mem_out_struct* pConfig)
{
    unsigned int reg;
#if 1    
    DISP_MSG(" disp_path_config_mem_out(), enable = %d, outFormat=%d, dstAddr=0x%x, ROI(%d,%d,%d,%d) \n",
            pConfig->enable,
            pConfig->outFormat,            
            pConfig->dstAddr,  
            pConfig->srcROI.x, 
            pConfig->srcROI.y, 
            pConfig->srcROI.width, 
            pConfig->srcROI.height);
#endif
    if(pConfig->enable==1 && pConfig->dstAddr==0)
    {
          DISP_ERR("pConfig->dstAddr==0! \n");
    }

    if(pConfig->enable==1)
    {
        mem_out_done = 0;
        disp_register_irq(DISP_MODULE_WDMA, _disp_path_wdma_callback);

        // config wdma0
        WDMAReset(0);
        WDMAConfig(0, 
                   WDMA_INPUT_FORMAT_ARGB, 
                   pConfig->srcROI.width,
                   pConfig->srcROI.height,
                   0,
                   0,
                   pConfig->srcROI.width, 
                   pConfig->srcROI.height, 
                   pConfig->outFormat, 
                   pConfig->dstAddr, 
                   pConfig->srcROI.width,
                   1, 
                   0);      
		if(pConfig->outFormat==eYUV_420_3P)
		{
			WDMAConfigUV(0, 
			pConfig->dstAddr+pConfig->srcROI.width*pConfig->srcROI.height, 
			pConfig->dstAddr+pConfig->srcROI.width*pConfig->srcROI.height*5/4, 
			pConfig->srcROI.width);
		} 
        WDMAStart(0);

        // mutex
        reg = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(gMutexID));
        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(gMutexID), reg|(1<<6)); //wdma0=6
        
        // ovl mout
        reg = DISP_REG_GET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN);
        DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, reg|(0x2));   // ovl_mout output to rdma & wdma
        
        //disp_dump_reg(DISP_MODULE_WDMA);
    }
    else
    {
        // mutex
        reg = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_MOD(gMutexID));
        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(gMutexID), reg&(~(1<<6))); //wdma0=6
        
        // ovl mout
        reg = DISP_REG_GET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN);
        DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, reg&(~0x2));   // ovl_mout output to bls
        
        // config wdma0
        //WDMAReset(0);
        disp_unregister_irq(DISP_MODULE_WDMA, _disp_path_wdma_callback);
    }

    return 0;
}
#endif

int disp_path_config(struct disp_path_config_struct* pConfig)
{
	fb_width = pConfig->srcROI.width;
	fb_height = pConfig->srcROI.height;
    return disp_path_config_(pConfig, gMutexID);
}

DISP_MODULE_ENUM g_dst_module;


#ifdef DDP_USE_CLOCK_API
unsigned int reg_offset = 0;
// #define DDP_RECORD_REG_BACKUP_RESTORE  // print the reg value before backup and after restore
void reg_backup(unsigned int reg_addr)
{
   *(pRegBackup+reg_offset) = DISP_REG_GET(reg_addr);
#ifdef DDP_RECORD_REG_BACKUP_RESTORE
      printk("0x%08x(0x%08x), ", reg_addr, *(pRegBackup+reg_offset));
      if((reg_offset+1)%8==0)
          printk("\n");
#endif      
      reg_offset++;
      if(reg_offset>=DDP_BACKUP_REG_NUM)
      {
          DISP_ERR("reg_backup fail, reg_offset=%d, regBackupSize=%d \n", reg_offset, DDP_BACKUP_REG_NUM);        
      }
}

void reg_restore(unsigned int reg_addr)
{
      DISP_REG_SET(reg_addr, *(pRegBackup+reg_offset));
#ifdef DDP_RECORD_REG_BACKUP_RESTORE
      printk("0x%08x(0x%08x), ", reg_addr, DISP_REG_GET(reg_addr));
      if((reg_offset+1)%8==0)
          printk("\n");
#endif                
      reg_offset++;
      
      if(reg_offset>=DDP_BACKUP_REG_NUM)
      {
          DISP_ERR("reg_restore fail, reg_offset=%d, regBackupSize=%d \n", reg_offset, DDP_BACKUP_REG_NUM);
      }
}

unsigned int g_reg_backup[200];
static void disp_reg_backup_module(DISP_MODULE_ENUM module)
{
    unsigned int index = 0;
    if(module==DISP_MODULE_OVL)
    {    
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_STA                     );           
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_INTEN                   ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_INTSTA                  ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_EN                      ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_TRIG                    ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RST                     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_ROI_SIZE                ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_DATAPATH_CON            ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_ROI_BGCLR               ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_SRC_CON                 ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L0_CON                  ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L0_SRCKEY               ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L0_SRC_SIZE             ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L0_OFFSET               ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L0_ADDR                 ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L0_PITCH                ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L1_CON                  ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L1_SRCKEY               ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L1_SRC_SIZE             ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L1_OFFSET               ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L1_ADDR                 ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L1_PITCH                ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L2_CON                  ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L2_SRCKEY               ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L2_SRC_SIZE             ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L2_OFFSET               ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L2_ADDR                 ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L2_PITCH                ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L3_CON                  ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L3_SRCKEY               ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L3_SRC_SIZE             ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L3_OFFSET               ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L3_ADDR                 ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L3_PITCH                ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA0_CTRL              ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA0_MEM_START_TRIG    ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA0_MEM_GMC_SETTING   ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA0_MEM_SLOW_CON      ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA0_FIFO_CTRL         ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA1_CTRL              ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA1_MEM_START_TRIG    ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA1_MEM_GMC_SETTING   ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA1_MEM_SLOW_CON      ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA1_FIFO_CTRL         ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA2_CTRL              ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA2_MEM_START_TRIG    ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA2_MEM_GMC_SETTING   ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA2_MEM_SLOW_CON      ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA2_FIFO_CTRL         ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA3_CTRL              ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA3_MEM_START_TRIG    ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA3_MEM_GMC_SETTING   ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA3_MEM_SLOW_CON      ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA3_FIFO_CTRL         ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L0_Y2R_PARA_R0          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L0_Y2R_PARA_R1          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L0_Y2R_PARA_G0          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L0_Y2R_PARA_G1          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L0_Y2R_PARA_B0          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L0_Y2R_PARA_B1          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L0_Y2R_PARA_YUV_A_0     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L0_Y2R_PARA_YUV_A_1     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L0_Y2R_PARA_RGB_A_0     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L0_Y2R_PARA_RGB_A_1     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L1_Y2R_PARA_R0          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L1_Y2R_PARA_R1          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L1_Y2R_PARA_G0          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L1_Y2R_PARA_G1          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L1_Y2R_PARA_B0          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L1_Y2R_PARA_B1          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L1_Y2R_PARA_YUV_A_0     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L1_Y2R_PARA_YUV_A_1     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L1_Y2R_PARA_RGB_A_0     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L1_Y2R_PARA_RGB_A_1     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L2_Y2R_PARA_R0          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L2_Y2R_PARA_R1          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L2_Y2R_PARA_G0          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L2_Y2R_PARA_G1          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L2_Y2R_PARA_B0          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L2_Y2R_PARA_B1          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L2_Y2R_PARA_YUV_A_0     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L2_Y2R_PARA_YUV_A_1     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L2_Y2R_PARA_RGB_A_0     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L2_Y2R_PARA_RGB_A_1     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L3_Y2R_PARA_R0          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L3_Y2R_PARA_R1          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L3_Y2R_PARA_G0          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L3_Y2R_PARA_G1          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L3_Y2R_PARA_B0          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L3_Y2R_PARA_B1          ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L3_Y2R_PARA_YUV_A_0     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L3_Y2R_PARA_YUV_A_1     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L3_Y2R_PARA_RGB_A_0     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_L3_Y2R_PARA_RGB_A_1     ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_DEBUG_MON_SEL           ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA0_MEM_GMC_SETTING2  ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA1_MEM_GMC_SETTING2  ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA2_MEM_GMC_SETTING2  ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_RDMA3_MEM_GMC_SETTING2  ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_FLOW_CTRL_DBG           ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_ADDCON_DBG              ); 
        g_reg_backup[index++] = DISP_REG_GET(DISP_REG_OVL_OUTMUX_DBG              ); 

    }
}

static void disp_reg_restore_module(DISP_MODULE_ENUM module)
{
    unsigned int index = 0;
    if(module==DISP_MODULE_OVL)
    {   
        DISP_REG_SET(DISP_REG_OVL_STA                     , g_reg_backup[index++]);           
        DISP_REG_SET(DISP_REG_OVL_INTEN                   , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_INTSTA                  , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_EN                      , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_TRIG                    , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RST                     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_ROI_SIZE                , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_DATAPATH_CON            , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_ROI_BGCLR               , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_SRC_CON                 , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L0_CON                  , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L0_SRCKEY               , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L0_SRC_SIZE             , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L0_OFFSET               , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L0_ADDR                 , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L0_PITCH                , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L1_CON                  , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L1_SRCKEY               , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L1_SRC_SIZE             , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L1_OFFSET               , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L1_ADDR                 , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L1_PITCH                , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L2_CON                  , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L2_SRCKEY               , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L2_SRC_SIZE             , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L2_OFFSET               , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L2_ADDR                 , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L2_PITCH                , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L3_CON                  , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L3_SRCKEY               , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L3_SRC_SIZE             , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L3_OFFSET               , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L3_ADDR                 , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L3_PITCH                , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA0_CTRL              , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA0_MEM_START_TRIG    , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA0_MEM_GMC_SETTING   , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA0_MEM_SLOW_CON      , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA0_FIFO_CTRL         , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA1_CTRL              , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA1_MEM_START_TRIG    , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA1_MEM_GMC_SETTING   , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA1_MEM_SLOW_CON      , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA1_FIFO_CTRL         , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA2_CTRL              , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA2_MEM_START_TRIG    , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA2_MEM_GMC_SETTING   , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA2_MEM_SLOW_CON      , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA2_FIFO_CTRL         , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA3_CTRL              , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA3_MEM_START_TRIG    , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA3_MEM_GMC_SETTING   , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA3_MEM_SLOW_CON      , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA3_FIFO_CTRL         , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L0_Y2R_PARA_R0          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L0_Y2R_PARA_R1          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L0_Y2R_PARA_G0          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L0_Y2R_PARA_G1          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L0_Y2R_PARA_B0          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L0_Y2R_PARA_B1          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L0_Y2R_PARA_YUV_A_0     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L0_Y2R_PARA_YUV_A_1     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L0_Y2R_PARA_RGB_A_0     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L0_Y2R_PARA_RGB_A_1     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L1_Y2R_PARA_R0          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L1_Y2R_PARA_R1          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L1_Y2R_PARA_G0          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L1_Y2R_PARA_G1          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L1_Y2R_PARA_B0          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L1_Y2R_PARA_B1          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L1_Y2R_PARA_YUV_A_0     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L1_Y2R_PARA_YUV_A_1     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L1_Y2R_PARA_RGB_A_0     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L1_Y2R_PARA_RGB_A_1     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L2_Y2R_PARA_R0          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L2_Y2R_PARA_R1          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L2_Y2R_PARA_G0          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L2_Y2R_PARA_G1          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L2_Y2R_PARA_B0          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L2_Y2R_PARA_B1          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L2_Y2R_PARA_YUV_A_0     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L2_Y2R_PARA_YUV_A_1     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L2_Y2R_PARA_RGB_A_0     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L2_Y2R_PARA_RGB_A_1     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L3_Y2R_PARA_R0          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L3_Y2R_PARA_R1          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L3_Y2R_PARA_G0          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L3_Y2R_PARA_G1          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L3_Y2R_PARA_B0          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L3_Y2R_PARA_B1          , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L3_Y2R_PARA_YUV_A_0     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L3_Y2R_PARA_YUV_A_1     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L3_Y2R_PARA_RGB_A_0     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_L3_Y2R_PARA_RGB_A_1     , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_DEBUG_MON_SEL           , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA0_MEM_GMC_SETTING2  , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA1_MEM_GMC_SETTING2  , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA2_MEM_GMC_SETTING2  , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_RDMA3_MEM_GMC_SETTING2  , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_FLOW_CTRL_DBG           , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_ADDCON_DBG              , g_reg_backup[index++]); 
        DISP_REG_SET(DISP_REG_OVL_OUTMUX_DBG              , g_reg_backup[index++]); 

    }

}


static int disp_mutex_backup(void)
{
    int i;

    reg_backup(DISP_REG_CONFIG_MUTEX_INTEN);
    reg_backup(DISP_REG_CONFIG_REG_UPD_TIMEOUT);

    for (i=0; i<6; i++)
    {
    	reg_backup(DISP_REG_CONFIG_MUTEX_EN(i));
    	reg_backup(DISP_REG_CONFIG_MUTEX_MOD(i));
    	reg_backup(DISP_REG_CONFIG_MUTEX_SOF(i));
    }

    return 0;
}

#if 0
static int disp_bls_backup()
{
    int i;

    reg_backup(DISP_REG_BLS_DEBUG);
    reg_backup(DISP_REG_BLS_PWM_DUTY);

    reg_backup(DISP_REG_BLS_BLS_SETTING);
    reg_backup(DISP_REG_BLS_SRC_SIZE);
    reg_backup(DISP_REG_BLS_GAMMA_SETTING);
    reg_backup(DISP_REG_BLS_GAMMA_BOUNDARY);

    /* BLS Luminance LUT */
    for (i=0; i<=32; i++)
    {
    	reg_backup(DISP_REG_BLS_LUMINANCE(i));
    }

    /* BLS Luminance 255 */
    reg_backup(DISP_REG_BLS_LUMINANCE_255);

    /* Dither */
    for (i=0; i<18; i++)
    {
    	reg_backup(DISP_REG_BLS_DITHER(i));
    }

    reg_backup(DISP_REG_BLS_INTEN);
    reg_backup(DISP_REG_BLS_EN);

    return 0;
}
#endif

static int disp_mutex_restore(void)
{
    int i;

    reg_restore(DISP_REG_CONFIG_MUTEX_INTEN);
    reg_restore(DISP_REG_CONFIG_REG_UPD_TIMEOUT);

    for (i=0; i<6; i++)
    {
    	reg_restore(DISP_REG_CONFIG_MUTEX_EN(i));
        reg_restore(DISP_REG_CONFIG_MUTEX_MOD(i));
        reg_restore(DISP_REG_CONFIG_MUTEX_SOF(i));
    }

    return 0;
}

#if 0
static int disp_bls_restore()
{
    int i;

    reg_restore(DISP_REG_BLS_DEBUG);
    reg_restore(DISP_REG_BLS_PWM_DUTY);

    reg_restore(DISP_REG_BLS_BLS_SETTING);
    reg_restore(DISP_REG_BLS_SRC_SIZE);
    reg_restore(DISP_REG_BLS_GAMMA_SETTING);
    reg_restore(DISP_REG_BLS_GAMMA_BOUNDARY);

    /* BLS Luminance LUT */
    for (i=0; i<=32; i++)
    {
    	reg_restore(DISP_REG_BLS_LUMINANCE(i));
    }

    /* BLS Luminance 255 */
    reg_restore(DISP_REG_BLS_LUMINANCE_255);

    /* Dither */
    for (i=0; i<18; i++)
    {
    	reg_restore(DISP_REG_BLS_DITHER(i));
    }

    reg_restore(DISP_REG_BLS_INTEN);
    reg_restore(DISP_REG_BLS_EN);

    return 0;
}
#endif

static int disp_reg_backup(void)
{
	unsigned int index;
	reg_offset = 0;
	DISP_MSG("disp_reg_backup() start, *pRegBackup=0x%x, reg_offset=%d  \n", *pRegBackup, reg_offset);

	// Config
	reg_backup(DISP_REG_CONFIG_DISP_OVL_MOUT_EN );
	reg_backup(DISP_REG_CONFIG_DISP_OUT_SEL     );

	// backup dpi1 from bls for 8127 hdmi main path
	#if MTK_HDMI_MAIN_PATH
	reg_backup(DISP_REG_CONFIG_DPI1_SEL     );
	#endif
	
	// Mutex & BLS
	disp_mutex_backup();
	// BLS
	//disp_bls_backup();

	// OVL
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
    // When backup OVL registers, consider secure case
    if (0 == gOvlSecure)
    {
#endif // CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
	reg_backup(DISP_REG_OVL_STA                     );
	reg_backup(DISP_REG_OVL_INTEN                   );
	reg_backup(DISP_REG_OVL_INTSTA                  );
	reg_backup(DISP_REG_OVL_EN                      );
	reg_backup(DISP_REG_OVL_TRIG                    );
	reg_backup(DISP_REG_OVL_RST                     );
	reg_backup(DISP_REG_OVL_ROI_SIZE                );
	reg_backup(DISP_REG_OVL_DATAPATH_CON            );
	reg_backup(DISP_REG_OVL_ROI_BGCLR               );
	reg_backup(DISP_REG_OVL_SRC_CON                 );
	reg_backup(DISP_REG_OVL_L0_CON                  );
	reg_backup(DISP_REG_OVL_L0_SRCKEY               );
	reg_backup(DISP_REG_OVL_L0_SRC_SIZE             );
	reg_backup(DISP_REG_OVL_L0_OFFSET               );
	reg_backup(DISP_REG_OVL_L0_ADDR                 );
	reg_backup(DISP_REG_OVL_L0_PITCH                );
	reg_backup(DISP_REG_OVL_L1_CON                  );
	reg_backup(DISP_REG_OVL_L1_SRCKEY               );
	reg_backup(DISP_REG_OVL_L1_SRC_SIZE             );
	reg_backup(DISP_REG_OVL_L1_OFFSET               );
	reg_backup(DISP_REG_OVL_L1_ADDR                 );
	reg_backup(DISP_REG_OVL_L1_PITCH                );
	reg_backup(DISP_REG_OVL_L2_CON                  );
	reg_backup(DISP_REG_OVL_L2_SRCKEY               );
	reg_backup(DISP_REG_OVL_L2_SRC_SIZE             );
	reg_backup(DISP_REG_OVL_L2_OFFSET               );
	reg_backup(DISP_REG_OVL_L2_ADDR                 );
	reg_backup(DISP_REG_OVL_L2_PITCH                );
	reg_backup(DISP_REG_OVL_L3_CON                  );
	reg_backup(DISP_REG_OVL_L3_SRCKEY               );
	reg_backup(DISP_REG_OVL_L3_SRC_SIZE             );
	reg_backup(DISP_REG_OVL_L3_OFFSET               );
	reg_backup(DISP_REG_OVL_L3_ADDR                 );
	reg_backup(DISP_REG_OVL_L3_PITCH                );
	reg_backup(DISP_REG_OVL_RDMA0_CTRL              );
	reg_backup(DISP_REG_OVL_RDMA0_MEM_START_TRIG    );
	reg_backup(DISP_REG_OVL_RDMA0_MEM_GMC_SETTING   );
	reg_backup(DISP_REG_OVL_RDMA0_MEM_SLOW_CON      );
	reg_backup(DISP_REG_OVL_RDMA0_FIFO_CTRL         );
	reg_backup(DISP_REG_OVL_RDMA1_CTRL              );
	reg_backup(DISP_REG_OVL_RDMA1_MEM_START_TRIG    );
	reg_backup(DISP_REG_OVL_RDMA1_MEM_GMC_SETTING   );
	reg_backup(DISP_REG_OVL_RDMA1_MEM_SLOW_CON      );
	reg_backup(DISP_REG_OVL_RDMA1_FIFO_CTRL         );
	reg_backup(DISP_REG_OVL_RDMA2_CTRL              );
	reg_backup(DISP_REG_OVL_RDMA2_MEM_START_TRIG    );
	reg_backup(DISP_REG_OVL_RDMA2_MEM_GMC_SETTING   );
	reg_backup(DISP_REG_OVL_RDMA2_MEM_SLOW_CON      );
	reg_backup(DISP_REG_OVL_RDMA2_FIFO_CTRL         );
	reg_backup(DISP_REG_OVL_RDMA3_CTRL              );
	reg_backup(DISP_REG_OVL_RDMA3_MEM_START_TRIG    );
	reg_backup(DISP_REG_OVL_RDMA3_MEM_GMC_SETTING   );
	reg_backup(DISP_REG_OVL_RDMA3_MEM_SLOW_CON      );
	reg_backup(DISP_REG_OVL_RDMA3_FIFO_CTRL         );
	reg_backup(DISP_REG_OVL_L0_Y2R_PARA_R0          );
	reg_backup(DISP_REG_OVL_L0_Y2R_PARA_R1          );
	reg_backup(DISP_REG_OVL_L0_Y2R_PARA_G0          );
	reg_backup(DISP_REG_OVL_L0_Y2R_PARA_G1          );
	reg_backup(DISP_REG_OVL_L0_Y2R_PARA_B0          );
	reg_backup(DISP_REG_OVL_L0_Y2R_PARA_B1          );
	reg_backup(DISP_REG_OVL_L0_Y2R_PARA_YUV_A_0     );
	reg_backup(DISP_REG_OVL_L0_Y2R_PARA_YUV_A_1     );
	reg_backup(DISP_REG_OVL_L0_Y2R_PARA_RGB_A_0     );
	reg_backup(DISP_REG_OVL_L0_Y2R_PARA_RGB_A_1     );
	reg_backup(DISP_REG_OVL_L1_Y2R_PARA_R0          );
	reg_backup(DISP_REG_OVL_L1_Y2R_PARA_R1          );
	reg_backup(DISP_REG_OVL_L1_Y2R_PARA_G0          );
	reg_backup(DISP_REG_OVL_L1_Y2R_PARA_G1          );
	reg_backup(DISP_REG_OVL_L1_Y2R_PARA_B0          );
	reg_backup(DISP_REG_OVL_L1_Y2R_PARA_B1          );
	reg_backup(DISP_REG_OVL_L1_Y2R_PARA_YUV_A_0     );
	reg_backup(DISP_REG_OVL_L1_Y2R_PARA_YUV_A_1     );
	reg_backup(DISP_REG_OVL_L1_Y2R_PARA_RGB_A_0     );
	reg_backup(DISP_REG_OVL_L1_Y2R_PARA_RGB_A_1     );
	reg_backup(DISP_REG_OVL_L2_Y2R_PARA_R0          );
	reg_backup(DISP_REG_OVL_L2_Y2R_PARA_R1          );
	reg_backup(DISP_REG_OVL_L2_Y2R_PARA_G0          );
	reg_backup(DISP_REG_OVL_L2_Y2R_PARA_G1          );
	reg_backup(DISP_REG_OVL_L2_Y2R_PARA_B0          );
	reg_backup(DISP_REG_OVL_L2_Y2R_PARA_B1          );
	reg_backup(DISP_REG_OVL_L2_Y2R_PARA_YUV_A_0     );
	reg_backup(DISP_REG_OVL_L2_Y2R_PARA_YUV_A_1     );
	reg_backup(DISP_REG_OVL_L2_Y2R_PARA_RGB_A_0     );
	reg_backup(DISP_REG_OVL_L2_Y2R_PARA_RGB_A_1     );
	reg_backup(DISP_REG_OVL_L3_Y2R_PARA_R0          );
	reg_backup(DISP_REG_OVL_L3_Y2R_PARA_R1          );
	reg_backup(DISP_REG_OVL_L3_Y2R_PARA_G0          );
	reg_backup(DISP_REG_OVL_L3_Y2R_PARA_G1          );
	reg_backup(DISP_REG_OVL_L3_Y2R_PARA_B0          );
	reg_backup(DISP_REG_OVL_L3_Y2R_PARA_B1          );
	reg_backup(DISP_REG_OVL_L3_Y2R_PARA_YUV_A_0     );
	reg_backup(DISP_REG_OVL_L3_Y2R_PARA_YUV_A_1     );
	reg_backup(DISP_REG_OVL_L3_Y2R_PARA_RGB_A_0     );
	reg_backup(DISP_REG_OVL_L3_Y2R_PARA_RGB_A_1     );
	reg_backup(DISP_REG_OVL_DEBUG_MON_SEL           );
	reg_backup(DISP_REG_OVL_RDMA0_MEM_GMC_SETTING2  );
	reg_backup(DISP_REG_OVL_RDMA1_MEM_GMC_SETTING2  );
	reg_backup(DISP_REG_OVL_RDMA2_MEM_GMC_SETTING2  );
	reg_backup(DISP_REG_OVL_RDMA3_MEM_GMC_SETTING2  );
	reg_backup(DISP_REG_OVL_FLOW_CTRL_DBG           );
	reg_backup(DISP_REG_OVL_ADDCON_DBG              );
	reg_backup(DISP_REG_OVL_OUTMUX_DBG              );

#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
    }
	else
	{
        MTEEC_PARAM param[4];
        unsigned int paramTypes = 0;
        TZ_RESULT ret =
            KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_OVL_BACKUP_REG, paramTypes, param);
        if (ret != TZ_RESULT_SUCCESS)
        {
            DISP_ERR("KREE_TeeServiceCall(TZCMD_DDP_OVL_BACKUP_REG) fail, ret=%d \n", ret);
        }
	}
#endif // CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT

	// RDMA, RDMA1
	for (index = 0; index < 1; index++) {   //Donot back up RDMA1. It's only for HDMI using
		reg_backup(DISP_REG_RDMA_INT_ENABLE        + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_INT_STATUS        + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_GLOBAL_CON        + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_SIZE_CON_0        + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_SIZE_CON_1        + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_TARGET_LINE       + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_MEM_CON           + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_MEM_START_ADDR    + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_MEM_SRC_PITCH     + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_MEM_GMC_SETTING_0 + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_MEM_SLOW_CON      + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_MEM_GMC_SETTING_1 + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_FIFO_CON          + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_FIFO_LOG          + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_CF_00             + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_CF_01             + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_CF_02             + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_CF_10             + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_CF_11             + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_CF_12             + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_CF_20             + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_CF_21             + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_CF_22             + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_CF_PRE_ADD0       + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_CF_PRE_ADD1       + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_CF_PRE_ADD2       + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_CF_POST_ADD0      + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_CF_POST_ADD1      + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_CF_POST_ADD2      + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_DUMMY             + index*DISP_INDEX_OFFSET);
		reg_backup(DISP_REG_RDMA_DEBUG_OUT_SEL     + index*DISP_INDEX_OFFSET);
	}
	// WDMA    
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
    // when backup WDMA 1 registers, consider secure case
	//pr_err("gOvlLayerSecure[0]=%d, gOvlLayerSecure[1]=%d, gOvlLayerSecure[2]=%d, gOvlLayerSecure[3]=%d",gOvlLayerSecure[0],gOvlLayerSecure[1],gOvlLayerSecure[2],gOvlLayerSecure[3]);
    //if (0 == gMemOutSecure)    
	if (0 == (gOvlLayerSecure[0]|gOvlLayerSecure[1]|gOvlLayerSecure[2]|gOvlLayerSecure[3]))
    {
#endif // CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
	reg_backup(DISP_REG_WDMA_INTEN          );
	reg_backup(DISP_REG_WDMA_INTSTA         );
	reg_backup(DISP_REG_WDMA_EN             );
	reg_backup(DISP_REG_WDMA_RST            );
	reg_backup(DISP_REG_WDMA_SMI_CON        );
	reg_backup(DISP_REG_WDMA_CFG            );
	reg_backup(DISP_REG_WDMA_SRC_SIZE       );
	reg_backup(DISP_REG_WDMA_CLIP_SIZE      );
	reg_backup(DISP_REG_WDMA_CLIP_COORD     );
	reg_backup(DISP_REG_WDMA_DST_ADDR       );
	reg_backup(DISP_REG_WDMA_DST_W_IN_BYTE  );
	reg_backup(DISP_REG_WDMA_ALPHA          );
	reg_backup(DISP_REG_WDMA_BUF_ADDR       );
	reg_backup(DISP_REG_WDMA_STA            );
	reg_backup(DISP_REG_WDMA_BUF_CON1       );
	reg_backup(DISP_REG_WDMA_BUF_CON2       );
	reg_backup(DISP_REG_WDMA_C00            );
	reg_backup(DISP_REG_WDMA_C02            );
	reg_backup(DISP_REG_WDMA_C10            );
	reg_backup(DISP_REG_WDMA_C12            );
	reg_backup(DISP_REG_WDMA_C20            );
	reg_backup(DISP_REG_WDMA_C22            );
	reg_backup(DISP_REG_WDMA_PRE_ADD0       );
	reg_backup(DISP_REG_WDMA_PRE_ADD2       );
	reg_backup(DISP_REG_WDMA_POST_ADD0      );
	reg_backup(DISP_REG_WDMA_POST_ADD2      );
	reg_backup(DISP_REG_WDMA_DST_U_ADDR     );
	reg_backup(DISP_REG_WDMA_DST_V_ADDR     );
	reg_backup(DISP_REG_WDMA_DST_UV_PITCH   );
	reg_backup(DISP_REG_WDMA_DITHER_CON     );
	reg_backup(DISP_REG_WDMA_FLOW_CTRL_DBG  );
	reg_backup(DISP_REG_WDMA_EXEC_DBG       );
	reg_backup(DISP_REG_WDMA_CLIP_DBG       );
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
    }
    else
    {
		//pr_err("[secured] disp_reg_backup gMemOutSecure=%d gOvlSecure=%d", gMemOutSecure, gOvlSecure);
		MTEEC_PARAM param[4];
        unsigned int paramTypes = 0;
        TZ_RESULT ret =
            KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_WDMA_BACKUP_REG, paramTypes, param);
        if (ret != TZ_RESULT_SUCCESS)
        {
            DISP_ERR("KREE_TeeServiceCall(TZCMD_DDP_WDMA_BACKUP_REG) fail, ret=%d \n", ret);
        }
    }
#endif // CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT

	DISP_MSG("disp_reg_backup() end, *pRegBackup=0x%x, reg_offset=%d \n", *pRegBackup, reg_offset);

	return 0;
}

static int disp_reg_restore(void)
{
	unsigned int index;
    reg_offset = 0;
    DISP_MSG("disp_reg_restore(*) start, *pRegBackup=0x%x, reg_offset=%d  \n", *pRegBackup, reg_offset);

    // Config
    reg_restore(DISP_REG_CONFIG_DISP_OVL_MOUT_EN );     
    reg_restore(DISP_REG_CONFIG_DISP_OUT_SEL     );     

	// restore dpi1 from bls for 8127 hdmi main path
    #if MTK_HDMI_MAIN_PATH
	reg_restore(DISP_REG_CONFIG_DPI1_SEL     );
    #endif
    
    // Mutex
    disp_mutex_restore();
    // BLS
    //disp_bls_restore();

    // OVL
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
    // when restoring OVL registers, consider secure case
    if (0 == gOvlSecure)
    {
#endif // CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
    reg_restore(DISP_REG_OVL_STA                     );           
    reg_restore(DISP_REG_OVL_INTEN                   ); 
    reg_restore(DISP_REG_OVL_INTSTA                  ); 
    reg_restore(DISP_REG_OVL_EN                      ); 
    reg_restore(DISP_REG_OVL_TRIG                    ); 
    reg_restore(DISP_REG_OVL_RST                     ); 
    reg_restore(DISP_REG_OVL_ROI_SIZE                ); 
    reg_restore(DISP_REG_OVL_DATAPATH_CON            ); 
    reg_restore(DISP_REG_OVL_ROI_BGCLR               ); 
    reg_restore(DISP_REG_OVL_SRC_CON                 ); 
    reg_restore(DISP_REG_OVL_L0_CON                  ); 
    reg_restore(DISP_REG_OVL_L0_SRCKEY               ); 
    reg_restore(DISP_REG_OVL_L0_SRC_SIZE             ); 
    reg_restore(DISP_REG_OVL_L0_OFFSET               ); 
    reg_restore(DISP_REG_OVL_L0_ADDR                 ); 
    reg_restore(DISP_REG_OVL_L0_PITCH                ); 
    reg_restore(DISP_REG_OVL_L1_CON                  ); 
    reg_restore(DISP_REG_OVL_L1_SRCKEY               ); 
    reg_restore(DISP_REG_OVL_L1_SRC_SIZE             ); 
    reg_restore(DISP_REG_OVL_L1_OFFSET               ); 
    reg_restore(DISP_REG_OVL_L1_ADDR                 ); 
    reg_restore(DISP_REG_OVL_L1_PITCH                ); 
    reg_restore(DISP_REG_OVL_L2_CON                  ); 
    reg_restore(DISP_REG_OVL_L2_SRCKEY               ); 
    reg_restore(DISP_REG_OVL_L2_SRC_SIZE             ); 
    reg_restore(DISP_REG_OVL_L2_OFFSET               ); 
    reg_restore(DISP_REG_OVL_L2_ADDR                 ); 
    reg_restore(DISP_REG_OVL_L2_PITCH                ); 
    reg_restore(DISP_REG_OVL_L3_CON                  ); 
    reg_restore(DISP_REG_OVL_L3_SRCKEY               ); 
    reg_restore(DISP_REG_OVL_L3_SRC_SIZE             ); 
    reg_restore(DISP_REG_OVL_L3_OFFSET               ); 
    reg_restore(DISP_REG_OVL_L3_ADDR                 ); 
    reg_restore(DISP_REG_OVL_L3_PITCH                ); 
    reg_restore(DISP_REG_OVL_RDMA0_CTRL              ); 
    reg_restore(DISP_REG_OVL_RDMA0_MEM_START_TRIG    ); 
    reg_restore(DISP_REG_OVL_RDMA0_MEM_GMC_SETTING   ); 
    reg_restore(DISP_REG_OVL_RDMA0_MEM_SLOW_CON      ); 
    reg_restore(DISP_REG_OVL_RDMA0_FIFO_CTRL         ); 
    reg_restore(DISP_REG_OVL_RDMA1_CTRL              ); 
    reg_restore(DISP_REG_OVL_RDMA1_MEM_START_TRIG    ); 
    reg_restore(DISP_REG_OVL_RDMA1_MEM_GMC_SETTING   ); 
    reg_restore(DISP_REG_OVL_RDMA1_MEM_SLOW_CON      ); 
    reg_restore(DISP_REG_OVL_RDMA1_FIFO_CTRL         ); 
    reg_restore(DISP_REG_OVL_RDMA2_CTRL              ); 
    reg_restore(DISP_REG_OVL_RDMA2_MEM_START_TRIG    ); 
    reg_restore(DISP_REG_OVL_RDMA2_MEM_GMC_SETTING   ); 
    reg_restore(DISP_REG_OVL_RDMA2_MEM_SLOW_CON      ); 
    reg_restore(DISP_REG_OVL_RDMA2_FIFO_CTRL         ); 
    reg_restore(DISP_REG_OVL_RDMA3_CTRL              ); 
    reg_restore(DISP_REG_OVL_RDMA3_MEM_START_TRIG    ); 
    reg_restore(DISP_REG_OVL_RDMA3_MEM_GMC_SETTING   ); 
    reg_restore(DISP_REG_OVL_RDMA3_MEM_SLOW_CON      ); 
    reg_restore(DISP_REG_OVL_RDMA3_FIFO_CTRL         ); 
    reg_restore(DISP_REG_OVL_L0_Y2R_PARA_R0          ); 
    reg_restore(DISP_REG_OVL_L0_Y2R_PARA_R1          ); 
    reg_restore(DISP_REG_OVL_L0_Y2R_PARA_G0          ); 
    reg_restore(DISP_REG_OVL_L0_Y2R_PARA_G1          ); 
    reg_restore(DISP_REG_OVL_L0_Y2R_PARA_B0          ); 
    reg_restore(DISP_REG_OVL_L0_Y2R_PARA_B1          ); 
    reg_restore(DISP_REG_OVL_L0_Y2R_PARA_YUV_A_0     ); 
    reg_restore(DISP_REG_OVL_L0_Y2R_PARA_YUV_A_1     ); 
    reg_restore(DISP_REG_OVL_L0_Y2R_PARA_RGB_A_0     ); 
    reg_restore(DISP_REG_OVL_L0_Y2R_PARA_RGB_A_1     ); 
    reg_restore(DISP_REG_OVL_L1_Y2R_PARA_R0          ); 
    reg_restore(DISP_REG_OVL_L1_Y2R_PARA_R1          ); 
    reg_restore(DISP_REG_OVL_L1_Y2R_PARA_G0          ); 
    reg_restore(DISP_REG_OVL_L1_Y2R_PARA_G1          ); 
    reg_restore(DISP_REG_OVL_L1_Y2R_PARA_B0          ); 
    reg_restore(DISP_REG_OVL_L1_Y2R_PARA_B1          ); 
    reg_restore(DISP_REG_OVL_L1_Y2R_PARA_YUV_A_0     ); 
    reg_restore(DISP_REG_OVL_L1_Y2R_PARA_YUV_A_1     ); 
    reg_restore(DISP_REG_OVL_L1_Y2R_PARA_RGB_A_0     ); 
    reg_restore(DISP_REG_OVL_L1_Y2R_PARA_RGB_A_1     ); 
    reg_restore(DISP_REG_OVL_L2_Y2R_PARA_R0          ); 
    reg_restore(DISP_REG_OVL_L2_Y2R_PARA_R1          ); 
    reg_restore(DISP_REG_OVL_L2_Y2R_PARA_G0          ); 
    reg_restore(DISP_REG_OVL_L2_Y2R_PARA_G1          ); 
    reg_restore(DISP_REG_OVL_L2_Y2R_PARA_B0          ); 
    reg_restore(DISP_REG_OVL_L2_Y2R_PARA_B1          ); 
    reg_restore(DISP_REG_OVL_L2_Y2R_PARA_YUV_A_0     ); 
    reg_restore(DISP_REG_OVL_L2_Y2R_PARA_YUV_A_1     ); 
    reg_restore(DISP_REG_OVL_L2_Y2R_PARA_RGB_A_0     ); 
    reg_restore(DISP_REG_OVL_L2_Y2R_PARA_RGB_A_1     ); 
    reg_restore(DISP_REG_OVL_L3_Y2R_PARA_R0          ); 
    reg_restore(DISP_REG_OVL_L3_Y2R_PARA_R1          ); 
    reg_restore(DISP_REG_OVL_L3_Y2R_PARA_G0          ); 
    reg_restore(DISP_REG_OVL_L3_Y2R_PARA_G1          ); 
    reg_restore(DISP_REG_OVL_L3_Y2R_PARA_B0          ); 
    reg_restore(DISP_REG_OVL_L3_Y2R_PARA_B1          ); 
    reg_restore(DISP_REG_OVL_L3_Y2R_PARA_YUV_A_0     ); 
    reg_restore(DISP_REG_OVL_L3_Y2R_PARA_YUV_A_1     ); 
    reg_restore(DISP_REG_OVL_L3_Y2R_PARA_RGB_A_0     ); 
    reg_restore(DISP_REG_OVL_L3_Y2R_PARA_RGB_A_1     ); 
    reg_restore(DISP_REG_OVL_DEBUG_MON_SEL           ); 
    reg_restore(DISP_REG_OVL_RDMA0_MEM_GMC_SETTING2  ); 
    reg_restore(DISP_REG_OVL_RDMA1_MEM_GMC_SETTING2  ); 
    reg_restore(DISP_REG_OVL_RDMA2_MEM_GMC_SETTING2  ); 
    reg_restore(DISP_REG_OVL_RDMA3_MEM_GMC_SETTING2  ); 
    reg_restore(DISP_REG_OVL_FLOW_CTRL_DBG           ); 
    reg_restore(DISP_REG_OVL_ADDCON_DBG              ); 
    reg_restore(DISP_REG_OVL_OUTMUX_DBG              ); 
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
    }
    else
    {
        MTEEC_PARAM param[4];
        unsigned int paramTypes = 0;
        TZ_RESULT ret =
            KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_OVL_RESTORE_REG, paramTypes, param);
        if (ret != TZ_RESULT_SUCCESS)
        {
            DISP_ERR("KREE_TeeServiceCall(TZCMD_DDP_OVL_RESTORE_REG) fail, ret=%d \n", ret);
        }
    }
#endif // CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT

    
    // RDMA, RDMA1
    for (index = 0; index < 1; index++) {   //Donot restore RDMA1. It's only for HDMI using.
    	reg_restore(DISP_REG_RDMA_INT_ENABLE        + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_INT_STATUS        + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_GLOBAL_CON        + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_SIZE_CON_0        + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_SIZE_CON_1        + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_TARGET_LINE       + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_MEM_CON           + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_MEM_START_ADDR    + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_MEM_SRC_PITCH     + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_MEM_GMC_SETTING_0 + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_MEM_SLOW_CON      + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_MEM_GMC_SETTING_1 + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_FIFO_CON          + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_FIFO_LOG          + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_CF_00             + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_CF_01             + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_CF_02             + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_CF_10             + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_CF_11             + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_CF_12             + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_CF_20             + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_CF_21             + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_CF_22             + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_CF_PRE_ADD0       + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_CF_PRE_ADD1       + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_CF_PRE_ADD2       + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_CF_POST_ADD0      + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_CF_POST_ADD1      + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_CF_POST_ADD2      + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_DUMMY             + index*DISP_INDEX_OFFSET);
    	reg_restore(DISP_REG_RDMA_DEBUG_OUT_SEL     + index*DISP_INDEX_OFFSET);
    }
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
	//pr_err("disp_reg_resotre gMemOutSecure=%d", gMemOutSecure);
	// when restoring WDMA 1 registers, consider secure case
    //if (0 == gMemOutSecure)
    if (0 == (gOvlLayerSecure[0]|gOvlLayerSecure[1]|gOvlLayerSecure[2]|gOvlLayerSecure[3]))
    {
#endif // CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
    reg_restore(DISP_REG_WDMA_INTEN          );             
    reg_restore(DISP_REG_WDMA_INTSTA         );     
    reg_restore(DISP_REG_WDMA_EN             );     
    reg_restore(DISP_REG_WDMA_RST            );     
    reg_restore(DISP_REG_WDMA_SMI_CON        );     
    reg_restore(DISP_REG_WDMA_CFG            );     
    reg_restore(DISP_REG_WDMA_SRC_SIZE       );     
    reg_restore(DISP_REG_WDMA_CLIP_SIZE      );     
    reg_restore(DISP_REG_WDMA_CLIP_COORD     );     
    reg_restore(DISP_REG_WDMA_DST_ADDR       );     
    reg_restore(DISP_REG_WDMA_DST_W_IN_BYTE  );     
    reg_restore(DISP_REG_WDMA_ALPHA          );     
    reg_restore(DISP_REG_WDMA_BUF_ADDR       );     
    reg_restore(DISP_REG_WDMA_STA            );     
    reg_restore(DISP_REG_WDMA_BUF_CON1       );     
    reg_restore(DISP_REG_WDMA_BUF_CON2       );     
    reg_restore(DISP_REG_WDMA_C00            );     
    reg_restore(DISP_REG_WDMA_C02            );     
    reg_restore(DISP_REG_WDMA_C10            );     
    reg_restore(DISP_REG_WDMA_C12            );     
    reg_restore(DISP_REG_WDMA_C20            );     
    reg_restore(DISP_REG_WDMA_C22            );     
    reg_restore(DISP_REG_WDMA_PRE_ADD0       );     
    reg_restore(DISP_REG_WDMA_PRE_ADD2       );     
    reg_restore(DISP_REG_WDMA_POST_ADD0      );     
    reg_restore(DISP_REG_WDMA_POST_ADD2      );     
    reg_restore(DISP_REG_WDMA_DST_U_ADDR     );     
    reg_restore(DISP_REG_WDMA_DST_V_ADDR     );     
    reg_restore(DISP_REG_WDMA_DST_UV_PITCH   );     
    reg_restore(DISP_REG_WDMA_DITHER_CON     );     
    reg_restore(DISP_REG_WDMA_FLOW_CTRL_DBG  );     
    reg_restore(DISP_REG_WDMA_EXEC_DBG       );     
    reg_restore(DISP_REG_WDMA_CLIP_DBG       );     
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
    }
    else
    {
        MTEEC_PARAM param[4];
        unsigned int paramTypes = 0;
        TZ_RESULT ret =
            KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_WDMA_RESTORE_REG, paramTypes, param);
        if (ret != TZ_RESULT_SUCCESS)
        {
            DISP_ERR("KREE_TeeServiceCall(TZCMD_DDP_WDMA_RESTORE_REG) fail, ret=%d \n", ret);
        }
    }
#endif // CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT

    //DISP_MSG("disp_reg_restore() release mutex \n");
    //disp_path_release_mutex();
    DISP_MSG("disp_reg_restore() done \n");
    
    disp_bls_init(DISP_GetScreenWidth(),DISP_GetScreenHeight());

    DpEngine_COLORonInit();
    DpEngine_COLORonConfig(DISP_GetScreenWidth(),DISP_GetScreenHeight());

    return 0;        
}

unsigned int disp_intr_status[DISP_MODULE_MAX] = {0};
int disp_intr_restore(void)
{
    // restore intr enable reg 
    //DISP_REG_SET(DISP_REG_ROT_INTERRUPT_ENABLE,   disp_intr_status[DISP_MODULE_ROT]  );  
    //DISP_REG_SET(DISP_REG_SCL_INTEN,              disp_intr_status[DISP_MODULE_SCL]  ); 
    DISP_REG_SET(DISP_REG_OVL_INTEN,              disp_intr_status[DISP_MODULE_OVL]  ); 
    //DISP_REG_SET(DISP_REG_WDMA_INTEN,             disp_intr_status[DISP_MODULE_WDMA]); 
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
	if (0 == (gOvlLayerSecure[0]|gOvlLayerSecure[1]|gOvlLayerSecure[2]|gOvlLayerSecure[3]))
    {
#endif
    	DISP_REG_SET(DISP_REG_WDMA_INTEN,      disp_intr_status[DISP_MODULE_WDMA]);
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
    }
	else
	{
		MTEEC_PARAM param[4];
		unsigned int paramTypes;
		TZ_RESULT ret;
		param[0].value.a = 1;  //operation: 0 backup; 1 restore; 2 set
		paramTypes = TZ_ParamTypes2(TZPT_VALUE_INPUT,TZPT_VALUE_INPUT);
		ret = KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_WDMA_INTEN, paramTypes, param);
		if(ret!= TZ_RESULT_SUCCESS)
		{
			DISP_ERR("TZCMD_DDP_WDMA_INTEN fail, ret=%d \n", ret);
		}
	}
#endif

	DISP_REG_SET(DISP_REG_RDMA_INT_ENABLE,        disp_intr_status[DISP_MODULE_RDMA0]); 
    //DISP_REG_SET(DISP_REG_RDMA_INT_ENABLE+0xa000, disp_intr_status[DISP_MODULE_RDMA1]); 
    DISP_REG_SET(DISP_REG_CONFIG_MUTEX_INTEN,     disp_intr_status[DISP_MODULE_MUTEX]); 

    disp_register_rdma1_irq();

    return 0;
}

// TODO: color, tdshp, gamma, bls, cmdq intr management should add later
int disp_intr_disable_and_clear(void)
{
    // backup intr enable reg
    //disp_intr_status[DISP_MODULE_ROT] = DISP_REG_GET(DISP_REG_ROT_INTERRUPT_ENABLE);
    //disp_intr_status[DISP_MODULE_SCL] = DISP_REG_GET(DISP_REG_SCL_INTEN);
    disp_intr_status[DISP_MODULE_OVL] = DISP_REG_GET(DISP_REG_OVL_INTEN);
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
    if (0 == (gOvlLayerSecure[0]|gOvlLayerSecure[1]|gOvlLayerSecure[2]|gOvlLayerSecure[3]))
#endif
	{
		disp_intr_status[DISP_MODULE_WDMA] = DISP_REG_GET(DISP_REG_WDMA_INTEN);
	}
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
	else
	{
		MTEEC_PARAM param[4];
		unsigned int paramTypes;
		TZ_RESULT ret;

		param[0].value.a = 0;  //operation: 0 backup; 1 restore; 2 set
		paramTypes = TZ_ParamTypes2(TZPT_VALUE_INPUT,TZPT_VALUE_INPUT);
		ret = KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_WDMA_INTEN, paramTypes, param);
		if(ret!= TZ_RESULT_SUCCESS)
		{
			DISP_ERR("TZCMD_DDP_WDMA_INTEN fail, ret=%d \n", ret);
		}
	}
#endif
    //disp_intr_status[DISP_MODULE_WDMA1] = DISP_REG_GET(DISP_REG_WDMA_INTEN+0x1000);
    disp_intr_status[DISP_MODULE_RDMA0] = DISP_REG_GET(DISP_REG_RDMA_INT_ENABLE);
    //disp_intr_status[DISP_MODULE_RDMA1] = DISP_REG_GET(DISP_REG_RDMA_INT_ENABLE+0xa000);
    disp_intr_status[DISP_MODULE_MUTEX] = DISP_REG_GET(DISP_REG_CONFIG_MUTEX_INTEN);
    
    // disable intr
    //DISP_REG_SET(DISP_REG_ROT_INTERRUPT_ENABLE, 0);
    //DISP_REG_SET(DISP_REG_SCL_INTEN, 0);
    DISP_REG_SET(DISP_REG_OVL_INTEN, 0);
    //DISP_REG_SET(DISP_REG_WDMA_INTEN, 0);
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT    
	if (0 == (gOvlLayerSecure[0]|gOvlLayerSecure[1]|gOvlLayerSecure[2]|gOvlLayerSecure[3]))
    {
#endif
		DISP_REG_SET(DISP_REG_WDMA_INTEN, 0);  //+1000
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
    }
	else
	{
		MTEEC_PARAM param[4];
		unsigned int paramTypes;
		TZ_RESULT ret;
			param[0].value.a = 2;	//operation: 0 backup; 1 restore; 2 set
			param[1].value.a = 0;
			paramTypes = TZ_ParamTypes2(TZPT_VALUE_INPUT,TZPT_VALUE_INPUT);
			ret = KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_WDMA_INTEN, paramTypes, param);
		if(ret!= TZ_RESULT_SUCCESS)
		{
			DISP_ERR("TZCMD_DDP_WDMA_INTEN fail, ret=%d \n", ret);
		}
	}
#endif
    DISP_REG_SET(DISP_REG_RDMA_INT_ENABLE, 0);
    //DISP_REG_SET(DISP_REG_RDMA_INT_ENABLE+0x1000, 0);
    DISP_REG_SET(DISP_REG_CONFIG_MUTEX_INTEN, 0);
    //disable bls interrupt
    DISP_REG_SET(DISP_REG_BLS_INTEN, 0);
    
    // clear intr status
    //DISP_REG_SET(DISP_REG_ROT_INTERRUPT_STATUS, 0);  
    //DISP_REG_SET(DISP_REG_SCL_INTSTA, 0);               
    DISP_REG_SET(DISP_REG_OVL_INTSTA, 0);    
    //DISP_REG_SET(DISP_REG_WDMA_INTSTA, 0); 
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
    if (0 == (gOvlLayerSecure[0]|gOvlLayerSecure[1]|gOvlLayerSecure[2]|gOvlLayerSecure[3]))
    {
#endif
    	DISP_REG_SET(DISP_REG_WDMA_INTSTA, 0);  //+1000
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
	}
	else
	{
			MTEEC_PARAM param[4];
			unsigned int paramTypes;
			TZ_RESULT ret;
			
			param[0].value.a = 0;
			paramTypes = TZ_ParamTypes1(TZPT_VALUE_INPUT);			
			ret = KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_WDMA_INTSTA, paramTypes, param);
			if(ret!= TZ_RESULT_SUCCESS)
			{
				DISP_ERR("TZCMD_DDP_WDMA_INTSTA fail, ret=%d \n", ret);
			}
	}
#endif
    DISP_REG_SET(DISP_REG_RDMA_INT_STATUS, 0);    
    //DISP_REG_SET(DISP_REG_RDMA_INT_STATUS+0x1000, 0);            
    DISP_REG_SET(DISP_REG_CONFIG_MUTEX_INTSTA, 0);

    disp_unregister_rdma1_irq();
    //clear bls interrupt status	
    DISP_REG_SET(DISP_REG_BLS_INTSTA, 0);
    return 0;	  
}

/* for hdmi path */
int disp_rdma1_intr_restore(void)
{
    // restore intr enable reg
    DISP_REG_SET(DISP_REG_RDMA_INT_ENABLE+0xa000, disp_intr_status[DISP_MODULE_RDMA1]);

    return 0;
}

/* for hdmi path */
int disp_rdma1_intr_disable_and_clear(void)
{
    // backup intr enable reg
    disp_intr_status[DISP_MODULE_RDMA1] = DISP_REG_GET(DISP_REG_RDMA_INT_ENABLE+0xa000);
    DISP_REG_SET(DISP_REG_RDMA_INT_ENABLE+0xa000, 0);

    return 0;
}

int disp_path_clock_on(char* name)
{
    if(name != NULL)
    {
        DISP_MSG("disp_path_power_on, caller:%s \n", name);
    }

    enable_clock(MT_CG_DISP0_SMI_COMMON   , "DDP");
    DISP_MSG("%s %d\n", __FUNCTION__, __LINE__);
    enable_clock(MT_CG_DISP0_SMI_LARB0   , "DDP");
    DISP_MSG("%s %d\n", __FUNCTION__, __LINE__);
    //enable_clock(MT_CG_DISP0_MUTEX   , "DDP");
    enable_clock(MT_CG_DISP0_MUTEX_32K   , "DDP");
    //enable_clock(MT_CG_DISP0_MM_CMDQ , "DDP");
//    //enable_clock(MT_CG_DISP0_CMDQ_SMI    , "DDP");
    
//    enable_clock(MT_CG_DISP0_ROT_ENGINE  , "DDP");    
//    enable_clock(MT_CG_DISP0_ROT_SMI     , "DDP");
//    enable_clock(MT_CG_DISP0_SCL         , "DDP");
//    enable_clock(MT_CG_DISP0_DISP_WDMA, "DDP");
//    //enable_clock(MT_CG_DISP0_WDMA0_SMI   , "DDP");

    enable_clock(MT_CG_DISP0_DISP_OVL  , "DDP");
    //enable_clock(MT_CG_DISP0_OVL_SMI     , "DDP");
    enable_clock(MT_CG_DISP0_DISP_COLOR       , "DDP");
//    enable_clock(MT_CG_DISP0_2DSHP       , "DDP");
    enable_clock(MT_CG_DISP0_DISP_BLS         , "DDP");
    enable_clock(MT_CG_DISP0_DISP_WDMA, "DDP");
    //enable_clock(MT_CG_DISP0_WDMA0_SMI   , "DDP");
    //enable_clock(MT_CG_DISP0_RDMA0_ENGINE, "DDP");
    //enable_clock(MT_CG_DISP0_RDMA0_SMI   , "DDP");
    //enable_clock(MT_CG_DISP0_RDMA0_OUTPUT, "DDP");
    
    enable_clock(MT_CG_DISP0_DISP_RDMA, "DDP");
    //enable_clock(MT_CG_DISP0_DISP_RMDA1, "DDP");
    //enable_clock(MT_CG_DISP0_RDMA1_SMI   , "DDP");
    //enable_clock(MT_CG_DISP0_RDMA1_OUTPUT, "DDP");
    //enable_clock(MT_CG_DISP0_GAMMA_ENGINE, "DDP");
    //enable_clock(MT_CG_DISP0_GAMMA_PIXEL , "DDP");    

    //enable_clock(MT_CG_DISP0_G2D_ENGINE  , "DDP");
    //enable_clock(MT_CG_DISP0_G2D_SMI     , "DDP");

	
// Let BLS PWM clock on/off together with dispsy since backlight is configured by event merge thread
// Ennable BLS PWM clock before BLE_EN is set
//#if defined(MTK_AAL_SUPPORT) 
    enable_clock(MT_CG_DISP0_MDP_BLS_26M         , "DDP");
//#endif

    // restore ddp related registers
    if (strncmp(name, "ipoh_mtkfb", 10))
    {
    	if(*pRegBackup != DDP_UNBACKED_REG_MEM)
    	{
    		disp_reg_restore();

    		// restore intr enable registers
    		disp_intr_restore();
    	}
    	else
    	{
    		DISP_MSG("disp_path_clock_on(), dose not call disp_reg_restore, cause mem not inited! \n");
    	}
    }
    else
    {
        // reset AAL flags
        disp_aal_reset();
    }
    
    DISP_MSG("DISP CG:%x\n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0));
    return 0;
}


int disp_path_clock_off(char* name)
{
    if(name != NULL)
    {
        DISP_MSG("disp_path_power_off, caller:%s \n", name);
    }
    
    // disable intr and clear intr status
    disp_intr_disable_and_clear();
    
    // backup ddp related registers
    disp_reg_backup();

// Let BLS PWM clock on/off together with dispsy since backlight is configured by event merge thread
// Disable BLS PWM clock before other dispsys clock    
//#if defined(MTK_AAL_SUPPORT) 
    disable_clock(MT_CG_DISP0_MDP_BLS_26M         , "DDP");
//#endif

    //disable_clock(MT_CG_DISP0_MM_CMDQ , "DDP");
//    //disable_clock(MT_CG_DISP0_CMDQ_SMI    , "DDP");
    
//    disable_clock(MT_CG_DISP0_ROT_ENGINE  , "DDP");    
//    disable_clock(MT_CG_DISP0_ROT_SMI     , "DDP");
//    disable_clock(MT_CG_DISP0_SCL         , "DDP");
//    disable_clock(MT_CG_DISP0_DISP_WDMA, "DDP");
//    //disable_clock(MT_CG_DISP0_WDMA0_SMI   , "DDP");

    // Better to reset DMA engine before disable their clock
    RDMAStop(0);
    RDMAReset(0);
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
	if (0 == (gOvlLayerSecure[0]|gOvlLayerSecure[1]|gOvlLayerSecure[2]|gOvlLayerSecure[3]))
	{
#endif
    	WDMAStop(0);
    	WDMAReset(0);
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
	}
	else
	{
		MTEEC_PARAM param[4];
		unsigned int paramTypes;
		TZ_RESULT ret;
		
		param[0].value.a = 0;
		paramTypes = TZ_ParamTypes1(TZPT_VALUE_INPUT);			
		ret = KREE_TeeServiceCall(ddp_session_handle(),TZCMD_DDP_WDMA_STOP, paramTypes, param);
		if(ret!= TZ_RESULT_SUCCESS)
		{
			DISP_ERR("TZCMD_DDP_WDMA_STOP fail, ret=%d \n", ret);
		}

		param[0].value.a = 0;
		paramTypes = TZ_ParamTypes1(TZPT_VALUE_INPUT);			
		ret = KREE_TeeServiceCall(ddp_session_handle(),TZCMD_DDP_WDMA_RST, paramTypes, param);
		if(ret!= TZ_RESULT_SUCCESS)
		{
			DISP_ERR("TZCMD_DDP_WDMA_RST fail, ret=%d \n", ret);
		}
	}
#endif
    OVLStop();
    OVLReset();

    disable_clock(MT_CG_DISP0_DISP_OVL  , "DDP");
    //disable_clock(MT_CG_DISP0_OVL_SMI     , "DDP");
    disable_clock(MT_CG_DISP0_DISP_COLOR       , "DDP");
//    disable_clock(MT_CG_DISP0_2DSHP       , "DDP");
    disable_clock(MT_CG_DISP0_DISP_BLS         , "DDP");
    disable_clock(MT_CG_DISP0_DISP_WDMA, "DDP");
    //disable_clock(MT_CG_DISP0_WDMA0_SMI   , "DDP");
    //disable_clock(MT_CG_DISP0_RDMA0_ENGINE, "DDP");
    //disable_clock(MT_CG_DISP0_RDMA0_SMI   , "DDP");
    //disable_clock(MT_CG_DISP0_RDMA0_OUTPUT, "DDP");
    
    disable_clock(MT_CG_DISP0_DISP_RDMA, "DDP");
    //disable_clock(MT_CG_DISP0_DISP_RMDA1, "DDP");
    //disable_clock(MT_CG_DISP0_RDMA1_SMI   , "DDP");
    //disable_clock(MT_CG_DISP0_RDMA1_OUTPUT, "DDP");
    //disable_clock(MT_CG_DISP0_GAMMA_ENGINE, "DDP");
    //disable_clock(MT_CG_DISP0_GAMMA_PIXEL , "DDP");    

	if(clk_is_force_on(MT_CG_DISP0_SMI_LARB0))
	{
		pr_err("[DDP] clr MT_CG_DISP0_SMI_LARB0 is forced on\n");
		clk_clr_force_on(MT_CG_DISP0_SMI_LARB0);
	}
	//pr_err("1. smi clk=0x%x\n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0));

	if(clk_is_force_on(MT_CG_DISP0_SMI_COMMON))
	{
		pr_err("[DDP] clr MT_CG_DISP0_SMI_COMMON is forced on\n");
		clk_clr_force_on(MT_CG_DISP0_SMI_COMMON);
	}
    if(0) // if(g_dst_module==DISP_MODULE_DPI0)
    {
        DISP_MSG("warning: do not power off MT_CG_DISP0_SMI_LARB0 for DPI resume issue\n");
    }
    else
    {
        //disable_clock(MT_CG_DISP0_MUTEX   , "DDP");
        disable_clock(MT_CG_DISP0_MUTEX_32K   , "DDP");
        disable_clock(MT_CG_DISP0_SMI_LARB0   , "DDP");
        disable_clock(MT_CG_DISP0_SMI_COMMON   , "DDP");
    } 
    //disable_clock(MT_CG_DISP0_G2D_ENGINE  , "DDP");
    //disable_clock(MT_CG_DISP0_G2D_SMI     , "DDP");

    //DISP_MSG("DISP CG:%x\n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0));
    return 0;
}
#else

static int disp_path_clock_on(char* name)
{
    return 0;
}
static int disp_path_clock_off(char* name)
{
    return 0;
}

#endif

int disp_bls_set_max_backlight(unsigned int level)
{
  return disp_bls_set_max_backlight_(level);
}

int disp_hdmi_path_clock_on(char* name)
{
    return 0;
}

int disp_hdmi_path_clock_off(char* name)
{
    return 0;
}

#if defined(MTK_OVERLAY_ENGINE_SUPPORT)


void _disp_path_ovl_rdma_callback(unsigned int param)
{
    if(gdisp_path_register_ovl_rdma_callback != NULL)
        gdisp_path_register_ovl_rdma_callback(param);
}


void disp_path_register_ovl_rdma_callback(void (*callback)(unsigned int param),unsigned int param)
{
    gdisp_path_register_ovl_rdma_callback = callback;
}


void disp_path_unregister_ovl_rdma_callback(void (*callback)(unsigned int param),unsigned int param)
{
    gdisp_path_register_ovl_rdma_callback = NULL;
}

int disp_path_config_layer_(OVL_CONFIG_STRUCT* pOvlConfig,int OvlSecure)
{
//    unsigned int reg_addr;
	//log added temporarily for SQC
	/*if(pOvlConfig->layer == 0 || pOvlConfig->layer == 1){
	pr_err("[DDP] config_layer(), layer=%d, en=%d, addr start:0x%x, addr end:0x%x, size:0x%x, src(%d, %d, %d, %d), dst(%d, %d, %d, %d), pitch=%d, sec=%d, ovl_sec = %d\n ",
			pOvlConfig->layer,	 // layer
			pOvlConfig->layer_en,
			pOvlConfig->addr, // addr
			pOvlConfig->addr+pOvlConfig->src_pitch*pOvlConfig->src_h, // addr
			pOvlConfig->src_pitch*pOvlConfig->src_h,
			pOvlConfig->src_x,	// x
			pOvlConfig->src_y,	// y
			pOvlConfig->src_w,	// w
			pOvlConfig->src_h,	// h
			pOvlConfig->dst_x,	// x
			pOvlConfig->dst_y,	// y
			pOvlConfig->dst_w, // width
			pOvlConfig->dst_h, // height
			pOvlConfig->src_pitch, //pitch, pixel number
			pOvlConfig->security,
			OvlSecure);
	}*/
    /*pr_info("[DDP] config_layer(), layer=%d, en=%d, source=%d, fmt=%d, "
    "addr start:0x%x, addr end:0x%x, size:0x%x, src(%d, %d, %d, %d), dst(%d, %d, %d, %d), pitch=%d,"
    "keyEn=%d, key=%d, aen=%d, alpha=%d, isTdshp=%d, idx=%d, sec=%d, ovl_sec = %d\n ",
        pOvlConfig->layer,   // layer
        pOvlConfig->layer_en,
        pOvlConfig->source,   // data source (0=memory)
        pOvlConfig->fmt,
        pOvlConfig->addr, // addr
        pOvlConfig->addr+pOvlConfig->src_pitch*pOvlConfig->src_h, // addr
        pOvlConfig->src_pitch*pOvlConfig->src_h,
        pOvlConfig->src_x,  // x
        pOvlConfig->src_y,  // y
        pOvlConfig->src_w,  // w
        pOvlConfig->src_h,  // h
        pOvlConfig->dst_x,  // x
        pOvlConfig->dst_y,  // y
        pOvlConfig->dst_w, // width
        pOvlConfig->dst_h, // height
        pOvlConfig->src_pitch, //pitch, pixel number
        pOvlConfig->keyEn,  //color key
        pOvlConfig->key,  //color key
        pOvlConfig->aen, // alpha enable
        pOvlConfig->alpha,
        pOvlConfig->isTdshp,
        pOvlConfig->buff_idx,
        pOvlConfig->security,
        OvlSecure);*/

    // config overlay
    MMProfileLogEx(DDP_MMP_Events.Debug, MMProfileFlagPulse, pOvlConfig->layer, pOvlConfig->layer_en);

    if(pOvlConfig->security==OVL_LAYER_SECURE_BUFFER)
		gOvlLayerSecure[pOvlConfig->layer] = 1;
	else
		gOvlLayerSecure[pOvlConfig->layer] = 0;

#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
    // if OVL already in secure state (DAPC protected)
    if (OvlSecure == 1)
    {
        MTEEC_PARAM param[4];
        unsigned int paramTypes;
        TZ_RESULT ret;
        
        param[0].value.a = pOvlConfig->layer;
        param[1].value.a = pOvlConfig->layer_en;
        paramTypes = TZ_ParamTypes2(TZPT_VALUE_INPUT,TZPT_VALUE_INPUT);
        ret = KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_OVL_LAYER_SWITCH, paramTypes, param);
        if(ret!= TZ_RESULT_SUCCESS)
        {
            DISP_ERR("[DAPC] KREE_TeeServiceCall(TZCMD_DDP_OVL_LAYER_SWITCH) fail, ret=%d \n", ret);
        }

        // switch m4u port
        if (gOvlLayerSecure[0]|gOvlLayerSecure[1]|gOvlLayerSecure[2]|gOvlLayerSecure[3])
        {
            MMProfileLogEx(MTKFB_MMP_Events.OVL_WDMA_M4U_InTEE, MMProfileFlagStart, 0, 0);
        }
        else
        {
            MMProfileLogEx(MTKFB_MMP_Events.OVL_WDMA_M4U_InTEE, MMProfileFlagEnd, 0, 0);
        }
    }
    else
#endif
    {
        if(!gOvlLayerSecure[pOvlConfig->layer])
        {
            OVLLayerSwitch(pOvlConfig->layer, pOvlConfig->layer_en);
        }
        else
        {
			OVLLayerSwitch(pOvlConfig->layer, pOvlConfig->layer_en);
			DISP_ERR("ovl_sec = 0 LayerSecure[%d]=%d en%d\n",pOvlConfig->layer,gOvlLayerSecure[pOvlConfig->layer], pOvlConfig->layer_en);
        }
    }


    if(pOvlConfig->layer_en!=0)
    {
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
        if (OvlSecure == 1)
        {
        	//[8127] if the layered frame is secured but the layer is not, we have to sync the non-secured mva to secured page table
        	//if(gOvlLayerSecure[pOvlConfig->layer] == 0)
        	//{
        	//  m4u_do_sync_nonsec_sec_pgtable(pOvlConfig->addr,unsigned int size, int port);
        	//}
        	MTEEC_PARAM param[4];
            unsigned int paramTypes;
            TZ_RESULT ret;
            KREE_SHAREDMEM_HANDLE p_ovl_config_layer_share_handle = 0;
            KREE_SHAREDMEM_PARAM  sharedParam;
            //p_ovl_config_layer_share_handle = disp_register_share_memory(pOvlConfig, sizeof(OVL_CONFIG_STRUCT));
            // Register shared memory
            sharedParam.buffer = pOvlConfig;
            sharedParam.size = sizeof(OVL_CONFIG_STRUCT);
            ret = KREE_RegisterSharedmem (ddp_mem_session_handle(), &p_ovl_config_layer_share_handle, &sharedParam);
            if (ret != TZ_RESULT_SUCCESS)
            {
                DISP_ERR("disp_register_share_memory Error: %d, line:%d, ddp_mem_session(%d)", ret, __LINE__, (unsigned int)ddp_mem_session_handle());
                return 0;
            }
            
            param[0].memref.handle = (uint32_t) p_ovl_config_layer_share_handle;
            param[0].memref.offset = 0;
            param[0].memref.size = sizeof(OVL_CONFIG_STRUCT);    
            param[1].value.a = gOvlLayerSecure[pOvlConfig->layer]; // wether the display buffer is a secured one
            paramTypes = TZ_ParamTypes2(TZPT_MEMREF_INPUT, TZPT_VALUE_INPUT);
            DISP_DBG("config_layer handle=0x%x \n", param[0].memref.handle);
            
            ret = KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_OVL_LAYER_CONFIG, paramTypes, param);
            if(ret!= TZ_RESULT_SUCCESS)
            {
                DISP_ERR("TZCMD_DDP_OVL_LAYER_CONFIG fail, ret=%d \n", ret);
            }
            
            ret = KREE_UnregisterSharedmem(ddp_mem_session_handle(), p_ovl_config_layer_share_handle); 
            if (ret)
            {
                DISP_ERR("UREE_UnregisterSharedmem p_ovl_config_layer_share_handle Error: %d, line:%d, ddp_mem_session(%d)", ret, __LINE__, (unsigned int)ddp_mem_session_handle());
            }
        }
        else
#endif
        {
            if(!gOvlLayerSecure[pOvlConfig->layer])
            {
                MMProfileLogEx(DDP_MMP_Events.Debug, MMProfileFlagPulse, 
                    pOvlConfig->layer, pOvlConfig->layer_en);
                OVLLayerConfig(pOvlConfig->layer,   // layer
                           pOvlConfig->source,   // data source (0=memory)
                           pOvlConfig->fmt,
                           pOvlConfig->addr, // addr
                           pOvlConfig->src_x,  // x
                           pOvlConfig->src_y,  // y
                           pOvlConfig->src_pitch, //pitch, pixel number
                           pOvlConfig->dst_x,  // x
                           pOvlConfig->dst_y,  // y
                           pOvlConfig->dst_w, // width
                           pOvlConfig->dst_h, // height
                           pOvlConfig->keyEn,  //color key
                           pOvlConfig->key,  //color key
                           pOvlConfig->aen, // alpha enable
                           pOvlConfig->alpha); // alpha
                OVLStart();
            }
            else
            {
                DISP_ERR("error, do not support security==OVL_LAYER_SECURE_BUFFER! \n");
            }
        }
    }

    return 0;
}


int gOvlEngineControl = 0;
int disp_path_config_layer_ovl_engine_control(int enable)
{
    gOvlEngineControl = enable;
    return 0;
}


int disp_path_config_layer(OVL_CONFIG_STRUCT* pOvlConfig)
{
    if(gOvlEngineControl)
		return 0;
	
    return disp_path_config_layer_(pOvlConfig,0);
}


int disp_path_config_layer_ovl_engine(OVL_CONFIG_STRUCT* pOvlConfig,int OvlSecure)
{
    if(!gOvlEngineControl)
		return 0;

    //OVLStop();

    _DISP_DumpLayer(pOvlConfig);

    disp_path_config_layer_(pOvlConfig,OvlSecure);

    //OVLStart();

    return 0;
}


DECLARE_WAIT_QUEUE_HEAD(ovl_mem_out_wq);
static unsigned int ovl_mem_out_done = 0;


void disp_path_wait_ovl_wdma_callbak(unsigned int param)
{
    ovl_mem_out_done = 1;
    wake_up_interruptible(&ovl_mem_out_wq);
}

void disp_path_wait_ovl_wdma_done(void)
{
    int ret = 0;

    ret = wait_event_interruptible_timeout(
                    ovl_mem_out_wq,
                    ovl_mem_out_done,
                    disp_ms2jiffies(1000) );


    if(ret==0) // timeout
    {
        DISP_ERR("disp_path_wait_ovl_mem_out_done timeout \n");
        disp_dump_reg(DISP_MODULE_WDMA);
    }
    else if(ret<0) // intr by a signal
    {
        DISP_ERR("disp_path_wait_ovl_mem_out_done intr by a signal ret=%d \n", ret);
    }

    ovl_mem_out_done = 0;
}
void (*gdisp_path_register_ovl_wdma_callback)(unsigned int param) = NULL;


void _disp_path_ovl_wdma_callback(unsigned int param)
{   
    if(gdisp_path_register_ovl_wdma_callback != NULL)
		gdisp_path_register_ovl_wdma_callback(param);
}


void disp_path_register_ovl_wdma_callback(void (*callback)(unsigned int param),unsigned int param)
{
    gdisp_path_register_ovl_wdma_callback = callback;
}


void disp_path_unregister_ovl_wdma_callback(void (*callback)(unsigned int param),unsigned int param)
{
    gdisp_path_register_ovl_wdma_callback = NULL;
}


int disp_path_config_OVL_WDMA_path(int mutex_id)
{
#if 1
	// mutex module
    DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(mutex_id), 0x48); //ovl, wdma1

	// mutex sof
	DISP_REG_SET(DISP_REG_CONFIG_MUTEX_SOF(mutex_id), 0); //single mode

    // ovl mout
    DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, 1<<1);   // ovl_mout output to wdma1

#else
	// mutex module
	DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(mutex_id), 1<<2 | 1<<6 | 1<<9); //ovl, wdma1

	// mutex sof
	DISP_REG_SET(DISP_REG_CONFIG_MUTEX_SOF(mutex_id), 0); //single mode

    // ovl mout
    DISP_REG_SET(DISP_REG_CONFIG_OVL_MOUT_EN, 1<<1);   // ovl_mout output to BLS

	DISP_REG_SET(DISP_REG_CONFIG_BLS_SEL, 0);		  // bls_sel from ovl

    /////bypass BLS
	DISP_REG_SET(DISP_REG_BLS_RST, 0x1);
	DISP_REG_SET(DISP_REG_BLS_RST, 0x0);
	DISP_REG_SET(DISP_REG_BLS_EN, 0x80000000);

    // UFO to WDMA
    *(volatile int *)(0xF4000880) = 0x3;
#endif

	return 0;
}


int disp_path_config_OVL_WDMA(struct disp_path_config_mem_out_struct* pConfig, int OvlSecure)
{
    DISP_DBG(" disp_path_config_OVL_WDMA(), enable = %d, outFormat=%d, dstAddr=0x%x, ROI(%d,%d,%d,%d) \n",
            pConfig->enable,
            pConfig->outFormat,
            pConfig->dstAddr,
            pConfig->srcROI.x,
            pConfig->srcROI.y,
            pConfig->srcROI.width,
            pConfig->srcROI.height);

    if(pConfig->enable==1 && pConfig->dstAddr==0)
    {
          DISP_ERR("pConfig->dstAddr==0! \n");
    }

    if(pConfig->enable==1)
    {
        OVLROI(pConfig->srcROI.width, // width
            pConfig->srcROI.height, // height
            0);// background B
			       	
        mem_out_done = 0;
        disp_register_irq(DISP_MODULE_WDMA, _disp_path_ovl_wdma_callback);


#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
		// if OVL already in secure state (DAPC protected)
		if (OvlSecure)
		{
			MTEEC_PARAM param[4];
			unsigned int paramTypes;
			TZ_RESULT ret;
			KREE_SHAREDMEM_HANDLE p_wdma_config_share_handle = 0;
			KREE_SHAREDMEM_PARAM  sharedParam;

			// Register shared memory
			sharedParam.buffer = pConfig;
			sharedParam.size = sizeof(struct disp_path_config_mem_out_struct);
			ret = KREE_RegisterSharedmem (ddp_mem_session_handle(), &p_wdma_config_share_handle, &sharedParam);
			if (ret != TZ_RESULT_SUCCESS)
			{
				DISP_ERR("disp_register_share_memory Error: %d, line:%d, ddp_mem_session(%d)", ret, __LINE__, (unsigned int)ddp_mem_session_handle());
				return 0;
			}
			
			param[0].memref.handle = (uint32_t) p_wdma_config_share_handle;
			param[0].memref.offset = 0;
			param[0].memref.size = sizeof(struct disp_path_config_mem_out_struct);
			param[1].value.a = pConfig->security;
            param[2].value.a = 1;
			paramTypes = TZ_ParamTypes3(TZPT_MEMREF_INPUT, TZPT_VALUE_INPUT, TZPT_VALUE_INPUT);
			DISP_DBG("wdma config handle=0x%x \n", param[0].memref.handle);
			
			ret = KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_WDMA_CONFIG, paramTypes, param);
			if(ret!= TZ_RESULT_SUCCESS)
			{
				DISP_ERR("TZCMD_DDP_WDMA_CONFIG fail, ret=%d \n", ret);
			}
			
			ret = KREE_UnregisterSharedmem(ddp_mem_session_handle(), p_wdma_config_share_handle); 
			if (ret)
			{
				DISP_ERR("UREE_UnregisterSharedmem p_wdma_config_share_handle Error: %d, line:%d, ddp_mem_session(%d)", ret, __LINE__, (unsigned int)ddp_mem_session_handle());
			}
		}
		else
#endif
		{
            // config wdma1
            WDMAReset(1);
            WDMAConfig(1,
                       WDMA_INPUT_FORMAT_ARGB,
                       pConfig->srcROI.width,
                       pConfig->srcROI.height,
                       0,
                       0,
                       pConfig->srcROI.width,
                       pConfig->srcROI.height,
                       pConfig->outFormat,
                       pConfig->dstAddr,
                       (pConfig->dstPitch)?pConfig->dstPitch:pConfig->srcROI.width,
                       1,
                       0);
			
	        if(pConfig->outFormat == eYUV_420_3P) {
				unsigned int picSz, uAddr, vAddr, width_align, height_align;
				width_align = ALIGN_TO(pConfig->srcROI.width, 16);
				height_align = ALIGN_TO(pConfig->srcROI.height, 16);
				picSz = width_align * height_align;
				uAddr = pConfig->dstAddr + picSz;					
				if (pConfig->dstPitch)
					vAddr = uAddr + ((ALIGN_TO(pConfig->dstPitch>>1, 16)*height_align)>>1);
				else
					vAddr = uAddr + ((ALIGN_TO(pConfig->srcROI.width>>1, 16)*height_align)>>1);	

				pr_err("[normal] WFD dstAddr:0x%08x picSz:0x%08x uAddr:0x%08x vAddr:0x%08x\n",
					pConfig->dstAddr,picSz,uAddr,vAddr);
				pr_err("[normal] WFD width:%d align width:%d, height:%d, align width:%d\n",
					pConfig->srcROI.width, width_align, pConfig->srcROI.height, height_align);
				WDMAConfigUV(1, uAddr, vAddr, pConfig->srcROI.width);
				if (pConfig->dstPitch) {
					pr_err("[normal] WFD dstPitch:%d\n", pConfig->dstPitch);
					DISP_REG_SET(DISP_INDEX_OFFSET + DISP_REG_WDMA_DST_UV_PITCH, 
										ALIGN_TO(pConfig->dstPitch>>1, 16));
				}
			}	

            WDMAStart(1);
	    }
    }
    else
    {
        disp_unregister_irq(DISP_MODULE_WDMA, _disp_path_ovl_wdma_callback);
    }

    return 0;
}
#endif
int disp_path_config_(struct disp_path_config_struct* pConfig, int mutexId)
{
    unsigned int mutexSof;
    //unsigned int mutexValue;

#if 1//MTK_HDMI_MAIN_PATH
	/*original panel(main) path srcmodule 0(DISP_MODULE_OVL),dstmodule 6(DISP_MODULE_DPI0)*/
	/*original hdmi(sub) path srcmodule 5(DISP_MODULE_RDMA1),dstmodule 7(DISP_MODULE_DPI1)*/
	
	/*new hdmi(main path) srcmodule 0(DISP_MODULE_OVL),dstmodule 7(DISP_MODULE_DPI1)*/
	/*ovl->rdma0->color->bls->dpi1->hdmi*/
#if MTK_HDMI_MAIN_PATH
	//dump_stack();
#endif

	DISP_PRINTF(DDP_FLOW_LOG, "%s,srcModule=%d(%s),addr=0x%x,inFormat=%d,pitch=%d,bgROI(%d,%d,%d,%d),bgColor=0x%x",
	__func__,
        pConfig->srcModule,
        disp_module_name[pConfig->srcModule],
	pConfig->addr,
	pConfig->inFormat,
	pConfig->pitch,
	pConfig->bgROI.x,
	pConfig->bgROI.y,
	pConfig->bgROI.width,
	pConfig->bgROI.height,
	pConfig->bgColor);

	DISP_PRINTF(DDP_FLOW_LOG, "%s,outFormat=%d, dstModule=%d(%s), dstAddr=0x%x, dstPitch=%d,mutexId=%d,srcROI(%d,%d,%d,%d)",
	__func__,	
	pConfig->outFormat,
	pConfig->dstModule,
	disp_module_name[pConfig->dstModule],
	pConfig->dstAddr,
	pConfig->dstPitch,
	mutexId,
	pConfig->srcROI.x,
	pConfig->srcROI.y,
	pConfig->srcROI.width,
	pConfig->srcROI.height);

	DISP_PRINTF(DDP_FLOW_LOG, "%s, src(%d %d),dst(%d %d)",
	__func__,	
	pConfig->srcWidth,
	pConfig->srcHeight,
	pConfig->dstWidth,
	pConfig->dstHeight);
#else
    DISP_DBG("[DDP] disp_path_config(), srcModule=%d, addr=0x%x, inFormat=%d, \n\
            pitch=%d, bgROI(%d,%d,%d,%d), bgColor=%d, outFormat=%d, dstModule=%d, dstAddr=0x%x, dstPitch=%d, mutexId=%d \n",
            pConfig->srcModule,
            pConfig->addr,
            pConfig->inFormat,
            pConfig->pitch,
            pConfig->bgROI.x,
            pConfig->bgROI.y,
            pConfig->bgROI.width,
            pConfig->bgROI.height,
            pConfig->bgColor,
            pConfig->outFormat,
            pConfig->dstModule,
            pConfig->dstAddr,
            pConfig->dstPitch,
            mutexId);
#endif

    g_dst_module = pConfig->dstModule;

#ifdef DDP_USE_CLOCK_API
#else
        // TODO: clock manager sholud manager the clock ,not here
        DISP_REG_SET(DISP_REG_CONFIG_MMSYS_CG_CLR0 , 0xFFFFFFFF);
        DISP_REG_SET(DISP_REG_CONFIG_MMSYS_CG_CLR1 , 0xFFFFFFFF);
        DISP_REG_SET(DISP_REG_CONFIG_MMSYS_CG_CON0 , 0xFFF00000);
        DISP_REG_SET(DISP_REG_CONFIG_MMSYS_CG_CON1 , 0xFFFFC000);
#endif

	/*config mutex mode*/
	/*mutex sof source 0xf400e030(50,70,90,b0)*/
	/*mode 0 from single mode,1 from dsi,2 from dpi0,3 from dpi1*/
        switch(pConfig->dstModule)
        {
            case DISP_MODULE_DSI_VDO:
                mutexSof = 1;
                break;

            case DISP_MODULE_DPI0:
                mutexSof = 2;
                break;

            case DISP_MODULE_DPI1:
                mutexSof = 3;
                break;

            case DISP_MODULE_DBI:
            case DISP_MODULE_DSI_CMD:
            case DISP_MODULE_WDMA:
                mutexSof = 0;
                break;

            default:
                mutexSof = 0;
                DISP_ERR("unknown dstModule=%d \n", pConfig->dstModule); 
                return -1;
        }

       
        {
	        if(pConfig->dstModule==DISP_MODULE_WDMA)
	        {
	            DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(mutexId), (1<<3)|(1<<6)); //ovl, wdma0
	        }
	        else if(pConfig->srcModule==DISP_MODULE_OVL)
	        {
	        
	            DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(mutexId), (1<<3)|(1<<10)|(1<<7)|(1<<9)); //ovl, rdma0, color, bls

	        }
            else if(pConfig->srcModule==DISP_MODULE_RDMA0)
	        {

	            DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(mutexId), (1<<10)|(1<<7)|(1<<9)); //rdma, color, bls

	        } 
	        else if(pConfig->srcModule==DISP_MODULE_RDMA1)
	        {

	            DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(mutexId), (1<<12)); //rdma1

	        } 
        }		
        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_SOF(mutexId), mutexSof);
        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_INTSTA, (1 << mutexId));

        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_INTEN , 0x1e0f);  //bit 0.1.2.3. 9.10.11.12
        DISP_REG_SET(DISP_REG_CONFIG_MUTEX_EN(mutexId), 1);
        if (DISP_IsDecoupleMode()) {
        	disp_path_init_m4u_port(DISP_MODULE_RDMA0);
        	disp_path_init_m4u_port(DISP_MODULE_WDMA);
        	DISP_REG_SET(DISP_REG_CONFIG_MUTEX_EN(gMemOutMutexID), 1);
        	DISP_REG_SET(DISP_REG_CONFIG_MUTEX_INTEN , 0x0603);
        	/// config OVL-WDMA data path with mutex0
        	DISP_REG_SET(DISP_REG_CONFIG_MUTEX_MOD(gMemOutMutexID), (1<<3)|(1<<6)); // OVL-->WDMA
        	DISP_REG_SET(DISP_REG_CONFIG_MUTEX_SOF(gMemOutMutexID), 0x0);// single mode
        	DISP_REG_SET(DISP_REG_CONFIG_MUTEX_INTSTA, (1 << gMemOutMutexID)|(1 << mutexId));
        }
        ///> config config reg
        switch(pConfig->dstModule)
        {
        case DISP_MODULE_DSI_VDO:
        case DISP_MODULE_DSI_CMD:
        	if (DISP_IsDecoupleMode()) {
        		DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, 1<<1);  // OVL-->WDMA0
        	} else {
        		DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, 1<<0);  // OVL-->RDMA
        	}
            DISP_REG_SET(DISP_REG_CONFIG_DISP_OUT_SEL, 0);        // Output to DSI
            break;

        case DISP_MODULE_DPI0:
		    printk("[DDP] dstModule DISP_MODULE_DPI0\n");

#if MTK_HDMI_MAIN_PATH
    		/*rdma1->dpi0*/
    		 DISP_REG_SET(DISP_REG_CONFIG_DISP_OUT_SEL, 0x6); /*[3:2] rdma1 out,[1:0] BLS OUT*/

    		 /*rdma1<-dpi0*/
     		 DISP_REG_SET(DISP_REG_CONFIG_DPI0_SEL, 0x1); /*[0] 0 from bls, 1 from rdma1 */

#else
    		if (DISP_IsDecoupleMode()) 
            {
        		DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, 1<<1);  // OVL-->WDMA0
        	} 
            else 
            {
            #ifdef MTK_OVERLAY_ENGINE_SUPPORT
                DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, 1<<1);  // OVL-->WDMA0
                DISP_REG_SET(DISP_REG_CONFIG_DISP_OUT_SEL, 0x9); /*[3:2] rdma1 out to DPI1,[1:0] BLS OUT to DPI0*/
                DISP_REG_SET(DISP_REG_CONFIG_DPI0_SEL, 0x0); /*[0] 0 from bls, 1 from rdma1 */
            #else
        		DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, 1<<0);  // OVL-->RDMA
    	        DISP_REG_SET(DISP_REG_CONFIG_DISP_OUT_SEL, 0x1);         // dpi0
    	    #endif
        	}
            DISP_REG_SET(DISP_REG_CONFIG_DISP_OUT_SEL, 1);         // Output to DPI0
#endif
            break;

	    case DISP_MODULE_DPI1:
    		printk("[DDP] dstModule DISP_MODULE_DPI1\n");

    		#if MTK_HDMI_MAIN_PATH

			if (DISP_IsDecoupleMode()) {
				DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, 1<<1);  // OVL-->WDMA0
			} else {
				DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, 1<<0);  // OVL-->RDMA
			}
    		/*bls->dpi1 0xf400004c*/
    		/*0 dis, 1 dpi0, 2 dpi1*/
    		 DISP_REG_SET(DISP_REG_CONFIG_DISP_OUT_SEL, 0x2);
    		/*bls<-dpi1 0xf4000064*/
    		/* 0 from bls, 1 from rdma1*/
            DISP_REG_SET(DISP_REG_CONFIG_DPI1_SEL, 0x0);
#else
            DISP_REG_SET_FIELD(REG_FLD(2, 2), DISP_REG_CONFIG_DISP_OUT_SEL, 0x2); // rdma1_mout to dpi1
            DISP_REG_SET(DISP_REG_CONFIG_DPI1_SEL, 0x1);        // dpi1_sel from rdma1
#endif
            break;

        case DISP_MODULE_WDMA:
            DISP_REG_SET(DISP_REG_CONFIG_DISP_OVL_MOUT_EN, 0x1<<1);   // wdma0
            break;

        default:
           printk("[DDP] error! unknown dstModule=%d \n", pConfig->dstModule); 
        }    
        
        ///> config engines
        if(
        #if /*MTK_HDMI_MAIN_PATH ||*/ defined(MTK_OVERLAY_ENGINE_SUPPORT)
        pConfig->srcModule==DISP_MODULE_OVL
        #else
        pConfig->srcModule!=DISP_MODULE_RDMA1
        #endif
        )
        {
        	// config OVL
            OVLROI(pConfig->bgROI.width, // width
                   pConfig->bgROI.height, // height
                   pConfig->bgColor);// background B

            if(pConfig->dstModule!=DISP_MODULE_DSI_VDO && pConfig->dstModule!=DISP_MODULE_DPI0)
            {
                OVLStop();
                // OVLReset();
            }
    
            if(pConfig->ovl_config.layer<4)
            {
#if defined(MTK_OVERLAY_ENGINE_SUPPORT)
     			disp_path_config_layer_(&pConfig->ovl_config, (int)pConfig->ovl_config.security); // modify for switch
#else
                OVLLayerSwitch(pConfig->ovl_config.layer, pConfig->ovl_config.layer_en);
                if(pConfig->ovl_config.layer_en!=0)
                {
                    if(pConfig->ovl_config.addr==0 ||
                       pConfig->ovl_config.dst_w==0    ||
                       pConfig->ovl_config.dst_h==0    )
                    {
                        DISP_ERR("ovl parameter invalidate, addr=0x%x, w=%d, h=%d \n",
                               pConfig->ovl_config.addr, 
                               pConfig->ovl_config.dst_w,
                               pConfig->ovl_config.dst_h);
                        return -1;
                    }
                
                    OVLLayerConfig(pConfig->ovl_config.layer,   // layer
                                   pConfig->ovl_config.source,   // data source (0=memory)
                                   pConfig->ovl_config.fmt, 
                                   pConfig->ovl_config.addr, // addr 
                                   pConfig->ovl_config.src_x,  // x
                                   pConfig->ovl_config.src_y,  // y
                                   pConfig->ovl_config.src_pitch, //pitch, pixel number
                                   pConfig->ovl_config.dst_x,  // x
                                   pConfig->ovl_config.dst_y,  // y
                                   pConfig->ovl_config.dst_w, // width
                                   pConfig->ovl_config.dst_h, // height
                                   pConfig->ovl_config.keyEn,  //color key
                                   pConfig->ovl_config.key,  //color key
                                   pConfig->ovl_config.aen, // alpha enable
                                   pConfig->ovl_config.alpha); // alpha
                }
#endif
            }
            else
            {
                DISP_ERR("layer ID undefined! %d \n", pConfig->ovl_config.layer);
            }
            OVLStart();
#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT
            if(pConfig->dstModule==DISP_MODULE_WDMA)  //1. mem->ovl->wdma0->mem
            {
                if(pConfig->WDMA1Security == 1)
				{
					if(pConfig->dstAddr==0 ||
	                   pConfig->srcROI.width==0    ||
	                   pConfig->srcROI.height==0    )
	                {
	                    DISP_ERR("wdma parameter invalidate, addr=0x%x, w=%d, h=%d \n",
	                           pConfig->dstAddr,
	                           pConfig->srcROI.width,
	                           pConfig->srcROI.height);
	                    return -1;
	                }

					MTEEC_PARAM param[4];
					unsigned int paramTypes;
					TZ_RESULT ret;
					KREE_SHAREDMEM_HANDLE p_wdma_config_share_handle = 0;
					KREE_SHAREDMEM_PARAM  sharedParam;

					// Register shared memory
					sharedParam.buffer = pConfig;
					sharedParam.size = sizeof(struct disp_path_config_mem_out_struct);
					ret = KREE_RegisterSharedmem (ddp_mem_session_handle(), &p_wdma_config_share_handle, &sharedParam);
					if (ret != TZ_RESULT_SUCCESS)
					{
						DISP_ERR("disp_register_share_memory Error: %d, line:%d, ddp_mem_session(%d)", ret, __LINE__, (unsigned int)ddp_mem_session_handle());
						return 0;
					}
					
					param[0].memref.handle = (uint32_t) p_wdma_config_share_handle;
					param[0].memref.offset = 0;
					param[0].memref.size = sizeof(struct disp_path_config_mem_out_struct);
					param[1].value.a = pConfig->WDMA1Security;
					param[2].value.a = 1;   //if do wdmaRest and wdmaStart

					paramTypes = TZ_ParamTypes3(TZPT_MEMREF_INPUT, TZPT_VALUE_INPUT, TZPT_VALUE_INPUT);
					DISP_MSG("[ddp_path_config_]wdma config handle=0x%x ifReset=%d\n", param[0].memref.handle, param[2].value.a);
					
					ret = KREE_TeeServiceCall(ddp_session_handle(), TZCMD_DDP_WDMA_CONFIG, paramTypes, param);
					if(ret!= TZ_RESULT_SUCCESS)
					{
						DISP_ERR("TZCMD_DDP_WDMA_CONFIG fail, ret=%d \n", ret);
					}
					
					ret = KREE_UnregisterSharedmem(ddp_mem_session_handle(), p_wdma_config_share_handle); 
					if (ret)
					{
						DISP_ERR("UREE_UnregisterSharedmem p_wdma_config_share_handle Error: %d, line:%d, ddp_mem_session(%d)", ret, __LINE__, (unsigned int)ddp_mem_session_handle());
					}
				}
				else
				{
					WDMAReset(0);
	                if(pConfig->dstAddr==0 ||
	                   pConfig->srcROI.width==0    ||
	                   pConfig->srcROI.height==0    )
	                {
	                    DISP_ERR("wdma parameter invalidate, addr=0x%x, w=%d, h=%d \n",
	                           pConfig->dstAddr, 
	                           pConfig->srcROI.width,
	                           pConfig->srcROI.height);
	                    return -1;
	                }
	
	                WDMAConfig(0, 
	                           WDMA_INPUT_FORMAT_ARGB, 
	                           pConfig->srcROI.width, 
	                           pConfig->srcROI.height, 
	                           0, 
	                           0, 
	                           pConfig->srcROI.width, 
	                           pConfig->srcROI.height, 
	                           pConfig->outFormat, 
	                           pConfig->dstAddr, 
	                           pConfig->srcROI.width, 
	                           1, 
	                           0);      
	                WDMAStart(0);
				}
            }
#else
            if(pConfig->dstModule==DISP_MODULE_WDMA)  //1. mem->ovl->wdma0->mem
            {
                WDMAReset(0);
                if(pConfig->dstAddr==0 ||
                   pConfig->srcROI.width==0    ||
                   pConfig->srcROI.height==0    )
                {
                    DISP_ERR("wdma parameter invalidate, addr=0x%x, w=%d, h=%d \n",
                           pConfig->dstAddr, 
                           pConfig->srcROI.width,
                           pConfig->srcROI.height);
                    return -1;
                }

                WDMAConfig(0, 
                           WDMA_INPUT_FORMAT_ARGB, 
                           pConfig->srcROI.width, 
                           pConfig->srcROI.height, 
                           0, 
                           0, 
                           pConfig->srcROI.width, 
                           pConfig->srcROI.height, 
                           pConfig->outFormat, 
                           pConfig->dstAddr, 
                           pConfig->srcROI.width, 
                           1, 
                           0);      
                WDMAStart(0);
            }
#endif
            else    //2. ovl->bls->rdma0->lcd
            {

                disp_bls_init(pConfig->srcROI.width, pConfig->srcROI.height);

               //=============================config PQ start==================================				 
                DpEngine_COLORonInit();
                DpEngine_COLORonConfig(pConfig->srcROI.width,  //width
                                                     pConfig->srcROI.height); //height

								
                //=============================config PQ end==================================
                ///config RDMA
                if(pConfig->dstModule!=DISP_MODULE_DSI_VDO && pConfig->dstModule!=DISP_MODULE_DPI0)
                {
                    RDMAStop(0);
                    RDMAReset(0);
                }
                if(pConfig->srcROI.width==0    ||
                   pConfig->srcROI.height==0    )
                {
                    DISP_ERR("rdma parameter invalidate, w=%d, h=%d \n",
                           pConfig->srcROI.width,
                           pConfig->srcROI.height);
                    return -1;
                }

                if (DISP_IsDecoupleMode()) {
                	printk("mt8127 from de-couple\n");
                	WDMAReset(0);
                	if(decouple_addr==0 ||
                			pConfig->srcROI.width==0    ||
                			pConfig->srcROI.height==0    )
                	{
                		DISP_ERR("wdma parameter invalidate, addr=0x%x, w=%d, h=%d \n",
                				decouple_addr,
                				pConfig->srcROI.width,
                				pConfig->srcROI.height);
                		return -1;
                	}
                	WDMAConfig(0,
                			WDMA_INPUT_FORMAT_ARGB,
                			pConfig->srcROI.width,
                			pConfig->srcROI.height,
                			0,
                			0,
                			pConfig->srcROI.width,
                			pConfig->srcROI.height,
                			eRGB888,
                			decouple_addr,
                			pConfig->srcROI.width,
                			1,
                			0);
                	WDMAStart(0);
                	// Register WDMA intr
                	mem_out_done = 0;
                	disp_register_irq(DISP_MODULE_WDMA, _disp_path_wdma_callback);

				#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT 
               		RDMAConfig_(0,
                			RDMA_MODE_MEMORY,       	 // mem mode
                			eRGB888,    				 // inputFormat
                			decouple_addr,               // display lk logo when entering kernel
                			RDMA_OUTPUT_FORMAT_ARGB,     // output format
                			pConfig->srcROI.width*3,     // pitch, eRGB888
                			pConfig->srcROI.width,       // width
                			pConfig->srcROI.height,      // height
                			0,                           // byte swap
                			0,
                            pConfig->RDMA0Security);
				#else
                	RDMAConfig(0,
                			RDMA_MODE_MEMORY,       	 // mem mode
                			eRGB888,    				 // inputFormat
                			decouple_addr,               // display lk logo when entering kernel
                			RDMA_OUTPUT_FORMAT_ARGB,     // output format
                			pConfig->srcROI.width*3,     // pitch, eRGB888
                			pConfig->srcROI.width,       // width
                			pConfig->srcROI.height,      // height
                			0,                           // byte swap
                			0);
				#endif
                	RDMAStart(0);
                } 
				else {

                	printk("[DDP] from couple to RDMA0\n");

				#ifdef CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT 
                	RDMAConfig_(0,
                			RDMA_MODE_DIRECT_LINK,       ///direct link mode
                			eRGB888,    // inputFormat
                			0,                        // address
                			pConfig->outFormat,          // output format
                			pConfig->pitch,              // pitch
                			pConfig->srcROI.width,       // width
                			pConfig->srcROI.height,      // height
                			0,                           //byte swap
                			0,
							pConfig->RDMA0Security); 
				#else
                	RDMAConfig(0,
                			RDMA_MODE_DIRECT_LINK,       ///direct link mode
                			eRGB888,    // inputFormat
                			0,                        // address
                			pConfig->outFormat,          // output format
                			pConfig->pitch,              // pitch
                			pConfig->srcROI.width,       // width
                			pConfig->srcROI.height,      // height
                			0,                           //byte swap
                			0);                          // is RGB swap
				#endif
                	RDMAStart(0);
                }
			}
         }
        else  //src module is RDMA1
        {
        	unsigned rdma_idx = 1;
        	//DpColorFormat inFormat = eARGB8888;
	        #if MTK_HDMI_MAIN_PATH || defined(MTK_OVERLAY_ENGINE_SUPPORT)
	        if(pConfig->srcModule==DISP_MODULE_RDMA0) 
            {
	        	rdma_idx = 0;
	        	//inFormat = eARGB8888;
	        }
		    #endif

            if(pConfig->addr==0 ||
               pConfig->srcWidth==0    ||
               pConfig->srcHeight==0    )
            {
                DISP_ERR("rdma parameter invalidate, addr=0x%x, w=%d, h=%d \n",
                           pConfig->addr,
                           pConfig->srcWidth,
                           pConfig->srcHeight);
                return -1;
            }
		
            printk("[DDP] srcModule from rdma%d\n",rdma_idx);
            ///config RDMA
            //RDMAStop(rdma_idx);
            //RDMAReset(rdma_idx);


            #if defined(MTK_OVERLAY_ENGINE_SUPPORT)
            RDMAConfig(rdma_idx,
                       RDMA_MODE_MEMORY,       ///direct link mode
                       eRGB888,                // inputFormat
                       pConfig->addr,          // address
                       pConfig->outFormat,     // output format
                       pConfig->pitch,          //
                       pConfig->srcWidth,
                       pConfig->srcHeight,
                       0,                       //byte swap
                       0);                      // is RGB swap
            #else
            RDMAConfig(rdma_idx, 
                           RDMA_MODE_MEMORY,       		///direct link mode
                           eARGB8888,    				// inputFormat
                           pConfig->addr,               // address
                           pConfig->outFormat,   		// output format
                           pConfig->pitch,              // pitch
                           pConfig->srcWidth,       // width
                           pConfig->srcHeight,      // height
                           0,                           //byte swap
                           0);   
            #endif

            RDMAStart(rdma_idx);

            if(rdma_idx == 1) {
            	/*stop rdma1 for main hdmi path*/
            	//RDMAStop(rdma_idx);

            }
    
                    /////bypass BLS
                //dispsys_bypass_bls(pConfig->srcROI.width, pConfig->srcROI.height);

                /////bypass COLOR
                //dispsys_bypass_color(pConfig->srcROI.width, pConfig->srcROI.height);
        }

#if 0
        disp_dump_reg(DISP_MODULE_OVL);
        disp_dump_reg(DISP_MODULE_WDMA);
        disp_dump_reg(DISP_MODULE_COLOR);
        disp_dump_reg(DISP_MODULE_BLS);
        disp_dump_reg(DISP_MODULE_DPI0);
        disp_dump_reg(DISP_MODULE_RDMA1);
        disp_dump_reg(DISP_MODULE_CONFIG);
        disp_dump_reg(DISP_MODULE_MUTEX);
#endif

/*************************************************/
// Ultra config
    // ovl ultra 0x40402020
    DISP_REG_SET(DISP_REG_OVL_RDMA0_MEM_GMC_SETTING, 0x40402020);
    DISP_REG_SET(DISP_REG_OVL_RDMA1_MEM_GMC_SETTING, 0x40402020);
    DISP_REG_SET(DISP_REG_OVL_RDMA2_MEM_GMC_SETTING, 0x40402020);
    DISP_REG_SET(DISP_REG_OVL_RDMA3_MEM_GMC_SETTING, 0x40402020);
    // disp_rdma1 ultra
    //DISP_REG_SET(DISP_REG_RDMA_MEM_GMC_SETTING_0, 0x20402040);
    // disp_wdma0 ultra
    //DISP_REG_SET(DISP_REG_WDMA_BUF_CON1, 0x10000000);
    //DISP_REG_SET(DISP_REG_WDMA_BUF_CON2, 0x20402020);

    // a workaround for OVL hang after back to back grlast
    DISP_REG_SET(DISP_REG_OVL_DATAPATH_CON, (1<<29));

    return 0;
}


