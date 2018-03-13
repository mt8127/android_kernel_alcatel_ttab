#include <linux/time.h>
#include <mach/m4u.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0))
#include <linux/sw_sync.h>
#else
#include <../drivers/staging/android/sw_sync.h>
#endif
#include <linux/sched.h>
//#include <linux/ion.h>
#include <linux/ion_drv.h>
#include "disp_ovl_engine_core.h"
#include "disp_ovl_engine_sw.h"
#include "disp_ovl_engine_hw.h"
#include "debug.h"

// Ovl_engine parameter
DISP_OVL_ENGINE_PARAMS disp_ovl_engine;
DEFINE_SEMAPHORE(disp_ovl_engine_semaphore);
int disp_ovl_engine_log_level = 0;

// Ovl_engine thread
static struct task_struct *disp_ovl_engine_task = NULL;
static int disp_ovl_engine_kthread(void *data);
static wait_queue_head_t disp_ovl_engine_wq;
static unsigned int gWakeupOvlEngineThread = 0;

// rdma0 update thread
static struct task_struct *disp_rdma0_update_task = NULL;
static int disp_ovl_engine_rdma0_update_kthread(void *data);
static wait_queue_head_t disp_rdma0_update_wq;
static unsigned int gWakeupRdma0UpdateThread = 0;
DEFINE_SEMAPHORE(disp_rdma0_update_semaphore);

// 
static int curr_instance_id = 0;

// Complete notification
wait_queue_head_t disp_ovl_complete_wq;
unsigned long gStartTriggerTime = 0;

unsigned int gDumpOVLEnable = 0;
extern unsigned int gCaptureOvlDownX;
extern unsigned int gCaptureOvlDownY;

#ifdef DISP_OVL_ENGINE_REQUEST
struct disp_ovl_engine_request_struct gOverlayRequest;
wait_queue_head_t disp_ovl_get_request_wq;
static unsigned int gWakeupGetRequest = 0;
wait_queue_head_t disp_ovl_ack_request_wq;
static unsigned int gWakeupAckRequest = 0;
#endif

//static size_t ovl_eng_log_on = false;

unsigned long disp_ovl_engine_get_current_time_us(void);

void set_ovlengine_debug_level(int level)
{
    disp_ovl_engine_log_level = level;
}

void disp_ovl_engine_init(void)
{
    unsigned int i=0;
    static int init = 0;
	DISP_OVL_ENGINE_INFO("disp_ovl_engine_init\n");

    if (!init)
    {
        init = 1;
        // Init ovl_engine parameter
        disp_ovl_engine.bInit = true;

        memset(disp_ovl_engine.Instance, 0, sizeof(DISP_OVL_ENGINE_INSTANCE)*OVL_ENGINE_INSTANCE_MAX_INDEX);
        memset(disp_ovl_engine.OvlBufAddr, 0, sizeof(unsigned int) * OVL_ENGINE_OVL_BUFFER_NUMBER);
		disp_ovl_engine.current_instance = NULL;

        disp_ovl_engine.bCouple = true;
        disp_ovl_engine.captured_layer_config = disp_ovl_engine.layer_config[0];
        disp_ovl_engine.realtime_layer_config = disp_ovl_engine.layer_config[0];

        while(i < OVL_ENGINE_INSTANCE_MAX_INDEX)
        {
            disp_ovl_engine.Instance[i].index = i;
			disp_ovl_engine.Instance[i].fgCompleted = 0;
            atomic_set(&disp_ovl_engine.Instance[i].OverlaySettingDirtyFlag,0);
            atomic_set(&disp_ovl_engine.Instance[i].OverlaySettingApplied ,0);
#ifdef DISP_OVL_ENGINE_FENCE
            disp_ovl_engine.Instance[i].wq = alloc_workqueue("ovl_eng_wq",WQ_HIGHPRI | WQ_UNBOUND | WQ_MEM_RECLAIM, 1);   
            if (!disp_ovl_engine.Instance[i].wq) {
                DISP_OVL_ENGINE_ERR("failed to create overlay engine work queue\n");
                return;
            }

            disp_ovl_engine.Instance[i].timeline = sw_sync_timeline_create("overlay");
            disp_ovl_engine.Instance[i].timeline_max = 0;
            disp_ovl_engine.Instance[i].timeline_skip = 0;
#endif
            i ++;
        }
        i = 0;
        while ( i<DDP_OVL_LAYER_MUN)
        {
            disp_ovl_engine.layer_config[0][i].layer = i;
            disp_ovl_engine.layer_config[1][i].layer = i;
            i ++;
        }

#ifdef DISP_OVL_ENGINE_SW_SUPPORT
        disp_ovl_engine_sw_init();
        disp_ovl_engine_sw_register_irq(disp_ovl_engine_interrupt_handler);
#endif

#ifdef DISP_OVL_ENGINE_HW_SUPPORT
		disp_ovl_engine_hw_init();
		disp_ovl_engine_hw_register_irq(disp_ovl_engine_interrupt_handler);
#endif
		/* Init ion */
		disp_ovl_engine.ion_client = ion_client_create(g_ion_device, "overlay");
		if (IS_ERR(disp_ovl_engine.ion_client))
			DISP_OVL_ENGINE_ERR("failed to create ion client\n");
		
        // Init ovl_engine complete notification
        init_waitqueue_head(&disp_ovl_complete_wq);

        // Init ovl_engine thread
        init_waitqueue_head(&disp_ovl_engine_wq);
        disp_ovl_engine_task = kthread_create(
            disp_ovl_engine_kthread, NULL, "ovl_engine_kthread");
        wake_up_process(disp_ovl_engine_task);
		DISP_OVL_ENGINE_INFO("kthread_create ovl_engine_kthread\n");    

        // Init rdma0 update thread
        init_waitqueue_head(&disp_rdma0_update_wq);
        disp_rdma0_update_task = kthread_create(
            disp_ovl_engine_rdma0_update_kthread, NULL, "rdma0_update_kthread");
        wake_up_process(disp_rdma0_update_task);
    	DISP_OVL_ENGINE_INFO("kthread_create rdma0_update_kthread\n");  
		
#ifdef DISP_OVL_ENGINE_REQUEST
        // Init ovl_engine request
        init_waitqueue_head(&disp_ovl_get_request_wq);
        init_waitqueue_head(&disp_ovl_ack_request_wq);
#endif

    }
}

void disp_ovl_engine_dump_ovl(int idx)
{
    MMP_MetaDataBitmap_t Bitmap;
    unsigned int buf_size = DISP_GetScreenWidth() * DISP_GetScreenHeight() * 3;
    
    if (gDumpOVLEnable)
    {
        if (idx == 0)
        {
            Bitmap.data1 = disp_ovl_engine.RdmaRdIdx;
            Bitmap.data2 = disp_ovl_engine.OvlWrIdx;
            Bitmap.width = DISP_GetScreenWidth();
            Bitmap.height = DISP_GetScreenHeight();
            Bitmap.format = MMProfileBitmapBGR888;
            Bitmap.start_pos = 0;
            Bitmap.bpp = 24;
            Bitmap.pitch = DISP_GetScreenWidth()*3;
            Bitmap.data_size = Bitmap.pitch * Bitmap.height;
            Bitmap.down_sample_x = gCaptureOvlDownX;
            Bitmap.down_sample_y = gCaptureOvlDownY;
            Bitmap.pData = disp_ovl_engine.OvlBufAddr_va[disp_ovl_engine.RdmaRdIdx];

            printk("[OVL] start to dump OVL\n");
            //disphal_dma_map_kernel(disp_ovl_engine.OvlBufAddr[disp_ovl_engine.RdmaRdIdx], 
            //    buf_size, (unsigned int*)&Bitmap.pData, &Bitmap.data_size);

            //printk("[OVL] map 0x%x(mva)->0x%x(va) size:0x%x\n", 
            //    disp_ovl_engine.OvlBufAddr[disp_ovl_engine.RdmaRdIdx], 
            //    Bitmap.pData, Bitmap.data_size);
            
            //if (Bitmap.data_size != 0)
                MMProfileLogMetaBitmap(MTKFB_MMP_Events.OvlDump, MMProfileFlagPulse, &Bitmap); 
            
            //disphal_dma_unmap_kernel(disp_ovl_engine.OvlBufAddr[disp_ovl_engine.RdmaRdIdx], 
            //    Bitmap.data_size, (unsigned int)Bitmap.pData);
            printk("[OVL] dump OVL end\n");
        }    
        else if (idx == 1)
        {
            
        }
    }
}


/*
1. Instance status manager
2. signal fence and free ION handle
3. wake up OVL complete WQ
*/
static int disp_ovl_engine_kthread(void *data)
{
    int wait_ret = 0;
    struct sched_param param = { .sched_priority = RTPM_PRIO_SCRN_UPDATE };
    DISP_OVL_ENGINE_INSTANCE *temp = &disp_ovl_engine.Instance[curr_instance_id];
    
    sched_setscheduler(current, SCHED_RR, &param);
    
    while(1)
    {
		//DISP_OVL_ENGINE_DBG("wait_event_interruptible\n");

        wait_ret = wait_event_interruptible(disp_ovl_engine_wq, gWakeupOvlEngineThread);
        gWakeupOvlEngineThread = 0;

		DISP_OVL_ENGINE_DBG("disp_ovl_engine_kthread wake_up\n");

		if(down_interruptible(&disp_ovl_engine_semaphore)) 
        {
			DISP_OVL_ENGINE_ERR("disp_ovl_engine_kthread down_interruptible(disp_ovl_engine_semaphore) fail\n");
            continue;
		}

		DISP_OVL_ENGINE_DBG("disp_ovl_engine_kthread instance %d status %d\n",
			curr_instance_id, temp->status);
                
        // Overlay complete notification
        {
            int i;

            for(i=0; i<OVL_ENGINE_INSTANCE_MAX_INDEX; i++)
            {
                if(disp_ovl_engine.Instance[i].status == OVERLAY_STATUS_COMPLETE)
                {				
                    MMProfileLogEx(MTKFB_MMP_Events.Instance_status[i], MMProfileFlagEnd, curr_instance_id, OVERLAY_STATUS_COMPLETE);
                    disp_ovl_engine.Instance[i].status = OVERLAY_STATUS_IDLE;
                    disp_ovl_engine.Instance[i].fgCompleted = 1;
#ifdef DISP_OVL_ENGINE_FENCE
                    if(disp_ovl_engine.Instance[i].outFence != -1)
                    {
                        sw_sync_timeline_inc(disp_ovl_engine.Instance[i].timeline, 
                            1+disp_ovl_engine.Instance[i].timeline_skip);
                        DISP_OVL_ENGINE_DBG("disp_ovl_engine_kthread %d timeline inc %d\n",
                            i, 1+disp_ovl_engine.Instance[i].timeline_skip);
                        disp_ovl_engine.Instance[i].timeline_skip = 0;                        
                        //printk("[02420] disp_ovl_engine_kthread %d timeline inc, timeline->value = %d\n",i,
                        //disp_ovl_engine.Instance[i].timeline->value);
                        MMProfileLogEx(MTKFB_MMP_Events.OVLEngine_timeline[i], MMProfileFlagPulse, 
                            i, disp_ovl_engine.Instance[i].timeline->value);
                    }
					#if 0 //monica merge
					// free ion handle
                    if (DECOUPLE_MODE == disp_ovl_engine.Instance[i].mode){
                        int j;

                        for (j=0; j< HW_OVERLAY_COUNT;j ++){
                            if (disp_ovl_engine.Instance[i].cached_layer_config[j].fgIonHandleImport){
                                ion_free(disp_ovl_engine.ion_client, disp_ovl_engine.Instance[i].cached_layer_config[j].ion_handles);
                                disp_ovl_engine.Instance[i].cached_layer_config[j].fgIonHandleImport = FALSE;
                                disp_ovl_engine.Instance[i].cached_layer_config[j].ion_handles = NULL;
                            }
                        }
                    }
					#endif

    				/* free ion handle */
    				if (DECOUPLE_MODE == disp_ovl_engine.Instance[i].mode) 
                    {
						int j;
						DISP_OVL_ENGINE_INSTANCE *temp = &disp_ovl_engine.Instance[i];
						for (j = 0; j < HW_OVERLAY_COUNT; j++) 
                        {
							if (temp->cached_layer_config[j].fgIonHandleImport) 
                            {
								ion_free(disp_ovl_engine.ion_client, temp->cached_layer_config[j].ion_handles);
								temp->cached_layer_config[j].fgIonHandleImport = FALSE;								
								temp->cached_layer_config[j].ion_handles = NULL;
							}
						}
    				}
#endif
                    disp_ovl_engine_dump_ovl(i);
        
                    wake_up_all(&disp_ovl_complete_wq);
        
                    DISP_OVL_ENGINE_INFO("disp_ovl_engine_kthread overlay complete %d\n",i);
                }
            }
        }

        if(temp->status == OVERLAY_STATUS_BUSY)   //time out
        {
            unsigned long currentTime = disp_ovl_engine_get_current_time_us();

            if((currentTime - gStartTriggerTime) > 1000)
            {
#if 0 //monica merge
//#ifdef DISP_OVL_ENGINE_FENCE
                if (disp_ovl_engine.Instance[curr_instance_id].outFence != -1) {
                    sw_sync_timeline_inc(disp_ovl_engine.
                                 Instance[curr_instance_id].timeline,
                                 1 +
                                 disp_ovl_engine.
                                 Instance[curr_instance_id].timeline_skip);
                    DISP_OVL_ENGINE_DBG
                        ("disp_ovl_engine_kthread %d timeline inc for timeout instance %d\n",
                         curr_instance_id,
                         1 + disp_ovl_engine.Instance[curr_instance_id].timeline_skip);
                    disp_ovl_engine.Instance[curr_instance_id].timeline_skip = 0;
                }
                if (DECOUPLE_MODE == disp_ovl_engine.Instance[curr_instance_id].mode){
                    int j;
                    for (j=0; j< HW_OVERLAY_COUNT;j ++){
                        if (disp_ovl_engine.Instance[curr_instance_id].cached_layer_config[j].fgIonHandleImport){
                            ion_free(disp_ovl_engine.ion_client, disp_ovl_engine.Instance[curr_instance_id].cached_layer_config[j].ion_handles);
                            disp_ovl_engine.Instance[curr_instance_id].cached_layer_config[j].fgIonHandleImport = FALSE;
                            disp_ovl_engine.Instance[curr_instance_id].cached_layer_config[j].ion_handles = NULL;
                        }
                    }
                }
                //disp_ovl_engine_hw_reset();
#endif

#ifdef DISP_OVL_ENGINE_FENCE
				if (temp->outFence != -1) 
                {
					sw_sync_timeline_inc(temp->timeline,1 
                        + temp->timeline_skip);
					DISP_OVL_ENGINE_DBG("disp_ovl_engine_kthread %d timeline inc for timeout instance %d\n",
							 curr_instance_id, 1 + temp->timeline_skip);
					temp->timeline_skip = 0;
                    MMProfileLogEx(MTKFB_MMP_Events.OVLEngine_timeline[curr_instance_id], MMProfileFlagPulse, 
                            curr_instance_id, disp_ovl_engine.Instance[curr_instance_id].timeline->value);
				}
                
				if (DECOUPLE_MODE == temp->mode) 
                {
					int j;
				    for (j = 0; j < HW_OVERLAY_COUNT; j++) 
                    {
					    if (temp->cached_layer_config[j].fgIonHandleImport) 
                        {
							ion_free(disp_ovl_engine.ion_client, temp->cached_layer_config[j].ion_handles);
							temp->cached_layer_config[j].fgIonHandleImport = FALSE;
							temp->cached_layer_config[j].ion_handles = NULL;
						}
					}
				}
				//disp_ovl_engine_hw_reset();
#endif

                temp->status = OVERLAY_STATUS_IDLE;
                DISP_OVL_ENGINE_ERR("disp_ovl_engine.Instance[%d] busy too long, reset to idle\n",
                   curr_instance_id);
            } 
            else
            {
                up(&disp_ovl_engine_semaphore); 
                MMProfileLogEx(MTKFB_MMP_Events.Instance_status[curr_instance_id], MMProfileFlagPulse, curr_instance_id, OVERLAY_STATUS_BUSY);
                continue;
            }
        }

        {
            int i;

			//DISP_OVL_ENGINE_DBG("disp_ovl_engine_kthread find next request\n");
			
            // Find next overlay request    
            for(i=0; i<OVL_ENGINE_INSTANCE_MAX_INDEX; i++)
            {
                curr_instance_id++;
                curr_instance_id %= OVL_ENGINE_INSTANCE_MAX_INDEX;
                temp = &disp_ovl_engine.Instance[curr_instance_id];

				//DISP_OVL_ENGINE_DBG("disp_ovl_engine_kthread instance %d status %d\n",
				//	curr_instance_id,temp->status);
                
                if(temp->status == OVERLAY_STATUS_TRIGGER)
                {
                    MMProfileLogEx(MTKFB_MMP_Events.Instance_status[curr_instance_id], MMProfileFlagStart, curr_instance_id, OVERLAY_STATUS_TRIGGER);
                 	if(temp->MemOutConfig.dirty)
                	{
                		atomic_set(&temp->OverlaySettingDirtyFlag,1);
					}
                    
                    DISP_OVL_ENGINE_INFO("disp_ovl_engine instance %d(status %d)will be service\n", 
                        curr_instance_id, temp->status);
                    temp->status = OVERLAY_STATUS_BUSY;

					gStartTriggerTime = disp_ovl_engine_get_current_time_us();						
                    // Trigger overlay
#ifdef DISP_OVL_ENGINE_SW_SUPPORT
                    disp_ovl_engine_sw_set_params(temp);
                    disp_ovl_engine_trigger_sw_overlay();
#endif

#ifdef DISP_OVL_ENGINE_HW_SUPPORT
                    disp_ovl_engine_hw_set_params(temp);
                    disp_ovl_engine_trigger_hw_overlay();
#endif
                    
                    break;
                }
            }           
        }
        
        up(&disp_ovl_engine_semaphore);
        
        if (kthread_should_stop())
            break;
    }
    
    return OVL_OK;
}

void disp_ovl_engine_wake_up_ovl_engine_thread(void)
{
    MMProfileLogEx(MTKFB_MMP_Events.wake_up_ovl_engine_thread, MMProfileFlagPulse, 0, 0);
    
    gWakeupOvlEngineThread = 1;
    wake_up(&disp_ovl_engine_wq);
}

void disp_ovl_engine_interrupt_handler(unsigned int param)
{
    DISP_OVL_ENGINE_INFO("disp_ovl_engine_interrupt_handler %d\n",curr_instance_id);
    MMProfileLogEx(MTKFB_MMP_Events.interrupt_handler, MMProfileFlagPulse, param, 0);

    if (OVERLAY_STATUS_BUSY == disp_ovl_engine.Instance[curr_instance_id].status) 
    {
        disp_ovl_engine.Instance[curr_instance_id].status = OVERLAY_STATUS_COMPLETE;
        MMProfileLogEx(MTKFB_MMP_Events.OVLEngine_ovlwdma_status, MMProfileFlagEnd, 
            disp_ovl_engine.OvlWrIdx, disp_ovl_engine.OvlBufAddr[disp_ovl_engine.OvlWrIdx]);
        
        disp_ovl_engine.RdmaRdIdx = disp_ovl_engine.OvlWrIdx;
        MMProfileLogEx(MTKFB_MMP_Events.OVLEngine_rdma_status, MMProfileFlagPulse, 
            disp_ovl_engine.RdmaRdIdx, disp_ovl_engine.OvlBufAddr[disp_ovl_engine.RdmaRdIdx]);
    } 
    else 
    {
        int i = 0;
        for (i = 0; i < OVL_ENGINE_INSTANCE_MAX_INDEX; i++) 
        {
            if (OVERLAY_STATUS_BUSY == disp_ovl_engine.Instance[i].status) 
            {
                disp_ovl_engine.Instance[i].status = OVERLAY_STATUS_COMPLETE;
            }
        }
    }

    disp_ovl_engine_wake_up_ovl_engine_thread();
}

#ifdef DISP_OVL_ENGINE_REQUEST
int Disp_Ovl_Engine_Set_Request(struct disp_ovl_engine_request_struct *overlayRequest,int timeout)
{
    int wait_ret;
	
	gOverlayRequest.request = overlayRequest->request;
	gOverlayRequest.value = overlayRequest->value;
    gOverlayRequest.ret = -1;

	DISP_OVL_ENGINE_INFO("Disp_Ovl_Engine_Set_Request request %d, value %d\n",
        gOverlayRequest.request,gOverlayRequest.value);

	gWakeupGetRequest = 1;
    wake_up(&disp_ovl_get_request_wq);

    wait_ret = wait_event_interruptible_timeout(disp_ovl_ack_request_wq, gWakeupAckRequest, timeout);
    gWakeupAckRequest = 0;

	DISP_OVL_ENGINE_INFO("Disp_Ovl_Engine_Set_Request ret %d\n",gOverlayRequest.ret);

    overlayRequest->ret = gOverlayRequest.ret;

    return OVL_OK;
}

int Disp_Ovl_Engine_Get_Request(struct disp_ovl_engine_request_struct *overlayRequest)
{
    int wait_ret;

	wait_ret = wait_event_interruptible(disp_ovl_get_request_wq, gWakeupGetRequest);
	gWakeupGetRequest = 0;

	DISP_OVL_ENGINE_INFO("Disp_Ovl_Engine_Get_Request request %d, value %d\n",
        gOverlayRequest.request,gOverlayRequest.value);

    overlayRequest->request = gOverlayRequest.request;
	overlayRequest->value = gOverlayRequest.value;

    return OVL_OK;
}

int Disp_Ovl_Engine_Ack_Request(struct disp_ovl_engine_request_struct *overlayRequest)
{
	gOverlayRequest.ret = overlayRequest->ret;

	DISP_OVL_ENGINE_INFO("Disp_Ovl_Engine_Ack_Request ret %d\n",gOverlayRequest.ret);

    gWakeupAckRequest = 1;
    wake_up(&disp_ovl_ack_request_wq);

    return OVL_OK;
}
#endif


unsigned long disp_ovl_engine_get_current_time_us(void)
{
    struct timeval t;
    do_gettimeofday(&t);
    return t.tv_sec * 1000 + t.tv_usec / 1000;
}

void disp_ovl_engine_wake_up_rdma0_update_thread(void)
{    
    gWakeupRdma0UpdateThread = 1;
    wake_up(&disp_rdma0_update_wq);
}

static int disp_ovl_engine_rdma0_update_kthread(void *data)
{
    int wait_ret = 0;
    struct sched_param param = { .sched_priority = RTPM_PRIO_SCRN_UPDATE };
    
    sched_setscheduler(current, SCHED_RR, &param);
    
    while(1)
    {
		//DISP_OVL_ENGINE_DBG("wait_event_interruptible\n");

        wait_ret = wait_event_interruptible(disp_rdma0_update_wq, gWakeupRdma0UpdateThread);
        gWakeupRdma0UpdateThread = 0;

		//DISP_OVL_ENGINE_DBG("disp_ovl_engine_rdma0_update_kthread wake_up\n");

		if(down_interruptible(&disp_rdma0_update_semaphore)) 
        {
			DISP_OVL_ENGINE_ERR("disp_ovl_engine_rdma0_update_kthread down_interruptible(disp_rdma0_update_semaphore) fail\n");
            continue;
		}

        disp_ovl_engine_update_rdma0();
        
        up(&disp_rdma0_update_semaphore);
        
        if (kthread_should_stop())
            break;
    }
    
    return OVL_OK;
}


