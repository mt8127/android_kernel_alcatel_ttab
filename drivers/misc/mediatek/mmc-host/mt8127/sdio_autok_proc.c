#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>

#ifdef CHANGE_SCHED_POLICY
#include <linux/sched.h>
#endif

#include "mt_sd.h"
#include "sdio_autok.h"

#define PROC_AUTOK_NAME "autok"

#ifndef MTK_SDIO30_ONLINE_TUNING_SUPPORT
#define DMA_ON 0
#define DMA_OFF 1
#endif

struct proc_dir_entry *s_proc = NULL;

#ifdef USE_KERNEL_THREAD
struct sdio_autok_thread_data g_autok_thread_data;
struct task_struct *task;
#else   // USE_KERNEL_THREAD
struct workqueue_struct             *g_autok_wq;
struct sdio_autok_workqueue_data    g_autok_thread_data;
#endif  // USE_KERNEL_THREAD

unsigned int autok_done = 0;

extern int sdio_autok_processed;

extern void mmc_set_clock(struct mmc_host *host, unsigned int hz);

#ifdef USE_KERNEL_THREAD
static int autok_thread_func(void *data);
#else   // USE_KERNEL_THREAD
static int autok_thread_func(struct work_struct *data);
#endif  // USE_KERNEL_THREAD

#if 0//ndef USE_KERNEL_THREAD

/*
 * The poor guys doing the actual heavy lifting.  All on-duty workers
 * are either serving the manager role, on idle list or on busy hash.
 */
struct worker {
	/* on idle list while idle, on busy hash table while busy */
	union {
		struct list_head	entry;	/* L: while idle */
		struct hlist_node	hentry;	/* L: while busy */
	};

	struct work_struct	*current_work;	/* L: work being processed */
	struct cpu_workqueue_struct *current_cwq; /* L: current_work's cwq */
	struct list_head	scheduled;	/* L: scheduled works */
	struct task_struct	*task;		/* I: worker task */
	struct global_cwq	*gcwq;		/* I: the associated gcwq */
	/* 64 bytes boundary on 64bit, 32 on 32bit */
	unsigned long		last_active;	/* L: last active timestamp */
	unsigned int		flags;		/* X: flags */
	int			id;		/* I: worker id */
	struct work_struct	rebind_work;	/* L: rebind worker to cpu */
};

/*
 * The externally visible workqueue abstraction is an array of
 * per-CPU workqueues:
 */
struct workqueue_struct {
	unsigned int		flags;		/* W: WQ_* flags */
	union {
		struct cpu_workqueue_struct __percpu	*pcpu;
		struct cpu_workqueue_struct		*single;
		unsigned long				v;
	} cpu_wq;				/* I: cwq's */
	struct list_head	list;		/* W: list of all workqueues */

	struct mutex		flush_mutex;	/* protects wq flushing */
	int			work_color;	/* F: current work color */
	int			flush_color;	/* F: current flush color */
	atomic_t		nr_cwqs_to_flush; /* flush in progress */
	struct wq_flusher	*first_flusher;	/* F: first flusher */
	struct list_head	flusher_queue;	/* F: flush waiters */
	struct list_head	flusher_overflow; /* F: flush overflow list */

	mayday_mask_t		mayday_mask;	/* cpus requesting rescue */
	struct worker		*rescuer;	/* I: rescue worker */

	int			nr_drainers;	/* W: drain in progress */
	int			saved_max_active; /* W: saved cwq max_active */
#ifdef CONFIG_LOCKDEP
	struct lockdep_map	lockdep_map;
#endif
	char			name[];		/* I: workqueue name */
};

#endif  // USE_KERNEL_THREAD

extern void msdc_ungate_clock(struct msdc_host* host);
extern void msdc_gate_clock(struct msdc_host* host, int delay);

#define msdc_dma_on()        sdr_clr_bits(MSDC_CFG, MSDC_CFG_PIO)
#define msdc_dma_off()       sdr_set_bits(MSDC_CFG, MSDC_CFG_PIO)
#define msdc_dma_status()    ((sdr_read32(MSDC_CFG) & MSDC_CFG_PIO) >> 3)

void autok_claim_host(struct msdc_host *host)
{
    mmc_claim_host(host->mmc);
    printk("[%s] msdc%d host claimed\n", __func__, host->id);
}  

void autok_release_host(struct msdc_host *host)
{
    mmc_release_host(host->mmc);
    printk("[%s] msdc%d host released\n", __func__, host->id);
}


static int autok_writeproc(struct file *file,const char *buffer,
                           unsigned long count, void *data)
{
    char stage;
    char bufferContent[PROC_BUF_SIZE];
    char *bufContIdx;
    struct msdc_host *host;
    struct sdio_func *sdioFunc;
    struct mmc_host *mmc;
    int len;
    int procParamsOffset = 0;
    int i;
    
    printk(KERN_INFO "[%s] (/proc/%s) called\n", __func__, PROC_AUTOK_NAME);
    
    if(count >= PROC_BUF_SIZE)
    {
        printk(KERN_INFO "[%s] proc input size (%ld) is larger than buffer size (%d) \n", __func__, count, PROC_BUF_SIZE);
        return -EFAULT;
    }
    
    if (copy_from_user(bufferContent, buffer, count))
        return -EFAULT;
        
    bufferContent[count] = '\0';
    printk(KERN_INFO "[%s] bufferContent: (count = %ld)\n", __func__, count);
    for(i = 0; i < count; i++)
        printk(" %x ", bufferContent[i]);
    printk("\n");
    
    // Parsing bufferContent
    bufContIdx = bufferContent;
    sdioFunc = (struct sdio_func *)(*(int *)bufContIdx);
    bufContIdx += 4;
    procParamsOffset += 4;
    stage = *bufContIdx;
    bufContIdx += 1;
    procParamsOffset += 1;
    if(count <= procParamsOffset)
    {
        printk(KERN_INFO "[%s] count <= procParamsOffset, count = %d, procParamsOffset = %d\n", __func__, count, procParamsOffset);
        stage = 1;
    }
    else
    {
        memcpy(&len, bufContIdx, sizeof(int));
        bufContIdx = bufContIdx + sizeof(int);
        procParamsOffset += sizeof(int);
        if(len > count - procParamsOffset)
        {
            printk(KERN_INFO "[%s] autok stage 1 result len (%d) is larger than actual proc input size (%ld) \n", __func__, len, count - procParamsOffset);
            return -EFAULT;
        }
        
		memcpy(g_autok_thread_data.autok_stage1_result, bufContIdx, len);
        g_autok_thread_data.len = len;
        
		printk(KERN_INFO "[%s] autok_stage1_result: (len = %d)\n", __func__, len);
        for(i = 0; i < len; i++)
            printk(" %x ", g_autok_thread_data.autok_stage1_result[i]);
        printk("\n");
    }
    
    printk(KERN_INFO "[%s] stage = %d\n", __func__, stage);
    
    if(sdioFunc == NULL)
    {
        printk(KERN_INFO "[%s] sdioFunc = NULL\n", __func__);
        return -EFAULT;
    }
        
    mmc = sdioFunc->card->host;
    host = mmc_priv(mmc);
    
    // Set clock to card max clock
    sdio_autok_processed = 1;
    //printk(KERN_INFO "[%s] mmc->ios.clock = %d, mmc->ios.power_mode = %d\n", __func__, mmc->ios.clock, mmc->ios.power_mode);
    mmc_set_clock(mmc, mmc->ios.clock);
    
    g_autok_thread_data.host = host;
    g_autok_thread_data.sdioFunc = sdioFunc;
    g_autok_thread_data.stage = stage;

    autok_done = 0;

#ifdef USE_KERNEL_THREAD    
    task = kthread_run(&autok_thread_func,(void *)(&g_autok_thread_data),"autokp");
    //if(!IS_ERR(task))
    //    wake_up_process(task);
#else   // USE_KERNEL_THREAD
    queue_delayed_work_on(0, g_autok_wq, (struct delayed_work *)&g_autok_thread_data, msecs_to_jiffies(0));
#endif  // USE_KERNEL_THREAD
    
    return count;
}


static int autok_readproc(char *page, char **start, off_t off,
                          int count, int *eof, void *data)
{
    void *param;
    int len;
    int i;
    char *p = page;
    
    printk(KERN_INFO "[%s] (/proc/%s) called\n", __func__, PROC_AUTOK_NAME);
    
    // read auto-K result from auto-K callback function
    msdc_autok_stg1_data_get(&param, &len);
    
    memcpy(p, &len, sizeof(int));
    p = p + sizeof(int);
    memcpy(p, param, len);
    
    printk(KERN_INFO "[%s] page = (len = %d)\n", __func__, len);
    for(i = 0; i < len + sizeof(int); i++)
    {
        printk(" %x ", page[i]);
    }
    
    printk("\n");
    
    return len + sizeof(int);
}

extern unsigned int autok_get_current_vcore_offset(void);
extern void mt_cpufreq_disable(unsigned int type, bool disabled);

#ifdef USE_KERNEL_THREAD
static int autok_thread_func(void *data)
#else   // USE_KERNEL_THREAD
static int autok_thread_func(struct work_struct *data)
#endif  // USE_KERNEL_THREAD
{
    int err = 0;
    unsigned int vcore_uv = 0;
    void *msdc_param = NULL;
    int len = 0;
    
    #ifdef USE_KERNEL_THREAD
    struct sdio_autok_thread_data *autok_thread_data = (struct sdio_autok_thread_data *)data;
    #else   // USE_KERNEL_THREAD
    struct sdio_autok_workqueue_data *autok_thread_data = (struct sdio_autok_workqueue_data *)data;
        #ifdef CHANGE_SCHED_POLICY
    struct task_struct *ltask = (struct task_struct *)get_wq_task(g_autok_wq);
        #endif  // CHANGE_SCHED_POLICY
    #endif  // USE_KERNEL_THREAD
    
    struct msdc_host *host = autok_thread_data->host;
    struct sdio_func *sdioFunc = autok_thread_data->sdioFunc;
    char stage = autok_thread_data->stage;
    char *envp[2];
    char *lteprocenvp[2];
    u32 base = host->base;
    u32 dma = msdc_dma_status();
#ifdef CHANGE_SCHED_POLICY    
    struct sched_param param;
    int sched_policy;
    
    #ifdef SCHED_POLICY_INFO
    sched_policy = sched_getscheduler(0);
    printk("[%s] orig. sched policy: %d\n", __func__, sched_policy);
    
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if( sched_setscheduler( 0, SCHED_FIFO, &param ) == -1 )
    {
    	printk("[%s] sched_setscheduler fail\n", __func__);
    }
    
    sched_policy = sched_getscheduler(0);
    printk("[%s] sched policy FIFO: %d\n", __func__, sched_policy);
    #endif
    
    //param.sched_priority = sched_get_priority_max(SCHED_RR);
    param.sched_priority = 1;
    #ifdef USE_KERNEL_THREAD
    if( sched_setscheduler( task, SCHED_RR, &param ) == -1 )
    {
    	printk("[%s] sched_setscheduler fail\n", __func__);
    }
    #else   // USE_KERNEL_THREAD
    if( sched_setscheduler( ltask, SCHED_RR, &param ) == -1 )
    {
    	printk("[%s] sched_setscheduler fail\n", __func__);
    }
    #endif  // USE_KERNEL_THREAD
    
    #ifdef SCHED_POLICY_INFO
    sched_policy = sched_getscheduler(0);
    printk("[%s] modified sched policy: %d\n", __func__, sched_policy);
    #endif
#endif

	mt_cpufreq_disable(0, true);
	
    vcore_uv = autok_get_current_vcore_offset();

#ifdef MTK_SDIO30_ONLINE_TUNING_SUPPORT    
    atomic_set(&host->ot_work.ot_disable, 1);
#endif  // MTK_SDIO30_ONLINE_TUNING_SUPPORT

    //sdio_claim_host(sdioFunc);
    autok_claim_host(host);
    
    msdc_ungate_clock(host);
    
    /* Set PIO mode */
    msdc_dma_off();
    
    if(stage == 1) {
        // call stage 1 auto-K callback function
        msdc_autok_stg1_cal(host, vcore_uv);

        // read auto-K result from auto-K callback function
        if(msdc_autok_stg1_data_get(&msdc_param, &len) == 0)
        {
            // apply MSDC parameter for current vcore
            msdc_autok_apply_param(host, msdc_param, len, vcore_uv);
        }
        else
            printk("[%s] msdc_autok_stg1_data_get error\n", __func__);
        
        envp[0] = "FROM=sdio_autok";
        envp[1] = NULL;
        err = kobject_uevent_env(&host->mmc->class_dev.kobj, KOBJ_ONLINE, envp);
        if(err < 0)
            printk(KERN_INFO "[%s] kobject_uevent_env error = %d\n", __func__, err);
    } else if(stage == 2) {
        // call stage 2 auto-K callback function
        msdc_autok_stg2_cal(host, autok_thread_data->autok_stage1_result, autok_thread_data->len, vcore_uv);
    } else {
        printk(KERN_INFO "[%s] stage %d doesn't support in auto-K\n", __func__, stage);
        //sdio_release_host(sdioFunc);
        autok_release_host(host);
		mt_cpufreq_disable(0, false);
        return -EFAULT;
    }
    
    if(dma == DMA_ON)
        msdc_dma_on();
    msdc_gate_clock(host,1);

    
    //sdio_release_host(sdioFunc);
    autok_release_host(host);
    
    vcore_uv = autok_get_current_vcore_offset();
	
	mt_cpufreq_disable(0, false);
    
#ifdef MTK_SDIO30_ONLINE_TUNING_SUPPORT
    atomic_set(&host->ot_work.autok_done, 1);
    atomic_set(&host->ot_work.ot_disable, 0);
#endif  // MTK_SDIO30_ONLINE_TUNING_SUPPORT
    
    autok_done = 1;
    
    lteprocenvp[0] = "FROM=autok_done";
    lteprocenvp[1] = NULL;
    err = kobject_uevent_env(&host->mmc->class_dev.kobj, KOBJ_ONLINE, lteprocenvp);
    if(err < 0)
        printk(KERN_INFO "[%s] kobject_uevent_env error = %d\n", __func__, err);
    
    return 0;
}

#ifdef LINUX310
static const struct file_operations autok_proc_ops = {
    .owner      = THIS_MODULE,
    .read       = autok_readproc,
    .write      = autok_writeproc,
};
#endif

static int autok_module_init(void)
{
#ifdef LINUX310
    s_proc = proc_create(PROC_AUTOK_NAME, 0660, NULL, &autok_proc_ops);
#else
    s_proc = create_proc_entry(PROC_AUTOK_NAME, 0660, NULL);
#endif
    
    if (s_proc == NULL) {
		remove_proc_entry(PROC_AUTOK_NAME, NULL);
		printk(KERN_ALERT "Error: Could not initialize /proc/%s\n",
			PROC_AUTOK_NAME);
		return -ENOMEM;
	}
#ifndef LINUX310
    s_proc->write_proc = autok_writeproc;
    s_proc->read_proc = autok_readproc;
#endif
    s_proc->gid = 1000;
    
    printk(KERN_INFO "/proc/%s created\n", PROC_AUTOK_NAME);

#ifdef USE_KERNEL_THREAD    
    //task = kthread_create(&autok_thread_func,(void *)(&g_autok_thread_data),"autokp");
#else   // USE_KERNEL_THREAD
    g_autok_wq = create_workqueue("autok_queue");
    INIT_DELAYED_WORK((struct delayed_work *)(&g_autok_thread_data), autok_thread_func);
#endif  // USE_KERNEL_THREAD
    
	return 0;	/* everything is ok */
}

static void autok_module_exit(void)
{
    remove_proc_entry(PROC_AUTOK_NAME, NULL);

#ifdef USE_KERNEL_THREAD
	//ret = kthread_stop(task);
#else   // USE_KERNEL_THREAD
    flush_workqueue(g_autok_wq);
    destroy_workqueue(g_autok_wq);
#endif  // USE_KERNEL_THREAD

    printk(KERN_INFO "/proc/%s removed\n", PROC_AUTOK_NAME);
}

module_init(autok_module_init);
module_exit(autok_module_exit);

MODULE_AUTHOR("MediaTek Inc.");
MODULE_DESCRIPTION("MediaTek SDIO Auto-K Proc");
MODULE_LICENSE("GPL");