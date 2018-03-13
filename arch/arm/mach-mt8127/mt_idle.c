#define __MT_IDLE_C__

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/cpu.h>

#include <linux/types.h>
#include <linux/string.h>
#include <mach/mt_cirq.h>
#include <asm/system_misc.h>

#include <mach/mt_typedefs.h>
#include <mach/sync_write.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_dcm.h>
#include <mach/mt_gpt.h>
#include <mach/mt_spm_idle.h>
#include <mach/mt_spm_sleep.h>
#include <mach/hotplug.h>
#include <mach/mt_cpufreq.h>
#include <mach/mt_power_gs.h>
#include <mach/mt_ptp.h>

#include <mach/mt_idle.h>

#define USING_XLOG

#ifdef USING_XLOG
#include <linux/xlog.h>

#define TAG     "Power/swap"

#define idle_err(fmt, args...)       \
	xlog_printk(ANDROID_LOG_ERROR, TAG, fmt, ##args)
#define idle_warn(fmt, args...)      \
	xlog_printk(ANDROID_LOG_WARN, TAG, fmt, ##args)
#define idle_info(fmt, args...)      \
	xlog_printk(ANDROID_LOG_INFO, TAG, fmt, ##args)
#define idle_dbg(fmt, args...)       \
	xlog_printk(ANDROID_LOG_DEBUG, TAG, fmt, ##args)
#define idle_ver(fmt, args...)       \
	xlog_printk(ANDROID_LOG_VERBOSE, TAG, fmt, ##args)

#else /* !USING_XLOG */

#define TAG     "[Power/swap] "

#define idle_err(fmt, args...)       \
	pr_err(TAG fmt, ##args)
#define idle_warn(fmt, args...)      \
	pr_warn(TAG fmt, ##args)
#define idle_info(fmt, args...)      \
	pr_notice(TAG fmt, ##args)
#define idle_dbg(fmt, args...)       \
	pr_info(TAG fmt, ##args)
#define idle_ver(fmt, args...)       \
	pr_debug(TAG fmt, ##args)

#endif


#define idle_readl(addr) \
	DRV_Reg32(addr)

#define idle_writel(addr, val)   \
	mt65xx_reg_sync_writel(val, addr)

#define idle_setl(addr, val) \
	mt65xx_reg_sync_writel(idle_readl(addr) | (val), addr)

#define idle_clrl(addr, val) \
	mt65xx_reg_sync_writel(idle_readl(addr) & ~(val), addr)


#define INVALID_GRP_ID(grp) (grp < 0 || grp >= NR_GRPS)
bool __attribute__((weak))
clkmgr_idle_can_enter(unsigned int *condition_mask, unsigned int *block_mask)
{
	return false;
}

enum {
	IDLE_TYPE_SO = 0,
	IDLE_TYPE_DP = 1,
	IDLE_TYPE_SL = 2,
	IDLE_TYPE_RG = 3,
	NR_TYPES = 4,
};

enum {
	BY_CPU = 0,
	BY_CLK = 1,
	BY_TMR = 2,
	BY_OTH = 3,
	BY_VTG = 4,
	NR_REASONS = 5
};

static const char *idle_name[NR_TYPES] = {
	"soidle",
	"dpidle",
	"slidle",
	"rgidle",
};

static const char *reason_name[NR_REASONS] = {
	"by_cpu",
	"by_clk",
	"by_tmr",
	"by_oth",
	"by_vtg",
};

static int idle_switch[NR_TYPES] = {
	1,  /* soidle switch */
	1,  /* dpidle switch */
	1,  /* slidle switch */
	1,  /* rgidle switch */
};

/************************************************
 * SODI part
 ************************************************/
#ifdef SPM_SODI_ENABLED

static unsigned int soidle_gpt_percpu[NR_CPUS] = {
	GPT4,
	GPT1,
	GPT4,
	GPT5,
};

static unsigned int soidle_condition_mask[NR_GRPS] = {
	0x11eee5c1, /* PERI0: */
	0x00000007, /* PERI1: */
	0x00000000, /* INFRA: */
	0x00000000, /* TOPCK: */
	0x000a4cc4, /* DISP0: */
	0x00000000, /* DISP1: */
	0x000003e1, /* IMAGE: */
	0x00000001, /* MFG:   */
	0x00000040, /* AUDIO: */
	0x00000000, /* VDEC0: */
	0x00000001, /* VDEC1: */
};

static unsigned int soidle_block_mask[NR_GRPS] = {0x0};

static unsigned int soidle_timer_left[NR_CPUS];
static unsigned int soidle_timer_left2[NR_CPUS];
static unsigned int soidle_time_critera = 13000; /* 1ms */


static unsigned long soidle_cnt[NR_CPUS] = {0};
static unsigned long soidle_block_cnt[NR_CPUS][NR_REASONS] = { {0} };

static DEFINE_MUTEX(soidle_locked);

static void enable_soidle_by_mask(int grp, unsigned int mask)
{
	mutex_lock(&soidle_locked);
	soidle_condition_mask[grp] &= ~mask;
	mutex_unlock(&soidle_locked);
}

static void disable_soidle_by_mask(int grp, unsigned int mask)
{
	mutex_lock(&soidle_locked);
	soidle_condition_mask[grp] |= mask;
	mutex_unlock(&soidle_locked);
}

void enable_soidle_by_bit(int id)
{
	int grp = clk_id_to_grp_id(id);
	unsigned int mask = clk_id_to_mask(id);
	BUG_ON(INVALID_GRP_ID(grp));
	enable_soidle_by_mask(grp, mask);
}
EXPORT_SYMBOL(enable_soidle_by_bit);

void disable_soidle_by_bit(int id)
{
	int grp = clk_id_to_grp_id(id);
	unsigned int mask = clk_id_to_mask(id);
	BUG_ON(INVALID_GRP_ID(grp));
	disable_soidle_by_mask(grp, mask);
}
EXPORT_SYMBOL(disable_soidle_by_bit);


bool soidle_can_enter(int cpu)
{
	int reason = NR_REASONS;

	return false;

	if (TRUE == gSpm_IsLcmVideoMode) {
		reason = BY_OTH;
		goto out;
	}

	if (gSPM_SODI_EN != 0) {
		reason = BY_OTH;
		goto out;
	}

	/*if hotplug-ing, can't enter sodi avoid bootslave corrupt*/
	if (atomic_read(&is_in_hotplug) >= 1) {
		reason = BY_CPU;
		goto out;
	}

	if (atomic_read(&hotplug_cpu_count) != 1) {
		reason = BY_CPU;
		goto out;
	}

	if (cpu == 0) {
		memset(soidle_block_mask, 0, NR_GRPS * sizeof(unsigned int));
		if (!clkmgr_idle_can_enter(soidle_condition_mask, soidle_block_mask)) {
			reason = BY_CLK;
			goto out;
		}
	}

	soidle_timer_left[cpu] = localtimer_get_counter();
	if (soidle_timer_left[cpu] < soidle_time_critera ||
			((int)soidle_timer_left[cpu]) < 0) {
		reason = BY_TMR;
		goto out;
	}

out:
	if (reason < NR_REASONS) {
		soidle_block_cnt[cpu][reason]++;
		return false;
	} else
		return true;
}

void soidle_before_wfi(int cpu)
{
	int err = 0;
	unsigned int id = soidle_gpt_percpu[cpu];

	free_gpt(id);
	err = request_gpt(id, GPT_ONE_SHOT, GPT_CLK_SRC_SYS, GPT_CLK_DIV_1,
				0, NULL, GPT_NOAUTOEN);
	if (err)
		idle_info("[%s]fail to request GPT4\n", __func__);

	soidle_timer_left2[cpu] = localtimer_get_counter();


	if ((int)soidle_timer_left2[cpu] <= 0)
		gpt_set_cmp(id, 1); /* Trigger GPT4 Timerout imediately */
	else
		gpt_set_cmp(id, soidle_timer_left2[cpu]);

	start_gpt(id);

}

void soidle_after_wfi(int cpu)
{
	unsigned int id = soidle_gpt_percpu[cpu];

		if (gpt_check_and_ack_irq(id))
			localtimer_set_next_event(1);
		else {
			/* waked up by other wakeup source */
			unsigned int cnt, cmp;
			gpt_get_cnt(id, &cnt);
			gpt_get_cmp(id, &cmp);
			if (unlikely(cmp < cnt)) {
				idle_err("[%s]GPT%d: counter = %10u, compare = %10u\n", __func__,
						id + 1, cnt, cmp);
				BUG();
			}

			localtimer_set_next_event(cmp-cnt);
			stop_gpt(id);
		}

	soidle_cnt[cpu]++;
}

#endif /* SPM_SODI_ENABLED */

/************************************************
 * deep idle part
 ************************************************/
static unsigned int dpidle_condition_mask[NR_GRPS] = {
	0x1bfd0ffd, /* PERI0: */
	0x00000007, /* PERI1: */
	0x0000a080, /* INFRA: */
	0x00000000, /* TOPCK: */
	0x001fffff, /* DISP0: */
	0x00003fff, /* DISP1: */
	0x000003e1, /* IMAGE: */
	0x00000001, /* MFG:   */
	0x00000000, /* AUDIO: */
	0x00000001, /* VDEC0: */
	0x00000001, /* VDEC1: */
};

static unsigned int dpidle_block_mask[NR_GRPS] = {0x0};


static unsigned int dpidle_timer_left;
static unsigned int dpidle_timer_left2;
static unsigned int dpidle_time_critera = 26000;

static unsigned long dpidle_cnt[NR_CPUS] = {0};
static unsigned long dpidle_block_cnt[NR_REASONS] = {0};

static DEFINE_MUTEX(dpidle_locked);

static void enable_dpidle_by_mask(int grp, unsigned int mask)
{
	mutex_lock(&dpidle_locked);
	dpidle_condition_mask[grp] &= ~mask;
	mutex_unlock(&dpidle_locked);
}

static void disable_dpidle_by_mask(int grp, unsigned int mask)
{
	mutex_lock(&dpidle_locked);
	dpidle_condition_mask[grp] |= mask;
	mutex_unlock(&dpidle_locked);
}

void enable_dpidle_by_bit(int id)
{
	int grp = clk_id_to_grp_id(id);
	unsigned int mask = clk_id_to_mask(id);
	BUG_ON(INVALID_GRP_ID(grp));
	enable_dpidle_by_mask(grp, mask);
}
EXPORT_SYMBOL(enable_dpidle_by_bit);

void disable_dpidle_by_bit(int id)
{
	int grp = clk_id_to_grp_id(id);
	unsigned int mask = clk_id_to_mask(id);
	BUG_ON(INVALID_GRP_ID(grp));
	disable_dpidle_by_mask(grp, mask);
}
EXPORT_SYMBOL(disable_dpidle_by_bit);


static bool dpidle_can_enter(void)
{
	int reason = NR_REASONS;

#ifdef SPM_SODI_ENABLED
	if (gSPM_SODI_EN != 0) {
		reason = BY_OTH;
		goto out;
	}
#endif

	if (!mt_cpufreq_earlysuspend_status_get()) {
		reason = BY_VTG;
		goto out;
	}

	if (atomic_read(&hotplug_cpu_count) != 1) {
		reason = BY_CPU;
		goto out;
	}

	memset(dpidle_block_mask, 0, NR_GRPS * sizeof(unsigned int));
	if (!clkmgr_idle_can_enter(dpidle_condition_mask, dpidle_block_mask)) {
		reason = BY_CLK;
		goto out;
	}

	dpidle_timer_left = localtimer_get_counter();
	if (dpidle_timer_left < dpidle_time_critera ||
			((int)dpidle_timer_left) < 0) {
		reason = BY_TMR;
		goto out;
	}

out:
	if (reason < NR_REASONS) {
		dpidle_block_cnt[reason]++;
		return false;
	} else
		return true;
}

static unsigned int clk_cfg_3;

#define faudintbus_pll2sq() \
	do { \
		clk_cfg_3 = idle_readl(CLK_CFG_3); \
		idle_writel(CLK_CFG_3, clk_cfg_3 & 0xF8FFFFFF); \
	} while (0)

#define faudintbus_sq2pll() \
	idle_writel(CLK_CFG_3, clk_cfg_3)


void spm_dpidle_before_wfi(void)
{

	mt_power_gs_dump_dpidle();

	bus_dcm_enable();

	faudintbus_pll2sq();

#if 0
		dpidle_timer_left = localtimer_get_counter();
		gpt_set_cmp(GPT4, dpidle_timer_left);
#else
		dpidle_timer_left2 = localtimer_get_counter();
		gpt_set_cmp(GPT4, dpidle_timer_left2);
#endif
		start_gpt(GPT4);

}

void spm_dpidle_after_wfi(void)
{
#if 0
	idle_info("[%s]timer_left=%u, timer_left2=%u, delta=%u\n",
		dpidle_timer_left, dpidle_timer_left2, dpidle_timer_left-dpidle_timer_left2);
#endif

	if (gpt_check_and_ack_irq(GPT4)) {
		/* waked up by WAKEUP_GPT */
		localtimer_set_next_event(1);
	} else {
		/* waked up by other wakeup source */
		unsigned int cnt, cmp;
		gpt_get_cnt(GPT4, &cnt);
		gpt_get_cmp(GPT4, &cmp);
		if (unlikely(cmp < cnt)) {
			idle_err("[%s]GPT%d: counter = %10u, compare = %10u\n", __func__,
					GPT4 + 1, cnt, cmp);
			BUG();
		}

		localtimer_set_next_event(cmp-cnt);
		stop_gpt(GPT4);
	}

	faudintbus_sq2pll();

	bus_dcm_disable();

	dpidle_cnt[0]++;
}


/************************************************
 * slow idle part
 ************************************************/
static unsigned int slidle_condition_mask[NR_GRPS] = {
	0x11e01000, /* PERI0: */
	0x00000000, /* PERI1: */
	0x00000000, /* INFRA: */
	0x00000000, /* TOPCK: */
	0x00000000, /* DISP0: */
	0x00000000, /* DISP1: */
	0x00000000, /* IMAGE: */
	0x00000000, /* MFG:   */
	0x00000000, /* AUDIO: */
	0x00000000, /* VDEC0: */
	0x00000000, /* VDEC1: */
};
/* */
static unsigned int slidle_block_mask[NR_GRPS] = {0x0};

static unsigned long slidle_cnt[NR_CPUS] = {0};
static unsigned long slidle_block_cnt[NR_REASONS] = {0};

static DEFINE_MUTEX(slidle_locked);


static void enable_slidle_by_mask(int grp, unsigned int mask)
{
	mutex_lock(&slidle_locked);
	slidle_condition_mask[grp] &= ~mask;
	mutex_unlock(&slidle_locked);
}

static void disable_slidle_by_mask(int grp, unsigned int mask)
{
	mutex_lock(&slidle_locked);
	slidle_condition_mask[grp] |= mask;
	mutex_unlock(&slidle_locked);
}

void enable_slidle_by_bit(int id)
{
	int grp = clk_id_to_grp_id(id);
	unsigned int mask = clk_id_to_mask(id);
	BUG_ON(INVALID_GRP_ID(grp));
	enable_slidle_by_mask(grp, mask);
}
EXPORT_SYMBOL(enable_slidle_by_bit);

void disable_slidle_by_bit(int id)
{
	int grp = clk_id_to_grp_id(id);
	unsigned int mask = clk_id_to_mask(id);
	BUG_ON(INVALID_GRP_ID(grp));
	disable_slidle_by_mask(grp, mask);
}
EXPORT_SYMBOL(disable_slidle_by_bit);

static bool slidle_can_enter(void)
{
	int reason = NR_REASONS;
	if (atomic_read(&hotplug_cpu_count) != 1) {
		reason = BY_CPU;
		goto out;
	}

	memset(slidle_block_mask, 0, NR_GRPS * sizeof(unsigned int));
	if (!clkmgr_idle_can_enter(slidle_condition_mask, slidle_block_mask)) {
		reason = BY_CLK;
		goto out;
	}

#if EN_PTP_OD
	if (ptp_data[0]) {
		reason = BY_OTH;
		goto out;
	}
#endif

out:
	if (reason < NR_REASONS) {
		slidle_block_cnt[reason]++;
		return false;
	} else {
		return true;
	}
}

static void slidle_before_wfi(int cpu)
{
	bus_dcm_enable();
}

static void slidle_after_wfi(int cpu)
{
	bus_dcm_disable();

	slidle_cnt[cpu]++;
}

static void go_to_slidle(int cpu)
{
	slidle_before_wfi(cpu);

	dsb();
	__asm__ __volatile__("wfi" : : : "memory");

	slidle_after_wfi(cpu);
}


/************************************************
 * regular idle part
 ************************************************/
static unsigned long rgidle_cnt[NR_CPUS] = {0};

static void rgidle_before_wfi(int cpu)
{
	mt_power_gs_dump_idle();
}

static void rgidle_after_wfi(int cpu)
{
	rgidle_cnt[cpu]++;
}

static noinline void go_to_rgidle(int cpu)
{
	rgidle_before_wfi(cpu);

	dsb();
	__asm__ __volatile__("wfi" : : : "memory");

	rgidle_after_wfi(cpu);
}

/************************************************
 * idle task flow part
 ************************************************/

/*
 * xxidle_handler return 1 if enter and exit the low power state
 */
#ifdef SPM_SODI_ENABLED

static int sodi_cpu_pdn = 1;
static inline int soidle_handler(int cpu)
{
	if (idle_switch[IDLE_TYPE_SO]) {
		if (soidle_can_enter(cpu)) {
			spm_go_to_sodi(sodi_cpu_pdn);
			return 1;
		}
	}

	return 0;
}
#else
static inline int soidle_handler(int cpu)
{
	return 0;
}
#endif

static int dpidle_cpu_pdn = 1;

static inline int dpidle_handler(int cpu)
{
	int ret = 0;
	if (idle_switch[IDLE_TYPE_DP]) {
		if (dpidle_can_enter()) {
			spm_go_to_dpidle(dpidle_cpu_pdn, 0);
			ret = 1;
		}
	}

	return ret;
}

static inline int slidle_handler(int cpu)
{
	int ret = 0;
	if (idle_switch[IDLE_TYPE_SL]) {
		if (slidle_can_enter()) {
			go_to_slidle(cpu);
			ret = 1;
		}
	}

	return ret;
}

static inline int rgidle_handler(int cpu)
{
	int ret = 0;
	if (idle_switch[IDLE_TYPE_RG]) {
		go_to_rgidle(cpu);
		ret = 1;
	}

	return ret;
}

static int (*idle_handlers[NR_TYPES])(int) = {
	soidle_handler,
	dpidle_handler,
	slidle_handler,
	rgidle_handler,
};


void arch_idle(void)
{
	int cpu = smp_processor_id();
	int i;

	for (i = 0; i < NR_TYPES; i++) {
		if (idle_handlers[i](cpu))
			break;
	}
}

#define idle_attr(_name)                         \
static struct kobj_attribute _name##_attr = {   \
	.attr = {                                   \
		.name = __stringify(_name),             \
		.mode = 0644,                           \
	},                                          \
	.show = _name##_show,                       \
	.store = _name##_store,                     \
}


#ifdef SPM_SODI_ENABLED
static ssize_t soidle_state_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	int len = 0;
	char *p = buf;

	int cpus, reason, i;

	p += sprintf(p, "*********** screen on idle state ************\n");
	p += sprintf(p, "soidle_time_critera=%u\n", soidle_time_critera);

	for (cpus = 0; cpus < nr_cpu_ids; cpus++) {
		p += sprintf(p, "cpu:%d\n", cpus);
		for (reason = 0; reason < NR_REASONS; reason++) {
			p += sprintf(p, "[%d]soidle_block_cnt[%s]=%lu\n", reason,
					reason_name[reason], soidle_block_cnt[cpus][reason]);
		}
		p += sprintf(p, "\n");
	}

for (i = 0; i < NR_GRPS; i++) {
		p += sprintf(p, "[%02d]soidle_condition_mask[%-8s]=0x%08x\t\tsoidle_block_mask[%-8s]=0x%08x\n", i,
				grp_get_name(i), soidle_condition_mask[i],
				grp_get_name(i), soidle_block_mask[i]);
	}

	p += sprintf(p, "\n********** soidle command help **********\n");
	p += sprintf(p, "soidle help:   cat /sys/power/soidle_state\n");
	p += sprintf(p, "switch on/off: echo [soidle] 1/0 > /sys/power/soidle_state\n");
	p += sprintf(p, "en_so_by_bit:  echo enable id > /sys/power/soidle_state\n");
	p += sprintf(p, "dis_so_by_bit: echo disable id > /sys/power/soidle_state\n");
	p += sprintf(p, "modify tm_cri: echo time value(dec) > /sys/power/soidle_state\n");

	len = p - buf;
	return len;
}

static ssize_t soidle_state_store(struct kobject *kobj,
				struct kobj_attribute *attr, const char *buf, size_t n)
{
	char cmd[32];
	int param;

	if (sscanf(buf, "%s %d", cmd, &param) == 2) {
		if (!strcmp(cmd, "soidle"))
			idle_switch[IDLE_TYPE_SO] = param;
		else if (!strcmp(cmd, "enable"))
			enable_soidle_by_bit(param);
		else if (!strcmp(cmd, "disable"))
			disable_soidle_by_bit(param);
		else if (!strcmp(cmd, "time"))
			soidle_time_critera = param;

		return n;
	} else if (sscanf(buf, "%d", &param) == 1) {
		idle_switch[IDLE_TYPE_SO] = param;
		return n;
	}

	return -EINVAL;
}
idle_attr(soidle_state);
#endif

static ssize_t dpidle_state_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	int len = 0;
	char *p = buf;

	int i;

	p += sprintf(p, "*********** deep idle state ************\n");
	p += sprintf(p, "dpidle_cpu_pdn = %d\n", dpidle_cpu_pdn);
	p += sprintf(p, "dpidle_time_critera=%u\n", dpidle_time_critera);

	for (i = 0; i < NR_REASONS; i++) {
		p += sprintf(p, "[%d]dpidle_block_cnt[%s]=%lu\n", i, reason_name[i],
				dpidle_block_cnt[i]);
	}

	p += sprintf(p, "\n");

	for (i = 0; i < NR_GRPS; i++) {
		p += sprintf(p, "[%02d]dpidle_condition_mask[%-8s]=0x%08x\t\tdpidle_block_mask[%-8s]=0x%08x\n", i,
				grp_get_name(i), dpidle_condition_mask[i],
				grp_get_name(i), dpidle_block_mask[i]);
	}

	p += sprintf(p, "\n*********** dpidle command help  ************\n");
	p += sprintf(p, "dpidle help:   cat /sys/power/dpidle_state\n");
	p += sprintf(p, "switch on/off: echo [dpidle] 1/0 > /sys/power/dpidle_state\n");
	p += sprintf(p, "cpupdn on/off: echo cpupdn 1/0 > /sys/power/dpidle_state\n");
	p += sprintf(p, "en_dp_by_bit:  echo enable id > /sys/power/dpidle_state\n");
	p += sprintf(p, "dis_dp_by_bit: echo disable id > /sys/power/dpidle_state\n");
	p += sprintf(p, "modify tm_cri: echo time value(dec) > /sys/power/dpidle_state\n");

	len = p - buf;
	return len;
}

static ssize_t dpidle_state_store(struct kobject *kobj,
				struct kobj_attribute *attr, const char *buf, size_t n)
{
	char cmd[32];
	int param;

	if (sscanf(buf, "%s %d", cmd, &param) == 2) {
		if (!strcmp(cmd, "dpidle"))
			idle_switch[IDLE_TYPE_DP] = param;
		else if (!strcmp(cmd, "enable"))
			enable_dpidle_by_bit(param);
		else if (!strcmp(cmd, "disable"))
			disable_dpidle_by_bit(param);
		else if (!strcmp(cmd, "cpupdn"))
			dpidle_cpu_pdn = !!param;
		else if (!strcmp(cmd, "time"))
			dpidle_time_critera = param;

		return n;
	} else if (sscanf(buf, "%d", &param) == 1) {
		idle_switch[IDLE_TYPE_DP] = param;
		return n;
	}

	return -EINVAL;
}
idle_attr(dpidle_state);

static ssize_t slidle_state_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	int len = 0;
	char *p = buf;

	int i;

	p += sprintf(p, "*********** slow idle state ************\n");
	for (i = 0; i < NR_REASONS; i++) {
		p += sprintf(p, "[%d]slidle_block_cnt[%s]=%lu\n",
				i, reason_name[i], slidle_block_cnt[i]);
	}

	p += sprintf(p, "\n");

	for (i = 0; i < NR_GRPS; i++) {
		p += sprintf(p, "[%02d]slidle_condition_mask[%-8s]=0x%08x\t\tslidle_block_mask[%-8s]=0x%08x\n", i,
				grp_get_name(i), slidle_condition_mask[i],
				grp_get_name(i), slidle_block_mask[i]);
	}


	p += sprintf(p, "\n********** slidle command help **********\n");
	p += sprintf(p, "slidle help:   cat /sys/power/slidle_state\n");
	p += sprintf(p, "switch on/off: echo [slidle] 1/0 > /sys/power/slidle_state\n");

	len = p - buf;
	return len;
}

static ssize_t slidle_state_store(struct kobject *kobj,
				struct kobj_attribute *attr, const char *buf, size_t n)
{
	char cmd[32];
	int param;

	if (sscanf(buf, "%s %d", cmd, &param) == 2) {
		if (!strcmp(cmd, "slidle"))
			idle_switch[IDLE_TYPE_SL] = param;
		else if (!strcmp(cmd, "enable"))
			enable_slidle_by_bit(param);
		else if (!strcmp(cmd, "disable"))
			disable_slidle_by_bit(param);

		return n;
	} else if (sscanf(buf, "%d", &param) == 1) {
		idle_switch[IDLE_TYPE_SL] = param;
		return n;
	}

	return -EINVAL;
}
idle_attr(slidle_state);

static ssize_t rgidle_state_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	int len = 0;
	char *p = buf;

	p += sprintf(p, "*********** regular idle state ************\n");
	p += sprintf(p, "\n********** rgidle command help **********\n");
	p += sprintf(p, "rgidle help:   cat /sys/power/rgidle_state\n");
	p += sprintf(p, "switch on/off: echo [rgidle] 1/0 > /sys/power/rgidle_state\n");

	len = p - buf;
	return len;
}

static ssize_t rgidle_state_store(struct kobject *kobj,
				struct kobj_attribute *attr, const char *buf, size_t n)
{
	char cmd[32];
	int param;

	if (sscanf(buf, "%s %d", cmd, &param) == 2) {
		if (!strcmp(cmd, "rgidle"))
			idle_switch[IDLE_TYPE_RG] = param;

		return n;
	} else if (sscanf(buf, "%d", &param) == 1) {
		idle_switch[IDLE_TYPE_RG] = param;
		return n;
	}

	return -EINVAL;
}
idle_attr(rgidle_state);

static ssize_t idle_state_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	int len = 0;
	char *p = buf;

	int i;

	p += sprintf(p, "********** idle state dump **********\n");
#ifdef SPM_SODI_ENABLED
	for (i = 0; i < nr_cpu_ids; i++) {
		p += sprintf(p, "soidle_cnt[%d]=%lu, dpidle_cnt[%d]=%lu, slidle_cnt[%d]=%lu, rgidle_cnt[%d]=%lu\n",
				i, soidle_cnt[i], i, dpidle_cnt[i],
				i, slidle_cnt[i], i, rgidle_cnt[i]);
	}
#else
	for (i = 0; i < nr_cpu_ids; i++) {
		p += sprintf(p, "dpidle_cnt[%d]=%lu, slidle_cnt[%d]=%lu, rgidle_cnt[%d]=%lu\n",
				i, dpidle_cnt[i], i, slidle_cnt[i], i, rgidle_cnt[i]);
	}
#endif

	p += sprintf(p, "\n********** variables dump **********\n");
	for (i = 0; i < NR_TYPES; i++)
		p += sprintf(p, "%s_switch=%d, ", idle_name[i], idle_switch[i]);
	p += sprintf(p, "\n");

	p += sprintf(p, "\n********** idle command help **********\n");
	p += sprintf(p, "status help:   cat /sys/power/idle_state\n");
	p += sprintf(p, "switch on/off: echo switch mask > /sys/power/idle_state\n");

#ifdef SPM_SODI_ENABLED
	p += sprintf(p, "soidle help:   cat /sys/power/soidle_state\n");
#else
	p += sprintf(p, "soidle help:   soidle is unavailable\n");
#endif
	p += sprintf(p, "dpidle help:   cat /sys/power/dpidle_state\n");
	p += sprintf(p, "slidle help:   cat /sys/power/slidle_state\n");
	p += sprintf(p, "rgidle help:   cat /sys/power/rgidle_state\n");

	len = p - buf;
	return len;
}

static ssize_t idle_state_store(struct kobject *kobj,
				struct kobj_attribute *attr, const char *buf, size_t n)
{
	char cmd[32];
	int idx;
	int param;

	if (sscanf(buf, "%s %x", cmd, &param) == 2) {
		if (!strcmp(cmd, "switch")) {
			for (idx = 0; idx < NR_TYPES; idx++) {
#ifndef SPM_SODI_ENABLED
				if (idx == IDLE_TYPE_SO)
					continue;
#endif
				idle_switch[idx] = (param & (1U << idx)) ? 1 : 0;
			}
		}
		return n;
	}

	return -EINVAL;
}
idle_attr(idle_state);


void mt_idle_init(void)
{
	int err = 0;

	idle_info("[%s]entry!!\n", __func__);
	arm_pm_idle = arch_idle;

#ifndef SPM_SODI_ENABLED
	idle_switch[IDLE_TYPE_SO] = 0;
#endif

	err = free_gpt(GPT1);
	if (err)
		idle_info("[%s]fail to free GPT1\n", __func__);

	err = request_gpt(GPT1, GPT_ONE_SHOT, GPT_CLK_SRC_SYS, GPT_CLK_DIV_1,
				0, NULL, GPT_NOAUTOEN);
	if (err)
		idle_info("[%s]fail to request GPT1\n", __func__);

	err = request_gpt(GPT4, GPT_ONE_SHOT, GPT_CLK_SRC_SYS, GPT_CLK_DIV_1,
				0, NULL, GPT_NOAUTOEN);
	if (err)
		idle_info("[%s]fail to request GPT4\n", __func__);

	err = request_gpt(GPT5, GPT_ONE_SHOT, GPT_CLK_SRC_SYS, GPT_CLK_DIV_1,
				0, NULL, GPT_NOAUTOEN);
	if (err)
		idle_info("[%s]fail to request GPT5\n", __func__);

	err = sysfs_create_file(power_kobj, &idle_state_attr.attr);
#ifdef SPM_SODI_ENABLED
	err |= sysfs_create_file(power_kobj, &soidle_state_attr.attr);
#endif
	err |= sysfs_create_file(power_kobj, &dpidle_state_attr.attr);
	err |= sysfs_create_file(power_kobj, &slidle_state_attr.attr);
	err |= sysfs_create_file(power_kobj, &rgidle_state_attr.attr);

	if (err)
		idle_err("[%s]: fail to create sysfs\n", __func__);
}
