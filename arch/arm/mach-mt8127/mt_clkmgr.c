#define __MT_CLKMGR_C__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>

#include <asm/uaccess.h>

#include <linux/device.h>
#include <linux/platform_device.h>

#include <mach/mt_typedefs.h>
#include <mach/sync_write.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_dcm.h>
#include <mach/mt_spm.h>
#include <mach/mt_spm_mtcmos.h>
#include <mach/mt_spm_sleep.h>
#include <mach/mt_freqhopping.h>
#include <mach/mt_gpufreq.h>
#include <linux/earlysuspend.h>


#define DISABLE_BUG_ON_CNT      0
#define WORKAROUND_DPI0_MUX     1
#define WORKAROUND_DAPC         1

#if CLKMGR_8127
	#include <linux/fs.h>
	#include <linux/seq_file.h>
#endif /* CLKMGR_8127 */

/* #define CONFIG_CLKMGR_BRINGUP */

/* #define CLK_LOG_TOP */
/* #define CLK_LOG */
/* #define SYS_LOG */
/* #define MUX_LOG_TOP */
#define MUX_LOG
/* #define PLL_LOG_TOP */
#define PLL_LOG

/************************************************
 **********         log debug          **********
 ************************************************/

#define USING_XLOG

#ifdef USING_XLOG
#include <linux/xlog.h>

#define TAG     "Power/clkmgr"

#define clk_err(fmt, args...)       \
	xlog_printk(ANDROID_LOG_ERROR, TAG, fmt, ##args)
#define clk_warn(fmt, args...)      \
	xlog_printk(ANDROID_LOG_WARN, TAG, fmt, ##args)
#define clk_info(fmt, args...)      \
	xlog_printk(ANDROID_LOG_INFO, TAG, fmt, ##args)
#define clk_dbg(fmt, args...)       \
	xlog_printk(ANDROID_LOG_DEBUG, TAG, fmt, ##args)
#define clk_ver(fmt, args...)       \
	xlog_printk(ANDROID_LOG_VERBOSE, TAG, fmt, ##args)

#else

#define TAG     "[Power/clkmgr] "

#define clk_err(fmt, args...)       \
	pr_err(TAG fmt, ##args)
#define clk_warn(fmt, args...)      \
	pr_warn(TAG fmt, ##args)
#define clk_info(fmt, args...)      \
	pr_notice(TAG fmt, ##args)
#define clk_dbg(fmt, args...)       \
	pr_info(TAG fmt, ##args)
#define clk_ver(fmt, args...)       \
	pr_debug(TAG fmt, ##args)

#endif



/************************************************
 **********     macro / functions      **********
 ************************************************/

#define BITS(_bits_, _val_) ((BIT((1 ? _bits_) + 1) - BIT((0 ? _bits_))) & (_val_ << (0 ? _bits_)))
#define BITMASK(_bits_)     (BIT((1 ? _bits_) + 1) - BIT((0 ? _bits_)))
#define ABS_DIFF(a, b)      ((a) > (b) ? (a) - (b) : (b) - (a))


/************************************************
 **********      register access       **********
 ************************************************/

#define clk_readl(addr) \
	DRV_Reg32(addr)

#define clk_writel(addr, val)   \
	mt65xx_reg_sync_writel(val, addr)

#define clk_setl(addr, val) \
	mt65xx_reg_sync_writel(clk_readl(addr) | (val), addr)

#define clk_clrl(addr, val) \
	mt65xx_reg_sync_writel(clk_readl(addr) & ~(val), addr)

#define clk_writel_mask(addr, mask, val) \
	clk_writel(addr, (clk_readl(addr) & ~(mask)) | (val))



/************************************************
 **********      struct definition     **********
 ************************************************/

/* #define CONFIG_CLKMGR_STAT */

struct pll;
struct pll_ops {
	int (*get_state)(struct pll *pll);
	void (*enable)(struct pll *pll);
	void (*disable)(struct pll *pll);
	void (*fsel)(struct pll *pll, unsigned int value);
	int (*dump_regs)(struct pll *pll, unsigned int *ptr);
	unsigned int (*vco_calc)(struct pll *pll);
	int (*hp_enable)(struct pll *pll);
	int (*hp_disable)(struct pll *pll);
#if CLKMGR_8127
	unsigned int (*get_postdiv)(struct pll *pll);
	void (*set_postdiv)(struct pll *pll, unsigned int postdiv);
	unsigned int (*get_pcw)(struct pll *pll);
	void (*set_pcw)(struct pll *pll, unsigned int pcw);
	int (*set_freq)(struct pll *pll, unsigned int freq);
#endif /* CLKMGR_8127 */
};

struct pll {
	const char *name;
	int type;
	int mode;
	int feat;
	int state;
	unsigned int cnt;
	unsigned int en_mask;
	unsigned int base_addr;
	unsigned int pwr_addr;
	struct pll_ops *ops;
	unsigned int hp_id;
	int hp_switch;
#ifdef CONFIG_CLKMGR_STAT
	struct list_head head;
#endif
};


struct subsys;
struct subsys_ops {
	int (*enable)(struct subsys *sys);
	int (*disable)(struct subsys *sys);
	int (*get_state)(struct subsys *sys);
	int (*dump_regs)(struct subsys *sys, unsigned int *ptr);
};

struct subsys {
	const char *name;
	int type;
	int force_on;
	unsigned int cnt;
	unsigned int state;
	unsigned int default_sta;
	unsigned int sta_mask;  /* mask in PWR_STATUS */
	unsigned int ctl_addr;
	struct subsys_ops *ops;
	struct cg_grp *start;
	unsigned int nr_grps;
	struct clkmux *mux;
#ifdef CONFIG_CLKMGR_STAT
	struct list_head head;
#endif
};


struct clkmux;
struct clkmux_ops {
	void (*sel)(struct clkmux *mux, unsigned int clksrc);
	void (*enable)(struct clkmux *mux);
	void (*disable)(struct clkmux *mux);
};

struct clkmux {
	const char *name;
	unsigned int cnt;
	unsigned int base_addr;
	unsigned int sel_mask;
	unsigned int pdn_mask;
	unsigned int offset;
	unsigned int nr_inputs;
	struct clkmux_ops *ops;
	struct clkmux *parent;
	struct clkmux *siblings;
	struct pll *pll;
#ifdef CONFIG_CLKMGR_STAT
	struct list_head head;
#endif
};


struct cg_grp;
struct cg_grp_ops {
	int (*prepare)(struct cg_grp *grp);
	int (*finished)(struct cg_grp *grp);
	unsigned int (*get_state)(struct cg_grp *grp);
	int (*dump_regs)(struct cg_grp *grp, unsigned int *ptr);
};

struct cg_grp {
	const char *name;
	unsigned int set_addr;
	unsigned int clr_addr;
	unsigned int sta_addr;
	unsigned int mask;
	unsigned int state;
	struct cg_grp_ops *ops;
	struct subsys *sys;
};


struct cg_clk;
struct cg_clk_ops {
	int (*get_state)(struct cg_clk *clk);
	int (*check_validity)(struct cg_clk *clk); /* 1: valid, 0: invalid */
	int (*enable)(struct cg_clk *clk);
	int (*disable)(struct cg_clk *clk);
};

struct cg_clk {
	int cnt;
	unsigned int state;
	unsigned int mask;
	int force_on;
	struct cg_clk_ops *ops;
	struct cg_grp *grp;
	struct clkmux *mux;
	struct cg_clk *parent;
#if CLKMGR_8127
	const char *name;
#endif /* CLKMGR_8127 */
#ifdef CONFIG_CLKMGR_STAT
	struct list_head head;
#endif
};


#ifdef CONFIG_CLKMGR_STAT
struct stat_node {
	struct list_head link;
	unsigned int cnt_on;
	unsigned int cnt_off;
	char name[0];
};
#endif



/************************************************
 **********      global variablies     **********
 ************************************************/

#define PWR_DOWN    0
#define PWR_ON      1

static int initialized;

static struct pll plls[NR_PLLS];
static struct subsys syss[NR_SYSS];
static struct clkmux muxs[NR_MUXS];
static struct cg_grp grps[NR_GRPS];
static struct cg_clk clks[NR_CLKS];



/************************************************
 **********      spin lock protect     **********
 ************************************************/

static DEFINE_SPINLOCK(clock_lock);

#define clkmgr_lock(flags) \
	spin_lock_irqsave(&clock_lock, flags)

#define clkmgr_unlock(flags) \
	spin_unlock_irqrestore(&clock_lock, flags)

#define clkmgr_locked() \
	spin_is_locked(&clock_lock)


#if !CLKMGR_8127

int clkmgr_is_locked(void)
{
	return clkmgr_locked();
}
EXPORT_SYMBOL(clkmgr_is_locked);

#endif /* !CLKMGR_8127 */



/************************************************
 **********     clkmgr stat debug      **********
 ************************************************/

#ifdef CONFIG_CLKMGR_STAT
void update_stat_locked(struct list_head *head, char *name, int op)
{
	struct list_head *pos = NULL;
	struct stat_node *node = NULL;
	int len = strlen(name);
	int new_node = 1;

	list_for_each(pos, head) {
		node = list_entry(pos, struct stat_node, link);
		if (!strncmp(node->name, name, len)) {
			new_node = 0;
			break;
		}
	}

	if (new_node) {
		node = NULL;
		node = kzalloc(sizeof(*node) + len + 1, GFP_ATOMIC);
		if (!node) {
			clk_err("[%s]: malloc stat node for %s fail\n", __func__, name);
			return;
		} else {
			memcpy(node->name, name, len);
			list_add_tail(&node->link, head);
		}
	}

	if (op)
		node->cnt_on++;
	else
		node->cnt_off++;
}
#endif



/************************************************
 **********    function declaration    **********
 ************************************************/

static int pll_enable_locked(struct pll *pll);
static int pll_disable_locked(struct pll *pll);

static int sys_enable_locked(struct subsys *sys);
static int sys_disable_locked(struct subsys *sys, int force_off);

static void mux_enable_locked(struct clkmux *mux);
static void mux_disable_locked(struct clkmux *mux);

static int clk_enable_locked(struct cg_clk *clk);
static int clk_disable_locked(struct cg_clk *clk);


static inline int pll_enable_internal(struct pll *pll, char *name)
{
	int err;
	err = pll_enable_locked(pll);
#ifdef CONFIG_CLKMGR_STAT
	update_stat_locked(&pll->head, name, 1);
#endif
	return err;
}

static inline int pll_disable_internal(struct pll *pll, char *name)
{
	int err;
	err = pll_disable_locked(pll);
#ifdef CONFIG_CLKMGR_STAT
	update_stat_locked(&pll->head, name, 0);
#endif
	return err;
}


static inline int subsys_enable_internal(struct subsys *sys, char *name)
{
	int err;
	err = sys_enable_locked(sys);
#ifdef CONFIG_CLKMGR_STAT
	update_stat_locked(&sys->head, name, 1);
#endif
	return err;
}

static inline int subsys_disable_internal(struct subsys *sys, int force_off, char *name)
{
	int err;
	err = sys_disable_locked(sys, force_off);
#ifdef CONFIG_CLKMGR_STAT
	update_stat_locked(&sys->head, name, 0);
#endif
	return err;
}


static inline void mux_enable_internal(struct clkmux *mux, char *name)
{
	mux_enable_locked(mux);
#ifdef CONFIG_CLKMGR_STAT
	update_stat_locked(&mux->head, name, 1);
#endif
}

static inline void mux_disable_internal(struct clkmux *mux, char *name)
{
	mux_disable_locked(mux);
#ifdef CONFIG_CLKMGR_STAT
	update_stat_locked(&mux->head, name, 0);
#endif
}


static inline int clk_enable_internal(struct cg_clk *clk, char *name)
{
	int err;
	err = clk_enable_locked(clk);
#ifdef CONFIG_CLKMGR_STAT
	update_stat_locked(&clk->head, name, 1);
#endif
	return err;
}

static inline int clk_disable_internal(struct cg_clk *clk, char *name)
{
	int err;
	err = clk_disable_locked(clk);
#ifdef CONFIG_CLKMGR_STAT
	update_stat_locked(&clk->head, name, 0);
#endif
	return err;
}



/************************************************
 **********          pll part          **********
 ************************************************/

#define PLL_TYPE_SDM    0
#define PLL_TYPE_LC     1

#define HAVE_RST_BAR    (0x1 << 0)
#define HAVE_PLL_HP     (0x1 << 1)
#define HAVE_FIX_FRQ    (0x1 << 2)
#define Others          (0x1 << 3)

#define RST_BAR_MASK    0x1000000

static struct pll_ops arm_pll_ops;
static struct pll_ops sdm_pll_ops;

#if CLKMGR_8127
static struct pll_ops aud_pll_ops;
#endif /* CLKMGR_8127 */

static struct pll plls[NR_PLLS] = {
	[ARMPLL] = {
		.name = __stringify(ARMPLL),
		.type = PLL_TYPE_SDM,
		.feat = HAVE_PLL_HP,
		.en_mask = 0x00000001,
		.base_addr = ARMPLL_CON0,
		.pwr_addr = ARMPLL_PWR_CON0,
		.ops = &arm_pll_ops,
		.hp_id = MT658X_FH_ARM_PLL,
		.hp_switch = 1,
	},
	[MAINPLL] = {
		.name = __stringify(MAINPLL),
		.type = PLL_TYPE_SDM,
		.feat = HAVE_PLL_HP | HAVE_RST_BAR,
		.en_mask = 0x78000001,
		.base_addr = MAINPLL_CON0,
		.pwr_addr = MAINPLL_PWR_CON0,
		.ops = &sdm_pll_ops,
		.hp_id = MT658X_FH_MAIN_PLL,
		.hp_switch = 1,
	},
	[MSDCPLL] = {
		.name = __stringify(MSDCPLL),
		.type = PLL_TYPE_SDM,
		.feat = HAVE_PLL_HP,
		.en_mask = 0x00000001,
		.base_addr = MSDCPLL_CON0,
		.pwr_addr = MSDCPLL_PWR_CON0,
		.ops = &sdm_pll_ops,
		.hp_id = MT658X_FH_MSDC_PLL,
		.hp_switch = 1,
	},
	[UNIVPLL] = {
		.name = __stringify(UNIVPLL),
		.type = PLL_TYPE_SDM,
		.feat = HAVE_RST_BAR | HAVE_FIX_FRQ,
		.en_mask = 0xFC000001,
		.base_addr = UNIVPLL_CON0,
		.pwr_addr = UNIVPLL_PWR_CON0,
		.ops = &sdm_pll_ops,
	},
	[MMPLL] = {
		.name = __stringify(MMPLL),
		.type = PLL_TYPE_SDM,
		.feat = HAVE_PLL_HP,
		.en_mask = 0x00000001,
		.base_addr = MMPLL_CON0,
		.pwr_addr = MMPLL_PWR_CON0,
		.ops = &sdm_pll_ops,
		.hp_id = MT658X_FH_MM_PLL,
		.hp_switch = 1,
	},
	[VENCPLL] = {
		.name = __stringify(VENCPLL),
		.type = PLL_TYPE_SDM,
		.feat = HAVE_PLL_HP,
		.en_mask = 0x00000001,
		.base_addr = VENCPLL_CON0,
		.pwr_addr = VENCPLL_PWR_CON0,
		.ops = &sdm_pll_ops,
		.hp_id = MT658X_FH_VENC_PLL,
		.hp_switch = 1,
	},
	[TVDPLL] = {
		.name = __stringify(TVDPLL),
		.type = PLL_TYPE_SDM,
		.feat = HAVE_PLL_HP,
		.en_mask = 0x00000001,
		.base_addr = TVDPLL_CON0,
		.pwr_addr = TVDPLL_PWR_CON0,
		.ops = &sdm_pll_ops,
		.hp_id = MT658X_FH_TVD_PLL,
		.hp_switch = 1,
	},
	[LVDSPLL] = {
		.name = __stringify(LVDSPLL),
		.type = PLL_TYPE_SDM,
		.feat = HAVE_PLL_HP,
		.en_mask = 0x00000001,
		.base_addr = LVDSPLL_CON0,
		.pwr_addr = LVDSPLL_PWR_CON0,
		.ops = &sdm_pll_ops,
		.hp_id = MT658X_FH_LVDS_PLL,
		.hp_switch = 1,
	},
	[AUDPLL] = {
		.name = __stringify(AUDPLL),
		.type = PLL_TYPE_SDM,
		.feat = 0x0,
		.en_mask = 0x00000001,
		.base_addr = AUDPLL_CON0,
		.pwr_addr = AUDPLL_PWR_CON0,
		.ops = &aud_pll_ops,
	}
};

static struct pll *id_to_pll(unsigned int id)
{
	return id < NR_PLLS ? plls + id : NULL;
}

#define PLL_PWR_ON  (0x1 << 0)
#define PLL_ISO_EN  (0x1 << 1)

#define SDM_PLL_N_INFO_MASK 0x001FFFFF
#define UNIV_SDM_PLL_N_INFO_MASK 0x001fc000
#define AUD_SDM_PLL_N_INFO_MASK 0x7FFFFFFF
#define SDM_PLL_N_INFO_CHG  0x80000000
#define ARMPLL_POSDIV_MASK  0x07000000

#define PLL_FBKDIV_MASK     0x00007F00
#define PLL_FBKDIV_OFFSET   0x8


static int pll_get_state_op(struct pll *pll)
{
	return clk_readl(pll->base_addr) & 0x1;
}

static void sdm_pll_enable_op(struct pll *pll)
{
#ifdef PLL_LOG
	clk_dbg("[%s]: pll->name=%s\n", __func__, pll->name);
#endif

	clk_setl(pll->pwr_addr, PLL_PWR_ON);
	udelay(2);
	clk_clrl(pll->pwr_addr, PLL_ISO_EN);

	clk_setl(pll->base_addr, pll->en_mask);
	udelay(20);

	if (pll->feat & HAVE_RST_BAR)
		clk_setl(pll->base_addr, RST_BAR_MASK);
}

static void sdm_pll_disable_op(struct pll *pll)
{
#ifdef PLL_LOG
	clk_dbg("[%s]: pll->name=%s\n", __func__, pll->name);
#endif

	if (pll->feat & HAVE_RST_BAR)
		clk_clrl(pll->base_addr, RST_BAR_MASK);

	clk_clrl(pll->base_addr, 0x1);

	clk_setl(pll->pwr_addr, PLL_ISO_EN);
	clk_clrl(pll->pwr_addr, PLL_PWR_ON);
}

static void sdm_pll_fsel_op(struct pll *pll, unsigned int value)
{
	unsigned int ctrl_value;

	ctrl_value = clk_readl(pll->base_addr + 4);
	if (pll->base_addr == UNIVPLL_CON0) {
		ctrl_value &= ~UNIV_SDM_PLL_N_INFO_MASK;
		ctrl_value |= value & UNIV_SDM_PLL_N_INFO_MASK;
	} else if (pll->base_addr == AUDPLL_CON0) {
		ctrl_value &= ~AUD_SDM_PLL_N_INFO_MASK;
		ctrl_value |= value & AUD_SDM_PLL_N_INFO_MASK;
	} else {
		ctrl_value &= ~SDM_PLL_N_INFO_MASK;
		ctrl_value |= value & SDM_PLL_N_INFO_MASK;
	}

	ctrl_value |= SDM_PLL_N_INFO_CHG;

	clk_writel(pll->base_addr + 4, ctrl_value);
	udelay(20);
}

static int sdm_pll_dump_regs_op(struct pll *pll, unsigned int *ptr)
{
	*(ptr) = clk_readl(pll->base_addr);
	*(++ptr) = clk_readl(pll->base_addr + 4);
	*(++ptr) = clk_readl(pll->pwr_addr);

	return 3;
}


static const unsigned int pll_posdiv_map[8] = {1, 2, 4, 8, 16, 16, 16, 16};


static unsigned int sdm_pll_get_postdiv_op(struct pll *pll)
{
	unsigned int con0 = clk_readl(pll->base_addr);          /* PLL_CON0 */
	unsigned int posdiv = (con0 & BITMASK(6 : 4)) >> 4;     /* bit[6:4] */

	return posdiv;
}


static void sdm_pll_set_postdiv_op(struct pll *pll, unsigned int postdiv)
{
	unsigned con0_addr = pll->base_addr;
	unsigned int con0_value = clk_readl(con0_addr);                                     /* PLL_CON0 */

	clk_writel(con0_addr, (con0_value & ~BITMASK(6 : 4)) | BITS(6 : 4, postdiv));       /* bit[8:6] */
}


static unsigned int sdm_pll_get_pcw_op(struct pll *pll)
{
	unsigned int con1_addr = pll->base_addr + 4;
	unsigned int reg_value;

	reg_value = clk_readl(con1_addr);               /* PLL_CON1 */
	reg_value &= SDM_PLL_N_INFO_MASK;               /* bit[20:0] */

	return reg_value;
}


static void sdm_pll_set_pcw_op(struct pll *pll, unsigned int pcw)
{
	unsigned int con1_addr = pll->base_addr + 4;
	unsigned int reg_value;
	unsigned int pll_en;

	pll_en = clk_readl(pll->base_addr) & BIT(0);    /* PLL_CON0[0] */

	reg_value = clk_readl(con1_addr);
	reg_value &= ~SDM_PLL_N_INFO_MASK;
	reg_value |= pcw & SDM_PLL_N_INFO_MASK;         /* PLL_CON1[20:0] = pcw */

	if (pll_en)
		reg_value |= SDM_PLL_N_INFO_CHG;            /* PLL_CON1[31] = 1 */

	clk_writel(con1_addr, reg_value);

	if (pll_en)
		udelay(20);
}


static unsigned int calc_pll_vco_freq(
	unsigned int fin,
	unsigned int pcw,
	unsigned int vcodivsel,
	unsigned int prediv,
	unsigned int pcw_f_bits)
{
	/* vco = (fin * pcw * vcodivsel / prediv) >> pcw_f_bits; */
	uint64_t vco = fin;
	vco = vco * pcw * vcodivsel;
	if (prediv > 1)
		do_div(vco, prediv);
	vco >>= pcw_f_bits;
	return (unsigned int)vco;
}


static unsigned int sdm_pll_vco_calc_op(struct pll *pll)
{
	unsigned int con1 = clk_readl(pll->base_addr + 4);

	unsigned int vcodivsel = 1;
	unsigned int prediv = 1;
	unsigned int pcw = con1 & BITMASK(20 : 0);                      /* bit[20:0] */

	return calc_pll_vco_freq(26000, pcw, vcodivsel, prediv, 14);    /* 26M = 26000K Hz */
}


static int general_pll_calc_freq(unsigned int *pcw, int *postdiv_idx, unsigned int freq, int pcwfbits)
{
	const uint64_t freq_max = 2000 * 1000 * 1000;   /* 2000 MHz */
	const uint64_t freq_min = 1000 * 1000 * 1000;   /* 1000 MHz */
	const uint32_t fin = 26 * 1000 * 1000;          /* 26 MHz */
	static const uint64_t postdiv[] = {1, 2, 4, 8, 16};
	uint64_t n_info;
	int idx;

	/* search suitable postdiv */
	for (idx = 0; idx < ARRAY_SIZE(postdiv) && postdiv[idx] * freq <= freq_min; idx++)
		;

	if (idx >= ARRAY_SIZE(postdiv))
		return -2;                              /* freq is out of range (too low) */
	else if (postdiv[idx] * freq > freq_max)
		return -3;                              /* freq is out of range (too high) */

	n_info = (postdiv[idx] * freq) << pcwfbits; /* n_info = freq * postdiv / 26MHz * 2^pcwfbits */
	do_div(n_info, fin);

	*postdiv_idx = idx;
	*pcw = (unsigned int)n_info;

	return 0;
}


static int general_pll_set_freq_op(struct pll *pll, unsigned int freq, int pcwfbits)
{
	int r;
	unsigned int pcw = 0;
	int postdiv_idx = 0;

	r = general_pll_calc_freq(&pcw, &postdiv_idx, freq, pcwfbits);

	if (r == 0) {
		pll->ops->set_postdiv(pll, postdiv_idx);
		pll->ops->set_pcw(pll, pcw);
	}

	return r;
}


static int sdm_pll_set_freq_op(struct pll *pll, unsigned int freq)
{
	return general_pll_set_freq_op(pll, freq, 14);
}


static int sdm_pll_hp_enable_op(struct pll *pll)
{
	int err;
	unsigned int vco = 0;

	if (!pll->hp_switch || (pll->state == PWR_DOWN))
		return 0;

	err = freqhopping_config(pll->hp_id, vco, 1);

	return err;
}

static int sdm_pll_hp_disable_op(struct pll *pll)
{
	int err;
	unsigned int vco = 0;

	if (!pll->hp_switch || (pll->state == PWR_ON))
		return 0;

	err = freqhopping_config(pll->hp_id, vco, 0);

	return err;
}

static struct pll_ops sdm_pll_ops = {
	.get_state = pll_get_state_op,
	.enable = sdm_pll_enable_op,
	.disable = sdm_pll_disable_op,
	.fsel = sdm_pll_fsel_op,
	.dump_regs = sdm_pll_dump_regs_op,
	.vco_calc = sdm_pll_vco_calc_op,
	.hp_enable = sdm_pll_hp_enable_op,
	.hp_disable = sdm_pll_hp_disable_op,
#if CLKMGR_8127
	.get_postdiv    = sdm_pll_get_postdiv_op,
	.set_postdiv    = sdm_pll_set_postdiv_op,
	.get_pcw        = sdm_pll_get_pcw_op,
	.set_pcw        = sdm_pll_set_pcw_op,
	.set_freq       = sdm_pll_set_freq_op,
#endif /* CLKMGR_8127 */
};


#if CLKMGR_8127


static unsigned int aud_pll_get_pcw_op(struct pll *pll)
{
	unsigned int con1_addr = pll->base_addr + 4;
	unsigned int reg_value = clk_readl(con1_addr);  /* PLL_CON1; */

	reg_value &= BITMASK(30 : 0);                   /* bit[30:0] */
	return reg_value;
}


static void aud_pll_set_pcw_op(struct pll *pll, unsigned int pcw)
{
	unsigned int con1_addr = pll->base_addr + 4;
	unsigned int con3_addr = pll->base_addr + 0x01B8;
	unsigned int reg_value;
	unsigned int pll_en;

	pll_en = clk_readl(pll->base_addr) & BIT(0);
	reg_value = clk_readl(con1_addr);                   /* AUDPLL_CON1[30:0] (PLL_SDM_PCW) = pcw */
	reg_value = (reg_value & ~BITMASK(30 : 0)) | BITS(30 : 0, pcw);

	if (pll_en)                                         /* AUDPLL_CON1[31] (AUDPLL_SDM_PCW_CHG) = 1 "and"  */
		reg_value |= SDM_PLL_N_INFO_CHG;                /* AUDPLL_CON3[31] (AUDPLL_TUNER_EN) = 1 */

	clk_writel(con3_addr, reg_value + 1);               /* AUDPLL_CON3[30:0] (AUDPLL_TUNER_N_INFO) = (pcw + 1) */
	clk_writel(con1_addr, reg_value);                   /* AUDPLL_CON1 */

	if (pll_en)
		udelay(20);
}


static unsigned int aud_pll_vco_calc_op(struct pll *pll)
{
	unsigned int con1 = clk_readl(pll->base_addr + 4);

	unsigned int vcodivsel = 1;
	unsigned int prediv = 1;
	unsigned int pcw = con1 & BITMASK(30 : 0);                      /* bit[30:0] */

	return calc_pll_vco_freq(26000, pcw, vcodivsel, prediv, 24);    /* 26M = 26000K Hz */
}


static int aud_pll_set_freq_op(struct pll *pll, unsigned int freq)
{
	return general_pll_set_freq_op(pll, freq, 24);
}


static struct pll_ops aud_pll_ops = {
	.get_state      = pll_get_state_op,
	.enable         = sdm_pll_enable_op,
	.disable        = sdm_pll_disable_op,
	.fsel           = sdm_pll_fsel_op,
	.dump_regs      = sdm_pll_dump_regs_op,
	.vco_calc       = aud_pll_vco_calc_op,
	.hp_enable      = sdm_pll_hp_enable_op,
	.hp_disable     = sdm_pll_hp_disable_op,
	.get_postdiv    = sdm_pll_get_postdiv_op,
	.set_postdiv    = sdm_pll_set_postdiv_op,
	.get_pcw        = aud_pll_get_pcw_op,
	.set_pcw        = aud_pll_set_pcw_op,
	.set_freq       = aud_pll_set_freq_op,
};


#endif /* CLKMGR_8127 */


static void arm_pll_fsel_op(struct pll *pll, unsigned int value)
{
	unsigned int ctrl_value;

	ctrl_value = clk_readl(pll->base_addr + 4);
	ctrl_value &= ~(SDM_PLL_N_INFO_MASK | ARMPLL_POSDIV_MASK);
	ctrl_value |= value & (SDM_PLL_N_INFO_MASK | ARMPLL_POSDIV_MASK);
	ctrl_value |= SDM_PLL_N_INFO_CHG;

	clk_writel(pll->base_addr + 4, ctrl_value);
	udelay(20);
}


#if CLKMGR_8127


static unsigned int arm_pll_get_postdiv_op(struct pll *pll)
{
	unsigned int con1 = clk_readl(pll->base_addr + 4);      /* PLL_CON1 */
	unsigned int posdiv = (con1 & BITMASK(26 : 24)) >> 24;  /* bit[26:24] */

	return posdiv;
}


static void arm_pll_set_postdiv_op(struct pll *pll, unsigned int postdiv)
{
	unsigned con1_addr = pll->base_addr + 4;
	unsigned int con1_value = clk_readl(con1_addr);                                     /* PLL_CON1 */

	clk_writel(con1_addr, (con1_value & ~BITMASK(26 : 24)) | BITS(26 : 24, postdiv));   /* bit[26:24] */
}


static int arm_pll_set_freq_op(struct pll *pll, unsigned int freq)
{
	const int pcwfbits = 14;
	int r;
	unsigned int pcw = 0;
	int postdiv_idx = 0;

	r = general_pll_calc_freq(&pcw, &postdiv_idx, freq, pcwfbits);

	if (r == 0) {
		unsigned int reg_val = BITS(26 : 24, postdiv_idx) | BITS(20 : 0, pcw);
		arm_pll_fsel_op(pll, reg_val);
	}

	return r;
}


#endif /* CLKMGR_8127 */


static struct pll_ops arm_pll_ops = {
	.get_state = pll_get_state_op,
	.enable = sdm_pll_enable_op,
	.disable = sdm_pll_disable_op,
	.fsel = arm_pll_fsel_op,
	.dump_regs = sdm_pll_dump_regs_op,
	.vco_calc = sdm_pll_vco_calc_op,
	.hp_enable = sdm_pll_hp_enable_op,
	.hp_disable = sdm_pll_hp_disable_op,
#if CLKMGR_8127
	.get_postdiv    = arm_pll_get_postdiv_op,
	.set_postdiv    = arm_pll_set_postdiv_op,
	.get_pcw        = sdm_pll_get_pcw_op,
	.set_pcw        = sdm_pll_set_pcw_op,
	.set_freq       = arm_pll_set_freq_op,
#endif /* CLKMGR_8127 */
};


#if CLKMGR_8127


static unsigned int pll_freq_calc_op(struct pll *pll)
{
	return pll->ops->vco_calc(pll) / pll_posdiv_map[pll->ops->get_postdiv(pll)];
}


unsigned int pll_get_freq(int id)
{
	unsigned int r;
	unsigned long flags;
	struct pll *pll = id_to_pll(id);
	BUG_ON(!pll);

	clkmgr_lock(flags);
	r = pll_freq_calc_op(pll);
	clkmgr_unlock(flags);

	return r;
}
EXPORT_SYMBOL(pll_get_freq);


unsigned int pll_set_freq(int id, unsigned int freq_khz)
{
	unsigned int r;
	unsigned long flags;
	struct pll *pll = id_to_pll(id);
	BUG_ON(!pll);

	clkmgr_lock(flags);
	r = pll->ops->set_freq(pll, freq_khz * 1000);
	clkmgr_unlock(flags);

	return r;
}
EXPORT_SYMBOL(pll_set_freq);


#endif /* CLKMGR_8127 */


static int get_pll_state_locked(struct pll *pll)
{
	if (likely(initialized))
		return pll->state;
	else
		return pll->ops->get_state(pll);
}

static int pll_enable_locked(struct pll *pll)
{
#ifdef PLL_LOG_TOP
	clk_info("[%s]: Start. pll->name=%s, pll->cnt=%d, pll->state=%d\n", __func__, pll->name, pll->cnt, pll->state);
#endif

	pll->cnt++;

	if (pll->cnt > 1)
		return 0;

	if (pll->state == PWR_DOWN) {
		pll->ops->enable(pll);
		pll->state = PWR_ON;
	}

	if (pll->ops->hp_enable)
		pll->ops->hp_enable(pll);

#ifdef PLL_LOG_TOP
	clk_info("[%s]: End. pll->name=%s, pll->cnt=%d, pll->state=%d\n", __func__, pll->name, pll->cnt, pll->state);
#endif
	return 0;
}

static int pll_disable_locked(struct pll *pll)
{
#ifdef PLL_LOG_TOP
	clk_info("[%s]: Start. pll->name=%s, pll->cnt=%d, pll->state=%d\n", __func__, pll->name, pll->cnt, pll->state);
#endif

#if DISABLE_BUG_ON_CNT
	WARN_ON(!pll->cnt);
	if (!pll->cnt)
		return 0;
#else /* !DISABLE_BUG_ON_CNT */
	BUG_ON(!pll->cnt);
#endif /* DISABLE_BUG_ON_CNT */

	pll->cnt--;

	if (pll->cnt > 0)
		return 0;

	if (pll->state == PWR_ON) {
		pll->ops->disable(pll);
		pll->state = PWR_DOWN;
	}

	if (pll->ops->hp_disable)
		pll->ops->hp_disable(pll);

#ifdef PLL_LOG_TOP
	clk_info("[%s]: End. pll->name=%s, pll->cnt=%d, pll->state=%d\n", __func__, pll->name, pll->cnt, pll->state);
#endif
	return 0;
}


static int pll_fsel_locked(struct pll *pll, unsigned int value)
{
	pll->ops->fsel(pll, value);
	if (pll->ops->hp_enable)
		pll->ops->hp_enable(pll);

	return 0;
}

int pll_is_on(int id)
{
	int state;
	unsigned long flags;
	struct pll *pll = id_to_pll(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 1;
#endif

	BUG_ON(!pll);

	clkmgr_lock(flags);
	state = get_pll_state_locked(pll);
	clkmgr_unlock(flags);

	return state;
}
EXPORT_SYMBOL(pll_is_on);

int enable_pll(int id, char *name)
{
	int err;
	unsigned long flags;
	struct pll *pll = id_to_pll(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!pll);
	BUG_ON(!name);
#ifdef PLL_LOG_TOP
	clk_info("[%s]: id=%d, name=%s\n", __func__, id, name);
#endif
	clkmgr_lock(flags);
	err = pll_enable_internal(pll, name);
	clkmgr_unlock(flags);

	return err;
}
EXPORT_SYMBOL(enable_pll);

int disable_pll(int id, char *name)
{
	int err;
	unsigned long flags;
	struct pll *pll = id_to_pll(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!pll);
	BUG_ON(!name);
#ifdef PLL_LOG_TOP
	clk_info("[%s]: id=%d, name=%s\n", __func__, id, name);
#endif
	clkmgr_lock(flags);
	err = pll_disable_internal(pll, name);
	clkmgr_unlock(flags);

	return err;
}
EXPORT_SYMBOL(disable_pll);


#if !CLKMGR_8127


int enable_pll_spec(int id, char *name)
{
	int err;
	unsigned long flags;
	struct pll *pll = id_to_pll(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!pll);
	BUG_ON(!name);
#ifdef PLL_LOG_TOP
	clk_info("[%s]: id=%d, name=%s\n", __func__, id, name);
#endif
	clkmgr_lock(flags);
	err = pll_enable_internal(pll, name);
	clkmgr_unlock(flags);

	return err;
}
EXPORT_SYMBOL(enable_pll_spec);

int disable_pll_spec(int id, char *name)
{
	int err;
	unsigned long flags;
	struct pll *pll = id_to_pll(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!pll);
	BUG_ON(!name);
#ifdef PLL_LOG_TOP
	clk_info("[%s]: id=%d, name=%s\n", __func__, id, name);
#endif
	clkmgr_lock(flags);
	err = pll_disable_internal(pll, name);
	clkmgr_unlock(flags);

	return err;
}
EXPORT_SYMBOL(disable_pll_spec);


#endif /* !CLKMGR_8127 */


int pll_fsel(int id, unsigned int value)
{
	int err;
	unsigned long flags;
	struct pll *pll = id_to_pll(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!pll);

	clkmgr_lock(flags);
	err = pll_fsel_locked(pll, value);
	clkmgr_unlock(flags);

	return err;
}
EXPORT_SYMBOL(pll_fsel);


int pll_hp_switch_on(int id, int hp_on)
{
	int err = 0;
	unsigned long flags;
	int old_value;
	struct pll *pll = id_to_pll(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!pll);

	if (pll->type != PLL_TYPE_SDM) {
		err = -EINVAL;
		goto out;
	}

	clkmgr_lock(flags);
	old_value = pll->hp_switch;
	if (old_value == 0) {
		pll->hp_switch = 1;
		if (hp_on)
			err = pll->ops->hp_enable(pll);
	}
	clkmgr_unlock(flags);

out:
	return err;
}
EXPORT_SYMBOL(pll_hp_switch_on);

int pll_hp_switch_off(int id, int hp_off)
{
	int err = 0;
	unsigned long flags;
	int old_value;
	struct pll *pll = id_to_pll(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!pll);

	if (pll->type != PLL_TYPE_SDM) {
		err = -EINVAL;
		goto out;
	}

	clkmgr_lock(flags);
	old_value = pll->hp_switch;
	if (old_value == 1) {
		if (hp_off)
			err = pll->ops->hp_disable(pll);
		pll->hp_switch = 0;
	}
	clkmgr_unlock(flags);

out:
	return err;
}
EXPORT_SYMBOL(pll_hp_switch_off);


int pll_dump_regs(int id, unsigned int *ptr)
{
	struct pll *pll = id_to_pll(id);

	BUG_ON(!initialized);
	BUG_ON(!pll);

	return pll->ops->dump_regs(pll, ptr);
}
EXPORT_SYMBOL(pll_dump_regs);

const char *pll_get_name(int id)
{
	struct pll *pll = id_to_pll(id);

	BUG_ON(!initialized);
	BUG_ON(!pll);

	return pll->name;
}


#if !CLKMGR_8127


void enable_clksq1(void)
{
	unsigned long flags;

	return;

	clkmgr_lock(flags);
	clk_setl(AP_PLL_CON0, 0x1 << 0);
	udelay(200);
	clk_setl(AP_PLL_CON0, 0x1 << 1);
	clkmgr_unlock(flags);
}
EXPORT_SYMBOL(enable_clksq1);

void disable_clksq1(void)
{
	unsigned long flags;

	return;

	clkmgr_lock(flags);
	clk_clrl(AP_PLL_CON0, 0x3 << 0);
	clkmgr_unlock(flags);
}
EXPORT_SYMBOL(disable_clksq1);

void clksq1_sw2hw(void)
{
	unsigned long flags;

	return;

	clkmgr_lock(flags);
	clk_clrl(AP_PLL_CON1, 0x3 << 0);
	clkmgr_unlock(flags);
}
EXPORT_SYMBOL(clksq1_sw2hw);

void clksq1_hw2sw(void)
{
	unsigned long flags;

	return;

	clkmgr_lock(flags);
	clk_setl(AP_PLL_CON1, 0x3 << 0);
	clkmgr_unlock(flags);
}
EXPORT_SYMBOL(clksq1_hw2sw);


#endif /* !CLKMGR_8127 */



/************************************************
 **********         subsys part        **********
 ************************************************/

#define SYS_TYPE_MODEM    0
#define SYS_TYPE_MEDIA    1
#define SYS_TYPE_OTHER    2
#define SYS_TYPE_CONN     3

static struct subsys_ops conn_sys_ops;
static struct subsys_ops dpy_sys_ops;
static struct subsys_ops dis_sys_ops;
static struct subsys_ops mfg_sys_ops;
static struct subsys_ops isp_sys_ops;
static struct subsys_ops ifr_sys_ops;
static struct subsys_ops vde_sys_ops;

static struct subsys syss[NR_SYSS] = {
	[SYS_CONN] = {
		.name = __stringify(SYS_CONN),
		.type = SYS_TYPE_CONN,
		.default_sta = PWR_DOWN,
		.sta_mask = 1U << 1,
		.ctl_addr = SPM_CONN_PWR_CON,
		.ops = &conn_sys_ops,
	},
	[SYS_DPY] = {
		.name = __stringify(SYS_DPY),
		.type = SYS_TYPE_OTHER,
		.default_sta = PWR_ON,
		.sta_mask = 1U << 2,
		.ctl_addr = SPM_DPY_PWR_CON,
		.ops = &dpy_sys_ops,
	},
	[SYS_DIS] = {
		.name = __stringify(SYS_DIS),
		.type = SYS_TYPE_MEDIA,
		.default_sta = PWR_ON,
		.sta_mask = 1U << 3,
		.ctl_addr = SPM_DIS_PWR_CON,
		.ops = &dis_sys_ops,
		.start = &grps[CG_DISP0],
		.nr_grps = 2,
		.mux = &muxs[MT_MUX_MM],
	},
	[SYS_MFG] = {
		.name = __stringify(SYS_MFG),
		.type = SYS_TYPE_MEDIA,
		.default_sta = PWR_ON,
		.sta_mask = 1U << 4,
		.ctl_addr = SPM_MFG_PWR_CON,
		.ops = &mfg_sys_ops,
		.start = &grps[CG_MFG],
		.nr_grps = 1,
		.mux = &muxs[MT_MUX_MFG],
	},
	[SYS_ISP] = {
		.name = __stringify(SYS_ISP),
		.type = SYS_TYPE_MEDIA,
		.default_sta = PWR_ON,
		.sta_mask = 1U << 5,
		.ctl_addr = SPM_ISP_PWR_CON,
		.ops = &isp_sys_ops,
		.start = &grps[CG_IMAGE],
		.nr_grps = 1,
	},
	[SYS_IFR] = {
		.name = __stringify(SYS_IFR),
		.type = SYS_TYPE_OTHER,
		.default_sta = PWR_ON,
		.sta_mask = 1U << 6,
		.ctl_addr = SPM_IFR_PWR_CON,
		.ops = &ifr_sys_ops,
	},
	[SYS_VDE] = {
		.name = __stringify(SYS_VDE),
		.type = SYS_TYPE_MEDIA,
		.default_sta = PWR_ON,
		.sta_mask = 1U << 7,
		.ctl_addr = SPM_VDE_PWR_CON,
		.ops = &vde_sys_ops,
		.start = &grps[CG_VDEC0],
		.nr_grps = 2,
		.mux = &muxs[MT_MUX_VDEC],
	}
};


static void larb_backup(int larb_idx);
static void larb_restore(int larb_idx);


static struct subsys *id_to_sys(unsigned int id)
{
	return id < NR_SYSS ? syss + id : NULL;
}

static int conn_sys_enable_op(struct subsys *sys)
{
	int err;
	err = spm_mtcmos_ctrl_connsys(STA_POWER_ON);
	return err;
}

static int conn_sys_disable_op(struct subsys *sys)
{
	int err;
	err = spm_mtcmos_ctrl_connsys(STA_POWER_DOWN);
	return err;
}

static int dpy_sys_enable_op(struct subsys *sys)
{
	int err;
	err = spm_mtcmos_ctrl_ddrphy(STA_POWER_ON);
	return err;
}

static int dpy_sys_disable_op(struct subsys *sys)
{
	int err;
	err = spm_mtcmos_ctrl_ddrphy(STA_POWER_DOWN);
	return err;
}

static int dis_sys_enable_op(struct subsys *sys)
{
	int err;
#ifdef SYS_LOG
	clk_info("[%s]: sys->name=%s\n", __func__, sys->name);
#endif

	err = spm_mtcmos_ctrl_disp(STA_POWER_ON);
	larb_restore(MT_LARB_DISP);
	return err;
}

static int dis_sys_disable_op(struct subsys *sys)
{
	int err;
#ifdef SYS_LOG
	clk_info("[%s]: sys->name=%s\n", __func__, sys->name);
#endif

	larb_backup(MT_LARB_DISP);
	err = spm_mtcmos_ctrl_disp(STA_POWER_DOWN);
	return err;
}

static int mfg_sys_enable_op(struct subsys *sys)
{
	int err;
#ifdef SYS_LOG
	clk_info("[%s]: sys->name=%s\n", __func__, sys->name);
#endif

	err = spm_mtcmos_ctrl_mfg(STA_POWER_ON);

	return err;
}

static int mfg_sys_disable_op(struct subsys *sys)
{
	int err;
#ifdef SYS_LOG
	clk_info("[%s]: sys->name=%s\n", __func__, sys->name);
#endif

	err = spm_mtcmos_ctrl_mfg(STA_POWER_DOWN);

	return err;
}

static int isp_sys_enable_op(struct subsys *sys)
{
	int err;
#ifdef SYS_LOG
	clk_info("[%s]: sys->name=%s\n", __func__, sys->name);
#endif
	err = spm_mtcmos_ctrl_isp(STA_POWER_ON);
	larb_restore(MT_LARB_IMG);
	return err;
}

static int isp_sys_disable_op(struct subsys *sys)
{
	int err;
#ifdef SYS_LOG
	clk_info("[%s]: sys->name=%s\n", __func__, sys->name);
#endif
	larb_backup(MT_LARB_IMG);
	err = spm_mtcmos_ctrl_isp(STA_POWER_DOWN);
	return err;
}

static int ifr_sys_enable_op(struct subsys *sys)
{
	int err;
#ifdef SYS_LOG
	clk_info("[%s]: sys->name=%s\n", __func__, sys->name);
#endif
	err = spm_mtcmos_ctrl_infra(STA_POWER_ON);
	return err;
}

static int ifr_sys_disable_op(struct subsys *sys)
{
	int err;
#ifdef SYS_LOG
	clk_info("[%s]: sys->name=%s\n", __func__, sys->name);
#endif
	err = spm_mtcmos_ctrl_infra(STA_POWER_DOWN);
	return err;
}

static int vde_sys_enable_op(struct subsys *sys)
{
	int err;
#ifdef SYS_LOG
	clk_info("[%s]: sys->name=%s\n", __func__, sys->name);
#endif
	err = spm_mtcmos_ctrl_vdec(STA_POWER_ON);
	larb_restore(MT_LARB_VDEC);
	return err;
}

static int vde_sys_disable_op(struct subsys *sys)
{
	int err;
#ifdef SYS_LOG
	clk_info("[%s]: sys->name=%s\n", __func__, sys->name);
#endif
	larb_backup(MT_LARB_VDEC);
	err = spm_mtcmos_ctrl_vdec(STA_POWER_DOWN);
	return err;
}

static int sys_get_state_op(struct subsys *sys)
{
	unsigned int sta = clk_readl(SPM_PWR_STATUS);
	unsigned int sta_s = clk_readl(SPM_PWR_STATUS_S);

	return (sta & sys->sta_mask) && (sta_s & sys->sta_mask);
}

static int sys_dump_regs_op(struct subsys *sys, unsigned int *ptr)
{
	*(ptr) = clk_readl(sys->ctl_addr);
	return 1;
}

static struct subsys_ops conn_sys_ops = {
	.enable = conn_sys_enable_op,
	.disable = conn_sys_disable_op,
	.get_state = sys_get_state_op,
	.dump_regs = sys_dump_regs_op,
};

static struct subsys_ops dpy_sys_ops = {
	.enable = dpy_sys_enable_op,
	.disable = dpy_sys_disable_op,
	.get_state = sys_get_state_op,
	.dump_regs = sys_dump_regs_op,
};

static struct subsys_ops dis_sys_ops = {
	.enable = dis_sys_enable_op,
	.disable = dis_sys_disable_op,
	.get_state = sys_get_state_op,
	.dump_regs = sys_dump_regs_op,
};

static struct subsys_ops mfg_sys_ops = {
	.enable = mfg_sys_enable_op,
	.disable = mfg_sys_disable_op,
	.get_state = sys_get_state_op,
	.dump_regs = sys_dump_regs_op,
};

static struct subsys_ops isp_sys_ops = {
	.enable = isp_sys_enable_op,
	.disable = isp_sys_disable_op,
	.get_state = sys_get_state_op,
	.dump_regs = sys_dump_regs_op,
};

static struct subsys_ops ifr_sys_ops = {
	.enable = ifr_sys_enable_op,
	.disable = ifr_sys_disable_op,
	.get_state = sys_get_state_op,
	.dump_regs = sys_dump_regs_op,
};

static struct subsys_ops vde_sys_ops = {
	.enable = vde_sys_enable_op,
	.disable = vde_sys_disable_op,
	.get_state = sys_get_state_op,
	.dump_regs = sys_dump_regs_op,
};




static int get_sys_state_locked(struct subsys *sys)
{
	if (likely(initialized))
		return sys->state;
	else
		return sys->ops->get_state(sys);
}

int subsys_is_on(int id)
{
	int state;
	unsigned long flags;
	struct subsys *sys = id_to_sys(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 1;
#endif

	BUG_ON(!sys);

	clkmgr_lock(flags);
	state = get_sys_state_locked(sys);
	clkmgr_unlock(flags);

	return state;
}
EXPORT_SYMBOL(subsys_is_on);

/* #define STATE_CHECK_DEBUG */

static int sys_enable_locked(struct subsys *sys)
{
	int err;
	int local_state = sys->state; /* get_subsys_local_state(sys); */

#ifdef STATE_CHECK_DEBUG
	int reg_state = sys->ops->get_state(sys); /* get_subsys_reg_state(sys); */
	BUG_ON(local_state != reg_state);
#endif

#ifdef SYS_LOG
	clk_info("[%s]: Start. sys->name=%s, sys->state=%d\n", __func__, sys->name, sys->state);
#endif

	if (local_state == PWR_ON)
		return 0;

	if (sys->mux)
		mux_enable_internal(sys->mux, "sys");

	err = sys->ops->enable(sys);
	WARN_ON(err);

	if (!err)
		sys->state = PWR_ON;

#ifdef SYS_LOG
	clk_info("[%s]: End. sys->name=%s, sys->state=%d\n", __func__, sys->name, sys->state);
#endif
	return err;
}

static int sys_disable_locked(struct subsys *sys, int force_off)
{
	int err;
	int local_state = sys->state; /* get_subsys_local_state(sys); */
	int i;
	struct cg_grp *grp;

#ifdef STATE_CHECK_DEBUG
	int reg_state = sys->ops->get_state(sys); /* get_subsys_reg_state(sys); */
	BUG_ON(local_state != reg_state);
#endif

#ifdef SYS_LOG
	clk_info("[%s]: Start. sys->name=%s, sys->state=%d, force_off=%d\n",
		__func__, sys->name, sys->state, force_off);
#endif
	if (!force_off) {
		/* could be power off or not */
		for (i = 0; i < sys->nr_grps; i++) {
			grp = sys->start + i;
			if (grp->state)
				return 0;
		}
	}

	if (local_state == PWR_DOWN)
		return 0;

	err = sys->ops->disable(sys);
	WARN_ON(err);

	if (!err)
		sys->state = PWR_DOWN;

	if (sys->mux)
		mux_disable_internal(sys->mux, "sys");

#ifdef SYS_LOG
	clk_info("[%s]: End. sys->name=%s, sys->state=%d, force_off=%d\n",
		__func__, sys->name, sys->state, force_off);
#endif
	return err;
}

int enable_subsys(int id, char *name)
{
	int err;
	unsigned long flags;
	struct subsys *sys = id_to_sys(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!sys);

	clkmgr_lock(flags);
	err = subsys_enable_internal(sys, name);
	clkmgr_unlock(flags);

	return err;
}
EXPORT_SYMBOL(enable_subsys);

int disable_subsys(int id, char *name)
{
	int err;
	unsigned long flags;
	struct subsys *sys = id_to_sys(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!sys);

	clkmgr_lock(flags);
	err = subsys_disable_internal(sys, 0, name);
	clkmgr_unlock(flags);

	return err;
}
EXPORT_SYMBOL(disable_subsys);

int disable_subsys_force(int id, char *name)
{
	int err;
	unsigned long flags;
	struct subsys *sys = id_to_sys(id);

	BUG_ON(!initialized);
	BUG_ON(!sys);

	clkmgr_lock(flags);
	err = subsys_disable_internal(sys, 1, name);
	clkmgr_unlock(flags);

	return err;
}

int subsys_dump_regs(int id, unsigned int *ptr)
{
	struct subsys *sys = id_to_sys(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!sys);

	return sys->ops->dump_regs(sys, ptr);
}
EXPORT_SYMBOL(subsys_dump_regs);

const char *subsys_get_name(int id)
{
	struct subsys *sys = id_to_sys(id);

	BUG_ON(!initialized);
	BUG_ON(!sys);

	return sys->name;
}

#define JIFFIES_PER_LOOP 10


#if !CLKMGR_8127


int md_power_on(int id)
{
	return 0;
}
EXPORT_SYMBOL(md_power_on);

int md_power_off(int id, unsigned int timeout)
{
	return 0;
}
EXPORT_SYMBOL(md_power_off);


#endif /* !CLKMGR_8127 */


int conn_power_on(void)
{
	int err;
	unsigned long flags;
	struct subsys *sys = id_to_sys(SYS_CONN);

	BUG_ON(!initialized);
	BUG_ON(!sys);
	BUG_ON(sys->type != SYS_TYPE_CONN);

	clkmgr_lock(flags);
	err = subsys_enable_internal(sys, "conn");
	clkmgr_unlock(flags);

	WARN_ON(err);

	return err;
}
EXPORT_SYMBOL(conn_power_on);

int conn_power_off(void)
{
	int err;
	unsigned long flags;
	struct subsys *sys = id_to_sys(SYS_CONN);

	BUG_ON(!initialized);
	BUG_ON(!sys);
	BUG_ON(sys->type != SYS_TYPE_CONN);

	clkmgr_lock(flags);
	err = subsys_disable_internal(sys, 0, "conn");
	clkmgr_unlock(flags);

	WARN_ON(err);

	return err;
}
EXPORT_SYMBOL(conn_power_off);


static DEFINE_MUTEX(larb_monitor_lock);
static LIST_HEAD(larb_monitor_handlers);

void register_larb_monitor(struct larb_monitor *handler)
{
	struct list_head *pos;

#if defined(CONFIG_CLKMGR_BRINGUP)
	return;
#endif

	mutex_lock(&larb_monitor_lock);
	list_for_each(pos, &larb_monitor_handlers) {
		struct larb_monitor *l;
		l = list_entry(pos, struct larb_monitor, link);
		if (l->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	mutex_unlock(&larb_monitor_lock);
}
EXPORT_SYMBOL(register_larb_monitor);


void unregister_larb_monitor(struct larb_monitor *handler)
{
#if defined(CONFIG_CLKMGR_BRINGUP)
	return;
#endif

	mutex_lock(&larb_monitor_lock);
	list_del(&handler->link);
	mutex_unlock(&larb_monitor_lock);
}
EXPORT_SYMBOL(unregister_larb_monitor);

static void larb_clk_prepare(int larb_idx)
{
	switch (larb_idx) {
	case MT_LARB_DISP:
		/* display */
		clk_writel(DISP_CG_CLR0, 0x3);
		break;
	case MT_LARB_VDEC:
		/* vde */
		clk_writel(LARB_CKEN_SET, 0x1);
		break;
	case MT_LARB_IMG:
		/* isp */
		clk_writel(IMG_CG_CLR, 0x1);
		break;
	default:
		BUG();
	}
}

static void larb_clk_finish(int larb_idx)
{
	switch (larb_idx) {
	case MT_LARB_DISP:
		/* display */
		clk_writel(DISP_CG_SET0, 0x3);
		break;
	case MT_LARB_VDEC:
		/* vde */
		clk_writel(LARB_CKEN_CLR, 0x1);
		break;
	case MT_LARB_IMG:
		/* isp */
		clk_writel(IMG_CG_SET, 0x1);
		break;
	default:
		BUG();
	}
}

static void larb_backup(int larb_idx)
{
	struct larb_monitor *pos;

	clk_dbg("[%s]: start to backup larb%d\n", __func__, larb_idx);

	larb_clk_prepare(larb_idx);

	list_for_each_entry(pos, &larb_monitor_handlers, link) {
		if (pos->backup != NULL)
			pos->backup(pos, larb_idx);
	}

	larb_clk_finish(larb_idx);
}

static void larb_restore(int larb_idx)
{
	struct larb_monitor *pos;

	clk_dbg("[%s]: start to restore larb%d\n", __func__, larb_idx);

	larb_clk_prepare(larb_idx);

	list_for_each_entry(pos, &larb_monitor_handlers, link) {
		if (pos->restore != NULL)
			pos->restore(pos, larb_idx);
	}

	larb_clk_finish(larb_idx);
}



/************************************************
 **********         clkmux part        **********
 ************************************************/

static struct clkmux_ops clkmux_ops;
static struct clkmux_ops dpi0_clkmux_ops;

static struct clkmux muxs[NR_MUXS] = {
	[MT_MUX_MM] = {
		.name = __stringify(MT_MUX_MM),
		.base_addr = CLK_CFG_0,
		.sel_mask = 0x07000000,
		.pdn_mask = 0x80000000,
		.offset = 24,
		.nr_inputs = 8,
		.ops = &clkmux_ops,
		.pll = &plls[VENCPLL],
	},
	[MT_MUX_DDRPHYCFG] = {
		.name = __stringify(MT_MUX_DDRPHYCFG),
		.base_addr = CLK_CFG_0,
		.sel_mask = 0x00010000,
		.pdn_mask = 0x00800000,
		.offset = 16,
		.nr_inputs = 2,
		.ops = &clkmux_ops,
	},
	[MT_MUX_MEM] = {
		.name = __stringify(MT_MUX_MEM),
		.base_addr = CLK_CFG_0,
		.sel_mask = 0x00000100,
		.pdn_mask = 0x00008000,
		.offset = 8,
		.nr_inputs = 2,
		.ops = &clkmux_ops,
	},
	[MT_MUX_AXI] = {
		.name = __stringify(MT_MUX_AXI),
		.base_addr = CLK_CFG_0,
		.sel_mask = 0x00000007,
		.pdn_mask = 0x00000080,
		.offset = 0,
		.nr_inputs = 8,
		.ops = &clkmux_ops,
	},
	[MT_MUX_CAMTG] = {
		.name = __stringify(MT_MUX_CAMTG),
		.base_addr = CLK_CFG_1,
		.sel_mask = 0x07000000,
		.pdn_mask = 0x80000000,
		.offset = 24,
		.nr_inputs = 7,
		.ops = &clkmux_ops,
		.pll = &plls[UNIVPLL],
	},
	[MT_MUX_MFG] = {
		.name = __stringify(MT_MUX_MFG),
		.base_addr = CLK_CFG_1,
		.sel_mask = 0x00070000,
		.pdn_mask = 0x00800000,
		.offset = 16,
		.nr_inputs = 8,
		.ops = &clkmux_ops,
		.pll = &plls[MMPLL],
	},
	[MT_MUX_VDEC] = {
		.name = __stringify(MT_MUX_VDEC),
		.base_addr = CLK_CFG_1,
		.sel_mask = 0x00000F00,
		.pdn_mask = 0x00008000,
		.offset = 8,
		.nr_inputs = 9,
		.ops = &clkmux_ops,
	},
	[MT_MUX_PWM] = {
		.name = __stringify(MT_MUX_PWM),
		.base_addr = CLK_CFG_1,
		.sel_mask = 0x00000003,
		.pdn_mask = 0x00000080,
		.offset = 0,
		.nr_inputs = 4,
		.ops = &clkmux_ops,
		.pll = &plls[UNIVPLL],
	},
	[MT_MUX_MSDC30_0] = {
		.name = __stringify(MT_MUX_MSDC30_0),
		.base_addr = CLK_CFG_2,
		.sel_mask = 0x07000000,
		.pdn_mask = 0x80000000,
		.offset = 24,
		.nr_inputs = 6,
		.ops = &clkmux_ops,
		.pll = &plls[MSDCPLL],
	},
	[MT_MUX_USB20] = {
		.name = __stringify(MT_MUX_USB20),
		.base_addr = CLK_CFG_2,
		.sel_mask = 0x00030000,
		.pdn_mask = 0x00800000,
		.offset = 16,
		.nr_inputs = 3,
		.ops = &clkmux_ops,
		.pll = &plls[UNIVPLL],
	},
	[MT_MUX_SPI] = {
		.name = __stringify(MT_MUX_SPI),
		.base_addr = CLK_CFG_2,
		.sel_mask = 0x00000700,
		.pdn_mask = 0x00008000,
		.offset = 8,
		.nr_inputs = 5,
		.ops = &clkmux_ops,
	},
	[MT_MUX_UART] = {
		.name = __stringify(MT_MUX_UART),
		.base_addr = CLK_CFG_2,
		.sel_mask = 0x00000001,
		.pdn_mask = 0x00000080,
		.offset = 0,
		.nr_inputs = 2,
		.ops = &clkmux_ops,
	},
	[MT_MUX_AUDINTBUS] = {
		.name = __stringify(MT_MUX_AUDINTBUS),
		.base_addr = CLK_CFG_3,
		.sel_mask = 0x07000000,
		.pdn_mask = 0x80000000,
		.offset = 24,
		.nr_inputs = 6,
		.ops = &clkmux_ops,
		.siblings = &muxs[MT_MUX_AUDIO],
	},
	[MT_MUX_AUDIO] = {
		.name = __stringify(MT_MUX_AUDIO),
		.base_addr = CLK_CFG_3,
		.sel_mask = 0x00010000,
		.pdn_mask = 0x00800000,
		.offset = 16,
		.nr_inputs = 2,
		.ops = &clkmux_ops,
	},
	[MT_MUX_MSDC30_2] = {
		.name = __stringify(MT_MUX_MSDC30_2),
		.base_addr = CLK_CFG_3,
		.sel_mask = 0x00000700,
		.pdn_mask = 0x00008000,
		.offset = 8,
		.nr_inputs = 6,
		.ops = &clkmux_ops,
		.pll = &plls[MSDCPLL],
	},
	[MT_MUX_MSDC30_1] = {
		.name = __stringify(MT_MUX_MSDC30_1),
		.base_addr = CLK_CFG_3,
		.sel_mask = 0x00000007,
		.pdn_mask = 0x00000080,
		.offset = 0,
		.nr_inputs = 6,
		.ops = &clkmux_ops,
		.pll = &plls[MSDCPLL],
	},
	[MT_MUX_DPI1] = {
		.name = __stringify(MT_MUX_DPI1),
		.base_addr = CLK_CFG_4,
		.sel_mask = 0x03000000,
		.pdn_mask = 0x80000000,
		.offset = 24,
		.nr_inputs = 4,
		.ops = &clkmux_ops,
		.pll = &plls[TVDPLL],
	},
	[MT_MUX_DPI0] = {
		.name = __stringify(MT_MUX_DPI0),
		.base_addr = CLK_CFG_4,
		.sel_mask = 0x00070000,
		.pdn_mask = 0x00800000,
		.offset = 16,
		.nr_inputs = 8,
		.ops = &dpi0_clkmux_ops,
		.pll = &plls[LVDSPLL],
	},
	[MT_MUX_SCP] = {
		.name = __stringify(MT_MUX_SCP),
		.base_addr = CLK_CFG_4,
		.sel_mask = 0x00000300,
		.pdn_mask = 0x00008000,
		.offset = 8,
		.nr_inputs = 4,
		.ops = &clkmux_ops,
	},
	[MT_MUX_PMICSPI] = {
		.name = __stringify(MT_MUX_PMICSPI),
		.base_addr = CLK_CFG_4,
		.sel_mask = 0x0000000F,
		.pdn_mask = 0x00000080,
		.offset = 0,
		.nr_inputs = 11,
		.ops = &clkmux_ops,
	},
	[MT_MUX_DPILVDS] = {
		.name = __stringify(MT_MUX_DPILVDS),
		.base_addr = CLK_CFG_5,
		.sel_mask = 0x07000000,
		.pdn_mask = 0x80000000,
		.offset = 24,
		.nr_inputs = 8,
		.ops = &clkmux_ops,
		.pll = &plls[LVDSPLL],
	},
	[MT_MUX_APLL] = {
		.name = __stringify(MT_MUX_APLL),
		.base_addr = CLK_CFG_5,
		.sel_mask = 0x00070000,
		.pdn_mask = 0x00800000,
		.offset = 16,
		.nr_inputs = 8,
		.ops = &clkmux_ops,
		.pll = &plls[AUDPLL],
	},
	[MT_MUX_HDMI] = {
		.name = __stringify(MT_MUX_HDMI),
		.base_addr = CLK_CFG_5,
		.sel_mask = 0x00000300,
		.pdn_mask = 0x00008000,
		.offset = 8,
		.nr_inputs = 4,
		.ops = &clkmux_ops,
	},
	[MT_MUX_TVE] = {
		.name = __stringify(MT_MUX_TVE),
		.base_addr = CLK_CFG_5,
		.sel_mask = 0x00000007,
		.pdn_mask = 0x00000080,
		.offset = 0,
		.nr_inputs = 8,
		.ops = &clkmux_ops,
		.pll = &plls[TVDPLL],
	},
	[MT_MUX_ETH_50M] = {
		.name = __stringify(MT_MUX_ETH_50M),
		.base_addr = CLK_CFG_6,
		.sel_mask = 0x00070000,
		.pdn_mask = 0x00800000,
		.offset = 16,
		.nr_inputs = 8,
		.ops = &clkmux_ops,
		.pll = &plls[LVDSPLL],
	},
	[MT_MUX_NFI2X] = {
		.name = __stringify(MT_MUX_NFI2X),
		.base_addr = CLK_CFG_6,
		.sel_mask = 0x00000700,
		.pdn_mask = 0x00008000,
		.offset = 8,
		.nr_inputs = 8,
		.ops = &clkmux_ops,
	},
	[MT_MUX_RTC] = {
		.name = __stringify(MT_MUX_RTC),
		.base_addr = CLK_CFG_6,
		.sel_mask = 0x00000003,
		.pdn_mask = 0x00000080,
		.offset = 0,
		.nr_inputs = 4,
		.ops = &clkmux_ops,
	},
};


static struct clkmux *id_to_mux(unsigned int id)
{
	return id < NR_MUXS ? muxs + id : NULL;
}

static void clkmux_sel_op(struct clkmux *mux, unsigned clksrc)
{
	unsigned int reg;

#ifdef MUX_LOG
	clk_dbg("[%s]: mux->name=%s, clksrc=%d\n", __func__, mux->name, clksrc);
#endif

	reg = clk_readl(mux->base_addr);

	reg &= ~(mux->sel_mask);
	reg |= (clksrc << mux->offset) & mux->sel_mask;

	clk_writel(mux->base_addr, reg);
}

static void clkmux_enable_op(struct clkmux *mux)
{
#ifdef MUX_LOG
	clk_dbg("[%s]: mux->name=%s\n", __func__, mux->name);
#endif

	clk_clrl(mux->base_addr, mux->pdn_mask);
}

static void clkmux_disable_op(struct clkmux *mux)
{
#ifdef MUX_LOG
	clk_dbg("[%s]: mux->name=%s\n", __func__, mux->name);
#endif

	clk_setl(mux->base_addr, mux->pdn_mask);
}

static struct clkmux_ops clkmux_ops = {
	.sel = clkmux_sel_op,
	.enable = clkmux_enable_op,
	.disable = clkmux_disable_op,
};


static void dpi0_clkmux_sel_op(struct clkmux *mux, unsigned clksrc)
{
	unsigned int reg;

#ifdef MUX_LOG
	clk_dbg("[%s]: mux->name=%s, clksrc=%d\n", __func__, mux->name, clksrc);
#endif

	reg = clk_readl(mux->base_addr);

#if WORKAROUND_DPI0_MUX

	{
		unsigned int old_setting = (reg & mux->sel_mask) >> mux->offset;
		unsigned int lsb_mask = 0x3;    /* lowest 2 bits */

		if ((old_setting != clksrc) && ((old_setting ^ clksrc) & lsb_mask) == 0) {
			/* if lowest 2 bits are the same, then change mux to a temp setting */
			unsigned int temp_setting[] = {5, 4, 5, 6};
			reg &= ~(mux->sel_mask);
			reg |= (temp_setting[clksrc & lsb_mask] << mux->offset) & mux->sel_mask;
			clk_writel(mux->base_addr, reg);

			reg = clk_readl(mux->base_addr);
		}
	}

#endif /* WORKAROUND_DPI0_MUX */

	reg &= ~(mux->sel_mask);
	reg |= (clksrc << mux->offset) & mux->sel_mask;
	clk_writel(mux->base_addr, reg);
}


static struct clkmux_ops dpi0_clkmux_ops = {
	.sel = dpi0_clkmux_sel_op,
	.enable = clkmux_enable_op,
	.disable = clkmux_disable_op,
};

static void clkmux_sel_locked(struct clkmux *mux, unsigned int clksrc)
{
	mux->ops->sel(mux, clksrc);
}

static void mux_enable_locked(struct clkmux *mux)
{
#ifdef MUX_LOG_TOP
	clk_info("[%s]: Start. mux->name=%s, mux->cnt=%d\n", __func__, mux->name, mux->cnt);
#endif

	mux->cnt++;

	if (mux->cnt > 1)
		return;

	if (mux->pll)
		pll_enable_internal(mux->pll, "mux");

	mux->ops->enable(mux);

	if (mux->siblings)
		mux_enable_internal(mux->siblings, "mux_s");

#ifdef MUX_LOG_TOP
	clk_info("[%s]: End. mux->name=%s, mux->cnt=%d\n", __func__, mux->name, mux->cnt);
#endif
}

static void mux_disable_locked(struct clkmux *mux)
{
#ifdef MUX_LOG_TOP
	clk_info("[%s]: Start. mux->name=%s, mux->cnt=%d\n", __func__, mux->name, mux->cnt);
#endif

#if DISABLE_BUG_ON_CNT
	WARN_ON(!mux->cnt);
	if (!mux->cnt)
		return 0;
#else /* !DISABLE_BUG_ON_CNT */
	BUG_ON(!mux->cnt);
#endif /* DISABLE_BUG_ON_CNT */

	mux->cnt--;

	if (mux->cnt > 0)
		return;

	mux->ops->disable(mux);

	if (mux->siblings)
		mux_disable_internal(mux->siblings, "mux_s");

	if (mux->pll)
		pll_disable_internal(mux->pll, "mux");

#ifdef MUX_LOG_TOP
	clk_info("[%s]: End. mux->name=%s, mux->cnt=%d\n", __func__, mux->name, mux->cnt);
#endif
}

int clkmux_sel(int id, unsigned int clksrc, char *name)
{
	unsigned long flags;
	struct clkmux *mux = id_to_mux(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!mux);
	BUG_ON(clksrc >= mux->nr_inputs);

	clkmgr_lock(flags);
	clkmux_sel_locked(mux, clksrc);
	clkmgr_unlock(flags);

	return 0;
}
EXPORT_SYMBOL(clkmux_sel);

void enable_mux(int id, char *name)
{
	unsigned long flags;
	struct clkmux *mux = id_to_mux(id);

#if defined(CONFIG_CLKMGR_BRINGUP) || defined(CONFIG_MT8127_FPGA)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!mux);
	BUG_ON(!name);
#ifdef MUX_LOG_TOP
	clk_info("[%s]: id=%d, name=%s\n", __func__, id, name);
#endif
	clkmgr_lock(flags);
	mux_enable_internal(mux, name);
	clkmgr_unlock(flags);

	return;
}
EXPORT_SYMBOL(enable_mux);

void disable_mux(int id, char *name)
{
	unsigned long flags;
	struct clkmux *mux = id_to_mux(id);

#if defined(CONFIG_CLKMGR_BRINGUP) || defined(CONFIG_MT8127_FPGA)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!mux);
	BUG_ON(!name);
#ifdef MUX_LOG_TOP
	clk_info("[%s]: id=%d, name=%s\n", __func__, id, name);
#endif
	clkmgr_lock(flags);
	mux_disable_internal(mux, name);
	clkmgr_unlock(flags);

	return;
}
EXPORT_SYMBOL(disable_mux);


/************************************************
 **********         cg_grp part        **********
 ************************************************/

static struct cg_grp_ops general_cg_grp_ops;
static struct cg_grp_ops vdec_cg_grp_ops;


static struct cg_grp grps[NR_GRPS] = {
	[CG_PERI0] = {
		.name = __stringify(CG_PERI0),
		.set_addr = PERI_PDN0_SET,
		.clr_addr = PERI_PDN0_CLR,
		.sta_addr = PERI_PDN0_STA,
		.ops = &general_cg_grp_ops,
	},
	[CG_PERI1] = {
		.name = __stringify(CG_PERI1),
		.set_addr = PERI_PDN1_SET,
		.clr_addr = PERI_PDN1_CLR,
		.sta_addr = PERI_PDN1_STA,
		.ops = &general_cg_grp_ops,
	},
	[CG_INFRA] = {
		.name = __stringify(CG_INFRA),
		.set_addr = INFRA_PDN_SET,
		.clr_addr = INFRA_PDN_CLR,
		.sta_addr = INFRA_PDN_STA,
		.ops = &general_cg_grp_ops,
	},
	[CG_TOPCK] = {
		.name = __stringify(CG_TOPCK),
		.set_addr = CLK_CFG_4_SET,
		.clr_addr = CLK_CFG_4_CLR,
		.sta_addr = CLK_CFG_4,
		.ops = &general_cg_grp_ops,
	},
	[CG_DISP0] = {
		.name = __stringify(CG_DISP0),
		.set_addr = DISP_CG_SET0,
		.clr_addr = DISP_CG_CLR0,
		.sta_addr = DISP_CG_CON0,
		.ops = &general_cg_grp_ops,
		.sys = &syss[SYS_DIS],
	},
	[CG_DISP1] = {
		.name = __stringify(CG_DISP1),
		.set_addr = DISP_CG_SET1,
		.clr_addr = DISP_CG_CLR1,
		.sta_addr = DISP_CG_CON1,
		.ops = &general_cg_grp_ops,
		.sys = &syss[SYS_DIS],
	},
	[CG_IMAGE] = {
		.name = __stringify(CG_IMAGE),
		.set_addr = IMG_CG_SET,
		.clr_addr = IMG_CG_CLR,
		.sta_addr = IMG_CG_CON,
		.ops = &general_cg_grp_ops,
		.sys = &syss[SYS_ISP],
	},
	[CG_MFG] = {
		.name = __stringify(CG_MFG),
		.set_addr = MFG_CG_SET,
		.clr_addr = MFG_CG_CLR,
		.sta_addr = MFG_CG_CON,
		.ops = &general_cg_grp_ops,
		.sys = &syss[SYS_MFG],
	},
	[CG_AUDIO] = {
		.name = __stringify(CG_AUDIO),
		.sta_addr = AUDIO_TOP_CON0,
		.ops = &general_cg_grp_ops,
	},
	[CG_VDEC0] = {
		.name = __stringify(CG_VDEC0),
		.set_addr = VDEC_CKEN_CLR,    /* revert CLR and SET here to reuse general_cg_grp_ops */
		.clr_addr = VDEC_CKEN_SET,
		.ops = &vdec_cg_grp_ops,
		.sys = &syss[SYS_VDE],
	},
	[CG_VDEC1] = {
		.name = __stringify(CG_VDEC1),
		.set_addr = LARB_CKEN_CLR,    /* revert CLR and SET here to reuse general_cg_grp_ops */
		.clr_addr = LARB_CKEN_SET,
		.ops = &vdec_cg_grp_ops,
		.sys = &syss[SYS_VDE],
	}
};

static struct cg_grp *id_to_grp(unsigned int id)
{
	return id < NR_GRPS ? grps + id : NULL;
}

static unsigned int general_grp_get_state_op(struct cg_grp *grp)
{
	unsigned int val;
	struct subsys *sys = grp->sys;

	if (sys && !sys->state)
		return 0;

	val = clk_readl(grp->sta_addr);
	val = (~val) & (grp->mask);
	return val;
}

static int general_grp_dump_regs_op(struct cg_grp *grp, unsigned int *ptr)
{
	*(ptr) = clk_readl(grp->sta_addr);

	return 1;
}

static struct cg_grp_ops general_cg_grp_ops = {
	.get_state = general_grp_get_state_op,
	.dump_regs = general_grp_dump_regs_op,
};

static unsigned int vdec_grp_get_state_op(struct cg_grp *grp)
{
	unsigned int val = clk_readl(grp->set_addr);
	val &= grp->mask;
	return val;
}

static int vdec_grp_dump_regs_op(struct cg_grp *grp, unsigned int *ptr)
{
	*(ptr) = clk_readl(grp->set_addr);
	*(++ptr) = clk_readl(grp->clr_addr);

	return 2;
}

static struct cg_grp_ops vdec_cg_grp_ops = {
	.get_state = vdec_grp_get_state_op,
	.dump_regs = vdec_grp_dump_regs_op,
};



/************************************************
 **********         cg_clk part        **********
 ************************************************/

static struct cg_clk_ops general_cg_clk_ops;
static struct cg_clk_ops audio_cg_clk_ops;
static struct cg_clk_ops audsys_cg_clk_ops; /* @audio sys */
static struct cg_clk_ops vdec_cg_clk_ops;


static struct cg_clk clks[NR_CLKS] = {
	[MT_CG_PERI_NFI] = {
		.name = "MT_CG_PERI_NFI",
		.grp = &grps[CG_PERI0],
		.mask = BIT(0),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_NFI2X],
	},
	[MT_CG_PERI_THERM] = {
		.name = "MT_CG_PERI_THERM",
		.grp = &grps[CG_PERI0],
		.mask = BIT(1),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_PWM1] = {
		.name = "MT_CG_PERI_PWM1",
		.grp = &grps[CG_PERI0],
		.mask = BIT(2),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_PWM2] = {
		.name = "MT_CG_PERI_PWM2",
		.grp = &grps[CG_PERI0],
		.mask = BIT(3),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_PWM3] = {
		.name = "MT_CG_PERI_PWM3",
		.grp = &grps[CG_PERI0],
		.mask = BIT(4),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_PWM4] = {
		.name = "MT_CG_PERI_PWM4",
		.grp = &grps[CG_PERI0],
		.mask = BIT(5),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_PWM5] = {
		.name = "MT_CG_PERI_PWM5",
		.grp = &grps[CG_PERI0],
		.mask = BIT(6),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_PWM6] = {
		.name = "MT_CG_PERI_PWM6",
		.grp = &grps[CG_PERI0],
		.mask = BIT(7),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_PWM7] = {
		.name = "MT_CG_PERI_PWM7",
		.grp = &grps[CG_PERI0],
		.mask = BIT(8),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_PWM] = {
		.name = "MT_CG_PERI_PWM",
		.grp = &grps[CG_PERI0],
		.mask = BIT(9),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_USB0] = {
		.name = "MT_CG_PERI_USB0",
		.grp = &grps[CG_PERI0],
		.mask = BIT(10),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_USB20],
	},
	[MT_CG_PERI_USB1] = {
		.name = "MT_CG_PERI_USB1",
		.grp = &grps[CG_PERI0],
		.mask = BIT(11),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_USB20],
	},
	[MT_CG_PERI_AP_DMA] = {
		.name = "MT_CG_PERI_AP_DMA",
		.grp = &grps[CG_PERI0],
		.mask = BIT(12),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_MSDC30_0] = {
		.name = "MT_CG_PERI_MSDC30_0",
		.grp = &grps[CG_PERI0],
		.mask = BIT(13),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_MSDC30_0],
	},
	[MT_CG_PERI_MSDC30_1] = {
		.name = "MT_CG_PERI_MSDC30_1",
		.grp = &grps[CG_PERI0],
		.mask = BIT(14),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_MSDC30_1],
	},
	[MT_CG_PERI_MSDC30_2] = {
		.name = "MT_CG_PERI_MSDC30_2",
		.grp = &grps[CG_PERI0],
		.mask = BIT(15),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_MSDC30_2],
	},
	[MT_CG_PERI_NLI] = {
		.name = "MT_CG_PERI_NLI",
		.grp = &grps[CG_PERI0],
		.mask = BIT(16),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_UART0] = {
		.name = "MT_CG_PERI_UART0",
		.grp = &grps[CG_PERI0],
		.mask = BIT(17),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_UART],
	},
	[MT_CG_PERI_UART1] = {
		.name = "MT_CG_PERI_UART1",
		.grp = &grps[CG_PERI0],
		.mask = BIT(18),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_UART],
	},
	[MT_CG_PERI_UART2] = {
		.name = "MT_CG_PERI_UART2",
		.grp = &grps[CG_PERI0],
		.mask = BIT(19),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_UART],
	},
	[MT_CG_PERI_UART3] = {
		.name = "MT_CG_PERI_UART3",
		.grp = &grps[CG_PERI0],
		.mask = BIT(20),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_UART],
	},
	[MT_CG_PERI_BTIF] = {
		.name = "MT_CG_PERI_BTIF",
		.grp = &grps[CG_PERI0],
		.mask = BIT(21),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_I2C0] = {
		.name = "MT_CG_PERI_I2C0",
		.grp = &grps[CG_PERI0],
		.mask = BIT(22),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_I2C1] = {
		.name = "MT_CG_PERI_I2C1",
		.grp = &grps[CG_PERI0],
		.mask = BIT(23),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_I2C2] = {
		.name = "MT_CG_PERI_I2C2",
		.grp = &grps[CG_PERI0],
		.mask = BIT(24),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_I2C3] = {
		.name = "MT_CG_PERI_I2C3",
		.grp = &grps[CG_PERI0],
		.mask = BIT(25),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_AUXADC] = {
		.name = "MT_CG_PERI_AUXADC",
		.grp = &grps[CG_PERI0],
		.mask = BIT(26),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_SPI0] = {
		.name = "MT_CG_PERI_SPI0",
		.grp = &grps[CG_PERI0],
		.mask = BIT(27),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_SPI],
	},
	[MT_CG_PERI_ETH] = {
		.name = "MT_CG_PERI_ETH",
		.grp = &grps[CG_PERI0],
		.mask = BIT(28),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_ETH_50M],
	},
	[MT_CG_PERI_USB0_MCU] = {
		.name = "MT_CG_PERI_USB0_MCU",
		.grp = &grps[CG_PERI0],
		.mask = BIT(29),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_USB1_MCU] = {
		.name = "MT_CG_PERI_USB1_MCU",
		.grp = &grps[CG_PERI0],
		.mask = BIT(30),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_USB_SLV] = {
		.name = "MT_CG_PERI_USB_SLV",
		.grp = &grps[CG_PERI0],
		.mask = BIT(31),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_GCPU] = {
		.name = "MT_CG_PERI_GCPU",
		.grp = &grps[CG_PERI1],
		.mask = BIT(0),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_PERI_NFI_ECC] = {
		.name = "MT_CG_PERI_NFI_ECC",
		.grp = &grps[CG_PERI1],
		.mask = BIT(1),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_NFI2X],
	},
	[MT_CG_PERI_NFIPAD] = {
		.name = "MT_CG_PERI_NFIPAD",
		.grp = &grps[CG_PERI1],
		.mask = BIT(2),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_NFI2X],
	},

	[MT_CG_INFRA_DBGCLK] = {
		.name = "MT_CG_INFRA_DBGCLK",
		.grp = &grps[CG_INFRA],
		.mask = BIT(0),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_INFRA_SMI] = {
		.name = "MT_CG_INFRA_SMI",
		.grp = &grps[CG_INFRA],
		.mask = BIT(1),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_INFRA_AUDIO] = {
		.name = "MT_CG_INFRA_AUDIO",
		.grp = &grps[CG_INFRA],
		.mask = BIT(5),
		.ops = &audio_cg_clk_ops,
		.mux = &muxs[MT_MUX_AUDINTBUS],
	},
	[MT_CG_INFRA_EFUSE] = {
		.name = "MT_CG_INFRA_EFUSE",
		.grp = &grps[CG_INFRA],
		.mask = BIT(6),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_INFRA_L2C_SRAM] = {
		.name = "MT_CG_INFRA_L2C_SRAM",
		.grp = &grps[CG_INFRA],
		.mask = BIT(7),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_INFRA_M4U] = {
		.name = "MT_CG_INFRA_M4U",
		.grp = &grps[CG_INFRA],
		.mask = BIT(8),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_INFRA_CONNMCU] = {
		.name = "MT_CG_INFRA_CONNMCU",
		.grp = &grps[CG_INFRA],
		.mask = BIT(12),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_INFRA_TRNG] = {
		.name = "MT_CG_INFRA_TRNG",
		.grp = &grps[CG_INFRA],
		.mask = BIT(13),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_INFRA_CPUM] = {
		.name = "MT_CG_INFRA_CPUM",
		.grp = &grps[CG_INFRA],
		.mask = BIT(15),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_INFRA_KP] = {
		.name = "MT_CG_INFRA_KP",
		.grp = &grps[CG_INFRA],
		.mask = BIT(16),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_INFRA_CEC] = {
		.name = "MT_CG_INFRA_CEC",
		.grp = &grps[CG_INFRA],
		.mask = BIT(18),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_INFRA_IRRX] = {
		.name = "MT_CG_INFRA_IRRX",
		.grp = &grps[CG_INFRA],
		.mask = BIT(19),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_INFRA_PMICSPI_SHARE] = {
		.name = "MT_CG_INFRA_PMICSPI_SHARE",
		.grp = &grps[CG_INFRA],
		.mask = BIT(22),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_PMICSPI],
	},
	[MT_CG_INFRA_PMICWRAP] = {
		.name = "MT_CG_INFRA_PMICWRAP",
		.grp = &grps[CG_INFRA],
		.mask = BIT(23),
		.ops = &general_cg_clk_ops,
	},

	[MT_CG_TOPCK_PMICSPI] = {
		.name = "MT_CG_TOPCK_PMICSPI",
		.grp = &grps[CG_TOPCK],
		.mask = BIT(5),
		.ops = &general_cg_clk_ops,
	},

	[MT_CG_DISP0_SMI_COMMON] = {
		.name = "MT_CG_DISP0_SMI_COMMON",
		.grp = &grps[CG_DISP0],
		.mask = BIT(0),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_SMI_LARB0] = {
		.name = "MT_CG_DISP0_SMI_LARB0",
		.grp = &grps[CG_DISP0],
		.mask = BIT(1),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_MM_CMDQ] = {
		.name = "MT_CG_DISP0_MM_CMDQ",
		.grp = &grps[CG_DISP0],
		.mask = BIT(2),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_MUTEX] = {
		.name = "MT_CG_DISP0_MUTEX",
		.grp = &grps[CG_DISP0],
		.mask = BIT(3),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_DISP_COLOR] = {
		.name = "MT_CG_DISP0_DISP_COLOR",
		.grp = &grps[CG_DISP0],
		.mask = BIT(4),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_DISP_BLS] = {
		.name = "MT_CG_DISP0_DISP_BLS",
		.grp = &grps[CG_DISP0],
		.mask = BIT(5),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_PWM],
	},
	[MT_CG_DISP0_DISP_WDMA] = {
		.name = "MT_CG_DISP0_DISP_WDMA",
		.grp = &grps[CG_DISP0],
		.mask = BIT(6),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_DISP_RDMA] = {
		.name = "MT_CG_DISP0_DISP_RDMA",
		.grp = &grps[CG_DISP0],
		.mask = BIT(7),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_DISP_OVL] = {
		.name = "MT_CG_DISP0_DISP_OVL",
		.grp = &grps[CG_DISP0],
		.mask = BIT(8),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_MDP_TDSHP] = {
		.name = "MT_CG_DISP0_MDP_TDSHP",
		.grp = &grps[CG_DISP0],
		.mask = BIT(9),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_MDP_WROT] = {
		.name = "MT_CG_DISP0_MDP_WROT",
		.grp = &grps[CG_DISP0],
		.mask = BIT(10),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_MDP_WDMA] = {
		.name = "MT_CG_DISP0_MDP_WDMA",
		.grp = &grps[CG_DISP0],
		.mask = BIT(11),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_MDP_RSZ1] = {
		.name = "MT_CG_DISP0_MDP_RSZ1",
		.grp = &grps[CG_DISP0],
		.mask = BIT(12),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_MDP_RSZ0] = {
		.name = "MT_CG_DISP0_MDP_RSZ0",
		.grp = &grps[CG_DISP0],
		.mask = BIT(13),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_MDP_RDMA] = {
		.name = "MT_CG_DISP0_MDP_RDMA",
		.grp = &grps[CG_DISP0],
		.mask = BIT(14),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_MDP_BLS_26M] = {
		.name = "MT_CG_DISP0_MDP_BLS_26M",
		.grp = &grps[CG_DISP0],
		.mask = BIT(15),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_PWM],
	},
	[MT_CG_DISP0_CAM_MDP] = {
		.name = "MT_CG_DISP0_CAM_MDP",
		.grp = &grps[CG_DISP0],
		.mask = BIT(16),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_FAKE_ENG] = {
		.name = "MT_CG_DISP0_FAKE_ENG",
		.grp = &grps[CG_DISP0],
		.mask = BIT(17),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_MUTEX_32K] = {
		.name = "MT_CG_DISP0_MUTEX_32K",
		.grp = &grps[CG_DISP0],
		.mask = BIT(18),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_DISP_RMDA1] = {
		.name = "MT_CG_DISP0_DISP_RMDA1",
		.grp = &grps[CG_DISP0],
		.mask = BIT(19),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP0_DISP_UFOE] = {
		.name = "MT_CG_DISP0_DISP_UFOE",
		.grp = &grps[CG_DISP0],
		.mask = BIT(20),
		.ops = &general_cg_clk_ops,
	},

	[MT_CG_DISP1_DSI_ENGINE] = {
		.name = "MT_CG_DISP1_DSI_ENGINE",
		.grp = &grps[CG_DISP1],
		.mask = BIT(0),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP1_DSI_DIGITAL] = {
		.name = "MT_CG_DISP1_DSI_DIGITAL",
		.grp = &grps[CG_DISP1],
		.mask = BIT(1),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP1_DPI_DIGITAL_LANE] = {
		.name = "MT_CG_DISP1_DPI_DIGITAL_LANE",
		.grp = &grps[CG_DISP1],
		.mask = BIT(2),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP1_DPI_ENGINE] = {
		.name = "MT_CG_DISP1_DPI_ENGINE",
		.grp = &grps[CG_DISP1],
		.mask = BIT(3),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP1_DPI1_DIGITAL_LANE] = {
		.name = "MT_CG_DISP1_DPI1_DIGITAL_LANE",
		.grp = &grps[CG_DISP1],
		.mask = BIT(4),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_DPI1],
	},
	[MT_CG_DISP1_DPI1_ENGINE] = {
		.name = "MT_CG_DISP1_DPI1_ENGINE",
		.grp = &grps[CG_DISP1],
		.mask = BIT(5),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP1_TVE_OUTPUT_CLOCK] = {
		.name = "MT_CG_DISP1_TVE_OUTPUT_CLOCK",
		.grp = &grps[CG_DISP1],
		.mask = BIT(6),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_TVE],
	},
	[MT_CG_DISP1_TVE_INPUT_CLOCK] = {
		.name = "MT_CG_DISP1_TVE_INPUT_CLOCK",
		.grp = &grps[CG_DISP1],
		.mask = BIT(7),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP1_HDMI_PIXEL_CLOCK] = {
		.name = "MT_CG_DISP1_HDMI_PIXEL_CLOCK",
		.grp = &grps[CG_DISP1],
		.mask = BIT(8),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_DPI1],
	},
	[MT_CG_DISP1_HDMI_PLL_CLOCK] = {
		.name = "MT_CG_DISP1_HDMI_PLL_CLOCK",
		.grp = &grps[CG_DISP1],
		.mask = BIT(9),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_HDMI],
	},
	[MT_CG_DISP1_HDMI_AUDIO_CLOCK] = {
		.name = "MT_CG_DISP1_HDMI_AUDIO_CLOCK",
		.grp = &grps[CG_DISP1],
		.mask = BIT(10),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP1_HDMI_SPDIF_CLOCK] = {
		.name = "MT_CG_DISP1_HDMI_SPDIF_CLOCK",
		.grp = &grps[CG_DISP1],
		.mask = BIT(11),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_APLL],
	},
	[MT_CG_DISP1_LVDS_PIXEL_CLOCK] = {
		.name = "MT_CG_DISP1_LVDS_PIXEL_CLOCK",
		.grp = &grps[CG_DISP1],
		.mask = BIT(12),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_DISP1_LVDS_CTS_CLOCK] = {
		.name = "MT_CG_DISP1_LVDS_CTS_CLOCK",
		.grp = &grps[CG_DISP1],
		.mask = BIT(13),
		.ops = &general_cg_clk_ops,
	},

	[MT_CG_IMAGE_LARB2_SMI] = {
		.name = "MT_CG_IMAGE_LARB2_SMI",
		.grp = &grps[CG_IMAGE],
		.mask = BIT(0),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_IMAGE_CAM_SMI] = {
		.name = "MT_CG_IMAGE_CAM_SMI",
		.grp = &grps[CG_IMAGE],
		.mask = BIT(5),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_IMAGE_CAM_CAM] = {
		.name = "MT_CG_IMAGE_CAM_CAM",
		.grp = &grps[CG_IMAGE],
		.mask = BIT(6),
		.ops = &general_cg_clk_ops,
	},
	[MT_CG_IMAGE_SEN_TG] = {
		.name = "MT_CG_IMAGE_SEN_TG",
		.grp = &grps[CG_IMAGE],
		.mask = BIT(7),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_CAMTG],
	},
	[MT_CG_IMAGE_SEN_CAM] = {
		.name = "MT_CG_IMAGE_SEN_CAM",
		.grp = &grps[CG_IMAGE],
		.mask = BIT(8),
		.ops = &general_cg_clk_ops,
		.mux = &muxs[MT_MUX_CAMTG],
	},
	[MT_CG_IMAGE_VENC_JPENC] = {
		.name = "MT_CG_IMAGE_VENC_JPENC",
		.grp = &grps[CG_IMAGE],
		.mask = BIT(9),
		.ops = &general_cg_clk_ops,
	},

	[MT_CG_MFG_G3D] = {
		.name = "MT_CG_MFG_G3D",
		.grp = &grps[CG_MFG],
		.mask = BIT(0),
		.ops = &general_cg_clk_ops,
	},

	[MT_CG_AUDIO_AFE] = {
		.name = "MT_CG_AUDIO_AFE",
		.grp = &grps[CG_AUDIO],
		.mask = BIT(2),
		.ops = &audsys_cg_clk_ops,
		.parent = &clks[MT_CG_INFRA_AUDIO],
	},
	[MT_CG_AUDIO_I2S] = {
		.name = "MT_CG_AUDIO_I2S",
		.grp = &grps[CG_AUDIO],
		.mask = BIT(6),
		.ops = &audsys_cg_clk_ops,
		.parent = &clks[MT_CG_INFRA_AUDIO],
	},
	[MT_CG_AUDIO_APLL_TUNER_CK] = {
		.name = "MT_CG_AUDIO_APLL_TUNER_CK",
		.grp = &grps[CG_AUDIO],
		.mask = BIT(19),
		.ops = &audsys_cg_clk_ops,
		.mux = &muxs[MT_MUX_APLL],
		.parent = &clks[MT_CG_INFRA_AUDIO],
	},
	[MT_CG_AUDIO_HDMI_CK] = {
		.name = "MT_CG_AUDIO_HDMI_CK",
		.grp = &grps[CG_AUDIO],
		.mask = BIT(20),
		.ops = &audsys_cg_clk_ops,
		.mux = &muxs[MT_MUX_APLL],
		.parent = &clks[MT_CG_INFRA_AUDIO],
	},
	[MT_CG_AUDIO_SPDF_CK] = {
		.name = "MT_CG_AUDIO_SPDF_CK",
		.grp = &grps[CG_AUDIO],
		.mask = BIT(21),
		.ops = &audsys_cg_clk_ops,
		.mux = &muxs[MT_MUX_APLL],
		.parent = &clks[MT_CG_INFRA_AUDIO],
	},
	[MT_CG_AUDIO_SPDF2_CK] = {
		.name = "MT_CG_AUDIO_SPDF2_CK",
		.grp = &grps[CG_AUDIO],
		.mask = BIT(22),
		.ops = &audsys_cg_clk_ops,
		.mux = &muxs[MT_MUX_APLL],
		.parent = &clks[MT_CG_INFRA_AUDIO],
	},

	[MT_CG_VDEC0_VDEC] = {
		.name = "MT_CG_VDEC0_VDEC",
		.grp = &grps[CG_VDEC0],
		.mask = BIT(0),
		.ops = &vdec_cg_clk_ops,
		.mux = &muxs[MT_MUX_VDEC],
	},

	[MT_CG_VDEC1_LARB] = {
		.name = "MT_CG_VDEC1_LARB",
		.grp = &grps[CG_VDEC1],
		.mask = BIT(0),
		.ops = &vdec_cg_clk_ops,
	},
};


static struct cg_clk *id_to_clk(unsigned int id)
{
	return id < NR_CLKS ? clks + id : NULL;
}

static int general_clk_get_state_op(struct cg_clk *clk)
{
	struct subsys *sys = clk->grp->sys;
	if (sys && !sys->state)
		return PWR_DOWN;

	return (clk_readl(clk->grp->sta_addr) & (clk->mask)) ? PWR_DOWN : PWR_ON;
}

static int general_clk_check_validity_op(struct cg_clk *clk)
{
	int valid = 0;
	if (clk->mask & clk->grp->mask)
		valid = 1;

	return valid;
}

static int general_clk_enable_op(struct cg_clk *clk)
{
#ifdef CLK_LOG
	clk_info("[%s]: clk->grp->name=%s, clk->mask=0x%x\n", __func__, clk->grp->name, clk->mask);
#endif

	clk_writel(clk->grp->clr_addr, clk->mask);
	return 0;
}

static int general_clk_disable_op(struct cg_clk *clk)
{
#ifdef CLK_LOG
	clk_info("[%s]: clk->grp->name=%s, clk->mask=0x%x\n", __func__, clk->grp->name, clk->mask);
#endif

	clk_writel(clk->grp->set_addr, clk->mask);
	return 0;
}

static struct cg_clk_ops general_cg_clk_ops = {
	.get_state = general_clk_get_state_op,
	.check_validity = general_clk_check_validity_op,
	.enable = general_clk_enable_op,
	.disable = general_clk_disable_op,
};

static int audio_clk_enable_op(struct cg_clk *clk)
{
#ifdef CLK_LOG
	clk_info("[%s]: clk->grp->name=%s, clk->mask=0x%x\n", __func__, clk->grp->name, clk->mask);
#endif

	clk_writel(clk->grp->clr_addr, clk->mask);
	return 0;
}

static int audio_clk_disable_op(struct cg_clk *clk)
{
#ifdef CLK_LOG
	clk_info("[%s]: clk->grp->name=%s, clk->mask=0x%x\n", __func__, clk->grp->name, clk->mask);
#endif

	clk_writel(clk->grp->set_addr, clk->mask);
	return 0;
}

static struct cg_clk_ops audio_cg_clk_ops = {
	.get_state = general_clk_get_state_op,
	.check_validity = general_clk_check_validity_op,
	.enable = audio_clk_enable_op,
	.disable = audio_clk_disable_op,
};


static int audsys_clk_enable_op(struct cg_clk *clk)
{
	clk_clrl(clk->grp->sta_addr, clk->mask);
	return 0;
}

static int audsys_clk_disable_op(struct cg_clk *clk)
{
	clk_setl(clk->grp->sta_addr, clk->mask);
	return 0;
}

static struct cg_clk_ops audsys_cg_clk_ops = {
	.get_state = general_clk_get_state_op,
	.check_validity = general_clk_check_validity_op,
	.enable = audsys_clk_enable_op,
	.disable = audsys_clk_disable_op,
};

static int vdec_clk_get_state_op(struct cg_clk *clk)
{
	return (clk_readl(clk->grp->set_addr) & (clk->mask)) ? PWR_ON : PWR_DOWN;
}

static struct cg_clk_ops vdec_cg_clk_ops = {
	.get_state = vdec_clk_get_state_op,
	.check_validity = general_clk_check_validity_op,
	.enable = general_clk_enable_op,
	.disable = general_clk_disable_op,
};


static int power_prepare_locked(struct cg_grp *grp)
{
	int err = 0;
	if (grp->sys)
		err = subsys_enable_internal(grp->sys, "clk");

	return err;
}

static int power_finish_locked(struct cg_grp *grp)
{
	int err = 0;
	if (grp->sys)
		err = subsys_disable_internal(grp->sys, 0, "clk");

	return err;
}

static int clk_enable_locked(struct cg_clk *clk)
{
	struct cg_grp *grp = clk->grp;
	unsigned int local_state;
#ifdef STATE_CHECK_DEBUG
	unsigned int reg_state;
#endif
	int err;

#ifdef CLK_LOG
	clk_info("[%s]: Start. grp->name=%s, grp->state=0x%x, clk->mask=0x%x, clk->cnt=%d, clk->state=%d\n",
		__func__, grp->name, grp->state, clk->mask, clk->cnt, clk->state);
#endif

	clk->cnt++;

	if (clk->cnt > 1)
		return 0;

	local_state = clk->state;

#ifdef STATE_CHECK_DEBUG
	reg_state = grp->ops->get_state(grp, clk);
	BUG_ON(local_state != reg_state);
#endif

#if !defined(CONFIG_MTK_LDVT)
	if (clk->mux)
		mux_enable_internal(clk->mux, "clk");

	err = power_prepare_locked(grp);
	BUG_ON(err);
#endif

	if (clk->parent)
		clk_enable_internal(clk->parent, "clk");

	if (local_state == PWR_ON)
		return 0;

	clk->ops->enable(clk);

	clk->state = PWR_ON;
	grp->state |= clk->mask;
#ifdef CLK_LOG
	clk_info("[%s]: End. grp->name=%s, grp->state=0x%x, clk->mask=0x%x, clk->cnt=%d, clk->state=%d\n",
		__func__, grp->name, grp->state, clk->mask, clk->cnt, clk->state);
#endif

	return 0;
}

static int clk_disable_locked(struct cg_clk *clk)
{
	struct cg_grp *grp = clk->grp;
	unsigned int local_state;
#ifdef STATE_CHECK_DEBUG
	unsigned int reg_state;
#endif
	int err;

#ifdef CLK_LOG
	clk_info("[%s]: Start. grp->name=%s, grp->state=0x%x, clk->mask=0x%x, clk->cnt=%d, clk->state=%d\n",
		__func__, grp->name, grp->state, clk->mask, clk->cnt, clk->state);
#endif

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 0;
#endif

#if DISABLE_BUG_ON_CNT
	WARN_ON(!clk->cnt);
	if (!clk->cnt)
		return 0;
#else /* !DISABLE_BUG_ON_CNT */
	BUG_ON(!clk->cnt);
#endif /* DISABLE_BUG_ON_CNT */

	clk->cnt--;

	if (clk->cnt > 0)
		return 0;

	local_state = clk->state;

#ifdef STATE_CHECK_DEBUG
	reg_state = grp->ops->get_state(grp, clk);
	BUG_ON(local_state != reg_state);
#endif

	if (local_state == PWR_DOWN)
		return 0;

	if (clk->force_on)
		return 0;

	clk->ops->disable(clk);

	clk->state = PWR_DOWN;
	grp->state &= ~(clk->mask);

	if (clk->parent)
		clk_disable_internal(clk->parent, "clk");

#if !defined(CONFIG_MTK_LDVT)
	err = power_finish_locked(grp);
	BUG_ON(err);

	if (clk->mux)
		mux_disable_internal(clk->mux, "clk");

#endif

#ifdef CLK_LOG
	clk_info("[%s]: End. grp->name=%s, grp->state=0x%x, clk->mask=0x%x, clk->cnt=%d, clk->state=%d\n",
		__func__, grp->name, grp->state, clk->mask, clk->cnt, clk->state);
#endif

	return 0;
}

static int get_clk_state_locked(struct cg_clk *clk)
{
	if (likely(initialized))
		return clk->state;
	else
		return clk->ops->get_state(clk);
}

int mt_enable_clock(enum cg_clk_id id, char *name)
{
	int err;
	unsigned long flags;
	struct cg_clk *clk = id_to_clk(id);

#if defined(CONFIG_CLKMGR_BRINGUP) || defined(CONFIG_MT8127_FPGA)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!clk);
	BUG_ON(!clk->grp);
	BUG_ON(!clk->ops->check_validity(clk));
	BUG_ON(!name);
#ifdef CLK_LOG_TOP
	clk_info("[%s]: id=%d, names=%s\n", __func__, id, name);
#else
	if (id == MT_CG_DISP0_SMI_COMMON)
		clk_dbg("[%s]: id=%d, names=%s\n", __func__, id, name);
#endif

	clkmgr_lock(flags);
	err = clk_enable_internal(clk, name);
	clkmgr_unlock(flags);

	return err;
}
EXPORT_SYMBOL(mt_enable_clock);


int mt_disable_clock(enum cg_clk_id id, char *name)
{
	int err;
	unsigned long flags;
	struct cg_clk *clk = id_to_clk(id);

#if defined(CONFIG_CLKMGR_BRINGUP) || defined(CONFIG_MT8127_FPGA)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!clk);
	BUG_ON(!clk->grp);
	BUG_ON(!clk->ops->check_validity(clk));
	BUG_ON(!name);
#ifdef CLK_LOG_TOP
	clk_info("[%s]: id=%d, names=%s\n", __func__, id, name);
#else
	if (id == MT_CG_DISP0_SMI_COMMON)
		clk_dbg("[%s]: id=%d, names=%s\n", __func__, id, name);
#endif

	clkmgr_lock(flags);
	err = clk_disable_internal(clk, name);
	clkmgr_unlock(flags);

	return err;
}
EXPORT_SYMBOL(mt_disable_clock);


#if !CLKMGR_8127


int enable_clock_ext_locked(int id, char *name)
{
	int err;
	struct cg_clk *clk = id_to_clk(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!clk);
	BUG_ON(!clk->grp);
	BUG_ON(!clk->ops->check_validity(clk));

	BUG_ON(!clkmgr_locked());
	err = clk_enable_internal(clk, name);

	return err;
}
EXPORT_SYMBOL(enable_clock_ext_locked);


int disable_clock_ext_locked(int id, char *name)
{
	int err;
	struct cg_clk *clk = id_to_clk(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 0;
#endif

	BUG_ON(!initialized);
	BUG_ON(!clk);
	BUG_ON(!clk->grp);
	BUG_ON(!clk->ops->check_validity(clk));

	BUG_ON(!clkmgr_locked());
	err = clk_disable_internal(clk, name);

	return err;
}
EXPORT_SYMBOL(disable_clock_ext_locked);


#endif /* !CLKMGR_8127 */


int clock_is_on(int id)
{
	int state;
	unsigned long flags;
	struct cg_clk *clk = id_to_clk(id);

#if defined(CONFIG_CLKMGR_BRINGUP) || defined(CONFIG_MT8127_FPGA)
	return 1;
#endif

	BUG_ON(!clk);
	BUG_ON(!clk->grp);
	BUG_ON(!clk->ops->check_validity(clk));

	clkmgr_lock(flags);
	state = get_clk_state_locked(clk);
	clkmgr_unlock(flags);

	return state;
}
EXPORT_SYMBOL(clock_is_on);


static void clk_set_force_on_locked(struct cg_clk *clk)
{
	clk->force_on = 1;
}

static void clk_clr_force_on_locked(struct cg_clk *clk)
{
	clk->force_on = 0;
}

void clk_set_force_on(int id)
{
	unsigned long flags;
	struct cg_clk *clk = id_to_clk(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return;
#endif

	BUG_ON(!initialized);
	BUG_ON(!clk);
	BUG_ON(!clk->grp);
	BUG_ON(!clk->ops->check_validity(clk));

	clkmgr_lock(flags);
	clk_set_force_on_locked(clk);
	clkmgr_unlock(flags);
}
EXPORT_SYMBOL(clk_set_force_on);

void clk_clr_force_on(int id)
{
	unsigned long flags;
	struct cg_clk *clk = id_to_clk(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return;
#endif

	BUG_ON(!initialized);
	BUG_ON(!clk);
	BUG_ON(!clk->grp);
	BUG_ON(!clk->ops->check_validity(clk));

	clkmgr_lock(flags);
	clk_clr_force_on_locked(clk);
	clkmgr_unlock(flags);
}
EXPORT_SYMBOL(clk_clr_force_on);

int clk_is_force_on(int id)
{
	struct cg_clk *clk = id_to_clk(id);

#if defined(CONFIG_CLKMGR_BRINGUP)
	return 1;
#endif

	BUG_ON(!initialized);
	BUG_ON(!clk);
	BUG_ON(!clk->grp);
	BUG_ON(!clk->ops->check_validity(clk));

	return clk->force_on;
}

int grp_dump_regs(int id, unsigned int *ptr)
{
	struct cg_grp *grp = id_to_grp(id);

	BUG_ON(!grp);

	return grp->ops->dump_regs(grp, ptr);
}
EXPORT_SYMBOL(grp_dump_regs);

const char *grp_get_name(int id)
{
	struct cg_grp *grp = id_to_grp(id);

	BUG_ON(!grp);

	return grp->name;
}

void print_grp_regs(void)
{
	int i;
	int cnt;
	unsigned int value[2];
	const char *name;

	for (i = 0; i < NR_GRPS; i++) {
		name = grp_get_name(i);
		cnt = grp_dump_regs(i, value);
		if (cnt == 1)
			clk_info("[%02d][%-8s]=[0x%08x]\n", i, name, value[0]);
		else
			clk_info("[%02d][%-8s]=[0x%08x][0x%08x]\n", i, name, value[0], value[1]);
	}
}

int clk_id_to_grp_id(enum cg_clk_id id)
{
	struct cg_clk *clk = id_to_clk(id);
	return (NULL == clk) ? NR_GRPS : (clk->grp - &grps[0]);
}

unsigned int clk_id_to_mask(enum cg_clk_id id)
{
	struct cg_clk *clk = id_to_clk(id);
	return (NULL == clk) ? 0 : clk->mask;
}



/************************************************
 **********       initialization       **********
 ************************************************/

static void cg_all_force_on(void)
{
	int i;
	for (i = 0; i < NR_CLKS; i++) {
		struct cg_clk *clk = id_to_clk(i);
		BUG_ON(!clk || !clk->ops);

		clk->ops->enable(clk);
#if WORKAROUND_DAPC
		clk->state = PWR_ON;
		clk->grp->mask |= clk->mask;
		clk->grp->state |= clk->mask;
#ifdef CONFIG_CLKMGR_STAT
		INIT_LIST_HEAD(&clk->head);
#endif
#endif /* !WORKAROUND_DAPC */
	}
}

static void cg_bootup_pdn(void)
{
	enum cg_clk_id ignored_pdn[] = {
		MT_CG_PERI_THERM,
		MT_CG_PERI_AP_DMA,
		MT_CG_PERI_UART0,
		MT_CG_PERI_AUXADC,
		MT_CG_PERI_USB0_MCU,
		MT_CG_PERI_USB1_MCU,
		MT_CG_PERI_USB_SLV,
		MT_CG_PERI_GCPU,
		MT_CG_INFRA_DBGCLK,
		MT_CG_INFRA_SMI,
		MT_CG_INFRA_EFUSE,
		MT_CG_INFRA_M4U,
		MT_CG_INFRA_KP,
		MT_CG_INFRA_PMICSPI_SHARE,
		MT_CG_INFRA_PMICWRAP,
		MT_CG_TOPCK_PMICSPI,
		MT_CG_DISP0_SMI_COMMON,
		MT_CG_DISP0_SMI_LARB0,
		MT_CG_DISP0_DISP_COLOR,
		MT_CG_DISP0_DISP_BLS,
		MT_CG_DISP0_DISP_RDMA,
		MT_CG_DISP0_DISP_OVL,
		MT_CG_DISP0_MDP_BLS_26M,
		MT_CG_DISP0_MUTEX_32K,
		MT_CG_DISP0_DISP_UFOE,
		MT_CG_DISP1_DSI_ENGINE,
		MT_CG_DISP1_DSI_DIGITAL,
		MT_CG_DISP1_DPI_DIGITAL_LANE,
		MT_CG_DISP1_DPI_ENGINE,
		MT_CG_DISP1_LVDS_PIXEL_CLOCK,
		MT_CG_DISP1_LVDS_CTS_CLOCK,
	};

	int len = ARRAY_SIZE(ignored_pdn);
	int i;
	for (i = NR_CLKS - 1; i >= 0; i--) {
		int j;
		struct cg_clk *clk;

		for (j = 0; j < len && i != ignored_pdn[j]; j++)
			;

		if (j != len)       /* ignore this clock */
			continue;

		clk = id_to_clk(i);
		clk->ops->disable(clk);
#if WORKAROUND_DAPC
		clk->state = PWR_DOWN;
		clk->grp->state &= ~(clk->mask);
#endif /* !WORKAROUND_DAPC */
	}
}


static void mt_subsys_init(void)
{
	int i;
	struct subsys *sys;

	for (i = 0; i < NR_SYSS; i++) {
		sys = &syss[i];
		sys->state = sys->ops->get_state(sys);
		if (sys->state != sys->default_sta) {
			clk_info("[%s]%s, change state: (%u->%u)\n", __func__,
					sys->name, sys->state, sys->default_sta);
			if (sys->default_sta == PWR_DOWN)
				sys_disable_locked(sys, 1);
			else
				sys_enable_locked(sys);
		}
#ifdef CONFIG_CLKMGR_STAT
		INIT_LIST_HEAD(&sys->head);
#endif
	}
}

static void mt_plls_init(void)
{
	int i;
	struct pll *pll;
	for (i = 0; i < NR_PLLS; i++) {
		pll = &plls[i];
		pll->state = pll->ops->get_state(pll);

#ifdef CONFIG_CLKMGR_STAT
		INIT_LIST_HEAD(&pll->head);
#endif
	}
}


#if !CLKMGR_8127


static void mt_plls_enable_hp(void)
{
	int i;
	struct pll *pll;
	for (i = 0; i < NR_PLLS; i++) {
		pll = &plls[i];
		if (pll->ops->hp_enable)
			pll->ops->hp_enable(pll);
	}
}


#endif /* CLKMGR_8127 */


static void mt_muxs_init(void)
{
	int i;
	struct clkmux *mux;

	clk_setl(CLK_CFG_2, 0x00008000);

	for (i = 0; i < NR_MUXS; i++) {
		mux = &muxs[i];
#ifdef CONFIG_CLKMGR_STAT
		INIT_LIST_HEAD(&mux->head);
#endif

	}
}


static void mt_clks_init(void)
{
#if !WORKAROUND_DAPC
	int i;
	struct cg_grp *grp;
	struct cg_clk *clk;

	for (i = 0; i < NR_CLKS; i++) {
		clk = id_to_clk(i);
		BUG_ON(!clk || !clk->ops);

		clk->state = clk->ops->get_state(clk);
		clk->grp->mask |= clk->mask;

#ifdef CONFIG_CLKMGR_STAT
		INIT_LIST_HEAD(&clk->head);
#endif
	}

	for (i = 0; i < NR_GRPS; i++) {
		grp = id_to_grp(i);
		grp->state = grp->ops->get_state(grp);
	}
#endif /* !WORKAROUND_DAPC */

	/* Don't disable these clock until it's clk_clr_force_on() is called */
	clk_set_force_on_locked(&clks[MT_CG_DISP0_SMI_LARB0]);
	clk_set_force_on_locked(&clks[MT_CG_DISP0_SMI_COMMON]);
}


#if CLKMGR_8127

static int id_from_mux(struct clkmux *mux)
{
	int i;
	for (i = 0; i < NR_MUXS; i++) {
		if (mux == &muxs[i])
			return i;
	}

	return -1;
}


static int id_from_pll(struct pll *p)
{
	int i;
	for (i = 0; i < NR_PLLS; i++) {
		if (p == &plls[i])
			return i;
	}

	return -1;
}


static void init_pll_by_mux(int muxid, int8_t *mux_init, int8_t *pll_init, int clkstate, int sysstate)
{
	struct clkmux *mux;

	if (muxid == -1)
		return;

	if (mux_init[muxid] < 0)
		mux_init[muxid] = 0;

	if (clkstate == PWR_ON || sysstate == PWR_ON)
		mux_init[muxid]++;

	mux = id_to_mux(muxid);

	if (sysstate == PWR_ON) {
		mux->cnt++;

		if (mux->pll)
			mux->pll->cnt++;
	}

	if (mux_init[muxid] > 1 && sysstate != PWR_ON)
		return;

	if (mux->pll) {
		int pllid = id_from_pll(mux->pll);
		if (pllid != -1) {
			if (pll_init[pllid] < 0)
				pll_init[pllid] = 0;

			if (clkstate == PWR_ON)
				pll_init[pllid]++;
		}
	}

	if (mux->parent) {
		int id = id_from_mux(mux->parent);
		init_pll_by_mux(id, mux_init, pll_init, clkstate, sysstate);
	}

	if (mux->siblings) {
		int id = id_from_mux(mux->siblings);
		init_pll_by_mux(id, mux_init, pll_init, clkstate, sysstate);
	}
}


static void init_mux_pll_by_clk(struct cg_clk *clk, int8_t *mux_init, int8_t *pll_init)
{
	if (!clk)
		return;

	if (clk->mux) {
		int id = id_from_mux(clk->mux);
		init_pll_by_mux(id, mux_init, pll_init, clk->state, PWR_DOWN);
	}

	if (clk->grp && clk->grp->sys && clk->grp->sys->mux) {
		int id = id_from_mux(clk->grp->sys->mux);
		init_pll_by_mux(id, mux_init, pll_init, clk->state, PWR_DOWN);
	}

	if (clk->parent)
		init_mux_pll_by_clk(clk->parent, mux_init, pll_init);
}


static void init_mux_pll_by_subsys(struct subsys *sys, int8_t *mux_init, int8_t *pll_init)
{
	if (!sys)
		return;

	if (sys->mux) {
		int id = id_from_mux(sys->mux);
		init_pll_by_mux(id, mux_init, pll_init, PWR_DOWN, sys->state);
	}
}


static void mt_clk_mux_pll_post_init(void)
{
	int i;

	int8_t mux_init[NR_MUXS];   /* -1: ignore ; 0: off ; 1+: on */
	int8_t pll_init[NR_PLLS];

	memset(mux_init, -1, sizeof(mux_init));
	memset(pll_init, -1, sizeof(pll_init));

	for (i = 0; i < NR_CLKS; i++) {
		struct cg_clk *clk = id_to_clk(i);
		init_mux_pll_by_clk(clk, mux_init, pll_init);
	}

	for (i = 0; i < NR_SYSS; i++) {
		struct subsys *sys = id_to_sys(i);
		init_mux_pll_by_subsys(sys, mux_init, pll_init);
	}

	for (i = 0; i < NR_MUXS; i++) {
		struct clkmux *mux = id_to_mux(i);

		if (mux_init[i] || mux->cnt > 0)
			mux->ops->enable(mux);
		else
			mux->ops->disable(mux);
	}

	for (i = 0; i < NR_PLLS; i++) {
		struct pll *p = id_to_pll(i);

		if (pll_init[i] || p->cnt > 0) {
			p->ops->enable(p);
			p->state = PWR_ON;
		} else {
			p->ops->disable(p);
			p->state = PWR_DOWN;
		}
	}
}


#endif /* CLKMGR_8127 */


int mt_clkmgr_bringup_init(void)
{
#if (!defined(CONFIG_CLKMGR_BRINGUP) && !defined(CONFIG_MTK_LDVT))
	BUG_ON(initialized);
#else
	if (initialized)
		return 0;
#endif

	spm_mtcmos_ctrl_vdec(STA_POWER_ON);
	spm_mtcmos_ctrl_isp(STA_POWER_ON);
	spm_mtcmos_ctrl_mfg(STA_POWER_ON);

	cg_all_force_on();
#if (!defined(CONFIG_CLKMGR_BRINGUP) && !defined(CONFIG_MTK_LDVT))
	cg_bootup_pdn();
#endif
#if (!defined(CONFIG_CLKMGR_BRINGUP))
	mt_plls_init();
	mt_subsys_init();
	mt_muxs_init();
	mt_clks_init();
#if CLKMGR_8127
	mt_clk_mux_pll_post_init();
#endif /* CLKMGR_8127 */
#endif
	initialized = 1;

	mt_freqhopping_init();
	mt_freqhopping_pll_init();
#if !defined(CONFIG_CLKMGR_BRINGUP) && !WORKAROUND_DAPC
	print_grp_regs();
#endif
	return 0;
}

void mt_clkmgr_init(void)
{
#if CLKMGR_8127

	mt_clkmgr_bringup_init();

#else /* !CLKMGR_8127 */

	unsigned long flags;
	BUG_ON(initialized);

	mt_plls_init();
	mt_subsys_init();
	mt_muxs_init();
	mt_clks_init();
	mt_clk_mux_pll_post_init();

	initialized = 1;

	mt_freqhopping_init();

	clkmgr_lock(flags);
	mt_freqhopping_pll_init();
	mt_plls_enable_hp();
	clkmgr_unlock(flags);

#endif /* CLKMGR_8127 */
}



#ifdef CONFIG_MTK_MMC
	/* msdc_clk_status() is declared in mt_clkmgr.h */
#else
void msdc_clk_status(int *status)
{
	*status = 0;
}
#endif

bool clkmgr_idle_can_enter(unsigned int *condition_mask, unsigned int *block_mask)
{
	int i, j;
	unsigned int cg_mask = 0;

#ifdef MTK_EMMC_SUPPORT
	unsigned int sd_mask = 0;
	msdc_clk_status(&sd_mask);
	if (sd_mask) {
		block_mask[CG_PERI0] |= sd_mask;
		return false;
	}
#endif

	for (i = CG_PERI0; i < NR_GRPS; i++) {
		cg_mask = grps[i].state & condition_mask[i];
		if (cg_mask) {
			for (j = CG_PERI0; j < NR_GRPS; j++)
				block_mask[j] = grps[j].state & condition_mask[j];

			return false;
		}
	}

	return true;
}


bool isp_vdec_on_off(void)
{
	unsigned int state;

	state = subsys_is_on(SYS_ISP);
	if (state == PWR_ON)
		return true;

	state = subsys_is_on(SYS_VDE);
	if (state == PWR_ON)
		return true;

	return false;
}
EXPORT_SYMBOL(isp_vdec_on_off);


/************************************************
 **********       function debug       **********
 ************************************************/

#if CLKMGR_8127


static void dump_pll_info1(struct seq_file *s, int clkidx)
{
	if (clkidx < NR_PLLS) {
		struct pll *pll = &plls[clkidx];
		unsigned int freq = pll_freq_calc_op(pll);

		seq_printf(s, "[%d] %7s: %4d.%03d MHz: %3s (%2d)\n",
			clkidx, pll->name, freq / 1000, freq % 1000, pll->state ? "ON" : "off", pll->cnt);
	}
}


#endif /* CLKMGR_8127 */


static int pll_test_show(struct seq_file *s, void *v)
{
	int i, j;
	int cnt;
	unsigned int value[3];
	const char *name;

#if CLKMGR_8127
	for (i = 0; i < NR_PLLS; i++)
		dump_pll_info1(s, i);
#endif /* CLKMGR_8127 */

	seq_printf(s, "********** pll register dump **********\n");
	for (i = 0; i < NR_PLLS; i++) {
		name = pll_get_name(i);
		cnt = pll_dump_regs(i, value);
		for (j = 0; j < cnt; j++)
			seq_printf(s, "[%d][%-7s reg%d]=[0x%08x]\n", i, name, j, value[j]);
	}
	seq_printf(s, "MIPI : CLK_DSI_PLL_CON0=0x%08x\n", clk_readl(CLK_DSI_PLL_CON0));

	seq_printf(s, "\n********** pll_test help **********\n");
	seq_printf(s, "enable  pll: echo enable  id [mod_name] > /proc/clkmgr/pll_test\n");
	seq_printf(s, "disable pll: echo disable id [mod_name] > /proc/clkmgr/pll_test\n");
#if CLKMGR_8127
	seq_printf(s, "set freq   : echo setfreq id freq_khz   > /proc/clkmgr/pll_test\n");
#endif /* CLKMGR_8127 */

	return 0;
}


static int pll_test_open(struct inode *inode, struct file *file)
{
	return single_open(file, pll_test_show, NULL);
}


static int pll_test_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;

	char cmd[10];
	char mod_name[10];
	int id;
	unsigned int freq;
	int err = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (sscanf(desc, "%s %d %u", cmd, &id, &freq) == 3) {
		if (!strcmp(cmd, "setfreq")) {
			unsigned int r = pll_set_freq(id, freq);
			if (r != 0)
				clk_info("pll_set_freq(): 0x%08x, id: %d, freq: %u\n", r, id, freq);
		}
	} else if (sscanf(desc, "%s %d %s", cmd, &id, mod_name) == 3) {
		if (!strcmp(cmd, "enable"))
			err = enable_pll(id, mod_name);
		else if (!strcmp(cmd, "disable"))
			err = disable_pll(id, mod_name);
	} else if (sscanf(desc, "%s %d", cmd, &id) == 2) {
		if (!strcmp(cmd, "enable"))
			err = enable_pll(id, "pll_test");
		else if (!strcmp(cmd, "disable"))
			err = disable_pll(id, "pll_test");
	}

	clk_info("[%s]%s pll %d: result is %d\n", __func__, cmd, id, err);

	return count;
}


static const struct file_operations pll_test_fops = {
	.owner   = THIS_MODULE,
	.write   = pll_test_write,
	.open    = pll_test_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};


static int pll_fsel_show(struct seq_file *s, void *v)
{
	int i;
	int cnt;
	unsigned int value[3];
	const char *name;

	for (i = 0; i < NR_PLLS; i++) {
		name = pll_get_name(i);
		if (pll_is_on(i)) {
			cnt = pll_dump_regs(i, value);
			if (cnt >= 2)
				seq_printf(s, "[%d][%-7s]=[0x%08x%08x]\n", i, name, value[0], value[1]);
			else
				seq_printf(s, "[%d][%-7s]=[0x%08x]\n", i, name, value[0]);
		} else
			seq_printf(s, "[%d][%-7s]=[-1]\n", i, name);
	}

	seq_printf(s, "\n********** pll_fsel help **********\n");
	seq_printf(s, "adjust pll frequency:  echo id freq > /proc/clkmgr/pll_fsel\n");

	return 0;
}


static int pll_fsel_open(struct inode *inode, struct file *file)
{
	return single_open(file, pll_fsel_show, NULL);
}


static int pll_fsel_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;

	int id;
	unsigned int value;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (sscanf(desc, "%d %x", &id, &value) == 2)
		pll_fsel(id, value);

	return count;
}


static const struct file_operations pll_fsel_fops = {
	.owner   = THIS_MODULE,
	.write   = pll_fsel_write,
	.open    = pll_fsel_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};


#ifdef CONFIG_CLKMGR_STAT


static int pll_stat_show(struct seq_file *s, void *v)
{
	struct pll *pll;
	struct list_head *pos;
	struct stat_node *node;
	int i;

	seq_printf(s, "\n********** pll stat dump **********\n");
	for (i = 0; i < NR_PLLS; i++) {
		pll = id_to_pll(i);
		seq_printf(s, "[%d][%-7s]state=%u, cnt=%u", i, pll->name,
				pll->state, pll->cnt);
		list_for_each(pos, &pll->head) {
			node = list_entry(pos, struct stat_node, link);
			seq_printf(s, "\t(%s,%u,%u)", node->name, node->cnt_on, node->cnt_off);
		}
		seq_printf(s, "\n");
	}

	seq_printf(s, "\n********** pll_dump help **********\n");

	return 0;
}


static int pll_stat_open(struct inode *inode, struct file *file)
{
	return single_open(file, pll_stat_show, NULL);
}


static const struct file_operations pll_stat_fops = {
	.owner   = THIS_MODULE,
	.open    = pll_stat_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};


#endif /* CONFIG_CLKMGR_STAT */


static int subsys_test_show(struct seq_file *s, void *v)
{
	int i;
	int state;
	unsigned int value = 0, sta, sta_s;
	const char *name;

	sta = clk_readl(SPM_PWR_STATUS);
	sta_s = clk_readl(SPM_PWR_STATUS_S);

	seq_printf(s, "********** subsys register dump **********\n");
	for (i = 0; i < NR_SYSS; i++) {
		name = subsys_get_name(i);
		state = subsys_is_on(i);
		subsys_dump_regs(i, &value);
		seq_printf(s, "[%d][%-8s]=[0x%08x], state(%u)\n", i, name, value, state);
	}
	seq_printf(s, "SPM_PWR_STATUS=0x%08x, SPM_PWR_STATUS_S=0x%08x\n", sta, sta_s);

	seq_printf(s, "\n********** subsys_test help **********\n");
	seq_printf(s, "enable subsys:  echo enable id > /proc/clkmgr/subsys_test\n");
	seq_printf(s, "disable subsys: echo disable id [force_off] > /proc/clkmgr/subsys_test\n");

	return 0;
}


static int subsys_test_open(struct inode *inode, struct file *file)
{
    return single_open(file, subsys_test_show, NULL);
}


static int subsys_test_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;

	char cmd[10];
	int id;
	int force_off;
	int err = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (sscanf(desc, "%s %d %d", cmd, &id, &force_off) == 3) {
		if (!strcmp(cmd, "disable"))
			err = disable_subsys_force(id, "test");
	} else if (sscanf(desc, "%s %d", cmd, &id) == 2) {
		if (!strcmp(cmd, "enable"))
			err = enable_subsys(id, "test");
		else if (!strcmp(cmd, "disable"))
			err = disable_subsys(id, "test");
	}

	clk_info("[%s]%s subsys %d: result is %d\n", __func__, cmd, id, err);

	return count;
}


static const struct file_operations subsys_test_fops = {
	.owner   = THIS_MODULE,
	.write   = subsys_test_write,
	.open    = subsys_test_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};


#ifdef CONFIG_CLKMGR_STAT


static int subsys_stat_show(struct seq_file *s, void *v)
{
	struct subsys *sys;
	struct list_head *pos;
	struct stat_node *node;
	int i;

	seq_printf(s, "\n********** subsys stat dump **********\n");
	for (i = 0; i < NR_SYSS; i++) {
		sys = id_to_sys(i);
		seq_printf(s, "[%d][%-7s]state=%u", i, sys->name, sys->state);
		list_for_each(pos, &sys->head) {
			node = list_entry(pos, struct stat_node, link);
			seq_printf(s, "\t(%s,%u,%u)", node->name, node->cnt_on, node->cnt_off);
		}
		seq_printf(s, "\n");
	}

	seq_printf(s, "\n********** subsys_dump help **********\n");

	return 0;
}


static int subsys_stat_open(struct inode *inode, struct file *file)
{
	return single_open(file, subsys_stat_show, NULL);
}


static const struct file_operations subsys_stat_fops = {
	.owner   = THIS_MODULE,
	.open    = subsys_stat_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};


#endif


#if CLKMGR_8127


#define MAX_MUX_CLKSRC      12


enum FMETER_TYPE {
	ABIST,
	CKGEN
};


static const char * const ABIST_CLK_NAME[] = {
	[ABIST_AD_MAIN_H546M_CK]            = "AD_MAIN_H546M_CK",
	[ABIST_AD_MAIN_H364M_CK]            = "AD_MAIN_H364M_CK",
	[ABIST_AD_MAIN_H218P4M_CK]          = "AD_MAIN_H218P4M_CK",
	[ABIST_AD_MAIN_H156M_CK]            = "AD_MAIN_H156M_CK",
	[ABIST_AD_UNIV_624M_CK]             = "AD_UNIV_624M_CK",
	[ABIST_AD_UNIV_416M_CK]             = "AD_UNIV_416M_CK",
	[ABIST_AD_UNIV_249P6M_CK]           = "AD_UNIV_249P6M_CK",
	[ABIST_AD_UNIV_178P3M_CK]           = "AD_UNIV_178P3M_CK",
	[ABIST_AD_UNIV_48M_CK]              = "AD_UNIV_48M_CK",
	[ABIST_AD_USB_48M_CK]               = "AD_USB_48M_CK",
	[ABIST_AD_MMPLL_CK]                 = "AD_MMPLL_CK",
	[ABIST_AD_MSDCPLL_CK]               = "AD_MSDCPLL_CK",
	[ABIST_AD_DPICLK]                   = "AD_DPICLK",
	[ABIST_CLKPH_MCK_O]                 = "clkph_mck_o",
	[ABIST_AD_MEMPLL2_CKOUT0_PRE_ISO]   = "AD_MEMPLL2_CKOUT0_PRE_ISO",
	[ABIST_AD_MCUPLL1_H481M_CK]         = "AD_MCUPLL1_H481M_CK",
	[ABIST_AD_MDPLL1_416M_CK]           = "AD_MDPLL1_416M_CK",
	[ABIST_AD_WPLL_CK]                  = "AD_WPLL_CK",
	[ABIST_AD_WHPLL_CK]                 = "AD_WHPLL_CK",
	[ABIST_RTC32K_CK_I]                 = "rtc32k_ck_i",
	[ABIST_AD_SYS_26M_CK]               = "AD_SYS_26M_CK",
	[ABIST_AD_VENCPLL_CK]               = "AD_VENCPLL_CK",
	[ABIST_AD_MIPI_26M_CK]              = "AD_MIPI_26M_CK",
	[ABIST_AD_MEM_26M_CK]               = "AD_MEM_26M_CK",
	[ABIST_AD_PLLGP_TST_CK]             = "AD_PLLGP_TST_CK",
	[ABIST_AD_DSI0_LNTC_DSICLK]         = "AD_DSI0_LNTC_DSICLK",
	[ABIST_AD_MPPLL_TST_CK]             = "AD_MPPLL_TST_CK",
	[ABIST_ARMPLL_OCC_MON]              = "armpll_occ_mon",
	[ABIST_AD_MEM2MIPI_26M_CK]          = "AD_MEM2MIPI_26M_CK",
	[ABIST_AD_MEMPLL_MONCLK]            = "AD_MEMPLL_MONCLK",
	[ABIST_AD_MEMPLL2_MONCLK]           = "AD_MEMPLL2_MONCLK",
	[ABIST_AD_MEMPLL3_MONCLK]           = "AD_MEMPLL3_MONCLK",
	[ABIST_AD_MEMPLL4_MONCLK]           = "AD_MEMPLL4_MONCLK",
	[ABIST_AD_MEMPLL_REFCLK]            = "AD_MEMPLL_REFCLK",
	[ABIST_AD_MEMPLL_FBCLK]             = "AD_MEMPLL_FBCLK",
	[ABIST_AD_MEMPLL2_REFCLK]           = "AD_MEMPLL2_REFCLK",
	[ABIST_AD_MEMPLL2_FBCLK]            = "AD_MEMPLL2_FBCLK",
	[ABIST_AD_MEMPLL3_REFCLK]           = "AD_MEMPLL3_REFCLK",
	[ABIST_AD_MEMPLL3_FBCLK]            = "AD_MEMPLL3_FBCLK",
	[ABIST_AD_MEMPLL4_REFCLK]           = "AD_MEMPLL4_REFCLK",
	[ABIST_AD_MEMPLL4_FBCLK]            = "AD_MEMPLL4_FBCLK",
	[ABIST_AD_MEMPLL_TSTDIV2_CK]        = "AD_MEMPLL_TSTDIV2_CK",
	[ABIST_AD_LVDSPLL_CK]               = "AD_LVDSPLL_CK",
	[ABIST_AD_LVDSTX_MONCLK]            = "AD_LVDSTX_MONCLK",
	[ABIST_AD_HDMITX_MONCLK]            = "AD_HDMITX_MONCLK",
	[ABIST_AD_USB20_C240M]              = "AD_USB20_C240M",
	[ABIST_AD_USB20_C240M_1P]           = "AD_USB20_C240M_1P",
	[ABIST_AD_MONREF_CK]                = "AD_MONREF_CK",
	[ABIST_AD_MONFBK_CK]                = "AD_MONFBK_CK",
	[ABIST_AD_TVDPLL_CK]                = "AD_TVDPLL_CK",
	[ABIST_AD_AUDPLL_CK]                = "AD_AUDPLL_CK",
	[ABIST_AD_LVDSPLL_ETH_CK]           = "AD_LVDSPLL_ETH_CK",
};


static const char * const CKGEN_CLK_NAME[] = {
	[CKGEN_HF_FAXI_CK]          = "hf_faxi_ck",
	[CKGEN_HD_FAXI_CK]          = "hd_faxi_ck",
	[CKGEN_HF_FNFI2X_CK]        = "hf_fnfi2x_ck",
	[CKGEN_HF_FDDRPHYCFG_CK]    = "hf_fddrphycfg_ck",
	[CKGEN_HF_FMM_CK]           = "hf_fmm_ck",
	[CKGEN_F_FPWM_CK]           = "f_fpwm_ck",
	[CKGEN_HF_FVDEC_CK]         = "hf_fvdec_ck",
	[CKGEN_HF_FMFG_CK]          = "hf_fmfg_ck",
	[CKGEN_HF_FCAMTG_CK]        = "hf_fcamtg_ck",
	[CKGEN_F_FUART_CK]          = "f_fuart_ck",
	[CKGEN_HF_FSPI_CK]          = "hf_fspi_ck",
	[CKGEN_F_FUSB20_CK]         = "f_fusb20_ck",
	[CKGEN_HF_FMSDC30_0_CK]     = "hf_fmsdc30_0_ck",
	[CKGEN_HF_FMSDC30_1_CK]     = "hf_fmsdc30_1_ck",
	[CKGEN_HF_FMSDC30_2_CK]     = "hf_fmsdc30_2_ck",
	[CKGEN_HF_FAUDIO_CK]        = "hf_faudio_ck",
	[CKGEN_HF_FAUD_INTBUS_CK]   = "hf_faud_intbus_ck",
	[CKGEN_HF_FPMICSPI_CK]      = "hf_fpmicspi_ck",
	[CKGEN_F_FRTC_CK]           = "f_frtc_ck",
	[CKGEN_F_F26M_CK]           = "f_f26m_ck",
	[CKGEN_F_F32K_MD1_CK]       = "f_f32k_md1_ck",
	[CKGEN_F_FRTC_CONN_CK]      = "f_frtc_conn_ck",
	[CKGEN_HF_FETH_50M_CK]      = "hf_feth_50m_ck",
	[CKGEN_HD_HAXI_NLI_CK]      = "hd_haxi_nli_ck",
	[CKGEN_HD_QAXIDCM_CK]       = "hd_qaxidcm_ck",
	[CKGEN_F_FFPC_CK]           = "f_ffpc_ck",
	[CKGEN_HF_FDPI0_CK]         = "hf_fdpi0_ck",
	[CKGEN_F_FCKBUS_CK_SCAN]    = "f_fckbus_ck_scan",
	[CKGEN_F_FCKRTC_CK_SCAN]    = "f_fckrtc_ck_scan",
	[CKGEN_HF_FDPILVDS_CK]      = "hf_fdpilvds_ck",
};


enum mux_clksrc_id {
	mux_clksrc_null,

	external_32k,
	internal_32k,
	audpll,
	audpll_d16,
	audpll_d24,
	audpll_d4,
	audpll_d8,
	clk26m,
	dmpll_ck,
	dmpll_d2,
	dmpll_d4,
	dmpll_x2_ck,
	f_f26m_ck,
	fpc_ck,
	hdmipll,
	hdmipll_d2,
	hdmipll_d3,
	lvdspll,
	lvdspll_d2,
	lvdspll_d4,
	lvdspll_d8,
	lvdspll_eth,
	mipipll,
	mipipll_d2,
	mipipll_d4,
	mmpll_ck,
	mmpll_d2,
	msdcpll_ck,
	msdcpll_d2,
	syspll1_d16,
	syspll1_d2,
	syspll1_d4,
	syspll1_d8,
	syspll2_d2,
	syspll2_d4,
	syspll2_d8,
	syspll3_d2,
	syspll3_d4,
	syspll4_d2,
	syspll4_d4,
	syspll_d3,
	syspll_d5,
	syspll_d7,
	tvdpll,
	tvdpll_d2,
	tvdpll_d4,
	univpll1_d2,
	univpll1_d4,
	univpll1_d8,
	univpll2_d2,
	univpll2_d4,
	univpll2_d8,
	univpll3_d2,
	univpll3_d4,
	univpll3_d8,
	univpll_d26,
	univpll_d3,
	univpll_d5,
	vencpll_ck,

	mux_clksrc_end
};


struct mux_t {
	enum CKGEN_CLK ckgen_clk;
	enum mux_clksrc_id clksrc[MAX_MUX_CLKSRC];
};


static const char * const clksrc_name[] = {
	[external_32k] = "32k_external",
	[internal_32k] = "32k_internal",
	[audpll]       = "audpll",
	[audpll_d16]   = "audpll_d16",
	[audpll_d24]   = "audpll_d24",
	[audpll_d4]    = "audpll_d4",
	[audpll_d8]    = "audpll_d8",
	[clk26m]       = "clk26m",
	[dmpll_ck]     = "dmpll_ck",
	[dmpll_d2]     = "dmpll_d2",
	[dmpll_d4]     = "dmpll_d4",
	[dmpll_x2_ck]  = "dmpll_x2_ck",
	[f_f26m_ck]    = "f_f26m_ck",
	[fpc_ck]       = "fpc_ck",
	[hdmipll]      = "hdmipll",
	[hdmipll_d2]   = "hdmipll_d2",
	[hdmipll_d3]   = "hdmipll_d3",
	[lvdspll]      = "lvdspll",
	[lvdspll_d2]   = "lvdspll_d2",
	[lvdspll_d4]   = "lvdspll_d4",
	[lvdspll_d8]   = "lvdspll_d8",
	[lvdspll_eth]  = "lvdspll_eth",
	[mipipll]      = "mipipll",
	[mipipll_d2]   = "mipipll_d2",
	[mipipll_d4]   = "mipipll_d4",
	[mmpll_ck]     = "mmpll_ck",
	[mmpll_d2]     = "mmpll_d2",
	[msdcpll_ck]   = "msdcpll_ck",
	[msdcpll_d2]   = "msdcpll_d2",
	[syspll1_d16]  = "syspll1_d16",
	[syspll1_d2]   = "syspll1_d2",
	[syspll1_d4]   = "syspll1_d4",
	[syspll1_d8]   = "syspll1_d8",
	[syspll2_d2]   = "syspll2_d2",
	[syspll2_d4]   = "syspll2_d4",
	[syspll2_d8]   = "syspll2_d8",
	[syspll3_d2]   = "syspll3_d2",
	[syspll3_d4]   = "syspll3_d4",
	[syspll4_d2]   = "syspll4_d2",
	[syspll4_d4]   = "syspll4_d4",
	[syspll_d3]    = "syspll_d3",
	[syspll_d5]    = "syspll_d5",
	[syspll_d7]    = "syspll_d7",
	[tvdpll]       = "tvdpll",
	[tvdpll_d2]    = "tvdpll_d2",
	[tvdpll_d4]    = "tvdpll_d4",
	[univpll1_d2]  = "univpll1_d2",
	[univpll1_d4]  = "univpll1_d4",
	[univpll1_d8]  = "univpll1_d8",
	[univpll2_d2]  = "univpll2_d2",
	[univpll2_d4]  = "univpll2_d4",
	[univpll2_d8]  = "univpll2_d8",
	[univpll3_d2]  = "univpll3_d2",
	[univpll3_d4]  = "univpll3_d4",
	[univpll3_d8]  = "univpll3_d8",
	[univpll_d26]  = "univpll_d26",
	[univpll_d3]   = "univpll_d3",
	[univpll_d5]   = "univpll_d5",
	[vencpll_ck]   = "vencpll_ck",
};


static struct mux_t g_mux[] = {
	[MT_MUX_MM]        = {.ckgen_clk = CKGEN_HF_FMM_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  vencpll_ck,
										[2] =  syspll1_d2,
										[3] =  syspll1_d4,
										[4] =  univpll_d5,
										[5] =  univpll1_d2,
										[6] =  univpll2_d2,
										[7] =  dmpll_ck
									},
						 },
	[MT_MUX_DDRPHYCFG] = {.ckgen_clk = CKGEN_HF_FDDRPHYCFG_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  syspll1_d8,
									},
						 },
	[MT_MUX_MEM]       = {
						  .clksrc = {
										[0] =  clk26m,
										[1] =  dmpll_ck,
									},
						 },
	[MT_MUX_AXI]       = {.ckgen_clk = CKGEN_HF_FAXI_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  syspll1_d2,
										[2] =  syspll_d5,
										[3] =  syspll1_d4,
										[4] =  univpll_d5,
										[5] =  univpll2_d2,
										[6] =  dmpll_ck,
										[7] =  dmpll_d2,
									},
						 },
	[MT_MUX_CAMTG]     = {.ckgen_clk = CKGEN_HF_FCAMTG_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  univpll_d26,
										[2] =  univpll2_d2,
										[3] =  syspll3_d2,
										[4] =  syspll3_d4,
										[5] =  msdcpll_d2,
										[6] =  mmpll_d2,
									},
						 },
	[MT_MUX_MFG]       = {.ckgen_clk = CKGEN_HF_FMFG_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  mmpll_ck,
										[2] =  dmpll_x2_ck,
										[3] =  msdcpll_ck,
										[4] =  clk26m,
										[5] =  syspll_d3,
										[6] =  univpll_d3,
										[7] =  univpll1_d2,
									},
						 },
	[MT_MUX_VDEC]      = {.ckgen_clk = CKGEN_HF_FVDEC_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  syspll1_d2,
										[2] =  syspll_d5,
										[3] =  syspll1_d4,
										[4] =  univpll_d5,
										[5] =  univpll2_d2,
										[6] =  univpll2_d4,
										[7] =  msdcpll_d2,
										[8] =  mmpll_d2,
									},
						 },
	[MT_MUX_PWM]       = {.ckgen_clk = CKGEN_F_FPWM_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  univpll2_d4,
										[2] =  univpll3_d2,
										[3] =  univpll1_d4,
									},
						 },
	[MT_MUX_MSDC30_0]  = {.ckgen_clk = CKGEN_HF_FMSDC30_0_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  msdcpll_d2,
										[2] =  syspll2_d2,
										[3] =  syspll1_d4,
										[4] =  univpll1_d4,
										[5] =  univpll2_d4,
									},
						 },
	[MT_MUX_USB20]     = {.ckgen_clk = CKGEN_F_FUSB20_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  univpll1_d8,
										[2] =  univpll3_d4,
									},
						 },
	[MT_MUX_SPI]       = {.ckgen_clk = CKGEN_HF_FSPI_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  syspll3_d2,
										[2] =  syspll4_d2,
										[3] =  univpll2_d4,
										[4] =  univpll1_d8,
									},
						 },
	[MT_MUX_UART]      = {.ckgen_clk = CKGEN_F_FUART_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  univpll2_d8,
									},
						 },
	[MT_MUX_AUDINTBUS] = {.ckgen_clk = CKGEN_HF_FAUD_INTBUS_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  syspll1_d4,
										[2] =  syspll3_d2,
										[3] =  syspll4_d2,
										[4] =  univpll3_d2,
										[5] =  univpll2_d4,
									},
						 },
	[MT_MUX_AUDIO]     = {.ckgen_clk = CKGEN_HF_FAUDIO_CK,
						  .clksrc = {
										[0] =  f_f26m_ck,
										[1] =  syspll1_d16,
									},
						 },
	[MT_MUX_MSDC30_2]  = {.ckgen_clk = CKGEN_HF_FMSDC30_2_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  msdcpll_d2,
										[2] =  syspll2_d2,
										[3] =  syspll1_d4,
										[4] =  univpll1_d4,
										[5] =  univpll2_d4,
									},
						 },
	[MT_MUX_MSDC30_1]  = {.ckgen_clk = CKGEN_HF_FMSDC30_1_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  msdcpll_d2,
										[2] =  syspll2_d2,
										[3] =  syspll1_d4,
										[4] =  univpll1_d4,
										[5] =  univpll2_d4,
									},
						 },
	[MT_MUX_DPI1]      = {
						  .clksrc = {
										[0] =  clk26m,
										[1] =  tvdpll,
										[2] =  tvdpll_d2,
										[3] =  tvdpll_d4,
									},
						 },
	[MT_MUX_DPI0]      = {.ckgen_clk = CKGEN_HF_FDPI0_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  mipipll,
										[2] =  mipipll_d2,
										[3] =  mipipll_d4,
										[4] =  lvdspll,
										[5] =  lvdspll_d2,
										[6] =  lvdspll_d4,
										[7] =  lvdspll_d8,
									},
						 },
	[MT_MUX_SCP]       = {
						  .clksrc = {
										[0] =  clk26m,
										[1] =  syspll1_d8,
										[2] =  dmpll_d2,
										[3] =  dmpll_d4,
									},
						 },
	[MT_MUX_PMICSPI]   = {.ckgen_clk = CKGEN_HF_FPMICSPI_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  syspll1_d8,
										[2] =  syspll2_d4,
										[3] =  syspll4_d2,
										[4] =  syspll3_d4,
										[5] =  syspll2_d8,
										[6] =  syspll1_d16,
										[7] =  univpll3_d4,
										[8] =  univpll_d26,
										[9] =  dmpll_d2,
										[10] = dmpll_d4,
									},
						 },
	[MT_MUX_DPILVDS]   = {.ckgen_clk = CKGEN_HF_FDPILVDS_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  lvdspll,
										[2] =  lvdspll_d2,
										[3] =  lvdspll_d4,
										[4] =  lvdspll_d8,
										[5] =  fpc_ck,
										[6] =  clk26m,
										[7] =  clk26m,
									},
						 },
	[MT_MUX_APLL]      = {
						  .clksrc = {
										[0] =  clk26m,
										[1] =  audpll,
										[2] =  audpll_d4,
										[3] =  audpll_d8,
										[4] =  audpll_d16,
										[5] =  audpll_d24,
										[6] =  clk26m,
										[7] =  clk26m,
									},
						 },
	[MT_MUX_HDMI]      = {
						  .clksrc = {
										[0] =  clk26m,
										[1] =  hdmipll,
										[2] =  hdmipll_d2,
										[3] =  hdmipll_d3,
									},
						 },
	[MT_MUX_TVE]       = {
						  .clksrc = {
										[0] =  clk26m,
										[1] =  mipipll,
										[2] =  mipipll_d2,
										[3] =  mipipll_d4,
										[4] =  clk26m,
										[5] =  tvdpll,
										[6] =  tvdpll_d2,
										[7] =  tvdpll_d4,
									},
						 },
	[MT_MUX_ETH_50M]   = {.ckgen_clk = CKGEN_HF_FETH_50M_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  syspll3_d4,
										[2] =  univpll2_d8,
										[3] =  lvdspll_eth,
										[4] =  univpll_d26,
										[5] =  syspll2_d8,
										[6] =  syspll4_d4,
										[7] =  univpll3_d8,
									},
						 },
	[MT_MUX_NFI2X]     = {.ckgen_clk = CKGEN_HF_FNFI2X_CK,
						  .clksrc = {
										[0] =  clk26m,
										[1] =  syspll2_d2,
										[2] =  syspll_d7,
										[3] =  univpll3_d2,
										[4] =  syspll2_d4,
										[5] =  univpll3_d4,
										[6] =  syspll4_d4,
										[7] =  clk26m,
									},
						 },
	[MT_MUX_RTC]       = {.ckgen_clk = CKGEN_F_FRTC_CK,
						  .clksrc = {
										[0] =  internal_32k,
										[1] =  external_32k,
										[2] =  clk26m,
										[3] =  univpll3_d8,
									},
						 },
};


static void seq_printf_mux(struct seq_file *s, int mux_id)
{
	int reg_value;
	int mux_value;
	int mux_pdn;
	const char *mux_en;
	const char *mux_clksrc;

	struct clkmux *mux = id_to_mux(mux_id);
	if (!mux)
		return;

	reg_value = clk_readl(mux->base_addr);
	mux_value = (reg_value & mux->sel_mask) >> mux->offset;
	mux_pdn = reg_value & mux->pdn_mask;
	mux_en = mux_pdn ? "off" : "ON";
	mux_clksrc = clksrc_name[g_mux[mux_id].clksrc[mux_value]];

	seq_printf(s, "[ %2d, %2d, %3s, %-16s, %2d: %-12s ]\n",
		mux_id, mux->cnt, mux_en, mux->name, mux_value, mux_clksrc);
}


#endif /* CLKMGR_8127 */


static int mux_test_show(struct seq_file *s, void *v)
{
#if CLKMGR_8127
	int i;
	for (i = 0; i < NR_MUXS; i++)
		seq_printf_mux(s, i);
#endif /* CLKMGR_8127 */

	seq_printf(s, "********** mux register dump *********\n");
	seq_printf(s, "[CLK_CFG_0] =0x%08x\n", clk_readl(CLK_CFG_0));
	seq_printf(s, "[CLK_CFG_1] =0x%08x\n", clk_readl(CLK_CFG_1));
	seq_printf(s, "[CLK_CFG_2] =0x%08x\n", clk_readl(CLK_CFG_2));
	seq_printf(s, "[CLK_CFG_3] =0x%08x\n", clk_readl(CLK_CFG_3));
	seq_printf(s, "[CLK_CFG_4] =0x%08x\n", clk_readl(CLK_CFG_4));
	seq_printf(s, "[CLK_CFG_5] =0x%08x\n", clk_readl(CLK_CFG_5));
	seq_printf(s, "[CLK_CFG_6] =0x%08x\n", clk_readl(CLK_CFG_6));
	seq_printf(s, "[CLK_CFG_8] =0x%08x\n", clk_readl(CLK_CFG_8));
	seq_printf(s, "[CLK_CFG_9] =0x%08x\n", clk_readl(CLK_CFG_9));
	seq_printf(s, "[CLK_CFG_10]=0x%08x\n", clk_readl(CLK_CFG_10));
	seq_printf(s, "[CLK_CFG_11]=0x%08x\n", clk_readl(CLK_CFG_11));

	seq_printf(s, "\n********** mux_test help *********\n");
	seq_printf(s, "clkmux     : echo clkmux  mux_id val [mod_name] > /proc/clkmgr/mux_test\n");
	seq_printf(s, "enable  mux: echo enable  mux_id     [mod_name] > /proc/clkmgr/mux_test\n");
	seq_printf(s, "disable mux: echo disable mux_id     [mod_name] > /proc/clkmgr/mux_test\n");

	return 0;
}


static int mux_test_open(struct inode *inode, struct file *file)
{
	return single_open(file, mux_test_show, NULL);
}


#if CLKMGR_8127


static int mux_test_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;

	char cmd[10];
	char mod_name[10];
	int mux_id;
	int val;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);

	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (sscanf(desc, "%s %d %d %s", cmd, &mux_id, &val, mod_name) == 4) {
		if (!strcmp(cmd, "clkmux"))
			clkmux_sel(mux_id, val, mod_name);
	} else if (sscanf(desc, "%s %d %s", cmd, &mux_id, mod_name) == 3) {
		if (!strcmp(cmd, "enable"))
			enable_mux(mux_id, mod_name);
		else if (!strcmp(cmd, "disable"))
			disable_mux(mux_id, mod_name);
	} else if (sscanf(desc, "%s %d", cmd, &mux_id) == 2) {
		if (!strcmp(cmd, "enable"))
			enable_mux(mux_id, "mux_test");
		else if (!strcmp(cmd, "disable"))
			disable_mux(mux_id, "mux_test");
	}

	return count;
}


#endif /* CLKMGR_8127 */


static const struct file_operations mux_test_fops = {
	.owner   = THIS_MODULE,
	.write   = mux_test_write,
	.open    = mux_test_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};


#ifdef CONFIG_CLKMGR_STAT


static int mux_stat_show(struct seq_file *s, void *v)
{
	struct clkmux *mux;
	struct list_head *pos;
	struct stat_node *node;
	int i;

	seq_printf(s, "********** mux stat dump **********\n");
	for (i = 0; i < NR_MUXS; i++) {
		mux = id_to_mux(i);

		seq_printf(s, "[%02d][%-14s]cnt=%u", i, mux->name, mux->cnt);

		list_for_each(pos, &mux->head) {
			node = list_entry(pos, struct stat_node, link);
			seq_printf(s, "\t(%s,%u,%u)", node->name, node->cnt_on, node->cnt_off);
		}
		seq_printf(s, "\n");
	}

	seq_printf(s, "\n********** mux_dump help **********\n");

	return 0;
}


static int mux_stat_open(struct inode *inode, struct file *file)
{
	return single_open(file, mux_stat_show, NULL);
}


static const struct file_operations mux_stat_fops = {
	.owner   = THIS_MODULE,
	.open    = mux_stat_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};


#endif


static int clk_test_read(char *page, char **start, off_t off,
				int count, int *eof, void *data)
{
	char *p = page;
	int len = 0;

	int i;
	int cnt;
	unsigned int value[2];
	const char *name;

	p += sprintf(p, "********** clk register dump **********\n");

	for (i = 0; i < NR_GRPS; i++) {
		name = grp_get_name(i);
		cnt = grp_dump_regs(i, value);
		if (cnt == 1)
			p += sprintf(p, "[%02d][%-8s]=[0x%08x]\n", i, name, value[0]);
		else
			p += sprintf(p, "[%02d][%-8s]=[0x%08x][0x%08x]\n", i, name, value[0], value[1]);
	}

	p += sprintf(p, "\n********** clk_test help **********\n");
	p += sprintf(p, "enable  clk: echo enable  id [mod_name] > /proc/clkmgr/clk_test\n");
	p += sprintf(p, "disable clk: echo disable id [mod_name] > /proc/clkmgr/clk_test\n");
	p += sprintf(p, "read state:  echo id > /proc/clkmgr/clk_test\n");

	*start = page + off;

	len = p - page;
	if (len > off)
		len -= off;
	else
		len = 0;

	*eof = 1;
	return len < count ? len  : count;
}

static int clk_test_write(struct file *file, const char *buffer,
						  size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;

	char cmd[10];
	char mod_name[10];
	int id;
	int err;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (sscanf(desc, "%s %d %s", cmd, &id, mod_name) == 3) {
		if (!strcmp(cmd, "enable"))
			err = enable_clock(id, mod_name);
		else if (!strcmp(cmd, "disable"))
			err = disable_clock(id, mod_name);
	} else if (sscanf(desc, "%s %d", cmd, &id) == 2) {
		if (!strcmp(cmd, "enable"))
			err = enable_clock(id, "pll_test");
		else if (!strcmp(cmd, "disable"))
			err = disable_clock(id, "pll_test");
	} else if (sscanf(desc, "%d", &id) == 1)
		clk_info("clock %d is %s\n", id, clock_is_on(id) ? "on" : "off");

	return count;
}


#if CLKMGR_8127

static void *clk_test_seq_start(struct seq_file *s, loff_t *pos)
{
	loff_t *spos;

	if (*pos >= NR_CLKS + 1)
		return NULL;

	spos = kmalloc(sizeof(loff_t), GFP_KERNEL);
	if (!spos)
		return NULL;

	*spos = *pos;
	return spos;
}


static void *clk_test_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	loff_t *spos = (loff_t *)v;
	loff_t i = *spos;

	if (i == NR_CLKS)
		++i;
	else {
		while (i < NR_CLKS && clks[++i].name == NULL)
			;
	}

	*pos = i;
	*spos = i;

	if (*pos >= NR_CLKS + 1)
		return NULL;

	return spos;
}


static void clk_test_seq_stop(struct seq_file *s, void *v)
{
	kfree(v);
}


static int clk_test_seq_show(struct seq_file *s, void *v)
{
	loff_t *spos = (loff_t *)v;
	loff_t i = *spos;
	struct cg_clk *clk = &clks[i];

	if (i < NR_CLKS) {
		seq_printf(s, "[ %3lld,%3d,%4s, 0x%08X, %-29s, %-29s, %-8s, %-16s ]\n",
				   i,
				   clk->cnt,
				   clk->state ? "ON" : "off",
				   clk->mask,
				   clk->name ? clk->name : "",
				   clk->parent ? clk->parent->name : "",
				   clk->grp ? clk->grp->name : "",
				   clk->mux ? clk->mux->name : "");
	} else {
		const int PS = 1280;
		char page[PS];
		char *start = NULL;
		int eof = 0;
		clk_test_read(page, &start, 0, PS, &eof, NULL);
		seq_puts(s, page);
	}

	return 0;
}


static const struct seq_operations clk_test_seq_ops = {
	.start = clk_test_seq_start,
	.next  = clk_test_seq_next,
	.stop  = clk_test_seq_stop,
	.show  = clk_test_seq_show
};


static int clk_test_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &clk_test_seq_ops);
};


static const struct file_operations clk_test_fops = {
	.owner   = THIS_MODULE,
	.write   = clk_test_write,
	.open    = clk_test_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};


#endif /* CLKMGR_8127 */


#if CLKMGR_8127


static void set_fmeter_divider(uint32_t k1)
{
	uint32_t v = k1 << 24 | 0xFF << 16 | k1 << 8 | k1;
	clk_writel(CLK_MISC_CFG_1, v);
}


static uint32_t fmeter_freq(enum FMETER_TYPE type, int k1, int clk)
{
	int i;
	uint32_t freq = 0;

	uint32_t clk_cfg_reg    = (type == CKGEN) ? CLK_CFG_9            : CLK_CFG_8;
	uint32_t cksw_mask      = (type == CKGEN) ? BITMASK(20 : 16)     : BITMASK(13 : 8);
	uint32_t cksw_val       = (type == CKGEN) ? BITS(20 : 16, clk)   : BITS(13 : 8, clk);
	uint32_t tri_bit        = (type == CKGEN) ? BIT(4)               : BIT(0);
	uint32_t cnt_reg        = (type == CKGEN) ? CLK26CALI_2          : CLK26CALI_1;
	uint32_t clk_misc_cfg_1;
	uint32_t clk_cfg_val;

	/* setup fmeter */
	clk_setl(CLK26CALI_0, BIT(7));                          /* enable fmeter_en */

	clk_misc_cfg_1 = clk_readl(CLK_MISC_CFG_1);             /* backup CLK_MISC_CFG_1 value */
	set_fmeter_divider(k1);                                 /* set divider (0 = /1) */

	clk_cfg_val = clk_readl(clk_cfg_reg);                   /* backup clk_cfg_reg value */
	clk_writel_mask(clk_cfg_reg, cksw_mask, cksw_val);      /* select cksw */

	clk_setl(CLK26CALI_0, tri_bit);                         /* start fmeter */

	/* wait fmeter */
	for (i = 100; i > 0 && clk_readl(CLK26CALI_0) & tri_bit; i--)
		udelay(50);

	if (clk_readl(CLK26CALI_0) & tri_bit) {
		/* failed */
	} else {
		uint32_t cnt = clk_readl(cnt_reg) & 0xFFFF;
		freq = (cnt * 26000) * (k1 + 1) / 1024;        /* (KHz) ; freq = counter * 26M / 1024 */
	}

	/* restore register settings */
	clk_writel(clk_cfg_reg, clk_cfg_val);
	clk_writel(CLK_MISC_CFG_1, clk_misc_cfg_1);

	return freq;
}


static uint32_t measure_stable_fmeter_freq(enum FMETER_TYPE type, int k1, int clk)
{
	const int diff_threshold = 10;
	uint32_t last_freq = 0;
	uint32_t freq = fmeter_freq(type, k1, clk);
	uint32_t maxfreq = max(freq, last_freq);

	while (maxfreq > 0 && ABS_DIFF(freq, last_freq) * 100 / maxfreq > diff_threshold) {
		last_freq = freq;
		freq = fmeter_freq(type, k1, clk);
		maxfreq = max(freq, last_freq);
	}

	return freq;
}


uint32_t measure_abist_freq(enum ABIST_CLK clk)
{
	return measure_stable_fmeter_freq(ABIST, 0, clk);
}
EXPORT_SYMBOL(measure_abist_freq);


uint32_t measure_ckgen_freq(enum CKGEN_CLK clk)
{
	return measure_stable_fmeter_freq(CKGEN, 0, clk);
}
EXPORT_SYMBOL(measure_ckgen_freq);


static void measure_abist_clock(enum ABIST_CLK clk, struct seq_file *s)
{
	uint32_t freq = measure_abist_freq(clk);
	seq_printf(s, "%2d: %-25s %7u\n", clk, ABIST_CLK_NAME[clk], freq);
}


static void measure_ckgen_clock(enum CKGEN_CLK clk, struct seq_file *s)
{
	uint32_t freq = measure_ckgen_freq(clk);
	seq_printf(s, "%2d: %-25s %7u\n", clk, CKGEN_CLK_NAME[clk], freq);
}


static int fmeter_show(struct seq_file *s, void *v)
{
	enum ABIST_CLK abist_clks[] = {
		ABIST_AD_MAIN_H546M_CK,
		ABIST_AD_MAIN_H364M_CK,
		ABIST_AD_MAIN_H218P4M_CK,
		ABIST_AD_MAIN_H156M_CK,
		ABIST_AD_UNIV_624M_CK,
		ABIST_AD_UNIV_416M_CK,
		ABIST_AD_UNIV_249P6M_CK,
		ABIST_AD_UNIV_178P3M_CK,
		ABIST_AD_UNIV_48M_CK,
		ABIST_AD_USB_48M_CK,
		ABIST_AD_MMPLL_CK,
		ABIST_AD_MSDCPLL_CK,
		ABIST_AD_DPICLK,
		ABIST_CLKPH_MCK_O,
		ABIST_AD_MEMPLL2_CKOUT0_PRE_ISO,
		ABIST_AD_MCUPLL1_H481M_CK,
		ABIST_AD_MDPLL1_416M_CK,
		ABIST_AD_WPLL_CK,
		ABIST_AD_WHPLL_CK,
		ABIST_RTC32K_CK_I,
		ABIST_AD_SYS_26M_CK,
		ABIST_AD_VENCPLL_CK,
		ABIST_AD_MIPI_26M_CK,
		ABIST_AD_MEM_26M_CK,
		ABIST_AD_PLLGP_TST_CK,
		ABIST_AD_DSI0_LNTC_DSICLK,
		ABIST_AD_MPPLL_TST_CK,
		ABIST_ARMPLL_OCC_MON,
		ABIST_AD_MEM2MIPI_26M_CK,
		ABIST_AD_MEMPLL_MONCLK,
		ABIST_AD_MEMPLL2_MONCLK,
		ABIST_AD_MEMPLL3_MONCLK,
		ABIST_AD_MEMPLL4_MONCLK,
		ABIST_AD_MEMPLL_REFCLK,
		ABIST_AD_MEMPLL_FBCLK,
		ABIST_AD_MEMPLL2_REFCLK,
		ABIST_AD_MEMPLL2_FBCLK,
		ABIST_AD_MEMPLL3_REFCLK,
		ABIST_AD_MEMPLL3_FBCLK,
		ABIST_AD_MEMPLL4_REFCLK,
		ABIST_AD_MEMPLL4_FBCLK,
		ABIST_AD_MEMPLL_TSTDIV2_CK,
		ABIST_AD_LVDSPLL_CK,
		ABIST_AD_LVDSTX_MONCLK,
		ABIST_AD_HDMITX_MONCLK,
		ABIST_AD_USB20_C240M,
		ABIST_AD_USB20_C240M_1P,
		ABIST_AD_MONREF_CK,
		ABIST_AD_MONFBK_CK,
		ABIST_AD_TVDPLL_CK,
		ABIST_AD_AUDPLL_CK,
		ABIST_AD_LVDSPLL_ETH_CK,
	};

	enum CKGEN_CLK ckgen_clks[] = {
		CKGEN_HF_FAXI_CK,
		CKGEN_HD_FAXI_CK,
		CKGEN_HF_FNFI2X_CK,
		CKGEN_HF_FDDRPHYCFG_CK,
		CKGEN_HF_FMM_CK,
		CKGEN_F_FPWM_CK,
		CKGEN_HF_FVDEC_CK,
		CKGEN_HF_FMFG_CK,
		CKGEN_HF_FCAMTG_CK,
		CKGEN_F_FUART_CK,
		CKGEN_HF_FSPI_CK,
		CKGEN_F_FUSB20_CK,
		CKGEN_HF_FMSDC30_0_CK,
		CKGEN_HF_FMSDC30_1_CK,
		CKGEN_HF_FMSDC30_2_CK,
		CKGEN_HF_FAUDIO_CK,
		CKGEN_HF_FAUD_INTBUS_CK,
		CKGEN_HF_FPMICSPI_CK,
		CKGEN_F_FRTC_CK,
		CKGEN_F_F26M_CK,
		CKGEN_F_F32K_MD1_CK,
		CKGEN_F_FRTC_CONN_CK,
		CKGEN_HF_FETH_50M_CK,
		CKGEN_HD_HAXI_NLI_CK,
		CKGEN_HD_QAXIDCM_CK,
		CKGEN_F_FFPC_CK,
		CKGEN_HF_FDPI0_CK,
		CKGEN_F_FCKBUS_CK_SCAN,
		CKGEN_F_FCKRTC_CK_SCAN,
		CKGEN_HF_FDPILVDS_CK,
	};

	int i;

	uint32_t old_pll_hp_con0 = clk_readl(PLL_HP_CON0);
	clk_writel(PLL_HP_CON0, 0x0);                       /* disable PLL hopping */

	seq_puts(s, "abist:\n");

	for (i = 0; i < ARRAY_SIZE(abist_clks); i++)
		measure_abist_clock(abist_clks[i], s);

	seq_puts(s, "ckgen:\n");

	for (i = 0; i < ARRAY_SIZE(ckgen_clks); i++)
		measure_ckgen_clock(ckgen_clks[i], s);

	/* restore old setting */
	clk_writel(PLL_HP_CON0, old_pll_hp_con0);

	return 0;
}


static int fmeter_open(struct inode *inode, struct file *file)
{
	return single_open(file, fmeter_show, NULL);
}


static const struct file_operations fmeter_fops = {
	.owner      = THIS_MODULE,
	.open       = fmeter_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};


#endif /* CLKMGR_8127 */


#ifdef CONFIG_CLKMGR_STAT


static int clk_stat_show(struct seq_file *s, void *v)
{
	struct cg_clk *clk;
	struct list_head *pos;
	struct stat_node *node;
	int i, grp, offset;
	int skip;

	seq_printf(s, "\n********** clk stat dump **********\n");
	for (i = 0; i < NR_CLKS; i++) {
		grp = i / 32;
		offset = i % 32;
		if (offset == 0)
			seq_printf(s, "\n*****[%02d][%-8s]*****\n", grp, grp_get_name(grp));

		clk = id_to_clk(i);
		if (!clk || !clk->grp || !clk->ops->check_validity(clk))
			continue;

		skip = (clk->cnt == 0) && (clk->state == 0) && list_empty(&clk->head);
		if (skip)
			continue;

		seq_printf(s, "[%02d]state=%u, cnt=%u", offset, clk->state, clk->cnt);
		list_for_each(pos, &clk->head) {
			node = list_entry(pos, struct stat_node, link);
			seq_printf(s, "\t(%s,%u,%u)", node->name, node->cnt_on, node->cnt_off);
		}
		seq_printf(s, "\n");
	}

	return 0;
}


static int clk_stat_open(struct inode *inode, struct file *file)
{
	return single_open(file, clk_stat_show, NULL);
}


static const struct file_operations clk_stat_fops = {
	.owner   = THIS_MODULE,
	.open    = clk_stat_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};


#endif


static int clk_force_on_show(struct seq_file *s, void *v)
{
	int i;
	struct cg_clk *clk;

	seq_printf(s, "********** clk force on info dump **********\n");
	for (i = 0; i < NR_CLKS; i++) {
		clk = &clks[i];
		if (clk->force_on) {
			seq_printf(s, "clock %d (0x%08x @ %s) is force on\n", i,
					clk->mask, clk->grp->name);
		}
	}

	seq_printf(s, "\n********** clk_force_on help **********\n");
	seq_printf(s, "set clk force on: echo set id > /proc/clkmgr/clk_force_on\n");
	seq_printf(s, "clr clk force on: echo clr id > /proc/clkmgr/clk_force_on\n");

	return 0;
}


static int clk_force_on_open(struct inode *inode, struct file *file)
{
	return single_open(file, clk_force_on_show, NULL);
}


static int clk_force_on_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;

	char cmd[10];
	int id;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (sscanf(desc, "%s %d", cmd, &id) == 2) {
		if (!strcmp(cmd, "set"))
			clk_set_force_on(id);
		else if (!strcmp(cmd, "clr"))
			clk_clr_force_on(id);
	}

	return count;
}


static const struct file_operations clk_force_on_fops = {
	.owner   = THIS_MODULE,
	.write   = clk_force_on_write,
	.open    = clk_force_on_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};


static int udelay_test_show(struct seq_file *s, void *v)
{
	seq_printf(s, "\n********** udelay_test help **********\n");
	seq_printf(s, "test udelay:  echo delay > /proc/clkmgr/udelay_test\n");

	return 0;
}


static int udelay_test_open(struct inode *inode, struct file *file)
{
	return single_open(file, udelay_test_show, NULL);
}


static int udelay_test_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;

	unsigned int delay;
	unsigned int pre, pos;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;

	desc[len] = '\0';

	if (sscanf(desc, "%u", &delay) == 1) {
		pre = clk_readl(0xF0008028);
		udelay(delay);
		pos = clk_readl(0xF0008028);
		clk_info("udelay(%u) test: pre=0x%08x, pos=0x%08x, delta=%u\n",
				delay, pre, pos, pos-pre);
	}

	return count;
}


static const struct file_operations udelay_test_fops = {
	.owner   = THIS_MODULE,
	.write   = udelay_test_write,
	.open    = udelay_test_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};


void mt_clkmgr_debug_init(void)
{
	struct proc_dir_entry *entry;
	struct proc_dir_entry *clkmgr_dir;

	clkmgr_dir = proc_mkdir("clkmgr", NULL);
	if (!clkmgr_dir) {
		clk_err("[%s]: fail to mkdir /proc/clkmgr\n", __func__);
		return;
	}

	entry = proc_create("pll_test", 00640, clkmgr_dir, &pll_test_fops);
	entry = proc_create("pll_fsel", 00640, clkmgr_dir, &pll_fsel_fops);

#ifdef CONFIG_CLKMGR_STAT
	entry = proc_create("pll_stat", 00440, clkmgr_dir, &pll_stat_fops);
#endif

	entry = proc_create("subsys_test", 00640, clkmgr_dir, &subsys_test_fops);

#ifdef CONFIG_CLKMGR_STAT
	entry = proc_create("subsys_stat", 00440, clkmgr_dir, &subsys_stat_fops);
#endif

	entry = proc_create("mux_test", 00640, clkmgr_dir, &mux_test_fops);

#ifdef CONFIG_CLKMGR_STAT
	entry = proc_create("mux_stat", 00440, clkmgr_dir, &mux_stat_fops);
#endif

	entry = proc_create("clk_test", 00640, clkmgr_dir, &clk_test_fops);

#ifdef CONFIG_CLKMGR_STAT
	entry = proc_create("clk_stat", 00440, clkmgr_dir, &clk_stat_fops);
#endif

	entry = proc_create("clk_force_on", 00640, clkmgr_dir, &clk_force_on_fops);
	entry = proc_create("udelay_test", 00640, clkmgr_dir, &udelay_test_fops);

#if CLKMGR_8127
	entry = proc_create("fmeter", 00640, clkmgr_dir, &fmeter_fops);
#endif /* CLKMGR_8127 */

}

/***********************************
*for early suspend
************************************/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void clkmgr_early_suspend(struct early_suspend *h)
{
	clk_info("[%s]: Dump Top MUX register, CLK_CFG_0=0x%x, CLK_CFG_1=0x%x, CLK_CFG_2=0x%x, CLK_CFG_3=0x%x\n",
		__func__, clk_readl(CLK_CFG_0), clk_readl(CLK_CFG_1), clk_readl(CLK_CFG_2), clk_readl(CLK_CFG_3));

	return;
}
static void clkmgr_late_resume(struct early_suspend *h)
{
	clk_info("[%s]: Dump Top MUX register, CLK_CFG_0=0x%x, CLK_CFG_1=0x%x, CLK_CFG_2=0x%x, CLK_CFG_3=0x%x\n",
		__func__, clk_readl(CLK_CFG_0), clk_readl(CLK_CFG_1), clk_readl(CLK_CFG_2), clk_readl(CLK_CFG_3));

	return;
}

static struct early_suspend mt_clkmgr_early_suspend_handler = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 250,
	.suspend = clkmgr_early_suspend,
	.resume  = clkmgr_late_resume,
};
#endif /* CONFIG_HAS_EARLYSUSPEND */

struct platform_device CLK_helper_device = {
	.name = "CLK",
	.id   = -1,
	.dev  = {},
};

int CLK_pm_restore_noirq(struct device *device)
{
	struct subsys *sys;

	sys = &syss[SYS_DIS];
	sys->state = sys->ops->get_state(sys);

	muxs[MT_MUX_MM].cnt = 1;
	plls[VENCPLL].cnt = 1;

	clk_set_force_on_locked(&clks[MT_CG_DISP0_SMI_LARB0]);
	clk_set_force_on_locked(&clks[MT_CG_DISP0_SMI_COMMON]);

	clk_info("CLK_pm_restore_noirq\n");

	return 0;
}

static const struct dev_pm_ops CLK_helper_pm_ops = {
	.restore_noirq = CLK_pm_restore_noirq,
};

static struct platform_driver CLK_helper_driver = {
	.driver     = {
		.name   = "CLK",
#ifdef CONFIG_PM
		.pm     = &CLK_helper_pm_ops,
#endif
		.owner  = THIS_MODULE,
	},
};

static int mt_clkmgr_debug_bringup_init(void)
{
	int ret;

#if defined(CONFIG_CLKMGR_BRINGUP) || defined(CONFIG_MTK_LDVT)
	mt_clkmgr_bringup_init();
#endif

	mt_clkmgr_debug_init();

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&mt_clkmgr_early_suspend_handler);
#endif

	ret = platform_device_register(&CLK_helper_device);
	if (ret) {
		clk_warn("CLK_helper_device register fail(%d)\n", ret);
		return ret;
	}

	ret = platform_driver_register(&CLK_helper_driver);
	if (ret) {
		clk_warn("CLK_helper_driver register fail(%d)\n", ret);
		return ret;
	}

	return 0;
}
static int __init mt_clkmgr_late_init(void)
{
	return 0;
}

module_init(mt_clkmgr_debug_bringup_init);
late_initcall(mt_clkmgr_late_init);


/*************CLKM****************/
void CLKM_32K(bool flag)
{
	unsigned long flags;

	clkmgr_lock(flags);
	if (flag == true) {
		/* output 32K */
		clk_setl(CLK_CFG_10, 0x00000002); /* CLK_CKMON1_SEL use 2:rtc32k_ck_i */
	} else {
		/* disable 32K */
		clk_clrl(CLK_CFG_10, 0x00000002);
	}
	clkmgr_unlock(flags);
	return;
}
EXPORT_SYMBOL(CLKM_32K);


int CLK_Monitor(enum ckmon_sel ckmon, enum monitor_clk_sel sel, int div)
{
	unsigned long flags;
	unsigned int ckmon_shift = 0;
	unsigned int temp;

	if (div > 255 || ckmon == 0) {
		clk_info("CLK_Monitor error parameter\n");
		return 1;
	}

	clkmgr_lock(flags);

	if (ckmon == 1)
		ckmon_shift = 0;
	else if (ckmon == 2)
		ckmon_shift = 8;
	else if (ckmon == 3)
		ckmon_shift = 16;

	temp = clk_readl(CLK_CFG_10);
	temp = temp & (~(0xf << ckmon_shift));
	temp = temp | ((sel & 0xf) << ckmon_shift);
	clk_writel(CLK_CFG_10, temp);

	temp = clk_readl(CLK_CFG_11);
	temp = temp & (~(0xff << ckmon_shift));
	temp = temp | ((div & 0xff) << ckmon_shift);
	clk_writel(CLK_CFG_11, temp);

	clk_info("CLK_Monitor Reg: CLK_CFG_10=0x%x, CLK_CFG_11=0x%x\n",
		clk_readl(CLK_CFG_10), clk_readl(CLK_CFG_11));

	clkmgr_unlock(flags);
	return 0;
}
EXPORT_SYMBOL(CLK_Monitor);


#if CLKMGR_CLKM0


int CLK_Monitor_0(enum ckmon_sel ckmon, enum monitor_clk_sel_0 sel, int div)
{
	unsigned long flags;
	unsigned int temp;

	if (div > 255 || ckmon > 0) {
		clk_info("CLK_Monitor_0 error parameter\n");
		return 1;
	}
    
	clkmgr_lock(flags);

	temp = clk_readl(CLK26CALI_0);
	clk_writel(CLK26CALI_0, temp | 0x80);

	clk_writel(CLK_CFG_8, sel << 8);
    
	temp = clk_readl(CLK_MISC_CFG_1);
	clk_writel(CLK_MISC_CFG_1, div & 0xff);

	clk_info("CLK_Monitor_0 Reg: CLK26CALI_0=0x%08x, CLK_CFG_8=0x%08x, CLK_MISC_CFG_1=0x%08x\n", 
		clk_readl(CLK26CALI_0), clk_readl(CLK_CFG_8), clk_readl(CLK_MISC_CFG_1));

	clkmgr_unlock(flags);
	return 0;
}
EXPORT_SYMBOL(CLK_Monitor_0);


#endif /* CLKMGR_CLKM0 */
