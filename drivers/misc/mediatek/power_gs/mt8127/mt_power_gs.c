#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_spm_idle.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_power_gs.h>

#include <mach/mt_pmic_wrap.h>
#ifdef CONFIG_MTK_PMIC_MT6397
#include <mach/pmic_mt6320_sw.h>
#else
#include <mach/pmic_mt6323_sw.h>
#endif
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>



#define gs_read(addr) (*(volatile u32 *)(addr))

struct proc_dir_entry *mt_power_gs_dir = NULL;

#ifdef CONFIG_MTK_PMIC_MT6397

static U16 gs6397_pmic_read(U16 addr)
{
	U32 rdata=0;
	pwrap_read((U32)addr, &rdata);
	return (U16)rdata;
}

char pmic_6397_reg_0x13e[][10] = { 
 "Vproc15", "Vsram15", "Vcore", "Vgpu", "Vio18", "Vproc7", "Vsram7", "Vdram",
"Vusb", "Vtcxo", "n.a ", "n.a ", "n.a ", "n.a", "n.a", "Vrtc"
};

char pmic_6397_reg_0x140[][10] = { 
"Vmch", "Vmc", "Vio28", "Vibr", "Vgp6", "Vgp5", "Vgp4", "Vcamaf",
"Vcamio", "Vcamd", "Vemc_3v3", "n.a", "Vcama", "n.a", "n.a", "Va28"
};

#else

#define SUPPORT_MT6333 0
static U16 gs6323_pmic_read(U16 addr)
{
	U32 rdata=0;
	pwrap_read((U32)addr, &rdata);
	return (U16)rdata;
}

#if SUPPORT_MT6333
extern kal_uint32 mt6333_get_reg_value(kal_uint32 reg);
static U16 gs6333_pmic_read(U16 addr)
{
	U32 rdata=0;
	rdata = mt6333_get_reg_value((U32)addr);
	return (U16)rdata;
}
#endif
#endif



static void mt_power_gs_compare_pll(void)
{
    if (pll_is_on(MSDCPLL)) {
        printk("MSDCPLL: %s\n", pll_is_on(MSDCPLL) ?  "on" : "off");
    }
/*
    if (subsys_is_on(SYS_MD1)) {
        printk("SYS_MD1: %s\n", subsys_is_on(SYS_MD1) ? "on" : "off");
    }
*/
    if (subsys_is_on(SYS_CONN)) {
        printk("SYS_CONN: %s\n", subsys_is_on(SYS_CONN) ? "on" : "off");
    }

    if (subsys_is_on(SYS_DIS)) {
        printk("SYS_DIS: %s\n", subsys_is_on(SYS_DIS) ? "on" : "off");
    }

    if (subsys_is_on(SYS_MFG)) {
        printk("SYS_MFG: %s\n", subsys_is_on(SYS_MFG) ? "on" : "off");
    }

    if (subsys_is_on(SYS_ISP)) {
        printk("SYS_ISP: %s\n", subsys_is_on(SYS_ISP) ? "on" : "off");
    }

    if (subsys_is_on(SYS_VDE)) {
        printk("SYS_VDE: %s\n", subsys_is_on(SYS_VDE) ? "on" : "off");
    }
}

static void Golden_Setting_Compare_PMIC_LDO(void)                                         
{                 
#ifdef CONFIG_MTK_PMIC_MT6397
    u16 temp_value, temp_i;

    temp_value = gs6397_pmic_read(0x13E);
	//clc_notice("PMIC 0x13E : temp_value=%d\n", temp_value); 
    for( temp_i=0 ; temp_i<16 ; temp_i++ )
    {
        if( (temp_value & (0x1<<temp_i)) == (0x1<<temp_i) )
        {
            clc_notice("PMIC %s : On.\n", pmic_6397_reg_0x13e[temp_i]);      
        }
    }
    
    // PMIC 0x140 ==========================================
    temp_value = gs6397_pmic_read(0x140);
    //clc_notice("PMIC 0x140 : temp_value=%d\n", temp_value); 
    for( temp_i=0 ; temp_i<16 ; temp_i++ )
    {
        if( (temp_value & (0x1<<temp_i)) == (0x1<<temp_i) )
        {
            clc_notice("PMIC %s : On.\n", pmic_6397_reg_0x140[temp_i]);      
        }
    }
#else

#endif    
}


void mt_power_gs_diff_output(unsigned int val1, unsigned int val2)
{
    int i = 0;
    unsigned int diff = val1 ^ val2;

    while (diff != 0)
    {
        if ((diff % 2) != 0) printk("%d ", i);
        diff /= 2;
        i++;
    }
    printk("\n");
}

void mt_power_gs_compare(char *scenario, \
                         unsigned int *mt8127_power_gs, unsigned int mt8127_power_gs_len, \
                         unsigned int *first_power_gs, unsigned int first_power_gs_len, \
                         unsigned int *second_power_gs, unsigned int second_power_gs_len)
{
    unsigned int i, val1, val2;

    // MT8127
    for (i = 0 ; i < mt8127_power_gs_len ; i += 3)
    {
        val1 = gs_read(mt8127_power_gs[i]) & mt8127_power_gs[i+1];
        val2 = mt8127_power_gs[i+2] & mt8127_power_gs[i+1];
        if (val1 != val2)
        {
            printk("%s - MT8127 - 0x%x - 0x%x - 0x%x - 0x%x - ", \
                    scenario, mt8127_power_gs[i], gs_read(mt8127_power_gs[i]), mt8127_power_gs[i+1], mt8127_power_gs[i+2]);
            mt_power_gs_diff_output(val1, val2);
        }
    }
#ifdef CONFIG_MTK_PMIC_MT6397
    for (i = 0 ; i < first_power_gs_len ; i += 3)
    {
        val1 = gs6397_pmic_read(first_power_gs[i]) & first_power_gs[i+1];
        val2 = first_power_gs[i+2] & first_power_gs[i+1];
        if (val1 != val2)
        {
            printk("%s - MT6397 - 0x%x - 0x%x - 0x%x - 0x%x - ", \
                    scenario, first_power_gs[i], gs6397_pmic_read(first_power_gs[i]), first_power_gs[i+1], first_power_gs[i+2]);
            mt_power_gs_diff_output(val1, val2);
        }
    }
#else
    // MT6323
    for (i = 0 ; i < first_power_gs_len ; i += 3)
    {
        val1 = gs6323_pmic_read(first_power_gs[i]) & first_power_gs[i+1];
        val2 = first_power_gs[i+2] & first_power_gs[i+1];
        if (val1 != val2)
        {
            printk("%s - MT6323 - 0x%x - 0x%x - 0x%x - 0x%x - ", \
                    scenario, first_power_gs[i], gs6323_pmic_read(first_power_gs[i]), first_power_gs[i+1], first_power_gs[i+2]);
            mt_power_gs_diff_output(val1, val2);
        }
    }

    #if SUPPORT_MT6333
    // MT6333
    for (i = 0 ; i < second_power_gs_len ; i += 3)
    {
        val1 = gs6333_pmic_read(second_power_gs[i]) & second_power_gs[i+1];
        val2 = second_power_gs[i+2] & second_power_gs[i+1];
        if (val1 != val2)
        {
            printk("%s - MT6333 - 0x%x - 0x%x - 0x%x - 0x%x - ", \
                    scenario, second_power_gs[i], gs6333_pmic_read(second_power_gs[i]), second_power_gs[i+1], second_power_gs[i+2]);
            mt_power_gs_diff_output(val1, val2);
        }
    }
    #endif
#endif
    Golden_Setting_Compare_PMIC_LDO();
    
    mt_power_gs_compare_pll();
}
EXPORT_SYMBOL(mt_power_gs_compare);

static void __exit mt_power_gs_exit(void)
{
    //return 0;
}

static int __init mt_power_gs_init(void)
{
    mt_power_gs_dir = proc_mkdir("mt_power_gs", NULL);
    if (!mt_power_gs_dir)
    {
        printk("[%s]: mkdir /proc/mt_power_gs failed\n", __FUNCTION__);
    }

    return 0;
}

module_init(mt_power_gs_init);
module_exit(mt_power_gs_exit);

MODULE_DESCRIPTION("MT8127 Low Power Golden Setting");
