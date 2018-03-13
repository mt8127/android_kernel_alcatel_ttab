#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_power_gs.h>
#include <mach/sync_write.h>

extern unsigned int *mt8127_power_gs_suspend;
extern unsigned int mt8127_power_gs_suspend_len;

#ifdef CONFIG_MTK_PMIC_MT6397
extern unsigned int *mt6397_power_gs_suspend;
extern unsigned int mt6397_power_gs_suspend_len;
#else

extern unsigned int *mt6323_power_gs_suspend;
extern unsigned int mt6323_power_gs_suspend_len;
#endif

#define slp_read(addr)              (*(volatile u32 *)(addr))
#define slp_write(addr, val)        mt65xx_reg_sync_writel(val, addr)

#ifdef CONFIG_MTK_PMIC_MT6397

#else
unsigned int mt6333_power_gs_suspend[] = {
    // Buck
    0x009F, 0x0080, 0x0000,
    0x00A0, 0x0007, 0x0000,
    0x006D, 0x007f, 0x0010,
};
#endif

void mt_power_gs_dump_suspend(void)
{
    #ifdef CONFIG_MTK_PMIC_MT6397
    mt_power_gs_compare("Suspend",                                            \
                        mt8127_power_gs_suspend, mt8127_power_gs_suspend_len, \
                        mt6397_power_gs_suspend, mt6397_power_gs_suspend_len, \
                        NULL, 0);
    #else
    mt_power_gs_compare("Suspend",                                            \
                        mt8127_power_gs_suspend, mt8127_power_gs_suspend_len, \
                        mt6323_power_gs_suspend, mt6323_power_gs_suspend_len, \
                        mt6333_power_gs_suspend, sizeof(mt6333_power_gs_suspend));
    #endif
    if(slp_read(0xf0010044) != 0x88492480)
    {
    	printk("slp_read(0xf0010044)=0x%x\n",slp_read(0xf0010044));
//    	slp_write(0xf0010044, 0x88492480);
    }	
    if(slp_read(0xf0010040) != 0x00000080)
    {
    	printk("slp_read(0xf0010044)=0x%x\n",slp_read(0xf0010040));
//    	slp_write(0xf0010040, 0x00000080);
    }
    if(slp_read(0xf0010000) != 0x00000400)
    {
    	printk("slp_read(0xf0010000)=0x%x\n",slp_read(0xf0010000));
//    	slp_write(0xf0010000, 0x00000400);
    }
    if(slp_read(0xf0010004) != 0x00000820)
    {
    	printk("slp_read(0xf0010004)=0x%x\n",slp_read(0xf0010004));
//    	slp_write(0xf0010004, 0x00000820);
    }
    if(slp_read(0xf0010008) != 0x00000400)
    {
    	printk("slp_read(0xf0010008)=0x%x\n",slp_read(0xf0010008));
//    	slp_write(0xf0010008, 0x00000400);
    }
    if(slp_read(0xf001000C) != 0x00000100)
    {
    	printk("slp_read(0xf001000C)=0x%x\n",slp_read(0xf001000C));
//    	slp_write(0xf001000C, 0x00000100);
    }
    if(slp_read(0xf0010010) != 0x00000100)
    {
    	printk("slp_read(0xf0010010)=0x%x\n",slp_read(0xf0010010));
//    	slp_write(0xf0010010, 0x00000100);
    }
    if(slp_read(0xf0010014) != 0x00000100)
    {
    	printk("slp_read(0xf0010014)=0x%x\n",slp_read(0xf0010014));
//    	slp_write(0xf0010014, 0x00000100);
    }
    if(slp_read(0xf0010068) != 0x0000002)
    {
    	printk("slp_read(0xf0010068)=0x%x\n",slp_read(0xf0010068));
//    	slp_write(0xf0010068, 0x00000002);
    }
    if(slp_read(0xf0010050) != 0x00000000)
    {
    	printk("slp_read(0xf0010050)=0x%x\n",slp_read(0xf0010050));
//    	slp_write(0xf0010050, 0x00000000);
    }
    if(slp_read(0xf0010824) != 0x24248800)
    {
    	printk("slp_read(0xf0010824)=0x%x\n",slp_read(0xf0010824));
//    	slp_write(0xf0010824, 0x24248800);
    }
    if(slp_read(0xf0010820) != 0x00000000)
    {
    	printk("slp_read(0xf0010820)=0x%x\n",slp_read(0xf0010820));
//    	slp_write(0xf0010820, 0x00000000);
    }
    if(slp_read(0xf0010800) != 0x00008000)
    {
    	printk("slp_read(0xf0010800)=0x%x\n",slp_read(0xf0010800));
//    	slp_write(0xf0010800, 0x00008000);
    }
    if(slp_read(0xf0010804) != 0x00008000)
    {
    	printk("slp_read(0xf0010804)=0x%x\n",slp_read(0xf0010804));
//    	slp_write(0xf0010804, 0x00008000);
    }
    if(slp_read(0xf0010808) != 0x00008000)
    {
    	printk("slp_read(0xf0010808)=0x%x\n",slp_read(0xf0010808));
//    	slp_write(0xf0010808, 0x00008000);
    }
    if(slp_read(0xf001080C) != 0x00008000)
    {
    	printk("slp_read(0xf001080C)=0x%x\n",slp_read(0xf001080C));
//    	slp_write(0xf001080C, 0x00008000);
    }
    if(slp_read(0xf0010810) != 0x00008000)
    {
    	printk("slp_read(0xf0010810)=0x%x\n",slp_read(0xf0010810));
//    	slp_write(0xf0010810, 0x00008000);
    }
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
static int dump_suspend_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
    int len = 0;
    char *p = buf;

    p += sprintf(p, "mt_power_gs : suspend\n");

    mt_power_gs_dump_suspend();

    len = p - buf;
    return len;
}
#else
static ssize_t dump_suspend_read(struct file *flip, char __user *buf, size_t count, loff_t *offset)
{
    int len = 0;
    char *p = buf;

    p += sprintf(p, "mt_power_gs : suspend\n");

    mt_power_gs_dump_suspend();

    len = p - buf;
    return len;
}

static const struct file_operations suspend_proc_fops = {
    .owner = THIS_MODULE,
    .read = dump_suspend_read,
};
#endif

static void __exit mt_power_gs_suspend_exit(void)
{
    //return 0;
}

static int __init mt_power_gs_suspend_init(void)
{
    struct proc_dir_entry *mt_entry = NULL;

    if (!mt_power_gs_dir)
    {
        printk("[%s]: mkdir /proc/mt_power_gs failed\n", __FUNCTION__);
    }
    else
    {
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
        mt_entry = create_proc_entry("dump_suspend", S_IRUGO | S_IWUSR | S_IWGRP, mt_power_gs_dir);
        if (mt_entry)
        {
            mt_entry->read_proc = dump_suspend_read;
        }
    #else
        mt_entry = proc_create("dump_suspend", S_IRUGO | S_IWUSR | S_IWGRP, mt_power_gs_dir, &suspend_proc_fops);
        if (!mt_entry) 
        {
            printk("[%s]: create proc file /proc/mt_power_gs/dump_suspend failed\n", __FUNCTION__);
        }
    #endif
    }

    return 0;
}

module_init(mt_power_gs_suspend_init);
module_exit(mt_power_gs_suspend_exit);

MODULE_DESCRIPTION("MT8127 Power Golden Setting - Suspend");
