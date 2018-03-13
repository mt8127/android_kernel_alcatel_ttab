#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_power_gs.h>

extern unsigned int *mt8127_power_gs_tdd_talking;
extern unsigned int mt8127_power_gs_tdd_talking_len;

extern unsigned int *mt6323_power_gs_tdd_talking;
extern unsigned int mt6323_power_gs_tdd_talking_len;

unsigned int mt6333_power_gs_tdd_talking[] = {
    // Buck
    0x009F, 0x0080, 0x0000,
    0x00A0, 0x0007, 0x0003,
    0x006D, 0x007f, 0x0010,
};

void mt_power_gs_dump_tdd_talking(void)
{
    mt_power_gs_compare("TDD Talking",                                                \
                        mt8127_power_gs_tdd_talking, mt8127_power_gs_tdd_talking_len, \
                        mt6323_power_gs_tdd_talking, mt6323_power_gs_tdd_talking_len, \
                        mt6333_power_gs_tdd_talking, sizeof(mt6333_power_gs_tdd_talking));
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
static int dump_tdd_talking_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
    int len = 0;
    char *p = buf;

    p += sprintf(p, "mt_power_gs : tdd_talking\n");

    mt_power_gs_dump_tdd_talking();

    len = p - buf;
    return len;
}
#else
static int dump_tdd_talking_read(struct file *flip, char __user *buf, size_t count, loff_t *offset)
{
    int len = 0;
    char *p = buf;

    p += sprintf(p, "mt_power_gs : tdd_talking\n");

    mt_power_gs_dump_tdd_talking();

    len = p - buf;
    return len;
}

static const struct file_operations tdd_talking_proc_fops = {
    .owner = THIS_MODULE,
    .read = dump_tdd_talking_read,
};
#endif

static void __exit mt_power_gs_tdd_talking_exit(void)
{
    //return 0;
}

static int __init mt_power_gs_tdd_talking_init(void)
{
    struct proc_dir_entry *mt_entry = NULL;

    if (!mt_power_gs_dir)
    {
        printk("[%s]: mkdir /proc/mt_power_gs failed\n", __FUNCTION__);
    }
    else
    {
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
        mt_entry = create_proc_entry("dump_tdd_talking", S_IRUGO | S_IWUSR | S_IWGRP, mt_power_gs_dir);
        if (mt_entry)
        {
            mt_entry->read_proc = dump_tdd_talking_read;
        }
    #else
        mt_entry = proc_create("dump_tdd_talking", S_IRUGO | S_IWUSR | S_IWGRP, mt_power_gs_dir, &tdd_talking_proc_fops);
        if (!mt_entry) 
        {
            printk("[%s]: create proc file /proc/mt_power_gs/dump_tdd_talking failed\n", __FUNCTION__);
        }
    #endif
    }

    return 0;
}

module_init(mt_power_gs_tdd_talking_init);
module_exit(mt_power_gs_tdd_talking_exit);

MODULE_DESCRIPTION("MT8127 Power Golden Setting - TDD Talking");
