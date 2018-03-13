#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_power_gs.h>

extern unsigned int *mt8127_power_gs_data_connection;
extern unsigned int mt8127_power_gs_data_connection_len;

extern unsigned int *mt6323_power_gs_data_connection;
extern unsigned int mt6323_power_gs_data_connection_len;

unsigned int mt6333_power_gs_data_connection[] = {
    // Buck
    0x009F, 0x0080, 0x0000,
    0x00A0, 0x0007, 0x0003,
    0x006D, 0x007f, 0x0010,
};

void mt_power_gs_dump_data_connection(void)
{
    mt_power_gs_compare("Data Connection",                                                    \
                        mt8127_power_gs_data_connection, mt8127_power_gs_data_connection_len, \
                        mt6323_power_gs_data_connection, mt6323_power_gs_data_connection_len, \
                        mt6333_power_gs_data_connection, sizeof(mt6333_power_gs_data_connection));
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
static int dump_data_connection_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
    int len = 0;
    char *p = buf;

    p += sprintf(p, "mt_power_gs : data_connection\n");

    mt_power_gs_dump_data_connection();

    len = p - buf;
    return len;
}
#else
static int dump_data_connection_read(struct file *flip, char __user *buf, size_t count, loff_t *offset)
{
    int len = 0;
    char *p = buf;

    p += sprintf(p, "mt_power_gs : data_connection\n");

    mt_power_gs_dump_data_connection();

    len = p - buf;
    return len;
}

static const struct file_operations data_connection_proc_fops = {
    .owner = THIS_MODULE,
    .read = dump_data_connection_read,
};
#endif

static void __exit mt_power_gs_data_connection_exit(void)
{
    //return 0;
}

static int __init mt_power_gs_data_connection_init(void)
{
    struct proc_dir_entry *mt_entry = NULL;

    if (!mt_power_gs_dir)
    {
        printk("[%s]: mkdir /proc/mt_power_gs failed\n", __FUNCTION__);
    }
    else
    {
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0)
        mt_entry = create_proc_entry("dump_data_connection", S_IRUGO | S_IWUSR | S_IWGRP, mt_power_gs_dir);
        if (mt_entry)
        {
            mt_entry->read_proc = dump_data_connection_read;
        }
    #else
        mt_entry = proc_create("dump_data_connection", S_IRUGO | S_IWUSR | S_IWGRP, mt_power_gs_dir, &data_connection_proc_fops);
        if (!mt_entry) 
        {
            printk("[%s]: create proc file /proc/mt_power_gs/dump_data_connection failed\n", __FUNCTION__);
        }
    #endif
    }

    return 0;
}

module_init(mt_power_gs_data_connection_init);
module_exit(mt_power_gs_data_connection_exit);

MODULE_DESCRIPTION("MT8127 Power Golden Setting - Data Connection");
