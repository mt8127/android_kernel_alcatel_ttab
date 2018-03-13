/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/jiffies.h>
#include <linux/seq_file.h>

#include <asm/system.h>
#include <asm/uaccess.h>

#include "mach/sync_write.h"
#include "mach/mt_typedefs.h"
#include "mach/mt_cpufreq.h"

static struct hrtimer mt_cpu_ss_timer;
struct task_struct *mt_cpu_ss_thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(mt_cpu_ss_timer_waiter);

static int mt_cpu_ss_period_s = 0;
static int mt_cpu_ss_period_ns = 100;

static int mt_cpu_ss_timer_flag = 0;

static bool mt_cpu_ss_debug_mode = false;
static bool mt_cpu_ss_period_mode = false;

enum hrtimer_restart mt_cpu_ss_timer_func(struct hrtimer *timer)
{
    if (mt_cpu_ss_debug_mode)
        printk("[%s]: enter timer function\n", __FUNCTION__);

    mt_cpu_ss_timer_flag = 1; wake_up_interruptible(&mt_cpu_ss_timer_waiter);

    return HRTIMER_NORESTART;
}

int mt_cpu_ss_thread_handler(void *unused)
{
    kal_uint32 flag = 0;

    do
    {
        ktime_t ktime = ktime_set(mt_cpu_ss_period_s, mt_cpu_ss_period_ns);

        wait_event_interruptible(mt_cpu_ss_timer_waiter, mt_cpu_ss_timer_flag != 0);
        mt_cpu_ss_timer_flag = 0;

        if (!flag)
        {
            mt65xx_reg_sync_writel((DRV_Reg32(TOP_CKMUXSEL) & 0x0ff3), TOP_CKMUXSEL);
            flag = 1;
        }
        else
        {
            mt65xx_reg_sync_writel((DRV_Reg32(TOP_CKMUXSEL) | 0x0004), TOP_CKMUXSEL);
            flag = 0;
        }

        if (mt_cpu_ss_debug_mode)
            printk("[%s]: TOP_CKMUXSEL = 0x%x\n", __FUNCTION__, DRV_Reg32(TOP_CKMUXSEL));

        hrtimer_start(&mt_cpu_ss_timer, ktime, HRTIMER_MODE_REL);

    } while (!kthread_should_stop());

    return 0;
}

static int cpu_ss_mode_show(struct seq_file* s, void* v)
{
    if ((DRV_Reg32(TOP_CKMUXSEL) & 0x000C) == 0)
        seq_printf(s, "CPU clock source is CLKSQ\n");
    else
        seq_printf(s, "CPU clock source is ARMPLL\n");

    return 0;
}

static int cpu_ss_mode_open(struct inode *inode, struct file *file)
{
    return single_open(file, cpu_ss_mode_show, NULL);
}

static ssize_t cpu_ss_mode_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
    int len = 0, mode = 0;
    char desc[32];

    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
    if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';

    if (sscanf(desc, "%d", &mode) == 1)
    {
        if (mode)
        {
            printk("[%s]: config cpu speed switch mode = ARMPLL\n", __FUNCTION__);
            mt65xx_reg_sync_writel((DRV_Reg32(TOP_CKMUXSEL) | 0x0004), TOP_CKMUXSEL);
        }
        else
        {
            printk("[%s]: config cpu speed switch mode = CLKSQ\n", __FUNCTION__);
            mt65xx_reg_sync_writel((DRV_Reg32(TOP_CKMUXSEL) & 0x0ff3), TOP_CKMUXSEL);
        }

        return count;
    }
    else
    {
        printk("[%s]: bad argument!! should be \"1\" or \"0\"\n", __FUNCTION__);
    }

    return -EINVAL;
}

static int cpu_ss_period_show(struct seq_file* s, void* v)
{
    seq_printf(s, "%d (s) %d (ns)\n", mt_cpu_ss_period_s, mt_cpu_ss_period_ns);

    return 0;
}

static int cpu_ss_period_open(struct inode *inode, struct file *file)
{
    return single_open(file, cpu_ss_period_show, NULL);
}

static ssize_t cpu_ss_period_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
    int len = 0, s = 0, ns = 0;
    char desc[32];

    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
    if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';

    if (sscanf(desc, "%d %d", &s, &ns) == 2)
    {
        printk("[%s]: set cpu speed switch period = %d (s), %d (ns)\n", __FUNCTION__, s, ns);
        mt_cpu_ss_period_s = s;
        mt_cpu_ss_period_ns = ns;
        return count;
    }
    else
    {
        printk("[%s]: bad argument!! should be \"[s]\" or \"[ns]\"\n", __FUNCTION__);
    }

    return -EINVAL;
}

static int cpu_ss_period_mode_show(struct seq_file* s, void* v)
{
    if (mt_cpu_ss_period_mode)
        seq_printf(s, "enable");
    else
        seq_printf(s, "disable");

    return 0;
}

static int cpu_ss_period_mode_open(struct inode *inode, struct file *file)
{
    return single_open(file, cpu_ss_period_mode_show, NULL);
}

static ssize_t cpu_ss_period_mode_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
    int len = 0;
    char mode[20], desc[32];
    ktime_t ktime = ktime_set(mt_cpu_ss_period_s, mt_cpu_ss_period_ns);

    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
    if (copy_from_user(desc, buffer, len)) 
    {
        return 0;
    }
    desc[len] = '\0';

    if (sscanf(desc, "%s", mode) == 1)
    {
        if (!strcmp(mode, "enable"))
        {
            printk("[%s]: enable cpu speed switch period mode\n", __FUNCTION__);
            mt_cpu_ss_period_mode = true;

            mt_cpu_ss_thread = kthread_run(mt_cpu_ss_thread_handler, 0, "cpu speed switch");
            if (IS_ERR(mt_cpu_ss_thread))
            {
                printk("[%s]: failed to create cpu speed switch thread\n", __FUNCTION__);
            }

            hrtimer_start(&mt_cpu_ss_timer, ktime, HRTIMER_MODE_REL);
            return count;
        }
        else if (!strcmp(mode, "disable"))
        {
            printk("[%s]: disable cpu speed switch period mode\n", __FUNCTION__);
            mt_cpu_ss_period_mode = false;

            kthread_stop(mt_cpu_ss_thread);

            mt65xx_reg_sync_writel((DRV_Reg32(TOP_CKMUXSEL) | 0x0004), TOP_CKMUXSEL);

            hrtimer_cancel(&mt_cpu_ss_timer);
            return count;
        }
        else
        {
            printk("[%s]: bad argument!! should be \"enable\" or \"disable\"\n", __FUNCTION__);
        }
    }
    else
    {
        printk("[%s]: bad argument!! should be \"enable\" or \"disable\"\n", __FUNCTION__);
    }

    return -EINVAL;
}

static int cpu_ss_debug_mode_show(struct seq_file* s, void* v)
{
    if (mt_cpu_ss_debug_mode)
        seq_printf(s, "enable");
    else
        seq_printf(s, "disable");

    return 0;
}

static int cpu_ss_debug_mode_open(struct inode *inode, struct file *file)
{
    return single_open(file, cpu_ss_debug_mode_show, NULL);
}

static ssize_t cpu_ss_debug_mode_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
    int len = 0;
    char mode[20], desc[32];

    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
    if (copy_from_user(desc, buffer, len)) 
    {
        return 0;
    }
    desc[len] = '\0';

    if (sscanf(desc, "%s", mode) == 1)
    {
        if (!strcmp(mode, "enable"))
        {
            printk("[%s]: enable cpu speed switch debug mode\n", __FUNCTION__);
            mt_cpu_ss_debug_mode = true;
            return count;
        }
        else if (!strcmp(mode, "disable"))
        {
            printk("[%s]: disable cpu speed switch debug mode\n", __FUNCTION__);
            mt_cpu_ss_debug_mode = false;
            return count;
        }
        else
        {
            printk("[%s]: bad argument!! should be \"enable\" or \"disable\"\n", __FUNCTION__);
        }
    }
    else
    {
        printk("[%s]: bad argument!! should be \"enable\" or \"disable\"\n", __FUNCTION__);
    }

    return -EINVAL;
}


static const struct file_operations cpu_ss_debug_mode_fops = {
	.owner      = THIS_MODULE,
	.write      = cpu_ss_debug_mode_write,
    .open       = cpu_ss_debug_mode_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static const struct file_operations cpu_ss_period_mode_fops = {
	.owner      = THIS_MODULE,
	.write      = cpu_ss_period_mode_write,
    .open       = cpu_ss_period_mode_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static const struct file_operations cpu_ss_period_fops = {
	.owner      = THIS_MODULE,
	.write      = cpu_ss_period_write,
    .open       = cpu_ss_period_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static const struct file_operations cpu_ss_mode_fops = {
	.owner      = THIS_MODULE,
	.write      = cpu_ss_mode_write,
    .open       = cpu_ss_mode_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};


/*********************************
* cpu speed stress initialization 
**********************************/
static int __init mt_cpu_ss_init(void)
{
    struct proc_dir_entry *mt_entry = NULL;
    struct proc_dir_entry *mt_cpu_ss_dir = NULL;

    hrtimer_init(&mt_cpu_ss_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    mt_cpu_ss_timer.function = mt_cpu_ss_timer_func;

    mt_cpu_ss_dir = proc_mkdir("cpu_ss", NULL);
    if (!mt_cpu_ss_dir)
    {
        pr_err("[%s]: mkdir /proc/cpu_ss failed\n", __FUNCTION__);
    }
    else
    {
        mt_entry = proc_create("cpu_ss_debug_mode", S_IRUGO | S_IWUSR | S_IWGRP, mt_cpu_ss_dir, &cpu_ss_debug_mode_fops);
        if (!mt_entry)
        {
            printk("[mtktscpu_init]: create /proc/cpu_ss/cpu_ss_debug_mode failed\n");
        }

        mt_entry = proc_create("cpu_ss_period_mode", S_IRUGO | S_IWUSR | S_IWGRP, mt_cpu_ss_dir, &cpu_ss_period_mode_fops);
        if (!mt_entry)
        {
            printk("[mtktscpu_init]: create /proc/cpu_ss/cpu_ss_period_mode failed\n");
        }

        mt_entry = proc_create("cpu_ss_period", S_IRUGO | S_IWUSR | S_IWGRP, mt_cpu_ss_dir, &cpu_ss_period_fops);
        if (!mt_entry)
        {
            printk("[mtktscpu_init]: create /proc/cpu_ss/cpu_ss_period failed\n");
        }

        mt_entry = proc_create("cpu_ss_mode", S_IRUGO | S_IWUSR | S_IWGRP, mt_cpu_ss_dir, &cpu_ss_mode_fops);
        if (!mt_entry)
        {
            printk("[mtktscpu_init]: create /proc/cpu_ss/cpu_ss_mode failed\n");
        }
    }

    return 0;
}

static void __exit mt_cpu_ss_exit(void)
{

}

module_init(mt_cpu_ss_init);
module_exit(mt_cpu_ss_exit);

MODULE_DESCRIPTION("MediaTek CPU Speed Stress driver");
MODULE_LICENSE("GPL");