/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2012. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*
 * Version: V2.0
 * Release Date: 2013/08/28
 * Contact: andrew@goodix.com, meta@goodix.com
 * Revision Record:
 *      V1.4:
 *          1. New Heartbeat/ESD-protect Mechanism(external watchdog)
 *          2. doze mode, sliding wakeup
 *          3. config length verification & 3 more config groups(GT9 Sensor_ID: 0 ~ 5)
 *          4. charger status switch
 *                  By Meta, 2013/03/11
 *      V1.6:
 *          1. pen/stylus support
 *          2. slide wakeup, new esd optimization
 *                  By Meta, 2013/04/18
 *      V1.8:
 *          1. read double check & fixed config support
 *          2. other optimizations
 *                  By Meta, 2013/06/08
 *      V2.0:
 *          1. compatible with GT9XXF
 *          2. I2C DMA support
 *                  By Meta, 2013/08/28
 *      V2.2:
 *          1. update gt9xx_config to compatible with Linux 3.10
 *          2. gesture wakeup
 *          3. pen separate input device, active-pen button support
 *          4. coordinates & keys optimization
 *          5. no longer support GT915S
 *                  By Meta, 2014/01/14
 *      V2.2.6:
 *          Special edition for GT910 flashless
 *          1. firmware check
 *      V2.2.7
 *		 Special edition for GT910 flashless
 *          1. modified to support gesture wakeup module
 */

#include "tpd.h"
#include "tpd_custom_gt9xx.h"
#include <asm/ptrace.h>

#ifndef TPD_NO_GPIO
#include "cust_gpio_usage.h"
#endif
#ifdef TPD_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

#if GTP_SUPPORT_I2C_DMA
    #include <linux/dma-mapping.h>
#endif

extern int gesture_value;
extern char tpd_firmware_version_val[5];
extern int tpd_config_version;
extern int tpd_x_line;
extern int tpd_y_line;
/*[PLATFORM]-Add-BEGIN by falin.luo@tcl.com 2015/5/6*/
/*MMI test app read sensor id to distinguish the tp manufacturer*/
extern u8 tpd_sensor_id;
/*[PLATFORM]-Add-END by falin.luo@tcl.com 2015/5/6*/

int gtp_autotool_setting;
EXPORT_SYMBOL(gtp_autotool_setting);
extern struct tpd_device *tpd;

static int tpd_sleep_flag = 0;
static int tpd_flag = 0;
int tpd_halt = 0;
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);

#ifdef TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

typedef enum
{
    DOZE_DISABLED = 0,
    DOZE_ENABLED = 1,
    DOZE_WAKEUP = 2,
}DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct i2c_client *client);

#if GTP_CHARGER_SWITCH
    #ifdef MT6573
        #define CHR_CON0      (0xF7000000+0x2FA00)
    #else
        extern kal_bool upmu_is_chr_det(void);
    #endif
    static void gtp_charger_switch(s32 dir_update);
#endif

#if GTP_HAVE_TOUCH_KEY
const u16 touch_key_array[] = GTP_KEY_TAB;
#define GTP_MAX_KEY_NUM ( sizeof( touch_key_array )/sizeof( touch_key_array[0] ) )
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
//static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#if GTP_SUPPORT_I2C_DMA
s32 i2c_dma_write(struct i2c_client *client, u16 addr, u8 *txbuf, s32 len);
s32 i2c_dma_read(struct i2c_client *client, u16 addr, u8 *rxbuf, s32 len);

static u8 *gpDMABuf_va = NULL;
static u32 gpDMABuf_pa = 0;
#endif

s32 gtp_send_cfg(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);
static void tpd_eint_interrupt_handler(void);
static int touch_event_handler(void *unused);
static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
static int tpd_i2c_remove(struct i2c_client *client);
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);

#if GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client *);
extern void uninit_wr_node(void);
#endif

#if (GTP_ESD_PROTECT || GTP_COMPATIBLE_MODE)
static s32 force_reset_guitar(s32);
#endif

#if GTP_ESD_PROTECT
static int clk_tick_cnt = 200;
u8 esd_running = 0;
spinlock_t esd_lock;
extern unsigned char gtp_default_FW_fl[];
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct *gtp_esd_check_workqueue = NULL;
static s32 gtp_init_ext_watchdog(struct i2c_client *client);
static void gtp_esd_check_func(struct work_struct *);
void gtp_esd_switch(struct i2c_client *client, s32 on);
#endif

#ifdef TPD_PROXIMITY
#define TPD_PROXIMITY_VALID_REG                   0x814E
#define TPD_PROXIMITY_ENABLE_REG                  0x8042
static u8 tpd_proximity_flag = 0;
static u8 tpd_proximity_detect = 1;//0-->close ; 1--> far away
#endif

struct i2c_client *i2c_client_point = NULL;
static const struct i2c_device_id tpd_i2c_id[] = {{"gt9xx", 0}, {}};
static unsigned short force[] = {0, 0xBA, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces,};
static struct i2c_board_info __initdata i2c_tpd = { I2C_BOARD_INFO("gt9xx", (0xBA >> 1))};
static struct i2c_driver tpd_i2c_driver =
{
    .probe = tpd_i2c_probe,
    .remove = tpd_i2c_remove,
    .detect = tpd_i2c_detect,
    .driver.name = "gt9xx",
    .id_table = tpd_i2c_id,
    .address_list = (const unsigned short *) forces,
};

static u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
    = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

#pragma pack(1)
typedef struct
{
    u16 pid;                 //product id   //
    u16 vid;                 //version id   //
} st_tpd_info;
#pragma pack()

st_tpd_info tpd_info;
u8 int_type = 0;
u32 abs_x_max = 0;
u32 abs_y_max = 0;
u8 gtp_rawdiff_mode = 0;
u8 cfg_len = 0;
u8 grp_cfg_version = 0;
u8 fixed_config = 0;
u8 pnl_init_error = 0;

#if GTP_WITH_PEN
struct input_dev *pen_dev;
#endif

#if GTP_COMPATIBLE_MODE
u8 driver_num = 0;
u8 sensor_num = 0;
/*[PLATFORM]-Add-BEGIN by falin.luo@tcl.com 2015/4/29*/
/*change counter data type to u16, because GTP_CHK_FS_MNT_MAX is 300 */
u16 gtp_ref_retries = 0;
u16 gtp_clk_retries = 0;
static u16 is_data_mounted = 0;
/*[PLATFORM]-Add-NED   by falin.luo@tcl.com 2015/4/29*/
CHIP_TYPE_T gtp_chip_type = CHIP_TYPE_GT9;
u8 rqst_processing = 0;
u8 is_950 = 0;
u8 after_912_1020 = 0;


//add by caoyang for ·À¶¶
//static struct timer_list buttons_timer;
//static DECLARE_WAIT_QUEUE_HEAD(button_waitq);
//end

extern u8 gup_check_fs_mounted(char *path_name);
extern u8 gup_clk_calibration(void);
extern int gup_reload_fw_dsp(void *dir, u8 dwn_mode);
extern s32 gup_fw_download_proc(void *dir, u8 dwn_mode);
void gtp_get_chip_type(struct i2c_client *client);
u8 gtp_fw_startup(struct i2c_client *client);
static u8 gtp_bak_ref_proc(struct i2c_client *client, u8 mode);
static u8 gtp_main_clk_proc(struct i2c_client *client);
static void gtp_recovery_reset(struct i2c_client *client);

#if GTP_COMPATIBLE_MODE
u8 gtp_hopping_buf[16] = {0};
#endif

#if GTP_FL_LITTLE_SYSTEM
u8 power_is_down = 0;
u8 little_fw_mode = 0;
u8 fw_block = 0;            // 0: not started, 1 ~ 11/12 ss51 seg a/b each 2K, 13: ss51 seg b, 10K/12K
u8 block_section = 1;       // 1 ~ 8, 2K total, 256 Bytes each

char symbolic_state1[][20] = {"OTHERS", "BUFFER_FULL", "CHECK_COMPLETE", "CHECK_ERROR", "WAIT_CHECK", "OTHERS", "OTHERS", "OTHERS"};
char symbolic_state2[][20] = {"OTHERS", "IS_A_SEG", "IS_B_SEG_FIRST", "IS_B_SEG_OTHER", "IS_B_SEG_LAST", "OTHERS", "OTHERS", "OTHERS"};

static void tpd_up(s32 x, s32 y, s32 id);
extern u8 gup_burn_ss51_block(struct i2c_client *client, s32 block_section, s32 fw_block, u16 *fw_chksum);
extern u8 gup_burn_ss51_seg_b(struct i2c_client *client, s32 size, u16 *fw_chksum);
#endif

#endif

/* proc file system */
s32 i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len);
s32 i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len);

static ssize_t gt91xx_config_read_proc(struct file *, char __user *, size_t, loff_t *);
static ssize_t gt91xx_config_write_proc(struct file *, const char __user *, size_t, loff_t *);

static struct proc_dir_entry *gt91xx_config_proc = NULL;
static const struct file_operations config_proc_ops = {
    .owner = THIS_MODULE,
    .read = gt91xx_config_read_proc,
    .write = gt91xx_config_write_proc,
};

#define VELOCITY_CUSTOM
#ifdef VELOCITY_CUSTOM
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 10
#endif

// for magnify velocity********************************************
#define TOUCH_IOC_MAGIC 'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)

int g_v_magnify_x = TPD_VELOCITY_CUSTOM_X;
int g_v_magnify_y = TPD_VELOCITY_CUSTOM_Y;
static int tpd_misc_open(struct inode *inode, struct file *file)
{
    return nonseekable_open(inode, file);
}

static int tpd_misc_release(struct inode *inode, struct file *file)
{
    return 0;
}

static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
                               unsigned long arg)
{
    //char strbuf[256];
    void __user *data;

    long err = 0;

    if (_IOC_DIR(cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    }
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
    {
        err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }

    if (err)
    {
        printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }

    switch (cmd)
    {
        case TPD_GET_VELOCITY_CUSTOM_X:
            data = (void __user *) arg;

            if (data == NULL)
            {
                err = -EINVAL;
                break;
            }

            if (copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
            {
                err = -EFAULT;
                break;
            }

            break;

        case TPD_GET_VELOCITY_CUSTOM_Y:
            data = (void __user *) arg;

            if (data == NULL)
            {
                err = -EINVAL;
                break;
            }

            if (copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
            {
                err = -EFAULT;
                break;
            }

            break;

        default:
            printk("tpd: unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
            break;

    }

    return err;
}


static struct file_operations tpd_fops =
{
//  .owner = THIS_MODULE,
    .open = tpd_misc_open,
    .release = tpd_misc_release,
    .unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "gt9xx_touch",
    .fops = &tpd_fops,
};

//**********************************************
#endif

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, "mtk-tpd");
    return 0;
}

#ifdef TPD_PROXIMITY
static s32 tpd_get_ps_value(void)
{
    return tpd_proximity_detect;
}

static s32 tpd_enable_ps(s32 enable)
{
    u8  state;
    s32 ret = -1;

    if (enable)
    {
        state = 1;
        tpd_proximity_flag = 1;
        GTP_INFO("TPD proximity function to be on.");
    }
    else
    {
        state = 0;
        tpd_proximity_flag = 0;
        GTP_INFO("TPD proximity function to be off.");
    }

    ret = i2c_write_bytes(i2c_client_point, TPD_PROXIMITY_ENABLE_REG, &state, 1);

    if (ret < 0)
    {
        GTP_ERROR("TPD %s proximity cmd failed.", state ? "enable" : "disable");
        return ret;
    }

    GTP_INFO("TPD proximity function %s success.", state ? "enable" : "disable");
    return 0;
}

s32 tpd_ps_operate(void *self, u32 command, void *buff_in, s32 size_in,
                   void *buff_out, s32 size_out, s32 *actualout)
{
    s32 err = 0;
    s32 value;
    hwm_sensor_data *sensor_data;

    switch (command)
    {
        case SENSOR_DELAY:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                GTP_ERROR("Set delay parameter error!");
                err = -EINVAL;
            }

            // Do nothing
            break;

        case SENSOR_ENABLE:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                GTP_ERROR("Enable sensor parameter error!");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                err = tpd_enable_ps(value);
            }

            break;

        case SENSOR_GET_DATA:
            if ((buff_out == NULL) || (size_out < sizeof(hwm_sensor_data)))
            {
                GTP_ERROR("Get sensor data parameter error!");
                err = -EINVAL;
            }
            else
            {
                sensor_data = (hwm_sensor_data *)buff_out;
                sensor_data->values[0] = tpd_get_ps_value();
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
            }

            break;

        default:
            GTP_ERROR("proxmy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}
#endif


static ssize_t gt91xx_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
    char *ptr = page;
    char temp_data[GTP_CONFIG_MAX_LENGTH + 2] = {0};
    int i;

    if (*ppos)  // CMD call again
    {
        return 0;
    }

    ptr += sprintf(ptr, "==== GT9XX config init value====\n");

    for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", config[i + 2]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }

    ptr += sprintf(ptr, "\n");

    ptr += sprintf(ptr, "==== GT9XX config real value====\n");
    i2c_read_bytes(i2c_client_point, GTP_REG_CONFIG_DATA, temp_data, GTP_CONFIG_MAX_LENGTH);

    for (i = 0 ; i < GTP_CONFIG_MAX_LENGTH ; i++)
    {
        ptr += sprintf(ptr, "0x%02X ", temp_data[i]);

        if (i % 8 == 7)
            ptr += sprintf(ptr, "\n");
    }
    *ppos += ptr - page;
    return (ptr - page);
}

static ssize_t gt91xx_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
    s32 ret = 0;

    GTP_DEBUG("write count %d\n", count);

    if (count > GTP_CONFIG_MAX_LENGTH)
    {
        GTP_ERROR("size not match [%d:%d]\n", GTP_CONFIG_MAX_LENGTH, count);
        return -EFAULT;
    }

    if (copy_from_user(&config[2], buffer, count))
    {
        GTP_ERROR("copy from user fail\n");
        return -EFAULT;
    }

    ret = gtp_send_cfg(i2c_client_point);
    abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
    abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
    int_type = (config[TRIGGER_LOC]) & 0x03;

    if (ret < 0)
    {
        GTP_ERROR("send config failed.");
    }

    return count;
}

#if GTP_SUPPORT_I2C_DMA
s32 i2c_dma_read(struct i2c_client *client, u16 addr, u8 *rxbuf, s32 len)
{
    int ret;
    s32 retry = 0;
    u8 buffer[2];

    struct i2c_msg msg[2] =
    {
        {
            .addr = (client->addr & I2C_MASK_FLAG),
            .flags = 0,
            .buf = buffer,
            .len = 2,
            .timing = I2C_MASTER_CLOCK
        },
        {
            .addr = (client->addr & I2C_MASK_FLAG),
            .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
            .flags = I2C_M_RD,
            .buf = (u8*)gpDMABuf_pa,
            .len = len,
            .timing = I2C_MASTER_CLOCK
        },
    };

    buffer[0] = (addr >> 8) & 0xFF;
    buffer[1] = addr & 0xFF;

    if (rxbuf == NULL)
        return -1;

    //GTP_DEBUG("dma i2c read: 0x%04X, %d bytes(s)", addr, len);
    for (retry = 0; retry < 5; ++retry)
    {
        ret = i2c_transfer(client->adapter, &msg[0], 2);
        if (ret < 0)
        {
            continue;
        }
        memcpy(rxbuf, gpDMABuf_va, len);
        return 0;
    }
    GTP_ERROR("Dma I2C Read Error: 0x%04X, %d byte(s), err-code: %d", addr, len, ret);
    return ret;
}


s32 i2c_dma_write(struct i2c_client *client, u16 addr, u8 *txbuf, s32 len)
{
    int ret;
    s32 retry = 0;
    u8 *wr_buf = gpDMABuf_va;

    struct i2c_msg msg =
    {
        .addr = (client->addr & I2C_MASK_FLAG),
        .ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
        .flags = 0,
        .buf = (u8*)gpDMABuf_pa,
        .len = 2 + len,
        .timing = I2C_MASTER_CLOCK
    };

    wr_buf[0] = (u8)((addr >> 8) & 0xFF);
    wr_buf[1] = (u8)(addr & 0xFF);

    if (txbuf == NULL)
        return -1;

    //GTP_DEBUG("dma i2c write: 0x%04X, %d bytes(s)", addr, len);
    memcpy(wr_buf+2, txbuf, len);
    for (retry = 0; retry < 5; ++retry)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret < 0)
        {
            continue;
        }
        return 0;
    }
    GTP_ERROR("Dma I2C Write Error: 0x%04X, %d byte(s), err-code: %d", addr, len, ret);
    return ret;
}

s32 i2c_read_bytes_dma(struct i2c_client *client, u16 addr, u8 *rxbuf, s32 len)
{
    s32 left = len;
    s32 read_len = 0;
    u8 *rd_buf = rxbuf;
    s32 ret = 0;

    //GTP_DEBUG("Read bytes dma: 0x%04X, %d byte(s)", addr, len);
    while (left > 0)
    {
        if (left > GTP_DMA_MAX_TRANSACTION_LENGTH)
        {
            read_len = GTP_DMA_MAX_TRANSACTION_LENGTH;
        }
        else
        {
            read_len = left;
        }
        ret = i2c_dma_read(client, addr, rd_buf, read_len);
        if (ret < 0)
        {
            GTP_ERROR("dma read failed");
            return -1;
        }

        left -= read_len;
        addr += read_len;
        rd_buf += read_len;
    }
    return 0;
}

s32 i2c_write_bytes_dma(struct i2c_client *client, u16 addr, u8 *txbuf, s32 len)
{

    s32 ret = 0;
    s32 write_len = 0;
    s32 left = len;
    u8 *wr_buf = txbuf;

    //GTP_DEBUG("Write bytes dma: 0x%04X, %d byte(s)", addr, len);
    while (left > 0)
    {
        if (left > GTP_DMA_MAX_I2C_TRANSFER_SIZE)
        {
            write_len = GTP_DMA_MAX_I2C_TRANSFER_SIZE;
        }
        else
        {
            write_len = left;
        }
        ret = i2c_dma_write(client, addr, wr_buf, write_len);

        if (ret < 0)
        {
            GTP_ERROR("dma i2c write failed!");
            return -1;
        }

        left -= write_len;
        addr += write_len;
        wr_buf += write_len;
    }
    return 0;
}
#endif


int i2c_read_bytes_non_dma(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
    u8 buffer[GTP_ADDR_LENGTH];
    u8 retry;
    u16 left = len;
    u16 offset = 0;

    struct i2c_msg msg[2] =
    {
        {
            //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
            .addr = ((client->addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)),
            .flags = 0,
            .buf = buffer,
            .len = GTP_ADDR_LENGTH,
            .timing = I2C_MASTER_CLOCK
        },
        {
            //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
            .addr = ((client->addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)),
            .flags = I2C_M_RD,
            .timing = I2C_MASTER_CLOCK
        },
    };

    if (rxbuf == NULL)
        return -1;

    //GTP_DEBUG("i2c_read_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

    while (left > 0)
    {
        buffer[0] = ((addr + offset) >> 8) & 0xFF;
        buffer[1] = (addr + offset) & 0xFF;

        msg[1].buf = &rxbuf[offset];

        if (left > MAX_TRANSACTION_LENGTH)
        {
            msg[1].len = MAX_TRANSACTION_LENGTH;
            left -= MAX_TRANSACTION_LENGTH;
            offset += MAX_TRANSACTION_LENGTH;
        }
        else
        {
            msg[1].len = left;
            left = 0;
        }

        retry = 0;

        while (i2c_transfer(client->adapter, &msg[0], 2) != 2)
        {
            retry++;

            //if (retry == 20)
            if (retry == 5)
            {
                GTP_ERROR("I2C read 0x%X length=%d failed\n", addr + offset, len);
                return -1;
            }
        }
    }

    return 0;
}


int i2c_read_bytes(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
#if GTP_SUPPORT_I2C_DMA
    return i2c_read_bytes_dma(client, addr, rxbuf, len);
#else
    return i2c_read_bytes_non_dma(client, addr, rxbuf, len);
#endif
}

s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    s32 ret = -1;
    u16 addr = (buf[0] << 8) + buf[1];

    ret = i2c_read_bytes_non_dma(client, addr, &buf[2], len - 2);

    if (!ret)
    {
        return 2;
    }
    else
    {
    //#if GTP_GESTURE_WAKEUP
    if(gesture_value > 0){
        if (DOZE_ENABLED == doze_status)
        {
            return ret;
        }
    //#endif
    }
    #if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == gtp_chip_type)
        {
        #if GTP_FL_LITTLE_SYSTEM
            if (little_fw_mode)
            {
                // do nothing
                GTP_INFO("Little fw enabled, no esd reset.");
            }
            else
        #endif
            {
                gtp_recovery_reset(client);
            }
        }
        else
    #endif
        {
            gtp_reset_guitar(client, 20);
        }
        return ret;
    }
}


s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
    u8 buf[16] = {0};
    u8 confirm_buf[16] = {0};
    u8 retry = 0;

    while (retry++ < 3)
    {
        memset(buf, 0xAA, 16);
        buf[0] = (u8)(addr >> 8);
        buf[1] = (u8)(addr & 0xFF);
        gtp_i2c_read(client, buf, len + 2);

        memset(confirm_buf, 0xAB, 16);
        confirm_buf[0] = (u8)(addr >> 8);
        confirm_buf[1] = (u8)(addr & 0xFF);
        gtp_i2c_read(client, confirm_buf, len + 2);

        if (!memcmp(buf, confirm_buf, len+2))
        {
            memcpy(rxbuf, confirm_buf+2, len);
            return SUCCESS;
        }
    }
    GTP_ERROR("i2c read 0x%04X, %d bytes, double check failed!", addr, len);
    return FAIL;
}

int i2c_write_bytes_non_dma(struct i2c_client *client, u16 addr, u8 *txbuf, int len)
{
    u8 buffer[MAX_TRANSACTION_LENGTH];
    u16 left = len;
    u16 offset = 0;
    u8 retry = 0;

    struct i2c_msg msg =
    {
        //.addr = ((client->addr &I2C_MASK_FLAG) | (I2C_ENEXT_FLAG)),
        .addr = ((client->addr &I2C_MASK_FLAG) | (I2C_PUSHPULL_FLAG)),
        .flags = 0,
        .buf = buffer,
        .timing = I2C_MASTER_CLOCK,
    };


    if (txbuf == NULL)
        return -1;

    //GTP_DEBUG("i2c_write_bytes to device %02X address %04X len %d\n", client->addr, addr, len);

    while (left > 0)
    {
        retry = 0;

        buffer[0] = ((addr + offset) >> 8) & 0xFF;
        buffer[1] = (addr + offset) & 0xFF;

        if (left > MAX_I2C_TRANSFER_SIZE)
        {
            memcpy(&buffer[GTP_ADDR_LENGTH], &txbuf[offset], MAX_I2C_TRANSFER_SIZE);
            msg.len = MAX_TRANSACTION_LENGTH;
            left -= MAX_I2C_TRANSFER_SIZE;
            offset += MAX_I2C_TRANSFER_SIZE;
        }
        else
        {
            memcpy(&buffer[GTP_ADDR_LENGTH], &txbuf[offset], left);
            msg.len = left + GTP_ADDR_LENGTH;
            left = 0;
        }

        //GTP_DEBUG("byte left %d offset %d\n", left, offset);

        while (i2c_transfer(client->adapter, &msg, 1) != 1)
        {
            retry++;

            //if (retry == 20)
            if (retry == 5)
            {
                //dump_stack();
                GTP_ERROR("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
                return -1;
            }
        }
    }

    return 0;
}

int i2c_write_bytes(struct i2c_client *client, u16 addr, u8 *txbuf, int len)
{
#if GTP_SUPPORT_I2C_DMA
    return i2c_write_bytes_dma(client, addr, txbuf, len);
#else
    return i2c_write_bytes_non_dma(client, addr, txbuf, len);
#endif
}

s32 gtp_i2c_write(struct i2c_client *client, u8 *buf, s32 len)
{
    s32 ret = -1;
    u16 addr = (buf[0] << 8) + buf[1];

    ret = i2c_write_bytes_non_dma(client, addr, &buf[2], len - 2);

    if (!ret)
    {
        return 1;
    }
    else
    {
    //#if GTP_GESTURE_WAKEUP
    if(gesture_value > 0){
        if (DOZE_ENABLED == doze_status)
        {
            return ret;
        }
    //#endif
    }
    #if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == gtp_chip_type)
        {
        #if GTP_FL_LITTLE_SYSTEM
            if (little_fw_mode)
            {
                // do nothing
                GTP_INFO("Little fw enabled, no esd reset.");
            }
            else
        #endif
            {
                gtp_recovery_reset(client);
            }
        }
        else
    #endif
        {
            gtp_reset_guitar(client, 20);
        }
        return ret;
    }
}



/*******************************************************
Function:
    Send config Function.

Input:
    client: i2c client.

Output:
    Executive outcomes.0--success,non-0--fail.
*******************************************************/
s32 gtp_send_cfg(struct i2c_client *client)
{
    s32 ret = 1;

#if GTP_DRIVER_SEND_CFG
    s32 retry = 0;
    if (fixed_config)
    {
        GTP_INFO("Ic fixed config, no config sent!");
        return 0;
    }
    else if (pnl_init_error)
    {
        GTP_INFO("Error occurred in init_panel, no config sent!");
        return 0;
    }

    GTP_INFO("Driver Send Config");
    for (retry = 0; retry < 5; retry++)
    {
        ret = gtp_i2c_write(client, config, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);

        if (ret > 0)
        {
            break;
        }
    }
#endif
    return ret;
}


/*******************************************************
Function:
    Read goodix touchscreen version function.

Input:
    client: i2c client struct.
    version:address to store version info

Output:
    Executive outcomes.0---succeed.
*******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16 *version)
{
    s32 ret = -1;
    s32 i;
    u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

    GTP_DEBUG_FUNC();

    ret = gtp_i2c_read(client, buf, sizeof(buf));

    if (ret < 0)
    {
        GTP_ERROR("GTP read version failed");
        return ret;
    }

    if (version)
    {
        *version = (buf[7] << 8) | buf[6];
    }

    tpd_info.vid = *version;
    tpd_info.pid = 0x00;

    for (i = 0; i < 4; i++)
    {
        if (buf[i + 2] < 0x30)break;

        tpd_info.pid |= ((buf[i + 2] - 0x30) << ((3 - i) * 4));
    }

    if (buf[5] == 0x00)
    {
        GTP_INFO("IC VERSION: %c%c%c_%02x%02x",
             buf[2], buf[3], buf[4], buf[7], buf[6]);
    }
    else
    {
        GTP_INFO("IC VERSION:%c%c%c%c_%02x%02x",
             buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
    }
    sprintf(tpd_firmware_version_val,"%02x%02x",buf[7], buf[6]);
    tpd_firmware_version_val[4] = '\0';
    printk(KERN_ERR"caoyang test tpd_firmware_version_val:%s",tpd_firmware_version_val);
    return ret;
}

#if GTP_DRIVER_SEND_CFG
/*******************************************************
Function:
    Get information from ic, such as resolution and
    int trigger type
Input:
    client: i2c client private struct.

Output:
    FAIL: i2c failed, SUCCESS: i2c ok
*******************************************************/
static s32 gtp_get_info(struct i2c_client *client)
{
    u8 opr_buf[6] = {0};
    s32 ret = 0;

    opr_buf[0] = (u8)((GTP_REG_CONFIG_DATA+1) >> 8);
    opr_buf[1] = (u8)((GTP_REG_CONFIG_DATA+1) & 0xFF);

    ret = gtp_i2c_read(client, opr_buf, 6);
    if (ret < 0)
    {
        return FAIL;
    }

    abs_x_max = (opr_buf[3] << 8) + opr_buf[2];
    abs_y_max = (opr_buf[5] << 8) + opr_buf[4];

    opr_buf[0] = (u8)((GTP_REG_CONFIG_DATA+6) >> 8);
    opr_buf[1] = (u8)((GTP_REG_CONFIG_DATA+6) & 0xFF);

    ret = gtp_i2c_read(client, opr_buf, 3);
    if (ret < 0)
    {
        return FAIL;
    }
    int_type = opr_buf[2] & 0x03;

    GTP_INFO("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
            abs_x_max,abs_y_max, int_type);

    return SUCCESS;
}
#endif


/*******************************************************
Function:
    GTP initialize function.

Input:
    client: i2c client private struct.

Output:
    Executive outcomes.0---succeed.
*******************************************************/
static s32 gtp_init_panel(struct i2c_client *client)
{
    s32 ret = 0;

#if GTP_DRIVER_SEND_CFG
    s32 i;
    u8 check_sum = 0;
    u8 opr_buf[16];
    u8 sensor_id = 0;

    u8 cfg_info_group1[] = CTP_CFG_GROUP1;
    u8 cfg_info_group2[] = CTP_CFG_GROUP2;
    u8 cfg_info_group3[] = CTP_CFG_GROUP3;
    u8 cfg_info_group4[] = CTP_CFG_GROUP4;
    u8 cfg_info_group5[] = CTP_CFG_GROUP5;
    u8 cfg_info_group6[] = CTP_CFG_GROUP6;
    u8 *send_cfg_buf[] = {cfg_info_group1, cfg_info_group2, cfg_info_group3,
                        cfg_info_group4, cfg_info_group5, cfg_info_group6};
    u8 cfg_info_len[] = { CFG_GROUP_LEN(cfg_info_group1),
                          CFG_GROUP_LEN(cfg_info_group2),
                          CFG_GROUP_LEN(cfg_info_group3),
                          CFG_GROUP_LEN(cfg_info_group4),
                          CFG_GROUP_LEN(cfg_info_group5),
                          CFG_GROUP_LEN(cfg_info_group6)};

    GTP_DEBUG("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d",
        cfg_info_len[0], cfg_info_len[1], cfg_info_len[2], cfg_info_len[3],
        cfg_info_len[4], cfg_info_len[5]);

    if ((!cfg_info_len[1]) && (!cfg_info_len[2]) &&
        (!cfg_info_len[3]) && (!cfg_info_len[4]) &&
        (!cfg_info_len[5]))
    {
        sensor_id = 0;
    }
    else
    {
    #if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == gtp_chip_type)
        {
            msleep(50);
        }
    #endif
        ret = gtp_i2c_read_dbl_check(client, GTP_REG_SENSOR_ID, &sensor_id, 1);
        if (SUCCESS == ret)
        {
            if (sensor_id >= 0x06)
            {
                GTP_ERROR("Invalid sensor_id(0x%02X), No Config Sent!", sensor_id);
                pnl_init_error = 1;
                return -1;
            }
        }
        else
        {
            GTP_ERROR("Failed to get sensor_id, No config sent!");
            pnl_init_error = 1;
            return -1;
        }
        GTP_INFO("Sensor_ID: %d", sensor_id);
    }

    cfg_len = cfg_info_len[sensor_id];
    tpd_config_version= send_cfg_buf[sensor_id][0];
	/*[PLATFORM]-Add-BEGIN by falin.luo@tcl.com 2015/5/6*/
	/*MMI test app read sensor id to distinguish the tp manufacturer*/
	tpd_sensor_id = sensor_id;
	/*[PLATFORM]-Add-END by falin.luo@tcl.com 2015/5/6*/
    GTP_INFO("CTP_CONFIG_GROUP%d used, config length: %d", sensor_id + 1, cfg_len);

    if (cfg_len < GTP_CONFIG_MIN_LENGTH)
    {
        GTP_ERROR("CTP_CONFIG_GROUP%d is INVALID CONFIG GROUP! NO Config Sent! You need to check you header file CFG_GROUP section!", sensor_id+1);
        pnl_init_error = 1;
        return -1;
    }

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == gtp_chip_type)
    {
        fixed_config = 0;
    }
    else
#endif
    {
        ret = gtp_i2c_read_dbl_check(client, GTP_REG_CONFIG_DATA, &opr_buf[0], 1);

        if (ret == SUCCESS)
        {
            GTP_DEBUG("CFG_CONFIG_GROUP%d Config Version: %d, 0x%02X; IC Config Version: %d, 0x%02X", sensor_id+1,
                        send_cfg_buf[sensor_id][0], send_cfg_buf[sensor_id][0], opr_buf[0], opr_buf[0]);

            if (opr_buf[0] < 90)
            {
                grp_cfg_version = send_cfg_buf[sensor_id][0];       // backup group config version
                send_cfg_buf[sensor_id][0] = 0x00;
                fixed_config = 0;
            }
            else        // treated as fixed config, not send config
            {
                GTP_INFO("Ic fixed config with config version(%d)", opr_buf[0]);
                fixed_config = 1;
                gtp_get_info(client);
                return 0;
            }
        }
        else
        {
            GTP_ERROR("Failed to get ic config version!No config sent!");
            return -1;
        }
    }

    memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
    memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id], cfg_len);

#if GTP_CUSTOM_CFG
    config[RESOLUTION_LOC]     = (u8)GTP_MAX_WIDTH;
    config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH>>8);
    config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
    config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT>>8);

    if (GTP_INT_TRIGGER == 0)  //RISING
    {
        config[TRIGGER_LOC] &= 0xfe;
    }
    else if (GTP_INT_TRIGGER == 1)  //FALLING
    {
        config[TRIGGER_LOC] |= 0x01;
    }
#endif  // GTP_CUSTOM_CFG

    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < cfg_len; i++)
    {
        check_sum += config[i];
    }
    config[cfg_len] = (~check_sum) + 1;

#else // DRIVER NOT SEND CONFIG
    cfg_len = GTP_CONFIG_MAX_LENGTH;
    ret = gtp_i2c_read(client, config, cfg_len + GTP_ADDR_LENGTH);
    if (ret < 0)
    {
        GTP_ERROR("Read Config Failed, Using DEFAULT Resolution & INT Trigger!");
        abs_x_max = GTP_MAX_WIDTH;
        abs_y_max = GTP_MAX_HEIGHT;
        int_type = GTP_INT_TRIGGER;
    }
#endif // GTP_DRIVER_SEND_CFG

    GTP_DEBUG_FUNC();
    if ((abs_x_max == 0) && (abs_y_max == 0))
    {
        abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
        abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
        int_type = (config[TRIGGER_LOC]) & 0x03;
    }

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == gtp_chip_type)
    {
        u8 have_key = 0;
        if (is_950)
        {
            driver_num = config[GTP_REG_MATRIX_DRVNUM - GTP_REG_CONFIG_DATA + 2];
            sensor_num = config[GTP_REG_MATRIX_SENNUM - GTP_REG_CONFIG_DATA + 2];
        }
        else
        {
            driver_num = (config[CFG_LOC_DRVA_NUM]&0x1F) + (config[CFG_LOC_DRVB_NUM]&0x1F);
            sensor_num = (config[CFG_LOC_SENS_NUM]&0x0F) + ((config[CFG_LOC_SENS_NUM]>>4)&0x0F);
        }

        have_key = config[GTP_REG_HAVE_KEY - GTP_REG_CONFIG_DATA + 2] & 0x01;  // have key or not
        if (1 == have_key)
        {
            driver_num--;
        }

        if ((cfg_len == 186) && after_912_1020)
        {
            GTP_DEBUG("Firmware after 912_1020, set config length to 228.");

            cfg_len = 228;

            config[GTP_ADDR_LENGTH + 226] = config[GTP_ADDR_LENGTH + 184];

            memset(&config[GTP_ADDR_LENGTH + 184], 0x00, 228 - 186);

            config[GTP_ADDR_LENGTH + 227] = 0x01;

        }
        tpd_x_line = driver_num;
	tpd_y_line = sensor_num;
        GTP_INFO("Driver * Sensor: %d * %d(Key: %d), X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
            driver_num, sensor_num, have_key, abs_x_max,abs_y_max,int_type);
    }
    else
#endif
    {
    #if GTP_DRIVER_SEND_CFG
        ret = gtp_send_cfg(client);
        if (ret < 0)
        {
            GTP_ERROR("Send config error.");
        }
        // set config version to CTP_CFG_GROUP
        // for resume to send config
        config[GTP_ADDR_LENGTH] = grp_cfg_version;
        check_sum = 0;
        for (i = GTP_ADDR_LENGTH; i < cfg_len; i++)
        {
            check_sum += config[i];
        }
        config[cfg_len] = (~check_sum) + 1;
    #endif
        GTP_INFO("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
            abs_x_max,abs_y_max,int_type);
    }

    msleep(10);
    return 0;
}

static s8 gtp_i2c_test(struct i2c_client *client)
{

    u8 retry = 0;
    s8 ret = -1;
    u32 hw_info = 0;

    GTP_DEBUG_FUNC();

    while (retry++ < 5)
    {
        ret = i2c_read_bytes(client, GTP_REG_HW_INFO, (u8 *)&hw_info, sizeof(hw_info));

        if ((!ret) && (hw_info == 0x00900600))              //20121212
        {
            return ret;
        }

        GTP_ERROR("GTP_REG_HW_INFO : %08X", hw_info);
        GTP_ERROR("GTP i2c test failed time %d.", retry);
        msleep(10);
    }

    return -1;
}



/*******************************************************
Function:
    Set INT pin  as input for FW sync.

Note:
  If the INT is high, It means there is pull up resistor attached on the INT pin.
  Pull low the INT pin manaully for FW sync.
*******************************************************/
void gtp_int_sync(s32 ms)
{
    GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
    msleep(ms);
    GTP_GPIO_AS_INT(GTP_INT_PORT);
}

void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
    GTP_INFO("GTP RESET!\n");
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
    msleep(ms);
    GTP_GPIO_OUTPUT(GTP_INT_PORT, client->addr == 0x14);

    msleep(2);
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);

    msleep(6);                      //must >= 6ms

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == gtp_chip_type)
    {
        return;
    }
#endif

    gtp_int_sync(50);
#if GTP_ESD_PROTECT
    gtp_init_ext_watchdog(i2c_client_point);
#endif
}

static int tpd_power_on(struct i2c_client *client)
{
    int ret = 0;
    int reset_count = 0;

reset_proc:
    GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
    GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
    msleep(10);

#ifdef MT6573
    // power on CTP
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);

#else   // ( defined(MT6575) || defined(MT6577) || defined(MT6589) )

    #ifdef TPD_POWER_SOURCE_CUSTOM
        hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
    #else
        hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
    #endif
    #ifdef TPD_POWER_SOURCE_1800
        hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
    #endif

#endif

    gtp_reset_guitar(client, 20);

#if GTP_COMPATIBLE_MODE
    gtp_get_chip_type(client);

    if (CHIP_TYPE_GT9F == gtp_chip_type)
    {
        ret = gup_fw_download_proc(NULL, GTP_FL_FW_BURN);

        if(FAIL == ret)
        {
            GTP_ERROR("[tpd_power_on]Download fw failed.");
            if(reset_count++ < TPD_MAX_RESET_COUNT)
            {
                goto reset_proc;
            }
            else
            {
                return -1;
            }
        }

        ret = gtp_fw_startup(client);
        if(FAIL == ret)
        {
            GTP_ERROR("[tpd_power_on]Startup fw failed.");
            if(reset_count++ < TPD_MAX_RESET_COUNT)
            {
                goto reset_proc;
            }
            else
            {
                return -1;
            }
        }
    }
    else
#endif
    {
        ret = gtp_i2c_test(client);

        if (ret < 0)
        {
            GTP_ERROR("I2C communication ERROR!");

            if (reset_count < TPD_MAX_RESET_COUNT)
            {
                reset_count++;
                goto reset_proc;
            }
        }
    }
    return ret;
}

//**************** For GT9XXF Start ********************//
#if GTP_COMPATIBLE_MODE


void gtp_get_chip_type(struct i2c_client *client)
{
    u8 opr_buf[10] = {0x00};
    s32 ret = 0;

    msleep(10);

    ret = gtp_i2c_read_dbl_check(client, GTP_REG_CHIP_TYPE, opr_buf, 10);

    if (FAIL == ret)
    {
        GTP_ERROR("Failed to get chip-type, set chip type default: GOODIX_GT9");
        gtp_chip_type = CHIP_TYPE_GT9;
        return;
    }

    if (!memcmp(opr_buf, "GOODIX_GT9", 10))
    {
        gtp_chip_type = CHIP_TYPE_GT9;
    }
    else // GT9XXF
    {
        gtp_chip_type = CHIP_TYPE_GT9F;
    }
    GTP_INFO("Chip Type: %s", (gtp_chip_type == CHIP_TYPE_GT9) ? "GOODIX_GT9" : "GOODIX_GT9F");
}

static u8 gtp_bak_ref_proc(struct i2c_client *client, u8 mode)
{
    s32 i = 0;
    s32 j = 0;
    s32 ret = 0;
    struct file *flp = NULL;
    u8 *refp = NULL;
    u32 ref_len = 0;
    u32 ref_seg_len = 0;
    s32 ref_grps = 0;
    s32 ref_chksum = 0;
    u16 tmp = 0;

    GTP_DEBUG("[gtp_bak_ref_proc]Driver:%d,Sensor:%d.", driver_num, sensor_num);

    //check file-system mounted
    GTP_DEBUG("[gtp_bak_ref_proc]Waiting for FS %d", gtp_ref_retries);
    if (/*gup_check_fs_mounted("/data") == FAIL*/!is_data_mounted)/*[PLATFORM]-MOD by falin.luo@tcl.com 2015/4/29*/
    {
        GTP_DEBUG("[gtp_bak_ref_proc]/data not mounted");
        if(gtp_ref_retries++ < GTP_CHK_FS_MNT_MAX)
        {
        	msleep(100);/*[PLATFORM]-ADD by falin.luo@tcl.com 2015/4/29*/
            return FAIL;
        }
    }
    else
    {
        GTP_DEBUG("[gtp_bak_ref_proc]/data mounted !!!!");
    }

    if (is_950)
    {
        ref_seg_len = (driver_num * (sensor_num - 1) + 2) * 2;
        ref_grps = 6;
        ref_len =  ref_seg_len * 6;  // for GT950, backup-reference for six segments
    }
    else
    {
        ref_len = driver_num*(sensor_num-2)*2 + 4;
        ref_seg_len = ref_len;
        ref_grps = 1;
    }

    refp = (u8 *)kzalloc(ref_len, GFP_KERNEL);
    if(refp == NULL)
    {
        GTP_ERROR("Failed to allocate memory for reference buffer!");
        return FAIL;
    }
    memset(refp, 0, ref_len);

    //get ref file data
    flp = filp_open(GTP_BAK_REF_PATH, O_RDWR | O_CREAT, 0666);
    if (IS_ERR(flp))
    {
        GTP_ERROR("Failed to open/create %s.", GTP_BAK_REF_PATH);
        if (GTP_BAK_REF_SEND == mode)
        {
            goto default_bak_ref;
        }
        else
        {
            goto exit_ref_proc;
        }
    }

    switch (mode)
    {
    case GTP_BAK_REF_SEND:
        {
            flp->f_op->llseek(flp, 0, SEEK_SET);
            ret = flp->f_op->read(flp, (char *)refp, ref_len, &flp->f_pos);
            if(ret < 0)
            {
                GTP_ERROR("Read ref file failed, send default bak ref.");
                goto default_bak_ref;
            }
            //checksum ref file
            for (j = 0; j < ref_grps; ++j)
            {
                ref_chksum = 0;
                for(i=0; i<ref_seg_len-2; i+=2)
                {
                    ref_chksum += ((refp[i + j * ref_seg_len]<<8) + refp[i + 1 + j * ref_seg_len]);
                }

                GTP_DEBUG("Reference chksum:0x%04X", ref_chksum&0xFF);
                tmp = ref_chksum + (refp[ref_seg_len + j * ref_seg_len -2]<<8) + refp[ref_seg_len + j * ref_seg_len -1];
                if(1 != tmp)
                {
                    GTP_DEBUG("Invalid checksum for reference, reset reference.");
                    memset(&refp[j * ref_seg_len], 0, ref_seg_len);
                    refp[ref_seg_len - 1 + j * ref_seg_len] = 0x01;
                }
                else
                {
                    if (j == (ref_grps - 1))
                    {
                        GTP_INFO("Reference data in %s used.", GTP_BAK_REF_PATH);
                    }
                }

            }
            ret = i2c_write_bytes(client, GTP_REG_BAK_REF, refp, ref_len);
            if(-1 == ret)
            {
                GTP_ERROR("Write ref i2c error.");
                ret = FAIL;
                goto exit_ref_proc;
            }
        }
        break;

    case GTP_BAK_REF_STORE:
        {
            ret = i2c_read_bytes(client, GTP_REG_BAK_REF, refp, ref_len);
            if(-1 == ret)
            {
                GTP_ERROR("Read ref i2c error.");
                ret = FAIL;
                goto exit_ref_proc;
            }
            flp->f_op->llseek(flp, 0, SEEK_SET);
            flp->f_op->write(flp, (char *)refp, ref_len, &flp->f_pos);
        }
        break;

    default:
        GTP_ERROR("Invalid Argument(%d) for backup reference", mode);
        ret = FAIL;
        goto exit_ref_proc;
    }

    ret = SUCCESS;
    goto exit_ref_proc;

default_bak_ref:
    for (j = 0; j < ref_grps; ++j)
    {
        memset(&refp[j * ref_seg_len], 0, ref_seg_len);
        refp[j * ref_seg_len + ref_seg_len - 1] = 0x01;  // checksum = 1
    }
    ret = i2c_write_bytes(client, GTP_REG_BAK_REF, refp, ref_len);
    if (flp && !IS_ERR(flp))
    {
        GTP_INFO("Write backup-reference data into %s", GTP_BAK_REF_PATH);
        flp->f_op->llseek(flp, 0, SEEK_SET);
        flp->f_op->write(flp, (char*)refp, ref_len, &flp->f_pos);
    }
    if (ret < 0)
    {
        GTP_ERROR("Failed to load the default backup reference");
        ret = FAIL;
    }
    else
    {
        ret = SUCCESS;
    }
exit_ref_proc:
    if (refp)
    {
        kfree(refp);
    }
    if (flp && !IS_ERR(flp))
    {
        filp_close(flp, NULL);
    }
    return ret;
}

u8 gtp_fw_startup(struct i2c_client *client)
{
    u8 wr_buf[4];
    s32 ret = 0;

    //init sw WDT
    wr_buf[0] = 0xAA;
    ret = i2c_write_bytes(client, 0x8041, wr_buf, 1);
    if (ret < 0)
    {
        GTP_ERROR("I2C error to firmware startup.");
        return FAIL;
    }
    //release SS51 & DSP
    wr_buf[0] = 0x00;
    i2c_write_bytes(client, 0x4180, wr_buf, 1);

    //int sync
    gtp_int_sync(25);

    //check fw run status
    i2c_read_bytes(client, 0x8041, wr_buf, 1);
    if(0xAA == wr_buf[0])
    {
        GTP_ERROR("IC works abnormally,startup failed.");
        return FAIL;
    }
    else
    {
        GTP_DEBUG("IC works normally,Startup success.");
        wr_buf[0] = 0xAA;
        i2c_write_bytes(client, 0x8041, wr_buf, 1);
        return SUCCESS;
    }
}


static void gtp_recovery_reset(struct i2c_client *client)
{
#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_OFF);
#endif
    force_reset_guitar(0);
#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_ON);
#endif
}

static u8 gtp_check_clk_legality(u8 *p_clk_buf)
{
    u8 i = 0;
    u8 clk_chksum = p_clk_buf[5];

    for(i = 0; i < 5; i++)
    {
        if((p_clk_buf[i] < 50) || (p_clk_buf[i] > 120) ||
            (p_clk_buf[i] != p_clk_buf[0]))
        {
            break;
        }
        clk_chksum += p_clk_buf[i];
    }

    if((i == 5) && (clk_chksum == 0))
    {
        GTP_DEBUG("Valid main clock data.");
        return SUCCESS;
    }
    GTP_ERROR("Invalid main clock data.");
    return FAIL;
}

static u8 gtp_main_clk_proc(struct i2c_client *client)
{
    s32 ret = 0;
    u8  i = 0;
    u8  clk_cal_result = 0;
    u8  clk_chksum = 0;
    u8  gtp_clk_buf[6] = {0};
    struct file *flp = NULL;

    GTP_DEBUG("[gtp_main_clk_proc]Waiting for FS %d", gtp_clk_retries);/*[PLATFORM]-MOD by falin.luo@tcl.com 2015/4/29*/
    if (/*gup_check_fs_mounted("/data") == FAIL*/!is_data_mounted)/*[PLATFORM]-MOD by falin.luo@tcl.com 2015/4/29*/
    {
        GTP_DEBUG("[gtp_main_clk_proc]/data not mounted");
        if(gtp_clk_retries++ < GTP_CHK_FS_MNT_MAX)
        {
        	msleep(100);/*[PLATFORM]-ADD by falin.luo@tcl.com 2015/4/29*/
            return FAIL;
        }
        else
        {
            GTP_ERROR("[gtp_main_clk_proc]Wait for file system timeout,need cal clk");
        }
    }
    else
    {
        GTP_DEBUG("[gtp_main_clk_proc]/data mounted !!!!");
        flp = filp_open(GTP_MAIN_CLK_PATH, O_RDWR | O_CREAT, 0666);
        if (!IS_ERR(flp))
        {
            flp->f_op->llseek(flp, 0, SEEK_SET);
            ret = flp->f_op->read(flp, (char *)gtp_clk_buf, 6, &flp->f_pos);
            if(ret > 0)
            {
                ret = gtp_check_clk_legality(gtp_clk_buf);
                if(SUCCESS == ret)
                {
                    GTP_DEBUG("[gtp_main_clk_proc]Open & read & check clk file success.");
                    goto send_main_clk;
                }
            }
        }
        GTP_ERROR("[gtp_main_clk_proc]Check clk file failed,need cal clk");
    }

    //cal clk
#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_OFF);
#endif
    clk_cal_result = gup_clk_calibration();
    force_reset_guitar(0);
    GTP_DEBUG("&&&&&&&&&&clk cal result:%d", clk_cal_result);

#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_ON);
#endif

    if(clk_cal_result < 50 || clk_cal_result > 120)
    {
        GTP_ERROR("Invalid main clock: %d", clk_cal_result);
        ret = FAIL;
        goto exit_clk_proc;
    }

    for(i = 0;i < 5; i++)
    {
        gtp_clk_buf[i] = clk_cal_result;
        clk_chksum += gtp_clk_buf[i];
    }
    gtp_clk_buf[5] = 0 - clk_chksum;

send_main_clk:

    ret = i2c_write_bytes(client, 0x8020, gtp_clk_buf, 6);

    if (flp && !IS_ERR(flp))
    {
        flp->f_op->llseek(flp, 0, SEEK_SET);
        flp->f_op->write(flp, (char *)gtp_clk_buf, 6, &flp->f_pos);
    }

    if(-1 == ret)
    {
        GTP_ERROR("[gtp_main_clk_proc]send main clk i2c error!");
        ret = FAIL;
    }
    else
    {
        ret = SUCCESS;
    }

exit_clk_proc:
    if (flp && !IS_ERR(flp))
    {
        filp_close(flp, NULL);
    }
    return ret;
}

u8 gtp_verify_hopping_buf(struct i2c_client *client)
{
    u16 checksum = 0;
    s32 i = 0;

    for (i = 0; i < 6; i += 2)
    {
        checksum += (gtp_hopping_buf[i] << 8) + gtp_hopping_buf[i+1];
    }
    if ((checksum + ((u16)(gtp_hopping_buf[6] << 8) + (u16)gtp_hopping_buf[7])) & 0xFFFF)
    {
        GTP_ERROR("Wrong checksum for hopping buffer, get hopping data from config instead!");
        goto get_config_hopping;
    }

    for (i = 0; i < 8; i++)
    {
        if (gtp_hopping_buf[i] != gtp_hopping_buf[i+8])
        {
            GTP_ERROR("Hopping buffer is not identical, get data from config instead!");
            goto get_config_hopping;
        }
    }
    GTP_DEBUG("Hopping buffer is okay");
    return SUCCESS;

get_config_hopping:
    memcpy(gtp_hopping_buf, &config[2 + 0x8065 - GTP_REG_CONFIG_DATA], 4);
    gtp_hopping_buf[4] = 0xAA;
    gtp_hopping_buf[5] = 0x55;

    checksum = 0;
    for (i = 0; i < 6; i += 2)
    {
        checksum += (gtp_hopping_buf[i] << 8) + gtp_hopping_buf[i+1];
    }
    checksum = 0 - checksum;
    gtp_hopping_buf[6] = (u8)(checksum >> 8);
    gtp_hopping_buf[7] = (u8)(checksum & 0xFF);

    for (i = 0; i < 8; i++)
    {
        gtp_hopping_buf[i+8] = gtp_hopping_buf[i];
    }
    return SUCCESS;
}

u8 gtp_hopping_proc(struct i2c_client *client, s32 mode)
{
    s32 ret = 0;

    GTP_DEBUG("Store hopping data, wait for /data mounted.");
	/*[PLATFORM]-MOD-BEGIN by falin.luo@tcl.com 2015/4/29*/
//    ret = gup_check_fs_mounted("/data");
	ret = is_data_mounted ? SUCCESS : FAIL;
	/*[PLATFORM]-MOD-END by falin.luo@tcl.com 2015/4/29*/

    if (FAIL == ret)
    {
        GTP_DEBUG("/data not mounted.");
        return FAIL;
    }
    GTP_DEBUG("/data Mounted!");

    if (GTP_HOPPING_SEND == mode)
    {
        gtp_verify_hopping_buf(client);

        ret = i2c_write_bytes(client, 0x8030, gtp_hopping_buf, 16);
        if (ret < 0)
        {
            return FAIL;
        }
        else
        {
            return SUCCESS;
        }
    }
    else
    {
        ret = i2c_read_bytes(client, 0x8030, gtp_hopping_buf, 16);

        if (ret < 0)
        {
            GTP_ERROR("Failed to read hopping data from hopping buffer, get from config instead.");
            return FAIL;
        }
        return gtp_verify_hopping_buf(client);
    }
}

#if GTP_FL_LITTLE_SYSTEM
s32 gtp_resume_timeout(void *none)
{
    s32 timeout = 0;
    GTP_DEBUG("Resume timeout thread kicks off.");

    while (timeout++ < (10 * 10))
    {
        msleep(100);
        if (!little_fw_mode)
        {
            GTP_DEBUG("Resume timeout thread terminated while counting.");
            return 0;
        }
    }

    if (little_fw_mode)
    {
        GTP_INFO("Download big ss51 firmware timeout, process esd reset.");
        little_fw_mode = 0;
        gtp_recovery_reset(i2c_client_point);
    }

    return 0;
}

u8 gtp_get_state1(struct i2c_client *client)
{
    u8 state1 = 0;
    s32 ret = 0;

    ret = i2c_read_bytes(client, GTP_REG_STATE1, &state1, 1);

    if (ret < 0)
    {
        GTP_ERROR("Failed to get state1!");
        return 0xFF;
    }
    return state1;
}

u8 gtp_get_state2(struct i2c_client *client)
{
    u8 state2 = 0;
    s32 ret = 0;

    ret = i2c_read_bytes(client, GTP_REG_STATE2, &state2, 1);

    if (ret < 0)
    {
        GTP_ERROR("Failed to get state2!");
        return 0xFF;
    }
    return state2;
}

// size: k in unit
u8 gtp_send_check_info(struct i2c_client *client, u16 fw_chksum)
{
    s32 ret = 0;
    u8 checksum = 0;
    u8 bank = 0;
    u8 state1 = 0x00;
    u8 state2 = 0x00;
    u16 start_addr = 0x0000;
    u8 checkinfo_buf[10] = {0};


    switch (fw_block)
    {
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
        bank = 0x01;
        state1 = BUFFER_FULL;
        state2 = IS_A_SEG;
        start_addr = (0xC000 + (fw_block - 1) * 1024 * 2);
        break;

    case 9:
        bank = 0x00;
        state1 = BUFFER_FULL;
        state2 = IS_A_SEG;
        start_addr = 0xF800;
        break;

    case 10:
        bank = 0x00;
        state1 = BUFFER_FULL;
        state2 = IS_A_SEG;
        start_addr = 0xF000;
        break;

    case 11:
        bank = 0x00;
        state1 = BUFFER_FULL;
        state2 = IS_B_SEG_FIRST;
        start_addr = 0xE800;
        break;

    case 12:
        bank = 0x00;
        state1 = WAIT_CHECK;
        state2 = IS_B_SEG_LAST;
        start_addr = 0xC000;
        break;

    default:
        GTP_ERROR("Invalid firmware block(%d) for checkinfo.", fw_block);
        return FAIL;
    }

    checkinfo_buf[3] = bank;
    checkinfo_buf[4] = (u8)(start_addr >> 8);
    checkinfo_buf[5] = (u8)(start_addr & 0xFF);
    checkinfo_buf[7] = state2;
    checkinfo_buf[8] = state1;

    GTP_DEBUG("fw_block: %d, fw_chksum: 0x%04X", fw_block, fw_chksum);
    if (fw_block == 12)
    {
        checkinfo_buf[6] = 80;
    }
    else
    {
        checkinfo_buf[6] = 16;
    }

    checkinfo_buf[0] = (u8)(fw_chksum >> 8);
    checkinfo_buf[1] = (u8)(fw_chksum & 0xFF);

    checksum = 0;
    for (ret = 0; ret < 6; ret++)
    {
        checksum += checkinfo_buf[3 + ret];
    }
    checkinfo_buf[2] = 1 - checksum;

    ret = i2c_write_bytes(client, GTP_REG_CHECKINFO, checkinfo_buf, 9);
    if (ret < 0)
    {
        GTP_ERROR("Failed to send checkinfo!");
        return FAIL;
    }
    else
    {
        GTP_DEBUG("Send checkinfo successfully!");
    }
    return SUCCESS;
}



u8 gtp_resume_fw_startup(struct i2c_client *client)
{
    u8 buf;
    u32 retry = 0;
    s32 ret = 0;

    GTP_INFO("Big ss51 firmware startup.");

    while (retry++ < 10)
    {
        buf = 0x0C;
        ret = i2c_write_bytes(client, 0x4180, &buf, 1);   // hold ss51 & dsp
        if (ret < 0)
        {
            GTP_ERROR("Failed to hold ss51 & dsp.");
            return FAIL;
        }

        buf = 0x00;
        ret = i2c_read_bytes(client, 0x4180, &buf, 1);
        if (ret < 0)
        {
            GTP_ERROR("Failed to get hold ss51 & dsp status.");
            return FAIL;
        }

        if (buf == 0x0C)
        {
            GTP_DEBUG("SS51 & Dsp confirm hold!");
            break;
        }
    }

    if (retry >= 10)
    {
        GTP_ERROR("Hold ss51 & dsp retry exhausted.");
        return FAIL;
    }

    buf = 0x03;
    i2c_write_bytes(client, 0x4048, &buf, 1);   // select bank3

    buf = 0x00;
    i2c_write_bytes(client, 0x4049, &buf, 1);

    return gtp_fw_startup(client);
}

u8 gtp_download_seg_b(struct i2c_client *client)
{
    s32 ret = 0;
    u16 fw_chksum = 0;
    u8 state1 = 0;

    if (block_section != 10)
    {
        state1 = 0x00;
        i2c_write_bytes(client, 0x4048, &state1, 1);        // select bank0

        ret = gup_burn_ss51_seg_b(client, 10, &fw_chksum);

        if (FAIL == ret)
        {
            GTP_ERROR("Failed to burn ss51 seg B, process reburn.");
            return FAIL;
        }

        ret = gtp_send_check_info(i2c_client_point, fw_chksum);
        if (FAIL == ret)
        {
            GTP_ERROR("Send checkinfo failed, process resend.");
            return FAIL;
        }
    }
    msleep(1);

    state1 = gtp_get_state1(client);

    if (CHECK_COMPLETE == state1)
    {
        GTP_INFO("Burn ss51 Block12 successfully");
    }
    else if (CHECK_ERROR == state1)
    {
        GTP_DEBUG("Big SS51 Seg B check error, process reburn!");
        return FAIL;
    }
    else
    {
        GTP_ERROR("Big SS51 Seg B check imcomplete(state1:%s), process recheck.", symbolic_state1[state1&0x07]);
        block_section = 10;
        return FAIL;
    }
    ret = gtp_resume_fw_startup(client);

    little_fw_mode = 0;
    if (FAIL == ret)
    {
        GTP_ERROR("Big ss51 firmware startup failed, process esd reset.");
        if (!tpd_halt)
        {
            gtp_recovery_reset(i2c_client_point);
        }
    }
    else
    {
        GTP_INFO("Switch to big ss51 firmware successfully!");
    }
#if GTP_ESD_PROTECT
    if (!tpd_halt)
    {
        gtp_esd_switch(client, SWITCH_ON);
    }
#endif
    return SUCCESS;
}


u8 gtp_download_big_ss51(struct i2c_client *client)
{
    u8 state1, state2;
    s32 ret = 0;
    s32 i = 0;
    static u16 fw_chksum = 0x0000;

    //GTP_DEBUG("Block: %d, Block Section: %d", fw_block, block_section);
    if (!little_fw_mode)
    {
        GTP_ERROR("Download big ss51 timeout!");
        return FAIL;
    }

    if (block_section == 10)     // one firmware block burned
    {
        if (fw_block == 11)
        {
            mdelay(3);
        }

        state1 = gtp_get_state1(i2c_client_point);
        state2 = gtp_get_state2(i2c_client_point);

        GTP_DEBUG("state1: %02X (%s), state2: %02X (%s)", state1, symbolic_state1[state1&0x07], state2, symbolic_state2[state2&0x07]);

        if (CHECK_COMPLETE == state1)
        {
            block_section = 1;
            GTP_DEBUG("Burn ss51 Block%d successfully!", fw_block);
            if (fw_block == 11)
            {
            #if GTP_ESD_PROTECT
                gtp_esd_switch(client, SWITCH_OFF);
            #endif
                tpd_up(0, 0, 0);      // release all
                input_sync(tpd->dev);
                GTP_DEBUG("Release touch manually.");
                fw_block = 12;
                for (i = 0; i < 5; i++)
                {
                    if (!little_fw_mode || (fw_block != 12))
                    {
                        GTP_ERROR("Download big ss51 timeout!");
                        return FAIL;
                    }
                    ret = gtp_download_seg_b(client);

                    if (SUCCESS == ret)
                    {
                        break;
                    }
                }
                return SUCCESS;
            }
        }
        else if (CHECK_ERROR == state1)
        {
            GTP_ERROR("Block%d check error, process reburn.", fw_block);
            block_section = 1;
            fw_block--;
        }
        else
        {
            GTP_DEBUG("Block%d check incomplete, process recheck.", fw_block);
            block_section = 10;
        }
    }
    if (block_section < 9)
    {
        ret = gup_burn_ss51_block(i2c_client_point, block_section, fw_block+1, &fw_chksum);
        if (FAIL == ret)
        {
            GTP_ERROR("Burn block%d section%d failed, reburn block%d", fw_block+1, block_section, fw_block+1);
        }
        else
        {
            block_section++;
        }
        if (block_section == 9)       // one firmware block downloaded
        {
            fw_block++;
            block_section = 10;
            ret = gtp_send_check_info(i2c_client_point, fw_chksum);
        }
    }
    return ret;
}
#endif

#endif
//************* For GT9XXF End **********************//

#if GTP_WITH_PEN
static void gtp_pen_init(void)
{
    s32 ret = 0;

    pen_dev = input_allocate_device();
    if (pen_dev == NULL)
    {
        GTP_ERROR("Failed to allocate input device for pen/stylus.");
        return;
    }

    pen_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
    pen_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

    set_bit(BTN_TOOL_PEN, pen_dev->keybit);
    set_bit(INPUT_PROP_DIRECT, pen_dev->propbit);
    //set_bit(INPUT_PROP_POINTER, pen_dev->propbit);

#if GTP_PEN_HAVE_BUTTON
    input_set_capability(pen_dev, EV_KEY, BTN_STYLUS);
    input_set_capability(pen_dev, EV_KEY, BTN_STYLUS2);
#endif

    input_set_abs_params(pen_dev, ABS_MT_POSITION_X, 0, TPD_RES_X, 0, 0);
    input_set_abs_params(pen_dev, ABS_MT_POSITION_Y, 0, TPD_RES_Y, 0, 0);
    input_set_abs_params(pen_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
    input_set_abs_params(pen_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(pen_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

    pen_dev->name = "mtk-pen";
    pen_dev->phys = "input/ts";
    pen_dev->id.bustype = BUS_I2C;

    ret = input_register_device(pen_dev);
    if (ret)
    {
        GTP_ERROR("Register %s input device failed", pen_dev->name);
        return;
    }
}

static void gtp_pen_down(s32 x, s32 y, s32 size, s32 id)
{
    input_report_key(pen_dev, BTN_TOOL_PEN, 1);
    input_report_key(pen_dev, BTN_TOUCH, 1);
    input_report_abs(pen_dev, ABS_MT_POSITION_X, x);
    input_report_abs(pen_dev, ABS_MT_POSITION_Y, y);
    if ((!size) && (!id))
    {
        input_report_abs(pen_dev, ABS_MT_PRESSURE, 100);
        input_report_abs(pen_dev, ABS_MT_TOUCH_MAJOR, 100);
    }
    else
    {
        input_report_abs(pen_dev, ABS_MT_PRESSURE, size);
        input_report_abs(pen_dev, ABS_MT_TOUCH_MAJOR, size);
        input_report_abs(pen_dev, ABS_MT_TRACKING_ID, id);
    }
    input_mt_sync(pen_dev);
}

static void gtp_pen_up(void)
{
    input_report_key(pen_dev, BTN_TOOL_PEN, 0);
    input_report_key(pen_dev, BTN_TOUCH, 0);
}
#endif


static s32 tpd_i2c_probe_next(struct i2c_client *client)
{
    s32 err = 0;
    s32 ret = 0;

    u16 version_info;


    ret = tpd_power_on(client);

    if (ret < 0)
    {
        GTP_ERROR("I2C communication ERROR!");
    }

    ret = gtp_read_version(client, &version_info);

    if (ret < 0)
    {
        GTP_ERROR("Read version failed.");
    }

    ret = gtp_init_panel(client);

    if (ret < 0)
    {
        GTP_ERROR("GTP init panel failed.");
    }

#if GTP_CREATE_WR_NODE
    init_wr_node(client);
#endif
    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);


    if (IS_ERR(thread))
    {
        err = PTR_ERR(thread);
        GTP_ERROR(TPD_DEVICE " failed to create kernel thread: %d\n", err);
    }

    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

#if GTP_AUTO_UPDATE
    ret = gup_init_update_proc(client);

    if (ret < 0)
    {
        GTP_ERROR("Create update thread error.");
    }
#endif

#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_ON);
#endif


    return 0;
}

static ssize_t cfg_load_enable_store(struct device *dev,
                    struct device_attribute *attr,
                        const char *buf, size_t count)
{
	tpd_i2c_probe_next(i2c_client_point);	
	return count;
}
static ssize_t cfg_load_enable_show(struct device *dev,
                    struct device_attribute *attr,
                        const char *buf, size_t count)
{
	return count;
}
static  DEVICE_ATTR(cfg_load_enable, S_IRUGO|S_IWUSR, cfg_load_enable_show, cfg_load_enable_store);

/*[PLATFORM]-Add-BEGIN by falin.luo@tcl.com 2015/4/29*/
/*sys interface to get and set the data partition mount status*/
#ifdef GTP_COMPATIBLE_MODE

static ssize_t data_is_mount_store(struct device *dev,
                    struct device_attribute *attr,
                        const char *buf, size_t count)
{
	GTP_INFO("enter %s", __func__);

	
	is_data_mounted = ((buf[0] == '1') ? 1 : 0);

	GTP_INFO("is_data_mount = %d, buf = %s", is_data_mounted, buf);

	return count;
}
static ssize_t data_is_mount_show(struct device *dev,
                    struct device_attribute *attr,
                        char *buf, size_t count)
{
	return snprintf(buf, PAGE_SIZE, "is_data_mounted = %d\n", is_data_mounted);
}

static DEVICE_ATTR(data_is_mount, 0644, data_is_mount_show, data_is_mount_store);

#endif

/*[PLATFORM]-Add-END by falin.luo@tcl.com 2015/4/29*/


static struct miscdevice cfg_misc_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "tp_cfg",
   // .fops = &tpd_fops,
};

#if 0
static void buttons_timer_function(unsigned long data)
{
    printk(KERN_ERR"caoyang1\n");
    input_report_key(tpd->dev, KEY_POWER, 1);
	input_sync(tpd->dev);
	input_report_key(tpd->dev, KEY_POWER, 0);
	input_sync(tpd->dev);
	tpd_sleep_flag = 1;

}
#endif

static s32 tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 err = 0;
    s32 ret = 0;

    u16 version_info;
#if GTP_HAVE_TOUCH_KEY
    s32 idx = 0;
#endif
#ifdef TPD_PROXIMITY
    struct hwmsen_object obj_ps;
#endif

    i2c_client_point = client;
//add by caoyang for ·À¶¶
//do not use the timer because it cause bug
#if 0
    init_timer(&buttons_timer);
	buttons_timer.function = buttons_timer_function;
	add_timer(&buttons_timer);
#endif
#if 0
    ret = tpd_power_on(client);

    if (ret < 0)
    {
        GTP_ERROR("I2C communication ERROR!");
    }
#endif
#ifdef VELOCITY_CUSTOM

    if ((err = misc_register(&tpd_misc_device)))
    {
        printk("mtk_tpd: tpd_misc_device register failed\n");
    }

#endif
#if 0
    ret = gtp_read_version(client, &version_info);

    if (ret < 0)
    {
        GTP_ERROR("Read version failed.");
    }

    ret = gtp_init_panel(client);

    if (ret < 0)
    {
        GTP_ERROR("GTP init panel failed.");
    }
#endif
    // Create proc file system
    gt91xx_config_proc = proc_create(GT91XX_CONFIG_PROC_FILE, 0666, NULL, &config_proc_ops);
    if (gt91xx_config_proc == NULL)
    {
        GTP_ERROR("create_proc_entry %s failed\n", GT91XX_CONFIG_PROC_FILE);
    }
    else
    {
        GTP_INFO("create proc entry %s success", GT91XX_CONFIG_PROC_FILE);
    }
#if 0
#if GTP_CREATE_WR_NODE
    init_wr_node(client);
#endif

    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);


    if (IS_ERR(thread))
    {
        err = PTR_ERR(thread);
        GTP_ERROR(TPD_DEVICE " failed to create kernel thread: %d\n", err);
    }
#endif


#if GTP_HAVE_TOUCH_KEY

    for (idx = 0; idx < GTP_MAX_KEY_NUM; idx++)
    {
        input_set_capability(tpd->dev, EV_KEY, touch_key_array[idx]);
    }

#endif
    input_set_capability(tpd->dev, EV_KEY, KEY_POWER);

#if GTP_WITH_PEN
    gtp_pen_init();
#endif
    // set INT mode
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);

    msleep(50);

#if 1
    if (!int_type)  //EINTF_TRIGGER
    {
        mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_RISING, tpd_eint_interrupt_handler, 1);
    }
    else
    {
        mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
    }

#endif

#if 0
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

#if GTP_AUTO_UPDATE
    ret = gup_init_update_proc(client);

    if (ret < 0)
    {
        GTP_ERROR("Create update thread error.");
    }
#endif
#endif
#ifdef TPD_PROXIMITY
    //obj_ps.self = cm3623_obj;
    obj_ps.polling = 0;         //0--interrupt mode;1--polling mode;
    obj_ps.sensor_operate = tpd_ps_operate;

    if ((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
    {
        GTP_ERROR("hwmsen attach fail, return:%d.", err);
    }

#endif
#if 0
#if GTP_ESD_PROTECT
    gtp_esd_switch(client, SWITCH_ON);
#endif

    tpd_load_status = 1;
#endif
	misc_register(&cfg_misc_device);
/*[PLATFORM]-Add-BEGIN by falin.luo@tcl.com 2015/4/29*/
#ifdef GTP_COMPATIBLE_MODE
	device_create_file(cfg_misc_device.this_device, &dev_attr_data_is_mount);
#endif
/*[PLATFORM]-Add-END by falin.luo@tcl.com 2015/4/29*/

	device_create_file(cfg_misc_device.this_device, &dev_attr_cfg_load_enable);
    tpd_load_status = 1;
    return 0;
}


static void tpd_eint_interrupt_handler(void)
{
    TPD_DEBUG_PRINT_INT;

    tpd_flag = 1;

    wake_up_interruptible(&waiter);
}
static int tpd_i2c_remove(struct i2c_client *client)
{
#if GTP_CREATE_WR_NODE
    uninit_wr_node();
#endif

#if GTP_ESD_PROTECT
    destroy_workqueue(gtp_esd_check_workqueue);
#endif

    return 0;
}
#if (GTP_ESD_PROTECT || GTP_COMPATIBLE_MODE)
static s32 force_reset_guitar(s32 resume)
{
    s32 i = 0;
    s32 ret = 0;

    if (!resume)
    {
        GTP_INFO("Force_reset_guitar");
    }
    else
    {
        GTP_INFO("Download little system.");
    }

    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

    GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
    GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
#ifdef MT6573
    //Power off TP
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
    msleep(30);
    //Power on TP
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
    msleep(30);
#else           // ( defined(MT6575) || defined(MT6577) || defined(MT6589) )
    // Power off TP
    #ifdef TPD_POWER_SOURCE_CUSTOM
        hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
    #else
        hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
    #endif
        msleep(30);

    // Power on TP
    #ifdef TPD_POWER_SOURCE_CUSTOM
        hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
    #else
        hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
    #endif
        msleep(30);

#endif

    for (i = 0; i < 5; i++)
    {
    #if GTP_COMPATIBLE_MODE
        if (CHIP_TYPE_GT9F == gtp_chip_type)
        {
        #if GTP_FL_LITTLE_SYSTEM
            //check code ram
            if (resume)         // poweroff resume
            {
                ret = gup_fw_download_proc(NULL, GTP_FL_PWR_RESUME_BURN);
                if (FAIL == ret)
                {
                    GTP_ERROR("Failed to download little system.");
                    continue;
                }
            }
            else
        #endif
            {
                ret = gup_fw_download_proc(NULL, GTP_FL_ESD_RECOVERY);
            }
            if(FAIL == ret)
            {
                GTP_ERROR("[force_reset_guitar]Check & repair fw failed.");
                continue;
            }
            //startup fw
            ret = gtp_fw_startup(i2c_client_point);
            if(FAIL == ret)
            {
                if (resume)
                {
                    GTP_ERROR("Failed to startup little system.");
                }
                else
                {
                    GTP_ERROR("GT9XXF start up failed.");
                }
                continue;
            }
            break;
        }
        else
    #endif
        {
            //Reset Guitar
            gtp_reset_guitar(i2c_client_point, 20);
            msleep(50);
            //Send config
            ret = gtp_send_cfg(i2c_client_point);

            if (ret < 0)
            {
                continue;
            }
        }
        break;
    }

    if (i >= 5)
    {
        if (resume)
        {
            GTP_ERROR("Failed to download little system.");
        }
        else
        {
            GTP_ERROR("Failed to reset guitar.");
        }
        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        return FAIL;
    }
 #if (GTP_COMPATIBLE_MODE && GTP_FL_LITTLE_SYSTEM)
    if (CHIP_TYPE_GT9F == gtp_chip_type)
    {
        if (resume)
        {
            little_fw_mode = 1;
            fw_block = 0;
            block_section = 1;

            msleep(1);
            gtp_main_clk_proc(i2c_client_point);
            gtp_hopping_proc(i2c_client_point, GTP_HOPPING_SEND);
            gtp_bak_ref_proc(i2c_client_point, GTP_BAK_REF_SEND);
        }
        else
        {
            little_fw_mode = 0;
            fw_block = 0;
            block_section = 0;
        }
    }
#endif

    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    return SUCCESS;
}
#endif

#if GTP_ESD_PROTECT
static s32 gtp_init_ext_watchdog(struct i2c_client *client)
{
    u8 opr_buffer[2] = {0xAA};
    GTP_DEBUG("Init external watchdog.");
    return i2c_write_bytes(client, 0x8041, opr_buffer, 1);
}

void gtp_esd_switch(struct i2c_client *client, s32 on)
{
    spin_lock(&esd_lock);
    if (SWITCH_ON == on)     // switch on esd
    {
        if (!esd_running)
        {
            esd_running = 1;
            spin_unlock(&esd_lock);
            GTP_INFO("Esd started");
            queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, clk_tick_cnt);
        }
        else
        {
            spin_unlock(&esd_lock);
        }
    }
    else    // switch off esd
    {
        if (esd_running)
        {
            esd_running = 0;
            spin_unlock(&esd_lock);
            GTP_INFO("Esd cancelled");
            cancel_delayed_work_sync(&gtp_esd_check_work);
        }
        else
        {
            spin_unlock(&esd_lock);
        }
    }
}

static s32 gtp_check_fw(void)
{
    u8 versionBuff[4] = {0};
    s32 ret,retry =  0;

    while(retry++ < 3)
    {
        ret = i2c_read_bytes_non_dma(i2c_client_point, 0x8140, versionBuff, sizeof(versionBuff));
        if(ret < 0)
        {
        	continue;
        }
        if( memcmp(versionBuff, &gtp_default_FW_fl[4], 4) !=0 )
        {
        	continue;
        }

        return 1;
    }

    GTP_ERROR("Check running fw version error!");
    return 0;
}

static void gtp_esd_check_func(struct work_struct *work)
{
    s32 i = 0;
    s32 ret = -1;
    u8 esd_buf[3] = {0x00};

    if (tpd_halt)
    {
        GTP_INFO("Esd suspended!");
        return;
    }
    for (i = 0; i < 3; i++)
    {
        ret = i2c_read_bytes_non_dma(i2c_client_point, 0x8040, esd_buf, 2);

        GTP_DEBUG("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X", esd_buf[0], esd_buf[1]);
        if (ret < 0)
        {
            // IIC communication problem
            continue;
        }
        else
        {
            if ((esd_buf[0] == 0xAA) || (esd_buf[1] != 0xAA))
            {
                u8 chk_buf[2] = {0x00};
                i2c_read_bytes_non_dma(i2c_client_point, 0x8040, chk_buf, 2);

                GTP_DEBUG("[Check]0x8040 = 0x%02X, 0x8041 = 0x%02X", chk_buf[0], chk_buf[1]);

                if ( (chk_buf[0] == 0xAA) || (chk_buf[1] != 0xAA) )
                {
                    i = 3;          // jump to reset guitar
                    break;
                }
                else
                {
                    continue;
                }
            }
            else
            {
                // IC works normally, Write 0x8040 0xAA, feed the watchdog
                esd_buf[0] = 0xAA;
                i2c_write_bytes_non_dma(i2c_client_point, 0x8040, esd_buf, 1);

                break;
            }
        }
    }

    if (i >= 3 || !gtp_check_fw())
    {
    #if GTP_COMPATIBLE_MODE
        if ((CHIP_TYPE_GT9F == gtp_chip_type) && (1 == rqst_processing))
        {
            GTP_INFO("Request Processing, no reset guitar.");
        }
        else
    #endif
        {
            GTP_INFO("IC works abnormally! Process reset guitar.");
            esd_buf[0] = 0x01;
            esd_buf[1] = 0x01;
            esd_buf[2] = 0x01;
            i2c_write_bytes(i2c_client_point, 0x4226, esd_buf, 3);
            msleep(50);
            force_reset_guitar(0);
        }
    }

    if (!tpd_halt)
    {
        queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, clk_tick_cnt);
    }
    else
    {
        GTP_INFO("Esd suspended!");
    }

    return;
}
#endif

static void tpd_down(s32 x, s32 y, s32 size, s32 id)
{
    if ((!size) && (!id))
    {
        input_report_abs(tpd->dev, ABS_MT_PRESSURE, 100);
        input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 100);
    }
    else
    {
        input_report_abs(tpd->dev, ABS_MT_PRESSURE, size);
        input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, size);
        /* track id Start 0 */
        input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
    }

    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    input_mt_sync(tpd->dev);
    TPD_EM_PRINT(x, y, x, y, id, 1);

#if (defined(MT6575)||defined(MT6577))

    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {
        tpd_button(x, y, 1);
    }

#endif
}

static void tpd_up(s32 x, s32 y, s32 id)
{
    input_report_key(tpd->dev, BTN_TOUCH, 0);
    input_mt_sync(tpd->dev);
    TPD_EM_PRINT(x, y, x, y, id, 0);

#if (defined(MT6575) || defined(MT6577))

    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {
        tpd_button(x, y, 0);
    }

#endif
}
#if GTP_CHARGER_SWITCH
static void gtp_charger_switch(s32 dir_update)
{
    u32 chr_status = 0;
    u8 chr_cmd[3] = {0x80, 0x40};
    static u8 chr_pluggedin = 0;
    int ret = 0;

#ifdef MT6573
    chr_status = *(volatile u32 *)CHR_CON0;
    chr_status &= (1 << 13);
#else   // ( defined(MT6575) || defined(MT6577) || defined(MT6589) )
    chr_status = upmu_is_chr_det();
#endif

    if (chr_status)     // charger plugged in
    {
        if (!chr_pluggedin || dir_update)
        {
            chr_cmd[2] = 6;
            ret = gtp_i2c_write(i2c_client_point, chr_cmd, 3);
            if (ret > 0)
            {
                GTP_INFO("Update status for Charger Plugin");
            }
            chr_pluggedin = 1;
        }
    }
    else            // charger plugged out
    {
        if (chr_pluggedin || dir_update)
        {
            chr_cmd[2] = 7;
            ret = gtp_i2c_write(i2c_client_point, chr_cmd, 3);
            if (ret > 0)
            {
                GTP_INFO("Update status for Charger Plugout");
            }
            chr_pluggedin = 0;
        }
    }
}
#endif

static int touch_event_handler(void *unused)
{
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
    u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
    u8  touch_num = 0;
    u8  finger = 0;
    static u8 pre_touch = 0;
    static u8 pre_key = 0;
#if GTP_WITH_PEN
    u8 pen_active = 0;
    static u8 pre_pen = 0;
#endif
    u8  key_value = 0;
    u8 *coor_data = NULL;
    s32 input_x = 0;
    s32 input_y = 0;
    s32 input_w = 0;
    s32 id = 0;
    s32 i  = 0;
    s32 ret = -1;

#if GTP_COMPATIBLE_MODE
    u8  rqst_data[3] = {(u8)(GTP_REG_RQST >> 8), (u8)(GTP_REG_RQST & 0xFF), 0};
#endif

#ifdef TPD_PROXIMITY
    s32 err = 0;
    hwm_sensor_data sensor_data;
    u8 proximity_status;
#endif
    u8 proximity_status;        //add by caoyang
    u8 doze_buf[3] = {0x81, 0x4B};

    sched_setscheduler(current, SCHED_RR, &param);
    do
    {
        set_current_state(TASK_INTERRUPTIBLE);

        while (tpd_halt)
        {
        //#if GTP_GESTURE_WAKEUP
        if(gesture_value > 0){
            if (DOZE_ENABLED == doze_status)
            {
                break;
            }
        //#endif
        }
            tpd_flag = 0;
            msleep(20);
        }

        wait_event_interruptible(waiter, tpd_flag != 0);
        tpd_flag = 0;
        TPD_DEBUG_SET_TIME;
        set_current_state(TASK_RUNNING);

    #if GTP_CHARGER_SWITCH
        gtp_charger_switch(0);
    #endif

    if(gesture_value > 0){
    //#if GTP_GESTURE_WAKEUP
        if (DOZE_ENABLED == doze_status)
        {
            ret = gtp_i2c_read(i2c_client_point, doze_buf, 3);
            GTP_DEBUG("0x814B = 0x%02X", doze_buf[2]);
            if (ret > 0)
            {
                if ((doze_buf[2] == 'a') || (doze_buf[2] == 'b') || (doze_buf[2] == 'c') ||
                    (doze_buf[2] == 'd') || (doze_buf[2] == 'e') || (doze_buf[2] == 'g') ||
                    (doze_buf[2] == 'h') || (doze_buf[2] == 'm') || (doze_buf[2] == 'o') ||
                    (doze_buf[2] == 'q') || (doze_buf[2] == 's') || (doze_buf[2] == 'v') ||
                    (doze_buf[2] == 'w') || (doze_buf[2] == 'y') || (doze_buf[2] == 'z') ||
                    (doze_buf[2] == 0x5E) /* ^ */
                    )
                {
                    if (doze_buf[2] != 0x5E)
                    {
                        GTP_INFO("Wakeup by gesture(%c), light up the screen!", doze_buf[2]);
                    }
                    else
                    {
                        GTP_INFO("Wakeup by gesture(^), light up the screen!");
                    }

                    doze_status = DOZE_WAKEUP;
                    input_report_key(tpd->dev, KEY_POWER, 1);
                    input_sync(tpd->dev);
                    input_report_key(tpd->dev, KEY_POWER, 0);
                    input_sync(tpd->dev);
                    // clear 0x814B
                    doze_buf[2] = 0x00;
                    gtp_i2c_write(i2c_client_point, doze_buf, 3);
                }
                else if ( (doze_buf[2] == 0xAA) || (doze_buf[2] == 0xBB) ||
                    (doze_buf[2] == 0xAB) || (doze_buf[2] == 0xBA) )
                {
                    char *direction[4] = {"Right", "Down", "Up", "Left"};
                    u8 type = ((doze_buf[2] & 0x0F) - 0x0A) + (((doze_buf[2] >> 4) & 0x0F) - 0x0A) * 2;

                    GTP_INFO("%s slide to light up the screen!", direction[type]);
                    doze_status = DOZE_WAKEUP;
                    input_report_key(tpd->dev, KEY_POWER, 1);
                    input_sync(tpd->dev);
                    input_report_key(tpd->dev, KEY_POWER, 0);
                    input_sync(tpd->dev);
                    // clear 0x814B
                    doze_buf[2] = 0x00;
                    gtp_i2c_write(i2c_client_point, doze_buf, 3);
                }
                else if (0xCC == doze_buf[2])
                {
                    GTP_INFO("Double click to light up the screen!");
                    doze_status = DOZE_WAKEUP;
                    input_report_key(tpd->dev, KEY_POWER, 1);
                    input_sync(tpd->dev);
                    input_report_key(tpd->dev, KEY_POWER, 0);
                    input_sync(tpd->dev);
                    // clear 0x814B
                    doze_buf[2] = 0x00;
                    gtp_i2c_write(i2c_client_point, doze_buf, 3);
                }
                else
                {
                    // clear 0x814B
                    doze_buf[2] = 0x00;
                    gtp_i2c_write(i2c_client_point, doze_buf, 3);
                    gtp_enter_doze(i2c_client_point);
                }
            }
            continue;
        }
    //#endif
    }
        ret = gtp_i2c_read(i2c_client_point, point_data, 12);
        if (ret < 0)
        {
            GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
            continue;
        }
        finger = point_data[GTP_ADDR_LENGTH];

    #if GTP_COMPATIBLE_MODE
        if ((finger == 0x00) && (CHIP_TYPE_GT9F == gtp_chip_type))
        {
            ret = gtp_i2c_read(i2c_client_point, rqst_data, 3);

            if(ret < 0)
            {
                GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
                continue;
            }
            switch (rqst_data[2])
            {
                case GTP_RQST_BAK_REF:
                    GTP_INFO("Request Ref.");
                    rqst_processing = 1;
                    ret = gtp_bak_ref_proc(i2c_client_point, GTP_BAK_REF_SEND);
                    if(SUCCESS == ret)
                    {
                        GTP_INFO("Send ref success.");
                        rqst_data[2] = GTP_RQST_RESPONDED;
                        gtp_i2c_write(i2c_client_point, rqst_data, 3);
                        rqst_processing = 0;
                    }
                    goto exit_work_func;

                case GTP_RQST_CONFIG:
                    GTP_INFO("Request Config.");
                    ret = gtp_send_cfg(i2c_client_point);
                    if (ret < 0)
                    {
                        GTP_ERROR("Send config error.");
                    }
                    else
                    {
                        GTP_INFO("Send config success.");
                        rqst_data[2] = GTP_RQST_RESPONDED;
                        gtp_i2c_write(i2c_client_point, rqst_data, 3);
                    }
                    goto exit_work_func;

                case GTP_RQST_MAIN_CLOCK:
                    GTP_INFO("Request main clock.");
                    rqst_processing = 1;
                    ret = gtp_main_clk_proc(i2c_client_point);
                    if(SUCCESS == ret)
                    {
                        GTP_INFO("Send main clk success.");
                    #if GTP_POWER_CTRL_SLEEP
                        {
                            ret = gtp_hopping_proc(i2c_client_point, GTP_HOPPING_SEND);
                            if (FAIL == ret)
                            {
                                GTP_ERROR("Failed to send hopping data.");
                                goto exit_work_func;
                            }
                            else
                            {
                                GTP_INFO("Send hopping data success.");
                            }
                        }
                    #endif
                        rqst_data[2] = GTP_RQST_RESPONDED;
                        gtp_i2c_write(i2c_client_point, rqst_data, 3);
                        rqst_processing = 0;
                    }
                    goto exit_work_func;

                case GTP_RQST_RESET:
				//#if GTP_GESTURE_WAKEUP
                if(gesture_value > 0){
				if (DOZE_ENABLED == doze_status) {
					u8 reg_data[3] = {(u8)(0x801F >> 8), (u8)(0x801F & 0xFF), 0};
					gtp_i2c_write(i2c_client_point, reg_data, 3);
                    gtp_recovery_reset(i2c_client_point);
					gtp_enter_doze(i2c_client_point);
				}
                }else
				//#endif
				{
					gtp_recovery_reset(i2c_client_point);
				}
					GTP_INFO("Request Reset.");
                    goto exit_work_func;

            #if GTP_POWER_CTRL_SLEEP
                case GTP_RQST_STORE_HOPPING:
                    GTP_INFO("Request store hopping data.");
                    ret = gtp_hopping_proc(i2c_client_point, GTP_HOPPING_STORE);
                    if (FAIL == ret)
                    {
                        GTP_ERROR("Failed to store hopping data.");
                    }
                    else
                    {
                        GTP_INFO("Hopping data stored.");
                        rqst_data[2] = GTP_RQST_RESPONDED;
                        gtp_i2c_write(i2c_client_point, rqst_data, 3);
                    }
                    goto exit_work_func;

                case GTP_RQST_STORE_BAK_REF:
                    GTP_INFO("Request store backup reference.");
                    ret = gtp_bak_ref_proc(i2c_client_point, GTP_BAK_REF_STORE);
                    if (FAIL == ret)
                    {
                        GTP_ERROR("Failed to store backup reference data.");
                    }
                    else
                    {
                        GTP_INFO("Backup reference data stored.");
                        rqst_data[2] = GTP_RQST_RESPONDED;
                        gtp_i2c_write(i2c_client_point, rqst_data, 3);
                    }
                    goto exit_work_func;
            #endif

                default:
                    GTP_INFO("Undefined request code: 0x%02X", rqst_data[2]);
                    rqst_data[2] = GTP_RQST_RESPONDED;
                    gtp_i2c_write(i2c_client_point, rqst_data, 3);
                    break;
            }
        }
    #endif

        if (finger == 0x00)
        {
            continue;
        }

        if ((finger & 0x80) == 0)
        {
            goto exit_work_func;
        }
if((gtp_autotool_setting == 1) && !(point_data[GTP_ADDR_LENGTH] & 0x40)){
    gtp_autotool_setting = 0;
}
	//add by caoyang	
if(gesture_value > 0){
    proximity_status = point_data[GTP_ADDR_LENGTH];
         GTP_DEBUG("REG INDEX[0x814E]:0x%02X\n", proximity_status);
	if(tpd_sleep_flag == 1){
		goto exit_work_func;
	}
          if (proximity_status & 0x40)                //proximity or large touch detect,enable hwm_sensor.
          {
                printk(KERN_ERR"caoyang0 test for xipin\n");
        		//mod_timer(&buttons_timer, jiffies+HZ/50);
          if(gtp_autotool_setting == 1){
              gtp_autotool_setting = 0;
              goto exit_work_func;
          }
#if 1
		  input_report_key(tpd->dev, KEY_POWER, 1);
		  input_sync(tpd->dev);
		  input_report_key(tpd->dev, KEY_POWER, 0);
		  input_sync(tpd->dev);
          tpd_sleep_flag = 1;
#endif
		  goto exit_work_func;
            }
	//end
}
    #ifdef TPD_PROXIMITY
        if (tpd_proximity_flag == 1)
        {
            proximity_status = point_data[GTP_ADDR_LENGTH];
            GTP_DEBUG("REG INDEX[0x814E]:0x%02X\n", proximity_status);

            if (proximity_status & 0x60)                //proximity or large touch detect,enable hwm_sensor.
            {
                tpd_proximity_detect = 0;
                //sensor_data.values[0] = 0;
            }
            else
            {
                tpd_proximity_detect = 1;
                //sensor_data.values[0] = 1;
            }

            //get raw data
            GTP_DEBUG(" ps change\n");
            GTP_DEBUG("PROXIMITY STATUS:0x%02X\n", tpd_proximity_detect);
            //map and store data to hwm_sensor_data
            sensor_data.values[0] = tpd_get_ps_value();
            sensor_data.value_divide = 1;
            sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
            //report to the up-layer
            ret = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data);

            if (ret)
            {
                GTP_ERROR("Call hwmsen_get_interrupt_data fail = %d\n", err);
            }
        }

    #endif

        touch_num = finger & 0x0f;

        if (touch_num > GTP_MAX_TOUCH)
        {
            goto exit_work_func;
        }

        if (touch_num > 1)
        {
            u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

            ret = gtp_i2c_read(i2c_client_point, buf, 2 + 8 * (touch_num - 1));
            memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
        }

    #if (GTP_HAVE_TOUCH_KEY || GTP_PEN_HAVE_BUTTON)
        key_value = point_data[3 + 8 * touch_num];

        if (key_value || pre_key)
        {
        #if GTP_PEN_HAVE_BUTTON
            if (key_value == 0x40)
            {
                GTP_DEBUG("BTN_STYLUS & BTN_STYLUS2 Down.");
                input_report_key(pen_dev, BTN_STYLUS, 1);
                input_report_key(pen_dev, BTN_STYLUS2, 1);
                pen_active = 1;
            }
            else if (key_value == 0x10)
            {
                GTP_DEBUG("BTN_STYLUS Down, BTN_STYLUS2 Up.");
                input_report_key(pen_dev, BTN_STYLUS, 1);
                input_report_key(pen_dev, BTN_STYLUS2, 0);
                pen_active = 1;
            }
            else if (key_value == 0x20)
            {
                GTP_DEBUG("BTN_STYLUS Up, BTN_STYLUS2 Down.");
                input_report_key(pen_dev, BTN_STYLUS, 0);
                input_report_key(pen_dev, BTN_STYLUS2, 1);
                pen_active = 1;
            }
            else
            {
                GTP_DEBUG("BTN_STYLUS & BTN_STYLUS2 Up.");
                input_report_key(pen_dev, BTN_STYLUS, 0);
                input_report_key(pen_dev, BTN_STYLUS2, 0);
                if ( (pre_key == 0x40) || (pre_key == 0x20) ||
                     (pre_key == 0x10)
                   )
                {
                    pen_active = 1;
                }
            }
            if (pen_active)
            {
                touch_num = 0;      // shield pen point
                //pre_touch = 0;    // clear last pen status
            }
        #endif
        #if GTP_HAVE_TOUCH_KEY
            if (!pre_touch)
            {
                for (i = 0; i < GTP_MAX_KEY_NUM; i++)
                {
                    input_report_key(tpd->dev, touch_key_array[i], key_value & (0x01 << i));
                }
                touch_num = 0;  // shiled fingers
            }
        #endif
        }
    #endif
        pre_key = key_value;

        GTP_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);

        if (touch_num)
        {
            for (i = 0; i < touch_num; i++)
            {
                coor_data = &point_data[i * 8 + 3];

                id = coor_data[0] & 0x0F;
                input_x  = coor_data[1] | coor_data[2] << 8;
                input_y  = coor_data[3] | coor_data[4] << 8;
                input_w  = coor_data[5] | coor_data[6] << 8;
//chenhui add start
//				input_y=abs_y_max-input_y;
//				input_x=abs_x_max-input_x;
//				s32 temp_input_x=input_x;
//				input_x=input_y;
//				input_y=temp_input_x;

//chenhui add end

                input_x = TPD_WARP_X(abs_x_max, input_x);
                input_y = TPD_WARP_Y(abs_y_max, input_y);

            #if GTP_WITH_PEN
                id = coor_data[0];
                if ((id & 0x80))      // pen/stylus is activated
                {
                    GTP_DEBUG("Pen touch DOWN!");
                    pre_pen = 1;
                    //id &= 0x7F;
                    id = 0;
                    GTP_DEBUG("(%d)(%d, %d)[%d]", id, input_x, input_y, input_w);
                    gtp_pen_down(input_x, input_y, input_w, id);
                    pen_active = 1;
                }
                else
            #endif
                {
                    GTP_DEBUG(" (%d)(%d, %d)[%d]", id, input_x, input_y, input_w);
                    tpd_down(input_x, input_y, input_w, id);
                }
            }
        }
        else
        {
            if (pre_touch)
            {
            #if GTP_WITH_PEN
                if (pre_pen)
                {
                    GTP_DEBUG("Pen touch UP!");
                    gtp_pen_up();
                    pre_pen = 0;
                    pen_active = 1;
                }
                else
            #endif
                {
                    GTP_DEBUG("Touch Release!");
                    tpd_up(0, 0, 0);
                }
            }
        }
        pre_touch = touch_num;

    #if GTP_WITH_PEN
        if (pen_active)
        {
            pen_active = 0;
            input_sync(pen_dev);
        }
        else
    #endif
        {
            input_sync(tpd->dev);
        }

exit_work_func:

        if (!gtp_rawdiff_mode)
        {
            ret = gtp_i2c_write(i2c_client_point, end_cmd, 3);

            if (ret < 0)
            {
                GTP_ERROR("I2C write end_cmd  error!");
            }
        }
    #if (GTP_COMPATIBLE_MODE && GTP_FL_LITTLE_SYSTEM)
        if (CHIP_TYPE_GT9F == gtp_chip_type)
        {
            if (little_fw_mode)
            {
                if ((fw_block == 0) && (block_section == 1))
                {
                    GTP_INFO("Begin downloading big ss51 firmware");
                }
                gtp_download_big_ss51(i2c_client_point);
            }
        }
    #endif
    } while (!kthread_should_stop());

    return 0;
}

static int tpd_local_init(void)
{
#if GTP_ESD_PROTECT
    clk_tick_cnt = 2 * HZ;   // HZ: clock ticks in 1 second generated by system
    GTP_DEBUG("Clock ticks for an esd cycle: %d", clk_tick_cnt);
    INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
    gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
    spin_lock_init(&esd_lock);          // 2.6.39 & later
    // esd_lock = SPIN_LOCK_UNLOCKED;   // 2.6.39 & before
#endif

#if GTP_SUPPORT_I2C_DMA
    gpDMABuf_va = (u8 *)dma_alloc_coherent(NULL, GTP_DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL);
    if(!gpDMABuf_va){
        GTP_ERROR("[Error] Allocate DMA I2C Buffer failed!\n");
    }
    memset(gpDMABuf_va, 0, GTP_DMA_MAX_TRANSACTION_LENGTH);
#endif
    if (i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        GTP_ERROR("unable to add i2c driver.\n");
        return -1;
    }

    if (tpd_load_status == 0) //if(tpd_load_status == 0) // disable auto load touch driver for linux3.0 porting
    {
        GTP_ERROR("add error touch panel driver.\n");
        i2c_del_driver(&tpd_i2c_driver);
        return -1;
    }

	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, (GTP_MAX_TOUCH - 1), 0, 0);
#ifdef TPD_HAVE_BUTTON
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8 * 4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8 * 4);
#endif

    // set vendor string
    tpd->dev->id.vendor = 0x00;
    tpd->dev->id.product = tpd_info.pid;
    tpd->dev->id.version = tpd_info.vid;

    GTP_DEBUG("end %s, %d\n", __FUNCTION__, __LINE__);
    tpd_type_cap = 1;

    return 0;
}

static s8 gtp_enter_doze(struct i2c_client *client)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 8};

    GTP_DEBUG_FUNC();

    GTP_DEBUG("Entering gesture mode...");
    while(retry++ < 5)
    {
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x46;
        ret = gtp_i2c_write(client, i2c_control_buf, 3);
        if (ret < 0)
        {
            GTP_DEBUG("Failed to set gesture flag into 0x8046, %d", retry);
            continue;
        }
        i2c_control_buf[0] = 0x80;
        i2c_control_buf[1] = 0x40;
        ret = gtp_i2c_write(client, i2c_control_buf, 3);
        if (ret > 0)
        {
            doze_status = DOZE_ENABLED;
            GTP_INFO("Gesture mode enabled.");
            return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send gesture cmd failed.");
    return ret;
}

/*******************************************************
Function:
    Eter sleep function.

Input:
    client:i2c_client.

Output:
    Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_enter_sleep(struct i2c_client *client)
{
#if (GTP_COMPATIBLE_MODE && !GTP_POWER_CTRL_SLEEP)
    if (CHIP_TYPE_GT9F == gtp_chip_type)
    {
        u8 i2c_status_buf[3] = {0x80, 0x44, 0x00};
        s32 ret = 0;

        ret = gtp_i2c_read(client, i2c_status_buf, 3);
        if(ret <= 0)
        {
             GTP_ERROR("[gtp_enter_sleep]Read ref status reg error.");
        }

        if (i2c_status_buf[2] & 0x80)
        {
            //Store bak ref
            ret = gtp_bak_ref_proc(client, GTP_BAK_REF_STORE);
            if(FAIL == ret)
            {
                GTP_ERROR("[gtp_enter_sleep]Store bak ref failed.");
            }
        }
    }
#endif
#if GTP_POWER_CTRL_SLEEP

    GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
    GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
    msleep(10);

#ifdef MT6573
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
    msleep(30);
#else               // ( defined(MT6575) || defined(MT6577) || defined(MT6589) )

    #ifdef TPD_POWER_SOURCE_1800
        hwPowerDown(TPD_POWER_SOURCE_1800, "TP");
    #endif

    #ifdef TPD_POWER_SOURCE_CUSTOM
        hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
    #else
        hwPowerDown(MT65XX_POWER_LDO_VGP2, "TP");
    #endif
#endif

    GTP_INFO("GTP enter sleep by poweroff!");
    return 0;

#else
    {
        s8 ret = -1;
        s8 retry = 0;
        u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};


        GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
        msleep(5);

        while (retry++ < 5)
        {
            ret = gtp_i2c_write(client, i2c_control_buf, 3);

            if (ret > 0)
            {
                GTP_INFO("GTP enter sleep!");

                return ret;
            }

            msleep(10);
        }

        GTP_ERROR("GTP send sleep cmd failed.");
        return ret;
    }
#endif
}

/*******************************************************
Function:
    Wakeup from sleep mode Function.

Input:
    client:i2c_client.

Output:
    Executive outcomes.0--success,non-0--fail.
*******************************************************/
static s8 gtp_wakeup_sleep(struct i2c_client *client)
{
    u8 retry = 0;
    s8 ret = -1;

    GTP_DEBUG("GTP wakeup begin.");

#if GTP_POWER_CTRL_SLEEP
	#if (GTP_COMPATIBLE_MODE && GTP_FL_LITTLE_SYSTEM)
    if (CHIP_TYPE_GT9F == gtp_chip_type)
    {
        power_is_down = 0;
        little_fw_mode = 1;
        kthread_run(gtp_resume_timeout, (void*)NULL, "resume_timeout");

        ret = force_reset_guitar(1);
        if (FAIL == ret)
        {
            little_fw_mode = 0;
            gtp_recovery_reset(client);
        }
		if (!gtp_check_fw()) {
		       little_fw_mode = 0;
		       gtp_recovery_reset(client);
		 }
		 return 0;
	}
	#endif // end compatible mode & fl little system

    while (retry++ < 5)
    {
        ret = tpd_power_on(client);

        if (ret < 0)
        {
            GTP_ERROR("I2C Power on ERROR!");
            continue;
        }
        GTP_INFO("Ic wakeup by poweron");
        return 0;
    }
#else  // esle PowerCtrlSleep
	while (retry++ < 5) {
    //#if GTP_GESTURE_WAKEUP
    if(gesture_value > 0){
        if (DOZE_WAKEUP != doze_status)
        {
            GTP_INFO("Powerkey wakeup.");
        }
        else
        {
            GTP_INFO("Gesture wakeup.");
        }
        doze_status = DOZE_DISABLED;

        mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
        gtp_reset_guitar(client, 20);
		ret = gup_reload_fw_dsp(NULL, GTP_FL_READ_REPAIR);
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		if (ret == FAIL) {
			GTP_ERROR("[gtp_wakeup_sleep]Reload dsp code failed.");
			continue;
		}
		ret = gtp_fw_startup(client);
		if (ret == FAIL) {
			GTP_ERROR("[gtp_wakeup_sleep]Startup fw failed.");
			continue;
		}
		if (!gtp_check_fw()) {
       		gtp_recovery_reset(client);
		 }
		return 0; // succeed
    //#else // else gesture wakeup
    }else{
		GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);  //wake up by int gpio
		msleep(5);

		#if GTP_COMPATIBLE_MODE
		if (CHIP_TYPE_GT9F == gtp_chip_type) {
			u8 opr_buf[2] = {0};

			ret = gtp_i2c_test(client);
			if (ret >= 0) {
				// Hold ss51 & dsp
				opr_buf[0] = 0x0C;
				ret = i2c_write_bytes(client, 0x4180, opr_buf, 1);
				if (ret < 0) {
					GTP_DEBUG("Hold ss51 & dsp I2C error,retry:%d", retry);
					continue;
				}

				// Confirm hold
				opr_buf[0] = 0x00;
				ret = i2c_read_bytes(client, 0x4180, opr_buf, 1);
				if (ret < 0) {
					GTP_DEBUG("confirm ss51 & dsp hold, I2C error,retry:%d", retry);
					continue;
				}
				if (0x0C != opr_buf[0]) {
					GTP_DEBUG("ss51 & dsp not hold, val: %d, retry: %d", opr_buf[0], retry);
					continue;
				}
				GTP_DEBUG("ss51 & dsp has been hold");

				ret = gtp_fw_startup(client);
				if (FAIL == ret) {
					GTP_ERROR("[gtp_wakeup_sleep]Startup fw failed.");
					continue;
				}
				GTP_INFO("flashless wakeup sleep success");
				return ret;
			}
			force_reset_guitar(0);
			retry = 0;
			break;
		}
		//#endif // end compatible mode
        }
		ret = gtp_i2c_test(client);
        if (ret >= 0)
        {
            GTP_INFO("GTP wakeup sleep.");
        //#if (!GTP_GESTURE_WAKEUP)
        if(!(gesture_value > 0))
            {
                gtp_int_sync(25);
            }
        //#endif
            return ret;
        }
        gtp_reset_guitar(client, 20);
      #endif // end  gesture wakeup
	}

	if (retry >= 5) {
		GTP_ERROR("wakeup retry timeout, process esd reset");
		force_reset_guitar(0);
	}
#endif // end PowerCtrlSleep
    GTP_ERROR("GTP wakeup sleep failed.");
    return ret;
}

/* Function to manage low power suspend */
static void tpd_suspend(struct early_suspend *h)
{
    s32 ret = -1;

    GTP_INFO("System suspend.");

#ifdef TPD_PROXIMITY

    if (tpd_proximity_flag == 1)
    {
        return ;
    }

#endif
#if (GTP_COMPATIBLE_MODE && GTP_FL_LITTLE_SYSTEM)
    little_fw_mode = 0;
    power_is_down = 1;
#endif

    tpd_halt = 1;
#if GTP_ESD_PROTECT
    gtp_esd_switch(i2c_client_point, SWITCH_OFF);
#endif
//#if GTP_GESTURE_WAKEUP
if(gesture_value > 0){
    ret = gtp_enter_doze(i2c_client_point);
//#else
}else{
    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    ret = gtp_enter_sleep(i2c_client_point);
//#endif
}
    if (ret < 0)
    {
        GTP_ERROR("GTP early suspend failed.");
    }
    // to avoid waking up while not sleeping, delay 48 + 10ms to ensure reliability
    msleep(58);
}

/* Function to manage power-on resume */
static void tpd_resume(struct early_suspend *h)
{
    s32 ret = -1;

    GTP_INFO("System resume.");

#ifdef TPD_PROXIMITY

    if (tpd_proximity_flag == 1)
    {
        return ;
    }

#endif
    tpd_sleep_flag = 0;
    ret = gtp_wakeup_sleep(i2c_client_point);

    if (ret < 0)
    {
        GTP_ERROR("GTP later resume failed.");
    }

#if GTP_COMPATIBLE_MODE
    if (CHIP_TYPE_GT9F == gtp_chip_type)
    {
        // do nothing
    }
    else
#endif
    {
        gtp_send_cfg(i2c_client_point);
    }

#if GTP_CHARGER_SWITCH
    gtp_charger_switch(1);  // force update
#endif

    tpd_halt = 0;
//#if GTP_GESTURE_WAKEUP
if(gesture_value > 0){
    doze_status = DOZE_DISABLED;
//#else
}else{
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
//#endif
}

#if GTP_ESD_PROTECT
	gtp_init_ext_watchdog(i2c_client_point);
    gtp_esd_switch(i2c_client_point, SWITCH_ON);
#endif

}

static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = "gt9xx",
    .tpd_local_init = tpd_local_init,
    .suspend = tpd_suspend,
    .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
    if(g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT || g_boot_mode == LOW_POWER_OFF_CHARGING_BOOT)   //add by zero
    	   return 0;
    GTP_INFO("MediaTek gt91xx touch panel driver init\n");
#ifdef I2C_BUS_NUMBER
    i2c_register_board_info(I2C_BUS_NUMBER, &i2c_tpd, 1);
#else
    i2c_register_board_info(0, &i2c_tpd, 1);
#endif
    if (tpd_driver_add(&tpd_device_driver) < 0)
        GTP_INFO("add generic driver failed\n");

    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
    GTP_INFO("MediaTek gt91xx touch panel driver exit\n");
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

