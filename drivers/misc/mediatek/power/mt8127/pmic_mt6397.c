/*****************************************************************************
 *
 * Filename:
 * ---------
 *    pmic_mt6397.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines PMIC functions
 *
 * Author:
 * -------
 * James Lo
 *
 ****************************************************************************/
#include <generated/autoconf.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/aee.h>
#include <linux/xlog.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/writeback.h>

#include <asm/uaccess.h>

#include <mach/pmic_mt6397_sw.h>
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>
#include <mach/mt_pm_ldo.h>
//#include <mach/mt6320_pmic_feature_api.h>
#include <mach/eint.h>
#include <mach/mt_pmic_wrap.h>
#include <mach/mt_gpio.h>
#include <mach/mtk_rtc.h>
#include <mach/mt_spm_mtcmos.h>
#include <mach/charging.h>

//#include "mt6320_battery.h"

#include <mtk_kpd.h>

#if defined (CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)
#include <mach/mt_boot.h>
#include <mach/system.h>
#include "mach/mt_gpt.h"
#endif
#if 1
#include <mach/mt_clkmgr.h>
#else
//#include <mach/mt_clock_manager.h>
//----------------------------------------------------------------------test
#define MT65XX_UPLL 3
static void enable_pll(int id, char *mod_name)
{
    printk("enable_pll is not ready.\n");
}
static void disable_pll(int id, char *mod_name)
{
    printk("disable_pll is not ready.\n");
}
//----------------------------------------------------------------------
#endif
#ifdef MTK_INTERNAL_MHL_SUPPORT
#include "mhl_extern.h"
#endif

#include <cust_gpio_usage.h>
#include <cust_eint.h>
//----------------------------------------------------------------------test
#if 0
#define CUST_EINT_POLARITY_LOW              0
#define CUST_EINT_POLARITY_HIGH             1
#define CUST_EINT_DEBOUNCE_DISABLE          0
#define CUST_EINT_DEBOUNCE_ENABLE           1
#define CUST_EINT_EDGE_SENSITIVE            0
#define CUST_EINT_LEVEL_SENSITIVE           1

#define CUST_EINT_MT6320_PMIC_NUM              3
#define CUST_EINT_MT6320_PMIC_DEBOUNCE_CN      1
#define CUST_EINT_MT6320_PMIC_POLARITY         CUST_EINT_POLARITY_HIGH
#define CUST_EINT_MT6320_PMIC_SENSITIVE        CUST_EINT_LEVEL_SENSITIVE
#define CUST_EINT_MT6320_PMIC_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_ENABLE
#endif
//----------------------------------------------------------------------

//==============================================================================
// PMIC related define
//==============================================================================
#define VOLTAGE_FULL_RANGE     1200
#define ADC_PRECISE         1024 // 10 bits
static DEFINE_MUTEX(pmic_lock_mutex);
static DEFINE_MUTEX(pmic_adc_mutex);

//==============================================================================
// Extern
//==============================================================================
extern int g_R_BAT_SENSE;
extern int g_R_I_SENSE;
extern int g_R_CHARGER_1;
extern int g_R_CHARGER_2;

extern int bat_thread_kthread(void *x);
extern void charger_hv_detect_sw_workaround_init(void);
extern int accdet_irq_handler(void);
extern void accdet_auxadc_switch(int enable);
extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
#if defined (CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)
extern void mt_power_off(void);
static kal_bool long_pwrkey_press = false;
static unsigned long timer_pre = 0; 
static unsigned long timer_pos = 0; 
#define LONG_PWRKEY_PRESS_TIME 		500*1000000    //500ms
#endif
//==============================================================================
// PMIC lock/unlock APIs
//==============================================================================
void pmic_lock(void)
{
    mutex_lock(&pmic_lock_mutex);
}

void pmic_unlock(void)
{
    mutex_unlock(&pmic_lock_mutex);
}

kal_uint32 upmu_get_reg_value(kal_uint32 reg)
{
    U32 ret=0;
    U32 reg_val=0;

    //printk("[upmu_get_reg_value] \n");
    ret=pmic_read_interface(reg, &reg_val, 0xFFFF, 0x0);
    
    return reg_val;
}

void upmu_set_reg_value(kal_uint32 reg, kal_uint32 reg_val)
{
    U32 ret=0;

    //printk("[upmu_set_reg_value] \n");
    ret=pmic_config_interface(reg, reg_val, 0xFFFF, 0x0);    
}

//==============================================================================
// PMIC-AUXADC 
//==============================================================================
extern int Enable_BATDRV_LOG;

int PMIC_IMM_GetOneChannelValue(int dwChannel, int deCount, int trimd)
{
    kal_int32 u4Sample_times = 0;
    kal_int32 u4channel[8] = {0,0,0,0,0,0,0,0};
    kal_int32 adc_result=0;
    kal_int32 adc_result_temp=0;
    kal_int32 r_val_temp=0;    
    kal_int32 count=0;
    kal_int32 count_time_out=1000;
    kal_int32 ret_data=0;

    mutex_lock(&pmic_adc_mutex);

    if(dwChannel==1)
    {
        upmu_set_reg_value(0x0020, 0x0801);
        upmu_set_rg_source_ch0_norm_sel(1);
        upmu_set_rg_source_ch0_lbat_sel(1);
        dwChannel=0;
        mdelay(1);
    }

    /*
        0 : V_BAT
        1 : V_I_Sense
        2 : V_Charger
        3 : V_TBAT
        4~7 : reserved    
    */
    upmu_set_rg_auxadc_chsel(dwChannel);

    //upmu_set_rg_avg_num(0x3);

    if(dwChannel==3)
    {
        upmu_set_rg_buf_pwd_on(1);
        upmu_set_rg_buf_pwd_b(1);
        upmu_set_baton_tdet_en(1);
        mdelay(20);
    }

    if(dwChannel==4)
    {
        upmu_set_rg_vbuf_en(0);
        upmu_set_rg_vbuf_byp(1);
        
        if(trimd==2)
        {
            upmu_set_rg_vbuf_calen(0); /* For T_PMIC*/
            upmu_set_rg_spl_num(0x10);
            //upmu_set_rg_spl_num(0x1E);
            //upmu_set_rg_avg_num(0x6);
            trimd=1;
        }
        else
        {
            upmu_set_rg_vbuf_calen(1); /* For T_BAT*/
        }
    }
    if(dwChannel == 5)
    {
#ifdef CONFIG_MTK_ACCDET
        accdet_auxadc_switch(1);
#endif
    }
    u4Sample_times=0;
    
    do
    {
        upmu_set_rg_auxadc_start(0);
        upmu_set_rg_auxadc_start(1);

        //Duo to HW limitation
        udelay(30);

        count=0;
        ret_data=0;

        switch(dwChannel){         
            case 0:    
                while( upmu_get_rg_adc_rdy_c0() != 1 )
                {    
                    if( (count++) > count_time_out)
                    {
                        xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
                        break;
                    }
                }
                if(trimd == 1) {
                    ret_data = upmu_get_rg_adc_out_c0_trim();
                } else {            
                    ret_data = upmu_get_rg_adc_out_c0();
                }
                break;
            case 1:    
                while( upmu_get_rg_adc_rdy_c1() != 1 )
                {    
                    if( (count++) > count_time_out)
                    {
                        xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
                        break;
                    }
                }
                if(trimd == 1) {
                    ret_data = upmu_get_rg_adc_out_c1_trim();
                } else {            
                    ret_data = upmu_get_rg_adc_out_c1();
                }
                break;
            case 2:    
                while( upmu_get_rg_adc_rdy_c2() != 1 )
                {    
                    if( (count++) > count_time_out)
                    {
                        xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
                        break;
                    }
                }
                if(trimd == 1) {
                    ret_data = upmu_get_rg_adc_out_c2_trim();
                } else {            
                    ret_data = upmu_get_rg_adc_out_c2();
                }
                break;
            case 3:    
                while( upmu_get_rg_adc_rdy_c3() != 1 )
                {    
                    if( (count++) > count_time_out)
                    {
                        xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
                        break;
                    }
                }
                if(trimd == 1) {
                    ret_data = upmu_get_rg_adc_out_c3_trim();
                } else {            
                    ret_data = upmu_get_rg_adc_out_c3();
                }
                break;
            case 4:    
                while( upmu_get_rg_adc_rdy_c4() != 1 )
                {    
                    if( (count++) > count_time_out)
                    {
                        xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
                        break;
                    }
                }
                if(trimd == 1) {
                    ret_data = upmu_get_rg_adc_out_c4_trim();
                } else {            
                    ret_data = upmu_get_rg_adc_out_c4();
                }
                break;
            case 5:    
                while( upmu_get_rg_adc_rdy_c5() != 1 )
                {    
                    if( (count++) > count_time_out)
                    {
                        xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
                        break;
                    }
                }
                if(trimd == 1) {
                    ret_data = upmu_get_rg_adc_out_c5_trim();
                } else {            
                    ret_data = upmu_get_rg_adc_out_c5();
                }
                break;
            case 6:    
                while( upmu_get_rg_adc_rdy_c6() != 1 )
                {    
                    if( (count++) > count_time_out)
                    {
                        xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
                        break;
                    }
                }
                if(trimd == 1) {
                    ret_data = upmu_get_rg_adc_out_c6_trim();
                } else {            
                    ret_data = upmu_get_rg_adc_out_c6();
                }
                break;
            case 7:    
                while( upmu_get_rg_adc_rdy_c7() != 1 )
                {    
                    if( (count++) > count_time_out)
                    {
                        xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[IMM_GetOneChannelValue_PMIC] (%d) Time out!\n", dwChannel);
                        break;
                    }
                }
                if(trimd == 1) {
                    ret_data = upmu_get_rg_adc_out_c7_trim();
                } else {            
                    ret_data = upmu_get_rg_adc_out_c7();
                }
                break;    
            default:
                xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[AUXADC] Invalid channel value(%d,%d)\n", dwChannel, trimd);
                return -1;
                break;
        }

        u4channel[dwChannel] += ret_data;

        u4Sample_times++;

        if (Enable_BATDRV_LOG == 1)
        {
            //debug
            xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[AUXADC] u4Sample_times=%d, ret_data=%d, u4channel[%d]=%d.\n", 
                u4Sample_times, ret_data, dwChannel, u4channel[dwChannel]);
        }
        
    }while (u4Sample_times < deCount);

    /* Value averaging  */ 
    u4channel[dwChannel] = u4channel[dwChannel] / deCount;
    adc_result_temp = u4channel[dwChannel];

    switch(dwChannel){         
        case 0:                
            r_val_temp = g_R_BAT_SENSE;            
            adc_result = (adc_result_temp*r_val_temp*VOLTAGE_FULL_RANGE)/ADC_PRECISE;
            break;
        case 1:    
            r_val_temp = g_R_I_SENSE;
            adc_result = (adc_result_temp*r_val_temp*VOLTAGE_FULL_RANGE)/ADC_PRECISE;
            break;
        case 2:    
            r_val_temp = (((g_R_CHARGER_1+g_R_CHARGER_2)*100)/g_R_CHARGER_2);
            adc_result = (adc_result_temp*r_val_temp*VOLTAGE_FULL_RANGE)/ADC_PRECISE;
            break;
        case 3:    
            r_val_temp = 1;
            adc_result = (adc_result_temp*r_val_temp*VOLTAGE_FULL_RANGE)/ADC_PRECISE;
            break;
        case 4:    
            r_val_temp = 1;
            adc_result = (adc_result_temp*r_val_temp*VOLTAGE_FULL_RANGE)/ADC_PRECISE;
            break;
        case 5:    
            r_val_temp = 1;
            adc_result = (adc_result_temp*r_val_temp*VOLTAGE_FULL_RANGE)/ADC_PRECISE;
            break;
        case 6:    
            r_val_temp = 1;
            adc_result = (adc_result_temp*r_val_temp*VOLTAGE_FULL_RANGE)/ADC_PRECISE;
            break;
        case 7:    
            r_val_temp = 1;
            adc_result = (adc_result_temp*r_val_temp*VOLTAGE_FULL_RANGE)/ADC_PRECISE;
            break;    
        default:
            xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[AUXADC] Invalid channel value(%d,%d)\n", dwChannel, trimd);
            return -1;
            break;
    }

    if (Enable_BATDRV_LOG == 1)
    {
        //debug
        xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[AUXADC] adc_result_temp=%d, adc_result=%d, r_val_temp=%d.\n", 
                adc_result_temp, adc_result, r_val_temp);
    }

    count=0;

    if(dwChannel==0)
    {
        upmu_set_rg_source_ch0_norm_sel(0);
        upmu_set_rg_source_ch0_lbat_sel(0);
    }

    if(dwChannel==3)
    {
        upmu_set_baton_tdet_en(0);     
        upmu_set_rg_buf_pwd_b(0);
        upmu_set_rg_buf_pwd_on(0);
    }

    if(dwChannel==4)
    {
        //upmu_set_rg_vbuf_en(0);
        //upmu_set_rg_vbuf_byp(0);
        upmu_set_rg_vbuf_calen(0);
    }
    if(dwChannel == 5)
    {
#ifdef CONFIG_MTK_ACCDET
        accdet_auxadc_switch(0);
#endif
    }

    upmu_set_rg_spl_num(0x1);

    mutex_unlock(&pmic_adc_mutex);

    return adc_result;
    
}

//==============================================================================
// PMIC-Charger Type Detection
//==============================================================================

void upmu_interrupt_chrdet_int_en(kal_uint32 val)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[upmu_interrupt_chrdet_int_en] val=%d.\r\n", val);

    upmu_set_rg_int_en_chrdet(val);
}
EXPORT_SYMBOL(upmu_interrupt_chrdet_int_en);

//==============================================================================
// PMIC Interrupt service
//==============================================================================
int pmic_thread_timeout=0;
static DEFINE_MUTEX(pmic_mutex);
static DECLARE_WAIT_QUEUE_HEAD(pmic_thread_wq);

extern int g_chr_event;

#ifdef MTK_BATTERY_NO_HAL
extern struct wake_lock battery_suspend_lock;
#endif

#ifdef CONFIG_MTK_SMART_BATTERY
extern void wake_up_bat (void);
#endif

extern int bat_volt_check_point;
extern int g_bat_init_flag;

void wake_up_pmic(void)
{
    //xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[wake_up_pmic]\r\n");
    pmic_thread_timeout = 1;
    wake_up(&pmic_thread_wq);
}
EXPORT_SYMBOL(wake_up_pmic);

#define WAKE_LOCK_INITIALIZED            (1U << 8)

#if defined(CONFIG_POWER_EXT)
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);
#endif

#ifdef MTK_BATTERY_NO_HAL
void do_chrdet_int_task(void)
{
    U32 ret=0;
    U32 ret_val=0;
    U32 reg_val=0;
    
    ret=upmu_get_rgs_chrdet();
    if(ret==1)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[do_chrdet_int_task] charger exist!\n");
        g_charger_in_flag = 1;

        #if defined(CONFIG_POWER_EXT)
        mt_usb_connect();
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[do_chrdet_int_task] call mt_usb_connect() in EVB\n");
        #endif
    }
    else
    {
#if defined (CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)
		if(g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT || g_boot_mode == LOW_POWER_OFF_CHARGING_BOOT)
		{
			xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_thread_kthread] Unplug Charger/USB In Kernel Power Off Charging Mode!  Shutdown OS!\r\n");
			mt_power_off();
		}
#endif
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[do_chrdet_int_task] charger NOT exist!\n");
        g_charger_in_flag = 0;
        g_first_check = 0;

        //RG_BC11_BB_CTRL=1
        ret_val=pmic_config_interface(CHR_CON18,0x1,PMIC_RG_BC11_BB_CTRL_MASK,PMIC_RG_BC11_BB_CTRL_SHIFT);
        //RG_BC11_BIAS_EN=0
        ret_val=pmic_config_interface(CHR_CON19,0x0,PMIC_RG_BC11_BIAS_EN_MASK,PMIC_RG_BC11_BIAS_EN_SHIFT);
        //RG_BC11_VSRC_EN[1:0]=00
        ret_val=pmic_config_interface(CHR_CON18,0x0,PMIC_RG_BC11_VSRC_EN_MASK,PMIC_RG_BC11_VSRC_EN_SHIFT);
        //check
        ret_val=pmic_read_interface(CHR_CON18,&reg_val,0xFFFF,0);
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Reg[0x%x]=0x%x\n", CHR_CON18, reg_val);
        ret_val=pmic_read_interface(CHR_CON19,&reg_val,0xFFFF,0);
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Reg[0x%x]=0x%x\n", CHR_CON19, reg_val);

        #if defined(CONFIG_POWER_EXT)
        mt_usb_disconnect();
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[do_chrdet_int_task] call mt_usb_disconnect() in EVB\n");
        #endif
    }
    
    #if defined(CONFIG_POWER_EXT)
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[do_chrdet_int_task] Environment=FPGA/EVB\n");
    #else
    wake_lock(&battery_suspend_lock);
    g_chr_event = 1;

    if(g_bat_init_flag==1)
        wake_up_bat();
    else
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[do_chrdet_int_task] battery thread not ready, will do after bettery init.\n");    
    #endif
}
#else
extern void do_chrdet_int_task(void);
#endif

void cust_pmic_interrupt_en_setting(void)
{
    upmu_set_rg_int_en_ov(0);
    upmu_set_rg_int_en_chrdet(1);
    upmu_set_rg_int_en_bvalid_det(0);
    upmu_set_rg_int_en_vbaton_undet(0);
    upmu_set_rg_int_en_thr_h(0);
    upmu_set_rg_int_en_thr_l(0);
    upmu_set_rg_int_en_pwrkey(1);
    upmu_set_rg_int_en_watchdog(0);
    upmu_set_rg_int_en_fg_bat_h(0);
    upmu_set_rg_int_en_fg_bat_l(0);
    upmu_set_rg_int_en_bat_h(0);
    upmu_set_rg_int_en_bat_l(0);
    upmu_set_rg_int_en_spkr(0);
    upmu_set_rg_int_en_spkl(0);
    upmu_set_rg_int_en_spkr_ab(0);
    upmu_set_rg_int_en_spkl_ab(0);
    
    upmu_set_rg_int_en_vdrm(0);
    upmu_set_rg_int_en_vsrmca7(0);
    upmu_set_rg_int_en_vpca7(0);
    upmu_set_rg_int_en_vio18(0);
    upmu_set_rg_int_en_vgpu(0);
    upmu_set_rg_int_en_vcore(0);
    upmu_set_rg_int_en_vsrmca15(0);
    upmu_set_rg_int_en_vca15(0);
    upmu_set_rg_int_en_hdmi_cec(0);
#ifdef MTK_INTERNAL_MHL_SUPPORT	
 //   upmu_set_rg_int_en_hdmi_sifm(1);
#else
    upmu_set_rg_int_en_hdmi_sifm(0);
#endif
    upmu_set_rg_int_en_pwrkey_rstb(0);
    upmu_set_rg_int_en_rtc(1);
    upmu_set_rg_int_en_audio(0);
    upmu_set_rg_int_en_accdet(1);
    upmu_set_rg_int_en_homekey(1);
    upmu_set_rg_int_en_ldo(0);
}

void spkl_ab_int_handler(void)
{
    kal_uint32 ret=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[spkl_ab_int_handler]....\n");
    
    ret=pmic_config_interface(INT_STATUS0,0x1,0x1,0);
    
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void spkr_ab_int_handler(void)
{
    kal_uint32 ret=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[spkr_ab_int_handler]....\n");
    
    ret=pmic_config_interface(INT_STATUS0,0x1,0x1,1);
    
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void spkl_int_handler(void)
{
    kal_uint32 ret=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[spkl_int_handler]....\n");
    
    ret=pmic_config_interface(INT_STATUS0,0x1,0x1,2);
    
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void spkr_int_handler(void)
{
    kal_uint32 ret=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[spkr_int_handler]....\n");
    
    ret=pmic_config_interface(INT_STATUS0,0x1,0x1,3);
    
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void bat_l_int_handler(void)
{
    kal_uint32 ret=0;
    
    kal_uint32 lbat_debounce_count_max=0;
    kal_uint32 lbat_debounce_count_min=0;
    
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[bat_l_int_handler]....\n");

    upmu_set_rg_lbat_irq_en_max(0);
	upmu_set_rg_lbat_irq_en_min(0);
    upmu_set_rg_lbat_en_max(0);
    upmu_set_rg_lbat_en_min(0);

    lbat_debounce_count_max = upmu_get_rg_lbat_debounce_count_max();
    lbat_debounce_count_min = upmu_get_rg_lbat_debounce_count_min();

    printk("[bat_l_int_handler] (lbat_debounce_count_max=%d, lbat_debounce_count_min=%d) \n", 
    lbat_debounce_count_max, lbat_debounce_count_min);
    
    ret=pmic_config_interface(INT_STATUS0,0x1,0x1,4);
    
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void bat_h_int_handler(void)
{
    kal_uint32 ret=0;

    kal_uint32 lbat_debounce_count_max=0;
    kal_uint32 lbat_debounce_count_min=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[bat_h_int_handler]....\n");

    upmu_set_rg_int_en_bat_h(0);

    upmu_set_rg_lbat_irq_en_max(0);
    upmu_set_rg_lbat_irq_en_min(0);
    upmu_set_rg_lbat_en_max(0);
    upmu_set_rg_lbat_en_min(0);

    lbat_debounce_count_max = upmu_get_rg_lbat_debounce_count_max();
    lbat_debounce_count_min = upmu_get_rg_lbat_debounce_count_min();

    printk("[bat_h_int_handler] (lbat_debounce_count_max=%d, lbat_debounce_count_min=%d) \n", 
    lbat_debounce_count_max, lbat_debounce_count_min);

    ret=pmic_config_interface(INT_STATUS0,0x1,0x1,5);
    
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void fg_bat_l_int_handler(void)
{
    kal_uint32 ret=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[fg_bat_l_int_handler]....\n");
    
    ret=pmic_config_interface(INT_STATUS0,0x1,0x1,6);
    
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void fg_bat_h_int_handler(void)
{
    kal_uint32 ret=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[fg_bat_h_int_handler]....\n");
    
    ret=pmic_config_interface(INT_STATUS0,0x1,0x1,7);
    
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void watchdog_int_handler(void)
{
    kal_uint32 ret=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[watchdog_int_handler]....\n");
    
    ret=pmic_config_interface(INT_STATUS0,0x1,0x1,8);
    
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void pwrkey_int_handler(void)
{
    kal_uint32 ret=0;

    //xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pwrkey_int_handler]....\n");

    if(upmu_get_pwrkey_deb()==1)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pwrkey_int_handler] Release pwrkey\n");
#if defined (CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)
		if(g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT && timer_pre != 0)
		{
				timer_pos = sched_clock();
				if(timer_pos - timer_pre >= LONG_PWRKEY_PRESS_TIME)
				{
					long_pwrkey_press = true;
				}
				xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_thread_kthread] timer_pos = %ld, timer_pre = %ld, timer_pos-timer_pre = %ld, long_pwrkey_press = %d\r\n",timer_pos, timer_pre, timer_pos-timer_pre, long_pwrkey_press);
				if(long_pwrkey_press)   //500ms
				{
					xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_thread_kthread] Power Key Pressed during kernel power off charging, reboot OS\r\n");
					arch_reset(0, NULL);
				}
		}
#endif
        kpd_pwrkey_pmic_handler(0x0);
    }
    else
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pwrkey_int_handler] Press pwrkey\n");
#if defined (CONFIG_MTK_KERNEL_POWER_OFF_CHARGING)
		if(g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT)
		{
			timer_pre = sched_clock();
		}
#endif
        kpd_pwrkey_pmic_handler(0x1);
    }
    
    ret=pmic_config_interface(INT_STATUS0,0x1,0x1,9);
    
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void thr_l_int_handler(void)
{
    kal_uint32 ret=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[thr_l_int_handler]....\n");
    
    ret=pmic_config_interface(INT_STATUS0,0x1,0x1,10);
    
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void thr_h_int_handler(void)
{
    kal_uint32 ret=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[thr_h_int_handler]....\n");
    
    ret=pmic_config_interface(INT_STATUS0,0x1,0x1,11);
    
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void vbaton_undet_int_handler(void)
{
    kal_uint32 ret=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[vbaton_undet_int_handler]....\n");
    
    ret=pmic_config_interface(INT_STATUS0,0x1,0x1,12);
    
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void bvalid_det_int_handler(void)
{
    kal_uint32 ret=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[bvalid_det_int_handler]....\n");
    
    ret=pmic_config_interface(INT_STATUS0,0x1,0x1,13);
    
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void chrdet_int_handler(void)
{
    // read clear, already read
    
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[chrdet_int_handler]....\n");
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
    if (!upmu_get_rgs_chrdet())
    {
        int boot_mode = 0;
        boot_mode = get_boot_mode();
        
        if(boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT || boot_mode == LOW_POWER_OFF_CHARGING_BOOT)
        {
            xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[chrdet_int_handler] Unplug Charger/USB In Kernel Power Off Charging Mode!  Shutdown OS!\r\n");
            mt_power_off();
        }
    }
#endif
    upmu_get_rg_int_status_chrdet();//read for clear
    do_chrdet_int_task();
}
void ov_int_handler(void)
{
    // read clear, already read
    
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[ov_int_handler]....\n");
}
void ldo_int_handler(void)
{
    kal_uint32 ret=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[ldo_int_handler]....\n");
    
    ret=pmic_config_interface(INT_STATUS1,0x1,0x1,0);

    upmu_set_rg_int_en_ldo(0);
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}   
void homekey_int_handler(void)
{
    kal_uint32 ret=0;

    //xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[homekey_int_handler]....\n");

    if(upmu_get_homekey_deb()==1)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[homekey_int_handler] Release HomeKey\r\n");
        kpd_pmic_rstkey_handler(0x0);
    }
    else
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[homekey_int_handler] Press HomeKey\r\n");
        kpd_pmic_rstkey_handler(0x1);
    }
    
    ret=pmic_config_interface(INT_STATUS1,0x1,0x1,1);
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}    
#ifdef CONFIG_MTK_ACCDET
void accdet_int_handler(void)
{
    kal_uint32 ret=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[accdet_int_handler]....\n");
	
    ret = accdet_irq_handler();
	if(0 == ret){
		xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[accdet_int_handler] don't finished\n");
	}
    ret=pmic_config_interface(INT_STATUS1,0x1,0x1,2);
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}    
#endif
void audio_int_handler(void)
{
    kal_uint32 ret=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[audio_int_handler]....\n");
    
    ret=pmic_config_interface(INT_STATUS1,0x1,0x1,3);
    
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}    
void rtc_int_handler(void)
{
    kal_uint32 ret=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[rtc_int_handler]....\n");
    rtc_irq_handler();
    ret=pmic_config_interface(INT_STATUS1,0x1,0x1,4);
    
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void pwrkey_rstb_int_handler(void)
{
    kal_uint32 ret=0;
    
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pwrkey_rstb_int_handler]....\n");

    ret=pmic_config_interface(INT_STATUS1,0x1,PMIC_RG_INT_STATUS_PWRKEY_RSTB_MASK,PMIC_RG_INT_STATUS_PWRKEY_RSTB_SHIFT);

    upmu_set_rg_int_en_pwrkey_rstb(0);
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void hdmi_sifm_int_handler(void)
{
    kal_uint32 ret=0;
    
    //xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[hdmi_sifm_int_handler]....\n");

    ret=pmic_config_interface(INT_STATUS1,0x1,PMIC_RG_INT_STATUS_HDMI_SIFM_MASK,PMIC_RG_INT_STATUS_HDMI_SIFM_SHIFT);

    upmu_set_rg_int_en_hdmi_sifm(0);
    
#ifdef MTK_INTERNAL_MHL_SUPPORT
    vMhlTriggerIntTask();
#else
    mdelay(1);
#endif
}
void hdmi_cec_int_handler(void)
{
    kal_uint32 ret=0;
    
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[hdmi_cec_int_handler]....\n");

    ret=pmic_config_interface(INT_STATUS1,0x1,PMIC_RG_INT_STATUS_HDMI_CEC_MASK,PMIC_RG_INT_STATUS_HDMI_CEC_SHIFT);

    upmu_set_rg_int_en_hdmi_cec(0);
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void vca15_int_handler(void)
{
    kal_uint32 ret=0;
    
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[vca15_int_handler]....\n");

    upmu_set_rg_pwmoc_ck_pdn(1);

    ret=pmic_config_interface(INT_STATUS1,0x1,0x1,8);

    upmu_set_rg_int_en_vca15(0);
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}    
void vsrmca15_int_handler(void)
{
    kal_uint32 ret=0;
    
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[vsrmca15_int_handler]....\n");
    
    upmu_set_rg_pwmoc_ck_pdn(1);

    ret=pmic_config_interface(INT_STATUS1,0x1,0x1,9);

    upmu_set_rg_int_en_vsrmca15(0);
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void vcore_int_handler(void)
{
    kal_uint32 ret=0;
    
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[vcore_int_handler]....\n");

    upmu_set_rg_pwmoc_ck_pdn(1);

    ret=pmic_config_interface(INT_STATUS1,0x1,0x1,10);

    upmu_set_rg_int_en_vcore(0);
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}
void vgpu_int_handler(void)
{
    kal_uint32 ret=0;
    
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[vgpu_int_handler]....\n");

    upmu_set_rg_pwmoc_ck_pdn(1);
    
    ret=pmic_config_interface(INT_STATUS1,0x1,0x1,11);

    upmu_set_rg_int_en_vgpu(0);
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}  
void vio18_int_handler(void)
{
    kal_uint32 ret=0;
    
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[vio18_int_handler]....\n");

    upmu_set_rg_pwmoc_ck_pdn(1);
    
    ret=pmic_config_interface(INT_STATUS1,0x1,0x1,12);

    upmu_set_rg_int_en_vio18(0);
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
} 
void vpca7_int_handler(void)
{
    kal_uint32 ret=0;
    
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[vpca7_int_handler]....\n");

    upmu_set_rg_pwmoc_ck_pdn(1);

    ret=pmic_config_interface(INT_STATUS1,0x1,0x1,13);

    upmu_set_rg_int_en_vpca7(0);
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}    
void vsrmca7_int_handler(void)
{
    kal_uint32 ret=0;
    
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[vsrmca7_int_handler]....\n");
    
    upmu_set_rg_pwmoc_ck_pdn(1);

    ret=pmic_config_interface(INT_STATUS1,0x1,0x1,14);

    upmu_set_rg_int_en_vsrmca7(0);
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}    

void vdrm_int_handler(void)
{
    kal_uint32 ret=0;
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[vdrm_int_handler]....\n");

    upmu_set_rg_pwmoc_ck_pdn(1);
    
    ret=pmic_config_interface(INT_STATUS1,0x1,0x1,15);    

    upmu_set_rg_int_en_vdrm(0);
    //make sure register operation is effective, no fake interrupt received
    mdelay(1);
}

static int pmic_thread_kthread(void *x)
{
    kal_uint32 ret=0;
    //kal_uint32 ret_val=0;
    //kal_uint32 reg_val=0;
    kal_uint32 int_status_val_0=0;
    kal_uint32 int_status_val_1=0;

    //xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_thread_kthread] enter\n");

    /* Run on a process content */
    while (1) {
        mutex_lock(&pmic_mutex);

        //xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_thread_kthread] running\n");

        //--------------------------------------------------------------------------------
        ret=pmic_read_interface(INT_STATUS0,(&int_status_val_0),0xFFFF,0x0);
        //xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[INT] int_status_val_0=0x%x\n", int_status_val_0);

        if( (((int_status_val_0)&(0x0001))>>0) == 1 )  { spkl_ab_int_handler();      }
        if( (((int_status_val_0)&(0x0002))>>1) == 1 )  { spkr_ab_int_handler();      }         
        if( (((int_status_val_0)&(0x0004))>>2) == 1 )  { spkl_int_handler();         }
        if( (((int_status_val_0)&(0x0008))>>3) == 1 )  { spkr_int_handler();         }
        if( (((int_status_val_0)&(0x0010))>>4) == 1 )  { bat_l_int_handler();        }
        if( (((int_status_val_0)&(0x0020))>>5) == 1 )  { bat_h_int_handler();        }
        if( (((int_status_val_0)&(0x0040))>>6) == 1 )  { fg_bat_l_int_handler();     }
        if( (((int_status_val_0)&(0x0080))>>7) == 1 )  { fg_bat_h_int_handler();     }
        if( (((int_status_val_0)&(0x0100))>>8) == 1 )  { watchdog_int_handler();     }
        if( (((int_status_val_0)&(0x0200))>>9) == 1 )  { pwrkey_int_handler();       }
        if( (((int_status_val_0)&(0x0400))>>10) == 1 ) { thr_l_int_handler();        }
        if( (((int_status_val_0)&(0x0800))>>11) == 1 ) { thr_h_int_handler();        }
        if( (((int_status_val_0)&(0x1000))>>12) == 1 ) { vbaton_undet_int_handler(); }
        if( (((int_status_val_0)&(0x2000))>>13) == 1 ) { bvalid_det_int_handler();   }
        if( (((int_status_val_0)&(0x4000))>>14) == 1 ) { chrdet_int_handler();       }
        if( (((int_status_val_0)&(0x8000))>>15) == 1 ) { ov_int_handler();           }                       
        //--------------------------------------------------------------------------------
        ret=pmic_read_interface(INT_STATUS1,(&int_status_val_1),0xFFFF,0x0);
        //xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[INT] int_status_val_1=0x%x\n", int_status_val_1);

        if( (((int_status_val_1)&(0x0001))>>0) == 1 )  { ldo_int_handler();          }
        if( (((int_status_val_1)&(0x0002))>>1) == 1 )  { homekey_int_handler();      }
#ifdef CONFIG_MTK_ACCDET
        if( (((int_status_val_1)&(0x0004))>>2) == 1 )  { accdet_int_handler();       }
#endif
        if( (((int_status_val_1)&(0x0008))>>3) == 1 )  { audio_int_handler();        }
        if( (((int_status_val_1)&(0x0010))>>4) == 1 )  { rtc_int_handler();          }
        if( (((int_status_val_1)&(0x0020))>>5) == 1 )  { pwrkey_rstb_int_handler();  }
        if( (((int_status_val_1)&(0x0040))>>6) == 1 )  { hdmi_sifm_int_handler();    }
        if( (((int_status_val_1)&(0x0080))>>7) == 1 )  { hdmi_cec_int_handler();     }
        if( (((int_status_val_1)&(0x0100))>>8) == 1 )  { vca15_int_handler();        }
        if( (((int_status_val_1)&(0x0200))>>9) == 1 )  { vsrmca15_int_handler();     }
        if( (((int_status_val_1)&(0x0400))>>10) == 1 ) { vcore_int_handler();        }
        if( (((int_status_val_1)&(0x0800))>>11) == 1 ) { vgpu_int_handler();         }
        if( (((int_status_val_1)&(0x1000))>>12) == 1 ) { vio18_int_handler();        }
        if( (((int_status_val_1)&(0x2000))>>13) == 1 ) { vpca7_int_handler();        }
        if( (((int_status_val_1)&(0x4000))>>14) == 1 ) { vsrmca7_int_handler();      }
        if( (((int_status_val_1)&(0x8000))>>15) == 1 ) { vdrm_int_handler();         }                   
        //--------------------------------------------------------------------------------

        //move this delay to individual int_handler, because some int_handler(e.g. HDMI) need fast response time
        //mdelay(1);
        
#ifndef CONFIG_MT8127_FPGA
        //to do, check EINT setting in dct tool     
        mt_eint_unmask(CUST_EINT_MT6323_PMIC_NUM);
#endif
        //set INT_EN, in PMIC_EINT_SETTING()
        cust_pmic_interrupt_en_setting();

        mutex_unlock(&pmic_mutex);

        wait_event(pmic_thread_wq, pmic_thread_timeout);

        pmic_thread_timeout=0;
    }

    return 0;
}

void mt6397_pmic_eint_irq(void)
{
    //xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "6397 get int\n");

    //pmic internal
    wake_up_pmic();

    return ;
}

void PMIC_EINT_SETTING(void)
{
#ifndef CONFIG_MT8127_FPGA
    //to do, check EINT setting in dct tool
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[PMIC_EINT_SETTING] start: CUST_EINT_MT6397_PMIC_NUM=%d\n",CUST_EINT_MT6323_PMIC_NUM);
#endif
    //ON/OFF interrupt
    cust_pmic_interrupt_en_setting();

#ifdef MTK_INTERNAL_MHL_SUPPORT	
    upmu_set_rg_int_en_hdmi_sifm(1);
#endif

    //GPIO Setting for early porting
    //mt_set_gpio_mode(GPIO52,GPIO_MODE_01); //EINT0 mode 1 on GPIO52
    mt_set_gpio_mode(GPIO_PMIC_EINT_PIN,GPIO_PMIC_EINT_PIN_M_EINT);
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[CUST_EINT] %d,%d for usage\n", GPIO_PMIC_EINT_PIN, GPIO_PMIC_EINT_PIN_M_EINT);

    //EINT setting
    //mt_eint_set_sens(       CUST_EINT_MT6320_PMIC_NUM,
    //                            CUST_EINT_MT6320_PMIC_SENSITIVE);
    //mt_eint_set_polarity(   CUST_EINT_MT6320_PMIC_NUM,
    //                            CUST_EINT_MT6320_PMIC_POLARITY);        // set positive polarity
#ifndef CONFIG_MT8127_FPGA
//to do, check EINT setting in dct tool     
    mt_eint_set_hw_debounce(CUST_EINT_MT6323_PMIC_NUM,
                                CUST_EINT_MT6323_PMIC_DEBOUNCE_CN);     // set debounce time
    mt_eint_registration(   CUST_EINT_MT6323_PMIC_NUM,
                            CUST_EINT_MT6323_PMIC_TYPE,                                
                                mt6397_pmic_eint_irq,
                                0);

    mt_eint_unmask(CUST_EINT_MT6323_PMIC_NUM);

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[CUST_EINT] CUST_EINT_MT6397_PMIC_NUM=%d\n", CUST_EINT_MT6323_PMIC_NUM);
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[CUST_EINT] CUST_EINT_MT6397_PMIC_DEBOUNCE_CN=%d\n", CUST_EINT_MT6323_PMIC_DEBOUNCE_CN);
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[CUST_EINT] CUST_EINT_MT6397_PMIC_TYPE=%d\n", CUST_EINT_MT6323_PMIC_TYPE);    
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[CUST_EINT] CUST_EINT_MT6397_PMIC_DEBOUNCE_EN=%d\n", CUST_EINT_MT6323_PMIC_DEBOUNCE_EN);
#endif    
}

void PMIC_DUMP_ALL_Register(void)
{
    kal_uint32 i=0;
    kal_uint32 ret=0;
    kal_uint32 reg_val=0;

    for (i=0;i<0xFFFF;i++)
    {
        ret=pmic_read_interface(i,&reg_val,0xFFFF,0);
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Reg[0x%x]=0x%x\n", i, reg_val);
    }
}

//==============================================================================
// PMIC read/write APIs
//==============================================================================
#define CONFIG_PMIC_HW_ACCESS_EN

#define PMIC_REG_NUM 0xFFFF

static DEFINE_MUTEX(pmic_access_mutex);
//U32 pmic6320_reg[PMIC_REG_NUM] = {0};

U32 pmic_read_interface (U32 RegNum, U32 *val, U32 MASK, U32 SHIFT)
{
    U32 return_value = 0;

#if defined(CONFIG_PMIC_HW_ACCESS_EN)
    U32 pmic_reg = 0;
    U32 rdata;

    mutex_lock(&pmic_access_mutex);

    //mt6320_read_byte(RegNum, &pmic6320_reg);
    return_value= pwrap_wacs2(0, (RegNum), 0, &rdata);
    pmic_reg=rdata;
    if(return_value!=0)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_read_interface] Reg[%x]= pmic_wrap read data fail\n", RegNum);
        mutex_unlock(&pmic_access_mutex);
        return return_value;
    }
    //xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[pmic_read_interface] Reg[%x]=0x%x\n", RegNum, pmic_reg);

    pmic_reg &= (MASK << SHIFT);
    *val = (pmic_reg >> SHIFT);
    //xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[pmic_read_interface] val=0x%x\n", *val);

    mutex_unlock(&pmic_access_mutex);
#else
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_read_interface] Can not access HW PMIC\n");
#endif

    return return_value;
}

U32 pmic_config_interface (U32 RegNum, U32 val, U32 MASK, U32 SHIFT)
{
    U32 return_value = 0;

#if defined(CONFIG_PMIC_HW_ACCESS_EN)
    U32 pmic_reg = 0;
    U32 rdata;

    mutex_lock(&pmic_access_mutex);

    //1. mt6320_read_byte(RegNum, &pmic_reg);
    return_value= pwrap_wacs2(0, (RegNum), 0, &rdata);
    pmic_reg=rdata;
    if(return_value!=0)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_config_interface] Reg[%x]= pmic_wrap read data fail\n", RegNum);
        mutex_unlock(&pmic_access_mutex);
        return return_value;
    }
    //xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[pmic_config_interface] Reg[%x]=0x%x\n", RegNum, pmic_reg);

    pmic_reg &= ~(MASK << SHIFT);
    pmic_reg |= (val << SHIFT);

    //2. mt6320_write_byte(RegNum, pmic_reg);
    return_value= pwrap_wacs2(1, (RegNum), pmic_reg, &rdata);
    if(return_value!=0)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_config_interface] Reg[%x]= pmic_wrap read data fail\n", RegNum);
        mutex_unlock(&pmic_access_mutex);
        return return_value;
    }
    //xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[pmic_config_interface] write Reg[%x]=0x%x\n", RegNum, pmic_reg);

    #if 0
    //3. Double Check
    //mt6320_read_byte(RegNum, &pmic_reg);
    return_value= pwrap_wacs2(0, (RegNum), 0, &rdata);
    pmic_reg=rdata;
    if(return_value!=0)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_config_interface] Reg[%x]= pmic_wrap write data fail\n", RegNum);
        mutex_unlock(&pmic_access_mutex);
        return return_value;
    }
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[pmic_config_interface] Reg[%x]=0x%x\n", RegNum, pmic_reg);
    #endif

    mutex_unlock(&pmic_access_mutex);
#else
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_config_interface] Can not access HW PMIC\n");
#endif

    return return_value;
}

//==============================================================================
// PMIC read/write APIs : nolock
//==============================================================================
U32 pmic_read_interface_nolock (U32 RegNum, U32 *val, U32 MASK, U32 SHIFT)
{
    U32 return_value = 0;

#if defined(CONFIG_PMIC_HW_ACCESS_EN)
    U32 pmic_reg = 0;
    U32 rdata;

    //mt6320_read_byte(RegNum, &pmic6320_reg);
    return_value= pwrap_wacs2(0, (RegNum), 0, &rdata);
    pmic_reg=rdata;
    if(return_value!=0)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_read_interface] Reg[%x]= pmic_wrap read data fail\n", RegNum);
        return return_value;
    }
    //xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[pmic_read_interface] Reg[%x]=0x%x\n", RegNum, pmic_reg);

    pmic_reg &= (MASK << SHIFT);
    *val = (pmic_reg >> SHIFT);
    //xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[pmic_read_interface] val=0x%x\n", *val);
#else
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_read_interface] Can not access HW PMIC\n");
#endif

    return return_value;
}

U32 pmic_config_interface_nolock (U32 RegNum, U32 val, U32 MASK, U32 SHIFT)
{
    U32 return_value = 0;

#if defined(CONFIG_PMIC_HW_ACCESS_EN)
    U32 pmic_reg = 0;
    U32 rdata;

    //1. mt6320_read_byte(RegNum, &pmic_reg);
    return_value= pwrap_wacs2(0, (RegNum), 0, &rdata);
    pmic_reg=rdata;
    if(return_value!=0)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_config_interface] Reg[%x]= pmic_wrap read data fail\n", RegNum);
        return return_value;
    }
    //xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[pmic_config_interface] Reg[%x]=0x%x\n", RegNum, pmic_reg);

    pmic_reg &= ~(MASK << SHIFT);
    pmic_reg |= (val << SHIFT);

    //2. mt6320_write_byte(RegNum, pmic_reg);
    return_value= pwrap_wacs2(1, (RegNum), pmic_reg, &rdata);
    if(return_value!=0)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_config_interface] Reg[%x]= pmic_wrap read data fail\n", RegNum);
        return return_value;
    }
    //xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[pmic_config_interface] write Reg[%x]=0x%x\n", RegNum, pmic_reg);

    #if 0
    //3. Double Check
    //mt6320_read_byte(RegNum, &pmic_reg);
    return_value= pwrap_wacs2(0, (RegNum), 0, &rdata);
    pmic_reg=rdata;
    if(return_value!=0)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_config_interface] Reg[%x]= pmic_wrap write data fail\n", RegNum);
        return return_value;
    }
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[pmic_config_interface] Reg[%x]=0x%x\n", RegNum, pmic_reg);
    #endif

#else
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_config_interface] Can not access HW PMIC\n");
#endif

    return return_value;
}

//==============================================================================
// mt-pmic dev_attr APIs
//==============================================================================
U32 g_reg_value=0;
static ssize_t show_pmic_access(struct device *dev,struct device_attribute *attr, char *buf)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[show_pmic_access] 0x%x\n", g_reg_value);
    return sprintf(buf, "%u\n", g_reg_value);
}
static ssize_t store_pmic_access(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    int ret=0;
    char *pvalue = NULL;
    U32 reg_value = 0;
    U32 reg_address = 0;
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[store_pmic_access] \n");
    if(buf != NULL && size != 0)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[store_pmic_access] buf is %s and size is %d \n",buf,size);
        reg_address = simple_strtoul(buf,&pvalue,16);

        if(size > 5)
        {
            reg_value = simple_strtoul((pvalue+1),NULL,16);
            xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[store_pmic_access] write PMU reg 0x%x with value 0x%x !\n",reg_address,reg_value);
            ret=pmic_config_interface(reg_address, reg_value, 0xFFFF, 0x0);
        }
        else
        {
            ret=pmic_read_interface(reg_address, &g_reg_value, 0xFFFF, 0x0);
            xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[store_pmic_access] read PMU reg 0x%x with value 0x%x !\n",reg_address,g_reg_value);
            xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[store_pmic_access] Please use \"cat pmic_access\" to get value\r\n");
        }
    }
    return size;
}
static DEVICE_ATTR(pmic_access, 0664, show_pmic_access, store_pmic_access); //664

//==============================================================================
// LDO EN APIs
//==============================================================================
void dct_pmic_VIO28_enable(kal_bool dctEnable)
{
#if 1
    /* for Sensor co-power with other devices */
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VIO28_enable] No enable can setting!\n");
#else
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VIO28_enable] %d\n", dctEnable);

    if(dctEnable == KAL_TRUE)
    {
        upmu_set_vio28_en(1);
    }
    else
    {
        upmu_set_vio28_en(0);
    }
#endif  
}

void dct_pmic_VUSB_enable(kal_bool dctEnable)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VUSB_enable] %d\n", dctEnable);

    if(dctEnable == KAL_TRUE)
    {
        upmu_set_rg_vusb_en(1);
    }
    else
    {
        upmu_set_rg_vusb_en(0);
    }
}

void dct_pmic_VMC_enable(kal_bool dctEnable)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VMC_enable] %d\n", dctEnable);

    if(dctEnable == KAL_TRUE)
    {
        upmu_set_rg_vmc_en(1);
    }
    else
    {
        upmu_set_rg_vmc_en(0);
    }
}

void dct_pmic_VMCH_enable(kal_bool dctEnable)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VMCH_enable] %d\n", dctEnable);

    if(dctEnable == KAL_TRUE)
    {
        upmu_set_rg_vmch_en(1);
    }
    else
    {
        upmu_set_rg_vmch_en(0);
    }
}

void dct_pmic_VEMC_3V3_enable(kal_bool dctEnable)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VEMC_3V3_enable] %d\n", dctEnable);

    if(dctEnable == KAL_TRUE)
    {
        upmu_set_rg_vemc_3v3_en(1);
    }
    else
    {
        upmu_set_rg_vemc_3v3_en(0);
    }
}

void dct_pmic_VCAMD_enable(kal_bool dctEnable)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VCAMD_enable] %d\n", dctEnable);

    if(dctEnable == KAL_TRUE)
    {
        upmu_set_rg_vcamd_sw_en(1);
    }
    else
    {
        upmu_set_rg_vcamd_sw_en(0);
    }
}

void dct_pmic_VCAMIO_enable(kal_bool dctEnable)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VCAMIO_enable] %d\n", dctEnable);

    if(dctEnable == KAL_TRUE)
    {
        upmu_set_rg_vcamio_sw_en(1);
    }
    else
    {
        upmu_set_rg_vcamio_sw_en(0);
    }
}

void dct_pmic_VCAMAF_enable(kal_bool dctEnable)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VCAMAF_enable] %d\n", dctEnable);

    if(dctEnable == KAL_TRUE)
    {
        upmu_set_rg_vcamaf_sw_en(1);
    }
    else
    {
        upmu_set_rg_vcamaf_sw_en(0);
    }
}

void dct_pmic_VGP4_enable(kal_bool dctEnable)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VGP4_enable] %d\n", dctEnable);

    if(dctEnable == KAL_TRUE)
    {
        upmu_set_rg_vgp4_sw_en(1);
    }
    else
    {
        upmu_set_rg_vgp4_sw_en(0);
    }
}

void dct_pmic_VGP5_enable(kal_bool dctEnable)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VGP5_enable] %d\n", dctEnable);

    if(dctEnable == KAL_TRUE)
    {
        upmu_set_rg_vgp5_sw_en(1);
    }
    else
    {
        upmu_set_rg_vgp5_sw_en(0);
    }
}

void dct_pmic_VGP6_enable(kal_bool dctEnable)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VGP6_enable] %d\n", dctEnable);

    if(dctEnable == KAL_TRUE)
    {
        upmu_set_rg_vgp6_sw_en(1);
    }
    else
    {
        upmu_set_rg_vgp6_sw_en(0);
    }
}

void dct_pmic_VIBR_enable(kal_bool dctEnable)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VIBR_enable] %d\n", dctEnable);

    if(dctEnable == KAL_TRUE)
    {
        upmu_set_rg_vibr_en(1);
    }
    else
    {
        upmu_set_rg_vibr_en(0);
    }
}

void dct_pmic_VRTC_enable(kal_bool dctEnable)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VRTC_enable] %d\n", dctEnable);

    if(dctEnable == KAL_TRUE)
    {
        upmu_set_vrtc_en(1);
    }
    else
    {
        upmu_set_vrtc_en(0);
    }
}

void dct_pmic_VTCXO_enable(kal_bool dctEnable)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VTCXO_enable] %d\n", dctEnable);

    if(dctEnable == KAL_TRUE)
    {
        upmu_set_rg_vtcxo_en(1);
    }
    else
    {
        upmu_set_rg_vtcxo_en(0);
    }
}

void dct_pmic_VA28_enable(kal_bool dctEnable)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VA28_enable] %d\n", dctEnable);

    if(dctEnable == KAL_TRUE)
    {
        upmu_set_rg_va28_en(1);
    }
    else
    {
        upmu_set_rg_va28_en(0);
    }
}

void dct_pmic_VCAMA_enable(kal_bool dctEnable)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VCAMA_enable] %d\n", dctEnable);

    if(dctEnable == KAL_TRUE)
    {
        upmu_set_rg_vcama_en(1);
    }
    else
    {
        upmu_set_rg_vcama_en(0);
    }
}

//==============================================================================
// LDO SEL APIs
//==============================================================================
void dct_pmic_VIO28_sel(kal_uint32 volt)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VIO28_enable] No voltage can setting!\n");
}

void dct_pmic_VUSB_sel(kal_uint32 volt)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VUSB_sel] No voltage can setting!\n");
}

void dct_pmic_VMC_sel(kal_uint32 volt)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "****[dct_pmic_VMC_sel] value=%d \n", volt);

    if(volt == VOL_DEFAULT)     { upmu_set_rg_vmc_vosel(1);}
    else if(volt == VOL_3300){ upmu_set_rg_vmc_vosel(1);}
    else if(volt == VOL_1800){ upmu_set_rg_vmc_vosel(0);}
    else{
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Error Setting %d. DO nothing.\r\n", volt);
    }
}

void dct_pmic_VMCH_sel(kal_uint32 volt)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "****[dct_pmic_VMCH_sel] value=%d \n", volt);

    if(volt == VOL_DEFAULT)     { upmu_set_rg_vmch_vosel(1);}
    else if(volt == VOL_3000){ upmu_set_rg_vmch_vosel(0);}
    else if(volt == VOL_3300){ upmu_set_rg_vmch_vosel(1);}
    else{
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Error Setting %d. DO nothing.\r\n", volt);
    }
}

void dct_pmic_VEMC_3V3_sel(kal_uint32 volt)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "****[dct_pmic_VEMC_3V3_sel] value=%d \n", volt);
    
    if(volt == VOL_DEFAULT)  { upmu_set_rg_vemc_3v3_vosel(1);}
    else if(volt == VOL_3000){ upmu_set_rg_vemc_3v3_vosel(0);}
    else if(volt == VOL_3300){ upmu_set_rg_vemc_3v3_vosel(1);}
    else{
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Error Setting %d. DO nothing.\r\n", volt);
    }    
}

void dct_pmic_VCAMD_sel(kal_uint32 volt)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "****[dct_pmic_VCAMD_sel] value=%d \n", volt);

    if(volt == VOL_DEFAULT)     { upmu_set_rg_vcamd_vosel(5);}
    else if(volt == VOL_1200){ upmu_set_rg_vcamd_vosel(0);}
    else if(volt == VOL_1300){ upmu_set_rg_vcamd_vosel(1);}
    else if(volt == VOL_1500){ upmu_set_rg_vcamd_vosel(2);}
    else if(volt == VOL_1800){ upmu_set_rg_vcamd_vosel(3);}
    else if(volt == VOL_2500){ upmu_set_rg_vcamd_vosel(4);}
    else if(volt == VOL_2800){ upmu_set_rg_vcamd_vosel(5);}
    else if(volt == VOL_3000){ upmu_set_rg_vcamd_vosel(6);}
    else if(volt == VOL_3300){ upmu_set_rg_vcamd_vosel(7);}
    else{
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Error Setting %d. DO nothing.\r\n", volt);
    }
}

void dct_pmic_VCAMIO_sel(kal_uint32 volt)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "****[dct_pmic_VCAMIO_sel] value=%d \n", volt);

    if(volt == VOL_DEFAULT)     { upmu_set_rg_vcamio_vosel(5);}
    else if(volt == VOL_1200){ upmu_set_rg_vcamio_vosel(0);}
    else if(volt == VOL_1300){ upmu_set_rg_vcamio_vosel(1);}
    else if(volt == VOL_1500){ upmu_set_rg_vcamio_vosel(2);}
    else if(volt == VOL_1800){ upmu_set_rg_vcamio_vosel(3);}
    else if(volt == VOL_2500){ upmu_set_rg_vcamio_vosel(4);}
    else if(volt == VOL_2800){ upmu_set_rg_vcamio_vosel(5);}
    else if(volt == VOL_3000){ upmu_set_rg_vcamio_vosel(6);}
    else if(volt == VOL_3300){ upmu_set_rg_vcamio_vosel(7);}
    else{
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Error Setting %d. DO nothing.\r\n", volt);
    }
}

void dct_pmic_VCAMAF_sel(kal_uint32 volt)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "****[dct_pmic_VCAMAF_sel] value=%d \n", volt);

    if(volt == VOL_DEFAULT)     { upmu_set_rg_vcamaf_vosel(5);}
    else if(volt == VOL_1200){ upmu_set_rg_vcamaf_vosel(0);}
    else if(volt == VOL_1300){ upmu_set_rg_vcamaf_vosel(1);}
    else if(volt == VOL_1500){ upmu_set_rg_vcamaf_vosel(2);}
    else if(volt == VOL_1800){ upmu_set_rg_vcamaf_vosel(3);}
    else if(volt == VOL_2500){ upmu_set_rg_vcamaf_vosel(4);}
    else if(volt == VOL_2800){ upmu_set_rg_vcamaf_vosel(5);}
    else if(volt == VOL_3000){ upmu_set_rg_vcamaf_vosel(6);}
    else if(volt == VOL_3300){ upmu_set_rg_vcamaf_vosel(7);}
    else{
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Error Setting %d. DO nothing.\r\n", volt);
    }
}

void dct_pmic_VGP4_sel(kal_uint32 volt)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "****[dct_pmic_VGP4_sel] value=%d \n", volt);

    if(volt == VOL_DEFAULT)     { upmu_set_rg_vgp4_vosel(5);}
    else if(volt == VOL_1200){ upmu_set_rg_vgp4_vosel(0);}
    else if(volt == VOL_1300){ upmu_set_rg_vgp4_vosel(1);}
    else if(volt == VOL_1500){ upmu_set_rg_vgp4_vosel(2);}
    else if(volt == VOL_1800){ upmu_set_rg_vgp4_vosel(3);}
    else if(volt == VOL_2500){ upmu_set_rg_vgp4_vosel(4);}
    else if(volt == VOL_2800){ upmu_set_rg_vgp4_vosel(5);}
    else if(volt == VOL_3000){ upmu_set_rg_vgp4_vosel(6);}
    else if(volt == VOL_3300){ upmu_set_rg_vgp4_vosel(7);}
    else{
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Error Setting %d. DO nothing.\r\n", volt);
    }
}

void dct_pmic_VGP5_sel(kal_uint32 volt)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "****[dct_pmic_VGP5_sel] value=%d \n", volt);

    if(volt == VOL_DEFAULT)     { upmu_set_rg_vgp5_vosel(5);}
    else if(volt == VOL_1200){ upmu_set_rg_vgp5_vosel(0);}
    else if(volt == VOL_1300){ upmu_set_rg_vgp5_vosel(1);}
    else if(volt == VOL_1500){ upmu_set_rg_vgp5_vosel(2);}
    else if(volt == VOL_1800){ upmu_set_rg_vgp5_vosel(3);}
    else if(volt == VOL_2500){ upmu_set_rg_vgp5_vosel(4);}
    else if(volt == VOL_2800){ upmu_set_rg_vgp5_vosel(5);}
    else if(volt == VOL_3000){ upmu_set_rg_vgp5_vosel(6);}
    else if(volt == VOL_2000){ upmu_set_rg_vgp5_vosel(7);}
    else{
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Error Setting %d. DO nothing.\r\n", volt);
    }
}

void dct_pmic_VGP6_sel(kal_uint32 volt)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "****[dct_pmic_VGP6_sel] value=%d \n", volt);

    if(volt == VOL_DEFAULT)     { upmu_set_rg_vgp6_vosel(5);}
    else if(volt == VOL_1200){ upmu_set_rg_vgp6_vosel(0);}
    else if(volt == VOL_1300){ upmu_set_rg_vgp6_vosel(1);}
    else if(volt == VOL_1500){ upmu_set_rg_vgp6_vosel(2);}
    else if(volt == VOL_1800){ upmu_set_rg_vgp6_vosel(3);}
    else if(volt == VOL_2500){ upmu_set_rg_vgp6_vosel(4);}
    else if(volt == VOL_2800){ upmu_set_rg_vgp6_vosel(5);}
    else if(volt == VOL_3000){ upmu_set_rg_vgp6_vosel(6);}
    else if(volt == VOL_3300){ upmu_set_rg_vgp6_vosel(7);}
    else{
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Error Setting %d. DO nothing.\r\n", volt);
    }
}

void dct_pmic_VIBR_sel(kal_uint32 volt)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "****[dct_pmic_VIBR_sel] value=%d \n", volt);

    if(volt == VOL_DEFAULT)     { upmu_set_rg_vibr_vosel(6);}
    else if(volt == VOL_1300){ upmu_set_rg_vibr_vosel(0);}
    else if(volt == VOL_1500){ upmu_set_rg_vibr_vosel(1);}
    else if(volt == VOL_1800){ upmu_set_rg_vibr_vosel(2);}
    else if(volt == VOL_2000){ upmu_set_rg_vibr_vosel(3);}
    else if(volt == VOL_2500){ upmu_set_rg_vibr_vosel(4);}
    else if(volt == VOL_2800){ upmu_set_rg_vibr_vosel(5);}
    else if(volt == VOL_3000){ upmu_set_rg_vibr_vosel(6);}
    else if(volt == VOL_3300){ upmu_set_rg_vibr_vosel(7);}
    else{
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Error Setting %d. DO nothing.\r\n", volt);
    }
}

void dct_pmic_VRTC_sel(kal_uint32 volt)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VRTC_sel] No voltage can setting!\n");
}

void dct_pmic_VTCXO_sel(kal_uint32 volt)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VTCXO_sel] No voltage can setting!\n");
}

void dct_pmic_VA28_sel(kal_uint32 volt)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[dct_pmic_VA28_sel] No voltage can setting!\n");
}

void dct_pmic_VCAMA_sel(kal_uint32 volt)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "****[dct_pmic_VCAMA_sel] value=%d \n", volt);

    if(volt == VOL_DEFAULT)     { upmu_set_rg_vcama_vosel(3);}
    else if(volt == VOL_1500){ upmu_set_rg_vcama_vosel(0);}
    else if(volt == VOL_1800){ upmu_set_rg_vcama_vosel(1);}
    else if(volt == VOL_2500){ upmu_set_rg_vcama_vosel(2);}
    else if(volt == VOL_2800){ upmu_set_rg_vcama_vosel(3);}
    else{
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Error Setting %d. DO nothing.\r\n", volt);
    }
}

//==============================================================================
// LDO EN & SEL common API
//==============================================================================
void pmic_ldo_enable(MT65XX_POWER powerId, kal_bool powerEnable)
{
    //xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[pmic_ldo_enable] Receive powerId %d, action is %d\n", powerId, powerEnable);

    //Need integrate with DCT : using DCT APIs

    if(     powerId == MT65XX_POWER_LDO_VIO28)      { dct_pmic_VIO28_enable(powerEnable); }
    else if(powerId == MT65XX_POWER_LDO_VUSB)       { dct_pmic_VUSB_enable(powerEnable); }
    else if(powerId == MT65XX_POWER_LDO_VMC)        { dct_pmic_VMC_enable(powerEnable); }
    else if(powerId == MT65XX_POWER_LDO_VMCH)        { dct_pmic_VMCH_enable(powerEnable); }
    else if(powerId == MT65XX_POWER_LDO_VEMC_3V3)    { dct_pmic_VEMC_3V3_enable(powerEnable); }
    else if(powerId == MT65XX_POWER_LDO_VCAMD)        { dct_pmic_VCAMD_enable(powerEnable); }
    else if(powerId == MT65XX_POWER_LDO_VCAMIO)        { dct_pmic_VCAMIO_enable(powerEnable); }
    else if(powerId == MT65XX_POWER_LDO_VCAMAF)        { dct_pmic_VCAMAF_enable(powerEnable); }
    else if(powerId == MT65XX_POWER_LDO_VGP4)        { dct_pmic_VGP4_enable(powerEnable); }
    else if(powerId == MT65XX_POWER_LDO_VGP5)        { dct_pmic_VGP5_enable(powerEnable); }
    else if(powerId == MT65XX_POWER_LDO_VGP6)        { dct_pmic_VGP6_enable(powerEnable); }
    else if(powerId == MT65XX_POWER_LDO_VIBR)        { dct_pmic_VIBR_enable(powerEnable); }
    else if(powerId == MT65XX_POWER_LDO_VRTC)        { dct_pmic_VRTC_enable(powerEnable); }    
    else if(powerId == MT65XX_POWER_LDO_VTCXO)        { dct_pmic_VTCXO_enable(powerEnable); }
    else if(powerId == MT65XX_POWER_LDO_VA28)        { dct_pmic_VA28_enable(powerEnable); }
    else if(powerId == MT65XX_POWER_LDO_VCAMA)        { dct_pmic_VCAMA_enable(powerEnable); }
    else
    {
        xlog_printk(ANDROID_LOG_WARN, "Power/PMIC", "[pmic_ldo_enable] UnKnown powerId (%d)\n", powerId);
    }
}

void pmic_ldo_vol_sel(MT65XX_POWER powerId, MT65XX_POWER_VOLTAGE powerVolt)
{
    //xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[pmic_ldo_vol_sel] Receive powerId %d, action is %d\n", powerId, powerVolt);

    //Need integrate with DCT : using DCT APIs

    if(     powerId == MT65XX_POWER_LDO_VIO28)      {    dct_pmic_VIO28_sel(powerVolt); }
    else if(powerId == MT65XX_POWER_LDO_VUSB)       {    dct_pmic_VUSB_sel(powerVolt); }
    else if(powerId == MT65XX_POWER_LDO_VMC)        {    dct_pmic_VMC_sel(powerVolt); }
    else if(powerId == MT65XX_POWER_LDO_VMCH)        {    dct_pmic_VMCH_sel(powerVolt); }
    else if(powerId == MT65XX_POWER_LDO_VEMC_3V3)    {    dct_pmic_VEMC_3V3_sel(powerVolt); }
    else if(powerId == MT65XX_POWER_LDO_VCAMD)        {    dct_pmic_VCAMD_sel(powerVolt); }
    else if(powerId == MT65XX_POWER_LDO_VCAMIO)        {    dct_pmic_VCAMIO_sel(powerVolt); }
    else if(powerId == MT65XX_POWER_LDO_VCAMAF)        {    dct_pmic_VCAMAF_sel(powerVolt); }
    else if(powerId == MT65XX_POWER_LDO_VGP4)        {    dct_pmic_VGP4_sel(powerVolt); }
    else if(powerId == MT65XX_POWER_LDO_VGP5)        {    dct_pmic_VGP5_sel(powerVolt); }
    else if(powerId == MT65XX_POWER_LDO_VGP6)        {    dct_pmic_VGP6_sel(powerVolt); }    
    else if(powerId == MT65XX_POWER_LDO_VIBR)        {    dct_pmic_VIBR_sel(powerVolt); }
    else if(powerId == MT65XX_POWER_LDO_VRTC)        {    dct_pmic_VRTC_sel(powerVolt); }    
    else if(powerId == MT65XX_POWER_LDO_VTCXO)        {    dct_pmic_VTCXO_sel(powerVolt); }
    else if(powerId == MT65XX_POWER_LDO_VA28)        {    dct_pmic_VA28_sel(powerVolt); }
    else if(powerId == MT65XX_POWER_LDO_VCAMA)        {    dct_pmic_VCAMA_sel(powerVolt); }
    else
    {
        xlog_printk(ANDROID_LOG_WARN, "Power/PMIC", "[pmic_ldo_ldo_vol_sel] UnKnown powerId (%d)\n", powerId);
    }
}

//==============================================================================
// EM
//==============================================================================
static ssize_t show_BUCK_VPCA7_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    //kal_uint32 ret=0;
    //kal_uint32 reg_address = VPCA7_CON7;
    //kal_uint32 reg_val=0;
    
    //ret = pmic_read_interface(reg_address, &reg_val, 0x01, 13);    
    //ret_value = reg_val;
    ret_value = upmu_get_qi_vpca7_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] BUCK_VPCA7_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VPCA7_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(BUCK_VPCA7_STATUS, 0664, show_BUCK_VPCA7_STATUS, store_BUCK_VPCA7_STATUS);

static ssize_t show_BUCK_VSRMCA7_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x23A;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 13);
    ret_value = reg_val;
    */
    ret_value = upmu_get_qi_vsrmca7_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] BUCK_VSRMCA7_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VSRMCA7_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(BUCK_VSRMCA7_STATUS, 0664, show_BUCK_VSRMCA7_STATUS, store_BUCK_VSRMCA7_STATUS);

static ssize_t show_BUCK_VCA15_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    //kal_uint32 ret=0;
    //kal_uint32 reg_address = VCA15_CON7;
    //kal_uint32 reg_val=0;
    
    //ret = pmic_read_interface(reg_address, &reg_val, 0x01, 13);    
    //ret_value = reg_val;
    ret_value = upmu_get_qi_vca15_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] BUCK_VCA15_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VCA15_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(BUCK_VCA15_STATUS, 0664, show_BUCK_VCA15_STATUS, store_BUCK_VCA15_STATUS);

static ssize_t show_BUCK_VSRMCA15_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x23A;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 13);
    ret_value = reg_val;
    */
    ret_value = upmu_get_qi_vsrmca15_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] BUCK_VSRMCA15_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VSRMCA15_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(BUCK_VSRMCA15_STATUS, 0664, show_BUCK_VSRMCA15_STATUS, store_BUCK_VSRMCA15_STATUS);

static ssize_t show_BUCK_VCORE_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x266;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 13);
    ret_value = reg_val;
    */
    ret_value = upmu_get_qi_vcore_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] BUCK_VCORE_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VCORE_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(BUCK_VCORE_STATUS, 0664, show_BUCK_VCORE_STATUS, store_BUCK_VCORE_STATUS);

static ssize_t show_BUCK_VDRM_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x28C;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 13);
    ret_value = reg_val;
    */
    ret_value = upmu_get_qi_vdrm_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] BUCK_VDRM_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VDRM_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(BUCK_VDRM_STATUS, 0664, show_BUCK_VDRM_STATUS, store_BUCK_VDRM_STATUS);

static ssize_t show_BUCK_VIO18_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x30E;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 13);
    ret_value = reg_val;
    */
    ret_value = upmu_get_qi_vio18_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] BUCK_VIO18_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VIO18_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(BUCK_VIO18_STATUS, 0664, show_BUCK_VIO18_STATUS, store_BUCK_VIO18_STATUS);

static ssize_t show_BUCK_VGPU_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x388;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 13);
    ret_value = reg_val;
    */
    ret_value = upmu_get_qi_vgpu_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] BUCK_VGPU_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VGPU_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(BUCK_VGPU_STATUS, 0664, show_BUCK_VGPU_STATUS, store_BUCK_VGPU_STATUS);

static ssize_t show_LDO_VIO28_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x420;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 15);
    ret_value = reg_val;
    */
    ret_value = upmu_get_qi_vio28_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VIO28_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VIO28_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VIO28_STATUS, 0664, show_LDO_VIO28_STATUS, store_LDO_VIO28_STATUS);

static ssize_t show_LDO_VUSB_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x422;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 15);
    ret_value = reg_val;
    */
    ret_value = upmu_get_qi_vusb_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VUSB_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VUSB_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VUSB_STATUS, 0664, show_LDO_VUSB_STATUS, store_LDO_VUSB_STATUS);

static ssize_t show_LDO_VMC_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x424;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 15);
    ret_value = reg_val;
    */
    ret_value = upmu_get_qi_vmc_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VMC_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VMC_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VMC_STATUS, 0664, show_LDO_VMC_STATUS, store_LDO_VMC_STATUS);

static ssize_t show_LDO_VMCH_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x426;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 15);
    ret_value = reg_val;
    */
    ret_value = upmu_get_qi_vmch_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VMCH_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VMCH_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VMCH_STATUS, 0664, show_LDO_VMCH_STATUS, store_LDO_VMCH_STATUS);

static ssize_t show_LDO_VEMC_3V3_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x428;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 15);
    ret_value = reg_val;
    */
    ret_value = upmu_get_qi_vemc_3v3_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VEMC_3V3_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VEMC_3V3_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VEMC_3V3_STATUS, 0664, show_LDO_VEMC_3V3_STATUS, store_LDO_VEMC_3V3_STATUS);

static ssize_t show_LDO_VCAMD_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x42A;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 15);
    ret_value = reg_val;
    */
    ret_value = upmu_get_status_vcamd_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VCAMD_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VCAMD_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VCAMD_STATUS, 0664, show_LDO_VCAMD_STATUS, store_LDO_VCAMD_STATUS);

static ssize_t show_LDO_VCAMIO_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x42C;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 15);
    ret_value = reg_val;
    */
    ret_value = upmu_get_status_vcamio_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VCAMIO_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VCAMIO_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VCAMIO_STATUS, 0664, show_LDO_VCAMIO_STATUS, store_LDO_VCAMIO_STATUS);

static ssize_t show_LDO_VCAMAF_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x42E;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 15);
    ret_value = reg_val;
    */
    ret_value = upmu_get_status_vcamaf_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VCAMAF_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VCAMAF_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VCAMAF_STATUS, 0664, show_LDO_VCAMAF_STATUS, store_LDO_VCAMAF_STATUS);

static ssize_t show_LDO_VGP4_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x430;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 15);
    ret_value = reg_val;
    */
    ret_value = upmu_get_status_vgp4_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VGP4_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VGP4_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VGP4_STATUS, 0664, show_LDO_VGP4_STATUS, store_LDO_VGP4_STATUS);

static ssize_t show_LDO_VGP5_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x432;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 15);
    ret_value = reg_val;
    */
    ret_value = upmu_get_status_vgp5_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VGP5_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VGP5_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VGP5_STATUS, 0664, show_LDO_VGP5_STATUS, store_LDO_VGP5_STATUS);

static ssize_t show_LDO_VGP6_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x434;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 15);
    ret_value = reg_val;
    */
    ret_value = upmu_get_status_vgp6_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VGP6_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VGP6_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VGP6_STATUS, 0664, show_LDO_VGP6_STATUS, store_LDO_VGP6_STATUS);

static ssize_t show_LDO_VIBR_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x466;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 15);
    ret_value = reg_val;
    */
    ret_value = upmu_get_status_vibr_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VIBR_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VIBR_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VIBR_STATUS, 0664, show_LDO_VIBR_STATUS, store_LDO_VIBR_STATUS);

static ssize_t show_LDO_VRTC_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x43A;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 15);
    ret_value = reg_val;
    */
    ret_value = upmu_get_qi_vrtc_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VRTC_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VRTC_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VRTC_STATUS, 0664, show_LDO_VRTC_STATUS, store_LDO_VRTC_STATUS);

static ssize_t show_LDO_VTCXO_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x402;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 15);
    ret_value = reg_val;
    */
    ret_value = upmu_get_qi_vtcxo_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VTCXO_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VTCXO_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VTCXO_STATUS, 0664, show_LDO_VTCXO_STATUS, store_LDO_VTCXO_STATUS);

static ssize_t show_LDO_VA28_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x406;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 15);
    ret_value = reg_val;
    */
    ret_value = upmu_get_qi_va28_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VA28_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VA28_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VA28_STATUS, 0664, show_LDO_VA28_STATUS, store_LDO_VA28_STATUS);

static ssize_t show_LDO_VCAMA_STATUS(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    /*
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x408;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 15);
    ret_value = reg_val;
    */
    ret_value = upmu_get_status_vcama_en();
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VCAMA_STATUS : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VCAMA_STATUS(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VCAMA_STATUS, 0664, show_LDO_VCAMA_STATUS, store_LDO_VCAMA_STATUS);

// voltage ---------------------------------------------------------------------------------

static ssize_t show_BUCK_VPCA7_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    //kal_uint32 ret=0;
    //kal_uint32 reg_address=0x21E;
    kal_uint32 reg_val=0;
    
    //ret = pmic_read_interface(reg_address, &reg_val, 0x7F, 0);
    reg_val = upmu_get_ni_vpca7_vosel();
    ret_value = 70000 + (reg_val*625);
    ret_value = ret_value / 100;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] BUCK_VPCA7_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VPCA7_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(BUCK_VPCA7_VOLTAGE, 0664, show_BUCK_VPCA7_VOLTAGE, store_BUCK_VPCA7_VOLTAGE);

static ssize_t show_BUCK_VSRMCA7_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    //kal_uint32 ret=0;
    //kal_uint32 reg_address=0x244;
    kal_uint32 reg_val=0;
    
    //ret = pmic_read_interface(reg_address, &reg_val, 0x7F, 0);
    reg_val = upmu_get_ni_vsrmca7_vosel();
    ret_value = 70000 + (reg_val*625);
    ret_value = ret_value / 100;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] BUCK_VSRMCA7_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VSRMCA7_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(BUCK_VSRMCA7_VOLTAGE, 0664, show_BUCK_VSRMCA7_VOLTAGE, store_BUCK_VSRMCA7_VOLTAGE);

static ssize_t show_BUCK_VCA15_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    //kal_uint32 ret=0;
    //kal_uint32 reg_address=0x21E;
    kal_uint32 reg_val=0;
    
    //ret = pmic_read_interface(reg_address, &reg_val, 0x7F, 0);
    reg_val = upmu_get_ni_vca15_vosel();
    ret_value = 70000 + (reg_val*625);
    ret_value = ret_value / 100;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] BUCK_VCA15_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VCA15_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(BUCK_VCA15_VOLTAGE, 0664, show_BUCK_VCA15_VOLTAGE, store_BUCK_VCA15_VOLTAGE);

static ssize_t show_BUCK_VSRMCA15_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    //kal_uint32 ret=0;
    //kal_uint32 reg_address=0x244;
    kal_uint32 reg_val=0;
    
    //ret = pmic_read_interface(reg_address, &reg_val, 0x7F, 0);
    reg_val = upmu_get_ni_vsrmca15_vosel();
    ret_value = 70000 + (reg_val*625);
    ret_value = ret_value / 100;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] BUCK_VSRMCA15_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VSRMCA15_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(BUCK_VSRMCA15_VOLTAGE, 0664, show_BUCK_VSRMCA15_VOLTAGE, store_BUCK_VSRMCA15_VOLTAGE);

static ssize_t show_BUCK_VCORE_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    //kal_uint32 ret=0;
    //kal_uint32 reg_address=0x270;
    kal_uint32 reg_val=0;
    
    //ret = pmic_read_interface(reg_address, &reg_val, 0x7F, 0);
    reg_val = upmu_get_ni_vcore_vosel();
    ret_value = 70000 + (reg_val*625);
    ret_value = ret_value / 100;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] BUCK_VCORE_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VCORE_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(BUCK_VCORE_VOLTAGE, 0664, show_BUCK_VCORE_VOLTAGE, store_BUCK_VCORE_VOLTAGE);

static ssize_t show_BUCK_VDRM_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    //kal_uint32 ret=0;
    //kal_uint32 reg_address=0x296;
    kal_uint32 reg_val=0;
    
    //ret = pmic_read_interface(reg_address, &reg_val, 0x7F, 0);
    reg_val = upmu_get_ni_vdrm_vosel();
    ret_value = 80000 + (reg_val*625);
    ret_value = ret_value / 100;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] BUCK_VDRM_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VDRM_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(BUCK_VDRM_VOLTAGE, 0664, show_BUCK_VDRM_VOLTAGE, store_BUCK_VDRM_VOLTAGE);

static ssize_t show_BUCK_VIO18_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    //kal_uint32 ret=0;
    //kal_uint32 reg_address=0x318;
    kal_uint32 reg_val=0;
    
    //ret = pmic_read_interface(reg_address, &reg_val, 0x1F, 0);
    reg_val = upmu_get_ni_vio18_vosel();
    ret_value = 1500 + (reg_val*20);    
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] BUCK_VIO18_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VIO18_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(BUCK_VIO18_VOLTAGE, 0664, show_BUCK_VIO18_VOLTAGE, store_BUCK_VIO18_VOLTAGE);

static ssize_t show_BUCK_VGPU_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    //kal_uint32 ret=0;
    //kal_uint32 reg_address=0x392;
    kal_uint32 reg_val=0;
    
    //ret = pmic_read_interface(reg_address, &reg_val, 0x1F, 0);
    reg_val = upmu_get_ni_vgpu_vosel();
    ret_value = 70000 + (reg_val*625);
    ret_value = ret_value / 100;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] BUCK_VGPU_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_BUCK_VGPU_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(BUCK_VGPU_VOLTAGE, 0664, show_BUCK_VGPU_VOLTAGE, store_BUCK_VGPU_VOLTAGE);

static ssize_t show_LDO_VIO28_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
            
    ret_value = 2800;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VIO28_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VIO28_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VIO28_VOLTAGE, 0664, show_LDO_VIO28_VOLTAGE, store_LDO_VIO28_VOLTAGE);

static ssize_t show_LDO_VUSB_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
   
    ret_value = 3300;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VUSB_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VUSB_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VUSB_VOLTAGE, 0664, show_LDO_VUSB_VOLTAGE, store_LDO_VUSB_VOLTAGE);

static ssize_t show_LDO_VMC_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x44A;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 4);
    if(reg_val == 0)
        ret_value = 1800;
    else if(reg_val == 1)
        ret_value = 3300;        
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VMC_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VMC_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VMC_VOLTAGE, 0664, show_LDO_VMC_VOLTAGE, store_LDO_VMC_VOLTAGE);

static ssize_t show_LDO_VMCH_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x432;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 7);
    if(reg_val == 0)
        ret_value = 3000;
    else if(reg_val == 1)
        ret_value = 3300;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VMCH_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VMCH_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VMCH_VOLTAGE, 0664, show_LDO_VMCH_VOLTAGE, store_LDO_VMCH_VOLTAGE);

static ssize_t show_LDO_VEMC_3V3_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x434;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x01, 4);
    if(reg_val == 0)
        ret_value = 3000;
    else if(reg_val == 1)
        ret_value = 3300;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VEMC_3V3_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VEMC_3V3_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VEMC_3V3_VOLTAGE, 0664, show_LDO_VEMC_3V3_VOLTAGE, store_LDO_VEMC_3V3_VOLTAGE);

static ssize_t show_LDO_VCAMD_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x436;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x07, 5);
    if(reg_val == 0)
        ret_value = 1200;
    else if(reg_val == 1)
        ret_value = 1300;
    else if(reg_val == 2)
        ret_value = 1500;
    else if(reg_val == 3)
        ret_value = 1800;        
    else if(reg_val == 4)
        ret_value = 2500;
    else if(reg_val == 5)
        ret_value = 2800;
    else if(reg_val == 6)
        ret_value = 3000;
    else if(reg_val == 7)
        ret_value = 3300; 
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VCAMD_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VCAMD_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VCAMD_VOLTAGE, 0664, show_LDO_VCAMD_VOLTAGE, store_LDO_VCAMD_VOLTAGE);

static ssize_t show_LDO_VCAMIO_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x438;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x07, 5);
    if(reg_val == 0)
        ret_value = 1200;
    else if(reg_val == 1)
        ret_value = 1300;
    else if(reg_val == 2)
        ret_value = 1500;
    else if(reg_val == 3)
        ret_value = 1800;        
    else if(reg_val == 4)
        ret_value = 2500;
    else if(reg_val == 5)
        ret_value = 2800;
    else if(reg_val == 6)
        ret_value = 3000;
    else if(reg_val == 7)
        ret_value = 3300; 
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VCAMIO_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VCAMIO_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VCAMIO_VOLTAGE, 0664, show_LDO_VCAMIO_VOLTAGE, store_LDO_VCAMIO_VOLTAGE);

static ssize_t show_LDO_VCAMAF_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x43A;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x07, 5);
    if(reg_val == 0)
        ret_value = 1200;
    else if(reg_val == 1)
        ret_value = 1300;
    else if(reg_val == 2)
        ret_value = 1500;
    else if(reg_val == 3)
        ret_value = 1800;        
    else if(reg_val == 4)
        ret_value = 2500;
    else if(reg_val == 5)
        ret_value = 2800;
    else if(reg_val == 6)
        ret_value = 3000;
    else if(reg_val == 7)
        ret_value = 3300;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VCAMAF_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VCAMAF_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VCAMAF_VOLTAGE, 0664, show_LDO_VCAMAF_VOLTAGE, store_LDO_VCAMAF_VOLTAGE);

static ssize_t show_LDO_VGP4_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x43C;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x07, 5);
    if(reg_val == 0)
        ret_value = 1200;
    else if(reg_val == 1)
        ret_value = 1300;
    else if(reg_val == 2)
        ret_value = 1500;
    else if(reg_val == 3)
        ret_value = 1800;        
    else if(reg_val == 4)
        ret_value = 2500;
    else if(reg_val == 5)
        ret_value = 2800;
    else if(reg_val == 6)
        ret_value = 3000;
    else if(reg_val == 7)
        ret_value = 3300;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VGP4_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VGP4_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VGP4_VOLTAGE, 0664, show_LDO_VGP4_VOLTAGE, store_LDO_VGP4_VOLTAGE);

static ssize_t show_LDO_VGP5_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x43E;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x07, 5);
    if(reg_val == 0)
        ret_value = 1200;
    else if(reg_val == 1)
        ret_value = 1300;
    else if(reg_val == 2)
        ret_value = 1500;
    else if(reg_val == 3)
        ret_value = 1800;        
    else if(reg_val == 4)
        ret_value = 2500;
    else if(reg_val == 5)
        ret_value = 2800;
    else if(reg_val == 6)
        ret_value = 3000;
    else if(reg_val == 7)
        ret_value = 2000;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VGP5_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VGP5_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VGP5_VOLTAGE, 0664, show_LDO_VGP5_VOLTAGE, store_LDO_VGP5_VOLTAGE);

static ssize_t show_LDO_VGP6_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x45A;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x07, 5);
    if(reg_val == 0)
        ret_value = 1200;
    else if(reg_val == 1)
        ret_value = 1300;
    else if(reg_val == 2)
        ret_value = 1500;
    else if(reg_val == 3)
        ret_value = 1800;        
    else if(reg_val == 4)
        ret_value = 2500;
    else if(reg_val == 5)
        ret_value = 2800;
    else if(reg_val == 6)
        ret_value = 3000;
    else if(reg_val == 7)
        ret_value = 3300;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VGP6_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VGP6_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VGP6_VOLTAGE, 0664, show_LDO_VGP6_VOLTAGE, store_LDO_VGP6_VOLTAGE);

static ssize_t show_LDO_VIBR_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x442;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x07, 9);
    if(reg_val == 0)
        ret_value = 1300;
    else if(reg_val == 1)
        ret_value = 1500;
    else if(reg_val == 2)
        ret_value = 1800;        
    else if(reg_val == 3)
        ret_value = 2000;        
    else if(reg_val == 4)
        ret_value = 2500;
    else if(reg_val == 5)
        ret_value = 2800;
    else if(reg_val == 6)
        ret_value = 3000;
    else if(reg_val == 7)
        ret_value = 3300;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VIBR_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VIBR_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VIBR_VOLTAGE, 0664, show_LDO_VIBR_VOLTAGE, store_LDO_VIBR_VOLTAGE);

static ssize_t show_LDO_VRTC_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    ret_value = 2800;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VRTC_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VRTC_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VRTC_VOLTAGE, 0664, show_LDO_VRTC_VOLTAGE, store_LDO_VRTC_VOLTAGE);

static ssize_t show_LDO_VTCXO_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    ret_value = 2800;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VTCXO_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VTCXO_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VTCXO_VOLTAGE, 0664, show_LDO_VTCXO_VOLTAGE, store_LDO_VTCXO_VOLTAGE);

static ssize_t show_LDO_VA28_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    ret_value = 2800;
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VA28_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VA28_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VA28_VOLTAGE, 0664, show_LDO_VA28_VOLTAGE, store_LDO_VA28_VOLTAGE);

static ssize_t show_LDO_VCAMA_VOLTAGE(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_uint32 ret_value=0;
    
    kal_uint32 ret=0;
    kal_uint32 reg_address=0x40C;
    kal_uint32 reg_val=0;
    
    ret = pmic_read_interface(reg_address, &reg_val, 0x03, 6);
    if(reg_val == 0)
        ret_value = 1500;
    else if(reg_val == 1)
        ret_value = 1800;
    else if(reg_val == 2)
        ret_value = 2500;
    else if(reg_val == 3)
        ret_value = 2800;        
    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] LDO_VCAMA_VOLTAGE : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_LDO_VCAMA_VOLTAGE(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/PMIC", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(LDO_VCAMA_VOLTAGE, 0664, show_LDO_VCAMA_VOLTAGE, store_LDO_VCAMA_VOLTAGE);


//==============================================================================
// PMIC6397 device driver
//==============================================================================
void ldo_service_test(void)
{
    hwPowerOn(MT65XX_POWER_LDO_VIO28,    VOL_DEFAULT, "ldo_test");
    hwPowerOn(MT65XX_POWER_LDO_VUSB,     VOL_DEFAULT, "ldo_test");
    hwPowerOn(MT65XX_POWER_LDO_VMC,      VOL_DEFAULT, "ldo_test");
    hwPowerOn(MT65XX_POWER_LDO_VMCH,     VOL_DEFAULT, "ldo_test");
    hwPowerOn(MT65XX_POWER_LDO_VEMC_3V3, VOL_DEFAULT, "ldo_test");
    hwPowerOn(MT65XX_POWER_LDO_VCAMD,    VOL_DEFAULT, "ldo_test");
    hwPowerOn(MT65XX_POWER_LDO_VCAMIO,   VOL_DEFAULT, "ldo_test");
    hwPowerOn(MT65XX_POWER_LDO_VCAMAF,   VOL_DEFAULT, "ldo_test");
    hwPowerOn(MT65XX_POWER_LDO_VGP4,     VOL_DEFAULT, "ldo_test");
    hwPowerOn(MT65XX_POWER_LDO_VGP5,     VOL_DEFAULT, "ldo_test");
    hwPowerOn(MT65XX_POWER_LDO_VGP6,     VOL_DEFAULT, "ldo_test");    
    hwPowerOn(MT65XX_POWER_LDO_VIBR,     VOL_DEFAULT, "ldo_test");
    hwPowerOn(MT65XX_POWER_LDO_VRTC,     VOL_DEFAULT, "ldo_test");    
    hwPowerOn(MT65XX_POWER_LDO_VTCXO,    VOL_DEFAULT, "ldo_test");
    hwPowerOn(MT65XX_POWER_LDO_VA28,     VOL_DEFAULT, "ldo_test");
    hwPowerOn(MT65XX_POWER_LDO_VCAMA,    VOL_DEFAULT, "ldo_test");

    hwPowerDown(MT65XX_POWER_LDO_VIO28,     "ldo_test");
    hwPowerDown(MT65XX_POWER_LDO_VUSB,      "ldo_test");
    hwPowerDown(MT65XX_POWER_LDO_VMC,       "ldo_test");
    hwPowerDown(MT65XX_POWER_LDO_VMCH,      "ldo_test");
    hwPowerDown(MT65XX_POWER_LDO_VEMC_3V3,  "ldo_test");
    hwPowerDown(MT65XX_POWER_LDO_VCAMD,     "ldo_test");
    hwPowerDown(MT65XX_POWER_LDO_VCAMIO,    "ldo_test");
    hwPowerDown(MT65XX_POWER_LDO_VCAMAF,    "ldo_test");
    hwPowerDown(MT65XX_POWER_LDO_VGP4,      "ldo_test");
    hwPowerDown(MT65XX_POWER_LDO_VGP5,      "ldo_test");
    hwPowerDown(MT65XX_POWER_LDO_VGP6,      "ldo_test");    
    hwPowerDown(MT65XX_POWER_LDO_VIBR,      "ldo_test");
    hwPowerDown(MT65XX_POWER_LDO_VRTC,      "ldo_test");
    hwPowerDown(MT65XX_POWER_LDO_VTCXO,     "ldo_test");
    hwPowerDown(MT65XX_POWER_LDO_VA28,      "ldo_test");
    hwPowerDown(MT65XX_POWER_LDO_VCAMA,     "ldo_test");
}

void PMIC_INIT_SETTING_V1(void)
{
    U32 chip_version = 0;
    U32 ret = 0;

    chip_version = upmu_get_cid();

    if(chip_version >= PMIC6397_E1_CID_CODE)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[Kernel_PMIC_INIT_SETTING_V1] PMIC Chip = %x\n",chip_version);
        
        //put init setting from DE/SA        
        ret = pmic_config_interface(0x2,0xB,0xF,4); // [7:4]: RG_VCDT_HV_VTH; 7V OVP
        ret = pmic_config_interface(0xC,0x1,0x7,1); // [3:1]: RG_VBAT_OV_VTH; VBAT_OV=4.3V
        ret = pmic_config_interface(0x1A,0x3,0xF,0); // [3:0]: RG_CHRWDT_TD; align to 6250's
        ret = pmic_config_interface(0x24,0x1,0x1,1); // [1:1]: RG_BC11_RST; 
        ret = pmic_config_interface(0x2A,0x0,0x7,4); // [6:4]: RG_CSDAC_STP; align to 6250's setting
        ret = pmic_config_interface(0x2E,0x1,0x1,7); // [7:7]: RG_ULC_DET_EN; 
        ret = pmic_config_interface(0x2E,0x1,0x1,6); // [6:6]: RG_HWCV_EN; 
        ret = pmic_config_interface(0x2E,0x1,0x1,2); // [2:2]: RG_CSDAC_MODE; 
        ret = pmic_config_interface(0x102,0x0,0x1,3); // [3:3]: RG_PWMOC_CK_PDN; For OC protection
        ret = pmic_config_interface(0x128,0x1,0x1,9); // [9:9]: RG_SRCVOLT_HW_AUTO_EN; 
        ret = pmic_config_interface(0x128,0x1,0x1,8); // [8:8]: RG_OSC_SEL_AUTO; 
        ret = pmic_config_interface(0x128,0x1,0x1,6); // [6:6]: RG_SMPS_DIV2_SRC_AUTOFF_DIS; 
        ret = pmic_config_interface(0x128,0x1,0x1,5); // [5:5]: RG_SMPS_AUTOFF_DIS; 
        ret = pmic_config_interface(0x130,0x1,0x1,7); // [7:7]: VDRM_DEG_EN; 
        ret = pmic_config_interface(0x130,0x1,0x1,6); // [6:6]: VSRMCA7_DEG_EN; 
        ret = pmic_config_interface(0x130,0x1,0x1,5); // [5:5]: VPCA7_DEG_EN; 
        ret = pmic_config_interface(0x130,0x1,0x1,4); // [4:4]: VIO18_DEG_EN; 
        ret = pmic_config_interface(0x130,0x1,0x1,3); // [3:3]: VGPU_DEG_EN; For OC protection
        ret = pmic_config_interface(0x130,0x1,0x1,2); // [2:2]: VCORE_DEG_EN; 
        ret = pmic_config_interface(0x130,0x1,0x1,1); // [1:1]: VSRMCA15_DEG_EN; 
        ret = pmic_config_interface(0x130,0x1,0x1,0); // [0:0]: VCA15_DEG_EN; 
        ret = pmic_config_interface(0x178,0x1,0x1,11); // [11:11]: RG_INT_EN_THR_H; 
        ret = pmic_config_interface(0x178,0x1,0x1,10); // [10:10]: RG_INT_EN_THR_L; 
        ret = pmic_config_interface(0x178,0x1,0x1,4); // [4:4]: RG_INT_EN_BAT_L; 
        ret = pmic_config_interface(0x17E,0x1,0x1,11); // [11:11]: RG_INT_EN_VGPU; OC protection
        ret = pmic_config_interface(0x17E,0x1,0x1,8); // [8:8]: RG_INT_EN_VCA15; OC protection
        ret = pmic_config_interface(0x206,0x600,0x0FFF,0); // [12:0]: BUCK_RSV; for OC protection
        ret = pmic_config_interface(0x210,0x0,0x3,10); // [11:10]: QI_VCORE_VSLEEP; sleep mode only (0.85V)
        ret = pmic_config_interface(0x210,0x0,0x3,6); // [7:6]: QI_VSRMCA7_VSLEEP; sleep mode only (0.85V)
        ret = pmic_config_interface(0x210,0x0,0x3,2); // [3:2]: QI_VPCA7_VSLEEP; sleep mode only (0.85V)
        ret = pmic_config_interface(0x216,0x0,0x3,12); // [13:12]: RG_VCA15_CSL2; for OC protection
        ret = pmic_config_interface(0x216,0x0,0x3,10); // [11:10]: RG_VCA15_CSL1; for OC protection
        ret = pmic_config_interface(0x224,0x1,0x1,15); // [15:15]: VCA15_SFCHG_REN; soft change rising enable
        ret = pmic_config_interface(0x224,0x5,0x7F,8); // [14:8]: VCA15_SFCHG_RRATE; soft change rising step=0.5us
        ret = pmic_config_interface(0x224,0x1,0x1,7); // [7:7]: VCA15_SFCHG_FEN; soft change falling enable
        ret = pmic_config_interface(0x224,0x17,0x7F,0); // [6:0]: VCA15_SFCHG_FRATE; soft change falling step=2us
        ret = pmic_config_interface(0x238,0x1,0x1,8); // [8:8]: VCA15_VSLEEP_EN; set sleep mode reference voltage from R2R to V2V
        ret = pmic_config_interface(0x238,0x3,0x3,4); // [5:4]: VCA15_VOSEL_TRANS_EN; rising & falling enable
        ret = pmic_config_interface(0x24A,0x1,0x1,15); // [15:15]: VSRMCA15_SFCHG_REN; 
        ret = pmic_config_interface(0x24A,0x5,0x7F,8); // [14:8]: VSRMCA15_SFCHG_RRATE; 
        ret = pmic_config_interface(0x24A,0x1,0x1,7); // [7:7]: VSRMCA15_SFCHG_FEN; 
        ret = pmic_config_interface(0x24A,0x17,0x7F,0); // [6:0]: VSRMCA15_SFCHG_FRATE; 
        ret = pmic_config_interface(0x25E,0x1,0x1,8); // [8:8]: VSRMCA15_VSLEEP_EN; set sleep mode reference voltage from R2R to V2V
        ret = pmic_config_interface(0x25E,0x3,0x3,4); // [5:4]: VSRMCA15_VOSEL_TRANS_EN; rising & falling enable
        ret = pmic_config_interface(0x270,0x1,0x1,1); // [1:1]: VCORE_VOSEL_CTRL; sleep mode voltage control follow SRCLKEN
        ret = pmic_config_interface(0x276,0x1,0x1,15); // [15:15]: VCORE_SFCHG_REN; 
        ret = pmic_config_interface(0x276,0x5,0x7F,8); // [14:8]: VCORE_SFCHG_RRATE; 
        ret = pmic_config_interface(0x276,0x17,0x7F,0); // [6:0]: VCORE_SFCHG_FRATE; 
        ret = pmic_config_interface(0x27C,0x18,0x7F,0); // [6:0]: VCORE_VOSEL_SLEEP; Sleep mode setting only (0.85V)
        ret = pmic_config_interface(0x28A,0x1,0x1,8); // [8:8]: VCORE_VSLEEP_EN; Sleep mode HW control  R2R to VtoV
        ret = pmic_config_interface(0x28A,0x0,0x3,4); // [5:4]: VCORE_VOSEL_TRANS_EN; Follows MT6320 VCORE setting.
        ret = pmic_config_interface(0x28A,0x3,0x3,0); // [1:0]: VCORE_TRANSTD; 
        ret = pmic_config_interface(0x28E,0x1,0x3,8); // [9:8]: RG_VGPU_CSL; for OC protection
        ret = pmic_config_interface(0x29C,0x1,0x1,15); // [15:15]: VGPU_SFCHG_REN; 
        ret = pmic_config_interface(0x29C,0x5,0x7F,8); // [14:8]: VGPU_SFCHG_RRATE; 
        ret = pmic_config_interface(0x29C,0x17,0x7F,0); // [6:0]: VGPU_SFCHG_FRATE; 
        ret = pmic_config_interface(0x2B0,0x0,0x3,4); // [5:4]: VGPU_VOSEL_TRANS_EN; 
        ret = pmic_config_interface(0x2B0,0x3,0x3,0); // [1:0]: VGPU_TRANSTD; 
        ret = pmic_config_interface(0x332,0x0,0x3,4); // [5:4]: VPCA7_VOSEL_SEL; 
        ret = pmic_config_interface(0x336,0x1,0x1,15); // [15:15]: VPCA7_SFCHG_REN; 
        ret = pmic_config_interface(0x336,0x5,0x7F,8); // [14:8]: VPCA7_SFCHG_RRATE; 
        ret = pmic_config_interface(0x336,0x1,0x1,7); // [7:7]: VPCA7_SFCHG_FEN; 
        ret = pmic_config_interface(0x336,0x17,0x7F,0); // [6:0]: VPCA7_SFCHG_FRATE; 
        ret = pmic_config_interface(0x33C,0x18,0x7F,0); // [6:0]: VPCA7_VOSEL_SLEEP; 
        ret = pmic_config_interface(0x34A,0x1,0x1,8); // [8:8]: VPCA7_VSLEEP_EN; 
        ret = pmic_config_interface(0x34A,0x3,0x3,4); // [5:4]: VPCA7_VOSEL_TRANS_EN; 
        ret = pmic_config_interface(0x356,0x1,0x1,5); // [5:5]: VSRMCA7_TRACK_SLEEP_CTRL; 
        ret = pmic_config_interface(0x358,0x0,0x3,4); // [5:4]: VSRMCA7_VOSEL_SEL; 
        ret = pmic_config_interface(0x35C,0x1,0x1,15); // [15:15]: VSRMCA7_SFCHG_REN; 
        ret = pmic_config_interface(0x35C,0x5,0x7F,8); // [14:8]: VSRMCA7_SFCHG_RRATE; 
        ret = pmic_config_interface(0x35C,0x1,0x1,7); // [7:7]: VSRMCA7_SFCHG_FEN; 
        ret = pmic_config_interface(0x35C,0x17,0x7F,0); // [6:0]: VSRMCA7_SFCHG_FRATE; 
        ret = pmic_config_interface(0x362,0x18,0x7F,0); // [6:0]: VSRMCA7_VOSEL_SLEEP; 
        ret = pmic_config_interface(0x370,0x1,0x1,8); // [8:8]: VSRMCA7_VSLEEP_EN; 
        ret = pmic_config_interface(0x370,0x3,0x3,4); // [5:4]: VSRMCA7_VOSEL_TRANS_EN; 
        ret = pmic_config_interface(0x39C,0x1,0x1,8); // [8:8]: VDRM_VSLEEP_EN; 
        ret = pmic_config_interface(0x440,0x1,0x1,2); // [2:2]: VIBR_THER_SHEN_EN; 
        ret = pmic_config_interface(0x500,0x1,0x1,5); // [5:5]: THR_HWPDN_EN; 
        ret = pmic_config_interface(0x502,0x1,0x1,3); // [3:3]: RG_RST_DRVSEL; 
        ret = pmic_config_interface(0x502,0x1,0x1,2); // [2:2]: RG_EN_DRVSEL; 
        ret = pmic_config_interface(0x508,0x1,0x1,1); // [1:1]: PWRBB_DEB_EN; 
        ret = pmic_config_interface(0x50C,0x1,0x1,12); // [12:12]: VSRMCA15_PG_H2L_EN; 
        ret = pmic_config_interface(0x50C,0x1,0x1,11); // [11:11]: VPCA15_PG_H2L_EN; 
        ret = pmic_config_interface(0x50C,0x1,0x1,10); // [10:10]: VCORE_PG_H2L_EN; 
        ret = pmic_config_interface(0x50C,0x1,0x1,9); // [9:9]: VSRMCA7_PG_H2L_EN; 
        ret = pmic_config_interface(0x50C,0x1,0x1,8); // [8:8]: VPCA7_PG_H2L_EN; 
        ret = pmic_config_interface(0x512,0x1,0x1,1); // [1:1]: STRUP_PWROFF_PREOFF_EN; 
        ret = pmic_config_interface(0x512,0x1,0x1,0); // [0:0]: STRUP_PWROFF_SEQ_EN; 
        ret = pmic_config_interface(0x55E,0xFC,0xFF,8); // [15:8]: RG_ADC_TRIM_CH_SEL; 
        ret = pmic_config_interface(0x560,0x1,0x1,1); // [1:1]: FLASH_THER_SHDN_EN; 
        ret = pmic_config_interface(0x566,0x1,0x1,1); // [1:1]: KPLED_THER_SHDN_EN; 
        ret = pmic_config_interface(0x600,0x1,0x1,9); // [9:9]: SPK_THER_SHDN_L_EN; 
        ret = pmic_config_interface(0x604,0x1,0x1,0); // [0:0]: RG_SPK_INTG_RST_L; 
        ret = pmic_config_interface(0x606,0x1,0x1,9); // [9:9]: SPK_THER_SHDN_R_EN; 
        ret = pmic_config_interface(0x60A,0x1,0xF,11); // [14:11]: RG_SPKPGA_GAINR; 
        ret = pmic_config_interface(0x612,0x1,0xF,8); // [11:8]: RG_SPKPGA_GAINL; 
        ret = pmic_config_interface(0x632,0x1,0x1,8); // [8:8]: FG_SLP_EN; 
        ret = pmic_config_interface(0x638,0xFFC2,0xFFFF,0); // [15:0]: FG_SLP_CUR_TH; 
        ret = pmic_config_interface(0x63A,0x14,0xFF,0); // [7:0]: FG_SLP_TIME; 
        ret = pmic_config_interface(0x63C,0xFF,0xFF,8); // [15:8]: FG_DET_TIME; 
        ret = pmic_config_interface(0x714,0x1,0x1,7); // [7:7]: RG_LCLDO_ENC_REMOTE_SENSE_VA28; 
        ret = pmic_config_interface(0x714,0x1,0x1,4); // [4:4]: RG_LCLDO_REMOTE_SENSE_VA33; 
        ret = pmic_config_interface(0x714,0x1,0x1,1); // [1:1]: RG_HCLDO_REMOTE_SENSE_VA33; 
        ret = pmic_config_interface(0x71A,0x1,0x1,15); // [15:15]: RG_NCP_REMOTE_SENSE_VA18; 
        ret = pmic_config_interface(0x260,0x4,0x7F,8); // [14:8]: VSRMCA15_VOSEL_OFFSET; set offset=25mV
        ret = pmic_config_interface(0x260,0x0,0x7F,0); // [6:0]: VSRMCA15_VOSEL_DELTA; set delta=0mV
        ret = pmic_config_interface(0x262,0x5C,0x7F,8); // [14:8]: VSRMCA15_VOSEL_ON_HB; set HB=1.275V
        ret = pmic_config_interface(0x262,0x38,0x7F,0); // [6:0]: VSRMCA15_VOSEL_ON_LB; set LB=1.05000V
        ret = pmic_config_interface(0x264,0x18,0x7F,0); // [6:0]: VSRMCA15_VOSEL_SLEEP_LB; set sleep LB=0.85000V
        ret = pmic_config_interface(0x372,0x4,0x7F,8); // [14:8]: VSRMCA7_VOSEL_OFFSET; set offset=25mV
        ret = pmic_config_interface(0x372,0x0,0x7F,0); // [6:0]: VSRMCA7_VOSEL_DELTA; set delta=0mV
        ret = pmic_config_interface(0x374,0x5C,0x7F,8); // [14:8]: VSRMCA7_VOSEL_ON_HB; set HB=1.275V
        ret = pmic_config_interface(0x374,0x38,0x7F,0); // [6:0]: VSRMCA7_VOSEL_ON_LB; set LB=1.05000V
        ret = pmic_config_interface(0x376,0x18,0x7F,0); // [6:0]: VSRMCA7_VOSEL_SLEEP_LB; set sleep LB=0.85000V
        ret = pmic_config_interface(0x21E,0x1,0x1,1); // [1:1]: VCA15_VOSEL_CTRL; DVS HW control by SRCLKEN
        ret = pmic_config_interface(0x244,0x1,0x1,1); // [1:1]: VSRMCA15_VOSEL_CTRL; 
        ret = pmic_config_interface(0x330,0x1,0x1,1); // [1:1]: VPCA7_VOSEL_CTRL;
        ret = pmic_config_interface(0x356,0x1,0x1,1); // [1:1]: VSRMCA7_VOSEL_CTRL;
        ret = pmic_config_interface(0x21E,0x0,0x1,4); // [4:4]: VCA15_TRACK_ON_CTRL; DVFS tracking enable
        ret = pmic_config_interface(0x244,0x0,0x1,4); // [4:4]: VSRMCA15_TRACK_ON_CTRL; 
        ret = pmic_config_interface(0x330,0x0,0x1,4); // [4:4]: VPCA7_TRACK_ON_CTRL; 
        ret = pmic_config_interface(0x356,0x0,0x1,4); // [4:4]: VSRMCA7_TRACK_ON_CTRL;
        ret = pmic_config_interface(0x134,0x3,0x3,14); // [15:14]: VGPU OC; 
        ret = pmic_config_interface(0x134,0x3,0x3,2); // [3:2]: VCA15 OC;        
        #if 0
        //dump register
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x002, upmu_get_reg_value(0x002));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x00C, upmu_get_reg_value(0x00C));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x01A, upmu_get_reg_value(0x01A));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x024, upmu_get_reg_value(0x024));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x02A, upmu_get_reg_value(0x02A));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x02E, upmu_get_reg_value(0x02E));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x102, upmu_get_reg_value(0x102));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x128, upmu_get_reg_value(0x128));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x130, upmu_get_reg_value(0x130));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x134, upmu_get_reg_value(0x134)); 
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x178, upmu_get_reg_value(0x178));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x17E, upmu_get_reg_value(0x17E));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x206, upmu_get_reg_value(0x206));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x210, upmu_get_reg_value(0x210));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x216, upmu_get_reg_value(0x216));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x21E, upmu_get_reg_value(0x21E));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x224, upmu_get_reg_value(0x224));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x238, upmu_get_reg_value(0x238));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x244, upmu_get_reg_value(0x244));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x24A, upmu_get_reg_value(0x24A));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x25E, upmu_get_reg_value(0x25E));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x260, upmu_get_reg_value(0x260));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x262, upmu_get_reg_value(0x262));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x264, upmu_get_reg_value(0x264));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x270, upmu_get_reg_value(0x270));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x276, upmu_get_reg_value(0x276));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x27C, upmu_get_reg_value(0x27C));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x28A, upmu_get_reg_value(0x28A));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x28E, upmu_get_reg_value(0x28E));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x29C, upmu_get_reg_value(0x29C));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x2B0, upmu_get_reg_value(0x2B0));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x330, upmu_get_reg_value(0x330));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x332, upmu_get_reg_value(0x332));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x336, upmu_get_reg_value(0x336));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x33C, upmu_get_reg_value(0x33C));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x34A, upmu_get_reg_value(0x34A));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x356, upmu_get_reg_value(0x356));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x358, upmu_get_reg_value(0x358));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x35C, upmu_get_reg_value(0x35C));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x362, upmu_get_reg_value(0x362));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x370, upmu_get_reg_value(0x370));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x372, upmu_get_reg_value(0x372));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x374, upmu_get_reg_value(0x374));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x376, upmu_get_reg_value(0x376));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x39C, upmu_get_reg_value(0x39C));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x440, upmu_get_reg_value(0x440));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x500, upmu_get_reg_value(0x500));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x502, upmu_get_reg_value(0x502));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x508, upmu_get_reg_value(0x508));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x50C, upmu_get_reg_value(0x50C));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x512, upmu_get_reg_value(0x512));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x55E, upmu_get_reg_value(0x55E));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x560, upmu_get_reg_value(0x560));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x566, upmu_get_reg_value(0x566));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x600, upmu_get_reg_value(0x600));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x604, upmu_get_reg_value(0x604));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x606, upmu_get_reg_value(0x606));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x60A, upmu_get_reg_value(0x60A));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x612, upmu_get_reg_value(0x612));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x632, upmu_get_reg_value(0x632));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x638, upmu_get_reg_value(0x638));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x63A, upmu_get_reg_value(0x63A));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x63C, upmu_get_reg_value(0x63C));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x714, upmu_get_reg_value(0x714));
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_init_setting] Reg[0x%x]=0x%x\n", 0x71A, upmu_get_reg_value(0x71A));        
        #endif
    }
    else
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[Kernel_PMIC_INIT_SETTING_V1] Unknown PMIC Chip (%x)\n",chip_version);
    }
}

void PMIC_CUSTOM_SETTING_V1(void)
{
#if 0    
    U32 ret = 0, reg_val = 0;
#endif    

#if 0  //disable DCXO HW contol for bringup by SPM (YT Lee) request.
    upmu_set_rg_srcvolt_hw_auto_en(1);
    
    //enable BB 26MHz clock
    upmu_set_rg_dcxo_ldo_dbb_reg_en(0x1);
    upmu_set_rg_dcxo_s2a_ldo_bb_en(0x1);

    upmu_set_rg_srclkperi_hw_auto_en(1);

    //enable RF1 26MHz clock
    upmu_set_rg_dcxo_ldo_rf1_reg_en(0x1);
    upmu_set_rg_dcxo_s2a_ldo_rf1_en(0x1); /* clock on for internal 32K */
    upmu_set_rg_dcxo_por2_ldo_rf1_en(0x1); /* clock on for external 32K */

    //enable RF2 26MHz clock
    upmu_set_rg_dcxo_ldo_rf2_reg_en(0x1);
    upmu_set_rg_dcxo_s2a_ldo_rf2_en(0x1); /* clock on for internal 32K */
    upmu_set_rg_dcxo_por2_ldo_rf2_en(0x1); /* clock on for external 32K */
#else    
    //enable HW control DCXO 26MHz on-off, request by SPM module
    upmu_set_rg_srcvolt_hw_auto_en(1);
    upmu_set_rg_dcxo_ldo_dbb_reg_en(0);

    //enable HW control DCXO RF clk on-off, request by low power module task
    upmu_set_rg_srclkperi_hw_auto_en(1);
    upmu_set_rg_dcxo_ldo_rf1_reg_en(0);
    
#ifndef MTK_PMIC_RF2_26M_ALWAYS_ON
    upmu_set_rg_dcxo_ldo_rf2_reg_en(0);
#else
    //enable RF2 26MHz clock
    upmu_set_rg_dcxo_ldo_rf2_reg_en(0x1);
    upmu_set_rg_dcxo_s2a_ldo_rf2_en(0x1); /* clock on for internal 32K */
    upmu_set_rg_dcxo_por2_ldo_rf2_en(0x1); /* clock on for external 32K */
#endif
#endif

    //config vsrmca7, vsrmca15 voltage by DVFS PIC(Louis Yu) request
    //pmic_config_interface(VSRMCA7_CON9, 0x5D, PMIC_VSRMCA7_VOSEL_MASK, PMIC_VSRMCA7_VOSEL_SHIFT); //VSRMCA7 1.28125V  1011101
    upmu_set_vsrmca7_vosel(0x5D); //1.28125V
    upmu_set_vsrmca7_vosel_on(0x5D); //1.28125V
    //pmic_config_interface(VSRMCA15_CON9, 0x5D, PMIC_VSRMCA15_VOSEL_MASK, PMIC_VSRMCA15_VOSEL_SHIFT); //VSRMCA15 1.28125V
    upmu_set_vsrmca15_vosel(0x5D); //1.28125V
    upmu_set_vsrmca15_vosel_on(0x5D); //1.28125V
    
#if 0    
    //config vcore to HW control mode by deep idle PIC(YT Lee) request
    upmu_set_vcore_vosel_ctrl(1);
    
    //enable VGP6 by default
    upmu_set_rg_vgp6_sw_en(1);
    upmu_set_rg_vgp6_vosel(0x7);
    ret = pmic_read_interface(DIGLDO_CON33, &reg_val, 0xFFFF, 0);
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[%s] Reg[0x%x] = 0x%x\n", __FUNCTION__, DIGLDO_CON33, reg_val);
#endif
}

void pmic_low_power_setting(void)
{
    U32 ret=0;
#if 0    
    U32 reg_val=0;
#endif    
    U32 chip_version = 0;

    chip_version = upmu_get_cid();
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_low_power_setting] CLKCTL - 20121018 by Juinn-Ting\n");

#if 1
    upmu_set_vio18_vsleep_en(1);
	/* top */
    ret = pmic_config_interface(0x102  , 0x8080 , 0x8080 ,0);
    ret = pmic_config_interface(0x108  , 0x0882 , 0x0882 ,0);   
    ret = pmic_config_interface(0x12a, 0x0000, 0x8c00, 0);	/* reg_ck:24MHz */
    ret = pmic_config_interface(0x206  , 0x0060 , 0x0060 ,0);
    ret = pmic_config_interface(0x402  , 0x0001 , 0x0001 ,0);

    if (chip_version > PMIC6397_E1_CID_CODE) {
        /* printk("@@@@@@@@0x128 chip_version=0x%x\n", chip_version); */
        ret = pmic_config_interface(0x128  , 0x0000 , 0x0060 ,0);
    }
    /* VTCXO control */
    if (chip_version > PMIC6397_E1_CID_CODE) {
        /* enter low power mode when suspend */
	/* printk("@@@@@@@@0x400 0x446 chip_version=0x%x\n", chip_version); */
	ret = pmic_config_interface(0x400, 0x4400, 0x6c01, 0);
        ret = pmic_config_interface(0x446  , 0x0100 , 0x0100 ,0);
    }    
#endif
    

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_low_power_setting] Done\n");
}

void pmic_setting_depends_rtc(void)
{
    U32 ret=0;

#if (!defined(CONFIG_POWER_EXT) && defined(CONFIG_MTK_RTC))
    if( crystal_exist_status() )
#else
    if( 0 )
#endif
    {
        // with 32K
        ret = pmic_config_interface(ANALDO_CON1,    3,    0x7,    12); // [14:12]=3(VTCXO_SRCLK_EN_SEL),
        ret = pmic_config_interface(ANALDO_CON1,    1,    0x1,    11); // [11]=1(VTCXO_ON_CTRL), 
        ret = pmic_config_interface(ANALDO_CON1,    0,    0x1,    1);  // [1]=0(VTCXO_LP_SET), 
        ret = pmic_config_interface(ANALDO_CON1,    0,    0x1,    0);  // [0]=0(VTCXO_LP_SEL),
        
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_setting_depends_rtc] With 32K. Reg[0x%x]=0x%x\n", 
            ANALDO_CON1, upmu_get_reg_value(ANALDO_CON1));
    }
    else
    {
        // without 32K
        ret = pmic_config_interface(ANALDO_CON1,    0,    0x1,    11); // [11]=0(VTCXO_ON_CTRL), 
        ret = pmic_config_interface(ANALDO_CON1,    1,    0x1,    10); // [10]=1(RG_VTCXO_EN), 
        ret = pmic_config_interface(ANALDO_CON1,    3,    0x7,    4);  // [6:4]=3(VTCXO_SRCLK_MODE_SEL), 
        ret = pmic_config_interface(ANALDO_CON1,    1,    0x1,    0);  // [0]=1(VTCXO_LP_SEL),

        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_setting_depends_rtc] Without 32K. Reg[0x%x]=0x%x\n", 
            ANALDO_CON1, upmu_get_reg_value(ANALDO_CON1));
    }
}

int g_gpu_status_bit=1;

int pmic_get_gpu_status_bit_info(void)
{
    return g_gpu_status_bit;
}
EXPORT_SYMBOL(pmic_get_gpu_status_bit_info);

int get_spm_gpu_status(void)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[get_spm_gpu_status] wait spm driver service ready\n");

    return 0;
}

void pmic_gpu_power_enable(int power_en)
{
    if(g_gpu_status_bit == 1)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_gpu_power_enable] gpu is not powered by VRF18_2\n");
    }
    else
    {
        if(power_en == 1)
        {
            upmu_set_vgpu_en(1);            
        }
        else
        {
            upmu_set_vgpu_en(0);
        }
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_gpu_power_enable] Reg[0x%x]=%x\n", 
            VGPU_CON7, upmu_get_reg_value(VGPU_CON7));
    }
}
EXPORT_SYMBOL(pmic_gpu_power_enable);

int g_pmic_cid=0;

static int pmic_mt6397_probe(struct platform_device *dev)
{
    U32 ret_val=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "******** MT6397 pmic driver probe!! ********\n" );

#ifdef MTK_BATTERY_NO_HAL
    //init battery wakelock for battery/charger event
    wake_lock_init(&battery_suspend_lock, WAKE_LOCK_SUSPEND, "battery wakelock");    
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "init battery wakelock for battery/charger event.\n" );
#endif

    //get PMIC CID
    ret_val=upmu_get_cid();
    g_pmic_cid=ret_val;
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "MT6397 PMIC CID=0x%x\n", ret_val );

    //VRF18_2 usage protection
    //pmic_vrf18_2_usage_protection();

    //enable rtc 32k to pmic
    rtc_gpio_enable_32k(RTC_GPIO_USER_PMIC);

    //pmic initial setting
    PMIC_INIT_SETTING_V1();
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[PMIC_INIT_SETTING_V1] Done\n");
    PMIC_CUSTOM_SETTING_V1();
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[PMIC_CUSTOM_SETTING_V1] Done\n");

    //pmic low power setting
    pmic_low_power_setting();

    //pmic setting with RTC
    //pmic_setting_depends_rtc();
    
    upmu_set_rg_pwrkey_int_sel(1);
    upmu_set_rg_homekey_int_sel(1);
    upmu_set_rg_homekey_puen(1);
    
    //PMIC Interrupt Service
    PMIC_EINT_SETTING();
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[PMIC_EINT_SETTING] Done\n");
    
    kthread_run(pmic_thread_kthread, NULL, "pmic_thread_kthread");
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_thread_kthread] Done\n");

    //Dump register
    //#ifndef USER_BUILD_KERNEL
    //PMIC_DUMP_ALL_Register();
    //#endif

    #if defined(CONFIG_POWER_EXT)
    ret_val = pmic_config_interface(0x002E,0x0010,0x00FF,0);
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "[pmic_thread_kthread] add for EVB\n");
    #endif

    return 0;
}

static int pmic_mt6397_remove(struct platform_device *dev)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "******** MT6397 pmic driver remove!! ********\n" );

    return 0;
}

static void pmic_mt6397_shutdown(struct platform_device *dev)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "******** MT6397 pmic driver shutdown!! ********\n" );
}

static int pmic_mt6397_suspend(struct platform_device *dev, pm_message_t state)
{
    U32 ret=0;

    //xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "******** MT6397 pmic driver suspend!! ********\n" );
#if 0 //MT6320 config
    //Set PMIC register 0x022A bit[5:4] =00 before system into sleep mode.
    ret = pmic_config_interface(0x22A,0x0,0x3,4);
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Suspend: Reg[0x%x]=0x%x\n",0x22A, upmu_get_reg_value(0x22A));
#endif    

    //Set PMIC CA7, CA15 TRANS_EN to disable(0x0) before system into sleep mode.
    ret = pmic_config_interface(VCA15_CON18, 0x0, PMIC_VCA15_VOSEL_TRANS_EN_MASK, PMIC_VCA15_VOSEL_TRANS_EN_SHIFT);
    ret = pmic_config_interface(VPCA7_CON18, 0x0, PMIC_VPCA7_VOSEL_TRANS_EN_MASK, PMIC_VPCA7_VOSEL_TRANS_EN_SHIFT);
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Resume: Reg[0x%x]=0x%x\n", VCA15_CON18, upmu_get_reg_value(VCA15_CON18));
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Resume: Reg[0x%x]=0x%x\n", VPCA7_CON18, upmu_get_reg_value(VPCA7_CON18));

    return 0;
}

static int pmic_mt6397_resume(struct platform_device *dev)
{
    U32 ret=0;

    //xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "******** MT6397 pmic driver resume!! ********\n" );
    
#if 0 // MT6320 config
    //Set PMIC register 0x022A bit[5:4] =01 after system resume.
    ret = pmic_config_interface(0x22A,0x1,0x3,4);
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Resume: Reg[0x%x]=0x%x\n",0x22A, upmu_get_reg_value(0x22A));
#endif
    
    //Set PMIC CA7, CA15 TRANS_EN to falling enable(0x1) after system resume.
    ret = pmic_config_interface(VCA15_CON18, 0x1, PMIC_VCA15_VOSEL_TRANS_EN_MASK, PMIC_VCA15_VOSEL_TRANS_EN_SHIFT);
    ret = pmic_config_interface(VPCA7_CON18, 0x1, PMIC_VPCA7_VOSEL_TRANS_EN_MASK, PMIC_VPCA7_VOSEL_TRANS_EN_SHIFT);
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Resume: Reg[0x%x]=0x%x\n", VCA15_CON18, upmu_get_reg_value(VCA15_CON18));
    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "Resume: Reg[0x%x]=0x%x\n", VPCA7_CON18, upmu_get_reg_value(VPCA7_CON18));

    return 0;
}

struct platform_device pmic_mt6397_device = {
    .name   = "pmic_mt6397",
    .id        = -1,
};

static struct platform_driver pmic_mt6397_driver = {
    .probe        = pmic_mt6397_probe,
    .remove        = pmic_mt6397_remove,
    .shutdown    = pmic_mt6397_shutdown,
    //#ifdef CONFIG_PM
    .suspend    = pmic_mt6397_suspend,
    .resume        = pmic_mt6397_resume,
    //#endif
    .driver     = {
        .name = "pmic_mt6397",
    },
};

//==============================================================================
// PMIC6397 device driver
//==============================================================================
static int mt_pmic_probe(struct platform_device *dev)
{
    int ret_device_file = 0;

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "******** mt_pmic_probe!! ********\n" );

    ret_device_file = device_create_file(&(dev->dev), &dev_attr_pmic_access);

    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BUCK_VPCA7_STATUS);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BUCK_VSRMCA7_STATUS);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BUCK_VCA15_STATUS);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BUCK_VSRMCA15_STATUS);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BUCK_VCORE_STATUS);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BUCK_VDRM_STATUS);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BUCK_VIO18_STATUS);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BUCK_VGPU_STATUS);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VIO28_STATUS);   
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VUSB_STATUS);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VMC_STATUS);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VMCH_STATUS);   
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VEMC_3V3_STATUS);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VCAMD_STATUS);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VCAMIO_STATUS);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VCAMAF_STATUS);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VGP4_STATUS);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VGP5_STATUS);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VGP6_STATUS);        
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VIBR_STATUS);   
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VRTC_STATUS);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VTCXO_STATUS); 
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VA28_STATUS);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VCAMA_STATUS);
    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BUCK_VPCA7_VOLTAGE);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BUCK_VSRMCA7_VOLTAGE);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BUCK_VCA15_VOLTAGE);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BUCK_VSRMCA15_VOLTAGE);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BUCK_VCORE_VOLTAGE);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BUCK_VDRM_VOLTAGE);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BUCK_VIO18_VOLTAGE);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BUCK_VGPU_VOLTAGE);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VIO28_VOLTAGE);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VUSB_VOLTAGE);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VMC_VOLTAGE);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VMCH_VOLTAGE);   
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VEMC_3V3_VOLTAGE);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VCAMD_VOLTAGE);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VCAMIO_VOLTAGE);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VCAMAF_VOLTAGE);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VGP4_VOLTAGE);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VGP5_VOLTAGE);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VGP6_VOLTAGE);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VIBR_VOLTAGE);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VRTC_VOLTAGE);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VTCXO_VOLTAGE);   
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VA28_VOLTAGE);    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_LDO_VCAMA_VOLTAGE);  


    return 0;
}

struct platform_device mt_pmic_device = {
    .name   = "mt-pmic",
    .id        = -1,
};

static struct platform_driver mt_pmic_driver = {
    .probe        = mt_pmic_probe,
    .driver     = {
        .name = "mt-pmic",
    },
};

//==============================================================================
// PMIC6397 mudule init/exit
//==============================================================================
static int __init pmic_mt6397_init(void)
{
    int ret;

    // PMIC device driver register
    ret = platform_device_register(&pmic_mt6397_device);
    if (ret) {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "****[pmic_mt6397_init] Unable to device register(%d)\n", ret);
        return ret;
    }
    ret = platform_driver_register(&pmic_mt6397_driver);
    if (ret) {
        xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "****[pmic_mt6397_init] Unable to register driver (%d)\n", ret);
        return ret;
    }

    // PMIC user space access interface
    ret = platform_device_register(&mt_pmic_device);
    if (ret) {
            xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "****[pmic_mt6397_init] Unable to device register(%d)\n", ret);
            return ret;
    }
    ret = platform_driver_register(&mt_pmic_driver);
    if (ret) {
            xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "****[pmic_mt6397_init] Unable to register driver (%d)\n", ret);
            return ret;
    }

    xlog_printk(ANDROID_LOG_INFO, "Power/PMIC", "****[pmic_mt6397_init] Initialization : DONE !!\n");

    return 0;
}

static void __exit pmic_mt6397_exit (void)
{
}

fs_initcall(pmic_mt6397_init);

//module_init(pmic_mt6397_init);
module_exit(pmic_mt6397_exit);

MODULE_AUTHOR("Tank Hung");
MODULE_DESCRIPTION("MT6397 PMIC Device Driver");
MODULE_LICENSE("GPL");

