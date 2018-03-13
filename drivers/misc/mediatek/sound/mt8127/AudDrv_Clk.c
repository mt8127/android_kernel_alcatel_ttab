/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*******************************************************************************
 *
 * Filename:
 * ---------
 *   AudDrv_Clk.c
 *
 * Project:
 * --------
 *   MT8127  Audio Driver clock control implement
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 * Chipeng Chang (MTK02308)
 *
 *------------------------------------------------------------------------------
 * $Revision: #1 $
 * $Modtime:$
 * $Log:$
 *
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/
#include <mach/mt_clkmgr.h>
#include <mach/mt_pm_ldo.h>
#ifdef CONFIG_MTK_PMIC_MT6397
#include <mach/pmic_mt6397_sw.h>
#include "AudDrv_Ana_6397.h"
#else
#include <mach/pmic_mt6323_sw.h>
#endif
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>

#include "AudDrv_Common.h"
#include "AudDrv_Clk.h"
#include "AudDrv_Afe.h"
#include <linux/spinlock.h>
#include <linux/delay.h>



/*****************************************************************************
 *                         D A T A   T Y P E S
 *****************************************************************************/

int         Aud_Core_Clk_cntr   = 0;
int        Aud_AFE_Clk_cntr    = 0;
int        Aud_ADC_Clk_cntr    = 0;
int        Aud_I2S_Clk_cntr    = 0;
int        Aud_ANA_Clk_cntr    = 0;
int        Aud_LineIn_Clk_cntr = 0;
int        Aud_HDMI_Clk_cntr = 0;
int        Afe_Mem_Pwr_on = 0;
int        Aud_APLL_Tuner_Clk_cntr = 0;
int        Aud_SPDIF_Clk_cntr = 0;
int        Aud_HDMI_DVT_Clk_cntr = 0;
int        Aud_TOP_APLL_Clk_cntr = 0;
static DEFINE_SPINLOCK(auddrv_Clk_lock);
// amp mutex lock
static DEFINE_MUTEX(auddrv_pmic_mutex);

/*****************************************************************************
 *                         INLINE FUNCTION
 *****************************************************************************/
inline void TURN_ON_AFE_CLOCK(void)
{
    if (enable_clock(MT_CG_INFRA_AUDIO, "AUDIO"))
    {
        xlog_printk(ANDROID_LOG_ERROR, "Sound", "Aud enable_clock MT_CG_INFRA_AUDIO fail !\n");
    }
    if (enable_clock(MT_CG_AUDIO_AFE, "AUDIO"))
    {
        xlog_printk(ANDROID_LOG_ERROR, "Sound", "Aud enable_clock MT_CG_AUDIO_AFE fail !\n");
    }
}

inline void TURN_OFF_AFE_CLOCK(void)
{
    if (disable_clock(MT_CG_AUDIO_AFE, "AUDIO"))
    {
        xlog_printk(ANDROID_LOG_ERROR, "Sound", "Aud disable_clock MT_CG_AUDIO_AFE fail !\n");
    }
    if (disable_clock(MT_CG_INFRA_AUDIO, "AUDIO"))
    {
        xlog_printk(ANDROID_LOG_ERROR, "Sound", "Aud disable_clock MT_CG_INFRA_AUDIO fail !\n");
    }
}

inline void TURN_ON_HDMI_CLOCK(void)
{
    if (enable_clock(MT_CG_INFRA_AUDIO, "AUDIO"))
    {
        xlog_printk(ANDROID_LOG_ERROR, "Sound", "Aud enable_clock MT_CG_INFRA_AUDIO fail !\n");
    }
    if (enable_clock(MT_CG_AUDIO_HDMI_CK, "AUDIO"))
    {
        xlog_printk(ANDROID_LOG_ERROR, "Sound", "Aud enable_clock MT_CG_AUDIO_HDMI_CK fail !\n");
    }
}

inline void TURN_OFF_HDMI_CLOCK(void)
{
    if (disable_clock(MT_CG_AUDIO_HDMI_CK, "AUDIO"))
    {
        xlog_printk(ANDROID_LOG_ERROR, "Sound", "Aud disable_clock MT_CG_AUDIO_HDMI_CK fail !\n");
    }
    if (disable_clock(MT_CG_INFRA_AUDIO, "AUDIO"))
    {
        xlog_printk(ANDROID_LOG_ERROR, "Sound", "Aud disable_clock MT_CG_INFRA_AUDIO fail !\n");
    }
}

inline void TURN_ON_APLL_TUNER_CLOCK(void)
{
    if (enable_clock(MT_CG_INFRA_AUDIO, "AUDIO"))
    {
        xlog_printk(ANDROID_LOG_ERROR, "Sound", "Aud enable_clock MT_CG_INFRA_AUDIO fail !\n");
    }
    if (enable_clock(MT_CG_AUDIO_APLL_TUNER_CK, "AUDIO"))
    {
        xlog_printk(ANDROID_LOG_ERROR, "Sound", "Aud enable_clock MT_CG_AUDIO_APLL_TUNER_CK fail !\n");
    }
}

inline void TURN_OFF_APLL_TUNER_CLOCK(void)
{
    if (disable_clock(MT_CG_AUDIO_APLL_TUNER_CK, "AUDIO"))
    {
        xlog_printk(ANDROID_LOG_ERROR, "Sound", "Aud disable_clock MT_CG_AUDIO_APLL_TUNER_CK fail !\n");
    }
    if (disable_clock(MT_CG_INFRA_AUDIO, "AUDIO"))
    {
        xlog_printk(ANDROID_LOG_ERROR, "Sound", "Aud disable_clock MT_CG_INFRA_AUDIO fail !\n");
    }
}

inline void TURN_ON_SPDIF_CLOCK(void)
{
    if (enable_clock(MT_CG_INFRA_AUDIO, "AUDIO"))
    {
        xlog_printk(ANDROID_LOG_ERROR, "Sound", "Aud enable_clock MT_CG_INFRA_AUDIO fail !\n");
    }
    if (enable_clock(MT_CG_AUDIO_SPDF_CK, "AUDIO"))
    {
        xlog_printk(ANDROID_LOG_ERROR, "Sound", "Aud enable_clock MT_CG_AUDIO_SPDF_CK fail !\n");
    }
}

inline void TURN_OFF_SPDIF_CLOCK(void)
{
    if (disable_clock(MT_CG_AUDIO_SPDF_CK, "AUDIO"))
    {
        xlog_printk(ANDROID_LOG_ERROR, "Sound", "disable_clock MT_CG_AUDIO_SPDF_CK fail !\n");
    }
    if (disable_clock(MT_CG_INFRA_AUDIO, "AUDIO"))
    {
        xlog_printk(ANDROID_LOG_ERROR, "Sound", "Aud disable_clock MT_CG_INFRA_AUDIO fail !\n");
    }
}

/*****************************************************************************
 * FUNCTION
 *  AudDrv_Clk_On / AudDrv_Clk_Off
 *
 * DESCRIPTION
 *  Enable/Disable PLL(26M clock) \ AFE clock
 *
 *****************************************************************************
 */
void AudDrv_Clk_On(void)
{
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    if (Aud_AFE_Clk_cntr == 0)
    {
        PRINTK_AUD_CLK("+AudDrv_Clk_On, Aud_AFE_Clk_cntr:%d \n", Aud_AFE_Clk_cntr);
#ifdef PM_MANAGER_API
        TURN_ON_AFE_CLOCK();
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x60004000, 0xffffffff);  // bit2: afe power on
#else
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x60004000, 0xffffffff);  // bit2: afe power on
#endif
    }
    Aud_AFE_Clk_cntr++;
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
    //PRINTK_AUD_CLK("-AudDrv_Clk_On, Aud_AFE_Clk_cntr:%d \n",Aud_AFE_Clk_cntr);
}

void AudDrv_Clk_Off(void)
{
    unsigned long flags;
    //PRINTK_AUD_CLK("+!! AudDrv_Clk_Off, Aud_AFE_Clk_cntr:%d \n",Aud_AFE_Clk_cntr);
    spin_lock_irqsave(&auddrv_Clk_lock, flags);

    Aud_AFE_Clk_cntr--;
    if (Aud_AFE_Clk_cntr == 0)
    {
        PRINTK_AUD_CLK("+ AudDrv_Clk_Off, Aud_AFE_Clk_cntr:%d \n", Aud_AFE_Clk_cntr);
        {
            // Disable AFE clock
#ifdef PM_MANAGER_API
            Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00004044, 0x00004044);    // bit2: power down afe
            TURN_OFF_AFE_CLOCK();
#else
            Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00000000, 0x00004043);  // bit2: power on
#endif
        }
    }
    else if (Aud_AFE_Clk_cntr < 0)
    {
        PRINTK_AUD_ERROR("!! AudDrv_Clk_Off, Aud_AFE_Clk_cntr<0 (%d) \n", Aud_AFE_Clk_cntr);
        AUDIO_ASSERT(true);
        Aud_AFE_Clk_cntr = 0;
    }

    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
    //PRINTK_AUD_CLK("-!! AudDrv_Clk_Off, Aud_AFE_Clk_cntr:%d \n",Aud_AFE_Clk_cntr);
}

/*****************************************************************************
* FUNCTION
*  AudDrv_Suspend_Clk_Off / AudDrv_Suspend_Clk_On
*
* DESCRIPTION
*  Enable/Disable AFE clock for suspend
*
*****************************************************************************
*/
void AudDrv_Suspend_Clk_On(void)
{
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);

    if (Aud_AFE_Clk_cntr > 0)
    {
        PRINTK_AUD_CLK("AudDrv_Suspend_Clk_On Aud_AFE_Clk_cntr:%d ANA_Clk(%d) \n", Aud_AFE_Clk_cntr, Aud_ANA_Clk_cntr);
#ifdef PM_MANAGER_API

        //Enable AFE clock
        TURN_ON_AFE_CLOCK();
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x60004000, 0xffffffff); // bit2: afe power on, bit6: I2S power on
        if (Aud_HDMI_Clk_cntr > 0)
        {
            TURN_ON_HDMI_CLOCK();
        }
        if (Aud_APLL_Tuner_Clk_cntr > 0)
        {
            TURN_ON_APLL_TUNER_CLOCK();
        }
        if (Aud_I2S_Clk_cntr > 0)
        {
            ///Enable I2S clock
            if (enable_clock(MT_CG_AUDIO_I2S, "AUDIO"))
            {
                xlog_printk(ANDROID_LOG_ERROR, "Sound", "AudDrv_Suspend_Clk_On() Aud enable_clock() MT_CG_AUDIO_I2S fail");
            }
        }
        //Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00004000, 0x00004044); // bit2: afe power on, bit6: I2S power on
#else
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00004000, 0x00004044); // bit2: afe power on, bit6: I2S power on
#endif
    }
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
    if (Aud_ANA_Clk_cntr > 0)
    {
        PRINTK_AUD_CLK("AudDrv_Suspend_Clk_On Aud_AFE_Clk_cntr:%d ANA_Clk(%d) \n", Aud_AFE_Clk_cntr, Aud_ANA_Clk_cntr);
    #ifdef CONFIG_MTK_PMIC_MT6397
        upmu_set_rg_clksq_en(1);
    #else
        upmu_set_rg_clksq_en_aud(1);
    #endif
    }
    //PRINTK_AUD_CLK("-AudDrv_Suspend_Clk_On Aud_AFE_Clk_cntr:%d ANA_Clk(%d) \n",Aud_AFE_Clk_cntr,Aud_ANA_Clk_cntr);
}

void AudDrv_Suspend_Clk_Off(void)
{
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    if (Aud_AFE_Clk_cntr > 0)
    {
        PRINTK_AUD_CLK("AudDrv_Suspend_Clk_Off Aud_AFE_Clk_cntr:%d ANA_Clk(%d)\n", Aud_AFE_Clk_cntr, Aud_ANA_Clk_cntr);
#ifdef PM_MANAGER_API
        //Disable AFE clock and I2S clock
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00004044, 0x00004044); // bit2: afe power off, bit6: I2S power off

        if (Aud_APLL_Tuner_Clk_cntr > 0)
        {
            TURN_OFF_APLL_TUNER_CLOCK();
        }
        if (Aud_HDMI_Clk_cntr > 0)
        {
            TURN_OFF_HDMI_CLOCK();
        }

        TURN_OFF_AFE_CLOCK();

        if (Aud_I2S_Clk_cntr > 0)
        {
            if (disable_clock(MT_CG_AUDIO_I2S, "AUDIO"))
            {
                xlog_printk(ANDROID_LOG_ERROR, "Sound", "AudDrv_Suspend_Clk_Off() disable_clock MT_CG_AUDIO_I2S fail");
            }
        }
#else
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00004044, 0x00004044);  // bit2: afe power off, bit6: I2S power off
#endif
    }
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
    if (Aud_ANA_Clk_cntr > 0)
    {
        PRINTK_AUD_CLK("AudDrv_Suspend_Clk_On Aud_AFE_Clk_cntr:%d ANA_Clk(%d) \n", Aud_AFE_Clk_cntr, Aud_ANA_Clk_cntr);
#ifdef CONFIG_MTK_PMIC_MT6397
        upmu_set_rg_clksq_en(0);
#else
        upmu_set_rg_clksq_en_aud(0);
#endif
    }
}

#ifdef CONFIG_MTK_PMIC_MT6397
/*****************************************************************************
 * FUNCTION
 *  AudDrv_ANA_Top_On / AudDrv_ANA_Top_Off
 *
 * DESCRIPTION
 *  Enable/Disable analog part clock
 *
 *****************************************************************************/
void AudDrv_ANA_Top_On(void)
{
    Ana_Set_Reg(TOP_CKPDN_CLR, 0x0003 , 0x00000003);
}

void AudDrv_ANA_Top_Off(void)
{
    Ana_Set_Reg(TOP_CKPDN_SET, 0x0003 , 0x00000003);
}
#endif

/*****************************************************************************
 * FUNCTION
 *  AudDrv_ANA_Clk_On / AudDrv_ANA_Clk_Off
 *
 * DESCRIPTION
 *  Enable/Disable analog part clock
 *
 *****************************************************************************/
void AudDrv_ANA_Clk_On(void)
{
    mutex_lock(&auddrv_pmic_mutex);
    if (Aud_ANA_Clk_cntr == 0)
    {
        PRINTK_AUD_CLK("+AudDrv_ANA_Clk_On, Aud_ANA_Clk_cntr:%d \n", Aud_ANA_Clk_cntr);
#ifdef CONFIG_MTK_PMIC_MT6397
        upmu_set_rg_clksq_en(1);
        AudDrv_ANA_Top_On();
#else
        upmu_set_rg_clksq_en_aud(1);
#endif
    }
    Aud_ANA_Clk_cntr++;
    mutex_unlock(&auddrv_pmic_mutex);
    //PRINTK_AUD_CLK("-AudDrv_ANA_Clk_Off, Aud_ANA_Clk_cntr:%d \n",Aud_ANA_Clk_cntr);
}

void AudDrv_ANA_Clk_Off(void)
{
    //PRINTK_AUD_CLK("+AudDrv_ANA_Clk_Off, Aud_ADC_Clk_cntr:%d \n",  Aud_ANA_Clk_cntr);
    mutex_lock(&auddrv_pmic_mutex);
    Aud_ANA_Clk_cntr--;
    if (Aud_ANA_Clk_cntr == 0)
    {
        PRINTK_AUD_CLK("+AudDrv_ANA_Clk_Off disable_clock Ana clk(%x)\n", Aud_ANA_Clk_cntr);
        // Disable ADC clock
#ifdef PM_MANAGER_API
    #ifdef CONFIG_MTK_PMIC_MT6397
        upmu_set_rg_clksq_en(0);
        AudDrv_ANA_Top_Off();
    #else
        upmu_set_rg_clksq_en_aud(0);
    #endif
#else
        // TODO:: open ADC clock....
#endif
    }
    else if (Aud_ANA_Clk_cntr < 0)
    {
        PRINTK_AUD_ERROR("!! AudDrv_ANA_Clk_Off, Aud_ADC_Clk_cntr<0 (%d) \n", Aud_ANA_Clk_cntr);
        AUDIO_ASSERT(true);
        Aud_ANA_Clk_cntr = 0;
    }
    mutex_unlock(&auddrv_pmic_mutex);
    //PRINTK_AUD_CLK("-AudDrv_ANA_Clk_Off, Aud_ADC_Clk_cntr:%d \n", Aud_ANA_Clk_cntr);
}

/*****************************************************************************
 * FUNCTION
  *  AudDrv_ADC_Clk_On / AudDrv_ADC_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable analog part clock
  *
  *****************************************************************************/
void AudDrv_ADC_Clk_On(void)
{
    //PRINTK_AUDDRV("+AudDrv_ADC_Clk_On, Aud_ADC_Clk_cntr:%d \n", Aud_ADC_Clk_cntr);
    mutex_lock(&auddrv_pmic_mutex);

    if (Aud_ADC_Clk_cntr == 0)
    {
        PRINTK_AUDDRV("+AudDrv_ADC_Clk_On enable_clock ADC clk(%x)\n", Aud_ADC_Clk_cntr);
#ifdef PM_MANAGER_API
        //hwPowerOn(MT65XX_POWER_LDO_VA28,VOL_2800 , "AUDIO");
#endif
    }
    Aud_ADC_Clk_cntr++;
    mutex_unlock(&auddrv_pmic_mutex);
}

void AudDrv_ADC_Clk_Off(void)
{
    //PRINTK_AUDDRV("+AudDrv_ADC_Clk_Off, Aud_ADC_Clk_cntr:%d \n", Aud_ADC_Clk_cntr);
    mutex_lock(&auddrv_pmic_mutex);
    Aud_ADC_Clk_cntr--;
    if (Aud_ADC_Clk_cntr == 0)
    {
        PRINTK_AUDDRV("+AudDrv_ADC_Clk_On disable_clock ADC clk(%x)\n", Aud_ADC_Clk_cntr);
#ifdef PM_MANAGER_API
        //hwPowerDown(MT65XX_POWER_LDO_VA28, "AUDIO");
#endif
    }
    if (Aud_ADC_Clk_cntr < 0)
    {
        PRINTK_AUDDRV("!! AudDrv_ADC_Clk_Off, Aud_ADC_Clk_cntr<0 (%d) \n", Aud_ADC_Clk_cntr);
        Aud_ADC_Clk_cntr = 0;
    }
    mutex_unlock(&auddrv_pmic_mutex);
    //PRINTK_AUDDRV("-AudDrv_ADC_Clk_Off, Aud_ADC_Clk_cntr:%d \n", Aud_ADC_Clk_cntr);
}

/*****************************************************************************
  * FUNCTION
  *  AudDrv_I2S_Clk_On / AudDrv_I2S_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable analog part clock
  *
  *****************************************************************************/
void AudDrv_I2S_Clk_On(void)
{
    unsigned long flags;
    //PRINTK_AUD_CLK("+AudDrv_I2S_Clk_On, Aud_I2S_Clk_cntr:%d \n", Aud_I2S_Clk_cntr);
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    if (Aud_I2S_Clk_cntr == 0)
    {
#ifdef PM_MANAGER_API
        if (enable_clock(MT_CG_AUDIO_I2S, "AUDIO"))
        {
            PRINTK_AUD_ERROR("Aud enable_clock MT65XX_PDN_AUDIO_I2S fail !!!\n");
        }
#else
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00000040, 0x00000040);  //power on I2S clock
#endif
    }
    Aud_I2S_Clk_cntr++;
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_I2S_Clk_Off(void)
{
    unsigned long flags;
    //PRINTK_AUD_CLK("+AudDrv_I2S_Clk_Off, Aud_I2S_Clk_cntr:%d \n", Aud_I2S_Clk_cntr);
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    Aud_I2S_Clk_cntr--;
    if (Aud_I2S_Clk_cntr == 0)
    {
#ifdef PM_MANAGER_API
        if (disable_clock(MT_CG_AUDIO_I2S, "AUDIO"))
        {
            PRINTK_AUD_ERROR("disable_clock MT_CG_AUDIO_I2S fail");
        }
#else
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00000000, 0x00000040);  //power off I2S clock
#endif
    }
    else if (Aud_I2S_Clk_cntr < 0)
    {
        PRINTK_AUD_ERROR("!! AudDrv_I2S_Clk_Off, Aud_I2S_Clk_cntr<0 (%d) \n", Aud_I2S_Clk_cntr);
        AUDIO_ASSERT(true);
        Aud_I2S_Clk_cntr = 0;
    }
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
    //PRINTK_AUD_CLK("-AudDrv_I2S_Clk_Off, Aud_I2S_Clk_cntr:%d \n",Aud_I2S_Clk_cntr);
}

/*****************************************************************************
  * FUNCTION
  *  AudDrv_Core_Clk_On / AudDrv_Core_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable analog part clock
  *
  *****************************************************************************/
void AudDrv_Core_Clk_On(void)
{
    //PRINTK_AUD_CLK("+AudDrv_Core_Clk_On, Aud_Core_Clk_cntr:%d \n", Aud_Core_Clk_cntr);
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    if (Aud_Core_Clk_cntr == 0)
    {
#ifdef PM_MANAGER_API
        TURN_ON_AFE_CLOCK();
#endif
    }
    Aud_Core_Clk_cntr++;
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
    //PRINTK_AUD_CLK("-AudDrv_Core_Clk_On, Aud_Core_Clk_cntr:%d \n", Aud_Core_Clk_cntr);
}


void AudDrv_Core_Clk_Off(void)
{
    //PRINTK_AUD_CLK("+AudDrv_Core_Clk_On, Aud_Core_Clk_cntr:%d \n", Aud_Core_Clk_cntr);
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    if (Aud_Core_Clk_cntr == 0)
    {
#ifdef PM_MANAGER_API
        TURN_OFF_AFE_CLOCK();
#endif
    }
    Aud_Core_Clk_cntr++;
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
    //PRINTK_AUD_CLK("-AudDrv_Core_Clk_On, Aud_Core_Clk_cntr:%d \n", Aud_Core_Clk_cntr);
}


/*****************************************************************************
  * FUNCTION
  *  AudDrv_Linein_Clk_On / AudDrv_Linein_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable analog part clock
  *
  *****************************************************************************/
void AudDrv_Linein_Clk_On(void)
{
    PRINTK_AUD_CLK("+AudDrv_Linein_Clk_On, Aud_I2S_Clk_cntr:%d \n", Aud_LineIn_Clk_cntr);
    if (Aud_LineIn_Clk_cntr == 0)
    {
#ifdef PM_MANAGER_API
        AudDrv_ANA_Clk_On();
        AudDrv_Clk_On();
#else
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00000040, 0x00000040);  //power on I2S clock
#endif
    }
    Aud_LineIn_Clk_cntr++;
}

void AudDrv_Linein_Clk_Off(void)
{
    PRINTK_AUD_CLK("+AudDrv_Linein_Clk_Off, Aud_I2S_Clk_cntr:%d \n", Aud_LineIn_Clk_cntr);
    Aud_LineIn_Clk_cntr--;
    if (Aud_LineIn_Clk_cntr == 0)
    {
#ifdef PM_MANAGER_API
        AudDrv_ANA_Clk_On();
        AudDrv_Clk_On();
#else
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00000000, 0x00000040);  //power off I2S clock
#endif
    }
    else if (Aud_LineIn_Clk_cntr < 0)
    {
        PRINTK_AUD_ERROR("!! AudDrv_Linein_Clk_Off, Aud_I2S_Clk_cntr<0 (%d) \n", Aud_LineIn_Clk_cntr);
        AUDIO_ASSERT(true);
        Aud_LineIn_Clk_cntr = 0;
    }
    PRINTK_AUD_CLK("-AudDrv_I2S_Clk_Off, Aud_I2S_Clk_cntr:%d \n", Aud_LineIn_Clk_cntr);
}

/*****************************************************************************
  * FUNCTION
  *  AudDrv_HDMI_Clk_On / AudDrv_HDMI_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable analog part clock
  *
  *****************************************************************************/
/*
void AudDrv_HDMI_Clk_On(void)
{
    PRINTK_AUD_CLK("+AudDrv_HDMI_Clk_On, Aud_I2S_Clk_cntr:%d \n", Aud_HDMI_Clk_cntr);
    if (Aud_HDMI_Clk_cntr == 0)
    {
        AudDrv_ANA_Clk_On();
        AudDrv_Clk_On();
    }
    Aud_HDMI_Clk_cntr++;
}

void AudDrv_HDMI_Clk_Off(void)
{
    PRINTK_AUD_CLK("+AudDrv_HDMI_Clk_Off, Aud_I2S_Clk_cntr:%d \n", Aud_HDMI_Clk_cntr);
    Aud_HDMI_Clk_cntr--;
    if (Aud_HDMI_Clk_cntr == 0)
    {
        AudDrv_ANA_Clk_Off();
        AudDrv_Clk_Off();
    }
    else if (Aud_HDMI_Clk_cntr < 0)
    {
        PRINTK_AUD_ERROR("!! AudDrv_Linein_Clk_Off, Aud_I2S_Clk_cntr<0 (%d) \n", Aud_HDMI_Clk_cntr);
        AUDIO_ASSERT(true);
        Aud_HDMI_Clk_cntr = 0;
    }
    PRINTK_AUD_CLK("-AudDrv_I2S_Clk_Off, Aud_I2S_Clk_cntr:%d \n", Aud_HDMI_Clk_cntr);
}
*/
/*****************************************************************************
  * FUNCTION
  *  AudDrv_SetHDMIClkSource
  *
  * DESCRIPTION
  *  Set HDMI Source Clock
  *
  *****************************************************************************/
void AudDrv_SetHDMIClkSource(UINT32 SampleRate, int apllclksel)
{
    //TBD
    UINT32 APLL_TUNER_N_INFO = AUDPLL_TUNER_N_98M;
    UINT32 APLL_SDM_PCW = AUDPLL_SDM_PCW_98M; //apll tuner always equal to sdm+1
    UINT32 ck_apll = 0;
    UINT32 u4HDMI_BCK_DIV;
    UINT32 BitWidth = 3; // default = 32 bits
    u4HDMI_BCK_DIV = (128 / ((BitWidth + 1) * 8 * 2) / 2) - 1;
    if ((u4HDMI_BCK_DIV < 0) || (u4HDMI_BCK_DIV > 63))
    {
        PRINTK_AUD_CLK("vClockSetting:u4HDMI_BCK_DIV is out of range.\n");
    }
    ck_apll = apllclksel;


    if ((SampleRate == 44100) || (SampleRate == 88200) || (SampleRate == 176400))
    {
        APLL_TUNER_N_INFO = AUDPLL_TUNER_N_90M;
        APLL_SDM_PCW = AUDPLL_SDM_PCW_90M;
    }

    switch (apllclksel)
    {
        case APLL_D4:
            clkmux_sel(MT_MUX_APLL, 2, "AUDIO");
            break;
        case APLL_D8:
            clkmux_sel(MT_MUX_APLL, 3, "AUDIO");
            break;
        case APLL_D24:
            clkmux_sel(MT_MUX_APLL, 5, "AUDIO");
            break;
        case APLL_D16:
        default: //default 48k
            //APLL_DIV : 2048/128=16
            clkmux_sel(MT_MUX_APLL, 4, "AUDIO");
            break;
    }

    //Set APLL source clock SDM PCW info
#ifdef PM_MANAGER_API
    pll_fsel(AUDPLL, APLL_SDM_PCW);
    // Set HDMI BCK DIV
    Afe_Set_Reg(AUDIO_TOP_CON3, u4HDMI_BCK_DIV << HDMI_BCK_DIV_POS, ((0x1 << HDMI_BCK_DIV_LEN) - 1) << HDMI_BCK_DIV_POS);
#else
    Afe_Set_Reg(AUDPLL_CON1, APLL_SDM_PCW << AUDPLL_SDM_PCW_POS, (0x1 << AUDPLL_SDM_PCW_LEN - 1) << AUDPLL_SDM_PCW_POS);
    //Set APLL tuner clock N info
    Afe_Set_Reg(AUDPLL_CON3, APLL_TUNER_N_INFO << AUDPLL_TUNER_N_INFO_POS, (0x1 << AUDPLL_TUNER_N_INFO_LEN - 1) << AUDPLL_TUNER_N_INFO_POS);
    // Set MCLK clock
    Afe_Set_Reg(CLK_CFG_5, ck_apll << CLK_APLL_SEL_POS, (0x1 << CLK_APLL_SEL_LEN - 1) << CLK_APLL_SEL_POS);
    // Set HDMI BCK DIV
    Afe_Set_Reg(AUDIO_TOP_CON3, u4HDMI_BCK_DIV << HDMI_BCK_DIV_POS, (0x1 << HDMI_BCK_DIV_LEN - 1) << HDMI_BCK_DIV_POS);
    // Turn on APLL source clock
    Afe_Set_Reg(AUDPLL_CON3, 0x1 << AUDPLL_TUNER_EN_POS, (0x1 << 0x1 - 1) << AUDPLL_TUNER_EN_POS);
    // pdn_apll enable turn on
    Afe_Set_Reg(CLK_CFG_5, 0x1 << PDN_APLL_POS, (0x1 << 0x1 - 1) << PDN_APLL_POS);
#endif

}

/*****************************************************************************
* FUNCTION
*  AudDrv_TOP_Apll_Clk_On / AudDrv_TOP_Apll_Clk_Off
*
* DESCRIPTION
*  Enable/Disable top apll clock
*
*****************************************************************************/

void AudDrv_TOP_Apll_Clk_On(void)
{
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    PRINTK_AUD_CLK("+AudDrv_TOP_Apll_Clk_On, Aud_APLL_Tuner_Clk_cntr:%d \n", Aud_TOP_APLL_Clk_cntr);
    if (Aud_TOP_APLL_Clk_cntr == 0)
    {
#ifdef PM_MANAGER_API
        /*   Should be removed, MT8127 clk mngr will set MT_CG_TOP_PDN_APLL when MT_CG_AUDIO_APLL_TUNER_CK, MT_CG_AUDIO_HDMI_CK, MT_CG_AUDIO_SPDF_CK, MT_CG_AUDIO_SPDF2_CK is set
        if (enable_clock(MT_CG_TOP_PDN_APLL , "AUDIO"))
        {
            xlog_printk(ANDROID_LOG_ERROR, "Sound", "Aud enable_clock MT_CG_TOP_PDN_APLL fail !!!\n");
        }
        */
#endif
    }
    Aud_TOP_APLL_Clk_cntr++;
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_TOP_Apll_Clk_Off(void)
{
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    PRINTK_AUD_CLK("+AudDrv_TOP_Apll_Clk_Off, Aud_APLL_Tuner_Clk_cntr:%d \n", Aud_TOP_APLL_Clk_cntr);
    Aud_TOP_APLL_Clk_cntr--;
    if (Aud_TOP_APLL_Clk_cntr == 0)
    {
        // Disable apll tuner clock
#ifdef PM_MANAGER_API
        /*   Should be removed, MT8127 clk mngr will set MT_CG_TOP_PDN_APLL when MT_CG_AUDIO_APLL_TUNER_CK, MT_CG_AUDIO_HDMI_CK, MT_CG_AUDIO_SPDF_CK, MT_CG_AUDIO_SPDF2_CK is set
        if (disable_clock(MT_CG_TOP_PDN_APLL , "AUDIO"))
        {
            xlog_printk(ANDROID_LOG_ERROR, "Sound", "disable_clock MT_CG_TOP_PDN_APLL fail");
        }
        */
#endif
    }
    else if (Aud_TOP_APLL_Clk_cntr < 0)
    {
        PRINTK_AUD_ERROR("!! AudDrv_HDMI_Clk_Off, Aud_TOP_APLL_Clk_cntr<0 (%d) \n", Aud_TOP_APLL_Clk_cntr);
        AUDIO_ASSERT(true);
        Aud_TOP_APLL_Clk_cntr = 0;
    }
    PRINTK_AUD_CLK("-AudDrv_TOP_Apll_Clk_Off, Aud_TOP_APLL_Clk_cntr:%d \n", Aud_TOP_APLL_Clk_cntr);
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

/*****************************************************************************
  * FUNCTION
  *  AudDrv_HDMI_Clk_On / AudDrv_HDMI_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable analog part clock
  *
  *****************************************************************************/

void AudDrv_HDMI_Clk_On(void)
{
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    PRINTK_AUD_CLK("+AudDrv_HDMI_Clk_On, Aud_HDMI_Clk_cntr:%d \n", Aud_HDMI_Clk_cntr);
    if (Aud_HDMI_Clk_cntr == 0)
    {
#ifdef PM_MANAGER_API
        //Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00004000, 0x00004000);
        if((Afe_Get_Reg(AUDIOAFE_TOP_CON0)&&0x00004000)== 0)
            PRINTK_AUDDRV("WARNING, APB3_SEL in AUDIOAFE_TOP_CON0 is not correct!!");
        TURN_ON_HDMI_CLOCK();
#else
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00004000, 0x00104000);
#endif
    }
    Aud_HDMI_Clk_cntr++;
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_HDMI_Clk_Off(void)
{
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    PRINTK_AUD_CLK("+AudDrv_HDMI_Clk_Off, Aud_HDMI_Clk_cntr:%d \n", Aud_HDMI_Clk_cntr);
    Aud_HDMI_Clk_cntr--;
    if (Aud_HDMI_Clk_cntr == 0)
    {
        // Disable HDMI clock
#ifdef PM_MANAGER_API
        //Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00004000, 0x00004000);
        if((Afe_Get_Reg(AUDIOAFE_TOP_CON0)&&0x00004000)== 0)
            PRINTK_AUDDRV("WARNING, APB3_SEL in AUDIOAFE_TOP_CON0 is not correct!!");        
        TURN_OFF_HDMI_CLOCK();
#else
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00104000, 0x00104000);
#endif
    }
    else if (Aud_HDMI_Clk_cntr < 0)
    {
        PRINTK_AUD_ERROR("!! AudDrv_HDMI_Clk_Off, Aud_HDMI_Clk_cntr<0 (%d) \n", Aud_HDMI_Clk_cntr);
        AUDIO_ASSERT(true);
        Aud_HDMI_Clk_cntr = 0;
    }
    PRINTK_AUD_CLK("-AudDrv_HDMI_Clk_Off, Aud_HDMI_Clk_cntr:%d \n", Aud_HDMI_Clk_cntr);
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}
/*****************************************************************************
  * FUNCTION
  *  AudDrv_APLL_TUNER_Clk_On / AudDrv_APLL_TUNER_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable apll tuner clock
  *
  *****************************************************************************/

void AudDrv_APLL_TUNER_Clk_On(void)
{
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    PRINTK_AUD_CLK("+AudDrv_APLL_TUNER_Clk_On, Aud_APLL_Tuner_Clk_cntr:%d \n", Aud_APLL_Tuner_Clk_cntr);
    if (Aud_APLL_Tuner_Clk_cntr == 0)
    {
#ifdef PM_MANAGER_API
        //Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00004000, 0x00004000);
        if((Afe_Get_Reg(AUDIOAFE_TOP_CON0)&&0x00004000)== 0)
            PRINTK_AUDDRV("WARNING, APB3_SEL in AUDIOAFE_TOP_CON0 is not correct!!");        
        TURN_ON_APLL_TUNER_CLOCK();
#else
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00004000, 0x00084000);
#endif
    }
    Aud_APLL_Tuner_Clk_cntr++;
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_APLL_TUNER_Clk_Off(void)
{
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    PRINTK_AUD_CLK("+AudDrv_HDMI_Clk_Off, Aud_APLL_Tuner_Clk_cntr:%d \n", Aud_APLL_Tuner_Clk_cntr);
    Aud_APLL_Tuner_Clk_cntr--;
    if (Aud_APLL_Tuner_Clk_cntr == 0)
    {
        // Disable apll tuner clock
#ifdef PM_MANAGER_API
        //Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00004000, 0x00004000);
        if((Afe_Get_Reg(AUDIOAFE_TOP_CON0)&&0x00004000)== 0)
            PRINTK_AUDDRV("WARNING, APB3_SEL in AUDIOAFE_TOP_CON0 is not correct!!");        
        TURN_OFF_APLL_TUNER_CLOCK();
#else
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00084000, 0x00084000);
#endif
    }
    else if (Aud_APLL_Tuner_Clk_cntr < 0)
    {
        PRINTK_AUD_ERROR("!! AudDrv_HDMI_Clk_Off, Aud_APLL_Tuner_Clk_cntr<0 (%d) \n", Aud_APLL_Tuner_Clk_cntr);
        AUDIO_ASSERT(true);
        Aud_APLL_Tuner_Clk_cntr = 0;
    }
    PRINTK_AUD_CLK("-AudDrv_HDMI_Clk_Off, Aud_APLL_Tuner_Clk_cntr:%d \n", Aud_APLL_Tuner_Clk_cntr);
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}
/*****************************************************************************
  * FUNCTION
  *  AudDrv_SPDIF_Clk_On / AudDrv_SPDIF_Clk_Off
  *
  * DESCRIPTION
  *  Enable/Disable SPDIF clock
  *
  *****************************************************************************/

void AudDrv_SPDIF_Clk_On(void)
{
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    PRINTK_AUD_CLK("+AudDrv_SPDIF_Clk_On, Aud_SPDIF_Clk_cntr:%d\n", Aud_SPDIF_Clk_cntr);
    if (Aud_SPDIF_Clk_cntr == 0)
    {
#ifdef PM_MANAGER_API
        //Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00004000, 0x00004000);
        if((Afe_Get_Reg(AUDIOAFE_TOP_CON0)&&0x00004000)== 0)
            PRINTK_AUDDRV("WARNING, APB3_SEL in AUDIOAFE_TOP_CON0 is not correct!!");        
        TURN_ON_SPDIF_CLOCK();
#else
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00004000, 0x00204000);
#endif
    }
    Aud_SPDIF_Clk_cntr++;
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_SPDIF_Clk_Off(void)
{
    unsigned long flags;
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    PRINTK_AUD_CLK("+AudDrv_SPDIF_Clk_Off, Aud_SPDIF_Clk_cntr:%d\n", Aud_SPDIF_Clk_cntr);
    Aud_SPDIF_Clk_cntr--;
    if (Aud_SPDIF_Clk_cntr == 0)
    {
        // Disable SPDIF clock
#ifdef PM_MANAGER_API
        //Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00004000, 0x00004000);
        if((Afe_Get_Reg(AUDIOAFE_TOP_CON0)&&0x00004000)== 0)
            PRINTK_AUDDRV("WARNING, APB3_SEL in AUDIOAFE_TOP_CON0 is not correct!!");        
        TURN_OFF_SPDIF_CLOCK();
#else
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00204000, 0x00204000);
#endif
    }
    else if (Aud_SPDIF_Clk_cntr < 0)
    {
        PRINTK_AUD_ERROR("!! AudDrv_SPDIF_Clk_Off, Aud_SPDIF_Clk_cntr<0 (%d)\n", Aud_SPDIF_Clk_cntr);
        AUDIO_ASSERT(true);
        Aud_SPDIF_Clk_cntr = 0;
    }
    PRINTK_AUD_CLK("-AudDrv_SPDIF_Clk_Off, Aud_SPDIF_Clk_cntr:%d\n", Aud_SPDIF_Clk_cntr);
    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
}

void AudDrv_Clk_On_DisableISR(void)
{
    unsigned long flags;
    PRINTK_AUD_CLK("+AudDrv_Clk_On_DisableISR");
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    if (Aud_AFE_Clk_cntr == 0)
    {
        PRINTK_AUD_CLK("+AudDrv_Clk_On, Aud_AFE_Clk_cntr:%d \n", Aud_AFE_Clk_cntr);
#ifdef PM_MANAGER_API
        TURN_ON_AFE_CLOCK();
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x60004000, 0xffffffff);  // bit2: afe power on
#else
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x60004000, 0xffffffff);  // bit2: afe power on
#endif
    }

    Afe_Set_Reg(AFE_IRQ_MCU_CON, 0, 0x03);

    if (Aud_AFE_Clk_cntr == 0)
    {
        PRINTK_AUD_CLK("+ AudDrv_Clk_Off, Aud_AFE_Clk_cntr:%d \n", Aud_AFE_Clk_cntr);
        {
            // Disable AFE clock
#ifdef PM_MANAGER_API
            Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00004044, 0x00004044);    // bit2: power down afe
            TURN_OFF_AFE_CLOCK();
#else
            Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00000000, 0x00004043);  // bit2: power on
#endif
        }
    }

    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
    PRINTK_AUD_CLK("-AudDrv_Clk_On_DisableISR");
    //PRINTK_AUD_CLK("-!! AudDrv_Clk_Off, Aud_AFE_Clk_cntr:%d \n",Aud_AFE_Clk_cntr);
}

void AudDrv_Clk_On_ClrISRStatus(void)
{
    unsigned long flags;
    PRINTK_AUD_CLK("+AudDrv_Clk_On_ClrISRStatus");
    spin_lock_irqsave(&auddrv_Clk_lock, flags);
    if (Aud_AFE_Clk_cntr == 0)
    {
        PRINTK_AUD_CLK("+AudDrv_Clk_On, Aud_AFE_Clk_cntr:%d \n", Aud_AFE_Clk_cntr);
#ifdef PM_MANAGER_API
        TURN_ON_AFE_CLOCK();
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x60004000, 0xffffffff);  // bit2: afe power on
#else
        Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x60004000, 0xffffffff);  // bit2: afe power on
#endif
    }

    Afe_Set_Reg(AFE_IRQ_CLR, 1 << 6 , 0xff);
    Afe_Set_Reg(AFE_IRQ_CLR, 1 , 0xff);
    Afe_Set_Reg(AFE_IRQ_CLR, 1 << 1 , 0xff);
    Afe_Set_Reg(AFE_IRQ_CLR, 1 << 2 , 0xff);
    Afe_Set_Reg(AFE_IRQ_CLR, 1 << 3 , 0xff);
    Afe_Set_Reg(AFE_IRQ_CLR, 1 << 4 , 0xff);
    Afe_Set_Reg(AFE_IRQ_CLR, 1 << 5 , 0xff);

     if (Aud_AFE_Clk_cntr == 0)
    {
        PRINTK_AUD_CLK("+ AudDrv_Clk_Off, Aud_AFE_Clk_cntr:%d \n", Aud_AFE_Clk_cntr);
        {
            // Disable AFE clock
#ifdef PM_MANAGER_API
            Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00004044, 0x00004044);    // bit2: power down afe
            TURN_OFF_AFE_CLOCK();
#else
            Afe_Set_Reg(AUDIOAFE_TOP_CON0, 0x00000000, 0x00004043);  // bit2: power on
#endif
        }
    }

    spin_unlock_irqrestore(&auddrv_Clk_lock, flags);
    PRINTK_AUD_CLK("-AudDrv_Clk_On_ClrISRStatus");
    //PRINTK_AUD_CLK("-!! AudDrv_Clk_Off, Aud_AFE_Clk_cntr:%d \n",Aud_AFE_Clk_cntr);
}
// export symbol for other module use
EXPORT_SYMBOL(AudDrv_Clk_On);
EXPORT_SYMBOL(AudDrv_Clk_Off);
EXPORT_SYMBOL(AudDrv_ANA_Clk_On);
EXPORT_SYMBOL(AudDrv_ANA_Clk_Off);
EXPORT_SYMBOL(AudDrv_I2S_Clk_On);
EXPORT_SYMBOL(AudDrv_I2S_Clk_Off);

