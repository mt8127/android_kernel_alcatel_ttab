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
 *   AudDrv_Ana.c
 *
 * Project:
 * --------
 *   MT8127  Audio Driver ana Register setting
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 * Luke Liu
 * Chipeng Chang
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

#include "AudDrv_Common.h"
#include "AudDrv_Ana_6397.h"
#include "AudDrv_Clk.h"
#include "AudDrv_ioctl.h"

// define this to use wrapper to control
#define AUDIO_USING_WRAP_DRIVER
#ifdef AUDIO_USING_WRAP_DRIVER
#include <mach/mt_pmic_wrap.h>
#endif

/*****************************************************************************
 *                         D A T A   T Y P E S
 *****************************************************************************/

void Ana_Set_Reg(uint32 offset, uint32 value, uint32 mask)
{
    // set pmic register or analog CONTROL_IFACE_PATH
    int ret = 0;
#ifdef AUDIO_USING_WRAP_DRIVER
    uint32 Reg_Value = Ana_Get_Reg(offset);
    Reg_Value &= (~mask);
    Reg_Value |= (value & mask);
    ret = pwrap_write(offset, Reg_Value);
    Reg_Value = Ana_Get_Reg(offset);
    if ((Reg_Value & mask) != (value & mask))
    {
        printk("Ana_Set_Reg offset= 0x%x , value = 0x%x mask = 0x%x ret = %d Reg_Value = 0x%x\n", offset, value, mask, ret, Reg_Value);
    }
#endif
}

uint32 Ana_Get_Reg(uint32 offset)
{
    // get pmic register
    int ret = 0;
    uint32 Rdata = 0;
#ifdef AUDIO_USING_WRAP_DRIVER
    ret = pwrap_read(offset, &Rdata);
#endif
    //PRINTK_ANA_REG ("Ana_Get_Reg offset= 0x%x  Rdata = 0x%x ret = %d\n",offset,Rdata,ret);
    return Rdata;
}

void AudDrv_Store_reg_ANA(AudAna_Suspend_Reg *pBackup_reg)
{
    PRINTK_AUDDRV("+AudDrv_Store_reg_ANA \n");

    if (pBackup_reg == NULL)
    {
        PRINTK_AUDDRV("pBackup_reg is null \n");
        PRINTK_AUDDRV("-AudDrv_Store_reg_ANA \n");
        return;
    }

    AudDrv_ANA_Clk_On();

    //backup digital part registers
    pBackup_reg->Suspend_Ana_AFE_UL_DL_CON0 = Ana_Get_Reg(AFE_UL_DL_CON0);
    pBackup_reg->Suspend_Ana_AFE_DL_SRC2_CON0_H = Ana_Get_Reg(AFE_DL_SRC2_CON0_H);
    pBackup_reg->Suspend_Ana_AFE_DL_SRC2_CON0_L = Ana_Get_Reg(AFE_DL_SRC2_CON0_L);
    pBackup_reg->Suspend_Ana_AFE_DL_SDM_CON0 = Ana_Get_Reg(AFE_DL_SDM_CON0);
    pBackup_reg->Suspend_Ana_AFE_DL_SDM_CON1 = Ana_Get_Reg(AFE_DL_SDM_CON1);
    pBackup_reg->Suspend_Ana_AFE_UL_SRC_CON0_H = Ana_Get_Reg(AFE_UL_SRC_CON0_H);
    pBackup_reg->Suspend_Ana_AFE_UL_SRC_CON0_L = Ana_Get_Reg(AFE_UL_SRC_CON0_L);
    pBackup_reg->Suspend_Ana_AFE_UL_SRC_CON1_H = Ana_Get_Reg(AFE_UL_SRC_CON1_H);
    pBackup_reg->Suspend_Ana_AFE_UL_SRC_CON1_L = Ana_Get_Reg(AFE_UL_SRC_CON1_L);
    pBackup_reg->Suspend_Ana_AFE_TOP_CON0 = Ana_Get_Reg(ANA_AFE_TOP_CON0);
    pBackup_reg->Suspend_Ana_AUDIO_TOP_CON0 = Ana_Get_Reg(ANA_AUDIO_TOP_CON0);
    pBackup_reg->Suspend_Ana_AFUNC_AUD_CON0 = Ana_Get_Reg(AFUNC_AUD_CON0);
    pBackup_reg->Suspend_Ana_AFUNC_AUD_CON1 = Ana_Get_Reg(AFUNC_AUD_CON1);
    pBackup_reg->Suspend_Ana_AFUNC_AUD_CON2 = Ana_Get_Reg(AFUNC_AUD_CON2);
    pBackup_reg->Suspend_Ana_AFUNC_AUD_CON3 = Ana_Get_Reg(AFUNC_AUD_CON3);
    pBackup_reg->Suspend_Ana_AFUNC_AUD_CON4 = Ana_Get_Reg(AFUNC_AUD_CON4);
    pBackup_reg->Suspend_Ana_AFE_UP8X_FIFO_CFG0 = Ana_Get_Reg(AFE_UP8X_FIFO_CFG0);
    pBackup_reg->Suspend_Ana_AFE_DL_DC_COMP_CFG0 = Ana_Get_Reg(AFE_DL_DC_COMP_CFG0);
    pBackup_reg->Suspend_Ana_AFE_DL_DC_COMP_CFG1 = Ana_Get_Reg(AFE_DL_DC_COMP_CFG1);
    pBackup_reg->Suspend_Ana_AFE_DL_DC_COMP_CFG2 = Ana_Get_Reg(AFE_DL_DC_COMP_CFG2);
    pBackup_reg->Suspend_Ana_AFE_PMIC_NEWIF_CFG0 = Ana_Get_Reg(AFE_PMIC_NEWIF_CFG0);
    pBackup_reg->Suspend_Ana_AFE_PMIC_NEWIF_CFG1 = Ana_Get_Reg(AFE_PMIC_NEWIF_CFG1);
    pBackup_reg->Suspend_Ana_AFE_PMIC_NEWIF_CFG2 = Ana_Get_Reg(AFE_PMIC_NEWIF_CFG2);
    pBackup_reg->Suspend_Ana_AFE_PMIC_NEWIF_CFG3 = Ana_Get_Reg(AFE_PMIC_NEWIF_CFG3);
    pBackup_reg->Suspend_Ana_AFE_SGEN_CFG0 = Ana_Get_Reg(AFE_SGEN_CFG0);
    pBackup_reg->Suspend_Ana_AFE_SGEN_CFG1 = Ana_Get_Reg(AFE_SGEN_CFG1);


    //backup analog part registers
    pBackup_reg->Suspend_Ana_TOP_CKPDN = Ana_Get_Reg(TOP_CKPDN);
    pBackup_reg->Suspend_Ana_TOP_CKPDN_SET = Ana_Get_Reg(TOP_CKPDN_SET);
    pBackup_reg->Suspend_Ana_TOP_CKPDN_CLR = Ana_Get_Reg(TOP_CKPDN_CLR);
    pBackup_reg->Suspend_Ana_TOP_CKPDN2 = Ana_Get_Reg(TOP_CKPDN2);
    pBackup_reg->Suspend_Ana_TOP_CKPDN2_SET = Ana_Get_Reg(TOP_CKPDN2_SET);
    pBackup_reg->Suspend_Ana_TOP_CKPDN2_CLR = Ana_Get_Reg(TOP_CKPDN2_CLR);
    pBackup_reg->Suspend_Ana_TOP_CKCON1 = Ana_Get_Reg(TOP_CKCON1);    
    pBackup_reg->Suspend_Ana_SPK_CON0 = Ana_Get_Reg(SPK_CON0);
    pBackup_reg->Suspend_Ana_SPK_CON1 = Ana_Get_Reg(SPK_CON1);
    pBackup_reg->Suspend_Ana_SPK_CON2 = Ana_Get_Reg(SPK_CON2);
    pBackup_reg->Suspend_Ana_SPK_CON3 = Ana_Get_Reg(SPK_CON3);
    pBackup_reg->Suspend_Ana_SPK_CON4 = Ana_Get_Reg(SPK_CON4);
    pBackup_reg->Suspend_Ana_SPK_CON5 = Ana_Get_Reg(SPK_CON5);
    pBackup_reg->Suspend_Ana_SPK_CON6 = Ana_Get_Reg(SPK_CON6);
    pBackup_reg->Suspend_Ana_SPK_CON7 = Ana_Get_Reg(SPK_CON7);
    pBackup_reg->Suspend_Ana_SPK_CON8 = Ana_Get_Reg(SPK_CON8);
    pBackup_reg->Suspend_Ana_SPK_CON9 = Ana_Get_Reg(SPK_CON9);
    pBackup_reg->Suspend_Ana_SPK_CON10 = Ana_Get_Reg(SPK_CON10);
    pBackup_reg->Suspend_Ana_SPK_CON11 = Ana_Get_Reg(SPK_CON11);
    pBackup_reg->Suspend_Ana_AUDDAC_CON0 = Ana_Get_Reg(AUDDAC_CON0);
    pBackup_reg->Suspend_Ana_AUDBUF_CFG0 = Ana_Get_Reg(AUDBUF_CFG0);
    pBackup_reg->Suspend_Ana_AUDBUF_CFG1 = Ana_Get_Reg(AUDBUF_CFG1);
    pBackup_reg->Suspend_Ana_AUDBUF_CFG2 = Ana_Get_Reg(AUDBUF_CFG2);
    pBackup_reg->Suspend_Ana_AUDBUF_CFG3 = Ana_Get_Reg(AUDBUF_CFG3);
    pBackup_reg->Suspend_Ana_AUDBUF_CFG4 = Ana_Get_Reg(AUDBUF_CFG4);
    pBackup_reg->Suspend_Ana_IBIASDIST_CFG0 = Ana_Get_Reg(IBIASDIST_CFG0);
    pBackup_reg->Suspend_Ana_AUDACCDEPOP_CFG0 = Ana_Get_Reg(AUDACCDEPOP_CFG0);
    pBackup_reg->Suspend_Ana_AUD_IV_CFG0 = Ana_Get_Reg(AUD_IV_CFG0);
    pBackup_reg->Suspend_Ana_AUDCLKGEN_CFG0 = Ana_Get_Reg(AUDCLKGEN_CFG0);
    pBackup_reg->Suspend_Ana_AUDLDO_CFG0 = Ana_Get_Reg(AUDLDO_CFG0);
    pBackup_reg->Suspend_Ana_AUDLDO_CFG1 = Ana_Get_Reg(AUDLDO_CFG1);
    pBackup_reg->Suspend_Ana_AUDNVREGGLB_CFG0 = Ana_Get_Reg(AUDNVREGGLB_CFG0);
    pBackup_reg->Suspend_Ana_AUD_NCP0 = Ana_Get_Reg(AUD_NCP0);
    pBackup_reg->Suspend_Ana_AUDPREAMP_CON0 = Ana_Get_Reg(AUDPREAMP_CON0);
    pBackup_reg->Suspend_Ana_AUDADC_CON0 = Ana_Get_Reg(AUDADC_CON0);
    pBackup_reg->Suspend_Ana_AUDADC_CON1 = Ana_Get_Reg(AUDADC_CON1);
    pBackup_reg->Suspend_Ana_AUDADC_CON2 = Ana_Get_Reg(AUDADC_CON2);
    pBackup_reg->Suspend_Ana_AUDADC_CON3 = Ana_Get_Reg(AUDADC_CON3);
    pBackup_reg->Suspend_Ana_AUDADC_CON4 = Ana_Get_Reg(AUDADC_CON4);
    pBackup_reg->Suspend_Ana_AUDADC_CON5 = Ana_Get_Reg(AUDADC_CON5);
    pBackup_reg->Suspend_Ana_AUDADC_CON6 = Ana_Get_Reg(AUDADC_CON6);
    pBackup_reg->Suspend_Ana_AUDDIGMI_CON0 = Ana_Get_Reg(AUDDIGMI_CON0);
    pBackup_reg->Suspend_Ana_AUDLSBUF_CON0 = Ana_Get_Reg(AUDLSBUF_CON0);
    pBackup_reg->Suspend_Ana_AUDLSBUF_CON1 = Ana_Get_Reg(AUDLSBUF_CON1);
    pBackup_reg->Suspend_Ana_AUDENCSPARE_CON0 = Ana_Get_Reg(AUDENCSPARE_CON0);
    pBackup_reg->Suspend_Ana_AUDENCCLKSQ_CON0 = Ana_Get_Reg(AUDENCCLKSQ_CON0);
    pBackup_reg->Suspend_Ana_AUDPREAMPGAIN_CON0 = Ana_Get_Reg(AUDPREAMPGAIN_CON0);
    pBackup_reg->Suspend_Ana_ZCD_CON0 = Ana_Get_Reg(ZCD_CON0);
    pBackup_reg->Suspend_Ana_ZCD_CON1 = Ana_Get_Reg(ZCD_CON1);
    pBackup_reg->Suspend_Ana_ZCD_CON2 = Ana_Get_Reg(ZCD_CON2);
    pBackup_reg->Suspend_Ana_ZCD_CON3 = Ana_Get_Reg(ZCD_CON3);
    pBackup_reg->Suspend_Ana_ZCD_CON4 = Ana_Get_Reg(ZCD_CON4);
    pBackup_reg->Suspend_Ana_ZCD_CON5 = Ana_Get_Reg(ZCD_CON5);
    pBackup_reg->Suspend_Ana_NCP_CLKDIV_CON0 = Ana_Get_Reg(NCP_CLKDIV_CON0);
    pBackup_reg->Suspend_Ana_NCP_CLKDIV_CON1 = Ana_Get_Reg(NCP_CLKDIV_CON1);
    AudDrv_ANA_Clk_Off();

}

void AudDrv_Recover_reg_ANA(AudAna_Suspend_Reg *pBackup_reg)
{
    PRINTK_AUDDRV("+AudDrv_Recover_reg_ANA \n");

    if (pBackup_reg == NULL)
    {
        PRINTK_AUDDRV("pBackup_reg is null \n");
        PRINTK_AUDDRV("-AudDrv_Recover_reg_ANA \n");
        return;
    }

    AudDrv_ANA_Clk_On();

    Ana_Set_Reg(SPK_CON0, 0x0000, 0xffff); // 1
    Ana_Set_Reg(SPK_CON3, 0x0000, 0xffff); // 1
    Ana_Set_Reg(SPK_CON11, 0x0000, 0xffff); // 1
    Ana_Set_Reg(AUDBUF_CFG0,0x0000,0xffff);
    Ana_Set_Reg(IBIASDIST_CFG0,0x1552,0xffff);
    Ana_Set_Reg(AUDDAC_CON0,0x0000,0xffff);
    Ana_Set_Reg(AUD_IV_CFG0, 0x0000, 0xffff); // 1
    Ana_Set_Reg(AUDCLKGEN_CFG0,0x0000,0xffff);
    Ana_Set_Reg(AUDNVREGGLB_CFG0,0x0004,0xffff);
    Ana_Set_Reg(NCP_CLKDIV_CON1,0x0001,0xffff);
    Ana_Set_Reg(AUD_NCP0,0x8000,0xffff);
    Ana_Set_Reg(AUDLDO_CFG0,0x0192,0xffff);

    Ana_Set_Reg(AFE_UL_DL_CON0, pBackup_reg->Suspend_Ana_AFE_UL_DL_CON0,0xFFFF);
    Ana_Set_Reg(AFE_DL_SRC2_CON0_H, pBackup_reg->Suspend_Ana_AFE_DL_SRC2_CON0_H,0xFFFF);
    Ana_Set_Reg(AFE_DL_SRC2_CON0_L, pBackup_reg->Suspend_Ana_AFE_DL_SRC2_CON0_L,0xFFFF);
    Ana_Set_Reg(AFE_DL_SDM_CON0, pBackup_reg->Suspend_Ana_AFE_DL_SDM_CON0,0xFFFF);
    Ana_Set_Reg(AFE_DL_SDM_CON1, pBackup_reg->Suspend_Ana_AFE_DL_SDM_CON1,0xFFFF);
    Ana_Set_Reg(AFE_UL_SRC_CON0_H, pBackup_reg->Suspend_Ana_AFE_UL_SRC_CON0_H,0xFFFF);
    Ana_Set_Reg(AFE_UL_SRC_CON0_L, pBackup_reg->Suspend_Ana_AFE_UL_SRC_CON0_L,0xFFFF);
    Ana_Set_Reg(AFE_UL_SRC_CON1_H, pBackup_reg->Suspend_Ana_AFE_UL_SRC_CON1_H,0xFFFF);
    Ana_Set_Reg(AFE_UL_SRC_CON1_L, pBackup_reg->Suspend_Ana_AFE_UL_SRC_CON1_L,0xFFFF);
    Ana_Set_Reg(ANA_AFE_TOP_CON0, pBackup_reg->Suspend_Ana_AFE_TOP_CON0,0xFFFF);
    Ana_Set_Reg(ANA_AUDIO_TOP_CON0, pBackup_reg->Suspend_Ana_AUDIO_TOP_CON0,0xFFFF);
    Ana_Set_Reg(AFUNC_AUD_CON0, pBackup_reg->Suspend_Ana_AFUNC_AUD_CON0,0xFFFF);
    Ana_Set_Reg(AFUNC_AUD_CON1, pBackup_reg->Suspend_Ana_AFUNC_AUD_CON1,0xFFFF);
    Ana_Set_Reg(AFUNC_AUD_CON2, pBackup_reg->Suspend_Ana_AFUNC_AUD_CON2,0xFFFF);
    Ana_Set_Reg(AFUNC_AUD_CON3, pBackup_reg->Suspend_Ana_AFUNC_AUD_CON3,0xFFFF);
    Ana_Set_Reg(AFUNC_AUD_CON4, pBackup_reg->Suspend_Ana_AFUNC_AUD_CON4,0xFFFF);
    Ana_Set_Reg(AFE_UP8X_FIFO_CFG0, pBackup_reg->Suspend_Ana_AFE_UP8X_FIFO_CFG0,0xFFFF);
    Ana_Set_Reg(AFE_DL_DC_COMP_CFG0, pBackup_reg->Suspend_Ana_AFE_DL_DC_COMP_CFG0,0xFFFF);
    Ana_Set_Reg(AFE_DL_DC_COMP_CFG1, pBackup_reg->Suspend_Ana_AFE_DL_DC_COMP_CFG1,0xFFFF);
    Ana_Set_Reg(AFE_DL_DC_COMP_CFG2, pBackup_reg->Suspend_Ana_AFE_DL_DC_COMP_CFG2,0xFFFF);
    Ana_Set_Reg(AFE_PMIC_NEWIF_CFG0, pBackup_reg->Suspend_Ana_AFE_PMIC_NEWIF_CFG0,0xFFFF);
    Ana_Set_Reg(AFE_PMIC_NEWIF_CFG1, pBackup_reg->Suspend_Ana_AFE_PMIC_NEWIF_CFG1,0xFFFF);
    Ana_Set_Reg(AFE_PMIC_NEWIF_CFG2, pBackup_reg->Suspend_Ana_AFE_PMIC_NEWIF_CFG2,0xFFFF);
    Ana_Set_Reg(AFE_PMIC_NEWIF_CFG3, pBackup_reg->Suspend_Ana_AFE_PMIC_NEWIF_CFG3,0xFFFF);
    Ana_Set_Reg(AFE_SGEN_CFG0, pBackup_reg->Suspend_Ana_AFE_SGEN_CFG0,0xFFFF);
    Ana_Set_Reg(AFE_SGEN_CFG1, pBackup_reg->Suspend_Ana_AFE_SGEN_CFG1,0xFFFF);


    //backup analog part registers
    //Ana_Set_Reg(TOP_CKPDN, pBackup_reg->Suspend_Ana_TOP_CKPDN,0xFFFF);
    //Ana_Set_Reg(TOP_CKPDN2, pBackup_reg->Suspend_Ana_TOP_CKPDN2,0xFFFF);
    //Ana_Set_Reg(TOP_CKCON1, pBackup_reg->Suspend_Ana_TOP_CKCON1,0xFFFF);    
    Ana_Set_Reg(SPK_CON0, pBackup_reg->Suspend_Ana_SPK_CON0, 0xFFFF);
    Ana_Set_Reg(SPK_CON1, pBackup_reg->Suspend_Ana_SPK_CON1, 0xFFFF);
    Ana_Set_Reg(SPK_CON2, pBackup_reg->Suspend_Ana_SPK_CON2, 0xFFFF);
    Ana_Set_Reg(SPK_CON3, pBackup_reg->Suspend_Ana_SPK_CON3,0xFFFF);
    Ana_Set_Reg(SPK_CON4, pBackup_reg->Suspend_Ana_SPK_CON4,0xFFFF);
    Ana_Set_Reg(SPK_CON5, pBackup_reg->Suspend_Ana_SPK_CON5,0xFFFF);
    Ana_Set_Reg(SPK_CON6, pBackup_reg->Suspend_Ana_SPK_CON6, 0xFFFF);
    Ana_Set_Reg(SPK_CON7, pBackup_reg->Suspend_Ana_SPK_CON7, 0xFFFF);
    Ana_Set_Reg(SPK_CON8, pBackup_reg->Suspend_Ana_SPK_CON8, 0xFFFF);
    Ana_Set_Reg(SPK_CON9, pBackup_reg->Suspend_Ana_SPK_CON9, 0xFFFF);
    Ana_Set_Reg(SPK_CON10, pBackup_reg->Suspend_Ana_SPK_CON10, 0xFFFF);
    Ana_Set_Reg(SPK_CON11, pBackup_reg->Suspend_Ana_SPK_CON11, 0xFFFF);
    //Ana_Set_Reg(AUDDAC_CON0, pBackup_reg->Suspend_Ana_AUDDAC_CON0,0xFFFF);
    //Ana_Set_Reg(AUDBUF_CFG0, pBackup_reg->Suspend_Ana_AUDBUF_CFG0,0xFFFF);
    Ana_Set_Reg(AUDBUF_CFG1, pBackup_reg->Suspend_Ana_AUDBUF_CFG1,0xFFFF);
    Ana_Set_Reg(AUDBUF_CFG2, pBackup_reg->Suspend_Ana_AUDBUF_CFG2,0xFFFF);
    Ana_Set_Reg(AUDBUF_CFG3, pBackup_reg->Suspend_Ana_AUDBUF_CFG3,0xFFFF);
    Ana_Set_Reg(AUDBUF_CFG4, pBackup_reg->Suspend_Ana_AUDBUF_CFG4,0xFFFF);
    //Ana_Set_Reg(IBIASDIST_CFG0, pBackup_reg->Suspend_Ana_IBIASDIST_CFG0,0xFFFF);
    Ana_Set_Reg(AUDACCDEPOP_CFG0, pBackup_reg->Suspend_Ana_AUDACCDEPOP_CFG0,0xFFFF);
    Ana_Set_Reg(AUD_IV_CFG0, pBackup_reg->Suspend_Ana_AUD_IV_CFG0,0xFFFF);
    //Ana_Set_Reg(AUDCLKGEN_CFG0, pBackup_reg->Suspend_Ana_AUDCLKGEN_CFG0,0xFFFF);
    //Ana_Set_Reg(AUDLDO_CFG0, pBackup_reg->Suspend_Ana_AUDLDO_CFG0,0xFFFF);
    Ana_Set_Reg(AUDLDO_CFG1, pBackup_reg->Suspend_Ana_AUDLDO_CFG1,0xFFFF);
    //Ana_Set_Reg(AUDNVREGGLB_CFG0, pBackup_reg->Suspend_Ana_AUDNVREGGLB_CFG0,0xFFFF);
    //Ana_Set_Reg(AUD_NCP0, pBackup_reg->Suspend_Ana_AUD_NCP0,0xFFFF);
    Ana_Set_Reg(AUDPREAMP_CON0, pBackup_reg->Suspend_Ana_AUDPREAMP_CON0,0xFFFF);
    Ana_Set_Reg(AUDADC_CON0, pBackup_reg->Suspend_Ana_AUDADC_CON0,0xFFFF);
    Ana_Set_Reg(AUDADC_CON1, pBackup_reg->Suspend_Ana_AUDADC_CON1,0xFFFF);
    Ana_Set_Reg(AUDADC_CON2, pBackup_reg->Suspend_Ana_AUDADC_CON2,0xFFFF);
    Ana_Set_Reg(AUDADC_CON3, pBackup_reg->Suspend_Ana_AUDADC_CON3,0xFFFF);
    Ana_Set_Reg(AUDADC_CON4, pBackup_reg->Suspend_Ana_AUDADC_CON4,0xFFFF);
    Ana_Set_Reg(AUDADC_CON5, pBackup_reg->Suspend_Ana_AUDADC_CON5,0xFFFF);
    Ana_Set_Reg(AUDADC_CON6, pBackup_reg->Suspend_Ana_AUDADC_CON6,0xFFFF);
    Ana_Set_Reg(AUDDIGMI_CON0, pBackup_reg->Suspend_Ana_AUDDIGMI_CON0,0xFFFF);
    Ana_Set_Reg(AUDLSBUF_CON0, pBackup_reg->Suspend_Ana_AUDLSBUF_CON0,0xFFFF);
    Ana_Set_Reg(AUDLSBUF_CON1, pBackup_reg->Suspend_Ana_AUDLSBUF_CON1,0xFFFF);
    Ana_Set_Reg(AUDENCSPARE_CON0, pBackup_reg->Suspend_Ana_AUDENCSPARE_CON0,0xFFFF);
    Ana_Set_Reg(AUDENCCLKSQ_CON0, pBackup_reg->Suspend_Ana_AUDENCCLKSQ_CON0,0xFFFF);
    Ana_Set_Reg(AUDPREAMPGAIN_CON0, pBackup_reg->Suspend_Ana_AUDPREAMPGAIN_CON0,0xFFFF);
    Ana_Set_Reg(ZCD_CON0, pBackup_reg->Suspend_Ana_ZCD_CON0,0xFFFF);
    Ana_Set_Reg(ZCD_CON1, pBackup_reg->Suspend_Ana_ZCD_CON1,0xFFFF);
    Ana_Set_Reg(ZCD_CON2, pBackup_reg->Suspend_Ana_ZCD_CON2,0xFFFF);
    Ana_Set_Reg(ZCD_CON3, pBackup_reg->Suspend_Ana_ZCD_CON3,0xFFFF);
    Ana_Set_Reg(ZCD_CON4, pBackup_reg->Suspend_Ana_ZCD_CON4,0xFFFF);
    Ana_Set_Reg(ZCD_CON5, pBackup_reg->Suspend_Ana_ZCD_CON5,0xFFFF);
    Ana_Set_Reg(NCP_CLKDIV_CON0, pBackup_reg->Suspend_Ana_NCP_CLKDIV_CON0,0xFFFF);
    //Ana_Set_Reg(NCP_CLKDIV_CON1, pBackup_reg->Suspend_Ana_NCP_CLKDIV_CON1,0xFFFF);

    PRINTK_AUDDRV("-AudDrv_Recover_reg_ANA \n");
    AudDrv_ANA_Clk_Off();

}

void Ana_Log_Print(void)
{
    AudDrv_ANA_Clk_On();
    printk("+Ana_Log_Print \n");
    printk("AFE_UL_DL_CON0	= 0x%x\n", Ana_Get_Reg(AFE_UL_DL_CON0));
    printk("AFE_DL_SRC2_CON0_H	= 0x%x\n", Ana_Get_Reg(AFE_DL_SRC2_CON0_H));
    printk("AFE_DL_SRC2_CON0_L	= 0x%x\n", Ana_Get_Reg(AFE_DL_SRC2_CON0_L));
    printk("AFE_DL_SDM_CON0  = 0x%x\n", Ana_Get_Reg(AFE_DL_SDM_CON0));
    printk("AFE_DL_SDM_CON1  = 0x%x\n", Ana_Get_Reg(AFE_DL_SDM_CON1));
    printk("AFE_UL_SRC_CON0_H  = 0x%x\n", Ana_Get_Reg(AFE_UL_SRC_CON0_H));
    printk("AFE_UL_SRC_CON0_L  = 0x%x\n", Ana_Get_Reg(AFE_UL_SRC_CON0_L));
    printk("AFE_UL_SRC_CON1_H  = 0x%x\n", Ana_Get_Reg(AFE_UL_SRC_CON1_H));
    printk("AFE_UL_SRC_CON1_L  = 0x%x\n", Ana_Get_Reg(AFE_UL_SRC_CON1_L));
    printk("ANA_AFE_TOP_CON0  = 0x%x\n", Ana_Get_Reg(ANA_AFE_TOP_CON0));
    printk("ANA_AUDIO_TOP_CON0	= 0x%x\n", Ana_Get_Reg(ANA_AUDIO_TOP_CON0));

    printk("AFUNC_AUD_CON0	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_CON0));
    printk("AFUNC_AUD_CON1	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_CON1));
    printk("AFUNC_AUD_CON2	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_CON2));
    printk("AFUNC_AUD_CON3	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_CON3));
    printk("AFUNC_AUD_CON4	= 0x%x\n", Ana_Get_Reg(AFUNC_AUD_CON4));

    printk("AFE_PMIC_NEWIF_CFG0  = 0x%x\n", Ana_Get_Reg(AFE_PMIC_NEWIF_CFG0));
    printk("AFE_PMIC_NEWIF_CFG1  = 0x%x\n", Ana_Get_Reg(AFE_PMIC_NEWIF_CFG1));
    printk("AFE_PMIC_NEWIF_CFG2  = 0x%x\n", Ana_Get_Reg(AFE_PMIC_NEWIF_CFG2));
    printk("AFE_PMIC_NEWIF_CFG3  = 0x%x\n", Ana_Get_Reg(AFE_PMIC_NEWIF_CFG3));
    printk("AFE_SGEN_CFG0  = 0x%x\n", Ana_Get_Reg(AFE_SGEN_CFG0));
    printk("AFE_SGEN_CFG1  = 0x%x\n", Ana_Get_Reg(AFE_SGEN_CFG1));


    printk("TOP_CKPDN  = 0x%x\n", Ana_Get_Reg(TOP_CKPDN));
    printk("TOP_CKPDN_SET  = 0x%x\n", Ana_Get_Reg(TOP_CKPDN_SET));
    printk("TOP_CKPDN_CLR  = 0x%x\n", Ana_Get_Reg(TOP_CKPDN_CLR));
    printk("TOP_CKPDN2	= 0x%x\n", Ana_Get_Reg(TOP_CKPDN2));
    printk("TOP_CKCON1	= 0x%x\n", Ana_Get_Reg(TOP_CKCON1));
    printk("SPK_CON0  = 0x%x\n", Ana_Get_Reg(SPK_CON0));
    printk("SPK_CON1  = 0x%x\n", Ana_Get_Reg(SPK_CON1));
    printk("SPK_CON2  = 0x%x\n", Ana_Get_Reg(SPK_CON2));
    printk("SPK_CON3  = 0x%x\n", Ana_Get_Reg(SPK_CON3));
    printk("SPK_CON4  = 0x%x\n", Ana_Get_Reg(SPK_CON4));
    printk("SPK_CON5  = 0x%x\n", Ana_Get_Reg(SPK_CON5));
    printk("SPK_CON6  = 0x%x\n", Ana_Get_Reg(SPK_CON6));
    printk("SPK_CON7  = 0x%x\n", Ana_Get_Reg(SPK_CON7));
    printk("SPK_CON8  = 0x%x\n", Ana_Get_Reg(SPK_CON8));
    printk("SPK_CON9  = 0x%x\n", Ana_Get_Reg(SPK_CON9));
    printk("SPK_CON10  = 0x%x\n", Ana_Get_Reg(SPK_CON10));
    printk("SPK_CON11  = 0x%x\n", Ana_Get_Reg(SPK_CON11));

    printk("AUDDAC_CON0  = 0x%x\n", Ana_Get_Reg(AUDDAC_CON0));
    printk("AUDBUF_CFG0  = 0x%x\n", Ana_Get_Reg(AUDBUF_CFG0));
    printk("AUDBUF_CFG1  = 0x%x\n", Ana_Get_Reg(AUDBUF_CFG1));
    printk("AUDBUF_CFG2  = 0x%x\n", Ana_Get_Reg(AUDBUF_CFG2));
    printk("AUDBUF_CFG3  = 0x%x\n", Ana_Get_Reg(AUDBUF_CFG3));
    printk("AUDBUF_CFG4  = 0x%x\n", Ana_Get_Reg(AUDBUF_CFG4));

    printk("IBIASDIST_CFG0	= 0x%x\n", Ana_Get_Reg(IBIASDIST_CFG0));
    printk("AUDACCDEPOP_CFG0  = 0x%x\n", Ana_Get_Reg(AUDACCDEPOP_CFG0));
    printk("AUD_IV_CFG0  = 0x%x\n", Ana_Get_Reg(AUD_IV_CFG0));
    printk("AUDCLKGEN_CFG0	= 0x%x\n", Ana_Get_Reg(AUDCLKGEN_CFG0));
    printk("AUDLDO_CFG0  = 0x%x\n", Ana_Get_Reg(AUDLDO_CFG0));
    printk("AUDLDO_CFG1  = 0x%x\n", Ana_Get_Reg(AUDLDO_CFG1));
    printk("AUDNVREGGLB_CFG0  = 0x%x\n", Ana_Get_Reg(AUDNVREGGLB_CFG0));
    printk("AUD_NCP0  = 0x%x\n", Ana_Get_Reg(AUD_NCP0));
    printk("AUDPREAMP_CON0	= 0x%x\n", Ana_Get_Reg(AUDPREAMP_CON0));
    printk("AUDADC_CON0  = 0x%x\n", Ana_Get_Reg(AUDADC_CON0));
    printk("AUDADC_CON1  = 0x%x\n", Ana_Get_Reg(AUDADC_CON1));
    printk("AUDADC_CON2  = 0x%x\n", Ana_Get_Reg(AUDADC_CON2));
    printk("AUDADC_CON3  = 0x%x\n", Ana_Get_Reg(AUDADC_CON3));
    printk("AUDADC_CON4  = 0x%x\n", Ana_Get_Reg(AUDADC_CON4));
    printk("AUDADC_CON5  = 0x%x\n", Ana_Get_Reg(AUDADC_CON5));
    printk("AUDADC_CON6  = 0x%x\n", Ana_Get_Reg(AUDADC_CON6));
    printk("AUDDIGMI_CON0  = 0x%x\n", Ana_Get_Reg(AUDDIGMI_CON0));
    printk("AUDLSBUF_CON0  = 0x%x\n", Ana_Get_Reg(AUDLSBUF_CON0));
    printk("AUDLSBUF_CON1  = 0x%x\n", Ana_Get_Reg(AUDLSBUF_CON1));
    printk("AUDENCSPARE_CON0  = 0x%x\n", Ana_Get_Reg(AUDENCSPARE_CON0));
    printk("AUDENCCLKSQ_CON0  = 0x%x\n", Ana_Get_Reg(AUDENCCLKSQ_CON0));
    printk("AUDPREAMPGAIN_CON0	= 0x%x\n", Ana_Get_Reg(AUDPREAMPGAIN_CON0));
    printk("ZCD_CON0  = 0x%x\n", Ana_Get_Reg(ZCD_CON0));
    printk("ZCD_CON1  = 0x%x\n", Ana_Get_Reg(ZCD_CON1));
    printk("ZCD_CON2  = 0x%x\n", Ana_Get_Reg(ZCD_CON2));
    printk("ZCD_CON3  = 0x%x\n", Ana_Get_Reg(ZCD_CON3));
    printk("ZCD_CON4  = 0x%x\n", Ana_Get_Reg(ZCD_CON4));
    printk("ZCD_CON5  = 0x%x\n", Ana_Get_Reg(ZCD_CON5));
    printk("NCP_CLKDIV_CON0  = 0x%x\n", Ana_Get_Reg(NCP_CLKDIV_CON0));
    printk("NCP_CLKDIV_CON1  = 0x%x\n", Ana_Get_Reg(NCP_CLKDIV_CON1));
    AudDrv_ANA_Clk_Off();
    printk("-Ana_Log_Print \n");
}


// export symbols for other module using
EXPORT_SYMBOL(Ana_Log_Print);
EXPORT_SYMBOL(Ana_Set_Reg);
EXPORT_SYMBOL(Ana_Get_Reg);


