#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <mach/mt_typedefs.h>
#include <mach/sync_write.h>
#include <mach/mt_dcm.h>
#include <mach/mt_clkmgr.h>

/* support mtk in-house tee */
#ifdef CONFIG_MTK_IN_HOUSE_TEE_SUPPORT
/* enable protection for SMI DCM due to DAPC protecting SMI control register */
#define SUPPORT_MTEE_SMI_DCM_PROT 1
#else
#define SUPPORT_MTEE_SMI_DCM_PROT 0
#endif


#define USING_XLOG

#ifdef USING_XLOG

#include <linux/xlog.h>
#define TAG     "Power/dcm"

/* #define DCM_ENABLE_DCM_CFG */

#define dcm_err(fmt, args...)       \
	xlog_printk(ANDROID_LOG_ERROR, TAG, fmt, ##args)
#define dcm_warn(fmt, args...)      \
	xlog_printk(ANDROID_LOG_WARN, TAG, fmt, ##args)
#define dcm_info(fmt, args...)      \
	xlog_printk(ANDROID_LOG_INFO, TAG, fmt, ##args)
#define dcm_dbg(fmt, args...)       \
	xlog_printk(ANDROID_LOG_DEBUG, TAG, fmt, ##args)
#define dcm_ver(fmt, args...)       \
	xlog_printk(ANDROID_LOG_VERBOSE, TAG, fmt, ##args)

#else /* !USING_XLOG */

#define TAG     "[Power/dcm] "

#define dcm_err(fmt, args...) \
	pr_err(TAG fmt, ##args)
#define dcm_warn(fmt, args...) \
	pr_warn(TAG fmt, ##args)
#define dcm_info(fmt, args...) \
	pr_notice(TAG fmt, ##args)
#define dcm_dbg(fmt, args...) \
	pr_info(TAG fmt, ##args)
#define dcm_ver(fmt, args...) \
	pr_debug(TAG fmt, ##args)

#endif


#if SUPPORT_MTEE_SMI_DCM_PROT
#include "trustzone/kree/system.h"
#include "tz_cross/trustzone.h"
#include "tz_cross/ta_dcm.h"

/*
	control SMI DCM in MTEE
*/
int i4MTEE_SMI_DCM_Ctrl(unsigned int ui4_enable)
{
	TZ_RESULT ret;
	KREE_SESSION_HANDLE dcm_session;
	MTEEC_PARAM param[4];
	uint32_t cmd;
	uint32_t paramTypes;

	dcm_info("Ctrl SMI DCM in kernel (%d) - start\n", ui4_enable);

	if (0 == ui4_enable)
		cmd = TZCMD_DCM_DISABLE_DCM;
	else
		cmd = TZCMD_DCM_ENABLE_DCM;

	ret = KREE_CreateSession(TZ_TA_DCM_UUID, &dcm_session);
	if (ret != TZ_RESULT_SUCCESS) {
		dcm_info("Error: create dcm_session error %d\n", ret);
		return -1;
	}

	paramTypes = TZ_ParamTypes1(TZPT_VALUE_INPUT);

	param[0].value.a = SMI_DCM;
	ret = KREE_TeeServiceCall(dcm_session, cmd, paramTypes, param);
	if (TZ_RESULT_SUCCESS != ret)
		dcm_info("Error: fail to control SMI DCM in MTEE (%d)\n", ret);

	ret = KREE_CloseSession(dcm_session);
	if (ret != TZ_RESULT_SUCCESS) {
		dcm_info("Error: close dcm_session error %d\n", ret);
		return -1;
	}

	dcm_info("Ctrl SMI DCM in kernel (%d) - end\n", ui4_enable);
	return 0;
}

/*
	Dump SMI DCM register in MTEE
*/
int i4MTEE_SMI_DCM_GetStatus(
	unsigned int *pui4_smi_com_dcm,
	unsigned int *pui4_smi_sec_dcm,
	unsigned int *pui4_m4u_dcm)
{
	TZ_RESULT ret;
	KREE_SESSION_HANDLE dcm_session;
	MTEEC_PARAM param[4];
	uint32_t paramTypes;

	dcm_info("Get SMI DCM status in kernel - start\n");

	if ((NULL == pui4_smi_com_dcm) ||
		(NULL == pui4_smi_sec_dcm) ||
		(NULL == pui4_m4u_dcm)) {
		dcm_warn("Error: NULL pointers for get SMI DCM status !!!\n");
		return -1;
	}

	ret = KREE_CreateSession(TZ_TA_DCM_UUID, &dcm_session);
	if (ret != TZ_RESULT_SUCCESS) {
		dcm_warn("Error: create dcm_session error %d\n", ret);
		return -1;
	}

	paramTypes = TZ_ParamTypes3(TZPT_VALUE_INPUT, TZPT_VALUE_OUTPUT, TZPT_VALUE_OUTPUT);
	param[0].value.a = SMI_DCM;
	param[1].value.a = 0;
	param[1].value.b = 0;
	param[2].value.a = 0;

	ret = KREE_TeeServiceCall(dcm_session, TZCMD_DCM_GET_DCM_STATUS, paramTypes, param);
	if (TZ_RESULT_SUCCESS != ret)
		dcm_info("Error: fail to get status of SMI DCM in MTEE (%d)\n", ret);

	ret = KREE_CloseSession(dcm_session);
	if (ret != TZ_RESULT_SUCCESS) {
		dcm_info("Error: close dcm_session error %d\n", ret);
		return -1;
	}

	*pui4_smi_com_dcm = param[1].value.a;
	*pui4_smi_sec_dcm = param[1].value.b;
	*pui4_m4u_dcm     = param[2].value.a;

	dcm_info("Get SMI DCM status in kernel - done\n");
	return 0;
}

/*
	Dump SMI DCM operation register in MTEE
*/
int i4MTEE_SMI_DCM_GetOpStatus(
	unsigned int *pui4_smi_com_set,
	unsigned int *pui4_smi_com_clr)
{
	TZ_RESULT ret;
	KREE_SESSION_HANDLE dcm_session;
	MTEEC_PARAM param[4];
	uint32_t paramTypes;

	dcm_info("Get SMI DCM op status in kernel - start\n");

	if ((NULL == pui4_smi_com_set) ||
		(NULL == pui4_smi_com_clr)) {
		dcm_warn("Error: NULL pointers for get SMI DCM op status !!!\n");
		return -1;
	}

	ret = KREE_CreateSession(TZ_TA_DCM_UUID, &dcm_session);
	if (ret != TZ_RESULT_SUCCESS) {
		dcm_warn("Error: create dcm_session error %d\n", ret);
		return -1;
	}

	paramTypes = TZ_ParamTypes2(TZPT_VALUE_INPUT, TZPT_VALUE_OUTPUT);
	param[0].value.a = SMI_DCM;
	param[1].value.a = 0;
	param[1].value.b = 0;

	ret = KREE_TeeServiceCall(dcm_session, TZCMD_DCM_GET_DCM_OP_STATUS, paramTypes, param);
	if (TZ_RESULT_SUCCESS != ret)
		dcm_info("Error: fail to get op status of SMI DCM in MTEE (%d)\n", ret);

	ret = KREE_CloseSession(dcm_session);
	if (ret != TZ_RESULT_SUCCESS) {
		dcm_info("Error: close dcm_session error %d\n", ret);
		return -1;
	}

	*pui4_smi_com_set = param[1].value.a;
	*pui4_smi_com_clr = param[1].value.b;

	dcm_info("Get SMI DCM op status in kernel - done\n");
	return 0;
}
#endif /* SUPPORT_MTEE_SMI_DCM_PROT */


#define dcm_readl(addr)         DRV_Reg32(addr)


#define dcm_writel(addr, val)   mt65xx_reg_sync_writel((val), ((void *)addr))

#define dcm_setl(addr, val)     mt65xx_reg_sync_writel(dcm_readl(addr) | (val), ((void *)addr))

#define dcm_clrl(addr, val)     mt65xx_reg_sync_writel(dcm_readl(addr) & ~(val), ((void *)addr))


static DEFINE_MUTEX(dcm_lock);

static unsigned int dcm_sta;

void dcm_dump_regs(unsigned int type)
{
	mutex_lock(&dcm_lock);

	if (type & CPU_DCM) {
		unsigned int mcf_biu_con, ca7_misc_config;

		mcf_biu_con = dcm_readl(MCU_BIU_CON);
		ca7_misc_config = dcm_readl(CA7_MISC_CONFIG);
		dcm_info("[CPU_DCM]MCU_BIU_CON(0x%08x), CA7_MISC_CONFIG(0x%08x)\n",  mcf_biu_con, ca7_misc_config);
	}

	if (type & TOPCKGEN_DCM) {
		unsigned int dcm_cfg, dcm_scp_cfg_cfg0, dcm_scp_cfg_cfg1;
		dcm_cfg = dcm_readl(DCM_CFG);
		dcm_scp_cfg_cfg0 = dcm_readl(CLK_SCP_CFG_0);
		dcm_scp_cfg_cfg1 = dcm_readl(CLK_SCP_CFG_1);

		dcm_info("[IFR_DCM]DCM_CFG(0x%08x)\n", dcm_cfg);
		dcm_info("[IFR_DCM]CLK_SCP_CFG_0(0x%08x)\n", dcm_scp_cfg_cfg0);
		dcm_info("[IFR_DCM]CLK_SCP_CFG_1(0x%08x)\n", dcm_scp_cfg_cfg1);

	}

	if (type & IFR_DCM) {
		unsigned int fsel, dbc, ctl;
		unsigned int dramc;
		fsel = dcm_readl(INFRA_DCMFSEL);
		dbc = dcm_readl(INFRA_DCMDBC);
		ctl = dcm_readl(INFRA_DCMCTL);
		dramc = dcm_readl(DRAMC_PD_CTRL);
		dcm_info("[IFR_DCM]FSEL(0x%08x), DBC(0x%08x), CTL(0x%08x)\n", fsel, dbc, ctl);
		dcm_info("[IFR_DCM]DRAMC_PD_CTRL(0x%08x)\n", dramc);
	}

	if (type & PER_DCM) {
		unsigned int top_ckdiv1, top_dcmctl, top_dcmdbc;
		unsigned int infra_dcmctl, infra_dcmdbc, infra_dcmsel, infra_pd_Ctrl;

		top_ckdiv1    = dcm_readl(TOP_CKDIV1);
		top_dcmctl    = dcm_readl(TOP_DCMCTL);
		top_dcmdbc    = dcm_readl(TOP_DCMDBC);
		infra_dcmctl  = dcm_readl(INFRA_DCMCTL);
		infra_dcmdbc  = dcm_readl(INFRA_DCMDBC);
		infra_dcmsel  = dcm_readl(INFRA_DCMFSEL);
		infra_pd_Ctrl = dcm_readl(DRAMC_PD_CTRL);

		dcm_info("[PER_DCM]TOP_CKDIV1(0x%08x), TOP_DCMCTL(0x%08x), TOP_DCMDBC(0x%08x), INFRA_DCMCTL(0x%08x), INFRA_DCMDBC(0x%08x), INFRA_DCMFSEL(0x%08x), DRAMC_PD_CTRL(0x%08x)\n",
			top_ckdiv1, top_dcmctl, top_dcmdbc, infra_dcmctl, infra_dcmdbc, infra_dcmsel, infra_pd_Ctrl);
	}

	if (type & SMI_DCM) {

		unsigned int smi_dcm_control, smi_common_ao_smi_con, mmu_dcm;
		unsigned int smi_common_ao_smi_con_set, smi_common_ao_smi_con_clr;
#if SUPPORT_MTEE_SMI_DCM_PROT
		/* SMI_SECURE_XXX register is protected by MTEE */
		/* Note: driver initialization should not call this function due to driver iniitializaion sequence */
		int iret;
		iret = i4MTEE_SMI_DCM_GetStatus(&smi_dcm_control, &smi_common_ao_smi_con, &mmu_dcm);
		iret = i4MTEE_SMI_DCM_GetOpStatus(&smi_common_ao_smi_con_set, &smi_common_ao_smi_con_clr);
#else
		smi_dcm_control = dcm_readl(SMI_DCM_CONTROL);

		smi_common_ao_smi_con = dcm_readl(SMI_COMMON_AO_SMI_CON);
		smi_common_ao_smi_con_set = dcm_readl(SMI_COMMON_AO_SMI_CON_SET);
		smi_common_ao_smi_con_clr = dcm_readl(SMI_COMMON_AO_SMI_CON_CLR);

		mmu_dcm = dcm_readl(MMU_DCM);
#endif  /* SUPPORT_MTEE_SMI_DCM_PROT */
		dcm_info("[SMI_DCM]SMI_DCM_CONTROL(0x%08x), SMI_COMMON_AO_SMI_CON(0x%08x), MMU_DCM(0x%08x)\n" ,
				smi_dcm_control, smi_common_ao_smi_con, mmu_dcm);
		dcm_info("[SMI_DCM]SMI_COMMON_AO_SMI_CON_SET(0x%08x)\n", smi_common_ao_smi_con_set);
		dcm_info("[SMI_DCM]SMI_COMMON_AO_SMI_CON_CLR(0x%08x)\n", smi_common_ao_smi_con_clr);

	}

	if (type & MFG_DCM) {
		if (subsys_is_on(SYS_MFG)) {
			unsigned int mfg0;
			mfg0 = dcm_readl(MFG_DCM_CON_0);
			dcm_info("[MFG_DCM]MFG_DCM_CON_0(0x%08x)\n", mfg0);
		} else
			dcm_info("[MFG_DCM]subsy MFG is off\n");
	}

	if (type & DIS_DCM) {
		if (subsys_is_on(SYS_DIS)) {
			unsigned int dis0, dis_set0, dis_clr0, dis1, dis_set1, dis_clr1;
			unsigned int smilarb0_dcm_sta, smilarb0_dcm_con, smilarb0_dcm_set;
			dis0 = dcm_readl(DISP_HW_DCM_DIS0);
			dis_set0 = dcm_readl(DISP_HW_DCM_DIS_SET0);
			dis_clr0 = dcm_readl(DISP_HW_DCM_DIS_CLR0);

			dis1 = dcm_readl(DISP_HW_DCM_DIS1);
			dis_set1 = dcm_readl(DISP_HW_DCM_DIS_SET1);
			dis_clr1 = dcm_readl(DISP_HW_DCM_DIS_CLR1);

			smilarb0_dcm_sta = dcm_readl(SMILARB0_DCM_STA);
			smilarb0_dcm_con = dcm_readl(SMILARB0_DCM_CON);
			smilarb0_dcm_set = dcm_readl(SMILARB0_DCM_SET);

			dcm_info("[DIS_DCM]DISP_HW_DCM_DIS_SET0(0x%08x), DISP_HW_DCM_DIS_CLR0(0x%08x), DISP_HW_DCM_DIS_CLR0(0x%08x)\n",
					dis0, dis_set0, dis_clr0);
			dcm_info("[DIS_DCM]DISP_HW_DCM_DIS_SET1(0x%08x), DISP_HW_DCM_DIS_CLR1(0x%08x), DISP_HW_DCM_DIS_CLR1(0x%08x)\n",
					dis1, dis_set1, dis_clr1);
			dcm_info("[DIS_DCM]SMILARB0_DCM_STA(0x%08x), SMILARB0_DCM_CON(0x%08x), SMILARB0_DCM_SET(0x%08x)\n",
				smilarb0_dcm_sta, smilarb0_dcm_con, smilarb0_dcm_set);
		} else
			dcm_info("[DIS_DCM]subsys DIS is off\n");
	}

	if (type & ISP_DCM) {
		if (subsys_is_on(SYS_ISP)) {
			unsigned int raw, rgb, yuv, cdp, dma;
			unsigned int  jpgenc, venc_dcm, venc_cg, smilarb2_dcm_sta, smilarb2_dcm_com;
			raw = dcm_readl(CAM_CTL_RAW_DCM);
			rgb = dcm_readl(CAM_CTL_RGB_DCM);
			yuv = dcm_readl(CAM_CTL_YUV_DCM);
			cdp = dcm_readl(CAM_CTL_CDP_DCM);
			dma = dcm_readl(CAM_CTL_DMA_DCM);
			jpgenc = dcm_readl(JPGENC_DCM_CTRL);
			venc_dcm = dcm_readl(VENC_CLK_DCM_CTRL);
			venc_cg = dcm_readl(VENC_CLK_CG_CTRL);
			smilarb2_dcm_sta = dcm_readl(SMILARB2_DCM_STA);
			smilarb2_dcm_com = dcm_readl(SMILARB2_DCM_CON);

			dcm_info("[ISP_DCM]CAM_CTL_RAW_DCM(0x%08x), CAM_CTL_RGB_DCM(0x%08x)\n",
					raw, rgb);
			dcm_info("[ISP_DCM]CAM_CTL_YUV_DCM(0x%08x), CAM_CTL_CDP_DCM(0x%08x)\n",
					yuv, cdp);
			dcm_info("[ISP_DCM] JPGENC_DCM_CTRL(0x%08x), VENC_CLK_CG_CTRL(0x%08x)\n",
					jpgenc, venc_cg);

			dcm_info("[ISP_DCM]SMILARB2_DCM_STA(0x%08x)\n", smilarb2_dcm_sta);
			dcm_info("[ISP_DCM]SMILARB2_DCM_CON(0x%08x)\n", smilarb2_dcm_com);
		} else
			dcm_info("[ISP_DCM]subsys ISP is off\n");
	}

	if (type & VDE_DCM) {
		if (subsys_is_on(SYS_VDE)) {
			unsigned int vdec, smilarb1_dcm_sta, smilarb1_dcm_com, smilarb1_dcm_set;
			vdec = dcm_readl(VDEC_DCM_CON);
			smilarb1_dcm_sta = dcm_readl(SMILARB1_DCM_STA);
			smilarb1_dcm_com = dcm_readl(SMILARB1_DCM_CON);
			smilarb1_dcm_set = dcm_readl(SMILARB1_DCM_SET);
			dcm_info("[VDE_DCM]VDEC_DCM_CON(0x%08x), SMILARB1_DCM_STA(0x%08x), SMILARB1_DCM_CON(0x%08x), SMILARB1_DCM_SET(0x%08x)\n",
					vdec, smilarb1_dcm_sta, smilarb1_dcm_com, smilarb1_dcm_set);
		} else
			dcm_info("[VDE_DCM]subsys VDE is off\n");
	}

	mutex_unlock(&dcm_lock);

}

void dcm_enable(unsigned int type)
{
	unsigned int temp;

	dcm_info("[%s]type:0x%08x\n", __func__, type);

	mutex_lock(&dcm_lock);

	if (type & CPU_DCM) {
		dcm_info("[%s][CPU_DCM     ]=0x%08x\n", __func__, CPU_DCM);

		dcm_setl(MCU_BIU_CON, 0x1 << 12);
		dcm_setl(CA7_MISC_CONFIG, 0x1 << 9);
		dcm_sta |= CPU_DCM;

	}


	if (type & TOPCKGEN_DCM) {
		dcm_info("[%s][TOPCKGEN_DCM]=0x%08x\n", __func__, TOPCKGEN_DCM);

#ifdef DCM_ENABLE_DCM_CFG /* AXI bus dcm, don't need to set by KL Tong */
		/* default value are all 0, use default value */
		dcm_writel(DCM_CFG, 0xFFFFFF7F); /* set bit0~bit4=0, bit7=0, bit8~bit14=0, bit15=0???? */
#endif
		dcm_setl(CLK_SCP_CFG_0, 0x3FF); /* set bit0~bit9=1, SCP control register 1 */
		dcm_setl(CLK_SCP_CFG_1, ((0x1 << 4) | 0x1)); /* set bit0=1 and bit4=1, SCP control register 1 */
		dcm_sta |= TOPCKGEN_DCM;

	}

	/* Infrasys_dcm */
	if (type & IFR_DCM) {
		dcm_info("[%s][IFR_DCM     ]=0x%08x\n", __func__, IFR_DCM);

		dcm_clrl(TOP_CKDIV1, 0x0000001f); /* 5'h0, 00xxx: 1/1 */
		dcm_setl(TOP_DCMCTL, 0x00000007); /* set bit0~bit2=1 */
		dcm_setl(TOP_DCMDBC, 0x00000001); /* set bit0=1, force to 26M */
		dcm_setl(INFRA_DCMCTL, 0x00000303); /* set bit0, bit1, bit8, bit9=1, DCM debouncing counter=0 */
		dcm_setl(INFRA_DCMDBC, 0x00000300); /* set bit8, bit9=1 first */
		dcm_clrl(INFRA_DCMDBC, 0x0000007F); /* then clear b0~b6 */

#if 0 /* divided most, save power, */
		dcm_writel(INFRA_DCMFSEL, 0xFFE0F0F8); /* clear bit0~bit2, clear bit8~bit11, clear bit16~bit20 */
#else /* divided by 1 */
		dcm_writel(INFRA_DCMFSEL, 0xFFF0F0F8); /* clear bit0~bit2, clear bit8~bit11, set bit20=1 */
#endif

		dcm_setl(DRAMC_PD_CTRL, 0x3 << 24); /* set bit24, bit25=1 */
		dcm_sta |= IFR_DCM;

	}

	if (type & PER_DCM) {
		dcm_info("[%s][PER_DCM     ]=0x%08x\n", __func__, PER_DCM);

		dcm_clrl(PERI_GLOBALCON_DCMCTL, 0x00001F00); /* clear bit8~bit12=0 */
		dcm_setl(PERI_GLOBALCON_DCMCTL, 0x000000F3); /* set bit0, bit1, bit4~bit7=1 */

		dcm_setl(PERI_GLOBALCON_DCMDBC, 0x1<<7); /* set bit7=1 */
		dcm_clrl(PERI_GLOBALCON_DCMDBC, 0x0000007F); /* clear bit0~bit6=0 */

		dcm_clrl(PERI_GLOBALCON_DCMFSEL, 0x00000007); /* clear bit0~bit2 */
		dcm_clrl(PERI_GLOBALCON_DCMFSEL, 0x00000F00); /* clear bit8~bit11 */
		dcm_clrl(PERI_GLOBALCON_DCMFSEL, 0x001F0000); /* clear bit16~bit20 */

		/* MSDC module */
		dcm_clrl(MSDC0_IP_DCM, 0xFF800000); /* clear bit23~bit31=0 */
		dcm_clrl(MSDC1_IP_DCM, 0xFF800000); /* clear bit23~bit31=0 */
		dcm_clrl(MSDC2_IP_DCM, 0xFF800000); /* clear bit23~bit31=0 */

		/* USB */
		dcm_clrl(PERI_USB0_DCM, 0x00070000); /* clear bit16~bit18=0 */

		/* PMIC */
		dcm_setl(PMIC_WRAP_DCM_EN, 0x1); /* set bit0=1 */

		/* I2C */
		dcm_setl(I2C0_I2CREG_HW_CG_EN, 0x1); /* set bit0=1 */
		dcm_setl(I2C1_I2CREG_HW_CG_EN, 0x1); /* set bit0=1 */
		dcm_setl(I2C2_I2CREG_HW_CG_EN, 0x1); /* set bit0=1 */

		dcm_sta |= PER_DCM;

	}

	if (type & SMI_DCM) {
#if SUPPORT_MTEE_SMI_DCM_PROT
		int iret;
#endif  /* SUPPORT_MTEE_SMI_DCM_PROT */

		dcm_info("[%s][SMI_DCM     ]=0x%08x\n", __func__, SMI_DCM);

#if SUPPORT_MTEE_SMI_DCM_PROT
		/* SMI_SECURE_XXX register is protected by MTEE */
		/* Note: driver initialization should not call this function due to driver iniitializaion sequence */
		iret = i4MTEE_SMI_DCM_Ctrl(1);
#else
		/* smi_common */
		dcm_writel(SMI_DCM_CONTROL, 0x1); /* set bit 0=1 */
		/* RO */
		dcm_readl(SMI_COMMON_AO_SMI_CON);

		dcm_setl(SMI_COMMON_AO_SMI_CON_SET, 0x1 << 2);

		/* NA */
		dcm_readl(SMI_COMMON_AO_SMI_CON_CLR);

		/* m4u_dcm */
		dcm_setl(MMU_DCM, 0x1); /* set bit0=1 */
#endif  /* SUPPORT_MTEE_SMI_DCM_PROT */
		dcm_sta |= SMI_DCM;

	}

	if (type & MFG_DCM) {
		dcm_info("[%s][MFG_DCM     ]=0x%08x, subsys_is_on(SYS_MFG)=%d\n",
			__func__, MFG_DCM, subsys_is_on(SYS_MFG));

		if (subsys_is_on(SYS_MFG)) {
			temp = dcm_readl(MFG_DCM_CON_0);
			temp &= 0xFFFE0000; /* set B[0:6]=0111111, B[8:13]=0,, B[14]=1,, B[15]=1,, B[16]=0 */
			temp |= 0x0000C03F;
			dcm_writel(MFG_DCM_CON_0, temp);
			dcm_sta |= MFG_DCM;
		}
	}

	if (type & DIS_DCM) {
		dcm_info("[%s][DIS_DCM     ]=0x%08x, subsys_is_on(SYS_DIS)=%d\n",
			__func__, DIS_DCM, subsys_is_on(SYS_DIS));

		if (subsys_is_on(SYS_DIS)) {
			dcm_writel(DISP_HW_DCM_DIS0, 0x0);
			dcm_writel(DISP_HW_DCM_DIS_SET0, 0x0);
			dcm_writel(DISP_HW_DCM_DIS_CLR0, 0xFFFFFFFF);

			dcm_writel(DISP_HW_DCM_DIS1, 0x0);
			dcm_writel(DISP_HW_DCM_DIS_SET1, 0x0);
			dcm_writel(DISP_HW_DCM_DIS_CLR1, 0xFFFFFFFF);

			/* LARB0 „³ DISP, MDP */
			/* RO, bootup set once status = 1'b0, DCM off setting=N/A */
			dcm_readl(SMILARB0_DCM_STA);
			/* RO, bootup set once status = 1'b1, DCM off setting=1'b0 */
			dcm_readl(SMILARB0_DCM_CON);
			dcm_setl(SMILARB0_DCM_SET, 0x1<<15); /* set bit15=1 */
			/* N/A */
			dcm_readl(SMILARB0_DCM_CON);

			dcm_sta |= DIS_DCM;
		}

	}

	if (type & ISP_DCM) {

		dcm_info("[%s][ISP_DCM     ]=0x%08x, subsys_is_on(SYS_ISP)=%d\n",
			__func__, ISP_DCM, subsys_is_on(SYS_ISP));

		if (subsys_is_on(SYS_ISP)) {
			dcm_writel(CAM_CTL_RAW_DCM, 0xFFFF8000); /* set bit0~bit14=0 */
			dcm_writel(CAM_CTL_RGB_DCM, 0xFFFFFE00); /* set bit0~bit8=0 */
			dcm_writel(CAM_CTL_YUV_DCM, 0xFFFFFFF0); /* set bit0~bit3=0 */
			dcm_writel(CAM_CTL_CDP_DCM, 0xFFFFFE00); /* set bit0~bit8=0 */
			dcm_writel(CAM_CTL_DMA_DCM, 0xFFFFFFC0); /* set bit0~bit5=0 */

			dcm_clrl(JPGENC_DCM_CTRL, 0x1); /* clear bit0=0 */

			dcm_setl(VENC_CLK_DCM_CTRL, 0x1); /* ok */
			dcm_writel(VENC_CLK_CG_CTRL, 0xFFFFFFFF);

			/* LARB2 „³ ISP, VENC */
			/* RO, bootup set once status = 1'b0, DCM off setting=N/A */
			dcm_readl(SMILARB2_DCM_STA);
			/* RO, bootup set once status = 1'b1, DCM off setting=1'b0 */
			dcm_readl(SMILARB2_DCM_CON);
			dcm_setl(SMILARB2_DCM_SET, 0x1<<15); /* set bit15=1 */
			/* N/A */
			dcm_readl(SMILARB2_DCM_CON);

			dcm_sta |= ISP_DCM;

		}

	}

	if (type & VDE_DCM) {

		dcm_info("[%s][VDE_DCM     ]=0x%08x, subsys_is_on(SYS_VDE)=%d\n",
			__func__, VDE_DCM, subsys_is_on(SYS_VDE));

		if (subsys_is_on(SYS_VDE)) {
			dcm_clrl(VDEC_DCM_CON, 0x1); /* clear bit0 */

			/* LARB1 „³ VDEC */
			/* RO, bootup set once status = 1'b0, DCM off setting=N/A */
			dcm_readl(SMILARB1_DCM_STA);
			/* RO, bootup set once status = 1'b1, DCM off setting=1'b0 */
			dcm_readl(SMILARB1_DCM_CON);
			dcm_setl(SMILARB1_DCM_SET, 0x1<<15); /* set bit15=1 */
			/* N/A */
			dcm_readl(SMILARB1_DCM_SET);

			dcm_sta |= VDE_DCM;
		}

	}

	mutex_unlock(&dcm_lock);

}

void dcm_disable(unsigned int type)
{
	dcm_info("[%s]type:0x%08x\n", __func__, type);

	mutex_lock(&dcm_lock);

	if (type & CPU_DCM) {

		dcm_info("[%s][CPU_DCM     ]=0x%08x\n", __func__, CPU_DCM);
		dcm_clrl(MCU_BIU_CON, 0x1 << 12); /* set bit12=0 */
		dcm_clrl(CA7_MISC_CONFIG, 0x1 << 9); /* set bit9=0 */
		dcm_sta &= ~CPU_DCM;
	}

	if (type & TOPCKGEN_DCM) {

		dcm_info("[%s][TOPCKGEN_DCM]=0x%08x\n", __func__, TOPCKGEN_DCM);
#ifdef DCM_ENABLE_DCM_CFG /* AXI bus dcm, don't need to set by KL Tong */
		/* default value are all 0, use default value */
		dcm_clrl(DCM_CFG, (0x1 << 7)); /* set bit7=0 */
#endif
		dcm_setl(CLK_SCP_CFG_0, 0x3FF); /* set bit0~bit9=1, SCP control register 1 */
		dcm_setl(CLK_SCP_CFG_1, ((0x1 << 4) | 0x1)); /* set bit0=1 and bit4=1, SCP control register 1 */
		dcm_sta &= ~TOPCKGEN_DCM;
	}

	if (type & PER_DCM) {
		dcm_info("[%s][PER_DCM     ]=0x%08x\n", __func__, PER_DCM);

		dcm_clrl(PERI_GLOBALCON_DCMCTL, 0x00001F00); /* clear bit8~bit12=0 */
		dcm_clrl(PERI_GLOBALCON_DCMCTL, 0x000000F3); /* set bit0, bit1, bit4~bit7=0 */

		dcm_setl(PERI_GLOBALCON_DCMDBC, 0x1<<7); /* set bit7=1 */
		dcm_clrl(PERI_GLOBALCON_DCMDBC, 0x0000007F); /* clear bit0~bit6=0 */

		dcm_clrl(PERI_GLOBALCON_DCMFSEL, 0x00000007); /* clear bit0~bit2 */
		dcm_clrl(PERI_GLOBALCON_DCMFSEL, 0x00000F00); /* clear bit8~bit11 */
		dcm_clrl(PERI_GLOBALCON_DCMFSEL, 0x001F0000); /* clear bit16~bit20 */

		/* MSDC module */
		dcm_setl(MSDC0_IP_DCM, 0xFF800000); /* set bit23~bit31=1 */
		dcm_setl(MSDC1_IP_DCM, 0xFF800000); /* set bit23~bit31=1 */
		dcm_setl(MSDC2_IP_DCM, 0xFF800000); /* set bit23~bit31=1 */


		/* USB */
		dcm_setl(PERI_USB0_DCM, 0x00070000); /* set bit16~bit18=1 */

		/* PMIC */
		dcm_clrl(PMIC_WRAP_DCM_EN, 0x1); /* set bit0=0 */

		/* I2C */
		dcm_clrl(I2C0_I2CREG_HW_CG_EN, 0x1); /* set bit0=0 */
		dcm_clrl(I2C1_I2CREG_HW_CG_EN, 0x1); /* set bit0=0 */
		dcm_clrl(I2C2_I2CREG_HW_CG_EN, 0x1); /* set bit0=0 */

		dcm_sta &= ~PER_DCM;
	}

	/* Infrasys_dcm */
	if (type & IFR_DCM) {

		dcm_info("[%s][IFR_DCM     ]=0x%08x\n", __func__, IFR_DCM);
		/* should off DRAMC first than off TOP_DCMCTL */
		dcm_setl(DRAMC_PD_CTRL, 0x1 << 24); /* set bit24=1 */
		dcm_clrl(DRAMC_PD_CTRL, 0x1 << 25); /* set bit25=0 */

		dcm_clrl(TOP_DCMCTL, 0x00000006); /* clear bit1, bit2=0, bit0 doesn't need to clear */

		dcm_clrl(INFRA_DCMCTL, 0x00000303); /* set bit0, bit1, bit8, bit9=1, DCM debouncing counter=0 */

		dcm_sta &= ~IFR_DCM;

	}

	if (type & SMI_DCM) {
#if SUPPORT_MTEE_SMI_DCM_PROT
		int iret;
#endif  /* SUPPORT_MTEE_SMI_DCM_PROT */

		dcm_info("[%s][SMI_DCM     ]=0x%08x\n", __func__, SMI_DCM);

#if SUPPORT_MTEE_SMI_DCM_PROT
		/* SMI_SECURE_XXX register is protected by MTEE */
		iret = i4MTEE_SMI_DCM_Ctrl(0);
#else
		/* smi_common */
		dcm_clrl(SMI_DCM_CONTROL, 0x1); /* set bit0=0 */

		/* RU=read status */
		dcm_readl(SMI_COMMON_AO_SMI_CON);
		/* RU=read status */
		dcm_readl(SMI_COMMON_AO_SMI_CON_SET);
		dcm_setl(SMI_COMMON_AO_SMI_CON_CLR, 0x4); /* set bit2=1 */

		/* m4u_dcm */
		dcm_clrl(MMU_DCM, 0x1); /* set bit0=0 */
#endif /* SUPPORT_MTEE_SMI_DCM_PROT */
		dcm_sta &= ~SMI_DCM;
	}

	if (type & MFG_DCM) {

		dcm_info("[%s][MFG_DCM     ]=0x%08x\n", __func__, MFG_DCM);

		dcm_clrl(MFG_DCM_CON_0, 0x8000); /* disable dcm, clear bit 15 */

		dcm_sta &= ~MFG_DCM;
	}

	if (type & DIS_DCM) {

		dcm_info("[%s][DIS_DCM     ]=0x%08x\n", __func__, DIS_DCM);

		dcm_writel(DISP_HW_DCM_DIS0, 0xFFFFFFFF);
		dcm_writel(DISP_HW_DCM_DIS_SET0, 0xFFFFFFFF);
		dcm_writel(DISP_HW_DCM_DIS_CLR0, 0x00000000);

		dcm_writel(DISP_HW_DCM_DIS1, 0xFFFFFFFF);
		dcm_writel(DISP_HW_DCM_DIS_SET1, 0xFFFFFFFF);
		dcm_writel(DISP_HW_DCM_DIS_CLR1, 0x0);

		/* LARB0 „³ DISP, MDP */
		/* RO, bootup set once status = 1'b0, DCM off setting=N/A */
		dcm_readl(SMILARB0_DCM_STA);
		/* RO, bootup set once status = 1'b1, DCM off setting=1'b0 */
		dcm_readl(SMILARB0_DCM_CON);
		/* N/A */
		dcm_readl(SMILARB0_DCM_SET);
		dcm_setl(SMILARB0_DCM_CLR, (0x1 << 15)); /* set bit15=1 */

		dcm_sta &= ~DIS_DCM;
	}

	if (type & ISP_DCM) {

		dcm_info("[%s][ISP_DCM     ]=0x%08x\n", __func__, ISP_DCM);

		dcm_setl(CAM_CTL_RAW_DCM, 0x00007FFF); /* set bit0~bit14=1 */
		dcm_setl(CAM_CTL_RGB_DCM, 0x000001FF); /* set bit0~bit8=1 */
		dcm_setl(CAM_CTL_YUV_DCM, 0x0000000F); /* set bit0~bit3=1 */
		dcm_setl(CAM_CTL_CDP_DCM, 0x000001FF); /* set bit0~bit8=1 */
		dcm_setl(CAM_CTL_DMA_DCM, 0x0000003F); /* set bit0~bit5=1 */

		dcm_setl(JPGENC_DCM_CTRL, 0x00000001); /* set bit0=1 */

		dcm_writel(VENC_CLK_DCM_CTRL, 0xFFFFFFFE); /* clear bit0 */
		dcm_writel(VENC_CLK_CG_CTRL,  0x00000000); /* clear bit0~bit31 */

		/* LARB2 „³ ISP, VENC */
		/* RO, bootup set once status = 1'b0, DCM off setting=N/A */
		dcm_readl(SMILARB2_DCM_STA);
		/* RO, bootup set once status = 1'b1, DCM off setting=1'b0 */
		dcm_readl(SMILARB2_DCM_CON);
		/* N/A */
		dcm_readl(SMILARB2_DCM_SET);
		dcm_setl(SMILARB2_DCM_CLR, (0x1 << 15)); /* set bit15=1 */

		dcm_sta &= ~ISP_DCM;
	}

	if (type & VDE_DCM) {

		dcm_info("[%s][VDE_DCM     ]=0x%08x\n", __func__, VDE_DCM);

		dcm_setl(VDEC_DCM_CON, 0x1); /* set bit0=1 */

		/* LARB1 „³ VDEC */
		/* RO, bootup set once status = 1'b0, DCM off setting=N/A */
		dcm_readl(SMILARB1_DCM_STA);
		/* RO, bootup set once status = 1'b1, DCM off setting=1'b0 */
		dcm_readl(SMILARB1_DCM_CON);
		/* N/A */
		dcm_readl(SMILARB1_DCM_SET);
		dcm_setl(SMILARB1_DCM_CLR, (0x1 << 15)); /* set bit15=1 */

		dcm_sta &= ~VDE_DCM;
	}

	mutex_unlock(&dcm_lock);
}

void bus_dcm_enable(void)
{
	dcm_writel(DCM_CFG, 0x1 << 7 | 0xF);
}

void bus_dcm_disable(void)
{
	dcm_clrl(DCM_CFG, 0x1 << 7);
}

static unsigned int infra_dcm;

void disable_infra_dcm(void)
{
	infra_dcm = dcm_readl(INFRA_DCMCTL);
	dcm_clrl(INFRA_DCMCTL, 0x100);
}

void restore_infra_dcm(void)
{
	dcm_writel(INFRA_DCMCTL, infra_dcm);
}

static unsigned int peri_dcm;

void disable_peri_dcm(void)
{
	peri_dcm = dcm_readl(PERI_GLOBALCON_DCMCTL);
	dcm_clrl(PERI_GLOBALCON_DCMCTL, 0x1);
}

void restore_peri_dcm(void)
{
	dcm_writel(PERI_GLOBALCON_DCMCTL, peri_dcm);
}

#define dcm_attr(_name)                         \
static struct kobj_attribute _name##_attr = {   \
	.attr = {                                   \
		.name = __stringify(_name),             \
		.mode = 0644,                           \
	},                                          \
	.show = _name##_show,                       \
	.store = _name##_store,                     \
}

static const char *dcm_name[NR_DCMS] = {
	"CPU_DCM",
	"IFR_DCM",
	"PER_DCM",
	"SMI_DCM",
	"MFG_DCM",
	"DIS_DCM",
	"ISP_DCM",
	"VDE_DCM",
	"TOPCKGEN_DCM",
};

static ssize_t dcm_state_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int len = 0;
	char *p = buf;

	int i;
	unsigned int sta;

	p += sprintf(p, "********** dcm_state dump **********\n");
	mutex_lock(&dcm_lock);

	for (i = 0; i < NR_DCMS; i++) {
		sta = dcm_sta & (0x1 << i);
		p += sprintf(p, "[%d][%s]%s\n", i, dcm_name[i], sta ? "on" : "off");
	}

	mutex_unlock(&dcm_lock);

	p += sprintf(p, "\n********** dcm_state help *********\n");
	p += sprintf(p, "enable dcm:    echo enable mask(dec) > /sys/power/dcm_state\n");
	p += sprintf(p, "disable dcm:   echo disable mask(dec) > /sys/power/dcm_state\n");
	p += sprintf(p, "dump reg:      echo dump mask(dec) > /sys/power/dcm_state\n");


	len = p - buf;
	return len;
}

static ssize_t dcm_state_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n)
{
	char cmd[10];
	unsigned int mask;

	if (sscanf(buf, "%s %x", cmd, &mask) == 2) {
		mask &= ALL_DCM;

		/*
		Need to enable MM clock before setting Smi_secure register
		to avoid system crash while screen is off(screen off with USB cable)
		*/
		enable_mux(MT_MUX_MM, "DCM");

		if (!strcmp(cmd, "enable")) {
			dcm_dump_regs(mask);
			dcm_enable(mask);
			dcm_dump_regs(mask);
		} else if (!strcmp(cmd, "disable")) {
			dcm_dump_regs(mask);
			dcm_disable(mask);
			dcm_dump_regs(mask);
		} else if (!strcmp(cmd, "dump"))
			dcm_dump_regs(mask);

		disable_mux(MT_MUX_MM, "DCM");

		return n;
	}

	return -EINVAL;
}
dcm_attr(dcm_state);


void mt_dcm_init(void)
{
	int err = 0;

	dcm_info("[%s]entry!!, ALL_DCM=%d\n", __func__, ALL_DCM);
#if SUPPORT_MTEE_SMI_DCM_PROT
	/*
	Note:
	 1. SMI_SECURE_XXX register is protected by MTEE
	    SMI_DCM is enabled by DCM driver in MTEE
	 2. Although initialization sequence for DCM kernel driver and MTEE driver is not guarantee in kernel,
	    it is ok to make status of SMI_DCM to be set to "enable" in DCM kernel driver initialization
	*/
	dcm_enable(ALL_DCM & (~SMI_DCM));
	dcm_sta |= SMI_DCM;
#else
	dcm_enable(ALL_DCM);
#endif

	err = sysfs_create_file(power_kobj, &dcm_state_attr.attr);

	if (err)
		dcm_err("[%s]: fail to create sysfs\n", __func__);
}
