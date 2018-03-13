/******************************************************************************
 * mt6575_vibrator.c - MT6575 Android Linux Vibrator Device Driver
 * 
 * Copyright 2009-2010 MediaTek Co.,Ltd.
 * 
 * DESCRIPTION:
 *     This file provid the other drivers vibrator relative functions
 *
 ******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <mach/mt_typedefs.h>
#include <cust_vibrator.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>
#include <linux/delay.h>

#ifdef CONFIG_MTK_PMIC_MT6397
static int vibe_mode = 0;
#endif
extern S32 pwrap_read( U32  adr, U32 *rdata );
extern S32 pwrap_write( U32  adr, U32  wdata );
#if 0
// pmic wrap read and write func
static unsigned int vibr_pmic_pwrap_read(U32 addr)
{

	U32 val =0;
	pwrap_read(addr, &val);
	return val;
	
}

static void vibr_pmic_pwrap_write(unsigned int addr, unsigned int wdata)

{
	//unsigned int val =0;
    pwrap_write(addr, wdata);
}
#endif
extern void dct_pmic_VIBR_enable(kal_bool dctEnable);

void vibr_Enable_HW(void)
{
	
	
        printk("[vibrator]vibr_Enable \n");

#ifdef CONFIG_MTK_PMIC_MT6397	
		
	/*
	if(hwPowerOn(MT65XX_POWER_LDO_VIBR, VOL_2800, "VIBR")) {
		ldo_state=1;
	}
	*/
	//dct_pmic_VIBR_enable(1);

	upmu_set_rg_vibr_sw_mode(1);// [bit 5]: VIBR_SW_MODE   0=HW, 1=SW
	upmu_set_rg_vibr_vosel(7);  // [bit 11-9]: VIBR_SEL,  101=2.8V, 110=3.0V, 111=3.3V
	upmu_set_rg_vibr_fr_ori(1);  //[bit 4-3]: VIBR_FR_ORI,  00=float, 01=forward, 10=braking, 11=backward
	//upmu_set_rg_vibr_mst_time();    //[bit 7-6]: VIBR_MST_TIME,  00=1us, 01=2us, 10=4us, 11=8us

	upmu_set_rg_vibr_en(1);     //[bit 15]: VIBR_EN,  1=enable
	upmu_set_rg_vibr_pwdb(1);    //[bit 6]: VIBR_PWDB,   1=enable

#else
        dct_pmic_VIBR_enable(1);
#endif

        return;

}

void vibr_Disable_HW(void)
{
	printk("[vibrator]vibr_Disable \n");

#ifdef CONFIG_MTK_PMIC_MT6397

	/*
	if(hwPowerDown(MT65XX_POWER_LDO_VIBR, "VIBR")) {
		ldo_state=0;
	}
	*/
	//dct_pmic_VIBR_enable(0);
	switch(vibe_mode)
        {
        case 1:
            upmu_set_rg_vibr_fr_ori(2);  //[bit 4-3]: VIBR_FR_ORI,  00=float, 01=forward, 10=braking, 11=backward
            upmu_set_rg_vibr_fr_ori(3);  //[bit 4-3]: VIBR_FR_ORI,  00=float, 01=forward, 10=braking, 11=backward

            msleep(30); //delay 30ms
            upmu_set_rg_vibr_fr_ori(2);  //[bit 4-3]: VIBR_FR_ORI,  00=float, 01=forward, 10=braking, 11=backward
            upmu_set_rg_vibr_en(0);     //[bit 15]: VIBR_EN,  1=enable
            upmu_set_rg_vibr_pwdb(0);    //[bit 6]: VIBR_PWDB,   1=enable
        case 0:
        default:
            upmu_set_rg_vibr_en(0);     //[bit 15]: VIBR_EN,  1=enable
            upmu_set_rg_vibr_pwdb(0);    //[bit 6]: VIBR_PWDB,   1=enable
            break;
       }
       vibe_mode = 0;
#else
       dct_pmic_VIBR_enable(0);
#endif

       return;
}

void vibr_power_set(void)
{
#ifdef CUST_VIBR_VOL
	struct vibrator_hw* hw = get_cust_vibrator_hw();	
	printk("[vibrator]vibr_init: vibrator set voltage = %d\n", hw->vib_vol);
	upmu_set_rg_vibr_vosel(hw->vib_vol);
#endif
}

struct vibrator_hw* mt_get_cust_vibrator_hw(void)
{
	return get_cust_vibrator_hw();
}
