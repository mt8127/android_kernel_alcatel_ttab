#ifndef BUILD_LK
	#include <linux/string.h>
#endif
	#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <platform/upmu_common.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
	#include <mach/mt_pm_ldo.h>
	#include <mach/upmu_common.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(1024)
#define FRAME_HEIGHT 										(600)

#define REGFLAG_DELAY             		0XFE
#define REGFLAG_END_OF_TABLE      	0xFFF   // END OF REGISTERS MARKER

#define LCM_ID_79007        			0x00
#define LCM_DSI_CMD_MODE			0

#define GPIO_RESET_PIN		GPIO83
#define GPIO_STBYB_PIN 		GPIO117
#define GPIO_AVDD_PIN 		GPIO119
#define GPIO_LCM_PWR_EN		GPIO89
#ifndef TRUE
    #define   TRUE     1
#endif

#ifndef FALSE
    #define   FALSE    0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 										(lcm_util.udelay(n))
#define MDELAY(n) 										(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif



static void lcm_gpio_set(int num, int enable)
{
#ifdef BUILD_LK
printf("\n\n\n===================== %s %d: num:%d => %d =======================\n\n\n", __func__, __LINE__, num, enable);
#endif
	lcm_util.set_gpio_mode(num, GPIO_MODE_00);
	lcm_util.set_gpio_dir(num, GPIO_DIR_OUT);

	if (enable)
		lcm_util.set_gpio_out(num, GPIO_OUT_ONE);
	else
		lcm_util.set_gpio_out(num, GPIO_OUT_ZERO);
}

static void lcd_reset(unsigned char enabled)
{
	 if (enabled)
	 {
		  mt_set_gpio_mode(GPIO_RESET_PIN, GPIO_MODE_00);
		  mt_set_gpio_dir(GPIO_RESET_PIN, GPIO_DIR_OUT);
		  mt_set_gpio_out(GPIO_RESET_PIN, GPIO_OUT_ONE);
	 }
	 else
	 {
		  mt_set_gpio_mode(GPIO_RESET_PIN, GPIO_MODE_00);
		  mt_set_gpio_dir(GPIO_RESET_PIN, GPIO_DIR_OUT);
		  mt_set_gpio_out(GPIO_RESET_PIN, GPIO_OUT_ZERO);
	 }
}

static void lcd_stbyb(unsigned char enabled)
{
	 if (enabled)
	 {
		  mt_set_gpio_mode(GPIO_STBYB_PIN, GPIO_MODE_00);
		  mt_set_gpio_dir(GPIO_STBYB_PIN, GPIO_DIR_OUT);
		  mt_set_gpio_out(GPIO_STBYB_PIN, GPIO_OUT_ONE);
	 }
	 else
	 {
		  mt_set_gpio_mode(GPIO_STBYB_PIN, GPIO_MODE_00);
		  mt_set_gpio_dir(GPIO_STBYB_PIN, GPIO_DIR_OUT);
		  mt_set_gpio_out(GPIO_STBYB_PIN, GPIO_OUT_ZERO);
	 }
}

static void lcd_DVDD_power_en(unsigned char enabled)
{
	if (enabled)
	{
		upmu_set_rg_vgp1_vosel(0x3);     //1.8V   
		upmu_set_rg_vgp1_en(0x1);
	}
	else
	{
		upmu_set_rg_vgp1_en(0x0);
	}
}
static void lcd_AVDD_power_en(unsigned char enabled)
{
	//lcm_gpio_set(GPIO_LCM_PWR_EN, enabled);
		  mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
		  mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
		  mt_set_gpio_out(GPIO_LCM_PWR_EN, enabled);
}


static void init_lcm_registers(void)
{    
	unsigned int data_array[16];    
#ifdef BUILD_LK    
	//printf("[IND][K] y_____0%s\n", __func__);
#else    
	//printk("[IND][K] y_____1%s\n", __func__);
#endif
	//[BUGFIX]-Mod-BEGIN by SCDTABLET.jinghuang@tcl.com,05/21/2015,1008656£¬
	//import gamma2.2 V1 parameters.
	data_array[0] = 0x77801500;	
	dsi_set_cmdq(&data_array, 1, 1);	
	MDELAY(1);	
	data_array[0] = 0x77811500;	
	dsi_set_cmdq(&data_array, 1, 1);	
	MDELAY(1);	
	data_array[0] = 0xa8821500;	
	dsi_set_cmdq(&data_array, 1, 1);	
	MDELAY(1);	
	data_array[0] = 0xfe831500;	
	dsi_set_cmdq(&data_array, 1, 1);	
	MDELAY(1);	
	data_array[0] = 0xb0841500;	
	dsi_set_cmdq(&data_array, 1, 1);	
	MDELAY(1);	
	data_array[0] = 0x77851500;	
	dsi_set_cmdq(&data_array, 1, 1);	
	MDELAY(1);	
	data_array[0] = 0x77861500;	
	dsi_set_cmdq(&data_array, 1, 1);	
	MDELAY(1);
	//[BUGFIX]-Mod-END by SCDTABLET.jinghuang@tcl.com

	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	//[BUGFIX]-Mod-BEGIN by SCDTABLET.jinghuang@tcl.com,28/4/2015,
	//improve dsi config and power on/off sequence
		memset(params, 0, sizeof(LCM_PARAMS));

		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
#endif

		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;

		// Video mode setting
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 0x05;// 3    2
		params->dsi.vertical_backporch					= 23;// 20	 1
		params->dsi.vertical_frontporch 				= 12; // 1	12
		params->dsi.vertical_active_line				= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active				= 0x16;// 50  2
		params->dsi.horizontal_backporch				= 160;
		params->dsi.horizontal_frontporch				= 160;
		params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;
		params->dsi.PLL_CLOCK=175;
		params->dsi.ssc_disable = TRUE;
		//[BUGFIX]-Mod-END by SCDTABLET.jinghuang@tcl.com

		}



static void lcm_init(void)
{
	//[BUGFIX]-Mod-BEGIN by SCDTABLET.jinghuang@tcl.com,28/4/2015,
	//improve dsi config and power on/off sequence

	lcd_DVDD_power_en(0);
	lcd_stbyb(0);
	lcd_reset(0);
	lcd_AVDD_power_en(0);
	MDELAY(30);

	lcd_DVDD_power_en(1);
	MDELAY(10);
	
	lcd_stbyb(1);
	MDELAY(10);
	
	lcd_reset(1);
	MDELAY(20);
	lcd_reset(0);
	MDELAY(10);
	lcd_reset(1);
	
	lcd_AVDD_power_en(1);

	init_lcm_registers();
	MDELAY(50);
	//[BUGFIX]-Mod-END by SCDTABLET.jinghuang@tcl.com

}


static void lcm_suspend(void)
{
	//[BUGFIX]-Mod-BEGIN by SCDTABLET.jinghuang@tcl.com,28/4/2015,
	//improve dsi config and power on/off sequence
	lcd_stbyb(0);
	MDELAY(80);
	
	lcd_AVDD_power_en(0);  
	MDELAY(50);
	
	lcd_DVDD_power_en(0); 
	MDELAY(10);
	
	lcd_reset(0);
	//[BUGFIX]-Mod-END by SCDTABLET.jinghuang@tcl.com
}


static void lcm_resume(void)
{
	lcm_init();
}

static unsigned int lcm_compare_id(void)
{
	return 1; 
}

static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];
	int ret = 0;
	
	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	array[0] = 0x5A871500;
	dsi_set_cmdq(array, 1, 1);

	array[0] = 0x125B1500;
	dsi_set_cmdq(array, 1, 1);

	array[0] = 0x55871500;
	dsi_set_cmdq(array, 1, 1);

	array[0] = 0x125B1500;
	dsi_set_cmdq(array, 1, 1);

	//array[0] = 0x08B11500;
	//dsi_set_cmdq(array, 1, 1);
	
	//MDELAY(2000);
	
	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x81, buffer, 1);
	//printk("\n ZERO] [0x81]=0x%02x\n", buffer[0]);

	if(buffer[0] != 0x77)
	{
		printk("[LCM ERROR] [0x81]=0x%02x\n", buffer[0]);
		return TRUE;
	}
	else
	{
		printk("[LCM ESD OK] [0x81]=0x%02x\n", buffer[0]);
		array[0] = 0x925B1500;
		dsi_set_cmdq(array, 1, 1);

		array[0] = 0x5A871500;
		dsi_set_cmdq(array, 1, 1);

		array[0] = 0x925B1500;
		dsi_set_cmdq(array, 1, 1);

		array[0] = 0x00871500;
		dsi_set_cmdq(array, 1, 1);
		return FALSE;
	}

	// return TRUE: need recovery
	// return FALSE: No need recovery
#else
	return FALSE;
 #endif
}
static unsigned int lcm_esd_recover(void)
{
 #ifndef BUILD_LK
	printk("\n [LCM ERROR] lcm_esd_recover \n");
 #endif
 	lcd_DVDD_power_en(0);   
 	MDELAY(30);
	lcm_init();
	return TRUE;
}

LCM_DRIVER kr076_ek79007_wsvga_dsi_vdo_lcm_drv =
{
       .name			= "kr076_ek79007_wsvga_dsi_vdo",
	.set_util_funcs 	= lcm_set_util_funcs,
	.get_params     	= lcm_get_params,
	.init           		= lcm_init,
	.suspend        		= lcm_suspend,
	.resume         		= lcm_resume,
	.compare_id    	= lcm_compare_id,
	.esd_check   		= lcm_esd_check,
	.esd_recover		= lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
	.set_backlight		= lcm_setbacklight,
       .update         		= lcm_update,
#endif
};




