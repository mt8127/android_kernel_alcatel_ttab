#ifdef BUILD_LK
#include <string.h>
//Evan add
#include <debug.h>
#include <sys/types.h>
#include <platform/mt_i2c.h>
//Evan add end
#else
#include <linux/string.h>
#endif

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/upmu_common.h>
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  (800)
#define FRAME_HEIGHT (1280)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE    0
#define LCM_ID 0x03  //DPT+AUO+Rm72013

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util = {
    .set_gpio_out = NULL,
};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0x50,1,{0x77}},
	{0xE1,1,{0x66}},
	{0xDC,1,{0x67}},
	{0xD3,1,{0xC8}},
	{0x50,1,{0x00}},
	{0xF0,2,{0x5A,0x5A}},
	{0xF5,1,{0x96}},
	{0xC3,3,{0x40,0x00,0x28}},
	{REGFLAG_DELAY, 6, {}},
	{0x11,1,{}},
	{0x29,1,{}},
	{REGFLAG_DELAY, 200, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_power_off_setting[] = {
	{0x28,1,{}},
	{0x10,1,{}},
	{REGFLAG_DELAY, 5, {}},
	{0xF0,2,{0x5A,0x5A}},
	{0xC3,3,{0x40,0x00,0x20}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcd_power_en(unsigned char enabled)
{
    if (enabled) {      
#ifdef BUILD_LK
		upmu_set_rg_vgp1_vosel(0x7);//3.3V
		upmu_set_rg_vgp1_en(0x1);
		upmu_set_rg_vgp3_vosel(0x3);//1.8V
		upmu_set_rg_vgp3_en(0x1);
#else
		upmu_set_rg_vgp1_vosel(0x7);//3.3V
		upmu_set_rg_vgp1_en(0x1);
		upmu_set_rg_vgp3_vosel(0x3);//1.8V
		upmu_set_rg_vgp3_en(0x1);
#endif
    } else {      
#ifdef BUILD_LK
		upmu_set_rg_vgp1_en(0);
		upmu_set_rg_vgp1_vosel(0);
		upmu_set_rg_vgp3_en(0);
		upmu_set_rg_vgp3_vosel(0);
#else
		upmu_set_rg_vgp1_en(0);
		upmu_set_rg_vgp1_vosel(0);
		upmu_set_rg_vgp3_en(0);
		upmu_set_rg_vgp3_vosel(0);
#endif
    }
}


static void lcd_reset(unsigned char enabled)
{
    if (enabled) {
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
    } else {	
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);    	
    }
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
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type     = LCM_TYPE_DSI;
    params->width    = FRAME_WIDTH;
    params->height   = FRAME_HEIGHT;
    params->dsi.mode = SYNC_EVENT_VDO_MODE; //BURST_VDO_MODE;BURST_VDO_MODE;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;  //LCM_DSI_FORMAT_RGB666;

    // Video mode setting		
    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;  //LCM_PACKED_PS_18BIT_RGB666;

    params->dsi.vertical_sync_active				= 4;//0;
    params->dsi.vertical_backporch					= 8;//23;
    params->dsi.vertical_frontporch 				= 8;
    params->dsi.vertical_active_line				= FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active				= 4;//0;
    params->dsi.horizontal_backporch				= 132;//160;
    params->dsi.horizontal_frontporch				= 24; //100;//160;
    params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;

    params->dsi.PLL_CLOCK = 250;//148;
}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}

static void lcm_init(void)
{
    unsigned int data_array[16];
	
#ifdef BUILD_LK
    printf("%s, LK \n", __func__);
#else
    printk("%s, kernel", __func__);
#endif

    lcd_reset(0);
    lcd_power_en(0);
    lcd_power_en(1);
    MDELAY(5);
	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE);//iml8881 bias
	MDELAY(50);
    lcd_reset(1);
	MDELAY(15);
	lcd_reset(0);
	MDELAY(15);
	lcd_reset(1);
    MDELAY(15);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
    unsigned int data_array[16];

#ifdef BUILD_LK
    printf("%s, LK \n", __func__);
#else
    printk("%s, kernel", __func__);
#endif

	push_table(lcm_power_off_setting, sizeof(lcm_power_off_setting) / sizeof(struct LCM_setting_table), 1);

    lcd_reset(0);
	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ZERO);//iml8881 bias
    lcd_power_en(0);
    MDELAY(10);
}


static void lcm_resume(void)
{
    unsigned int data_array[16];

#ifdef BUILD_LK
    printf("%s, LK \n", __func__);
#else
    printk("%s, kernel", __func__);
#endif
	
	lcm_init();
}
/*
static unsigned int lcm_esd_test = FALSE;

static unsigned int lcm_esd_check(void)
{

  #ifndef BUILD_LK

	char  buffer[1];
	int   array[4];
	int ret = 0;

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x0A, buffer, 1);
	//printk("[LCM ERROR] [0x0A]=0x%02x\n", buffer[0]);
	if(buffer[0] != 0x9C)
	{
	//	printk("[LCM ERROR] [0x0A]=0x%02x\n", buffer[0]);
		ret++;
	}

	// return TRUE: need recovery
	// return FALSE: No need recovery
	if(ret)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
 #endif
}

static unsigned int lcm_esd_recover(void)
{
#ifdef BUILD_LK
    printf("%s, LK \n", __func__);
#else
    printk("%s, KERNEL \n", __func__);
#endif
	lcm_init();
	return TRUE;
}
*/
static unsigned int lcm_compare_id(void)
{
    unsigned int id1 = 0, id2 = 0, id = 0;

#ifdef BUILD_LK
	printf("%s, LK \n", __func__);
#endif
    lcd_reset(0);
    lcd_power_en(0);
    lcd_power_en(1);
    lcd_reset(1);
    MDELAY(20);
	lcd_reset(0);
	MDELAY(20);
	lcd_reset(1);
	MDELAY(20);

	mt_set_gpio_mode(GPIO_HALL_2_PIN, GPIO_HALL_2_PIN_M_GPIO);	
	mt_set_gpio_dir(GPIO_HALL_2_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_HALL_2_PIN, GPIO_PULL_DISABLE);
	mt_set_gpio_mode(GPIO_HALL_1_PIN, GPIO_HALL_1_PIN_M_GPIO);	
	mt_set_gpio_dir(GPIO_HALL_1_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_HALL_1_PIN, GPIO_PULL_DISABLE);
	MDELAY(2);

	id1 = mt_get_gpio_in(GPIO_HALL_1_PIN);
    id2 = mt_get_gpio_in(GPIO_HALL_2_PIN);
	id = (id1<<1)|(id2);
#ifdef BUILD_LK
    printf("DPT+AUO id1=%d,id2=%d,id=0x%x\n",id1,id2,id);
#else
    printk("DPT+AUO id1=%d,id2=%d,id=0x%x\n",id1,id2,id);
#endif
	lcd_reset(0);
    lcd_power_en(0);
    MDELAY(10);
    return (LCM_ID == id)?1:0;
}

LCM_DRIVER rm72013_wxga_dsi_vdo_lcm_drv = 
{
    .name           = "rm72013_wxga_dsi_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
	//.esd_check		= lcm_esd_check,
	//.esd_recover	= lcm_esd_recover,
    .compare_id    = lcm_compare_id,
};


