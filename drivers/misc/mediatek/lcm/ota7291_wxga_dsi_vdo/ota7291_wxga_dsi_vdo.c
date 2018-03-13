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

#ifdef BUILD_LK
#define TPS_I2C_BUS 0
#define TPS_I2C_ID I2C2
#define TPS_SLAVE_ADDR 0x74
#define TPS_data_size 13 //1024
const uint8_t e2prom_data[TPS_data_size] = {0xFA,0x3C,0x28,0x09,0x09,0x08,0x23,0x03,0x01,0x06,0x01,0x7D,0x7D};
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  (800)
#define FRAME_HEIGHT (1280)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

#define LCM_ID 0x01  //KDT+AUO+OTA7291
#define LCM_DSI_CMD_MODE    0

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
	{0xd8,1,{0x66}},
	{0x19,18,{0x00,0x00,0xfc,0x51,0xf2,0x3f,0xff,0xf3,0x3f,0x91,0x00,0x00,0x00,0x1c,0xf8,0x1f,0x01,0xc3}},
	{0x19,18,{0x60,0x00,0x64,0xa0,0x10,0x0a,0x1e,0x1c,0x36,0x9c,0x10,0x09,0x3a,0x1e,0x80,0x91,0x02,0x3c}},
};


#ifdef BUILD_LK
/*----------------------------------------*/
/*--------------TPS65640------------------*/ 
static int tps65640_write_data(void)
{
	printf("Evan:%s\n",__func__);
	int ret=0;
	mt_i2c i2c = {0};
    i2c.id = TPS_I2C_ID;
    i2c.addr = TPS_SLAVE_ADDR;
    i2c.mode = ST_MODE;
    i2c.speed = 100;
    i2c.dma_en = 0;

	u8 buf1[TPS_data_size+1];
	buf1[0] = 0x00;
	int i=0;
	for(i=0;i<TPS_data_size;i++)
		buf1[i+1]=e2prom_data[i];
	ret = i2c_write(&i2c, buf1, TPS_data_size+1); 
	if(!ret)
		{
			printf("--------Evan:%s step1 error---------\n",__func__);
			return ret;
		}
	
	u8 buf2[2];
	buf2[0]=0xFF;
	buf2[1]=0x80;
	ret = i2c_write(&i2c, buf2, 2);
	if(!ret)
		{
			printf("--------Evan:%s step2 error---------\n",__func__);
			return ret;
		}
	
	return ret;
}

static int tps65640_read_data(uint8_t *data)
{
	printf("Evan:%s\n",__func__);
	int ret=0;
	mt_i2c i2c = {0};
    i2c.id = TPS_I2C_ID;
    i2c.addr = TPS_SLAVE_ADDR;
    i2c.mode = ST_MODE;
    i2c.speed = 100;
    i2c.dma_en = 0;

	u8 buf1[2];
	buf1[0]=0xFF;
	buf1[1]=0x01;
	ret = i2c_write(&i2c, buf1, 2);
	if(!ret)
		{
			printf("--------Evan:%s step1 error---------\n",__func__);
			return ret;
		}

	uint8_t buf2=0x00;
	ret = i2c_write(&i2c, buf2, 1);
	if(!ret)
		{
			printf("--------Evan:%s step2-1 error---------\n",__func__);
			return ret;
		}

	ret=i2c_read(&i2c, data, TPS_data_size);
	if(!ret)
		{
			printf("--------Evan:%s step2-2 error---------\n",__func__);
			return ret;
		}
	return ret;
		
}



static int tps65640_e2prom_data_check(void)
{
	printf("Evan:%s\n",__func__);
	int i=0,ret=1;
	uint8_t data[TPS_data_size];
	//read e2prom data
	tps65640_read_data(data);
	//check e2prom data
	for(i=0;i<TPS_data_size;i++)
		if(data[i]!=e2prom_data[i])
			{
				ret=0;
				printf("---------Evan:e2prom_data need update--------\n");
				break;
			}
	if(ret)
		printf("---------Evan:e2prom_data is OK--------\n");
	else
		ret=tps65640_write_data();
	return ret; 
}

static int tps65640_enable()
{
	printf("Evan:%s\n",__func__);
	//todo:IC power on, 
}

/*----------------------------------------*/   
#endif

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

    params->dsi.PLL_CLOCK = 260;//148;
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
    MDELAY(50);
    lcd_reset(1);
	MDELAY(20);
	lcd_reset(0);
	MDELAY(20);
	lcd_reset(1);
    MDELAY(20);
	
    mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE);//iml8881 bias
    MDELAY(50);
#ifdef BUILD_LK
	//extern int tps65640_e2prom_data_check(void);
	//tps65640_e2prom_data_check();	
#endif

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
	
	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ZERO);//iml8881 bias
	MDELAY(10);
    lcd_reset(0);
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
    printf("KDT+AUO id1=%d,id2=%d,id=0x%x\n",id1,id2,id);
#else
    printk("KDT+AUO id1=%d,id2=%d,id=0x%x\n",id1,id2,id);
#endif
	lcd_reset(0);
    lcd_power_en(0);
    MDELAY(10);
    return (LCM_ID == id)?1:0;
}

LCM_DRIVER ota7291_wxga_dsi_vdo_lcm_drv = 
{
    .name           = "ota7291_wxga_dsi_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id    	= lcm_compare_id,
};


