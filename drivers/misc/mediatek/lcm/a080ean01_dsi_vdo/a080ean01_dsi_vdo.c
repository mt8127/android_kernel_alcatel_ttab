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
#include "a080ean01_dsi_vdo.h"
#ifdef BUILD_LK
//#define TPS_I2C_BUS 2  //0
#define TPS_I2C_ID I2C2
#define TPS_SLAVE_ADDR 0x74
#define TPS_data_size 13 //1024
static const uint8_t e2prom_data[TPS_data_size] = {0xFA,0x3c,0x08,0x09,0x09,0x08,0x23,0x0a,0x01,0x06,0x01,0x7D,0x7D};
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  (800) //(1024)
#define FRAME_HEIGHT (1280) //(600)

#define LCM_ID 0x00    //Trust+AUO
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

#ifdef BUILD_LK
/*----------------------------------------*/
/*--------------TPS65640------------------*/
static int tps65640_read_id(void)
{
	printf("Evan:%s\n",__func__);
	int ret=0;
	U8 id=0;
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
	if(ret)
		{
			printf("--------Evan:%s step1 error---------\n",__func__);
			//return ret;
		}


	
	uint8_t buf2=0x07;
	ret = i2c_write(&i2c, &buf2, 1);
	if(ret)
		{
			printf("--------Evan:%s step2-1 error---------\n",__func__);
			//return ret;
		}
	//uint8_t buf3[TPS_data_size];
	ret=i2c_read(&i2c, &id, 1);
	if(ret)
		{
			printf("--------Evan:%s step2-2 error---------\n",__func__);
			//return ret;
		}
	printf("-------Evan:id=%d--------",id);
	return ret;
}

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

	/*write  step1*/
	u8 buf1[8];
	u8 buf11[8];
	buf1[0] = 0x00;
	buf11[0]=0x07;
	//lens=TPS_data_size+1;
	int i=0;
	for(i=0;i<7;i++)  //TPS_data_size
		buf1[i+1]=e2prom_data[i];
	for(i=0;i<6;i++)  //TPS_data_size
		buf11[i+1]=e2prom_data[i+7];
		
	ret = i2c_write(&i2c, buf1, 8); //TPS_data_size+1
	if(ret)
		{
			printf("--------Evan:%s step1 error---------\n",__func__);
			return ret;
		}
	
	ret = i2c_write(&i2c, buf11, 7); //TPS_data_size+1
	if(ret)
		{
			printf("--------Evan:%s step1 error---------\n",__func__);
			return ret;
		}
	
	/*write  step2*/
	u8 buf2[2];
	buf2[0]=0xFF;
	buf2[1]=0x80;
	ret = i2c_write(&i2c, buf2, 2);
	if(ret)
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
	
	/*read step1*/
	u8 buf1[2];
	buf1[0]=0xFF;
	buf1[1]=0x01;
	ret = i2c_write(&i2c, buf1, 2);
	if(ret)
		{
			printf("--------Evan:%s step1 error---------\n",__func__);
			return ret;
		}
	
	/*read step2*/
	uint8_t buf2=0x00;
	ret = i2c_write(&i2c, &buf2, 1);
	if(ret)
		{
			printf("--------Evan:%s step2-1 error---------\n",__func__);
			return ret;
		}
	//uint8_t buf3[TPS_data_size];
	ret=i2c_read(&i2c, data, 8);//TPS_data_size
	if(ret)
		{
			printf("--------Evan:%s step2-2 error---------\n",__func__);
			return ret;
		}

	/*read step2*/
	buf2=0x08;
	ret = i2c_write(&i2c, &buf2, 1);
	if(ret)
		{
			printf("--------Evan:%s step2-2-1 error---------\n",__func__);
			return ret;
		}
	//uint8_t buf3[TPS_data_size];
	ret=i2c_read(&i2c, &data[8], (TPS_data_size-8));//TPS_data_size
	if(ret)
		{
			printf("--------Evan:%s step2-2-2 error---------\n",__func__);
			return ret;
		}

	
	int i=0;
	for(i=0;i<TPS_data_size;i++)
			printf("------------Evan:read data%d : 0x%x --------\n",i,data[i]);
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
		{
			ret=tps65640_write_data();
			MDELAY(100);
		}
	
	tps65640_read_data(data);
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
    if (enabled)
    {
        mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
    }
    else
    {	
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

    params->dsi.vertical_sync_active				= 4;//10;//0;
    params->dsi.vertical_backporch				= 8;//13;//23;
    params->dsi.vertical_frontporch 				= 8;//12;
    params->dsi.vertical_active_line				= FRAME_HEIGHT; 

    params->dsi.horizontal_sync_active				= 4;//120;//0;
    params->dsi.horizontal_backporch				= 132;//100;//160;
    params->dsi.horizontal_frontporch				= 24;//100;//160;
    params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;

    params->dsi.PLL_CLOCK = 250;//148;
}


static void init_lcm_registers(void)
{
#ifdef BUILD_LK
    printf("%s, LK \n", __func__);
#else
    printk("%s, kernel", __func__);
#endif
}

//extern void tps65640_en(void);

static void extra_cmd(void)
{
	unsigned int data_array[16];
	//--------------LCM tmp start------------//
		
		data_array[0] = 0x00033902;
		data_array[1] = 0x005A5AF1;
		dsi_set_cmdq(data_array, 2, 1);
		MDELAY(30);
	
		data_array[0] = 0x00033902;
		data_array[1] = 0x00A5A5FC;
		dsi_set_cmdq(data_array, 2, 1);
		MDELAY(30);
	
		data_array[0] = 0x00033902;
		data_array[1] = 0x001000D0;
		dsi_set_cmdq(data_array, 2, 1);
		MDELAY(30);
	
		data_array[0] = 0x00043902;
		data_array[1] = 0xA04E01BC;
		dsi_set_cmdq(data_array, 2, 1);
		MDELAY(30);
	
		data_array[0] = 0x00063902;
		data_array[1] = 0x1C1003E1;
		data_array[2] = 0x00000782;
		dsi_set_cmdq(data_array, 3, 1);
		MDELAY(30);
	
		data_array[0] = 0x10B11500;
		dsi_set_cmdq(data_array, 1, 1);
		MDELAY(30);
	
		data_array[0] = 0x00053902;
		data_array[1] = 0x2F2214B2;
		data_array[2] = 0x00000004;
		dsi_set_cmdq(data_array, 3, 1);
		MDELAY(30);
	
		data_array[0] = 0x00063902;
		data_array[1] = 0x080C02F2;
		data_array[2] = 0x00001888;
		dsi_set_cmdq(data_array, 3, 1);
		MDELAY(30);
	
		data_array[0] = 0x1EB51500;
		dsi_set_cmdq(data_array, 1, 1);
		MDELAY(30);
	
		data_array[0] = 0x04B01500;
		dsi_set_cmdq(data_array, 1, 1);
		MDELAY(30);
	
		data_array[0] = 0x09FD1500;
		dsi_set_cmdq(data_array, 1, 1);
		MDELAY(30);
	
		data_array[0] = 0x00073902;
		data_array[1] = 0x862163F6;
		data_array[2] = 0x00000000;
		dsi_set_cmdq(data_array, 3, 1);
		MDELAY(30);
	
		data_array[0] = 0x00043902;
		data_array[1] = 0x104C5ED8;
		dsi_set_cmdq(data_array, 2, 1);
		MDELAY(30);
		//
		data_array[0] = 0x000C3902;
		data_array[1] = 0xE0C001F3;
		data_array[2] = 0x3581D062;
		data_array[3] = 0x002430F3;
		dsi_set_cmdq(data_array, 4, 1);
		MDELAY(30);
		
		data_array[0] = 0x002E3902;
		data_array[1] = 0x030200F4;
		data_array[2] = 0x09020326;
		data_array[3] = 0x16160700;
		data_array[4] = 0x08080003;
		data_array[5] = 0x12000003;
		data_array[6] = 0x011E1D1C;
		data_array[7] = 0x02040109;
		data_array[8] = 0x72757461;
		data_array[9] = 0x00808083;
		data_array[10] = 0x28010100;
		data_array[11] = 0x01280304;
		data_array[12] = 0x000032D1;
		dsi_set_cmdq(data_array, 13, 1);
		MDELAY(30);
	
		//
		data_array[0] = 0x001B3902;
		data_array[1] = 0x42429DF5;
		data_array[2] = 0x4F98AB5F;
		data_array[3] = 0x0443330F;
		data_array[4] = 0x05525459;
		data_array[5] = 0x60406040;
		data_array[6] = 0x52262740;
		data_array[7] = 0x00186D25;
		dsi_set_cmdq(data_array, 8, 1);
		MDELAY(30);
	//
		data_array[0] = 0x000B3902;
		data_array[1] = 0x3F3F3FEE;
		data_array[2] = 0x3F3F3F00;
		data_array[3] = 0x00221100;
		dsi_set_cmdq(data_array, 4, 1);
		MDELAY(30); 
	
		data_array[0] = 0x00123902;
		data_array[1] = 0x431212EF;
		data_array[2] = 0x24849043;
		data_array[3] = 0x21210081;
		data_array[4] = 0x80400303;
		data_array[5] = 0x00000082;
		dsi_set_cmdq(data_array, 6, 1);
		MDELAY(30);
	
		data_array[0] = 0x00123902;
		data_array[1] = 0x10350CFA;
		data_array[2] = 0x1A141C14;
		data_array[3] = 0x2C271F1E;
		data_array[4] = 0x30383B33;
		data_array[5] = 0x00003030;
		dsi_set_cmdq(data_array, 6, 1);
		MDELAY(30);
	
		data_array[0] = 0x00123902;
		data_array[1] = 0x10350CFB;
		data_array[2] = 0x1A141C14;
		data_array[3] = 0x2C271F1E;
		data_array[4] = 0x30383B33;
		data_array[5] = 0x00003030;
		dsi_set_cmdq(data_array, 6, 1);
		MDELAY(30);
	
		data_array[0] = 0x00213902;
		data_array[1] = 0x090B0BF7;
		data_array[2] = 0x080A0A09;
		data_array[3] = 0x16160108;
		data_array[4] = 0x01071717;
		data_array[5] = 0x090B0B01;
		data_array[6] = 0x080A0A09;
		data_array[7] = 0x16160108;
		data_array[8] = 0x01071717;
		data_array[9] = 0x00000001;
		dsi_set_cmdq(data_array, 10, 1);
		MDELAY(30);
		//--------------LCM tmp end------------//


}

static void set_reg_for_ESD(void)
{
	unsigned int data_array[16];
	data_array[0] = 0x00033902;
	data_array[1] = 0x5A5AF1;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);

	data_array[0] = 0x00053902;
	data_array[1] = 0x1C1003E1;
	data_array[2] = 0x00;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(10);	

	data_array[0] = 0x00033902;
	data_array[1] = 0x1000D0;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(10);
}

static void lcm_init(void)
{
    unsigned int data_array[16];

#ifdef BUILD_LK
    printf("CDT+AUO %s, LK \n", __func__);
#else
    printk("CDT+AUO %s, kernel", __func__);
#endif
	//tps65640_en();
    //mt_set_gpio_out(GPIO118, GPIO_OUT_ONE);
    lcd_reset(0);
    lcd_power_en(0);
    lcd_power_en(1);
    lcd_reset(1);
    MDELAY(20);
	lcd_reset(0);
	MDELAY(20);
	lcd_reset(1);
	MDELAY(20);

    /* software reset */
    //data_array[0] = 0x00010500;
   // dsi_set_cmdq(data_array, 1, 1);

//    MDELAY(80);
	
	/* customer */
	//data_array[0] = 0xFEA11500;
	//dsi_set_cmdq(data_array, 1, 1);
	//MDELAY(80);
    /* set display on */
    //data_array[0] = 0x00290500; 
   // dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00033902;
	data_array[1] = 0x005A5AF0;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(30);

	//add extra cmd ,remove later
	//extra_cmd();
	set_reg_for_ESD();
	
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(30);
	//data_array[0] = 0x00290500;
	//dsi_set_cmdq(data_array, 1, 1);
	//MDELAY(10);

	data_array[0] = 0x00043902;
	data_array[1] = 0x280040c3;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(20);

	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE); 
#ifdef BUILD_LK
//	tps65640_e2prom_data_check();	
#endif
	MDELAY(170);
	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(30);
}


static void lcm_suspend(void)
{
    unsigned int data_array[16];

#ifdef BUILD_LK
    printf("%s, LK \n", __func__);
#else
    printk("%s, kernel", __func__);
#endif
	//mt_set_gpio_out(GPIO118, GPIO_OUT_ZERO);//set standby LOW
    /* set display off */
    //data_array[0] = 0x00280500;
   // dsi_set_cmdq(data_array, 1, 1);

    /* enter sleep mode */
   // data_array[0] = 0x00100500;
    //dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00280500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(2);

	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ZERO);
	MDELAY(2);

	data_array[0] = 0x00043902;
	data_array[1] = 0x200040c3;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(20);
	
	data_array[0] = 0x00100500;
	dsi_set_cmdq(data_array, 1, 1);
    MDELAY(200);
	
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
	if((buffer[0]&0x80)!=0x80)
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

	id1 = mt_get_gpio_in(GPIO_HALL_2_PIN);
    id2 = mt_get_gpio_in(GPIO_HALL_1_PIN);
	id = (id1<<1)|(id2);
#ifdef BUILD_LK
    printf("CDT+AUO id1=%d,id2=%d,id=0x%x\n",id1,id2,id);
#else
    printk("CDT+AUO id1=%d,id2=%d,id=0x%x\n",id1,id2,id);
#endif
	lcd_reset(0);
    lcd_power_en(0);
    MDELAY(10);
    return (LCM_ID == id)?1:0;
}

LCM_DRIVER a080ean01_dsi_vdo_lcm_drv = 
{
    .name           = "a080ean01_dsi_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    //.esd_check   	= lcm_esd_check,
   // .esd_recover    = lcm_esd_recover,
	.compare_id    = lcm_compare_id,
};


