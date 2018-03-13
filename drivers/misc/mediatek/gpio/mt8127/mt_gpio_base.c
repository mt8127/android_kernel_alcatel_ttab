/******************************************************************************
 * mt_gpio_base.c - MTKLinux GPIO Device Driver
 * 
 * Copyright 2008-2009 MediaTek Co.,Ltd.
 * 
 * DESCRIPTION:
 *     This file provid the other drivers GPIO relative functions
 *
 ******************************************************************************/

#include <mach/sync_write.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_gpio_core.h>
#include <mach/mt_gpio_base.h>
#include <cust_gpio_usage.h>

/*-------for special kpad pupd-----------*/
struct kpad_pupd {
	unsigned char 	pin;
	unsigned char	reg;
	unsigned char	bit;
};
static struct kpad_pupd kpad_pupd_spec[] = {
	{GPIO33,	0,	2},
	{GPIO34,	0,	6},
	{GPIO35,	0,	10},
	{GPIO36,	1,	2},
	{GPIO37,	1,	6},
	{GPIO38,	1,	10}
};
/*-------for msdc pupd-----------*/
struct msdc_pupd {
	unsigned char 	pin;
	unsigned char	reg;
	unsigned char	bit;
};
static struct msdc_pupd msdc_pupd_spec[2][6] = {
	{
	{GPIO121,	1,	8},
	{GPIO122,	0,	8},
	{GPIO123,	3,	0},
	{GPIO124,	3,	4},
	{GPIO125,	3,	8},
	{GPIO126,	3,	12}},
	{
	{GPIO85,	1,	8},
	{GPIO86,	0,	8},
	{GPIO87,	3,	0},
	{GPIO88,	3,	4},
	{GPIO89,	3,	8},
	{GPIO90,	3,	12}}
};
/*-------for eint14~eint17 eint21 pupd-----------*/
struct eint_pupd {
	unsigned char 	pin;
	unsigned char	reg;
	unsigned char	bit;
};
static struct eint_pupd eint_pupd_spec[] = {
	{GPIO46,	0,	2},
	{GPIO47,	0,	6},
	{GPIO48,	0,	10},
	{GPIO49,	0,	14},
	{GPIO142,	1,	2}
};

/*---------------------------------------*/
static GPIO_REGS *gpio_reg = (GPIO_REGS*)(GPIO_BASE);
/*---------------------------------------------------------------------------*/
int mt_set_gpio_dir_base(unsigned long pin, unsigned long dir)
{
    unsigned long pos;
    unsigned long bit;
    GPIO_REGS *reg = gpio_reg;

    if ((dir == GPIO_DIR_OUT) &&
    			((pin >= GPIO75 && pin <= GPIO78)||(pin >= GPIO57 && pin <= GPIO58)
    			||(pin >= GPIO138 &&pin <= GPIO141))){
    		return -ERINVAL;
    }
    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;
    
    if (dir == GPIO_DIR_IN)
        GPIO_SET_BITS((1L << bit), &reg->dir[pos].rst);
    else
        GPIO_SET_BITS((1L << bit), &reg->dir[pos].set);
    return RSUCCESS;
}
/*---------------------------------------------------------------------------*/
int mt_get_gpio_dir_base(unsigned long pin)
{    
    unsigned long pos;
    unsigned long bit;
    unsigned long data;
    GPIO_REGS *reg = gpio_reg;

    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;
    
    data = GPIO_RD32(&reg->dir[pos].val);
    return (((data & (1L << bit)) != 0)? 1: 0);        
}
/*---------------------------------------------------------------------------*/
int mt_set_gpio_pull_enable_base(unsigned long pin, unsigned long enable)
{
    unsigned long pos;
    unsigned long bit;
	unsigned long i,j;
    GPIO_REGS *reg = gpio_reg;

	/*for special kpad pupd*/
	for(i = 0; i < sizeof(kpad_pupd_spec)/sizeof(kpad_pupd_spec[0]); i++){
		if (pin == kpad_pupd_spec[i].pin){
			if (enable == GPIO_PULL_DISABLE){
				GPIO_SET_BITS((3L << (kpad_pupd_spec[i].bit-2)), &reg->kpad_ctrl[kpad_pupd_spec[i].reg].rst);
			} else {
				GPIO_SET_BITS((1L << (kpad_pupd_spec[i].bit-2)), &reg->kpad_ctrl[kpad_pupd_spec[i].reg].set);
			}
			return RSUCCESS;
		}
	}

	/*for special eint pupd*/
	if(((pin >= GPIO46) && (pin <= GPIO49)) || (pin == GPIO142)) {
		for(i = 0; i < sizeof(eint_pupd_spec)/sizeof(eint_pupd_spec[0]); i++){
			if (pin == eint_pupd_spec[i].pin){
				if (enable == GPIO_PULL_ENABLE)
					GPIO_SET_BITS((1L << (eint_pupd_spec[i].bit-2)), &reg->eint_ctrl[eint_pupd_spec[i].reg].set);
				else
					GPIO_SET_BITS((3L << (eint_pupd_spec[i].bit-2)), &reg->eint_ctrl[eint_pupd_spec[i].reg].rst);
				return RSUCCESS;
			}
		}
	}

	if(((pin >= GPIO85)&&(pin <= GPIO90)) ||((pin >= GPIO121)&&(pin <= GPIO126))) {
		/* msdc IO */
		for(i = 0; i < sizeof(msdc_pupd_spec)/sizeof(msdc_pupd_spec[0]); i++){
			for(j = 0; j < sizeof(msdc_pupd_spec[0])/sizeof(msdc_pupd_spec[0][0]); j++){
				if (pin == msdc_pupd_spec[i][j].pin){
					if (enable == GPIO_PULL_DISABLE){
						if (i == 0) {
							GPIO_SET_BITS((1L << (msdc_pupd_spec[i][j].bit + 1)), &reg->msdc1_ctrl[msdc_pupd_spec[i][j].reg].rst);
							GPIO_SET_BITS((1L << (msdc_pupd_spec[i][j].bit + 2)), &reg->msdc1_ctrl[msdc_pupd_spec[i][j].reg].rst);
						} else if (i == 1) {
							GPIO_SET_BITS((1L << (msdc_pupd_spec[i][j].bit + 1)), &reg->msdc2_ctrl[msdc_pupd_spec[i][j].reg].rst);
							GPIO_SET_BITS((1L << (msdc_pupd_spec[i][j].bit + 2)), &reg->msdc2_ctrl[msdc_pupd_spec[i][j].reg].rst);
						}
					}else{
						if (i == 0) {
							GPIO_SET_BITS((1L << (msdc_pupd_spec[i][j].bit + 1)), &reg->msdc1_ctrl[msdc_pupd_spec[i][j].reg].set);
							GPIO_SET_BITS((1L << (msdc_pupd_spec[i][j].bit + 2)), &reg->msdc1_ctrl[msdc_pupd_spec[i][j].reg].rst);
						} else if (i == 1) {
							GPIO_SET_BITS((1L << (msdc_pupd_spec[i][j].bit + 1)), &reg->msdc2_ctrl[msdc_pupd_spec[i][j].reg].set);
							GPIO_SET_BITS((1L << (msdc_pupd_spec[i][j].bit + 2)), &reg->msdc2_ctrl[msdc_pupd_spec[i][j].reg].rst);
						}
					}
					return RSUCCESS;
				}
			}
		}
	}

	if ((pin >= GPIO127)&&(pin <= GPIO137)){
		return GPIO_PULL_EN_UNSUPPORTED;
	}else{
		pos = pin / MAX_GPIO_REG_BITS;
	bit = pin % MAX_GPIO_REG_BITS;

	if (enable == GPIO_PULL_DISABLE)
		GPIO_SET_BITS((1L << bit), &reg->pullen[pos].rst);
	else
		GPIO_SET_BITS((1L << bit), &reg->pullen[pos].set);
	}
    return RSUCCESS;
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
int mt_get_gpio_pull_enable_base(unsigned long pin)
{
    unsigned long pos=0;
    unsigned long bit=0;
    unsigned long data=0;
	unsigned long i,j;
    GPIO_REGS *reg = gpio_reg;

	/*for special kpad pupd*/
	for(i = 0; i < sizeof(kpad_pupd_spec)/sizeof(kpad_pupd_spec[0]); i++){
		if (pin == kpad_pupd_spec[i].pin){
			data = GPIO_RD32(&reg->kpad_ctrl[kpad_pupd_spec[i].reg].val);
			return (((data & (3L << (kpad_pupd_spec[i].bit-2))) == 0)? 0: 1);
		}
	}

	/*for special eint pupd*/
	if(((pin >= GPIO46) && (pin <= GPIO49)) || (pin == GPIO142)) {
		for(i = 0; i < sizeof(eint_pupd_spec)/sizeof(eint_pupd_spec[0]); i++){
			if (pin == eint_pupd_spec[i].pin){
				data = GPIO_RD32(&reg->eint_ctrl[eint_pupd_spec[i].reg].val);
				return (((data & (3L << (eint_pupd_spec[i].bit-2))) == 0)? 0: 1);
			}
		}
	}

	if(((pin >= GPIO85)&&(pin <= GPIO90)) ||((pin >= GPIO121)&&(pin <= GPIO126))) {
			/* msdc IO */
			for(i = 0; i < sizeof(msdc_pupd_spec)/sizeof(msdc_pupd_spec[0]); i++){
				for(j = 0; j < sizeof(msdc_pupd_spec[0])/sizeof(msdc_pupd_spec[0][0]); j++){
					if (pin == msdc_pupd_spec[i][j].pin){
						if (i == 0) {
							data = GPIO_RD32(&reg->msdc1_ctrl[msdc_pupd_spec[i][j].reg].val);
						} else if (i == 1) {
							data = GPIO_RD32(&reg->msdc2_ctrl[msdc_pupd_spec[i][j].reg].val);
						}

						return (((data & (3L << (msdc_pupd_spec[i][j].bit + 1))) == 0)? 0: 1);
				}
			}
		}
	}

    if ((pin >= GPIO127)&&(pin <= GPIO137)){
		return GPIO_PULL_EN_UNSUPPORTED;
	}else{
	pos = pin / MAX_GPIO_REG_BITS;
	bit = pin % MAX_GPIO_REG_BITS;

	data = GPIO_RD32(&reg->pullen[pos].val);
	}
    return (((data & (1L << bit)) != 0)? 1: 0);        
}
/*---------------------------------------------------------------------------*/
int mt_set_gpio_ies_base(unsigned long pin, unsigned long enable)
{
    unsigned long pos;
    unsigned long bit;
    GPIO_REGS *reg = gpio_reg;

	pos = pin / MAX_GPIO_REG_BITS;
	bit = pin % MAX_GPIO_REG_BITS;

	if (enable == GPIO_IES_DISABLE)
		GPIO_SET_BITS((1L << bit), &reg->ies[pos].rst);
	else
		GPIO_SET_BITS((1L << bit), &reg->ies[pos].set);

    return RSUCCESS;
}
/*---------------------------------------------------------------------------*/
int mt_get_gpio_ies_base(unsigned long pin)
{
    unsigned long pos;
    unsigned long bit;
    unsigned long data;
    GPIO_REGS *reg = gpio_reg;

	pos = pin / MAX_GPIO_REG_BITS;
	bit = pin % MAX_GPIO_REG_BITS;

	data = GPIO_RD32(&reg->ies[pos].val);
    return (((data & (1L << bit)) != 0)? 1: 0);        
}
/*---------------------------------------------------------------------------*/
int mt_set_gpio_pull_select_base(unsigned long pin, unsigned long select)
{
    unsigned long pos;
    unsigned long bit;
	unsigned long i,j;

    GPIO_REGS *reg = gpio_reg;

	for(i = 0; i < sizeof(kpad_pupd_spec)/sizeof(kpad_pupd_spec[0]); i++){
		if (pin == kpad_pupd_spec[i].pin){
			if (select == GPIO_PULL_DOWN)
				GPIO_SET_BITS((1L << kpad_pupd_spec[i].bit), &reg->kpad_ctrl[kpad_pupd_spec[i].reg].set);
			else
				GPIO_SET_BITS((1L << kpad_pupd_spec[i].bit), &reg->kpad_ctrl[kpad_pupd_spec[i].reg].rst);
			return RSUCCESS;
		}
	}

	/*for special eint pupd*/
	if(((pin >= GPIO46) && (pin <= GPIO49)) || (pin == GPIO142)) {
		for(i = 0; i < sizeof(eint_pupd_spec)/sizeof(eint_pupd_spec[0]); i++){
			if (pin == eint_pupd_spec[i].pin){
				if (select == GPIO_PULL_DOWN)
					GPIO_SET_BITS((1L << eint_pupd_spec[i].bit), &reg->eint_ctrl[eint_pupd_spec[i].reg].set);
				else
					GPIO_SET_BITS((1L << eint_pupd_spec[i].bit), &reg->eint_ctrl[eint_pupd_spec[i].reg].rst);
				return RSUCCESS;
			}
		}
	}


	/* msdc IO */
	if (((pin >= GPIO85)&&(pin <= GPIO90))||((pin >= GPIO121)&&(pin <= GPIO126))){
		for(i = 0; i < sizeof(msdc_pupd_spec)/sizeof(msdc_pupd_spec[0]); i++){
			for(j = 0; j < sizeof(msdc_pupd_spec[0])/sizeof(msdc_pupd_spec[0][0]); j++){
				if (pin == msdc_pupd_spec[i][j].pin){
					if (select == GPIO_PULL_DOWN){
						if (i == 0) {
							GPIO_SET_BITS((1L << msdc_pupd_spec[i][j].bit), &reg->msdc1_ctrl[msdc_pupd_spec[i][j].reg].set);
						} else if (i == 1) {
							GPIO_SET_BITS((1L << msdc_pupd_spec[i][j].bit), &reg->msdc2_ctrl[msdc_pupd_spec[i][j].reg].set);
						}
					}else{
						if (i == 0) {
							GPIO_SET_BITS((1L << msdc_pupd_spec[i][j].bit), &reg->msdc1_ctrl[msdc_pupd_spec[i][j].reg].rst);
						} else if (i == 1) {
							GPIO_SET_BITS((1L << msdc_pupd_spec[i][j].bit), &reg->msdc2_ctrl[msdc_pupd_spec[i][j].reg].rst);
						}
					}
					return RSUCCESS;
				}
			}
		}
	}

	if ((select == GPIO_PULL_UP) &&
				(((pin >= GPIO75)&&(pin <= GPIO78))||((pin >= GPIO57)&&(pin <= GPIO58))||((pin >= GPIO138)&&(pin <= GPIO141)))){
			return -ERINVAL;
	}else{
	pos = pin / MAX_GPIO_REG_BITS;
	bit = pin % MAX_GPIO_REG_BITS;
	
	if (select == GPIO_PULL_DOWN)
		GPIO_SET_BITS((1L << bit), &reg->pullsel[pos].rst);
	else
		GPIO_SET_BITS((1L << bit), &reg->pullsel[pos].set);
	}
    return RSUCCESS;
}
/*---------------------------------------------------------------------------*/
int mt_get_gpio_pull_select_base(unsigned long pin)
{
    unsigned long pos=0;
    unsigned long bit=0;
    unsigned long data=0;
	unsigned long i,j;
    GPIO_REGS *reg = gpio_reg;

	/*for special kpad pupd*/
	for(i = 0; i < sizeof(kpad_pupd_spec)/sizeof(kpad_pupd_spec[0]); i++){
		if (pin == kpad_pupd_spec[i].pin){
			data = GPIO_RD32(&reg->kpad_ctrl[kpad_pupd_spec[i].reg].val);
			return (((data & (1L << kpad_pupd_spec[i].bit)) != 0)? 0: 1);
		}
	}

	/*for special eint pupd*/
	if(((pin >= GPIO46) && (pin <= GPIO49)) || (pin == GPIO142)) {
		for(i = 0; i < sizeof(eint_pupd_spec)/sizeof(eint_pupd_spec[0]); i++){
			if (pin == eint_pupd_spec[i].pin){
				data = GPIO_RD32(&reg->eint_ctrl[eint_pupd_spec[i].reg].val);
				return (((data & (1L << eint_pupd_spec[i].bit)) != 0)? 0: 1);
			}
		}
	}

	/* msdc IO */
	if (((pin >= GPIO85)&&(pin <= GPIO90))||((pin >= GPIO121)&&(pin <= GPIO126))){
		for(i = 0; i < sizeof(msdc_pupd_spec)/sizeof(msdc_pupd_spec[0]); i++){
			for(j = 0; j < sizeof(msdc_pupd_spec[0])/sizeof(msdc_pupd_spec[0][0]); j++){
				if (pin == msdc_pupd_spec[i][j].pin){
					if (i == 0) {
						data = GPIO_RD32(&reg->msdc1_ctrl[msdc_pupd_spec[i][j].reg].val);
					} else if (i == 1) {
						data = GPIO_RD32(&reg->msdc2_ctrl[msdc_pupd_spec[i][j].reg].val);
					}

					return (((data & (1L << msdc_pupd_spec[i][j].bit)) != 0)? 0: 1);
				}
			}
		}
	}


	pos = pin / MAX_GPIO_REG_BITS;
	bit = pin % MAX_GPIO_REG_BITS;

	data = GPIO_RD32(&reg->pullsel[pos].val);

    return (((data & (1L << bit)) != 0)? 1: 0);        
}
/*---------------------------------------------------------------------------*/
int mt_set_gpio_inversion_base(unsigned long pin, unsigned long enable)
{/*FIX-ME
    unsigned long pos;
    unsigned long bit;
    unsigned long data;
    unsigned long mask;
    GPIO_REGS *reg = gpio_reg;

	//GPIO44~GPIO49 is special pin for sim
	if((pin >= 44) && (pin <= 46)){
		data = GPIO_RD32(GPIO_BASE+ 0X990);
    	mask = (1L << 1) - 1;    
		//pos = pin / 4;
		bit = pin % 4;
		data &= ~(mask << (bit));
		data |= (enable << (bit));
		
		GPIO_WR32(GPIO_BASE+ 0X990, data);
	}else if((pin >= 47) && (pin <= 49)){
		pin -= 3;
		data = GPIO_RD32(GPIO_BASE+ 0X9B0);
    	mask = (1L << 1) - 1;    
		//pos = pin / 4;
		bit = pin % 4;
		data &= ~(mask << (bit));
		data |= (enable << (bit));
		
		GPIO_WR32(GPIO_BASE+ 0X9B0, data);
	}else{
		pos = pin / MAX_GPIO_REG_BITS;
		bit = pin % MAX_GPIO_REG_BITS;
		
		if (enable == GPIO_DATA_UNINV)
			GPIO_SET_BITS((1L << bit), &reg->dinv[pos].rst);
		else
			GPIO_SET_BITS((1L << bit), &reg->dinv[pos].set);
*/
    return RSUCCESS;
}
/*---------------------------------------------------------------------------*/
int mt_get_gpio_inversion_base(unsigned long pin)
{/*FIX-ME
    unsigned long pos;
    unsigned long bit;
    unsigned long data;
    GPIO_REGS *reg = gpio_reg;

	//GPIO44~GPIO49 is special pin for sim
	if((pin >= 44) && (pin <= 46)){
		data = GPIO_RD32(GPIO_BASE+ 0X990);
		bit = (pin % 4);
	}else if((pin >= 47) && (pin <= 49)){
		pin -= 3;
		data = GPIO_RD32(GPIO_BASE+ 0X9B0);
		bit = (pin % 4);
	}else{
		pos = pin / MAX_GPIO_REG_BITS;
		bit = pin % MAX_GPIO_REG_BITS;

		data = GPIO_RD32(&reg->dinv[pos].val);
	}
    return (((data & (1L << bit)) != 0)? 1: 0);    */
	return 0;//FIX-ME
}
/*---------------------------------------------------------------------------*/
int mt_set_gpio_out_base(unsigned long pin, unsigned long output)
{
    unsigned long pos;
    unsigned long bit;
    GPIO_REGS *reg = gpio_reg;

    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;
    
    if (output == GPIO_OUT_ZERO)
        GPIO_SET_BITS((1L << bit), &reg->dout[pos].rst);
    else
        GPIO_SET_BITS((1L << bit), &reg->dout[pos].set);
    return RSUCCESS;
}
/*---------------------------------------------------------------------------*/
int mt_get_gpio_out_base(unsigned long pin)
{
    unsigned long pos;
    unsigned long bit;
    unsigned long data;
    GPIO_REGS *reg = gpio_reg;

    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;

    data = GPIO_RD32(&reg->dout[pos].val);
    return (((data & (1L << bit)) != 0)? 1: 0);        
}
/*---------------------------------------------------------------------------*/
int mt_get_gpio_in_base(unsigned long pin)
{
    unsigned long pos;
    unsigned long bit;
    unsigned long data;
    GPIO_REGS *reg = gpio_reg;

    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;

    data = GPIO_RD32(&reg->din[pos].val);
    return (((data & (1L << bit)) != 0)? 1: 0);        
}
/*---------------------------------------------------------------------------*/
int mt_set_gpio_mode_base(unsigned long pin, unsigned long mode)
{
    unsigned long pos;
    unsigned long bit;
    unsigned long data;
    unsigned long mask = (1L << GPIO_MODE_BITS) - 1;    
    GPIO_REGS *reg = gpio_reg;

	pos = pin / MAX_GPIO_MODE_PER_REG;
	bit = pin % MAX_GPIO_MODE_PER_REG;
   
	data = GPIO_RD32(&reg->mode[pos].val);

	data &= ~(mask << (GPIO_MODE_BITS*bit));
	data |= (mode << (GPIO_MODE_BITS*bit));
	
	GPIO_WR32(&reg->mode[pos].val, data);

    return RSUCCESS;
}
/*---------------------------------------------------------------------------*/
int mt_get_gpio_mode_base(unsigned long pin)
{
    unsigned long pos;
    unsigned long bit;
    unsigned long data;
    unsigned long mask = (1L << GPIO_MODE_BITS) - 1;    
    GPIO_REGS *reg = gpio_reg;

	pos = pin / MAX_GPIO_MODE_PER_REG;
	bit = pin % MAX_GPIO_MODE_PER_REG;

	data = GPIO_RD32(&reg->mode[pos].val);
	
	return ((data >> (GPIO_MODE_BITS*bit)) & mask);
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
#ifdef CONFIG_PM 
/*---------------------------------------------------------------------------*/
void mt_gpio_suspend(void)
{
    /* compatible with HAL */
}
/*---------------------------------------------------------------------------*/
void mt_gpio_resume(void)
{
	/* compatible with HAL */
}
/*---------------------------------------------------------------------------*/
#endif /*CONFIG_PM*/
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
int mt_set_gpio_smt_base(unsigned long pin, unsigned long enable)
{
	/* compatible with HAL */
	return RSUCCESS;
}

/*---------------------------------------------------------------------------*/
int mt_get_gpio_smt_base(unsigned long pin)
{
	/* compatible with HAL */
	return RSUCCESS;
}