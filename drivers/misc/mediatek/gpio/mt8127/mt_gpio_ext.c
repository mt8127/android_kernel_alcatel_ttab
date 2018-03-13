/******************************************************************************
 * mt_gpio_ext.c - MTKLinux GPIO Device Driver
 * 
 * Copyright 2008-2009 MediaTek Co.,Ltd.
 * 
 * DESCRIPTION:
 *     This file provid the other drivers GPIO debug functions
 *
 ******************************************************************************/

#include <mach/mt_reg_base.h>
#include <mach/mt_pmic_wrap.h>
#include <mach/mt_gpio.h>
#include <mach/mt_gpio_core.h>
#include <mach/mt_gpio_ext.h>

#define MAX_GPIO_REG_BITS      16
#define MAX_GPIO_MODE_PER_REG  5
#define GPIO_MODE_BITS         3 
#define GPIOEXT_BASE        (0xC000) 			//PMIC GPIO base.

static GPIOEXT_REGS *gpioext_reg = (GPIOEXT_REGS*)(GPIOEXT_BASE);
//set extend GPIO
/*---------------------------------------------------------------------------*/
int mt_set_gpio_dir_ext(unsigned long pin, unsigned long dir)
{
    unsigned long pos;
    unsigned long bit;
	int ret=0;
    GPIOEXT_REGS *reg = gpioext_reg;

	pin -= MT_GPIO_EXT_START;    
    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;
    
    if (dir == GPIO_DIR_IN)
        ret=GPIOEXT_SET_BITS((1L << bit), &reg->dir[pos].rst);
    else
        ret=GPIOEXT_SET_BITS((1L << bit), &reg->dir[pos].set);
	if(ret!=0) return -ERWRAPPER;
    return RSUCCESS;
    
}
/*---------------------------------------------------------------------------*/
int mt_get_gpio_dir_ext(unsigned long pin)
{    
    unsigned long pos;
    unsigned long bit;
    signed long long data;
    GPIOEXT_REGS *reg = gpioext_reg;

	pin -= MT_GPIO_EXT_START;    
    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;
    
    data = GPIOEXT_RD(&reg->dir[pos].val);
    if(data < 0) return -ERWRAPPER;
    return (((data & (1L << bit)) != 0)? 1: 0);        
}
/*---------------------------------------------------------------------------*/
int mt_set_gpio_pull_enable_ext(unsigned long pin, unsigned long enable)
{
    unsigned long pos;
    unsigned long bit;
	int ret;
    GPIOEXT_REGS *reg = gpioext_reg;
    
	pin -= MT_GPIO_EXT_START;    
    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;

    if (enable == GPIO_PULL_DISABLE)
        ret=GPIOEXT_SET_BITS((1L << bit), &reg->pullen[pos].rst);
    else
        ret=GPIOEXT_SET_BITS((1L << bit), &reg->pullen[pos].set);
    if(ret < 0) return -ERWRAPPER;
    return RSUCCESS;
}
/*---------------------------------------------------------------------------*/
int mt_get_gpio_pull_enable_ext(unsigned long pin)
{
    unsigned long pos;
    unsigned long bit;
    signed long long data;
    GPIOEXT_REGS *reg = gpioext_reg;

	pin -= MT_GPIO_EXT_START;    
    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;

    data = GPIOEXT_RD(&reg->pullen[pos].val);
    if(data < 0) return -ERWRAPPER;
    return (((data & (1L << bit)) != 0)? 1: 0);        
}
/*---------------------------------------------------------------------------*/
int mt_set_gpio_ies_ext(unsigned long pin, unsigned long enable)
{
    return RSUCCESS;
}
/*---------------------------------------------------------------------------*/
int mt_get_gpio_ies_ext(unsigned long pin)
{
    return RSUCCESS;
}
/*---------------------------------------------------------------------------*/
int mt_set_gpio_pull_select_ext(unsigned long pin, unsigned long select)
{
    unsigned long pos;
    unsigned long bit;
	int ret=0;
    GPIOEXT_REGS *reg = gpioext_reg;

	pin -= MT_GPIO_EXT_START;    
    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;
    
    if (select == GPIO_PULL_DOWN)
        ret=GPIOEXT_SET_BITS((1L << bit), &reg->pullsel[pos].rst);
    else
        ret=GPIOEXT_SET_BITS((1L << bit), &reg->pullsel[pos].set);
	if(ret!=0) return -ERWRAPPER;
    return RSUCCESS;
}
/*---------------------------------------------------------------------------*/
int mt_get_gpio_pull_select_ext(unsigned long pin)
{
    unsigned long pos;
    unsigned long bit;
    signed long long data;
    GPIOEXT_REGS *reg = gpioext_reg;
    
	pin -= MT_GPIO_EXT_START;    
    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;

    data = GPIOEXT_RD(&reg->pullsel[pos].val);
    if(data < 0) return -ERWRAPPER;
    return (((data & (1L << bit)) != 0)? 1: 0);        
}
/*---------------------------------------------------------------------------*/
int mt_set_gpio_inversion_ext(unsigned long pin, unsigned long enable)
{
    unsigned long pos;
    unsigned long bit;
	int ret = 0;
    GPIOEXT_REGS *reg = gpioext_reg;

	pin -= MT_GPIO_EXT_START;    
    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;
    
    if (enable == GPIO_DATA_UNINV)
        ret=GPIOEXT_SET_BITS((1L << bit), &reg->dinv[pos].rst);
    else
        ret=GPIOEXT_SET_BITS((1L << bit), &reg->dinv[pos].set);
	if(ret!=0) return -ERWRAPPER;
    return RSUCCESS;
}
/*---------------------------------------------------------------------------*/
int mt_get_gpio_inversion_ext(unsigned long pin)
{
    unsigned long pos;
    unsigned long bit;
    signed long long data;
    GPIOEXT_REGS *reg = gpioext_reg;
    
	pin -= MT_GPIO_EXT_START;    
    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;

    data = GPIOEXT_RD(&reg->dinv[pos].val);
    if(data < 0) return -ERWRAPPER;
    return (((data & (1L << bit)) != 0)? 1: 0);        
}
/*---------------------------------------------------------------------------*/
int mt_set_gpio_out_ext(unsigned long pin, unsigned long output)
{
    unsigned long pos;
    unsigned long bit;
	int ret = 0;
    GPIOEXT_REGS *reg = gpioext_reg;

	pin -= MT_GPIO_EXT_START;    
    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;
    
    if (output == GPIO_OUT_ZERO)
        ret=GPIOEXT_SET_BITS((1L << bit), &reg->dout[pos].rst);
    else
        ret=GPIOEXT_SET_BITS((1L << bit), &reg->dout[pos].set);
	if(ret!=0) return -ERWRAPPER;
    return RSUCCESS;
}
/*---------------------------------------------------------------------------*/
int mt_get_gpio_out_ext(unsigned long pin)
{
    unsigned long pos;
    unsigned long bit;
    signed long long data;
    GPIOEXT_REGS *reg = gpioext_reg;

	pin -= MT_GPIO_EXT_START;    
    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;

    data = GPIOEXT_RD(&reg->dout[pos].val);
    if(data < 0) return -ERWRAPPER;
    return (((data & (1L << bit)) != 0)? 1: 0);        
}
/*---------------------------------------------------------------------------*/
int mt_get_gpio_in_ext(unsigned long pin)
{
    unsigned long pos;
    unsigned long bit;
    signed long long data;
    GPIOEXT_REGS *reg = gpioext_reg;
    
	pin -= MT_GPIO_EXT_START;    
    pos = pin / MAX_GPIO_REG_BITS;
    bit = pin % MAX_GPIO_REG_BITS;

    data = GPIOEXT_RD(&reg->din[pos].val);
    if(data < 0) return -ERWRAPPER;
    return (((data & (1L << bit)) != 0)? 1: 0);        
}
/*---------------------------------------------------------------------------*/
int mt_set_gpio_mode_ext(unsigned long pin, unsigned long mode)
{
    unsigned long pos;
    unsigned long bit;
    signed long long data;
	int ret=0;
    unsigned long mask = (1L << GPIO_MODE_BITS) - 1;    
    GPIOEXT_REGS *reg = gpioext_reg;

	pin -= MT_GPIO_EXT_START;    
    pos = pin / MAX_GPIO_MODE_PER_REG;
    bit = pin % MAX_GPIO_MODE_PER_REG;
   
    data = GPIOEXT_RD(&reg->mode[pos].val);
    if(data < 0){    
		return -ERWRAPPER;
	}

    data &= ~(mask << (GPIO_MODE_BITS*bit));
    data |= (mode << (GPIO_MODE_BITS*bit));
    
    ret = GPIOEXT_WR(&reg->mode[pos].val, data);
	if(ret!=0) return -ERWRAPPER;
    return RSUCCESS;
}
/*---------------------------------------------------------------------------*/
int mt_get_gpio_mode_ext(unsigned long pin)
{
    unsigned long pos;
    unsigned long bit;
    signed long long data;
    unsigned long mask = (1L << GPIO_MODE_BITS) - 1;    
    GPIOEXT_REGS *reg = gpioext_reg;
    
	pin -= MT_GPIO_EXT_START;    
    pos = pin / MAX_GPIO_MODE_PER_REG;
    bit = pin % MAX_GPIO_MODE_PER_REG;

    data = GPIOEXT_RD(&reg->mode[pos].val);
    if(data < 0) return -ERWRAPPER;
    
    return ((data >> (GPIO_MODE_BITS*bit)) & mask);
}

/*---------------------------------------------------------------------------*/
int mt_set_gpio_smt_ext(unsigned long pin, unsigned long enable)
{
	dump_stack();
	return ERINVAL;
}

/*---------------------------------------------------------------------------*/
int mt_get_gpio_smt_ext(unsigned long pin)
{
	dump_stack();
	return ERINVAL;
}
