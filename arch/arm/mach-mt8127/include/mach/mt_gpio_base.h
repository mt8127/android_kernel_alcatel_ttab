#ifndef _MT_GPIO_BASE_H_
#define _MT_GPIO_BASE_H_

#include <mach/sync_write.h>

#define GPIO_WR32(addr, data)   mt65xx_reg_sync_writel(data, addr)
#define GPIO_RD32(addr)         __raw_readl(addr)
//#define GPIO_SET_BITS(BIT,REG)   ((*(volatile unsigned long*)(REG)) = (unsigned long)(BIT))
//#define GPIO_CLR_BITS(BIT,REG)   ((*(volatile unsigned long*)(REG)) &= ~((unsigned long)(BIT)))
#define GPIO_SET_BITS(BIT,REG)   GPIO_WR32(REG, (unsigned long)(BIT))
#define GPIO_CLR_BITS(BIT,REG)   GPIO_WR32(REG,GPIO_RD32(REG) & ~((unsigned long)(BIT)))

#define MAX_GPIO_REG_BITS      16
#define MAX_GPIO_MODE_PER_REG  5
#define GPIO_MODE_BITS         3 

#define MT_GPIO_BASE_START 0
typedef enum GPIO_PIN
{    
    GPIO_UNSUPPORTED = -1,    
	GPIO0 = MT_GPIO_BASE_START,
    GPIO1  , GPIO2  , GPIO3  , GPIO4  , GPIO5  , GPIO6  , GPIO7  , 
    GPIO8  , GPIO9  , GPIO10 , GPIO11 , GPIO12 , GPIO13 , GPIO14 , GPIO15 , 
    GPIO16 , GPIO17 , GPIO18 , GPIO19 , GPIO20 , GPIO21 , GPIO22 , GPIO23 , 
    GPIO24 , GPIO25 , GPIO26 , GPIO27 , GPIO28 , GPIO29 , GPIO30 , GPIO31 , 
    GPIO32 , GPIO33 , GPIO34 , GPIO35 , GPIO36 , GPIO37 , GPIO38 , GPIO39 , 
    GPIO40 , GPIO41 , GPIO42 , GPIO43 , GPIO44 , GPIO45 , GPIO46 , GPIO47 , 
    GPIO48 , GPIO49 , GPIO50 , GPIO51 , GPIO52 , GPIO53 , GPIO54 , GPIO55 , 
    GPIO56 , GPIO57 , GPIO58 , GPIO59 , GPIO60 , GPIO61 , GPIO62 , GPIO63 , 
    GPIO64 , GPIO65 , GPIO66 , GPIO67 , GPIO68 , GPIO69 , GPIO70 , GPIO71 , 
    GPIO72 , GPIO73 , GPIO74 , GPIO75 , GPIO76 , GPIO77 , GPIO78 , GPIO79 , 
    GPIO80 , GPIO81 , GPIO82 , GPIO83 , GPIO84 , GPIO85 , GPIO86 , GPIO87 , 
    GPIO88 , GPIO89 , GPIO90 , GPIO91 , GPIO92 , GPIO93 , GPIO94 , GPIO95 , 
    GPIO96 , GPIO97 , GPIO98 , GPIO99 , GPIO100, GPIO101, GPIO102, GPIO103, 
    GPIO104, GPIO105, GPIO106, GPIO107, GPIO108, GPIO109, GPIO110, GPIO111, 
    GPIO112, GPIO113, GPIO114, GPIO115, GPIO116, GPIO117, GPIO118, GPIO119, 
    GPIO120, GPIO121, GPIO122, GPIO123, GPIO124, GPIO125, GPIO126, GPIO127, 
    GPIO128, GPIO129, GPIO130, GPIO131, GPIO132, GPIO133, GPIO134, GPIO135, 
    GPIO136, GPIO137, GPIO138, GPIO139, GPIO140, GPIO141, GPIO142,
    MT_GPIO_BASE_MAX
}GPIO_PIN;

/*----------------------------------------------------------------------------*/
typedef struct {        /*FIXME: check GPIO spec*/
    unsigned short val;        
    unsigned short _align1;
    unsigned short set;
    unsigned short _align2;
    unsigned short rst;
    unsigned short _align3[3];
} VAL_REGS;
/*----------------------------------------------------------------------------*/
typedef struct {
    VAL_REGS    dir[9];              /*0x0000 ~ 0x008F: 144 bytes*/
    unsigned char       rsv00[112];  /*0x0090 ~ 0x00FF: 112 bytes*/
    VAL_REGS    pullen[9];           /*0x0100 ~ 0x018F: 144 bytes*/
    unsigned char       rsv01[112];  /*0x0190 ~ 0x01FF: 112 bytes*/
    VAL_REGS    pullsel[9];          /*0x0200 ~ 0x028F: 144 bytes*/
    unsigned char       rsv02[112];  /*0x0290 ~ 0x02FF: 112 bytes*/
    unsigned char       rsv03[256];  /*0x0300 ~ 0x03FF: 256 bytes*/
    VAL_REGS    dout[9];             /*0x0400 ~ 0x048F: 144 bytes*/
    unsigned char       rsv04[112];  /*0x0490 ~ 0x04FF: 112 bytes*/
    VAL_REGS    din[9];              /*0x0500 ~ 0x058F: 144 bytes*/
    unsigned char       rsv05[112];	 /*0x0590 ~ 0x05FF: 112 bytes*/
    VAL_REGS    mode[29];            /*0x0600 ~ 0x07CF: 464 bytes*/
	unsigned char		rsv06[304];	 /*0x07D0 ~ 0x08FF: 304 bytes*/
	VAL_REGS    ies[2];            	 /*0x0900 ~ 0x091F:  32 bytes*/
    VAL_REGS    smt[2];        		 /*0x0920 ~ 0x093F:	 32 bytes*/
	unsigned char		rsv07[192];	 /*0x0940 ~ 0x09FF: 192 bytes*/
	VAL_REGS    tdsel[5];        	 /*0x0A00 ~ 0x0A4F:  80 bytes*/
	unsigned char		rsv08[48];	 /*0x0A50 ~ 0x0A7F:  48 bytes*/
	VAL_REGS    rdsel[4];        	 /*0x0A80 ~ 0x0ABF:  64 bytes*/
	unsigned char		rsv09[64];	 /*0x0AC0 ~ 0x0AFF:  64 bytes*/
	VAL_REGS    drv_mode[8];         /*0x0B00 ~ 0x0B7F: 128 bytes*/
	unsigned char		rsv10[128];	 /*0x0B80 ~ 0x0BFF: 128 bytes*/
	VAL_REGS    msdc0_ctrl[7];       /*0x0C00 ~ 0x0C6F:	112 bytes*/
	VAL_REGS    msdc1_ctrl[6];       /*0x0C70 ~ 0x0CCF:	 96 bytes*/
	VAL_REGS    msdc2_ctrl[6];       /*0x0CD0 ~ 0x0D2F:	 96 bytes*/
	VAL_REGS    tm;                  /*0x0D30 ~ 0x0D3F:	 16 bytes*/
	VAL_REGS    usb;                 /*0x0D40 ~ 0x0D4F:	 16 bytes*/
	VAL_REGS    odd33_ctrl[3];       /*0x0D50 ~ 0x0D7F:	 48 bytes*/
	unsigned char		rsv11[16];	 /*0x0D80 ~ 0x0D8F:  16 bytes*/
	VAL_REGS    kpad_ctrl[2];        /*0x0D90 ~ 0x0DAF:	 32 bytes*/
	VAL_REGS    eint_ctrl[2];        /*0x0DB0 ~ 0x0DCF:	 32 bytes*/
	unsigned char		rsv12[80];	 /*0x0DD0 ~ 0x0E1F:  80 bytes*/
	VAL_REGS    bias_ctrl[2];        /*0x0E20 ~ 0x0E3F:	 32 bytes*/
} GPIO_REGS;

#endif //_MT_GPIO_BASE_H_
