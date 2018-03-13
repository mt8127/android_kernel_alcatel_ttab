#ifndef __DPI_DRV_H__
#define __DPI_DRV_H__

#include "lcm_drv.h"
#include "disp_intr.h"
#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------

#define DPI_CHECK_RET(expr)             \
    do {                                \
        DPI_STATUS ret = (expr);        \
        ASSERT(DPI_STATUS_OK == ret);   \
    } while (0)

// ---------------------------------------------------------------------------

typedef enum
{	
   DPI_STATUS_OK = 0,

   DPI_STATUS_ERROR,
} DPI_STATUS;


typedef enum
{
    DPI_FB_FORMAT_RGB565 = 0,
    DPI_FB_FORMAT_RGB888 = 1,
    DPI_FB_FORMAT_XRGB888 = 1,
    DPI_FB_FORMAT_RGBX888 = 1,
    DPI_FB_FORMAT_NUM,
} DPI_FB_FORMAT;
        

typedef enum
{
    DPI_RGB_ORDER_RGB = 0,
    DPI_RGB_ORDER_BGR = 1,
} DPI_RGB_ORDER;


typedef enum
{
    DPI_FB_0   = 0,
    DPI_FB_1   = 1,
    DPI_FB_2   = 2,
    DPI_FB_NUM,
} DPI_FB_ID;


typedef enum
{
    DPI_POLARITY_RISING  = 0,
    DPI_POLARITY_FALLING = 1
} DPI_POLARITY;

/*
typedef enum
{
    DPI_OUTPUT_BIT_NUM_8BITS = 0,
    DPI_OUTPUT_BIT_NUM_10BITS = 1,
    DPI_OUTPUT_BIT_NUM_12BITS = 2
} DPI_OUTPUT_BIT_NUM;

typedef enum
{
    DPI_OUTPUT_CHANNEL_SWAP_RGB = 0,
    DPI_OUTPUT_CHANNEL_SWAP_GBR = 1,
    DPI_OUTPUT_CHANNEL_SWAP_BRG = 2,
    DPI_OUTPUT_CHANNEL_SWAP_RBG = 3,
    DPI_OUTPUT_CHANNEL_SWAP_GRB = 4,
    DPI_OUTPUT_CHANNEL_SWAP_BGR = 5
} DPI_OUTPUT_CHANNEL_SWAP;

typedef enum
{
    DPI_OUTPUT_YC_MAP_RGB_OR_CrYCb = 0, // {R[7:4],G[7:4],B[7:4]} or {Cr[7:4],Y[7:4],Cb[7:4]}
    DPI_OUTPUT_YC_MAP_CYCY         = 4, // {C[11:4],Y[11:4],C[3:0],Y[3:0]}
    DPI_OUTPUT_YC_MAP_YCYC         = 5, // {Y[11:4],C[11:4],Y[3:0],C[3:0]}
    DPI_OUTPUT_YC_MAP_CY           = 6, // {C[11:0],Y[11:0]}
    DPI_OUTPUT_YC_MAP_YC           = 7  // {Y[11:0],C[11:0]}
} DPI_OUTPUT_YC_MAP;

typedef  enum
{
   RGB = 0,
   RGB_FULL,
   YCBCR_444,
   YCBCR_422,
   XV_YCC,
   YCBCR_444_FULL,
   YCBCR_422_FULL

} COLOR_SPACE_T;
*/
// ---------------------------------------------------------------------------

DPI_STATUS DPI_Init(BOOL isDpiPoweredOn);
DPI_STATUS DPI_Deinit(void);

DPI_STATUS DPI_Init_PLL(void);
DPI_STATUS DPI_Set_DrivingCurrent(LCM_PARAMS *lcm_params);

DPI_STATUS DPI_PowerOn(void);
DPI_STATUS DPI_PowerOff(void);

DPI_STATUS DPI_EnableClk(void);
DPI_STATUS DPI_DisableClk(void);

DPI_STATUS DPI_EnableSeqOutput(BOOL enable);
DPI_STATUS DPI_SetRGBOrder(DPI_RGB_ORDER input, DPI_RGB_ORDER output);

DPI_STATUS DPI_ConfigPixelClk(DPI_POLARITY polarity, UINT32 divisor, UINT32 duty);
DPI_STATUS DPI_ConfigDataEnable(DPI_POLARITY polarity);
DPI_STATUS DPI_ConfigVsync(DPI_POLARITY polarity,
                           UINT32 pulseWidth, UINT32 backPorch, UINT32 frontPorch);
DPI_STATUS DPI_ConfigHsync(DPI_POLARITY polarity,
                           UINT32 pulseWidth, UINT32 backPorch, UINT32 frontPorch);

DPI_STATUS DPI_FBSyncFlipWithLCD(BOOL enable);
DPI_STATUS DPI_SetDSIMode(BOOL enable);
BOOL 	   DPI_IsDSIMode(void);
DPI_STATUS DPI_FBSetFormat(DPI_FB_FORMAT format);
DPI_FB_FORMAT DPI_FBGetFormat(void);
DPI_STATUS DPI_FBSetSize(UINT32 width, UINT32 height);
DPI_STATUS DPI_FBEnable(DPI_FB_ID id, BOOL enable);
DPI_STATUS DPI_FBSetAddress(DPI_FB_ID id, UINT32 address);
DPI_STATUS DPI_FBSetPitch(DPI_FB_ID id, UINT32 pitchInByte);

DPI_STATUS DPI_SetFifoThreshold(UINT32 low, UINT32 high);

// Debug
DPI_STATUS DPI_DumpRegisters(void);

DPI_STATUS DPI_Capture_Framebuffer(unsigned int pvbuf, unsigned int bpp);

//FM De-sense
DPI_STATUS DPI_FMDesense_Query(void);
DPI_STATUS DPI_FM_Desense(unsigned long freq);
DPI_STATUS DPI_Get_Default_CLK(unsigned int *clk);
DPI_STATUS DPI_Get_Current_CLK(unsigned int *clk);
DPI_STATUS DPI_Change_CLK(unsigned int clk);
DPI_STATUS DPI_Reset_CLK(void);

void DPI_mipi_switch(bool on);
void DPI_DisableIrq(void);
void DPI_EnableIrq(void);
DPI_STATUS DPI_FreeIRQ(void);

DPI_STATUS DPI_EnableInterrupt(DISP_INTERRUPT_EVENTS eventID);
DPI_STATUS DPI_SetInterruptCallback(void (*pCB)(DISP_INTERRUPT_EVENTS eventID));
void DPI_WaitVSYNC(void);
void DPI_InitVSYNC(unsigned int vsync_interval);
void DPI_PauseVSYNC(bool enable);

DPI_STATUS DPI_ConfigLVDS(LCM_PARAMS *lcm_params);
DPI_STATUS DPI_LVDS_Enable(void);
DPI_STATUS DPI_LVDS_Disable(void);

DPI_STATUS DPI_ConfigHDMI(void);

unsigned int DPI_Check_LCM(void);
// ---------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif // __DPI_DRV_H__
