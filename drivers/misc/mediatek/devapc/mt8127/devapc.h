#define MOD_NO_IN_1_DEVAPC                  16
//#define DEVAPC_MODULE_MAX_NUM               32  
#define DEVAPC_TAG                          "DEVAPC"
#define MAX_TIMEOUT                         100
#define ABORT_EMI                           0x00002000 
 
 
// device apc attribute
 typedef enum
 {
     E_L0=0,
     E_L1,
     E_L2,
     E_L3,
     E_MAX_APC_ATTR
 }APC_ATTR;
 
 // domain index 
 typedef enum
 {
     E_DOMAIN_0 = 0,
     E_DOMAIN_1 ,
     E_DOMAIN_2 , 
     E_DOMAIN_3 ,
     E_MAX
 }E_MASK_DOM;
 
 
 typedef struct {
     const char      *device_name;
     bool            forbidden;
 } DEVICE_INFO;
 
  
 static DEVICE_INFO D_APC0_Devices[] = {
     {"INFRA_AO_TOP_LEVEL_CLOCK_GENERATOR",      FALSE},
     {"INFRA_AO_INFRASYS_CONFIG_REGS",           FALSE},
     {"INFRA_AO_FHCTL",                          FALSE},
     {"INFRA_AO_PERISYS_CONFIG_REGS",            FALSE},
     {"INFRA_AO_DRAM_CONTROLLER",                FALSE},
     {"INFRA_AO_GPIO_CONTROLLER",                FALSE},
     {"INFRA_AO_TOP_LEVEL_SLP_MANAGER",          FALSE},
     {"INFRA_AO_TOP_LEVEL_RESET_GENERATOR",      FALSE},
     {"INFRA_AO_GPT",                            FALSE},
     {"INFRA_AO_RSVD",                           FALSE},
     {"INFRA_AO_SEJ",                            FALSE},
     {"INFRA_AO_APMCU_EINT_CONTROLLER",          FALSE},
     {"INFRA_AO_SMI_CONTROL_REG1",               FALSE},
     {"INFRA_AO_PMIC_WRAP_CONTROL_REG",          FALSE},
     {"INFRA_AO_DEVICE_APC_AO",                  FALSE},
     {"INFRA_AO_DDRPHY_CONTROL_REG",             FALSE},
     {"INFRA_AO_MIPI_CONTROL_REG",               FALSE},
     {"INFRA_AO_KPAD_CONTROL_REG",               FALSE},
     {"INFRA_AO_CEC_CONTROL_REG",                FALSE},
     {"INFRA_AO_IRRX_CONTROL_REG",               FALSE},
     {"INFRASYS_MCUSYS_CONFIG_REG",              FALSE},
     {"INFRASYS_CONTROL_REG",                    FALSE},
     {"INFRASYS_BOOTROM/SRAM",                   FALSE},
     {"INFRASYS_EMI_BUS_INTERFACE",              FALSE},
     {"INFRASYS_SYSTEM_CIRQ",                    FALSE},
     {"INFRASYS_M4U_CONFIGURATION",              FALSE},
     {"INFRASYS_EFUSEC",                         FALSE},
     {"INFRASYS_DEVICE_APC_MONITOR",             FALSE},
     {"INFRASYS_MCU_BIU_CONFIGURATION",          FALSE},
     {"INFRASYS_AP_MIXED_CONTROL_REG",           FALSE},
     {"INFRASYS_RSVD",                           FALSE},
     {"INFRASYS_RSVD",                           FALSE},
     {"INFRASYS_GPIO1_CONTROLLER",               FALSE},
     {"INFRASYS_MBIST_CONTROL_REG",              FALSE},
     {"INFRASYS_DRAMC_NAO_REGION_REG",           FALSE},
     {"INFRASYS_TRNG",                           FALSE},
     {"DEGBUG CORESIGHT",                        FALSE},
     {"DMA",                                     FALSE},
     {"AUXADC",                                  FALSE},
     {"UART0",                                   FALSE},
     {"UART1",                                   FALSE},
     {"UART2",                                   FALSE},
     {"UART3",                                   FALSE},
     {"PWM",                                     FALSE},
     {"I2C0",                                    FALSE},
     {"I2C1",                                    FALSE},
     {"I2C2",                                    FALSE},
     {"SPI0",                                    FALSE},
     {"PTP",                                     FALSE},
     {"BTIF",                                    FALSE},
     {"NFI",                                     FALSE},
     {"NFI_ECC",                                 FALSE},
     {"NLI_ARBITER",                             FALSE},
     {"GCPU",                                    FALSE},
     {"GCPU",                                    FALSE},
     {"GCPU_MMU",                                FALSE},
     {"I2C3",                                    FALSE},
     {"USB0",                                    FALSE},     
     {"USB1",                                    FALSE},    // Need additional mapping: Violation Index -> 64 
     {"USBSIF",                                  FALSE},    // Need additional mapping: Violation Index -> 58
     {"AUDIO",                                   FALSE},    // Need additional mapping: Violation Index -> 59
     {"MSDC0",                                   FALSE},    // Need additional mapping: Violation Index -> 60
     {"MSDC1",                                   FALSE},    // Need additional mapping: Violation Index -> 61
     {"MSDC2",                                   FALSE},    // Need additional mapping: Violation Index -> 62
     {"WCN_AHB_SLAVE",                           FALSE},    // Need additional mapping: Violation Index -> 63
     {"NIC_ETHER_WRAP",                          FALSE},
     {"IMGSYS_CONFIG",                           FALSE},
     {"IMGSYS_SMI_LARB2",                        FALSE},
     {"IMGSYS_CAM0",                             FALSE},
     {"IMGSYS_CAM1",                             FALSE},
     {"IMGSYS_SENINF",                           FALSE},
     {"IMGSYS_VENC",                             FALSE},
     {"IMGSYS_JPGENC",                           FALSE},
     {"IMGSYS_MIPI_RX",                          FALSE},
     {"G3D_CONFIG",                              FALSE},
     {"MALI",                                    FALSE},
     {"MMSYS_CONFIG",                            FALSE},
     {"MDP_RDMA",                                FALSE},
     {"MDP_RSZ0",                                FALSE},
     {"MDP_RSZ1",                                FALSE},
     {"MDP_WDMA",                                FALSE},
     {"MDP_WROT",                                FALSE},
     {"MDP_TDSHP",                               FALSE},
     {"DISP_OVL",                                FALSE},
     {"DISP_RDMA",                               FALSE},
     {"DISP_WDMA",                               FALSE},
     {"DISP_BLS",                                FALSE},
     {"DISP_COLOR",                              FALSE},
     {"DSI",                                     FALSE},
     {"DPI",                                     FALSE},
     {"MM_MUTEX",                                FALSE},
     {"MM_CMDQ",                                 FALSE},
     {"SMI_LARB0",                               FALSE},
     {"SMI_COMMON",                              FALSE},
     {"DISP_RDMA1",                              FALSE},
     {"DISP_UFOE",                               FALSE},
     {"DPI1",                                    FALSE},
     {"HDMI",                                    FALSE},
     {"LVDS",                                    FALSE},
     {"TVE",                                     FALSE},
     {"VDECSYS_CONFIGURATION",                   FALSE},
     {"VDECSYS_SMI_LARB1",                       FALSE},
     {"VDEC",                                    FALSE},
     {NULL,                                      FALSE},
  };

 
 
 
#define SET_SINGLE_MODULE(apcnum, domnum, index, module, permission_control)     \
 {                                                                               \
     mt65xx_reg_sync_writel(readl(DEVAPC##apcnum##_D##domnum##_APC_##index) & ~(0x3 << (2 * module)), DEVAPC##apcnum##_D##domnum##_APC_##index); \
     mt65xx_reg_sync_writel(readl(DEVAPC##apcnum##_D##domnum##_APC_##index) | (permission_control << (2 * module)),DEVAPC##apcnum##_D##domnum##_APC_##index); \
 }                                                                               \
 
#define UNMASK_SINGLE_MODULE_IRQ(apcnum, domnum, module_index)                  \
 {                                                                               \
     mt65xx_reg_sync_writel(readl(DEVAPC##apcnum##_D##domnum##_VIO_MASK) & ~(module_index),      \
         DEVAPC##apcnum##_D##domnum##_VIO_MASK);                                 \
 }                                                                               \
 
#define CLEAR_SINGLE_VIO_STA(apcnum, domnum, module_index)                     \
 {                                                                               \
     mt65xx_reg_sync_writel(readl(DEVAPC##apcnum##_D##domnum##_VIO_STA) | (module_index),        \
         DEVAPC##apcnum##_D##domnum##_VIO_STA);                                  \
 }                                                                               \

 
