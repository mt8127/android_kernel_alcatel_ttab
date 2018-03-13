#ifdef CONFIG_MTK_PMIC_MT6397
#ifndef MTK_ALPS_BOX_SUPPORT
  #include "upmu_common_mt6397.c"
#endif
#else
#include "upmu_common_mt6323.c"
#endif