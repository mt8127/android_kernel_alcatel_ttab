#ifndef _MT_PMIC_COMMON_H_
#define _MT_PMIC_COMMON_H_

#ifdef CONFIG_MTK_PMIC_MT6397
#include <mach/upmu_common_mt6397.h>
#else
#include <mach/upmu_common_mt6323.h>
#endif

#endif //_MT_PMIC_COMMON_H
