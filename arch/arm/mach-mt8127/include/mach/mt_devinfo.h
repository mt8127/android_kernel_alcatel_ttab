#ifndef __MT_DEVINFO_H
#define __MT_DEVFINO_H

#include <linux/types.h>

#ifdef CONFIG_MTK_MTD_NAND
#include <mach/mt_partitioninfo.h>
#include <mach/nand_device_define.h>
#endif

/*device information data*/
#define ATAG_DEVINFO_DATA 0x41000804
#define ATAG_DEVINFO_DATA_SIZE 26

struct tag_devinfo_data {
    u32 devinfo_data[ATAG_DEVINFO_DATA_SIZE]; 	/* device information */
    u32 devinfo_data_size;                      /* device information size */
};

#endif
