#ifndef __PARTITION_DEFINE_H__
#define __PARTITION_DEFINE_H__





#ifndef CONFIG_MTK_EMMC_SUPPORT
extern int get_part_num_nand();

#define PART_NUM			get_part_num_nand()

#ifndef RAND_START_ADDR
#define RAND_START_ADDR   1024
#endif

#else
extern int PART_NUM;
#endif

#define PART_MAX_COUNT			 40

#define WRITE_SIZE_Byte		512
typedef enum  {
	EMMC = 1,
	NAND = 2,
} dev_type;

typedef enum {
	EMMC_PART_UNKNOWN=0
	,EMMC_PART_BOOT1
	,EMMC_PART_BOOT2
	,EMMC_PART_RPMB
	,EMMC_PART_GP1
	,EMMC_PART_GP2
	,EMMC_PART_GP3
	,EMMC_PART_GP4
	,EMMC_PART_USER
	,EMMC_PART_END
} Region;

struct excel_info{
	char * name;
	unsigned long long size;
	unsigned long long start_address;
	dev_type type ;
	unsigned int partition_idx;
	Region region;
};

extern struct excel_info PartInfo[PART_MAX_COUNT];

#endif
