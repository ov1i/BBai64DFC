#ifndef RSC_TABLE_TYPES_H
#define RSC_TABLE_TYPES_H

#include <data_types.h>

/* PRIVATE CORE MEM 16MB (maybe we ll make it bigger if we don't switch to two cores)*/
#define MCU1_0_DMA_REGION_PA       0xa0000000U
#define MCU1_0_DMA_REGION_SIZE     0x00100000U  // 1MB
#define MCU1_0_CORE_REGION_PA      0xa0100000U
#define MCU1_0_CORE_REGION_SIZE    0x00f00000U  // 15MB

#define MCU1_1_DMA_REGION_PA       0xa1000000U
#define MCU1_1_DMA_REGION_SIZE     0x00100000U  // 1MB
#define MCU1_1_CORE_REGION_PA      0xa1100000U
#define MCU1_1_CORE_REGION_SIZE    0x00f00000U  // 15MB

#define MCU2_0_DMA_REGION_PA       0xa2000000U
#define MCU2_0_DMA_REGION_SIZE     0x00100000U  // 1MB
#define MCU2_0_CORE_REGION_PA      0xa2100000U
#define MCU2_0_CORE_REGION_SIZE    0x00f00000U  // 15MB

#define MCU2_1_DMA_REGION_PA       0xa3000000U
#define MCU2_1_DMA_REGION_SIZE     0x00100000U  // 1MB
#define MCU2_1_CORE_REGION_PA      0xa3100000U
#define MCU2_1_CORE_REGION_SIZE    0x00f00000U  // 15MB

#define MCU3_0_DMA_REGION_PA       0xa4000000U
#define MCU3_0_DMA_REGION_SIZE     0x00100000U  // 1MB
#define MCU3_0_CORE_REGION_PA      0xa4100000U
#define MCU3_0_CORE_REGION_SIZE    0x00f00000U  // 15MB

#define MCU3_1_DMA_REGION_PA       0xa5000000U
#define MCU3_1_DMA_REGION_SIZE     0x00100000U  // 1MB
#define MCU3_1_CORE_REGION_PA      0xa5100000U
#define MCU3_1_CORE_REGION_SIZE    0x00f00000U  // 15MB

#define C66x_1_DMA_REGION_PA       0xa6000000U
#define C66x_1_DMA_REGION_SIZE     0x00100000U  // 1MB
#define C66x_1_CORE_REGION_PA      0xa6100000U
#define C66x_1_CORE_REGION_SIZE    0x00f00000U  // 15MB

#define C66x_2_DMA_REGION_PA       0xa7000000U
#define C66x_2_DMA_REGION_SIZE     0x00100000U  // 1MB
#define C66x_2_CORE_REGION_PA      0xa7100000U
#define C66x_2_CORE_REGION_SIZE    0x00f00000U  // 15MB

#define C7x_DMA_REGION_PA          0xa8000000U
#define C7x_DMA_REGION_SIZE        0x00100000U  // 1MB
#define C7x_CORE_REGION_PA         0xa8100000U
#define C7x_CORE_REGION_SIZE       0x00f00000U  // 15MB

#define SHARED_REGION_PA           0xaa000000U
#define SHARED_REGION_SIZE         0x01c00000U  // 28MB

struct resource_table_base {
    uint32 ver;
    uint32 num;
    uint32 reserved[2];
};

enum fw_resource_type {
	RSC_CARVEOUT	    = 0,        //THE ONLY ONE USED FOR ALLIGNED MEM BUFF
	RSC_DEVMEM		    = 1,        //TO BE IMPLEMENTED (DO NOT USE)
	RSC_TRACE		    = 2,        //TO BE IMPLEMENTED (DO NOT USE)
	RSC_VDEV		    = 3,        //TO BE IMPLEMENTED (DO NOT USE)
	RSC_LAST		    = 4,        //TO BE IMPLEMENTED (DO NOT USE)
	RSC_VENDOR_START	= 128,      //TO BE IMPLEMENTED (DO NOT USE)
	RSC_VENDOR_END	    = 512       //TO BE IMPLEMENTED (DO NOT USE)
};

struct fw_rsc_carveout {
    uint32 type;
    uint32 da;
    uint32 pa;
    uint32 len;
    uint32 flags;
    uint32 reserved;
    uint8 name[32];
};


#endif
