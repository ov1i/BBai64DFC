#ifndef RSC_TABLE_H
#define RSC_TABLE_H

#include <stdint.h>
#include <ti/drv/ipc/include/ipc_rsctypes.h>

struct my_resource_table {
    uint32_t ver;       // Version: always 1
    uint32_t num;       // Number of entries
    uint32_t reserved[2];
    uint32_t offset[1]; // Empty/minimal table = no actual resources
};

#endif // RSC_TABLE_H
