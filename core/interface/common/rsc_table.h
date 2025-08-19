#ifndef RSC_TABLE_H
#define RSC_TABLE_H

#include <stdint.h>
#include <rsc_table_types.h>
#include <data_types.h>

/* We keep this here just to clarify the main data type for our resource table */
struct my_resource_table {
    struct resource_table_base  base;
    uint32                      offset[1];
    // struct fw_rsc_carveout      carveout_shared_full;
};


#endif // RSC_TABLE_H
