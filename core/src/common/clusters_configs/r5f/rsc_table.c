#include <stddef.h>
#include <rsc_table_types.h>
#include "rsc_table.h"

const struct my_resource_table resourceTable
__attribute__((section(".resource_table"), aligned(4096), used)) = {
    .base = {
        .ver = 1,
        .num = 0,
        .reserved = { 0, 0 }
    },
    .offset = {
        0
        // offsetof(struct my_resource_table, carveout_shared_full)
    },
    {

    }
};
