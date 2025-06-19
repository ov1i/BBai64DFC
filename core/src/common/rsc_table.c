#include <stddef.h>
#include "rsc_table.h"

#define RSC_CARVEOUT_SIZE  (0x80000U)

const struct my_resource_table resourceTable
__attribute__((section(".resource_table"), aligned(4096), used)) = {
    1,  // version
    0,  // number of entries
    { 0, 0 },  // reserved
    { }  // no entries
};
