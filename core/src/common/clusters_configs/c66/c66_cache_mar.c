#ifdef _TMS320C6X

#include <ti/osal/osal.h>
#include <ti/osal/CacheP.h>
#include <data_types.h>
#include <clusters/memory_map_defines.inc>

/* To set C66x Cache MAR Registers */
void ipc_cacheMarInit(void)
{
    /* disable cache for non-cached sections */
    CacheP_setMar((void *)(IPC_SHARED_START), (uint32)IPC_SHARED_SIZE, CacheP_Mar_DISABLE);
    CacheP_setMar((void *)(VISION_TELE_START), (uint32)VISION_TELE_SIZE, CacheP_Mar_DISABLE);
#if defined(BUILD_C66X_1)
    CacheP_setMar((void *)(C66x1_IPC_DATA_BASE), (uint32)0x01000000, CacheP_Mar_DISABLE);
#endif
#if defined(BUILD_C66X_2)
    CacheP_setMar((void *)(C66x2_IPC_DATA_BASE), (uint32)0x01000000, CacheP_Mar_DISABLE);
#endif

}
#endif /* #ifdef _TMS320C6X */
