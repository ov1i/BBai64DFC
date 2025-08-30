#include <stddef.h>
#include <string.h>
#include "rsc_table.h"

#define VRING_ALIGN  (4096U)
#define VRING_NUM    (256U)

// macro wrapper for offsetof
#ifndef OFFSETOF
#define OFFSETOF(type, member) ((uint32)offsetof(type, member)) 
#endif

static void setName(uint8 dst[32], const char *name)
{
    size_t n = strlen(name);
    if (n > 31) n = 31;
    memset(dst, 0, 32);
    memcpy(dst, name, n);
}

const struct my_resource_table resourceTable __attribute__((section(".resource_table"), aligned(4096), used)) = {
    .base = {
        .ver = 1U,
        .num = 1U,
        .reserved = { 0U, 0U }
    },

    .offset = {
        OFFSETOF(struct my_resource_table, vdev_rpmsg),
    },

    .vdev_rpmsg = {
        .type          = RSC_VDEV,
        .id            = VIRTIO_ID_RPMSG,
        .notifyid      = 0U,             /* host will fill */
        .dfeatures     = RPMSG_F_NS,     /* advertise name service */
        .gfeatures     = 0U,
        .config_len    = 0U,
        .status        = 0U,
        .num_of_vrings = 2U,
        .reserved      = {0U, 0U},
    },
    .vdev_vring0 = {
        .da       = 0U,                  /* let host place in IPC region */
        .align    = VRING_ALIGN,
        .num      = VRING_NUM,
        .notifyid = 0U,                  /* host will fill */
    },
    .vdev_vring1 = {
        .da       = 0U,                  /* let host place in IPC region */
        .align    = VRING_ALIGN,
        .num      = VRING_NUM,
        .notifyid = 0U,                  /* host will fill */
    }
};


__attribute__((constructor)) static void rsc_ctor_fill_names(void) {
    
    (void)setName; /* suppress unused warning for the const */
}
