#ifndef RSC_TABLE_TYPES_H
#define RSC_TABLE_TYPES_H

#include <data_types.h>

#define IPC_REGION_PA              0xAA000000U
#define IPC_REGION_SIZE            0x01000000U  /* 16 MiB */

#define VISU_TELE_REGION_PA        0xAB000000U
#define VISU_TELE_SIZE             0x00800000U  /* 8 MiB */

#define VIRTIO_ID_RPMSG            7U
#define RPMSG_F_NS                 (1U << 0)  /* name service feature */

struct resource_table_base {
    uint32 ver;
    uint32 num;      
    uint32 reserved[2];
};

enum fw_resource_type {
    RSC_CARVEOUT      = 0,
    RSC_DEVMEM        = 1,
    RSC_TRACE         = 2,
    RSC_VDEV          = 3,
    RSC_LAST          = 4,
    RSC_VENDOR_START  = 128,
    RSC_VENDOR_END    = 512
};

struct fw_rsc_carveout {
    uint32 type;     /* RSC_CARVEOUT */
    uint32 da;       /* device address (IPA) */
    uint32 pa;       /* physical address (optional, 0 = let host place) */
    uint32 len;      /* size in bytes */
    uint32 flags;    
    uint32 reserved; 
    uint8  name[32]; /* NULL-terminated */
};

struct fw_rsc_trace {
    uint32 type;     /* RSC_TRACE */
    uint32 da;       /* device address (0 = let host place) */
    uint32 len;      /* buffer size in bytes */
    uint32 reserved;
    uint8  name[32]; /* label shown by Linux, NULL-terminated */
};

// rpmsg
struct fw_rsc_vdev {
    uint32 type;           /* RSC_VDEV */
    uint32 id;             /* VIRTIO_ID_RPMSG */
    uint32 notifyid;       /* filled/used by host */
    uint32 dfeatures;      /* device features */
    uint32 gfeatures;      /* host-acked features (filled by host) */
    uint32 config_len;     /* cfg size (0 for rpmsg) */
    uint8  status;         /* set by host */
    uint8  num_of_vrings;  /* 2 */
    uint8  reserved[2];
};

// vring
struct fw_rsc_vdev_vring {
    uint32 da;       /* device address */
    uint32 align;    /* vring alignment  */
    uint32 num;      /* descriptors */
    uint32 notifyid; /* used by host */
};

#endif /* RSC_TABLE_TYPES_H */
