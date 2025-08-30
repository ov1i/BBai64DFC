#ifndef RSC_TABLE_H
#define RSC_TABLE_H

#include <stdint.h>
#include <rsc_table_types.h>
#include <data_types.h>

struct my_resource_table {
    struct resource_table_base base;
    uint32 offset[3];  /* we now have 3 resources */

    /* trace buffer */
    struct fw_rsc_trace        trace;

    /* vdev: virtio-rpmsg, with two vrings */
    struct fw_rsc_vdev        vdev_rpmsg;
    struct fw_rsc_vdev_vring  vdev_vring0;
    struct fw_rsc_vdev_vring  vdev_vring1;

    /* carveout: images + telemetry fixed block */
    struct fw_rsc_carveout    carveout_vision_tele;
};

#endif /* RSC_TABLE_H */
