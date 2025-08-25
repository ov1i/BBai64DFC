#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H
#include <stdint.h>
#include "shared_types.h"

struct DFC_t_OFlow_Params {
  sint32 cell_w     = 32;
  sint32 cell_h     = 32;
  sint32 min_tracks = 20;
  float64 texture_det= 1e6;   // Gxx*Gyy - Gxy^2 (tune)
  float64 mad_gate   = 3.0;   // outlier gate (MAD)
};

// global px/s
DFC_t_OFlow_Output get_global_flow(const uint8* prevY,
                                    const uint8* currY,
                                    sint32 width, sint32 height, sint32 stride,
                                    float64 dt_s,
                                    const DFC_t_OFlow_Params& params);

#endif
