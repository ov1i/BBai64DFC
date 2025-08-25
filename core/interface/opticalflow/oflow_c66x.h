#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H
#include <stdint.h>
#include "shared_types.h"

struct DFC_t_OFlow_Params {
  sint32 cell_w             = 32;
  sint32 cell_h             = 32;
  sint32 min_tracks         = 20;
  float64 texture_det       = 1e6;   // Gxx*Gyy - Gxy^2 (tune)
  float64 mad_gate          = 3.0;   // outlier gate (MAD)
  float64 zero_th_pxs       = 5.0;   // px/s th considered to be near 0
  float64 zero_maj_min      = 0.30;  // majority of cells near zero (meaning they have no movement)
  float64 quad_spread_gain  = 3.0;   // how much quadrant disagreement we tolerate
  float64 med_th_pxs        = 20.0;  // ignore tiny medians; only care if |median| is big
  float64 qualityFailed     = 0.02;  // quality when we signal false movement < .05 needed
};

// global px/s
DFC_t_OFlow_Output get_global_flow(const uint8* prevY, const uint8* currY,
                                    sint32 width, sint32 height, sint32 stride,
                                    float64 dt_s,
                                    const DFC_t_OFlow_Params& params);

#endif
