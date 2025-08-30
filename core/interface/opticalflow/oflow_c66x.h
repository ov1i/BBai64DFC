#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include <stdint.h>
#include <shared_types.h>

// Params DEF
typedef struct {
    sint32   cell_w;            /* default: 32 */
    sint32   cell_h;            /* default: 32 */
    sint32   min_tracks;        /* default: 20 */
    float64  texture_det;       /* default: 1e6  -> Gxx*Gyy - Gxy^2 (tune) */
    float64  mad_gate;          /* default: 3.0  -> outlier gate (MAD) */
    float64  zero_th_pxs;       /* default: 5.0  -> px/s threshold considered near 0 */
    float64  zero_maj_min;      /* default: 0.30 -> majority of cells near zero movement */
    float64  quad_spread_gain;  /* default: 3.0  -> tolerance for quadrant disagreement */
    float64  med_th_pxs;        /* default: 20.0 -> ignore tiny medians unless |median| is large */
    float64  qualityFailed;     /* default: 0.02 -> quality threshold for false movement */
} DFC_t_OFlow_Params;

// Initializes the params
static inline void DFC_init_OFlow_Params(DFC_t_OFlow_Params* params) {
    if (!params) return;
    params->cell_w          = 32;
    params->cell_h          = 32;
    params->min_tracks      = 20;
    params->texture_det     = 1e6;      // Gxx*Gyy - Gxy^2 (tune)
    params->mad_gate        = 3.0;      // outlier gate (MAD)
    params->zero_th_pxs     = 5.0;      // px/s th considered to be near 0
    params->zero_maj_min    = 0.30;     // majority of cells near zero (meaning they have no movement)
    params->quad_spread_gain= 3.0;      // how much quadrant disagreement we tolerate
    params->med_th_pxs      = 20.0;     // ignore tiny medians; only care if |median| is big
    params->qualityFailed   = 0.02;     // quality when we signal false movement < .05 needed
}

// global px/s
DFC_t_OFlow_Output get_global_flow(const uint8* prevY,
                                   const uint8* currY,
                                   sint32 width,
                                   sint32 height,
                                   sint32 stride,
                                   float64 dt_s,
                                   const DFC_t_OFlow_Params* params);

#endif /* OPTICAL_FLOW_H */
