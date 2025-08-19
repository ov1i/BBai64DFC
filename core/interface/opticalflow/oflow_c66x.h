#ifndef OPTICAL_FLOW_DSP_H
#define OPTICAL_FLOW_DSP_H

#include <ti/csl/arch/csl_arch.h> 
#include <stdint.h>
#include <math.h>
#include <c6x.h> 

#define FRAME_WIDTH   320
#define FRAME_HEIGHT  240
#define FRAME_SIZE    (FRAME_WIDTH * FRAME_HEIGHT)

// Feature output format: x1,y1 - x2,y2 (motion vector)
typedef struct {
    int16_t x1, y1;
    int16_t x2, y2;
} FlowFeature;

#define MAX_FEATURES 1024

typedef struct {
    FlowFeature features[MAX_FEATURES];
    uint16_t size;
} FlowOutput;

#define FRAME1_ADDR   ((uint8_t*)0x9C000000)
#define FRAME2_ADDR   ((uint8_t*)0x9C012000)
#define FLOW_OUT_ADDR ((FlowOutput*)0x9C024000)

/** FUNCTIONS */

static inline float fast_sqrt(float n) {
    float x = n;
    float y = 1;
    float eps = 0.000001f;
    while (x - y > eps) {
        x = (x + y) * 0.5f;
        y = n / x;
    }
    return x;
}

void compute_optical_flow_enh(uint8_t* frame1, uint8_t* frame2, FlowOutput* out,  uint16_t y_start, uint16_t y_end);

#endif
