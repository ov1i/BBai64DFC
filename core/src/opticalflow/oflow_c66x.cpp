#include "oflow_c66x.h"
#include <ti/csl/arch/csl_arch.h>  // For core ID detection on TI SoCs
#include <stdint.h>
#include <math.h>
#include <c6x.h>

void compute_optical_flow_enh(uint8_t* frame1, uint8_t* frame2, FlowOutput* out, int y_start, int y_end) {
    const int width = FRAME_WIDTH;
    const int height = FRAME_HEIGHT;
    const int window = 2;
    const int step = 1;
    out->size = 0;

    const uint32_t ones = 0x01010101; // for dotp4h summing 4 bytes

    for (int y = y_start + window; y < y_end - window - 1; y += step) {
        for (int x = window; x < width - window - 9; x += 4) {
            for (int i = 0; i < 4; ++i) {
                int idx = x + i;

                uint8_t f1p   = frame1[y * width + idx];
                uint8_t f1p1  = frame1[y * width + idx + 1];
                uint8_t f2p   = frame2[y * width + idx];
                uint8_t f2p1  = frame2[y * width + idx + 1];

                uint8_t f1d   = frame1[(y + 1) * width + idx];
                uint8_t f1d1  = frame1[(y + 1) * width + idx + 1];
                uint8_t f2d   = frame2[(y + 1) * width + idx];
                uint8_t f2d1  = frame2[(y + 1) * width + idx + 1];

                uint32_t pos = _pack4(f2p1, f2d1, f1p1, f1d1);
                uint32_t neg = _pack4(f2p,  f2d,  f1p,  f1d);

                int Ix_sum = _dotp4h(_sub4(pos, neg), ones);

                pos = _pack4(f2p1, f2p, f1p1, f1p);
                neg = _pack4(f2d1, f2d, f1d1, f1d);
                int Iy_sum = _dotp4h(_sub4(pos, neg), ones);

                pos = _pack4(f2p, f2p1, f2d, f2d1);
                neg = _pack4(f1p, f1p1, f1d, f1d1);
                int It_sum = _dotp4h(_sub4(pos, neg), ones);

                float Ix = 0.25f * Ix_sum;
                float Iy = 0.25f * Iy_sum;
                float It = 0.25f * It_sum;

                float mag = Ix * Ix + Iy * Iy;
                if (mag < 1e-5f) continue;

                float scale = fabsf(It) / fast_sqrtf(mag);
                float dx = Ix * scale;
                float dy = Iy * scale;

                if (dx > LOWER_THRESHOLD && dx < UPPER_THRESHOLD &&
                    dy > LOWER_THRESHOLD && dy < UPPER_THRESHOLD &&
                    out->size < MAX_FEATURES) {
                    out->features[out->size].x1 = idx;
                    out->features[out->size].y1 = y;
                    out->features[out->size].x2 = idx + (int16_t)dx;
                    out->features[out->size].y2 = y + (int16_t)dy;
                    out->size++;
                }
            }
        }
    }
}
