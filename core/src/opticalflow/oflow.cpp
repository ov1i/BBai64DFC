#include "oflow.h"
#include <math.h>

void compute_optical_flow_split(uint8_t* frame1, uint8_t* frame2, FlowOutput* out,  uint16_t y_start, uint16_t y_end) {
    const int width = FRAME_WIDTH;
    const int height = FRAME_HEIGHT;
    const int window = 2;
    const int step = 1;
    out->size = 0;

    for (int y = y_start + window; y < y_end - window - 1; y += step) {
        for (int x = window; x < width - window - 1; x += step) {
            int idx = y * width + x;

            float Ix = 0.25f * (frame2[idx + 1] + frame2[idx + width + 1] + frame1[idx + 1] + frame1[idx + width + 1])
                     - 0.25f * (frame2[idx] + frame2[idx + width] + frame1[idx] + frame1[idx + width]);

            float Iy = 0.25f * (frame2[idx + 1] + frame2[idx] + frame1[idx + 1] + frame1[idx])
                     - 0.25f * (frame2[idx + width + 1] + frame2[idx + width] + frame1[idx + width + 1] + frame1[idx + width]);

            float It = 0.25f * (frame2[idx] + frame2[idx + 1] + frame2[idx + width] + frame2[idx + width + 1])
                     - 0.25f * (frame1[idx] + frame1[idx + 1] + frame1[idx + width] + frame1[idx + width + 1]);

            float mag = Ix * Ix + Iy * Iy;
            if (mag < 1e-5f) continue;

            float scale = fabsf(It) / fast_sqrtf(mag);
            float dx = Ix * scale;
            float dy = Iy * scale;

            if (dx > LOWER_THRESHOLD && dx < UPPER_THRESHOLD &&
                dy > LOWER_THRESHOLD && dy < UPPER_THRESHOLD &&
                out->size < MAX_FEATURES) {
                out->features[out->size].x1 = x;
                out->features[out->size].y1 = y;
                out->features[out->size].x2 = x + (int16_t)dx;
                out->features[out->size].y2 = y + (int16_t)dy;
                out->size++;
            }
        }
    }
}
