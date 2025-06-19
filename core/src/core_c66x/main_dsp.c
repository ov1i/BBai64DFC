#include <ti/csl/arch/csl_arch.h>  // For core ID detection on TI SoCs
#include <stdint.h>
#include <math.h>
#include "oflow.h"



void main() {
    uint32_t core_id = CSL_chipReadReg(CSL_CHIP_DNUM); // 0 for C66x_0, 1 for C66x_1

    uint8_t* frame1 = FRAME1_ADDR;
    uint8_t* frame2 = FRAME2_ADDR;

    FlowOutput* out = &FLOW_OUT_BASE[core_id];

    int y_start = (core_id == 0) ? 0 : (FRAME_HEIGHT / 2);
    int y_end   = (core_id == 0) ? (FRAME_HEIGHT / 2) : FRAME_HEIGHT;

    compute_optical_flow_split(frame1, frame2, out, y_start, y_end);

    while (1) { }
}
