#include <string.h>
#include <stdlib.h>

#include <c6x.h>
#include "common/types/shared_types.h" 
#include "opticalflow/oflow_c66x.h"
#include "comms/rpmsg_helper.hpp"

static inline volatile DFC_t_ImageHeader* getpImageHeader(uint8 idx) {
    return (volatile DFC_t_ImageHeader*)(uintptr_t)getpMemAddr(idx, width, height);
}
static inline volatile uint8* getpImageData(uint8 idx) {
    return (volatile uint8*)getpImageHeader(idx) + sizeof(DFC_t_ImageHeader);
}

static void copyImageAtomically(uint8 slot, uint8* dst, DFC_t_ImageHeader* outH) {
    volatile DFC_t_ImageHeader* pImageHeader = getpImageHeader(slot);
    volatile uint8* pImageData = getpImageData(slot);
    const uint32 imageSize = getImgDataSize(width, height);

    for(;;) {
        uint32 seqLock0 = pImageHeader->seqLock;
        _mfence();
        if(seqLock0 & 1u) {
            continue; // writer in progress
        }

        memcpy(dst, (const void*)pImageData, imageSize);
        _mfence();

        DFC_t_ImageHeader tmp{};
        memcpy(&tmp, (const void*)pImageHeader, sizeof(tmp));
        _mfence();
        uint32 seqLock1 = tmp.seqLock;

        if(seqLock0 == seqLock1 && !(seqLock1 & 1u)) {
            *outH = tmp; // stable image+header snapshot
            return;
        }
    }
}

int main(void) {
    C_RPMsgHelper::init();

    C_RPMsgHelper link_from_A72;  // recv IMREADY from A72
    C_RPMsgHelper link_to_R5F;    // send DFC_t_MsgOpticalFlow to R5F

    // A72 -> C66x
    link_from_A72.open(DFC_t_ProcIDs::A72, "a72_to_c66x", nullptr);

    // C66x -> R5F
    link_to_R5F.open(IPC_R5FSS0_0, "c66x_to_r5f", "r5f_to_c66x");
    for(uint8 i = 0; i < 50 && !link_to_R5F.isDstReady(); ++i) {
      link_to_R5F.tryResolve(100);
    }

    const uint32 imageSize = getImgDataSize(width, height);
    uint8* prevBuffer = (uint8*)malloc(imageSize);
    uint8* currBuffer = (uint8*)malloc(imageSize);
    if(!prevBuffer || !currBuffer) {
        for(;;) {
           // NO-OP HANG alloc erro 
        }
    }

    uint64 prev_ts_ns = 0ull;
    bool firstCycle = true;

    DFC_t_OFlow_Params params{};
    params.cell_w      = 32;
    params.cell_h      = 32;
    params.min_tracks  = 20;
    params.texture_det = 1e6;
    params.mad_gate    = 3.0;

    for(;;) {
        DFC_t_MsgISP imready{};
        int n = link_from_A72.recv(&imready, (sint32)sizeof(imready), -1);
        if(n != (sint32)sizeof(imready) || imready.type != IMREADY) {
            continue; // block till frame available
        }

        const uint8 currentSlot = (uint8)(imready.slot & 1u);
        DFC_t_ImageHeader currentImageHeader{};
        copyImageAtomically(currentSlot, currBuffer, &currentImageHeader);

        // sanity check (always same)
        if(currentImageHeader.width  != width  || currentImageHeader.height != height || currentImageHeader.stride != width  || currentImageHeader.size != imageSize) {
            uint8* tmp = prevBuffer; prevBuffer = currBuffer; currBuffer = tmp;
            prev_ts_ns = currentImageHeader.ts_ns;
            firstCycle  = false;
            continue;
        }

        // in case of first cycle no processing for u G 
        if(firstCycle) {
            memcpy(prevBuffer, currBuffer, imageSize);
            prev_ts_ns = currentImageHeader.ts_ns;
            firstCycle = false;
            continue;
        }

        // synch sanity check
        if(!(currentImageHeader.ts_ns > prev_ts_ns)) {
            continue;
        }

        // finally got to something
        const float64 dt_s = (float64)(currentImageHeader.ts_ns - prev_ts_ns) / 1e9;
        const DFC_t_OFlow_Output flow =  get_global_flow((const uint8*)prevBuffer, (const uint8*)currBuffer, (sint32)width, (sint32)height, (sint32)width, dt_s, params);

        // Prepare msg for R5F
        DFC_t_MsgOpticalFlow msg{};
        msg.magic       = DFC_FLOW_RAW_MAGIC;
        msg.ts_prev_ns  = prev_ts_ns;
        msg.ts_curr_ns  = currentImageHeader.ts_ns;
        msg.u_px_per_s  = flow.valid ? flow.u_px_per_s : 0.0;
        msg.v_px_per_s  = flow.valid ? flow.v_px_per_s : 0.0;
        msg.quality     = flow.valid ? flow.quality : 0.0;
        msg.width       = (uint16)width;
        msg.height      = (uint16)height;

        (void)link_to_R5F.send(&msg, sizeof(msg)); // send msg to R5F expect no response

        // spin me baby right round right round (swap buffers)
        uint8* tmp = prevBuffer; 
        prevBuffer = currBuffer; 
        currBuffer = tmp;
        prev_ts_ns = currentImageHeader.ts_ns;
        firstCycle  = false;
    }
}
