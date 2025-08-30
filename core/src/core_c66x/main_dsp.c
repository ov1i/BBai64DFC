#include "comms/RPMsg_helper_c.h"
#include "opticalflow/oflow_c66x.h"
#include <string.h>
#include <stdlib.h>
#include <c6x.h>

static inline volatile DFC_t_ImageHeader* getpImageHeader(uint8 idx) {
    return (volatile DFC_t_ImageHeader*)(uintptr_t)getMemAddr(idx, IMG_W, IMG_H);
}
static inline volatile uint8* getpImageData(uint8 idx) {
    return (volatile uint8*)getpImageHeader(idx) + sizeof(DFC_t_ImageHeader);
}

static void copyImageAtomically(uint8 slot, uint8* dst, DFC_t_ImageHeader* outH) {
    volatile DFC_t_ImageHeader* pImageHeader = getpImageHeader(slot);
    volatile uint8* pImageData = getpImageData(slot);
    const uint32 imageSize = getImgDataSize(IMG_W, IMG_H);

    for(;;) {
        uint32 seqLock0 = pImageHeader->seqLock;
        _mfence();
        if(seqLock0 & 1u) {
            continue; // writer in progress
        }

        memcpy(dst, (const void*)pImageData, imageSize);
        _mfence();

        DFC_t_ImageHeader tmp;
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
    rpmsg_init();

    DFC_t_RPMsgWrapper* link_from_A72 = rpmsg_createWrapper();  // recv IMREADY from A72
    DFC_t_RPMsgWrapper* link_to_R5F = rpmsg_createWrapper();    // send DFC_t_MsgOpticalFlow to R5F

    // A72 -> C66x
    rpmsg_open(link_from_A72, A72_procID, "c66x_from_a72", NULL);

    // C66x -> R5F
    rpmsg_open(link_to_R5F, R5F_procID, "c66x_to_r5f", "r5f_from_c66x");
    uint8 i;
    for(i = 0; i < 50 && rpmsg_isDstReady(link_to_R5F) == 0; ++i) {
      rpmsg_tryResolve(link_to_R5F, 100);
    }

    const uint32 imageSize = getImgDataSize(IMG_W, IMG_H);
    uint8* prevBuffer = (uint8*)malloc(imageSize);
    uint8* currBuffer = (uint8*)malloc(imageSize);
    if(!prevBuffer || !currBuffer) {
        for(;;) {
           // NO-OP HANG alloc erro 
        }
    }

    uint64 prev_ts_ns = 0ull;
    uint8 firstCycle = 1U;

    DFC_t_OFlow_Params params;
    DFC_init_OFlow_Params(&params);

    for(;;) {
        DFC_t_MsgISP imready;
        sint32 n = rpmsg_recv(link_from_A72, &imready, (sint32)sizeof(imready), -1);
        if(n != (sint32)sizeof(imready) || imready.type != IMREADY) {
            continue; // block till frame available
        }

        const uint8 currentSlot = (uint8)(imready.slot & 1u);
        DFC_t_ImageHeader currentImageHeader;
        copyImageAtomically(currentSlot, currBuffer, &currentImageHeader);

        // sanity check (always same)
        if(currentImageHeader.width  != IMG_W  || currentImageHeader.height != IMG_H || currentImageHeader.stride != IMG_W  || currentImageHeader.size != imageSize) {
            uint8* tmp = prevBuffer; prevBuffer = currBuffer; currBuffer = tmp;
            prev_ts_ns = currentImageHeader.ts_ns;
            firstCycle  = 0U;
            continue;
        }

        // in case of first cycle no processing for u G 
        if(firstCycle == 1U) {
            memcpy(prevBuffer, currBuffer, imageSize);
            prev_ts_ns = currentImageHeader.ts_ns;
            firstCycle = 0U;
            continue;
        }

        // synch sanity check
        if(!(currentImageHeader.ts_ns > prev_ts_ns)) {
            continue;
        }

        // finally got to something
        const float64 dt_s = (float64)(currentImageHeader.ts_ns - prev_ts_ns) / 1e9;
        const DFC_t_OFlow_Output flow =  get_global_flow((const uint8*)prevBuffer, (const uint8*)currBuffer, (sint32)IMG_W, (sint32)IMG_H, (sint32)IMG_W, dt_s, &params);

        // Prepare msg for R5F
        DFC_t_MsgOpticalFlow msg;
        msg.magic       = DFC_FLOW_RAW_MAGIC;
        msg.ts_prev_ns  = prev_ts_ns;
        msg.ts_curr_ns  = currentImageHeader.ts_ns;
        msg.u_px_per_s  = flow.valid ? flow.u_px_per_s : 0.0;
        msg.v_px_per_s  = flow.valid ? flow.v_px_per_s : 0.0;
        msg.quality     = flow.valid ? flow.quality : 0.0;
        msg.width       = (uint16)IMG_W;
        msg.height      = (uint16)IMG_H;
        
        if(rpmsg_isDstReady(link_to_R5F) == 1) {
            (void)rpmsg_send(link_to_R5F, &msg, sizeof(msg)); // send msg to R5F expect no response
        } else {
            static uint32 tick = 0;
            if((tick++ & 0x3F) == 0) { 
                (void)rpmsg_tryResolve(link_to_R5F, 100); 
            }
            // drop of frame
        }
        // spin me baby right round right round (swap buffers)
        uint8* tmp = prevBuffer; 
        prevBuffer = currBuffer; 
        currBuffer = tmp;
        prev_ts_ns = currentImageHeader.ts_ns;
        firstCycle  = 0U;
    }

    // WE COMPILE ON C89 -> this gets flaged as unreachable, so we comment out in order to not switch to a newer version
    // rpmsg_destroyWrapper(link_to_R5F);
    // rpmsg_destroyWrapper(link_from_A72);
    // return 0;
}
