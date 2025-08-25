#ifndef SHARED_TYPES_H
#define SHARED_TYPES_H

#include "data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

const uint32 OF_MAGIC = 0x4F464C4FU; // sanity check via magic no.
const uint32 width = 640u;
const uint32 height = 480u;
const uint64 baseAddr = 0xAB000000ull;

enum { 
  IMREADY    = 1, 
  IMREQUEST  = 2, 
  IMCONSUMED = 3,
};
enum { 
  IMPREV = 0,
  IMCURR = 1, 
};

typedef struct __attribute__((aligned(64))) {
  uint64 ts_ns;
  uint32 width;
  uint32 height;
  uint32 stride;
  uint32 size;
  uint32 seqLock;
  float64 duration;
  uint32 padding;
} DFC_t_ImageHeader;

typedef struct __attribute__((packed)) {
  uint8  type;       // IMREADY
  uint8  slot;       // buffer slot to which write will happen
  uint16 _pad;
  uint32 seqLock;    // even=stable can read, odd=write in progress
  uint64 ts_ns;
} DFC_t_MsgISP;

typedef struct __attribute__((packed)) {
  uint32 magic;        // 'OFLO'
  uint64 ts_prev_ns;
  uint64 ts_curr_ns;
  float64 u_px_per_s;   // +right (ghfo pxl/sec)
  float64 v_px_per_s;   // +down (gvfo pxl/sec)
  float64 quality;      // 0..1 fraction of valid tracks
  uint16 width;
  uint16 height;
} DFC_t_MsgOpticalFlow;

typedef struct {
  float64 u_px_s;
  float64 v_px_s;
  float64 quality;    // 0..1 inliers
  bool valid;
} DFC_t_OFlow_Output;

// WE KEEP THEM HERE In order to prevent usesless includes, will act as duplicates but same values
struct DFC_t_ProcIDs {
  static constexpr uint16 A72  = 0U;
  static constexpr uint16 R5F  = 3U; // MAIN Island pointing to the first half but will be the full MCU2 since we in lockstep
  static constexpr uint16 C66  = 7U;  
};

static inline uint32 getImgDataSize(uint32 w, uint32 h) { return w*h; }
static inline uint64 getImgePortSize(uint32 w, uint32 h) { return sizeof(DFC_t_ImageHeader) + getImgDataSize(w,h); }
static inline uint64 getMemAddr(uint8 idx, uint32 w, uint32 h) { return baseAddr + (idx==IMPREV ? 0ull : getImgePortSize(w,h)); }

#ifdef __cplusplus
}
#endif
#endif

