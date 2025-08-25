#ifndef VISION_SHARED_H
#define VISION_SHARED_H

#include <stdint.h>
#include <stddef.h>
#include "data_types.h"

static constexpr uint16  width  = 640U;
static constexpr uint16  height = 480U;
static constexpr uint8   fps    = 90U;

static constexpr uint64  baseAddr = 0xAB000000ULL;

static constexpr uint8   kV4L2BufferCount = 2U;
enum : uint8 { SLOT_PREV = 0, SLOT_CURR = 1 };

enum : uint8 { IMREADY = 1, IMREQUEST = 2, TELEM_READY = 3 };

typedef struct __attribute__((aligned(64))) {
    uint64  ts_ns;
    uint32  width;
    uint32  height;
    uint32  stride;
    uint32  size;
    uint32  seqLock;   // even=stable can read, odd=write in progress
    float64 duration;  // duration in seconds to convert + write
    uint32  padding;
} DFC_t_ImageHeader;

// V4L2 mmap buffer descriptor
typedef struct {
    void*  start;
    size_t length;
} DFC_t_V4L2Buf;

// informs c66x new frame ready for fetching
#pragma pack(push,1)
typedef struct {
    uint8  type;       // IMREADY
    uint8  slot;       // buffer slot to which write will happen
    uint16 _pad;
    uint32 seqLock;    // even=stable can read, odd=write in progress
    uint64 ts_ns;
} DFC_t_MsgFrameReady;

typedef struct {
    uint8  type;       // TELEM_READY
    uint8  _pad[3];
} DFC_t_MsgTelemReady;
#pragma pack(pop)

inline constexpr size_t getImageDataSize() {
    return static_cast<size_t>(width) * static_cast<size_t>(height);
}
inline constexpr size_t getImagePortSize() {
    return sizeof(DFC_t_ImageHeader) + getImageDataSize();
}
constexpr size_t align_up(size_t v, size_t a) { return (v + a - 1) & ~(a - 1); }

// Memory layout: [slot0][slot1][telemetry region starts here]
inline constexpr uint64 telem_offset = static_cast<uint64>(align_up(2 * getImagePortSize(), 64));
inline constexpr uint64 telem_addr = baseAddr + telem_offset;

// should we wrap telemetry in a seqlock envelope so readers get atomic snapshot ?
typedef struct __attribute__((packed,aligned(8))) {
    uint32 seqLock;     // -|-
    uint32 _pad;
    /// TODO: SHOULD BE UPDATED AFTER MAIN DONE
    uint64 timestamp_imu_1;
    uint64 timestamp_imu_2;
    float64 ax, ay, az;
    float64 gx, gy, gz;
    float64 mx, my, mz;
    float64 temp;
    uint8  mag_rdy;
    uint8  _pad2[7];  // align to 8
} DFC_t_TelemetryPacket;


struct __attribute__((packed)) DFC_t_UDPImgHeader {
    uint32 magic;       // 'IMG1' = 0x31474D49
    uint32 frame_id;    // increments per send
    uint64 ts_ns;       // slot timestamp
    uint16 width;
    uint16 height;
    uint8  idx;       // 0=prev, 1=curr
    uint16 chunk_idx;
    uint16 chunk_count;
};

// stable read via seqlock
static inline bool seqlock_read(const volatile uint32* seq) {
    uint32 s = __atomic_load_n(seq, __ATOMIC_ACQUIRE);
    return (s & 1u) == 0u ? true : false; // stable or not
}
static inline bool seqlock_read_retry(const volatile uint32* seq, uint32 beginVal) {
    uint32 endVal = __atomic_load_n(seq, __ATOMIC_ACQUIRE);
    return (beginVal != endVal); // retry if changed or smth occured
}

#endif // VISION_SHARED_H
