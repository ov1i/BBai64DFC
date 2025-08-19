#ifndef ISP_H
#define ISP_H

#include "common/types/data_types.h"

constexpr uint16  width  = 640U;
constexpr uint16  height = 480U;
constexpr uint8   fps    = 90U;

constexpr uint64  baseAddr = 0xAA000000ULL;

constexpr uint8   kV4L2BufferCount = 2U;

enum : uint8 { 
    IMREADY = 1,
    IMREQUEST = 2,
};

enum : uint8 {
    IMPREV = 0,
    IMCURR = 1,
};

typedef struct __attribute__((aligned(64))) {
    uint64 ts_ns;       // timestamp
    uint32 width;       // image width
    uint32 height;      // image height
    uint32 stride;      // bytes per row
    uint32 size;        // size in bytes (width*height)
    uint32 seqLock;     // locks shared mem
    float64 duration;   // processing duration
    uint32 padding;     // header aligned
} ImageHeader_t;

typedef struct {
    void*  start{nullptr};
    size_t length{0};
} V4L2Buf_t;

#pragma pack(push,1)
typedef struct {
    uint8  type;
    uint16 padding;
    uint32 seqLock;
    uint64 ts_ns;
} MsgFrameReady_t;
#pragma pack(pop)

inline constexpr size_t getImageDataSize() { 
    return static_cast<size_t>(width) * static_cast<size_t>(height); 
}

inline constexpr size_t getImagePortSize() { 
    return sizeof(ImageHeader_t) + getImageDataSize(); 
}

inline constexpr uint64 memswper(uint8 idx) {
    return baseAddr + (idx == IMPREV ? 0ULL : static_cast<uint64>(getImagePortSize()));
}

#endif // ISP_H
