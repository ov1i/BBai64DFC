#ifndef VISION_SHARED_H
#define VISION_SHARED_H

#include <stdint.h>
#include <stddef.h>
#include "data_types.h"


static constexpr uint32 DFC_TELE_MAGIC = 0x54454C45u;

static constexpr uint16  width  = 640U;
static constexpr uint16  height = 480U;
static constexpr uint8   fps    = 90U;

static constexpr uint64  baseAddr = 0xAB000000ULL;

static constexpr uint8   kV4L2BufferCount = 2U;
enum : uint8 { SLOT_PREV = 0, SLOT_CURR = 1 };

enum : uint8 { IMREADY = 1, IMREQUEST = 2 };

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
#pragma pack(pop)

struct __attribute__((packed)) DFC_t_TelemetryPacket {
  // Header
  uint32 magic;     
  uint16 size;
  uint32 seq;

  // IMU / Mag (no timestamps)
  float32    ax, ay, az, gx, gy, gz;
  float32    mx, my, mz;
  float32    mag_adjustment[3];
  uint8  mag_rdy;
  uint8  _pad_mag[7];
  float32    imu_temp;

  // BMP280 (no timestamps)
  float32    baro_p_hPa;
  float32    baro_alt_m;
  float32    baro_temp_C;

  // EKF nominal
  float32    pN, pE, pD;
  float32    vN, vE, vD;
  float32    qw, qx, qy, qz;
  float32    bgx, bgy, bgz;
  float32    bax, bay, baz;

  // EKF covariance (diagonal only)
  float32    Pdiag[15];

  // RC
  float32    thr, roll, pitch, yaw;
  uint8  arm;
  uint8  mode;
  uint8  _pad_rc[6];

  // Optical flow
  float32    of_u, of_v, of_quality;
  uint8  of_valid;
  uint8  _pad_of[7];

  // Motors
  uint16 m0, m1, m2, m3;
  uint8  _pad_mot[4];

  // PID setpoints
  float32    pos_sp_N, pos_sp_E, pos_sp_D;
  float32    vel_sp_N, vel_sp_E, vel_sp_D;
  float32    yaw_sp;
  uint8  pos_sp_valid;
  uint8  _pad_sp[7];
};


struct __attribute__((packed)) DFC_t_UDPImgHeader {
    uint32 magic;
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
inline constexpr size_t getImageDataSize() {
    return static_cast<size_t>(width) * static_cast<size_t>(height);
}
inline constexpr size_t getImagePortSize() {
    return sizeof(DFC_t_ImageHeader) + getImageDataSize();
}

#endif // VISION_SHARED_H
