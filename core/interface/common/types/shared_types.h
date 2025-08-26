#ifndef SHARED_TYPES_H
#define SHARED_TYPES_H

#include "data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

const uint32 DFC_FLOW_RAW_MAGIC = 0x4F464C4FU; // sanity check via magic no.
const uint32 DFC_TELE_MAGIC =     0x54454C45U; // sanity check via magic no.
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
  uint32 magic;
  uint64 ts_prev_ns;
  uint64 ts_curr_ns;
  float64 u_px_per_s;   // +right (ghfo pxl/sec)
  float64 v_px_per_s;   // +down (gvfo pxl/sec)
  float64 quality;      // 0..1 fraction of valid tracks
  uint16 width;
  uint16 height;
} DFC_t_MsgOpticalFlow;


struct __attribute__((packed)) DFC_t_TelemetryPacket {
  // Header
  uint32 magic;     
  uint16 size;
  uint32 seq;

  // IMU / Mag (no timestamps)
  float32    ax, ay, az, gx, gy, gz;
  float32    mx, my, mz;
  float32    mag_adjustment[3];
  uint8      mag_rdy;
  uint8      _pad_mag[7];
  float32    imu_temp;

  // BMP280 (no timestamps)
  float32    baro_p_hPa;
  float32    baro_alt_m;
  float32    baro_temp_C;

  // EKF nominal
  float32    pN, pE, pD;      // POS NED 
  float32    vN, vE, vD;      // VELOCITY NED
  float32    qw, qx, qy, qz;  // ATTITUDE
  float32    bgx, bgy, bgz;   // bias gyro
  float32    bax, bay, baz;   // bias acc

  // EKF covariance
  float32    Pdiag[15];

  // RC
  float32     thr, roll, pitch, yaw;
  uint8       arm;
  uint8       mode;
  uint8       _pad_rc[6];

  // Optical flow
  float32      of_u, of_v, of_quality;
  uint8        of_valid;
  uint8        _pad_of[7];

  // Motors
  uint16        m0, m1, m2, m3;
  uint8         _pad_mot[4];

  // PID setpoints
  float32    pos_sp_N, pos_sp_E, pos_sp_D;
  float32    vel_sp_N, vel_sp_E, vel_sp_D;
  float32    yaw_sp;
  uint8      pos_sp_valid;
  uint8      _pad_sp[7];
};

typedef struct {
  float64 u_px_per_s;
  float64 v_px_per_s;
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

