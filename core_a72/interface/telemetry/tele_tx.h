#ifndef TELE_TX_H
#define TELE_TX_H

#include <iostream>
#include "common/types/data_types.h"

struct __attribute__((packed)) TelemetryPacket {
    uint64 timestamp_imu_1;
    uint64 timestamp_imu_2;
    float64 ax, ay, az;
    float64 gx, gy, gz;
    float64 mx, my, mz;
    float64 temp;
    bool mag_rdy;
    // float32 quat[4];
    // float32 altitude;
};

constexpr uint64 shared_mem_addr = 0xAA000000U;
constexpr uint64 shared_mem_size = sizeof(TelemetryPacket);
#endif