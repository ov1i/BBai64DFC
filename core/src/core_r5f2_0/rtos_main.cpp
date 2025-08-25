extern "C" {
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <ti/osal/TaskP.h>
#include <ti/osal/SemaphoreP.h>
#include <ti/osal/QueueP.h>
#include <ti/osal/TimerP.h>
#include <ti/osal/osal.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/board/board.h>
#include <ti/drv/i2c/i2c.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/csl/arch/r5/csl_arm_r5_mpu.h>
// #include "rpmsg_lite.h"

}
#include "imu/mpu9250.h"
#include "dfc_types.h"


static uint8 gTaskStackIMU[IMU_TASKSTACKSIZE]                   __attribute__((aligned(32)));
// static uint8 gTaskStackEKF[EKF_TASKSTACKSIZE]                   __attribute__((aligned(32)));
static uint8 gTaskStackInit[INIT_TASKSTACKSIZE]                 __attribute__((aligned(32)));

void initTask(void *arg0, void *arg1) {
    imu::C_IMU* pImuOBJ = (imu::C_IMU*)arg0;
    SemaphoreP_Handle* pInitSempahHandler = (SemaphoreP_Handle*)arg1;
    Board_STATUS boardStatus = BOARD_SOK;

    UART_init();
    UART_stdioInit(0);
    
    while(pImuOBJ == NULL) {
        UART_printf("Imu object corrupted...checkup required!\r\n");
    }
    while(!pImuOBJ->init()) {
        UART_printf("MPU9250 init failed! Retrying in a bit...\r\n");
        TaskP_sleep(10000);
    }
    UART_printf("All init OK, releasing app tasks!\r\n");
    SemaphoreP_post(*pInitSempahHandler);
    vTaskDelete(NULL);
}

void imuReadTask(void *arg0, void *arg1) {
    DFC_t_IMU_TaskArgs *imu_task_args = (DFC_t_IMU_TaskArgs *)arg0;
    SemaphoreP_Handle* pInitSempahHandler = (SemaphoreP_Handle*)arg1;
    DFC_t_MPU9250_Data imuRawData;
    
    SemaphoreP_pend(*pInitSempahHandler, SemaphoreP_WAIT_FOREVER);

    while (1) {
        // SemaphoreP_pend(imu_task_args->imu_data_semaph, SemaphoreP_WAIT_FOREVER);

        if (imu_task_args->imu->update()) {
            imu_task_args->imu->getCurrentRawData(&imuRawData);
            UART_printf("IMU: ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f mx=%.2f my=%.2f mz=%.2f T=%.2f\r\n",
                       imuRawData.ax, imuRawData.ay, imuRawData.az,
                       imuRawData.gx, imuRawData.gy, imuRawData.gz,
                       imuRawData.mx, imuRawData.my, imuRawData.mz,
                       imuRawData.temp);
            // QueueP_put(imu_task_args->imu_data_queue, (QueueP_Elem *)&imuRawData);
        } else {
            UART_printf("IMU read failed\r\n");
        }
        TaskP_sleepInMsecs(5);
    }
}

// void ekfTask(void *arg0, void *arg1) {
//     DFC_t_EKF_TaskArgs_t *args = (DFC_t_EKF_TaskArgs_t *)arg0;
//     SemaphoreP_Handle* pInitSempahHandler = (SemaphoreP_Handle*)arg1;
//     DFC_t_MPU9250_Data *imuRawData;

//     SemaphoreP_pend(*pInitSempahHandler, SemaphoreP_WAIT_FOREVER);

//     while (1) {
//         imuRawData = (DFC_t_MPU9250_Data*)QueueP_get(args->imu_data_queue);
//         imuRawData->ax = imuRawData->ax;
//         vTaskDelay(pdMS_TO_TICKS(50)); // 50 ms

//         SemaphoreP_post(args->imu_data_semaph);
//     }
// }

int main(void) {
    OS_init();

    TaskP_Params imuTaskParams, initTaskParams; //, ekfTaskParams, loggerTaskParams;
    QueueP_Params queueParams;
    QueueP_Params_init(&queueParams);
    QueueP_Handle imu_data_queue = QueueP_create(&queueParams);
    SemaphoreP_Handle imu_data_semaph = SemaphoreP_create(1, NULL);
    SemaphoreP_Handle init_semaph = SemaphoreP_create(0, NULL);

    imu::C_IMU imuObj;
    DFC_t_IMU_TaskArgs imuTaskArgs = { &imuObj, imu_data_queue, imu_data_semaph };
    // DFC_t_EKF_TaskArgs_t ekfTaskArgs = { imu_data_queue, imu_data_semaph };

    
    TaskP_Params_init(&imuTaskParams);
    imuTaskParams.stack      = gTaskStackIMU;
    imuTaskParams.stacksize  = IMU_TASKSTACKSIZE;
    imuTaskParams.priority   = 3;
    imuTaskParams.arg0       = (void *)&imuTaskArgs;
    imuTaskParams.arg1       = (void *)&init_semaph;
    imuTaskParams.name       = "IMU_Read";
    TaskP_create(imuReadTask, &imuTaskParams);

    // TaskP_Params_init(&ekfTaskParams);
    // ekfTaskParams.stack      = gTaskStackEKF;
    // ekfTaskParams.stacksize  = EKF_TASKSTACKSIZE;
    // ekfTaskParams.priority   = 2;
    // ekfTaskParams.arg0       = (void *)&ekfTaskArgs;
    // ekfTaskParams.arg1       = (void *)&init_semaph;
    // ekfTaskParams.name       = "EKF";
    // TaskP_create(ekfTask, &ekfTaskParams);

    TaskP_Params_init(&initTaskParams);
    initTaskParams.stack = gTaskStackInit;
    initTaskParams.stacksize = INIT_TASKSTACKSIZE;
    initTaskParams.priority = 4;
    initTaskParams.arg0 = (void *)&imuObj;
    initTaskParams.arg1 = (void *)&init_semaph;
    initTaskParams.name = "Init";
    TaskP_create(initTask, &initTaskParams);

    OS_start();

    while (1) { ; }

    return 0;
}



extern "C" {
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <ti/osal/TaskP.h>
#include <ti/osal/SemaphoreP.h>
#include <ti/osal/QueueP.h>
#include <ti/osal/TimerP.h>
#include <ti/osal/osal.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/board/board.h>
#include <ti/drv/i2c/i2c.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/csl/arch/r5/csl_arm_r5_mpu.h>
}

#include <cstring>
#include <cmath>

// ---- Your modules (adjust paths/names to your tree) -----------------------
#include "comms/rpmsg_helper.hpp"     // your helper (R5F<->A72, R5F<->C66x)
#include "imu/mpu9250.h"              // imu::C_IMU
#include "baro/bmp280.h"              // baro::C_BMP280   (assumed)
#include "ekf/ekf.h"                  // ekf::C_EKF + DFC_t_* types
#include "controller/PIDController.h" // ctrl::C_PIDController
#include "pwm/PWMgen.h"               // motor outputs
#include "rc/rc_sbus.h"               // rc::C_RC (sbus/ppm) — replace with your driver
#include "common/types/shared_types.h"// DFC_t_ImageHeader, width/height if needed
#include "common/dfc_types.h"         // DFC_t_* (IMU/Baro structs, EKF params, PID params, RC, Flow msg)

// --------------------------------------------------------------------------
//                               RATES & STACKS
// --------------------------------------------------------------------------
static constexpr uint32_t RATE_EKF_PRED_HZ   = 200;   // 5 ms (IMU pace)
static constexpr uint32_t RATE_CTRL_HZ       = 200;   // 5 ms (after IMU)
static constexpr uint32_t RATE_BARO_HZ       = 50;    // 20 ms
static constexpr uint32_t RATE_RC_HZ         = 200;   // 5 ms
static constexpr uint32_t RATE_TELEM_HZ      = 50;    // 20 ms
static constexpr uint32_t RATE_RPMSG_POLL_HZ = 500;   // for flow/RPMsg poll (if needed)

// Stacks (align to 32)
#define STK(sz)  __attribute__((aligned(32))) static uint8_t sz

STK(gStackInit[2048]);
STK(gStackIMU[2048]);
STK(gStackBaro[1536]);
STK(gStackFlow[2048]);
STK(gStackRC[1536]);
STK(gStackCtrl[2048]);
STK(gStackTelem[2048]);

// --------------------------------------------------------------------------
//                         GLOBAL SINGLETONS / CONTEXT
// --------------------------------------------------------------------------
static imu::C_IMU              gIMU;
static baro::C_BMP280          gBARO;          // implement or replace by your BMP task
static ekf::C_EKF              gEKF;
static ctrl::C_PIDController   gCTRL;
static rc::C_RC                gRC;            // replace with your RC driver

// RPMsg
static rpmsg::C_RPMsgHelper    gRpmg_from_C66x;  // optical flow in
static rpmsg::C_RPMsgHelper    gRpmg_to_A72;     // telemetry out (A72 forwards via UDP)
static rpmsg::C_RPMsgHelper    gRpmg_from_A72;   // optional commands in

// Sync
static SemaphoreP_Handle gInitDoneSem  = nullptr;   // released when init ends
static SemaphoreP_Handle gImuTickSem   = nullptr;   // "fresh IMU/EKF predict happened"

// Shared “last known” data for control loop
static DFC_t_RcInputs gRcLatest{};
static float64        gGyroLatest[3] = {0,0,0};

// --------------------------------------------------------------------------
//                       PARAMETER FILLERS (EDIT TO TASTE)
// --------------------------------------------------------------------------
static void fill_default_ekf_params(DFC_t_EKF_Params& p) {
  std::memset(&p, 0, sizeof(p));
  // sensor noises (rough defaults; tune!)
  p.gyro_noise     = 0.02;     // rad/s / sqrt(Hz)
  p.acc_noise      = 0.8;      // m/s^2 / sqrt(Hz)
  p.gyro_bias_rw   = 0.0005;   // rad/s^2 sqrt
  p.acc_bias_rw    = 0.02;     // m/s^3 sqrt
  p.baro_std       = 0.5;      // m (alt sigma)
  p.mag_std        = 0.05;     // normed mag sigma
  // mag north vector (approx NED in your location). Adjust!
  p.magN[0] = 0.2; p.magN[1] = 0.0; p.magN[2] = 0.98;

  // optical flow
  p.flow_vel_std   = 0.3;      // m/s equiv noise near 1m
  p.focalLengthX   = 320.0;    // IMX219 fx (in pixels) — set from calibration
  p.focalLengthY   = 320.0;    // IMX219 fy (in pixels) — set from calibration
}

static void fill_default_ctrl_params(DFC_t_PIDController_Params& c) {
  std::memset(&c, 0, sizeof(c));
  c.pwm_min_us = 1000; c.pwm_max_us = 2000;

  // RC shaping
  c.rc_deadband = 0.05;
  c.rc_expo     = 0.2;
  c.rc_max_rate[0] = 3.5; c.rc_max_rate[1] = 3.5; c.rc_max_rate[2] = 3.0; // rad/s
  c.rc_yaw_rate = 2.5;
  c.rc_max_tilt = 0.5;  // ~28 deg

  // Position & velocity (POS_HOLD)
  c.kp_pos_xy = 1.0; c.ki_pos_xy = 0.05; c.vel_i_lim_xy = 1.0;
  c.kp_vel_xy = 2.0; c.ki_vel_xy = 0.2;  c.acc_i_lim_xy = 2.0;
  c.max_vel_xy = 2.0; c.max_acc_xy = 4.0;

  c.kp_pos_z  = 1.0; c.ki_pos_z = 0.1; c.vel_i_lim_z = 1.0;
  c.kp_vel_z  = 2.0; c.ki_vel_z = 0.2; c.acc_i_lim_z = 2.0;
  c.max_vel_z_up = 1.0; c.max_vel_z_down = 1.0;

  // Attitude outer
  c.kp_att_roll = 6.0; c.kp_att_pitch = 6.0; c.kp_att_yaw = 3.0;

  // Rate inner
  c.kp_rate[0]=0.25; c.ki_rate[0]=0.25; c.kd_rate[0]=0.0;
  c.kp_rate[1]=0.25; c.ki_rate[1]=0.25; c.kd_rate[1]=0.0;
  c.kp_rate[2]=0.25; c.ki_rate[2]=0.20; c.kd_rate[2]=0.0;
  c.rate_i_lim[0]=0.4; c.rate_i_lim[1]=0.4; c.rate_i_lim[2]=0.3;

  c.hover_thrust = 0.5;
  c.min_thrust   = 0.0;
  c.max_thrust   = 1.0;

  // mixers
  c.mix_roll = 0.8; c.mix_pitch = 0.8; c.mix_yaw = 0.5;
}

// --------------------------------------------------------------------------
//                                 INIT TASK
// --------------------------------------------------------------------------
static void initTask(void *arg0, void *arg1) {
  (void)arg0; (void)arg1;
  UART_init(); UART_stdioInit(0);
  UART_printf("\r\n[R5F] Init start...\r\n");

  Board_initCfg boardCfg = BOARD_INIT_MODULE_CLOCK | BOARD_INIT_PINMUX_CONFIG;
  if (Board_init(boardCfg) != BOARD_SOK) {
    UART_printf("[R5F] Board_init failed\r\n");
  }

  // (Optional) pinmux adjustments for I2C pins you showed earlier go here

  // I2C init (for IMU + BARO)
  I2C_init();

  // IPC/RPMsg init
  rpmsg::C_RPMsgHelper::init();

  // Open endpoints:
  //  - from C66x (flow)
  gRpmg_from_C66x.open(DFC_t_ProcIDs::C66X, "r5f_from_c66x", "c66x_to_r5f"); // local announce + resolve remote
  //  - to A72 (telemetry)
  gRpmg_to_A72.open(DFC_t_ProcIDs::A72, "r5f_to_a72", "a72_from_r5f");
  //  - from A72 (optional cmds)
  gRpmg_from_A72.open(DFC_t_ProcIDs::A72, "r5f_from_a72", "a72_to_r5f");

  // IMU bring-up & quick gyro bias calibration
  while(!gIMU.init()) {
    UART_printf("[R5F] MPU9250 init failed, retry...\r\n"); vTaskDelay(pdMS_TO_TICKS(200));
  }
  UART_printf("[R5F] IMU OK, calibrating gyro... keep still.\r\n");
  gIMU.calibrateGyroBlocking(2000 /* ms */);
  UART_printf("[R5F] Gyro calibration done.\r\n");

  // BARO bring-up & ground ref
  while(!gBARO.init()) {
    UART_printf("[R5F] BMP280 init failed, retry...\r\n"); vTaskDelay(pdMS_TO_TICKS(200));
  }
  gBARO.captureGroundReference(1500 /* ms averaging window */);
  UART_printf("[R5F] Baro ground ref OK.\r\n");

  // EKF reset + params
  DFC_t_EKF_Params ekfP; fill_default_ekf_params(ekfP);
  gEKF.setCustomParams(ekfP);
  gEKF.reset(gIMU.now_ns());

  // Controller params + motors safe
  DFC_t_PIDController_Params cP; fill_default_ctrl_params(cP);
  gCTRL.init(cP);

  // RC input
  gRC.init(); // pick your port (sbus/ppm) inside

  // Optional: ESC calibration (commented for safety)
  // gCTRL.calibrateEscBlocking();

  SemaphoreP_post(gInitDoneSem);
  UART_printf("[R5F] Init done.\r\n");
  vTaskDelete(NULL);
}

// --------------------------------------------------------------------------
//                                 IMU TASK
// --------------------------------------------------------------------------
static void imuTask(void *arg0, void *arg1) {
  (void)arg0; (void)arg1;
  SemaphoreP_pend(gInitDoneSem, SemaphoreP_WAIT_FOREVER);

  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / RATE_EKF_PRED_HZ);

  for (;;) {
    vTaskDelayUntil(&last, period);

    DFC_t_MPU9250_Data s{};
    if (gIMU.update()) {
      gIMU.getCurrentRawData(&s);

      // EKF predict step happens inside handle_imu()
      gEKF.handle_imu(s);

      // keep the latest gyro for the rate controller
      gGyroLatest[0] = s.gx; gGyroLatest[1] = s.gy; gGyroLatest[2] = s.gz;

      // let control loop know a fresh predict is available
      SemaphoreP_post(gImuTickSem);
    }
  }
}

// --------------------------------------------------------------------------
//                                 BARO TASK
// --------------------------------------------------------------------------
static void baroTask(void *arg0, void *arg1) {
  (void)arg0; (void)arg1;
  SemaphoreP_pend(gInitDoneSem, SemaphoreP_WAIT_FOREVER);

  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / RATE_BARO_HZ);

  for (;;) {
    vTaskDelayUntil(&last, period);

    DFC_t_BMP280_Data b{};
    if (gBARO.read(b)) {
      // altitude in meters (NED: update_baro expects +down; your BMP code
      // should already provide relative altitude, positive down)
      gEKF.handle_baro(b);
    }
  }
}

// --------------------------------------------------------------------------
//                         OPTICAL FLOW RPMSG (C66x) TASK
// --------------------------------------------------------------------------
static void flowTask(void *arg0, void *arg1) {
  (void)arg0; (void)arg1;
  SemaphoreP_pend(gInitDoneSem, SemaphoreP_WAIT_FOREVER);

  // Try to resolve remote endpoint for a while (non-fatal if late)
  for (int i=0;i<40 && !gRpmg_from_C66x.isDstReady(); ++i) {
    gRpmg_from_C66x.tryResolve();
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  // Blocking receive loop
  for (;;) {
    DFC_t_MsgOpticalFlow msg{};
    const int n = gRpmg_from_C66x.recv(&msg, (int)sizeof(msg), -1);
    if (n == (int)sizeof(msg) && msg.magic == DFC_FLOW_RAW_MAGIC) {
      gEKF.handle_flow(msg); // does gyro-mean derotation internally, then update
    }
  }
}

// --------------------------------------------------------------------------
//                                 RC TASK
// --------------------------------------------------------------------------
static void rcTask(void *arg0, void *arg1) {
  (void)arg0; (void)arg1;
  SemaphoreP_pend(gInitDoneSem, SemaphoreP_WAIT_FOREVER);

  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / RATE_RC_HZ);

  for (;;) {
    vTaskDelayUntil(&last, period);
    gRC.read(gRcLatest);  // fill DFC_t_RcInputs (mode, roll/pitch/yaw/thr, arm)
  }
}

// --------------------------------------------------------------------------
//                              CONTROL (PID) TASK
// --------------------------------------------------------------------------
static void ctrlTask(void *arg0, void *arg1) {
  (void)arg0; (void)arg1;
  SemaphoreP_pend(gInitDoneSem, SemaphoreP_WAIT_FOREVER);

  // Run right after IMU/EKF tick to minimize latency
  for (;;) {
    // Wait up to one period for a fresh tick; if missed, still run (failsafe)
    if (SemaphoreP_pend(gImuTickSem, pdMS_TO_TICKS(1000 / RATE_CTRL_HZ)) == SemaphoreP_TIMEOUT) {
      // no fresh IMU tick; fall back to periodic
      vTaskDelay(pdMS_TO_TICKS(1000 / RATE_CTRL_HZ));
    }

    // Use EKF state snapshot
    const DFC_t_EKF_State st = gEKF.state(); // returns by value (small struct)

    // dt: you can compute from EKF timestamps if you want; we keep constant 5ms
    constexpr float64 dt = 1.0/ (float64)RATE_CTRL_HZ;

    // Controller update → PWM
    gCTRL.update(st, gGyroLatest, gRcLatest, dt);
  }
}

// --------------------------------------------------------------------------
//                              TELEMETRY TASK
// --------------------------------------------------------------------------
static void telemTask(void *arg0, void *arg1) {
  (void)arg0; (void)arg1;
  SemaphoreP_pend(gInitDoneSem, SemaphoreP_WAIT_FOREVER);

  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / RATE_TELEM_HZ);

  for (;;) {
    vTaskDelayUntil(&last, period);

    // pack a minimal telemetry (extend as you like)
    struct __attribute__((packed)) TelemetryMsg {
      uint32_t magic;
      uint64_t t_ns;
      float    p[3], v[3];
      float    q[4];
      float    thr;      // last thrust cmd (optional: expose from controller)
      uint8_t  mode;
      uint8_t  armed;
    } m{};

    m.magic = 0x54454C45; // 'TELE'
    const auto st = gEKF.state();
    m.t_ns = st.t_ns;
    m.p[0] = st.p[0]; m.p[1]=st.p[1]; m.p[2]=st.p[2];
    m.v[0] = st.v[0]; m.v[1]=st.v[1]; m.v[2]=st.v[2];
    m.q[0] = st.q[0]; m.q[1]=st.q[1]; m.q[2]=st.q[2]; m.q[3]=st.q[3];
    m.thr  = 0.0f;  // TODO: feed from controller if you keep a getter
    m.mode = (uint8_t)gRcLatest.mode;
    m.armed= gRcLatest.arm ? 1 : 0;

    (void)gRpmg_to_A72.send(&m, (int)sizeof(m));
  }
}

// --------------------------------------------------------------------------
//                                   MAIN
// --------------------------------------------------------------------------
int main(void) {
  OS_init();

  // Semaphores
  gInitDoneSem = SemaphoreP_create(0, NULL);
  gImuTickSem  = SemaphoreP_create(0, NULL);

  // Create tasks (priorities: higher number = higher priority in TI-RTOS/FreeRTOS)
  TaskP_Params p;
  TaskP_Params_init(&p);

  // Init (highest)
  p.stack = gStackInit; p.stacksize = sizeof(gStackInit); p.priority = 6;
  p.name  = (char*)"Init"; TaskP_create(initTask, &p);

  // IMU (200 Hz)
  TaskP_Params_init(&p);
  p.stack = gStackIMU; p.stacksize = sizeof(gStackIMU); p.priority = 5;
  p.name  = (char*)"IMU"; TaskP_create(imuTask, &p);

  // Control (200 Hz) — runs after IMU tick
  TaskP_Params_init(&p);
  p.stack = gStackCtrl; p.stacksize = sizeof(gStackCtrl); p.priority = 4;
  p.name  = (char*)"CTRL"; TaskP_create(ctrlTask, &p);

  // Flow RPMsg (blocking receive)
  TaskP_Params_init(&p);
  p.stack = gStackFlow; p.stacksize = sizeof(gStackFlow); p.priority = 3;
  p.name  = (char*)"FLOW"; TaskP_create(flowTask, &p);

  // Baro (50 Hz)
  TaskP_Params_init(&p);
  p.stack = gStackBaro; p.stacksize = sizeof(gStackBaro); p.priority = 3;
  p.name  = (char*)"BARO"; TaskP_create(baroTask, &p);

  // RC (200 Hz)
  TaskP_Params_init(&p);
  p.stack = gStackRC; p.stacksize = sizeof(gStackRC); p.priority = 3;
  p.name  = (char*)"RC"; TaskP_create(rcTask, &p);

  // Telemetry (50 Hz)
  TaskP_Params_init(&p);
  p.stack = gStackTelem; p.stacksize = sizeof(gStackTelem); p.priority = 2;
  p.name  = (char*)"TELEM"; TaskP_create(telemTask, &p);

  OS_start();

  // should never reach
  for(;;) { }
  return 0;
}
