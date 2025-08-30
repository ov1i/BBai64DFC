extern "C" {
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <ti/osal/SemaphoreP.h>
#include <ti/osal/TimerP.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/board/board.h>
}

#include <type_traits>
#include <cstring>
#include <cstdint>

#include <imu/mpu9250.h>
#include <barometer/bmp280.h>
#include <ekf/ekf.h>
#include <controller/PIDController.h>
#include <controller/rcIA6.h>
#include <pwm/PWMgen.h>
#include <comms/RPMsg_helper.h>
#include <comms/TraceLogger.h>
#include <dfc_types.h>


#if defined(__GNUC__) || defined(__clang__)
  #define MEM_BARRIER() __asm__ __volatile__("dmb ish" ::: "memory")
#else
  #define MEM_BARRIER() __sync_synchronize()
#endif

template <typename T>
struct SeqLatest {
  static_assert(std::is_trivially_copyable<T>::value, "SeqLatest<T> requires trivially copyable T");
  alignas(8) volatile uint32 seq{0};
  alignas(8) T               val{};
};

// in order to have 1 writer multiple readers and dump read mid change
template <typename T>
inline void seq_write(SeqLatest<T>& s, const T& v){
  s.seq++;
  MEM_BARRIER();

#if defined(__GNUC__) || defined(__clang__)
  __builtin_memcpy((void*)&s.val, &v, sizeof(T));
#else
  std::memcpy((void*)&s.val, &v, sizeof(T));
#endif

  MEM_BARRIER();
  s.seq++;
}

// prevents torn reads
template <typename T>
inline bool seq_read(const SeqLatest<T>& s, T& out){
  uint32 a = s.seq; 
  MEM_BARRIER();

#if defined(__GNUC__) || defined(__clang__)
  __builtin_memcpy(&out, (const void*)&s.val, sizeof(T));
#else
  std::memcpy(&out, (const void*)&s.val, sizeof(T));
#endif

  MEM_BARRIER();
  uint32 b = s.seq;
  return (a == b) && !(b & 1u); // dump read if corrupted
}

extern "C" void routeDebugP(void);

static constexpr uint32 HZ_IMU   = 200;
static constexpr uint32 HZ_CTRL  = 200;
static constexpr uint32 HZ_BARO  = 50;
static constexpr uint32 HZ_TELEM = 50;

static imu::C_IMU            gIMU;
static baro::C_BMP280        gBARO;
static ekf::C_EKF            gEKF;
static ctrl::C_PIDController gCTRL;
static rc::C_RcIA6           gRC;

static rpmsg::C_RPMsgHelper  gFromC66x;     // optical flow(c66x) -> R5F
static rpmsg::C_RPMsgHelper  gToA72;        // telemetry(r5f)     -> A72
static rpmsg::C_RPMsgHelper  gToA72Calib;   // calib data(r5f)    -> A72

// init sync
static SemaphoreP_Handle gInitDoneSem = nullptr;

static SeqLatest<DFC_t_RcInputs>            gRcLatest;
static SeqLatest<DFC_t_MPU9250_Data>        gImuLatest;
static SeqLatest<DFC_t_EKF_State>           gEkfLatest;
static SeqLatest<DFC_t_BMP280_Data>         gBaroLatest;
static SeqLatest<DFC_t_PIDControllerState>  gPidLatest;
static SeqLatest<float64[3]>                gGyroLatest;

static inline uint64 now_ns() { return (uint64)TimerP_getTimeInUsecs() * 1000ull; }           // both are good
static inline uint32 now_ms() { return (uint32)(xTaskGetTickCount() * portTICK_PERIOD_MS); }  // both are good

enum class EkfMsgType : uint8 { IMU, BARO, FLOW };

struct EkfMsgIMU  { DFC_t_MPU9250_Data data; };
struct EkfMsgBARO { DFC_t_BMP280_Data  data; };
struct EkfMsgFLOW { DFC_t_MsgOpticalFlow data; };

struct EkfMsg {
  EkfMsgType type;
  union {
    EkfMsgIMU  imu;
    EkfMsgBARO baro;
    EkfMsgFLOW flow;
  };
};

// separate queues per producer, EKF task uses a QueueSet to multiplex
static QueueHandle_t qIMU  = nullptr;
static QueueHandle_t qBARO = nullptr;
static QueueHandle_t qFLOW = nullptr;
static QueueSetHandle_t qSET = nullptr;


// init first at boot
static void initTask() {
  routeDebugP();
  UART_init(); 
  UART_stdioInit(0);
  rpmsg::C_RPMsgHelper::init();

  UART_printf("[R5F] init…\r\n");
  DebugP_log0("[R5F] init…\r\n");

  // IMU & BARO drivers
  while(!gIMU.init())  { 
    UART_printf("[R5F] IMU init retry\r\n");
    DebugP_log0("[R5F] IMU init retry\r\n"); 
    vTaskDelay(pdMS_TO_TICKS(200)); 
  }
  while(!gBARO.init()) { 
    UART_printf("[R5F] BARO init retry\r\n"); 
    DebugP_log0("[R5F] BARO init retry\r\n"); 
    vTaskDelay(pdMS_TO_TICKS(200)); 
  }

  // RC driver 
  DFC_t_RcParams rcParams{};
  while(!gRC.init(rcParams)) { 
    DebugP_log0("[R5F] RC reader init retry\r\n"); 
    vTaskDelay(pdMS_TO_TICKS(200)); 
  }

  // PWM 
  DFC_t_PWMgen_Params pwmParams{};
  while(!PWMgen::init(pwmParams)) { 
    UART_printf("[R5F] PWM generator init retry\r\n"); 
    DebugP_log0("[R5F] PWM generator init retry\r\n"); 
    vTaskDelay(pdMS_TO_TICKS(200)); 
  }

  if (PWMgen::detectESCCalibrationGesture(gRC, 3000, 300)) {
    PWMgen::calibrateESC(4000, 2000);
  }

  gBARO.ground_reset();

  {
    const uint32 duration = pdMS_TO_TICKS(1500);
    const uint32 step = pdMS_TO_TICKS(1000 / HZ_BARO);
    uint32 ts = xTaskGetTickCount();

    while((xTaskGetTickCount() - ts) < duration) {
        if(gBARO.update()) {
            gBARO.ground_collect();
        }
        vTaskDelay(step);
    }
  }

  if(!gBARO.ground_finalize()) {
    UART_printf("[R5F] BARO ground finalize: not enough samples falling back.\r\n");
    DebugP_log0("[R5F] BARO ground finalize: not enough samples falling back.\r\n");
  } else {
    UART_printf("[R5F] BARO P0 set.\r\n");
    DebugP_log0("[R5F] BARO P0 set.\r\n");
  }

  // Bias calculation for gyro
  float64 bg[3] = { 0 } ;
  gIMU.getCalibrationGyroBias(3000, &bg[0]); // EVERY BOOT SHOULD BE DONE

  while(!gToA72Calib.open(A72_procID, "r5f_to_a72_calib", "a72_from_r5f_calib")) {
      UART_printf("[R5F] RPMsg pipe between R5F and A72 retry\r\n");
      DebugP_log0("[R5F] RPMsg pipe between R5F and A72 retry\r\n");
      vTaskDelay(pdMS_TO_TICKS(200));       
  };

  const uint32 timeout = xTaskGetTickCount() + pdMS_TO_TICKS(60000); // about 1 min timeout
  while(!gToA72Calib.isDstReady() && xTaskGetTickCount() < timeout) {
    gToA72Calib.tryResolve(100); // we need to make sure the calib end can be reached
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  const bool fullCalib = gToA72Calib.isDstReady();

  if(PWMgen::detectFULLCalibrationGesture(gRC, 5000, 300) && fullCalib) {
    UART_printf("[R5F] ACC+MAG capture START (45000 ms)\r\n");
    DebugP_log0("[R5F] ACC+MAG capture START (45000 ms)\r\n");

    DFC_t_CalibChunk chunk{};
    chunk.magic = DFC_CALI_MAGIC;
    chunk.seq   = 0;
    chunk.count = 0;

    const uint32 calibDuration  = pdMS_TO_TICKS(45000);
    uint32 calibStart = xTaskGetTickCount();
    uint32 ts = calibStart;

    const uint16 max_chunk_size = (uint16)(sizeof(chunk.payload)/sizeof(chunk.payload[0]));
    uint32 sent = 0, kept = 0;

    while ((xTaskGetTickCount() - calibStart) < calibDuration) {
      vTaskDelayUntil(&ts, pdMS_TO_TICKS(10));

      float64 a[3], m[3];
      gIMU.getRawAccMagSample(&a[0], &m[0]);

      auto &s = chunk.payload[chunk.count];
      s.ax = (float32)a[0]; s.ay = (float32)a[1]; s.az = (float32)a[2];
      s.mx = (float32)m[0]; s.my = (float32)m[1]; s.mz = (float32)m[2];

      chunk.count++; 
      kept++;

      if (chunk.count == max_chunk_size) {
        (void)gToA72Calib.send(&chunk, (int)sizeof(DFC_t_CalibChunk));
        chunk.seq++;
        chunk.count = 0;
        sent++;
      }
    }
    // flush partial chunk
    if(chunk.count > 0) {
      sint32 size = (sint32)(offsetof(DFC_t_CalibChunk, payload) + chunk.count*sizeof(DFC_t_CalibSample));
      (void)gToA72Calib.send(&chunk, size);
      sent++;
    }
    UART_printf("[R5F] Calib capture DONE: chunks=%u samples=%u\r\n", sent, kept);
    DebugP_log0("[R5F] Calib capture DONE: chunks=%u samples=%u\r\n", sent, kept);
  }

  // EKF params & reset to current time
  {
  const uint32 calibStart = xTaskGetTickCount();
  const uint32 calibDuration = pdMS_TO_TICKS(1200);

  float64 ax = 0,ay = 0,az = 0;  
  uint32 sampleCountAcc = 0;

  float64 mx = 0,my = 0,mz = 0;  
  uint32 sampleCountMag = 0;

  uint32 ts = calibStart;
  while((xTaskGetTickCount() - ts) < calibDuration) {
    if(gIMU.update()) {
      auto data = gIMU.getData();
      ax += data.ax; 
      ay += data.ay; 
      az += data.az; 
      sampleCountAcc++;

      if(data.mag_rdy) { 
        mx += data.mx; 
        my += data.my; 
        mz += data.mz; 
        sampleCountMag++; }
    }
    vTaskDelayUntil(&ts, pdMS_TO_TICKS(5));
  }

  const bool fuseMag = (sampleCountMag >= 10);
  const float64 decl = 0.087266; // ROMANIA MAG DECLINATION in rad/s (4-6 degree -> we pick 5 degree)
  // this means the geographic north is referenced with about 5 degree from the actual magnetic north
  // every year is differen hence we need new calib every year (shifts daily)

  gEKF.graceful_state_init( sampleCountAcc ? ax/sampleCountAcc : 0, 
                          sampleCountAcc ? ay/sampleCountAcc : 0, 
                          sampleCountAcc ? az/sampleCountAcc : 0,
                          sampleCountMag ? mx/sampleCountMag : 0, 
                          sampleCountMag ? my/sampleCountMag : 0, 
                          sampleCountMag ? mz/sampleCountMag : 0,
                          fuseMag, bg, decl, now_ns());
  UART_printf("[R5F] EKF graceful init done");
  DebugP_log0("[R5F] EKF graceful init done");
  }

  // PID controller init + params
  DFC_t_PIDController_Params ctrlParams{};
  gCTRL.init(ctrlParams);

  // RPMsg endpoints 
  while(!gFromC66x.open(C66_procID, "r5f_from_c66x", nullptr)) {
    UART_printf("[R5F] RPMsg pipe between R5F and C66X retry\r\n");
    DebugP_log0("[R5F] RPMsg pipe between R5F and C66X retry\r\n");
    vTaskDelay(pdMS_TO_TICKS(200));       
  }

  while(!gToA72.open(A72_procID, "r5f_to_a72", "a72_from_r5f")) {
    UART_printf("[R5F] RPMsg pipe between R5F and A72 retry\r\n");
    DebugP_log0("[R5F] RPMsg pipe between R5F and A72 retry\r\n");
    vTaskDelay(pdMS_TO_TICKS(200));       
  } 

  for(uint8 i = 0; i < 50 && !gToA72.isDstReady(); ++i) {
    gToA72.tryResolve(100);
  }

  SemaphoreP_post(gInitDoneSem);
  vTaskDelete(NULL);
}

// IMU: 200 Hz -> qIMU
static void imuTask(void*) {
  SemaphoreP_pend(gInitDoneSem, SemaphoreP_WAIT_FOREVER);
  uint32 last = xTaskGetTickCount();
  const uint32 per = pdMS_TO_TICKS(1000 / HZ_IMU);

  for (;;) {
    vTaskDelayUntil(&last, per);
    if (!gIMU.update()) continue;

    DFC_t_MPU9250_Data data = gIMU.getData();
    if (data.ts_ns == 0) data.ts_ns = now_ns();

    // publish for telemetry
    seq_write(gImuLatest, data);

    // publish gyro for controller immediately (no EKF dependency)
    float64 g3[3] = { data.gx, data.gy, data.gz };
    seq_write(gGyroLatest, g3);

    EkfMsg m{}; 
    m.type = EkfMsgType::IMU; 
    m.imu.data = data;

    (void)xQueueSendToBack(qIMU, &m, 0);    // drop on overflow, EKF is the bottleneck anyway
  }
}

// BARO: 50 Hz -> qBARO
static void baroTask(void*) {
  SemaphoreP_pend(gInitDoneSem, SemaphoreP_WAIT_FOREVER);
  uint32 last = xTaskGetTickCount();
  const uint32 step = pdMS_TO_TICKS(1000 / HZ_BARO);

  for (;;) {
    vTaskDelayUntil(&last, step);
    if (!gBARO.update()) continue;

    DFC_t_BMP280_Data data = gBARO.getData();
    if (data.ts_ns == 0) data.ts_ns = now_ns();

    seq_write(gBaroLatest, data);

    EkfMsg m{}; 
    m.type = EkfMsgType::BARO;
    m.baro.data = data;

    (void)xQueueSendToBack(qBARO, &m, 0);
  }
}

// Optical flow (blocking recv) -> qFLOW
static void flowTask(void*) {
  SemaphoreP_pend(gInitDoneSem, SemaphoreP_WAIT_FOREVER);

  for (uint8 i = 0; i < 40 && !gFromC66x.isDstReady(); ++i) {
    gFromC66x.tryResolve(250);
    vTaskDelay(pdMS_TO_TICKS(25));
  }

  for (;;) {
    DFC_t_MsgOpticalFlow msg{};
    sint32 n = gFromC66x.recv(&msg, (sint32)sizeof(msg), -1);   // block until a frame arrives
    if (n == (sint32)sizeof(msg) && msg.magic == DFC_FLOW_RAW_MAGIC) {
      EkfMsg m{}; 
      m.type = EkfMsgType::FLOW; 
      m.flow.data = msg;

      (void)xQueueSendToBack(qFLOW, &m, 0);
    }
  }
}

// RC reader (fast poll, ~250 Hz) -> snapshot only (controller reads it)
static void rcTask(void*) {
  SemaphoreP_pend(gInitDoneSem, SemaphoreP_WAIT_FOREVER);
  for (;;) {
    gRC.update();
    seq_write(gRcLatest, gRC.getData());

    vTaskDelay(pdMS_TO_TICKS(4));
  }
}

// EKF worker (single owner of gEKF)
// Pulls from a QueueSet so messages are handled in arrival order.
static void ekfTask(void*) {
  SemaphoreP_pend(gInitDoneSem, SemaphoreP_WAIT_FOREVER);

  for (;;) {
    QueueSetMemberHandle_t ready = xQueueSelectFromSet(qSET, portMAX_DELAY);
    if (!ready) continue;

    EkfMsg m{};
    if (xQueueReceive(ready, &m, 0) != pdTRUE) continue;

    switch (m.type) {
      case EkfMsgType::IMU:
        gEKF.handle_imu(m.imu.data);
        break;
      case EkfMsgType::BARO:
        gEKF.handle_baro(m.baro.data);
        break;
      case EkfMsgType::FLOW:
        gEKF.handle_flow(m.flow.data);
        break;
    }

    // publish fresh EKF state snapshot after each update
    seq_write(gEkfLatest, gEKF.getState());
  }
}

// controller (triggered by IMU rate)
static void ctrlTask(void*) {
  SemaphoreP_pend(gInitDoneSem, SemaphoreP_WAIT_FOREVER);
  uint32 last = xTaskGetTickCount();
  const uint32 step = pdMS_TO_TICKS(1000 / HZ_CTRL);

  for (;;) {
    vTaskDelayUntil(&last, step);

    DFC_t_EKF_State data {};
    if (!seq_read(gEkfLatest, data)) continue;     // no EKF state yet

    float64 gyro[3]; 
    if (!seq_read(gGyroLatest, gyro)) continue;

    DFC_t_RcInputs rc; 
    if (!seq_read(gRcLatest, rc)) rc = {};

    gCTRL.update(data, gyro, rc, 1.0 / HZ_CTRL);
    seq_write(gPidLatest, gCTRL.getState());
  }
}

// telemetry: 50 Hz 
static void telemTask(void*) {
  SemaphoreP_pend(gInitDoneSem, SemaphoreP_WAIT_FOREVER);
  uint32 last = xTaskGetTickCount();
  const uint32 step = pdMS_TO_TICKS(1000 / HZ_TELEM);
  static uint32 s_seq = 0;

  // caches to avoid blank frames on rare seqlock collisions
  static DFC_t_MPU9250_Data                 imu_cache{};
  static DFC_t_BMP280_Data                  baro_cache{};
  static DFC_t_EKF_State                    st_cache{};
  static DFC_t_RcInputs                     rc_cache{};
  static DFC_t_PIDControllerState           pid_cache {};

  for (;;) {
    vTaskDelayUntil(&last, step);

    // Update caches (keep last valid on collision)
    DFC_t_MPU9250_Data imu_tmp;         if(seq_read(gImuLatest, imu_tmp)) imu_cache = imu_tmp;
    DFC_t_BMP280_Data baro_tmp;         if(seq_read(gBaroLatest, baro_tmp)) baro_cache = baro_tmp;
    DFC_t_EKF_State st_tmp;             if(seq_read(gEkfLatest, st_tmp)) st_cache  = st_tmp;
    DFC_t_RcInputs rc_tmp;              if(seq_read(gRcLatest, rc_tmp)) rc_cache = rc_tmp;
    DFC_t_PIDControllerState pid_tmp;   if(seq_read(gPidLatest, pid_tmp)) pid_cache = pid_tmp;

    // Build packet
    DFC_t_TelemetryPacket m{};
    m.magic = DFC_TELE_MAGIC; 
    m.size  = (uint16)sizeof(DFC_t_TelemetryPacket);
    m.seq   = s_seq++;

    // IMU/MAG
    m.ax = (float32)imu_cache.ax; m.ay =( float32)imu_cache.ay; m.az = (float32)imu_cache.az;
    m.gx = (float32)imu_cache.gx; m.gy = (float32)imu_cache.gy; m.gz = (float32)imu_cache.gz;
    m.mx = (float32)imu_cache.mx; m.my = (float32)imu_cache.my; m.mz = (float32)imu_cache.mz;

    m.mag_adjustment[0] = (float32)imu_cache.mag_adjustment[0];
    m.mag_adjustment[1] = (float32)imu_cache.mag_adjustment[1];
    m.mag_adjustment[2] = (float32)imu_cache.mag_adjustment[2];

    m.imu_temp = (float32)imu_cache.temp;

    m.mag_rdy = imu_cache.mag_rdy ? 1u : 0u;

    // BARO
    m.baro_p_hPa  = (float32)baro_cache.pressure;
    m.baro_alt_m  = (float32)baro_cache.altitude;
    m.baro_temp_C = (float32)baro_cache.temp;

    // EKF nominal
    m.pN = (float32)st_cache.p[0]; m.pE = (float32)st_cache.p[1]; m.pD = (float32)st_cache.p[2];
    m.vN = (float32)st_cache.v[0]; m.vE = (float32)st_cache.v[1]; m.vD = (float32)st_cache.v[2];
    m.qw = (float32)st_cache.q[0]; m.qx = (float32)st_cache.q[1]; m.qy = (float32)st_cache.q[2]; m.qz = (float32)st_cache.q[3];

    // biases + P diag
    m.bgx = (float32)st_cache.bg[0]; m.bgy = (float32)st_cache.bg[1]; m.bgz = (float32)st_cache.bg[2];
    m.bax = (float32)st_cache.ba[0]; m.bay = (float32)st_cache.ba[1]; m.baz = (float32)st_cache.ba[2];
    for (uint8 i=0;i<15;i++) m.Pdiag[i] = (float32)st_cache.P[i*15 + i];

    // RC
    m.thr   = (float32)rc_cache.thr; 
    m.roll  = (float32)rc_cache.roll;
    m.pitch = (float32)rc_cache.pitch;
    m.yaw   = (float32)rc_cache.yaw;
    m.arm = rc_cache.arm ? 1u : 0u;
    m.mode= (uint8)rc_cache.mode;

    // Optical flow
    m.of_u          = (float32)st_cache.oflow_u; 
    m.of_v          = (float32)st_cache.oflow_v; 
    m.of_quality    = (float32)st_cache.oflow_quality; 
    m.of_valid      = st_cache.oflow_valid;

    // Motors + PID setpoints
    m.m0 = pid_cache.m1;
    m.m1 = pid_cache.m2; 
    m.m2 = pid_cache.m3; 
    m.m3 = pid_cache.m4;

    m.pos_sp_N = (float32)pid_cache.pos_sp_NED[0]; 
    m.pos_sp_E = (float32)pid_cache.pos_sp_NED[1]; 
    m.pos_sp_D = (float32)pid_cache.pos_sp_NED[2];

    m.vel_sp_N = (float32)pid_cache.vel_sp_NED[0];
    m.vel_sp_E = (float32)pid_cache.vel_sp_NED[1];
    m.vel_sp_D = (float32)pid_cache.vel_sp_NED[2];

    m.yaw_sp        = (float32)pid_cache.yaw_sp;
    m.pos_sp_valid  = pid_cache.pos_sp_valid ? 1u : 0u;

    // Send to A72 via RPMsg
    if (gToA72.isDstReady()) {
      (void)gToA72.send(&m, (sint32)sizeof(m));
    } else {
      static uint32 resolve_tick = 0;
      if((resolve_tick++ & 0x7D0) == 0) {
        (void)gToA72.tryResolve(100);   
      }
      // DROP TELEMETRY FRAME BY DEFAULT IF NO DST
    }
  }
}

int main(void) {
  gInitDoneSem = SemaphoreP_create(0, NULL);

  // queues + set (depths tuned to keep latency low)
  qIMU  = xQueueCreate(16, sizeof(EkfMsg));
  qBARO = xQueueCreate( 8, sizeof(EkfMsg));
  qFLOW = xQueueCreate(16, sizeof(EkfMsg));

  qSET = xQueueCreateSet(16 + 8 + 16);
  xQueueAddToSet(qIMU,  qSET);
  xQueueAddToSet(qBARO, qSET);
  xQueueAddToSet(qFLOW, qSET);

  // tasks
  xTaskCreate(initTask, "init",  3072, NULL, 6, NULL);
  xTaskCreate(imuTask,  "imu",   2048, NULL, 5, NULL);
  xTaskCreate(baroTask, "baro",  1536, NULL, 4, NULL);
  xTaskCreate(flowTask, "flow",  2048, NULL, 4, NULL);
  xTaskCreate(ekfTask,  "ekf",   3072, NULL, 5, NULL);
  xTaskCreate(ctrlTask, "ctrl",  2048, NULL, 4, NULL);
  xTaskCreate(rcTask,   "rc",    1536, NULL, 3, NULL);
  xTaskCreate(telemTask,"telem", 1536, NULL, 2, NULL);

  vTaskStartScheduler();
  for(;;){} // should never return

  return 0;
}
