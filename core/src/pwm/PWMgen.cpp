#include "pwm/PWMgen.h"

extern "C" {
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/csl/soc.h>
#include <ti/csl/cslr.h>
#include <ti/csl/csl.h>
#include <ti/csl/cslr_epwm.h>
#include <ti/board/board.h>
#include <ti/board/board_cfg.h>
#include <ti/osal/TaskP.h>

}

#include <cmath>

namespace PWMgen {

static DFC_t_PWMgen_Params g_params;

// Map our 4 logical channels to the two EHRPWM instances and their A/B outputs
struct ChanMap { volatile CSL_epwmRegs* base; bool isA; };
static ChanMap g_map[4];

// Time-base precompute
static uint16 g_tbprd = 0;       // period counts
static uint16 g_clkdiv = 0;      // TBCTL.CLKDIV
static uint16 g_hspdiv = 0;      // TBCTL.HSPCLKDIV
static float64 g_counts_per_us = 0.0; // to convert us to counts

sint32 preInit(uint32 id) {
  Sciclient_ConfigPrms_t config; 
  Sciclient_configPrmsInit(&config);
  sint32 status = Sciclient_init(&config);
  if(status && status != CSL_PASS) return status;
  
  struct tisci_msg_set_device_req  req  = {};
  struct tisci_msg_set_device_resp resp = {};

  Sciclient_ReqPrm_t reqParam  = { 0 };
  Sciclient_RespPrm_t respParam = { 0 };

  req.id         = id;
  req.state      = TISCI_MSG_VALUE_DEVICE_SW_STATE_ON;
  req.hdr.flags  = TISCI_MSG_FLAG_AOP;

  reqParam.messageType    = TISCI_MSG_SET_DEVICE;
  reqParam.pReqPayload    = (const uint8*)&req;
  reqParam.reqPayloadSize = (uint32)sizeof(req);
  reqParam.timeout        = SCICLIENT_SERVICE_WAIT_FOREVER;

  respParam.pRespPayload    = (uint8*)&resp;
  respParam.respPayloadSize = (uint32)sizeof(resp);

  status = Sciclient_service(&reqParam, &respParam);
  if (status != CSL_PASS) return status;

  if ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK) {
    return CSL_EFAIL;
  }
  
  return CSL_PASS;
}

void setupDividersPeriod(float64 clk_hz, uint32 freq_hz, uint16& clkdiv, uint16& hspdiv, uint16& tbprd, float64& counts_per_us) {
  // ePWM has two dividers that are encoded in "code" here are the actual vals: CLKDIV -> 1,2,4,8 and HSPCLKDIV -> 1,2,4,6,8,10,12,14
  const uint16 clkdiv_true_vals[4]   = {1,2,4,8};
  const uint16 hspdiv_true_vals[8]   = {1,2,4,6,8,10,12,14};

  float64 best_err = 1e99;
  uint16 best_clkdiv = 1, best_hspdiv = 1, best_tbprd = 0;

  const float64 period_s = 1.0 / (float64)freq_hz;
  for (uint8 i=0;i<4;i++) {
    for (uint8 j=0;j<8;j++) {
      float64 clk = clk_hz / (clkdiv_true_vals[i] * hspdiv_true_vals[j]);
      float64 counts_per_prd   = clk * period_s;
      if (counts_per_prd > 65535.0) continue;           // won’t fit :O
      if (counts_per_prd < 100.0)  continue;            // too small  -_-
      // I guess in between is perfect 0_0
      float64 err = std::abs(counts_per_prd - 40000.0);
      if (err < best_err) {
        best_err  = err;
        best_clkdiv = clkdiv_true_vals[i];
        best_hspdiv = hspdiv_true_vals[j];
        best_tbprd = (uint16)(counts_per_prd + 0.5); // round to the nearest (pretty little trick, to avoid std::round)
      }
    }
  }
  if (best_tbprd == 0) {
    // Fallback: force max period with largest divider
    best_clkdiv = 8; best_hspdiv = 14;
    float64 clk = clk_hz / (best_clkdiv * best_hspdiv);
    best_tbprd = (uint16)std::fmin(65535.0, clk * period_s);
  }

  switch (best_clkdiv) { 
    case 1: 
        clkdiv = 0; // 1 is 0
        break; 
    case 2: 
        clkdiv = 1; // 2 is 1
        break;
    case 4: 
        clkdiv = 2; // 4 is 2
        break; 
    default: 
        clkdiv = 3; // 8 is 3
        break; 
  } 

  switch (best_hspdiv) {
    case 1:  
        hspdiv = 0; // 1 is 0
        break; 
    case 2:  
        hspdiv = 1; // 2 is 1
        break; 
    case 4:  
        hspdiv = 2; // 4 is 2
        break; 
    case 6:  
        hspdiv = 3; // 6 is 3
        break;
    case 8:  
        hspdiv = 4; // 8 is 4
        break; 
    case 10: 
        hspdiv = 5; // 10 is 5
        break; 
    case 12: 
        hspdiv = 6; // 12 is 6
        break; 
    default: 
        hspdiv = 7; // 14 is 7
        break; 
  }

  //NICE MATH G

  tbprd = best_tbprd;
  float64 clk = clk_hz / (float64)(best_clkdiv * best_hspdiv);
  counts_per_us = clk / 1e6;
}

void setupEPWM(volatile CSL_epwmRegs* pwm) {
  uint16 tbctl = 0;
  tbctl |= (0 << CSL_EPWM_TBCTL_CTRMODE_SHIFT);  // up-count
  tbctl |= (g_hspdiv << CSL_EPWM_TBCTL_HSPCLKDIV_SHIFT);
  tbctl |= (g_clkdiv << CSL_EPWM_TBCTL_CLKDIV_SHIFT);
  tbctl |= (2 << CSL_EPWM_TBCTL_FREE_SOFT_SHIFT); // run free
  pwm->TBCTL = tbctl;

  pwm->TBPRD  = g_tbprd;    // period
  pwm->TBPHS  = 0;          // no phase

  uint16 cmpctl = 0;
  // shadowing enable, loading will occur when we hit CTR=ZERO
  cmpctl |= (0u << CSL_EPWM_CMPCTL_SHDWAMODE_SHIFT);  // 0 => SHADOW
  cmpctl |= (0u << CSL_EPWM_CMPCTL_LOADAMODE_SHIFT);  // 0 => load on ZRO
  cmpctl |= (0u << CSL_EPWM_CMPCTL_SHDWBMODE_SHIFT);  // OTHER HALF WILL BE THE SAME INSTR
  cmpctl |= (0u << CSL_EPWM_CMPCTL_LOADBMODE_SHIFT);
  pwm->CMPCTL = cmpctl;

  // UP AND DOWN we go.. driving UP and DOWN the pins
  pwm->AQCTLA = ( (2 << CSL_EPWM_AQCTLA_ZRO_SHIFT) | (1 << CSL_EPWM_AQCTLA_CAU_SHIFT) );
  pwm->AQCTLB = ( (2 << CSL_EPWM_AQCTLB_ZRO_SHIFT) | (1 << CSL_EPWM_AQCTLB_CBU_SHIFT) );

  // initial 1000us (min) pulse
  uint16 minLvl_counts = (uint16)(g_counts_per_us * g_params.min_us + 0.5);
  if (minLvl_counts > g_tbprd) minLvl_counts = g_tbprd;
  pwm->CMPA = minLvl_counts;
  pwm->CMPB = minLvl_counts;

  // we make sure this are not interfering 
  pwm->TZFRC = 0; 
  pwm->TZCLR = 0xFFFF;
}

bool init(const DFC_t_PWMgen_Params& params) {
  g_params = params;
  
  uint32 muxData = 0x10006u;
  if (Board_pinmuxSetReg(BOARD_SOC_DOMAIN_MAIN, static_cast<uint32>(PADCONFIG_OFFSET_REG89), muxData) == BOARD_SOK) {
      DebugP_log0("V29 pin succesfully set in EHRPWM output mode!\r\n");
  } else {
      DebugP_log0("V29 pin EHRPWM output mode switch failed..\r\n");
    return false;
  }
  if (Board_pinmuxSetReg(BOARD_SOC_DOMAIN_MAIN, static_cast<uint32>(PADCONFIG_OFFSET_REG90), muxData) == BOARD_SOK) {
      DebugP_log0("V27 pin succesfully set in ECAP input mode!\r\n");
  } else {
      DebugP_log0("V27 pin EHRPWM output mode switch failed..\r\n");
    return false;
  }

  if (Board_pinmuxSetReg(BOARD_SOC_DOMAIN_MAIN, static_cast<uint32>(PADCONFIG_OFFSET_REG94), muxData) == BOARD_SOK) {
      DebugP_log0("U27 pin succesfully set in EHRPWM output mode!\r\n");
  } else {
      DebugP_log0("U27 pin EHRPWM output mode switch failed..\r\n");
    return false;
  }

  if (Board_pinmuxSetReg(BOARD_SOC_DOMAIN_MAIN, static_cast<uint32>(PADCONFIG_OFFSET_REG95), muxData) == BOARD_SOK) {
      DebugP_log0("U24 pin succesfully set in EHRPWM output mode!\r\n");
  } else {
      DebugP_log0("U24 pin EHRPWM output mode switch failed..\r\n");
    return false;
  }

  // Power the two EHRPWM instances we use
  if (preInit(TISCI_DEV_EHRPWM0) != 0) return false;
  if (preInit(TISCI_DEV_EHRPWM2) != 0) return false;

  // Precompute dividers/period
  setupDividersPeriod(g_params.clk_hz, g_params.freq_hz, g_clkdiv, g_hspdiv, g_tbprd, g_counts_per_us);

  // Prepare base pointers
  volatile CSL_epwmRegs* pPWMInst0 = (volatile CSL_epwmRegs*)CSL_EHRPWM0_EPWM_BASE;
  volatile CSL_epwmRegs* pPWMInst2 = (volatile CSL_epwmRegs*)CSL_EHRPWM2_EPWM_BASE;

  // Setup both modules
  setupEPWM(pPWMInst0);
  setupEPWM(pPWMInst2);

  // Channel map
  g_map[0] = { pPWMInst0, true  };
  g_map[1] = { pPWMInst0, false };
  g_map[2] = { pPWMInst2, true  };
  g_map[3] = { pPWMInst2, false };

  return true;
}

inline uint16 us_to_counts(uint32 us) {
  if (us == 0) return 0;

  float64 c = g_counts_per_us * (float64)us;

  if (c < 1.0) c = 1.0;
  if (c > (float64)g_tbprd) c = (float64)g_tbprd;

  return (uint16)(c + 0.5);
}

void calibrateESC(uint32 maxDelay_ms, uint32 minDelay_ms) {
  write_motor_us(0, g_params.max_us);
  write_motor_us(1, g_params.max_us);
  write_motor_us(2, g_params.max_us);
  write_motor_us(3, g_params.max_us);
  vTaskDelay(pdMS_TO_TICKS(maxDelay_ms));
  
  write_motor_us(0, g_params.min_us);
  write_motor_us(1, g_params.min_us);
  write_motor_us(2, g_params.min_us);
  write_motor_us(3, g_params.min_us);
  vTaskDelay(pdMS_TO_TICKS(minDelay_ms));
}

bool detectESCCalibrationGesture(rc::C_RcIA6& rc, uint32 window_ms, uint32 gestureDuration_ms) {
  const uint32 gestureCaptureEnd = now_ms() + window_ms;
  uint32 currentHold_ms = 0;

  while (now_ms() < gestureCaptureEnd) {
    rc.update();
    const DFC_t_RcInputs rcData = rc.getData();
    const bool gesture = (rcData.thr > 0.80);

    if (gesture) {
      currentHold_ms += 10;
      if (currentHold_ms >= gestureDuration_ms) return true;
    } else {
      currentHold_ms = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return false;
}

bool detectFULLCalibrationGesture(rc::C_RcIA6& rc, uint32 window_ms, uint32 gestureDuration_ms) {
  const uint32 gestureCaptureEnd = now_ms() + window_ms;
  uint32 currentHold_ms = 0;

  while (now_ms() < gestureCaptureEnd) {
    rc.update();
    const DFC_t_RcInputs rcData = rc.getData();
    const bool gesture = (rcData.thr >= 0.8 ) && (rcData.pitch >= 0.8);

    if (gesture) {
      currentHold_ms += 10;
      if (currentHold_ms >= gestureDuration_ms) return true;
    } else {
      currentHold_ms = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return false;
}

void write_motor_us(uint8 ch, uint32 usec) {
  if (ch > 3) return;

  if (usec != 0) {
    if (usec < g_params.min_us) usec = g_params.min_us;
    if (usec > g_params.max_us) usec = g_params.max_us;
  }

  uint16 counts = us_to_counts(usec);
  volatile CSL_epwmRegs* pwm = g_map[ch].base;

  if (g_map[ch].isA) {
    pwm->CMPA = counts;
  }
  else {
    pwm->CMPB = counts;
  }
}

void outputWrapper(uint32 m0, uint32 m1, uint32 m2, uint32 m3) {
  write_motor_us(0, m0); write_motor_us(1, m1); write_motor_us(2, m2); write_motor_us(3, m3);
}

} // namespace PWMgen
