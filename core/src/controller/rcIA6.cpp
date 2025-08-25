#include "RCInputECAP.hp"

extern "C" {
#include <ti/drv/sciclient/include/tisci/tisci_devices.h>
#include <ti/drv/sciclient/sciclient.h>

#include <drivers/hw_include/cslr_ecap.h>
#include <drivers/hw_include/cslr_soc.h>

#include <ti/board/board.h>
#include <ti/board/src/board_internal.h>

#include <FreeRTOS.h>
#include <task.h>
}

namespace rc {

static inline uint32 now_ms() {
  return (uint32)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

sint32 C_RcIA6::preInit(uint16 id) {
  Sciclient_ConfigPrms_t config; 
  Sciclient_configPrmsInit(&config);
  int32_t st = Sciclient_init(&config);
  if (st && st != SCICLIENT_EALREADY_OPEN)
    return st;

  struct tisci_msg_set_device_req req = { 0 };
  struct tisci_msg_set_device_resp resp = { 0 };

  Sciclient_ReqPrm_t reqParam  = { 0 };
  Sciclient_RespPrm_t respParam = { 0 };

  req.id         = id;
  req.state      = TISCI_MSG_VALUE_DEVICE_SW_STATE_ON;
  req.hdr.type   = TISCI_MSG_SET_DEVICE;
  req.hdr.host   = TISCI_HOST_ID_MAIN_0;
  req.hdr.flags  = TISCI_MSG_FLAG_AOP;

  reqParam.messageType    = TISCI_MSG_SET_DEVICE_STATE;
  reqParam.pReqPayload    = (const uint8*)&req;
  reqParam.reqPayloadSize = (uint32)sizeof(req);
  reqParam.timeout        = SCICLIENT_SERVICE_WAIT_FOREVER;

  respParam.pRespPayload    = (uint8*)&resp;
  respParam.respPayloadSize = (uint32)sizeof(resp);

  status = Sciclient_service(&reqParam, &respParam);
  if (status != CSL_PASS) return status;

  if ((sresp.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK) {
    return CSL_EFAIL;
  }
  
  return CSL_PASS;
}

void C_RcIA6::initCaptureEvents(volatile CSL_ecapRegs *ecap) {
  ecap->ECCTL2 = 0; // capture mode, counter stopped
  ecap->ECCLR = 0xFFFF; // clear all pending events start, I'm freshhh I'm cleannn
  ecap->ECEINT = 0x0000; // don't interrupt me

  // E1=rising, E2=falling, reset on E2; no prescale;
  ecap->ECCTL1 = CSL_ECAP_ECCTL1_CAPLDEN_MASK |
                 (0 << CSL_ECAP_ECCTL1_PRESCALE_SHIFT) |
                 (0 << CSL_ECAP_ECCTL1_CAP1POL_SHIFT) |
                 (1 << CSL_ECAP_ECCTL1_CAP2POL_SHIFT) |
                 (0 << CSL_ECAP_ECCTL1_CAP3POL_SHIFT) |
                 (1 << CSL_ECAP_ECCTL1_CAP4POL_SHIFT) |
                 (0 << CSL_ECAP_ECCTL1_CTRRST1_SHIFT) |
                 (1 << CSL_ECAP_ECCTL1_CTRRST2_SHIFT) |
                 (0 << CSL_ECAP_ECCTL1_CTRRST3_SHIFT) |
                 (0 << CSL_ECAP_ECCTL1_CTRRST4_SHIFT);

  // continuous, wrap at event2, start counter
  ecap->ECCTL2 = (0 << CSL_ECAP_ECCTL2_CONT_ONESHT_SHIFT) |
                 (1 << CSL_ECAP_ECCTL2_STOP_WRAP_SHIFT) |
                 (0 << CSL_ECAP_ECCTL2_REARM_SHIFT) |
                 (0 << CSL_ECAP_ECCTL2_SYNCI_EN_SHIFT) |
                 (0 << CSL_ECAP_ECCTL2_SYNCO_SEL_SHIFT) |
                 (1 << CSL_ECAP_ECCTL2_TSCTRSTOP_SHIFT) |
                 (0 << CSL_ECAP_ECCTL2_CAP_APWM_SHIFT);
}

bool C_RcIA6::read(volatile CSL_ecapRegs *ecap, uint32 &cap1, uint32 &cap2) {
  uint16 flg = ecap->ECFLG;
  if (flg & CSL_ECAP_ECFLG_CEVT2_MASK) {
    cap1 = ecap->CAP1; // event 1 (rising)
    cap2 = ecap->CAP2; // event 2 (falling)
    ecap->ECCLR = CSL_ECAP_ECCLR_CEVT2_MASK | CSL_ECAP_ECCLR_INT_MASK;
    return true;
  }
  return false;
}

bool C_RcIA6::init(const DFC_t_RcParams &params) {
    m_Params = params;
    m_Data = {};
    m_Data.mode = params.default_mode;

  uint32 muxData = 0x30000h;
  if (Board_pinmuxSetReg(BOARD_SOC_DOMAIN_MAIN, static_cast<uint32>(PADCONFIG_OFFSET_REG140), muxData) != BOARD_SOK) {
    UART_printf("U2 pin succesfully set in ECAP input mode (settings: %X)!\r\n", muxData);
    return false;
  } else {
    UART_printf("U2 pin ECAP input mode switch failed..\r\n");
  }
  muxData = 0x30001h;
  if (Board_pinmuxSetReg(BOARD_SOC_DOMAIN_MAIN, static_cast<uint32>(PADCONFIG_OFFSET_REG142), muxData) != BOARD_SOK) {
    UART_printf("V6 pin succesfully set in ECAP input mode (settings: %X)!\r\n", muxData);
    return false;
  } else {
    UART_printf("V6 pin ECAP input mode switch failed..\r\n");
  }

  if (Board_pinmuxSetReg(BOARD_SOC_DOMAIN_MAIN, static_cast<uint32>(PADCONFIG_OFFSET_REG143), muxData) != BOARD_SOK) {
    UART_printf("V5 pin succesfully set in ECAP input mode (settings: %X)!\r\n", muxData);
    return false;
  } else {
    UART_printf("V5 pin ECAP input mode switch failed..\r\n");
  }

  // Enable ECAP0/1/2 clocks
  if (preInit(TISCI_DEV_ECAP0) != 0)
    return false;
  if (preInit(TISCI_DEV_ECAP1) != 0)
    return false;
  if (preInit(TISCI_DEV_ECAP2) != 0)
    return false;

  // Configure capture on all three PINS
  initCaptureEvents((volatile CSL_ecapRegs *)CSL_ECAP0_CTL_STS_BASE); // P9.28 - Throttle
  initCaptureEvents((volatile CSL_ecapRegs *)CSL_ECAP1_CTL_STS_BASE); // P9.30 - Pitch
  initCaptureEvents((volatile CSL_ecapRegs *)CSL_ECAP2_CTL_STS_BASE); // P9.29 - Roll

  // Prime filters
  m_NormThrottle = fmap(1500.0, m_Params.thr_min_us, m_Params.thr_max_us, 0.0, 1.0);
  m_NormRoll = 0.0;
  m_NormlPitch = 0.0;

  m_LastStep_ms = now_ms();
  m_ArmHold_ms = 0;
  m_ModeHold_ms = 0;

  return true;
}

void C_RcIA6::update() {
  volatile CSL_ecapRegs *pECAP0 = (volatile CSL_ecapRegs *)CSL_ECAP0_CTL_STS_BASE; // throttle
  volatile CSL_ecapRegs *pECAP1 = (volatile CSL_ecapRegs *)CSL_ECAP1_CTL_STS_BASE; // pitch
  volatile CSL_ecapRegs *pECAP2 = (volatile CSL_ecapRegs *)CSL_ECAP2_CTL_STS_BASE; // roll

  const float64 ticks_to_us = 1e6 / m_Params.ecap_clk_hz;

  uint32 now = now_ms();
  uint32 dt = now - m_LastStep_ms;
  m_LastStep_ms = now;

  uint32 cap1, cap2;
  if (read(pECAP0, cap1, cap2)) {
    m_Throttle_cap1 = cap1;
    m_Throttle_cap2 = cap2;
    m_Throttle_ms = now;
  }

  if (read(pECAP1, cap1, cap2)) {
    m_Pitch_cap1 = cap1;
    m_Pitch_cap2 = cap2;
    m_Pitch_ms = now;
  }

  if (read(pECAP2, cap1, cap2)) {
    m_Roll_cap1 = cap1;
    m_Roll_cap2 = cap2;
    m_Roll_ms = now;
  }

  bool fs_thr = (now - m_Throttle_ms) > m_Params.timeout_ms;
  bool fs_roll = (now - m_Roll_ms) > m_Params.timeout_ms;
  bool fs_pitch = (now - m_Pitch_ms) > m_Params.timeout_ms;

  if (fs_thr || fs_roll || fs_pitch) {
    m_Data.thr = 0.0;
    m_Data.roll = m_Data.pitch = m_Data.yaw = 0.0;
    m_Data.arm = false;
    m_ArmHold_ms = 0;
  } else {
    float64 thr_us = (float64)(m_Throttle_cap2 - m_Throttle_cap1) * ticks_to_us;
    float64 roll_us = (float64)(m_Roll_cap2 - m_Roll_cap1) * ticks_to_us;
    float64 pitch_us = (float64)(m_Pitch_cap2 - m_Pitch_cap1) * ticks_to_us;

    thr_us = clampf(thr_us, 800.0, 2200.0);
    roll_us = clampf(roll_us, 800.0, 2200.0);
    pitch_us = clampf(pitch_us, 800.0, 2200.0);

    float64 thr_raw = fmap(thr_us, m_Params.thr_min_us, m_Params.thr_max_us, 0.0, 1.0);
    float64 roll_raw = pwm_to_sym(roll_us, m_Params.center_us, m_Params.span_half_us);
    float64 pitch_raw = pwm_to_sym(pitch_us, m_Params.center_us, m_Params.span_half_us);

    roll_raw = apply_deadband(roll_raw, m_Params.deadband);
    pitch_raw = apply_deadband(pitch_raw, m_Params.deadband);

    if (m_Params.invert_roll)
      roll_raw = -roll_raw;
    if (m_Params.invert_pitch)
      pitch_raw = -pitch_raw;

    m_NormThrottle = lpf(m_NormThrottle, thr_raw, m_Params.lpf_alpha);
    m_NormRoll = lpf(m_NormRoll, roll_raw, m_Params.lpf_alpha);
    m_NormlPitch = lpf(m_NormlPitch, pitch_raw, m_Params.lpf_alpha);

    m_Data.thr = clampf(m_NormThrottle, 0.0, 1.0);
    m_Data.roll = clampf(m_NormRoll, -1.0, 1.0);
    m_Data.pitch = clampf(m_NormlPitch, -1.0, 1.0);
    m_Data.yaw = 0.0;

    if (!m_Data.arm) {
      if (m_Data.thr < 0.05 && m_Data.roll > 0.8) {
        m_ArmHold_ms += dt;
        if (m_ArmHold_ms > 700) {
          m_Data.arm = true;
          m_ArmHold_ms = 0;
        }
      } else {
        m_ArmHold_ms = 0;
      }
    } else {
      if (m_Data.thr < 0.05 && m_Data.roll < -0.8) {
        m_Data.arm = false;
        m_ArmHold_ms = 0;
      }
    }

    if (m_Data.thr < 0.05) {
      bool center = (std::fabs(m_Data.roll) < 0.20) && (std::fabs(m_Data.pitch) < 0.20);

      if (m_Data.pitch > 0.80) {
        m_ModeHold_ms += dt;
        if (m_ModeHold_ms > 700) {
          m_Data.mode = DFC_t_Mode::ANGLE;
          m_ModeHold_ms = 0;
        }
      } else if (m_Data.pitch < -0.80) {
        m_ModeHold_ms += dt;
        if (m_ModeHold_ms > 700) {
          m_Data.mode = DFC_t_Mode::ACRO;
          m_ModeHold_ms = 0;
        }
      } else if (center) {
        m_ModeHold_ms += dt;
        if (m_ModeHold_ms > 700) {
          m_Data.mode = DFC_t_Mode::POS_HOLD;
          m_ModeHold_ms = 0;
        }
      } else {
        m_ModeHold_ms = 0;
      }
    } else {
      m_ModeHold_ms = 0;
    }
  }
}

} // namespace rc
