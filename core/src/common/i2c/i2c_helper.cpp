#include "i2c/i2c_helper.h"
extern "C" {
#include <ti/board/board.h>
#include <ti/board/src/devices/common/common.h>
#include <ti/csl/csl_timer.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/hw_types.h>
#include <ti/csl/soc.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/osal/TaskP.h>

}

namespace i2c {
bool C_I2C::startModule(uint32 id) {
  Sciclient_ConfigPrms_t config; 
  Sciclient_configPrmsInit(&config);
  sint32 status = Sciclient_init(&config);
  if (status && status != CSL_PASS) return false;
  
  struct tisci_msg_set_device_req  req  = {};
  struct tisci_msg_set_device_resp resp = {} ;

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
  if (status != CSL_PASS) return false;

  if ((respParam.flags & TISCI_MSG_FLAG_ACK) != TISCI_MSG_FLAG_ACK) {
    return false;
  }

  return true;
}

bool C_I2C::preinit(uint32 instance) {
  I2C_HwAttrs i2cCfg;
  Board_STATUS boardStatus = BOARD_SOK;
  sint32 retVal = CSL_SOK;
  uint32 baseAddr;
  uint32 muxData;

  switch (instance) {
  case 0:
    baseAddr = CSL_MCU_I2C0_CFG_BASE;
    break;
  case 1:
    baseAddr = CSL_MCU_I2C1_CFG_BASE;
    break;
  case 2:
    baseAddr = CSL_I2C0_CFG_BASE;
    break;
  case 3:
    baseAddr = CSL_I2C1_CFG_BASE;
    break;
  case 4:
    baseAddr = CSL_I2C2_CFG_BASE;
    if(!startModule(TISCI_DEV_I2C4)) return false;
    /// Y5 - pin
    muxData = 0x64002;
    boardStatus = Board_pinmuxSetReg(BOARD_SOC_DOMAIN_MAIN, static_cast<uint32>(PADCONFIG_OFFSET_REG120), muxData);
    if(boardStatus == BOARD_SOK) {
        DebugP_log0("Y5 pin succesfully set in i2c mode!\r\n");
    } else {
        DebugP_log0("Y5 pin i2c mode switch failed..\r\n");
      return false;
    }
    /// Y1 - pin
    boardStatus = Board_pinmuxSetReg(BOARD_SOC_DOMAIN_MAIN, static_cast<uint32>(PADCONFIG_OFFSET_REG119), muxData);
    if(boardStatus == BOARD_SOK) {
        DebugP_log0("Y1 pin succesfully set in i2c mode!\r\n");
    } else {
        DebugP_log0("Y1 pin i2c mode switch failed..\r\n");
      return false;
    }
    break;
  case 5:
    baseAddr = CSL_I2C3_CFG_BASE;
    break;
  case 6:
    baseAddr = CSL_I2C4_CFG_BASE;
    if(!startModule(TISCI_DEV_I2C6)) return false;
    /// AA3 - pin
    muxData = 0x64002;
    boardStatus = Board_pinmuxSetReg(BOARD_SOC_DOMAIN_MAIN, static_cast<uint32>(PADCONFIG_OFFSET_REG116), muxData);
    if(boardStatus == BOARD_SOK) {
        DebugP_log0("AA3 pin succesfully set in i2c mode!\r\n");
    } else {
        DebugP_log0("AA3 pin i2c mode switch failed..\r\n");
      return false;
    }
    /// Y2 - pin
    boardStatus = Board_pinmuxSetReg(BOARD_SOC_DOMAIN_MAIN, static_cast<uint32>(PADCONFIG_OFFSET_REG121), muxData);
    if(boardStatus == BOARD_SOK) {
        DebugP_log0("Y2 pin succesfully set in i2c mode!\r\n");
    } else {
        DebugP_log0("Y2 pin i2c mode switch failed..\r\n");
      return false;
    }
    break;
  default:
    retVal = CSL_EFAIL;
    break;
  }

  if (retVal == CSL_EFAIL) {
    return false;
  }

  if (I2C_socGetInitCfg(instance, &i2cCfg) == I2C_STATUS_ERROR) {
    return false;
  }

  i2cCfg.baseAddr = baseAddr;
  i2cCfg.enableIntr = BFALSE;

  if (I2C_socSetInitCfg(instance, &i2cCfg) == I2C_STATUS_ERROR) {
    return false;
  }

  I2C_init();

  return true;
}

bool C_I2C::init(uint32 instance, I2C_BitRate bitRate) {
  I2C_Params i2cParams;

  if (instance >= I2C_HWIP_MAX_CNT) {
      DebugP_log0("I2C init error: instance out of range \n");
    return false;
  }

  preinit(instance);

  I2C_Params_init(&i2cParams);
  i2cParams.transferMode = I2C_MODE_BLOCKING;
  i2cParams.bitRate = bitRate;

  m_handler = I2C_open(instance, &i2cParams);
  if (m_handler == NULL) {
      DebugP_log0("I2C init error: I2C_open() failed\n");
    return false;
  }

  //   Sciclient_pmSetModuleState(TISCI_DEV_I2C6,
  //   TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
  //                              TISCI_MSG_FLAG_AOP,
  //                              SCICLIENT_SERVICE_WAIT_FOREVER);
  //     DebugP_log0("Took owenership of i2c instance %u\n", instance);

  //   Sciclient_pmSetModuleRst(TISCI_DEV_I2C6, 1,
  //   SCICLIENT_SERVICE_WAIT_FOREVER); Sciclient_pmSetModuleRst(TISCI_DEV_I2C6,
  //   0, SCICLIENT_SERVICE_WAIT_FOREVER);   DebugP_log0("Reset performed on the
  //     i2c instance %u\n", instance);

    DebugP_log0("\n\rProbing in progress...\r\n");
  for (uint16 slaveAddr = 0U; slaveAddr < 128U; slaveAddr++) {
    if (I2C_STATUS_SUCCESS == I2C_control(m_handler, I2C_CMD_PROBE, &slaveAddr)) {
        DebugP_log0("I2C inst passed for local addr!!! \r\n");
    } else {
        DebugP_log0("I2C inst failed for local addr!!! \r\n");
    }
  }

    DebugP_log0("I2C instance initialized successfully\n");

  return true;
}

bool C_I2C::writeRegSingletVal(uint8 dev_addr, const uint8 *reg_n_data) {
  if (!m_handler || !reg_n_data) {
      DebugP_log0("writeRegSingletVal: Invalid arguments\n");
    return false;
  }

  I2C_Transaction i2cTrans;
  I2C_transactionInit(&i2cTrans);

  i2cTrans.slaveAddress = static_cast<uint32>(dev_addr);
  i2cTrans.writeBuf = const_cast<uint8 *>(reg_n_data);
  i2cTrans.writeCount = 2;
  i2cTrans.readBuf = nullptr;
  i2cTrans.readCount = 0;
  i2cTrans.timeout = 10000;

  sint16 errCode = I2C_transfer(m_handler, &i2cTrans);
  if (errCode != I2C_STS_SUCCESS) {
      DebugP_log0("writeRegSingletVal: I2C_transfer failed\n");
      DebugP_log0("writeRegSingletVal: I2C_transfer error\r\n");
    return false;
  }

  return true;
}

// bool C_I2C::rw(uint32 dev_addr, uint8 regAddr, uint8 *regData, uint8 size) {
//   Board_STATUS status = BOARD_SOK;
//   I2C_Transaction transaction;

//   I2C_transactionInit(&transaction);

//   transaction.slaveAddress = dev_addr;
//   transaction.writeBuf = &regAddr;
//   transaction.writeCount = 1;
//   transaction.readBuf = NULL;
//   transaction.readCount = 0;
//   transaction.timeout = 10000;

//   status = I2C_transfer(m_handler, &transaction);
//   if (status != I2C_STS_SUCCESS) {
//       DebugP_log0(
//         "Failing while transmitting the rd reg addr with error code - %d\n",
//         status);
//     return false;
//   }

//   transaction.writeBuf = NULL;
//   transaction.writeCount = 0;
//   transaction.readBuf = regData;
//   transaction.readCount = size;

//   status = I2C_transfer(m_handler, &transaction);
//   if (status != I2C_STS_SUCCESS) {
//       DebugP_log0("Failing while reading the register data by returning - %d\n",
//                 status);
//     return false;
//   }

//   return true;
// }

bool C_I2C::rw(uint8 dev_addr, const uint8 *wbuffer, size_t wsize,
               uint8 *rbuffer, size_t rsize) {
  if (!wbuffer || wsize == 0 || rsize == 0) {
      DebugP_log0("rw: Invalid arguments\n");
    return false;
  }

  I2C_Transaction i2cTrans;
  I2C_transactionInit(&i2cTrans);

  i2cTrans.slaveAddress = dev_addr;
  i2cTrans.writeBuf = const_cast<uint8 *>(wbuffer);
  i2cTrans.writeCount = wsize;
  i2cTrans.readBuf = rbuffer;
  i2cTrans.readCount = rsize;
  i2cTrans.timeout = 10000;
  
  bool success = I2C_transfer(m_handler, &i2cTrans);
  if (!success) {
      DebugP_log0("i2cWriteRead: I2C_transfer failed\n");
  }
  return success;
}

I2C_Handle &C_I2C::getHandler() { return m_handler; }

} // namespace i2c
