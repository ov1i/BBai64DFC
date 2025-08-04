#include "i2c/i2c_helper.h"
extern "C" {
#include <ti/board/board.h>
#include <ti/board/src/devices/common/common.h>
#include <ti/csl/csl_timer.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/hw_types.h>
#include <ti/csl/soc.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
}

/// TODO: Change UART_printf to logger thread
namespace i2c {
bool C_I2C::preinit(uint32 instance) {

  uint32 baseAddr;
  I2C_HwAttrs i2cCfg;
  sint32 retVal = CSL_SOK;

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
    break;
  case 5:
    baseAddr = CSL_I2C3_CFG_BASE;
    break;
  case 6:
    baseAddr = CSL_I2C4_CFG_BASE;
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

bool C_I2C::init(uint32 instance, uint32 bitRate) {
  I2C_Params i2cParams;

  /// TODO: Add checks for I2C occupied by other core/task
  if (instance >= I2C_HWIP_MAX_CNT) {
    UART_printf("I2C init error: instance %u out of range (max %u)\n", instance,
                I2C_HWIP_MAX_CNT - 1);
    return false;
  }

  preinit(instance);

  I2C_Params_init(&i2cParams);
  i2cParams.transferMode = I2C_MODE_BLOCKING;
  i2cParams.bitRate = bitRate;

  m_handler = I2C_open(instance, &i2cParams);
  if (m_handler == NULL) {
    UART_printf("I2C init error: I2C_open() failed for instance %u\n",
                instance);
    return false;
  }

  //   Sciclient_pmSetModuleState(TISCI_DEV_I2C6,
  //   TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
  //                              TISCI_MSG_FLAG_AOP,
  //                              SCICLIENT_SERVICE_WAIT_FOREVER);
  //   UART_printf("Took owenership of i2c instance %u\n", instance);

  //   Sciclient_pmSetModuleRst(TISCI_DEV_I2C6, 1,
  //   SCICLIENT_SERVICE_WAIT_FOREVER); Sciclient_pmSetModuleRst(TISCI_DEV_I2C6,
  //   0, SCICLIENT_SERVICE_WAIT_FOREVER); UART_printf("Reset performed on the
  //     i2c instance %u\n", instance);

  UART_printf("\n\rProbing for %d is in progress...\r\n", instance);
  for (uint16 slaveAddr = 0U; slaveAddr < 128U; slaveAddr++) {
    if (I2C_STATUS_SUCCESS ==
        I2C_control(m_handler, I2C_CMD_PROBE, &slaveAddr)) {
      UART_printf("I2C%d: Passed for address 0x%x !!! \r\n", instance,
                  slaveAddr);
    } else {
      UART_printf("I2C%d: Failed for address 0x%x !!! \r\n", instance,
                  slaveAddr);
    }
  }

  UART_printf("I2C instance %u initialized successfully\n", instance);

  return true;
}

bool C_I2C::writeRegSingletVal(uint8 dev_addr, const uint8 *reg_n_data) {
  if (!m_handler || !reg_n_data) {
    UART_printf("writeRegSingletVal: Invalid arguments\n");
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
    UART_printf("writeRegSingletVal: I2C_transfer failed (dev 0x%02X)\n",
                dev_addr);
    UART_printf("writeRegSingletVal: I2C_transfer error code -> %d \r\n",
                errCode);
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
//     UART_printf(
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
//     UART_printf("Failing while reading the register data by returning - %d\n",
//                 status);
//     return false;
//   }

//   return true;
// }

bool C_I2C::rw(uint8 dev_addr, const uint8 *wbuffer, size_t wsize,
               uint8 *rbuffer, size_t rsize) {
  if (!wbuffer || wsize == 0 || rsize == 0) {
    UART_printf("rw: Invalid arguments\n");
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
    UART_printf("i2cWriteRead: I2C_transfer failed (dev 0x%02X)\n", dev_addr);
  }
  return success;
}

I2C_Handle &C_I2C::getHandler() { return m_handler; }

} // namespace i2c
