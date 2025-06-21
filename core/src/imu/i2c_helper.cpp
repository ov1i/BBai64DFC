#include "imu/i2c_helper.h"
extern "C" {
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
}

///TODO: Change UART_printf to logger thread
namespace i2c {
    bool C_I2C::init(uint32 instance) {
        I2C_Params i2cParams;

        ///TODO: Add checks for I2C occupied by other core/task
        if (instance >= I2C_HWIP_MAX_CNT) {
            UART_printf("I2C init error: instance %u out of range (max %u)\n", instance, I2C_HWIP_MAX_CNT - 1);
            return false;
        }

        I2C_Params_init(&i2cParams);
        i2cParams.transferMode = I2C_MODE_BLOCKING;
        i2cParams.bitRate = I2C_100kHz;

        m_handler = I2C_open(instance, &i2cParams);
        if (m_handler == nullptr) {
            UART_printf("I2C init error: I2C_open() failed for instance %u\n", instance);
            return false;
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

        i2cTrans.slaveAddress = dev_addr;
        i2cTrans.writeBuf = const_cast<uint8*>(reg_n_data);
        i2cTrans.writeCount = 2;
        i2cTrans.readBuf = nullptr;
        i2cTrans.readCount = 0;

        bool success = I2C_transfer(m_handler, &i2cTrans);
        if (!success) {
            UART_printf("writeRegSingletVal: I2C_transfer failed (dev 0x%02X)\n", dev_addr);
        }
        return success;
    }

    bool C_I2C::rw(uint8 dev_addr, const uint8 *wbuffer, size_t wsize, uint8 *rbuffer, size_t rsize) { 
        if (!wbuffer || wsize == 0 || rsize == 0) {
            UART_printf("rw: Invalid arguments\n");
            return false;
        }

        I2C_Transaction i2cTrans;
        I2C_transactionInit(&i2cTrans);

        i2cTrans.slaveAddress = dev_addr;
        i2cTrans.writeBuf = const_cast<uint8*>(wbuffer);
        i2cTrans.writeCount = wsize;
        i2cTrans.readBuf = rbuffer;
        i2cTrans.readCount = rsize;

        bool success = I2C_transfer(m_handler, &i2cTrans);
        if (!success) {
            UART_printf("i2cWriteRead: I2C_transfer failed (dev 0x%02X)\n", dev_addr);
        }
        return success;
        }

        I2C_Handle &C_I2C::getHandler() {
            return m_handler;
        }

} // namespace i2c


