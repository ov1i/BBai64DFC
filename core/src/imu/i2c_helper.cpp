#include "i2c_helper.h"
#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/I2C_v1.h>
#include <ti/drv/i2c/soc/I2C_soc.h>

bool i2cWrite(I2C_Handle handle, uint8_t devAddr, uint8_t *data, size_t len) {
    I2C_Transaction i2cTrans;
    I2C_transactionInit(&i2cTrans);
    i2cTrans.slaveAddress = devAddr;
    i2cTrans.writeBuf = data;
    i2cTrans.writeCount = len;
    i2cTrans.readBuf = NULL;
    i2cTrans.readCount = 0;
    return I2C_transfer(handle, &i2cTrans);
}

bool i2cWriteRead(I2C_Handle handle, uint8_t devAddr, uint8_t *wdata, size_t wlen, uint8_t *rdata, size_t rlen) {
    I2C_Transaction i2cTrans;
    I2C_transactionInit(&i2cTrans);
    i2cTrans.slaveAddress = devAddr;
    i2cTrans.writeBuf = wdata;
    i2cTrans.writeCount = wlen;
    i2cTrans.readBuf = rdata;
    i2cTrans.readCount = rlen;
    return I2C_transfer(handle, &i2cTrans);
}

I2C_Handle i2cInit(uint32_t instance) {
    I2C_Params i2cParams;
    I2C_Handle handle;
    I2C_Params_init(&i2cParams);
    i2cParams.transferMode = I2C_MODE_BLOCKING;
    i2cParams.bitRate = I2C_100kHz;
    handle = I2C_open(instance, &i2cParams);
    return handle;
}
