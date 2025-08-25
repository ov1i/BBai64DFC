#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

extern "C" {
#include <stdint.h>
#include <stddef.h>
#include <ti/drv/i2c/i2c.h>
#include <ti/drv/i2c/soc/i2c_soc.h>
}
#include <dfc_types.h>

#ifdef __cplusplus

namespace i2c {

class C_I2C {
    public:
        C_I2C() = default;
        ~C_I2C() = default;

        bool preinit(uint32 instance);
        bool init(uint32 instance, I2C_BitRate bitRate);
        bool writeRegSingletVal(uint8 dev_addr, const uint8 *reg_n_data);
        // bool rw(uint32 dev_addr, uint8 regAddr, uint8 *regData, uint8 size);
        bool rw(uint8 dev_addr, const uint8 *wbuffer, size_t wsize, uint8 *rbuffer, size_t rsize);

        I2C_Handle &getHandler();
    private:
        bool startModule(uint32 id);
        I2C_Handle m_handler;
};

} // namespace i2c
#endif

#endif // I2C_INTERFACE_H
