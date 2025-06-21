#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include <stdint.h>
#include <stddef.h>
extern "C" {
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

        bool init(uint32 instance);
        bool writeRegSingletVal(uint8 dev_addr, const uint8 *reg_n_data);
        bool rw(uint8 dev_addr, const uint8 *wbuffer, size_t wsize, uint8 *rbuffer, size_t rsize);

        I2C_Handle &getHandler();
    private:
        I2C_Handle m_handler;
};

} // namespace i2c
#endif

#endif // I2C_INTERFACE_H
