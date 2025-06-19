#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include <stdint.h>
#include <stddef.h>
#include <ti/drv/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t base_addr;  
    uint32_t speed_hz;   
    uint8_t instance;
} i2c_config_t;

int i2c_init(const i2c_config_t *config);
int i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);
int i2c_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // I2C_INTERFACE_H
