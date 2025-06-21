#include "imu/mpu9250.h"
#include <utils.h>
#include <dfc_types.h>

extern "C" {
#include <ti/csl/csl_types.h>
#include <ti/csl/csl_utils.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
}

namespace imu {
    bool C_IMU::init() {

        m_i2cHandler.init(6);

        uint8 reg_n_data[2];
        uint8 temp;
        uint8 buffer[3];

        // 1. Check MPU WHO_AM_I
        if (!m_i2cHandler.rw(MPU9250_ADDRESS, &MPU9250_WHO_AM_I_GENERAL, 1, &temp, 1) || temp != 113) {
            UART_printf("[DEBUG]: WHO_AM_I failed: %d\r\n", temp);
            return false;
        }
        UART_printf("[DEBUG]: WHO_AM_I...[ SUCCESS ]\r\n");

        // 2. Set clock to X gyro
        reg_n_data[0] = MPU9250_PWR_MGMNT_1;
        reg_n_data[1] = 0x01;
        if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
            !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) || temp != 1) {
            UART_printf("[DEBUG]: Clock source...[ FAILED ]\r\n");
            return false;
        }
        UART_printf("[DEBUG]: Clock source...[ SUCCESS ]\r\n");

        // 3. Accelerometer scale = ±4g (0x08)
        reg_n_data[0] = MPU9250_ACC_CONFIG_1;
        reg_n_data[1] = 0x08;
        if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
            !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) || temp != 8) {
            UART_printf("[DEBUG]: Acc scale...[ FAILED ]\r\n");
            return false;
        }
        UART_printf("[DEBUG]: Acc scale...[ SUCCESS ]\r\n");

        // 4. Gyroscope scale = ±500 dps (0x08)
        reg_n_data[0] = MPU9250_GYRO_CONFIG;
        reg_n_data[1] = 0x08;
        if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
            !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) || temp != 8) {
            UART_printf("[DEBUG]: Gyro scale...[ FAILED ]\r\n");
            return false;
        }
        UART_printf("[DEBUG]: Gyro scale...[ SUCCESS ]\r\n");

        // 5. Low-pass filter for acc (10Hz, 0x05)
        reg_n_data[0] = MPU9250_ACC_CONFIG_2;
        reg_n_data[1] = 0x05;
        if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
            !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) || temp != 5) {
            UART_printf("[DEBUG]: Acc LPF...[ FAILED ]\r\n");
            return false;
        }
        UART_printf("[DEBUG]: Acc LPF...[ SUCCESS ]\r\n");

        // 6. Low-pass filter for gyro (10Hz, 0x05)
        reg_n_data[0] = MPU9250_CONFIG;
        reg_n_data[1] = 0x05;
        if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
            !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) || temp != 5) {
            UART_printf("[DEBUG]: Gyro LPF...[ FAILED ]\r\n");
            return false;
        }
        UART_printf("[DEBUG]: Gyro LPF...[ SUCCESS ]\r\n");

        // 7. Disable I2C master (enable bypass to AK8963 mag)
        reg_n_data[0] = MPU9250_USER_CTRL;
        reg_n_data[1] = 0x00;
        if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
            !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) || temp != 0) {
            UART_printf("[DEBUG]: I2C master disable...[ FAILED ]\r\n");
            return false;
        }
        UART_printf("[DEBUG]: I2C master disable...[ SUCCESS ]\r\n");

        // 8. Enable bypass multiplexer
        reg_n_data[0] = MPU9250_INT_PIN_CONFIG;
        reg_n_data[1] = 0x02;
        if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
            !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) || temp != 2) {
            UART_printf("[DEBUG]: Bypass enable...[ FAILED ]\r\n");
            return false;
        }
        UART_printf("[DEBUG]: Bypass enable...[ SUCCESS ]\r\n");

        // --- Magnetometer (AK8963) ---
        // 9. Check AK8963 WHO_AM_I
        uint8 mag_whoami_reg = MPU9250_AK8963_DEVICE_ID;
        if (!m_i2cHandler.rw(MPU9250_AK8963_ADDR, &mag_whoami_reg, 1, &temp, 1) || temp != 72) {
            UART_printf("[DEBUG]: MAG WHO_AM_I...[ FAILED ]\r\n");
            return false;
        }
        UART_printf("[DEBUG]: MAG WHO_AM_I...[ SUCCESS ]\r\n");

        // 10. Enter Fuse ROM access mode (for sensitivity adjustment)
        reg_n_data[0] = MPU9250_MAG_CONTROL_CONFIG;
        reg_n_data[1] = 0x0F;
        if (!m_i2cHandler.writeRegSingletVal(MPU9250_AK8963_ADDR, reg_n_data)) {
            UART_printf("[DEBUG]: MAG Fuse ROM...[ FAILED ]\r\n");
            return false;
        }

        // 11. Read sensitivity adjustment values
        uint8 asax_reg = MPU9250_MAG_ASAX_CONFIG;
        if (!m_i2cHandler.rw(MPU9250_AK8963_ADDR, &asax_reg, 1, buffer, 3)) {
            UART_printf("[DEBUG]: MAG ASA read...[ FAILED ]\r\n");
            return false;
        }
        float32 mag_sens[3];
        mag_sens[0] = ((buffer[0]-128)*0.5/128.0)+1.0;
        mag_sens[1] = ((buffer[1]-128)*0.5/128.0)+1.0;
        mag_sens[2] = ((buffer[2]-128)*0.5/128.0)+1.0;

        // 12. Power down mag
        reg_n_data[0] = MPU9250_MAG_CONTROL_CONFIG;
        reg_n_data[1] = 0x00;
        if (!m_i2cHandler.writeRegSingletVal(MPU9250_AK8963_ADDR, reg_n_data)) {
            UART_printf("[DEBUG]: MAG power down...[ FAILED ]\r\n");
            return false;
        }

        // 13. Continuous mode 2 (100Hz, 16-bit)
        reg_n_data[0] = MPU9250_MAG_CONTROL_CONFIG;
        reg_n_data[1] = 0x16;
        if (!m_i2cHandler.writeRegSingletVal(MPU9250_AK8963_ADDR, reg_n_data)) {
            UART_printf("[DEBUG]: MAG continuous mode...[ FAILED ]\r\n");
            return false;
        }

        UART_printf("[DEBUG]: MPU9250 init complete!\r\n");
        return true;
    }

    bool C_IMU::update() {
        uint8 wbuffer[1] = {MPU9250_ACC_X_H}; // Usually 0x3B
        uint8 rbuffer[14]; // 6 accel, 2 temp, 6 gyro

        // Read 14 bytes: accel (6), temp (2), gyro (6)
        if (!m_i2cHandler.rw(MPU9250_ADDRESS, wbuffer, 1, rbuffer, 14)) {
            UART_printf("Failed to read data from MPU9250\r\n");
            return false;
        }

        // Accel
        m_rawData.ax = static_cast<float64>((sint16)((rbuffer[0] << 8) | rbuffer[1]));
        m_rawData.ay = static_cast<float64>((sint16)((rbuffer[2] << 8) | rbuffer[3]));
        m_rawData.az = static_cast<float64>((sint16)((rbuffer[4] << 8) | rbuffer[5]));

        // Gyro
        m_rawData.gx = static_cast<float64>((sint16)((rbuffer[8]  << 8) | rbuffer[9]));
        m_rawData.gy = static_cast<float64>((sint16)((rbuffer[10] << 8) | rbuffer[11]));
        m_rawData.gz = static_cast<float64>((sint16)((rbuffer[12] << 8) | rbuffer[13]));

        uint8 mag_status1_reg = 0x02;
        uint8 status1;
        if (!m_i2cHandler.rw(MPU9250_AK8963_ADDR, &mag_status1_reg, 1, &status1, 1)) {
            UART_printf("Failed to read AK8963 status1\r\n");
            return false;
        }
        if (!(status1 & 0x01)) { // Data not ready
            UART_printf("Magnetometer data not ready\r\n");
            return false;
        }

        // 2. Read mag data (0x03 to 0x08, 7 bytes: 6 data, 1 status2)
        uint8 mag_data_reg = 0x03;
        uint8 mag_data[7];
        if (!m_i2cHandler.rw(MPU9250_AK8963_ADDR, &mag_data_reg, 1, mag_data, 7)) {
            UART_printf("Failed to read AK8963 mag data\r\n");
            return false;
        }

        // 3. Check for magnetic overflow (status2 bit 3)
        if (mag_data[6] & 0x08) {
            UART_printf("Magnetometer overflow!\r\n");
            return false;
        }

        // 4. Parse mag data (little endian!)
        m_rawData.mx = static_cast<float64>((sint16)((mag_data[1] << 8) | mag_data[0]));
        m_rawData.my = static_cast<float64>((sint16)((mag_data[3] << 8) | mag_data[2]));
        m_rawData.mz = static_cast<float64>((sint16)((mag_data[5] << 8) | mag_data[4]));

        // Temp (optional)
        sint16 temp_raw = (rbuffer[6] << 8) | rbuffer[7];
        m_rawData.temp = static_cast<float64>(temp_raw);

        return true;
    }

    DFC_t_MPU9250_Data &C_IMU::getCurrentRawData() {
        return m_rawData;
    }

}  // namespace imu
