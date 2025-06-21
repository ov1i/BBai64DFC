extern "C" {
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <ti/osal/TaskP.h>
#include <ti/osal/SemaphoreP.h>
#include <ti/osal/QueueP.h>
#include <ti/osal/TimerP.h>
#include <ti/osal/osal.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
}
#include "comms/logger.h"
#include "imu/mpu9250.h"
#include "dfc_types.h"


static uint8 gTaskStackIMU[IMU_TASKSTACKSIZE]                   __attribute__((aligned(32)));
static uint8 gTaskStackEKF[EKF_TASKSTACKSIZE]                   __attribute__((aligned(32)));
static uint8 gTaskStackLogger[LOGGER_TASKSTACKSIZE]             __attribute__((aligned(32)));

void imuReadTask(void *arg0, void *arg1) {
    DFC_t_IMU_TaskArgs *args = (DFC_t_IMU_TaskArgs *)arg0;
    DFC_t_MPU9250_Data imuRawData;

    while (1) {
        SemaphoreP_pend(args->imu_data_semaph, SemaphoreP_WAIT_FOREVER);

        if (args->imu->update()) {
            imuRawData = args->imu->getCurrentRawData();
            logger_log("IMU: ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f mx=%.2f my=%.2f mz=%.2f T=%.2f\r\n",
                       imuRawData.ax, imuRawData.ay, imuRawData.az,
                       imuRawData.gx, imuRawData.gy, imuRawData.gz,
                       imuRawData.mx, imuRawData.my, imuRawData.mz,
                       imuRawData.temp);
            QueueP_put(args->imu_data_queue, (QueueP_Elem *)&imuRawData);
        } else {
            logger_log("IMU read failed\r\n");
        }
    }
}

void ekfTask(void *arg0, void *arg1) {
    DFC_t_EKF_TaskArgs_t *args = (DFC_t_EKF_TaskArgs_t *)arg0;
    DFC_t_MPU9250_Data *imuRawData;

    while (1) {
        imuRawData = (DFC_t_MPU9250_Data*)QueueP_get(args->imu_data_queue);
        imuRawData->ax = imuRawData->ax;
        vTaskDelay(pdMS_TO_TICKS(50)); // 50 ms

        SemaphoreP_post(args->imu_data_semaph);
    }
}

void loggerTask(void *arg0, void *arg1) {
    char line[128];
    while (1) {
        uint32 n = logger_read(line, sizeof(line));
        if (n > 0) {
            UART_printf("%s", line);
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // 50 ms
    }
}

int main(void) {
    OS_init();
    UART_init();
    UART_stdioInit(0);
    logger_init();

    imu::C_IMU imuObj;
    QueueP_Params queueParams;
    QueueP_Params_init(&queueParams);
    QueueP_Handle imu_data_queue = QueueP_create(&queueParams);
    SemaphoreP_Handle imu_data_semaph = SemaphoreP_create(1, NULL);

    DFC_t_IMU_TaskArgs imuTaskArgs = { &imuObj, imu_data_queue, imu_data_semaph };
    DFC_t_EKF_TaskArgs_t ekfTaskArgs = { imu_data_queue, imu_data_semaph };

    TaskP_Params imuTaskParams, ekfTaskParams, loggerTaskParams;
    TaskP_Params_init(&imuTaskParams);
    imuTaskParams.stack      = gTaskStackIMU;
    imuTaskParams.stacksize  = IMU_TASKSTACKSIZE;
    imuTaskParams.priority   = 3;
    imuTaskParams.arg0       = (void *)&imuTaskArgs;
    imuTaskParams.name       = "IMU_Read";
    TaskP_create(imuReadTask, &imuTaskParams);

    TaskP_Params_init(&ekfTaskParams);
    ekfTaskParams.stack      = gTaskStackEKF;
    ekfTaskParams.stacksize  = EKF_TASKSTACKSIZE;
    ekfTaskParams.priority   = 2;
    ekfTaskParams.arg0       = (void *)&ekfTaskArgs;
    ekfTaskParams.name       = "EKF";
    TaskP_create(ekfTask, &ekfTaskParams);

    TaskP_Params_init(&loggerTaskParams);
    loggerTaskParams.stack      = gTaskStackLogger;
    loggerTaskParams.stacksize  = LOGGER_BUFFER_SIZE;
    loggerTaskParams.priority   = 1;
    loggerTaskParams.arg0       = NULL;
    loggerTaskParams.name       = "Logger";
    TaskP_create(loggerTask, &loggerTaskParams);

    OS_start();

    while (1) { ; }
    return 0;
}
