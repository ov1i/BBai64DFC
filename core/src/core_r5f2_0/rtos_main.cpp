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
#include <ti/board/board.h>
#include <ti/drv/i2c/i2c.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/csl/arch/r5/csl_arm_r5_mpu.h>
// #include "rpmsg_lite.h"

}
// #include "comms/logger.h"
#include "imu/mpu9250.h"
#include "dfc_types.h"


static uint8 gTaskStackIMU[IMU_TASKSTACKSIZE]                   __attribute__((aligned(32)));
// static uint8 gTaskStackEKF[EKF_TASKSTACKSIZE]                   __attribute__((aligned(32)));
// static uint8 gTaskStackLogger[LOGGER_TASKSTACKSIZE]             __attribute__((aligned(32)));
static uint8 gTaskStackInit[INIT_TASKSTACKSIZE]                 __attribute__((aligned(32)));

void initTask(void *arg0, void *arg1) {
    imu::C_IMU* pImuOBJ = (imu::C_IMU*)arg0;
    SemaphoreP_Handle* pInitSempahHandler = (SemaphoreP_Handle*)arg1;
    Board_STATUS boardStatus = BOARD_SOK;
    uint32 muxData;;

    UART_init();
    UART_stdioInit(0);
    
    /// AA3 - pin
    muxData = 0x64002;
    boardStatus = Board_pinmuxSetReg(BOARD_SOC_DOMAIN_MAIN, static_cast<uint32>(PADCONFIG_OFFSET_REG116), muxData);
    if(boardStatus == BOARD_SOK) {
        UART_printf("AA3 pin succesfully set in i2c mode!\r\n");
    } else {
        UART_printf("AA3 pin i2c mode switch failed..\r\n");
    }

    boardStatus = Board_pinmuxGetReg(BOARD_SOC_DOMAIN_MAIN, static_cast<uint32>(PADCONFIG_OFFSET_REG116), &muxData);
    if(boardStatus == BOARD_SOK) {
        UART_printf("AA3 pin fetched and checked succefully (settings: %X)\r\n", muxData);
    } else {
        UART_printf("AA3 pin fetched and checks failed..\r\n");
    }

    /// Y2 - pin
    muxData = 0x64002;
    boardStatus = Board_pinmuxSetReg(BOARD_SOC_DOMAIN_MAIN, static_cast<uint32>(PADCONFIG_OFFSET_REG121), muxData);
    if(boardStatus == BOARD_SOK) {
        UART_printf("Y2 pin succesfully set in i2c mode!\r\n");
    } else {
        UART_printf("Y2 pin i2c mode switch failed..\r\n");
    }

    boardStatus = Board_pinmuxGetReg(BOARD_SOC_DOMAIN_MAIN, static_cast<uint32>(PADCONFIG_OFFSET_REG121), &muxData);
    if(boardStatus == BOARD_SOK) {
        UART_printf("Y2 pin fetched and checked succefully (settings: %X)\r\n", muxData);
    } else {
        UART_printf("Y2 pin fetched and checks failed..\r\n");
    }

    if(boardStatus != BOARD_SOK) {
        UART_printf("Board init failed! Please restart the board...\r\n");
    } else {
        UART_printf("Board initialization was succesful!\r\n");
    }

    // logger_init();
    while(pImuOBJ == NULL) {
        UART_printf("Imu object corrupted...checkup required!\r\n");
    }
    while(!pImuOBJ->init()) {
        UART_printf("MPU9250 init failed! Retrying in a bit...\r\n");
        TaskP_sleep(10000);
    }
    UART_printf("All init OK, releasing app tasks!\r\n");
    SemaphoreP_post(*pInitSempahHandler);
    vTaskDelete(NULL);
}

void imuReadTask(void *arg0, void *arg1) {
    DFC_t_IMU_TaskArgs *imu_task_args = (DFC_t_IMU_TaskArgs *)arg0;
    SemaphoreP_Handle* pInitSempahHandler = (SemaphoreP_Handle*)arg1;
    DFC_t_MPU9250_Data imuRawData;
    
    SemaphoreP_pend(*pInitSempahHandler, SemaphoreP_WAIT_FOREVER);

    while (1) {
        // SemaphoreP_pend(imu_task_args->imu_data_semaph, SemaphoreP_WAIT_FOREVER);

        if (imu_task_args->imu->update()) {
            imu_task_args->imu->getCurrentRawData(&imuRawData);
            UART_printf("IMU: ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f mx=%.2f my=%.2f mz=%.2f T=%.2f\r\n",
                       imuRawData.ax, imuRawData.ay, imuRawData.az,
                       imuRawData.gx, imuRawData.gy, imuRawData.gz,
                       imuRawData.mx, imuRawData.my, imuRawData.mz,
                       imuRawData.temp);
            // QueueP_put(imu_task_args->imu_data_queue, (QueueP_Elem *)&imuRawData);
        } else {
            UART_printf("IMU read failed\r\n");
        }
        TaskP_sleepInMsecs(5);
    }
}

// void ekfTask(void *arg0, void *arg1) {
//     DFC_t_EKF_TaskArgs_t *args = (DFC_t_EKF_TaskArgs_t *)arg0;
//     SemaphoreP_Handle* pInitSempahHandler = (SemaphoreP_Handle*)arg1;
//     DFC_t_MPU9250_Data *imuRawData;

//     SemaphoreP_pend(*pInitSempahHandler, SemaphoreP_WAIT_FOREVER);

//     while (1) {
//         imuRawData = (DFC_t_MPU9250_Data*)QueueP_get(args->imu_data_queue);
//         imuRawData->ax = imuRawData->ax;
//         vTaskDelay(pdMS_TO_TICKS(50)); // 50 ms

//         SemaphoreP_post(args->imu_data_semaph);
//     }
// }

// void logger_task(void* params) {
//     char msg[LOGGER_MSG_MAXLEN];
//     comms::Logger& logger = comms::Logger::instance();
//     for (;;) {
//         if (logger.read(msg, sizeof(msg)) > 0) {
//             logger.send_log(msg);
//         }
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

int main(void) {
    OS_init();

    TaskP_Params imuTaskParams, initTaskParams; //, ekfTaskParams, loggerTaskParams;
    QueueP_Params queueParams;
    QueueP_Params_init(&queueParams);
    QueueP_Handle imu_data_queue = QueueP_create(&queueParams);
    SemaphoreP_Handle imu_data_semaph = SemaphoreP_create(1, NULL);
    SemaphoreP_Handle init_semaph = SemaphoreP_create(0, NULL);

    imu::C_IMU imuObj;
    DFC_t_IMU_TaskArgs imuTaskArgs = { &imuObj, imu_data_queue, imu_data_semaph };
    // DFC_t_EKF_TaskArgs_t ekfTaskArgs = { imu_data_queue, imu_data_semaph };

    
    TaskP_Params_init(&imuTaskParams);
    imuTaskParams.stack      = gTaskStackIMU;
    imuTaskParams.stacksize  = IMU_TASKSTACKSIZE;
    imuTaskParams.priority   = 3;
    imuTaskParams.arg0       = (void *)&imuTaskArgs;
    imuTaskParams.arg1       = (void *)&init_semaph;
    imuTaskParams.name       = "IMU_Read";
    TaskP_create(imuReadTask, &imuTaskParams);

    // TaskP_Params_init(&ekfTaskParams);
    // ekfTaskParams.stack      = gTaskStackEKF;
    // ekfTaskParams.stacksize  = EKF_TASKSTACKSIZE;
    // ekfTaskParams.priority   = 2;
    // ekfTaskParams.arg0       = (void *)&ekfTaskArgs;
    // ekfTaskParams.arg1       = (void *)&init_semaph;
    // ekfTaskParams.name       = "EKF";
    // TaskP_create(ekfTask, &ekfTaskParams);

    TaskP_Params_init(&initTaskParams);
    initTaskParams.stack = gTaskStackInit;
    initTaskParams.stacksize = INIT_TASKSTACKSIZE;
    initTaskParams.priority = 4;
    initTaskParams.arg0 = (void *)&imuObj;
    initTaskParams.arg1 = (void *)&init_semaph;
    initTaskParams.name = "Init";
    TaskP_create(initTask, &initTaskParams);

    OS_start();

    while (1) { ; }

    return 0;
}
