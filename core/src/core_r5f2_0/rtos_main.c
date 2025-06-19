#include "FreeRTOS.h"
#include "task.h"
#include "ti/drv/uart/UART.h"
#include "ti/drv/uart/UART_stdio.h"

void uartPrintTask(void *arg)
{
    (void)arg;
    while (1)
    {
        UART_printf("Hello World from R5F with FreeRTOS!\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void)
{
    UART_init();
    UART_stdioInit(0);
    xTaskCreate(uartPrintTask, "UART_Print", 512, NULL, 1, NULL);
    vTaskStartScheduler();
    while (1);
}
