#include "system.h"
#include "comms.h"

/* TODO: include your RF / UART driver headers here, e.g.:
   #include "uart_driver.h"
   #include "rf_transceiver.h"
*/

void vCommsTask(void *pvParameters)
{
    (void)pvParameters;

    StatusReport_t report;

    printf("[Comms] Task started\n");

    for (;;)
    {
        updateHeartbeat(HB_COMMS);

        /* Drain all pending status reports and transmit each one */
        while (xQueueReceive(xStatusQueue, &report, 0) == pdTRUE)
        {
            /* TODO: replace with real RF/UART transmission
               uart_send(&report, sizeof(report));
            */
            printf("[Comms] >> [%s] Level:%d Code:%lu — %s\n",
                   report.task_name,
                   (int)report.level,
                   (unsigned long)report.error_code,
                   report.message);
        }

        /* TODO: poll for incoming Earth commands and forward to xCommandQueue
           MotorCommand_t earthCmd;
           if (uart_receive(&earthCmd, sizeof(earthCmd)) == OK) {
               xQueueSend(xCommandQueue, &earthCmd, 0);
           }
        */

        /* TODO: detect loss of Earth signal and set EVENT_COMMS_LOST */

        vTaskDelay(pdMS_TO_TICKS(PERIOD_COMMS_MS));
    }
}
