#include "system.h"
#include "power.h"

/* TODO: include your ADC / power-monitor driver headers here, e.g.:
   #include "adc_driver.h"
   #include "ina226_power_monitor.h"
*/

void vPowerTask(void *pvParameters)
{
    (void)pvParameters;

    uint32_t estimatedPower = 0;

    printf("[Power] Task started\n");

    for (;;)
    {
        updateHeartbeat(HB_POWER);

        /* TODO: read actual power consumption from ADC/power-monitor IC
           estimatedPower = adc_read_power_units();
        */
        estimatedPower = 70; /* placeholder */

        printf("[Power] Current usage: %lu / %d units\n",
               (unsigned long)estimatedPower, POWER_BUDGET_MAX);

        if (estimatedPower > POWER_BUDGET_MAX)
        {
            xEventGroupSetBits(xSystemEventGroup, EVENT_LOW_POWER);

            /* Suspend lowest-priority non-critical task first */
            if (xNavigationHandle != NULL)
            {
                vTaskSuspend(xNavigationHandle);
                sendStatusReport("Power", STATUS_WARNING, 3,
                                 "Navigation suspended — budget exceeded");
            }

            /* TODO: add further shedding tiers (Sensor, Comms) */
        }
        else
        {
            xEventGroupClearBits(xSystemEventGroup, EVENT_LOW_POWER);

            /* Resume tasks that were suspended for power saving */
            if (xNavigationHandle != NULL &&
                eTaskGetState(xNavigationHandle) == eSuspended)
            {
                vTaskResume(xNavigationHandle);
                printf("[Power] Navigation resumed\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(PERIOD_POWER_MS));
    }
}
