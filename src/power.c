#include "system.h"
#include "power.h"

void vPowerTask(void *pvParameters)
{
    (void)pvParameters;

    uint32_t estimatedPower = 0;

    printf("[Power] Task started\n");

    for (;;)
    {
        updateHeartbeat(HB_POWER);

        estimatedPower = 70;

        printf("[Power] Current usage: %lu / %d units\n",
               (unsigned long)estimatedPower, POWER_BUDGET_MAX);

        if (estimatedPower > POWER_BUDGET_MAX)
        {
            xEventGroupSetBits(xSystemEventGroup, EVENT_LOW_POWER);

            if (xNavigationHandle != NULL)
            {
                vTaskSuspend(xNavigationHandle);
                sendStatusReport("Power", STATUS_WARNING, 3,
                                 "Navigation suspended — budget exceeded");
            }
        }
        else
        {
            xEventGroupClearBits(xSystemEventGroup, EVENT_LOW_POWER);

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
