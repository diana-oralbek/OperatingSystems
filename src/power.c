#include "system.h"
#include "power.h"
#include <stdlib.h>

/* Per-subsystem power costs (units) */
#define POWER_BASE_LOAD          15   /* CPU, memory, standby */
#define POWER_MOTOR_COST         40   /* drivetrain when active */
#define POWER_SENSOR_COST        10
#define POWER_NAVIGATION_COST    15
#define POWER_COMMS_COST          8
/* Cold batteries are less efficient; penalty applied below -70 °C */
#define COLD_THRESHOLD_FIXEDPT  -7000
#define COLD_PENALTY              10
/* ±3 units of random measurement noise */
#define POWER_NOISE_RANGE        20

static uint32_t estimatePowerUsage(void)
{
    uint32_t power = POWER_BASE_LOAD;

    if (xMotorTaskHandle != NULL &&
        eTaskGetState(xMotorTaskHandle) != eSuspended)
        power += POWER_MOTOR_COST;

    if (xSensorTaskHandle != NULL &&
        eTaskGetState(xSensorTaskHandle) != eSuspended)
        power += POWER_SENSOR_COST;

    if (xNavigationHandle != NULL &&
        eTaskGetState(xNavigationHandle) != eSuspended)
        power += POWER_NAVIGATION_COST;

    if (xCommsTaskHandle != NULL &&
        eTaskGetState(xCommsTaskHandle) != eSuspended)
        power += POWER_COMMS_COST;

    /* Cold temperature degrades battery efficiency */
    if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        if (xSharedSensorData.temperature < COLD_THRESHOLD_FIXEDPT)
            power += COLD_PENALTY;
        xSemaphoreGive(xSensorDataMutex);
    }

    power += (uint32_t)(rand() % POWER_NOISE_RANGE);
    return power;
}

void vPowerTask(void *pvParameters)
{
    (void)pvParameters;

    uint32_t estimatedPower;

    printf("[Power] Task started\n");

    for (;;)
    {
        updateHeartbeat(HB_POWER);

        estimatedPower = estimatePowerUsage();

        printf("[Power] Estimated usage: %lu / %d units\n",
               (unsigned long)estimatedPower, POWER_BUDGET_MAX);

        if (estimatedPower > POWER_BUDGET_MAX)
        {
            xEventGroupSetBits(xSystemEventGroup, EVENT_LOW_POWER);

            if (xNavigationHandle != NULL &&
                eTaskGetState(xNavigationHandle) != eSuspended)
            {
                vTaskSuspend(xNavigationHandle);
                sendStatusReport("Power", STATUS_WARNING, 3,
                                 "Navigation suspended — budget exceeded");
                printf("[Power] Navigation suspended to save power\n");
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
