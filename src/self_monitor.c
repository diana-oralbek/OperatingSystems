#include "system.h"
#include "self_monitor.h"
#include "sensor.h"

void vSelfMonitorTask(void *pvParameters)
{
    (void)pvParameters;

    printf("[SelfMonitor] Task started\n");

    for (;;)
    {
        /* SelfMonitor intentionally does NOT call updateHeartbeat()
           for itself — it is the one checking all others */

        /* MainComputer */
        if (!isTaskAlive(HB_MAIN))
        {
            printf("[SelfMonitor] WARNING: MainComputer appears frozen!\n");
            sendStatusReport("SelfMonitor", STATUS_ERROR, 10,
                             "MainComputer heartbeat lost");
            /* TODO: attempt recovery — delete and recreate the task */
        }

        /* Motor */
        if (!isTaskAlive(HB_MOTOR))
        {
            printf("[SelfMonitor] WARNING: Motor task appears frozen!\n");
            xEventGroupSetBits(xSystemEventGroup, EVENT_MOTOR_FAULT);
            sendStatusReport("SelfMonitor", STATUS_ERROR, 11,
                             "Motor heartbeat lost");
            /* TODO: attempt recovery */
        }

        /* Sensor */
        if (!isTaskAlive(HB_SENSOR))
        {
            printf("[SelfMonitor] WARNING: Sensor task appears frozen!\n");
            xEventGroupSetBits(xSystemEventGroup, EVENT_SENSOR_FAULT);
            sendStatusReport("SelfMonitor", STATUS_ERROR, 12,
                             "Sensor heartbeat lost");
            vTaskDelete(xSensorTaskHandle);
            xSensorTaskHandle = NULL;
            xTaskCreate(vSensorTask, "Sensor", STACK_SENSOR, NULL,
                        PRIORITY_SENSOR, &xSensorTaskHandle);
            printf("[SelfMonitor] Sensor task restarted\n");
        }

        /* Comms */
        if (!isTaskAlive(HB_COMMS))
        {
            printf("[SelfMonitor] WARNING: Comms task appears frozen!\n");
            xEventGroupSetBits(xSystemEventGroup, EVENT_COMMS_LOST);
            sendStatusReport("SelfMonitor", STATUS_ERROR, 13,
                             "Comms heartbeat lost");
        }

        /* Navigation — skip check when intentionally suspended by Power task */
        if (xNavigationHandle != NULL &&
            eTaskGetState(xNavigationHandle) != eSuspended &&
            !isTaskAlive(HB_NAV))
        {
            printf("[SelfMonitor] WARNING: Navigation task appears frozen!\n");
            sendStatusReport("SelfMonitor", STATUS_WARNING, 14,
                             "Navigation heartbeat lost");
        }

        /* TODO: add stack high-water mark checks, e.g.:
           UBaseType_t hwm = uxTaskGetStackHighWaterMark(xMotorTaskHandle);
           if (hwm < 20) {
               sendStatusReport("SelfMonitor", STATUS_WARNING, 20,
                                "Motor stack nearly full");
           }
        */

        vTaskDelay(pdMS_TO_TICKS(PERIOD_SELF_MONITOR_MS));
    }
}
