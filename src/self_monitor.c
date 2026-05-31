#include "system.h"
#include "self_monitor.h"
#include "sensor.h"
#include "motor.h"
#include "main_computer.h"
#include "comms.h"
#include "power.h"

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
            vTaskDelete(xMainComputerTask);
            xMainComputerTask = NULL;
            xTaskCreate(vMainComputerTask, "MainComputer", STACK_MAIN_COMPUTER,
                        NULL, PRIORITY_MAIN_COMPUTER, &xMainComputerTask);
            printf("[SelfMonitor] MainComputer task restarted\n");
        }

        /* Motor */
        if (!isTaskAlive(HB_MOTOR))
        {
            printf("[SelfMonitor] WARNING: Motor task appears frozen!\n");
            xEventGroupSetBits(xSystemEventGroup, EVENT_MOTOR_FAULT);
            sendStatusReport("SelfMonitor", STATUS_ERROR, 11,
                             "Motor heartbeat lost");
            vTaskDelete(xMotorTaskHandle);
            xMotorTaskHandle = NULL;
            xTaskCreate(vMotorTask, "Motor", STACK_MOTOR, NULL,
                        PRIORITY_MOTOR, &xMotorTaskHandle);
            printf("[SelfMonitor] Motor task restarted\n");
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
            vTaskDelete(xCommsTaskHandle);
            xCommsTaskHandle = NULL;
            xTaskCreate(vCommsTask, "Comms", STACK_COMMS, NULL,
                        PRIORITY_COMMS, &xCommsTaskHandle);
            printf("[SelfMonitor] Comms task restarted\n");
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

        /* Power */
        if (!isTaskAlive(HB_POWER))
        {
            printf("[SelfMonitor] WARNING: Power task appears frozen!\n");
            sendStatusReport("SelfMonitor", STATUS_ERROR, 15,
                             "Power heartbeat lost");
            vTaskDelete(xPowerTaskHandle);
            xPowerTaskHandle = NULL;
            xTaskCreate(vPowerTask, "Power", STACK_POWER, NULL,
                        PRIORITY_POWER, &xPowerTaskHandle);
            printf("[SelfMonitor] Power task restarted\n");
        }

        /* Stack high-water mark checks — warn when any task is nearly out of stack */
        {
            UBaseType_t hwm;

            if (xMainComputerTask != NULL) {
                hwm = uxTaskGetStackHighWaterMark(xMainComputerTask);
                if (hwm < 20)
                    sendStatusReport("SelfMonitor", STATUS_WARNING, 20,
                                     "MainComputer stack nearly full");
            }
            if (xMotorTaskHandle != NULL) {
                hwm = uxTaskGetStackHighWaterMark(xMotorTaskHandle);
                if (hwm < 20)
                    sendStatusReport("SelfMonitor", STATUS_WARNING, 21,
                                     "Motor stack nearly full");
            }
            if (xSensorTaskHandle != NULL) {
                hwm = uxTaskGetStackHighWaterMark(xSensorTaskHandle);
                if (hwm < 20)
                    sendStatusReport("SelfMonitor", STATUS_WARNING, 22,
                                     "Sensor stack nearly full");
            }
            if (xCommsTaskHandle != NULL) {
                hwm = uxTaskGetStackHighWaterMark(xCommsTaskHandle);
                if (hwm < 20)
                    sendStatusReport("SelfMonitor", STATUS_WARNING, 23,
                                     "Comms stack nearly full");
            }
            if (xNavigationHandle != NULL) {
                hwm = uxTaskGetStackHighWaterMark(xNavigationHandle);
                if (hwm < 20)
                    sendStatusReport("SelfMonitor", STATUS_WARNING, 24,
                                     "Navigation stack nearly full");
            }
            if (xPowerTaskHandle != NULL) {
                hwm = uxTaskGetStackHighWaterMark(xPowerTaskHandle);
                if (hwm < 20)
                    sendStatusReport("SelfMonitor", STATUS_WARNING, 25,
                                     "Power stack nearly full");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(PERIOD_SELF_MONITOR_MS));
    }
}