#include "system.h"
#include "sensor.h"

void vSensorTask(void *pvParameters)
{
    (void)pvParameters;

    SensorData_t data;

    printf("[Sensor] Task started\n");

    for (;;)
    {
        updateHeartbeat(HB_SENSOR);

        data.temperature  = -6300;   /* -63.00 °C */
        data.distance_cm  = 150;     /* 1.5 m */
        data.altitude_cm  = 0;
        data.timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            xSharedSensorData = data;
            xSemaphoreGive(xSensorDataMutex);
        }
        else
        {
            sendStatusReport("Sensor", STATUS_WARNING, 2,
                             "Sensor mutex timeout — shared buffer not updated");
            xEventGroupSetBits(xSystemEventGroup, EVENT_SENSOR_FAULT);
        }

        if (xQueueSend(xSensorQueue, &data, 0) != pdTRUE)
            printf("[Sensor] Warning: sensor queue full\n");

        vTaskDelay(pdMS_TO_TICKS(PERIOD_SENSOR_MS));
    }
}
