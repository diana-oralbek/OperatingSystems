#include "system.h"
#include "sensor.h"

/* TODO: include your sensor HAL headers here, e.g.:
   #include "temperature_sensor.h"
   #include "lidar.h"
   #include "altimeter.h"
*/

void vSensorTask(void *pvParameters)
{
    (void)pvParameters;

    SensorData_t data;

    printf("[Sensor] Task started\n");

    for (;;)
    {
        updateHeartbeat(HB_SENSOR);

        /* TODO: replace with real HAL reads
           data.temperature  = temperature_read_celsius_x100();
           data.distance_cm  = lidar_read_cm();
           data.altitude_cm  = altimeter_read_cm();
        */
        data.temperature  = -6300;          /* placeholder: -63.00 °C */
        data.distance_cm  = 150;            /* placeholder: 1.5 m */
        data.altitude_cm  = 0;
        data.timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        /* Write to shared buffer (protected) */
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

        /* Push to Navigation queue (non-blocking) */
        if (xQueueSend(xSensorQueue, &data, 0) != pdTRUE)
        {
            /* TODO: decide whether to drop or block here */
            printf("[Sensor] Warning: sensor queue full\n");
        }

        vTaskDelay(pdMS_TO_TICKS(PERIOD_SENSOR_MS));
    }
}
