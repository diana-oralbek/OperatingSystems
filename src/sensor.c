#include "system.h"
#include "sensor.h"
#include <stdlib.h>

void vSensorTask(void *pvParameters)
{
    (void)pvParameters;

    SensorData_t data;

    static int32_t  sim_temp = -6300;   /* -63.00 °C in fixed point */
    static int32_t  sim_dist = 150;
    static int32_t  sim_alt  = 0;

    printf("[Sensor] Task started\n");

    for (;;)
    {
        updateHeartbeat(HB_SENSOR);

        /* Drift readings slightly each cycle to simulate a real sensor */
        sim_temp += (rand() % 11) - 5;
        if (sim_temp < -8000) sim_temp = -8000;
        if (sim_temp > -5000) sim_temp = -5000;

        sim_dist += (rand() % 21) - 10;
        if (sim_dist < 20)  sim_dist = 20;
        if (sim_dist > 500) sim_dist = 500;

        sim_alt += (rand() % 7) - 3;
        if (sim_alt < -200) sim_alt = -200;
        if (sim_alt >  200) sim_alt =  200;

        data.temperature  = sim_temp;
        data.distance_cm  = (uint32_t)sim_dist;
        data.altitude_cm  = sim_alt;
        data.timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        printf("[Sensor] Temp:%.2f C  Dist:%ucm  Alt:%ldcm\n",
               (float)data.temperature / 100.0f,
               (unsigned)data.distance_cm,
               (long)data.altitude_cm);

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
