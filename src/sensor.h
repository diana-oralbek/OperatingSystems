#ifndef SENSOR_H
#define SENSOR_H

/* Sensor task — priority 3.
   Reads temperature, LIDAR distance, and altitude; writes to
   xSharedSensorData and pushes a copy to xSensorQueue. */
void vSensorTask(void *pvParameters);

#endif /* SENSOR_H */
