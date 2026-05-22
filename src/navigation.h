#ifndef NAVIGATION_H
#define NAVIGATION_H

/* Navigation task — priority 2.
   Consumes SensorData_t from xSensorQueue, records waypoints,
   and signals EVENT_OBSTACLE_DETECTED when distance is critical. */
void vNavigationTask(void *pvParameters);

#endif /* NAVIGATION_H */
