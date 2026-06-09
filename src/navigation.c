#include "system.h"
#include "navigation.h"
#include "watchdog.h"

/* Maximum waypoints held in the ring buffer before oldest is overwritten */
#define MAX_WAYPOINTS   50

/* Each waypoint stores sensor readings plus the rover's position
   at the moment it was recorded. */
typedef struct {
    int32_t     x_cm;
    int32_t     y_cm;
    int32_t     altitude_cm;
    uint32_t    distance_cm;
    int32_t     temperature;    /* Celsius * 100 fixed-point */
    uint32_t    timestamp_ms;
} Waypoint_t;

/* Ring-buffer route map — protected by xNavMapMutex */
static Waypoint_t   xRouteMap[MAX_WAYPOINTS];
static uint32_t     xWaypointHead  = 0;
static uint32_t     xWaypointCount = 0;

/* ── Snapshot current position ──────────────────────────────── */
static void snapshotPosition(int32_t *out_x, int32_t *out_y)
{
    *out_x = 0;
    *out_y = 0;
    if (xSemaphoreTake(xOdometryMutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        *out_x = xRoverOdometry.x_cm;
        *out_y = xRoverOdometry.y_cm;
        xSemaphoreGive(xOdometryMutex);
    }
}

/* ── Record waypoint ────────────────────────────────────────── */
static void recordWaypoint(const SensorData_t *data)
{
    int32_t x, y;
    snapshotPosition(&x, &y);

    if (xSemaphoreTake(xNavMapMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        xRouteMap[xWaypointHead].x_cm         = x;
        xRouteMap[xWaypointHead].y_cm         = y;
        xRouteMap[xWaypointHead].altitude_cm  = data->altitude_cm;
        xRouteMap[xWaypointHead].distance_cm  = data->distance_cm;
        xRouteMap[xWaypointHead].temperature  = data->temperature;
        xRouteMap[xWaypointHead].timestamp_ms = data->timestamp_ms;

        xWaypointHead = (xWaypointHead + 1) % MAX_WAYPOINTS;
        xWaypointCount++;

        xSemaphoreGive(xNavMapMutex);
    }
    else
    {
        sendStatusReport("Navigation", STATUS_WARNING, 4,
                         "Nav map mutex timeout — waypoint skipped");
    }
}

/* ── Log waypoint to console ────────────────────────────────── */
static void logWaypoint(const SensorData_t *data, int32_t x, int32_t y)
{
    float originDist = roverOriginDistance();

    /* Fixed-point temperature: e.g. -6300 → "-63.00 C" */
    int32_t  tempInt = data->temperature / 100;
    int32_t  tempFrac = data->temperature < 0
                        ? -(data->temperature % 100)
                        :  (data->temperature % 100);

    printf("[Navigation] WP#%lu  Pos:(%ld,%ld)cm  "
           "Origin:%.1fcm  Sensor dist:%lucm  Temp:%ld.%02ldC\n",
           (unsigned long)xWaypointCount,
           (long)x, (long)y,
           originDist,
           (unsigned long)data->distance_cm,
           (long)tempInt, (long)tempFrac);
}

/* ── Task entry point ───────────────────────────────────────── */
void vNavigationTask(void *pvParameters)
{
    (void)pvParameters;

    SensorData_t data;

    printf("[Navigation] Task started\n");

    for (;;)
    {
        updateHeartbeat(HB_NAV);
        feedWatchdog();

        /* Wait for a fresh sensor reading from xSensorQueue */
        if (xQueueReceive(xSensorQueue, &data, pdMS_TO_TICKS(300)) == pdTRUE)
        {
            int32_t x, y;
            snapshotPosition(&x, &y);

            recordWaypoint(&data);
            logWaypoint(&data, x, y);

            /* Proximity alert — also checked by Motor, but Navigation
               has the full route context and may detect patterns first */
            if (data.distance_cm < 50)
            {
                xEventGroupSetBits(xSystemEventGroup, EVENT_OBSTACLE_DETECTED);
                sendStatusReport("Navigation", STATUS_WARNING, 5,
                                 "Critical proximity — obstacle event raised");
            }

            /* Warn when ring buffer wraps to avoid silent data loss */
            if (xWaypointCount > 0 && (xWaypointCount % MAX_WAYPOINTS) == 0)
            {
                sendStatusReport("Navigation", STATUS_WARNING, 6,
                                 "Route map full — oldest waypoints overwritten");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(PERIOD_NAVIGATION_MS));
    }
}
