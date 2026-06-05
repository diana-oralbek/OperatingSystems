#include "system.h"
#include <math.h>

/* ── Shared resource definitions ────────────────────────────── */
SemaphoreHandle_t   xSensorDataMutex    = NULL;
SemaphoreHandle_t   xMotorMutex         = NULL;
SemaphoreHandle_t   xNavMapMutex        = NULL;

QueueHandle_t       xCommandQueue       = NULL;
QueueHandle_t       xSensorQueue        = NULL;
QueueHandle_t       xStatusQueue        = NULL;

EventGroupHandle_t  xSystemEventGroup   = NULL;

TaskHandle_t        xMainComputerTask   = NULL;
TaskHandle_t        xMotorTaskHandle    = NULL;
TaskHandle_t        xCommsTaskHandle    = NULL;
TaskHandle_t        xSelfMonitorHandle  = NULL;
TaskHandle_t        xSensorTaskHandle   = NULL;
TaskHandle_t        xNavigationHandle   = NULL;
TaskHandle_t        xPowerTaskHandle    = NULL;
TaskHandle_t        xMCTHandle          = NULL;

SensorData_t        xSharedSensorData;
TaskHeartbeat_t     xHeartbeats[8];

RoverOdometry_t     xRoverOdometry  = { 0, 0, 0, 0 };
SemaphoreHandle_t   xOdometryMutex  = NULL;

/* ── Helper implementations ─────────────────────────────────── */

/* Non-blocking send to Comms queue; dropped if queue is full */
void sendStatusReport(const char *taskName, StatusLevel_t level,
                      uint32_t errorCode, const char *message)
{
    StatusReport_t report;
    strncpy(report.task_name, taskName, sizeof(report.task_name) - 1);
    report.task_name[sizeof(report.task_name) - 1] = '\0';
    report.level        = level;
    report.error_code   = errorCode;
    report.timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    strncpy(report.message, message, sizeof(report.message) - 1);
    report.message[sizeof(report.message) - 1] = '\0';
    xQueueSend(xStatusQueue, &report, 0);
}

/* Each task calls this at the top of its loop so SelfMonitor
   knows it is still running */
void updateHeartbeat(HeartbeatIndex_t index)
{
    xHeartbeats[index].last_ping_ms =
        (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    xHeartbeats[index].is_alive = true;
}

/* Returns false when a task has not pinged within HEARTBEAT_TIMEOUT_MS */
bool isTaskAlive(HeartbeatIndex_t index)
{
    uint32_t now     = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    uint32_t elapsed = now - xHeartbeats[index].last_ping_ms;
    return (elapsed < HEARTBEAT_TIMEOUT_MS);
}

/* Euclidean distance from origin — mutex-safe, no math.h needed in callers */
float roverOriginDistance(void)
{
    float result = 0.0f;
    if (xSemaphoreTake(xOdometryMutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        float x = (float)xRoverOdometry.x_cm;
        float y = (float)xRoverOdometry.y_cm;
        result  = sqrtf(x * x + y * y);
        xSemaphoreGive(xOdometryMutex);
    }
    return result;
}
