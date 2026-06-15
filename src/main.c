#include "system.h"
#include <stdlib.h>
#include <time.h>
#include "main_computer.h"
#include "motor.h"
#include "navigation.h"
#include "sensor.h"
#include "comms.h"
#include "power.h"
#include "self_monitor.h"
#include "mct.h"
#include "watchdog.h"

static void runSelfTests(void)
{
    printf("\n=== Self Tests ===\n");

    /* Test 1: command queue */
    MotorCommand_t cmd      = { CMD_STOP, 0, 0 };
    MotorCommand_t received = { 0, 0, 0 };
    xQueueSend(xCommandQueue, &cmd, 0);
    if (xQueueReceive(xCommandQueue, &received, 0) == pdTRUE &&
        received.type == CMD_STOP)
        printf("[TEST] Command queue:  PASS\n");
    else
        printf("[TEST] Command queue:  FAIL\n");

    /* Test 2: sensor mutex */
    if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        xSemaphoreGive(xSensorDataMutex);
        printf("[TEST] Sensor mutex:   PASS\n");
    } else {
        printf("[TEST] Sensor mutex:   FAIL\n");
    }

    /* Test 3: event group set/clear */
    xEventGroupSetBits(xSystemEventGroup, EVENT_OBSTACLE_DETECTED);
    EventBits_t bits = xEventGroupGetBits(xSystemEventGroup);
    xEventGroupClearBits(xSystemEventGroup, EVENT_OBSTACLE_DETECTED);
    if (bits & EVENT_OBSTACLE_DETECTED)
        printf("[TEST] Event group:    PASS\n");
    else
        printf("[TEST] Event group:    FAIL\n");

    /* Test 4: odometry initialised to origin */
    if (xRoverOdometry.x_cm == 0 && xRoverOdometry.y_cm == 0)
        printf("[TEST] Odometry init:  PASS\n");
    else
        printf("[TEST] Odometry init:  FAIL\n");

    printf("=== Tests Done ===\n\n");
}

int main(void)
{
    setvbuf(stdout, NULL, _IOLBF, 0);   /* line-buffered even when piped */

    printf("==============================================\n");
    printf("  Mars Rover Exploration System — Booting   \n");
    printf("==============================================\n");

    /* ── Step 1: Mutexes ────────────────────────────────────── */
    xSensorDataMutex = xSemaphoreCreateMutex();
    xMotorMutex      = xSemaphoreCreateMutex();
    xNavMapMutex     = xSemaphoreCreateMutex();
    xOdometryMutex   = xSemaphoreCreateMutex();
    configASSERT(xSensorDataMutex != NULL);
    configASSERT(xMotorMutex      != NULL);
    configASSERT(xNavMapMutex     != NULL);
    configASSERT(xOdometryMutex   != NULL);

    /* ── Step 2: Queues ─────────────────────────────────────── */
    xCommandQueue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(MotorCommand_t));
    xSensorQueue  = xQueueCreate(SENSOR_QUEUE_LENGTH,  sizeof(SensorData_t));
    xStatusQueue  = xQueueCreate(STATUS_QUEUE_LENGTH,  sizeof(StatusReport_t));
    configASSERT(xCommandQueue != NULL);
    configASSERT(xSensorQueue  != NULL);
    configASSERT(xStatusQueue  != NULL);

    /* ── Step 3: Event group ────────────────────────────────── */
    xSystemEventGroup = xEventGroupCreate();
    configASSERT(xSystemEventGroup != NULL);

    /* ── Step 4: Heartbeat table ────────────────────────────── */
    memset(xHeartbeats, 0, sizeof(xHeartbeats));

    /* ── Step 5: Tasks ──────────────────────────────────────── */
    xTaskCreate(vMainComputerTask, "MainComp", STACK_MAIN_COMPUTER, NULL,
                PRIORITY_MAIN_COMPUTER,  &xMainComputerTask);

    xTaskCreate(vMotorTask,        "Motor",    STACK_MOTOR,         NULL,
                PRIORITY_MOTOR,          &xMotorTaskHandle);

    xTaskCreate(vCommsTask,        "Comms",    STACK_COMMS,         NULL,
                PRIORITY_COMMS,          &xCommsTaskHandle);

    xTaskCreate(vSelfMonitorTask,  "SelfMon",  STACK_SELF_MONITOR,  NULL,
                PRIORITY_SELF_MONITOR,   &xSelfMonitorHandle);

    xTaskCreate(vSensorTask,       "Sensor",   STACK_SENSOR,        NULL,
                PRIORITY_SENSOR,         &xSensorTaskHandle);

    xTaskCreate(vNavigationTask,   "Nav",      STACK_NAVIGATION,    NULL,
                PRIORITY_NAVIGATION,     &xNavigationHandle);

    xTaskCreate(vPowerTask,        "Power",    STACK_POWER,         NULL,
                PRIORITY_POWER,          &xPowerTaskHandle);

    xTaskCreate(vMCTTask,          "MCT",      STACK_MCT,           NULL,
                PRIORITY_MCT,            &xMCTHandle);

    /* ── Step 6: Seed RNG ───────────────────────────────────── */
    srand((unsigned int)time(NULL));

    /* ── Step 7: Self-tests (IPC smoke check before scheduler) ─ */
    runSelfTests();

    /* ── Step 8: Watchdog timer ────────────────────────────── */
    initWatchdog();

    /* ── Step 9: Start scheduler ────────────────────────────── */
    printf("[Main] All tasks created. Starting scheduler...\n");
    vTaskStartScheduler();

    printf("[Main] ERROR: Scheduler returned unexpectedly!\n");
    for (;;);
    return 0;
}

void vApplicationMallocFailedHook(void)
{
    printf("[FreeRTOS] FATAL: Heap allocation failed!\n");
    taskDISABLE_INTERRUPTS();
    for (;;);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    printf("[FreeRTOS] FATAL: Stack overflow in task '%s'!\n", pcTaskName);
    taskDISABLE_INTERRUPTS();
    for (;;);
}

void vApplicationTickHook(void) {}
void vApplicationIdleHook(void) {}
