#ifndef SYSTEM_H
#define SYSTEM_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ── Stack sizes (words) ────────────────────────────────────── */
#define STACK_MAIN_COMPUTER     512
#define STACK_MOTOR             256
#define STACK_COMMS             256
#define STACK_SELF_MONITOR      256
#define STACK_SENSOR            256
#define STACK_NAVIGATION        512
#define STACK_POWER             128
#define STACK_MCT               256

/* ── Task priorities ────────────────────────────────────────── */
#define PRIORITY_MAIN_COMPUTER  7
#define PRIORITY_MOTOR          6
#define PRIORITY_COMMS          5
#define PRIORITY_SELF_MONITOR   4
#define PRIORITY_SENSOR         3
#define PRIORITY_NAVIGATION     2
#define PRIORITY_POWER          1
#define PRIORITY_MCT            1

/* ── Queue depths ───────────────────────────────────────────── */
#define COMMAND_QUEUE_LENGTH    10
#define SENSOR_QUEUE_LENGTH     20
#define STATUS_QUEUE_LENGTH     15

/* ── Task periods (ms) ──────────────────────────────────────── */
#define PERIOD_MOTOR_MS         500
#define PERIOD_SENSOR_MS        1000
#define PERIOD_COMMS_MS         2000
#define PERIOD_NAVIGATION_MS    1500
#define PERIOD_POWER_MS         3000
#define PERIOD_SELF_MONITOR_MS  2000

/* ── Thresholds ─────────────────────────────────────────────── */
#define POWER_BUDGET_MAX        100
#define HEARTBEAT_TIMEOUT_MS    5000

/* ── Event group bits ───────────────────────────────────────── */
#define EVENT_OBSTACLE_DETECTED   ( 1 << 0 )
#define EVENT_LOW_POWER           ( 1 << 1 )
#define EVENT_COMMS_LOST          ( 1 << 2 )
#define EVENT_SENSOR_FAULT        ( 1 << 3 )
#define EVENT_MOTOR_FAULT         ( 1 << 4 )
#define EVENT_EMERGENCY_STOP      ( 1 << 5 )
#define EVENT_MISSION_COMPLETE    ( 1 << 6 )

/* ── Message types ──────────────────────────────────────────── */
typedef enum {
    CMD_MOVE_FORWARD,
    CMD_MOVE_BACKWARD,
    CMD_TURN_LEFT,
    CMD_TURN_RIGHT,
    CMD_STOP,
    CMD_EMERGENCY_STOP
} MotorCommandType_t;

typedef struct {
    MotorCommandType_t  type;
    uint32_t            speed;          /* 0–100 percent */
    uint32_t            duration_ms;
} MotorCommand_t;

typedef struct {
    int32_t     temperature;            /* Celsius * 100 (fixed point) */
    uint32_t    distance_cm;            /* LIDAR front distance */
    int32_t     altitude_cm;            /* Topographic elevation */
    uint32_t    timestamp_ms;
} SensorData_t;

typedef enum {
    STATUS_OK,
    STATUS_WARNING,
    STATUS_ERROR,
    STATUS_FAULT
} StatusLevel_t;

typedef struct {
    char            task_name[16];
    StatusLevel_t   level;
    uint32_t        error_code;
    char            message[64];
    uint32_t        timestamp_ms;
} StatusReport_t;

typedef struct {
    uint32_t    last_ping_ms;
    bool        is_alive;
} TaskHeartbeat_t;

typedef enum {
    HB_MAIN = 0, HB_MOTOR, HB_COMMS,
    HB_MONITOR, HB_SENSOR, HB_NAV, HB_POWER, HB_MCT
} HeartbeatIndex_t;

/* ── Shared resource handles (defined in system.c) ──────────── */
extern SemaphoreHandle_t    xSensorDataMutex;
extern SemaphoreHandle_t    xMotorMutex;
extern SemaphoreHandle_t    xNavMapMutex;

extern QueueHandle_t        xCommandQueue;
extern QueueHandle_t        xSensorQueue;
extern QueueHandle_t        xStatusQueue;

extern EventGroupHandle_t   xSystemEventGroup;

extern TaskHandle_t         xMainComputerTask;
extern TaskHandle_t         xMotorTaskHandle;
extern TaskHandle_t         xCommsTaskHandle;
extern TaskHandle_t         xSelfMonitorHandle;
extern TaskHandle_t         xSensorTaskHandle;
extern TaskHandle_t         xNavigationHandle;
extern TaskHandle_t         xPowerTaskHandle;
extern TaskHandle_t         xMCTHandle;

extern SensorData_t         xSharedSensorData;
extern TaskHeartbeat_t      xHeartbeats[8];

/* ── Rover odometry — position tracking ─────────────────────── */
typedef struct {
    int32_t     x_cm;           /* Easting  from origin (+E / -W) */
    int32_t     y_cm;           /* Northing from origin (+N / -S) */
    int32_t     heading_deg;    /* 0 = N   90 = E   180 = S   270 = W */
    uint32_t    total_dist_cm;  /* Total path length traveled */
} RoverOdometry_t;

extern RoverOdometry_t      xRoverOdometry;
extern SemaphoreHandle_t    xOdometryMutex;

/* ── Fault injection flags (set via MCT 'fault' command) ────── */
extern volatile bool xFaultWatchdog;   /* suppress feedWatchdog() in all tasks  */
extern volatile bool xFaultComms;      /* suppress Earth contact in Comms task  */

/* ── Helper function prototypes (defined in system.c) ───────── */
void  sendStatusReport(const char *taskName, StatusLevel_t level,
                       uint32_t errorCode, const char *message);
void  updateHeartbeat(HeartbeatIndex_t index);
bool  isTaskAlive(HeartbeatIndex_t index);

/* Returns Euclidean distance from origin in cm.
   Acquires xOdometryMutex internally; returns 0 on timeout. */
float roverOriginDistance(void);

#endif /* SYSTEM_H */
