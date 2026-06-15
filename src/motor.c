#include "system.h"
#include "motor.h"
#include "watchdog.h"
#include <stdlib.h>     /* rand() */

#define SIM_DIST_INITIAL_CM     300
#define SIM_DIST_MIN_CM          20
#define SIM_DIST_MAX_CM         500
#define OBSTACLE_THRESHOLD_CM    50

/* 2-D obstacle memory — so the rover respects obstacles it has seen before,
   even after turning away and returning on the same heading. */
#define MAX_STORED_OBSTACLES    50
#define OBSTACLE_MERGE_CM       80
#define OBSTACLE_PATH_WIDTH     60   /* lateral tolerance to count as "in path" */

#define ABS32(x) ((x) < 0 ? -(x) : (x))

typedef struct { int32_t x; int32_t y; } Obs2D_t;
static Obs2D_t  knownObstacles[MAX_STORED_OBSTACLES];
static int      knownObstacleCount = 0;

static int32_t simObstacleDist = SIM_DIST_INITIAL_CM;

static void recordObstacle(int32_t rx, int32_t ry, int32_t hdg, int32_t dist)
{
    int32_t ox, oy;
    switch (hdg) {
        case   0: ox = rx;        oy = ry + dist; break;
        case  90: ox = rx + dist; oy = ry;        break;
        case 180: ox = rx;        oy = ry - dist; break;
        case 270: ox = rx - dist; oy = ry;        break;
        default: return;
    }
    for (int i = 0; i < knownObstacleCount; i++)
        if (ABS32(knownObstacles[i].x - ox) <= OBSTACLE_MERGE_CM &&
            ABS32(knownObstacles[i].y - oy) <= OBSTACLE_MERGE_CM)
            return;
    if (knownObstacleCount < MAX_STORED_OBSTACLES)
    {
        knownObstacles[knownObstacleCount].x = ox;
        knownObstacles[knownObstacleCount].y = oy;
        knownObstacleCount++;
    }
}

static void applyKnownObstacles(int32_t rx, int32_t ry, int32_t hdg)
{
    for (int i = 0; i < knownObstacleCount; i++)
    {
        int32_t ox = knownObstacles[i].x;
        int32_t oy = knownObstacles[i].y;
        int32_t along, lateral;
        switch (hdg) {
            case   0: along = oy - ry; lateral = ABS32(ox - rx); break;
            case  90: along = ox - rx; lateral = ABS32(oy - ry); break;
            case 180: along = ry - oy; lateral = ABS32(ox - rx); break;
            case 270: along = rx - ox; lateral = ABS32(oy - ry); break;
            default: continue;
        }
        if (along > 0 && lateral <= OBSTACLE_PATH_WIDTH && along < simObstacleDist)
            simObstacleDist = along;
    }
}

static const char *headingToStr(int32_t deg)
{
    switch (deg) {
        case   0: return "N";
        case  90: return "E";
        case 180: return "S";
        case 270: return "W";
        default:  return "?";
    }
}

static void updateOdometry(const MotorCommand_t *cmd)
{
    /* 100% speed for 1000 ms ≈ 100 cm traveled; ±5 cm terrain variation */
    uint32_t moved_cm = (cmd->speed * cmd->duration_ms) / 1000;
    int32_t  terrain  = (rand() % 11) - 5;

    if (xSemaphoreTake(xOdometryMutex, pdMS_TO_TICKS(50)) != pdTRUE)
        return;

    switch (cmd->type)
    {
        case CMD_MOVE_FORWARD:
            {
                int32_t clearance = simObstacleDist - SIM_DIST_MIN_CM;
                if (clearance < 0) clearance = 0;
                if ((int32_t)moved_cm > clearance)
                    moved_cm = (uint32_t)clearance;
            }
            simObstacleDist -= (int32_t)moved_cm + terrain;

            switch (xRoverOdometry.heading_deg) {
                case   0: xRoverOdometry.y_cm += (int32_t)moved_cm; break;
                case  90: xRoverOdometry.x_cm += (int32_t)moved_cm; break;
                case 180: xRoverOdometry.y_cm -= (int32_t)moved_cm; break;
                case 270: xRoverOdometry.x_cm -= (int32_t)moved_cm; break;
            }
            xRoverOdometry.total_dist_cm += moved_cm;

            if (simObstacleDist < OBSTACLE_THRESHOLD_CM)
                recordObstacle(xRoverOdometry.x_cm, xRoverOdometry.y_cm,
                               xRoverOdometry.heading_deg, simObstacleDist);

            applyKnownObstacles(xRoverOdometry.x_cm, xRoverOdometry.y_cm,
                                xRoverOdometry.heading_deg);
            break;

        case CMD_MOVE_BACKWARD:
            simObstacleDist += (int32_t)moved_cm + terrain;

            switch (xRoverOdometry.heading_deg) {
                case   0: xRoverOdometry.y_cm -= (int32_t)moved_cm; break;
                case  90: xRoverOdometry.x_cm -= (int32_t)moved_cm; break;
                case 180: xRoverOdometry.y_cm += (int32_t)moved_cm; break;
                case 270: xRoverOdometry.x_cm += (int32_t)moved_cm; break;
            }
            xRoverOdometry.total_dist_cm += moved_cm;

            applyKnownObstacles(xRoverOdometry.x_cm, xRoverOdometry.y_cm,
                                xRoverOdometry.heading_deg);
            break;

        case CMD_TURN_LEFT:
            xRoverOdometry.heading_deg = (xRoverOdometry.heading_deg - 90 + 360) % 360;
            simObstacleDist = 150 + (rand() % 250);
            applyKnownObstacles(xRoverOdometry.x_cm, xRoverOdometry.y_cm,
                                xRoverOdometry.heading_deg);
            break;

        case CMD_TURN_RIGHT:
            xRoverOdometry.heading_deg = (xRoverOdometry.heading_deg + 90) % 360;
            simObstacleDist = 150 + (rand() % 250);
            applyKnownObstacles(xRoverOdometry.x_cm, xRoverOdometry.y_cm,
                                xRoverOdometry.heading_deg);
            break;

        case CMD_STOP:
        case CMD_EMERGENCY_STOP:
            if (simObstacleDist < OBSTACLE_THRESHOLD_CM)
                simObstacleDist += 20 + (rand() % 30);
            break;
    }

    if (simObstacleDist < SIM_DIST_MIN_CM) simObstacleDist = SIM_DIST_MIN_CM;
    if (simObstacleDist > SIM_DIST_MAX_CM) simObstacleDist = SIM_DIST_MAX_CM;

    xSemaphoreGive(xOdometryMutex);
}

/* ── Position report ────────────────────────────────────────────
 *  Printed after every movement command.
 * ─────────────────────────────────────────────────────────────── */
static void printPosition(void)
{
    if (xSemaphoreTake(xOdometryMutex, pdMS_TO_TICKS(20)) != pdTRUE)
        return;

    int32_t  x     = xRoverOdometry.x_cm;
    int32_t  y     = xRoverOdometry.y_cm;
    int32_t  hdg   = xRoverOdometry.heading_deg;
    uint32_t total = xRoverOdometry.total_dist_cm;

    xSemaphoreGive(xOdometryMutex);

    float originDist = roverOriginDistance();

    printf("[Motor] Pos:(%ld, %ld)cm  Heading:%s  Path:%lucm  Obstacle:%ldcm\n",
           (long)x, (long)y,
           headingToStr(hdg),
           (unsigned long)total,
           (long)simObstacleDist);

    printf("[Motor] >> Distance from start: %.1f cm\n", originDist);
}

/* ── Command execution ──────────────────────────────────────── */
static void executeMotorCommand(const MotorCommand_t *cmd)
{
    switch (cmd->type)
    {
        case CMD_MOVE_FORWARD:
            printf("[Motor] FORWARD  speed:%lu%%  dur:%lums  "
                   "obstacle ahead: %ldcm\n",
                   (unsigned long)cmd->speed,
                   (unsigned long)cmd->duration_ms,
                   (long)simObstacleDist);
            break;
        case CMD_MOVE_BACKWARD:
            printf("[Motor] BACKWARD  speed:%lu%%  dur:%lums\n",
                   (unsigned long)cmd->speed,
                   (unsigned long)cmd->duration_ms);
            break;
        case CMD_TURN_LEFT:
            printf("[Motor] TURN LEFT\n");
            break;
        case CMD_TURN_RIGHT:
            printf("[Motor] TURN RIGHT\n");
            break;
        case CMD_STOP:
            printf("[Motor] STOP\n");
            break;
        case CMD_EMERGENCY_STOP:
            printf("[Motor] !!! EMERGENCY STOP !!!\n");
            break;
    }
}

/* ── Task entry point ───────────────────────────────────────── */
void vMotorTask(void *pvParameters)
{
    (void)pvParameters;

    static bool    obstacleWasSet = false;
    MotorCommand_t cmd;

    printf("[Motor] Task started\n");
    printPosition();

    for (;;)
    {
        updateHeartbeat(HB_MOTOR);
        feedWatchdog();

        if (xQueueReceive(xCommandQueue, &cmd, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            if (xSemaphoreTake(xMotorMutex, pdMS_TO_TICKS(200)) == pdTRUE)
            {
                executeMotorCommand(&cmd);
                updateOdometry(&cmd);

                if (cmd.type == CMD_MOVE_FORWARD || cmd.type == CMD_MOVE_BACKWARD)
                    printPosition();

                if (cmd.type == CMD_MOVE_FORWARD &&
                    simObstacleDist < OBSTACLE_THRESHOLD_CM)
                {
                    obstacleWasSet = true;
                    printf("[Motor] Obstacle at %ldcm — alerting MainComputer\n",
                           (long)simObstacleDist);
                    xEventGroupSetBits(xSystemEventGroup, EVENT_OBSTACLE_DETECTED);
                    sendStatusReport("Motor", STATUS_WARNING, 1,
                                     "Obstacle within stopping distance");
                }
                else if (cmd.type == CMD_MOVE_FORWARD && obstacleWasSet)
                {
                    obstacleWasSet = false;
                    sendStatusReport("Motor", STATUS_OK, 0, "Path clear");
                }

                xSemaphoreGive(xMotorMutex);
            }
            else
            {
                sendStatusReport("Motor", STATUS_WARNING, 2,
                                 "Motor mutex timeout — command dropped");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(PERIOD_MOTOR_MS));
    }
}
