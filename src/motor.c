#include "system.h"
#include "motor.h"
#include <stdlib.h>     /* rand() */

/* Obstacle distance is managed by the motor's own physics model.
 * Moving forward decreases it, backward increases it, turning resets it. */
#define SIM_DIST_INITIAL_CM     200
#define SIM_DIST_MIN_CM          20
#define SIM_DIST_MAX_CM         500
#define OBSTACLE_THRESHOLD_CM    50

static int32_t simObstacleDist = SIM_DIST_INITIAL_CM;

/* ── Heading helpers ────────────────────────────────────────── */
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

/* ── Odometry update ────────────────────────────────────────────
 *  Updates x/y position (dead-reckoning) and obstacle distance
 *  based on the executed command. Clamps distance to valid range.
 * ─────────────────────────────────────────────────────────────── */
static void updateOdometry(const MotorCommand_t *cmd)
{
    /* 100% speed for 1000 ms ≈ 100 cm traveled; ±10 cm terrain variation */
    uint32_t moved_cm = (cmd->speed * cmd->duration_ms) / 1000;
    int32_t  terrain  = (rand() % 21) - 10;

    if (xSemaphoreTake(xOdometryMutex, pdMS_TO_TICKS(50)) != pdTRUE)
        return;

    switch (cmd->type)
    {
        case CMD_MOVE_FORWARD:
            simObstacleDist -= (int32_t)moved_cm + terrain;

            switch (xRoverOdometry.heading_deg) {
                case   0: xRoverOdometry.y_cm += (int32_t)moved_cm; break;
                case  90: xRoverOdometry.x_cm += (int32_t)moved_cm; break;
                case 180: xRoverOdometry.y_cm -= (int32_t)moved_cm; break;
                case 270: xRoverOdometry.x_cm -= (int32_t)moved_cm; break;
            }
            xRoverOdometry.total_dist_cm += moved_cm;
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
            break;

        case CMD_TURN_LEFT:
            xRoverOdometry.heading_deg = (xRoverOdometry.heading_deg - 90 + 360) % 360;
            if (simObstacleDist < OBSTACLE_THRESHOLD_CM)
                simObstacleDist = 150 + (rand() % 120);
            else
                simObstacleDist += (rand() % 40) - 20;
            break;

        case CMD_TURN_RIGHT:
            xRoverOdometry.heading_deg = (xRoverOdometry.heading_deg + 90) % 360;
            if (simObstacleDist < OBSTACLE_THRESHOLD_CM)
                simObstacleDist = 150 + (rand() % 120);
            else
                simObstacleDist += (rand() % 40) - 20;
            break;

        case CMD_STOP:
        case CMD_EMERGENCY_STOP:
            if (simObstacleDist < OBSTACLE_THRESHOLD_CM)
                simObstacleDist += 20 + (rand() % 30);
            break;
    }

    /* Hard clamp */
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

    MotorCommand_t cmd;

    printf("[Motor] Task started\n");

    for (;;)
    {
        updateHeartbeat(HB_MOTOR);

        /* Block waiting for a command from MainComputer (500 ms timeout) */
        if (xQueueReceive(xCommandQueue, &cmd, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            if (xSemaphoreTake(xMotorMutex, pdMS_TO_TICKS(200)) == pdTRUE)
            {
                executeMotorCommand(&cmd);
                updateOdometry(&cmd);

                /* Show distance from origin only when rover is actually moving */
                if (cmd.type == CMD_MOVE_FORWARD || cmd.type == CMD_MOVE_BACKWARD)

                
                printPosition();

                /* Signal obstacle event if too close after a forward move */
                if (cmd.type == CMD_MOVE_FORWARD &&
                    simObstacleDist < OBSTACLE_THRESHOLD_CM)
                {
                    printf("[Motor] Obstacle at %ldcm — alerting MainComputer\n",
                           (long)simObstacleDist);
                    xEventGroupSetBits(xSystemEventGroup, EVENT_OBSTACLE_DETECTED);
                    sendStatusReport("Motor", STATUS_WARNING, 1,
                                     "Obstacle within stopping distance");
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
