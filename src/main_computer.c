#include "system.h"
#include "main_computer.h"
#include "watchdog.h"
#include <stdlib.h>

/* Orchestrator — reads system-wide events and issues motor commands.
   All other tasks report up through events/queues; this task decides
   what the rover does next. */
void vMainComputerTask(void *pvParameters)
{
    (void)pvParameters;

    static bool     pendingTurn          = false;
    static int      consecutiveObstacles = 0;
    static int      forwardCount         = 0;
    MotorCommand_t  cmd;
    EventBits_t     eventBits;

    printf("[MainComputer] Task started\n");
    sendStatusReport("MainComputer", STATUS_OK, 0, "System boot complete");

    for (;;)
    {
        updateHeartbeat(HB_MAIN);
        feedWatchdog();

        eventBits = xEventGroupGetBits(xSystemEventGroup);

        if (eventBits & EVENT_EMERGENCY_STOP)
        {
            cmd.type        = CMD_EMERGENCY_STOP;
            cmd.speed       = 0;
            cmd.duration_ms = 0;
            xQueueSend(xCommandQueue, &cmd, portMAX_DELAY);
            printf("[MainComputer] EMERGENCY STOP issued\n");
        }
        else if (eventBits & EVENT_OBSTACLE_DETECTED)
        {
            cmd.type        = CMD_STOP;
            cmd.speed       = 0;
            cmd.duration_ms = 500;
            xQueueSend(xCommandQueue, &cmd, pdMS_TO_TICKS(100));
            printf("[MainComputer] Obstacle detected — stopping\n");
            xEventGroupClearBits(xSystemEventGroup, EVENT_OBSTACLE_DETECTED);
            consecutiveObstacles++;
            forwardCount = 0;
            pendingTurn  = true;
        }
        else if (pendingTurn)
        {
            /* Alternate turn direction each obstacle to avoid ping-ponging.
               After 3+ consecutive obstacles we are likely in a corner:
               turn right twice (two ticks) to effectively reverse direction. */
            if (consecutiveObstacles >= 3)
            {
                cmd.type             = CMD_TURN_RIGHT;
                consecutiveObstacles = 1;   /* reset to 1 so next is left */
                pendingTurn          = true; /* issue a second turn next tick */
            }
            else if (consecutiveObstacles % 2 == 1)
            {
                cmd.type    = CMD_TURN_RIGHT;
                pendingTurn = false;
            }
            else
            {
                cmd.type    = CMD_TURN_LEFT;
                pendingTurn = false;
            }
            cmd.speed       = 0;
            cmd.duration_ms = 0;
            xQueueSend(xCommandQueue, &cmd, pdMS_TO_TICKS(100));
            printf("[MainComputer] Obstacle avoidance — turning %s\n",
                   cmd.type == CMD_TURN_LEFT ? "LEFT" : "RIGHT");
            sendStatusReport("MainComputer", STATUS_OK, 0, "Obstacle avoidance turn");
        }
        else if (eventBits & EVENT_LOW_POWER)
        {
            consecutiveObstacles = 0;
            forwardCount++;
            cmd.type        = CMD_MOVE_FORWARD;
            cmd.speed       = 20;
            cmd.duration_ms = 1000;
            xQueueSend(xCommandQueue, &cmd, pdMS_TO_TICKS(100));
            printf("[MainComputer] Low power — reduced speed\n");
        }
        else
        {
            /* Normal mission. Every 8th forward step turn right to sweep
               a new sector — keeps the rover exploring rather than running
               the same straight line indefinitely. */
            consecutiveObstacles = 0;
            forwardCount++;

            if (forwardCount % 8 == 0)
            {
                cmd.type        = CMD_TURN_RIGHT;
                cmd.speed       = 0;
                cmd.duration_ms = 0;
                printf("[MainComputer] Exploration sweep — turning right\n");
            }
            else
            {
                cmd.type        = CMD_MOVE_FORWARD;
                cmd.speed       = 50;
                cmd.duration_ms = 1000;
            }
            xQueueSend(xCommandQueue, &cmd, pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
