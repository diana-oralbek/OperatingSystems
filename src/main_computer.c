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

    static bool     pendingTurn = false;
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
            pendingTurn = true;
        }
        else if (pendingTurn)
        {
            /* One tick after stopping: turn to avoid the obstacle */
            cmd.type        = (rand() % 2) ? CMD_TURN_LEFT : CMD_TURN_RIGHT;
            cmd.speed       = 0;
            cmd.duration_ms = 0;
            xQueueSend(xCommandQueue, &cmd, pdMS_TO_TICKS(100));
            printf("[MainComputer] Obstacle avoidance — turning %s\n",
                   cmd.type == CMD_TURN_LEFT ? "LEFT" : "RIGHT");
            sendStatusReport("MainComputer", STATUS_OK, 0, "Obstacle avoidance turn");
            pendingTurn = false;
        }
        else if (eventBits & EVENT_LOW_POWER)
        {
            /* Reduce operations: slow down to conserve energy */
            cmd.type        = CMD_MOVE_FORWARD;
            cmd.speed       = 20;
            cmd.duration_ms = 1000;
            xQueueSend(xCommandQueue, &cmd, pdMS_TO_TICKS(100));
            printf("[MainComputer] Low power — reduced speed\n");
        }
        else
        {
            /* Normal mission: advance at cruise speed */
            cmd.type        = CMD_MOVE_FORWARD;
            cmd.speed       = 50;
            cmd.duration_ms = 1000;
            xQueueSend(xCommandQueue, &cmd, pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
