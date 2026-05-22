#include "system.h"
#include "main_computer.h"

/* Orchestrator — reads system-wide events and issues motor commands.
   All other tasks report up through events/queues; this task decides
   what the rover does next. */
void vMainComputerTask(void *pvParameters)
{
    (void)pvParameters;

    MotorCommand_t  cmd;
    EventBits_t     eventBits;

    printf("[MainComputer] Task started\n");
    sendStatusReport("MainComputer", STATUS_OK, 0, "System boot complete");

    for (;;)
    {
        updateHeartbeat(HB_MAIN);

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

            /* Clear the bit so we don't keep re-stopping */
            xEventGroupClearBits(xSystemEventGroup, EVENT_OBSTACLE_DETECTED);
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

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
