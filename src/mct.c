#include "system.h"
#include "mct.h"
#include <ctype.h>

#define MCT_BUF_LEN  128

static void toLower(char *s)
{
    for (; *s; s++) *s = (char)tolower((unsigned char)*s);
}

static void printHelp(void)
{
    printf("\n[MCT] ===== Mission Control Terminal =====\n");
    printf("[MCT]  forward  [speed%%] [duration_ms]\n");
    printf("[MCT]  backward [speed%%] [duration_ms]\n");
    printf("[MCT]  left\n");
    printf("[MCT]  right\n");
    printf("[MCT]  stop\n");
    printf("[MCT]  estop\n");
    printf("[MCT]  status\n");
    printf("[MCT]  fault <watchdog|comms|motor|sensor>\n");
    printf("[MCT]  fault clear\n");
    printf("[MCT]  help\n");
    printf("[MCT] =========================================\n\n");
}

/* ── Fault injection ────────────────────────────────────────── */
static void handleFault(const char *line)
{
    char target[32] = {0};
    sscanf(line, "%*s %31s", target);   /* skip "fault", read next word */

    if (strcmp(target, "watchdog") == 0)
    {
        xFaultWatchdog = true;
        printf("[MCT] FAULT INJECT: watchdog suppressed — fires in ~8s\n");
        sendStatusReport("MCT", STATUS_WARNING, 50, "Fault inject: watchdog");
    }
    else if (strcmp(target, "comms") == 0)
    {
        xFaultComms = true;
        printf("[MCT] FAULT INJECT: Earth contact suppressed — COMMS LOST in ~3s\n");
        sendStatusReport("MCT", STATUS_WARNING, 51, "Fault inject: comms lost");
    }
    else if (strcmp(target, "motor") == 0)
    {
        if (xMotorTaskHandle != NULL)
        {
            printf("[MCT] FAULT INJECT: suspending Motor task — SelfMonitor detects in ~5s\n");
            sendStatusReport("MCT", STATUS_WARNING, 52, "Fault inject: motor freeze");
            vTaskSuspend(xMotorTaskHandle);
        }
    }
    else if (strcmp(target, "sensor") == 0)
    {
        if (xSensorTaskHandle != NULL)
        {
            printf("[MCT] FAULT INJECT: suspending Sensor task — SelfMonitor detects in ~5s\n");
            sendStatusReport("MCT", STATUS_WARNING, 53, "Fault inject: sensor freeze");
            vTaskSuspend(xSensorTaskHandle);
        }
    }
    else if (strcmp(target, "clear") == 0)
    {
        xFaultWatchdog = false;
        xFaultComms    = false;
        /* Resume motor/sensor if they were suspended by fault injection */
        if (xMotorTaskHandle  != NULL &&
            eTaskGetState(xMotorTaskHandle) == eSuspended)
            vTaskResume(xMotorTaskHandle);
        if (xSensorTaskHandle != NULL &&
            eTaskGetState(xSensorTaskHandle) == eSuspended)
            vTaskResume(xSensorTaskHandle);
        xEventGroupClearBits(xSystemEventGroup,
                             EVENT_EMERGENCY_STOP | EVENT_COMMS_LOST |
                             EVENT_SENSOR_FAULT   | EVENT_MOTOR_FAULT);
        printf("[MCT] FAULT CLEAR: all fault flags reset\n");
        sendStatusReport("MCT", STATUS_OK, 0, "Fault flags cleared");
    }
    else
    {
        printf("[MCT] Unknown fault target '%s'. "
               "Use: watchdog | comms | motor | sensor | clear\n", target);
    }
}

static void printStatus(void)
{
    int32_t  x = 0, y = 0, heading = 0;
    uint32_t total = 0;

    if (xSemaphoreTake(xOdometryMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        x       = xRoverOdometry.x_cm;
        y       = xRoverOdometry.y_cm;
        heading = xRoverOdometry.heading_deg;
        total   = xRoverOdometry.total_dist_cm;
        xSemaphoreGive(xOdometryMutex);
    }

    float dist = roverOriginDistance();

    printf("[MCT] Status — Pos:(%ld,%ld)cm  Heading:%ld deg  "
           "Path:%lu cm  Origin:%.1f cm\n",
           (long)x, (long)y, (long)heading,
           (unsigned long)total, dist);
}

void vMCTTask(void *pvParameters)
{
    (void)pvParameters;

    char          line[MCT_BUF_LEN];
    char          word[32];
    MotorCommand_t cmd;

    printf("\n[MCT] Mission Control Terminal ready.\n");
    printHelp();

    for (;;)
    {
        updateHeartbeat(HB_MCT);

        printf("[MCT] > ");
        fflush(stdout);

        if (fgets(line, sizeof(line), stdin) == NULL)
        {
            /* stdin closed — park the task */
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        /* strip newline */
        line[strcspn(line, "\n")] = '\0';

        /* default command parameters */
        unsigned long spd = 50, dur = 1000;
        word[0] = '\0';

        sscanf(line, "%31s %lu %lu", word, &spd, &dur);
        toLower(word);

        cmd.speed       = (uint32_t)spd;
        cmd.duration_ms = (uint32_t)dur;

        if (strcmp(word, "forward") == 0)
        {
            cmd.type = CMD_MOVE_FORWARD;
            printf("[MCT] >> FORWARD  speed:%lu%%  duration:%lu ms\n",
                   (unsigned long)cmd.speed, (unsigned long)cmd.duration_ms);
            xQueueSend(xCommandQueue, &cmd, pdMS_TO_TICKS(200));
        }
        else if (strcmp(word, "backward") == 0)
        {
            cmd.type = CMD_MOVE_BACKWARD;
            printf("[MCT] >> BACKWARD  speed:%lu%%  duration:%lu ms\n",
                   (unsigned long)cmd.speed, (unsigned long)cmd.duration_ms);
            xQueueSend(xCommandQueue, &cmd, pdMS_TO_TICKS(200));
        }
        else if (strcmp(word, "left") == 0)
        {
            cmd.type        = CMD_TURN_LEFT;
            cmd.speed       = 0;
            cmd.duration_ms = 0;
            printf("[MCT] >> TURN LEFT\n");
            xQueueSend(xCommandQueue, &cmd, pdMS_TO_TICKS(200));
        }
        else if (strcmp(word, "right") == 0)
        {
            cmd.type        = CMD_TURN_RIGHT;
            cmd.speed       = 0;
            cmd.duration_ms = 0;
            printf("[MCT] >> TURN RIGHT\n");
            xQueueSend(xCommandQueue, &cmd, pdMS_TO_TICKS(200));
        }
        else if (strcmp(word, "stop") == 0)
        {
            cmd.type        = CMD_STOP;
            cmd.speed       = 0;
            cmd.duration_ms = 0;
            printf("[MCT] >> STOP\n");
            xQueueSend(xCommandQueue, &cmd, pdMS_TO_TICKS(200));
        }
        else if (strcmp(word, "estop") == 0)
        {
            cmd.type        = CMD_EMERGENCY_STOP;
            cmd.speed       = 0;
            cmd.duration_ms = 0;
            xEventGroupSetBits(xSystemEventGroup, EVENT_EMERGENCY_STOP);
            printf("[MCT] >> !!! EMERGENCY STOP !!!\n");
            xQueueSend(xCommandQueue, &cmd, pdMS_TO_TICKS(200));
        }
        else if (strcmp(word, "status") == 0)
        {
            printStatus();
        }
        else if (strcmp(word, "fault") == 0)
        {
            handleFault(line);
        }
        else if (strcmp(word, "help") == 0 || strcmp(word, "?") == 0)
        {
            printHelp();
        }
        else if (word[0] != '\0')
        {
            printf("[MCT] Unknown command: '%s'  (type 'help' for commands)\n",
                   word);
        }

        updateHeartbeat(HB_MCT);
    }
}
