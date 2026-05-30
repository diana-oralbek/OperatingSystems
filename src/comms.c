#include "system.h"
#include "comms.h"
#include <stdlib.h>

/* Silence duration (ms) before declaring Earth signal lost */
#define COMMS_LOSS_TIMEOUT_MS   3000
/* Per-cycle probability (0–99) that Earth sends a contact ping */
#define COMMS_CONTACT_CHANCE    70
/* Per-cycle probability (0–99) that the ping carries an actual command */
#define COMMS_CMD_CHANCE        25

static const char *cmdTypeName(MotorCommandType_t t)
{
    switch (t) {
        case CMD_MOVE_FORWARD:   return "FORWARD";
        case CMD_MOVE_BACKWARD:  return "BACKWARD";
        case CMD_TURN_LEFT:      return "TURN_LEFT";
        case CMD_TURN_RIGHT:     return "TURN_RIGHT";
        case CMD_STOP:           return "STOP";
        case CMD_EMERGENCY_STOP: return "EMERGENCY_STOP";
        default:                 return "UNKNOWN";
    }
}

void vCommsTask(void *pvParameters)
{
    (void)pvParameters;

    StatusReport_t  report;
    MotorCommand_t  earthCmd;
    uint32_t        lastContactMs;
    bool            commsLost = false;

    static const MotorCommandType_t earthCmdPool[] = {
        CMD_MOVE_FORWARD, CMD_MOVE_FORWARD,
        CMD_TURN_LEFT, CMD_TURN_RIGHT,
        CMD_MOVE_BACKWARD, CMD_STOP
    };

    printf("[Comms] Task started\n");
    lastContactMs = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    for (;;)
    {
        updateHeartbeat(HB_COMMS);

        /* ── Outgoing: drain status queue and transmit to Earth ─── */
        while (xQueueReceive(xStatusQueue, &report, 0) == pdTRUE)
        {
            printf("[Comms] TX >> [%s] Level:%d Code:%lu — %s\n",
                   report.task_name,
                   (int)report.level,
                   (unsigned long)report.error_code,
                   report.message);
        }

        /* ── Incoming: simulate Earth contact window ────────────── */
        if ((rand() % 100) < COMMS_CONTACT_CHANCE)
        {
            lastContactMs = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

            if ((rand() % 100) < COMMS_CMD_CHANCE)
            {
                earthCmd.type        = earthCmdPool[rand() % 6];
                earthCmd.speed       = 30 + (uint32_t)(rand() % 41);
                earthCmd.duration_ms = 500 + (uint32_t)(rand() % 1001);

                if (xQueueSend(xCommandQueue, &earthCmd, 0) == pdTRUE)
                    printf("[Comms] RX << Earth cmd: %s  speed:%lu%%\n",
                           cmdTypeName(earthCmd.type),
                           (unsigned long)earthCmd.speed);
            }
        }

        /* ── Signal-loss detection ──────────────────────────────── */
        uint32_t nowMs    = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        uint32_t silenceMs = nowMs - lastContactMs;

        if (!commsLost && silenceMs > COMMS_LOSS_TIMEOUT_MS)
        {
            commsLost = true;
            xEventGroupSetBits(xSystemEventGroup, EVENT_COMMS_LOST);
            sendStatusReport("Comms", STATUS_ERROR, 10, "Earth signal lost");
            printf("[Comms] !!! Earth signal LOST — silence: %lums\n",
                   (unsigned long)silenceMs);
        }
        else if (commsLost && silenceMs <= COMMS_LOSS_TIMEOUT_MS)
        {
            commsLost = false;
            xEventGroupClearBits(xSystemEventGroup, EVENT_COMMS_LOST);
            sendStatusReport("Comms", STATUS_OK, 0, "Earth signal restored");
            printf("[Comms] Earth signal RESTORED\n");
        }

        vTaskDelay(pdMS_TO_TICKS(PERIOD_COMMS_MS));
    }
}
