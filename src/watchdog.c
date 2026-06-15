#include "system.h"
#include "watchdog.h"

#define WATCHDOG_TIMEOUT_MS  8000

static TimerHandle_t xWatchdogTimer = NULL;

static void prvWatchdogExpired(TimerHandle_t xTimer)
{
    (void)xTimer;
    printf("[Watchdog] TIMEOUT — no task activity for %dms! "
           "Triggering emergency stop.\n", WATCHDOG_TIMEOUT_MS);
    sendStatusReport("Watchdog", STATUS_FAULT, 99,
                     "Watchdog timeout — system unresponsive");
    xEventGroupSetBits(xSystemEventGroup, EVENT_EMERGENCY_STOP);
}

void initWatchdog(void)
{
    xWatchdogTimer = xTimerCreate(
        "Watchdog",
        pdMS_TO_TICKS(WATCHDOG_TIMEOUT_MS),
        pdFALSE,            /* one-shot: must be re-armed by feedWatchdog() */
        NULL,
        prvWatchdogExpired
    );
    configASSERT(xWatchdogTimer != NULL);
    xTimerStart(xWatchdogTimer, 0);
    printf("[Watchdog] Started (timeout %dms)\n", WATCHDOG_TIMEOUT_MS);
}

/* Reset the watchdog countdown — suppressed when fault injection is active */
void feedWatchdog(void)
{
    if (xWatchdogTimer != NULL && !xFaultWatchdog)
        xTimerReset(xWatchdogTimer, 0);
}
