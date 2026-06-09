#ifndef WATCHDOG_H
#define WATCHDOG_H

/* Hardware-style software watchdog backed by a FreeRTOS one-shot timer.
 * Any task must call feedWatchdog() within WATCHDOG_TIMEOUT_MS or the
 * timer fires and triggers EVENT_EMERGENCY_STOP. */

void initWatchdog(void);
void feedWatchdog(void);

#endif /* WATCHDOG_H */
