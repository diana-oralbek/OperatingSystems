#ifndef SELF_MONITOR_H
#define SELF_MONITOR_H

/* Self-Monitor / Watchdog task — priority 4.
   Checks heartbeat timestamps for all other tasks and raises
   fault events when a task appears frozen. */
void vSelfMonitorTask(void *pvParameters);

#endif /* SELF_MONITOR_H */
