#ifndef POWER_H
#define POWER_H

/* Power task — priority 1 (lowest).
   Monitors power consumption; suspends non-critical tasks when
   budget is exceeded and resumes them when budget recovers. */
void vPowerTask(void *pvParameters);

#endif /* POWER_H */
