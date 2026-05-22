#ifndef MOTOR_H
#define MOTOR_H

/* Driver / Motor task — priority 6.
   Receives MotorCommand_t from xCommandQueue, drives motors,
   and signals EVENT_OBSTACLE_DETECTED when proximity is too close. */
void vMotorTask(void *pvParameters);

#endif /* MOTOR_H */
