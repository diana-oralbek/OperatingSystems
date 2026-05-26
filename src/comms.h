#ifndef COMMS_H
#define COMMS_H

/* Communication task — priority 5.
   Drains xStatusQueue and transmits reports to Earth;
   receives incoming commands and forwards them to xCommandQueue. */
void vCommsTask(void *pvParameters);

#endif /* COMMS_H */
