#ifndef MAIN_COMPUTER_H
#define MAIN_COMPUTER_H

/* Orchestrator task — highest priority (7).
   Reads system events and dispatches motor commands. */
void vMainComputerTask(void *pvParameters);

#endif /* MAIN_COMPUTER_H */
