#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/* ASCII command protocol + telemetry over UART3. Call from the main loop. */
void protocol_init(void);
void protocol_poll(void);
/* Periodic telemetry and event reporting. */
void protocol_service(uint32_t now_ms);
/* Time of the last valid command from any link (UART or CAN). */
void protocol_feed_watchdog(void);
uint32_t protocol_last_cmd_ms(void);

#endif
