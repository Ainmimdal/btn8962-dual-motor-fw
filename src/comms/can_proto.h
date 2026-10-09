#ifndef CAN_PROTO_H
#define CAN_PROTO_H

#include <stdbool.h>
#include <stdint.h>

/* ODrive 0.5.x "CAN Simple" compatible protocol, one node per motor, plus a
 * vendor extension on commands 0x1E/0x1F (see can_proto.c and the README). */
void can_proto_init(bool bus_ok);
void can_proto_poll(uint32_t now_ms);
/* The command watchdog stopped the motors: reported as the ODrive
 * WATCHDOG_TIMER_EXPIRED axis error until errors are cleared. */
void can_proto_note_watchdog(void);

/* Provided by main.c (or the simulator): motors off, then reset the MCU. */
void system_reboot(void);

#endif
