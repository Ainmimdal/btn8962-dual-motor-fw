#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t id;
    uint8_t len;
    bool rtr;           /* remote (request) frame: no data, len = requested DLC */
    uint8_t data[8];
} can_frame_t;

/* bxCAN on PA11/PA12, standard IDs. kbps is 125, 250, 500 or 1000 (anything
 * else falls back to 125). Nothing is received until can_bus_set_filter().
 * Returns false if the controller didn't start (e.g. no transceiver/bus). */
bool can_bus_init(uint32_t kbps);
/* Accept data and remote frames for two ODrive-style node IDs
 * (ID bits 10..5 = node), any command. Can be called again at runtime. */
void can_bus_set_filter(uint8_t node_a, uint8_t node_b);
/* Queue a data frame (software queue in front of the 3 mailboxes); false if
 * the queue is full and the frame was dropped. Main-loop only. */
bool can_bus_send(uint16_t id, const uint8_t *data, uint8_t len);
/* Move queued frames into free mailboxes; call every main-loop pass. */
void can_bus_service(void);
/* Pop a received frame; false if none. */
bool can_bus_recv(can_frame_t *f);
/* Error state register (ESR) for diagnostics. */
uint32_t can_bus_esr(void);

#endif
