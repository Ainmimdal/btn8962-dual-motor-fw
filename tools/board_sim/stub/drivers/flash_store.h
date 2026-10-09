#pragma once
#include <stdbool.h>
#include <stdint.h>
#define FLASH_STORE_SIZE 1024u
extern uint8_t sim_flash[FLASH_STORE_SIZE];
static inline const void *flash_store_addr(void) { return sim_flash; }
bool flash_store_write(const void *data, uint32_t len);
