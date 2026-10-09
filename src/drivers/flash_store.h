#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include <stdbool.h>
#include <stdint.h>

/* One 1 KB page at the top of the 64 KB flash (0x0800FC00) for settings.
 * The firmware is ~30 KB, so it never reaches this page. */
#define FLASH_STORE_ADDR 0x0800FC00u
#define FLASH_STORE_SIZE 1024u

static inline const void *flash_store_addr(void) { return (const void *)FLASH_STORE_ADDR; }
/* Erase the page and write `len` bytes (len <= FLASH_STORE_SIZE). */
bool flash_store_write(const void *data, uint32_t len);

#endif
