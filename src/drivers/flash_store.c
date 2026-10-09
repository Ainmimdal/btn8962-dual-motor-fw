#include <string.h>
#include "drivers/flash_store.h"
#include "stm32f1xx_hal.h"

bool flash_store_write(const void *data, uint32_t len)
{
    FLASH_EraseInitTypeDef er = {0};
    uint32_t page_err = 0;
    const uint8_t *src = data;
    bool ok = true;

    if (len > FLASH_STORE_SIZE)
        return false;

    HAL_FLASH_Unlock();
    er.TypeErase = FLASH_TYPEERASE_PAGES;
    er.PageAddress = FLASH_STORE_ADDR;
    er.NbPages = 1;
    if (HAL_FLASHEx_Erase(&er, &page_err) != HAL_OK)
        ok = false;
    for (uint32_t off = 0; ok && off < len; off += 4) {
        uint32_t w = 0xFFFFFFFFu;
        memcpy(&w, src + off, len - off >= 4 ? 4 : len - off);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORE_ADDR + off, w) != HAL_OK)
            ok = false;
    }
    HAL_FLASH_Lock();
    return ok && memcmp((const void *)FLASH_STORE_ADDR, data, len) == 0;
}
