/* Core exception handlers. Peripheral IRQ handlers live with their drivers. */
#include "stm32f1xx_hal.h"
#include "drivers/board.h"

void SysTick_Handler(void) { HAL_IncTick(); }

void NMI_Handler(void) { board_panic(); }
void HardFault_Handler(void) { board_panic(); }
void MemManage_Handler(void) { board_panic(); }
void BusFault_Handler(void) { board_panic(); }
void UsageFault_Handler(void) { board_panic(); }
