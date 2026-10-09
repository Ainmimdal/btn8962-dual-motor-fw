/*
 * pins.h - every pin assignment, named after the schematic nets
 * (btn7971pcb_layoutrevision, U11 = STM32F103C8T6, LQFP48 pin numbers).
 *
 * Nothing outside this file should name a GPIO port/pin directly.
 */
#ifndef PINS_H
#define PINS_H

#include "stm32f1xx_hal.h"
#include "config.h"

/* ---- Half-bridge IN, TIM2 PWM (via 1k series R) ------------------------- */
/* Net P1: PA3, pin 13, TIM2_CH4 -> R2  -> U6 IN  (motor 1, side A, OUT1)    */
#define P1_GPIO_Port      GPIOA
#define P1_Pin            GPIO_PIN_3
#define P1_TIM_CHANNEL    TIM_CHANNEL_4
#define P1_CCR            (TIM2->CCR4)
/* Net P2: PA2, pin 12, TIM2_CH3 -> R25 -> U8 IN  (motor 1, side B, OUT2)    */
#define P2_GPIO_Port      GPIOA
#define P2_Pin            GPIO_PIN_2
#define P2_TIM_CHANNEL    TIM_CHANNEL_3
#define P2_CCR            (TIM2->CCR3)
/* Net P3: PA1, pin 11, TIM2_CH2 -> R49 -> U2 IN  (motor 2, side A, OUT3)    */
#define P3_GPIO_Port      GPIOA
#define P3_Pin            GPIO_PIN_1
#define P3_TIM_CHANNEL    TIM_CHANNEL_2
#define P3_CCR            (TIM2->CCR2)
/* Net P4: PA0, pin 10, TIM2_CH1 -> R50 -> U3 IN  (motor 2, side B, OUT4)    */
#define P4_GPIO_Port      GPIOA
#define P4_Pin            GPIO_PIN_0
#define P4_TIM_CHANNEL    TIM_CHANNEL_1
#define P4_CCR            (TIM2->CCR1)

/* ---- Half-bridge INH enables (10k pull-down on board) ------------------- */
/* Net E1: PC13, pin 2 -> U6 + U8 INH (motor 1)                              */
#define E1_GPIO_Port      GPIOC
#define E1_Pin            GPIO_PIN_13
/* Net E2: PC14, pin 3 -> U2 + U3 INH (motor 2)                              */
#define E2_GPIO_Port      GPIOC
#define E2_Pin            GPIO_PIN_14

/* ---- Current sense (BTN8962 IS -> 3.3k to GND) -------------------------- */
#define IS1_GPIO_Port     GPIOA      /* PA4, pin 14, U6 IS (R11) */
#define IS1_Pin           GPIO_PIN_4
#define IS1_ADC_CHANNEL   ADC_CHANNEL_4
#define IS2_GPIO_Port     GPIOA      /* PA5, pin 15, U8 IS (R14) */
#define IS2_Pin           GPIO_PIN_5
#define IS2_ADC_CHANNEL   ADC_CHANNEL_5
#define IS3_GPIO_Port     GPIOB      /* PB0, pin 18, U2 IS (R17) */
#define IS3_Pin           GPIO_PIN_0
#define IS3_ADC_CHANNEL   ADC_CHANNEL_8
#define IS4_GPIO_Port     GPIOB      /* PB1, pin 19, U3 IS (R20) */
#define IS4_Pin           GPIO_PIN_1
#define IS4_ADC_CHANNEL   ADC_CHANNEL_9

/* ---- Encoder nets as drawn (CN1 / CN2, encoders powered from +5V) ------- */
#define ENC_A1_GPIO_Port  GPIOA      /* PA6, pin 16, CN1-2, TIM3_CH1 (not 5V tolerant) */
#define ENC_A1_Pin        GPIO_PIN_6
#define ENC_B1_GPIO_Port  GPIOA      /* PA7, pin 17, CN1-3, TIM3_CH2 (not 5V tolerant) */
#define ENC_B1_Pin        GPIO_PIN_7
#define ENC_Z1_GPIO_Port  GPIOA      /* PA8, pin 29, CN1-1, TIM1_CH1 (5V tolerant) */
#define ENC_Z1_Pin        GPIO_PIN_8
#define ENC_A2_GPIO_Port  GPIOB      /* PB6, pin 42, CN2-2, TIM4_CH1 */
#define ENC_A2_Pin        GPIO_PIN_6
#define ENC_B2_GPIO_Port  GPIOB      /* PB7, pin 43, CN2-3, TIM4_CH2 */
#define ENC_B2_Pin        GPIO_PIN_7
#define ENC_Z2_GPIO_Port  GPIOB      /* PB9, pin 46, CN2-1, not used by this firmware */
#define ENC_Z2_Pin        GPIO_PIN_9

/* ---- RS485 nets (MAX3485 U1) - never used by this firmware -------------- */
#define RS485_TX_GPIO_Port    GPIOA  /* PA9,  pin 30, U1 DI */
#define RS485_TX_Pin          GPIO_PIN_9
#define RS485_RX_GPIO_Port    GPIOA  /* PA10, pin 31, U1 RO */
#define RS485_RX_Pin          GPIO_PIN_10
#define RS485_DE_RE_GPIO_Port GPIOB  /* PB12, pin 25, U1 DE + RE# */
#define RS485_DE_RE_Pin       GPIO_PIN_12

/* ---- Encoder 1 as actually used, per ENC1_CONFIG ------------------------ */
#if ENC1_CONFIG == ENC1_ON_TIM1
/* Rework: encoder A -> net ENC_Z1 (CN1-1, PA8), encoder B -> net RS485_TX
 * (PA9, U1 pad 4, U1 not fitted). CN1-2/CN1-3 (PA6/PA7) must stay open. */
#define ENC1_TIM          TIM1
#define ENC1_A_GPIO_Port  ENC_Z1_GPIO_Port
#define ENC1_A_Pin        ENC_Z1_Pin
#define ENC1_B_GPIO_Port  RS485_TX_GPIO_Port
#define ENC1_B_Pin        RS485_TX_Pin
#define ENC1_TIM_CLK_ENABLE() __HAL_RCC_TIM1_CLK_ENABLE()
#else /* ENC1_ON_TIM3_3V3 */
#define ENC1_TIM          TIM3
#define ENC1_A_GPIO_Port  ENC_A1_GPIO_Port
#define ENC1_A_Pin        ENC_A1_Pin
#define ENC1_B_GPIO_Port  ENC_B1_GPIO_Port
#define ENC1_B_Pin        ENC_B1_Pin
#define ENC1_TIM_CLK_ENABLE() __HAL_RCC_TIM3_CLK_ENABLE()
#define ENC1_Z_GPIO_Port  ENC_Z1_GPIO_Port   /* optional index, ENC1_INDEX_ENABLE */
#define ENC1_Z_Pin        ENC_Z1_Pin
#endif

#define ENC2_TIM          TIM4
#define ENC2_TIM_CLK_ENABLE() __HAL_RCC_TIM4_CLK_ENABLE()

/* ---- Limit switches (switch to GND, no external pull-up) ---------------- */
#define LIMIT1_GPIO_Port  GPIOB      /* PB15, pin 28, U13 */
#define LIMIT1_Pin        GPIO_PIN_15
#define LIMIT2_GPIO_Port  GPIOB      /* PB2, pin 20, U15 (also BOOT1; BOOT0 is pulled low) */
#define LIMIT2_Pin        GPIO_PIN_2

/* ---- LEDs: +3V3 -> LED -> 100R -> pin, so active LOW -------------------- */
#define STATUS_GPIO_Port  GPIOB      /* PB8, pin 45, U18 green */
#define STATUS_Pin        GPIO_PIN_8
#define ERROR_GPIO_Port   GPIOB      /* PB5, pin 41, U19 red */
#define ERROR_Pin         GPIO_PIN_5

/* ---- UART3 to U4 (XH2.54 3-pin) ----------------------------------------- */
#define UART_TX_GPIO_Port GPIOB      /* PB10, pin 21, USART3_TX */
#define UART_TX_Pin       GPIO_PIN_10
#define UART_RX_GPIO_Port GPIOB      /* PB11, pin 22, USART3_RX */
#define UART_RX_Pin       GPIO_PIN_11

/* ---- CAN to U14 SN65HVD230 ---------------------------------------------- */
#define CAN_RX_GPIO_Port  GPIOA      /* PA11, pin 32, U14 R */
#define CAN_RX_Pin        GPIO_PIN_11
#define CAN_TX_GPIO_Port  GPIOA      /* PA12, pin 33, U14 D */
#define CAN_TX_Pin        GPIO_PIN_12

/* ---- PPM inputs to U17 (TIM1_CH1N/CH2N pins: output-only, so EXTI) ------ */
#define PPM1_GPIO_Port    GPIOB      /* PB13, pin 26, U17-2 */
#define PPM1_Pin          GPIO_PIN_13
#define PPM2_GPIO_Port    GPIOB      /* PB14, pin 27, U17-1 */
#define PPM2_Pin          GPIO_PIN_14

/* ---- SWD to H1 (JTAG disabled, PA15/PB3/PB4 unused) --------------------- */
#define SWDIO_Pin         GPIO_PIN_13   /* PA13, pin 34 */
#define SWCLK_Pin         GPIO_PIN_14   /* PA14, pin 37 */

/* ---- Pins left as analog inputs (unused) -------------------------------- */
/* Common to both configs: PA10, PA15, PB3, PB4, PB9, PB12, PC15.            */
#define UNUSED_GPIOA_COMMON (RS485_RX_Pin | GPIO_PIN_15)
#define UNUSED_GPIOB_COMMON (GPIO_PIN_3 | GPIO_PIN_4 | ENC_Z2_Pin | RS485_DE_RE_Pin)
#define UNUSED_GPIOC_COMMON (GPIO_PIN_15)
#if ENC1_CONFIG == ENC1_ON_TIM1
#define UNUSED_GPIOA_ENC1   (ENC_A1_Pin | ENC_B1_Pin)           /* PA6, PA7 */
#elif ENC1_INDEX_ENABLE
#define UNUSED_GPIOA_ENC1   (RS485_TX_Pin)                      /* PA9 */
#else
#define UNUSED_GPIOA_ENC1   (RS485_TX_Pin | ENC_Z1_Pin)         /* PA9, PA8 */
#endif

#endif /* PINS_H */
