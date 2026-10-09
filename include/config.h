/*
 * config.h - every compile-time option and tunable in one place.
 *
 * Values marked [runtime] are only the defaults of runtime parameters
 * (control/params.c): they can be changed over UART/CAN and saved to flash.
 *
 * Build flags (normally set per environment in platformio.ini):
 *   ENC1_CONFIG        ENC1_ON_TIM1 (default) or ENC1_ON_TIM3_3V3
 *   USE_CURRENT_SENSE  1 (default) = IS ADC+DMA, current limit, overcurrent fault
 *                      0           = no IS ADC, encoder-based stall check, duty cap + ramp
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ------------------------------------------------------------------------- */
/* Build options                                                             */
/* ------------------------------------------------------------------------- */
#define ENC1_ON_TIM1        1   /* TIM1 encoder on PA8 (A) / PA9 (B), RS485 not fitted */
#define ENC1_ON_TIM3_3V3    2   /* TIM3 encoder on PA6 (A) / PA7 (B), encoder on 3.3 V */

#ifndef ENC1_CONFIG
#define ENC1_CONFIG ENC1_ON_TIM1
#endif
#if (ENC1_CONFIG != ENC1_ON_TIM1) && (ENC1_CONFIG != ENC1_ON_TIM3_3V3)
#error "ENC1_CONFIG must be ENC1_ON_TIM1 or ENC1_ON_TIM3_3V3"
#endif

#ifndef USE_CURRENT_SENSE
#define USE_CURRENT_SENSE 1
#endif

/* Optional Z index for encoder 1 on PA8 (only possible in ENC1_ON_TIM3_3V3,
 * because PA8 is encoder A in ENC1_ON_TIM1). Latches the position on a rising
 * edge; reported by "GET 1". */
#ifndef ENC1_INDEX_ENABLE
#define ENC1_INDEX_ENABLE 0
#endif
#if ENC1_INDEX_ENABLE && (ENC1_CONFIG != ENC1_ON_TIM3_3V3)
#error "ENC1_INDEX_ENABLE needs ENC1_CONFIG == ENC1_ON_TIM3_3V3 (PA8 is encoder A otherwise)"
#endif

#ifndef USE_CAN
#define USE_CAN 1               /* bxCAN on PA11/PA12 */
#endif
#ifndef USE_IWDG
#define USE_IWDG 1              /* independent watchdog, ~100 ms */
#endif

#define FW_VERSION "0.3.0"

/* ------------------------------------------------------------------------- */
/* Clocks: HSI 8 MHz / 2 * 16 = 64 MHz (no HSE fitted)                       */
/* ------------------------------------------------------------------------- */
#define SYSCLK_HZ        64000000UL
#define TIM2_CLK_HZ      64000000UL   /* APB1 = 32 MHz, timer clock x2 */

/* ------------------------------------------------------------------------- */
/* PWM and control loop                                                      */
/* ------------------------------------------------------------------------- */
#define PWM_FREQ_HZ      20000UL      /* TIM2 centre-aligned PWM frequency */
#define CTRL_LOOP_HZ     1000UL       /* must divide PWM_FREQ_HZ exactly */
#define POS_LOOP_DIV     1            /* outer position loop runs every N control ticks */

#define PWM_ARR          (TIM2_CLK_HZ / (2UL * PWM_FREQ_HZ))   /* 1600 @ 20 kHz */
#define CTRL_DECIM       (PWM_FREQ_HZ / CTRL_LOOP_HZ)
#define CTRL_DT_S        (1.0f / (float)CTRL_LOOP_HZ)

/* Motor/encoder polarity: set to 1 if a motor runs backwards for positive
 * duty, or the encoder counts down when the motor turns forward. */
#define MOTOR1_INVERT    0  /* [runtime] */
#define MOTOR2_INVERT    0  /* [runtime] */
#define ENC1_INVERT      0  /* [runtime] */
#define ENC2_INVERT      0  /* [runtime] */

/* Encoder input digital filter (TIMx_CCMR1 ICxF, 0..15). 6 = fDTS/4, N=6. */
#define ENC_INPUT_FILTER 6

/* Velocity estimate: counts/s, first-order low-pass on the per-tick delta. */
#define VEL_LPF_ALPHA    0.2f

/* Duty cap and slew limit (fraction of full scale, and full-scale per second).
 * Without current sense these are the only protection, so the defaults are
 * deliberately conservative. */
#if USE_CURRENT_SENSE
#define DUTY_MAX         0.95f  /* [runtime] */
#define DUTY_RAMP_PER_S  20.0f        /* [runtime] 0 -> 100 % in 50 ms */
#else
#define DUTY_MAX         0.60f  /* [runtime] */
#define DUTY_RAMP_PER_S  2.0f         /* [runtime] 0 -> 100 % in 500 ms */
#endif

/* Outer position loop output limit (velocity set-point, counts/s). */
#define VEL_MAX_CPS      20000.0f  /* [runtime] */

/* Default PID gains. Units: velocity loop = duty per (count/s),
 * position loop = (count/s) per count. Tune on the real mechanism. */
#define VEL_KP_DEFAULT   0.0005f  /* [runtime] */
#define VEL_KI_DEFAULT   0.005f  /* [runtime] */
#define VEL_KD_DEFAULT   0.0f  /* [runtime] */
#define POS_KP_DEFAULT   5.0f  /* [runtime] */
#define POS_KI_DEFAULT   0.0f  /* [runtime] */
#define POS_KD_DEFAULT   0.0f  /* [runtime] */
#define PID_D_LPF_ALPHA  0.2f         /* low-pass on the derivative term */

/* ------------------------------------------------------------------------- */
/* Current sense (BTN8962 IS pin -> R_IS to GND -> ADC)                      */
/* ------------------------------------------------------------------------- */
/* R_IS on the schematic: R11, R14, R17, R20 = 3.3k. */
#define IS_R_SENSE_OHMS      3300.0f
/* BTN8962 load-current / sense-current ratio (typical). Check the datasheet
 * for your part; a BTN7971 would need a very different value. */
#define IS_KILIS             19500.0f
#define ADC_VREF_V           3.3f
#define ADC_FULL_SCALE       4095.0f
/* Amps of load current per ADC count (~4.76 mA/count with the values above;
 * full scale ~19.5 A). */
#define IS_AMPS_PER_COUNT    ((ADC_VREF_V / ADC_FULL_SCALE) / IS_R_SENSE_OHMS * IS_KILIS)

#define IS_AVG_SAMPLES       16       /* PWM periods averaged (<= CTRL_DECIM) */
#define IS_OFFSET_MAX_COUNTS 200      /* warn at boot if the zero offset is above this */

#define CURRENT_LIMIT_A      5.0f     /* [runtime] soft limit: duty is pulled back above this */
#define CURRENT_LIMIT_KI     20.0f    /* duty-scale reduction per (A over limit * s) */
#define CURRENT_LIMIT_RECOVER_PER_S 2.0f
#define CURRENT_TRIP_A       8.0f     /* [runtime] hard limit: latched OVERCURRENT fault */
#define CURRENT_TRIP_MS      20       /* [runtime] ...after this many consecutive ms above it */
/* A BTN8962 in fault (overtemp/overcurrent/undervoltage) drives IS to its
 * saturation level, which reads as ADC full scale with R_IS = 3.3k. */
#define IS_DRIVER_FAULT_COUNTS 4000
#define IS_DRIVER_FAULT_MS   3

/* ------------------------------------------------------------------------- */
/* Stall detection (both settings; the "effort" test differs)                */
/* ------------------------------------------------------------------------- */
#define STALL_TIMEOUT_MS     500      /* [runtime] effort high and no movement for this long -> STALL */
#define STALL_MIN_COUNTS     10       /* [runtime] movement smaller than this counts as "not moving" */
#if USE_CURRENT_SENSE
#define STALL_CURRENT_A      (0.8f * CURRENT_LIMIT_A)  /* [runtime] */
#else
#define STALL_DUTY_MIN       0.30f    /* [runtime] |duty| at or above this counts as "high duty" */
#endif

/* ------------------------------------------------------------------------- */
/* Limit switches (active low, internal pull-up, switch to GND)              */
/* ------------------------------------------------------------------------- */
#define LIMIT_DEBOUNCE_MS    5
/* LIMIT1 stops motor 1, LIMIT2 stops motor 2 (0-based motor index). */
#define LIMIT1_MOTOR         0
#define LIMIT2_MOTOR         1

/* ------------------------------------------------------------------------- */
/* PPM / RC pulse inputs (EXTI + DWT cycle counter)                          */
/* ------------------------------------------------------------------------- */
#define PPM_MIN_US           800
#define PPM_MAX_US           2200
#define PPM_TIMEOUT_MS       100

/* ------------------------------------------------------------------------- */
/* Comms                                                                     */
/* ------------------------------------------------------------------------- */
#define UART_BAUD            115200
#define UART_RX_BUF          256
#define UART_TX_BUF          2048     /* fits a full "CFG" listing */
#define CMD_LINE_MAX         96
#define TELEM_DEFAULT_MS     50       /* [runtime] 0 = off; change at runtime with "TELEM <ms>" */
#define CMD_WATCHDOG_MS      500      /* [runtime] 0 = disabled */

/* bxCAN: APB1 32 MHz, 16 tq per bit (1 + 11 + 4), sample point 75 %, SJW 4.
 * The prescaler is 2000 / kbit/s: 16 @ 125k, 8 @ 250k, 4 @ 500k, 2 @ 1M.
 * The wide SJW is for clock tolerance, because the HSI is the only clock
 * source; above 500k the HSI is marginal (see README). */
#define CAN_SJW              CAN_SJW_4TQ
#define CAN_BS1              CAN_BS1_11TQ
#define CAN_BS2              CAN_BS2_4TQ
#define CAN_BAUD_KBPS        125      /* [runtime] 125/250/500/1000, applied at boot */

/* CAN protocol: ODrive "CAN Simple" framing, so the board can share a bus
 * with ODrive 0.5.x axes. Each motor is one node: ID = node_id << 5 | cmd.
 * The node IDs must differ from every ODrive axis on the bus (ODrive's
 * default is 0, so pick something else on the ODrive side or here). */
#define CAN_M1_NODE_ID       40       /* [runtime] 0..62 */
#define CAN_M2_NODE_ID       41       /* [runtime] 0..62 */
#define CAN_HEARTBEAT_MS     100      /* [runtime] cyclic heartbeat (cmd 0x01), 0 = off */
#define CAN_ENCODER_MS       20       /* [runtime] cyclic encoder estimates (cmd 0x09), 0 = off */
#define CAN_TELEM_MS         100      /* [runtime] cyclic vendor status (cmd 0x1F), 0 = off */
#define CAN_ODRIVE_VER       56       /* [runtime] 56 = 0.5.6 heartbeat/error layout, 51 = 0.5.1 */

/* Encoder counts per output "turn", used to convert to ODrive units (turns,
 * turns/s). The default of 1 makes ODrive "turns" equal to raw counts. */
#define ENC1_CPR             1        /* [runtime] */
#define ENC2_CPR             1        /* [runtime] */

/* ------------------------------------------------------------------------- */
/* Compile-time checks                                                       */
/* ------------------------------------------------------------------------- */
_Static_assert((PWM_FREQ_HZ % CTRL_LOOP_HZ) == 0, "CTRL_LOOP_HZ must divide PWM_FREQ_HZ");
_Static_assert(PWM_ARR >= 200 && PWM_ARR <= 65535, "PWM_FREQ_HZ out of range");
_Static_assert(IS_AVG_SAMPLES >= 1 && IS_AVG_SAMPLES <= CTRL_DECIM,
               "IS_AVG_SAMPLES must be 1..PWM periods per control tick");

#endif /* CONFIG_H */
