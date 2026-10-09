#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#define NUM_MOTORS 2

typedef enum {
    MODE_OFF = 0,   /* INH low, bridge high-impedance */
    MODE_DUTY,      /* open-loop duty, target in [-1, 1] */
    MODE_VEL,       /* velocity PID, target in counts/s */
    MODE_POS,       /* position PID -> velocity PID, target in counts */
} motor_mode_t;

typedef enum { LOOP_VEL = 0, LOOP_POS } loop_id_t;

/* Latched fault bits (cleared only by an explicit clear command). */
#define FAULT_OVERCURRENT  (1u << 0)   /* IS above CURRENT_TRIP_A for CURRENT_TRIP_MS */
#define FAULT_STALL        (1u << 1)   /* high effort, no encoder movement */
#define FAULT_LIMIT        (1u << 2)   /* limit switch pressed */
#define FAULT_DRIVER       (1u << 3)   /* BTN8962 signalled a fault on IS */
#define FAULT_ESTOP        (1u << 4)   /* emergency stop requested (CAN Estop) */

typedef struct {
    motor_mode_t mode;
    uint8_t faults;
    int32_t pos;            /* counts */
    float vel;              /* counts/s */
    float current;          /* A, signed (positive = positive duty direction) */
    bool current_valid;     /* false when USE_CURRENT_SENSE = 0 */
    float duty;             /* applied duty, [-1, 1] */
    float target;           /* duty, counts/s or counts depending on mode */
    bool limit;             /* debounced limit switch state */
} motor_status_t;

void motor_init(void);

/* Change mode. Refused (false) while a fault is latched, except MODE_OFF. */
bool motor_set_mode(uint8_t m, motor_mode_t mode);
/* Set the target for the current mode. False if the motor is OFF. */
bool motor_set_target(uint8_t m, float target);
/* Gains are stored in the parameter table (control/params.h). False if out of range. */
bool motor_set_gains(uint8_t m, loop_id_t loop, float kp, float ki, float kd);
void motor_get_gains(uint8_t m, loop_id_t loop, float *kp, float *ki, float *kd);
/* Clear latched faults. Returns true if the limit switch is still pressed
 * (the limit fault is edge-triggered, so the motor can be driven off it). */
bool motor_clear_fault(uint8_t m);
/* Redefine the current position as 0 (position target shifts with it). */
void motor_zero(uint8_t m);
void motor_stop_all(void);
/* Latch FAULT_ESTOP on one motor: INH off until motor_clear_fault(). */
void motor_estop(uint8_t m);
void motor_get_status(uint8_t m, motor_status_t *s);
bool motor_any_enabled(void);
bool motor_any_fault(void);
uint32_t motor_ticks(void);

/* The control loop itself (declared in drivers/pwm.h): control_tick(). */

#endif
