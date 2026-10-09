#ifndef PID_H
#define PID_H

typedef struct {
    float kp, ki, kd;
    float integ;
    float d_filt;
} pid_t;

void pid_init(pid_t *p, float kp, float ki, float kd);
void pid_reset(pid_t *p, float integ);

/* err = setpoint - measurement; dmeas = change in measurement since the last
 * call (derivative on measurement, so set-point steps don't kick).
 * Output is clamped to [lo, hi] (which may be a narrow slew-rate window);
 * the integrator stops while the output is pinned against that window in
 * the direction of the error, and is always kept within +/-ilim. */
float pid_update(pid_t *p, float err, float dmeas, float dt, float lo, float hi, float ilim);

#endif
