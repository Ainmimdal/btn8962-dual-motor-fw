#include "control/pid.h"
#include "config.h"

void pid_init(pid_t *p, float kp, float ki, float kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    pid_reset(p, 0.0f);
}

void pid_reset(pid_t *p, float integ)
{
    p->integ = integ;
    p->d_filt = 0.0f;
}

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

float pid_update(pid_t *p, float err, float dmeas, float dt, float lo, float hi, float ilim)
{
    float pterm = p->kp * err;

    p->d_filt += PID_D_LPF_ALPHA * ((-p->kd * dmeas / dt) - p->d_filt);

    float integ = p->integ + p->ki * err * dt;
    float out = pterm + integ + p->d_filt;
    int wind_up = (out > hi && err > 0.0f) || (out < lo && err < 0.0f);
    if (!wind_up)
        p->integ = clampf(integ, -ilim, ilim);

    return clampf(pterm + p->integ + p->d_filt, lo, hi);
}
