/* Sensorless speed controller. foc_step() is called from the ADC interrupt once
 * per PWM period; everything it touches is in this struct, so the same code runs
 * in firmware and in the host simulation. */
#ifndef FOC_H
#define FOC_H

#include "foc_math.h"
#include "observer.h"

typedef enum { FOC_IDLE, FOC_ALIGN, FOC_OPENLOOP, FOC_RUN, FOC_FAULT } foc_state_t;

typedef enum { FAULT_NONE, FAULT_OVERCURRENT, FAULT_UNDERVOLTAGE, FAULT_STARTUP } foc_fault_t;

typedef struct {
    float rs, ls, flux;     /* ohm, henry, weber (phase values) */
    int pole_pairs;
    float pwm_hz;           /* control loop rate = PWM rate */
    float i_max;            /* current limit for the speed loop, A */
    float i_trip;           /* hardware-like overcurrent trip, A */
    float vbus_min;         /* undervoltage lockout, V */
    float current_bw_hz;    /* current loop bandwidth */
    float speed_kp, speed_ki;
    float align_a, align_s; /* alignment current and time */
    float align_ramp_s;     /* current ramp-in at the start of alignment, 0 = step */
    float ramp_a, ramp_s, ramp_rpm; /* I/f startup: current, duration, target speed */
    float damp_s;           /* startup damping: forced-angle shift per unit slip, s */
    float accel_rpm_s;      /* speed reference slew limit in closed loop */
} foc_config_t;

typedef struct {
    foc_config_t cfg;
    foc_state_t state;
    foc_fault_t fault;
    float dt;

    pi_t pi_d, pi_q, pi_speed;
    observer_t obs;

    float theta;         /* angle used for Park transforms */
    float theta_ol;      /* open-loop angle during startup */
    float omega_ol;
    float t_state;       /* time in current state */
    int speed_div;

    float speed_ref_rpm; /* requested speed */
    float speed_cmd_rpm; /* slew-limited reference fed to the speed loop */
    float id_ref, iq_ref;

    /* last values, exported for telemetry */
    dq_t i_dq, v_dq;
    ab_t v_ab_prev;
    float speed_rpm;
} foc_t;

void foc_init(foc_t *f, const foc_config_t *cfg);
void foc_start(foc_t *f, float speed_rpm);
void foc_stop(foc_t *f);
void foc_set_speed(foc_t *f, float speed_rpm);

/* One control period. ia, ib in amperes, vbus in volts. Writes duty cycles in
 * [0, 1]; returns 0 when the bridge must be disabled (idle or fault). */
int foc_step(foc_t *f, float ia, float ib, float vbus, abc_t *duty);

#endif
