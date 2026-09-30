#include "foc.h"

#define RPM_TO_RAD(p) (FOC_2PI / 60.0f * (float)(p))
#define SPEED_LOOP_DIV 20 /* speed loop runs at pwm_hz / 20, 1 kHz at 20 kHz PWM */

void foc_init(foc_t *f, const foc_config_t *cfg)
{
    *f = (foc_t){0};
    f->cfg = *cfg;
    f->dt = 1.0f / cfg->pwm_hz;

    /* Current loop: pole-zero cancellation of the R-L plant gives a first-order
     * closed loop with the requested bandwidth. */
    float wc = FOC_2PI * cfg->current_bw_hz;
    float kp = cfg->ls * wc, ki = cfg->rs * wc * f->dt;
    f->pi_d = (pi_t){.kp = kp, .ki = ki};
    f->pi_q = (pi_t){.kp = kp, .ki = ki};
    f->pi_speed = (pi_t){.kp = cfg->speed_kp, .ki = cfg->speed_ki * f->dt * SPEED_LOOP_DIV, .out_min = -cfg->i_max, .out_max = cfg->i_max};

    float gamma = 1000.0f / (cfg->flux * cfg->flux);
    observer_init(&f->obs, cfg->rs, cfg->ls, cfg->flux, gamma, 60.0f);
}

void foc_set_speed(foc_t *f, float rpm) { f->speed_ref_rpm = rpm; }

void foc_start(foc_t *f, float rpm)
{
    if (f->state != FOC_IDLE) return;
    f->speed_ref_rpm = rpm;
    f->state = FOC_ALIGN;
    f->t_state = 0.0f;
    f->theta_ol = -0.5f * FOC_PI; /* see FOC_ALIGN */
    f->omega_ol = 0.0f;
    pi_reset(&f->pi_d);
    pi_reset(&f->pi_q);
    pi_reset(&f->pi_speed);
    observer_init(&f->obs, f->cfg.rs, f->cfg.ls, f->cfg.flux, f->obs.gamma, 60.0f);
}

void foc_stop(foc_t *f)
{
    f->state = FOC_IDLE;
    f->id_ref = f->iq_ref = 0.0f;
}

/* Startup damping shift. Clamped: at a standstill the observer has no back-EMF
 * to lock onto, and a wrong estimate must not swing the vector around. */
static float damping(const foc_t *f, float slip)
{
    float a = f->cfg.damp_s * slip, lim = 0.35f; /* 20 deg electrical */
    return a > lim ? lim : a < -lim ? -lim : a;
}

static void enter(foc_t *f, foc_state_t s)
{
    f->state = s;
    f->t_state = 0.0f;
}

static int trip(foc_t *f, foc_fault_t why)
{
    f->fault = why;
    enter(f, FOC_FAULT);
    return 0;
}

int foc_step(foc_t *f, float ia, float ib, float vbus, abc_t *duty)
{
    const foc_config_t *c = &f->cfg;
    duty->a = duty->b = duty->c = 0.5f;

    float ic = -ia - ib;
    float peak = ia > 0 ? ia : -ia;
    if ((ib > 0 ? ib : -ib) > peak) peak = ib > 0 ? ib : -ib;
    if ((ic > 0 ? ic : -ic) > peak) peak = ic > 0 ? ic : -ic;
    if (f->state != FOC_IDLE && f->state != FOC_FAULT) {
        if (peak > c->i_trip) return trip(f, FAULT_OVERCURRENT);
        if (vbus < c->vbus_min) return trip(f, FAULT_UNDERVOLTAGE);
    }
    if (f->state == FOC_IDLE || f->state == FOC_FAULT) return 0;

    ab_t i_ab = foc_clarke(ia, ib);
    /* the observer integrates the voltage applied during the previous period */
    observer_update(&f->obs, f->v_ab_prev, i_ab, f->dt);
    f->t_state += f->dt;

    switch (f->state) {
    case FOC_ALIGN: {
        /* Pull the rotor to 0 rad. The current is put on the q axis of a frame
         * at -90 deg: the vector points at 0 rad, exactly where I/f will start
         * pushing, so the handover to I/f does not rotate it. It ramps in so
         * the rotor is not kicked, and the angle is nudged against the
         * estimated speed so the snap does not ring. */
        float k = c->align_ramp_s > 0.0f ? f->t_state / c->align_ramp_s : 1.0f;
        f->theta = foc_wrap(f->theta_ol + damping(f, -f->obs.omega));
        f->id_ref = 0.0f;
        f->iq_ref = c->align_a * (k < 1.0f ? k : 1.0f);
        if (f->t_state >= c->align_s) enter(f, FOC_OPENLOOP);
        break;
    }

    case FOC_OPENLOOP: {
        /* I/f: fixed current, frequency ramps. Current control alone makes the
         * rotor a spring-mass system around the forced angle with almost no
         * damping; shifting the angle by the slip between the forced and the
         * estimated speed adds the missing damping term. */
        float target = RPM_TO_RAD(c->ramp_rpm) * (float)c->pole_pairs;
        f->omega_ol = target * (f->t_state < c->ramp_s ? f->t_state / c->ramp_s : 1.0f);
        f->theta_ol = foc_wrap(f->theta_ol + f->omega_ol * f->dt);
        f->theta = foc_wrap(f->theta_ol + damping(f, f->omega_ol - f->obs.omega));
        f->id_ref = 0.0f;
        f->iq_ref = c->ramp_a;
        /* hand over once the observer tracks the forced speed within 10% */
        float e = f->obs.omega - f->omega_ol;
        if (f->t_state >= c->ramp_s && (e > 0 ? e : -e) < 0.1f * target) {
            /* bumpless: rewrite every integrator in the observer frame. The
             * speed loop starts from the torque-producing current, the current
             * loops from the voltage just applied minus the feed-forward that
             * RUN adds on top, and the reference from the speed reached. */
            float so, cs, we = f->obs.omega;
            foc_sincos(f->obs.theta, &so, &cs);
            dq_t i = foc_park(i_ab, so, cs), v = foc_park(f->v_ab_prev, so, cs);
            f->pi_speed.integ = i.q;
            f->iq_ref = i.q;
            f->pi_d.integ = v.d + we * c->ls * i.q;
            f->pi_q.integ = v.q - we * (c->ls * i.d + c->flux);
            f->speed_cmd_rpm = f->obs.omega / (float)c->pole_pairs * (60.0f / FOC_2PI);
            f->speed_div = 0;
            enter(f, FOC_RUN);
        } else if (f->t_state > c->ramp_s + 1.0f) {
            return trip(f, FAULT_STARTUP);
        }
        break;
    }

    case FOC_RUN:
        f->theta = f->obs.theta;
        if (++f->speed_div >= SPEED_LOOP_DIV) {
            f->speed_div = 0;
            float step = c->accel_rpm_s * f->dt * SPEED_LOOP_DIV;
            float d = f->speed_ref_rpm - f->speed_cmd_rpm;
            f->speed_cmd_rpm += d > step ? step : d < -step ? -step : d;
            float err = RPM_TO_RAD(f->speed_cmd_rpm) - f->obs.omega / (float)c->pole_pairs;
            f->iq_ref = pi_step(&f->pi_speed, err);
        }
        f->id_ref = 0.0f;
        break;

    default:
        return 0;
    }

    f->speed_rpm = f->obs.omega / (float)c->pole_pairs * (60.0f / FOC_2PI);

    float s, co;
    foc_sincos(f->theta, &s, &co);
    f->i_dq = foc_park(i_ab, s, co);

    /* voltage limit of the linear SVPWM range, shared between d and q (d first) */
    float vmax = vbus * FOC_1_SQRT3 * 0.95f;
    f->pi_d.out_max = vmax;
    f->pi_d.out_min = -vmax;
    f->v_dq.d = pi_step(&f->pi_d, f->id_ref - f->i_dq.d);
    float vq_max = __builtin_sqrtf(vmax * vmax - f->v_dq.d * f->v_dq.d);
    f->pi_q.out_max = vq_max;
    f->pi_q.out_min = -vq_max;
    f->v_dq.q = pi_step(&f->pi_q, f->iq_ref - f->i_dq.q);

    /* decoupling feed-forward: cancels the speed-dependent cross terms */
    float we = f->obs.omega;
    dq_t v = f->v_dq;
    if (f->state == FOC_RUN) {
        v.d -= we * c->ls * f->i_dq.q;
        v.q += we * (c->ls * f->i_dq.d + c->flux);
    }

    ab_t v_ab = foc_ipark(v, s, co);
    foc_svpwm(v_ab, vbus, duty);
    f->v_ab_prev = v_ab;
    return 1;
}
