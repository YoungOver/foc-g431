/* Host tests: math primitives, then the full controller closed around the motor
 * model. Run `./tests --csv run.csv` to dump the startup trace for plotting. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../core/foc.h"
#include "motor_sim.h"

static int failures, checks;
#define CHECK(cond, ...)                                  \
    do {                                                  \
        checks++;                                         \
        if (!(cond)) {                                    \
            failures++;                                   \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);   \
            printf(__VA_ARGS__);                          \
            printf("\n");                                 \
        }                                                 \
    } while (0)

static double frand(void) { return rand() / (double)RAND_MAX; }

static void test_sincos(void)
{
    double worst = 0;
    for (int i = -20000; i <= 20000; i++) {
        float th = i * 0.00157f, s, c;
        foc_sincos(th, &s, &c);
        double e = fmax(fabs(s - sin(th)), fabs(c - cos(th)));
        if (e > worst) worst = e;
    }
    CHECK(worst < 1e-4, "sincos max error %.2e", worst);
}

static void test_transforms(void)
{
    for (int k = 0; k < 1000; k++) {
        float th = (float)(frand() * 2 * M_PI), s, c;
        foc_sincos(th, &s, &c);
        ab_t x = {(float)(frand() * 20 - 10), (float)(frand() * 20 - 10)};
        ab_t y = foc_ipark(foc_park(x, s, c), s, c);
        /* table sin/cos satisfy s^2 + c^2 = 1 only to 1.5e-4, so the error scales with |x| */
        double tol = 3e-4 * (1 + hypot(x.alpha, x.beta));
        CHECK(fabs(x.alpha - y.alpha) < tol && fabs(x.beta - y.beta) < tol, "park round trip");
    }
    /* balanced three-phase set of amplitude 1 maps to a unit vector */
    ab_t v = foc_clarke(1.0f, -0.5f);
    CHECK(fabs(v.alpha - 1) < 1e-6 && fabs(v.beta) < 1e-6, "clarke");
}

static void test_svpwm(void)
{
    const float vbus = 24;
    for (int k = 0; k < 2000; k++) {
        double r = frand() * vbus * FOC_1_SQRT3 * 0.999, a = frand() * 2 * M_PI;
        ab_t v = {(float)(r * cos(a)), (float)(r * sin(a))};
        abc_t d;
        int clamped = foc_svpwm(v, vbus, &d);
        CHECK(!clamped, "linear range clamped");
        CHECK(d.a >= -1e-6 && d.a <= 1 + 1e-6 && d.b >= -1e-6 && d.b <= 1 + 1e-6 && d.c >= -1e-6 && d.c <= 1 + 1e-6, "duty out of range");
        /* line-to-line voltages must equal the requested vector's */
        double vab = (d.a - d.b) * vbus, want = 1.5 * v.alpha - 0.5 * sqrt(3.0) * v.beta;
        CHECK(fabs(vab - want) < 1e-3, "vab %f want %f", vab, want);
    }
    abc_t d;
    CHECK(foc_svpwm((ab_t){100, 0}, vbus, &d) == 1, "over-range vector not clamped");
}

static void test_pi_antiwindup(void)
{
    pi_t p = {.kp = 1, .ki = 0.1f, .out_min = -1, .out_max = 1};
    for (int i = 0; i < 1000; i++) pi_step(&p, 10);
    CHECK(p.integ < 1.5f, "integrator wound up to %f", p.integ);
    /* after saturation the output must leave the limit within a few steps */
    int n = 0;
    while (pi_step(&p, -0.5f) >= 1.0f && n < 100) n++;
    CHECK(n < 10, "recovery took %d steps", n);
}

#ifndef DAMP
#define DAMP 0.002f
#endif
#ifndef ACCEL
#define ACCEL 10000.0f
#endif
#ifndef ALIGN_RAMP
#define ALIGN_RAMP 0.075f
#endif

static const foc_config_t cfg = {
    .rs = 0.25f, .ls = 120e-6f, .flux = 0.0045f, .pole_pairs = 7,
    .pwm_hz = 20000, .i_max = 12, .i_trip = 25, .vbus_min = 10,
    .current_bw_hz = 1500, .speed_kp = 0.05f, .speed_ki = 1.0f,
    .align_a = 4, .align_s = 0.15f, .align_ramp_s = ALIGN_RAMP,
    .ramp_a = 5, .ramp_s = 0.4f, .ramp_rpm = 600,
    .damp_s = DAMP, .accel_rpm_s = ACCEL,
};

static motor_t new_motor(void)
{
    return (motor_t){.rs = 0.25f, .ld = 120e-6f, .lq = 120e-6f, .flux = 0.0045f, .p = 7, .j = 2e-5f, .b = 1e-5f, .theta = 1.1};
}

static void test_current_step(void)
{
    /* locked rotor: run only the current loop by forcing the ALIGN state */
    foc_config_t step_cfg = cfg;
    step_cfg.align_ramp_s = 0; /* a true step */
    foc_t f;
    foc_init(&f, &step_cfg);
    motor_t m = new_motor();
    m.j = 1e6f;
    m.theta = 0;
    foc_start(&f, 0);
    abc_t d;
    float peak = 0, t95 = -1;
    for (int k = 0; k < 400; k++) { /* 20 ms */
        float ia, ib;
        motor_currents(&m, &ia, &ib);
        foc_step(&f, ia, ib, 24, &d);
        motor_step(&m, d, 24, f.dt, 4);
        if (m.id > peak) peak = (float)m.id;
        if (t95 < 0 && m.id > 0.95 * cfg.align_a) t95 = k * f.dt;
    }
    CHECK(t95 > 0 && t95 < 0.5e-3f, "id reached 95%% after %.3f ms", t95 * 1e3);
    CHECK(peak < cfg.align_a * 1.05f, "overshoot %.1f%%", (peak / cfg.align_a - 1) * 100);
    CHECK(fabs(m.id - cfg.align_a) < 0.02, "steady id %.3f", m.id);
}

typedef struct {
    float settle_s, speed_err, obs_err_deg, load_dip, iq_peak;
    float overshoot;     /* peak above target before the load step, fraction */
    float swing_rpm;     /* worst rotor speed deviation from the forced speed during I/f */
    foc_state_t final_state;
} run_t;

/* Real motors differ from their datasheet: the plant can be scaled against the
 * parameters the controller was configured with. */
static float mis_rs = 1, mis_ls = 1, mis_flux = 1;

static run_t run_sensorless(const char *csv)
{
    FILE *out = csv ? fopen(csv, "w") : NULL;
    if (out) fprintf(out, "t,rpm_true,rpm_est,iq,iq_ref,theta_err_deg,state\n");
    foc_t f;
    foc_init(&f, &cfg);
    motor_t m = new_motor();
    m.rs *= mis_rs;
    m.ld *= mis_ls;
    m.lq *= mis_ls;
    m.flux *= mis_flux;
    const float target = 3000;
    foc_start(&f, target);
    run_t r = {.settle_s = -1};
    abc_t d;
    double err_acc = 0;
    int err_n = 0;
    float min_after_load = 1e9f;
    for (int k = 0; k < 20000 * 3; k++) { /* 3 s */
        float t = k * f.dt;
        if (k == 20000 * 2) m.load = 0.03f; /* load step at t = 2 s */
        float ia, ib;
        motor_currents(&m, &ia, &ib);
        ia += (float)((frand() - 0.5) * 0.04); /* ADC noise, +-20 mA */
        ib += (float)((frand() - 0.5) * 0.04);
        foc_step(&f, ia, ib, 24, &d);
        motor_step(&m, d, 24, f.dt, 4);
        float rpm = motor_rpm(&m);
        if (f.state == FOC_RUN && r.settle_s < 0 && fabs(rpm - target) < 0.02f * target) r.settle_s = t;
        if (f.state == FOC_RUN && t > 1.5f && t < 2.0f) {
            double e = fmod(f.obs.theta - m.theta + 3 * M_PI, 2 * M_PI) - M_PI;
            err_acc += e * e;
            err_n++;
        }
        if (t > 2.0f && rpm < min_after_load) min_after_load = rpm;
        if (f.state == FOC_RUN && t < 2.0f && (rpm - target) / target > r.overshoot) r.overshoot = (rpm - target) / target;
        if (f.state == FOC_OPENLOOP && f.t_state > 0.1f) {
            float forced = f.omega_ol / cfg.pole_pairs * (float)(60 / (2 * M_PI));
            if (fabsf(rpm - forced) > r.swing_rpm) r.swing_rpm = fabsf(rpm - forced);
        }
        if (fabs(f.iq_ref) > r.iq_peak) r.iq_peak = fabs(f.iq_ref);
        if (out && k % 20 == 0) {
            double e = fmod(f.obs.theta - m.theta + 3 * M_PI, 2 * M_PI) - M_PI;
            fprintf(out, "%.4f,%.1f,%.1f,%.3f,%.3f,%.2f,%d\n", t, rpm, f.speed_rpm, m.iq, f.iq_ref, e * 180 / M_PI, f.state);
        }
    }
    r.speed_err = fabs(motor_rpm(&m) - target) / target;
    r.obs_err_deg = err_n ? (float)(sqrt(err_acc / err_n) * 180 / M_PI) : 999;
    r.load_dip = (target - min_after_load) / target;
    r.final_state = f.state;
    if (out) fclose(out);
    return r;
}

static void test_sensorless_speed_loop(const char *csv)
{
    run_t r = run_sensorless(csv);
    printf("  sensorless: settled %.2f s, overshoot %.1f%%, startup swing %.1f rpm, final error %.2f%%, observer rms error %.2f deg, load dip %.1f%%, peak iq_ref %.1f A\n",
           r.settle_s, r.overshoot * 100, r.swing_rpm, r.speed_err * 100, r.obs_err_deg, r.load_dip * 100, r.iq_peak);
    CHECK(r.final_state == FOC_RUN, "controller ended in state %d", r.final_state);
    CHECK(r.settle_s > 0 && r.settle_s < 1.5f, "settle time %.2f s", r.settle_s);
    CHECK(r.speed_err < 0.02f, "final speed error %.2f%%", r.speed_err * 100);
    CHECK(r.obs_err_deg < 5, "observer angle error %.2f deg", r.obs_err_deg);
    CHECK(r.load_dip < 0.15f, "speed dip under load %.1f%%", r.load_dip * 100);
    CHECK(r.overshoot < 0.03f, "overshoot %.1f%%", r.overshoot * 100);
    CHECK(r.swing_rpm < 100, "startup swing %.0f rpm", r.swing_rpm);
}

static void test_parameter_mismatch(void)
{
    /* hot winding (+25% R), saturated iron (-15% L), stronger magnets (+5% flux) */
    mis_rs = 1.25f, mis_ls = 0.85f, mis_flux = 1.05f;
    run_t r = run_sensorless(NULL);
    mis_rs = mis_ls = mis_flux = 1;
    printf("  mismatch:   settled %.2f s, overshoot %.1f%%, startup swing %.1f rpm, final error %.2f%%, observer rms error %.2f deg, load dip %.1f%%\n",
           r.settle_s, r.overshoot * 100, r.swing_rpm, r.speed_err * 100, r.obs_err_deg, r.load_dip * 100);
    CHECK(r.final_state == FOC_RUN, "controller ended in state %d", r.final_state);
    CHECK(r.speed_err < 0.02f, "final speed error %.2f%%", r.speed_err * 100);
    CHECK(r.obs_err_deg < 10, "observer angle error %.2f deg", r.obs_err_deg);
}

static void test_overcurrent_trip(void)
{
    foc_t f;
    foc_init(&f, &cfg);
    foc_start(&f, 1000);
    abc_t d;
    int on = foc_step(&f, 30, -15, 24, &d);
    CHECK(!on && f.state == FOC_FAULT && f.fault == FAULT_OVERCURRENT, "no trip at 30 A");
    CHECK(d.a == 0.5f && d.b == 0.5f && d.c == 0.5f, "outputs not neutral after trip");
    foc_start(&f, 1000); /* a fault latches until the application clears it */
    CHECK(f.state == FOC_FAULT, "fault did not latch");
}

int main(int argc, char **argv)
{
    const char *csv = argc > 2 && strcmp(argv[1], "--csv") == 0 ? argv[2] : NULL;
    srand(7);
    test_sincos();
    test_transforms();
    test_svpwm();
    test_pi_antiwindup();
    test_current_step();
    test_overcurrent_trip();
    test_sensorless_speed_loop(csv);
    test_parameter_mismatch();
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
