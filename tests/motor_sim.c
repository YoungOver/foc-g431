#include "motor_sim.h"
#include <math.h>

typedef struct { double id, iq, wm, th; } st_t;

static st_t deriv(const motor_t *m, st_t s, double va, double vb)
{
    double c = cos(s.th), sn = sin(s.th);
    double vd = c * va + sn * vb, vq = -sn * va + c * vb;
    double we = m->p * s.wm;
    st_t d;
    d.id = (vd - m->rs * s.id + we * m->lq * s.iq) / m->ld;
    d.iq = (vq - m->rs * s.iq - we * (m->ld * s.id + m->flux)) / m->lq;
    double te = 1.5 * m->p * (m->flux * s.iq + (m->ld - m->lq) * s.id * s.iq);
    /* static friction style load: opposes motion, never drives the rotor backwards */
    double tl = s.wm > 1e-3 ? m->load : (s.wm < -1e-3 ? -m->load : 0.0);
    d.wm = (te - m->b * s.wm - tl) / m->j;
    d.th = we;
    return d;
}

static st_t add(st_t a, st_t b, double k)
{
    return (st_t){a.id + k * b.id, a.iq + k * b.iq, a.wm + k * b.wm, a.th + k * b.th};
}

void motor_step(motor_t *m, abc_t duty, float vbus, float dt, int substeps)
{
    double mean = (duty.a + duty.b + duty.c) / 3.0;
    double va = (duty.a - mean) * vbus, vb = (duty.b - mean) * vbus, vc = (duty.c - mean) * vbus;
    double valpha = (2.0 * va - vb - vc) / 3.0;
    double vbeta = (vb - vc) / sqrt(3.0);
    double h = (double)dt / substeps;
    st_t s = {m->id, m->iq, m->wm, m->theta};
    for (int i = 0; i < substeps; i++) {
        st_t k1 = deriv(m, s, valpha, vbeta);
        st_t k2 = deriv(m, add(s, k1, h / 2), valpha, vbeta);
        st_t k3 = deriv(m, add(s, k2, h / 2), valpha, vbeta);
        st_t k4 = deriv(m, add(s, k3, h), valpha, vbeta);
        s.id += h / 6 * (k1.id + 2 * k2.id + 2 * k3.id + k4.id);
        s.iq += h / 6 * (k1.iq + 2 * k2.iq + 2 * k3.iq + k4.iq);
        s.wm += h / 6 * (k1.wm + 2 * k2.wm + 2 * k3.wm + k4.wm);
        s.th += h / 6 * (k1.th + 2 * k2.th + 2 * k3.th + k4.th);
    }
    m->id = s.id;
    m->iq = s.iq;
    m->wm = s.wm;
    m->theta = fmod(s.th, 2 * M_PI);
    if (m->theta < 0) m->theta += 2 * M_PI;
}

void motor_currents(const motor_t *m, float *ia, float *ib)
{
    double c = cos(m->theta), s = sin(m->theta);
    double ial = c * m->id - s * m->iq, ibe = s * m->id + c * m->iq;
    *ia = (float)ial;
    *ib = (float)(-0.5 * ial + 0.5 * sqrt(3.0) * ibe);
}

float motor_rpm(const motor_t *m) { return (float)(m->wm * 60.0 / (2 * M_PI)); }
