#include "observer.h"

void observer_init(observer_t *o, float rs, float ls, float flux, float gamma, float pll_bw_hz)
{
    *o = (observer_t){0};
    o->rs = rs;
    o->ls = ls;
    o->flux = flux;
    o->gamma = gamma;
    /* critically damped second-order PLL at the requested bandwidth */
    float wn = FOC_2PI * pll_bw_hz;
    o->pll_kp = 2.0f * wn;
    o->pll_ki = wn * wn;
    /* start the flux estimate on the circle of radius `flux` so the
     * correction term is zero before the first real sample */
    o->x_alpha = flux;
    o->x_beta = 0.0f;
}

void observer_update(observer_t *o, ab_t v, ab_t i, float dt)
{
    /* rotor flux = stator flux - L*i must have magnitude `flux`; the error in
     * its squared magnitude drives the estimate back onto that circle */
    float ra = o->x_alpha - o->ls * i.alpha;
    float rb = o->x_beta - o->ls * i.beta;
    float err = o->flux * o->flux - (ra * ra + rb * rb);

    o->x_alpha += dt * (v.alpha - o->rs * i.alpha + 0.5f * o->gamma * ra * err);
    o->x_beta += dt * (v.beta - o->rs * i.beta + 0.5f * o->gamma * rb * err);

    ra = o->x_alpha - o->ls * i.alpha;
    rb = o->x_beta - o->ls * i.beta;

    /* PLL on the rotor flux vector: phase error ~ cross product of the
     * estimated unit vector and the PLL's own angle */
    float s, c;
    foc_sincos(o->theta, &s, &c);
    float mag = __builtin_sqrtf(ra * ra + rb * rb) + 1e-9f;
    float perr = (rb * c - ra * s) / mag;

    o->pll_integ += o->pll_ki * perr * dt;
    o->omega = o->pll_kp * perr + o->pll_integ;
    o->theta = foc_wrap(o->theta + o->omega * dt);
}
