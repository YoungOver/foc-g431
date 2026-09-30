/* Sensorless rotor angle estimation for a surface PMSM:
 * nonlinear flux observer (Ortega et al., 2011) followed by a PLL.
 * The observer needs only phase resistance, inductance and flux linkage. */
#ifndef OBSERVER_H
#define OBSERVER_H

#include "foc_math.h"

typedef struct {
    /* motor parameters */
    float rs, ls, flux;
    /* observer gain, typically 1e3 / flux^2 order of magnitude */
    float gamma;
    /* PLL gains in rad/s and (rad/s)^2 */
    float pll_kp, pll_ki;

    /* state */
    float x_alpha, x_beta; /* integrated stator flux estimate */
    float theta;           /* electrical angle from the PLL */
    float omega;           /* electrical speed from the PLL, rad/s */
    float pll_integ;
} observer_t;

void observer_init(observer_t *o, float rs, float ls, float flux, float gamma, float pll_bw_hz);

/* One update with the voltage actually applied in the previous period and the
 * current just measured, both in the alpha-beta frame. dt in seconds. */
void observer_update(observer_t *o, ab_t v, ab_t i, float dt);

#endif
