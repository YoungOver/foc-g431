/* Averaged PMSM model in the rotor frame, integrated with RK4.
 * The inverter is ideal: phase voltage = (duty - mean duty) * vbus. */
#ifndef MOTOR_SIM_H
#define MOTOR_SIM_H

#include "../core/foc_math.h"

typedef struct {
    float rs, ld, lq, flux;
    int p;              /* pole pairs */
    float j, b;         /* inertia kg*m^2, viscous friction N*m*s */
    float load;         /* load torque, N*m */
    /* state */
    double id, iq, wm, theta; /* theta: electrical angle */
} motor_t;

void motor_step(motor_t *m, abc_t duty, float vbus, float dt, int substeps);
void motor_currents(const motor_t *m, float *ia, float *ib);
float motor_rpm(const motor_t *m);

#endif
