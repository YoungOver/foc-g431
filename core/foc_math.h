/* Portable FOC building blocks. No hardware access: the same code runs on the
 * STM32G431 (single-precision FPU) and in host tests against a motor model. */
#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <stdint.h>

#define FOC_PI      3.14159265358979f
#define FOC_2PI     6.28318530717959f
#define FOC_SQRT3   1.73205080756888f
#define FOC_1_SQRT3 0.57735026918963f

typedef struct { float a, b, c; } abc_t;
typedef struct { float alpha, beta; } ab_t;
typedef struct { float d, q; } dq_t;

/* sin and cos from one 256-entry table with linear interpolation:
 * max error 7.5e-5, about 40 cycles on Cortex-M4F. */
void foc_sincos(float theta, float *s, float *c);

/* Wraps any angle into [0, 2pi). */
float foc_wrap(float theta);

/* Amplitude-invariant Clarke transform; assumes ia + ib + ic = 0. */
ab_t foc_clarke(float ia, float ib);
dq_t foc_park(ab_t x, float s, float c);
ab_t foc_ipark(dq_t x, float s, float c);

/* Space vector modulation by min-max injection. Input is the stator voltage in
 * volts, output duty cycles in [0, 1]. Returns 1 if the vector was clamped to
 * the linear range (|v| <= vbus / sqrt3). */
int foc_svpwm(ab_t v, float vbus, abc_t *duty);

typedef struct {
    float kp, ki;       /* ki already multiplied by the sample period */
    float out_min, out_max;
    float integ;
} pi_t;

/* PI with conditional integration: the integrator stops growing while the
 * output is saturated in the same direction as the error (no windup). */
float pi_step(pi_t *p, float err);
void pi_reset(pi_t *p);

#endif
