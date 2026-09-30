#include "foc_math.h"

#define LUT_N 256

/* One full period plus a guard sample, so interpolation never wraps. Storing a
 * quadrant would save 800 bytes of flash at the cost of branches on every call. */
static const float sin_lut[LUT_N + 1] = {
#include "sin_lut.inc"
};

float foc_wrap(float theta)
{
    theta -= FOC_2PI * (float)(int32_t)(theta * (1.0f / FOC_2PI));
    if (theta < 0.0f) theta += FOC_2PI;
    if (theta >= FOC_2PI) theta -= FOC_2PI;
    return theta;
}

static float lut(float x) /* x in table units, [0, LUT_N) */
{
    int32_t i = (int32_t)x;
    float f = x - (float)i;
    return sin_lut[i] + f * (sin_lut[i + 1] - sin_lut[i]);
}

void foc_sincos(float theta, float *s, float *c)
{
    const float k = (float)LUT_N / FOC_2PI;
    float x = foc_wrap(theta) * k;
    float xc = x + (float)(LUT_N / 4);
    if (xc >= (float)LUT_N) xc -= (float)LUT_N;
    *s = lut(x);
    *c = lut(xc);
}

ab_t foc_clarke(float ia, float ib)
{
    ab_t r = { ia, FOC_1_SQRT3 * (ia + 2.0f * ib) };
    return r;
}

dq_t foc_park(ab_t x, float s, float c)
{
    dq_t r = { c * x.alpha + s * x.beta, -s * x.alpha + c * x.beta };
    return r;
}

ab_t foc_ipark(dq_t x, float s, float c)
{
    ab_t r = { c * x.d - s * x.q, s * x.d + c * x.q };
    return r;
}

int foc_svpwm(ab_t v, float vbus, abc_t *duty)
{
    int clamped = 0;
    float vmax = vbus * FOC_1_SQRT3;
    float mag2 = v.alpha * v.alpha + v.beta * v.beta;
    if (mag2 > vmax * vmax) {
        /* scale down, keep the angle: over-modulation distorts current */
        float k = vmax / __builtin_sqrtf(mag2);
        v.alpha *= k;
        v.beta *= k;
        clamped = 1;
    }
    /* inverse Clarke to phase voltages */
    float va = v.alpha;
    float vb = -0.5f * v.alpha + 0.5f * FOC_SQRT3 * v.beta;
    float vc = -0.5f * v.alpha - 0.5f * FOC_SQRT3 * v.beta;
    /* min-max injection centres the three phases: same result as sector-based SVPWM */
    float mx = va > vb ? (va > vc ? va : vc) : (vb > vc ? vb : vc);
    float mn = va < vb ? (va < vc ? va : vc) : (vb < vc ? vb : vc);
    float off = -0.5f * (mx + mn);
    float inv = 1.0f / vbus;
    duty->a = 0.5f + (va + off) * inv;
    duty->b = 0.5f + (vb + off) * inv;
    duty->c = 0.5f + (vc + off) * inv;
    return clamped;
}

float pi_step(pi_t *p, float err)
{
    float out = p->kp * err + p->integ + p->ki * err;
    if (out > p->out_max) {
        if (err < 0.0f) p->integ += p->ki * err;
        return p->out_max;
    }
    if (out < p->out_min) {
        if (err > 0.0f) p->integ += p->ki * err;
        return p->out_min;
    }
    p->integ += p->ki * err;
    return out;
}

void pi_reset(pi_t *p) { p->integ = 0.0f; }
