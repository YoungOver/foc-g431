/* Board and motor configuration. Values marked "measure" must be checked on
 * the actual board and motor before the first spin: wrong current gain or
 * phase order is the classic way to burn a bridge. */
#ifndef BOARD_H
#define BOARD_H

#define SYSCLK_HZ   170000000u
#define PWM_HZ      20000u
#define PWM_ARR     (SYSCLK_HZ / (2u * PWM_HZ)) /* centre-aligned: counts up and down */
#define DEADTIME_NS 400u

/* Current sensing: shunt through the internal op-amp in PGA mode.
 * amps per ADC count = VREF / 4096 / (R_shunt * gain). measure */
#define VREF        3.3f
#define R_SHUNT     0.003f
#define PGA_GAIN    16.0f
#define AMPS_PER_LSB (VREF / 4096.0f / (R_SHUNT * PGA_GAIN))

/* Bus voltage divider on PA0. measure */
#define VBUS_PER_LSB (VREF / 4096.0f * 10.3f)

/* ADC channels of the op-amp outputs (RM0440, internal connections) */
#define ADC1_CH_OPAMP1 13
#define ADC2_CH_OPAMP2 16
#define ADC1_CH_VBUS   1

/* Injected trigger: TIM1_TRGO2 (RM0440, ADC external trigger table). measure on
 * other G4 parts: the mapping differs between ADC groups */
#define ADC_JEXTSEL_TIM1_TRGO2 8u

#define MOTOR_CONFIG                                                             \
    {                                                                            \
        .rs = 0.25f, .ls = 120e-6f, .flux = 0.0045f, .pole_pairs = 7,            \
        .pwm_hz = (float)PWM_HZ, .i_max = 12, .i_trip = 25, .vbus_min = 10,      \
        .current_bw_hz = 1500, .speed_kp = 0.05f, .speed_ki = 1.0f,              \
        .align_a = 4, .align_s = 0.15f, .align_ramp_s = 0.075f,                  \
        .ramp_a = 5, .ramp_s = 0.4f, .ramp_rpm = 600,                            \
        .damp_s = 0.002f, .accel_rpm_s = 10000,                                  \
    }

#endif
