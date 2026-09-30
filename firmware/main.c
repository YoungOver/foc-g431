/* B-G431B-ESC1 sensorless FOC firmware.
 *
 * Timing: TIM1 runs centre-aligned at 20 kHz. Its OC4 fires at the top of the
 * count, where all low-side switches conduct, and triggers injected conversions
 * of the phase currents on ADC1 and ADC2 at the same instant. The end-of-
 * conversion interrupt runs the whole control step and writes the next duty
 * cycles; the preload registers apply them at the following update. */
#include <stdint.h>
#include "regs.h"
#include "board.h"
#include "../core/foc.h"
#include "telemetry.h"

#define CCM __attribute__((section(".ccm")))

static CCM foc_t foc;
static CCM float offset_a, offset_b;
static volatile uint32_t ms;
static volatile uint32_t loop_cycles, loop_cycles_max;

static void clock_init(void)
{
    RCC_APB1ENR1 |= 1u << 28;          /* PWR */
    PWR_CR5 &= ~(1u << 8);             /* range 1 boost mode, required above 150 MHz */
    FLASH_ACR = (FLASH_ACR & ~0xFu) | 4u | (1u << 8) | (1u << 9) | (1u << 10); /* 4 WS, prefetch, I/D cache */
    while ((FLASH_ACR & 0xFu) != 4u) {
    }
    RCC_CR |= 1u << 8;                 /* HSI16 on */
    while (!(RCC_CR & (1u << 10))) {
    }
    /* PLL: HSI16 / 4 * 85 / 2 = 170 MHz */
    RCC_PLLCFGR = 2u | (3u << 4) | (85u << 8) | (1u << 24);
    RCC_CR |= RCC_CR_PLLON;
    while (!(RCC_CR & RCC_CR_PLLRDY)) {
    }
    /* step through AHB/2 for 1 us as RM0440 asks when jumping above 80 MHz */
    RCC_CFGR = (RCC_CFGR & ~((0xFu << 4) | 3u)) | (8u << 4) | 3u;
    while (((RCC_CFGR >> 2) & 3u) != 3u) {
    }
    for (volatile int i = 0; i < 200; i++) {
    }
    RCC_CFGR &= ~(0xFu << 4);
}

static void gpio_af(uint32_t port, int pin, int af)
{
    GPIO_MODER(port) = (GPIO_MODER(port) & ~(3u << (2 * pin))) | (2u << (2 * pin));
    GPIO_OSPEEDR(port) |= 3u << (2 * pin);
    volatile uint32_t *afr = pin < 8 ? &GPIO_AFRL(port) : &GPIO_AFRH(port);
    int sh = 4 * (pin & 7);
    *afr = (*afr & ~(0xFu << sh)) | ((uint32_t)af << sh);
}

static void pwm_init(void)
{
    RCC_AHB2ENR |= 0x7u; /* GPIOA, B, C */
    RCC_APB2ENR |= 1u << 11; /* TIM1 */
    gpio_af(GPIOA_BASE, 8, 6);  /* CH1  */
    gpio_af(GPIOA_BASE, 9, 6);  /* CH2  */
    gpio_af(GPIOA_BASE, 10, 6); /* CH3  */
    gpio_af(GPIOC_BASE, 13, 4); /* CH1N */
    gpio_af(GPIOA_BASE, 12, 6); /* CH2N */
    gpio_af(GPIOB_BASE, 15, 4); /* CH3N */

    TIM1_PSC = 0;
    TIM1_ARR = PWM_ARR;
    TIM1_RCR = 1;                       /* update once per full up-down period */
    TIM1_CR1 = (1u << 5) | (1u << 7);   /* centre-aligned mode 1, ARR preload */
    /* PWM mode 1 with preload on CH1..CH3, CH4 as ADC trigger */
    TIM1_CCMR1 = (6u << 4) | (1u << 3) | (6u << 12) | (1u << 11);
    TIM1_CCMR2 = (6u << 4) | (1u << 3) | (7u << 12);
    TIM1_CCR4 = PWM_ARR - 1;            /* sample near the top: low sides on */
    TIM1_CR2 = 7u << 20;                /* TRGO2 = OC4REF */
    TIM1_CCER = 0x555u;                 /* CH1..CH3 and complements enabled, CH4 internal */
    uint32_t dt = DEADTIME_NS * (SYSCLK_HZ / 1000000u) / 1000u; /* < 128: direct DTG encoding */
    TIM1_BDTR = dt;                     /* MOE stays off until the controller runs */
    TIM1_CCR1 = TIM1_CCR2 = TIM1_CCR3 = PWM_ARR / 2;
    TIM1_EGR = 1;
    TIM1_CR1 |= 1u;
}

static void bridge(int on)
{
    if (on)
        TIM1_BDTR |= 1u << 15;
    else
        TIM1_BDTR &= ~(1u << 15);
}

static void adc_enable(uint32_t adc)
{
    ADC_CR(adc) &= ~ADC_CR_DEEPPWD;
    ADC_CR(adc) |= ADC_CR_ADVREGEN;
    for (volatile int i = 0; i < 4000; i++) { /* tADCVREG_STUP = 20 us */
    }
    ADC_CR(adc) |= ADC_CR_ADCAL;
    while (ADC_CR(adc) & ADC_CR_ADCAL) {
    }
    ADC_ISR(adc) = ADC_ISR_ADRDY;
    ADC_CR(adc) |= ADC_CR_ADEN;
    while (!(ADC_ISR(adc) & ADC_ISR_ADRDY)) {
    }
}

static void adc_init(void)
{
    RCC_AHB2ENR |= 1u << 13;          /* ADC12 */
    ADC12_CCR = 3u << 16;             /* HCLK/4 = 42.5 MHz */
    for (int n = 0; n < 2; n++)       /* OPAMP1, OPAMP2: PGA x16 to the ADC internally */
        OPAMP_CSR(n) = 1u | (2u << 5) | (4u << 14) | (1u << 8);
    adc_enable(ADC1_BASE);
    adc_enable(ADC2_BASE);
    ADC_SMPR1(ADC1_BASE) = 0x2u << (3 * ADC1_CH_VBUS);
    /* ADC1: phase A then Vbus; ADC2: phase B. Rising edge of TIM1_TRGO2. */
    ADC_JSQR(ADC1_BASE) = 1u | (ADC_JEXTSEL_TIM1_TRGO2 << 2) | (1u << 7) | (ADC1_CH_OPAMP1 << 9) | (ADC1_CH_VBUS << 15);
    ADC_JSQR(ADC2_BASE) = 0u | (ADC_JEXTSEL_TIM1_TRGO2 << 2) | (1u << 7) | (ADC2_CH_OPAMP2 << 9);
    ADC_IER(ADC1_BASE) = ADC_ISR_JEOS;
    NVIC_IPR8(IRQ_ADC1_2) = 0x00;     /* highest priority: nothing may delay the loop */
    NVIC_ISER(IRQ_ADC1_2 / 32) = 1u << (IRQ_ADC1_2 % 32);
    ADC_CR(ADC1_BASE) |= ADC_CR_JADSTART;
    ADC_CR(ADC2_BASE) |= ADC_CR_JADSTART;
}

/* Zero-current offsets with the bridge off: average 1024 samples per phase. */
static volatile int calib_left = 1024;
static float acc_a, acc_b;

void ADC1_2_IRQHandler(void)
{
    uint32_t t0 = DWT_CYCCNT;
    ADC_ISR(ADC1_BASE) = ADC_ISR_JEOS;
    float raw_a = (float)ADC_JDR1(ADC1_BASE), raw_b = (float)ADC_JDR1(ADC2_BASE);
    float vbus = (float)ADC_JDR2(ADC1_BASE) * VBUS_PER_LSB;

    if (calib_left > 0) {
        acc_a += raw_a;
        acc_b += raw_b;
        if (--calib_left == 0) {
            offset_a = acc_a / 1024.0f;
            offset_b = acc_b / 1024.0f;
        }
        return;
    }
    /* shunts sit in the low-side legs: positive phase current reads negative */
    float ia = (offset_a - raw_a) * AMPS_PER_LSB;
    float ib = (offset_b - raw_b) * AMPS_PER_LSB;

    abc_t d;
    int on = foc_step(&foc, ia, ib, vbus, &d);
    TIM1_CCR1 = (uint32_t)(d.a * (float)PWM_ARR);
    TIM1_CCR2 = (uint32_t)(d.b * (float)PWM_ARR);
    TIM1_CCR3 = (uint32_t)(d.c * (float)PWM_ARR);
    bridge(on);

    uint32_t dt = DWT_CYCCNT - t0;
    loop_cycles = dt;
    if (dt > loop_cycles_max) loop_cycles_max = dt;
    telemetry_sample(&foc, vbus, dt);
}

void SysTick_Handler(void) { ms++; }
void HardFault_Handler(void)
{
    bridge(0);
    for (;;) {
    }
}

int main(void)
{
    clock_init();
    DEMCR |= 1u << 24; /* cycle counter for loop timing */
    DWT_CTRL |= 1u;
    SYST_RVR = SYSCLK_HZ / 1000u - 1u;
    SYST_CSR = 7u;

    static const foc_config_t cfg = MOTOR_CONFIG;
    foc_init(&foc, &cfg);
    telemetry_init();
    pwm_init();
    adc_init();

    for (;;) {
        /* commands: "g<rpm>" start, "s<rpm>" set speed, "x" stop, "c" clear fault */
        telemetry_cmd_t cmd;
        if (telemetry_poll(&cmd)) {
            __asm volatile("cpsid i");
            switch (cmd.op) {
            case 'g': foc_start(&foc, cmd.value); break;
            case 's': foc_set_speed(&foc, cmd.value); break;
            case 'x': foc_stop(&foc); break;
            case 'c':
                if (foc.state == FOC_FAULT) foc.state = FOC_IDLE, foc.fault = FAULT_NONE;
                break;
            }
            __asm volatile("cpsie i");
        }
        telemetry_flush(ms);
    }
}
