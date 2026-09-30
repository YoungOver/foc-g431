/* Minimal register map for the peripherals this firmware uses on STM32G431.
 * Offsets and bit positions follow RM0440. Only what is used is defined. */
#ifndef REGS_H
#define REGS_H

#include <stdint.h>

#define REG(addr) (*(volatile uint32_t *)(addr))

/* RCC */
#define RCC_BASE      0x40021000u
#define RCC_CR        REG(RCC_BASE + 0x00)
#define RCC_CFGR      REG(RCC_BASE + 0x08)
#define RCC_PLLCFGR   REG(RCC_BASE + 0x0C)
#define RCC_AHB1ENR   REG(RCC_BASE + 0x48)
#define RCC_AHB2ENR   REG(RCC_BASE + 0x4C)
#define RCC_APB1ENR1  REG(RCC_BASE + 0x58)
#define RCC_APB2ENR   REG(RCC_BASE + 0x60)
#define RCC_CCIPR     REG(RCC_BASE + 0x88)

#define RCC_CR_HSEON   (1u << 16)
#define RCC_CR_HSERDY  (1u << 17)
#define RCC_CR_PLLON   (1u << 24)
#define RCC_CR_PLLRDY  (1u << 25)

/* FLASH and PWR: 170 MHz needs 4 wait states and voltage range 1 boost */
#define FLASH_ACR     REG(0x40022000u)
#define PWR_CR5       REG(0x40007000u + 0x80)

/* GPIO */
#define GPIOA_BASE 0x48000000u
#define GPIOB_BASE 0x48000400u
#define GPIOC_BASE 0x48000800u
#define GPIO_MODER(b)   REG((b) + 0x00)
#define GPIO_OSPEEDR(b) REG((b) + 0x08)
#define GPIO_BSRR(b)    REG((b) + 0x18)
#define GPIO_AFRL(b)    REG((b) + 0x20)
#define GPIO_AFRH(b)    REG((b) + 0x24)

/* TIM1: advanced timer with complementary outputs and dead time */
#define TIM1_BASE   0x40012C00u
#define TIM1_CR1    REG(TIM1_BASE + 0x00)
#define TIM1_CR2    REG(TIM1_BASE + 0x04)
#define TIM1_CCMR1  REG(TIM1_BASE + 0x18)
#define TIM1_CCMR2  REG(TIM1_BASE + 0x1C)
#define TIM1_CCER   REG(TIM1_BASE + 0x20)
#define TIM1_PSC    REG(TIM1_BASE + 0x28)
#define TIM1_ARR    REG(TIM1_BASE + 0x2C)
#define TIM1_RCR    REG(TIM1_BASE + 0x30)
#define TIM1_CCR1   REG(TIM1_BASE + 0x34)
#define TIM1_CCR2   REG(TIM1_BASE + 0x38)
#define TIM1_CCR3   REG(TIM1_BASE + 0x3C)
#define TIM1_CCR4   REG(TIM1_BASE + 0x40)
#define TIM1_BDTR   REG(TIM1_BASE + 0x44)
#define TIM1_EGR    REG(TIM1_BASE + 0x14)

/* ADC1/ADC2 and their common block */
#define ADC1_BASE   0x50000000u
#define ADC2_BASE   0x50000100u
#define ADC_ISR(b)  REG((b) + 0x00)
#define ADC_IER(b)  REG((b) + 0x04)
#define ADC_CR(b)   REG((b) + 0x08)
#define ADC_SMPR1(b) REG((b) + 0x14)
#define ADC_JSQR(b) REG((b) + 0x4C)
#define ADC_JDR1(b) REG((b) + 0x80)
#define ADC_JDR2(b) REG((b) + 0x84)
#define ADC12_CCR   REG(0x50000300u + 0x08)

#define ADC_CR_ADEN     (1u << 0)
#define ADC_CR_JADSTART (1u << 3)
#define ADC_CR_ADVREGEN (1u << 28)
#define ADC_CR_DEEPPWD  (1u << 29)
#define ADC_CR_ADCAL    (1u << 31)
#define ADC_ISR_ADRDY   (1u << 0)
#define ADC_ISR_JEOS    (1u << 6)

/* OPAMP1..3 amplify the shunt voltages on the B-G431B-ESC1 */
#define OPAMP_CSR(n) REG(0x40010300u + 4u * (unsigned)(n))

/* USART2 routed to the ST-LINK virtual COM port */
#define USART2_BASE 0x40004400u
#define USART2_CR1  REG(USART2_BASE + 0x00)
#define USART2_BRR  REG(USART2_BASE + 0x0C)
#define USART2_ISR  REG(USART2_BASE + 0x1C)
#define USART2_RDR  REG(USART2_BASE + 0x24)
#define USART2_TDR  REG(USART2_BASE + 0x28)

/* Cortex-M4 core */
#define NVIC_ISER(n)  REG(0xE000E100u + 4u * (n))
#define NVIC_IPR8(n)  (*(volatile uint8_t *)(0xE000E400u + (n)))
#define SCB_CPACR     REG(0xE000ED88u)
#define SYST_CSR      REG(0xE000E010u)
#define SYST_RVR      REG(0xE000E014u)
#define DWT_CTRL      REG(0xE0001000u)
#define DWT_CYCCNT    REG(0xE0001004u)
#define DEMCR         REG(0xE000EDFCu)

#define IRQ_ADC1_2 18

#endif
