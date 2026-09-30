/* Reset handler and vector table in C: copies .data, clears .bss and
 * CCM, enables the FPU and calls main. No vendor startup files. */
#include <stdint.h>
#include "regs.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _sccm, _eccm, _estack;
extern int main(void);
void Reset_Handler(void);
void Default_Handler(void);
void ADC1_2_IRQHandler(void);
void SysTick_Handler(void);
void HardFault_Handler(void);

void Default_Handler(void)
{
    for (;;) {
    }
}

void Reset_Handler(void)
{
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss;) *dst++ = 0;
    for (dst = &_sccm; dst < &_eccm;) *dst++ = 0;
    SCB_CPACR |= (0xFu << 20); /* full access to CP10/CP11: the FPU */
    __asm volatile("dsb\n isb");
    main();
    for (;;) {
    }
}

/* 16 core vectors + 102 device interrupts (RM0440 table 97) */
#define N_VECTORS (16 + 102)

__attribute__((section(".isr_vector"), used)) void (*const vector_table[N_VECTORS])(void) = {
    [0] = (void (*)(void))&_estack,
    [1] = Reset_Handler,
    [2] = Default_Handler,   /* NMI */
    [3] = HardFault_Handler,
    [4] = Default_Handler,   /* MemManage */
    [5] = Default_Handler,   /* BusFault */
    [6] = Default_Handler,   /* UsageFault */
    [11] = Default_Handler,  /* SVCall */
    [14] = Default_Handler,  /* PendSV */
    [15] = SysTick_Handler,
    [16 + IRQ_ADC1_2] = ADC1_2_IRQHandler,
};
