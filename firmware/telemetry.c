#include "telemetry.h"
#include "regs.h"
#include "board.h"

static volatile telemetry_frame_t latest;
static uint8_t tx[64];
static int tx_len, tx_pos;
static char rx[16];
static int rx_len;
static uint32_t last_ms;

uint8_t crc8(const uint8_t *p, int n) /* polynomial 0x07, init 0 */
{
    uint8_t c = 0;
    while (n--) {
        c ^= *p++;
        for (int i = 0; i < 8; i++) c = (uint8_t)(c & 0x80 ? (c << 1) ^ 0x07 : c << 1);
    }
    return c;
}

void telemetry_init(void)
{
    RCC_APB1ENR1 |= 1u << 17; /* USART2 */
    /* PB3 TX, PB4 RX, AF7 */
    GPIO_MODER(GPIOB_BASE) = (GPIO_MODER(GPIOB_BASE) & ~(0xFu << 6)) | (0xAu << 6);
    GPIO_AFRL(GPIOB_BASE) = (GPIO_AFRL(GPIOB_BASE) & ~(0xFFu << 12)) | (0x77u << 12);
    USART2_BRR = SYSCLK_HZ / 921600u;
    USART2_CR1 = (1u << 3) | (1u << 2) | 1u; /* TE, RE, UE */
}

void telemetry_sample(const foc_t *f, float vbus, uint32_t cycles)
{
    latest.rpm = f->speed_rpm;
    latest.iq = f->i_dq.q;
    latest.iq_ref = f->iq_ref;
    latest.id = f->i_dq.d;
    latest.vbus = vbus;
    latest.loop_cycles = (uint16_t)(cycles > 0xFFFF ? 0xFFFF : cycles);
    latest.state = (uint8_t)f->state;
    latest.fault = (uint8_t)f->fault;
}

void telemetry_flush(uint32_t ms)
{
    /* push bytes while the transmit register is free; never wait */
    while (tx_pos < tx_len && (USART2_ISR & (1u << 7))) USART2_TDR = tx[tx_pos++];
    if (tx_pos < tx_len || ms - last_ms < 2) return;
    last_ms = ms;

    __asm volatile("cpsid i");
    telemetry_frame_t fr = latest; /* consistent snapshot of one control period */
    __asm volatile("cpsie i");
    fr.t_ms = ms;

    tx[0] = 0xA5;
    tx[1] = 0x5A;
    tx[2] = sizeof fr;
    const uint8_t *p = (const uint8_t *)&fr;
    for (unsigned i = 0; i < sizeof fr; i++) tx[3 + i] = p[i];
    tx[3 + sizeof fr] = crc8(tx + 2, 1 + sizeof fr);
    tx_len = 4 + sizeof fr;
    tx_pos = 0;
}

static float parse_float(const char *s)
{
    float v = 0, sign = 1;
    if (*s == '-') sign = -1, s++;
    while (*s >= '0' && *s <= '9') v = v * 10 + (float)(*s++ - '0');
    return sign * v;
}

int telemetry_poll(telemetry_cmd_t *cmd)
{
    while (USART2_ISR & (1u << 5)) {
        char ch = (char)USART2_RDR;
        if (ch == '\n' || ch == '\r') {
            if (rx_len == 0) continue;
            rx[rx_len] = 0;
            cmd->op = rx[0];
            cmd->value = parse_float(rx + 1);
            rx_len = 0;
            return 1;
        }
        if (rx_len < (int)sizeof rx - 1) rx[rx_len++] = ch;
    }
    return 0;
}
