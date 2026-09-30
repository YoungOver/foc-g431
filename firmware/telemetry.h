/* Binary telemetry over USART2 at 921600 baud and a tiny text command parser.
 * Frame: 0xA5 0x5A | len u8 | payload | crc8. Payload is telemetry_frame_t. */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "../core/foc.h"

typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    float rpm, iq, iq_ref, id, vbus;
    uint16_t loop_cycles;
    uint8_t state, fault;
} telemetry_frame_t;

typedef struct {
    char op;
    float value;
} telemetry_cmd_t;

void telemetry_init(void);
/* called from the control interrupt: keeps the latest values, never blocks */
void telemetry_sample(const foc_t *f, float vbus, uint32_t cycles);
/* called from the main loop: sends one frame every 2 ms */
void telemetry_flush(uint32_t ms);
int telemetry_poll(telemetry_cmd_t *cmd);

uint8_t crc8(const uint8_t *p, int n);

#endif
