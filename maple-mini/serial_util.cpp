/******************************************************************************
 * serial_util
 *
 * Copyright (C) 2013 Paul Pelzl
 * Simplified BSD license.  See README.md for details.
 *
 * Extra functionality for working with libmaple serial ports.
 ******************************************************************************/

#include <wirish/wirish.h>
#include <libmaple/usart.h>
#include "serial_util.h"

namespace libmaple_util {

rl_status::type
read_line(HardwareSerial * const serial, char * const buf,
        const size_t buf_len, const uint32_t timeout)
{
    size_t chars_read = 0;
    while (true) {
        const uint32_t start_time = millis();
        while (!serial->available() &&
                (timeout == TIMEOUT_NONE || millis() - start_time < timeout));
        if (serial->available()) {
            const char c = static_cast<char>(serial->read());
            if (c == '\r') {
                // silently discard
            } else if (c == '\n') {
                if (chars_read < buf_len) {
                    buf[chars_read] = '\0';
                    return rl_status::OK;
                } else {
                    return rl_status::FULL_BUFFER;
                }
            } else if (chars_read < buf_len) {
                buf[chars_read] = c;
                chars_read++;
            } else {
                return rl_status::FULL_BUFFER;
            }
        } else {
            return rl_status::TIMEOUT;
        }
    }

    // Never happens
    return rl_status::FULL_BUFFER;
}


void
setup_hw_flow_control(struct usart_dev * const udev,
        const uint8_t rts_pin, const uint8_t cts_pin)
{
    udev->regs->CR3 |= USART_CR3_CTSE_BIT;
    udev->regs->CR3 |= USART_CR3_RTSE_BIT;

    const stm32_pin_info & rts = PIN_MAP[rts_pin];
    const stm32_pin_info & cts = PIN_MAP[cts_pin];
    gpio_set_mode(rts.gpio_device, rts.gpio_bit, GPIO_AF_OUTPUT_PP);
    gpio_set_mode(cts.gpio_device, cts.gpio_bit, GPIO_INPUT_FLOATING);
}

}   // namespace libmaple_util


