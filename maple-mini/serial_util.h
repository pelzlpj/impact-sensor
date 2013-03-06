
#ifndef INCLUDE_GUARD_d16eb6f8_a620_43a3_ad40_a0096752d705
#define INCLUDE_GUARD_d16eb6f8_a620_43a3_ad40_a0096752d705

#include <stdint.h>
#include <stddef.h>

struct usart_dev;

namespace libmaple_util {

namespace rl_status {
    enum type { OK, FULL_BUFFER, TIMEOUT };
}

const uint32_t TIMEOUT_NONE = -1;

// Read a line from serial port <serial>, stopping at \n.  Carriage returns are
// discarded. If a character cannot be read for more than <timeout> msec, the
// function terminates; setting <timeout>==TIMEOUT_NONE permits an indefinite wait.
//
// Upon success, the buffer is null-terminated.
//
// Returns: status code
rl_status::type
read_line(HardwareSerial * const serial, char * const buf,
        const size_t buf_len, const uint32_t timeout);


// Configure RTS/CTS flow control for the given USART.  (libmaple doesn't
// have the hooks for flow control.)
void
setup_hw_flow_control(struct usart_dev * const udev,
        const uint8_t rts_pin, const uint8_t cts_pin);

}   // namespace libmaple_util

#endif  // INCLUDE_GUARD_d16eb6f8_a620_43a3_ad40_a0096752d705

