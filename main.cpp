#include <cstring>
#include <wirish/wirish.h>
#include <libmaple/usart.h>
#include <libmaple/gpio.h>


const int BOARD_USART3_RTS_PIN = BOARD_SPI2_MISO_PIN;
const int BOARD_USART3_CTS_PIN = BOARD_SPI2_SCK_PIN;


static void setup_hw_flow_control(struct usart_dev * const udev,
        const int rts_pin, const int cts_pin)
{
    udev->regs->CR3 |= USART_CR3_CTSE_BIT;
    udev->regs->CR3 |= USART_CR3_RTSE_BIT;

    const stm32_pin_info & rts = PIN_MAP[rts_pin];
    const stm32_pin_info & cts = PIN_MAP[cts_pin];
    gpio_set_mode(rts.gpio_device, rts.gpio_bit, GPIO_AF_OUTPUT_PP);
    gpio_set_mode(cts.gpio_device, cts.gpio_bit, GPIO_INPUT_FLOATING);
}


namespace rl_status {
    enum type { OK, FULL_BUFFER, TIMEOUT };
}

// Read a line from serial port <serial>, stopping at carriage return.
// If a character cannot be read for more than <timeout> msec, the
// function terminates; setting <timeout>==0 disables the timeout.
//
// Upon success, the buffer is null-terminated.
//
// Returns: status code
static rl_status::type read_line(HardwareSerial * const serial, char * const buf,
        const size_t buf_len, const uint32_t timeout)
{
    size_t chars_read = 0;
    while (true) {
        const uint32_t start_time = millis();
        while (!serial->available() &&
                (!timeout || millis() - start_time < timeout));
        if (serial->available()) {
            const char c = static_cast<char>(serial->read());
            if (c == '\r') {
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


class RN42
{
private:
    HardwareSerial * const serial;

public:
    RN42(HardwareSerial * serial_, int rts_pin, int cts_pin) :
        serial (serial_)
    {
        setup_hw_flow_control(serial->c_dev(), rts_pin, cts_pin);
        serial->begin(115200);
    }

    // Asserts chip reset to the RN42.  After chip reset completes,
    // the device is placed in command mode.
    //
    // Returns: true if command mode negotiated, false otherwise
    bool reset(void)
    {
        // TODO: assert chip reset... don't have the wire yet.

        // Enter command mode
        serial->flush();
        serial->print("$$$");

        char buf[4];
        if (read_line(serial, buf, sizeof(buf), 200) != rl_status::OK) {
            return false;
        }

        return std::strcmp(buf, "CMD") == 0;
    }
};

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}


int main(void) {
    pinMode(BOARD_LED_PIN, OUTPUT);

    RN42 rn42(&Serial3, BOARD_USART3_RTS_PIN, BOARD_USART3_CTS_PIN);
    const bool reset_status = rn42.reset();
    if (!reset_status) {
        SerialUSB.println("Error: unable to reset RN42.");
    }

    while (true) {
        SerialUSB.print("Reset status: ");
        SerialUSB.println(reset_status ? "true" : "false");
        toggleLED();
        delay(300);
    }

    return 0;
}
