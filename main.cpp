#include <cstring>
#include <wirish/wirish.h>
#include <libmaple/usart.h>
#include <libmaple/gpio.h>


const int BOARD_USART3_RTS_PIN = BOARD_SPI2_MISO_PIN;
const int BOARD_USART3_CTS_PIN = BOARD_SPI2_SCK_PIN;


// Configure RTS/CTS flow control for the given USART.  (libmaple doesn't
// have the hooks for flow control.)
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


// Read a line from serial port <serial>, stopping at CRLF.
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


namespace cmd_status {
    enum type { 
        CMD_AOK,        // Command accepted by RN42
        CMD_ERR,        // Command rejected by RN42 as invalid
        CMD_UNREC,      // Command received by RN42, and not recognized
        COMM_ERROR      // Serial communication error
    };
}

class RN42
{
private:
    HardwareSerial * const serial;

    // Send a '\r'-terminated command, and wait for the AOK response.
    cmd_status::type issue_command(const char * cmd)
    {
        serial->flush();
        serial->print(cmd);

        char buf[4];
        if (read_line(serial, buf, sizeof(buf), 200) != rl_status::OK) {
            SerialUSB.println("comm error");
            return cmd_status::COMM_ERROR;
        }
        SerialUSB.println(buf);

        if (std::strcmp(buf, "AOK") == 0) {
            return cmd_status::CMD_AOK;
        } else if (std::strcmp(buf, "ERR") == 0) {
            return cmd_status::CMD_ERR;
        } else if (std::strcmp(buf, "?") == 0) {
            return cmd_status::CMD_UNREC;
        } else {
            return cmd_status::COMM_ERROR;
        }
    }


public:
    RN42(HardwareSerial * serial_, int rts_pin, int cts_pin) :
        serial (serial_)
    {
        setup_hw_flow_control(serial->c_dev(), rts_pin, cts_pin);
        serial->begin(115200);
    }

    // Asserts chip reset to the RN42.  After chip reset completes,
    // the device is placed in Command Mode.
    //
    // Returns: true if command mode negotiated, false otherwise
    bool reset(void)
    {
        // TODO: assert chip reset... don't have the wire yet.

        serial->flush();

        // One-second halt in upstream traffic is required on both sides
        // of the command code
        delay(1100);
        serial->print("$$$");
        delay(1100);

        // Expected response is "CMD\r"
        char buf[4];
        if (read_line(serial, buf, sizeof(buf), 200) != rl_status::OK) {
            return false;
        }
        return std::strcmp(buf, "CMD") == 0;
    }


    // Temporarily disables connections.
    //
    // Returns: command status
    cmd_status::type disable_connections(void)
    {
        serial->flush();
        serial->print("Q\r");
        char buf[6];
        if (read_line(serial, buf, sizeof(buf), 200) != rl_status::OK) {
            return cmd_status::COMM_ERROR;
        }
        // User's Guide says this should return "Quiet", but that looks incorrect.
        return std::strcmp(buf, "Q=0") == 0 ? cmd_status::CMD_AOK : cmd_status::COMM_ERROR;
    }


    // Places the RN42 in a low power state which persists until reset.
    //
    // Returns: true if rn42 is nonresponsive, false if the USART shows activity
    bool enter_low_power_mode(void)
    {
        if (disable_connections() != cmd_status::CMD_AOK) {
            SerialUSB.println("Warning: failed to disable connections.");
        }
        serial->flush();

        serial->print("Z\r");
        delay(100);
        return !serial->available();
    }


    cmd_status::type reboot(void)
    {
        return issue_command("R,1\r");
    }


    cmd_status::type get_command(const char * command, char * response, size_t response_len)
    {
        serial->print(command);
        if (read_line(serial, response, response_len, 200) != rl_status::OK) {
            return cmd_status::COMM_ERROR;
        }
        return cmd_status::CMD_AOK;
    }
};


static void print_cmd_status(cmd_status::type st)
{
    switch (st) {
        case cmd_status::CMD_AOK:
            SerialUSB.print("AOK");
            break;
        case cmd_status::CMD_ERR:
            SerialUSB.print("ERR");
            break;
        case cmd_status::CMD_UNREC:
            SerialUSB.print("?");
            break;
        case cmd_status::COMM_ERROR:
            SerialUSB.print("COMM ERROR");
            break;
    }
}


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

    delay(5000);
    SerialUSB.print("low power: ");
    SerialUSB.println(rn42.enter_low_power_mode() ? "true" : "false");
    while (true) {
        const char c = static_cast<char>(Serial3.read());
        SerialUSB.write(c);
    }

    while (true) {
        SerialUSB.print("Reset status: ");
        SerialUSB.println(reset_status ? "true" : "false");

        toggleLED();
        delay(300);
    }

    return 0;
}
