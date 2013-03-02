#include <cstring>
#include <wirish/wirish.h>
#include <libmaple/usart.h>
#include <libmaple/gpio.h>


const uint8_t BOARD_USART3_RTS_PIN = BOARD_SPI2_MISO_PIN;
const uint8_t BOARD_USART3_CTS_PIN = BOARD_SPI2_SCK_PIN;
const uint8_t BOARD_USART1_CK_PIN  = 27;


// Configure RTS/CTS flow control for the given USART.  (libmaple doesn't
// have the hooks for flow control.)
static void setup_hw_flow_control(struct usart_dev * const udev,
        const uint8_t rts_pin, const uint8_t cts_pin)
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

const uint32_t TIMEOUT_NONE = -1;

// Read a line from serial port <serial>, stopping at \n.  Carriage returns are
// discarded. If a character cannot be read for more than <timeout> msec, the
// function terminates; setting <timeout>==TIMEOUT_NONE permits an indefinite wait.
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


namespace rn42_state {
    enum type { RESET, COMMAND_MODE, DATA_MODE, UNKNOWN };
}

class RN42
{
private:
    HardwareSerial * const ser;
    const uint8_t reset_pin;
    const uint8_t conn_pin;
    rn42_state::type state;

    // Attempts to enter Command Mode.
    //
    // Returns: true if successful, false if the current state is unknown.
    bool enter_command_mode(void)
    {
        if (state == rn42_state::COMMAND_MODE) {
            return true;
        }

        if (state != rn42_state::DATA_MODE) {
            reset();
        }

        ser->flush();

        // One-second halt in upstream traffic is required on both sides
        // of the command code
        delay(1100);
        ser->print("$$$");
        delay(1100);

        // Expected response is "CMD"
        char buf[4];
        if (read_line(ser, buf, sizeof(buf), 200) != rl_status::OK) {
            return false;
        }
        state = std::strcmp(buf, "CMD") == 0 ? rn42_state::COMMAND_MODE : rn42_state::UNKNOWN;
        return state == rn42_state::COMMAND_MODE;
    }

public:
    struct PinAssignments
    {
        uint8_t rts;        // USART RTS pin
        uint8_t cts;        // USART CTS pin
        uint8_t reset;      // GPIO wired to active-low RN42 reset
        uint8_t conn;       // GPIO wired to RN42 pin 13 (connection status)
    };

    // Construct a new instance using the given serial port and
    // pin assignments.  The chip is held in reset.
    RN42(HardwareSerial * serial_, const PinAssignments & pins) :
        ser       (serial_),
        reset_pin (pins.reset),
        conn_pin  (pins.conn),
        state     (rn42_state::RESET)
    {
        setup_hw_flow_control(ser->c_dev(), pins.rts, pins.cts);
        ser->begin(115200);

        pinMode(conn_pin,  INPUT);
        pinMode(reset_pin, OUTPUT);
        assert_reset();
    }

    // Asserts chip reset to the RN42.  The chip should come up in
    // Data Mode.
    void reset(void)
    {
        assert_reset();
        // Datasheet says a 160us pulse, but that seems inadequate.
        delay(10);
        clear_reset();
    }

    // Asserts the reset signal to the RN42.  While in reset the
    // chip consumes ~1.5mA; this compares favorably to the "deep sleep"
    // mode which consumes more like 5mA.
    void assert_reset(void)
    {
        digitalWrite(reset_pin, LOW);
        state = rn42_state::RESET;
    }

    // Clears the reset signal to the RN42.  The chip should come up
    // in Data Mode.
    void clear_reset(void)
    {
        digitalWrite(reset_pin, HIGH);
        state = rn42_state::DATA_MODE;
    }

    // Force the RN42 to enter Fast Data Mode as soon as a connection is
    // available.
    //
    // By default, the RN42 has a 60-second configuration window in which it will
    // snoop the data stream (in both directions) for an escape sequence which
    // enables command mode.  Unfortunately, this "Data Mode" seems really
    // glitchy.  So we disable it in favor of using the "Fast Data Mode" which
    // passes all data.  We can still configure the chip using the local serial
    // port as long as there is no connection.
    //
    // Returns: true if successful, false otherwise.
    bool enter_fast_data_mode(void)
    {
        if (!enter_command_mode()) {
            return false;
        }

        ser->flush();
        ser->print("ST,0\r");

        // Expected response is "AOK"
        char buf[4];
        if (read_line(ser, buf, sizeof(buf), 200) != rl_status::OK ||
                std::strcmp(buf, "AOK") != 0) {
            return false;
        }

        // "Set" commands only take effect after reset
        reset();
        return true;
    }

    // Returns: RN42 connection state
    bool is_connected(void)
    {
        return digitalRead(conn_pin) == HIGH;
    }

    // Returns: handle to the serial device attached to the RN42
    HardwareSerial * serial(void)
    {
        return ser;
    }
};


// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}


struct fsm_context
{
    RN42 * const rn42;
    void (*state)(struct fsm_context *);
};

void state_reset_connection(struct fsm_context * ctx);
void state_wait_connection (struct fsm_context * ctx);
void state_send_data       (struct fsm_context * ctx);
void state_wait_checksum   (struct fsm_context * ctx);
void state_idle            (struct fsm_context * ctx);


void state_reset_connection(struct fsm_context * ctx)
{
    SerialUSB.println("Resetting link...");
    ctx->rn42->reset();
    ctx->state = state_wait_connection;
}


void state_wait_connection(struct fsm_context * ctx)
{
    SerialUSB.println("Waiting for connection...");
    while (!ctx->rn42->is_connected()) {
        toggleLED();
        delay(100);
    }

    ctx->state = state_send_data;
}


void state_send_data(struct fsm_context * ctx)
{
    SerialUSB.println("Sending data...");
    ctx->rn42->serial()->println("100");
    for (int i = 0; i < 100; i++) {
        if (!ctx->rn42->is_connected()) {
            ctx->state = state_reset_connection;
            return;
        }
        toggleLED();
        ctx->rn42->serial()->print(i);
        ctx->rn42->serial()->print("\r\n");
    }

    ctx->state = state_wait_checksum;
}


void state_wait_checksum(struct fsm_context * ctx)
{
    SerialUSB.println("Waiting for checksum...");
    char buf[10];
    if (!ctx->rn42->is_connected() ||
            read_line(ctx->rn42->serial(), buf, sizeof(buf), 1000) != rl_status::OK) {
        // Reset the RN42, reestablish the link, and try again
        ctx->state = state_reset_connection;
        return;
    }

    ctx->rn42->assert_reset();
    ctx->state = state_idle;
}


void state_idle(struct fsm_context * ctx)
{
    SerialUSB.println("Idling.");
    toggleLED();
    delay(500);
}


int main(void) {
    pinMode(BOARD_LED_PIN, OUTPUT);

    RN42::PinAssignments rn42_pins;
    rn42_pins.cts   = BOARD_USART3_RTS_PIN;
    rn42_pins.rts   = BOARD_USART3_CTS_PIN;
    rn42_pins.reset = BOARD_SPI2_MOSI_PIN;
    rn42_pins.conn  = BOARD_USART1_CK_PIN;

    RN42 rn42(&Serial3, rn42_pins);

    {
        delay(5000);
        SerialUSB.println("Now entering Fast Data Mode.");
        const bool st = rn42.enter_fast_data_mode();
        SerialUSB.print("Fast Data Mode status: ");
        SerialUSB.println(st ? "ok" : "failed");
    }

    struct fsm_context ctx = {
        &rn42,
        state_wait_connection
    };

    while (true) {
        ctx.state(&ctx);
    }

    return 0;
}

