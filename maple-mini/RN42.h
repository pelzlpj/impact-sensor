
#ifndef INCLUDE_GUARD_9d8d2841_e2ce_48b2_b74c_c1808234d18e
#define INCLUDE_GUARD_9d8d2841_e2ce_48b2_b74c_c1808234d18e

#include <wirish/wirish.h>

namespace bluetooth {

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

    bool enter_command_mode(void);

public:
    struct pin_assignments
    {
        uint8_t rts;        // USART RTS pin
        uint8_t cts;        // USART CTS pin
        uint8_t reset;      // GPIO wired to active-low RN42 reset
        uint8_t conn;       // GPIO wired to RN42 pin 13 (connection status)
    };

    // Construct a new instance using the given serial port and
    // pin assignments.  The chip is held in reset.
    RN42(HardwareSerial * serial, const pin_assignments & pins);

    // Asserts chip reset to the RN42.  The chip should come up in
    // Data Mode.
    void reset(void);

    // Asserts the reset signal to the RN42.  While in reset the
    // chip consumes ~1.5mA; this compares favorably to the "deep sleep"
    // mode which consumes more like 5mA.
    void assert_reset(void);

    // Clears the reset signal to the RN42.  The chip should come up
    // in Data Mode.
    void clear_reset(void);

    // Force the RN42 to enter Fast Data Mode as soon as a connection is
    // available.  It is recommended to call this once at power-on.
    //
    // By default, the RN42 has a 60-second configuration window in which it will
    // snoop the data stream (in both directions) for an escape sequence which
    // enables command mode.  Unfortunately, this "Data Mode" seems really
    // glitchy.  So we disable it in favor of using the "Fast Data Mode" which
    // passes all data.  We can still configure the chip using the local serial
    // port as long as there is no connection.
    //
    // Returns: true if successful, false otherwise.
    bool configure_fast_data_mode(void);

    // Returns: RN42 connection state
    bool is_connected(void);

    // Returns: handle to the serial device attached to the RN42
    HardwareSerial * serial(void);
};


}   // namespace bluetooth

#endif // INCLUDE_GUARD_9d8d2841_e2ce_48b2_b74c_c1808234d18e

