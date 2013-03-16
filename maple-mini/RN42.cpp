/******************************************************************************
 * RN42
 *
 * Copyright (C) 2013 Paul Pelzl
 * Simplified BSD license.  See README.md for details.
 *
 * Code for managing an RN42 bluetooth module connected to one of the
 * STM32 USARTs.
 ******************************************************************************/

#include <cstring>
#include "RN42.h"
#include "serial_util.h"

using namespace libmaple_util;

namespace bluetooth {

// Attempts to enter Command Mode.
//
// Returns: true if successful, false if the current state is unknown.
bool
RN42::enter_command_mode(void)
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


RN42::RN42(HardwareSerial * serial_, const pin_assignments & pins) :
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

void
RN42::reset(void)
{
    assert_reset();
    // Datasheet says a 160us pulse, but that seems inadequate.
    delay(10);
    clear_reset();
}

void
RN42::assert_reset(void)
{
    digitalWrite(reset_pin, LOW);
    state = rn42_state::RESET;
}

void
RN42::clear_reset(void)
{
    digitalWrite(reset_pin, HIGH);
    state = rn42_state::DATA_MODE;
}

bool
RN42::configure_fast_data_mode(void)
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

bool
RN42::is_connected(void)
{
    return digitalRead(conn_pin) == HIGH;
}

HardwareSerial *
RN42::serial(void)
{
    return ser;
}

}   // namespace bluetooth

