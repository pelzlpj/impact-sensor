/******************************************************************************
 * toplevel module
 *
 * Copyright (C) 2013 Paul Pelzl
 * Simplified BSD license.  See README.md for details.
 *
 * Implements a state machine which ping-pongs between (1) collecting
 * accelerometer data until a collision event is detected and (2)
 * transmitting the data to a host PC.
 ******************************************************************************/

#include <cstring>
#include "AccelSampler.h"
#include <wirish/wirish.h>
#include "RN42.h"
#include "util.h"
#include "serial_util.h"


using namespace libmaple_util;

namespace {

    // Used for RN42 connections
    const uint8_t BOARD_USART3_RTS_PIN = BOARD_SPI2_MISO_PIN;
    const uint8_t BOARD_USART3_CTS_PIN = BOARD_SPI2_SCK_PIN;
    const uint8_t BOARD_USART1_CK_PIN  = 27;

    // Used for accelerometer axes
    const uint8_t BOARD_ADC_IN0        = 11;
    const uint8_t BOARD_ADC_IN1        = 10;
    const uint8_t BOARD_ADC_IN2        = 9;

    // Accelerometer storage goes here.  Libmaple data structures
    // take up enough space that this buffer can't be increased
    // much without smashing the stack.
    uint16_t accel_buf[5000];

    struct fsm_context
    {
        // Bluetooth module used to send data to host PC
        bluetooth::RN42 * const rn42;

        // Accelerometer module used for data capture
        accel::AccelSampler * const accel;

        // Offset into <accel_buf> ring buffer where captured
        // data begins
        size_t sample_buf_start;

        // Current FSM state
        void (*state)(struct fsm_context *);
    };

    void state_acquire_data    (struct fsm_context * ctx);
    void state_reset_connection(struct fsm_context * ctx);
    void state_wait_connection (struct fsm_context * ctx);
    void state_send_data       (struct fsm_context * ctx);
    void state_wait_response   (struct fsm_context * ctx);

    void write_32le(HardwareSerial * const serial, const uint32_t val)
    {
        serial->write(static_cast<uint8_t>(val >>  0));
        serial->write(static_cast<uint8_t>(val >>  8));
        serial->write(static_cast<uint8_t>(val >> 16));
        serial->write(static_cast<uint8_t>(val >> 24));
    }

    void state_acquire_data(struct fsm_context * ctx)
    {
        SerialUSB.println("Acquiring data...");
        ctx->sample_buf_start = ctx->accel->capture_event();
        ctx->rn42->clear_reset();
        ctx->state = state_wait_connection;
    }

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

        SerialUSB.println("Waiting for command...");
        char buf[10];
        if (read_line(ctx->rn42->serial(), buf, sizeof(buf), 30000) != rl_status::OK) {
            ctx->state = state_reset_connection;
            return;
        }
        if (std::strcmp(buf, "send") == 0) {
            ctx->state = state_send_data;
        } else {
            SerialUSB.println("Bad command.");
            ctx->state = state_reset_connection;
        }
    }

    // Payloads sent to the remote listener look like:
    // 1) uint32 payload byte count, encoded as little-endian
    // 2) payload bytes
    // 3) uint32 Adler-32 checksum of the payload, encoded
    //    as little-endian
    void state_send_data(struct fsm_context * ctx)
    {
        SerialUSB.println("Sending data...");
        const uint32_t byte_count = sizeof(accel_buf);
        write_32le(ctx->rn42->serial(), byte_count);

        uint32_t adler = 1;
        for (uint32_t i = 0; i < ARRAY_COUNT(accel_buf); i++) {
            if (!ctx->rn42->is_connected()) {
                ctx->state = state_reset_connection;
                return;
            }
            toggleLED();
            const size_t ofs   = (ctx->sample_buf_start + i) % ARRAY_COUNT(accel_buf);
            const uint16_t val = accel_buf[ofs];
            const uint8_t lo   = val >> 0;
            const uint8_t hi   = val >> 8;

            ctx->rn42->serial()->write(lo);
            adler = util::adler32(&lo, sizeof(lo), adler);

            ctx->rn42->serial()->write(hi);
            adler = util::adler32(&hi, sizeof(hi), adler);
        }

        write_32le(ctx->rn42->serial(), adler);
        ctx->state = state_wait_response;
    }

    // Valid responses from the remote listener are:
    //
    // 1) "ack"  --> payload received OK, go back to data acquisition
    // 2) "send" --> error receiving payload, send it again
    //
    // If anything else happens, reset the connection and wait for
    // the remote listener to try again.
    void state_wait_response(struct fsm_context * ctx)
    {
        SerialUSB.println("Waiting for response...");
        char buf[10];
        if (!ctx->rn42->is_connected() ||
                read_line(ctx->rn42->serial(), buf, sizeof(buf), 5000) != rl_status::OK) {
            ctx->state = state_reset_connection;
            return;
        }
        if (std::strcmp(buf, "send") == 0) {
            SerialUSB.println("Retransmit requested.");
            ctx->state = state_send_data;
        } else if (std::strcmp(buf, "ack") == 0) {
            SerialUSB.println("Payload acknowledged.");
            ctx->rn42->assert_reset();
            ctx->state = state_acquire_data;
        }
    }

}   // end anonymous namespace


// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}


int main(void) {
    using namespace bluetooth;

    pinMode(BOARD_LED_PIN, OUTPUT);

    SerialUSB.println("RN42 init...");
    RN42::pin_assignments rn42_pins;
    rn42_pins.cts   = BOARD_USART3_RTS_PIN;
    rn42_pins.rts   = BOARD_USART3_CTS_PIN;
    rn42_pins.reset = BOARD_SPI2_MOSI_PIN;
    rn42_pins.conn  = BOARD_USART1_CK_PIN;
    RN42 rn42(&Serial3, rn42_pins);
    if (!rn42.configure_fast_data_mode()) {
        SerialUSB.println("Warning: unable to configure RN42 fast data mode.");
    }
    // Start out with RN42 in lower-power mode
    rn42.assert_reset();

    SerialUSB.println("Accelerometer/ADC init...");
    accel::AccelSampler::pin_assignments accel_pins;
	accel_pins.adc_x = BOARD_ADC_IN0;
	accel_pins.adc_y = BOARD_ADC_IN1;
	accel_pins.adc_z = BOARD_ADC_IN2;
    accel::AccelSampler accel(accel_pins, accel_buf, ARRAY_COUNT(accel_buf));
    if (!accel.init()) {
		SerialUSB.println("Error: unable to init accel module.");
		while (true);
    }
    accel.calibrate();

    struct fsm_context ctx = {
        &rn42,
        &accel,
        0,
        state_acquire_data
    };

    while (true) {
        ctx.state(&ctx);
    }

    return 0;
}

