#include <cstring>
#include "Accel.h"
#include <wirish/wirish.h>
#include "RN42.h"
#include "util.h"
#include "serial_util.h"


using namespace libmaple_util;

const uint8_t BOARD_USART3_RTS_PIN = BOARD_SPI2_MISO_PIN;
const uint8_t BOARD_USART3_CTS_PIN = BOARD_SPI2_SCK_PIN;
const uint8_t BOARD_USART1_CK_PIN  = 27;
const uint8_t BOARD_ADC_IN0        = 11;
const uint8_t BOARD_ADC_IN1        = 10;
const uint8_t BOARD_ADC_IN2        = 9;


// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}


struct fsm_context
{
    bluetooth::RN42 * const rn42;
    void (*state)(struct fsm_context *);
};

void state_reset_connection(struct fsm_context * ctx);
void state_wait_connection (struct fsm_context * ctx);
void state_send_data       (struct fsm_context * ctx);
void state_wait_response   (struct fsm_context * ctx);
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


void state_send_data(struct fsm_context * ctx)
{
    SerialUSB.println("Sending data...");
    const uint32_t byte_count = 100;
    ctx->rn42->serial()->write(static_cast<uint8_t>(byte_count >>  0));
    ctx->rn42->serial()->write(static_cast<uint8_t>(byte_count >>  8));
    ctx->rn42->serial()->write(static_cast<uint8_t>(byte_count >> 16));
    ctx->rn42->serial()->write(static_cast<uint8_t>(byte_count >> 24));

    uint32_t adler = 1;
    for (uint8_t i = 0; i < byte_count; i++) {
        if (!ctx->rn42->is_connected()) {
            ctx->state = state_reset_connection;
            return;
        }
        toggleLED();
        ctx->rn42->serial()->write(i);
        adler = util::adler32(&i, sizeof(i), adler);
    }

    ctx->rn42->serial()->write(static_cast<uint8_t>(adler >>  0));
    ctx->rn42->serial()->write(static_cast<uint8_t>(adler >>  8));
    ctx->rn42->serial()->write(static_cast<uint8_t>(adler >> 16));
    ctx->rn42->serial()->write(static_cast<uint8_t>(adler >> 24));

    ctx->state = state_wait_response;
}


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
        ctx->state = state_idle;
    }
}


void state_idle(struct fsm_context * ctx)
{
    SerialUSB.println("Idling.");
    toggleLED();
    delay(500);
}


int main(void) {
    using namespace bluetooth;

    pinMode(BOARD_LED_PIN, OUTPUT);

    RN42::pin_assignments rn42_pins;
    rn42_pins.cts   = BOARD_USART3_RTS_PIN;
    rn42_pins.rts   = BOARD_USART3_CTS_PIN;
    rn42_pins.reset = BOARD_SPI2_MOSI_PIN;
    rn42_pins.conn  = BOARD_USART1_CK_PIN;

    RN42 rn42(&Serial3, rn42_pins);

#if 0
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
#endif

	AccelSampler::pin_assignments accel_pins;
	accel_pins.adc_x = BOARD_ADC_IN0;
	accel_pins.adc_y = BOARD_ADC_IN1;
	accel_pins.adc_z = BOARD_ADC_IN2;

	delay(5000);

	SerialUSB.println("Init accel module");
	uint16_t buf[10];
	AccelSampler accel(accel_pins, buf, ARRAY_COUNT(buf));
	if (!accel.init()) {
		SerialUSB.println("Error: unable to init accel module.");
		while (true);
	}
	accel.power_up_adc();
    accel.calibrate();

    while (true) {
        toggleLED();
        delay(1000);
    }

    return 0;
}

