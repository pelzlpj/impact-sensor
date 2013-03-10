#ifndef INCLUDE_GUARD_53da3484_c130_4662_a62e_0a6675871352
#define INCLUDE_GUARD_53da3484_c130_4662_a62e_0a6675871352


#include <stdint.h>
#include <algorithm>
#include <limits>
#include <wirish/wirish.h>
#include <libmaple/adc.h>
#include <libmaple/dma.h>

// Shame on you, libmaple...
#undef min
#undef max

#include "util.h"

namespace ADC_CR1 {
    const uint32_t SCAN  = (1UL << 8);
    const uint32_t EOCIE = (1UL << 5);
}

namespace ADC_CR2  {
    const uint32_t SWSTART = (1UL << 22);
    const uint32_t DMA     = (1UL << 8);
    const uint32_t ADON    = (1UL << 0);
}

const size_t NUM_AXES           = 3;
const size_t MEDIAN_FILTER_LEN  = 5;
const uint32_t CALIBRATION_MSEC = 10000;

// e.g. "5" ==> "collision event occurs 1/5 of the way
// through the buffer"
const unsigned int TRIGGER_DIVISIONS = 4;

const uint16_t MIN_12BIT  = 0;
const uint16_t ZERO_12BIT = 2048;
const uint16_t MAX_12BIT  = 4096;

// Sample at 250us intervals == 4kHz (compare to ~1kHz bandwidth of ADXL377)
const uint32_t SAMPLE_PERIOD_USEC   = 250;
const uint32_t TOGGLE_LED_ISR_COUNT = 500000 / SAMPLE_PERIOD_USEC;

const uint16_t COLLISION_ACCEL_THRESHOLD = 200;


struct calib_range
{
    uint16_t low;
    uint16_t high;
};

void dma_isr(void);
void timer_isr(void);

class AccelSampler;

namespace {
    AccelSampler * sampler;
    volatile bool dma_complete  = false;
    volatile uint32_t isr_count = 0;
}


template<typename T>
class RingBuffer
{
private:
    T * const buf;
    const size_t buf_len;
    size_t ofs;

public:
    RingBuffer(T * buf_, size_t buf_len_, const T & init_val) :
        buf     (buf_),
        buf_len (buf_len_),
        ofs     (0)
    {
        for (size_t i = 0; i < buf_len; i++) {
            buf[i] = init_val;
        }
    }

    void append(const T & val)
    {
        buf[ofs] = val;
        ofs = (ofs + 1) % buf_len;
    }

    size_t size(void) const
    {
        return buf_len;
    }

    // Element 0 is most recent, element <buf_len>-1 is oldest
    const T & operator[](size_t index) const
    {
        index %= buf_len;
        const size_t circ_index = (ofs + buf_len - index) % buf_len;
        return buf[circ_index];
    }
};


// Five-element median filter, using element repetition at the boundaries.
template<typename T>
T median5_filter_one(const RingBuffer<T> & input, size_t ofs)
{
    T data[5];
    data[2] = input[ofs];

    data[1] = ofs > 0 ? input[ofs-1] : input[ofs];
    data[0] = ofs > 1 ? input[ofs-2] : data[1];

    data[3] = ofs < input.size() - 1 ? input[ofs+1] : input[ofs];
    data[4] = ofs < input.size() - 2 ? input[ofs+2] : data[3];

    //std::sort(&data[0], &data[ARRAY_COUNT(data)]);
    return data[2];
}


template<typename T>
T median5_filter_latest(const RingBuffer<T> & input)
{
    return median5_filter_one(input, 2);
}


class AccelSampler
{
public:
    struct pin_assignments
    {
        uint8_t adc_x;
        uint8_t adc_y;
        uint8_t adc_z;
    };

private:
    const pin_assignments pins;

    // Trigger for ADC sampling interrupts
    HardwareTimer timer;

    // Buffer where sample data is stored (one 16-bit word per
    // sample, representing magnitude of acceleration vector)
    uint16_t * const sample_buf;
    const size_t sample_buf_len;

    // sample_buf is used as a circular buffer, so we need to
    // keep track of an offset as well as the number of elements
    // stored.
    size_t next_sample_ofs;
    size_t sample_count;

    // running count of post-trigger samples, or UINT32_MAX if
    // no collision has been detected
    uint32_t samples_after_trigger;
    const uint32_t max_samples_after_trigger;

    // DMA engine moves one sample's worth of data (x, y, z) into
    // this buffer
    uint16_t dma_buf[NUM_AXES] __attribute__((aligned));
    
    // Zero points recorded during calibration process
    uint16_t zero_points[NUM_AXES];

    // True if init() was invoked successfully
    bool is_initialized;

    friend void timer_isr(void);

    void power_up_adc(void)
    {
        // ADON transition from 0 to 1 wakes up the ADC from
        // Power Down mode.  Conversion does not begin until
        // 1 is written to ADON a second time.
        ADC1->regs->CR2 = ADC_CR2::DMA | ADC_CR2::ADON;
        adc_calibrate(ADC1);

        // datasheet requires 1us delay before conversion can begin
        delay(1);
    }

    void power_down_adc(void)
    {
        // Setting ADON to 0
        ADC1->regs->CR2 = ADC_CR2::DMA;
    }

    // Prime the DMA engine to carry out transfer of a single sample
    // (three ADC channels)
    //
    // Returns: 0 on success, DMA_TUBE_CFG_* on failure
    int setup_dma(void)
    {
        dma_tube_config cfg = {0};
        cfg.tube_src      = &ADC1->regs->DR;
        cfg.tube_src_size = DMA_SIZE_16BITS;
        cfg.tube_dst      = dma_buf;
        cfg.tube_dst_size = DMA_SIZE_16BITS;
        cfg.tube_nr_xfers = ARRAY_COUNT(dma_buf);
        cfg.tube_flags    = DMA_CFG_DST_INC | DMA_CFG_CMPLT_IE;
        cfg.tube_req_src  = DMA_REQ_SRC_ADC1;
        return dma_tube_cfg(DMA1, DMA_CH1, &cfg);
    }

    void start_adc_conversion(void)
    {
        setup_dma();
        dma_enable(DMA1, DMA_CH1);
        ADC1->regs->CR2 = ADC_CR2::DMA | ADC_CR2::ADON;
    }

    void wait_dma_complete(void)
    {
        while (!dma_complete);
        dma_complete = false;
    }

    // Acquire an ADC sample synchronously, causing the dma_buf to be updated.
    void acquire_sample_sync(void)
    {
        start_adc_conversion();
        wait_dma_complete();
    }

    void capture_calibration_samples(
            RingBuffer<uint16_t> ring_buffers[NUM_AXES],
            calib_range calib_ranges[NUM_AXES])
    {
        uint32_t sample_count = 0;
        const uint32_t start_time = millis();
        while (millis() - start_time < CALIBRATION_MSEC) {
            acquire_sample_sync();
            sample_count++;

            for (size_t j = 0; j < NUM_AXES; j++) {
                ring_buffers[j].append(dma_buf[j]);

                if (sample_count >= MEDIAN_FILTER_LEN) {
                    const uint16_t filtered_sample =
                        median5_filter_latest(ring_buffers[j]);
                    if (filtered_sample < calib_ranges[j].low) {
                        calib_ranges[j].low = filtered_sample;
                    }
                    if (filtered_sample > calib_ranges[j].high) {
                        calib_ranges[j].high = filtered_sample;
                    }
                }
            }

            if (sample_count % 5 == 0) {
                toggleLED();
            }
            delay(10);
        }
    }

    // Process a sample sitting in <dma_buf>.
    //
    // Returns: true if sampling should continue, false if sampling should cease
    //          because a collision event has been recorded
    bool process_sample(void)
    {
        // Store magnitude of acceleration vector, measured relative to
        // accelerometer zero point.  While I haven't counted cycles, the
        // expectation is that the for loop will finish executing well in
        // advance of the next timer interrupt; high-frequency sampling
        // would need to double-buffer.
        uint32_t sum_sq = 0;
        for (size_t i = 0; i < ARRAY_COUNT(dma_buf); i++) {
            const int32_t val =
                static_cast<int32_t>(dma_buf[i]) -
                static_cast<int32_t>(zero_points[i]);
            sum_sq += val * val;
        }
        sample_buf[next_sample_ofs] = util::sqrt_uint32(sum_sq);

        if (samples_after_trigger == std::numeric_limits<uint32_t>::max()) {
           if (sample_count == sample_buf_len &&
                   sample_buf[next_sample_ofs] > COLLISION_ACCEL_THRESHOLD) {
               samples_after_trigger = 0;
           }
        } else {
            samples_after_trigger++;
            if (samples_after_trigger == max_samples_after_trigger) {
                return false;
            }
        }

        next_sample_ofs++;
        if (sample_count < sample_buf_len) {
            sample_count++;
        }
        if (next_sample_ofs == sample_buf_len) {
            next_sample_ofs = 0;
        }

        return true;
    }


public:
    // Construct a new AccelSampler instance associated with the specified
    // Maple Mini pins.  Samples will be stored in the buffer provided by the caller.
    // The caller must invoke init() to complete the construction process.
    AccelSampler(const pin_assignments & pins_, uint16_t * buf, size_t buf_len) :
        pins                      (pins_),
        timer                     (2),   // General-purpose timer, no need for advanced timer
        sample_buf                (buf),
        sample_buf_len            (buf_len),
        next_sample_ofs           (0),
        sample_count              (0),
        samples_after_trigger     (std::numeric_limits<uint32_t>::max()),
        max_samples_after_trigger (buf_len * (TRIGGER_DIVISIONS - 1) / TRIGGER_DIVISIONS),
        is_initialized            (false)
    {
        for (size_t i = 0; i < ARRAY_COUNT(zero_points); i++) {
            zero_points[i] = ZERO_12BIT;
        }
    }

    bool init(void)
    {
        if (is_initialized) {
            return true;
        }

        // Must use ADC1 in order to access DMA capability.
        if (PIN_MAP[pins.adc_x].adc_device != ADC1 ||
                PIN_MAP[pins.adc_y].adc_device != ADC1 ||
                PIN_MAP[pins.adc_z].adc_device != ADC1) {
            SerialUSB.println("Accel(): invalid pin configuration.");
            return false;
        }

        const stm32_pin_info * pins_info[] = {
            &PIN_MAP[pins.adc_x],
            &PIN_MAP[pins.adc_y],
            &PIN_MAP[pins.adc_z]
        };

        for (size_t i = 0; i < ARRAY_COUNT(pins_info); i++) {
            const stm32_pin_info & info = *pins_info[i];
            adc_config_gpio(info.adc_device, info.gpio_device, info.gpio_bit);
        }

        adc_init(ADC1);

        // ADC clock is limited to 14MHz.  PCLK2 is running at 72MHz, so this
        // prescaler setting is within spec.
        adc_set_prescaler(ADC_PRE_PCLK2_DIV_8);

        // With ADC clock set to 9MHz, this setting is averaging samples over 18us.
        adc_set_sample_rate(ADC1, ADC_SMPR_239_5);

        // Set up a scan group for these three inputs
        const uint32_t SQR1_L = ARRAY_COUNT(pins_info) - 1;
        ADC1->regs->SQR1 = (SQR1_L << 20);
        ADC1->regs->SQR3 =
            (pins_info[0]->adc_channel << 0) |       // SQ1
            (pins_info[1]->adc_channel << 5) |       // SQ2
            (pins_info[2]->adc_channel << 10);       // SQ3

        // Configure scan mode
        ADC1->regs->CR1 = ADC_CR1::SCAN;

        // Set up DMA to move ADC conversion data into the buffer
        dma_init(DMA1);
        dma_attach_interrupt(DMA1, DMA_CH1, dma_isr);
        if (setup_dma() != 0) {
            return false;
        }

        // Start ADC in Power Down state
        power_down_adc();

        sampler        = this;
        is_initialized = true;
        return true;
    }

    // Perform continuous capture until a collision event is detected.
    // After the collision, 3/4 of a buffer is filled before stopping.
    //
    // Returns: offset into circular buffer where capture event recording
    //          begins
    size_t capture_event(void)
    {
        if (!is_initialized) {
            return 0;
        }

        power_up_adc();

        next_sample_ofs       = 0;
        sample_count          = 0;
        samples_after_trigger = std::numeric_limits<uint32_t>::max();

        // Configure a periodic up-counter with an interrupt
        // that fires once per period, at counter value 1
        timer.pause();
        timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
        timer.setPeriod(SAMPLE_PERIOD_USEC);
        timer.setCompare(TIMER_CH1, 1);
        timer.refresh();
        timer.attachInterrupt(TIMER_CH1, timer_isr);
        timer.resume();

        do {
            wait_dma_complete();
        } while (process_sample());
        timer.detachInterrupt(TIMER_CH1);

        power_down_adc();

        return next_sample_ofs;
    }


    // Compute the zero points of the accelerometer.
    //
    // The user should slowly roll the accelerometer through all values of all
    // three axes.  The zero points are determined by (1) using a median filter
    // to compute a min and max acceleration on each axis (due to acceleration
    // of gravity) and then (2) averaging the min and max to get an approximate
    // zero point.
    //
    // Returns: true if calibration is successful, false otherwise.
    bool calibrate(void)
    {
        if (!is_initialized) {
            return false;
        }

        power_up_adc();

        SerialUSB.println("Calibrating accelerometer...");
        calib_range calib_ranges[] = {
            {MAX_12BIT, MIN_12BIT},
            {MAX_12BIT, MIN_12BIT},
            {MAX_12BIT, MIN_12BIT}
        };

        uint16_t ring_buffer_storage[NUM_AXES][MEDIAN_FILTER_LEN];
        RingBuffer<uint16_t> ring_buffers[] = {
            RingBuffer<uint16_t>(ring_buffer_storage[0],
                    ARRAY_COUNT(ring_buffer_storage[0]), ZERO_12BIT),
            RingBuffer<uint16_t>(ring_buffer_storage[1],
                    ARRAY_COUNT(ring_buffer_storage[1]), ZERO_12BIT),
            RingBuffer<uint16_t>(ring_buffer_storage[2],
                    ARRAY_COUNT(ring_buffer_storage[2]), ZERO_12BIT)
        };

        capture_calibration_samples(ring_buffers, calib_ranges);

        bool calib_ok = true;
        for (size_t j = 0; j < ARRAY_COUNT(calib_ranges); j++) {
            if (calib_ranges[j].low > calib_ranges[j].high) {
                calib_ok = false;
            } else {
                // Overflow is not a concern... the range is only 12 bits.
                zero_points[j] = (calib_ranges[j].high + calib_ranges[j].low) / 2;
            }
        }

        if (!calib_ok) {
            SerialUSB.println("Calibration failure.");
            goto out;
        }

        SerialUSB.println("Calibration complete:");
        for (size_t j = 0; j < ARRAY_COUNT(zero_points); j++) {
            SerialUSB.print("    ");
            SerialUSB.print(j);
            SerialUSB.print(": ");
            SerialUSB.println(zero_points[j]);
        }

    out:
        power_down_adc(); 
        return calib_ok;
    }
};


void dma_isr(void)
{
    dma_disable(DMA1, DMA_CH1);
    dma_complete = true;
}


void timer_isr(void)
{
    sampler->start_adc_conversion();

    isr_count++;
    if (isr_count == TOGGLE_LED_ISR_COUNT) {
        isr_count = 0;
        toggleLED();
    }
}


#endif  // INCLUDE_GUARD_53da3484_c130_4662_a62e_0a6675871352

