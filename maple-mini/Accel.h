#ifndef INCLUDE_GUARD_53da3484_c130_4662_a62e_0a6675871352
#define INCLUDE_GUARD_53da3484_c130_4662_a62e_0a6675871352


#include <algorithm>
#include <stdint.h>
#include <wirish/wirish.h>
#include <libmaple/adc.h>
#include <libmaple/dma.h>
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


class AccelSampler;
void dma_isr(void);

namespace {
    AccelSampler * sampler_instance = NULL;
    volatile bool dma_complete      = false;
}


const size_t NUM_AXES = 3;

const uint32_t CALIBRATION_MSEC = 10000;

const uint16_t MIN_12BIT  = 0;
const uint16_t ZERO_12BIT = 2048;
const uint16_t MAX_12BIT  = 4096;


struct calib_range
{
    uint16_t low;
    uint16_t high;
};


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

    // Buffer where sample data is stored (one 16-bit word per
    // sample, representing magnitude of acceleration vector)
    uint16_t * const sample_buf;
    const size_t sample_buf_len;

    // sample_buf is used as a circular buffer, so we need to
    // keep track of an offset as well as the number of elements
    // stored.
    size_t sample_buf_start;
    size_t sample_count;

    // DMA engine moves one sample's worth of data (x, y, z) into
    // this buffer
    uint16_t dma_buf[NUM_AXES] __attribute__((aligned));
    
    // Zero points recorded during calibration process
    uint16_t zero_points[NUM_AXES];

    bool is_initialized;

    friend void dma_isr(void);

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

public:
    // Construct a new AccelSampler instance associated with the specified
    // Maple Mini pins.  Samples will be stored in the buffer provided by the caller.
    // The caller must invoke init() to complete the construction process.
    AccelSampler(const pin_assignments & pins_, uint16_t * buf, size_t buf_len) :
        pins             (pins_),
        sample_buf       (buf),
        sample_buf_len   (buf_len),
        sample_buf_start (0),
        sample_count     (0),
        is_initialized   (false)
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

        sampler_instance = this;
        is_initialized   = true;
        return true;
    }

    void power_up_adc(void)
    {
        if (!is_initialized) {
            return;
        }

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
        if (!is_initialized) {
            return;
        }

        // Setting ADON to 0
        ADC1->regs->CR2 = ADC_CR2::DMA;
    }

    void start_adc_conversion(void)
    {
        if (!is_initialized) {
            return;
        }

        setup_dma();
        dma_enable(DMA1, DMA_CH1);
        ADC1->regs->CR2 = ADC_CR2::DMA | ADC_CR2::ADON;
    }


    void acquire_raw_sample(uint16_t output[NUM_AXES])
    {
        if (!is_initialized) {
            return;
        }
        start_adc_conversion();
        while (!dma_complete);
        for (size_t i = 0; i < NUM_AXES; i++) {
            output[i] = dma_buf[i];
        }
        dma_complete = false;
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
        SerialUSB.println("Calibrating accelerometer...");
        calib_range calib_ranges[] = {
            {MAX_12BIT, MIN_12BIT},
            {MAX_12BIT, MIN_12BIT},
            {MAX_12BIT, MIN_12BIT}
        };

        const size_t WINDOW_SIZE = 5;
        uint16_t ring_buffer_storage[NUM_AXES][WINDOW_SIZE];
        RingBuffer<uint16_t> ring_buffers[] = {
            RingBuffer<uint16_t>(ring_buffer_storage[0],
                    ARRAY_COUNT(ring_buffer_storage[0]), ZERO_12BIT),
            RingBuffer<uint16_t>(ring_buffer_storage[1],
                    ARRAY_COUNT(ring_buffer_storage[1]), ZERO_12BIT),
            RingBuffer<uint16_t>(ring_buffer_storage[2],
                    ARRAY_COUNT(ring_buffer_storage[2]), ZERO_12BIT)
        };

        uint32_t sample_count = 0;
        const uint32_t start_time = millis();
        while (millis() - start_time < CALIBRATION_MSEC) {
            uint16_t samples[NUM_AXES];
            acquire_raw_sample(samples);
            sample_count++;

            for (size_t j = 0; j < NUM_AXES; j++) {
                ring_buffers[j].append(samples[j]);

                if (sample_count >= WINDOW_SIZE) {
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
        SerialUSB.println("Samples captured");

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
            return false;
        }

        SerialUSB.println("Calibration complete:");
        for (size_t j = 0; j < ARRAY_COUNT(zero_points); j++) {
            SerialUSB.print("    ");
            SerialUSB.print(j);
            SerialUSB.print(": ");
            SerialUSB.println(zero_points[j]);
        }

        return true;
    }
};


void dma_isr(void)
{
    const size_t ofs =
        (sampler_instance->sample_buf_start + sampler_instance->sample_count) %
        sampler_instance->sample_buf_len;

    // TODO: pythagorean distance
    sampler_instance->sample_buf[ofs] = sampler_instance->dma_buf[0];

    if (sampler_instance->sample_count < sampler_instance->sample_buf_len) {
        sampler_instance->sample_count++;
    } else {
        sampler_instance->sample_buf_start =
            (sampler_instance->sample_buf_start + 1) % sampler_instance->sample_count;
    }

    dma_disable(DMA1, DMA_CH1);
    dma_complete = true;
}


#endif  // INCLUDE_GUARD_53da3484_c130_4662_a62e_0a6675871352

