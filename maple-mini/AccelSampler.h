/******************************************************************************
 * AccelSampler
 *
 * Copyright (C) 2013 Paul Pelzl
 * Simplified BSD license.  See README.md for details.
 *
 * Implementation of ADXL377 sampling via STM32 ADC.
 ******************************************************************************/
#ifndef INCLUDE_GUARD_53da3484_c130_4662_a62e_0a6675871352
#define INCLUDE_GUARD_53da3484_c130_4662_a62e_0a6675871352


#include <stdint.h>
#include <stddef.h>

#include <wirish/wirish.h>
// Shame on you, libmaple...
#undef min
#undef max

namespace accel {

const size_t NUM_AXES = 3;

struct calib_range;
template<typename T> class RingBuffer;

namespace detail {
    void timer_isr(void);
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

    friend void detail::timer_isr(void);

    void power_up_adc(void);
    void power_down_adc(void);
    int setup_dma(void);
    void start_adc_conversion(void);
    void wait_dma_complete(void);
    void acquire_sample_sync(void);
    void capture_calibration_samples(
            RingBuffer<uint16_t> ring_buffers[NUM_AXES],
            calib_range calib_ranges[NUM_AXES]);
    bool process_sample(void);


public:
    // Construct a new AccelSampler instance associated with the specified
    // Maple Mini pins.  Samples will be stored in the buffer provided by the caller.
    // The caller must invoke init() to complete the construction process.
    AccelSampler(const pin_assignments & pins_, uint16_t * buf, size_t buf_len);

    // Returns: true if successfully initialized, false otherwise.
    bool init(void);

    // Perform continuous capture until a collision event is detected.
    // After the collision, 3/4 of a buffer is filled before stopping.
    //
    // Returns: offset into circular buffer where capture event recording
    //          begins
    size_t capture_event(void);

    // Compute the zero points of the accelerometer.
    //
    // The user should slowly roll the accelerometer through all values of all
    // three axes.  The zero points are determined by (1) using a median filter
    // to compute a min and max acceleration on each axis (due to acceleration
    // of gravity) and then (2) averaging the min and max to get an approximate
    // zero point.
    //
    // Returns: true if calibration is successful, false otherwise.
    bool calibrate(void);
};

}   // namespace accel

#endif  // INCLUDE_GUARD_53da3484_c130_4662_a62e_0a6675871352

