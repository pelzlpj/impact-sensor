#ifndef INCLUDE_GUARD_53da3484_c130_4662_a62e_0a6675871352
#define INCLUDE_GUARD_53da3484_c130_4662_a62e_0a6675871352

#include <stddef.h>
#include <libmaple/adc.h>
#include <libmaple/dma.h>

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


class AccelSampler
{
public:
    struct PinAssignments
    {
        uint8_t adc_x;
        uint8_t adc_y;
        uint8_t adc_z;
    };

private:
    const PinAssignments pins;
    const adc_dev * const adc;

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
    uint16_t dma_buf[3] __attribute__((aligned));
    
    bool is_initialized;

    friend void dma_isr(void);

    // Prime the DMA engine to carry out transfer of a single sample
    // (three ADC channels)
    //
    // Returns: 0 on success, DMA_TUBE_CFG_* on failure
    int setup_dma(void)
    {
        dma_tube_config cfg = {0};
        cfg.tube_src      = &adc->regs->DR;
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
    AccelSampler(const PinAssignments & pins_, uint16_t * buf, size_t buf_len) :
        pins             (pins_),
        adc              (PIN_MAP[pins.adc_x].adc_device),
        sample_buf       (buf),
        sample_buf_len   (buf_len),
        sample_buf_start (0),
        sample_count     (0),
        is_initialized   (false)
    {}

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

        // Set up a scan group for these three inputs
        const uint32_t SQR1_L = ARRAY_COUNT(pins_info) - 1;
        adc->regs->SQR1 = (SQR1_L << 20);
        adc->regs->SQR3 =
            (pins_info[0]->adc_channel << 0) |       // SQ1
            (pins_info[1]->adc_channel << 5) |       // SQ2
            (pins_info[2]->adc_channel << 10);       // SQ3

        // Configure scan mode
        adc->regs->CR1 = ADC_CR1::SCAN;

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
        adc->regs->CR2 = ADC_CR2::DMA | ADC_CR2::ADON;

        // datasheet requires 1us delay before conversion can begin
        delay(1);
    }

    void power_down_adc(void)
    {
        if (!is_initialized) {
            return;
        }

        // Setting ADON to 0
        adc->regs->CR2 = ADC_CR2::DMA;
    }

    void start_adc_conversion(void)
    {
        if (!is_initialized) {
            return;
        }

        setup_dma();
        dma_enable(DMA1, DMA_CH1);
        adc->regs->CR2 = ADC_CR2::DMA | ADC_CR2::ADON;
    }

    bool read_sample(uint16_t * out_buf)
    {
        if (!is_initialized) {
            return false;
        }

        if (dma_complete) {
            for (size_t i = 0; i < ARRAY_COUNT(dma_buf); i++) {
                *out_buf++ = dma_buf[i];
            }
            dma_complete = false;
            return true;
        }
        return false;
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

