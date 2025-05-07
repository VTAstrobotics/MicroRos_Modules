// pico_filter_app.h
#ifndef PICO_FILTER_APP_H
#define PICO_FILTER_APP_H

#include <cstdint>
#include "pico/stdlib.h"
#include "hardware/adc.h"

// Fixed-point configuration (Q15 format)
static constexpr int Q = 15;
static constexpr int32_t FIX_SCALE = 1 << Q;

// Class implementing a single biquad in Direct Form II, Q15 fixed-point
class DF2_Filter {
public:
    DF2_Filter(); //default constructor with 10 kHz cutoff
    // Constructor: takes fixed-point coefficients a[1..3], b[0..3]
    DF2_Filter(int32_t a1, int32_t a2, int32_t a3,
        int32_t b0, int32_t b1, int32_t b2, int32_t b3);

    // Reset state buffer to zero
    void reset();

    // Process one sample x (raw integer, e.g. ADC << shift), returns filtered y in Q15
    int32_t process(int32_t x);

private:
    int32_t a1_ = -2.4986, a2_ = 2.1153, a3_ = -0.6041, b0_ = 0.0016, b1_ = 0.0047, b2_ = 0.0047, b3_ = 0.0016;
    int32_t w_[3];
};

// Application class handling ADC sampling, filtering, and GPIO output
class PicoFilterApp {
public:
    // adc_gpio: GPIO pin for ADC; sample_period_us: interval in microseconds
    PicoFilterApp(uint adc_gpio, uint32_t sample_period_us);

    // Start the processing loop
    void run();

private:
    uint32_t sample_period_us_;
    DF2_Filter filter_;
};

#endif // PICO_FILTER_APP_H
