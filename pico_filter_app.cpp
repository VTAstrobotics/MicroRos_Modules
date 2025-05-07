// pico_filter_app.cpp
#include "pico_filter_app.hpp"

//--- DF2_Filter implementation ---
// Default constructor: uses 10 kHz cutoff fixed-point coefficients
DF2_Filter::DF2_Filter()
    : DF2_Filter(
        // a1, a2, a3 (Q15)
        static_cast<int32_t>(-2.4986 * FIX_SCALE - 0.5),
        static_cast<int32_t>( 2.1153 * FIX_SCALE + 0.5),
        static_cast<int32_t>(-0.6041 * FIX_SCALE - 0.5),
        // b0, b1, b2, b3
        static_cast<int32_t>( 0.0016 * FIX_SCALE + 0.5),
        static_cast<int32_t>( 0.0047 * FIX_SCALE + 0.5),
        static_cast<int32_t>( 0.0047 * FIX_SCALE + 0.5),
        static_cast<int32_t>( 0.0016 * FIX_SCALE + 0.5)
    )
{
}

DF2_Filter::DF2_Filter(int32_t a1, int32_t a2, int32_t a3,
                       int32_t b0, int32_t b1, int32_t b2, int32_t b3)
  : a1_(a1), a2_(a2), a3_(a3), b0_(b0), b1_(b1), b2_(b2), b3_(b3)
{
    reset();
}

void DF2_Filter::reset() {
    w_[0] = w_[1] = w_[2] = 0;
}

int32_t DF2_Filter::process(int32_t x) {
    // Direct Form II: compute new state
    int64_t wn = (int64_t)x * FIX_SCALE;
    wn -= (int64_t)a1_ * w_[0];
    wn -= (int64_t)a2_ * w_[1];
    wn -= (int64_t)a3_ * w_[2];
    wn = (wn + (1 << (Q - 1))) >> Q;

    // Compute output y[n]
    int64_t acc = 0;
    acc += (int64_t)b0_ * wn;
    acc += (int64_t)b1_ * w_[0];
    acc += (int64_t)b2_ * w_[1];
    acc += (int64_t)b3_ * w_[2];
    int32_t y = static_cast<int32_t>((acc + (1 << (Q - 1))) >> Q);

    // Shift state buffer
    w_[2] = w_[1];
    w_[1] = w_[0];
    w_[0] = static_cast<int32_t>(wn);

    return y;
}

//--- PicoFilterApp implementation ---
PicoFilterApp::PicoFilterApp(uint adc_gpio, uint32_t sample_period_us)
  : sample_period_us_(sample_period_us),
    filter_()   // default 10 kHz
{
    // Initialize ADC
    adc_init();
    adc_gpio_init(adc_gpio);
    adc_select_input(0);

    // // Initialize GPIO pins 0..7
    // for (int i = 0; i < 8; ++i) {
    //     gpio_init(i);
    //     gpio_set_dir(i, GPIO_OUT);
    // }
}

void PicoFilterApp::run() {
    while (true) {
        uint16_t raw  = adc_read();
        int32_t  x    = raw << 4;

        int32_t y_q15 = filter_.process(x);

        // Revert to 8-bit shift to match original behavior (0–255 range)
        int32_t out_val = y_q15 >> 8;
        if (out_val < 0)   out_val = 0;
        if (out_val > 255) out_val = 255;

        uint32_t mask = static_cast<uint32_t>(out_val);
        // Write to GPIO pins 0..7 in one masked operation (exact original bit outputs)
        gpio_put_masked(0xFF, mask);

        busy_wait_us(sample_period_us_);
    }
}
