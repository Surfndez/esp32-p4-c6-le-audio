#pragma once

// Cyberdeck - converts audio to the Tab5 codec format (48 kHz, stereo).
//
// Any input rate (8-96 kHz), mono or stereo. Polyphase filter with a windowed sinc
// (Kaiser window, ~80 dB attenuation where aliases would be audible): the rate ratio is a fraction
// L/M (44.1 kHz -> 48 kHz = 160/147, 24 kHz = 2/1); odd rates use the nearest fraction with a
// denominator up to kMaxPhases (tempo error < 3e-6, ~10 ms per hour).
// Earlier versions used linear interpolation: at 44.1 kHz -3.4 dB at 15 kHz and the 15 kHz image
// only -15 dB down at 18.9 kHz; at 24 kHz (film audio) the 12-24 kHz images were barely attenuated.
//
// Output is float (full scale = 1.0): the equaliser, limiter and dither follow (audio_dsp.h),
// and 48 kHz passes unchanged (int16 sample / 32768 - exact, bit for bit back).
//
// Pure logic - tested on the host (a host unit test).

#include <cstddef>
#include <cstdint>
#include <vector>

namespace app::media {

class Resampler {
public:
    static constexpr int kOutRate   = 48000;
    static constexpr int kMaxPhases = 640;  // 11,025 kHz -> 48 kHz = 640/147
    static constexpr int kMaxTaps   = 64;

    // The filter is designed only when the rate changes (same rate - only a state reset).
    void configure(int rate, int channels);
    // `frames` input frames (interleaved channels) -> appends 48 kHz stereo frames (L, R) to out.
    void process(const int16_t* in, size_t frames, std::vector<float>& out);
    // Same for float input (full scale 1.0) - after a tempo change (timestretch.h).
    void process(const float* in, size_t frames, std::vector<float>& out);
    void reset();

    int rate() const
    {
        return rate_;
    }
    int channels() const
    {
        return channels_;
    }
    bool passthrough() const
    {
        return rate_ == kOutRate;
    }
    int taps() const
    {
        return taps_;
    }
    int phases() const
    {
        return L_;
    }
    int decim() const
    {
        return M_;
    }

private:
    template <class T>
    void run(const T* in, size_t frames, float scale, std::vector<float>& out);
    void design();

    int rate_     = 0;
    int channels_ = 1;
    int L_ = 1, M_ = 1;    // L/M output samples per input sample
    int taps_     = 0;     // coefficients per phase
    std::vector<float> coef_;  // L_ x taps_, in window order (from the oldest sample)
    std::vector<float> hist_;  // 2 x taps_ per channel (window without modulo), channels one after another
    int w_   = 0;              // write position in the history
    int acc_ = 0;              // phase of the next output sample (0..L_-1) relative to the current input sample
};

}  // namespace app::media
