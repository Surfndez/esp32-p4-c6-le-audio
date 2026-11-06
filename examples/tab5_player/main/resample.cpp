#include "resample.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace app::media {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Bessel I0 function (series) - Kaiser window. In float: the P4 has no hardware double (the filter is designed
// on every new rate).
float besselI0(float x)
{
    float sum = 1, term = 1;
    const float q = x * x / 4;
    for (int k = 1; k < 40; k++) {
        term *= q / ((float)k * k);
        sum += term;
        if (term < sum * 1e-8f) {
            break;
        }
    }
    return sum;
}

}  // namespace

void Resampler::configure(int rate, int channels)
{
    rate = std::clamp(rate, 8000, 96000);
    channels = channels >= 2 ? 2 : 1;
    if (rate != rate_ || channels != channels_ || (rate != kOutRate && coef_.empty())) {
        rate_     = rate;
        channels_ = channels;
        design();
    }
    reset();
}

void Resampler::design()
{
    coef_.clear();
    hist_.clear();
    taps_ = 0;
    L_ = M_ = 1;
    if (rate_ == kOutRate) {
        return;  // 48 kHz - no filter
    }
    const int g = std::gcd(rate_, kOutRate);
    L_          = kOutRate / g;
    M_          = rate_ / g;
    if (L_ > kMaxPhases) {
        // Nearest fraction with a denominator up to kMaxPhases (e.g. 44101 Hz) - tempo error below 3e-6.
        const double want = (double)rate_ / kOutRate;
        double best       = 1e9;
        for (int l = 1; l <= kMaxPhases; l++) {
            const int m = (int)std::lround(want * l);
            if (m < 1) {
                continue;
            }
            const double err = std::fabs((double)m / l - want);
            if (err < best - 1e-15) {
                best = err;
                L_   = l;
                M_   = m;
            }
        }
    }
    // Passband up to 20 kHz (or 0.45 fs); stopband where an image/alias would be audible:
    // when raising the rate, the image of the band (fs - f) below 20 kHz must be cut, and above 24 kHz
    // it folds to 48k - x (audible only from x > 28 kHz). Always fp < fstop.
    double fp = 0, fstop = 0;
    if (rate_ < kOutRate) {
        fp               = std::min(20000.0, 0.45 * rate_);
        const double img = rate_ - fp;
        fstop            = img >= 20000.0 ? std::max(img, 28000.0) : img;
    } else {
        fp    = 20000.0;
        fstop = 28000.0;
    }
    const double kAtten = 80.0;
    const float beta    = (float)(0.1102 * (kAtten - 8.7));
    const double dw     = 2 * kPi * (fstop - fp) / rate_;
    int n               = (int)std::ceil((kAtten - 8.0) / (2.285 * dw));
    n                   = std::clamp((n + 1) & ~1, 16, kMaxTaps);
    taps_               = n;
    const float fcn     = (float)((fp + fstop) / 2 / ((double)L_ * rate_));  // cutoff relative to L * fs
    const float len     = (float)n * L_;
    const float center  = (len - 1) / 2;
    const float i0b     = besselI0(beta);
    const float pi      = (float)kPi;
    coef_.assign((size_t)L_ * n, 0.0f);
    std::vector<float> ph((size_t)n);
    for (int p = 0; p < L_; p++) {
        float sum = 0;
        for (int j = 0; j < n; j++) {
            const int i   = (n - 1 - j) * L_ + p;  // index in the prototype; the window runs from the oldest sample
            const float t = (float)i - center;
            const float x = 2 * fcn * t;
            const float s = std::fabs(x) < 1e-7f ? 1.0f : std::sin(pi * x) / (pi * x);
            const float r = t / (len / 2);
            const float k = std::fabs(r) >= 1 ? 0.0f : besselI0(beta * std::sqrt(1 - r * r)) / i0b;
            ph[(size_t)j] = 2 * fcn * s * k;
            sum += ph[(size_t)j];
        }
        // Each phase has gain 1 for DC - no DC ripple between phases.
        for (int j = 0; j < n; j++) {
            coef_[(size_t)p * n + j] = sum != 0 ? ph[(size_t)j] / sum : 0.0f;
        }
    }
    hist_.assign((size_t)2 * n * 2, 0.0f);
}

void Resampler::reset()
{
    std::fill(hist_.begin(), hist_.end(), 0.0f);
    w_   = 0;
    acc_ = 0;
}

template <class T>
void Resampler::run(const T* in, size_t frames, float scale, std::vector<float>& out)
{
    if (frames == 0) {
        return;
    }
    if (rate_ == 0) {
        configure(16000, 1);
    }
    const int ch = channels_;
    if (rate_ == kOutRate) {
        out.reserve(out.size() + frames * 2);
        for (size_t f = 0; f < frames; f++) {
            const float l = (float)in[f * ch] * scale;
            out.push_back(l);
            out.push_back(ch == 2 ? (float)in[f * ch + 1] * scale : l);
        }
        return;
    }
    const int n  = taps_;
    float* hl    = hist_.data();
    float* hr    = hist_.data() + 2 * n;
    out.reserve(out.size() + (size_t)((uint64_t)frames * L_ / M_ + 2) * 2);
    for (size_t f = 0; f < frames; f++) {
        w_ = w_ + 1 == n ? 0 : w_ + 1;
        const float l = (float)in[f * ch] * scale;
        hl[w_] = hl[w_ + n] = l;
        if (ch == 2) {
            const float r = (float)in[f * ch + 1] * scale;
            hr[w_] = hr[w_ + n] = r;
        }
        // Window: hist[w_+1 .. w_+n] = from the oldest to the current sample.
        const float* xl = hl + w_ + 1;
        const float* xr = hr + w_ + 1;
        while (acc_ < L_) {
            const float* c = coef_.data() + (size_t)acc_ * n;
            float sl = 0, sr = 0;
            if (ch == 2) {
                for (int j = 0; j < n; j++) {
                    sl += c[j] * xl[j];
                    sr += c[j] * xr[j];
                }
            } else {
                for (int j = 0; j < n; j++) {
                    sl += c[j] * xl[j];
                }
                sr = sl;
            }
            out.push_back(sl);
            out.push_back(sr);
            acc_ += M_;
        }
        acc_ -= L_;
    }
}

void Resampler::process(const int16_t* in, size_t frames, std::vector<float>& out)
{
    run(in, frames, 1.0f / 32768.0f, out);
}

void Resampler::process(const float* in, size_t frames, std::vector<float>& out)
{
    run(in, frames, 1.0f, out);
}

}  // namespace app::media
