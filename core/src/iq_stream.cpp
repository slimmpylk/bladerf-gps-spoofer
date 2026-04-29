// ─────────────────────────────────────────────────────────────────────────────
// iq_stream.cpp — Multi-Channel I/Q Combiner and bladeRF Sample Scaler
// ─────────────────────────────────────────────────────────────────────────────

#include "iq_stream.hpp"
#include <algorithm>    // std::max_element
#include <cmath>        // std::abs, std::round
#include <numeric>      // std::transform
#include <stdexcept>    // std::invalid_argument, std::runtime_error

namespace gps {

// ─────────────────────────────────────────────────────────────────────────────
// sum_channels()
//
// Element-wise sum of N channel buffers.
// If channel 1 has [a0, a1, a2...] and channel 2 has [b0, b1, b2...]
// the result is [a0+b0, a1+b1, a2+b2...].
//
// WHY check sizes match?
//   If one channel has 2048 samples and another has 2047, we'd either
//   read out of bounds or silently drop samples. Both are worse than
//   failing loudly here.
// ─────────────────────────────────────────────────────────────────────────────
std::vector<std::complex<float>>
sum_channels(const std::vector<std::vector<std::complex<float>>>& channels) {
    if (channels.empty()) return {};

    const size_t n = channels[0].size();

    // Validate all channels have the same length
    for (size_t ch = 1; ch < channels.size(); ++ch) {
        if (channels[ch].size() != n) {
            throw std::invalid_argument(
                "sum_channels: channel " + std::to_string(ch) +
                " has " + std::to_string(channels[ch].size()) +
                " samples, expected " + std::to_string(n)
            );
        }
    }

    // Start with zeros
    std::vector<std::complex<float>> result(n, {0.0f, 0.0f});

    // Sum all channels element-wise
    // WHY a range-for over channels then a manual index loop?
    //   The outer loop iterates channels (usually 8-10).
    //   The inner loop iterates samples (2048 per epoch).
    //   This order is cache-friendly: we read each channel's data
    //   sequentially, which is optimal for CPU cache prefetching.
    for (const auto& ch : channels) {
        for (size_t i = 0; i < n; ++i) {
            result[i] += ch[i];
        }
    }

    return result;
}


// ─────────────────────────────────────────────────────────────────────────────
// find_peak_magnitude()
//
// Find the largest |sample| in the buffer. Used to set the scale factor
// so the loudest sample fits just within the SC16Q11 range.
//
// WHY not just use the theoretical maximum (num_channels × amplitude)?
//   The theoretical max assumes all channels peak simultaneously with
//   aligned phases — a worst case that rarely occurs in practice.
//   The actual peak magnitude is usually 2-4× lower than the worst case.
//   Scaling to the actual peak gives better SNR (uses more of the DAC range)
//   while still preventing clipping.
// ─────────────────────────────────────────────────────────────────────────────
float find_peak_magnitude(const std::vector<std::complex<float>>& samples) {
    float peak = 0.0f;
    for (const auto& s : samples) {
        // std::abs on complex<float> returns the magnitude: √(i² + q²)
        const float mag = std::abs(s);
        if (mag > peak) peak = mag;
    }
    // Return at least 1.0 to avoid division by zero when all samples are 0
    return std::max(peak, 1.0f);
}


// ─────────────────────────────────────────────────────────────────────────────
// float_to_sc16q11()
//
// Converts floating-point I/Q samples to the bladeRF's SC16Q11 format.
//
// SC16Q11 means:
//   - Signed 16-bit integer (int16_t)
//   - Q11 fixed-point: 11 fractional bits
//   - Range: [-2048, 2047] maps to [-1.0, +1.0] in the normalised domain
//
// Conversion: integer = clamp(round(float × scale_factor), -2048, +2047)
//
// WHY clamp after rounding?
//   Floating-point rounding can occasionally produce a value just above
//   2047.5 which rounds to 2048 — one step outside int16's useful range.
//   Clamping prevents this from wrapping around to -32768.
// ─────────────────────────────────────────────────────────────────────────────
SC16Q11Buffer float_to_sc16q11(
    const std::vector<std::complex<float>>& samples,
    float scale_factor)
{
    SC16Q11Buffer buffer;
    buffer.reserve(samples.size());

    for (const auto& s : samples) {
        // Scale and round
        const float fi = s.real() * scale_factor;
        const float fq = s.imag() * scale_factor;

        // Clamp to valid SC16Q11 range [-2048, 2047]
        // WHY not just static_cast? If fi = 2048.4f, casting directly gives
        // 2048, which overflows int16_t to -32768. Clamping first is safe.
        const int32_t ri = static_cast<int32_t>(std::round(fi));
        const int32_t rq = static_cast<int32_t>(std::round(fq));

        SC16Q11Sample sample;
        sample.i = static_cast<int16_t>(std::max(-2048, std::min(2047, ri)));
        sample.q = static_cast<int16_t>(std::max(-2048, std::min(2047, rq)));

        buffer.push_back(sample);
    }

    return buffer;
}


// ─────────────────────────────────────────────────────────────────────────────
// IQStream class implementation
// ─────────────────────────────────────────────────────────────────────────────

IQStream::IQStream(int num_channels, float headroom_factor)
    : num_channels_(num_channels)
    , headroom_factor_(headroom_factor)
{
    // Pre-allocate the accumulator for one epoch (2048 samples)
    // Starting with zeros — add_channel_epoch() adds to this.
    accumulator_.assign(2048, {0.0f, 0.0f});
}

void IQStream::add_channel_epoch(
    const std::vector<std::complex<float>>& samples)
{
    // First call sets the expected size; subsequent calls must match
    if (samples.size() != accumulator_.size()) {
        throw std::invalid_argument(
            "add_channel_epoch: got " + std::to_string(samples.size()) +
            " samples, expected " + std::to_string(accumulator_.size())
        );
    }

    // Accumulate: element-wise addition
    for (size_t i = 0; i < samples.size(); ++i) {
        accumulator_[i] += samples[i];
    }
}

SC16Q11Buffer IQStream::finalise() {
    // Find the peak in the accumulated sum
    const float peak = find_peak_magnitude(accumulator_);

    // Update the running peak tracker.
    // WHY track peak across epochs with a decay?
    //   The peak can vary between epochs (satellite geometry changes).
    //   Using a long-term average prevents gain hunting (oscillating scale).
    //   Simple approach: take the max of current peak and 90% of previous.
    //   This lets gain decrease slowly (avoiding sudden amplitude drops)
    //   while responding quickly to increases.
    peak_magnitude_ = std::max(peak, peak_magnitude_ * 0.9f);

    // Compute scale factor: target output level = headroom × 2047
    // scale_factor × peak_magnitude = headroom × 2047
    const float scale_factor = (headroom_factor_ * 2047.0f) / peak_magnitude_;

    return float_to_sc16q11(accumulator_, scale_factor);
}

void IQStream::reset() {
    // Clear accumulator for the next epoch
    // WHY assign() instead of clear() + resize()?
    //   assign() sets all elements to zero in one call.
    //   clear() + resize() leaves values uninitialised.
    std::fill(accumulator_.begin(), accumulator_.end(),
              std::complex<float>{0.0f, 0.0f});
}

int IQStream::sample_count() const {
    return static_cast<int>(accumulator_.size());
}

} // namespace gps