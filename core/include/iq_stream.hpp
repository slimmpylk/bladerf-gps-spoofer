#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// iq_stream.hpp — Multi-Channel I/Q Combiner and bladeRF Sample Scaler
//
// WHAT THIS DOES:
//   This is the last step before the bladeRF hardware.
//   It takes the per-channel complex<float> samples from the modulator,
//   sums all satellite channels together, and scales the result to the
//   SC16Q11 integer format the bladeRF DAC expects.
//
// THE PIPELINE:
//
//   Channel 1: [I/Q float samples] ─┐
//   Channel 2: [I/Q float samples] ─┤
//   Channel 3: [I/Q float samples] ─┼─► SUM ─► SCALE ─► SC16Q11 ─► bladeRF
//   ...                             │
//   Channel N: [I/Q float samples] ─┘
//
// WHY SUM THE CHANNELS?
//   GPS uses CDMA — all satellites transmit on the same frequency.
//   A real receiver receives all of them simultaneously and separates them
//   by correlating with each PRN's C/A code. We must simulate this by
//   generating all satellites' signals and adding them together before
//   sending to the bladeRF, just as they would arrive at a real antenna.
//
// SC16Q11 FORMAT:
//   The bladeRF expects samples as pairs of signed 16-bit integers.
//   The "Q11" means 11 fractional bits: the value range [-2048, 2047]
//   represents the normalised range [-1.0, +1.0].
//   We scale our float sum to fit within this range with headroom.
//
// REFERENCE: Nuand bladeRF wiki — SC16Q11 sample format
//            libbladeRF API docs — bladerf_sync_tx()
// ─────────────────────────────────────────────────────────────────────────────

#include <complex>
#include <cstdint>
#include <vector>

namespace gps {

// ─────────────────────────────────────────────────────────────────────────────
// SC16Q11Sample — one bladeRF sample
//
// Two int16_t packed together: [I, Q]
// The bladeRF's DMA buffer is just an array of these.
//
// WHY a struct and not std::pair<int16_t, int16_t>?
//   std::pair has .first and .second — unclear which is I and which is Q.
//   A named struct is self-documenting and matches the libbladeRF layout.
//
// MEMORY LAYOUT:
//   The bladeRF expects I immediately followed by Q in memory.
//   C++ guarantees struct members are laid out in order with no padding
//   for types that match their alignment — int16_t pairs are always packed.
//   So &sample.i gives a pointer to the start of a 4-byte bladeRF sample.
// ─────────────────────────────────────────────────────────────────────────────
struct SC16Q11Sample {
    int16_t i = 0;   // in-phase component,    range [-2048, 2047]
    int16_t q = 0;   // quadrature component,  range [-2048, 2047]
};

// The complete buffer type for a block of bladeRF samples
using SC16Q11Buffer = std::vector<SC16Q11Sample>;


// ─────────────────────────────────────────────────────────────────────────────
// IQStream — manages the multi-channel sum and conversion pipeline
//
// WHY a class here instead of free functions?
//   This is the first component that has MUTABLE STATE that persists
//   across calls: the peak amplitude tracker for auto-gain control.
//   When you have persistent state that belongs to a specific object,
//   a class is the right tool. Free functions are for stateless operations.
//
// USAGE:
//   IQStream stream(8);                          // 8 satellite channels
//   stream.add_channel_epoch(channel1_samples);  // add each channel
//   stream.add_channel_epoch(channel2_samples);
//   ...
//   auto buffer = stream.finalise();             // get SC16Q11Buffer
//   stream.reset();                              // prepare for next epoch
// ─────────────────────────────────────────────────────────────────────────────

class IQStream {
public:
    // headroom_factor: scale the sum to this fraction of full scale.
    // 0.6 = use 60% of the DAC range, keeping 40% headroom for peaks.
    // WHY 60%? When N channels sum with aligned phases (worst case),
    // the peak amplitude is N times larger than a single channel.
    // With 8 channels at amplitude 1.0, peak = 8.0. At 60% headroom:
    // max_output = 0.6 × 2047 / 8 ≈ 154 per channel — safe from clipping.
    explicit IQStream(int num_channels, float headroom_factor = 0.6f);

    // Add one epoch (2048 samples) of one satellite channel.
    // Call this once per channel per epoch before calling finalise().
    void add_channel_epoch(const std::vector<std::complex<float>>& samples);

    // Sum all accumulated channel epochs, scale to SC16Q11, return buffer.
    // After calling this, you MUST call reset() before the next epoch.
    [[nodiscard]] SC16Q11Buffer finalise();

    // Clear the accumulator for the next epoch.
    // Does NOT reset the gain — gain adapts across epochs.
    void reset();

    // How many samples are currently accumulated
    [[nodiscard]] int sample_count() const;

private:
    int   num_channels_;
    float headroom_factor_;

    // Accumulator: sum of all channels, float precision
    // WHY float and not int? Summing 8 channels of int16 could overflow.
    // float has 23 mantissa bits — enough for summing ~1000 channels without loss.
    std::vector<std::complex<float>> accumulator_;

    // Peak magnitude seen in the accumulator (for auto-scale)
    float peak_magnitude_ = 1.0f;
};


// ─────────────────────────────────────────────────────────────────────────────
// Free functions (stateless, testable in isolation)
// ─────────────────────────────────────────────────────────────────────────────

// Sum a vector of per-channel sample buffers into one combined buffer.
// All buffers must have the same length.
[[nodiscard]] std::vector<std::complex<float>>
sum_channels(const std::vector<std::vector<std::complex<float>>>& channels);

// Convert float I/Q samples to SC16Q11 integers.
// scale_factor: multiplier applied before rounding.
//   Typically: (headroom × 2047) / peak_magnitude
[[nodiscard]] SC16Q11Buffer
float_to_sc16q11(const std::vector<std::complex<float>>& samples,
                 float scale_factor);

// Find the peak magnitude in a sample buffer (for auto-scaling)
[[nodiscard]] float
find_peak_magnitude(const std::vector<std::complex<float>>& samples);

} // namespace gps