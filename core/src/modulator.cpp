// ─────────────────────────────────────────────────────────────────────────────
// modulator.cpp — Per-Channel BPSK I/Q Modulator
//
// THE SIGNAL EQUATION (per channel k, per sample n):
//
//   I_k(n) = A · D(n) · CA(n) · cos(2π · f_D · n/f_s + φ)
//   Q_k(n) = A · D(n) · CA(n) · sin(2π · f_D · n/f_s + φ)
//
// Where:
//   A    = amplitude (all channels equal)
//   D(n) = navigation data bit at sample n  ∈ {-1, +1}
//   CA(n)= C/A code chip at sample n        ∈ {-1, +1}
//   f_D  = Doppler shift for this satellite (Hz)
//   f_s  = sample rate (2.048 Msps)
//   φ    = carrier phase (radians, advances each sample)
//
// The cos/sin pair gives us I and Q — the two components of a complex
// baseband signal. Together they encode both amplitude AND phase of the
// modulated carrier. The bladeRF's hardware multiplies this baseband
// signal by cos(2π · 1575.42MHz · t) to shift it up to RF.
// ─────────────────────────────────────────────────────────────────────────────

#include "modulator.hpp"
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace gps {

// ─────────────────────────────────────────────────────────────────────────────
// compute_range()
//
// Simple 3D Euclidean distance between satellite and receiver in ECEF.
// ρ = √((xs-xr)² + (ys-yr)² + (zs-zr)²)
// ─────────────────────────────────────────────────────────────────────────────
double compute_range(const EcefPosition& sat, const EcefPosition& rx) {
    const double dx = sat.x - rx.x;
    const double dy = sat.y - rx.y;
    const double dz = sat.z - rx.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// ─────────────────────────────────────────────────────────────────────────────
// compute_doppler()
//
// f_D = -(v_sv - v_rx) · ê_{rx→sv} / λ_L1
//
// WHY negative sign?
//   When the satellite moves TOWARD the receiver (closing velocity > 0),
//   the signal frequency increases (blue shift). The dot product of the
//   velocity difference with the unit vector is positive when closing,
//   so we negate it to get a positive Doppler for approaching satellites.
//
// λ_L1 = c / f_L1 = 299792458 / 1575420000 ≈ 0.1903 m
// ─────────────────────────────────────────────────────────────────────────────
double compute_doppler(const EcefVelocity& sat_vel,
                       const EcefVelocity& rx_vel,
                       double ux, double uy, double uz) {
    constexpr double kLambdaL1 = wgs84::kSpeedOfLight / 1575.42e6;

    // Relative velocity of satellite w.r.t. receiver, projected onto LOS
    const double rel_vx = sat_vel.vx - rx_vel.vx;
    const double rel_vy = sat_vel.vy - rx_vel.vy;
    const double rel_vz = sat_vel.vz - rx_vel.vz;

    // Dot product with unit vector (line-of-sight component)
    const double v_los = rel_vx*ux + rel_vy*uy + rel_vz*uz;

    return -v_los / kLambdaL1;
}

// ─────────────────────────────────────────────────────────────────────────────
// compute_code_phase()
//
// τ = (ρ / c) × f_chip
//
// This converts a distance (metres) to a C/A code phase offset (chips).
// The receiver measures this phase offset and uses it to compute range.
//
// Example: if satellite is 20,086,195 m away:
//   τ = (20,086,195 / 299,792,458) × 1,023,000 ≈ 68,571.2 chips
//   Wrapped: 68,571.2 mod 1023 ≈ 501.2 chips
//
// We wrap modulo kCACodeLength because the code repeats every 1ms.
// The receiver can't distinguish ranges that differ by whole code lengths
// (multiples of 300km), but that doesn't matter — it resolves ambiguity
// using the nav message TOW.
// ─────────────────────────────────────────────────────────────────────────────
double compute_code_phase(double range_m) {
    const double chips = (range_m / wgs84::kSpeedOfLight) * kChipRate;
    // fmod gives floating-point modulo — keeps the fractional chip offset
    return std::fmod(chips, static_cast<double>(kCACodeLength));
}

// ─────────────────────────────────────────────────────────────────────────────
// make_channel()
//
// Assembles a ChannelConfig from satellite state + receiver position.
// This is called once per satellite per update epoch (every ~1 second
// in practice — geometry changes slowly enough).
// ─────────────────────────────────────────────────────────────────────────────
ChannelConfig make_channel(
    int                   prn,
    const SatelliteState& sat_state,
    const EcefPosition&   receiver_pos,
    const EcefVelocity&   receiver_vel,
    double                gps_time_s,
    const NavBitStream&   nav_bits)
{
    if (!is_valid_prn(prn)) {
        throw std::invalid_argument("Invalid PRN in make_channel");
    }

    ChannelConfig cfg;
    cfg.prn = prn;

    // ── Range and unit vector ─────────────────────────────────────────────
    const EcefPosition sat_pos{sat_state.x, sat_state.y, sat_state.z};
    const double range = compute_range(sat_pos, receiver_pos);

    // Unit vector from receiver to satellite
    // WHY divide by range? We need a direction only (magnitude = 1).
    // The dot product in compute_doppler needs a normalised vector.
    const double ux = (sat_state.x - receiver_pos.x) / range;
    const double uy = (sat_state.y - receiver_pos.y) / range;
    const double uz = (sat_state.z - receiver_pos.z) / range;

    // ── Doppler shift ─────────────────────────────────────────────────────
    const EcefVelocity sat_vel{sat_state.vx, sat_state.vy, sat_state.vz};
    cfg.doppler_hz = compute_doppler(sat_vel, receiver_vel, ux, uy, uz);

    // ── Code phase ────────────────────────────────────────────────────────
    // Apply satellite clock correction to the range
    // ρ_corrected = ρ - c · Δt_sv
    const double corrected_range = range
        - wgs84::kSpeedOfLight * sat_state.clock_error;

    cfg.code_phase    = compute_code_phase(corrected_range);
    cfg.carrier_phase = 0.0;   // arbitrary starting phase
    cfg.amplitude     = 1.0f;

    // ── Pre-generate C/A code ─────────────────────────────────────────────
    // WHY pre-generate and cache?
    //   The C/A code for a given PRN never changes — it's a fixed sequence.
    //   generate_ca_code() takes microseconds. Calling it once here and
    //   caching the result avoids recomputing it 2048 times per millisecond.
    cfg.ca_code  = generate_ca_code(prn);
    cfg.nav_bits = nav_bits;

    return cfg;
}

// ─────────────────────────────────────────────────────────────────────────────
// generate_epoch()
//
// Core of the modulator. Generates kSamplesPerMs (2048) I/Q samples for
// one 1ms C/A epoch for a single satellite channel.
//
// SAMPLE-BY-SAMPLE LOOP:
//   For each sample n:
//   1. Look up C/A chip at the current code phase position
//   2. Look up NAV data bit at the current bit position
//   3. Convert both to ±1 signal domain
//   4. Compute the Doppler-shifted phase angle
//   5. Output I = A · D · CA · cos(φ),  Q = A · D · CA · sin(φ)
//   6. Advance code phase by (chip_rate / sample_rate) per sample
//   7. Advance carrier phase by (2π · f_D / sample_rate) per sample
// ─────────────────────────────────────────────────────────────────────────────
void generate_epoch(
    const ChannelConfig&              cfg,
    ModulatorState&                   state,
    std::vector<std::complex<float>>& samples)
{
    // Pre-compute the phase increment per sample for the Doppler frequency.
    // WHY pre-compute? This value is constant across the whole epoch.
    // Computing it once saves 2048 multiplications.
    const double phase_inc = 2.0 * std::numbers::pi * cfg.doppler_hz / kSampleRate;

    // Samples per nav bit: at 2.048 Msps and 50 bps nav rate:
    // 2,048,000 / 50 = 40,960 samples per nav bit
    constexpr int kSamplesPerNavBit = static_cast<int>(kSampleRate / kNavBitRate);

    // Code phase increment per sample:
    // At 2.048 Msps and 1.023 Mcps: 1,023,000 / 2,048,000 ≈ 0.4995 chips/sample
    // This is slightly less than 0.5 — the code advances ~0.5 chips per sample
    const double chip_inc = kChipRate / kSampleRate;

    // Work with local copies of state for the inner loop
    // WHY? Avoids writing to state on every iteration — write once at end
    double code_phase    = state.code_phase;
    double carrier_phase = state.carrier_phase;
    int    nav_bit_idx   = state.nav_bit_idx;
    int    sample_count  = state.sample_count;

    // Reserve space for the new samples
    // WHY reserve? samples.push_back() can trigger reallocations.
    // Reserving upfront makes the loop allocation-free.
    samples.reserve(samples.size() + kSamplesPerMs);

    for (int n = 0; n < kSamplesPerMs; ++n) {

        // ── Step 1: Get C/A chip at current code phase ────────────────────
        // code_phase is a floating-point chip position (0.0 to 1022.999).
        // We take the integer part to index the code array.
        //
        // WHY static_cast<int> instead of std::floor?
        //   For positive values they're equivalent but static_cast is faster.
        //   code_phase is always positive here so this is safe.
        const int chip_idx = static_cast<int>(code_phase) % kCACodeLength;
        const int8_t raw_chip = cfg.ca_code[chip_idx];

        // ── Step 2: Get NAV data bit at current sample position ───────────
        // One nav bit lasts kSamplesPerNavBit (40,960) samples.
        // nav_bit_idx advances one position every kSamplesPerNavBit samples.
        //
        // WHY sample_count for nav bit timing instead of code phase?
        //   Nav bits are synchronised to absolute time, not code epochs.
        //   A nav bit might span multiple C/A epochs. Using an absolute
        //   sample counter is more accurate than relying on epoch boundaries.
        const int bit_pos = nav_bit_idx % NavBitStream::kTotalBits;
        const uint8_t raw_bit = cfg.nav_bits[bit_pos];

        // ── Step 3: Convert to ±1 signal domain ──────────────────────────
        // C/A: 0 → +1, 1 → -1  (NRZ-L per IS-GPS-200N)
        // NAV: same mapping
        const float chip_val = (raw_chip == 0) ?  1.0f : -1.0f;
        const float nav_val  = (raw_bit  == 0) ?  1.0f : -1.0f;

        // ── Step 4: Combined spreading × data ────────────────────────────
        // In the ±1 domain, XOR becomes multiplication.
        // This is the BPSK spreading operation — the nav bit can flip
        // the entire C/A code sequence phase by 180°.
        const float spread = cfg.amplitude * chip_val * nav_val;

        // ── Step 5: Generate I and Q ──────────────────────────────────────
        // I = spread · cos(φ)
        // Q = spread · sin(φ)
        //
        // WHY std::cos/sin on float?
        //   carrier_phase is double for precision but we cast to float
        //   for the final output. 32-bit float is plenty for I/Q samples
        //   that will be further quantised to 12-bit integers.
        const float i_sample = spread * static_cast<float>(std::cos(carrier_phase));
        const float q_sample = spread * static_cast<float>(std::sin(carrier_phase));

        samples.push_back(std::complex<float>(i_sample, q_sample));

        // ── Step 6: Advance code phase ────────────────────────────────────
        code_phase += chip_inc;
        if (code_phase >= kCACodeLength) {
            code_phase -= kCACodeLength;  // wrap at 1023 chips (1ms epoch)
        }

        // ── Step 7: Advance carrier phase ─────────────────────────────────
        carrier_phase += phase_inc;
        // Keep phase in [0, 2π) to prevent floating-point precision loss
        // over long runs. After millions of samples, carrier_phase could
        // reach 10^9 radians where cos/sin precision degrades.
        if (carrier_phase > 2.0 * std::numbers::pi) {
            carrier_phase -= 2.0 * std::numbers::pi;
        }

        // ── Step 8: Advance nav bit counter ──────────────────────────────
        ++sample_count;
        if (sample_count % kSamplesPerNavBit == 0) {
            ++nav_bit_idx;
        }
    }

    // Write updated state back
    state.code_phase    = code_phase;
    state.carrier_phase = carrier_phase;
    state.nav_bit_idx   = nav_bit_idx;
    state.sample_count  = sample_count;
}

} // namespace gps