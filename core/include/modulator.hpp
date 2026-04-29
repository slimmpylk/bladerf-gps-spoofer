#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// modulator.hpp — Per-Channel BPSK I/Q Modulator
//
// WHAT THIS DOES:
//   This is where the three components we've built finally meet each other.
//   For each visible satellite (each "channel"), it:
//     1. Takes the C/A code    (from ca_code.hpp)
//     2. Takes the NAV bits    (from nav_message.hpp)
//     3. Applies Doppler shift (from ephemeris.hpp geometry)
//     4. Produces complex I/Q baseband samples
//
// THE OUTPUT:
//   A stream of complex<float> samples at 2.048 Msps.
//   I = in-phase component, Q = quadrature (90° shifted) component.
//   These get summed across all channels in iq_stream.cpp, then
//   converted to SC16Q11 integers for the bladeRF.
//
// REFERENCE: IS-GPS-200N §3.3.1 — Signal structure
//            Proakis (2001) Ch.5 — BPSK modulation
// ─────────────────────────────────────────────────────────────────────────────

#include "ca_code.hpp"
#include "ephemeris.hpp"
#include "nav_message.hpp"
#include <complex>
#include <cstdint>
#include <vector>

namespace gps {

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────

constexpr double kChipRate    = 1.023e6;   // C/A chip rate (chips/second)
constexpr double kNavBitRate  = 50.0;      // Navigation data rate (bits/second)
constexpr double kNavBitSamples = 1.0;     // placeholder, computed from sample rate

// Samples per chip at 2.048 Msps: 2,048,000 / 1,023,000 ≈ 2.0 samples/chip
// WHY 2.048 Msps? It gives exactly 2 samples per chip (2.048/1.023 ≈ 2.0),
// making the code phase arithmetic clean. Also the minimum practical rate
// for GPS L1 C/A baseband simulation.
constexpr double kSampleRate  = 2.048e6;   // samples per second
constexpr int    kSamplesPerMs = 2048;     // samples per 1ms C/A epoch


// ─────────────────────────────────────────────────────────────────────────────
// ChannelConfig — everything needed to modulate one satellite channel
//
// WHY a config struct instead of function parameters?
//   We need to pass 6+ values. A struct is cleaner and lets us pre-compute
//   things once (like the CA code) rather than regenerating every call.
//   The modulator holds a vector of these — one per visible satellite.
// ─────────────────────────────────────────────────────────────────────────────

struct ChannelConfig {
    int     prn          = 0;      // Satellite PRN (1–32)
    double  doppler_hz   = 0.0;    // Carrier Doppler shift (Hz), typically ±5000
    double  code_phase   = 0.0;    // Initial code phase offset (chips, 0–1022)
    double  carrier_phase= 0.0;    // Initial carrier phase (radians)
    float   amplitude    = 1.0f;   // Relative signal amplitude (all channels equal by default)
    CACode  ca_code{};             // Pre-generated 1023-chip C/A sequence
    NavBitStream nav_bits{};       // Pre-encoded 1500-bit navigation message
};

// ─────────────────────────────────────────────────────────────────────────────
// ModulatorState — tracks the running state of the modulator across epochs
//
// WHY track state?
//   The modulator generates samples in 1ms blocks (one C/A epoch at a time).
//   Between blocks, the code phase, carrier phase, and nav bit position must
//   be preserved — otherwise the signal would restart from zero every millisecond
//   which would look like a completely different satellite to a receiver.
// ─────────────────────────────────────────────────────────────────────────────

struct ModulatorState {
    double code_phase    = 0.0;   // current chip position (0.0–1022.999...)
    double carrier_phase = 0.0;   // current carrier phase (radians)
    int    nav_bit_idx   = 0;     // current position in NavBitStream (0–1499)
    int    sample_count  = 0;     // total samples generated (used for nav bit timing)
};

// ─────────────────────────────────────────────────────────────────────────────
// Function declarations
// ─────────────────────────────────────────────────────────────────────────────

// Build a ChannelConfig for one satellite.
// Computes code phase and Doppler from the satellite's state and the
// receiver's target position.
//
// sat_state:    computed by compute_satellite_state()
// receiver_pos: the fake receiver position (where we want the phone to think it is)
// gps_time_s:   current GPS time in seconds
[[nodiscard]] ChannelConfig make_channel(
    int                    prn,
    const SatelliteState&  sat_state,
    const EcefPosition&    receiver_pos,
    const EcefVelocity&    receiver_vel,   // zero for stationary spoof
    double                 gps_time_s,
    const NavBitStream&    nav_bits
);

// Generate one 1ms block of I/Q samples for a single channel.
//
// WHY 1ms blocks?
//   One C/A epoch = 1ms = 2048 samples at 2.048 Msps.
//   Generating one epoch at a time lets us:
//     - Update geometry (Doppler, code phase) every epoch
//     - Handle nav bit transitions exactly at epoch boundaries
//     - Keep memory usage low (2048 samples at a time, not the whole second)
//
// The state is updated in-place so the next call continues seamlessly.
void generate_epoch(
    const ChannelConfig&  cfg,
    ModulatorState&       state,        // updated by this call
    std::vector<std::complex<float>>& samples  // output appended here
);

// Compute the geometric range between satellite and receiver (metres)
[[nodiscard]] double compute_range(
    const EcefPosition& sat,
    const EcefPosition& rx
);

// Compute Doppler shift in Hz
// sat_vel: satellite ECEF velocity (m/s)
// rx_vel:  receiver ECEF velocity (m/s, zero for stationary)
// unit_vec: unit vector from receiver to satellite
[[nodiscard]] double compute_doppler(
    const EcefVelocity& sat_vel,
    const EcefVelocity& rx_vel,
    double ux, double uy, double uz   // unit vector components
);

// Compute code phase offset in chips from geometric range
[[nodiscard]] double compute_code_phase(double range_m);

} // namespace gps