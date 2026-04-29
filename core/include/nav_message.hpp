#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// nav_message.hpp — GPS L1 C/A Navigation Message Encoder
//
// WHAT THIS DOES:
//   Takes an Ephemeris struct (orbital data) and encodes it into the 1500-bit
//   GPS navigation message frame that a receiver needs to compute satellite
//   position. This is the 50 bps data stream riding on top of the C/A code.
//
// WHY THIS IS NEEDED:
//   The C/A code tells your phone WHICH satellite it's looking at and HOW FAR
//   away it is (via code phase timing). But to turn that distance into a
//   position fix, the phone needs to know WHERE the satellite is in space.
//   That's what the navigation message carries — the Keplerian orbital elements
//   the phone uses to compute the satellite's ECEF position.
//
//   If the nav message is missing or has wrong parity, the phone discards it
//   and waits. No nav message = no position fix, even if the C/A code is perfect.
//
// REFERENCE: IS-GPS-200N §20.3 — Navigation Message
//            IS-GPS-200N §20.3.5.2 — Parity encoding (Table 20-XIV)
//            IS-GPS-200N Table 20-III — Navigation message parameters & scale factors
// ─────────────────────────────────────────────────────────────────────────────

#include "ephemeris.hpp"
#include <array>
#include <cstdint>
#include <vector>

namespace gps {

// ─────────────────────────────────────────────────────────────────────────────
// Type definitions
//
// WHY uint32_t for a "30-bit word"?
//   GPS navigation words are 30 bits. C++ has no 30-bit integer type.
//   uint32_t (32-bit unsigned) is the smallest standard type that fits.
//   We just use the lower 30 bits and ignore the upper 2.
//
// WHY not std::bitset<30>?
//   bitset is good for bit manipulation but awkward for arithmetic.
//   We need to do both. uint32_t with explicit bit masks is clearer.
// ─────────────────────────────────────────────────────────────────────────────

using NavWord     = uint32_t;                    // 30-bit GPS nav word
using NavSubframe = std::array<NavWord, 10>;     // 10 words = 300 bits = 6 seconds
using NavFrame    = std::array<NavSubframe, 5>;  // 5 subframes = 1500 bits = 30 seconds

// ─────────────────────────────────────────────────────────────────────────────
// NavBitStream — the final output of this module
//
// The modulator needs individual bits at 50 bps. This struct holds the
// complete 1500-bit frame as a flat array of 0s and 1s.
//
// WHY a separate struct and not just return NavFrame?
//   NavFrame is a 2D structure (5 subframes × 10 words × 30 bits).
//   The modulator just wants a linear sequence of bits it can index with
//   a single counter. Flattening it once here means the modulator doesn't
//   need to do the 3D-to-1D conversion on every sample.
// ─────────────────────────────────────────────────────────────────────────────

struct NavBitStream {
    static constexpr int kTotalBits = 1500;          // 5 × 10 × 30
    std::array<uint8_t, kTotalBits> bits{};           // 0 or 1, one per element

    // Get bit at position i (0-based)
    // WHY operator[] instead of a method called get_bit()?
    //   operator[] lets us write stream[i] instead of stream.get_bit(i).
    //   More natural — it IS an array, so it should behave like one.
    uint8_t operator[](int i) const { return bits[i]; }
};


// ─────────────────────────────────────────────────────────────────────────────
// Parity computation — IS-GPS-200N §20.3.5.2
//
// WHY expose this as a public function?
//   The parity algorithm is the most error-prone part of the nav message.
//   Making it public means we can test it directly against the IS-GPS-200N
//   Table 20-XIV known vectors before trusting the full message encoder.
//
// Parameters:
//   data_bits  — the 24 data bits of the word (bits 1-24, 1-indexed per spec)
//                packed into bits [23:0] of a uint32_t
//   D29_star   — bit 29 of the PREVIOUS transmitted word (0 or 1)
//   D30_star   — bit 30 of the PREVIOUS transmitted word (0 or 1)
//
// Returns:
//   The complete 30-bit word: 24 data bits + 6 parity bits,
//   with D29*/D30* conditioning applied.
// ─────────────────────────────────────────────────────────────────────────────
[[nodiscard]] NavWord
compute_nav_word(uint32_t data_bits, uint8_t D29_star, uint8_t D30_star);


// ─────────────────────────────────────────────────────────────────────────────
// Main encoder — builds the complete 1500-bit NavBitStream
//
// Parameters:
//   eph      — the satellite's orbital and clock parameters (from RINEX)
//   gps_tow  — GPS Time Of Week in seconds (0–604799)
//              This tells the receiver WHEN this message was sent,
//              so it can compute t_k = t - t_oe correctly.
//
// Returns: NavBitStream ready to feed to the modulator
// ─────────────────────────────────────────────────────────────────────────────
[[nodiscard]] NavBitStream
encode_nav_message(const Ephemeris& eph, double gps_tow);


// ─────────────────────────────────────────────────────────────────────────────
// Individual subframe encoders (also public for testing)
// ─────────────────────────────────────────────────────────────────────────────

// Subframe 1: Satellite clock corrections + GPS week number + health
[[nodiscard]] NavSubframe encode_subframe1(const Ephemeris& eph, uint32_t tow_count);

// Subframe 2: Ephemeris part I (M0, e, sqrt_A, t_oe, correction terms)
[[nodiscard]] NavSubframe encode_subframe2(const Ephemeris& eph, uint32_t tow_count);

// Subframe 3: Ephemeris part II (i0, Omega0, omega, rates)
[[nodiscard]] NavSubframe encode_subframe3(const Ephemeris& eph, uint32_t tow_count);

// Flatten a NavFrame into a linear NavBitStream
[[nodiscard]] NavBitStream frame_to_bitstream(const NavFrame& frame);

} // namespace gps