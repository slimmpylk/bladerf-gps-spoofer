// ─────────────────────────────────────────────────────────────────────────────
// nav_message.cpp — GPS L1 C/A Navigation Message Encoder
//
// THE BIG PICTURE of what this file does:
//
//   The phone needs to know where the satellite is in space.
//   The satellite broadcasts this as a stream of bits at 50 bps.
//   Those bits are organized into words → subframes → frames.
//
//   This file takes our Ephemeris struct (floating-point orbital parameters)
//   and packs them into the exact bit layout the IS-GPS-200N standard defines,
//   with correct Hamming parity on every word.
//
// THE TWO KEY CHALLENGES:
//
//   1. SCALE FACTORS — the orbital parameters are doubles (e.g. M0 = 0.5412 rad)
//      but the nav message stores them as fixed-point integers with defined
//      scale factors. M0 is stored as a 32-bit integer where 1 LSB = π×2⁻³¹ rad.
//      We must scale correctly or every satellite position will be wrong.
//
//   2. PARITY — each 30-bit word has 6 Hamming parity bits. If any bit is wrong
//      the receiver discards the word silently. Getting parity exactly right,
//      including the D29*/D30* chaining between adjacent words, is critical.
// ─────────────────────────────────────────────────────────────────────────────

#include "nav_message.hpp"
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace gps {

// ─────────────────────────────────────────────────────────────────────────────
// Scale factor helpers
//
// The navigation message stores floating-point values as integers.
// Each parameter has a defined "scale factor" (LSB value) from IS-GPS-200N
// Table 20-III. To encode: integer = round(value / scale_factor)
//
// Example: M0 scale factor is π × 2⁻³¹ rad/LSB
//   If M0 = 1.2345 rad:
//   integer = round(1.2345 / (π × 2⁻³¹)) = round(1.2345 × 2³¹ / π) = 843,314,XXX
//
// WHY these specific scale factors?
//   The scale factors were chosen so the values fit exactly in the allocated
//   bit widths with enough precision for the receivers to compute sub-metre
//   satellite positions. They're defined in the GPS ICD and cannot be changed.
//
// WHY static functions?
//   These helpers are only used inside this .cpp file. 'static' means they
//   have internal linkage — they don't pollute the global namespace and can't
//   be called from other files. This is the C++ equivalent of "private to
//   this file."
// ─────────────────────────────────────────────────────────────────────────────

// Scale a value in radians where 1 LSB = π × 2^(-power)
// Used for angles: M0, omega, Omega0, i0 (32-bit, signed)
static int32_t scale_semicircles(double value_rad, int power) {
    // Convert radians to semicircles (1 semicircle = π radians)
    // then scale by 2^power
    return static_cast<int32_t>(
        std::round(value_rad / std::numbers::pi * std::pow(2.0, power))
    );
}

// Scale a value where 1 LSB = 2^(-power) of the unit
// Used for most other parameters
static int32_t scale_value(double value, int power) {
    return static_cast<int32_t>(
        std::round(value * std::pow(2.0, power))
    );
}

// Mask to N bits (keep only the lowest N bits of a value)
// WHY? scale_semicircles returns int32_t but we might only have 8 bits
// allocated in the nav message. This masks to the correct width.
static constexpr uint32_t mask_bits(int32_t value, int n_bits) {
    return static_cast<uint32_t>(value) & ((1u << n_bits) - 1);
}


// ─────────────────────────────────────────────────────────────────────────────
// compute_nav_word() — Parity encoder
//
// This is the most critical function in this file.
// IS-GPS-200N §20.3.5.2, Table 20-XIV defines exactly which data bits
// contribute to each parity bit.
//
// The parity equations (D = transmitted bit, d = data bit, * = prev word):
//   D25 = D29* ⊕ d1⊕d2⊕d3⊕d5⊕d6⊕d10⊕d11⊕d12⊕d13⊕d14⊕d17⊕d18⊕d20⊕d23
//   D26 = D30* ⊕ d2⊕d3⊕d4⊕d6⊕d7⊕d11⊕d12⊕d13⊕d14⊕d15⊕d18⊕d19⊕d21⊕d24
//   D27 = D29* ⊕ d1⊕d3⊕d4⊕d5⊕d7⊕d8⊕d12⊕d13⊕d14⊕d15⊕d16⊕d19⊕d20⊕d22
//   D28 = D30* ⊕ d2⊕d4⊕d5⊕d6⊕d8⊕d9⊕d13⊕d14⊕d15⊕d16⊕d17⊕d20⊕d21⊕d23
//   D29 = D30* ⊕ d1⊕d2⊕d3⊕d5⊕d6⊕d7⊕d9⊕d10⊕d14⊕d15⊕d16⊕d17⊕d18⊕d21⊕d22⊕d24
//   D30 = D30* ⊕ d3⊕d5⊕d6⊕d8⊕d9⊕d10⊕d11⊕d13⊕d15⊕d19⊕d22⊕d23⊕d24
//
// IMPORTANT: if D30* = 1, all 24 data bits are COMPLEMENTED before parity.
// This is the "bit inversion" rule — it ensures the parity works correctly
// regardless of the previous word's last bit.
// ─────────────────────────────────────────────────────────────────────────────

NavWord compute_nav_word(uint32_t data_bits, uint8_t D29_star, uint8_t D30_star) {
    // Step 1: If D30* = 1, invert all 24 data bits (IS-GPS-200N §20.3.5.2)
    // WHY? This conditioning ensures that the parity algorithm produces
    // valid codewords regardless of the previous word's polarity.
    if (D30_star) {
        data_bits ^= 0x00FFFFFFu;  // XOR with 24 ones = flip all 24 data bits
    }

    // Step 2: Extract individual data bits d1..d24
    // The spec uses 1-based indexing. d1 is the MSB (bit 23 in 0-based),
    // d24 is the LSB (bit 0 in 0-based).
    //
    // WHY this bit ordering?
    //   GPS navigation message is transmitted MSB first. The spec numbers
    //   bits 1 (first transmitted) through 30 (last transmitted).
    //   In our uint32_t, bit 23 = d1 (first), bit 0 = d24 (last data bit).
    //
    // Helper lambda: extract bit n (1-based) from data_bits
    // Bit 1 = position 23, bit 2 = position 22, ..., bit n = position (24-n)
    auto d = [&](int n) -> uint32_t {
        return (data_bits >> (24 - n)) & 1u;
    };

    // Step 3: Compute 6 parity bits per IS-GPS-200N Table 20-XIV
    // Each is an XOR of specific data bits plus one of D29*/D30*
    const uint32_t D25 = D29_star ^ d(1)^d(2)^d(3)^d(5)^d(6)^d(10)
                        ^d(11)^d(12)^d(13)^d(14)^d(17)^d(18)^d(20)^d(23);

    const uint32_t D26 = D30_star ^ d(2)^d(3)^d(4)^d(6)^d(7)^d(11)
                        ^d(12)^d(13)^d(14)^d(15)^d(18)^d(19)^d(21)^d(24);

    const uint32_t D27 = D29_star ^ d(1)^d(3)^d(4)^d(5)^d(7)^d(8)
                        ^d(12)^d(13)^d(14)^d(15)^d(16)^d(19)^d(20)^d(22);

    const uint32_t D28 = D30_star ^ d(2)^d(4)^d(5)^d(6)^d(8)^d(9)
                        ^d(13)^d(14)^d(15)^d(16)^d(17)^d(20)^d(21)^d(23);

    const uint32_t D29 = D30_star ^ d(1)^d(2)^d(3)^d(5)^d(6)^d(7)^d(9)
                        ^d(10)^d(14)^d(15)^d(16)^d(17)^d(18)^d(21)^d(22)^d(24);

    const uint32_t D30 = D30_star ^ d(3)^d(5)^d(6)^d(8)^d(9)^d(10)
                        ^d(11)^d(13)^d(15)^d(19)^d(22)^d(23)^d(24);

    // Step 4: Pack into 30-bit word
    // Layout: [d1..d24][D25..D30] = bits 29..6 then bits 5..0
    //
    // Shift data bits up 6 positions (to make room for parity in bits 5..0)
    // then OR in the 6 parity bits.
    const NavWord word = ((data_bits & 0x00FFFFFFu) << 6)
                       | (D25 << 5) | (D26 << 4) | (D27 << 3)
                       | (D28 << 2) | (D29 << 1) | D30;

    return word;
}


// ─────────────────────────────────────────────────────────────────────────────
// TLM and HOW words — every subframe starts with these two
//
// TLM = TeLemetry word (word 1 of every subframe)
//   - Preamble: 10001011 (0x8B) — fixed sync pattern
//   - TLM message: 14 bits (we set to 0 for our purposes)
//   - Reserved: 2 bits
//
// HOW = Hand-Over Word (word 2 of every subframe)
//   - TOW count: 17 bits — GPS Time Of Week in 6-second units
//   - Alert flag: 1 bit
//   - Anti-spoof flag: 1 bit
//   - Subframe ID: 3 bits (1–5)
//   - Reserved: 2 bits
//
// WHY "hand-over"? In older GPS receivers, the HOW helped "hand over" from
// the acquisition stage to the tracking stage by providing the current
// GPS time. Modern receivers use it to know the current TOW.
// ─────────────────────────────────────────────────────────────────────────────

static NavWord make_tlm(uint8_t D29_prev, uint8_t D30_prev) {
    // TLM preamble is always 10001011
    // 14 TLM message bits = 0 (reserved/unused in our implementation)
    // 2 reserved bits = 0
    // Total 24 data bits: preamble(8) + tlm_msg(14) + reserved(2)
    constexpr uint32_t kPreamble = 0x8Bu;  // 10001011
    const uint32_t data = (kPreamble << 16);  // preamble in bits 23..16, rest 0
    return compute_nav_word(data, D29_prev, D30_prev);
}

static NavWord make_how(uint32_t tow_count, uint8_t subframe_id,
                         uint8_t D29_prev, uint8_t D30_prev) {
    // tow_count: TOW in units of 6 seconds (17 bits, range 0..100799)
    // subframe_id: 1..5 (3 bits)
    // Layout: [tow(17)][alert(1)][anti-spoof(1)][subframe_id(3)][reserved(2)]
    //         = 24 bits total
    const uint32_t data = ((tow_count & 0x1FFFFu) << 7)
                        | (0u << 6)              // alert = 0
                        | (0u << 5)              // anti-spoof = 0
                        | ((subframe_id & 0x7u) << 2)
                        | 0u;                    // reserved = 0
    return compute_nav_word(data, D29_prev, D30_prev);
}

// Helper: get D29 and D30 (the last two bits) from a completed NavWord
// These become D29* and D30* for the NEXT word's parity computation.
static inline uint8_t get_D29(NavWord w) { return (w >> 1) & 1u; }
static inline uint8_t get_D30(NavWord w) { return  w       & 1u; }


// ─────────────────────────────────────────────────────────────────────────────
// encode_subframe1() — Clock corrections
//
// Subframe 1 contains:
//   - GPS week number (10 bits)
//   - Satellite health (6 bits) — we set 0 (healthy)
//   - Clock reference time t_oc (16 bits)
//   - Clock correction coefficients af0, af1, af2
//   - Group delay Tgd
//
// Scale factors from IS-GPS-200N Table 20-III:
//   t_oc: 2^4 seconds/LSB   (16 bits)
//   af2:  2^-55 s/s²/LSB    (8 bits, signed)
//   af1:  2^-43 s/s/LSB     (16 bits, signed)
//   af0:  2^-31 s/LSB       (22 bits, signed)
//   TGD:  2^-31 s/LSB       (8 bits, signed)
// ─────────────────────────────────────────────────────────────────────────────

NavSubframe encode_subframe1(const Ephemeris& eph, uint32_t tow_count) {
    NavSubframe sf{};

    // Word 1: TLM (no previous word — use D29*=D30*=0 for first word of frame)
    sf[0] = make_tlm(0, 0);

    // Word 2: HOW
    sf[1] = make_how(tow_count, 1, get_D29(sf[0]), get_D30(sf[0]));

    // Word 3: GPS week (10 bits) + codes on L2 (2) + URA (4) + health (6) + IODC MSBs (2)
    {
        const uint32_t week  = eph.gps_week & 0x3FFu;  // 10 bits
        const uint32_t codes = 0u;   // L2 codes = 01 (C/A only)
        const uint32_t ura   = 0u;   // URA index 0 = best accuracy
        const uint32_t health= 0u;   // 0 = fully operational
        const uint32_t iodc_msb = 0u;
        const uint32_t data  = (week << 14) | (codes << 12) | (ura << 8)
                             | (health << 2) | iodc_msb;
        sf[2] = compute_nav_word(data, get_D29(sf[1]), get_D30(sf[1]));
    }

    // Words 4–7: reserved (set to zero)
    for (int w = 3; w <= 6; ++w) {
        sf[w] = compute_nav_word(0, get_D29(sf[w-1]), get_D30(sf[w-1]));
    }

    // Word 8: TGD (8 bits signed) + IODC LSBs (8 bits) + reserved (8 bits)
    {
        const int32_t tgd_scaled = scale_value(eph.T_GD, 31);
        const uint32_t tgd = mask_bits(tgd_scaled, 8);
        const uint32_t data = (tgd << 8);  // IODC LSB = 0, reserved = 0
        sf[7] = compute_nav_word(data, get_D29(sf[6]), get_D30(sf[6]));
    }

    // Word 9: t_oc (16 bits) + af2 (8 bits signed)
    {
        const uint32_t toc = mask_bits(scale_value(eph.t_oc, -4), 16);
        const int32_t af2_scaled = scale_value(eph.a_f2, 55);
        const uint32_t af2 = mask_bits(af2_scaled, 8);
        const uint32_t data = (toc << 8) | af2;
        sf[8] = compute_nav_word(data, get_D29(sf[7]), get_D30(sf[7]));
    }

    // Word 10: af1 (16 bits signed) + af0 (22 bits signed) — NOTE: only 22 bits for af0!
    // af0 is split: 22 bits in word 10, but standard allocates 22 bits total
    {
        const int32_t af1_scaled = scale_value(eph.a_f1, 43);
        const int32_t af0_scaled = scale_value(eph.a_f0, 31);
        const uint32_t af1 = mask_bits(af1_scaled, 16);
        const uint32_t af0 = mask_bits(af0_scaled, 22);
        // Word 10: af1(16) + af0(22) — but we only have 24 data bits total
        // af1 uses bits 23..8, af0 MSBs use bits 7..0 (8 bits), rest in next word
        // Simplified: pack af1(16) + af0 MSByte(8) = 24 bits
        const uint32_t data = (af1 << 8) | ((af0 >> 14) & 0xFFu);
        sf[9] = compute_nav_word(data, get_D29(sf[8]), get_D30(sf[8]));
    }

    return sf;
}


// ─────────────────────────────────────────────────────────────────────────────
// encode_subframe2() — Ephemeris Part I
//
// Contains: IODE, Crs, Δn, M0, Cuc, e, Cus, √A, t_oe, fit_interval, AODO
//
// Scale factors (IS-GPS-200N Table 20-III):
//   Crs:  2^-5  m/LSB           (16 bits, signed)
//   Δn:   π×2^-43 rad/s/LSB    (16 bits, signed)
//   M0:   π×2^-31 rad/LSB      (32 bits, signed) — spans 2 words
//   Cuc:  2^-29 rad/LSB         (16 bits, signed)
//   e:    2^-33/LSB             (32 bits, unsigned) — spans 2 words
//   Cus:  2^-29 rad/LSB         (16 bits, signed)
//   √A:   2^-19 m^½/LSB        (32 bits, unsigned) — spans 2 words
//   t_oe: 2^4 s/LSB             (16 bits, unsigned)
// ─────────────────────────────────────────────────────────────────────────────

NavSubframe encode_subframe2(const Ephemeris& eph, uint32_t tow_count) {
    NavSubframe sf{};

    sf[0] = make_tlm(0, 0);
    sf[1] = make_how(tow_count, 2, get_D29(sf[0]), get_D30(sf[0]));

    // Word 3: IODE (8 bits) + Crs (16 bits)
    {
        const int32_t crs = scale_value(eph.C_rs, 5);
        const uint32_t data = (0u << 16) | mask_bits(crs, 16); // IODE=0
        sf[2] = compute_nav_word(data, get_D29(sf[1]), get_D30(sf[1]));
    }

    // Word 4: Δn (16 bits) + M0 MSBs (8 bits)
    {
        const int32_t dn  = scale_semicircles(eph.delta_n, 43);
        const int32_t m0  = scale_semicircles(eph.M0, 31);
        const uint32_t data = (mask_bits(dn, 16) << 8) | ((mask_bits(m0, 32) >> 24) & 0xFFu);
        sf[3] = compute_nav_word(data, get_D29(sf[2]), get_D30(sf[2]));
    }

    // Word 5: M0 LSBs (24 bits)
    {
        const int32_t m0 = scale_semicircles(eph.M0, 31);
        const uint32_t data = mask_bits(m0, 32) & 0x00FFFFFFu;
        sf[4] = compute_nav_word(data, get_D29(sf[3]), get_D30(sf[3]));
    }

    // Word 6: Cuc (16 bits) + e MSBs (8 bits)
    {
        const int32_t  cuc = scale_value(eph.C_uc, 29);
        const uint32_t e32 = static_cast<uint32_t>(std::round(eph.e * std::pow(2.0, 33)));
        const uint32_t data = (mask_bits(cuc, 16) << 8) | ((e32 >> 24) & 0xFFu);
        sf[5] = compute_nav_word(data, get_D29(sf[4]), get_D30(sf[4]));
    }

    // Word 7: e LSBs (24 bits)
    {
        const uint32_t e32 = static_cast<uint32_t>(std::round(eph.e * std::pow(2.0, 33)));
        sf[6] = compute_nav_word(e32 & 0x00FFFFFFu, get_D29(sf[5]), get_D30(sf[5]));
    }

    // Word 8: Cus (16 bits) + √A MSBs (8 bits)
    {
        const int32_t  cus  = scale_value(eph.C_us, 29);
        const uint32_t sqA  = static_cast<uint32_t>(std::round(eph.sqrt_A * std::pow(2.0, 19)));
        const uint32_t data = (mask_bits(cus, 16) << 8) | ((sqA >> 24) & 0xFFu);
        sf[7] = compute_nav_word(data, get_D29(sf[6]), get_D30(sf[6]));
    }

    // Word 9: √A LSBs (24 bits)
    {
        const uint32_t sqA = static_cast<uint32_t>(std::round(eph.sqrt_A * std::pow(2.0, 19)));
        sf[8] = compute_nav_word(sqA & 0x00FFFFFFu, get_D29(sf[7]), get_D30(sf[7]));
    }

    // Word 10: t_oe (16 bits) + fit interval (1 bit) + AODO (5 bits) + reserved (2 bits)
    {
        const uint32_t toe  = mask_bits(scale_value(eph.t_oe, -4), 16);
        const uint32_t data = (toe << 8) | 0u;  // fit=0, AODO=0, reserved=0
        sf[9] = compute_nav_word(data, get_D29(sf[8]), get_D30(sf[8]));
    }

    return sf;
}


// ─────────────────────────────────────────────────────────────────────────────
// encode_subframe3() — Ephemeris Part II
//
// Contains: Cic, Ω0, Cis, i0, Crc, ω, Ω̇, İ, IODE
//
// Scale factors:
//   Cic, Cis: 2^-29 rad/LSB   (16 bits, signed)
//   Ω0, i0:   π×2^-31 rad/LSB (32 bits, signed) — spans 2 words each
//   Crc:      2^-5 m/LSB      (16 bits, signed)
//   ω:        π×2^-31 rad/LSB (32 bits, signed) — spans 2 words
//   Ω̇:        π×2^-43 rad/s/LSB (24 bits, signed)
//   İ:        π×2^-43 rad/s/LSB (14 bits, signed)
// ─────────────────────────────────────────────────────────────────────────────

NavSubframe encode_subframe3(const Ephemeris& eph, uint32_t tow_count) {
    NavSubframe sf{};

    sf[0] = make_tlm(0, 0);
    sf[1] = make_how(tow_count, 3, get_D29(sf[0]), get_D30(sf[0]));

    // Word 3: Cic (16 bits) + Ω0 MSBs (8 bits)
    {
        const int32_t  cic  = scale_value(eph.C_ic, 29);
        const int32_t  om0  = scale_semicircles(eph.Omega0, 31);
        const uint32_t data = (mask_bits(cic, 16) << 8) | ((mask_bits(om0,32) >> 24) & 0xFFu);
        sf[2] = compute_nav_word(data, get_D29(sf[1]), get_D30(sf[1]));
    }

    // Word 4: Ω0 LSBs (24 bits)
    {
        const int32_t om0 = scale_semicircles(eph.Omega0, 31);
        sf[3] = compute_nav_word(mask_bits(om0,32) & 0x00FFFFFFu, get_D29(sf[2]), get_D30(sf[2]));
    }

    // Word 5: Cis (16 bits) + i0 MSBs (8 bits)
    {
        const int32_t  cis = scale_value(eph.C_is, 29);
        const int32_t  i0  = scale_semicircles(eph.i0, 31);
        const uint32_t data = (mask_bits(cis,16) << 8) | ((mask_bits(i0,32) >> 24) & 0xFFu);
        sf[4] = compute_nav_word(data, get_D29(sf[3]), get_D30(sf[3]));
    }

    // Word 6: i0 LSBs (24 bits)
    {
        const int32_t i0 = scale_semicircles(eph.i0, 31);
        sf[5] = compute_nav_word(mask_bits(i0,32) & 0x00FFFFFFu, get_D29(sf[4]), get_D30(sf[4]));
    }

    // Word 7: Crc (16 bits) + ω MSBs (8 bits)
    {
        const int32_t  crc = scale_value(eph.C_rc, 5);
        const int32_t  om  = scale_semicircles(eph.omega, 31);
        const uint32_t data = (mask_bits(crc,16) << 8) | ((mask_bits(om,32) >> 24) & 0xFFu);
        sf[6] = compute_nav_word(data, get_D29(sf[5]), get_D30(sf[5]));
    }

    // Word 8: ω LSBs (24 bits)
    {
        const int32_t om = scale_semicircles(eph.omega, 31);
        sf[7] = compute_nav_word(mask_bits(om,32) & 0x00FFFFFFu, get_D29(sf[6]), get_D30(sf[6]));
    }

    // Word 9: Ω̇ (24 bits signed)
    {
        const int32_t omdot = scale_semicircles(eph.Omega_dot, 43);
        sf[8] = compute_nav_word(mask_bits(omdot, 24), get_D29(sf[7]), get_D30(sf[7]));
    }

    // Word 10: IODE (8 bits) + İ (14 bits) + reserved (2 bits)
    {
        const int32_t  idot = scale_semicircles(eph.i_dot, 43);
        const uint32_t data = (0u << 16)                      // IODE = 0
                            | (mask_bits(idot, 14) << 2)       // İ in bits 15..2
                            | 0u;                              // reserved
        sf[9] = compute_nav_word(data, get_D29(sf[8]), get_D30(sf[8]));
    }

    return sf;
}


// ─────────────────────────────────────────────────────────────────────────────
// frame_to_bitstream() — flatten 2D frame → 1D bit array
//
// The modulator just wants a flat array of bits it can index with a counter.
// This converts NavFrame (5 × 10 × 30 bits) into NavBitStream (1500 bits).
//
// Bit ordering: MSB first within each word (bit 29 transmitted first).
// WHY MSB first? GPS transmits the most significant bit first — this is
// defined in IS-GPS-200N §20.3.1. The modulator must output them in this order.
// ─────────────────────────────────────────────────────────────────────────────

NavBitStream frame_to_bitstream(const NavFrame& frame) {
    NavBitStream stream;
    int bit_idx = 0;

    for (const auto& subframe : frame) {
        for (const auto& word : subframe) {
            // Extract 30 bits, MSB (bit 29) first
            for (int b = 29; b >= 0; --b) {
                stream.bits[bit_idx++] = (word >> b) & 1u;
            }
        }
    }

    return stream;
}


// ─────────────────────────────────────────────────────────────────────────────
// encode_nav_message() — top-level encoder
//
// Builds the complete 1500-bit frame by assembling SF1, SF2, SF3 and
// filling SF4/SF5 with zeros (almanac — not needed for basic spoofing).
//
// tow_count: GPS Time Of Week in 6-second units
//   TOW in seconds / 6 = tow_count
//   e.g. if GPS time is 3600s into the week → tow_count = 600
// ─────────────────────────────────────────────────────────────────────────────

NavBitStream encode_nav_message(const Ephemeris& eph, double gps_tow) {
    if (!eph.valid) {
        throw std::invalid_argument("Cannot encode nav message from invalid ephemeris");
    }

    // Convert GPS TOW (seconds) to HOW count (6-second units)
    // IS-GPS-200N: HOW TOW count = (TOW in seconds) / 6
    const uint32_t tow_count = static_cast<uint32_t>(gps_tow / 6.0) & 0x1FFFFu;

    NavFrame frame{};
    frame[0] = encode_subframe1(eph, tow_count);
    frame[1] = encode_subframe2(eph, tow_count + 1);
    frame[2] = encode_subframe3(eph, tow_count + 2);

    // Subframes 4 and 5: fill with zeros (almanac, not required for basic lock)
    // A receiver will still acquire and track with only SF1-3 valid.
    for (int w = 0; w < 10; ++w) {
        const uint8_t d29_prev = (w == 0) ? 0 : get_D29(frame[3][w-1]);
        const uint8_t d30_prev = (w == 0) ? 0 : get_D30(frame[3][w-1]);
        frame[3][w] = compute_nav_word(0, d29_prev, d30_prev);
        frame[4][w] = compute_nav_word(0,
            get_D29(frame[3][w]), get_D30(frame[3][w]));
    }
    // Fix SF4/SF5 TLM/HOW
    frame[3][0] = make_tlm(0, 0);
    frame[3][1] = make_how(tow_count + 3, 4, get_D29(frame[3][0]), get_D30(frame[3][0]));
    frame[4][0] = make_tlm(0, 0);
    frame[4][1] = make_how(tow_count + 4, 5, get_D29(frame[4][0]), get_D30(frame[4][0]));

    return frame_to_bitstream(frame);
}

} // namespace gps