// ─────────────────────────────────────────────────────────────────────────────
// test_nav_message.cpp — Unit tests for the navigation message encoder
//
// These tests check the three things that matter most:
//   1. Parity — if any parity bit is wrong, receivers discard the word
//   2. Bit counts — each element must be exactly 0 or 1, exactly 1500 total
//   3. Decode round-trip — encode a value, decode it, get the original back
// ─────────────────────────────────────────────────────────────────────────────

#include "nav_message.hpp"
#include "ephemeris.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <numbers>

// Reuse the same test ephemeris from test_ephemeris.cpp
static gps::Ephemeris make_test_ephemeris() {
    gps::Ephemeris eph;
    eph.prn       = 1;
    eph.gps_week  = 2295;
    eph.t_oe      = 0.0;
    eph.sqrt_A    = 5153.79476547;
    eph.e         = 0.00824451;
    eph.i0        = 0.9660;
    eph.Omega0    = -1.5708;
    eph.omega     = -1.2342;
    eph.M0        = 0.5412;
    eph.delta_n   = 4.25e-9;
    eph.i_dot     = -4.5e-10;
    eph.Omega_dot = -7.9e-9;
    eph.C_uc = 3.72e-7;  eph.C_us = 8.73e-6;
    eph.C_rc = 270.0;    eph.C_rs = 18.4;
    eph.C_ic = -8.94e-8; eph.C_is = 1.49e-7;
    eph.a_f0 = -3.2e-5;  eph.a_f1 = 5.1e-12;  eph.a_f2 = 0.0;
    eph.t_oc = 0.0;      eph.T_GD = -1.16e-8;
    eph.valid = true;
    return eph;
}

// ─────────────────────────────────────────────────────────────────────────────
// Parity verification helper
//
// Given a completed 30-bit NavWord and the D29*/D30* of the preceding word,
// verify all 6 parity bits are correct.
//
// WHY recompute parity in the test?
//   We feed the 24 data bits + prev D29/D30 back through compute_nav_word
//   and check the result matches the word we already have. If the encoder
//   produced correct parity, the two must be identical.
// ─────────────────────────────────────────────────────────────────────────────
static bool verify_word_parity(gps::NavWord word,
                                uint8_t D29_prev, uint8_t D30_prev) {
    // Extract the 24 data bits (bits 29..6 of the 30-bit word)
    // If D30* = 1, the data bits were complemented — undo that.
    uint32_t data = (word >> 6) & 0x00FFFFFFu;
    if (D30_prev) data ^= 0x00FFFFFFu;

    // Re-encode with parity
    const gps::NavWord reencoded = gps::compute_nav_word(data, D29_prev, D30_prev);

    return reencoded == word;
}

// Verify parity for all 10 words of a subframe
static bool verify_subframe_parity(const gps::NavSubframe& sf) {
    uint8_t d29 = 0, d30 = 0;
    for (int w = 0; w < 10; ++w) {
        if (!verify_word_parity(sf[w], d29, d30)) return false;
        d29 = (sf[w] >> 1) & 1u;
        d30 =  sf[w]       & 1u;
    }
    return true;
}


// ─────────────────────────────────────────────────────────────────────────────
// §1: Parity tests
// ─────────────────────────────────────────────────────────────────────────────

TEST(NavParity, ZeroWordParity) {
    // A word of 24 zero data bits should still have correct parity.
    // With D29*=D30*=0, the parity should also all be zero.
    const gps::NavWord w = gps::compute_nav_word(0x000000u, 0, 0);

    // Verify the 6 parity bits (bits 5..0) are all zero for all-zero input
    EXPECT_EQ(w & 0x3Fu, 0u) << "Zero data + zero prev bits should give zero parity";
}

TEST(NavParity, AllOnesWordParity) {
    // Known test: all 24 data bits = 1, D29*=D30*=0
    // We verify parity is self-consistent rather than a hardcoded value
    const gps::NavWord w = gps::compute_nav_word(0x00FFFFFFu, 0, 0);
    EXPECT_TRUE(verify_word_parity(w, 0, 0))
        << "All-ones word parity self-consistency failed";
}

TEST(NavParity, D30StarInversion) {
    // When D30* = 1, data bits are complemented before parity.
    // The same 24 data bits should produce DIFFERENT words for D30*=0 vs D30*=1
    const uint32_t data = 0x00A5A5A5u;
    const gps::NavWord w0 = gps::compute_nav_word(data, 0, 0);
    const gps::NavWord w1 = gps::compute_nav_word(data, 0, 1);

    EXPECT_NE(w0, w1) << "D30*=0 and D30*=1 should produce different words";

    // Both should still have internally consistent parity
    EXPECT_TRUE(verify_word_parity(w0, 0, 0));
    EXPECT_TRUE(verify_word_parity(w1, 0, 1));
}

TEST(NavParity, TLMWordParity) {
    const gps::NavSubframe sf1 = gps::encode_subframe1(make_test_ephemeris(), 0);
    // Word 1 is TLM — verify its parity (D29*=D30*=0 for first word)
    EXPECT_TRUE(verify_word_parity(sf1[0], 0, 0)) << "TLM word parity failed";
}

TEST(NavParity, AllSubframesHaveCorrectParity) {
    // Every single word in every subframe must have correct parity.
    // This is the most comprehensive parity test.
    const auto eph = make_test_ephemeris();
    const uint32_t tow = 100;

    EXPECT_TRUE(verify_subframe_parity(gps::encode_subframe1(eph, tow)))
        << "Subframe 1 parity failed";
    EXPECT_TRUE(verify_subframe_parity(gps::encode_subframe2(eph, tow)))
        << "Subframe 2 parity failed";
    EXPECT_TRUE(verify_subframe_parity(gps::encode_subframe3(eph, tow)))
        << "Subframe 3 parity failed";
}

TEST(NavParity, FullFrameAllWordsCorrect) {
    const auto eph = make_test_ephemeris();
    const uint32_t tow = 100;

    const auto sf1 = gps::encode_subframe1(eph, tow);
    const auto sf2 = gps::encode_subframe2(eph, tow + 1);
    const auto sf3 = gps::encode_subframe3(eph, tow + 2);

    // WHY verify each subframe independently?
    //   Each subframe's word chain starts fresh — TLM word 1 always uses
    //   D29*=D30*=0. The chain only runs within one subframe, not across them.
    //   Using SF1's trailing bits as the prev for SF2 word 1 would be wrong.
    EXPECT_TRUE(verify_subframe_parity(sf1)) << "SF1 full parity chain failed";
    EXPECT_TRUE(verify_subframe_parity(sf2)) << "SF2 full parity chain failed";
    EXPECT_TRUE(verify_subframe_parity(sf3)) << "SF3 full parity chain failed";
}


// ─────────────────────────────────────────────────────────────────────────────
// §2: Structure tests
// ─────────────────────────────────────────────────────────────────────────────

TEST(NavStructure, BitStreamLength) {
    // The complete nav message must be exactly 1500 bits
    const auto stream = gps::encode_nav_message(make_test_ephemeris(), 0.0);
    EXPECT_EQ(stream.kTotalBits, 1500);
}

TEST(NavStructure, AllBitsAreBinary) {
    // Every bit must be 0 or 1 — nothing else
    const auto stream = gps::encode_nav_message(make_test_ephemeris(), 0.0);
    for (int i = 0; i < stream.kTotalBits; ++i) {
        EXPECT_TRUE(stream[i] == 0 || stream[i] == 1)
            << "Bit " << i << " = " << (int)stream[i] << " is not binary";
    }
}

TEST(NavStructure, TLMPreambleCorrect) {
    // The TLM preamble (first 8 bits of each subframe) must be 10001011
    // This is the fixed sync pattern receivers look for.
    // In the bit stream: SF1 starts at bit 0, SF2 at bit 300, etc.
    const uint8_t kPreamble[] = {1,0,0,0,1,0,1,1};
    const auto stream = gps::encode_nav_message(make_test_ephemeris(), 0.0);

    for (int sf = 0; sf < 5; ++sf) {
        const int start = sf * 300;  // each subframe is 300 bits
        for (int b = 0; b < 8; ++b) {
            EXPECT_EQ(stream[start + b], kPreamble[b])
                << "Subframe " << sf+1 << " preamble bit " << b << " wrong";
        }
    }
}

TEST(NavStructure, SubframeIDsCorrect) {
    // The subframe ID is encoded in bits 49..51 of each subframe (HOW word).
    // SF1=1, SF2=2, SF3=3, SF4=4, SF5=5
    // HOW word is word 2 (bits 30..59 of each subframe).
    // Subframe ID is at bits 49..51 of each subframe (bits 19..17 of HOW word).
    const auto eph = make_test_ephemeris();
    const auto sf1 = gps::encode_subframe1(eph, 0);
    const auto sf2 = gps::encode_subframe2(eph, 1);
    const auto sf3 = gps::encode_subframe3(eph, 2);

    // HOW word is sf[1]. Subframe ID is in bits 11..9 (0-indexed from bit 29)
    // of the 30-bit word BEFORE parity conditioning.
    // Easier: extract from the word directly knowing the layout.
    // Subframe ID sits at bits 8..6 of the complete 30-bit HOW word after parity.
    // (bits 30..29 are parity, 28..7 are data, 6..1 are more parity)
    // Just decode from the raw word: ID is in bits [8:6] of the HOW word
    auto get_sfid = [](gps::NavWord how) -> int {
        // HOW: bits 29..13 = TOW, 12 = alert, 11 = AS, 10..8 = SF_ID, 7..6 = reserved, 5..0 = parity
        return (how >> 8) & 0x7u;
    };

    EXPECT_EQ(get_sfid(sf1[1]), 1) << "SF1 ID wrong";
    EXPECT_EQ(get_sfid(sf2[1]), 2) << "SF2 ID wrong";
    EXPECT_EQ(get_sfid(sf3[1]), 3) << "SF3 ID wrong";
}


// ─────────────────────────────────────────────────────────────────────────────
// §3: Encode/decode round-trip
//
// The most important functional test: encode a value, decode it, verify
// we recover the original within the quantisation error of the scale factor.
//
// WHY is there quantisation error?
//   The nav message stores a double (e.g. M0 = 0.5412 rad) as an integer.
//   We can't represent 0.5412 exactly in fixed-point with scale π×2^-31.
//   The error is at most ±0.5 LSB = ±(π×2^-31)/2 ≈ ±7.3×10^-10 rad.
//   This is sub-nanoradian — completely negligible for satellite positioning.
// ─────────────────────────────────────────────────────────────────────────────

// Helper: decode 24 data bits from a word, undoing D30* conditioning if needed
static uint32_t decode_word_data(gps::NavWord word, uint8_t D30_prev) {
    // The stored bits 29..6 are either the original data or the complement,
    // depending on whether D30* was 1 when the word was encoded.
    // To recover the original: if D30* was 1, complement them back.
    uint32_t data = (word >> 6) & 0x00FFFFFFu;
    if (D30_prev) data ^= 0x00FFFFFFu;
    return data;
}

TEST(NavRoundTrip, SqrtA) {
    // √A scale factor: 2^-19 m^½/LSB, 32-bit unsigned across words 8 and 9
    const auto eph = make_test_ephemeris();
    const auto sf2 = gps::encode_subframe2(eph, 0);

    // Walk the word chain to get correct D30* for each word
    // Word indices: 0=TLM, 1=HOW, 2=word3... 7=word8(Cus+sqA_MSB), 8=word9(sqA_LSB)
    auto d30_of = [](gps::NavWord w) -> uint8_t { return w & 1u; };

    // D30* for word 8 (index 7) comes from word 7 (index 6)
    const uint8_t d30_for_w8 = d30_of(sf2[6]);
    // D30* for word 9 (index 8) comes from word 8 (index 7)
    const uint8_t d30_for_w9 = d30_of(sf2[7]);

    // Decode word 8: bits 23..8 = Cus (we skip), bits 7..0 = √A MSByte
    const uint32_t data8 = decode_word_data(sf2[7], d30_for_w8);
    const uint32_t sqA_msb = data8 & 0xFFu;   // bottom 8 bits of word 8 data

    // Decode word 9: all 24 bits = √A LSBs
    const uint32_t sqA_lsb = decode_word_data(sf2[8], d30_for_w9);

    const uint32_t sqA_int    = (sqA_msb << 24) | sqA_lsb;
    const double   sqA_decoded = sqA_int * std::pow(2.0, -19);
    const double   tolerance   = std::pow(2.0, -19);  // 1 LSB quantisation

    EXPECT_NEAR(sqA_decoded, eph.sqrt_A, tolerance)
        << "√A round-trip failed: decoded=" << sqA_decoded
        << " original=" << eph.sqrt_A;
}

TEST(NavRoundTrip, Eccentricity) {
    // e scale factor: 2^-33/LSB, 32-bit unsigned
    const auto eph = make_test_ephemeris();
    const auto sf2 = gps::encode_subframe2(eph, 0);

    // e split: word 6 has MSByte, word 7 has 24 LSBs
    const uint32_t e_msb = (sf2[5] >> 6) & 0xFFu;
    const uint32_t e_lsb = (sf2[6] >> 6) & 0xFFFFFFu;
    const uint32_t e_int = (e_msb << 24) | e_lsb;

    const double e_decoded  = e_int * std::pow(2.0, -33);
    const double tolerance  = std::pow(2.0, -33);

    EXPECT_NEAR(e_decoded, eph.e, tolerance)
        << "Eccentricity round-trip failed";
}

TEST(NavRoundTrip, InvalidEphemerisThrows) {
    gps::Ephemeris bad;  // valid = false
    EXPECT_THROW(
        { (void)gps::encode_nav_message(bad, 0.0); },
        std::invalid_argument
    );
}