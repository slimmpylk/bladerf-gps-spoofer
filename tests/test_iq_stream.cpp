// ─────────────────────────────────────────────────────────────────────────────
// test_iq_stream.cpp — Unit tests for the I/Q stream combiner
// ─────────────────────────────────────────────────────────────────────────────

#include "iq_stream.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <complex>

using namespace std::complex_literals;  // enables 1.0if syntax

// ─────────────────────────────────────────────────────────────────────────────
// §1: sum_channels()
// ─────────────────────────────────────────────────────────────────────────────

TEST(IQStream, SumTwoChannels) {
    // Two channels of 4 samples each — verify element-wise sum
    std::vector<std::complex<float>> ch1 = {{1,0},{2,0},{3,0},{4,0}};
    std::vector<std::complex<float>> ch2 = {{1,0},{1,0},{1,0},{1,0}};

    const auto result = gps::sum_channels({ch1, ch2});

    ASSERT_EQ(result.size(), 4u);
    EXPECT_NEAR(result[0].real(), 2.0f, 1e-5f);
    EXPECT_NEAR(result[1].real(), 3.0f, 1e-5f);
    EXPECT_NEAR(result[2].real(), 4.0f, 1e-5f);
    EXPECT_NEAR(result[3].real(), 5.0f, 1e-5f);
}

TEST(IQStream, SumEmptyReturnsEmpty) {
    const auto result = gps::sum_channels({});
    EXPECT_TRUE(result.empty());
}

TEST(IQStream, SumMismatchedSizesThrows) {
    std::vector<std::complex<float>> ch1(100, {1,0});
    std::vector<std::complex<float>> ch2(99,  {1,0});  // wrong size
    EXPECT_THROW(gps::sum_channels({ch1, ch2}), std::invalid_argument);
}

TEST(IQStream, SumSingleChannel) {
    // Sum of one channel should equal that channel unchanged
    std::vector<std::complex<float>> ch = {{1,2},{3,4},{5,6}};
    const auto result = gps::sum_channels({ch});
    ASSERT_EQ(result.size(), ch.size());
    for (size_t i = 0; i < ch.size(); ++i) {
        EXPECT_NEAR(result[i].real(), ch[i].real(), 1e-5f);
        EXPECT_NEAR(result[i].imag(), ch[i].imag(), 1e-5f);
    }
}

TEST(IQStream, SumEightChannelsEqualAmplitude) {
    // 8 channels of constant 1.0 — sum should be 8.0
    const int N = 2048;
    std::vector<std::vector<std::complex<float>>> channels(8,
        std::vector<std::complex<float>>(N, {1.0f, 0.0f}));

    const auto result = gps::sum_channels(channels);

    ASSERT_EQ(static_cast<int>(result.size()), N);
    EXPECT_NEAR(result[0].real(), 8.0f, 1e-4f);
    EXPECT_NEAR(result[0].imag(), 0.0f, 1e-4f);
}

// ─────────────────────────────────────────────────────────────────────────────
// §2: find_peak_magnitude()
// ─────────────────────────────────────────────────────────────────────────────

TEST(IQStream, PeakAllZeroReturnsOne) {
    // All-zero samples: peak should be 1.0 (floor to prevent div-by-zero)
    std::vector<std::complex<float>> samples(100, {0,0});
    EXPECT_NEAR(gps::find_peak_magnitude(samples), 1.0f, 1e-5f);
}

TEST(IQStream, PeakFindsMaxMagnitude) {
    // find_peak_magnitude() floors at 1.0 to prevent division by zero.
    // When all samples have magnitude < 1.0, it returns 1.0.
    std::vector<std::complex<float>> samples_below_one = {
        {0.5f, 0.0f}, {0.0f, 0.8f}, {0.6f, 0.6f}, {0.1f, 0.1f}
    };
    EXPECT_NEAR(gps::find_peak_magnitude(samples_below_one), 1.0f, 1e-5f)
        << "Peaks below 1.0 should return 1.0 (floor prevents div-by-zero)";

    // When peak is above 1.0, the actual maximum magnitude is returned
    std::vector<std::complex<float>> samples_above_one = {
        {1.5f, 0.0f},   // magnitude = 1.5  ← max
        {0.0f, 1.2f},
        {1.0f, 1.0f}    // magnitude = √2 ≈ 1.414
    };
    EXPECT_NEAR(gps::find_peak_magnitude(samples_above_one), 1.5f, 1e-4f);
}

// ─────────────────────────────────────────────────────────────────────────────
// §3: float_to_sc16q11()
// ─────────────────────────────────────────────────────────────────────────────

TEST(IQStream, SC16Q11ZeroInput) {
    // Zero float → zero integer
    std::vector<std::complex<float>> samples(4, {0.0f, 0.0f});
    const auto buf = gps::float_to_sc16q11(samples, 2047.0f);
    for (const auto& s : buf) {
        EXPECT_EQ(s.i, 0);
        EXPECT_EQ(s.q, 0);
    }
}

TEST(IQStream, SC16Q11FullScalePositive) {
    // Input = 1.0, scale = 2047 → output should be 2047
    std::vector<std::complex<float>> samples = {{1.0f, 1.0f}};
    const auto buf = gps::float_to_sc16q11(samples, 2047.0f);
    EXPECT_EQ(buf[0].i, 2047);
    EXPECT_EQ(buf[0].q, 2047);
}

TEST(IQStream, SC16Q11FullScaleNegative) {
    // Input = -1.0, scale = 2047 → output should be -2047
    std::vector<std::complex<float>> samples = {{-1.0f, -1.0f}};
    const auto buf = gps::float_to_sc16q11(samples, 2047.0f);
    EXPECT_EQ(buf[0].i, -2047);
    EXPECT_EQ(buf[0].q, -2047);
}

TEST(IQStream, SC16Q11ClipsAtBoundary) {
    // +2.0 × 2047 = +4094 → clamped to  2047
    // -2.0 × 2047 = -4094 → clamped to -2048
    std::vector<std::complex<float>> samples = {{2.0f, -2.0f}};
    const auto buf = gps::float_to_sc16q11(samples, 2047.0f);
    EXPECT_EQ(buf[0].i,  2047) << "Positive overflow must clamp to 2047";
    EXPECT_EQ(buf[0].q, -2048) << "Negative overflow must clamp to -2048";
}

TEST(IQStream, SC16Q11NegativeClampCorrect) {
    // -2.0 * 2047 = -4094 → clamped to -2048
    std::vector<std::complex<float>> samples = {{0.0f, -2.0f}};
    const auto buf = gps::float_to_sc16q11(samples, 2047.0f);
    EXPECT_EQ(buf[0].q, -2048) << "Large negative should clamp to -2048";
}

TEST(IQStream, SC16Q11HeadroomScaling) {
    // With 60% headroom: peak 1.0 → output = 0.6 × 2047 ≈ 1228
    const float headroom    = 0.6f;
    const float scale       = headroom * 2047.0f;
    std::vector<std::complex<float>> samples = {{1.0f, 0.0f}};
    const auto buf = gps::float_to_sc16q11(samples, scale);
    EXPECT_NEAR(buf[0].i, static_cast<int16_t>(std::round(scale)), 1)
        << "60% headroom scaling incorrect";
}

// ─────────────────────────────────────────────────────────────────────────────
// §4: IQStream class
// ─────────────────────────────────────────────────────────────────────────────

TEST(IQStream, ClassBasicEpoch) {
    // Add 2 channels, finalise → should get 2048 SC16Q11 samples
    gps::IQStream stream(2);

    // Two simple channels: all +0.5
    const std::vector<std::complex<float>> ch(2048, {0.5f, 0.0f});
    stream.add_channel_epoch(ch);
    stream.add_channel_epoch(ch);

    const auto buf = stream.finalise();

    ASSERT_EQ(static_cast<int>(buf.size()), 2048);

    // Sum of two 0.5 channels = 1.0. With 60% headroom:
    // scale = 0.6 × 2047 / 1.0 ≈ 1228. All I samples should be ~1228.
    EXPECT_GT(buf[0].i, 1000) << "I samples unexpectedly small";
    EXPECT_LT(buf[0].i, 2048) << "I samples exceeded SC16Q11 range";
    EXPECT_EQ(buf[0].q, 0)    << "Q should be zero for all-real input";
}

TEST(IQStream, ClassResetClearsAccumulator) {
    gps::IQStream stream(1);
    const std::vector<std::complex<float>> ch(2048, {1.0f, 0.0f});
    stream.add_channel_epoch(ch);

    // Reset and re-add a zero channel
    stream.reset();
    const std::vector<std::complex<float>> zero(2048, {0.0f, 0.0f});
    stream.add_channel_epoch(zero);

    const auto buf = stream.finalise();

    // After reset, sum should be 0 → all output samples should be 0
    for (const auto& s : buf) {
        EXPECT_EQ(s.i, 0) << "After reset, I should be zero";
        EXPECT_EQ(s.q, 0) << "After reset, Q should be zero";
    }
}

TEST(IQStream, ClassWrongSizeThrows) {
    gps::IQStream stream(2);
    const std::vector<std::complex<float>> wrong_size(100, {1.0f, 0.0f}); // not 2048
    EXPECT_THROW(stream.add_channel_epoch(wrong_size), std::invalid_argument);
}

TEST(IQStream, ClassOutputStaysInRange) {
    // Even with 8 channels summed, output must stay within SC16Q11 limits
    gps::IQStream stream(8);
    const std::vector<std::complex<float>> ch(2048, {1.0f, 1.0f});
    for (int i = 0; i < 8; ++i) stream.add_channel_epoch(ch);

    const auto buf = stream.finalise();

    for (const auto& s : buf) {
        EXPECT_GE(s.i, -2048) << "I underflow";
        EXPECT_LE(s.i,  2047) << "I overflow";
        EXPECT_GE(s.q, -2048) << "Q underflow";
        EXPECT_LE(s.q,  2047) << "Q overflow";
    }
}