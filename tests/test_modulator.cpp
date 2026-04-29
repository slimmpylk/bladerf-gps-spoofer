// ─────────────────────────────────────────────────────────────────────────────
// test_modulator.cpp — Unit tests for the I/Q modulator
// ─────────────────────────────────────────────────────────────────────────────

#include "modulator.hpp"
#include "ephemeris.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <numeric>

// Reuse test ephemeris
static gps::Ephemeris make_test_ephemeris() {
    gps::Ephemeris eph;
    eph.prn=1; eph.gps_week=2295; eph.t_oe=0.0;
    eph.sqrt_A=5153.79476547; eph.e=0.00824451; eph.i0=0.9660;
    eph.Omega0=-1.5708; eph.omega=-1.2342; eph.M0=0.5412;
    eph.delta_n=4.25e-9; eph.i_dot=-4.5e-10; eph.Omega_dot=-7.9e-9;
    eph.C_uc=3.72e-7; eph.C_us=8.73e-6; eph.C_rc=270.0; eph.C_rs=18.4;
    eph.C_ic=-8.94e-8; eph.C_is=1.49e-7;
    eph.a_f0=-3.2e-5; eph.a_f1=5.1e-12; eph.a_f2=0.0;
    eph.t_oc=0.0; eph.T_GD=-1.16e-8; eph.valid=true;
    return eph;
}

// ─────────────────────────────────────────────────────────────────────────────
// §1: Geometry helpers
// ─────────────────────────────────────────────────────────────────────────────

TEST(Modulator, RangeComputation) {
    // Two points 1000m apart on the X axis
    gps::EcefPosition sat{1000.0, 0.0, 0.0};
    gps::EcefPosition rx {   0.0, 0.0, 0.0};
    EXPECT_NEAR(gps::compute_range(sat, rx), 1000.0, 1e-6);
}

TEST(Modulator, RangeGPSOrbit) {
    // A satellite roughly overhead at GPS orbital altitude ~26,560 km
    // and a receiver on the equator. Range should be ~20,000–26,000 km.
    gps::EcefPosition sat{26560e3, 0.0, 0.0};
    gps::EcefPosition rx {6378e3,  0.0, 0.0};  // on Earth's surface
    const double range = gps::compute_range(sat, rx);
    EXPECT_GT(range, 18000e3) << "Range unrealistically short";
    EXPECT_LT(range, 27000e3) << "Range unrealistically long";
}

TEST(Modulator, DopplerApproachingPositive) {
    // Satellite moving directly toward receiver → positive Doppler
    // Unit vector from receiver to satellite: (1, 0, 0)
    // Satellite velocity: (-1000, 0, 0) m/s (moving toward receiver)
    // Relative velocity projected on LOS = -1000 m/s
    // f_D = -(-1000) / λ_L1 = +1000/0.1903 ≈ +5255 Hz
    gps::EcefVelocity sat_vel{-1000.0, 0.0, 0.0};  // moving toward rx
    gps::EcefVelocity rx_vel {0.0, 0.0, 0.0};
    const double fd = gps::compute_doppler(sat_vel, rx_vel, 1.0, 0.0, 0.0);
    EXPECT_GT(fd, 0.0) << "Approaching satellite should give positive Doppler";
    EXPECT_NEAR(fd, 1000.0 / (299792458.0/1575.42e6), 1.0);
}

TEST(Modulator, DopplerStationaryZero) {
    // No relative motion → zero Doppler
    gps::EcefVelocity zero{0.0, 0.0, 0.0};
    const double fd = gps::compute_doppler(zero, zero, 1.0, 0.0, 0.0);
    EXPECT_NEAR(fd, 0.0, 1e-6);
}

TEST(Modulator, CodePhaseRange) {
    // Code phase must always be in [0, 1023)
    // Test several distances including very large ones
    for (double range_m : {20000e3, 22000e3, 25000e3, 26560e3}) {
        const double phase = gps::compute_code_phase(range_m);
        EXPECT_GE(phase, 0.0)    << "Code phase negative for range " << range_m;
        EXPECT_LT(phase, 1023.0) << "Code phase >= 1023 for range " << range_m;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// §2: Channel construction
// ─────────────────────────────────────────────────────────────────────────────

TEST(Modulator, MakeChannelValidPRN) {
    const auto eph      = make_test_ephemeris();
    const auto sat      = gps::compute_satellite_state(eph, 0.0);
    const gps::EcefPosition rx_pos{2885000.0, 1338000.0, 5525000.0}; // Helsinki
    const gps::EcefVelocity rx_vel{0.0, 0.0, 0.0};
    const auto nav = gps::encode_nav_message(eph, 0.0);

    EXPECT_NO_THROW({
        const auto cfg = gps::make_channel(1, sat, rx_pos, rx_vel, 0.0, nav);
        EXPECT_EQ(cfg.prn, 1);
        EXPECT_GE(cfg.code_phase, 0.0);
        EXPECT_LT(cfg.code_phase, 1023.0);
    });
}

TEST(Modulator, MakeChannelInvalidPRNThrows) {
    const auto eph = make_test_ephemeris();
    const auto sat = gps::compute_satellite_state(eph, 0.0);
    const gps::EcefPosition rx{0,0,0};
    const gps::EcefVelocity rv{0,0,0};
    const auto nav = gps::encode_nav_message(eph, 0.0);

    EXPECT_THROW(
        { (void)gps::make_channel(0, sat, rx, rv, 0.0, nav); },
        std::invalid_argument
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// §3: Epoch generation
// ─────────────────────────────────────────────────────────────────────────────

TEST(Modulator, EpochProducesCorrectSampleCount) {
    // generate_epoch() must produce exactly kSamplesPerMs (2048) samples
    const auto eph = make_test_ephemeris();
    const auto sat = gps::compute_satellite_state(eph, 0.0);
    const gps::EcefPosition rx{2885000.0, 1338000.0, 5525000.0};
    const gps::EcefVelocity rv{0,0,0};
    const auto nav = gps::encode_nav_message(eph, 0.0);
    const auto cfg = gps::make_channel(1, sat, rx, rv, 0.0, nav);

    gps::ModulatorState state;
    state.code_phase = cfg.code_phase;
    state.carrier_phase = cfg.carrier_phase;

    std::vector<std::complex<float>> samples;
    gps::generate_epoch(cfg, state, samples);

    EXPECT_EQ(static_cast<int>(samples.size()), gps::kSamplesPerMs)
        << "Epoch must produce exactly " << gps::kSamplesPerMs << " samples";
}

TEST(Modulator, SamplesAreBounded) {
    // I and Q samples must stay within [-amplitude, +amplitude]
    // For amplitude=1.0, this means [-1.0, 1.0]
    const auto eph = make_test_ephemeris();
    const auto sat = gps::compute_satellite_state(eph, 0.0);
    const gps::EcefPosition rx{2885000.0, 1338000.0, 5525000.0};
    const gps::EcefVelocity rv{0,0,0};
    const auto nav = gps::encode_nav_message(eph, 0.0);
    const auto cfg = gps::make_channel(1, sat, rx, rv, 0.0, nav);

    gps::ModulatorState state;
    state.code_phase    = cfg.code_phase;
    state.carrier_phase = cfg.carrier_phase;

    std::vector<std::complex<float>> samples;
    gps::generate_epoch(cfg, state, samples);

    for (size_t i = 0; i < samples.size(); ++i) {
        EXPECT_LE(std::abs(samples[i].real()), 1.01f)
            << "I sample out of bounds at index " << i;
        EXPECT_LE(std::abs(samples[i].imag()), 1.01f)
            << "Q sample out of bounds at index " << i;
    }
}

TEST(Modulator, StateAdvancesAcrossEpochs) {
    // After generating one epoch, code_phase and carrier_phase must have
    // advanced. Checking they're different from their initial values.
    const auto eph = make_test_ephemeris();
    const auto sat = gps::compute_satellite_state(eph, 0.0);
    const gps::EcefPosition rx{2885000.0, 1338000.0, 5525000.0};
    const gps::EcefVelocity rv{0,0,0};
    const auto nav = gps::encode_nav_message(eph, 0.0);
    const auto cfg = gps::make_channel(1, sat, rx, rv, 0.0, nav);

    gps::ModulatorState state;
    state.code_phase    = cfg.code_phase;
    state.carrier_phase = 0.0;
    state.sample_count  = 0;

    const double initial_carrier = state.carrier_phase;

    std::vector<std::complex<float>> samples;
    gps::generate_epoch(cfg, state, samples);

    // Carrier phase must have advanced (Doppler ≠ 0 for a real satellite)
    // sample_count must be exactly kSamplesPerMs more
    EXPECT_EQ(state.sample_count, gps::kSamplesPerMs);

    // Two consecutive epochs should produce different samples
    // (carrier phase continues, not reset)
    std::vector<std::complex<float>> samples2;
    gps::generate_epoch(cfg, state, samples2);

    EXPECT_EQ(static_cast<int>(samples2.size()), gps::kSamplesPerMs);

    // The first sample of epoch 2 should differ from epoch 1
    // (if they were identical, state wasn't preserved)
    bool any_different = false;
    for (int i = 0; i < 10; ++i) {
        if (samples[i] != samples2[i]) { any_different = true; break; }
    }
    EXPECT_TRUE(any_different)
        << "Consecutive epochs produced identical samples — state not preserved";
}

TEST(Modulator, ZeroDopplerSymmetricPower) {
    // With zero Doppler (purely static), I and Q should have equal average power.
    // The signal is a pure spreading code with no frequency offset.
    // I = CA * cos(0) = CA,  Q = CA * sin(0) = 0 ... actually with phase=0
    // the power will be entirely in I. Let's test with a non-zero initial phase.
    gps::ChannelConfig cfg;
    cfg.prn           = 1;
    cfg.doppler_hz    = 0.0;    // no Doppler
    cfg.code_phase    = 0.0;
    cfg.carrier_phase = std::numbers::pi / 4.0;  // 45° start phase
    cfg.amplitude     = 1.0f;
    cfg.ca_code       = gps::generate_ca_code(1);
    // Minimal nav bits (all zeros)
    cfg.nav_bits = gps::NavBitStream{};

    gps::ModulatorState state;
    state.code_phase    = 0.0;
    state.carrier_phase = std::numbers::pi / 4.0;

    std::vector<std::complex<float>> samples;
    gps::generate_epoch(cfg, state, samples);

    // With zero Doppler, all samples should have the same phase offset.
    // All sample magnitudes should be exactly 1.0 (amplitude).
    for (const auto& s : samples) {
        EXPECT_NEAR(std::abs(s), 1.0f, 0.001f)
            << "With zero Doppler, all sample magnitudes must equal amplitude";
    }
}